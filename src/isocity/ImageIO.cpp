#include "isocity/Export.hpp"

#include "isocity/Checksum.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

inline std::string LowerExt(const std::string& path)
{
  const std::size_t slash = path.find_last_of("/\\");
  const std::size_t dot = path.find_last_of('.');
  if (dot == std::string::npos) return {};
  if (slash != std::string::npos && dot < slash) return {};
  return ToLower(path.substr(dot));
}

inline bool ReadExact(std::istream& is, void* dst, std::size_t n)
{
  is.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
  return static_cast<bool>(is);
}

inline bool WriteExact(std::ostream& os, const void* src, std::size_t n)
{
  os.write(reinterpret_cast<const char*>(src), static_cast<std::streamsize>(n));
  return static_cast<bool>(os);
}

inline std::uint32_t ReadU32BE(std::istream& is, bool& ok)
{
  std::uint8_t b[4] = {0, 0, 0, 0};
  ok = ReadExact(is, b, 4);
  if (!ok) return 0;
  return (static_cast<std::uint32_t>(b[0]) << 24) | (static_cast<std::uint32_t>(b[1]) << 16) |
         (static_cast<std::uint32_t>(b[2]) << 8) | static_cast<std::uint32_t>(b[3]);
}

inline bool WriteU32BE(std::ostream& os, std::uint32_t v)
{
  const std::uint8_t b[4] = {
      static_cast<std::uint8_t>((v >> 24) & 0xFFu),
      static_cast<std::uint8_t>((v >> 16) & 0xFFu),
      static_cast<std::uint8_t>((v >> 8) & 0xFFu),
      static_cast<std::uint8_t>(v & 0xFFu),
  };
  return WriteExact(os, b, 4);
}

inline std::uint32_t CrcPngChunk(const char type[4], const std::uint8_t* data, std::size_t size)
{
  std::uint32_t crc = 0xFFFFFFFFu;
  crc = Crc32Update(crc, reinterpret_cast<const std::uint8_t*>(type), 4);
  if (data && size > 0) crc = Crc32Update(crc, data, size);
  return crc ^ 0xFFFFFFFFu;
}

bool WritePngChunk(std::ostream& os, const char type[4], const std::uint8_t* data, std::size_t size, std::string& outError)
{
  outError.clear();
  if (size > 0xFFFFFFFFu) {
    outError = "PNG chunk too large";
    return false;
  }

  if (!WriteU32BE(os, static_cast<std::uint32_t>(size))) {
    outError = "PNG write failed (chunk length)";
    return false;
  }
  if (!WriteExact(os, type, 4)) {
    outError = "PNG write failed (chunk type)";
    return false;
  }
  if (size > 0 && data) {
    if (!WriteExact(os, data, size)) {
      outError = "PNG write failed (chunk data)";
      return false;
    }
  }

  const std::uint32_t crc = CrcPngChunk(type, data, size);
  if (!WriteU32BE(os, crc)) {
    outError = "PNG write failed (chunk crc)";
    return false;
  }

  return true;
}

bool HasPngSignature(const std::uint8_t* b, std::size_t n)
{
  static constexpr std::uint8_t kSig[8] = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
  if (!b || n < 8) return false;
  for (int i = 0; i < 8; ++i) {
    if (b[i] != kSig[i]) return false;
  }
  return true;
}


struct BitReader {
  const std::vector<std::uint8_t>& in;
  std::size_t pos = 0;
  std::uint64_t bitbuf = 0;
  int bitcount = 0;

  explicit BitReader(const std::vector<std::uint8_t>& src, std::size_t startPos = 0)
      : in(src), pos(startPos), bitbuf(0), bitcount(0)
  {
  }

  bool ensure(int n, std::string& outError)
  {
    while (bitcount < n) {
      if (pos >= in.size()) {
        outError = "truncated bitstream";
        return false;
      }
      bitbuf |= static_cast<std::uint64_t>(in[pos++]) << bitcount;
      bitcount += 8;
    }
    return true;
  }

  bool peekBits(int n, std::uint32_t& outVal, std::string& outError)
  {
    if (n <= 0) {
      outVal = 0;
      return true;
    }
    if (n > 24) {
      outError = "internal error: peekBits too wide";
      return false;
    }
    if (!ensure(n, outError)) return false;
    outVal = static_cast<std::uint32_t>(bitbuf & ((1ull << n) - 1ull));
    return true;
  }

  bool dropBits(int n, std::string& outError)
  {
    if (n < 0) {
      outError = "internal error: negative dropBits";
      return false;
    }
    if (n == 0) return true;
    if (n > bitcount) {
      outError = "internal error: dropBits underflow";
      return false;
    }
    bitbuf >>= n;
    bitcount -= n;
    return true;
  }

  bool readBits(int n, std::uint32_t& outVal, std::string& outError)
  {
    if (n <= 0) {
      outVal = 0;
      return true;
    }
    if (n > 24) {
      outError = "internal error: readBits too wide";
      return false;
    }
    if (!ensure(n, outError)) return false;
    outVal = static_cast<std::uint32_t>(bitbuf & ((1ull << n) - 1ull));
    bitbuf >>= n;
    bitcount -= n;
    return true;
  }

  void alignToByte()
  {
    const int drop = bitcount & 7;
    bitbuf >>= drop;
    bitcount -= drop;
  }

  bool readAlignedBytes(std::vector<std::uint8_t>& dst, std::size_t n, std::string& outError)
  {
    if ((bitcount & 7) != 0) {
      outError = "internal error: bitstream not byte-aligned";
      return false;
    }

    // Consume any buffered whole bytes first.
    while (n > 0 && bitcount >= 8) {
      dst.push_back(static_cast<std::uint8_t>(bitbuf & 0xFFu));
      bitbuf >>= 8;
      bitcount -= 8;
      --n;
    }

    // Now we should be fully byte-aligned with an empty buffer.
    if (bitcount != 0) {
      outError = "internal error: bit buffer not empty on byte boundary";
      return false;
    }

    if (n > 0) {
      if (pos + n > in.size()) {
        outError = "truncated bitstream";
        return false;
      }
      dst.insert(dst.end(), in.begin() + static_cast<std::ptrdiff_t>(pos), in.begin() + static_cast<std::ptrdiff_t>(pos + n));
      pos += n;
    }

    return true;
  }
};

struct HuffmanEntry {
  std::uint16_t sym = 0;
  std::uint8_t len = 0;
};

inline std::uint32_t ReverseBits(std::uint32_t v, int bits)
{
  std::uint32_t r = 0u;
  for (int i = 0; i < bits; ++i) {
    r = (r << 1) | (v & 1u);
    v >>= 1;
  }
  return r;
}


struct HuffmanTable {
  int maxLen = 0;
  std::vector<HuffmanEntry> table;

  bool build(const std::vector<std::uint8_t>& lengths, std::string& outError)
  {
    outError.clear();
    maxLen = 0;

    std::array<int, 16> count = {};
    for (std::uint8_t l : lengths) {
      if (l == 0u) continue;
      if (l > 15u) {
        outError = "invalid Huffman code length (>15)";
        return false;
      }
      ++count[static_cast<std::size_t>(l)];
      maxLen = std::max(maxLen, static_cast<int>(l));
    }

    if (maxLen == 0) {
      outError = "empty Huffman tree";
      return false;
    }

    // Check for over-subscribed trees.
    // (Incomplete trees are tolerated; invalid codes will decode as len==0.)
    int left = 1;
    for (int len = 1; len <= maxLen; ++len) {
      left <<= 1;
      left -= count[static_cast<std::size_t>(len)];
      if (left < 0) {
        outError = "over-subscribed Huffman tree";
        return false;
      }
    }

    const std::size_t tableSize = 1u << static_cast<std::size_t>(maxLen);
    table.assign(tableSize, HuffmanEntry{});

    std::array<int, 16> nextCode = {};
    int code = 0;
    for (int len = 1; len <= maxLen; ++len) {
      code = (code + count[static_cast<std::size_t>(len - 1)]) << 1;
      nextCode[static_cast<std::size_t>(len)] = code;
    }

    for (std::size_t sym = 0; sym < lengths.size(); ++sym) {
      const int len = static_cast<int>(lengths[sym]);
      if (len == 0) continue;

      const int codeVal = nextCode[static_cast<std::size_t>(len)]++;
      if (codeVal < 0 || codeVal >= (1 << len)) {
        outError = "invalid Huffman code (out of range)";
        return false;
      }

      const std::uint32_t codeRev = ReverseBits(static_cast<std::uint32_t>(codeVal), len);
      const int fill = 1 << (maxLen - len);
      for (int i = 0; i < fill; ++i) {
        const int idx = static_cast<int>(codeRev | (static_cast<std::uint32_t>(i) << len));
        if (idx < 0 || static_cast<std::size_t>(idx) >= table.size()) continue;
        if (table[static_cast<std::size_t>(idx)].len != 0u) {
          outError = "Huffman table collision";
          return false;
        }
        table[static_cast<std::size_t>(idx)] = HuffmanEntry{static_cast<std::uint16_t>(sym), static_cast<std::uint8_t>(len)};
      }
    }

    return true;
  }

  bool decode(BitReader& br, std::uint16_t& outSym, std::string& outError) const
  {
    outError.clear();
    if (maxLen <= 0) {
      outError = "Huffman table not initialized";
      return false;
    }

    std::uint32_t bits = 0;
    if (!br.peekBits(maxLen, bits, outError)) return false;

    const HuffmanEntry e = table[static_cast<std::size_t>(bits)];
    if (e.len == 0u) {
      outError = "invalid Huffman code";
      return false;
    }

    if (!br.dropBits(static_cast<int>(e.len), outError)) return false;
    outSym = e.sym;
    return true;
  }
};

bool BuildFixedHuffmanTables(HuffmanTable& outLitLen, HuffmanTable& outDist, std::string& outError)
{
  outError.clear();

  std::vector<std::uint8_t> litLenLens(288, 0u);
  for (int i = 0; i <= 143; ++i) litLenLens[static_cast<std::size_t>(i)] = 8u;
  for (int i = 144; i <= 255; ++i) litLenLens[static_cast<std::size_t>(i)] = 9u;
  for (int i = 256; i <= 279; ++i) litLenLens[static_cast<std::size_t>(i)] = 7u;
  for (int i = 280; i <= 287; ++i) litLenLens[static_cast<std::size_t>(i)] = 8u;

  std::vector<std::uint8_t> distLens(32, 5u);

  if (!outLitLen.build(litLenLens, outError)) return false;
  if (!outDist.build(distLens, outError)) return false;
  return true;
}

bool BuildDynamicHuffmanTables(BitReader& br, HuffmanTable& outLitLen, HuffmanTable& outDist, std::string& outError)
{
  outError.clear();

  std::uint32_t v = 0;
  if (!br.readBits(5, v, outError)) return false;
  const int HLIT = static_cast<int>(v) + 257; // # lit/len codes (257..286)
  if (!br.readBits(5, v, outError)) return false;
  const int HDIST = static_cast<int>(v) + 1; // # dist codes (1..32)
  if (!br.readBits(4, v, outError)) return false;
  const int HCLEN = static_cast<int>(v) + 4; // # code-length codes (4..19)

  static constexpr int kCLOrder[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

  std::vector<std::uint8_t> clLens(19, 0u);
  for (int i = 0; i < HCLEN; ++i) {
    if (!br.readBits(3, v, outError)) return false;
    const int sym = kCLOrder[i];
    clLens[static_cast<std::size_t>(sym)] = static_cast<std::uint8_t>(v);
  }

  HuffmanTable clTable;
  if (!clTable.build(clLens, outError)) {
    outError = "failed to build code-length Huffman table: " + outError;
    return false;
  }

  const int total = HLIT + HDIST;
  std::vector<std::uint8_t> lengths;
  lengths.reserve(static_cast<std::size_t>(total));

  while (static_cast<int>(lengths.size()) < total) {
    std::uint16_t sym = 0;
    if (!clTable.decode(br, sym, outError)) {
      outError = "failed to decode code-length symbol: " + outError;
      return false;
    }

    if (sym <= 15u) {
      lengths.push_back(static_cast<std::uint8_t>(sym));
    } else if (sym == 16u) {
      if (lengths.empty()) {
        outError = "repeat code 16 with no previous length";
        return false;
      }
      if (!br.readBits(2, v, outError)) return false;
      const int repeat = static_cast<int>(v) + 3;
      const std::uint8_t prev = lengths.back();
      for (int r = 0; r < repeat; ++r) {
        if (static_cast<int>(lengths.size()) >= total) {
          outError = "repeat overruns code-length buffer";
          return false;
        }
        lengths.push_back(prev);
      }
    } else if (sym == 17u) {
      if (!br.readBits(3, v, outError)) return false;
      const int repeat = static_cast<int>(v) + 3;
      for (int r = 0; r < repeat; ++r) {
        if (static_cast<int>(lengths.size()) >= total) {
          outError = "repeat overruns code-length buffer";
          return false;
        }
        lengths.push_back(0u);
      }
    } else if (sym == 18u) {
      if (!br.readBits(7, v, outError)) return false;
      const int repeat = static_cast<int>(v) + 11;
      for (int r = 0; r < repeat; ++r) {
        if (static_cast<int>(lengths.size()) >= total) {
          outError = "repeat overruns code-length buffer";
          return false;
        }
        lengths.push_back(0u);
      }
    } else {
      outError = "invalid code-length symbol";
      return false;
    }
  }

  // Literal/length alphabet has 286 codes (0..285).
  std::vector<std::uint8_t> litLenLens(286, 0u);
  for (int i = 0; i < HLIT; ++i) {
    litLenLens[static_cast<std::size_t>(i)] = lengths[static_cast<std::size_t>(i)];
  }
  if (litLenLens[256] == 0u) {
    outError = "dynamic Huffman table missing end-of-block symbol (256)";
    return false;
  }

  // Distance alphabet has up to 32 codes (0..31), but only 0..29 are used.
  std::vector<std::uint8_t> distLens(32, 0u);
  for (int i = 0; i < HDIST; ++i) {
    distLens[static_cast<std::size_t>(i)] = lengths[static_cast<std::size_t>(HLIT + i)];
  }

  bool anyDist = false;
  for (std::uint8_t l : distLens) {
    if (l != 0u) {
      anyDist = true;
      break;
    }
  }
  if (!anyDist) {
    // Valid streams with no matches might still encode an all-zero distance tree.
    // Provide a dummy single code so we can proceed; any attempt to use it will still validate distances.
    distLens[0] = 1u;
  }

  if (!outLitLen.build(litLenLens, outError)) {
    outError = "failed to build lit/len Huffman table: " + outError;
    return false;
  }
  if (!outDist.build(distLens, outError)) {
    outError = "failed to build dist Huffman table: " + outError;
    return false;
  }

  return true;
}

bool DecompressZlib(const std::vector<std::uint8_t>& in, std::vector<std::uint8_t>& out, std::size_t maxOut, std::string& outError)
{
  outError.clear();
  out.clear();

  if (in.size() < 2 + 4) {
    outError = "zlib stream too small";
    return false;
  }

  const std::uint8_t cmf = in[0];
  const std::uint8_t flg = in[1];

  // RFC1950 header checks.
  const std::uint16_t cmfFlg = static_cast<std::uint16_t>(cmf) * 256u + static_cast<std::uint16_t>(flg);
  if ((cmfFlg % 31u) != 0u) {
    outError = "invalid zlib header (FCHECK)";
    return false;
  }
  if ((cmf & 0x0Fu) != 8u) {
    outError = "unsupported zlib compression method (expected DEFLATE)";
    return false;
  }
  if ((flg & 0x20u) != 0u) {
    outError = "unsupported zlib preset dictionary";
    return false;
  }

  BitReader br(in, 2);

  // Pre-build fixed Huffman tables once.
  static bool s_fixedReady = false;
  static HuffmanTable s_fixedLitLen;
  static HuffmanTable s_fixedDist;
  if (!s_fixedReady) {
    std::string herr;
    if (!BuildFixedHuffmanTables(s_fixedLitLen, s_fixedDist, herr)) {
      outError = "internal error: failed to build fixed Huffman tables: " + herr;
      return false;
    }
    s_fixedReady = true;
  }

  static constexpr int kLenBase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27,
                                       31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
  static constexpr int kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                        2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
  static constexpr int kDistBase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
                                        193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097,
                                        6145, 8193, 12289, 16385, 24577};
  static constexpr int kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6,
                                         6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

  auto CheckOutCap = [&](std::size_t add) -> bool {
    if (maxOut > 0 && out.size() + add > maxOut) {
      outError = "decompressed data exceeds expected size";
      return false;
    }
    return true;
  };

  while (true) {
    std::uint32_t bfinal = 0;
    std::uint32_t btype = 0;
    if (!br.readBits(1, bfinal, outError)) return false;
    if (!br.readBits(2, btype, outError)) return false;

    if (btype == 0u) {
      // Stored block: align to byte boundary, then LEN/NLEN + raw bytes.
      br.alignToByte();

      std::uint32_t len = 0;
      std::uint32_t nlen = 0;
      if (!br.readBits(16, len, outError)) return false;
      if (!br.readBits(16, nlen, outError)) return false;
      if (((len ^ 0xFFFFu) & 0xFFFFu) != (nlen & 0xFFFFu)) {
        outError = "stored block LEN/NLEN mismatch";
        return false;
      }

      if (!CheckOutCap(static_cast<std::size_t>(len))) return false;
      if (!br.readAlignedBytes(out, static_cast<std::size_t>(len), outError)) return false;
    } else if (btype == 1u || btype == 2u) {
      HuffmanTable litLen;
      HuffmanTable dist;

      if (btype == 1u) {
        litLen = s_fixedLitLen;
        dist = s_fixedDist;
      } else {
        if (!BuildDynamicHuffmanTables(br, litLen, dist, outError)) return false;
      }

      while (true) {
        std::uint16_t sym = 0;
        if (!litLen.decode(br, sym, outError)) return false;

        if (sym < 256u) {
          if (!CheckOutCap(1u)) return false;
          out.push_back(static_cast<std::uint8_t>(sym));
          continue;
        }

        if (sym == 256u) {
          break; // end of block
        }

        if (sym > 285u) {
          outError = "invalid literal/length symbol";
          return false;
        }

        const int lenCode = static_cast<int>(sym) - 257;
        int length = kLenBase[lenCode];
        const int le = kLenExtra[lenCode];
        if (le > 0) {
          std::uint32_t extra = 0;
          if (!br.readBits(le, extra, outError)) return false;
          length += static_cast<int>(extra);
        }

        std::uint16_t distSym = 0;
        if (!dist.decode(br, distSym, outError)) return false;
        if (distSym >= 30u) {
          outError = "invalid distance symbol";
          return false;
        }

        int distance = kDistBase[static_cast<int>(distSym)];
        const int de = kDistExtra[static_cast<int>(distSym)];
        if (de > 0) {
          std::uint32_t extra = 0;
          if (!br.readBits(de, extra, outError)) return false;
          distance += static_cast<int>(extra);
        }

        if (distance <= 0 || static_cast<std::size_t>(distance) > out.size()) {
          outError = "invalid distance back-reference";
          return false;
        }

        if (!CheckOutCap(static_cast<std::size_t>(length))) return false;

        // Copy match (may overlap).
        const std::size_t distU = static_cast<std::size_t>(distance);
        for (int i = 0; i < length; ++i) {
          out.push_back(out[out.size() - distU]);
        }
      }
    } else {
      outError = "unsupported/invalid DEFLATE block type";
      return false;
    }

    if (bfinal) break;
  }

  // Adler32 checksum follows the DEFLATE stream; it is byte-aligned.
  br.alignToByte();

  std::uint32_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
  if (!br.readBits(8, b0, outError) || !br.readBits(8, b1, outError) || !br.readBits(8, b2, outError) ||
      !br.readBits(8, b3, outError)) {
    outError = "missing Adler32";
    return false;
  }
  const std::uint32_t expected = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;

  const std::uint32_t got = Adler32(out.data(), out.size());
  if (got != expected) {
    std::ostringstream oss;
    oss << "Adler32 mismatch (expected 0x" << std::hex << expected << ", got 0x" << got << ")";
    outError = oss.str();
    return false;
  }

  return true;
}




inline int AbsInt(int x) { return (x < 0) ? -x : x; }

inline std::uint8_t PaethPredictor(std::uint8_t a, std::uint8_t b, std::uint8_t c)
{
  const int ia = static_cast<int>(a);
  const int ib = static_cast<int>(b);
  const int ic = static_cast<int>(c);
  const int p = ia + ib - ic;
  const int pa = AbsInt(p - ia);
  const int pb = AbsInt(p - ib);
  const int pc = AbsInt(p - ic);

  if (pa <= pb && pa <= pc) return a;
  if (pb <= pc) return b;
  return c;
}

bool UnfilterScanline(std::uint8_t filterType, const std::uint8_t* src,
                      const std::vector<std::uint8_t>& prev,
                      std::vector<std::uint8_t>& dst,
                      std::size_t bpp,
                      std::string& outError)
{
  outError.clear();
  const std::size_t n = dst.size();
  if (prev.size() != n) {
    outError = "internal error: previous scanline size mismatch";
    return false;
  }

  switch (filterType) {
    case 0: { // None
      std::copy(src, src + static_cast<std::ptrdiff_t>(n), dst.begin());
      return true;
    }
    case 1: { // Sub
      for (std::size_t i = 0; i < n; ++i) {
        const std::uint8_t left = (i >= bpp) ? dst[i - bpp] : 0u;
        dst[i] = static_cast<std::uint8_t>(src[i] + left);
      }
      return true;
    }
    case 2: { // Up
      for (std::size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<std::uint8_t>(src[i] + prev[i]);
      }
      return true;
    }
    case 3: { // Average
      for (std::size_t i = 0; i < n; ++i) {
        const std::uint8_t left = (i >= bpp) ? dst[i - bpp] : 0u;
        const std::uint8_t up = prev[i];
        const int avg = (static_cast<int>(left) + static_cast<int>(up)) / 2;
        dst[i] = static_cast<std::uint8_t>(static_cast<int>(src[i]) + avg);
      }
      return true;
    }
    case 4: { // Paeth
      for (std::size_t i = 0; i < n; ++i) {
        const std::uint8_t left = (i >= bpp) ? dst[i - bpp] : 0u;
        const std::uint8_t up = prev[i];
        const std::uint8_t upLeft = (i >= bpp) ? prev[i - bpp] : 0u;
        const std::uint8_t pr = PaethPredictor(left, up, upLeft);
        dst[i] = static_cast<std::uint8_t>(src[i] + pr);
      }
      return true;
    }
    default: {
      std::ostringstream oss;
      oss << "unsupported PNG filter type (" << static_cast<int>(filterType) << ")";
      outError = oss.str();
      return false;
    }
  }
}

std::vector<std::uint8_t> CompressZlibStored(const std::uint8_t* data, std::size_t size)
{
  // zlib header: CMF=0x78 (deflate, 32k window), FLG=0x01 (no preset dict, check bits).
  std::vector<std::uint8_t> out;
  out.reserve(size + 64);
  out.push_back(0x78u);
  out.push_back(0x01u);

  const std::uint8_t* p = data;
  std::size_t remaining = size;
  while (remaining > 0) {
    const std::uint16_t chunk = static_cast<std::uint16_t>(std::min<std::size_t>(remaining, 65535u));
    const bool final = (remaining == chunk);

    // Stored block header (byte-aligned): BFINAL + BTYPE(00) + padding.
    out.push_back(final ? 0x01u : 0x00u);

    // LEN and NLEN are little-endian.
    out.push_back(static_cast<std::uint8_t>(chunk & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((chunk >> 8) & 0xFFu));
    const std::uint16_t nlen = static_cast<std::uint16_t>(chunk ^ 0xFFFFu);
    out.push_back(static_cast<std::uint8_t>(nlen & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((nlen >> 8) & 0xFFu));

    out.insert(out.end(), p, p + chunk);
    p += chunk;
    remaining -= chunk;
  }

  const std::uint32_t adler = Adler32(data, size);
  // Adler32 is big-endian.
  out.push_back(static_cast<std::uint8_t>((adler >> 24) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((adler >> 16) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((adler >> 8) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>(adler & 0xFFu));

  return out;
}

} // namespace

bool WritePng(const std::string& path, const PpmImage& img, std::string& outError)
{
  outError.clear();

  if (img.width <= 0 || img.height <= 0) {
    outError = "invalid image dimensions";
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u;
  if (img.rgb.size() != expected) {
    std::ostringstream oss;
    oss << "invalid image buffer size (expected " << expected << ", got " << img.rgb.size() << ")";
    outError = oss.str();
    return false;
  }

  // Build raw scanlines (filter byte 0 + RGB data per row).
  const std::size_t rowBytes = 1u + static_cast<std::size_t>(img.width) * 3u;
  std::vector<std::uint8_t> raw;
  raw.resize(rowBytes * static_cast<std::size_t>(img.height));

  for (int y = 0; y < img.height; ++y) {
    const std::size_t dstRow = static_cast<std::size_t>(y) * rowBytes;
    raw[dstRow] = 0u; // filter: none

    const std::size_t srcRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) * 3u;
    std::copy(img.rgb.begin() + static_cast<std::ptrdiff_t>(srcRow),
              img.rgb.begin() + static_cast<std::ptrdiff_t>(srcRow + static_cast<std::size_t>(img.width) * 3u),
              raw.begin() + static_cast<std::ptrdiff_t>(dstRow + 1u));
  }

  const std::vector<std::uint8_t> z = CompressZlibStored(raw.data(), raw.size());

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for writing";
    return false;
  }

  static constexpr std::uint8_t kSig[8] = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
  if (!WriteExact(f, kSig, sizeof(kSig))) {
    outError = "failed to write PNG signature";
    return false;
  }

  // IHDR (13 bytes)
  {
    std::uint8_t ihdr[13] = {};
    ihdr[0] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.width) >> 24) & 0xFFu);
    ihdr[1] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.width) >> 16) & 0xFFu);
    ihdr[2] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.width) >> 8) & 0xFFu);
    ihdr[3] = static_cast<std::uint8_t>(static_cast<std::uint32_t>(img.width) & 0xFFu);
    ihdr[4] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.height) >> 24) & 0xFFu);
    ihdr[5] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.height) >> 16) & 0xFFu);
    ihdr[6] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.height) >> 8) & 0xFFu);
    ihdr[7] = static_cast<std::uint8_t>(static_cast<std::uint32_t>(img.height) & 0xFFu);
    ihdr[8] = 8u;  // bit depth
    ihdr[9] = 2u;  // color type: truecolor
    ihdr[10] = 0u; // compression
    ihdr[11] = 0u; // filter
    ihdr[12] = 0u; // interlace

    std::string err;
    const char type[4] = {'I', 'H', 'D', 'R'};
    if (!WritePngChunk(f, type, ihdr, sizeof(ihdr), err)) {
      outError = err;
      return false;
    }
  }

  // IDAT
  {
    std::string err;
    const char type[4] = {'I', 'D', 'A', 'T'};
    if (!WritePngChunk(f, type, z.data(), z.size(), err)) {
      outError = err;
      return false;
    }
  }

  // IEND
  {
    std::string err;
    const char type[4] = {'I', 'E', 'N', 'D'};
    if (!WritePngChunk(f, type, nullptr, 0, err)) {
      outError = err;
      return false;
    }
  }

  return true;
}



bool WritePngRGBA(const std::string& path, const RgbaImage& img, std::string& outError)
{
  outError.clear();

  if (img.width <= 0 || img.height <= 0) {
    outError = "invalid image dimensions";
    return false;
  }
  const std::size_t expected =
      static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u;
  if (img.rgba.size() != expected) {
    std::ostringstream oss;
    oss << "invalid image buffer size (expected " << expected << ", got " << img.rgba.size() << ")";
    outError = oss.str();
    return false;
  }

  // Build raw scanlines (filter byte 0 + RGBA data per row).
  const std::size_t rowBytes = 1u + static_cast<std::size_t>(img.width) * 4u;
  std::vector<std::uint8_t> raw;
  raw.resize(rowBytes * static_cast<std::size_t>(img.height));

  for (int y = 0; y < img.height; ++y) {
    const std::size_t dstRow = static_cast<std::size_t>(y) * rowBytes;
    raw[dstRow] = 0u; // filter: none

    const std::size_t srcRow =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) * 4u;
    std::copy(img.rgba.begin() + static_cast<std::ptrdiff_t>(srcRow),
              img.rgba.begin() + static_cast<std::ptrdiff_t>(srcRow + static_cast<std::size_t>(img.width) * 4u),
              raw.begin() + static_cast<std::ptrdiff_t>(dstRow + 1u));
  }

  const std::vector<std::uint8_t> z = CompressZlibStored(raw.data(), raw.size());

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for writing";
    return false;
  }

  static constexpr std::uint8_t kSig[8] = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
  if (!WriteExact(f, kSig, sizeof(kSig))) {
    outError = "failed to write PNG signature";
    return false;
  }

  // IHDR (13 bytes)
  {
    std::uint8_t ihdr[13] = {};
    ihdr[0] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.width) >> 24) & 0xFFu);
    ihdr[1] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.width) >> 16) & 0xFFu);
    ihdr[2] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.width) >> 8) & 0xFFu);
    ihdr[3] = static_cast<std::uint8_t>(static_cast<std::uint32_t>(img.width) & 0xFFu);
    ihdr[4] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.height) >> 24) & 0xFFu);
    ihdr[5] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.height) >> 16) & 0xFFu);
    ihdr[6] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(img.height) >> 8) & 0xFFu);
    ihdr[7] = static_cast<std::uint8_t>(static_cast<std::uint32_t>(img.height) & 0xFFu);
    ihdr[8] = 8u;  // bit depth
    ihdr[9] = 6u;  // color type: truecolor + alpha (RGBA)
    ihdr[10] = 0u; // compression
    ihdr[11] = 0u; // filter
    ihdr[12] = 0u; // interlace

    std::string err;
    const char type[4] = {'I', 'H', 'D', 'R'};
    if (!WritePngChunk(f, type, ihdr, sizeof(ihdr), err)) {
      outError = err;
      return false;
    }
  }

  // IDAT
  {
    std::string err;
    const char type[4] = {'I', 'D', 'A', 'T'};
    if (!WritePngChunk(f, type, z.data(), z.size(), err)) {
      outError = err;
      return false;
    }
  }

  // IEND
  {
    std::string err;
    const char type[4] = {'I', 'E', 'N', 'D'};
    if (!WritePngChunk(f, type, nullptr, 0, err)) {
      outError = err;
      return false;
    }
  }

  return true;
}


bool ReadPngRGBA(const std::string& path, RgbaImage& outImg, std::string& outError)
{
  outError.clear();
  outImg = {};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for reading";
    return false;
  }

  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (!HasPngSignature(bytes.data(), bytes.size())) {
    outError = "not a PNG file (missing signature)";
    return false;
  }

  std::size_t off = 8;

  bool haveIHDR = false;
  int w = 0;
  int h = 0;
  std::uint8_t bitDepth = 0;
  std::uint8_t colorType = 0;
  std::uint8_t compression = 0;
  std::uint8_t filterMethod = 0;
  std::uint8_t interlace = 0;

  std::vector<std::uint8_t> idat;
  std::vector<std::uint8_t> plteRgb;
  std::vector<std::uint8_t> trnsAlpha;

  while (off + 12 <= bytes.size()) {
    const std::uint32_t len = (static_cast<std::uint32_t>(bytes[off + 0]) << 24) | (static_cast<std::uint32_t>(bytes[off + 1]) << 16) |
                              (static_cast<std::uint32_t>(bytes[off + 2]) << 8) | static_cast<std::uint32_t>(bytes[off + 3]);
    const char type[4] = {static_cast<char>(bytes[off + 4]), static_cast<char>(bytes[off + 5]), static_cast<char>(bytes[off + 6]),
                          static_cast<char>(bytes[off + 7])};

    const std::size_t chunkStart = off + 8;
    const std::size_t chunkEnd = chunkStart + static_cast<std::size_t>(len);
    const std::size_t crcPos = chunkEnd;

    if (chunkEnd + 4 > bytes.size()) {
      outError = "truncated PNG chunk";
      return false;
    }

    const std::uint32_t crcFile = (static_cast<std::uint32_t>(bytes[crcPos + 0]) << 24) | (static_cast<std::uint32_t>(bytes[crcPos + 1]) << 16) |
                                  (static_cast<std::uint32_t>(bytes[crcPos + 2]) << 8) | static_cast<std::uint32_t>(bytes[crcPos + 3]);

    const std::uint32_t crcExpected = CrcPngChunk(type, bytes.data() + chunkStart, static_cast<std::size_t>(len));
    if (crcFile != crcExpected) {
      std::ostringstream oss;
      oss << "PNG CRC mismatch for chunk '" << std::string(type, 4) << "'";
      outError = oss.str();
      return false;
    }

    const std::string t(type, 4);

    if (t == "IHDR") {
      if (len != 13u) {
        outError = "invalid IHDR length";
        return false;
      }

      const std::uint32_t wb = (static_cast<std::uint32_t>(bytes[chunkStart + 0]) << 24) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 1]) << 16) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 2]) << 8) |
                               static_cast<std::uint32_t>(bytes[chunkStart + 3]);
      const std::uint32_t hb = (static_cast<std::uint32_t>(bytes[chunkStart + 4]) << 24) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 5]) << 16) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 6]) << 8) |
                               static_cast<std::uint32_t>(bytes[chunkStart + 7]);

      if (wb == 0 || hb == 0 || wb > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
          hb > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        outError = "invalid IHDR dimensions";
        return false;
      }
      w = static_cast<int>(wb);
      h = static_cast<int>(hb);

      bitDepth = bytes[chunkStart + 8];
      colorType = bytes[chunkStart + 9];
      compression = bytes[chunkStart + 10];
      filterMethod = bytes[chunkStart + 11];
      interlace = bytes[chunkStart + 12];

      // Supported subset:
      //  - bit depth 8
      //  - color type 6 (RGBA), 2 (RGB), or 3 (indexed)
      //  - compression 0, filter method 0, interlace 0
      if (bitDepth != 8u || (colorType != 6u && colorType != 2u && colorType != 3u) ||
          compression != 0u || filterMethod != 0u || interlace != 0u) {
        outError = "unsupported PNG format (expected RGBA8/RGB8/Indexed8, no interlace, filter method 0)";
        return false;
      }
      haveIHDR = true;
    } else if (t == "PLTE") {
      plteRgb.assign(bytes.begin() + static_cast<std::ptrdiff_t>(chunkStart), bytes.begin() + static_cast<std::ptrdiff_t>(chunkEnd));
      if ((plteRgb.size() % 3u) != 0u) {
        outError = "invalid PLTE chunk length (must be a multiple of 3)";
        return false;
      }
      const std::size_t entries = plteRgb.size() / 3u;
      if (entries < 1u || entries > 256u) {
        outError = "invalid PLTE palette size (must be 1..256)";
        return false;
      }
    } else if (t == "tRNS") {
      trnsAlpha.assign(bytes.begin() + static_cast<std::ptrdiff_t>(chunkStart), bytes.begin() + static_cast<std::ptrdiff_t>(chunkEnd));
      if (trnsAlpha.size() > 256u) {
        outError = "invalid tRNS chunk length";
        return false;
      }
    } else if (t == "IDAT") {
      idat.insert(idat.end(),
                  bytes.begin() + static_cast<std::ptrdiff_t>(chunkStart),
                  bytes.begin() + static_cast<std::ptrdiff_t>(chunkEnd));
    } else if (t == "IEND") {
      break;
    }

    off = crcPos + 4;
  }

  if (!haveIHDR) {
    outError = "missing IHDR";
    return false;
  }
  if (idat.empty()) {
    outError = "missing IDAT";
    return false;
  }

  if (colorType == 3u) {
    if (plteRgb.empty()) {
      outError = "indexed PNG missing PLTE palette";
      return false;
    }
    const std::size_t palEntries = plteRgb.size() / 3u;
    if (!trnsAlpha.empty() && trnsAlpha.size() > palEntries) {
      outError = "invalid tRNS length (exceeds palette size)";
      return false;
    }
  }

  const std::size_t bpp = (colorType == 6u) ? 4u : (colorType == 2u) ? 3u : 1u;
  const std::size_t rowBytes = 1u + static_cast<std::size_t>(w) * bpp;
  const std::size_t expectedRaw = rowBytes * static_cast<std::size_t>(h);

  std::vector<std::uint8_t> raw;
  std::string derr;
  if (!DecompressZlib(idat, raw, expectedRaw, derr)) {
    outError = "failed to decompress IDAT: " + derr;
    return false;
  }
  if (raw.size() != expectedRaw) {
    std::ostringstream oss;
    oss << "unexpected decompressed size (expected " << expectedRaw << ", got " << raw.size() << ")";
    outError = oss.str();
    return false;
  }

  RgbaImage img;
  img.width = w;
  img.height = h;
  img.rgba.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u);

  std::vector<std::uint8_t> prev(static_cast<std::size_t>(w) * bpp, 0u);
  std::vector<std::uint8_t> recon(static_cast<std::size_t>(w) * bpp, 0u);

  const std::size_t palEntries = plteRgb.size() / 3u;

  for (int y = 0; y < h; ++y) {
    const std::size_t srcRow = static_cast<std::size_t>(y) * rowBytes;
    const std::uint8_t filterType = raw[srcRow];
    const std::uint8_t* src = raw.data() + srcRow + 1u;

    std::string ferr;
    if (!UnfilterScanline(filterType, src, prev, recon, bpp, ferr)) {
      outError = "failed to unfilter scanline: " + ferr;
      return false;
    }

    const std::size_t dstRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * 4u;
    std::uint8_t* dst = img.rgba.data() + dstRow;

    if (colorType == 6u) {
      // RGBA8: direct copy.
      std::copy(recon.begin(), recon.end(), dst);
    } else if (colorType == 2u) {
      // RGB8: expand to RGBA with opaque alpha.
      for (int x = 0; x < w; ++x) {
        const std::size_t si = static_cast<std::size_t>(x) * 3u;
        const std::size_t di = static_cast<std::size_t>(x) * 4u;
        dst[di + 0u] = recon[si + 0u];
        dst[di + 1u] = recon[si + 1u];
        dst[di + 2u] = recon[si + 2u];
        dst[di + 3u] = 255u;
      }
    } else {
      // Indexed8: palette lookup + optional per-entry alpha.
      for (int x = 0; x < w; ++x) {
        const std::uint8_t idx = recon[static_cast<std::size_t>(x)];
        if (idx >= palEntries) {
          outError = "indexed PNG pixel references palette entry out of range";
          return false;
        }
        const std::size_t pi = static_cast<std::size_t>(idx) * 3u;
        const std::size_t di = static_cast<std::size_t>(x) * 4u;
        dst[di + 0u] = plteRgb[pi + 0u];
        dst[di + 1u] = plteRgb[pi + 1u];
        dst[di + 2u] = plteRgb[pi + 2u];
        dst[di + 3u] = (static_cast<std::size_t>(idx) < trnsAlpha.size()) ? trnsAlpha[static_cast<std::size_t>(idx)] : 255u;
      }
    }

    std::swap(prev, recon);
  }

  outImg = std::move(img);
  return true;
}


bool WritePngIndexed(const std::string& path, int width, int height,
                     const std::vector<std::uint8_t>& indices,
                     const std::vector<std::uint8_t>& paletteRgba,
                     std::string& outError)
{
  outError.clear();

  if (width <= 0 || height <= 0) {
    outError = "invalid image dimensions";
    return false;
  }

  const std::size_t expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (indices.size() != expected) {
    std::ostringstream oss;
    oss << "invalid index buffer size (expected " << expected << ", got " << indices.size() << ")";
    outError = oss.str();
    return false;
  }
  if (paletteRgba.empty() || (paletteRgba.size() % 4u) != 0u) {
    outError = "invalid palette buffer (expected RGBA bytes)";
    return false;
  }

  const std::size_t palSize = paletteRgba.size() / 4u;
  if (palSize < 1u || palSize > 256u) {
    outError = "palette size must be in [1, 256]";
    return false;
  }

  // Build raw scanlines (filter byte 0 + index data per row).
  const std::size_t rowBytes = 1u + static_cast<std::size_t>(width);
  std::vector<std::uint8_t> raw;
  raw.resize(rowBytes * static_cast<std::size_t>(height));

  for (int y = 0; y < height; ++y) {
    const std::size_t dstRow = static_cast<std::size_t>(y) * rowBytes;
    raw[dstRow] = 0u; // filter: none
    const std::size_t srcRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    std::copy(indices.begin() + static_cast<std::ptrdiff_t>(srcRow),
              indices.begin() + static_cast<std::ptrdiff_t>(srcRow + static_cast<std::size_t>(width)),
              raw.begin() + static_cast<std::ptrdiff_t>(dstRow + 1u));
  }

  const std::vector<std::uint8_t> z = CompressZlibStored(raw.data(), raw.size());

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for writing";
    return false;
  }

  static constexpr std::uint8_t kSig[8] = {0x89u, 'P', 'N', 'G', 0x0Du, 0x0Au, 0x1Au, 0x0Au};
  if (!WriteExact(f, kSig, sizeof(kSig))) {
    outError = "failed to write PNG signature";
    return false;
  }

  // IHDR (13 bytes)
  {
    std::uint8_t ihdr[13] = {};
    ihdr[0] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(width) >> 24) & 0xFFu);
    ihdr[1] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(width) >> 16) & 0xFFu);
    ihdr[2] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(width) >> 8) & 0xFFu);
    ihdr[3] = static_cast<std::uint8_t>(static_cast<std::uint32_t>(width) & 0xFFu);
    ihdr[4] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(height) >> 24) & 0xFFu);
    ihdr[5] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(height) >> 16) & 0xFFu);
    ihdr[6] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(height) >> 8) & 0xFFu);
    ihdr[7] = static_cast<std::uint8_t>(static_cast<std::uint32_t>(height) & 0xFFu);
    ihdr[8] = 8u;  // bit depth (per palette index)
    ihdr[9] = 3u;  // color type: indexed-color
    ihdr[10] = 0u; // compression
    ihdr[11] = 0u; // filter
    ihdr[12] = 0u; // interlace

    std::string err;
    const char type[4] = {'I', 'H', 'D', 'R'};
    if (!WritePngChunk(f, type, ihdr, sizeof(ihdr), err)) {
      outError = err;
      return false;
    }
  }

  // PLTE (RGB palette)
  {
    std::vector<std::uint8_t> plte;
    plte.resize(palSize * 3u);
    for (std::size_t i = 0; i < palSize; ++i) {
      plte[i * 3u + 0u] = paletteRgba[i * 4u + 0u];
      plte[i * 3u + 1u] = paletteRgba[i * 4u + 1u];
      plte[i * 3u + 2u] = paletteRgba[i * 4u + 2u];
    }

    std::string err;
    const char type[4] = {'P', 'L', 'T', 'E'};
    if (!WritePngChunk(f, type, plte.data(), plte.size(), err)) {
      outError = err;
      return false;
    }
  }

  // tRNS (alpha per palette entry)
  {
    std::vector<std::uint8_t> trns;
    trns.resize(palSize);
    bool anyAlpha = false;
    for (std::size_t i = 0; i < palSize; ++i) {
      const std::uint8_t a = paletteRgba[i * 4u + 3u];
      trns[i] = a;
      if (a != 255u) anyAlpha = true;
    }

    if (anyAlpha) {
      std::string err;
      const char type[4] = {'t', 'R', 'N', 'S'};
      if (!WritePngChunk(f, type, trns.data(), trns.size(), err)) {
        outError = err;
        return false;
      }
    }
  }

  // IDAT
  {
    std::string err;
    const char type[4] = {'I', 'D', 'A', 'T'};
    if (!WritePngChunk(f, type, z.data(), z.size(), err)) {
      outError = err;
      return false;
    }
  }

  // IEND
  {
    std::string err;
    const char type[4] = {'I', 'E', 'N', 'D'};
    if (!WritePngChunk(f, type, nullptr, 0, err)) {
      outError = err;
      return false;
    }
  }

  return true;
}

PpmImage CompositeOverSolid(const RgbaImage& img, std::uint8_t bgR, std::uint8_t bgG, std::uint8_t bgB)
{
  PpmImage out;
  out.width = img.width;
  out.height = img.height;
  out.rgb.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u);

  const std::size_t pxCount = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height);
  for (std::size_t i = 0; i < pxCount; ++i) {
    const std::size_t si = i * 4u;
    const std::size_t di = i * 3u;

    const std::uint8_t sr = img.rgba[si + 0];
    const std::uint8_t sg = img.rgba[si + 1];
    const std::uint8_t sb = img.rgba[si + 2];
    const std::uint8_t sa = img.rgba[si + 3];

    const std::uint32_t a = static_cast<std::uint32_t>(sa);
    const std::uint32_t ia = 255u - a;

    out.rgb[di + 0] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(sr) * a + static_cast<std::uint32_t>(bgR) * ia + 127u) / 255u);
    out.rgb[di + 1] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(sg) * a + static_cast<std::uint32_t>(bgG) * ia + 127u) / 255u);
    out.rgb[di + 2] = static_cast<std::uint8_t>((static_cast<std::uint32_t>(sb) * a + static_cast<std::uint32_t>(bgB) * ia + 127u) / 255u);
  }
  return out;
}


bool ReadPng(const std::string& path, PpmImage& outImg, std::string& outError)
{
  outError.clear();
  outImg = {};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for reading";
    return false;
  }

  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (!HasPngSignature(bytes.data(), bytes.size())) {
    outError = "not a PNG file (missing signature)";
    return false;
  }

  std::size_t off = 8;

  bool haveIHDR = false;
  int w = 0;
  int h = 0;

  std::vector<std::uint8_t> idat;

  while (off + 12 <= bytes.size()) {
    const std::uint32_t len = (static_cast<std::uint32_t>(bytes[off + 0]) << 24) | (static_cast<std::uint32_t>(bytes[off + 1]) << 16) |
                              (static_cast<std::uint32_t>(bytes[off + 2]) << 8) | static_cast<std::uint32_t>(bytes[off + 3]);
    const char type[4] = {static_cast<char>(bytes[off + 4]), static_cast<char>(bytes[off + 5]), static_cast<char>(bytes[off + 6]),
                          static_cast<char>(bytes[off + 7])};

    const std::size_t chunkStart = off + 8;
    const std::size_t chunkEnd = chunkStart + static_cast<std::size_t>(len);
    const std::size_t crcPos = chunkEnd;

    if (chunkEnd + 4 > bytes.size()) {
      outError = "truncated PNG chunk";
      return false;
    }

    const std::uint32_t crcFile = (static_cast<std::uint32_t>(bytes[crcPos + 0]) << 24) | (static_cast<std::uint32_t>(bytes[crcPos + 1]) << 16) |
                                  (static_cast<std::uint32_t>(bytes[crcPos + 2]) << 8) | static_cast<std::uint32_t>(bytes[crcPos + 3]);
    const std::uint32_t crcExpected = CrcPngChunk(type, bytes.data() + chunkStart, static_cast<std::size_t>(len));
    if (crcFile != crcExpected) {
      std::ostringstream oss;
      oss << "PNG CRC mismatch for chunk '" << std::string(type, 4) << "'";
      outError = oss.str();
      return false;
    }

    const std::string t(type, 4);
    if (t == "IHDR") {
      if (len != 13u) {
        outError = "invalid IHDR length";
        return false;
      }

      const std::uint32_t wb = (static_cast<std::uint32_t>(bytes[chunkStart + 0]) << 24) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 1]) << 16) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 2]) << 8) |
                               static_cast<std::uint32_t>(bytes[chunkStart + 3]);
      const std::uint32_t hb = (static_cast<std::uint32_t>(bytes[chunkStart + 4]) << 24) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 5]) << 16) |
                               (static_cast<std::uint32_t>(bytes[chunkStart + 6]) << 8) |
                               static_cast<std::uint32_t>(bytes[chunkStart + 7]);
      if (wb == 0 || hb == 0 || wb > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
          hb > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        outError = "invalid IHDR dimensions";
        return false;
      }
      w = static_cast<int>(wb);
      h = static_cast<int>(hb);

      const std::uint8_t bitDepth = bytes[chunkStart + 8];
      const std::uint8_t colorType = bytes[chunkStart + 9];
      const std::uint8_t compression = bytes[chunkStart + 10];
      const std::uint8_t filterMethod = bytes[chunkStart + 11];
      const std::uint8_t interlace = bytes[chunkStart + 12];

      if (bitDepth != 8u || colorType != 2u || compression != 0u || filterMethod != 0u || interlace != 0u) {
        outError = "unsupported PNG format (expected RGB8, no interlace, filter method 0)";
        return false;
      }
      haveIHDR = true;
    } else if (t == "IDAT") {
      idat.insert(idat.end(),
                  bytes.begin() + static_cast<std::ptrdiff_t>(chunkStart),
                  bytes.begin() + static_cast<std::ptrdiff_t>(chunkEnd));
    } else if (t == "IEND") {
      break;
    }

    off = crcPos + 4;
  }

  if (!haveIHDR) {
    outError = "missing IHDR";
    return false;
  }
  if (idat.empty()) {
    outError = "missing IDAT";
    return false;
  }

  const std::size_t bpp = 3u;
  const std::size_t rowBytes = 1u + static_cast<std::size_t>(w) * bpp;
  const std::size_t expectedRaw = rowBytes * static_cast<std::size_t>(h);

  std::vector<std::uint8_t> raw;
  std::string derr;
  if (!DecompressZlib(idat, raw, expectedRaw, derr)) {
    outError = "failed to decompress IDAT: " + derr;
    return false;
  }

  if (raw.size() != expectedRaw) {
    std::ostringstream oss;
    oss << "unexpected decompressed size (expected " << expectedRaw << ", got " << raw.size() << ")";
    outError = oss.str();
    return false;
  }

  PpmImage img;
  img.width = w;
  img.height = h;
  img.rgb.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u);

  std::vector<std::uint8_t> prev(static_cast<std::size_t>(w) * bpp, 0u);
  std::vector<std::uint8_t> recon(static_cast<std::size_t>(w) * bpp, 0u);

  for (int y = 0; y < h; ++y) {
    const std::size_t srcRow = static_cast<std::size_t>(y) * rowBytes;
    const std::uint8_t filterType = raw[srcRow];
    const std::uint8_t* src = raw.data() + srcRow + 1u;

    std::string ferr;
    if (!UnfilterScanline(filterType, src, prev, recon, bpp, ferr)) {
      outError = "failed to unfilter scanline: " + ferr;
      return false;
    }

    const std::size_t dstRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * 3u;
    std::copy(recon.begin(), recon.end(), img.rgb.begin() + static_cast<std::ptrdiff_t>(dstRow));

    std::swap(prev, recon);
  }

  outImg = std::move(img);
  return true;
}


bool ReadImageAuto(const std::string& path, PpmImage& outImg, std::string& outError)
{
  const std::string ext = LowerExt(path);
  if (ext == ".png") return ReadPng(path, outImg, outError);
  if (ext == ".ppm" || ext == ".pnm") return ReadPpm(path, outImg, outError);

  // Probe magic.
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for reading";
    return false;
  }

  std::uint8_t head[8] = {};
  f.read(reinterpret_cast<char*>(head), static_cast<std::streamsize>(sizeof(head)));
  const std::size_t got = static_cast<std::size_t>(f.gcount());

  if (HasPngSignature(head, got)) return ReadPng(path, outImg, outError);
  if (got >= 2 && head[0] == 'P' && head[1] == '6') return ReadPpm(path, outImg, outError);

  outError = "unknown image format (expected .ppm or .png)";
  return false;
}

bool WriteImageAuto(const std::string& path, const PpmImage& img, std::string& outError)
{
  const std::string ext = LowerExt(path);
  if (ext == ".png") return WritePng(path, img, outError);
  return WritePpm(path, img, outError);
}

} // namespace isocity
