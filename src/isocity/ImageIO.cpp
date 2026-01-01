#include "isocity/Export.hpp"

#include "isocity/Checksum.hpp"

#include <algorithm>
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

bool DecompressZlibStored(const std::vector<std::uint8_t>& in, std::vector<std::uint8_t>& out, std::string& outError)
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

  std::size_t pos = 2;

  while (true) {
    if (pos >= in.size()) {
      outError = "truncated DEFLATE stream";
      return false;
    }

    // NOTE: This decoder only supports streams produced by WritePng:
    // it assumes the DEFLATE stream is byte-aligned and uses stored blocks.
    const std::uint8_t hdr = in[pos++];
    const bool bfinal = (hdr & 0x01u) != 0u;
    const std::uint8_t btype = (hdr >> 1) & 0x03u;
    if (btype != 0u) {
      outError = "unsupported DEFLATE block type (expected stored/uncompressed)";
      return false;
    }

    if (pos + 4 > in.size()) {
      outError = "truncated stored block header";
      return false;
    }

    const std::uint16_t len = static_cast<std::uint16_t>(in[pos + 0] | (static_cast<std::uint16_t>(in[pos + 1]) << 8));
    const std::uint16_t nlen = static_cast<std::uint16_t>(in[pos + 2] | (static_cast<std::uint16_t>(in[pos + 3]) << 8));
    pos += 4;
    if (static_cast<std::uint16_t>(len ^ 0xFFFFu) != nlen) {
      outError = "stored block LEN/NLEN mismatch";
      return false;
    }

    if (pos + len > in.size()) {
      outError = "truncated stored block payload";
      return false;
    }

    out.insert(out.end(), in.begin() + static_cast<std::ptrdiff_t>(pos), in.begin() + static_cast<std::ptrdiff_t>(pos + len));
    pos += len;

    if (bfinal) break;
  }

  if (pos + 4 > in.size()) {
    outError = "missing Adler32";
    return false;
  }

  const std::uint32_t expected = (static_cast<std::uint32_t>(in[pos + 0]) << 24) | (static_cast<std::uint32_t>(in[pos + 1]) << 16) |
                                 (static_cast<std::uint32_t>(in[pos + 2]) << 8) | static_cast<std::uint32_t>(in[pos + 3]);
  pos += 4;

  const std::uint32_t got = Adler32(out.data(), out.size());
  if (got != expected) {
    std::ostringstream oss;
    oss << "Adler32 mismatch (expected 0x" << std::hex << expected << ", got 0x" << got << ")";
    outError = oss.str();
    return false;
  }

  // PNG IDAT should not have extra bytes, but tolerate trailing zeros.
  return true;
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

bool ReadPng(const std::string& path, PpmImage& outImg, std::string& outError)
{
  outError.clear();
  outImg = PpmImage{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for reading";
    return false;
  }

  std::uint8_t sig[8] = {};
  if (!ReadExact(f, sig, sizeof(sig)) || !HasPngSignature(sig, sizeof(sig))) {
    outError = "invalid PNG signature";
    return false;
  }

  int w = 0;
  int h = 0;
  bool haveIHDR = false;
  std::vector<std::uint8_t> idat;

  constexpr std::uint32_t kMaxChunk = 256u * 1024u * 1024u;

  while (true) {
    bool ok = false;
    const std::uint32_t len = ReadU32BE(f, ok);
    if (!ok) {
      outError = "truncated PNG (chunk length)";
      return false;
    }
    if (len > kMaxChunk) {
      outError = "PNG chunk too large";
      return false;
    }

    char type[4] = {0, 0, 0, 0};
    if (!ReadExact(f, type, 4)) {
      outError = "truncated PNG (chunk type)";
      return false;
    }

    std::vector<std::uint8_t> data;
    data.resize(static_cast<std::size_t>(len));
    if (len > 0) {
      if (!ReadExact(f, data.data(), data.size())) {
        outError = "truncated PNG (chunk data)";
        return false;
      }
    }

    const std::uint32_t crcFile = ReadU32BE(f, ok);
    if (!ok) {
      outError = "truncated PNG (chunk crc)";
      return false;
    }
    const std::uint32_t crcExpected = CrcPngChunk(type, data.data(), data.size());
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

      const std::uint32_t wb = (static_cast<std::uint32_t>(data[0]) << 24) | (static_cast<std::uint32_t>(data[1]) << 16) |
                               (static_cast<std::uint32_t>(data[2]) << 8) | static_cast<std::uint32_t>(data[3]);
      const std::uint32_t hb = (static_cast<std::uint32_t>(data[4]) << 24) | (static_cast<std::uint32_t>(data[5]) << 16) |
                               (static_cast<std::uint32_t>(data[6]) << 8) | static_cast<std::uint32_t>(data[7]);
      if (wb == 0 || hb == 0 || wb > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
          hb > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        outError = "invalid IHDR dimensions";
        return false;
      }
      w = static_cast<int>(wb);
      h = static_cast<int>(hb);

      const std::uint8_t bitDepth = data[8];
      const std::uint8_t colorType = data[9];
      const std::uint8_t compression = data[10];
      const std::uint8_t filter = data[11];
      const std::uint8_t interlace = data[12];

      if (bitDepth != 8u || colorType != 2u || compression != 0u || filter != 0u || interlace != 0u) {
        outError = "unsupported PNG format (expected RGB8, no interlace, filter 0)";
        return false;
      }
      haveIHDR = true;
    } else if (t == "IDAT") {
      idat.insert(idat.end(), data.begin(), data.end());
    } else if (t == "IEND") {
      break;
    } else {
      // Ignore other chunks.
    }
  }

  if (!haveIHDR) {
    outError = "missing IHDR";
    return false;
  }
  if (idat.empty()) {
    outError = "missing IDAT";
    return false;
  }

  std::vector<std::uint8_t> raw;
  std::string derr;
  if (!DecompressZlibStored(idat, raw, derr)) {
    outError = "failed to decompress IDAT: " + derr;
    return false;
  }

  const std::size_t rowBytes = 1u + static_cast<std::size_t>(w) * 3u;
  const std::size_t expectedRaw = rowBytes * static_cast<std::size_t>(h);
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

  for (int y = 0; y < h; ++y) {
    const std::size_t srcRow = static_cast<std::size_t>(y) * rowBytes;
    const std::uint8_t filter = raw[srcRow];
    if (filter != 0u) {
      outError = "unsupported PNG filter (expected 0)";
      return false;
    }
    const std::size_t dstRow = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * 3u;
    std::copy(raw.begin() + static_cast<std::ptrdiff_t>(srcRow + 1u),
              raw.begin() + static_cast<std::ptrdiff_t>(srcRow + 1u + static_cast<std::size_t>(w) * 3u),
              img.rgb.begin() + static_cast<std::ptrdiff_t>(dstRow));
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
