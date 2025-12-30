#include "isocity/Compression.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

constexpr std::size_t kMaxLit = 128;
constexpr std::size_t kMinMatch = 4;   // matches shorter than this are stored as literals
constexpr std::size_t kMaxMatch = 130; // encoded as (len-3) in 7 bits

inline std::uint32_t Hash3(const std::uint8_t* p)
{
  // A tiny 3-byte hash.
  // This doesn't need to be cryptographic; just good enough to find repeats.
  const std::uint32_t v = (static_cast<std::uint32_t>(p[0]) << 16u) |
                          (static_cast<std::uint32_t>(p[1]) << 8u) |
                          (static_cast<std::uint32_t>(p[2]));
  // Mix bits (xorshift-ish).
  std::uint32_t x = v * 2654435761u;
  x ^= x >> 16;
  x *= 2246822519u;
  x ^= x >> 13;
  return x;
}

inline void EmitLiteral(std::vector<std::uint8_t>& out, const std::uint8_t* bytes, std::size_t len)
{
  while (len > 0) {
    const std::size_t chunk = std::min<std::size_t>(len, kMaxLit);
    out.push_back(static_cast<std::uint8_t>(chunk - 1));
    out.insert(out.end(), bytes, bytes + chunk);
    bytes += chunk;
    len -= chunk;
  }
}

inline void EmitMatch(std::vector<std::uint8_t>& out, std::uint16_t offset, std::size_t len)
{
  // Split very long matches into multiple commands.
  while (len > 0) {
    const std::size_t chunk = std::min<std::size_t>(len, kMaxMatch);
    const std::uint8_t tag = static_cast<std::uint8_t>(0x80u | static_cast<std::uint8_t>(chunk - 3));
    out.push_back(tag);
    out.push_back(static_cast<std::uint8_t>(offset & 0xFFu));
    out.push_back(static_cast<std::uint8_t>((offset >> 8u) & 0xFFu));
    len -= chunk;
  }
}

} // namespace

bool CompressSLLZ(const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& out)
{
  out.clear();
  if (!data || size == 0) {
    return true;
  }

  // We use a single "last position" table keyed by a 3-byte hash.
  // This is essentially a small LZ77/LZSS-style compressor.
  constexpr std::size_t kHashSize = 1u << 16u;
  std::vector<int> last(kHashSize, -1);

  std::size_t i = 0;
  std::size_t litStart = 0;

  while (i < size) {
    if (i + 3 > size) {
      break; // remainder is literals
    }

    const std::uint32_t h = Hash3(data + i) & static_cast<std::uint32_t>(kHashSize - 1u);
    const int prev = last[h];
    last[h] = static_cast<int>(i);

    std::size_t bestLen = 0;
    std::size_t bestOff = 0;

    if (prev >= 0) {
      const std::size_t j = static_cast<std::size_t>(prev);
      const std::size_t off = i - j;
      if (off > 0 && off <= static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max())) {
        const std::size_t maxLen = std::min<std::size_t>(kMaxMatch, size - i);
        std::size_t len = 0;
        while (len < maxLen && data[j + len] == data[i + len]) {
          ++len;
        }
        if (len >= kMinMatch) {
          bestLen = len;
          bestOff = off;
        }
      }
    }

    if (bestLen >= kMinMatch) {
      // Flush preceding literals.
      if (i > litStart) {
        EmitLiteral(out, data + litStart, i - litStart);
      }

      EmitMatch(out, static_cast<std::uint16_t>(bestOff), bestLen);

      // Update hashes for the bytes we just consumed so subsequent matches can find them.
      // (This is still linear-time and keeps compression reasonable.)
      for (std::size_t k = 1; k < bestLen; ++k) {
        const std::size_t p = i + k;
        if (p + 3 > size) break;
        const std::uint32_t hk = Hash3(data + p) & static_cast<std::uint32_t>(kHashSize - 1u);
        last[hk] = static_cast<int>(p);
      }

      i += bestLen;
      litStart = i;
      continue;
    }

    ++i;
  }

  // Flush tail literals.
  if (litStart < size) {
    EmitLiteral(out, data + litStart, size - litStart);
  }

  return true;
}

bool DecompressSLLZ(const std::uint8_t* data, std::size_t size, std::size_t expectedSize,
                    std::vector<std::uint8_t>& out, std::string& outError)
{
  outError.clear();
  out.clear();
  if (expectedSize == 0) {
    return true;
  }
  if (!data) {
    outError = "Null input";
    return false;
  }

  out.reserve(expectedSize);

  std::size_t i = 0;
  while (i < size && out.size() < expectedSize) {
    const std::uint8_t tag = data[i++];
    if ((tag & 0x80u) == 0u) {
      const std::size_t len = static_cast<std::size_t>(tag) + 1u;
      if (i + len > size) {
        outError = "Truncated literal run";
        return false;
      }
      if (out.size() + len > expectedSize) {
        outError = "Literal run exceeds expected output size";
        return false;
      }
      out.insert(out.end(), data + i, data + i + len);
      i += len;
    } else {
      const std::size_t len = static_cast<std::size_t>(tag & 0x7Fu) + 3u;
      if (i + 2 > size) {
        outError = "Truncated match header";
        return false;
      }
      const std::uint16_t off = static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[i]) |
                                                          (static_cast<std::uint16_t>(data[i + 1]) << 8u));
      i += 2;
      if (off == 0) {
        outError = "Invalid match offset (0)";
        return false;
      }
      if (static_cast<std::size_t>(off) > out.size()) {
        outError = "Invalid match offset (beyond output)";
        return false;
      }
      if (out.size() + len > expectedSize) {
        outError = "Match exceeds expected output size";
        return false;
      }

      const std::size_t src = out.size() - static_cast<std::size_t>(off);
      for (std::size_t k = 0; k < len; ++k) {
        out.push_back(out[src + k]);
      }
    }
  }

  if (out.size() != expectedSize) {
    outError = "Decompressed size mismatch";
    return false;
  }
  if (i != size) {
    // We fully consumed the expected output but still have bytes remaining (or vice-versa).
    outError = "Extra data after decompression";
    return false;
  }

  return true;
}

} // namespace isocity
