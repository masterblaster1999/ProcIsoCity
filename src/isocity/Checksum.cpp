#include "isocity/Checksum.hpp"

#include <cstddef>
#include <cstdint>

namespace isocity {

namespace {

const std::uint32_t* Crc32Table()
{
  static std::uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int bit = 0; bit < 8; ++bit) {
        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      }
      table[i] = c;
    }
    init = true;
  }
  return table;
}

} // namespace

std::uint32_t Crc32Update(std::uint32_t crc, const std::uint8_t* data, std::size_t size)
{
  if (!data || size == 0) return crc;
  const std::uint32_t* table = Crc32Table();
  for (std::size_t i = 0; i < size; ++i) {
    crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return crc;
}

std::uint32_t Adler32Update(std::uint32_t adler, const std::uint8_t* data, std::size_t size)
{
  // Implementation based on zlib's recommended chunking (5552 bytes per mod) to
  // reduce the number of expensive modulo operations.
  constexpr std::uint32_t kMod = 65521u;

  std::uint32_t a = adler & 0xFFFFu;
  std::uint32_t b = (adler >> 16) & 0xFFFFu;

  if (!data || size == 0) return (b << 16) | a;

  while (size > 0) {
    const std::size_t chunk = (size > 5552u) ? 5552u : size;
    size -= chunk;

    for (std::size_t i = 0; i < chunk; ++i) {
      a += static_cast<std::uint32_t>(data[i]);
      b += a;
    }

    a %= kMod;
    b %= kMod;
    data += chunk;
  }

  return (b << 16) | a;
}

} // namespace isocity
