#pragma once

#include <cstddef>
#include <cstdint>

namespace isocity {

// Common checksum utilities used across the project.
//
// CRC32:
//   - IEEE 802.3 polynomial (0xEDB88320)
//   - Standard init/final XOR (init = 0xFFFFFFFF, finalize by XOR with 0xFFFFFFFF)
//
// Adler32:
//   - zlib/RFC1950 checksum (init = 1)

// Incremental CRC32 update.
//
// Typical usage:
//   std::uint32_t crc = 0xFFFFFFFFu;
//   crc = Crc32Update(crc, data, size);
//   ...
//   crc ^= 0xFFFFFFFFu;
std::uint32_t Crc32Update(std::uint32_t crc, const std::uint8_t* data, std::size_t size);

// Convenience: compute a finalized CRC32 for a single buffer.
inline std::uint32_t Crc32(const std::uint8_t* data, std::size_t size)
{
  std::uint32_t crc = 0xFFFFFFFFu;
  crc = Crc32Update(crc, data, size);
  return crc ^ 0xFFFFFFFFu;
}

// Incremental Adler32 update (init with 1).
std::uint32_t Adler32Update(std::uint32_t adler, const std::uint8_t* data, std::size_t size);

// Convenience: compute Adler32 for a single buffer.
inline std::uint32_t Adler32(const std::uint8_t* data, std::size_t size)
{
  return Adler32Update(1u, data, size);
}

} // namespace isocity
