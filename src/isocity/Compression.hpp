#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

enum class CompressionMethod : std::uint8_t {
  None = 0,
  SLLZ = 1,
};

// Tiny internal compression used by the save system.
//
// SLLZ (Simple Literal/LZ) format:
//   A stream of commands. Each command begins with a one-byte tag.
//
//   If (tag & 0x80) == 0:
//     Literal run of length (tag + 1) bytes follows.
//     Length range: 1..128.
//
//   If (tag & 0x80) != 0:
//     Back-reference (LZ) copy:
//       length = (tag & 0x7F) + 3   (range: 3..130)
//       then a 16-bit little-endian offset follows (1..65535)
//       meaning: copy `length` bytes from (out.size() - offset).
//     Copies are allowed to overlap (like memmove).
//
// This format is intentionally simple and self-contained (no external deps).

// Compress `data[0..size)` into `out`.
// Returns true on success.
bool CompressSLLZ(const std::uint8_t* data, std::size_t size, std::vector<std::uint8_t>& out);

// Decompress `data[0..size)` (SLLZ) into `out`, expecting exactly `expectedSize` bytes.
// Returns true on success; on failure returns false and writes `outError`.
bool DecompressSLLZ(const std::uint8_t* data, std::size_t size, std::size_t expectedSize,
                    std::vector<std::uint8_t>& out, std::string& outError);

} // namespace isocity
