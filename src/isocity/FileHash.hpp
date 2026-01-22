#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace isocity {

// Lightweight file hashing utilities.
//
// These hashes are intended for build tooling and regression metadata (e.g.
// artifact manifests). They are *not* cryptographic hashes.

struct FileHashInfo {
  std::uint64_t sizeBytes = 0;
  std::uint64_t fnv1a64 = 0; // FNV-1a 64-bit over the file bytes.
};

// Compute a FNV-1a 64-bit hash of a byte buffer.
//
// seed defaults to the standard FNV-1a 64-bit offset basis.
std::uint64_t Fnv1a64(const void* data, std::size_t size,
                      std::uint64_t seed = 14695981039346656037ull);

// Compute file byte length and a FNV-1a 64-bit hash of its bytes.
// Returns false and fills outErr on failure.
bool ComputeFileHashFNV1a64(const std::string& path, FileHashInfo& out, std::string& outErr);

} // namespace isocity
