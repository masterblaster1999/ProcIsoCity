#include "isocity/FileHash.hpp"

#include <fstream>

namespace isocity {

static constexpr std::uint64_t kFnv1a64OffsetBasis = 14695981039346656037ull;
static constexpr std::uint64_t kFnv1a64Prime = 1099511628211ull;

std::uint64_t Fnv1a64(const void* data, std::size_t size, std::uint64_t seed)
{
  if (!data || size == 0) return seed;
  const auto* bytes = static_cast<const unsigned char*>(data);

  std::uint64_t h = seed;
  for (std::size_t i = 0; i < size; ++i) {
    h ^= static_cast<std::uint64_t>(bytes[i]);
    h *= kFnv1a64Prime;
  }
  return h;
}

bool ComputeFileHashFNV1a64(const std::string& path, FileHashInfo& out, std::string& outErr)
{
  out = FileHashInfo{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outErr = "failed to open file";
    return false;
  }

  std::uint64_t sizeBytes = 0;
  std::uint64_t h = kFnv1a64OffsetBasis;

  char buf[64 * 1024];
  while (f) {
    f.read(buf, static_cast<std::streamsize>(sizeof(buf)));
    const std::streamsize n = f.gcount();
    if (n > 0) {
      sizeBytes += static_cast<std::uint64_t>(n);
      h = Fnv1a64(buf, static_cast<std::size_t>(n), h);
    }
  }

  // If the loop terminated due to an error other than EOF, report it.
  if (!f.eof()) {
    outErr = "failed while reading file";
    return false;
  }

  out.sizeBytes = sizeBytes;
  out.fnv1a64 = h;
  outErr.clear();
  return true;
}

} // namespace isocity
