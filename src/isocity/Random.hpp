#pragma once

#include <cstdint>
#include <chrono>
#include <limits>

namespace isocity {

// SplitMix64: small, fast, high-quality generator for seeds / hashing.
inline std::uint64_t SplitMix64Next(std::uint64_t& state)
{
  std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

inline std::uint64_t TimeSeed()
{
  const auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::uint64_t s = static_cast<std::uint64_t>(now);
  // One extra mix so the very low bits also get "random-looking".
  (void)SplitMix64Next(s);
  return s;
}

struct RNG {
  std::uint64_t state = 0;

  explicit RNG(std::uint64_t seed)
      : state(seed ? seed : 0x12345678ABCDEF00ULL)
  {}

  std::uint64_t nextU64() { return SplitMix64Next(state); }

  std::uint32_t nextU32() { return static_cast<std::uint32_t>(nextU64() >> 32); }

  // [0, 1)
  float nextF01()
  {
    // 24-bit mantissa for float
    const std::uint32_t u = nextU32() >> 8;
    return static_cast<float>(u) / static_cast<float>(1u << 24);
  }

  // Back-compat alias (older code calls this uniform01).
  float uniform01() { return nextF01(); }

  int rangeInt(int minInclusive, int maxInclusive)
  {
    if (maxInclusive <= minInclusive) return minInclusive;
    const std::uint32_t span = static_cast<std::uint32_t>(maxInclusive - minInclusive + 1);
    return minInclusive + static_cast<int>(nextU32() % span);
  }

  float rangeFloat(float minInclusive, float maxInclusive)
  {
    const float t = nextF01();
    return minInclusive + (maxInclusive - minInclusive) * t;
  }

  bool chance(float p) { return nextF01() < p; }
};

// Deterministic 2D integer hash -> uint32.
// Useful for per-tile variation or noise seed material.
inline std::uint32_t HashCoords32(int x, int y, std::uint32_t seed)
{
  // Mix x/y into 64 bits (treat as unsigned to avoid UB on shifts).
  std::uint64_t v = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  v |= (static_cast<std::uint64_t>(static_cast<std::uint32_t>(y)) << 32);
  v ^= (static_cast<std::uint64_t>(seed) * 0xD6E8FEB86659FD93ULL);

  // Finalize with splitmix mix steps (without state increment).
  v ^= v >> 30;
  v *= 0xBF58476D1CE4E5B9ULL;
  v ^= v >> 27;
  v *= 0x94D049BB133111EBULL;
  v ^= v >> 31;

  return static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
}

} // namespace isocity
