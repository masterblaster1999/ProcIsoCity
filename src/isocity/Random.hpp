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
  // Use the SplitMix64 output as the seed; the internal state increment alone
  // does not provide the mixing we want here.
  return SplitMix64Next(s);
}

struct RNG {
  std::uint64_t state = 0;

  explicit RNG(std::uint64_t seed)
      : state(seed ? seed : 0x12345678ABCDEF00ULL)
  {}

  std::uint64_t nextU64() { return SplitMix64Next(state); }

  std::uint32_t nextU32() { return static_cast<std::uint32_t>(nextU64() >> 32); }

  // Uniform integer in [0, maxExclusive).
  // Uses rejection sampling to avoid modulo bias for arbitrary bounds.
  std::uint32_t rangeU32(std::uint32_t maxExclusive)
  {
    if (maxExclusive <= 1u) return 0u;

    // Power-of-two fast path: modulo == bitmask. Keeps results identical while
    // avoiding a division.
    if ((maxExclusive & (maxExclusive - 1u)) == 0u) {
      return nextU32() & (maxExclusive - 1u);
    }

    // threshold == 2^32 % maxExclusive.
    // Compute in 64-bit to avoid MSVC warning C4146 (unary minus on unsigned).
    const std::uint32_t threshold = static_cast<std::uint32_t>((std::uint64_t{1} << 32) % maxExclusive);
    while (true) {
      const std::uint32_t r = nextU32();
      if (r >= threshold) return r % maxExclusive;
    }
  }

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

    const std::int64_t lo = static_cast<std::int64_t>(minInclusive);
    const std::int64_t hi = static_cast<std::int64_t>(maxInclusive);
    const std::int64_t span = hi - lo + 1;

    // Overflow guard (shouldn't happen for normal int ranges, but keep it robust).
    if (span <= 0) return minInclusive;

    // Fast path for the full 32-bit range (span == 2^32).
    // Avoids unsigned->signed implementation-defined casts.
    if (span == (std::int64_t{1} << 32)) {
      const std::int64_t r = static_cast<std::int64_t>(nextU32());
      return static_cast<int>(lo + r);
    }

    // Most uses have small spans; use rejection sampling to avoid modulo bias.
    if (span <= static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
      const std::uint32_t uSpan = static_cast<std::uint32_t>(span);
      const std::uint32_t r = rangeU32(uSpan);
      return static_cast<int>(lo + static_cast<std::int64_t>(r));
    }

    // Fallback (span > 2^32): use 64-bit modulo. This is extremely unlikely for int,
    // but keeps the function well-defined if it ever happens.
    return static_cast<int>(lo + static_cast<std::int64_t>(nextU64() % static_cast<std::uint64_t>(span)));
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
