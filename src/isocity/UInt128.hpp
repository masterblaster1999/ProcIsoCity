#pragma once

#include <cstdint>

namespace isocity {

// Minimal, portable unsigned 128-bit integer.
//
// Rationale:
// - GCC/Clang support __int128, but MSVC does not.
// - ProcIsoCity aims to keep the core library (and CLI tools) compiler/OS agnostic.
//
// This type intentionally implements only the small set of operations we need
// for deterministic fixed-point math in the core library.
struct UInt128 {
  std::uint64_t lo = 0;
  std::uint64_t hi = 0;

  constexpr UInt128() = default;
  constexpr explicit UInt128(std::uint64_t v) : lo(v), hi(0) {}
  constexpr UInt128(std::uint64_t lo_, std::uint64_t hi_) : lo(lo_), hi(hi_) {}

  static constexpr UInt128 FromU64(std::uint64_t v) { return UInt128(v); }

  constexpr bool operator==(const UInt128& o) const { return lo == o.lo && hi == o.hi; }
  constexpr bool operator!=(const UInt128& o) const { return !(*this == o); }

  constexpr bool operator<(const UInt128& o) const
  {
    return (hi < o.hi) || (hi == o.hi && lo < o.lo);
  }
  constexpr bool operator>=(const UInt128& o) const { return !(*this < o); }

  // Multiply by a 32-bit value (m). The caller is responsible for ensuring the
  // true mathematical product fits within 128 bits.
  constexpr void MulU32(std::uint32_t m)
  {
    const std::uint64_t a0 = lo & 0xFFFFFFFFULL;
    const std::uint64_t a1 = lo >> 32;
    const std::uint64_t a2 = hi & 0xFFFFFFFFULL;
    const std::uint64_t a3 = hi >> 32;

    std::uint64_t carry = 0;

    const std::uint64_t p0 = a0 * static_cast<std::uint64_t>(m);
    const std::uint32_t r0 = static_cast<std::uint32_t>(p0 & 0xFFFFFFFFULL);
    carry = p0 >> 32;

    const std::uint64_t p1 = a1 * static_cast<std::uint64_t>(m) + carry;
    const std::uint32_t r1 = static_cast<std::uint32_t>(p1 & 0xFFFFFFFFULL);
    carry = p1 >> 32;

    const std::uint64_t p2 = a2 * static_cast<std::uint64_t>(m) + carry;
    const std::uint32_t r2 = static_cast<std::uint32_t>(p2 & 0xFFFFFFFFULL);
    carry = p2 >> 32;

    const std::uint64_t p3 = a3 * static_cast<std::uint64_t>(m) + carry;
    const std::uint32_t r3 = static_cast<std::uint32_t>(p3 & 0xFFFFFFFFULL);

    lo = static_cast<std::uint64_t>(r0) | (static_cast<std::uint64_t>(r1) << 32);
    hi = static_cast<std::uint64_t>(r2) | (static_cast<std::uint64_t>(r3) << 32);
  }

  // Multiply by 2.
  constexpr void Mul2()
  {
    hi = (hi << 1) | (lo >> 63);
    lo <<= 1;
  }

  // Add a 64-bit value.
  constexpr void AddU64(std::uint64_t v)
  {
    const std::uint64_t old = lo;
    lo += v;
    if (lo < old) {
      // carry
      ++hi;
    }
  }

  // Subtract another 128-bit value (assumes *this >= v).
  constexpr void Sub(const UInt128& v)
  {
    const std::uint64_t old = lo;
    lo -= v.lo;
    const std::uint64_t borrow = (old < v.lo) ? 1ULL : 0ULL;
    hi -= v.hi + borrow;
  }
};

} // namespace isocity
