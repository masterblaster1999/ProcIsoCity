#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>

namespace isocity {

// Deterministic fixed-point helpers intended for simulation/procedural code.
//
// Motivation:
//  - Transcendental functions (std::sin/cos/...) may produce slightly different
//    results across platforms/standard library implementations.
//  - For systems that feed into hashes/replays, those tiny differences can
//    accumulate or cross a rounding threshold.
//
// These helpers avoid libm calls and use only integer math.
//
// Q16 fixed-point: 1.0 == 65536.
constexpr int kQ16 = 1 << 16;

inline int ClampQ16(int v)
{
  return std::clamp(v, -kQ16, kQ16);
}

inline int PositiveMod(int a, int m)
{
  if (m <= 0) return 0;
  int r = a % m;
  if (r < 0) r += m;
  return r;
}

// Triangle wave in Q16, range [-1, +1].
//
// - day: non-negative time index (days)
// - periodDays: >= 2 (values <2 are clamped)
// - phaseDays: shift in days
inline int TriangleWaveQ16(int day, int periodDays, int phaseDays = 0)
{
  const int p = std::max(2, periodDays);
  const int t = PositiveMod(day + phaseDays, p);

  const int half = p / 2;
  const int denomUp = std::max(1, half);
  const int denomDown = std::max(1, p - half);

  int up = 0; // 0..kQ16
  if (t < half) {
    up = (t * kQ16) / denomUp;
  } else {
    up = ((p - t) * kQ16) / denomDown;
  }

  const int tri = 2 * up - kQ16;
  return ClampQ16(tri);
}

// Convert a triangle wave (Q16) into a smoother "pseudo-sine" wave (Q16).
//
// Uses a deterministic parabolic smoothing:
//   y = x * (2 - |x|)   for x in [-1,1]
//
// This is not a mathematically exact sine, but it is smooth-ish, fast, and
// importantly deterministic without libm.
inline int ParabolicSineQ16FromTriangle(int triQ16)
{
  const int x = ClampQ16(triQ16);
  const int ax = std::abs(x);

  const std::int64_t y = (static_cast<std::int64_t>(x) *
                          (static_cast<std::int64_t>(2 * kQ16) - static_cast<std::int64_t>(ax))) /
                         static_cast<std::int64_t>(kQ16);

  return ClampQ16(static_cast<int>(y));
}

inline int PseudoSineWaveQ16(int day, int periodDays, int phaseDays = 0)
{
  return ParabolicSineQ16FromTriangle(TriangleWaveQ16(day, periodDays, phaseDays));
}

inline float Q16ToFloat(int vQ16)
{
  return static_cast<float>(vQ16) / static_cast<float>(kQ16);
}

// Deterministic noise from a uint32, in Q16 range [-1, +1].
inline int NoiseQ16FromU32(std::uint32_t u)
{
  // Use 16 low bits so the mapping is stable and easy to reason about.
  const int raw = static_cast<int>(u & 0xFFFFu) - 32768; // [-32768, 32767]
  const int q16 = raw * 2;                               // [-65536, 65534]
  return ClampQ16(q16);
}


// -----------------------------------------------------------------------------------------------
// Deterministic trig approximations (no std::sin/cos).
//
// NOTE:
//  - These are intended for procedural/simulation code paths where bitwise determinism
//    across platforms matters more than perfect trig accuracy.
//  - They avoid libm trig calls; only basic arithmetic, abs, and a small amount of
//    range reduction are used.
//
// The approximation used here is a well-known fast sine fit:
//   y = Bx + Cx|x|
//   y = P*(y|y| - y) + y
// for x in [-pi, pi].
//
// It is continuous and reasonably accurate for animation, procedural patterns,
// and mild geometric perturbations.
// -----------------------------------------------------------------------------------------------

constexpr float kPiF = 3.14159265358979323846f;
constexpr float kTwoPiF = 6.28318530717958647692f;
constexpr float kHalfPiF = 1.57079632679489661923f;
constexpr float kInvTwoPiF = 0.15915494309189533577f; // 1/(2*pi)

inline float AbsF(float x) { return (x < 0.0f) ? -x : x; }

// Deterministic floor-to-int for finite inputs in a modest range.
// Avoids std::floor() so we don't depend on libm semantics.
inline int FloorToInt(float x)
{
  int i = static_cast<int>(x); // trunc toward 0
  if (x < 0.0f && static_cast<float>(i) != x) {
    // For negative non-integers, truncation acts like ceil; subtract 1 for floor.
    i -= 1;
  }
  return i;
}


// Deterministic round-to-int for finite inputs in a modest range.
//
// This avoids std::round/std::lround so we don't depend on libm semantics.
inline int RoundToInt(float x)
{
  if (x >= 0.0f) return FloorToInt(x + 0.5f);
  // For negative numbers, mirror the positive path.
  return -FloorToInt((-x) + 0.5f);
}

// Convert a clamped [0,1] float to Q16 (0..kQ16) with deterministic rounding.
// NaNs map to 0.
inline int Float01ToQ16(float x)
{
  if (!(x == x)) return 0;
  if (x <= 0.0f) return 0;
  if (x >= 1.0f) return kQ16;
  return std::clamp(RoundToInt(x * static_cast<float>(kQ16)), 0, kQ16);
}


// Wrap a scalar into [0, 1).
inline float Wrap01(float x)
{
  const int k = FloorToInt(x);
  float f = x - static_cast<float>(k);
  if (f < 0.0f) f += 1.0f;
  // Rare floating rounding can produce 1.0 exactly.
  if (f >= 1.0f) f = 0.0f;
  return f;
}

// Wrap an angle in radians into [-pi, pi].
inline float WrapAnglePi(float rad)
{
  const float turns = rad * kInvTwoPiF;
  const int k = FloorToInt(turns);
  float frac = turns - static_cast<float>(k); // [0,1) for positive, (-1,0] for negative
  if (frac < 0.0f) frac += 1.0f;

  float a = frac * kTwoPiF; // [0,2pi)
  if (a > kPiF) a -= kTwoPiF;
  return a;
}

inline float FastSinWrapped(float xWrapped)
{
  // xWrapped must be in [-pi, pi].
  constexpr float B = 4.0f / kPiF;
  constexpr float C = -4.0f / (kPiF * kPiF);

  float y = B * xWrapped + C * xWrapped * AbsF(xWrapped);

  // Improve peak accuracy.
  constexpr float P = 0.225f;
  y = P * (y * AbsF(y) - y) + y;
  return y;
}

inline float FastSinRad(float rad) { return FastSinWrapped(WrapAnglePi(rad)); }

inline float FastCosRad(float rad)
{
  // cos(x) = sin(x + pi/2)
  float x = WrapAnglePi(rad);
  x += kHalfPiF;
  if (x > kPiF) x -= kTwoPiF;
  return FastSinWrapped(x);
}

inline void FastSinCosRad(float rad, float& outSin, float& outCos)
{
  const float x = WrapAnglePi(rad);
  outSin = FastSinWrapped(x);

  float c = x + kHalfPiF;
  if (c > kPiF) c -= kTwoPiF;
  outCos = FastSinWrapped(c);
}

// Triangle wave from a normalized phase in Q16, where 0..kQ16 maps to one full cycle.
// phaseQ16 may be any integer; it is wrapped internally.
inline int TriangleWaveQ16FromPhase(int phaseQ16)
{
  const int p = PositiveMod(phaseQ16, kQ16);
  const int half = kQ16 / 2;

  int tri = 0;
  if (p < half) {
    // -1 -> +1 over [0, 0.5)
    tri = -kQ16 + 4 * p;
  } else {
    // +1 -> -1 over [0.5, 1)
    tri = 3 * kQ16 - 4 * p;
  }
  return ClampQ16(tri);
}

inline int PseudoSineQ16FromPhase(int phaseQ16)
{
  return ParabolicSineQ16FromTriangle(TriangleWaveQ16FromPhase(phaseQ16));
}

inline void PseudoSinCosQ16FromPhase(int phaseQ16, int& outSinQ16, int& outCosQ16)
{
  outSinQ16 = PseudoSineQ16FromPhase(phaseQ16);
  outCosQ16 = PseudoSineQ16FromPhase(phaseQ16 + (kQ16 / 4));
}

} // namespace isocity
