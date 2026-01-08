#pragma once

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace isocity {

inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }

inline float SmoothStep(float t) { return t * t * (3.0f - 2.0f * t); }

// Hash an integer grid point to [0, 1].
inline float Hash01(int ix, int iy, std::uint32_t seed)
{
  const std::uint32_t h = HashCoords32(ix, iy, seed);
  return static_cast<float>(h) / static_cast<float>(std::numeric_limits<std::uint32_t>::max());
}

// Wrap i into [0, m-1] (handles negative i). If m <= 0, returns i unchanged.
inline int WrapMod(int i, int m)
{
  if (m <= 0) return i;
  const int r = i % m;
  return (r < 0) ? (r + m) : r;
}

// Hash an integer grid point to [0, 1], with the lattice coordinates wrapped to a periodic domain.
//
// periodX/periodY describe the repeat period in integer lattice units. For example, if periodX=32,
// then Hash01Periodic(ix+32, iy) == Hash01Periodic(ix, iy).
inline float Hash01Periodic(int ix, int iy, std::uint32_t seed, int periodX, int periodY)
{
  if (periodX > 0) ix = WrapMod(ix, periodX);
  if (periodY > 0) iy = WrapMod(iy, periodY);
  return Hash01(ix, iy, seed);
}

// 2D value noise in [0, 1] using smooth interpolation.
inline float ValueNoise2D(float x, float y, std::uint32_t seed)
{
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const float tx = SmoothStep(x - static_cast<float>(x0));
  const float ty = SmoothStep(y - static_cast<float>(y0));

  const float v00 = Hash01(x0, y0, seed);
  const float v10 = Hash01(x1, y0, seed);
  const float v01 = Hash01(x0, y1, seed);
  const float v11 = Hash01(x1, y1, seed);

  const float a = Lerp(v00, v10, tx);
  const float b = Lerp(v01, v11, tx);
  return Lerp(a, b, ty);
}

// 2D value noise in [0, 1] that tiles with periodX/periodY.
//
// This is useful for generating seamless textures (cloud masks, water normals, etc.).
inline float ValueNoise2DPeriodic(float x, float y, std::uint32_t seed, int periodX, int periodY)
{
  if (periodX <= 0 || periodY <= 0) {
    return ValueNoise2D(x, y, seed);
  }

  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const float tx = SmoothStep(x - static_cast<float>(x0));
  const float ty = SmoothStep(y - static_cast<float>(y0));

  const float v00 = Hash01Periodic(x0, y0, seed, periodX, periodY);
  const float v10 = Hash01Periodic(x1, y0, seed, periodX, periodY);
  const float v01 = Hash01Periodic(x0, y1, seed, periodX, periodY);
  const float v11 = Hash01Periodic(x1, y1, seed, periodX, periodY);

  const float a = Lerp(v00, v10, tx);
  const float b = Lerp(v01, v11, tx);
  return Lerp(a, b, ty);
}

// Fractal Brownian Motion (fbm) in ~[0, 1] (normalized).
inline float FBm2D(float x, float y, std::uint32_t seed, int octaves = 5, float lacunarity = 2.0f,
                   float gain = 0.5f)
{
  float amp = 1.0f;
  float freq = 1.0f;
  float sum = 0.0f;
  float norm = 0.0f;

  for (int i = 0; i < octaves; ++i) {
    sum += ValueNoise2D(x * freq, y * freq, seed + static_cast<std::uint32_t>(i * 1013)) * amp;
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }

  if (norm > 0.0f) sum /= norm;
  return std::clamp(sum, 0.0f, 1.0f);
}

// Fractal Brownian Motion (fbm) in ~[0, 1] (normalized) that tiles with the provided periods.
//
// NOTE: The period parameters are in the same units as x/y. Each octave scales both the sample
// coordinates and the period, so the resulting fbm repeats at the *original* periodX/periodY.
inline float FBm2DPeriodic(float x, float y, std::uint32_t seed, int periodX, int periodY, int octaves = 5,
                           float lacunarity = 2.0f, float gain = 0.5f)
{
  if (periodX <= 0 || periodY <= 0) {
    return FBm2D(x, y, seed, octaves, lacunarity, gain);
  }

  float amp = 1.0f;
  float freq = 1.0f;
  float sum = 0.0f;
  float norm = 0.0f;

  for (int i = 0; i < octaves; ++i) {
    const int px = std::max(1, static_cast<int>(std::lround(static_cast<double>(periodX) * static_cast<double>(freq))));
    const int py = std::max(1, static_cast<int>(std::lround(static_cast<double>(periodY) * static_cast<double>(freq))));

    sum += ValueNoise2DPeriodic(x * freq, y * freq, seed + static_cast<std::uint32_t>(i * 1013), px, py) * amp;
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }

  if (norm > 0.0f) sum /= norm;
  return std::clamp(sum, 0.0f, 1.0f);
}

// Periodic domain-warped fbm (tileable).
//
// This uses two lower-octave periodic fbm calls to generate a warp vector, then samples the main
// periodic fbm at the warped coordinates. Because both the warp field and the base field are
// periodic with the same periods, the result tiles seamlessly.
inline float DomainWarpFBm2DPeriodic(float x, float y, std::uint32_t seed, int periodX, int periodY,
                                    int octaves = 5, float lacunarity = 2.0f, float gain = 0.5f,
                                    float warpAmp = 1.0f)
{
  if (periodX <= 0 || periodY <= 0) {
    // Non-periodic fallback: warp still works, but won't tile.
    const float wx = FBm2D(x + 19.37f, y + 47.11f, seed ^ 0x68BC21EBu, 3, lacunarity, gain);
    const float wy = FBm2D(x - 31.17f, y + 11.83f, seed ^ 0x02E5BE93u, 3, lacunarity, gain);
    const float dx = (wx * 2.0f - 1.0f) * warpAmp;
    const float dy = (wy * 2.0f - 1.0f) * warpAmp;
    return FBm2D(x + dx, y + dy, seed, octaves, lacunarity, gain);
  }

  // A small octave count keeps the warp smooth (large-scale flowy shapes).
  constexpr int kWarpOctaves = 3;

  const float wx = FBm2DPeriodic(x + 19.37f, y + 47.11f, seed ^ 0x68BC21EBu, periodX, periodY,
                                kWarpOctaves, lacunarity, gain);
  const float wy = FBm2DPeriodic(x - 31.17f, y + 11.83f, seed ^ 0x02E5BE93u, periodX, periodY,
                                kWarpOctaves, lacunarity, gain);

  const float dx = (wx * 2.0f - 1.0f) * warpAmp;
  const float dy = (wy * 2.0f - 1.0f) * warpAmp;

  return FBm2DPeriodic(x + dx, y + dy, seed, periodX, periodY, octaves, lacunarity, gain);
}

} // namespace isocity
