#pragma once

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace isocity {

inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }

inline float SmoothStep(float t) { return t * t * (3.0f - 2.0f * t); }

// Positive modulo (wrap into [0, m)). Useful for periodic/tileable noise.
inline int PositiveMod(int x, int m)
{
  if (m <= 0) return x;
  int r = x % m;
  if (r < 0) r += m;
  return r;
}

// Hash an integer grid point to [0, 1].
inline float Hash01(int ix, int iy, std::uint32_t seed)
{
  const std::uint32_t h = HashCoords32(ix, iy, seed);
  return static_cast<float>(h) / static_cast<float>(std::numeric_limits<std::uint32_t>::max());
}

// Hash an integer grid point to [0, 1] with explicit wrap periods.
inline float Hash01Periodic(int ix, int iy, std::uint32_t seed, int periodX, int periodY)
{
  if (periodX > 0) ix = PositiveMod(ix, periodX);
  if (periodY > 0) iy = PositiveMod(iy, periodY);
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

// Tileable value noise in [0, 1] using smooth interpolation.
//
// periodX/periodY define the lattice repeat in *integer* grid space.
// To generate a seamless texture of size S, a convenient mapping is:
//   nx = x * periodX / (S - 1)
//   ny = y * periodY / (S - 1)
inline float ValueNoise2DPeriodic(float x, float y, std::uint32_t seed, int periodX, int periodY)
{
  if (periodX <= 0 || periodY <= 0) return ValueNoise2D(x, y, seed);

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

// Periodic/tileable fbm in ~[0, 1] (normalized).
inline float FBm2DPeriodic(float x, float y, std::uint32_t seed, int periodX, int periodY, int octaves = 5,
                           float lacunarity = 2.0f, float gain = 0.5f)
{
  if (periodX <= 0 || periodY <= 0) return FBm2D(x, y, seed, octaves, lacunarity, gain);

  float amp = 1.0f;
  float freq = 1.0f;
  float sum = 0.0f;
  float norm = 0.0f;

  for (int i = 0; i < octaves; ++i) {
    const float xf = x * freq;
    const float yf = y * freq;

    // When we scale input by freq, we must scale the period too to keep the output periodic
    // in the original coordinate system. Using lacunarity=2 keeps this integer-friendly.
    const int px = std::max(1, static_cast<int>(std::lround(static_cast<float>(periodX) * freq)));
    const int py = std::max(1, static_cast<int>(std::lround(static_cast<float>(periodY) * freq)));

    sum += ValueNoise2DPeriodic(xf, yf, seed + static_cast<std::uint32_t>(i * 1013), px, py) * amp;
    norm += amp;
    amp *= gain;
    freq *= lacunarity;
  }

  if (norm > 0.0f) sum /= norm;
  return std::clamp(sum, 0.0f, 1.0f);
}

// Cheap domain-warped periodic fbm (useful for cloud-like patterns).
inline float DomainWarpFBm2DPeriodic(float x, float y, std::uint32_t seed, int periodX, int periodY,
                                    int octaves = 5, float lacunarity = 2.0f, float gain = 0.5f,
                                    float warpStrength = 2.0f)
{
  if (periodX <= 0 || periodY <= 0) {
    const float wx = FBm2D(x + 19.1f, y - 7.7f, seed ^ 0x9E3779B9u, octaves, lacunarity, gain) - 0.5f;
    const float wy = FBm2D(x - 13.4f, y + 11.9f, seed ^ 0xB5297A4Du, octaves, lacunarity, gain) - 0.5f;
    return FBm2D(x + wx * warpStrength, y + wy * warpStrength, seed ^ 0x68E31DA4u, octaves, lacunarity, gain);
  }

  const float wx = FBm2DPeriodic(x + 19.1f, y - 7.7f, seed ^ 0x9E3779B9u, periodX, periodY, octaves,
                                 lacunarity, gain) - 0.5f;
  const float wy = FBm2DPeriodic(x - 13.4f, y + 11.9f, seed ^ 0xB5297A4Du, periodX, periodY, octaves,
                                 lacunarity, gain) - 0.5f;

  return FBm2DPeriodic(x + wx * warpStrength, y + wy * warpStrength, seed ^ 0x68E31DA4u, periodX, periodY,
                       octaves, lacunarity, gain);
}

} // namespace isocity
