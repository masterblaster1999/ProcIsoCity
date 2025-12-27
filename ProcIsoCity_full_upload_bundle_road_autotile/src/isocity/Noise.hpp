#pragma once

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace isocity {

inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }

inline float SmoothStep(float t) { return t * t * (3.0f - 2.0f * t); }

// Hash an integer grid point to [0, 1].
inline float Hash01(int ix, int iy, std::uint32_t seed)
{
  const std::uint32_t h = HashCoords32(ix, iy, seed);
  return static_cast<float>(h) / static_cast<float>(std::numeric_limits<std::uint32_t>::max());
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

} // namespace isocity
