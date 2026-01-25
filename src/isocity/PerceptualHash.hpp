#pragma once

// PerceptualHash.hpp
//
// Tiny, dependency-free perceptual hashing helpers.
//
// This is a pragmatic implementation of a classic pHash-style pipeline:
//   1) Downsample to a small grayscale "image" (default 32x32)
//   2) Compute a low-frequency 2D DCT (default 8x8)
//   3) Threshold coefficients against the median to build a 64-bit hash
//
// The goal in ProcIsoCity is not cryptographic security; it is a compact,
// deterministic "layout signature" that enables fast similarity/distance
// comparisons between procedural cities (e.g., for diversity selection in seed
// mining).

#include "isocity/World.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

struct PHashOptions {
  // Size of the internal downsample buffer.
  int downW = 32;
  int downH = 32;

  // DCT low-frequency block size.
  // 8 => classic 64-bit pHash from 8x8 coefficients.
  int dctSize = 8;
};

inline int HammingDistance64(std::uint64_t a, std::uint64_t b)
{
  return static_cast<int>(std::popcount(a ^ b));
}

namespace detail {

template <typename SampleFn>
inline double SampleBilinear(int srcW, int srcH, const SampleFn& sample, double x, double y)
{
  if (srcW <= 0 || srcH <= 0) return 0.0;

  const double maxX = static_cast<double>(srcW - 1);
  const double maxY = static_cast<double>(srcH - 1);

  x = std::clamp(x, 0.0, maxX);
  y = std::clamp(y, 0.0, maxY);

  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = std::min(x0 + 1, srcW - 1);
  const int y1 = std::min(y0 + 1, srcH - 1);

  const double tx = x - static_cast<double>(x0);
  const double ty = y - static_cast<double>(y0);

  const double a = static_cast<double>(sample(x0, y0));
  const double b = static_cast<double>(sample(x1, y0));
  const double c = static_cast<double>(sample(x0, y1));
  const double d = static_cast<double>(sample(x1, y1));

  const double ab = a + (b - a) * tx;
  const double cd = c + (d - c) * tx;
  return ab + (cd - ab) * ty;
}

inline double MedianOf(std::vector<double> v)
{
  if (v.empty()) return 0.0;
  const std::size_t n = v.size();
  const std::size_t mid = n / 2;

  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  double m = v[mid];

  // If even, average the two middle values.
  if ((n & 1u) == 0u) {
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid - 1), v.end());
    m = 0.5 * (m + v[mid - 1]);
  }
  return m;
}

} // namespace detail

// Compute a 64-bit perceptual hash over an implicit grayscale image via a
// caller-provided sampler.
//
// The sampler must be callable as: float sample(int x, int y)
//
// srcW/srcH describe the sampler's coordinate system: x in [0,srcW), y in [0,srcH).
// The algorithm bilinearly samples the source when downsampling.
template <typename SampleFn>
inline std::uint64_t ComputePHashSample(int srcW, int srcH, const SampleFn& sample, PHashOptions opt = {})
{
  if (srcW <= 0 || srcH <= 0) return 0;
  if (opt.downW <= 0 || opt.downH <= 0) return 0;

  const int downW = opt.downW;
  const int downH = opt.downH;

  int dctN = opt.dctSize;
  dctN = std::max(1, dctN);
  dctN = std::min(dctN, downW);
  dctN = std::min(dctN, downH);

  // Downsample with bilinear sampling.
  std::vector<double> down;
  down.resize(static_cast<std::size_t>(downW) * static_cast<std::size_t>(downH));

  for (int y = 0; y < downH; ++y) {
    const double sy = (static_cast<double>(y) + 0.5) * (static_cast<double>(srcH) / static_cast<double>(downH)) - 0.5;
    for (int x = 0; x < downW; ++x) {
      const double sx =
          (static_cast<double>(x) + 0.5) * (static_cast<double>(srcW) / static_cast<double>(downW)) - 0.5;
      down[static_cast<std::size_t>(y) * static_cast<std::size_t>(downW) + static_cast<std::size_t>(x)] =
          detail::SampleBilinear(srcW, srcH, sample, sx, sy);
    }
  }

  // Precompute cosine tables for low frequencies.
  constexpr double kPi = 3.141592653589793238462643383279502884;
  std::vector<double> cosX;
  std::vector<double> cosY;
  cosX.resize(static_cast<std::size_t>(dctN) * static_cast<std::size_t>(downW));
  cosY.resize(static_cast<std::size_t>(dctN) * static_cast<std::size_t>(downH));

  for (int u = 0; u < dctN; ++u) {
    for (int x = 0; x < downW; ++x) {
      const double a = (kPi * (2.0 * static_cast<double>(x) + 1.0) * static_cast<double>(u)) /
                       (2.0 * static_cast<double>(downW));
      cosX[static_cast<std::size_t>(u) * static_cast<std::size_t>(downW) + static_cast<std::size_t>(x)] = std::cos(a);
    }
    for (int y = 0; y < downH; ++y) {
      const double a = (kPi * (2.0 * static_cast<double>(y) + 1.0) * static_cast<double>(u)) /
                       (2.0 * static_cast<double>(downH));
      cosY[static_cast<std::size_t>(u) * static_cast<std::size_t>(downH) + static_cast<std::size_t>(y)] = std::cos(a);
    }
  }

  auto alpha = [](int k, int n) -> double {
    const double dn = static_cast<double>(n);
    return (k == 0) ? std::sqrt(1.0 / dn) : std::sqrt(2.0 / dn);
  };

  // Compute low-frequency DCT block.
  std::vector<double> coeff;
  coeff.resize(static_cast<std::size_t>(dctN) * static_cast<std::size_t>(dctN));

  for (int u = 0; u < dctN; ++u) {
    const double au = alpha(u, downW);
    for (int v = 0; v < dctN; ++v) {
      const double av = alpha(v, downH);

      double sum = 0.0;
      for (int y = 0; y < downH; ++y) {
        const double cy = cosY[static_cast<std::size_t>(v) * static_cast<std::size_t>(downH) + static_cast<std::size_t>(y)];
        const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(downW);
        for (int x = 0; x < downW; ++x) {
          const double cx =
              cosX[static_cast<std::size_t>(u) * static_cast<std::size_t>(downW) + static_cast<std::size_t>(x)];
          sum += down[row + static_cast<std::size_t>(x)] * cx * cy;
        }
      }

      coeff[static_cast<std::size_t>(u) * static_cast<std::size_t>(dctN) + static_cast<std::size_t>(v)] =
          au * av * sum;
    }
  }

  // Median excluding DC (0,0), but we still emit a DC bit for a full 64-bit hash.
  std::vector<double> coeffNoDc;
  coeffNoDc.reserve(static_cast<std::size_t>(dctN) * static_cast<std::size_t>(dctN) - 1u);
  for (int u = 0; u < dctN; ++u) {
    for (int v = 0; v < dctN; ++v) {
      if (u == 0 && v == 0) continue;
      coeffNoDc.push_back(coeff[static_cast<std::size_t>(u) * static_cast<std::size_t>(dctN) + static_cast<std::size_t>(v)]);
    }
  }

  const double med = detail::MedianOf(std::move(coeffNoDc));

  // Emit bits in row-major order of the DCT block.
  // If dctN < 8, the remaining high bits stay 0.
  std::uint64_t out = 0;
  const int maxBits = std::min(64, dctN * dctN);
  for (int i = 0; i < maxBits; ++i) {
    const double c = coeff[static_cast<std::size_t>(i)];
    if (c > med) out |= (1ull << static_cast<unsigned>(i));
  }
  return out;
}

// ----------------------------------------------------------------------------
// ProcIsoCity helpers (world -> grayscale mapping)
// ----------------------------------------------------------------------------

inline float TilePHashIntensity(const Tile& t)
{
  // Base terrain.
  float v = 0.0f;
  switch (t.terrain) {
  case Terrain::Water: v = 0.0f; break;
  case Terrain::Sand: v = 30.0f; break;
  case Terrain::Grass: v = 60.0f; break;
  }

  // Overlays override the visual "mass" in the signature.
  switch (t.overlay) {
  case Overlay::None: break;
  case Overlay::Park: v = 95.0f; break;
  case Overlay::Road: v = 130.0f + 8.0f * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;
  case Overlay::Residential: v = 175.0f + 12.0f * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;
  case Overlay::Commercial: v = 195.0f + 12.0f * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;
  case Overlay::Industrial: v = 215.0f + 12.0f * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;

  // Civic buildings are treated as strong anchors in the layout.
  case Overlay::School:
  case Overlay::Hospital:
  case Overlay::PoliceStation:
  case Overlay::FireStation: v = 245.0f; break;
  }

  // Lightly inject relief to distinguish similar layouts on different terrain.
  const float h = std::clamp(t.height, 0.0f, 1.0f);
  v += h * 10.0f;

  return v;
}

inline std::uint64_t ComputeWorldOverlayPHash(const World& world, PHashOptions opt = {})
{
  const int w = world.width();
  const int h = world.height();
  return ComputePHashSample(w, h, [&](int x, int y) -> float {
    return TilePHashIntensity(world.at(x, y));
  }, opt);
}

} // namespace isocity
