#include "isocity/LandUseMix.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// Inclusive rectangle sum on an integral image built with a 1-tile border.
// pref has size (w+1)*(h+1). Rectangle coords are in tile space [0,w-1],[0,h-1].
inline int SumRectInclusive(const std::vector<int>& pref, int w, int h, int x0, int y0, int x1, int y1)
{
  if (w <= 0 || h <= 0) return 0;
  x0 = std::clamp(x0, 0, w - 1);
  y0 = std::clamp(y0, 0, h - 1);
  x1 = std::clamp(x1, 0, w - 1);
  y1 = std::clamp(y1, 0, h - 1);
  if (x1 < x0 || y1 < y0) return 0;

  const int W1 = w + 1;
  auto idx = [&](int x, int y) -> std::size_t {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(W1) + static_cast<std::size_t>(x);
  };

  const int xa = x0;
  const int ya = y0;
  const int xb = x1 + 1;
  const int yb = y1 + 1;

  // Standard summed-area table rectangle query.
  const int A = pref[idx(xa, ya)];
  const int B = pref[idx(xb, ya)];
  const int C = pref[idx(xa, yb)];
  const int D = pref[idx(xb, yb)];
  return D - B - C + A;
}

} // namespace

LandUseMixResult ComputeLandUseMix(const World& world, const LandUseMixConfig& cfg)
{
  LandUseMixResult out{};
  out.w = world.width();
  out.h = world.height();
  out.radius = std::max(0, cfg.radius);

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  // Categories:
  //   0: residential
  //   1: commercial
  //   2: industrial
  //   3: park (optional)
  //   4: civic (optional)
  int K = 3;
  const int parkIdx = cfg.includeParks ? K++ : -1;
  const int civicIdx = cfg.includeCivic ? K++ : -1;
  out.categories = K;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.mix01.assign(n, 0.0f);
  out.density01.assign(n, 0.0f);

  // Build integral images for each category.
  const int W1 = w + 1;
  const int H1 = h + 1;
  const std::size_t prefN = static_cast<std::size_t>(W1) * static_cast<std::size_t>(H1);
  std::vector<std::vector<int>> pref(static_cast<std::size_t>(K), std::vector<int>(prefN, 0));

  auto prefIdx = [&](int x, int y) -> std::size_t {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(W1) + static_cast<std::size_t>(x);
  };

  for (int y = 1; y <= h; ++y) {
    for (int x = 1; x <= w; ++x) {
      const Tile& t = world.at(x - 1, y - 1);

      int cat = -1;
      switch (t.overlay) {
      case Overlay::Residential: cat = 0; break;
      case Overlay::Commercial: cat = 1; break;
      case Overlay::Industrial: cat = 2; break;
      case Overlay::Park:
        if (parkIdx >= 0) cat = parkIdx;
        break;
      case Overlay::School:
      case Overlay::Hospital:
      case Overlay::PoliceStation:
      case Overlay::FireStation:
        if (civicIdx >= 0) cat = civicIdx;
        break;
      default:
        break;
      }

      const std::size_t i = prefIdx(x, y);
      const std::size_t iL = prefIdx(x - 1, y);
      const std::size_t iU = prefIdx(x, y - 1);
      const std::size_t iUL = prefIdx(x - 1, y - 1);

      for (int k = 0; k < K; ++k) {
        const int v = (k == cat) ? 1 : 0;
        pref[static_cast<std::size_t>(k)][i] = v + pref[static_cast<std::size_t>(k)][iL] + pref[static_cast<std::size_t>(k)][iU] -
                                              pref[static_cast<std::size_t>(k)][iUL];
      }
    }
  }

  const double logK = (K > 1) ? std::log(static_cast<double>(K)) : 1.0;

  // Compute per-tile mix.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int x0 = x - out.radius;
      const int y0 = y - out.radius;
      const int x1 = x + out.radius;
      const int y1 = y + out.radius;

      int counts[5] = {0, 0, 0, 0, 0};
      int total = 0;
      for (int k = 0; k < K; ++k) {
        const int c = SumRectInclusive(pref[static_cast<std::size_t>(k)], w, h, x0, y0, x1, y1);
        counts[k] = c;
        total += c;
      }

      const int ax0 = std::clamp(x0, 0, w - 1);
      const int ay0 = std::clamp(y0, 0, h - 1);
      const int ax1 = std::clamp(x1, 0, w - 1);
      const int ay1 = std::clamp(y1, 0, h - 1);
      const int area = (ax1 - ax0 + 1) * (ay1 - ay0 + 1);

      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);

      if (total <= 0 || area <= 0 || K <= 1) {
        out.mix01[idx] = 0.0f;
        out.density01[idx] = 0.0f;
        continue;
      }

      const float density = Clamp01(static_cast<float>(total) / static_cast<float>(area));
      out.density01[idx] = density;

      double entropy = 0.0;
      for (int k = 0; k < K; ++k) {
        const int c = counts[k];
        if (c <= 0) continue;
        const double p = static_cast<double>(c) / static_cast<double>(total);
        // p*log(p) is well-defined for p>0.
        entropy -= p * std::log(p);
      }

      double e01 = (logK > 1e-12) ? (entropy / logK) : 0.0;
      e01 = std::clamp(e01, 0.0, 1.0);

      if (cfg.applyDensityWeight) {
        const double wgt = std::sqrt(static_cast<double>(density));
        e01 *= wgt;
      }

      const float mix = static_cast<float>(std::clamp(e01, 0.0, 1.0));
      out.mix01[idx] = mix;
      out.maxMix = std::max(out.maxMix, mix);
    }
  }

  return out;
}

} // namespace isocity
