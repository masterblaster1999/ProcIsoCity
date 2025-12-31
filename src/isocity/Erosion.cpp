#include "isocity/Erosion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace isocity {

namespace {

static inline std::size_t idx(int x, int y, int w) { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x); }

static void ApplyThermal(std::vector<float>& h, int w, int hgt, int iters, float talus, float rate)
{
  if (w <= 0 || hgt <= 0) return;
  if (iters <= 0) return;

  talus = std::max(0.0f, talus);
  rate = std::clamp(rate, 0.0f, 1.0f);

  std::vector<float> delta(h.size(), 0.0f);

  constexpr int dx4[4] = {1, -1, 0, 0};
  constexpr int dy4[4] = {0, 0, 1, -1};

  for (int iter = 0; iter < iters; ++iter) {
    std::fill(delta.begin(), delta.end(), 0.0f);

    for (int y = 0; y < hgt; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = idx(x, y, w);
        const float cur = h[i];

        // Find lowest neighbor.
        float minH = cur;
        int minX = x;
        int minY = y;

        for (int k = 0; k < 4; ++k) {
          const int nx = x + dx4[k];
          const int ny = y + dy4[k];
          if (nx < 0 || nx >= w || ny < 0 || ny >= hgt) continue;
          const float nh = h[idx(nx, ny, w)];
          if (nh < minH) {
            minH = nh;
            minX = nx;
            minY = ny;
          }
        }

        const float diff = cur - minH;
        if (diff <= talus) continue;

        const float move = rate * (diff - talus);
        if (move <= 0.0f) continue;

        delta[i] -= move;
        delta[idx(minX, minY, w)] += move;
      }
    }

    for (std::size_t i = 0; i < h.size(); ++i) {
      h[i] += delta[i];
    }
  }
}

static void ComputeFlowDir(const std::vector<float>& h, int w, int hgt, std::vector<int>& outDir)
{
  outDir.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(hgt), -1);
  if (w <= 0 || hgt <= 0) return;

  // 4-neighborhood for simplicity.
  constexpr int dx4[4] = {1, -1, 0, 0};
  constexpr int dy4[4] = {0, 0, 1, -1};

  for (int y = 0; y < hgt; ++y) {
    for (int x = 0; x < w; ++x) {
      const float cur = h[idx(x, y, w)];
      float bestH = cur;
      int best = -1;

      // Deterministic tie-break: check neighbors in fixed order.
      for (int k = 0; k < 4; ++k) {
        const int nx = x + dx4[k];
        const int ny = y + dy4[k];
        if (nx < 0 || nx >= w || ny < 0 || ny >= hgt) continue;
        const float nh = h[idx(nx, ny, w)];
        if (nh < bestH) {
          bestH = nh;
          best = ny * w + nx;
        }
      }

      // If no strictly-lower neighbor, keep as sink.
      outDir[idx(x, y, w)] = best;
    }
  }
}

static void FlowAccumulation(const std::vector<float>& h, int w, int hgt,
                             const std::vector<int>& dir, std::vector<int>& outAccum)
{
  outAccum.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(hgt), 1);
  if (w <= 0 || hgt <= 0) return;

  // Sort indices by height descending (higher contributes to lower).
  std::vector<int> order;
  order.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(hgt));
  for (int i = 0; i < w * hgt; ++i) order.push_back(i);

  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const float ha = h[static_cast<std::size_t>(a)];
    const float hb = h[static_cast<std::size_t>(b)];
    if (ha == hb) return a < b;
    return ha > hb;
  });

  for (int i : order) {
    const int to = dir[static_cast<std::size_t>(i)];
    if (to < 0) continue;
    outAccum[static_cast<std::size_t>(to)] += outAccum[static_cast<std::size_t>(i)];
  }
}

static void ApplyRivers(std::vector<float>& h, int w, int hgt, const ErosionConfig& cfg)
{
  if (w <= 0 || hgt <= 0) return;

  std::vector<int> dir;
  ComputeFlowDir(h, w, hgt, dir);

  std::vector<int> accum;
  FlowAccumulation(h, w, hgt, dir, accum);

  int maxA = 1;
  for (int a : accum) maxA = std::max(maxA, a);

  int minA = cfg.riverMinAccum;
  if (minA <= 0) {
    // Auto threshold: scales with map area but never too small.
    const int area = w * hgt;
    minA = std::max(32, area / 64);
  }
  minA = std::max(2, minA);

  const float carve = std::max(0.0f, cfg.riverCarve);
  const float power = std::max(0.01f, cfg.riverCarvePower);

  if (carve <= 0.0f) return;

  for (int y = 0; y < hgt; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = idx(x, y, w);
      const int a = accum[i];
      if (a < minA) continue;

      float t = 1.0f;
      if (maxA > minA) {
        t = static_cast<float>(a - minA) / static_cast<float>(maxA - minA);
        t = std::clamp(t, 0.0f, 1.0f);
      }
      const float amt = carve * std::pow(t, power);
      h[i] -= amt;
    }
  }
}

static void ApplySmoothing(std::vector<float>& h, int w, int hgt, int iters, float rate)
{
  if (w <= 0 || hgt <= 0) return;
  if (iters <= 0) return;
  rate = std::clamp(rate, 0.0f, 1.0f);
  if (rate <= 0.0f) return;

  std::vector<float> tmp(h.size(), 0.0f);

  constexpr int dx4[4] = {1, -1, 0, 0};
  constexpr int dy4[4] = {0, 0, 1, -1};

  for (int iter = 0; iter < iters; ++iter) {
    for (int y = 0; y < hgt; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = idx(x, y, w);
        float sum = h[i];
        int cnt = 1;
        for (int k = 0; k < 4; ++k) {
          const int nx = x + dx4[k];
          const int ny = y + dy4[k];
          if (nx < 0 || nx >= w || ny < 0 || ny >= hgt) continue;
          sum += h[idx(nx, ny, w)];
          cnt++;
        }
        const float avg = sum / static_cast<float>(cnt);
        tmp[i] = h[i] + (avg - h[i]) * rate;
      }
    }
    h.swap(tmp);
  }
}

static void ApplyQuantize(std::vector<float>& h, int scale)
{
  if (scale <= 0) return;
  const float s = static_cast<float>(scale);
  for (float& v : h) {
    v = std::round(v * s) / s;
  }
}

} // namespace

void ApplyErosion(std::vector<float>& heights, int w, int h, const ErosionConfig& cfg, std::uint64_t /*seed*/)
{
  if (!cfg.enabled) return;
  if (w <= 0 || h <= 0) return;
  if (static_cast<int>(heights.size()) != w * h) return;

  // Thermal erosion first: redistributes material.
  ApplyThermal(heights, w, h, cfg.thermalIterations, cfg.thermalTalus, cfg.thermalRate);

  // Rivers: carve channels based on flow accumulation.
  if (cfg.riversEnabled) {
    ApplyRivers(heights, w, h, cfg);
  }

  // Light smoothing after carving.
  ApplySmoothing(heights, w, h, cfg.smoothIterations, cfg.smoothRate);

  // Quantization helps keep downstream classification stable.
  ApplyQuantize(heights, cfg.quantizeScale);

  // Clamp to a reasonable range (noise stage already tends to be ~[-0.2, 1]).
  for (float& v : heights) {
    v = std::clamp(v, -1.0f, 2.0f);
  }
}

} // namespace isocity
