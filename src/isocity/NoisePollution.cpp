#include "isocity/NoisePollution.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace isocity {
namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

struct OffsetW {
  int dx = 0;
  int dy = 0;
  float w = 0.0f;
};

} // namespace

NoiseResult ComputeNoisePollution(const World& world, const NoiseConfig& cfg,
                                 const TrafficResult* traffic, const GoodsResult* goods)
{
  NoiseResult out{};
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  out.w = w;
  out.h = h;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.noise01.assign(n, 0.0f);

  // --- normalize flows if present ---
  std::uint16_t maxCommute = 0;
  if (traffic && traffic->roadTraffic.size() == n) {
    maxCommute = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (maxCommute == 0) {
      for (std::uint16_t v : traffic->roadTraffic) maxCommute = std::max(maxCommute, v);
    }
  }

  std::uint16_t maxGoods = 0;
  if (goods && goods->roadGoodsTraffic.size() == n) {
    maxGoods = static_cast<std::uint16_t>(std::clamp(goods->maxRoadGoodsTraffic, 0, 65535));
    if (maxGoods == 0) {
      for (std::uint16_t v : goods->roadGoodsTraffic) maxGoods = std::max(maxGoods, v);
    }
  }

  // --- emission field (roads + land-use sources, plus sinks) ---
  std::vector<float> emission(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float e = 0.0f;

      // Sinks.
      if (t.overlay == Overlay::Park) e -= cfg.parkSink;
      if (t.terrain == Terrain::Water) e -= cfg.waterSink;

      // Major land-use sources.
      if (t.overlay == Overlay::Industrial) e += cfg.industrialSource;
      else if (t.overlay == Overlay::Commercial) e += cfg.commercialSource;

      // Road sources.
      if (t.overlay == Overlay::Road) {
        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        e += cfg.roadBase + cfg.roadClassBoost * static_cast<float>(lvl - 1);

        float commute01 = 0.0f;
        if (maxCommute > 0 && traffic && traffic->roadTraffic.size() == n) {
          commute01 = static_cast<float>(traffic->roadTraffic[i]) / static_cast<float>(maxCommute);
        } else {
          // Fallback: a small constant so roads don't look silent without traffic.
          commute01 = 0.20f;
        }
        e += cfg.commuteTrafficBoost * Clamp01(commute01);

        if (maxGoods > 0 && goods && goods->roadGoodsTraffic.size() == n) {
          const float goods01 = static_cast<float>(goods->roadGoodsTraffic[i]) / static_cast<float>(maxGoods);
          e += cfg.goodsTrafficBoost * Clamp01(goods01);
        }
      }

      // Clamp intermediate emission so sinks don't dominate too hard.
      emission[i] = std::clamp(e, -cfg.emissionClamp, cfg.emissionClamp);
    }
  }

  // --- kernel offsets (diamond / Manhattan ball) ---
  const int r = std::max(0, cfg.radius);
  std::vector<OffsetW> kernel;
  kernel.reserve(static_cast<std::size_t>((2 * r + 1) * (2 * r + 1)));

  const float decay = std::max(0.01f, cfg.decayPerTile);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const int md = std::abs(dx) + std::abs(dy);
      if (md > r) continue;
      const float wgt = 1.0f / (1.0f + static_cast<float>(md) * decay);
      kernel.push_back(OffsetW{dx, dy, wgt});
    }
  }

  // --- convolve ---
  float globalMax = 0.0f;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float acc = 0.0f;
      float wSum = 0.0f;

      for (const OffsetW& o : kernel) {
        const int xx = x + o.dx;
        const int yy = y + o.dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        const std::size_t j = FlatIdx(xx, yy, w);
        acc += emission[j] * o.w;
        wSum += o.w;
      }

      float v = (wSum > 0.0f) ? (acc / wSum) : 0.0f;

      // Convert back to [0,1] in a stable way.
      if (cfg.emissionClamp > 1e-6f) v /= cfg.emissionClamp;
      if (cfg.clamp01) v = Clamp01(v);

      // Gentle curve so low values remain visible.
      v = std::sqrt(std::max(0.0f, v));

      out.noise01[FlatIdx(x, y, w)] = v;
      globalMax = std::max(globalMax, v);
    }
  }

  out.maxNoise = globalMax;
  return out;
}

} // namespace isocity
