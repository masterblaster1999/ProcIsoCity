#include "isocity/HeatIsland.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool IsCivic(Overlay o)
{
  return (o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation);
}

} // namespace

HeatIslandResult ComputeHeatIsland(const World& world, const HeatIslandConfig& cfg,
                                  const TrafficResult* traffic, const GoodsResult* goods)
{
  HeatIslandResult out{};
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.w = w;
  out.h = h;
  out.iterations = std::max(0, cfg.iterations);
  out.diffusion = std::clamp(cfg.diffusion, 0.0f, 1.0f);
  out.eightConnected = cfg.eightConnected;

  out.heat.assign(n, 0.0f);
  out.heat01.assign(n, 0.0f);

  // --- normalize traffic/goods if present ---
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

  const float clampAbs = std::max(0.01f, cfg.sourceClampAbs);
  const float occScale = static_cast<float>(std::max(1, cfg.occupantScale));

  // --- base heat signal (sources/sinks) ---
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float s = 0.0f;

      // Elevation cooling.
      s -= cfg.elevationCooling * Clamp01(t.height);

      // Terrain sinks.
      if (t.terrain == Terrain::Water) {
        s -= cfg.waterSink;
      }

      // Overlay-based sources/sinks.
      switch (t.overlay) {
      case Overlay::Road: {
        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        s += cfg.roadBase + cfg.roadClassBoost * static_cast<float>(lvl - 1);

        float commute01 = 0.0f;
        if (maxCommute > 0 && traffic && traffic->roadTraffic.size() == n) {
          commute01 = static_cast<float>(traffic->roadTraffic[i]) / static_cast<float>(maxCommute);
        } else {
          commute01 = cfg.fallbackCommuteTraffic01;
        }
        s += cfg.roadTrafficBoost * Clamp01(commute01);

        float goods01 = 0.0f;
        if (maxGoods > 0 && goods && goods->roadGoodsTraffic.size() == n) {
          goods01 = static_cast<float>(goods->roadGoodsTraffic[i]) / static_cast<float>(maxGoods);
        } else {
          goods01 = cfg.fallbackGoodsTraffic01;
        }
        s += cfg.roadGoodsBoost * Clamp01(goods01);
      } break;

      case Overlay::Residential:
        s += cfg.residentialSource;
        break;
      case Overlay::Commercial:
        s += cfg.commercialSource;
        break;
      case Overlay::Industrial:
        s += cfg.industrialSource;
        break;
      case Overlay::Park:
        s -= cfg.parkSink;
        break;

      default:
        if (IsCivic(t.overlay)) {
          s += cfg.civicSource;
        }
        break;
      }

      // Population/employment density: treat occupants as an anthropogenic heat proxy.
      if (t.occupants > 0) {
        const float occ01 = Clamp01(static_cast<float>(t.occupants) / occScale);
        s += cfg.occupantBoost * occ01;
      }

      out.heat[i] = std::clamp(s, -clampAbs, clampAbs);
    }
  }

  // --- diffusion / smoothing ---
  const int iters = std::max(0, cfg.iterations);
  const float a = std::clamp(cfg.diffusion, 0.0f, 1.0f);
  if (iters > 0 && a > 0.0f) {
    std::vector<float> cur = out.heat;
    std::vector<float> nxt(n, 0.0f);

    auto sample = [&](int xx, int yy) -> float {
      xx = std::clamp(xx, 0, w - 1);
      yy = std::clamp(yy, 0, h - 1);
      return cur[FlatIdx(xx, yy, w)];
    };

    for (int iter = 0; iter < iters; ++iter) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          float sum = 0.0f;
          int cnt = 0;

          // 4-neighborhood.
          sum += sample(x - 1, y); ++cnt;
          sum += sample(x + 1, y); ++cnt;
          sum += sample(x, y - 1); ++cnt;
          sum += sample(x, y + 1); ++cnt;

          if (cfg.eightConnected) {
            sum += sample(x - 1, y - 1); ++cnt;
            sum += sample(x + 1, y - 1); ++cnt;
            sum += sample(x - 1, y + 1); ++cnt;
            sum += sample(x + 1, y + 1); ++cnt;
          }

          const float avg = (cnt > 0) ? (sum / static_cast<float>(cnt)) : cur[FlatIdx(x, y, w)];
          const std::size_t i = FlatIdx(x, y, w);
          const float v = cur[i];
          nxt[i] = v + a * (avg - v);
        }
      }
      cur.swap(nxt);
    }

    out.heat.swap(cur);
  }

  // --- normalize to [0,1] ---
  float mn = std::numeric_limits<float>::infinity();
  float mx = -std::numeric_limits<float>::infinity();
  for (float v : out.heat) {
    mn = std::min(mn, v);
    mx = std::max(mx, v);
  }
  if (!std::isfinite(mn) || !std::isfinite(mx)) {
    mn = 0.0f;
    mx = 0.0f;
  }

  out.minHeat = mn;
  out.maxHeat = mx;

  const float denom = mx - mn;
  if (denom > 1e-6f) {
    for (std::size_t i = 0; i < n; ++i) {
      out.heat01[i] = Clamp01((out.heat[i] - mn) / denom);
    }
  } else {
    // Flat map: arbitrarily choose 0.5.
    std::fill(out.heat01.begin(), out.heat01.end(), 0.5f);
  }

  return out;
}

} // namespace isocity
