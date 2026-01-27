#include "isocity/AirPollution.hpp"

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

inline std::uint64_t Mix64(std::uint64_t x)
{
  // splitmix64-style mixer
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  x = x ^ (x >> 31);
  return x;
}

float SampleBilinearClamped(const std::vector<float>& field, int w, int h, float fx, float fy)
{
  if (w <= 0 || h <= 0) return 0.0f;
  fx = std::clamp(fx, 0.0f, static_cast<float>(w - 1));
  fy = std::clamp(fy, 0.0f, static_cast<float>(h - 1));

  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  const int x1 = std::min(x0 + 1, w - 1);
  const int y1 = std::min(y0 + 1, h - 1);

  const float tx = fx - static_cast<float>(x0);
  const float ty = fy - static_cast<float>(y0);

  const float a = field[FlatIdx(x0, y0, w)];
  const float b = field[FlatIdx(x1, y0, w)];
  const float c = field[FlatIdx(x0, y1, w)];
  const float d = field[FlatIdx(x1, y1, w)];

  const float ab = a + (b - a) * tx;
  const float cd = c + (d - c) * tx;
  return ab + (cd - ab) * ty;
}

} // namespace

const char* WindDirName(WindDir d)
{
  switch (d) {
  case WindDir::None:
    return "none";
  case WindDir::N:
    return "n";
  case WindDir::NE:
    return "ne";
  case WindDir::E:
    return "e";
  case WindDir::SE:
    return "se";
  case WindDir::S:
    return "s";
  case WindDir::SW:
    return "sw";
  case WindDir::W:
    return "w";
  case WindDir::NW:
    return "nw";
  default:
    return "unknown";
  }
}

void WindDirVector(WindDir d, float& outX, float& outY)
{
  outX = 0.0f;
  outY = 0.0f;
  switch (d) {
  case WindDir::None:
    return;
  case WindDir::N:
    outX = 0.0f;
    outY = -1.0f;
    return;
  case WindDir::NE:
    outX = 0.70710678f;
    outY = -0.70710678f;
    return;
  case WindDir::E:
    outX = 1.0f;
    outY = 0.0f;
    return;
  case WindDir::SE:
    outX = 0.70710678f;
    outY = 0.70710678f;
    return;
  case WindDir::S:
    outX = 0.0f;
    outY = 1.0f;
    return;
  case WindDir::SW:
    outX = -0.70710678f;
    outY = 0.70710678f;
    return;
  case WindDir::W:
    outX = -1.0f;
    outY = 0.0f;
    return;
  case WindDir::NW:
    outX = -0.70710678f;
    outY = -0.70710678f;
    return;
  default:
    return;
  }
}

WindDir InferWindDirFromSeed(std::uint64_t seed)
{
  const std::uint64_t h = Mix64(seed);
  const int idx = static_cast<int>((h >> 61) & 0x7ull); // 0..7
  // Map to N..NW (skip None).
  return static_cast<WindDir>(idx + 1);
}

AirPollutionResult ComputeAirPollution(const World& world, const AirPollutionConfig& cfg,
                                       const TrafficResult* traffic, const GoodsResult* goods)
{
  AirPollutionResult out{};
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.w = w;
  out.h = h;
  out.cfg = cfg;
  out.emission01.assign(n, 0.0f);
  out.pollution01.assign(n, 0.0f);
  out.highExposureThreshold = cfg.clamp01 > 0.0f ? 0.65f : 0.65f;

  const float clampAbs = std::max(0.01f, cfg.clamp01);
  const int iters = std::max(0, cfg.iterations);
  const float diff = std::clamp(cfg.diffusion, 0.0f, 1.0f);
  const float adv = std::clamp(cfg.advection, 0.0f, 1.0f);
  const float windSpeed = std::max(0.0f, cfg.windSpeed);
  const float decay = std::clamp(cfg.decayPerIteration, 0.0f, 1.0f);
  const float occScale = static_cast<float>(std::max(1, cfg.occupantScale));

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

  // --- compute emission field ---
  float maxEmission = 0.0f;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float e = 0.0f;

      // Ventilation with elevation.
      e -= cfg.elevationVentilation * Clamp01(t.height);

      // Terrain sinks.
      if (t.terrain == Terrain::Water) {
        e -= cfg.waterSink;
      }

      // Overlay-based sources/sinks.
      switch (t.overlay) {
      case Overlay::Road: {
        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        e += cfg.roadBase + cfg.roadClassBoost * static_cast<float>(lvl - 1);

        float commute01 = 0.0f;
        if (maxCommute > 0 && traffic && traffic->roadTraffic.size() == n) {
          commute01 = static_cast<float>(traffic->roadTraffic[i]) / static_cast<float>(maxCommute);
        } else {
          commute01 = cfg.fallbackCommuteTraffic01;
        }
        e += cfg.commuteTrafficBoost * Clamp01(commute01);

        float goods01 = 0.0f;
        if (maxGoods > 0 && goods && goods->roadGoodsTraffic.size() == n) {
          goods01 = static_cast<float>(goods->roadGoodsTraffic[i]) / static_cast<float>(maxGoods);
        } else {
          goods01 = cfg.fallbackGoodsTraffic01;
        }
        e += cfg.goodsTrafficBoost * Clamp01(goods01);
      } break;

      case Overlay::Residential:
        e += cfg.residentialSource;
        break;
      case Overlay::Commercial:
        e += cfg.commercialSource;
        break;
      case Overlay::Industrial:
        e += cfg.industrialSource;
        break;
      case Overlay::Park:
        e -= cfg.parkSink;
        break;

      default:
        if (IsCivic(t.overlay)) {
          e += cfg.civicSource;
        }
        break;
      }

      if (t.occupants > 0) {
        const float occ01 = Clamp01(static_cast<float>(t.occupants) / occScale);
        e += cfg.occupantBoost * occ01;
      }

      // Clamp to [0, clampAbs] and normalize into [0,1].
      e = std::clamp(e, 0.0f, clampAbs);
      const float e01 = Clamp01(e / clampAbs);
      out.emission01[i] = e01;
      maxEmission = std::max(maxEmission, e01);
    }
  }
  out.maxEmission01 = maxEmission;

  // --- transport (advection + diffusion + deposition + decay) ---
  std::vector<float> cur = out.emission01;
  std::vector<float> tmp(n, 0.0f);
  std::vector<float> nxt(n, 0.0f);

  // Wind.
  WindDir windDir = cfg.fixedWindDir;
  if (cfg.windFromSeed) {
    windDir = InferWindDirFromSeed(world.seed());
  }
  float windX = 0.0f, windY = 0.0f;
  WindDirVector(windDir, windX, windY);

  auto sampleClamp = [&](const std::vector<float>& field, int xx, int yy) -> float {
    xx = std::clamp(xx, 0, w - 1);
    yy = std::clamp(yy, 0, h - 1);
    return field[FlatIdx(xx, yy, w)];
  };

  for (int iter = 0; iter < iters; ++iter) {
    // Diffusion: blend towards neighbor average.
    if (diff > 0.0f) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          float sum = 0.0f;
          int cnt = 0;

          sum += sampleClamp(cur, x - 1, y); ++cnt;
          sum += sampleClamp(cur, x + 1, y); ++cnt;
          sum += sampleClamp(cur, x, y - 1); ++cnt;
          sum += sampleClamp(cur, x, y + 1); ++cnt;

          if (cfg.eightConnected) {
            sum += sampleClamp(cur, x - 1, y - 1); ++cnt;
            sum += sampleClamp(cur, x + 1, y - 1); ++cnt;
            sum += sampleClamp(cur, x - 1, y + 1); ++cnt;
            sum += sampleClamp(cur, x + 1, y + 1); ++cnt;
          }

          const std::size_t i = FlatIdx(x, y, w);
          const float v = cur[i];
          const float avg = (cnt > 0) ? (sum / static_cast<float>(cnt)) : v;
          tmp[i] = v + diff * (avg - v);
        }
      }
    } else {
      tmp = cur;
    }

    // Advection: sample from upwind.
    if (adv > 0.0f && windSpeed > 0.0f && (windX != 0.0f || windY != 0.0f)) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t i = FlatIdx(x, y, w);
          const float srcX = static_cast<float>(x) - windX * windSpeed;
          const float srcY = static_cast<float>(y) - windY * windSpeed;
          const float up = SampleBilinearClamped(tmp, w, h, srcX, srcY);
          nxt[i] = tmp[i] + adv * (up - tmp[i]);
        }
      }
    } else {
      nxt = tmp;
    }

    // Deposition + decay (local sinks that actively remove transported pollution).
    if (decay > 0.0f || cfg.depositionPark > 0.0f || cfg.depositionWater > 0.0f) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t i = FlatIdx(x, y, w);
          const Tile& t = world.at(x, y);

          float v = nxt[i];

          float dep = 0.0f;
          if (t.overlay == Overlay::Park) dep += cfg.depositionPark;
          if (t.terrain == Terrain::Water) dep += cfg.depositionWater;
          dep = std::clamp(dep, 0.0f, 0.95f);

          if (dep > 0.0f) v *= (1.0f - dep);
          if (decay > 0.0f) v *= (1.0f - decay);

          // Always keep in [0,1].
          nxt[i] = Clamp01(v);
        }
      }
    }

    cur.swap(nxt);
  }

  out.pollution01.swap(cur);

  // --- stats ---
  float maxP = 0.0f;
  for (float v : out.pollution01) maxP = std::max(maxP, v);
  out.maxPollution01 = maxP;

  // Residential-weighted exposure summary.
  std::int64_t pop = 0;
  double sumPopWeighted = 0.0;
  std::int64_t highPop = 0;

  int resTiles = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants == 0) continue;

      ++resTiles;
      pop += t.occupants;
      sumPopWeighted += static_cast<double>(out.pollution01[i]) * static_cast<double>(t.occupants);

      if (out.pollution01[i] >= out.highExposureThreshold) {
        highPop += t.occupants;
      }
    }
  }

  out.residentialTileCount = resTiles;
  out.residentPopulation = static_cast<int>(std::clamp<std::int64_t>(pop, 0, std::numeric_limits<int>::max()));

  if (pop > 0) {
    out.residentAvgPollution01 = static_cast<float>(sumPopWeighted / static_cast<double>(pop));
    out.residentHighExposureFrac = static_cast<float>(static_cast<double>(highPop) / static_cast<double>(pop));
  } else {
    out.residentAvgPollution01 = 0.0f;
    out.residentHighExposureFrac = 0.0f;
  }

  return out;
}

} // namespace isocity
