#include "isocity/SkyView.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// 8- and 16-direction integer step sets (grid coordinates: +x east, +y south).
struct DirStep {
  int dx = 0;
  int dy = 0;
  float distPerStep = 1.0f; // sqrt(dx*dx + dy*dy)
};

constexpr float kSqrt2 = 1.41421356237f;
constexpr float kSqrt5 = 2.23606797750f;

static constexpr DirStep kDirs8[8] = {
    { 1,  0, 1.0f},   // E
    { 1, -1, kSqrt2}, // NE
    { 0, -1, 1.0f},   // N
    {-1, -1, kSqrt2}, // NW
    {-1,  0, 1.0f},   // W
    {-1,  1, kSqrt2}, // SW
    { 0,  1, 1.0f},   // S
    { 1,  1, kSqrt2}, // SE
};

static constexpr DirStep kDirs16[16] = {
    { 1,  0, 1.0f},   // E
    { 2, -1, kSqrt5}, // ENE
    { 1, -1, kSqrt2}, // NE
    { 1, -2, kSqrt5}, // NNE
    { 0, -1, 1.0f},   // N
    {-1, -2, kSqrt5}, // NNW
    {-1, -1, kSqrt2}, // NW
    {-2, -1, kSqrt5}, // WNW
    {-1,  0, 1.0f},   // W
    {-2,  1, kSqrt5}, // WSW
    {-1,  1, kSqrt2}, // SW
    {-1,  2, kSqrt5}, // SSW
    { 0,  1, 1.0f},   // S
    { 1,  2, kSqrt5}, // SSE
    { 1,  1, kSqrt2}, // SE
    { 2,  1, kSqrt5}, // ESE
};

inline bool IsCivic(Overlay o)
{
  return o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation;
}

inline float BuildingHeight(const Tile& t, const SkyViewConfig& cfg)
{
  if (!cfg.includeBuildings) return 0.0f;

  float h = 0.0f;
  const int lvl = std::clamp<int>(t.level, 1, 3);

  switch (t.overlay) {
  case Overlay::Residential: h = cfg.residentialHeightPerLevel * static_cast<float>(lvl); break;
  case Overlay::Commercial:  h = cfg.commercialHeightPerLevel  * static_cast<float>(lvl); break;
  case Overlay::Industrial:  h = cfg.industrialHeightPerLevel  * static_cast<float>(lvl); break;
  case Overlay::Road:
  case Overlay::Park:
  case Overlay::None:
    h = 0.0f;
    break;
  default:
    if (IsCivic(t.overlay)) {
      h = cfg.civicHeightPerLevel * static_cast<float>(lvl);
    }
    break;
  }

  if (h > 0.0f && cfg.occupantScale > 0 && cfg.occupantHeightBoost > 0.0f) {
    const float occ01 = Clamp01(static_cast<float>(t.occupants) / static_cast<float>(cfg.occupantScale));
    h += cfg.occupantHeightBoost * occ01;
  }

  return h;
}

} // namespace

SkyViewResult ComputeSkyViewFactor(const World& world, const SkyViewConfig& cfg)
{
  SkyViewResult out{};
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.skyView01.assign(n, 1.0f);
  out.canyon01.assign(n, 0.0f);

  auto idx = [&](int x, int y) -> std::size_t {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
  };

  // Effective heights.
  std::vector<float> effH(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      effH[idx(x, y)] = t.height + BuildingHeight(t, cfg);
    }
  }

  const bool use16 = (cfg.azimuthSamples > 8);
  const DirStep* dirs = use16 ? kDirs16 : kDirs8;
  const int dirCount = use16 ? 16 : 8;

  const int maxR = std::clamp(cfg.maxHorizonRadius, 1, std::max(w, h));

  double sumAll = 0.0;
  double sumRoad = 0.0;
  int roadCount = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i0 = idx(x, y);
      const float h0 = effH[i0];

      float acc = 0.0f;
      for (int d = 0; d < dirCount; ++d) {
        const DirStep ds = dirs[d];
        float maxAng = 0.0f;
        int sx = x;
        int sy = y;
        for (int step = 1; step <= maxR; ++step) {
          sx += ds.dx;
          sy += ds.dy;
          if (sx < 0 || sy < 0 || sx >= w || sy >= h) break;
          const float dh = effH[idx(sx, sy)] - h0;
          if (dh <= 0.0f) continue;
          const float dist = static_cast<float>(step) * ds.distPerStep;
          if (dist <= 0.0f) continue;
          const float ang = std::atan2(dh, dist);
          if (ang > maxAng) maxAng = ang;
        }

        const float c = std::cos(maxAng);
        acc += c * c;
      }

      const float svf = Clamp01(acc / static_cast<float>(dirCount));
      out.skyView01[i0] = svf;
      out.canyon01[i0] = 1.0f - svf;

      sumAll += static_cast<double>(svf);
      if (world.at(x, y).overlay == Overlay::Road) {
        sumRoad += static_cast<double>(svf);
        roadCount++;
      }
    }
  }

  out.meanSkyView = (n > 0) ? static_cast<float>(sumAll / static_cast<double>(n)) : 0.0f;
  out.roadTileCount = roadCount;
  out.meanRoadSkyView = (roadCount > 0) ? static_cast<float>(sumRoad / static_cast<double>(roadCount)) : 0.0f;

  return out;
}

} // namespace isocity
