#include "isocity/FireRisk.hpp"

#include "isocity/Isochrone.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/ZoneMetrics.hpp"

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

inline float BaseFlammability(const Tile& t, const FireRiskConfig& cfg)
{
  if (t.terrain == Terrain::Water) return cfg.baseWater;
  switch (t.overlay) {
  case Overlay::None: return cfg.baseEmpty;
  case Overlay::Road:
    // Bridges (roads on water) should behave like water for fire risk.
    if (t.terrain == Terrain::Water) return cfg.baseWater;
    return cfg.baseRoad;
  case Overlay::Park: return cfg.basePark;
  case Overlay::Residential: return cfg.baseResidential;
  case Overlay::Commercial: return cfg.baseCommercial;
  case Overlay::Industrial: return cfg.baseIndustrial;
  default:
    if (IsCivic(t.overlay)) return cfg.baseCivic;
    return cfg.baseEmpty;
  }
}

inline float Level01(const Tile& t)
{
  // For zones + services, the level is in [1,3]. Treat missing/invalid as 1.
  const int lvl = std::clamp(static_cast<int>(t.level), 1, 3);
  return static_cast<float>(lvl - 1) / 2.0f;
}

inline float Occupancy01(const Tile& t)
{
  if (!IsZoneOverlay(t.overlay)) return 0.0f;
  const int cap = CapacityForTile(t);
  if (cap <= 0) return 0.0f;
  return Clamp01(static_cast<float>(t.occupants) / static_cast<float>(cap));
}

inline bool AdjacentRoadIndex(const World& world, int x, int y,
                              const std::vector<std::uint8_t>* roadToEdge,
                              int& outRoadIdx)
{
  const int w = world.width();
  const int h = world.height();

  auto tryDir = [&](int nx, int ny) -> bool {
    if (nx < 0 || ny < 0 || nx >= w || ny >= h) return false;
    const Tile& rt = world.at(nx, ny);
    if (rt.overlay != Overlay::Road) return false;
    const int idx = ny * w + nx;
    if (roadToEdge && roadToEdge->size() == static_cast<std::size_t>(w * h)) {
      if ((*roadToEdge)[static_cast<std::size_t>(idx)] == 0) return false;
    }
    outRoadIdx = idx;
    return true;
  };

  // Prefer cardinal directions.
  if (tryDir(x - 1, y)) return true;
  if (tryDir(x + 1, y)) return true;
  if (tryDir(x, y - 1)) return true;
  if (tryDir(x, y + 1)) return true;

  return false;
}

} // namespace

FireRiskResult ComputeFireRisk(const World& world, const FireRiskConfig& cfg,
                               const ZoneAccessMap* precomputedZoneAccess,
                               const std::vector<std::uint8_t>* precomputedRoadToEdge)
{
  FireRiskResult out{};
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.w = w;
  out.h = h;

  out.responseCostMilli.assign(n, -1);
  out.coverage01.assign(n, 0.0f);
  out.riskRaw.assign(n, 0.0f);
  out.risk01.assign(n, 0.0f);

  // --- optional cached maps ---
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = precomputedRoadToEdge;
  if (cfg.requireOutsideConnection) {
    if (!roadToEdge || roadToEdge->size() != n) {
      ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
      roadToEdge = &roadToEdgeOwned;
    }
  } else {
    roadToEdge = nullptr;
  }

  ZoneAccessMap zoneAccessOwned;
  const ZoneAccessMap* zoneAccess = precomputedZoneAccess;
  if (!zoneAccess || zoneAccess->w != w || zoneAccess->h != h || zoneAccess->roadIdx.size() != n) {
    zoneAccessOwned = BuildZoneAccessMap(world, roadToEdge);
    zoneAccess = &zoneAccessOwned;
  }

  // --- gather Fire Station access roads ---
  std::vector<int> sources;
  sources.reserve(64);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::FireStation) continue;
      out.fireStationCount++;

      int ridx = -1;
      if (AdjacentRoadIndex(world, x, y, roadToEdge, ridx)) {
        sources.push_back(ridx);
      }
    }
  }

  // Deduplicate.
  std::sort(sources.begin(), sources.end());
  sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
  out.sourceRoadCount = static_cast<int>(sources.size());

  // --- road + tile response cost ---
  RoadIsochroneField roadField{};
  if (!sources.empty()) {
    RoadIsochroneConfig rc;
    rc.requireOutsideConnection = cfg.requireOutsideConnection;
    rc.weightMode = cfg.weightMode;
    rc.computeOwner = false;
    roadField = BuildRoadIsochroneField(world, sources, rc, roadToEdge);
  } else {
    // No stations => all unreachable.
    roadField.w = w;
    roadField.h = h;
    roadField.costMilli.assign(n, -1);
    roadField.steps.assign(n, -1);
  }

  TileAccessCostConfig tc;
  tc.includeRoadTiles = true;
  tc.includeZones = true;
  tc.includeNonZonesAdjacentToRoad = true;
  tc.includeWater = false;
  tc.accessStepCostMilli = std::max(0, cfg.accessStepCostMilli);
  tc.useZoneAccessMap = true;

  out.responseCostMilli = BuildTileAccessCostField(world, roadField, tc, roadToEdge, zoneAccess);

  const int radiusSteps = std::max(1, cfg.responseRadiusSteps);
  const int radiusMilli = radiusSteps * 1000;
  const float invRadius = (radiusMilli > 0) ? (1.0f / static_cast<float>(radiusMilli)) : 0.0f;

  for (std::size_t i = 0; i < n; ++i) {
    const int c = out.responseCostMilli[i];
    if (c < 0 || c > radiusMilli) {
      out.coverage01[i] = 0.0f;
      continue;
    }
    // Simple smooth falloff: 1 at cost=0, 0 at cost=radius.
    const float t = Clamp01(static_cast<float>(c) * invRadius);
    // smoothstep(0..1)
    const float s = t * t * (3.0f - 2.0f * t);
    out.coverage01[i] = Clamp01(1.0f - s);
  }

  // --- base ignition risk ---
  std::vector<float> ignite(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float r = BaseFlammability(t, cfg);

      // Density + level boost (primarily for zones).
      const float occ01 = Occupancy01(t);
      const float lvl01 = Level01(t);
      r *= (1.0f + cfg.occupancyWeight * occ01);
      r *= (1.0f + cfg.levelWeight * lvl01);

      ignite[i] = r;
    }
  }

  // --- diffuse to create neighborhood-scale hot-spots ---
  const int iters = std::max(0, cfg.diffusionIterations);
  const float a = std::clamp(cfg.diffusion, 0.0f, 1.0f);
  std::vector<float> cur = ignite;
  std::vector<float> nxt(n, 0.0f);

  auto sample = [&](int xx, int yy) -> float {
    xx = std::clamp(xx, 0, w - 1);
    yy = std::clamp(yy, 0, h - 1);
    return cur[FlatIdx(xx, yy, w)];
  };

  if (iters > 0 && a > 0.0f) {
    for (int iter = 0; iter < iters; ++iter) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const Tile& t = world.at(x, y);
          const std::size_t i = FlatIdx(x, y, w);
          if (t.terrain == Terrain::Water) {
            nxt[i] = 0.0f;
            continue;
          }

          float sum = 0.0f;
          int cnt = 0;

          sum += sample(x - 1, y); ++cnt;
          sum += sample(x + 1, y); ++cnt;
          sum += sample(x, y - 1); ++cnt;
          sum += sample(x, y + 1); ++cnt;

          if (cfg.diffusionEightConnected) {
            sum += sample(x - 1, y - 1); ++cnt;
            sum += sample(x + 1, y - 1); ++cnt;
            sum += sample(x - 1, y + 1); ++cnt;
            sum += sample(x + 1, y + 1); ++cnt;
          }

          const float avg = (cnt > 0) ? (sum / static_cast<float>(cnt)) : cur[i];
          const float v = cur[i];
          nxt[i] = v + a * (avg - v);
        }
      }
      cur.swap(nxt);
    }
  }

  // --- apply coverage mitigation, clamp, and compute summary stats ---
  float sumRisk = 0.0f;
  float sumCov = 0.0f;
  int zoneCount = 0;
  int highCount = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float r = cur[i];
      const float cov = out.coverage01[i];
      r *= (1.0f - cfg.coverageMitigation * cov);

      // Roads and water should not show up as hot spots.
      if (t.overlay == Overlay::Road || t.terrain == Terrain::Water) {
        r *= 0.25f;
      }

      out.riskRaw[i] = r;
      const float r01 = Clamp01(r);
      out.risk01[i] = r01;

      if (IsZoneOverlay(t.overlay)) {
        sumRisk += r01;
        sumCov += cov;
        zoneCount++;
        if (r01 >= cfg.highRiskThreshold) highCount++;
      }
    }
  }

  if (zoneCount > 0) {
    out.avgZoneRisk = sumRisk / static_cast<float>(zoneCount);
    out.avgZoneCoverage = sumCov / static_cast<float>(zoneCount);
  }
  out.highRiskZoneTiles = highCount;

  return out;
}

} // namespace isocity
