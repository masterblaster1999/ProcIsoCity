#include "isocity/Walkability.hpp"

#include "isocity/Isochrone.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline float Smoothstep(float x)
{
  x = Clamp01(x);
  return x * x * (3.0f - 2.0f * x);
}

inline float ScoreFromCostMilli(int costMilli, int idealSteps, int maxSteps)
{
  if (costMilli < 0) return 0.0f;
  if (maxSteps <= idealSteps) {
    return (costMilli <= idealSteps * 1000) ? 1.0f : 0.0f;
  }

  const float steps = static_cast<float>(costMilli) / 1000.0f;
  if (steps <= static_cast<float>(idealSteps)) return 1.0f;
  if (steps >= static_cast<float>(maxSteps)) return 0.0f;

  const float t = (steps - static_cast<float>(idealSteps)) /
                  std::max(1e-6f, static_cast<float>(maxSteps - idealSteps));
  return 1.0f - Smoothstep(t);
}

struct CategoryField {
  WalkAmenity kind = WalkAmenity::Park;
  WalkabilityCategoryConfig cfg{};
  std::vector<int>* outCostMilli = nullptr;
  std::vector<float>* outScore01 = nullptr;
};

inline bool IsZone(Overlay o)
{
  return o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial;
}

inline bool IsAmenityTile(WalkAmenity a, const Tile& t)
{
  switch (a) {
    case WalkAmenity::Park: return t.overlay == Overlay::Park;
    case WalkAmenity::Retail: return t.overlay == Overlay::Commercial;
    case WalkAmenity::Education: return t.overlay == Overlay::School;
    case WalkAmenity::Health: return t.overlay == Overlay::Hospital;
    case WalkAmenity::Safety: return t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation;
    default: return false;
  }
}

inline const WalkabilityCategoryConfig& GetCfg(const WalkabilityConfig& cfg, WalkAmenity a)
{
  switch (a) {
    case WalkAmenity::Park: return cfg.park;
    case WalkAmenity::Retail: return cfg.retail;
    case WalkAmenity::Education: return cfg.education;
    case WalkAmenity::Health: return cfg.health;
    case WalkAmenity::Safety: return cfg.safety;
    default: return cfg.park;
  }
}

inline std::vector<int> GatherAmenitySourceRoads(const World& world,
                                                 WalkAmenity a,
                                                 const std::vector<std::uint8_t>* roadToEdgeMask,
                                                 const ZoneAccessMap* zoneAccess)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  std::vector<std::uint8_t> seen;
  seen.assign(n, 0);

  std::vector<int> out;
  out.reserve(256);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (!IsAmenityTile(a, t)) continue;

      Point road{-1, -1};
      bool ok = PickAdjacentRoadTile(world, roadToEdgeMask, x, y, road);

      // For zoned amenities (notably Commercial), allow interior tiles to contribute
      // sources through ZoneAccessMap.
      if (!ok && zoneAccess && IsZone(t.overlay)) {
        ok = PickZoneAccessRoadTile(*zoneAccess, x, y, road);
        if (ok && roadToEdgeMask && !roadToEdgeMask->empty()) {
          const int ridx = road.y * w + road.x;
          if (ridx < 0 || static_cast<std::size_t>(ridx) >= n || (*roadToEdgeMask)[static_cast<std::size_t>(ridx)] == 0) {
            ok = false;
          }
        }
      }

      if (!ok) continue;
      const int ridx = road.y * w + road.x;
      if (ridx < 0) continue;
      const std::size_t fi = static_cast<std::size_t>(ridx);
      if (fi >= n) continue;
      if (seen[fi]) continue;
      seen[fi] = 1;
      out.push_back(ridx);
    }
  }

  // Keep deterministic order: sort ascending.
  std::sort(out.begin(), out.end());
  return out;
}

inline void EnsureSizeI(std::vector<int>& v, std::size_t n, int fill)
{
  if (v.size() != n) v.assign(n, fill);
}

inline void EnsureSizeF(std::vector<float>& v, std::size_t n, float fill)
{
  if (v.size() != n) v.assign(n, fill);
}

inline void EnsureSizeU8(std::vector<std::uint8_t>& v, std::size_t n, std::uint8_t fill)
{
  if (v.size() != n) v.assign(n, fill);
}

} // namespace

const char* WalkAmenityName(WalkAmenity a)
{
  switch (a) {
    case WalkAmenity::Park: return "park";
    case WalkAmenity::Retail: return "retail";
    case WalkAmenity::Education: return "education";
    case WalkAmenity::Health: return "health";
    case WalkAmenity::Safety: return "safety";
    default: return "unknown";
  }
}

WalkabilityResult ComputeWalkability(const World& world,
                                    const WalkabilityConfig& cfg,
                                    const std::vector<std::uint8_t>* precomputedRoadToEdge,
                                    const ZoneAccessMap* precomputedZoneAccess)
{
  WalkabilityResult out{};
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  if (!cfg.enabled || w <= 0 || h <= 0) {
    return out;
  }

  // Optional outside-connection mask.
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (cfg.requireOutsideConnection) {
    if (precomputedRoadToEdge && precomputedRoadToEdge->size() == n) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      roadToEdgeOwned.assign(n, 0);
      ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
      roadToEdge = &roadToEdgeOwned;
    }
  }

  // Optional ZoneAccessMap (also used for amenity source discovery).
  ZoneAccessMap zoneAccessOwned;
  const ZoneAccessMap* zoneAccess = nullptr;
  if (precomputedZoneAccess && precomputedZoneAccess->w == w && precomputedZoneAccess->h == h &&
      precomputedZoneAccess->roadIdx.size() == n) {
    zoneAccess = precomputedZoneAccess;
  } else {
    zoneAccessOwned = BuildZoneAccessMap(world, roadToEdge);
    if (zoneAccessOwned.w == w && zoneAccessOwned.h == h && zoneAccessOwned.roadIdx.size() == n) {
      zoneAccess = &zoneAccessOwned;
    }
  }

  // Prepare output buffers.
  EnsureSizeI(out.costParkMilli, n, -1);
  EnsureSizeI(out.costRetailMilli, n, -1);
  EnsureSizeI(out.costEducationMilli, n, -1);
  EnsureSizeI(out.costHealthMilli, n, -1);
  EnsureSizeI(out.costSafetyMilli, n, -1);

  EnsureSizeF(out.park01, n, 0.0f);
  EnsureSizeF(out.retail01, n, 0.0f);
  EnsureSizeF(out.education01, n, 0.0f);
  EnsureSizeF(out.health01, n, 0.0f);
  EnsureSizeF(out.safety01, n, 0.0f);
  EnsureSizeF(out.overall01, n, 0.0f);

  EnsureSizeU8(out.coverageMask, n, 0);

  // Compute each category's cost and score fields.
  std::array<CategoryField, static_cast<int>(WalkAmenity::Count)> cats = {
      CategoryField{WalkAmenity::Park, cfg.park, &out.costParkMilli, &out.park01},
      CategoryField{WalkAmenity::Retail, cfg.retail, &out.costRetailMilli, &out.retail01},
      CategoryField{WalkAmenity::Education, cfg.education, &out.costEducationMilli, &out.education01},
      CategoryField{WalkAmenity::Health, cfg.health, &out.costHealthMilli, &out.health01},
      CategoryField{WalkAmenity::Safety, cfg.safety, &out.costSafetyMilli, &out.safety01},
  };

  RoadIsochroneConfig rcfg{};
  rcfg.requireOutsideConnection = cfg.requireOutsideConnection;
  rcfg.weightMode = cfg.weightMode;
  rcfg.computeOwner = false;

  TileAccessCostConfig tcfg{};
  tcfg.includeRoadTiles = true;
  tcfg.includeZones = true;
  tcfg.includeNonZonesAdjacentToRoad = true;
  tcfg.includeWater = false;
  tcfg.accessStepCostMilli = std::max(0, cfg.accessStepCostMilli);
  tcfg.useZoneAccessMap = true;

  const int coverSteps = std::max(0, cfg.coverageThresholdSteps);
  const int coverMilli = coverSteps * 1000;

  for (CategoryField& cf : cats) {
    const int ci = static_cast<int>(cf.kind);
    const WalkabilityCategoryConfig& ccfg = cf.cfg;
    if (!ccfg.enabled || !(ccfg.weight > 0.0f)) {
      out.sourceCount[ci] = 0;
      continue;
    }

    const std::vector<int> sources = GatherAmenitySourceRoads(world, cf.kind, roadToEdge, zoneAccess);
    out.sourceCount[ci] = static_cast<int>(sources.size());

    if (sources.empty()) {
      // Leave costs at -1 and scores at 0.
      continue;
    }

    const RoadIsochroneField roadField = BuildRoadIsochroneField(world, sources, rcfg, roadToEdge, nullptr);
    const std::vector<int> tileCost = BuildTileAccessCostField(world, roadField, tcfg, roadToEdge, zoneAccess);

    if (tileCost.size() != n) continue;
    *(cf.outCostMilli) = tileCost;

    // Compute per-tile scores and coverage.
    std::vector<float>& score01 = *(cf.outScore01);
    for (std::size_t i = 0; i < n; ++i) {
      const int c = tileCost[i];
      score01[i] = ScoreFromCostMilli(c, std::max(0, ccfg.idealSteps), std::max(0, ccfg.maxSteps));
      if (c >= 0 && c <= coverMilli) {
        out.coverageMask[i] = static_cast<std::uint8_t>(out.coverageMask[i] | (1u << ci));
      }
    }
  }

  // Combine into overall score.
  float weightSum = 0.0f;
  std::array<float, static_cast<int>(WalkAmenity::Count)> weights{};
  for (int i = 0; i < static_cast<int>(WalkAmenity::Count); ++i) {
    const WalkAmenity a = static_cast<WalkAmenity>(i);
    const WalkabilityCategoryConfig& ccfg = GetCfg(cfg, a);
    const float wgt = (ccfg.enabled && ccfg.weight > 0.0f) ? ccfg.weight : 0.0f;
    weights[i] = wgt;
    weightSum += wgt;
  }
  if (weightSum <= 1e-6f) {
    // Nothing enabled.
    return out;
  }

  for (std::size_t i = 0; i < n; ++i) {
    float acc = 0.0f;
    acc += weights[static_cast<int>(WalkAmenity::Park)] * out.park01[i];
    acc += weights[static_cast<int>(WalkAmenity::Retail)] * out.retail01[i];
    acc += weights[static_cast<int>(WalkAmenity::Education)] * out.education01[i];
    acc += weights[static_cast<int>(WalkAmenity::Health)] * out.health01[i];
    acc += weights[static_cast<int>(WalkAmenity::Safety)] * out.safety01[i];
    out.overall01[i] = Clamp01(acc / weightSum);
  }

  // Residential-weighted summary.
  std::array<std::uint64_t, static_cast<int>(WalkAmenity::Count)> coveredPop{};
  std::uint64_t allCoveredPop = 0;
  int resTileCount = 0;
  std::uint64_t pop = 0;
  double sumScore = 0.0;

  std::uint8_t enabledMask = 0;
  for (int i = 0; i < static_cast<int>(WalkAmenity::Count); ++i) {
    const WalkAmenity a = static_cast<WalkAmenity>(i);
    const WalkabilityCategoryConfig& ccfg = GetCfg(cfg, a);
    if (ccfg.enabled && ccfg.weight > 0.0f) enabledMask |= static_cast<std::uint8_t>(1u << i);
  }

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants == 0) continue;
      ++resTileCount;
      const std::size_t i = FlatIdx(x, y, w);
      const std::uint64_t wgt = static_cast<std::uint64_t>(t.occupants);
      pop += wgt;
      sumScore += static_cast<double>(out.overall01[i]) * static_cast<double>(wgt);

      const std::uint8_t m = out.coverageMask[i];
      for (int c = 0; c < static_cast<int>(WalkAmenity::Count); ++c) {
        if ((enabledMask & (1u << c)) == 0) continue;
        if (m & (1u << c)) coveredPop[c] += wgt;
      }
      if (enabledMask != 0 && (m & enabledMask) == enabledMask) {
        allCoveredPop += wgt;
      }
    }
  }

  out.residentPopulation = static_cast<int>(std::min<std::uint64_t>(pop, static_cast<std::uint64_t>(std::numeric_limits<int>::max())));
  out.residentialTileCount = resTileCount;
  if (pop > 0) {
    out.residentAvgOverall01 = static_cast<float>(Clamp01(static_cast<float>(sumScore / static_cast<double>(pop))));
    for (int c = 0; c < static_cast<int>(WalkAmenity::Count); ++c) {
      if ((enabledMask & (1u << c)) == 0) {
        out.residentCoverageFrac[c] = 0.0f;
      } else {
        out.residentCoverageFrac[c] = static_cast<float>(Clamp01(static_cast<float>(static_cast<double>(coveredPop[c]) / static_cast<double>(pop))));
      }
    }
    out.residentAllCategoriesFrac = static_cast<float>(Clamp01(static_cast<float>(static_cast<double>(allCoveredPop) / static_cast<double>(pop))));
  }

  return out;
}

} // namespace isocity
