#include "isocity/ParkOptimizer.hpp"

#include "isocity/Isochrone.hpp"
#include "isocity/Pathfinding.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>

namespace isocity {

namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool IsIncludedZoneOverlay(Overlay o, const ParkOptimizerConfig& cfg)
{
  if (o == Overlay::Residential) return cfg.includeResidential;
  if (o == Overlay::Commercial) return cfg.includeCommercial;
  if (o == Overlay::Industrial) return cfg.includeIndustrial;
  return false;
}

inline std::uint64_t DemandWeightForTile(const Tile& t, const ParkOptimizerConfig& cfg)
{
  if (cfg.demandMode == ParkDemandMode::Tiles) return 1u;
  // Occupants mode: zones with 0 occupants contribute nothing.
  return static_cast<std::uint64_t>(t.occupants);
}

inline int ApplyTargetCost(int costMilli, int targetCostMilli)
{
  if (targetCostMilli <= 0) return costMilli;
  if (costMilli <= 0) return costMilli;
  const int d = costMilli - targetCostMilli;
  return (d > 0) ? d : 0;
}

} // namespace

ParkOptimizerResult SuggestParkPlacements(const World& world,
                                         const ParkOptimizerConfig& cfg,
                                         const ZoneAccessMap* precomputedZoneAccess,
                                         const std::vector<std::uint8_t>* precomputedRoadToEdge)
{
  ParkOptimizerResult out;
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;
  if (cfg.parksToAdd <= 0) return out;

  // --- Outside connection mask (optional) ---
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (cfg.requireOutsideConnection) {
    if (precomputedRoadToEdge && precomputedRoadToEdge->size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      roadToEdgeOwned.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
      ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
      roadToEdge = &roadToEdgeOwned;
    }
  }

  // --- Zone access mapping ---
  ZoneAccessMap zoneAccessOwned;
  const ZoneAccessMap* zoneAccess = nullptr;
  if (precomputedZoneAccess && precomputedZoneAccess->w == w && precomputedZoneAccess->h == h &&
      precomputedZoneAccess->roadIdx.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
    zoneAccess = precomputedZoneAccess;
  } else {
    zoneAccessOwned = BuildZoneAccessMap(world, roadToEdge);
    zoneAccess = &zoneAccessOwned;
  }

  // --- Demand aggregation: zone tiles -> their access road tile ---
  const std::size_t N = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<std::uint64_t> demandOnRoad(N, 0);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (!IsIncludedZoneOverlay(t.overlay, cfg)) continue;
      const std::size_t idx = FlatIdx(x, y, w);
      if (idx >= zoneAccess->roadIdx.size()) continue;
      const int ridx = zoneAccess->roadIdx[idx];
      if (ridx < 0) continue;
      const std::uint64_t wgt = DemandWeightForTile(t, cfg);
      if (wgt == 0) continue;
      const std::size_t r = static_cast<std::size_t>(ridx);
      if (r >= demandOnRoad.size()) continue;
      demandOnRoad[r] += wgt;
      out.totalDemandWeight += wgt;
    }
  }

  // If there's no demand, planning is pointless.
  if (out.totalDemandWeight == 0) return out;

  // --- Existing park sources (access road tiles) ---
  std::vector<int> sources;
  sources.reserve(64);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Park) continue;
      if (t.terrain == Terrain::Water) continue;
      Point road;
      if (!PickAdjacentRoadTile(world, roadToEdge, x, y, road)) continue;
      const int ridx = road.y * w + road.x;
      sources.push_back(ridx);
      out.existingParks += 1;
    }
  }
  std::sort(sources.begin(), sources.end());
  sources.erase(std::unique(sources.begin(), sources.end()), sources.end());

  // --- Candidate placement: for each road tile, pick one adjacent empty buildable tile ---
  std::vector<int> candidateParkTile(N, -1);

  auto canPlaceParkAt = [&](int x, int y) -> bool {
    if (!world.inBounds(x, y)) return false;
    const Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water) return false;
    if (t.overlay != Overlay::None) return false;
    return true;
  };

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // N,E,S,W
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const std::size_t ridx = FlatIdx(x, y, w);
      if (ridx >= candidateParkTile.size()) continue;
      if (roadToEdge && (*roadToEdge)[ridx] == 0) continue;

      int pick = -1;
      for (const auto& d : dirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (!canPlaceParkAt(nx, ny)) continue;
        pick = static_cast<int>(FlatIdx(nx, ny, w));
        break;
      }
      candidateParkTile[ridx] = pick;
    }
  }

  // Track park tiles already used by our suggestions.
  std::vector<std::uint8_t> usedParkTile(N, std::uint8_t{0});

  // Greedy iterations.
  std::vector<int> curSources = sources;
  curSources.reserve(sources.size() + static_cast<std::size_t>(cfg.parksToAdd));

  RoadIsochroneConfig icfg;
  icfg.requireOutsideConnection = cfg.requireOutsideConnection;
  icfg.weightMode = cfg.weightMode;
  icfg.computeOwner = false;

  // Use a deterministic "big cost" when a road tile is unreachable from existing parks.
  // (Also used when there are no initial parks at all.)
  const int unreachablePenalty = (w + h + 8) * 1000;

  for (int iter = 0; iter < cfg.parksToAdd; ++iter) {
    const bool haveBaseline = !curSources.empty();
    RoadIsochroneField roadField;
    if (haveBaseline) {
      roadField = BuildRoadIsochroneField(world, curSources, icfg, roadToEdge, nullptr);
    }

    int bestRoad = -1;
    int bestCost = -1;
    std::uint64_t bestDemand = 0;
    double bestScore = -1.0;

    for (std::size_t ridx = 0; ridx < N; ++ridx) {
      const std::uint64_t dmd = demandOnRoad[ridx];
      if (dmd == 0) continue;

      const int parkIdx = candidateParkTile[ridx];
      if (parkIdx < 0) continue;
      if (parkIdx >= static_cast<int>(N)) continue;
      if (usedParkTile[static_cast<std::size_t>(parkIdx)] != 0) continue;

      int cost = unreachablePenalty;
      if (haveBaseline) {
        if (ridx < roadField.costMilli.size()) {
          cost = roadField.costMilli[ridx];
          if (cost < 0) cost = unreachablePenalty;
        }
      }

      const int effCost = ApplyTargetCost(cost, cfg.targetCostMilli);
      const double score = static_cast<double>(effCost) * static_cast<double>(dmd);

      if (score > bestScore) {
        bestScore = score;
        bestRoad = static_cast<int>(ridx);
        bestCost = haveBaseline ? (ridx < roadField.costMilli.size() ? roadField.costMilli[ridx] : -1) : -1;
        bestDemand = dmd;
      } else if (score == bestScore && bestRoad >= 0) {
        // Stable tie-break: choose the smaller road index.
        if (static_cast<int>(ridx) < bestRoad) {
          bestRoad = static_cast<int>(ridx);
          bestCost = haveBaseline ? (ridx < roadField.costMilli.size() ? roadField.costMilli[ridx] : -1) : -1;
          bestDemand = dmd;
        }
      }
    }

    if (bestRoad < 0) break;

    const int parkIdx = candidateParkTile[static_cast<std::size_t>(bestRoad)];
    if (parkIdx < 0 || parkIdx >= static_cast<int>(N)) break;

    usedParkTile[static_cast<std::size_t>(parkIdx)] = 1;

    ParkPlacement p;
    p.parkTile = Point{parkIdx % w, parkIdx / w};
    p.accessRoad = Point{bestRoad % w, bestRoad / w};
    p.demandWeight = bestDemand;
    p.costMilliBefore = bestCost;
    p.score = bestScore;
    out.placements.push_back(p);

    // Add the access road tile as a new source (even if it duplicates an existing source;
    // Dijkstra will handle duplicates cheaply, but we keep it simple here).
    curSources.push_back(bestRoad);
  }

  return out;
}

void ApplyParkPlacements(World& world, const std::vector<ParkPlacement>& placements)
{
  for (const ParkPlacement& p : placements) {
    world.setOverlay(Overlay::Park, p.parkTile.x, p.parkTile.y);
  }
}

} // namespace isocity
