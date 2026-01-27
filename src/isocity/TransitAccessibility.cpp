#include "isocity/TransitAccessibility.hpp"

#include "isocity/Goods.hpp"
#include "isocity/Isochrone.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/Sim.hpp" // TransitDemandMode
#include "isocity/Traffic.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_set>

namespace isocity {
namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline float Clamp01(float v)
{
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

inline float SmoothStep01(float t)
{
  t = Clamp01(t);
  return t * t * (3.0f - 2.0f * t);
}

inline float StepsToAccess01(int steps, int goodSteps, int badSteps)
{
  if (steps < 0) return 0.0f;
  goodSteps = std::max(0, goodSteps);
  badSteps = std::max(goodSteps + 1, badSteps);
  if (steps <= goodSteps) return 1.0f;
  if (steps >= badSteps) return 0.0f;

  const float t = static_cast<float>(steps - goodSteps) / static_cast<float>(badSteps - goodSteps);
  // t=0 => good, t=1 => bad.
  return 1.0f - SmoothStep01(t);
}

inline int BestAdjacentRoadSteps(const World& world, const RoadIsochroneField& field, int x, int y)
{
  const int w = world.width();
  const int h = world.height();
  int best = -1;
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    const int nx = x + dx[k];
    const int ny = y + dy[k];
    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
    const Tile& t2 = world.at(nx, ny);
    if (t2.overlay != Overlay::Road) continue;
    const std::size_t ni = FlatIdx(nx, ny, w);
    if (ni >= field.steps.size()) continue;
    const int s = field.steps[ni];
    if (s < 0) continue;
    if (best < 0 || s < best) best = s;
  }
  return best;
}

inline std::uint64_t HashStopKey(int x, int y)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
}

} // namespace


TransitAccessibilityResult ComputeTransitAccessibility(const World& world,
                                                      const TransitAccessibilityConfig& cfgIn,
                                                      const TransitAccessibilityInputs& in)
{
  TransitAccessibilityResult out;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.w = w;
  out.h = h;
  out.cfg = cfgIn;

  out.stepsToStop.assign(n, -1);
  out.access01.assign(n, 0.0f);
  out.modeSharePotential01.assign(n, 0.0f);
  out.stopMask.assign(n, 0);
  out.corridorMask.assign(n, 0);

  // Outside-connection mask.
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (out.cfg.requireOutsideConnection) {
    if (in.roadToEdgeMask && in.roadToEdgeMask->size() == n) {
      roadToEdge = in.roadToEdgeMask;
    } else {
      ComputeRoadsConnectedToEdge(world, roadToEdgeLocal);
      roadToEdge = &roadToEdgeLocal;
    }
  }

  // Road graph.
  RoadGraph roadGraphLocal;
  const RoadGraph* g = nullptr;
  if (in.roadGraph && !in.roadGraph->edges.empty()) {
    g = in.roadGraph;
  } else {
    roadGraphLocal = BuildRoadGraph(world);
    g = &roadGraphLocal;
  }

  if (!g || g->edges.empty()) {
    return out;
  }

  // Transit plan.
  TransitPlan planLocal;
  const TransitPlan* plan = nullptr;
  if (in.plan) {
    plan = in.plan;
  } else {
    // Demand signal on road tiles.
    std::vector<std::uint32_t> roadFlow(n, 0);

    const TransitDemandMode dm = out.cfg.demandMode;
    const bool needTraffic = (dm == TransitDemandMode::Commute || dm == TransitDemandMode::Combined);
    const bool needGoods = (dm == TransitDemandMode::Goods || dm == TransitDemandMode::Combined);

    if (needTraffic && in.traffic && in.traffic->roadTraffic.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        roadFlow[i] += static_cast<std::uint32_t>(in.traffic->roadTraffic[i]);
      }
    }
    if (needGoods && in.goods && in.goods->roadGoodsTraffic.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        roadFlow[i] += static_cast<std::uint32_t>(in.goods->roadGoodsTraffic[i]);
      }
    }

    // Fallback: if no demand signal is available, still plan a line using a tiny uniform
    // baseline on road tiles so exports don't silently become blank.
    bool anyDemand = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (roadFlow[i] != 0) {
        anyDemand = true;
        break;
      }
    }
    if (!anyDemand) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const Tile& t = world.at(x, y);
          if (t.overlay == Overlay::Road && t.terrain != Terrain::Water) {
            roadFlow[FlatIdx(x, y, w)] = 1;
          }
        }
      }
    }

    const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(world, *g, roadFlow);
    std::vector<std::uint64_t> edgeDemand(g->edges.size(), 0);
    for (std::size_t ei = 0; ei < g->edges.size() && ei < agg.edges.size(); ++ei) {
      edgeDemand[ei] = agg.edges[ei].sumTrafficInterior;
    }

    TransitPlannerConfig pcfg = out.cfg.plannerCfg;
    if (pcfg.seedSalt == 0) {
      pcfg.seedSalt = (world.seed() ^ 0xD11B0A5A2B22F3A1ULL) ^
                      (static_cast<std::uint64_t>(dm) * 0x9E3779B97F4A7C15ULL);
    }

    planLocal = PlanTransitLines(*g, edgeDemand, pcfg, &world);
    planLocal.cfg = pcfg;
    plan = &planLocal;
  }

  if (!plan || plan->lines.empty()) {
    // No lines => no stops; leave outputs as zeroes.
    return out;
  }

  out.plannedLines = static_cast<int>(plan->lines.size());

  // Served corridors: mark all road tiles that belong to a served edge.
  std::vector<std::uint8_t> servedEdge(g->edges.size(), 0);
  for (const TransitLine& line : plan->lines) {
    for (int ei : line.edges) {
      if (ei >= 0 && ei < static_cast<int>(servedEdge.size())) {
        servedEdge[static_cast<std::size_t>(ei)] = 1;
      }
    }
  }

  for (std::size_t ei = 0; ei < servedEdge.size(); ++ei) {
    if (!servedEdge[ei]) continue;
    const RoadGraphEdge& e = g->edges[ei];
    for (const Point& p : e.tiles) {
      if (!world.inBounds(p.x, p.y)) continue;
      const std::size_t ti = FlatIdx(p.x, p.y, w);
      if (ti < n) out.corridorMask[ti] = 1;
    }
  }

  // Corridor coverage: what fraction of commute-demand is on served edges.
  {
    std::vector<std::uint32_t> commuteFlow(n, 0);
    if (in.traffic && in.traffic->roadTraffic.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        commuteFlow[i] = static_cast<std::uint32_t>(in.traffic->roadTraffic[i]);
      }
    } else {
      // If traffic isn't available, approximate with the served corridor mask (uniform demand on roads).
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const Tile& t = world.at(x, y);
          if (t.overlay == Overlay::Road && t.terrain != Terrain::Water) {
            commuteFlow[FlatIdx(x, y, w)] = 1;
          }
        }
      }
    }
    const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(world, *g, commuteFlow);
    std::uint64_t total = 0;
    std::uint64_t covered = 0;
    for (std::size_t ei = 0; ei < g->edges.size() && ei < agg.edges.size(); ++ei) {
      const std::uint64_t d = agg.edges[ei].sumTrafficInterior;
      total += d;
      if (servedEdge[ei]) covered += d;
    }
    out.corridorCoverage = (total > 0) ? static_cast<float>(static_cast<double>(covered) / static_cast<double>(total)) : 0.0f;
  }

  // Stop generation.
  const int stopSpacing = std::max(2, out.cfg.stopSpacingTiles);
  std::vector<int> stopRoadIdx;
  stopRoadIdx.reserve(plan->lines.size() * 8);
  std::unordered_set<std::uint64_t> stopSeen;
  stopSeen.reserve(plan->lines.size() * 32);

  for (const TransitLine& line : plan->lines) {
    std::vector<Point> stops;
    if (!BuildTransitLineStopTiles(*g, line, stopSpacing, stops)) continue;
    for (const Point& s : stops) {
      if (!world.inBounds(s.x, s.y)) continue;
      const Tile& t = world.at(s.x, s.y);
      if (t.overlay != Overlay::Road) continue;
      const std::uint64_t key = HashStopKey(s.x, s.y);
      if (stopSeen.insert(key).second) {
        const std::size_t idx = FlatIdx(s.x, s.y, w);
        if (idx < n) {
          stopRoadIdx.push_back(static_cast<int>(idx));
          out.stopMask[idx] = 1;
        }
      }
    }
  }

  out.plannedStops = static_cast<int>(stopRoadIdx.size());
  if (stopRoadIdx.empty()) {
    return out;
  }

  // Isochrone field from stops.
  RoadIsochroneConfig icfg;
  icfg.requireOutsideConnection = out.cfg.requireOutsideConnection;
  icfg.weightMode = IsochroneWeightMode::Steps;

  const RoadIsochroneField stopField = BuildRoadIsochroneField(world, stopRoadIdx, icfg, roadToEdge, nullptr);

  // Zone access map (for interior zone tiles).
  ZoneAccessMap zoneAccessLocal;
  const ZoneAccessMap* zoneAccess = nullptr;
  if (in.zoneAccess) {
    zoneAccess = in.zoneAccess;
  } else {
    zoneAccessLocal = BuildZoneAccessMap(world, roadToEdge);
    zoneAccess = &zoneAccessLocal;
  }

  // Map stopField.steps onto all tiles.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) {
        out.stepsToStop[i] = -1;
        continue;
      }

      int s = -1;
      if (t.overlay == Overlay::Road) {
        if (i < stopField.steps.size()) s = stopField.steps[i];
      } else if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                 t.overlay == Overlay::Industrial || t.overlay == Overlay::School ||
                 t.overlay == Overlay::Hospital || t.overlay == Overlay::PoliceStation ||
                 t.overlay == Overlay::FireStation) {
        // Prefer zone-access mapping (supports interior tiles in a connected component).
        if (zoneAccess && zoneAccess->roadIdx.size() == n) {
          const int ridx = zoneAccess->roadIdx[i];
          if (ridx >= 0 && static_cast<std::size_t>(ridx) < stopField.steps.size()) {
            s = stopField.steps[static_cast<std::size_t>(ridx)];
          }
        }

        if (s < 0) {
          // Fallback: adjacent road.
          s = BestAdjacentRoadSteps(world, stopField, x, y);
        }
      } else {
        // Non-zone tiles: allow adjacency access (parks, etc.).
        s = BestAdjacentRoadSteps(world, stopField, x, y);
      }

      out.stepsToStop[i] = s;
      out.access01[i] = StepsToAccess01(s, out.cfg.goodSteps, out.cfg.badSteps);
    }
  }

  // Access shares (res/jobs) and per-tile mode share potential.
  double resTotal = 0.0;
  double resServed = 0.0;
  double jobsTotal = 0.0;
  double jobsServed = 0.0;

  auto isServed = [&](int steps) -> bool {
    return (steps >= 0 && steps <= std::max(0, out.cfg.walkRadiusSteps));
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) continue;
      if (!(t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial)) continue;

      const double occ = static_cast<double>(std::max(0, static_cast<int>(t.occupants)));
      const int s = out.stepsToStop[i];
      const bool served = isServed(s);

      if (t.overlay == Overlay::Residential) {
        resTotal += occ;
        if (served) resServed += occ;
      } else {
        jobsTotal += occ;
        if (served) jobsServed += occ;
      }
    }
  }

  out.resStopAccessShare = (resTotal > 0.0) ? static_cast<float>(resServed / resTotal) : 0.0f;
  out.jobsStopAccessShare = (jobsTotal > 0.0) ? static_cast<float>(jobsServed / jobsTotal) : 0.0f;
  out.accessCoverage = std::sqrt(std::max(0.0f, out.resStopAccessShare) * std::max(0.0f, out.jobsStopAccessShare));
  out.overallCoverage = out.corridorCoverage * out.accessCoverage;

  const float maxShare = std::clamp(out.cfg.maxModeShare, 0.0f, 1.0f);
  const float travelMult = std::max(0.05f, out.cfg.travelTimeMultiplier);
  const float attractiveness = std::max(0.0f, out.cfg.serviceLevel) / travelMult;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) continue;

      float otherShare = 0.0f;
      if (t.overlay == Overlay::Residential) {
        otherShare = out.jobsStopAccessShare;
      } else if (t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
        otherShare = out.resStopAccessShare;
      } else {
        continue;
      }

      const int steps = out.stepsToStop[i];
      const int r = std::max(1, out.cfg.walkRadiusSteps);
      float tileWalk01 = 0.0f;
      if (steps >= 0 && steps <= r) {
        // 1 at steps==0, ~0 at steps==r.
        tileWalk01 = 1.0f - static_cast<float>(steps) / static_cast<float>(r);
      }

      // Local coverage: corridor coverage * geometric mean(origin/destination access).
      const float localAccess = std::sqrt(std::max(0.0f, tileWalk01) * std::max(0.0f, otherShare));
      const float cov = out.corridorCoverage * localAccess;
      const float base = cov * attractiveness;

      float modeShare = 0.0f;
      if (maxShare > 1e-6f && base > 0.0f) {
        // Match simulator's saturating curve: maxShare * (1 - exp(-k*base))
        const float k = 1.2f;
        modeShare = maxShare * (1.0f - std::exp(-k * base));
      }

      // Normalize to [0,1] by maxShare for visualization.
      out.modeSharePotential01[i] = (maxShare > 1e-6f) ? Clamp01(modeShare / maxShare) : 0.0f;
    }
  }

  return out;
}

} // namespace isocity
