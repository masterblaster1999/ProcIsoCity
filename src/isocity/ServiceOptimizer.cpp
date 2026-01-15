#include "isocity/ServiceOptimizer.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_set>

namespace isocity {

namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline int ClampLevel(int lvl) { return std::clamp(lvl, 1, 3); }

bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == n;
}

float DistanceWeight(const ServicesModelSettings& cfg, int costMilli, int radiusMilli)
{
  if (radiusMilli <= 0) return 0.0f;
  if (costMilli < 0 || costMilli > radiusMilli) return 0.0f;

  const float frac = static_cast<float>(costMilli) / static_cast<float>(radiusMilli);
  const float cut0 = std::clamp(cfg.distanceBandCutFrac[0], 0.0f, 1.0f);
  const float cut1 = std::clamp(cfg.distanceBandCutFrac[1], cut0, 1.0f);

  if (frac <= cut0) return cfg.distanceBandWeight[0];
  if (frac <= cut1) return cfg.distanceBandWeight[1];
  return cfg.distanceBandWeight[2];
}

float BaseDemandForTile(const Tile& t, const ServicesModelSettings& cfg)
{
  bool ok = false;
  if (t.overlay == Overlay::Residential) ok = cfg.demandResidential;
  else if (t.overlay == Overlay::Commercial) ok = cfg.demandCommercial;
  else if (t.overlay == Overlay::Industrial) ok = cfg.demandIndustrial;

  if (!ok) return 0.0f;

  if (cfg.demandMode == ServiceDemandMode::Tiles) return 1.0f;
  return static_cast<float>(t.occupants);
}

float DemandMultForService(const ServicesModelSettings& cfg, ServiceType t)
{
  switch (t) {
    case ServiceType::Education: return cfg.educationDemandMult;
    case ServiceType::Health: return cfg.healthDemandMult;
    case ServiceType::Safety: return cfg.safetyDemandMult;
  }
  return 1.0f;
}

int SupplyForService(const ServicesModelSettings& cfg, ServiceType t, int level)
{
  const int li = ClampLevel(level) - 1;
  switch (t) {
    case ServiceType::Education: return std::max(0, cfg.educationSupplyPerLevel[li]);
    case ServiceType::Health: return std::max(0, cfg.healthSupplyPerLevel[li]);
    case ServiceType::Safety: return std::max(0, cfg.safetySupplyPerLevel[li]);
  }
  return 0;
}

float AccessToSatisfaction(float access, float targetAccess)
{
  if (!(access > 0.0f)) return 0.0f;
  if (!(targetAccess > 0.0f)) return Clamp01(access);

  // Set k so access==targetAccess yields ~0.5.
  const float k = static_cast<float>(std::log(2.0)) / targetAccess;
  const float sat = 1.0f - static_cast<float>(std::exp(-access * k));
  return Clamp01(sat);
}

struct LocalSearchScratch {
  // We keep arrays sized to the full world and reset only visited nodes.
  std::vector<int> distSteps;
  std::vector<int> costMilli;
  std::vector<int> heapCost;
  std::vector<int> heapSteps;

  std::vector<int> queue;
  std::vector<int> visited;

  struct HeapNode {
    int cost = 0;
    int steps = 0;
    int idx = 0;
  };

  struct HeapCmp {
    bool operator()(const HeapNode& a, const HeapNode& b) const
    {
      if (a.cost != b.cost) return a.cost > b.cost;
      if (a.steps != b.steps) return a.steps > b.steps;
      return a.idx > b.idx;
    }
  };

  std::priority_queue<HeapNode, std::vector<HeapNode>, HeapCmp> heap;

  void ensureSize(std::size_t n)
  {
    if (distSteps.size() != n) distSteps.assign(n, -1);
    if (costMilli.size() != n) costMilli.assign(n, -1);
    if (heapCost.size() != n) heapCost.assign(n, std::numeric_limits<int>::max() / 4);
    if (heapSteps.size() != n) heapSteps.assign(n, std::numeric_limits<int>::max() / 4);
  }

  void resetVisited()
  {
    for (int idx : visited) {
      const std::size_t u = static_cast<std::size_t>(idx);
      if (u < distSteps.size()) distSteps[u] = -1;
      if (u < costMilli.size()) costMilli[u] = -1;
      if (u < heapCost.size()) heapCost[u] = std::numeric_limits<int>::max() / 4;
      if (u < heapSteps.size()) heapSteps[u] = std::numeric_limits<int>::max() / 4;
    }
    visited.clear();
    queue.clear();
    while (!heap.empty()) heap.pop();
  }
};

inline bool IsTraversableRoad(const World& world, const std::vector<std::uint8_t>* roadToEdge, int ridx)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) return false;
  const int x = ridx % w;
  const int y = ridx / w;
  if (!world.inBounds(x, y)) return false;
  if (world.at(x, y).overlay != Overlay::Road) return false;
  if (roadToEdge && roadToEdge->size() == n) {
    if ((*roadToEdge)[static_cast<std::size_t>(ridx)] == 0) return false;
  }
  return true;
}

// Enumerate road tiles reachable within radiusMilli from a single road source.
//
// Calls cb(idx, costMilli, steps) for every visited road tile (including the source).
template <typename Callback>
void EnumerateRoadWithinRadius(const World& world,
                               const ServicesModelSettings& cfg,
                               int sourceRoadIdx,
                               int radiusMilli,
                               const std::vector<std::uint8_t>* roadToEdge,
                               LocalSearchScratch& scratch,
                               Callback&& cb)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  scratch.ensureSize(n);
  scratch.resetVisited();

  if (!IsTraversableRoad(world, roadToEdge, sourceRoadIdx)) return;

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // N,E,S,W

  const bool useTravelTime = (cfg.weightMode == IsochroneWeightMode::TravelTime);

  if (!useTravelTime) {
    // --- unweighted BFS (steps), deterministic neighbor order ---
    scratch.queue.push_back(sourceRoadIdx);
    scratch.visited.push_back(sourceRoadIdx);
    scratch.distSteps[static_cast<std::size_t>(sourceRoadIdx)] = 0;
    scratch.costMilli[static_cast<std::size_t>(sourceRoadIdx)] = 0;

    std::size_t head = 0;
    while (head < scratch.queue.size()) {
      const int u = scratch.queue[head++];
      const std::size_t uu = static_cast<std::size_t>(u);
      const int ux = u % w;
      const int uy = u / w;
      const int ccur = scratch.costMilli[uu];
      const int dcur = scratch.distSteps[uu];

      if (ccur > radiusMilli) continue;

      cb(u, ccur, dcur);

      for (const auto& d : dirs) {
        const int nx = ux + d[0];
        const int ny = uy + d[1];
        if (!world.inBounds(nx, ny)) continue;
        const int nidx = ny * w + nx;
        if (!IsTraversableRoad(world, roadToEdge, nidx)) continue;
        const std::size_t nu = static_cast<std::size_t>(nidx);
        if (scratch.distSteps[nu] != -1) continue;

        const Tile& nt = world.at(nx, ny);
        const int lvl = static_cast<int>(nt.level);
        int moveCost = (nt.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(lvl)
                                                      : RoadTravelTimeMilliForLevel(lvl);
        const int ncost = ccur + moveCost;

        scratch.distSteps[nu] = dcur + 1;
        scratch.costMilli[nu] = ncost;
        scratch.queue.push_back(nidx);
        scratch.visited.push_back(nidx);
      }
    }
    return;
  }

  // --- weighted Dijkstra (travel time) ---
  constexpr int INF = std::numeric_limits<int>::max() / 4;
  const std::size_t su = static_cast<std::size_t>(sourceRoadIdx);
  scratch.heapCost[su] = 0;
  scratch.heapSteps[su] = 0;
  scratch.visited.push_back(sourceRoadIdx);
  scratch.heap.push(LocalSearchScratch::HeapNode{0, 0, sourceRoadIdx});

  while (!scratch.heap.empty()) {
    const auto cur = scratch.heap.top();
    scratch.heap.pop();
    const std::size_t uu = static_cast<std::size_t>(cur.idx);
    if (uu >= n) continue;
    if (cur.cost != scratch.heapCost[uu] || cur.steps != scratch.heapSteps[uu]) continue;
    if (cur.cost > radiusMilli) break;

    cb(cur.idx, cur.cost, cur.steps);

    const int ux = cur.idx % w;
    const int uy = cur.idx / w;
    for (const auto& d : dirs) {
      const int nx = ux + d[0];
      const int ny = uy + d[1];
      if (!world.inBounds(nx, ny)) continue;
      const int nidx = ny * w + nx;
      if (!IsTraversableRoad(world, roadToEdge, nidx)) continue;
      const std::size_t nu = static_cast<std::size_t>(nidx);

      const Tile& nt = world.at(nx, ny);
      const int lvl = static_cast<int>(nt.level);
      int moveCost = (nt.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(lvl)
                                                    : RoadTravelTimeMilliForLevel(lvl);

      const int ncost = cur.cost + moveCost;
      const int nsteps = cur.steps + 1;
      if (ncost > radiusMilli) continue;

      bool improve = false;
      if (ncost < scratch.heapCost[nu]) {
        improve = true;
      } else if (ncost == scratch.heapCost[nu]) {
        if (nsteps < scratch.heapSteps[nu]) improve = true;
      }

      if (!improve) continue;

      if (scratch.heapCost[nu] == INF) {
        scratch.visited.push_back(nidx);
      }
      scratch.heapCost[nu] = ncost;
      scratch.heapSteps[nu] = nsteps;
      scratch.heap.push(LocalSearchScratch::HeapNode{ncost, nsteps, nidx});
    }
  }
}

double ObjectiveGainForCandidate(const World& world,
                                 const ServicesModelSettings& modelCfg,
                                 ServiceType type,
                                 int facilityLevel,
                                 int accessRoadIdx,
                                 int radiusMilli,
                                 const std::vector<std::uint8_t>* roadToEdge,
                                 const std::vector<double>& demandOnRoad,
                                 const std::vector<double>& accessOnRoad,
                                 LocalSearchScratch& scratch,
                                 double* outLocalDemandSum,
                                 double* outRatio)
{
  if (outLocalDemandSum) *outLocalDemandSum = 0.0;
  if (outRatio) *outRatio = 0.0;

  const int supply = SupplyForService(modelCfg, type, facilityLevel);
  if (supply <= 0) return 0.0;

  // Collect visited demand nodes and weights so we can compute demandSum first.
  struct NodeW {
    int idx = 0;
    float wgt = 0.0f;
  };
  std::vector<NodeW> nodes;
  nodes.reserve(256);

  double demandSum = 0.0;

  EnumerateRoadWithinRadius(world, modelCfg, accessRoadIdx, radiusMilli, roadToEdge, scratch,
                            [&](int idx, int costMilli, int /*steps*/) {
                              const float wgt = DistanceWeight(modelCfg, costMilli, radiusMilli);
                              if (!(wgt > 0.0f)) return;
                              const std::size_t u = static_cast<std::size_t>(idx);
                              if (u >= demandOnRoad.size()) return;
                              const double dem = demandOnRoad[u];
                              if (!(dem > 0.0)) return;
                              nodes.push_back(NodeW{idx, wgt});
                              demandSum += dem * static_cast<double>(wgt);
                            });

  if (!(demandSum > 0.0)) return 0.0;

  const double ratio = static_cast<double>(supply) / demandSum;
  if (outLocalDemandSum) *outLocalDemandSum = demandSum;
  if (outRatio) *outRatio = ratio;

  const float targetAccess = modelCfg.targetAccess;

  double gain = 0.0;
  for (const NodeW& nw : nodes) {
    const std::size_t u = static_cast<std::size_t>(nw.idx);
    if (u >= demandOnRoad.size() || u >= accessOnRoad.size()) continue;
    const double dem = demandOnRoad[u];
    const double curAccess = accessOnRoad[u];
    const double add = ratio * static_cast<double>(nw.wgt);
    const float sat0 = AccessToSatisfaction(static_cast<float>(curAccess), targetAccess);
    const float sat1 = AccessToSatisfaction(static_cast<float>(curAccess + add), targetAccess);
    const double delta = static_cast<double>(sat1 - sat0);
    if (!(delta > 0.0)) continue;
    gain += dem * delta;
  }

  return gain;
}

void ApplyFacilityContribution(const World& world,
                               const ServicesModelSettings& modelCfg,
                               ServiceType type,
                               int facilityLevel,
                               int accessRoadIdx,
                               int radiusMilli,
                               const std::vector<std::uint8_t>* roadToEdge,
                               const std::vector<double>& demandOnRoad,
                               std::vector<double>& accessOnRoad,
                               LocalSearchScratch& scratch)
{
  const int supply = SupplyForService(modelCfg, type, facilityLevel);
  if (supply <= 0) return;

  // First pass: compute facility-local demand sum.
  double demandSum = 0.0;
  EnumerateRoadWithinRadius(world, modelCfg, accessRoadIdx, radiusMilli, roadToEdge, scratch,
                            [&](int idx, int costMilli, int /*steps*/) {
                              const float wgt = DistanceWeight(modelCfg, costMilli, radiusMilli);
                              if (!(wgt > 0.0f)) return;
                              const std::size_t u = static_cast<std::size_t>(idx);
                              if (u >= demandOnRoad.size()) return;
                              const double dem = demandOnRoad[u];
                              if (!(dem > 0.0)) return;
                              demandSum += dem * static_cast<double>(wgt);
                            });
  if (!(demandSum > 0.0)) return;
  const double ratio = static_cast<double>(supply) / demandSum;

  // Second pass: distribute ratio onto demand nodes.
  EnumerateRoadWithinRadius(world, modelCfg, accessRoadIdx, radiusMilli, roadToEdge, scratch,
                            [&](int idx, int costMilli, int /*steps*/) {
                              const float wgt = DistanceWeight(modelCfg, costMilli, radiusMilli);
                              if (!(wgt > 0.0f)) return;
                              const std::size_t u = static_cast<std::size_t>(idx);
                              if (u >= demandOnRoad.size() || u >= accessOnRoad.size()) return;
                              const double dem = demandOnRoad[u];
                              if (!(dem > 0.0)) return;
                              accessOnRoad[u] += ratio * static_cast<double>(wgt);
                            });
}

} // namespace

std::vector<ServiceFacility> FacilitiesFromPlacements(const std::vector<ServicePlacement>& placements)
{
  std::vector<ServiceFacility> out;
  out.reserve(placements.size());
  for (const auto& p : placements) out.push_back(p.facility);
  return out;
}

ServiceOptimizerResult SuggestServiceFacilities(const World& world,
                                               const ServiceOptimizerConfig& cfg,
                                               const std::vector<ServiceFacility>& existingFacilities,
                                               const ZoneAccessMap* precomputedZoneAccess,
                                               const std::vector<std::uint8_t>* precomputedRoadToEdge)
{
  ServiceOptimizerResult out;
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  // Outside connection mask.
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (cfg.modelCfg.requireOutsideConnection) {
    if (MaskUsable(precomputedRoadToEdge, w, h)) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      roadToEdgeOwned.assign(n, 0);
      ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
      roadToEdge = &roadToEdgeOwned;
    }
  }

  // Zone access map.
  ZoneAccessMap zamOwned;
  const ZoneAccessMap* zam = nullptr;
  if (precomputedZoneAccess && precomputedZoneAccess->w == w && precomputedZoneAccess->h == h &&
      precomputedZoneAccess->roadIdx.size() == n) {
    zam = precomputedZoneAccess;
  } else {
    zamOwned = BuildZoneAccessMap(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr);
    if (zamOwned.w == w && zamOwned.h == h && zamOwned.roadIdx.size() == n) {
      zam = &zamOwned;
    }
  }
  if (!zam) return out;

  // Demand aggregated onto road tiles.
  std::vector<double> demandOnRoad(n, 0.0);
  const float demandMult = DemandMultForService(cfg.modelCfg, cfg.type);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);
      const float bd = BaseDemandForTile(t, cfg.modelCfg);
      if (!(bd > 0.0f)) continue;
      if (!(demandMult > 0.0f)) continue;
      const int ridx = zam->roadIdx[idx];
      if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) continue;
      demandOnRoad[static_cast<std::size_t>(ridx)] += static_cast<double>(bd) * static_cast<double>(demandMult);
      out.totalDemandWeight += static_cast<std::uint64_t>(std::lround(static_cast<double>(bd) * 1000.0));
    }
  }

  // Candidate facility tile per road access point.
  std::vector<int> candidateTileIdx(n, -1);
  auto isEmptyLand = [&](int x, int y) -> bool {
    if (!world.inBounds(x, y)) return false;
    const Tile& t = world.at(x, y);
    return (t.terrain != Terrain::Water) && (t.overlay == Overlay::None);
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& rt = world.at(x, y);
      if (rt.overlay != Overlay::Road) continue;
      const int ridx = y * w + x;
      if (!IsTraversableRoad(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, ridx)) continue;

      // Default: no candidate.
      int pick = -1;

      // Prefer empty land adjacent to the road.
      if (cfg.requireEmptyLand) {
        constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // N,E,S,W
        for (const auto& d : dirs) {
          const int nx = x + d[0];
          const int ny = y + d[1];
          if (!isEmptyLand(nx, ny)) continue;

          if (cfg.requireStableAccessRoad) {
            Point access{-1, -1};
            if (!PickAdjacentRoadTile(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, nx, ny, access)) {
              continue;
            }
            if (access.x != x || access.y != y) continue;
          }

          pick = static_cast<int>(FlatIdx(nx, ny, w));
          break;
        }
      } else {
        pick = ridx; // place directly on the road tile.
      }

      candidateTileIdx[static_cast<std::size_t>(ridx)] = pick;
    }
  }

  // Build candidate list sorted by local-demand score.
  struct Cand {
    int ridx = 0;
    double score = 0.0;
  };

  std::vector<Cand> cands;
  cands.reserve(n / 8u);

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int ridx = y * w + x;
      const std::size_t ur = static_cast<std::size_t>(ridx);
      if (ur >= candidateTileIdx.size()) continue;
      const int tileIdx = candidateTileIdx[ur];
      if (tileIdx < 0) continue;

      if (!IsTraversableRoad(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, ridx)) continue;

      // Base score: demand mapped onto this road access point.
      double s = (ur < demandOnRoad.size()) ? demandOnRoad[ur] : 0.0;

      // Add a small neighborhood blur so candidates slightly away from the heaviest road can still win.
      for (const auto& d : dirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (!world.inBounds(nx, ny)) continue;
        const int nidx = ny * w + nx;
        const std::size_t un = static_cast<std::size_t>(nidx);
        if (un >= demandOnRoad.size()) continue;
        s += 0.5 * demandOnRoad[un];
      }

      cands.push_back(Cand{ridx, s});
    }
  }

  std::stable_sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
    if (a.score != b.score) return a.score > b.score;
    return a.ridx < b.ridx;
  });

  if (cfg.candidateLimit > 0 && static_cast<int>(cands.size()) > cfg.candidateLimit) {
    cands.resize(static_cast<std::size_t>(cfg.candidateLimit));
  }

  // Seed access field with existing facilities.
  std::vector<double> accessOnRoad(n, 0.0);
  std::vector<Point> existingAccessRoads;

  const int radiusMilli = std::max(0, cfg.modelCfg.catchmentRadiusSteps) * 1000;
  LocalSearchScratch scratch;

  auto addExistingFacility = [&](const ServiceFacility& f) {
    if (!f.enabled) return;
    if (cfg.considerOnlySameTypeExisting && f.type != cfg.type) return;
    if (!world.inBounds(f.tile.x, f.tile.y)) return;

    Point access{0, 0};
    const Tile& ft = world.at(f.tile.x, f.tile.y);
    bool hasAccess = false;
    if (ft.overlay == Overlay::Road) {
      access = f.tile;
      hasAccess = true;
    } else {
      hasAccess = PickAdjacentRoadTile(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, f.tile.x, f.tile.y, access);
    }
    if (!hasAccess) return;

    const int aidx = access.y * w + access.x;
    if (!IsTraversableRoad(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, aidx)) return;

    ApplyFacilityContribution(world, cfg.modelCfg, cfg.type, static_cast<int>(f.level), aidx, radiusMilli,
                              MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, demandOnRoad, accessOnRoad, scratch);
    existingAccessRoads.push_back(access);
    out.existingFacilities++;
  };

  for (const auto& f : existingFacilities) addExistingFacility(f);

  auto tooCloseToExisting = [&](int roadIdx) -> bool {
    if (cfg.minSeparationMilli <= 0) return false;
    const int rx = roadIdx % w;
    const int ry = roadIdx / w;
    for (const Point& p : existingAccessRoads) {
      const int man = std::abs(rx - p.x) + std::abs(ry - p.y);
      const int approxMilli = man * 1000;
      if (approxMilli < cfg.minSeparationMilli) return true;
    }
    return false;
  };

  std::unordered_set<int> usedAccessRoad;
  usedAccessRoad.reserve(static_cast<std::size_t>(cfg.facilitiesToAdd) * 2u);

  std::unordered_set<int> usedFacilityTile;
  usedFacilityTile.reserve(static_cast<std::size_t>(cfg.facilitiesToAdd) * 2u);

  // Greedy selection.
  const int toAdd = std::max(0, cfg.facilitiesToAdd);
  for (int iter = 0; iter < toAdd; ++iter) {
    int bestRoad = -1;
    double bestGain = 0.0;
    double bestLocalDemand = 0.0;
    double bestRatio = 0.0;

    for (const Cand& c : cands) {
      const int ridx = c.ridx;
      if (usedAccessRoad.find(ridx) != usedAccessRoad.end()) continue;
      if (tooCloseToExisting(ridx)) continue;

      const std::size_t ur = static_cast<std::size_t>(ridx);
      if (ur >= candidateTileIdx.size()) continue;
      const int tileIdx = candidateTileIdx[ur];
      if (tileIdx < 0) continue;
      if (cfg.requireEmptyLand && usedFacilityTile.find(tileIdx) != usedFacilityTile.end()) continue;

      double localDem = 0.0;
      double ratio = 0.0;
      const double gain = ObjectiveGainForCandidate(world, cfg.modelCfg, cfg.type, static_cast<int>(cfg.facilityLevel),
                                                   ridx, radiusMilli, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr,
                                                   demandOnRoad, accessOnRoad, scratch, &localDem, &ratio);

      constexpr double kEps = 1e-12;
      if (gain > bestGain + kEps || (std::abs(gain - bestGain) <= kEps && ridx < bestRoad)) {
        bestGain = gain;
        bestRoad = ridx;
        bestLocalDemand = localDem;
        bestRatio = ratio;
      }
    }

    if (bestRoad < 0 || !(bestGain > 0.0)) break;

    const std::size_t ubr = static_cast<std::size_t>(bestRoad);
    if (ubr >= candidateTileIdx.size()) break;
    const int tileIdx = candidateTileIdx[ubr];
    if (tileIdx < 0) break;

    const int fx = tileIdx % w;
    const int fy = tileIdx / w;
    const int ax = bestRoad % w;
    const int ay = bestRoad / w;

    ServicePlacement p;
    p.facility.tile = Point{fx, fy};
    p.facility.type = cfg.type;
    p.facility.level = static_cast<std::uint8_t>(ClampLevel(static_cast<int>(cfg.facilityLevel)));
    p.facility.enabled = true;
    p.accessRoad = Point{ax, ay};
    p.marginalGain = bestGain;
    p.localDemandSum = bestLocalDemand;
    p.ratio = bestRatio;
    out.placements.push_back(p);

    // Update access field.
    ApplyFacilityContribution(world, cfg.modelCfg, cfg.type, static_cast<int>(cfg.facilityLevel), bestRoad, radiusMilli,
                              MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, demandOnRoad, accessOnRoad, scratch);

    usedAccessRoad.insert(bestRoad);
    usedFacilityTile.insert(tileIdx);
    existingAccessRoads.push_back(Point{ax, ay});
  }

  return out;
}

} // namespace isocity
