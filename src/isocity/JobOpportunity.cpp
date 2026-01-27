#include "isocity/JobOpportunity.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline float Clamp01(float v)
{
  return std::clamp(v, 0.0f, 1.0f);
}

// Compute a simple percentile (q in [0,1]) of a sample set.
// Returns 0 when samples is empty.
float Percentile(std::vector<float> samples, float q)
{
  if (samples.empty()) return 0.0f;
  q = std::clamp(q, 0.0f, 1.0f);
  std::sort(samples.begin(), samples.end());
  const std::size_t n = samples.size();
  const std::size_t idx = static_cast<std::size_t>(std::floor(q * static_cast<float>(n - 1)));
  return samples[std::min(idx, n - 1)];
}

struct AdjRoadPick {
  int roadIdx = -1;
  int costMilli = -1;
};

AdjRoadPick PickBestAdjacentRoad(const World& world, int x, int y, const RoadFlowField& ff)
{
  AdjRoadPick out;
  if (ff.w <= 0 || ff.h <= 0) return out;
  const int w = ff.w;
  const int h = ff.h;
  if (x < 0 || y < 0 || x >= w || y >= h) return out;

  constexpr int kDirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  int bestCost = std::numeric_limits<int>::max();
  int bestIdx = -1;

  for (int d = 0; d < 4; ++d) {
    const int nx = x + kDirs[d][0];
    const int ny = y + kDirs[d][1];
    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
    const Tile& t = world.at(nx, ny);
    if (t.overlay != Overlay::Road) continue;
    const int ridx = ny * w + nx;
    if (static_cast<std::size_t>(ridx) >= ff.cost.size()) continue;
    const int c = ff.cost[static_cast<std::size_t>(ridx)];
    if (c < 0) continue;
    if (c < bestCost) {
      bestCost = c;
      bestIdx = ridx;
    }
  }

  if (bestIdx >= 0 && bestCost != std::numeric_limits<int>::max()) {
    out.roadIdx = bestIdx;
    out.costMilli = bestCost;
  }
  return out;
}

} // namespace

JobOpportunityResult ComputeJobOpportunity(const World& world, const JobOpportunityConfig& cfg,
                                          const TrafficResult* traffic,
                                          const std::vector<std::uint8_t>* precomputedRoadToEdge,
                                          const ZoneAccessMap* precomputedZoneAccess)
{
  JobOpportunityResult out;
  out.cfg = cfg;
  out.w = world.width();
  out.h = world.height();

  const int w = out.w;
  const int h = out.h;
  if (!cfg.enabled || w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.jobAccessCostMilli.assign(n, -1);
  out.jobAccess01.assign(n, 0.0f);
  out.jobOpportunityRaw.assign(n, 0.0f);
  out.jobOpportunity01.assign(n, 0.0f);
  out.roadOpportunityRaw.assign(n, 0.0f);

  // ---- Outside-connection mask (optional) ----
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;

  if (cfg.requireOutsideConnection) {
    if (precomputedRoadToEdge && precomputedRoadToEdge->size() == n) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      ComputeRoadsConnectedToEdge(world, roadToEdgeLocal);
      if (roadToEdgeLocal.size() == n) roadToEdge = &roadToEdgeLocal;
    }
  }

  // ---- Zone access mapping ----
  ZoneAccessMap zoneAccessLocal;
  const ZoneAccessMap* zoneAccess = precomputedZoneAccess;
  if (!zoneAccess || zoneAccess->w != w || zoneAccess->h != h || zoneAccess->roadIdx.size() != n) {
    zoneAccessLocal = BuildZoneAccessMap(world, roadToEdge);
    zoneAccess = &zoneAccessLocal;
  }

  // ---- Build job sources on road tiles ----
  // Accumulate job capacity onto the road access point for each job zone tile.
  std::vector<int> jobsOnRoad(n, 0);
  int totalJobs = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);

      int cap = 0;
      if (t.overlay == Overlay::Commercial && cfg.includeCommercialJobs) {
        cap = JobsCommercialForLevel(static_cast<int>(t.level));
      } else if (t.overlay == Overlay::Industrial && cfg.includeIndustrialJobs) {
        cap = JobsIndustrialForLevel(static_cast<int>(t.level));
      } else {
        continue;
      }

      if (cap <= 0) continue;
      if (idx >= zoneAccess->roadIdx.size()) continue;
      const int roadIdx = zoneAccess->roadIdx[idx];
      if (roadIdx < 0 || static_cast<std::size_t>(roadIdx) >= n) continue;

      jobsOnRoad[static_cast<std::size_t>(roadIdx)] += cap;
      totalJobs += cap;
    }
  }

  out.jobSourceCapacity = totalJobs;

  std::vector<int> sourceRoadIdx;
  sourceRoadIdx.reserve(n / 8);
  for (std::size_t i = 0; i < n; ++i) {
    if (jobsOnRoad[i] > 0) sourceRoadIdx.push_back(static_cast<int>(i));
  }
  out.jobSourceRoadTiles = static_cast<int>(sourceRoadIdx.size());

  // If there are no job sources, return an all-zero result.
  if (sourceRoadIdx.empty()) return out;

  // ---- Optional congestion-aware per-road tile extra costs ----
  std::vector<int> extraCostMilli;
  const std::vector<int>* extraCostPtr = nullptr;

  if (traffic && cfg.congestionCosts && traffic->roadTraffic.size() == n) {
    extraCostMilli.assign(n, 0);

    const float capScale = std::max(0.01f, cfg.congestionCapacityScale);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;
        if (roadToEdge && idx < roadToEdge->size() && (*roadToEdge)[idx] == 0) continue;

        // Base tile travel time.
        int baseMilli = (t.terrain == Terrain::Water)
                            ? RoadBridgeTravelTimeMilliForLevel(static_cast<int>(t.level))
                            : RoadTravelTimeMilliForLevel(static_cast<int>(t.level));

        // Capacity derived from road class.
        float cap = static_cast<float>(RoadCapacityForLevel(cfg.roadTileCapacity, static_cast<int>(t.level)));
        cap *= capScale;
        cap = std::max(1.0f, cap);

        const float v = static_cast<float>(traffic->roadTraffic[idx]);
        float ratio = v / cap;
        ratio = std::min(ratio, cfg.congestionRatioClamp);
        ratio = std::max(0.0f, ratio);

        const float mult = 1.0f + cfg.congestionAlpha * std::pow(ratio, cfg.congestionBeta);
        const int add = static_cast<int>(std::lround((mult - 1.0f) * static_cast<float>(baseMilli)));
        extraCostMilli[idx] = std::max(0, add);
      }
    }

    extraCostPtr = &extraCostMilli;
  }

  // ---- Nearest-job access cost: road multi-source flow field ----
  RoadFlowFieldConfig fcfg;
  fcfg.requireOutsideConnection = cfg.requireOutsideConnection;
  fcfg.computeOwner = false;
  fcfg.useTravelTime = cfg.useTravelTime;

  const RoadFlowField field = BuildRoadFlowField(world, sourceRoadIdx, fcfg, roadToEdge, extraCostPtr);

  // Map road cost to all tiles.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);

      int c = -1;
      if (t.overlay == Overlay::Road) {
        if (idx < field.cost.size()) c = field.cost[idx];
      } else {
        const int ridx = (idx < zoneAccess->roadIdx.size()) ? zoneAccess->roadIdx[idx] : -1;
        if (ridx >= 0 && static_cast<std::size_t>(ridx) < field.cost.size()) {
          const int rc = field.cost[static_cast<std::size_t>(ridx)];
          if (rc >= 0) c = rc + cfg.accessStepCostMilli;
        } else {
          // Fallback: pick best adjacent road.
          const AdjRoadPick pick = PickBestAdjacentRoad(world, x, y, field);
          if (pick.roadIdx >= 0 && pick.costMilli >= 0) c = pick.costMilli + cfg.accessStepCostMilli;
        }
      }

      out.jobAccessCostMilli[idx] = c;
    }
  }

  // Normalize access cost -> accessibility score.
  const int ideal = std::max(0, cfg.idealAccessCostMilli);
  const int maxv = std::max(ideal + 1, cfg.maxAccessCostMilli);
  for (std::size_t i = 0; i < n; ++i) {
    const int c = out.jobAccessCostMilli[i];
    float s = 0.0f;
    if (c >= 0) {
      if (c <= ideal) {
        s = 1.0f;
      } else if (c >= maxv) {
        s = 0.0f;
      } else {
        s = 1.0f - (static_cast<float>(c - ideal) / static_cast<float>(maxv - ideal));
      }
    }
    out.jobAccess01[i] = Clamp01(s);
  }

  // ---- Opportunity diffusion on the road network ----
  // Precompute traversable road mask.
  std::vector<std::uint8_t> traversableRoad(n, std::uint8_t{0});
  int roadCount = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      if (roadToEdge && idx < roadToEdge->size() && (*roadToEdge)[idx] == 0) continue;
      traversableRoad[idx] = std::uint8_t{1};
      roadCount++;
    }
  }

  // Neighbor data (max degree 4).
  std::vector<std::array<int, 4>> nbrIdx(n);
  std::vector<std::array<float, 4>> nbrW(n);
  std::vector<std::uint8_t> nbrDeg(n, std::uint8_t{0});

  constexpr int kDirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (idx >= traversableRoad.size() || traversableRoad[idx] == 0) continue;

      float sumW = 0.0f;
      std::uint8_t deg = 0;

      for (int d = 0; d < 4; ++d) {
        const int nx = x + kDirs[d][0];
        const int ny = y + kDirs[d][1];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const std::size_t nidx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx);
        if (nidx >= traversableRoad.size() || traversableRoad[nidx] == 0) continue;

        // Edge cost = travel-time to enter the neighbor tile.
        const Tile& nt = world.at(nx, ny);
        int edgeMilli = (nt.terrain == Terrain::Water)
                            ? RoadBridgeTravelTimeMilliForLevel(static_cast<int>(nt.level))
                            : RoadTravelTimeMilliForLevel(static_cast<int>(nt.level));
        if (extraCostPtr && nidx < extraCostPtr->size()) edgeMilli += (*extraCostPtr)[nidx];

        const float edgeSteps = static_cast<float>(edgeMilli) / 1000.0f;
        const float wgt = std::exp(-cfg.edgeImpedanceBeta * edgeSteps);

        if (deg < 4) {
          nbrIdx[idx][deg] = static_cast<int>(nidx);
          nbrW[idx][deg] = wgt;
          deg++;
          sumW += wgt;
        }
      }

      nbrDeg[idx] = deg;
      if (sumW > 0.0f) {
        for (std::uint8_t k = 0; k < deg; ++k) {
          nbrW[idx][k] = nbrW[idx][k] / sumW;
        }
      }
    }
  }

  // Initialize road opportunity with job source strength.
  std::vector<float> roadVal(n, 0.0f);
  std::vector<float> roadNext(n, 0.0f);

  for (std::size_t i = 0; i < n; ++i) {
    if (traversableRoad[i] == 0) continue;
    roadVal[i] = static_cast<float>(std::max(0, jobsOnRoad[i]));
  }

  const int iters = std::max(0, cfg.diffusionIterations);
  const float decay = std::clamp(cfg.diffusionDecay, 0.0f, 1.0f);

  for (int iter = 0; iter < iters; ++iter) {
    for (std::size_t i = 0; i < n; ++i) {
      if (traversableRoad[i] == 0) {
        roadNext[i] = 0.0f;
        continue;
      }

      float neigh = 0.0f;
      const std::uint8_t deg = nbrDeg[i];
      for (std::uint8_t k = 0; k < deg; ++k) {
        const int j = nbrIdx[i][k];
        if (j < 0 || static_cast<std::size_t>(j) >= n) continue;
        neigh += nbrW[i][k] * roadVal[static_cast<std::size_t>(j)];
      }

      const float src = static_cast<float>(std::max(0, jobsOnRoad[i]));
      roadNext[i] = src + decay * neigh;
    }

    roadVal.swap(roadNext);
  }

  // Copy road-only field for debugging.
  for (std::size_t i = 0; i < n; ++i) {
    if (traversableRoad[i] != 0) out.roadOpportunityRaw[i] = roadVal[i];
  }

  // Map road opportunity to all tiles (via zone access / adjacent road).
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);

      float v = 0.0f;
      if (t.overlay == Overlay::Road) {
        if (idx < roadVal.size()) v = roadVal[idx];
      } else {
        const int ridx = (idx < zoneAccess->roadIdx.size()) ? zoneAccess->roadIdx[idx] : -1;
        if (ridx >= 0 && static_cast<std::size_t>(ridx) < roadVal.size()) {
          v = roadVal[static_cast<std::size_t>(ridx)];
        } else {
          // Fallback: pick the adjacent road with the best nearest-job access cost.
          const AdjRoadPick pick = PickBestAdjacentRoad(world, x, y, field);
          if (pick.roadIdx >= 0 && static_cast<std::size_t>(pick.roadIdx) < roadVal.size()) {
            v = roadVal[static_cast<std::size_t>(pick.roadIdx)];
          }
        }
      }

      out.jobOpportunityRaw[idx] = std::max(0.0f, v);
    }
  }

  // Normalize opportunity with log compression + robust percentile.
  std::vector<float> samples;
  samples.reserve(n / 2);
  for (float v : out.jobOpportunityRaw) {
    if (v > 0.0f) samples.push_back(v);
  }

  const float denom = std::max(1.0f, Percentile(samples, cfg.opportunityPercentile));
  const float logDen = std::max(1e-6f, std::log1p(denom));

  for (std::size_t i = 0; i < n; ++i) {
    const float v = out.jobOpportunityRaw[i];
    const float s = std::log1p(std::max(0.0f, v)) / logDen;
    out.jobOpportunity01[i] = Clamp01(s);
  }

  // ---- Residential-weighted summary ----
  std::int64_t pop = 0;
  std::int64_t popUnreach = 0;

  double sumAccess01 = 0.0;
  double sumOpp01 = 0.0;
  double sumCostSteps = 0.0;
  std::int64_t popReach = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      const int occ = static_cast<int>(t.occupants);
      if (occ <= 0) continue;

      out.residentTileCount++;
      pop += occ;

      sumAccess01 += static_cast<double>(occ) * static_cast<double>(out.jobAccess01[idx]);
      sumOpp01 += static_cast<double>(occ) * static_cast<double>(out.jobOpportunity01[idx]);

      const int c = out.jobAccessCostMilli[idx];
      if (c >= 0) {
        popReach += occ;
        sumCostSteps += static_cast<double>(occ) * (static_cast<double>(c) / 1000.0);
      } else {
        popUnreach += occ;
      }
    }
  }

  out.residentPopulation = static_cast<int>(std::clamp<std::int64_t>(pop, 0, std::numeric_limits<int>::max()));
  out.residentUnreachablePopulation =
    static_cast<int>(std::clamp<std::int64_t>(popUnreach, 0, std::numeric_limits<int>::max()));

  if (pop > 0) {
    out.residentMeanAccess01 = static_cast<float>(sumAccess01 / static_cast<double>(pop));
    out.residentMeanOpportunity01 = static_cast<float>(sumOpp01 / static_cast<double>(pop));
  }

  if (popReach > 0) {
    out.residentMeanAccessCostSteps = static_cast<float>(sumCostSteps / static_cast<double>(popReach));
  }

  return out;
}

} // namespace isocity
