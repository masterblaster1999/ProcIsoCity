#include "isocity/Evacuation.hpp"

#include "isocity/FlowField.hpp"
#include "isocity/Road.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace isocity {

namespace {

inline bool MaskUsable(const std::vector<std::uint8_t>* mask, std::size_t n)
{
  return mask && mask->size() == n;
}

struct ResHeapNode {
  int cost = 0;    // milli-steps
  int steps = 0;   // interior zone steps
  int road = -1;   // chosen access road tile
  int idx = 0;     // zone tile index
  int prev = -1;   // previous zone tile index (tie-break)
};

struct ResHeapCmp {
  bool operator()(const ResHeapNode& a, const ResHeapNode& b) const
  {
    // priority_queue is max-heap; invert comparisons for a min-heap.
    if (a.cost != b.cost) return a.cost > b.cost;
    if (a.steps != b.steps) return a.steps > b.steps;
    if (a.road != b.road) return a.road > b.road;
    if (a.prev != b.prev) return a.prev > b.prev;
    return a.idx > b.idx;
  }
};

inline bool BetterLabel(int costA, int stepsA, int roadA, int prevA, int idxA,
                        int costB, int stepsB, int roadB, int prevB, int idxB)
{
  if (costA != costB) return costA < costB;
  if (stepsA != stepsB) return stepsA < stepsB;
  if (roadA != roadB) return roadA < roadB;
  if (prevA != prevB) {
    // Prefer a defined prev, and then the smaller one.
    if (prevB < 0) return prevA >= 0;
    if (prevA < 0) return false;
    return prevA < prevB;
  }
  return idxA < idxB;
}

static void ComputeWeightedStats(const std::vector<std::pair<int, int>>& costWeight,
                                 float& outAvgSteps, float& outP95Steps)
{
  outAvgSteps = 0.0f;
  outP95Steps = 0.0f;
  if (costWeight.empty()) return;

  std::uint64_t totalW = 0;
  std::uint64_t sumCost = 0;
  for (const auto& cw : costWeight) {
    const int c = cw.first;
    const int w = cw.second;
    if (c < 0 || w <= 0) continue;
    totalW += static_cast<std::uint64_t>(w);
    sumCost += static_cast<std::uint64_t>(c) * static_cast<std::uint64_t>(w);
  }
  if (totalW == 0) return;

  outAvgSteps = static_cast<float>(static_cast<double>(sumCost) / static_cast<double>(totalW) / 1000.0);

  std::vector<std::pair<int, int>> sorted = costWeight;
  std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
    if (a.first != b.first) return a.first < b.first;
    return a.second < b.second;
  });

  // Weighted 95th percentile (smallest cost where cumulative weight >= 95%).
  const std::uint64_t target = (totalW * 95u + 99u) / 100u; // ceil(0.95*totalW)
  std::uint64_t acc = 0;
  int p95Cost = sorted.back().first;
  for (const auto& cw : sorted) {
    const int c = cw.first;
    const int w = cw.second;
    if (c < 0 || w <= 0) continue;
    acc += static_cast<std::uint64_t>(w);
    if (acc >= target) {
      p95Cost = c;
      break;
    }
  }
  outP95Steps = static_cast<float>(static_cast<double>(p95Cost) / 1000.0);
}

} // namespace

EvacuationResult ComputeEvacuationToEdge(const World& world, const EvacuationConfig& cfg,
                                        const std::vector<std::uint8_t>* blockedMask)
{
  EvacuationResult out;

  const int w = world.width();
  const int h = world.height();
  out.w = w;
  out.h = h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const bool blockedOk = MaskUsable(blockedMask, n);

  auto flatIdx = [&](int x, int y) -> int { return y * w + x; };
  auto inBounds = [&](int x, int y) -> bool { return x >= 0 && y >= 0 && x < w && y < h; };

  auto isBlocked = [&](int idx) -> bool {
    if (!blockedOk) return false;
    const std::size_t ui = static_cast<std::size_t>(idx);
    return ui < blockedMask->size() && (*blockedMask)[ui] != 0;
  };

  // --- Identify exit road tiles (edge roads) ---
  std::vector<int> exitRoad;
  exitRoad.reserve(static_cast<std::size_t>(w + h) * 2u);
  std::vector<std::uint8_t> seenExit(n, 0);

  auto considerExit = [&](int x, int y) {
    if (!inBounds(x, y)) return;
    const int idx = flatIdx(x, y);
    const std::size_t ui = static_cast<std::size_t>(idx);
    if (ui >= seenExit.size()) return;
    if (seenExit[ui]) return;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) return;
    if (isBlocked(idx)) return;
    seenExit[ui] = 1;
    exitRoad.push_back(idx);
  };

  for (int x = 0; x < w; ++x) {
    considerExit(x, 0);
    considerExit(x, h - 1);
  }
  for (int y = 0; y < h; ++y) {
    considerExit(0, y);
    considerExit(w - 1, y);
  }
  out.exitSources = static_cast<int>(exitRoad.size());

  // --- Build road flow field to nearest exit ---
  RoadFlowFieldConfig fcfg;
  fcfg.requireOutsideConnection = false;
  fcfg.computeOwner = false;
  fcfg.useTravelTime = cfg.useTravelTime;

  RoadFlowField roadFF = BuildRoadFlowField(world, exitRoad, fcfg, nullptr, nullptr, blockedMask);
  out.roadDistSteps = roadFF.dist;
  out.roadCostMilli = roadFF.cost;
  out.roadParent = roadFF.parent;

  out.resCostMilli.assign(n, -1);
  out.resAccessRoad.assign(n, -1);
  out.evacRoadFlow.assign(n, 0);

  // --- Residential component Dijkstra (walk inside zones + choose best adjacent reachable road) ---
  std::vector<std::uint8_t> visitedRes(n, 0);
  std::vector<int> stamp(n, 0);
  int stampId = 1;

  constexpr int kDirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

  auto isPassableResidential = [&](int idx) -> bool {
    if (idx < 0 || static_cast<std::size_t>(idx) >= n) return false;
    if (isBlocked(idx)) return false;
    const int x = idx % w;
    const int y = idx / w;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Residential) return false;
    if (t.terrain == Terrain::Water) return false;
    return true;
  };

  // Work arrays (only valid for indices currently in the component).
  constexpr int INF = std::numeric_limits<int>::max() / 4;
  std::vector<int> bestCost(n, INF);
  std::vector<int> bestSteps(n, INF);
  std::vector<int> bestRoad(n, -1);
  std::vector<int> bestPrev(n, -1);

  std::vector<int> q;
  q.reserve(n / 8);
  std::vector<int> comp;
  comp.reserve(n / 8);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int start = flatIdx(x, y);
      const std::size_t us = static_cast<std::size_t>(start);
      if (us >= visitedRes.size()) continue;
      if (visitedRes[us]) continue;
      if (!isPassableResidential(start)) continue;

      // --- Gather a connected component of passable Residential tiles (4-neighborhood) ---
      visitedRes[us] = 1;
      q.clear();
      comp.clear();
      q.push_back(start);

      while (!q.empty()) {
        const int cur = q.back();
        q.pop_back();
        comp.push_back(cur);
        stamp[static_cast<std::size_t>(cur)] = stampId;

        const int cx = cur % w;
        const int cy = cur / w;
        for (const auto& d : kDirs) {
          const int nx = cx + d[0];
          const int ny = cy + d[1];
          if (!inBounds(nx, ny)) continue;
          const int ni = flatIdx(nx, ny);
          const std::size_t ui = static_cast<std::size_t>(ni);
          if (ui >= visitedRes.size()) continue;
          if (visitedRes[ui]) continue;
          if (!isPassableResidential(ni)) continue;
          visitedRes[ui] = 1;
          q.push_back(ni);
        }
      }

      // Reset work arrays for this component.
      for (int idx : comp) {
        const std::size_t ui = static_cast<std::size_t>(idx);
        bestCost[ui] = INF;
        bestSteps[ui] = INF;
        bestRoad[ui] = -1;
        bestPrev[ui] = -1;
      }

      // Identify boundary sources (res tiles adjacent to a *reachable* and *unblocked* road).
      std::priority_queue<ResHeapNode, std::vector<ResHeapNode>, ResHeapCmp> heap;

      for (int zi : comp) {
        const int zx = zi % w;
        const int zy = zi / w;

        int bestR = -1;
        int bestRCost = INF;
        int bestRSteps = INF;

        for (const auto& d : kDirs) {
          const int rx = zx + d[0];
          const int ry = zy + d[1];
          if (!inBounds(rx, ry)) continue;
          const int ridx = flatIdx(rx, ry);
          const Tile& rt = world.at(rx, ry);
          if (rt.overlay != Overlay::Road) continue;
          if (isBlocked(ridx)) continue;

          const std::size_t ur = static_cast<std::size_t>(ridx);
          if (ur >= roadFF.cost.size()) continue;
          const int rc = roadFF.cost[ur];
          const int rs = roadFF.dist[ur];
          if (rc < 0 || rs < 0) continue; // unreachable to an exit

          if (bestR < 0 || rc < bestRCost || (rc == bestRCost && rs < bestRSteps) ||
              (rc == bestRCost && rs == bestRSteps && ridx < bestR)) {
            bestR = ridx;
            bestRCost = rc;
            bestRSteps = rs;
          }
        }

        if (bestR >= 0) {
          const int initCost = bestRCost + std::max(0, cfg.walkCostMilli);
          const std::size_t uz = static_cast<std::size_t>(zi);

          // Multi-source: keep the best initial label for this tile.
          if (BetterLabel(initCost, 0, bestR, -1, zi, bestCost[uz], bestSteps[uz], bestRoad[uz], bestPrev[uz], zi)) {
            bestCost[uz] = initCost;
            bestSteps[uz] = 0;
            bestRoad[uz] = bestR;
            bestPrev[uz] = -1;
            heap.push(ResHeapNode{initCost, 0, bestR, zi, -1});
          }
        }
      }

      if (heap.empty()) {
        // No reachable road-adjacent tiles in this residential block.
        // Leave out.resCostMilli as -1 for these tiles.
        ++stampId;
        continue;
      }

      // Dijkstra within the zone component.
      while (!heap.empty()) {
        const ResHeapNode cur = heap.top();
        heap.pop();

        const std::size_t ui = static_cast<std::size_t>(cur.idx);
        if (ui >= bestCost.size()) continue;
        if (cur.cost != bestCost[ui] || cur.steps != bestSteps[ui] || cur.road != bestRoad[ui] || cur.prev != bestPrev[ui]) {
          continue;
        }

        const int cx = cur.idx % w;
        const int cy = cur.idx / w;

        for (const auto& d : kDirs) {
          const int nx = cx + d[0];
          const int ny = cy + d[1];
          if (!inBounds(nx, ny)) continue;
          const int ni = flatIdx(nx, ny);
          const std::size_t un = static_cast<std::size_t>(ni);
          if (un >= stamp.size()) continue;
          if (stamp[un] != stampId) continue;

          const int nCost = cur.cost + std::max(0, cfg.walkCostMilli);
          const int nSteps = cur.steps + 1;
          const int nRoad = cur.road;
          const int nPrev = cur.idx;

          if (BetterLabel(nCost, nSteps, nRoad, nPrev, ni, bestCost[un], bestSteps[un], bestRoad[un], bestPrev[un], ni)) {
            bestCost[un] = nCost;
            bestSteps[un] = nSteps;
            bestRoad[un] = nRoad;
            bestPrev[un] = nPrev;
            heap.push(ResHeapNode{nCost, nSteps, nRoad, ni, nPrev});
          }
        }
      }

      // Write component results.
      for (int idx : comp) {
        const std::size_t ui = static_cast<std::size_t>(idx);
        if (bestCost[ui] != INF && bestRoad[ui] >= 0) {
          out.resCostMilli[ui] = bestCost[ui];
          out.resAccessRoad[ui] = bestRoad[ui];
        }
      }

      ++stampId;
    }
  }

  // --- Summaries + road flow aggregation ---
  std::vector<std::pair<int, int>> costWeight;
  costWeight.reserve(n / 8);

  for (int yy = 0; yy < h; ++yy) {
    for (int xx = 0; xx < w; ++xx) {
      const int idx = flatIdx(xx, yy);
      const std::size_t ui = static_cast<std::size_t>(idx);
      const Tile& t = world.at(xx, yy);
      if (t.overlay != Overlay::Residential) continue;
      if (t.terrain == Terrain::Water) continue;

      out.residentialTiles++;
      out.population += static_cast<int>(t.occupants);

      if (isBlocked(idx)) {
        out.floodedResidentialTiles++;
        out.floodedPopulation += static_cast<int>(t.occupants);
        continue;
      }

      const int c = (ui < out.resCostMilli.size()) ? out.resCostMilli[ui] : -1;
      if (c >= 0) {
        out.reachableResidentialTiles++;
        out.reachablePopulation += static_cast<int>(t.occupants);
        // Weight by occupants when possible; fall back to 1 later if everything is zero.
        if (t.occupants > 0) {
          costWeight.emplace_back(c, t.occupants);
        }
      } else {
        out.unreachableResidentialTiles++;
        out.unreachablePopulation += static_cast<int>(t.occupants);
      }
    }
  }

  // If the world isn't populated (all occupants==0), use tile-weighting for time stats.
  if (costWeight.empty()) {
    for (std::size_t i = 0; i < out.resCostMilli.size(); ++i) {
      if (out.resCostMilli[i] >= 0) {
        costWeight.emplace_back(out.resCostMilli[i], 1);
      }
    }
  }

  ComputeWeightedStats(costWeight, out.avgEvacTime, out.p95EvacTime);

  // Aggregate evacuation demand onto roads.
  for (int yy = 0; yy < h; ++yy) {
    for (int xx = 0; xx < w; ++xx) {
      const int idx = flatIdx(xx, yy);
      const std::size_t ui = static_cast<std::size_t>(idx);
      const Tile& t = world.at(xx, yy);
      if (t.overlay != Overlay::Residential) continue;
      if (t.terrain == Terrain::Water) continue;
      if (isBlocked(idx)) continue;
      if (t.occupants <= 0) continue;
      if (ui >= out.resAccessRoad.size()) continue;
      const int startRoad = out.resAccessRoad[ui];
      if (startRoad < 0) continue;
      if (static_cast<std::size_t>(startRoad) >= out.roadParent.size()) continue;

      int ridx = startRoad;
      int safety = 0;
      while (ridx >= 0 && static_cast<std::size_t>(ridx) < n && safety < static_cast<int>(n)) {
        out.evacRoadFlow[static_cast<std::size_t>(ridx)] += static_cast<std::uint32_t>(t.occupants);
        ridx = out.roadParent[static_cast<std::size_t>(ridx)];
        ++safety;
      }
    }
  }

  // Congestion summary.
  std::uint64_t totalFlow = 0;
  std::uint64_t totalExcess = 0;
  out.maxEvacRoadFlow = 0;
  out.congestedRoadTiles = 0;

  for (int yy = 0; yy < h; ++yy) {
    for (int xx = 0; xx < w; ++xx) {
      const int idx = flatIdx(xx, yy);
      const std::size_t ui = static_cast<std::size_t>(idx);
      const Tile& t = world.at(xx, yy);
      if (t.overlay != Overlay::Road) continue;
      const std::uint32_t flow = out.evacRoadFlow[ui];
      if (flow == 0) continue;

      out.maxEvacRoadFlow = std::max(out.maxEvacRoadFlow, flow);
      totalFlow += static_cast<std::uint64_t>(flow);

      const int cap = RoadCapacityForLevel(std::max(0, cfg.roadTileCapacity), t.level);
      if (static_cast<std::uint64_t>(flow) > static_cast<std::uint64_t>(cap)) {
        out.congestedRoadTiles++;
        totalExcess += static_cast<std::uint64_t>(flow - static_cast<std::uint32_t>(std::max(0, cap)));
      }
    }
  }

  if (totalFlow > 0) {
    const double ratio = static_cast<double>(totalExcess) / static_cast<double>(totalFlow);
    out.congestion = static_cast<float>(std::clamp(ratio, 0.0, 1.0));
  } else {
    out.congestion = 0.0f;
  }

  out.congestionFrac = static_cast<float>(out.congestion);
  return out;
}

} // namespace isocity
