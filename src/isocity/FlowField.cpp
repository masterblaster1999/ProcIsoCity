#include "isocity/FlowField.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace isocity {

namespace {

bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t expect = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == expect;
}

struct HeapNode {
  int cost = 0;
  int steps = 0;
  int owner = 0;
  int idx = 0;
};

struct HeapCmp {
  bool operator()(const HeapNode& a, const HeapNode& b) const
  {
    // priority_queue is max-heap by default; return true if a should come after b.
    if (a.cost != b.cost) return a.cost > b.cost;
    if (a.steps != b.steps) return a.steps > b.steps;
    if (a.owner != b.owner) return a.owner > b.owner;
    return a.idx > b.idx;
  }
};

} // namespace

RoadFlowField BuildRoadFlowField(const World& world, const std::vector<int>& sourceRoadIdx
                                 , const RoadFlowFieldConfig& cfg
                                 , const std::vector<std::uint8_t>* precomputedRoadToEdge
                                 , const std::vector<int>* extraCostMilli
                                 , const std::vector<std::uint8_t>* roadBlockMask
                                 , const std::vector<int>* sourceInitialCostMilli)
{
  RoadFlowField out;

  const int w = world.width();
  const int h = world.height();
  out.w = w;
  out.h = h;

  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.dist.assign(n, -1);
  out.cost.assign(n, -1);
  out.parent.assign(n, -1);
  if (cfg.computeOwner) {
    out.owner.assign(n, -1);
  } else {
    out.owner.clear();
  }

  // Outside connection mask.
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;

  if (cfg.requireOutsideConnection) {
    if (MaskUsable(precomputedRoadToEdge, w, h)) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      ComputeRoadsConnectedToEdge(world, roadToEdgeLocal);
      roadToEdge = &roadToEdgeLocal;
    }
  }

  const bool blockOk = MaskUsable(roadBlockMask, w, h);

  auto isTraversableRoad = [&](int ridx) -> bool {
    if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) return false;
    const int x = ridx % w;
    const int y = ridx / w;
    if (!world.inBounds(x, y)) return false;
    if (world.at(x, y).overlay != Overlay::Road) return false;
    if (blockOk) {
      const std::size_t ui = static_cast<std::size_t>(ridx);
      if (ui < roadBlockMask->size() && (*roadBlockMask)[ui] != 0) return false;
    }
    if (cfg.requireOutsideConnection) {
      const std::size_t ui = static_cast<std::size_t>(ridx);
      if (!roadToEdge || ui >= roadToEdge->size() || (*roadToEdge)[ui] == 0) return false;
    }
    return true;
  };

  if (sourceRoadIdx.empty()) return out;

  const std::vector<int>* extraCost = nullptr;
  if (extraCostMilli && extraCostMilli->size() == n) {
    extraCost = extraCostMilli;
  }

  const std::vector<int>* sourceInit = nullptr;
  if (sourceInitialCostMilli && sourceInitialCostMilli->size() == sourceRoadIdx.size()) {
    sourceInit = sourceInitialCostMilli;
  }

  // Deterministic neighbor order.
  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

  if (!cfg.useTravelTime) {
    // --- Unweighted multi-source BFS (steps) ---
    std::vector<int> queue;
    queue.reserve(sourceRoadIdx.size());

    for (int si = 0; si < static_cast<int>(sourceRoadIdx.size()); ++si) {
      const int sidx = sourceRoadIdx[static_cast<std::size_t>(si)];
      if (!isTraversableRoad(sidx)) continue;
      const std::size_t ui = static_cast<std::size_t>(sidx);
      if (out.dist[ui] == 0) continue; // already seeded

      const int initCost = sourceInit ? std::max(0, (*sourceInit)[static_cast<std::size_t>(si)]) : 0;

      out.dist[ui] = 0;
      out.cost[ui] = initCost;
      out.parent[ui] = -1;
      if (cfg.computeOwner) out.owner[ui] = si;
      queue.push_back(sidx);
    }

    std::size_t head = 0;
    while (head < queue.size()) {
      const int uidx = queue[head++];
      const std::size_t uu = static_cast<std::size_t>(uidx);
      const int ux = uidx % w;
      const int uy = uidx / w;
      const int dcur = out.dist[uu];
      const int ccur = out.cost[uu];

      for (const auto& d : dirs) {
        const int nx = ux + d[0];
        const int ny = uy + d[1];
        if (!world.inBounds(nx, ny)) continue;
        const int nidx = ny * w + nx;
        if (!isTraversableRoad(nidx)) continue;
        const std::size_t nu = static_cast<std::size_t>(nidx);
        if (out.dist[nu] != -1) continue;

        out.dist[nu] = dcur + 1;
        const Tile& nt = world.at(nx, ny);
        const int lvl = static_cast<int>(nt.level);
        int moveCost = (nt.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(lvl)
                                                  : RoadTravelTimeMilliForLevel(lvl);
        if (extraCost) {
          // Extra per-tile penalty (e.g., congestion). Treated as cost to ENTER the tile.
          moveCost += std::max(0, (*extraCost)[nu]);
        }
        out.cost[nu] = ccur + moveCost;
        out.parent[nu] = uidx;
        if (cfg.computeOwner) out.owner[nu] = out.owner[uu];
        queue.push_back(nidx);
      }
    }

    return out;
  }

  // --- Weighted multi-source Dijkstra (travel time) ---
  constexpr int INF = std::numeric_limits<int>::max() / 4;
  std::vector<int> bestCost(n, INF);
  std::vector<int> bestSteps(n, INF);

  std::priority_queue<HeapNode, std::vector<HeapNode>, HeapCmp> heap;

  for (int si = 0; si < static_cast<int>(sourceRoadIdx.size()); ++si) {
    const int sidx = sourceRoadIdx[static_cast<std::size_t>(si)];
    if (!isTraversableRoad(sidx)) continue;
    const std::size_t ui = static_cast<std::size_t>(sidx);

    const int initCost = sourceInit ? std::max(0, (*sourceInit)[static_cast<std::size_t>(si)]) : 0;

    bool improve = false;
    if (initCost < bestCost[ui]) {
      improve = true;
    } else if (initCost == bestCost[ui]) {
      if (cfg.computeOwner) {
        const int oldOwner = out.owner[ui];
        if (oldOwner < 0 || si < oldOwner) {
          improve = true;
        }
      }
    }

    if (!improve) continue;

    bestCost[ui] = initCost;
    bestSteps[ui] = 0;
    out.parent[ui] = -1;
    if (cfg.computeOwner) out.owner[ui] = si;

    heap.push(HeapNode{initCost, 0, cfg.computeOwner ? si : 0, sidx});
  }

  while (!heap.empty()) {
    const HeapNode cur = heap.top();
    heap.pop();

    const std::size_t uu = static_cast<std::size_t>(cur.idx);
    if (uu >= n) continue;
    if (cur.cost != bestCost[uu] || cur.steps != bestSteps[uu]) continue;

    const int ux = cur.idx % w;
    const int uy = cur.idx / w;

    for (const auto& d : dirs) {
      const int nx = ux + d[0];
      const int ny = uy + d[1];
      if (!world.inBounds(nx, ny)) continue;
      const int nidx = ny * w + nx;
      if (!isTraversableRoad(nidx)) continue;

      const std::size_t nu = static_cast<std::size_t>(nidx);
      const Tile& nt = world.at(nx, ny);
      const int lvl = static_cast<int>(nt.level);
      int moveCost = (nt.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(lvl)
                                                : RoadTravelTimeMilliForLevel(lvl);
      if (extraCost) {
        // Extra per-tile penalty (e.g., congestion). Treated as cost to ENTER the tile.
        moveCost += std::max(0, (*extraCost)[nu]);
      }
      const int nc = cur.cost + moveCost;
      const int ns = cur.steps + 1;
      const int nOwner = cfg.computeOwner ? out.owner[uu] : 0;

      bool improve = false;
      if (nc < bestCost[nu]) {
        improve = true;
      } else if (nc == bestCost[nu] && ns < bestSteps[nu]) {
        improve = true;
      } else if (nc == bestCost[nu] && ns == bestSteps[nu]) {
        if (cfg.computeOwner) {
          const int oldOwner = out.owner[nu];
          if (oldOwner < 0 || nOwner < oldOwner) {
            improve = true;
          }
        }
        if (!improve) {
          const int oldParent = out.parent[nu];
          if (oldParent < 0 || cur.idx < oldParent) {
            improve = true;
          }
        }
      }

      if (!improve) continue;

      bestCost[nu] = nc;
      bestSteps[nu] = ns;
      out.parent[nu] = cur.idx;
      if (cfg.computeOwner) out.owner[nu] = nOwner;

      heap.push(HeapNode{nc, ns, nOwner, nidx});
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    if (bestCost[i] == INF) continue;
    out.dist[i] = bestSteps[i];
    out.cost[i] = bestCost[i];
  }

  return out;
}

} // namespace isocity
