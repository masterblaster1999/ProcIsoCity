#include "isocity/Goods.hpp"

#include "isocity/FlowField.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace isocity {

namespace {

inline std::uint16_t SatAddU16(std::uint16_t cur, std::uint32_t add)
{
  std::uint32_t v = static_cast<std::uint32_t>(cur) + add;
  if (v > 65535u) v = 65535u;
  return static_cast<std::uint16_t>(v);
}

bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t expect = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == expect;
}

inline int BaseIndustrialSupply(int level)
{
  // Mirror the industrial job capacity from the core sim.
  return 12 * std::clamp(level, 0, 3);
}

inline int BaseCommercialDemand(int level)
{
  // Mirror the commercial job capacity from the core sim.
  return 8 * std::clamp(level, 0, 3);
}

struct Source {
  int roadIdx = -1;
  int supply = 0;
  int remaining = 0;
};

struct Consumer {
  int x = 0;
  int y = 0;
  int roadIdx = -1;
  int demand = 0;
  int dist = -1; // steps along the chosen producer path
  int cost = -1; // travel-time cost (milli-steps)
  int owner = -1;
};

struct SearchNode {
  int cost = 0;  // travel-time cost
  int steps = 0; // road steps
  int idx = 0;   // road tile index
};

struct SearchCmp {
  bool operator()(const SearchNode& a, const SearchNode& b) const
  {
    // priority_queue is max-heap by default; return true if a should come after b.
    if (a.cost != b.cost) return a.cost > b.cost;
    if (a.steps != b.steps) return a.steps > b.steps;
    return a.idx > b.idx;
  }
};

} // namespace

GoodsResult ComputeGoodsFlow(const World& world, const GoodsConfig& cfg,
                            const std::vector<std::uint8_t>* precomputedRoadToEdge,
                            const ZoneAccessMap* precomputedZoneAccess)
{
  GoodsResult out;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.roadGoodsTraffic.assign(n, 0);
  out.commercialFill.assign(n, 255);

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

  // Zone access: allows interior tiles of a connected zoned area to be reachable via a
  // road-adjacent boundary tile.
  ZoneAccessMap zoneAccessLocal;
  const ZoneAccessMap* zoneAccess = nullptr;

  if (precomputedZoneAccess && precomputedZoneAccess->w == w && precomputedZoneAccess->h == h &&
      precomputedZoneAccess->roadIdx.size() == n) {
    zoneAccess = precomputedZoneAccess;
  } else {
    zoneAccessLocal = BuildZoneAccessMap(world, roadToEdge);
    zoneAccess = &zoneAccessLocal;
  }

  auto isTraversableRoad = [&](int ridx) -> bool {
    if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) return false;
    const int x = ridx % w;
    const int y = ridx / w;
    if (!world.inBounds(x, y)) return false;
    if (world.at(x, y).overlay != Overlay::Road) return false;
    if (cfg.requireOutsideConnection) {
      const std::size_t ui = static_cast<std::size_t>(ridx);
      if (!roadToEdge || ui >= roadToEdge->size() || (*roadToEdge)[ui] == 0) return false;
    }
    return true;
  };

  // --- Gather industrial supply per road tile (merge multiple producers on the same road access point) ---
  std::vector<int> supplyPerRoad(n, 0);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Industrial) continue;
      if (t.level == 0) continue;

      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (zidx >= zoneAccess->roadIdx.size()) continue;
      if (zoneAccess->roadIdx[zidx] < 0) continue;

      // Prefer a directly-adjacent road if available; otherwise use the propagated access road.
      int ridx = -1;
      Point road{};
      if (PickAdjacentRoadTile(world, roadToEdge, x, y, road)) {
        ridx = road.y * w + road.x;
      } else {
        ridx = zoneAccess->roadIdx[zidx];
      }
      if (!isTraversableRoad(ridx)) continue;

      const float raw = static_cast<float>(BaseIndustrialSupply(static_cast<int>(t.level))) * cfg.supplyScale;
      const int supply = std::max(0, static_cast<int>(std::lround(raw)));
      if (supply <= 0) continue;

      supplyPerRoad[static_cast<std::size_t>(ridx)] += supply;
      out.goodsProduced += supply;
    }
  }

  std::vector<Source> sources;
  sources.reserve(n / 64);

  for (int ridx = 0; ridx < static_cast<int>(n); ++ridx) {
    const int supply = supplyPerRoad[static_cast<std::size_t>(ridx)];
    if (supply <= 0) continue;
    if (!isTraversableRoad(ridx)) continue;
    sources.push_back(Source{ridx, supply, supply});
  }

  // --- Multi-source search from industrial sources (nearest-producer labeling, travel-time weighted) ---
  std::vector<int> sourceRoadIdx;
  sourceRoadIdx.reserve(sources.size());
  for (const Source& s : sources) {
    sourceRoadIdx.push_back(s.roadIdx);
  }

  RoadFlowFieldConfig prodCfg;
  prodCfg.requireOutsideConnection = cfg.requireOutsideConnection;
  prodCfg.computeOwner = true;
  prodCfg.useTravelTime = true;

  const RoadFlowField prodField = BuildRoadFlowField(world, sourceRoadIdx, prodCfg, roadToEdge);

  // --- Search from map-edge roads (for imports/exports routing) ---
  std::vector<int> edgeSources;
  edgeSources.reserve(static_cast<std::size_t>(w + h) * 2u);

  auto pushEdge = [&](int ex, int ey) {
    const int ridx = ey * w + ex;
    if (!isTraversableRoad(ridx)) return;
    edgeSources.push_back(ridx);
  };

  // Seed with border roads (deterministic order).
  for (int x = 0; x < w; ++x) {
    pushEdge(x, 0);
    if (h > 1) pushEdge(x, h - 1);
  }
  for (int y = 1; y < h - 1; ++y) {
    pushEdge(0, y);
    if (w > 1) pushEdge(w - 1, y);
  }

  RoadFlowField edgeField;
  if (cfg.allowImports || cfg.allowExports) {
    RoadFlowFieldConfig edgeCfg;
    edgeCfg.requireOutsideConnection = cfg.requireOutsideConnection;
    edgeCfg.computeOwner = false;
    edgeCfg.useTravelTime = true;
    edgeField = BuildRoadFlowField(world, edgeSources, edgeCfg, roadToEdge);
  }

  auto addAlongParentChain = [&](int startIdx, const std::vector<int>& par, int amount) {
    if (amount <= 0) return;
    int cur = startIdx;
    int guard = 0;
    while (cur != -1 && guard++ < static_cast<int>(n) + 8) {
      const std::size_t ui = static_cast<std::size_t>(cur);
      out.roadGoodsTraffic[ui] = SatAddU16(out.roadGoodsTraffic[ui], static_cast<std::uint32_t>(amount));
      cur = par[ui];
    }
  };

  // --- Collect consumers (commercial demand) ---
  std::vector<Consumer> consumers;
  consumers.reserve(n / 64);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Commercial) continue;
      if (t.level == 0) continue;
      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (zidx >= zoneAccess->roadIdx.size()) continue;
      if (zoneAccess->roadIdx[zidx] < 0) continue;

      const float raw = static_cast<float>(BaseCommercialDemand(static_cast<int>(t.level))) * cfg.demandScale;
      const int demand = std::max(0, static_cast<int>(std::lround(raw)));
      if (demand <= 0) continue;

      int ridx = -1;
      Point road{};
      if (PickAdjacentRoadTile(world, roadToEdge, x, y, road)) {
        ridx = road.y * w + road.x;
      } else {
        ridx = zoneAccess->roadIdx[zidx];
      }
      if (!isTraversableRoad(ridx)) continue;

      const std::size_t ur = static_cast<std::size_t>(ridx);
      const int d = (!sources.empty() && ur < prodField.dist.size()) ? prodField.dist[ur] : -1;
      const int c = (!sources.empty() && ur < prodField.cost.size()) ? prodField.cost[ur] : -1;
      const int own = ((d >= 0 && c >= 0) && ur < prodField.owner.size()) ? prodField.owner[ur] : -1;

      consumers.push_back(Consumer{x, y, ridx, demand, d, c, own});
      out.goodsDemand += demand;
    }
  }

  // Prioritize closer consumers first so scarce supply serves the nearest commercial areas.
  std::sort(consumers.begin(), consumers.end(), [&](const Consumer& a, const Consumer& b) {
    const int ca = (a.cost >= 0) ? a.cost : std::numeric_limits<int>::max();
    const int cb = (b.cost >= 0) ? b.cost : std::numeric_limits<int>::max();
    if (ca != cb) return ca < cb;

    const int da = (a.dist >= 0) ? a.dist : std::numeric_limits<int>::max();
    const int db = (b.dist >= 0) ? b.dist : std::numeric_limits<int>::max();
    if (da != db) return da < db;

    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  });

  // --- Allocate goods ---
  // Map road tile index -> source index (in `sources`).
  // This lets us quickly test if a visited road tile is a producer access point.
  std::vector<int> sourceByRoadIdx(n, -1);
  for (int si = 0; si < static_cast<int>(sources.size()); ++si) {
    const int ridx = sources[static_cast<std::size_t>(si)].roadIdx;
    if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) continue;
    sourceByRoadIdx[static_cast<std::size_t>(ridx)] = si;
  }

  // Scratch buffers for on-demand "next-nearest producer" searches.
  // We avoid O(n) re-initialization by only resetting touched indices.
  constexpr int INF = std::numeric_limits<int>::max() / 4;
  std::vector<int> searchCost(n, INF);
  std::vector<int> searchSteps(n, INF);
  std::vector<int> searchParent(n, -1);
  std::vector<int> touched;
  touched.reserve(1024);

  auto resetSearchScratch = [&]() {
    for (int idx : touched) {
      if (idx < 0 || static_cast<std::size_t>(idx) >= n) continue;
      const std::size_t ui = static_cast<std::size_t>(idx);
      searchCost[ui] = INF;
      searchSteps[ui] = INF;
      searchParent[ui] = -1;
    }
    touched.clear();
  };

  // Find the nearest (travel-time) source with remaining supply, starting from a road tile.
  // On success, returns true and outputs:
  //  - outSourceIndex: index into `sources`
  //  - outSourceRoadIdx: the road tile idx where that source is anchored
  // The `searchParent` array encodes the chosen path *back to start* (prev pointers).
  auto findNearestSourceWithSupply = [&](int startRoadIdx, int& outSourceIndex, int& outSourceRoadIdx) -> bool {
    outSourceIndex = -1;
    outSourceRoadIdx = -1;

    if (!isTraversableRoad(startRoadIdx)) return false;

    resetSearchScratch();

    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchCmp> heap;

    const std::size_t us = static_cast<std::size_t>(startRoadIdx);
    searchCost[us] = 0;
    searchSteps[us] = 0;
    searchParent[us] = -1;
    touched.push_back(startRoadIdx);
    heap.push(SearchNode{0, 0, startRoadIdx});

    // Deterministic neighbor order (matches FlowField).
    constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

    while (!heap.empty()) {
      const SearchNode cur = heap.top();
      heap.pop();

      if (cur.idx < 0 || static_cast<std::size_t>(cur.idx) >= n) continue;
      const std::size_t uu = static_cast<std::size_t>(cur.idx);
      if (cur.cost != searchCost[uu] || cur.steps != searchSteps[uu]) continue;

      const int srcIdx = sourceByRoadIdx[uu];
      if (srcIdx >= 0 && srcIdx < static_cast<int>(sources.size()) && sources[static_cast<std::size_t>(srcIdx)].remaining > 0) {
        outSourceIndex = srcIdx;
        outSourceRoadIdx = cur.idx;
        return true;
      }

      const int ux = cur.idx % w;
      const int uy = cur.idx / w;

      for (const auto& d : dirs) {
        const int nx = ux + d[0];
        const int ny = uy + d[1];
        if (!world.inBounds(nx, ny)) continue;
        const int nidx = ny * w + nx;
        if (!isTraversableRoad(nidx)) continue;
        if (nidx < 0 || static_cast<std::size_t>(nidx) >= n) continue;

        const std::size_t nu = static_cast<std::size_t>(nidx);
        int moveCost = RoadTravelTimeMilliForLevel(static_cast<int>(world.at(nx, ny).level));
        const int nc = cur.cost + moveCost;
        const int ns = cur.steps + 1;

        bool improve = false;
        if (nc < searchCost[nu]) {
          improve = true;
        } else if (nc == searchCost[nu] && ns < searchSteps[nu]) {
          improve = true;
        } else if (nc == searchCost[nu] && ns == searchSteps[nu]) {
          const int oldParent = searchParent[nu];
          if (oldParent < 0 || cur.idx < oldParent) {
            improve = true;
          }
        }

        if (!improve) continue;

        if (searchCost[nu] == INF) {
          touched.push_back(nidx);
        }
        searchCost[nu] = nc;
        searchSteps[nu] = ns;
        searchParent[nu] = cur.idx;

        heap.push(SearchNode{nc, ns, nidx});
      }
    }

    return false;
  };

  // Add goods traffic along a path encoded by a "prev" parent map (node -> previous node).
  // `endIdx` is typically a source road tile; walking parents eventually reaches start (-1).
  auto addAlongPrevChain = [&](int endIdx, const std::vector<int>& prev, int amount) {
    if (amount <= 0) return;
    int cur = endIdx;
    int guard = 0;
    while (cur != -1 && guard++ < static_cast<int>(n) + 8) {
      if (cur < 0 || static_cast<std::size_t>(cur) >= n) break;
      const std::size_t ui = static_cast<std::size_t>(cur);
      out.roadGoodsTraffic[ui] = SatAddU16(out.roadGoodsTraffic[ui], static_cast<std::uint32_t>(amount));
      cur = prev[ui];
    }
  };

  for (const Consumer& c : consumers) {
    int remaining = c.demand;
    int delivered = 0;

    // Deliver from nearest local producer if possible.
    if (!sources.empty() && c.cost >= 0 && c.owner >= 0 && c.owner < static_cast<int>(sources.size())) {
      Source& src = sources[static_cast<std::size_t>(c.owner)];
      const int give = std::min(src.remaining, remaining);
      if (give > 0) {
        src.remaining -= give;
        remaining -= give;
        delivered += give;
        addAlongParentChain(c.roadIdx, prodField.parent, give);
      }
    }

    // If the nearest producer runs out, fall back to the next-nearest reachable producers
    // (deterministic travel-time Dijkstra) before importing.
    while (remaining > 0 && !sources.empty()) {
      int srcIdx = -1;
      int srcRoadIdx = -1;
      if (!findNearestSourceWithSupply(c.roadIdx, srcIdx, srcRoadIdx)) break;
      if (srcIdx < 0 || srcIdx >= static_cast<int>(sources.size())) break;

      Source& src = sources[static_cast<std::size_t>(srcIdx)];
      const int give = std::min(src.remaining, remaining);
      if (give <= 0) {
        // Defensive: should be impossible (findNearestSourceWithSupply checks remaining>0).
        src.remaining = 0;
        continue;
      }

      src.remaining -= give;
      remaining -= give;
      delivered += give;

      // `searchParent` stores prev pointers back to the consumer road tile.
      // Adding along the reverse chain still marks the same road tiles.
      addAlongPrevChain(srcRoadIdx, searchParent, give);
    }

    // Import any remaining demand from the edge if allowed.
    if (remaining > 0 && cfg.allowImports) {
      const int ed = (!edgeField.cost.empty()) ? edgeField.cost[static_cast<std::size_t>(c.roadIdx)] : -1;
      if (ed >= 0) {
        const int imp = remaining;
        remaining = 0;
        delivered += imp;
        out.goodsImported += imp;
        addAlongParentChain(c.roadIdx, edgeField.parent, imp);
      }
    }

    if (remaining > 0) {
      out.unreachableDemand += remaining;
    }

    out.goodsDelivered += delivered;

    // Commercial tile fill ratio for overlays.
    const float ratio = (c.demand > 0)
                            ? std::clamp(static_cast<float>(delivered) / static_cast<float>(c.demand), 0.0f, 1.0f)
                            : 1.0f;
    const std::size_t tidx = static_cast<std::size_t>(c.y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(c.x);
    if (tidx < out.commercialFill.size()) {
      out.commercialFill[tidx] = static_cast<std::uint8_t>(std::lround(ratio * 255.0f));
    }
  }

  // --- Export any surplus supply ---
  if (cfg.allowExports && !sources.empty()) {
    for (const Source& src : sources) {
      if (src.remaining <= 0) continue;
      const int ed = (!edgeField.cost.empty()) ? edgeField.cost[static_cast<std::size_t>(src.roadIdx)] : -1;
      if (ed < 0) continue;

      out.goodsExported += src.remaining;
      addAlongParentChain(src.roadIdx, edgeField.parent, src.remaining);
    }
  }

  // --- Post stats ---
  if (out.goodsDemand > 0) {
    out.satisfaction = std::clamp(static_cast<float>(out.goodsDelivered) / static_cast<float>(out.goodsDemand), 0.0f, 1.0f);
  } else {
    out.satisfaction = 1.0f;
  }

  int maxT = 0;
  for (std::uint16_t t : out.roadGoodsTraffic) {
    const int v = static_cast<int>(t);
    if (v > maxT) maxT = v;
  }
  out.maxRoadGoodsTraffic = maxT;

  return out;
}

} // namespace isocity
