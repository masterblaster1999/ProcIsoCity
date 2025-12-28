#include "isocity/Goods.hpp"

#include "isocity/Pathfinding.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
  int dist = -1;
  int owner = -1;
};

} // namespace

GoodsResult ComputeGoodsFlow(const World& world, const GoodsConfig& cfg,
                            const std::vector<std::uint8_t>* precomputedRoadToEdge)
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

  auto zoneHasAccess = [&](int zx, int zy) -> bool {
    if (!world.hasAdjacentRoad(zx, zy)) return false;
    if (!cfg.requireOutsideConnection) return true;
    return HasAdjacentRoadConnectedToEdge(world, *roadToEdge, zx, zy);
  };

  // --- Gather industrial supply per road tile (merge multiple producers on the same road access point) ---
  std::vector<int> supplyPerRoad(n, 0);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Industrial) continue;
      if (t.level == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      Point road{};
      if (!PickAdjacentRoadTile(world, roadToEdge, x, y, road)) continue;
      const int ridx = road.y * w + road.x;
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

  // --- Multi-source BFS from industrial sources (nearest-producer labeling) ---
  std::vector<int> dist(n, -1);
  std::vector<int> parent(n, -1);
  std::vector<int> owner(n, -1);
  std::vector<int> queue;
  queue.reserve(n / 2);

  if (!sources.empty()) {
    for (int si = 0; si < static_cast<int>(sources.size()); ++si) {
      const int sidx = sources[static_cast<std::size_t>(si)].roadIdx;
      if (sidx < 0 || static_cast<std::size_t>(sidx) >= n) continue;
      dist[static_cast<std::size_t>(sidx)] = 0;
      parent[static_cast<std::size_t>(sidx)] = -1;
      owner[static_cast<std::size_t>(sidx)] = si;
      queue.push_back(sidx);
    }

    std::size_t head = 0;
    while (head < queue.size()) {
      const int idx = queue[head++];
      const int x = idx % w;
      const int y = idx / w;

      const int dcur = dist[static_cast<std::size_t>(idx)];
      if (dcur < 0) continue;

      // Deterministic neighbor order: N, E, S, W.
      constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
      for (const auto& d : dirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const int nidx = ny * w + nx;
        if (!isTraversableRoad(nidx)) continue;
        if (dist[static_cast<std::size_t>(nidx)] != -1) continue;

        dist[static_cast<std::size_t>(nidx)] = dcur + 1;
        parent[static_cast<std::size_t>(nidx)] = idx;
        owner[static_cast<std::size_t>(nidx)] = owner[static_cast<std::size_t>(idx)];
        queue.push_back(nidx);
      }
    }
  }

  // --- BFS from map-edge roads (for imports/exports routing) ---
  std::vector<int> edgeDist(n, -1);
  std::vector<int> edgeParent(n, -1);
  std::vector<int> edgeQueue;
  edgeQueue.reserve(n / 4);

  auto pushEdge = [&](int ex, int ey) {
    const int ridx = ey * w + ex;
    if (!isTraversableRoad(ridx)) return;
    if (edgeDist[static_cast<std::size_t>(ridx)] != -1) return;
    edgeDist[static_cast<std::size_t>(ridx)] = 0;
    edgeParent[static_cast<std::size_t>(ridx)] = -1;
    edgeQueue.push_back(ridx);
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

  // Only run BFS if we may need it.
  if ((!edgeQueue.empty()) && (cfg.allowImports || cfg.allowExports)) {
    std::size_t head = 0;
    while (head < edgeQueue.size()) {
      const int idx = edgeQueue[head++];
      const int x = idx % w;
      const int y = idx / w;

      const int dcur = edgeDist[static_cast<std::size_t>(idx)];
      if (dcur < 0) continue;

      // Deterministic neighbor order: N, E, S, W.
      constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
      for (const auto& d : dirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const int nidx = ny * w + nx;
        if (!isTraversableRoad(nidx)) continue;
        if (edgeDist[static_cast<std::size_t>(nidx)] != -1) continue;

        edgeDist[static_cast<std::size_t>(nidx)] = dcur + 1;
        edgeParent[static_cast<std::size_t>(nidx)] = idx;
        edgeQueue.push_back(nidx);
      }
    }
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
      if (!zoneHasAccess(x, y)) continue;

      const float raw = static_cast<float>(BaseCommercialDemand(static_cast<int>(t.level))) * cfg.demandScale;
      const int demand = std::max(0, static_cast<int>(std::lround(raw)));
      if (demand <= 0) continue;

      Point road{};
      if (!PickAdjacentRoadTile(world, roadToEdge, x, y, road)) continue;
      const int ridx = road.y * w + road.x;
      if (!isTraversableRoad(ridx)) continue;

      const int d = (!sources.empty()) ? dist[static_cast<std::size_t>(ridx)] : -1;
      const int own = (d >= 0) ? owner[static_cast<std::size_t>(ridx)] : -1;

      consumers.push_back(Consumer{x, y, ridx, demand, d, own});
      out.goodsDemand += demand;
    }
  }

  // Prioritize closer consumers first so scarce supply serves the nearest commercial areas.
  std::sort(consumers.begin(), consumers.end(), [&](const Consumer& a, const Consumer& b) {
    const int da = (a.dist >= 0) ? a.dist : std::numeric_limits<int>::max();
    const int db = (b.dist >= 0) ? b.dist : std::numeric_limits<int>::max();
    if (da != db) return da < db;
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  });

  // --- Allocate goods ---
  int deliveredTotal = 0;

  for (const Consumer& c : consumers) {
    int remaining = c.demand;
    int delivered = 0;

    // Deliver from nearest local producer if possible.
    if (!sources.empty() && c.dist >= 0 && c.owner >= 0 && c.owner < static_cast<int>(sources.size())) {
      Source& src = sources[static_cast<std::size_t>(c.owner)];
      const int give = std::min(src.remaining, remaining);
      if (give > 0) {
        src.remaining -= give;
        remaining -= give;
        delivered += give;
        addAlongParentChain(c.roadIdx, parent, give);
      }
    }

    // Import any remaining demand from the edge if allowed.
    if (remaining > 0 && cfg.allowImports) {
      const int ed = edgeDist[static_cast<std::size_t>(c.roadIdx)];
      if (ed >= 0) {
        const int imp = remaining;
        remaining = 0;
        delivered += imp;
        out.goodsImported += imp;
        addAlongParentChain(c.roadIdx, edgeParent, imp);
      }
    }

    if (remaining > 0) {
      out.unreachableDemand += remaining;
    }

    out.goodsDelivered += delivered;
    deliveredTotal += delivered;

    // Commercial tile fill ratio for overlays.
    const float ratio = (c.demand > 0) ? std::clamp(static_cast<float>(delivered) / static_cast<float>(c.demand), 0.0f, 1.0f)
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
      const int ed = edgeDist[static_cast<std::size_t>(src.roadIdx)];
      if (ed < 0) continue;

      out.goodsExported += src.remaining;
      addAlongParentChain(src.roadIdx, edgeParent, src.remaining);
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

  (void)deliveredTotal;

  return out;
}

} // namespace isocity
