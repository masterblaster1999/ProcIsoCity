#include "isocity/Districting.hpp"

#include "isocity/FlowField.hpp"
#include "isocity/Pathfinding.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <queue>

namespace isocity {

namespace {

inline int ClampDistrictCount(int d)
{
  return std::clamp(d, 1, kDistrictCount);
}

// Deterministic neighbor order (tile grid).
constexpr int kDirs[4][2] = {
    {0, -1},
    {1, 0},
    {0, 1},
    {-1, 0},
};

struct HeapNode {
  int dist = 0;
  int owner = 0;
  int idx = 0;
};

struct HeapCmp {
  bool operator()(const HeapNode& a, const HeapNode& b) const
  {
    // Min-heap via priority_queue (reverse compare).
    if (a.dist != b.dist) return a.dist > b.dist;
    if (a.owner != b.owner) return a.owner > b.owner;
    return a.idx > b.idx;
  }
};

static int PickRoadClosestToCenter(const World& world, const std::vector<int>& roadIdx)
{
  if (roadIdx.empty()) return -1;
  const int cx = world.width() / 2;
  const int cy = world.height() / 2;

  int bestIdx = roadIdx.front();
  int bestD = std::numeric_limits<int>::max();
  for (int idx : roadIdx) {
    const int x = idx % world.width();
    const int y = idx / world.width();
    const int d = std::abs(x - cx) + std::abs(y - cy);
    if (d < bestD || (d == bestD && idx < bestIdx)) {
      bestD = d;
      bestIdx = idx;
    }
  }
  return bestIdx;
}

static std::vector<int> CollectRoadIdx(const World& world, const AutoDistrictConfig& cfg,
                                       std::vector<std::uint8_t>* outRoadToEdgeMask)
{
  std::vector<int> roads;
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  std::vector<std::uint8_t> local;
  const std::vector<std::uint8_t>* mask = nullptr;
  if (cfg.requireOutsideConnection) {
    ComputeRoadsConnectedToEdge(world, local);
    mask = &local;
  }

  roads.reserve(n / 8u);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int idx = y * w + x;
      if (mask) {
        const std::size_t ui = static_cast<std::size_t>(idx);
        if (ui >= mask->size() || (*mask)[ui] == 0) continue;
      }
      roads.push_back(idx);
    }
  }

  if (outRoadToEdgeMask) {
    if (mask) {
      *outRoadToEdgeMask = *mask;
    } else {
      outRoadToEdgeMask->clear();
    }
  }
  return roads;
}

static void FarthestPointSeedsOnRoads(const World& world, const std::vector<int>& roads,
                                      const AutoDistrictConfig& cfg,
                                      const std::vector<std::uint8_t>* roadToEdgeMask,
                                      std::vector<int>& outSeeds)
{
  outSeeds.clear();
  const int want = ClampDistrictCount(cfg.districts);
  if (roads.empty() || want <= 0) return;

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  std::vector<std::uint8_t> isSeed(n, std::uint8_t{0});

  const int first = PickRoadClosestToCenter(world, roads);
  if (first < 0) return;
  outSeeds.push_back(first);
  isSeed[static_cast<std::size_t>(first)] = 1u;

  for (int k = 1; k < want; ++k) {
    RoadFlowFieldConfig fcfg;
    fcfg.requireOutsideConnection = cfg.requireOutsideConnection;
    fcfg.computeOwner = false;
    fcfg.useTravelTime = cfg.useTravelTime;
    const RoadFlowField ff = BuildRoadFlowField(world, outSeeds, fcfg, roadToEdgeMask, nullptr);

    // Prefer selecting an unreachable road tile (disconnected component) to ensure coverage.
    int bestUnreach = -1;
    for (int ridx : roads) {
      const std::size_t ui = static_cast<std::size_t>(ridx);
      if (ui >= ff.dist.size()) continue;
      if (isSeed[ui]) continue;
      if (ff.dist[ui] == -1) {
        if (bestUnreach < 0 || ridx < bestUnreach) bestUnreach = ridx;
      }
    }
    if (bestUnreach >= 0) {
      outSeeds.push_back(bestUnreach);
      isSeed[static_cast<std::size_t>(bestUnreach)] = 1u;
      continue;
    }

    int bestIdx = -1;
    int bestMetric = -1;
    for (int ridx : roads) {
      const std::size_t ui = static_cast<std::size_t>(ridx);
      if (ui >= ff.dist.size()) continue;
      if (isSeed[ui]) continue;
      const int metric = cfg.useTravelTime ? ff.cost[ui] : ff.dist[ui];
      if (metric < 0) continue;
      if (metric > bestMetric || (metric == bestMetric && ridx < bestIdx)) {
        bestMetric = metric;
        bestIdx = ridx;
      }
    }

    if (bestIdx < 0) break;
    outSeeds.push_back(bestIdx);
    isSeed[static_cast<std::size_t>(bestIdx)] = 1u;
  }
}

static void AssignRoadOwners(const World& world, const std::vector<int>& seeds,
                             const AutoDistrictConfig& cfg,
                             const std::vector<std::uint8_t>* roadToEdgeMask,
                             std::vector<int>& outRoadOwner)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  outRoadOwner.assign(n, -1);
  if (seeds.empty() || w <= 0 || h <= 0) return;

  RoadFlowFieldConfig fcfg;
  fcfg.requireOutsideConnection = cfg.requireOutsideConnection;
  fcfg.computeOwner = true;
  fcfg.useTravelTime = cfg.useTravelTime;
  const RoadFlowField ff = BuildRoadFlowField(world, seeds, fcfg, roadToEdgeMask, nullptr);
  if (ff.owner.empty()) return;

  for (std::size_t i = 0; i < n && i < ff.owner.size(); ++i) {
    // Only keep ownership for road tiles.
    const int x = static_cast<int>(i % static_cast<std::size_t>(w));
    const int y = static_cast<int>(i / static_cast<std::size_t>(w));
    if (!world.inBounds(x, y)) continue;
    if (world.at(x, y).overlay != Overlay::Road) continue;

    const int owner = ff.owner[i];
    if (owner < 0) continue;
    outRoadOwner[i] = owner;
  }
}

static void PropagateOwnersFromRoads(const World& world, const AutoDistrictConfig& cfg,
                                     const std::vector<int>& roadOwner,
                                     std::vector<std::uint8_t>& outDistricts)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  outDistricts.assign(n, 0u);
  if (w <= 0 || h <= 0) return;

  constexpr int INF = std::numeric_limits<int>::max() / 4;
  std::vector<int> dist(n, INF);
  std::vector<int> owner(n, -1);

  std::priority_queue<HeapNode, std::vector<HeapNode>, HeapCmp> heap;

  // Seed all road tiles that have an owner.
  for (std::size_t i = 0; i < n; ++i) {
    const int o = (i < roadOwner.size()) ? roadOwner[i] : -1;
    if (o < 0) continue;
    dist[i] = 0;
    owner[i] = o;
    heap.push(HeapNode{0, o, static_cast<int>(i)});
  }

  while (!heap.empty()) {
    const HeapNode cur = heap.top();
    heap.pop();

    const std::size_t ui = static_cast<std::size_t>(cur.idx);
    if (ui >= n) continue;
    if (cur.dist != dist[ui] || cur.owner != owner[ui]) continue;

    const int x = cur.idx % w;
    const int y = cur.idx / w;

    for (const auto& d : kDirs) {
      const int nx = x + d[0];
      const int ny = y + d[1];
      if (!world.inBounds(nx, ny)) continue;
      const int nidx = ny * w + nx;
      const std::size_t nu = static_cast<std::size_t>(nidx);

      if (!cfg.includeWater && world.at(nx, ny).terrain == Terrain::Water) {
        continue;
      }

      const int nd = cur.dist + 1;
      const int no = cur.owner;

      bool improve = false;
      if (nd < dist[nu]) {
        improve = true;
      } else if (nd == dist[nu] && (owner[nu] < 0 || no < owner[nu])) {
        improve = true;
      }

      if (!improve) continue;
      dist[nu] = nd;
      owner[nu] = no;
      heap.push(HeapNode{nd, no, nidx});
    }
  }

  // Emit districts.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (i >= n) continue;

      if (!cfg.includeWater && world.at(x, y).terrain == Terrain::Water) {
        // Leave unchanged (caller can pre-fill).
        continue;
      }

      const int o = owner[i];
      const int d = (o < 0) ? 0 : std::clamp(o, 0, kDistrictCount - 1);
      outDistricts[i] = static_cast<std::uint8_t>(d);
    }
  }
}

static void FallbackVoronoiNoRoads(const World& world, const AutoDistrictConfig& cfg,
                                   std::vector<std::uint8_t>& outDistricts)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  outDistricts.assign(n, 0u);
  if (w <= 0 || h <= 0) return;

  const int want = ClampDistrictCount(cfg.districts);

  // Candidate tiles: buildable land unless includeWater is true.
  std::vector<int> candidates;
  candidates.reserve(n);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (!cfg.includeWater && t.terrain == Terrain::Water) continue;
      if (!cfg.includeWater && !world.isBuildable(x, y)) continue;
      candidates.push_back(y * w + x);
    }
  }
  if (candidates.empty()) return;

  // Seed picking: farthest-point sampling by Manhattan distance.
  std::vector<int> seeds;
  seeds.reserve(static_cast<std::size_t>(want));

  // First seed: closest candidate to center.
  {
    const int cx = w / 2;
    const int cy = h / 2;
    int best = candidates.front();
    int bestD = std::numeric_limits<int>::max();
    for (int idx : candidates) {
      const int x = idx % w;
      const int y = idx / w;
      const int d = std::abs(x - cx) + std::abs(y - cy);
      if (d < bestD || (d == bestD && idx < best)) {
        bestD = d;
        best = idx;
      }
    }
    seeds.push_back(best);
  }

  // Track distance to nearest seed for candidates.
  std::vector<int> bestDist(candidates.size(), 0);
  auto distTo = [&](int aIdx, int bIdx) -> int {
    const int ax = aIdx % w;
    const int ay = aIdx / w;
    const int bx = bIdx % w;
    const int by = bIdx / w;
    return std::abs(ax - bx) + std::abs(ay - by);
  };

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    bestDist[i] = distTo(candidates[i], seeds.front());
  }

  while (static_cast<int>(seeds.size()) < want) {
    int bestIdx = -1;
    int bestD = -1;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const int idx = candidates[i];
      const int d = bestDist[i];
      if (d > bestD || (d == bestD && idx < bestIdx)) {
        bestD = d;
        bestIdx = idx;
      }
    }
    if (bestIdx < 0) break;
    seeds.push_back(bestIdx);

    // Update nearest distances.
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      bestDist[i] = std::min(bestDist[i], distTo(candidates[i], bestIdx));
    }
  }

  // Assign all tiles to nearest seed.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (i >= n) continue;

      if (!cfg.includeWater && world.at(x, y).terrain == Terrain::Water) {
        continue;
      }

      int bestSeed = 0;
      int bestD = std::numeric_limits<int>::max();
      for (int si = 0; si < static_cast<int>(seeds.size()); ++si) {
        const int sidx = seeds[static_cast<std::size_t>(si)];
        const int d = distTo(y * w + x, sidx);
        if (d < bestD || (d == bestD && si < bestSeed)) {
          bestD = d;
          bestSeed = si;
        }
      }
      outDistricts[i] = static_cast<std::uint8_t>(std::clamp(bestSeed, 0, kDistrictCount - 1));
    }
  }
}

} // namespace

AutoDistrictResult ComputeAutoDistricts(const World& world, std::vector<std::uint8_t>& outDistricts,
                                       const AutoDistrictConfig& cfg)
{
  AutoDistrictResult res;
  res.districtsRequested = ClampDistrictCount(cfg.districts);

  // Start from the existing districts if we need to preserve water.
  if (!cfg.includeWater) {
    const int w = world.width();
    const int h = world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
    outDistricts.resize(n);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        if (i >= n) continue;
        outDistricts[i] = world.at(x, y).district;
      }
    }
  }

  std::vector<std::uint8_t> roadToEdgeMask;
  const std::vector<int> roads = CollectRoadIdx(world, cfg, &roadToEdgeMask);
  const std::vector<std::uint8_t>* roadMaskPtr = cfg.requireOutsideConnection ? &roadToEdgeMask : nullptr;

  if (roads.empty()) {
    // No roads: fall back to a simple Voronoi partition over tiles.
    FallbackVoronoiNoRoads(world, cfg, outDistricts);
    res.districtsUsed = std::min(res.districtsRequested, kDistrictCount);
    res.seedRoadIdx.clear();
    return res;
  }

  // 1) Pick seed road tiles.
  std::vector<int> seeds;
  FarthestPointSeedsOnRoads(world, roads, cfg, roadMaskPtr, seeds);
  if (seeds.empty()) {
    // Shouldn't happen, but keep behavior defined.
    FallbackVoronoiNoRoads(world, cfg, outDistricts);
    res.districtsUsed = std::min(res.districtsRequested, kDistrictCount);
    res.seedRoadIdx.clear();
    return res;
  }
  res.seedRoadIdx = seeds;
  res.districtsUsed = std::min(static_cast<int>(seeds.size()), kDistrictCount);

  // 2) Assign each road tile to its nearest seed.
  std::vector<int> roadOwner;
  AssignRoadOwners(world, seeds, cfg, roadMaskPtr, roadOwner);

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  if (outDistricts.size() != n) outDistricts.assign(n, 0u);

  // If we are *not* filling all tiles, just apply road ownership.
  if (!cfg.fillAllTiles) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        if (i >= n) continue;
        if (world.at(x, y).overlay != Overlay::Road) continue;
        const int o = (i < roadOwner.size()) ? roadOwner[i] : -1;
        if (o < 0) continue;
        outDistricts[i] = static_cast<std::uint8_t>(std::clamp(o, 0, kDistrictCount - 1));
      }
    }
    return res;
  }

  // 3) Propagate those road owners out to all tiles by nearest-road distance.
  PropagateOwnersFromRoads(world, cfg, roadOwner, outDistricts);
  return res;
}

AutoDistrictResult AutoAssignDistricts(World& world, const AutoDistrictConfig& cfg)
{
  std::vector<std::uint8_t> districts;
  AutoDistrictResult res = ComputeAutoDistricts(world, districts, cfg);

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  if (districts.size() != n) return res;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (i >= n) continue;
      world.at(x, y).district = districts[i];
    }
  }

  return res;
}

} // namespace isocity
