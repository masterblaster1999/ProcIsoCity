#include "isocity/BlockDistricting.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace isocity {

namespace {

constexpr int kInf = std::numeric_limits<int>::max() / 4;

struct Item {
  int dist = kInf;
  int district = 0;
  int node = -1;
};

struct ItemGreater {
  bool operator()(const Item& a, const Item& b) const
  {
    if (a.dist != b.dist) return a.dist > b.dist;
    if (a.district != b.district) return a.district > b.district;
    return a.node > b.node;
  }
};

inline int ClampDistrictCount(int d)
{
  if (d < 1) return 1;
  if (d > kDistrictCount) return kDistrictCount;
  return d;
}

std::vector<int> BfsDist(const CityBlockGraphResult& g, int start)
{
  const int n = static_cast<int>(g.blocks.blocks.size());
  std::vector<int> dist(static_cast<std::size_t>(n), kInf);
  if (start < 0 || start >= n) return dist;

  std::queue<int> q;
  dist[static_cast<std::size_t>(start)] = 0;
  q.push(start);

  while (!q.empty()) {
    const int u = q.front();
    q.pop();
    const int du = dist[static_cast<std::size_t>(u)];

    const auto& inc = g.blockToEdges[static_cast<std::size_t>(u)];
    for (int ei : inc) {
      if (ei < 0 || ei >= static_cast<int>(g.edges.size())) continue;
      const CityBlockAdjacency& e = g.edges[static_cast<std::size_t>(ei)];
      const int v = (e.a == u) ? e.b : e.a;
      if (v < 0 || v >= n) continue;
      if (du >= kInf - 1) continue;
      const int nd = du + 1;
      if (nd < dist[static_cast<std::size_t>(v)]) {
        dist[static_cast<std::size_t>(v)] = nd;
        q.push(v);
      }
    }
  }

  return dist;
}

BlockDistrictResult ComputeFromGraph(const CityBlockGraphResult& g, const BlockDistrictConfig& cfg)
{
  BlockDistrictResult out;

  const int n = static_cast<int>(g.blocks.blocks.size());
  if (n <= 0) {
    out.districtsRequested = ClampDistrictCount(cfg.districts);
    out.districtsUsed = 0;
    return out;
  }

  out.districtsRequested = ClampDistrictCount(cfg.districts);
  const int K = std::min(out.districtsRequested, n);
  out.districtsUsed = K;

  // --- Seed selection (farthest-point sampling on block graph hop distance) ---
  std::vector<int> seeds;
  seeds.reserve(static_cast<std::size_t>(K));

  // First seed: largest block by area (tie: lowest id).
  int first = 0;
  int bestArea = g.blocks.blocks[0].area;
  for (int i = 1; i < n; ++i) {
    const int a = g.blocks.blocks[static_cast<std::size_t>(i)].area;
    if (a > bestArea) {
      bestArea = a;
      first = i;
    }
  }
  seeds.push_back(first);

  std::vector<int> minDist(static_cast<std::size_t>(n), kInf);
  {
    const std::vector<int> d = BfsDist(g, first);
    for (int i = 0; i < n; ++i) {
      minDist[static_cast<std::size_t>(i)] = std::min(minDist[static_cast<std::size_t>(i)], d[static_cast<std::size_t>(i)]);
    }
  }

  auto isSeed = [&](int id) -> bool {
    for (int s : seeds) {
      if (s == id) return true;
    }
    return false;
  };

  while (static_cast<int>(seeds.size()) < K) {
    int best = -1;
    int bestD = -1;
    int bestA = -1;

    for (int i = 0; i < n; ++i) {
      if (isSeed(i)) continue;
      const int d = minDist[static_cast<std::size_t>(i)];
      const int a = g.blocks.blocks[static_cast<std::size_t>(i)].area;

      // Prefer larger distance; INF beats any finite distance.
      if (best < 0 || d > bestD || (d == bestD && a > bestA) || (d == bestD && a == bestA && i < best)) {
        best = i;
        bestD = d;
        bestA = a;
      }
    }

    if (best < 0) break;
    seeds.push_back(best);

    const std::vector<int> d = BfsDist(g, best);
    for (int i = 0; i < n; ++i) {
      minDist[static_cast<std::size_t>(i)] = std::min(minDist[static_cast<std::size_t>(i)], d[static_cast<std::size_t>(i)]);
    }
  }

  out.seedBlockId = seeds;

  // --- Multi-source assignment with lexicographic tie-break: (dist, seedIndex) ---
  std::vector<int> dist(static_cast<std::size_t>(n), kInf);
  std::vector<int> owner(static_cast<std::size_t>(n), kDistrictCount); // larger than any valid district

  std::priority_queue<Item, std::vector<Item>, ItemGreater> pq;

  for (int d = 0; d < static_cast<int>(seeds.size()); ++d) {
    const int b = seeds[static_cast<std::size_t>(d)];
    dist[static_cast<std::size_t>(b)] = 0;
    owner[static_cast<std::size_t>(b)] = d;
    pq.push(Item{0, d, b});
  }

  while (!pq.empty()) {
    const Item cur = pq.top();
    pq.pop();

    const int u = cur.node;
    if (u < 0 || u >= n) continue;

    const int du = dist[static_cast<std::size_t>(u)];
    const int ou = owner[static_cast<std::size_t>(u)];
    if (cur.dist != du || cur.district != ou) continue;

    const auto& inc = g.blockToEdges[static_cast<std::size_t>(u)];
    for (int ei : inc) {
      if (ei < 0 || ei >= static_cast<int>(g.edges.size())) continue;
      const CityBlockAdjacency& e = g.edges[static_cast<std::size_t>(ei)];
      const int v = (e.a == u) ? e.b : e.a;
      if (v < 0 || v >= n) continue;

      if (du >= kInf - 1) continue;
      const int nd = du + 1;
      const int no = ou;

      const int dv = dist[static_cast<std::size_t>(v)];
      const int ov = owner[static_cast<std::size_t>(v)];
      if (nd < dv || (nd == dv && no < ov)) {
        dist[static_cast<std::size_t>(v)] = nd;
        owner[static_cast<std::size_t>(v)] = no;
        pq.push(Item{nd, no, v});
      }
    }
  }

  out.blockToDistrict.assign(static_cast<std::size_t>(n), 0);
  for (int i = 0; i < n; ++i) {
    int d = owner[static_cast<std::size_t>(i)];
    if (d < 0) d = 0;
    if (d >= kDistrictCount) d = 0;
    out.blockToDistrict[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(d);

    out.blocksPerDistrict[static_cast<std::size_t>(d)]++;
    out.tilesPerDistrict[static_cast<std::size_t>(d)] += g.blocks.blocks[static_cast<std::size_t>(i)].area;
  }

  return out;
}

} // namespace

BlockDistrictResult ComputeBlockDistricts(const World& world, const BlockDistrictConfig& cfg,
                                         const CityBlockGraphResult* precomputedGraph)
{
  CityBlockGraphResult graph;
  if (precomputedGraph) {
    graph = *precomputedGraph;
  } else {
    graph = BuildCityBlockGraph(world);
  }

  return ComputeFromGraph(graph, cfg);
}

BlockDistrictResult AssignDistrictsByBlocks(World& world, const BlockDistrictConfig& cfg,
                                           const CityBlockGraphResult* precomputedGraph)
{
  CityBlockGraphResult graph;
  if (precomputedGraph) {
    graph = *precomputedGraph;
  } else {
    graph = BuildCityBlockGraph(world);
  }

  BlockDistrictResult out = ComputeFromGraph(graph, cfg);
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  // Write block districts.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const int bid = (idx < graph.blocks.tileToBlock.size()) ? graph.blocks.tileToBlock[idx] : -1;
      if (bid >= 0 && bid < static_cast<int>(out.blockToDistrict.size())) {
        world.at(x, y).district = out.blockToDistrict[static_cast<std::size_t>(bid)];
        continue;
      }

      Tile& t = world.at(x, y);

      // Roads: choose majority district among adjacent blocks.
      if (cfg.fillRoadTiles && t.overlay == Overlay::Road) {
        int counts[kDistrictCount] = {0};
        int bestD = 0;
        int bestC = -1;

        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};

        for (int k = 0; k < 4; ++k) {
          const int x2 = nx[k];
          const int y2 = ny[k];
          if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;
          const std::size_t nidx = static_cast<std::size_t>(y2) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x2);
          const int nb = (nidx < graph.blocks.tileToBlock.size()) ? graph.blocks.tileToBlock[nidx] : -1;
          if (nb < 0 || nb >= static_cast<int>(out.blockToDistrict.size())) continue;
          const int d = static_cast<int>(out.blockToDistrict[static_cast<std::size_t>(nb)]);
          if (d < 0 || d >= kDistrictCount) continue;
          counts[d]++;
        }

        for (int d = 0; d < kDistrictCount; ++d) {
          const int c = counts[d];
          if (c > bestC || (c == bestC && d < bestD)) {
            bestC = c;
            bestD = d;
          }
        }

        t.district = static_cast<std::uint8_t>(bestD);
        continue;
      }

      // Water: optional. Only attempt to infer from adjacent non-water.
      if (cfg.includeWater && t.terrain == Terrain::Water) {
        int counts[kDistrictCount] = {0};
        int bestD = static_cast<int>(t.district);
        int bestC = -1;

        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};
        for (int k = 0; k < 4; ++k) {
          const int x2 = nx[k];
          const int y2 = ny[k];
          if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;
          const Tile& n = world.at(x2, y2);
          if (n.terrain == Terrain::Water) continue;
          const int d = static_cast<int>(n.district);
          if (d < 0 || d >= kDistrictCount) continue;
          counts[d]++;
        }

        for (int d = 0; d < kDistrictCount; ++d) {
          const int c = counts[d];
          if (c > bestC || (c == bestC && d < bestD)) {
            bestC = c;
            bestD = d;
          }
        }

        t.district = static_cast<std::uint8_t>(bestD);
        continue;
      }

      // Otherwise: leave unchanged.
    }
  }

  // Recompute per-district tile counts from the world (includes roads, etc.).
  out.tilesPerDistrict.fill(0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (!cfg.includeWater && world.at(x, y).terrain == Terrain::Water) continue;
      const int d = static_cast<int>(world.at(x, y).district);
      if (d < 0 || d >= kDistrictCount) continue;
      out.tilesPerDistrict[static_cast<std::size_t>(d)]++;
    }
  }

  return out;
}

} // namespace isocity
