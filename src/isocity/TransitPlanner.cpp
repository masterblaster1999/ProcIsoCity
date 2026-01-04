#include "isocity/TransitPlanner.hpp"

#include "isocity/Road.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace isocity {

namespace {

constexpr std::uint64_t kInf64 = (std::numeric_limits<std::uint64_t>::max() / 4u);

std::uint64_t SplitMix64(std::uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

inline int Manhattan(const Point& a, const Point& b)
{
  const int dx = (a.x > b.x) ? (a.x - b.x) : (b.x - a.x);
  const int dy = (a.y > b.y) ? (a.y - b.y) : (b.y - a.y);
  return dx + dy;
}

std::vector<std::uint64_t> ComputeBaseEdgeCost(const RoadGraph& g, TransitEdgeWeightMode mode, const World* world)
{
  std::vector<std::uint64_t> cost;
  cost.resize(g.edges.size(), 1);

  for (std::size_t ei = 0; ei < g.edges.size(); ++ei) {
    const RoadGraphEdge& e = g.edges[ei];

    std::uint64_t w = 1;
    if (mode == TransitEdgeWeightMode::Steps) {
      w = static_cast<std::uint64_t>(std::max(1, e.length));
    } else {
      if (!world) {
        w = static_cast<std::uint64_t>(std::max(1, e.length)) * 1000ull;
      } else {
        std::uint64_t sum = 0;
        if (e.tiles.size() >= 2) {
          for (std::size_t k = 1; k < e.tiles.size(); ++k) {
            const Point& p = e.tiles[k];
            if (!world->inBounds(p.x, p.y)) continue;
            const Tile& t = world->at(p.x, p.y);
            if (t.overlay != Overlay::Road) continue;
            const int level = static_cast<int>(t.level);
            const int c = (t.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(level)
                                                        : RoadTravelTimeMilliForLevel(level);
            sum += static_cast<std::uint64_t>(std::max(1, c));
          }
        }
        if (sum == 0) sum = static_cast<std::uint64_t>(std::max(1, e.length)) * 1000ull;
        w = sum;
      }
    }

    cost[ei] = std::max<std::uint64_t>(1, w);
  }

  return cost;
}

struct DijkstraOut {
  std::vector<std::uint64_t> dist;
  std::vector<int> parent;
  std::vector<int> parentEdge;
};

template <typename EdgeCostFn>
DijkstraOut Dijkstra(const RoadGraph& g, int start, EdgeCostFn edgeCost)
{
  DijkstraOut out;
  const int n = static_cast<int>(g.nodes.size());
  out.dist.assign(static_cast<std::size_t>(n), kInf64);
  out.parent.assign(static_cast<std::size_t>(n), -1);
  out.parentEdge.assign(static_cast<std::size_t>(n), -1);
  if (start < 0 || start >= n) return out;

  using Item = std::pair<std::uint64_t, int>; // (dist, node)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  out.dist[static_cast<std::size_t>(start)] = 0;
  pq.push({0, start});

  while (!pq.empty()) {
    const Item cur = pq.top();
    pq.pop();
    const std::uint64_t d = cur.first;
    const int u = cur.second;
    if (u < 0 || u >= n) continue;
    if (d != out.dist[static_cast<std::size_t>(u)]) continue;

    const RoadGraphNode& nu = g.nodes[static_cast<std::size_t>(u)];
    for (int ei : nu.edges) {
      if (ei < 0 || ei >= static_cast<int>(g.edges.size())) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
      const int v = (e.a == u) ? e.b : e.a;
      if (v < 0 || v >= n) continue;
      const std::uint64_t w = edgeCost(ei);
      if (w == 0 || d > kInf64 - w) continue;
      const std::uint64_t nd = d + w;
      if (nd < out.dist[static_cast<std::size_t>(v)]) {
        out.dist[static_cast<std::size_t>(v)] = nd;
        out.parent[static_cast<std::size_t>(v)] = u;
        out.parentEdge[static_cast<std::size_t>(v)] = ei;
        pq.push({nd, v});
      }
    }
  }

  return out;
}

bool ReconstructPath(const DijkstraOut& dj, int start, int goal, std::vector<int>& outNodes, std::vector<int>& outEdges)
{
  outNodes.clear();
  outEdges.clear();
  if (start < 0 || goal < 0) return false;
  if (start >= static_cast<int>(dj.dist.size()) || goal >= static_cast<int>(dj.dist.size())) return false;
  if (dj.dist[static_cast<std::size_t>(goal)] >= kInf64) return false;

  int cur = goal;
  while (cur != -1) {
    outNodes.push_back(cur);
    if (cur == start) break;
    const int pe = dj.parentEdge[static_cast<std::size_t>(cur)];
    const int pv = dj.parent[static_cast<std::size_t>(cur)];
    if (pe < 0 || pv < 0) {
      // Broken chain.
      outNodes.clear();
      outEdges.clear();
      return false;
    }
    outEdges.push_back(pe);
    cur = pv;
  }

  if (outNodes.empty() || outNodes.back() != start) {
    outNodes.clear();
    outEdges.clear();
    return false;
  }

  std::reverse(outNodes.begin(), outNodes.end());
  std::reverse(outEdges.begin(), outEdges.end());
  return outNodes.size() >= 2 && outEdges.size() + 1 == outNodes.size();
}

std::vector<int> PickEndpointNodes(const RoadGraph& g,
                                  const std::vector<std::uint64_t>& edgeDemand,
                                  int endpointCandidates,
                                  std::uint64_t seedSalt)
{
  std::vector<int> out;
  const int n = static_cast<int>(g.nodes.size());
  if (n <= 0 || endpointCandidates <= 0) return out;

  // Node weight = incident demand.
  std::vector<std::uint64_t> nodeW(static_cast<std::size_t>(n), 0);
  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const std::uint64_t d = (ei < static_cast<int>(edgeDemand.size())) ? edgeDemand[static_cast<std::size_t>(ei)] : 0;
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    if (e.a >= 0 && e.a < n) nodeW[static_cast<std::size_t>(e.a)] += d;
    if (e.b >= 0 && e.b < n) nodeW[static_cast<std::size_t>(e.b)] += d;
  }

  struct Cand {
    std::uint64_t w = 0;
    std::uint64_t h = 0;
    int id = 0;
  };

  std::vector<Cand> c;
  c.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const std::uint64_t w = nodeW[static_cast<std::size_t>(i)];
    // Keep even zero-weight nodes around; farthest-first can still benefit on sparse maps.
    const std::uint64_t h = SplitMix64(static_cast<std::uint64_t>(i) ^ seedSalt);
    c.push_back(Cand{w, h, i});
  }

  std::sort(c.begin(), c.end(), [](const Cand& a, const Cand& b) {
    // Desc by weight, then stable hash, then id.
    if (a.w != b.w) return a.w > b.w;
    if (a.h != b.h) return a.h < b.h;
    return a.id < b.id;
  });

  // Candidate pool: take some multiple of endpointCandidates from the top.
  const int pool = std::min<int>(n, std::max<int>(endpointCandidates * 4, endpointCandidates));
  if (pool <= 0) return out;

  std::vector<int> poolIds;
  poolIds.reserve(static_cast<std::size_t>(pool));
  for (int i = 0; i < pool; ++i) poolIds.push_back(c[static_cast<std::size_t>(i)].id);

  // Farthest-first selection in Manhattan tile space for determinism & speed.
  std::vector<char> selected(static_cast<std::size_t>(n), 0);
  auto add = [&](int id) {
    if (id < 0 || id >= n) return;
    if (selected[static_cast<std::size_t>(id)]) return;
    selected[static_cast<std::size_t>(id)] = 1;
    out.push_back(id);
  };

  add(poolIds.front());
  while (static_cast<int>(out.size()) < endpointCandidates && static_cast<int>(out.size()) < pool) {
    int best = -1;
    int bestDist = -1;
    std::uint64_t bestW = 0;
    std::uint64_t bestH = 0;

    for (int id : poolIds) {
      if (id < 0 || id >= n) continue;
      if (selected[static_cast<std::size_t>(id)]) continue;
      const Point p = g.nodes[static_cast<std::size_t>(id)].pos;

      int near = std::numeric_limits<int>::max();
      for (int sid : out) {
        const Point sp = g.nodes[static_cast<std::size_t>(sid)].pos;
        near = std::min(near, Manhattan(p, sp));
      }

      const std::uint64_t w = nodeW[static_cast<std::size_t>(id)];
      const std::uint64_t h = SplitMix64(static_cast<std::uint64_t>(id) ^ seedSalt);

      if (near > bestDist || (near == bestDist && (w > bestW || (w == bestW && (h < bestH || (h == bestH && id < best)))))) {
        bestDist = near;
        best = id;
        bestW = w;
        bestH = h;
      }
    }

    if (best < 0) break;
    add(best);
  }

  // Keep order stable (not required for correctness, but makes outputs friendlier).
  std::sort(out.begin(), out.end());
  return out;
}

} // namespace

TransitPlan PlanTransitLines(const RoadGraph& g, const std::vector<std::uint64_t>& edgeDemand,
                             const TransitPlannerConfig& cfg, const World* world)
{
  TransitPlan plan;
  plan.cfg = cfg;
  if (edgeDemand.size() != g.edges.size()) return plan;

  const int n = static_cast<int>(g.nodes.size());
  if (n <= 1 || g.edges.empty()) return plan;

  // Precompute base edge costs.
  const std::vector<std::uint64_t> baseCost = ComputeBaseEdgeCost(g, cfg.weightMode, world);

  // Total demand.
  for (std::uint64_t d : edgeDemand) plan.totalDemand += d;

  // Pick endpoints.
  const std::uint64_t salt = SplitMix64(cfg.seedSalt ^ plan.totalDemand);
  const std::vector<int> endpoints = PickEndpointNodes(g, edgeDemand, cfg.endpointCandidates, salt);
  if (endpoints.size() < 2) return plan;

  // Precompute base shortest distances between endpoints (detour reference).
  std::vector<std::vector<std::uint64_t>> baseDist;
  baseDist.resize(endpoints.size());
  for (std::size_t i = 0; i < endpoints.size(); ++i) {
    const int s = endpoints[i];
    const DijkstraOut dj = Dijkstra(g, s, [&](int ei) { return baseCost[static_cast<std::size_t>(ei)]; });
    baseDist[i].resize(endpoints.size(), kInf64);
    for (std::size_t j = 0; j < endpoints.size(); ++j) {
      const int t = endpoints[j];
      if (t < 0 || t >= static_cast<int>(dj.dist.size())) continue;
      baseDist[i][j] = dj.dist[static_cast<std::size_t>(t)];
    }
  }

  // Remaining demand that we mutate as we pick lines.
  std::vector<std::uint64_t> rem = edgeDemand;

  auto recomputeMaxDemand = [&]() {
    std::uint64_t mx = 0;
    for (std::uint64_t d : rem) mx = std::max(mx, d);
    return std::max<std::uint64_t>(1, mx);
  };

  std::uint64_t maxDemand = recomputeMaxDemand();

  for (int li = 0; li < std::max(0, cfg.maxLines); ++li) {
    TransitLine best;
    double bestScore = -1.0;
    std::uint64_t bestDemand = 0;
    std::uint64_t bestCost = 0;
    int bestA = -1, bestB = -1;

    for (std::size_t ai = 0; ai < endpoints.size(); ++ai) {
      for (std::size_t bi = ai + 1; bi < endpoints.size(); ++bi) {
        const int a = endpoints[ai];
        const int b = endpoints[bi];
        const std::uint64_t baseShortest = baseDist[ai][bi];
        if (baseShortest >= kInf64 || baseShortest == 0) continue;

        // Dijkstra with demand-biased cost.
        auto edgeCost = [&](int ei) -> std::uint64_t {
          if (ei < 0 || ei >= static_cast<int>(rem.size())) return 1;
          const std::uint64_t bc = baseCost[static_cast<std::size_t>(ei)];
          const std::uint64_t d = rem[static_cast<std::size_t>(ei)];
          const double dn = (d >= cfg.minEdgeDemand) ? (static_cast<double>(d) / static_cast<double>(maxDemand)) : 0.0;
          const double clamped = (dn < 0.0) ? 0.0 : (dn > 1.0 ? 1.0 : dn);
          const double mult = 1.0 + cfg.demandBias * (1.0 - clamped);
          const double w = static_cast<double>(bc) * mult;
          const double wClamped = (w < 1.0) ? 1.0 : w;
          if (wClamped > static_cast<double>(kInf64 - 1)) return kInf64 - 1;
          return static_cast<std::uint64_t>(wClamped + 0.5);
        };

        const DijkstraOut dj = Dijkstra(g, a, edgeCost);
        std::vector<int> pathNodes;
        std::vector<int> pathEdges;
        if (!ReconstructPath(dj, a, b, pathNodes, pathEdges)) continue;

        // Compute base cost and remaining-demand covered by this path.
        std::uint64_t pathCost = 0;
        std::uint64_t pathDemand = 0;
        for (int ei : pathEdges) {
          if (ei < 0 || ei >= static_cast<int>(rem.size())) continue;
          pathCost += baseCost[static_cast<std::size_t>(ei)];
          pathDemand += rem[static_cast<std::size_t>(ei)];
        }

        if (pathCost == 0) continue;
        if (pathDemand < cfg.minLineDemand) continue;

        const double detour = static_cast<double>(pathCost) / static_cast<double>(baseShortest);
        if (cfg.maxDetour > 0.0 && detour > cfg.maxDetour) continue;

        const double score = static_cast<double>(pathDemand) / static_cast<double>(pathCost);

        if (score > bestScore ||
            (score == bestScore && (pathDemand > bestDemand ||
                                    (pathDemand == bestDemand && (pathCost < bestCost ||
                                                                 (pathCost == bestCost && (a < bestA ||
                                                                                          (a == bestA && b < bestB)))))))) {
          bestScore = score;
          bestDemand = pathDemand;
          bestCost = pathCost;
          bestA = a;
          bestB = b;
          best.nodes = std::move(pathNodes);
          best.edges = std::move(pathEdges);
          best.sumDemand = pathDemand;
          best.baseCost = pathCost;
        }
      }
    }

    if (bestScore < 0.0 || best.nodes.size() < 2 || best.edges.empty()) break;

    best.id = static_cast<int>(plan.lines.size());
    plan.lines.push_back(best);

    // Consume demand on used edges.
    for (int ei : plan.lines.back().edges) {
      if (ei < 0 || ei >= static_cast<int>(rem.size())) continue;
      const std::uint64_t before = rem[static_cast<std::size_t>(ei)];
      if (before == 0) continue;
      const double afterF = static_cast<double>(before) * (1.0 - std::max(0.0, std::min(1.0, cfg.coverFraction)));
      const std::uint64_t after = static_cast<std::uint64_t>(afterF + 0.5);
      rem[static_cast<std::size_t>(ei)] = after;
      plan.coveredDemand += (before - after);
    }

    maxDemand = recomputeMaxDemand();
    if (maxDemand == 0) break;
  }

  return plan;
}

bool BuildTransitLineTilePolyline(const RoadGraph& g, const TransitLine& line, std::vector<Point>& outTiles)
{
  outTiles.clear();
  if (line.nodes.size() < 2) return false;
  if (line.edges.size() + 1 != line.nodes.size()) return false;

  for (std::size_t i = 0; i < line.edges.size(); ++i) {
    const int ei = line.edges[i];
    const int a = line.nodes[i];
    const int b = line.nodes[i + 1];
    if (ei < 0 || ei >= static_cast<int>(g.edges.size())) return false;
    if (a < 0 || a >= static_cast<int>(g.nodes.size())) return false;
    if (b < 0 || b >= static_cast<int>(g.nodes.size())) return false;

    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const Point pa = g.nodes[static_cast<std::size_t>(a)].pos;
    const Point pb = g.nodes[static_cast<std::size_t>(b)].pos;

    if (e.tiles.empty()) return false;

    bool forward = true;
    if (e.tiles.front().x == pa.x && e.tiles.front().y == pa.y && e.tiles.back().x == pb.x && e.tiles.back().y == pb.y) {
      forward = true;
    } else if (e.tiles.front().x == pb.x && e.tiles.front().y == pb.y && e.tiles.back().x == pa.x && e.tiles.back().y == pa.y) {
      forward = false;
    } else {
      // Unexpected; fall back to comparing with edge endpoints.
      forward = (e.a == a && e.b == b);
    }

    if (forward) {
      for (std::size_t k = 0; k < e.tiles.size(); ++k) {
        if (!outTiles.empty() && k == 0) continue; // avoid duplicate node tile
        outTiles.push_back(e.tiles[k]);
      }
    } else {
      for (std::size_t k = 0; k < e.tiles.size(); ++k) {
        const std::size_t rk = e.tiles.size() - 1 - k;
        if (!outTiles.empty() && k == 0) continue;
        outTiles.push_back(e.tiles[rk]);
      }
    }
  }

  return !outTiles.empty();
}


bool BuildTransitLineStopTiles(const RoadGraph& g, const TransitLine& line, int stopSpacingTiles,
                               std::vector<Point>& outStops)
{
  outStops.clear();
  std::vector<Point> tiles;
  if (!BuildTransitLineTilePolyline(g, line, tiles)) return false;
  if (tiles.empty()) return false;

  const int spacing = std::max(1, stopSpacingTiles);

  outStops.push_back(tiles.front());

  int lastIdx = 0;
  // Always keep the final tile as a stop; only iterate interior tiles here.
  for (int i = 1; i + 1 < static_cast<int>(tiles.size()); ++i) {
    if (i - lastIdx >= spacing) {
      const Point p = tiles[static_cast<std::size_t>(i)];
      // Avoid duplicates in degenerate cases.
      if (outStops.empty() || outStops.back().x != p.x || outStops.back().y != p.y) {
        outStops.push_back(p);
      }
      lastIdx = i;
    }
  }

  const Point last = tiles.back();
  if (outStops.empty() || outStops.back().x != last.x || outStops.back().y != last.y) {
    outStops.push_back(last);
  }

  return !outStops.empty();
}

} // namespace isocity
