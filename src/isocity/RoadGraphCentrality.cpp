#include "isocity/RoadGraphCentrality.hpp"

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

constexpr int kInf = std::numeric_limits<int>::max() / 4;

// Deterministic 64-bit mixing (splitmix64).
// Used only for stable source sampling.
std::uint64_t SplitMix64(std::uint64_t x)
{
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

struct Adj {
  int to = -1;
  int edge = -1;
  int w = 1;
};

struct Pred {
  int v = -1;
  int edge = -1;
};

std::vector<int> ComputeEdgeWeights(const RoadGraph& g, RoadGraphEdgeWeightMode mode, const World* world)
{
  std::vector<int> out;
  out.resize(g.edges.size(), 1);

  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];

    int w = 1;
    if (mode == RoadGraphEdgeWeightMode::Steps) {
      w = std::max(1, e.length);
    } else {
      // Travel time: sum per-tile milli-cost along the edge polyline.
      // If the world isn't available, fall back to a street-weight approximation.
      if (!world) {
        w = std::max(1, e.length * 1000);
      } else {
        std::int64_t sum = 0;
        if (e.tiles.size() >= 2) {
          for (std::size_t k = 1; k < e.tiles.size(); ++k) {
            const Point& p = e.tiles[k];
            if (!world->inBounds(p.x, p.y)) continue;
            const Tile& t = world->at(p.x, p.y);
            if (t.overlay != Overlay::Road) continue;
            const int level = static_cast<int>(t.level);
            const int cost = (t.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(level)
                                                           : RoadTravelTimeMilliForLevel(level);
            sum += static_cast<std::int64_t>(std::max(1, cost));
          }
        }
        if (sum <= 0) sum = static_cast<std::int64_t>(std::max(1, e.length)) * 1000;
        if (sum > static_cast<std::int64_t>(kInf - 1)) sum = static_cast<std::int64_t>(kInf - 1);
        w = static_cast<int>(sum);
      }
    }

    out[static_cast<std::size_t>(ei)] = std::max(1, w);
  }

  return out;
}

std::vector<int> PickSourcesDeterministic(int n, int maxSources)
{
  std::vector<int> sources;
  if (n <= 0) return sources;

  if (maxSources <= 0 || maxSources >= n) {
    sources.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) sources[static_cast<std::size_t>(i)] = i;
    return sources;
  }

  struct Key {
    std::uint64_t h = 0;
    int id = 0;
  };

  std::vector<Key> keys;
  keys.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    // The constants are arbitrary; this just needs to be deterministic.
    const std::uint64_t h = SplitMix64(static_cast<std::uint64_t>(i) * 0xD1B54A32D192ED03ULL);
    keys.push_back(Key{h, i});
  }

  std::sort(keys.begin(), keys.end(), [](const Key& a, const Key& b) {
    if (a.h != b.h) return a.h < b.h;
    return a.id < b.id;
  });

  sources.reserve(static_cast<std::size_t>(maxSources));
  for (int i = 0; i < maxSources; ++i) {
    sources.push_back(keys[static_cast<std::size_t>(i)].id);
  }

  // Keep processing order stable (not required for correctness, but makes debugging nicer).
  std::sort(sources.begin(), sources.end());
  return sources;
}

} // namespace

RoadGraphCentralityResult ComputeRoadGraphCentrality(const RoadGraph& g, const RoadGraphCentralityConfig& cfg,
                                                    const World* worldForWeights)
{
  RoadGraphCentralityResult out;
  out.nodes = static_cast<int>(g.nodes.size());
  out.edges = static_cast<int>(g.edges.size());

  const int n = out.nodes;
  const int m = out.edges;
  if (n <= 0 || m <= 0) return out;

  out.nodeBetweenness.assign(static_cast<std::size_t>(n), 0.0);
  out.edgeBetweenness.assign(static_cast<std::size_t>(m), 0.0);

  // Only compute closeness when we process all sources (exact all-pairs distances).
  // If the caller enables sampling, they likely care primarily about betweenness.
  const bool computeCloseness = (cfg.maxSources <= 0 || cfg.maxSources >= n);
  if (computeCloseness) {
    out.nodeCloseness.assign(static_cast<std::size_t>(n), 0.0);
    out.nodeHarmonicCloseness.assign(static_cast<std::size_t>(n), 0.0);
  }

  // Build adjacency.
  const std::vector<int> edgeW = ComputeEdgeWeights(g, cfg.weightMode, worldForWeights);
  std::vector<std::vector<Adj>> adj(static_cast<std::size_t>(n));
  for (int u = 0; u < n; ++u) {
    const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(u)];
    std::vector<Adj>& a = adj[static_cast<std::size_t>(u)];
    a.reserve(node.edges.size());
    for (int ei : node.edges) {
      if (ei < 0 || ei >= m) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
      const int v = (e.a == u) ? e.b : ((e.b == u) ? e.a : -1);
      if (v < 0 || v >= n) continue;
      const int w = edgeW[static_cast<std::size_t>(ei)];
      a.push_back(Adj{v, ei, std::max(1, w)});
    }

    // Deterministic traversal order helps keep results stable across platforms.
    std::sort(a.begin(), a.end(), [](const Adj& lhs, const Adj& rhs) {
      if (lhs.to != rhs.to) return lhs.to < rhs.to;
      if (lhs.edge != rhs.edge) return lhs.edge < rhs.edge;
      return lhs.w < rhs.w;
    });
  }

  // Choose sources.
  const std::vector<int> sources = PickSourcesDeterministic(n, cfg.maxSources);
  out.sourcesUsed = static_cast<int>(sources.size());

  // Scratch buffers.
  std::vector<int> dist(static_cast<std::size_t>(n), kInf);
  std::vector<double> sigma(static_cast<std::size_t>(n), 0.0);
  std::vector<double> delta(static_cast<std::size_t>(n), 0.0);
  std::vector<std::vector<Pred>> P(static_cast<std::size_t>(n));
  std::vector<int> S;
  S.reserve(static_cast<std::size_t>(n));

  struct Item {
    int d = 0;
    int v = 0;
  };

  const auto cmp = [](const Item& a, const Item& b) {
    if (a.d != b.d) return a.d > b.d; // min-heap by distance
    return a.v > b.v;
  };
  std::priority_queue<Item, std::vector<Item>, decltype(cmp)> pq(cmp);

  for (int s : sources) {
    if (s < 0 || s >= n) continue;

    // Reset scratch.
    for (int i = 0; i < n; ++i) {
      dist[static_cast<std::size_t>(i)] = kInf;
      sigma[static_cast<std::size_t>(i)] = 0.0;
      delta[static_cast<std::size_t>(i)] = 0.0;
      P[static_cast<std::size_t>(i)].clear();
    }
    while (!pq.empty()) pq.pop();
    S.clear();

    dist[static_cast<std::size_t>(s)] = 0;
    sigma[static_cast<std::size_t>(s)] = 1.0;
    pq.push(Item{0, s});

    // Multi-source single-source shortest paths (Dijkstra; weights are positive ints).
    while (!pq.empty()) {
      const Item it = pq.top();
      pq.pop();
      const int v = it.v;
      const int dv = it.d;
      if (v < 0 || v >= n) continue;
      if (dv != dist[static_cast<std::size_t>(v)]) continue;

      S.push_back(v);

      for (const Adj& e : adj[static_cast<std::size_t>(v)]) {
        const int w = e.to;
        const int nd = (dv >= kInf - e.w) ? kInf : (dv + e.w);
        if (nd < dist[static_cast<std::size_t>(w)]) {
          dist[static_cast<std::size_t>(w)] = nd;
          pq.push(Item{nd, w});

          sigma[static_cast<std::size_t>(w)] = sigma[static_cast<std::size_t>(v)];
          P[static_cast<std::size_t>(w)].clear();
          P[static_cast<std::size_t>(w)].push_back(Pred{v, e.edge});
        } else if (nd == dist[static_cast<std::size_t>(w)]) {
          sigma[static_cast<std::size_t>(w)] += sigma[static_cast<std::size_t>(v)];
          P[static_cast<std::size_t>(w)].push_back(Pred{v, e.edge});
        }
      }
    }

    // Closeness variants for this source (only when we run all sources).
    if (computeCloseness) {
      double sumDist = 0.0;
      double sumInv = 0.0;
      int reachable = 0;
      for (int i = 0; i < n; ++i) {
        const int d = dist[static_cast<std::size_t>(i)];
        if (d >= kInf) continue;
        reachable++;
        if (i == s) continue;
        sumDist += static_cast<double>(d);
        if (d > 0) sumInv += 1.0 / static_cast<double>(d);
      }

      double closeness = 0.0;
      if (reachable > 1 && sumDist > 0.0) {
        closeness = static_cast<double>(reachable - 1) / sumDist;
        if (cfg.closenessComponentScale && n > 1) {
          closeness *= static_cast<double>(reachable - 1) / static_cast<double>(n - 1);
        }
      }

      out.nodeCloseness[static_cast<std::size_t>(s)] = closeness;
      out.nodeHarmonicCloseness[static_cast<std::size_t>(s)] = sumInv;
    }

    // Accumulate dependencies in reverse distance order.
    for (int si = static_cast<int>(S.size()) - 1; si >= 0; --si) {
      const int w = S[static_cast<std::size_t>(si)];
      const double sigmaW = sigma[static_cast<std::size_t>(w)];
      if (sigmaW <= 0.0) continue;

      for (const Pred& p : P[static_cast<std::size_t>(w)]) {
        if (p.v < 0 || p.v >= n) continue;
        const double sigmaV = sigma[static_cast<std::size_t>(p.v)];
        const double c = (sigmaV / sigmaW) * (1.0 + delta[static_cast<std::size_t>(w)]);
        delta[static_cast<std::size_t>(p.v)] += c;
        if (p.edge >= 0 && p.edge < m) {
          out.edgeBetweenness[static_cast<std::size_t>(p.edge)] += c;
        }
      }

      if (w != s) {
        out.nodeBetweenness[static_cast<std::size_t>(w)] += delta[static_cast<std::size_t>(w)];
      }
    }
  }

  // Optional scaling for sampled sources.
  if (cfg.scaleSampleToFull && out.sourcesUsed > 0 && out.sourcesUsed < n) {
    const double scale = static_cast<double>(n) / static_cast<double>(out.sourcesUsed);
    for (double& v : out.nodeBetweenness) v *= scale;
    for (double& v : out.edgeBetweenness) v *= scale;
  }

  // Undirected correction: Brandes algorithm counts each pair twice when summing over all sources.
  if (cfg.undirected) {
    for (double& v : out.nodeBetweenness) v *= 0.5;
    for (double& v : out.edgeBetweenness) v *= 0.5;
  }

  // Normalized betweenness variants.
  if (cfg.normalizeBetweenness) {
    out.nodeBetweennessNorm.assign(static_cast<std::size_t>(n), 0.0);
    out.edgeBetweennessNorm.assign(static_cast<std::size_t>(m), 0.0);

    // Node normalization matches common tooling (e.g., NetworkX):
    //  - directed: 1 / ((n-1)(n-2))
    //  - undirected: 2 / ((n-1)(n-2))
    double nodeScale = 0.0;
    if (n > 2) {
      const double denom = static_cast<double>(n - 1) * static_cast<double>(n - 2);
      nodeScale = cfg.undirected ? (2.0 / denom) : (1.0 / denom);
    }

    // Edge normalization matches common tooling (e.g., NetworkX):
    //  - directed: 1 / (n(n-1))
    //  - undirected: 2 / (n(n-1))
    double edgeScale = 0.0;
    if (n > 1) {
      const double denom = static_cast<double>(n) * static_cast<double>(n - 1);
      edgeScale = cfg.undirected ? (2.0 / denom) : (1.0 / denom);
    }

    for (int i = 0; i < n; ++i) {
      out.nodeBetweennessNorm[static_cast<std::size_t>(i)] = out.nodeBetweenness[static_cast<std::size_t>(i)] * nodeScale;
    }
    for (int i = 0; i < m; ++i) {
      out.edgeBetweennessNorm[static_cast<std::size_t>(i)] = out.edgeBetweenness[static_cast<std::size_t>(i)] * edgeScale;
    }
  }

  return out;
}

} // namespace isocity
