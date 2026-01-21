#include "isocity/RoadGraphResilience.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <queue>
#include <vector>

namespace isocity {

namespace {

inline int Other(int u, const RoadGraphEdge& e)
{
  return (e.a == u) ? e.b : e.a;
}

} // namespace

RoadGraphResilienceResult ComputeRoadGraphResilience(const RoadGraph& g)
{
  RoadGraphResilienceResult out;

  const int n = static_cast<int>(g.nodes.size());
  const int m = static_cast<int>(g.edges.size());

  out.isArticulationNode.assign(static_cast<std::size_t>(n), 0);
  out.isBridgeEdge.assign(static_cast<std::size_t>(m), 0);
  out.bridgeSubtreeNodes.assign(static_cast<std::size_t>(m), 0);
  out.bridgeOtherNodes.assign(static_cast<std::size_t>(m), 0);

  out.nodeComponent.assign(static_cast<std::size_t>(n), -1);
  out.componentSize.clear();

  if (n == 0) return out;

  std::vector<int> disc(static_cast<std::size_t>(n), -1);
  std::vector<int> low(static_cast<std::size_t>(n), -1);
  std::vector<int> parent(static_cast<std::size_t>(n), -1);
  std::vector<int> parentEdge(static_cast<std::size_t>(n), -1);
  std::vector<int> childCount(static_cast<std::size_t>(n), 0);
  std::vector<int> subtreeSize(static_cast<std::size_t>(n), 0);

  int time = 0;
  int compId = 0;

  struct Frame {
    int u = -1;
    std::size_t it = 0;
  };

  for (int root = 0; root < n; ++root) {
    if (disc[static_cast<std::size_t>(root)] != -1) continue;

    out.componentSize.push_back(0);

    // Initialize root.
    disc[static_cast<std::size_t>(root)] = time;
    low[static_cast<std::size_t>(root)] = time;
    ++time;

    parent[static_cast<std::size_t>(root)] = -1;
    parentEdge[static_cast<std::size_t>(root)] = -1;
    childCount[static_cast<std::size_t>(root)] = 0;
    subtreeSize[static_cast<std::size_t>(root)] = 1;

    out.nodeComponent[static_cast<std::size_t>(root)] = compId;
    out.componentSize[static_cast<std::size_t>(compId)] += 1;

    std::vector<Frame> st;
    st.push_back(Frame{root, 0});

    while (!st.empty()) {
      Frame& f = st.back();
      const int u = f.u;

      if (u < 0 || u >= n) {
        st.pop_back();
        continue;
      }

      const std::vector<int>& adj = g.nodes[static_cast<std::size_t>(u)].edges;

      if (f.it < adj.size()) {
        const int ei = adj[f.it++];
        if (ei < 0 || ei >= m) continue;

        const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
        const int v = Other(u, e);
        if (v < 0 || v >= n) continue;

        if (disc[static_cast<std::size_t>(v)] == -1) {
          // Tree edge.
          parent[static_cast<std::size_t>(v)] = u;
          parentEdge[static_cast<std::size_t>(v)] = ei;
          childCount[static_cast<std::size_t>(u)] += 1;

          disc[static_cast<std::size_t>(v)] = time;
          low[static_cast<std::size_t>(v)] = time;
          ++time;

          subtreeSize[static_cast<std::size_t>(v)] = 1;

          out.nodeComponent[static_cast<std::size_t>(v)] = compId;
          out.componentSize[static_cast<std::size_t>(compId)] += 1;

          st.push_back(Frame{v, 0});
        } else if (ei != parentEdge[static_cast<std::size_t>(u)]) {
          // Back edge (skip only the exact parent edge so parallel edges are handled correctly).
          const int dv = disc[static_cast<std::size_t>(v)];
          low[static_cast<std::size_t>(u)] = std::min(low[static_cast<std::size_t>(u)], dv);
        }
      } else {
        // Finished u: propagate to parent and run bridge/articulation checks.
        st.pop_back();

        const int p = parent[static_cast<std::size_t>(u)];
        if (p != -1) {
          subtreeSize[static_cast<std::size_t>(p)] += subtreeSize[static_cast<std::size_t>(u)];

          low[static_cast<std::size_t>(p)] =
              std::min(low[static_cast<std::size_t>(p)], low[static_cast<std::size_t>(u)]);

          // Bridge check for edge parentEdge[u].
          const int pe = parentEdge[static_cast<std::size_t>(u)];
          if (pe >= 0 && pe < m) {
            if (low[static_cast<std::size_t>(u)] > disc[static_cast<std::size_t>(p)]) {
              out.isBridgeEdge[static_cast<std::size_t>(pe)] = 1;
              out.bridgeSubtreeNodes[static_cast<std::size_t>(pe)] = subtreeSize[static_cast<std::size_t>(u)];
            }
          }

          // Articulation check for parent p (non-root).
          const int pp = parent[static_cast<std::size_t>(p)];
          if (pp != -1) {
            if (low[static_cast<std::size_t>(u)] >= disc[static_cast<std::size_t>(p)]) {
              out.isArticulationNode[static_cast<std::size_t>(p)] = 1;
            }
          }
        } else {
          // Root articulation rule.
          if (childCount[static_cast<std::size_t>(u)] > 1) {
            out.isArticulationNode[static_cast<std::size_t>(u)] = 1;
          }
        }
      }
    }

    ++compId;
  }

  // Fill bridgeOtherNodes now that component sizes are known.
  for (int ei = 0; ei < m; ++ei) {
    if (!out.isBridgeEdge[static_cast<std::size_t>(ei)]) continue;
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    if (e.a < 0 || e.a >= n) continue;
    const int cid = out.nodeComponent[static_cast<std::size_t>(e.a)];
    if (cid < 0 || cid >= static_cast<int>(out.componentSize.size())) continue;
    const int compN = out.componentSize[static_cast<std::size_t>(cid)];
    const int subN = out.bridgeSubtreeNodes[static_cast<std::size_t>(ei)];
    out.bridgeOtherNodes[static_cast<std::size_t>(ei)] = std::max(0, compN - subN);
  }

  // Build convenience lists.
  for (int i = 0; i < n; ++i) {
    if (out.isArticulationNode[static_cast<std::size_t>(i)]) out.articulationNodes.push_back(i);
  }
  for (int i = 0; i < m; ++i) {
    if (out.isBridgeEdge[static_cast<std::size_t>(i)]) out.bridgeEdges.push_back(i);
  }

  return out;
}

bool ComputeRoadGraphBridgeCut(const RoadGraph& g, int edgeIndex, RoadGraphBridgeCut& out)
{
  out.sideA.clear();
  out.sideB.clear();

  const int n = static_cast<int>(g.nodes.size());
  const int m = static_cast<int>(g.edges.size());
  if (edgeIndex < 0 || edgeIndex >= m) return false;
  if (n == 0) return false;

  const RoadGraphEdge& be = g.edges[static_cast<std::size_t>(edgeIndex)];
  if (be.a < 0 || be.a >= n || be.b < 0 || be.b >= n) return false;

  auto bfs = [&](int start, std::vector<std::uint8_t>& vis) {
    std::queue<int> q;
    q.push(start);
    vis[static_cast<std::size_t>(start)] = 1;

    while (!q.empty()) {
      const int u = q.front();
      q.pop();

      const std::vector<int>& adj = g.nodes[static_cast<std::size_t>(u)].edges;
      for (const int ei : adj) {
        if (ei == edgeIndex) continue;
        if (ei < 0 || ei >= m) continue;
        const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
        const int v = Other(u, e);
        if (v < 0 || v >= n) continue;
        if (vis[static_cast<std::size_t>(v)]) continue;
        vis[static_cast<std::size_t>(v)] = 1;
        q.push(v);
      }
    }
  };

  std::vector<std::uint8_t> visA(static_cast<std::size_t>(n), std::uint8_t{0});
  bfs(be.a, visA);

  // If we can still reach the other endpoint, then this edge doesn't induce a cut.
  if (visA[static_cast<std::size_t>(be.b)]) return false;

  std::vector<std::uint8_t> visB(static_cast<std::size_t>(n), std::uint8_t{0});
  bfs(be.b, visB);

  for (int i = 0; i < n; ++i) {
    if (visA[static_cast<std::size_t>(i)]) out.sideA.push_back(i);
    if (visB[static_cast<std::size_t>(i)]) out.sideB.push_back(i);
  }

  return (!out.sideA.empty() && !out.sideB.empty());
}

std::vector<std::uint64_t> BuildBlockedMovesForRoadGraphEdge(const RoadGraph& g, int edgeIndex, int worldWidth)
{
  std::vector<std::uint64_t> blocked;
  const int m = static_cast<int>(g.edges.size());
  if (edgeIndex < 0 || edgeIndex >= m) return blocked;
  if (worldWidth <= 0) return blocked;

  const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(edgeIndex)];
  if (e.tiles.size() < 2) return blocked;

  auto pack = [&](int fromIdx, int toIdx) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(fromIdx)) << 32) |
           static_cast<std::uint64_t>(static_cast<std::uint32_t>(toIdx));
  };

  blocked.reserve(e.tiles.size() * 2u);

  for (std::size_t i = 0; i + 1 < e.tiles.size(); ++i) {
    const Point a = e.tiles[i];
    const Point b = e.tiles[i + 1];
    const int ia = a.y * worldWidth + a.x;
    const int ib = b.y * worldWidth + b.x;
    blocked.push_back(pack(ia, ib));
    blocked.push_back(pack(ib, ia));
  }

  std::sort(blocked.begin(), blocked.end());
  blocked.erase(std::unique(blocked.begin(), blocked.end()), blocked.end());
  return blocked;
}

} // namespace isocity
