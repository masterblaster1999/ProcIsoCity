#include "isocity/RoadGraphRouting.hpp"

#include "isocity/Road.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace isocity {

namespace {

constexpr int kINF = std::numeric_limits<int>::max() / 4;

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool IsRoadTile(const World& world, int x, int y)
{
  if (!world.inBounds(x, y)) return false;
  return world.at(x, y).overlay == Overlay::Road;
}

inline int RoadTileTravelTimeMilli(const World& world, int x, int y)
{
  const Tile& t = world.at(x, y);
  const int lvl = static_cast<int>(t.level);
  // Roads placed on water are treated as bridges and get an extra routing penalty.
  if (t.terrain == Terrain::Water && t.overlay == Overlay::Road) {
    return RoadBridgeTravelTimeMilliForLevel(lvl);
  }
  return RoadTravelTimeMilliForLevel(lvl);
}

inline int Manhattan(const Point& a, const Point& b)
{
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

inline int MinRoadStepCostMilli()
{
  // Keep this in sync with RoadTravelTimeMilliForLevel.
  int m = RoadTravelTimeMilliForLevel(1);
  m = std::min(m, RoadTravelTimeMilliForLevel(2));
  m = std::min(m, RoadTravelTimeMilliForLevel(3));
  return m;
}

struct NodePath {
  std::vector<int> nodes; // inclusive
  std::vector<int> edges; // edges[i] connects nodes[i] -> nodes[i+1]
  int primary = kINF;
  int secondary = kINF;
  bool ok = false;
};

struct OpenNode {
  int f = 0;
  int primary = 0;
  int secondary = 0;
  int node = -1;
};

struct OpenCmp {
  bool operator()(const OpenNode& a, const OpenNode& b) const
  {
    // min-heap emulation for std::priority_queue
    if (a.f != b.f) return a.f > b.f;
    if (a.primary != b.primary) return a.primary > b.primary;
    if (a.secondary != b.secondary) return a.secondary > b.secondary;
    return a.node > b.node;
  }
};

int EdgeCostPrimary(const RoadGraph& g, const RoadGraphWeights& wts, int edgeId, int fromNode, RoadRouteMetric metric)
{
  const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(edgeId)];
  const RoadGraphEdgeWeights& ew = wts.edge[static_cast<std::size_t>(edgeId)];

  const int cost = (fromNode == e.a) ? ew.costABMilli : ew.costBAMilli;
  const int steps = ew.steps;

  return (metric == RoadRouteMetric::TravelTime) ? cost : steps;
}

int EdgeCostSecondary(const RoadGraph& g, const RoadGraphWeights& wts, int edgeId, int fromNode, RoadRouteMetric metric)
{
  const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(edgeId)];
  const RoadGraphEdgeWeights& ew = wts.edge[static_cast<std::size_t>(edgeId)];

  const int cost = (fromNode == e.a) ? ew.costABMilli : ew.costBAMilli;
  const int steps = ew.steps;

  return (metric == RoadRouteMetric::TravelTime) ? steps : cost;
}

int HeuristicPrimary(const RoadGraph& g, int node, int goal, RoadRouteMetric metric)
{
  const Point a = g.nodes[static_cast<std::size_t>(node)].pos;
  const Point b = g.nodes[static_cast<std::size_t>(goal)].pos;
  const int md = Manhattan(a, b);
  if (metric == RoadRouteMetric::Steps) return md;
  return md * MinRoadStepCostMilli();
}

NodePath FindNodePathAStar(const RoadGraph& g, const RoadGraphWeights& wts, int startNode, int goalNode,
                          RoadRouteMetric metric)
{
  NodePath out;

  const int nNodes = static_cast<int>(g.nodes.size());
  if (startNode < 0 || startNode >= nNodes) return out;
  if (goalNode < 0 || goalNode >= nNodes) return out;

  if (startNode == goalNode) {
    out.nodes = {startNode};
    out.edges.clear();
    out.primary = 0;
    out.secondary = 0;
    out.ok = true;
    return out;
  }

  std::vector<int> bestP(static_cast<std::size_t>(nNodes), kINF);
  std::vector<int> bestS(static_cast<std::size_t>(nNodes), kINF);
  std::vector<int> prevNode(static_cast<std::size_t>(nNodes), -1);
  std::vector<int> prevEdge(static_cast<std::size_t>(nNodes), -1);

  std::priority_queue<OpenNode, std::vector<OpenNode>, OpenCmp> open;

  bestP[static_cast<std::size_t>(startNode)] = 0;
  bestS[static_cast<std::size_t>(startNode)] = 0;
  open.push(OpenNode{HeuristicPrimary(g, startNode, goalNode, metric), 0, 0, startNode});

  while (!open.empty()) {
    const OpenNode cur = open.top();
    open.pop();

    const std::size_t cu = static_cast<std::size_t>(cur.node);
    if (cu >= bestP.size()) continue;
    if (cur.primary != bestP[cu] || cur.secondary != bestS[cu]) continue;

    if (cur.node == goalNode) break;

    const RoadGraphNode& gn = g.nodes[cu];
    for (const int edgeId : gn.edges) {
      if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= g.edges.size()) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(edgeId)];

      const int neigh = (cur.node == e.a) ? e.b : e.a;
      if (neigh < 0 || neigh >= nNodes) continue;

      const int dp = EdgeCostPrimary(g, wts, edgeId, cur.node, metric);
      const int ds = EdgeCostSecondary(g, wts, edgeId, cur.node, metric);

      const int np = cur.primary + dp;
      const int ns = cur.secondary + ds;

      const std::size_t nu = static_cast<std::size_t>(neigh);

      bool improve = false;
      if (np < bestP[nu]) {
        improve = true;
      } else if (np == bestP[nu] && ns < bestS[nu]) {
        improve = true;
      } else if (np == bestP[nu] && ns == bestS[nu]) {
        const int oldPrev = prevNode[nu];
        const int oldEdge = prevEdge[nu];
        if (oldPrev < 0 || cur.node < oldPrev || (cur.node == oldPrev && edgeId < oldEdge)) {
          improve = true;
        }
      }

      if (!improve) continue;

      bestP[nu] = np;
      bestS[nu] = ns;
      prevNode[nu] = cur.node;
      prevEdge[nu] = edgeId;

      const int h = HeuristicPrimary(g, neigh, goalNode, metric);
      open.push(OpenNode{np + h, np, ns, neigh});
    }
  }

  const std::size_t gu = static_cast<std::size_t>(goalNode);
  if (gu >= bestP.size() || bestP[gu] >= kINF) return out;

  // Reconstruct path.
  std::vector<int> nodesRev;
  std::vector<int> edgesRev;
  nodesRev.reserve(64);
  edgesRev.reserve(64);

  int cur = goalNode;
  nodesRev.push_back(cur);

  int guard = 0;
  while (cur != startNode && guard++ < nNodes + 8) {
    const std::size_t cu2 = static_cast<std::size_t>(cur);
    const int p = prevNode[cu2];
    const int e = prevEdge[cu2];
    if (p < 0 || e < 0) break;
    edgesRev.push_back(e);
    cur = p;
    nodesRev.push_back(cur);
  }

  if (nodesRev.empty() || nodesRev.back() != startNode) return out;

  std::reverse(nodesRev.begin(), nodesRev.end());
  std::reverse(edgesRev.begin(), edgesRev.end());

  out.nodes = std::move(nodesRev);
  out.edges = std::move(edgesRev);
  out.primary = bestP[gu];
  out.secondary = bestS[gu];
  out.ok = true;
  return out;
}

struct EndpointChoice {
  int node = -1;
  std::vector<Point> segment; // inclusive, starts at tile, ends at node tile (start choices) / goal tile (goal choices)
  int steps = 0;
  int costMilli = 0;
};

int SegmentCostMilli(const World& world, const std::vector<Point>& seg)
{
  if (seg.size() < 2) return 0;
  int c = 0;
  for (std::size_t i = 1; i < seg.size(); ++i) {
    c += RoadTileTravelTimeMilli(world, seg[i].x, seg[i].y);
  }
  return c;
}

} // namespace

RoadGraphIndex BuildRoadGraphIndex(const World& world, const RoadGraph& g)
{
  RoadGraphIndex idx;
  idx.w = world.width();
  idx.h = world.height();
  const int w = idx.w;
  const int h = idx.h;
  if (w <= 0 || h <= 0) return idx;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  idx.tileToNode.assign(n, -1);
  idx.tileToEdge.assign(n, -1);
  idx.tileToEdgeOffset.assign(n, -1);

  // Node mapping.
  for (int ni = 0; ni < static_cast<int>(g.nodes.size()); ++ni) {
    const Point p = g.nodes[static_cast<std::size_t>(ni)].pos;
    if (!world.inBounds(p.x, p.y)) continue;
    idx.tileToNode[FlatIdx(p.x, p.y, w)] = ni;
  }

  // Edge mapping for interior tiles only.
  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    if (e.tiles.size() < 3) continue;

    for (std::size_t ti = 1; ti + 1 < e.tiles.size(); ++ti) {
      const Point p = e.tiles[ti];
      if (!world.inBounds(p.x, p.y)) continue;
      const std::size_t flat = FlatIdx(p.x, p.y, w);
      if (flat >= idx.tileToEdge.size()) continue;
      idx.tileToEdge[flat] = ei;
      idx.tileToEdgeOffset[flat] = static_cast<int>(ti);
    }
  }

  return idx;
}

RoadGraphWeights BuildRoadGraphWeights(const World& world, const RoadGraph& g)
{
  RoadGraphWeights out;
  out.edge.resize(g.edges.size());

  for (std::size_t ei = 0; ei < g.edges.size(); ++ei) {
    const RoadGraphEdge& e = g.edges[ei];
    RoadGraphEdgeWeights w;
    w.steps = e.length;

    // Direction a -> b: exclude tiles[0] (start), include tiles.back() (dest).
    int cab = 0;
    for (std::size_t i = 1; i < e.tiles.size(); ++i) {
      const Point p = e.tiles[i];
      if (!world.inBounds(p.x, p.y)) continue;
      cab += RoadTileTravelTimeMilli(world, p.x, p.y);
    }

    // Direction b -> a: exclude tiles.back() (start), include tiles[0] (dest).
    int cba = 0;
    if (e.tiles.size() >= 2) {
      for (std::size_t i = 0; i + 1 < e.tiles.size(); ++i) {
        const Point p = e.tiles[i];
        if (!world.inBounds(p.x, p.y)) continue;
        cba += RoadTileTravelTimeMilli(world, p.x, p.y);
      }
    }

    w.costABMilli = cab;
    w.costBAMilli = cba;

    out.edge[ei] = w;
  }

  return out;
}

RoadRouteResult FindRoadRouteAStar(const World& world, const RoadGraph& g, const RoadGraphIndex& idx,
                                  const RoadGraphWeights& wts, Point start, Point goal, const RoadRouteConfig& cfg)
{
  RoadRouteResult out;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;
  if (!world.inBounds(start.x, start.y) || !world.inBounds(goal.x, goal.y)) return out;
  if (!IsRoadTile(world, start.x, start.y) || !IsRoadTile(world, goal.x, goal.y)) return out;
  if (g.nodes.empty()) return out;

  if (start.x == goal.x && start.y == goal.y) {
    out.path = {start};
    out.steps = 0;
    out.costMilli = 0;
    return out;
  }

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (idx.w != w || idx.h != h) return out;
  if (idx.tileToNode.size() != n || idx.tileToEdge.size() != n || idx.tileToEdgeOffset.size() != n) return out;
  if (wts.edge.size() != g.edges.size()) return out;

  const std::size_t sFlat = FlatIdx(start.x, start.y, w);
  const std::size_t gFlat = FlatIdx(goal.x, goal.y, w);

  const int sNode = idx.tileToNode[sFlat];
  const int gNode = idx.tileToNode[gFlat];

  const int sEdge = (sNode >= 0) ? -1 : idx.tileToEdge[sFlat];
  const int gEdge = (gNode >= 0) ? -1 : idx.tileToEdge[gFlat];
  const int sOff = (sEdge >= 0) ? idx.tileToEdgeOffset[sFlat] : -1;
  const int gOff = (gEdge >= 0) ? idx.tileToEdgeOffset[gFlat] : -1;

  // Fast path: start and goal are interior tiles on the same edge.
  if (sEdge >= 0 && sEdge == gEdge && sOff >= 0 && gOff >= 0) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(sEdge)];

    std::vector<Point> path;
    path.reserve(static_cast<std::size_t>(std::abs(sOff - gOff)) + 2u);

    if (sOff <= gOff) {
      for (int i = sOff; i <= gOff; ++i) {
        if (i < 0 || static_cast<std::size_t>(i) >= e.tiles.size()) break;
        path.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
    } else {
      for (int i = sOff; i >= gOff; --i) {
        if (i < 0 || static_cast<std::size_t>(i) >= e.tiles.size()) break;
        path.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
    }

    if (path.size() >= 2) {
      out.path = std::move(path);
      out.steps = static_cast<int>(out.path.size()) - 1;
      out.costMilli = SegmentCostMilli(world, out.path);
    }

    return out;
  }

  // Build endpoint choices for start.
  std::vector<EndpointChoice> startChoices;
  if (sNode >= 0) {
    EndpointChoice c;
    c.node = sNode;
    c.segment = {start};
    c.steps = 0;
    c.costMilli = 0;
    startChoices.push_back(std::move(c));
  } else if (sEdge >= 0 && sOff >= 0) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(sEdge)];
    const int last = static_cast<int>(e.tiles.size()) - 1;

    // Toward node a (tiles[0]).
    {
      EndpointChoice c;
      c.node = e.a;
      c.segment.reserve(static_cast<std::size_t>(sOff) + 2u);
      for (int i = sOff; i >= 0; --i) {
        c.segment.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
      c.steps = static_cast<int>(c.segment.size()) - 1;
      c.costMilli = SegmentCostMilli(world, c.segment);
      startChoices.push_back(std::move(c));
    }

    // Toward node b (tiles[last]).
    {
      EndpointChoice c;
      c.node = e.b;
      c.segment.reserve(static_cast<std::size_t>(last - sOff) + 2u);
      for (int i = sOff; i <= last; ++i) {
        c.segment.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
      c.steps = static_cast<int>(c.segment.size()) - 1;
      c.costMilli = SegmentCostMilli(world, c.segment);
      startChoices.push_back(std::move(c));
    }
  }

  // Build endpoint choices for goal.
  std::vector<EndpointChoice> goalChoices;
  if (gNode >= 0) {
    EndpointChoice c;
    c.node = gNode;
    c.segment = {goal};
    c.steps = 0;
    c.costMilli = 0;
    goalChoices.push_back(std::move(c));
  } else if (gEdge >= 0 && gOff >= 0) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(gEdge)];
    const int last = static_cast<int>(e.tiles.size()) - 1;

    // From node a (tiles[0]) toward goal at offset gOff.
    {
      EndpointChoice c;
      c.node = e.a;
      c.segment.reserve(static_cast<std::size_t>(gOff) + 2u);
      for (int i = 0; i <= gOff; ++i) {
        c.segment.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
      c.steps = static_cast<int>(c.segment.size()) - 1;
      c.costMilli = SegmentCostMilli(world, c.segment);
      goalChoices.push_back(std::move(c));
    }

    // From node b (tiles[last]) toward goal (reverse).
    {
      EndpointChoice c;
      c.node = e.b;
      c.segment.reserve(static_cast<std::size_t>(last - gOff) + 2u);
      for (int i = last; i >= gOff; --i) {
        c.segment.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
      c.steps = static_cast<int>(c.segment.size()) - 1;
      c.costMilli = SegmentCostMilli(world, c.segment);
      goalChoices.push_back(std::move(c));
    }
  }

  if (startChoices.empty() || goalChoices.empty()) return out;

  // Evaluate all endpoint-pair combos (<=4) and pick the best.
  const RoadRouteMetric metric = cfg.metric;

  int bestPrimary = kINF;
  int bestSecondary = kINF;
  int bestStart = -1;
  int bestGoal = -1;
  NodePath bestNodePath;

  for (int si = 0; si < static_cast<int>(startChoices.size()); ++si) {
    const EndpointChoice& sc = startChoices[static_cast<std::size_t>(si)];

    for (int gi = 0; gi < static_cast<int>(goalChoices.size()); ++gi) {
      const EndpointChoice& gc = goalChoices[static_cast<std::size_t>(gi)];

      const int scP = (metric == RoadRouteMetric::TravelTime) ? sc.costMilli : sc.steps;
      const int scS = (metric == RoadRouteMetric::TravelTime) ? sc.steps : sc.costMilli;
      const int gcP = (metric == RoadRouteMetric::TravelTime) ? gc.costMilli : gc.steps;
      const int gcS = (metric == RoadRouteMetric::TravelTime) ? gc.steps : gc.costMilli;

      NodePath np = FindNodePathAStar(g, wts, sc.node, gc.node, metric);
      if (!np.ok) continue;

      const int totalP = scP + np.primary + gcP;
      const int totalS = scS + np.secondary + gcS;

      bool improve = false;
      if (totalP < bestPrimary) {
        improve = true;
      } else if (totalP == bestPrimary && totalS < bestSecondary) {
        improve = true;
      } else if (totalP == bestPrimary && totalS == bestSecondary) {
        if (bestStart < 0 || si < bestStart || (si == bestStart && gi < bestGoal)) {
          improve = true;
        }
      }

      if (!improve) continue;

      bestPrimary = totalP;
      bestSecondary = totalS;
      bestStart = si;
      bestGoal = gi;
      bestNodePath = std::move(np);
    }
  }

  if (bestStart < 0 || bestGoal < 0 || !bestNodePath.ok) return out;

  const EndpointChoice& sc = startChoices[static_cast<std::size_t>(bestStart)];
  const EndpointChoice& gc = goalChoices[static_cast<std::size_t>(bestGoal)];

  // Assemble tile path: startSegment + nodeEdges + goalSegment.
  std::vector<Point> path;
  path.reserve(256);

  path.insert(path.end(), sc.segment.begin(), sc.segment.end());

  int curNode = sc.node;
  for (const int edgeId : bestNodePath.edges) {
    if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= g.edges.size()) break;
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(edgeId)];

    if (curNode != e.a && curNode != e.b) break;
    const bool forward = (curNode == e.a);
    const int nextNode = forward ? e.b : e.a;

    if (forward) {
      // Edge tiles are stored from a->b. Skip the first tile to avoid duplication.
      for (std::size_t i = 1; i < e.tiles.size(); ++i) {
        path.push_back(e.tiles[i]);
      }
    } else {
      // Reverse traversal: edge tiles are stored a->b, but we need b->a.
      // Skip tiles.back() (the current node tile) to avoid duplication.
      for (int i = static_cast<int>(e.tiles.size()) - 2; i >= 0; --i) {
        path.push_back(e.tiles[static_cast<std::size_t>(i)]);
      }
    }

    curNode = nextNode;
  }

  // The reverse edge assembly above is intentionally conservative, but we may have introduced duplicates.
  // Normalize: remove consecutive duplicates.
  if (!path.empty()) {
    std::size_t wpos = 1;
    for (std::size_t r = 1; r < path.size(); ++r) {
      if (path[r].x == path[wpos - 1].x && path[r].y == path[wpos - 1].y) continue;
      path[wpos++] = path[r];
    }
    path.resize(wpos);
  }

  // Append goal segment, skipping its first tile (which is already included as the goal node).
  if (!gc.segment.empty()) {
    for (std::size_t i = 1; i < gc.segment.size(); ++i) {
      path.push_back(gc.segment[i]);
    }
  }

  // Validate endpoints.
  if (path.size() < 2) return out;
  if (path.front().x != start.x || path.front().y != start.y) return out;
  if (path.back().x != goal.x || path.back().y != goal.y) return out;

  out.path = std::move(path);
  out.steps = static_cast<int>(out.path.size()) - 1;
  out.costMilli = SegmentCostMilli(world, out.path);

  return out;
}

} // namespace isocity