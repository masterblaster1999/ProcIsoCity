#include "isocity/RoadGraph.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace isocity {

namespace {

constexpr int kDirs[4][2] = {
    {0, -1}, // N
    {1, 0},  // E
    {0, 1},  // S
    {-1, 0}, // W
};

inline bool IsRoad(const World& world, int x, int y)
{
  return world.inBounds(x, y) && world.at(x, y).overlay == Overlay::Road;
}

inline int Degree4(const World& world, int x, int y)
{
  int deg = 0;
  for (const auto& d : kDirs) {
    if (IsRoad(world, x + d[0], y + d[1])) deg++;
  }
  return deg;
}

inline bool IsStraightDegree2(const World& world, int x, int y)
{
  // Assumes tile is a road.
  const bool n = IsRoad(world, x, y - 1);
  const bool s = IsRoad(world, x, y + 1);
  const bool e = IsRoad(world, x + 1, y);
  const bool w = IsRoad(world, x - 1, y);

  // Degree 2 and opposite neighbors => straight.
  if ((n && s) && !e && !w) return true;
  if ((e && w) && !n && !s) return true;
  return false;
}

inline bool IsGraphNode(const World& world, int x, int y)
{
  if (!IsRoad(world, x, y)) return false;

  const int deg = Degree4(world, x, y);
  if (deg != 2) return true; // endpoint (0/1) or intersection (3/4)

  // Degree 2: node only if it's a corner (not straight).
  return !IsStraightDegree2(world, x, y);
}

inline std::size_t FlatIndex(int w, int x, int y)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

} // namespace

RoadGraph BuildRoadGraph(const World& world)
{
  RoadGraph g;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return g;

  // Map from tile -> node id (or -1 if not a node).
  std::vector<int> nodeId(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);

  // Pass 1: create nodes.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (!IsGraphNode(world, x, y)) continue;
      const int id = static_cast<int>(g.nodes.size());
      g.nodes.push_back(RoadGraphNode{Point{x, y}, {}});
      nodeId[FlatIndex(w, x, y)] = id;
    }
  }

  // Helper to trace from a node along a neighbor direction until we hit another node.
  auto traceToNextNode = [&](Point startNodePos, Point first, std::vector<Point>& outTiles, int& outEndId) -> bool {
    outTiles.clear();
    outEndId = -1;

    Point prev = startNodePos;
    Point cur = first;

    outTiles.push_back(startNodePos);

    // Hard cap to prevent infinite loops on malformed data.
    const int maxSteps = w * h + 8;
    for (int steps = 0; steps < maxSteps; ++steps) {
      if (!world.inBounds(cur.x, cur.y)) return false;
      if (world.at(cur.x, cur.y).overlay != Overlay::Road) return false;

      outTiles.push_back(cur);

      const int id = nodeId[FlatIndex(w, cur.x, cur.y)];
      if (id != -1) {
        outEndId = id;
        return true;
      }

      // Choose the next road neighbor (excluding prev).
      Point next{-999, -999};
      int choices = 0;
      for (const auto& d : kDirs) {
        const int nx = cur.x + d[0];
        const int ny = cur.y + d[1];
        if (!IsRoad(world, nx, ny)) continue;
        if (nx == prev.x && ny == prev.y) continue;
        next = Point{nx, ny};
        choices++;
      }

      // Degree-2 straight tiles should have exactly 1 forward choice.
      if (choices != 1) return false;

      prev = cur;
      cur = next;
    }

    return false;
  };

  // Pass 2: create edges by walking from each node in each direction.
  // To avoid duplicates, only add an edge when startId < endId.
  for (int a = 0; a < static_cast<int>(g.nodes.size()); ++a) {
    const Point p = g.nodes[static_cast<std::size_t>(a)].pos;

    for (const auto& d : kDirs) {
      const int nx = p.x + d[0];
      const int ny = p.y + d[1];
      if (!IsRoad(world, nx, ny)) continue;

      std::vector<Point> tiles;
      int b = -1;
      if (!traceToNextNode(p, Point{nx, ny}, tiles, b)) continue;
      if (b < 0 || b == a) continue;

      if (a < b) {
        RoadGraphEdge e;
        e.a = a;
        e.b = b;
        e.length = std::max(0, static_cast<int>(tiles.size()) - 1);
        e.tiles = std::move(tiles);

        const int ei = static_cast<int>(g.edges.size());
        g.edges.push_back(std::move(e));

        g.nodes[static_cast<std::size_t>(a)].edges.push_back(ei);
        g.nodes[static_cast<std::size_t>(b)].edges.push_back(ei);
      }
    }
  }

  return g;
}

} // namespace isocity
