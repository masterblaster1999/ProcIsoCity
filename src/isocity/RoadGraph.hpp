#pragma once

#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <vector>

namespace isocity {

// A lightweight graph extracted from the road tile grid.
//
// Nodes are road tiles that are:
//  - intersections / endpoints (degree != 2), OR
//  - corners (degree == 2 but not straight).
//
// Edges connect nodes by following straight road segments through degree-2 straight tiles.
// This is a useful building block for future systems (traffic, goods flow, routing, etc.)
// and for debug visualization.
struct RoadGraphEdge {
  int a = -1;
  int b = -1;

  // Number of steps between nodes (Manhattan edges), i.e. tiles.size()-1.
  int length = 0;

  // Tile coordinates along the edge, inclusive of endpoints.
  // Note: order is deterministic but not guaranteed to be (a -> b); it depends on build order.
  std::vector<Point> tiles;
};

struct RoadGraphNode {
  Point pos{};
  std::vector<int> edges; // indices into RoadGraph::edges
};

struct RoadGraph {
  std::vector<RoadGraphNode> nodes;
  std::vector<RoadGraphEdge> edges;
};

// Build a compressed road graph from the current world's road tiles.
RoadGraph BuildRoadGraph(const World& world);

} // namespace isocity
