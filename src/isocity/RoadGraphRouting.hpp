#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Fast lookup tables to map road tiles to road-graph primitives.
//
// The index intentionally maps only *interior* edge tiles to an edge id/offset.
// Endpoint tiles are mapped as nodes (tileToNode) and left unmapped in tileToEdge
// to avoid ambiguity at intersections/corners.
struct RoadGraphIndex {
  int w = 0;
  int h = 0;

  std::vector<int> tileToNode;       // size w*h
  std::vector<int> tileToEdge;       // size w*h
  std::vector<int> tileToEdgeOffset; // size w*h
};

// Directional weights for each edge in the road graph.
struct RoadGraphEdgeWeights {
  int steps = 0;       // tile-steps between endpoints (== RoadGraphEdge::length)
  int costABMilli = 0; // travel-time cost from edge.a -> edge.b
  int costBAMilli = 0; // travel-time cost from edge.b -> edge.a
};

struct RoadGraphWeights {
  // Parallel to RoadGraph::edges.
  std::vector<RoadGraphEdgeWeights> edge;
};

enum class RoadRouteMetric : std::uint8_t {
  TravelTime = 0, // minimize travel time (milli-steps), tie-break by fewer steps
  Steps = 1,      // minimize step count (tiles), tie-break by lower travel time
};

struct RoadRouteConfig {
  RoadRouteMetric metric = RoadRouteMetric::TravelTime;
};

struct RoadRouteResult {
  std::vector<Point> path; // road tiles from start..goal inclusive

  int steps = -1;     // path.size()-1 (tile edges), or -1 on failure
  int costMilli = -1; // travel time cost (sum of entered tile costs), or -1 on failure
};

// Build a road-graph tile index for fast routing queries.
RoadGraphIndex BuildRoadGraphIndex(const World& world, const RoadGraph& g);

// Precompute edge traversal costs/steps for the current world + road graph.
RoadGraphWeights BuildRoadGraphWeights(const World& world, const RoadGraph& g);

// Find a road route between two road tiles using A* over the RoadGraph.
//
// Returns an empty path on failure.
RoadRouteResult FindRoadRouteAStar(const World& world, const RoadGraph& g, const RoadGraphIndex& idx,
                                  const RoadGraphWeights& wts, Point start, Point goal,
                                  const RoadRouteConfig& cfg = {});

} // namespace isocity
