#pragma once

#include "isocity/RoadGraph.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Graph-resilience / vulnerability analysis utilities for RoadGraph.
//
// This module identifies:
//  - articulation nodes (cut-vertices)
//  - bridge edges (cut-edges)
//
// These are useful for highlighting "single point of failure" segments in a road network,
// debugging outside-connection issues, and for suggesting where bypass roads would most
// improve redundancy.

struct RoadGraphResilienceResult {
  // Per-node flag: 1 => articulation node.
  std::vector<std::uint8_t> isArticulationNode;

  // Per-edge flag: 1 => bridge edge.
  std::vector<std::uint8_t> isBridgeEdge;

  // For bridge edges: size of each side if the bridge is removed (node counts).
  // These are 0 for non-bridge edges.
  //
  // Note: bridgeSubtreeNodes[e] corresponds to the DFS child-side when the bridge was
  // discovered; bridgeOtherNodes[e] is the remaining nodes in that connected component.
  std::vector<int> bridgeSubtreeNodes;
  std::vector<int> bridgeOtherNodes;

  // Connected component id for each node.
  std::vector<int> nodeComponent;

  // Size of each component (by component id).
  std::vector<int> componentSize;

  // Convenience lists.
  std::vector<int> articulationNodes;
  std::vector<int> bridgeEdges;
};

// Compute articulation points and bridges in an undirected RoadGraph.
RoadGraphResilienceResult ComputeRoadGraphResilience(const RoadGraph& g);

struct RoadGraphBridgeCut {
  std::vector<int> sideA; // nodes reachable from edge.a when the edge is removed
  std::vector<int> sideB; // nodes reachable from edge.b when the edge is removed
};

// Compute the node partition induced by removing a bridge edge.
// Returns false if edgeIndex is invalid or if endpoints remain connected without this edge.
bool ComputeRoadGraphBridgeCut(const RoadGraph& g, int edgeIndex, RoadGraphBridgeCut& out);

// Build a sorted list of blocked directed moves for an edge's tile polyline.
// This matches the key format consumed by FindRoadBuildPathBetweenSets():
//   key = ((uint64_t)fromIdx << 32) | toIdx, where idx = y*worldWidth + x.
std::vector<std::uint64_t> BuildBlockedMovesForRoadGraphEdge(const RoadGraph& g, int edgeIndex, int worldWidth);

} // namespace isocity
