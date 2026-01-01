#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Centrality metrics for the compressed RoadGraph.
//
// This module is intended for analysis and tooling:
//  - identifying structural "bottlenecks" (high betweenness) even before traffic is simulated
//  - spotting overly tree-like procedural road layouts
//  - producing deterministic regression artifacts for CI (JSON/CSV/DOT)
//
// Implementation notes:
//  - Node/edge betweenness uses a deterministic variant of Brandes' algorithm.
//  - Edge weights can be either edge-length (tile steps) or travel-time milli-steps derived
//    from road class (Street/Avenue/Highway) along the underlying tile polyline.

enum class RoadGraphEdgeWeightMode : std::uint8_t {
  Steps = 0,          // weight = RoadGraphEdge::length
  TravelTimeMilli = 1 // weight = sum of per-tile RoadTravelTimeMilliForLevel along the edge
};

struct RoadGraphCentralityConfig {
  RoadGraphEdgeWeightMode weightMode = RoadGraphEdgeWeightMode::Steps;

  // Limit the number of source nodes processed.
  //  - 0 => all nodes (exact)
  //  - N>0 => sample N sources deterministically and scale the result to approximate full-graph centrality
  int maxSources = 0;

  // If true and maxSources is active, scale the sampled result by (N / sourcesUsed).
  bool scaleSampleToFull = true;

  // If true, divide betweenness values by 2 (the standard correction for undirected graphs).
  bool undirected = true;

  // Compute normalized 0..1 betweenness values.
  bool normalizeBetweenness = true;

  // Closeness options.
  // When true, scale closeness by (reachable-1)/(N-1) so nodes in small disconnected components
  // get lower closeness values.
  bool closenessComponentScale = true;
};

struct RoadGraphCentralityResult {
  int nodes = 0;
  int edges = 0;
  int sourcesUsed = 0;

  // Per-node betweenness (raw) and optional normalized values.
  std::vector<double> nodeBetweenness;
  std::vector<double> nodeBetweennessNorm;

  // Per-edge betweenness (raw) and optional normalized values.
  std::vector<double> edgeBetweenness;
  std::vector<double> edgeBetweennessNorm;

  // Closeness variants (computed using the same edge weights as betweenness):
  //  - closeness: (reachable-1) / sum(dist)
  //  - harmonic: sum(1/dist)
  std::vector<double> nodeCloseness;
  std::vector<double> nodeHarmonicCloseness;
};

// Compute centrality metrics for a RoadGraph.
//
// If cfg.weightMode == TravelTimeMilli, you should pass worldForWeights so the function can derive
// travel-time weights from each edge's tile polyline.
RoadGraphCentralityResult ComputeRoadGraphCentrality(const RoadGraph& g, const RoadGraphCentralityConfig& cfg = {},
                                                    const World* worldForWeights = nullptr);

} // namespace isocity
