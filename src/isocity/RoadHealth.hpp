#pragma once

#include "isocity/RoadGraphCentrality.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/RoadResilienceBypass.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Road network "health" analysis
// -----------------------------------------------------------------------------
//
// This module bridges existing graph analytics (centrality + cut edges) into
// a *per-tile* representation suitable for:
//   - map layers (heatmaps)
//   - tile_metrics.csv
//   - fast headless reporting
//
// Outputs:
//   - centrality01: edge/node betweenness mapped onto road tiles (0..1)
//   - vulnerability01: bridge/cut impact + articulation markers (0..1)
//   - bypassMask: suggested bypass road-build paths (0/1)
//
// The intent is to highlight:
//   - structural bottlenecks (high betweenness) *even before* traffic simulation
//   - single points of failure (bridge edges / cut vertices)
//   - actionable resilience improvements (suggested bypasses)
//
// The implementation is deterministic and dependency-free.
// -----------------------------------------------------------------------------

struct RoadHealthConfig {
  // Which RoadGraph edge weighting to use for centrality.
  RoadGraphEdgeWeightMode weightMode = RoadGraphEdgeWeightMode::TravelTimeMilli;

  // Betweenness sampling:
  //   - maxSources == 0: auto (exact for small graphs, sampled for large)
  //   - maxSources  > 0: sampled betweenness (deterministic)
  int maxSources = 0;
  int autoExactMaxNodes = 650;
  int autoSampleSources = 256;

  // If true, also stamp node betweenness onto tiles around each node.
  bool includeNodeCentrality = true;

  // Vulnerability:
  // Articulation nodes (cut vertices) get at least this vulnerability marker.
  float articulationVulnerabilityBase = 0.70f;

  // Suggested bypass overlay:
  // When true, run the bypass planner for bridge edges and provide a tile mask.
  bool includeBypass = true;
  RoadResilienceBypassConfig bypassCfg{};
};

struct RoadHealthResult {
  int w = 0;
  int h = 0;

  RoadHealthConfig cfg{};

  // RoadGraph stats.
  int nodes = 0;
  int edges = 0;
  int sourcesUsed = 0;

  // Resilience stats.
  int bridgeEdges = 0;
  int articulationNodes = 0;

  // Per-tile fields (size = w*h).
  std::vector<float> centrality01;
  std::vector<float> vulnerability01;

  // 1 when the tile is part of a suggested bypass path.
  std::vector<std::uint8_t> bypassMask;

  // For headless/reporting use: the actual bypass suggestions.
  std::vector<RoadResilienceBypassSuggestion> bypasses;
};

// Compute a deterministic road network health analysis for a world.
//
// traffic is optional and only used for ranking bypass suggestions (when enabled).
RoadHealthResult ComputeRoadHealth(const World& world, const RoadHealthConfig& cfg = {},
                                  const TrafficResult* traffic = nullptr);

} // namespace isocity
