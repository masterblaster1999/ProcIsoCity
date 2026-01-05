#pragma once

#include "isocity/World.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace isocity {

// Configuration for building a road flow field.
struct RoadFlowFieldConfig {
  // If true, road traversal is restricted to road tiles connected to the map edge.
  // (This matches the "outside connection" rule used by the simulator.)
  bool requireOutsideConnection = false;

  // If true, compute an `owner` label for each reachable road tile indicating
  // which source claimed it (0..sources-1). This is useful for "nearest producer"
  // style logic.
  bool computeOwner = false;

  // If true, build the field using travel-time weights derived from road class.
  // This runs a deterministic multi-source Dijkstra on the road grid.
  //
  // dist still reports step-count (edges) along the chosen route, but routing/ownership is
  // based on travel-time (see RoadFlowField::cost).
  bool useTravelTime = false;
};

// Result of a road-network multi-source search.
//
// dist[idx]   = road steps (edges) along the chosen (cost-optimal) path from idx to a source, or -1 if unreachable.
// cost[idx]   = travel-time cost to a source in milli-steps (street step == 1000), or -1 if unreachable.
// parent[idx] = the next road tile index on the chosen path from idx towards a source.
//               parent[source] == -1.
// owner[idx]  = (optional) which source index claimed this tile. Empty if not requested.
struct RoadFlowField {
  int w = 0;
  int h = 0;

  std::vector<int> dist;
  std::vector<int> cost;
  std::vector<int> parent;
  std::vector<int> owner;

  bool empty() const { return w <= 0 || h <= 0; }
  std::size_t size() const { return dist.size(); }
};

// Build a deterministic road flow field.
//
// - `sourceRoadIdx` are linear indices (y*w + x) of road tiles that act as sources.
// - If `cfg.requireOutsideConnection` is true, traversal is limited to roads that are
//   connected to the map edge.
// - If a usable precomputed outside-connection mask is provided, it will be reused.
// - If `extraCostMilli` is provided (and has size w*h), its values are treated as an
//   additional per-road-tile travel-time penalty (in milli-steps) applied when entering
//   that road tile. This is useful for congestion-aware routing.
// - If `roadBlockMask` is provided (and has size w*h), any road tile with mask!=0 is treated
//   as non-traversable. This is useful for scenario analysis (closures, flooding, construction).
// - If `sourceInitialCostMilli` is provided (and has size sourceRoadIdx.size()), its values are
//   treated as an initial per-source travel-time offset (in milli-steps) added to the total
//   cost of any route terminating at that source. This is useful for *soft* capacity constraints
//   (e.g., pushing flow away from overloaded destinations) while keeping routing deterministic.
RoadFlowField BuildRoadFlowField(const World& world, const std::vector<int>& sourceRoadIdx,
                                 const RoadFlowFieldConfig& cfg = {},
                                 const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr,
                                 const std::vector<int>* extraCostMilli = nullptr,
                                 const std::vector<std::uint8_t>* roadBlockMask = nullptr,
                                 const std::vector<int>* sourceInitialCostMilli = nullptr);

} // namespace isocity
