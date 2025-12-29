#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Simple commuting / traffic model derived from the road tile grid.
//
// The goal is NOT to be a full traffic simulator yet. Instead, we compute:
//  - A plausible "commute to the nearest job" assignment over the road network
//  - A per-road-tile traffic count you can visualize as a heatmap
//  - Aggregate metrics (avg commute distance/time, congestion ratio)
//
// This provides a solid foundation for future expansions:
//  - per-edge capacities (using RoadGraph)
//  - multiple route choices / congestion-aware routing
//  - goods flow between industrial/commercial, service coverage, etc.
struct TrafficConfig {
  // Match the simulator's classic "outside connection" rule.
  // When enabled, only road components that touch the map edge are considered usable.
  bool requireOutsideConnection = true;

  // Soft capacity per road tile for the congestion metric.
  // Above this, we treat the excess as "congested" traffic.
  int roadTileCapacity = 28;

  // Include job zones as commute destinations.
  bool includeCommercialJobs = true;
  bool includeIndustrialJobs = true;

  // --- Congestion-aware routing (optional) ---
  //
  // If enabled, commute assignment is performed in multiple incremental passes.
  // Each pass rebuilds the shortest-path flow field using travel times that increase
  // with predicted traffic/capacity, which naturally spreads commuters across
  // alternate routes.
  //
  // When disabled, all commuters are assigned along the single shortest path under
  // free-flow travel times (the classic behavior).
  bool congestionAwareRouting = false;

  // Number of assignment passes (>=1). More passes => closer to an equilibrium
  // but more CPU cost.
  int congestionIterations = 4;

  // BPR-style travel time curve parameters:
  //   t = t0 * (1 + alpha * (v/c)^beta)
  // Where v is predicted commuters on the road tile, and c is capacity.
  float congestionAlpha = 0.15f;
  float congestionBeta = 4.0f;

  // Scale the capacity used for congestion costs (1.0 = use true capacity).
  // Values < 1 make congestion kick in sooner; values > 1 make it more forgiving.
  float congestionCapacityScale = 1.0f;

  // Clamp v/c to avoid extreme costs and keep the integer path costs bounded.
  // With the default (3.0) and BPR defaults, the per-tile travel time multiplier is
  // ~13x at the clamp.
  float congestionRatioClamp = 3.0f;
};

struct TrafficResult {
  // Flat array size w*h.
  // roadTraffic[idx] is the number of commuters that traverse that road tile.
  // Non-road tiles are 0.
  std::vector<std::uint16_t> roadTraffic;

  int totalCommuters = 0;
  int reachableCommuters = 0;
  int unreachableCommuters = 0;

  // Weighted by commuters, measured in road steps (edges).
  // Note: routing may use travel-time weights, so this is "steps along the chosen route",
  // not necessarily the minimum-step path.
  float avgCommute = 0.0f;
  float p95Commute = 0.0f;

  // Weighted by commuters, measured in "street-step equivalent" travel time.
  // This uses road class speeds (Street/Avenue/Highway), so faster roads reduce this value.
  float avgCommuteTime = 0.0f;
  float p95CommuteTime = 0.0f;

  // 0..1 ratio of "excess" traffic above capacity.
  float congestion = 0.0f;

  int congestedRoadTiles = 0;
  int maxTraffic = 0;

  // Debug/telemetry: which routing model was used.
  bool usedCongestionAwareRouting = false;
  int routingPasses = 1;
};

// Compute a traffic heatmap by assigning commuters to their nearest reachable job access point.
//
// employedShare controls how many residents participate in commuting:
//   - 1.0 => all residents commute (useful for debugging)
//   - employed / population => only a share commutes (matches the sim's employment)
//
// If requireOutsideConnection is true, you can optionally pass a precomputed
// road-to-edge mask (from ComputeRoadsConnectedToEdge) to avoid recomputation.
TrafficResult ComputeCommuteTraffic(const World& world, const TrafficConfig& cfg, float employedShare = 1.0f,
                                   const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr);

} // namespace isocity
