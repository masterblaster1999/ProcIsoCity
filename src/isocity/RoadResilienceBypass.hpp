#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Road network resilience: bypass planner
// -----------------------------------------------------------------------------
//
// A RoadGraph "bridge edge" (cut-edge) is a road segment which, if removed,
// disconnects the graph into two components. In a city context this corresponds
// to "single points of failure" (eg. a bridge over water, a lone tunnel, a
// single arterial connecting a suburb).
//
// This module suggests bypasses by finding a cheapest road-build path that
// reconnects the two sides of a bridge edge *without* using the bridge segment.
//
// The path is found using the existing road-build A* (FindRoadBuildPathBetweenSets)
// over buildable tiles, with the bridge segment disallowed via blocked moves.
// -----------------------------------------------------------------------------

struct RoadResilienceBypassConfig {
  // Maximum number of bridge edges to consider (ranked by traffic if available,
  // otherwise ranked by cut size). 0 disables suggestions.
  int top = 5;

  // If true, optimize the road-build path by money cost; otherwise optimize by
  // new-tile count.
  bool moneyObjective = true;

  // Target road level for suggested bypass paths (1..3).
  int targetLevel = 1;

  // If true, allow the planner to propose bridges over water.
  bool allowBridges = false;

  // Optional cap on the planner's primary cost (0 => no limit).
  int maxPrimaryCost = 0;

  // Max number of road-graph nodes to seed on each side of the cut.
  // This keeps the multi-source search tractable on large maps.
  int maxNodesPerSide = 256;

  // If true and a TrafficResult is provided, rank bridge edges by usage
  // (max per-tile traffic over the edge) before cut size.
  bool rankByTraffic = true;
};

struct RoadResilienceBypassSuggestion {
  int bridgeEdge = -1;
  int cutSize = 0;     // min(sideA, sideB) in node count
  int primaryCost = 0; // either money or newTiles depending on plan settings
  int moneyCost = 0;   // always computed for application
  int newTiles = 0;
  int steps = 0;

  int targetLevel = 1;
  bool allowBridges = false;
  bool moneyObjective = true;

  std::vector<Point> path;
};

// Convenience helpers.
int CountNewRoadTilesInPath(const World& world, const std::vector<Point>& path);
int EstimateMoneyCostForRoadPath(const World& world, const std::vector<Point>& path, int targetLevel);

// Compute a ranked list of bypass suggestions.
//
// If `traffic` is provided and cfg.rankByTraffic==true, bridge edges are ranked
// by the maximum per-tile traffic value along the edge (ties broken by cut size).
std::vector<RoadResilienceBypassSuggestion> SuggestRoadResilienceBypasses(
    const World& world,
    const RoadGraph& roadGraph,
    const RoadGraphResilienceResult& resilience,
    const RoadResilienceBypassConfig& cfg,
    const TrafficResult* traffic = nullptr);

// Apply a suggested bypass to the world (charging money via World::applyRoad).
//
// This validates buildability (overlay None/Road) and bridge requirements, and
// respects a minimum money reserve.

enum class RoadResilienceBypassApplyResult : std::uint8_t {
  Applied = 0,
  Noop,
  OutOfBounds,
  Blocked,
  NeedsBridges,
  InsufficientFunds,
};

struct RoadResilienceBypassApplyReport {
  RoadResilienceBypassApplyResult result = RoadResilienceBypassApplyResult::Noop;
  int moneyCost = 0;
  int builtTiles = 0;
  int upgradedTiles = 0;
};

RoadResilienceBypassApplyReport ApplyRoadResilienceBypass(World& world, const RoadResilienceBypassSuggestion& s,
                                                         int minMoneyReserve = 0);

} // namespace isocity
