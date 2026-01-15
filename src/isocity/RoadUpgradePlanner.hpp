#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// High-level planning utilities for suggesting road *upgrades* (Tile::level)
// under a budget, given a per-road-tile flow map.
//
// The intent is to support tooling / experimentation:
//  - choose which streets to upgrade into avenues/highways
//  - quantify approximate congestion relief and travel-time savings
//  - export a machine-readable "upgrade plan" for downstream processing

enum class RoadUpgradeObjective : std::uint8_t {
  Congestion = 0, // maximize reduction in per-tile excess flow (flow - capacity)
  Time = 1,       // maximize flow-weighted travel time saved
  Hybrid = 2,     // linear combination of Congestion + Time
};

struct RoadUpgradePlannerConfig {
  // Base capacity for a street tile. If useRoadLevelCapacity is true, capacity
  // is scaled by road class using RoadCapacityForLevel().
  int baseTileCapacity = 28;

  bool useRoadLevelCapacity = true;

  // If false, upgrade candidates use edge interior tiles only (excludes the
  // endpoint node tiles) which reduces overlap between edges.
  //
  // Edges with no interior tiles (length 1) still fall back to using endpoints
  // so they can be upgraded.
  bool upgradeEndpoints = false;

  // Maximum road level to propose (clamped to [1,3]).
  int maxTargetLevel = 3;

  // Only consider edges whose current max utilization (flow/capacity) is >= this.
  // Set to 0 to disable filtering.
  double minUtilConsider = 1.0;

  RoadUpgradeObjective objective = RoadUpgradeObjective::Congestion;

  // Only used when objective == Hybrid.
  double hybridExcessWeight = 1.0;
  double hybridTimeWeight = 1.0;

  // Money budget for selected upgrades.
  //  - budget < 0  => unlimited
  //  - budget == 0 => select nothing (useful for "report only")
  int budget = -1;
};

// Per-edge upgrade decision (chosen by PlanRoadUpgrades).
struct RoadUpgradeEdge {
  int edgeIndex = -1;
  int a = -1;
  int b = -1;
  int targetLevel = 1;

  int cost = 0; // estimated money cost (does not mutate World::stats().money)
  std::uint64_t timeSaved = 0;
  std::uint64_t excessReduced = 0;
  int tileCount = 0;
};

struct RoadUpgradePlan {
  int w = 0;
  int h = 0;
  RoadUpgradePlannerConfig cfg{};

  int totalCost = 0;
  std::uint64_t totalTimeSaved = 0;
  std::uint64_t totalExcessReduced = 0;

  // Planning runtime (wall-clock). Primarily for UI/profiling.
  double runtimeSec = 0.0;

  // Chosen upgrades (one per edge).
  std::vector<RoadUpgradeEdge> edges;

  // Per-tile target road level (0 = no change). Size = w*h.
  std::vector<std::uint8_t> tileTargetLevel;
};

// Plan road upgrades based on a per-tile flow map.
//
// `roadFlow` must be size world.width()*world.height() with indexing idx = y*w + x.
// Non-road tiles are ignored.
RoadUpgradePlan PlanRoadUpgrades(const World& world, const RoadGraph& g,
                                 const std::vector<std::uint32_t>& roadFlow,
                                 const RoadUpgradePlannerConfig& cfg = {});

// Apply a RoadUpgradePlan to a world by upgrading road Tile::level.
//
// This function is intentionally "tooling friendly": it does not charge money.
void ApplyRoadUpgradePlan(World& world, const RoadUpgradePlan& plan);

} // namespace isocity
