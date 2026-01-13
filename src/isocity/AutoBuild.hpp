#pragma once

#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <string>
#include <vector>

namespace isocity {

// Deterministic "city bot" that can grow a town from an empty map.
//
// Primary goals:
//  - Useful for headless CI/regression scenarios (scripts + hashes)
//  - Quickly generate interesting test worlds without hand-authoring edits
//  - Keep behavior deterministic: same inputs => same outputs

struct AutoBuildConfig {
  // How many zone tiles to attempt to place per simulated day.
  int zonesPerDay = 3;

  // When placing zones, the bot will attempt to grow small contiguous "blocks"
  // from a single seed tile.
  //
  // This is a big quality-of-life improvement for headless scenarios:
  //  - the city grows faster (fewer days of very sparse development)
  //  - blocks look more realistic than isolated single-tile zoning
  //  - it leverages the engine's "interior zoning" rule (connected components
  //    inherit road access via boundary tiles)
  //
  // 1 => classic behavior (one zoning action places one tile)
  // N => attempt to place up to N connected tiles per zoning "seed".
  int zoneClusterMaxTiles = 4;

  // How many road expansion operations to attempt per simulated day.
  int roadsPerDay = 1;

  // How many parks to attempt to place per simulated day (parks are only placed
  // when the bot believes they are needed).
  int parksPerDay = 1;

  // If true, parks are placed using ParkOptimizer (a deterministic greedy
  // planner) instead of a local adjacency heuristic.
  bool useParkOptimizer = true;

  // New roads are built at this class/level (1=Street, 2=Avenue, 3=Highway).
  int roadLevel = 1;

  // If true, road expansion uses the road-building path planner (A* over a
  // money/slope-aware cost model) instead of simple straight spurs.
  //
  // The planner produces more plausible arterial growth and is much better at
  // routing around lakes / obstacles (especially when allowBridges==false).
  bool useRoadPlanner = true;

  // If true, allow building roads on water (bridges).
  bool allowBridges = false;

  // Don't spend below this reserve; the bot will pause construction to let the
  // sim accumulate money.
  int minMoneyReserve = 15;

  // Prefer placing parks near zones: target ratio is ~1 park per N zone tiles.
  int parkPerZoneTiles = 18;

  // --- Road upgrades ---
  // When congestion exceeds this threshold, the bot upgrades the most loaded
  // road tiles.
  bool autoUpgradeRoads = true;
  float congestionUpgradeThreshold = 0.25f;
  int roadUpgradesPerDay = 2;

  // --- Road resilience bypasses ---
  // Bridge edges (cut-edges) are single points of failure in the road graph.
  // When enabled, the bot can proactively build bypass roads that add redundancy
  // by creating an alternate connection around heavily-used bridge segments.
  bool autoBuildResilienceBypasses = false;
  float resilienceBypassCongestionThreshold = 0.35f;
  int resilienceBypassesPerDay = 1;

  // Planner tuning (mirrors the in-game resilience bypass planner).
  int resilienceBypassTop = 5;
  bool resilienceBypassMoneyObjective = true;
  int resilienceBypassTargetLevel = 1;
  bool resilienceBypassAllowBridges = false;
  int resilienceBypassMaxCost = 0; // 0 => no limit
  int resilienceBypassMaxNodesPerSide = 256;

  // Recompute land value field every N simulated days (1 = every day).
  // Lower values respond faster but cost more CPU.
  int landValueRecalcDays = 5;

  // When requireOutsideConnection is enabled in the SimConfig, only build zones
  // and parks adjacent to road components that touch the map edge.
  bool respectOutsideConnection = true;

  // If true and SimConfig::requireOutsideConnection is enabled, attempt to
  // create at least one road-to-edge connection before placing zones.
  bool ensureOutsideConnection = true;

  // Max length (in tiles) of a road expansion action.
  //
  // - With useRoadPlanner==false, this is the straight spur length cap.
  // - With useRoadPlanner==true, the planned corridor is truncated to this
  //   length if the optimal path is longer.
  int maxRoadSpurLength = 7;
};

struct AutoBuildReport {
  int daysRequested = 0;
  int daysSimulated = 0;

  int roadsBuilt = 0;
  int roadsUpgraded = 0;
  int zonesBuilt = 0;
  int parksBuilt = 0;

  int failedBuilds = 0;
};

// Parse a single config key/value pair.
//
// Keys are case-insensitive. Supported keys:
//  - zonesPerDay, roadsPerDay, parksPerDay
//  - zoneClusterMaxTiles
//  - useParkOptimizer
//  - roadLevel, allowBridges, useRoadPlanner
//  - minMoneyReserve, parkPerZoneTiles
//  - autoUpgradeRoads, congestionUpgradeThreshold, roadUpgradesPerDay
//  - autoBuildResilienceBypasses, resilienceBypassCongestionThreshold, resilienceBypassesPerDay
//  - resilienceBypassTop, resilienceBypassMoneyObjective, resilienceBypassTargetLevel
//  - resilienceBypassAllowBridges, resilienceBypassMaxCost, resilienceBypassMaxNodesPerSide
//  - landValueRecalcDays, respectOutsideConnection, ensureOutsideConnection
//  - maxRoadSpurLength
bool ParseAutoBuildKey(const std::string& key, const std::string& value, AutoBuildConfig& cfg, std::string& outError);

// Run the bot for N simulated days.
//
// The bot performs edits (roads/zones/parks) then advances the simulator by one
// day, repeating N times.
//
// If outDailyStats is non-null, Stats snapshots after each simulated day are
// appended (mirrors ScriptRunner::tick behavior).
AutoBuildReport RunAutoBuild(World& world, Simulator& sim, const AutoBuildConfig& cfg, int days,
                             std::vector<Stats>* outDailyStats = nullptr);

} // namespace isocity
