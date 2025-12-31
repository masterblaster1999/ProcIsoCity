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

  // How many road expansion operations to attempt per simulated day.
  int roadsPerDay = 1;

  // How many parks to attempt to place per simulated day (parks are only placed
  // when the bot believes they are needed).
  int parksPerDay = 1;

  // New roads are built at this class/level (1=Street, 2=Avenue, 3=Highway).
  int roadLevel = 1;

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

  // Recompute land value field every N simulated days (1 = every day).
  // Lower values respond faster but cost more CPU.
  int landValueRecalcDays = 5;

  // When requireOutsideConnection is enabled in the SimConfig, only build zones
  // and parks adjacent to road components that touch the map edge.
  bool respectOutsideConnection = true;

  // If true and SimConfig::requireOutsideConnection is enabled, attempt to
  // create at least one road-to-edge connection before placing zones.
  bool ensureOutsideConnection = true;

  // Max length of a road spur built during expansion.
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
//  - roadLevel, allowBridges
//  - minMoneyReserve, parkPerZoneTiles
//  - autoUpgradeRoads, congestionUpgradeThreshold, roadUpgradesPerDay
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
