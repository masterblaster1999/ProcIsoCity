#pragma once

#include "isocity/RunoffPollution.hpp"
#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Hydrology-aware green infrastructure placement suggestions.
//
// This module builds on the deterministic RunoffPollution routing model to
// estimate where adding filtration (modeled as converting empty tiles to parks)
// would most reduce *population-weighted* downstream exposure.
//
// Unlike the amenity-driven ParkOptimizer, this is a "stormwater lens" tool:
// it prioritizes tiles that intercept large routed pollutant mass before it
// reaches many residents.

enum class RunoffMitigationDemandMode : std::uint8_t {
  ResidentialOccupants = 0, // weight = occupants on Residential tiles
  AllOccupants = 1,         // weight = occupants on any tile
  ResidentialTiles = 2,     // weight = 1 on Residential tiles
  ZoneTiles = 3,            // weight = 1 on R/C/I zone tiles
};

struct RunoffMitigationConfig {
  // The underlying runoff model configuration (sources, dilution, filtration).
  RunoffPollutionConfig runoffCfg{};

  // How "downstream impact" is weighted.
  RunoffMitigationDemandMode demandMode = RunoffMitigationDemandMode::ResidentialOccupants;

  // How many new park tiles to suggest.
  int parksToAdd = 12;

  // Minimum Manhattan distance between suggested parks (>=0).
  int minSeparation = 3;

  // Candidate filtering.
  bool allowReplaceRoad = false;
  bool allowReplaceZones = false; // Residential/Commercial/Industrial/civic

  // If true, water tiles are never selected.
  bool excludeWater = true;
};

struct RunoffMitigationPlacement {
  Point tile{0, 0};

  // Raw first-order objective reduction estimate used for ranking.
  // Larger is better.
  double benefit = 0.0;
};

struct RunoffMitigationResult {
  int w = 0;
  int h = 0;
  RunoffMitigationConfig cfg{};

  int candidateCount = 0;

  // Per-tile score fields.
  std::vector<float> priorityRaw; // >=0 (not normalized)
  std::vector<float> priority01;  // normalized to [0,1]

  // Plan mask for suggested parks (0/1).
  std::vector<std::uint8_t> planMask;

  // Suggested park placements (in greedy selection order).
  std::vector<RunoffMitigationPlacement> placements;

  // Population-weighted exposure objective before/after applying plan.
  // Units are arbitrary but consistent within a run.
  double objectiveBefore = 0.0;
  double objectiveAfter = 0.0;
  double objectiveReduction = 0.0;
};

// Compute stormwater-driven park placement suggestions.
//
// traffic is optional; when omitted, the runoff model falls back to
// RunoffPollutionConfig::fallbackCommuteTraffic01 for road sources.
RunoffMitigationResult SuggestRunoffMitigationParks(const World& world,
                                                   const RunoffMitigationConfig& cfg = {},
                                                   const TrafficResult* traffic = nullptr);

// Apply suggested placements to a world (Overlay::Park) without charging money.
//
// NOTE: This does not recompute derived simulator stats. Tooling callers usually
// follow with Simulator::refreshDerivedStats().
void ApplyRunoffMitigationParks(World& world, const std::vector<RunoffMitigationPlacement>& placements);

} // namespace isocity
