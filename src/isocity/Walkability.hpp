#pragma once

#include "isocity/Isochrone.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace isocity {

struct ZoneAccessMap;

// Walkability / "15-minute city" style accessibility heuristic.
//
// This module computes per-tile distances (via the road network) to several
// amenity categories and maps them to a normalized 0..1 walkability score.
//
// The output is intended for:
//  - layer exports (top-down and isometric)
//  - tile_metrics.csv analysis
//  - simple batch CLI scoring
//
// Design goals:
//  - deterministic + dependency-free
//  - reuses the existing road isochrone / zone access machinery
//  - robust defaults aligned with in-game expectations (outside-connection rule)

enum class WalkAmenity : std::uint8_t {
  Park = 0,
  Retail = 1,
  Education = 2,
  Health = 3,
  Safety = 4,
  Count = 5,
};

const char* WalkAmenityName(WalkAmenity a);

struct WalkabilityCategoryConfig {
  bool enabled = true;

  // Distance at which the category is considered "excellent".
  // Within this radius, the category score is 1.
  int idealSteps = 6;

  // Distance at which the category contributes nothing.
  // Beyond this radius, the category score is 0.
  int maxSteps = 18;

  // Weight of this category in the combined overall score.
  float weight = 1.0f;
};

struct WalkabilityConfig {
  bool enabled = true;

  // Outside connection rule: if true, all routing is restricted to roads
  // connected to the map edge.
  bool requireOutsideConnection = true;

  // Routing metric.
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;

  // Added when mapping a road cost onto a non-road tile.
  // Think "walk from the road to the parcel".
  int accessStepCostMilli = 1000;

  // Coverage threshold used for the per-tile coverage bitmask and summary stats.
  // (e.g. "15-minute" radius). Interpreted in Street-step equivalents.
  int coverageThresholdSteps = 15;

  WalkabilityCategoryConfig park{.enabled = true, .idealSteps = 6, .maxSteps = 18, .weight = 1.0f};
  WalkabilityCategoryConfig retail{.enabled = true, .idealSteps = 6, .maxSteps = 18, .weight = 1.0f};
  WalkabilityCategoryConfig education{.enabled = true, .idealSteps = 8, .maxSteps = 24, .weight = 1.0f};
  WalkabilityCategoryConfig health{.enabled = true, .idealSteps = 8, .maxSteps = 24, .weight = 1.0f};
  WalkabilityCategoryConfig safety{.enabled = true, .idealSteps = 8, .maxSteps = 24, .weight = 1.0f};
};

struct WalkabilityResult {
  int w = 0;
  int h = 0;

  WalkabilityConfig cfg{};

  // How many distinct source road tiles were used per category.
  std::array<int, static_cast<int>(WalkAmenity::Count)> sourceCount{};

  // Per-tile access cost (milli-steps). -1 means unreachable.
  std::vector<int> costParkMilli;
  std::vector<int> costRetailMilli;
  std::vector<int> costEducationMilli;
  std::vector<int> costHealthMilli;
  std::vector<int> costSafetyMilli;

  // Per-tile normalized category scores (0..1).
  std::vector<float> park01;
  std::vector<float> retail01;
  std::vector<float> education01;
  std::vector<float> health01;
  std::vector<float> safety01;

  // Combined overall score (0..1).
  std::vector<float> overall01;

  // Bit i is set when category i is reachable within cfg.coverageThresholdSteps.
  std::vector<std::uint8_t> coverageMask;

  // ---- Simple residential-weighted summary ----
  int residentialTileCount = 0; // number of Residential tiles with occupants > 0
  int residentPopulation = 0;   // sum of occupants over Residential tiles
  float residentAvgOverall01 = 0.0f;
  std::array<float, static_cast<int>(WalkAmenity::Count)> residentCoverageFrac{};
  float residentAllCategoriesFrac = 0.0f; // share with all enabled amenities covered
};

// Compute walkability for a world.
//
// precomputedRoadToEdge: optional cached mask computed by ComputeRoadsConnectedToEdge.
// precomputedZoneAccess: optional cached ZoneAccessMap (must match outside-connection rule).
WalkabilityResult ComputeWalkability(const World& world,
                                    const WalkabilityConfig& cfg = {},
                                    const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr,
                                    const ZoneAccessMap* precomputedZoneAccess = nullptr);

} // namespace isocity
