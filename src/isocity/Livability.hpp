#pragma once

#include "isocity/AirPollution.hpp"
#include "isocity/Goods.hpp"
#include "isocity/HeatIsland.hpp"
#include "isocity/NoisePollution.hpp"
#include "isocity/Services.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Walkability.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A deterministic, explainable "livability" composite index.
//
// This module is intended to give scenario generators and external analysis tools
// a single, human-centric field that aggregates:
//  - access to civic services
//  - walkability / amenity accessibility
//  - environmental hazards (air pollution, noise, heat)
//
// The output is a per-tile score in [0,1] (higher = better), plus an optional
// "intervention priority" score in [0,1] (higher = more urgent).
//
// The priority score is population-weighted: low livability in densely occupied
// residential tiles rises to the top.

struct LivabilityConfig {
  // Component weights. These will be normalized internally.
  float weightServices = 0.30f;
  float weightWalkability = 0.25f;
  float weightCleanAir = 0.20f;
  float weightQuiet = 0.15f;
  float weightThermalComfort = 0.10f;

  // Service / walkability model settings.
  bool requireOutsideConnection = true;
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;

  // Services catchment radius (in road-network steps).
  int servicesCatchmentRadiusSteps = 18;

  // Walkability coverage threshold (in steps). A tile scores well when key
  // amenities are reachable within this budget.
  int walkCoverageThresholdSteps = 15;

  // Convert hazard -> comfort via:
  //   comfort = pow(1 - hazard01, hazardComfortExponent)
  // A value > 1 makes the index more sensitive to high hazards.
  float hazardComfortExponent = 1.0f;

  // Priority scoring.
  //
  // pop01 = clamp(occupants / priorityOccupantScale, 0, 1)
  // priority = pow(1 - livability, priorityNeedExponent) * pow(pop01, priorityOccupantExponent)
  int priorityOccupantScale = 80;
  float priorityOccupantExponent = 0.5f;
  float priorityNeedExponent = 1.0f;
};

struct LivabilityResult {
  int w = 0;
  int h = 0;
  LivabilityConfig cfg{};

  // Per-tile score in [0,1]. Higher is better.
  std::vector<float> livability01;

  // Per-tile intervention priority in [0,1]. Higher means more urgent.
  std::vector<float> priority01;

  float maxLivability01 = 0.0f;
  float maxPriority01 = 0.0f;

  // Residential-only summary stats.
  int residentPopulation = 0;
  int residentTileCount = 0;

  float residentMeanLivability01 = 0.0f;

  float residentMeanServices01 = 0.0f;
  float residentMeanWalkability01 = 0.0f;
  float residentMeanCleanAir01 = 0.0f;
  float residentMeanQuiet01 = 0.0f;
  float residentMeanThermalComfort01 = 0.0f;

  // Weighted percentiles of livability among residents.
  float residentP10 = 0.0f;
  float residentMedian = 0.0f;
  float residentP90 = 0.0f;

  // Weighted Gini coefficient of livability among residents (0=equal, 1=unequal).
  float residentGini = 0.0f;
};

// Compute a composite livability score per tile.
//
// traffic/goods are optional. If provided, noise/heat/air models can incorporate
// commute and freight intensity.
LivabilityResult ComputeLivability(const World& world, const LivabilityConfig& cfg = {},
                                  const TrafficResult* traffic = nullptr,
                                  const GoodsResult* goods = nullptr);

} // namespace isocity
