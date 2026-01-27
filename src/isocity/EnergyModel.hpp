#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

struct SolarPotentialResult;
struct HeatIslandResult;

// -----------------------------------------------------------------------------
// Urban energy demand / rooftop solar balance heuristic
//
// This module provides a deterministic, explainable *proxy* for:
//   - building operational energy demand (by land use + occupants)
//   - rooftop solar supply potential (via SolarPotential)
//   - a per-tile net balance signal (supply - demand)
//
// It is intentionally not a physically accurate energy simulation. The goal is a
// stable, tunable signal that can be exported as map layers and used for city
// planning tools.
// -----------------------------------------------------------------------------

struct EnergyModelConfig {
  // --- Demand ---
  // Baseline demand per developed tile (dimensionless). This captures "always on"
  // building loads even when occupancy is low.
  float residentialBaseDemand = 0.15f;
  float commercialBaseDemand = 0.18f;
  float industrialBaseDemand = 0.22f;
  float civicBaseDemand = 0.20f;

  // Demand per occupant (occupants represent residents or workers depending on zone).
  float residentialDemandPerOccupant = 0.012f;
  float commercialDemandPerOccupant = 0.018f;
  float industrialDemandPerOccupant = 0.025f;
  float civicDemandPerOccupant = 0.016f;

  // Demand scaling by building level (1..3):
  //   demand *= (1 + levelDemandBoost * (level - 1))
  float levelDemandBoost = 0.15f;

  // If true, demand is multiplied by a factor derived from the heat island field:
  //   demand *= (1 + heatCoolingBoost * heat01)
  // This is a cheap proxy for increased cooling needs in hotter microclimates.
  bool useHeatIslandCooling = true;
  float heatCoolingBoost = 0.25f;

  // If true, buildings with 0 occupants still contribute their baseline demand.
  bool includeBaseDemandWhenEmpty = true;

  // --- Rooftop solar supply ---
  // Solar supply scale applied to SolarPotentialResult::potential01:
  //   solarRaw = solarSupplyScale * potential01 * (1 + levelSupplyBoost*(level-1))
  float solarSupplyScale = 1.4f;

  // Optional small boost for higher-intensity buildings (often more roof equipment
  // / larger roof footprints in this game's heuristics).
  float levelSupplyBoost = 0.05f;

  // If true, only zoned/civic tiles are considered to have rooftops.
  bool requireRoofForSolar = true;

  // Normalization stability epsilon.
  float minNormDenom = 1e-4f;
};

struct EnergyModelResult {
  int w = 0;
  int h = 0;
  EnergyModelConfig cfg{};

  // Raw proxy units (dimensionless).
  std::vector<float> demandRaw;
  std::vector<float> solarRaw;
  std::vector<float> netRaw; // solarRaw - demandRaw

  // Normalized [0,1] fields for visualization.
  std::vector<float> demand01;
  std::vector<float> solar01;

  // Energy balance mapped into [0,1]: 0=deficit, 0.5=neutral, 1=surplus.
  std::vector<float> balance01;

  float maxDemandRaw = 0.0f;
  float maxSolarRaw = 0.0f;
  float maxAbsNetRaw = 0.0f;

  // Aggregate stats.
  float totalDemandRaw = 0.0f;
  float totalSolarRaw = 0.0f;
  float totalNetRaw = 0.0f;

  int buildingTileCount = 0;
  int populationOnBuildingTiles = 0;

  // Proxy renewable share = totalSolar / totalDemand (clamped to [0,1]).
  float renewableShare01 = 0.0f;
};

EnergyModelResult ComputeEnergyModel(const World& world,
                                    const EnergyModelConfig& cfg = {},
                                    const SolarPotentialResult* solar = nullptr,
                                    const HeatIslandResult* heatIsland = nullptr);

} // namespace isocity
