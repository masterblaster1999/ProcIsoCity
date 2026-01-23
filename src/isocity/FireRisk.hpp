#pragma once

#include "isocity/Isochrone.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A deterministic, lightweight fire-risk model.
//
// This is *not* a full fire simulation. The intent is to provide a
// SimCity-style "risk/coverage" view that:
//  - highlights dense, contiguous flammable development (R/C/I)
//  - considers response distance to Fire Stations along the road network
//  - treats parks/water/roads as partial firebreaks by lowering local risk
//
// The output is a per-tile risk in [0,1] and a per-tile "coverage" proxy in [0,1]
// derived from travel-time weighted shortest paths.

struct FireRiskConfig {
  // If true, the road traversal for response distance is limited to roads that
  // connect to the map border ("outside connection"), matching the simulation's
  // requireOutsideConnection behavior.
  bool requireOutsideConnection = true;

  // Whether response distance uses step-count or travel-time weighting.
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;

  // Maximum response distance (in road steps). Tiles beyond this radius
  // receive zero coverage.
  int responseRadiusSteps = 18;

  // Per-tile access cost (in milli-steps) added when mapping a non-road tile to
  // its access road. Setting this to a non-zero value slightly penalizes large
  // zoning blocks.
  int accessStepCostMilli = 0;

  // Base ignition/flammability weights by land use.
  float baseEmpty = 0.06f;
  float baseRoad = 0.03f;
  float basePark = 0.14f;
  float baseResidential = 0.55f;
  float baseCommercial = 0.65f;
  float baseIndustrial = 0.85f;
  float baseCivic = 0.32f;
  float baseWater = 0.0f;

  // Additional multiplicative risk from occupancy density.
  // risk *= (1 + occupancyWeight * occFrac)
  float occupancyWeight = 0.40f;

  // Additional multiplicative risk from building level.
  // risk *= (1 + levelWeight * level01)
  float levelWeight = 0.20f;

  // Diffusion / smoothing iterations to "spread" risk across contiguous
  // neighborhoods. This makes hot-spots feel less noisy and more district-like.
  int diffusionIterations = 3;
  float diffusion = 0.25f;      // 0..1
  bool diffusionEightConnected = true;

  // Risk reduction from coverage.
  // risk *= (1 - coverageMitigation * coverage01)
  float coverageMitigation = 0.65f;

  // Threshold used for summary stats.
  float highRiskThreshold = 0.75f;
};

struct FireRiskResult {
  int w = 0;
  int h = 0;

  // Number of Fire Stations discovered in the world.
  int fireStationCount = 0;
  int sourceRoadCount = 0;

  // Response cost in milli-steps (Street step = 1000). -1 = unreachable.
  std::vector<int> responseCostMilli;

  // Coverage proxy in [0,1] derived from responseCostMilli.
  std::vector<float> coverage01;

  // Intermediate risk field (pre-clamp) after diffusion and coverage mitigation.
  std::vector<float> riskRaw;

  // Final risk field in [0,1].
  std::vector<float> risk01;

  // Summary across R/C/I zone tiles (useful for UI/news).
  float avgZoneRisk = 0.0f;
  float avgZoneCoverage = 0.0f;
  int highRiskZoneTiles = 0;
};

// Compute per-tile fire risk and response coverage.
//
// precomputedZoneAccess / precomputedRoadToEdge can be supplied to avoid
// repeated computation if the caller already has these maps.
FireRiskResult ComputeFireRisk(const World& world, const FireRiskConfig& cfg = {},
                               const ZoneAccessMap* precomputedZoneAccess = nullptr,
                               const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr);

} // namespace isocity
