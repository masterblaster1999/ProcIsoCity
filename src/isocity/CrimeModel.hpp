#pragma once

#include "isocity/Isochrone.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

struct TrafficResult;
struct GoodsResult;
struct JobOpportunityResult;
struct NoiseResult;
struct ZoneAccessMap;

// Lightweight, deterministic crime-risk proxy.
//
// This module provides a per-tile crime risk score (0..1) and a policing
// accessibility score (0..1) derived from travel-time to the nearest police
// station.
//
// Design goals:
//  - deterministic and renderer-independent
//  - cheap enough for interactive heatmaps
//  - explainable (factors are simple and tunable)
//
// It intentionally avoids any claim of real-world predictive validity.

struct CrimeModelConfig {
  bool enabled = true;

  // Match the classic outside-connection rule used by several simulators.
  bool requireOutsideConnection = true;

  // How road distance is measured.
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;

  // Added when mapping a road cost onto a non-road tile ("walk from road to parcel").
  int accessStepCostMilli = 1000;

  // Police response mapping.
  // - responseHalfLifeCostMilli controls the curve: cost==halfLife => access ~= 0.5.
  // - responseMaxCostMilli fades access to 0 near this threshold.
  int responseHalfLifeCostMilli = 12000;
  int responseMaxCostMilli = 45000;

  // Strength of policing effect. risk *= (1 - policeSuppressionStrength * policeAccess01).
  float policeSuppressionStrength = 0.45f;

  // --- Congestion-aware routing (optional) ---
  // When enabled and traffic is provided, we add a BPR-style extra cost per road tile.
  bool congestionCosts = true;
  int roadTileCapacity = 28;
  float congestionAlpha = 0.15f;
  float congestionBeta = 4.0f;
  float congestionCapacityScale = 1.0f;
  float congestionRatioClamp = 3.0f;

  // --- Base risk by land use (rough defaults) ---
  float baseNone = 0.15f;
  float baseRoad = 0.30f;
  float baseResidential = 0.22f;
  float baseCommercial = 0.45f;
  float baseIndustrial = 0.55f;
  float basePark = 0.25f;
  float baseService = 0.12f;

  // Multiplier added per zone level above 1.
  float levelBoost = 0.06f;

  // --- Additional factors (added to risk before policing is applied) ---
  // Density proxy (uses Tile::occupants). Scaled by the 95th percentile of occupants.
  float occupantsWeight = 0.18f;

  // Economic stress proxy from job access/opportunity (when provided).
  float jobAccessWeight = 0.16f;       // (1 - jobAccess01)
  float jobOpportunityWeight = 0.06f;  // (1 - jobOpportunity01)

  // Activity / opportunity proxies (when provided).
  float trafficOpportunityWeight = 0.10f;
  float goodsTrafficWeight = 0.08f;
  float noiseWeight = 0.10f;

  // Output curve (applied after clamping to [0,1]).
  // <1 expands high-risk differences; >1 compresses.
  float riskCurveExp = 0.75f;
};

struct CrimeModelResult {
  int w = 0;
  int h = 0;
  CrimeModelConfig cfg{};

  int policeStations = 0;
  int policeAccessRoadTiles = 0;

  // Per-tile response cost to nearest police station in milli-steps. -1 => unreachable.
  std::vector<int> policeCostMilli;

  // Per-tile policing accessibility in [0,1]. Higher is better.
  std::vector<float> policeAccess01;

  // Per-tile crime risk in [0,1]. Higher is worse.
  std::vector<float> risk01;

  // --- Residential-weighted summaries ---
  int residentPopulation = 0;
  float residentMeanRisk = 0.0f;
  float residentMeanPoliceAccess = 0.0f;
};

CrimeModelResult ComputeCrimeModel(const World& world, const CrimeModelConfig& cfg = {},
                                  const TrafficResult* traffic = nullptr,
                                  const GoodsResult* goods = nullptr,
                                  const JobOpportunityResult* jobs = nullptr,
                                  const NoiseResult* noise = nullptr,
                                  const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr,
                                  const ZoneAccessMap* precomputedZoneAccess = nullptr);

} // namespace isocity
