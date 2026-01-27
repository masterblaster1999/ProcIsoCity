#pragma once

#include "isocity/FlowField.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Job accessibility + opportunity analytics.
//
// The simulation already models commuters traveling to the nearest job access
// point. This module generalizes that idea into two explainable per-tile fields:
//
//  1) job_access: "How easy is it to reach *a* job from here?" (nearest-job cost)
//  2) job_opportunity: "How many job opportunities are reachable?" (diffusion/gravity)
//
// Design goals:
//  - deterministic, dependency-free
//  - reuse existing road/zone access + flow-field machinery
//  - optionally incorporate predicted traffic congestion as an extra travel-time penalty

struct JobOpportunityConfig {
  bool enabled = true;

  // Match the classic outside-connection rule used by the simulator.
  bool requireOutsideConnection = true;

  // When true, road routing uses travel-time weights derived from road class.
  // When false, all road edges are treated as equal cost.
  bool useTravelTime = true;

  // Added when mapping a road cost onto a non-road tile.
  // Think "walk from the road to the parcel".
  int accessStepCostMilli = 1000;

  // Access score mapping:
  //   cost <= ideal -> score 1
  //   cost >= max   -> score 0
  int idealAccessCostMilli = 8000;
  int maxAccessCostMilli = 30000;

  // Include job zones as sources.
  bool includeCommercialJobs = true;
  bool includeIndustrialJobs = true;

  // --- Congestion-aware costs (optional) ---
  //
  // If traffic is provided, we can add a BPR-style extra cost for entering
  // congested road tiles:
  //   t = t0 * (1 + alpha * (v/c)^beta)
  // where v is commuters on the tile and c is the per-tile capacity.
  bool congestionCosts = true;

  int roadTileCapacity = 28;
  float congestionAlpha = 0.15f;
  float congestionBeta = 4.0f;
  float congestionCapacityScale = 1.0f;
  float congestionRatioClamp = 3.0f;

  // --- Opportunity diffusion ---
  //
  // Opportunity is modeled as a stable diffusion process on the road network:
  //   O = S + decay * P(O)
  // where S is job "source strength" on each road tile and P is a weighted
  // neighbor averaging operator.
  //
  // The resulting field behaves like a gravity model (many nearby jobs => high)
  // while remaining cheap and deterministic.
  int diffusionIterations = 28;

  // Contribution weight of neighbor influence per iteration (0..1).
  float diffusionDecay = 0.88f;

  // Edge impedance factor used for neighbor weights:
  //   w = exp(-edgeImpedanceBeta * edgeCostSteps)
  // where edgeCostSteps = (edgeCostMilli / 1000).
  float edgeImpedanceBeta = 0.35f;

  // --- Normalization ---
  // Use a robust percentile of the log-compressed opportunity field as the
  // "white point" for mapping to 0..1.
  float opportunityPercentile = 0.95f;
};

struct JobOpportunityResult {
  int w = 0;
  int h = 0;
  JobOpportunityConfig cfg{};

  // Number of unique road tiles that act as job sources.
  int jobSourceRoadTiles = 0;

  // Total job capacity represented by the sources.
  int jobSourceCapacity = 0;

  // Per-tile nearest-job travel-time cost in milli-steps. -1 means unreachable.
  std::vector<int> jobAccessCostMilli;

  // Per-tile normalized accessibility score (0..1). Higher is better.
  std::vector<float> jobAccess01;

  // Per-tile raw opportunity value (arbitrary units, >=0).
  std::vector<float> jobOpportunityRaw;

  // Per-tile normalized opportunity score (0..1). Higher is better.
  std::vector<float> jobOpportunity01;

  // Debug: opportunity on road tiles only (size w*h; non-road is 0).
  std::vector<float> roadOpportunityRaw;

  // --- Residential-weighted summary stats ---
  int residentTileCount = 0;
  int residentPopulation = 0;
  int residentUnreachablePopulation = 0;

  float residentMeanAccess01 = 0.0f;
  float residentMeanOpportunity01 = 0.0f;
  float residentMeanAccessCostSteps = 0.0f; // costMilli / 1000
};

// Compute job accessibility + opportunity for a world.
//
// traffic is optional. When provided and cfg.congestionCosts is enabled, the
// routing and diffusion weights incorporate congestion penalties.
//
// precomputedRoadToEdge: optional cached mask computed by ComputeRoadsConnectedToEdge.
// precomputedZoneAccess: optional cached ZoneAccessMap (must match outside-connection rule).
JobOpportunityResult ComputeJobOpportunity(const World& world, const JobOpportunityConfig& cfg = {},
                                          const TrafficResult* traffic = nullptr,
                                          const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr,
                                          const ZoneAccessMap* precomputedZoneAccess = nullptr);

} // namespace isocity
