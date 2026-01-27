// Copyright 2026
// SPDX-License-Identifier: MIT

#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

struct TrafficResult;
struct SkyViewResult;

// -----------------------------------------------------------------------------
// Traffic safety proxy
// -----------------------------------------------------------------------------
//
// This module computes *relative* crash-risk and exposure maps using only in-game
// signals (traffic intensity, intersection geometry, and optional sky-view/canyon
// confinement).
//
// It is intentionally a deterministic, lightweight heuristic intended for
// gameplay/visualization and "what-if" planning. It is NOT a calibrated real-world
// crash prediction model.
// -----------------------------------------------------------------------------

struct TrafficSafetyConfig {
  bool enabled = true;

  // If true, only roads connected to the outside edge are considered.
  bool requireOutsideConnection = true;

  // Traffic normalization.
  float trafficPercentile = 0.95f; // p95 of road traffic counts
  float trafficExponent = 0.70f;   // curvature for traffic -> risk

  // Crash risk proxy:
  //   rawRisk = trafficTerm * (base + geometryWeight * geom01 + canyonWeight * canyon01)
  float baseFactor = 0.25f;
  float geometryWeight = 0.60f;
  float canyonWeight = 0.35f; // uses SkyViewResult::canyon01 (0..1). Set to 0 to ignore.

  // Robust scaling of raw risk into risk01.
  float riskPercentile = 0.95f;

  // Exposure is a neighborhood average (box filter) of risk01.
  int exposureRadius = 6;          // tiles
  float exposurePercentile = 0.95f;

  // Priority highlights residential tiles with high exposure and high population.
  float priorityPercentile = 0.95f;
};

struct TrafficSafetyResult {
  int w = 0;
  int h = 0;
  TrafficSafetyConfig cfg{};

  float trafficPctl = 0.0f;
  float riskScale = 1.0f;
  float exposureScale = 1.0f;
  float priorityScale = 1.0f;

  std::vector<float> risk01;     // per-tile crash risk (roads only)
  std::vector<float> exposure01; // per-tile neighborhood exposure (all tiles)
  std::vector<float> priority01; // per-tile intervention priority (residential)

  int roadTilesConsidered = 0;
  int residentPopulation = 0;
  float residentMeanExposure = 0.0f;
  float residentMeanPriority = 0.0f;
};

TrafficSafetyResult ComputeTrafficSafety(const World& world,
                                        const TrafficSafetyConfig& cfg = {},
                                        const TrafficResult* traffic = nullptr,
                                        const SkyViewResult* skyView = nullptr,
                                        const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr);

} // namespace isocity
