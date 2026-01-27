#pragma once

#include "isocity/Hydrology.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A deterministic, lightweight runoff / stormwater pollution heuristic.
//
// The model estimates where pollutant *loads* originate (roads + land use), then
// routes that load downhill using a simple D4 flow-direction field (see Hydrology).
// Along the way, certain tiles act as partial sinks/filters (parks, grass, water),
// producing a routed concentration proxy.
//
// Semantics:
//  - localLoad01 is a per-tile source term in [0,1] (higher = more runoff load).
//  - pollution01 is a routed concentration proxy in [0,1] (higher = worse).
//
// The fields are intended for:
//  - exports (map layers)
//  - tile_metrics.csv analysis
//
// Design goals:
//  - deterministic: no runtime RNG
//  - explainable: sources and sinks are based on obvious world features
//  - dependency-free: uses only core utilities already in the project

struct RunoffPollutionConfig {
  // --- Load/source weights (heuristic, tunable) ---
  float roadBase = 0.10f;
  float roadClassBoost = 0.05f;     // additional load for higher class roads (level)
  float roadTrafficBoost = 0.55f;   // scale of normalized commute traffic contribution

  float residentialLoad = 0.05f;
  float commercialLoad = 0.18f;
  float industrialLoad = 0.70f;
  float civicLoad = 0.08f;

  // Extra load from local population/employment density.
  float occupantBoost = 0.10f;
  int occupantScale = 60;

  // Clamp for the raw local load value before normalization.
  float clampLoad = 1.0f;

  // Fallback normalized traffic when TrafficResult is not provided.
  float fallbackCommuteTraffic01 = 0.12f;

  // --- Routing / dilution ---
  // Dilution exponent for routed concentration:
  //   concentration = outflowMass / pow(flowAccum, dilutionExponent)
  //  - 0 => no dilution
  //  - 1 => divide by flow accumulation
  float dilutionExponent = 1.0f;

  // --- Filtering / sinks (fractions of mass retained/removed at a tile) ---
  float filterPark = 0.50f;
  float filterGrass = 0.05f;
  float filterSand = 0.02f;
  float filterRoad = 0.00f;

  bool waterIsSink = true;
  float filterWater = 0.95f;

  // --- Exposure summary thresholds ---
  float highExposureThreshold01 = 0.65f;
};

struct RunoffPollutionResult {
  int w = 0;
  int h = 0;
  RunoffPollutionConfig cfg{};

  // Flow accumulation from the hydrology field (>=1 for non-empty maps).
  std::vector<int> flowAccum;
  int maxFlowAccum = 0;

  // Per-tile local source term.
  std::vector<float> localLoad;   // raw (clamped)
  std::vector<float> localLoad01; // normalized [0,1]

  // Per-tile routed concentration proxy.
  std::vector<float> concentration; // raw
  std::vector<float> pollution01;   // normalized [0,1]

  float maxLocalLoad = 0.0f;
  float maxConcentration = 0.0f;

  // ---- Simple residential-weighted exposure summary ----
  int residentialTileCount = 0; // Residential tiles with occupants > 0
  int residentPopulation = 0;   // sum of occupants over Residential tiles
  float residentAvgPollution01 = 0.0f;
  float residentHighExposureFrac = 0.0f; // share of residents with pollution01 >= threshold
};

// Compute runoff pollution layers.
//
// traffic is optional; if omitted, road load uses fallbackCommuteTraffic01.
RunoffPollutionResult ComputeRunoffPollution(const World& world,
                                             const RunoffPollutionConfig& cfg = {},
                                             const TrafficResult* traffic = nullptr);

} // namespace isocity
