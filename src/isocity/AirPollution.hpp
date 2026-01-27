#pragma once

#include "isocity/Goods.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A deterministic, lightweight air-pollution / air-quality transport heuristic.
//
// This model is intentionally simple, fast, and dependency-free. It combines
// local emission sources (traffic, goods, and land use) with a cheap
// advection–diffusion solver to produce a per-tile concentration field.
//
// Key design goals:
//  - deterministic: no runtime RNG, stable across runs
//  - explainable: emissions come from obvious sources
//  - useful: supports exports and tile_metrics.csv analysis
//
// Notes on semantics:
//  - pollution01 is in [0,1], where 0 means clean air and 1 means high pollution.
//  - emission01 is the clamped per-tile source term (also [0,1]) before transport.

enum class WindDir : std::uint8_t {
  None = 0,
  N,
  NE,
  E,
  SE,
  S,
  SW,
  W,
  NW,
};

const char* WindDirName(WindDir d);

// Convert a wind direction to a unit-ish vector.
// When d==None, returns (0,0).
void WindDirVector(WindDir d, float& outX, float& outY);

// Deterministically pick a plausible prevailing wind direction from a seed.
// This is used by default so different ProcGen seeds get different wind.
WindDir InferWindDirFromSeed(std::uint64_t seed);

struct AirPollutionConfig {
  // Transport iterations (more = wider spread).
  int iterations = 96;

  // Diffusion strength per iteration: 0 => none, 1 => replace with neighbor average.
  float diffusion = 0.12f;

  // Advection blend: 0 => no wind transport, 1 => fully sample from upwind.
  float advection = 0.38f;

  // Wind speed in tiles/iteration (semi-Lagrangian sample offset).
  float windSpeed = 1.05f;

  // Dissipation/decay per iteration (0..1). Represents mixing/ventilation.
  float decayPerIteration = 0.010f;

  // Use 8-connected neighbors for diffusion.
  bool eightConnected = true;

  // If true, choose wind direction from world.seed(). If false, use fixedWindDir.
  bool windFromSeed = true;
  WindDir fixedWindDir = WindDir::E;

  // --- Emission weights (heuristic, tunable) ---
  float roadBase = 0.08f;
  float roadClassBoost = 0.04f;    // extra emission for higher-class roads (level)
  float commuteTrafficBoost = 0.55f;
  float goodsTrafficBoost = 0.28f;

  float residentialSource = 0.04f;
  float commercialSource = 0.18f;
  float industrialSource = 0.72f;
  float civicSource = 0.08f;

  // --- Local sinks / ventilation (applied to the source term) ---
  float parkSink = 0.12f;
  float waterSink = 0.20f;

  // Higher elevations ventilate slightly better (Tile::height in [0,1]).
  float elevationVentilation = 0.10f;

  // Extra emission from local population/employment density (based on Tile::occupants).
  float occupantBoost = 0.10f;
  int occupantScale = 60;

  // Per-iteration deposition: certain surfaces actively remove pollution.
  float depositionPark = 0.10f;
  float depositionWater = 0.18f;

  // Clamp for emission/concentration fields.
  float clamp01 = 1.0f;

  // Fallback normalized traffic when traffic/goods results are not provided.
  float fallbackCommuteTraffic01 = 0.12f;
  float fallbackGoodsTraffic01 = 0.04f;
};

struct AirPollutionResult {
  int w = 0;
  int h = 0;
  AirPollutionConfig cfg{};

  // Per-tile emission source term in [0,1].
  std::vector<float> emission01;

  // Per-tile transported pollution concentration in [0,1].
  std::vector<float> pollution01;

  float maxEmission01 = 0.0f;
  float maxPollution01 = 0.0f;

  // ---- Simple residential-weighted exposure summary ----
  int residentialTileCount = 0; // Residential tiles with occupants > 0
  int residentPopulation = 0;   // sum of occupants over Residential tiles
  float residentAvgPollution01 = 0.0f;
  float residentHighExposureFrac = 0.0f; // share of residents with pollution01 >= highExposureThreshold

  float highExposureThreshold = 0.65f;
};

// Compute a per-tile air pollution field.
//
// traffic/goods are optional. If omitted, roads still emit via fallbackCommuteTraffic01/fallbackGoodsTraffic01.
AirPollutionResult ComputeAirPollution(const World& world,
                                       const AirPollutionConfig& cfg = {},
                                       const TrafficResult* traffic = nullptr,
                                       const GoodsResult* goods = nullptr);

} // namespace isocity
