#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A lightweight, deterministic solar exposure + rooftop PV potential heuristic.
//
// The goal is *not* physical accuracy; it's to provide a useful, explainable signal for
// tooling and exports:
//   - exposure01: "how much sky/sun does this tile see?" (0..1)
//   - roofArea01: rough proxy for available rooftop area on zoned/civic tiles (0..1)
//   - potential01: exposure01 * roofArea01 (0..1)
//
// The model uses a coarse horizon scan in a small number of azimuth directions and a
// few solar-altitude samples. It supports a single-sun-position mode that is handy for
// tests and debugging.
//
// Coordinate convention for azimuth degrees (for singleSample):
//   - 0°   = East (+x)
//   - 90°  = North (-y)
//   - 180° = West (-x)
//   - 270° = South (+y)

struct SolarPotentialConfig {
  // Maximum scan radius in tiles when computing the horizon.
  int maxHorizonRadius = 64;

  // Number of azimuth directions sampled (8 or 16 recommended). Values <= 8 use an
  // 8-direction compass; larger values use a 16-direction compass.
  int azimuthSamples = 16;

  // Solar altitude samples in degrees above the horizon.
  std::vector<float> altitudeDeg = {15.0f, 30.0f, 45.0f, 60.0f};

  // If true, compute exposure for a single sun position. This is mostly intended for
  // unit tests and "what if the sun is here?" debugging.
  bool singleSample = false;
  float singleAzimuthDeg = 90.0f;   // see convention above
  float singleAltitudeDeg = 25.0f;  // degrees above horizon

  // If true, include simple building heights (derived from overlay/level/occupants)
  // in the horizon/shading computation.
  bool includeBuildings = true;

  // --- Building height heuristic (added to Tile::height for shading) ---
  float residentialHeightPerLevel = 0.05f;
  float commercialHeightPerLevel = 0.07f;
  float industrialHeightPerLevel = 0.06f;
  float civicHeightPerLevel = 0.08f;

  // Extra height from occupant density (rough proxy for intensity).
  float occupantHeightBoost = 0.04f;
  int occupantScale = 60;

  // --- Rooftop area heuristic (0..1) ---
  float roofResidential = 0.55f;
  float roofCommercial = 0.75f;
  float roofIndustrial = 1.00f;
  float roofCivic = 0.65f;

  // Increase roof factor based on occupancy.
  float roofOccupantBoost = 0.25f;
  int roofOccupantScale = 80;

  // Clamp for the output fields (kept for tuning consistency with other heuristics).
  float clamp01 = 1.0f;
};

struct SolarPotentialResult {
  int w = 0;
  int h = 0;
  SolarPotentialConfig cfg{};

  // Per-tile solar exposure (0..1).
  std::vector<float> exposure01;

  // Per-tile "roof area" proxy (0..1). Non-building tiles are 0.
  std::vector<float> roofArea01;

  // Combined PV potential proxy (0..1): exposure01 * roofArea01.
  std::vector<float> potential01;

  float maxExposure01 = 0.0f;
  float maxPotential01 = 0.0f;

  // --- Simple summary stats (mostly for CLI / reports) ---
  int roofTileCount = 0;
  int residentPopulation = 0; // sum of occupants on Residential tiles
  float perCapitaPotential = 0.0f;

  float highPotentialThreshold = 0.65f;
  float roofHighPotentialFrac = 0.0f; // share of roof tiles with potential >= threshold
};

// Compute solar exposure + rooftop PV potential for a world.
SolarPotentialResult ComputeSolarPotential(const World& world, const SolarPotentialConfig& cfg = {});

} // namespace isocity
