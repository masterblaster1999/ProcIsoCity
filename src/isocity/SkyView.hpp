#pragma once

#include "isocity/World.hpp"

#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Sky View Factor (SVF) / Urban Canyon Confinement
//
// A lightweight, deterministic urban-morphology heuristic:
//   - skyView01: approximate "how much sky is visible" from each tile (0..1)
//   - canyon01:  complementary confinement proxy (1 - skyView01)
//
// This intentionally trades physical accuracy for:
//   - speed (O(N * radius * dirs))
//   - determinism
//   - explainability
//
// Method:
//   1. Build an "effective height" for each tile: Tile::height plus a simple
//      building-height heuristic derived from overlay/level/occupants.
//   2. For each tile, scan outward in a small set of azimuth directions and
//      compute a horizon elevation angle (max atan2(dh, dist)).
//   3. Approximate sky view factor as the average of cos^2(horizonAngle).
//
// Useful as an urban-canyon / ventilation proxy, and as a standalone map layer.
// -----------------------------------------------------------------------------

struct SkyViewConfig {
  // Maximum scan radius in tiles when computing the horizon.
  int maxHorizonRadius = 64;

  // Number of azimuth directions sampled (8 or 16 recommended). Values <= 8 use
  // an 8-direction compass; larger values use a 16-direction compass.
  int azimuthSamples = 16;

  // If true, include simple building heights (derived from overlay/level/occupants)
  // in the horizon computation.
  bool includeBuildings = true;

  // --- Building height heuristic (added to Tile::height for occlusion) ---
  float residentialHeightPerLevel = 0.05f;
  float commercialHeightPerLevel = 0.07f;
  float industrialHeightPerLevel = 0.06f;
  float civicHeightPerLevel = 0.08f;

  // Extra height from occupant density (rough proxy for intensity).
  float occupantHeightBoost = 0.04f;
  int occupantScale = 60;
};

struct SkyViewResult {
  int w = 0;
  int h = 0;
  SkyViewConfig cfg{};

  // Per-tile approximate sky view factor (0..1). Higher = more open.
  std::vector<float> skyView01;

  // Per-tile confinement proxy (0..1): 1 - skyView01. Higher = more enclosed.
  std::vector<float> canyon01;

  // Simple summary stats.
  float meanSkyView = 0.0f;
  float meanRoadSkyView = 0.0f;
  int roadTileCount = 0;
};

// Compute the sky view factor + canyon confinement proxies for a world.
SkyViewResult ComputeSkyViewFactor(const World& world, const SkyViewConfig& cfg = {});

} // namespace isocity
