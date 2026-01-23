#pragma once

#include "isocity/World.hpp"

#include <vector>

namespace isocity {

// A lightweight, deterministic land-use mix / diversity metric.
//
// In urban planning literature, Shannon entropy is a common way to quantify
// how "mixed" an area is (0 = single use, 1 = perfectly even distribution
// across all considered use categories).
//
// We compute a per-tile score using a square neighborhood window and a small
// set of land-use categories derived from the tile overlay. The implementation
// uses integral images for O(1) neighborhood queries per tile.
//
// Notes:
//  - This is not meant to be a prescriptive real-world planning metric.
//    It is designed as a stable, tunable heuristic suitable for procedural
//    city generation debugging and gameplay/analysis tooling.

struct LandUseMixConfig {
  // Neighborhood radius in tiles (square window: (2r+1)^2).
  int radius = 6;

  // Include parks as a land-use category.
  bool includeParks = true;

  // Include civic/service buildings (school/hospital/police/fire) as a single
  // additional category.
  bool includeCivic = false;

  // If true, down-weight entropy by local "developed" density so tiny or
  // sparsely-built neighborhoods don't appear overly mixed.
  bool applyDensityWeight = true;
};

struct LandUseMixResult {
  int w = 0;
  int h = 0;
  int radius = 0;
  int categories = 0;

  // Final mix score in [0,1] per tile.
  std::vector<float> mix01;

  // Fraction of tiles in the neighborhood window that belong to the considered
  // land-use categories (0..1). Useful for visualization fading.
  std::vector<float> density01;

  float maxMix = 0.0f;
};

// Compute per-tile land-use mix. The result vectors are size w*h.
LandUseMixResult ComputeLandUseMix(const World& world, const LandUseMixConfig& cfg = {});

} // namespace isocity
