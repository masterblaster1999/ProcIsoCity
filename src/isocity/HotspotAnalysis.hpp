#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Spatial hotspot / clustering analysis.
//
// We implement the Getis-Ord Gi* statistic for a per-tile scalar field.
// The result is a per-tile z-score-like statistic plus a convenient 0..1 mapping
// for visualization.

enum class HotspotClass : std::uint8_t {
  Cold = 0,
  Neutral = 1,
  Hot = 2,
};

// Stable name used for reports/debugging.
const char* HotspotClassName(HotspotClass c);

struct HotspotConfig {
  bool enabled = true;

  // Square window radius (Chebyshev distance) used for neighborhood sums.
  // radius=0 means only the tile itself.
  int radius = 6;

  // If true, water tiles are excluded from the analysis (both globally and locally).
  bool excludeWater = true;

  // z-score threshold used for classification (typical: 1.96 for ~95% under normality).
  float zThreshold = 1.96f;

  // Mapping parameter used for z->01 conversion:
  //   z01 = 0.5 + 0.5 * tanh(z / zScale)
  float zScale = 3.0f;
};

struct HotspotResult {
  int w = 0;
  int h = 0;
  HotspotConfig cfg{};

  int validCount = 0;
  float mean = 0.0f;
  float stdev = 0.0f;

  // Per-tile Getis-Ord Gi* z-scores (0 for invalid/uncomputed).
  std::vector<float> z;

  // Per-tile z mapped to [0,1] (0=cold, 0.5=neutral, 1=hot).
  std::vector<float> z01;

  // Per-tile classification (HotspotClass), stored as byte for compactness.
  std::vector<std::uint8_t> cls;

  int hotCount = 0;
  int coldCount = 0;
};

// Compute Getis-Ord Gi* hotspots for a scalar field.
//
// - field must be size w*h.
// - validMask (optional) must also be size w*h. When provided, only mask!=0
//   participates in the global statistics and neighborhood sums.
// - Neighborhood is a square window with radius cfg.radius and binary weights.
HotspotResult ComputeHotspotsGiStar(int w, int h,
                                   const std::vector<float>& field,
                                   const std::vector<std::uint8_t>* validMask = nullptr,
                                   const HotspotConfig& cfg = {});

// Convenience wrapper: builds a validMask from the world (optionally excluding water).
HotspotResult ComputeHotspotsGiStar(const World& world,
                                   const std::vector<float>& field,
                                   const HotspotConfig& cfg = {});

} // namespace isocity
