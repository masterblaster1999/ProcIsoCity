#pragma once

#include "isocity/World.hpp"

#include <algorithm>
#include <cmath>

namespace isocity {

// Controls how Tile::height is visualized as screen-space elevation.
//
// This is intentionally independent from the simulation: elevation is currently a visual + editing
// feature (terraforming). Core systems (traffic/goods/etc.) do not use slopes yet.
struct ElevationSettings {
  // Maximum vertical offset in world units (pixels) when Tile::height == 1.
  // Set to 0 to render the world flat.
  float maxPixels = 0.0f;

  // 0 => no quantization (smooth). Otherwise snap height to N steps for a voxel/terrace look.
  int quantizeSteps = 0;

  // If true, water is always rendered at elevation 0 regardless of Tile::height.
  bool flattenWater = true;
};

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline float TileElevationPx(const Tile& t, const ElevationSettings& s)
{
  if (s.maxPixels <= 0.0f) return 0.0f;
  if (s.flattenWater && t.terrain == Terrain::Water) return 0.0f;

  float h = Clamp01(t.height);

  if (s.quantizeSteps > 0) {
    const float q = static_cast<float>(s.quantizeSteps);
    h = std::round(h * q) / q;
  }

  return h * s.maxPixels;
}

} // namespace isocity
