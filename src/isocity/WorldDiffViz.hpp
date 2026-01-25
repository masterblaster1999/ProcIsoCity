#pragma once

#include "isocity/Export.hpp"
#include "isocity/World.hpp"

#include <cstdint>

namespace isocity {

// Bounding box of tiles that differ between two worlds (over the overlapping region).
struct WorldDiffBounds {
  int widthA = 0;
  int heightA = 0;
  int widthB = 0;
  int heightB = 0;

  int overlapW = 0;
  int overlapH = 0;

  // True when the two worlds have different dimensions.
  bool sizeMismatch = false;

  bool anyDifferent = false;

  // Inclusive min and exclusive max coordinates in the overlapping region.
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;

  // Number of tiles that differ (overlapping region only).
  int tilesDifferent = 0;
};

// Compute a bounding box around all tiles that differ between A and B.
// The box is expressed in the coordinate system of the overlapping region
// [0..min(wA,wB), 0..min(hA,hB)).
WorldDiffBounds ComputeWorldDiffBounds(const World& a, const World& b, float heightEps = 1e-6f);

// Render a color-coded diff visualization (RGB):
//   - R: terrain diffs (strong) + variation diffs (medium) + occupants diffs (medium)
//   - G: overlay diffs (strong) + variation diffs (medium) + level diffs (medium) + district diffs (low)
//   - B: height diffs (strong) + level diffs (medium) + occupants diffs (medium) + district diffs (low)
//
// All images are rendered over the overlapping region only (min dimensions when sizes differ).
PpmImage RenderWorldDiffColor(const World& a, const World& b, float heightEps = 1e-6f);

// Render a grayscale diff visualization where intensity corresponds to the number
// of differing fields per tile.
PpmImage RenderWorldDiffCount(const World& a, const World& b, float heightEps = 1e-6f);

} // namespace isocity
