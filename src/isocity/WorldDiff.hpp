#pragma once

#include "isocity/World.hpp"

namespace isocity {

// Lightweight world diff statistics.
//
// Intended uses:
//   - headless tooling (compare saves or simulation outputs)
//   - debugging procedural generation regressions
//   - CI diagnostics when a deterministic hash changes
struct WorldDiffStats {
  int widthA = 0;
  int heightA = 0;
  int widthB = 0;
  int heightB = 0;

  // True when the two worlds have different dimensions.
  // In that case, diff counts are computed for the overlapping region only.
  bool sizeMismatch = false;

  // Number of tiles compared (overlapping region).
  int tilesCompared = 0;

  // Count of tiles where at least one field differs.
  int tilesDifferent = 0;

  // Per-field difference counts (also over the overlapping region).
  int terrainDifferent = 0;
  int overlayDifferent = 0;
  int heightDifferent = 0;
  int variationDifferent = 0;
  int levelDifferent = 0;
  int occupantsDifferent = 0;
  int districtDifferent = 0;
};

// Compare two worlds tile-by-tile.
//
// heightEps controls the floating-point tolerance for Tile::height comparisons.
WorldDiffStats DiffWorldTiles(const World& a, const World& b, float heightEps = 1e-6f);

} // namespace isocity
