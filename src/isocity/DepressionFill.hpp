#pragma once

#include <cstdint>
#include <vector>

namespace isocity {

// Priority-Flood depression filling.
//
// This is a classic DEM (digital elevation model) preprocessing step used in
// terrain hydrology to remove sinks / closed basins by "filling" them up to the
// lowest spill elevation.
//
// In ProcIsoCity, this is useful for:
//   - headless flood-risk / ponding analysis ("where would water accumulate?")
//   - generating deterministic "depression depth" maps for regression artifacts
//
// The implementation is intentionally dependency-free and deterministic so it
// can be used from tests, CLI tooling, and (optionally) the simulation layer.

struct DepressionFillConfig {
  // If true, all map edge cells are treated as drainage outlets.
  // This is the standard Priority-Flood setup for a bounded DEM.
  bool includeEdges = true;

  // Minimum "lift" above the current processed cell when filling a lower neighbor.
  //
  // epsilon=0 preserves perfectly flat spill surfaces (deterministic but can create
  // ambiguous flats). A small epsilon (e.g. 1e-5) can help downstream flow-direction
  // computations avoid ties.
  float epsilon = 0.0f;
};

struct DepressionFillResult {
  int w = 0;
  int h = 0;

  // Filled heightfield (size w*h, row-major). Same units as the input.
  std::vector<float> filled;

  // Per-cell water depth (filled - input height, clamped to >=0), size w*h.
  std::vector<float> depth;

  // Aggregates.
  int filledCells = 0;   // number of cells with depth > 0
  float maxDepth = 0.0f; // max depth over all cells
  double volume = 0.0;   // sum(depth) over all cells
};

// Fill depressions in a heightfield using a deterministic Priority-Flood.
//
// - heights: input heightfield, size must equal w*h (row-major).
// - drainMask (optional): extra outlet cells (size w*h). Any non-zero value is treated as an outlet.
//   Typical use: mark existing water bodies as drains so lakes are preserved rather than filled.
// - cfg: controls edge seeding and epsilon.
//
// On invalid inputs (dimension mismatch), returns an empty result (w/h copied, vectors empty).
DepressionFillResult FillDepressionsPriorityFlood(const std::vector<float>& heights,
                                                  int w,
                                                  int h,
                                                  const std::vector<std::uint8_t>* drainMask = nullptr,
                                                  const DepressionFillConfig& cfg = {});

} // namespace isocity
