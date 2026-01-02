#pragma once

#include <cstdint>
#include <vector>

namespace isocity {

// Simple flood-risk utilities built on top of the project's heightfield.
//
// These helpers are dependency-free and deterministic so they can be used in
// headless tooling and tests.

struct SeaFloodConfig {
  // If true, only cells connected to the map edge (via flooded cells) are
  // considered flooded. This models coastal/edge-connected flooding.
  //
  // If false, any cell whose height <= seaLevel is flooded (ignores connectivity).
  bool requireEdgeConnection = true;

  // If true, use 8-neighborhood connectivity instead of 4-neighborhood.
  bool eightConnected = false;
};

struct SeaFloodResult {
  int w = 0;
  int h = 0;
  float seaLevel = 0.0f;

  // 0/1 per cell (row-major), size w*h.
  std::vector<std::uint8_t> flooded;

  // Per-cell flood depth (seaLevel - height) for flooded cells, else 0.
  std::vector<float> depth;

  int floodedCells = 0;
  float maxDepth = 0.0f;
};

// Compute a coastal flood mask for a given sea level.
//
// - heights: input heightfield, size must equal w*h (row-major).
// - seaLevel: threshold; cells with height <= seaLevel are floodable.
SeaFloodResult ComputeSeaLevelFlood(const std::vector<float>& heights,
                                   int w,
                                   int h,
                                   float seaLevel,
                                   const SeaFloodConfig& cfg = {});

// Connected component labeling for values above a threshold.
//
// This is useful for turning depth maps into per-depression / per-flood-region
// polygons via the Vectorize module.

struct ThresholdComponent {
  int label = 0;        // >= 1
  int area = 0;         // number of tiles in the component
  float maxValue = 0.0f;
  double sumValue = 0.0;

  // Tile-center sums (x+0.5, y+0.5) for centroids.
  double sumX = 0.0;
  double sumY = 0.0;

  // Inclusive bounds.
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;
};

struct ThresholdComponents {
  int w = 0;
  int h = 0;
  float threshold = 0.0f;
  bool eightConnected = false;

  // Size w*h; 0 = background, >=1 = component label.
  std::vector<int> labels;

  // components[label-1] corresponds to label.
  std::vector<ThresholdComponent> components;

  bool empty() const { return w <= 0 || h <= 0; }
};

ThresholdComponents LabelComponentsAboveThreshold(const std::vector<float>& values,
                                                 int w,
                                                 int h,
                                                 float threshold,
                                                 bool eightConnected = false);

} // namespace isocity
