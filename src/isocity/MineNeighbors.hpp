#pragma once

#include "isocity/SeedMiner.hpp"

#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Mine neighbors (k-nearest-neighbors graph)
//
// Mining often yields a ranked set of seeds, but exploration workflows benefit
// from *local* navigation:
//   - "show me cities most similar to this one"
//   - "walk the space" by following nearest-neighbor links
//
// This module computes a deterministic kNN graph over a selected subset of
// MineRecords using the same mining distance spaces used elsewhere:
//   - Scalar KPI feature space
//   - Layout (pHash Hamming distance)
//   - Hybrid blend
//
// The result is designed to be embedded into gallery JSON/HTML exports.
// -----------------------------------------------------------------------------

struct MineNeighborsConfig {
  // Number of neighbors per point (k). Clamped into [0, n-1].
  int k = 8;

  // Distance space.
  MineDiversityMode space = MineDiversityMode::Hybrid;

  // Used when space==Hybrid. In [0,1].
  double layoutWeight = 0.50;

  // Used for scalar/hybrid: if true, standardize metrics with median+MAD.
  // If false, use mean/stddev.
  bool robustScaling = true;

  // Metrics used for scalar/hybrid. If empty, a reasonable default set is used.
  std::vector<MineMetric> metrics;
};

struct MineNeighborsResult {
  MineNeighborsConfig cfg{};

  // Copy of the input selection (indices into `recs`).
  std::vector<int> selectedIndices;

  // neighbors[i] is a list of neighbor entry indices (0..n-1) for entry i,
  // sorted by ascending distance.
  std::vector<std::vector<int>> neighbors;

  // distances[i][j] is the distance to neighbors[i][j].
  std::vector<std::vector<double>> distances;

  bool ok = false;
  std::string warning;
};

// Compute deterministic k-nearest-neighbors for the selected indices.
//
// Notes:
// - Distances are computed between entries in the *selected subset* (not across
//   the entire mined record list).
// - Returned neighbor lists contain entry indices (0..n-1), which are stable
//   for the selected subset and map back to `selectedIndices[entry]`.
MineNeighborsResult ComputeMineNeighborsKNN(const std::vector<MineRecord>& recs,
                                           const std::vector<int>& selectedIndices,
                                           const MineNeighborsConfig& cfg);

} // namespace isocity
