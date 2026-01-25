#pragma once

#include "isocity/SeedMiner.hpp"

#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Mine embedding (2D visualization)
//
// When mining thousands of seeds, it is often useful to *visualize* how the
// discovered cities relate to each other.
//
// We provide a deterministic 2D embedding using classical Multidimensional
// Scaling (MDS): given a distance function between MineRecords (scalar KPI
// features, layout pHash, or a hybrid), compute an approximate Euclidean 2D
// layout that preserves those distances as well as possible.
//
// Design goals:
// - Deterministic (no RNG required).
// - No external dependencies.
// - Works for any metric distance used by the mining tooling.
// -----------------------------------------------------------------------------

struct MineEmbeddingConfig {
  // Distance space.
  MineDiversityMode space = MineDiversityMode::Hybrid;

  // Used when space==Hybrid. In [0,1].
  double layoutWeight = 0.50;

  // Used for scalar/hybrid: if true, standardize metrics with median+MAD.
  // If false, use mean/stddev.
  bool robustScaling = true;

  // Metrics used for scalar/hybrid. If empty, a reasonable default set is used.
  std::vector<MineMetric> metrics;

  // Power iteration steps used to extract the top eigenvectors.
  // Higher values are slower but can improve embedding stability.
  int powerIters = 64;
};

struct MineEmbeddingPoint {
  // Index into the input MineRecord array.
  int recIndex = -1;

  // 2D embedding coordinates (arbitrary scale).
  double x = 0.0;
  double y = 0.0;
};

struct MineEmbeddingResult {
  MineEmbeddingConfig cfg{};

  // One point per selected index, in the same order as `selectedIndices`.
  std::vector<MineEmbeddingPoint> points;

  // Leading eigenvalues of the (centered) Gram matrix.
  double eigen1 = 0.0;
  double eigen2 = 0.0;

  bool ok = false;
  std::string warning;
};

// Compute a 2D embedding using classical MDS over the chosen distance space.
//
// Notes:
// - `selectedIndices` are indices into `recs`.
// - The result points are returned in the same order as `selectedIndices`.
// - If the underlying distance is not perfectly Euclidean, negative eigenvalues
//   can appear. We clamp negative eigenvalues to 0 when producing coordinates.
MineEmbeddingResult ComputeMineEmbeddingMDS(const std::vector<MineRecord>& recs,
                                           const std::vector<int>& selectedIndices,
                                           const MineEmbeddingConfig& cfg);

} // namespace isocity
