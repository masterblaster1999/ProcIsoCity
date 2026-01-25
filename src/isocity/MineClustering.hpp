#pragma once

#include "isocity/SeedMiner.hpp"

#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Mine clustering (k-medoids in a general metric space)
//
// Mining can easily produce hundreds or thousands of candidate seeds.
// A ranked list is useful, but humans often want a *summary*:
//   - which cities are similar?
//   - what are representative examples (medoids) of each cluster?
//
// We implement a deterministic k-medoids clustering routine over the same
// mining distance spaces used elsewhere:
//   - Scalar KPI feature space
//   - Layout space (pHash Hamming distance)
//   - Hybrid (weighted sum)
//
// This works for non-Euclidean distances because medoids are always chosen
// among actual points, and only the distance metric is required.
// -----------------------------------------------------------------------------

struct MineClusteringConfig {
  // Requested cluster count. The algorithm clamps k into [1, n].
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

  // Maximum refinement iterations.
  int maxIters = 30;
};

struct MineClusteringResult {
  MineClusteringConfig cfg{};

  // Copy of the input selection (indices into `recs`).
  std::vector<int> selectedIndices;

  // assignment[i] is the cluster id for selectedIndices[i]. Size == n.
  // Cluster ids are in [0, k-1].
  std::vector<int> assignment;

  // Size == k. Number of points assigned to each cluster.
  std::vector<int> clusterSizes;

  // Size == k. medoidEntry[c] is an index into selectedIndices (0..n-1), or -1.
  std::vector<int> medoidEntry;

  // Size == k. medoidRecIndex[c] is an index into `recs`, or -1.
  std::vector<int> medoidRecIndex;

  // Sum of distances from each point to its assigned medoid.
  double totalCost = 0.0;

  // Mean silhouette coefficient over all points (in [-1, 1]).
  double avgSilhouette = 0.0;

  bool ok = false;
  std::string warning;
};

// Compute deterministic k-medoids clustering for the selected indices.
//
// Notes:
// - Initialization uses a deterministic farthest-first strategy.
// - Refinement alternates assignment/update steps until convergence or
//   maxIters is reached.
MineClusteringResult ComputeMineClusteringKMedoids(const std::vector<MineRecord>& recs,
                                                   const std::vector<int>& selectedIndices,
                                                   const MineClusteringConfig& cfg);

// Convenience: return medoid indices into `recs` (one per non-empty cluster).
std::vector<int> MineClusteringMedoidIndices(const MineClusteringResult& res);

} // namespace isocity
