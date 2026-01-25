#include "isocity/MineClustering.hpp"

#include "isocity/PerceptualHash.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

namespace isocity {

namespace {

static std::vector<MineMetric> DefaultClusteringMetrics()
{
  // A compact but expressive behavior vector spanning macro KPIs and physical layout.
  // Users can override this via MineClusteringConfig::metrics.
  return {
      MineMetric::Population,
      MineMetric::Happiness,
      MineMetric::AvgLandValue,
      MineMetric::TrafficCongestion,
      MineMetric::GoodsSatisfaction,
      MineMetric::ServicesOverallSatisfaction,
      MineMetric::WaterFrac,
      MineMetric::RoadFrac,
      MineMetric::ZoneFrac,
      MineMetric::ParkFrac,
      MineMetric::FloodRisk,
  };
}

static double MedianOfSorted(const std::vector<double>& v)
{
  if (v.empty()) return 0.0;
  const std::size_t n = v.size();
  const std::size_t mid = n / 2;
  if ((n & 1u) != 0u) return v[mid];
  return 0.5 * (v[mid - 1] + v[mid]);
}

static void FitStandardizer(const std::vector<MineRecord>& recs,
                           const std::vector<int>& sel,
                           const std::vector<MineMetric>& metrics,
                           bool robust,
                           std::vector<double>& center,
                           std::vector<double>& scale)
{
  const int n = static_cast<int>(sel.size());
  const int d = static_cast<int>(metrics.size());
  center.assign(static_cast<std::size_t>(std::max(0, d)), 0.0);
  scale.assign(static_cast<std::size_t>(std::max(0, d)), 1.0);

  if (n <= 0 || d <= 0) return;

  std::vector<double> col;
  col.reserve(static_cast<std::size_t>(n));

  for (int j = 0; j < d; ++j) {
    col.clear();
    const MineMetric m = metrics[static_cast<std::size_t>(j)];
    for (int i = 0; i < n; ++i) {
      const int ridx = sel[static_cast<std::size_t>(i)];
      if (ridx < 0 || static_cast<std::size_t>(ridx) >= recs.size()) continue;
      const double v = MineMetricValue(recs[static_cast<std::size_t>(ridx)], m);
      col.push_back(std::isfinite(v) ? v : 0.0);
    }

    if (col.empty()) {
      center[static_cast<std::size_t>(j)] = 0.0;
      scale[static_cast<std::size_t>(j)] = 1.0;
      continue;
    }

    if (robust) {
      std::sort(col.begin(), col.end());
      const double med = MedianOfSorted(col);

      std::vector<double> dev;
      dev.reserve(col.size());
      for (double v : col) dev.push_back(std::fabs(v - med));
      std::sort(dev.begin(), dev.end());
      const double mad = MedianOfSorted(dev);

      // Consistent MAD scale factor for normal distributions.
      double s = mad * 1.4826;
      if (!(s > 1.0e-12) || !std::isfinite(s)) s = 1.0;

      center[static_cast<std::size_t>(j)] = med;
      scale[static_cast<std::size_t>(j)] = s;
    } else {
      double mean = 0.0;
      for (double v : col) mean += v;
      mean /= static_cast<double>(col.size());

      double var = 0.0;
      for (double v : col) {
        const double dv = v - mean;
        var += dv * dv;
      }
      var /= static_cast<double>(col.size());
      double s = std::sqrt(var);
      if (!(s > 1.0e-12) || !std::isfinite(s)) s = 1.0;

      center[static_cast<std::size_t>(j)] = mean;
      scale[static_cast<std::size_t>(j)] = s;
    }
  }
}

static double ScalarDistance(const std::vector<double>& feats, int dim, int a, int b)
{
  if (dim <= 0) return 0.0;
  const std::size_t d = static_cast<std::size_t>(dim);
  const std::size_t baseA = static_cast<std::size_t>(a) * d;
  const std::size_t baseB = static_cast<std::size_t>(b) * d;

  double sum = 0.0;
  for (std::size_t j = 0; j < d; ++j) {
    const double dv = feats[baseA + j] - feats[baseB + j];
    sum += dv * dv;
  }
  const double dist = std::sqrt(sum);
  return dist / std::sqrt(static_cast<double>(dim));
}

static bool BetterByScoreSeed(const MineRecord& a, const MineRecord& b)
{
  if (a.score != b.score) return a.score > b.score;
  return a.seed < b.seed;
}

} // namespace

MineClusteringResult ComputeMineClusteringKMedoids(const std::vector<MineRecord>& recs,
                                                   const std::vector<int>& selectedIndices,
                                                   const MineClusteringConfig& cfg)
{
  MineClusteringResult out;
  out.cfg = cfg;
  out.selectedIndices = selectedIndices;

  const int n = static_cast<int>(selectedIndices.size());
  if (n <= 0) {
    out.ok = false;
    out.warning = "no selected indices";
    return out;
  }

  // Clamp k.
  int k = cfg.k;
  if (k <= 0) k = 1;
  k = std::min(k, n);
  out.cfg.k = k;

  const MineDiversityMode space = cfg.space;
  const double lw = std::clamp(cfg.layoutWeight, 0.0, 1.0);

  // Resolve metrics for scalar/hybrid spaces.
  std::vector<MineMetric> metrics = cfg.metrics;
  if ((space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) && metrics.empty()) {
    metrics = DefaultClusteringMetrics();
  }

  // Precompute standardized feature vectors for scalar distance (over selected subset).
  const int dim = static_cast<int>(metrics.size());
  std::vector<double> feats;
  if (space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) {
    std::vector<double> center;
    std::vector<double> scale;
    FitStandardizer(recs, selectedIndices, metrics, cfg.robustScaling, center, scale);

    feats.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(std::max(0, dim)), 0.0);
    for (int i = 0; i < n; ++i) {
      const int ridx = selectedIndices[static_cast<std::size_t>(i)];
      if (ridx < 0 || static_cast<std::size_t>(ridx) >= recs.size()) continue;
      const MineRecord& r = recs[static_cast<std::size_t>(ridx)];
      for (int j = 0; j < dim; ++j) {
        const double v = MineMetricValue(r, metrics[static_cast<std::size_t>(j)]);
        const double c = center[static_cast<std::size_t>(j)];
        const double s = scale[static_cast<std::size_t>(j)];
        feats[static_cast<std::size_t>(i) * static_cast<std::size_t>(dim) + static_cast<std::size_t>(j)] = (v - c) / s;
      }
    }
  }

  auto distEntry = [&](int aEntry, int bEntry) -> double {
    // aEntry/bEntry are indices into selectedIndices.
    const int aRec = selectedIndices[static_cast<std::size_t>(aEntry)];
    const int bRec = selectedIndices[static_cast<std::size_t>(bEntry)];
    if (aRec < 0 || bRec < 0 || static_cast<std::size_t>(aRec) >= recs.size() || static_cast<std::size_t>(bRec) >= recs.size()) {
      return 0.0;
    }

    if (space == MineDiversityMode::Layout) {
      const int hd = HammingDistance64(recs[static_cast<std::size_t>(aRec)].overlayPHash,
                                      recs[static_cast<std::size_t>(bRec)].overlayPHash);
      return static_cast<double>(hd) / 64.0;
    }

    const double ds = ScalarDistance(feats, dim, aEntry, bEntry);

    if (space == MineDiversityMode::Scalar) {
      return ds;
    }

    const int hd = HammingDistance64(recs[static_cast<std::size_t>(aRec)].overlayPHash,
                                    recs[static_cast<std::size_t>(bRec)].overlayPHash);
    const double dl = static_cast<double>(hd) / 64.0;
    return (1.0 - lw) * ds + lw * dl;
  };

  // ---------------------------------------------------------------------------
  // Initialization: deterministic farthest-first traversal.
  // ---------------------------------------------------------------------------
  std::vector<int> medoids;
  medoids.reserve(static_cast<std::size_t>(k));

  // First medoid: highest score (tie: smallest seed).
  int best0 = 0;
  for (int i = 1; i < n; ++i) {
    const int aRec = selectedIndices[static_cast<std::size_t>(best0)];
    const int bRec = selectedIndices[static_cast<std::size_t>(i)];
    if (aRec < 0 || bRec < 0 || static_cast<std::size_t>(aRec) >= recs.size() || static_cast<std::size_t>(bRec) >= recs.size()) continue;
    const MineRecord& a = recs[static_cast<std::size_t>(aRec)];
    const MineRecord& b = recs[static_cast<std::size_t>(bRec)];
    if (BetterByScoreSeed(b, a)) best0 = i;
  }
  medoids.push_back(best0);

  auto isMedoid = [&](int entry) -> bool {
    return std::find(medoids.begin(), medoids.end(), entry) != medoids.end();
  };

  while (static_cast<int>(medoids.size()) < k) {
    int pick = -1;
    double bestMinDist = -1.0;

    for (int i = 0; i < n; ++i) {
      if (isMedoid(i)) continue;

      double minD = std::numeric_limits<double>::infinity();
      for (int m : medoids) {
        minD = std::min(minD, distEntry(i, m));
      }
      if (!std::isfinite(minD)) minD = 0.0;

      if (minD > bestMinDist) {
        bestMinDist = minD;
        pick = i;
      } else if (minD == bestMinDist && pick >= 0) {
        // Tie-break deterministically by score then seed.
        const int pRec = selectedIndices[static_cast<std::size_t>(pick)];
        const int iRec = selectedIndices[static_cast<std::size_t>(i)];
        if (pRec >= 0 && iRec >= 0 && static_cast<std::size_t>(pRec) < recs.size() && static_cast<std::size_t>(iRec) < recs.size()) {
          const MineRecord& pa = recs[static_cast<std::size_t>(pRec)];
          const MineRecord& ib = recs[static_cast<std::size_t>(iRec)];
          if (BetterByScoreSeed(ib, pa)) pick = i;
        }
      }
    }

    if (pick < 0) break;
    medoids.push_back(pick);
  }

  // ---------------------------------------------------------------------------
  // Refinement: alternate assignment and medoid updates.
  // ---------------------------------------------------------------------------
  out.assignment.assign(static_cast<std::size_t>(n), 0);

  auto assignAll = [&]() {
    for (int i = 0; i < n; ++i) {
      int bestC = 0;
      double bestD = std::numeric_limits<double>::infinity();
      for (int c = 0; c < static_cast<int>(medoids.size()); ++c) {
        const double d = distEntry(i, medoids[static_cast<std::size_t>(c)]);
        if (d < bestD) {
          bestD = d;
          bestC = c;
        }
      }
      out.assignment[static_cast<std::size_t>(i)] = bestC;
    }
  };

  auto computeClusterSizes = [&]() {
    out.clusterSizes.assign(static_cast<std::size_t>(k), 0);
    for (int i = 0; i < n; ++i) {
      const int c = out.assignment[static_cast<std::size_t>(i)];
      if (c >= 0 && c < k) out.clusterSizes[static_cast<std::size_t>(c)]++;
    }
  };

  auto updateMedoids = [&]() -> bool {
    bool changed = false;

    // Build member lists.
    std::vector<std::vector<int>> members;
    members.resize(static_cast<std::size_t>(k));
    for (int i = 0; i < n; ++i) {
      const int c = out.assignment[static_cast<std::size_t>(i)];
      if (c >= 0 && c < k) members[static_cast<std::size_t>(c)].push_back(i);
    }

    // If any cluster is empty, re-seed its medoid to the farthest non-medoid point.
    for (int c = 0; c < k; ++c) {
      if (!members[static_cast<std::size_t>(c)].empty()) continue;

      int pick = -1;
      double bestMinDist = -1.0;
      for (int i = 0; i < n; ++i) {
        if (isMedoid(i)) continue;
        double minD = std::numeric_limits<double>::infinity();
        for (int m : medoids) {
          if (m == medoids[static_cast<std::size_t>(c)]) continue;
          minD = std::min(minD, distEntry(i, m));
        }
        if (!std::isfinite(minD)) minD = 0.0;
        if (minD > bestMinDist) {
          bestMinDist = minD;
          pick = i;
        }
      }

      if (pick >= 0) {
        medoids[static_cast<std::size_t>(c)] = pick;
        changed = true;
        // Rebuild members in next outer iteration (assignment will run again).
      }
    }

    // For each cluster, choose the point that minimizes sum distances to members.
    for (int c = 0; c < k; ++c) {
      if (members[static_cast<std::size_t>(c)].empty()) continue;

      int bestMed = medoids[static_cast<std::size_t>(c)];
      double bestCost = std::numeric_limits<double>::infinity();

      for (int cand : members[static_cast<std::size_t>(c)]) {
        double cost = 0.0;
        for (int other : members[static_cast<std::size_t>(c)]) {
          cost += distEntry(cand, other);
        }

        if (cost < bestCost) {
          bestCost = cost;
          bestMed = cand;
        } else if (cost == bestCost) {
          // Tie-break by score then seed.
          const int bRec = selectedIndices[static_cast<std::size_t>(bestMed)];
          const int cRec = selectedIndices[static_cast<std::size_t>(cand)];
          if (bRec >= 0 && cRec >= 0 && static_cast<std::size_t>(bRec) < recs.size() && static_cast<std::size_t>(cRec) < recs.size()) {
            const MineRecord& br = recs[static_cast<std::size_t>(bRec)];
            const MineRecord& cr = recs[static_cast<std::size_t>(cRec)];
            if (BetterByScoreSeed(cr, br)) bestMed = cand;
          }
        }
      }

      if (bestMed != medoids[static_cast<std::size_t>(c)]) {
        medoids[static_cast<std::size_t>(c)] = bestMed;
        changed = true;
      }
    }

    return changed;
  };

  const int iters = std::max(1, cfg.maxIters);
  for (int it = 0; it < iters; ++it) {
    assignAll();
    computeClusterSizes();

    const bool changed = updateMedoids();
    if (!changed) {
      break;
    }
  }

  // Final assignment after last medoid update.
  assignAll();
  computeClusterSizes();

  // Fill medoid outputs.
  out.medoidEntry.assign(static_cast<std::size_t>(k), -1);
  out.medoidRecIndex.assign(static_cast<std::size_t>(k), -1);
  for (int c = 0; c < k && c < static_cast<int>(medoids.size()); ++c) {
    const int e = medoids[static_cast<std::size_t>(c)];
    out.medoidEntry[static_cast<std::size_t>(c)] = e;
    if (e >= 0 && e < n) {
      out.medoidRecIndex[static_cast<std::size_t>(c)] = selectedIndices[static_cast<std::size_t>(e)];
    }
  }

  // Total cost.
  out.totalCost = 0.0;
  for (int i = 0; i < n; ++i) {
    const int c = out.assignment[static_cast<std::size_t>(i)];
    if (c < 0 || c >= k) continue;
    const int m = out.medoidEntry[static_cast<std::size_t>(c)];
    if (m < 0) continue;
    out.totalCost += distEntry(i, m);
  }

  // Silhouette score.
  // (This is O(n^2) but n is typically small for selected sets.)
  double silSum = 0.0;
  int silCount = 0;

  // Precompute members per cluster for convenience.
  std::vector<std::vector<int>> members;
  members.resize(static_cast<std::size_t>(k));
  for (int i = 0; i < n; ++i) {
    const int c = out.assignment[static_cast<std::size_t>(i)];
    if (c >= 0 && c < k) members[static_cast<std::size_t>(c)].push_back(i);
  }

  for (int i = 0; i < n; ++i) {
    const int c = out.assignment[static_cast<std::size_t>(i)];
    if (c < 0 || c >= k) continue;

    const std::vector<int>& own = members[static_cast<std::size_t>(c)];
    if (own.size() <= 1u) {
      // Undefined; conventionally 0.
      silSum += 0.0;
      silCount++;
      continue;
    }

    double a = 0.0;
    for (int j : own) {
      if (j == i) continue;
      a += distEntry(i, j);
    }
    a /= static_cast<double>(std::max<std::size_t>(1u, own.size() - 1u));

    double b = std::numeric_limits<double>::infinity();
    for (int c2 = 0; c2 < k; ++c2) {
      if (c2 == c) continue;
      const std::vector<int>& oth = members[static_cast<std::size_t>(c2)];
      if (oth.empty()) continue;

      double avg = 0.0;
      for (int j : oth) {
        avg += distEntry(i, j);
      }
      avg /= static_cast<double>(oth.size());
      b = std::min(b, avg);
    }

    if (!std::isfinite(b)) {
      silSum += 0.0;
      silCount++;
      continue;
    }

    const double denom = std::max(a, b);
    double s = 0.0;
    if (denom > 1.0e-12 && std::isfinite(denom)) {
      s = (b - a) / denom;
      if (!std::isfinite(s)) s = 0.0;
      s = std::clamp(s, -1.0, 1.0);
    }

    silSum += s;
    silCount++;
  }

  out.avgSilhouette = (silCount > 0) ? (silSum / static_cast<double>(silCount)) : 0.0;

  out.ok = true;

  // Warnings.
  if (k >= n) {
    out.warning = "k >= n (each point becomes its own cluster)";
  } else if (space == MineDiversityMode::Layout && k > 1 && n > 2) {
    // Nothing wrong, but layout-only clustering can be surprising if pHashes collide.
    out.warning.clear();
  }

  return out;
}

std::vector<int> MineClusteringMedoidIndices(const MineClusteringResult& res)
{
  std::vector<int> out;
  for (int id : res.medoidRecIndex) {
    if (id >= 0) out.push_back(id);
  }
  return out;
}

} // namespace isocity
