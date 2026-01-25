#include "isocity/MineNeighbors.hpp"

#include "isocity/PerceptualHash.hpp"
#include "isocity/VPTree.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace isocity {

namespace {

static std::vector<MineMetric> DefaultNeighborMetrics()
{
  // A compact but expressive behavior vector spanning macro KPIs and physical layout.
  // Matches the spirit of MineClustering defaults.
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

} // namespace

MineNeighborsResult ComputeMineNeighborsKNN(const std::vector<MineRecord>& recs,
                                           const std::vector<int>& selectedIndices,
                                           const MineNeighborsConfig& cfg)
{
  MineNeighborsResult out;
  out.cfg = cfg;
  out.selectedIndices = selectedIndices;

  const int n = static_cast<int>(selectedIndices.size());
  if (n <= 0) {
    out.ok = false;
    out.warning = "no selected indices";
    return out;
  }

  int k = cfg.k;
  if (k < 0) k = 0;
  k = std::min(k, std::max(0, n - 1));
  out.cfg.k = k;

  out.neighbors.assign(static_cast<std::size_t>(n), {});
  out.distances.assign(static_cast<std::size_t>(n), {});

  if (k == 0 || n == 1) {
    out.ok = true;
    return out;
  }

  const MineDiversityMode space = cfg.space;
  const double lw = std::clamp(cfg.layoutWeight, 0.0, 1.0);

  // Resolve metrics for scalar/hybrid spaces.
  std::vector<MineMetric> metrics = cfg.metrics;
  if ((space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) && metrics.empty()) {
    metrics = DefaultNeighborMetrics();
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
    // aEntry/bEntry are indices into selectedIndices (0..n-1).
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

  // Build a VP-tree over entry ids 0..n-1 for efficient deterministic kNN.
  std::vector<int> ids;
  ids.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) ids.push_back(i);

  VPTree<decltype(distEntry)> tree(std::move(ids), distEntry);

  for (int i = 0; i < n; ++i) {
    const std::vector<std::pair<double, int>> knn = tree.kNearest(i, k);
    out.neighbors[static_cast<std::size_t>(i)].reserve(knn.size());
    out.distances[static_cast<std::size_t>(i)].reserve(knn.size());
    for (const auto& pr : knn) {
      out.distances[static_cast<std::size_t>(i)].push_back(pr.first);
      out.neighbors[static_cast<std::size_t>(i)].push_back(pr.second);
    }
  }

  out.ok = true;
  return out;
}

} // namespace isocity
