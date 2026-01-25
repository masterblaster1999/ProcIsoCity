#include "isocity/MineEmbedding.hpp"

#include "isocity/PerceptualHash.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

namespace isocity {

namespace {

static std::vector<MineMetric> DefaultEmbeddingMetrics()
{
  // A compact, fairly stable set of KPIs that tends to produce a meaningful
  // geometry in practice.
  return {
      MineMetric::Population,
      MineMetric::Happiness,
      MineMetric::AvgLandValue,
      MineMetric::TrafficCongestion,
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

static double Dot(const std::vector<double>& a, const std::vector<double>& b)
{
  const std::size_t n = std::min(a.size(), b.size());
  double s = 0.0;
  for (std::size_t i = 0; i < n; ++i) s += a[i] * b[i];
  return s;
}

static double Norm(const std::vector<double>& a)
{
  return std::sqrt(Dot(a, a));
}

static void Normalize(std::vector<double>& a)
{
  const double n = Norm(a);
  if (!(n > 1.0e-30) || !std::isfinite(n)) {
    return;
  }
  for (double& v : a) v /= n;
}

static void MulMatVec(const std::vector<double>& M, int n, const std::vector<double>& v, std::vector<double>& out)
{
  out.assign(static_cast<std::size_t>(n), 0.0);
  if (n <= 0) return;
  for (int i = 0; i < n; ++i) {
    const std::size_t row = static_cast<std::size_t>(i) * static_cast<std::size_t>(n);
    double s = 0.0;
    for (int j = 0; j < n; ++j) {
      s += M[row + static_cast<std::size_t>(j)] * v[static_cast<std::size_t>(j)];
    }
    out[static_cast<std::size_t>(i)] = s;
  }
}

static void OrthogonalizeAgainst(std::vector<double>& v, const std::vector<double>& basis)
{
  if (basis.empty() || v.empty()) return;
  const double d = Dot(v, basis);
  for (std::size_t i = 0; i < v.size() && i < basis.size(); ++i) {
    v[i] -= d * basis[i];
  }
}

static double GershgorinLowerBound(const std::vector<double>& A, int n)
{
  // Gershgorin circle theorem gives a cheap bound on the minimum eigenvalue:
  //   lambda_min >= min_i (a_ii - sum_{j!=i} |a_ij|)
  // For symmetric matrices, this is a reasonable conservative bound.
  if (n <= 0) return 0.0;
  double lb = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const std::size_t row = static_cast<std::size_t>(i) * static_cast<std::size_t>(n);
    double radius = 0.0;
    for (int j = 0; j < n; ++j) {
      if (j == i) continue;
      radius += std::fabs(A[row + static_cast<std::size_t>(j)]);
    }
    const double center = A[row + static_cast<std::size_t>(i)];
    lb = std::min(lb, center - radius);
  }
  if (!std::isfinite(lb)) return 0.0;
  return lb;
}

static bool PowerIterationTop2(const std::vector<double>& A, int n, int iters,
                              std::vector<double>& outV1, double& outL1,
                              std::vector<double>& outV2, double& outL2)
{
  outV1.assign(static_cast<std::size_t>(n), 0.0);
  outV2.assign(static_cast<std::size_t>(n), 0.0);
  outL1 = 0.0;
  outL2 = 0.0;
  if (n <= 0) return false;

  iters = std::max(1, iters);

  // Deterministic initial vectors.
  for (int i = 0; i < n; ++i) {
    outV1[static_cast<std::size_t>(i)] = (i & 1) ? -1.0 : 1.0;
    outV2[static_cast<std::size_t>(i)] = (i % 3 == 0) ? 1.0 : -1.0;
  }
  Normalize(outV1);

  std::vector<double> tmp;

  // Power iteration converges to the eigenvalue with largest *magnitude*.
  // Classical MDS wants the largest *algebraic* (most positive) eigenvalues.
  // We therefore shift the matrix by +shift*I so all eigenvalues are positive,
  // which makes "largest magnitude" coincide with "largest value".
  const double lb = GershgorinLowerBound(A, n);
  const double shift = (lb < 0.0) ? (-lb + 1.0e-6) : 0.0;

  // First eigenvector.
  for (int k = 0; k < iters; ++k) {
    MulMatVec(A, n, outV1, tmp);
    if (shift != 0.0) {
      for (int i = 0; i < n; ++i) tmp[static_cast<std::size_t>(i)] += shift * outV1[static_cast<std::size_t>(i)];
    }
    if (!(Norm(tmp) > 1.0e-30)) break;
    outV1 = tmp;
    Normalize(outV1);
  }
  MulMatVec(A, n, outV1, tmp);
  outL1 = Dot(outV1, tmp);

  // Second eigenvector (orthogonal to first).
  OrthogonalizeAgainst(outV2, outV1);
  Normalize(outV2);

  for (int k = 0; k < iters; ++k) {
    MulMatVec(A, n, outV2, tmp);
    if (shift != 0.0) {
      for (int i = 0; i < n; ++i) tmp[static_cast<std::size_t>(i)] += shift * outV2[static_cast<std::size_t>(i)];
    }
    OrthogonalizeAgainst(tmp, outV1);
    if (!(Norm(tmp) > 1.0e-30)) break;
    outV2 = tmp;
    Normalize(outV2);
  }
  MulMatVec(A, n, outV2, tmp);
  outL2 = Dot(outV2, tmp);

  return true;
}

} // namespace

MineEmbeddingResult ComputeMineEmbeddingMDS(const std::vector<MineRecord>& recs,
                                           const std::vector<int>& selectedIndices,
                                           const MineEmbeddingConfig& cfg)
{
  MineEmbeddingResult out;
  out.cfg = cfg;

  const int n = static_cast<int>(selectedIndices.size());
  if (n <= 1) {
    out.ok = false;
    out.warning = "Need at least 2 selected seeds for embedding";
    return out;
  }

  for (int idx : selectedIndices) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= recs.size()) {
      out.ok = false;
      out.warning = "Selected index out of range";
      return out;
    }
  }

  const MineDiversityMode space = cfg.space;
  const double lw = std::clamp(cfg.layoutWeight, 0.0, 1.0);

  // Resolve metrics and precompute standardized scalar feature vectors when needed.
  std::vector<MineMetric> metrics = cfg.metrics;
  if ((space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) && metrics.empty()) {
    metrics = DefaultEmbeddingMetrics();
  }

  const int dim = static_cast<int>(metrics.size());
  std::vector<double> feats;
  if (space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) {
    std::vector<double> center;
    std::vector<double> scale;
    FitStandardizer(recs, selectedIndices, metrics, cfg.robustScaling, center, scale);

    feats.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(std::max(0, dim)), 0.0);

    for (int i = 0; i < n; ++i) {
      const int ridx = selectedIndices[static_cast<std::size_t>(i)];
      const MineRecord& r = recs[static_cast<std::size_t>(ridx)];
      for (int j = 0; j < dim; ++j) {
        const double v = MineMetricValue(r, metrics[static_cast<std::size_t>(j)]);
        const double c = center[static_cast<std::size_t>(j)];
        const double s = scale[static_cast<std::size_t>(j)];
        feats[static_cast<std::size_t>(i) * static_cast<std::size_t>(dim) + static_cast<std::size_t>(j)] = (std::isfinite(v) ? (v - c) : (0.0 - c)) / s;
      }
    }
  }

  auto distFn = [&](int a, int b) -> double {
    // a,b are indices in [0,n)
    const int ra = selectedIndices[static_cast<std::size_t>(a)];
    const int rb = selectedIndices[static_cast<std::size_t>(b)];
    const MineRecord& A = recs[static_cast<std::size_t>(ra)];
    const MineRecord& B = recs[static_cast<std::size_t>(rb)];

    const int hd = HammingDistance64(A.overlayPHash, B.overlayPHash);
    const double dl = static_cast<double>(hd) / 64.0;

    if (space == MineDiversityMode::Layout) return dl;

    const double ds = ScalarDistance(feats, dim, a, b);

    if (space == MineDiversityMode::Scalar) return ds;

    // Hybrid.
    return (1.0 - lw) * ds + lw * dl;
  };

  // Build squared distance matrix.
  std::vector<double> D2;
  D2.assign(static_cast<std::size_t>(n) * static_cast<std::size_t>(n), 0.0);

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const double d = distFn(i, j);
      const double v = d * d;
      D2[static_cast<std::size_t>(i) * static_cast<std::size_t>(n) + static_cast<std::size_t>(j)] = v;
      D2[static_cast<std::size_t>(j) * static_cast<std::size_t>(n) + static_cast<std::size_t>(i)] = v;
    }
  }

  // Row means and total mean.
  std::vector<double> rowMean(static_cast<std::size_t>(n), 0.0);
  for (int i = 0; i < n; ++i) {
    double s = 0.0;
    const std::size_t row = static_cast<std::size_t>(i) * static_cast<std::size_t>(n);
    for (int j = 0; j < n; ++j) s += D2[row + static_cast<std::size_t>(j)];
    rowMean[static_cast<std::size_t>(i)] = s / static_cast<double>(n);
  }

  const double totalMean = std::accumulate(rowMean.begin(), rowMean.end(), 0.0) / static_cast<double>(n);

  // Double-centered Gram matrix B.
  std::vector<double> B;
  B.assign(static_cast<std::size_t>(n) * static_cast<std::size_t>(n), 0.0);
  for (int i = 0; i < n; ++i) {
    const std::size_t row = static_cast<std::size_t>(i) * static_cast<std::size_t>(n);
    for (int j = 0; j < n; ++j) {
      const double v = D2[row + static_cast<std::size_t>(j)]
          - rowMean[static_cast<std::size_t>(i)]
          - rowMean[static_cast<std::size_t>(j)]
          + totalMean;
      B[row + static_cast<std::size_t>(j)] = -0.5 * v;
    }
  }

  std::vector<double> v1, v2;
  double l1 = 0.0, l2 = 0.0;
  if (!PowerIterationTop2(B, n, cfg.powerIters, v1, l1, v2, l2)) {
    out.ok = false;
    out.warning = "Failed to compute eigenvectors";
    return out;
  }

  out.eigen1 = l1;
  out.eigen2 = l2;

  const double s1 = (l1 > 0.0 && std::isfinite(l1)) ? std::sqrt(l1) : 0.0;
  const double s2 = (l2 > 0.0 && std::isfinite(l2)) ? std::sqrt(l2) : 0.0;

  out.points.resize(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    MineEmbeddingPoint p;
    p.recIndex = selectedIndices[static_cast<std::size_t>(i)];
    p.x = v1[static_cast<std::size_t>(i)] * s1;
    p.y = v2[static_cast<std::size_t>(i)] * s2;
    out.points[static_cast<std::size_t>(i)] = p;
  }

  out.ok = true;
  if (!(l1 > 0.0)) {
    out.warning = "Embedding eigenvalues are non-positive; the chosen distance may be highly non-Euclidean.";
  }

  return out;
}

} // namespace isocity
