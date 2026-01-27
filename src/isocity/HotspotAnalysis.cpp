#include "isocity/HotspotAnalysis.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace isocity {

const char* HotspotClassName(HotspotClass c)
{
  switch (c) {
  case HotspotClass::Cold: return "cold";
  case HotspotClass::Neutral: return "neutral";
  case HotspotClass::Hot: return "hot";
  default: return "unknown";
  }
}

namespace {

inline float Clamp01(float v)
{
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

inline std::size_t PIdx(int x, int y, int pw)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(pw) + static_cast<std::size_t>(x);
}

// Build a summed-area table (integral image) for the value field and the mask count.
// Prefix arrays have shape (h+1) x (w+1).
static void BuildPrefixSums(int w, int h,
                            const std::vector<float>& field,
                            const std::vector<std::uint8_t>* validMask,
                            std::vector<double>& outSum,
                            std::vector<int>& outCount)
{
  const int pw = w + 1;
  const int ph = h + 1;
  const std::size_t pn = static_cast<std::size_t>(pw) * static_cast<std::size_t>(ph);

  outSum.assign(pn, 0.0);
  outCount.assign(pn, 0);

  if (w <= 0 || h <= 0) return;

  // Row-wise accumulation; then add previous rows.
  for (int y = 0; y < h; ++y) {
    double rowSum = 0.0;
    int rowCount = 0;
    const int py = y + 1;

    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const bool valid = (!validMask) ? true : ((*validMask)[i] != 0);

      if (valid) {
        rowSum += static_cast<double>(field[i]);
        rowCount += 1;
      }

      const int px = x + 1;
      const std::size_t p = PIdx(px, py, pw);
      const std::size_t pAbove = PIdx(px, py - 1, pw);
      outSum[p] = outSum[pAbove] + rowSum;
      outCount[p] = outCount[pAbove] + rowCount;
    }
  }
}

inline double RectSum(const std::vector<double>& p, int pw, int x0, int y0, int x1, int y1)
{
  // Inclusive rectangle query on prefix arrays.
  const int xa = x0;
  const int xb = x1 + 1;
  const int ya = y0;
  const int yb = y1 + 1;

  const double A = p[PIdx(xa, ya, pw)];
  const double B = p[PIdx(xb, ya, pw)];
  const double C = p[PIdx(xa, yb, pw)];
  const double D = p[PIdx(xb, yb, pw)];

  return (D - B - C + A);
}

inline int RectSum(const std::vector<int>& p, int pw, int x0, int y0, int x1, int y1)
{
  const int xa = x0;
  const int xb = x1 + 1;
  const int ya = y0;
  const int yb = y1 + 1;

  const int A = p[PIdx(xa, ya, pw)];
  const int B = p[PIdx(xb, ya, pw)];
  const int C = p[PIdx(xa, yb, pw)];
  const int D = p[PIdx(xb, yb, pw)];

  return (D - B - C + A);
}

} // namespace

HotspotResult ComputeHotspotsGiStar(int w, int h,
                                   const std::vector<float>& field,
                                   const std::vector<std::uint8_t>* validMask,
                                   const HotspotConfig& cfg)
{
  HotspotResult out{};
  out.w = std::max(0, w);
  out.h = std::max(0, h);
  out.cfg = cfg;

  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  if (!cfg.enabled || w <= 0 || h <= 0 || field.size() != n) {
    return out;
  }
  if (validMask && validMask->size() != n) {
    // Ignore an invalid mask size.
    validMask = nullptr;
  }

  // Pre-size outputs with neutral defaults.
  out.z.assign(n, 0.0f);
  out.z01.assign(n, 0.5f);
  out.cls.assign(n, static_cast<std::uint8_t>(HotspotClass::Neutral));

  // Global mean + standard deviation over valid tiles.
  double sum = 0.0;
  double sumSq = 0.0;
  int count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (validMask && (*validMask)[i] == 0) continue;
    const double v = static_cast<double>(field[i]);
    sum += v;
    sumSq += v * v;
    ++count;
  }

  out.validCount = count;
  if (count <= 1) {
    return out;
  }

  const double mean = sum / static_cast<double>(count);
  const double var = std::max(0.0, (sumSq / static_cast<double>(count)) - mean * mean);
  const double stdev = std::sqrt(var);

  out.mean = static_cast<float>(mean);
  out.stdev = static_cast<float>(stdev);

  if (!(stdev > 0.0) || !std::isfinite(stdev)) {
    return out;
  }

  // Integral images for fast window sums.
  std::vector<double> prefixSum;
  std::vector<int> prefixCount;
  BuildPrefixSums(w, h, field, validMask, prefixSum, prefixCount);

  const int pw = w + 1;
  const int radius = std::max(0, cfg.radius);
  const double denomScale = stdev;
  const float zScale = (cfg.zScale > 1.0e-6f) ? cfg.zScale : 3.0f;

  int hot = 0;
  int cold = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (validMask && (*validMask)[i] == 0) {
        // leave neutral
        continue;
      }

      const int x0 = std::max(0, x - radius);
      const int x1 = std::min(w - 1, x + radius);
      const int y0 = std::max(0, y - radius);
      const int y1 = std::min(h - 1, y + radius);

      const double sumX = RectSum(prefixSum, pw, x0, y0, x1, y1);
      const int sumW = RectSum(prefixCount, pw, x0, y0, x1, y1);

      if (sumW <= 0) {
        continue;
      }

      // Binary weights => sumW2 == sumW.
      const double sumW2 = static_cast<double>(sumW);
      const double sumW_d = static_cast<double>(sumW);

      // Gi* denominator term: S * sqrt((n*sumW2 - sumW^2) / (n-1)).
      const double denomTerm = (static_cast<double>(count) * sumW2 - (sumW_d * sumW_d)) / static_cast<double>(count - 1);

      if (!(denomTerm > 0.0) || !std::isfinite(denomTerm)) {
        continue;
      }

      const double denom = denomScale * std::sqrt(denomTerm);
      if (!(denom > 0.0) || !std::isfinite(denom)) {
        continue;
      }

      const double num = sumX - mean * sumW_d;
      const double z = num / denom;

      const float zf = static_cast<float>(std::isfinite(z) ? z : 0.0);
      out.z[i] = zf;

      // Map to 0..1 for visualization.
      const float z01 = Clamp01(0.5f + 0.5f * static_cast<float>(std::tanh(static_cast<double>(zf) / static_cast<double>(zScale))));
      out.z01[i] = z01;

      HotspotClass cls = HotspotClass::Neutral;
      if (zf >= cfg.zThreshold) {
        cls = HotspotClass::Hot;
        ++hot;
      } else if (zf <= -cfg.zThreshold) {
        cls = HotspotClass::Cold;
        ++cold;
      }
      out.cls[i] = static_cast<std::uint8_t>(cls);
    }
  }

  out.hotCount = hot;
  out.coldCount = cold;

  return out;
}

HotspotResult ComputeHotspotsGiStar(const World& world,
                                   const std::vector<float>& field,
                                   const HotspotConfig& cfg)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  if (w <= 0 || h <= 0 || field.size() != n) {
    HotspotResult out{};
    out.w = std::max(0, w);
    out.h = std::max(0, h);
    out.cfg = cfg;
    return out;
  }

  if (!cfg.excludeWater) {
    return ComputeHotspotsGiStar(w, h, field, nullptr, cfg);
  }

  std::vector<std::uint8_t> mask(n, 1);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (world.at(x, y).terrain == Terrain::Water) {
        mask[i] = 0;
      }
    }
  }

  return ComputeHotspotsGiStar(w, h, field, &mask, cfg);
}

} // namespace isocity
