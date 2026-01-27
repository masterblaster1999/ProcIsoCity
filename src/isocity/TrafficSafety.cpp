// Copyright 2026
// SPDX-License-Identifier: MIT

#include "isocity/TrafficSafety.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/SkyView.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>

namespace isocity {

namespace {

inline float Clamp01(float v)
{
  return std::clamp(v, 0.0f, 1.0f);
}

float Percentile(std::vector<float> v, float q)
{
  if (v.empty()) return 0.0f;
  q = std::clamp(q, 0.0f, 1.0f);
  const std::size_t n = v.size();
  const std::size_t k = static_cast<std::size_t>(std::floor(q * static_cast<float>(n - 1)));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

inline std::size_t Idx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

// Integral image over a float grid.
// out has size (w+1)*(h+1) and uses the standard summed-area-table convention.
void BuildIntegralImage(const std::vector<float>& grid, int w, int h, std::vector<double>& out)
{
  out.assign(static_cast<std::size_t>(w + 1) * static_cast<std::size_t>(h + 1), 0.0);
  const int W = w + 1;

  for (int y = 0; y < h; ++y) {
    double row = 0.0;
    for (int x = 0; x < w; ++x) {
      row += static_cast<double>(grid[Idx(x, y, w)]);
      out[static_cast<std::size_t>(y + 1) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x + 1)] =
          out[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x + 1)] + row;
    }
  }
}

double BoxSum(const std::vector<double>& integral, int w, int h, int x0, int y0, int x1, int y1)
{
  // Clamp to valid inclusive coordinates.
  x0 = std::clamp(x0, 0, w - 1);
  x1 = std::clamp(x1, 0, w - 1);
  y0 = std::clamp(y0, 0, h - 1);
  y1 = std::clamp(y1, 0, h - 1);
  if (x1 < x0) std::swap(x0, x1);
  if (y1 < y0) std::swap(y0, y1);

  const int W = w + 1;
  const std::size_t A = static_cast<std::size_t>(y0) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x0);
  const std::size_t B = static_cast<std::size_t>(y0) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x1 + 1);
  const std::size_t C = static_cast<std::size_t>(y1 + 1) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x0);
  const std::size_t D = static_cast<std::size_t>(y1 + 1) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x1 + 1);

  return integral[D] - integral[B] - integral[C] + integral[A];
}

float Geometry01ForRoadTile(bool n, bool s, bool e, bool w)
{
  const int deg = (n ? 1 : 0) + (s ? 1 : 0) + (e ? 1 : 0) + (w ? 1 : 0);
  if (deg <= 0) return 0.20f;
  if (deg == 1) return 0.35f;
  if (deg == 2) {
    const bool straight = (n && s) || (e && w);
    return straight ? 0.45f : 0.65f;
  }
  if (deg == 3) return 0.85f;
  return 1.00f; // deg==4
}

} // namespace


TrafficSafetyResult ComputeTrafficSafety(const World& world, const TrafficSafetyConfig& cfg,
                                        const TrafficResult* traffic,
                                        const SkyViewResult* skyView,
                                        const std::vector<std::uint8_t>* precomputedRoadToEdge)
{
  TrafficSafetyResult out{};
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.risk01.assign(n, 0.0f);
  out.exposure01.assign(n, 0.0f);
  out.priority01.assign(n, 0.0f);

  if (!cfg.enabled) return out;

  // Road-to-edge connectivity mask.
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = precomputedRoadToEdge;
  if (cfg.requireOutsideConnection) {
    if (!roadToEdge || roadToEdge->size() != n) {
      ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
      roadToEdge = &roadToEdgeOwned;
    }
  }

  // Optional sky-view driven canyon confinement.
  SkyViewResult skyOwned{};
  const SkyViewResult* sky = skyView;
  if (cfg.canyonWeight > 0.0f) {
    const bool valid = (sky && sky->canyon01.size() == n);
    if (!valid) {
      SkyViewConfig vc{};
      vc.azimuthSamples = 16;
      vc.maxHorizonRadius = 64;
      vc.includeBuildings = true;
      skyOwned = ComputeSkyViewFactor(world, vc);
      sky = &skyOwned;
    }
  }

  // Traffic normalization.
  float trafficP = 0.0f;
  if (traffic && traffic->roadTraffic.size() == n) {
    std::vector<float> samples;
    samples.reserve(n / 4);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = Idx(x, y, w);
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;
        if (cfg.requireOutsideConnection && roadToEdge && (*roadToEdge)[i] == 0) continue;
        const float v = static_cast<float>(traffic->roadTraffic[i]);
        if (v <= 0.0f) continue;
        samples.push_back(v);
      }
    }
    trafficP = Percentile(std::move(samples), cfg.trafficPercentile);
  }
  if (trafficP <= 0.0f) trafficP = 1.0f;
  out.trafficPctl = trafficP;

  // Raw risk, later robust-scaled.
  std::vector<float> rawRisk(n, 0.0f);
  std::vector<float> rawSamples;
  rawSamples.reserve(n / 4);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;

      const std::size_t i = Idx(x, y, w);
      if (cfg.requireOutsideConnection && roadToEdge && (*roadToEdge)[i] == 0) continue;

      const auto isRoad = [&](int nx, int ny) -> bool {
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) return false;
        const std::size_t j = Idx(nx, ny, w);
        if (world.at(nx, ny).overlay != Overlay::Road) return false;
        if (cfg.requireOutsideConnection && roadToEdge && (*roadToEdge)[j] == 0) return false;
        return true;
      };

      const bool nR = isRoad(x, y - 1);
      const bool sR = isRoad(x, y + 1);
      const bool eR = isRoad(x + 1, y);
      const bool wR = isRoad(x - 1, y);
      const float geom01 = Geometry01ForRoadTile(nR, sR, eR, wR);

      float trafficTerm = 1.0f;
      if (traffic && traffic->roadTraffic.size() == n) {
        const float v = static_cast<float>(traffic->roadTraffic[i]);
        const float t01 = Clamp01(v / trafficP);
        trafficTerm = std::pow(t01, std::max(0.0f, cfg.trafficExponent));
      }

      const float canyon01 = (cfg.canyonWeight > 0.0f && sky && sky->canyon01.size() == n)
                                 ? Clamp01(sky->canyon01[i])
                                 : 0.0f;

      const float raw = trafficTerm *
                        (cfg.baseFactor + cfg.geometryWeight * geom01 + cfg.canyonWeight * canyon01);

      rawRisk[i] = raw;
      rawSamples.push_back(raw);
      out.roadTilesConsidered += 1;
    }
  }

  const float riskScale = std::max(1.0e-6f, Percentile(std::move(rawSamples), cfg.riskPercentile));
  out.riskScale = riskScale;
  for (std::size_t i = 0; i < n; ++i) {
    if (rawRisk[i] <= 0.0f) {
      out.risk01[i] = 0.0f;
    } else {
      out.risk01[i] = Clamp01(rawRisk[i] / riskScale);
    }
  }

  // Exposure = neighborhood average of risk01 (box filter).
  {
    const int r = std::max(0, cfg.exposureRadius);
    std::vector<double> integral;
    BuildIntegralImage(out.risk01, w, h, integral);

    std::vector<float> exposureRaw(n, 0.0f);
    std::vector<float> exposureSamples;
    exposureSamples.reserve(n);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int x0 = x - r;
        const int x1 = x + r;
        const int y0 = y - r;
        const int y1 = y + r;
        const double sum = BoxSum(integral, w, h, x0, y0, x1, y1);
        const int cx0 = std::clamp(x0, 0, w - 1);
        const int cx1 = std::clamp(x1, 0, w - 1);
        const int cy0 = std::clamp(y0, 0, h - 1);
        const int cy1 = std::clamp(y1, 0, h - 1);
        const double area = static_cast<double>((cx1 - cx0 + 1) * (cy1 - cy0 + 1));
        const float v = (area > 0.0) ? static_cast<float>(sum / area) : 0.0f;
        exposureRaw[Idx(x, y, w)] = v;
        exposureSamples.push_back(v);
      }
    }

    const float expScale = std::max(1.0e-6f, Percentile(std::move(exposureSamples), cfg.exposurePercentile));
    out.exposureScale = expScale;
    for (std::size_t i = 0; i < n; ++i) {
      out.exposure01[i] = (exposureRaw[i] <= 0.0f) ? 0.0f : Clamp01(exposureRaw[i] / expScale);
    }
  }

  // Priority = exposure01 * normalized residential population.
  {
    std::vector<float> popSamples;
    popSamples.reserve(n / 4);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Residential) continue;
        if (t.occupants <= 0) continue;
        popSamples.push_back(static_cast<float>(t.occupants));
      }
    }
    const float popP = std::max(1.0f, Percentile(std::move(popSamples), 0.95f));

    std::vector<float> priSamples;
    priSamples.reserve(n / 4);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const Tile& t = world.at(x, y);
        const std::size_t i = Idx(x, y, w);
        if (t.overlay != Overlay::Residential) {
          out.priority01[i] = 0.0f;
          continue;
        }

        const float pop01 = Clamp01(static_cast<float>(t.occupants) / popP);
        const float raw = out.exposure01[i] * pop01;
        out.priority01[i] = raw;
        priSamples.push_back(raw);
      }
    }

    const float priScale = std::max(1.0e-6f, Percentile(std::move(priSamples), cfg.priorityPercentile));
    out.priorityScale = priScale;
    for (std::size_t i = 0; i < n; ++i) {
      out.priority01[i] = (out.priority01[i] <= 0.0f) ? 0.0f : Clamp01(out.priority01[i] / priScale);
    }
  }

  // Summary stats.
  {
    double sumExp = 0.0;
    double sumPri = 0.0;
    int pop = 0;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Residential) continue;
        const int occ = static_cast<int>(t.occupants);
        if (occ == 0) continue;
        const std::size_t i = Idx(x, y, w);
        pop += occ;
        sumExp += static_cast<double>(out.exposure01[i]) * static_cast<double>(occ);
        sumPri += static_cast<double>(out.priority01[i]) * static_cast<double>(occ);
      }
    }
    out.residentPopulation = pop;
    if (pop > 0) {
      out.residentMeanExposure = static_cast<float>(sumExp / static_cast<double>(pop));
      out.residentMeanPriority = static_cast<float>(sumPri / static_cast<double>(pop));
    }
  }

  return out;
}

} // namespace isocity
