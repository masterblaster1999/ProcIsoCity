#include "isocity/Livability.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <utility>
#include <vector>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

struct WeightedSample {
  float v = 0.0f;
  int w = 0;
};

static float WeightedPercentile(std::vector<WeightedSample> samples, float q)
{
  if (samples.empty()) return 0.0f;
  q = Clamp01(q);

  // Remove zero/negative weights.
  samples.erase(std::remove_if(samples.begin(), samples.end(), [](const WeightedSample& s) { return s.w <= 0; }),
                samples.end());
  if (samples.empty()) return 0.0f;

  std::sort(samples.begin(), samples.end(), [](const WeightedSample& a, const WeightedSample& b) {
    if (a.v != b.v) return a.v < b.v;
    return a.w < b.w;
  });

  long long totalW = 0;
  for (const auto& s : samples) totalW += static_cast<long long>(s.w);
  if (totalW <= 0) return 0.0f;

  const double target = static_cast<double>(q) * static_cast<double>(totalW);
  long long accW = 0;
  for (const auto& s : samples) {
    accW += static_cast<long long>(s.w);
    if (static_cast<double>(accW) >= target) {
      return s.v;
    }
  }
  return samples.back().v;
}

static float WeightedGini(std::vector<WeightedSample> samples)
{
  // Gini for nonnegative values.
  samples.erase(std::remove_if(samples.begin(), samples.end(), [](const WeightedSample& s) {
                  return s.w <= 0 || !(s.v >= 0.0f);
                }),
                samples.end());
  if (samples.empty()) return 0.0f;

  std::sort(samples.begin(), samples.end(), [](const WeightedSample& a, const WeightedSample& b) {
    if (a.v != b.v) return a.v < b.v;
    return a.w < b.w;
  });

  double totalW = 0.0;
  double totalV = 0.0;
  for (const auto& s : samples) {
    totalW += static_cast<double>(s.w);
    totalV += static_cast<double>(s.w) * static_cast<double>(s.v);
  }
  if (totalW <= 0.0 || totalV <= 0.0) return 0.0f;

  // Area under the Lorenz curve.
  double cw = 0.0;
  double cv = 0.0;
  double prevP = 0.0;
  double prevQ = 0.0;
  double area = 0.0;

  for (const auto& s : samples) {
    cw += static_cast<double>(s.w);
    cv += static_cast<double>(s.w) * static_cast<double>(s.v);
    const double p = cw / totalW;
    const double q = cv / totalV;
    area += (q + prevQ) * 0.5 * (p - prevP);
    prevP = p;
    prevQ = q;
  }

  double g = 1.0 - 2.0 * area;
  if (!std::isfinite(g)) g = 0.0;
  g = std::clamp(g, 0.0, 1.0);
  return static_cast<float>(g);
}

static float HazardToComfort(float hazard01, float exponent)
{
  hazard01 = Clamp01(hazard01);
  exponent = std::max(0.01f, exponent);
  const float c = 1.0f - hazard01;
  return Clamp01(static_cast<float>(std::pow(static_cast<double>(c), static_cast<double>(exponent))));
}

} // namespace

LivabilityResult ComputeLivability(const World& world, const LivabilityConfig& cfgIn,
                                  const TrafficResult* traffic,
                                  const GoodsResult* goods)
{
  LivabilityResult out{};
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfgIn;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.livability01.assign(n, 0.0f);
  out.priority01.assign(n, 0.0f);

  // Normalize weights.
  float ws = std::max(0.0f, cfgIn.weightServices);
  float ww = std::max(0.0f, cfgIn.weightWalkability);
  float wa = std::max(0.0f, cfgIn.weightCleanAir);
  float wq = std::max(0.0f, cfgIn.weightQuiet);
  float wt = std::max(0.0f, cfgIn.weightThermalComfort);

  const float sumW = ws + ww + wa + wq + wt;
  if (sumW <= 1.0e-6f) {
    ws = ww = wa = wq = wt = 1.0f;
  }
  const float invSumW = 1.0f / (ws + ww + wa + wq + wt);
  ws *= invSumW;
  ww *= invSumW;
  wa *= invSumW;
  wq *= invSumW;
  wt *= invSumW;

  // --- Compute component fields ---

  // Services.
  ServicesModelSettings svcCfg{};
  svcCfg.enabled = true;
  svcCfg.requireOutsideConnection = cfgIn.requireOutsideConnection;
  svcCfg.weightMode = cfgIn.weightMode;
  svcCfg.catchmentRadiusSteps = cfgIn.servicesCatchmentRadiusSteps;

  const std::vector<ServiceFacility> facilities = ExtractServiceFacilitiesFromWorld(world);
  const ServicesResult services = ComputeServices(world, svcCfg, facilities);

  // Walkability.
  WalkabilityConfig wc{};
  wc.enabled = true;
  wc.requireOutsideConnection = cfgIn.requireOutsideConnection;
  wc.weightMode = cfgIn.weightMode;
  wc.coverageThresholdSteps = cfgIn.walkCoverageThresholdSteps;

  const WalkabilityResult walkability = ComputeWalkability(world, wc);

  // Hazards.
  NoiseConfig nc{};
  const NoiseResult noise = ComputeNoisePollution(world, nc, traffic, goods);

  HeatIslandConfig hc{};
  const HeatIslandResult heat = ComputeHeatIsland(world, hc, traffic, goods);

  AirPollutionConfig ac{};
  // Keep the wind stable for a given world seed.
  ac.windFromSeed = true;
  const AirPollutionResult air = ComputeAirPollution(world, ac, traffic, goods);

  // Occupant normalization for priority.
  const int occScale = std::max(1, cfgIn.priorityOccupantScale);
  const float occExp = std::max(0.0f, cfgIn.priorityOccupantExponent);
  const float needExp = std::max(0.0f, cfgIn.priorityNeedExponent);

  // Resident summary accumulation.
  double sumLiv = 0.0;
  double sumSvc = 0.0;
  double sumWalk = 0.0;
  double sumAir = 0.0;
  double sumQuiet = 0.0;
  double sumTherm = 0.0;

  std::vector<WeightedSample> residentSamples;
  residentSamples.reserve(256);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);

      const float svc01 = (services.overall.size() == n) ? Clamp01(services.overall[i]) : 0.0f;
      const float walk01 = (walkability.overall01.size() == n) ? Clamp01(walkability.overall01[i]) : 0.0f;

      const float cleanAir01 = (air.pollution01.size() == n)
                                  ? HazardToComfort(air.pollution01[i], cfgIn.hazardComfortExponent)
                                  : 0.0f;
      const float quiet01 = (noise.noise01.size() == n)
                                ? HazardToComfort(noise.noise01[i], cfgIn.hazardComfortExponent)
                                : 0.0f;
      const float thermal01 = (heat.heat01.size() == n)
                                  ? HazardToComfort(heat.heat01[i], cfgIn.hazardComfortExponent)
                                  : 0.0f;

      const float liv = Clamp01(ws * svc01 + ww * walk01 + wa * cleanAir01 + wq * quiet01 + wt * thermal01);
      out.livability01[i] = liv;
      out.maxLivability01 = std::max(out.maxLivability01, liv);

      const Tile& t = world.at(x, y);
      const int occ = static_cast<int>(t.occupants);

      float pop01 = Clamp01(static_cast<float>(occ) / static_cast<float>(occScale));
      if (occExp != 1.0f) {
        pop01 = Clamp01(static_cast<float>(std::pow(static_cast<double>(pop01), static_cast<double>(occExp))));
      }

      float need01 = 1.0f - liv;
      if (needExp != 1.0f) {
        need01 = Clamp01(static_cast<float>(std::pow(static_cast<double>(need01), static_cast<double>(needExp))));
      }

      const float pr = Clamp01(need01 * pop01);
      out.priority01[i] = pr;
      out.maxPriority01 = std::max(out.maxPriority01, pr);

      // Resident-only summary (residential tiles with occupants).
      if (t.overlay == Overlay::Residential && occ > 0) {
        out.residentTileCount++;
        out.residentPopulation += occ;

        sumLiv += static_cast<double>(liv) * static_cast<double>(occ);
        sumSvc += static_cast<double>(svc01) * static_cast<double>(occ);
        sumWalk += static_cast<double>(walk01) * static_cast<double>(occ);
        sumAir += static_cast<double>(cleanAir01) * static_cast<double>(occ);
        sumQuiet += static_cast<double>(quiet01) * static_cast<double>(occ);
        sumTherm += static_cast<double>(thermal01) * static_cast<double>(occ);

        residentSamples.push_back(WeightedSample{liv, occ});
      }
    }
  }

  if (out.residentPopulation > 0) {
    const double invPop = 1.0 / static_cast<double>(out.residentPopulation);
    out.residentMeanLivability01 = static_cast<float>(sumLiv * invPop);
    out.residentMeanServices01 = static_cast<float>(sumSvc * invPop);
    out.residentMeanWalkability01 = static_cast<float>(sumWalk * invPop);
    out.residentMeanCleanAir01 = static_cast<float>(sumAir * invPop);
    out.residentMeanQuiet01 = static_cast<float>(sumQuiet * invPop);
    out.residentMeanThermalComfort01 = static_cast<float>(sumTherm * invPop);

    out.residentP10 = WeightedPercentile(residentSamples, 0.10f);
    out.residentMedian = WeightedPercentile(residentSamples, 0.50f);
    out.residentP90 = WeightedPercentile(residentSamples, 0.90f);
    out.residentGini = WeightedGini(residentSamples);
  }

  return out;
}

} // namespace isocity
