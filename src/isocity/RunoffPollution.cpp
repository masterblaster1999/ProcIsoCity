#include "isocity/RunoffPollution.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool IsCivic(Overlay o)
{
  return (o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation);
}

} // namespace

RunoffPollutionResult ComputeRunoffPollution(const World& world, const RunoffPollutionConfig& cfg, const TrafficResult* traffic)
{
  RunoffPollutionResult out{};

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.w = w;
  out.h = h;
  out.cfg = cfg;
  out.flowAccum.assign(n, 1);
  out.localLoad.assign(n, 0.0f);
  out.localLoad01.assign(n, 0.0f);
  out.concentration.assign(n, 0.0f);
  out.pollution01.assign(n, 0.0f);

  // Build a heightfield for hydrology.
  std::vector<float> heights(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      heights[FlatIdx(x, y, w)] = world.at(x, y).height;
    }
  }

  const HydrologyField hydro = BuildHydrologyField(heights, w, h);
  if (hydro.accum.size() == n) {
    out.flowAccum = hydro.accum;
    out.maxFlowAccum = hydro.maxAccum;
  }

  // Normalize traffic if provided.
  std::uint16_t maxTraffic = 0;
  if (traffic && traffic->roadTraffic.size() == n) {
    maxTraffic = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (maxTraffic == 0) {
      for (std::uint16_t v : traffic->roadTraffic) maxTraffic = std::max(maxTraffic, v);
    }
  }

  const float clampAbs = std::max(0.01f, cfg.clampLoad);
  const float occScale = static_cast<float>(std::max(1, cfg.occupantScale));

  // --- local load field ---
  float maxLoad = 0.0f;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float l = 0.0f;

      switch (t.overlay) {
      case Overlay::Road: {
        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        l += cfg.roadBase + cfg.roadClassBoost * static_cast<float>(lvl - 1);

        float tr01 = 0.0f;
        if (maxTraffic > 0 && traffic && traffic->roadTraffic.size() == n) {
          tr01 = static_cast<float>(traffic->roadTraffic[i]) / static_cast<float>(maxTraffic);
        } else {
          tr01 = cfg.fallbackCommuteTraffic01;
        }
        l += cfg.roadTrafficBoost * Clamp01(tr01);
      } break;

      case Overlay::Residential:
        l += cfg.residentialLoad;
        break;
      case Overlay::Commercial:
        l += cfg.commercialLoad;
        break;
      case Overlay::Industrial:
        l += cfg.industrialLoad;
        break;
      default:
        if (IsCivic(t.overlay)) {
          l += cfg.civicLoad;
        }
        break;
      }

      if (t.occupants > 0) {
        const float occ01 = Clamp01(static_cast<float>(t.occupants) / occScale);
        l += cfg.occupantBoost * occ01;
      }

      l = std::clamp(l, 0.0f, clampAbs);
      out.localLoad[i] = l;
      maxLoad = std::max(maxLoad, l);
    }
  }
  out.maxLocalLoad = maxLoad;

  // --- route downhill ---
  std::vector<float> mass = out.localLoad;

  // Sort indices by height descending (higher routes into lower).
  std::vector<int> order;
  order.reserve(n);
  for (int i = 0; i < w * h; ++i) order.push_back(i);

  std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
    const float ha = heights[static_cast<std::size_t>(a)];
    const float hb = heights[static_cast<std::size_t>(b)];
    if (ha == hb) return a < b;
    return ha > hb;
  });

  const float dilExp = cfg.dilutionExponent;
  float maxConc = 0.0f;

  for (int iLin : order) {
    const std::size_t i = static_cast<std::size_t>(iLin);
    const int x = (w > 0) ? (iLin % w) : 0;
    const int y = (w > 0) ? (iLin / w) : 0;

    const Tile& t = world.at(x, y);

    // Compute retention (fraction of mass removed at this tile).
    float retain = 0.0f;
    if (t.terrain == Terrain::Water) {
      retain = cfg.waterIsSink ? cfg.filterWater : 0.0f;
    } else {
      if (t.overlay == Overlay::Park) retain += cfg.filterPark;
      if (t.terrain == Terrain::Grass) retain += cfg.filterGrass;
      if (t.terrain == Terrain::Sand) retain += cfg.filterSand;
      if (t.overlay == Overlay::Road) retain += cfg.filterRoad;
    }
    retain = std::clamp(retain, 0.0f, 1.0f);

    const float m = mass[i];
    const float outflow = m * (1.0f - retain);

    // Dilution proxy using flow accumulation.
    float denom = 1.0f;
    if (dilExp != 0.0f && out.flowAccum.size() == n) {
      const int a = std::max(1, out.flowAccum[i]);
      denom = std::pow(static_cast<float>(a), dilExp);
      if (!std::isfinite(denom) || denom <= 0.0f) denom = 1.0f;
    }

    const float conc = outflow / denom;
    out.concentration[i] = conc;
    maxConc = std::max(maxConc, conc);

    // Route remaining mass to downstream neighbor.
    if (hydro.dir.size() == n) {
      const int to = hydro.dir[i];
      if (to >= 0 && to < w * h) {
        mass[static_cast<std::size_t>(to)] += outflow;
      }
    }
  }

  out.maxConcentration = maxConc;

  // --- normalize fields ---
  for (std::size_t i = 0; i < n; ++i) {
    if (maxLoad > 0.0f) out.localLoad01[i] = Clamp01(out.localLoad[i] / maxLoad);
    if (maxConc > 0.0f) out.pollution01[i] = Clamp01(out.concentration[i] / maxConc);
  }

  // --- exposure summary (residential-weighted) ---
  int resTiles = 0;
  int pop = 0;
  double sum = 0.0;
  int highPop = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants == 0) continue;

      resTiles += 1;
      pop += static_cast<int>(t.occupants);

      const float p01 = out.pollution01[i];
      sum += static_cast<double>(p01) * static_cast<double>(t.occupants);
      if (p01 >= cfg.highExposureThreshold01) {
        highPop += static_cast<int>(t.occupants);
      }
    }
  }

  out.residentialTileCount = resTiles;
  out.residentPopulation = pop;
  if (pop > 0) {
    out.residentAvgPollution01 = static_cast<float>(sum / static_cast<double>(pop));
    out.residentHighExposureFrac = static_cast<float>(static_cast<double>(highPop) / static_cast<double>(pop));
  }

  return out;
}

} // namespace isocity
