#include "isocity/CrimeModel.hpp"

#include "isocity/JobOpportunity.hpp"
#include "isocity/NoisePollution.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace isocity {
namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

float PercentileInt(std::vector<int> samples, float q)
{
  if (samples.empty()) return 0.0f;
  q = std::clamp(q, 0.0f, 1.0f);
  const std::size_t k = static_cast<std::size_t>(std::lround(q * static_cast<float>(samples.size() - 1)));
  std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(k), samples.end());
  return static_cast<float>(samples[k]);
}

// Pick an adjacent road tile index for a facility tile.
// Returns -1 if none.
int PickAdjacentRoad(const World& world, int x, int y, const std::vector<std::uint8_t>* roadToEdgeMask)
{
  const int w = world.width();
  const int h = world.height();
  auto okRoad = [&](int xx, int yy) -> int {
    if (xx < 0 || yy < 0 || xx >= w || yy >= h) return -1;
    const Tile& t = world.at(xx, yy);
    if (t.overlay != Overlay::Road) return -1;
    const int idx = yy * w + xx;
    if (roadToEdgeMask && static_cast<std::size_t>(idx) < roadToEdgeMask->size()) {
      if ((*roadToEdgeMask)[static_cast<std::size_t>(idx)] == 0) return -1;
    }
    return idx;
  };

  // Deterministic order.
  if (int a = okRoad(x, y - 1); a >= 0) return a;
  if (int a = okRoad(x + 1, y); a >= 0) return a;
  if (int a = okRoad(x, y + 1); a >= 0) return a;
  if (int a = okRoad(x - 1, y); a >= 0) return a;
  return -1;
}

} // namespace

CrimeModelResult ComputeCrimeModel(const World& world, const CrimeModelConfig& cfg,
                                  const TrafficResult* traffic, const GoodsResult* goods,
                                  const JobOpportunityResult* jobs, const NoiseResult* noise,
                                  const std::vector<std::uint8_t>* precomputedRoadToEdge,
                                  const ZoneAccessMap* precomputedZoneAccess)
{
  CrimeModelResult out{};
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  out.w = w;
  out.h = h;
  out.cfg = cfg;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.policeCostMilli.assign(n, -1);
  out.policeAccess01.assign(n, 0.0f);
  out.risk01.assign(n, 0.0f);

  if (!cfg.enabled) return out;

  // Outside-connection mask (optional).
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = precomputedRoadToEdge;
  if (cfg.requireOutsideConnection && (!roadToEdge || roadToEdge->size() != n)) {
    ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
    roadToEdge = &roadToEdgeOwned;
  }

  // Zone access map (optional).
  ZoneAccessMap zoneAccessOwned;
  const ZoneAccessMap* zoneAccess = precomputedZoneAccess;
  if (!zoneAccess || zoneAccess->roadIdx.size() != n) {
    zoneAccessOwned = BuildZoneAccessMap(world, roadToEdge);
    zoneAccess = &zoneAccessOwned;
  }

  // --- Collect police station sources ---
  std::vector<int> policeSourceRoadIdx;
  policeSourceRoadIdx.reserve(n / 64);
  std::vector<std::uint8_t> used;
  used.assign(n, 0);

  int stations = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::PoliceStation) continue;
      ++stations;
      const int ridx = PickAdjacentRoad(world, x, y, roadToEdge);
      if (ridx < 0) continue;
      if (static_cast<std::size_t>(ridx) >= n) continue;
      if (used[static_cast<std::size_t>(ridx)] != 0) continue;
      used[static_cast<std::size_t>(ridx)] = 1;
      policeSourceRoadIdx.push_back(ridx);
    }
  }
  out.policeStations = stations;
  out.policeAccessRoadTiles = static_cast<int>(policeSourceRoadIdx.size());

  // --- Optional congestion-aware costs ---
  std::vector<int> extraCostMilli;
  const std::vector<int>* extraCostPtr = nullptr;
  if (traffic && cfg.congestionCosts && traffic->roadTraffic.size() == n) {
    extraCostMilli.assign(n, 0);
    const float capScale = std::max(0.01f, cfg.congestionCapacityScale);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t idx = FlatIdx(x, y, w);
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;
        if (roadToEdge && idx < roadToEdge->size() && (*roadToEdge)[idx] == 0) continue;

        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        int baseMilli = (t.terrain == Terrain::Water) ? RoadBridgeTravelTimeMilliForLevel(lvl)
                                                      : RoadTravelTimeMilliForLevel(lvl);

        float cap = static_cast<float>(RoadCapacityForLevel(cfg.roadTileCapacity, lvl));
        cap *= capScale;
        cap = std::max(1.0f, cap);

        const float v = static_cast<float>(traffic->roadTraffic[idx]);
        float ratio = v / cap;
        ratio = std::min(ratio, cfg.congestionRatioClamp);
        ratio = std::max(0.0f, ratio);

        const float mult = 1.0f + cfg.congestionAlpha * std::pow(ratio, cfg.congestionBeta);
        const int add = static_cast<int>(std::lround((mult - 1.0f) * static_cast<float>(baseMilli)));
        extraCostMilli[idx] = std::max(0, add);
      }
    }
    extraCostPtr = &extraCostMilli;
  }

  // --- Compute police access field ---
  if (!policeSourceRoadIdx.empty()) {
    RoadIsochroneConfig rcfg{};
    rcfg.requireOutsideConnection = cfg.requireOutsideConnection;
    rcfg.weightMode = cfg.weightMode;
    rcfg.computeOwner = false;

    const RoadIsochroneField roadField = BuildRoadIsochroneField(world, policeSourceRoadIdx, rcfg, roadToEdge, extraCostPtr);

    TileAccessCostConfig tcfg{};
    tcfg.includeRoadTiles = true;
    tcfg.includeZones = true;
    tcfg.includeNonZonesAdjacentToRoad = true;
    tcfg.includeWater = false;
    tcfg.accessStepCostMilli = cfg.accessStepCostMilli;
    tcfg.useZoneAccessMap = true;

    out.policeCostMilli = BuildTileAccessCostField(world, roadField, tcfg, roadToEdge, zoneAccess);

    const float halfLife = std::max(1.0f, static_cast<float>(cfg.responseHalfLifeCostMilli));
    const float maxCost = std::max(1.0f, static_cast<float>(cfg.responseMaxCostMilli));

    for (std::size_t i = 0; i < n; ++i) {
      const int c = out.policeCostMilli[i];
      if (c < 0) {
        out.policeAccess01[i] = 0.0f;
        continue;
      }
      const float cc = static_cast<float>(c);
      float score = 1.0f / (1.0f + cc / halfLife);
      // Fade to 0 near maxCost.
      const float fade = 1.0f - Clamp01((cc - 0.70f * maxCost) / (0.30f * maxCost));
      score *= fade;
      out.policeAccess01[i] = Clamp01(score);
    }
  }

  // --- Normalize traffic/goods for activity proxies ---
  std::uint16_t maxCommute = 0;
  if (traffic && traffic->roadTraffic.size() == n) {
    maxCommute = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (maxCommute == 0) {
      for (std::uint16_t v : traffic->roadTraffic) maxCommute = std::max(maxCommute, v);
    }
  }

  std::uint16_t maxGoods = 0;
  if (goods && goods->roadGoodsTraffic.size() == n) {
    maxGoods = static_cast<std::uint16_t>(std::clamp(goods->maxRoadGoodsTraffic, 0, 65535));
    if (maxGoods == 0) {
      for (std::uint16_t v : goods->roadGoodsTraffic) maxGoods = std::max(maxGoods, v);
    }
  }

  // --- Occupancy percentile for scaling ---
  std::vector<int> occSamples;
  occSamples.reserve(n / 2);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
        if (t.occupants > 0) occSamples.push_back(static_cast<int>(t.occupants));
      }
    }
  }
  const float occP95 = std::max(1.0f, PercentileInt(std::move(occSamples), 0.95f));

  // --- Main risk model ---
  std::uint64_t resPop = 0;
  double resRiskSum = 0.0;
  double resPoliceSum = 0.0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float r = cfg.baseNone;
      switch (t.overlay) {
      case Overlay::Road: r = cfg.baseRoad; break;
      case Overlay::Residential: r = cfg.baseResidential; break;
      case Overlay::Commercial: r = cfg.baseCommercial; break;
      case Overlay::Industrial: r = cfg.baseIndustrial; break;
      case Overlay::Park: r = cfg.basePark; break;
      case Overlay::School:
      case Overlay::Hospital:
      case Overlay::PoliceStation:
      case Overlay::FireStation: r = cfg.baseService; break;
      default: break;
      }

      if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
        const int lv = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        r += cfg.levelBoost * static_cast<float>(lv - 1);

        if (t.occupants > 0) {
          const float occ01 = Clamp01(static_cast<float>(t.occupants) / occP95);
          r += cfg.occupantsWeight * std::sqrt(occ01);
        }
      }

      // Economic stress from job access/opportunity.
      if (jobs && jobs->jobAccess01.size() == n && jobs->jobOpportunity01.size() == n) {
        const float a = Clamp01(jobs->jobAccess01[i]);
        const float o = Clamp01(jobs->jobOpportunity01[i]);
        r += cfg.jobAccessWeight * (1.0f - a);
        r += cfg.jobOpportunityWeight * (1.0f - o);
      }

      // Activity / opportunity proxies.
      if (maxCommute > 0 && traffic && traffic->roadTraffic.size() == n) {
        const float flow01 = static_cast<float>(traffic->roadTraffic[i]) / static_cast<float>(maxCommute);
        r += cfg.trafficOpportunityWeight * Clamp01(std::sqrt(std::max(0.0f, flow01)));
      }
      if (maxGoods > 0 && goods && goods->roadGoodsTraffic.size() == n) {
        const float g01 = static_cast<float>(goods->roadGoodsTraffic[i]) / static_cast<float>(maxGoods);
        r += cfg.goodsTrafficWeight * Clamp01(std::sqrt(std::max(0.0f, g01)));
      }

      if (noise && noise->noise01.size() == n) {
        r += cfg.noiseWeight * Clamp01(noise->noise01[i]);
      }

      // Apply policing suppression.
      const float p = (out.policeAccess01.size() == n) ? Clamp01(out.policeAccess01[i]) : 0.0f;
      r *= (1.0f - cfg.policeSuppressionStrength * p);
      if (t.overlay == Overlay::PoliceStation) r *= 0.25f;

      r = Clamp01(r);
      r = std::pow(r, std::max(0.05f, cfg.riskCurveExp));
      out.risk01[i] = r;

      if (t.overlay == Overlay::Residential && t.occupants > 0) {
        resPop += static_cast<std::uint64_t>(t.occupants);
        resRiskSum += static_cast<double>(r) * static_cast<double>(t.occupants);
        resPoliceSum += static_cast<double>(p) * static_cast<double>(t.occupants);
      }
    }
  }

  out.residentPopulation = static_cast<int>(std::min<std::uint64_t>(resPop, 0x7fffffffULL));
  if (resPop > 0) {
    out.residentMeanRisk = static_cast<float>(resRiskSum / static_cast<double>(resPop));
    out.residentMeanPoliceAccess = static_cast<float>(resPoliceSum / static_cast<double>(resPop));
  }

  return out;
}

} // namespace isocity
