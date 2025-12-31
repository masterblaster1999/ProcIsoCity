#include "isocity/Sim.hpp"

#include "isocity/Road.hpp"

#include "isocity/Pathfinding.hpp"

#include "isocity/Traffic.hpp"

#include "isocity/Goods.hpp"

#include "isocity/LandValue.hpp"

#include "isocity/Random.hpp"

#include "isocity/ZoneMetrics.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

namespace isocity {

namespace {
inline int JobsForTile(const Tile& t)
{
  if (t.overlay == Overlay::Commercial) return JobsCommercialForLevel(t.level);
  if (t.overlay == Overlay::Industrial) return JobsIndustrialForLevel(t.level);
  return 0;
}

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

float ResidentialDemand(float jobPressure, float happiness, float avgLandValue)
{
  // A tiny, stable "meter" that avoids runaway population early.
  // The intention is that jobs are the main driver, happiness matters, and
  // overall land value nudges demand upward in nice cities.
  const float jp = std::min(jobPressure, 1.0f);
  float d = 0.12f + 0.65f * jp + 0.25f * happiness + 0.10f * avgLandValue;
  return Clamp01(d);
}

float AvgLandValueNonWater(const World& world, const LandValueResult& lv)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0.0f;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (lv.value.size() != n) return 0.0f;

  double sum = 0.0;
  int count = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (world.at(x, y).terrain == Terrain::Water) continue;
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      sum += static_cast<double>(lv.value[idx]);
      count++;
    }
  }
  if (count <= 0) return 0.0f;
  return static_cast<float>(sum / static_cast<double>(count));
}

// Traffic/commute parameters.
// Tuned to be noticeable but not dominate the early-game economy.
constexpr float kCommuteTarget = 24.0f; // avg road-steps where the penalty reaches its cap
constexpr float kCommutePenaltyCap = 0.18f;
constexpr float kCongestionPenaltyCap = 0.18f;
constexpr float kGoodsPenaltyCap = 0.16f;

struct ScanResult {
  int housingCap = 0;
  int jobsCap = 0;
  int roads = 0;
  int roadMaintenanceUnits = 0;
  int parks = 0;
  int zoneTiles = 0;
  int population = 0;
};

ScanResult ScanWorld(const World& world)
{
  ScanResult r;

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);

      if (t.overlay == Overlay::Road) {
        r.roads++;
        r.roadMaintenanceUnits += (t.terrain == Terrain::Water)
          ? RoadBridgeMaintenanceUnitsForLevel(static_cast<int>(t.level))
          : RoadMaintenanceUnitsForLevel(static_cast<int>(t.level));
      }
      if (t.overlay == Overlay::Park) r.parks++;

      if (t.overlay == Overlay::Residential) {
        r.zoneTiles++;
        r.housingCap += HousingForLevel(t.level);
        r.population += static_cast<int>(t.occupants);
      } else if (t.overlay == Overlay::Commercial) {
        r.zoneTiles++;
        r.jobsCap += JobsCommercialForLevel(t.level);
      } else if (t.overlay == Overlay::Industrial) {
        r.zoneTiles++;
        r.jobsCap += JobsIndustrialForLevel(t.level);
      }
    }
  }

  return r;
}

// Compute which road tiles are connected to the map border ("outside connection").
//
// We treat the map edge as the entry point for citizens/jobs (classic city builder rule).
// A road component that does not touch the edge is considered disconnected and won't provide access.
void ComputeEdgeConnectedRoads(const World& world, std::vector<std::uint8_t>& outRoadToEdge)
{
  // Implementation lives in the core pathfinding/utility module so it can be reused by
  // the simulation, renderer debug overlays, and future systems.
  ComputeRoadsConnectedToEdge(world, outRoadToEdge);
}

bool HasAdjacentEdgeConnectedRoad(const World& world, const std::vector<std::uint8_t>& roadToEdge, int x, int y)
{
  return HasAdjacentRoadConnectedToEdge(world, roadToEdge, x, y);
}

// Parks are modeled as an *area of influence* rather than a global ratio.
// We compute a simple "coverage" ratio: what fraction of zone tiles are within
// a Manhattan distance <= radius of any park that is connected to a road.
//
// Notes:
// - We treat Water as a barrier so disconnected islands don't share park benefits.
// - This is intentionally lightweight: a multi-source BFS on a 96x96 grid is cheap.
float ParkCoverageRatio(const World& world, int radius, const std::vector<std::uint8_t>* roadToEdge)
{
  // Compatibility mode: treat parks as a global ratio (old behavior).
  if (radius <= 0) {
    int zones = 0;
    int parks = 0;
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);

        if (t.overlay == Overlay::Park) {
          bool connected = world.hasAdjacentRoad(x, y);
          if (roadToEdge) connected = HasAdjacentEdgeConnectedRoad(world, *roadToEdge, x, y);
          if (connected) parks++;
        }

        const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                             t.overlay == Overlay::Industrial);
        if (isZone) zones++;
      }
    }
    if (zones <= 0) return 0.0f;
    return static_cast<float>(parks) / static_cast<float>(zones);
  }

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0.0f;

  constexpr int kInf = 1'000'000;
  std::vector<int> dist(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), kInf);
  std::vector<int> queue;
  queue.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 8);

  auto idxOf = [&](int x, int y) -> int { return y * w + x; };

  // Initialize BFS sources: parks that have a road neighbor.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Park) continue;
      if (t.terrain == Terrain::Water) continue;
      bool connected = world.hasAdjacentRoad(x, y);
      if (roadToEdge) connected = HasAdjacentEdgeConnectedRoad(world, *roadToEdge, x, y);
      if (!connected) continue;
      const int idx = idxOf(x, y);
      dist[static_cast<std::size_t>(idx)] = 0;
      queue.push_back(idx);
    }
  }

  // No connected parks => no coverage.
  if (queue.empty()) return 0.0f;

  // Multi-source BFS (4-neighborhood) limited to the configured radius.
  std::size_t qHead = 0;
  while (qHead < queue.size()) {
    const int idx = queue[qHead++];
    const int d = dist[static_cast<std::size_t>(idx)];
    if (d >= radius) continue;

    const int x = idx % w;
    const int y = idx / w;

    constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& dir : dirs) {
      const int nx = x + dir[0];
      const int ny = y + dir[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const Tile& nt = world.at(nx, ny);
      if (nt.terrain == Terrain::Water && nt.overlay != Overlay::Road) continue;

      const int nidx = idxOf(nx, ny);
      int& nd = dist[static_cast<std::size_t>(nidx)];
      if (nd <= d + 1) continue;
      nd = d + 1;
      queue.push_back(nidx);
    }
  }

  int zones = 0;
  int covered = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                           t.overlay == Overlay::Industrial);
      if (!isZone) continue;

      zones++;
      const int idx = idxOf(x, y);
      if (dist[static_cast<std::size_t>(idx)] <= radius) covered++;
    }
  }

  if (zones <= 0) return 0.0f;
  return static_cast<float>(covered) / static_cast<float>(zones);
}
} // namespace

void Simulator::stepOnce(World& world)
{
  // Ensure manual stepping is deterministic and doesn't accidentally queue extra ticks.
  m_accum = 0.0f;
  step(world);
}

void Simulator::update(World& world, float dt)
{
  (void)update(world, dt, nullptr);
}

int Simulator::update(World& world, float dt, std::vector<Stats>* outTickStats)
{
  m_accum += dt;
  int ticks = 0;

  while (m_accum >= m_cfg.tickSeconds) {
    m_accum -= m_cfg.tickSeconds;
    step(world);
    ++ticks;

    if (outTickStats) outTickStats->push_back(world.stats());
  }

  return ticks;
}

void Simulator::refreshDerivedStats(World& world) const
{
  refreshDerivedStatsInternal(world, nullptr, nullptr);
}

void Simulator::refreshDerivedStatsInternal(World& world, const std::vector<std::uint8_t>* precomputedRoadToEdge,
                                            const ZoneAccessMap* precomputedZoneAccess) const
{
  Stats& s = world.stats();
  const ScanResult scan = ScanWorld(world);

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  // Precompute which roads are connected to the map border ("outside connection").
  // When requireOutsideConnection is enabled, derived systems only consider road components
  // that touch the map edge.
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (m_cfg.requireOutsideConnection) {
    if (precomputedRoadToEdge && precomputedRoadToEdge->size() == n) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      ComputeEdgeConnectedRoads(world, roadToEdgeLocal);
      roadToEdge = &roadToEdgeLocal;
    }
  }

  const std::vector<std::uint8_t>* edgeMask = m_cfg.requireOutsideConnection ? roadToEdge : nullptr;

  // Zone access: allows interior tiles of a connected zoned area to be reachable via a
  // road-adjacent boundary tile.
  ZoneAccessMap zoneAccessLocal;
  const ZoneAccessMap* zoneAccess = nullptr;
  if (precomputedZoneAccess && precomputedZoneAccess->w == w && precomputedZoneAccess->h == h &&
      precomputedZoneAccess->roadIdx.size() == n) {
    zoneAccess = precomputedZoneAccess;
  } else {
    zoneAccessLocal = BuildZoneAccessMap(world, edgeMask);
    zoneAccess = &zoneAccessLocal;
  }


  // Only job tiles that are actually reachable should count as capacity.
  // Otherwise the sim can incorrectly show "employment" (and income) even when all jobs
  // are on disconnected road components.
  int jobsCapAccessible = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;
      if (!HasZoneAccess(*zoneAccess, x, y)) continue;
      jobsCapAccessible += JobsForTile(t);
    }
  }

  // Employment: fill accessible jobs up to population.
  const int employed = std::min(scan.population, jobsCapAccessible);

  // Traffic/commute model: estimate how far (and how congested) the average commute is.
  // This is a derived system (no agents yet): we run a multi-source road search from job access points
  // over the road network and route commuters along parent pointers back to the jobs.
  const float employedShare = (scan.population > 0)
                                 ? (static_cast<float>(employed) / static_cast<float>(scan.population))
                                 : 0.0f;

  TrafficConfig tc;
  tc.requireOutsideConnection = m_cfg.requireOutsideConnection;
  // Runtime traffic model tuning (not persisted in saves).
  tc.congestionAwareRouting = m_trafficModel.congestionAwareRouting;
  tc.congestionIterations = m_trafficModel.congestionIterations;
  tc.congestionAlpha = m_trafficModel.congestionAlpha;
  tc.congestionBeta = m_trafficModel.congestionBeta;
  tc.congestionCapacityScale = m_trafficModel.congestionCapacityScale;
  tc.congestionRatioClamp = m_trafficModel.congestionRatioClamp;
  const TrafficResult traffic = ComputeCommuteTraffic(world, tc, employedShare, roadToEdge, zoneAccess);

  s.commuters = traffic.totalCommuters;
  s.commutersUnreachable = traffic.unreachableCommuters;
  s.avgCommute = traffic.avgCommute;
  s.p95Commute = traffic.p95Commute;
  s.avgCommuteTime = traffic.avgCommuteTime;
  s.p95CommuteTime = traffic.p95CommuteTime;
  s.trafficCongestion = traffic.congestion;
  s.congestedRoadTiles = traffic.congestedRoadTiles;
  s.maxRoadTraffic = traffic.maxTraffic;

  // Goods/logistics model: route industrial output to commercial demand along roads.
  GoodsConfig gc;
  gc.requireOutsideConnection = m_cfg.requireOutsideConnection;
  const GoodsResult goods = ComputeGoodsFlow(world, gc, roadToEdge, zoneAccess);
  s.goodsProduced = goods.goodsProduced;
  s.goodsDemand = goods.goodsDemand;
  s.goodsDelivered = goods.goodsDelivered;
  s.goodsImported = goods.goodsImported;
  s.goodsExported = goods.goodsExported;
  s.goodsUnreachableDemand = goods.unreachableDemand;
  s.goodsSatisfaction = goods.satisfaction;
  s.maxRoadGoodsTraffic = goods.maxRoadGoodsTraffic;

  // Land value (amenities + pollution + optional traffic spill). Used both for
  // display and for the simple tax model.
  LandValueConfig lvc;
  lvc.requireOutsideConnection = m_cfg.requireOutsideConnection;
  const LandValueResult lv = ComputeLandValue(world, lvc, &traffic, roadToEdge);
  s.avgLandValue = AvgLandValueNonWater(world, lv);

  // Economy snapshot (does NOT mutate money here; that's handled in step()).
  // Taxes scale by land value so attractive areas generate more revenue.
  const float lvBase = 0.75f;
  const float lvScale = 0.75f;

  const bool useDistrictPolicies = m_cfg.districtPoliciesEnabled;

  auto clampDistrict = [](int d) -> int { return std::clamp(d, 0, kDistrictCount - 1); };

  auto policyForTile = [&](const Tile& t) -> const DistrictPolicy& {
    const int d = clampDistrict(static_cast<int>(t.district));
    return m_cfg.districtPolicies[static_cast<std::size_t>(d)];
  };

  auto taxMultFor = [&](const Tile& t) -> float {
    if (!useDistrictPolicies) return 1.0f;
    const DistrictPolicy& p = policyForTile(t);
    if (t.overlay == Overlay::Residential) return std::max(0.0f, p.taxResidentialMult);
    if (t.overlay == Overlay::Commercial) return std::max(0.0f, p.taxCommercialMult);
    if (t.overlay == Overlay::Industrial) return std::max(0.0f, p.taxIndustrialMult);
    return 1.0f;
  };

  auto roadMaintMultFor = [&](const Tile& t) -> float {
    if (!useDistrictPolicies) return 1.0f;
    return std::max(0.0f, policyForTile(t).roadMaintenanceMult);
  };

  auto parkMaintMultFor = [&](const Tile& t) -> float {
    if (!useDistrictPolicies) return 1.0f;
    return std::max(0.0f, policyForTile(t).parkMaintenanceMult);
  };

  int taxRevenue = 0;
  int roadMaint = 0;
  int parkMaint = 0;

  const bool lvOk =
    (lv.value.size() == static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height()));

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);

      // Maintenance is per-tile and may be scaled by district policy.
      if (t.overlay == Overlay::Road) {
        const int units = (t.terrain == Terrain::Water)
          ? RoadBridgeMaintenanceUnitsForLevel(static_cast<int>(t.level))
          : RoadMaintenanceUnitsForLevel(static_cast<int>(t.level));
        const float mult = roadMaintMultFor(t);
        const float raw = static_cast<float>(units * std::max(0, m_cfg.maintenanceRoad)) * mult;
        roadMaint += std::max(0, static_cast<int>(std::lround(raw)));
      } else if (t.overlay == Overlay::Park) {
        const float mult = parkMaintMultFor(t);
        const float raw = static_cast<float>(std::max(0, m_cfg.maintenancePark)) * mult;
        parkMaint += std::max(0, static_cast<int>(std::lround(raw)));
      }

      // Taxes apply only to occupied zones.
      if (t.overlay != Overlay::Residential && t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;
      if (t.occupants == 0) continue;
      if (!lvOk) continue;

      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) +
                              static_cast<std::size_t>(x);
      const float lvMult = lvBase + lvScale * lv.value[idx];

      int taxPerOcc = 0;
      if (t.overlay == Overlay::Residential) taxPerOcc = m_cfg.taxResidential;
      else if (t.overlay == Overlay::Commercial) taxPerOcc = m_cfg.taxCommercial;
      else if (t.overlay == Overlay::Industrial) taxPerOcc = m_cfg.taxIndustrial;

      const float taxMult = taxMultFor(t);
      const float raw = static_cast<float>(t.occupants) * static_cast<float>(taxPerOcc) * lvMult * taxMult;
      const int add = std::max(0, static_cast<int>(std::lround(raw)));
      taxRevenue += add;
    }
  }

  const int maintenance = roadMaint + parkMaint;

  // Trade: imports cost money, exports earn money. Keep the exchange rate mild so it
  // doesn't dominate the early game.
  const int importCost = goods.goodsImported / 20;
  const int exportRevenue = goods.goodsExported / 25;

  s.taxRevenue = taxRevenue;
  s.maintenanceCost = maintenance;
  s.upgradeCost = 0;
  s.importCost = importCost;
  s.exportRevenue = exportRevenue;
  s.income = taxRevenue + exportRevenue;
  s.expenses = maintenance + importCost;
  s.avgTaxPerCapita =
    (scan.population > 0) ? (static_cast<float>(taxRevenue) / static_cast<float>(scan.population)) : 0.0f;

  // Happiness: parks help (locally), unemployment hurts, and commutes/congestion add friction.
  const float parkCoverage = ParkCoverageRatio(world, m_cfg.parkInfluenceRadius, roadToEdge);
  const float parkBonus = std::min(0.25f, parkCoverage * 0.35f);

  const float unemployment = (scan.population > 0)
                                 ? (1.0f - (static_cast<float>(employed) / static_cast<float>(scan.population)))
                                 : 0.0f;

  const float commuteNorm = (traffic.reachableCommuters > 0)
                                ? std::clamp(traffic.avgCommuteTime / kCommuteTarget, 0.0f, 2.0f)
                                : 0.0f;
  const float commutePenalty = std::min(kCommutePenaltyCap, commuteNorm * kCommutePenaltyCap);
  const float congestionPenalty = std::min(kCongestionPenaltyCap, traffic.congestion * (kCongestionPenaltyCap * 1.35f));

  const float goodsPenalty = std::min(kGoodsPenaltyCap, (1.0f - goods.satisfaction) * kGoodsPenaltyCap);

  const float taxPenalty = std::min(0.20f, s.avgTaxPerCapita * std::max(0.0f, m_cfg.taxHappinessPerCapita));
  const float lvBonus = std::clamp((s.avgLandValue - 0.50f) * 0.10f, -0.05f, 0.05f);

  s.happiness = Clamp01(0.45f + parkBonus + lvBonus - unemployment * 0.35f - commutePenalty - congestionPenalty - goodsPenalty - taxPenalty);

  // Demand meter (for UI/debug): recompute using the newly derived happiness.
  const float jobPressure = (scan.housingCap > 0)
                                ? (static_cast<float>(jobsCapAccessible) / static_cast<float>(scan.housingCap))
                                : 0.0f;
  s.demandResidential = ResidentialDemand(jobPressure, s.happiness, s.avgLandValue);

  s.population = scan.population;
  s.housingCapacity = scan.housingCap;
  s.jobsCapacity = scan.jobsCap;
  s.jobsCapacityAccessible = jobsCapAccessible;
  s.employed = employed;
  s.roads = scan.roads;
  s.parks = scan.parks;
}

void Simulator::step(World& world)
{
  Stats& s = world.stats();
  s.day++;

  // Precompute which roads are connected to the map border ("outside connection").
  // When requireOutsideConnection is enabled, zones only function if they touch a road
  // component that reaches the edge of the map.
  std::vector<std::uint8_t> roadToEdge;
  if (m_cfg.requireOutsideConnection) {
    ComputeEdgeConnectedRoads(world, roadToEdge);
  }

  const std::vector<std::uint8_t>* edgeMask = m_cfg.requireOutsideConnection ? &roadToEdge : nullptr;
  const ZoneAccessMap zoneAccess = BuildZoneAccessMap(world, edgeMask);

  auto hasZoneAccess = [&](int x, int y) -> bool {
    return HasZoneAccess(zoneAccess, x, y);
  };

  // Land value field (no traffic spill for the simulation growth step).
  LandValueConfig lvc;
  lvc.requireOutsideConnection = m_cfg.requireOutsideConnection;
  const LandValueResult lv = ComputeLandValue(world, lvc, nullptr,
                                              m_cfg.requireOutsideConnection ? &roadToEdge : nullptr);
  const float avgLv = AvgLandValueNonWater(world, lv);

  // Optional auto-development: use *previous* happiness and current land value.
  // This keeps the system deterministic and avoids circular dependencies.
  int upgradeCost = 0;
  RNG rng(world.seed() ^ (static_cast<std::uint64_t>(s.day) * 0x9E3779B97F4A7C15ULL));
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
      if (!isZone) continue;
      if (!hasZoneAccess(x, y)) continue;

      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) +
                              static_cast<std::size_t>(x);
      const float lvVal = (idx < lv.value.size()) ? lv.value[idx] : 0.0f;

      const int cap = (t.overlay == Overlay::Residential) ? HousingForLevel(t.level) : JobsForTile(t);
      const float occFrac = (cap > 0) ? (static_cast<float>(t.occupants) / static_cast<float>(cap)) : 0.0f;

      // Upgrade: happy + high land value + mostly full.
      if (t.level < 3 && s.happiness > 0.58f && lvVal > 0.45f && occFrac > 0.70f && s.money > 80) {
        const float p = 0.0010f + 0.0040f * s.happiness * (0.6f + 0.4f * lvVal) * occFrac;
        if (rng.chance(p)) {
          t.level++;
          // Some disruption during construction.
          t.occupants = static_cast<std::uint16_t>(static_cast<int>(t.occupants) * 0.85f);
          upgradeCost += 15 + 20 * t.level;
        }
      }

      // Downgrade: unhappy + low land value + mostly empty.
      if (t.level > 1 && s.happiness < 0.42f && lvVal < 0.25f && occFrac < 0.35f) {
        const float p = 0.0008f + 0.0030f * (0.42f - s.happiness) * (0.25f - lvVal) * (1.0f - occFrac);
        if (rng.chance(p)) {
          t.level--;
          const int newCap = (t.overlay == Overlay::Residential) ? HousingForLevel(t.level) : JobsForTile(t);
          t.occupants = static_cast<std::uint16_t>(std::min<int>(static_cast<int>(t.occupants), newCap));
        }
      }
    }
  }

  // Pass 1: capacities and static counts.
  const ScanResult scan = ScanWorld(world);
  const int housingCap = scan.housingCap;

  // Jobs that are reachable this tick (by road, and optionally via an outside-connected component).
  int jobsCapAccessible = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;
      if (!hasZoneAccess(x, y)) continue;
      jobsCapAccessible += JobsForTile(t);
    }
  }

  // Demand model (global): housing grows if there are jobs + happiness + overall land value.
  const float jobPressure = (housingCap > 0)
                                ? (static_cast<float>(jobsCapAccessible) / static_cast<float>(housingCap))
                                : 0.0f;
  const float demand = ResidentialDemand(jobPressure, s.happiness, avgLv);

  // Pass 2: residential update (population moves toward target occupancy).
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;

      const bool access = hasZoneAccess(x, y);
      const int cap = HousingForLevel(t.level);

      if (!access) {
        const int decay = 1 + t.level;
        t.occupants = static_cast<std::uint16_t>(std::max(0, static_cast<int>(t.occupants) - decay));
        continue;
      }

      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) +
                              static_cast<std::size_t>(x);
      const float lvVal = (idx < lv.value.size()) ? lv.value[idx] : 0.0f;

      const float desir = std::clamp(1.0f + m_cfg.residentialDesirabilityWeight * (lvVal - 0.5f), 0.40f, 1.60f);
      const float tileDemand = Clamp01(demand * desir);

      const int target = std::clamp(static_cast<int>(std::lround(static_cast<float>(cap) * tileDemand)), 0, cap);
      const int cur = static_cast<int>(t.occupants);

      if (cur < target) {
        const int grow = 1 + t.level;
        t.occupants = static_cast<std::uint16_t>(std::min(cap, cur + grow));
      } else if (cur > target) {
        t.occupants = static_cast<std::uint16_t>(std::max(0, cur - 1));
      }
    }
  }

  // Recompute population after residential update.
  int population = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay == Overlay::Residential) population += static_cast<int>(t.occupants);
    }
  }

  // Employment: fill jobs up to population.
  const int employed = std::min(population, jobsCapAccessible);

  // Pass 3: distribute employment across job tiles with desirability weighting.
  struct JobSite {
    int x = 0;
    int y = 0;
    int cap = 0;
    Overlay type = Overlay::None;
    float weight = 1.0f;
  };

  std::vector<JobSite> sites;
  sites.reserve(static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height()) / 8u);

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;

      const bool access = hasZoneAccess(x, y);
      if (!access) {
        t.occupants = static_cast<std::uint16_t>(std::max(0, static_cast<int>(t.occupants) - 1));
        continue;
      }

      const int cap = JobsForTile(t);
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) +
                              static_cast<std::size_t>(x);
      const float lvVal = (idx < lv.value.size()) ? lv.value[idx] : 0.0f;

      const float w = (t.overlay == Overlay::Commercial) ? m_cfg.commercialDesirabilityWeight
                                                         : m_cfg.industrialDesirabilityWeight;
      const float desirBase = (t.overlay == Overlay::Commercial) ? lvVal : (1.0f - lvVal);
      const float weight = std::clamp(1.0f + w * (desirBase - 0.5f), 0.25f, 2.0f);

      sites.push_back(JobSite{x, y, cap, t.overlay, weight});
    }
  }

  std::sort(sites.begin(), sites.end(), [&](const JobSite& a, const JobSite& b) {
    if (a.weight != b.weight) return a.weight > b.weight;
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  });

  int remainingWorkers = employed;
  for (const JobSite& s0 : sites) {
    Tile& t = world.at(s0.x, s0.y);
    const int assigned = std::min(s0.cap, remainingWorkers);
    t.occupants = static_cast<std::uint16_t>(assigned);
    remainingWorkers -= assigned;
  }

  // Any remaining accessible job tiles not assigned above should be emptied.
  if (remainingWorkers <= 0) {
    for (const JobSite& s0 : sites) {
      if (s0.cap <= 0) continue;
      // Already assigned in the loop above.
    }
  }

  // Recompute derived stats (traffic, goods, happiness, budget metrics) for the new state.
  refreshDerivedStatsInternal(world, edgeMask, &zoneAccess);

  // Include upgrade spending in the budget and apply the net change to money.
  s.upgradeCost = upgradeCost;
  s.expenses += upgradeCost;

  s.money += (s.income - s.expenses);
}


} // namespace isocity
