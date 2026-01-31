#include "isocity/Sim.hpp"

#include "isocity/Road.hpp"

#include "isocity/Pathfinding.hpp"

#include "isocity/Traffic.hpp"

#include "isocity/Goods.hpp"

#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"

#include "isocity/LandValue.hpp"

#include "isocity/DeterministicMath.hpp"

#include "isocity/Isochrone.hpp"

#include "isocity/Random.hpp"

#include "isocity/ZoneMetrics.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/FireRisk.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>
#include <limits>

namespace isocity {

namespace {
inline int JobsForTile(const Tile& t)
{
  if (t.overlay == Overlay::Commercial) return JobsCommercialForLevel(t.level);
  if (t.overlay == Overlay::Industrial) return JobsIndustrialForLevel(t.level);
  return 0;
}

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// ---------------------------------------------------------------------------
// Municipal bonds / debt service
//
// The simulator treats debt service as a daily expense that reduces cash and
// slowly amortizes outstanding balances. This lives in the simulation layer so
// the budget graphs and economy metrics naturally include it.
// ---------------------------------------------------------------------------

inline int ComputeDailyDebtInterest(int balance, int aprBasisPoints)
{
  if (balance <= 0 || aprBasisPoints <= 0) return 0;

  // APR basis points (1bp = 0.01%) converted to a per-day rate.
  // interest = balance * (apr / 365)
  //          = balance * (bp / 10000) / 365
  //          = balance * bp / (10000*365)
  constexpr std::int64_t kDenom = 10000LL * 365LL;
  const std::int64_t num = static_cast<std::int64_t>(balance) * static_cast<std::int64_t>(aprBasisPoints);

  // Round up so small balances still accrue at least 1 when appropriate.
  std::int64_t interest = (num + kDenom - 1) / kDenom;
  if (interest < 0) interest = 0;
  if (interest > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
    interest = static_cast<std::int64_t>(std::numeric_limits<int>::max());
  }
  return static_cast<int>(interest);
}

inline int ApplyDebtService(World& world)
{
  auto& debts = world.debts();
  if (debts.empty()) return 0;

  std::int64_t totalPaid = 0;

  for (DebtItem& d : debts) {
    if (d.balance <= 0 || d.daysLeft <= 0) continue;

    const int interest = ComputeDailyDebtInterest(d.balance, d.aprBasisPoints);

    // Avoid overflow.
    if (interest > 0 && d.balance > (std::numeric_limits<int>::max() - interest)) {
      d.balance = std::numeric_limits<int>::max();
    } else {
      d.balance += interest;
    }

    int pay = d.dailyPayment;
    // Force full payoff on the final day to avoid rounding drift.
    if (d.daysLeft <= 1) pay = d.balance;

    pay = std::clamp(pay, 0, d.balance);
    d.balance -= pay;
    totalPaid += pay;

    d.daysLeft -= 1;
    if (d.daysLeft < 0) d.daysLeft = 0;
  }

  // Remove repaid / expired entries.
  debts.erase(std::remove_if(debts.begin(), debts.end(),
                             [](const DebtItem& d) { return d.balance <= 0 || d.daysLeft <= 0; }),
              debts.end());

  if (totalPaid <= 0) return 0;
  if (totalPaid > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
    return std::numeric_limits<int>::max();
  }
  return static_cast<int>(totalPaid);
}


float ResidentialDemand(float jobPressure, float happiness, float avgLandValue)
{
  // A tiny, stable "meter" that avoids runaway population early.
  // The intention is that jobs are the main driver, happiness matters, and
  // overall land value nudges demand upward in nice cities.
  const float jp = std::min(jobPressure, 1.0f);
  float d = 0.12f + 0.65f * jp + 0.25f * happiness + 0.10f * avgLandValue;
  return Clamp01(d);
}

float CommercialDemand(int population, int jobsCommercialAccessible, float goodsSatisfaction,
                       float happiness, float avgLandValue, int taxCommercial)
{
  if (population <= 0) return 0.0f;

  // A lightweight SimCity-ish meter: commercial demand rises when population outgrows
  // accessible commercial job capacity, and falls when oversupplied.
  const float pop = static_cast<float>(population);

  // Rough target: ~0.28 service jobs per resident.
  const float desired = std::max(6.0f, pop * 0.28f);
  const float gap = (desired - static_cast<float>(std::max(0, jobsCommercialAccessible))) / desired; // [-inf..1]
  const float shortage = Clamp01(gap);
  const float oversupply = Clamp01(-gap);

  const float sizeFactor = Clamp01(pop / 140.0f);
  const float goodsFactor = Clamp01(0.35f + 0.65f * goodsSatisfaction);
  const float happyFactor = Clamp01(0.55f + 0.45f * happiness);
  const float lvFactor = Clamp01(0.60f + 0.40f * avgLandValue);
  const float taxFactor = Clamp01(1.05f - 0.06f * static_cast<float>(std::max(0, taxCommercial)));

  float d = 0.08f + 0.72f * shortage - 0.55f * oversupply + 0.20f * sizeFactor;
  d *= goodsFactor * happyFactor * lvFactor * taxFactor;
  return Clamp01(d);
}

float IndustrialDemand(float jobPressure, int population, int jobsIndustrialAccessible,
                       float goodsSatisfaction, float tradeMarketIndex,
                       float happiness, float avgLandValue, int taxIndustrial)
{
  // Industrial demand is a blend of:
  //  - job shortfall (need more employment capacity)
  //  - goods shortfall (need more production/logistics)
  //  - trade/market strength (export demand)
  // and is tempered by land value (industry prefers cheaper land) and tax.

  const float jobsNeed = Clamp01(1.0f - std::min(1.0f, jobPressure));
  const float goodsNeed = Clamp01(1.0f - goodsSatisfaction);

  // If we already have lots of industrial capacity relative to population, dampen.
  const float pop = static_cast<float>(std::max(0, population));
  const float desired = std::max(6.0f, pop * 0.22f);
  const float gap = (desired - static_cast<float>(std::max(0, jobsIndustrialAccessible))) / desired;
  const float shortage = Clamp01(gap);
  const float oversupply = Clamp01(-gap);

  const float happyFactor = Clamp01(0.55f + 0.45f * happiness);
  const float lvFactor = Clamp01(0.75f + 0.35f * (0.55f - avgLandValue));
  const float taxFactor = Clamp01(1.05f - 0.06f * static_cast<float>(std::max(0, taxIndustrial)));
  const float tradeFactor = Clamp01(0.70f + 0.30f * std::clamp(tradeMarketIndex, 0.0f, 2.0f));

  float d = 0.06f + 0.55f * jobsNeed + 0.35f * goodsNeed + 0.20f * shortage - 0.45f * oversupply;
  d *= happyFactor * lvFactor * taxFactor * tradeFactor;
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

// ---------------------------------------------------------------------------
// Fire incidents (emergent gameplay)
// ---------------------------------------------------------------------------

inline bool IsZoneFlammable(Overlay o)
{
  return (o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial);
}

struct FireIncidentOutcome {
  bool happened = false;
  bool zoneLayoutChanged = false; // true if any tile overlay was cleared

  int damaged = 0;
  int destroyed = 0;
  int displaced = 0;
  int jobsLostCap = 0;

  int originX = -1;
  int originY = -1;
  int originDistrict = -1;

  int cost = 0;
  float happinessPenalty = 0.0f;
};

FireIncidentOutcome TryApplyFireIncident(World& world,
                                        const FireIncidentSettings& settings,
                                        bool requireOutsideConnection,
                                        const ZoneAccessMap& zoneAccess,
                                        const std::vector<std::uint8_t>* roadToEdge,
                                        int population,
                                        int zoneTiles,
                                        RNG& rng)
{
  FireIncidentOutcome out;
  if (!settings.enabled) return out;
  if (population < settings.minPopulation) return out;
  if (zoneTiles < settings.minZoneTiles) return out;

  // Count fire stations (cheap; used to scale frequency).
  int fireStations = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      if (world.at(x, y).overlay == Overlay::FireStation) fireStations++;
    }
  }

  float p = settings.baseChancePerDay;
  p += settings.chancePer100Population * (static_cast<float>(population) / 100.0f);

  // Scale gently with the amount of built city.
  const float zoneScale = std::clamp(static_cast<float>(zoneTiles) / 120.0f, 0.35f, 1.75f);
  p *= zoneScale;

  // Fire stations reduce incidents.
  if (fireStations <= 0) {
    p *= settings.noStationMultiplier;
  } else {
    const float fac = std::clamp(1.0f - settings.stationChanceMitigation * static_cast<float>(fireStations),
                                 settings.minChanceFactor, 1.0f);
    p *= fac;
  }
  p = std::clamp(p, 0.0f, 0.12f);

  if (!rng.chance(p)) return out;

  // Build a fire risk field to choose plausible origins and to modulate severity.
  FireRiskConfig frc{};
  frc.requireOutsideConnection = requireOutsideConnection;
  FireRiskResult fr = ComputeFireRisk(world, frc, &zoneAccess, roadToEdge);

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (fr.risk01.size() != n || fr.coverage01.size() != n) return out;

  // Weighted pick of a flammable zone tile.
  double totalW = 0.0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (!IsZoneFlammable(t.overlay)) continue;
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const float risk = fr.risk01[idx];
      if (!(risk > 0.01f)) continue;

      // Favor higher-risk, higher-intensity tiles.
      const float levelFactor = std::clamp(static_cast<float>(t.level) / 5.0f, 0.35f, 1.25f);
      float occFactor = 0.65f;
      if (t.overlay == Overlay::Residential) {
        const int cap = std::max(1, HousingForLevel(t.level));
        occFactor = std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.05f, 1.0f);
      }
      const double wgt = static_cast<double>(risk) * static_cast<double>(risk) *
                         static_cast<double>(levelFactor) * (0.35 + 0.65 * static_cast<double>(occFactor));
      totalW += wgt;
    }
  }
  if (!(totalW > 0.0)) return out;

  const double pick = static_cast<double>(rng.nextF01()) * totalW;
  double acc = 0.0;
  int ox = -1;
  int oy = -1;
  for (int y = 0; y < h && ox < 0; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (!IsZoneFlammable(t.overlay)) continue;
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const float risk = fr.risk01[idx];
      if (!(risk > 0.01f)) continue;

      const float levelFactor = std::clamp(static_cast<float>(t.level) / 5.0f, 0.35f, 1.25f);
      float occFactor = 0.65f;
      if (t.overlay == Overlay::Residential) {
        const int cap = std::max(1, HousingForLevel(t.level));
        occFactor = std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.05f, 1.0f);
      }
      const double wgt = static_cast<double>(risk) * static_cast<double>(risk) *
                         static_cast<double>(levelFactor) * (0.35 + 0.65 * static_cast<double>(occFactor));
      acc += wgt;
      if (acc >= pick) {
        ox = x;
        oy = y;
        break;
      }
    }
  }
  if (ox < 0 || oy < 0) return out;

  const std::size_t oIdx = static_cast<std::size_t>(oy) * static_cast<std::size_t>(w) + static_cast<std::size_t>(ox);
  const float originRisk = fr.risk01[oIdx];
  const float originCov = fr.coverage01[oIdx];

  // Determine fire size/intensity.
  int maxTiles = rng.rangeInt(settings.minAffectedTiles, settings.maxAffectedTiles);
  float intensity = 0.55f + 0.85f * Clamp01(originRisk);
  intensity *= (1.10f - 0.70f * Clamp01(originCov));
  maxTiles = std::clamp(static_cast<int>(std::lround(static_cast<float>(maxTiles) * intensity)),
                        settings.minAffectedTiles, settings.maxAffectedTiles);

  // Grow the affected set via BFS with probabilistic spread.
  std::vector<std::uint8_t> visited;
  visited.assign(n, 0);
  std::vector<int> queue;
  queue.reserve(static_cast<std::size_t>(maxTiles) * 2u);
  std::vector<int> affected;
  affected.reserve(static_cast<std::size_t>(maxTiles));

  auto pushIf = [&](int nx, int ny, float spreadP) {
    if (nx < 0 || ny < 0 || nx >= w || ny >= h) return;
    const std::size_t nidx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx);
    if (visited[nidx]) return;
    const Tile& nt = world.at(nx, ny);
    if (!IsZoneFlammable(nt.overlay)) return;
    if (!rng.chance(spreadP)) return;
    visited[nidx] = 1;
    queue.push_back(static_cast<int>(nidx));
  };

  visited[oIdx] = 1;
  queue.push_back(static_cast<int>(oIdx));
  std::size_t qh = 0;
  while (qh < queue.size() && affected.size() < static_cast<std::size_t>(maxTiles)) {
    const int idxI = queue[qh++];
    const int x = idxI % w;
    const int y = idxI / w;
    const Tile& t = world.at(x, y);
    if (!IsZoneFlammable(t.overlay)) continue;
    affected.push_back(idxI);

    // Spread is influenced by local risk and mitigated by fire coverage.
    const std::size_t idx = static_cast<std::size_t>(idxI);
    const float nRisk = fr.risk01[idx];
    const float nCov = fr.coverage01[idx];
    float sp = settings.spreadBase;
    sp *= (0.65f + 0.85f * Clamp01(nRisk));
    sp *= (1.05f - 0.65f * Clamp01(nCov));
    sp = std::clamp(sp, 0.05f, 0.92f);

    pushIf(x + 1, y, sp);
    pushIf(x - 1, y, sp);
    pushIf(x, y + 1, sp);
    pushIf(x, y - 1, sp);
  }

  // Apply damage.
  for (int idxI : affected) {
    const int x = idxI % w;
    const int y = idxI / w;
    Tile& t = world.at(x, y);
    if (!IsZoneFlammable(t.overlay)) continue;

    const std::size_t idx = static_cast<std::size_t>(idxI);
    const float risk = fr.risk01[idx];
    const float cov = fr.coverage01[idx];

    const int prevLevel = static_cast<int>(t.level);
    const Overlay prevOverlay = t.overlay;
    const int prevOcc = static_cast<int>(t.occupants);
    const int prevJobsCap = JobsForTile(t);
    const int prevHousingCap = (t.overlay == Overlay::Residential) ? HousingForLevel(t.level) : 0;

    // Severity: higher risk and lower coverage => more destruction.
    const float sev = Clamp01(0.30f + 0.85f * risk + 0.25f * rng.nextF01());
    float destroyP = settings.destroyBase * (0.55f + sev) * (1.10f - 0.70f * Clamp01(cov));
    destroyP = std::clamp(destroyP, 0.05f, 0.85f);

    const bool destroyed = rng.chance(destroyP);
    if (destroyed) {
      out.destroyed++;
      if (prevOverlay == Overlay::Residential) {
        out.displaced += prevOcc;
      }
      if (prevJobsCap > 0) {
        out.jobsLostCap += prevJobsCap;
      }

      // Clear the tile.
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
      out.zoneLayoutChanged = true;
      continue;
    }

    out.damaged++;

    // Downgrade the building more often when severity is high.
    if (t.level > 1) {
      const float downP = std::clamp(0.55f + 0.35f * sev, 0.0f, 0.95f);
      if (rng.chance(downP)) {
        t.level = static_cast<std::uint8_t>(std::max(1, prevLevel - 1));
      }
    }

    // Evacuation / reduced occupancy.
    if (prevOverlay == Overlay::Residential) {
      const int cap = std::max(0, HousingForLevel(t.level));
      const float keepFrac = std::clamp(0.15f + 0.55f * Clamp01(cov) - 0.35f * sev, 0.0f, 1.0f);
      const int newOcc = std::clamp(static_cast<int>(std::lround(static_cast<float>(prevOcc) * keepFrac)), 0, cap);
      out.displaced += (prevOcc - newOcc);
      t.occupants = static_cast<std::uint16_t>(newOcc);
    } else {
      // Businesses shut down for the day (capacity is modeled by level/overlay).
      t.occupants = 0;
    }

    // Track lost job capacity caused by downgrade.
    if (prevJobsCap > 0) {
      const int afterCap = JobsForTile(t);
      out.jobsLostCap += std::max(0, prevJobsCap - afterCap);
    }
    // Track displaced residents due to downgrade cap reduction (even if occupants were already low).
    if (prevHousingCap > 0) {
      const int afterCap = HousingForLevel(t.level);
      // If cap shrank below current occupancy, the clamp above already handled it.
      (void)afterCap;
    }
  }

  const int totalTiles = out.damaged + out.destroyed;
  if (totalTiles <= 0) return FireIncidentOutcome{};

  // Costs and citywide penalty.
  out.cost = out.damaged * settings.costPerDamagedTile + out.destroyed * settings.costPerDestroyedTile;
  out.cost += ((out.displaced + 9) / 10) * settings.costPer10Displaced;
  out.cost += ((out.jobsLostCap + 9) / 10) * settings.costPer10JobsCapLost;

  float pen = settings.happinessPenaltyBase;
  pen += static_cast<float>(totalTiles) * settings.happinessPenaltyPerTile;
  pen += (static_cast<float>(out.displaced) / 100.0f) * settings.happinessPenaltyPer100Displaced;
  out.happinessPenalty = std::clamp(pen, 0.0f, settings.maxHappinessPenalty);

  out.happened = true;
  out.originX = ox;
  out.originY = oy;
  out.originDistrict = world.at(ox, oy).district;
  return out;
}

struct TrafficIncidentOutcome {
  bool happened = false;

  int injuries = 0;
  int cost = 0;
  float happinessPenalty = 0.0f;

  int originX = -1;
  int originY = -1;
  int originDistrict = -1;
};

TrafficIncidentOutcome TryApplyTrafficIncident(const World& world, const TrafficIncidentSettings& settings,
                                               const Stats& prevStats, int population, int zoneTileCount, RNG& rng)
{
  if (!settings.enabled) return TrafficIncidentOutcome{};
  if (population < settings.minPopulation) return TrafficIncidentOutcome{};
  if (zoneTileCount < settings.minZoneTiles) return TrafficIncidentOutcome{};

  int ox = prevStats.trafficSafetyHotspotX;
  int oy = prevStats.trafficSafetyHotspotY;
  if (!world.inBounds(ox, oy) || world.at(ox, oy).overlay != Overlay::Road) return TrafficIncidentOutcome{};

  const float exposure = std::clamp(prevStats.trafficSafetyResidentMeanExposure, 0.0f, 1.0f);
  const float hotspotRisk = std::clamp(prevStats.trafficSafetyHotspotRisk01, 0.0f, 1.0f);

  float chance = settings.baseChancePerDay;
  chance += settings.chancePer100Population * (static_cast<float>(population) / 100.0f);
  chance *= (1.0f + settings.exposureChanceBoost * exposure);
  chance *= (1.0f + settings.hotspotRiskChanceBoost * hotspotRisk);
  chance = std::clamp(chance, 0.0f, settings.maxChancePerDay);

  if (!rng.chance(chance)) return TrafficIncidentOutcome{};

  // Severity scaled by hotspot risk (0..1) with some randomness.
  const float bonusF = hotspotRisk * std::max(0.0f, settings.injuriesRiskBonus);
  const int extra = std::max(0, static_cast<int>(std::lround(bonusF)));
  const int baseMax = std::max(settings.minInjuries, settings.maxInjuries);
  const int injuriesCap = baseMax + std::max(0, static_cast<int>(std::ceil(std::max(0.0f, settings.injuriesRiskBonus))));

  const int base = settings.minInjuries + static_cast<int>(rng.rangeU32(static_cast<std::uint32_t>(baseMax - settings.minInjuries + 1)));
  const int injuries = std::clamp(base + extra, settings.minInjuries, injuriesCap);

  // Emergency response mitigation: strong safety services reduce injuries/cost/penalty.
  float responseFactor = 1.0f;
  if (prevStats.servicesSafetyFacilities <= 0) {
    responseFactor *= settings.noSafetyServicesMultiplier;
  } else {
    const float sat = std::clamp(prevStats.servicesSafetySatisfaction, 0.0f, 1.0f);
    responseFactor *= std::clamp(1.0f - settings.safetySatisfactionMitigation * sat, settings.minSafetyMitigation, 1.0f);
  }

  TrafficIncidentOutcome out{};
  out.happened = true;
  out.originX = ox;
  out.originY = oy;
  out.originDistrict = world.at(ox, oy).district;

  const int adjInjuries = std::clamp(static_cast<int>(std::lround(static_cast<float>(injuries) * responseFactor)), settings.minInjuries,
                                    injuriesCap);
  out.injuries = adjInjuries;

  const float rawPenalty = (settings.happinessPenaltyBase + static_cast<float>(adjInjuries) * settings.happinessPenaltyPerInjury);
  out.happinessPenalty = std::clamp(rawPenalty, 0.0f, settings.maxHappinessPenalty);

  const float rawCost = static_cast<float>(settings.costBase + adjInjuries * settings.costPerInjury);
  out.cost = std::max(0, static_cast<int>(std::lround(rawCost)));
  return out;
}

// Parks are modeled as an *area of influence* rather than a global ratio.
// We compute a simple "coverage" ratio: what fraction of zone tiles can reach
// any park (via their access road) within a travel-time threshold.
//
// Notes:
// - We treat Water as a barrier so disconnected islands don't share park benefits.
// - This is intentionally lightweight: a multi-source road isochrone + mapping
//   (still cheap on a 96x96 grid).
float ParkCoverageRatio(const World& world, int radius, const std::vector<std::uint8_t>* roadToEdge,
                        const ZoneAccessMap* zoneAccess)
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

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const bool edgeOk = (roadToEdge && roadToEdge->size() == n);

  // Sources are the road tiles adjacent to parks (optionally requiring outside connection).
  std::vector<std::uint8_t> srcMask(n, std::uint8_t{0});
  constexpr int kDirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Park) continue;
      if (t.terrain == Terrain::Water) continue;

      for (const auto& d : kDirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (!world.inBounds(nx, ny)) continue;
        const Tile& nt = world.at(nx, ny);
        if (nt.overlay != Overlay::Road) continue;
        const std::size_t ridx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) +
                                 static_cast<std::size_t>(nx);
        if (ridx >= n) continue;
        if (edgeOk && (*roadToEdge)[ridx] == 0) continue;
        srcMask[ridx] = 1;
      }
    }
  }

  std::vector<int> sources;
  sources.reserve(n / 64);
  for (std::size_t i = 0; i < n; ++i) {
    if (srcMask[i] != 0) sources.push_back(static_cast<int>(i));
  }

  if (sources.empty()) return 0.0f;

  RoadIsochroneConfig icfg;
  icfg.requireOutsideConnection = edgeOk;
  icfg.weightMode = IsochroneWeightMode::TravelTime;
  icfg.computeOwner = false;

  const RoadIsochroneField roadField = BuildRoadIsochroneField(world, sources, icfg, edgeOk ? roadToEdge : nullptr);

  TileAccessCostConfig tcfg;
  tcfg.includeRoadTiles = false;
  tcfg.includeZones = true;
  tcfg.includeNonZonesAdjacentToRoad = true;
  tcfg.includeWater = false;
  // Walk from zone parcel to its access road.
  tcfg.accessStepCostMilli = 1000;
  tcfg.useZoneAccessMap = true;

  const ZoneAccessMap* zam = nullptr;
  if (zoneAccess && zoneAccess->w == w && zoneAccess->h == h && zoneAccess->roadIdx.size() == n) {
    zam = zoneAccess;
  }

  const std::vector<int> tileCost =
      BuildTileAccessCostField(world, roadField, tcfg, edgeOk ? roadToEdge : nullptr, zam);

  const int thresholdMilli = std::max(0, radius) * 1000;
  int zones = 0;
  int covered = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                           t.overlay == Overlay::Industrial);
      if (!isZone) continue;
      zones++;
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      if (idx >= tileCost.size()) continue;
      const int c = tileCost[idx];
      if (c >= 0 && c <= thresholdMilli) covered++;
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
  (void)updateLimited(world, dt, /*maxTicks=*/0, /*maxBacklogTicks=*/0, nullptr);
}

int Simulator::update(World& world, float dt, std::vector<Stats>* outTickStats)
{
  return updateLimited(world, dt, /*maxTicks=*/0, /*maxBacklogTicks=*/0, outTickStats);
}

int Simulator::updateLimited(World& world, float dt, int maxTicks, int maxBacklogTicks,
                             std::vector<Stats>* outTickStats)
{
  // Reject negative or NaN dt (NaN fails all comparisons).
  if (!(dt > 0.0f)) return 0;

  const float tickSec = m_cfg.tickSeconds;
  if (!(tickSec > 1.0e-6f)) {
    // Degenerate tick size; avoid infinite loops.
    m_accum = 0.0f;
    return 0;
  }

  m_accum += dt;
  if (!(m_accum >= 0.0f)) {
    // Handles NaN and negative accumulation.
    m_accum = 0.0f;
    return 0;
  }

  if (maxBacklogTicks > 0) {
    const float maxBacklogSec = tickSec * static_cast<float>(maxBacklogTicks);
    if (m_accum > maxBacklogSec) m_accum = maxBacklogSec;
  }

  const bool limitTicks = (maxTicks > 0);
  int ticks = 0;

  while (m_accum >= tickSec) {
    if (limitTicks && ticks >= maxTicks) break;

    m_accum -= tickSec;
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
  int jobsCapCommercialAccessible = 0;
  int jobsCapIndustrialAccessible = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;
      if (!HasZoneAccess(*zoneAccess, x, y)) continue;
      const int cap = JobsForTile(t);
      jobsCapAccessible += cap;
      if (t.overlay == Overlay::Commercial) jobsCapCommercialAccessible += cap;
      else if (t.overlay == Overlay::Industrial) jobsCapIndustrialAccessible += cap;
    }
  }

  // Employment: fill accessible jobs up to population.
  const int employed = std::min(scan.population, jobsCapAccessible);

  // --- Derived transit stats ---
  // These are always recomputed here (not persisted).
  s.transitLines = 0;
  s.transitStops = 0;
  s.transitRiders = 0;
  s.transitModeShare = 0.0f;
  s.transitCommuteCoverage = 0.0f;
  s.transitCost = 0;

  // --- Derived public services / civic accessibility stats ---
  s.servicesEducationFacilities = 0;
  s.servicesHealthFacilities = 0;
  s.servicesSafetyFacilities = 0;
  s.servicesEducationSatisfaction = 0.0f;
  s.servicesHealthSatisfaction = 0.0f;
  s.servicesSafetySatisfaction = 0.0f;
  s.servicesOverallSatisfaction = 0.0f;
  s.servicesMaintenanceCost = 0;

  bool servicesActive = false;
  float servicesOverallSat = 0.0f;
  int servicesMaint = 0;

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
  tc.capacityAwareJobs = m_trafficModel.capacityAwareJobs;
  tc.jobAssignmentIterations = m_trafficModel.jobAssignmentIterations;
  tc.jobPenaltyBaseMilli = m_trafficModel.jobPenaltyBaseMilli;
  const TrafficResult trafficBase = ComputeCommuteTraffic(world, tc, employedShare, roadToEdge, zoneAccess);

  // Road traffic is car-only. When transit is disabled, this is simply the full commute traffic.
  TrafficResult trafficRoad = trafficBase;

  // Commute stats presented to the player represent *all* reachable commuters (car + transit).
  float avgCommuteAll = trafficBase.avgCommute;
  float p95CommuteAll = trafficBase.p95Commute;
  float avgCommuteTimeAll = trafficBase.avgCommuteTime;
  float p95CommuteTimeAll = trafficBase.p95CommuteTime;

  // Procedural trade market: decide today's outside trade conditions.
  //
  // The resulting import/export capacity throttles are fed into the goods model.
  TradeMarketSummary tradePlan;
  if (m_tradeModel.enabled) {
    tradePlan = PlanTradeMarket(world, s.day, m_tradeModel);
  } else {
    // Legacy behavior: full availability at a fixed exchange rate.
    tradePlan.day = s.day;
    tradePlan.chosenImportPartner = -1;
    tradePlan.chosenExportPartner = -1;
    tradePlan.importCapacityPct = m_tradeModel.allowImports ? 100 : 0;
    tradePlan.exportCapacityPct = m_tradeModel.allowExports ? 100 : 0;
    tradePlan.importDisrupted = false;
    tradePlan.exportDisrupted = false;
    tradePlan.marketIndex = 1.0f;
  }

  // Procedural macro economy (optional): compute a deterministic daily snapshot and
  // feed its multipliers into the goods model and budget/happiness calculations.
  EconomySnapshot eco{};
  std::array<float, static_cast<std::size_t>(kDistrictCount)> econTaxBaseMult{};
  econTaxBaseMult.fill(1.0f);

  std::vector<float> econIndSupplyMult;
  std::vector<float> econComDemandMult;

  if (m_economyModel.enabled) {
    eco = ComputeEconomySnapshot(world, s.day, m_economyModel);

    s.economyIndex = eco.economyIndex;
    s.economyInflation = eco.inflation;
    s.economyCityWealth = eco.cityWealth;
    s.economyEventKind = static_cast<int>(eco.activeEvent.kind);
    s.economyEventDaysLeft = std::max(0, eco.activeEventDaysLeft);

    for (int d = 0; d < kDistrictCount; ++d) {
      const float m = eco.districts[static_cast<std::size_t>(d)].taxBaseMult;
      econTaxBaseMult[static_cast<std::size_t>(d)] = std::max(0.0f, m);
    }

    // Build per-tile multipliers (only meaningful on job-zone tiles).
    econIndSupplyMult.assign(n, 1.0f);
    econComDemandMult.assign(n, 1.0f);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const Tile& t = world.at(x, y);
        const int d = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        if (idx >= n) continue;

        if (t.overlay == Overlay::Industrial) {
          econIndSupplyMult[idx] = eco.districts[static_cast<std::size_t>(d)].industrialSupplyMult;
        } else if (t.overlay == Overlay::Commercial) {
          econComDemandMult[idx] = eco.districts[static_cast<std::size_t>(d)].commercialDemandMult;
        }
      }
    }
  } else {
    // Baseline values so UI/debug output is stable.
    s.economyIndex = 1.0f;
    s.economyInflation = 0.0f;
    s.economyCityWealth = 0.5f;
    s.economyEventKind = 0;
    s.economyEventDaysLeft = 0;
  }

  // Goods/logistics model: route industrial output to commercial demand along roads.
  GoodsConfig gc;
  gc.requireOutsideConnection = m_cfg.requireOutsideConnection;
  gc.allowImports = m_tradeModel.allowImports;
  gc.allowExports = m_tradeModel.allowExports;
  gc.importCapacityPct = std::clamp(tradePlan.importCapacityPct, 0, 100);
  gc.exportCapacityPct = std::clamp(tradePlan.exportCapacityPct, 0, 100);
  if (m_economyModel.enabled) {
    gc.industrialSupplyMult = &econIndSupplyMult;
    gc.commercialDemandMult = &econComDemandMult;
  }
  const GoodsResult goods = ComputeGoodsFlow(world, gc, roadToEdge, zoneAccess);
  s.goodsProduced = goods.goodsProduced;
  s.goodsDemand = goods.goodsDemand;
  s.goodsDelivered = goods.goodsDelivered;
  s.goodsImported = goods.goodsImported;
  s.goodsExported = goods.goodsExported;
  s.goodsUnreachableDemand = goods.unreachableDemand;
  s.goodsSatisfaction = goods.satisfaction;
  s.maxRoadGoodsTraffic = goods.maxRoadGoodsTraffic;


  // ---------------------------------------------------------------------------
  // Transit mode shift model (optional)
  // ---------------------------------------------------------------------------
  // This is a lightweight "first layer": we plan a set of bus lines on the RoadGraph
  // using an aggregated demand signal, then estimate (a) how much commute demand those
  // corridors cover and (b) what fraction of commuters would shift away from cars.
  //
  // The resulting effect is:
  //   - reduced road traffic + congestion (car commuters only)
  //   - improved average commute time (blended car + transit)
  //   - an operating cost line item in the budget
  if (m_transitModel.enabled && trafficBase.reachableCommuters > 0) {
    RoadGraph g = BuildRoadGraph(world);

    if (!g.edges.empty()) {
      const std::size_t nTiles = n;

      // Build a per-road-tile demand signal.
      std::vector<std::uint32_t> roadFlow(nTiles, 0u);
      const TransitDemandMode dm = m_transitModel.demandMode;

      const bool needCommute = (dm == TransitDemandMode::Commute || dm == TransitDemandMode::Combined);
      const bool needGoods = (dm == TransitDemandMode::Goods || dm == TransitDemandMode::Combined);

      if (needCommute && trafficBase.roadTraffic.size() == nTiles) {
        for (std::size_t i = 0; i < nTiles; ++i) {
          roadFlow[i] += static_cast<std::uint32_t>(trafficBase.roadTraffic[i]);
        }
      }
      if (needGoods && goods.roadGoodsTraffic.size() == nTiles) {
        for (std::size_t i = 0; i < nTiles; ++i) {
          roadFlow[i] += static_cast<std::uint32_t>(goods.roadGoodsTraffic[i]);
        }
      }

      // Aggregate road-tile demand onto the compressed RoadGraph edges.
      const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(world, g, roadFlow);
      std::vector<std::uint64_t> edgeDemand(g.edges.size(), 0ull);
      for (std::size_t ei = 0; ei < g.edges.size() && ei < agg.edges.size(); ++ei) {
        // Prefer interior demand to avoid double-counting nodes across adjacent edges.
        edgeDemand[ei] = agg.edges[ei].sumTrafficInterior;
      }

      // Commute-only edge demand, used for coverage + mode share estimation.
      std::vector<std::uint64_t> commuteEdgeDemand(g.edges.size(), 0ull);
      if (trafficBase.roadTraffic.size() == nTiles) {
        std::vector<std::uint32_t> commuteFlow(nTiles, 0u);
        for (std::size_t i = 0; i < nTiles; ++i) {
          commuteFlow[i] = static_cast<std::uint32_t>(trafficBase.roadTraffic[i]);
        }
        const RoadGraphTrafficResult aggC = AggregateFlowOnRoadGraph(world, g, commuteFlow);
        for (std::size_t ei = 0; ei < g.edges.size() && ei < aggC.edges.size(); ++ei) {
          commuteEdgeDemand[ei] = aggC.edges[ei].sumTrafficInterior;
        }
      }

      // Planner config: keep the default deterministic per-world unless the user overrides seedSalt.
      TransitPlannerConfig pcfg = m_transitModel.plannerCfg;
      if (pcfg.seedSalt == 0) {
        pcfg.seedSalt = (world.seed() ^ 0xA2B3C4D5E6F70911ULL) ^ (static_cast<std::uint64_t>(dm) * 0x9E3779B97F4A7C15ULL);
      }

      TransitPlan plan = PlanTransitLines(g, edgeDemand, pcfg, &world);
      plan.cfg = pcfg;

      // Track which edges are served by at least one line.
      std::vector<std::uint8_t> served(g.edges.size(), std::uint8_t{0});
      for (const TransitLine& line : plan.lines) {
        for (int ei : line.edges) {
          if (ei >= 0 && static_cast<std::size_t>(ei) < served.size()) {
            served[static_cast<std::size_t>(ei)] = 1u;
          }
        }
      }

      // Coverage of commute demand along served corridors (edge-based).
      std::uint64_t commuteTotal = 0ull;
      std::uint64_t commuteCovered = 0ull;
      for (std::size_t ei = 0; ei < commuteEdgeDemand.size(); ++ei) {
        commuteTotal += commuteEdgeDemand[ei];
        if (served[ei]) commuteCovered += commuteEdgeDemand[ei];
      }

      const float corridorCoverage =
          (commuteTotal > 0)
              ? std::clamp(static_cast<float>(static_cast<double>(commuteCovered) / static_cast<double>(commuteTotal)),
                           0.0f, 1.0f)
              : 0.0f;

      // Unique served road tiles (for cost accounting).
      std::vector<std::uint8_t> servedTileMask(nTiles, std::uint8_t{0});
      for (std::size_t ei = 0; ei < served.size() && ei < g.edges.size(); ++ei) {
        if (!served[ei]) continue;
        for (const Point& p : g.edges[ei].tiles) {
          if (!world.inBounds(p.x, p.y)) continue;
          const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(p.x);
          if (idx < servedTileMask.size()) servedTileMask[idx] = 1u;
        }
      }
      int servedTileCount = 0;
      for (std::uint8_t b : servedTileMask) servedTileCount += (b != 0);

      // Stops: deterministically sample each line and collect unique stop road tiles.
      const int stopSpacing = std::max(2, m_transitModel.stopSpacingTiles);
      std::vector<int> stopRoadIdx;
      stopRoadIdx.reserve(256);
      std::vector<std::uint8_t> stopSeen(nTiles, std::uint8_t{0});
      std::vector<Point> tmpStops;
      tmpStops.reserve(256);

      auto addStop = [&](const Point& p) {
        if (!world.inBounds(p.x, p.y)) return;
        const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w) +
                                static_cast<std::size_t>(p.x);
        if (idx >= stopSeen.size() || stopSeen[idx]) return;

        const Tile& t = world.at(p.x, p.y);
        if (t.overlay != Overlay::Road) return;

        stopSeen[idx] = 1u;
        stopRoadIdx.push_back(static_cast<int>(idx));
      };

      for (const TransitLine& line : plan.lines) {
        tmpStops.clear();
        if (!BuildTransitLineStopTiles(g, line, stopSpacing, tmpStops)) continue;
        for (const Point& p : tmpStops) addStop(p);
      }

      const int stopCount = static_cast<int>(stopRoadIdx.size());

      // Stop-access coverage: share of residents and jobs within walking distance of any stop.
      //
      // We approximate walking distance using *unweighted road steps* to the nearest stop (isochrone Steps mode).
      // A commuter needs access at both ends, so we combine origin/destination accessibility with a geometric mean.
      // kWalkRadiusSteps is a rule-of-thumb walk-to-stop radius (similar to the common ~400m / 5-minute
      // transit service area), and matches the 10-step bucket used by the transitplan CLI access summary.
      constexpr int kWalkRadiusSteps = 10;

      std::uint64_t resTotal = 0ull;
      std::uint64_t resServed = 0ull;
      std::uint64_t jobsTotal = 0ull;
      std::uint64_t jobsServed = 0ull;

      // Average walk distance to the nearest stop for served tiles (steps).
      double resWalkAvg = 0.0;
      double jobsWalkAvg = 0.0;

      float accessCoverage = 0.0f;

      if (!stopRoadIdx.empty()) {
        RoadIsochroneConfig icfg;
        icfg.requireOutsideConnection = (roadToEdge != nullptr);
        icfg.weightMode = IsochroneWeightMode::Steps;
        icfg.computeOwner = false;

        const RoadIsochroneField stopField =
            BuildRoadIsochroneField(world, stopRoadIdx, icfg, roadToEdge, nullptr);

        std::uint64_t resWalkSum = 0ull;
        std::uint64_t resWalkW = 0ull;
        std::uint64_t jobsWalkSum = 0ull;
        std::uint64_t jobsWalkW = 0ull;

        for (int y = 0; y < h; ++y) {
          for (int x = 0; x < w; ++x) {
            const std::size_t tidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                     static_cast<std::size_t>(x);
            if (tidx >= zoneAccess->roadIdx.size()) continue;

            const int ridx = zoneAccess->roadIdx[tidx];
            if (ridx < 0) continue;

            const Tile& t = world.at(x, y);
            if (t.occupants <= 0) continue;
            const std::uint64_t wgt = static_cast<std::uint64_t>(t.occupants);

            const int steps = (static_cast<std::size_t>(ridx) < stopField.steps.size())
                                ? stopField.steps[static_cast<std::size_t>(ridx)]
                                : -1;
	            const bool tileServed = (steps >= 0 && steps <= kWalkRadiusSteps);

            if (t.overlay == Overlay::Residential) {
              resTotal += wgt;
	              if (tileServed) {
                resServed += wgt;
                resWalkSum += static_cast<std::uint64_t>(steps) * wgt;
                resWalkW += wgt;
              }
            } else if (t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
              jobsTotal += wgt;
	              if (tileServed) {
                jobsServed += wgt;
                jobsWalkSum += static_cast<std::uint64_t>(steps) * wgt;
                jobsWalkW += wgt;
              }
            }
          }
        }

        const float resShare =
            (resTotal > 0)
                ? std::clamp(static_cast<float>(static_cast<double>(resServed) / static_cast<double>(resTotal)),
                             0.0f, 1.0f)
                : 0.0f;
        const float jobsShare =
            (jobsTotal > 0)
                ? std::clamp(static_cast<float>(static_cast<double>(jobsServed) / static_cast<double>(jobsTotal)),
                             0.0f, 1.0f)
                : 0.0f;

        accessCoverage =
            std::clamp(static_cast<float>(std::sqrt(static_cast<double>(resShare) * static_cast<double>(jobsShare))),
                       0.0f, 1.0f);

        if (resWalkW > 0) resWalkAvg = static_cast<double>(resWalkSum) / static_cast<double>(resWalkW);
        if (jobsWalkW > 0) jobsWalkAvg = static_cast<double>(jobsWalkSum) / static_cast<double>(jobsWalkW);
      }

      // Final commute coverage used by the transit mode-shift model combines:
      //  - corridorCoverage: are we serving the high-demand road corridors?
      //  - accessCoverage: can residents/jobs actually walk to a stop?
      const float coverage = std::clamp(corridorCoverage * accessCoverage, 0.0f, 1.0f);

      // Ridership model.
      const float service = std::max(0.0f, m_transitModel.serviceLevel);
      const float maxShare = std::clamp(m_transitModel.maxModeShare, 0.0f, 1.0f);
      const float tMult = std::clamp(m_transitModel.travelTimeMultiplier, 0.25f, 2.5f);

      // A smooth saturating function: higher coverage + higher service + faster travel => more shift.
      const float attractiveness = (tMult > 1.0e-3f) ? (service / tMult) : service;
      const float base = std::max(0.0f, coverage) * std::max(0.0f, attractiveness);
      const float modeShare = maxShare * (1.0f - static_cast<float>(std::exp(-1.2 * static_cast<double>(base))));
      const int riders = std::clamp(static_cast<int>(std::lround(static_cast<double>(trafficBase.reachableCommuters) * static_cast<double>(modeShare))),
                                    0, trafficBase.reachableCommuters);

      s.transitLines = static_cast<int>(plan.lines.size());
      s.transitStops = stopCount;
      s.transitRiders = riders;
      s.transitModeShare = (trafficBase.totalCommuters > 0)
                             ? std::clamp(static_cast<float>(riders) / static_cast<float>(trafficBase.totalCommuters), 0.0f, 1.0f)
                             : 0.0f;
      s.transitCommuteCoverage = coverage;

      // Operating cost: proportional to network footprint + stop count, scaled by service.
      const int costPerTile = std::max(0, m_transitModel.costPerTile);
      const int costPerStop = std::max(0, m_transitModel.costPerStop);
      const double rawCost = static_cast<double>(service) *
                             (static_cast<double>(servedTileCount) * static_cast<double>(costPerTile) +
                              static_cast<double>(stopCount) * static_cast<double>(costPerStop));
      s.transitCost = std::max(0, static_cast<int>(std::lround(rawCost)));

      // Recompute road traffic using the reduced car commuter share.
      const float employedShareCar = std::max(0.0f, employedShare * (1.0f - s.transitModeShare));
      trafficRoad = ComputeCommuteTraffic(world, tc, employedShareCar, roadToEdge, zoneAccess);

      // Blend commute times for happiness/UI.
      const int reachableAll = trafficBase.reachableCommuters;
      const int carReachable = std::max(0, reachableAll - riders);
      if (reachableAll > 0) {
        const float wCar = static_cast<float>(carReachable) / static_cast<float>(reachableAll);
        const float wT = static_cast<float>(riders) / static_cast<float>(reachableAll);

        // Transit adds a small wait/dwell penalty that decreases with service.
        const float waitPenalty = 3.0f / std::max(0.25f, service);

        // Walking to/from stops: average steps-to-nearest-stop at origins + destinations.
        // We scale it modestly: RoadTravelTimeMilliForLevel models vehicle speeds, but this is meant
        // as a lightweight accessibility penalty in the same "street-step" units as avgCommuteTime.
        constexpr float kWalkTimeMultiplier = 1.5f;
        const float walkPenalty =
            static_cast<float>(std::max(0.0, resWalkAvg) + std::max(0.0, jobsWalkAvg)) * kWalkTimeMultiplier;

        const float transitAvgTime = trafficBase.avgCommuteTime * tMult + waitPenalty + walkPenalty;
        const float transitP95Time = trafficBase.p95CommuteTime * tMult + waitPenalty * 1.5f + walkPenalty * 1.5f;

        avgCommuteAll = trafficRoad.avgCommute * wCar + trafficBase.avgCommute * wT;
        avgCommuteTimeAll = trafficRoad.avgCommuteTime * wCar + transitAvgTime * wT;

        p95CommuteAll = std::max(trafficRoad.p95Commute, trafficBase.p95Commute);
        p95CommuteTimeAll = std::max(trafficRoad.p95CommuteTime, transitP95Time);
      }
    }
  }

  // Commit the traffic-derived stats.
  s.commuters = trafficBase.totalCommuters;
  s.commutersUnreachable = trafficBase.unreachableCommuters;
  s.avgCommute = avgCommuteAll;
  s.p95Commute = p95CommuteAll;
  s.avgCommuteTime = avgCommuteTimeAll;
  s.p95CommuteTime = p95CommuteTimeAll;
  s.trafficCongestion = trafficRoad.congestion;
  s.congestedRoadTiles = trafficRoad.congestedRoadTiles;
  s.maxRoadTraffic = trafficRoad.maxTraffic;

  // Traffic safety (derived; does not require the app/UI).
  //
  // We compute a resident-weighted exposure/priority metric and a deterministic
  // high-risk road "hotspot" for use by the (optional) traffic incident system.
  s.trafficSafetyRoadTilesConsidered = 0;
  s.trafficSafetyResidentPopulation = 0;
  s.trafficSafetyResidentMeanExposure = 0.0f;
  s.trafficSafetyResidentMeanPriority = 0.0f;
  s.trafficSafetyHappinessPenalty = 0.0f;
  s.trafficSafetyHotspotX = -1;
  s.trafficSafetyHotspotY = -1;
  s.trafficSafetyHotspotDistrict = -1;
  s.trafficSafetyHotspotRisk01 = 0.0f;

  if (m_trafficSafetyModel.enabled) {
    TrafficSafetyConfig tscfg = m_trafficSafetyModel.cfg;
    tscfg.enabled = true;
    tscfg.requireOutsideConnection = m_cfg.requireOutsideConnection;
    // Keep the simulator headless/cheap: avoid SkyView (canyon) unless explicitly precomputed.
    tscfg.canyonWeight = 0.0f;

    const TrafficSafetyResult ts = ComputeTrafficSafety(world, tscfg, &trafficRoad, /*skyView=*/nullptr,
                                                        tscfg.requireOutsideConnection ? roadToEdge : nullptr);

    s.trafficSafetyResidentPopulation = std::max(0, ts.residentPopulation);
    s.trafficSafetyResidentMeanExposure = std::clamp(ts.residentMeanExposure, 0.0f, 1.0f);
    s.trafficSafetyResidentMeanPriority = std::clamp(ts.residentMeanPriority, 0.0f, 1.0f);
    s.trafficSafetyRoadTilesConsidered = std::max(0, ts.roadTilesConsidered);

    // Continuous, citywide happiness penalty (mild by default).
    const float pen = s.trafficSafetyResidentMeanExposure * std::max(0.0f, m_trafficSafetyModel.happinessPenaltyScale);
    s.trafficSafetyHappinessPenalty = std::min(std::max(0.0f, m_trafficSafetyModel.maxHappinessPenalty),
                                               std::max(0.0f, pen));

    // Pick a deterministic high-risk road tile for UI/news/incident location.
    // Use integer weights for stability.
    const int w = world.width();
    const int h = world.height();
    const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    if (w > 0 && h > 0 && ts.risk01.size() == n) {
      uint64_t totalW = 0;
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const Tile& t = world.at(x, y);
          if (t.overlay != Overlay::Road) continue;
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
          const float r = ts.risk01[idx];
          const int rq = Float01ToQ16(std::clamp(r, 0.0f, 1.0f));
          if (rq <= 0) continue;
          const uint64_t wgt = static_cast<uint64_t>(rq) * static_cast<uint64_t>(rq);
          if (wgt == 0) continue;
          totalW += wgt;
        }
      }

      if (totalW > 0) {
        // Separate RNG stream from other subsystems.
        const uint64_t seed = world.seed() ^ (static_cast<uint64_t>(s.day) * 0xD6E8FEB86659FD93ULL) ^ 0xA1C1D7E5BADC0FFELL;
        RNG rng(seed);
        const uint64_t pick = rng.nextU64() % totalW;
        uint64_t acc = 0;
        bool chosen = false;

        for (int y = 0; y < h && !chosen; ++y) {
          for (int x = 0; x < w && !chosen; ++x) {
            const Tile& t = world.at(x, y);
            if (t.overlay != Overlay::Road) continue;
            const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            const float r = ts.risk01[idx];
            const int rq = Float01ToQ16(std::clamp(r, 0.0f, 1.0f));
            if (rq <= 0) continue;
            const uint64_t wgt = static_cast<uint64_t>(rq) * static_cast<uint64_t>(rq);
            if (wgt == 0) continue;
            acc += wgt;
            if (acc > pick) {
              s.trafficSafetyHotspotX = x;
              s.trafficSafetyHotspotY = y;
              s.trafficSafetyHotspotDistrict = t.district;
              s.trafficSafetyHotspotRisk01 = std::clamp(r, 0.0f, 1.0f);
              chosen = true;
            }
          }
        }
      }
    }
  }

  // Land value (amenities + pollution + optional traffic spill). Used both for
  // display and for the simple tax model.
  LandValueConfig lvc;
  lvc.requireOutsideConnection = m_cfg.requireOutsideConnection;
  const LandValueResult lv = ComputeLandValue(world, lvc, &trafficRoad, roadToEdge);
  s.avgLandValue = AvgLandValueNonWater(world, lv);

  // Air quality (derived; headless).
  //
  // Uses the AirPollution model to estimate resident-weighted exposure based on
  // land use + traffic + goods movement. This feeds into a mild citywide
  // happiness penalty to encourage zoning buffers, parks, and congestion reduction.
  s.airPollutionResidentPopulation = 0;
  s.airPollutionResidentAvg01 = 0.0f;
  s.airPollutionResidentHighExposureFrac = 0.0f;
  s.airPollutionHappinessPenalty = 0.0f;

  if (m_airPollutionModel.enabled) {
    AirPollutionConfig apcfg = m_airPollutionModel.cfg;
    // Ensure deterministic wind if requested.
    // (ComputeAirPollution uses world.seed() when windFromSeed is enabled.)
    const AirPollutionResult ap = ComputeAirPollution(world, apcfg, &trafficRoad, &goods);

    s.airPollutionResidentPopulation = std::max(0, ap.residentPopulation);
    s.airPollutionResidentAvg01 = std::clamp(ap.residentAvgPollution01, 0.0f, 1.0f);
    s.airPollutionResidentHighExposureFrac = std::clamp(ap.residentHighExposureFrac, 0.0f, 1.0f);

    const float avgScale = std::max(0.0f, m_airPollutionModel.happinessPenaltyScale);
    const float highScale = std::max(0.0f, m_airPollutionModel.highExposurePenaltyScale);
    const float maxPen = std::max(0.0f, m_airPollutionModel.maxHappinessPenalty);

    const float raw = s.airPollutionResidentAvg01 * avgScale + s.airPollutionResidentHighExposureFrac * highScale;
    s.airPollutionHappinessPenalty = std::clamp(raw, 0.0f, maxPen);
  }

  // Public services / civic accessibility (optional).
  //
  // This is a headless accessibility-to-satisfaction field driven by explicit
  // service facility tiles (schools, hospitals, etc).
  {
    const std::vector<ServiceFacility> facilities = ExtractServiceFacilitiesFromWorld(world);
    const bool autoEnable = !facilities.empty();

    if (m_servicesModel.enabled || autoEnable) {
      ServicesModelSettings cfg = m_servicesModel;
      cfg.enabled = true;
      // The simulator may compute ZoneAccessMap / road-to-edge masks for other systems
      // based on the *global* outside-connection rule (m_cfg.requireOutsideConnection).
      //
      // The services model has its own outside-connection toggle. If it differs from the
      // global sim setting, we must not reuse those caches or we can over/underestimate
      // accessibility (especially for disconnected road components).
      const bool reuseAccessCaches = (cfg.requireOutsideConnection == m_cfg.requireOutsideConnection);
      const ZoneAccessMap* servicesZoneAccess = reuseAccessCaches ? zoneAccess : nullptr;
      const std::vector<std::uint8_t>* servicesRoadToEdge =
        (reuseAccessCaches && cfg.requireOutsideConnection) ? roadToEdge : nullptr;

      const ServicesResult sr = ComputeServices(world, cfg, facilities, servicesZoneAccess, servicesRoadToEdge);

      s.servicesEducationFacilities = sr.activeFacilities[static_cast<std::size_t>(ServiceType::Education)];
      s.servicesHealthFacilities = sr.activeFacilities[static_cast<std::size_t>(ServiceType::Health)];
      s.servicesSafetyFacilities = sr.activeFacilities[static_cast<std::size_t>(ServiceType::Safety)];

      s.servicesEducationSatisfaction = sr.educationSatisfaction;
      s.servicesHealthSatisfaction = sr.healthSatisfaction;
      s.servicesSafetySatisfaction = sr.safetySatisfaction;
      s.servicesOverallSatisfaction = sr.overallSatisfaction;

      servicesActive = true;
      servicesOverallSat = sr.overallSatisfaction;
      servicesMaint = sr.maintenanceCostPerDay;
      s.servicesMaintenanceCost = servicesMaint;
    }
  }

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
      const int d = clampDistrict(static_cast<int>(t.district));
      const float econMult = econTaxBaseMult[static_cast<std::size_t>(d)];
      const float raw = static_cast<float>(t.occupants) * static_cast<float>(taxPerOcc) * lvMult * taxMult * econMult;
      const int add = std::max(0, static_cast<int>(std::lround(raw)));
      taxRevenue += add;
    }
  }

  const int maintenance = roadMaint + parkMaint + servicesMaint;

  // Trade: compute import cost + export revenue.
  //
  // When the trade market is disabled, fall back to the legacy fixed exchange
  // rates.
  int importCost = goods.goodsImported / 20;
  int exportRevenue = goods.goodsExported / 25;

  // Populate trade snapshot fields (for UI/debug) regardless of the chosen model.
  s.tradeImportPartner = tradePlan.chosenImportPartner;
  s.tradeExportPartner = tradePlan.chosenExportPartner;
  s.tradeImportCapacityPct = std::clamp(tradePlan.importCapacityPct, 0, 100);
  s.tradeExportCapacityPct = std::clamp(tradePlan.exportCapacityPct, 0, 100);
  s.tradeImportDisrupted = tradePlan.importDisrupted;
  s.tradeExportDisrupted = tradePlan.exportDisrupted;
  s.tradeMarketIndex = tradePlan.marketIndex;

  if (m_tradeModel.enabled) {
    const TradeMarketResult tr = ComputeTradeMarket(world, s.day, m_tradeModel, goods, tradePlan);
    importCost = tr.importCost;
    exportRevenue = tr.exportRevenue;

    // Copy the realized daily plan (should match tradePlan, but we treat the trade
    // module as the source of truth for derived UI fields).
    s.tradeImportPartner = tr.summary.chosenImportPartner;
    s.tradeExportPartner = tr.summary.chosenExportPartner;
    s.tradeImportCapacityPct = std::clamp(tr.summary.importCapacityPct, 0, 100);
    s.tradeExportCapacityPct = std::clamp(tr.summary.exportCapacityPct, 0, 100);
    s.tradeImportDisrupted = tr.summary.importDisrupted;
    s.tradeExportDisrupted = tr.summary.exportDisrupted;
    s.tradeMarketIndex = tr.summary.marketIndex;
  }

  s.taxRevenue = taxRevenue;
  s.maintenanceCost = maintenance;
  s.upgradeCost = 0;
  s.importCost = importCost;
  s.exportRevenue = exportRevenue;
  s.income = taxRevenue + exportRevenue;
  // Note: transitCost is computed earlier in this function (and is 0 when transit is disabled).
  s.expenses = maintenance + importCost + s.transitCost;
  s.avgTaxPerCapita =
    (scan.population > 0) ? (static_cast<float>(taxRevenue) / static_cast<float>(scan.population)) : 0.0f;

  // Happiness: parks help (locally), unemployment hurts, and commutes/congestion add friction.
  const float parkCoverage = ParkCoverageRatio(world, m_cfg.parkInfluenceRadius, roadToEdge, zoneAccess);
  const float parkBonus = std::min(0.25f, parkCoverage * 0.35f);

  const float unemployment = (scan.population > 0)
                                 ? (1.0f - (static_cast<float>(employed) / static_cast<float>(scan.population)))
                                 : 0.0f;

  const float commuteNorm = (trafficBase.reachableCommuters > 0)
                                ? std::clamp(s.avgCommuteTime / kCommuteTarget, 0.0f, 2.0f)
                                : 0.0f;
  const float commutePenalty = std::min(kCommutePenaltyCap, commuteNorm * kCommutePenaltyCap);
  const float congestionPenalty = std::min(kCongestionPenaltyCap, trafficRoad.congestion * (kCongestionPenaltyCap * 1.35f));

  const float goodsPenalty = std::min(kGoodsPenaltyCap, (1.0f - goods.satisfaction) * kGoodsPenaltyCap);

  const float taxPenalty = std::min(0.20f, s.avgTaxPerCapita * std::max(0.0f, m_cfg.taxHappinessPerCapita));
  const float inflationPenalty = std::min(0.06f, std::max(0.0f, s.economyInflation) * 1.25f);
  const float lvBonus = std::clamp((s.avgLandValue - 0.50f) * 0.10f, -0.05f, 0.05f);

  const float firePenalty = std::clamp(s.fireIncidentHappinessPenalty, 0.0f, 0.35f);

  const float trafficSafetyPenalty =
      std::clamp(s.trafficSafetyHappinessPenalty, 0.0f, std::max(0.0f, m_trafficSafetyModel.maxHappinessPenalty));

  const float trafficIncidentPenalty =
      std::clamp(s.trafficIncidentHappinessPenalty, 0.0f, std::max(0.0f, m_trafficIncidents.maxHappinessPenalty));

  const float airPollutionPenalty =
      std::clamp(s.airPollutionHappinessPenalty, 0.0f, std::max(0.0f, m_airPollutionModel.maxHappinessPenalty));

  float servicesBonus = 0.0f;
  if (servicesActive && scan.population > 0) {
    const float sat = std::clamp(servicesOverallSat, 0.0f, 1.0f);
    // Neutral around 0.5; modest boost/penalty range.
    servicesBonus = std::clamp((sat - 0.5f) * 0.20f, -0.10f, 0.10f);
  }

  s.happiness = Clamp01(0.45f + parkBonus + lvBonus + servicesBonus - unemployment * 0.35f - commutePenalty - congestionPenalty - goodsPenalty - taxPenalty - inflationPenalty - firePenalty - trafficSafetyPenalty - trafficIncidentPenalty - airPollutionPenalty);

  // Demand meter (for UI/debug): recompute using the newly derived happiness.
  const float jobPressure = (scan.housingCap > 0)
                                ? (static_cast<float>(jobsCapAccessible) / static_cast<float>(scan.housingCap))
                                : 0.0f;
  s.demandResidential = ResidentialDemand(jobPressure, s.happiness, s.avgLandValue);

  s.demandCommercial = CommercialDemand(scan.population, jobsCapCommercialAccessible, goods.satisfaction,
                                        s.happiness, s.avgLandValue, m_cfg.taxCommercial);
  s.demandIndustrial = IndustrialDemand(jobPressure, scan.population, jobsCapIndustrialAccessible, goods.satisfaction,
                                        s.tradeMarketIndex, s.happiness, s.avgLandValue, m_cfg.taxIndustrial);

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

  // Reset per-day incident fields (derived).
  s.fireIncidentDamaged = 0;
  s.fireIncidentDestroyed = 0;
  s.fireIncidentDisplaced = 0;
  s.fireIncidentJobsLostCap = 0;
  s.fireIncidentCost = 0;
  s.fireIncidentOriginX = -1;
  s.fireIncidentOriginY = -1;
  s.fireIncidentDistrict = -1;
  s.fireIncidentHappinessPenalty = 0.0f;

  s.trafficIncidentInjuries = 0;
  s.trafficIncidentCost = 0;
  s.trafficIncidentOriginX = -1;
  s.trafficIncidentOriginY = -1;
  s.trafficIncidentDistrict = -1;
  s.trafficIncidentHappinessPenalty = 0.0f;

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

      float zoneDemand = 0.0f;
      if (t.overlay == Overlay::Residential) zoneDemand = s.demandResidential;
      else if (t.overlay == Overlay::Commercial) zoneDemand = s.demandCommercial;
      else if (t.overlay == Overlay::Industrial) zoneDemand = s.demandIndustrial;

      // Upgrade: happy + high land value + mostly full.
      if (t.level < 3 && s.happiness > 0.58f && lvVal > 0.45f && occFrac > 0.70f && s.money > 80 && zoneDemand > 0.45f) {
        const float demandBoost = 0.55f + 0.90f * zoneDemand;
        const float p = (0.0010f + 0.0040f * s.happiness * (0.6f + 0.4f * lvVal) * occFrac) * demandBoost;
        if (rng.chance(p)) {
          t.level++;
          // Some disruption during construction.
          t.occupants = static_cast<std::uint16_t>(static_cast<int>(t.occupants) * 0.85f);
          upgradeCost += 15 + 20 * t.level;
        }
      }

      // Downgrade: unhappy + low land value + mostly empty.
      if (t.level > 1 && (s.happiness < 0.42f || zoneDemand < 0.22f) && lvVal < 0.25f && occFrac < 0.35f) {
        const float demandPress = Clamp01(0.30f - zoneDemand);
        const float p = 0.0008f + 0.0030f * (0.42f - s.happiness) * (0.25f - lvVal) * (1.0f - occFrac) +
                        0.0022f * demandPress * (0.8f + 0.2f * (0.35f - occFrac));
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
    int weightQ16 = kQ16; // deterministic desirability key
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

      // Determinism note:
      //  - Sorting by raw floats can be sensitive to tiny cross-platform rounding differences.
      //  - We quantize land value to Q16 and compute a fixed-point desirability key.
      const float wCfg = (t.overlay == Overlay::Commercial) ? m_cfg.commercialDesirabilityWeight
                                                           : m_cfg.industrialDesirabilityWeight;

      const int lvQ16 = Float01ToQ16(std::clamp(lvVal, 0.0f, 1.0f));
      const int desirQ16 = (t.overlay == Overlay::Commercial) ? lvQ16 : (kQ16 - lvQ16);

      // Quantize config weight to Q16. Clamp to a sane range to avoid overflow and extreme behavior.
      const float wCfgClamped = std::clamp(wCfg, -4.0f, 4.0f);
      const int wQ16 = RoundToInt(wCfgClamped * static_cast<float>(kQ16));

      const int deltaQ16 = desirQ16 - (kQ16 / 2);
      const std::int64_t scaled = (static_cast<std::int64_t>(wQ16) * static_cast<std::int64_t>(deltaQ16)) /
                                  static_cast<std::int64_t>(kQ16);

      int weightQ16 = kQ16 + static_cast<int>(scaled);
      weightQ16 = std::clamp(weightQ16, kQ16 / 4, kQ16 * 2);

      sites.push_back(JobSite{x, y, cap, t.overlay, weightQ16});
    }
  }

  std::sort(sites.begin(), sites.end(), [&](const JobSite& a, const JobSite& b) {
    if (a.weightQ16 != b.weightQ16) return a.weightQ16 > b.weightQ16;
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

  // Traffic incident system (rare, deterministic per-day RNG stream).
  {
    RNG crashRng(world.seed() ^ (static_cast<std::uint64_t>(s.day) * 0xA0761D6478BD642FULL));
    const TrafficIncidentOutcome crash = TryApplyTrafficIncident(world, m_trafficIncidents, s,
                                                                 population, scan.zoneTiles,
                                                                 crashRng);
    if (crash.happened) {
      s.trafficIncidentInjuries = crash.injuries;
      s.trafficIncidentCost = crash.cost;
      s.trafficIncidentOriginX = crash.originX;
      s.trafficIncidentOriginY = crash.originY;
      s.trafficIncidentDistrict = crash.originDistrict;
      s.trafficIncidentHappinessPenalty = crash.happinessPenalty;
    }
  }

  // Fire incident system (rare, deterministic per-day RNG stream).
  const ZoneAccessMap* zoneAccessPtr = &zoneAccess;
  ZoneAccessMap zoneAccessAfter;
  {
    RNG fireRng(world.seed() ^ (static_cast<std::uint64_t>(s.day) * 0xD1B54A32D192ED03ULL));
    const FireIncidentOutcome fire = TryApplyFireIncident(world, m_fireIncidents,
                                                         m_cfg.requireOutsideConnection,
                                                         zoneAccess, edgeMask,
                                                         population, scan.zoneTiles,
                                                         fireRng);
    if (fire.happened) {
      s.fireIncidentDamaged = fire.damaged;
      s.fireIncidentDestroyed = fire.destroyed;
      s.fireIncidentDisplaced = fire.displaced;
      s.fireIncidentJobsLostCap = fire.jobsLostCap;
      s.fireIncidentCost = fire.cost;
      s.fireIncidentOriginX = fire.originX;
      s.fireIncidentOriginY = fire.originY;
      s.fireIncidentDistrict = fire.originDistrict;
      s.fireIncidentHappinessPenalty = fire.happinessPenalty;

      // If we cleared any zone tiles, recompute zone-access for accurate derived stats.
      if (fire.zoneLayoutChanged) {
        zoneAccessAfter = BuildZoneAccessMap(world, edgeMask);
        zoneAccessPtr = &zoneAccessAfter;
      }
    }
  }

  // Recompute derived stats (traffic, goods, happiness, budget metrics) for the new state.
  refreshDerivedStatsInternal(world, edgeMask, zoneAccessPtr);

  // Add incident response costs after refresh (refresh recomputes the base expense breakdown).
  if (s.fireIncidentCost > 0) {
    s.expenses += s.fireIncidentCost;
  }
  if (s.trafficIncidentCost > 0) {
    s.expenses += s.trafficIncidentCost;
  }

  // Include upgrade spending in the budget and apply the net change to money.
  s.upgradeCost = upgradeCost;
  s.expenses += upgradeCost;

  // Debt service (municipal bonds) behaves like an additional daily expense.
  const int debtService = ApplyDebtService(world);
  if (debtService > 0) {
    s.expenses += debtService;
  }

  s.money += (s.income - s.expenses);
}


} // namespace isocity