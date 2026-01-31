#pragma once

#include "isocity/World.hpp"
#include "isocity/TransitPlanner.hpp"
#include "isocity/TradeMarket.hpp"
#include "isocity/Economy.hpp"
#include "isocity/Services.hpp"
#include "isocity/TrafficSafety.hpp"
#include "isocity/AirPollution.hpp"

#include <array>
#include <vector>

namespace isocity {

struct ZoneAccessMap;

struct DistrictPolicy {
  // Multipliers applied to the base citywide policy numbers.
  //  1.0 = no change
  //  0.5 = half
  //  2.0 = double
  float taxResidentialMult = 1.0f;
  float taxCommercialMult = 1.0f;
  float taxIndustrialMult = 1.0f;

  float roadMaintenanceMult = 1.0f;
  float parkMaintenanceMult = 1.0f;
};

struct SimConfig {
  float tickSeconds = 0.5f; // how often the sim advances (in real seconds)

  // Parks boost happiness for *nearby* zone tiles (a simple "coverage" model).
  //
  // Local park influence radius.
  //
  // When >0, park coverage is computed via a road-network isochrone seeded from
  // parks' adjacent road tiles, and evaluated on zone parcels using ZoneAccessMap.
  // The unit is "street-step equivalents" (1 street road step ~= 1000 milli).
  //
  // Set to 0 to disable locality (parks behave like a global ratio again).
  int parkInfluenceRadius = 6;

  // If true, residential/commercial/industrial tiles only function when their
  // adjacent road network connects to the map edge (an \"outside\" connection).
  //
  // This is a classic city-builder rule: connect roads to the border to bring in
  // citizens/jobs and keep zones from stagnating.
  bool requireOutsideConnection = true;

  // --- Economy / policy ---
  // Simple per-day taxes (integer dollars) applied per occupant on each zone type.
  // The simulator also applies a land-value multiplier so high-value areas generate
  // more tax revenue.
  int taxResidential = 1;
  int taxCommercial = 2;
  int taxIndustrial = 2;

  // Per-day maintenance costs per tile.
  int maintenanceRoad = 1;
  int maintenancePark = 1;

  // How strongly taxes reduce happiness (approx. per-tax-dollar per-capita).
  // This is intentionally small; the game is still a sandbox.
  float taxHappinessPerCapita = 0.02f;

  // --- Growth tuning ---
  // Residential target occupancy is multiplied by a per-tile desirability factor derived
  // from land value. Higher values concentrate growth in attractive neighborhoods.
  float residentialDesirabilityWeight = 0.70f;
  float commercialDesirabilityWeight = 0.80f;
  float industrialDesirabilityWeight = 0.80f;

  // District policies: optional per-district multipliers applied during budget/tax calculations.
  bool districtPoliciesEnabled = false;
  std::array<DistrictPolicy, kDistrictCount> districtPolicies{};
};

// Non-persistent runtime tuning for derived systems.
//
// This is intentionally NOT part of SimConfig (and therefore not saved) so
// experimental model tweaks can evolve rapidly without forcing save-version
// bumps.
struct TrafficModelSettings {
  // Enable multi-pass congestion-aware routing for commute estimates.
  bool congestionAwareRouting = false;

  // Number of incremental assignment passes.
  // 1 => classic all-or-nothing assignment (fast).
  int congestionIterations = 4;

  // BPR-style travel time curve parameters:
  //   t = t0 * (1 + alpha * (v/c)^beta)
  float congestionAlpha = 0.15f;
  float congestionBeta = 4.0f;

  float congestionCapacityScale = 1.0f;
  float congestionRatioClamp = 3.0f;

  // Enable capacity-aware job assignment (soft constraints via per-source penalties).
  // When enabled, commute destinations become less attractive once their nearby job capacity
  // is 'full', which tends to produce more realistic commute lengths in cities with a few
  // strong job centers.
  bool capacityAwareJobs = false;

  // Iterations used to fit the per-source penalties (>=1).
  int jobAssignmentIterations = 6;

  // Baseline penalty scale (milli-travel-time units). Roughly, 1000 ~= one street tile.
  int jobPenaltyBaseMilli = 8000;
};

// Demand signal used by the transit planner and the transit mode-shift model.
enum class TransitDemandMode : std::uint8_t { Commute = 0, Goods = 1, Combined = 2 };

inline const char* TransitDemandModeName(TransitDemandMode m)
{
  switch (m) {
    case TransitDemandMode::Commute: return "commute";
    case TransitDemandMode::Goods: return "goods";
    case TransitDemandMode::Combined: return "combined";
  }
  return "combined";
}

// Non-persistent runtime tuning for the transit system.
//
// This is intentionally not part of SimConfig (and therefore not saved) so the
// transit model can iterate quickly without save-version churn.
struct TransitModelSettings {
  // Master enable for simulation impacts (mode shift, cost, stats). The planner
  // overlay can still be used even when this is disabled.
  bool enabled = false;

  // How heavily the city funds/operates the system. Used as a multiplier for
  // ridership potential and operating cost.
  float serviceLevel = 1.0f;

  // Maximum share of commuters that can plausibly shift to transit.
  float maxModeShare = 0.35f;

  // Transit travel time relative to car travel time, using the same underlying
  // road network path lengths as a proxy.
  //
  // < 1.0 => faster (more attractive), > 1.0 => slower.
  float travelTimeMultiplier = 0.75f;

  // Sampling spacing for stop generation (used for stop count and cost).
  int stopSpacingTiles = 12;

  // Per-tick operating costs.
  int costPerTile = 1;
  int costPerStop = 2;

  // Planner input mode (what flow signal lines are optimized for).
  TransitDemandMode demandMode = TransitDemandMode::Combined;

  // Planner parameters (line count, weight mode, demand bias, etc.).
  TransitPlannerConfig plannerCfg{};
};

// Non-persistent runtime tuning for traffic safety gameplay.
//
// This is intentionally not part of SimConfig (and therefore not saved) so the
// model can evolve without forcing save-version churn.
struct TrafficSafetyModelSettings {
  // Master enable for simulation impacts (happiness penalties, hotspot stats).
  // The heatmap overlay remains available regardless.
  bool enabled = true;

  // TrafficSafety model configuration used for derived stats.
  //
  // Notes:
  // - requireOutsideConnection is overridden by SimConfig::requireOutsideConnection.
  // - canyonWeight is forced to 0 in the simulator to avoid expensive SkyView computation.
  TrafficSafetyConfig cfg{};

  // Continuous citywide happiness penalty derived from residentMeanExposure.
  // penalty = clamp(residentMeanExposure * happinessPenaltyScale, 0, maxHappinessPenalty)
  float happinessPenaltyScale = 0.07f;
  float maxHappinessPenalty = 0.10f;
};

// Non-persistent runtime tuning for air quality gameplay.
//
// This is intentionally not part of SimConfig (and therefore not saved) so the
// model can evolve without save-version churn.
struct AirPollutionModelSettings {
  // Master enable for simulation impacts (happiness penalties).
  // The app's heatmap overlay remains available regardless.
  bool enabled = true;

  // Air pollution transport + emission configuration.
  AirPollutionConfig cfg{};

  // Happiness penalty derived from resident-weighted exposure summary:
  //   penalty = clamp(residentAvgPollution01 * happinessPenaltyScale +
  //                  residentHighExposureFrac * highExposurePenaltyScale,
  //                  0, maxHappinessPenalty)
  float happinessPenaltyScale = 0.06f;
  float highExposurePenaltyScale = 0.04f;
  float maxHappinessPenalty = 0.12f;
};

// Non-persistent runtime tuning for emergent traffic incident gameplay.
//
// This system uses the TrafficSafety hotspot (a deterministic high-risk road
// tile) as an "incident candidate" signal from the previous day.
struct TrafficIncidentSettings {
  // Master enable.
  bool enabled = true;

  // Do not trigger incidents in very small towns.
  int minPopulation = 60;
  int minZoneTiles = 12;

  // Base chance per simulation day (after minPopulation/minZoneTiles).
  float baseChancePerDay = 0.0060f;

  // Additional chance per 100 residents.
  float chancePer100Population = 0.0010f;

  // Multiplicative chance boosts based on the previous-day safety stats.
  float exposureChanceBoost = 0.75f;   // scales with residentMeanExposure (0..1)
  float hotspotRiskChanceBoost = 0.50f; // scales with hotspotRisk01 (0..1)
  float maxChancePerDay = 0.18f;

  // Severity / budget effects.
  int minInjuries = 1;
  int maxInjuries = 12;
  float injuriesRiskBonus = 8.0f; // additional injuries when risk01 ~ 1.0

  float happinessPenaltyBase = 0.012f;
  float happinessPenaltyPerInjury = 0.0018f;
  float maxHappinessPenalty = 0.18f;

  int costBase = 6;
  int costPerInjury = 2;

  // If the city has no safety facilities (police/fire), incidents are harsher.
  float noSafetyServicesMultiplier = 1.25f;

  // If safety services exist, high safety satisfaction mitigates severity.
  float safetySatisfactionMitigation = 0.35f;
  float minSafetyMitigation = 0.65f;
};

// Non-persistent runtime tuning for emergent "incident" gameplay.
//
// This is not part of SimConfig (and therefore not saved) so the system can
// evolve without save-version churn.
struct FireIncidentSettings {
  // Master enable.
  bool enabled = true;

  // Do not trigger fires in very small towns.
  int minPopulation = 40;
  int minZoneTiles = 12;

  // Base chance per simulation day (after minPopulation/minZoneTiles).
  // Typical mid-sized cities will see a few incidents per in-game year.
  float baseChancePerDay = 0.0070f;

  // Additional chance per 100 residents.
  float chancePer100Population = 0.0015f;

  // If the city has no Fire Stations at all, multiply the chance.
  float noStationMultiplier = 1.65f;

  // Per-station reduction applied to the chance, clamped by minChanceFactor.
  float stationChanceMitigation = 0.18f;
  float minChanceFactor = 0.45f;

  // How many tiles are affected when a fire happens.
  int minAffectedTiles = 4;
  int maxAffectedTiles = 28;

  // Fire spread tuning.
  float spreadBase = 0.68f; // base probability of propagating to a neighbor

  // Damage tuning (per affected tile).
  float destroyBase = 0.22f; // chance to clear the building

  // Citywide happiness penalty applied when a fire happens.
  float happinessPenaltyBase = 0.03f;
  float happinessPenaltyPerTile = 0.0020f;
  float happinessPenaltyPer100Displaced = 0.0060f;
  float maxHappinessPenalty = 0.20f;

  // Response cost (added to expenses on the day of the incident).
  int costPerDamagedTile = 6;
  int costPerDestroyedTile = 12;
  int costPer10Displaced = 1;
  int costPer10JobsCapLost = 1;
};

class Simulator {
public:
  explicit Simulator(SimConfig cfg = {}) : m_cfg(cfg) {}

  void update(World& world, float dt);

  // Same as update(), but optionally collects a Stats snapshot after each tick.
  // Returns the number of ticks processed.
  int update(World& world, float dt, std::vector<Stats>* outTickStats);

  // Like update(), but clamps how many simulation ticks can be processed in a single call.
  //
  // This is useful for real-time game loops: if a frame stalls (e.g., breakpoint, alt-tab, hitch),
  // an unbounded catch-up can cause a "spiral of death" where one long frame triggers many
  // expensive ticks, causing the next frame to be even longer.
  //
  // - maxTicks <= 0 disables the per-call tick limit (behaves like update()).
  // - maxBacklogTicks > 0 clamps the internal accumulator so extremely large dt spikes do not
  //   queue an unbounded amount of work.
  // - outTickStats optionally collects a Stats snapshot after each processed tick.
  //
  // Returns the number of ticks processed.
  int updateLimited(World& world, float dt, int maxTicks, int maxBacklogTicks,
                    std::vector<Stats>* outTickStats = nullptr);

  // Inspect the internal tick accumulator (useful for debugging/perf overlays).
  float accumulatedSeconds() const { return m_accum; }
  int accumulatedTicks() const
  {
    const float ts = m_cfg.tickSeconds;
    if (!(ts > 1.0e-6f)) return 0;
    if (!(m_accum > 0.0f)) return 0;
    return static_cast<int>(m_accum / ts);
  }

  // Advance the simulation by exactly one tick (increments day, updates economy, etc.).
  // Resets the internal timer accumulator so stepping is deterministic.
  void stepOnce(World& world);

  // Clears the internal tick accumulator (useful when pausing/unpausing or changing sim speed).
  void resetTimer() { m_accum = 0.0f; }

  const SimConfig& config() const { return m_cfg; }

  // Mutable access so the game layer can implement a small policy/budget UI.
  SimConfig& config() { return m_cfg; }

  const TrafficModelSettings& trafficModel() const { return m_trafficModel; }
  TrafficModelSettings& trafficModel() { return m_trafficModel; }

  const TrafficSafetyModelSettings& trafficSafetyModel() const { return m_trafficSafetyModel; }
  TrafficSafetyModelSettings& trafficSafetyModel() { return m_trafficSafetyModel; }

  const AirPollutionModelSettings& airPollutionModel() const { return m_airPollutionModel; }
  AirPollutionModelSettings& airPollutionModel() { return m_airPollutionModel; }

  const TrafficIncidentSettings& trafficIncidents() const { return m_trafficIncidents; }
  TrafficIncidentSettings& trafficIncidents() { return m_trafficIncidents; }

  const TransitModelSettings& transitModel() const { return m_transitModel; }
  TransitModelSettings& transitModel() { return m_transitModel; }

  const ServicesModelSettings& servicesModel() const { return m_servicesModel; }
  ServicesModelSettings& servicesModel() { return m_servicesModel; }

  const TradeModelSettings& tradeModel() const { return m_tradeModel; }
  TradeModelSettings& tradeModel() { return m_tradeModel; }

  const EconomyModelSettings& economyModel() const { return m_economyModel; }
  EconomyModelSettings& economyModel() { return m_economyModel; }

  const FireIncidentSettings& fireIncidents() const { return m_fireIncidents; }
  FireIncidentSettings& fireIncidents() { return m_fireIncidents; }

  // Recompute derived HUD stats (population/capacities/roads/parks/employment/happiness)
  // without advancing time or modifying tiles.
  void refreshDerivedStats(World& world) const;

private:
  // Internal: allow callers (Simulator::step) to reuse expensive derived data
  // like the road-to-edge mask and ZoneAccessMap across multiple subsystems.
  void refreshDerivedStatsInternal(World& world, const std::vector<std::uint8_t>* precomputedRoadToEdge,
                                  const ZoneAccessMap* precomputedZoneAccess) const;

  void step(World& world);

  SimConfig m_cfg;
  TrafficModelSettings m_trafficModel{};
  TrafficSafetyModelSettings m_trafficSafetyModel{};
  AirPollutionModelSettings m_airPollutionModel{};
  TrafficIncidentSettings m_trafficIncidents{};
  TransitModelSettings m_transitModel{};
  ServicesModelSettings m_servicesModel{};
  TradeModelSettings m_tradeModel{};
  EconomyModelSettings m_economyModel{};
  FireIncidentSettings m_fireIncidents{};
  float m_accum = 0.0f;
};

} // namespace isocity
