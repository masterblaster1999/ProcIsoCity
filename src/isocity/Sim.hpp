#pragma once

#include "isocity/World.hpp"

#include <array>
#include <vector>

namespace isocity {

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
  // Manhattan distance in tile space.
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
};

class Simulator {
public:
  explicit Simulator(SimConfig cfg = {}) : m_cfg(cfg) {}

  void update(World& world, float dt);

  // Same as update(), but optionally collects a Stats snapshot after each tick.
  // Returns the number of ticks processed.
  int update(World& world, float dt, std::vector<Stats>* outTickStats);

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

  // Recompute derived HUD stats (population/capacities/roads/parks/employment/happiness)
  // without advancing time or modifying tiles.
  void refreshDerivedStats(World& world) const;

private:
  void step(World& world);

  SimConfig m_cfg;
  TrafficModelSettings m_trafficModel{};
  float m_accum = 0.0f;
};

} // namespace isocity
