#pragma once

#include "isocity/Sim.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace isocity {

// Per-district aggregation used by UI and tooling.
//
// Notes:
// - This is intentionally a *derived* view: it does not affect simulation.
// - The budget numbers mirror Simulator::refreshDerivedStats() for taxes and
//   maintenance where possible.
struct DistrictSummary {
  int id = 0;

  int tiles = 0;
  int landTiles = 0;
  int waterTiles = 0;

  int roads = 0;
  int parks = 0;

  int resTiles = 0;
  int comTiles = 0;
  int indTiles = 0;

  int zoneTiles = 0;
  int zoneTilesAccessible = 0;

  int population = 0;
  int housingCapacity = 0;

  int jobsCapacity = 0;
  int jobsCapacityAccessible = 0;
  int employed = 0;

  float avgLandValue = 0.0f;

  int taxRevenue = 0;

  int roadMaintenanceCost = 0;
  int parkMaintenanceCost = 0;
  int maintenanceCost = 0;

  int net = 0;
};

struct DistrictStatsResult {
  std::array<DistrictSummary, static_cast<std::size_t>(kDistrictCount)> districts{};
  DistrictSummary total{};
};

// Computes summary stats for each district.
//
// landValueField:
//   Optional per-tile land value field (size must be width*height). If absent,
//   avgLandValue and taxRevenue will be zeroed.
//
// roadToEdgeMask:
//   Optional cached mask computed via ComputeRoadsConnectedToEdge (size must be
//   width*height). If absent and SimConfig::requireOutsideConnection is true,
//   it will be computed internally.
DistrictStatsResult ComputeDistrictStats(const World& world, const SimConfig& simCfg,
                                        const std::vector<float>* landValueField = nullptr,
                                        const std::vector<std::uint8_t>* roadToEdgeMask = nullptr);

} // namespace isocity
