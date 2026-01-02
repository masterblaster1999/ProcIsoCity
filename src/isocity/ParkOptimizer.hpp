#pragma once

#include "isocity/Isochrone.hpp"
#include "isocity/Types.hpp"
#include "isocity/World.hpp"
#include "isocity/ZoneAccess.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Greedy, tooling-oriented park placement suggestions.
//
// The core game currently models parks as a lightweight happiness booster.
// As a city grows, it becomes hard to eyeball where new parks will help most.
// This module provides a deterministic, dependency-free way to suggest park
// placements based on:
//   - where zone demand lives (tiles or occupants)
//   - how far that demand is from the *nearest existing park* along the road
//     network (weighted by steps or travel-time)
//
// Design goals:
//   - fast enough for headless CLI tooling (iterative exploration)
//   - deterministic output (stable tie-breaking)
//   - does NOT mutate World during planning (Apply is separate)

enum class ParkDemandMode : std::uint8_t {
  Tiles = 0,     // each eligible zone tile contributes weight=1
  Occupants = 1, // each eligible zone tile contributes weight=Tile::occupants
};

struct ParkOptimizerConfig {
  // If true, only consider road components connected to the map edge
  // (classic "outside connection" rule).
  bool requireOutsideConnection = true;

  // How road distance is measured when scoring underserved demand.
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;

  // How zone demand is weighted.
  ParkDemandMode demandMode = ParkDemandMode::Occupants;

  // Which zone overlays contribute demand.
  bool includeResidential = true;
  bool includeCommercial = true;
  bool includeIndustrial = true;

  // How many additional parks to propose.
  int parksToAdd = 10;

  // Optional: focus the planner on improving locations beyond a "service level".
  //
  // When >0, the score uses max(0, costMilli - targetCostMilli) instead of costMilli,
  // which prioritizes demand that is farther than the target.
  //
  // Units: milli-steps (Street step ~= 1000).
  int targetCostMilli = 0;
};

struct ParkPlacement {
  // Tile coordinates where the park should be placed (overlay = Park).
  Point parkTile{0, 0};

  // Adjacent road tile that acts as the "access" source for distance scoring.
  Point accessRoad{0, 0};

  // Total demand weight aggregated onto this accessRoad tile.
  std::uint64_t demandWeight = 0;

  // Distance from this accessRoad tile to the nearest existing park access road.
  // -1 if unreachable or if there were no initial parks.
  int costMilliBefore = -1;

  // Planner score used for ranking.
  // Higher is better.
  double score = 0.0;
};

struct ParkOptimizerResult {
  int w = 0;
  int h = 0;
  ParkOptimizerConfig cfg{};

  int existingParks = 0;
  std::uint64_t totalDemandWeight = 0;

  std::vector<ParkPlacement> placements;
};

// Suggest park placements without mutating the world.
//
// - precomputedZoneAccess / precomputedRoadToEdge are optional caches.
// - Returned placements are ordered by selection order (greedy iterations).
ParkOptimizerResult SuggestParkPlacements(const World& world,
                                         const ParkOptimizerConfig& cfg,
                                         const ZoneAccessMap* precomputedZoneAccess = nullptr,
                                         const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr);

// Apply suggested placements to a world by setting Overlay::Park on the target tiles.
//
// NOTE: This does not charge money and does not recompute derived stats.
// Tooling/CLI callers typically follow this with Simulator::refreshDerivedStats().
void ApplyParkPlacements(World& world, const std::vector<ParkPlacement>& placements);

} // namespace isocity
