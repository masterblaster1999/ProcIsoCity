#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Deterministic automatic district assignment.
//
// Why?
//  - Districts are useful for overlays and per-district policy multipliers.
//  - Painting them manually is fine for small maps, but larger cities benefit
//    from a quick, repeatable "starter" partition that you can refine.
//
// The auto-districting algorithm:
//  1) Chooses up to N seed road tiles using deterministic farthest-point sampling
//     on the road network (travel-time weighted by road class if enabled).
//  2) Assigns each road tile to its nearest seed via a multi-source road flow field.
//  3) Optionally propagates those road districts to all tiles by nearest-road distance.

struct AutoDistrictConfig {
  // Requested number of districts. Clamped to [1, kDistrictCount].
  int districts = kDistrictCount;

  // If true, only consider roads that are connected to the map edge for seed
  // selection and road ownership.
  bool requireOutsideConnection = false;

  // If true, use travel-time weights for road distance/ownership.
  // (Highways "reach" farther than streets.)
  bool useTravelTime = true;

  // If true, propagate road-assigned districts out to all tiles.
  // If false, only road tiles are updated.
  bool fillAllTiles = true;

  // If false and fillAllTiles=true, water tiles are left unchanged.
  bool includeWater = true;
};

struct AutoDistrictResult {
  int districtsRequested = kDistrictCount;
  int districtsUsed = 0; // number of seeds actually used (<= requested)

  // Linear indices (y*w + x) of the chosen seed road tiles.
  std::vector<int> seedRoadIdx;
};

// Compute districts for a world and return the per-tile district IDs.
//
// outDistricts is resized to width*height.
AutoDistrictResult ComputeAutoDistricts(const World& world, std::vector<std::uint8_t>& outDistricts,
                                       const AutoDistrictConfig& cfg = {});

// Convenience helper: compute + write districts into the world's tiles.
AutoDistrictResult AutoAssignDistricts(World& world, const AutoDistrictConfig& cfg = {});

} // namespace isocity
