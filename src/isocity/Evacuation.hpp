#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Evacuation analysis: compute which residential tiles can reach a safe "exit" road tile
// on the map edge under an optional per-tile hazard/closure mask.
//
// The intended use is scenario analysis:
//  - coastal flooding / ponding (see proc_isocity_floodrisk)
//  - construction closures
//  - resilience / redundancy studies
//
// This module is headless and deterministic.

struct EvacuationConfig {
  // If true, routing on the road network is weighted by RoadTravelTimeMilliForLevel(level).
  // If false, routing uses unweighted step distance.
  bool useTravelTime = true;

  // Walking cost used to move within a connected Residential zone component to a boundary tile,
  // and for the final step from the boundary tile onto the chosen adjacent road.
  //
  // Units: milli-steps (street step == 1000).
  int walkCostMilli = 1000;

  // Base street capacity used for a simple congestion/utilization estimate.
  // Per-road capacity is derived with RoadCapacityForLevel(base, Tile::level).
  int roadTileCapacity = 28;
};

struct EvacuationResult {
  int w = 0;
  int h = 0;

  // Number of unique exit road tiles considered as flow-field sources.
  int exitSources = 0;

  // Road flow field distances/costs/parents to the nearest exit.
  // Size = w*h, with -1 for non-road tiles or unreachable roads.
  std::vector<int> roadDistSteps;
  std::vector<int> roadCostMilli;
  std::vector<int> roadParent;

  // Per-residential-tile evacuation cost (includes walking to road) in milli-steps.
  // Size = w*h; -1 for non-residential tiles or unreachable.
  std::vector<int> resCostMilli;

  // The chosen access road tile index (linear y*w+x) for each residential tile.
  // Size = w*h; -1 for non-residential tiles or unreachable.
  std::vector<int> resAccessRoad;

  // Evacuation demand aggregated onto the road network by following roadParent from each
  // residential tile's access road. Size = w*h; non-road tiles are 0.
  std::vector<std::uint32_t> evacRoadFlow;

  std::uint32_t maxEvacRoadFlow = 0;

  // Simple congestion estimate:
  //   congestion = (sum(max(0, flow-capacity)) / sum(flow))
  // over road tiles with flow>0. Range [0..1].
  int congestedRoadTiles = 0;
  float congestion = 0.0f;
  // Compatibility alias for UI (percent).
  float congestionFrac = 0.0f;

  // Residential accessibility summary (tile counts).
  int residentialTiles = 0;
  int floodedResidentialTiles = 0;
  int reachableResidentialTiles = 0;
  int unreachableResidentialTiles = 0;

  // Residential accessibility summary (population = sum(Tile::occupants)).
  int population = 0;
  int floodedPopulation = 0;
  int reachablePopulation = 0;
  int unreachablePopulation = 0;

  // Evacuation time stats for reachable residents.
  // Units: "street steps" (milli / 1000).
  float avgEvacTime = 0.0f;
  float p95EvacTime = 0.0f;
};

// Compute evacuation accessibility to the map edge.
//
// Exits:
//  - Any road tile on the map boundary (x==0||y==0||x==w-1||y==h-1).
//
// blockedMask:
//  - Optional w*h mask where !=0 marks a hazardous/blocked tile.
//  - Blocked road tiles are not traversable.
//  - Blocked residential tiles are counted as "flooded" and excluded from routing.
EvacuationResult ComputeEvacuationToEdge(const World& world, const EvacuationConfig& cfg,
                                        const std::vector<std::uint8_t>* blockedMask = nullptr);

} // namespace isocity
