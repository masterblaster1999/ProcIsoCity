#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

struct ZoneAccessMap;

// Accessibility / isochrone helpers.
//
// An "isochrone" is a region reachable within a given travel-time threshold.
// In ProcIsoCity, we approximate travel time by running deterministic multi-source
// searches over the road grid (optionally weighted by road class), then mapping
// non-road tiles (zones, parks, etc.) to their nearest/access road tiles.

enum class IsochroneWeightMode : std::uint8_t {
  Steps = 0,      // shortest-path by road steps (edges)
  TravelTime = 1, // shortest-path by travel-time weights (Street/Avenue/Highway)
};

// Per-road-tile accessibility result.
struct RoadIsochroneField {
  int w = 0;
  int h = 0;

  // Flat arrays of size w*h.
  // Values are in "milli-steps" (Street step = 1000). -1 means unreachable or non-road.
  std::vector<int> costMilli;

  // Number of road edges along the chosen route. -1 means unreachable or non-road.
  std::vector<int> steps;

  // Optional per-road-tile source ownership label (0..sources-1). -1 means unreachable/non-road.
  // Empty unless computeOwner=true.
  std::vector<int> owner;

  bool empty() const { return w <= 0 || h <= 0; }
  std::size_t size() const { return costMilli.size(); }
};

struct RoadIsochroneConfig {
  bool requireOutsideConnection = false;
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;
  bool computeOwner = false;
};

// Build a road accessibility field from one or more source road tiles.
//
// - sourceRoadIdx are linear indices (y*w + x) of road tiles.
// - If cfg.requireOutsideConnection is true, traversal is restricted to roads
//   connected to the map edge.
// - If cfg.weightMode==TravelTime, routing is travel-time weighted (deterministic
//   multi-source Dijkstra) so faster roads reach further.
//
// extraCostMilli (optional): additional per-tile penalty applied when entering a
// road tile (useful for congestion-aware variants).
RoadIsochroneField BuildRoadIsochroneField(const World& world,
                                           const std::vector<int>& sourceRoadIdx,
                                           const RoadIsochroneConfig& cfg = {},
                                           const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr,
                                           const std::vector<int>* extraCostMilli = nullptr);

// Configuration for mapping road accessibility onto arbitrary tiles.
struct TileAccessCostConfig {
  // Which tiles should receive a cost value.
  bool includeRoadTiles = true;
  bool includeZones = true;
  bool includeNonZonesAdjacentToRoad = true;

  // If false, water tiles always remain unreachable (-1).
  bool includeWater = false;

  // Added when mapping a non-road tile to a road tile.
  // (e.g., "walk from road to parcel").
  int accessStepCostMilli = 0;

  // If true, zone tiles use ZoneAccessMap for interior parcel access.
  // If false, zones fall back to the adjacent-road rule.
  bool useZoneAccessMap = true;
};

// Build a per-tile accessibility cost field derived from a RoadIsochroneField.
//
// Mapping rules:
//  - Road tiles copy their road cost.
//  - Zone tiles (R/C/I) use ZoneAccessMap to map to an access road (supports
//    multi-tile zoning blocks).
//  - Other non-water tiles optionally use the minimum cost among adjacent road tiles.
//
// Returned vector size is w*h; values are milli-steps (Street step = 1000), -1 for unreachable.
std::vector<int> BuildTileAccessCostField(const World& world,
                                         const RoadIsochroneField& roadField,
                                         const TileAccessCostConfig& cfg,
                                         const std::vector<std::uint8_t>* roadToEdgeMask = nullptr,
                                         const ZoneAccessMap* precomputedZoneAccess = nullptr);

} // namespace isocity
