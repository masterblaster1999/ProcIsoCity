#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

struct ZoneAccessMap;

// A small "goods flow" model:
//  - Industrial zones produce goods (supply).
//  - Commercial zones consume goods (demand).
//  - Goods are routed along roads to nearby producers (travel-time weighted).
//    If the nearest producer can't fully satisfy demand, we deterministically
//    fall back to the next-nearest reachable producer(s) before importing.
//  - Optional imports/exports use the map edge as a trade partner.
//
// This is intentionally lightweight: it's a deterministic, tile-based heuristic
// meant to create interesting constraints for gameplay and debug overlays.
struct GoodsConfig {
  // If true, only zones adjacent to a road network that connects to the map edge
  // participate (mirrors SimConfig::requireOutsideConnection).
  bool requireOutsideConnection = true;

  // If true, unmet commercial demand is imported from the map edge (if reachable).
  bool allowImports = true;

  // Import capacity throttle (percentage in [0,100]).
  //
  // When < 100, even reachable edge imports may be partially unavailable.
  // This is used by the procedural trade system to model disruptions.
  int importCapacityPct = 100;

  // If true, surplus industrial production is exported to the map edge (if reachable).
  bool allowExports = true;

  // Export capacity throttle (percentage in [0,100]).
  int exportCapacityPct = 100;

  // Scale factors applied to base zone production/consumption.
  // Base industrial supply is 12 * level (mirrors industrial job capacity).
  // Base commercial demand is 8 * level (mirrors commercial job capacity).
  float supplyScale = 1.0f;
  float demandScale = 1.0f;

  // Optional per-tile multipliers applied in addition to the global scales above.
  //
  // If provided, these arrays must have size width*height and use the world grid
  // index: idx = y*width + x.
  //
  // This hook is used by the procedural economy model to create district-level
  // variation in production/consumption.
  const std::vector<float>* industrialSupplyMult = nullptr;
  const std::vector<float>* commercialDemandMult = nullptr;
};

struct GoodsResult {
  // Per-tile road traffic caused by goods shipments (local deliveries + imports + exports).
  // Non-road tiles are always 0.
  std::vector<std::uint16_t> roadGoodsTraffic;

  // For commercial tiles: delivered/demand mapped to 0..255.
  // For all other tiles: 255.
  std::vector<std::uint8_t> commercialFill;

  int goodsProduced = 0;  // local industrial supply
  int goodsDemand = 0;    // commercial demand
  int goodsDelivered = 0; // delivered to commercial (local + imports)
  int goodsImported = 0;
  int goodsExported = 0;
  int unreachableDemand = 0;

  // goodsDelivered / goodsDemand, clamped to [0,1]. If goodsDemand==0, this is 1.
  float satisfaction = 1.0f;

  int maxRoadGoodsTraffic = 0;
};

// Optional debug/analysis output for goods flow.
//
// This is intended for headless tooling (CLI, regression tests, GIS exports), not for
// the core simulation. It is only populated when explicitly requested.
//
// Indices are road tile indices in the world grid: idx = y*w + x.
enum class GoodsOdType : std::uint8_t {
  Local = 0,
  Import = 1,
  Export = 2,
};

inline const char* GoodsOdTypeName(GoodsOdType t)
{
  switch (t) {
    case GoodsOdType::Local:
      return "local";
    case GoodsOdType::Import:
      return "import";
    case GoodsOdType::Export:
      return "export";
  }
  return "local";
}

struct GoodsOdEdge {
  // Road tile index of the origin and destination. These are always valid road tiles.
  int srcRoadIdx = -1;
  int dstRoadIdx = -1;

  // Total units shipped along this OD pair.
  int amount = 0;

  // Route metrics.
  //
  // When aggregating, we keep totals (amount-weighted) so callers can compute
  // mean distance/time, as well as min/max.
  //
  // Units:
  //  - steps: road steps (edges)
  //  - costMilli: travel-time cost in milli-steps (street step == 1000)
  std::uint64_t totalSteps = 0;
  std::uint64_t totalCostMilli = 0;

  int minSteps = -1;
  int maxSteps = -1;
  int minCostMilli = -1;
  int maxCostMilli = -1;

  GoodsOdType type = GoodsOdType::Local;
};

struct GoodsFlowDebug {
  int w = 0;
  int h = 0;

  // Aggregated origin-destination flows between road access points.
  //
  // Note: this is aggregated by endpoints, not by exact route geometry.
  std::vector<GoodsOdEdge> od;
};

// Compute the current goods flow. This is pure/derived (does not mutate the world).
//
// If cfg.requireOutsideConnection is true, you can optionally supply a precomputed
// road-to-edge mask (as produced by ComputeRoadsConnectedToEdge) to avoid
// recomputing it.
//
// If you already computed a ZoneAccessMap for the same world + outside-connection rule,
// you may also pass it to avoid rebuilding the zone access mapping.
GoodsResult ComputeGoodsFlow(const World& world, const GoodsConfig& cfg,
                            const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr,
                            const ZoneAccessMap* precomputedZoneAccess = nullptr,
                            GoodsFlowDebug* outDebug = nullptr);

} // namespace isocity
