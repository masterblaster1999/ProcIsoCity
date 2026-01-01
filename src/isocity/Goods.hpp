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

  // If true, surplus industrial production is exported to the map edge (if reachable).
  bool allowExports = true;

  // Scale factors applied to base zone production/consumption.
  // Base industrial supply is 12 * level (mirrors industrial job capacity).
  // Base commercial demand is 8 * level (mirrors commercial job capacity).
  float supplyScale = 1.0f;
  float demandScale = 1.0f;
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
                            const ZoneAccessMap* precomputedZoneAccess = nullptr);

} // namespace isocity
