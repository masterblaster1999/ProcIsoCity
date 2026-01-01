#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Capacity model used when aggregating tile-level traffic onto a RoadGraph.
struct RoadGraphTrafficConfig {
  // Base capacity for a street tile. If useRoadLevelCapacity is true, this
  // is scaled by RoadCapacityForLevel(base, tile.level).
  int baseTileCapacity = 28;

  // If false, every road tile uses baseTileCapacity as its capacity (ignores
  // road class / Tile::level).
  bool useRoadLevelCapacity = true;
};

struct RoadGraphTrafficNodeStats {
  Point pos{};
  int degree = 0;

  int traffic = 0;   // traffic on this node's road tile (0 if non-road)
  int capacity = 0;  // capacity on this node's road tile (0 if non-road)
  double util = 0.0; // traffic / capacity (0 if capacity == 0)

  // Derived from incident edges (interior-only, so this doesn't double-count
  // node tiles). Useful for ranking intersections by nearby congestion.
  std::uint64_t incidentSumTraffic = 0;
  double incidentMaxUtil = 0.0;
};

struct RoadGraphTrafficEdgeStats {
  int a = -1;
  int b = -1;

  int length = 0; // same semantics as RoadGraphEdge::length
  int tileCount = 0;
  int interiorTileCount = 0; // excludes endpoints (node tiles)

  // ---- All tiles (including endpoints) ----
  std::uint64_t sumTrafficAll = 0;
  int maxTrafficAll = 0;

  std::uint64_t sumCapacityAll = 0;
  int minCapacityAll = 0;
  int maxCapacityAll = 0;

  double sumUtilAll = 0.0;
  double maxUtilAll = 0.0;

  int congestedTilesAll = 0;           // tiles where traffic > capacity
  std::uint64_t excessTrafficAll = 0;  // sum max(0, traffic - capacity)

  // ---- Interior tiles only (excluding endpoints) ----
  std::uint64_t sumTrafficInterior = 0;
  int maxTrafficInterior = 0;

  std::uint64_t sumCapacityInterior = 0;
  int minCapacityInterior = 0;
  int maxCapacityInterior = 0;

  double sumUtilInterior = 0.0;
  double maxUtilInterior = 0.0;

  int congestedTilesInterior = 0;
  std::uint64_t excessTrafficInterior = 0;
};

struct RoadGraphTrafficResult {
  int w = 0;
  int h = 0;
  RoadGraphTrafficConfig cfg{};

  // Parallel to RoadGraph::nodes / RoadGraph::edges.
  std::vector<RoadGraphTrafficNodeStats> nodes;
  std::vector<RoadGraphTrafficEdgeStats> edges;
};

// Aggregate a per-road-tile traffic heatmap (TrafficResult::roadTraffic) onto a compressed RoadGraph.
//
// Intended for:
//  - quick bottleneck detection (rank edges by max utilization)
//  - exporting a smaller traffic representation for visualization / analysis
//  - higher-level future work (signal placement, road upgrades, path rerouting)
//
// Notes:
//  - Edge "interior" stats exclude the endpoint node tiles to reduce double-counting across edges.
//  - Node traffic/capacity/util are computed directly from the node tile.
RoadGraphTrafficResult AggregateTrafficOnRoadGraph(const World& world, const RoadGraph& g, const TrafficResult& traffic,
                                                  const RoadGraphTrafficConfig& cfg = {});

// Aggregate a generic per-tile road flow map onto a RoadGraph.
//
// This is useful for:
//  - combining multiple flow sources (eg. commuters + goods) into a single analysis
//  - tools that operate on synthetic / externally produced flow maps
//
// `roadFlow` must have size world.width()*world.height() and use the same indexing as
// TrafficResult::roadTraffic (idx = y*w + x). Values are interpreted as "vehicles".
RoadGraphTrafficResult AggregateFlowOnRoadGraph(const World& world, const RoadGraph& g,
                                                const std::vector<std::uint32_t>& roadFlow,
                                                const RoadGraphTrafficConfig& cfg = {});

} // namespace isocity
