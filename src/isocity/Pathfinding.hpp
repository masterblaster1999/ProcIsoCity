#pragma once

#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Compute which road tiles are connected to the map edge ("outside connection").
//
// The returned mask is a flat array of size world.width() * world.height() where:
//   - mask[y*w + x] == 1  => the tile is a road and is connected to the edge via roads
//   - mask[y*w + x] == 0  => otherwise
//
// Non-road tiles are always 0.
void ComputeRoadsConnectedToEdge(const World& world, std::vector<std::uint8_t>& outRoadToEdge);

// Convenience: check if (x,y) has an adjacent road tile that is marked as connected-to-edge
// in a mask produced by ComputeRoadsConnectedToEdge().
bool HasAdjacentRoadConnectedToEdge(const World& world, const std::vector<std::uint8_t>& roadToEdge, int x, int y);

// Deterministically pick an adjacent road tile for a given tile.
//
// This is used by multiple systems (traffic, goods flow, services) to map a
// zone tile to its nearest road access point. The selection order is N,E,S,W
// for stability/determinism.
//
// If roadToEdgeMask is provided (and matches world dimensions), only road tiles
// that are connected to the map edge are considered.
bool PickAdjacentRoadTile(const World& world, const std::vector<std::uint8_t>* roadToEdgeMask, int x, int y,
                          Point& outRoad);


// Find a shortest path along road tiles (4-neighborhood).
//
// Returns true and fills outPath (inclusive of start & goal) when a path exists.
// outCost (if non-null) is the number of steps (edges), i.e. outPath.size()-1.
bool FindRoadPathAStar(const World& world, Point start, Point goal, std::vector<Point>& outPath, int* outCost = nullptr);

// Extended road A* with travel-time weights and optional per-tile penalties.
//
// This is useful for:
//  - vehicle routing that prefers faster road classes (Street/Avenue/Highway)
//  - "avoidance" routing (safety, noise, etc.) by supplying extra per-tile costs
//  - turn-penalized routes that prefer fewer turns for clearer navigation
//
// Costs are expressed in "milli-steps" where 1000 ~= one Street step.
enum class RoadPathMetric : std::uint8_t {
  Steps = 0,      // Optimize number of road-tile steps (classic behavior)
  TravelTime = 1, // Optimize weighted travel time (road class speed + penalties)
};

struct RoadPathAStarConfig {
  // Which metric to optimize.
  RoadPathMetric metric = RoadPathMetric::Steps;

  // Optional additional per-tile cost in milli-steps, added when ENTERING a road tile.
  // If provided, the vector must have size world.width()*world.height().
  // Non-road tiles are ignored (but the vector must still cover the full grid).
  const std::vector<int>* extraTileCostMilli = nullptr;

  // Turn penalty in milli-steps, added when the direction changes between consecutive moves.
  // Only applied when metric == TravelTime (it is still reported in the cost breakdown).
  int turnPenaltyMilli = 0;

  // When metric==Steps, FindRoadPathAStarEx() will still compute travel-time costs for reporting.
  bool computeTravelTimeCost = true;
};

struct RoadPathCostBreakdown {
  // Base road travel-time cost (includes bridge penalties and road class speeds).
  int travelTimeMilli = 0;

  // Sum of extraTileCostMilli values along the path (excluding the start tile).
  int extraCostMilli = 0;

  // Total turn penalties along the path.
  int turnPenaltyMilli = 0;

  int totalCostMilli() const { return travelTimeMilli + extraCostMilli + turnPenaltyMilli; }
};

// Compute the travel-time cost of a road-tile path, including optional penalties.
//
// Path is expected to be a valid road-tile polyline (inclusive of start and goal).
RoadPathCostBreakdown ComputeRoadPathCostMilli(const World& world, const std::vector<Point>& path,
                                               const std::vector<int>* extraTileCostMilli = nullptr,
                                               int turnPenaltyMilli = 0);

// Find a road route using either classic step-count A* or travel-time weighted A*.
//
// Always outputs:
//  - outPath: inclusive start..goal polyline
//  - outSteps: step count (outPath.size()-1)
//
// When outCostMilli or outBreakdown are provided, the function also computes:
//  - weighted travel time (milli-steps), optionally including extraTileCostMilli and turn penalties.
//
// Notes:
//  - If cfg.metric==Steps, the chosen path matches FindRoadPathAStar() (hazards/turn penalties do NOT affect
//    the chosen path), but costs are still reported when requested.
//  - If cfg.metric==TravelTime, the path is optimized by weighted time + penalties.
bool FindRoadPathAStarEx(const World& world, Point start, Point goal, std::vector<Point>& outPath,
                         int* outSteps = nullptr, int* outCostMilli = nullptr,
                         RoadPathCostBreakdown* outBreakdown = nullptr,
                         const RoadPathAStarConfig& cfg = {});


// Find the shortest path from a road tile to *any* road tile on the map edge.
// Useful for debugging "outside connection" road networks.
bool FindRoadPathToEdge(const World& world, Point start, std::vector<Point>& outPath, int* outCost = nullptr);

// Find a shortest path over *buildable* (non-water) tiles (4-neighborhood).
//
// This is useful during procedural generation to route roads around lakes,
// but can also be reused for other systems (eg. services/agents) later.
//
// Returns true and fills outPath (inclusive of start & goal) when a path exists.
// outCost (if non-null) is the number of steps (edges), i.e. outPath.size()-1.
bool FindLandPathAStar(const World& world, Point start, Point goal, std::vector<Point>& outPath, int* outCost = nullptr);

// Configuration for road-building path planning.
//
// This is used by both:
//  - ProcGen (connect hubs, prefer reusing existing roads)
//  - the in-game Shift+drag road planner (optimize money cost, allow bridges)
struct RoadBuildPathConfig {
  enum class CostModel : std::uint8_t {
    // Classic behavior: cost = number of *new* road tiles that would be created.
    // Existing roads have cost 0.
    NewTiles = 0,

    // Money-aware planner: cost = actual economy cost to build/upgrade each tile
    // to `targetLevel` (including bridge multipliers and upgrade deltas).
    Money = 1,
  };

  // Desired road level (1..3). Only used when costModel==Money.
  int targetLevel = 1;

  // If true, allow the planner to traverse Water tiles (roads on water are bridges).
  bool allowBridges = false;

  // Which cost model to optimize.
  CostModel costModel = CostModel::NewTiles;

  // Optional terrain-aware penalty: discourage building roads over steep slopes.
  //
  // This is implemented as an *additional per-move cost* proportional to the
  // absolute height delta between adjacent tiles:
  //
  //   extraCost = round(|height(to) - height(from)| * slopeCost)
  //
  // Where height is the procedural heightfield stored in Tile::height.
  //
  // Notes:
  //  - slopeCost is interpreted as "cost units per 1.0 height delta".
  //  - slopeCost == 0 disables the feature (default).
  //  - By default, slope penalty is NOT applied when traversing existing roads,
  //    so the planner still strongly prefers reusing already-built roads.
  int slopeCost = 0;

  // If true, apply slopeCost even when stepping onto existing road tiles.
  // Default false so cost-surface routing prefers reusing roads.
  bool slopeCostAffectsExistingRoads = false;
};

// Find a road-building path between two tiles.
//
// The returned path is restricted to tiles where a road *can* exist:
//  - overlay is None or Road (we never punch through zones/parks)
//  - terrain must be non-water unless cfg.allowBridges == true
//
// The path is optimized according to cfg.costModel, and ties are broken by
// fewer steps, fewer turns, and stable per-tile variation bits.
//
// outPrimaryCost (if non-null) is the estimated primary cost optimized by the planner:
//  - base tile cost according to cfg.costModel
//  - plus (when cfg.slopeCost > 0) an extra per-move slope penalty
bool FindRoadBuildPath(const World& world, Point start, Point goal, std::vector<Point>& outPath,
                       int* outPrimaryCost = nullptr, const RoadBuildPathConfig& cfg = {});

// Multi-source / multi-target road-building path search.
//
// Finds the best road-build path from ANY tile in `starts` to ANY tile in `goals`.
//
// This is a generalization of FindRoadBuildPath() used by higher-level tooling
// (eg. road network resilience analysis) where you want to reconnect two *sets*
// of nodes without having to try every pair.
//
// Optional features:
//  - blockedDirectedMoves: if provided, forbids traversing specific directed
//    moves between adjacent tiles. Keys must use:
//        idx = y*world.width() + x
//        key = ((uint64_t)fromIdx << 32) | (uint32_t)toIdx
//    The vector is expected to be sorted (the function will still behave
//    correctly if it isn't, but may copy/sort internally).
//  - maxPrimaryCost: if >= 0, aborts if the best path would exceed this cost.
bool FindRoadBuildPathBetweenSets(const World& world,
                                 const std::vector<Point>& starts,
                                 const std::vector<Point>& goals,
                                 std::vector<Point>& outPath,
                                 int* outPrimaryCost = nullptr,
                                 const RoadBuildPathConfig& cfg = {},
                                 const std::vector<std::uint64_t>* blockedDirectedMoves = nullptr,
                                 int maxPrimaryCost = -1);

} // namespace isocity
