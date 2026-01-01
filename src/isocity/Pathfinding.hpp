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
};

// Find a road-building path between two tiles.
//
// The returned path is restricted to tiles where a road *can* exist:
//  - overlay is None or Road (we never punch through zones/parks)
//  - terrain must be non-water unless cfg.allowBridges == true
//
// The path is optimized according to cfg.costModel, and ties are broken by
// fewer steps and stable per-tile variation bits.
//
// outPrimaryCost (if non-null) is:
//  - cfg.costModel==NewTiles => number of tiles in the path that are not already roads
//  - cfg.costModel==Money    => estimated money cost to build/upgrade the whole path
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
