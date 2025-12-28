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

// Find a road-building path between two tiles.
//
// The returned path is restricted to buildable tiles where a road *can* exist
// (terrain != Water, overlay is None or Road). This makes it suitable for UI
// tools that want to "drag a road" without punching through zones/parks.
//
// The path is optimized for *build cost* (number of non-road tiles in the path),
// and ties are broken by shorter length.
//
// outBuildCost (if non-null) is the number of tiles in the path that are not
// already roads (i.e. how many new road tiles would be created).
bool FindRoadBuildPath(const World& world, Point start, Point goal, std::vector<Point>& outPath,
                       int* outBuildCost = nullptr);

} // namespace isocity
