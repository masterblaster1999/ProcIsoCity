#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// CityBlocks: connected components of *non-road* land.
//
// A "block" is a maximal 4-connected region of tiles where:
//  - terrain != Water
//  - overlay != Road
//
// Roads and water are treated as hard boundaries.
//
// Why this exists:
//  - Road networks naturally partition cities into blocks.
//  - Blocks are a useful primitive for higher-level systems (zoning heuristics,
//    services coverage, merged-building placement, district tools, analytics exports).
//  - Keeping the algorithm in the raylib-free core library makes it deterministic and testable.
//
// Notes:
//  - The block enumeration order is deterministic: row-major scan (y, then x).
//  - Edge counts are computed as *tile edges* (not unique boundary segments).
struct CityBlock {
  int id = -1;

  // Number of tiles in the block.
  int area = 0;

  // Axis-aligned bounds (inclusive).
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;

  // Boundary edge counts (4-neighborhood edges).
  int roadEdges = 0;    // edges adjacent to road tiles
  int waterEdges = 0;   // edges adjacent to water tiles
  int outsideEdges = 0; // edges adjacent to outside of the world

  // Convenience derived counts.
  int boundaryEdges() const { return roadEdges + waterEdges + outsideEdges; }

  // Number of tiles that have at least one adjacent road neighbor.
  int roadAdjTiles = 0;

  // Overlay composition inside the block (roads are excluded by definition).
  int parks = 0;
  int residential = 0;
  int commercial = 0;
  int industrial = 0;
  int other = 0;
};

struct CityBlocksResult {
  int w = 0;
  int h = 0;

  // Per-block aggregates.
  std::vector<CityBlock> blocks;

  // Per-tile mapping (size = w*h):
  //  - >=0 : block id
  //  - -1  : not in a block (water or road)
  std::vector<int> tileToBlock;

  bool empty() const { return w <= 0 || h <= 0 || tileToBlock.empty(); }
};

// Build city blocks for the given world.
CityBlocksResult BuildCityBlocks(const World& world);

} // namespace isocity
