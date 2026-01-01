#pragma once

#include "isocity/CityBlocks.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace isocity {

// CityBlockGraph: adjacency graph between CityBlocks across road tiles.
//
// Blocks (as produced by BuildCityBlocks) are 4-connected components of non-road land.
// This module builds a higher-level undirected graph where:
//  - nodes == blocks
//  - edges == "these two blocks touch the same road tile"
//
// Notes:
//  - Deterministic: edges are returned in sorted (a,b) order.
//  - This is a *derived* view: it does not mutate or persist anything.

// Per-block "frontage" metrics, broken down by road level.
// Road levels use the same convention as World::applyRoad:
//   1 = Street, 2 = Avenue, 3 = Highway
struct CityBlockFrontage {
  // Count of block boundary edges adjacent to road tiles, per road level.
  // Index 0 is unused.
  std::array<int, 4> roadEdgesByLevel{};

  // Number of block tiles that have at least one adjacent road tile of a given level.
  // A tile can contribute to multiple levels (e.g., adjacent to both a street and an avenue).
  // Index 0 is unused.
  std::array<int, 4> roadAdjTilesByLevel{};
};

// Undirected adjacency between two blocks.
struct CityBlockAdjacency {
  int a = -1;
  int b = -1;

  // Number of road tiles that are adjacent to both blocks.
  int touchingRoadTiles = 0;

  // Same, but broken down by road level of the road tile.
  // Index 0 is unused.
  std::array<int, 4> touchingRoadTilesByLevel{};
};

struct CityBlockGraphResult {
  CityBlocksResult blocks;

  // Per-block frontage metrics (size == blocks.blocks.size()).
  std::vector<CityBlockFrontage> frontage;

  // Sorted adjacency edges.
  std::vector<CityBlockAdjacency> edges;

  // For each block, list of incident edge indices.
  std::vector<std::vector<int>> blockToEdges;

  bool empty() const { return blocks.empty(); }
};

// Build block adjacency + frontage metrics for a world.
//
// If precomputedBlocks is provided, it is used as the block segmentation.
CityBlockGraphResult BuildCityBlockGraph(const World& world, const CityBlocksResult* precomputedBlocks = nullptr);

} // namespace isocity
