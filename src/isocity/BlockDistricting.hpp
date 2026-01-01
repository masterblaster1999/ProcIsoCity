#pragma once

#include "isocity/CityBlockGraph.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace isocity {

// Block-based districting: assign administrative districts using CityBlocks + their adjacency graph.
//
// Rationale:
//  - Road-based district seeding is great for transportation-centric partitions.
//  - For "neighborhood" style partitions, it is often more intuitive to use city blocks
//    as the unit of clustering and then flood-fill ownership across the block adjacency graph.
//
// The algorithm is deterministic:
//  1) Build CityBlocks + CityBlockGraph.
//  2) Pick up to K seed blocks using farthest-point sampling over the block graph
//     (unweighted hop distance). Seeds start with the largest block (area), then repeatedly
//     pick the block maximizing distance to the nearest existing seed.
//  3) Assign each block to the lexicographically smallest (distance, seedIndex) via a
//     multi-source Dijkstra on the block graph.
//  4) Write district IDs onto tiles. Roads can optionally inherit the majority district
//     of adjacent blocks.

struct BlockDistrictConfig {
  // Requested number of districts. Clamped to [1, kDistrictCount] and to blockCount.
  int districts = kDistrictCount;

  // If true, assign road tiles based on adjacent blocks.
  bool fillRoadTiles = true;

  // If true, allow water tiles to be assigned. If false, water remains unchanged.
  // (Most maps keep water as district 0 for simplicity.)
  bool includeWater = false;
};

struct BlockDistrictResult {
  int districtsRequested = kDistrictCount;
  int districtsUsed = 0;

  // Block IDs chosen as seeds, in seedIndex order (seedIndex == district id).
  std::vector<int> seedBlockId;

  // Per-block district assignment (size == blockCount).
  std::vector<std::uint8_t> blockToDistrict;

  // Summary counts.
  std::array<int, kDistrictCount> blocksPerDistrict{};
  std::array<int, kDistrictCount> tilesPerDistrict{};
};

// Compute a district assignment for blocks in a world (does not mutate the world).
//
// If precomputedGraph is provided, it is used directly (must match the world).
BlockDistrictResult ComputeBlockDistricts(const World& world, const BlockDistrictConfig& cfg,
                                         const CityBlockGraphResult* precomputedGraph = nullptr);

// Compute and write block-based district IDs into the world.
BlockDistrictResult AssignDistrictsByBlocks(World& world, const BlockDistrictConfig& cfg,
                                           const CityBlockGraphResult* precomputedGraph = nullptr);

} // namespace isocity
