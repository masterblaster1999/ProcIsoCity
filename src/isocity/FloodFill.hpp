#pragma once

#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Deterministic flood-fill helpers used by both headless tooling (scripts/tests)
// and the in-game editor.
//
// We intentionally keep this in the core (raylib-free) library so:
//   - behavior stays identical across CLI and interactive builds
//   - scripts/regressions can rely on the exact same region selection

enum class FloodFillMode : std::uint8_t {
  RoadComponent = 0,
  WaterBody = 1,
  LandBlock = 2,
};

struct FloodFillResult {
  int w = 0;
  int h = 0;

  // Flat array (size w*h). 1 => tile is part of the filled region.
  std::vector<std::uint8_t> mask;

  // Deterministic visit order list of tiles in the filled region.
  // (The specific order is stable for a given world + start + config, but
  // callers should not rely on any particular traversal semantics.)
  std::vector<Point> tiles;
};

// Pick a reasonable fill mode based on the start tile.
// Matches the editor behavior:
//  - clicking a road fills the entire connected road component
//  - clicking water (excluding bridges) fills the connected water body
//  - otherwise fills the connected land block (bounded by water and optionally roads)
FloodFillMode ChooseFloodFillMode(const World& world, Point start);

// Compute a filled region starting at `start`.
//
// includeRoadsInLandBlock:
//  - only applies to FloodFillMode::LandBlock
//  - when false, roads are treated as solid boundaries
//  - when true, the fill can traverse and include roads (useful for district painting
//    and other "cross-block" tools)
FloodFillResult FloodFillRegion(const World& world, Point start, FloodFillMode mode,
                                bool includeRoadsInLandBlock = false);

// Convenience wrapper: ChooseFloodFillMode() + FloodFillRegion().
FloodFillResult FloodFillAuto(const World& world, Point start, bool includeRoadsInLandBlock = false);

} // namespace isocity
