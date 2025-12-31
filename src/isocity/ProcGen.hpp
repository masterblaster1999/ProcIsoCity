#pragma once

#include "isocity/Erosion.hpp"
#include "isocity/World.hpp"

#include <cstdint>

namespace isocity {

struct ProcGenConfig {
  float terrainScale = 0.08f;     // noise scale
  float waterLevel = 0.35f;       // below => water
  float sandLevel = 0.42f;        // below => sand (above water)
  int hubs = 4;                   // number of "town centers"
  int extraConnections = 2;       // extra road connections between hubs
  float zoneChance = 0.22f;       // chance to place a zone next to a road
  float parkChance = 0.06f;       // chance to place a park next to a road

  // Optional post-noise terrain shaping stage.
  // Enabled by default in the current generation pipeline.
  ErosionConfig erosion{};
};

World GenerateWorld(int width, int height, std::uint64_t seed, const ProcGenConfig& cfg = {});

} // namespace isocity
