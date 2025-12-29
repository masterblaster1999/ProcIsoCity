#pragma once

#include <cstdint>

namespace isocity {

struct Config {
  int windowWidth = 1280;
  int windowHeight = 720;

  int mapWidth = 96;
  int mapHeight = 96;

  // Isometric tile size in pixels (diamond bounding box).
  int tileWidth = 64;
  int tileHeight = 32;

  // --- Elevation rendering ---
  // Vertical offset is computed as: Tile::height * (tileHeight * elevationScale).
  // Set to 0 to render the world flat.
  float elevationScale = 0.75f;

  // 0 => smooth elevation. Otherwise snap to N steps for a terrace/voxel look.
  int elevationSteps = 16;

  // 0 => auto-generate a seed.
  std::uint64_t seed = 0;

  bool vsync = true;
};

} // namespace isocity
