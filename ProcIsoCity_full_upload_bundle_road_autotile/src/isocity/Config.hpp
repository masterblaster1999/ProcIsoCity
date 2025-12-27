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

  // 0 => auto-generate a seed.
  std::uint64_t seed = 0;

  bool vsync = true;
};

} // namespace isocity
