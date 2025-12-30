#pragma once

#include <cstdint>

namespace isocity {

struct Config {
  int windowWidth = 1280;
  int windowHeight = 720;

  // Window behavior
  bool windowResizable = true;
  bool windowHighDPI = false;
  int windowMinWidth = 960;
  int windowMinHeight = 540;

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

  // --- World render resolution scaling ---
  //
  // This controls an optional off-screen render target for the *world layer*
  // (terrain + world overlays). The UI is still rendered at full resolution.
  //
  // 1.0  => render the world at native window resolution.
  // <1.0 => render the world to a smaller target then upscale (faster, blurrier).
  // >1.0 => supersample the world then downscale (sharper, slower).
  float worldRenderScale = 1.0f;

  // When enabled, the world render scale is automatically adjusted between
  // [worldRenderScaleMin, worldRenderScaleMax] to hit worldRenderTargetFps.
  bool worldRenderScaleAuto = false;
  float worldRenderScaleMin = 0.70f;
  float worldRenderScaleMax = 1.00f;
  int worldRenderTargetFps = 60;

  // Upscaling filter for the world render target.
  // true  => point/nearest (crisper, can shimmer when moving)
  // false => bilinear (smoother)
  bool worldRenderFilterPoint = false;
  bool mergedZoneBuildings = true;
};

} // namespace isocity
