#pragma once

#include "isocity/Export.hpp"
#include "isocity/GfxPalette.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// Procedural “prop” sprites meant to sit on top of tiles (parks/roads) and add
// visual richness without external art assets.
//
// These sprites are intentionally lightweight, deterministic, and raylib-free so
// they can be generated in CI and consumed by headless tools.

enum class GfxPropKind : std::uint8_t {
  TreeDeciduous = 0,
  TreeConifer = 1,
  StreetLight = 2,
  VehicleCar = 3,
  VehicleTruck = 4,
};

struct GfxPropSprite {
  // Main color sprite (RGBA).
  RgbaImage color;

  // Optional emissive pass (RGBA). When not generated, emissive.rgba is empty.
  // Intended for night-time lights (street lamps, vehicle headlights, etc.).
  RgbaImage emissive;

  // Pivot point (in pixels) relative to the sprite top-left.
  //
  // For tile-sized props, this is typically (tileW/2, tileH/2).
  // For taller props (trees/streetlights), this is the ground-tile center.
  int pivotX = 0;
  int pivotY = 0;
};

struct GfxPropsConfig {
  int tileW = 64;
  int tileH = 32;

  // Fixed canvas height for tall props. If 0, derived from tileH.
  int tallSpriteH = 0;

  // Generate emissive map (streetlights, vehicle headlights).
  bool includeEmissive = false;
};

// Generate a single prop sprite for a given kind and deterministic variant.
//
// - variant is used only for deterministic variety; callers decide how many.
// - seed should typically match the tileset seed.
bool GenerateGfxPropSprite(GfxPropKind kind, int variant, std::uint32_t seed,
                           const GfxPropsConfig& cfg, const GfxPalette& pal,
                           GfxPropSprite& out, std::string& outError);

} // namespace isocity
