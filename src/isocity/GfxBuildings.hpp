#pragma once

#include "isocity/Export.hpp"
#include "isocity/GfxPalette.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// Procedural isometric building sprites.
//
// Sprites are generated on a transparent RGBA canvas sized to allow vertical
// extension above the base tile. A pivot is provided so renderers can align
// the sprite to the tile center on the ground plane.

enum class GfxBuildingKind : std::uint8_t {
  Residential = 0,
  Commercial = 1,
  Industrial = 2,
};

// Number of high-level "style families" for building variants.
//
// Callers can generate many variants per level, then group them into a small
// number of families (e.g., per neighborhood) to keep architecture coherent.
//
// This constant is shared between sprite generation and renderers.
constexpr int kGfxBuildingVariantFamilies = 4;

struct GfxBuildingSprite {
  // Main color sprite (RGBA).
  RgbaImage color;

  // Optional emissive pass (RGBA) containing only lit pixels (windows, signs).
  // When not generated, emissive.rgba is empty.
  RgbaImage emissive;

  // Pivot (in pixels) relative to the sprite's top-left.
  // Intended to be aligned with the tile center at ground level.
  int pivotX = 0;
  int pivotY = 0;
};

struct GfxBuildingsConfig {
  int tileW = 64;
  int tileH = 32;

  // Fixed sprite canvas height. If 0, an internal default derived from tileH
  // is used (large enough for level-3 commercial buildings).
  int spriteH = 0;

  // Max vertical extrusion height in pixels. If 0, derived from tileH.
  int maxHeightPx = 0;

  // Generate emissive lights for windows/signage.
  bool includeEmissive = false;
};

// Generate a single building sprite for a given zone kind / level / variant.
//
// - level is clamped to [1,3]
// - variant is used purely for deterministic variety; callers decide how many.
//
// Returns true on success.
bool GenerateGfxBuildingSprite(GfxBuildingKind kind, int level, int variant, std::uint32_t seed,
                               const GfxBuildingsConfig& cfg, const GfxPalette& pal, GfxBuildingSprite& out,
                               std::string& outError);

} // namespace isocity
