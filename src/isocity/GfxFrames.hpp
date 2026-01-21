#pragma once

#include "isocity/Export.hpp"     // RgbaImage
#include "isocity/GfxPalette.hpp" // GfxPalette

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Procedural UI frame / panel graphics
// -----------------------------------------------------------------------------------------------
//
// These are square RGBA sprites intended for UI widgets / panels.
//
// Typical use cases:
//  - 9-slice UI panels (border + center)
//  - in-game signage / decals
//  - mod packs that want coherent art without shipping external assets
//
// Frames are deterministic and raylib-free.


enum class GfxFrameDeco : std::uint8_t {
  // Pick a deterministic decoration per variant.
  Random = 0,

  CornerTriangles = 1,
  CornerDots = 2,
  TitleBar = 3,
};

const char* GfxFrameDecoName(GfxFrameDeco d);
bool ParseGfxFrameDeco(const std::string& s, GfxFrameDeco& out);

struct GfxFrameConfig {
  // Output sprite size in pixels (square).
  int sizePx = 64;

  // Decoration style.
  GfxFrameDeco deco = GfxFrameDeco::Random;

  // Border thickness in pixels. If 0, the generator chooses a reasonable default.
  int borderPx = 0;

  // Optional rounded-corner radius in pixels.
  // If > 0, pixels outside the rounded rectangle become transparent.
  int cornerRadiusPx = 0;

  // Pattern blend strength in [0,1]. 0 disables the interior pattern overlay.
  float patternStrength = 0.35f;
};

// Generate a single UI frame sprite.
//
// - variant selects a deterministic variant for the given seed.
// - seed should typically be derived from the world seed.
// - pal provides the color palette.
bool GenerateGfxFrame(int variant, std::uint32_t seed, const GfxFrameConfig& cfg,
                      const GfxPalette& pal, RgbaImage& out, std::string& outError);

// Generate a sprite sheet containing multiple frames in a grid layout.
//
// - count: number of tiles to generate.
// - columns: tiles per row (>= 1).
// - outNames (optional): receives per-tile names ("frame_0", ...).
bool GenerateGfxFrameSheet(int count, int columns, std::uint32_t seed, const GfxFrameConfig& cfg,
                           const GfxPalette& pal, RgbaImage& out,
                           std::vector<std::string>* outNames,
                           std::string& outError);

} // namespace isocity
