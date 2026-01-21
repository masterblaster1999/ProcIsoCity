#pragma once

#include "isocity/Export.hpp"     // RgbaImage
#include "isocity/GfxPalette.hpp" // GfxPalette

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Procedural seamless pattern tiles
// -----------------------------------------------------------------------------------------------
//
// These are small square RGBA textures intended for:
//  - UI backgrounds (grain, hatch, subtle noise)
//  - overlay patterns for mod tools
//  - external renderers that want a deterministic "style pack" without shipping art assets
//
// The generator is deterministic, dependency-free, and supports seamless tiling.


enum class GfxPatternStyle : std::uint8_t {
  // Pick a deterministic style per variant.
  Random = 0,

  Grain = 1,
  Hatch = 2,
  Bricks = 3,
  Waves = 4,
};

const char* GfxPatternStyleName(GfxPatternStyle s);
bool ParseGfxPatternStyle(const std::string& s, GfxPatternStyle& out);

struct GfxPatternConfig {
  // Output tile size in pixels (square).
  int sizePx = 64;

  // Pattern style.
  GfxPatternStyle style = GfxPatternStyle::Random;

  // If true, the pattern edges match so the tile can repeat seamlessly.
  bool tileable = true;

  // Internal period used for periodic noise (in noise-domain units).
  // Only used when tileable == true. Typical: 16..64.
  int period = 32;

  // Contrast multiplier applied to pattern modulation. 1.0 is neutral.
  float contrast = 1.0f;
};

// Generate a single pattern tile.
//
// - variant selects a deterministic variant for the given seed.
// - seed should typically be derived from the world seed.
// - pal is the palette used for colors.
bool GenerateGfxPattern(int variant, std::uint32_t seed, const GfxPatternConfig& cfg,
                        const GfxPalette& pal, RgbaImage& out, std::string& outError);

// Generate a sprite sheet containing multiple patterns in a grid layout.
//
// - count: number of tiles to generate.
// - columns: tiles per row (>= 1).
// - outNames (optional): receives per-tile names ("pattern_0", ...).
bool GenerateGfxPatternSheet(int count, int columns, std::uint32_t seed, const GfxPatternConfig& cfg,
                             const GfxPalette& pal, RgbaImage& out,
                             std::vector<std::string>* outNames,
                             std::string& outError);

} // namespace isocity
