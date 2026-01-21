#pragma once

#include "isocity/Export.hpp"     // RgbaImage
#include "isocity/GfxPalette.hpp" // GfxPalette

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Procedural "sigil" / badge graphics
// -----------------------------------------------------------------------------------------------
//
// ProcIsoCity is intentionally light on external art assets. Many visuals are
// generated procedurally (tiles, props, buildings). This module extends that
// philosophy to small square "badge" graphics that can be used as:
//  - district/region icons
//  - UI markers
//  - debug labels / legend items
//  - mod-friendly graphics packs
//
// The generator is deterministic, headless, and raylib-free.


enum class GfxSigilStyle : std::uint8_t {
  // Pick a deterministic style per variant.
  Random = 0,

  Blocks = 1,
  Starburst = 2,
  Chevron = 3,
};

const char* GfxSigilStyleName(GfxSigilStyle s);
bool ParseGfxSigilStyle(const std::string& s, GfxSigilStyle& out);

enum class GfxSigilGlyph : std::uint8_t {
  // Pick a deterministic glyph per variant.
  Random = 0,

  Triangle = 1,
  Dots = 2,
  Tower = 3,
};

const char* GfxSigilGlyphName(GfxSigilGlyph g);
bool ParseGfxSigilGlyph(const std::string& s, GfxSigilGlyph& out);

struct GfxSigilConfig {
  // Output icon size in pixels (square).
  int sizePx = 64;

  // Badge style.
  GfxSigilStyle style = GfxSigilStyle::Random;

  // Center glyph kind.
  GfxSigilGlyph glyph = GfxSigilGlyph::Random;

  // Probability of drawing the center glyph (0..1). If glyph is not Random,
  // any value > 0 will force drawing.
  float glyphChance = 0.85f;

  // Border thickness in pixels. If 0, the generator chooses a small default.
  int borderPx = 0;

  // If true, the icon background outside the circular badge is transparent.
  // If false, the full square is filled.
  bool transparentOutside = true;
};

// Generate a single badge icon.
//
// - variant selects a deterministic variant for the given seed.
// - seed should typically be derived from the world seed.
// - pal is the palette used for colors.
bool GenerateGfxSigil(int variant, std::uint32_t seed, const GfxSigilConfig& cfg,
                      const GfxPalette& pal, RgbaImage& out, std::string& outError);

// Generate a sprite sheet containing multiple sigils in a grid layout.
//
// - count: number of icons to generate.
// - columns: icons per row (>= 1).
// - outNames (optional): receives per-icon names ("sigil_0", ...).
bool GenerateGfxSigilSheet(int count, int columns, std::uint32_t seed, const GfxSigilConfig& cfg,
                           const GfxPalette& pal, RgbaImage& out,
                           std::vector<std::string>* outNames,
                           std::string& outError);

} // namespace isocity
