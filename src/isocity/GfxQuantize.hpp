#pragma once

#include "isocity/Export.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Deterministic RGBA -> indexed-color quantization intended for procedural sprite atlases.
//
// We keep this module dependency-free and stable across platforms so tilesets can be
// generated in CI with identical output.
//
// Implementation notes:
//  - Uses a weighted median-cut palette builder when unique colors exceed maxColors.
//  - Fully transparent pixels (A==0) are treated as background and mapped to palette index 0.
//  - Optional Floyd–Steinberg error diffusion can be enabled for nicer gradients.

struct GfxQuantizeConfig {
  // Max palette size (including the reserved transparent entry at index 0).
  // Valid range: [2, 256].
  int maxColors = 256;

  // If true, apply Floyd–Steinberg error diffusion when mapping pixels to palette entries.
  bool dither = false;

  // Dither strength multiplier in [0, +inf). 1.0 is standard.
  float ditherStrength = 1.0f;

  // If true, alpha participates in box splitting and nearest-color distance.
  // If false, RGB drives splitting/distance but alpha is still averaged into the palette.
  bool includeAlphaInDistance = true;
};

struct IndexedImage {
  int width = 0;
  int height = 0;

  // width*height bytes of palette indices.
  std::vector<std::uint8_t> indices;

  // Palette RGBA bytes: paletteSize*4.
  // Palette entry 0 is reserved for fully transparent pixels.
  std::vector<std::uint8_t> paletteRgba;
};

// Quantize an RGBA image to an indexed palette representation.
bool QuantizeRgbaToIndexed(const RgbaImage& src, const GfxQuantizeConfig& cfg,
                           IndexedImage& out, std::string& outError);

} // namespace isocity
