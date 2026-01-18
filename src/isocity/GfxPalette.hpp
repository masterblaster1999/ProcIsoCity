#pragma once

#include <cstdint>
#include <string>

namespace isocity {

// Tiny RGBA color type that is raylib-free so it can be used by headless tools.
struct Rgba8 {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t a = 255;
};

// High-level palette themes. These influence the *base hues* used for terrain,
// overlays, and road materials, but everything is still seeded and procedural.
//
// Classic attempts to match the existing in-app colors so that enabling the
// palette system doesn't radically change the project's default appearance.
enum class GfxTheme : std::uint8_t {
  Classic = 0,
  Autumn = 1,
  Desert = 2,
  Noir = 3,
  Neon = 4,
  Pastel = 5,
  SpaceColony = 6,
};

struct GfxPalette {
  // Terrain base colors (RGB) used before per-pixel detail is applied.
  Rgba8 water;
  Rgba8 sand;
  Rgba8 grass;

  // Road materials.
  Rgba8 roadAsphalt1;
  Rgba8 roadAsphalt2;
  Rgba8 roadAsphalt3;
  Rgba8 roadMarkWhite;
  Rgba8 roadMarkYellow;

  // Bridge deck materials.
  Rgba8 bridgeDeck1;
  Rgba8 bridgeDeck2;
  Rgba8 bridgeDeck3;

  // Overlay base colors (these are further patterned/dithered procedurally).
  Rgba8 overlayResidential;
  Rgba8 overlayCommercial;
  Rgba8 overlayIndustrial;
  Rgba8 overlayPark;

  // Small accent colors.
  Rgba8 shorelineFoam;
  Rgba8 treeDark;
};

// Deterministically synthesize a graphics palette.
//
// The output is stable across platforms (no <random>, only SplitMix-style RNG).
// `seed` is typically derived from the world seed, but can be decoupled for
// “same city, different art style” workflows.
GfxPalette GenerateGfxPalette(std::uint32_t seed, GfxTheme theme);

const char* GfxThemeName(GfxTheme t);
bool ParseGfxTheme(const std::string& s, GfxTheme& out);

} // namespace isocity
