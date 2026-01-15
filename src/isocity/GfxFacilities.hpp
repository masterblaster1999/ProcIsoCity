#pragma once

#include "isocity/Export.hpp"
#include "isocity/GfxPalette.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// Procedural isometric civic/service facility sprites.
//
// Facilities are distinct silhouettes meant for civic services so external tools
// (or future in-game placement UIs) can display recognizable schools/clinics/
// police stations/fire stations without requiring any authored art assets.

enum class GfxFacilityKind : std::uint8_t {
  // Education service (school / campus).
  Education = 0,
  // Health service (clinic / hospital).
  Health = 1,
  // Safety service variants (both map to ServiceType::Safety in the simulation).
  Police = 2,
  Fire = 3,
};

const char* GfxFacilityKindName(GfxFacilityKind k);
bool ParseGfxFacilityKind(const std::string& s, GfxFacilityKind& out);

struct GfxFacilitySprite {
  // Main color sprite (RGBA).
  RgbaImage color;

  // Optional emissive pass (RGBA) containing only lit pixels (signage/lights).
  // When not generated, emissive.rgba is empty.
  RgbaImage emissive;

  // Pivot (in pixels) relative to the sprite's top-left.
  // Intended to be aligned with the tile center at ground level.
  int pivotX = 0;
  int pivotY = 0;
};

struct GfxFacilitiesConfig {
  int tileW = 64;
  int tileH = 32;

  // Fixed sprite canvas height. If 0, an internal default derived from tileH
  // and the facility level is used.
  int spriteH = 0;

  // Generate emissive lights for signage / sirens.
  bool includeEmissive = false;
};

// Generate a single facility sprite for a given kind / level / variant.
//
// - level is clamped to [1,3]
// - variant is used purely for deterministic variety; callers decide how many.
//
// Returns true on success.
bool GenerateGfxFacilitySprite(GfxFacilityKind kind, int level, int variant, std::uint32_t seed,
                               const GfxFacilitiesConfig& cfg, const GfxPalette& pal, GfxFacilitySprite& out,
                               std::string& outError);

} // namespace isocity
