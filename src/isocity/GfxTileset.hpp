#pragma once

#include "isocity/Export.hpp"
#include "isocity/GfxAtlasFx.hpp"
#include "isocity/GfxPalette.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Atlas packing mode.
//
// Grid: deterministic fixed-column layout (legacy/default).
// MaxRects: variable-size rectangle packing (usually much denser for trimmed/tall sprites).
enum class GfxAtlasPackMode {
  Grid,
  MaxRects,
};

struct GfxTilesetConfig {
  // Single-tile texture size in pixels. Defaults match the in-app Renderer.
  int tileW = 64;
  int tileH = 32;

  // Atlas packing.
  //
  // When packMode == Grid:
  //  - columns controls how many sprites per row.
  // When packMode == MaxRects:
  //  - columns is ignored.
  int columns = 32;

  // Pixels between sprites (also used as outer margin).
  int padding = 2;

  // Packing strategy.
  GfxAtlasPackMode packMode = GfxAtlasPackMode::Grid;

  // Optional target bin width for MaxRects packing (0 = auto).
  // The output atlas width can still be rounded up by packPow2.
  int packWidth = 0;

  // If true, round atlas dimensions up to the next power-of-two.
  // Useful for some GPU/engine import pipelines.
  bool packPow2 = false;

  // Optional: trim transparent borders around each sprite before packing.
  // This can substantially shrink atlas size for diamond tiles and cutout sprites.
  bool trimTransparent = false;

  // When trimTransparent is enabled, keep this many pixels of border around the
  // non-transparent bounds (0 = exact tight bounds).
  int trimBorder = 1;

  // Procedural seed and theme.
  std::uint32_t seed = 1;
  GfxTheme theme = GfxTheme::Classic;

  // Content toggles (useful for smaller atlases).
  bool includeTerrain = true;
  bool includeRoads = true;
  bool includeBridges = true;
  bool includeOverlays = true;


  // If true, generate terrain transition tiles (mask 0..15) for shoreline/biome blending.
  // These are "blob tiles" (4-neighbor auto-tiling) intended for external renderers / mods.
  bool includeTransitions = true;

  // Variants per transition mask (default: 4). Higher values reduce repetition but increase atlas size.
  int transitionVariants = 4;

  // Optional taller building sprites (RGBA, transparent background).
  // These are packed into the same atlas after the tile-sized sprites, but
  // will have a larger height than tileH. Metadata includes a per-sprite pivot.
  bool includeBuildings = false;

  // How many deterministic variants to generate for each (kind, level) pair.
  int buildingVariants = 12;

  // If >0, fixed sprite canvas height for buildings. If 0, derived from tileH.
  int buildingSpriteH = 0;

  // Optional civic/service facility sprites (schools/clinics/police/fire stations).
  // These are taller, recognizable silhouettes intended to complement simulation tools
  // like the service optimizer.
  bool includeFacilities = false;

  // How many deterministic variants to generate for each (kind, level) pair.
  int facilityVariants = 8;

  // If >0, fixed sprite canvas height for facilities. If 0, derived from tileH.
  int facilitySpriteH = 0;

  // If true, generate an emissive atlas (same layout) with window/sign lighting.
  bool includeEmissive = false;

  // -------------------------------------------------------------------------------------------
  // Optional prop sprites (trees, street furniture, vehicles)
  //
  // These are taller (trees/streetlights) or tile-sized (vehicles) sprites intended to be
  // composited on top of the base tiles by external renderers/tools.
  // -------------------------------------------------------------------------------------------

  // If true, include tall prop sprites (trees + streetlights).
  bool includeProps = false;

  // Deterministic variants per prop kind (trees and streetlights).
  int propVariants = 16;

  // Fixed canvas height for tall props (0 = auto based on tileH).
  int propSpriteH = 0;

  // If true, include vehicle sprites (cars + trucks). These are tile-sized.
  bool includeVehicles = false;

  // Deterministic variants per vehicle kind.
  int vehicleVariants = 12;

  // -------------------------------------------------------------------------------------------
  // Derived texture outputs (same layout as the main atlas)
  //
  // These are intended for external renderers / mod pipelines:
  //  - height maps can drive parallax/relief effects
  //  - normal maps allow simple dynamic lighting
  //  - shadow masks provide a cheap 'drop shadow' for tall sprites
  // -------------------------------------------------------------------------------------------

  // If true, generate a grayscale height atlas.
  bool includeHeight = false;

  // If true, generate a normal-map atlas.
  bool includeNormals = false;

  // If true, generate a shadow-mask atlas.
  bool includeShadows = false;

  // If true, generate a signed distance field (SDF) atlas.
  // The SDF is encoded in RGB as: v = clamp(0.5 + sd/spreadPx, 0, 1).
  bool includeSdf = false;

  // SDF configuration (only used when includeSdf is true).
  GfxSdfConfig sdf;

  // Height derivation mode shared by height/normal generation.
  GfxHeightMode heightMode = GfxHeightMode::AlphaLuma;

  // Normal map strength (gradient scale).
  float normalStrength = 2.0f;

  // Shadow configuration (only used when includeShadows is true).
  GfxShadowConfig shadow;

  // If true, shadows are only generated for sprites taller than tileH.
  bool shadowTallSpritesOnly = true;
};

struct GfxAtlasEntry {
  std::string name;
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;

  // Pivot point (in pixels) relative to the sprite top-left.
  // For tile-sized diamonds, this is typically the diamond center.
  // For taller sprites (buildings), this corresponds to the tile center on the ground.
  int pivotX = 0;
  int pivotY = 0;

  // Optional trimming information.
  //
  // If trimTransparent was enabled during atlas generation, the sprite rect (w,h)
  // represents a cropped view of a larger logical canvas (srcW,srcH).
  // trimX/trimY specify the top-left offset of the cropped rect within that canvas.
  //
  // If trimming was disabled, srcW/srcH == w/h and trimX/trimY == 0.
  int srcW = 0;
  int srcH = 0;
  int trimX = 0;
  int trimY = 0;
};

struct GfxTilesetResult {
  // Original tile size used for generating diamond tiles.
  // (This may differ from trimmed terrain sprite dimensions.)
  int tileW = 0;
  int tileH = 0;

  RgbaImage atlas;
  // Optional emissive atlas (same size/layout as atlas). Empty if not generated.
  RgbaImage emissiveAtlas;

  // Optional derived atlases (same size/layout as atlas). Empty if not generated.
  RgbaImage heightAtlas;
  RgbaImage normalAtlas;
  RgbaImage shadowAtlas;

  // Optional signed distance field atlas (same size/layout as atlas). Empty if not generated.
  RgbaImage sdfAtlas;

  // SDF metadata (useful for external renderers).
  float sdfSpreadPx = 0.0f;
  float sdfAlphaThreshold = 0.0f;
  bool sdfOpaqueAlpha = true;

  std::vector<GfxAtlasEntry> entries;
};

// Generate a sprite atlas containing the project's core procedural textures:
//  - terrain diamonds (water/sand/grass) with multiple noise variants
//  - optional terrain transition tiles (shorelines/biomes), mask 0..15, variant 0..N
//  - road auto-tiles (mask 0..15, variant 0..3) for levels 1..3
//  - bridge auto-tiles (mask 0..15, variant 0..3) for levels 1..3
//  - overlay diamonds (res/com/ind/park)
//
// The output is deterministic and raylib-free.
bool GenerateGfxTileset(const GfxTilesetConfig& cfg, GfxTilesetResult& out, std::string& outError);

// Write a simple JSON metadata file mapping sprite names -> atlas rectangles.
bool WriteGfxTilesetMetaJson(const std::string& path, const GfxTilesetResult& ts, std::string& outError);

} // namespace isocity
