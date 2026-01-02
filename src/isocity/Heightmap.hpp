#pragma once

#include "isocity/Export.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Heightmap utilities
//
// These functions support importing/exporting raster heightmaps for headless tooling.
//
// Image coordinate system:
//  - origin at top-left
//  - x increases to the right
//  - y increases downward
//
// This matches the coordinate conventions used by the headless CLI exports.
// -----------------------------------------------------------------------------------------------

enum class HeightmapResample : std::uint8_t {
  None = 0,    // require image dimensions match the world
  Nearest = 1, // nearest-neighbor sampling to fit the world dimensions
  Bilinear = 2 // bilinear sampling to fit the world dimensions
};

struct HeightmapApplyConfig {
  // How to handle image size mismatches.
  HeightmapResample resample = HeightmapResample::None;

  // Optional image axis flips before sampling.
  bool flipX = false;
  bool flipY = false;

  // Optional inversion (useful when a tool exports "white = low" instead of "white = high").
  bool invert = false;

  // Map grayscale (0..1) to world height:
  //   height = gray01 * heightScale + heightOffset
  float heightScale = 1.0f;
  float heightOffset = 0.0f;

  // Clamp the resulting height to [0,1] after applying scale/offset.
  bool clamp01 = true;

  // Optionally recompute Terrain from the resulting height.
  bool reclassifyTerrain = true;
  float waterLevel = 0.35f; // < waterLevel => Terrain::Water
  float sandLevel = 0.42f;  // < sandLevel  => Terrain::Sand (above water)

  // When reclassifying, tiles that become water can be made consistent by clearing
  // overlays that cannot exist on water (everything except roads/none).
  bool bulldozeNonRoadOverlaysOnWater = true;
};

struct HeightmapApplyStats {
  int worldW = 0;
  int worldH = 0;
  int srcW = 0;
  int srcH = 0;

  float minHeight = 0.0f;
  float maxHeight = 0.0f;
  double meanHeight = 0.0;
  double stdevHeight = 0.0;

  std::uint64_t waterTiles = 0;
  std::uint64_t sandTiles = 0;
  std::uint64_t grassTiles = 0;

  std::uint64_t overlaysCleared = 0;
};

// Apply a raster heightmap to a World.
//
// The input image is treated as grayscale luminance.
// Returns false on error (e.g. size mismatch when resample==None).
bool ApplyHeightmap(World& world, const PpmImage& img, const HeightmapApplyConfig& cfg, std::string& outError,
                    HeightmapApplyStats* outStats = nullptr);

struct HeightmapExportConfig {
  // If true, linearly remap world heights to [0,1] using the world min/max before quantization.
  // If false, heights are assumed to be in 0..1-ish space and are simply clamped (if clamp01).
  bool normalize = false;

  // Clamp heights to [0,1] before quantization (after normalization if normalize==true).
  bool clamp01 = true;

  // Invert the output grayscale (1-value).
  bool invert = false;
};

// Export the world's current Tile::height values into a grayscale RGB image.
//
// outMinHeight/outMaxHeight (optional) return the raw world min/max (before normalization/clamp),
// useful for writing sidecar metadata.
PpmImage ExportHeightmapImage(const World& world, const HeightmapExportConfig& cfg, float* outMinHeight = nullptr,
                             float* outMaxHeight = nullptr);

} // namespace isocity
