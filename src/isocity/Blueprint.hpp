#pragma once

#include "isocity/World.hpp"
#include "isocity/WorldPatch.hpp" // Reuse TileFieldMask + WorldPatchTileDelta.

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// A compact, transformable "stamp" of a rectangular region.
//
// Blueprints are intended for tooling and deterministic content generation:
//  - capture a rectangle of tiles from a World
//  - save/load a compact binary representation
//  - apply the stamp at a new location (optionally rotated/mirrored)
//
// The in-memory representation reuses WorldPatchTileDelta (index + mask + Tile value)
// with the important difference that indices are relative to the blueprint rectangle
// (0..width*height-1), not the full world.

enum class BlueprintCompression : std::uint8_t {
  None = 0,
  SLLZ = 1,
};

const char* BlueprintCompressionName(BlueprintCompression c);

struct Blueprint {
  int width = 0;
  int height = 0;

  // Version of the on-disk format that this blueprint was loaded from.
  // When creating a blueprint in memory, this is set to the current version.
  std::uint32_t version = 0;

  // Sparse list of tile deltas within the blueprint rectangle.
  // Each delta's `index` is in row-major order within the blueprint.
  std::vector<WorldPatchTileDelta> tiles;
};

// Capture options for building a blueprint from a world rectangle.
struct BlueprintCaptureOptions {
  // Which fields to store in each tile delta.
  // Default: overlay + level + district + variation.
  std::uint8_t fieldMask =
      static_cast<std::uint8_t>(TileFieldMask::Overlay) |
      static_cast<std::uint8_t>(TileFieldMask::Level) |
      static_cast<std::uint8_t>(TileFieldMask::District) |
      static_cast<std::uint8_t>(TileFieldMask::Variation);

  // If true, only emit deltas for tiles whose overlay != None (requires Overlay in fieldMask).
  // Useful for creating compact "stamp" blueprints.
  bool sparseByOverlay = true;

  // If true and Occupants is in fieldMask, store 0 occupants in the blueprint (layout-only).
  bool zeroOccupants = true;
};

// Capture a blueprint from the rectangle [x0,x0+w) x [y0,y0+h) in `world`.
//
// If any part of the rectangle is out of bounds, this fails.
bool CaptureBlueprintRect(const World& world, int x0, int y0, int w, int h,
                          Blueprint& outBlueprint, std::string& outError,
                          const BlueprintCaptureOptions& opt = {});

// How to treat tiles that are present in the blueprint.
//
// Replace: apply every tile delta as-is.
// Stamp:   skip tile deltas whose overlay == None (only if Overlay is present in the delta mask).
enum class BlueprintApplyMode : std::uint8_t {
  Replace = 0,
  Stamp = 1,
};

// Geometric transform applied when stamping.
struct BlueprintTransform {
  // Rotation in degrees clockwise. Allowed values: 0, 90, 180, 270.
  int rotateDeg = 0;
  bool mirrorX = false; // mirror horizontally after rotation
  bool mirrorY = false; // mirror vertically after rotation
};

struct BlueprintApplyOptions {
  BlueprintApplyMode mode = BlueprintApplyMode::Stamp;

  // AND-mask applied to each tile delta's mask at apply time.
  // This lets you load a blueprint that contains many fields but only apply a subset.
  std::uint8_t fieldMask = 0xFFu;

  // If false, fail when any transformed tile would land out of bounds.
  // If true, silently skip out-of-bounds deltas.
  bool allowOutOfBounds = false;

  // If false, disallow placing non-road overlays on water tiles (hard error).
  // If true, best-effort apply (World::setOverlay will still clamp some invalid ops).
  bool force = true;

  // When true, recompute the road auto-tiling masks after applying.
  bool recomputeRoadMasks = true;

  BlueprintTransform transform;
};

// Apply a blueprint at top-left destination tile (dstX, dstY) in `world`.
// Transform is applied relative to the blueprint's origin.
bool ApplyBlueprint(World& world, const Blueprint& bp, int dstX, int dstY,
                    const BlueprintApplyOptions& opt, std::string& outError);

// --- Blueprint operations ---

// Create a new blueprint by applying a geometric transform in blueprint space.
//
// This is useful for tooling (e.g., pre-rotating a stamp) and for verifying that
// apply-time transforms behave as expected.
bool TransformBlueprint(const Blueprint& src, const BlueprintTransform& tr,
                        Blueprint& outBlueprint, std::string& outError);

// Crop a blueprint to the minimal axis-aligned bounds that contain all tile deltas.
//
// The returned offsets (outOffsetX/Y) are the top-left corner of the crop region
// in the source blueprint. When applying the cropped blueprint into a world, add
// these offsets to your destination coordinate.
//
// If the source blueprint has no deltas, this produces a 1x1 empty blueprint and
// returns offsets (0,0).
bool CropBlueprintToDeltasBounds(const Blueprint& src, Blueprint& outBlueprint,
                                 int& outOffsetX, int& outOffsetY,
                                 std::string& outError, int pad = 0);

// Options for capturing a blueprint that represents a *diff* between two worlds.
struct BlueprintDiffOptions {
  // Field mask used both for comparison and for emitted deltas.
  // Only fields included here can appear in the resulting deltas.
  std::uint8_t fieldMask = 0xFFu;

  // When > 0, heights are considered equal if abs(a-b) <= heightEpsilon.
  // When <= 0, heights are compared exactly.
  float heightEpsilon = 0.0f;

  // If true and Occupants is in fieldMask, store 0 occupants in the diff blueprint
  // (layout-only diffs).
  bool zeroOccupants = false;
};

// Capture a blueprint representing the differences between two worlds over a rectangle.
//
// The output blueprint dimensions are (w,h), and delta indices are relative to the
// rectangle's origin.
//
// The two worlds must have identical dimensions.
bool CaptureBlueprintDiffRect(const World& baseWorld, const World& targetWorld,
                             int x0, int y0, int w, int h,
                             Blueprint& outBlueprint, std::string& outError,
                             const BlueprintDiffOptions& opt = {});


// --- Binary format ---
//
// Header:
//   magic[8]  = "ISOBLUEP"
//   u32       = version
//   i32       = width
//   i32       = height
//   u8        = compression (BlueprintCompression)
//   u32       = payloadSize (uncompressed bytes)
//   u32       = payloadSizeCompressed
// Payload (uncompressed):
//   varu32    = tileCount
//   repeated tile deltas (index delta varu32, mask u8, then fields in mask)
//
// This mirrors the WorldPatch tile delta encoding but omits base/target hashes and configs.

bool SerializeBlueprintBinary(const Blueprint& bp, std::vector<std::uint8_t>& outBytes,
                              std::string& outError,
                              BlueprintCompression compression = BlueprintCompression::SLLZ);

bool DeserializeBlueprintBinary(const std::vector<std::uint8_t>& bytes, Blueprint& outBlueprint,
                                std::string& outError);

bool SaveBlueprintBinary(const Blueprint& bp, const std::string& path,
                         std::string& outError,
                         BlueprintCompression compression = BlueprintCompression::SLLZ);

bool LoadBlueprintBinary(Blueprint& outBlueprint, const std::string& path,
                         std::string& outError);

} // namespace isocity
