#pragma once

#include "isocity/World.hpp"

#include <string>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// WorldTransform
//
// Utilities for applying simple geometric transforms to an entire World (rotate/mirror/crop).
//
// Semantics:
//   - Rotation is clockwise in tile space, about the origin (0,0).
//   - mirrorX/mirrorY are applied AFTER rotation, in the rotated coordinate system.
//     mirrorX flips horizontally (x -> w-1-x).
//     mirrorY flips vertically (y -> h-1-y).
//   - Crop is applied last, in the rotated/mirrored space.
//
// Notes:
//   - The resulting world always recomputes road auto-tiling masks (Tile::variation low bits)
//     so the world is immediately render-safe without requiring a load/recompute cycle.
// -----------------------------------------------------------------------------------------------

struct WorldTransformConfig {
  // Clockwise rotation (degrees). Supported: 0, 90, 180, 270.
  int rotateDeg = 0;

  // Optional mirroring after rotation.
  bool mirrorX = false;
  bool mirrorY = false;

  // Optional crop applied after rotation/mirroring.
  bool hasCrop = false;
  int cropX = 0;
  int cropY = 0;
  int cropW = 0;
  int cropH = 0;
};

// Validate cfg for a given source dimension.
// Returns true if valid; otherwise sets outError.
bool ValidateWorldTransform(const WorldTransformConfig& cfg, int srcW, int srcH, std::string& outError);

// Compute output dimensions after applying the transform (including crop).
bool ComputeWorldTransformDims(const WorldTransformConfig& cfg, int srcW, int srcH, int& outW, int& outH,
                              std::string& outError);

// Map an output coordinate (xOut, yOut) in the transformed world back to the corresponding source tile.
// Returns false (with outError) if the mapping is invalid/out-of-range.
bool MapTransformedToSource(const WorldTransformConfig& cfg, int srcW, int srcH, int xOut, int yOut, int& outSrcX,
                            int& outSrcY, std::string& outError);

// Apply the transform to src and write the result to outWorld.
//
// copyStats:
//   If true, copies Stats from src into the output world before performing any fixups.
//
// The resulting world always recomputes road auto-tiling masks (Tile::variation low bits).
bool TransformWorld(const World& src, World& outWorld, const WorldTransformConfig& cfg, std::string& outError,
                    bool copyStats = true);

} // namespace isocity
