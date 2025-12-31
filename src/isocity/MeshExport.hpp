#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <iosfwd>
#include <string>

namespace isocity {

// Export a world as a simple 3D mesh (Wavefront OBJ + MTL).
//
// This is intentionally lightweight:
//  - no external dependencies
//  - deterministic output (driven entirely by tile data)
//  - suitable for tooling / offline inspection / interoperability with DCC tools
//
// The exporter produces:
//  - a top surface per tile (terrain OR overlay surface)
//  - optional "cliff" side walls between tiles whose heights differ
//  - optional simple box buildings for zone tiles

struct MeshExportConfig {
  // If non-empty, WriteWorldObjMtl() writes a `mtllib <name>` line.
  // When exporting to files via ExportWorldObjMtl(), this defaults to the
  // filename of the provided mtlPath.
  std::string mtlFileName;

  // Optional object name (`o <name>`). If empty, "world" is used.
  std::string objectName;

  // Horizontal scale of a tile in OBJ units.
  float tileSize = 1.0f;

  // Vertical scale applied to Tile::height.
  // Terrain heights are typically ~[-0.2..1.0], so a larger scale helps readability.
  float heightScale = 8.0f;

  // If a tile has a non-None overlay, its top surface is raised by this amount.
  // This is purely visual and avoids z-fighting when importing into engines.
  float overlayOffset = 0.02f;

  // Include the per-tile top surfaces (terrain and/or overlays).
  bool includeTopSurfaces = true;

  // Include vertical walls between adjacent tiles when their heights differ.
  bool includeCliffs = true;
  float cliffThreshold = 0.02f;

  // Include simple buildings (axis-aligned boxes) on zone tiles.
  bool includeBuildings = true;

  // Building footprint as a fraction of tileSize (0..1).
  float buildingFootprint = 0.60f;

  // Building height components (in multiples of tileSize).
  float buildingBaseHeight = 0.35f;
  float buildingPerLevelHeight = 0.45f;
  float buildingOccHeight = 0.35f;

  // Optional crop rectangle (tile-space) to export only a subregion.
  bool hasCrop = false;
  int cropX = 0;
  int cropY = 0;
  int cropW = 0;
  int cropH = 0;

  // If true and hasCrop, the OBJ origin is shifted so that (cropX,cropY) maps to (0,0).
  bool originAtCrop = true;
};

struct MeshExportStats {
  std::uint64_t vertices = 0;
  std::uint64_t triangles = 0;
};

// Stream an OBJ + MTL pair.
// Returns false on invalid cfg (e.g., bad crop) or stream failure.
bool WriteWorldObjMtl(std::ostream& objOut, std::ostream& mtlOut, const World& world,
                      const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError);

// Convenience wrapper that writes two files.
bool ExportWorldObjMtl(const std::string& objPath, const std::string& mtlPath, const World& world,
                       const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError);

} // namespace isocity
