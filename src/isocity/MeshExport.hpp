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
//  - optional simple box buildings for zone tiles (and single-tile civic facilities)

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

  // Optional height quantization applied during export (after heightScale).
  //
  // This snaps terrain heights to a fixed step size, which can:
  //  - reduce mesh complexity when used with mergeTopSurfaces
  //  - make exported geometry more "blocky" / stylized (terraced)
  //
  // Units are OBJ/glTF units (after applying heightScale).
  // Set to 0 to disable quantization (default).
  float heightQuantization = 0.25f;

  // If true, merge adjacent top-surface tiles that share the same exported
  // material and quantized height into larger quads.
  //
  // This can dramatically reduce vertex/triangle count for flat regions.
  // It does not currently merge cliffs or buildings.
  bool mergeTopSurfaces = true;

  // Include the per-tile top surfaces (terrain and/or overlays).
  bool includeTopSurfaces = true;

  // Include vertical walls between adjacent tiles when their heights differ.
  bool includeCliffs = true;
  float cliffThreshold = 0.02f;

  // Include simple buildings (axis-aligned boxes) on zone tiles.
  bool includeBuildings = true;

  // If true, attempt to merge adjacent zone tiles into a single building volume
  // based on ZoneBuildingParcels.
  //
  // Notes:
  //  - Only parcels that are fully inside the export bounds are merged.
  //  - Parcels are merged only when their base terrain height range is within
  //    mergeBuildingsMaxBaseHeightRange after applying heightScale and optional heightQuantization.
  //  - Parcels that cannot be merged fall back to per-tile boxes.
  //
  // This reduces vertex/triangle counts dramatically for dense neighborhoods
  // and makes exported buildings match the renderer's parcelization behavior.
  bool mergeBuildings = true;
  // Maximum allowed difference between the highest and lowest tile base heights within a merged parcel.
  // A value of 0 requires a perfectly flat base. Values > 0 allow gentle slopes (useful on rolling terrain).
  float mergeBuildingsMaxBaseHeightRange = 1.0f;

  // Building footprint as a fraction of tileSize (0..1).
  float buildingFootprint = 0.70f;

  // Building height components (in multiples of tileSize).
  float buildingBaseHeight = 0.35f;
  float buildingPerLevelHeight = 0.45f;
  float buildingOccHeight = 0.35f;

  // Optional extra height (in multiples of tileSize) applied based on building
  // footprint area.
  //
  // When exporting merged building parcels, larger footprints generally look
  // better when they become taller. The boost uses log2(areaTiles) so single-tile
  // buildings are unchanged.
  float buildingAreaHeight = 0.15f;

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

// Set the mesh export config to the legacy (pre-merging) defaults.
//
// Notes:
//  - Intended for scripts/CLI tools that want stable backwards-compatible exports.
//  - Does NOT modify cfg.mtlFileName or cfg.objectName.
void ApplyLegacyMeshExportDefaults(MeshExportConfig& cfg);

} // namespace isocity
