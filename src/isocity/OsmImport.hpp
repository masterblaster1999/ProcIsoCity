#pragma once

#include "isocity/World.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace isocity {

// Minimal, dependency-free OpenStreetMap (OSM XML) importer.
//
// This is intentionally not a full XML/OSM library. The goal is deterministic,
// portable ingestion suitable for headless tooling and regression pipelines.
//
// Supported (configurable) feature mapping:
//   - Roads:        highway=* ways           -> Overlay::Road (tiered levels)
//   - Water areas:  natural=water (closed)   -> Terrain::Water
//   - Waterways:    waterway=river/stream    -> Terrain::Water (thin line)
//   - Landuse:      landuse=residential/...  -> zone overlays
//   - Parks:        leisure=park/garden      -> Overlay::Park
//   - Buildings:    building=* (closed)      -> zone overlays (rough heuristic)

struct OsmLatLonBounds {
  double minLat = 0.0;
  double minLon = 0.0;
  double maxLat = 0.0;
  double maxLon = 0.0;
  bool valid = false;
};

struct OsmImportConfig {
  // If width/height are <= 0, the importer will choose a size based on
  // metersPerTile and the OSM bounds.
  int width = 0;
  int height = 0;

  // Auto-sizing resolution. Only used when width/height are not provided.
  // Typical values: 10..50.
  double metersPerTile = 20.0;

  // Padding (in tiles) around the rasterized OSM bounds when mapping to a
  // fixed grid.
  int padding = 2;

  // If true and a <bounds .../> tag exists, prefer it over scanning node
  // coordinates for extents.
  bool preferBoundsTag = true;

  // --- Feature toggles ---
  // Defaults preserve the original behavior (roads-only).
  bool importRoads = true;
  bool importWater = false;
  bool importLanduse = false;
  bool importParks = false;
  bool importBuildings = false;

  // If true, imported polygons (landuse/parks/buildings) may overwrite existing
  // non-road overlays. Roads are never overwritten by these features.
  //
  // This is most useful when importing both landuse AND buildings where a
  // building footprint might sit inside a different landuse polygon.
  bool overwriteNonRoadOverlays = false;

  // Road raster width:
  //   - If fixedRadius >= 0: use that Manhattan radius for all roads.
  //   - Else if thickenByClass: Street=0, Avenue=1, Highway=2.
  //   - Else: thin lines (radius 0).
  int fixedRadius = -1;
  bool thickenByClass = true;

  // Waterway (river/stream/canal) line thickness. Manhattan radius in tiles.
  // Only used when importWater=true.
  int waterwayRadius = 1;
};

struct OsmImportStats {
  std::size_t nodesParsed = 0;
  std::size_t waysParsed = 0;

  // --- Imported way counts ---
  std::size_t highwayWaysImported = 0;
  std::size_t waterWaysImported = 0;
  std::size_t landuseWaysImported = 0;
  std::size_t parkWaysImported = 0;
  std::size_t buildingWaysImported = 0;

  // --- Resulting tile counts (final world state) ---
  std::size_t roadTilesPainted = 0;
  std::size_t waterTilesPainted = 0;
  std::size_t zoneTilesPainted = 0;
  std::size_t parkTilesPainted = 0;

  OsmLatLonBounds bounds;
  int outWidth = 0;
  int outHeight = 0;
};

// Import tagged ways from an OSM XML file into an existing world.
//
// The world size is *not* changed; callers should ensure the world dimensions
// match the intended mapping. (Use ImportOsmXmlRoadsToNewWorld for auto-size.)
//
// Despite the function name, this may import more than roads if enabled in
// OsmImportConfig.
bool ImportOsmXmlRoads(const std::string& osmPath,
                       World& world,
                       const OsmImportConfig& cfg,
                       OsmImportStats* outStats,
                       std::string& outError);

// Convenience: parse OSM + choose world size (if cfg width/height <= 0),
// construct a new world, then import.
bool ImportOsmXmlRoadsToNewWorld(const std::string& osmPath,
                                 std::uint64_t seed,
                                 const OsmImportConfig& cfg,
                                 World& outWorld,
                                 OsmImportStats* outStats,
                                 std::string& outError);

} // namespace isocity
