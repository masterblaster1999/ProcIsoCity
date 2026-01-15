#pragma once

#include "isocity/Types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

struct World;
struct ZoneAccessMap;
struct ZoneBuildingParcels;

// -----------------------------------------------------------------------------------------------
// StreetNames
//
// A deterministic, dependency-free "semantic" layer on top of the road grid.
//
// - Extracts the compressed road graph (RoadGraph) and groups edges into longer "streets"
//   using a continuation heuristic (straight-through at intersections, bend-through at corners).
// - Generates stable, pronounceable procedural names per street.
// - Assigns simple, deterministic addresses to zone building parcels.
//
// Notes:
// - This is intentionally NOT persisted to the save file yet. It is derived data.
// - Designed for headless tooling (CLI exports), but can later feed in-game UI.
// -----------------------------------------------------------------------------------------------

struct StreetNamingConfig {
  // Safety limit for extremely dense maps.
  int maxStreets = 4096;

  // When true, we try to continue a named street "through" intersections
  // when the straight continuation is unambiguous.
  bool mergeThroughIntersections = true;

  // When true, we allow a street to continue through 90-degree corners.
  // This helps organic roads that curve over many corner nodes.
  bool mergeThroughCorners = true;

  // If true, generate some "grid"-style ordinal street names (1st/2nd/3rd...).
  bool allowOrdinalNames = true;
};

struct StreetInfo {
  int id = -1;

  // Dominant road level for this street (1=Street, 2=Avenue, 3=Highway).
  int roadLevel = 1;

  // Procedurally generated display name (e.g., "Asterwood Ave", "I-17").
  std::string name;

  // Debug/analytics.
  int edgeCount = 0;
  int tileCount = 0;

  // Bounding box in tile coordinates.
  Point bboxMin{0, 0};
  Point bboxMax{0, 0};

  // Axis used for address numbering.
  // 0 = horizontal (x axis), 1 = vertical (y axis)
  int axis = 0;
  int axisMin = 0;
};

struct StreetNamingResult {
  int w = 0;
  int h = 0;

  // Per-tile mapping for road tiles: roadTileToStreetId[y*w+x] = street id, else -1.
  std::vector<int> roadTileToStreetId;

  std::vector<StreetInfo> streets;
};

struct AddressConfig {
  // House numbers increment by this step along the street axis.
  int numberStep = 10;

  // Even/odd assignment across the street is stable but arbitrary; flip if you want.
  bool flipParity = false;
};

struct ParcelAddress {
  int parcelIndex = -1;
  int streetId = -1;
  int houseNumber = 0;
  std::string streetName;
  std::string full;

  // For exporters.
  Point roadTile{0, 0};
  Point parcelAnchor{0, 0};
};

// Build street groupings + names.
StreetNamingResult BuildStreetNames(const World& world, const StreetNamingConfig& cfg = {});

// Assign parcel addresses for existing zones.
//
// If precomputedZoneAccess / precomputedParcels are provided, they are used.
// Otherwise, the function builds them internally.
std::vector<ParcelAddress> BuildParcelAddresses(const World& world,
                                                const StreetNamingResult& streets,
                                                const AddressConfig& cfg = {},
                                                const ZoneAccessMap* precomputedZoneAccess = nullptr,
                                                const ZoneBuildingParcels* precomputedParcels = nullptr);

} // namespace isocity
