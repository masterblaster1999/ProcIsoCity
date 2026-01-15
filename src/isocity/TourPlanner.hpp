#pragma once

#include "isocity/Cartography.hpp" // CartographyConfig, GenerateCityName, RenderLabeledIsoPoster
#include "isocity/Wayfinding.hpp"  // RouteResult

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

class World;

// A lightweight procedural "tour guide" layer on top of the generated city.
//
// The goal is to take a raw world (roads + zoning + parks + terrain) and synthesize:
//  - a small set of interesting POIs (peaks, parks, waterfront, bottlenecks, etc.)
//  - an ordered walking tour between them (using Wayfinding)
//  - an optional annotated poster image (Cartography + route overlay)
//
// This intentionally stays dependency-free and deterministic so it can be used in CLI tooling
// and regression tests.

enum class PoiKind : std::uint8_t {
  CivicCenter = 0,
  Bottleneck = 1,
  Peak = 2,
  Waterfront = 3,
  GrandPark = 4,
  Market = 5,
  Works = 6,
  DistrictHub = 7,
};

const char* PoiKindName(PoiKind k);

struct Poi {
  PoiKind kind = PoiKind::CivicCenter;
  int id = 0; // stable within a tour build

  std::string name;
  std::string description;

  // Primary road tile used for routing.
  Point roadTile{0, 0};

  // Best-effort context.
  int streetId = -1;
  std::string streetName;
  std::string nearAddress; // nearest generated parcel address (may be empty)

  int district = -1;
  std::string districtName;

  // For debug/exports.
  float featureValue = 0.0f;
  std::uint64_t score = 0;
};

struct TourStop {
  Poi poi;

  // Route from previous stop (or from start when stopIndex == 0).
  RouteResult routeFromPrev;
};

struct TourPlan {
  std::string title;

  std::uint64_t seed = 0;
  int width = 0;
  int height = 0;

  ParcelAddress start;
  std::string startQuery;

  std::vector<TourStop> stops;
  int totalSteps = 0;
};

struct TourConfig {
  // Maximum number of POIs in the tour (excludes the start).
  int maxStops = 6;

  // Enforce a minimum Manhattan separation between selected POIs.
  int minSeparationTiles = 10;

  // Centrality sampling to keep Brandes-based metrics fast on larger worlds.
  //  0 => exact (potentially expensive)
  //  N => deterministically sample N source nodes
  int centralityMaxSources = 48;

  // Feature toggles.
  bool includeBottleneck = true;
  bool includePeak = true;
  bool includeWaterfront = true;
  bool includePark = true;
  bool includeMarket = true;
  bool includeIndustry = true;
  bool includeDistrictHubs = true;

  // Limit the number of district hub POIs considered.
  int maxDistrictHubs = 2;

  // Tie-break salt so multiple tours can be generated for the same city.
  std::uint64_t seedSalt = 0;
};

// Poster rendering config (Cartography base image + route overlay + stop markers).
struct TourPosterConfig {
  ExportLayer layer = ExportLayer::Overlay;
  IsoOverviewConfig isoCfg{};
  StreetNamingConfig streetCfg{};
  CartographyConfig cartCfg{};

  bool drawRoute = true;
  bool drawMarkers = true;
  bool drawStopNumbers = true;
  bool drawKeyBox = true;

  // Route styling.
  int routeLineWidthPx = 2;
  std::uint8_t routeAlpha = 200;

  // Marker styling.
  int markerRadiusPx = 7;
  std::uint8_t markerAlpha = 235;
};

struct TourPosterResult {
  RgbaImage image;
  std::string title;
};

// Build a deterministic procedural tour plan for a world.
//
// If startQuery is empty, the tour starts near the structural city center.
// If startQuery is not empty, it can be:
//  - a parcel address ("120 Asterwood Ave")
//  - an intersection ("Asterwood Ave & 3rd St")
//  - a coordinate ("12,34")
TourPlan BuildProceduralTour(const World& world, const std::string& startQuery,
                             const TourConfig& cfg = {},
                             const StreetNamingConfig& streetCfg = {},
                             const AddressIndexConfig& indexCfg = {});

// Render an annotated tour poster: a labeled cartography poster with the tour route and stop markers.
TourPosterResult RenderTourPoster(const World& world, const TourPlan& tour, const TourPosterConfig& cfg);

} // namespace isocity
