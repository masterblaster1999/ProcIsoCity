#pragma once

#include "isocity/StreetNames.hpp"
#include "isocity/Types.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace isocity {

class World;

// -----------------------------------------------------------------------------------------------
// Wayfinding
//
// A deterministic, dependency-free "navigation" layer built on top of:
//  - StreetNames (streets + parcel addresses)
//  - Pathfinding (road A* routing)
//
// This module provides:
//  - A small address index + fuzzy street matching
//  - Geocoding of addresses / street intersections / coordinates
//  - Turn-by-turn style route instructions from a road-tile polyline
//
// Notes:
//  - Like StreetNames, this is currently derived data (not persisted).
//  - Outputs are designed for CLI tools and future in-game UI.
// -----------------------------------------------------------------------------------------------

struct AddressIndexConfig {
  // Enable edit-distance-based fallback matching when no exact street key is found.
  bool allowFuzzy = true;

  // Maximum number of street suggestions to return on a miss.
  int maxSuggestions = 5;

  // Maximum accepted edit distance for automatically selecting a fuzzy match.
  // Set to <=0 to always accept the best fuzzy match.
  int maxAutoEditDistance = 4;
};

struct AddressIndex {
  AddressIndexConfig cfg{};

  // Status for callers that want a simple "did the index build" check.
  //
  // Notes:
  //  - An index can be "ok" and still be empty (e.g. a roadless world or a very small map).
  //  - Geocoding of coordinate endpoints ("x,y") does not require any addresses.
  bool ok = false;
  std::string error;

  // All parcel addresses (as produced by BuildParcelAddresses()).
  std::vector<ParcelAddress> addresses;

  // Unique normalized street keys.
  std::vector<std::string> streetKeys;

  // Display name for each street key (first observed, stable).
  std::vector<std::string> streetDisplay;

  // For each street key, the indices (into addresses) that lie on that street.
  std::vector<std::vector<int>> streetToAddress;

  // Map normalized key -> streetKeys index.
  std::unordered_map<std::string, int> keyToStreet;

  void clear()
  {
    ok = false;
    error.clear();
    addresses.clear();
    streetKeys.clear();
    streetDisplay.clear();
    streetToAddress.clear();
    keyToStreet.clear();
  }
};

// Build an index for fast geocoding / street lookup.
AddressIndex BuildAddressIndex(const std::vector<ParcelAddress>& addresses,
                               const AddressIndexConfig& cfg = {});

struct GeocodeMatch {
  bool ok = false;

  // Index into AddressIndex.addresses when query resolves to a parcel address.
  // For non-parcel endpoints (intersection/coordinate), this is -1.
  int addressIndex = -1;

  // Resolved endpoint.
  ParcelAddress endpoint;

  // If ok=false, a human friendly error.
  std::string error;

  // Suggestions (street display names) for invalid or fuzzy queries.
  std::vector<std::string> suggestions;
};

// Resolve an endpoint query into a road-tile waypoint.
//
// Supported query syntaxes:
//  - "123 Asterwood Ave" (house number + street name)
//  - "Asterwood Ave" (street name -> picks median address on that street)
//  - "Asterwood Ave & 2nd St" (street intersection; also supports '@')
//  - "x,y" (tile coordinate; snapped to nearest road if needed)
GeocodeMatch GeocodeEndpoint(const World& world, const StreetNamingResult& streets,
                             const AddressIndex& index, const std::string& query);

// Navigation maneuver type (roughly similar to OSRM's step maneuvers).
struct RouteManeuver {
  std::string type;     // "depart", "turn", "continue", "arrive"
  std::string modifier; // "left", "right", "straight", "uturn"

  // Bearings in degrees clockwise from true north (0=N, 90=E, 180=S, 270=W).
  int bearingBefore = 0;
  int bearingAfter = 0;

  // Number of road-tile steps covered by this maneuver's travel segment.
  int steps = 0;

  // Street metadata for this travel segment.
  int streetId = -1;
  std::string streetName;

  // Indices into RouteResult.pathTiles (inclusive range) that correspond to this maneuver.
  // For travel maneuvers: [pathStart, pathEnd] spans steps+1 tiles.
  // For "arrive": pathStart=pathEnd=last tile.
  int pathStart = 0;
  int pathEnd = 0;

  std::string instruction;
};

enum class WayfindingRouteMetric : std::uint8_t {
  Steps = 0,
  TravelTime = 1,
};

// Configuration for route computation.
//
// The classic RouteBetweenEndpoints() function uses the default config (Steps).
// Use the overload that takes WayfindingRouteConfig for travel-time weighted routing
// and optional per-tile hazard avoidance penalties.
struct WayfindingRouteConfig {
  WayfindingRouteMetric metric = WayfindingRouteMetric::Steps;

  // Turn penalty (milli-steps) added when the direction changes.
  // Only affects routing when metric==TravelTime.
  int turnPenaltyMilli = 0;

  // Optional hazard weights in milli-steps added per tile at hazard01==1.
  // Set any weight to 0 to disable that factor.
  int wTrafficMilli = 0; // avoid high traffic volume
  int wCrashMilli = 0;   // avoid high crash-risk corridors
  int wCrimeMilli = 0;   // avoid high crime-risk corridors
  int wNoiseMilli = 0;   // avoid noisy corridors

  // When true, hazard models will ignore isolated road components (classic "outside connection" rule).
  bool requireOutsideConnectionForHazards = false;
};

struct RouteResult {
  bool ok = false;
  std::string error;

  ParcelAddress from;
  ParcelAddress to;

  // Start/end road tiles (may differ from parcel anchors).
  Point startRoad{0, 0};
  Point goalRoad{0, 0};

  // Road-tile polyline (inclusive of start and goal).
  std::vector<Point> pathTiles;

  // Path cost in road steps (== pathTiles.size()-1).
  int pathCost = 0;

  // Configuration used to compute this route.
  WayfindingRouteConfig routeCfg{};

  // Weighted costs (milli-steps). 1000 ~= one Street step.
  // These are always computed for reporting, but only affect routing when routeCfg.metric==TravelTime.
  int pathTravelTimeMilli = 0;      // base travel-time (road class speeds + bridges)
  int pathHazardPenaltyMilli = 0;   // sum of hazard avoidance penalties (traffic/crash/crime/noise)
  int pathTurnPenaltyMilli = 0;     // sum of turn penalties
  int pathCostMilli = 0;            // total = travel + hazard + turns

  // Turn-by-turn maneuvers.
  std::vector<RouteManeuver> maneuvers;
};

// Compute a road route between two endpoints (typically parcel addresses or intersections).
RouteResult RouteBetweenEndpoints(const World& world, const StreetNamingResult& streets,
                                  const ParcelAddress& from, const ParcelAddress& to);

// Overload with configurable metric / avoidance penalties.
RouteResult RouteBetweenEndpoints(const World& world, const StreetNamingResult& streets,
                                  const ParcelAddress& from, const ParcelAddress& to,
                                  const WayfindingRouteConfig& cfg);

} // namespace isocity
