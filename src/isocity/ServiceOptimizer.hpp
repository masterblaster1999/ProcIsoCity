#pragma once

#include "isocity/Services.hpp"
#include "isocity/ZoneAccess.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Greedy, capacity-aware public service facility placement suggestions.
//
// The existing Services model (ComputeServices) can evaluate a set of civic facilities
// (education/health/safety) using an E2SFCA-style accessibility approach.
//
// This optimizer answers the next practical question:
//   "Where should I place the *next N* facilities so the city-wide satisfaction improves fastest?"
//
// Design goals:
//  - deterministic output (stable tie-breaking; no RNG)
//  - fast enough for headless CLI tooling
//  - does NOT mutate World during planning

struct ServiceOptimizerConfig {
  // Service model tuning used for demand weighting, distance decay, and the
  // access->satisfaction curve.
  ServicesModelSettings modelCfg{};

  // Which service we are optimizing for.
  ServiceType type = ServiceType::Education;

  // How many new facilities to propose.
  int facilitiesToAdd = 8;

  // Proposed facility level (1..3). Higher levels provide more supply in the
  // underlying services model.
  std::uint8_t facilityLevel = 1;

  // Candidate pruning: evaluate at most this many road access points.
  // Candidates are ranked by a simple local-demand heuristic.
  int candidateLimit = 700;

  // When true, facilities are placed on buildable empty land (Overlay::None)
  // adjacent to a road. When false, we fall back to placing facilities directly
  // on the access-road tile if no empty land is available.
  bool requireEmptyLand = true;

  // When true, the chosen facility tile must map back to the selected access road
  // under PickAdjacentRoadTile's deterministic order (N,E,S,W). This avoids
  // "facility connects to a different road than intended" surprises.
  bool requireStableAccessRoad = true;

  // Optional: avoid clustering facilities by enforcing a minimum manhattan
  // separation between access roads. 0 disables.
  // Units: milli-steps (street step ~= 1000).
  int minSeparationMilli = 0;

  // When true, only facilities of the same ServiceType are considered as
  // pre-existing competitors during planning (typical SimCity-style behavior).
  // When false, all existing facilities are included (mostly useful for
  // experimentation).
  bool considerOnlySameTypeExisting = true;
};

struct ServicePlacement {
  // Facility to add.
  ServiceFacility facility{};

  // Road tile used as the access point for distance scoring.
  Point accessRoad{0, 0};

  // Marginal objective gain (demand-weighted satisfaction increase) at the time
  // this facility was chosen.
  double marginalGain = 0.0;

  // Facility-local weighted demand in its catchment (used to compute ratio).
  double localDemandSum = 0.0;

  // Facility ratio (supply / localDemandSum) used by the heuristic.
  double ratio = 0.0;
};

struct ServiceOptimizerResult {
  int w = 0;
  int h = 0;
  ServiceOptimizerConfig cfg{};

  std::uint64_t totalDemandWeight = 0;
  int existingFacilities = 0;

  // Proposed facilities, in greedy selection order.
  std::vector<ServicePlacement> placements;
};

// Suggest new service facilities without mutating the world.
//
// - existingFacilities may include multiple types; cfg.considerOnlySameTypeExisting
//   controls whether other types are ignored.
// - precomputedZoneAccess / precomputedRoadToEdge are optional caches.
ServiceOptimizerResult SuggestServiceFacilities(const World& world,
                                               const ServiceOptimizerConfig& cfg,
                                               const std::vector<ServiceFacility>& existingFacilities = {},
                                               const ZoneAccessMap* precomputedZoneAccess = nullptr,
                                               const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr);

// Convenience: extract a facility list from placements.
std::vector<ServiceFacility> FacilitiesFromPlacements(const std::vector<ServicePlacement>& placements);

} // namespace isocity
