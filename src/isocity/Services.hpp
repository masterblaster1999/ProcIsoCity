#pragma once

#include "isocity/Isochrone.hpp"
#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace isocity {

// Public services / civic accessibility model.
//
// This module is intentionally headless and renderer-independent.
// It provides an accessibility-to-satisfaction field that other layers
// (Simulator, UI overlays, optimizers) can consume.

enum class ServiceType : std::uint8_t {
  Education = 0,
  Health = 1,
  Safety = 2,
};

inline const char* ServiceTypeName(ServiceType t)
{
  switch (t) {
    case ServiceType::Education: return "education";
    case ServiceType::Health: return "health";
    case ServiceType::Safety: return "safety";
  }
  return "service";
}

enum class ServiceDemandMode : std::uint8_t {
  Tiles = 0,     // each eligible zone tile contributes weight=1
  Occupants = 1, // each eligible zone tile contributes weight=Tile::occupants
};

struct ServiceFacility {
  // Facility location in tile coordinates.
  Point tile{0, 0};

  ServiceType type = ServiceType::Education;

  // Facility level (1..3). Higher levels are assumed to have higher capacity/cost.
  std::uint8_t level = 1;

  // Master toggle for the facility.
  bool enabled = true;
};

// Non-persistent runtime tuning for the services model.
//
// Like the transit/trade model settings, this struct is *not* persisted in saves
// to avoid save-format churn while iterating on experimental mechanics.
struct ServicesModelSettings {
  bool enabled = false;

  // If true, only roads connected to the map edge are considered valid
  // access networks for facilities/demand.
  bool requireOutsideConnection = true;

  // How road distance is measured (steps vs travel-time weighted).
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;

  // Catchment radius for facilities.
  // Unit: "street-step equivalents" (1 street step ~= 1000 milli).
  int catchmentRadiusSteps = 18;

  // Distance-decay approximation (E2SFCA-style): three bands inside the catchment.
  //
  // The band cut points are fractions of the catchment radius.
  // Example defaults:
  //   0..0.33 => weight 1.00
  //   0.33..0.66 => weight 0.60
  //   0.66..1.00 => weight 0.30
  std::array<float, 3> distanceBandWeight{1.0f, 0.6f, 0.3f};
  std::array<float, 2> distanceBandCutFrac{0.33f, 0.66f};

  // How demand is weighted on zone tiles.
  ServiceDemandMode demandMode = ServiceDemandMode::Occupants;

  // Which zones contribute demand.
  bool demandResidential = true;
  bool demandCommercial = true;
  bool demandIndustrial = true;

  // Per-service demand multipliers applied to the base demand weight.
  float educationDemandMult = 1.0f;
  float healthDemandMult = 1.0f;
  float safetyDemandMult = 1.0f;

  // Facility capacity ("service units") provided per day, per facility level.
  // These are interpreted relative to demand weights (tiles or occupants).
  std::array<int, 3> educationSupplyPerLevel{200, 500, 900};
  std::array<int, 3> healthSupplyPerLevel{200, 500, 900};
  std::array<int, 3> safetySupplyPerLevel{150, 350, 700};

  // Optional per-day maintenance costs per facility (used later by the simulator).
  std::array<int, 3> educationMaintenancePerDay{1, 2, 4};
  std::array<int, 3> healthMaintenancePerDay{1, 2, 4};
  std::array<int, 3> safetyMaintenancePerDay{1, 2, 4};

  // Accessibility-to-satisfaction mapping.
  //
  // targetAccess is interpreted as the level of accessibility where a tile hits
  // roughly 50% satisfaction (using a smooth saturating curve).
  float targetAccess = 0.8f;

  bool operator==(const ServicesModelSettings& o) const
  {
    return enabled == o.enabled && requireOutsideConnection == o.requireOutsideConnection && weightMode == o.weightMode &&
           catchmentRadiusSteps == o.catchmentRadiusSteps && distanceBandWeight == o.distanceBandWeight &&
           distanceBandCutFrac == o.distanceBandCutFrac && demandMode == o.demandMode &&
           demandResidential == o.demandResidential && demandCommercial == o.demandCommercial &&
           demandIndustrial == o.demandIndustrial && educationDemandMult == o.educationDemandMult &&
           healthDemandMult == o.healthDemandMult && safetyDemandMult == o.safetyDemandMult &&
           educationSupplyPerLevel == o.educationSupplyPerLevel && healthSupplyPerLevel == o.healthSupplyPerLevel &&
           safetySupplyPerLevel == o.safetySupplyPerLevel && educationMaintenancePerDay == o.educationMaintenancePerDay &&
           healthMaintenancePerDay == o.healthMaintenancePerDay && safetyMaintenancePerDay == o.safetyMaintenancePerDay &&
           targetAccess == o.targetAccess;
  }

  bool operator!=(const ServicesModelSettings& o) const { return !(*this == o); }
};

struct ZoneAccessMap;

struct ServicesResult {
  int w = 0;
  int h = 0;
  ServicesModelSettings cfg{};

  // Facility counts by ServiceType index.
  std::array<int, 3> totalFacilities{};
  std::array<int, 3> activeFacilities{};

  // Demand-weighted citywide satisfaction (0..1).
  float educationSatisfaction = 0.0f;
  float healthSatisfaction = 0.0f;
  float safetySatisfaction = 0.0f;
  float overallSatisfaction = 0.0f;

  // Sum of per-day maintenance costs for active facilities.
  int maintenanceCostPerDay = 0;

  // Per-tile satisfaction fields (size w*h). Values are in [0,1].
  std::vector<float> education;
  std::vector<float> health;
  std::vector<float> safety;
  std::vector<float> overall;
};

// Compute services satisfaction fields using an E2SFCA-style accessibility model.
//
// Inputs:
//  - facilities: explicit facility locations/capacities (service buildings will hook into this later).
//  - precomputedZoneAccess / precomputedRoadToEdge: optional caches from other sim subsystems.
ServicesResult ComputeServices(const World& world, const ServicesModelSettings& cfg,
                               const std::vector<ServiceFacility>& facilities,
                               const ZoneAccessMap* precomputedZoneAccess = nullptr,
                               const std::vector<std::uint8_t>* precomputedRoadToEdge = nullptr);

// Convenience: scan the World for service/civic facility tiles and build a facility list
// suitable for ComputeServices().
//
// This makes it easy to hook the services model into the simulator without introducing
// a separate entity system yet.
std::vector<ServiceFacility> ExtractServiceFacilitiesFromWorld(const World& world);

} // namespace isocity
