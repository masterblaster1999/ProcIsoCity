#pragma once

#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// A lightweight, deterministic macro-economy layer.
//
// This is intentionally a *derived* model: it does not store mutable state.
// Callers compute a snapshot for a given day and then feed the resulting
// multipliers into other systems (goods flow, taxes, happiness).

enum class EconomySectorKind : std::uint8_t {
  Agriculture = 0,
  Manufacturing = 1,
  Logistics = 2,
  Energy = 3,
  Tech = 4,
  Tourism = 5,
  Finance = 6,
  Construction = 7,
};

inline const char* EconomySectorKindName(EconomySectorKind k)
{
  switch (k) {
    case EconomySectorKind::Agriculture:    return "agriculture";
    case EconomySectorKind::Manufacturing: return "manufacturing";
    case EconomySectorKind::Logistics:     return "logistics";
    case EconomySectorKind::Energy:        return "energy";
    case EconomySectorKind::Tech:          return "tech";
    case EconomySectorKind::Tourism:       return "tourism";
    case EconomySectorKind::Finance:       return "finance";
    case EconomySectorKind::Construction:  return "construction";
  }
  return "tech";
}

enum class EconomyEventKind : std::uint8_t {
  None = 0,
  Recession = 1,
  FuelSpike = 2,
  ImportShock = 3,
  ExportBoom = 4,
  TechBoom = 5,
  TourismSurge = 6,
};

inline const char* EconomyEventKindName(EconomyEventKind k)
{
  switch (k) {
    case EconomyEventKind::None:         return "none";
    case EconomyEventKind::Recession:    return "recession";
    case EconomyEventKind::FuelSpike:    return "fuel_spike";
    case EconomyEventKind::ImportShock:  return "import_shock";
    case EconomyEventKind::ExportBoom:   return "export_boom";
    case EconomyEventKind::TechBoom:     return "tech_boom";
    case EconomyEventKind::TourismSurge: return "tourism_surge";
  }
  return "none";
}

struct EconomySector {
  EconomySectorKind kind = EconomySectorKind::Tech;
  std::string name;

  // Affinities used to modulate district multipliers.
  // 0..1, where higher values increase the corresponding zone multipliers.
  float industrialAffinity = 0.5f;
  float commercialAffinity = 0.5f;

  // How sensitive the sector is to shocks/volatility.
  // 0..1 (higher => more swing during events).
  float volatility = 0.5f;
};

struct EconomyEvent {
  EconomyEventKind kind = EconomyEventKind::None;
  int startDay = -1;
  int durationDays = 0;
  float severity = 0.0f; // 0..1
};

struct DistrictEconomyProfile {
  int dominantSector = -1; // index into EconomySnapshot::sectors

  // 0..1-ish latent indices.
  float wealth = 0.5f;
  float productivity = 0.5f;

  // Multipliers consumed by other systems.
  float taxBaseMult = 1.0f;
  float industrialSupplyMult = 1.0f;
  float commercialDemandMult = 1.0f;
};

struct EconomySnapshot {
  int day = 0;

  // Macro state (roughly ~1.0 baseline).
  float economyIndex = 1.0f;

  // Inflation / volatility proxy (0..~0.15 typical). Used as a happiness friction term.
  float inflation = 0.0f;

  // Aggregate wealth proxy for UI/debug.
  float cityWealth = 0.5f;

  EconomyEvent activeEvent{};
  int activeEventDaysLeft = 0;

  std::vector<EconomySector> sectors;
  std::array<DistrictEconomyProfile, static_cast<std::size_t>(kDistrictCount)> districts{};
};

// Non-persistent runtime tuning for the macro economy.
//
// This is intentionally not part of SimConfig so the model can evolve without
// forcing save-version bumps.
struct EconomyModelSettings {
  bool enabled = false;

  // Optional seed salt to allow alternative economies for the same world seed.
  std::uint64_t seedSalt = 0;

  // Number of economic sectors to generate (>=1).
  int sectorCount = 6;

  // Macro cycle period (days). Typical: 20..60.
  float macroPeriodDays = 28.0f;

  // Event generation parameters.
  int minEventDurationDays = 3;
  int maxEventDurationDays = 8;

  // How far back to scan for a deterministic event start.
  int eventScanbackDays = 16;

  bool operator==(const EconomyModelSettings& o) const
  {
    return enabled == o.enabled && seedSalt == o.seedSalt && sectorCount == o.sectorCount &&
           macroPeriodDays == o.macroPeriodDays && minEventDurationDays == o.minEventDurationDays &&
           maxEventDurationDays == o.maxEventDurationDays && eventScanbackDays == o.eventScanbackDays;
  }

  bool operator!=(const EconomyModelSettings& o) const { return !(*this == o); }
};

// Compute the deterministic economy snapshot for the given day.
EconomySnapshot ComputeEconomySnapshot(const World& world, int day, const EconomyModelSettings& settings);

} // namespace isocity
