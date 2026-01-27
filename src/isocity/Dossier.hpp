#pragma once

#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// City dossier exporter
//
// A dossier is a self-contained export bundle for a city/world:
//   - map layers (PNG/PPM)
//   - tile_metrics.csv (per-tile derived metrics)
//   - ticks.csv (time-series Stats)
//   - summary.json (metadata + serialized ProcGen/Sim configs)
//   - world.bin (save file)
//   - index.html (portable viewer)
//
// The headless CLI tool `proc_isocity_dossier` uses the same exporter.
// The interactive game can also call this to export the *current* world.
// -----------------------------------------------------------------------------

struct CityDossierProgress {
  int stepIndex = 0;
  int stepCount = 0;
  std::string stage;
};

using CityDossierProgressFn = std::function<bool(const CityDossierProgress&)>;

struct CityDossierConfig {
  // Output directory (required).
  std::filesystem::path outDir;

  // Image format extension used for exports. Recommended: "png".
  // "ppm" is also supported (fast + dependency-free).
  std::string format = "png";

  // Nearest-neighbor upscale for top-down exports.
  int exportScale = 2;

  // Which top-down layers to export.
  std::vector<ExportLayer> layers2d = {
      ExportLayer::Terrain,
      ExportLayer::Overlay,
      ExportLayer::Height,
      ExportLayer::LandValue,
      ExportLayer::Traffic,
      ExportLayer::RoadCentrality,
      ExportLayer::RoadVulnerability,
      ExportLayer::RoadBypass,
      ExportLayer::GoodsTraffic,
      ExportLayer::GoodsFill,
      ExportLayer::District,
      ExportLayer::FloodDepth,
      ExportLayer::PondingDepth,
      ExportLayer::Noise,
      ExportLayer::LandUseMix,
      ExportLayer::HeatIsland,
      ExportLayer::SkyView,
      ExportLayer::CanyonConfinement,
      ExportLayer::SolarExposure,
      ExportLayer::SolarPotential,
      ExportLayer::EnergyDemand,
      ExportLayer::EnergySolar,
      ExportLayer::EnergyBalance,
      ExportLayer::CarbonEmission,
      ExportLayer::CarbonSequestration,
      ExportLayer::CarbonBalance,
      ExportLayer::CrimeRisk,
      ExportLayer::PoliceAccess,

    ExportLayer::TrafficCrashRisk,
    ExportLayer::TrafficCrashExposure,
    ExportLayer::TrafficCrashPriority,
      ExportLayer::TransitAccess,
      ExportLayer::TransitModeSharePotential,
      ExportLayer::AirPollution,
      ExportLayer::AirPollutionEmission,
      ExportLayer::RunoffPollution,
      ExportLayer::RunoffPollutionLoad,
      ExportLayer::RunoffMitigationPriority,
      ExportLayer::RunoffMitigationPlan,
      ExportLayer::FireRisk,
      ExportLayer::Walkability,
      ExportLayer::JobAccess,
      ExportLayer::JobOpportunity,
      ExportLayer::Livability,
      ExportLayer::InterventionPriority,
      ExportLayer::LivabilityHotspot,
      ExportLayer::InterventionHotspot,
  };

  // Optional isometric overview exports.
  bool exportIso = true;
  std::vector<ExportLayer> layersIso = {
      ExportLayer::Overlay,
      ExportLayer::RoadCentrality,
      ExportLayer::RoadVulnerability,
      ExportLayer::RoadBypass,
      ExportLayer::LandValue,
      ExportLayer::HeatIsland,
      ExportLayer::SkyView,
      ExportLayer::CanyonConfinement,
      ExportLayer::SolarExposure,
      ExportLayer::SolarPotential,
      ExportLayer::EnergyDemand,
      ExportLayer::EnergySolar,
      ExportLayer::EnergyBalance,
      ExportLayer::CarbonEmission,
      ExportLayer::CarbonSequestration,
      ExportLayer::CarbonBalance,
      ExportLayer::CrimeRisk,
      ExportLayer::PoliceAccess,
      ExportLayer::TransitAccess,
      ExportLayer::TransitModeSharePotential,
      ExportLayer::AirPollution,
      ExportLayer::AirPollutionEmission,
      ExportLayer::RunoffPollution,
      ExportLayer::RunoffPollutionLoad,
      ExportLayer::RunoffMitigationPriority,
      ExportLayer::RunoffMitigationPlan,
      ExportLayer::FireRisk,
      ExportLayer::Walkability,
      ExportLayer::JobAccess,
      ExportLayer::JobOpportunity,
      ExportLayer::Livability,
      ExportLayer::InterventionPriority,
      ExportLayer::LivabilityHotspot,
      ExportLayer::InterventionHotspot,
  };

  // Optional CPU software 3D render.
  bool export3d = false;

  // When export3d is enabled, this config controls the render.
  bool export3dPreview = true;  // include the 3D render preview in the dossier index
  Render3DConfig render3dCfg{};

  // Data exports.
  bool writeTileMetricsCsv = true;
  bool writeTicksCsv = true;
  bool writeSummaryJson = true;
  bool writeWorldBinary = true;
  bool writeHtml = true;
};

struct CityDossierResult {
  std::filesystem::path outDir;
  std::uint64_t hash = 0;
};

// Export a dossier for the provided world.
//
// Notes:
// - This function may call Simulator::refreshDerivedStats(world) to ensure
//   derived stats are up-to-date for exports.
// - If ticks is empty, ticks.csv is written with a header only and the HTML
//   report uses world.stats() as the latest Stats.
bool WriteCityDossier(World& world,
                      const ProcGenConfig& procCfg,
                      const SimConfig& simCfg,
                      const std::vector<Stats>& ticks,
                      const CityDossierConfig& cfg,
                      CityDossierResult* outRes,
                      std::string& outErr,
                      const CityDossierProgressFn& progress = CityDossierProgressFn());

} // namespace isocity
