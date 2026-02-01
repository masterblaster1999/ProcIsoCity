#include "isocity/AutoBuild.hpp"
#include "isocity/Dossier.hpp"
#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include "cli/CliParse.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

bool EnsureDir(const std::filesystem::path& p)
{
  return isocity::cli::EnsureDir(p);
}

bool ParseI32(const std::string& s, int* out)
{
  return isocity::cli::ParseI32(s, out);
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  return isocity::cli::ParseU64(s, out);
}

bool ParseF32(const std::string& s, float* out)
{
  return isocity::cli::ParseF32(s, out);
}

bool ParseBool01(const std::string& s, bool* out)
{
  return isocity::cli::ParseBool01(s, out);
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  return isocity::cli::ParseWxH(s, outW, outH);
}

std::vector<std::string> SplitCommaList(const std::string& s)
{
  return isocity::cli::SplitCommaList(s);
}

bool ParseLayerList(const std::string& s, std::vector<ExportLayer>* out, std::string& err)
{
  if (!out) {
    err = "internal error: null out";
    return false;
  }
  out->clear();
  const auto parts = SplitCommaList(s);
  if (parts.empty()) {
    err = "empty layer list";
    return false;
  }
  for (const std::string& p : parts) {
    ExportLayer layer{};
    if (!ParseExportLayer(p, layer)) {
      err = "unknown layer: '" + p + "'";
      return false;
    }
    out->push_back(layer);
  }
  return true;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_dossier (one-command city dossier exporter)\n\n"
      << "Generates (or loads) a world, optionally runs the simulator and/or AutoBuild,\n"
      << "then exports a full dossier: top-down layers, optional isometric and 3D renders,\n"
      << "tile_metrics.csv, ticks.csv, summary.json, chronicle.json, and an index.html viewer.\n\n"
      << "Usage:\n"
      << "  proc_isocity_dossier --out-dir <dir> [options]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>              Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                   Seed for generation (default 1).\n"
      << "  --size <WxH>                   World size (default 256x256).\n\n"
      << "ProcGen (when generating):\n"
      << "  --gen-preset <name>            Terrain preset (classic/island/archipelago/...).\n"
      << "  --gen-preset-strength <N>      Preset strength (default 1).\n"
      << "  --gen-road-layout <name>       Road layout (organic/grid/radial/tensor_field/physarum/medial_axis/voronoi_cells/space_colonization).\n"
      << "  --gen-districting <name>       Districting (voronoi/road_flow/block_graph/watershed).\n"
      << "  --gen-hubs <N>                 Hub count (default 4).\n"
      << "  --gen-water-level <0..1>       Water threshold (default 0.35).\n\n"
      << "Simulation:\n"
      << "  --require-outside <0|1>        Require road-to-edge for zones (default 1).\n"
      << "  --autobuild-days <N>           Run AutoBuild for N days (default 0).\n"
      << "  --days <N>                     Simulate N additional days (default 0).\n\n"
      << "Export:\n"
      << "  --format <png|ppm>             Image format (default png).\n"
      << "  --scale <N>                    Nearest-neighbor scale for top-down layers (default 2).\n"
      << "  --layers <a,b,c>               Top-down layers to export (default includes many analytics).\n"
      << "  --iso <0|1>                    Enable isometric exports (default 1).\n"
      << "  --iso-layers <a,b,c>           Iso layers (default includes overlay/landvalue/heat/etc).\n"
      << "  --3d <0|1>                     Enable a 3D overlay render (default 0).\n\n";
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::filesystem::path outDir;

  std::uint64_t seed = 1;
  int w = 256;
  int h = 256;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::optional<bool> requireOutsideOverride;

  int autobuildDays = 0;
  int simDays = 0;

  std::string format = "png";
  int exportScale = 2;

  std::vector<ExportLayer> layers2d = {
      ExportLayer::Terrain,
      ExportLayer::Overlay,
      ExportLayer::Height,
      ExportLayer::LandValue,
      ExportLayer::Traffic,
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
      ExportLayer::TrafficCrashRisk,
      ExportLayer::TrafficCrashExposure,
      ExportLayer::TrafficCrashPriority,
      ExportLayer::RunoffPollution,
      ExportLayer::RunoffPollutionLoad,
      ExportLayer::RunoffMitigationPriority,
      ExportLayer::RunoffMitigationPlan,
  };

  bool exportIso = true;
  std::vector<ExportLayer> layersIso = {
      ExportLayer::Overlay,
      ExportLayer::LandValue,
      ExportLayer::HeatIsland,
      ExportLayer::SkyView,
      ExportLayer::CanyonConfinement,
      ExportLayer::RunoffMitigationPriority,
      ExportLayer::RunoffMitigationPlan,
  };

  bool export3d = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        std::exit(1);
      }
      return std::string(argv[++i]);
    };

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    }
    if (arg == "--load") {
      loadPath = need("--load");
      continue;
    }
    if (arg == "--out-dir") {
      outDir = need("--out-dir");
      continue;
    }
    if (arg == "--seed") {
      const std::string v = need("--seed");
      if (!ParseU64(v, &seed)) {
        std::cerr << "Invalid seed: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--size") {
      const std::string v = need("--size");
      if (!ParseWxH(v, &w, &h)) {
        std::cerr << "Invalid size: " << v << " (expected WxH)\n";
        return 1;
      }
      continue;
    }
    if (arg == "--require-outside") {
      const std::string v = need("--require-outside");
      bool b = true;
      if (!ParseBool01(v, &b)) {
        std::cerr << "Invalid bool: " << v << "\n";
        return 1;
      }
      requireOutsideOverride = b;
      continue;
    }
    if (arg == "--autobuild-days") {
      const std::string v = need("--autobuild-days");
      if (!ParseI32(v, &autobuildDays) || autobuildDays < 0) {
        std::cerr << "Invalid autobuild days: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--days") {
      const std::string v = need("--days");
      if (!ParseI32(v, &simDays) || simDays < 0) {
        std::cerr << "Invalid days: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--format") {
      format = need("--format");
      if (format != "png" && format != "ppm") {
        std::cerr << "Unsupported format: " << format << " (expected png|ppm)\n";
        return 1;
      }
      continue;
    }
    if (arg == "--scale") {
      const std::string v = need("--scale");
      if (!ParseI32(v, &exportScale) || exportScale < 1 || exportScale > 32) {
        std::cerr << "Invalid scale: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--layers") {
      const std::string v = need("--layers");
      std::string err;
      if (!ParseLayerList(v, &layers2d, err)) {
        std::cerr << "Invalid layers: " << err << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--iso") {
      const std::string v = need("--iso");
      bool b = true;
      if (!ParseBool01(v, &b)) {
        std::cerr << "Invalid bool: " << v << "\n";
        return 1;
      }
      exportIso = b;
      continue;
    }
    if (arg == "--iso-layers") {
      const std::string v = need("--iso-layers");
      std::string err;
      if (!ParseLayerList(v, &layersIso, err)) {
        std::cerr << "Invalid iso layers: " << err << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--3d") {
      const std::string v = need("--3d");
      bool b = false;
      if (!ParseBool01(v, &b)) {
        std::cerr << "Invalid bool: " << v << "\n";
        return 1;
      }
      export3d = b;
      continue;
    }

    // ProcGen tuning.
    if (arg == "--gen-preset") {
      const std::string v = need("--gen-preset");
      ProcGenTerrainPreset p{};
      if (!ParseProcGenTerrainPreset(v, p)) {
        std::cerr << "Invalid terrain preset: " << v << "\n";
        return 1;
      }
      procCfg.terrainPreset = p;
      continue;
    }
    if (arg == "--gen-preset-strength") {
      const std::string v = need("--gen-preset-strength");
      float f = 1.0f;
      if (!ParseF32(v, &f) || f < 0.0f || f > 10.0f) {
        std::cerr << "Invalid preset strength: " << v << "\n";
        return 1;
      }
      procCfg.terrainPresetStrength = f;
      continue;
    }
    if (arg == "--gen-road-layout") {
      const std::string v = need("--gen-road-layout");
      ProcGenRoadLayout r{};
      if (!ParseProcGenRoadLayout(v, r)) {
        std::cerr << "Invalid road layout: " << v << "\n";
        return 1;
      }
      procCfg.roadLayout = r;
      continue;
    }
    if (arg == "--gen-districting") {
      const std::string v = need("--gen-districting");
      ProcGenDistrictingMode m{};
      if (!ParseProcGenDistrictingMode(v, m)) {
        std::cerr << "Invalid districting mode: " << v << "\n";
        return 1;
      }
      procCfg.districtingMode = m;
      continue;
    }
    if (arg == "--gen-hubs") {
      const std::string v = need("--gen-hubs");
      int iv = 0;
      if (!ParseI32(v, &iv) || iv < 1 || iv > 64) {
        std::cerr << "Invalid hubs: " << v << "\n";
        return 1;
      }
      procCfg.hubs = iv;
      continue;
    }
    if (arg == "--gen-water-level") {
      const std::string v = need("--gen-water-level");
      float f = 0.0f;
      if (!ParseF32(v, &f) || f < 0.0f || f > 1.0f) {
        std::cerr << "Invalid water level: " << v << "\n";
        return 1;
      }
      procCfg.waterLevel = f;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << " (try --help)\n";
    return 1;
  }

  if (outDir.empty()) {
    std::cerr << "Missing required --out-dir\n";
    return 1;
  }
  if (!EnsureDir(outDir)) {
    std::cerr << "Failed to create output dir: " << outDir.string() << "\n";
    return 1;
  }

  // Load or generate.
  World world;
  {
    std::string err;
    if (!loadPath.empty()) {
      if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
        std::cerr << "Failed to load save: " << loadPath << " (" << err << ")\n";
        return 1;
      }
    } else {
      world = GenerateWorld(w, h, seed, procCfg);
    }
  }

  // Apply runtime SimConfig overrides (regardless of load vs generate).
  if (requireOutsideOverride.has_value()) {
    simCfg.requireOutsideConnection = *requireOutsideOverride;
  }

  // Simulation / AutoBuild.
  Simulator sim(simCfg);
  sim.refreshDerivedStats(world);

  std::vector<Stats> ticks;
  ticks.push_back(world.stats());

  if (autobuildDays > 0) {
    AutoBuildConfig acfg{};
    acfg.respectOutsideConnection = true;
    acfg.ensureOutsideConnection = true;
    RunAutoBuild(world, sim, acfg, autobuildDays, &ticks);
  }
  for (int d = 0; d < simDays; ++d) {
    sim.stepOnce(world);
    ticks.push_back(world.stats());
  }

  CityDossierConfig cfg;
  cfg.outDir = outDir;
  cfg.format = format;
  cfg.exportScale = exportScale;
  cfg.layers2d = layers2d;
  cfg.exportIso = exportIso;
  cfg.layersIso = layersIso;
  cfg.export3d = export3d;

  CityDossierResult res;
  std::string err;
  if (!WriteCityDossier(world, procCfg, sim.config(), ticks, cfg, &res, err)) {
    std::cerr << "Failed to write dossier: " << err << "\n";
    return 1;
  }

  std::cout << "Wrote dossier: " << res.outDir.string() << "\n";
  return 0;
}
