#include "isocity/AutoBuild.hpp"
#include "isocity/Chronicle.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include "cli/CliParse.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

using namespace isocity;

bool ParseI32(const std::string& s, int* out)
{
  return isocity::cli::ParseI32(s, out);
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  return isocity::cli::ParseU64(s, out);
}

bool ParseBool01(const std::string& s, bool* out)
{
  return isocity::cli::ParseBool01(s, out);
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  return isocity::cli::ParseWxH(s, outW, outH);
}

std::filesystem::path DefaultMarkdownPath(const std::filesystem::path& jsonPath)
{
  std::filesystem::path md = jsonPath;
  if (md.extension() == ".json") {
    md.replace_extension(".md");
  } else {
    md += ".md";
  }
  return md;
}

bool EnsureParentDir(const std::filesystem::path& p)
{
  return isocity::cli::EnsureParentDir(p);
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_chronicle (procedural city newspaper)\n\n"
      << "Generates chronicle.json / chronicle.md from per-day Stats snapshots.\n"
      << "You can load an existing save.bin, optionally run AutoBuild and/or\n"
      << "simulate forward, then write a deterministic daily headline feed.\n\n"
      << "Usage:\n"
      << "  proc_isocity_chronicle [--load save.bin | --seed N --size WxH] [options]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>              Load an existing save (recommended).\n"
      << "  --seed <u64>                   Seed for generation (default 1).\n"
      << "  --size <WxH>                   World size (default 256x256).\n\n"
      << "ProcGen (when generating):\n"
      << "  --gen-preset <name>            Terrain preset.\n"
      << "  --gen-road-layout <name>       Road layout.\n"
      << "  --gen-districting <name>       Districting mode.\n\n"
      << "Simulation:\n"
      << "  --require-outside <0|1>        Override outside connection rule (default: from save or 1).\n"
      << "  --autobuild-days <N>           Run AutoBuild for N days (default 0).\n"
      << "  --days <N>                     Simulate N additional days (default 0).\n\n"
      << "Output:\n"
      << "  --out <chronicle.json>         Output JSON path (default chronicle.json).\n"
      << "  --md <chronicle.md>            Output Markdown path (default next to --out).\n"
      << "  --no-md                        Disable Markdown output.\n\n";
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 256;
  int h = 256;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::optional<bool> requireOutsideOverride;

  int autobuildDays = 0;
  int simDays = 0;

  std::filesystem::path outJson = "chronicle.json";
  std::filesystem::path outMd;
  bool writeMd = true;

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

    // Simulation.
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

    // Output.
    if (arg == "--out") {
      outJson = need("--out");
      continue;
    }
    if (arg == "--md") {
      outMd = need("--md");
      continue;
    }
    if (arg == "--no-md") {
      writeMd = false;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << " (try --help)\n";
    return 1;
  }

  if (outMd.empty() && writeMd) outMd = DefaultMarkdownPath(outJson);

  if (!EnsureParentDir(outJson)) {
    std::cerr << "Failed to create parent directory for: " << outJson.string() << "\n";
    return 1;
  }
  if (writeMd && !EnsureParentDir(outMd)) {
    std::cerr << "Failed to create parent directory for: " << outMd.string() << "\n";
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
      // If the caller didn't override requireOutside, keep the default behavior.
      if (!requireOutsideOverride.has_value()) {
        requireOutsideOverride = true;
      }
    }
  }

  if (requireOutsideOverride.has_value()) {
    simCfg.requireOutsideConnection = *requireOutsideOverride;
  }

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

  const Chronicle chron = GenerateCityChronicle(world, ticks);

  {
    std::string err;
    if (!WriteCityChronicleJson(outJson.string(), chron, err)) {
      std::cerr << "Failed to write JSON: " << err << "\n";
      return 1;
    }
  }

  if (writeMd) {
    std::string err;
    if (!WriteCityChronicleMarkdown(outMd.string(), chron, err)) {
      std::cerr << "Failed to write Markdown: " << err << "\n";
      return 1;
    }
  }

  std::cout << "Wrote " << outJson.string();
  if (writeMd) std::cout << " and " << outMd.string();
  std::cout << "\n";
  return 0;
}
