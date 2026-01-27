#include "isocity/Livability.hpp"

#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout << "proc_isocity_livability\n";
  std::cout << "\n";
  std::cout << "Compute a composite per-tile livability index and population-weighted intervention priority.\n";
  std::cout << "\n";
  std::cout << "Usage:\n";
  std::cout << "  proc_isocity_livability <save.bin> [--json out.json] [--render-dir out_dir] [--scale N]\n";
  std::cout << "                       [--w-services F] [--w-walk F] [--w-air F] [--w-quiet F] [--w-heat F]\n";
  std::cout << "                       [--hazard-exp F] [--occ-scale N] [--occ-exp F] [--need-exp F]\n";
  std::cout << "                       [--quiet] [--verify-crc]\n";
  std::cout << "\n";
  std::cout << "Notes:\n";
  std::cout << "  - Weights are normalized automatically.\n";
  std::cout << "  - Hazard comfort is computed as pow(1 - hazard01, hazard-exp).\n";
  std::cout << "  - Priority is computed as pow(1 - livability, need-exp) * pow(pop01, occ-exp),\n";
  std::cout << "    where pop01 = clamp(occupants / occ-scale, 0..1).\n";
}

static bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

static bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  if (v < -1.0e9 || v > 1.0e9) return false;
  *out = static_cast<float>(v);
  return true;
}

static void PrintSummary(const LivabilityResult& r)
{
  auto pct = [](double v01) -> double { return v01 * 100.0; };

  std::cout << "Livability summary\n";
  std::cout << "- map: " << r.w << " x " << r.h << " tiles\n";
  std::cout << "- max livability: " << pct(r.maxLivability01) << "%\n";
  std::cout << "- max priority: " << pct(r.maxPriority01) << "%\n";
  std::cout << "- resident population: " << r.residentPopulation << " (tiles: " << r.residentTileCount << ")\n";
  if (r.residentPopulation > 0) {
    std::cout << "- resident mean livability: " << pct(r.residentMeanLivability01) << "%\n";
    std::cout << "  - services: " << pct(r.residentMeanServices01) << "%\n";
    std::cout << "  - walkability: " << pct(r.residentMeanWalkability01) << "%\n";
    std::cout << "  - clean air: " << pct(r.residentMeanCleanAir01) << "%\n";
    std::cout << "  - quiet: " << pct(r.residentMeanQuiet01) << "%\n";
    std::cout << "  - thermal comfort: " << pct(r.residentMeanThermalComfort01) << "%\n";
    std::cout << "- resident livability percentiles: p10=" << pct(r.residentP10) << "%, p50=" << pct(r.residentMedian)
              << "%, p90=" << pct(r.residentP90) << "%\n";
    std::cout << "- resident livability gini: " << r.residentGini << "\n";
  }
}

static bool WriteReportJson(const std::string& outPath, const std::string& inPath, const SaveSummary* sum,
                            const World& world, const LivabilityResult& r)
{
  JsonValue root = JsonValue::MakeObject();

  auto add = [](JsonValue& obj, const char* key, JsonValue v) { obj.objectValue.emplace_back(key, std::move(v)); };

  add(root, "file", JsonValue::MakeString(inPath));
  add(root, "width", JsonValue::MakeNumber(static_cast<double>(world.width())));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(world.height())));

  if (sum) {
    add(root, "version", JsonValue::MakeNumber(static_cast<double>(sum->version)));
    add(root, "seed", JsonValue::MakeNumber(static_cast<double>(sum->seed)));
  }

  // Config / weights.
  {
    JsonValue cfg = JsonValue::MakeObject();
    add(cfg, "weightServices", JsonValue::MakeNumber(static_cast<double>(r.cfg.weightServices)));
    add(cfg, "weightWalkability", JsonValue::MakeNumber(static_cast<double>(r.cfg.weightWalkability)));
    add(cfg, "weightCleanAir", JsonValue::MakeNumber(static_cast<double>(r.cfg.weightCleanAir)));
    add(cfg, "weightQuiet", JsonValue::MakeNumber(static_cast<double>(r.cfg.weightQuiet)));
    add(cfg, "weightThermalComfort", JsonValue::MakeNumber(static_cast<double>(r.cfg.weightThermalComfort)));

    add(cfg, "requireOutsideConnection", JsonValue::MakeBool(r.cfg.requireOutsideConnection));
    add(cfg, "weightMode", JsonValue::MakeString(r.cfg.weightMode == IsochroneWeightMode::Steps ? "steps" : "time"));

    add(cfg, "servicesCatchmentRadiusSteps", JsonValue::MakeNumber(static_cast<double>(r.cfg.servicesCatchmentRadiusSteps)));
    add(cfg, "walkCoverageThresholdSteps", JsonValue::MakeNumber(static_cast<double>(r.cfg.walkCoverageThresholdSteps)));

    add(cfg, "hazardComfortExponent", JsonValue::MakeNumber(static_cast<double>(r.cfg.hazardComfortExponent)));

    add(cfg, "priorityOccupantScale", JsonValue::MakeNumber(static_cast<double>(r.cfg.priorityOccupantScale)));
    add(cfg, "priorityOccupantExponent", JsonValue::MakeNumber(static_cast<double>(r.cfg.priorityOccupantExponent)));
    add(cfg, "priorityNeedExponent", JsonValue::MakeNumber(static_cast<double>(r.cfg.priorityNeedExponent)));

    add(root, "config", std::move(cfg));
  }

  // Summary.
  {
    JsonValue s = JsonValue::MakeObject();
    add(s, "maxLivability01", JsonValue::MakeNumber(static_cast<double>(r.maxLivability01)));
    add(s, "maxPriority01", JsonValue::MakeNumber(static_cast<double>(r.maxPriority01)));

    add(s, "residentPopulation", JsonValue::MakeNumber(static_cast<double>(r.residentPopulation)));
    add(s, "residentTileCount", JsonValue::MakeNumber(static_cast<double>(r.residentTileCount)));

    add(s, "residentMeanLivability01", JsonValue::MakeNumber(static_cast<double>(r.residentMeanLivability01)));
    add(s, "residentMeanServices01", JsonValue::MakeNumber(static_cast<double>(r.residentMeanServices01)));
    add(s, "residentMeanWalkability01", JsonValue::MakeNumber(static_cast<double>(r.residentMeanWalkability01)));
    add(s, "residentMeanCleanAir01", JsonValue::MakeNumber(static_cast<double>(r.residentMeanCleanAir01)));
    add(s, "residentMeanQuiet01", JsonValue::MakeNumber(static_cast<double>(r.residentMeanQuiet01)));
    add(s, "residentMeanThermalComfort01", JsonValue::MakeNumber(static_cast<double>(r.residentMeanThermalComfort01)));

    add(s, "residentP10", JsonValue::MakeNumber(static_cast<double>(r.residentP10)));
    add(s, "residentMedian", JsonValue::MakeNumber(static_cast<double>(r.residentMedian)));
    add(s, "residentP90", JsonValue::MakeNumber(static_cast<double>(r.residentP90)));
    add(s, "residentGini", JsonValue::MakeNumber(static_cast<double>(r.residentGini)));

    add(root, "summary", std::move(s));
  }

  std::string err;
  return WriteJsonFile(outPath, root, err, JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false});
}

} // namespace

int main(int argc, char** argv)
{
  std::string inPath;
  std::string outJson;
  std::string renderDir;
  int scale = 2;
  bool quiet = false;
  bool verifyCrc = false;

  LivabilityConfig cfg{};

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }
    if (arg == "--quiet") {
      quiet = true;
      continue;
    }
    if (arg == "--verify-crc") {
      verifyCrc = true;
      continue;
    }
    if (arg == "--json" && i + 1 < argc) {
      outJson = argv[++i];
      continue;
    }
    if (arg == "--render-dir" && i + 1 < argc) {
      renderDir = argv[++i];
      continue;
    }
    if (arg == "--scale" && i + 1 < argc) {
      if (!ParseI32(argv[++i], &scale) || scale < 1) {
        std::cerr << "Invalid --scale\n";
        return 2;
      }
      continue;
    }

    if (arg == "--w-services" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.weightServices)) {
        std::cerr << "Invalid --w-services\n";
        return 2;
      }
      continue;
    }
    if (arg == "--w-walk" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.weightWalkability)) {
        std::cerr << "Invalid --w-walk\n";
        return 2;
      }
      continue;
    }
    if (arg == "--w-air" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.weightCleanAir)) {
        std::cerr << "Invalid --w-air\n";
        return 2;
      }
      continue;
    }
    if (arg == "--w-quiet" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.weightQuiet)) {
        std::cerr << "Invalid --w-quiet\n";
        return 2;
      }
      continue;
    }
    if (arg == "--w-heat" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.weightThermalComfort)) {
        std::cerr << "Invalid --w-heat\n";
        return 2;
      }
      continue;
    }

    if (arg == "--hazard-exp" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.hazardComfortExponent)) {
        std::cerr << "Invalid --hazard-exp\n";
        return 2;
      }
      continue;
    }
    if (arg == "--occ-scale" && i + 1 < argc) {
      if (!ParseI32(argv[++i], &cfg.priorityOccupantScale) || cfg.priorityOccupantScale <= 0) {
        std::cerr << "Invalid --occ-scale\n";
        return 2;
      }
      continue;
    }
    if (arg == "--occ-exp" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.priorityOccupantExponent)) {
        std::cerr << "Invalid --occ-exp\n";
        return 2;
      }
      continue;
    }
    if (arg == "--need-exp" && i + 1 < argc) {
      if (!ParseF32(argv[++i], &cfg.priorityNeedExponent)) {
        std::cerr << "Invalid --need-exp\n";
        return 2;
      }
      continue;
    }

    if (!arg.empty() && arg[0] != '-') {
      inPath = arg;
      continue;
    }

    std::cerr << "Unknown arg: " << arg << "\n";
    PrintHelp();
    return 2;
  }

  if (inPath.empty()) {
    PrintHelp();
    return 2;
  }

  SaveSummary summary{};
  SaveSummary* summaryPtr = nullptr;
  std::string err;

  if (verifyCrc) {
    if (!ReadSaveSummary(inPath, summary, err, /*verifyCrc=*/true)) {
      std::cerr << "Failed to read save summary: " << err << "\n";
      return 1;
    }
    if (summary.crcChecked && !summary.crcOk) {
      std::cerr << "CRC check failed: save appears corrupted\n";
      return 1;
    }
    summaryPtr = &summary;
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;
  if (!LoadWorldBinary(world, procCfg, simCfg, inPath, err)) {
    std::cerr << "Failed to load save: " << err << "\n";
    return 1;
  }

  cfg.requireOutsideConnection = simCfg.requireOutsideConnection;

  // Optional road-to-edge mask for traffic/goods.
  std::vector<std::uint8_t> roadToEdge;
  const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (simCfg.requireOutsideConnection) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
    roadToEdgePtr = &roadToEdge;
  }

  TrafficConfig tc{};
  tc.requireOutsideConnection = simCfg.requireOutsideConnection;
  const TrafficResult trafficRes = ComputeCommuteTraffic(world, tc, /*employedShare=*/1.0f, roadToEdgePtr);

  GoodsConfig gc{};
  gc.requireOutsideConnection = simCfg.requireOutsideConnection;
  gc.allowImports = true;
  gc.allowExports = true;
  gc.importCapacityPct = std::clamp(world.stats().tradeImportCapacityPct, 0, 100);
  gc.exportCapacityPct = std::clamp(world.stats().tradeExportCapacityPct, 0, 100);
  const GoodsResult goodsRes = ComputeGoodsFlow(world, gc, roadToEdgePtr);

  const LivabilityResult res = ComputeLivability(world, cfg, &trafficRes, &goodsRes);

  if (!quiet) {
    PrintSummary(res);
  }

  if (!outJson.empty()) {
    if (!WriteReportJson(outJson, inPath, summaryPtr, world, res)) {
      std::cerr << "Failed to write JSON report\n";
      return 1;
    }
  }

  if (!renderDir.empty()) {
    std::error_code ec;
    fs::create_directories(renderDir, ec);
    if (ec) {
      std::cerr << "Failed to create render directory\n";
      return 1;
    }

    struct L {
      ExportLayer layer;
      const char* name;
    } layers[] = {
        {ExportLayer::Livability, "map_livability.ppm"},
        {ExportLayer::InterventionPriority, "map_intervention_priority.ppm"},
    };

    for (const L& l : layers) {
      PpmImage img = RenderPpmLayer(world, l.layer, nullptr, &trafficRes, &goodsRes);
      if (scale > 1) img = ScaleNearest(img, scale);

      std::string ioErr;
      const fs::path outPath = fs::path(renderDir) / l.name;
      if (!WritePpm(outPath.string(), img, ioErr)) {
        std::cerr << "Failed to write " << outPath.string() << ": " << ioErr << "\n";
        return 1;
      }
    }
  }

  return 0;
}
