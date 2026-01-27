#include "isocity/RunoffPollution.hpp"

#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
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
  std::cout << "proc_isocity_runoff\n";
  std::cout << "\n";
  std::cout << "Compute a simple downhill runoff / stormwater pollution proxy (sources + routing + filtration).\n";
  std::cout << "\n";
  std::cout << "Usage:\n";
  std::cout << "  proc_isocity_runoff <save.bin> [--json out.json] [--render-dir out_dir] [--scale N]\n";
  std::cout << "                     [--no-traffic] [--quiet] [--verify-crc]\n";
  std::cout << "\n";
  std::cout << "Notes:\n";
  std::cout << "  - The model is deterministic and uses a D4 downhill flow-direction field.\n";
  std::cout << "  - --render-dir writes PPM maps for runoff_pollution and runoff_load layers.\n";
}

static bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  char* end = nullptr;
  errno = 0;
  long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0 || end == s.c_str() || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

static void PrintSummary(const RunoffPollutionResult& r)
{
  std::cout << "Runoff / stormwater pollution (heuristic)\n";
  std::cout << "  maxConcentration: " << r.maxConcentration << "\n";
  std::cout << "  maxFlowAccum:     " << r.maxFlowAccum << "\n";
  std::cout << "  residentPop:      " << r.residentPopulation << "\n";
  std::cout << "  residentTiles:    " << r.residentialTileCount << "\n";
  std::cout << "  residentAvg01:    " << r.residentAvgPollution01 << "\n";
  std::cout << "  residentHighFrac: " << r.residentHighExposureFrac << "\n";
}

static bool WriteReportJson(const std::string& outPath, const std::string& inPath, const SaveSummary* sum,
                            const World& world, const RunoffPollutionResult& r)
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

  // Config.
  {
    JsonValue cfg = JsonValue::MakeObject();
    add(cfg, "roadBase", JsonValue::MakeNumber(static_cast<double>(r.cfg.roadBase)));
    add(cfg, "roadClassBoost", JsonValue::MakeNumber(static_cast<double>(r.cfg.roadClassBoost)));
    add(cfg, "roadTrafficBoost", JsonValue::MakeNumber(static_cast<double>(r.cfg.roadTrafficBoost)));

    add(cfg, "residentialLoad", JsonValue::MakeNumber(static_cast<double>(r.cfg.residentialLoad)));
    add(cfg, "commercialLoad", JsonValue::MakeNumber(static_cast<double>(r.cfg.commercialLoad)));
    add(cfg, "industrialLoad", JsonValue::MakeNumber(static_cast<double>(r.cfg.industrialLoad)));
    add(cfg, "civicLoad", JsonValue::MakeNumber(static_cast<double>(r.cfg.civicLoad)));

    add(cfg, "occupantBoost", JsonValue::MakeNumber(static_cast<double>(r.cfg.occupantBoost)));
    add(cfg, "occupantScale", JsonValue::MakeNumber(static_cast<double>(r.cfg.occupantScale)));

    add(cfg, "filterPark", JsonValue::MakeNumber(static_cast<double>(r.cfg.filterPark)));
    add(cfg, "filterGrass", JsonValue::MakeNumber(static_cast<double>(r.cfg.filterGrass)));
    add(cfg, "filterSand", JsonValue::MakeNumber(static_cast<double>(r.cfg.filterSand)));
    add(cfg, "filterRoad", JsonValue::MakeNumber(static_cast<double>(r.cfg.filterRoad)));

    add(cfg, "waterIsSink", JsonValue::MakeBool(r.cfg.waterIsSink));
    add(cfg, "filterWater", JsonValue::MakeNumber(static_cast<double>(r.cfg.filterWater)));

    add(cfg, "dilutionExponent", JsonValue::MakeNumber(static_cast<double>(r.cfg.dilutionExponent)));
    add(cfg, "clampLoad", JsonValue::MakeNumber(static_cast<double>(r.cfg.clampLoad)));
    add(cfg, "fallbackCommuteTraffic01", JsonValue::MakeNumber(static_cast<double>(r.cfg.fallbackCommuteTraffic01)));
    add(cfg, "highExposureThreshold01", JsonValue::MakeNumber(static_cast<double>(r.cfg.highExposureThreshold01)));

    add(root, "config", std::move(cfg));
  }

  // Summary.
  {
    JsonValue s = JsonValue::MakeObject();
    add(s, "maxLocalLoad", JsonValue::MakeNumber(static_cast<double>(r.maxLocalLoad)));
    add(s, "maxConcentration", JsonValue::MakeNumber(static_cast<double>(r.maxConcentration)));
    add(s, "maxFlowAccum", JsonValue::MakeNumber(static_cast<double>(r.maxFlowAccum)));

    add(s, "residentPopulation", JsonValue::MakeNumber(static_cast<double>(r.residentPopulation)));
    add(s, "residentialTileCount", JsonValue::MakeNumber(static_cast<double>(r.residentialTileCount)));
    add(s, "residentAvgPollution01", JsonValue::MakeNumber(static_cast<double>(r.residentAvgPollution01)));
    add(s, "residentHighExposureFrac", JsonValue::MakeNumber(static_cast<double>(r.residentHighExposureFrac)));

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
  bool useTraffic = true;

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
    if (arg == "--no-traffic") {
      useTraffic = false;
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

    // First non-flag is the input path.
    if (!arg.empty() && arg[0] != '-') {
      inPath = arg;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    return 2;
  }

  if (inPath.empty()) {
    PrintHelp();
    return 2;
  }

  std::string err;
  SaveSummary summary;
  const SaveSummary* summaryPtr = nullptr;
  if (verifyCrc || !outJson.empty()) {
    if (!ReadSaveSummary(inPath, summary, err, verifyCrc)) {
      std::cerr << "Failed to read save summary: " << err << "\n";
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

  // Optional road-to-edge mask for traffic.
  std::vector<std::uint8_t> roadToEdge;
  const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (simCfg.requireOutsideConnection) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
    roadToEdgePtr = &roadToEdge;
  }

  TrafficResult trafficRes;
  const TrafficResult* trafficPtr = nullptr;
  if (useTraffic) {
    TrafficConfig tc{};
    tc.requireOutsideConnection = simCfg.requireOutsideConnection;
    trafficRes = ComputeCommuteTraffic(world, tc, /*employedShare=*/1.0f, roadToEdgePtr);
    trafficPtr = &trafficRes;
  }

  RunoffPollutionConfig cfg{};
  const RunoffPollutionResult res = ComputeRunoffPollution(world, cfg, trafficPtr);

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
        {ExportLayer::RunoffPollution, "map_runoff_pollution.ppm"},
        {ExportLayer::RunoffPollutionLoad, "map_runoff_load.ppm"},
    };

    for (const L& l : layers) {
      PpmImage img = RenderPpmLayer(world, l.layer, nullptr, trafficPtr, nullptr);
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
