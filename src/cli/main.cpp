#include "isocity/Hash.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Export.hpp"
#include "isocity/Pathfinding.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  int base = 10;
  std::size_t offset = 0;
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    base = 16;
    offset = 2;
  }

  char* end = nullptr;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string::npos) return false;

  const std::string a = s.substr(0, pos);
  const std::string b = s.substr(pos + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_cli (headless simulation runner)\n\n"
      << "Usage:\n"
      << "  proc_isocity_cli [--load <save.bin>] [--seed <u64>] [--size <WxH>] [--days <N>]\n"
      << "                 [--out <summary.json>] [--csv <ticks.csv>] [--save <save.bin>]\n"
      << "                 [--require-outside <0|1>] [--tax-res <N>] [--tax-com <N>] [--tax-ind <N>]\n"
      << "                 [--maint-road <N>] [--maint-park <N>]\n"
      << "                 [--export-ppm <layer> <out.ppm>]... [--export-scale <N>] [--export-tiles-csv <tiles.csv>]\n"
      << "                 [--batch <N>]\n\n"
      << "Export layers (for --export-ppm):\n"
      << "  terrain overlay height landvalue traffic goods_traffic goods_fill district\n\n"
      << "Batch mode:\n"
      << "  - --batch N>1 runs N simulations with seeds (seed, seed+1, ...).\n"
      << "  - To write per-run files, include {seed} or {run} in any output path.\n"
      << "    Example: --out out_{seed}.json  --export-ppm overlay map_{seed}.ppm\n\n"
      << "Notes:\n"
      << "  - If --load is provided, the world + ProcGenConfig + SimConfig are loaded from the save.\n"
      << "  - Otherwise, a new world is generated from (--seed, --size).\n"
      << "  - --days advances the simulator by N ticks via Simulator::stepOnce().\n"
      << "  - A stable 64-bit hash of the final world is included in the JSON output.\n";
}

bool WriteJsonSummary(const isocity::World& world, std::uint64_t hash, const std::string& outPath)
{
  const isocity::Stats& s = world.stats();

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"width\": " << world.width() << ",\n";
  oss << "  \"height\": " << world.height() << ",\n";
  oss << "  \"seed\": " << world.seed() << ",\n";
  oss << "  \"hash\": \"" << HexU64(hash) << "\",\n";
  oss << "  \"stats\": {\n";
  oss << "    \"day\": " << s.day << ",\n";
  oss << "    \"population\": " << s.population << ",\n";
  oss << "    \"housingCapacity\": " << s.housingCapacity << ",\n";
  oss << "    \"jobsCapacity\": " << s.jobsCapacity << ",\n";
  oss << "    \"jobsCapacityAccessible\": " << s.jobsCapacityAccessible << ",\n";
  oss << "    \"employed\": " << s.employed << ",\n";
  oss << "    \"happiness\": " << s.happiness << ",\n";
  oss << "    \"money\": " << s.money << ",\n";
  oss << "    \"roads\": " << s.roads << ",\n";
  oss << "    \"parks\": " << s.parks << ",\n";
  oss << "    \"avgCommuteTime\": " << s.avgCommuteTime << ",\n";
  oss << "    \"trafficCongestion\": " << s.trafficCongestion << ",\n";
  oss << "    \"goodsDemand\": " << s.goodsDemand << ",\n";
  oss << "    \"goodsDelivered\": " << s.goodsDelivered << ",\n";
  oss << "    \"goodsSatisfaction\": " << s.goodsSatisfaction << ",\n";
  oss << "    \"avgLandValue\": " << s.avgLandValue << ",\n";
  oss << "    \"demandResidential\": " << s.demandResidential << "\n";
  oss << "  }\n";
  oss << "}\n";

  if (outPath.empty()) {
    std::cout << oss.str();
    return true;
  }

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

bool WriteCsvRow(std::ostream& os, const isocity::Stats& s)
{
  os << s.day << ','
     << s.population << ','
     << s.money << ','
     << s.housingCapacity << ','
     << s.jobsCapacity << ','
     << s.jobsCapacityAccessible << ','
     << s.employed << ','
     << s.happiness << ','
     << s.roads << ','
     << s.parks << ','
     << s.avgCommuteTime << ','
     << s.trafficCongestion << ','
     << s.goodsDemand << ','
     << s.goodsDelivered << ','
     << s.goodsSatisfaction << ','
     << s.avgLandValue << ','
     << s.demandResidential
     << '\n';
  return static_cast<bool>(os);
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string savePath;
  std::string outJson;
  std::string outCsv;

  // --- New headless export options ---
  struct PpmExport {
    ExportLayer layer = ExportLayer::Overlay;
    std::string path;
  };
  std::vector<PpmExport> ppmExports;
  int exportScale = 1;
  std::string tilesCsvPath;

  // Batch mode (optional): run multiple seeds in one invocation.
  int batchRuns = 1;


  std::uint64_t seed = 1;
  bool seedProvided = false;

  int w = 96;
  int h = 96;
  int days = 0;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string val;

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, val)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
      loadPath = val;
    } else if (arg == "--save") {
      if (!requireValue(i, val)) {
        std::cerr << "--save requires a path\n";
        return 2;
      }
      savePath = val;
    } else if (arg == "--out" || arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << arg << " requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      outCsv = val;
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
      seedProvided = true;
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &w, &h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--days" || arg == "--ticks") {
      if (!requireValue(i, val) || !ParseI32(val, &days) || days < 0) {
        std::cerr << arg << " requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      if (!requireValue(i, val)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      simCfg.requireOutsideConnection = (b != 0);
    } else if (arg == "--tax-res") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxResidential)) {
        std::cerr << "--tax-res requires an integer\n";
        return 2;
      }
    } else if (arg == "--tax-com") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxCommercial)) {
        std::cerr << "--tax-com requires an integer\n";
        return 2;
      }
    } else if (arg == "--tax-ind") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxIndustrial)) {
        std::cerr << "--tax-ind requires an integer\n";
        return 2;
      }
    } else if (arg == "--maint-road") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenanceRoad)) {
        std::cerr << "--maint-road requires an integer\n";
        return 2;
      }
    } else if (arg == "--maint-park") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenancePark)) {
        std::cerr << "--maint-park requires an integer\n";
        return 2;
      }
    } else if (arg == "--export-ppm") {
      std::string layerName;
      std::string outPath;
      if (!requireValue(i, layerName) || !requireValue(i, outPath)) {
        std::cerr << "--export-ppm requires: <layer> <out.ppm>\n";
        return 2;
      }
      ExportLayer layer = ExportLayer::Overlay;
      if (!ParseExportLayer(layerName, layer)) {
        std::cerr << "Unknown export layer: " << layerName << "\n";
        std::cerr << "Valid layers: terrain overlay height landvalue traffic goods_traffic goods_fill district\n";
        return 2;
      }
      ppmExports.push_back(PpmExport{layer, outPath});
    } else if (arg == "--export-scale") {
      if (!requireValue(i, val)) {
        std::cerr << "--export-scale requires an integer\n";
        return 2;
      }
      int s = 1;
      if (!ParseI32(val, &s) || s < 1) {
        std::cerr << "--export-scale requires an integer >= 1\n";
        return 2;
      }
      exportScale = s;
    } else if (arg == "--export-tiles-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--export-tiles-csv requires a path\n";
        return 2;
      }
      tilesCsvPath = val;
    } else if (arg == "--batch") {
      if (!requireValue(i, val)) {
        std::cerr << "--batch requires an integer\n";
        return 2;
      }
      int n = 1;
      if (!ParseI32(val, &n) || n < 1) {
        std::cerr << "--batch requires an integer >= 1\n";
        return 2;
      }
      batchRuns = n;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n\n";
      PrintHelp();
      return 2;
    }
  }

  if (batchRuns > 1 && !loadPath.empty()) {
    std::cerr << "--batch cannot be combined with --load\n";
    return 2;
  }

  if (!seedProvided) {
    // Keep a deterministic default so CI runs are stable.
    seed = 1;
  }

  auto hasBatchToken = [](const std::string& p) -> bool {
    return p.find("{seed}") != std::string::npos || p.find("{run}") != std::string::npos;
  };

  if (batchRuns > 1) {
    auto checkTemplate = [&](const std::string& p, const char* flag) -> bool {
      if (p.empty()) return true;
      if (hasBatchToken(p)) return true;
      std::cerr << "When using --batch, " << flag << " should include {seed} or {run} to avoid overwriting: " << p << "\n";
      return false;
    };

    if (!checkTemplate(outJson, "--out")) return 2;
    if (!checkTemplate(outCsv, "--csv")) return 2;
    if (!checkTemplate(savePath, "--save")) return 2;
    if (!checkTemplate(tilesCsvPath, "--export-tiles-csv")) return 2;
    for (const auto& e : ppmExports) {
      if (!checkTemplate(e.path, "--export-ppm")) return 2;
    }
  }

  auto replaceAll = [](std::string s, const std::string& from, const std::string& to) -> std::string {
    if (from.empty()) return s;
    std::size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
    return s;
  };

  auto expandPath = [&](const std::string& tmpl, int runIdx, std::uint64_t runSeed) -> std::string {
    if (tmpl.empty()) return {};
    std::string out = tmpl;
    out = replaceAll(out, "{seed}", std::to_string(runSeed));
    out = replaceAll(out, "{run}", std::to_string(runIdx));
    out = replaceAll(out, "{w}", std::to_string(w));
    out = replaceAll(out, "{h}", std::to_string(h));
    out = replaceAll(out, "{days}", std::to_string(days));
    return out;
  };

  auto ensureParentDir = [](const std::string& filePath) {
    if (filePath.empty()) return;
    const std::filesystem::path p(filePath);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
    }
  };

  auto runOne = [&](int runIdx, std::uint64_t requestedSeed) -> int {
    World world;
    ProcGenConfig runProcCfg = procCfg;
    SimConfig runSimCfg = simCfg;

    if (!loadPath.empty()) {
      std::string err;
      if (!LoadWorldBinary(world, runProcCfg, runSimCfg, loadPath, err)) {
        std::cerr << "Failed to load save: " << err << "\n";
        return 1;
      }
    } else {
      world = GenerateWorld(w, h, requestedSeed, runProcCfg);
    }

    const std::uint64_t actualSeed = world.seed();

    Simulator sim(runSimCfg);
    sim.refreshDerivedStats(world);

    std::ofstream csv;
    const std::string csvPath = expandPath(outCsv, runIdx, actualSeed);
    if (!csvPath.empty()) {
      ensureParentDir(csvPath);
      csv.open(csvPath, std::ios::binary);
      if (!csv) {
        std::cerr << "Failed to open CSV for writing: " << csvPath << "\n";
        return 1;
      }
      csv << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
      WriteCsvRow(csv, world.stats());
    }

    for (int i = 0; i < days; ++i) {
      sim.stepOnce(world);
      if (csv) WriteCsvRow(csv, world.stats());
    }

    sim.refreshDerivedStats(world);

    const std::string saveP = expandPath(savePath, runIdx, actualSeed);
    if (!saveP.empty()) {
      ensureParentDir(saveP);
      std::string err;
      if (!SaveWorldBinary(world, runProcCfg, sim.config(), saveP, err)) {
        std::cerr << "Failed to save world: " << err << "\n";
        return 1;
      }
    }

    const std::string tilesP = expandPath(tilesCsvPath, runIdx, actualSeed);
    if (!tilesP.empty()) {
      ensureParentDir(tilesP);
      std::string err;
      if (!WriteTilesCsv(world, tilesP, err)) {
        std::cerr << "Failed to write tiles CSV: " << tilesP;
        if (!err.empty()) std::cerr << " (" << err << ")";
        std::cerr << "\n";
        return 1;
      }
    }

    // Optional derived-map exports (PPM images)
    if (!ppmExports.empty()) {
      bool needTraffic = false;
      bool needGoods = false;
      bool needLandValue = false;

      for (const auto& e : ppmExports) {
        if (e.layer == ExportLayer::Traffic) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }

      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
      if (sim.config().requireOutsideConnection && (needTraffic || needGoods || needLandValue)) {
        ComputeRoadsConnectedToEdge(world, roadToEdge);
        roadToEdgeMask = &roadToEdge;
      }

      std::optional<TrafficResult> trafficRes;
      if (needTraffic || needLandValue) {
        TrafficConfig tc{};
        tc.requireOutsideConnection = sim.config().requireOutsideConnection;

        // Mirror simulator traffic model settings (so CLI exports match in-game overlays).
        tc.congestionAwareRouting = sim.trafficModel().congestionAwareRouting;
        tc.congestionIterations = sim.trafficModel().congestionIterations;
        tc.congestionAlpha = sim.trafficModel().congestionAlpha;
        tc.congestionBeta = sim.trafficModel().congestionBeta;
        tc.congestionCapacityScale = sim.trafficModel().congestionCapacityScale;
        tc.congestionRatioClamp = sim.trafficModel().congestionRatioClamp;

        const float employedShare = (world.stats().population > 0)
                                        ? static_cast<float>(world.stats().employed) / static_cast<float>(world.stats().population)
                                        : 0.0f;

        trafficRes = ComputeCommuteTraffic(world, tc, employedShare, roadToEdgeMask);
      }

      std::optional<GoodsResult> goodsRes;
      if (needGoods) {
        GoodsConfig gc{};
        gc.requireOutsideConnection = sim.config().requireOutsideConnection;
        goodsRes = ComputeGoodsFlow(world, gc, roadToEdgeMask);
      }

      std::optional<LandValueResult> landValueRes;
      if (needLandValue) {
        LandValueConfig lc{};
        landValueRes = ComputeLandValue(world, lc, trafficRes ? &(*trafficRes) : nullptr, roadToEdgeMask);
      }

      for (const auto& e : ppmExports) {
        const std::string outP = expandPath(e.path, runIdx, actualSeed);
        ensureParentDir(outP);

        PpmImage img = RenderPpmLayer(world, e.layer, landValueRes ? &(*landValueRes) : nullptr,
                                     trafficRes ? &(*trafficRes) : nullptr, goodsRes ? &(*goodsRes) : nullptr);
        if (exportScale > 1) img = ScaleNearest(img, exportScale);

        std::string err;
        if (!WritePpm(outP, img, err)) {
          std::cerr << "Failed to write PPM (" << ExportLayerName(e.layer) << "): " << outP << " (" << err << ")\n";
          return 1;
        }
      }
    }

    const std::uint64_t hash = HashWorld(world, true);
    const std::string jsonP = expandPath(outJson, runIdx, actualSeed);
    if (!WriteJsonSummary(world, hash, jsonP)) {
      std::cerr << "Failed to write JSON summary" << (jsonP.empty() ? "" : (": " + jsonP)) << "\n";
      return 1;
    }

    return 0;
  };

  for (int runIdx = 0; runIdx < batchRuns; ++runIdx) {
    const std::uint64_t runSeed = seed + static_cast<std::uint64_t>(runIdx);
    const int rc = runOne(runIdx, runSeed);
    if (rc != 0) return rc;
  }

  return 0;
}
