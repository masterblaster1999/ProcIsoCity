#include "isocity/AutoBuild.hpp"
#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Random.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

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
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, pos), &w)) return false;
  if (!ParseI32(s.substr(pos + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

bool EnsureParentDir(const std::string& path)
{
  if (path.empty()) return true;
  try {
    std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

std::string ReplaceAll(std::string s, const std::string& from, const std::string& to)
{
  if (from.empty()) return s;
  std::size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
  return s;
}

std::string ExpandTemplate(const std::string& pattern, const isocity::World& world, std::uint64_t hash)
{
  std::string out = pattern;
  out = ReplaceAll(out, "{w}", std::to_string(world.width()));
  out = ReplaceAll(out, "{h}", std::to_string(world.height()));
  out = ReplaceAll(out, "{seed}", std::to_string(world.seed()));
  out = ReplaceAll(out, "{day}", std::to_string(world.stats().day));
  out = ReplaceAll(out, "{money}", std::to_string(world.stats().money));
  out = ReplaceAll(out, "{hash}", HexU64(hash));
  return out;
}

bool WriteStatsCsv(const std::string& path, const std::vector<isocity::Stats>& rows)
{
  EnsureParentDir(path);
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;

  f << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
  for (const auto& s : rows) {
    f << s.day << ','
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
      << s.demandResidential << '\n';
  }

  return static_cast<bool>(f);
}

bool WriteJsonSummary(const isocity::World& world, const isocity::AutoBuildReport& r, std::uint64_t hash,
                      const std::string& outPath)
{
  const isocity::Stats& s = world.stats();

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"width\": " << world.width() << ",\n";
  oss << "  \"height\": " << world.height() << ",\n";
  oss << "  \"seed\": " << world.seed() << ",\n";
  oss << "  \"hash\": \"" << HexU64(hash) << "\",\n";
  oss << "  \"autobuild\": {\n";
  oss << "    \"daysSimulated\": " << r.daysSimulated << ",\n";
  oss << "    \"zonesBuilt\": " << r.zonesBuilt << ",\n";
  oss << "    \"roadsBuilt\": " << r.roadsBuilt << ",\n";
  oss << "    \"parksBuilt\": " << r.parksBuilt << ",\n";
  oss << "    \"roadsUpgraded\": " << r.roadsUpgraded << "\n";
  oss << "  },\n";
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

  EnsureParentDir(outPath);
  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_autobuild (deterministic city bot)\n\n"
      << "Usage:\n"
      << "  proc_isocity_autobuild --days <N> [--load <save.bin>] [--save <out.bin>] [--out <summary.json>] [--csv <ticks.csv>]\n"
      << "                      [--size WxH --seed <u64> [--empty]]\n"
      << "                      [--money <N>] [--bot <key> <value>]...\n"
      << "                      [--export-ppm <layer> <out.ppm>]... [--export-scale <N>] [--export-tiles-csv <tiles.csv>]\n\n"
      << "Notes:\n"
      << "  - If --load is omitted, a world is generated from --size/--seed (or defaults).\n"
      << "  - --empty creates a flat grass world instead of procedural generation.\n"
      << "  - Output paths support simple templates: {seed} {w} {h} {day} {money} {hash}\n"
      << "  - Export layers: terrain overlay height landvalue traffic goods_traffic goods_fill district\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  if (argc < 2) {
    PrintHelp();
    return 1;
  }

  int w = 96;
  int h = 96;
  std::uint64_t seed = 1;
  bool useEmpty = false;
  int days = -1;
  int startMoney = -1;

  std::string loadPath;
  std::string savePath;
  std::string outPath;
  std::string csvPath;
  std::string tilesCsvPath;
  int exportScale = 1;
  std::vector<std::pair<ExportLayer, std::string>> exports;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  AutoBuildConfig botCfg{};

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help" || a == "help") {
      PrintHelp();
      return 0;
    }

    if (a == "--size" && i + 1 < argc) {
      if (!ParseWxH(argv[++i], &w, &h)) {
        std::cerr << "--size expects WxH\n";
        return 1;
      }
      continue;
    }
    if (a == "--seed" && i + 1 < argc) {
      if (!ParseU64(argv[++i], &seed)) {
        std::cerr << "--seed expects u64 (decimal or 0x...)\n";
        return 1;
      }
      continue;
    }
    if (a == "--empty") {
      useEmpty = true;
      continue;
    }
    if (a == "--days" && i + 1 < argc) {
      if (!ParseI32(argv[++i], &days) || days < 0) {
        std::cerr << "--days expects non-negative integer\n";
        return 1;
      }
      continue;
    }
    if (a == "--money" && i + 1 < argc) {
      if (!ParseI32(argv[++i], &startMoney)) {
        std::cerr << "--money expects integer\n";
        return 1;
      }
      continue;
    }

    if (a == "--load" && i + 1 < argc) {
      loadPath = argv[++i];
      continue;
    }
    if (a == "--save" && i + 1 < argc) {
      savePath = argv[++i];
      continue;
    }
    if (a == "--out" && i + 1 < argc) {
      outPath = argv[++i];
      continue;
    }
    if (a == "--csv" && i + 1 < argc) {
      csvPath = argv[++i];
      continue;
    }
    if (a == "--export-scale" && i + 1 < argc) {
      if (!ParseI32(argv[++i], &exportScale) || exportScale < 1) {
        std::cerr << "--export-scale expects integer >= 1\n";
        return 1;
      }
      continue;
    }
    if (a == "--export-tiles-csv" && i + 1 < argc) {
      tilesCsvPath = argv[++i];
      continue;
    }

    if (a == "--bot" && i + 2 < argc) {
      const std::string key = argv[++i];
      const std::string val = argv[++i];
      std::string err;
      if (!ParseAutoBuildKey(key, val, botCfg, err)) {
        std::cerr << "--bot parse error: " << err << "\n";
        return 1;
      }
      continue;
    }

    if (a == "--export-ppm" && i + 2 < argc) {
      ExportLayer layer;
      const std::string layerName = argv[++i];
      const std::string path = argv[++i];
      if (!ParseExportLayer(layerName, layer)) {
        std::cerr << "Unknown export layer: " << layerName << "\n";
        return 1;
      }
      exports.emplace_back(layer, path);
      continue;
    }

    std::cerr << "Unknown option: " << a << "\n";
    return 1;
  }

  if (days < 0) {
    std::cerr << "Missing required: --days <N>\n";
    return 1;
  }

  World world;
  std::string err;
  if (!loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Load failed: " << err << "\n";
      return 1;
    }
  } else {
    if (useEmpty) {
      world = World(w, h, seed);
      const std::uint32_t seed32 = static_cast<std::uint32_t>(seed) ^ static_cast<std::uint32_t>(seed >> 32);
      for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
          Tile& t = world.at(x, y);
          t.terrain = Terrain::Grass;
          t.overlay = Overlay::None;
          t.height = 0.60f;
          t.variation = static_cast<std::uint8_t>(HashCoords32(x, y, seed32 ^ 0xA3C59AC3u) & 0xFFu);
          t.level = 1;
          t.occupants = 0;
          t.district = 0;
        }
      }
    } else {
      world = GenerateWorld(w, h, seed, procCfg);
    }
  }

  if (startMoney >= 0) {
    world.stats().money = startMoney;
  }

  Simulator sim(simCfg);
  sim.refreshDerivedStats(world);

  std::vector<Stats> tickStats;
  const AutoBuildReport report = RunAutoBuild(world, sim, botCfg, days, &tickStats);

  // Ensure final derived stats are up-to-date.
  sim.refreshDerivedStats(world);
  const std::uint64_t hash = HashWorld(world, /*includeStats=*/true);

  // Template-expand output paths now that we have final state.
  if (!savePath.empty()) savePath = ExpandTemplate(savePath, world, hash);
  if (!outPath.empty()) outPath = ExpandTemplate(outPath, world, hash);
  if (!csvPath.empty()) csvPath = ExpandTemplate(csvPath, world, hash);
  if (!tilesCsvPath.empty()) tilesCsvPath = ExpandTemplate(tilesCsvPath, world, hash);
  for (auto& e : exports) {
    e.second = ExpandTemplate(e.second, world, hash);
  }

  if (!csvPath.empty()) {
    if (!WriteStatsCsv(csvPath, tickStats)) {
      std::cerr << "Failed to write CSV: " << csvPath << "\n";
      return 1;
    }
  }

  if (!WriteJsonSummary(world, report, hash, outPath)) {
    std::cerr << "Failed to write summary: " << outPath << "\n";
    return 1;
  }

  if (!savePath.empty()) {
    std::string errSave;
    EnsureParentDir(savePath);
    if (!SaveWorldBinary(world, procCfg, simCfg, savePath, errSave)) {
      std::cerr << "Save failed: " << errSave << "\n";
      return 1;
    }
  }

  if (!tilesCsvPath.empty()) {
    std::string errTiles;
    if (!WriteTilesCsv(world, tilesCsvPath, errTiles)) {
      std::cerr << "tiles CSV failed: " << errTiles << "\n";
      return 1;
    }
  }

  // Exports (PPM).
  if (!exports.empty()) {
    // Compute derived fields once.
    TrafficConfig tc;
    tc.requireOutsideConnection = simCfg.requireOutsideConnection;
    const TrafficResult tr = ComputeCommuteTraffic(world, tc);

    GoodsConfig gc;
    gc.requireOutsideConnection = simCfg.requireOutsideConnection;
    std::vector<std::uint8_t> roadToEdge;
    if (simCfg.requireOutsideConnection) {
      ComputeRoadsConnectedToEdge(world, roadToEdge);
    }

    const GoodsResult gr = ComputeGoodsFlow(world, gc, simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

    LandValueConfig lvc;
    lvc.parkRadius = simCfg.parkInfluenceRadius;
    lvc.requireOutsideConnection = simCfg.requireOutsideConnection;
    const LandValueResult lvr = ComputeLandValue(world, lvc, &tr, simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

    for (const auto& [layer, outP] : exports) {
      const std::string outFile = outP;

      const PpmImage raw = RenderPpmLayer(world, layer, &lvr, &tr, &gr);
      const PpmImage img = (exportScale > 1) ? ScaleNearest(raw, exportScale) : raw;

      std::string errPpm;
      if (!WritePpm(outFile, img, errPpm)) {
        std::cerr << "PPM export failed: " << outFile << " : " << errPpm << "\n";
        return 1;
      }
    }
  }

  return 0;
}
