#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Replay.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/WorldPatch.hpp"

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

bool ReadFileBytes(const std::string& path, std::vector<std::uint8_t>& outBytes)
{
  outBytes.clear();
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  f.seekg(0, std::ios::end);
  const std::streampos end = f.tellg();
  if (end < 0) return false;
  f.seekg(0, std::ios::beg);

  const std::size_t sz = static_cast<std::size_t>(end);
  outBytes.resize(sz);
  if (sz > 0) {
    f.read(reinterpret_cast<char*>(outBytes.data()), static_cast<std::streamsize>(sz));
    if (!f.good()) return false;
  }
  return true;
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

  EnsureParentDir(outPath);
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

void PrintHelp()
{
  std::cout
      << "proc_isocity_replay (deterministic replay journal tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_replay pack <base.bin> <target.bin> <out.isoreplay> [--no-proc] [--no-sim] [--no-stats] [--no-compress]\n"
      << "                            [--note <text>]... [--assert-final-hash] [--assert-final-hash-raw]\n"
      << "  proc_isocity_replay info <replay.isoreplay>\n"
      << "  proc_isocity_replay play <replay.isoreplay> [--force] [--out <summary.json>] [--csv <ticks.csv>] [--save <final.bin>]\n"
      << "                          [--ignore-asserts]\n"
      << "                          [--export-ppm <layer> <out.ppm>]... [--export-scale <N>] [--export-tiles-csv <tiles.csv>]\n\n"
      << "Notes:\n"
      << "  - Replay files embed a full base save plus a stream of Tick/Patch/Snapshot events.\n"
      << "  - --force disables strict patch hash checks during playback (useful for debugging).\n"
      << "  - --ignore-asserts skips AssertHash events during playback.\n"
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

  const std::string cmd = argv[1];
  if (cmd == "-h" || cmd == "--help" || cmd == "help") {
    PrintHelp();
    return 0;
  }

  if (cmd == "pack") {
    if (argc < 5) {
      PrintHelp();
      return 1;
    }

    const std::string basePath = argv[2];
    const std::string targetPath = argv[3];
    const std::string outPath = argv[4];

    bool includeProc = true;
    bool includeSim = true;
    bool includeStats = true;
    WorldPatchCompression compression = WorldPatchCompression::SLLZ;
    bool assertFinalHash = false;
    bool assertFinalHashRaw = false;
    std::vector<std::string> notes;

    for (int i = 5; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--no-proc") includeProc = false;
      else if (a == "--no-sim") includeSim = false;
      else if (a == "--no-stats") includeStats = false;
      else if (a == "--no-compress") compression = WorldPatchCompression::None;
      else if (a == "--assert-final-hash") assertFinalHash = true;
      else if (a == "--assert-final-hash-raw") assertFinalHashRaw = true;
      else if (a == "--note") {
        if (i + 1 >= argc) {
          std::cerr << "--note requires a string\n";
          return 1;
        }
        notes.push_back(argv[++i]);
      }
      else {
        std::cerr << "Unknown option: " << a << "\n";
        return 1;
      }
    }

    World baseW;
    ProcGenConfig baseP;
    SimConfig baseS;
    World targetW;
    ProcGenConfig targetP;
    SimConfig targetS;
    std::string err;
    if (!LoadWorldBinary(baseW, baseP, baseS, basePath, err)) {
      std::cerr << "Failed to load base save: " << err << "\n";
      return 1;
    }
    if (!LoadWorldBinary(targetW, targetP, targetS, targetPath, err)) {
      std::cerr << "Failed to load target save: " << err << "\n";
      return 1;
    }

    WorldPatch patch;
    if (!MakeWorldPatch(baseW, baseP, baseS, targetW, targetP, targetS, patch, err, includeProc, includeSim,
                        includeStats)) {
      std::cerr << "Failed to create patch: " << err << "\n";
      return 1;
    }

    std::vector<std::uint8_t> patchBytes;
    if (!SerializeWorldPatchBinary(patch, patchBytes, err, compression)) {
      std::cerr << "Failed to serialize patch: " << err << "\n";
      return 1;
    }

    Replay replay;
    replay.baseSave.clear();
    if (!ReadFileBytes(basePath, replay.baseSave)) {
      std::cerr << "Failed to read base save bytes: " << basePath << "\n";
      return 1;
    }

    for (const std::string& n : notes) {
      ReplayEvent evNote;
      evNote.type = ReplayEventType::Note;
      evNote.note = n;
      replay.events.push_back(std::move(evNote));
    }

    ReplayEvent ev;
    ev.type = ReplayEventType::Patch;
    ev.patch = std::move(patchBytes);
    replay.events.push_back(std::move(ev));

    const bool wantAssert = assertFinalHash || assertFinalHashRaw;
    if (wantAssert) {
      ReplayEvent evAssert;
      evAssert.type = ReplayEventType::AssertHash;
      evAssert.includeStatsInHash = assertFinalHash;
      // In the pack case we already have targetW in memory, so we can compute
      // the expected value at authoring time.
      evAssert.expectedHash = HashWorld(targetW, evAssert.includeStatsInHash);
      evAssert.label = "final";
      replay.events.push_back(std::move(evAssert));
    }

    EnsureParentDir(outPath);
    if (!SaveReplayBinary(replay, outPath, err)) {
      std::cerr << "Failed to write replay: " << err << "\n";
      return 1;
    }

    std::cout << "Wrote replay: " << outPath << " (events=" << replay.events.size() << ")\n";
    return 0;
  }

  if (cmd == "info") {
    if (argc < 3) {
      PrintHelp();
      return 1;
    }

    const std::string path = argv[2];
    Replay replay;
    std::string err;
    if (!LoadReplayBinary(replay, path, err)) {
      std::cerr << "Failed to load replay: " << err << "\n";
      return 1;
    }

    std::size_t ticks = 0;
    std::size_t patches = 0;
    std::size_t snapshots = 0;
    std::size_t notes = 0;
    std::size_t asserts = 0;
    for (const ReplayEvent& e : replay.events) {
      if (e.type == ReplayEventType::Tick) {
        ticks += e.ticks;
      } else if (e.type == ReplayEventType::Patch) {
        patches++;
      } else if (e.type == ReplayEventType::Snapshot) {
        snapshots++;
      } else if (e.type == ReplayEventType::Note) {
        notes++;
      } else if (e.type == ReplayEventType::AssertHash) {
        asserts++;
      }
    }

    World world;
    ProcGenConfig proc;
    SimConfig sim;
    if (!LoadWorldBinaryFromBytes(world, proc, sim, replay.baseSave, err)) {
      std::cerr << "Replay base load failed: " << err << "\n";
      return 1;
    }

    std::cout << "Replay: " << path << "\n";
    std::cout << "  version: " << replay.version << "\n";
    std::cout << "  base save bytes: " << replay.baseSave.size() << "\n";
    std::cout << "  events: " << replay.events.size() << " (patches=" << patches << ", snapshots=" << snapshots
              << ", notes=" << notes << ", asserts=" << asserts << ", totalTicks=" << ticks << ")\n";
    std::cout << "  base world: " << world.width() << "x" << world.height() << "  seed=" << world.seed() << "  day="
              << world.stats().day << "\n";
    return 0;
  }

  if (cmd == "play") {
    if (argc < 3) {
      PrintHelp();
      return 1;
    }

    const std::string path = argv[2];
    std::string outJson;
    std::string outCsv;
    std::string savePath;
    std::string tilesCsvPath;
    int exportScale = 1;
    bool strict = true;
    bool strictAsserts = true;

    struct PpmExport {
      ExportLayer layer = ExportLayer::Overlay;
      std::string path;
    };
    std::vector<PpmExport> ppmExports;

    for (int i = 3; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--force") {
        strict = false;
      } else if (a == "--ignore-asserts" || a == "--no-asserts") {
        strictAsserts = false;
      } else if (a == "--out" || a == "--json") {
        if (i + 1 >= argc) {
          std::cerr << "--out requires a path\n";
          return 1;
        }
        outJson = argv[++i];
      } else if (a == "--csv") {
        if (i + 1 >= argc) {
          std::cerr << "--csv requires a path\n";
          return 1;
        }
        outCsv = argv[++i];
      } else if (a == "--save") {
        if (i + 1 >= argc) {
          std::cerr << "--save requires a path\n";
          return 1;
        }
        savePath = argv[++i];
      } else if (a == "--export-scale") {
        if (i + 1 >= argc) {
          std::cerr << "--export-scale requires an integer\n";
          return 1;
        }
        if (!ParseI32(argv[++i], &exportScale) || exportScale < 1) {
          std::cerr << "Invalid export scale\n";
          return 1;
        }
      } else if (a == "--export-tiles-csv") {
        if (i + 1 >= argc) {
          std::cerr << "--export-tiles-csv requires a path\n";
          return 1;
        }
        tilesCsvPath = argv[++i];
      } else if (a == "--export-ppm") {
        if (i + 2 >= argc) {
          std::cerr << "--export-ppm requires <layer> <path>\n";
          return 1;
        }
        ExportLayer layer;
        if (!ParseExportLayer(argv[i + 1], layer)) {
          std::cerr << "Unknown export layer: " << argv[i + 1] << "\n";
          return 1;
        }
        PpmExport e;
        e.layer = layer;
        e.path = argv[i + 2];
        ppmExports.push_back(e);
        i += 2;
      } else {
        std::cerr << "Unknown option: " << a << "\n";
        return 1;
      }
    }

    Replay replay;
    std::string err;
    if (!LoadReplayBinary(replay, path, err)) {
      std::cerr << "Failed to load replay: " << err << "\n";
      return 1;
    }

    World world;
    ProcGenConfig proc;
    SimConfig sim;

    std::vector<Stats> tickStats;
    std::vector<Stats>* tickPtr = outCsv.empty() ? nullptr : &tickStats;
    if (!PlayReplay(replay, world, proc, sim, err, strict, strictAsserts, tickPtr)) {
      std::cerr << "Replay failed: " << err << "\n";
      return 1;
    }

    // Refresh derived stats post-playback (some are recomputed by Simulator anyway,
    // but this makes sure a final snapshot is consistent).
    Simulator s(sim);
    s.refreshDerivedStats(world);

    const std::uint64_t hash = HashWorld(world, true);
    std::cout << "Final hash: " << HexU64(hash) << "\n";

    if (!outJson.empty()) {
      if (!WriteJsonSummary(world, hash, outJson)) {
        std::cerr << "Failed to write JSON summary: " << outJson << "\n";
        return 1;
      }
    }

    if (!outCsv.empty()) {
      EnsureParentDir(outCsv);
      std::ofstream csv(outCsv, std::ios::binary);
      if (!csv) {
        std::cerr << "Failed to open CSV for writing: " << outCsv << "\n";
        return 1;
      }
      csv << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
      for (const Stats& st : tickStats) {
        WriteCsvRow(csv, st);
      }
    }

    if (!savePath.empty()) {
      EnsureParentDir(savePath);
      if (!SaveWorldBinary(world, proc, sim, savePath, err)) {
        std::cerr << "Failed to save world: " << err << "\n";
        return 1;
      }
    }

    if (!tilesCsvPath.empty()) {
      EnsureParentDir(tilesCsvPath);
      if (!WriteTilesCsv(world, tilesCsvPath, err)) {
        std::cerr << "Failed to write tiles CSV: " << tilesCsvPath;
        if (!err.empty()) std::cerr << " (" << err << ")";
        std::cerr << "\n";
        return 1;
      }
    }

    if (!ppmExports.empty()) {
      bool needTraffic = false;
      bool needGoods = false;
      bool needLandValue = false;
      for (const auto& e : ppmExports) {
        if (e.layer == ExportLayer::Traffic) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }

      std::optional<TrafficResult> traffic;
      std::optional<GoodsResult> goods;
      std::optional<LandValueResult> land;

      if (needTraffic || needLandValue || needGoods) {
        TrafficConfig cfg;
        cfg.requireOutsideConnection = sim.requireOutsideConnection;
        traffic = ComputeCommuteTraffic(world, cfg, (world.stats().population > 0) ? (float(world.stats().employed) / float(world.stats().population)) : 1.0f);
      }
      if (needGoods) {
        GoodsConfig gcfg;
        gcfg.requireOutsideConnection = sim.requireOutsideConnection;
        goods = ComputeGoodsFlow(world, gcfg, nullptr);
      }
      if (needLandValue) {
        LandValueConfig lcfg;
        lcfg.requireOutsideConnection = sim.requireOutsideConnection;
        land = ComputeLandValue(world, lcfg, traffic ? &(*traffic) : nullptr, nullptr);
      }

      for (const auto& e : ppmExports) {
        PpmImage img = RenderPpmLayer(world, e.layer, land ? &(*land) : nullptr, traffic ? &(*traffic) : nullptr,
                                      goods ? &(*goods) : nullptr);
        if (exportScale > 1) img = ScaleNearest(img, exportScale);
        EnsureParentDir(e.path);
        if (!WritePpm(e.path, img, err)) {
          std::cerr << "Failed to write PPM: " << e.path;
          if (!err.empty()) std::cerr << " (" << err << ")";
          std::cerr << "\n";
          return 1;
        }
      }
    }

    return 0;
  }

  PrintHelp();
  return 1;
}
