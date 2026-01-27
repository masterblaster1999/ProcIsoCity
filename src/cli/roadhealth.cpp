#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/RoadHealth.hpp"
#include "isocity/SaveLoad.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;
namespace fs = std::filesystem;

void PrintHelp()
{
  std::cout
      << "proc_isocity_roadhealth (road network criticality + resilience bypass suggestions)\n\n"
      << "Usage:\n"
      << "  proc_isocity_roadhealth <save.bin> [options]\n\n"
      << "Options:\n"
      << "  --json <out.json>          Write a JSON report.\n"
      << "  --render-dir <dir>         Write PPM maps (road_centrality, road_vulnerability, road_bypass).\n"
      << "  --scale <N>                Nearest-neighbor scale for rendered maps (default: 2).\n"
      << "  --weight-mode <time|steps> Centrality weighting (default: time).\n"
      << "  --max-sources <N>           Sampled betweenness sources (0=auto, default: 0).\n"
      << "  --bypass-top <N>           Number of bypass suggestions (default: 3, 0 disables).\n"
      << "  --allow-bridges            Allow suggested bypasses to include bridges over water.\n"
      << "  --verify-crc               Verify CRC for v3+ saves (slower, but detects corruption).\n"
      << "  --quiet                    Suppress stdout summary (errors still print).\n"
      << "  -h, --help                 Show this help.\n";
}

bool ParseInt(const std::string& s, int& out)
{
  try {
    std::size_t pos = 0;
    const int v = std::stoi(s, &pos, 10);
    if (pos != s.size()) return false;
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseWeightMode(const std::string& s, RoadGraphEdgeWeightMode& out)
{
  if (s == "time" || s == "travel" || s == "traveltime") {
    out = RoadGraphEdgeWeightMode::TravelTimeMilli;
    return true;
  }
  if (s == "steps" || s == "distance" || s == "dist") {
    out = RoadGraphEdgeWeightMode::Steps;
    return true;
  }
  return false;
}

struct FieldStats {
  double mean = 0.0;
  double max = 0.0;
  int count = 0;
};

FieldStats ComputeRoadTileStats(const World& world, const std::vector<float>& field)
{
  FieldStats s{};
  if (world.width() <= 0 || world.height() <= 0) return s;
  const std::size_t n = static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height());
  if (field.size() != n) return s;

  double sum = 0.0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) +
                            static_cast<std::size_t>(x);
      const double v = static_cast<double>(field[i]);
      sum += v;
      s.max = std::max(s.max, v);
      s.count++;
    }
  }
  s.mean = (s.count > 0) ? (sum / static_cast<double>(s.count)) : 0.0;
  return s;
}

int CountMaskOnes(const std::vector<std::uint8_t>& m)
{
  int c = 0;
  for (std::uint8_t v : m) c += (v ? 1 : 0);
  return c;
}

static bool WriteReportJson(const std::string& outPath, const std::string& inPath,
                            const SaveSummary* sum, const World& world,
                            const RoadHealthResult& r,
                            const FieldStats& centStats, const FieldStats& vulnStats)
{
  JsonValue root = JsonValue::MakeObject();

  auto add = [](JsonValue& obj, const char* key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

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
    add(cfg, "weightMode",
        JsonValue::MakeString(r.cfg.weightMode == RoadGraphEdgeWeightMode::Steps ? "steps" : "time"));
    add(cfg, "maxSources", JsonValue::MakeNumber(static_cast<double>(r.cfg.maxSources)));
    add(cfg, "includeNodeCentrality", JsonValue::MakeBool(r.cfg.includeNodeCentrality));
    add(cfg, "articulationVulnerabilityBase",
        JsonValue::MakeNumber(static_cast<double>(r.cfg.articulationVulnerabilityBase)));
    add(cfg, "includeBypass", JsonValue::MakeBool(r.cfg.includeBypass));

    JsonValue bc = JsonValue::MakeObject();
    add(bc, "top", JsonValue::MakeNumber(static_cast<double>(r.cfg.bypassCfg.top)));
    add(bc, "moneyObjective", JsonValue::MakeBool(r.cfg.bypassCfg.moneyObjective));
    add(bc, "targetLevel", JsonValue::MakeNumber(static_cast<double>(r.cfg.bypassCfg.targetLevel)));
    add(bc, "allowBridges", JsonValue::MakeBool(r.cfg.bypassCfg.allowBridges));
    add(bc, "rankByTraffic", JsonValue::MakeBool(r.cfg.bypassCfg.rankByTraffic));
    add(cfg, "bypassConfig", std::move(bc));

    add(root, "config", std::move(cfg));
  }

  // Summary.
  {
    JsonValue s = JsonValue::MakeObject();
    add(s, "roadGraphNodes", JsonValue::MakeNumber(static_cast<double>(r.nodes)));
    add(s, "roadGraphEdges", JsonValue::MakeNumber(static_cast<double>(r.edges)));
    add(s, "centralitySourcesUsed", JsonValue::MakeNumber(static_cast<double>(r.sourcesUsed)));
    add(s, "bridgeEdges", JsonValue::MakeNumber(static_cast<double>(r.bridgeEdges)));
    add(s, "articulationNodes", JsonValue::MakeNumber(static_cast<double>(r.articulationNodes)));
    add(s, "bypassSuggestions", JsonValue::MakeNumber(static_cast<double>(r.bypasses.size())));
    add(s, "bypassTiles", JsonValue::MakeNumber(static_cast<double>(CountMaskOnes(r.bypassMask))));

    JsonValue cs = JsonValue::MakeObject();
    add(cs, "meanOnRoad", JsonValue::MakeNumber(centStats.mean));
    add(cs, "maxOnRoad", JsonValue::MakeNumber(centStats.max));
    add(cs, "roadTileCount", JsonValue::MakeNumber(static_cast<double>(centStats.count)));
    add(s, "centralityStats", std::move(cs));

    JsonValue vs = JsonValue::MakeObject();
    add(vs, "meanOnRoad", JsonValue::MakeNumber(vulnStats.mean));
    add(vs, "maxOnRoad", JsonValue::MakeNumber(vulnStats.max));
    add(vs, "roadTileCount", JsonValue::MakeNumber(static_cast<double>(vulnStats.count)));
    add(s, "vulnerabilityStats", std::move(vs));

    add(root, "summary", std::move(s));
  }

  // Bypass suggestions.
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(r.bypasses.size());

    for (const RoadResilienceBypassSuggestion& sug : r.bypasses) {
      JsonValue o = JsonValue::MakeObject();
      add(o, "bridgeEdge", JsonValue::MakeNumber(static_cast<double>(sug.bridgeEdge)));
      add(o, "cutSize", JsonValue::MakeNumber(static_cast<double>(sug.cutSize)));
      add(o, "primaryCost", JsonValue::MakeNumber(static_cast<double>(sug.primaryCost)));
      add(o, "moneyCost", JsonValue::MakeNumber(static_cast<double>(sug.moneyCost)));
      add(o, "newTiles", JsonValue::MakeNumber(static_cast<double>(sug.newTiles)));
      add(o, "steps", JsonValue::MakeNumber(static_cast<double>(sug.steps)));
      add(o, "targetLevel", JsonValue::MakeNumber(static_cast<double>(sug.targetLevel)));
      add(o, "allowBridges", JsonValue::MakeBool(sug.allowBridges));
      add(o, "moneyObjective", JsonValue::MakeBool(sug.moneyObjective));

      // Path as [[x,y],...]
      JsonValue p = JsonValue::MakeArray();
      p.arrayValue.reserve(sug.path.size());
      for (const Point& pt : sug.path) {
        JsonValue xy = JsonValue::MakeArray();
        xy.arrayValue.emplace_back(JsonValue::MakeNumber(static_cast<double>(pt.x)));
        xy.arrayValue.emplace_back(JsonValue::MakeNumber(static_cast<double>(pt.y)));
        p.arrayValue.emplace_back(std::move(xy));
      }
      add(o, "path", std::move(p));

      arr.arrayValue.emplace_back(std::move(o));
    }

    add(root, "bypasses", std::move(arr));
  }

  std::string err;
  return WriteJsonFile(outPath, root, err, JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false});
}

static void PrintSummary(const RoadHealthResult& r, const FieldStats& cent, const FieldStats& vuln)
{
  auto pct = [](double v01) -> double { return v01 * 100.0; };

  std::cout << "Road health summary\n";
  std::cout << "- road graph: " << r.nodes << " nodes, " << r.edges << " edges\n";
  std::cout << "- centrality sources used: " << r.sourcesUsed << "\n";
  std::cout << "- bridges: " << r.bridgeEdges << ", articulation nodes: " << r.articulationNodes << "\n";
  std::cout << "- centrality (road tiles): mean=" << pct(cent.mean) << "%, max=" << pct(cent.max) << "%\n";
  std::cout << "- vulnerability (road tiles): mean=" << pct(vuln.mean) << "%, max=" << pct(vuln.max) << "%\n";

  if (!r.bypasses.empty()) {
    std::cout << "- bypass suggestions: " << r.bypasses.size() << " (tiles marked: " << CountMaskOnes(r.bypassMask) << ")\n";
    const int show = std::min(3, static_cast<int>(r.bypasses.size()));
    for (int i = 0; i < show; ++i) {
      const RoadResilienceBypassSuggestion& s = r.bypasses[static_cast<std::size_t>(i)];
      std::cout << "  - #" << (i + 1) << ": bridgeEdge=" << s.bridgeEdge << " cutSize=" << s.cutSize
                << " newTiles=" << s.newTiles << " moneyCost=" << s.moneyCost << " steps=" << s.steps << "\n";
    }
  }
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

  RoadHealthConfig rh{};
  rh.weightMode = RoadGraphEdgeWeightMode::TravelTimeMilli;
  rh.maxSources = 0;
  rh.autoExactMaxNodes = 650;
  rh.autoSampleSources = 256;
  rh.includeNodeCentrality = true;
  rh.articulationVulnerabilityBase = 0.70f;
  rh.includeBypass = true;
  rh.bypassCfg.top = 3;
  rh.bypassCfg.moneyObjective = true;
  rh.bypassCfg.targetLevel = 1;
  rh.bypassCfg.allowBridges = false;
  rh.bypassCfg.rankByTraffic = true;

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
      int v = 2;
      if (!ParseInt(argv[++i], v) || v <= 0) {
        std::cerr << "Invalid --scale value\n";
        return 2;
      }
      scale = v;
      continue;
    }
    if (arg == "--weight-mode" && i + 1 < argc) {
      RoadGraphEdgeWeightMode wm = RoadGraphEdgeWeightMode::TravelTimeMilli;
      if (!ParseWeightMode(argv[++i], wm)) {
        std::cerr << "Invalid --weight-mode (use 'time' or 'steps')\n";
        return 2;
      }
      rh.weightMode = wm;
      continue;
    }
    if (arg == "--max-sources" && i + 1 < argc) {
      int v = 0;
      if (!ParseInt(argv[++i], v) || v < 0) {
        std::cerr << "Invalid --max-sources value\n";
        return 2;
      }
      rh.maxSources = v;
      continue;
    }
    if (arg == "--bypass-top" && i + 1 < argc) {
      int v = 0;
      if (!ParseInt(argv[++i], v) || v < 0) {
        std::cerr << "Invalid --bypass-top value\n";
        return 2;
      }
      rh.bypassCfg.top = v;
      rh.includeBypass = (v > 0);
      continue;
    }
    if (arg == "--allow-bridges") {
      rh.bypassCfg.allowBridges = true;
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      return 2;
    }

    if (inPath.empty()) {
      inPath = arg;
    } else {
      std::cerr << "Unexpected extra argument: " << arg << "\n";
      return 2;
    }
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

  // NOTE: We intentionally don't compute TrafficResult here; bypass ranking
  // falls back to cut size when traffic is unavailable.
  const RoadHealthResult res = ComputeRoadHealth(world, rh, nullptr);

  const FieldStats cent = ComputeRoadTileStats(world, res.centrality01);
  const FieldStats vuln = ComputeRoadTileStats(world, res.vulnerability01);

  if (!quiet) {
    PrintSummary(res, cent, vuln);
  }

  if (!outJson.empty()) {
    if (!WriteReportJson(outJson, inPath, summaryPtr, world, res, cent, vuln)) {
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
        {ExportLayer::RoadCentrality, "map_road_centrality.ppm"},
        {ExportLayer::RoadVulnerability, "map_road_vulnerability.ppm"},
        {ExportLayer::RoadBypass, "map_road_bypass.ppm"},
    };

    for (const L& l : layers) {
      PpmImage img = RenderPpmLayer(world, l.layer);
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
