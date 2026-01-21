#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Isochrone.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/TransitPlanner.hpp"
#include "isocity/TransitPlannerExport.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  long v = std::strtol(s.c_str(), &end, 10);
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

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  unsigned long long v = std::strtoull(s.c_str(), &end, 0);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseF64(const std::string& s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  double v = std::strtod(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

bool ParseSize(const std::string& s, int* outW, int* outH)
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

bool requireValue(int& i, std::string& out, int argc, char** argv)
{
  if (i + 1 >= argc) return false;
  out = argv[++i];
  return true;
}

enum class DemandMode : std::uint8_t {
  Commute,
  Goods,
  Combined,
};

bool ParseDemandMode(const std::string& s, DemandMode* out)
{
  if (!out) return false;
  if (s == "commute") {
    *out = DemandMode::Commute;
    return true;
  }
  if (s == "goods") {
    *out = DemandMode::Goods;
    return true;
  }
  if (s == "combined") {
    *out = DemandMode::Combined;
    return true;
  }
  return false;
}

bool ParseTransitWeightMode(const std::string& s, isocity::TransitEdgeWeightMode* out)
{
  if (!out) return false;
  if (s == "steps") {
    *out = isocity::TransitEdgeWeightMode::Steps;
    return true;
  }
  if (s == "time" || s == "travel" || s == "traveltime") {
    *out = isocity::TransitEdgeWeightMode::TravelTime;
    return true;
  }
  return false;
}


bool ParseStopMode(const std::string& s, isocity::TransitStopMode* out)
{
  if (!out) return false;
  if (s == "nodes" || s == "node") {
    *out = isocity::TransitStopMode::Nodes;
    return true;
  }
  if (s == "tiles" || s == "tile") {
    *out = isocity::TransitStopMode::Tiles;
    return true;
  }
  return false;
}

bool ParseIsochroneWeightMode(const std::string& s, isocity::IsochroneWeightMode* out)
{
  if (!out) return false;
  if (s == "steps" || s == "walk") {
    *out = isocity::IsochroneWeightMode::Steps;
    return true;
  }
  if (s == "time" || s == "travel" || s == "traveltime") {
    *out = isocity::IsochroneWeightMode::TravelTime;
    return true;
  }
  return false;
}

bool ParseExportLayerName(const std::string& s, isocity::ExportLayer* out)
{
  return isocity::ParseExportLayer(s, *out);
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_transitplan (transit line planning from simulated demand)\n\n"
      << "Generates (or loads) a world, optionally steps the simulation, builds a RoadGraph,\n"
      << "computes a demand signal on road tiles (commute traffic, goods shipments, or both),\n"
      << "aggregates demand onto the RoadGraph edges, then runs a deterministic greedy heuristic\n"
      << "to output a small set of high-demand transit lines.\n\n"
      << "Usage:\n"
      << "  proc_isocity_transitplan [--seed N] [--size WxH] [--days N] [--load save.bin] [options...]\n\n"
      << "Inputs:\n"
      << "  --load <path>          Load a save instead of generating a new world.\n"
      << "  --seed <u64>           World seed (decimal or 0x...). Default: 1\n"
      << "  --size <WxH>           World size when generating. Default: 128x128\n"
      << "  --days <N>             Step the simulation N days before analysis. Default: 60\n"
      << "\n"
      << "Demand:\n"
      << "  --demand <commute|goods|combined>   Demand source. Default: combined\n"
      << "  --require-outside <0|1>             Enforce outside-connection roads. Default: 1\n"
      << "  --allow-imports <0|1>               Goods config (only affects goods/combined). Default: 1\n"
      << "  --allow-exports <0|1>               Goods config (only affects goods/combined). Default: 1\n"
      << "\n"
      << "Planner:\n"
      << "  --lines <N>              Max number of lines. Default: 8\n"
      << "  --endpoints <N>          Endpoint candidate nodes. Default: 24\n"
      << "  --weight <steps|time>    Path cost metric. Default: time\n"
      << "  --demand-bias <f>        Demand bias strength. Default: 2.5\n"
      << "  --max-detour <f>         Max detour vs shortest path. Default: 1.6\n"
      << "  --cover-fraction <f>     Demand consumption per chosen line. Default: 0.7\n"
      << "  --min-edge-demand <N>    Ignore edges below this remaining demand for bias. Default: 1\n"
      << "  --min-line-demand <N>    Skip candidate lines below this demand. Default: 50\n"
      << "\n"
      << "Exports:\n"
      << "  --json <path>            Write JSON plan.\n"
      << "  --geojson <path>         Write GeoJSON FeatureCollection (LineString + optional stops).\n"
      << "  --include-tiles <0|1>    Include per-line road-tile polylines. Default: 1\n"
      << "  --include-stops <0|1>    Include stop points. Default: 1\n"
      << "  --stop-mode <nodes|tiles>  How stops are emitted/drawn. Default: nodes\n"
      << "  --stop-spacing <N>         Stop spacing in road tiles when stop-mode=tiles. Default: 12\n"
      << "\n"
      << "Access analysis:\n"
      << "  --access-json <path>       Write JSON summary of distance-to-stop for residents/jobs.\n"
      << "  --access-heat <path>       Write per-tile heatmap overlay of distance-to-nearest stop.\n"
      << "  --access-weight <steps|time>  Road routing weights for access. Default: steps\n"
      << "  --access-walk-cost <N>     Extra milli-step cost when mapping road->tile. Default: 0\n"
      << "\n"
      << "Images:\n"
      << "  --overlay <path>         Per-tile overlay image (PPM/PNG by extension).\n"
      << "  --iso <path>             Isometric overlay image (PPM/PNG by extension).\n"
      << "  --base-layer <name>      Base layer for images (overlay/terrain/height/...). Default: overlay\n"
      << "  --scale <N>              Nearest-neighbor upscale for images. Default: 4\n"
      << "  --iso-tile <WxH>         Isometric tile size. Default: 16x8\n"
      << "  --iso-height <N>         Isometric height scale in pixels. Default: 14\n"
      << "\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;
  int days = 60;

  DemandMode demandMode = DemandMode::Combined;
  bool requireOutside = true;

  // Goods options (only relevant if demandMode includes goods).
  bool allowImports = true;
  bool allowExports = true;

  TransitPlannerConfig pcfg{};
  pcfg.maxLines = 8;
  pcfg.endpointCandidates = 24;
  pcfg.weightMode = TransitEdgeWeightMode::TravelTime;
  pcfg.demandBias = 2.5;
  pcfg.maxDetour = 1.6;
  pcfg.coverFraction = 0.7;
  pcfg.minEdgeDemand = 1;
  pcfg.minLineDemand = 50;
  pcfg.seedSalt = 0;

  TransitPlanExportConfig ecfg{};
  ecfg.includeTiles = true;
  ecfg.includeStops = true;
  ecfg.stopMode = TransitStopMode::Nodes;
  ecfg.stopSpacingTiles = 12;

  std::string jsonPath;
  std::string geojsonPath;
  std::string overlayPath;
  std::string isoPath;

  // Optional access-to-transit analysis exports.
  std::string accessJsonPath;
  std::string accessHeatPath;
  IsochroneWeightMode accessWeightMode = IsochroneWeightMode::Steps;
  int accessWalkCostMilli = 0;

  ExportLayer baseLayer = ExportLayer::Overlay;
  int scale = 4;
  IsoOverviewConfig isoCfg;
  isoCfg.tileW = 16;
  isoCfg.tileH = 8;
  isoCfg.heightScalePx = 14;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string val;

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
      loadPath = val;
    } else if (arg == "--seed") {
      if (!requireValue(i, val, argc, argv) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires a u64\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val, argc, argv) || !ParseSize(val, &w, &h)) {
        std::cerr << "--size requires WxH\n";
        return 2;
      }
    } else if (arg == "--days") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &days) || days < 0) {
        std::cerr << "--days requires N >= 0\n";
        return 2;
      }
    } else if (arg == "--demand") {
      if (!requireValue(i, val, argc, argv) || !ParseDemandMode(val, &demandMode)) {
        std::cerr << "--demand requires commute|goods|combined\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &requireOutside)) {
        std::cerr << "--require-outside requires 0|1\n";
        return 2;
      }
    } else if (arg == "--allow-imports") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &allowImports)) {
        std::cerr << "--allow-imports requires 0|1\n";
        return 2;
      }
    } else if (arg == "--allow-exports") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &allowExports)) {
        std::cerr << "--allow-exports requires 0|1\n";
        return 2;
      }
    } else if (arg == "--lines") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &pcfg.maxLines) || pcfg.maxLines < 0) {
        std::cerr << "--lines requires N >= 0\n";
        return 2;
      }
    } else if (arg == "--endpoints") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &pcfg.endpointCandidates) || pcfg.endpointCandidates < 2) {
        std::cerr << "--endpoints requires N >= 2\n";
        return 2;
      }
    } else if (arg == "--weight") {
      if (!requireValue(i, val, argc, argv) || !ParseTransitWeightMode(val, &pcfg.weightMode)) {
        std::cerr << "--weight requires steps|time\n";
        return 2;
      }
    } else if (arg == "--demand-bias") {
      if (!requireValue(i, val, argc, argv) || !ParseF64(val, &pcfg.demandBias) || pcfg.demandBias < 0.0) {
        std::cerr << "--demand-bias requires f >= 0\n";
        return 2;
      }
    } else if (arg == "--max-detour") {
      if (!requireValue(i, val, argc, argv) || !ParseF64(val, &pcfg.maxDetour) || pcfg.maxDetour <= 0.0) {
        std::cerr << "--max-detour requires f > 0\n";
        return 2;
      }
    } else if (arg == "--cover-fraction") {
      if (!requireValue(i, val, argc, argv) || !ParseF64(val, &pcfg.coverFraction) || pcfg.coverFraction < 0.0 || pcfg.coverFraction > 1.0) {
        std::cerr << "--cover-fraction requires f in [0,1]\n";
        return 2;
      }
    } else if (arg == "--min-edge-demand") {
      if (!requireValue(i, val, argc, argv) || !ParseU64(val, &pcfg.minEdgeDemand)) {
        std::cerr << "--min-edge-demand requires u64\n";
        return 2;
      }
    } else if (arg == "--min-line-demand") {
      if (!requireValue(i, val, argc, argv) || !ParseU64(val, &pcfg.minLineDemand)) {
        std::cerr << "--min-line-demand requires u64\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      jsonPath = val;
    } else if (arg == "--geojson") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--geojson requires a path\n";
        return 2;
      }
      geojsonPath = val;
    } else if (arg == "--include-tiles") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &ecfg.includeTiles)) {
        std::cerr << "--include-tiles requires 0|1\n";
        return 2;
      }
    } else if (arg == "--include-stops") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &ecfg.includeStops)) {
        std::cerr << "--include-stops requires 0|1\n";
        return 2;
      }
    } else if (arg == "--stop-mode") {
      if (!requireValue(i, val, argc, argv) || !ParseStopMode(val, &ecfg.stopMode)) {
        std::cerr << "--stop-mode requires nodes|tiles\n";
        return 2;
      }
    } else if (arg == "--stop-spacing") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &ecfg.stopSpacingTiles) || ecfg.stopSpacingTiles < 1) {
        std::cerr << "--stop-spacing requires N >= 1\n";
        return 2;
      }
    } else if (arg == "--access-json") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--access-json requires a path\n";
        return 2;
      }
      accessJsonPath = val;
    } else if (arg == "--access-heat") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--access-heat requires a path\n";
        return 2;
      }
      accessHeatPath = val;
    } else if (arg == "--access-weight") {
      if (!requireValue(i, val, argc, argv) || !ParseIsochroneWeightMode(val, &accessWeightMode)) {
        std::cerr << "--access-weight requires steps|time\n";
        return 2;
      }
    } else if (arg == "--access-walk-cost") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &accessWalkCostMilli) || accessWalkCostMilli < 0) {
        std::cerr << "--access-walk-cost requires N >= 0\n";
        return 2;
      }
    } else if (arg == "--overlay") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--overlay requires a path\n";
        return 2;
      }
      overlayPath = val;
    } else if (arg == "--iso") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--iso requires a path\n";
        return 2;
      }
      isoPath = val;
    } else if (arg == "--base-layer") {
      if (!requireValue(i, val, argc, argv) || !ParseExportLayerName(val, &baseLayer)) {
        std::cerr << "--base-layer requires a valid layer name (e.g. overlay/terrain/height)\n";
        return 2;
      }
    } else if (arg == "--scale") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &scale) || scale < 1) {
        std::cerr << "--scale requires N >= 1\n";
        return 2;
      }
    } else if (arg == "--iso-tile") {
      int tw = 0, th = 0;
      if (!requireValue(i, val, argc, argv) || !ParseSize(val, &tw, &th) || (tw % 2) != 0 || (th % 2) != 0) {
        std::cerr << "--iso-tile requires even WxH (e.g. 16x8)\n";
        return 2;
      }
      isoCfg.tileW = tw;
      isoCfg.tileH = th;
    } else if (arg == "--iso-height") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &isoCfg.heightScalePx) || isoCfg.heightScalePx < 0) {
        std::cerr << "--iso-height requires N >= 0\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      std::cerr << "Run with --help for usage.\n";
      return 2;
    }
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n" << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  simCfg.requireOutsideConnection = requireOutside;
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) sim.stepOnce(world);
  if (days == 0) sim.refreshDerivedStats(world);

  // Precompute outside-connected roads and zone access once and share between traffic and goods.
  std::vector<std::uint8_t> roadToEdge;
  const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (requireOutside) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
    roadToEdgePtr = &roadToEdge;
  }
  const ZoneAccessMap zoneAccess = BuildZoneAccessMap(world, roadToEdgePtr);

  // Demand on road tiles.
  const int ww = world.width();
  const int hh = world.height();
  std::vector<std::uint32_t> roadFlow(static_cast<std::size_t>(ww) * static_cast<std::size_t>(hh), 0);

  TrafficResult tr;
  GoodsResult gr;

  if (demandMode == DemandMode::Commute || demandMode == DemandMode::Combined) {
    TrafficConfig tcfg;
    tcfg.requireOutsideConnection = requireOutside;
    float employedShare = 1.0f;
    const Stats& s = world.stats();
    if (s.population > 0) employedShare = static_cast<float>(s.employed) / static_cast<float>(s.population);
    tr = ComputeCommuteTraffic(world, tcfg, employedShare, roadToEdgePtr, &zoneAccess);
    for (std::size_t i = 0; i < roadFlow.size() && i < tr.roadTraffic.size(); ++i) {
      roadFlow[i] += static_cast<std::uint32_t>(tr.roadTraffic[i]);
    }
  }

  if (demandMode == DemandMode::Goods || demandMode == DemandMode::Combined) {
    GoodsConfig gcfg;
    gcfg.requireOutsideConnection = requireOutside;
    gcfg.allowImports = allowImports;
    gcfg.allowExports = allowExports;
    gr = ComputeGoodsFlow(world, gcfg, roadToEdgePtr, &zoneAccess, nullptr);
    for (std::size_t i = 0; i < roadFlow.size() && i < gr.roadGoodsTraffic.size(); ++i) {
      roadFlow[i] += static_cast<std::uint32_t>(gr.roadGoodsTraffic[i]);
    }
  }

  const RoadGraph rg = BuildRoadGraph(world);
  const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(world, rg, roadFlow);

  // Convert aggregated edge stats into a demand vector.
  std::vector<std::uint64_t> edgeDemand;
  edgeDemand.resize(rg.edges.size(), 0);
  for (std::size_t ei = 0; ei < edgeDemand.size() && ei < agg.edges.size(); ++ei) {
    const RoadGraphTrafficEdgeStats& es = agg.edges[ei];
    edgeDemand[ei] = (es.interiorTileCount > 0) ? es.sumTrafficInterior : es.sumTrafficAll;
  }

  const World* costWorld = (pcfg.weightMode == TransitEdgeWeightMode::TravelTime) ? &world : nullptr;
  const TransitPlan plan = PlanTransitLines(rg, edgeDemand, pcfg, costWorld);

  std::cout << "TransitPlan summary\n";
  std::cout << "  world: " << world.width() << "x" << world.height() << "  day=" << world.stats().day << "\n";
  std::cout << "  roadGraph: nodes=" << rg.nodes.size() << " edges=" << rg.edges.size() << "\n";
  std::cout << "  demandMode=" << (demandMode == DemandMode::Commute ? "commute" : (demandMode == DemandMode::Goods ? "goods" : "combined"))
            << "  totalDemand=" << plan.totalDemand << " coveredDemand=" << plan.coveredDemand << "\n";
  std::cout << "  planner: lines=" << plan.lines.size() << "/" << pcfg.maxLines
            << " endpoints=" << pcfg.endpointCandidates
            << " weight=" << TransitEdgeWeightModeName(pcfg.weightMode)
            << " demandBias=" << pcfg.demandBias
            << " maxDetour=" << pcfg.maxDetour
            << " coverFraction=" << pcfg.coverFraction << "\n";

  const int printLines = std::min<int>(static_cast<int>(plan.lines.size()), 10);
  for (int i = 0; i < printLines; ++i) {
    const TransitLine& l = plan.lines[static_cast<std::size_t>(i)];
    std::size_t stopCount = ecfg.includeStops ? l.nodes.size() : 0;
    if (ecfg.includeStops && ecfg.stopMode == TransitStopMode::Tiles) {
      std::vector<Point> stops;
      if (BuildTransitLineStopTiles(rg, l, ecfg.stopSpacingTiles, stops)) stopCount = stops.size();
    }
    std::cout << "    line " << l.id << ": stops=" << stopCount
              << " edges=" << l.edges.size()
              << " sumDemand=" << l.sumDemand
              << " baseCost=" << l.baseCost << "\n";
  }

  // Optional access-to-transit analysis (distance-to-nearest planned stop).
  //
  // This uses the same road-graph routing machinery as isochrones. By default the access metric is in
  // road-steps (IsochroneWeightMode::Steps), which is a decent proxy for walking distance to a stop.
  if (!accessJsonPath.empty() || !accessHeatPath.empty()) {
    const int ww = world.width();
    const int hh = world.height();
    const std::size_t n = static_cast<std::size_t>(ww) * static_cast<std::size_t>(hh);

    // Collect unique stop tiles across all planned lines.
    std::vector<int> stopRoadIdx;
    std::vector<Point> stopPoints;
    std::vector<std::uint8_t> seen(n, std::uint8_t{0});

    auto addStop = [&](const Point& p) {
      if (!world.inBounds(p.x, p.y)) return;
      const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(ww) +
                              static_cast<std::size_t>(p.x);
      if (idx >= seen.size() || seen[idx]) return;
      seen[idx] = 1;

      const Tile& t = world.at(p.x, p.y);
      if (t.overlay != Overlay::Road) return;

      stopRoadIdx.push_back(static_cast<int>(idx));
      stopPoints.push_back(p);
    };

    if (ecfg.includeStops) {
      for (const TransitLine& l : plan.lines) {
        if (ecfg.stopMode == TransitStopMode::Nodes) {
          for (int nid : l.nodes) {
            if (nid < 0 || nid >= static_cast<int>(rg.nodes.size())) continue;
            addStop(rg.nodes[static_cast<std::size_t>(nid)].pos);
          }
        } else {
          std::vector<Point> stops;
          if (BuildTransitLineStopTiles(rg, l, ecfg.stopSpacingTiles, stops)) {
            for (const Point& p : stops) addStop(p);
          }
        }
      }
    }

    RoadIsochroneConfig icfg;
    icfg.requireOutsideConnection = requireOutside;
    icfg.weightMode = accessWeightMode;
    icfg.computeOwner = false;

    const RoadIsochroneField roadField = BuildRoadIsochroneField(world, stopRoadIdx, icfg, roadToEdgePtr, nullptr);

    TileAccessCostConfig tcfg;
    tcfg.includeRoadTiles = true;
    tcfg.includeZones = true;
    tcfg.includeNonZonesAdjacentToRoad = true;
    tcfg.includeWater = false;
    tcfg.accessStepCostMilli = accessWalkCostMilli;
    tcfg.useZoneAccessMap = true;

    const std::vector<int> tileCost = BuildTileAccessCostField(world, roadField, tcfg, roadToEdgePtr);

    struct WeightedCostSummary {
      std::uint64_t totalWeight = 0;
      std::uint64_t reachableWeight = 0;
      double avgCostMilli = -1.0;
      int p50CostMilli = -1;
      int p95CostMilli = -1;
      std::vector<std::uint64_t> withinWeight;
    };

    const std::vector<int> thresholdsSteps = {5, 10, 20};

    auto weightedQuantile = [&](std::vector<std::pair<int, std::uint64_t>>& cw, std::uint64_t totalW, double q) -> int {
      if (cw.empty() || totalW == 0) return -1;
      std::sort(cw.begin(), cw.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
      const double target = q * static_cast<double>(totalW);
      std::uint64_t acc = 0;
      for (const auto& it : cw) {
        acc += it.second;
        if (static_cast<double>(acc) >= target) return it.first;
      }
      return cw.back().first;
    };

    auto computeSummary = [&](auto pred) -> WeightedCostSummary {
      WeightedCostSummary s;
      s.withinWeight.assign(thresholdsSteps.size(), 0);
      std::vector<std::pair<int, std::uint64_t>> cw;
      cw.reserve(1024);

      std::uint64_t sumCost = 0;
      for (int y = 0; y < hh; ++y) {
        for (int x = 0; x < ww; ++x) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) +
                                  static_cast<std::size_t>(x);
          if (idx >= tileCost.size()) continue;

          const Tile& t = world.at(x, y);
          if (!pred(t)) continue;

          const std::uint64_t wgt = static_cast<std::uint64_t>(t.occupants);
          if (wgt == 0) continue;

          s.totalWeight += wgt;

          const int c = tileCost[idx];
          if (c < 0) continue;

          s.reachableWeight += wgt;
          sumCost += static_cast<std::uint64_t>(c) * wgt;
          cw.emplace_back(c, wgt);

          for (std::size_t i = 0; i < thresholdsSteps.size(); ++i) {
            const int thr = thresholdsSteps[i] * 1000;
            if (c <= thr) s.withinWeight[i] += wgt;
          }
        }
      }

      if (s.reachableWeight > 0) {
        s.avgCostMilli = static_cast<double>(sumCost) / static_cast<double>(s.reachableWeight);
        s.p50CostMilli = weightedQuantile(cw, s.reachableWeight, 0.50);
        s.p95CostMilli = weightedQuantile(cw, s.reachableWeight, 0.95);
      }

      return s;
    };

    const WeightedCostSummary resident = computeSummary([](const Tile& t) { return t.overlay == Overlay::Residential; });
    const WeightedCostSummary jobs = computeSummary([](const Tile& t) {
      return t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial;
    });

    // JSON summary export.
    if (!accessJsonPath.empty()) {
      if (!EnsureParentDir(accessJsonPath)) {
        std::cerr << "Failed to create output directory for access-json: " << accessJsonPath << "\n";
        return 2;
      }
      std::ofstream f(accessJsonPath, std::ios::binary);
      if (!f) {
        std::cerr << "Failed to write access-json: " << accessJsonPath << "\n";
        return 2;
      }

      auto writeSummary = [&](const char* name, const WeightedCostSummary& s) {
        f << "    \"" << name << "\":{";
        f << "\"total\":" << s.totalWeight << ",";
        f << "\"reachable\":" << s.reachableWeight << ",";
        f << "\"shareReachable\":" << (s.totalWeight > 0 ? (static_cast<double>(s.reachableWeight) /
                                                         static_cast<double>(s.totalWeight))
                                                     : 0.0)
          << ",";
        f << "\"avgCostMilli\":" << s.avgCostMilli << ",";
        f << "\"p50CostMilli\":" << s.p50CostMilli << ",";
        f << "\"p95CostMilli\":" << s.p95CostMilli << ",";
        f << "\"withinSteps\":[";
        for (std::size_t i = 0; i < thresholdsSteps.size(); ++i) {
          if (i) f << ",";
          const double share = (s.totalWeight > 0) ? (static_cast<double>(s.withinWeight[i]) /
                                                    static_cast<double>(s.totalWeight))
                                                 : 0.0;
          f << "{\"steps\":" << thresholdsSteps[i] << ",\"count\":" << s.withinWeight[i] << ",\"share\":" << share << "}";
        }
        f << "]}";
      };

      f << "{\n";
      f << "  \"version\":1,\n";
      f << "  \"stopMode\":\"" << TransitStopModeName(ecfg.stopMode) << "\",\n";
      f << "  \"stopSpacingTiles\":" << ecfg.stopSpacingTiles << ",\n";
      f << "  \"accessWeightMode\":\"" << (accessWeightMode == IsochroneWeightMode::Steps ? "steps" : "time") << "\",\n";
      f << "  \"accessWalkCostMilli\":" << accessWalkCostMilli << ",\n";
      f << "  \"stopCount\":" << stopRoadIdx.size() << ",\n";
      f << "  \"thresholdsSteps\":[";
      for (std::size_t i = 0; i < thresholdsSteps.size(); ++i) {
        if (i) f << ",";
        f << thresholdsSteps[i];
      }
      f << "],\n";
      f << "  \"groups\":{\n";
      writeSummary("residents", resident);
      f << ",\n";
      writeSummary("jobs", jobs);
      f << "\n  }\n";
      f << "}\n";
    }

    // Heatmap export.
    if (!accessHeatPath.empty()) {
      PpmImage img = RenderPpmLayer(world, baseLayer, nullptr, nullptr, nullptr);

      // Pick a robust max for normalization (95th percentile of reachable costs).
      std::vector<int> costs;
      costs.reserve(tileCost.size());
      for (int c : tileCost) {
        if (c >= 0) costs.push_back(c);
      }
      int maxCost = 1;
      if (!costs.empty()) {
        std::sort(costs.begin(), costs.end());
        const std::size_t idx = (costs.size() - 1) * 95 / 100;
        maxCost = std::max(1, costs[idx]);
      }

      auto blend = [&](std::uint8_t& dst, std::uint8_t src) {
        // 1/3 original + 2/3 overlay.
        dst = static_cast<std::uint8_t>((static_cast<int>(dst) + static_cast<int>(src) * 2) / 3);
      };

      auto setPixel = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
        if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
        const std::size_t o = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) +
                               static_cast<std::size_t>(x)) *
                              3u;
        if (o + 2 >= img.rgb.size()) return;
        img.rgb[o + 0] = r;
        img.rgb[o + 1] = g;
        img.rgb[o + 2] = b;
      };

      // Heat overlay.
      for (int y = 0; y < hh; ++y) {
        for (int x = 0; x < ww; ++x) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) +
                                  static_cast<std::size_t>(x);
          if (idx >= tileCost.size()) continue;

          const int c = tileCost[idx];
          if (c < 0) continue;

          const double t = std::clamp(static_cast<double>(c) / static_cast<double>(maxCost), 0.0, 1.0);
          const std::uint8_t rr = static_cast<std::uint8_t>(t * 255.0 + 0.5);
          const std::uint8_t gg = static_cast<std::uint8_t>((1.0 - t) * 255.0 + 0.5);
          const std::uint8_t bb = 0;

          const std::size_t o = idx * 3u;
          if (o + 2 >= img.rgb.size()) continue;
          blend(img.rgb[o + 0], rr);
          blend(img.rgb[o + 1], gg);
          blend(img.rgb[o + 2], bb);
        }
      }

      // Draw line polylines (crisp on top of heatmap).
      auto lineColor = [](int lineId) -> std::array<std::uint8_t, 3> {
        std::uint32_t x = static_cast<std::uint32_t>(lineId) * 2654435761u;
        std::uint8_t r = static_cast<std::uint8_t>(64 + (x & 0x7F));
        std::uint8_t g = static_cast<std::uint8_t>(64 + ((x >> 8) & 0x7F));
        std::uint8_t b = static_cast<std::uint8_t>(64 + ((x >> 16) & 0x7F));
        return {r, g, b};
      };

      for (const TransitLine& l : plan.lines) {
        std::vector<Point> tiles;
        if (!BuildTransitLineTilePolyline(rg, l, tiles)) continue;
        const auto c = lineColor(l.id);
        for (const Point& p : tiles) {
          setPixel(p.x, p.y, c[0], c[1], c[2]);
        }
      }

      // Stops in white.
      for (const Point& p : stopPoints) setPixel(p.x, p.y, 255, 255, 255);

      if (scale > 1) img = ScaleNearest(img, scale);
      std::string err2;
      if (!WriteImageAuto(accessHeatPath, img, err2)) {
        std::cerr << "Failed to write access heatmap: " << accessHeatPath << "\n" << err2 << "\n";
        return 2;
      }
    }
  }

  std::string err;
  if (!jsonPath.empty()) {
    if (!ExportTransitPlanJson(jsonPath, rg, plan, ecfg, &err)) {
      std::cerr << "Failed to write JSON: " << jsonPath << "\n" << err << "\n";
      return 2;
    }
  }
  if (!geojsonPath.empty()) {
    if (!ExportTransitPlanGeoJson(geojsonPath, rg, plan, ecfg, &err)) {
      std::cerr << "Failed to write GeoJSON: " << geojsonPath << "\n" << err << "\n";
      return 2;
    }
  }
  if (!overlayPath.empty()) {
    PpmImage img = RenderTransitOverlayTile(world, baseLayer, rg, plan, ecfg);
    if (scale > 1) img = ScaleNearest(img, scale);
    if (!WriteImageAuto(overlayPath, img, err)) {
      std::cerr << "Failed to write overlay image: " << overlayPath << "\n" << err << "\n";
      return 2;
    }
  }
  if (!isoPath.empty()) {
    IsoOverviewResult iso = RenderTransitIsoOverlay(world, baseLayer, isoCfg, rg, plan, ecfg);
    PpmImage img = iso.image;
    if (scale > 1) img = ScaleNearest(img, scale);
    if (!WriteImageAuto(isoPath, img, err)) {
      std::cerr << "Failed to write iso image: " << isoPath << "\n" << err << "\n";
      return 2;
    }
  }

  return 0;
}
