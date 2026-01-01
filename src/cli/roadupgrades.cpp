#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/RoadGraphTrafficExport.hpp"
#include "isocity/RoadUpgradePlanner.hpp"
#include "isocity/Road.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace isocity;

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

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  unsigned long long v = std::strtoull(s.c_str(), &end, 10);
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
  *out = (v == 1);
  return true;
}

bool ParseSize(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const auto xPos = s.find('x');
  if (xPos == std::string::npos) return false;
  const std::string a = s.substr(0, xPos);
  const std::string b = s.substr(xPos + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

const char* ObjectiveName(RoadUpgradeObjective obj)
{
  switch (obj) {
    case RoadUpgradeObjective::Congestion: return "congestion";
    case RoadUpgradeObjective::Time: return "time";
    case RoadUpgradeObjective::Hybrid: return "hybrid";
    default: return "congestion";
  }
}

bool ParseObjective(const std::string& s, RoadUpgradeObjective* out)
{
  if (!out) return false;
  std::string t = s;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (t == "congestion" || t == "excess") {
    *out = RoadUpgradeObjective::Congestion;
    return true;
  }
  if (t == "time" || t == "travel" || t == "traveltime") {
    *out = RoadUpgradeObjective::Time;
    return true;
  }
  if (t == "hybrid" || t == "mix") {
    *out = RoadUpgradeObjective::Hybrid;
    return true;
  }
  return false;
}

void PrintHelp()
{
  std::cout
    << "proc_isocity_roadupgrades\n"
    << "\n"
    << "Plans road *upgrades* (street->avenue->highway) under a budget, based on a combined\n"
    << "per-road-tile flow map (commute traffic + optional goods shipments).\n"
    << "\n"
    << "World input:\n"
    << "  --load <save.bin>            Load a saved world.\n"
    << "  --seed <N>                   Generate a new world seed (default: 1).\n"
    << "  --size <WxH>                 World size when generating (default: 128x128).\n"
    << "  --days <N>                   Simulate N days before analyzing (default: 60).\n"
    << "  --require-outside <0|1>      Require road connectivity to map edge (default: 1).\n"
    << "\n"
    << "Traffic / goods:\n"
    << "  --base-capacity <N>          Street capacity per tile (default: 28).\n"
    << "  --use-road-level-cap <0|1>   Capacity scales with road level (default: 1).\n"
    << "  --include-goods <0|1>        Include goods shipments in the flow map (default: 1).\n"
    << "  --goods-weight <F>           Goods flow weight relative to commuters (default: 1.0).\n"
    << "\n"
    << "Upgrade planning:\n"
    << "  --budget <N>                 Money budget (default: -1 = unlimited).\n"
    << "  --objective <name>           congestion|time|hybrid (default: congestion).\n"
    << "  --min-util <F>               Only consider edges with max util >= F (default: 1.0).\n"
    << "  --upgrade-endpoints <0|1>    Include node tiles in edge upgrades (default: 0).\n"
    << "  --max-level <1..3>           Max level to propose (default: 3).\n"
    << "  --hybrid-excess-w <F>        Hybrid weight for excess reduction (default: 1.0).\n"
    << "  --hybrid-time-w <F>          Hybrid weight for time saved (default: 1.0).\n"
    << "\n"
    << "Outputs:\n"
    << "  --json <path>                Write a JSON report with the selected upgrades.\n"
    << "  --edges-csv <path>           Write upgraded edges CSV.\n"
    << "  --tiles-csv <path>           Write upgraded tiles CSV.\n"
    << "  --highlight <path>           Write an overlay image highlighting upgraded tiles.\n"
    << "  --scale <N>                  Nearest-neighbor upscale factor for highlight (default: 4).\n"
    << "  --dot <path>                 Export a DOT road-graph colored by combined utilization.\n"
    << "  --write-save <path>          Write a save with the upgrades applied (does not charge money).\n"
    << "  --include-tiles <0|1>        Include full per-tile upgrade list in JSON (default: 0).\n"
    << "\n"
    << "Examples:\n"
    << "  ./build/proc_isocity_roadupgrades --seed 1 --size 128x128 --days 60 --budget 250 \\\n"
    << "    --objective congestion --json upgrades.json --highlight upgrades.png --scale 4\n"
    << "\n";
}

bool WriteEdgesCsv(const std::string& path, const RoadUpgradePlan& plan, std::string& err)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "Failed to open " + path;
    return false;
  }
  f << "edge,a,b,targetLevel,cost,timeSaved,excessReduced,tileCount\n";
  for (const RoadUpgradeEdge& e : plan.edges) {
    f << e.edgeIndex << "," << e.a << "," << e.b << "," << e.targetLevel << "," << e.cost << ","
      << e.timeSaved << "," << e.excessReduced << "," << e.tileCount << "\n";
  }
  return true;
}

bool WriteTilesCsv(const std::string& path, const World& world,
                   const std::vector<std::uint32_t>& flow,
                   const RoadUpgradePlan& plan,
                   std::string& err)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return true;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (flow.size() != n || plan.tileTargetLevel.size() != n) return true;

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "Failed to open " + path;
    return false;
  }
  f << "x,y,fromLevel,toLevel,flow,isBridge,cost,oldCap,newCap,oldTime,newTime\n";

  const int baseCap = std::max(1, plan.cfg.baseTileCapacity);
  const bool useCaps = plan.cfg.useRoadLevelCapacity;

  for (std::size_t idx = 0; idx < n; ++idx) {
    const std::uint8_t tgt = plan.tileTargetLevel[idx];
    if (tgt == 0) continue;
    const int x = static_cast<int>(idx % static_cast<std::size_t>(w));
    const int y = static_cast<int>(idx / static_cast<std::size_t>(w));
    if (!world.inBounds(x, y)) continue;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) continue;

    const int fromLvl = std::max(1, std::min(3, static_cast<int>(t.level)));
    const int toLvl = std::max(fromLvl, std::max(1, std::min(3, static_cast<int>(tgt))));
    const bool isBridge = (t.terrain == Terrain::Water);

    const int cost = (toLvl > fromLvl) ? RoadPlacementCost(fromLvl, toLvl, /*alreadyRoad=*/true, isBridge) : 0;
    const int v = static_cast<int>(std::min<std::uint32_t>(flow[idx], 1000000u));

    const int oldCap = useCaps ? RoadCapacityForLevel(baseCap, fromLvl) : baseCap;
    const int newCap = useCaps ? RoadCapacityForLevel(baseCap, toLvl) : baseCap;

    const int oldTime = isBridge ? RoadBridgeTravelTimeMilliForLevel(fromLvl) : RoadTravelTimeMilliForLevel(fromLvl);
    const int newTime = isBridge ? RoadBridgeTravelTimeMilliForLevel(toLvl) : RoadTravelTimeMilliForLevel(toLvl);

    f << x << "," << y << "," << fromLvl << "," << toLvl << "," << v << "," << (isBridge ? 1 : 0) << "," << cost
      << "," << oldCap << "," << newCap << "," << oldTime << "," << newTime << "\n";
  }
  return true;
}

bool WritePlanJson(const std::string& path,
                   const World& world,
                   const TrafficResult& tr,
                   const GoodsResult* goods,
                   double goodsWeight,
                   const std::vector<std::uint32_t>& combinedFlow,
                   const RoadGraph& rg,
                   const RoadGraphTrafficResult& agg,
                   const RoadUpgradePlan& plan,
                   bool includeTiles,
                   std::string& err)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "Failed to open " + path;
    return false;
  }

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  std::uint32_t maxFlow = 0;
  std::uint64_t sumFlow = 0;
  for (std::uint32_t v : combinedFlow) {
    maxFlow = std::max(maxFlow, v);
    sumFlow += static_cast<std::uint64_t>(v);
  }

  const Stats& s = world.stats();

  f << std::fixed << std::setprecision(4);
  f << "{\n";
  f << "  \"world\": {\n";
  f << "    \"w\": " << w << ", \"h\": " << h << ", \"day\": " << s.day << ",\n";
  f << "    \"population\": " << s.population << ", \"employed\": " << s.employed << ",\n";
  f << "    \"jobsAccessible\": " << s.jobsCapacityAccessible << ",\n";
  f << "    \"avgCommuteTime\": " << s.avgCommuteTime << ", \"p95CommuteTime\": " << s.p95CommuteTime << ",\n";
  f << "    \"trafficCongestion\": " << s.trafficCongestion << ",\n";
  f << "    \"goodsSatisfaction\": " << s.goodsSatisfaction << "\n";
  f << "  },\n";

  f << "  \"traffic\": {\n";
  f << "    \"maxCommuteTileTraffic\": " << tr.maxTraffic << ",\n";
  f << "    \"usedCongestionAwareRouting\": " << (tr.usedCongestionAwareRouting ? "true" : "false") << ",\n";
  f << "    \"routingPasses\": " << tr.routingPasses << "\n";
  f << "  },\n";

  if (goods) {
    f << "  \"goods\": {\n";
    f << "    \"included\": true,\n";
    f << "    \"weight\": " << goodsWeight << ",\n";
    f << "    \"produced\": " << goods->goodsProduced << ",\n";
    f << "    \"demand\": " << goods->goodsDemand << ",\n";
    f << "    \"delivered\": " << goods->goodsDelivered << ",\n";
    f << "    \"imported\": " << goods->goodsImported << ",\n";
    f << "    \"exported\": " << goods->goodsExported << ",\n";
    f << "    \"satisfaction\": " << goods->satisfaction << "\n";
    f << "  },\n";
  } else {
    f << "  \"goods\": { \"included\": false },\n";
  }

  f << "  \"combinedFlow\": {\n";
  f << "    \"maxTileFlow\": " << maxFlow << ",\n";
  f << "    \"sumTileFlow\": " << sumFlow << "\n";
  f << "  },\n";

  f << "  \"roadGraph\": {\n";
  f << "    \"nodes\": " << rg.nodes.size() << ", \"edges\": " << rg.edges.size() << ",\n";
  f << "    \"aggCfg\": { \"baseTileCapacity\": " << agg.cfg.baseTileCapacity
    << ", \"useRoadLevelCapacity\": " << (agg.cfg.useRoadLevelCapacity ? "true" : "false") << " }\n";
  f << "  },\n";

  f << "  \"planCfg\": {\n";
  f << "    \"budget\": " << plan.cfg.budget << ",\n";
  f << "    \"objective\": \"" << ObjectiveName(plan.cfg.objective) << "\",\n";
  f << "    \"baseTileCapacity\": " << plan.cfg.baseTileCapacity << ",\n";
  f << "    \"useRoadLevelCapacity\": " << (plan.cfg.useRoadLevelCapacity ? "true" : "false") << ",\n";
  f << "    \"minUtilConsider\": " << plan.cfg.minUtilConsider << ",\n";
  f << "    \"upgradeEndpoints\": " << (plan.cfg.upgradeEndpoints ? "true" : "false") << ",\n";
  f << "    \"maxTargetLevel\": " << plan.cfg.maxTargetLevel << ",\n";
  f << "    \"hybridExcessWeight\": " << plan.cfg.hybridExcessWeight << ",\n";
  f << "    \"hybridTimeWeight\": " << plan.cfg.hybridTimeWeight << "\n";
  f << "  },\n";

  f << "  \"plan\": {\n";
  f << "    \"selectedEdges\": " << plan.edges.size() << ",\n";
  f << "    \"totalCost\": " << plan.totalCost << ",\n";
  f << "    \"totalTimeSaved\": " << plan.totalTimeSaved << ",\n";
  f << "    \"totalExcessReduced\": " << plan.totalExcessReduced << ",\n";
  f << "    \"edges\": [\n";
  for (std::size_t i = 0; i < plan.edges.size(); ++i) {
    const RoadUpgradeEdge& e = plan.edges[i];
    f << "      {\"edge\": " << e.edgeIndex << ", \"a\": " << e.a << ", \"b\": " << e.b
      << ", \"targetLevel\": " << e.targetLevel << ", \"cost\": " << e.cost
      << ", \"timeSaved\": " << e.timeSaved << ", \"excessReduced\": " << e.excessReduced
      << ", \"tileCount\": " << e.tileCount << "}";
    f << ((i + 1 < plan.edges.size()) ? ",\n" : "\n");
  }
  f << "    ]";

  if (includeTiles) {
    f << ",\n    \"tiles\": [\n";
    bool first = true;
    for (std::size_t idx = 0; idx < n; ++idx) {
      const std::uint8_t tgt = (idx < plan.tileTargetLevel.size()) ? plan.tileTargetLevel[idx] : 0;
      if (tgt == 0) continue;
      const int x = static_cast<int>(idx % static_cast<std::size_t>(w));
      const int y = static_cast<int>(idx / static_cast<std::size_t>(w));
      if (!first) f << ",\n";
      first = false;
      f << "      {\"x\": " << x << ", \"y\": " << y << ", \"toLevel\": " << static_cast<int>(tgt) << "}";
    }
    f << "\n    ]\n";
    f << "  }\n";
  } else {
    f << "\n  }\n";
  }

  f << "}\n";
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;
  int days = 60;
  bool requireOutside = true;

  int baseCapacity = 28;
  bool useRoadLevelCapacity = true;

  bool includeGoods = true;
  double goodsWeight = 1.0;

  int budget = -1;
  RoadUpgradeObjective objective = RoadUpgradeObjective::Congestion;
  double minUtil = 1.0;
  bool upgradeEndpoints = false;
  int maxLevel = 3;
  double hybridExcessW = 1.0;
  double hybridTimeW = 1.0;

  std::string jsonPath;
  std::string edgesCsvPath;
  std::string tilesCsvPath;
  std::string highlightPath;
  std::string dotPath;
  std::string writeSavePath;
  int scale = 4;
  bool includeTilesInJson = false;

  auto requireValue = [&](int& i, std::string& outVal) -> bool {
    if (i + 1 >= argc) return false;
    outVal = argv[++i];
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
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires an unsigned integer\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseSize(val, &w, &h)) {
        std::cerr << "--size requires WxH (eg. 128x128)\n";
        return 2;
      }
    } else if (arg == "--days") {
      if (!requireValue(i, val) || !ParseI32(val, &days) || days < 0) {
        std::cerr << "--days requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      if (!requireValue(i, val) || !ParseBool01(val, &requireOutside)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--base-capacity") {
      if (!requireValue(i, val) || !ParseI32(val, &baseCapacity) || baseCapacity <= 0) {
        std::cerr << "--base-capacity requires an integer > 0\n";
        return 2;
      }
    } else if (arg == "--use-road-level-cap") {
      if (!requireValue(i, val) || !ParseBool01(val, &useRoadLevelCapacity)) {
        std::cerr << "--use-road-level-cap requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--include-goods") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeGoods)) {
        std::cerr << "--include-goods requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--goods-weight") {
      if (!requireValue(i, val) || !ParseF64(val, &goodsWeight) || goodsWeight < 0.0) {
        std::cerr << "--goods-weight requires a float >= 0\n";
        return 2;
      }
    } else if (arg == "--budget") {
      if (!requireValue(i, val) || !ParseI32(val, &budget)) {
        std::cerr << "--budget requires an integer (use -1 for unlimited)\n";
        return 2;
      }
    } else if (arg == "--objective") {
      if (!requireValue(i, val) || !ParseObjective(val, &objective)) {
        std::cerr << "--objective requires congestion|time|hybrid\n";
        return 2;
      }
    } else if (arg == "--min-util") {
      if (!requireValue(i, val) || !ParseF64(val, &minUtil) || minUtil < 0.0) {
        std::cerr << "--min-util requires a float >= 0\n";
        return 2;
      }
    } else if (arg == "--upgrade-endpoints") {
      if (!requireValue(i, val) || !ParseBool01(val, &upgradeEndpoints)) {
        std::cerr << "--upgrade-endpoints requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--max-level") {
      if (!requireValue(i, val) || !ParseI32(val, &maxLevel) || maxLevel < 1 || maxLevel > 3) {
        std::cerr << "--max-level requires 1..3\n";
        return 2;
      }
    } else if (arg == "--hybrid-excess-w") {
      if (!requireValue(i, val) || !ParseF64(val, &hybridExcessW) || hybridExcessW < 0.0) {
        std::cerr << "--hybrid-excess-w requires a float >= 0\n";
        return 2;
      }
    } else if (arg == "--hybrid-time-w") {
      if (!requireValue(i, val) || !ParseF64(val, &hybridTimeW) || hybridTimeW < 0.0) {
        std::cerr << "--hybrid-time-w requires a float >= 0\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      jsonPath = val;
    } else if (arg == "--edges-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--edges-csv requires a path\n";
        return 2;
      }
      edgesCsvPath = val;
    } else if (arg == "--tiles-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--tiles-csv requires a path\n";
        return 2;
      }
      tilesCsvPath = val;
    } else if (arg == "--highlight") {
      if (!requireValue(i, val)) {
        std::cerr << "--highlight requires a path\n";
        return 2;
      }
      highlightPath = val;
    } else if (arg == "--scale") {
      if (!requireValue(i, val) || !ParseI32(val, &scale) || scale < 1) {
        std::cerr << "--scale requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--dot") {
      if (!requireValue(i, val)) {
        std::cerr << "--dot requires a path\n";
        return 2;
      }
      dotPath = val;
    } else if (arg == "--write-save") {
      if (!requireValue(i, val)) {
        std::cerr << "--write-save requires a path\n";
        return 2;
      }
      writeSavePath = val;
    } else if (arg == "--include-tiles") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeTilesInJson)) {
        std::cerr << "--include-tiles requires 0 or 1\n";
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

  // Load/generate.
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

  // Optionally simulate some days to populate zones.
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) sim.stepOnce(world);
  if (days == 0) sim.refreshDerivedStats(world);

  // Compute commute traffic.
  TrafficConfig tcfg;
  tcfg.requireOutsideConnection = requireOutside;
  tcfg.roadTileCapacity = baseCapacity;
  tcfg.includeCommercialJobs = true;
  tcfg.includeIndustrialJobs = true;

  float employedShare = 1.0f;
  if (world.stats().population > 0) {
    employedShare = static_cast<float>(world.stats().employed) / static_cast<float>(world.stats().population);
  }

  const TrafficResult tr = ComputeCommuteTraffic(world, tcfg, employedShare);

  // Optional goods flow.
  GoodsResult goods;
  GoodsResult* goodsPtr = nullptr;
  if (includeGoods) {
    GoodsConfig gcfg;
    gcfg.requireOutsideConnection = requireOutside;
    goods = ComputeGoodsFlow(world, gcfg);
    goodsPtr = &goods;
  }

  // Combine flows.
  const int worldW = world.width();
  const int worldH = world.height();
  const std::size_t n = static_cast<std::size_t>(worldW) * static_cast<std::size_t>(worldH);
  std::vector<std::uint32_t> combinedFlow(n, 0);
  if (tr.roadTraffic.size() == n) {
    for (std::size_t i = 0; i < n; ++i) combinedFlow[i] = static_cast<std::uint32_t>(tr.roadTraffic[i]);
  }
  if (goodsPtr && goodsPtr->roadGoodsTraffic.size() == n && goodsWeight > 0.0) {
    for (std::size_t i = 0; i < n; ++i) {
      const double add = goodsWeight * static_cast<double>(goodsPtr->roadGoodsTraffic[i]);
      const std::uint32_t addU = (add <= 0.0) ? 0u : static_cast<std::uint32_t>(std::llround(add));
      combinedFlow[i] = combinedFlow[i] + addU;
    }
  }

  const RoadGraph rg = BuildRoadGraph(world);

  RoadGraphTrafficConfig agCfg;
  agCfg.baseTileCapacity = baseCapacity;
  agCfg.useRoadLevelCapacity = useRoadLevelCapacity;
  const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(world, rg, combinedFlow, agCfg);

  RoadUpgradePlannerConfig ucfg;
  ucfg.baseTileCapacity = baseCapacity;
  ucfg.useRoadLevelCapacity = useRoadLevelCapacity;
  ucfg.budget = budget;
  ucfg.objective = objective;
  ucfg.minUtilConsider = minUtil;
  ucfg.upgradeEndpoints = upgradeEndpoints;
  ucfg.maxTargetLevel = maxLevel;
  ucfg.hybridExcessWeight = hybridExcessW;
  ucfg.hybridTimeWeight = hybridTimeW;

  const RoadUpgradePlan plan = PlanRoadUpgrades(world, rg, combinedFlow, ucfg);

  std::cout << "RoadUpgrades summary\n";
  std::cout << "  world: " << worldW << "x" << worldH << " day=" << world.stats().day << "\n";
  std::cout << "  roadGraph: nodes=" << rg.nodes.size() << " edges=" << rg.edges.size() << "\n";
  std::cout << "  combinedFlow: maxTileFlow=" << (combinedFlow.empty() ? 0u : *std::max_element(combinedFlow.begin(), combinedFlow.end()))
    << " includeGoods=" << (includeGoods ? 1 : 0) << " goodsWeight=" << goodsWeight << "\n";
  std::cout << "  plan: objective=" << ObjectiveName(objective) << " budget=" << budget
    << " selectedEdges=" << plan.edges.size() << " totalCost=" << plan.totalCost
    << " excessReduced=" << plan.totalExcessReduced << " timeSaved=" << plan.totalTimeSaved << "\n";

  std::string err;

  if (!dotPath.empty()) {
    RoadGraphTrafficExportConfig dotCfg;
    dotCfg.labelByUtilization = true;
    dotCfg.colorEdgesByUtilization = true;
    if (!ExportRoadGraphTrafficDot(dotPath, rg, agg, dotCfg, &err)) {
      std::cerr << "Failed to write DOT: " << dotPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!edgesCsvPath.empty()) {
    if (!WriteEdgesCsv(edgesCsvPath, plan, err)) {
      std::cerr << "Failed to write edges CSV: " << edgesCsvPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!tilesCsvPath.empty()) {
    if (!WriteTilesCsv(tilesCsvPath, world, combinedFlow, plan, err)) {
      std::cerr << "Failed to write tiles CSV: " << tilesCsvPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!highlightPath.empty()) {
    PpmImage img = RenderPpmLayer(world, ExportLayer::Overlay, nullptr, nullptr, nullptr);
    for (std::size_t idx = 0; idx < plan.tileTargetLevel.size(); ++idx) {
      if (plan.tileTargetLevel[idx] == 0) continue;
      const int x = static_cast<int>(idx % static_cast<std::size_t>(worldW));
      const int y = static_cast<int>(idx / static_cast<std::size_t>(worldW));
      SetPixel(img, x, y, 60, 140, 255);
    }
    img = ScaleNearest(img, scale);
    if (!WriteImageAuto(highlightPath, img, err)) {
      std::cerr << "Failed to write highlight image: " << highlightPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!writeSavePath.empty()) {
    World outWorld = world;
    ApplyRoadUpgradePlan(outWorld, plan);

    // Saving requires updated configs for compatibility.
    if (!SaveWorldBinary(outWorld, procCfg, simCfg, writeSavePath, err)) {
      std::cerr << "Failed to write save: " << writeSavePath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!jsonPath.empty()) {
    if (!WritePlanJson(jsonPath, world, tr, goodsPtr, goodsWeight, combinedFlow, rg, agg, plan, includeTilesInJson, err)) {
      std::cerr << "Failed to write JSON: " << jsonPath << "\n" << err << "\n";
      return 2;
    }
  }

  return 0;
}
