#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
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

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  float v = std::strtof(s.c_str(), &end);
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

inline void SetPixel(isocity::PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx =
      (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_roadresilience (headless road-network vulnerability + bypass suggestions)\n\n"
      << "Builds (or loads) a world, compresses its road network into a RoadGraph, then computes:\n"
      << "  - bridge edges (cut-edges): removing the edge disconnects the road network\n"
      << "  - articulation nodes (cut-vertices): removing the node disconnects the road network\n\n"
      << "Optionally, it will suggest bypass roads for the top-N bridge edges by searching for the\n"
      << "cheapest road-build path that reconnects the two sides WITHOUT using the bridge segment.\n\n"
      << "Usage:\n"
      << "  proc_isocity_roadresilience [--seed N] [--size WxH] [--days N] [--load save.bin] [options]\n\n"
      << "Inputs:\n"
      << "  --load <path>            Load a save instead of generating a new world.\n"
      << "  --seed <u64>             World seed (decimal or 0x...). Default: 1\n"
      << "  --size <WxH>             World size when generating. Default: 128x128\n"
      << "  --days <N>               Step the simulation N days before analysis. Default: 60\n"
      << "  --require-outside <0|1>  Outside-connection rule for sim/traffic. Default: 1\n"
      << "\n"
      << "Traffic (optional, used for ranking bridges by congestion):\n"
      << "  --traffic <0|1>              Compute commute traffic + aggregate to edges. Default: 1\n"
      << "  --traffic-capacity <N>       Base road tile capacity (street). Default: 28\n"
      << "  --use-road-level-cap <0|1>   If 1, scale capacity by road class (Tile::level). Default: 1\n"
      << "  --congestion-aware <0|1>     Enable multi-pass assignment. Default: 0\n"
      << "  --passes <N>                 Number of routing passes. Default: 4\n"
      << "  --alpha <f>                  Congestion curve alpha. Default: 0.15\n"
      << "  --beta <f>                   Congestion curve beta. Default: 4.0\n"
      << "  --cap-scale <f>              Capacity scale for congestion cost. Default: 1.0\n"
      << "  --ratio-clamp <f>            Clamp v/c in congestion curve. Default: 3.0\n"
      << "\n"
      << "Bypass suggestions:\n"
      << "  --suggest-bypasses <0|1>     Try to suggest bypass roads for bridge edges. Default: 1\n"
      << "  --bypass-top <N>             Consider the top N bridges (ranked by traffic util or cut size). Default: 10\n"
      << "  --bypass-money <0|1>         If 1, optimize money cost; else optimize new-tile count. Default: 0\n"
      << "  --bypass-target-level <N>    Target road level for money cost (1..3). Default: 1\n"
      << "  --bypass-allow-bridges <0|1> Allow building bridges on water. Default: 0\n"
      << "  --bypass-max-cost <N>        Optional cap on bypass primary cost (0 = unlimited). Default: 0\n"
      << "\n"
      << "Outputs:\n"
      << "  --json <path>                JSON report (bridges, articulations, suggestions).\n"
      << "  --include-tiles <0|1>        Include per-bridge tile polylines in JSON. Default: 0\n"
      << "  --bridges-csv <path>         Bridge edges CSV.\n"
      << "  --articulations-csv <path>   Articulation nodes CSV.\n"
      << "  --highlight-bridges <path>   Image overlay highlighting ALL bridge edges.\n"
      << "  --highlight-bypasses <path>  Image overlay highlighting suggested bypass paths.\n"
      << "  --scale <N>                  Nearest-neighbor upscale factor for images. Default: 4\n"
      << "  --write-best-save <path>     Write a save with the BEST bypass applied (if any).\n"
      << "\n";
}

struct BypassSuggestion {
  int bridgeEdge = -1;
  int fromNode = -1;
  int toNode = -1;
  int primaryCost = 0;
  int newTiles = 0;
  int steps = 0;
  std::vector<isocity::Point> path;
};

static int ClampI(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

static int CountNewRoadTiles(const isocity::World& world, const std::vector<isocity::Point>& path)
{
  int n = 0;
  for (const auto& p : path) {
    if (!world.inBounds(p.x, p.y)) continue;
    if (world.at(p.x, p.y).overlay != isocity::Overlay::Road) ++n;
  }
  return n;
}

static void ApplyRoadPath(isocity::World& world, const std::vector<isocity::Point>& path, int targetLevel)
{
  const int level = ClampI(targetLevel, 1, 3);
  for (const auto& p : path) {
    if (!world.inBounds(p.x, p.y)) continue;
    world.setRoad(p.x, p.y);
    world.at(p.x, p.y).level = static_cast<std::uint8_t>(level);
  }
  // Bulk edits: ensure masks are consistent even if setRoad() changed behavior later.
  world.recomputeRoadMasks();
}

static bool WriteBridgesCsv(const std::string& path, const isocity::RoadGraph& rg, const isocity::RoadGraphResilienceResult& res,
                            const isocity::RoadGraphTrafficResult* trafficAgg, std::string& err)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "Failed to open " + path;
    return false;
  }

  f << "edge,a,b,ax,ay,bx,by,length,tileCount,minorSideNodes,majorSideNodes,component";
  if (trafficAgg) {
    f << ",maxUtil,sumTrafficInterior,sumCapacityInterior,congestedTilesInterior,excessTrafficInterior";
  }
  f << "\n";

  for (const int ei : res.bridgeEdges) {
    if (ei < 0 || ei >= static_cast<int>(rg.edges.size())) continue;
    const isocity::RoadGraphEdge& e = rg.edges[static_cast<std::size_t>(ei)];
    const isocity::Point pa = (e.a >= 0 && e.a < static_cast<int>(rg.nodes.size())) ? rg.nodes[static_cast<std::size_t>(e.a)].pos : isocity::Point{-1, -1};
    const isocity::Point pb = (e.b >= 0 && e.b < static_cast<int>(rg.nodes.size())) ? rg.nodes[static_cast<std::size_t>(e.b)].pos : isocity::Point{-1, -1};

    const int sub = (ei < static_cast<int>(res.bridgeSubtreeNodes.size())) ? res.bridgeSubtreeNodes[static_cast<std::size_t>(ei)] : 0;
    const int oth = (ei < static_cast<int>(res.bridgeOtherNodes.size())) ? res.bridgeOtherNodes[static_cast<std::size_t>(ei)] : 0;
    const int minorSide = std::min(sub, oth);
    const int majorSide = std::max(sub, oth);

    int comp = -1;
    if (e.a >= 0 && e.a < static_cast<int>(res.nodeComponent.size())) comp = res.nodeComponent[static_cast<std::size_t>(e.a)];

    f << ei << "," << e.a << "," << e.b << "," << pa.x << "," << pa.y << "," << pb.x << "," << pb.y << ",";
    f << e.length << "," << e.tiles.size() << "," << minorSide << "," << majorSide << "," << comp;

    if (trafficAgg) {
      const isocity::RoadGraphTrafficEdgeStats& es = trafficAgg->edges[static_cast<std::size_t>(ei)];
      const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
      f << "," << u << "," << es.sumTrafficInterior << "," << es.sumCapacityInterior << "," << es.congestedTilesInterior << "," << es.excessTrafficInterior;
    }

    f << "\n";
  }

  return true;
}

static bool WriteArticulationsCsv(const std::string& path, const isocity::RoadGraph& rg, const isocity::RoadGraphResilienceResult& res,
                                  std::string& err)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "Failed to open " + path;
    return false;
  }

  f << "node,x,y,degree,component,componentSize\n";
  for (const int ni : res.articulationNodes) {
    if (ni < 0 || ni >= static_cast<int>(rg.nodes.size())) continue;
    const isocity::RoadGraphNode& n = rg.nodes[static_cast<std::size_t>(ni)];
    const int deg = static_cast<int>(n.edges.size());
    const int cid = (ni < static_cast<int>(res.nodeComponent.size())) ? res.nodeComponent[static_cast<std::size_t>(ni)] : -1;
    const int csz = (cid >= 0 && cid < static_cast<int>(res.componentSize.size())) ? res.componentSize[static_cast<std::size_t>(cid)] : 0;
    f << ni << "," << n.pos.x << "," << n.pos.y << "," << deg << "," << cid << "," << csz << "\n";
  }

  return true;
}

static void WriteJsonTiles(std::ostream& os, const std::vector<isocity::Point>& tiles)
{
  os << "[";
  for (std::size_t i = 0; i < tiles.size(); ++i) {
    if (i) os << ",";
    os << "[" << tiles[i].x << "," << tiles[i].y << "]";
  }
  os << "]";
}

static bool WriteJsonReport(const std::string& path,
                            const isocity::World& world,
                            const isocity::RoadGraph& rg,
                            const isocity::RoadGraphResilienceResult& res,
                            const isocity::TrafficResult* tr,
                            const isocity::RoadGraphTrafficResult* trafficAgg,
                            const std::vector<BypassSuggestion>& bypasses,
                            bool includeTiles,
                            std::string& err)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "Failed to open " + path;
    return false;
  }

  f << "{\n";

  // World summary.
  f << "  \"world\": {\"w\": " << world.width() << ", \"h\": " << world.height()
    << ", \"day\": " << world.stats().day << "},\n";
  f << "  \"roadGraph\": {\"nodes\": " << rg.nodes.size() << ", \"edges\": " << rg.edges.size() << "},\n";
  f << "  \"resilience\": {\"bridges\": " << res.bridgeEdges.size() << ", \"articulations\": " << res.articulationNodes.size() << "},\n";

  if (tr) {
    f << "  \"traffic\": {\"maxTileTraffic\": " << tr->maxTraffic
      << ", \"totalCommuters\": " << tr->totalCommuters
      << ", \"reachableCommuters\": " << tr->reachableCommuters
      << ", \"unreachableCommuters\": " << tr->unreachableCommuters
      << ", \"avgCommute\": " << tr->avgCommute
      << ", \"p95Commute\": " << tr->p95Commute
      << ", \"congestion\": " << tr->congestion
      << ", \"routing\": \"" << (tr->usedCongestionAwareRouting ? "congestionAware" : "classic") << "\"},\n";
  }

  // Bridges.
  f << "  \"bridges\": [\n";
  for (std::size_t i = 0; i < res.bridgeEdges.size(); ++i) {
    const int ei = res.bridgeEdges[i];
    if (ei < 0 || ei >= static_cast<int>(rg.edges.size())) continue;
    const isocity::RoadGraphEdge& e = rg.edges[static_cast<std::size_t>(ei)];

    const int sub = (ei < static_cast<int>(res.bridgeSubtreeNodes.size())) ? res.bridgeSubtreeNodes[static_cast<std::size_t>(ei)] : 0;
    const int oth = (ei < static_cast<int>(res.bridgeOtherNodes.size())) ? res.bridgeOtherNodes[static_cast<std::size_t>(ei)] : 0;
    const int minorSide = std::min(sub, oth);
    const int majorSide = std::max(sub, oth);

    const isocity::Point pa = (e.a >= 0 && e.a < static_cast<int>(rg.nodes.size())) ? rg.nodes[static_cast<std::size_t>(e.a)].pos : isocity::Point{-1, -1};
    const isocity::Point pb = (e.b >= 0 && e.b < static_cast<int>(rg.nodes.size())) ? rg.nodes[static_cast<std::size_t>(e.b)].pos : isocity::Point{-1, -1};

    f << "    {\"edge\": " << ei
      << ", \"a\": " << e.a << ", \"b\": " << e.b
      << ", \"ax\": " << pa.x << ", \"ay\": " << pa.y
      << ", \"bx\": " << pb.x << ", \"by\": " << pb.y
      << ", \"length\": " << e.length
      << ", \"minorSideNodes\": " << minorSide
      << ", \"majorSideNodes\": " << majorSide;

    if (trafficAgg) {
      const isocity::RoadGraphTrafficEdgeStats& es = trafficAgg->edges[static_cast<std::size_t>(ei)];
      const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
      f << ", \"maxUtil\": " << u
        << ", \"sumTrafficInterior\": " << es.sumTrafficInterior
        << ", \"sumCapacityInterior\": " << es.sumCapacityInterior
        << ", \"congestedTilesInterior\": " << es.congestedTilesInterior
        << ", \"excessTrafficInterior\": " << es.excessTrafficInterior;
    }

    if (includeTiles) {
      f << ", \"tiles\": ";
      WriteJsonTiles(f, e.tiles);
    }

    f << "}";
    if (i + 1 < res.bridgeEdges.size()) f << ",";
    f << "\n";
  }
  f << "  ],\n";

  // Articulations.
  f << "  \"articulations\": [\n";
  for (std::size_t i = 0; i < res.articulationNodes.size(); ++i) {
    const int ni = res.articulationNodes[i];
    if (ni < 0 || ni >= static_cast<int>(rg.nodes.size())) continue;
    const isocity::RoadGraphNode& n = rg.nodes[static_cast<std::size_t>(ni)];
    const int deg = static_cast<int>(n.edges.size());
    const int cid = (ni < static_cast<int>(res.nodeComponent.size())) ? res.nodeComponent[static_cast<std::size_t>(ni)] : -1;
    f << "    {\"node\": " << ni
      << ", \"x\": " << n.pos.x << ", \"y\": " << n.pos.y
      << ", \"degree\": " << deg
      << ", \"component\": " << cid
      << "}";
    if (i + 1 < res.articulationNodes.size()) f << ",";
    f << "\n";
  }
  f << "  ],\n";

  // Bypass suggestions.
  f << "  \"bypasses\": [\n";
  for (std::size_t i = 0; i < bypasses.size(); ++i) {
    const BypassSuggestion& b = bypasses[i];
    f << "    {\"bridgeEdge\": " << b.bridgeEdge
      << ", \"fromNode\": " << b.fromNode
      << ", \"toNode\": " << b.toNode
      << ", \"primaryCost\": " << b.primaryCost
      << ", \"newTiles\": " << b.newTiles
      << ", \"steps\": " << b.steps
      << ", \"path\": ";
    WriteJsonTiles(f, b.path);
    f << "}";
    if (i + 1 < bypasses.size()) f << ",";
    f << "\n";
  }
  f << "  ]\n";

  f << "}\n";
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string jsonPath;
  std::string bridgesCsvPath;
  std::string articulationsCsvPath;
  std::string highlightBridgesPath;
  std::string highlightBypassesPath;
  std::string writeBestSavePath;

  int w = 128;
  int h = 128;
  std::uint64_t seed = 1;
  int days = 60;

  bool requireOutside = true;

  // Traffic (optional)
  bool doTraffic = true;
  TrafficConfig tcfg{};
  int baseCapacity = tcfg.roadTileCapacity;
  bool useRoadLevelCapacity = true;

  bool congestionAware = false;
  int passes = tcfg.congestionIterations;
  float alpha = tcfg.congestionAlpha;
  float beta = tcfg.congestionBeta;
  float capScale = tcfg.congestionCapacityScale;
  float ratioClamp = tcfg.congestionRatioClamp;

  // Bypass suggestions
  bool suggestBypasses = true;
  int bypassTop = 10;
  bool bypassMoney = false;
  int bypassTargetLevel = 1;
  bool bypassAllowBridges = false;
  int bypassMaxCost = 0;

  bool includeTiles = false;
  int scale = 4;

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
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val, argc, argv) || !ParseSize(val, &w, &h)) {
        std::cerr << "--size requires WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--days") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &days) || days < 0) {
        std::cerr << "--days requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &requireOutside)) {
        std::cerr << "--require-outside requires 0|1\n";
        return 2;
      }
    } else if (arg == "--traffic") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &doTraffic)) {
        std::cerr << "--traffic requires 0|1\n";
        return 2;
      }
    } else if (arg == "--traffic-capacity") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &baseCapacity) || baseCapacity <= 0) {
        std::cerr << "--traffic-capacity requires an integer > 0\n";
        return 2;
      }
    } else if (arg == "--use-road-level-cap") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &useRoadLevelCapacity)) {
        std::cerr << "--use-road-level-cap requires 0|1\n";
        return 2;
      }
    } else if (arg == "--congestion-aware") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &congestionAware)) {
        std::cerr << "--congestion-aware requires 0|1\n";
        return 2;
      }
    } else if (arg == "--passes") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &passes) || passes < 1) {
        std::cerr << "--passes requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--alpha") {
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &alpha) || alpha < 0.0f) {
        std::cerr << "--alpha requires a float >= 0\n";
        return 2;
      }
    } else if (arg == "--beta") {
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &beta) || beta < 0.0f) {
        std::cerr << "--beta requires a float >= 0\n";
        return 2;
      }
    } else if (arg == "--cap-scale") {
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &capScale) || capScale <= 0.0f) {
        std::cerr << "--cap-scale requires a float > 0\n";
        return 2;
      }
    } else if (arg == "--ratio-clamp") {
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &ratioClamp) || ratioClamp <= 0.0f) {
        std::cerr << "--ratio-clamp requires a float > 0\n";
        return 2;
      }
    } else if (arg == "--suggest-bypasses") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &suggestBypasses)) {
        std::cerr << "--suggest-bypasses requires 0|1\n";
        return 2;
      }
    } else if (arg == "--bypass-top") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &bypassTop) || bypassTop < 0) {
        std::cerr << "--bypass-top requires an integer >= 0\n";
        return 2;
      }
    } else if (arg == "--bypass-money") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &bypassMoney)) {
        std::cerr << "--bypass-money requires 0|1\n";
        return 2;
      }
    } else if (arg == "--bypass-target-level") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &bypassTargetLevel)) {
        std::cerr << "--bypass-target-level requires an integer\n";
        return 2;
      }
      bypassTargetLevel = ClampI(bypassTargetLevel, 1, 3);
    } else if (arg == "--bypass-allow-bridges") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &bypassAllowBridges)) {
        std::cerr << "--bypass-allow-bridges requires 0|1\n";
        return 2;
      }
    } else if (arg == "--bypass-max-cost") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &bypassMaxCost) || bypassMaxCost < 0) {
        std::cerr << "--bypass-max-cost requires an integer >= 0\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      jsonPath = val;
    } else if (arg == "--include-tiles") {
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &includeTiles)) {
        std::cerr << "--include-tiles requires 0|1\n";
        return 2;
      }
    } else if (arg == "--bridges-csv") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--bridges-csv requires a path\n";
        return 2;
      }
      bridgesCsvPath = val;
    } else if (arg == "--articulations-csv") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--articulations-csv requires a path\n";
        return 2;
      }
      articulationsCsvPath = val;
    } else if (arg == "--highlight-bridges") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--highlight-bridges requires a path\n";
        return 2;
      }
      highlightBridgesPath = val;
    } else if (arg == "--highlight-bypasses") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--highlight-bypasses requires a path\n";
        return 2;
      }
      highlightBypassesPath = val;
    } else if (arg == "--scale") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &scale) || scale < 1) {
        std::cerr << "--scale requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--write-best-save") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--write-best-save requires a path\n";
        return 2;
      }
      writeBestSavePath = val;
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
    std::string loadErr;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, loadErr)) {
      std::cerr << "Failed to load save: " << loadPath << "\n" << loadErr << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  simCfg.requireOutsideConnection = requireOutside;

  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }
  if (days == 0) sim.refreshDerivedStats(world);

  const RoadGraph rg = BuildRoadGraph(world);
  const RoadGraphResilienceResult res = ComputeRoadGraphResilience(rg);

  std::cout << "RoadResilience summary\n";
  std::cout << "  world: " << world.width() << "x" << world.height() << " day=" << world.stats().day << "\n";
  std::cout << "  roadGraph: nodes=" << rg.nodes.size() << " edges=" << rg.edges.size() << "\n";
  std::cout << "  bridges=" << res.bridgeEdges.size() << " articulations=" << res.articulationNodes.size() << "\n";

  TrafficResult tr;
  RoadGraphTrafficResult trafficAgg;
  bool haveTraffic = false;

  if (doTraffic) {
    tcfg.requireOutsideConnection = requireOutside;
    tcfg.roadTileCapacity = baseCapacity;
    tcfg.congestionAwareRouting = congestionAware;
    tcfg.congestionIterations = passes;
    tcfg.congestionAlpha = alpha;
    tcfg.congestionBeta = beta;
    tcfg.congestionCapacityScale = capScale;
    tcfg.congestionRatioClamp = ratioClamp;

    float employedShare = 1.0f;
    const Stats& s = world.stats();
    if (s.population > 0) employedShare = static_cast<float>(s.employed) / static_cast<float>(s.population);

    tr = ComputeCommuteTraffic(world, tcfg, employedShare);

    RoadGraphTrafficConfig agCfg{};
    agCfg.baseTileCapacity = baseCapacity;
    agCfg.useRoadLevelCapacity = useRoadLevelCapacity;
    trafficAgg = AggregateTrafficOnRoadGraph(world, rg, tr, agCfg);

    haveTraffic = true;

    std::cout << "  traffic: maxTileTraffic=" << tr.maxTraffic
              << " routing=" << (tr.usedCongestionAwareRouting ? "congestionAware" : "classic")
              << " passes=" << tr.routingPasses << "\n";
  }

  // Precompute node index lookup by tile for suggestions (and JSON).
  const int W = world.width();
  const int H = world.height();
  std::vector<int> nodeAt(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), -1);
  for (int ni = 0; ni < static_cast<int>(rg.nodes.size()); ++ni) {
    const Point p = rg.nodes[static_cast<std::size_t>(ni)].pos;
    if (!world.inBounds(p.x, p.y)) continue;
    nodeAt[static_cast<std::size_t>(p.y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(p.x)] = ni;
  }

  // Rank bridges.
  std::vector<int> bridges = res.bridgeEdges;
  auto bridgeScore = [&](int ei) -> double {
    if (haveTraffic) {
      const RoadGraphTrafficEdgeStats& es = trafficAgg.edges[static_cast<std::size_t>(ei)];
      const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
      return u;
    }
    const int sub = (ei < static_cast<int>(res.bridgeSubtreeNodes.size())) ? res.bridgeSubtreeNodes[static_cast<std::size_t>(ei)] : 0;
    const int oth = (ei < static_cast<int>(res.bridgeOtherNodes.size())) ? res.bridgeOtherNodes[static_cast<std::size_t>(ei)] : 0;
    return static_cast<double>(std::min(sub, oth));
  };
  std::sort(bridges.begin(), bridges.end(), [&](int a, int b) {
    const double sa = bridgeScore(a);
    const double sb = bridgeScore(b);
    if (sa != sb) return sa > sb;
    return a < b;
  });

  const int topPrint = std::min<int>(10, static_cast<int>(bridges.size()));
  std::cout << "  top " << topPrint << " bridges by " << (haveTraffic ? "traffic utilization" : "cut size") << ":\n";
  for (int i = 0; i < topPrint; ++i) {
    const int ei = bridges[static_cast<std::size_t>(i)];
    const RoadGraphEdge& e = rg.edges[static_cast<std::size_t>(ei)];
    const int sub = res.bridgeSubtreeNodes[static_cast<std::size_t>(ei)];
    const int oth = res.bridgeOtherNodes[static_cast<std::size_t>(ei)];
    std::cout << "    bridge " << ei << " (" << e.a << "--" << e.b << ") len=" << e.length
              << " cut=" << std::min(sub, oth) << "/" << std::max(sub, oth);
    if (haveTraffic) {
      const RoadGraphTrafficEdgeStats& es = trafficAgg.edges[static_cast<std::size_t>(ei)];
      const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
      std::cout << " maxUtil=" << u;
    }
    std::cout << "\n";
  }

  // Suggest bypasses.
  std::vector<BypassSuggestion> bypasses;
  if (suggestBypasses && !bridges.empty()) {
    const int topK = (bypassTop > 0) ? std::min<int>(bypassTop, static_cast<int>(bridges.size()))
                                    : static_cast<int>(bridges.size());

    RoadBuildPathConfig pcfg{};
    pcfg.allowBridges = bypassAllowBridges;
    pcfg.targetLevel = bypassTargetLevel;
    pcfg.costModel = bypassMoney ? RoadBuildPathConfig::CostModel::Money : RoadBuildPathConfig::CostModel::NewTiles;

    bypasses.reserve(static_cast<std::size_t>(topK));

    for (int i = 0; i < topK; ++i) {
      const int bridgeEdge = bridges[static_cast<std::size_t>(i)];

      RoadGraphBridgeCut cut;
      if (!ComputeRoadGraphBridgeCut(rg, bridgeEdge, cut)) continue;

      // Prefer starting from the smaller side (less multi-source seeding).
      std::vector<int>* sideS = &cut.sideA;
      std::vector<int>* sideG = &cut.sideB;
      if (cut.sideA.size() > cut.sideB.size()) {
        sideS = &cut.sideB;
        sideG = &cut.sideA;
      }

      std::vector<Point> starts;
      std::vector<Point> goals;
      starts.reserve(sideS->size());
      goals.reserve(sideG->size());

      for (const int ni : *sideS) {
        if (ni < 0 || ni >= static_cast<int>(rg.nodes.size())) continue;
        starts.push_back(rg.nodes[static_cast<std::size_t>(ni)].pos);
      }
      for (const int ni : *sideG) {
        if (ni < 0 || ni >= static_cast<int>(rg.nodes.size())) continue;
        goals.push_back(rg.nodes[static_cast<std::size_t>(ni)].pos);
      }

      const std::vector<std::uint64_t> blockedMoves = BuildBlockedMovesForRoadGraphEdge(rg, bridgeEdge, world.width());

      std::vector<Point> path;
      int primaryCost = 0;
      const bool ok = FindRoadBuildPathBetweenSets(world, starts, goals, path, &primaryCost, pcfg, &blockedMoves, bypassMaxCost);

      if (!ok || path.empty()) continue;

      BypassSuggestion s;
      s.bridgeEdge = bridgeEdge;
      s.primaryCost = primaryCost;
      s.steps = static_cast<int>(path.size()) - 1;
      s.newTiles = CountNewRoadTiles(world, path);
      s.path = path;

      const Point p0 = path.front();
      const Point p1 = path.back();
      if (world.inBounds(p0.x, p0.y)) {
        s.fromNode = nodeAt[static_cast<std::size_t>(p0.y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(p0.x)];
      }
      if (world.inBounds(p1.x, p1.y)) {
        s.toNode = nodeAt[static_cast<std::size_t>(p1.y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(p1.x)];
      }

      bypasses.push_back(std::move(s));
    }

    std::cout << "  bypass suggestions: " << bypasses.size() << " / " << topK << "\n";
    const int printN = std::min<int>(5, static_cast<int>(bypasses.size()));
    for (int i = 0; i < printN; ++i) {
      const BypassSuggestion& s = bypasses[static_cast<std::size_t>(i)];
      std::cout << "    bypass for bridge " << s.bridgeEdge
                << " cost=" << s.primaryCost
                << " newTiles=" << s.newTiles
                << " steps=" << s.steps
                << " fromNode=" << s.fromNode
                << " toNode=" << s.toNode << "\n";
    }
  }

  // Exports.
  std::string err;
  if (!bridgesCsvPath.empty()) {
    if (!WriteBridgesCsv(bridgesCsvPath, rg, res, haveTraffic ? &trafficAgg : nullptr, err)) {
      std::cerr << "Failed to write bridges CSV: " << bridgesCsvPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!articulationsCsvPath.empty()) {
    if (!WriteArticulationsCsv(articulationsCsvPath, rg, res, err)) {
      std::cerr << "Failed to write articulations CSV: " << articulationsCsvPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!jsonPath.empty()) {
    if (!WriteJsonReport(jsonPath, world, rg, res, haveTraffic ? &tr : nullptr, haveTraffic ? &trafficAgg : nullptr, bypasses, includeTiles, err)) {
      std::cerr << "Failed to write JSON: " << jsonPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!highlightBridgesPath.empty()) {
    PpmImage img = RenderPpmLayer(world, ExportLayer::Overlay, nullptr, nullptr, nullptr);

    // Paint every bridge edge in red.
    for (const int ei : res.bridgeEdges) {
      if (ei < 0 || ei >= static_cast<int>(rg.edges.size())) continue;
      const RoadGraphEdge& e = rg.edges[static_cast<std::size_t>(ei)];
      for (const Point& p : e.tiles) {
        SetPixel(img, p.x, p.y, 255, 40, 40);
      }
      // Endpoints.
      if (e.a >= 0 && e.a < static_cast<int>(rg.nodes.size())) {
        const Point p = rg.nodes[static_cast<std::size_t>(e.a)].pos;
        SetPixel(img, p.x, p.y, 255, 235, 60);
      }
      if (e.b >= 0 && e.b < static_cast<int>(rg.nodes.size())) {
        const Point p = rg.nodes[static_cast<std::size_t>(e.b)].pos;
        SetPixel(img, p.x, p.y, 255, 235, 60);
      }
    }

    img = ScaleNearest(img, scale);
    if (!WriteImageAuto(highlightBridgesPath, img, err)) {
      std::cerr << "Failed to write highlight image: " << highlightBridgesPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!highlightBypassesPath.empty()) {
    PpmImage img = RenderPpmLayer(world, ExportLayer::Overlay, nullptr, nullptr, nullptr);

    // Paint bridge edges faintly, then bypasses in cyan.
    for (const int ei : res.bridgeEdges) {
      if (ei < 0 || ei >= static_cast<int>(rg.edges.size())) continue;
      const RoadGraphEdge& e = rg.edges[static_cast<std::size_t>(ei)];
      for (const Point& p : e.tiles) {
        SetPixel(img, p.x, p.y, 255, 80, 80);
      }
    }

    for (const BypassSuggestion& s : bypasses) {
      for (const Point& p : s.path) {
        SetPixel(img, p.x, p.y, 40, 220, 255);
      }
    }

    img = ScaleNearest(img, scale);
    if (!WriteImageAuto(highlightBypassesPath, img, err)) {
      std::cerr << "Failed to write bypass highlight image: " << highlightBypassesPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!writeBestSavePath.empty()) {
    if (bypasses.empty()) {
      std::cerr << "No bypass suggestions available; not writing save.\n";
    } else {
      ApplyRoadPath(world, bypasses[0].path, bypassTargetLevel);
      std::string saveErr;
      if (!SaveWorldBinary(world, procCfg, simCfg, writeBestSavePath, saveErr)) {
        std::cerr << "Failed to write save: " << writeBestSavePath << "\n" << saveErr << "\n";
        return 2;
      }
      std::cout << "Wrote best-bypass save: " << writeBestSavePath << "\n";
    }
  }

  return 0;
}
