#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/RoadGraphTrafficExport.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
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
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_trafficgraph (headless commute traffic + road graph aggregation)\n\n"
      << "Generates (or loads) a world, optionally steps the simulation for N days to populate zones,\n"
      << "computes a commute traffic heatmap (per road tile), then aggregates traffic onto the compressed\n"
      << "RoadGraph (nodes/intersections + edges/segments) and exports DOT/JSON/CSV.\n\n"
      << "Usage:\n"
      << "  proc_isocity_trafficgraph [--seed N] [--size WxH] [--days N] [--load save.bin] [outputs...]\n\n"
      << "Inputs:\n"
      << "  --load <path>          Load a save instead of generating a new world.\n"
      << "  --seed <u64>           World seed (decimal or 0x...). Default: 1\n"
      << "  --size <WxH>           World size when generating. Default: 128x128\n"
      << "  --days <N>             Step the simulation N days before computing traffic. Default: 60\n"
      << "\n"
      << "Traffic config:\n"
      << "  --require-outside <0|1>   Outside connection rule. Default: 1\n"
      << "  --traffic-capacity <N>    Base road tile capacity (street). Default: 28\n"
      << "  --use-road-level-cap <0|1>  If 1, scale capacity by road class (Tile::level). Default: 1\n"
      << "  --include-commercial <0|1> Include commercial jobs as commute targets. Default: 1\n"
      << "  --include-industrial <0|1> Include industrial jobs as commute targets. Default: 1\n"
      << "  --employed-share <f>       Override commuter share (0..1). Default: employed/pop if available, else 1\n"
      << "\n"
      << "Congestion-aware routing (optional):\n"
      << "  --congestion-aware <0|1> Enable multi-pass assignment. Default: 0\n"
      << "  --passes <N>              Number of routing passes. Default: 4\n"
      << "  --alpha <f>               Congestion curve alpha. Default: 0.15\n"
      << "  --beta <f>                Congestion curve beta. Default: 4.0\n"
      << "  --cap-scale <f>           Capacity scale for congestion cost. Default: 1.0\n"
      << "  --ratio-clamp <f>         Clamp v/c in congestion curve. Default: 3.0\n"
      << "\n"
      << "Capacity-aware job assignment (optional):\n"
      << "  --capacity-aware-jobs <0|1>  Enable capacity-aware job assignment. Default: 0\n"
      << "  --job-iters <N>              Penalty fitting iterations. Default: 6\n"
      << "  --job-penalty <N>            Penalty base (milli). Default: 8000\n"
      << "\n"
      << "Outputs:\n"
      << "  --dot <path>          GraphViz DOT (edges colored by utilization).\n"
      << "  --json <path>         JSON export (nodes/edges + traffic stats).\n"
      << "  --nodes-csv <path>    Node stats CSV.\n"
      << "  --edges-csv <path>    Edge stats CSV.\n"
      << "  --include-tiles <0|1> Include per-edge tile polylines in JSON. Default: 0\n"
      << "\n"
      << "Images:\n"
      << "  --heatmap <path>      Traffic heatmap image (PPM/PNG by extension).\n"
      << "  --highlight <path>    Overlay image highlighting the most congested edges.\n"
      << "  --highlight-top <N>   Highlight top N edges by max utilization. Default: 20\n"
      << "  --scale <N>           Nearest-neighbor upscale factor for images. Default: 4\n"
      << "\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string dotPath;
  std::string jsonPath;
  std::string nodesCsvPath;
  std::string edgesCsvPath;

  std::string heatmapPath;
  std::string highlightPath;

  bool includeTiles = false;
  int scale = 4;
  int highlightTop = 20;

  int w = 128;
  int h = 128;
  std::uint64_t seed = 1;

  int days = 60;

  // Traffic settings
  TrafficConfig tcfg{};
  bool requireOutside = true;
  int baseCapacity = tcfg.roadTileCapacity;
  bool useRoadLevelCapacity = true;
  bool includeCommercial = true;
  bool includeIndustrial = true;

  bool employedShareSet = false;
  float employedShareOverride = 1.0f;

  // Congestion-aware routing
  bool congestionAware = false;
  int passes = tcfg.congestionIterations;
  float alpha = tcfg.congestionAlpha;
  float beta = tcfg.congestionBeta;
  float capScale = tcfg.congestionCapacityScale;
  float ratioClamp = tcfg.congestionRatioClamp;

  // Capacity-aware job assignment
  bool capacityAwareJobs = false;
  int jobIters = tcfg.jobAssignmentIterations;
  int jobPenalty = tcfg.jobPenaltyBaseMilli;

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
    } else if (arg == "--dot") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--dot requires a path\n";
        return 2;
      }
      dotPath = val;
    } else if (arg == "--json") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      jsonPath = val;
    } else if (arg == "--nodes-csv") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--nodes-csv requires a path\n";
        return 2;
      }
      nodesCsvPath = val;
    } else if (arg == "--edges-csv") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--edges-csv requires a path\n";
        return 2;
      }
      edgesCsvPath = val;
    } else if (arg == "--include-tiles") {
      bool b = false;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--include-tiles requires 0 or 1\n";
        return 2;
      }
      includeTiles = b;
    } else if (arg == "--heatmap") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--heatmap requires a path\n";
        return 2;
      }
      heatmapPath = val;
    } else if (arg == "--highlight") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--highlight requires a path\n";
        return 2;
      }
      highlightPath = val;
    } else if (arg == "--highlight-top") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &highlightTop) || highlightTop < 0) {
        std::cerr << "--highlight-top requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--scale") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &scale) || scale <= 0) {
        std::cerr << "--scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      requireOutside = b;
    } else if (arg == "--traffic-capacity") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &baseCapacity) || baseCapacity < 0) {
        std::cerr << "--traffic-capacity requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--use-road-level-cap") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--use-road-level-cap requires 0 or 1\n";
        return 2;
      }
      useRoadLevelCapacity = b;
    } else if (arg == "--include-commercial") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--include-commercial requires 0 or 1\n";
        return 2;
      }
      includeCommercial = b;
    } else if (arg == "--include-industrial") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--include-industrial requires 0 or 1\n";
        return 2;
      }
      includeIndustrial = b;
    } else if (arg == "--employed-share") {
      float f = 1.0f;
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &f)) {
        std::cerr << "--employed-share requires a float\n";
        return 2;
      }
      employedShareSet = true;
      employedShareOverride = f;
    } else if (arg == "--congestion-aware") {
      bool b = false;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--congestion-aware requires 0 or 1\n";
        return 2;
      }
      congestionAware = b;
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
    } else if (arg == "--capacity-aware-jobs") {
      bool b = false;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--capacity-aware-jobs requires 0 or 1\n";
        return 2;
      }
      capacityAwareJobs = b;
    } else if (arg == "--job-iters") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &jobIters) || jobIters < 1) {
        std::cerr << "--job-iters requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--job-penalty") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &jobPenalty) || jobPenalty < 0) {
        std::cerr << "--job-penalty requires an integer >= 0\n";
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

  // Load/generate
  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  // Respect the requested outside-connection rule for both sim and traffic.
  simCfg.requireOutsideConnection = requireOutside;

  // Optionally simulate some days to populate zones.
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }
  if (days == 0) {
    // Ensure derived stats are populated even when we didn't tick.
    sim.refreshDerivedStats(world);
  }

  // Build traffic config.
  tcfg.requireOutsideConnection = requireOutside;
  tcfg.roadTileCapacity = baseCapacity;
  tcfg.includeCommercialJobs = includeCommercial;
  tcfg.includeIndustrialJobs = includeIndustrial;

  tcfg.congestionAwareRouting = congestionAware;
  tcfg.congestionIterations = passes;
  tcfg.congestionAlpha = alpha;
  tcfg.congestionBeta = beta;
  tcfg.congestionCapacityScale = capScale;
  tcfg.congestionRatioClamp = ratioClamp;

  tcfg.capacityAwareJobs = capacityAwareJobs;
  tcfg.jobAssignmentIterations = jobIters;
  tcfg.jobPenaltyBaseMilli = jobPenalty;

  float employedShare = 1.0f;
  if (employedShareSet) {
    employedShare = employedShareOverride;
  } else {
    const Stats& s = world.stats();
    if (s.population > 0) {
      employedShare = static_cast<float>(s.employed) / static_cast<float>(s.population);
    }
  }

  // Compute traffic heatmap.
  const TrafficResult tr = ComputeCommuteTraffic(world, tcfg, employedShare);

  // Build road graph and aggregate.
  const RoadGraph rg = BuildRoadGraph(world);
  RoadGraphTrafficConfig agCfg{};
  agCfg.baseTileCapacity = baseCapacity;
  agCfg.useRoadLevelCapacity = useRoadLevelCapacity;
  const RoadGraphTrafficResult agg = AggregateTrafficOnRoadGraph(world, rg, tr, agCfg);

  std::cout << "TrafficGraph summary\n";
  std::cout << "  world: " << world.width() << "x" << world.height() << "  day=" << world.stats().day << "\n";
  std::cout << "  pop=" << world.stats().population << " employed=" << world.stats().employed << " employedShare=" << employedShare << "\n";
  std::cout << "  roadGraph: nodes=" << rg.nodes.size() << " edges=" << rg.edges.size() << "\n";
  std::cout << "  traffic: maxTileTraffic=" << tr.maxTraffic
            << " routing=" << (tr.usedCongestionAwareRouting ? "congestionAware" : "classic")
            << " passes=" << tr.routingPasses
            << " jobCap=" << (tr.usedCapacityAwareJobs ? "on" : "off");
  if (tr.usedCapacityAwareJobs) {
    std::cout << " jobIters=" << tr.jobAssignmentIterations << " maxJobOver=" << tr.maxJobSourceOverload;
  }
  std::cout << "\n";

  // Rank edges by congestion.
  std::vector<int> edgeOrder;
  edgeOrder.reserve(agg.edges.size());
  for (int i = 0; i < static_cast<int>(agg.edges.size()); ++i) edgeOrder.push_back(i);
  std::sort(edgeOrder.begin(), edgeOrder.end(), [&](int a, int b) {
    auto utilOf = [&](int ei) {
      const RoadGraphTrafficEdgeStats& es = agg.edges[static_cast<std::size_t>(ei)];
      return (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
    };
    return utilOf(a) > utilOf(b);
  });

  const int topPrint = std::min<int>(10, static_cast<int>(edgeOrder.size()));
  std::cout << "  top " << topPrint << " edges by max utilization:\n";
  for (int i = 0; i < topPrint; ++i) {
    const int ei = edgeOrder[static_cast<std::size_t>(i)];
    const RoadGraphTrafficEdgeStats& es = agg.edges[static_cast<std::size_t>(ei)];
    const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
    std::cout << "    edge " << ei << " (" << es.a << "--" << es.b << ") len=" << es.length << " maxUtil=" << u
              << " sumTrafficInterior=" << es.sumTrafficInterior << "\n";
  }

  // Exports.
  std::string err;
  if (!dotPath.empty()) {
    RoadGraphTrafficExportConfig dotCfg{};
    if (!ExportRoadGraphTrafficDot(dotPath, rg, agg, dotCfg, &err)) {
      std::cerr << "Failed to write DOT: " << dotPath << "\n" << err << "\n";
      return 2;
    }
  }
  if (!jsonPath.empty()) {
    if (!ExportRoadGraphTrafficJson(jsonPath, rg, agg, includeTiles, &err)) {
      std::cerr << "Failed to write JSON: " << jsonPath << "\n" << err << "\n";
      return 2;
    }
  }
  if (!nodesCsvPath.empty()) {
    if (!ExportRoadGraphTrafficNodesCsv(nodesCsvPath, agg, &err)) {
      std::cerr << "Failed to write nodes CSV: " << nodesCsvPath << "\n" << err << "\n";
      return 2;
    }
  }
  if (!edgesCsvPath.empty()) {
    if (!ExportRoadGraphTrafficEdgesCsv(edgesCsvPath, agg, &err)) {
      std::cerr << "Failed to write edges CSV: " << edgesCsvPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!heatmapPath.empty()) {
    PpmImage img = RenderPpmLayer(world, ExportLayer::Traffic, nullptr, &tr, nullptr);
    img = ScaleNearest(img, scale);
    if (!WriteImageAuto(heatmapPath, img, err)) {
      std::cerr << "Failed to write heatmap image: " << heatmapPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!highlightPath.empty()) {
    // Base: overlay layer, then paint the top congested edges in red.
    PpmImage img = RenderPpmLayer(world, ExportLayer::Overlay, nullptr, nullptr, nullptr);

    const int k = (highlightTop > 0) ? std::min<int>(highlightTop, static_cast<int>(edgeOrder.size()))
                                     : static_cast<int>(edgeOrder.size());

    for (int i = 0; i < k; ++i) {
      const int ei = edgeOrder[static_cast<std::size_t>(i)];
      if (ei < 0 || ei >= static_cast<int>(rg.edges.size())) continue;
      const RoadGraphEdge& e = rg.edges[static_cast<std::size_t>(ei)];
      for (const Point& p : e.tiles) {
        SetPixel(img, p.x, p.y, 255, 40, 40);
      }
      // Highlight endpoints.
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
    if (!WriteImageAuto(highlightPath, img, err)) {
      std::cerr << "Failed to write highlight image: " << highlightPath << "\n" << err << "\n";
      return 2;
    }
  }

  return 0;
}
