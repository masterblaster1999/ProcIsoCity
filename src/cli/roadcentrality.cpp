#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphCentrality.hpp"
#include "isocity/RoadGraphCentralityExport.hpp"
#include "isocity/RoadGraphExport.hpp"
#include "isocity/SaveLoad.hpp"

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

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

bool ParseWeightMode(const std::string& s, isocity::RoadGraphEdgeWeightMode* out)
{
  if (!out) return false;
  if (s == "steps" || s == "len" || s == "length") {
    *out = isocity::RoadGraphEdgeWeightMode::Steps;
    return true;
  }
  if (s == "time" || s == "travel" || s == "travel_time" || s == "traveltime") {
    *out = isocity::RoadGraphEdgeWeightMode::TravelTimeMilli;
    return true;
  }
  return false;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_roadcentrality (headless road network centrality)\n\n"
      << "Computes betweenness centrality (nodes + edges) on the compressed RoadGraph and exports\n"
      << "results to DOT/JSON/CSV. Optionally renders a 1-pixel-per-tile heatmap highlighting\n"
      << "the top central nodes/edges on top of a chosen base layer.\n\n"
      << "Usage:\n"
      << "  proc_isocity_roadcentrality [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                           [--weight-mode <steps|time>] [--max-sources <N>]\n"
      << "                           [--dot <out.dot>] [--json <out.json>]\n"
      << "                           [--nodes-csv <out.csv>] [--edges-csv <out.csv>]\n"
      << "                           [--include-tiles <0|1>] [--color-components <0|1>]\n"
      << "                           [--ppm <out.ppm|out.png>] [--ppm-scale <N>]\n"
      << "                           [--top-nodes <N>] [--top-edges <N>]\n\n"
      << "Notes:\n"
      << "  - If --load is omitted, a world is generated from (--seed, --size).\n"
      << "  - --weight-mode=time weights edges by road travel time (Street/Avenue/Highway).\n"
      << "  - --max-sources allows approximate betweenness via deterministic source sampling.\n"
      << "    (Closeness metrics are only computed when all sources are processed.)\n";
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
  std::string ppmPath;

  bool includeTiles = false;
  bool colorComponents = true;
  int ppmScale = 4;
  int topNodes = 20;
  int topEdges = 30;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  RoadGraphCentralityConfig ccfg;
  ccfg.weightMode = RoadGraphEdgeWeightMode::Steps;
  ccfg.maxSources = 0;
  ccfg.scaleSampleToFull = true;
  ccfg.undirected = true;
  ccfg.normalizeBetweenness = true;
  ccfg.closenessComponentScale = true;

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
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &w, &h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--weight-mode") {
      if (!requireValue(i, val) || !ParseWeightMode(val, &ccfg.weightMode)) {
        std::cerr << "--weight-mode requires: steps|time\n";
        return 2;
      }
    } else if (arg == "--max-sources") {
      if (!requireValue(i, val) || !ParseI32(val, &ccfg.maxSources) || ccfg.maxSources < 0) {
        std::cerr << "--max-sources requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--scale-sample") {
      if (!requireValue(i, val) || !ParseBool01(val, &ccfg.scaleSampleToFull)) {
        std::cerr << "--scale-sample requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--normalize") {
      if (!requireValue(i, val) || !ParseBool01(val, &ccfg.normalizeBetweenness)) {
        std::cerr << "--normalize requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--dot") {
      if (!requireValue(i, val)) {
        std::cerr << "--dot requires a path\n";
        return 2;
      }
      dotPath = val;
    } else if (arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      jsonPath = val;
    } else if (arg == "--nodes-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--nodes-csv requires a path\n";
        return 2;
      }
      nodesCsvPath = val;
    } else if (arg == "--edges-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--edges-csv requires a path\n";
        return 2;
      }
      edgesCsvPath = val;
    } else if (arg == "--include-tiles") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeTiles)) {
        std::cerr << "--include-tiles requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--color-components") {
      if (!requireValue(i, val) || !ParseBool01(val, &colorComponents)) {
        std::cerr << "--color-components requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--ppm") {
      if (!requireValue(i, val)) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
      ppmPath = val;
    } else if (arg == "--ppm-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &ppmScale) || ppmScale <= 0) {
        std::cerr << "--ppm-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--top-nodes") {
      if (!requireValue(i, val) || !ParseI32(val, &topNodes) || topNodes < 0) {
        std::cerr << "--top-nodes requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--top-edges") {
      if (!requireValue(i, val) || !ParseI32(val, &topEdges) || topEdges < 0) {
        std::cerr << "--top-edges requires a non-negative integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};

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

  const RoadGraph g = BuildRoadGraph(world);
  const RoadGraphMetrics metrics = ComputeRoadGraphMetrics(g);

  std::cout << "RoadGraph metrics\n";
  std::cout << "  nodes:            " << metrics.nodes << "\n";
  std::cout << "  edges:            " << metrics.edges << "\n";
  std::cout << "  components:       " << metrics.components << "\n";
  std::cout << "  isolatedNodes:    " << metrics.isolatedNodes << "\n";
  std::cout << "  avgDegree:        " << metrics.avgDegree << "\n";
  std::cout << "  avgEdgeLength:    " << metrics.avgEdgeLength << "\n";
  std::cout << "  approxDiameter:   " << metrics.approxDiameter << " (nodes " << metrics.diameterA << " -> "
            << metrics.diameterB << ")\n";

  std::vector<int> compId;
  ComputeRoadGraphComponents(g, compId);

  const World* weightWorld = (ccfg.weightMode == RoadGraphEdgeWeightMode::TravelTimeMilli) ? &world : nullptr;
  const RoadGraphCentralityResult cent = ComputeRoadGraphCentrality(g, ccfg, weightWorld);

  std::cout << "\nCentrality\n";
  std::cout << "  sourcesUsed:      " << cent.sourcesUsed;
  if (ccfg.maxSources > 0 && cent.sourcesUsed < metrics.nodes) std::cout << " (sampled)";
  std::cout << "\n";

  // Print a short list of the most central nodes.
  struct TopNode {
    int id = -1;
    double v = 0.0;
  };
  std::vector<TopNode> top;
  top.reserve(static_cast<std::size_t>(metrics.nodes));
  const bool hasNorm = (static_cast<int>(cent.nodeBetweennessNorm.size()) == metrics.nodes);
  for (int i = 0; i < metrics.nodes; ++i) {
    const double v = hasNorm ? cent.nodeBetweennessNorm[static_cast<std::size_t>(i)]
                             : cent.nodeBetweenness[static_cast<std::size_t>(i)];
    top.push_back(TopNode{i, v});
  }
  std::sort(top.begin(), top.end(), [](const TopNode& a, const TopNode& b) {
    if (a.v != b.v) return a.v > b.v;
    return a.id < b.id;
  });

  const int printN = std::min(10, static_cast<int>(top.size()));
  std::cout << "  topNodes (" << (hasNorm ? "betweennessNorm" : "betweenness") << ")\n";
  for (int i = 0; i < printN; ++i) {
    const int id = top[static_cast<std::size_t>(i)].id;
    const double v = top[static_cast<std::size_t>(i)].v;
    const RoadGraphNode& n0 = g.nodes[static_cast<std::size_t>(id)];
    std::cout << "    #" << i + 1 << " node " << id << " @(" << n0.pos.x << "," << n0.pos.y << ") val="
              << v << "\n";
  }

  RoadGraphCentralityExportConfig ecfg;
  ecfg.includeEdgeTiles = includeTiles;
  ecfg.colorByComponent = colorComponents;

  std::string err;

  if (!dotPath.empty()) {
    if (!ExportRoadGraphCentralityDot(dotPath, g, cent, &compId, ecfg, &err)) {
      std::cerr << "DOT export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote dot -> " << dotPath << "\n";
  }

  if (!jsonPath.empty()) {
    if (!ExportRoadGraphCentralityJson(jsonPath, g, cent, &compId, ecfg, &err)) {
      std::cerr << "JSON export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote json -> " << jsonPath << "\n";
  }

  if (!nodesCsvPath.empty()) {
    if (!ExportRoadGraphCentralityNodesCsv(nodesCsvPath, g, cent, &compId, &err)) {
      std::cerr << "Nodes CSV export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote nodes csv -> " << nodesCsvPath << "\n";
  }

  if (!edgesCsvPath.empty()) {
    if (!ExportRoadGraphCentralityEdgesCsv(edgesCsvPath, g, cent, &compId, ecfg, &err)) {
      std::cerr << "Edges CSV export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote edges csv -> " << edgesCsvPath << "\n";
  }

  if (!ppmPath.empty()) {
    RoadGraphCentralityVizConfig vcfg;
    vcfg.baseLayer = ExportLayer::Overlay;
    vcfg.topNodes = topNodes;
    vcfg.topEdges = topEdges;
    vcfg.highlightEdgeTiles = true;

    PpmImage img = RenderRoadGraphCentralityDebugPpm(world, g, cent, vcfg);
    if (ppmScale > 1) img = ScaleNearest(img, ppmScale);

    if (!WriteImageAuto(ppmPath, img, err)) {
      std::cerr << "Image export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote ppm/png -> " << ppmPath << "\n";
  }

  return 0;
}
