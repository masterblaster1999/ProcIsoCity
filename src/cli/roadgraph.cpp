#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphExport.hpp"
#include "isocity/SaveLoad.hpp"

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

void PrintHelp()
{
  std::cout
      << "proc_isocity_roadgraph (headless road graph export)\n\n"
      << "Builds a compressed road graph (nodes=intersections/endpoints/corners) from a world\n"
      << "and exports it to DOT/JSON/CSV. Also computes simple connectivity metrics and an\n"
      << "approximate weighted diameter path (double-Dijkstra).\n\n"
      << "Usage:\n"
      << "  proc_isocity_roadgraph [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                       [--dot <out.dot>] [--json <out.json>]\n"
      << "                       [--nodes-csv <out.csv>] [--edges-csv <out.csv>]\n"
      << "                       [--include-tiles <0|1>] [--color-components <0|1>]\n"
      << "                       [--ppm <out.ppm>] [--diameter-ppm <out.ppm>] [--ppm-scale <N>]\n\n"
      << "Notes:\n"
      << "  - If --load is omitted, a world is generated from (--seed, --size).\n"
      << "  - --include-tiles may produce large JSON/CSV files on big maps.\n"
      << "  - PPM exports are one-pixel-per-tile (use --ppm-scale to upscale).\n";
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
  std::string diameterPpmPath;

  bool includeTiles = false;
  bool colorComponents = true;
  int ppmScale = 4;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

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
    } else if (arg == "--diameter-ppm") {
      if (!requireValue(i, val)) {
        std::cerr << "--diameter-ppm requires a path\n";
        return 2;
      }
      diameterPpmPath = val;
    } else if (arg == "--ppm-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &ppmScale) || ppmScale <= 0) {
        std::cerr << "--ppm-scale requires a positive integer\n";
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
  const RoadGraphDiameter diam = ComputeApproxRoadGraphDiameter(g);
  const RoadGraphMetrics metrics = ComputeRoadGraphMetrics(g);

  std::cout << "RoadGraph metrics\n";
  std::cout << "  nodes:               " << metrics.nodes << "\n";
  std::cout << "  edges:               " << metrics.edges << "\n";
  std::cout << "  components:          " << metrics.components << "\n";
  std::cout << "  largestComponent:    nodes=" << metrics.largestComponentNodes
            << " edges=" << metrics.largestComponentEdges << "\n";
  std::cout << "  isolatedNodes:       " << metrics.isolatedNodes << "\n";
  std::cout << "  totalEdgeLength:     " << metrics.totalEdgeLength << "\n";
  std::cout << "  avgDegree:           " << metrics.avgDegree << "\n";
  std::cout << "  avgEdgeLength:       " << metrics.avgEdgeLength << "\n";
  std::cout << "  approxDiameter:      " << metrics.approxDiameter << " (nodes " << metrics.diameterA << " -> "
            << metrics.diameterB << ")\n";

  RoadGraphExportConfig cfg;
  cfg.includeEdgeTiles = includeTiles;
  cfg.colorByComponent = colorComponents;

  std::string err;
  std::vector<int> compId;
  ComputeRoadGraphComponents(g, compId);

  if (!dotPath.empty()) {
    if (!ExportRoadGraphDot(dotPath, g, &metrics, cfg, &err)) {
      std::cerr << "DOT export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote dot -> " << dotPath << "\n";
  }

  if (!jsonPath.empty()) {
    if (!ExportRoadGraphJson(jsonPath, g, &metrics, &diam, cfg, &err)) {
      std::cerr << "JSON export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote json -> " << jsonPath << "\n";
  }

  if (!nodesCsvPath.empty()) {
    if (!ExportRoadGraphNodesCsv(nodesCsvPath, g, &compId, &err)) {
      std::cerr << "Nodes CSV export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote nodes csv -> " << nodesCsvPath << "\n";
  }

  if (!edgesCsvPath.empty()) {
    if (!ExportRoadGraphEdgesCsv(edgesCsvPath, g, &compId, cfg, &err)) {
      std::cerr << "Edges CSV export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote edges csv -> " << edgesCsvPath << "\n";
  }

  auto writePpm = [&](const std::string& path, const std::vector<Point>* highlight) -> bool {
    if (path.empty()) return true;
    PpmImage img = RenderRoadGraphDebugPpm(world, g, ExportLayer::Overlay, highlight);
    if (ppmScale > 1) img = ScaleNearest(img, ppmScale);
    std::string we;
    if (!WritePpm(path, img, we)) {
      std::cerr << "PPM export failed: " << we << "\n";
      return false;
    }
    std::cout << "wrote ppm -> " << path << "\n";
    return true;
  };

  if (!ppmPath.empty()) {
    if (!writePpm(ppmPath, nullptr)) return 2;
  }

  if (!diameterPpmPath.empty()) {
    std::vector<Point> tiles;
    if (!ExpandRoadGraphNodePathToTiles(g, diam.nodePath, tiles)) {
      std::cerr << "Failed to expand diameter path to tiles (graph may be empty)\n";
      tiles.clear();
    }
    if (!writePpm(diameterPpmPath, &tiles)) return 2;
  }

  return 0;
}
