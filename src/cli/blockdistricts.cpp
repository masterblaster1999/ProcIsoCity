#include "isocity/BlockDistricting.hpp"
#include "isocity/CityBlockGraph.hpp"
#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

void PrintHelp()
{
  std::cout
      << "proc_isocity_blockdistricts (headless block-based district assignment)\n\n"
      << "Assigns administrative districts (0..7) using city blocks and their adjacency graph.\n"
      << "Blocks are connected components of non-road, non-water land separated by roads/water.\n\n"
      << "Usage:\n"
      << "  proc_isocity_blockdistricts [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                           [--districts <N>] [--fill-roads <0|1>] [--include-water <0|1>]\n"
      << "                           [--json <out.json>] [--dot <out.dot>]\n"
      << "                           [--blocks-csv <out.csv>] [--edges-csv <out.csv>]\n"
      << "                           [--district-ppm <out.ppm|out.png>] [--scale <N>]\n"
      << "                           [--write-save <out.bin>] [--include-edges <0|1>]\n\n"
      << "Notes:\n"
      << "  - If --load is omitted, a world is generated from (--seed, --size).\n"
      << "  - --include-edges can make JSON large on big maps.\n";
}

// Small stable palette for DOT output.
const char* DistrictColor(int d)
{
  switch (d) {
  case 0: return "#1f77b4";
  case 1: return "#ff7f0e";
  case 2: return "#2ca02c";
  case 3: return "#d62728";
  case 4: return "#9467bd";
  case 5: return "#8c564b";
  case 6: return "#e377c2";
  case 7: return "#7f7f7f";
  default: return "#000000";
  }
}

bool WriteBlocksCsv(const std::string& path, const CityBlockGraphResult& g, const BlockDistrictResult& dres,
                    std::string& outError)
{
  if (!EnsureParentDir(path)) {
    outError = "failed to create output directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open " + path;
    return false;
  }

  f << "blockId,district,area,minX,minY,maxX,maxY,roadAdjTiles,roadEdges,waterEdges,outsideEdges,parks,residential,commercial,industrial,other,";
  f << "roadEdgesL1,roadEdgesL2,roadEdgesL3,roadAdjTilesL1,roadAdjTilesL2,roadAdjTilesL3\n";

  const int n = static_cast<int>(g.blocks.blocks.size());
  for (int i = 0; i < n; ++i) {
    const CityBlock& b = g.blocks.blocks[static_cast<std::size_t>(i)];
    const int dist = (i < static_cast<int>(dres.blockToDistrict.size())) ? static_cast<int>(dres.blockToDistrict[static_cast<std::size_t>(i)]) : 0;
    const CityBlockFrontage& fr = g.frontage[static_cast<std::size_t>(i)];

    f << b.id << ',' << dist << ',' << b.area << ','
      << b.minX << ',' << b.minY << ',' << b.maxX << ',' << b.maxY << ','
      << b.roadAdjTiles << ',' << b.roadEdges << ',' << b.waterEdges << ',' << b.outsideEdges << ','
      << b.parks << ',' << b.residential << ',' << b.commercial << ',' << b.industrial << ',' << b.other << ','
      << fr.roadEdgesByLevel[1] << ',' << fr.roadEdgesByLevel[2] << ',' << fr.roadEdgesByLevel[3] << ','
      << fr.roadAdjTilesByLevel[1] << ',' << fr.roadAdjTilesByLevel[2] << ',' << fr.roadAdjTilesByLevel[3] << '\n';
  }

  if (!f) {
    outError = "failed while writing " + path;
    return false;
  }
  return true;
}

bool WriteEdgesCsv(const std::string& path, const CityBlockGraphResult& g, const BlockDistrictResult& dres,
                   std::string& outError)
{
  if (!EnsureParentDir(path)) {
    outError = "failed to create output directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open " + path;
    return false;
  }

  f << "a,b,districtA,districtB,touchingRoadTiles,touchingRoadTilesL1,touchingRoadTilesL2,touchingRoadTilesL3\n";

  for (const CityBlockAdjacency& e : g.edges) {
    const int da = (e.a >= 0 && e.a < static_cast<int>(dres.blockToDistrict.size())) ? static_cast<int>(dres.blockToDistrict[static_cast<std::size_t>(e.a)]) : 0;
    const int db = (e.b >= 0 && e.b < static_cast<int>(dres.blockToDistrict.size())) ? static_cast<int>(dres.blockToDistrict[static_cast<std::size_t>(e.b)]) : 0;

    f << e.a << ',' << e.b << ',' << da << ',' << db << ','
      << e.touchingRoadTiles << ','
      << e.touchingRoadTilesByLevel[1] << ',' << e.touchingRoadTilesByLevel[2] << ',' << e.touchingRoadTilesByLevel[3]
      << '\n';
  }

  if (!f) {
    outError = "failed while writing " + path;
    return false;
  }
  return true;
}

bool WriteDot(const std::string& path, const CityBlockGraphResult& g, const BlockDistrictResult& dres, std::string& outError)
{
  if (!EnsureParentDir(path)) {
    outError = "failed to create output directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open " + path;
    return false;
  }

  int maxContact = 1;
  for (const CityBlockAdjacency& e : g.edges) {
    maxContact = std::max(maxContact, std::max(1, e.touchingRoadTiles));
  }

  f << "graph blockdistricts {\n";
  f << "  overlap=false;\n";
  f << "  splines=true;\n";
  f << "  node [shape=box, style=filled, fontname=\"Helvetica\", fontsize=10];\n";
  f << "  edge [color=\"#777777\", fontname=\"Helvetica\", fontsize=9];\n\n";

  const int n = static_cast<int>(g.blocks.blocks.size());
  for (int i = 0; i < n; ++i) {
    const CityBlock& b = g.blocks.blocks[static_cast<std::size_t>(i)];
    const int dist = (i < static_cast<int>(dres.blockToDistrict.size())) ? static_cast<int>(dres.blockToDistrict[static_cast<std::size_t>(i)]) : 0;
    f << "  b" << b.id << " [label=\"" << b.id << "\\nA=" << b.area << "\", fillcolor=\"" << DistrictColor(dist) << "\"];\n";
  }

  f << "\n";

  for (const CityBlockAdjacency& e : g.edges) {
    const int w = std::max(1, e.touchingRoadTiles);
    const double pen = 1.0 + 4.0 * (static_cast<double>(w) / static_cast<double>(maxContact));
    f << "  b" << e.a << " -- b" << e.b << " [label=\"" << w << "\", penwidth=" << pen << "];\n";
  }

  f << "}\n";

  if (!f) {
    outError = "failed while writing " + path;
    return false;
  }
  return true;
}

bool WriteJsonReport(const std::string& path, const World& world, const CityBlockGraphResult& g,
                     const BlockDistrictResult& dres, bool includeEdges, std::string& outError)
{
  if (!EnsureParentDir(path)) {
    outError = "failed to create output directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open " + path;
    return false;
  }

  f << "{\n";
  f << "  \"width\": " << world.width() << ",\n";
  f << "  \"height\": " << world.height() << ",\n";
  f << "  \"seed\": " << static_cast<std::uint64_t>(world.seed()) << ",\n";
  f << "  \"districtsRequested\": " << dres.districtsRequested << ",\n";
  f << "  \"districtsUsed\": " << dres.districtsUsed << ",\n";

  // Seed list.
  f << "  \"seeds\": [";
  for (std::size_t i = 0; i < dres.seedBlockId.size(); ++i) {
    if (i) f << ", ";
    const int bid = dres.seedBlockId[i];
    f << "{\"district\": " << i << ", \"blockId\": " << bid;
    if (bid >= 0 && bid < static_cast<int>(g.blocks.blocks.size())) {
      const CityBlock& b = g.blocks.blocks[static_cast<std::size_t>(bid)];
      f << ", \"area\": " << b.area;
      f << ", \"bounds\": [" << b.minX << ',' << b.minY << ',' << b.maxX << ',' << b.maxY << ']';
    }
    f << "}";
  }
  f << "],\n";

  // District summary.
  f << "  \"districtSummary\": [\n";
  for (int d = 0; d < kDistrictCount; ++d) {
    f << "    {\"district\": " << d << ", \"blocks\": " << dres.blocksPerDistrict[static_cast<std::size_t>(d)]
      << ", \"tiles\": " << dres.tilesPerDistrict[static_cast<std::size_t>(d)] << "}";
    if (d + 1 != kDistrictCount) f << ',';
    f << "\n";
  }
  f << "  ],\n";

  // Blocks (always included, but fairly compact).
  f << "  \"blocks\": [\n";
  for (std::size_t i = 0; i < g.blocks.blocks.size(); ++i) {
    const CityBlock& b = g.blocks.blocks[i];
    const int dist = (i < dres.blockToDistrict.size()) ? static_cast<int>(dres.blockToDistrict[i]) : 0;
    const CityBlockFrontage& fr = g.frontage[i];

    f << "    {\"id\": " << b.id;
    f << ", \"district\": " << dist;
    f << ", \"area\": " << b.area;
    f << ", \"bounds\": [" << b.minX << ',' << b.minY << ',' << b.maxX << ',' << b.maxY << ']';
    f << ", \"roadAdjTiles\": " << b.roadAdjTiles;
    f << ", \"roadEdges\": " << b.roadEdges;
    f << ", \"frontage\": {\"roadEdgesByLevel\": [0," << fr.roadEdgesByLevel[1] << ',' << fr.roadEdgesByLevel[2]
      << ',' << fr.roadEdgesByLevel[3] << "], \"roadAdjTilesByLevel\": [0," << fr.roadAdjTilesByLevel[1] << ','
      << fr.roadAdjTilesByLevel[2] << ',' << fr.roadAdjTilesByLevel[3] << "]}";
    f << "}";
    if (i + 1 != g.blocks.blocks.size()) f << ',';
    f << "\n";
  }
  f << "  ]";

  if (includeEdges) {
    f << ",\n  \"edges\": [\n";
    for (std::size_t i = 0; i < g.edges.size(); ++i) {
      const CityBlockAdjacency& e = g.edges[i];
      f << "    {\"a\": " << e.a << ", \"b\": " << e.b << ", \"touchingRoadTiles\": " << e.touchingRoadTiles
        << ", \"touchingRoadTilesByLevel\": [0," << e.touchingRoadTilesByLevel[1] << ',' << e.touchingRoadTilesByLevel[2]
        << ',' << e.touchingRoadTilesByLevel[3] << "]}";
      if (i + 1 != g.edges.size()) f << ',';
      f << "\n";
    }
    f << "  ]\n";
    f << "}\n";
  } else {
    f << "\n}\n";
  }

  if (!f) {
    outError = "failed while writing " + path;
    return false;
  }
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::string outJson;
  std::string outDot;
  std::string outBlocksCsv;
  std::string outEdgesCsv;
  std::string outDistrictPpm;
  std::string outSave;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  int districts = kDistrictCount;
  bool fillRoads = true;
  bool includeWater = false;
  bool includeEdges = false;

  int scale = 4;

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
    } else if (arg == "--districts") {
      if (!requireValue(i, val) || !ParseI32(val, &districts)) {
        std::cerr << "--districts requires an integer\n";
        return 2;
      }
    } else if (arg == "--fill-roads") {
      if (!requireValue(i, val) || !ParseBool01(val, &fillRoads)) {
        std::cerr << "--fill-roads requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--include-water") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeWater)) {
        std::cerr << "--include-water requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--include-edges") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeEdges)) {
        std::cerr << "--include-edges requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--dot") {
      if (!requireValue(i, val)) {
        std::cerr << "--dot requires a path\n";
        return 2;
      }
      outDot = val;
    } else if (arg == "--blocks-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--blocks-csv requires a path\n";
        return 2;
      }
      outBlocksCsv = val;
    } else if (arg == "--edges-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--edges-csv requires a path\n";
        return 2;
      }
      outEdgesCsv = val;
    } else if (arg == "--district-ppm") {
      if (!requireValue(i, val)) {
        std::cerr << "--district-ppm requires a path\n";
        return 2;
      }
      outDistrictPpm = val;
    } else if (arg == "--scale") {
      if (!requireValue(i, val) || !ParseI32(val, &scale) || scale <= 0) {
        std::cerr << "--scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--write-save") {
      if (!requireValue(i, val)) {
        std::cerr << "--write-save requires a path\n";
        return 2;
      }
      outSave = val;
    } else {
      std::cerr << "unknown arg: " << arg << "\n";
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

  BlockDistrictConfig cfg;
  cfg.districts = districts;
  cfg.fillRoadTiles = fillRoads;
  cfg.includeWater = includeWater;

  // Build graph once so we can export details.
  const CityBlockGraphResult graph = BuildCityBlockGraph(world);
  BlockDistrictResult dres = AssignDistrictsByBlocks(world, cfg, &graph);

  std::cout << "BlockDistricting\n";
  std::cout << "  blocks:            " << graph.blocks.blocks.size() << "\n";
  std::cout << "  blockEdges:        " << graph.edges.size() << "\n";
  std::cout << "  districtsUsed:     " << dres.districtsUsed << " (requested " << dres.districtsRequested << ")\n";

  for (int d = 0; d < kDistrictCount; ++d) {
    const int blocks = dres.blocksPerDistrict[static_cast<std::size_t>(d)];
    const int tiles = dres.tilesPerDistrict[static_cast<std::size_t>(d)];
    if (blocks == 0 && tiles == 0) continue;
    std::cout << "  district " << d << ": blocks=" << blocks << " tiles=" << tiles << "\n";
  }

  std::string err;

  if (!outBlocksCsv.empty()) {
    if (!WriteBlocksCsv(outBlocksCsv, graph, dres, err)) {
      std::cerr << "blocks CSV export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote blocks csv -> " << outBlocksCsv << "\n";
  }

  if (!outEdgesCsv.empty()) {
    if (!WriteEdgesCsv(outEdgesCsv, graph, dres, err)) {
      std::cerr << "edges CSV export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote edges csv -> " << outEdgesCsv << "\n";
  }

  if (!outDot.empty()) {
    if (!WriteDot(outDot, graph, dres, err)) {
      std::cerr << "DOT export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote dot -> " << outDot << "\n";
  }

  if (!outJson.empty()) {
    if (!WriteJsonReport(outJson, world, graph, dres, includeEdges, err)) {
      std::cerr << "JSON export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote json -> " << outJson << "\n";
  }

  if (!outDistrictPpm.empty()) {
    if (!EnsureParentDir(outDistrictPpm)) {
      std::cerr << "failed to create output directory for: " << outDistrictPpm << "\n";
      return 2;
    }
    const PpmImage img = RenderPpmLayer(world, ExportLayer::District);
    const PpmImage scaled = ScaleNearest(img, scale);
    if (!WriteImageAuto(outDistrictPpm, scaled, err)) {
      std::cerr << "district image export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote district image -> " << outDistrictPpm << "\n";
  }

  if (!outSave.empty()) {
    if (!EnsureParentDir(outSave)) {
      std::cerr << "failed to create output directory for: " << outSave << "\n";
      return 2;
    }

    if (!SaveWorldBinary(world, procCfg, simCfg, outSave, err)) {
      std::cerr << "failed to save: " << err << "\n";
      return 2;
    }
    std::cout << "wrote save -> " << outSave << "\n";
  }

  return 0;
}
