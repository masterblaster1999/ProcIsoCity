#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/RoadGraphTrafficExport.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Json.hpp"
#include "isocity/Sim.hpp"

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
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

struct OdRow {
  const isocity::GoodsOdEdge* e = nullptr;
  int amount = 0;
};

bool ExportOdCsv(const std::string& path, const isocity::World& world, const isocity::GoodsFlowDebug& dbg, std::string& outError)
{
  std::ofstream f(path);
  if (!f) {
    outError = "Failed to open for writing: " + path;
    return false;
  }

  auto xyOf = [&](int ridx, int& outX, int& outY) {
    const int w = world.width();
    outX = (w > 0) ? (ridx % w) : 0;
    outY = (w > 0) ? (ridx / w) : 0;
  };

  f << "type,amount,src_idx,src_x,src_y,dst_idx,dst_x,dst_y,mean_steps,mean_cost_milli,min_steps,max_steps,min_cost_milli,max_cost_milli,total_steps,total_cost_milli\n";
  for (const isocity::GoodsOdEdge& e : dbg.od) {
    if (e.amount <= 0) continue;
    int sx = 0, sy = 0, dx = 0, dy = 0;
    xyOf(e.srcRoadIdx, sx, sy);
    xyOf(e.dstRoadIdx, dx, dy);

    const double meanSteps = (e.amount > 0) ? (static_cast<double>(e.totalSteps) / static_cast<double>(e.amount)) : 0.0;
    const double meanCost = (e.amount > 0) ? (static_cast<double>(e.totalCostMilli) / static_cast<double>(e.amount)) : 0.0;

    f << isocity::GoodsOdTypeName(e.type) << ','
      << e.amount << ','
      << e.srcRoadIdx << ',' << sx << ',' << sy << ','
      << e.dstRoadIdx << ',' << dx << ',' << dy << ','
      << meanSteps << ',' << meanCost << ','
      << e.minSteps << ',' << e.maxSteps << ','
      << e.minCostMilli << ',' << e.maxCostMilli << ','
      << e.totalSteps << ',' << e.totalCostMilli << "\n";
  }

  return true;
}

bool ExportOdGeoJson(const std::string& path, const isocity::World& world, const isocity::GoodsFlowDebug& dbg,
                     int topN, int minAmount, std::string& outError)
{
  std::ofstream f(path);
  if (!f) {
    outError = "Failed to open for writing: " + path;
    return false;
  }

  std::vector<OdRow> rows;
  rows.reserve(dbg.od.size());
  for (const isocity::GoodsOdEdge& e : dbg.od) {
    if (e.amount < minAmount) continue;
    rows.push_back(OdRow{&e, e.amount});
  }

  std::sort(rows.begin(), rows.end(), [&](const OdRow& a, const OdRow& b) {
    if (a.amount != b.amount) return a.amount > b.amount;
    if (a.e && b.e) {
      if (a.e->type != b.e->type) return static_cast<int>(a.e->type) < static_cast<int>(b.e->type);
      if (a.e->srcRoadIdx != b.e->srcRoadIdx) return a.e->srcRoadIdx < b.e->srcRoadIdx;
      return a.e->dstRoadIdx < b.e->dstRoadIdx;
    }
    return a.e < b.e;
  });

  if (topN > 0 && static_cast<int>(rows.size()) > topN) {
    rows.resize(static_cast<std::size_t>(topN));
  }

  auto xyOf = [&](int ridx, int& outX, int& outY) {
    const int w = world.width();
    outX = (w > 0) ? (ridx % w) : 0;
    outY = (w > 0) ? (ridx / w) : 0;
  };

  isocity::JsonWriteOptions jopt{};
  jopt.pretty = true;
  jopt.indent = 2;
  jopt.sortKeys = false;

  isocity::JsonWriter jw(f, jopt);

  auto writeCoord = [&](double x, double y) {
    jw.beginArray();
    jw.numberValue(x);
    jw.numberValue(y);
    jw.endArray();
  };

  jw.beginObject();
  jw.key("type");
  jw.stringValue("FeatureCollection");
  jw.key("properties");
  jw.beginObject();
  jw.key("coordSpace");
  jw.stringValue("tile_center");
  jw.key("minAmount");
  jw.intValue(minAmount);
  jw.key("topN");
  jw.intValue(topN);
  jw.endObject();

  jw.key("features");
  jw.beginArray();

  for (const OdRow& r : rows) {
    if (!r.e) continue;
    const isocity::GoodsOdEdge& e = *r.e;
    if (e.amount <= 0) continue;

    int sx = 0, sy = 0, dx = 0, dy = 0;
    xyOf(e.srcRoadIdx, sx, sy);
    xyOf(e.dstRoadIdx, dx, dy);

    const double meanSteps =
        (e.amount > 0) ? (static_cast<double>(e.totalSteps) / static_cast<double>(e.amount)) : 0.0;
    const double meanCost =
        (e.amount > 0) ? (static_cast<double>(e.totalCostMilli) / static_cast<double>(e.amount)) : 0.0;

    // Coordinates are tile-center points in world grid space.
    const double sxf = static_cast<double>(sx) + 0.5;
    const double syf = static_cast<double>(sy) + 0.5;
    const double dxf = static_cast<double>(dx) + 0.5;
    const double dyf = static_cast<double>(dy) + 0.5;

    jw.beginObject();
    jw.key("type");
    jw.stringValue("Feature");

    jw.key("properties");
    jw.beginObject();
    jw.key("flow_type");
    jw.stringValue(isocity::GoodsOdTypeName(e.type));
    jw.key("amount");
    jw.intValue(e.amount);
    jw.key("src_idx");
    jw.intValue(e.srcRoadIdx);
    jw.key("dst_idx");
    jw.intValue(e.dstRoadIdx);
    jw.key("mean_steps");
    jw.numberValue(meanSteps);
    jw.key("mean_cost_milli");
    jw.numberValue(meanCost);
    jw.key("min_steps");
    jw.intValue(e.minSteps);
    jw.key("max_steps");
    jw.intValue(e.maxSteps);
    jw.key("min_cost_milli");
    jw.intValue(e.minCostMilli);
    jw.key("max_cost_milli");
    jw.intValue(e.maxCostMilli);
    jw.endObject();

    jw.key("geometry");
    jw.beginObject();
    jw.key("type");
    jw.stringValue("LineString");
    jw.key("coordinates");
    jw.beginArray();
    writeCoord(sxf, syf);
    writeCoord(dxf, dyf);
    jw.endArray();
    jw.endObject();

    jw.endObject();
  }

  jw.endArray();
  jw.endObject();

  if (!jw.ok() || !f.good()) {
    outError = jw.error();
    return false;
  }
  return true;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_goodsgraph (headless goods flow + road graph aggregation + OD export)\n\n"
      << "Generates (or loads) a world, optionally steps the simulation for N days to populate zones,\n"
      << "computes a goods flow heatmap (per road tile), aggregates it onto the compressed RoadGraph\n"
      << "(nodes/intersections + edges/segments) and exports DOT/JSON/CSV.\n\n"
      << "Additionally, it can export an origin-destination (OD) flow list between road access points\n"
      << "(industrial -> commercial, edge imports, and edge exports).\n\n"
      << "Usage:\n"
      << "  proc_isocity_goodsgraph [--seed N] [--size WxH] [--days N] [--load save.bin] [outputs...]\n\n"
      << "Inputs:\n"
      << "  --load <path>            Load a save instead of generating a new world.\n"
      << "  --seed <u64>             World seed (decimal or 0x...). Default: 1\n"
      << "  --size <WxH>             World size when generating. Default: 128x128\n"
      << "  --days <N>               Step the simulation N days before computing goods. Default: 60\n"
      << "\n"
      << "Goods config:\n"
      << "  --require-outside <0|1>  Outside connection rule. Default: 1\n"
      << "  --allow-imports <0|1>    Allow imports from edge. Default: 1\n"
      << "  --allow-exports <0|1>    Allow exports to edge. Default: 1\n"
      << "  --supply-scale <f>       Industrial supply scale. Default: 1.0\n"
      << "  --demand-scale <f>       Commercial demand scale. Default: 1.0\n"
      << "\n"
      << "Aggregation config:\n"
      << "  --tile-capacity <N>        Base road tile capacity (street). Default: 28\n"
      << "  --use-road-level-cap <0|1> If 1, scale capacity by road class (Tile::level). Default: 1\n"
      << "\n"
      << "Graph outputs:\n"
      << "  --dot <path>            GraphViz DOT (edges colored by utilization).\n"
      << "  --json <path>           JSON export (nodes/edges + flow stats).\n"
      << "  --nodes-csv <path>      Node stats CSV.\n"
      << "  --edges-csv <path>      Edge stats CSV.\n"
      << "  --include-tiles <0|1>   Include per-edge tile polylines in JSON. Default: 0\n"
      << "\n"
      << "Images:\n"
      << "  --heatmap <path>        Goods traffic heatmap image (PPM/PNG by extension).\n"
      << "  --fillmap <path>        Commercial fill heatmap image (PPM/PNG by extension).\n"
      << "  --highlight <path>      Overlay image highlighting the busiest edges (by max utilization).\n"
      << "  --highlight-top <N>     Highlight top N edges. Default: 20\n"
      << "  --scale <N>             Nearest-neighbor upscale factor for images. Default: 4\n"
      << "\n"
      << "OD exports (optional):\n"
      << "  --od-csv <path>         CSV of aggregated OD flows (local/import/export).\n"
      << "  --od-geojson <path>     GeoJSON desire lines (top-N by amount).\n"
      << "  --od-top <N>            Limit OD GeoJSON features (0 = all). Default: 200\n"
      << "  --od-min-amount <N>     Filter out small OD flows. Default: 1\n";
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
  std::string fillmapPath;
  std::string highlightPath;

  std::string odCsvPath;
  std::string odGeoJsonPath;

  bool includeTiles = false;
  int scale = 4;
  int highlightTop = 20;

  int odTop = 200;
  int odMinAmount = 1;

  int w = 128;
  int h = 128;
  std::uint64_t seed = 1;
  int days = 60;

  // Goods config.
  GoodsConfig gcfg{};
  bool requireOutside = gcfg.requireOutsideConnection;
  bool allowImports = gcfg.allowImports;
  bool allowExports = gcfg.allowExports;
  float supplyScale = gcfg.supplyScale;
  float demandScale = gcfg.demandScale;

  // Aggregation config.
  RoadGraphTrafficConfig agCfg{};
  int baseCapacity = agCfg.baseTileCapacity;
  bool useRoadLevelCapacity = agCfg.useRoadLevelCapacity;

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
    } else if (arg == "--fillmap") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--fillmap requires a path\n";
        return 2;
      }
      fillmapPath = val;
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
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &scale) || scale < 1) {
        std::cerr << "--scale requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      requireOutside = b;
    } else if (arg == "--allow-imports") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--allow-imports requires 0 or 1\n";
        return 2;
      }
      allowImports = b;
    } else if (arg == "--allow-exports") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--allow-exports requires 0 or 1\n";
        return 2;
      }
      allowExports = b;
    } else if (arg == "--supply-scale") {
      float f = 1.0f;
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &f) || f < 0.0f) {
        std::cerr << "--supply-scale requires a float >= 0\n";
        return 2;
      }
      supplyScale = f;
    } else if (arg == "--demand-scale") {
      float f = 1.0f;
      if (!requireValue(i, val, argc, argv) || !ParseF32(val, &f) || f < 0.0f) {
        std::cerr << "--demand-scale requires a float >= 0\n";
        return 2;
      }
      demandScale = f;
    } else if (arg == "--tile-capacity") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &baseCapacity) || baseCapacity < 1) {
        std::cerr << "--tile-capacity requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--use-road-level-cap") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--use-road-level-cap requires 0 or 1\n";
        return 2;
      }
      useRoadLevelCapacity = b;
    } else if (arg == "--od-csv") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--od-csv requires a path\n";
        return 2;
      }
      odCsvPath = val;
    } else if (arg == "--od-geojson") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--od-geojson requires a path\n";
        return 2;
      }
      odGeoJsonPath = val;
    } else if (arg == "--od-top") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &odTop) || odTop < 0) {
        std::cerr << "--od-top requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--od-min-amount") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &odMinAmount) || odMinAmount < 1) {
        std::cerr << "--od-min-amount requires an integer >= 1\n";
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
      std::cerr << "Failed to load save: " << loadPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  // Respect the requested outside-connection rule for the sim.
  simCfg.requireOutsideConnection = requireOutside;

  // Optionally simulate some days to populate zones.
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }
  if (days == 0) {
    sim.refreshDerivedStats(world);
  }

  // Build goods config.
  gcfg.requireOutsideConnection = requireOutside;
  gcfg.allowImports = allowImports;
  gcfg.allowExports = allowExports;
  gcfg.supplyScale = supplyScale;
  gcfg.demandScale = demandScale;

  const bool wantOd = (!odCsvPath.empty() || !odGeoJsonPath.empty());
  GoodsFlowDebug dbg;
  GoodsFlowDebug* dbgPtr = wantOd ? &dbg : nullptr;

  // Compute goods flow.
  const GoodsResult gr = ComputeGoodsFlow(world, gcfg, nullptr, nullptr, dbgPtr);

  // Build road graph and aggregate.
  const RoadGraph rg = BuildRoadGraph(world);

  agCfg.baseTileCapacity = baseCapacity;
  agCfg.useRoadLevelCapacity = useRoadLevelCapacity;

  std::vector<std::uint32_t> flow32;
  flow32.reserve(gr.roadGoodsTraffic.size());
  for (std::uint16_t v : gr.roadGoodsTraffic) {
    flow32.push_back(static_cast<std::uint32_t>(v));
  }

  const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(world, rg, flow32, agCfg);

  std::cout << "GoodsGraph summary\n";
  std::cout << "  world: " << world.width() << "x" << world.height() << "  day=" << world.stats().day << "\n";
  std::cout << "  roadGraph: nodes=" << rg.nodes.size() << " edges=" << rg.edges.size() << "\n";
  std::cout << "  goods: produced=" << gr.goodsProduced << " demand=" << gr.goodsDemand << " delivered=" << gr.goodsDelivered
            << " imported=" << gr.goodsImported << " exported=" << gr.goodsExported
            << " unreachableDemand=" << gr.unreachableDemand << " satisfaction=" << gr.satisfaction << "\n";
  std::cout << "  maxTileGoodsTraffic=" << gr.maxRoadGoodsTraffic << "\n";

  // Rank edges by utilization.
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
    PpmImage img = RenderPpmLayer(world, ExportLayer::GoodsTraffic, nullptr, nullptr, &gr);
    img = ScaleNearest(img, scale);
    if (!WriteImageAuto(heatmapPath, img, err)) {
      std::cerr << "Failed to write heatmap image: " << heatmapPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!fillmapPath.empty()) {
    PpmImage img = RenderPpmLayer(world, ExportLayer::GoodsFill, nullptr, nullptr, &gr);
    img = ScaleNearest(img, scale);
    if (!WriteImageAuto(fillmapPath, img, err)) {
      std::cerr << "Failed to write fillmap image: " << fillmapPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!highlightPath.empty()) {
    // Base: overlay layer, then paint the top edges in red.
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

  if (!odCsvPath.empty()) {
    if (!wantOd) {
      std::cerr << "Internal error: OD requested but debug info not computed\n";
      return 2;
    }
    if (!ExportOdCsv(odCsvPath, world, dbg, err)) {
      std::cerr << "Failed to write OD CSV: " << odCsvPath << "\n" << err << "\n";
      return 2;
    }
  }

  if (!odGeoJsonPath.empty()) {
    if (!wantOd) {
      std::cerr << "Internal error: OD requested but debug info not computed\n";
      return 2;
    }
    if (!ExportOdGeoJson(odGeoJsonPath, world, dbg, odTop, odMinAmount, err)) {
      std::cerr << "Failed to write OD GeoJSON: " << odGeoJsonPath << "\n" << err << "\n";
      return 2;
    }
  }

  return 0;
}
