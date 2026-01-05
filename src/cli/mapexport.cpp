// proc_isocity_mapexport
//
// Export a world to a single GeoJSON FeatureCollection suitable for GIS tooling.
//
// Design goals:
//  - dependency-free (no external geo/json libs)
//  - deterministic output ordering + float formatting
//  - useful defaults: road centerlines + landuse polygons + optional districts

#include "isocity/DistrictStats.hpp"
#include "isocity/GeoJsonExport.hpp"
#include "isocity/Hash.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Vectorize.hpp"
#include "isocity/World.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0) return false;
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
  errno = 0;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
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

bool EnsureParentDir(const std::string& path)
{
  if (path.empty()) return true;
  try {
    const std::filesystem::path p(path);
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
      << "proc_isocity_mapexport (headless GeoJSON map export)\n\n"
      << "Exports a world as a single GeoJSON FeatureCollection containing:\n"
      << "  - road centerlines (LineString per RoadGraph edge, optional nodes)\n"
      << "  - landuse polygons (Residential/Commercial/Industrial/Park)\n"
      << "  - optional water polygons\n"
      << "  - optional district polygons + per-district summary stats\n\n"
      << "Usage:\n"
      << "  proc_isocity_mapexport [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                       --geojson <out.geojson>\n"
      << "                       [--roads <0|1>] [--road-nodes <0|1>] [--road-tiles <0|1>]\n"
      << "                       [--zones <0|1>] [--parks <0|1>] [--water <0|1>]\n"
      << "                       [--districts <0|1>] [--district-water <0|1>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>                World size for generation (default: 96x96).\n\n"
      << "Outputs:\n"
      << "  --geojson <out.geojson>     Output GeoJSON FeatureCollection.\n\n"
      << "Layers:\n"
      << "  --roads <0|1>               Export road centerlines as LineStrings (default: 1).\n"
      << "  --road-nodes <0|1>          Also export road nodes as Points (default: 0).\n"
      << "  --road-tiles <0|1>          Export road footprint as polygons (default: 0).\n"
      << "  --zones <0|1>               Export zone polygons (default: 1).\n"
      << "  --parks <0|1>               Export park polygons (default: 1).\n"
      << "  --water <0|1>               Export water polygons (default: 1).\n"
      << "  --districts <0|1>           Export district polygons + stats (default: 0).\n"
      << "  --district-water <0|1>      Include water tiles in district polygons (default: 0).\n";
}

const char* LanduseNameForLabel(int label)
{
  // labels used by this tool's overlay grid
  switch (label) {
  case 1: return "residential";
  case 2: return "commercial";
  case 3: return "industrial";
  case 4: return "park";
  default: return "unknown";
  }
}

void WriteGeoJsonCoord(std::ostream& os, double x, double y)
{
  os << '[' << x << ',' << y << ']';
}

void WriteGeoJsonTileCenter(std::ostream& os, const Point& p)
{
  // Put tile centers at half-integer coordinates so they overlay cleanly with
  // tile-corner polygon layers.
  WriteGeoJsonCoord(os, static_cast<double>(p.x) + 0.5, static_cast<double>(p.y) + 0.5);
}

void WriteGeoJsonLineStringCoords(std::ostream& os, const std::vector<Point>& pts)
{
  os << '[';
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (i) os << ',';
    WriteGeoJsonTileCenter(os, pts[i]);
  }
  os << ']';
}

struct LayerAgg {
  int tiles = 0;
  int capacity = 0;
  int occupants = 0;
  int levelSum = 0;
  int minLevel = 999;
  int maxLevel = -999;
};

LayerAgg AggForOverlay(const World& world, Overlay o)
{
  LayerAgg a;
  const int w = world.width();
  const int h = world.height();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != o) continue;
      a.tiles++;
      const int lvl = ClampZoneLevel(t.level);
      a.capacity += CapacityForOverlayLevel(o, lvl);
      a.occupants += static_cast<int>(t.occupants);
      a.levelSum += lvl;
      a.minLevel = std::min(a.minLevel, lvl);
      a.maxLevel = std::max(a.maxLevel, lvl);
    }
  }
  if (a.tiles <= 0) {
    a.minLevel = 0;
    a.maxLevel = 0;
  }
  return a;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string geojsonPath;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  bool includeRoads = true;
  bool includeRoadNodes = false;
  bool includeRoadTiles = false;

  bool includeZones = true;
  bool includeParks = true;
  bool includeWater = true;

  bool includeDistricts = false;
  bool districtIncludeWater = false;

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
    } else if (arg == "--geojson") {
      if (!requireValue(i, val)) {
        std::cerr << "--geojson requires a path\n";
        return 2;
      }
      geojsonPath = val;
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
    } else if (arg == "--roads") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeRoads)) {
        std::cerr << "--roads requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--road-nodes") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeRoadNodes)) {
        std::cerr << "--road-nodes requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--road-tiles") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeRoadTiles)) {
        std::cerr << "--road-tiles requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--zones") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeZones)) {
        std::cerr << "--zones requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--parks") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeParks)) {
        std::cerr << "--parks requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--water") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeWater)) {
        std::cerr << "--water requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--districts") {
      if (!requireValue(i, val) || !ParseBool01(val, &includeDistricts)) {
        std::cerr << "--districts requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--district-water") {
      if (!requireValue(i, val) || !ParseBool01(val, &districtIncludeWater)) {
        std::cerr << "--district-water requires 0 or 1\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (geojsonPath.empty()) {
    std::cerr << "--geojson is required\n";
    PrintHelp();
    return 2;
  }

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::string err;

  if (!loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Load failed: " << err << "\n";
      return 3;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  if (world.width() <= 0 || world.height() <= 0) {
    std::cerr << "Invalid world dimensions\n";
    return 3;
  }

  if (!EnsureParentDir(geojsonPath)) {
    std::cerr << "Failed to create output directory\n";
    return 4;
  }
  std::ofstream os(geojsonPath, std::ios::binary);
  if (!os) {
    std::cerr << "Failed to open output file\n";
    return 4;
  }

  // Deterministic float formatting for tile centers.
  os.setf(std::ios::fixed);
  os << std::setprecision(6);

  const std::uint64_t hashTiles = HashWorld(world, false);
  const std::uint64_t hashAll = HashWorld(world, true);

  os << "{\n";
  os << "  \"type\":\"FeatureCollection\",\n";
  os << "  \"properties\":{\"w\":" << world.width()
     << ",\"h\":" << world.height()
     << ",\"seed\":" << world.seed()
     << ",\"hashTiles\":" << hashTiles
     << ",\"hash\":" << hashAll
     << "},\n";
  os << "  \"features\":[\n";

  bool firstFeature = true;
  auto emitComma = [&]() {
    if (!firstFeature) os << ",\n";
    firstFeature = false;
  };

  const int W = world.width();
  const int H = world.height();

  // --- Water polygons ---
  if (includeWater) {
    std::vector<int> labels(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), 0);
    int waterTiles = 0;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const Tile& t = world.at(x, y);
        if (t.terrain == Terrain::Water) {
          labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] = 1;
          waterTiles++;
        }
      }
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats st{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, W, H, 0, geoms, &st, &vErr)) {
      std::cerr << "Vectorize water failed: " << vErr << "\n";
      return 5;
    }

    for (const LabeledGeometry& g : geoms) {
      if (g.label != 1) continue;
      emitComma();
      os << "    {\"type\":\"Feature\",\"properties\":{";
      os << "\"layer\":\"water\",\"tiles\":" << waterTiles;
      os << "},\"geometry\":";
      WriteGeoJsonGeometry(os, g.geom);
      os << "}";
    }
  }

  // --- Landuse (zones + parks) polygons ---
  if (includeZones || includeParks) {
    // Labels:
    //  0 = background
    //  1 = Residential
    //  2 = Commercial
    //  3 = Industrial
    //  4 = Park
    std::vector<int> labels(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), 0);

    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const Tile& t = world.at(x, y);
        int lbl = 0;
        switch (t.overlay) {
        case Overlay::Residential: lbl = 1; break;
        case Overlay::Commercial: lbl = 2; break;
        case Overlay::Industrial: lbl = 3; break;
        case Overlay::Park: lbl = 4; break;
        default: lbl = 0; break;
        }
        labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] = lbl;
      }
    }

    const LayerAgg resAgg = AggForOverlay(world, Overlay::Residential);
    const LayerAgg comAgg = AggForOverlay(world, Overlay::Commercial);
    const LayerAgg indAgg = AggForOverlay(world, Overlay::Industrial);
    LayerAgg parkAgg;
    {
      parkAgg.tiles = 0;
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          if (world.at(x, y).overlay == Overlay::Park) parkAgg.tiles++;
        }
      }
      parkAgg.capacity = 0;
      parkAgg.occupants = 0;
      parkAgg.levelSum = 0;
      parkAgg.minLevel = 0;
      parkAgg.maxLevel = 0;
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats st{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, W, H, 0, geoms, &st, &vErr)) {
      std::cerr << "Vectorize landuse failed: " << vErr << "\n";
      return 5;
    }

    auto emitLanduse = [&](int label, const LayerAgg& agg) {
      for (const LabeledGeometry& g : geoms) {
        if (g.label != label) continue;
        emitComma();
        os << "    {\"type\":\"Feature\",\"properties\":{";
        os << "\"layer\":\"landuse\",\"kind\":\"" << LanduseNameForLabel(label) << "\"";
        os << ",\"tiles\":" << agg.tiles;
        if (label >= 1 && label <= 3) {
          os << ",\"capacity\":" << agg.capacity;
          os << ",\"occupants\":" << agg.occupants;
          if (agg.tiles > 0) {
            const double avgLvl = static_cast<double>(agg.levelSum) / static_cast<double>(agg.tiles);
            os << ",\"avgLevel\":" << avgLvl;
          } else {
            os << ",\"avgLevel\":0";
          }
          os << ",\"minLevel\":" << agg.minLevel;
          os << ",\"maxLevel\":" << agg.maxLevel;
        }
        os << "},\"geometry\":";
        WriteGeoJsonGeometry(os, g.geom);
        os << "}";
      }
    };

    if (includeZones) {
      emitLanduse(1, resAgg);
      emitLanduse(2, comAgg);
      emitLanduse(3, indAgg);
    }
    if (includeParks) {
      // Parks are level-less.
      emitLanduse(4, parkAgg);
    }
  }

  // --- Road footprint polygons (optional) ---
  if (includeRoadTiles) {
    std::vector<int> labels(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), 0);
    int roadTiles = 0;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay == Overlay::Road) {
          labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] = 1;
          roadTiles++;
        }
      }
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats st{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, W, H, 0, geoms, &st, &vErr)) {
      std::cerr << "Vectorize road tiles failed: " << vErr << "\n";
      return 5;
    }

    for (const LabeledGeometry& g : geoms) {
      if (g.label != 1) continue;
      emitComma();
      os << "    {\"type\":\"Feature\",\"properties\":{";
      os << "\"layer\":\"road_tiles\",\"tiles\":" << roadTiles;
      os << "},\"geometry\":";
      WriteGeoJsonGeometry(os, g.geom);
      os << "}";
    }
  }

  // --- Road centerlines (RoadGraph) ---
  if (includeRoads) {
    const RoadGraph g = BuildRoadGraph(world);

    if (includeRoadNodes) {
      for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        const RoadGraphNode& n = g.nodes[i];
        emitComma();
        os << "    {\"type\":\"Feature\",\"properties\":{";
        os << "\"layer\":\"road_node\",\"id\":" << i;
        os << ",\"degree\":" << n.edges.size();
        os << "},\"geometry\":{\"type\":\"Point\",\"coordinates\":";
        WriteGeoJsonTileCenter(os, n.pos);
        os << "}}";
      }
    }

    for (std::size_t ei = 0; ei < g.edges.size(); ++ei) {
      const RoadGraphEdge& e = g.edges[ei];
      int minLvl = 999;
      int maxLvl = -999;
      int sumLvl = 0;
      int waterCount = 0;
      for (const Point& p : e.tiles) {
        if (!world.inBounds(p.x, p.y)) continue;
        const Tile& t = world.at(p.x, p.y);
        const int lvl = std::clamp(static_cast<int>(t.level), 1, 3);
        minLvl = std::min(minLvl, lvl);
        maxLvl = std::max(maxLvl, lvl);
        sumLvl += lvl;
        if (t.terrain == Terrain::Water) waterCount++;
      }
      if (e.tiles.empty()) {
        minLvl = 0;
        maxLvl = 0;
      }
      const double avgLvl = (!e.tiles.empty()) ? (static_cast<double>(sumLvl) / static_cast<double>(e.tiles.size())) : 0.0;

      emitComma();
      os << "    {\"type\":\"Feature\",\"properties\":{";
      os << "\"layer\":\"road\",\"id\":" << ei;
      os << ",\"a\":" << e.a << ",\"b\":" << e.b;
      os << ",\"length\":" << e.length;
      os << ",\"tiles\":" << e.tiles.size();
      os << ",\"minLevel\":" << minLvl;
      os << ",\"maxLevel\":" << maxLvl;
      os << ",\"avgLevel\":" << avgLvl;
      os << ",\"waterTiles\":" << waterCount;
      os << "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":";
      WriteGeoJsonLineStringCoords(os, e.tiles);
      os << "}}";
    }
  }

  // --- District polygons + stats (optional) ---
  if (includeDistricts) {
    // Compute land value so district stats can report avgLandValue and tax revenue.
    const LandValueResult lv = ComputeLandValue(world, LandValueConfig{}, nullptr, nullptr);
    const std::vector<float>* lvField = (lv.value.size() == static_cast<std::size_t>(W) * static_cast<std::size_t>(H)) ? &lv.value : nullptr;
    const DistrictStatsResult ds = ComputeDistrictStats(world, simCfg, lvField, nullptr);

    const int bg = -1;
    std::vector<int> labels(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), bg);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const Tile& t = world.at(x, y);
        if (!districtIncludeWater && t.terrain == Terrain::Water) {
          labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] = bg;
        } else {
          labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] = static_cast<int>(t.district);
        }
      }
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats st{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, W, H, bg, geoms, &st, &vErr)) {
      std::cerr << "Vectorize districts failed: " << vErr << "\n";
      return 5;
    }

    for (const LabeledGeometry& g : geoms) {
      const int id = g.label;
      if (id < 0 || id >= kDistrictCount) continue;
      const DistrictSummary& d = ds.districts[static_cast<std::size_t>(id)];

      emitComma();
      os << "    {\"type\":\"Feature\",\"properties\":{";
      os << "\"layer\":\"district\",\"id\":" << id;
      os << ",\"tiles\":" << d.tiles;
      os << ",\"landTiles\":" << d.landTiles;
      os << ",\"waterTiles\":" << d.waterTiles;
      os << ",\"roads\":" << d.roads;
      os << ",\"parks\":" << d.parks;
      os << ",\"resTiles\":" << d.resTiles;
      os << ",\"comTiles\":" << d.comTiles;
      os << ",\"indTiles\":" << d.indTiles;
      os << ",\"population\":" << d.population;
      os << ",\"housingCapacity\":" << d.housingCapacity;
      os << ",\"jobsCapacity\":" << d.jobsCapacity;
      os << ",\"employed\":" << d.employed;
      os << ",\"avgLandValue\":" << static_cast<double>(d.avgLandValue);
      os << ",\"taxRevenue\":" << d.taxRevenue;
      os << ",\"maintenanceCost\":" << d.maintenanceCost;
      os << ",\"net\":" << d.net;
      os << "},\"geometry\":";
      WriteGeoJsonGeometry(os, g.geom);
      os << "}";
    }
  }

  os << "\n  ]\n";
  os << "}\n";

  return 0;
}
