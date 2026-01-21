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
#include "isocity/Geometry.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Random.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Vectorize.hpp"
#include "isocity/World.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <cmath>
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

bool ParseDouble(const std::string& s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

// Parse "lon,lat" into two doubles.
bool ParseLonLat(const std::string& s, double* outLon, double* outLat)
{
  if (!outLon || !outLat) return false;
  const std::size_t pos = s.find(',');
  if (pos == std::string::npos) return false;
  double lon = 0.0;
  double lat = 0.0;
  if (!ParseDouble(s.substr(0, pos), &lon)) return false;
  if (!ParseDouble(s.substr(pos + 1), &lat)) return false;
  *outLon = lon;
  *outLat = lat;
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
      << "                       [--geojson <out.geojson>]\n"
      << "                       [--mapbox <out_dir>]\n"
      << "                       [--roads <0|1>] [--road-nodes <0|1>] [--road-tiles <0|1>]\n"
      << "                       [--zones <0|1>] [--buildings <0|1>] [--parks <0|1>] [--water <0|1>]\n"
      << "                       [--districts <0|1>] [--district-water <0|1>]\n"
      << "                       [--mapbox-origin <lon,lat>] [--mapbox-meters-per-tile <m>]\n"
      << "                       [--mapbox-flip-y <0|1>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>                World size for generation (default: 96x96).\n\n"
      << "Outputs:\n"
      << "  --geojson <out.geojson>     Output GeoJSON FeatureCollection.\n\n"
      << "  --mapbox <out_dir>          Write a Mapbox/MapLibre-ready bundle:\n"
      << "                             - world.geojson (WGS84 lon/lat, approximate)\n"
      << "                             - style.json (Mapbox Style Spec v8)\n"
      << "                             - index.html + index_inline.html (MapLibre viewer)\n\n"
      << "Layers:\n"
      << "  --roads <0|1>               Export road centerlines as LineStrings (default: 1).\n"
      << "  --road-nodes <0|1>          Also export road nodes as Points (default: 0).\n"
      << "  --road-tiles <0|1>          Export road footprint as polygons (default: 0).\n"
      << "  --zones <0|1>               Export zone polygons (default: 1).\n"
      << "  --buildings <0|1>           Export procedural building footprints (zones + civic) for 3D extrusions (default: 0).\n"
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

// -------------------------------------------------------------------------------------------------
// Mapbox / MapLibre bundle helpers
// -------------------------------------------------------------------------------------------------

struct ExportLayers {
  bool roads = true;
  bool roadNodes = false;
  bool roadTiles = false;

  bool zones = true;
  bool buildings = false;
  bool parks = true;
  bool water = true;

  bool districts = false;
  bool districtIncludeWater = false;
};

struct GeoRef {
  // Interpreted as WGS84 degrees.
  double originLon = 0.0;
  double originLat = 0.0;

  // Tile size for the lon/lat conversion. This is an *approximate* equirectangular
  // projection (good enough for small local maps and Mapbox/MapLibre viewing).
  double metersPerTile = 10.0;

  // If true, y+ goes "down" in tile-space (screen coords), so lat decreases.
  bool flipY = true;
};

enum class CoordSpace {
  TileGrid,
  Wgs84LonLat,
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0; // WGS84 semi-major axis

inline double MetersToDegrees() { return 180.0 / (kPi * kEarthRadiusM); }

void TileCornerToLonLat(int x, int y, const GeoRef& g, double* outLon, double* outLat)
{
  if (!outLon || !outLat) return;
  const double dxM = static_cast<double>(x) * g.metersPerTile;
  const double dyTiles = static_cast<double>(y);
  const double dyM = (g.flipY ? -dyTiles : dyTiles) * g.metersPerTile;
  const double cosLat0 = std::max(1e-6, std::cos(g.originLat * kPi / 180.0));
  const double m2d = MetersToDegrees();
  *outLon = g.originLon + (dxM * m2d) / cosLat0;
  *outLat = g.originLat + (dyM * m2d);
}

void TileCenterToLonLat(const Point& p, const GeoRef& g, double* outLon, double* outLat)
{
  if (!outLon || !outLat) return;
  const double cx = static_cast<double>(p.x) + 0.5;
  const double cy = static_cast<double>(p.y) + 0.5;
  const double dxM = cx * g.metersPerTile;
  const double dyM = (g.flipY ? -cy : cy) * g.metersPerTile;
  const double cosLat0 = std::max(1e-6, std::cos(g.originLat * kPi / 180.0));
  const double m2d = MetersToDegrees();
  *outLon = g.originLon + (dxM * m2d) / cosLat0;
  *outLat = g.originLat + (dyM * m2d);
}

void ComputeLonLatBbox(int W, int H, const GeoRef& g, double* outMinLon, double* outMinLat, double* outMaxLon,
                       double* outMaxLat)
{
  if (!outMinLon || !outMinLat || !outMaxLon || !outMaxLat) return;
  double lon00 = 0.0, lat00 = 0.0;
  double lonW0 = 0.0, latW0 = 0.0;
  double lon0H = 0.0, lat0H = 0.0;
  double lonWH = 0.0, latWH = 0.0;
  TileCornerToLonLat(0, 0, g, &lon00, &lat00);
  TileCornerToLonLat(W, 0, g, &lonW0, &latW0);
  TileCornerToLonLat(0, H, g, &lon0H, &lat0H);
  TileCornerToLonLat(W, H, g, &lonWH, &latWH);
  *outMinLon = std::min({lon00, lonW0, lon0H, lonWH});
  *outMaxLon = std::max({lon00, lonW0, lon0H, lonWH});
  *outMinLat = std::min({lat00, latW0, lat0H, latWH});
  *outMaxLat = std::max({lat00, latW0, lat0H, latWH});
}

class GeoEmitter {
public:
  GeoEmitter(JsonWriter& jw, int W, int H, CoordSpace space, const GeoRef& g)
      : jw_(jw), W_(W), H_(H), space_(space), georef_(g)
  {
    if (space_ == CoordSpace::TileGrid) {
      bboxMinX_ = 0.0;
      bboxMinY_ = 0.0;
      bboxMaxX_ = static_cast<double>(W_);
      bboxMaxY_ = static_cast<double>(H_);
    } else {
      ComputeLonLatBbox(W_, H_, georef_, &bboxMinX_, &bboxMinY_, &bboxMaxX_, &bboxMaxY_);
    }
  }

  CoordSpace space() const { return space_; }
  const GeoRef& georef() const { return georef_; }

  double bboxMinX() const { return bboxMinX_; }
  double bboxMinY() const { return bboxMinY_; }
  double bboxMaxX() const { return bboxMaxX_; }
  double bboxMaxY() const { return bboxMaxY_; }

  void writeBbox()
  {
    jw_.beginArray();
    jw_.numberValue(bboxMinX_);
    jw_.numberValue(bboxMinY_);
    jw_.numberValue(bboxMaxX_);
    jw_.numberValue(bboxMaxY_);
    jw_.endArray();
  }

  void writeTileCenterCoord(const Point& p)
  {
    double x = 0.0;
    double y = 0.0;
    outCenter(p, &x, &y);
    jw_.beginArray();
    jw_.numberValue(x);
    jw_.numberValue(y);
    jw_.endArray();
  }

  void writeLineStringTileCenters(const std::vector<Point>& pts)
  {
    jw_.beginArray();
    for (const Point& p : pts) {
      writeTileCenterCoord(p);
    }
    jw_.endArray();
  }

  void writeGeometry(const VectorMultiPolygon& mp)
  {
    if (mp.polygons.empty()) {
      jw_.beginObject();
      jw_.key("type");
      jw_.stringValue("GeometryCollection");
      jw_.key("geometries");
      jw_.beginArray();
      jw_.endArray();
      jw_.endObject();
      return;
    }

    if (mp.polygons.size() == 1) {
      jw_.beginObject();
      jw_.key("type");
      jw_.stringValue("Polygon");
      jw_.key("coordinates");
      writePolygonCoords(mp.polygons[0]);
      jw_.endObject();
      return;
    }

    jw_.beginObject();
    jw_.key("type");
    jw_.stringValue("MultiPolygon");
    jw_.key("coordinates");
    writeMultiPolygonCoords(mp);
    jw_.endObject();
  }

private:
  void outCorner(const IPoint& p, double* outX, double* outY) const
  {
    if (!outX || !outY) return;
    if (space_ == CoordSpace::TileGrid) {
      *outX = static_cast<double>(p.x);
      *outY = static_cast<double>(p.y);
    } else {
      TileCornerToLonLat(p.x, p.y, georef_, outX, outY);
    }
  }

  void outCenter(const Point& p, double* outX, double* outY) const
  {
    if (!outX || !outY) return;
    if (space_ == CoordSpace::TileGrid) {
      *outX = static_cast<double>(p.x) + 0.5;
      *outY = static_cast<double>(p.y) + 0.5;
    } else {
      TileCenterToLonLat(p, georef_, outX, outY);
    }
  }

  void writeRing(const std::vector<IPoint>& ring)
  {
    jw_.beginArray();
    for (const IPoint& p : ring) {
      jw_.beginArray();
      if (space_ == CoordSpace::TileGrid) {
        jw_.intValue(p.x);
        jw_.intValue(p.y);
      } else {
        double x = 0.0;
        double y = 0.0;
        outCorner(p, &x, &y);
        jw_.numberValue(x);
        jw_.numberValue(y);
      }
      jw_.endArray();
    }
    jw_.endArray();
  }

  void writePolygonCoords(const VectorPolygon& poly)
  {
    jw_.beginArray();
    writeRing(poly.outer);
    for (const auto& hole : poly.holes) {
      writeRing(hole);
    }
    jw_.endArray();
  }

  void writeMultiPolygonCoords(const VectorMultiPolygon& mp)
  {
    jw_.beginArray();
    for (const auto& poly : mp.polygons) {
      writePolygonCoords(poly);
    }
    jw_.endArray();
  }

  JsonWriter& jw_;
  int W_ = 0;
  int H_ = 0;
  CoordSpace space_ = CoordSpace::TileGrid;
  GeoRef georef_{};
  double bboxMinX_ = 0.0;
  double bboxMinY_ = 0.0;
  double bboxMaxX_ = 0.0;
  double bboxMaxY_ = 0.0;
};

int ExportWorldGeoJson(std::ostream& os, const World& world, const SimConfig& simCfg, const ExportLayers& layers,
                       CoordSpace coordSpace, const GeoRef& georef, std::string* outErr)
{
  const int W = world.width();
  const int H = world.height();

  const std::uint64_t hashTiles = HashWorld(world, false);
  const std::uint64_t hashAll = HashWorld(world, true);

  JsonWriteOptions jopt{};
  jopt.pretty = true;
  jopt.indent = 2;
  jopt.sortKeys = false;

  JsonWriter jw(os, jopt);
  GeoEmitter emit(jw, W, H, coordSpace, georef);

  jw.beginObject();
  jw.key("type");
  jw.stringValue("FeatureCollection");

  // Useful for GIS tooling and sanity-checking.
  jw.key("bbox");
  emit.writeBbox();

  jw.key("properties");
  jw.beginObject();
  jw.key("w");
  jw.intValue(W);
  jw.key("h");
  jw.intValue(H);
  jw.key("seed");
  jw.uintValue(world.seed());
  jw.key("hashTiles");
  jw.uintValue(hashTiles);
  jw.key("hash");
  jw.uintValue(hashAll);

  if (coordSpace == CoordSpace::TileGrid) {
    jw.key("coordSpace");
    jw.stringValue("tile_grid");
    jw.key("polygonSpace");
    jw.stringValue("tile_corner");
    jw.key("lineSpace");
    jw.stringValue("tile_center");
  } else {
    jw.key("coordSpace");
    jw.stringValue("wgs84_lonlat");
    jw.key("polygonSpace");
    jw.stringValue("wgs84_lonlat");
    jw.key("lineSpace");
    jw.stringValue("wgs84_lonlat");
    jw.key("originLon");
    jw.numberValue(georef.originLon);
    jw.key("originLat");
    jw.numberValue(georef.originLat);
    jw.key("metersPerTile");
    jw.numberValue(georef.metersPerTile);
    jw.key("flipY");
    jw.boolValue(georef.flipY);
  }
  jw.endObject();

  jw.key("features");
  jw.beginArray();

  // --- Water polygons ---
  if (layers.water) {
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
      if (outErr) *outErr = "Vectorize water failed: " + vErr;
      return 5;
    }

    for (LabeledGeometry& g : geoms) {
      if (g.label != 1) continue;
      SimplifyVectorMultiPolygonCollinear(g.geom);

      jw.beginObject();
      jw.key("type");
      jw.stringValue("Feature");

      jw.key("properties");
      jw.beginObject();
      jw.key("layer");
      jw.stringValue("water");
      jw.key("tiles");
      jw.intValue(waterTiles);
      jw.endObject();

      jw.key("geometry");
      emit.writeGeometry(g.geom);
      jw.endObject();
    }
  }

  // --- Landuse (zones + parks) polygons ---
  if (layers.zones || layers.parks) {
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
      if (outErr) *outErr = "Vectorize landuse failed: " + vErr;
      return 5;
    }

    auto emitLanduse = [&](int label, const LayerAgg& agg) {
      for (LabeledGeometry& g : geoms) {
        if (g.label != label) continue;
        SimplifyVectorMultiPolygonCollinear(g.geom);

        jw.beginObject();
        jw.key("type");
        jw.stringValue("Feature");

        jw.key("properties");
        jw.beginObject();
        jw.key("layer");
        jw.stringValue("landuse");
        jw.key("kind");
        jw.stringValue(LanduseNameForLabel(label));
        jw.key("tiles");
        jw.intValue(agg.tiles);

        if (label >= 1 && label <= 3) {
          jw.key("capacity");
          jw.intValue(agg.capacity);
          jw.key("occupants");
          jw.intValue(agg.occupants);
          jw.key("avgLevel");
          jw.numberValue((agg.tiles > 0) ? (static_cast<double>(agg.levelSum) / static_cast<double>(agg.tiles)) : 0.0);
          jw.key("minLevel");
          jw.intValue(agg.minLevel);
          jw.key("maxLevel");
          jw.intValue(agg.maxLevel);
        }

        jw.endObject();

        jw.key("geometry");
        emit.writeGeometry(g.geom);
        jw.endObject();
      }
    };

    if (layers.zones) {
      emitLanduse(1, resAgg);
      emitLanduse(2, comAgg);
      emitLanduse(3, indAgg);
    }
    if (layers.parks) {
      // Parks are level-less.
      emitLanduse(4, parkAgg);
    }
  }

  // --- Buildings (procedural footprints) polygons (optional) ---
  //
  // For Mapbox/MapLibre 3D (fill-extrusion), we export building footprints as polygons
  // with per-feature height/base properties.
  //
  // To keep the output readable (and avoid one monolithic extrusion per zone), we do a
  // small deterministic "parcelization": zoned tiles of the same kind+level are
  // stochastically merged into small multi-tile building footprints using HashCoords32.
  //
  // Additionally, we compute a simple road distance-field (Manhattan steps) and use it
  // as a procedural signal to bias heights (e.g., commercial near roads tends to be taller).
  if (layers.buildings) {
    const int N = W * H;
    const auto idxOf = [&](int x, int y) -> int { return y * W + x; };

    auto frac01 = [](std::uint32_t h) -> float {
      return static_cast<float>(h & 0x00FFFFFFu) / static_cast<float>(1u << 24);
    };

    // --- Road distance field (Manhattan BFS). ---
    std::vector<int> roadDist(static_cast<std::size_t>(N), 1 << 20);
    std::vector<int> q;
    q.reserve(static_cast<std::size_t>(N));
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (world.at(x, y).overlay == Overlay::Road) {
          const int idx = idxOf(x, y);
          roadDist[static_cast<std::size_t>(idx)] = 0;
          q.push_back(idx);
        }
      }
    }
    for (std::size_t qi = 0; qi < q.size(); ++qi) {
      const int idx = q[qi];
      const int x = idx % W;
      const int y = idx / W;
      const int d = roadDist[static_cast<std::size_t>(idx)];

      auto relax = [&](int nx, int ny) {
        if (nx < 0 || nx >= W || ny < 0 || ny >= H) return;
        const int nidx = idxOf(nx, ny);
        int& nd = roadDist[static_cast<std::size_t>(nidx)];
        if (nd > d + 1) {
          nd = d + 1;
          q.push_back(nidx);
        }
      };

      relax(x - 1, y);
      relax(x + 1, y);
      relax(x, y - 1);
      relax(x, y + 1);
    }

    // --- Union-Find for stochastic building parcels. ---
    struct DSU {
      std::vector<int> parent;
      std::vector<int> size;

      explicit DSU(int n) : parent(static_cast<std::size_t>(n)), size(static_cast<std::size_t>(n), 1)
      {
        for (int i = 0; i < n; ++i) parent[static_cast<std::size_t>(i)] = i;
      }

      int find(int a)
      {
        int p = parent[static_cast<std::size_t>(a)];
        while (p != a) {
          const int gp = parent[static_cast<std::size_t>(p)];
          parent[static_cast<std::size_t>(a)] = gp;
          a = p;
          p = gp;
        }
        return a;
      }

      void unite(int a, int b)
      {
        a = find(a);
        b = find(b);
        if (a == b) return;
        int sa = size[static_cast<std::size_t>(a)];
        int sb = size[static_cast<std::size_t>(b)];
        if (sb > sa) {
          std::swap(a, b);
          std::swap(sa, sb);
        }
        parent[static_cast<std::size_t>(b)] = a;
        size[static_cast<std::size_t>(a)] = sa + sb;
      }
    };

    struct BuildTile {
      int kindId = 0; // 0=none, 1=res,2=com,3=ind,4=school,5=hospital,6=police,7=fire
      int level = 1;  // for zones only (1..3)
      int occupants = 0;
      int capacity = 0;
    };

    auto kindIdForOverlay = [](Overlay o) -> int {
      switch (o) {
      case Overlay::Residential: return 1;
      case Overlay::Commercial: return 2;
      case Overlay::Industrial: return 3;
      case Overlay::School: return 4;
      case Overlay::Hospital: return 5;
      case Overlay::PoliceStation: return 6;
      case Overlay::FireStation: return 7;
      default: return 0;
      }
    };

    auto kindNameForId = [](int id) -> const char* {
      switch (id) {
      case 1: return "residential";
      case 2: return "commercial";
      case 3: return "industrial";
      case 4: return "school";
      case 5: return "hospital";
      case 6: return "police";
      case 7: return "fire";
      default: return "unknown";
      }
    };

    auto mergeProb = [](int kindId, int lvl) -> float {
      const int l = std::clamp(lvl, 1, 3);
      // Lower probabilities => smaller parcels (more individual buildings).
      switch (kindId) {
      case 1: return 0.55f - 0.06f * static_cast<float>(l - 1); // residential
      case 2: return 0.48f - 0.08f * static_cast<float>(l - 1); // commercial
      case 3: return 0.42f - 0.06f * static_cast<float>(l - 1); // industrial
      default: return 0.0f; // civic buildings remain single-tile
      }
    };

    const std::uint32_t seedBase = static_cast<std::uint32_t>(world.seed() ^ 0xB01D1E5u);

    std::vector<BuildTile> bt(static_cast<std::size_t>(N));
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const Tile& t = world.at(x, y);
        const int kindId = kindIdForOverlay(t.overlay);
        if (kindId == 0) continue;

        BuildTile b;
        b.kindId = kindId;
        b.occupants = static_cast<int>(t.occupants);

        if (IsZoneOverlay(t.overlay)) {
          b.level = ClampZoneLevel(t.level);
          b.capacity = CapacityForTile(t);
        } else {
          // Civic/service buildings are modeled as single-tile facilities.
          b.level = 1;
          b.capacity = 0;
        }

        bt[static_cast<std::size_t>(idxOf(x, y))] = b;
      }
    }

    DSU dsu(N);

    // Stochastic edge merges (right + down) for zone tiles.
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int idx = idxOf(x, y);
        const BuildTile& a = bt[static_cast<std::size_t>(idx)];
        if (a.kindId == 0) continue;

        const float p = mergeProb(a.kindId, a.level);
        if (p <= 0.0f) continue;

        // Right neighbor.
        if (x + 1 < W) {
          const int j = idxOf(x + 1, y);
          const BuildTile& b = bt[static_cast<std::size_t>(j)];
          if (b.kindId == a.kindId && b.level == a.level) {
            const std::uint32_t h = HashCoords32(x, y, seedBase ^ 0x52E17u);
            if (frac01(h) < p) dsu.unite(idx, j);
          }
        }

        // Down neighbor.
        if (y + 1 < H) {
          const int j = idxOf(x, y + 1);
          const BuildTile& b = bt[static_cast<std::size_t>(j)];
          if (b.kindId == a.kindId && b.level == a.level) {
            const std::uint32_t h = HashCoords32(x, y, seedBase ^ 0xD00Fu);
            if (frac01(h) < p) dsu.unite(idx, j);
          }
        }
      }
    }

    struct BuildingAgg {
      int kindId = 0;
      int level = 1;
      int tiles = 0;
      int occupants = 0;
      int capacity = 0;
      int minRoadDist = 1 << 20;
      int sumRoadDist = 0;
      std::uint32_t seed = 0;
      double heightM = 0.0;
    };

    std::vector<int> rootToLabel(static_cast<std::size_t>(N), 0);
    std::vector<int> labels(static_cast<std::size_t>(N), 0);
    std::vector<BuildingAgg> agg;
    agg.reserve(static_cast<std::size_t>(N / 4 + 4));
    agg.push_back(BuildingAgg{}); // label 0 unused

    // Assign stable sequential labels in scanline order + aggregate stats.
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int idx = idxOf(x, y);
        const BuildTile& t = bt[static_cast<std::size_t>(idx)];
        if (t.kindId == 0) continue;

        const int r = dsu.find(idx);
        int lbl = rootToLabel[static_cast<std::size_t>(r)];
        if (lbl == 0) {
          BuildingAgg a;
          a.kindId = t.kindId;
          a.level = t.level;
          a.seed = HashCoords32(x, y, seedBase ^ 0xA81CEu);
          agg.push_back(a);
          lbl = static_cast<int>(agg.size()) - 1;
          rootToLabel[static_cast<std::size_t>(r)] = lbl;
        }

        labels[static_cast<std::size_t>(idx)] = lbl;
        BuildingAgg& a = agg[static_cast<std::size_t>(lbl)];
        a.tiles += 1;
        a.occupants += t.occupants;
        a.capacity += t.capacity;
        const int rd = roadDist[static_cast<std::size_t>(idx)];
        a.minRoadDist = std::min(a.minRoadDist, rd);
        a.sumRoadDist += rd;
      }
    }

    // Derive a plausible height for each building parcel.
    for (std::size_t i = 1; i < agg.size(); ++i) {
      BuildingAgg& a = agg[i];
      const float occRatio = (a.capacity > 0) ? std::clamp(static_cast<float>(a.occupants) / static_cast<float>(a.capacity), 0.0f, 1.0f) : 0.0f;
      const float roadNear = 1.0f / (1.0f + 0.35f * static_cast<float>(std::max(0, a.minRoadDist)));

      float base = 5.0f;
      float perLvl = 3.5f;
      float occBoost = 1.0f;
      float roadBoost = 0.0f;
      float jitterRange = 1.25f;

      switch (a.kindId) {
      case 1: // residential
        base = 4.5f;
        perLvl = 3.6f;
        occBoost = 1.6f;
        roadBoost = 0.6f;
        jitterRange = 1.25f;
        break;
      case 2: // commercial
        base = 6.0f;
        perLvl = 5.0f;
        occBoost = 2.2f;
        roadBoost = 2.6f;
        jitterRange = 2.10f;
        break;
      case 3: // industrial
        base = 5.5f;
        perLvl = 4.3f;
        occBoost = 1.8f;
        roadBoost = 1.4f;
        jitterRange = 1.80f;
        break;
      case 4: // school
        base = 9.0f;
        perLvl = 0.0f;
        occBoost = 0.0f;
        roadBoost = 0.4f;
        jitterRange = 1.0f;
        break;
      case 5: // hospital
        base = 14.0f;
        perLvl = 0.0f;
        occBoost = 0.0f;
        roadBoost = 0.8f;
        jitterRange = 2.4f;
        break;
      case 6: // police
        base = 10.0f;
        perLvl = 0.0f;
        occBoost = 0.0f;
        roadBoost = 0.5f;
        jitterRange = 1.4f;
        break;
      case 7: // fire
        base = 10.5f;
        perLvl = 0.0f;
        occBoost = 0.0f;
        roadBoost = 0.5f;
        jitterRange = 1.6f;
        break;
      default: break;
      }

      const float lvlF = static_cast<float>(std::max(1, a.level));
      const std::uint32_t hj = HashCoords32(a.tiles, a.level, a.seed ^ 0x9E3779B9u);
      const float jitter = (frac01(hj) - 0.5f) * 2.0f * jitterRange;

      float h = base + perLvl * lvlF + occBoost * occRatio + roadBoost * roadNear + jitter;
      // Multi-tile buildings read nicer when slightly taller.
      if (a.tiles >= 3) h += 0.35f * std::log1p(static_cast<float>(a.tiles));

      a.heightM = std::max(2.0, static_cast<double>(h));
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats st{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, W, H, 0, geoms, &st, &vErr)) {
      if (outErr) *outErr = "Vectorize buildings failed: " + vErr;
      return 5;
    }

    for (LabeledGeometry& g : geoms) {
      if (g.label <= 0) continue;
      if (static_cast<std::size_t>(g.label) >= agg.size()) continue;
      SimplifyVectorMultiPolygonCollinear(g.geom);

      const BuildingAgg& a = agg[static_cast<std::size_t>(g.label)];

      jw.beginObject();
      jw.key("type");
      jw.stringValue("Feature");

      jw.key("properties");
      jw.beginObject();
      jw.key("layer");
      jw.stringValue("building");
      jw.key("kind");
      jw.stringValue(kindNameForId(a.kindId));
      jw.key("level");
      jw.intValue(a.level);
      jw.key("tiles");
      jw.intValue(a.tiles);
      jw.key("occupants");
      jw.intValue(a.occupants);
      jw.key("capacity");
      jw.intValue(a.capacity);
      jw.key("minRoadDist");
      jw.intValue((a.minRoadDist >= (1 << 19)) ? -1 : a.minRoadDist);
      jw.key("height_m");
      jw.numberValue(a.heightM);
      jw.key("base_m");
      jw.numberValue(0.0);
      jw.endObject();

      jw.key("geometry");
      emit.writeGeometry(g.geom);
      jw.endObject();
    }
  }

  // --- Road footprint polygons (optional) ---
  if (layers.roadTiles) {
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
      if (outErr) *outErr = "Vectorize road tiles failed: " + vErr;
      return 5;
    }

    for (LabeledGeometry& g : geoms) {
      if (g.label != 1) continue;
      SimplifyVectorMultiPolygonCollinear(g.geom);

      jw.beginObject();
      jw.key("type");
      jw.stringValue("Feature");

      jw.key("properties");
      jw.beginObject();
      jw.key("layer");
      jw.stringValue("road_tiles");
      jw.key("tiles");
      jw.intValue(roadTiles);
      jw.endObject();

      jw.key("geometry");
      emit.writeGeometry(g.geom);
      jw.endObject();
    }
  }

  // --- Road centerlines (RoadGraph) ---
  if (layers.roads) {
    const RoadGraph g = BuildRoadGraph(world);

    if (layers.roadNodes) {
      for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        const RoadGraphNode& n = g.nodes[i];

        jw.beginObject();
        jw.key("type");
        jw.stringValue("Feature");

        jw.key("properties");
        jw.beginObject();
        jw.key("layer");
        jw.stringValue("road_node");
        jw.key("id");
        jw.intValue(static_cast<int>(i));
        jw.key("degree");
        jw.intValue(static_cast<int>(n.edges.size()));
        jw.endObject();

        jw.key("geometry");
        jw.beginObject();
        jw.key("type");
        jw.stringValue("Point");
        jw.key("coordinates");
        emit.writeTileCenterCoord(n.pos);
        jw.endObject();

        jw.endObject();
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
      const double avgLvl =
          (!e.tiles.empty()) ? (static_cast<double>(sumLvl) / static_cast<double>(e.tiles.size())) : 0.0;

      std::vector<Point> pts = e.tiles;
      SimplifyPolylineCollinear(pts);

      jw.beginObject();
      jw.key("type");
      jw.stringValue("Feature");

      jw.key("properties");
      jw.beginObject();
      jw.key("layer");
      jw.stringValue("road");
      jw.key("id");
      jw.intValue(static_cast<int>(ei));
      jw.key("a");
      jw.intValue(e.a);
      jw.key("b");
      jw.intValue(e.b);
      jw.key("length");
      jw.intValue(e.length);
      jw.key("tiles");
      jw.intValue(static_cast<int>(e.tiles.size()));
      jw.key("points");
      jw.intValue(static_cast<int>(pts.size()));
      jw.key("minLevel");
      jw.intValue(minLvl);
      jw.key("maxLevel");
      jw.intValue(maxLvl);
      jw.key("avgLevel");
      jw.numberValue(avgLvl);
      jw.key("waterTiles");
      jw.intValue(waterCount);
      jw.endObject();

      jw.key("geometry");
      jw.beginObject();
      jw.key("type");
      jw.stringValue("LineString");
      jw.key("coordinates");
      emit.writeLineStringTileCenters(pts);
      jw.endObject();

      jw.endObject();
    }
  }

  // --- District polygons + stats (optional) ---
  if (layers.districts) {
    // Compute land value so district stats can report avgLandValue and tax revenue.
    const LandValueResult lv = ComputeLandValue(world, LandValueConfig{}, nullptr, nullptr);
    const std::vector<float>* lvField =
        (lv.value.size() == static_cast<std::size_t>(W) * static_cast<std::size_t>(H)) ? &lv.value : nullptr;
    const DistrictStatsResult ds = ComputeDistrictStats(world, simCfg, lvField, nullptr);

    const int bg = -1;
    std::vector<int> labels(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), bg);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const Tile& t = world.at(x, y);
        if (!layers.districtIncludeWater && t.terrain == Terrain::Water) {
          labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] = bg;
        } else {
          labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)] =
              static_cast<int>(t.district);
        }
      }
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats st{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, W, H, bg, geoms, &st, &vErr)) {
      if (outErr) *outErr = "Vectorize districts failed: " + vErr;
      return 5;
    }

    for (LabeledGeometry& g : geoms) {
      const int id = g.label;
      if (id < 0 || id >= kDistrictCount) continue;

      SimplifyVectorMultiPolygonCollinear(g.geom);
      const DistrictSummary& d = ds.districts[static_cast<std::size_t>(id)];

      jw.beginObject();
      jw.key("type");
      jw.stringValue("Feature");

      jw.key("properties");
      jw.beginObject();
      jw.key("layer");
      jw.stringValue("district");
      jw.key("id");
      jw.intValue(id);
      jw.key("tiles");
      jw.intValue(d.tiles);
      jw.key("landTiles");
      jw.intValue(d.landTiles);
      jw.key("waterTiles");
      jw.intValue(d.waterTiles);
      jw.key("roads");
      jw.intValue(d.roads);
      jw.key("parks");
      jw.intValue(d.parks);
      jw.key("resTiles");
      jw.intValue(d.resTiles);
      jw.key("comTiles");
      jw.intValue(d.comTiles);
      jw.key("indTiles");
      jw.intValue(d.indTiles);
      jw.key("zoneTiles");
      jw.intValue(d.zoneTiles);
      jw.key("zoneTilesAccessible");
      jw.intValue(d.zoneTilesAccessible);
      jw.key("population");
      jw.intValue(d.population);
      jw.key("housingCapacity");
      jw.intValue(d.housingCapacity);
      jw.key("jobsCapacity");
      jw.intValue(d.jobsCapacity);
      jw.key("jobsCapacityAccessible");
      jw.intValue(d.jobsCapacityAccessible);
      jw.key("employed");
      jw.intValue(d.employed);
      jw.key("avgLandValue");
      jw.numberValue(static_cast<double>(d.avgLandValue));
      jw.key("taxRevenue");
      jw.intValue(d.taxRevenue);
      jw.key("roadMaintenanceCost");
      jw.intValue(d.roadMaintenanceCost);
      jw.key("parkMaintenanceCost");
      jw.intValue(d.parkMaintenanceCost);
      jw.key("maintenanceCost");
      jw.intValue(d.maintenanceCost);
      jw.key("net");
      jw.intValue(d.net);
      jw.endObject();

      jw.key("geometry");
      emit.writeGeometry(g.geom);
      jw.endObject();
    }
  }

  jw.endArray();  // features
  jw.endObject(); // root

  if (!jw.ok() || !os.good()) {
    if (outErr) *outErr = "GeoJSON write failed: " + jw.error();
    return 6;
  }

  return 0;
}

bool WriteTextFile(const std::string& path, const std::string& content, std::string* outErr)
{
  if (!EnsureParentDir(path)) {
    if (outErr) *outErr = "Failed to create output directory for: " + path;
    return false;
  }
  std::ofstream os(path, std::ios::binary);
  if (!os) {
    if (outErr) *outErr = "Failed to open output file: " + path;
    return false;
  }
  os.write(content.data(), static_cast<std::streamsize>(content.size()));
  if (!os.good()) {
    if (outErr) *outErr = "Failed to write output file: " + path;
    return false;
  }
  return true;
}

bool ReadTextFile(const std::string& path, std::string* out, std::string* outErr)
{
  if (!out) return false;
  std::ifstream is(path, std::ios::binary);
  if (!is) {
    if (outErr) *outErr = "Failed to open input file: " + path;
    return false;
  }
  std::stringstream ss;
  ss << is.rdbuf();
  *out = ss.str();
  return true;
}

bool WriteMapboxStyleJson(const std::string& path, const ExportLayers& layers, const std::string& geojsonRelPath,
                          double centerLon, double centerLat, std::string* outErr)
{
  if (!EnsureParentDir(path)) {
    if (outErr) *outErr = "Failed to create output directory for: " + path;
    return false;
  }
  std::ofstream os(path, std::ios::binary);
  if (!os) {
    if (outErr) *outErr = "Failed to open style output file: " + path;
    return false;
  }

  JsonWriteOptions jopt{};
  jopt.pretty = true;
  jopt.indent = 2;
  jopt.sortKeys = false;
  JsonWriter jw(os, jopt);

  jw.beginObject();
  jw.key("version");
  jw.intValue(8);
  jw.key("name");
  jw.stringValue("ProcIsoCity (generated)");
  jw.key("center");
  jw.beginArray();
  jw.numberValue(centerLon);
  jw.numberValue(centerLat);
  jw.endArray();
  jw.key("zoom");
  jw.numberValue(15.0);

  jw.key("sources");
  jw.beginObject();
  jw.key("world");
  jw.beginObject();
  jw.key("type");
  jw.stringValue("geojson");
  jw.key("data");
  jw.stringValue(geojsonRelPath);
  jw.endObject();
  jw.endObject();

  jw.key("layers");
  jw.beginArray();

  auto writeFilterLayerEq = [&](const char* propName, const char* value) {
    jw.beginArray();
    jw.stringValue("==");
    jw.beginArray();
    jw.stringValue("get");
    jw.stringValue(propName);
    jw.endArray();
    jw.stringValue(value);
    jw.endArray();
  };

  auto writeFilterAll = [&](const char* prop0, const char* val0, const char* prop1, const char* val1) {
    jw.beginArray();
    jw.stringValue("all");
    writeFilterLayerEq(prop0, val0);
    writeFilterLayerEq(prop1, val1);
    jw.endArray();
  };

  // background
  jw.beginObject();
  jw.key("id");
  jw.stringValue("background");
  jw.key("type");
  jw.stringValue("background");
  jw.key("paint");
  jw.beginObject();
  jw.key("background-color");
  jw.stringValue("#f4f1ec");
  jw.endObject();
  jw.endObject();

  if (layers.water) {
    jw.beginObject();
    jw.key("id");
    jw.stringValue("water-fill");
    jw.key("type");
    jw.stringValue("fill");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "water");
    jw.key("paint");
    jw.beginObject();
    jw.key("fill-color");
    jw.stringValue("#a0c8f0");
    jw.key("fill-opacity");
    jw.numberValue(0.95);
    jw.endObject();
    jw.endObject();
  }

  if (layers.parks) {
    jw.beginObject();
    jw.key("id");
    jw.stringValue("parks-fill");
    jw.key("type");
    jw.stringValue("fill");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterAll("layer", "landuse", "kind", "park");
    jw.key("paint");
    jw.beginObject();
    jw.key("fill-color");
    jw.stringValue("#cfe8c7");
    jw.key("fill-opacity");
    jw.numberValue(0.9);
    jw.endObject();
    jw.endObject();
  }

  if (layers.zones) {
    // Residential
    jw.beginObject();
    jw.key("id");
    jw.stringValue("residential-fill");
    jw.key("type");
    jw.stringValue("fill");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterAll("layer", "landuse", "kind", "residential");
    jw.key("paint");
    jw.beginObject();
    jw.key("fill-color");
    jw.stringValue("#f2e5d5");
    jw.key("fill-opacity");
    jw.numberValue(0.9);
    jw.endObject();
    jw.endObject();

    // Commercial
    jw.beginObject();
    jw.key("id");
    jw.stringValue("commercial-fill");
    jw.key("type");
    jw.stringValue("fill");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterAll("layer", "landuse", "kind", "commercial");
    jw.key("paint");
    jw.beginObject();
    jw.key("fill-color");
    jw.stringValue("#f0d4d4");
    jw.key("fill-opacity");
    jw.numberValue(0.85);
    jw.endObject();
    jw.endObject();

    // Industrial
    jw.beginObject();
    jw.key("id");
    jw.stringValue("industrial-fill");
    jw.key("type");
    jw.stringValue("fill");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterAll("layer", "landuse", "kind", "industrial");
    jw.key("paint");
    jw.beginObject();
    jw.key("fill-color");
    jw.stringValue("#e5e5e5");
    jw.key("fill-opacity");
    jw.numberValue(0.85);
    jw.endObject();
    jw.endObject();
  }

  if (layers.buildings) {
    // 3D buildings (fill-extrusion) driven by GeoJSON properties.
    jw.beginObject();
    jw.key("id");
    jw.stringValue("buildings");
    jw.key("type");
    jw.stringValue("fill-extrusion");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "building");
    jw.key("paint");
    jw.beginObject();

    // Color by kind.
    jw.key("fill-extrusion-color");
    jw.beginArray();
    jw.stringValue("match");
    jw.beginArray();
    jw.stringValue("get");
    jw.stringValue("kind");
    jw.endArray();
    jw.stringValue("residential");
    jw.stringValue("#e3d6c9");
    jw.stringValue("commercial");
    jw.stringValue("#e0bcbc");
    jw.stringValue("industrial");
    jw.stringValue("#d4d4d4");
    jw.stringValue("school");
    jw.stringValue("#cfe3f0");
    jw.stringValue("hospital");
    jw.stringValue("#f1cfe0");
    jw.stringValue("police");
    jw.stringValue("#d6dcf6");
    jw.stringValue("fire");
    jw.stringValue("#f2c7a4");
    jw.stringValue("#cccccc");
    jw.endArray();

    // Height/base in meters from feature props.
    jw.key("fill-extrusion-height");
    jw.beginArray();
    jw.stringValue("get");
    jw.stringValue("height_m");
    jw.endArray();

    jw.key("fill-extrusion-base");
    jw.beginArray();
    jw.stringValue("get");
    jw.stringValue("base_m");
    jw.endArray();

    jw.key("fill-extrusion-opacity");
    jw.numberValue(0.85);

    jw.key("fill-extrusion-vertical-gradient");
    jw.boolValue(true);

    jw.endObject();
    jw.endObject();

    // Subtle outline for readability.
    jw.beginObject();
    jw.key("id");
    jw.stringValue("buildings-outline");
    jw.key("type");
    jw.stringValue("line");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "building");
    jw.key("paint");
    jw.beginObject();
    jw.key("line-color");
    jw.stringValue("#9c8f80");
    jw.key("line-width");
    jw.numberValue(0.6);
    jw.key("line-opacity");
    jw.numberValue(0.35);
    jw.endObject();
    jw.endObject();
  }

  if (layers.roadTiles) {
    jw.beginObject();
    jw.key("id");
    jw.stringValue("road-tiles-fill");
    jw.key("type");
    jw.stringValue("fill");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "road_tiles");
    jw.key("paint");
    jw.beginObject();
    jw.key("fill-color");
    jw.stringValue("#efe9e0");
    jw.key("fill-opacity");
    jw.numberValue(0.6);
    jw.endObject();
    jw.endObject();
  }

  if (layers.districts) {
    jw.beginObject();
    jw.key("id");
    jw.stringValue("district-outline");
    jw.key("type");
    jw.stringValue("line");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "district");
    jw.key("paint");
    jw.beginObject();
    jw.key("line-color");
    jw.stringValue("#9c8f80");
    jw.key("line-width");
    jw.numberValue(1.0);
    jw.key("line-opacity");
    jw.numberValue(0.8);
    jw.endObject();
    jw.endObject();
  }

  if (layers.roads) {
    // Road casing (outline)
    jw.beginObject();
    jw.key("id");
    jw.stringValue("roads-casing");
    jw.key("type");
    jw.stringValue("line");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "road");
    jw.key("layout");
    jw.beginObject();
    jw.key("line-join");
    jw.stringValue("round");
    jw.key("line-cap");
    jw.stringValue("round");
    jw.endObject();
    jw.key("paint");
    jw.beginObject();
    jw.key("line-color");
    jw.stringValue("#c2b8aa");
    jw.key("line-opacity");
    jw.numberValue(0.9);
    jw.key("line-width");
    // line-width = interpolate(avgLevel, 1->2.5, 2->3.5, 3->4.5)
    jw.beginArray();
    jw.stringValue("interpolate");
    jw.beginArray();
    jw.stringValue("linear");
    jw.endArray();
    jw.beginArray();
    jw.stringValue("get");
    jw.stringValue("avgLevel");
    jw.endArray();
    jw.numberValue(1.0);
    jw.numberValue(2.5);
    jw.numberValue(2.0);
    jw.numberValue(3.5);
    jw.numberValue(3.0);
    jw.numberValue(4.5);
    jw.endArray();
    jw.endObject();
    jw.endObject();

    // Road fill
    jw.beginObject();
    jw.key("id");
    jw.stringValue("roads");
    jw.key("type");
    jw.stringValue("line");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "road");
    jw.key("layout");
    jw.beginObject();
    jw.key("line-join");
    jw.stringValue("round");
    jw.key("line-cap");
    jw.stringValue("round");
    jw.endObject();
    jw.key("paint");
    jw.beginObject();
    jw.key("line-color");
    jw.stringValue("#ffffff");
    jw.key("line-opacity");
    jw.numberValue(0.95);
    jw.key("line-width");
    jw.beginArray();
    jw.stringValue("interpolate");
    jw.beginArray();
    jw.stringValue("linear");
    jw.endArray();
    jw.beginArray();
    jw.stringValue("get");
    jw.stringValue("avgLevel");
    jw.endArray();
    jw.numberValue(1.0);
    jw.numberValue(1.5);
    jw.numberValue(2.0);
    jw.numberValue(2.2);
    jw.numberValue(3.0);
    jw.numberValue(3.0);
    jw.endArray();
    jw.endObject();
    jw.endObject();
  }

  if (layers.roadNodes) {
    jw.beginObject();
    jw.key("id");
    jw.stringValue("road-nodes");
    jw.key("type");
    jw.stringValue("circle");
    jw.key("source");
    jw.stringValue("world");
    jw.key("filter");
    writeFilterLayerEq("layer", "road_node");
    jw.key("paint");
    jw.beginObject();
    jw.key("circle-color");
    jw.stringValue("#666666");
    jw.key("circle-radius");
    jw.numberValue(2.5);
    jw.key("circle-opacity");
    jw.numberValue(0.85);
    jw.endObject();
    jw.endObject();
  }

  jw.endArray(); // layers
  jw.endObject();

  if (!jw.ok() || !os.good()) {
    if (outErr) *outErr = "Style write failed: " + jw.error();
    return false;
  }

  return true;
}

bool WriteMapboxIndexHtml(const std::string& path, const std::string& title, const std::string& styleRelPath,
                          double minLon, double minLat, double maxLon, double maxLat, double centerLon,
                          double centerLat, std::string* outErr)
{
  // MapLibre CDN links are based on the official MapLibre migration guide.
  // The version can be changed by hand later.
  const std::string cdnCss = "https://unpkg.com/maplibre-gl@5.16.0/dist/maplibre-gl.css";
  const std::string cdnJs = "https://unpkg.com/maplibre-gl@5.16.0/dist/maplibre-gl.js";

  std::ostringstream html;
  html << "<!DOCTYPE html>\n";
  html << "<html>\n";
  html << "<head>\n";
  html << "  <meta charset=\"utf-8\" />\n";
  html << "  <title>" << title << "</title>\n";
  html << "  <meta name=\"viewport\" content=\"initial-scale=1,maximum-scale=1,user-scalable=no\" />\n";
  html << "  <link href=\"" << cdnCss << "\" rel=\"stylesheet\" />\n";
  html << "  <style>\n";
  html << "    body { margin: 0; padding: 0; }\n";
  html << "    html, body, #map { height: 100%; }\n";
  html << "    #map { width: 100%; }\n";
  html << "    .panel { position: absolute; top: 10px; left: 10px; background: rgba(255,255,255,0.92); "
          "padding: 10px 12px; border-radius: 6px; font-family: sans-serif; font-size: 12px; "
          "max-width: 360px; }\n";
  html << "    .panel code { font-size: 11px; }\n";
  html << "  </style>\n";
  html << "</head>\n";
  html << "<body>\n";
  html << "  <div id=\"map\"></div>\n";
  html << "  <div class=\"panel\">\n";
  html << "    <div><b>ProcIsoCity Mapbox bundle</b></div>\n";
  html << "    <div style=\"margin-top:6px;\">If opened via <code>file://</code>, use <code>index_inline.html</code> "
          "or run a local web server (e.g. <code>python3 -m http.server</code>).</div>\n";
  html << "    <div id=\"info\" style=\"margin-top:8px; white-space:pre; font-family:monospace; font-size:12px;\">Hover for feature info.</div>\n";
  html << "    <label style=\"display:block;margin-top:6px;\"><input id=\"toggleBuildings\" type=\"checkbox\" checked> 3D buildings</label>\n";
  html << "  </div>\n";
  html << "  <script src=\"" << cdnJs << "\"></script>\n";
  html << "  <script>\n";
  html << "    const styleUrl = '" << styleRelPath << "';\n";
  html << "    const bounds = [[" << minLon << "," << minLat << "],[" << maxLon << "," << maxLat << "]];\n";
  html << "    const map = new maplibregl.Map({\n";
  html << "      container: 'map',\n";
  html << "      style: styleUrl,\n";
  html << "      center: [" << centerLon << "," << centerLat << "],\n";
  html << "      zoom: 15\n";
  html << "    });\n";
  html << "    map.addControl(new maplibregl.NavigationControl(), 'top-right');\n";
  html << "    map.on('load', () => {\n";
  html << "      map.fitBounds(bounds, { padding: 20, maxZoom: 18 });\n";
  html << "      try {\n";
  html << "        const st = map.getStyle();\n";
  html << "        const has3d = st && st.layers && st.layers.some(l => l.type === 'fill-extrusion');\n";
  html << "        if (has3d) { map.setPitch(60); map.setBearing(-20); }\n";
  html << "      } catch (e) {}\n";
  html << "      const cb = document.getElementById(\'toggleBuildings\');\n";
  html << "      if (cb) {\n";
  html << "        const has = !!map.getLayer(\'buildings\');\n";
  html << "        cb.disabled = !has;\n";
  html << "        cb.checked = has;\n";
  html << "        cb.addEventListener(\'change\', () => {\n";
  html << "          const vis = cb.checked ? \'visible\' : \'none\';\n";
  html << "          if (map.getLayer(\'buildings\')) map.setLayoutProperty(\'buildings\', \'visibility\', vis);\n";
  html << "          if (map.getLayer(\'buildings-outline\')) map.setLayoutProperty(\'buildings-outline\', \'visibility\', vis);\n";
  html << "        });\n";
  html << "      }\n";
  html << "    });\n";
  html << "    const info = document.getElementById('info');\n";
  html << "    function fmtProps(p) {\n";
  html << "      if (!p) return '';\n";
  html << "      const keys = Object.keys(p);\n";
  html << "      keys.sort();\n";
  html << "      const parts = [];\n";
  html << "      for (const k of keys) {\n";
  html << "        if (k === 'layer') continue;\n";
  html << "        parts.push(k + ': ' + p[k]);\n";
  html << "      }\n";
  html << "      return parts.join('\\n');\n";
  html << "    }\n";
  html << "    map.on('mousemove', (e) => {\n";
  html << "      const feats = map.queryRenderedFeatures(e.point);\n";
  html << "      if (!feats || feats.length === 0) {\n";
  html << "        info.textContent = 'Hover for feature info.';\n";
  html << "        return;\n";
  html << "      }\n";
  html << "      const f = feats[0];\n";
  html << "      const layer = f.properties && f.properties.layer ? f.properties.layer : f.layer.id;\n";
  html << "      const extra = (f.properties ? fmtProps(f.properties) : '');\n";
  html << "      info.textContent = extra ? (layer + \'\\n\' + extra) : layer;\n";
  html << "    });\n";
  html << "  </script>\n";
  html << "</body>\n";
  html << "</html>\n";

  return WriteTextFile(path, html.str(), outErr);
}

bool WriteMapboxInlineHtml(const std::string& path, const std::string& title, const ExportLayers& layers,
                           double minLon, double minLat, double maxLon, double maxLat, double centerLon,
                           double centerLat, const std::string& geojsonText, std::string* outErr)
{
  // Inline viewer does not rely on fetching world.geojson, so it works via file://
  const std::string cdnCss = "https://unpkg.com/maplibre-gl@5.16.0/dist/maplibre-gl.css";
  const std::string cdnJs = "https://unpkg.com/maplibre-gl@5.16.0/dist/maplibre-gl.js";

  // Inline viewer draws layers directly in JS, so it can work without fetch() via file://

  std::ostringstream html;
  html << "<!DOCTYPE html>\n";
  html << "<html>\n";
  html << "<head>\n";
  html << "  <meta charset=\"utf-8\" />\n";
  html << "  <title>" << title << "</title>\n";
  html << "  <meta name=\"viewport\" content=\"initial-scale=1,maximum-scale=1,user-scalable=no\" />\n";
  html << "  <link href=\"" << cdnCss << "\" rel=\"stylesheet\" />\n";
  html << "  <style>body{margin:0;padding:0;} html,body,#map{height:100%;} .panel{position:absolute;top:10px;left:10px;"
          "background:rgba(255,255,255,0.92);padding:10px 12px;border-radius:6px;font-family:sans-serif;font-size:12px;max-width:360px;}</style>\n";
  html << "</head>\n";
  html << "<body>\n";
  html << "  <div id=\"map\"></div>\n";
  html << "  <div class=\"panel\">\n";
  html << "    <div><b>ProcIsoCity Mapbox bundle</b> (inline GeoJSON)</div>\n";
  html << "    <div id=\"info\" style=\"margin-top:8px; white-space:pre; font-family:monospace; font-size:12px;\">Hover for feature info.</div>\n";
  html << "    <label style=\"display:block;margin-top:6px;\"><input id=\"toggleBuildings\" type=\"checkbox\" checked> 3D buildings</label>\n";
  html << "  </div>\n";
  html << "  <script id=\"world-geojson\" type=\"application/json\">\n";
  html << geojsonText;
  html << "\n  </script>\n";
  html << "  <script src=\"" << cdnJs << "\"></script>\n";
  html << "  <script>\n";
  html << "    const world = JSON.parse(document.getElementById('world-geojson').textContent);\n";
  html << "    const bounds = [[" << minLon << "," << minLat << "],[" << maxLon << "," << maxLat << "]];\n";
  html << "    const map = new maplibregl.Map({\n";
  html << "      container: 'map',\n";
  html << "      style: { version: 8, sources: {}, layers: [{ id: 'background', type: 'background', paint: { 'background-color': '#f4f1ec' } }] },\n";
  html << "      center: [" << centerLon << "," << centerLat << "],\n";
  html << "      zoom: 15\n";
  html << "    });\n";
  html << "    map.addControl(new maplibregl.NavigationControl(), 'top-right');\n";
  html << "    map.on('load', () => {\n";
  html << "      map.addSource('world', { type: 'geojson', data: world });\n";

  // Add layers roughly matching style.json (kept intentionally minimal).
  html << "      map.addLayer({ id: 'water-fill', type: 'fill', source: 'world', filter: ['==',['get','layer'],'water'], paint: { 'fill-color':'#a0c8f0','fill-opacity':0.95 } });\n";
  html << "      map.addLayer({ id: 'parks-fill', type: 'fill', source: 'world', filter: ['all',['==',['get','layer'],'landuse'],['==',['get','kind'],'park']], paint: { 'fill-color':'#cfe8c7','fill-opacity':0.9 } });\n";
  html << "      map.addLayer({ id: 'res-fill', type: 'fill', source: 'world', filter: ['all',['==',['get','layer'],'landuse'],['==',['get','kind'],'residential']], paint: { 'fill-color':'#f2e5d5','fill-opacity':0.9 } });\n";
  html << "      map.addLayer({ id: 'com-fill', type: 'fill', source: 'world', filter: ['all',['==',['get','layer'],'landuse'],['==',['get','kind'],'commercial']], paint: { 'fill-color':'#f0d4d4','fill-opacity':0.85 } });\n";
  html << "      map.addLayer({ id: 'ind-fill', type: 'fill', source: 'world', filter: ['all',['==',['get','layer'],'landuse'],['==',['get','kind'],'industrial']], paint: { 'fill-color':'#e5e5e5','fill-opacity':0.85 } });\n";
  if (layers.buildings) {
    html << "      map.addLayer({ id: 'buildings', type: 'fill-extrusion', source: 'world', filter: ['==',['get','layer'],'building'], paint: { \"fill-extrusion-color\": ['match',['get','kind'],'residential','#e3d6c9','commercial','#e0bcbc','industrial','#d4d4d4','school','#cfe3f0','hospital','#f1cfe0','police','#d6dcf6','fire','#f2c7a4','#cccccc'], \"fill-extrusion-height\": ['get','height_m'], \"fill-extrusion-base\": ['get','base_m'], \"fill-extrusion-opacity\": 0.85, \"fill-extrusion-vertical-gradient\": true } });\n";
    html << "      map.addLayer({ id: 'buildings-outline', type: 'line', source: 'world', filter: ['==',['get','layer'],'building'], paint: { 'line-color':'#9c8f80','line-width':0.6,'line-opacity':0.35 } });\n";
  }
  html << "      map.addLayer({ id: 'roads-casing', type: 'line', source: 'world', filter: ['==',['get','layer'],'road'], layout: { 'line-join':'round','line-cap':'round' }, paint: { 'line-color':'#c2b8aa','line-opacity':0.9,'line-width':['interpolate',['linear'],['get','avgLevel'],1,2.5,2,3.5,3,4.5] } });\n";
  html << "      map.addLayer({ id: 'roads', type: 'line', source: 'world', filter: ['==',['get','layer'],'road'], layout: { 'line-join':'round','line-cap':'round' }, paint: { 'line-color':'#ffffff','line-opacity':0.95,'line-width':['interpolate',['linear'],['get','avgLevel'],1,1.5,2,2.2,3,3.0] } });\n";
  html << "      map.fitBounds(bounds, { padding: 20, maxZoom: 18 });\n";
  if (layers.buildings) {
    html << "      map.setPitch(60);\n";
    html << "      map.setBearing(-20);\n";
    html << "      const cb = document.getElementById('toggleBuildings');\n";
    html << "      if (cb) {\n";
    html << "        cb.addEventListener('change', () => {\n";
    html << "          const vis = cb.checked ? 'visible' : 'none';\n";
    html << "          if (map.getLayer('buildings')) map.setLayoutProperty('buildings', 'visibility', vis);\n";
    html << "          if (map.getLayer('buildings-outline')) map.setLayoutProperty('buildings-outline', 'visibility', vis);\n";
    html << "        });\n";
    html << "      }\n";
  }
  html << "    });\n";
  html << "    const info = document.getElementById('info');\n";
  html << "    map.on('mousemove', (e) => {\n";
  html << "      const feats = map.queryRenderedFeatures(e.point);\n";
  html << "      if (!feats || feats.length === 0) { info.textContent = 'Hover for feature info.'; return; }\n";
  html << "      const f = feats[0];\n";
  html << "      const layer = f.properties && f.properties.layer ? f.properties.layer : f.layer.id;\n";
  html << "      const extra = (f.properties ? fmtProps(f.properties) : '');\n";
  html << "      info.textContent = extra ? (layer + \'\\n\' + extra) : layer;\n";
  html << "    });\n";
  html << "  </script>\n";
  html << "</body>\n";
  html << "</html>\n";

  return WriteTextFile(path, html.str(), outErr);
}

int WriteMapboxBundle(const std::string& outDir, const World& world, const SimConfig& simCfg, const ExportLayers& layers,
                      const GeoRef& georef, std::string* outErr)
{
  try {
    std::filesystem::create_directories(outDir);
  } catch (...) {
    if (outErr) *outErr = "Failed to create output directory: " + outDir;
    return 4;
  }

  const std::string geoPath = (std::filesystem::path(outDir) / "world.geojson").string();
  const std::string stylePath = (std::filesystem::path(outDir) / "style.json").string();
  const std::string indexPath = (std::filesystem::path(outDir) / "index.html").string();
  const std::string inlinePath = (std::filesystem::path(outDir) / "index_inline.html").string();

  // 1) GeoJSON in lon/lat.
  {
    std::ofstream os(geoPath, std::ios::binary);
    if (!os) {
      if (outErr) *outErr = "Failed to open output file: " + geoPath;
      return 4;
    }
    const int rc = ExportWorldGeoJson(os, world, simCfg, layers, CoordSpace::Wgs84LonLat, georef, outErr);
    if (rc != 0) return rc;
  }

  // Bounds/center for viewers.
  double minLon = 0.0, minLat = 0.0, maxLon = 0.0, maxLat = 0.0;
  ComputeLonLatBbox(world.width(), world.height(), georef, &minLon, &minLat, &maxLon, &maxLat);
  const double centerLon = 0.5 * (minLon + maxLon);
  const double centerLat = 0.5 * (minLat + maxLat);

  // 2) Mapbox style.json (v8).
  if (!WriteMapboxStyleJson(stylePath, layers, "world.geojson", centerLon, centerLat, outErr)) return 4;

  // 3) index.html viewer (requires local web server for relative fetches in many browsers).
  if (!WriteMapboxIndexHtml(indexPath, "ProcIsoCity Map", "style.json", minLon, minLat, maxLon, maxLat, centerLon,
                            centerLat, outErr)) {
    return 4;
  }

  // 4) index_inline.html viewer (embeds the GeoJSON so it works via file://).
  {
    std::string geoText;
    if (!ReadTextFile(geoPath, &geoText, outErr)) return 4;
    if (!WriteMapboxInlineHtml(inlinePath, "ProcIsoCity Map (inline)", layers, minLon, minLat, maxLon, maxLat, centerLon,
                               centerLat, geoText, outErr)) {
      return 4;
    }
  }

  return 0;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string geojsonPath;
  std::string mapboxDir;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  ExportLayers layers{};
  GeoRef mapboxGeo{};
  bool buildingsSet = false;

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
    } else if (arg == "--mapbox") {
      if (!requireValue(i, val)) {
        std::cerr << "--mapbox requires a directory\n";
        return 2;
      }
      mapboxDir = val;
    } else if (arg == "--mapbox-origin") {
      if (!requireValue(i, val) || !ParseLonLat(val, &mapboxGeo.originLon, &mapboxGeo.originLat)) {
        std::cerr << "--mapbox-origin requires format lon,lat (e.g. -122.42,37.77)\n";
        return 2;
      }
    } else if (arg == "--mapbox-meters-per-tile") {
      if (!requireValue(i, val) || !ParseDouble(val, &mapboxGeo.metersPerTile) || mapboxGeo.metersPerTile <= 0.0) {
        std::cerr << "--mapbox-meters-per-tile requires a positive number\n";
        return 2;
      }
    } else if (arg == "--mapbox-flip-y") {
      if (!requireValue(i, val) || !ParseBool01(val, &mapboxGeo.flipY)) {
        std::cerr << "--mapbox-flip-y requires 0 or 1\n";
        return 2;
      }
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
      if (!requireValue(i, val) || !ParseBool01(val, &layers.roads)) {
        std::cerr << "--roads requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--road-nodes") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.roadNodes)) {
        std::cerr << "--road-nodes requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--road-tiles") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.roadTiles)) {
        std::cerr << "--road-tiles requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--zones") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.zones)) {
        std::cerr << "--zones requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--buildings") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.buildings)) {
        std::cerr << "--buildings requires 0 or 1\n";
        return 2;
      }
      buildingsSet = true;
    } else if (arg == "--parks") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.parks)) {
        std::cerr << "--parks requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--water") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.water)) {
        std::cerr << "--water requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--districts") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.districts)) {
        std::cerr << "--districts requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--district-water") {
      if (!requireValue(i, val) || !ParseBool01(val, &layers.districtIncludeWater)) {
        std::cerr << "--district-water requires 0 or 1\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (geojsonPath.empty() && mapboxDir.empty()) {
    std::cerr << "At least one output is required: --geojson and/or --mapbox\n";
    PrintHelp();
    return 2;
  }

  // For the Mapbox/MapLibre bundle, enable 3D building footprints by default unless the user overrides.
  if (!mapboxDir.empty() && !buildingsSet) {
    layers.buildings = true;
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

  // Tile-space GeoJSON export (original behavior)
  if (!geojsonPath.empty()) {
    if (!EnsureParentDir(geojsonPath)) {
      std::cerr << "Failed to create output directory\n";
      return 4;
    }
    std::ofstream os(geojsonPath, std::ios::binary);
    if (!os) {
      std::cerr << "Failed to open output file\n";
      return 4;
    }

    std::string e;
    const int rc = ExportWorldGeoJson(os, world, simCfg, layers, CoordSpace::TileGrid, GeoRef{}, &e);
    if (rc != 0) {
      std::cerr << e << "\n";
      return rc;
    }
  }

  // Mapbox/MapLibre bundle export (lon/lat GeoJSON + style + HTML viewer)
  if (!mapboxDir.empty()) {
    std::string e;
    const int rc = WriteMapboxBundle(mapboxDir, world, simCfg, layers, mapboxGeo, &e);
    if (rc != 0) {
      std::cerr << e << "\n";
      return rc;
    }
    std::cout << "Wrote Mapbox bundle to: " << mapboxDir << "\n";
  }

  return 0;
}
