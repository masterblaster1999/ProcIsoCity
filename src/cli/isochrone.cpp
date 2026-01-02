#include "isocity/Export.hpp"
#include "isocity/Isochrone.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Vectorize.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstddef>
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

bool ParseF64(const std::string& s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
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

bool ParsePointCsv(const std::string& s, Point& out)
{
  const std::size_t comma = s.find(',');
  if (comma == std::string::npos) return false;
  int x = 0;
  int y = 0;
  if (!ParseI32(s.substr(0, comma), &x)) return false;
  if (!ParseI32(s.substr(comma + 1), &y)) return false;
  out = Point{x, y};
  return true;
}

std::vector<std::string> SplitCsv(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

bool ParseThresholdsCsv(const std::string& s, std::vector<double>& out)
{
  out.clear();
  for (const std::string& tok : SplitCsv(s)) {
    double v = 0.0;
    if (!ParseF64(tok, &v)) return false;
    if (v < 0.0) return false;
    out.push_back(v);
  }
  // Ensure strictly increasing thresholds (common isochrone assumption).
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return !out.empty();
}

bool EnsureParentDir(const std::string& path)
{
  if (path.empty()) return true;
  std::error_code ec;
  const std::filesystem::path p(path);
  const std::filesystem::path dir = p.parent_path();
  if (dir.empty()) return true;
  std::filesystem::create_directories(dir, ec);
  return !ec;
}

const char* WeightModeName(IsochroneWeightMode m)
{
  switch (m) {
  case IsochroneWeightMode::Steps: return "steps";
  case IsochroneWeightMode::TravelTime: return "time";
  }
  return "time";
}

bool ParseWeightMode(const std::string& s, IsochroneWeightMode& out)
{
  if (s == "steps" || s == "len" || s == "length") {
    out = IsochroneWeightMode::Steps;
    return true;
  }
  if (s == "time" || s == "travel" || s == "travel_time" || s == "traveltime") {
    out = IsochroneWeightMode::TravelTime;
    return true;
  }
  return false;
}

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline void HeatRampRedYellowGreen(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  auto clamp01 = [](float v) -> float { return (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); };
  const float t = clamp01(v01);
  // 0 -> red, 0.5 -> yellow, 1 -> green
  if (t <= 0.5f) {
    r = 255;
    g = static_cast<std::uint8_t>(255.0f * (t * 2.0f));
    b = 0;
  } else {
    r = static_cast<std::uint8_t>(255.0f * (1.0f - (t - 0.5f) * 2.0f));
    g = 255;
    b = 0;
  }
}

void BlendPixel(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, std::uint8_t hr, std::uint8_t hg, std::uint8_t hb)
{
  // Blend 2/3 heatmap, 1/3 base.
  r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
  g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
  b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
}

bool FindNearestRoad(const World& world, Point start, const std::vector<std::uint8_t>* roadToEdgeMask, Point& outRoad)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  auto roadOk = [&](int x, int y) -> bool {
    if (!world.inBounds(x, y)) return false;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) return false;
    if (roadToEdgeMask && roadToEdgeMask->size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
      const std::size_t idx = FlatIdx(x, y, w);
      if ((*roadToEdgeMask)[idx] == 0) return false;
    }
    return true;
  };

  // Deterministic expanding diamond search.
  const int maxR = std::max(w, h) + 2;
  for (int r = 0; r <= maxR; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      const int dy = r - std::abs(dx);
      const int x0 = start.x + dx;
      const int y0 = start.y + dy;
      if (roadOk(x0, y0)) {
        outRoad = Point{x0, y0};
        return true;
      }
      if (dy != 0) {
        const int y1 = start.y - dy;
        if (roadOk(x0, y1)) {
          outRoad = Point{x0, y1};
          return true;
        }
      }
    }
  }
  return false;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_isochrone (headless accessibility + isochrone export)\n\n"
      << "Computes travel-time accessibility from one or more source points using deterministic\n"
      << "multi-source road routing. Optionally exports isochrone polygons (tile-grid) as GeoJSON,\n"
      << "and a raster heatmap overlay for quick inspection.\n\n"
      << "Usage:\n"
      << "  proc_isocity_isochrone [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                      [--days <N>] [--require-outside <0|1>]\n"
      << "                      [--source <x,y>]... [--snap-to-road <0|1>]\n"
      << "                      [--weight-mode <steps|time>]\n"
      << "                      [--thresholds <csv>]\n"
      << "                      [--walk-cost <milli>]\n"
      << "                      [--geojson <out.geojson>] [--owner-geojson <out.geojson>]\n"
      << "                      [--json <out.json>]\n"
      << "                      [--ppm <out.png|out.ppm>] [--scale <N>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for ProcGen (default: 1).\n"
      << "  --size <WxH>                World size for generation (default: 96x96).\n"
      << "  --days <N>                  Advance the simulation N days before analysis (default: 0).\n"
      << "  --require-outside <0|1>     Restrict to road network connected to the map edge (default: 1).\n\n"
      << "Sources:\n"
      << "  --source <x,y>              Add a source point (repeatable). Default: center of map.\n"
      << "  --snap-to-road <0|1>        If 1, non-road sources are snapped to nearest road (default: 1).\n\n"
      << "Routing:\n"
      << "  --weight-mode <steps|time>  steps = shortest path by edge count; time = weighted by road class (default: time).\n"
      << "  --walk-cost <milli>         Cost added when mapping a non-road tile to a road tile (default: 0).\n\n"
      << "Isochrone rings:\n"
      << "  --thresholds <csv>          Comma-separated thresholds in street-step units (default: 5,10,20,40).\n"
      << "                              (Internally: milli-steps; street edge ~= 1000).\n\n"
      << "Outputs:\n"
      << "  --geojson <out.geojson>     GeoJSON FeatureCollection of MultiPolygon isochrone rings.\n"
      << "  --owner-geojson <out.geojson>  If multiple sources, GeoJSON polygons partitioned by nearest source.\n"
      << "  --json <out.json>           JSON summary (sources, thresholds, zone coverage metrics).\n"
      << "  --ppm <out.png|out.ppm>     Raster overlay heatmap (one pixel per tile; optional upscale).\n"
      << "  --scale <N>                 Upscale factor for --ppm (default: 4).\n";
}

struct ZoneAgg {
  std::uint64_t tiles = 0;
  std::uint64_t occupants = 0;
  std::uint64_t reachableTiles = 0;
  std::uint64_t reachableOccupants = 0;

  double sumCostStreetSteps = 0.0;
  double sumCostStreetStepsOcc = 0.0;
};

inline void AccumZone(const Tile& t, int costMilli, ZoneAgg& agg)
{
  agg.tiles += 1;
  agg.occupants += t.occupants;
  if (costMilli >= 0) {
    agg.reachableTiles += 1;
    agg.reachableOccupants += t.occupants;
    const double c = static_cast<double>(costMilli) / 1000.0;
    agg.sumCostStreetSteps += c;
    agg.sumCostStreetStepsOcc += c * static_cast<double>(t.occupants);
  }
}

void WriteGeoJsonHeader(std::ostream& os)
{
  os << "{\n  \"type\": \"FeatureCollection\",\n  \"features\": [\n";
}

void WriteGeoJsonFooter(std::ostream& os)
{
  os << "\n  ]\n}\n";
}

void WriteGeoJsonMultiPolygon(std::ostream& os, const VectorMultiPolygon& mp)
{
  // Coordinates are tile-corner integer points.
  os << "{\"type\":\"MultiPolygon\",\"coordinates\":[";
  for (std::size_t pi = 0; pi < mp.polygons.size(); ++pi) {
    if (pi) os << ',';
    const VectorPolygon& poly = mp.polygons[pi];
    os << '[';
    // Outer ring then holes.
    auto writeRing = [&](const std::vector<IPoint>& ring) {
      os << '[';
      for (std::size_t i = 0; i < ring.size(); ++i) {
        if (i) os << ',';
        os << '[' << ring[i].x << ',' << ring[i].y << ']';
      }
      os << ']';
    };
    writeRing(poly.outer);
    for (const auto& hole : poly.holes) {
      os << ',';
      writeRing(hole);
    }
    os << ']';
  }
  os << "]}";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;
  int days = 0;

  bool requireOutside = true;
  bool snapToRoad = true;
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;
  int walkCostMilli = 0;
  int ppmScale = 4;

  std::vector<Point> sourcePoints;
  std::vector<double> thresholds = {5.0, 10.0, 20.0, 40.0};

  std::string outGeoJson;
  std::string outOwnerGeoJson;
  std::string outJson;
  std::string outPpm;

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
    } else if (arg == "--snap-to-road") {
      if (!requireValue(i, val) || !ParseBool01(val, &snapToRoad)) {
        std::cerr << "--snap-to-road requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--source") {
      if (!requireValue(i, val)) {
        std::cerr << "--source requires x,y\n";
        return 2;
      }
      Point p{0, 0};
      if (!ParsePointCsv(val, p)) {
        std::cerr << "--source expects x,y\n";
        return 2;
      }
      sourcePoints.push_back(p);
    } else if (arg == "--weight-mode") {
      if (!requireValue(i, val) || !ParseWeightMode(val, weightMode)) {
        std::cerr << "--weight-mode requires steps|time\n";
        return 2;
      }
    } else if (arg == "--thresholds") {
      if (!requireValue(i, val) || !ParseThresholdsCsv(val, thresholds)) {
        std::cerr << "--thresholds requires a comma-separated float list (e.g. 5,10,20)\n";
        return 2;
      }
    } else if (arg == "--walk-cost") {
      if (!requireValue(i, val) || !ParseI32(val, &walkCostMilli) || walkCostMilli < 0) {
        std::cerr << "--walk-cost requires a non-negative integer (milli-steps)\n";
        return 2;
      }
    } else if (arg == "--geojson") {
      if (!requireValue(i, val)) {
        std::cerr << "--geojson requires a path\n";
        return 2;
      }
      outGeoJson = val;
    } else if (arg == "--owner-geojson") {
      if (!requireValue(i, val)) {
        std::cerr << "--owner-geojson requires a path\n";
        return 2;
      }
      outOwnerGeoJson = val;
    } else if (arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--ppm") {
      if (!requireValue(i, val)) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
      outPpm = val;
    } else if (arg == "--scale") {
      if (!requireValue(i, val) || !ParseI32(val, &ppmScale) || ppmScale <= 0) {
        std::cerr << "--scale requires a positive integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << " (use --help)\n";
      return 2;
    }
  }

  // Load or generate world.
  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Load failed: " << err << "\n";
      return 2;
    }
    seed = world.seed();
    w = world.width();
    h = world.height();
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  // Allow CLI override.
  simCfg.requireOutsideConnection = requireOutside;

  if (days > 0) {
    Simulator sim(simCfg);
    for (int d = 0; d < days; ++d) sim.stepOnce(world);
    sim.refreshDerivedStats(world);
  }

  const int ww = world.width();
  const int hh = world.height();
  if (ww <= 0 || hh <= 0) {
    std::cerr << "World has invalid size\n";
    return 2;
  }

  // Compute road-to-edge mask when needed.
  std::vector<std::uint8_t> roadToEdge;
  if (requireOutside) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
  }

  if (sourcePoints.empty()) {
    sourcePoints.push_back(Point{ww / 2, hh / 2});
  }

  // Resolve sources to road tiles.
  std::vector<Point> snappedSources;
  std::vector<int> sourceRoadIdx;
  snappedSources.reserve(sourcePoints.size());
  sourceRoadIdx.reserve(sourcePoints.size());

  for (const Point& p : sourcePoints) {
    Point s = p;
    if (!world.inBounds(s.x, s.y)) {
      std::cerr << "source out of bounds: " << s.x << "," << s.y << "\n";
      return 2;
    }
    const Tile& t = world.at(s.x, s.y);
    if (t.overlay != Overlay::Road) {
      if (!snapToRoad) {
        std::cerr << "source is not on a road (use --snap-to-road 1): " << s.x << "," << s.y << "\n";
        return 2;
      }
      Point nr{0, 0};
      if (!FindNearestRoad(world, s, requireOutside ? &roadToEdge : nullptr, nr)) {
        std::cerr << "failed to find a nearby road for source: " << s.x << "," << s.y << "\n";
        return 2;
      }
      s = nr;
    }

    const int idx = s.y * ww + s.x;
    snappedSources.push_back(s);
    sourceRoadIdx.push_back(idx);
  }

  // Build road isochrone.
  RoadIsochroneConfig icfg;
  icfg.requireOutsideConnection = requireOutside;
  icfg.weightMode = weightMode;
  icfg.computeOwner = (!outOwnerGeoJson.empty() && snappedSources.size() >= 2);

  const RoadIsochroneField roadField = BuildRoadIsochroneField(world, sourceRoadIdx, icfg, requireOutside ? &roadToEdge : nullptr);

  if (roadField.empty() || roadField.costMilli.empty()) {
    std::cerr << "isochrone build failed (empty road field)\n";
    return 2;
  }

  TileAccessCostConfig tcfg;
  tcfg.includeRoadTiles = true;
  tcfg.includeZones = true;
  tcfg.includeNonZonesAdjacentToRoad = true;
  tcfg.includeWater = false;
  tcfg.accessStepCostMilli = walkCostMilli;
  tcfg.useZoneAccessMap = true;

  const std::vector<int> tileCost = BuildTileAccessCostField(world, roadField, tcfg, requireOutside ? &roadToEdge : nullptr);

  // Compute max for heatmap scaling.
  int maxCost = 0;
  int minCost = std::numeric_limits<int>::max();
  for (int c : tileCost) {
    if (c < 0) continue;
    maxCost = std::max(maxCost, c);
    minCost = std::min(minCost, c);
  }
  if (minCost == std::numeric_limits<int>::max()) minCost = 0;

  // Raster output.
  if (!outPpm.empty()) {
    if (!EnsureParentDir(outPpm)) {
      std::cerr << "failed to create parent directory for: " << outPpm << "\n";
      return 2;
    }

    PpmImage img = RenderPpmLayer(world, ExportLayer::Overlay);

    const float denom = (maxCost > minCost) ? static_cast<float>(maxCost - minCost) : 1.0f;
    for (int y = 0; y < hh; ++y) {
      for (int x = 0; x < ww; ++x) {
        const std::size_t idx = FlatIdx(x, y, ww);
        const int c = tileCost[idx];
        if (c < 0) continue;

        const float t01 = static_cast<float>(c - minCost) / denom;
        std::uint8_t hr, hg, hb;
        // Low cost = green, high cost = red.
        HeatRampRedYellowGreen(1.0f - t01, hr, hg, hb);

        const std::size_t p = idx * 3u;
        if (p + 2 < img.rgb.size()) {
          BlendPixel(img.rgb[p + 0], img.rgb[p + 1], img.rgb[p + 2], hr, hg, hb);
        }
      }
    }

    // Mark snapped sources.
    for (const Point& s : snappedSources) {
      const std::size_t idx = FlatIdx(s.x, s.y, ww);
      const std::size_t p = idx * 3u;
      if (p + 2 < img.rgb.size()) {
        img.rgb[p + 0] = 20;
        img.rgb[p + 1] = 40;
        img.rgb[p + 2] = 255;
      }
    }

    if (ppmScale > 1) img = ScaleNearest(img, ppmScale);
    std::string we;
    if (!WriteImageAuto(outPpm, img, we)) {
      std::cerr << "image export failed: " << we << "\n";
      return 2;
    }
    std::cout << "wrote heatmap -> " << outPpm << "\n";
  }

  // JSON summary.
  if (!outJson.empty()) {
    if (!EnsureParentDir(outJson)) {
      std::cerr << "failed to create parent directory for: " << outJson << "\n";
      return 2;
    }

    ZoneAgg res{}, com{}, ind{};
    std::uint64_t reachableTiles = 0;
    std::uint64_t reachableNonWater = 0;

    for (int y = 0; y < hh; ++y) {
      for (int x = 0; x < ww; ++x) {
        const Tile& t = world.at(x, y);
        const std::size_t idx = FlatIdx(x, y, ww);
        const int c = tileCost[idx];
        if (t.terrain != Terrain::Water) {
          reachableNonWater += 1;
          if (c >= 0) reachableTiles += 1;
        }

        if (t.overlay == Overlay::Residential) AccumZone(t, c, res);
        else if (t.overlay == Overlay::Commercial) AccumZone(t, c, com);
        else if (t.overlay == Overlay::Industrial) AccumZone(t, c, ind);
      }
    }

    auto writeZoneObj = [&](std::ostream& os, const char* name, const ZoneAgg& a) {
      const double avg = (a.reachableTiles > 0) ? (a.sumCostStreetSteps / static_cast<double>(a.reachableTiles)) : 0.0;
      const double avgOcc = (a.reachableOccupants > 0) ? (a.sumCostStreetStepsOcc / static_cast<double>(a.reachableOccupants)) : 0.0;
      os << "    \"" << name << "\": {\n"
         << "      \"tiles\": " << a.tiles << ",\n"
         << "      \"occupants\": " << a.occupants << ",\n"
         << "      \"reachable_tiles\": " << a.reachableTiles << ",\n"
         << "      \"reachable_occupants\": " << a.reachableOccupants << ",\n"
         << "      \"avg_cost_street_steps\": " << std::fixed << std::setprecision(3) << avg << ",\n"
         << "      \"avg_cost_street_steps_occupant_weighted\": " << std::fixed << std::setprecision(3) << avgOcc << "\n"
         << "    }";
    };

    std::ofstream os(outJson);
    if (!os) {
      std::cerr << "failed to open: " << outJson << "\n";
      return 2;
    }

    os << "{\n";
    os << "  \"width\": " << ww << ",\n";
    os << "  \"height\": " << hh << ",\n";
    os << "  \"seed\": " << seed << ",\n";
    os << "  \"require_outside_connection\": " << (requireOutside ? "true" : "false") << ",\n";
    os << "  \"weight_mode\": \"" << WeightModeName(weightMode) << "\",\n";
    os << "  \"walk_cost_milli\": " << walkCostMilli << ",\n";
    os << "  \"sources\": [\n";
    for (std::size_t i = 0; i < snappedSources.size(); ++i) {
      if (i) os << ",\n";
      os << "    {\"x\": " << snappedSources[i].x << ", \"y\": " << snappedSources[i].y << "}";
    }
    os << "\n  ],\n";

    os << "  \"reachable_tiles_non_water\": " << reachableTiles << ",\n";
    os << "  \"total_tiles_non_water\": " << reachableNonWater << ",\n";
    os << "  \"zones\": {\n";
    writeZoneObj(os, "residential", res);
    os << ",\n";
    writeZoneObj(os, "commercial", com);
    os << ",\n";
    writeZoneObj(os, "industrial", ind);
    os << "\n  },\n";

    // Threshold stats.
    os << "  \"thresholds\": [\n";
    for (std::size_t ti = 0; ti < thresholds.size(); ++ti) {
      const double th = thresholds[ti];
      const int thMilli = static_cast<int>(std::llround(th * 1000.0));

      std::uint64_t tilesWithin = 0;
      std::uint64_t resTilesWithin = 0, comTilesWithin = 0, indTilesWithin = 0;
      std::uint64_t resOccWithin = 0, comOccWithin = 0, indOccWithin = 0;

      for (int y = 0; y < hh; ++y) {
        for (int x = 0; x < ww; ++x) {
          const std::size_t idx = FlatIdx(x, y, ww);
          const int c = tileCost[idx];
          if (c < 0 || c > thMilli) continue;
          const Tile& t = world.at(x, y);
          if (t.terrain != Terrain::Water) tilesWithin += 1;
          if (t.overlay == Overlay::Residential) {
            resTilesWithin += 1;
            resOccWithin += t.occupants;
          } else if (t.overlay == Overlay::Commercial) {
            comTilesWithin += 1;
            comOccWithin += t.occupants;
          } else if (t.overlay == Overlay::Industrial) {
            indTilesWithin += 1;
            indOccWithin += t.occupants;
          }
        }
      }

      if (ti) os << ",\n";
      os << "    {\n";
      os << "      \"threshold_street_steps\": " << std::fixed << std::setprecision(3) << th << ",\n";
      os << "      \"threshold_milli\": " << thMilli << ",\n";
      os << "      \"tiles_within\": " << tilesWithin << ",\n";
      os << "      \"residential_tiles_within\": " << resTilesWithin << ",\n";
      os << "      \"commercial_tiles_within\": " << comTilesWithin << ",\n";
      os << "      \"industrial_tiles_within\": " << indTilesWithin << ",\n";
      os << "      \"residential_occupants_within\": " << resOccWithin << ",\n";
      os << "      \"commercial_occupants_within\": " << comOccWithin << ",\n";
      os << "      \"industrial_occupants_within\": " << indOccWithin << "\n";
      os << "    }";
    }
    os << "\n  ]\n";
    os << "}\n";

    std::cout << "wrote json -> " << outJson << "\n";
  }

  // Isochrone GeoJSON (polygons).
  if (!outGeoJson.empty()) {
    if (!EnsureParentDir(outGeoJson)) {
      std::cerr << "failed to create parent directory for: " << outGeoJson << "\n";
      return 2;
    }

    std::ofstream os(outGeoJson);
    if (!os) {
      std::cerr << "failed to open: " << outGeoJson << "\n";
      return 2;
    }

    WriteGeoJsonHeader(os);

    bool first = true;
    for (double th : thresholds) {
      const int thMilli = static_cast<int>(std::llround(th * 1000.0));
      const std::size_t n = static_cast<std::size_t>(ww) * static_cast<std::size_t>(hh);
      std::vector<int> labels(n, 0);

      for (int y = 0; y < hh; ++y) {
        for (int x = 0; x < ww; ++x) {
          const std::size_t idx = FlatIdx(x, y, ww);
          const int c = tileCost[idx];
          if (c < 0 || c > thMilli) continue;
          if (world.at(x, y).terrain == Terrain::Water) continue;
          labels[idx] = 1;
        }
      }

      std::vector<LabeledGeometry> geoms;
      VectorizeStats vst;
      std::string verr;
      if (!VectorizeLabelGridToPolygons(labels, ww, hh, 0, geoms, &vst, &verr)) {
        std::cerr << "vectorize failed for threshold " << th << ": " << verr << "\n";
        return 2;
      }

      const VectorMultiPolygon* mp = nullptr;
      for (const auto& g : geoms) {
        if (g.label == 1) {
          mp = &g.geom;
          break;
        }
      }

      if (!mp || mp->polygons.empty()) continue;

      if (!first) os << ",\n";
      first = false;

      os << "    {\"type\":\"Feature\",\"properties\":{";
      os << "\"threshold_street_steps\":" << std::fixed << std::setprecision(3) << th << ',';
      os << "\"threshold_milli\":" << thMilli << ',';
      os << "\"weight_mode\":\"" << WeightModeName(weightMode) << "\",";
      os << "\"require_outside_connection\":" << (requireOutside ? "true" : "false") << ',';
      os << "\"source_count\":" << snappedSources.size();
      os << "},\"geometry\":";
      WriteGeoJsonMultiPolygon(os, *mp);
      os << "}";
    }

    WriteGeoJsonFooter(os);
    std::cout << "wrote geojson -> " << outGeoJson << "\n";
  }

  // Owner partition GeoJSON (Voronoi-like by nearest source on the road network).
  if (!outOwnerGeoJson.empty()) {
    if (snappedSources.size() < 2) {
      std::cerr << "--owner-geojson requires at least 2 sources\n";
      return 2;
    }
    if (roadField.owner.empty()) {
      std::cerr << "owner field not computed (internal error)\n";
      return 2;
    }
    if (!EnsureParentDir(outOwnerGeoJson)) {
      std::cerr << "failed to create parent directory for: " << outOwnerGeoJson << "\n";
      return 2;
    }

    const std::size_t n = static_cast<std::size_t>(ww) * static_cast<std::size_t>(hh);
    std::vector<int> labels(n, 0);

    // Assign zones to their access-road owner so partitions are meaningful off-road.
    const ZoneAccessMap zam = BuildZoneAccessMap(world, requireOutside ? &roadToEdge : nullptr);
    const bool haveZam = (zam.w == ww && zam.h == hh && zam.roadIdx.size() == n);

    for (int y = 0; y < hh; ++y) {
      for (int x = 0; x < ww; ++x) {
        const std::size_t idx = FlatIdx(x, y, ww);
        const Tile& t = world.at(x, y);
        if (t.terrain == Terrain::Water) continue;

        int owner = -1;
        if (t.overlay == Overlay::Road) {
          owner = roadField.owner[idx];
        } else if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
          if (haveZam) {
            const int ridx = zam.roadIdx[idx];
            if (ridx >= 0 && static_cast<std::size_t>(ridx) < n) {
              owner = roadField.owner[static_cast<std::size_t>(ridx)];
            }
          }
        }

        if (owner >= 0) labels[idx] = owner + 1;
      }
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats vst;
    std::string verr;
    if (!VectorizeLabelGridToPolygons(labels, ww, hh, 0, geoms, &vst, &verr)) {
      std::cerr << "vectorize failed: " << verr << "\n";
      return 2;
    }

    std::ofstream os(outOwnerGeoJson);
    if (!os) {
      std::cerr << "failed to open: " << outOwnerGeoJson << "\n";
      return 2;
    }

    WriteGeoJsonHeader(os);
    bool first = true;
    for (const auto& g : geoms) {
      if (g.label <= 0) continue;
      const int owner = g.label - 1;
      if (owner < 0 || static_cast<std::size_t>(owner) >= snappedSources.size()) continue;
      if (g.geom.polygons.empty()) continue;

      if (!first) os << ",\n";
      first = false;

      os << "    {\"type\":\"Feature\",\"properties\":{";
      os << "\"owner\":" << owner << ',';
      os << "\"source_x\":" << snappedSources[static_cast<std::size_t>(owner)].x << ',';
      os << "\"source_y\":" << snappedSources[static_cast<std::size_t>(owner)].y << ',';
      os << "\"weight_mode\":\"" << WeightModeName(weightMode) << "\",";
      os << "\"require_outside_connection\":" << (requireOutside ? "true" : "false");
      os << "},\"geometry\":";
      WriteGeoJsonMultiPolygon(os, g.geom);
      os << "}";
    }

    WriteGeoJsonFooter(os);
    std::cout << "wrote owner geojson -> " << outOwnerGeoJson << "\n";
  }

  return 0;
}
