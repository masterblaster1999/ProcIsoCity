#include "isocity/BlockDistricting.hpp"
#include "isocity/DistrictStats.hpp"
#include "isocity/Export.hpp"
#include "isocity/GeoJsonExport.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Vectorize.hpp"
#include "isocity/World.hpp"

#include <array>
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
      << "proc_isocity_districtreport (headless district analytics + vector export)\n\n"
      << "Loads (or generates) a world, optionally auto-assigns administrative districts using\n"
      << "block-based districting, advances the simulation N days, then produces per-district\n"
      << "summary stats + optional GeoJSON/SVG exports for external tooling.\n\n"
      << "Usage:\n"
      << "  proc_isocity_districtreport [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                          [--days <N>] [--require-outside <0|1>]\n"
      << "                          [--auto-district <0|1>] [--districts <N>] [--fill-roads <0|1>]\n"
      << "                          [--include-water <0|1>]\n"
      << "                          [--json <out.json>] [--csv <out.csv>]\n"
      << "                          [--geojson <out.geojson>] [--svg <out.svg>]\n"
      << "                          [--svg-scale <N>] [--svg-labels <0|1>]\n"
      << "                          [--ppm <out.ppm|out.png>] [--scale <N>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>                World size for generation (default: 128x128).\n"
      << "  --days <N>                  Simulate N days before reporting (default: 60).\n"
      << "  --require-outside <0|1>     Enforce outside-connection rule (default: 1).\n\n"
      << "District assignment (optional):\n"
      << "  --auto-district <0|1>       If 1, overwrite districts using block-based districting (default: 0).\n"
      << "  --districts <N>             Requested number of districts (1..8). Default: 8\n"
      << "  --fill-roads <0|1>          If 1, assign road tiles to majority-adjacent district. Default: 1\n"
      << "  --include-water <0|1>       If 1, include water tiles in vector exports. Default: 0\n\n"
      << "Outputs:\n"
      << "  --json <out.json>           JSON report (per-district stats + summary).\n"
      << "  --csv <out.csv>             CSV (one row per district).\n"
      << "  --geojson <out.geojson>     GeoJSON polygons per district (tile-corner coords).\n"
      << "  --svg <out.svg>             Colored SVG district map (tile coords).\n"
      << "  --svg-scale <N>             SVG pixel scale (default: 16).\n"
      << "  --svg-labels <0|1>          If 1, add district id labels to SVG. Default: 1\n"
      << "  --ppm <out.ppm|out.png>     Raster district layer (one pixel per tile, optional upscale).\n"
      << "  --scale <N>                 Upscale factor for --ppm (default: 4).\n";
}

const char* DistrictColorHex(int d)
{
  // Match proc_isocity_blockdistricts DOT palette where possible.
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

struct Bounds {
  int minX = std::numeric_limits<int>::max();
  int minY = std::numeric_limits<int>::max();
  int maxX = std::numeric_limits<int>::min();
  int maxY = std::numeric_limits<int>::min();
  bool valid = false;
};

void UpdateBounds(Bounds& b, int x, int y)
{
  b.valid = true;
  b.minX = std::min(b.minX, x);
  b.minY = std::min(b.minY, y);
  b.maxX = std::max(b.maxX, x);
  b.maxY = std::max(b.maxY, y);
}

bool PickLabelTile(const World& world, int districtId, const Bounds& b, int& outX, int& outY)
{
  if (!b.valid) return false;
  const int w = world.width();
  const int h = world.height();
  const int cx = (b.minX + b.maxX) / 2;
  const int cy = (b.minY + b.maxY) / 2;

  auto isOk = [&](int x, int y) -> bool {
    if (x < 0 || y < 0 || x >= w || y >= h) return false;
    const Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water) return false;
    return static_cast<int>(t.district) == districtId;
  };

  // Deterministic expanding diamond search.
  const int maxR = std::max(w, h) + 4;
  for (int r = 0; r <= maxR; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      const int dy = r - std::abs(dx);
      const int x0 = cx + dx;
      const int y0 = cy + dy;
      if (isOk(x0, y0)) {
        outX = x0;
        outY = y0;
        return true;
      }
      if (dy != 0) {
        const int y1 = cy - dy;
        if (isOk(x0, y1)) {
          outX = x0;
          outY = y1;
          return true;
        }
      }
    }
  }
  return false;
}

struct GeomMeta {
  int polygons = 0;
  int holes = 0;
};

std::array<GeomMeta, static_cast<std::size_t>(kDistrictCount)> ComputeGeomMeta(const std::vector<LabeledGeometry>& geoms)
{
  std::array<GeomMeta, static_cast<std::size_t>(kDistrictCount)> meta{};
  for (const auto& g : geoms) {
    const int d = std::clamp(g.label, 0, kDistrictCount - 1);
    GeomMeta& m = meta[static_cast<std::size_t>(d)];
    m.polygons += static_cast<int>(g.geom.polygons.size());
    for (const auto& p : g.geom.polygons) {
      m.holes += static_cast<int>(p.holes.size());
    }
  }
  return meta;
}

bool WriteCsv(const std::string& path, const DistrictStatsResult& ds,
              const std::array<GeomMeta, static_cast<std::size_t>(kDistrictCount)>& meta)
{
  std::ofstream f(path);
  if (!f) return false;

  f << "id,tiles,landTiles,waterTiles,roads,parks,resTiles,comTiles,indTiles,zoneTiles,zoneTilesAccessible,population,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,avgLandValue,taxRevenue,maintenanceCost,net,polygons,holes\n";
  for (int d = 0; d < kDistrictCount; ++d) {
    const DistrictSummary& s = ds.districts[static_cast<std::size_t>(d)];
    const GeomMeta& gm = meta[static_cast<std::size_t>(d)];
    f << s.id << "," << s.tiles << "," << s.landTiles << "," << s.waterTiles << "," << s.roads << "," << s.parks
      << "," << s.resTiles << "," << s.comTiles << "," << s.indTiles << "," << s.zoneTiles << "," << s.zoneTilesAccessible
      << "," << s.population << "," << s.housingCapacity << "," << s.jobsCapacity << "," << s.jobsCapacityAccessible
      << "," << s.employed << "," << s.avgLandValue << "," << s.taxRevenue << "," << s.maintenanceCost << "," << s.net
      << "," << gm.polygons << "," << gm.holes << "\n";
  }
  return true;
}

bool WriteJson(const std::string& path, const World& world, int daysSimulated, const SimConfig& simCfg,
               bool autoDistrict, const BlockDistrictConfig& bdCfg, const BlockDistrictResult* bdRes,
               const DistrictStatsResult& ds, const VectorizeStats& vstats,
               const std::array<GeomMeta, static_cast<std::size_t>(kDistrictCount)>& meta)
{
  std::ofstream f(path);
  if (!f) return false;

  f << "{\n";
  f << "  \"width\": " << world.width() << ",\n";
  f << "  \"height\": " << world.height() << ",\n";
  f << "  \"seed\": " << world.seed() << ",\n";
  f << "  \"daysSimulated\": " << daysSimulated << ",\n";
  f << "  \"requireOutsideConnection\": " << (simCfg.requireOutsideConnection ? "true" : "false") << ",\n";

  f << "  \"vectorize\": {\n";
  f << "    \"labels\": " << vstats.labels << ",\n";
  f << "    \"rings\": " << vstats.rings << ",\n";
  f << "    \"polygons\": " << vstats.polygons << ",\n";
  f << "    \"holes\": " << vstats.holes << "\n";
  f << "  },\n";

  f << "  \"districting\": {\n";
  f << "    \"auto\": " << (autoDistrict ? "true" : "false") << ",\n";
  f << "    \"districtsRequested\": " << bdCfg.districts << ",\n";
  f << "    \"fillRoadTiles\": " << (bdCfg.fillRoadTiles ? "true" : "false") << ",\n";
  f << "    \"includeWater\": " << (bdCfg.includeWater ? "true" : "false") << ",\n";
  if (bdRes) {
    f << "    \"districtsUsed\": " << bdRes->districtsUsed << "\n";
  } else {
    f << "    \"districtsUsed\": 0\n";
  }
  f << "  },\n";

  f << "  \"districts\": [\n";
  for (int d = 0; d < kDistrictCount; ++d) {
    const DistrictSummary& s = ds.districts[static_cast<std::size_t>(d)];
    const GeomMeta& gm = meta[static_cast<std::size_t>(d)];
    f << "    {\n";
    f << "      \"id\": " << s.id << ",\n";
    f << "      \"tiles\": " << s.tiles << ",\n";
    f << "      \"landTiles\": " << s.landTiles << ",\n";
    f << "      \"waterTiles\": " << s.waterTiles << ",\n";
    f << "      \"roads\": " << s.roads << ",\n";
    f << "      \"parks\": " << s.parks << ",\n";
    f << "      \"zones\": {\"res\": " << s.resTiles << ", \"com\": " << s.comTiles << ", \"ind\": " << s.indTiles
      << ", \"total\": " << s.zoneTiles << ", \"accessible\": " << s.zoneTilesAccessible << "},\n";
    f << "      \"population\": " << s.population << ",\n";
    f << "      \"housingCapacity\": " << s.housingCapacity << ",\n";
    f << "      \"jobsCapacity\": " << s.jobsCapacity << ",\n";
    f << "      \"jobsCapacityAccessible\": " << s.jobsCapacityAccessible << ",\n";
    f << "      \"employed\": " << s.employed << ",\n";
    f << "      \"avgLandValue\": " << s.avgLandValue << ",\n";
    f << "      \"taxRevenue\": " << s.taxRevenue << ",\n";
    f << "      \"maintenanceCost\": " << s.maintenanceCost << ",\n";
    f << "      \"net\": " << s.net << ",\n";
    f << "      \"geometry\": {\"polygons\": " << gm.polygons << ", \"holes\": " << gm.holes << "}\n";
    f << "    }";
    if (d + 1 < kDistrictCount) f << ",";
    f << "\n";
  }
  f << "  ]\n";
  f << "}\n";
  return true;
}

bool WriteGeoJson(const std::string& path, const World& world, const DistrictStatsResult& ds,
                  const std::vector<LabeledGeometry>& geoms,
                  const std::array<Bounds, static_cast<std::size_t>(kDistrictCount)>& bounds)
{
  std::ofstream f(path);
  if (!f) return false;

  // Vectorize returns a sorted label list.
  std::size_t gi = 0;

  f << "{\n";
  f << "  \"type\": \"FeatureCollection\",\n";
  f << "  \"name\": \"districts\",\n";
  f << "  \"properties\": {\"width\": " << world.width() << ", \"height\": " << world.height() << ", \"seed\": "
    << world.seed() << "},\n";
  f << "  \"features\": [\n";

  bool first = true;
  for (int d = 0; d < kDistrictCount; ++d) {
    const DistrictSummary& s = ds.districts[static_cast<std::size_t>(d)];
    while (gi < geoms.size() && geoms[gi].label < d) gi++;
    const VectorMultiPolygon* mp = nullptr;
    if (gi < geoms.size() && geoms[gi].label == d) mp = &geoms[gi].geom;

    if (!mp || mp->polygons.empty()) continue;

    if (!first) f << ",\n";
    first = false;

    // Label point for external tooling.
    double labelX = 0.0;
    double labelY = 0.0;
    int lx = 0, ly = 0;
    if (PickLabelTile(world, d, bounds[static_cast<std::size_t>(d)], lx, ly)) {
      labelX = static_cast<double>(lx) + 0.5;
      labelY = static_cast<double>(ly) + 0.5;
    }

    f << "    {\n";
    f << "      \"type\": \"Feature\",\n";
    f << "      \"properties\": {";
    f << "\"id\": " << s.id;
    f << ", \"tiles\": " << s.tiles;
    f << ", \"landTiles\": " << s.landTiles;
    f << ", \"waterTiles\": " << s.waterTiles;
    f << ", \"roads\": " << s.roads;
    f << ", \"parks\": " << s.parks;
    f << ", \"resTiles\": " << s.resTiles;
    f << ", \"comTiles\": " << s.comTiles;
    f << ", \"indTiles\": " << s.indTiles;
    f << ", \"zoneTiles\": " << s.zoneTiles;
    f << ", \"zoneTilesAccessible\": " << s.zoneTilesAccessible;
    f << ", \"population\": " << s.population;
    f << ", \"housingCapacity\": " << s.housingCapacity;
    f << ", \"jobsCapacity\": " << s.jobsCapacity;
    f << ", \"jobsCapacityAccessible\": " << s.jobsCapacityAccessible;
    f << ", \"employed\": " << s.employed;
    f << ", \"avgLandValue\": " << s.avgLandValue;
    f << ", \"taxRevenue\": " << s.taxRevenue;
    f << ", \"maintenanceCost\": " << s.maintenanceCost;
    f << ", \"net\": " << s.net;
    f << ", \"labelX\": " << labelX;
    f << ", \"labelY\": " << labelY;
    f << "},\n";

    f << "      \"geometry\": ";
    WriteGeoJsonGeometry(f, *mp);
    f << "\n";
    f << "    }";
  }

  f << "\n  ]\n";
  f << "}\n";
  return true;
}

void WriteSvgRingPath(std::ostream& os, const std::vector<IPoint>& ring)
{
  if (ring.size() < 4) return;
  // Ring is closed; skip the final repeated point.
  os << "M " << ring[0].x << " " << ring[0].y;
  for (std::size_t i = 1; i + 1 < ring.size(); ++i) {
    os << " L " << ring[i].x << " " << ring[i].y;
  }
  os << " Z ";
}

bool WriteSvg(const std::string& path, const World& world, const DistrictStatsResult& ds,
              const std::vector<LabeledGeometry>& geoms,
              const std::array<Bounds, static_cast<std::size_t>(kDistrictCount)>& bounds,
              int svgScale, bool svgLabels)
{
  std::ofstream f(path);
  if (!f) return false;

  const int w = world.width();
  const int h = world.height();
  const int pxW = std::max(1, w * std::max(1, svgScale));
  const int pxH = std::max(1, h * std::max(1, svgScale));

  f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << pxW << "\" height=\"" << pxH
    << "\" viewBox=\"0 0 " << w << " " << h << "\">\n";

  // If water was excluded from vectorization, use a water-ish background.
  f << "  <rect x=\"0\" y=\"0\" width=\"" << w << "\" height=\"" << h
    << "\" fill=\"#b5d8ff\" />\n";

  f << "  <g fill-rule=\"evenodd\" stroke=\"#222\" stroke-width=\"0.06\" stroke-linejoin=\"round\">\n";

  for (const auto& lg : geoms) {
    const int d = std::clamp(lg.label, 0, kDistrictCount - 1);
    const char* color = DistrictColorHex(d);
    for (const auto& poly : lg.geom.polygons) {
      f << "    <path fill=\"" << color << "\" fill-opacity=\"0.75\" d=\"";
      WriteSvgRingPath(f, poly.outer);
      for (const auto& hole : poly.holes) {
        WriteSvgRingPath(f, hole);
      }
      f << "\" />\n";
    }
  }

  f << "  </g>\n";

  if (svgLabels) {
    f << "  <g font-family=\"sans-serif\" font-size=\"0.8\" text-anchor=\"middle\" dominant-baseline=\"middle\" "
         "fill=\"#111\" stroke=\"#ffffff\" stroke-width=\"0.10\" paint-order=\"stroke\">\n";
    for (int d = 0; d < kDistrictCount; ++d) {
      int lx = 0, ly = 0;
      if (!PickLabelTile(world, d, bounds[static_cast<std::size_t>(d)], lx, ly)) continue;
      const DistrictSummary& s = ds.districts[static_cast<std::size_t>(d)];
      if (s.landTiles <= 0) continue;

      const double x = static_cast<double>(lx) + 0.5;
      const double y = static_cast<double>(ly) + 0.5;
      f << "    <text x=\"" << x << "\" y=\"" << y << "\">" << d << "</text>\n";
    }
    f << "  </g>\n";
  }

  f << "</svg>\n";
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

  bool autoDistrict = false;
  BlockDistrictConfig bdCfg{};
  bdCfg.districts = kDistrictCount;
  bdCfg.fillRoadTiles = true;
  bdCfg.includeWater = false;

  // Vector export water inclusion (independent of bdCfg.includeWater).
  bool includeWaterInVector = false;

  std::string jsonPath;
  std::string csvPath;
  std::string geojsonPath;
  std::string svgPath;
  std::string ppmPath;

  int ppmScale = 4;
  int svgScale = 16;
  bool svgLabels = true;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, loadPath)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
    } else if (arg == "--seed") {
      std::string v;
      if (!requireValue(i, v) || !ParseU64(v, &seed)) {
        std::cerr << "--seed requires a u64 (decimal or 0x...)\n";
        return 2;
      }
    } else if (arg == "--size") {
      std::string v;
      if (!requireValue(i, v) || !ParseWxH(v, &w, &h)) {
        std::cerr << "--size requires WxH\n";
        return 2;
      }
    } else if (arg == "--days") {
      std::string v;
      if (!requireValue(i, v) || !ParseI32(v, &days) || days < 0) {
        std::cerr << "--days requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &requireOutside)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--auto-district") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &autoDistrict)) {
        std::cerr << "--auto-district requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--districts") {
      std::string v;
      int n = 0;
      if (!requireValue(i, v) || !ParseI32(v, &n)) {
        std::cerr << "--districts requires an integer\n";
        return 2;
      }
      bdCfg.districts = std::clamp(n, 1, kDistrictCount);
    } else if (arg == "--fill-roads") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &bdCfg.fillRoadTiles)) {
        std::cerr << "--fill-roads requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--include-water") {
      std::string v;
      bool b = false;
      if (!requireValue(i, v) || !ParseBool01(v, &b)) {
        std::cerr << "--include-water requires 0 or 1\n";
        return 2;
      }
      includeWaterInVector = b;
    } else if (arg == "--json") {
      if (!requireValue(i, jsonPath)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
    } else if (arg == "--csv") {
      if (!requireValue(i, csvPath)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
    } else if (arg == "--geojson") {
      if (!requireValue(i, geojsonPath)) {
        std::cerr << "--geojson requires a path\n";
        return 2;
      }
    } else if (arg == "--svg") {
      if (!requireValue(i, svgPath)) {
        std::cerr << "--svg requires a path\n";
        return 2;
      }
    } else if (arg == "--svg-scale") {
      std::string v;
      if (!requireValue(i, v) || !ParseI32(v, &svgScale) || svgScale <= 0) {
        std::cerr << "--svg-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--svg-labels") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &svgLabels)) {
        std::cerr << "--svg-labels requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--ppm") {
      if (!requireValue(i, ppmPath)) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
    } else if (arg == "--scale") {
      std::string v;
      if (!requireValue(i, v) || !ParseI32(v, &ppmScale) || ppmScale <= 0) {
        std::cerr << "--scale requires a positive integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (jsonPath.empty() && csvPath.empty() && geojsonPath.empty() && svgPath.empty() && ppmPath.empty()) {
    std::cerr << "No outputs specified. Use --json/--csv/--geojson/--svg/--ppm.\n";
    PrintHelp();
    return 2;
  }

  if (!EnsureParentDir(jsonPath) || !EnsureParentDir(csvPath) || !EnsureParentDir(geojsonPath) || !EnsureParentDir(svgPath) ||
      !EnsureParentDir(ppmPath)) {
    std::cerr << "Failed to create output directory.\n";
    return 2;
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

  simCfg.requireOutsideConnection = requireOutside;

  BlockDistrictResult bdRes{};
  BlockDistrictResult* bdPtr = nullptr;
  if (autoDistrict) {
    // Keep water assignment behavior stable unless the caller explicitly wants it.
    bdCfg.includeWater = includeWaterInVector;
    bdRes = AssignDistrictsByBlocks(world, bdCfg);
    bdPtr = &bdRes;
  }

  // Advance the simulation so occupancy/jobs are meaningful.
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }

  // Road-to-edge mask (optional, reused by derived computations).
  std::vector<std::uint8_t> roadToEdge;
  if (simCfg.requireOutsideConnection) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
  }

  // Land value field (required for DistrictStats tax revenue).
  LandValueConfig lvCfg{};
  lvCfg.requireOutsideConnection = simCfg.requireOutsideConnection;
  const LandValueResult lv = ComputeLandValue(world, lvCfg, nullptr,
                                              simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

  const DistrictStatsResult ds = ComputeDistrictStats(world, simCfg, &lv.value,
                                                     simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

  // Bounds (land tiles) for SVG labeling.
  std::array<Bounds, static_cast<std::size_t>(kDistrictCount)> bounds{};
  {
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);
        if (!includeWaterInVector && t.terrain == Terrain::Water) continue;
        const int did = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
        UpdateBounds(bounds[static_cast<std::size_t>(did)], x, y);
      }
    }
  }

  // Vectorize district regions.
  std::vector<int> labels;
  labels.resize(static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height()), -1);
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) + static_cast<std::size_t>(x);
      if (!includeWaterInVector && t.terrain == Terrain::Water) {
        labels[idx] = -1;
      } else {
        labels[idx] = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
      }
    }
  }

  std::vector<LabeledGeometry> geoms;
  VectorizeStats vstats{};
  std::string vErr;
  if (!VectorizeLabelGridToPolygons(labels, world.width(), world.height(), -1, geoms, &vstats, &vErr)) {
    std::cerr << "Failed to vectorize district grid: " << vErr << "\n";
    return 2;
  }

  const auto meta = ComputeGeomMeta(geoms);

  if (!jsonPath.empty()) {
    if (!WriteJson(jsonPath, world, days, simCfg, autoDistrict, bdCfg, bdPtr, ds, vstats, meta)) {
      std::cerr << "Failed to write JSON: " << jsonPath << "\n";
      return 2;
    }
  }

  if (!csvPath.empty()) {
    if (!WriteCsv(csvPath, ds, meta)) {
      std::cerr << "Failed to write CSV: " << csvPath << "\n";
      return 2;
    }
  }

  if (!geojsonPath.empty()) {
    if (!WriteGeoJson(geojsonPath, world, ds, geoms, bounds)) {
      std::cerr << "Failed to write GeoJSON: " << geojsonPath << "\n";
      return 2;
    }
  }

  if (!svgPath.empty()) {
    if (!WriteSvg(svgPath, world, ds, geoms, bounds, svgScale, svgLabels)) {
      std::cerr << "Failed to write SVG: " << svgPath << "\n";
      return 2;
    }
  }

  if (!ppmPath.empty()) {
    PpmImage img = RenderPpmLayer(world, ExportLayer::District);
    if (ppmScale > 1) img = ScaleNearest(img, ppmScale);
    std::string err;
    if (!WriteImageAuto(ppmPath, img, err)) {
      std::cerr << "Failed to write image: " << err << "\n";
      return 2;
    }
  }

  return 0;
}
