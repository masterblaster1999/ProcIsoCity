#include "isocity/Export.hpp"
#include "isocity/GeoJsonExport.hpp"
#include "isocity/Hydrology.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Vectorize.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_hydrology (headless terrain hydrology analysis)\n\n"
      << "Compute a simple D4 flow-direction + accumulation field from tile heights and\n"
      << "export useful artifacts (accumulation heatmap, river mask, basin segmentation,\n"
      << "GeoJSON river lines / basin polygons).\n\n"
      << "Usage:\n"
      << "  proc_isocity_hydrology [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                      [--min-accum <N>] [--accum-log <0|1>]\n"
      << "                      [--accum <out.png>] [--rivers <out.png>] [--basins <out.png>]\n"
      << "                      [--scale <N>]\n"
      << "                      [--geojson-rivers <out.geojson>]\n"
      << "                      [--geojson-river-polys <out.geojson>]\n"
      << "                      [--geojson-basins <out.geojson> --top-basins <K> --min-basin-area <N>]\n"
      << "                      [--apply-rivers-water <0|1>] [--bulldoze-water <0|1>] [--save <out.bin>]\n"
      << "                      [--json <out.json>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load a save file (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>                World size (default: 96x96).\n\n"
      << "Hydrology controls:\n"
      << "  --min-accum <N>             River threshold (min accumulation). 0 => auto (default).\n"
      << "  --accum-log <0|1>           Log-scale accumulation visualization (default: 1).\n\n"
      << "Raster outputs:\n"
      << "  --accum <out.ppm|out.png>   Accumulation heatmap (grayscale).\n"
      << "  --rivers <out.ppm|out.png>  River mask (blue-on-black) for cells >= min-accum.\n"
      << "  --basins <out.ppm|out.png>  Basin segmentation visualization (hashed colors).\n"
      << "  --scale <N>                 Nearest-neighbor upscale for raster outputs (default: 1).\n\n"
      << "Vector outputs:\n"
      << "  --geojson-rivers <path>      River centerlines as GeoJSON LineString features.\n"
      << "  --geojson-river-polys <path> River area as GeoJSON Polygon/MultiPolygon (tile-corner space).\n"
      << "  --geojson-basins <path>      Top basin polygons as GeoJSON features (tile-corner space).\n"
      << "  --top-basins <K>             Number of basins to export (default: 0).\n"
      << "  --min-basin-area <N>         Minimum basin area (cells) to export (default: 0).\n\n"
      << "Optional world edit:\n"
      << "  --apply-rivers-water <0|1>   Convert river cells to Terrain::Water (default: 0).\n"
      << "  --bulldoze-water <0|1>       When converting to water, clear non-road overlays (default: 1).\n"
      << "  --save <out.bin>             Write the (optionally modified) world to a save file.\n\n"
      << "Report:\n"
      << "  --json <out.json>            Emit a JSON summary report.\n\n"
      << "Examples:\n"
      << "  # Export hydrology artifacts for a generated world\n"
      << "  proc_isocity_hydrology --seed 1 --size 128x128 --accum accum.png --rivers rivers.png --basins basins.png\n\n"
      << "  # Export river lines + top 10 basins to GeoJSON\n"
      << "  proc_isocity_hydrology --load city.bin --geojson-rivers rivers.geojson \\\n"
      << "    --geojson-basins basins.geojson --top-basins 10 --min-basin-area 200\n";
}

static bool ParseI32(const std::string& s, int* out)
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

static bool ParseU64(const std::string& s, std::uint64_t* out)
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

static bool ParseSize(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t x = s.find('x');
  if (x == std::string::npos) return false;
  int w = 0, h = 0;
  if (!ParseI32(s.substr(0, x), &w)) return false;
  if (!ParseI32(s.substr(x + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

static bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  if (s == "0") {
    *out = false;
    return true;
  }
  if (s == "1") {
    *out = true;
    return true;
  }
  return false;
}

static bool EnsureParentDir(const std::string& path)
{
  try {
    fs::path p(path);
    if (!p.has_parent_path()) return true;
    fs::path dir = p.parent_path();
    if (dir.empty()) return true;
    if (fs::exists(dir)) return true;
    return fs::create_directories(dir);
  } catch (...) {
    return false;
  }
}

static std::uint32_t Hash32(std::uint32_t x)
{
  // A small 32-bit mix function (avalanche). Deterministic, fast.
  x += 0x9E3779B9u;
  x = (x ^ (x >> 16)) * 0x7FEB352Du;
  x = (x ^ (x >> 15)) * 0x846CA68Bu;
  x = (x ^ (x >> 16));
  return x;
}

static PpmImage MakeAccumImage(const HydrologyField& field, bool logScale)
{
  PpmImage img;
  img.width = field.w;
  img.height = field.h;
  if (field.empty()) return img;

  const int total = field.w * field.h;
  img.rgb.resize(static_cast<std::size_t>(total) * 3u, 0);

  const int maxA = std::max(1, field.maxAccum);
  const double denomLog = std::log(1.0 + static_cast<double>(maxA));

  for (int i = 0; i < total; ++i) {
    const int a = (i < static_cast<int>(field.accum.size())) ? field.accum[static_cast<std::size_t>(i)] : 1;
    double t = 0.0;
    if (logScale) {
      t = std::log(1.0 + static_cast<double>(std::max(0, a))) / denomLog;
    } else {
      t = (maxA <= 1) ? 0.0 : static_cast<double>(a - 1) / static_cast<double>(maxA - 1);
    }
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    const std::uint8_t g = static_cast<std::uint8_t>(std::lround(t * 255.0));
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    img.rgb[o + 0] = g;
    img.rgb[o + 1] = g;
    img.rgb[o + 2] = g;
  }

  return img;
}

static PpmImage MakeRiverMaskImage(int w, int h, const std::vector<std::uint8_t>& riverMask)
{
  PpmImage img;
  img.width = w;
  img.height = h;
  if (w <= 0 || h <= 0) return img;

  const int total = w * h;
  img.rgb.resize(static_cast<std::size_t>(total) * 3u, 0);

  for (int i = 0; i < total; ++i) {
    const bool r = (i < static_cast<int>(riverMask.size())) ? (riverMask[static_cast<std::size_t>(i)] != 0) : false;
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    if (r) {
      img.rgb[o + 0] = 0;
      img.rgb[o + 1] = 96;
      img.rgb[o + 2] = 255;
    }
  }

  return img;
}

static PpmImage MakeBasinsImage(const BasinSegmentation& seg)
{
  PpmImage img;
  img.width = seg.w;
  img.height = seg.h;
  if (seg.empty()) return img;

  const int total = seg.w * seg.h;
  img.rgb.resize(static_cast<std::size_t>(total) * 3u, 0);

  std::vector<std::array<std::uint8_t, 3>> colors;
  colors.resize(seg.basins.size());
  for (std::size_t i = 0; i < seg.basins.size(); ++i) {
    const std::uint32_t h32 = Hash32(static_cast<std::uint32_t>(i) * 0xA511E9B3u + 0xC0FFEE11u);
    std::uint8_t r = static_cast<std::uint8_t>((h32)&0xFFu);
    std::uint8_t g = static_cast<std::uint8_t>((h32 >> 8) & 0xFFu);
    std::uint8_t b = static_cast<std::uint8_t>((h32 >> 16) & 0xFFu);
    // Avoid extremely dark regions.
    r |= 0x40u;
    g |= 0x40u;
    b |= 0x40u;
    colors[i] = {r, g, b};
  }

  for (int i = 0; i < total; ++i) {
    const int id = (i < static_cast<int>(seg.basinId.size())) ? seg.basinId[static_cast<std::size_t>(i)] : -1;
    if (id < 0 || id >= static_cast<int>(colors.size())) continue;
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    img.rgb[o + 0] = colors[static_cast<std::size_t>(id)][0];
    img.rgb[o + 1] = colors[static_cast<std::size_t>(id)][1];
    img.rgb[o + 2] = colors[static_cast<std::size_t>(id)][2];
  }

  return img;
}

struct RiverSegment {
  std::vector<std::pair<double, double>> pts; // (x,y) in tile coordinates
  int startIdx = -1;
  int endIdx = -1;
  int startAccum = 0;
  int endAccum = 0;
};

static std::pair<double, double> CenterXY(int idx, int w)
{
  const int x = (w > 0) ? (idx % w) : 0;
  const int y = (w > 0) ? (idx / w) : 0;
  return {static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5};
}

static std::vector<RiverSegment> ExtractRiverSegments(const HydrologyField& field, const std::vector<std::uint8_t>& riverMask)
{
  std::vector<RiverSegment> segs;
  if (field.empty()) return segs;

  const int w = field.w;
  const int h = field.h;
  const int total = w * h;
  if (static_cast<int>(field.dir.size()) != total || static_cast<int>(field.accum.size()) != total) return segs;
  if (static_cast<int>(riverMask.size()) != total) return segs;

  std::vector<int> indeg(total, 0);
  for (int i = 0; i < total; ++i) {
    if (riverMask[static_cast<std::size_t>(i)] == 0) continue;
    const int to = field.dir[static_cast<std::size_t>(i)];
    if (to < 0 || to >= total) continue;
    if (riverMask[static_cast<std::size_t>(to)] == 0) continue;
    indeg[to] += 1;
  }

  for (int i = 0; i < total; ++i) {
    if (riverMask[static_cast<std::size_t>(i)] == 0) continue;
    if (indeg[i] == 1) continue; // only start at sources/junctions

    const int to0 = field.dir[static_cast<std::size_t>(i)];
    if (to0 < 0 || to0 >= total) continue;
    if (riverMask[static_cast<std::size_t>(to0)] == 0) continue; // no downstream river edge

    RiverSegment s;
    s.startIdx = i;
    s.startAccum = field.accum[static_cast<std::size_t>(i)];

    int cur = i;
    s.pts.push_back(CenterXY(cur, w));

    while (true) {
      const int nxt = field.dir[static_cast<std::size_t>(cur)];
      if (nxt < 0 || nxt >= total) break;
      if (riverMask[static_cast<std::size_t>(nxt)] == 0) break;

      s.pts.push_back(CenterXY(nxt, w));
      cur = nxt;

      if (indeg[cur] != 1) break; // stop at junction
    }

    s.endIdx = cur;
    s.endAccum = field.accum[static_cast<std::size_t>(cur)];
    if (s.pts.size() >= 2) segs.push_back(std::move(s));
  }

  // Deterministic ordering: sort segments by start index (then end).
  std::sort(segs.begin(), segs.end(), [&](const RiverSegment& a, const RiverSegment& b) {
    if (a.startIdx != b.startIdx) return a.startIdx < b.startIdx;
    return a.endIdx < b.endIdx;
  });

  return segs;
}

static void WriteGeoJsonLineCoords(std::ostream& os, const std::vector<std::pair<double, double>>& pts)
{
  os << "[";
  os << std::fixed << std::setprecision(6);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (i) os << ",";
    os << "[" << pts[i].first << "," << pts[i].second << "]";
  }
  os << "]";
}

static void WriteJsonSummary(std::ostream& os,
                             const World& world,
                             const HydrologyField& field,
                             int riverMinAccumUsed,
                             int riverCells,
                             const BasinSegmentation& basins)
{
  os << "{\n";
  os << "  \"world\": {\"w\": " << world.width() << ", \"h\": " << world.height() << ", \"seed\": " << world.seed() << "},\n";
  os << "  \"hydrology\": {\n";
  os << "    \"maxAccum\": " << field.maxAccum << ",\n";
  os << "    \"riverMinAccum\": " << riverMinAccumUsed << ",\n";
  os << "    \"riverCells\": " << riverCells << "\n";
  os << "  },\n";
  os << "  \"basins\": {\n";
  os << "    \"count\": " << basins.basins.size() << ",\n";
  os << "    \"top\": [\n";

  const std::size_t topN = std::min<std::size_t>(basins.basins.size(), 10u);
  for (std::size_t i = 0; i < topN; ++i) {
    const BasinInfo& b = basins.basins[i];
    os << "      {\"id\": " << b.id << ", \"area\": " << b.area
       << ", \"sink\": {\"x\": " << b.sinkX << ", \"y\": " << b.sinkY << "}}";
    if (i + 1 < topN) os << ",";
    os << "\n";
  }

  os << "    ]\n";
  os << "  }\n";
  os << "}\n";
}

struct Options {
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  int minAccum = 0;       // 0 => auto
  bool accumLog = true;

  std::string outAccum;
  std::string outRivers;
  std::string outBasins;
  int scale = 1;

  std::string outGeoRivers;
  std::string outGeoRiverPolys;
  std::string outGeoBasins;

  int topBasins = 0;
  int minBasinArea = 0;

  bool applyRiversWater = false;
  bool bulldozeWater = true;
  std::string outSave;

  std::string outJson;
};

static int Run(int argc, char** argv)
{
  Options opt;

  if (argc <= 1) {
    PrintHelp();
    return 0;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](int n) -> bool { return (i + n) < argc; };

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--load") {
      if (!need(1)) {
        std::cerr << "--load requires a value\n";
        return 2;
      }
      opt.loadPath = argv[++i];
    } else if (a == "--seed") {
      if (!need(1)) {
        std::cerr << "--seed requires a value\n";
        return 2;
      }
      if (!ParseU64(argv[++i], &opt.seed)) {
        std::cerr << "Invalid --seed\n";
        return 2;
      }
    } else if (a == "--size") {
      if (!need(1)) {
        std::cerr << "--size requires a value\n";
        return 2;
      }
      if (!ParseSize(argv[++i], &opt.w, &opt.h)) {
        std::cerr << "Invalid --size (expected WxH)\n";
        return 2;
      }
    } else if (a == "--min-accum") {
      if (!need(1)) {
        std::cerr << "--min-accum requires a value\n";
        return 2;
      }
      if (!ParseI32(argv[++i], &opt.minAccum)) {
        std::cerr << "Invalid --min-accum\n";
        return 2;
      }
    } else if (a == "--accum-log") {
      if (!need(1)) {
        std::cerr << "--accum-log requires 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.accumLog)) {
        std::cerr << "Invalid --accum-log (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--accum") {
      if (!need(1)) {
        std::cerr << "--accum requires a path\n";
        return 2;
      }
      opt.outAccum = argv[++i];
    } else if (a == "--rivers") {
      if (!need(1)) {
        std::cerr << "--rivers requires a path\n";
        return 2;
      }
      opt.outRivers = argv[++i];
    } else if (a == "--basins") {
      if (!need(1)) {
        std::cerr << "--basins requires a path\n";
        return 2;
      }
      opt.outBasins = argv[++i];
    } else if (a == "--scale") {
      if (!need(1)) {
        std::cerr << "--scale requires a value\n";
        return 2;
      }
      if (!ParseI32(argv[++i], &opt.scale) || opt.scale < 1) {
        std::cerr << "Invalid --scale\n";
        return 2;
      }
    } else if (a == "--geojson-rivers") {
      if (!need(1)) {
        std::cerr << "--geojson-rivers requires a path\n";
        return 2;
      }
      opt.outGeoRivers = argv[++i];
    } else if (a == "--geojson-river-polys") {
      if (!need(1)) {
        std::cerr << "--geojson-river-polys requires a path\n";
        return 2;
      }
      opt.outGeoRiverPolys = argv[++i];
    } else if (a == "--geojson-basins") {
      if (!need(1)) {
        std::cerr << "--geojson-basins requires a path\n";
        return 2;
      }
      opt.outGeoBasins = argv[++i];
    } else if (a == "--top-basins") {
      if (!need(1)) {
        std::cerr << "--top-basins requires a value\n";
        return 2;
      }
      if (!ParseI32(argv[++i], &opt.topBasins) || opt.topBasins < 0) {
        std::cerr << "Invalid --top-basins\n";
        return 2;
      }
    } else if (a == "--min-basin-area") {
      if (!need(1)) {
        std::cerr << "--min-basin-area requires a value\n";
        return 2;
      }
      if (!ParseI32(argv[++i], &opt.minBasinArea) || opt.minBasinArea < 0) {
        std::cerr << "Invalid --min-basin-area\n";
        return 2;
      }
    } else if (a == "--apply-rivers-water") {
      if (!need(1)) {
        std::cerr << "--apply-rivers-water requires 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyRiversWater)) {
        std::cerr << "Invalid --apply-rivers-water (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--bulldoze-water") {
      if (!need(1)) {
        std::cerr << "--bulldoze-water requires 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.bulldozeWater)) {
        std::cerr << "Invalid --bulldoze-water (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--save") {
      if (!need(1)) {
        std::cerr << "--save requires a path\n";
        return 2;
      }
      opt.outSave = argv[++i];
    } else if (a == "--json") {
      if (!need(1)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      opt.outJson = argv[++i];
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      return 2;
    }
  }

  // Load or generate the world.
  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::string err;

  if (!opt.loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, opt.loadPath, err)) {
      std::cerr << "Failed to load save: " << opt.loadPath << "\n" << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(opt.w, opt.h, opt.seed, procCfg);
  }

  // Extract heights.
  const int w = world.width();
  const int h = world.height();
  std::vector<float> heights;
  heights.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = world.at(x, y).height;
    }
  }

  HydrologyField field = BuildHydrologyField(heights, w, h);

  int minAccumUsed = opt.minAccum;
  if (minAccumUsed <= 0) minAccumUsed = AutoRiverMinAccum(w, h);
  minAccumUsed = std::max(2, minAccumUsed);

  std::vector<std::uint8_t> riverMask = BuildRiverMask(field.accum, w, h, minAccumUsed);
  int riverCells = 0;
  for (std::uint8_t v : riverMask) riverCells += (v != 0) ? 1 : 0;

  BasinSegmentation basins = SegmentBasins(field.dir, w, h);

  // Optional world edit: convert river cells to water.
  if (opt.applyRiversWater && !riverMask.empty()) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        if (riverMask[i] == 0) continue;

        Tile& t = world.at(x, y);
        if (t.terrain != Terrain::Water) {
          t.terrain = Terrain::Water;
          if (opt.bulldozeWater && t.overlay != Overlay::None && t.overlay != Overlay::Road) {
            world.setOverlay(Overlay::None, x, y);
          }
        }
      }
    }
  }

  // Raster outputs.
  if (!opt.outAccum.empty()) {
    PpmImage img = MakeAccumImage(field, opt.accumLog);
    if (opt.scale > 1) img = ScaleNearest(img, opt.scale);

    if (!EnsureParentDir(opt.outAccum)) {
      std::cerr << "Failed to create output directory for: " << opt.outAccum << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outAccum, img, err)) {
      std::cerr << "Failed to write image: " << opt.outAccum << "\n" << err << "\n";
      return 2;
    }
  }

  if (!opt.outRivers.empty()) {
    PpmImage img = MakeRiverMaskImage(w, h, riverMask);
    if (opt.scale > 1) img = ScaleNearest(img, opt.scale);

    if (!EnsureParentDir(opt.outRivers)) {
      std::cerr << "Failed to create output directory for: " << opt.outRivers << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outRivers, img, err)) {
      std::cerr << "Failed to write image: " << opt.outRivers << "\n" << err << "\n";
      return 2;
    }
  }

  if (!opt.outBasins.empty()) {
    PpmImage img = MakeBasinsImage(basins);
    if (opt.scale > 1) img = ScaleNearest(img, opt.scale);

    if (!EnsureParentDir(opt.outBasins)) {
      std::cerr << "Failed to create output directory for: " << opt.outBasins << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outBasins, img, err)) {
      std::cerr << "Failed to write image: " << opt.outBasins << "\n" << err << "\n";
      return 2;
    }
  }

  // GeoJSON: river line segments.
  if (!opt.outGeoRivers.empty()) {
    const std::vector<RiverSegment> segs = ExtractRiverSegments(field, riverMask);

    if (!EnsureParentDir(opt.outGeoRivers)) {
      std::cerr << "Failed to create output directory for: " << opt.outGeoRivers << "\n";
      return 2;
    }

    std::ofstream os(opt.outGeoRivers, std::ios::binary);
    if (!os) {
      std::cerr << "Failed to open: " << opt.outGeoRivers << "\n";
      return 2;
    }

    os << "{\n";
    os << "  \"type\": \"FeatureCollection\",\n";
    os << "  \"properties\": {\"w\": " << w << ", \"h\": " << h << ", \"riverMinAccum\": " << minAccumUsed << "},\n";
    os << "  \"features\": [\n";

    for (std::size_t i = 0; i < segs.size(); ++i) {
      const RiverSegment& s = segs[i];
      os << "    {\"type\":\"Feature\",\"properties\":{\"segment\":" << i
         << ",\"startAccum\":" << s.startAccum << ",\"endAccum\":" << s.endAccum
         << ",\"points\":" << s.pts.size() << "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":";
      WriteGeoJsonLineCoords(os, s.pts);
      os << "}}";
      if (i + 1 < segs.size()) os << ",";
      os << "\n";
    }

    os << "  ]\n";
    os << "}\n";
  }

  // GeoJSON: river area polygons.
  if (!opt.outGeoRiverPolys.empty()) {
    if (!EnsureParentDir(opt.outGeoRiverPolys)) {
      std::cerr << "Failed to create output directory for: " << opt.outGeoRiverPolys << "\n";
      return 2;
    }

    std::ofstream os(opt.outGeoRiverPolys, std::ios::binary);
    if (!os) {
      std::cerr << "Failed to open: " << opt.outGeoRiverPolys << "\n";
      return 2;
    }

    std::vector<int> labels;
    labels.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    for (int i = 0; i < w * h; ++i) {
      if (!riverMask.empty() && riverMask[static_cast<std::size_t>(i)] != 0) labels[static_cast<std::size_t>(i)] = 1;
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats vs{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, w, h, 0, geoms, &vs, &vErr)) {
      std::cerr << "Vectorize failed for river mask\n" << vErr << "\n";
      return 2;
    }

    os << "{\n";
    os << "  \"type\": \"FeatureCollection\",\n";
    os << "  \"properties\": {\"w\": " << w << ", \"h\": " << h << ", \"riverMinAccum\": " << minAccumUsed << "},\n";
    os << "  \"features\": [\n";

    bool first = true;
    for (const auto& g : geoms) {
      if (g.label != 1) continue;
      if (!first) os << ",\n";
      first = false;

      os << "    {\"type\":\"Feature\",\"properties\":{\"kind\":\"river_area\",\"minAccum\":" << minAccumUsed
         << ",\"cells\":" << riverCells << "},\"geometry\":";
      WriteGeoJsonGeometry(os, g.geom);
      os << "}";
    }

    os << "\n  ]\n";
    os << "}\n";
  }

  // GeoJSON: basin polygons (top K).
  if (!opt.outGeoBasins.empty()) {
    if (opt.topBasins <= 0) {
      std::cerr << "--geojson-basins requires --top-basins > 0\n";
      return 2;
    }

    std::vector<std::uint8_t> include;
    include.assign(basins.basins.size(), 0);

    int kept = 0;
    for (const BasinInfo& b : basins.basins) {
      if (kept >= opt.topBasins) break;
      if (b.area < opt.minBasinArea) continue;
      if (b.id >= 0 && b.id < static_cast<int>(include.size())) {
        include[static_cast<std::size_t>(b.id)] = 1;
        kept++;
      }
    }

    std::vector<int> labels;
    labels.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);
    for (int i = 0; i < w * h; ++i) {
      const int bid = (i < static_cast<int>(basins.basinId.size())) ? basins.basinId[static_cast<std::size_t>(i)] : -1;
      if (bid < 0 || bid >= static_cast<int>(include.size())) continue;
      if (include[static_cast<std::size_t>(bid)] == 0) continue;
      labels[static_cast<std::size_t>(i)] = bid;
    }

    std::vector<LabeledGeometry> geoms;
    VectorizeStats vs{};
    std::string vErr;
    if (!VectorizeLabelGridToPolygons(labels, w, h, -1, geoms, &vs, &vErr)) {
      std::cerr << "Vectorize failed for basins\n" << vErr << "\n";
      return 2;
    }

    if (!EnsureParentDir(opt.outGeoBasins)) {
      std::cerr << "Failed to create output directory for: " << opt.outGeoBasins << "\n";
      return 2;
    }

    std::ofstream os(opt.outGeoBasins, std::ios::binary);
    if (!os) {
      std::cerr << "Failed to open: " << opt.outGeoBasins << "\n";
      return 2;
    }

    os << "{\n";
    os << "  \"type\": \"FeatureCollection\",\n";
    os << "  \"properties\": {\"w\": " << w << ", \"h\": " << h << "},\n";
    os << "  \"features\": [\n";

    for (std::size_t i = 0; i < geoms.size(); ++i) {
      const LabeledGeometry& g = geoms[i];
      const int bid = g.label;
      if (bid < 0 || bid >= static_cast<int>(basins.basins.size())) continue;
      const BasinInfo& info = basins.basins[static_cast<std::size_t>(bid)];

      os << "    {\"type\":\"Feature\",\"properties\":{\"basinId\":" << bid << ",\"area\":" << info.area
         << ",\"sinkX\":" << info.sinkX << ",\"sinkY\":" << info.sinkY << "},\"geometry\":";
      WriteGeoJsonGeometry(os, g.geom);
      os << "}";
      if (i + 1 < geoms.size()) os << ",";
      os << "\n";
    }

    os << "  ]\n";
    os << "}\n";
  }

  // Save world (optional).
  if (!opt.outSave.empty()) {
    if (!EnsureParentDir(opt.outSave)) {
      std::cerr << "Failed to create output directory for: " << opt.outSave << "\n";
      return 2;
    }
    if (!SaveWorldBinary(world, procCfg, simCfg, opt.outSave, err)) {
      std::cerr << "Failed to save: " << opt.outSave << "\n" << err << "\n";
      return 2;
    }
  }

  // JSON report (optional).
  if (!opt.outJson.empty()) {
    if (!EnsureParentDir(opt.outJson)) {
      std::cerr << "Failed to create output directory for: " << opt.outJson << "\n";
      return 2;
    }
    std::ofstream os(opt.outJson, std::ios::binary);
    if (!os) {
      std::cerr << "Failed to open: " << opt.outJson << "\n";
      return 2;
    }
    WriteJsonSummary(os, world, field, minAccumUsed, riverCells, basins);
  }

  return 0;
}

} // namespace

int main(int argc, char** argv)
{
  try {
    return Run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Fatal error: unknown exception\n";
    return 1;
  }
}
