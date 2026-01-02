#include "isocity/DepressionFill.hpp"
#include "isocity/Export.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/Json.hpp"
#include "isocity/GeoJsonExport.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Vectorize.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_floodrisk (headless flood / ponding risk analysis)\n\n"
      << "This tool provides two complementary analyses on the world heightfield:\n"
      << "  1) Sea-level flooding (connectivity-based coastal inundation)\n"
      << "  2) Depression fill depth (Priority-Flood ponding potential / sink filling)\n\n"
      << "Usage:\n"
      << "  proc_isocity_floodrisk [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                       [--days <N>] [--mode <sea|depressions|both>]\n"
      << "                       [--sea-level <F>] [--sea-connect-edge <0|1>] [--sea-8conn <0|1>]\n"
      << "                       [--dep-eps <F>] [--dep-min-depth <F>] [--dep-8conn <0|1>]\n"
      << "                       [--scale <N>]\n"
      << "                       [--sea-mask <out.png>] [--sea-depth <out.png>] [--sea-annotate <out.png>]\n"
      << "                       [--dep-filled <out.png>] [--dep-depth <out.png>] [--dep-annotate <out.png>]\n"
      << "                       [--sea-geojson <out.geojson>] [--dep-geojson <out.geojson>]\n"
      << "                       [--json <out.json>]\n"
      << "                       [--apply <none|sea|depressions|both>] [--bulldoze-water <0|1>]\n"
      << "                       [--save <out.bin>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>                World size (default: 96x96).\n"
      << "  --days <N>                  Simulate N days before analysis (default: 0).\n\n"
      << "Mode:\n"
      << "  --mode <sea|depressions|both>  Which analysis to run (default: both).\n\n"
      << "Sea-level flood controls:\n"
      << "  --sea-level <F>             Sea level threshold in height units. If omitted,\n"
      << "                             defaults to the save/procgen waterLevel when available.\n"
      << "  --sea-connect-edge <0|1>    Only flood regions connected to map edge (default: 1).\n"
      << "  --sea-8conn <0|1>           Use 8-neighborhood connectivity (default: 0).\n\n"
      << "Depression/ponding controls:\n"
      << "  --dep-eps <F>               Epsilon lift when filling (default: 0).\n"
      << "  --dep-min-depth <F>         Minimum depth to include in component outputs (default: 0.001).\n"
      << "  --dep-8conn <0|1>           Use 8-neighborhood connectivity (default: 0).\n\n"
      << "Outputs:\n"
      << "  --scale <N>                 Nearest-neighbor upscale for raster outputs (default: 1).\n"
      << "  --sea-mask <path>           Coastal flood mask (blue on black).\n"
      << "  --sea-depth <path>          Coastal flood depth (blue intensity).\n"
      << "  --sea-annotate <path>       Overlay flood mask on the city's overlay render.\n"
      << "  --dep-filled <path>         Filled heightfield (grayscale).\n"
      << "  --dep-depth <path>          Depression fill depth (grayscale).\n"
      << "  --dep-annotate <path>       Overlay depression mask on the city's overlay render.\n"
      << "  --sea-geojson <path>        Flood polygons as GeoJSON (tile-corner coordinates).\n"
      << "  --dep-geojson <path>        Depression polygons as GeoJSON (tile-corner coordinates).\n"
      << "  --json <path>               JSON summary report.\n\n"
      << "Optional world edit:\n"
      << "  --apply <mode>              Convert flooded/depression tiles to Terrain::Water (default: none).\n"
      << "  --bulldoze-water <0|1>       When converting to water, clear non-road overlays (default: 1).\n"
      << "  --save <out.bin>             Write the modified world to a save file.\n\n"
      << "Examples:\n"
      << "  # Analyze coastal flood at a raised sea level and export mask + polygons\n"
      << "  ./build/proc_isocity_floodrisk --seed 1 --size 128x128 --mode sea --sea-level 0.45 \\\n"
      << "    --sea-mask sea.png --sea-depth sea_depth.png --sea-geojson sea.geojson --json sea_report.json\n\n"
      << "  # Compute ponding potential (Priority-Flood depth) and export polygons\n"
      << "  ./build/proc_isocity_floodrisk --load save.bin --mode depressions --dep-depth ponds.png \\\n"
      << "    --dep-geojson ponds.geojson --dep-min-depth 0.01\n";
}

static bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  long v = std::strtol(s.c_str(), &end, 10);
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
  unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

static bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = static_cast<float>(v);
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

static bool ParseSize(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t x = s.find_first_of("xX");
  if (x == std::string::npos) return false;
  int w = 0, h = 0;
  if (!ParseI32(s.substr(0, x), &w)) return false;
  if (!ParseI32(s.substr(x + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
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

static std::vector<float> ExtractHeights(const World& world)
{
  std::vector<float> h;
  const int w = world.width();
  const int hh = world.height();
  h.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(hh));
  for (int y = 0; y < hh; ++y) {
    for (int x = 0; x < w; ++x) {
      h[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          world.at(x, y).height;
    }
  }
  return h;
}

static std::vector<std::uint8_t> BuildWaterDrainMask(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  std::vector<std::uint8_t> mask;
  mask.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water && t.overlay != Overlay::Road) {
        mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = 1;
      }
    }
  }
  return mask;
}

static PpmImage MakeMaskImage(int w, int h, const std::vector<std::uint8_t>& mask, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  PpmImage img;
  img.width = w;
  img.height = h;
  if (w <= 0 || h <= 0) return img;
  const int n = w * h;
  img.rgb.assign(static_cast<std::size_t>(n) * 3u, 0);

  for (int i = 0; i < n; ++i) {
    if (i >= static_cast<int>(mask.size())) break;
    if (mask[static_cast<std::size_t>(i)] == 0) continue;
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    img.rgb[o + 0] = r;
    img.rgb[o + 1] = g;
    img.rgb[o + 2] = b;
  }
  return img;
}

static PpmImage MakeDepthImageBlue(int w, int h, const std::vector<float>& depth, float maxDepth)
{
  PpmImage img;
  img.width = w;
  img.height = h;
  if (w <= 0 || h <= 0) return img;
  const int n = w * h;
  img.rgb.assign(static_cast<std::size_t>(n) * 3u, 0);

  if (maxDepth <= 0.0f) return img;

  for (int i = 0; i < n; ++i) {
    if (i >= static_cast<int>(depth.size())) break;
    const float d = depth[static_cast<std::size_t>(i)];
    if (d <= 0.0f) continue;
    float t = d / maxDepth;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const std::uint8_t v = static_cast<std::uint8_t>(std::lround(t * 255.0f));
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    img.rgb[o + 0] = 0;
    img.rgb[o + 1] = 0;
    img.rgb[o + 2] = v;
  }

  return img;
}

static PpmImage MakeDepthImageGray(int w, int h, const std::vector<float>& depth, float maxDepth)
{
  PpmImage img;
  img.width = w;
  img.height = h;
  if (w <= 0 || h <= 0) return img;
  const int n = w * h;
  img.rgb.assign(static_cast<std::size_t>(n) * 3u, 0);

  if (maxDepth <= 0.0f) return img;

  for (int i = 0; i < n; ++i) {
    if (i >= static_cast<int>(depth.size())) break;
    const float d = depth[static_cast<std::size_t>(i)];
    if (d <= 0.0f) continue;
    float t = d / maxDepth;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const std::uint8_t v = static_cast<std::uint8_t>(std::lround(t * 255.0f));
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    img.rgb[o + 0] = v;
    img.rgb[o + 1] = v;
    img.rgb[o + 2] = v;
  }

  return img;
}

static PpmImage MakeHeightImageGray(int w, int h, const std::vector<float>& heights)
{
  PpmImage img;
  img.width = w;
  img.height = h;
  if (w <= 0 || h <= 0) return img;
  const int n = w * h;
  img.rgb.assign(static_cast<std::size_t>(n) * 3u, 0);

  if (static_cast<int>(heights.size()) != n) return img;

  float minH = heights[0];
  float maxH = heights[0];
  for (int i = 1; i < n; ++i) {
    minH = std::min(minH, heights[static_cast<std::size_t>(i)]);
    maxH = std::max(maxH, heights[static_cast<std::size_t>(i)]);
  }

  const float denom = (maxH > minH) ? (maxH - minH) : 1.0f;

  for (int i = 0; i < n; ++i) {
    float t = (heights[static_cast<std::size_t>(i)] - minH) / denom;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const std::uint8_t v = static_cast<std::uint8_t>(std::lround(t * 255.0f));
    const std::size_t o = static_cast<std::size_t>(i) * 3u;
    img.rgb[o + 0] = v;
    img.rgb[o + 1] = v;
    img.rgb[o + 2] = v;
  }

  return img;
}

static void BlendPixel(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, std::uint8_t or_, std::uint8_t og, std::uint8_t ob)
{
  // Blend 2/3 overlay color, 1/3 original.
  r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(or_) * 2) / 3);
  g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(og) * 2) / 3);
  b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(ob) * 2) / 3);
}

static PpmImage MakeAnnotatedOverlay(const World& world, const std::vector<std::uint8_t>& mask, int scale)
{
  PpmImage base = RenderPpmLayer(world, ExportLayer::Overlay);
  if (scale > 1) base = ScaleNearest(base, scale);
  const int w = world.width();
  const int h = world.height();

  for (int ty = 0; ty < h; ++ty) {
    for (int tx = 0; tx < w; ++tx) {
      const std::size_t idx = static_cast<std::size_t>(ty) * static_cast<std::size_t>(w) + static_cast<std::size_t>(tx);
      if (idx >= mask.size()) continue;
      if (mask[idx] == 0) continue;

      for (int py = 0; py < std::max(1, scale); ++py) {
        for (int px = 0; px < std::max(1, scale); ++px) {
          const int x = tx * scale + px;
          const int y = ty * scale + py;
          if (x < 0 || y < 0 || x >= base.width || y >= base.height) continue;
          const std::size_t o = (static_cast<std::size_t>(y) * static_cast<std::size_t>(base.width) + static_cast<std::size_t>(x)) * 3u;
          if (o + 2 >= base.rgb.size()) continue;
          BlendPixel(base.rgb[o + 0], base.rgb[o + 1], base.rgb[o + 2], 0, 0, 255);
        }
      }
    }
  }

  return base;
}

static bool WriteGeoJsonComponents(const std::string& path,
                                  const ThresholdComponents& comps,
                                  const std::vector<LabeledGeometry>& geoms,
                                  const std::string& name,
                                  std::string& outError)
{
  if (!EnsureParentDir(path)) {
    outError = "failed to create parent directories";
    return false;
  }

  std::ofstream of(path);
  if (!of) {
    outError = "failed to open output file";
    return false;
  }

  of << "{\n";
  of << "  \"type\": \"FeatureCollection\",\n";
  of << "  \"name\": \"" << JsonEscape(name) << "\",\n";
  of << "  \"features\": [\n";

  bool first = true;
  for (const LabeledGeometry& g : geoms) {
    const int label = g.label;
    if (label <= 0) continue;
    if (label > static_cast<int>(comps.components.size())) continue;
    const ThresholdComponent& c = comps.components[static_cast<std::size_t>(label - 1)];
    if (c.area <= 0) continue;

    const double cx = c.sumX / static_cast<double>(c.area);
    const double cy = c.sumY / static_cast<double>(c.area);

    if (!first) of << ",\n";
    first = false;

    of << "    {\n";
    of << "      \"type\": \"Feature\",\n";
    of << "      \"properties\": {\n";
    of << "        \"label\": " << label << ",\n";
    of << "        \"area_tiles\": " << c.area << ",\n";
    of << "        \"max_value\": " << c.maxValue << ",\n";
    of << "        \"sum_value\": " << c.sumValue << ",\n";
    of << "        \"centroid\": [" << cx << ", " << cy << "],\n";
    of << "        \"bbox\": [" << c.minX << ", " << c.minY << ", " << c.maxX << ", " << c.maxY << "]\n";
    of << "      },\n";
    of << "      \"geometry\": ";
    WriteGeoJsonGeometry(of, g.geom);
    of << "\n";
    of << "    }";
  }

  of << "\n  ]\n";
  of << "}\n";

  return true;
}

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool ParseMode(const std::string& s, bool& outSea, bool& outDep)
{
  const std::string m = ToLower(s);
  if (m == "sea") {
    outSea = true;
    outDep = false;
    return true;
  }
  if (m == "depressions" || m == "ponding" || m == "dep") {
    outSea = false;
    outDep = true;
    return true;
  }
  if (m == "both" || m == "all") {
    outSea = true;
    outDep = true;
    return true;
  }
  return false;
}

static bool ParseApply(const std::string& s, bool& applySea, bool& applyDep)
{
  const std::string m = ToLower(s);
  if (m == "none" || m == "0") {
    applySea = false;
    applyDep = false;
    return true;
  }
  if (m == "sea") {
    applySea = true;
    applyDep = false;
    return true;
  }
  if (m == "depressions" || m == "ponding" || m == "dep") {
    applySea = false;
    applyDep = true;
    return true;
  }
  if (m == "both" || m == "all") {
    applySea = true;
    applyDep = true;
    return true;
  }
  return false;
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;
  int days = 0;

  bool doSea = true;
  bool doDep = true;

  bool seaLevelProvided = false;
  float seaLevel = 0.0f;
  bool seaConnectEdge = true;
  bool sea8 = false;

  float depEps = 0.0f;
  float depMinDepth = 0.001f;
  bool dep8 = false;

  int scale = 1;

  std::string outSeaMask;
  std::string outSeaDepth;
  std::string outSeaAnnotate;
  std::string outDepFilled;
  std::string outDepDepth;
  std::string outDepAnnotate;
  std::string outSeaGeoJson;
  std::string outDepGeoJson;
  std::string outJson;

  bool applySea = false;
  bool applyDep = false;
  bool bulldozeWater = true;
  std::string outSave;

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
        std::cerr << "--load expects a path\n";
        return 2;
      }
    } else if (arg == "--seed") {
      std::string v;
      if (!requireValue(i, v) || !ParseU64(v, &seed)) {
        std::cerr << "--seed expects a u64\n";
        return 2;
      }
    } else if (arg == "--size") {
      std::string v;
      if (!requireValue(i, v) || !ParseSize(v, &w, &h)) {
        std::cerr << "--size expects WxH\n";
        return 2;
      }
    } else if (arg == "--days") {
      std::string v;
      if (!requireValue(i, v) || !ParseI32(v, &days) || days < 0) {
        std::cerr << "--days expects a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--mode") {
      std::string v;
      if (!requireValue(i, v) || !ParseMode(v, doSea, doDep)) {
        std::cerr << "--mode expects: sea|depressions|both\n";
        return 2;
      }
    } else if (arg == "--sea-level") {
      std::string v;
      if (!requireValue(i, v) || !ParseF32(v, &seaLevel)) {
        std::cerr << "--sea-level expects a float\n";
        return 2;
      }
      seaLevelProvided = true;
    } else if (arg == "--sea-connect-edge") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &seaConnectEdge)) {
        std::cerr << "--sea-connect-edge expects 0|1\n";
        return 2;
      }
    } else if (arg == "--sea-8conn") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &sea8)) {
        std::cerr << "--sea-8conn expects 0|1\n";
        return 2;
      }
    } else if (arg == "--dep-eps") {
      std::string v;
      if (!requireValue(i, v) || !ParseF32(v, &depEps)) {
        std::cerr << "--dep-eps expects a float\n";
        return 2;
      }
    } else if (arg == "--dep-min-depth") {
      std::string v;
      if (!requireValue(i, v) || !ParseF32(v, &depMinDepth) || depMinDepth < 0.0f) {
        std::cerr << "--dep-min-depth expects a non-negative float\n";
        return 2;
      }
    } else if (arg == "--dep-8conn") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &dep8)) {
        std::cerr << "--dep-8conn expects 0|1\n";
        return 2;
      }
    } else if (arg == "--scale") {
      std::string v;
      if (!requireValue(i, v) || !ParseI32(v, &scale) || scale <= 0) {
        std::cerr << "--scale expects a positive integer\n";
        return 2;
      }
    } else if (arg == "--sea-mask") {
      if (!requireValue(i, outSeaMask)) {
        std::cerr << "--sea-mask expects a path\n";
        return 2;
      }
    } else if (arg == "--sea-depth") {
      if (!requireValue(i, outSeaDepth)) {
        std::cerr << "--sea-depth expects a path\n";
        return 2;
      }
    } else if (arg == "--sea-annotate") {
      if (!requireValue(i, outSeaAnnotate)) {
        std::cerr << "--sea-annotate expects a path\n";
        return 2;
      }
    } else if (arg == "--dep-filled") {
      if (!requireValue(i, outDepFilled)) {
        std::cerr << "--dep-filled expects a path\n";
        return 2;
      }
    } else if (arg == "--dep-depth") {
      if (!requireValue(i, outDepDepth)) {
        std::cerr << "--dep-depth expects a path\n";
        return 2;
      }
    } else if (arg == "--dep-annotate") {
      if (!requireValue(i, outDepAnnotate)) {
        std::cerr << "--dep-annotate expects a path\n";
        return 2;
      }
    } else if (arg == "--sea-geojson") {
      if (!requireValue(i, outSeaGeoJson)) {
        std::cerr << "--sea-geojson expects a path\n";
        return 2;
      }
    } else if (arg == "--dep-geojson") {
      if (!requireValue(i, outDepGeoJson)) {
        std::cerr << "--dep-geojson expects a path\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, outJson)) {
        std::cerr << "--json expects a path\n";
        return 2;
      }
    } else if (arg == "--apply") {
      std::string v;
      if (!requireValue(i, v) || !ParseApply(v, applySea, applyDep)) {
        std::cerr << "--apply expects: none|sea|depressions|both\n";
        return 2;
      }
    } else if (arg == "--bulldoze-water") {
      std::string v;
      if (!requireValue(i, v) || !ParseBool01(v, &bulldozeWater)) {
        std::cerr << "--bulldoze-water expects 0|1\n";
        return 2;
      }
    } else if (arg == "--save") {
      if (!requireValue(i, outSave)) {
        std::cerr << "--save expects a path\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      std::cerr << "Use --help for usage.\n";
      return 2;
    }
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load: " << loadPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "loaded " << loadPath << " (" << world.width() << "x" << world.height() << ")\n";
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
    std::cout << "generated world seed=" << seed << " size=" << w << "x" << h << "\n";
  }

  if (!seaLevelProvided) {
    seaLevel = procCfg.waterLevel;
  }

  if (days > 0) {
    Simulator sim(simCfg);
    for (int d = 0; d < days; ++d) sim.stepOnce(world);
    std::cout << "simulated days=" << days << " -> day=" << world.stats().day << " pop=" << world.stats().population
              << " money=" << world.stats().money << "\n";
  } else {
    // Ensure derived stats are present for reporting.
    Simulator sim(simCfg);
    sim.refreshDerivedStats(world);
  }

  const int W = world.width();
  const int H = world.height();
  const std::vector<float> heights = ExtractHeights(world);

  SeaFloodResult seaRes;
  ThresholdComponents seaComps;
  std::vector<LabeledGeometry> seaGeoms;

  DepressionFillResult depRes;
  ThresholdComponents depComps;
  std::vector<LabeledGeometry> depGeoms;

  if (doSea) {
    SeaFloodConfig sCfg;
    sCfg.requireEdgeConnection = seaConnectEdge;
    sCfg.eightConnected = sea8;

    seaRes = ComputeSeaLevelFlood(heights, W, H, seaLevel, sCfg);
    seaComps = LabelComponentsAboveThreshold(seaRes.depth, W, H, 0.0f, sea8);

    std::cout << "sea flood: level=" << seaLevel << " flooded_tiles=" << seaRes.floodedCells << " max_depth=" << seaRes.maxDepth
              << " components=" << seaComps.components.size() << "\n";

    if (!outSeaMask.empty()) {
      if (!EnsureParentDir(outSeaMask)) {
        std::cerr << "Failed to create parent dirs for: " << outSeaMask << "\n";
        return 2;
      }
      std::string err;
      PpmImage img = MakeMaskImage(W, H, seaRes.flooded, 0, 0, 255);
      if (scale > 1) img = ScaleNearest(img, scale);
      if (!WriteImageAuto(outSeaMask, img, err)) {
        std::cerr << "Failed to write sea mask: " << err << "\n";
        return 2;
      }
      std::cout << "wrote sea mask -> " << outSeaMask << "\n";
    }

    if (!outSeaDepth.empty()) {
      if (!EnsureParentDir(outSeaDepth)) {
        std::cerr << "Failed to create parent dirs for: " << outSeaDepth << "\n";
        return 2;
      }
      std::string err;
      PpmImage img = MakeDepthImageBlue(W, H, seaRes.depth, seaRes.maxDepth);
      if (scale > 1) img = ScaleNearest(img, scale);
      if (!WriteImageAuto(outSeaDepth, img, err)) {
        std::cerr << "Failed to write sea depth: " << err << "\n";
        return 2;
      }
      std::cout << "wrote sea depth -> " << outSeaDepth << "\n";
    }

    if (!outSeaAnnotate.empty()) {
      if (!EnsureParentDir(outSeaAnnotate)) {
        std::cerr << "Failed to create parent dirs for: " << outSeaAnnotate << "\n";
        return 2;
      }
      std::string err;
      PpmImage img = MakeAnnotatedOverlay(world, seaRes.flooded, scale);
      if (!WriteImageAuto(outSeaAnnotate, img, err)) {
        std::cerr << "Failed to write sea annotate: " << err << "\n";
        return 2;
      }
      std::cout << "wrote sea annotate -> " << outSeaAnnotate << "\n";
    }

    if (!outSeaGeoJson.empty()) {
      std::vector<LabeledGeometry> geoms;
      VectorizeStats stats;
      std::string vecErr;
      if (!VectorizeLabelGridToPolygons(seaComps.labels, W, H, 0, geoms, &stats, &vecErr)) {
        std::cerr << "Vectorize failed (sea): " << vecErr << "\n";
        return 2;
      }
      seaGeoms = geoms;

      std::string err;
      if (!WriteGeoJsonComponents(outSeaGeoJson, seaComps, seaGeoms, "sea_flood", err)) {
        std::cerr << "Failed to write sea geojson: " << err << "\n";
        return 2;
      }
      std::cout << "wrote sea geojson -> " << outSeaGeoJson << "\n";
    }
  }

  if (doDep) {
    DepressionFillConfig dCfg;
    dCfg.includeEdges = true;
    dCfg.epsilon = depEps;

    const std::vector<std::uint8_t> drainMask = BuildWaterDrainMask(world);
    depRes = FillDepressionsPriorityFlood(heights, W, H, &drainMask, dCfg);

    depComps = LabelComponentsAboveThreshold(depRes.depth, W, H, depMinDepth, dep8);

    std::cout << "depressions: filled_tiles=" << depRes.filledCells << " max_depth=" << depRes.maxDepth
              << " volume=" << depRes.volume << " components(>" << depMinDepth << ")=" << depComps.components.size() << "\n";

    if (!outDepFilled.empty()) {
      if (!EnsureParentDir(outDepFilled)) {
        std::cerr << "Failed to create parent dirs for: " << outDepFilled << "\n";
        return 2;
      }
      std::string err;
      PpmImage img = MakeHeightImageGray(W, H, depRes.filled);
      if (scale > 1) img = ScaleNearest(img, scale);
      if (!WriteImageAuto(outDepFilled, img, err)) {
        std::cerr << "Failed to write dep filled: " << err << "\n";
        return 2;
      }
      std::cout << "wrote dep filled -> " << outDepFilled << "\n";
    }

    if (!outDepDepth.empty()) {
      if (!EnsureParentDir(outDepDepth)) {
        std::cerr << "Failed to create parent dirs for: " << outDepDepth << "\n";
        return 2;
      }
      std::string err;
      PpmImage img = MakeDepthImageGray(W, H, depRes.depth, std::max(0.0f, depRes.maxDepth));
      if (scale > 1) img = ScaleNearest(img, scale);
      if (!WriteImageAuto(outDepDepth, img, err)) {
        std::cerr << "Failed to write dep depth: " << err << "\n";
        return 2;
      }
      std::cout << "wrote dep depth -> " << outDepDepth << "\n";
    }

    if (!outDepAnnotate.empty()) {
      if (!EnsureParentDir(outDepAnnotate)) {
        std::cerr << "Failed to create parent dirs for: " << outDepAnnotate << "\n";
        return 2;
      }
      // Build a binary mask from depComps (labels>0) so annotation matches the thresholded component view.
      std::vector<std::uint8_t> m;
      m.assign(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), 0);
      for (int i = 0; i < W * H && i < static_cast<int>(depComps.labels.size()); ++i) {
        if (depComps.labels[static_cast<std::size_t>(i)] > 0) m[static_cast<std::size_t>(i)] = 1;
      }
      std::string err;
      PpmImage img = MakeAnnotatedOverlay(world, m, scale);
      if (!WriteImageAuto(outDepAnnotate, img, err)) {
        std::cerr << "Failed to write dep annotate: " << err << "\n";
        return 2;
      }
      std::cout << "wrote dep annotate -> " << outDepAnnotate << "\n";
    }

    if (!outDepGeoJson.empty()) {
      std::vector<LabeledGeometry> geoms;
      VectorizeStats stats;
      std::string vecErr;
      if (!VectorizeLabelGridToPolygons(depComps.labels, W, H, 0, geoms, &stats, &vecErr)) {
        std::cerr << "Vectorize failed (dep): " << vecErr << "\n";
        return 2;
      }
      depGeoms = geoms;

      std::string err;
      if (!WriteGeoJsonComponents(outDepGeoJson, depComps, depGeoms, "depressions", err)) {
        std::cerr << "Failed to write dep geojson: " << err << "\n";
        return 2;
      }
      std::cout << "wrote dep geojson -> " << outDepGeoJson << "\n";
    }
  }

  if (!outJson.empty()) {
    if (!EnsureParentDir(outJson)) {
      std::cerr << "Failed to create parent dirs for: " << outJson << "\n";
      return 2;
    }
    std::ofstream of(outJson);
    if (!of) {
      std::cerr << "Failed to open json output: " << outJson << "\n";
      return 2;
    }

    of << "{\n";
    of << "  \"world\": {\n";
    of << "    \"width\": " << W << ",\n";
    of << "    \"height\": " << H << ",\n";
    of << "    \"day\": " << world.stats().day << ",\n";
    of << "    \"population\": " << world.stats().population << ",\n";
    of << "    \"money\": " << world.stats().money << "\n";
    of << "  },\n";

    of << "  \"sea\": {\n";
    of << "    \"enabled\": " << (doSea ? "true" : "false") << ",\n";
    of << "    \"sea_level\": " << seaLevel << ",\n";
    of << "    \"require_edge_connection\": " << (seaConnectEdge ? "true" : "false") << ",\n";
    of << "    \"eight_connected\": " << (sea8 ? "true" : "false") << ",\n";
    of << "    \"flooded_tiles\": " << seaRes.floodedCells << ",\n";
    of << "    \"max_depth\": " << seaRes.maxDepth << ",\n";
    of << "    \"components\": " << seaComps.components.size() << "\n";
    of << "  },\n";

    of << "  \"depressions\": {\n";
    of << "    \"enabled\": " << (doDep ? "true" : "false") << ",\n";
    of << "    \"epsilon\": " << depEps << ",\n";
    of << "    \"min_depth\": " << depMinDepth << ",\n";
    of << "    \"filled_tiles\": " << depRes.filledCells << ",\n";
    of << "    \"max_depth\": " << depRes.maxDepth << ",\n";
    of << "    \"volume\": " << depRes.volume << ",\n";
    of << "    \"components\": " << depComps.components.size() << "\n";
    of << "  }\n";

    of << "}\n";

    std::cout << "wrote json -> " << outJson << "\n";
  }

  if (!outSave.empty()) {
    if (!applySea && !applyDep) {
      std::cerr << "--save specified but --apply is none; refusing to write an unmodified save.\n";
      return 2;
    }

    if (!EnsureParentDir(outSave)) {
      std::cerr << "Failed to create parent dirs for: " << outSave << "\n";
      return 2;
    }

    World w2 = world;

    auto applyMaskAsWater = [&](const std::vector<std::uint8_t>& mask) {
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x);
          if (idx >= mask.size()) continue;
          if (mask[idx] == 0) continue;
          Tile& t = w2.at(x, y);
          t.terrain = Terrain::Water;
          if (bulldozeWater && t.overlay != Overlay::Road) {
            w2.bulldoze(x, y);
          }
        }
      }
    };

    if (applySea) {
      applyMaskAsWater(seaRes.flooded);
    }

    if (applyDep) {
      // Apply the *thresholded* depression mask (labels>0).
      std::vector<std::uint8_t> m;
      m.assign(static_cast<std::size_t>(W) * static_cast<std::size_t>(H), 0);
      for (int i = 0; i < W * H && i < static_cast<int>(depComps.labels.size()); ++i) {
        if (depComps.labels[static_cast<std::size_t>(i)] > 0) m[static_cast<std::size_t>(i)] = 1;
      }
      applyMaskAsWater(m);
    }

    Simulator sim(simCfg);
    sim.refreshDerivedStats(w2);

    std::string err;
    if (!SaveWorldBinary(w2, procCfg, simCfg, outSave, err)) {
      std::cerr << "Failed to write save: " << outSave << "\n";
      std::cerr << err << "\n";
      return 2;
    }

    std::cout << "wrote save -> " << outSave << "\n";
  }

  return 0;
}
