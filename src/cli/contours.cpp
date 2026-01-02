#include "isocity/Contours.hpp"
#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cerrno>
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

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t x = s.find('x');
  if (x == std::string::npos) return false;
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, x), &w)) return false;
  if (!ParseI32(s.substr(x + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
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

bool ParseLevelsCsv(const std::string& s, std::vector<double>& out)
{
  out.clear();
  for (const std::string& tok : SplitCsv(s)) {
    double v = 0.0;
    if (!ParseF64(tok, &v)) return false;
    out.push_back(v);
  }
  return !out.empty();
}

struct Options {
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  // Contour levels:
  std::vector<double> levels; // explicit
  double interval = 0.05;     // used when levels empty and count==0
  int count = 0;              // if >0, overrides interval
  double minLevel = std::numeric_limits<double>::quiet_NaN();
  double maxLevel = std::numeric_limits<double>::quiet_NaN();

  double heightScale = 1.0;

  // Stitching/simplification.
  double quantize = 1e-6;
  int decider = 1;
  double simplify = 0.0;
  int minPoints = 2;

  // Outputs.
  std::string outGeoJson;
  std::string outSvg;
  std::string outPpm;

  // Terrain analysis outputs.
  std::string outHillshade;
  std::string outSlope;

  // Raster params.
  int scale = 4;

  // SVG params.
  int svgScale = 16;
  int svgLabels = 0;

  // Hillshade params (azimuth in degrees where 0=+X (east), 90=+Y (south)).
  double sunAzimuthDeg = 315.0;
  double sunAltitudeDeg = 45.0;

  // Slope visualization scaling (value = clamp(slope * slopeScale, 0..1)).
  double slopeScale = 2.0;
};

void PrintHelp()
{
  std::cout
      << "proc_isocity_contours (headless topographic contour export)\n\n"
      << "Extracts contour polylines from the World's height field using marching squares, and\n"
      << "exports them as GeoJSON and/or SVG. Optionally writes raster debug images.\n\n"
      << "Usage:\n"
      << "  proc_isocity_contours [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                      [--height-scale <F>]\n"
      << "                      [--levels <csv> | --interval <F> | --count <N>]\n"
      << "                      [--min-level <F>] [--max-level <F>]\n"
      << "                      [--simplify <F>] [--min-points <N>]\n"
      << "                      [--geojson <out.geojson>] [--svg <out.svg>]\n"
      << "                      [--ppm <out.png|out.ppm> --scale <N>]\n"
      << "                      [--hillshade <out.png|out.ppm>] [--slope <out.png|out.ppm>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>     Load a save (overrides --seed/--size).\n"
      << "  --seed <u64>          Seed for ProcGen (default: 1).\n"
      << "  --size <WxH>          World size (default: 96x96).\n"
      << "  --height-scale <F>    Multiply Tile::height by this factor before contouring (default: 1).\n\n"
      << "Contour levels:\n"
      << "  --levels <csv>        Explicit iso-level list, e.g. 0.1,0.2,0.35\n"
      << "  --interval <F>        Generate regularly spaced levels (default: 0.05).\n"
      << "  --count <N>           Generate N evenly spaced levels between min/max (overrides interval).\n"
      << "  --min-level <F>       Clamp/generate levels starting at this minimum.\n"
      << "  --max-level <F>       Clamp/generate levels ending at this maximum.\n\n"
      << "Polyline controls:\n"
      << "  --quantize <F>        Endpoint quantization for stitching (default: 1e-6).\n"
      << "  --decider <0|1>       Resolve saddle cases with a deterministic decider (default: 1).\n"
      << "  --simplify <F>        Douglas-Peucker tolerance in tile units (default: 0).\n"
      << "  --min-points <N>      Drop polylines with <N points (default: 2).\n\n"
      << "Outputs:\n"
      << "  --geojson <path>      GeoJSON FeatureCollection of LineString contour features.\n"
      << "  --svg <path>          SVG rendering of contour lines.\n"
      << "  --svg-scale <N>       Pixels per tile for SVG (default: 16).\n"
      << "  --svg-labels <0|1>    If 1, label each contour with its level (default: 0).\n"
      << "  --ppm <path>          Raster preview with contours over the height layer.\n"
      << "  --scale <N>           Raster scale factor (default: 4).\n"
      << "  --hillshade <path>    Raster hillshade image (per tile).\n"
      << "  --slope <path>        Raster slope magnitude image (per tile).\n"
      << "  --sun-azimuth <deg>   Hillshade azimuth (0=+X east, 90=+Y south; default: 315).\n"
      << "  --sun-altitude <deg>  Hillshade altitude above horizon (default: 45).\n"
      << "  --slope-scale <F>     Slope visualization scale (default: 2).\n";
}

void DrawPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

void DrawLine(PpmImage& img, int x0, int y0, int x1, int y1, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  // Bresenham.
  int dx = std::abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  while (true) {
    DrawPixel(img, x0, y0, r, g, b);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void DrawPolyline(PpmImage& img, const std::vector<FPoint>& pts, int scale, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (pts.size() < 2 || scale <= 0) return;
  const int w = img.width;
  const int h = img.height;

  auto ToPix = [&](const FPoint& p, int& outX, int& outY) {
    const long long px = llround(p.x * static_cast<double>(scale));
    const long long py = llround(p.y * static_cast<double>(scale));
    outX = static_cast<int>(std::clamp<long long>(px, 0, std::max(0, w - 1)));
    outY = static_cast<int>(std::clamp<long long>(py, 0, std::max(0, h - 1)));
  };

  for (std::size_t i = 1; i < pts.size(); ++i) {
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    ToPix(pts[i - 1], x0, y0);
    ToPix(pts[i], x1, y1);
    DrawLine(img, x0, y0, x1, y1, r, g, b);
  }
}

PpmImage MakeHillshade(const std::vector<double>& corner, int cornerW, int cornerH, double azDeg, double altDeg)
{
  const int w = cornerW - 1;
  const int h = cornerH - 1;

  PpmImage img;
  img.width = w;
  img.height = h;
  img.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u, 0);

  const double az = azDeg * 3.14159265358979323846 / 180.0;
  const double alt = altDeg * 3.14159265358979323846 / 180.0;
  const double sunX = std::cos(alt) * std::cos(az);
  const double sunY = std::cos(alt) * std::sin(az);
  const double sunZ = std::sin(alt);

  auto At = [&](int x, int y) -> double {
    return corner[static_cast<std::size_t>(y) * static_cast<std::size_t>(cornerW) + static_cast<std::size_t>(x)];
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const double v00 = At(x, y);
      const double v10 = At(x + 1, y);
      const double v01 = At(x, y + 1);
      const double v11 = At(x + 1, y + 1);

      // Simple gradient over the cell (tile unit = 1).
      const double dx = 0.5 * ((v10 + v11) - (v00 + v01));
      const double dy = 0.5 * ((v01 + v11) - (v00 + v10));

      // Surface normal (in the tool's coordinate system: +x right, +y down).
      double nx = -dx;
      double ny = -dy;
      double nz = 1.0;
      const double invLen = 1.0 / std::sqrt(nx * nx + ny * ny + nz * nz);
      nx *= invLen;
      ny *= invLen;
      nz *= invLen;

      double shade = nx * sunX + ny * sunY + nz * sunZ;
      if (shade < 0.0) shade = 0.0;
      if (shade > 1.0) shade = 1.0;
      const std::uint8_t v = static_cast<std::uint8_t>(std::llround(shade * 255.0));

      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
      img.rgb[idx + 0] = v;
      img.rgb[idx + 1] = v;
      img.rgb[idx + 2] = v;
    }
  }

  return img;
}

PpmImage MakeSlope(const std::vector<double>& corner, int cornerW, int cornerH, double slopeScale)
{
  const int w = cornerW - 1;
  const int h = cornerH - 1;

  PpmImage img;
  img.width = w;
  img.height = h;
  img.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u, 0);

  auto At = [&](int x, int y) -> double {
    return corner[static_cast<std::size_t>(y) * static_cast<std::size_t>(cornerW) + static_cast<std::size_t>(x)];
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const double v00 = At(x, y);
      const double v10 = At(x + 1, y);
      const double v01 = At(x, y + 1);
      const double v11 = At(x + 1, y + 1);

      const double dx = 0.5 * ((v10 + v11) - (v00 + v01));
      const double dy = 0.5 * ((v01 + v11) - (v00 + v10));

      const double slope = std::sqrt(dx * dx + dy * dy);
      double v01n = slope * slopeScale;
      if (v01n < 0.0) v01n = 0.0;
      if (v01n > 1.0) v01n = 1.0;
      const std::uint8_t v = static_cast<std::uint8_t>(std::llround(v01n * 255.0));

      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
      img.rgb[idx + 0] = v;
      img.rgb[idx + 1] = v;
      img.rgb[idx + 2] = v;
    }
  }

  return img;
}

void WriteGeoJsonLineCoords(std::ostream& os, const std::vector<FPoint>& pts)
{
  os << "[";
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (i) os << ",";
    os << "[" << pts[i].x << "," << pts[i].y << "]";
  }
  os << "]";
}

bool WriteGeoJson(const std::string& path, const World& world, const std::vector<ContourLevel>& contours, double heightScale)
{
  if (!EnsureParentDir(path)) return false;
  std::ofstream os(path, std::ios::binary);
  if (!os) return false;

  // Deterministic float formatting.
  os.setf(std::ios::fixed);
  os << std::setprecision(6);

  os << "{\n";
  os << "  \"type\": \"FeatureCollection\",\n";
  os << "  \"properties\": {\"w\": " << world.width() << ", \"h\": " << world.height() << ", \"seed\": " << world.seed()
     << ", \"heightScale\": " << heightScale << "},\n";
  os << "  \"features\": [\n";

  bool first = true;
  std::size_t featureId = 0;
  for (const ContourLevel& lvl : contours) {
    for (const ContourPolyline& line : lvl.lines) {
      if (!first) os << ",\n";
      first = false;
      os << "    {\"type\":\"Feature\",\"properties\":{\"id\":" << featureId++
         << ",\"level\":" << lvl.level << ",\"closed\":" << (line.closed ? 1 : 0) << ",\"points\":" << line.pts.size() << "},"
         << "\"geometry\":{\"type\":\"LineString\",\"coordinates\":";
      WriteGeoJsonLineCoords(os, line.pts);
      os << "}}";
    }
  }

  os << "\n  ]\n";
  os << "}\n";
  return true;
}

bool WriteSvg(const std::string& path, const World& world, const std::vector<ContourLevel>& contours, int svgScale, int labels,
              double minLevel, double maxLevel)
{
  if (!EnsureParentDir(path)) return false;
  std::ofstream os(path, std::ios::binary);
  if (!os) return false;

  const int w = world.width();
  const int h = world.height();
  const int pxW = std::max(1, w * svgScale);
  const int pxH = std::max(1, h * svgScale);

  os.setf(std::ios::fixed);
  os << std::setprecision(3);

  os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  os << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << pxW << " " << pxH
     << "\" width=\"" << pxW << "\" height=\"" << pxH << "\">\n";
  os << "  <rect x=\"0\" y=\"0\" width=\"" << pxW << "\" height=\"" << pxH << "\" fill=\"white\"/>\n";

  const double denom = (std::isfinite(maxLevel) && std::isfinite(minLevel) && maxLevel > minLevel) ? (maxLevel - minLevel) : 1.0;

  auto StrokeForLevel = [&](double level) -> int {
    const double t = std::clamp((level - minLevel) / denom, 0.0, 1.0);
    // Darker for higher contours.
    const int g = static_cast<int>(std::llround(230.0 - t * 180.0));
    return std::clamp(g, 20, 240);
  };

  std::size_t id = 0;
  for (const ContourLevel& lvl : contours) {
    const int g = StrokeForLevel(lvl.level);
    for (const ContourPolyline& line : lvl.lines) {
      if (line.pts.size() < 2) continue;

      os << "  <path id=\"c" << id++ << "\" d=\"";
      for (std::size_t i = 0; i < line.pts.size(); ++i) {
        const double x = line.pts[i].x * svgScale;
        const double y = line.pts[i].y * svgScale;
        if (i == 0) {
          os << "M " << x << " " << y;
        } else {
          os << " L " << x << " " << y;
        }
      }
      if (line.closed) os << " Z";
      os << "\" fill=\"none\" stroke=\"rgb(" << g << "," << g << "," << g << ")\" stroke-width=\"1\"/>\n";

      if (labels) {
        const FPoint& p = line.pts.front();
        const double x = p.x * svgScale;
        const double y = p.y * svgScale;
        os << "  <text x=\"" << x + 2.0 << "\" y=\"" << y - 2.0
           << "\" font-size=\"10\" fill=\"black\">" << lvl.level << "</text>\n";
      }
    }
  }

  os << "</svg>\n";
  return true;
}

std::vector<double> BuildGeneratedLevels(const Options& opt, double computedMin, double computedMax)
{
  double lo = computedMin;
  double hi = computedMax;
  if (std::isfinite(opt.minLevel)) lo = opt.minLevel;
  if (std::isfinite(opt.maxLevel)) hi = opt.maxLevel;

  if (!(hi > lo)) {
    // Degenerate range; return nothing.
    return {};
  }

  std::vector<double> out;

  if (opt.count > 0) {
    // N levels evenly distributed between lo..hi, excluding endpoints.
    const double step = (hi - lo) / static_cast<double>(opt.count + 1);
    for (int i = 0; i < opt.count; ++i) {
      out.push_back(lo + step * static_cast<double>(i + 1));
    }
    return out;
  }

  const double interval = (opt.interval > 0.0) ? opt.interval : 0.05;

  // Start at the smallest multiple of interval strictly greater than lo.
  const double start = std::floor(lo / interval) * interval;
  for (double v = start; v <= hi + 1e-12; v += interval) {
    if (v <= lo + 1e-12) continue;
    if (v >= hi - 1e-12) continue;
    // Snap to interval grid to keep deterministic decimal representation.
    const double snapped = std::llround(v / interval) * interval;
    out.push_back(snapped);
  }

  // Dedup and sort.
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end(), [](double a, double b) { return std::abs(a - b) < 1e-12; }), out.end());
  return out;
}

} // namespace

int main(int argc, char** argv)
{
  Options opt;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto Need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << name << " requires a value\n";
        std::exit(2);
      }
      return std::string(argv[++i]);
    };

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--load") {
      opt.loadPath = Need("--load");
    } else if (a == "--seed") {
      if (!ParseU64(Need("--seed"), &opt.seed)) {
        std::cerr << "Invalid --seed\n";
        return 2;
      }
    } else if (a == "--size") {
      const std::string s = Need("--size");
      if (!ParseWxH(s, &opt.w, &opt.h)) {
        std::cerr << "Invalid --size (expected WxH)\n";
        return 2;
      }
    } else if (a == "--height-scale") {
      if (!ParseF64(Need("--height-scale"), &opt.heightScale)) {
        std::cerr << "Invalid --height-scale\n";
        return 2;
      }
    } else if (a == "--levels") {
      if (!ParseLevelsCsv(Need("--levels"), opt.levels)) {
        std::cerr << "Invalid --levels (expected csv floats)\n";
        return 2;
      }
    } else if (a == "--interval") {
      if (!ParseF64(Need("--interval"), &opt.interval) || opt.interval <= 0.0) {
        std::cerr << "Invalid --interval\n";
        return 2;
      }
    } else if (a == "--count") {
      if (!ParseI32(Need("--count"), &opt.count) || opt.count < 0) {
        std::cerr << "Invalid --count\n";
        return 2;
      }
    } else if (a == "--min-level") {
      if (!ParseF64(Need("--min-level"), &opt.minLevel)) {
        std::cerr << "Invalid --min-level\n";
        return 2;
      }
    } else if (a == "--max-level") {
      if (!ParseF64(Need("--max-level"), &opt.maxLevel)) {
        std::cerr << "Invalid --max-level\n";
        return 2;
      }
    } else if (a == "--quantize") {
      if (!ParseF64(Need("--quantize"), &opt.quantize) || opt.quantize <= 0.0) {
        std::cerr << "Invalid --quantize\n";
        return 2;
      }
    } else if (a == "--decider") {
      if (!ParseI32(Need("--decider"), &opt.decider)) {
        std::cerr << "Invalid --decider\n";
        return 2;
      }
    } else if (a == "--simplify") {
      if (!ParseF64(Need("--simplify"), &opt.simplify) || opt.simplify < 0.0) {
        std::cerr << "Invalid --simplify\n";
        return 2;
      }
    } else if (a == "--min-points") {
      if (!ParseI32(Need("--min-points"), &opt.minPoints) || opt.minPoints < 2) {
        std::cerr << "Invalid --min-points\n";
        return 2;
      }
    } else if (a == "--geojson") {
      opt.outGeoJson = Need("--geojson");
    } else if (a == "--svg") {
      opt.outSvg = Need("--svg");
    } else if (a == "--svg-scale") {
      if (!ParseI32(Need("--svg-scale"), &opt.svgScale) || opt.svgScale <= 0) {
        std::cerr << "Invalid --svg-scale\n";
        return 2;
      }
    } else if (a == "--svg-labels") {
      if (!ParseI32(Need("--svg-labels"), &opt.svgLabels)) {
        std::cerr << "Invalid --svg-labels\n";
        return 2;
      }
    } else if (a == "--ppm") {
      opt.outPpm = Need("--ppm");
    } else if (a == "--scale") {
      if (!ParseI32(Need("--scale"), &opt.scale) || opt.scale <= 0) {
        std::cerr << "Invalid --scale\n";
        return 2;
      }
    } else if (a == "--hillshade") {
      opt.outHillshade = Need("--hillshade");
    } else if (a == "--slope") {
      opt.outSlope = Need("--slope");
    } else if (a == "--sun-azimuth") {
      if (!ParseF64(Need("--sun-azimuth"), &opt.sunAzimuthDeg)) {
        std::cerr << "Invalid --sun-azimuth\n";
        return 2;
      }
    } else if (a == "--sun-altitude") {
      if (!ParseF64(Need("--sun-altitude"), &opt.sunAltitudeDeg)) {
        std::cerr << "Invalid --sun-altitude\n";
        return 2;
      }
    } else if (a == "--slope-scale") {
      if (!ParseF64(Need("--slope-scale"), &opt.slopeScale) || opt.slopeScale <= 0.0) {
        std::cerr << "Invalid --slope-scale\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      return 2;
    }
  }

  if (opt.outGeoJson.empty() && opt.outSvg.empty() && opt.outPpm.empty() && opt.outHillshade.empty() && opt.outSlope.empty()) {
    std::cerr << "No outputs requested. Use --geojson/--svg/--ppm/--hillshade/--slope.\n";
    return 2;
  }

  // Load or generate the world.
  World world;
  if (!opt.loadPath.empty()) {
    std::string err;
    ProcGenConfig procCfg;
    SimConfig simCfg;
    if (!LoadWorldBinary(world, procCfg, simCfg, opt.loadPath, err)) {
      std::cerr << "Failed to load save: " << opt.loadPath << "\n" << err << "\n";
      return 2;
    }
  } else {
    ProcGenConfig cfg;
    world = GenerateWorld(opt.w, opt.h, opt.seed, cfg);
  }

  const int w = world.width();
  const int h = world.height();

  // Corner grid.
  const std::vector<double> corner = BuildCornerHeightGrid(world, opt.heightScale);
  const int cornerW = w + 1;
  const int cornerH = h + 1;

  double minV = std::numeric_limits<double>::infinity();
  double maxV = -std::numeric_limits<double>::infinity();
  for (double v : corner) {
    minV = std::min(minV, v);
    maxV = std::max(maxV, v);
  }

  // Hillshade/slope outputs (optional).
  std::string err;
  if (!opt.outHillshade.empty()) {
    PpmImage img = MakeHillshade(corner, cornerW, cornerH, opt.sunAzimuthDeg, opt.sunAltitudeDeg);
    if (opt.scale > 1) img = ScaleNearest(img, opt.scale);
    if (!EnsureParentDir(opt.outHillshade)) {
      std::cerr << "Failed to create output directory for: " << opt.outHillshade << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outHillshade, img, err)) {
      std::cerr << "Failed to write hillshade: " << opt.outHillshade << "\n" << err << "\n";
      return 2;
    }
  }
  if (!opt.outSlope.empty()) {
    PpmImage img = MakeSlope(corner, cornerW, cornerH, opt.slopeScale);
    if (opt.scale > 1) img = ScaleNearest(img, opt.scale);
    if (!EnsureParentDir(opt.outSlope)) {
      std::cerr << "Failed to create output directory for: " << opt.outSlope << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outSlope, img, err)) {
      std::cerr << "Failed to write slope: " << opt.outSlope << "\n" << err << "\n";
      return 2;
    }
  }

  // Contours.
  std::vector<double> levels = opt.levels;
  if (levels.empty()) {
    levels = BuildGeneratedLevels(opt, minV, maxV);
  }
  if (levels.empty() && (opt.outGeoJson.size() || opt.outSvg.size() || opt.outPpm.size())) {
    std::cerr << "No contour levels to extract (height range is degenerate?).\n";
    return 2;
  }

  ContourConfig ccfg;
  ccfg.quantize = opt.quantize;
  ccfg.useAsymptoticDecider = (opt.decider != 0);
  ccfg.simplifyTolerance = opt.simplify;
  ccfg.minPoints = opt.minPoints;

  std::string cErr;
  const std::vector<ContourLevel> contours = ExtractContours(corner, cornerW, cornerH, levels, ccfg, &cErr);
  if (!cErr.empty()) {
    std::cerr << "Contour extraction failed: " << cErr << "\n";
    return 2;
  }

  const double lo = std::isfinite(opt.minLevel) ? opt.minLevel : minV;
  const double hi = std::isfinite(opt.maxLevel) ? opt.maxLevel : maxV;

  if (!opt.outGeoJson.empty()) {
    if (!WriteGeoJson(opt.outGeoJson, world, contours, opt.heightScale)) {
      std::cerr << "Failed to write geojson: " << opt.outGeoJson << "\n";
      return 2;
    }
  }

  if (!opt.outSvg.empty()) {
    if (!WriteSvg(opt.outSvg, world, contours, opt.svgScale, opt.svgLabels, lo, hi)) {
      std::cerr << "Failed to write svg: " << opt.outSvg << "\n";
      return 2;
    }
  }

  if (!opt.outPpm.empty()) {
    // Base image: height layer.
    PpmImage img = RenderPpmLayer(world, ExportLayer::Height);
    if (opt.scale > 1) img = ScaleNearest(img, opt.scale);

    // Draw contours on top.
    for (const ContourLevel& lvl : contours) {
      (void)lvl;
      for (const ContourPolyline& line : lvl.lines) {
        DrawPolyline(img, line.pts, opt.scale, 220, 10, 10);
      }
    }

    if (!EnsureParentDir(opt.outPpm)) {
      std::cerr << "Failed to create output directory for: " << opt.outPpm << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outPpm, img, err)) {
      std::cerr << "Failed to write image: " << opt.outPpm << "\n" << err << "\n";
      return 2;
    }
  }

  return 0;
}
