#include "isocity/Export.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cctype>
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

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool ParseI32(const std::string& s, int* out)
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
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

static bool ParseWxH(const std::string& s, int* outW, int* outH)
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

static bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

static bool ParsePointXY(const std::string& s, Point& out)
{
  const std::size_t pos = s.find(',');
  if (pos == std::string::npos) return false;
  int x = 0;
  int y = 0;
  if (!ParseI32(s.substr(0, pos), &x)) return false;
  if (!ParseI32(s.substr(pos + 1), &y)) return false;
  out = Point{x, y};
  return true;
}

static std::string EscapeJson(const std::string& s)
{
  std::ostringstream oss;
  for (unsigned char uc : s) {
    const char c = static_cast<char>(uc);
    switch (c) {
      case '\\': oss << "\\\\"; break;
      case '"': oss << "\\\""; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default:
        if (uc < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(uc) << std::dec;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
}

static bool EnsureParentDir(const std::string& path)
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

enum class Mode : std::uint8_t {
  Road = 0,
  Land,
  RoadBuild,
  ToEdge,
};

static const char* ModeName(Mode m)
{
  switch (m) {
    case Mode::Road: return "road";
    case Mode::Land: return "land";
    case Mode::RoadBuild: return "roadbuild";
    case Mode::ToEdge: return "to-edge";
    default: return "unknown";
  }
}

static bool ParseMode(const std::string& s, Mode& out)
{
  const std::string k = ToLower(s);
  if (k == "road") {
    out = Mode::Road;
    return true;
  }
  if (k == "land") {
    out = Mode::Land;
    return true;
  }
  if (k == "roadbuild" || k == "build") {
    out = Mode::RoadBuild;
    return true;
  }
  if (k == "to-edge" || k == "edge" || k == "outside") {
    out = Mode::ToEdge;
    return true;
  }
  return false;
}

enum class CostModel : std::uint8_t {
  NewTiles = 0,
  Money,
};

static const char* CostModelName(CostModel m)
{
  switch (m) {
    case CostModel::NewTiles: return "new";
    case CostModel::Money: return "money";
    default: return "unknown";
  }
}

static bool ParseCostModel(const std::string& s, CostModel& out)
{
  const std::string k = ToLower(s);
  if (k == "new" || k == "newtiles" || k == "tiles") {
    out = CostModel::NewTiles;
    return true;
  }
  if (k == "money" || k == "cost") {
    out = CostModel::Money;
    return true;
  }
  return false;
}

struct Options {
  // World input.
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  // Query.
  Mode mode = Mode::Road;
  bool haveStart = false;
  bool haveGoal = false;
  Point start{0, 0};
  Point goal{0, 0};

  // Road-build planner knobs.
  bool allowBridges = false;
  CostModel costModel = CostModel::NewTiles;
  int targetLevel = 1;

  // Outputs.
  std::string outJson;
  std::string outCsv;
  std::string outPpm;
  std::string outIso;
  std::string outSave; // roadbuild only

  ExportLayer baseLayer = ExportLayer::Overlay;
  int ppmScale = 4;

  IsoOverviewConfig isoCfg{};

  bool quiet = false;
};

static void PrintHelp()
{
  std::cout
      << "proc_isocity_path (headless pathfinding / routing tool)\n\n"
      << "Finds shortest paths over the world's grids for debugging/analysis and emits optional\n"
      << "artifacts (JSON/CSV/PPM/PNG, and an isometric overview with the path overlaid).\n\n"
      << "World input:\n"
      << "  --load <save.bin>              Load a save. If omitted, a world is generated.\n"
      << "  --seed <u64>                   Seed used when generating a world (default: 1).\n"
      << "  --size <WxH>                   Size used when generating a world (default: 96x96).\n\n"
      << "Path query:\n"
      << "  --mode <road|land|roadbuild|to-edge>  Default: road\n"
      << "  --start <x,y>                  Start tile coordinate (required).\n"
      << "  --goal <x,y>                   Goal tile coordinate (required except mode=to-edge).\n\n"
      << "Road-build mode options (mode=roadbuild):\n"
      << "  --allow-bridges <0|1>           Allow roads across water (bridges). Default: 0\n"
      << "  --cost-model <new|money>        Optimize for new tiles vs money cost. Default: new\n"
      << "  --target-level <1..3>           Planned road level (Street/Avenue/Highway). Default: 1\n"
      << "  --write-save <out.bin>          Write a save with the planned road built (no cost charged).\n\n"
      << "Outputs:\n"
      << "  --json <out.json>               Write a JSON report (includes the full tile path).\n"
      << "  --csv <out.csv>                 Write the tile path as CSV (x,y).\n"
      << "  --ppm <out.ppm|out.png>         Write a one-pixel-per-tile debug image with the path highlighted.\n"
      << "  --layer <name>                  Base render layer for --ppm/--iso. Default: overlay\n"
      << "                                (terrain, overlay, height, district, landvalue, traffic, goods_traffic, goods_fill)\n"
      << "  --ppm-scale <N>                 Upscale the --ppm image (nearest-neighbor). Default: 4\n"
      << "  --iso <out.ppm|out.png>         Write an isometric overview with the path overlaid.\n"
      << "  --iso-tile <WxH>                Isometric tile pixel size. Default: 16x8\n"
      << "  --iso-height <N>                Isometric height scale in pixels (0 disables). Default: 14\n"
      << "  --iso-margin <N>                Isometric image margin. Default: 8\n"
      << "  --iso-grid <0|1>                Draw iso tile grid lines. Default: 0\n"
      << "  --iso-cliffs <0|1>              Draw iso cliffs. Default: 1\n"
      << "  --quiet                         Suppress stdout summary (errors still print).\n"
      << "  -h, --help                      Show this help.\n\n"
      << "Exit codes:\n"
      << "  0  path found\n"
      << "  1  no path found\n"
      << "  2  error\n";
}

static int CountTurns(const std::vector<Point>& path)
{
  if (path.size() < 3) return 0;
  auto dir = [](Point a, Point b) -> Point {
    const int dx = (b.x > a.x) ? 1 : (b.x < a.x ? -1 : 0);
    const int dy = (b.y > a.y) ? 1 : (b.y < a.y ? -1 : 0);
    return Point{dx, dy};
  };

  int turns = 0;
  Point prevDir = dir(path[0], path[1]);
  for (std::size_t i = 2; i < path.size(); ++i) {
    const Point d = dir(path[i - 1], path[i]);
    if (d.x != prevDir.x || d.y != prevDir.y) ++turns;
    prevDir = d;
  }
  return turns;
}

static int CountNewRoadTiles(const World& world, const std::vector<Point>& path)
{
  int n = 0;
  for (const Point& p : path) {
    if (!world.inBounds(p.x, p.y)) continue;
    if (world.at(p.x, p.y).overlay != Overlay::Road) ++n;
  }
  return n;
}

static inline bool InBoundsImg(const PpmImage& img, int x, int y)
{
  return x >= 0 && y >= 0 && x < img.width && y < img.height;
}

static void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (!InBoundsImg(img, x, y)) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) +
                           static_cast<std::size_t>(x)) *
                          3;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

static void DrawDot(PpmImage& img, int cx, int cy, int radius, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      SetPixel(img, cx + dx, cy + dy, r, g, b);
    }
  }
}

static void DrawLine(PpmImage& img, int x0, int y0, int x1, int y1, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  // Bresenham.
  const int dx = std::abs(x1 - x0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  int x = x0;
  int y = y0;
  while (true) {
    SetPixel(img, x, y, r, g, b);
    if (x == x1 && y == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

static bool WritePathCsv(const std::string& path, const std::vector<Point>& tiles, std::string& err)
{
  if (!EnsureParentDir(path)) {
    err = "failed to create output directory for " + path;
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "failed to open " + path;
    return false;
  }
  f << "x,y\n";
  for (const Point& p : tiles) {
    f << p.x << "," << p.y << "\n";
  }
  return static_cast<bool>(f);
}

static bool WriteJsonReport(const std::string& outPath,
                            const Options& opt,
                            const World& world,
                            bool found,
                            int steps,
                            int turns,
                            int primaryCost,
                            int newRoadTiles,
                            const std::vector<Point>& tiles,
                            std::string& err)
{
  if (!EnsureParentDir(outPath)) {
    err = "failed to create output directory for " + outPath;
    return false;
  }

  std::ofstream f(outPath, std::ios::binary);
  if (!f) {
    err = "failed to open " + outPath;
    return false;
  }

  f << "{\n";
  f << "  \"mode\": \"" << ModeName(opt.mode) << "\",\n";
  if (!opt.loadPath.empty()) {
    f << "  \"load\": \"" << EscapeJson(opt.loadPath) << "\",\n";
  } else {
    f << "  \"seed\": " << opt.seed << ",\n";
    f << "  \"size\": \"" << opt.w << "x" << opt.h << "\",\n";
  }
  f << "  \"worldWidth\": " << world.width() << ",\n";
  f << "  \"worldHeight\": " << world.height() << ",\n";
  f << "  \"start\": {\"x\": " << opt.start.x << ", \"y\": " << opt.start.y << "},\n";
  if (opt.mode != Mode::ToEdge) {
    f << "  \"goal\": {\"x\": " << opt.goal.x << ", \"y\": " << opt.goal.y << "},\n";
  }
  if (found && !tiles.empty()) {
    const Point end = tiles.back();
    f << "  \"end\": {\"x\": " << end.x << ", \"y\": " << end.y << "},\n";
  }
  f << "  \"found\": " << (found ? "true" : "false") << ",\n";
  f << "  \"steps\": " << steps << ",\n";
  f << "  \"turns\": " << turns << ",\n";

  if (opt.mode == Mode::RoadBuild) {
    f << "  \"roadbuild\": {\n";
    f << "    \"allowBridges\": " << (opt.allowBridges ? "true" : "false") << ",\n";
    f << "    \"costModel\": \"" << CostModelName(opt.costModel) << "\",\n";
    f << "    \"targetLevel\": " << opt.targetLevel << ",\n";
    f << "    \"primaryCost\": " << primaryCost << ",\n";
    f << "    \"newRoadTiles\": " << newRoadTiles << "\n";
    f << "  },\n";
  }

  f << "  \"pathLength\": " << static_cast<int>(tiles.size()) << ",\n";
  f << "  \"path\": [";
  for (std::size_t i = 0; i < tiles.size(); ++i) {
    const Point p = tiles[i];
    if (i) f << ",";
    f << "[" << p.x << "," << p.y << "]";
  }
  f << "]\n";
  f << "}\n";

  return static_cast<bool>(f);
}

static void ApplyRoadPath(World& world, const std::vector<Point>& path, int targetLevel)
{
  const int level = std::clamp(targetLevel, 1, 3);
  for (const auto& p : path) {
    if (!world.inBounds(p.x, p.y)) continue;
    world.setRoad(p.x, p.y);
    world.at(p.x, p.y).level = static_cast<std::uint8_t>(level);
  }
  // Bulk edits: ensure masks are consistent.
  world.recomputeRoadMasks();
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  Options opt;

  // Defaults consistent with proc_isocity_cli docs.
  opt.isoCfg.tileW = 16;
  opt.isoCfg.tileH = 8;
  opt.isoCfg.heightScalePx = 14;
  opt.isoCfg.marginPx = 8;
  opt.isoCfg.drawGrid = false;
  opt.isoCfg.drawCliffs = true;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
    std::string val;

    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, val)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
      opt.loadPath = val;
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &opt.seed)) {
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &opt.w, &opt.h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--mode") {
      if (!requireValue(i, val) || !ParseMode(val, opt.mode)) {
        std::cerr << "--mode requires one of: road, land, roadbuild, to-edge\n";
        return 2;
      }
    } else if (arg == "--start") {
      if (!requireValue(i, val) || !ParsePointXY(val, opt.start)) {
        std::cerr << "--start requires format x,y (e.g. 12,34)\n";
        return 2;
      }
      opt.haveStart = true;
    } else if (arg == "--goal") {
      if (!requireValue(i, val) || !ParsePointXY(val, opt.goal)) {
        std::cerr << "--goal requires format x,y (e.g. 56,34)\n";
        return 2;
      }
      opt.haveGoal = true;
    } else if (arg == "--allow-bridges") {
      if (!requireValue(i, val) || !ParseBool01(val, &opt.allowBridges)) {
        std::cerr << "--allow-bridges requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--cost-model") {
      if (!requireValue(i, val) || !ParseCostModel(val, opt.costModel)) {
        std::cerr << "--cost-model requires new or money\n";
        return 2;
      }
    } else if (arg == "--target-level") {
      if (!requireValue(i, val) || !ParseI32(val, &opt.targetLevel) || opt.targetLevel < 1 || opt.targetLevel > 3) {
        std::cerr << "--target-level requires 1..3\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      opt.outJson = val;
    } else if (arg == "--csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      opt.outCsv = val;
    } else if (arg == "--ppm") {
      if (!requireValue(i, val)) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
      opt.outPpm = val;
    } else if (arg == "--ppm-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &opt.ppmScale) || opt.ppmScale <= 0) {
        std::cerr << "--ppm-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--layer") {
      if (!requireValue(i, val) || !ParseExportLayer(val, opt.baseLayer)) {
        std::cerr << "--layer requires a valid name (terrain, overlay, height, district, ...)\n";
        return 2;
      }
    } else if (arg == "--iso") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso requires a path\n";
        return 2;
      }
      opt.outIso = val;
    } else if (arg == "--iso-tile") {
      int tw = 0;
      int th = 0;
      if (!requireValue(i, val) || !ParseWxH(val, &tw, &th) || (tw % 2) != 0 || (th % 2) != 0) {
        std::cerr << "--iso-tile requires even WxH (e.g. 16x8)\n";
        return 2;
      }
      opt.isoCfg.tileW = tw;
      opt.isoCfg.tileH = th;
    } else if (arg == "--iso-height") {
      int hs = 0;
      if (!requireValue(i, val) || !ParseI32(val, &hs) || hs < 0) {
        std::cerr << "--iso-height requires a non-negative integer\n";
        return 2;
      }
      opt.isoCfg.heightScalePx = hs;
    } else if (arg == "--iso-margin") {
      int m = 0;
      if (!requireValue(i, val) || !ParseI32(val, &m) || m < 0) {
        std::cerr << "--iso-margin requires a non-negative integer\n";
        return 2;
      }
      opt.isoCfg.marginPx = m;
    } else if (arg == "--iso-grid") {
      bool b = false;
      if (!requireValue(i, val) || !ParseBool01(val, &b)) {
        std::cerr << "--iso-grid requires 0 or 1\n";
        return 2;
      }
      opt.isoCfg.drawGrid = b;
    } else if (arg == "--iso-cliffs") {
      bool b = true;
      if (!requireValue(i, val) || !ParseBool01(val, &b)) {
        std::cerr << "--iso-cliffs requires 0 or 1\n";
        return 2;
      }
      opt.isoCfg.drawCliffs = b;
    } else if (arg == "--write-save") {
      if (!requireValue(i, val)) {
        std::cerr << "--write-save requires a path\n";
        return 2;
      }
      opt.outSave = val;
    } else if (arg == "--quiet") {
      opt.quiet = true;
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      PrintHelp();
      return 2;
    } else {
      std::cerr << "Unexpected positional argument: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (!opt.haveStart) {
    std::cerr << "Missing required --start\n";
    PrintHelp();
    return 2;
  }
  if (opt.mode != Mode::ToEdge && !opt.haveGoal) {
    std::cerr << "Missing required --goal (mode != to-edge)\n";
    PrintHelp();
    return 2;
  }
  if (!opt.outSave.empty() && opt.mode != Mode::RoadBuild) {
    std::cerr << "--write-save is only valid for --mode roadbuild\n";
    return 2;
  }

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};

  if (!opt.loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, opt.loadPath, err)) {
      std::cerr << "Failed to load save: " << opt.loadPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(opt.w, opt.h, opt.seed, procCfg);
  }

  if (!world.inBounds(opt.start.x, opt.start.y)) {
    std::cerr << "--start is out of bounds for world size " << world.width() << "x" << world.height() << "\n";
    return 2;
  }
  if (opt.mode != Mode::ToEdge && !world.inBounds(opt.goal.x, opt.goal.y)) {
    std::cerr << "--goal is out of bounds for world size " << world.width() << "x" << world.height() << "\n";
    return 2;
  }

  std::vector<Point> path;
  bool found = false;
  int steps = 0;
  int turns = 0;
  int primaryCost = 0;
  int newRoadTiles = 0;

  if (opt.mode == Mode::Road) {
    found = FindRoadPathAStar(world, opt.start, opt.goal, path, &steps);
  } else if (opt.mode == Mode::Land) {
    found = FindLandPathAStar(world, opt.start, opt.goal, path, &steps);
  } else if (opt.mode == Mode::ToEdge) {
    found = FindRoadPathToEdge(world, opt.start, path, &steps);
  } else if (opt.mode == Mode::RoadBuild) {
    RoadBuildPathConfig cfg;
    cfg.allowBridges = opt.allowBridges;
    cfg.targetLevel = opt.targetLevel;
    cfg.costModel = (opt.costModel == CostModel::Money) ? RoadBuildPathConfig::CostModel::Money
                                                        : RoadBuildPathConfig::CostModel::NewTiles;
    found = FindRoadBuildPath(world, opt.start, opt.goal, path, &primaryCost, cfg);
    steps = found ? std::max(0, static_cast<int>(path.size()) - 1) : 0;
    newRoadTiles = found ? CountNewRoadTiles(world, path) : 0;
  }

  turns = found ? CountTurns(path) : 0;

  if (!opt.quiet) {
    std::cout << "mode: " << ModeName(opt.mode) << "\n";
    if (!opt.loadPath.empty()) {
      std::cout << "load: " << opt.loadPath << "\n";
    } else {
      std::cout << "seed: " << opt.seed << "\n";
      std::cout << "size: " << world.width() << "x" << world.height() << "\n";
    }
    std::cout << "start: " << opt.start.x << "," << opt.start.y << "\n";
    if (opt.mode != Mode::ToEdge) {
      std::cout << "goal: " << opt.goal.x << "," << opt.goal.y << "\n";
    }
    std::cout << "found: " << (found ? "true" : "false") << "\n";
    if (found) {
      std::cout << "pathLength: " << static_cast<int>(path.size()) << "\n";
      std::cout << "steps: " << steps << "\n";
      std::cout << "turns: " << turns << "\n";
      if (opt.mode == Mode::RoadBuild) {
        std::cout << "roadbuild.allowBridges: " << (opt.allowBridges ? 1 : 0) << "\n";
        std::cout << "roadbuild.costModel: " << CostModelName(opt.costModel) << "\n";
        std::cout << "roadbuild.targetLevel: " << opt.targetLevel << "\n";
        std::cout << "roadbuild.primaryCost: " << primaryCost << "\n";
        std::cout << "roadbuild.newRoadTiles: " << newRoadTiles << "\n";
      }
      if (!path.empty()) {
        const Point end = path.back();
        std::cout << "end: " << end.x << "," << end.y << "\n";
      }
    }
  }

  std::string err;

  if (!opt.outCsv.empty() && found) {
    if (!WritePathCsv(opt.outCsv, path, err)) {
      std::cerr << "Failed to write CSV: " << opt.outCsv << "\n" << err << "\n";
      return 2;
    }
  }

  if (!opt.outJson.empty()) {
    if (!WriteJsonReport(opt.outJson, opt, world, found, steps, turns, primaryCost, newRoadTiles, path, err)) {
      std::cerr << "Failed to write JSON: " << opt.outJson << "\n" << err << "\n";
      return 2;
    }
  }

  // Tile-grid visualization.
  if (!opt.outPpm.empty() && found) {
    PpmImage img = RenderPpmLayer(world, opt.baseLayer, nullptr, nullptr, nullptr);

    // Highlight path.
    for (const Point& p : path) {
      if (!world.inBounds(p.x, p.y)) continue;

      if (opt.mode == Mode::RoadBuild) {
        const bool alreadyRoad = (world.at(p.x, p.y).overlay == Overlay::Road);
        if (alreadyRoad) SetPixel(img, p.x, p.y, 40, 220, 255);
        else SetPixel(img, p.x, p.y, 255, 170, 40);
      } else {
        SetPixel(img, p.x, p.y, 255, 60, 60);
      }
    }

    // Start/end markers.
    SetPixel(img, opt.start.x, opt.start.y, 40, 255, 60);
    if (found && !path.empty()) {
      const Point end = path.back();
      SetPixel(img, end.x, end.y, 60, 100, 255);
    }

    img = ScaleNearest(img, opt.ppmScale);
    if (!EnsureParentDir(opt.outPpm)) {
      std::cerr << "Failed to create output directory for: " << opt.outPpm << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outPpm, img, err)) {
      std::cerr << "Failed to write image: " << opt.outPpm << "\n" << err << "\n";
      return 2;
    }
  }

  // Isometric visualization.
  if (!opt.outIso.empty() && found) {
    IsoOverviewResult iso = RenderIsoOverview(world, opt.baseLayer, opt.isoCfg, nullptr, nullptr, nullptr);

    // Draw a polyline through tile centers.
    int prevX = 0;
    int prevY = 0;
    bool havePrev = false;
    for (std::size_t i = 0; i < path.size(); ++i) {
      int px = 0;
      int py = 0;
      if (!IsoTileCenterToPixel(world, iso, path[i].x, path[i].y, px, py)) continue;

      // Colored by mode.
      std::uint8_t r = 255;
      std::uint8_t g = 60;
      std::uint8_t b = 60;
      if (opt.mode == Mode::RoadBuild) {
        const bool alreadyRoad = world.inBounds(path[i].x, path[i].y) && (world.at(path[i].x, path[i].y).overlay == Overlay::Road);
        if (alreadyRoad) {
          r = 40;
          g = 220;
          b = 255;
        } else {
          r = 255;
          g = 170;
          b = 40;
        }
      }

      if (havePrev) {
        DrawLine(iso.image, prevX, prevY, px, py, r, g, b);
        // A tiny bit thicker.
        DrawLine(iso.image, prevX + 1, prevY, px + 1, py, r, g, b);
        DrawLine(iso.image, prevX, prevY + 1, px, py + 1, r, g, b);
      }
      DrawDot(iso.image, px, py, 1, r, g, b);
      prevX = px;
      prevY = py;
      havePrev = true;
    }

    // Start/end markers.
    int sx = 0;
    int sy = 0;
    if (IsoTileCenterToPixel(world, iso, opt.start.x, opt.start.y, sx, sy)) {
      DrawDot(iso.image, sx, sy, 3, 40, 255, 60);
    }
    if (!path.empty()) {
      int ex = 0;
      int ey = 0;
      const Point end = path.back();
      if (IsoTileCenterToPixel(world, iso, end.x, end.y, ex, ey)) {
        DrawDot(iso.image, ex, ey, 3, 60, 100, 255);
      }
    }

    if (!EnsureParentDir(opt.outIso)) {
      std::cerr << "Failed to create output directory for: " << opt.outIso << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outIso, iso.image, err)) {
      std::cerr << "Failed to write iso image: " << opt.outIso << "\n" << err << "\n";
      return 2;
    }
  }

  if (!opt.outSave.empty() && found) {
    ApplyRoadPath(world, path, opt.targetLevel);
    if (!EnsureParentDir(opt.outSave)) {
      std::cerr << "Failed to create output directory for: " << opt.outSave << "\n";
      return 2;
    }
    std::string saveErr;
    if (!SaveWorldBinary(world, procCfg, simCfg, opt.outSave, saveErr)) {
      std::cerr << "Failed to write save: " << opt.outSave << "\n" << saveErr << "\n";
      return 2;
    }
    if (!opt.quiet) {
      std::cout << "wrote save -> " << opt.outSave << "\n";
    }
  }

  return found ? 0 : 1;
}
