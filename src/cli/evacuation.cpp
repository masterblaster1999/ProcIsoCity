#include "isocity/DepressionFill.hpp"
#include "isocity/Evacuation.hpp"
#include "isocity/Export.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Road.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;

enum class HazardMode {
  None,
  Sea,
  Depressions,
  Both,
};

static std::string ToLower(std::string s)
{
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
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

static bool ParseHazardMode(const std::string& s, HazardMode* out)
{
  if (!out) return false;
  const std::string m = ToLower(s);
  if (m == "none" || m == "off" || m == "0") {
    *out = HazardMode::None;
    return true;
  }
  if (m == "sea" || m == "coast" || m == "coastal") {
    *out = HazardMode::Sea;
    return true;
  }
  if (m == "depressions" || m == "pond" || m == "ponding" || m == "priority_flood") {
    *out = HazardMode::Depressions;
    return true;
  }
  if (m == "both" || m == "all") {
    *out = HazardMode::Both;
    return true;
  }
  return false;
}

static bool ParseWeightModeToUseTravelTime(const std::string& s, bool* out)
{
  if (!out) return false;
  const std::string m = ToLower(s);
  if (m == "time" || m == "travel" || m == "traveltime" || m == "travel_time") {
    *out = true;
    return true;
  }
  if (m == "steps" || m == "len" || m == "length" || m == "unweighted") {
    *out = false;
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

static std::vector<float> ExtractHeights(const isocity::World& world)
{
  std::vector<float> h;
  const int w = world.width();
  const int hh = world.height();
  h.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(hh));
  for (int y = 0; y < hh; ++y) {
    for (int x = 0; x < w; ++x) {
      h[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = world.at(x, y).height;
    }
  }
  return h;
}

static std::vector<std::uint8_t> BuildWaterDrainMask(const isocity::World& world)
{
  const int w = world.width();
  const int h = world.height();
  std::vector<std::uint8_t> mask;
  mask.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const isocity::Tile& t = world.at(x, y);
      // Treat non-bridge water tiles as drains.
      if (t.terrain == isocity::Terrain::Water && t.overlay != isocity::Overlay::Road) {
        mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = 1;
      }
    }
  }
  return mask;
}

static float Clamp01(float v)
{
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static std::uint8_t ToByte(float v)
{
  int iv = static_cast<int>(std::lround(v));
  if (iv < 0) iv = 0;
  if (iv > 255) iv = 255;
  return static_cast<std::uint8_t>(iv);
}

// 0 -> green, 0.5 -> yellow, 1 -> red
static void HeatRampGreenYellowRed(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  if (t <= 0.5f) {
    r = ToByte(255.0f * (t * 2.0f));
    g = 255;
    b = 0;
  } else {
    r = 255;
    g = ToByte(255.0f * (1.0f - (t - 0.5f) * 2.0f));
    b = 0;
  }
}

static void BlendPixel(std::uint8_t& br, std::uint8_t& bg, std::uint8_t& bb, std::uint8_t r, std::uint8_t g, std::uint8_t b,
                       float alpha)
{
  const float a = Clamp01(alpha);
  const float ia = 1.0f - a;
  br = ToByte(ia * static_cast<float>(br) + a * static_cast<float>(r));
  bg = ToByte(ia * static_cast<float>(bg) + a * static_cast<float>(g));
  bb = ToByte(ia * static_cast<float>(bb) + a * static_cast<float>(b));
}

static isocity::PpmImage MakeMaskImage(int w, int h, const std::vector<std::uint8_t>& mask, std::uint8_t r, std::uint8_t g,
                                      std::uint8_t b)
{
  isocity::PpmImage img;
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

static void PrintHelp()
{
  std::cout
      << "proc_isocity_evac (headless evacuation accessibility + bottleneck analysis)\n\n"
      << "Computes which Residential tiles can reach a safe road exit on the map edge under an\n"
      << "optional hazard mask (sea-level inundation, ponding potential via Priority-Flood, or both).\n\n"
      << "Outputs:\n"
      << "  - JSON summary (reachability, average/p95 evacuation time, road bottlenecks)\n"
      << "  - Optional images (hazard mask, annotated accessibility map, road demand heatmap)\n\n"
      << "Usage:\n"
      << "  proc_isocity_evac [--load <save.bin>] [--seed <u64>] [--size <WxH>] [--days <N>]\n"
      << "                   [--mode <none|sea|depressions|both>]\n"
      << "                   [--sea-level <f>] [--sea-connect-edge <0|1>] [--sea-8conn <0|1>]\n"
      << "                   [--dep-eps <f>] [--dep-min-depth <f>]\n"
      << "                   [--weight-mode <time|steps>] [--walk-cost <steps>] [--road-capacity <N>]\n"
      << "                   [--json <out.json>] [--top-roads-csv <out.csv>]\n"
      << "                   [--hazard <out.png>] [--annotate <out.png>] [--flow <out.png>]\n"
      << "                   [--ppm-scale <N>] [--top-n <N>]\n\n"
      << "Notes:\n"
      << "  - If --load is omitted, a world is generated from (--seed, --size).\n"
      << "  - --days runs the simulation for N ticks before analysis so zones populate.\n"
      << "  - --walk-cost is in street-steps (1.0 == 1000 milli-steps).\n\n"
      << "Examples:\n"
      << "  # Coastal evacuation analysis\n"
      << "  ./build/proc_isocity_evac --seed 1 --size 128x128 --days 120 --mode sea --sea-level 0.45 \\\n"
      << "    --json evac.json --annotate evac.png --flow evac_flow.png --hazard hazard.png --ppm-scale 4\n\n"
      << "  # Ponding-aware analysis on a save (Priority-Flood depth threshold)\n"
      << "  ./build/proc_isocity_evac --load save.bin --days 0 --mode depressions --dep-min-depth 0.02 \\\n"
      << "    --json evac_pond.json --annotate evac_pond.png --ppm-scale 4\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;
  int days = 0;

  HazardMode mode = HazardMode::Sea;

  bool seaLevelSet = false;
  float seaLevel = 0.0f;
  bool seaConnectEdge = true;
  bool sea8 = false;

  float depEps = 0.0f;
  float depMinDepth = 0.01f;

  bool useTravelTime = true;
  float walkCostSteps = 1.0f;
  int roadCapacity = 28;

  std::string outJson;
  std::string outTopRoadsCsv;
  std::string outHazard;
  std::string outAnnotate;
  std::string outFlow;
  int ppmScale = 1;
  int topN = 25;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    std::string v;

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    }

    if (a == "--load") {
      if (!requireValue(i, v)) {
        std::cerr << "--load requires a value\n";
        return 2;
      }
      loadPath = v;
      continue;
    }

    if (a == "--seed") {
      if (!requireValue(i, v) || !ParseU64(v, &seed)) {
        std::cerr << "--seed requires a u64\n";
        return 2;
      }
      continue;
    }

    if (a == "--size") {
      if (!requireValue(i, v) || !ParseSize(v, &w, &h)) {
        std::cerr << "--size requires WxH (e.g. 128x128)\n";
        return 2;
      }
      continue;
    }

    if (a == "--days") {
      if (!requireValue(i, v) || !ParseI32(v, &days) || days < 0) {
        std::cerr << "--days requires a non-negative integer\n";
        return 2;
      }
      continue;
    }

    if (a == "--mode") {
      if (!requireValue(i, v) || !ParseHazardMode(v, &mode)) {
        std::cerr << "--mode must be one of: none, sea, depressions, both\n";
        return 2;
      }
      continue;
    }

    if (a == "--sea-level") {
      if (!requireValue(i, v) || !ParseF32(v, &seaLevel)) {
        std::cerr << "--sea-level requires a float\n";
        return 2;
      }
      seaLevelSet = true;
      continue;
    }

    if (a == "--sea-connect-edge") {
      if (!requireValue(i, v) || !ParseBool01(v, &seaConnectEdge)) {
        std::cerr << "--sea-connect-edge requires 0 or 1\n";
        return 2;
      }
      continue;
    }

    if (a == "--sea-8conn") {
      if (!requireValue(i, v) || !ParseBool01(v, &sea8)) {
        std::cerr << "--sea-8conn requires 0 or 1\n";
        return 2;
      }
      continue;
    }

    if (a == "--dep-eps") {
      if (!requireValue(i, v) || !ParseF32(v, &depEps) || depEps < 0.0f) {
        std::cerr << "--dep-eps requires a non-negative float\n";
        return 2;
      }
      continue;
    }

    if (a == "--dep-min-depth") {
      if (!requireValue(i, v) || !ParseF32(v, &depMinDepth) || depMinDepth < 0.0f) {
        std::cerr << "--dep-min-depth requires a non-negative float\n";
        return 2;
      }
      continue;
    }

    if (a == "--weight-mode") {
      if (!requireValue(i, v) || !ParseWeightModeToUseTravelTime(v, &useTravelTime)) {
        std::cerr << "--weight-mode must be one of: time, steps\n";
        return 2;
      }
      continue;
    }

    if (a == "--walk-cost") {
      if (!requireValue(i, v) || !ParseF32(v, &walkCostSteps) || walkCostSteps < 0.0f) {
        std::cerr << "--walk-cost requires a non-negative float\n";
        return 2;
      }
      continue;
    }

    if (a == "--road-capacity") {
      if (!requireValue(i, v) || !ParseI32(v, &roadCapacity) || roadCapacity < 0) {
        std::cerr << "--road-capacity requires a non-negative int\n";
        return 2;
      }
      continue;
    }

    if (a == "--json") {
      if (!requireValue(i, v)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJson = v;
      continue;
    }

    if (a == "--top-roads-csv") {
      if (!requireValue(i, v)) {
        std::cerr << "--top-roads-csv requires a path\n";
        return 2;
      }
      outTopRoadsCsv = v;
      continue;
    }

    if (a == "--hazard") {
      if (!requireValue(i, v)) {
        std::cerr << "--hazard requires a path\n";
        return 2;
      }
      outHazard = v;
      continue;
    }

    if (a == "--annotate") {
      if (!requireValue(i, v)) {
        std::cerr << "--annotate requires a path\n";
        return 2;
      }
      outAnnotate = v;
      continue;
    }

    if (a == "--flow") {
      if (!requireValue(i, v)) {
        std::cerr << "--flow requires a path\n";
        return 2;
      }
      outFlow = v;
      continue;
    }

    if (a == "--ppm-scale") {
      if (!requireValue(i, v) || !ParseI32(v, &ppmScale) || ppmScale < 1) {
        std::cerr << "--ppm-scale requires an int >= 1\n";
        return 2;
      }
      continue;
    }

    if (a == "--top-n") {
      if (!requireValue(i, v) || !ParseI32(v, &topN) || topN < 0) {
        std::cerr << "--top-n requires a non-negative int\n";
        return 2;
      }
      continue;
    }

    std::cerr << "Unknown arg: " << a << "\n";
    PrintHelp();
    return 2;
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n" << err << "\n";
      return 1;
    }
  } else {
    // Generate from defaults.
    world = GenerateWorld(w, h, seed, procCfg);
  }

  // If the user didn't supply sea level, default to the ProcGen waterLevel.
  if (!seaLevelSet) {
    seaLevel = procCfg.waterLevel;
  }

  // Optional pre-simulation to populate zones.
  if (days > 0) {
    Simulator sim(simCfg);
    for (int d = 0; d < days; ++d) {
      sim.stepOnce(world);
    }
  }

  const int W = world.width();
  const int H = world.height();
  const std::size_t N = static_cast<std::size_t>(std::max(0, W)) * static_cast<std::size_t>(std::max(0, H));

  std::vector<std::uint8_t> hazardMask;
  hazardMask.assign(N, 0);

  if (mode != HazardMode::None) {
    const std::vector<float> heights = ExtractHeights(world);

    if (mode == HazardMode::Sea || mode == HazardMode::Both) {
      SeaFloodConfig scfg;
      scfg.requireEdgeConnection = seaConnectEdge;
      scfg.eightConnected = sea8;

      SeaFloodResult sea = ComputeSeaLevelFlood(heights, W, H, seaLevel, scfg);
      for (std::size_t i = 0; i < hazardMask.size() && i < sea.flooded.size(); ++i) {
        if (sea.flooded[i] != 0) hazardMask[i] = 1;
      }
    }

    if (mode == HazardMode::Depressions || mode == HazardMode::Both) {
      DepressionFillConfig dcfg;
      dcfg.epsilon = depEps;

      std::vector<std::uint8_t> drains = BuildWaterDrainMask(world);
      DepressionFillResult dep = FillDepressionsPriorityFlood(heights, W, H, &drains, dcfg);

      for (std::size_t i = 0; i < hazardMask.size() && i < dep.depth.size(); ++i) {
        if (dep.depth[i] >= depMinDepth) hazardMask[i] = 1;
      }
    }
  }

  EvacuationConfig ecfg;
  ecfg.useTravelTime = useTravelTime;
  ecfg.walkCostMilli = std::max(0, static_cast<int>(std::lround(walkCostSteps * 1000.0f)));
  ecfg.roadTileCapacity = std::max(0, roadCapacity);

  const EvacuationResult evac = ComputeEvacuationToEdge(world, ecfg, (mode == HazardMode::None) ? nullptr : &hazardMask);

  // --- Images ---
  if (!outHazard.empty()) {
    if (!EnsureParentDir(outHazard)) {
      std::cerr << "Failed to create output directory for: " << outHazard << "\n";
      return 1;
    }
    PpmImage img = MakeMaskImage(W, H, hazardMask, 40, 120, 255);
    if (ppmScale > 1) img = ScaleNearest(img, ppmScale);
    std::string err;
    if (!WriteImageAuto(outHazard, img, err)) {
      std::cerr << "Failed to write hazard image: " << err << "\n";
      return 1;
    }
  }

  if (!outAnnotate.empty()) {
    if (!EnsureParentDir(outAnnotate)) {
      std::cerr << "Failed to create output directory for: " << outAnnotate << "\n";
      return 1;
    }

    PpmImage base = RenderPpmLayer(world, ExportLayer::Overlay);

    // Find max reachable res cost for normalization.
    int maxCost = 0;
    for (int c : evac.resCostMilli) {
      if (c > maxCost) maxCost = c;
    }
    if (maxCost <= 0) maxCost = 1;

    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int idx = y * W + x;
        const std::size_t ui = static_cast<std::size_t>(idx);
        if (ui * 3u + 2u >= base.rgb.size()) continue;
        std::uint8_t& br = base.rgb[ui * 3u + 0u];
        std::uint8_t& bg = base.rgb[ui * 3u + 1u];
        std::uint8_t& bb = base.rgb[ui * 3u + 2u];

        const Tile& t = world.at(x, y);
        const bool hazard = (ui < hazardMask.size()) ? (hazardMask[ui] != 0) : false;

        if (hazard) {
          BlendPixel(br, bg, bb, 40, 120, 255, 0.65f);
        }

        if (t.overlay == Overlay::Residential && t.terrain != Terrain::Water && !hazard) {
          const int c = (ui < evac.resCostMilli.size()) ? evac.resCostMilli[ui] : -1;
          if (c < 0) {
            // Unreachable residential.
            BlendPixel(br, bg, bb, 255, 40, 40, 0.80f);
          } else {
            // Reachable: shade by evacuation time (green=fast, red=slow).
            const float t01 = Clamp01(static_cast<float>(c) / static_cast<float>(maxCost));
            std::uint8_t rr = 0, rg = 0, rb = 0;
            HeatRampGreenYellowRed(t01, rr, rg, rb);
            BlendPixel(br, bg, bb, rr, rg, rb, 0.55f);
          }
        }
      }
    }

    if (ppmScale > 1) base = ScaleNearest(base, ppmScale);
    std::string err;
    if (!WriteImageAuto(outAnnotate, base, err)) {
      std::cerr << "Failed to write annotated image: " << err << "\n";
      return 1;
    }
  }

  if (!outFlow.empty()) {
    if (!EnsureParentDir(outFlow)) {
      std::cerr << "Failed to create output directory for: " << outFlow << "\n";
      return 1;
    }

    // Terrain base.
    PpmImage img = RenderPpmLayer(world, ExportLayer::Terrain);
    const std::uint32_t maxFlow = std::max<std::uint32_t>(1u, evac.maxEvacRoadFlow);

    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int idx = y * W + x;
        const std::size_t ui = static_cast<std::size_t>(idx);
        if (ui * 3u + 2u >= img.rgb.size()) continue;
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;
        const std::uint32_t flow = (ui < evac.evacRoadFlow.size()) ? evac.evacRoadFlow[ui] : 0u;
        if (flow == 0u) continue;

        const float t01 = Clamp01(static_cast<float>(flow) / static_cast<float>(maxFlow));
        std::uint8_t r = 0, g = 0, b = 0;
        HeatRampGreenYellowRed(t01, r, g, b);
        std::uint8_t& br = img.rgb[ui * 3u + 0u];
        std::uint8_t& bg = img.rgb[ui * 3u + 1u];
        std::uint8_t& bb = img.rgb[ui * 3u + 2u];
        BlendPixel(br, bg, bb, r, g, b, 0.80f);
      }
    }

    if (ppmScale > 1) img = ScaleNearest(img, ppmScale);
    std::string err;
    if (!WriteImageAuto(outFlow, img, err)) {
      std::cerr << "Failed to write flow image: " << err << "\n";
      return 1;
    }
  }

  // --- Reports ---
  struct RoadRow {
    int x = 0;
    int y = 0;
    std::uint32_t flow = 0;
    int capacity = 0;
    double util = 0.0;
    int costToExit = -1;
    int stepsToExit = -1;
  };

  std::vector<RoadRow> top;
  top.reserve(static_cast<std::size_t>(topN));

  if (topN > 0) {
    std::vector<RoadRow> all;
    all.reserve(N / 4);
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        const int idx = y * W + x;
        const std::size_t ui = static_cast<std::size_t>(idx);
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;
        const std::uint32_t flow = (ui < evac.evacRoadFlow.size()) ? evac.evacRoadFlow[ui] : 0u;
        if (flow == 0u) continue;
        const int cap = RoadCapacityForLevel(std::max(0, roadCapacity), t.level);
        const double util = (cap > 0) ? (static_cast<double>(flow) / static_cast<double>(cap)) : 0.0;
        const int costToExit = (ui < evac.roadCostMilli.size()) ? evac.roadCostMilli[ui] : -1;
        const int stepsToExit = (ui < evac.roadDistSteps.size()) ? evac.roadDistSteps[ui] : -1;
        all.push_back(RoadRow{x, y, flow, cap, util, costToExit, stepsToExit});
      }
    }

    std::sort(all.begin(), all.end(), [](const RoadRow& a, const RoadRow& b) {
      if (a.util != b.util) return a.util > b.util;
      if (a.flow != b.flow) return a.flow > b.flow;
      if (a.y != b.y) return a.y < b.y;
      return a.x < b.x;
    });

    if (static_cast<int>(all.size()) > topN) all.resize(static_cast<std::size_t>(topN));
    top = std::move(all);
  }

  if (!outTopRoadsCsv.empty()) {
    if (!EnsureParentDir(outTopRoadsCsv)) {
      std::cerr << "Failed to create output directory for: " << outTopRoadsCsv << "\n";
      return 1;
    }
    std::ofstream f(outTopRoadsCsv);
    if (!f) {
      std::cerr << "Failed to open for write: " << outTopRoadsCsv << "\n";
      return 1;
    }
    f << "x,y,flow,capacity,util,costToExitMilli,stepsToExit\n";
    for (const auto& r : top) {
      f << r.x << "," << r.y << "," << r.flow << "," << r.capacity << "," << r.util << "," << r.costToExit << "," << r.stepsToExit
        << "\n";
    }
  }

  if (!outJson.empty()) {
    if (!EnsureParentDir(outJson)) {
      std::cerr << "Failed to create output directory for: " << outJson << "\n";
      return 1;
    }
    std::ofstream f(outJson);
    if (!f) {
      std::cerr << "Failed to open for write: " << outJson << "\n";
      return 1;
    }

    auto hazardModeName = [&]() -> const char* {
      switch (mode) {
      case HazardMode::None:
        return "none";
      case HazardMode::Sea:
        return "sea";
      case HazardMode::Depressions:
        return "depressions";
      case HazardMode::Both:
        return "both";
      }
      return "unknown";
    };

    f << "{\n";
    f << "  \"w\": " << W << ",\n";
    f << "  \"h\": " << H << ",\n";
    f << "  \"seed\": " << seed << ",\n";
    f << "  \"days\": " << days << ",\n";
    f << "  \"mode\": \"" << hazardModeName() << "\",\n";
    f << "  \"seaLevel\": " << seaLevel << ",\n";
    f << "  \"weightMode\": \"" << (useTravelTime ? "time" : "steps") << "\",\n";
    f << "  \"walkCostSteps\": " << walkCostSteps << ",\n";
    f << "  \"exitRoadSources\": " << evac.exitSources << ",\n";

    f << "  \"residential\": {\n";
    f << "    \"tiles\": " << evac.residentialTiles << ",\n";
    f << "    \"population\": " << evac.population << ",\n";
    f << "    \"floodedTiles\": " << evac.floodedResidentialTiles << ",\n";
    f << "    \"floodedPopulation\": " << evac.floodedPopulation << ",\n";
    f << "    \"reachableTiles\": " << evac.reachableResidentialTiles << ",\n";
    f << "    \"reachablePopulation\": " << evac.reachablePopulation << ",\n";
    f << "    \"unreachableTiles\": " << evac.unreachableResidentialTiles << ",\n";
    f << "    \"unreachablePopulation\": " << evac.unreachablePopulation << ",\n";
    f << "    \"avgEvacTimeSteps\": " << evac.avgEvacTime << ",\n";
    f << "    \"p95EvacTimeSteps\": " << evac.p95EvacTime << "\n";
    f << "  },\n";

    f << "  \"roads\": {\n";
    f << "    \"maxEvacFlow\": " << evac.maxEvacRoadFlow << ",\n";
    f << "    \"congestedTiles\": " << evac.congestedRoadTiles << ",\n";
    f << "    \"congestion\": " << evac.congestion << "\n";
    f << "  },\n";

    f << "  \"topRoads\": [\n";
    for (std::size_t i = 0; i < top.size(); ++i) {
      const auto& r = top[i];
      f << "    {\"x\": " << r.x << ", \"y\": " << r.y << ", \"flow\": " << r.flow << ", \"capacity\": " << r.capacity
        << ", \"util\": " << r.util << ", \"costToExitMilli\": " << r.costToExit << ", \"stepsToExit\": " << r.stepsToExit << "}";
      if (i + 1 < top.size()) f << ",";
      f << "\n";
    }
    f << "  ]\n";
    f << "}\n";
  }

  return 0;
}
