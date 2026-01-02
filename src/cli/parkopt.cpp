#include "isocity/Export.hpp"
#include "isocity/Isochrone.hpp"
#include "isocity/ParkOptimizer.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <algorithm>
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

using namespace isocity;

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

bool ParseDemandMode(const std::string& s, ParkDemandMode& out)
{
  if (s == "tiles" || s == "tile") {
    out = ParkDemandMode::Tiles;
    return true;
  }
  if (s == "occ" || s == "occupants" || s == "population") {
    out = ParkDemandMode::Occupants;
    return true;
  }
  return false;
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

void PrintHelp()
{
  std::cout
      << "proc_isocity_parkopt (headless park placement optimizer)\n\n"
      << "Suggests new park locations that best serve underserved zone demand.\n"
      << "Demand can be weighted by zone tiles or occupants, and distance is\n"
      << "measured along the road network (steps or travel-time).\n\n"
      << "Usage:\n"
      << "  proc_isocity_parkopt [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                    [--days <N>] [--require-outside <0|1>]\n"
      << "                    [--add <N>] [--weight-mode <steps|time>]\n"
      << "                    [--demand-mode <occupants|tiles>] [--zones <csv>]\n"
      << "                    [--target <streetSteps>]\n"
      << "                    [--json <out.json>] [--csv <out.csv>]\n"
      << "                    [--annotate <out.png|out.ppm>]\n"
      << "                    [--heat-before <out.png|out.ppm>] [--heat-after <out.png|out.ppm>]\n"
      << "                    [--heat-scale <N>] [--heat-max <streetSteps>]\n"
      << "                    [--save <out.bin>]\n\n"
      << "Notes:\n"
      << "  - The optimizer is greedy: it places parks one-by-one, each time\n"
      << "    picking the road access point with the highest (distance * demand) score.\n"
      << "  - It does not charge money; --save writes a tooling-friendly modified save.\n"
      << "  - Distance units are milli-steps (Street step ~= 1000).\n\n"
      << "Examples:\n"
      << "  # Generate a world, simulate 60 days, suggest 12 parks, write artifacts\n"
      << "  ./build/proc_isocity_parkopt --seed 1 --size 128x128 --days 60 --add 12 \\\n"
      << "    --json parks.json --annotate parks.png --heat-before park_before.png --heat-after park_after.png\n\n"
      << "  # Improve an existing save by placing 8 parks (writes a new save)\n"
      << "  ./build/proc_isocity_parkopt --load save.bin --add 8 --save save_more_parks.bin\n";
}

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
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

inline void BlendPixel(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, std::uint8_t hr, std::uint8_t hg, std::uint8_t hb)
{
  // Blend 2/3 heatmap, 1/3 base.
  r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
  g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
  b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
}

struct CostSummary {
  std::uint64_t totalWeight = 0;
  std::uint64_t reachableWeight = 0;
  std::uint64_t withinTargetWeight = 0;

  double avgCostSteps = 0.0;
  double p95CostSteps = 0.0;
  double maxCostSteps = 0.0;

  double reachableFrac() const
  {
    return (totalWeight > 0) ? (static_cast<double>(reachableWeight) / static_cast<double>(totalWeight)) : 0.0;
  }

  double withinTargetFrac() const
  {
    return (totalWeight > 0) ? (static_cast<double>(withinTargetWeight) / static_cast<double>(totalWeight)) : 0.0;
  }
};

bool IsIncludedZoneOverlay(Overlay o, const ParkOptimizerConfig& cfg)
{
  if (o == Overlay::Residential) return cfg.includeResidential;
  if (o == Overlay::Commercial) return cfg.includeCommercial;
  if (o == Overlay::Industrial) return cfg.includeIndustrial;
  return false;
}

std::uint64_t DemandWeightForTile(const Tile& t, const ParkOptimizerConfig& cfg)
{
  if (cfg.demandMode == ParkDemandMode::Tiles) return 1u;
  return static_cast<std::uint64_t>(t.occupants);
}

CostSummary SummarizeCosts(const World& world, const ParkOptimizerConfig& cfg, const std::vector<int>& costMilli,
                          int targetCostMilli)
{
  CostSummary s;
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return s;
  if (costMilli.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) return s;

  struct Row {
    int cost = 0;
    std::uint64_t w = 0;
  };
  std::vector<Row> rows;
  rows.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 2u);

  long double sumCost = 0.0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (!IsIncludedZoneOverlay(t.overlay, cfg)) continue;
      const std::uint64_t wgt = DemandWeightForTile(t, cfg);
      if (wgt == 0) continue;
      const int c = costMilli[FlatIdx(x, y, w)];
      s.totalWeight += wgt;
      if (c >= 0) {
        s.reachableWeight += wgt;
        sumCost += static_cast<long double>(c) * static_cast<long double>(wgt);
        rows.push_back(Row{c, wgt});
        if (targetCostMilli > 0 && c <= targetCostMilli) {
          s.withinTargetWeight += wgt;
        }
      }
    }
  }

  if (s.reachableWeight > 0) {
    const long double meanMilli = sumCost / static_cast<long double>(s.reachableWeight);
    s.avgCostSteps = static_cast<double>(meanMilli) / 1000.0;

    std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
      if (a.cost != b.cost) return a.cost < b.cost;
      return a.w < b.w;
    });

    const std::uint64_t qW = static_cast<std::uint64_t>(std::ceil(static_cast<long double>(s.reachableWeight) * 0.95L));
    std::uint64_t acc = 0;
    int p95 = rows.back().cost;
    for (const Row& r : rows) {
      acc += r.w;
      if (acc >= qW) {
        p95 = r.cost;
        break;
      }
    }
    s.p95CostSteps = static_cast<double>(p95) / 1000.0;
    s.maxCostSteps = static_cast<double>(rows.back().cost) / 1000.0;
  }

  return s;
}

std::vector<int> CollectParkSources(const World& world, const std::vector<std::uint8_t>* roadToEdgeMask)
{
  const int w = world.width();
  const int h = world.height();
  std::vector<int> sources;
  if (w <= 0 || h <= 0) return sources;
  sources.reserve(64);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Park) continue;
      if (t.terrain == Terrain::Water) continue;
      Point road;
      if (!PickAdjacentRoadTile(world, roadToEdgeMask, x, y, road)) continue;
      sources.push_back(road.y * w + road.x);
    }
  }

  std::sort(sources.begin(), sources.end());
  sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
  return sources;
}

RoadIsochroneField BuildParkRoadField(const World& world,
                                     const std::vector<int>& sources,
                                     const ParkOptimizerConfig& cfg,
                                     const std::vector<std::uint8_t>* roadToEdge)
{
  RoadIsochroneField empty;
  empty.w = world.width();
  empty.h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, empty.w)) * static_cast<std::size_t>(std::max(0, empty.h));
  empty.costMilli.assign(n, -1);
  empty.steps.assign(n, -1);

  if (sources.empty()) return empty;

  RoadIsochroneConfig icfg;
  icfg.requireOutsideConnection = cfg.requireOutsideConnection;
  icfg.weightMode = cfg.weightMode;
  icfg.computeOwner = false;

  return BuildRoadIsochroneField(world, sources, icfg, roadToEdge, nullptr);
}

bool WriteAnnotatedOverlay(const std::string& path, const World& world, const ParkOptimizerResult& plan,
                           std::string& outError)
{
  PpmImage img = RenderPpmLayer(world, ExportLayer::Overlay);
  if (img.width <= 0 || img.height <= 0 || img.rgb.empty()) {
    outError = "failed to render base overlay";
    return false;
  }

  // Highlight suggested parks in magenta.
  for (const ParkPlacement& p : plan.placements) {
    SetPixel(img, p.parkTile.x, p.parkTile.y, 255, 0, 255);
  }

  return WriteImageAuto(path, img, outError);
}

bool WriteCostHeatmapOverlay(const std::string& path, const World& world, const ParkOptimizerConfig& cfg,
                             const std::vector<int>& tileCostMilli,
                             int scale, int clampSteps,
                             const ParkOptimizerResult* plan,
                             std::string& outError)
{
  if (scale < 1) scale = 1;
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    outError = "invalid world dimensions";
    return false;
  }
  if (tileCostMilli.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
    outError = "cost field size mismatch";
    return false;
  }

  PpmImage base = RenderPpmLayer(world, ExportLayer::Overlay);
  if (base.width != w || base.height != h || base.rgb.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u) {
    outError = "failed to render base layer";
    return false;
  }

  const int clampMilli = std::max(1, clampSteps) * 1000;

  PpmImage out;
  out.width = w * scale;
  out.height = h * scale;
  out.rgb.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 3u, 0);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = FlatIdx(x, y, w);
      std::uint8_t r = base.rgb[idx * 3u + 0];
      std::uint8_t g = base.rgb[idx * 3u + 1];
      std::uint8_t b = base.rgb[idx * 3u + 2];

      const Tile& t = world.at(x, y);
      if (IsIncludedZoneOverlay(t.overlay, cfg)) {
        const int c = tileCostMilli[idx];
        if (c >= 0) {
          const float norm = 1.0f - std::min(1.0f, static_cast<float>(c) / static_cast<float>(clampMilli));
          std::uint8_t hr = 0, hg = 0, hb = 0;
          HeatRampRedYellowGreen(norm, hr, hg, hb);
          BlendPixel(r, g, b, hr, hg, hb);
        } else {
          // Unreachable zones: darken.
          r = static_cast<std::uint8_t>(static_cast<int>(r) / 4);
          g = static_cast<std::uint8_t>(static_cast<int>(g) / 4);
          b = static_cast<std::uint8_t>(static_cast<int>(b) / 4);
        }
      }

      // Upscale write.
      for (int oy = 0; oy < scale; ++oy) {
        for (int ox = 0; ox < scale; ++ox) {
          SetPixel(out, x * scale + ox, y * scale + oy, r, g, b);
        }
      }
    }
  }

  // Optionally highlight suggested parks.
  if (plan) {
    for (const ParkPlacement& p : plan->placements) {
      for (int oy = 0; oy < scale; ++oy) {
        for (int ox = 0; ox < scale; ++ox) {
          SetPixel(out, p.parkTile.x * scale + ox, p.parkTile.y * scale + oy, 255, 0, 255);
        }
      }
    }
  }

  return WriteImageAuto(path, out, outError);
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

bool WriteCsv(const std::string& path, const ParkOptimizerResult& plan, std::string& outError)
{
  std::ofstream f(path);
  if (!f) {
    outError = "Failed to open for writing: " + path;
    return false;
  }

  f << "rank,park_x,park_y,road_x,road_y,demand_weight,cost_before_milli,score\n";
  for (std::size_t i = 0; i < plan.placements.size(); ++i) {
    const ParkPlacement& p = plan.placements[i];
    f << (i + 1) << ','
      << p.parkTile.x << ',' << p.parkTile.y << ','
      << p.accessRoad.x << ',' << p.accessRoad.y << ','
      << p.demandWeight << ','
      << p.costMilliBefore << ','
      << p.score << "\n";
  }
  return true;
}

bool WriteJson(const std::string& path,
               const World& world,
               const ParkOptimizerResult& plan,
               const CostSummary& before,
               const CostSummary& after,
               int targetCostMilli,
               std::string& outError)
{
  std::ofstream f(path);
  if (!f) {
    outError = "Failed to open for writing: " + path;
    return false;
  }

  f << "{\n";
  f << "  \"world\": {\"w\":" << world.width() << ",\"h\":" << world.height() << ",\"day\":" << world.stats().day << "},\n";
  f << "  \"config\": {\n";
  f << "    \"require_outside\": " << (plan.cfg.requireOutsideConnection ? "true" : "false") << ",\n";
  f << "    \"weight_mode\": \"" << ((plan.cfg.weightMode == IsochroneWeightMode::Steps) ? "steps" : "time") << "\",\n";
  f << "    \"demand_mode\": \"" << ((plan.cfg.demandMode == ParkDemandMode::Tiles) ? "tiles" : "occupants") << "\",\n";
  f << "    \"include_residential\": " << (plan.cfg.includeResidential ? "true" : "false") << ",\n";
  f << "    \"include_commercial\": " << (plan.cfg.includeCommercial ? "true" : "false") << ",\n";
  f << "    \"include_industrial\": " << (plan.cfg.includeIndustrial ? "true" : "false") << ",\n";
  f << "    \"parks_to_add\": " << plan.cfg.parksToAdd;
  if (targetCostMilli > 0) {
    f << ",\n    \"target_cost_steps\": " << (static_cast<double>(targetCostMilli) / 1000.0);
  }
  f << "\n  },\n";

  f << "  \"summary\": {\n";
  f << "    \"existing_parks\": " << plan.existingParks << ",\n";
  f << "    \"total_demand_weight\": " << plan.totalDemandWeight << ",\n";
  f << "    \"before\": {\n";
  f << "      \"reachable_frac\": " << before.reachableFrac() << ",\n";
  f << "      \"avg_cost_steps\": " << before.avgCostSteps << ",\n";
  f << "      \"p95_cost_steps\": " << before.p95CostSteps << ",\n";
  f << "      \"max_cost_steps\": " << before.maxCostSteps;
  if (targetCostMilli > 0) {
    f << ",\n      \"within_target_frac\": " << before.withinTargetFrac();
  }
  f << "\n    },\n";

  f << "    \"after\": {\n";
  f << "      \"reachable_frac\": " << after.reachableFrac() << ",\n";
  f << "      \"avg_cost_steps\": " << after.avgCostSteps << ",\n";
  f << "      \"p95_cost_steps\": " << after.p95CostSteps << ",\n";
  f << "      \"max_cost_steps\": " << after.maxCostSteps;
  if (targetCostMilli > 0) {
    f << ",\n      \"within_target_frac\": " << after.withinTargetFrac();
  }
  f << "\n    }\n";
  f << "  },\n";

  f << "  \"placements\": [\n";
  for (std::size_t i = 0; i < plan.placements.size(); ++i) {
    const ParkPlacement& p = plan.placements[i];
    f << "    {\"rank\":" << (i + 1)
      << ",\"park_x\":" << p.parkTile.x
      << ",\"park_y\":" << p.parkTile.y
      << ",\"road_x\":" << p.accessRoad.x
      << ",\"road_y\":" << p.accessRoad.y
      << ",\"demand_weight\":" << p.demandWeight
      << ",\"cost_before_milli\":" << p.costMilliBefore
      << ",\"score\":" << p.score << "}";
    if (i + 1 < plan.placements.size()) f << ',';
    f << "\n";
  }
  f << "  ]\n";

  f << "}\n";
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc <= 1) {
    PrintHelp();
    return 0;
  }

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;
  int days = 0;
  bool requireOutside = true;

  ParkOptimizerConfig cfg;
  cfg.parksToAdd = 10;

  std::string zonesCsv = "res,com,ind";

  int targetSteps = 0;

  std::string outJson;
  std::string outCsv;
  std::string outAnnotate;
  std::string outHeatBefore;
  std::string outHeatAfter;
  int heatScale = 4;
  int heatMaxSteps = 40;
  std::string outSave;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
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
        std::cerr << "--seed requires a u64\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val, argc, argv) || !ParseSize(val, &w, &h)) {
        std::cerr << "--size requires WxH\n";
        return 2;
      }
    } else if (arg == "--days") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &days) || days < 0) {
        std::cerr << "--days requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      bool b = true;
      if (!requireValue(i, val, argc, argv) || !ParseBool01(val, &b)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      requireOutside = b;
      cfg.requireOutsideConnection = b;
    } else if (arg == "--add") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &cfg.parksToAdd) || cfg.parksToAdd < 0) {
        std::cerr << "--add requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--weight-mode") {
      if (!requireValue(i, val, argc, argv) || !ParseWeightMode(val, cfg.weightMode)) {
        std::cerr << "--weight-mode requires steps or time\n";
        return 2;
      }
    } else if (arg == "--demand-mode") {
      if (!requireValue(i, val, argc, argv) || !ParseDemandMode(val, cfg.demandMode)) {
        std::cerr << "--demand-mode requires occupants or tiles\n";
        return 2;
      }
    } else if (arg == "--zones") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--zones requires a csv string\n";
        return 2;
      }
      zonesCsv = val;
    } else if (arg == "--target") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &targetSteps) || targetSteps < 0) {
        std::cerr << "--target requires a non-negative integer (street steps)\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--csv") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      outCsv = val;
    } else if (arg == "--annotate") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--annotate requires a path\n";
        return 2;
      }
      outAnnotate = val;
    } else if (arg == "--heat-before") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--heat-before requires a path\n";
        return 2;
      }
      outHeatBefore = val;
    } else if (arg == "--heat-after") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--heat-after requires a path\n";
        return 2;
      }
      outHeatAfter = val;
    } else if (arg == "--heat-scale") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &heatScale) || heatScale < 1) {
        std::cerr << "--heat-scale requires an integer >= 1\n";
        return 2;
      }
    } else if (arg == "--heat-max") {
      if (!requireValue(i, val, argc, argv) || !ParseI32(val, &heatMaxSteps) || heatMaxSteps < 1) {
        std::cerr << "--heat-max requires an integer >= 1 (street steps)\n";
        return 2;
      }
    } else if (arg == "--save") {
      if (!requireValue(i, val, argc, argv)) {
        std::cerr << "--save requires a path\n";
        return 2;
      }
      outSave = val;
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      std::cerr << "Run with --help for usage.\n";
      return 2;
    }
  }

  // Zones selection.
  cfg.includeResidential = false;
  cfg.includeCommercial = false;
  cfg.includeIndustrial = false;
  for (const std::string& tok : SplitCsv(zonesCsv)) {
    if (tok == "all" || tok == "zones") {
      cfg.includeResidential = true;
      cfg.includeCommercial = true;
      cfg.includeIndustrial = true;
      break;
    }
    if (tok == "res" || tok == "residential") cfg.includeResidential = true;
    if (tok == "com" || tok == "commercial") cfg.includeCommercial = true;
    if (tok == "ind" || tok == "industrial") cfg.includeIndustrial = true;
  }
  if (!cfg.includeResidential && !cfg.includeCommercial && !cfg.includeIndustrial) {
    // Default to all if user provided empty/unknown tokens.
    cfg.includeResidential = true;
    cfg.includeCommercial = true;
    cfg.includeIndustrial = true;
  }

  const int targetCostMilli = (targetSteps > 0) ? targetSteps * 1000 : 0;
  cfg.targetCostMilli = targetCostMilli;

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

  simCfg.requireOutsideConnection = requireOutside;

  // Simulate if requested (populates occupancy), otherwise just refresh stats.
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }
  if (days == 0) {
    sim.refreshDerivedStats(world);
  }

  // Precompute outside-connection mask for analysis.
  std::vector<std::uint8_t> roadToEdge;
  std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (requireOutside) {
    roadToEdge.resize(static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height()));
    ComputeRoadsConnectedToEdge(world, roadToEdge);
    roadToEdgePtr = &roadToEdge;
  }

  // Baseline park sources.
  const std::vector<int> sourcesBefore = CollectParkSources(world, roadToEdgePtr);

  // Plan placements.
  const ParkOptimizerResult plan = SuggestParkPlacements(world, cfg, nullptr, roadToEdgePtr);

  // After sources: add the suggested access-road tiles.
  std::vector<int> sourcesAfter = sourcesBefore;
  sourcesAfter.reserve(sourcesAfter.size() + plan.placements.size());
  for (const ParkPlacement& p : plan.placements) {
    sourcesAfter.push_back(p.accessRoad.y * world.width() + p.accessRoad.x);
  }
  std::sort(sourcesAfter.begin(), sourcesAfter.end());
  sourcesAfter.erase(std::unique(sourcesAfter.begin(), sourcesAfter.end()), sourcesAfter.end());

  const RoadIsochroneField roadBefore = BuildParkRoadField(world, sourcesBefore, cfg, roadToEdgePtr);
  const RoadIsochroneField roadAfter = BuildParkRoadField(world, sourcesAfter, cfg, roadToEdgePtr);

  TileAccessCostConfig tcfg;
  tcfg.includeRoadTiles = false;
  tcfg.includeZones = true;
  tcfg.includeNonZonesAdjacentToRoad = false;
  tcfg.useZoneAccessMap = true;
  tcfg.accessStepCostMilli = 0;

  const std::vector<int> costBefore = BuildTileAccessCostField(world, roadBefore, tcfg, roadToEdgePtr);
  const std::vector<int> costAfter = BuildTileAccessCostField(world, roadAfter, tcfg, roadToEdgePtr);

  const CostSummary summaryBefore = SummarizeCosts(world, cfg, costBefore, targetCostMilli);
  const CostSummary summaryAfter = SummarizeCosts(world, cfg, costAfter, targetCostMilli);

  std::cout << "ParkOpt summary\n";
  std::cout << "  world: " << world.width() << "x" << world.height() << "  day=" << world.stats().day << "\n";
  std::cout << "  zones: res=" << (cfg.includeResidential ? 1 : 0)
            << " com=" << (cfg.includeCommercial ? 1 : 0)
            << " ind=" << (cfg.includeIndustrial ? 1 : 0)
            << "  demand_mode=" << ((cfg.demandMode == ParkDemandMode::Tiles) ? "tiles" : "occupants") << "\n";
  std::cout << "  existing_parks=" << plan.existingParks << "  proposed_parks=" << plan.placements.size() << "\n";
  std::cout << "  demand_total_weight=" << plan.totalDemandWeight << "\n";
  std::cout << "  before: reachable=" << summaryBefore.reachableFrac()
            << " avg=" << summaryBefore.avgCostSteps
            << " p95=" << summaryBefore.p95CostSteps
            << " max=" << summaryBefore.maxCostSteps;
  if (targetCostMilli > 0) {
    std::cout << " within_target=" << summaryBefore.withinTargetFrac();
  }
  std::cout << "\n";
  std::cout << "  after:  reachable=" << summaryAfter.reachableFrac()
            << " avg=" << summaryAfter.avgCostSteps
            << " p95=" << summaryAfter.p95CostSteps
            << " max=" << summaryAfter.maxCostSteps;
  if (targetCostMilli > 0) {
    std::cout << " within_target=" << summaryAfter.withinTargetFrac();
  }
  std::cout << "\n";

  if (!outCsv.empty()) {
    if (!EnsureParentDir(outCsv)) {
      std::cerr << "Failed to create parent dirs for: " << outCsv << "\n";
      return 2;
    }
    std::string err;
    if (!WriteCsv(outCsv, plan, err)) {
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "wrote csv -> " << outCsv << "\n";
  }

  if (!outJson.empty()) {
    if (!EnsureParentDir(outJson)) {
      std::cerr << "Failed to create parent dirs for: " << outJson << "\n";
      return 2;
    }
    std::string err;
    if (!WriteJson(outJson, world, plan, summaryBefore, summaryAfter, targetCostMilli, err)) {
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "wrote json -> " << outJson << "\n";
  }

  if (!outAnnotate.empty()) {
    if (!EnsureParentDir(outAnnotate)) {
      std::cerr << "Failed to create parent dirs for: " << outAnnotate << "\n";
      return 2;
    }
    std::string err;
    if (!WriteAnnotatedOverlay(outAnnotate, world, plan, err)) {
      std::cerr << "annotate export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote annotate -> " << outAnnotate << "\n";
  }

  if (!outHeatBefore.empty()) {
    if (!EnsureParentDir(outHeatBefore)) {
      std::cerr << "Failed to create parent dirs for: " << outHeatBefore << "\n";
      return 2;
    }
    std::string err;
    if (!WriteCostHeatmapOverlay(outHeatBefore, world, cfg, costBefore, heatScale, heatMaxSteps, &plan, err)) {
      std::cerr << "heat-before export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote heat-before -> " << outHeatBefore << "\n";
  }

  if (!outHeatAfter.empty()) {
    if (!EnsureParentDir(outHeatAfter)) {
      std::cerr << "Failed to create parent dirs for: " << outHeatAfter << "\n";
      return 2;
    }
    std::string err;
    if (!WriteCostHeatmapOverlay(outHeatAfter, world, cfg, costAfter, heatScale, heatMaxSteps, &plan, err)) {
      std::cerr << "heat-after export failed: " << err << "\n";
      return 2;
    }
    std::cout << "wrote heat-after -> " << outHeatAfter << "\n";
  }

  if (!outSave.empty()) {
    if (!EnsureParentDir(outSave)) {
      std::cerr << "Failed to create parent dirs for: " << outSave << "\n";
      return 2;
    }

    World w2 = world;
    ApplyParkPlacements(w2, plan.placements);

    // Refresh derived stats so the saved HUD numbers match the new world.
    Simulator sim2(simCfg);
    sim2.refreshDerivedStats(w2);

    std::string err;
    if (!SaveWorldBinary(w2, procCfg, simCfg, outSave, err)) {
      std::cerr << "Failed to write save: " << outSave << "\n";
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "wrote save -> " << outSave << "\n";
  }

  // Print top placements for convenience.
  const int top = std::min<int>(10, static_cast<int>(plan.placements.size()));
  if (top > 0) {
    std::cout << "Top placements\n";
    for (int i = 0; i < top; ++i) {
      const ParkPlacement& p = plan.placements[static_cast<std::size_t>(i)];
      const double costSteps = (p.costMilliBefore >= 0) ? (static_cast<double>(p.costMilliBefore) / 1000.0) : -1.0;
      std::cout << "  #" << (i + 1)
                << " park@(" << p.parkTile.x << "," << p.parkTile.y << ")"
                << " road@(" << p.accessRoad.x << "," << p.accessRoad.y << ")"
                << " demand=" << p.demandWeight
                << " cost_before_steps=" << costSteps
                << " score=" << p.score << "\n";
    }
  }

  return 0;
}
