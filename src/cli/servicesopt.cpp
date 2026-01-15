#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/ServiceOptimizer.hpp"
#include "isocity/Services.hpp"
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

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  float v = std::strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = v;
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

bool ParseDemandMode(const std::string& s, ServiceDemandMode& out)
{
  if (s == "tiles" || s == "tile") {
    out = ServiceDemandMode::Tiles;
    return true;
  }
  if (s == "occ" || s == "occupants" || s == "population") {
    out = ServiceDemandMode::Occupants;
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

std::string ToLower(std::string s)
{
  for (char& c : s) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return s;
}

bool ParseServiceType(const std::string& s, ServiceType& out)
{
  if (s == "education" || s == "edu" || s == "school") {
    out = ServiceType::Education;
    return true;
  }
  if (s == "health" || s == "clinic" || s == "hospital") {
    out = ServiceType::Health;
    return true;
  }
  if (s == "safety" || s == "police" || s == "fire") {
    out = ServiceType::Safety;
    return true;
  }
  return false;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_servicesopt (headless civic service placement optimizer)\n\n"
      << "Suggests new Education/Health/Safety facilities to increase demand-weighted service\n"
      << "satisfaction using a greedy, capacity-aware E2SFCA heuristic on the road network.\n\n"
      << "Usage:\n"
      << "  proc_isocity_servicesopt [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                        [--days <N>] [--require-outside <0|1>]\n"
      << "                        [--type <education|health|safety|all>]\n"
      << "                        [--add <N>] [--level <1..3>]\n"
      << "                        [--radius <streetSteps>] [--weight-mode <steps|time>]\n"
      << "                        [--demand-mode <tiles|occupants>] [--zones <csv>]\n"
      << "                        [--candidate-limit <N>] [--min-sep <streetSteps>]\n"
      << "                        [--target-access <float>]\n"
      << "                        [--json <out.json>] [--csv <out.csv>]\n"
      << "                        [--heat-before <out.png|out.ppm>] [--heat-after <out.png|out.ppm>]\n"
      << "                        [--scale <N>]\n\n"
      << "Examples:\n"
      << "  # Generate a world, simulate 120 days, and propose 8 schools\n"
      << "  ./build/proc_isocity_servicesopt --seed 1 --size 128x128 --days 120 --type education --add 8 \\\n"
      << "    --json edu.json --heat-after edu.png --scale 4\n\n"
      << "  # Improve all 3 services in one run (writes per-type placement list in JSON)\n"
      << "  ./build/proc_isocity_servicesopt --load save.bin --type all --add 6 --level 2 --json services.json\n";
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

inline void BlendPixel(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b,
                       std::uint8_t hr, std::uint8_t hg, std::uint8_t hb)
{
  // Blend 2/3 heatmap, 1/3 base.
  r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
  g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
  b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
}

bool WritePpmOrPng(const std::string& path, const PpmImage& img, std::string& outErr)
{
  const std::string lower = ToLower(path);
  if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".png") {
    return WritePng(path, img, outErr);
  }
  return WritePpm(path, img, outErr);
}

const std::vector<float>* PickField(const ServicesResult& r, ServiceType t)
{
  if (t == ServiceType::Education) return &r.education;
  if (t == ServiceType::Health) return &r.health;
  if (t == ServiceType::Safety) return &r.safety;
  return &r.overall;
}

PpmImage RenderServiceHeatmap(const World& world,
                              const std::vector<float>& field,
                              const std::vector<ServiceFacility>& facilities,
                              ServiceType t,
                              bool markAllFacilities,
                              int scale)
{
  PpmImage base = RenderPpmLayer(world, ExportLayer::Overlay);
  const int w = world.width();
  const int h = world.height();
  if (field.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const float sat = field[i];
        std::uint8_t hr = 0, hg = 0, hb = 0;
        HeatRampRedYellowGreen(sat, hr, hg, hb);
        std::uint8_t& r = base.rgb[i * 3u + 0u];
        std::uint8_t& g = base.rgb[i * 3u + 1u];
        std::uint8_t& b = base.rgb[i * 3u + 2u];
        BlendPixel(r, g, b, hr, hg, hb);
      }
    }
  }

  // Mark active facilities of this type.
  for (const auto& f : facilities) {
    if (!f.enabled) continue;
    if (!markAllFacilities && f.type != t) continue;
    if (!world.inBounds(f.tile.x, f.tile.y)) continue;
    const std::size_t i = static_cast<std::size_t>(f.tile.y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(f.tile.x);
    if (i * 3u + 2u >= base.rgb.size()) continue;
    base.rgb[i * 3u + 0u] = 255;
    base.rgb[i * 3u + 1u] = 255;
    base.rgb[i * 3u + 2u] = 255;
  }

  if (scale > 1) {
    base = ScaleNearest(base, scale);
  }
  return base;
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128, h = 128;
  int days = 120;

  bool requireOutside = true;
  std::string typeStr = "education";
  ServiceType type = ServiceType::Education;
  bool typeAll = false;

  int addN = 8;
  int level = 1;

  int radiusSteps = 18;
  IsochroneWeightMode weightMode = IsochroneWeightMode::TravelTime;
  ServiceDemandMode demandMode = ServiceDemandMode::Occupants;
  std::string zonesCsv = "res,com,ind";
  int candidateLimit = 700;
  int minSepSteps = 0;
  float targetAccess = 1.0f;

  std::string outJson;
  std::string outCsv;
  std::string heatBefore;
  std::string heatAfter;
  int scale = 4;

  // Parse args.
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--load") {
      if (!requireValue(i, loadPath, argc, argv)) { std::cerr << "--load needs a value\n"; return 2; }
    } else if (a == "--seed") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseU64(v, &seed)) { std::cerr << "Bad --seed\n"; return 2; }
    } else if (a == "--size") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseSize(v, &w, &h)) { std::cerr << "Bad --size\n"; return 2; }
    } else if (a == "--days") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &days)) { std::cerr << "Bad --days\n"; return 2; }
    } else if (a == "--require-outside") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseBool01(v, &requireOutside)) { std::cerr << "Bad --require-outside\n"; return 2; }
    } else if (a == "--type") {
      if (!requireValue(i, typeStr, argc, argv)) { std::cerr << "--type needs a value\n"; return 2; }
      if (typeStr == "all") {
        typeAll = true;
      } else {
        if (!ParseServiceType(typeStr, type)) { std::cerr << "Bad --type: " << typeStr << "\n"; return 2; }
      }
    } else if (a == "--add") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &addN)) { std::cerr << "Bad --add\n"; return 2; }
    } else if (a == "--level") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &level)) { std::cerr << "Bad --level\n"; return 2; }
    } else if (a == "--radius") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &radiusSteps)) { std::cerr << "Bad --radius\n"; return 2; }
    } else if (a == "--weight-mode") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseWeightMode(v, weightMode)) { std::cerr << "Bad --weight-mode\n"; return 2; }
    } else if (a == "--demand-mode") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseDemandMode(v, demandMode)) { std::cerr << "Bad --demand-mode\n"; return 2; }
    } else if (a == "--zones") {
      if (!requireValue(i, zonesCsv, argc, argv)) { std::cerr << "Bad --zones\n"; return 2; }
    } else if (a == "--candidate-limit") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &candidateLimit)) { std::cerr << "Bad --candidate-limit\n"; return 2; }
    } else if (a == "--min-sep") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &minSepSteps)) { std::cerr << "Bad --min-sep\n"; return 2; }
    } else if (a == "--target-access") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseF32(v, &targetAccess)) { std::cerr << "Bad --target-access\n"; return 2; }
    } else if (a == "--json") {
      if (!requireValue(i, outJson, argc, argv)) { std::cerr << "Bad --json\n"; return 2; }
    } else if (a == "--csv") {
      if (!requireValue(i, outCsv, argc, argv)) { std::cerr << "Bad --csv\n"; return 2; }
    } else if (a == "--heat-before") {
      if (!requireValue(i, heatBefore, argc, argv)) { std::cerr << "Bad --heat-before\n"; return 2; }
    } else if (a == "--heat-after") {
      if (!requireValue(i, heatAfter, argc, argv)) { std::cerr << "Bad --heat-after\n"; return 2; }
    } else if (a == "--scale") {
      std::string v; if (!requireValue(i, v, argc, argv) || !ParseI32(v, &scale)) { std::cerr << "Bad --scale\n"; return 2; }
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      return 2;
    }
  }

  if (addN < 0) addN = 0;
  if (level < 1) level = 1;
  if (level > 3) level = 3;
  if (radiusSteps < 0) radiusSteps = 0;
  if (candidateLimit < 1) candidateLimit = 1;
  if (scale < 1) scale = 1;

  ServicesModelSettings modelCfg;
  modelCfg.requireOutsideConnection = requireOutside;
  modelCfg.weightMode = weightMode;
  modelCfg.catchmentRadiusSteps = radiusSteps;
  modelCfg.demandMode = demandMode;
  modelCfg.targetAccess = targetAccess;

  // Zones mask.
  modelCfg.demandResidential = false;
  modelCfg.demandCommercial = false;
  modelCfg.demandIndustrial = false;
  for (const std::string& z : SplitCsv(ToLower(zonesCsv))) {
    if (z == "res" || z == "residential") modelCfg.demandResidential = true;
    else if (z == "com" || z == "commercial") modelCfg.demandCommercial = true;
    else if (z == "ind" || z == "industrial") modelCfg.demandIndustrial = true;
  }
  if (!modelCfg.demandResidential && !modelCfg.demandCommercial && !modelCfg.demandIndustrial) {
    modelCfg.demandResidential = modelCfg.demandCommercial = modelCfg.demandIndustrial = true;
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n" << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  simCfg.requireOutsideConnection = requireOutside;

  // Simulate if requested (populates occupants), otherwise just refresh stats.
  Simulator sim(simCfg);
  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }
  if (days == 0) sim.refreshDerivedStats(world);

  // Precompute outside-connection mask + zone access (reused by multiple runs).
  std::vector<std::uint8_t> roadToEdge;
  std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (requireOutside) {
    roadToEdge.resize(static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height()));
    ComputeRoadsConnectedToEdge(world, roadToEdge);
    roadToEdgePtr = &roadToEdge;
  }

  const ZoneAccessMap zam = BuildZoneAccessMap(world, roadToEdgePtr);

  const ServicesResult before = ComputeServices(world, modelCfg, {}, &zam, roadToEdgePtr);

  std::vector<ServiceFacility> facilities; // aggregate
  std::vector<ServiceOptimizerResult> plans;

  auto runOne = [&](ServiceType st) {
    ServiceOptimizerConfig ocfg;
    ocfg.modelCfg = modelCfg;
    ocfg.type = st;
    ocfg.facilitiesToAdd = addN;
    ocfg.facilityLevel = static_cast<std::uint8_t>(level);
    ocfg.candidateLimit = candidateLimit;
    ocfg.minSeparationMilli = std::max(0, minSepSteps) * 1000;
    ocfg.requireEmptyLand = true;
    ocfg.requireStableAccessRoad = true;

    const ServiceOptimizerResult plan = SuggestServiceFacilities(world, ocfg, facilities, &zam, roadToEdgePtr);
    plans.push_back(plan);
    const auto added = FacilitiesFromPlacements(plan.placements);
    facilities.insert(facilities.end(), added.begin(), added.end());
  };

  if (typeAll) {
    runOne(ServiceType::Education);
    runOne(ServiceType::Health);
    runOne(ServiceType::Safety);
  } else {
    runOne(type);
  }

  const ServicesResult after = ComputeServices(world, modelCfg, facilities, &zam, roadToEdgePtr);

  std::cout << "ServicesOpt summary\n";
  std::cout << "  world: " << world.width() << "x" << world.height() << "  day=" << world.stats().day << "\n";
  std::cout << "  require_outside=" << (requireOutside ? 1 : 0)
            << "  radius_steps=" << radiusSteps
            << "  weight_mode=" << ((weightMode == IsochroneWeightMode::TravelTime) ? "time" : "steps")
            << "  demand_mode=" << ((demandMode == ServiceDemandMode::Occupants) ? "occupants" : "tiles")
            << "\n";

  std::cout << "  before: edu=" << before.educationSatisfaction
            << " health=" << before.healthSatisfaction
            << " safety=" << before.safetySatisfaction
            << " overall=" << before.overallSatisfaction
            << " maintenance_per_day=" << before.maintenanceCostPerDay
            << "\n";
  std::cout << "  after:  edu=" << after.educationSatisfaction
            << " health=" << after.healthSatisfaction
            << " safety=" << after.safetySatisfaction
            << " overall=" << after.overallSatisfaction
            << " maintenance_per_day=" << after.maintenanceCostPerDay
            << "\n";

  // JSON output.
  if (!outJson.empty()) {
    {
      const auto parent = std::filesystem::path(outJson).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
    }
    std::ofstream os(outJson);
    if (!os) {
      std::cerr << "Failed to open json: " << outJson << "\n";
      return 2;
    }
    JsonWriter jw(os, JsonWriteOptions{true, 2, false});
    jw.beginObject();
    jw.key("world");
    jw.beginObject();
    jw.key("w"); jw.intValue(world.width());
    jw.key("h"); jw.intValue(world.height());
    jw.key("day"); jw.intValue(world.stats().day);
    jw.endObject();

    jw.key("config");
    jw.beginObject();
    jw.key("require_outside"); jw.boolValue(requireOutside);
    jw.key("radius_steps"); jw.intValue(radiusSteps);
    jw.key("weight_mode"); jw.stringValue((weightMode == IsochroneWeightMode::TravelTime) ? "time" : "steps");
    jw.key("demand_mode"); jw.stringValue((demandMode == ServiceDemandMode::Occupants) ? "occupants" : "tiles");
    jw.key("zones"); jw.stringValue(zonesCsv);
    jw.key("target_access"); jw.numberValue(targetAccess);
    jw.key("add"); jw.intValue(addN);
    jw.key("level"); jw.intValue(level);
    jw.key("candidate_limit"); jw.intValue(candidateLimit);
    jw.key("min_sep_steps"); jw.intValue(minSepSteps);
    jw.endObject();

    auto writeSummary = [&](const char* k, const ServicesResult& r) {
      jw.key(k);
      jw.beginObject();
      jw.key("education"); jw.numberValue(r.educationSatisfaction);
      jw.key("health"); jw.numberValue(r.healthSatisfaction);
      jw.key("safety"); jw.numberValue(r.safetySatisfaction);
      jw.key("overall"); jw.numberValue(r.overallSatisfaction);
      jw.key("maintenance_per_day"); jw.intValue(r.maintenanceCostPerDay);
      jw.endObject();
    };
    writeSummary("before", before);
    writeSummary("after", after);

    jw.key("plans");
    jw.beginArray();
    for (const auto& plan : plans) {
      jw.beginObject();
      jw.key("type"); jw.stringValue(ServiceTypeName(plan.cfg.type));
      jw.key("existing_facilities"); jw.intValue(plan.existingFacilities);
      jw.key("proposed_facilities"); jw.intValue(static_cast<int>(plan.placements.size()));

      jw.key("placements");
      jw.beginArray();
      for (const auto& p : plan.placements) {
        jw.beginObject();
        jw.key("x"); jw.intValue(p.facility.tile.x);
        jw.key("y"); jw.intValue(p.facility.tile.y);
        jw.key("access_x"); jw.intValue(p.accessRoad.x);
        jw.key("access_y"); jw.intValue(p.accessRoad.y);
        jw.key("level"); jw.intValue(static_cast<int>(p.facility.level));
        jw.key("marginal_gain"); jw.numberValue(p.marginalGain);
        jw.key("local_demand_sum"); jw.numberValue(p.localDemandSum);
        jw.key("ratio"); jw.numberValue(p.ratio);
        jw.endObject();
      }
      jw.endArray();
      jw.endObject();
    }
    jw.endArray();

    jw.endObject();
    os << "\n";
    if (!jw.ok()) {
      std::cerr << "JSON writer error: " << jw.error() << "\n";
      return 2;
    }
  }

  // CSV output.
  if (!outCsv.empty()) {
    {
      const auto parent = std::filesystem::path(outCsv).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
    }
    std::ofstream os(outCsv);
    if (!os) {
      std::cerr << "Failed to open csv: " << outCsv << "\n";
      return 2;
    }
    os << "type,x,y,access_x,access_y,level,marginal_gain,local_demand_sum,ratio\n";
    for (const auto& plan : plans) {
      for (const auto& p : plan.placements) {
        os << ServiceTypeName(plan.cfg.type) << "," << p.facility.tile.x << "," << p.facility.tile.y
           << "," << p.accessRoad.x << "," << p.accessRoad.y
           << "," << static_cast<int>(p.facility.level)
           << "," << p.marginalGain
           << "," << p.localDemandSum
           << "," << p.ratio
           << "\n";
      }
    }
  }

  // Heatmaps.
  if (!heatBefore.empty()) {
    {
      const auto parent = std::filesystem::path(heatBefore).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
    }

    const std::vector<float>& field = typeAll ? before.overall : *PickField(before, type);
    const ServiceType markType = typeAll ? ServiceType::Education : type;
    {
      PpmImage img = RenderServiceHeatmap(world, field, facilities, markType, typeAll, scale);
      std::string err;
      if (!WritePpmOrPng(heatBefore, img, err)) {
        std::cerr << "Failed to write heat-before: " << err << "\n";
        return 2;
      }
    }
  }
  if (!heatAfter.empty()) {
    {
      const auto parent = std::filesystem::path(heatAfter).parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
    }

    const std::vector<float>& field = typeAll ? after.overall : *PickField(after, type);
    const ServiceType markType = typeAll ? ServiceType::Education : type;
    {
      PpmImage img = RenderServiceHeatmap(world, field, facilities, markType, typeAll, scale);
      std::string err;
      if (!WritePpmOrPng(heatAfter, img, err)) {
        std::cerr << "Failed to write heat-after: " << err << "\n";
        return 2;
      }
    }
  }

  return 0;
}
