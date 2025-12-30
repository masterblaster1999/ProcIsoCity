#include "isocity/Brush.hpp"
#include "isocity/DistrictStats.hpp"
#include "isocity/Districting.hpp"
#include "isocity/Export.hpp"
#include "isocity/FloodFill.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

static std::string Trim(std::string s)
{
  auto isWs = [](unsigned char c) { return std::isspace(c) != 0; };
  while (!s.empty() && isWs(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && isWs(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

static std::vector<std::string> SplitWS(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  for (unsigned char uc : s) {
    const char c = static_cast<char>(uc);
    if (std::isspace(uc)) {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
      continue;
    }
    cur.push_back(c);
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static const char* ToolApplyResultName(ToolApplyResult r)
{
  switch (r) {
  case ToolApplyResult::Applied: return "Applied";
  case ToolApplyResult::Noop: return "Noop";
  case ToolApplyResult::OutOfBounds: return "OutOfBounds";
  case ToolApplyResult::BlockedWater: return "BlockedWater";
  case ToolApplyResult::BlockedNoRoad: return "BlockedNoRoad";
  case ToolApplyResult::BlockedOccupied: return "BlockedOccupied";
  case ToolApplyResult::InsufficientFunds: return "InsufficientFunds";
  default: return "Unknown";
  }
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

static bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const float v = std::strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  *out = v;
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

static bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  const std::string k = ToLower(s);
  if (k == "1" || k == "true" || k == "yes" || k == "on") {
    *out = true;
    return true;
  }
  if (k == "0" || k == "false" || k == "no" || k == "off") {
    *out = false;
    return true;
  }
  return false;
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

static std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static void PrintHelp()
{
  std::cout
      << "proc_isocity_script (headless scenario script runner)\n\n"
      << "Usage:\n"
      << "  proc_isocity_script <script.txt> [--out <summary.json>] [--csv <ticks.csv>] [--quiet]\n\n"
      << "Script format:\n"
      << "  - One command per line. Whitespace separated. # starts a comment.\n\n"
      << "Core commands:\n"
      << "  size WxH\n"
      << "  seed <u64|0x...>\n"
      << "  load <save.bin>\n"
      << "  generate\n"
      << "  save <out.bin>\n"
      << "  money <N>\n"
      << "  tick <N>\n\n"
      << "Editing commands:\n"
      << "  road x y [level]\n"
      << "  road_line x0 y0 x1 y1 [level]\n"
      << "  road_path x0 y0 x1 y1 [level] [allowBridges 0|1] [costModel newtiles|money]\n"
      << "  zone <res|com|ind> x y [level]\n"
      << "  park x y\n"
      << "  bulldoze x y\n"
      << "  district x y <id 0..7>\n"
      << "  district_auto [districts] [fillAllTiles 0|1] [useTravelTime 0|1] [requireOutside 0|1]\n"
      << "  flood <road|park|bulldoze|district|res|com|ind> x y [arg] [includeRoads 0|1]\n"
      << "  fill <road|park|bulldoze|district|res|com|ind> x0 y0 x1 y1 [arg]\n"
      << "  outline <road|park|bulldoze|district|res|com|ind> x0 y0 x1 y1 [arg]\n\n"
      << "Config commands:\n"
      << "  proc <key> <value>                 (terrainScale, waterLevel, sandLevel, hubs, extraConnections, zoneChance, parkChance)\n"
      << "  sim <key> <value>                  (requireOutsideConnection, parkInfluenceRadius, taxes, maintenance, desirability...)\n"
      << "  policy <districtId> <key> <value>  (taxResidentialMult, taxCommercialMult, taxIndustrialMult, roadMaintenanceMult, parkMaintenanceMult)\n"
      << "  traffic_model <key> <value>        (congestionAwareRouting, congestionIterations, congestionAlpha, congestionBeta, congestionCapacityScale, congestionRatioClamp)\n\n"
      << "Artifacts / assertions:\n"
      << "  export_ppm <layer> <out.ppm> [scale]\n"
      << "  export_tiles_csv <out.csv>\n"
      << "  districts_json <out.json>\n"
      << "  hash\n"
      << "  expect_hash <u64|0x...>\n";
}

static bool ApplyZoneTile(World& world, Tool tool, int x, int y, int targetLevel, ToolApplyResult* outFail = nullptr)
{
  targetLevel = std::clamp(targetLevel, 1, 3);

  // Ensure the correct overlay is present (placement or upgrade).
  ToolApplyResult r = world.applyTool(tool, x, y);
  if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
    if (outFail) *outFail = r;
    return false;
  }

  Tile& t = world.at(x, y);
  if (tool == Tool::Residential && t.overlay != Overlay::Residential) return false;
  if (tool == Tool::Commercial && t.overlay != Overlay::Commercial) return false;
  if (tool == Tool::Industrial && t.overlay != Overlay::Industrial) return false;

  // Upgrade until desired level.
  while (static_cast<int>(t.level) < targetLevel) {
    r = world.applyTool(tool, x, y);
    if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
      if (outFail) *outFail = r;
      return false;
    }
  }

  return true;
}

static bool ApplyRoadTile(World& world, int x, int y, int level, ToolApplyResult* outFail = nullptr)
{
  const ToolApplyResult r = world.applyRoad(x, y, level);
  if (outFail) *outFail = r;
  return (r == ToolApplyResult::Applied || r == ToolApplyResult::Noop);
}

static bool ApplyFill(World& world, const std::string& what, Point a, Point b, int arg,
                      Point* outFailP = nullptr, ToolApplyResult* outFailR = nullptr)
{
  const std::string k = ToLower(what);
  const int argOr1 = (arg > 0) ? arg : 1;

  const bool recognized = (k == "road" || k == "park" || k == "bulldoze" || k == "district" || k == "res" ||
                           k == "residential" || k == "com" || k == "commercial" || k == "ind" || k == "industrial");
  if (!recognized) {
    if (outFailP) *outFailP = a;
    if (outFailR) *outFailR = ToolApplyResult::Noop;
    return false;
  }

  bool ok = true;
  Point failP{0, 0};
  ToolApplyResult failR = ToolApplyResult::Noop;

  auto recordFail = [&](Point p, ToolApplyResult r) {
    ok = false;
    failP = p;
    failR = r;
  };

  ForEachRectFilled(a, b, [&](Point p) {
    if (!ok) return;
    if (!world.inBounds(p.x, p.y)) return;

    ToolApplyResult r = ToolApplyResult::Noop;

    if (k == "road") {
      if (!ApplyRoadTile(world, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "park") {
      r = world.applyTool(Tool::Park, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "bulldoze") {
      r = world.applyTool(Tool::Bulldoze, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "district") {
      r = world.applyDistrict(p.x, p.y, arg);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "res" || k == "residential") {
      if (!ApplyZoneTile(world, Tool::Residential, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "com" || k == "commercial") {
      if (!ApplyZoneTile(world, Tool::Commercial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "ind" || k == "industrial") {
      if (!ApplyZoneTile(world, Tool::Industrial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    }
  });

  if (!ok) {
    if (outFailP) *outFailP = failP;
    if (outFailR) *outFailR = failR;
    return false;
  }

  return true;
}

static bool ApplyOutline(World& world, const std::string& what, Point a, Point b, int arg,
                         Point* outFailP = nullptr, ToolApplyResult* outFailR = nullptr)
{
  const std::string k = ToLower(what);
  const int argOr1 = (arg > 0) ? arg : 1;

  const bool recognized = (k == "road" || k == "park" || k == "bulldoze" || k == "district" || k == "res" ||
                           k == "residential" || k == "com" || k == "commercial" || k == "ind" || k == "industrial");
  if (!recognized) {
    if (outFailP) *outFailP = a;
    if (outFailR) *outFailR = ToolApplyResult::Noop;
    return false;
  }

  bool ok = true;
  Point failP{0, 0};
  ToolApplyResult failR = ToolApplyResult::Noop;

  auto recordFail = [&](Point p, ToolApplyResult r) {
    ok = false;
    failP = p;
    failR = r;
  };

  ForEachRectOutline(a, b, [&](Point p) {
    if (!ok) return;
    if (!world.inBounds(p.x, p.y)) return;

    ToolApplyResult r = ToolApplyResult::Noop;

    if (k == "road") {
      if (!ApplyRoadTile(world, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "park") {
      r = world.applyTool(Tool::Park, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "bulldoze") {
      r = world.applyTool(Tool::Bulldoze, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "district") {
      r = world.applyDistrict(p.x, p.y, arg);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "res" || k == "residential") {
      if (!ApplyZoneTile(world, Tool::Residential, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "com" || k == "commercial") {
      if (!ApplyZoneTile(world, Tool::Commercial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "ind" || k == "industrial") {
      if (!ApplyZoneTile(world, Tool::Industrial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    }
  });

  if (!ok) {
    if (outFailP) *outFailP = failP;
    if (outFailR) *outFailR = failR;
    return false;
  }

  return true;
}

static bool ApplyFlood(World& world, const std::string& what, Point start, int arg, bool includeRoadsInLandBlock,
                       Point* outFailP = nullptr, ToolApplyResult* outFailR = nullptr)
{
  const std::string k = ToLower(what);
  const int argOr1 = (arg > 0) ? arg : 1;

  const bool recognized = (k == "road" || k == "park" || k == "bulldoze" || k == "district" || k == "res" ||
                           k == "residential" || k == "com" || k == "commercial" || k == "ind" || k == "industrial");
  if (!recognized) {
    if (outFailP) *outFailP = start;
    if (outFailR) *outFailR = ToolApplyResult::Noop;
    return false;
  }

  if (!world.inBounds(start.x, start.y)) {
    if (outFailP) *outFailP = start;
    if (outFailR) *outFailR = ToolApplyResult::OutOfBounds;
    return false;
  }

  const FloodFillResult region = FloodFillAuto(world, start, includeRoadsInLandBlock);

  bool ok = true;
  Point failP{0, 0};
  ToolApplyResult failR = ToolApplyResult::Noop;

  auto recordFail = [&](Point p, ToolApplyResult r) {
    ok = false;
    failP = p;
    failR = r;
  };

  for (const Point& p : region.tiles) {
    if (!ok) break;
    if (!world.inBounds(p.x, p.y)) continue;

    ToolApplyResult r = ToolApplyResult::Noop;

    if (k == "road") {
      if (!ApplyRoadTile(world, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "park") {
      r = world.applyTool(Tool::Park, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "bulldoze") {
      r = world.applyTool(Tool::Bulldoze, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "district") {
      r = world.applyDistrict(p.x, p.y, arg);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "res" || k == "residential") {
      if (!ApplyZoneTile(world, Tool::Residential, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "com" || k == "commercial") {
      if (!ApplyZoneTile(world, Tool::Commercial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "ind" || k == "industrial") {
      if (!ApplyZoneTile(world, Tool::Industrial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    }
  }

  if (!ok) {
    if (outFailP) *outFailP = failP;
    if (outFailR) *outFailR = failR;
    return false;
  }

  return true;
}

static bool WriteCsvTrace(const std::string& path, const std::vector<Stats>& ticks)
{
  if (path.empty()) return true;

  std::ofstream f(path, std::ios::binary);
  if (!f) return false;

  f << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";

  for (const Stats& s : ticks) {
    f << s.day << ','
      << s.population << ','
      << s.money << ','
      << s.housingCapacity << ','
      << s.jobsCapacity << ','
      << s.jobsCapacityAccessible << ','
      << s.employed << ','
      << s.happiness << ','
      << s.roads << ','
      << s.parks << ','
      << s.avgCommuteTime << ','
      << s.trafficCongestion << ','
      << s.goodsDemand << ','
      << s.goodsDelivered << ','
      << s.goodsSatisfaction << ','
      << s.avgLandValue << ','
      << s.demandResidential
      << '\n';
  }

  return static_cast<bool>(f);
}

static bool WriteJsonSummary(const std::string& outPath, const std::string& scriptPath, const World& world, std::uint64_t hash)
{
  if (outPath.empty()) return true;

  const Stats& s = world.stats();

  auto jsonEscape = [](const std::string& in) -> std::string {
    std::ostringstream oss;
    for (unsigned char uc : in) {
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
  };

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"script\": \"" << jsonEscape(scriptPath) << "\",\n";
  oss << "  \"width\": " << world.width() << ",\n";
  oss << "  \"height\": " << world.height() << ",\n";
  oss << "  \"seed\": " << world.seed() << ",\n";
  oss << "  \"hash\": \"" << HexU64(hash) << "\",\n";
  oss << "  \"stats\": {\n";
  oss << "    \"day\": " << s.day << ",\n";
  oss << "    \"population\": " << s.population << ",\n";
  oss << "    \"money\": " << s.money << ",\n";
  oss << "    \"happiness\": " << s.happiness << "\n";
  oss << "  }\n";
  oss << "}\n";

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

static bool WriteDistrictsJson(const std::string& outPath, const World& world, const SimConfig& simCfg)
{
  if (outPath.empty()) return true;

  // Derived fields for land-value-aware taxes.
  LandValueConfig lvc;
  lvc.requireOutsideConnection = simCfg.requireOutsideConnection;

  std::vector<std::uint8_t> roadToEdge;
  if (simCfg.requireOutsideConnection) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
  }

  const LandValueResult lv = ComputeLandValue(world, lvc, nullptr,
                                              simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

  DistrictStatsResult ds = ComputeDistrictStats(world, simCfg, &lv.value,
                                               simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"total\": {\n";
  oss << "    \"tiles\": " << ds.total.tiles << ",\n";
  oss << "    \"population\": " << ds.total.population << ",\n";
  oss << "    \"jobsCapacityAccessible\": " << ds.total.jobsCapacityAccessible << ",\n";
  oss << "    \"taxRevenue\": " << ds.total.taxRevenue << ",\n";
  oss << "    \"maintenanceCost\": " << ds.total.maintenanceCost << ",\n";
  oss << "    \"net\": " << ds.total.net << "\n";
  oss << "  },\n";
  oss << "  \"districts\": [\n";

  for (int i = 0; i < kDistrictCount; ++i) {
    const DistrictSummary& d = ds.districts[static_cast<std::size_t>(i)];
    oss << "    {\n";
    oss << "      \"id\": " << d.id << ",\n";
    oss << "      \"tiles\": " << d.tiles << ",\n";
    oss << "      \"population\": " << d.population << ",\n";
    oss << "      \"jobsCapacityAccessible\": " << d.jobsCapacityAccessible << ",\n";
    oss << "      \"avgLandValue\": " << d.avgLandValue << ",\n";
    oss << "      \"taxRevenue\": " << d.taxRevenue << ",\n";
    oss << "      \"maintenanceCost\": " << d.maintenanceCost << ",\n";
    oss << "      \"net\": " << d.net << "\n";
    oss << "    }";
    if (i != kDistrictCount - 1) oss << ',';
    oss << "\n";
  }

  oss << "  ]\n";
  oss << "}\n";

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

struct ExecContext {
  std::string scriptPath;

  // Defaults (can be overridden by script before generate()).
  int w = 96;
  int h = 96;
  std::uint64_t seed = 1;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  Simulator sim{SimConfig{}};

  World world;
  bool hasWorld = false;
  bool dirtyDerived = true;

  std::vector<Stats> tickStats;

  bool quiet = false;
};

static bool EnsureWorld(ExecContext& ctx, int lineNo)
{
  if (ctx.hasWorld) return true;
  std::cerr << ctx.scriptPath << ':' << lineNo << ": no world loaded/generated yet (use load/generate)\n";
  return false;
}

static void RefreshIfDirty(ExecContext& ctx)
{
  if (!ctx.hasWorld) return;
  if (!ctx.dirtyDerived) return;
  ctx.sim.config() = ctx.simCfg;
  ctx.sim.refreshDerivedStats(ctx.world);
  ctx.dirtyDerived = false;
}

static bool CmdProc(ExecContext& ctx, const std::vector<std::string>& t, int lineNo)
{
  if (t.size() != 3) {
    std::cerr << ctx.scriptPath << ':' << lineNo << ": proc expects: proc <key> <value>\n";
    return false;
  }

  const std::string key = ToLower(t[1]);
  const std::string val = t[2];

  if (key == "terrainscale") {
    return ParseF32(val, &ctx.procCfg.terrainScale);
  }
  if (key == "waterlevel") {
    return ParseF32(val, &ctx.procCfg.waterLevel);
  }
  if (key == "sandlevel") {
    return ParseF32(val, &ctx.procCfg.sandLevel);
  }
  if (key == "hubs") {
    return ParseI32(val, &ctx.procCfg.hubs);
  }
  if (key == "extraconnections" || key == "extra_connections") {
    return ParseI32(val, &ctx.procCfg.extraConnections);
  }
  if (key == "zonechance" || key == "zone_chance") {
    return ParseF32(val, &ctx.procCfg.zoneChance);
  }
  if (key == "parkchance" || key == "park_chance") {
    return ParseF32(val, &ctx.procCfg.parkChance);
  }

  std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown proc key: " << t[1] << "\n";
  return false;
}

static bool CmdSim(ExecContext& ctx, const std::vector<std::string>& t, int lineNo)
{
  if (t.size() != 3) {
    std::cerr << ctx.scriptPath << ':' << lineNo << ": sim expects: sim <key> <value>\n";
    return false;
  }

  const std::string key = ToLower(t[1]);
  const std::string val = t[2];

  if (key == "tickseconds" || key == "tick_seconds") {
    return ParseF32(val, &ctx.simCfg.tickSeconds);
  }
  if (key == "parkinfluenceradius" || key == "park_influence_radius") {
    return ParseI32(val, &ctx.simCfg.parkInfluenceRadius);
  }
  if (key == "requireoutsideconnection" || key == "require_outside_connection" || key == "require_outside") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.simCfg.requireOutsideConnection = b;
    return true;
  }

  if (key == "taxresidential" || key == "tax_residential" || key == "tax_res") {
    return ParseI32(val, &ctx.simCfg.taxResidential);
  }
  if (key == "taxcommercial" || key == "tax_commercial" || key == "tax_com") {
    return ParseI32(val, &ctx.simCfg.taxCommercial);
  }
  if (key == "taxindustrial" || key == "tax_industrial" || key == "tax_ind") {
    return ParseI32(val, &ctx.simCfg.taxIndustrial);
  }

  if (key == "maintenanceroad" || key == "maintenance_road" || key == "maint_road") {
    return ParseI32(val, &ctx.simCfg.maintenanceRoad);
  }
  if (key == "maintenancepark" || key == "maintenance_park" || key == "maint_park") {
    return ParseI32(val, &ctx.simCfg.maintenancePark);
  }

  if (key == "taxhappinesspercapita" || key == "tax_happiness_per_capita") {
    return ParseF32(val, &ctx.simCfg.taxHappinessPerCapita);
  }

  if (key == "residentialdesirabilityweight" || key == "residential_desirability_weight") {
    return ParseF32(val, &ctx.simCfg.residentialDesirabilityWeight);
  }
  if (key == "commercialdesirabilityweight" || key == "commercial_desirability_weight") {
    return ParseF32(val, &ctx.simCfg.commercialDesirabilityWeight);
  }
  if (key == "industrialdesirabilityweight" || key == "industrial_desirability_weight") {
    return ParseF32(val, &ctx.simCfg.industrialDesirabilityWeight);
  }

  if (key == "districtpoliciesenabled" || key == "district_policies_enabled") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.simCfg.districtPoliciesEnabled = b;
    return true;
  }

  std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown sim key: " << t[1] << "\n";
  return false;
}

static bool CmdPolicy(ExecContext& ctx, const std::vector<std::string>& t, int lineNo)
{
  if (t.size() != 4) {
    std::cerr << ctx.scriptPath << ':' << lineNo << ": policy expects: policy <districtId> <key> <value>\n";
    return false;
  }

  int id = 0;
  if (!ParseI32(t[1], &id)) return false;
  id = std::clamp(id, 0, kDistrictCount - 1);

  const std::string key = ToLower(t[2]);
  const std::string val = t[3];

  DistrictPolicy& p = ctx.simCfg.districtPolicies[static_cast<std::size_t>(id)];

  if (key == "taxresidentialmult" || key == "tax_residential_mult") return ParseF32(val, &p.taxResidentialMult);
  if (key == "taxcommercialmult" || key == "tax_commercial_mult") return ParseF32(val, &p.taxCommercialMult);
  if (key == "taxindustrialmult" || key == "tax_industrial_mult") return ParseF32(val, &p.taxIndustrialMult);
  if (key == "roadmaintenancemult" || key == "road_maintenance_mult") return ParseF32(val, &p.roadMaintenanceMult);
  if (key == "parkmaintenancemult" || key == "park_maintenance_mult") return ParseF32(val, &p.parkMaintenanceMult);

  std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown policy key: " << t[2] << "\n";
  return false;
}

static bool CmdTrafficModel(ExecContext& ctx, const std::vector<std::string>& t, int lineNo)
{
  if (t.size() != 3) {
    std::cerr << ctx.scriptPath << ':' << lineNo << ": traffic_model expects: traffic_model <key> <value>\n";
    return false;
  }

  const std::string key = ToLower(t[1]);
  const std::string val = t[2];

  TrafficModelSettings& tm = ctx.sim.trafficModel();

  if (key == "congestionawarerouting" || key == "congestion_aware_routing") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    tm.congestionAwareRouting = b;
    return true;
  }
  if (key == "congestioniterations" || key == "congestion_iterations") {
    return ParseI32(val, &tm.congestionIterations);
  }
  if (key == "congestionalpha" || key == "congestion_alpha") {
    return ParseF32(val, &tm.congestionAlpha);
  }
  if (key == "congestionbeta" || key == "congestion_beta") {
    return ParseF32(val, &tm.congestionBeta);
  }
  if (key == "congestioncapacityscale" || key == "congestion_capacity_scale") {
    return ParseF32(val, &tm.congestionCapacityScale);
  }
  if (key == "congestionratioclamp" || key == "congestion_ratio_clamp") {
    return ParseF32(val, &tm.congestionRatioClamp);
  }

  std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown traffic_model key: " << t[1] << "\n";
  return false;
}

static bool RunScript(ExecContext& ctx)
{
  std::ifstream f(ctx.scriptPath, std::ios::binary);
  if (!f) {
    std::cerr << "failed to open script: " << ctx.scriptPath << "\n";
    return false;
  }

  std::string line;
  int lineNo = 0;

  while (std::getline(f, line)) {
    lineNo++;

    // Strip comments.
    const std::size_t hashPos = line.find('#');
    if (hashPos != std::string::npos) line = line.substr(0, hashPos);
    line = Trim(line);
    if (line.empty()) continue;

    std::vector<std::string> t = SplitWS(line);
    if (t.empty()) continue;

    const std::string cmd = ToLower(t[0]);

    if (cmd == "size") {
      if (t.size() != 2 || !ParseWxH(t[1], &ctx.w, &ctx.h)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": size expects WxH\n";
        return false;
      }
      continue;
    }

    if (cmd == "seed") {
      if (t.size() != 2 || !ParseU64(t[1], &ctx.seed)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": seed expects u64 (decimal or 0x...)\n";
        return false;
      }
      continue;
    }

    if (cmd == "proc") {
      if (!CmdProc(ctx, t, lineNo)) return false;
      continue;
    }

    if (cmd == "sim") {
      if (!CmdSim(ctx, t, lineNo)) return false;
      // Sim config changes can affect derived stats.
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "policy") {
      if (!CmdPolicy(ctx, t, lineNo)) return false;
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "traffic_model") {
      if (!CmdTrafficModel(ctx, t, lineNo)) return false;
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "generate" || cmd == "gen") {
      ctx.world = GenerateWorld(ctx.w, ctx.h, ctx.seed, ctx.procCfg);
      ctx.hasWorld = true;
      ctx.sim.config() = ctx.simCfg;
      ctx.sim.resetTimer();
      ctx.dirtyDerived = true;
      RefreshIfDirty(ctx);
      if (!ctx.quiet) {
        std::cout << "generated " << ctx.w << "x" << ctx.h << " seed=" << ctx.seed << "\n";
      }
      continue;
    }

    if (cmd == "load") {
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": load expects: load <save.bin>\n";
        return false;
      }
      ProcGenConfig pc{};
      SimConfig sc{};
      std::string err;
      if (!LoadWorldBinary(ctx.world, pc, sc, t[1], err)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": load failed: " << err << "\n";
        return false;
      }
      ctx.procCfg = pc;
      ctx.simCfg = sc;
      ctx.sim.config() = ctx.simCfg;
      ctx.sim.resetTimer();
      ctx.hasWorld = true;
      ctx.dirtyDerived = true;
      RefreshIfDirty(ctx);
      if (!ctx.quiet) {
        std::cout << "loaded " << t[1] << " (" << ctx.world.width() << "x" << ctx.world.height() << ")\n";
      }
      continue;
    }

    if (cmd == "save") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": save expects: save <out.bin>\n";
        return false;
      }
      std::string err;
      if (!SaveWorldBinary(ctx.world, ctx.procCfg, ctx.simCfg, t[1], err)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": save failed: " << err << "\n";
        return false;
      }
      if (!ctx.quiet) {
        std::cout << "saved " << t[1] << "\n";
      }
      continue;
    }

    if (cmd == "money") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": money expects: money <N>\n";
        return false;
      }
      int v = 0;
      if (!ParseI32(t[1], &v)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": money expects integer\n";
        return false;
      }
      ctx.world.stats().money = v;
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "tick") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": tick expects: tick <N>\n";
        return false;
      }
      int n = 0;
      if (!ParseI32(t[1], &n) || n < 0) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": tick expects non-negative integer\n";
        return false;
      }

      ctx.sim.config() = ctx.simCfg;
      for (int i = 0; i < n; ++i) {
        ctx.sim.stepOnce(ctx.world);
        ctx.tickStats.push_back(ctx.world.stats());
      }
      ctx.dirtyDerived = false;
      continue;
    }

    if (cmd == "road") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 3 && t.size() != 4) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road expects: road x y [level]\n";
        return false;
      }
      int x = 0, y = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y)) return false;
      int level = 1;
      if (t.size() == 4 && !ParseI32(t[3], &level)) return false;
      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyRoadTile(ctx.world, x, y, level, &r)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road failed at " << x << ',' << y
                  << " (" << ToolApplyResultName(r) << ")\n";
        return false;
      }
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "road_line") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 5 && t.size() != 6) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road_line expects: road_line x0 y0 x1 y1 [level]\n";
        return false;
      }
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[1], &x0) || !ParseI32(t[2], &y0) || !ParseI32(t[3], &x1) || !ParseI32(t[4], &y1)) return false;
      int level = 1;
      if (t.size() == 6 && !ParseI32(t[5], &level)) return false;

      bool ok = true;
      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      ForEachLinePoint(Point{x0, y0}, Point{x1, y1}, [&](Point p) {
        if (!ok) return;
        if (!ctx.world.inBounds(p.x, p.y)) {
          ok = false;
          failP = p;
          failR = ToolApplyResult::OutOfBounds;
          return;
        }
        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyRoadTile(ctx.world, p.x, p.y, level, &r)) {
          ok = false;
          failP = p;
          failR = r;
        }
      });

      if (!ok) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road_line failed at " << failP.x << ',' << failP.y
                  << " (" << ToolApplyResultName(failR) << ")\n";
        return false;
      }

      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "road_path") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() < 5 || t.size() > 8) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road_path expects: road_path x0 y0 x1 y1 [level] [allowBridges 0|1] [costModel newtiles|money]\n";
        return false;
      }

      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[1], &x0) || !ParseI32(t[2], &y0) || !ParseI32(t[3], &x1) || !ParseI32(t[4], &y1)) return false;

      int level = 1;
      if (t.size() >= 6 && !ParseI32(t[5], &level)) return false;

      bool allowBridges = false;
      if (t.size() >= 7 && !ParseBool01(t[6], &allowBridges)) return false;

      RoadBuildPathConfig cfg;
      cfg.targetLevel = level;
      cfg.allowBridges = allowBridges;
      cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;

      if (t.size() >= 8) {
        const std::string cm = ToLower(t[7]);
        if (cm == "newtiles" || cm == "new_tiles") {
          cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;
        } else if (cm == "money") {
          cfg.costModel = RoadBuildPathConfig::CostModel::Money;
        } else {
          std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown costModel: " << t[7] << "\n";
          return false;
        }
      }

      std::vector<Point> path;
      int primaryCost = 0;
      if (!FindRoadBuildPath(ctx.world, Point{x0, y0}, Point{x1, y1}, path, &primaryCost, cfg)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road_path: no valid path\n";
        return false;
      }

      bool ok = true;
      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      for (const Point& p : path) {
        if (!ctx.world.inBounds(p.x, p.y)) {
          ok = false;
          failP = p;
          failR = ToolApplyResult::OutOfBounds;
          break;
        }
        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyRoadTile(ctx.world, p.x, p.y, level, &r)) {
          ok = false;
          failP = p;
          failR = r;
          break;
        }
      }

      if (!ok) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": road_path failed at " << failP.x << ',' << failP.y
                  << " (" << ToolApplyResultName(failR) << ")\n";
        return false;
      }

      if (!ctx.quiet) {
        std::cout << "road_path: " << path.size() << " tiles, estCost=" << primaryCost << "\n";
      }

      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "zone") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 4 && t.size() != 5) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": zone expects: zone <res|com|ind> x y [level]\n";
        return false;
      }

      const std::string which = ToLower(t[1]);
      Tool tool = Tool::Residential;
      if (which == "res" || which == "residential") tool = Tool::Residential;
      else if (which == "com" || which == "commercial") tool = Tool::Commercial;
      else if (which == "ind" || which == "industrial") tool = Tool::Industrial;
      else {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown zone type: " << t[1] << "\n";
        return false;
      }

      int x = 0, y = 0;
      if (!ParseI32(t[2], &x) || !ParseI32(t[3], &y)) return false;

      int lvl = 1;
      if (t.size() == 5 && !ParseI32(t[4], &lvl)) return false;

      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyZoneTile(ctx.world, tool, x, y, lvl, &r)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": zone failed at " << x << ',' << y
                  << " (" << ToolApplyResultName(r) << ")\n";
        return false;
      }

      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "park") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 3) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": park expects: park x y\n";
        return false;
      }
      int x = 0, y = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y)) return false;
      const ToolApplyResult r = ctx.world.applyTool(Tool::Park, x, y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": park failed at " << x << ',' << y
                  << " (" << ToolApplyResultName(r) << ")\n";
        return false;
      }
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "bulldoze") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 3) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": bulldoze expects: bulldoze x y\n";
        return false;
      }
      int x = 0, y = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y)) return false;
      const ToolApplyResult r = ctx.world.applyTool(Tool::Bulldoze, x, y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": bulldoze failed at " << x << ',' << y
                  << " (" << ToolApplyResultName(r) << ")\n";
        return false;
      }
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "district") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 4) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": district expects: district x y id\n";
        return false;
      }
      int x = 0, y = 0, id = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y) || !ParseI32(t[3], &id)) return false;
      const ToolApplyResult r = ctx.world.applyDistrict(x, y, id);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": district failed at " << x << ',' << y
                  << " (" << ToolApplyResultName(r) << ")\n";
        return false;
      }
      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "district_auto") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() < 1 || t.size() > 5) {
        std::cerr << ctx.scriptPath << ':' << lineNo
                  << ": district_auto expects: district_auto [districts] [fillAllTiles 0|1] [useTravelTime 0|1] [requireOutside 0|1]\n";
        return false;
      }

      AutoDistrictConfig cfg;
      cfg.districts = kDistrictCount;
      cfg.fillAllTiles = true;
      cfg.useTravelTime = true;
      cfg.requireOutsideConnection = false;

      int districts = cfg.districts;
      if (t.size() >= 2 && !ParseI32(t[1], &districts)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": district_auto: invalid districts\n";
        return false;
      }
      cfg.districts = districts;

      bool fillAll = cfg.fillAllTiles;
      if (t.size() >= 3 && !ParseBool01(t[2], &fillAll)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": district_auto: invalid fillAllTiles (use 0|1)\n";
        return false;
      }
      cfg.fillAllTiles = fillAll;

      bool useTT = cfg.useTravelTime;
      if (t.size() >= 4 && !ParseBool01(t[3], &useTT)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": district_auto: invalid useTravelTime (use 0|1)\n";
        return false;
      }
      cfg.useTravelTime = useTT;

      bool reqOut = cfg.requireOutsideConnection;
      if (t.size() >= 5 && !ParseBool01(t[4], &reqOut)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": district_auto: invalid requireOutside (use 0|1)\n";
        return false;
      }
      cfg.requireOutsideConnection = reqOut;

      const AutoDistrictResult r = AutoAssignDistricts(ctx.world, cfg);
      if (!ctx.quiet) {
        std::cout << "district_auto: requested=" << r.districtsRequested
                  << " used=" << r.districtsUsed
                  << " seeds=" << r.seedRoadIdx.size() << "\n";
      }

      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "flood") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() < 4 || t.size() > 6) {
        std::cerr << ctx.scriptPath << ':' << lineNo
                  << ": flood expects: flood <road|park|bulldoze|district|res|com|ind> x y [arg] [includeRoads 0|1]\n";
        return false;
      }

      const std::string what = t[1];
      const std::string whatLower = ToLower(what);

      int x = 0, y = 0;
      if (!ParseI32(t[2], &x) || !ParseI32(t[3], &y)) return false;

      int arg = 0;
      bool includeRoads = false;

      const bool isParkOrBulldoze = (whatLower == "park" || whatLower == "bulldoze");
      const bool isDistrict = (whatLower == "district");

      if (isParkOrBulldoze) {
        // No arg. Optional includeRoads.
        if (t.size() >= 5 && !ParseBool01(t[4], &includeRoads)) {
          std::cerr << ctx.scriptPath << ':' << lineNo << ": flood: invalid includeRoads (use 0|1)\n";
          return false;
        }
        if (t.size() == 6) {
          std::cerr << ctx.scriptPath << ':' << lineNo << ": flood: too many arguments for " << what << "\n";
          return false;
        }
      } else {
        // Arg is optional for road/zones, required for district.
        if (isDistrict && t.size() < 5) {
          std::cerr << ctx.scriptPath << ':' << lineNo << ": flood district expects: flood district x y <id> [includeRoads 0|1]\n";
          return false;
        }

        if (t.size() >= 5 && !ParseI32(t[4], &arg)) return false;
        if (t.size() == 6 && !ParseBool01(t[5], &includeRoads)) {
          std::cerr << ctx.scriptPath << ':' << lineNo << ": flood: invalid includeRoads (use 0|1)\n";
          return false;
        }
      }

      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      const bool ok = ApplyFlood(ctx.world, what, Point{x, y}, arg, includeRoads, &failP, &failR);
      if (!ok) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": flood failed at " << failP.x << ',' << failP.y
                  << " (" << ToolApplyResultName(failR) << ")\n";
        return false;
      }

      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "fill" || cmd == "outline") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() < 6 || t.size() > 7) {
        std::cerr << ctx.scriptPath << ':' << lineNo
                  << ": " << cmd << " expects: " << cmd << " <tool> x0 y0 x1 y1 [arg]\n";
        return false;
      }

      const std::string what = t[1];
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[2], &x0) || !ParseI32(t[3], &y0) || !ParseI32(t[4], &x1) || !ParseI32(t[5], &y1)) return false;

      int arg = 0;
      if (t.size() == 7 && !ParseI32(t[6], &arg)) return false;

      const std::string whatLower = ToLower(what);
      const bool recognized = (whatLower == "road" || whatLower == "park" || whatLower == "bulldoze" ||
                               whatLower == "district" || whatLower == "res" || whatLower == "residential" ||
                               whatLower == "com" || whatLower == "commercial" || whatLower == "ind" ||
                               whatLower == "industrial");
      if (!recognized) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown tool for " << cmd << ": " << what << "\n";
        return false;
      }

      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      const bool ok = (cmd == "fill") ? ApplyFill(ctx.world, what, Point{x0, y0}, Point{x1, y1}, arg, &failP, &failR)
                                       : ApplyOutline(ctx.world, what, Point{x0, y0}, Point{x1, y1}, arg, &failP, &failR);
      if (!ok) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": " << cmd << " failed at " << failP.x << ',' << failP.y
                  << " (" << ToolApplyResultName(failR) << ")\n";
        return false;
      }

      ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "export_ppm") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 3 && t.size() != 4) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": export_ppm expects: export_ppm <layer> <out.ppm> [scale]\n";
        return false;
      }

      ExportLayer layer = ExportLayer::Overlay;
      if (!ParseExportLayer(t[1], layer)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown export layer: " << t[1] << "\n";
        return false;
      }

      int scale = 1;
      if (t.size() == 4 && (!ParseI32(t[3], &scale) || scale <= 0)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": scale must be > 0\n";
        return false;
      }

      RefreshIfDirty(ctx);

      // Compute derived fields on demand.
      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
      if (ctx.simCfg.requireOutsideConnection) {
        ComputeRoadsConnectedToEdge(ctx.world, roadToEdge);
        roadToEdgePtr = &roadToEdge;
      }

      TrafficResult traffic;
      GoodsResult goods;
      LandValueResult lv;

      if (layer == ExportLayer::Traffic || layer == ExportLayer::LandValue ||
          layer == ExportLayer::GoodsTraffic || layer == ExportLayer::GoodsFill) {
        TrafficConfig tc;
        tc.requireOutsideConnection = ctx.simCfg.requireOutsideConnection;
        tc.congestionAwareRouting = ctx.sim.trafficModel().congestionAwareRouting;
        tc.congestionIterations = ctx.sim.trafficModel().congestionIterations;
        tc.congestionAlpha = ctx.sim.trafficModel().congestionAlpha;
        tc.congestionBeta = ctx.sim.trafficModel().congestionBeta;
        tc.congestionCapacityScale = ctx.sim.trafficModel().congestionCapacityScale;
        tc.congestionRatioClamp = ctx.sim.trafficModel().congestionRatioClamp;

        float employedShare = 1.0f;
        const int pop = ctx.world.stats().population;
        if (pop > 0) {
          employedShare = static_cast<float>(ctx.world.stats().employed) / static_cast<float>(pop);
        }

        traffic = ComputeCommuteTraffic(ctx.world, tc, employedShare, roadToEdgePtr);
      }

      if (layer == ExportLayer::GoodsTraffic || layer == ExportLayer::GoodsFill || layer == ExportLayer::LandValue) {
        GoodsConfig gc;
        gc.requireOutsideConnection = ctx.simCfg.requireOutsideConnection;
        goods = ComputeGoodsFlow(ctx.world, gc, roadToEdgePtr);
      }

      if (layer == ExportLayer::LandValue) {
        LandValueConfig lvc;
        lvc.requireOutsideConnection = ctx.simCfg.requireOutsideConnection;
        lv = ComputeLandValue(ctx.world, lvc, &traffic, roadToEdgePtr);
      }

      const LandValueResult* lvPtr = (layer == ExportLayer::LandValue) ? &lv : nullptr;
      const TrafficResult* trPtr = (layer == ExportLayer::Traffic || layer == ExportLayer::LandValue) ? &traffic : nullptr;
      const GoodsResult* gPtr = (layer == ExportLayer::GoodsTraffic || layer == ExportLayer::GoodsFill || layer == ExportLayer::LandValue) ? &goods : nullptr;

      PpmImage img = RenderPpmLayer(ctx.world, layer, lvPtr, trPtr, gPtr);
      if (scale > 1) img = ScaleNearest(img, scale);

      std::string err;
      if (!WritePpm(t[2], img, err)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": export_ppm failed: " << err << "\n";
        return false;
      }

      if (!ctx.quiet) {
        std::cout << "exported " << ExportLayerName(layer) << " -> " << t[2] << "\n";
      }
      continue;
    }

    if (cmd == "export_tiles_csv") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": export_tiles_csv expects: export_tiles_csv <out.csv>\n";
        return false;
      }
      std::string err;
      if (!WriteTilesCsv(ctx.world, t[1], err)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": export_tiles_csv failed: " << err << "\n";
        return false;
      }
      if (!ctx.quiet) {
        std::cout << "exported tiles csv -> " << t[1] << "\n";
      }
      continue;
    }

    if (cmd == "districts_json") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": districts_json expects: districts_json <out.json>\n";
        return false;
      }
      RefreshIfDirty(ctx);
      if (!WriteDistrictsJson(t[1], ctx.world, ctx.simCfg)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": districts_json failed\n";
        return false;
      }
      if (!ctx.quiet) {
        std::cout << "exported districts json -> " << t[1] << "\n";
      }
      continue;
    }

    if (cmd == "hash") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      RefreshIfDirty(ctx);
      const std::uint64_t h = HashWorld(ctx.world, true);
      std::cout << HexU64(h) << "\n";
      continue;
    }

    if (cmd == "expect_hash") {
      if (!EnsureWorld(ctx, lineNo)) return false;
      if (t.size() != 2) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": expect_hash expects: expect_hash <u64|0x...>\n";
        return false;
      }
      std::uint64_t want = 0;
      if (!ParseU64(t[1], &want)) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": invalid hash integer\n";
        return false;
      }
      RefreshIfDirty(ctx);
      const std::uint64_t got = HashWorld(ctx.world, true);
      if (got != want) {
        std::cerr << ctx.scriptPath << ':' << lineNo << ": expect_hash FAILED\n";
        std::cerr << "  want: " << HexU64(want) << "\n";
        std::cerr << "  got:  " << HexU64(got) << "\n";
        return false;
      }
      continue;
    }

    std::cerr << ctx.scriptPath << ':' << lineNo << ": unknown command: " << t[0] << "\n";
    return false;
  }

  return true;
}

} // namespace

int main(int argc, char** argv)
{
  ExecContext ctx;

  std::string outJson;
  std::string outCsv;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }
    if (arg == "--quiet") {
      ctx.quiet = true;
      continue;
    }
    if (arg == "--out" || arg == "--json") {
      if (i + 1 >= argc) {
        std::cerr << arg << " requires a path\n";
        return 2;
      }
      outJson = argv[++i];
      continue;
    }
    if (arg == "--csv") {
      if (i + 1 >= argc) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      outCsv = argv[++i];
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    }

    if (ctx.scriptPath.empty()) {
      ctx.scriptPath = arg;
    } else {
      std::cerr << "unexpected positional arg: " << arg << "\n";
      return 2;
    }
  }

  if (ctx.scriptPath.empty()) {
    PrintHelp();
    return 2;
  }

  if (!RunScript(ctx)) {
    return 1;
  }

  if (!ctx.hasWorld) {
    std::cerr << "script completed but no world was generated/loaded\n";
    return 1;
  }

  RefreshIfDirty(ctx);

  const std::uint64_t hash = HashWorld(ctx.world, true);

  if (!ctx.quiet) {
    const Stats& s = ctx.world.stats();
    std::cout << "done: " << ctx.world.width() << "x" << ctx.world.height() << " seed=" << ctx.world.seed()
              << " day=" << s.day << " pop=" << s.population << " money=" << s.money
              << " hash=" << HexU64(hash) << "\n";
  }

  if (!WriteJsonSummary(outJson, ctx.scriptPath, ctx.world, hash)) {
    std::cerr << "failed to write json summary: " << outJson << "\n";
    return 1;
  }

  if (!WriteCsvTrace(outCsv, ctx.tickStats)) {
    std::cerr << "failed to write csv: " << outCsv << "\n";
    return 1;
  }

  return 0;
}
