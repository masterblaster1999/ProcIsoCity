#include "isocity/AutoBuild.hpp"

#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/Road.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace isocity {

namespace {

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  if (s == "0" || s == "false" || s == "False" || s == "FALSE") {
    *out = false;
    return true;
  }
  if (s == "1" || s == "true" || s == "True" || s == "TRUE") {
    *out = true;
    return true;
  }
  return false;
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

static std::uint32_t DaySeed(const World& world, int day, std::uint32_t salt)
{
  const std::uint32_t s0 = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu);
  // Mix day + salt (avoid trivial correlations).
  std::uint32_t v = s0 ^ (static_cast<std::uint32_t>(day) * 0x9E3779B1u) ^ (salt * 0x85EBCA6Bu);
  // One cheap avalanche step.
  v ^= v >> 16;
  v *= 0x7FEB352Du;
  v ^= v >> 15;
  v *= 0x846CA68Bu;
  v ^= v >> 16;
  return v;
}

static bool HasAnyRoad(const World& world)
{
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      if (world.at(x, y).overlay == Overlay::Road) return true;
    }
  }
  return false;
}

static int CountZoneTiles(const World& world)
{
  int c = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Overlay o = world.at(x, y).overlay;
      if (o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial) ++c;
    }
  }
  return c;
}

static bool AnyEdgeConnectedRoad(const std::vector<std::uint8_t>& roadToEdge)
{
  for (std::uint8_t v : roadToEdge) {
    if (v != 0) return true;
  }
  return false;
}

static bool ApplyRoadTile(World& world, int x, int y, int level, bool allowBridges, ToolApplyResult* outRes)
{
  if (!world.inBounds(x, y)) {
    if (outRes) *outRes = ToolApplyResult::OutOfBounds;
    return false;
  }
  const Tile& t = world.at(x, y);
  if (t.terrain == Terrain::Water && !allowBridges) {
    if (outRes) *outRes = ToolApplyResult::BlockedWater;
    return false;
  }

  const ToolApplyResult r = world.applyRoad(x, y, level);
  if (outRes) *outRes = r;
  return (r == ToolApplyResult::Applied || r == ToolApplyResult::Noop);
}

static bool ApplyZoneTile(World& world, Tool zoneTool, int x, int y, int level, ToolApplyResult* outRes)
{
  if (!world.inBounds(x, y)) {
    if (outRes) *outRes = ToolApplyResult::OutOfBounds;
    return false;
  }
  if (level < 1) level = 1;
  if (level > 3) level = 3;

  ToolApplyResult r = ToolApplyResult::Noop;
  // Place once, then re-apply to upgrade.
  for (int i = 0; i < level; ++i) {
    r = world.applyTool(zoneTool, x, y);
    if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) break;
  }
  if (outRes) *outRes = r;
  return (r == ToolApplyResult::Applied || r == ToolApplyResult::Noop);
}

static bool ApplyParkTile(World& world, int x, int y, ToolApplyResult* outRes)
{
  if (!world.inBounds(x, y)) {
    if (outRes) *outRes = ToolApplyResult::OutOfBounds;
    return false;
  }
  const ToolApplyResult r = world.applyTool(Tool::Park, x, y);
  if (outRes) *outRes = r;
  return (r == ToolApplyResult::Applied || r == ToolApplyResult::Noop);
}

struct AdjCounts {
  int roads = 0;
  int parks = 0;
  int res = 0;
  int com = 0;
  int ind = 0;
};

static AdjCounts CountAdj(const World& world, int x, int y)
{
  AdjCounts a{};
  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const auto& d : dirs) {
    const int nx = x + d[0];
    const int ny = y + d[1];
    if (!world.inBounds(nx, ny)) continue;
    const Overlay o = world.at(nx, ny).overlay;
    if (o == Overlay::Road) ++a.roads;
    else if (o == Overlay::Park) ++a.parks;
    else if (o == Overlay::Residential) ++a.res;
    else if (o == Overlay::Commercial) ++a.com;
    else if (o == Overlay::Industrial) ++a.ind;
  }
  return a;
}

static bool IsCandidateBuildTile(const World& world, int x, int y, const SimConfig& simCfg,
                                const AutoBuildConfig& cfg, const std::vector<std::uint8_t>* roadToEdge)
{
  if (!world.inBounds(x, y)) return false;
  const Tile& t = world.at(x, y);
  if (t.overlay != Overlay::None) return false;
  if (t.terrain == Terrain::Water) return false;
  if (!world.hasAdjacentRoad(x, y)) return false;
  if (simCfg.requireOutsideConnection && cfg.respectOutsideConnection) {
    if (!roadToEdge || roadToEdge->empty()) return false;
    if (!HasAdjacentRoadConnectedToEdge(world, *roadToEdge, x, y)) return false;
  }
  return true;
}

static bool PickBestZoneCandidate(const World& world, Tool zoneTool, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                 const std::vector<std::uint8_t>* roadToEdge, const LandValueResult* lv,
                                 int day, int& outX, int& outY)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const bool lvOk = (lv && lv->w == w && lv->h == h && lv->value.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  int bestScore = std::numeric_limits<int>::min();
  std::uint32_t bestTie = 0xFFFFFFFFu;
  bool found = false;

  const std::uint32_t salt =
      (zoneTool == Tool::Residential) ? 0x00524553u :  // "RES"
      (zoneTool == Tool::Commercial)  ? 0x00434F4Du :  // "COM"
                                       0x00494E44u;   // "IND"
  const std::uint32_t seedBase = DaySeed(world, day, salt);

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      if (!IsCandidateBuildTile(world, x, y, simCfg, cfg, roadToEdge)) continue;

      const AdjCounts adj = CountAdj(world, x, y);

      float lv01 = 0.5f;
      if (lvOk) {
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        lv01 = std::clamp(lv->value[idx], 0.0f, 1.0f);
      }

      // Compute an integer score: higher is better.
      int score = 0;
      if (zoneTool == Tool::Residential) {
        score = static_cast<int>(lv01 * 1000.0f);
        score += adj.parks * 120;
        score += adj.res * 80;
        score -= adj.ind * 180;
      } else if (zoneTool == Tool::Commercial) {
        score = static_cast<int>(lv01 * 900.0f);
        score += adj.res * 110;
        score += adj.com * 70;
        score -= adj.ind * 120;
      } else {
        // Industrial
        score = static_cast<int>((1.0f - lv01) * 900.0f);
        score += adj.ind * 120;
        score -= adj.res * 200;
        score -= adj.parks * 140;
      }

      const std::uint32_t tie = HashCoords32(x, y, seedBase);
      // Small deterministic jitter to avoid rigid patterns when many candidates tie.
      score += static_cast<int>(tie & 0x3Fu);

      if (!found || score > bestScore || (score == bestScore && tie < bestTie)) {
        found = true;
        bestScore = score;
        bestTie = tie;
        outX = x;
        outY = y;
      }
    }
  }

  return found;
}

static bool PickBestParkCandidate(const World& world, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                 const std::vector<std::uint8_t>* roadToEdge, const LandValueResult* lv,
                                 int day, int& outX, int& outY)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const bool lvOk = (lv && lv->w == w && lv->h == h && lv->value.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  int bestScore = std::numeric_limits<int>::min();
  std::uint32_t bestTie = 0xFFFFFFFFu;
  bool found = false;

  const std::uint32_t seed2 = DaySeed(world, day, 0x5041524Bu); // "PARK"

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      if (!IsCandidateBuildTile(world, x, y, simCfg, cfg, roadToEdge)) continue;

      // Parks are most useful near zones.
      const AdjCounts adj = CountAdj(world, x, y);
      const int zoneAdj = adj.res + adj.com + adj.ind;
      if (zoneAdj == 0) continue;

      float lv01 = 0.5f;
      if (lvOk) {
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        lv01 = std::clamp(lv->value[idx], 0.0f, 1.0f);
      }

      int score = 0;
      score += zoneAdj * 180;
      score += static_cast<int>(lv01 * 300.0f);
      // Avoid placing parks directly next to industry when possible.
      score -= adj.ind * 120;

      const std::uint32_t tie = HashCoords32(x, y, seed2);
      score += static_cast<int>(tie & 0x3Fu);

      if (!found || score > bestScore || (score == bestScore && tie < bestTie)) {
        found = true;
        bestScore = score;
        bestTie = tie;
        outX = x;
        outY = y;
      }
    }
  }

  return found;
}

static bool PickRoadExpansionStart(const World& world, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                  const std::vector<std::uint8_t>* roadToEdge, int day,
                                  int& outX, int& outY, int& outDir)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const bool useEdgeMask = (simCfg.requireOutsideConnection && cfg.respectOutsideConnection && roadToEdge &&
                            roadToEdge->size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h) &&
                            AnyEdgeConnectedRoad(*roadToEdge));

  bool found = false;
  int bestScore = std::numeric_limits<int>::min();
  std::uint32_t bestTie = 0xFFFFFFFFu;
  int bestDir = 0;

  const std::uint32_t seedBase = DaySeed(world, day, 0x524F4144u); // "ROAD"

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;

      if (useEdgeMask) {
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        if ((*roadToEdge)[idx] == 0) continue;
      }

      // Count adjacent empty buildable tiles. Prefer frontier roads.
      int empties = 0;
      for (int d = 0; d < 4; ++d) {
        const int nx = x + dirs[d][0];
        const int ny = y + dirs[d][1];
        if (!world.inBounds(nx, ny)) continue;
        const Tile& nt = world.at(nx, ny);
        if (nt.overlay != Overlay::None) continue;
        if (nt.terrain == Terrain::Water && !cfg.allowBridges) continue;
        if (nt.terrain != Terrain::Water && !world.isBuildable(nx, ny)) continue;
        ++empties;
      }
      if (empties == 0) continue;

      // Small preference for lower-level roads to expand outward first.
      const int level = std::clamp(static_cast<int>(t.level), 1, 3);
      int score = empties * 100 - level * 10;

      const std::uint32_t tie = HashCoords32(x, y, seedBase);
      score += static_cast<int>(tie & 0x1Fu);

      if (!found || score > bestScore || (score == bestScore && tie < bestTie)) {
        found = true;
        bestScore = score;
        bestTie = tie;
        outX = x;
        outY = y;

        // Deterministic direction selection based on hash bits.
        const int startDir = static_cast<int>((tie >> 8) & 3u);
        bestDir = startDir;
      }
    }
  }

  if (!found) return false;

  // Choose a direction from the chosen road tile that is expandable.
  constexpr int dirs2[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int k = 0; k < 4; ++k) {
    const int d = (bestDir + k) & 3;
    const int nx = outX + dirs2[d][0];
    const int ny = outY + dirs2[d][1];
    if (!world.inBounds(nx, ny)) continue;
    const Tile& nt = world.at(nx, ny);
    if (nt.overlay != Overlay::None) continue;
    if (nt.terrain == Terrain::Water && !cfg.allowBridges) continue;
    if (nt.terrain != Terrain::Water && !world.isBuildable(nx, ny)) continue;
    outDir = d;
    return true;
  }

  return false;
}

static int BuildRoadSpur(World& world, const AutoBuildConfig& cfg, int startX, int startY, int dir, int day,
                         int* outPlaced)
{
  if (outPlaced) *outPlaced = 0;
  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  if (dir < 0 || dir > 3) return 0;

  const std::uint32_t seedBase = DaySeed(world, day, 0x53505552u); // "SPUR"
  const std::uint32_t h = HashCoords32(startX, startY, seedBase);
  const int maxLen = std::max(1, cfg.maxRoadSpurLength);
  const int len = 2 + static_cast<int>(h % static_cast<std::uint32_t>(std::max(1, maxLen - 1)));

  int placed = 0;
  int x = startX;
  int y = startY;
  for (int i = 0; i < len; ++i) {
    x += dirs[dir][0];
    y += dirs[dir][1];
    ToolApplyResult r = ToolApplyResult::Noop;
    if (!ApplyRoadTile(world, x, y, cfg.roadLevel, cfg.allowBridges, &r)) {
      break;
    }
    if (r == ToolApplyResult::Applied) ++placed;
  }

  if (outPlaced) *outPlaced = placed;
  return placed;
}

static bool EnsureOutsideRoadConnection(World& world, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                       std::vector<std::uint8_t>& roadToEdge)
{
  if (!simCfg.requireOutsideConnection) return true;

  ComputeRoadsConnectedToEdge(world, roadToEdge);
  if (AnyEdgeConnectedRoad(roadToEdge)) return true;

  // Pick a start: prefer an existing road, else find a buildable center tile.
  Point start{-1, -1};
  for (int y = 0; y < world.height() && start.x < 0; ++y) {
    for (int x = 0; x < world.width(); ++x) {
      if (world.at(x, y).overlay == Overlay::Road) {
        start = Point{x, y};
        break;
      }
    }
  }

  if (start.x < 0) {
    // Center-ish.
    start = Point{world.width() / 2, world.height() / 2};
    if (!world.inBounds(start.x, start.y) || world.at(start.x, start.y).terrain == Terrain::Water) {
      // Search for any land tile.
      for (int y = 0; y < world.height() && start.x < 0; ++y) {
        for (int x = 0; x < world.width(); ++x) {
          if (world.at(x, y).terrain != Terrain::Water) {
            start = Point{x, y};
            break;
          }
        }
      }
    }
  }

  if (!world.inBounds(start.x, start.y)) return false;

  // Find a buildable edge tile closest to start.
  Point goal{-1, -1};
  int bestD = std::numeric_limits<int>::max();
  auto consider = [&](int x, int y) {
    if (!world.inBounds(x, y)) return;
    const Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water && !cfg.allowBridges) return;
    if (t.terrain != Terrain::Water && !world.isBuildable(x, y)) return;
    const int d = std::abs(x - start.x) + std::abs(y - start.y);
    if (d < bestD) {
      bestD = d;
      goal = Point{x, y};
    }
  };

  for (int x = 0; x < world.width(); ++x) {
    consider(x, 0);
    consider(x, world.height() - 1);
  }
  for (int y = 0; y < world.height(); ++y) {
    consider(0, y);
    consider(world.width() - 1, y);
  }

  if (goal.x < 0) return false;

  RoadBuildPathConfig pathCfg;
  pathCfg.targetLevel = std::clamp(cfg.roadLevel, 1, 3);
  pathCfg.allowBridges = cfg.allowBridges;
  pathCfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;

  std::vector<Point> path;
  int cost = 0;
  if (!FindRoadBuildPath(world, start, goal, path, &cost, pathCfg)) {
    return false;
  }

  for (const Point& p : path) {
    ToolApplyResult r = ToolApplyResult::Noop;
    if (!ApplyRoadTile(world, p.x, p.y, cfg.roadLevel, cfg.allowBridges, &r)) {
      // If we can't afford to finish the connection, give up.
      return false;
    }
  }

  ComputeRoadsConnectedToEdge(world, roadToEdge);
  return AnyEdgeConnectedRoad(roadToEdge);
}

static void UpgradeMostCongestedRoads(World& world, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                     const std::vector<std::uint8_t>* roadToEdge, int day,
                                     int* ioUpgraded, int* ioFailed)
{
  if (!ioUpgraded || !ioFailed) return;
  if (*ioUpgraded >= cfg.roadUpgradesPerDay) return;

  const Stats& s = world.stats();
  if (s.population <= 0 || s.employed <= 0) return;

  const float employedShare = (s.population > 0) ? (static_cast<float>(s.employed) / static_cast<float>(s.population)) : 0.0f;
  if (employedShare <= 0.0f) return;

  TrafficConfig tc;
  tc.requireOutsideConnection = simCfg.requireOutsideConnection;
  tc.roadTileCapacity = 28;
  tc.congestionAwareRouting = true;
  tc.congestionIterations = 3;

  const std::vector<std::uint8_t>* maskPtr = nullptr;
  if (tc.requireOutsideConnection && roadToEdge &&
      roadToEdge->size() == static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height())) {
    maskPtr = roadToEdge;
  }

  const TrafficResult traffic = ComputeCommuteTraffic(world, tc, employedShare, maskPtr);
  if (traffic.roadTraffic.empty()) return;

  struct RoadCand {
    int x = 0;
    int y = 0;
    float ratio = 0.0f;
    std::uint32_t tie = 0;
  };

  std::vector<RoadCand> cands;
  cands.reserve(512);

  const int w = world.width();
  const int h = world.height();
  const int baseCap = tc.roadTileCapacity;
  const std::uint32_t seedBase = DaySeed(world, day, 0x55504752u); // "UPGR"

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int lvl = ClampRoadLevel(static_cast<int>(t.level));
      if (lvl >= 3) continue;
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const std::uint16_t v = traffic.roadTraffic[idx];
      if (v == 0) continue;
      const int cap = std::max(1, RoadCapacityForLevel(baseCap, lvl));
      const float ratio = static_cast<float>(v) / static_cast<float>(cap);
      if (ratio < 1.05f) continue; // only upgrade meaningfully loaded roads

      RoadCand rc;
      rc.x = x;
      rc.y = y;
      rc.ratio = ratio;
      rc.tie = HashCoords32(x, y, seedBase);
      cands.push_back(rc);
    }
  }

  std::sort(cands.begin(), cands.end(), [](const RoadCand& a, const RoadCand& b) {
    if (a.ratio != b.ratio) return a.ratio > b.ratio;
    return a.tie < b.tie;
  });

  for (const RoadCand& rc : cands) {
    if (*ioUpgraded >= cfg.roadUpgradesPerDay) break;
    if (world.stats().money <= cfg.minMoneyReserve) break;
    const int cur = ClampRoadLevel(static_cast<int>(world.at(rc.x, rc.y).level));
    ToolApplyResult r = ToolApplyResult::Noop;
    (void)ApplyRoadTile(world, rc.x, rc.y, cur + 1, true /* allowBridges irrelevant for existing road */, &r);
    if (r == ToolApplyResult::Applied) {
      ++(*ioUpgraded);
    } else if (r == ToolApplyResult::InsufficientFunds) {
      ++(*ioFailed);
      break;
    }
  }
}

} // namespace

bool ParseAutoBuildKey(const std::string& key, const std::string& value, AutoBuildConfig& cfg, std::string& outError)
{
  outError.clear();
  const std::string k = ToLower(key);

  auto bad = [&](const std::string& msg) {
    outError = msg;
    return false;
  };

  if (k == "zonesperday" || k == "zones_per_day") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("zonesPerDay expects non-negative int");
    cfg.zonesPerDay = v;
    return true;
  }
  if (k == "roadsperday" || k == "roads_per_day") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("roadsPerDay expects non-negative int");
    cfg.roadsPerDay = v;
    return true;
  }
  if (k == "parksperday" || k == "parks_per_day") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("parksPerDay expects non-negative int");
    cfg.parksPerDay = v;
    return true;
  }
  if (k == "roadlevel" || k == "road_level") {
    int v = 0;
    if (!ParseI32(value, &v)) return bad("roadLevel expects int");
    cfg.roadLevel = std::clamp(v, 1, 3);
    return true;
  }
  if (k == "allowbridges" || k == "allow_bridges") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("allowBridges expects 0|1");
    cfg.allowBridges = b;
    return true;
  }
  if (k == "minmoneyreserve" || k == "min_money_reserve" || k == "reserve") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("minMoneyReserve expects non-negative int");
    cfg.minMoneyReserve = v;
    return true;
  }
  if (k == "parkperzonetiles" || k == "park_per_zone_tiles" || k == "park_ratio") {
    int v = 0;
    if (!ParseI32(value, &v) || v <= 0) return bad("parkPerZoneTiles expects positive int");
    cfg.parkPerZoneTiles = v;
    return true;
  }
  if (k == "autoupgraderoads" || k == "auto_upgrade_roads") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("autoUpgradeRoads expects 0|1");
    cfg.autoUpgradeRoads = b;
    return true;
  }
  if (k == "congestionupgradethreshold" || k == "congestion_upgrade_threshold" || k == "congestion") {
    float f = 0.0f;
    if (!ParseF32(value, &f)) return bad("congestionUpgradeThreshold expects float");
    cfg.congestionUpgradeThreshold = std::clamp(f, 0.0f, 1.0f);
    return true;
  }
  if (k == "roadupgradesperday" || k == "road_upgrades_per_day") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("roadUpgradesPerDay expects non-negative int");
    cfg.roadUpgradesPerDay = v;
    return true;
  }
  if (k == "landvaluerecalcdays" || k == "land_value_recalc_days") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 1) return bad("landValueRecalcDays expects int >= 1");
    cfg.landValueRecalcDays = v;
    return true;
  }
  if (k == "respectoutsideconnection" || k == "respect_outside_connection") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("respectOutsideConnection expects 0|1");
    cfg.respectOutsideConnection = b;
    return true;
  }
  if (k == "ensureoutsideconnection" || k == "ensure_outside_connection") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("ensureOutsideConnection expects 0|1");
    cfg.ensureOutsideConnection = b;
    return true;
  }
  if (k == "maxroadspurlength" || k == "max_road_spur_length") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 1) return bad("maxRoadSpurLength expects int >= 1");
    cfg.maxRoadSpurLength = v;
    return true;
  }

  return bad("unknown autobuild key: " + key);
}

AutoBuildReport RunAutoBuild(World& world, Simulator& sim, const AutoBuildConfig& cfg, int days,
                             std::vector<Stats>* outDailyStats)
{
  AutoBuildReport rep;
  rep.daysRequested = std::max(0, days);
  rep.daysSimulated = 0;
  if (days <= 0) return rep;

  // Ensure we have a valid derived snapshot before making decisions.
  sim.refreshDerivedStats(world);

  const SimConfig& simCfg = sim.config();

  std::vector<std::uint8_t> roadToEdge;
  if (simCfg.requireOutsideConnection && cfg.ensureOutsideConnection) {
    (void)EnsureOutsideRoadConnection(world, simCfg, cfg, roadToEdge);
  }

  LandValueResult lv;
  int lastLvDay = std::numeric_limits<int>::min();

  for (int i = 0; i < days; ++i) {
    // The current derived state describes the world at the start of this day.
    const Stats& s = world.stats();
    const int day = s.day;

    // Recompute edge connectivity each day (cheap) so zoning respects outside connection.
    if (simCfg.requireOutsideConnection) {
      ComputeRoadsConnectedToEdge(world, roadToEdge);
    } else {
      roadToEdge.clear();
    }

    // (Re)compute land value occasionally to drive placement heuristics.
    if (cfg.landValueRecalcDays <= 1 || (day - lastLvDay) >= cfg.landValueRecalcDays ||
        lv.w != world.width() || lv.h != world.height() ||
        lv.value.size() != static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height())) {
      LandValueConfig lvc;
      lvc.requireOutsideConnection = simCfg.requireOutsideConnection;
      const std::vector<std::uint8_t>* maskPtr = (simCfg.requireOutsideConnection ? &roadToEdge : nullptr);
      lv = ComputeLandValue(world, lvc, nullptr, maskPtr);
      lastLvDay = day;
    }

    // If money is low, skip building and just simulate to collect income.
    const int moneyStart = world.stats().money;
    const bool canSpend = (moneyStart > cfg.minMoneyReserve);

    // Determine if we want a park.
    const int zoneTiles = CountZoneTiles(world);
    const bool wantPark = (canSpend && cfg.parksPerDay > 0 &&
                           (s.happiness < 0.45f || (zoneTiles > 0 && s.parks * cfg.parkPerZoneTiles < zoneTiles)));

    int parksPlacedToday = 0;
    if (wantPark) {
      for (int p = 0; p < cfg.parksPerDay; ++p) {
        if (world.stats().money <= cfg.minMoneyReserve) break;
        int px = 0, py = 0;
        if (!PickBestParkCandidate(world, simCfg, cfg, simCfg.requireOutsideConnection ? &roadToEdge : nullptr, &lv, day, px,
                                   py)) {
          break;
        }
        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyParkTile(world, px, py, &r) || r != ToolApplyResult::Applied) {
          ++rep.failedBuilds;
          if (r == ToolApplyResult::InsufficientFunds) break;
        } else {
          ++rep.parksBuilt;
          ++parksPlacedToday;
        }
      }
    }

    // Decide a zoning target based on job/housing balance.
    //
    // NOTE: When housing==0, the simulator's demand model needs *some* jobs to
    // exist first; otherwise residential target occupancy stays at 0.
    float jobPressure = 0.0f;
    if (s.housingCapacity <= 0) {
      jobPressure = (s.jobsCapacityAccessible > 0) ? 2.0f : 0.0f;
    } else {
      jobPressure = static_cast<float>(s.jobsCapacityAccessible) / static_cast<float>(s.housingCapacity);
    }

    Tool zoneTool = Tool::Residential;
    if (jobPressure > 1.10f || s.demandResidential > 0.55f) {
      zoneTool = Tool::Residential;
    } else if (jobPressure < 0.80f) {
      // Need jobs.
      zoneTool = (s.goodsSatisfaction < 0.80f) ? Tool::Industrial : Tool::Commercial;
    } else {
      // Balanced: add a mix.
      const std::uint32_t mix = HashCoords32(day, s.population, DaySeed(world, day, 0x004D4958u)); // "MIX"

      const int r = static_cast<int>(mix % 10u);
      if (r < 5) zoneTool = Tool::Residential;
      else if (r < 8) zoneTool = Tool::Commercial;
      else zoneTool = Tool::Industrial;
    }

    // Place zones.
    if (canSpend && cfg.zonesPerDay > 0) {
      for (int z = 0; z < cfg.zonesPerDay; ++z) {
        if (world.stats().money <= cfg.minMoneyReserve) break;
        int zx = 0, zy = 0;
        if (!PickBestZoneCandidate(world, zoneTool, simCfg, cfg, simCfg.requireOutsideConnection ? &roadToEdge : nullptr,
                                   &lv, day + z, zx, zy)) {
          break;
        }
        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyZoneTile(world, zoneTool, zx, zy, 1, &r) || r != ToolApplyResult::Applied) {
          ++rep.failedBuilds;
          if (r == ToolApplyResult::InsufficientFunds) break;
        } else {
          ++rep.zonesBuilt;
        }
      }
    }

    // Expand roads if we're running low on adjacent buildable tiles.
    if (canSpend && cfg.roadsPerDay > 0) {
      for (int r = 0; r < cfg.roadsPerDay; ++r) {
        if (world.stats().money <= cfg.minMoneyReserve) break;
        int sx = 0, sy = 0, dir = 0;
        if (!HasAnyRoad(world)) {
          // Seed an initial cross at center.
          const int cx = world.width() / 2;
          const int cy = world.height() / 2;
          ToolApplyResult rr = ToolApplyResult::Noop;
          (void)ApplyRoadTile(world, cx, cy, cfg.roadLevel, cfg.allowBridges, &rr);
          (void)ApplyRoadTile(world, cx + 1, cy, cfg.roadLevel, cfg.allowBridges, &rr);
          (void)ApplyRoadTile(world, cx - 1, cy, cfg.roadLevel, cfg.allowBridges, &rr);
          (void)ApplyRoadTile(world, cx, cy + 1, cfg.roadLevel, cfg.allowBridges, &rr);
          (void)ApplyRoadTile(world, cx, cy - 1, cfg.roadLevel, cfg.allowBridges, &rr);
          rep.roadsBuilt += 5;
          break;
        }
        if (!PickRoadExpansionStart(world, simCfg, cfg, simCfg.requireOutsideConnection ? &roadToEdge : nullptr, day + r,
                                   sx, sy, dir)) {
          break;
        }
        int placed = 0;
        BuildRoadSpur(world, cfg, sx, sy, dir, day + r, &placed);
        rep.roadsBuilt += placed;
        if (placed == 0) {
          ++rep.failedBuilds;
          break;
        }
      }
    }

    // Optional road upgrades when congestion spikes.
    if (cfg.autoUpgradeRoads && canSpend && s.trafficCongestion >= cfg.congestionUpgradeThreshold &&
        cfg.roadUpgradesPerDay > 0) {
      int upgraded = 0;
      int failed = 0;
      UpgradeMostCongestedRoads(world, simCfg, cfg, simCfg.requireOutsideConnection ? &roadToEdge : nullptr, day, &upgraded,
                               &failed);
      rep.roadsUpgraded += upgraded;
      rep.failedBuilds += failed;
    }

    // Advance the simulation by one day.
    sim.stepOnce(world);
    ++rep.daysSimulated;
    if (outDailyStats) outDailyStats->push_back(world.stats());
  }

  return rep;
}

} // namespace isocity
