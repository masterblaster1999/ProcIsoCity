#include "isocity/AutoBuild.hpp"

#include "isocity/LandValue.hpp"
#include "isocity/ParkOptimizer.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/Road.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/RoadResilienceBypass.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/ZoneAccess.hpp"

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

static Overlay ZoneOverlayForTool(Tool zoneTool)
{
  switch (zoneTool) {
  case Tool::Residential: return Overlay::Residential;
  case Tool::Commercial: return Overlay::Commercial;
  case Tool::Industrial: return Overlay::Industrial;
  default: return Overlay::None;
  }
}

static bool HasAdjacentAccessibleZoneTile(const World& world, const ZoneAccessMap& za, Overlay zoneOverlay, int x, int y)
{
  if (zoneOverlay == Overlay::None) return false;
  if (za.w != world.width() || za.h != world.height()) return false;
  if (za.roadIdx.size() != static_cast<std::size_t>(za.w) * static_cast<std::size_t>(za.h)) return false;

  const int w = za.w;
  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const auto& d : dirs) {
    const int nx = x + d[0];
    const int ny = y + d[1];
    if (!world.inBounds(nx, ny)) continue;
    const Tile& nt = world.at(nx, ny);
    if (nt.overlay != zoneOverlay) continue;
    const std::size_t idx = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx);
    if (idx >= za.roadIdx.size()) continue;
    if (za.roadIdx[idx] >= 0) return true;
  }
  return false;
}

static bool IsCandidateZoneTile(const World& world, Overlay zoneOverlay, int x, int y, const SimConfig& simCfg,
                               const AutoBuildConfig& cfg, const std::vector<std::uint8_t>* roadToEdge,
                               const ZoneAccessMap* zoneAccess)
{
  if (!world.inBounds(x, y)) return false;
  const Tile& t = world.at(x, y);
  if (t.overlay != Overlay::None) return false;
  if (t.terrain == Terrain::Water) return false;

  // Allow interior zoning: a tile is a candidate if it would have road access once zoned.
  if (!world.wouldZoneHaveRoadAccess(zoneOverlay, x, y)) return false;

  // Optional outside-connection rule: the zone component must touch a road component
  // that reaches the map edge.
  if (simCfg.requireOutsideConnection && cfg.respectOutsideConnection) {
    if (!roadToEdge || roadToEdge->empty()) return false;

    // Direct adjacency to an edge-connected road is always acceptable.
    if (HasAdjacentRoadConnectedToEdge(world, *roadToEdge, x, y)) return true;

    // Otherwise, we only allow building if this tile is adjacent to an already-accessible
    // zone tile of the same type (so the new tile inherits that access).
    if (!zoneAccess) return false;
    return HasAdjacentAccessibleZoneTile(world, *zoneAccess, zoneOverlay, x, y);
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

  const Overlay zoneOverlay = ZoneOverlayForTool(zoneTool);
  if (zoneOverlay == Overlay::None) return false;

  // If the sim enforces an outside connection, precompute a zone access map that only
  // considers edge-connected roads as valid access points.
  const bool requireOutside = (simCfg.requireOutsideConnection && cfg.respectOutsideConnection);
  ZoneAccessMap zoneAccess;
  if (requireOutside && roadToEdge) {
    zoneAccess = BuildZoneAccessMap(world, roadToEdge);
  }

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
      if (!IsCandidateZoneTile(world, zoneOverlay, x, y, simCfg, cfg, roadToEdge, requireOutside ? &zoneAccess : nullptr)) {
        continue;
      }

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

static int GrowZoneCluster(World& world, Tool zoneTool, Overlay zoneOverlay, int seedX, int seedY, int day,
                           int maxExtraTiles, int minMoneyReserve)
{
  if (maxExtraTiles <= 0) return 0;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0;

  auto idxOf = [&](int x, int y) -> int { return y * w + x; };

  std::vector<std::uint8_t> seen(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
  std::vector<Point> frontier;
  frontier.reserve(static_cast<std::size_t>(maxExtraTiles) + 8u);

  if (!world.inBounds(seedX, seedY)) return 0;
  {
    const int si = idxOf(seedX, seedY);
    if (si < 0 || static_cast<std::size_t>(si) >= seen.size()) return 0;
    seen[static_cast<std::size_t>(si)] = 1;
  }
  frontier.push_back(Point{seedX, seedY});

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // N,E,S,W

  const std::uint32_t seedBase = DaySeed(world, day, 0x424C4F4Bu); // "BLOK"

  int placed = 0;
  std::size_t head = 0;
  while (head < frontier.size() && placed < maxExtraTiles) {
    if (world.stats().money <= minMoneyReserve) break;

    const Point cur = frontier[head++];
    const std::uint32_t h0 = HashCoords32(cur.x, cur.y, seedBase);
    const int rot = static_cast<int>(h0 & 3u);

    for (int k = 0; k < 4 && placed < maxExtraTiles; ++k) {
      if (world.stats().money <= minMoneyReserve) break;

      const int d = (rot + k) & 3;
      const int nx = cur.x + dirs[d][0];
      const int ny = cur.y + dirs[d][1];
      if (!world.inBounds(nx, ny)) continue;

      const int ni = idxOf(nx, ny);
      if (ni < 0) continue;
      const std::size_t ui = static_cast<std::size_t>(ni);
      if (ui >= seen.size()) continue;
      if (seen[ui]) continue;
      seen[ui] = 1;

      const Tile& t = world.at(nx, ny);
      if (t.overlay != Overlay::None) continue;
      if (t.terrain == Terrain::Water) continue;

      // Only attempt tiles that would be reachable via the connected-component
      // zoning rule.
      if (!world.wouldZoneHaveRoadAccess(zoneOverlay, nx, ny)) continue;

      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyZoneTile(world, zoneTool, nx, ny, 1, &r) || r != ToolApplyResult::Applied) {
        continue;
      }

      ++placed;
      frontier.push_back(Point{nx, ny});
    }
  }

  return placed;
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

struct RoadDistanceField {
  int w = 0;
  int h = 0;

  // Manhattan steps from the nearest source road tile, or -1 if unreachable.
  std::vector<int> dist;

  // Flattened index of the nearest source road tile, or -1.
  std::vector<int> nearestRoadIdx;
};

static RoadDistanceField ComputeRoadDistanceField(const World& world, const AutoBuildConfig& cfg,
                                                  const std::vector<std::uint8_t>* roadToEdge,
                                                  const SimConfig& simCfg)
{
  RoadDistanceField f;
  f.w = world.width();
  f.h = world.height();
  const int w = f.w;
  const int h = f.h;
  if (w <= 0 || h <= 0) return f;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  f.dist.assign(n, -1);
  f.nearestRoadIdx.assign(n, -1);

  const bool useEdgeMask = (simCfg.requireOutsideConnection && cfg.respectOutsideConnection && roadToEdge &&
                            roadToEdge->size() == n && AnyEdgeConnectedRoad(*roadToEdge));

  auto idxOf = [&](int x, int y) -> int { return y * w + x; };

  auto canTraverse = [&](int x, int y) -> bool {
    if (!world.inBounds(x, y)) return false;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::None && t.overlay != Overlay::Road) return false;

    // Existing roads are always traversable, even if they are bridges or placed
    // on currently "unbuildable" steep terrain.
    if (t.overlay == Overlay::Road) return true;

    // For new road placement, respect the bot's bridge/buildability constraints.
    if (t.terrain == Terrain::Water) return cfg.allowBridges;
    return world.isBuildable(x, y);
  };

  std::vector<int> q;
  q.reserve(n);

  // Seed the BFS with existing road tiles.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int idx = idxOf(x, y);
      if (idx < 0) continue;
      const std::size_t ui = static_cast<std::size_t>(idx);
      if (ui >= n) continue;
      if (useEdgeMask && (*roadToEdge)[ui] == 0) continue;
      f.dist[ui] = 0;
      f.nearestRoadIdx[ui] = idx;
      q.push_back(idx);
    }
  }

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

  std::size_t head = 0;
  while (head < q.size()) {
    const int cur = q[head++];
    const int cx = cur % w;
    const int cy = cur / w;
    if (!world.inBounds(cx, cy)) continue;
    const std::size_t cIdx = static_cast<std::size_t>(cur);
    if (cIdx >= n) continue;

    const int cd = f.dist[cIdx];
    const int src = f.nearestRoadIdx[cIdx];

    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (!world.inBounds(nx, ny)) continue;
      if (!canTraverse(nx, ny)) continue;
      const int ni = idxOf(nx, ny);
      if (ni < 0) continue;
      const std::size_t ui = static_cast<std::size_t>(ni);
      if (ui >= n) continue;
      if (f.dist[ui] >= 0) continue;
      f.dist[ui] = cd + 1;
      f.nearestRoadIdx[ui] = src;
      q.push_back(ni);
    }
  }

  return f;
}

static bool PickPlannedRoadGoal(const World& world, const AutoBuildConfig& cfg, Tool zoneTool,
                                const RoadDistanceField& f, const LandValueResult* lv,
                                int day, int& outX, int& outY)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;
  if (f.w != w || f.h != h) return false;
  if (f.dist.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) return false;

  const bool lvOk = (lv && lv->w == w && lv->h == h && lv->value.size() == f.dist.size());

  // Avoid micro-spurs: try to expand at least a few steps out from the current network.
  const int minDist = std::max(3, std::max(1, cfg.maxRoadSpurLength) / 2);
  const int distClamp = std::max(8, std::max(1, cfg.maxRoadSpurLength) * 3);

  int bestScore = std::numeric_limits<int>::min();
  std::uint32_t bestTie = 0xFFFFFFFFu;
  bool found = false;

  const std::uint32_t salt =
      (zoneTool == Tool::Residential) ? 0x00524553u :  // "RES"
      (zoneTool == Tool::Commercial)  ? 0x00434F4Du :  // "COM"
      (zoneTool == Tool::Industrial)  ? 0x00494E44u :  // "IND"
                                       0x00524F44u;   // "ROD"
  const std::uint32_t seedBase = DaySeed(world, day, salt ^ 0x524F4144u); // mix "ROAD" in

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::None) continue;
      if (t.terrain == Terrain::Water && !cfg.allowBridges) continue;
      if (t.terrain != Terrain::Water && !world.isBuildable(x, y)) continue;

      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (idx >= f.dist.size()) continue;
      const int d0 = f.dist[idx];
      if (d0 < minDist) continue;
      if (f.nearestRoadIdx[idx] < 0) continue;

      const int d = std::min(d0, distClamp);

      float lv01 = 0.5f;
      if (lvOk) {
        lv01 = std::clamp((*lv).value[idx], 0.0f, 1.0f);
      }

      const AdjCounts adj = CountAdj(world, x, y);

      float potential = 0.5f;
      if (zoneTool == Tool::Residential || zoneTool == Tool::Commercial) {
        potential = lv01;
      } else if (zoneTool == Tool::Industrial) {
        potential = 1.0f - lv01;
      }

      int score = 0;
      score += d * 160;
      score += static_cast<int>(potential * 1000.0f);

      // Bias corridors to grow near existing development of the target type.
      if (zoneTool == Tool::Residential) {
        score += (adj.res + adj.parks) * 45;
        score -= adj.ind * 60;
      } else if (zoneTool == Tool::Commercial) {
        score += (adj.res + adj.com) * 40;
        score -= adj.ind * 40;
      } else if (zoneTool == Tool::Industrial) {
        score += adj.ind * 55;
        score -= (adj.res + adj.parks) * 60;
      }

      const std::uint32_t tie = HashCoords32(x, y, seedBase);
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

static int BuildPlannedRoadCorridor(World& world, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                    const std::vector<std::uint8_t>* roadToEdge,
                                    const LandValueResult* lv, Tool preferredZoneTool,
                                    int day, int* outPlaced)
{
  if (outPlaced) *outPlaced = 0;
  const int maxSteps = std::max(1, cfg.maxRoadSpurLength);

  const RoadDistanceField f = ComputeRoadDistanceField(world, cfg, roadToEdge, simCfg);
  if (f.dist.empty()) return 0;

  int gx = 0, gy = 0;
  if (!PickPlannedRoadGoal(world, cfg, preferredZoneTool, f, lv, day, gx, gy)) {
    return 0;
  }

  const std::size_t gIdx = static_cast<std::size_t>(gy) * static_cast<std::size_t>(world.width()) + static_cast<std::size_t>(gx);
  if (gIdx >= f.nearestRoadIdx.size()) return 0;
  const int startIdx = f.nearestRoadIdx[gIdx];
  if (startIdx < 0) return 0;
  const int sx = startIdx % world.width();
  const int sy = startIdx / world.width();
  if (!world.inBounds(sx, sy)) return 0;

  RoadBuildPathConfig pathCfg;
  pathCfg.targetLevel = std::clamp(cfg.roadLevel, 1, 3);
  pathCfg.allowBridges = cfg.allowBridges;
  pathCfg.costModel = RoadBuildPathConfig::CostModel::Money;

  std::vector<Point> path;
  int cost = 0;
  if (!FindRoadBuildPath(world, Point{sx, sy}, Point{gx, gy}, path, &cost, pathCfg)) {
    return 0;
  }

  int placed = 0;
  const int steps = std::min<int>(static_cast<int>(path.size()) - 1, maxSteps);
  for (int i = 1; i <= steps; ++i) {
    if (world.stats().money <= cfg.minMoneyReserve) break;
    const Point& p = path[static_cast<std::size_t>(i)];
    ToolApplyResult r = ToolApplyResult::Noop;
    if (!ApplyRoadTile(world, p.x, p.y, cfg.roadLevel, cfg.allowBridges, &r)) {
      break;
    }
    if (r == ToolApplyResult::Applied) ++placed;
    if (r == ToolApplyResult::InsufficientFunds) break;
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

static void BuildResilienceBypasses(World& world, const SimConfig& simCfg, const AutoBuildConfig& cfg,
                                   const std::vector<std::uint8_t>* roadToEdge, int day,
                                   int* ioBuilt, int* ioUpgraded, int* ioFailed)
{
  if (!ioBuilt || !ioUpgraded || !ioFailed) return;
  if (!cfg.autoBuildResilienceBypasses) return;
  if (cfg.resilienceBypassesPerDay <= 0) return;

  (void)day; // reserved for future deterministic tuning hooks

  const Stats& s = world.stats();
  if (s.trafficCongestion < cfg.resilienceBypassCongestionThreshold) return;
  if (world.stats().money <= cfg.minMoneyReserve) return;
  if (!HasAnyRoad(world)) return;

  // Build road graph + resilience.
  const RoadGraph roadGraph = BuildRoadGraph(world);
  if (roadGraph.nodes.empty() || roadGraph.edges.empty()) return;
  const RoadGraphResilienceResult resilience = ComputeRoadGraphResilience(roadGraph);
  if (resilience.bridgeEdges.empty()) return;

  // Optional traffic field for ranking: prioritize bridges that are both heavily
  // used and structurally vulnerable (large cut size).
  TrafficResult traffic;
  const TrafficResult* trafficPtr = nullptr;
  if (s.population > 0 && s.employed > 0) {
    const float employedShare =
        static_cast<float>(s.employed) / std::max(1.0f, static_cast<float>(s.population));
    if (employedShare > 0.0f) {
      TrafficConfig tc;
      tc.requireOutsideConnection = simCfg.requireOutsideConnection;
      tc.roadTileCapacity = 28;
      tc.congestionAwareRouting = true;
      tc.congestionIterations = 2; // bot can be a bit cheaper than the in-game overlay

      const std::vector<std::uint8_t>* maskPtr = nullptr;
      if (tc.requireOutsideConnection && roadToEdge &&
          roadToEdge->size() == static_cast<std::size_t>(world.width()) * static_cast<std::size_t>(world.height())) {
        maskPtr = roadToEdge;
      }

      traffic = ComputeCommuteTraffic(world, tc, employedShare, maskPtr);
      if (!traffic.roadTraffic.empty()) {
        trafficPtr = &traffic;
      }
    }
  }

  RoadResilienceBypassConfig pcfg;
  pcfg.top = std::max(0, cfg.resilienceBypassTop);
  pcfg.moneyObjective = cfg.resilienceBypassMoneyObjective;
  pcfg.targetLevel = std::clamp(cfg.resilienceBypassTargetLevel, 1, 3);
  pcfg.allowBridges = cfg.resilienceBypassAllowBridges;
  pcfg.maxPrimaryCost = cfg.resilienceBypassMaxCost;
  pcfg.maxNodesPerSide = std::max(1, cfg.resilienceBypassMaxNodesPerSide);
  pcfg.rankByTraffic = true;

  const std::vector<RoadResilienceBypassSuggestion> suggestions =
      SuggestRoadResilienceBypasses(world, roadGraph, resilience, pcfg, trafficPtr);

  if (suggestions.empty()) return;

  int builtBypasses = 0;
  bool anyAttempted = false;
  for (const RoadResilienceBypassSuggestion& sug : suggestions) {
    if (builtBypasses >= cfg.resilienceBypassesPerDay) break;
    if (world.stats().money <= cfg.minMoneyReserve) break;

    anyAttempted = true;
    const RoadResilienceBypassApplyReport ar = ApplyRoadResilienceBypass(world, sug, cfg.minMoneyReserve);
    if (ar.result == RoadResilienceBypassApplyResult::Applied) {
      *ioBuilt += ar.builtTiles;
      *ioUpgraded += ar.upgradedTiles;
      ++builtBypasses;
    }
  }

  if (builtBypasses == 0 && anyAttempted) {
    ++(*ioFailed);
  }
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
    int traffic = 0;
    int cap = 1;
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
      const int trafficV = static_cast<int>(v);
      // Deterministic threshold: ratio >= 1.05 (avoid float).
      if (trafficV * 100 < cap * 105) continue; // only upgrade meaningfully loaded roads

      RoadCand rc;
      rc.x = x;
      rc.y = y;
      rc.traffic = trafficV;
      rc.cap = cap;
      rc.tie = HashCoords32(x, y, seedBase);
      cands.push_back(rc);
    }
  }

  std::sort(cands.begin(), cands.end(), [](const RoadCand& a, const RoadCand& b) {
    // Compare ratios (traffic/cap) deterministically without floats via cross-multiplication.
    const std::int64_t lhs = static_cast<std::int64_t>(a.traffic) * static_cast<std::int64_t>(b.cap);
    const std::int64_t rhs = static_cast<std::int64_t>(b.traffic) * static_cast<std::int64_t>(a.cap);
    if (lhs != rhs) return lhs > rhs;
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
  if (k == "zoneclustermaxtiles" || k == "zone_cluster_max_tiles" || k == "zonecluster" || k == "zone_cluster") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 1) return bad("zoneClusterMaxTiles expects int >= 1");
    cfg.zoneClusterMaxTiles = v;
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
  if (k == "useparkoptimizer" || k == "use_park_optimizer" || k == "park_optimizer") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("useParkOptimizer expects 0|1");
    cfg.useParkOptimizer = b;
    return true;
  }
  if (k == "roadlevel" || k == "road_level") {
    int v = 0;
    if (!ParseI32(value, &v)) return bad("roadLevel expects int");
    cfg.roadLevel = std::clamp(v, 1, 3);
    return true;
  }
  if (k == "useroadplanner" || k == "use_road_planner" || k == "road_planner") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("useRoadPlanner expects 0|1");
    cfg.useRoadPlanner = b;
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
  if (k == "autobuildresiliencebypasses" || k == "auto_build_resilience_bypasses" ||
      k == "autoresiliencebypasses" || k == "auto_resilience_bypasses") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("autoBuildResilienceBypasses expects 0|1");
    cfg.autoBuildResilienceBypasses = b;
    return true;
  }
  if (k == "resiliencebypasscongestionthreshold" || k == "resilience_bypass_congestion_threshold" ||
      k == "resiliencebypasscongestion" || k == "resilience_bypass_congestion" ||
      k == "bypasscongestion" || k == "bypass_congestion") {
    float f = 0.0f;
    if (!ParseF32(value, &f)) return bad("resilienceBypassCongestionThreshold expects float");
    cfg.resilienceBypassCongestionThreshold = std::clamp(f, 0.0f, 1.0f);
    return true;
  }
  if (k == "resiliencebypassesperday" || k == "resilience_bypasses_per_day" ||
      k == "bypassesperday" || k == "bypasses_per_day") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("resilienceBypassesPerDay expects non-negative int");
    cfg.resilienceBypassesPerDay = v;
    return true;
  }
  if (k == "resiliencebypasstop" || k == "resilience_bypass_top" || k == "bypasstop") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("resilienceBypassTop expects non-negative int");
    cfg.resilienceBypassTop = v;
    return true;
  }
  if (k == "resiliencebypassmoneyobjective" || k == "resilience_bypass_money_objective" ||
      k == "resiliencebypassmoney" || k == "resilience_bypass_money") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("resilienceBypassMoneyObjective expects 0|1");
    cfg.resilienceBypassMoneyObjective = b;
    return true;
  }
  if (k == "resiliencebypasstargetlevel" || k == "resilience_bypass_target_level" ||
      k == "bypasstargetlevel" || k == "bypass_target_level") {
    int v = 0;
    if (!ParseI32(value, &v)) return bad("resilienceBypassTargetLevel expects int");
    cfg.resilienceBypassTargetLevel = std::clamp(v, 1, 3);
    return true;
  }
  if (k == "resiliencebypassallowbridges" || k == "resilience_bypass_allow_bridges" ||
      k == "bypassallowbridges" || k == "bypass_allow_bridges") {
    bool b = false;
    if (!ParseBool01(value, &b)) return bad("resilienceBypassAllowBridges expects 0|1");
    cfg.resilienceBypassAllowBridges = b;
    return true;
  }
  if (k == "resiliencebypassmaxcost" || k == "resilience_bypass_max_cost" || k == "bypassmaxcost") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 0) return bad("resilienceBypassMaxCost expects non-negative int");
    cfg.resilienceBypassMaxCost = v;
    return true;
  }
  if (k == "resiliencebypassmaxnodesperside" || k == "resilience_bypass_max_nodes_per_side" ||
      k == "bypassmaxnodesperside" || k == "bypass_max_nodes_per_side") {
    int v = 0;
    if (!ParseI32(value, &v) || v < 1) return bad("resilienceBypassMaxNodesPerSide expects int >= 1");
    cfg.resilienceBypassMaxNodesPerSide = v;
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
      if (cfg.useParkOptimizer) {
        ParkOptimizerConfig pc;
        pc.requireOutsideConnection = (simCfg.requireOutsideConnection && cfg.respectOutsideConnection);
        pc.weightMode = IsochroneWeightMode::TravelTime;
        pc.demandMode = ParkDemandMode::Tiles;
        pc.includeResidential = true;
        pc.includeCommercial = true;
        pc.includeIndustrial = true;
        pc.parksToAdd = std::max(0, cfg.parksPerDay);
        pc.targetCostMilli = std::max(0, simCfg.parkInfluenceRadius) * 1000;

        const ParkOptimizerResult pr = SuggestParkPlacements(world, pc, nullptr,
                                                            simCfg.requireOutsideConnection ? &roadToEdge : nullptr);
        for (const ParkPlacement& pp : pr.placements) {
          if (parksPlacedToday >= cfg.parksPerDay) break;
          if (world.stats().money <= cfg.minMoneyReserve) break;
          ToolApplyResult r = ToolApplyResult::Noop;
          if (!ApplyParkTile(world, pp.parkTile.x, pp.parkTile.y, &r) || r != ToolApplyResult::Applied) {
            ++rep.failedBuilds;
            if (r == ToolApplyResult::InsufficientFunds) break;
          } else {
            ++rep.parksBuilt;
            ++parksPlacedToday;
          }
        }

        // If planning produced nothing (eg. no eligible demand yet), fall back to
        // the local heuristic so early worlds can still get a few parks.
        if (parksPlacedToday == 0) {
          for (int p = 0; p < cfg.parksPerDay; ++p) {
            if (world.stats().money <= cfg.minMoneyReserve) break;
            int px = 0, py = 0;
            if (!PickBestParkCandidate(world, simCfg, cfg, simCfg.requireOutsideConnection ? &roadToEdge : nullptr, &lv, day,
                                       px, py)) {
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
      } else {
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
      int remaining = cfg.zonesPerDay;
      int zIter = 0;
      const Overlay zoneOv = ZoneOverlayForTool(zoneTool);

      while (remaining > 0) {
        if (world.stats().money <= cfg.minMoneyReserve) break;

        int zx = 0, zy = 0;
        if (!PickBestZoneCandidate(world, zoneTool, simCfg, cfg,
                                   simCfg.requireOutsideConnection ? &roadToEdge : nullptr,
                                   &lv, day + zIter, zx, zy)) {
          break;
        }
        ++zIter;

        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyZoneTile(world, zoneTool, zx, zy, 1, &r) || r != ToolApplyResult::Applied) {
          ++rep.failedBuilds;
          if (r == ToolApplyResult::InsufficientFunds) break;
          // Try a different candidate.
          continue;
        }

        ++rep.zonesBuilt;
        --remaining;

        // Opportunistically grow a small contiguous block from the seed tile.
        if (remaining <= 0) break;
        const int maxBlock = std::max(1, cfg.zoneClusterMaxTiles);
        if (maxBlock > 1) {
          const int maxExtra = std::min(remaining, maxBlock - 1);
          const int grown = GrowZoneCluster(world, zoneTool, zoneOv, zx, zy, day + zIter,
                                            maxExtra, cfg.minMoneyReserve);
          rep.zonesBuilt += grown;
          remaining -= grown;
        }
      }
    }

    // Expand roads if we're running low on adjacent buildable tiles.
    if (canSpend && cfg.roadsPerDay > 0) {
      for (int r = 0; r < cfg.roadsPerDay; ++r) {
        if (world.stats().money <= cfg.minMoneyReserve) break;
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
        int placed = 0;
        if (cfg.useRoadPlanner) {
          BuildPlannedRoadCorridor(world, simCfg, cfg,
                                   simCfg.requireOutsideConnection ? &roadToEdge : nullptr,
                                   &lv, zoneTool, day + r, &placed);
        } else {
          int sx = 0, sy = 0, dir = 0;
          if (!PickRoadExpansionStart(world, simCfg, cfg,
                                     simCfg.requireOutsideConnection ? &roadToEdge : nullptr,
                                     day + r, sx, sy, dir)) {
            break;
          }
          BuildRoadSpur(world, cfg, sx, sy, dir, day + r, &placed);
        }
        rep.roadsBuilt += placed;
        if (placed == 0) {
          ++rep.failedBuilds;
          break;
        }
      }
    }

    // Optional resilience bypasses: build redundant connections around bridge edges.
    if (cfg.autoBuildResilienceBypasses && canSpend && s.trafficCongestion >= cfg.resilienceBypassCongestionThreshold &&
        cfg.resilienceBypassesPerDay > 0) {
      int built = 0;
      int upgraded = 0;
      int failed = 0;
      BuildResilienceBypasses(world, simCfg, cfg, simCfg.requireOutsideConnection ? &roadToEdge : nullptr, day, &built, &upgraded, &failed);
      rep.roadsBuilt += built;
      rep.roadsUpgraded += upgraded;
      rep.failedBuilds += failed;
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
