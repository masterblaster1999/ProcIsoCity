#include "isocity/ProcGen.hpp"

#include "isocity/CityBlocks.hpp"
#include "isocity/Noise.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Hydrology.hpp"
#include "isocity/Road.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace isocity {

namespace {

// -----------------------------
// Helpers / small utilities
// -----------------------------

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline float Hash01From32(std::uint32_t h)
{
  // Use 24 bits of mantissa-like precision (matches other code in this file).
  return static_cast<float>((h >> 8) & 0x00FFFFFFu) / 16777216.0f;
}

inline float TileRand01(int x, int y, std::uint32_t seed)
{
  return Hash01From32(HashCoords32(x, y, seed));
}

inline std::size_t Idx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

struct P {
  int x = 0;
  int y = 0;
};

struct Edge {
  int a = 0;
  int b = 0;
  int dist = 0;
};

// -----------------------------
// Simple value noise based on HashCoords32.
// (We keep ProcGen deterministic and raylib-free.)
// -----------------------------

static float fade(float t)
{
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static float lerp(float a, float b, float t)
{
  return a + (b - a) * t;
}

static float noise2D(float x, float y, std::uint32_t seed)
{
  const int xi = static_cast<int>(std::floor(x));
  const int yi = static_cast<int>(std::floor(y));
  const float xf = x - static_cast<float>(xi);
  const float yf = y - static_cast<float>(yi);

  auto h01 = [&](int x0, int y0) {
    const std::uint32_t h = HashCoords32(x0, y0, seed);
    return Hash01From32(h);
  };

  const float n00 = h01(xi, yi);
  const float n10 = h01(xi + 1, yi);
  const float n01 = h01(xi, yi + 1);
  const float n11 = h01(xi + 1, yi + 1);

  const float u = fade(xf);
  const float v = fade(yf);

  const float x1 = lerp(n00, n10, u);
  const float x2 = lerp(n01, n11, u);
  return lerp(x1, x2, v);
}

static float fbmNormalized(float x, float y, std::uint32_t seed, int octaves)
{
  float total = 0.0f;
  float amp = 0.5f;
  float freq = 1.0f;
  float norm = 0.0f;

  for (int i = 0; i < octaves; ++i) {
    total += noise2D(x * freq, y * freq, seed + static_cast<std::uint32_t>(i) * 1013u) * amp;
    norm += amp;
    freq *= 2.0f;
    amp *= 0.5f;
  }

  return (norm > 0.0f) ? (total / norm) : 0.0f;
}

static float ridgedFbmNormalized(float x, float y, std::uint32_t seed, int octaves)
{
  float total = 0.0f;
  float amp = 0.5f;
  float freq = 1.0f;
  float norm = 0.0f;

  for (int i = 0; i < octaves; ++i) {
    float n = noise2D(x * freq, y * freq, seed + static_cast<std::uint32_t>(i) * 1013u);
    // Convert value-noise into a "ridged" 0..1 signal.
    n = 1.0f - std::fabs(n * 2.0f - 1.0f);
    n *= n;
    total += n * amp;
    norm += amp;
    freq *= 2.0f;
    amp *= 0.5f;
  }

  return (norm > 0.0f) ? (total / norm) : 0.0f;
}

// -----------------------------
// Road carving helpers
// -----------------------------

static P RandomLand(const World& world, RNG& rng)
{
  // Try a few random picks; fall back to scan.
  for (int i = 0; i < 2000; ++i) {
    const int x = rng.rangeInt(0, world.width() - 1);
    const int y = rng.rangeInt(0, world.height() - 1);
    const Tile& t = world.at(x, y);
    if (t.terrain != Terrain::Water) {
      return {x, y};
    }
  }

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain != Terrain::Water) {
        return {x, y};
      }
    }
  }

  // Degenerate: all water.
  return {world.width() / 2, world.height() / 2};
}

static bool WorldHasAnyWater(const World& world)
{
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      if (world.at(x, y).terrain == Terrain::Water) return true;
    }
  }
  return false;
}

// Add a small number of inland lakes by flooding large drainage basins.
//
// This leverages the existing hydrology utilities (D4 flow + basin segmentation)
// to create water bodies that feel "topographically plausible" without running a
// full depression-fill/overflow simulation.
//
// Key properties:
//  - deterministic (seed-driven)
//  - respects existing sea/water classification (won't spam lakes on very watery maps)
//  - produces irregular shapes (flooded contour) rather than perfect circles
static void AddProceduralLakes(World& world,
                               const std::vector<float>& heights,
                               float waterLevel,
                               float sandLevel,
                               std::uint32_t seed32)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (heights.size() != n) return;

  // Skip on extremely watery maps (already have plenty of water features).
  int existingWater = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (world.at(x, y).terrain == Terrain::Water) ++existingWater;
    }
  }
  const float waterFrac = (n > 0) ? (static_cast<float>(existingWater) / static_cast<float>(n)) : 0.0f;
  if (waterFrac >= 0.60f) return;

  const HydrologyField field = BuildHydrologyField(heights, w, h);
  if (field.empty()) return;

  const BasinSegmentation basins = SegmentBasins(field.dir, w, h);
  if (basins.empty() || basins.basins.empty()) return;

  struct Candidate {
    int basinId = -1;
    int sinkIndex = -1;
    int sx = 0;
    int sy = 0;
    int area = 0;
    float sinkH = 0.0f;
    float score = 0.0f;
  };

  const int totalArea = w * h;
  const int minBasinArea = std::max(80, totalArea / 180);
  const int minDim = std::min(w, h);

  // Prefer lowland basins (but not necessarily sea-level), reasonably away from the edge.
  const float targetH = waterLevel + 0.07f;
  const float hBand = 0.22f;

  std::vector<Candidate> cands;
  cands.reserve(64);

  // basins.basins is sorted by area desc; only scan the first handful for performance.
  const int maxScan = std::min(static_cast<int>(basins.basins.size()), 48);
  for (int i = 0; i < maxScan; ++i) {
    const BasinInfo& b = basins.basins[static_cast<std::size_t>(i)];
    if (b.area < minBasinArea) continue;

    const int sx = b.sinkX;
    const int sy = b.sinkY;

    const int edgeDist = std::min(std::min(sx, w - 1 - sx), std::min(sy, h - 1 - sy));
    if (edgeDist < 2) continue;

    const int sinkIndex = b.sinkIndex;
    if (sinkIndex < 0 || static_cast<std::size_t>(sinkIndex) >= n) continue;

    const float sinkH = heights[static_cast<std::size_t>(sinkIndex)];

    // Already under the sea/river classification.
    if (sinkH <= waterLevel + 0.01f) continue;
    // Too high => tends to look like a random puddle on a mountain plateau.
    if (sinkH > sandLevel + 0.35f) continue;

    const float areaN = static_cast<float>(b.area) / static_cast<float>(totalArea);
    const float lowland = Clamp01(1.0f - (std::fabs(sinkH - targetH) / hBand));
    const float edgeN = Clamp01(static_cast<float>(edgeDist) / static_cast<float>(std::max(1, minDim / 2)));
    const float rnd = TileRand01(sx, sy, seed32 ^ 0xC0FFEE11u);

    Candidate c;
    c.basinId = b.id;
    c.sinkIndex = sinkIndex;
    c.sx = sx;
    c.sy = sy;
    c.area = b.area;
    c.sinkH = sinkH;
    c.score = 0.55f * areaN + 0.30f * lowland + 0.10f * edgeN + 0.05f * rnd;
    cands.push_back(c);
  }

  if (cands.empty()) return;

  std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.area != b.area) return a.area > b.area;
    return a.sinkIndex < b.sinkIndex;
  });

  // Keep lake count small: we want rare landmarks, not a swamp map.
  int maxLakes = std::clamp(totalArea / 9000, 1, 4);
  if (waterFrac < 0.08f) {
    maxLakes = std::min(maxLakes + 1, 5);
  }

  const int minSep = std::clamp(minDim / 4, 14, 26);

  std::vector<Candidate> chosen;
  chosen.reserve(static_cast<std::size_t>(maxLakes));
  for (const Candidate& c : cands) {
    if (static_cast<int>(chosen.size()) >= maxLakes) break;
    bool ok = true;
    for (const Candidate& prev : chosen) {
      const int d = std::abs(c.sx - prev.sx) + std::abs(c.sy - prev.sy);
      if (d < minSep) {
        ok = false;
        break;
      }
    }
    if (ok) chosen.push_back(c);
  }
  if (chosen.empty()) return;

  // Global budget: don't flood too much of the map.
  const int maxExtraWater = static_cast<int>(static_cast<float>(totalArea) * 0.10f);
  int addedWater = 0;

  std::vector<std::uint8_t> visited(n, 0);
  std::vector<int> stack;
  stack.reserve(1024);
  std::vector<int> lakeCells;
  lakeCells.reserve(2048);

  constexpr int dx4[4] = {1, -1, 0, 0};
  constexpr int dy4[4] = {0, 0, 1, -1};

  for (const Candidate& c : chosen) {
    if (addedWater >= maxExtraWater) break;

    // Depth scales with basin area (bigger basin => deeper lake), plus a small jitter.
    const float areaN = static_cast<float>(c.area) / static_cast<float>(totalArea);
    const float depth = 0.018f + 0.070f * Clamp01(areaN * 10.0f);
    const float jitter = (TileRand01(c.sx, c.sy, seed32 ^ 0x9E3779B9u) - 0.5f) * 0.018f;

    float lakeLevel = c.sinkH + depth + jitter;
    lakeLevel = std::min(lakeLevel, c.sinkH + 0.14f);
    lakeLevel = std::min(lakeLevel, sandLevel + 0.10f);

    // Avoid creating lakes that would essentially be sea-level expansions.
    if (lakeLevel <= waterLevel + 0.02f) continue;

    visited.assign(n, 0);
    stack.clear();
    lakeCells.clear();

    if (c.sinkIndex < 0 || static_cast<std::size_t>(c.sinkIndex) >= n) continue;

    // Don't override already-water sinks (eg. rivers widened into a basin).
    if (world.at(c.sx, c.sy).terrain == Terrain::Water) continue;

    stack.push_back(c.sinkIndex);
    visited[static_cast<std::size_t>(c.sinkIndex)] = 1;

    while (!stack.empty()) {
      const int cur = stack.back();
      stack.pop_back();

      if (cur < 0 || cur >= totalArea) continue;
      if (basins.basinId[static_cast<std::size_t>(cur)] != c.basinId) continue;

      const int x = cur % w;
      const int y = cur / w;

      if (heights[static_cast<std::size_t>(cur)] > lakeLevel) continue;

      lakeCells.push_back(cur);

      for (int k = 0; k < 4; ++k) {
        const int nx = x + dx4[k];
        const int ny = y + dy4[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const int ni = ny * w + nx;
        const std::size_t uni = static_cast<std::size_t>(ni);
        if (visited[uni]) continue;
        if (basins.basinId[uni] != c.basinId) continue;
        if (heights[uni] > lakeLevel) continue;
        visited[uni] = 1;
        stack.push_back(ni);
      }
    }

    // Quality gates: avoid tiny puddles and avoid huge inland seas.
    if (lakeCells.size() < 12) continue;

    const std::size_t maxLakeTiles = std::max<std::size_t>(64, n / 10); // <= 10% of map
    if (lakeCells.size() > maxLakeTiles) continue;

    // Respect global water budget.
    if (addedWater + static_cast<int>(lakeCells.size()) > maxExtraWater) continue;

    int newlyFlooded = 0;
    for (const int idx : lakeCells) {
      const int x = idx % w;
      const int y = idx / w;
      Tile& t = world.at(x, y);
      if (t.terrain != Terrain::Water) {
        t.terrain = Terrain::Water;
        ++newlyFlooded;
      }
      // Lakes are terrain features; ensure we don't keep any overlay here.
      t.overlay = Overlay::None;
    }

    // Sandify immediate banks for legibility (and a more natural shoreline).
    const float bankMaxH = std::max(sandLevel, waterLevel + 0.02f) + 0.12f;
    for (const int idx : lakeCells) {
      const int x = idx % w;
      const int y = idx / w;
      for (int k = 0; k < 4; ++k) {
        const int nx = x + dx4[k];
        const int ny = y + dy4[k];
        if (!world.inBounds(nx, ny)) continue;
        Tile& t = world.at(nx, ny);
        if (t.terrain == Terrain::Water) continue;
        if (t.terrain == Terrain::Grass && t.height < bankMaxH) {
          t.terrain = Terrain::Sand;
        }
      }
    }

    addedWater += newlyFlooded;
  }
}

static float LocalSlopeMax4(const World& world, int x, int y)
{
  const float h0 = world.at(x, y).height;
  float best = 0.0f;
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int k = 0; k < 4; ++k) {
    const int nx = x + dx[k];
    const int ny = y + dy[k];
    if (!world.inBounds(nx, ny)) continue;
    best = std::max(best, std::fabs(h0 - world.at(nx, ny).height));
  }
  return best;
}

static void CountLandAndWaterInRadius(const World& world, int cx, int cy, int r, int& outTotal, int& outLand, int& outWater)
{
  outTotal = 0;
  outLand = 0;
  outWater = 0;

  for (int y = cy - r; y <= cy + r; ++y) {
    for (int x = cx - r; x <= cx + r; ++x) {
      if (!world.inBounds(x, y)) continue;
      ++outTotal;
      const Terrain t = world.at(x, y).terrain;
      if (t == Terrain::Water) {
        ++outWater;
      } else {
        ++outLand;
      }
    }
  }
}

static float ScoreHubCandidate(const World& world, int x, int y, bool hasAnyWater, std::uint32_t seed32)
{
  if (!world.inBounds(x, y)) return -1.0f;
  if (!world.isBuildable(x, y)) return -1.0f;

  // Prefer being a bit away from the very edge.
  const int edgeDist = std::min(std::min(x, world.width() - 1 - x), std::min(y, world.height() - 1 - y));

  // Prefer flatter spots.
  const float slope = LocalSlopeMax4(world, x, y);
  const float flatScore = std::exp(-slope * 38.0f); // ~1 on flat, decays quickly on steep tiles

  // Prefer areas with a good amount of buildable land nearby.
  int total = 0;
  int land = 0;
  int water = 0;
  CountLandAndWaterInRadius(world, x, y, /*r=*/6, total, land, water);
  const float landFrac = (total > 0) ? (static_cast<float>(land) / static_cast<float>(total)) : 0.0f;

  // Mildly prefer "near water" when water exists, but avoid points that are mostly water.
  float waterScore = 0.0f;
  if (hasAnyWater) {
    const float wFrac = (total > 0) ? (static_cast<float>(water) / static_cast<float>(total)) : 0.0f;
    const float target = 0.08f; // pleasant waterfront influence without being a tiny island
    const float denom = std::max(0.01f, target);
    waterScore = 1.0f - std::fabs(wFrac - target) / denom;
    waterScore = std::clamp(waterScore, 0.0f, 1.0f);
  }

  const float edgeNorm = std::clamp(static_cast<float>(edgeDist) / (0.5f * static_cast<float>(std::min(world.width(), world.height()))), 0.0f,
                                    1.0f);

  // Weighted sum + tiny deterministic tie-breaker.
  float score = 0.46f * flatScore + 0.34f * landFrac + 0.15f * edgeNorm + 0.05f * waterScore;
  score += 0.0001f * TileRand01(x, y, seed32 ^ 0x11CE5EEDu);
  return score;
}

static void SetRoadWithLevel(World& world, int x, int y, int level, bool allowBridges)
{
  if (!world.inBounds(x, y)) return;

  const Tile& t = world.at(x, y);
  if (t.terrain == Terrain::Water && !allowBridges) {
    // By default we avoid placing roads on water during procedural generation,
    // but higher-class connectors can opt-in to bridges.
    return;
  }

  world.setRoad(x, y);
  world.at(x, y).level = static_cast<std::uint8_t>(ClampRoadLevel(level));
}

static void CarveRoadWiggle(World& world, RNG& rng, P a, P b, int level, bool allowBridges)
{
  int x = a.x;
  int y = a.y;

  const int maxSteps = world.width() * world.height();
  for (int steps = 0; steps < maxSteps; ++steps) {
    SetRoadWithLevel(world, x, y, level, allowBridges);
    if (x == b.x && y == b.y) {
      break;
    }

    const int dx = (b.x > x) ? 1 : (b.x < x ? -1 : 0);
    const int dy = (b.y > y) ? 1 : (b.y < y ? -1 : 0);

    // Prefer moving toward target, but add a small wiggle.
    bool moveX = (std::abs(b.x - x) > std::abs(b.y - y));
    if (rng.chance(0.25f)) {
      moveX = !moveX;
    }

    if (moveX && dx != 0) {
      x += dx;
    } else if (dy != 0) {
      y += dy;
    } else if (dx != 0) {
      x += dx;
    }

    x = std::clamp(x, 0, world.width() - 1);
    y = std::clamp(y, 0, world.height() - 1);
  }
}

static void CarveRoad(World& world, RNG& rng, P a, P b, int level, bool allowBridges)
{
  const Point start{a.x, a.y};
  const Point goal{b.x, b.y};

  auto plan = [&](bool bridges,
                  RoadBuildPathConfig::CostModel model,
                  std::vector<Point>& outPath,
                  int* outPrimaryCost) -> bool {
    RoadBuildPathConfig cfg;
    cfg.costModel = model;
    cfg.targetLevel = ClampRoadLevel(level);
    cfg.allowBridges = bridges;

    // Terrain-aware routing: penalize steep slopes so higher-class roads
    // tend to follow valleys / gentle grades instead of "scaling" hills.
    //
    // This is intentionally small for minor roads and stronger for major
    // connectors (avenues/highways).
    const int minDim = std::min(world.width(), world.height());
    int slope = 0;
    if (level >= 3) {
      slope = 46;
    } else if (level >= 2) {
      slope = 36;
    } else {
      slope = 26;
    }
    if (minDim <= 64) {
      // Tiny maps: keep paths more direct.
      slope = std::max(0, slope - 10);
    }
    cfg.slopeCost = slope;
    cfg.slopeCostAffectsExistingRoads = false;

    return FindRoadBuildPath(world, start, goal, outPath, outPrimaryCost, cfg) && !outPath.empty();
  };

  // Always compute the land-only plan first. This keeps most roads "grounded" and
  // avoids surprise bridge spam in dense procedural street grids.
  std::vector<Point> landPath;
  int landCost = 0;
  const bool hasLand = plan(/*bridges=*/false, RoadBuildPathConfig::CostModel::NewTiles, landPath, &landCost);

  // Only consider bridges when explicitly allowed OR when land routing fails entirely.
  const bool tryBridges = (allowBridges || !hasLand);

  std::vector<Point> bridgePath;
  int bridgeCost = 0;
  const bool hasBridge = tryBridges
                             ? plan(/*bridges=*/true, RoadBuildPathConfig::CostModel::Money, bridgePath, &bridgeCost)
                             : false;

  bool chooseBridge = false;

  if (hasBridge && !hasLand) {
    chooseBridge = true;
  } else if (allowBridges && hasBridge && hasLand) {
    const int landSteps = static_cast<int>(landPath.size()) - 1;
    const int bridgeSteps = static_cast<int>(bridgePath.size()) - 1;

    int bridgeWaterTiles = 0;
    for (const Point& p : bridgePath) {
      if (world.at(p.x, p.y).terrain == Terrain::Water) ++bridgeWaterTiles;
    }

    const float ratio = (bridgeSteps > 0) ? (static_cast<float>(landSteps) / static_cast<float>(bridgeSteps)) : 0.0f;

    // Heuristic:
    // Prefer the bridge plan only when it avoids a meaningful detour AND the bridge
    // span is reasonable relative to the route length. The Dijkstra primary cost for
    // the bridge plan uses the Money model, which already penalizes bridge tiles by
    // kBridgeBuildCostMultiplier (see Road.hpp), so this mainly guards against
    // pathological "follow the river" solutions.
    const int maxBridgeTiles = std::max(3, bridgeSteps / 6);
    if (bridgeWaterTiles > 0 && ratio >= 1.25f && bridgeWaterTiles <= maxBridgeTiles) {
      chooseBridge = true;
    }
  }

  const std::vector<Point>* chosen = nullptr;
  bool chosenAllowBridges = false;

  if (chooseBridge && !bridgePath.empty()) {
    chosen = &bridgePath;
    chosenAllowBridges = true;
  } else if (hasLand && !landPath.empty()) {
    chosen = &landPath;
    chosenAllowBridges = false;
  } else if (hasBridge && !bridgePath.empty()) {
    // If we couldn't get a land path, fall back to bridges (even if allowBridges==false).
    chosen = &bridgePath;
    chosenAllowBridges = true;
  }

  if (chosen) {
    for (const Point& p : *chosen) {
      SetRoadWithLevel(world, p.x, p.y, level, chosenAllowBridges);
    }
    return;
  }

  // Fallback: a simple wiggly Manhattan carve.
  CarveRoadWiggle(world, rng, a, b, level, /*allowBridges=*/allowBridges || !hasLand);
}

static void CarveHubGrid(World& world, RNG& rng, P hub)
{
  // Create a small local grid around a hub.
  const int rad = 7 + rng.rangeInt(0, 3);

  for (int dy = -rad; dy <= rad; ++dy) {
    for (int dx = -rad; dx <= rad; ++dx) {
      const int x = hub.x + dx;
      const int y = hub.y + dy;
      if (!world.inBounds(x, y)) {
        continue;
      }
      if (!world.isBuildable(x, y)) {
        continue;
      }

      // Orthogonal grid lines.
      if (dx == 0 || dy == 0) {
        SetRoadWithLevel(world, x, y, 2, /*allowBridges=*/false);
        continue;
      }

      // Some secondary streets.
      if ((dx % 3 == 0 || dy % 3 == 0) && rng.chance(0.55f)) {
        SetRoadWithLevel(world, x, y, 1, /*allowBridges=*/false);
      }
    }
  }
}

static int ChooseHubConnectionLevel(const World& world, P a, P b)
{
  // Use distance to determine primary road class between hubs.
  const int dist = std::abs(a.x - b.x) + std::abs(a.y - b.y);
  const int diag = world.width() + world.height();

  if (dist > diag / 2) {
    return 3;
  }
  if (dist > diag / 4) {
    return 2;
  }
  return 1;
}

static int NearestHubDist(const std::vector<P>& hubs, int x, int y)
{
  int best = std::numeric_limits<int>::max();
  for (const P& h : hubs) {
    const int d = std::abs(h.x - x) + std::abs(h.y - y);
    best = std::min(best, d);
  }
  return best == std::numeric_limits<int>::max() ? 0 : best;
}

static int ManhattanDist(P a, P b)
{
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static std::uint32_t EdgeKeyU32(int a, int b)
{
  if (a > b) std::swap(a, b);
  return (static_cast<std::uint32_t>(a) << 16) ^ static_cast<std::uint32_t>(b);
}

static std::vector<Edge> BuildHubMST(const std::vector<P>& hubs)
{
  std::vector<Edge> edges;
  const int n = static_cast<int>(hubs.size());
  if (n <= 1) return edges;

  std::vector<int> bestDist(n, std::numeric_limits<int>::max());
  std::vector<int> parent(n, -1);
  std::vector<std::uint8_t> inTree(n, 0);

  bestDist[0] = 0;

  for (int iter = 0; iter < n; ++iter) {
    int u = -1;
    int uDist = std::numeric_limits<int>::max();

    for (int i = 0; i < n; ++i) {
      if (inTree[static_cast<std::size_t>(i)] != 0) continue;
      if (bestDist[static_cast<std::size_t>(i)] < uDist) {
        uDist = bestDist[static_cast<std::size_t>(i)];
        u = i;
      }
    }

    if (u < 0) break;
    inTree[static_cast<std::size_t>(u)] = 1;

    if (parent[static_cast<std::size_t>(u)] >= 0) {
      const int p = parent[static_cast<std::size_t>(u)];
      edges.push_back(Edge{p, u, ManhattanDist(hubs[static_cast<std::size_t>(p)], hubs[static_cast<std::size_t>(u)])});
    }

    for (int v = 0; v < n; ++v) {
      if (inTree[static_cast<std::size_t>(v)] != 0) continue;
      if (v == u) continue;

      const int d = ManhattanDist(hubs[static_cast<std::size_t>(u)], hubs[static_cast<std::size_t>(v)]);
      if (d < bestDist[static_cast<std::size_t>(v)] ||
          (d == bestDist[static_cast<std::size_t>(v)] && u < parent[static_cast<std::size_t>(v)])) {
        bestDist[static_cast<std::size_t>(v)] = d;
        parent[static_cast<std::size_t>(v)] = u;
      }
    }
  }

  return edges;
}


// -----------------------------
// Optional beltway ("ring road") generation
// -----------------------------

static bool FindNearestBuildableLand(const World& world, int cx, int cy, int maxR, std::uint32_t seed32, P& out)
{
  int bestD = std::numeric_limits<int>::max();
  std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();
  bool found = false;

  for (int dy = -maxR; dy <= maxR; ++dy) {
    for (int dx = -maxR; dx <= maxR; ++dx) {
      const int x = cx + dx;
      const int y = cy + dy;
      if (!world.inBounds(x, y)) continue;

      const int d = std::abs(dx) + std::abs(dy);
      if (d > maxR) continue;

      if (!world.isBuildable(x, y)) continue;

      const std::uint32_t tie = HashCoords32(x, y, seed32);
      if (d < bestD || (d == bestD && tie < bestTie)) {
        bestD = d;
        bestTie = tie;
        out = {x, y};
        found = true;
      }
    }
  }

  return found;
}

static void CarveBeltwayIfUseful(World& world, RNG& rng, const std::vector<P>& hubs, std::uint32_t seed32)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;
  if (hubs.size() < 3) return;

  const int minDim = std::min(w, h);
  if (minDim < 72) return; // too small for a meaningful beltway

  // Compute centroid of hubs.
  float cx = 0.0f;
  float cy = 0.0f;
  for (const P& p : hubs) {
    cx += static_cast<float>(p.x);
    cy += static_cast<float>(p.y);
  }
  cx /= static_cast<float>(hubs.size());
  cy /= static_cast<float>(hubs.size());

  // Compute mean Manhattan distance from centroid as a scale proxy.
  float meanD = 0.0f;
  for (const P& p : hubs) {
    meanD += std::fabs(static_cast<float>(p.x) - cx) + std::fabs(static_cast<float>(p.y) - cy);
  }
  meanD /= static_cast<float>(hubs.size());

  // Beltway radius: slightly beyond the hub cluster, clamped to keep inside bounds.
  const float margin = 8.0f;
  const float maxR =
      std::min(std::min(cx, static_cast<float>(w - 1) - cx), std::min(cy, static_cast<float>(h - 1) - cy)) - margin;
  if (maxR < 14.0f) return;

  float R = meanD * 1.15f + static_cast<float>(minDim) * 0.10f;
  R = std::clamp(R, static_cast<float>(minDim) * 0.22f, maxR);

  // Pick number of waypoints based on circumference / typical segment length.
  const float circumference = 6.2831853f * R;
  int points = static_cast<int>(std::round(circumference / 16.0f));
  points = std::clamp(points, 10, 16);

  std::vector<P> ring;
  ring.reserve(static_cast<std::size_t>(points));

  for (int i = 0; i < points; ++i) {
    const float ang = (6.2831853f * static_cast<float>(i)) / static_cast<float>(points);

    // Small jitter to avoid perfect circles.
    const float j = TileRand01(i, points, seed32 ^ 0xB17BEEFu) * 2.0f - 1.0f;
    const float r = R * (1.0f + 0.07f * j);

    const int tx = static_cast<int>(std::round(cx + r * std::cos(ang)));
    const int ty = static_cast<int>(std::round(cy + r * std::sin(ang)));

    P p{tx, ty};
    if (!FindNearestBuildableLand(world, tx, ty, /*maxR=*/6, seed32 ^ (0x9E3779B9u * static_cast<std::uint32_t>(i + 1)),
                                 p)) {
      continue;
    }

    ring.push_back(p);
  }

  // Remove consecutive duplicates (can happen if snapping picks the same land tile).
  std::vector<P> uniq;
  uniq.reserve(ring.size());
  for (const P& p : ring) {
    if (!uniq.empty() && uniq.back().x == p.x && uniq.back().y == p.y) continue;
    uniq.push_back(p);
  }
  if (uniq.size() >= 2 && uniq.front().x == uniq.back().x && uniq.front().y == uniq.back().y) {
    uniq.pop_back();
  }
  ring.swap(uniq);

  if (ring.size() < 6) return;

  // Carve segments with highway-ish level so it reads as a beltway.
  constexpr int beltwayLevel = 3;

  for (std::size_t i = 0; i < ring.size(); ++i) {
    const P a = ring[i];
    const P b = ring[(i + 1) % ring.size()];

    // Skip super short edges (degenerate waypoint placement).
    if (ManhattanDist(a, b) < 6) continue;

    CarveRoad(world, rng, a, b, beltwayLevel, /*allowBridges=*/true);
  }

  // Add a small number of spokes from hubs to the beltway (recognizable interchanges).
  const int maxSpokes = std::min(static_cast<int>(hubs.size()), 4);
  int spokes = 0;

  for (const P& hub : hubs) {
    if (spokes >= maxSpokes) break;

    // Find nearest beltway waypoint.
    int bestIdx = -1;
    int bestD = std::numeric_limits<int>::max();
    for (int i = 0; i < static_cast<int>(ring.size()); ++i) {
      const int d = ManhattanDist(hub, ring[static_cast<std::size_t>(i)]);
      if (d < bestD) {
        bestD = d;
        bestIdx = i;
      }
    }
    if (bestIdx < 0) continue;

    // Avoid spokes that are too short (hub already basically on the beltway).
    if (bestD < minDim / 7) continue;

    const P target = ring[static_cast<std::size_t>(bestIdx)];
    CarveRoad(world, rng, hub, target, /*level=*/2, /*allowBridges=*/true);
    ++spokes;
  }
}

// -----------------------------
// Zone selection (deterministic, uses hashed floats rather than RNG state)
// -----------------------------

static int MaxAdjacentRoadLevel(const World& world, int x, int y)
{
  int best = 0;
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int k = 0; k < 4; ++k) {
    const int nx = x + dx[k];
    const int ny = y + dy[k];
    if (!world.inBounds(nx, ny)) continue;
    const Tile& t = world.at(nx, ny);
    if (t.overlay != Overlay::Road) continue;
    best = std::max(best, ClampRoadLevel(static_cast<int>(t.level)));
  }

  return best;
}

static bool HasAdjacentWater4(const World& world, int x, int y)
{
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    const int nx = x + dx[k];
    const int ny = y + dy[k];
    if (!world.inBounds(nx, ny)) continue;
    if (world.at(nx, ny).terrain == Terrain::Water) return true;
  }
  return false;
}



// -----------------------------
// District-aware, land-value-driven zoning helpers
// -----------------------------

struct DistrictZoningProfile {
  float resW = 0.62f;
  float comW = 0.23f;
  float indW = 0.15f;
  float parkW = 0.07f;
};

struct DistrictZoningContext {
  std::array<DistrictZoningProfile, kDistrictCount> profile{};
  int cbdDistrict = 0;
  int waterfrontDistrict = -1;
  std::vector<int> industrialDistricts;
};

static DistrictZoningContext BuildDistrictZoningContext(const World& world, const std::vector<P>& hubs,
                                                        const LandValueResult& lvBase, std::uint32_t seed32)
{
  DistrictZoningContext ctx;
  ctx.industrialDistricts.clear();

  const int w = world.width();
  const int h = world.height();
  const int n = w * h;

  struct Acc {
    int count = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumLV = 0.0;
    double sumWater = 0.0;
    double sumPark = 0.0;
    int minEdgeDist = std::numeric_limits<int>::max();
    int waterAdj = 0;
  };

  std::array<Acc, kDistrictCount> acc{};

  const bool lvOk = (lvBase.value.size() == static_cast<std::size_t>(n));

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) continue;

      const int d = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
      Acc& a = acc[static_cast<std::size_t>(d)];
      a.count++;
      a.sumX += x;
      a.sumY += y;

      const int edgeDist = std::min(std::min(x, w - 1 - x), std::min(y, h - 1 - y));
      a.minEdgeDist = std::min(a.minEdgeDist, edgeDist);

      if (HasAdjacentWater4(world, x, y)) {
        a.waterAdj++;
      }

      if (lvOk) {
        const std::size_t idx = Idx(x, y, w);
        a.sumLV += lvBase.value[idx];
        if (idx < lvBase.waterAmenity.size()) a.sumWater += lvBase.waterAmenity[idx];
        if (idx < lvBase.parkAmenity.size()) a.sumPark += lvBase.parkAmenity[idx];
      }
    }
  }

  struct DInfo {
    int id = 0;
    float avgLV = 0.5f;
    float avgWater = 0.0f;
    float waterAdjRatio = 0.0f;
    float edge01 = 0.0f;
    float hubD01 = 0.5f;
  };

  std::array<DInfo, kDistrictCount> info{};
  for (int d = 0; d < kDistrictCount; ++d) {
    DInfo di;
    di.id = d;
    const Acc& a = acc[static_cast<std::size_t>(d)];
    if (a.count > 0) {
      const float inv = 1.0f / static_cast<float>(a.count);
      const int cx = static_cast<int>(std::round(a.sumX * inv));
      const int cy = static_cast<int>(std::round(a.sumY * inv));
      const int hubDist = NearestHubDist(hubs, cx, cy);
      di.hubD01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(w + h), 0.0f, 1.0f);

      di.avgLV = lvOk ? std::clamp(static_cast<float>(a.sumLV * inv), 0.0f, 1.0f) : 0.5f;
      di.avgWater = lvOk ? std::clamp(static_cast<float>(a.sumWater * inv), 0.0f, 1.0f) : 0.0f;
      di.waterAdjRatio = std::clamp(static_cast<float>(a.waterAdj) * inv, 0.0f, 1.0f);

      const float edgeDenom = std::max(1.0f, static_cast<float>(std::min(w, h)) * 0.5f);
      const float edgeDist = (a.minEdgeDist == std::numeric_limits<int>::max()) ? edgeDenom : static_cast<float>(a.minEdgeDist);
      di.edge01 = 1.0f - std::clamp(edgeDist / edgeDenom, 0.0f, 1.0f);
    } else {
      di.hubD01 = 1.0f;
      di.avgLV = 0.4f;
      di.avgWater = 0.0f;
      di.waterAdjRatio = 0.0f;
      di.edge01 = 0.0f;
    }
    info[static_cast<std::size_t>(d)] = di;
  }

  auto pickBest = [&](auto scoreFn, int forbidA, int forbidB) -> int {
    float best = -1e9f;
    int bestId = 0;
    for (int d = 0; d < kDistrictCount; ++d) {
      if (d == forbidA || d == forbidB) continue;
      const float s = scoreFn(info[static_cast<std::size_t>(d)]);
      if (s > best) {
        best = s;
        bestId = d;
      }
    }
    return bestId;
  };

  ctx.cbdDistrict = pickBest([](const DInfo& di) {
    return 0.55f * (1.0f - di.hubD01) + 0.45f * di.avgLV;
  }, -1, -1);

  ctx.waterfrontDistrict = pickBest([](const DInfo& di) {
    const float wscore = 0.55f * di.avgWater + 0.25f * di.waterAdjRatio;
    return wscore + 0.20f * di.avgLV + 0.10f * (1.0f - di.hubD01);
  }, ctx.cbdDistrict, -1);

  const int wantIndustrial = (std::min(w, h) >= 80 && static_cast<int>(hubs.size()) >= 4) ? 2 : 1;

  struct ScoredId { float s; int id; };
  std::vector<ScoredId> indCands;
  indCands.reserve(kDistrictCount);

  for (int d = 0; d < kDistrictCount; ++d) {
    if (d == ctx.cbdDistrict || d == ctx.waterfrontDistrict) continue;
    const DInfo& di = info[static_cast<std::size_t>(d)];
    float s = 0.45f * di.edge01 + 0.25f * di.hubD01 + 0.20f * (1.0f - di.avgLV) + 0.10f * (1.0f - di.avgWater);
    s += TileRand01(d, 0, seed32 ^ 0x1D15EA5Eu) * 0.02f;
    indCands.push_back(ScoredId{s, d});
  }

  std::sort(indCands.begin(), indCands.end(), [](const ScoredId& a, const ScoredId& b) {
    if (a.s != b.s) return a.s > b.s;
    return a.id < b.id;
  });

  for (int i = 0; i < wantIndustrial && i < static_cast<int>(indCands.size()); ++i) {
    ctx.industrialDistricts.push_back(indCands[static_cast<std::size_t>(i)].id);
  }
  if (ctx.industrialDistricts.empty()) {
    ctx.industrialDistricts.push_back(ctx.cbdDistrict);
  }

  // Base profile everywhere.
  for (int d = 0; d < kDistrictCount; ++d) {
    ctx.profile[static_cast<std::size_t>(d)] = DistrictZoningProfile{};

    // Small deterministic per-district variation so maps don't feel too uniform.
    const float j = TileRand01(d, 1, seed32 ^ 0xC0FFEEu);
    if (j < 0.18f) {
      // Slightly more commercial.
      auto& p = ctx.profile[static_cast<std::size_t>(d)];
      p.comW *= 1.18f;
      p.indW *= 0.92f;
    } else if (j > 0.88f) {
      // Slightly more industrial.
      auto& p = ctx.profile[static_cast<std::size_t>(d)];
      p.indW *= 1.15f;
      p.comW *= 0.95f;
      p.resW *= 0.95f;
    }
  }

  // Apply special district profiles.
  ctx.profile[static_cast<std::size_t>(ctx.cbdDistrict)] = DistrictZoningProfile{0.34f, 0.58f, 0.08f, 0.05f};
  ctx.profile[static_cast<std::size_t>(ctx.waterfrontDistrict)] = DistrictZoningProfile{0.55f, 0.28f, 0.05f, 0.17f};
  for (int id : ctx.industrialDistricts) {
    ctx.profile[static_cast<std::size_t>(std::clamp(id, 0, kDistrictCount - 1))] = DistrictZoningProfile{0.20f, 0.18f, 0.62f, 0.08f};
  }

  return ctx;
}

static std::vector<P> PickIndustrialAnchors(const World& world,
                                            const std::vector<std::vector<Point>>& blockRoadAdj,
                                            const std::vector<P>& hubs,
                                            const LandValueResult& lvBase,
                                            const std::vector<int>& industrialDistricts,
                                            std::uint32_t seed32)
{
  std::vector<P> anchors;
  anchors.reserve(industrialDistricts.size());

  const int w = world.width();
  const int h = world.height();
  const int n = w * h;
  const bool lvOk = (lvBase.value.size() == static_cast<std::size_t>(n));

  // Flatten road-adjacent tiles for fast scanning.
  std::vector<Point> all;
  all.reserve(4096);
  for (const auto& v : blockRoadAdj) {
    for (const Point& p : v) {
      all.push_back(p);
    }
  }

  const int minSpacing = std::max(10, std::min(w, h) / 4);

  for (int didRaw : industrialDistricts) {
    const int did = std::clamp(didRaw, 0, kDistrictCount - 1);
    float bestScore = -1e9f;
    P best{-1, -1};

    for (const Point& p : all) {
      if (!world.inBounds(p.x, p.y)) continue;
      const Tile& t = world.at(p.x, p.y);
      if (t.terrain == Terrain::Water) continue;
      if (t.overlay != Overlay::None) continue; // don't overwrite parks, etc.
      if (static_cast<int>(t.district) != did) continue;

      const int roadLevel = MaxAdjacentRoadLevel(world, p.x, p.y);
      if (roadLevel <= 0) continue;

      const std::size_t idx = Idx(p.x, p.y, w);
      const float lv = lvOk ? lvBase.value[idx] : 0.5f;
      const float water = (lvOk && idx < lvBase.waterAmenity.size()) ? lvBase.waterAmenity[idx]
                                                                    : (HasAdjacentWater4(world, p.x, p.y) ? 1.0f : 0.0f);

      const int hubDist = NearestHubDist(hubs, p.x, p.y);
      const float d01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(w + h), 0.0f, 1.0f);

      const int edgeDist = std::min(std::min(p.x, w - 1 - p.x), std::min(p.y, h - 1 - p.y));
      const float edge01 = 1.0f - std::clamp(static_cast<float>(edgeDist) / std::max(1.0f, static_cast<float>(std::min(w, h)) * 0.5f),
                                             0.0f, 1.0f);

      float s = 0.0f;
      s += (1.0f - lv) * 0.46f;
      s += (1.0f - water) * 0.14f;
      s += edge01 * 0.18f;
      s += (static_cast<float>(roadLevel) / 3.0f) * 0.14f;
      s += d01 * 0.08f;

      // Avoid placing multiple anchors right on top of each other.
      for (const P& a : anchors) {
        const int d = std::abs(a.x - p.x) + std::abs(a.y - p.y);
        if (d < minSpacing) {
          s -= (static_cast<float>(minSpacing - d) / static_cast<float>(minSpacing)) * 0.35f;
        }
      }

      // Tiny deterministic jitter for tie-breaking.
      s += TileRand01(p.x, p.y, seed32 ^ 0x51A71D00u) * 0.02f;

      if (s > bestScore) {
        bestScore = s;
        best = {p.x, p.y};
      }
    }

    if (best.x >= 0) {
      anchors.push_back(best);
    }
  }

  return anchors;
}

static Overlay PickZoneTypeLandValue(const DistrictZoningProfile& profile, float d01, int roadLevel, bool nearWater,
                                     float landValue, float waterAmenity, float parkAmenity, float pollution,
                                     float r01, const ProcGenConfig& cfg)
{
  // Start with district-level mix.
  float resW = profile.resW;
  float comW = profile.comW;
  float indW = profile.indW;
  float parkW = profile.parkW;

  // Respect global parkChance by scaling the park weight.
  if (cfg.parkChance <= 0.0f) {
    parkW = 0.0f;
  } else {
    const float scale = std::clamp(cfg.parkChance / 0.06f, 0.0f, 2.0f);
    parkW *= scale;
  }

  // Hub proximity: more commerce + density near the core.
  if (d01 < 0.25f) {
    comW *= 1.25f;
    indW *= 0.85f;
  } else if (d01 > 0.65f) {
    indW *= 1.10f;
  }

  // Road class: strip commerce/industry along bigger roads.
  if (roadLevel >= 2) {
    resW *= 0.80f;
    comW *= 1.25f;
    indW *= 1.15f;
    parkW *= 0.85f;
  }
  if (roadLevel >= 3) {
    resW *= 0.60f;
    comW *= 1.35f;
    indW *= 1.30f;
    parkW *= 0.75f;
  }

  // Land value: attractive areas pull res/com, cheap areas pull industry.
  const float v = Clamp01(landValue);
  resW *= (0.60f + 0.90f * v);
  comW *= (0.55f + 0.95f * v);
  indW *= (0.70f + 0.95f * (1.0f - v));

  // Water frontage & amenity: pushes away industry, encourages parks + res/com.
  const float w = Clamp01(std::max(waterAmenity, nearWater ? 1.0f : 0.0f));
  resW *= (1.0f + 0.22f * w);
  comW *= (1.0f + 0.40f * w);
  indW *= (1.0f - 0.70f * w);
  indW = std::max(0.01f, indW);

  // Park placement: prefer areas that are lacking park amenity, and slightly prefer water.
  const float pNeed = 1.0f - Clamp01(parkAmenity);
  parkW *= (0.55f + 0.70f * pNeed + 0.45f * w);

  // Pollution: avoid res/com in polluted areas; parks + industry tolerate it more.
  const float pol = Clamp01(pollution);
  resW *= (1.0f - 0.85f * pol);
  comW *= (1.0f - 0.70f * pol);
  indW *= (1.0f + 0.35f * pol);
  parkW *= (1.0f + 0.55f * pol);

  const float sum = resW + comW + indW + parkW;
  if (sum <= 0.0f) {
    return Overlay::Residential;
  }

  const float r = Clamp01(r01) * sum;
  if (r < resW) return Overlay::Residential;
  if (r < resW + comW) return Overlay::Commercial;
  if (r < resW + comW + indW) return Overlay::Industrial;
  return Overlay::Park;
}

static int PickZoneLevelLandValue(Overlay zone, float d01, int roadLevel, float landValue, float pollution,
                                  float r0, float r1, float r2, float r3)
{
  int lvl = 1;

  const float v = Clamp01(landValue);
  float score = 0.0f;

  if (zone == Overlay::Industrial) {
    // Industry tends to cluster in cheaper land but benefits from access to bigger roads.
    score = 0.45f * (static_cast<float>(roadLevel) / 3.0f) + 0.35f * (1.0f - v) + 0.20f * d01;
  } else if (zone == Overlay::Commercial) {
    score = 0.55f * (1.0f - d01) + 0.45f * v;
  } else if (zone == Overlay::Residential) {
    score = 0.50f * (1.0f - d01) + 0.50f * v;
  } else {
    score = 0.35f * (1.0f - d01) + 0.25f * v + 0.40f * (1.0f - Clamp01(pollution));
  }

  score = Clamp01(score);

  if (score > 0.62f && r0 < 0.55f) lvl = 2;
  if (score > 0.78f && r1 < 0.28f) lvl = 3;

  if (roadLevel >= 2 && r2 < 0.30f) lvl = std::max(lvl, 2);
  if (roadLevel >= 3 && r3 < 0.22f) lvl = 3;

  return std::clamp(lvl, 1, 3);
}

// -----------------------------
// Internal street carving inside large city blocks
// -----------------------------

static bool FindPathInBlock(const CityBlocksResult& cb, int blockId, Point start, Point goal, std::vector<Point>& outPath)
{
  outPath.clear();
  const int w = cb.w;
  const int h = cb.h;
  if (w <= 0 || h <= 0) return false;
  if (start.x < 0 || start.y < 0 || start.x >= w || start.y >= h) return false;
  if (goal.x < 0 || goal.y < 0 || goal.x >= w || goal.y >= h) return false;

  const int sIdx = start.y * w + start.x;
  const int gIdx = goal.y * w + goal.x;

  if (static_cast<std::size_t>(sIdx) >= cb.tileToBlock.size()) return false;
  if (static_cast<std::size_t>(gIdx) >= cb.tileToBlock.size()) return false;

  if (cb.tileToBlock[static_cast<std::size_t>(sIdx)] != blockId) return false;
  if (cb.tileToBlock[static_cast<std::size_t>(gIdx)] != blockId) return false;

  std::vector<int> prev;
  prev.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);

  std::vector<int> q;
  q.reserve(1024);
  std::size_t head = 0;

  q.push_back(sIdx);
  prev[static_cast<std::size_t>(sIdx)] = sIdx;

  const int nx[4] = {-1, 1, 0, 0};
  const int ny[4] = {0, 0, -1, 1};

  while (head < q.size()) {
    const int cur = q[head++];
    if (cur == gIdx) break;

    const int cx = cur % w;
    const int cy = cur / w;

    for (int k = 0; k < 4; ++k) {
      const int x2 = cx + nx[k];
      const int y2 = cy + ny[k];
      if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;

      const int nIdx = y2 * w + x2;
      if (prev[static_cast<std::size_t>(nIdx)] != -1) continue;

      if (cb.tileToBlock[static_cast<std::size_t>(nIdx)] != blockId) continue;

      prev[static_cast<std::size_t>(nIdx)] = cur;
      q.push_back(nIdx);
    }
  }

  if (prev[static_cast<std::size_t>(gIdx)] == -1) {
    return false;
  }

  // Reconstruct path.
  int cur = gIdx;
  while (cur != sIdx) {
    outPath.push_back(Point{cur % w, cur / w});
    cur = prev[static_cast<std::size_t>(cur)];
  }
  outPath.push_back(start);
  std::reverse(outPath.begin(), outPath.end());
  return true;
}

static void BuildBlockRoadAdj(const World& world, const CityBlocksResult& cb, std::vector<std::vector<Point>>& outRoadAdj)
{
  outRoadAdj.clear();
  outRoadAdj.resize(cb.blocks.size());

  const int w = cb.w;
  const int h = cb.h;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = y * w + x;
      const int bid = (static_cast<std::size_t>(idx) < cb.tileToBlock.size()) ? cb.tileToBlock[static_cast<std::size_t>(idx)] : -1;
      if (bid < 0) continue;

      if (world.hasAdjacentRoad(x, y)) {
        outRoadAdj[static_cast<std::size_t>(bid)].push_back(Point{x, y});
      }
    }
  }
}

static Point PickByHash(const std::vector<Point>& pts, std::uint32_t seed32)
{
  Point best = pts.front();
  std::uint32_t bestH = std::numeric_limits<std::uint32_t>::max();

  for (const Point& p : pts) {
    const std::uint32_t h = HashCoords32(p.x, p.y, seed32);
    if (h < bestH) {
      bestH = h;
      best = p;
    }
  }

  return best;
}

static bool PickOppositeRoadAdjPair(const CityBlock& b,
                                   const std::vector<Point>& adj,
                                   bool vertical,
                                   std::uint32_t seed32,
                                   Point& outA,
                                   Point& outB)
{
  if (adj.size() < 2) return false;

  int minAxis = std::numeric_limits<int>::max();
  int maxAxis = std::numeric_limits<int>::min();

  for (const Point& p : adj) {
    const int a = vertical ? p.y : p.x;
    minAxis = std::min(minAxis, a);
    maxAxis = std::max(maxAxis, a);
  }

  const int range = maxAxis - minAxis;
  if (range < 6) return false;

  // Candidate band near each extreme, clamped so we still get variety on large blocks.
  const int band = std::clamp(range / 5, 1, 4);

  std::vector<Point> side0;
  std::vector<Point> side1;
  side0.reserve(adj.size() / 3u + 2u);
  side1.reserve(adj.size() / 3u + 2u);

  for (const Point& p : adj) {
    const int a = vertical ? p.y : p.x;
    if (a <= minAxis + band) side0.push_back(p);
    if (a >= maxAxis - band) side1.push_back(p);
  }

  if (side0.empty() || side1.empty()) return false;

  outA = PickByHash(side0, seed32 ^ 0xA341316Cu);
  outB = PickByHash(side1, seed32 ^ 0xC8013EA4u);

  if (outA.x == outB.x && outA.y == outB.y) return false;

  // Ensure meaningful separation (avoid carving tiny stubs on the same side).
  const int d = std::abs(outA.x - outB.x) + std::abs(outA.y - outB.y);
  if (d < 10) return false;

  // Defensive: endpoints should lie within the block bbox.
  if (outA.x < b.minX || outA.x > b.maxX || outA.y < b.minY || outA.y > b.maxY) return false;
  if (outB.x < b.minX || outB.x > b.maxX || outB.y < b.minY || outB.y > b.maxY) return false;

  return true;
}

static bool PathHasInteriorCoverage(const CityBlock& b, const std::vector<Point>& path)
{
  if (path.empty()) return false;

  int maxInset = 0;
  for (const Point& p : path) {
    const int inset =
        std::min(std::min(p.x - b.minX, b.maxX - p.x), std::min(p.y - b.minY, b.maxY - p.y));
    maxInset = std::max(maxInset, inset);
  }

  // If the whole path hugs the boundary, it doesn't meaningfully subdivide the block.
  return maxInset >= 2;
}

static Point PickFarthestFrom(const std::vector<Point>& pts, Point from, std::uint32_t seed32)
{
  int bestD = -1;
  std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();
  Point best = from;

  for (const Point& q : pts) {
    const int d = std::abs(q.x - from.x) + std::abs(q.y - from.y);
    const std::uint32_t tie = HashCoords32(q.x, q.y, seed32);
    if (d > bestD || (d == bestD && tie < bestTie)) {
      bestD = d;
      bestTie = tie;
      best = q;
    }
  }

  return best;
}

static bool CarveOneBlockConnector(World& world,
                                   const CityBlocksResult& cb,
                                   const CityBlock& b,
                                   const std::vector<Point>& adj,
                                   bool vertical,
                                   int level,
                                   std::uint32_t seed32,
                                   std::vector<Point>& pathScratch)
{
  Point a{0, 0};
  Point c{0, 0};

  bool picked = PickOppositeRoadAdjPair(b, adj, vertical, seed32, a, c);
  if (!picked) {
    // Fallback: farthest-pair heuristic. This still yields useful results even
    // when the block is only bounded by roads on 2 sides.
    const std::uint32_t pick = HashCoords32(b.id, 17, seed32 ^ 0xA5A5F00Du);
    const Point p0 = adj[static_cast<std::size_t>(pick % static_cast<std::uint32_t>(adj.size()))];

    const Point p1 = PickFarthestFrom(adj, p0, seed32 ^ 0x1BADB002u);
    const Point p2 = PickFarthestFrom(adj, p1, seed32 ^ 0xC0DEC0DEu);

    if (p1.x == p2.x && p1.y == p2.y) return false;
    a = p1;
    c = p2;
  }

  if (!FindPathInBlock(cb, b.id, a, c, pathScratch)) {
    return false;
  }

  // Avoid carving trivial stubs.
  if (pathScratch.size() < 12) {
    return false;
  }
  if (!PathHasInteriorCoverage(b, pathScratch)) {
    return false;
  }

  for (const Point& p : pathScratch) {
    SetRoadWithLevel(world, p.x, p.y, level, /*allowBridges=*/false);
  }

  return true;
}

static void CarveInternalStreets(World& world, const std::vector<P>& hubs, std::uint32_t seed32)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  const int diag = w + h;
  const int minDim = std::min(w, h);

  // Multi-pass block subdivision:
  //  - pass 0: collector spines in very large blocks (level 2)
  //  - pass 1+: local streets in remaining oversized blocks (level 1)
  //
  // This yields a hierarchical street network and avoids huge monolithic blocks,
  // especially near hubs (where we want smaller blocks / higher permeability).
  const int passes = (minDim >= 80) ? 3 : 2;

  std::vector<Point> path;

  for (int pass = 0; pass < passes; ++pass) {
    CityBlocksResult cb = BuildCityBlocks(world);
    if (cb.blocks.empty()) break;

    std::vector<std::vector<Point>> roadAdj;
    BuildBlockRoadAdj(world, cb, roadAdj);

    const int minRoadEdges = (pass == 0) ? 16 : 12;

    for (const CityBlock& b : cb.blocks) {
      const int bid = b.id;
      if (bid < 0) continue;
      if (static_cast<std::size_t>(bid) >= roadAdj.size()) continue;

      const auto& adj = roadAdj[static_cast<std::size_t>(bid)];
      if (adj.size() < 2) continue;

      // Only subdivide blocks that are meaningfully bounded by roads.
      if (b.roadEdges < minRoadEdges) continue;
      if (b.area < 120) continue; // tiny blocks are fine as-is

      // Density gradient: near hubs we want smaller target blocks.
      const int cx = (b.minX + b.maxX) / 2;
      const int cy = (b.minY + b.maxY) / 2;
      const int hubDist = NearestHubDist(hubs, cx, cy);
      const float d01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(std::max(1, diag)), 0.0f, 1.0f);

      // Desired block size grows with distance from hubs.
      const float targetArea = lerp(90.0f, 280.0f, d01);

      // Pass thresholds: earlier passes only attack very large blocks.
      float thresh = targetArea;
      if (pass == 0) {
        thresh *= 2.20f;
      } else if (pass == 1) {
        thresh *= 1.55f;
      } else {
        thresh *= 1.25f;
      }

      if (static_cast<float>(b.area) < thresh) continue;

      // Choose number of connectors based on how oversized the block is.
      int connectors = 1;
      const float over = static_cast<float>(b.area) / std::max(1.0f, targetArea);

      if (pass == 0) {
        if (over >= 4.0f && adj.size() >= 10) {
          connectors = 3;
        } else if (over >= 2.7f && adj.size() >= 8) {
          connectors = 2;
        }
      } else if (pass == 1) {
        if (over >= 3.2f && adj.size() >= 10) {
          connectors = 2;
        }
      }

      connectors = std::clamp(connectors, 1, 3);

      // Prefer adding streets orthogonal to the longer dimension first.
      const int bw = b.maxX - b.minX + 1;
      const int bh = b.maxY - b.minY + 1;
      const bool preferVertical = (bw >= bh * 12 / 10); // wide blocks => north-south streets

      const int level = (pass == 0) ? 2 : 1;

      for (int c = 0; c < connectors; ++c) {
        const std::uint32_t cseed = seed32 ^ HashCoords32(bid, pass * 31 + c, seed32 ^ 0x9E3779B9u);
        const bool vertical = (c % 2 == 0) ? preferVertical : !preferVertical;

        if (CarveOneBlockConnector(world, cb, b, adj, vertical, level, cseed, path)) {
          continue;
        }
        // Fallback: try the other orientation.
        CarveOneBlockConnector(world, cb, b, adj, !vertical, level, cseed ^ 0xDEADBEEFu, path);
      }
    }
  }
}

// -----------------------------
// Zoning pass: seed from road-adjacent tiles and grow inward within blocks
// -----------------------------


static void PlaceZonesAndParksFromBlocks(World& world, const std::vector<P>& hubs, std::uint32_t seed32, const ProcGenConfig& cfg)
{
  CityBlocksResult cb = BuildCityBlocks(world);
  if (cb.blocks.empty()) return;

  const int w = cb.w;
  const int h = cb.h;
  const int n = w * h;

  std::vector<std::vector<Point>> blockRoadAdj;
  BuildBlockRoadAdj(world, cb, blockRoadAdj);

  // -------------------------------------------------------------------------
  // Phase A: seed parks along road edges (still gated by zoneChance so "no development"
  // configs remain stable for tests/experiments).
  // -------------------------------------------------------------------------
  if (cfg.zoneChance > 0.0f && cfg.parkChance > 0.0f) {
    for (const CityBlock& b : cb.blocks) {
      const int bid = b.id;
      if (bid < 0) continue;

      const auto& roadAdj = blockRoadAdj[static_cast<std::size_t>(bid)];
      for (const Point& p : roadAdj) {
        Tile& t = world.at(p.x, p.y);
        if (t.overlay != Overlay::None) continue;

        // Deterministic per-tile "developable" gate (shared with zoning).
        const float gate = TileRand01(p.x, p.y, seed32 ^ 0xDEADBEEFu);
        if (gate >= cfg.zoneChance) {
          continue;
        }

        const int roadLevel = MaxAdjacentRoadLevel(world, p.x, p.y);
        if (roadLevel <= 0) continue;

        const bool nearWater = HasAdjacentWater4(world, p.x, p.y);
        const int hubDist = NearestHubDist(hubs, p.x, p.y);
        const float d01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(w + h), 0.0f, 1.0f);

        // Park chance: a touch higher near hubs, but reduced along highways.
        float parkChance = cfg.parkChance;
        if (d01 < 0.25f) {
          parkChance += 0.05f;
        }
        if (nearWater) {
          // Waterfront + riverfront parks are a strong visual anchor.
          parkChance = parkChance * 1.25f + 0.08f;
        }
        if (roadLevel >= 3) {
          parkChance *= 0.60f;
        }
        parkChance = std::clamp(parkChance, 0.0f, 1.0f);

        const float rPark = TileRand01(p.x, p.y, seed32 ^ 0xBEEF1234u);
        if (rPark < parkChance) {
          t.overlay = Overlay::Park;
          t.level = 1;
          t.occupants = 0;
        }
      }
    }
  }

  // -------------------------------------------------------------------------
  // Land value field (parks contribute positive amenity; later, industrial anchors add pollution).
  // -------------------------------------------------------------------------
  std::vector<std::uint8_t> roadToEdge;
  ComputeRoadsConnectedToEdge(world, roadToEdge);

  LandValueConfig lvCfg; // defaults are tuned for in-game desirability/heatmaps
  LandValueResult lvBase = ComputeLandValue(world, lvCfg, nullptr, &roadToEdge);

  // Build per-district "zoning personality" so cities read as coherent regions
  // (CBD, waterfront, industrial/logistics).
  const DistrictZoningContext dz = BuildDistrictZoningContext(world, hubs, lvBase, seed32);

  // -------------------------------------------------------------------------
  // Phase B: place 1..2 industrial anchors in industrial-oriented districts.
  // These anchors seed early pollution so subsequent land-value-driven zoning
  // naturally buffers them with lower-value uses and parks.
  // -------------------------------------------------------------------------
  if (cfg.zoneChance > 0.0f) {
    const std::vector<P> anchors = PickIndustrialAnchors(world, blockRoadAdj, hubs, lvBase, dz.industrialDistricts, seed32 ^ 0xA11CEB0Bu);
    for (const P& a : anchors) {
      if (!world.inBounds(a.x, a.y)) continue;
      Tile& t = world.at(a.x, a.y);
      if (t.overlay != Overlay::None) continue;

      const int roadLevel = MaxAdjacentRoadLevel(world, a.x, a.y);
      if (roadLevel <= 0) continue;

      const int lvl = (roadLevel >= 3) ? 3 : (roadLevel >= 2 ? 2 : 1);

      t.overlay = Overlay::Industrial;
      t.level = static_cast<std::uint8_t>(lvl);
      t.occupants = 1;
    }
  }

  // Recompute land value now that industrial anchors exist (pollution field becomes meaningful).
  LandValueResult lv = ComputeLandValue(world, lvCfg, nullptr, &roadToEdge);
  const bool lvOk = (lv.value.size() == static_cast<std::size_t>(n));

  struct Seed {
    int idx = 0;
    Overlay zone = Overlay::None;
    std::uint8_t level = 1;
    std::uint8_t roadLevel = 1;
  };

  // Reusable scratch buffers for per-block BFS.
  std::vector<int> dist(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);
  std::vector<int> owner(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);
  std::vector<int> q;
  std::vector<int> order;
  std::vector<int> touched;
  q.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 4u);
  order.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 4u);
  touched.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 4u);

  const int nx[4] = {-1, 1, 0, 0};
  const int ny[4] = {0, 0, -1, 1};

  // -------------------------------------------------------------------------
  // Phase C: seed zones on road-adjacent tiles, then grow inward within blocks.
  // Zone selection is driven by land value + district context + road class.
  // -------------------------------------------------------------------------
  for (const CityBlock& b : cb.blocks) {
    const int bid = b.id;
    if (bid < 0) continue;

    const auto& roadAdj = blockRoadAdj[static_cast<std::size_t>(bid)];
    if (roadAdj.empty()) {
      continue;
    }

    std::vector<Seed> seeds;
    seeds.reserve(roadAdj.size() / 2u + 8u);

    for (const Point& p : roadAdj) {
      Tile& t = world.at(p.x, p.y);

      if (t.overlay == Overlay::Park) {
        continue;
      }

      const int roadLevel = MaxAdjacentRoadLevel(world, p.x, p.y);
      if (roadLevel <= 0) {
        continue;
      }

      // Include pre-placed industrial anchors as seeds so they can grow inward.
      if (t.overlay == Overlay::Industrial) {
        seeds.push_back(Seed{p.y * w + p.x, Overlay::Industrial, t.level, static_cast<std::uint8_t>(roadLevel)});
        continue;
      }

      if (t.overlay != Overlay::None) {
        continue;
      }

      // Deterministic per-tile chance gate.
      const float gate = TileRand01(p.x, p.y, seed32 ^ 0xDEADBEEFu);
      if (gate >= cfg.zoneChance) {
        continue;
      }

      const bool nearWater = HasAdjacentWater4(world, p.x, p.y);
      const int hubDist = NearestHubDist(hubs, p.x, p.y);
      const float d01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(w + h), 0.0f, 1.0f);

      const int district = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
      const DistrictZoningProfile& prof = dz.profile[static_cast<std::size_t>(district)];

      float landV = 0.5f;
      float waterA = nearWater ? 1.0f : 0.0f;
      float parkA = 0.0f;
      float pol = 0.0f;

      if (lvOk) {
        const std::size_t idx = Idx(p.x, p.y, w);
        landV = lv.value[idx];
        if (idx < lv.waterAmenity.size()) waterA = lv.waterAmenity[idx];
        if (idx < lv.parkAmenity.size()) parkA = lv.parkAmenity[idx];
        if (idx < lv.pollution.size()) pol = lv.pollution[idx];
      }

      const float rType = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE01u);
      Overlay zone = PickZoneTypeLandValue(prof, d01, roadLevel, nearWater, landV, waterA, parkA, pol, rType, cfg);

      if (zone == Overlay::Park) {
        // Parks don't grow inward like zones; treat them as blockers.
        if (cfg.parkChance > 0.0f) {
          t.overlay = Overlay::Park;
          t.level = 1;
          t.occupants = 0;
        }
        continue;
      }

      const float r0 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE02u);
      const float r1 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE03u);
      const float r2 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE04u);
      const float r3 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE05u);

      const int zLvl = PickZoneLevelLandValue(zone, d01, roadLevel, landV, pol, r0, r1, r2, r3);

      t.overlay = zone;
      t.level = static_cast<std::uint8_t>(zLvl);
      t.occupants = 1;

      seeds.push_back(Seed{p.y * w + p.x, zone, static_cast<std::uint8_t>(zLvl), static_cast<std::uint8_t>(roadLevel)});
    }

    if (seeds.empty()) {
      continue;
    }

    // --- BFS ownership + distance (restricted to tiles that are still empty in THIS block).
    q.clear();
    order.clear();
    touched.clear();

    for (int i = 0; i < static_cast<int>(seeds.size()); ++i) {
      const int sIdx = seeds[static_cast<std::size_t>(i)].idx;
      if (sIdx < 0 || sIdx >= w * h) continue;

      dist[static_cast<std::size_t>(sIdx)] = 0;
      owner[static_cast<std::size_t>(sIdx)] = i;
      q.push_back(sIdx);
      order.push_back(sIdx);
      touched.push_back(sIdx);
    }

    std::size_t head = 0;
    while (head < q.size()) {
      const int cur = q[head++];
      const int cx = cur % w;
      const int cy = cur / w;

      const int curD = dist[static_cast<std::size_t>(cur)];
      const int curOwner = owner[static_cast<std::size_t>(cur)];

      for (int k = 0; k < 4; ++k) {
        const int x2 = cx + nx[k];
        const int y2 = cy + ny[k];
        if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;

        const int nIdx = y2 * w + x2;
        if (cb.tileToBlock[static_cast<std::size_t>(nIdx)] != bid) continue;
        if (dist[static_cast<std::size_t>(nIdx)] != -1) continue;

        const Tile& nt = world.at(x2, y2);
        if (nt.overlay != Overlay::None) {
          // Parks (and seeded zones) are treated as blockers for inward growth.
          continue;
        }

        dist[static_cast<std::size_t>(nIdx)] = curD + 1;
        owner[static_cast<std::size_t>(nIdx)] = curOwner;
        q.push_back(nIdx);
        order.push_back(nIdx);
        touched.push_back(nIdx);
      }
    }

    // --- Growth pass: probabilistically zone tiles inward while maintaining connectivity.
    for (int idx : order) {
      const int d = dist[static_cast<std::size_t>(idx)];
      if (d <= 0) continue; // skip seeds

      const int o = owner[static_cast<std::size_t>(idx)];
      if (o < 0 || o >= static_cast<int>(seeds.size())) continue;

      const Seed& s = seeds[static_cast<std::size_t>(o)];
      const int x = idx % w;
      const int y = idx / w;

      // Probability: decays with distance into the block.
      float p = cfg.zoneChance * 1.85f;

      // Larger roads tend to encourage deeper (denser) build-out behind them.
      if (s.roadLevel >= 2) p *= 1.08f;
      if (s.roadLevel >= 3) p *= 1.10f;

      // Zone-type bias.
      switch (s.zone) {
      case Overlay::Residential: p *= 1.15f; break;
      case Overlay::Commercial: p *= 1.00f; break;
      case Overlay::Industrial: p *= 0.90f; break;
      default: break;
      }

      // Land-value shaping: densify attractive neighborhoods and keep dirty uses in cheap land.
      if (lvOk) {
        const std::size_t lvIdx = static_cast<std::size_t>(idx);
        const float lv01 = (lvIdx < lv.value.size()) ? Clamp01(lv.value[lvIdx]) : 0.5f;
        const float pol = (lvIdx < lv.pollution.size()) ? Clamp01(lv.pollution[lvIdx]) : 0.0f;

        if (s.zone == Overlay::Residential) {
          p *= (1.05f + 0.85f * lv01) * (1.0f - 0.65f * pol);
        } else if (s.zone == Overlay::Commercial) {
          p *= (1.00f + 0.90f * lv01) * (1.0f - 0.50f * pol);
        } else if (s.zone == Overlay::Industrial) {
          p *= (1.10f + 0.75f * (1.0f - lv01)) * (0.90f + 0.35f * pol);
        }
      }

      p *= std::exp(-(static_cast<float>(d) - 1.0f) / 2.8f);
      p = std::clamp(p, 0.0f, 0.92f);

      const float r = TileRand01(x, y, seed32 ^ 0xFACEB00Cu ^ (static_cast<std::uint32_t>(bid) * 0x9E3779B9u));
      if (r >= p) {
        continue;
      }

      // Connectivity check: the new tile must touch an already-zoned tile of the same type.
      bool hasSameNeighbor = false;
      for (int k = 0; k < 4; ++k) {
        const int x2 = x + nx[k];
        const int y2 = y + ny[k];
        if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;
        if (world.at(x2, y2).overlay == s.zone) {
          hasSameNeighbor = true;
          break;
        }
      }
      if (!hasSameNeighbor) {
        continue;
      }

      Tile& t = world.at(x, y);
      if (t.overlay != Overlay::None) {
        continue;
      }

      // Mild density decay deeper into blocks.
      int lvl = static_cast<int>(s.level);
      if (d >= 6) lvl = std::max(1, lvl - 1);
      if (d >= 12) lvl = std::max(1, lvl - 1);

      t.overlay = s.zone;
      t.level = static_cast<std::uint8_t>(std::clamp(lvl, 1, 3));
      t.occupants = 1;
    }

    // Reset scratch arrays for the next block.
    for (int idx : touched) {
      dist[static_cast<std::size_t>(idx)] = -1;
      owner[static_cast<std::size_t>(idx)] = -1;
    }
  }

  // -------------------------------------------------------------------------
  // Optional soft buffer: occasionally turn a road-adjacent residential/commercial
  // tile bordering industry into a park (small "green belt" feel).
  // -------------------------------------------------------------------------
  if (cfg.parkChance > 0.0f) {
    const float pBuffer = std::clamp(cfg.parkChance * 2.2f, 0.0f, 0.22f);
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Industrial) continue;

        for (int k = 0; k < 4; ++k) {
          const int nx2 = x + nx[k];
          const int ny2 = y + ny[k];
          if (!world.inBounds(nx2, ny2)) continue;

          Tile& nt = world.at(nx2, ny2);
          if (nt.overlay != Overlay::Residential && nt.overlay != Overlay::Commercial) continue;
          if (!world.hasAdjacentRoad(nx2, ny2)) continue;

          const float r = TileRand01(nx2, ny2, seed32 ^ 0xB0FF12A3u);
          if (r < pBuffer) {
            nt.overlay = Overlay::Park;
            nt.level = 1;
            nt.occupants = 0;
          }
        }
      }
    }
  }
}


static P FindClosestEdgeLand(const World& world, P from)
{
  P best{from.x, from.y};
  int bestDist = std::numeric_limits<int>::max();
  bool found = false;

  const auto consider = [&](int x, int y) {
    if (!world.inBounds(x, y)) {
      return;
    }
    if (!world.isBuildable(x, y)) {
      return;
    }
    const int d = std::abs(from.x - x) + std::abs(from.y - y);
    if (d < bestDist) {
      bestDist = d;
      best = {x, y};
      found = true;
    }
  };

  // Top / bottom.
  for (int x = 0; x < world.width(); ++x) {
    consider(x, 0);
    consider(x, world.height() - 1);
  }
  // Left / right.
  for (int y = 0; y < world.height(); ++y) {
    consider(0, y);
    consider(world.width() - 1, y);
  }

  if (found) {
    return best;
  }

  // Fallback (should be very rare).
  const int dxEdge = std::min(from.x, world.width() - 1 - from.x);
  const int dyEdge = std::min(from.y, world.height() - 1 - from.y);
  if (dxEdge < dyEdge) {
    best.x = (from.x < world.width() / 2) ? 0 : world.width() - 1;
    best.y = from.y;
  } else {
    best.x = from.x;
    best.y = (from.y < world.height() / 2) ? 0 : world.height() - 1;
  }
  return best;
}

static void AssignDistricts(World& world, const std::vector<P>& hubs, RNG& rng, std::uint32_t seed32)
{
  // Pick district sites: use hubs as anchors, then fill remaining with well-spaced random land points.
  std::vector<P> sites;
  sites.reserve(kDistrictCount);

  for (const P& h : hubs) {
    if (static_cast<int>(sites.size()) >= kDistrictCount) {
      break;
    }
    sites.push_back(h);
  }

  auto farEnough = [&](P p) {
    for (const P& s : sites) {
      const int d = std::abs(p.x - s.x) + std::abs(p.y - s.y);
      if (d < std::min(world.width(), world.height()) / 5) {
        return false;
      }
    }
    return true;
  };

  int tries = 0;
  while (static_cast<int>(sites.size()) < kDistrictCount && tries < 4000) {
    ++tries;
    const P p = RandomLand(world, rng);
    if (farEnough(p) || tries > 2500) {
      sites.push_back(p);
    }
  }

  if (sites.empty()) {
    sites.push_back({world.width() / 2, world.height() / 2});
  }

  // Voronoi assignment with a tiny deterministic jitter for less grid-like borders.
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      int bestId = 0;
      std::uint32_t bestScore = std::numeric_limits<std::uint32_t>::max();

      for (int i = 0; i < static_cast<int>(sites.size()); ++i) {
        const int dx = x - sites[i].x;
        const int dy = y - sites[i].y;
        const std::uint32_t dist2 = static_cast<std::uint32_t>(dx * dx + dy * dy);

        const std::uint32_t jitter = HashCoords32(x, y, seed32 ^ (0x9E3779B9u * static_cast<std::uint32_t>(i + 1))) &
                                     0xFFu;
        const std::uint32_t score = (dist2 << 8) + jitter;

        if (score < bestScore) {
          bestScore = score;
          bestId = i;
        }
      }

      world.at(x, y).district = static_cast<std::uint8_t>(std::clamp(bestId, 0, kDistrictCount - 1));
    }
  }
}

} // namespace

World GenerateWorld(int width, int height, std::uint64_t seed, const ProcGenConfig& cfg)
{
  // Mix the full 64-bit seed into 32-bit for noise/RNG.
  const std::uint32_t seed32 = static_cast<std::uint32_t>(seed) ^ static_cast<std::uint32_t>(seed >> 32);

  World world(width, height, seed);
  RNG rng(seed32);

  // Generate initial terrain heights (noise).
  std::vector<float> heights;
  heights.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const float fx = static_cast<float>(x);
      const float fy = static_cast<float>(y);

      // Domain warp (tile-space offsets) to break up axis-aligned artifacts.
      const float warpScale = cfg.terrainScale * 0.35f;
      const float warpAmp = std::clamp(static_cast<float>(std::min(width, height)) * 0.045f, 2.0f, 9.0f);

      const float wx = fx + (fbmNormalized(fx * warpScale, fy * warpScale, seed32 ^ 0x1234ABCDu, 3) * 2.0f - 1.0f) * warpAmp;
      const float wy = fy + (fbmNormalized(fx * warpScale, fy * warpScale, seed32 ^ 0x5678DCBAu, 3) * 2.0f - 1.0f) * warpAmp;

      // Mix of macro landmass noise, detail noise, and a ridged component for mountains.
      const float macro = fbmNormalized(wx * cfg.terrainScale * 0.55f, wy * cfg.terrainScale * 0.55f, seed32 ^ 0xBA5EBA11u, 5);
      const float detail = fbmNormalized(wx * cfg.terrainScale * 2.15f, wy * cfg.terrainScale * 2.15f, seed32 ^ 0xC001D00Du, 3);
      const float ridges = ridgedFbmNormalized(wx * cfg.terrainScale * 0.95f, wy * cfg.terrainScale * 0.95f, seed32 ^ 0xD00DFEEDu, 4);

      float n01 = 0.68f * macro + 0.22f * detail + 0.10f * ridges;
      n01 = std::clamp(n01, 0.0f, 1.0f);

      // Keep the height range stable so existing ProcGenConfig values still make sense.
      heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)] = n01 * 1.2f - 0.2f;
    }
  }

  // Optional post-pass: erosion + rivers + smoothing.
  ApplyErosion(heights, width, height, cfg.erosion, seed);

  // Commit heights into tiles and classify terrain.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Tile& t = world.at(x, y);
      t.height = heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)];

      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
      // district assigned later.

      // Stable variation for renderer/detailing.
      t.variation = static_cast<std::uint8_t>(HashCoords32(x, y, seed32 ^ 0xA3C59AC3u) & 0xFFu);

      if (t.height < cfg.waterLevel) {
        t.terrain = Terrain::Water;
      } else if (t.height < cfg.sandLevel) {
        t.terrain = Terrain::Sand;
      } else {
        t.terrain = Terrain::Grass;
      }
    }
  }

  // Convert hydrology river channels into actual water tiles.
  //
  // The erosion module can carve "river-like" channels into the heightfield, but the classic
  // terrain classification (height < waterLevel) often leaves these channels as Grass/Sand.
  //
  // By converting high-accumulation flow cells into Terrain::Water, ProcGen produces visible
  // rivers that:
  //  - increase water amenity in the land value model
  //  - create natural constraints for road/zoning layout
  //  - allow the bridge system to shine on higher-class connectors
  if (cfg.erosion.enabled && cfg.erosion.riversEnabled) {
    HydrologyField field = BuildHydrologyField(heights, width, height);

    int minAccum = cfg.erosion.riverMinAccum;
    if (minAccum <= 0) minAccum = AutoRiverMinAccum(width, height);
    minAccum = std::max(2, minAccum);

    const std::vector<std::uint8_t> riverMask = BuildRiverMask(field.accum, width, height, minAccum);
    if (!riverMask.empty() && riverMask.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
      // Widen rivers based on accumulation so major rivers read as more than a 1-tile line.
      std::vector<std::uint8_t> riverWater;
      riverWater.assign(riverMask.size(), 0);

      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
          if (riverMask[i] == 0) continue;

          const int accum = (i < field.accum.size()) ? field.accum[i] : 0;

          int rad = 0;
          if (accum >= minAccum * 12) {
            rad = 2;
          } else if (accum >= minAccum * 4) {
            rad = 1;
          }

          for (int dy = -rad; dy <= rad; ++dy) {
            for (int dx = -rad; dx <= rad; ++dx) {
              if (std::abs(dx) + std::abs(dy) > rad) continue;
              const int nx = x + dx;
              const int ny = y + dy;
              if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
              riverWater[static_cast<std::size_t>(ny) * static_cast<std::size_t>(width) + static_cast<std::size_t>(nx)] = 1;
            }
          }
        }
      }

      // Apply water mask.
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
          if (riverWater[i] == 0) continue;
          world.at(x, y).terrain = Terrain::Water;
        }
      }

      // River banks: gently sandify immediate neighbors of river water. This makes rivers
      // legible even when waterLevel is low and avoids "all grass" channels.
      const float bankMaxH = std::max(cfg.sandLevel, cfg.waterLevel + 0.02f) + 0.10f;
      const int dx4[4] = {1, -1, 0, 0};
      const int dy4[4] = {0, 0, 1, -1};

      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
          if (riverWater[i] == 0) continue;

          for (int k = 0; k < 4; ++k) {
            const int nx = x + dx4[k];
            const int ny = y + dy4[k];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            Tile& t = world.at(nx, ny);
            if (t.terrain == Terrain::Water) continue;
            if (t.terrain == Terrain::Grass && t.height < bankMaxH) {
              t.terrain = Terrain::Sand;
            }
          }
        }
      }
    }
  }

  // Add a small number of inland lakes (basin flooding) to break up large landmasses
  // and create additional water constraints/amenity for road + zoning generation.
  AddProceduralLakes(world, heights, cfg.waterLevel, cfg.sandLevel, seed32);


  // Pick hubs (town centers).
  //
  // Instead of pure uniform sampling, we score candidates so hubs tend to land on
  // flatter, more buildable areas (and optionally near some water) which produces
  // better road layouts + more coherent zoning.
  std::vector<P> hubPts;
  hubPts.reserve(std::max(1, cfg.hubs));

  const bool hasAnyWater = WorldHasAnyWater(world);
  const int minDist = std::clamp(std::min(width, height) / 4, 10, 20);

  for (int i = 0; i < cfg.hubs; ++i) {
    float bestScore = -1.0f;
    P best{width / 2, height / 2};
    const std::uint32_t scoreSeed = seed32 ^ 0xBADC0FFEu ^ (static_cast<std::uint32_t>(i + 1) * 0x9E3779B9u);

    // Best-of-N sampling: deterministic, and cheap at typical map sizes.
    for (int tries = 0; tries < 800; ++tries) {
      const int x = rng.rangeInt(0, width - 1);
      const int y = rng.rangeInt(0, height - 1);
      if (!world.isBuildable(x, y)) continue;

      bool farEnough = true;
      for (const P& h : hubPts) {
        if (std::abs(h.x - x) + std::abs(h.y - y) < minDist) {
          farEnough = false;
          break;
        }
      }
      if (!farEnough) continue;

      const float s = ScoreHubCandidate(world, x, y, hasAnyWater, scoreSeed);
      if (s > bestScore) {
        bestScore = s;
        best = {x, y};
      }
    }

    if (bestScore < 0.0f) {
      // Fallback: random land. We still try to avoid exact duplicates.
      best = RandomLand(world, rng);
      for (int tries = 0; tries < 1500; ++tries) {
        const P p = RandomLand(world, rng);
        bool ok = true;
        for (const P& h : hubPts) {
          if (p.x == h.x && p.y == h.y) {
            ok = false;
            break;
          }
        }
        if (ok) {
          best = p;
          break;
        }
      }
    }

    hubPts.push_back(best);
  }

  // Deduplicate hubs (rare on tiny maps).
  {
    std::vector<P> unique;
    unique.reserve(hubPts.size());
    for (const P& p : hubPts) {
      bool dup = false;
      for (const P& q : unique) {
        if (p.x == q.x && p.y == q.y) {
          dup = true;
          break;
        }
      }
      if (!dup) unique.push_back(p);
    }
    hubPts.swap(unique);
  }

  if (hubPts.empty()) {
    hubPts.push_back({width / 2, height / 2});
  }

  // Districts: create meaningful administrative regions from the start.
  AssignDistricts(world, hubPts, rng, seed32 ^ 0xC001D00Du);

  // Carve local street grids around hubs.
  for (const P& h : hubPts) {
    CarveHubGrid(world, rng, h);
  }

  // Connect hubs with a minimum-spanning-tree (MST) backbone instead of an arbitrary
  // sequential chain. This produces more natural arterial networks and guarantees
  // every hub is reachable.
  const std::vector<Edge> mst = BuildHubMST(hubPts);
  std::vector<std::uint32_t> usedEdgeKeys;
  usedEdgeKeys.reserve(mst.size() + static_cast<std::size_t>(cfg.extraConnections) + 4u);

  for (const Edge& e : mst) {
    usedEdgeKeys.push_back(EdgeKeyU32(e.a, e.b));
    const P a = hubPts[static_cast<std::size_t>(e.a)];
    const P b = hubPts[static_cast<std::size_t>(e.b)];
    const int lvl = ChooseHubConnectionLevel(world, a, b);
    CarveRoad(world, rng, a, b, lvl, /*allowBridges=*/(lvl >= 2));
  }

  auto edgeUsed = [&](std::uint32_t key) {
    return std::find(usedEdgeKeys.begin(), usedEdgeKeys.end(), key) != usedEdgeKeys.end();
  };

  // Extra connections: add short hub-to-hub loops via k-nearest-neighbor candidate edges.
  if (cfg.extraConnections > 0 && hubPts.size() >= 3) {
    const int n = static_cast<int>(hubPts.size());
    const int kNeighbors = std::min(3, n - 1);

    std::vector<Edge> candidates;
    candidates.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(kNeighbors));

    for (int i = 0; i < n; ++i) {
      std::vector<std::pair<int, int>> distTo;
      distTo.reserve(static_cast<std::size_t>(n) - 1u);
      for (int j = 0; j < n; ++j) {
        if (i == j) continue;
        distTo.push_back({ManhattanDist(hubPts[static_cast<std::size_t>(i)], hubPts[static_cast<std::size_t>(j)]), j});
      }
      std::sort(distTo.begin(), distTo.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        return a.second < b.second;
      });

      for (int k = 0; k < kNeighbors && k < static_cast<int>(distTo.size()); ++k) {
        const int j = distTo[static_cast<std::size_t>(k)].second;
        const int dist = distTo[static_cast<std::size_t>(k)].first;
        const int a = std::min(i, j);
        const int b = std::max(i, j);
        const std::uint32_t key = EdgeKeyU32(a, b);
        if (edgeUsed(key)) continue;

        // Also avoid duplicate candidate edges.
        bool dup = false;
        for (const Edge& e : candidates) {
          if (EdgeKeyU32(e.a, e.b) == key) {
            dup = true;
            break;
          }
        }
        if (dup) continue;

        candidates.push_back(Edge{a, b, dist});
      }
    }

    // Sort by distance, with a deterministic pseudo-random tiebreaker so different seeds
    // get different loop choices.
    std::sort(candidates.begin(), candidates.end(), [&](const Edge& ea, const Edge& eb) {
      if (ea.dist != eb.dist) return ea.dist < eb.dist;
      const std::uint32_t ha = HashCoords32(ea.a, ea.b, seed32 ^ 0xF00DFACEu);
      const std::uint32_t hb = HashCoords32(eb.a, eb.b, seed32 ^ 0xF00DFACEu);
      return ha < hb;
    });

    int added = 0;
    for (const Edge& e : candidates) {
      if (added >= cfg.extraConnections) break;
      const std::uint32_t key = EdgeKeyU32(e.a, e.b);
      if (edgeUsed(key)) continue;

      const P a = hubPts[static_cast<std::size_t>(e.a)];
      const P b = hubPts[static_cast<std::size_t>(e.b)];

      // Loops should be at least avenue-class so they meaningfully take load off the backbone.
      const int lvl = std::max(2, ChooseHubConnectionLevel(world, a, b));
      CarveRoad(world, rng, a, b, lvl, /*allowBridges=*/true);

      usedEdgeKeys.push_back(key);
      ++added;
    }
  }

  // Ensure at least one connection to the map edge (outside connection).
  // We pick the hub that is *closest* to an edge buildable tile so the outside link
  // feels like a "highway in" rather than an arbitrary diagonal cut.
  if (!hubPts.empty()) {
    int bestHub = 0;
    P bestEdge = FindClosestEdgeLand(world, hubPts[0]);
    int bestDist = ManhattanDist(hubPts[0], bestEdge);

    for (int i = 1; i < static_cast<int>(hubPts.size()); ++i) {
      const P edge = FindClosestEdgeLand(world, hubPts[static_cast<std::size_t>(i)]);
      const int d = ManhattanDist(hubPts[static_cast<std::size_t>(i)], edge);
      if (d < bestDist) {
        bestDist = d;
        bestHub = i;
        bestEdge = edge;
      }
    }

    const int lvl = std::max(2, ChooseHubConnectionLevel(world, hubPts[static_cast<std::size_t>(bestHub)], bestEdge));
    CarveRoad(world, rng, hubPts[static_cast<std::size_t>(bestHub)], bestEdge, lvl, /*allowBridges=*/true);
  }

  // Optional: carve a highway-ish beltway around the hub cluster.
  CarveBeltwayIfUseful(world, rng, hubPts, seed32 ^ 0xB17BEEFu);

  // Subdivide large blocks with a small hierarchical street network before zoning.
  CarveInternalStreets(world, hubPts, seed32 ^ 0x1337C0DEu);
  // Place zones and parks using block-aware inward growth.
  PlaceZonesAndParksFromBlocks(world, hubPts, seed32 ^ 0xD15EA5E5u, cfg);

  // Safety: rebuild masks in one pass.
  world.recomputeRoadMasks();

  return world;
}

} // namespace isocity
