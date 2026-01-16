#include "isocity/ProcGen.hpp"

#include "isocity/CityBlocks.hpp"

#include "isocity/BlockDistricting.hpp"
#include "isocity/Districting.hpp"
#include "isocity/Noise.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphRouting.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Hydrology.hpp"
#include "isocity/Road.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace isocity {

const char* ToString(ProcGenTerrainPreset p)
{
  switch (p) {
  case ProcGenTerrainPreset::Classic: return "classic";
  case ProcGenTerrainPreset::Island: return "island";
  case ProcGenTerrainPreset::Archipelago: return "archipelago";
  case ProcGenTerrainPreset::InlandSea: return "inland_sea";
  case ProcGenTerrainPreset::RiverValley: return "river_valley";
  case ProcGenTerrainPreset::MountainRing: return "mountain_ring";
  case ProcGenTerrainPreset::Fjords: return "fjords";
  case ProcGenTerrainPreset::Canyon: return "canyon";
  case ProcGenTerrainPreset::Volcano: return "volcano";
  case ProcGenTerrainPreset::Delta: return "delta";
  default: return "classic";
  }
}

static std::string LowerCopy(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool ParseProcGenTerrainPreset(const std::string& s, ProcGenTerrainPreset& out)
{
  const std::string t = LowerCopy(s);
  if (t.empty()) return false;

  auto eq = [&](const char* a) { return t == a; };

  if (eq("classic") || eq("default") || eq("continent") || eq("continental")) {
    out = ProcGenTerrainPreset::Classic;
    return true;
  }
  if (eq("island") || eq("islands")) {
    out = ProcGenTerrainPreset::Island;
    return true;
  }
  if (eq("archipelago") || eq("arch") || eq("isle") || eq("isles")) {
    out = ProcGenTerrainPreset::Archipelago;
    return true;
  }
  if (eq("inlandsea") || eq("inland_sea") || eq("sea") || eq("lake") || eq("inlandse") || eq("inland")) {
    out = ProcGenTerrainPreset::InlandSea;
    return true;
  }
  if (eq("river") || eq("rivervalley") || eq("river_valley") || eq("valley") || eq("river-valley")) {
    out = ProcGenTerrainPreset::RiverValley;
    return true;
  }
  if (eq("mountain") || eq("mountains") || eq("ring") || eq("mountainring") || eq("mountain_ring") ||
      eq("crater")) {
    out = ProcGenTerrainPreset::MountainRing;
    return true;
  }

  if (eq("fjord") || eq("fjords") || eq("glacier") || eq("glacial") || eq("inlet") || eq("inlets")) {
    out = ProcGenTerrainPreset::Fjords;
    return true;
  }

  if (eq("canyon") || eq("canyons") || eq("gorge") || eq("gorges") || eq("grandcanyon") ||
      eq("grand_canyon") || eq("grand-canyon")) {
    out = ProcGenTerrainPreset::Canyon;
    return true;
  }

  if (eq("volcano") || eq("volcanic") || eq("caldera") || eq("crater_lake") || eq("craterlake") ||
      eq("crater-lake")) {
    out = ProcGenTerrainPreset::Volcano;
    return true;
  }

  if (eq("delta") || eq("riverdelta") || eq("river_delta") || eq("river-delta") || eq("floodplain") ||
      eq("wetlands") || eq("marsh") || eq("marshes")) {
    out = ProcGenTerrainPreset::Delta;
    return true;
  }

  return false;
}

const char* ToString(ProcGenDistrictingMode m)
{
  switch (m) {
  case ProcGenDistrictingMode::Voronoi: return "voronoi";
  case ProcGenDistrictingMode::RoadFlow: return "road_flow";
  case ProcGenDistrictingMode::BlockGraph: return "block_graph";
  default: return "voronoi";
  }
}

bool ParseProcGenDistrictingMode(const std::string& s, ProcGenDistrictingMode& out)
{
  const std::string t = LowerCopy(s);
  if (t.empty()) return false;

  auto eq = [&](const char* a) { return t == a; };

  if (eq("voronoi") || eq("legacy") || eq("tile") || eq("tiles") || eq("tile_voronoi") || eq("tile-voronoi")) {
    out = ProcGenDistrictingMode::Voronoi;
    return true;
  }

  if (eq("road") || eq("roads") || eq("roadflow") || eq("road_flow") || eq("road-flow") || eq("flow") ||
      eq("auto") || eq("travel") || eq("traveltime") || eq("travel_time") || eq("travel-time")) {
    out = ProcGenDistrictingMode::RoadFlow;
    return true;
  }

  if (eq("block") || eq("blocks") || eq("blockgraph") || eq("block_graph") || eq("block-graph") ||
      eq("neighborhood") || eq("neighbourhood") || eq("neighborhoods") || eq("neighbourhoods")) {
    out = ProcGenDistrictingMode::BlockGraph;
    return true;
  }

  return false;
}

const char* ToString(ProcGenRoadLayout m)
{
  switch (m) {
  case ProcGenRoadLayout::Organic: return "organic";
  case ProcGenRoadLayout::Grid: return "grid";
  case ProcGenRoadLayout::Radial: return "radial";
  case ProcGenRoadLayout::SpaceColonization: return "space_colonization";
  default: return "organic";
  }
}

bool ParseProcGenRoadLayout(const std::string& s, ProcGenRoadLayout& out)
{
  const std::string t = LowerCopy(s);
  if (t.empty()) return false;

  auto eq = [&](const char* a) { return t == a; };

  if (eq("organic") || eq("org") || eq("classic") || eq("legacy") || eq("mst") || eq("default")) {
    out = ProcGenRoadLayout::Organic;
    return true;
  }

  if (eq("grid") || eq("manhattan") || eq("orthogonal") || eq("rect") || eq("rectilinear")) {
    out = ProcGenRoadLayout::Grid;
    return true;
  }

  if (eq("radial") || eq("ring") || eq("spoke") || eq("spokes") || eq("hubspoke") || eq("hub_spoke") ||
      eq("hub-and-spoke") || eq("hub_and_spoke")) {
    out = ProcGenRoadLayout::Radial;
    return true;
  }


  if (eq("space_colonization") || eq("space-colonization") || eq("spacecolonization") || eq("space") ||
      eq("colonization") || eq("colonisation") || eq("sca") || eq("sc")) {
    out = ProcGenRoadLayout::SpaceColonization;
    return true;
  }

  return false;
}

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

  const int nextLevel = ClampRoadLevel(level);
  int curLevel = 0;
  if (t.overlay == Overlay::Road) {
    curLevel = ClampRoadLevel(static_cast<int>(t.level));
  }

  world.setRoad(x, y);
  Tile& rt = world.at(x, y);
  rt.level = static_cast<std::uint8_t>(std::max(curLevel, nextLevel));
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



// -----------------------------
// Curvy arterial connectors
// -----------------------------
//
// Many city generators carve hub-to-hub arterials as the single cheapest path.
// That often produces overly-straight, "utility-corridor" highways.
//
// To make networks read as more *planned* (and more varied across seeds), we
// optionally route long, high-class connections through a deterministic waypoint
// offset perpendicular to the chord. The underlying A*/Dijkstra cost model still
// handles slopes/water, but the waypoint injects a gentle macro-curve.

static bool FindNearestWaypointTile(const World& world, int cx, int cy, int maxR, bool allowWater,
                                   std::uint32_t seed32, P& out)
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

      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water && !allowWater) continue;

      // Don't route through zones/parks; we only want empty land or existing roads.
      if (t.overlay != Overlay::None && t.overlay != Overlay::Road) continue;

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

static void CarveRoadCurvy(World& world, RNG& rng, P a, P b, int level, bool allowBridges, std::uint32_t seed32)
{
  const int dist = std::abs(a.x - b.x) + std::abs(a.y - b.y);

  // Short/low-class connections look better as direct cost-optimal paths.
  if (dist < 22 || level <= 1) {
    CarveRoad(world, rng, a, b, level, allowBridges);
    return;
  }

  // Compute a perpendicular waypoint near the midpoint.
  const int mx = (a.x + b.x) / 2;
  const int my = (a.y + b.y) / 2;
  const int dx = b.x - a.x;
  const int dy = b.y - a.y;

  // Deterministic side choice.
  const std::uint32_t h = HashCoords32(mx, my, seed32 ^ 0xA11CEB0Bu);
  const int sign = (h & 1u) ? 1 : -1;

  int px = 0;
  int py = 0;
  if (std::abs(dx) >= std::abs(dy)) {
    // Mostly horizontal chord -> offset vertically.
    py = sign;
  } else {
    // Mostly vertical chord -> offset horizontally.
    px = sign;
  }

  const float r = TileRand01(mx, my, seed32 ^ 0xC0FFEEu);
  int offset = static_cast<int>(std::round(static_cast<float>(dist) * (0.12f + 0.22f * r)));
  offset = std::clamp(offset, 6, std::max(6, dist / 2));

  int wx = mx + px * offset;
  int wy = my + py * offset;
  wx = std::clamp(wx, 1, std::max(1, world.width() - 2));
  wy = std::clamp(wy, 1, std::max(1, world.height() - 2));

  P wp{wx, wy};
  if (!FindNearestWaypointTile(world, wx, wy, /*maxR=*/10, /*allowWater=*/allowBridges,
                              seed32 ^ 0xBADC0DEu, wp)) {
    CarveRoad(world, rng, a, b, level, allowBridges);
    return;
  }

  // Avoid degenerate waypoints too close to endpoints.
  const int dA = std::abs(a.x - wp.x) + std::abs(a.y - wp.y);
  const int dB = std::abs(b.x - wp.x) + std::abs(b.y - wp.y);
  if (dA < 10 || dB < 10) {
    CarveRoad(world, rng, a, b, level, allowBridges);
    return;
  }

  // Carve via waypoint. If the first segment can't reach the waypoint (rare but possible
  // on extreme terrain/water layouts), fall back to a direct carve so connectivity isn't lost.
  const Overlay before = world.inBounds(wp.x, wp.y) ? world.at(wp.x, wp.y).overlay : Overlay::None;
  CarveRoad(world, rng, a, wp, level, allowBridges);
  const bool seg1Ok = world.inBounds(wp.x, wp.y) && (world.at(wp.x, wp.y).overlay == Overlay::Road || before == Overlay::Road);
  CarveRoad(world, rng, wp, b, level, allowBridges);

  if (!seg1Ok) {
    CarveRoad(world, rng, a, b, level, allowBridges);
  }
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
// Macro road layout modes
// -----------------------------

static void CarveHubConnectionsOrganic(World& world, RNG& rng, const std::vector<P>& hubPts, std::uint32_t seed32,
                                      const ProcGenConfig& cfg)
{
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
    CarveRoadCurvy(world, rng, a, b, lvl, /*allowBridges=*/(lvl >= 2), seed32 ^ HashCoords32(e.a, e.b, 0xC0FFEEu));
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
      CarveRoadCurvy(world, rng, a, b, lvl, /*allowBridges=*/true, seed32 ^ HashCoords32(e.a, e.b, 0xBADC0DEu));

      usedEdgeKeys.push_back(key);
      ++added;
    }
  }
}

static void CarveHubConnectionsGrid(World& world, RNG& rng, const std::vector<P>& hubPts, std::uint32_t seed32,
                                   const ProcGenConfig& cfg)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;
  if (hubPts.empty()) return;

  // Compute hub centroid (integer).
  int cx = 0;
  int cy = 0;
  for (const P& p : hubPts) {
    cx += p.x;
    cy += p.y;
  }
  cx = static_cast<int>(std::lround(static_cast<float>(cx) / static_cast<float>(hubPts.size())));
  cy = static_cast<int>(std::lround(static_cast<float>(cy) / static_cast<float>(hubPts.size())));
  cx = std::clamp(cx, 0, w - 1);
  cy = std::clamp(cy, 0, h - 1);

  const int minDim = std::min(w, h);
  int spacing = std::clamp(minDim / 6, 10, 18);
  if (minDim <= 48) {
    spacing = std::clamp(minDim / 5, 8, 14);
  }

  std::vector<int> xs;
  std::vector<int> ys;
  xs.reserve(16);
  ys.reserve(16);

  auto addUnique = [](std::vector<int>& v, int a) {
    if (std::find(v.begin(), v.end(), a) == v.end()) v.push_back(a);
  };

  addUnique(xs, cx);
  addUnique(ys, cy);

  for (int step = 1; step < 32; ++step) {
    const int x1 = cx + step * spacing;
    const int x2 = cx - step * spacing;
    if (x1 >= w && x2 < 0) break;
    if (x1 >= 0 && x1 < w) addUnique(xs, x1);
    if (x2 >= 0 && x2 < w) addUnique(xs, x2);
  }

  for (int step = 1; step < 32; ++step) {
    const int y1 = cy + step * spacing;
    const int y2 = cy - step * spacing;
    if (y1 >= h && y2 < 0) break;
    if (y1 >= 0 && y1 < h) addUnique(ys, y1);
    if (y2 >= 0 && y2 < h) addUnique(ys, y2);
  }

  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());

  auto isCentral = [&](int v, int c) { return std::abs(v - c) <= std::max(2, spacing / 2); };

  // Carve primary arterials as straight grid lines.
  for (int x : xs) {
    const int lvl = isCentral(x, cx) ? 3 : 2;
    for (int y = 0; y < h; ++y) {
      SetRoadWithLevel(world, x, y, lvl, /*allowBridges=*/true);
    }
  }

  for (int y : ys) {
    const int lvl = isCentral(y, cy) ? 3 : 2;
    for (int x = 0; x < w; ++x) {
      SetRoadWithLevel(world, x, y, lvl, /*allowBridges=*/true);
    }
  }

  // Snap each hub into the arterial grid via the nearest intersection.
  for (std::size_t i = 0; i < hubPts.size(); ++i) {
    const P hub = hubPts[i];

    int bestX = xs.empty() ? hub.x : xs[0];
    int bestY = ys.empty() ? hub.y : ys[0];
    int bestD = std::numeric_limits<int>::max();
    std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();

    for (int x : xs) {
      for (int y : ys) {
        const int d = std::abs(hub.x - x) + std::abs(hub.y - y);
        const std::uint32_t tie = HashCoords32(x, y, seed32 ^ HashCoords32(hub.x, hub.y, 0x4A7F3D21u));
        if (d < bestD || (d == bestD && tie < bestTie)) {
          bestD = d;
          bestTie = tie;
          bestX = x;
          bestY = y;
        }
      }
    }

    P target{bestX, bestY};
    (void)FindNearestWaypointTile(world, bestX, bestY, /*maxR=*/6, /*allowWater=*/false,
                                 seed32 ^ HashCoords32(bestX, bestY, 0x9E3779B9u), target);

    const int lvl = std::max(2, ChooseHubConnectionLevel(world, hub, target));
    CarveRoadCurvy(world, rng, hub, target, lvl, /*allowBridges=*/true, seed32 ^ HashCoords32(static_cast<int>(i), lvl, 0x6A71D00u));
  }

  // Use the extraConnections budget to add a few diagonal express links between distant hubs.
  if (cfg.extraConnections > 0 && hubPts.size() >= 2) {
    const int n = static_cast<int>(hubPts.size());
    std::vector<Edge> pairs;
    pairs.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n - 1) / 2u);

    for (int a = 0; a < n; ++a) {
      for (int b = a + 1; b < n; ++b) {
        pairs.push_back(Edge{a, b, ManhattanDist(hubPts[static_cast<std::size_t>(a)], hubPts[static_cast<std::size_t>(b)])});
      }
    }

    std::sort(pairs.begin(), pairs.end(), [&](const Edge& ea, const Edge& eb) {
      if (ea.dist != eb.dist) return ea.dist > eb.dist; // longest first
      const std::uint32_t ha = HashCoords32(ea.a, ea.b, seed32 ^ 0xD1A60A1Eu);
      const std::uint32_t hb = HashCoords32(eb.a, eb.b, seed32 ^ 0xD1A60A1Eu);
      return ha < hb;
    });

    int added = 0;
    std::vector<std::uint32_t> used;
    used.reserve(static_cast<std::size_t>(cfg.extraConnections) + 4u);

    for (const Edge& e : pairs) {
      if (added >= cfg.extraConnections) break;
      const std::uint32_t key = EdgeKeyU32(e.a, e.b);
      if (std::find(used.begin(), used.end(), key) != used.end()) continue;
      used.push_back(key);

      const P a = hubPts[static_cast<std::size_t>(e.a)];
      const P b = hubPts[static_cast<std::size_t>(e.b)];
      CarveRoadCurvy(world, rng, a, b, /*level=*/3, /*allowBridges=*/true, seed32 ^ HashCoords32(e.a, e.b, 0x51D1A6u));
      ++added;
    }
  }
}

static void CarveHubConnectionsRadial(World& world, RNG& rng, const std::vector<P>& hubPts, std::uint32_t seed32,
                                     const ProcGenConfig& cfg)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;
  if (hubPts.size() < 2) return;

  // Compute hub centroid.
  float cx = 0.0f;
  float cy = 0.0f;
  for (const P& p : hubPts) {
    cx += static_cast<float>(p.x);
    cy += static_cast<float>(p.y);
  }
  cx /= static_cast<float>(hubPts.size());
  cy /= static_cast<float>(hubPts.size());

  // Pick the most "central" hub as the spoke origin.
  int centerIdx = 0;
  int bestD = std::numeric_limits<int>::max();
  std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();
  for (int i = 0; i < static_cast<int>(hubPts.size()); ++i) {
    const P p = hubPts[static_cast<std::size_t>(i)];
    const int d = static_cast<int>(std::lround(std::fabs(static_cast<float>(p.x) - cx) + std::fabs(static_cast<float>(p.y) - cy)));
    const std::uint32_t tie = HashCoords32(p.x, p.y, seed32 ^ 0x13579BDFu);
    if (d < bestD || (d == bestD && tie < bestTie)) {
      bestD = d;
      bestTie = tie;
      centerIdx = i;
    }
  }

  const P center = hubPts[static_cast<std::size_t>(centerIdx)];

  // Spokes from the center to every other hub.
  for (int i = 0; i < static_cast<int>(hubPts.size()); ++i) {
    if (i == centerIdx) continue;
    const P p = hubPts[static_cast<std::size_t>(i)];
    const int lvl = std::max(2, ChooseHubConnectionLevel(world, center, p));
    CarveRoadCurvy(world, rng, center, p, lvl, /*allowBridges=*/true, seed32 ^ HashCoords32(centerIdx, i, 0x5F0CE001u));
  }

  // Outer ring: connect hubs around the centroid by angle.
  struct HubAngle {
    int idx = 0;
    float ang = 0.0f;
    float r = 0.0f;
  };

  std::vector<HubAngle> outer;
  outer.reserve(hubPts.size());
  for (int i = 0; i < static_cast<int>(hubPts.size()); ++i) {
    if (i == centerIdx) continue;
    const P p = hubPts[static_cast<std::size_t>(i)];
    const float dx = static_cast<float>(p.x) - cx;
    const float dy = static_cast<float>(p.y) - cy;
    HubAngle ha;
    ha.idx = i;
    ha.ang = std::atan2(dy, dx);
    ha.r = std::fabs(dx) + std::fabs(dy);
    outer.push_back(ha);
  }

  if (outer.size() >= 3) {
    std::sort(outer.begin(), outer.end(), [&](const HubAngle& a, const HubAngle& b) {
      if (a.ang != b.ang) return a.ang < b.ang;
      if (a.r != b.r) return a.r > b.r;
      const std::uint32_t ha = HashCoords32(a.idx, static_cast<int>(std::lround(a.r)), seed32 ^ 0xBEEFBEEFu);
      const std::uint32_t hb = HashCoords32(b.idx, static_cast<int>(std::lround(b.r)), seed32 ^ 0xBEEFBEEFu);
      return ha < hb;
    });

    const int diag = w + h;

    for (std::size_t i = 0; i < outer.size(); ++i) {
      const int aIdx = outer[i].idx;
      const int bIdx = outer[(i + 1) % outer.size()].idx;

      const P a = hubPts[static_cast<std::size_t>(aIdx)];
      const P b = hubPts[static_cast<std::size_t>(bIdx)];

      int lvl = 2;
      const int dist = ManhattanDist(a, b);
      if (dist > diag / 3) lvl = 3;

      CarveRoadCurvy(world, rng, a, b, lvl, /*allowBridges=*/true, seed32 ^ HashCoords32(aIdx, bIdx, 0xA71E0001u));
    }
  }

  // ExtraConnections budget: add a few "chords" across the ring.
  if (cfg.extraConnections > 0 && outer.size() >= 4) {
    const int n = static_cast<int>(outer.size());
    int added = 0;
    std::vector<std::uint32_t> used;
    used.reserve(static_cast<std::size_t>(cfg.extraConnections) + 4u);

    for (int i = 0; i < n && added < cfg.extraConnections; ++i) {
      const int aIdx = outer[static_cast<std::size_t>(i)].idx;
      const int bIdx = outer[static_cast<std::size_t>((i + n / 2 + added) % n)].idx;
      if (aIdx == bIdx) continue;

      const std::uint32_t key = EdgeKeyU32(aIdx, bIdx);
      if (std::find(used.begin(), used.end(), key) != used.end()) continue;
      used.push_back(key);

      const P a = hubPts[static_cast<std::size_t>(aIdx)];
      const P b = hubPts[static_cast<std::size_t>(bIdx)];
      CarveRoadCurvy(world, rng, a, b, /*level=*/3, /*allowBridges=*/true, seed32 ^ HashCoords32(aIdx, bIdx, 0xC0AD0001u));
      ++added;
    }
  }
}

// Space-colonization-style arterial growth.
//
// This mode uses an "attractor point" growth process (inspired by the space
// colonization algorithm used for biological branching) to grow arterial roads
// outward from the initial hub network.
//
// High-level goals:
//  - produce branching, tree-like arterial spines that explore the map
//  - remain deterministic and terrain-aware (reusing the existing road router)
//  - keep road density bounded (small number of attractors + fixed step)
static void CarveHubConnectionsSpaceColonization(World& world, RNG& rng, const std::vector<P>& hubPts,
                                                std::uint32_t seed32, const ProcGenConfig& cfg)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;
  if (hubPts.empty()) return;

  // First, ensure global hub connectivity with the "organic" backbone.
  // (MST backbone + a few deterministic loops).
  CarveHubConnectionsOrganic(world, rng, hubPts, seed32 ^ 0x5C010C01u, cfg);

  const int minDim = std::min(w, h);

  // --- Parameters (heuristic, but deterministic) ---
  const int influenceR = std::clamp(minDim / 2, 18, 52);
  const int killR = std::clamp(minDim / 32, 3, 6);
  const int step = std::clamp(minDim / 24, 3, 6);

  // Attractor density: keep it sparse so the road network stays readable.
  const int area = w * h;
  const int targetAttractors = std::clamp(area / 160, 40, 240);
  const int minAttractorSep = std::clamp(minDim / 14, 4, 9);

  auto hasRoadInRadius = [&](int x, int y, int r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (!world.inBounds(nx, ny)) continue;
        if (std::abs(dx) + std::abs(dy) > r) continue;
        if (world.at(nx, ny).overlay == Overlay::Road) return true;
      }
    }
    return false;
  };

  // --- Sample attractor points on buildable, currently-empty land away from existing roads ---
  std::vector<P> attractors;
  attractors.reserve(static_cast<std::size_t>(targetAttractors));

  const int avoidRoadR = 2;
  const int maxTries = std::max(4000, targetAttractors * 120);

  for (int tries = 0; tries < maxTries && static_cast<int>(attractors.size()) < targetAttractors; ++tries) {
    const int x = rng.rangeInt(0, w - 1);
    const int y = rng.rangeInt(0, h - 1);

    if (!world.isBuildable(x, y)) continue;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::None) continue;

    // Avoid the map edge so arterials don't just "hug" the boundary.
    const int edgeDist = std::min(std::min(x, w - 1 - x), std::min(y, h - 1 - y));
    if (edgeDist < 2) continue;

    // Avoid sampling directly adjacent to roads so growth explores new areas.
    if (hasRoadInRadius(x, y, avoidRoadR)) continue;

    // Lightweight Poisson-disc-ish spacing.
    bool ok = true;
    for (const P& a : attractors) {
      if (std::abs(a.x - x) + std::abs(a.y - y) < minAttractorSep) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;

    attractors.push_back({x, y});
  }

  if (attractors.empty()) {
    // Nothing to grow toward.
    return;
  }

  // --- Initial growth nodes: hubs + a downsampled set of existing arterials ---
  std::vector<P> nodes;
  nodes.reserve(static_cast<std::size_t>(hubPts.size()) + 256u);

  auto addNodeIfFar = [&](P p, int minSep) {
    if (!world.inBounds(p.x, p.y)) return;
    for (const P& n : nodes) {
      if (std::abs(n.x - p.x) + std::abs(n.y - p.y) <= minSep) return;
    }
    nodes.push_back(p);
  };

  for (const P& h0 : hubPts) {
    addNodeIfFar(h0, /*minSep=*/1);
  }

  // Sample existing avenue/highway tiles as additional branching seeds.
  // This makes growth able to branch from the backbone, not only from hubs.
  const int sampleDiv = std::clamp(minDim / 6, 9, 16);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      if (ClampRoadLevel(static_cast<int>(t.level)) < 2) continue;
      const std::uint32_t hh = HashCoords32(x, y, seed32 ^ 0x5EED5EEDu);
      if ((hh % static_cast<std::uint32_t>(sampleDiv)) != 0u) continue;
      addNodeIfFar({x, y}, /*minSep=*/step);
    }
  }

  if (nodes.empty()) {
    return;
  }

  struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
  };

  auto dist2 = [](P a, P b) {
    const int dx = a.x - b.x;
    const int dy = a.y - b.y;
    return dx * dx + dy * dy;
  };

  const int influenceR2 = influenceR * influenceR;
  const int killR2 = killR * killR;

  // --- Growth loop ---
  const int maxIters = std::clamp(area / 64, 200, 1400);

  for (int iter = 0; iter < maxIters && !attractors.empty(); ++iter) {
    std::vector<Vec2> acc(nodes.size());
    std::vector<int> cnt(nodes.size(), 0);

    bool anyAssigned = false;

    // Assign each attractor to its nearest node (within influence radius).
    for (const P& a : attractors) {
      int bestIdx = -1;
      int bestD2 = std::numeric_limits<int>::max();

      for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        const P n = nodes[static_cast<std::size_t>(i)];
        const int dx = a.x - n.x;
        const int dy = a.y - n.y;
        const int d2 = dx * dx + dy * dy;
        if (d2 > influenceR2) continue;
        if (d2 < bestD2) {
          bestD2 = d2;
          bestIdx = i;
        }
      }

      if (bestIdx >= 0 && bestD2 > 0) {
        anyAssigned = true;
        const P n = nodes[static_cast<std::size_t>(bestIdx)];
        const float fx = static_cast<float>(a.x - n.x);
        const float fy = static_cast<float>(a.y - n.y);
        const float len = std::sqrt(fx * fx + fy * fy);
        if (len > 0.0001f) {
          acc[static_cast<std::size_t>(bestIdx)].x += fx / len;
          acc[static_cast<std::size_t>(bestIdx)].y += fy / len;
          cnt[static_cast<std::size_t>(bestIdx)] += 1;
        }
      }
    }

    if (!anyAssigned) {
      break;
    }

    std::vector<P> newNodes;
    newNodes.reserve(nodes.size() / 2 + 8u);

    // Spawn one growth step from each influenced node.
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
      const int c = cnt[static_cast<std::size_t>(i)];
      if (c <= 0) continue;

      Vec2 dir = acc[static_cast<std::size_t>(i)];
      dir.x /= static_cast<float>(c);
      dir.y /= static_cast<float>(c);

      // Convert to a grid step.
      const float ax = std::fabs(dir.x);
      const float ay = std::fabs(dir.y);

      int sx = 0;
      int sy = 0;

      if (ax > ay) {
        sx = (dir.x > 0.0f) ? 1 : -1;
      } else if (ay > ax) {
        sy = (dir.y > 0.0f) ? 1 : -1;
      } else {
        // Deterministic tiebreak when the direction is near-diagonal.
        const std::uint32_t t = HashCoords32(nodes[static_cast<std::size_t>(i)].x, nodes[static_cast<std::size_t>(i)].y,
                                             seed32 ^ static_cast<std::uint32_t>(iter) ^ 0x51AB1E5u);
        if (t & 1u) {
          sx = (dir.x >= 0.0f) ? 1 : -1;
        } else {
          sy = (dir.y >= 0.0f) ? 1 : -1;
        }
      }

      if (sx == 0 && sy == 0) continue;

      const P cur = nodes[static_cast<std::size_t>(i)];
      P target{cur.x + sx * step, cur.y + sy * step};
      target.x = std::clamp(target.x, 1, std::max(1, w - 2));
      target.y = std::clamp(target.y, 1, std::max(1, h - 2));

      // Snap to a nearby suitable tile (land + empty/road).
      P snapped = target;
      if (!FindNearestWaypointTile(world, target.x, target.y, /*maxR=*/3, /*allowWater=*/false,
                                  seed32 ^ HashCoords32(cur.x, cur.y, static_cast<std::uint32_t>(iter) ^ 0xC01A1E5u),
                                  snapped)) {
        continue;
      }

      // Avoid spawning duplicate / extremely close nodes.
      bool dup = false;
      for (const P& n : nodes) {
        if (std::abs(n.x - snapped.x) + std::abs(n.y - snapped.y) <= 1) {
          dup = true;
          break;
        }
      }
      if (dup) continue;

      // Carve a new avenue-class arterial segment.
      CarveRoadCurvy(world, rng, cur, snapped, /*level=*/2, /*allowBridges=*/false,
                    seed32 ^ HashCoords32(cur.x, cur.y, static_cast<std::uint32_t>(iter) ^ 0x5CA1E5u));

      // Only accept the node if the carve actually created a road at the target.
      if (world.inBounds(snapped.x, snapped.y) && world.at(snapped.x, snapped.y).overlay == Overlay::Road) {
        newNodes.push_back(snapped);
      }
    }

    if (newNodes.empty()) {
      break;
    }

    // Commit new nodes.
    for (const P& p : newNodes) {
      addNodeIfFar(p, /*minSep=*/1);
    }

    // Prune attractors that have been reached.
    std::vector<P> kept;
    kept.reserve(attractors.size());

    for (const P& a : attractors) {
      bool reached = false;
      for (const P& n : nodes) {
        if (dist2(a, n) <= killR2) {
          reached = true;
          break;
        }
      }
      if (!reached) kept.push_back(a);
    }

    attractors.swap(kept);
  }
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

    CarveRoadCurvy(world, rng, a, b, beltwayLevel, /*allowBridges=*/true, seed32 ^ HashCoords32(a.x, a.y, 0xB17BEEFu) ^ HashCoords32(b.x, b.y, 0xB17BEEFu));
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
    CarveRoadCurvy(world, rng, hub, target, /*level=*/2, /*allowBridges=*/true, seed32 ^ HashCoords32(hub.x, hub.y, 0x05B0A1E5u) ^ HashCoords32(target.x, target.y, 0x05B0A1E5u));
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
// Road-network "stitching": opportunistic short bridges
// -----------------------------
//
// The terrain generator can produce rivers that cleanly bisect districts.
// Arterials generally bridge them, but local street grids may remain split
// into multiple components, hurting accessibility and later traffic routing.
//
// This pass looks for *single-tile* water gaps between two existing road
// components and selectively places bridges to reconnect the network.

static void ComputeRoadComponents(const World& world, std::vector<int>& outComp, std::vector<int>& outSize)
{
  const int w = world.width();
  const int h = world.height();
  const int n = w * h;
  outComp.assign(static_cast<std::size_t>(std::max(0, n)), -1);
  outSize.clear();

  if (w <= 0 || h <= 0) return;

  std::vector<int> q;
  q.reserve(4096);

  int compId = 0;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = y * w + x;
      if (outComp[static_cast<std::size_t>(idx)] != -1) continue;
      if (world.at(x, y).overlay != Overlay::Road) continue;

      // BFS.
      q.clear();
      std::size_t head = 0;
      q.push_back(idx);
      outComp[static_cast<std::size_t>(idx)] = compId;
      int count = 0;

      while (head < q.size()) {
        const int cur = q[head++];
        ++count;

        const int cx = cur % w;
        const int cy = cur / w;

        constexpr int dx[4] = {1, -1, 0, 0};
        constexpr int dy[4] = {0, 0, 1, -1};

        for (int k = 0; k < 4; ++k) {
          const int nx = cx + dx[k];
          const int ny = cy + dy[k];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const int nidx = ny * w + nx;
          if (outComp[static_cast<std::size_t>(nidx)] != -1) continue;
          if (world.at(nx, ny).overlay != Overlay::Road) continue;
          outComp[static_cast<std::size_t>(nidx)] = compId;
          q.push_back(nidx);
        }
      }

      outSize.push_back(count);
      ++compId;
    }
  }
}

struct BridgeCandidate {
  int x = 0;
  int y = 0;
  int ca = -1;
  int cb = -1;
  int level = 1;
  float score = 0.0f;
  std::uint32_t tie = 0;
};

static void StitchNarrowWaterBridges(World& world, const std::vector<P>& hubs, std::uint32_t seed32)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  std::vector<int> comp;
  std::vector<int> compSize;
  ComputeRoadComponents(world, comp, compSize);
  if (compSize.size() <= 1) return;

  auto compAt = [&](int x, int y) -> int {
    if (x < 0 || y < 0 || x >= w || y >= h) return -1;
    if (world.at(x, y).overlay != Overlay::Road) return -1;
    return comp[static_cast<std::size_t>(y * w + x)];
  };

  std::vector<BridgeCandidate> cands;
  cands.reserve(256);

  const int minDim = std::min(w, h);
  const float diag = static_cast<float>(w + h);

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain != Terrain::Water) continue;
      if (t.overlay != Overlay::None) continue; // don't overwrite existing bridges etc.

      // East-west gap.
      {
        const int c0 = compAt(x - 1, y);
        const int c1 = compAt(x + 1, y);
        if (c0 >= 0 && c1 >= 0 && c0 != c1) {
          BridgeCandidate bc;
          bc.x = x;
          bc.y = y;
          bc.ca = c0;
          bc.cb = c1;
          bc.level = std::max(ClampRoadLevel(static_cast<int>(world.at(x - 1, y).level)),
                              ClampRoadLevel(static_cast<int>(world.at(x + 1, y).level)));

          const int hubDist = NearestHubDist(hubs, x, y);
          const float d01 = std::clamp(static_cast<float>(hubDist) / std::max(1.0f, diag), 0.0f, 1.0f);

          // Prefer bridges that connect big components and are somewhat near the core.
          const float sizeScore = static_cast<float>(compSize[static_cast<std::size_t>(c0)] +
                                                     compSize[static_cast<std::size_t>(c1)]);
          bc.score = sizeScore * 0.60f + (1.0f - d01) * 45.0f + (static_cast<float>(bc.level) * 18.0f);

          // Slight bias away from the extreme map edge (avoid random sea causeways).
          const int edgeDist = std::min(std::min(x, w - 1 - x), std::min(y, h - 1 - y));
          const float edge01 = 1.0f - std::clamp(static_cast<float>(edgeDist) / std::max(1.0f, static_cast<float>(minDim) * 0.5f),
                                                 0.0f, 1.0f);
          bc.score -= edge01 * 20.0f;

          bc.tie = HashCoords32(x, y, seed32 ^ 0xBEEFB00Bu);
          cands.push_back(bc);
        }
      }

      // North-south gap.
      {
        const int c0 = compAt(x, y - 1);
        const int c1 = compAt(x, y + 1);
        if (c0 >= 0 && c1 >= 0 && c0 != c1) {
          BridgeCandidate bc;
          bc.x = x;
          bc.y = y;
          bc.ca = c0;
          bc.cb = c1;
          bc.level = std::max(ClampRoadLevel(static_cast<int>(world.at(x, y - 1).level)),
                              ClampRoadLevel(static_cast<int>(world.at(x, y + 1).level)));

          const int hubDist = NearestHubDist(hubs, x, y);
          const float d01 = std::clamp(static_cast<float>(hubDist) / std::max(1.0f, diag), 0.0f, 1.0f);

          const float sizeScore = static_cast<float>(compSize[static_cast<std::size_t>(c0)] +
                                                     compSize[static_cast<std::size_t>(c1)]);
          bc.score = sizeScore * 0.60f + (1.0f - d01) * 45.0f + (static_cast<float>(bc.level) * 18.0f);

          const int edgeDist = std::min(std::min(x, w - 1 - x), std::min(y, h - 1 - y));
          const float edge01 = 1.0f - std::clamp(static_cast<float>(edgeDist) / std::max(1.0f, static_cast<float>(minDim) * 0.5f),
                                                 0.0f, 1.0f);
          bc.score -= edge01 * 20.0f;

          bc.tie = HashCoords32(x, y, seed32 ^ 0xBEEFB00Bu);
          cands.push_back(bc);
        }
      }
    }
  }

  if (cands.empty()) return;

  std::sort(cands.begin(), cands.end(), [](const BridgeCandidate& a, const BridgeCandidate& b) {
    if (a.score != b.score) return a.score > b.score;
    return a.tie < b.tie;
  });

  // Budget: bigger maps can afford more stitch bridges.
  int maxBridges = std::clamp(minDim / 28, 2, 8);
  // If there are lots of components, allow a couple more.
  if (compSize.size() >= 4) maxBridges = std::min(10, maxBridges + 2);

  std::vector<P> chosen;
  chosen.reserve(static_cast<std::size_t>(maxBridges));

  for (const BridgeCandidate& bc : cands) {
    if (static_cast<int>(chosen.size()) >= maxBridges) break;

    // Keep bridges somewhat separated so we don't turn rivers into solid highways.
    bool tooClose = false;
    for (const P& p : chosen) {
      const int d = std::abs(p.x - bc.x) + std::abs(p.y - bc.y);
      if (d < 10) {
        tooClose = true;
        break;
      }
    }
    if (tooClose) continue;

    // Final safety check.
    if (!world.inBounds(bc.x, bc.y)) continue;
    Tile& t = world.at(bc.x, bc.y);
    if (t.terrain != Terrain::Water) continue;
    if (t.overlay != Overlay::None) continue;

    SetRoadWithLevel(world, bc.x, bc.y, bc.level, /*allowBridges=*/true);
    chosen.push_back(P{bc.x, bc.y});
  }
}

// -----------------------------
// Signature parks / greenways
// -----------------------------

static int PickCBDHubIndex(const std::vector<P>& hubs, int w, int h, std::uint32_t seed32)
{
  if (hubs.empty()) return -1;
  const int cx = w / 2;
  const int cy = h / 2;

  int best = 0;
  int bestD = std::numeric_limits<int>::max();
  std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();

  for (int i = 0; i < static_cast<int>(hubs.size()); ++i) {
    const P& p = hubs[static_cast<std::size_t>(i)];
    const int d = std::abs(p.x - cx) + std::abs(p.y - cy);
    const std::uint32_t tie = HashCoords32(p.x, p.y, seed32 ^ 0xC8D0BEEF);
    if (d < bestD || (d == bestD && tie < bestTie)) {
      bestD = d;
      bestTie = tie;
      best = i;
    }
  }

  return best;
}

static bool FindParkStartInBlock(const World& world, const CityBlocksResult& cb, const CityBlock& b, int inset,
                                int cx, int cy, std::uint32_t seed32, Point& out)
{
  const int w = cb.w;
  const int h = cb.h;
  if (w <= 0 || h <= 0) return false;

  const int bid = b.id;
  if (bid < 0) return false;

  const int maxR = std::max(8, std::min(b.maxX - b.minX, b.maxY - b.minY) / 2);

  int bestD = std::numeric_limits<int>::max();
  std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();
  bool found = false;

  for (int dy = -maxR; dy <= maxR; ++dy) {
    for (int dx = -maxR; dx <= maxR; ++dx) {
      const int x = cx + dx;
      const int y = cy + dy;
      if (x < 0 || y < 0 || x >= w || y >= h) continue;

      const int d = std::abs(dx) + std::abs(dy);
      if (d > maxR) continue;

      const int idx = y * w + x;
      if (cb.tileToBlock[static_cast<std::size_t>(idx)] != bid) continue;

      if (inset > 0) {
        if (x - b.minX < inset) continue;
        if (b.maxX - x < inset) continue;
        if (y - b.minY < inset) continue;
        if (b.maxY - y < inset) continue;
      }

      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) continue;
      if (t.overlay != Overlay::None) continue;

      const std::uint32_t tie = HashCoords32(x, y, seed32);
      if (d < bestD || (d == bestD && tie < bestTie)) {
        bestD = d;
        bestTie = tie;
        out = Point{x, y};
        found = true;
      }
    }
  }

  return found;
}

static int PlaceParkBlobInBlock(World& world, const CityBlocksResult& cb, const CityBlock& b, int targetArea,
                               int inset, std::uint32_t seed32)
{
  if (targetArea <= 0) return 0;
  const int w = cb.w;
  const int h = cb.h;
  const int n = w * h;
  if (w <= 0 || h <= 0 || n <= 0) return 0;

  const int bid = b.id;
  if (bid < 0) return 0;

  const int cx = (b.minX + b.maxX) / 2;
  const int cy = (b.minY + b.maxY) / 2;

  Point start;
  if (!FindParkStartInBlock(world, cb, b, inset, cx, cy, seed32 ^ 0xFACEB00Cu, start)) {
    return 0;
  }

  std::vector<std::uint8_t> seen;
  seen.assign(static_cast<std::size_t>(n), 0);

  std::vector<int> q;
  q.reserve(static_cast<std::size_t>(targetArea) * 4u);
  std::size_t head = 0;

  auto push = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    const int idx = y * w + x;
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (seen[uidx]) return;
    if (cb.tileToBlock[uidx] != bid) return;

    if (inset > 0) {
      if (x - b.minX < inset) return;
      if (b.maxX - x < inset) return;
      if (y - b.minY < inset) return;
      if (b.maxY - y < inset) return;
    }

    const Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water) return;
    if (t.overlay != Overlay::None) return;

    seen[uidx] = 1;
    q.push_back(idx);
  };

  push(start.x, start.y);

  // Local RNG so the park shape doesn't depend on global RNG call order.
  RNG prng((static_cast<std::uint64_t>(seed32) << 32) ^ static_cast<std::uint64_t>(bid) ^ 0x9E3779B97F4A7C15ULL);

  std::vector<int> chosen;
  chosen.reserve(static_cast<std::size_t>(targetArea));

  while (head < q.size() && static_cast<int>(chosen.size()) < targetArea) {
    const int cur = q[head++];
    chosen.push_back(cur);

    const int x = cur % w;
    const int y = cur / w;

    int order[4] = {0, 1, 2, 3};
    for (int i = 0; i < 4; ++i) {
      const int j = prng.rangeInt(i, 3);
      std::swap(order[i], order[j]);
    }

    constexpr int dx[4] = {1, -1, 0, 0};
    constexpr int dy[4] = {0, 0, 1, -1};

    for (int oi = 0; oi < 4; ++oi) {
      const int k = order[oi];
      const int nx = x + dx[k];
      const int ny = y + dy[k];

      // Slightly bias growth to be less diamond-shaped by randomly skipping a few frontier pushes.
      const float skip = TileRand01(nx, ny, seed32 ^ 0x13579BDFu);
      if (skip < 0.06f) continue;

      push(nx, ny);
    }
  }

  int placed = 0;
  for (int idx : chosen) {
    const int x = idx % w;
    const int y = idx / w;
    Tile& t = world.at(x, y);
    if (t.overlay != Overlay::None) continue;
    t.overlay = Overlay::Park;
    t.level = 1;
    t.occupants = 0;
    ++placed;
  }

  return placed;
}

static void PlaceMajorParksFromBlocks(World& world,
                                     const CityBlocksResult& cb,
                                     const std::vector<std::vector<Point>>& blockRoadAdj,
                                     const std::vector<P>& hubs,
                                     std::uint32_t seed32,
                                     const ProcGenConfig& cfg)
{
  (void)blockRoadAdj; // kept for future "entrance" heuristics

  if (cfg.zoneChance <= 0.0f) return;
  if (cfg.parkChance <= 0.0f) return;

  const int w = cb.w;
  const int h = cb.h;
  if (w <= 0 || h <= 0) return;

  const int minDim = std::min(w, h);
  if (minDim < 48) return; // tiny maps don't have room for signature parks

  const int cbdIdx = PickCBDHubIndex(hubs, w, h, seed32 ^ 0xC0DECAFEu);
  const P cbd = (cbdIdx >= 0) ? hubs[static_cast<std::size_t>(cbdIdx)] : P{w / 2, h / 2};

  // Number of major parks depends on map size.
  int want = (minDim >= 96) ? 2 : 1;

  struct BlockPick {
    int bid = -1;
    int cx = 0;
    int cy = 0;
    float score = -1e9f;
    std::uint32_t tie = 0;
  };

  std::vector<BlockPick> picks;
  picks.reserve(static_cast<std::size_t>(want));

  auto scoreBlock = [&](const CityBlock& b, int variant) -> float {
    if (b.id < 0) return -1e9f;

    const int bw = (b.maxX - b.minX + 1);
    const int bh = (b.maxY - b.minY + 1);
    if (bw < 12 || bh < 12) return -1e9f;
    if (b.area < 220) return -1e9f;

    // We want parks surrounded by streets so they create a "front".
    if (b.roadEdges < 18) return -1e9f;

    const int cx = (b.minX + b.maxX) / 2;
    const int cy = (b.minY + b.maxY) / 2;

    const int dCBD = std::abs(cx - cbd.x) + std::abs(cy - cbd.y);
    const float d01 = std::clamp(static_cast<float>(dCBD) / static_cast<float>(w + h), 0.0f, 1.0f);

    const float area01 = std::clamp(static_cast<float>(b.area) / 700.0f, 0.0f, 1.0f);

    const float bound = static_cast<float>(std::max(1, b.boundaryEdges()));
    const float water01 = std::clamp(static_cast<float>(b.waterEdges) / bound, 0.0f, 1.0f);

    // Variant 0: central park near CBD.
    // Variant 1: waterfront park / green anchor.
    float s = 0.0f;
    if (variant == 0) {
      s += (1.0f - d01) * 0.62f;
      s += area01 * 0.28f;
      s += water01 * 0.10f;
    } else {
      s += water01 * 0.55f;
      s += (1.0f - d01) * 0.25f;
      s += area01 * 0.20f;
    }

    // Prefer blocks with some hub proximity (avoid putting signature parks in total wilderness).
    const int hubDist = NearestHubDist(hubs, cx, cy);
    const float h01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(w + h), 0.0f, 1.0f);
    s += (1.0f - h01) * 0.08f;

    // Tie-break jitter.
    s += TileRand01(cx, cy, seed32 ^ 0x1CEB00DAu) * 0.02f;

    return s;
  };

  for (int variant = 0; variant < want; ++variant) {
    BlockPick best;

    for (const CityBlock& b : cb.blocks) {
      const float s = scoreBlock(b, variant);
      if (s < -1e8f) continue;
      const int cx = (b.minX + b.maxX) / 2;
      const int cy = (b.minY + b.maxY) / 2;

      // Keep major parks spread out.
      bool tooClose = false;
      for (const BlockPick& p : picks) {
        const int d = std::abs(p.cx - cx) + std::abs(p.cy - cy);
        if (d < minDim / 3) {
          tooClose = true;
          break;
        }
      }
      if (tooClose) continue;

      const std::uint32_t tie = HashCoords32(b.id, variant, seed32 ^ 0xABCDEF01u);
      if (s > best.score || (s == best.score && tie < best.tie)) {
        best.bid = b.id;
        best.cx = cx;
        best.cy = cy;
        best.score = s;
        best.tie = tie;
      }
    }

    if (best.bid >= 0) {
      picks.push_back(best);
    }
  }

  for (std::size_t i = 0; i < picks.size(); ++i) {
    const int bid = picks[i].bid;
    if (bid < 0 || bid >= static_cast<int>(cb.blocks.size())) continue;

    const CityBlock& b = cb.blocks[static_cast<std::size_t>(bid)];
    const int target = std::clamp(b.area / 6, 34, 180);
    const int inset = (target > 120) ? 2 : 1;

    (void)PlaceParkBlobInBlock(world, cb, b, target, inset, seed32 ^ (0x9E3779B9u * static_cast<std::uint32_t>(i + 1)));
  }
}

static void PlaceWaterfrontGreenways(World& world, const std::vector<P>& hubs, std::uint32_t seed32, const ProcGenConfig& cfg)
{
  if (cfg.zoneChance <= 0.0f) return;
  if (cfg.parkChance <= 0.0f) return;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  const int n = w * h;
  const int minDim = std::min(w, h);

  // Budget scales with map size and global park chance.
  const float parkScale = std::clamp(cfg.parkChance / 0.06f, 0.35f, 2.0f);
  int budget = static_cast<int>(std::round(static_cast<float>(minDim) * 3.0f * parkScale));
  budget = std::clamp(budget, 40, 320);

  auto isShore = [&](int x, int y) -> bool {
    if (!world.inBounds(x, y)) return false;
    const Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water) return false;
    if (t.overlay != Overlay::None) return false;
    return HasAdjacentWater4(world, x, y);
  };

  // Pick a few good shoreline seeds.
  struct Seed {
    int x = 0;
    int y = 0;
    float score = -1e9f;
    std::uint32_t tie = 0;
  };

  std::vector<Seed> candidates;
  candidates.reserve(2048);

  const float diag = static_cast<float>(w + h);

  for (int y = 1; y < h - 1; ++y) {
    for (int x = 1; x < w - 1; ++x) {
      if (!isShore(x, y)) continue;

      const int hubDist = NearestHubDist(hubs, x, y);
      const float d01 = std::clamp(static_cast<float>(hubDist) / std::max(1.0f, diag), 0.0f, 1.0f);

      float s = (1.0f - d01) * 0.55f;
      if (world.hasAdjacentRoad(x, y)) s += 0.35f;

      // Coastlines near the edge are allowed (they often are the edge), but slightly prefer
      // non-corner areas to avoid silly "park pinstripes".
      const int edgeDist = std::min(std::min(x, w - 1 - x), std::min(y, h - 1 - y));
      const float edge01 = 1.0f - std::clamp(static_cast<float>(edgeDist) / std::max(1.0f, static_cast<float>(minDim) * 0.5f),
                                             0.0f, 1.0f);
      s -= edge01 * 0.08f;

      const std::uint32_t tie = HashCoords32(x, y, seed32 ^ 0x5151C0DEu);
      s += Hash01From32(tie) * 0.02f;

      candidates.push_back(Seed{x, y, s, tie});
    }
  }

  if (candidates.empty()) return;

  std::sort(candidates.begin(), candidates.end(), [](const Seed& a, const Seed& b) {
    if (a.score != b.score) return a.score > b.score;
    return a.tie < b.tie;
  });

  int seeds = std::clamp(minDim / 64 + 2, 2, 4);
  if (budget < 70) seeds = std::min(seeds, 2);

  std::vector<Seed> chosen;
  chosen.reserve(static_cast<std::size_t>(seeds));

  for (const Seed& s : candidates) {
    if (static_cast<int>(chosen.size()) >= seeds) break;

    bool tooClose = false;
    for (const Seed& c : chosen) {
      const int d = std::abs(c.x - s.x) + std::abs(c.y - s.y);
      if (d < minDim / 4) {
        tooClose = true;
        break;
      }
    }
    if (tooClose) continue;
    chosen.push_back(s);
  }

  if (chosen.empty()) return;

  const int perSeed = std::max(10, budget / static_cast<int>(chosen.size()));

  std::vector<std::uint8_t> seen;
  seen.assign(static_cast<std::size_t>(n), 0);

  std::vector<int> q;
  q.reserve(4096);

  for (std::size_t si = 0; si < chosen.size(); ++si) {
    q.clear();
    std::size_t head = 0;

    const int sx = chosen[si].x;
    const int sy = chosen[si].y;
    const int sIdx = sy * w + sx;
    if (sIdx < 0 || sIdx >= n) continue;

    q.push_back(sIdx);
    seen[static_cast<std::size_t>(sIdx)] = 1;

    int placed = 0;

    while (head < q.size() && placed < perSeed) {
      const int cur = q[head++];
      const int x = cur % w;
      const int y = cur / w;

      if (isShore(x, y)) {
        Tile& t = world.at(x, y);
        if (t.overlay == Overlay::None) {
          t.overlay = Overlay::Park;
          t.level = 1;
          t.occupants = 0;
          ++placed;

          // Occasionally widen the greenway one tile inland for a more "promenade" feel.
          const float widen = TileRand01(x, y, seed32 ^ 0xA5A5BEEF);
          if (widen < 0.22f) {
            constexpr int dx[4] = {1, -1, 0, 0};
            constexpr int dy[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
              const int nx = x + dx[k];
              const int ny = y + dy[k];
              if (!world.inBounds(nx, ny)) continue;
              Tile& nt = world.at(nx, ny);
              if (nt.terrain == Terrain::Water) continue;
              if (nt.overlay != Overlay::None) continue;
              if (HasAdjacentWater4(world, nx, ny)) continue; // keep widening inland
              // A quick road-adjacency check avoids turning major boulevards into parks.
              if (world.hasAdjacentRoad(nx, ny) && TileRand01(nx, ny, seed32 ^ 0x1234ABCDu) < 0.45f) continue;
              nt.overlay = Overlay::Park;
              nt.level = 1;
              nt.occupants = 0;
              break;
            }
          }
        }
      }

      // Shoreline BFS neighbors.
      constexpr int dx[4] = {1, -1, 0, 0};
      constexpr int dy[4] = {0, 0, 1, -1};
      for (int k = 0; k < 4; ++k) {
        const int nx = x + dx[k];
        const int ny = y + dy[k];
        if (nx <= 0 || ny <= 0 || nx >= w - 1 || ny >= h - 1) continue;
        const int nIdx = ny * w + nx;
        const std::size_t u = static_cast<std::size_t>(nIdx);
        if (seen[u]) continue;

        // Only flood along coastline candidates.
        if (!isShore(nx, ny)) continue;

        seen[u] = 1;
        q.push_back(nIdx);
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
  // Phase 0: signature parks and waterfront greenways.
  // These are placed *before* the standard road-edge seeding so they influence
  // the land value field and produce more recognizable city landmarks.
  // -------------------------------------------------------------------------
  PlaceMajorParksFromBlocks(world, cb, blockRoadAdj, hubs, seed32 ^ 0xC3A7E5E1u, cfg);
  PlaceWaterfrontGreenways(world, hubs, seed32 ^ 0x6A9E4A71u, cfg);

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

// -----------------------------
// Macro terrain presets
// -----------------------------
//
// These operate purely on the heightfield before erosion/classification.
//
// IMPORTANT: ProcGenTerrainPreset::Classic MUST preserve the previous
// generation behavior exactly so that existing delta-saves remain stable.

static float Smoothstep(float e0, float e1, float x)
{
  if (e0 == e1) return (x < e0) ? 0.0f : 1.0f;
  const float t = Clamp01((x - e0) / (e1 - e0));
  return t * t * (3.0f - 2.0f * t);
}

static void ApplyTerrainPreset(std::vector<float>& heights, int width, int height, std::uint32_t seed32,
                              const ProcGenConfig& cfg)
{
  if (width <= 0 || height <= 0) return;
  if (heights.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) return;

  const ProcGenTerrainPreset preset = cfg.terrainPreset;
  const float strength = std::clamp(cfg.terrainPresetStrength, 0.0f, 2.5f);

  // Classic means: don't touch the heightfield at all.
  if (preset == ProcGenTerrainPreset::Classic || strength <= 0.0001f) {
    return;
  }

  const float cx = (static_cast<float>(width) - 1.0f) * 0.5f;
  const float cy = (static_cast<float>(height) - 1.0f) * 0.5f;
  const float invCx = (cx > 0.0f) ? (1.0f / cx) : 0.0f;
  const float invCy = (cy > 0.0f) ? (1.0f / cy) : 0.0f;
  const float minDim = static_cast<float>(std::min(width, height));

  const float coastScale = std::max(0.0001f, cfg.terrainScale * 0.65f);

  auto radial = [&](int x, int y) -> float {
    const float nx = (static_cast<float>(x) - cx) * invCx;
    const float ny = (static_cast<float>(y) - cy) * invCy;
    return std::sqrt(nx * nx + ny * ny);
  };

  // Local preset RNG (do NOT perturb the main ProcGen RNG stream).
  RNG prng(static_cast<std::uint64_t>(seed32) ^ 0x9E3779B97F4A7C15ULL);

  // Precompute a meandering river centerline for RiverValley.
  bool riverHorizontal = false;
  std::vector<float> riverLine;
  float riverBase = 0.0f;

  if (preset == ProcGenTerrainPreset::RiverValley) {
    riverHorizontal = ((HashCoords32(width, height, seed32 ^ 0xBADC0FFEu) & 1u) != 0u);
    const int len = riverHorizontal ? width : height;
    const int oth = riverHorizontal ? height : width;
    riverLine.resize(static_cast<std::size_t>(std::max(0, len)), 0.0f);

    const float base01 = 0.35f + 0.30f * prng.nextF01();
    riverBase = base01 * static_cast<float>(oth);

    const float amp = static_cast<float>(oth) * (0.18f + 0.10f * prng.nextF01());
    const float smallAmp = amp * 0.35f;

    for (int i = 0; i < len; ++i) {
      const float t = (len > 1) ? (static_cast<float>(i) / static_cast<float>(len - 1)) : 0.0f;
      // Use 1D fbm (x=t*k, y=const) to get a smooth meander.
      const float n0 = fbmNormalized(t * 2.2f, 7.3f, seed32 ^ 0xC0FFEEu, 4) * 2.0f - 1.0f;
      const float n1 = fbmNormalized(t * 6.8f, 1.1f, seed32 ^ 0xFACEB00Cu, 2) * 2.0f - 1.0f;
      float p = riverBase + n0 * amp + n1 * smallAmp;
      // Mild smooth drift so rivers don't always stay centered.
      const float drift = fbmNormalized(t * 1.15f, 3.9f, seed32 ^ 0x13579BDFu, 2) * 2.0f - 1.0f;
      p += (t - 0.5f) * static_cast<float>(oth) * (0.10f * drift);
      // Keep river away from the very edges so it can have banks.
      p = std::clamp(p, 2.0f, static_cast<float>(oth) - 3.0f);
      riverLine[static_cast<std::size_t>(i)] = p;
    }
  }

  // Precompute a deeper canyon centerline.
  bool canyonHorizontal = false;
  std::vector<float> canyonLine;
  float canyonBase = 0.0f;

  if (preset == ProcGenTerrainPreset::Canyon) {
    canyonHorizontal = ((HashCoords32(width, height, seed32 ^ 0xCA7B0A1Bu) & 1u) != 0u);
    const int len = canyonHorizontal ? width : height;
    const int oth = canyonHorizontal ? height : width;
    canyonLine.resize(static_cast<std::size_t>(std::max(0, len)), 0.0f);

    // Keep the canyon fairly central so it feels like a "feature" of the map.
    const float base01 = 0.42f + 0.16f * prng.nextF01();
    canyonBase = base01 * static_cast<float>(oth);

    // Big meanders.
    const float amp = static_cast<float>(oth) * (0.22f + 0.10f * prng.nextF01());
    const float smallAmp = amp * 0.42f;

    for (int i = 0; i < len; ++i) {
      const float t = (len > 1) ? (static_cast<float>(i) / static_cast<float>(len - 1)) : 0.0f;
      const float n0 = fbmNormalized(t * 1.65f, 2.9f, seed32 ^ 0xCA11AB1Eu, 4) * 2.0f - 1.0f;
      const float n1 = fbmNormalized(t * 5.75f, 7.7f, seed32 ^ 0x0DDC0FFEu, 2) * 2.0f - 1.0f;
      float p = canyonBase + n0 * amp + n1 * smallAmp;

      // Drift bias: encourages an S-curve that avoids always being centered.
      const float drift = fbmNormalized(t * 0.95f, 9.1f, seed32 ^ 0x13579BDFu, 2) * 2.0f - 1.0f;
      p += (t - 0.5f) * static_cast<float>(oth) * (0.14f * drift);

      p = std::clamp(p, 2.0f, static_cast<float>(oth) - 3.0f);
      canyonLine[static_cast<std::size_t>(i)] = p;
    }
  }

  // Precompute a river delta (main channel + two distributaries near the mouth).
  bool deltaHorizontal = false;
  bool deltaMouthAtMax = true; // bottom or right
  std::vector<float> deltaMain;
  std::vector<float> deltaB1;
  std::vector<float> deltaB2;
  float deltaMouth = 0.0f;
  float deltaSource = 0.0f;
  int deltaLen = 0;
  int deltaOth = 0;

  if (preset == ProcGenTerrainPreset::Delta) {
    // Pick a coast side deterministically.
    const int side = static_cast<int>(HashCoords32(width, height, seed32 ^ 0xD311A5E5u) % 4u);
    // 0 = top, 1 = right, 2 = bottom, 3 = left.
    deltaHorizontal = (side == 1 || side == 3);
    deltaMouthAtMax = (side == 2 || side == 1);
    deltaLen = deltaHorizontal ? width : height;
    deltaOth = deltaHorizontal ? height : width;
    deltaMain.resize(static_cast<std::size_t>(std::max(0, deltaLen)), 0.0f);
    deltaB1.resize(deltaMain.size(), 0.0f);
    deltaB2.resize(deltaMain.size(), 0.0f);

    // Mouth location along the coast.
    const float mouth01 = 0.38f + 0.24f * prng.nextF01();
    deltaMouth = mouth01 * static_cast<float>(deltaOth);

    // Source location on the opposite side.
    const float src01 = 0.28f + 0.44f * prng.nextF01();
    deltaSource = src01 * static_cast<float>(deltaOth);

    // Max branch divergence (in tiles) near the mouth.
    const float branchMax = static_cast<float>(deltaOth) * (0.08f + 0.06f * prng.nextF01());

    for (int i = 0; i < deltaLen; ++i) {
      const int ii = deltaMouthAtMax ? i : (deltaLen - 1 - i);
      const float t = (deltaLen > 1) ? (static_cast<float>(ii) / static_cast<float>(deltaLen - 1)) : 0.0f;

      // Blend from source position to mouth position.
      float p = Lerp(deltaSource, deltaMouth, t);

      // Long meanders that damp near the mouth (delta tends to straighten as it fans out).
      const float meander = (fbmNormalized(t * 2.05f, 3.1f, seed32 ^ 0xDE17A11Eu, 3) * 2.0f - 1.0f);
      const float damp = 1.0f - Smoothstep(0.72f, 0.98f, t);
      p += meander * static_cast<float>(deltaOth) * (0.12f * damp);

      // Small-scale wiggle.
      const float wiggle = (fbmNormalized(t * 6.8f, 9.7f, seed32 ^ 0xB16B00B5u, 2) * 2.0f - 1.0f);
      p += wiggle * static_cast<float>(deltaOth) * 0.020f;

      // Clamp away from edges.
      p = std::clamp(p, 2.0f, static_cast<float>(deltaOth) - 3.0f);
      deltaMain[static_cast<std::size_t>(i)] = p;

      // Distributaries: diverge near the mouth.
      const float div = Smoothstep(0.62f, 0.94f, t);
      const float off = div * branchMax;
      // Each branch gets its own slight noise.
      const float bN = (fbmNormalized(t * 8.1f, 1.3f, seed32 ^ 0x51A71D00u, 2) * 2.0f - 1.0f);
      deltaB1[static_cast<std::size_t>(i)] = std::clamp(p + off + bN * 1.6f, 2.0f, static_cast<float>(deltaOth) - 3.0f);
      deltaB2[static_cast<std::size_t>(i)] = std::clamp(p - off - bN * 1.6f, 2.0f, static_cast<float>(deltaOth) - 3.0f);
    }
  }

  // Volcano parameters (picked deterministically per-preset).
  float volcanoCraterR = 0.18f;
  float volcanoCraterInner = 0.10f;
  float volcanoRimSigma = 0.040f;
  if (preset == ProcGenTerrainPreset::Volcano) {
    volcanoCraterR = 0.16f + 0.06f * prng.nextF01();
    volcanoCraterInner = volcanoCraterR * (0.52f + 0.10f * prng.nextF01());
    volcanoRimSigma = 0.030f + 0.025f * prng.nextF01();
  }

  struct Island {
    float x = 0.0f;
    float y = 0.0f;
    float r = 10.0f;
  };

  std::vector<Island> islands;
  if (preset == ProcGenTerrainPreset::Archipelago) {
    const int n = std::clamp(static_cast<int>(std::round(minDim / 36.0f)) + 2, 3, 7);
    const float baseR = minDim * 0.20f;
    const float maxR = minDim * 0.34f;

    for (int i = 0; i < n; ++i) {
      Island isl;
      bool ok = false;
      for (int tries = 0; tries < 200 && !ok; ++tries) {
        isl.x = prng.rangeFloat(0.0f, static_cast<float>(width - 1));
        isl.y = prng.rangeFloat(0.0f, static_cast<float>(height - 1));
        isl.r = prng.rangeFloat(baseR, maxR);

        ok = true;
        for (const Island& other : islands) {
          const float dx = isl.x - other.x;
          const float dy = isl.y - other.y;
          const float d = std::sqrt(dx * dx + dy * dy);
          if (d < (isl.r + other.r) * 0.55f) {
            ok = false;
            break;
          }
        }
      }
      islands.push_back(isl);
    }
  }

  // Apply per-tile modification.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t i = Idx(x, y, width);
      float h = heights[i];

      const float r = radial(x, y);

      if (preset == ProcGenTerrainPreset::Island) {
        float fall = Smoothstep(0.78f, 1.22f, r);
        const float jitter = (fbmNormalized(static_cast<float>(x) * coastScale, static_cast<float>(y) * coastScale,
                                           seed32 ^ 0xA11CEB0Bu, 3) * 2.0f - 1.0f) * 0.16f;
        fall = Clamp01(fall + jitter);

        h -= fall * (0.52f * strength);
        h += (1.0f - fall) * (0.06f * strength);
      } else if (preset == ProcGenTerrainPreset::Archipelago) {
        // Multi-island mask.
        float mask = 0.0f;
        for (const Island& isl : islands) {
          const float dx = static_cast<float>(x) - isl.x;
          const float dy = static_cast<float>(y) - isl.y;
          const float d = std::sqrt(dx * dx + dy * dy);
          float t = 1.0f - (d / std::max(1.0f, isl.r));
          t = Clamp01(t);
          // Smooth edges.
          t = t * t * (3.0f - 2.0f * t);
          mask = std::max(mask, t);
        }

        // Ragged coastlines.
        const float rag = (fbmNormalized(static_cast<float>(x) * coastScale * 1.25f,
                                        static_cast<float>(y) * coastScale * 1.25f,
                                        seed32 ^ 0xB16B00B5u, 3) * 2.0f - 1.0f) * 0.18f;
        mask = Clamp01(mask + rag);

        // Edge falloff so we get surrounding ocean.
        const float edge = Smoothstep(0.85f, 1.30f, r);

        // Outside islands + near edges => push down aggressively.
        const float sea = std::max(edge, 1.0f - mask);

        // Blend: keep original noise for land interiors, but compress water areas.
        h = h * (0.45f + 0.90f * mask) - sea * (0.56f * strength);
      } else if (preset == ProcGenTerrainPreset::InlandSea) {
        // Central depression.
        const float sea = std::exp(-(r * r) / (2.0f * 0.42f * 0.42f));
        h -= sea * (0.62f * strength);

        // Surrounding ring uplift to keep land around the sea.
        const float dr = (r - 0.92f);
        const float ring = std::exp(-(dr * dr) / (2.0f * 0.18f * 0.18f));
        h += ring * (0.10f * strength);

        // A little coastline noise.
        const float n = (fbmNormalized(static_cast<float>(x) * coastScale * 0.85f,
                                      static_cast<float>(y) * coastScale * 0.85f,
                                      seed32 ^ 0xD00DFEEDu, 2) * 2.0f - 1.0f);
        h += n * (0.03f * strength);
      } else if (preset == ProcGenTerrainPreset::RiverValley) {
        if (!riverLine.empty()) {
          if (riverHorizontal) {
            const float y0 = riverLine[static_cast<std::size_t>(x)];
            const float d = std::abs(static_cast<float>(y) - y0);
            const float w01 = fbmNormalized(static_cast<float>(x) * 0.06f, 0.0f, seed32 ^ 0x1234ABCDu, 3);
            const float widthBase = 2.2f + w01 * 3.6f;
            const float bank = widthBase * 2.2f;
            const float t = Clamp01(1.0f - (d / widthBase));
            const float t2 = t * t;
            h -= t2 * (0.56f * strength);

            // Gentle banks/outwash.
            const float tb = Clamp01(1.0f - (d / bank));
            h -= tb * tb * (0.10f * strength);

            // Ensure a continuous wet core regardless of waterLevel.
            if (d < widthBase * 0.35f) {
              h = std::min(h, cfg.waterLevel - 0.12f - 0.04f * strength);
            }
          } else {
            const float x0 = riverLine[static_cast<std::size_t>(y)];
            const float d = std::abs(static_cast<float>(x) - x0);
            const float w01 = fbmNormalized(static_cast<float>(y) * 0.06f, 0.0f, seed32 ^ 0x5678DCBAu, 3);
            const float widthBase = 2.2f + w01 * 3.6f;
            const float bank = widthBase * 2.2f;
            const float t = Clamp01(1.0f - (d / widthBase));
            const float t2 = t * t;
            h -= t2 * (0.56f * strength);
            const float tb = Clamp01(1.0f - (d / bank));
            h -= tb * tb * (0.10f * strength);
            if (d < widthBase * 0.35f) {
              h = std::min(h, cfg.waterLevel - 0.12f - 0.04f * strength);
            }
          }
        }
      } else if (preset == ProcGenTerrainPreset::MountainRing) {
        // Ring-like ridge.
        const float ringR = 0.78f + (fbmNormalized(static_cast<float>(x) * coastScale * 0.40f,
                                                  static_cast<float>(y) * coastScale * 0.40f,
                                                  seed32 ^ 0xFEEDBEEFu, 2) - 0.5f) * 0.08f;
        const float dr = r - ringR;
        const float ring = std::exp(-(dr * dr) / (2.0f * 0.16f * 0.16f));
        h += ring * (0.32f * strength);

        // Basin inside the ring.
        const float basin = std::exp(-(r * r) / (2.0f * 0.55f * 0.55f));
        h -= basin * (0.14f * strength);
      } else if (preset == ProcGenTerrainPreset::Fjords) {
        // Glaciated coasts: edge sea + a rugged coastal mountain band.
        const int dxE = std::min(x, width - 1 - x);
        const int dyE = std::min(y, height - 1 - y);
        const float dEdge = static_cast<float>(std::min(dxE, dyE));
        const float denom = std::max(1.0f, (minDim * 0.5f));
        const float edge01 = std::clamp(dEdge / denom, 0.0f, 1.0f);

        // Water at the very edge, land inland.
        const float coast = Smoothstep(0.10f, 0.48f, edge01);
        const float sea = 1.0f - coast;
        h -= sea * (0.44f * strength);
        h += coast * (0.05f * strength);

        // Coastal mountains: a band inland from the coast.
        const float band = Smoothstep(0.14f, 0.26f, edge01) * (1.0f - Smoothstep(0.56f, 0.82f, edge01));
        const float rugged = (fbmNormalized(static_cast<float>(x) * coastScale * 0.55f,
                                            static_cast<float>(y) * coastScale * 0.55f,
                                            seed32 ^ 0xF10DDF00u, 3) * 2.0f - 1.0f);
        const float m = std::clamp(0.70f + 0.30f * rugged, 0.25f, 1.25f);
        h += band * m * (0.38f * strength);
      } else if (preset == ProcGenTerrainPreset::Canyon) {
        // Canyonlands: uplift into a plateau, then carve a deep winding canyon.
        h += (0.18f * strength);

        if (!canyonLine.empty()) {
          float d = 0.0f;
          if (canyonHorizontal) {
            const float y0 = canyonLine[static_cast<std::size_t>(x)];
            d = std::abs(static_cast<float>(y) - y0);
          } else {
            const float x0 = canyonLine[static_cast<std::size_t>(y)];
            d = std::abs(static_cast<float>(x) - x0);
          }

          const float w01 = fbmNormalized((canyonHorizontal ? static_cast<float>(x) : static_cast<float>(y)) * 0.055f,
                                          0.0f, seed32 ^ 0xCA7A0C00u, 3);
          const float widthBase = 1.7f + w01 * 2.7f;
          const float bank = widthBase * 3.4f;

          const float t = Clamp01(1.0f - (d / widthBase));
          h -= (t * t) * (0.82f * strength);

          // Wider eroded shoulders.
          const float tb = Clamp01(1.0f - (d / bank));
          h -= (tb * tb) * (0.18f * strength);

          // Ensure a continuous river core (visible even if river conversion is off).
          if (d < widthBase * 0.33f) {
            h = std::min(h, cfg.waterLevel - 0.10f - 0.04f * strength);
          }

          // Mesa terracing away from the canyon.
          if (d > bank * 1.1f && h > cfg.waterLevel + 0.14f) {
            const float step = 0.030f + 0.010f * (1.0f - std::min(strength, 1.0f));
            const float off = 0.35f;
            const float q = std::floor((h + off) / step) * step - off;
            h = Lerp(h, q, 0.40f * strength);
          }
        }
      } else if (preset == ProcGenTerrainPreset::Volcano) {
        // Volcanic cone + caldera.
        const float rn = std::clamp(r / 1.41421356f, 0.0f, 1.0f);

        // Cone: broad base with a sharper peak.
        const float cone = std::pow(std::max(0.0f, 1.0f - rn), 1.22f);
        const float rough = (fbmNormalized(static_cast<float>(x) * coastScale * 0.95f,
                                           static_cast<float>(y) * coastScale * 0.95f,
                                           seed32 ^ 0xBADA55E5u, 3) * 2.0f - 1.0f);
        h += cone * (0.62f * strength);
        h += rough * cone * (0.06f * strength);

        // Caldera depression.
        const float crater = 1.0f - Smoothstep(volcanoCraterInner, volcanoCraterR, rn);
        h -= crater * (0.70f * strength);

        // Rim uplift.
        const float dr = rn - volcanoCraterR;
        const float rim = std::exp(-(dr * dr) / (2.0f * volcanoRimSigma * volcanoRimSigma));
        h += rim * (0.22f * strength);
      } else if (preset == ProcGenTerrainPreset::Delta) {
        // Asymmetric coast + river delta. Use a macro downhill slope toward the mouth.
        if (!deltaMain.empty()) {
          const int a = deltaHorizontal ? x : y;
          const int ii = deltaMouthAtMax ? a : (deltaLen - 1 - a);
          const float t = (deltaLen > 1) ? (static_cast<float>(ii) / static_cast<float>(deltaLen - 1)) : 0.0f;

          // Slope: inland a bit higher, coast lower.
          h += (1.0f - t) * (0.08f * strength);
          h -= t * (0.36f * strength);

          // Wet coastal plain flattening.
          const float coastal = Smoothstep(0.70f, 0.98f, t);
          h -= coastal * coastal * (0.10f * strength);

          const float o = static_cast<float>(deltaHorizontal ? y : x);
          const float p0 = deltaMain[static_cast<std::size_t>(a)];
          const float p1 = deltaB1[static_cast<std::size_t>(a)];
          const float p2 = deltaB2[static_cast<std::size_t>(a)];

          const float d0 = std::abs(o - p0);
          const float d1 = std::abs(o - p1);
          const float d2 = std::abs(o - p2);

          const float widen = Smoothstep(0.62f, 1.0f, t);
          const float wBase = 2.0f + 1.3f * strength;
          const float w = wBase + widen * (4.2f + 2.4f * strength);
          const float bank = w * 2.25f;

          auto carve = [&](float d, float depthMul) {
            const float tc = Clamp01(1.0f - (d / w));
            const float tb = Clamp01(1.0f - (d / bank));
            h -= (tc * tc) * (0.50f * strength) * depthMul;
            h -= (tb * tb) * (0.12f * strength) * depthMul;

            if (d < w * 0.28f) {
              const float core = std::clamp(cfg.waterLevel - 0.10f - 0.04f * strength, -0.30f, 0.90f);
              h = std::min(h, core);
            }
          };

          carve(d0, 1.00f);
          carve(d1, 0.75f * widen);
          carve(d2, 0.75f * widen);

          // Marshy islands / sediment noise near the coast.
          const float marshN = (fbmNormalized(static_cast<float>(x) * coastScale * 1.20f,
                                              static_cast<float>(y) * coastScale * 1.20f,
                                              seed32 ^ 0xD311A5E5u, 3) * 2.0f - 1.0f);
          h -= marshN * coastal * (0.05f * strength);
        }
      }

      // Keep range stable-ish.
      heights[i] = std::clamp(h, -0.35f, 1.15f);
    }
  }

  // Fjords need a second pass to carve long, narrow inlets. Doing this as a
  // post-pass keeps the per-tile branch cheap and makes the result look more
  // "structural" than a simple distance-field.
  if (preset == ProcGenTerrainPreset::Fjords) {
    auto carveMinDiamond = [&](int cx, int cy, int rad, float floor, float bankRise) {
      rad = std::max(1, rad);
      const int x0 = std::max(0, cx - rad);
      const int x1 = std::min(width - 1, cx + rad);
      const int y0 = std::max(0, cy - rad);
      const int y1 = std::min(height - 1, cy + rad);

      for (int yy = y0; yy <= y1; ++yy) {
        for (int xx = x0; xx <= x1; ++xx) {
          const int dist = std::abs(xx - cx) + std::abs(yy - cy);
          if (dist > rad) continue;
          const float k = 1.0f - (static_cast<float>(dist) / static_cast<float>(rad));
          const float target = floor + (1.0f - k) * bankRise;
          const std::size_t idx = Idx(xx, yy, width);
          heights[idx] = std::min(heights[idx], target);
        }
      }
    };

    const int fjordCount = std::clamp(static_cast<int>(std::round(minDim / 42.0f)) + 3, 3, 11);
    const int extraWide = (minDim >= 200.0f) ? 1 : 0;

    for (int fi = 0; fi < fjordCount; ++fi) {
      const int side = prng.rangeInt(0, 3); // 0 top, 1 right, 2 bottom, 3 left

      float fx = 0.0f;
      float fy = 0.0f;
      float baseAngle = 0.0f;

      const int mx = std::clamp(static_cast<int>(std::round(minDim * 0.08f)), 2, std::max(2, width / 3));
      const int my = std::clamp(static_cast<int>(std::round(minDim * 0.08f)), 2, std::max(2, height / 3));

      if (side == 0) { // top
        fx = static_cast<float>(prng.rangeInt(mx, std::max(mx, width - 1 - mx)));
        fy = 0.0f;
        baseAngle = 1.5707963f;
      } else if (side == 2) { // bottom
        fx = static_cast<float>(prng.rangeInt(mx, std::max(mx, width - 1 - mx)));
        fy = static_cast<float>(height - 1);
        baseAngle = -1.5707963f;
      } else if (side == 3) { // left
        fx = 0.0f;
        fy = static_cast<float>(prng.rangeInt(my, std::max(my, height - 1 - my)));
        baseAngle = 0.0f;
      } else { // right
        fx = static_cast<float>(width - 1);
        fy = static_cast<float>(prng.rangeInt(my, std::max(my, height - 1 - my)));
        baseAngle = 3.1415927f;
      }

      // Inlet walk length and turning.
      const int steps = static_cast<int>(std::round(minDim * (0.45f + 0.35f * prng.nextF01())));
      float a = baseAngle + (prng.nextF01() - 0.5f) * 0.65f;

      const float floor = std::clamp(cfg.waterLevel - 0.12f - 0.06f * strength, -0.32f, 0.95f);
      const std::uint32_t fjSeed = seed32 ^ (0xF10DF00Du + static_cast<std::uint32_t>(fi + 1) * 0x9E3779B9u);

      for (int s = 0; s < steps; ++s) {
        const float tt = (steps > 1) ? (static_cast<float>(s) / static_cast<float>(steps - 1)) : 0.0f;

        // A little coherent steering so fjords don't look like pure random walks.
        const float steer = (fbmNormalized(fx * 0.055f, fy * 0.055f, fjSeed, 3) * 2.0f - 1.0f);
        a += steer * 0.10f + (prng.nextF01() - 0.5f) * 0.04f;

        fx += std::cos(a);
        fy += std::sin(a);

        const int ix = static_cast<int>(std::round(fx));
        const int iy = static_cast<int>(std::round(fy));
        if (ix < 2 || iy < 2 || ix >= width - 2 || iy >= height - 2) {
          break;
        }

        int rad = 2 + extraWide + ((tt < 0.22f) ? 1 : 0);
        if (prng.chance(0.06f)) rad += 1;

        // Banks taper slightly as we go inland.
        const float bankRise = (0.24f + 0.12f * strength) * (0.92f - 0.22f * tt);
        carveMinDiamond(ix, iy, rad, floor, bankRise);
      }
    }

    // Re-clamp after carving.
    for (float& h : heights) {
      h = std::clamp(h, -0.35f, 1.15f);
    }
  }
}

// -----------------------------
// Road hierarchy post-pass (v11)
//
// After the initial road carve (hubs/arterials/locals), we run a light-weight
// "centrality sampling" pass on the *road graph*:
//
//  1) Pick a set of "activity centers" (hubs, district centers, edges, plus a few
//     deterministic activity points).
//  2) Route between many center pairs using A* on the road graph.
//  3) Count per-tile traversal frequency (an approximation of betweenness
//     centrality / all-pairs flow).
//  4) Upgrade the most-used corridors to Avenue/Highway classes.
//
// The result is a clearer arterial structure and more believable zoning
// gradients (since road level influences initial zoning density).
// -----------------------------

enum class CenterKind : std::uint8_t { Hub = 0, District = 1, Edge = 2, Activity = 3 };

struct Center {
  Point p{};
  CenterKind kind = CenterKind::Activity;
  int id = 0;
};

static bool SnapToRoad(const World& world, const Point& in, int maxR, std::uint32_t seed32, Point& out)
{
  if (!world.inBounds(in.x, in.y)) return false;
  if (world.at(in.x, in.y).overlay == Overlay::Road) {
    out = in;
    return true;
  }

  bool found = false;
  int bestDist = std::numeric_limits<int>::max();
  std::uint32_t bestHash = 0xFFFFFFFFu;

  for (int dy = -maxR; dy <= maxR; ++dy) {
    for (int dx = -maxR; dx <= maxR; ++dx) {
      const int dist = std::abs(dx) + std::abs(dy);
      if (dist > maxR) continue;
      const int x = in.x + dx;
      const int y = in.y + dy;
      if (!world.inBounds(x, y)) continue;
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;

      const std::uint32_t h = HashCoords32(x, y, seed32);
      if (!found || dist < bestDist || (dist == bestDist && h < bestHash)) {
        out = Point{x, y};
        bestDist = dist;
        bestHash = h;
        found = true;
      }
    }
  }
  return found;
}

static void AddCenter(std::vector<Center>& centers, const World& world, const Point& p, CenterKind kind, int id,
                      std::uint32_t seed32)
{
  Point snapped{};
  const std::uint32_t salt = static_cast<std::uint32_t>(kind) * 0x9E3779B1u ^ static_cast<std::uint32_t>(id) * 0x85EBCA6Bu;
  if (!SnapToRoad(world, p, /*maxR=*/12, seed32 ^ salt, snapped)) return;

  for (const Center& c : centers) {
    if (c.p.x == snapped.x && c.p.y == snapped.y) return;
  }
  centers.push_back(Center{snapped, kind, id});
}

static void FillRoadLevelGaps(World& world, int targetLevel, int passes)
{
  const int w = world.width();
  const int h = world.height();
  for (int pass = 0; pass < passes; ++pass) {
    std::vector<Point> toUpgrade;
    toUpgrade.reserve(256);

    for (int y = 1; y < h - 1; ++y) {
      for (int x = 1; x < w - 1; ++x) {
        Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;
        const int cur = ClampRoadLevel(static_cast<int>(t.level));
        if (cur >= targetLevel) continue;

        const int up = (world.at(x, y - 1).overlay == Overlay::Road)
                           ? ClampRoadLevel(static_cast<int>(world.at(x, y - 1).level))
                           : 0;
        const int dn = (world.at(x, y + 1).overlay == Overlay::Road)
                           ? ClampRoadLevel(static_cast<int>(world.at(x, y + 1).level))
                           : 0;
        const int lf = (world.at(x - 1, y).overlay == Overlay::Road)
                           ? ClampRoadLevel(static_cast<int>(world.at(x - 1, y).level))
                           : 0;
        const int rt = (world.at(x + 1, y).overlay == Overlay::Road)
                           ? ClampRoadLevel(static_cast<int>(world.at(x + 1, y).level))
                           : 0;

        const bool verticalGap = (up >= targetLevel && dn >= targetLevel);
        const bool horizontalGap = (lf >= targetLevel && rt >= targetLevel);
        if (verticalGap || horizontalGap) {
          toUpgrade.push_back(Point{x, y});
        }
      }
    }

    if (toUpgrade.empty()) break;

    for (const Point& p : toUpgrade) {
      Tile& t = world.at(p.x, p.y);
      if (t.overlay == Overlay::Road) {
        t.level = static_cast<std::uint8_t>(ClampRoadLevel(targetLevel));
      }
    }
  }
}

static int PairWeight(CenterKind a, CenterKind b)
{
  // Symmetric weights; intentionally coarse.
  if (a > b) std::swap(a, b);

  if (a == CenterKind::Hub && b == CenterKind::Hub) return 7;
  if (a == CenterKind::Hub && b == CenterKind::District) return 6;
  if (a == CenterKind::Hub && b == CenterKind::Edge) return 6;
  if (a == CenterKind::District && b == CenterKind::District) return 4;
  if (a == CenterKind::District && b == CenterKind::Edge) return 5;
  if (a == CenterKind::Edge && b == CenterKind::Edge) return 3;
  return 2; // Activity combos
}

static void UpgradeRoadHierarchyFromCentrality(World& world, const std::vector<P>& hubs, std::uint32_t seed32,
                                               const ProcGenConfig& cfg)
{
  if (!cfg.roadHierarchyEnabled) return;
  if (cfg.roadHierarchyStrength <= 0.0001f) return;

  const int w = world.width();
  const int h = world.height();

  RoadGraph graph = BuildRoadGraph(world);
  if (graph.nodes.empty()) return;

  RoadGraphIndex index = BuildRoadGraphIndex(world, graph);
  RoadGraphWeights weights = BuildRoadGraphWeights(world, graph);

  // Collect centers.
  std::vector<Center> centers;
  centers.reserve(32);

  // Hubs (already roads due to CarveHubGrid).
  for (std::size_t i = 0; i < hubs.size(); ++i) {
    AddCenter(centers, world, Point{hubs[i].x, hubs[i].y}, CenterKind::Hub, static_cast<int>(i), seed32);
  }

  // District centers based on road tiles.
  struct Acc {
    std::int64_t sx = 0;
    std::int64_t sy = 0;
    std::int32_t n = 0;
  };
  std::array<Acc, static_cast<std::size_t>(kDistrictCount)> acc{};

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int d = static_cast<int>(t.district);
      if (d < 0 || d >= kDistrictCount) continue;
      Acc& a = acc[static_cast<std::size_t>(d)];
      a.sx += x;
      a.sy += y;
      a.n += 1;
    }
  }

  for (int d = 0; d < kDistrictCount; ++d) {
    const Acc& a = acc[static_cast<std::size_t>(d)];
    if (a.n <= 0) continue;
    const int cx = static_cast<int>(a.sx / a.n);
    const int cy = static_cast<int>(a.sy / a.n);
    AddCenter(centers, world, Point{cx, cy}, CenterKind::District, d, seed32 ^ 0xD15D1C7u);
  }

  // Edge connectors (search a thin border strip so we don't miss off-by-1 edge roads).
  auto addEdge = [&](int id, int x0, int y0, int x1, int y1) {
    bool found = false;
    Point best{};
    int bestScore = std::numeric_limits<int>::max();
    std::uint32_t bestHash = 0xFFFFFFFFu;

    const int cx = w / 2;
    const int cy = h / 2;

    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        if (!world.inBounds(x, y)) continue;
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Road) continue;

        const int score = std::abs(x - cx) + std::abs(y - cy); // prefer "central" edge exits
        const std::uint32_t hv = HashCoords32(x, y, seed32 ^ 0xED6EED6Eu);

        if (!found || score < bestScore || (score == bestScore && hv < bestHash)) {
          found = true;
          best = Point{x, y};
          bestScore = score;
          bestHash = hv;
        }
      }
    }

    if (found) {
      AddCenter(centers, world, best, CenterKind::Edge, id, seed32 ^ 0xE0E0E0E0u);
    }
  };

  // North, South, West, East strips.
  addEdge(0, 0, 0, w - 1, std::min(1, h - 1));
  addEdge(1, 0, std::max(0, h - 2), w - 1, h - 1);
  addEdge(2, 0, 0, std::min(1, w - 1), h - 1);
  addEdge(3, std::max(0, w - 2), 0, w - 1, h - 1);

  // Deterministic "activity" points scattered over the road network.
  const int minDim = std::min(w, h);
  const float s = std::clamp(cfg.roadHierarchyStrength, 0.0f, 3.0f);
  int targetActivity = std::clamp(minDim / 20, 4, 14);
  targetActivity = static_cast<int>(std::round(static_cast<float>(targetActivity) * std::sqrt(s)));

  struct Cand {
    std::uint32_t h = 0;
    Point p{};
  };
  std::vector<Cand> cands;
  cands.reserve(1024);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const std::uint32_t hv = HashCoords32(x, y, seed32 ^ 0xA11AC71Bu);
      cands.push_back(Cand{hv, Point{x, y}});
    }
  }

  std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.h < b.h; });

  auto farEnough = [&](const Point& p) {
    for (const Center& c : centers) {
      const int md = std::abs(c.p.x - p.x) + std::abs(c.p.y - p.y);
      if (md < 14) return false;
    }
    return true;
  };

  int addedActivity = 0;
  for (const Cand& cand : cands) {
    if (addedActivity >= targetActivity) break;
    if (!farEnough(cand.p)) continue;
    centers.push_back(Center{cand.p, CenterKind::Activity, addedActivity});
    ++addedActivity;
  }

  if (centers.size() < 2) return;

  // Accumulate traversal counts.
  std::vector<int> usage;
  usage.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);

  RoadRouteConfig routeCfg;
  routeCfg.metric = RoadRouteMetric::TravelTime;

  for (std::size_t i = 0; i < centers.size(); ++i) {
    for (std::size_t j = i + 1; j < centers.size(); ++j) {
      const int weight = PairWeight(centers[i].kind, centers[j].kind);
      if (weight <= 0) continue;

      const RoadRouteResult rr = FindRoadRouteAStar(world, graph, index, weights, centers[i].p, centers[j].p, routeCfg);
      if (rr.path.empty()) continue;
      if (rr.path.size() < 2) continue;

      for (const Point& p : rr.path) {
        if (!world.inBounds(p.x, p.y)) continue;
        if (world.at(p.x, p.y).overlay != Overlay::Road) continue;
        usage[Idx(p.x, p.y, w)] += weight;
      }
    }
  }

  // Rank road tiles by usage.
  int roadCount = 0;
  int maxUse = 0;
  std::vector<int> scored;
  scored.reserve(2048);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      ++roadCount;
      const int u = usage[Idx(x, y, w)];
      if (u > 0) scored.push_back(y * w + x);
      maxUse = std::max(maxUse, u);
    }
  }

  if (scored.empty() || maxUse <= 0) return;

  std::sort(scored.begin(), scored.end(), [&](int ia, int ib) {
    const int ua = usage[static_cast<std::size_t>(ia)];
    const int ub = usage[static_cast<std::size_t>(ib)];
    if (ua != ub) return ua > ub;

    const int ax = ia % w, ay = ia / w;
    const int bx = ib % w, by = ib / w;
    const std::uint32_t ha = HashCoords32(ax, ay, seed32 ^ 0xC0FFEE00u);
    const std::uint32_t hb = HashCoords32(bx, by, seed32 ^ 0xC0FFEE00u);
    return ha < hb;
  });

  const int baseHighway = std::clamp(roadCount / 45, 10, 260);
  const int baseAvenue = std::clamp(roadCount / 11, 50, 1100);

  int highwayBudget = static_cast<int>(std::round(static_cast<float>(baseHighway) * s));
  int avenueBudget = static_cast<int>(std::round(static_cast<float>(baseAvenue) * s));
  highwayBudget = std::clamp(highwayBudget, 0, static_cast<int>(scored.size()));
  avenueBudget = std::clamp(avenueBudget, 0, static_cast<int>(scored.size()));

  const int minAvenueUse = std::max(3, maxUse / 6);
  const int minHighwayUse = std::max(8, maxUse / 3);

  // Upgrade to highways (level 3).
  int hiUp = 0;
  for (int idxFlat : scored) {
    if (hiUp >= highwayBudget) break;
    const int u = usage[static_cast<std::size_t>(idxFlat)];
    if (u < minHighwayUse) break; // sorted; the rest will be lower

    const int x = idxFlat % w;
    const int y = idxFlat / w;
    Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) continue;

    const int cur = ClampRoadLevel(static_cast<int>(t.level));
    if (cur < 3) {
      t.level = static_cast<std::uint8_t>(3);
      ++hiUp;
    }
  }

  // Upgrade to avenues (level 2).
  int avUp = 0;
  for (int idxFlat : scored) {
    if (avUp >= avenueBudget) break;
    const int u = usage[static_cast<std::size_t>(idxFlat)];
    if (u < minAvenueUse) break;

    const int x = idxFlat % w;
    const int y = idxFlat / w;
    Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) continue;

    const int cur = ClampRoadLevel(static_cast<int>(t.level));
    if (cur < 2) {
      t.level = static_cast<std::uint8_t>(2);
      ++avUp;
    }
  }

  // Fill single-tile gaps so arterial lines don't look broken.
  FillRoadLevelGaps(world, 3, /*passes=*/2);
  FillRoadLevelGaps(world, 2, /*passes=*/2);
}



static void ApplyProcGenDistrictingMode(World& world, const ProcGenConfig& cfg)
{
  switch (cfg.districtingMode) {
  case ProcGenDistrictingMode::Voronoi:
    // Legacy behavior already assigned earlier.
    return;

  case ProcGenDistrictingMode::RoadFlow: {
    // Seed + partition from the road network. Travel-time weighting uses road class
    // so highways "reach" farther than streets.
    AutoDistrictConfig dc{};
    dc.districts = kDistrictCount;
    dc.requireOutsideConnection = false;
    dc.useTravelTime = true;
    dc.fillAllTiles = true;
    dc.includeWater = true;
    AutoAssignDistricts(world, dc);
    return;
  }

  case ProcGenDistrictingMode::BlockGraph: {
    // Neighborhood-style districts based on contiguous CityBlocks.
    BlockDistrictConfig bc{};
    bc.districts = kDistrictCount;
    bc.fillRoadTiles = true;
    bc.includeWater = true;
    AssignDistrictsByBlocks(world, bc, nullptr);
    return;
  }

  default:
    return;
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

  // Optional macro shaping (presets). Classic is a no-op.
  ApplyTerrainPreset(heights, width, height, seed32, cfg);

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

  // Connect hubs using the selected macro road layout.
  switch (cfg.roadLayout) {
  case ProcGenRoadLayout::Grid:
    CarveHubConnectionsGrid(world, rng, hubPts, seed32, cfg);
    break;
  case ProcGenRoadLayout::Radial:
    CarveHubConnectionsRadial(world, rng, hubPts, seed32, cfg);
    break;
  case ProcGenRoadLayout::SpaceColonization:
    CarveHubConnectionsSpaceColonization(world, rng, hubPts, seed32, cfg);
    break;
  case ProcGenRoadLayout::Organic:
  default:
    CarveHubConnectionsOrganic(world, rng, hubPts, seed32, cfg);
    break;
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
    CarveRoadCurvy(world, rng, hubPts[static_cast<std::size_t>(bestHub)], bestEdge, lvl, /*allowBridges=*/true, seed32 ^ HashCoords32(bestHub, lvl, 0xED9EED6Eu));
  }

  // Optional: carve a highway-ish beltway around the hub cluster.
  // For grid layouts we skip this so the macro structure reads more "planned".
  if (cfg.roadLayout != ProcGenRoadLayout::Grid) {
    CarveBeltwayIfUseful(world, rng, hubPts, seed32 ^ 0xB17BEEFu);
  }

  // Subdivide large blocks with a small hierarchical street network before zoning.
  CarveInternalStreets(world, hubPts, seed32 ^ 0x1337C0DEu);
  // Opportunistically stitch disconnected local networks across narrow water gaps.
  StitchNarrowWaterBridges(world, hubPts, seed32 ^ 0xB16B00B5u);

  // v11: post-process the generated road network to create a clearer
  // hierarchy of streets/avenues/highways based on sampled road-graph centrality.
  UpgradeRoadHierarchyFromCentrality(world, hubPts, seed32 ^ 0x51A71D00u, cfg);

  // v12: optionally reassign districts based on the generated road network.
  //
  // This produces cleaner, street-following district boundaries and gives the
  // zoning pass more coherent "neighborhood" inputs.
  ApplyProcGenDistrictingMode(world, cfg);

  // Place zones and parks using block-aware inward growth.
  PlaceZonesAndParksFromBlocks(world, hubPts, seed32 ^ 0xD15EA5E5u, cfg);

  // Safety: rebuild masks in one pass.
  world.recomputeRoadMasks();

  return world;
}

} // namespace isocity
