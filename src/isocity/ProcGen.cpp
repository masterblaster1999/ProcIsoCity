#include "isocity/ProcGen.hpp"

#include "isocity/CityBlocks.hpp"
#include "isocity/Noise.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

static float fbm(float x, float y, std::uint32_t seed, int octaves)
{
  float total = 0.0f;
  float amp = 0.5f;
  float freq = 1.0f;

  for (int i = 0; i < octaves; ++i) {
    total += noise2D(x * freq, y * freq, seed + static_cast<std::uint32_t>(i) * 1013u) * amp;
    freq *= 2.0f;
    amp *= 0.5f;
  }

  return total;
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

static void SetRoadWithLevel(World& world, int x, int y, int level)
{
  if (!world.inBounds(x, y)) {
    return;
  }
  if (!world.isBuildable(x, y)) {
    // We don't auto-place bridges as part of the procedural street network.
    return;
  }

  world.setRoad(x, y);
  world.at(x, y).level = static_cast<std::uint8_t>(ClampRoadLevel(level));
}

static void CarveRoadWiggle(World& world, RNG& rng, P a, P b, int level)
{
  int x = a.x;
  int y = a.y;

  const int maxSteps = world.width() * world.height();
  for (int steps = 0; steps < maxSteps; ++steps) {
    SetRoadWithLevel(world, x, y, level);
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

static void CarveRoad(World& world, RNG& rng, P a, P b, int level)
{
  // Use the general road-build path planner (keeps it consistent with tooling).
  std::vector<Point> path;
  RoadBuildPathConfig cfg;
  cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;
  cfg.targetLevel = ClampRoadLevel(level);
  cfg.allowBridges = false;

  const Point start{a.x, a.y};
  const Point goal{b.x, b.y};

  if (FindRoadBuildPath(world, start, goal, path, nullptr, cfg) && !path.empty()) {
    for (const Point& p : path) {
      SetRoadWithLevel(world, p.x, p.y, level);
    }
    return;
  }

  // Fallback: a simple wiggly Manhattan carve.
  CarveRoadWiggle(world, rng, a, b, level);
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
        SetRoadWithLevel(world, x, y, 2);
        continue;
      }

      // Some secondary streets.
      if ((dx % 3 == 0 || dy % 3 == 0) && rng.chance(0.55f)) {
        SetRoadWithLevel(world, x, y, 1);
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

// -----------------------------
// Zone selection (deterministic, uses hashed floats rather than RNG state)
// -----------------------------

static Overlay PickZoneTypeDet(float d01, int roadLevel, float r01)
{
  // Base weights: nearer hubs favor residential/commerce; farther favors industry.
  float resW = 0.65f * (1.0f - d01) + 0.10f;
  float comW = 0.20f * (1.0f - d01) + 0.10f;
  float indW = 0.10f + 0.60f * d01;

  // Higher-class roads skew toward commerce/industry.
  if (roadLevel >= 2) {
    resW *= 0.80f;
    comW *= 1.20f;
    indW *= 1.15f;
  }
  if (roadLevel >= 3) {
    resW *= 0.60f;
    comW *= 1.25f;
    indW *= 1.35f;
  }

  const float sum = resW + comW + indW;
  if (sum <= 0.0f) {
    return Overlay::Residential;
  }

  const float r = Clamp01(r01) * sum;
  if (r < resW) {
    return Overlay::Residential;
  }
  if (r < resW + comW) {
    return Overlay::Commercial;
  }
  return Overlay::Industrial;
}

static int PickZoneLevelDet(float d01, int roadLevel, float r0, float r1, float r2, float r3)
{
  // Default low density.
  int lvl = 1;

  // Near hubs we allow higher density more often.
  if (d01 < 0.25f && r0 < 0.35f) {
    lvl = 2;
  }
  if (d01 < 0.12f && r1 < 0.18f) {
    lvl = 3;
  }

  // Higher class roads allow higher density, too.
  if (roadLevel >= 2 && r2 < 0.25f) {
    lvl = std::max(lvl, 2);
  }
  if (roadLevel >= 3 && r3 < 0.18f) {
    lvl = 3;
  }

  return std::clamp(lvl, 1, 3);
}

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

static void CarveInternalStreets(World& world, const CityBlocksResult& cb0, std::uint32_t seed32)
{
  if (cb0.blocks.empty()) return;

  std::vector<std::vector<Point>> roadAdj;
  BuildBlockRoadAdj(world, cb0, roadAdj);

  std::vector<Point> path;

  for (const CityBlock& b : cb0.blocks) {
    const int bid = b.id;
    if (bid < 0) continue;
    const auto& adj = roadAdj[static_cast<std::size_t>(bid)];

    // Only subdivide large blocks that are meaningfully bounded by roads.
    if (b.area < 140) continue;
    if (b.roadEdges < 12) continue;
    if (adj.size() < 2) continue;

    // One connector is enough most of the time; huge blocks get a second.
    int connectors = 1;
    if (b.area >= 420 && adj.size() >= 8) connectors = 2;

    for (int c = 0; c < connectors; ++c) {
      const std::uint32_t pick = HashCoords32(bid, 17 + c, seed32 ^ 0xA5A5F00Du);
      const Point p0 = adj[static_cast<std::size_t>(pick % static_cast<std::uint32_t>(adj.size()))];

      auto farthest = [&](Point from) -> Point {
        int bestD = -1;
        std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();
        Point best = from;

        for (const Point& q : adj) {
          const int d = std::abs(q.x - from.x) + std::abs(q.y - from.y);
          const std::uint32_t tie = HashCoords32(q.x, q.y, seed32 ^ 0x1BADB002u ^ static_cast<std::uint32_t>(bid) * 0x9E3779B9u);
          if (d > bestD || (d == bestD && tie < bestTie)) {
            bestD = d;
            bestTie = tie;
            best = q;
          }
        }
        return best;
      };

      const Point p1 = farthest(p0);
      const Point p2 = farthest(p1);
      if (p1.x == p2.x && p1.y == p2.y) continue;

      if (!FindPathInBlock(cb0, bid, p1, p2, path)) {
        continue;
      }

      // Avoid carving trivial stubs.
      if (path.size() < 10) {
        continue;
      }

      // Bigger blocks get slightly higher-capacity internal collectors.
      const int level = (b.area >= 520) ? 2 : 1;

      for (const Point& p : path) {
        SetRoadWithLevel(world, p.x, p.y, level);
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

  std::vector<std::vector<Point>> blockRoadAdj;
  BuildBlockRoadAdj(world, cb, blockRoadAdj);

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

  for (const CityBlock& b : cb.blocks) {
    const int bid = b.id;
    if (bid < 0) continue;

    const auto& roadAdj = blockRoadAdj[static_cast<std::size_t>(bid)];
    if (roadAdj.empty()) {
      continue;
    }

    // --- Seed pass: choose parks/zones only on tiles directly adjacent to roads.
    std::vector<Seed> seeds;
    seeds.reserve(roadAdj.size() / 2u + 8u);

    for (const Point& p : roadAdj) {
      Tile& t = world.at(p.x, p.y);
      if (t.overlay != Overlay::None) {
        continue;
      }

      // Deterministic per-tile chance gate.
      const float gate = TileRand01(p.x, p.y, seed32 ^ 0xDEADBEEFu);
      if (gate >= cfg.zoneChance) {
        continue;
      }

      const int roadLevel = MaxAdjacentRoadLevel(world, p.x, p.y);
      if (roadLevel <= 0) {
        continue;
      }

      const int hubDist = NearestHubDist(hubs, p.x, p.y);
      const float d01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(w + h), 0.0f, 1.0f);

      // Park chance: a touch higher near hubs, but reduced along highways.
      float parkChance = cfg.parkChance;
      if (d01 < 0.25f) {
        parkChance += 0.05f;
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
        continue;
      }

      const float rType = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE01u);
      const Overlay zone = PickZoneTypeDet(d01, roadLevel, rType);

      const float r0 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE02u);
      const float r1 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE03u);
      const float r2 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE04u);
      const float r3 = TileRand01(p.x, p.y, seed32 ^ 0xC0FFEE05u);
      const int zLvl = PickZoneLevelDet(d01, roadLevel, r0, r1, r2, r3);

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

    // --- Growth pass: probabilistically zone tiles inward while maintaining connectivity
    // within each zone type (a zoned tile must touch an existing tile of the same type).
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
      const float n = fbm(x * cfg.terrainScale, y * cfg.terrainScale, seed32, 5);
      heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)] = n * 1.2f - 0.2f;
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

  // Pick hubs.
  std::vector<P> hubPts;
  hubPts.reserve(std::max(1, cfg.hubs));
  const int minDist = 12;

  for (int i = 0; i < cfg.hubs; ++i) {
    P p;
    bool ok = false;

    for (int tries = 0; tries < 1000; ++tries) {
      p = RandomLand(world, rng);
      ok = true;
      for (const P& h : hubPts) {
        const int d = std::abs(p.x - h.x) + std::abs(p.y - h.y);
        if (d < minDist) {
          ok = false;
          break;
        }
      }
      if (ok) {
        break;
      }
    }

    if (ok) {
      hubPts.push_back(p);
    }
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

  // Connect hubs with higher-class roads.
  for (size_t i = 0; i + 1 < hubPts.size(); ++i) {
    const int lvl = ChooseHubConnectionLevel(world, hubPts[i], hubPts[i + 1]);
    CarveRoad(world, rng, hubPts[i], hubPts[i + 1], lvl);
  }

  // Extra random connections to create loops.
  for (int i = 0; i < cfg.extraConnections && hubPts.size() >= 2; ++i) {
    const int a = rng.rangeInt(0, static_cast<int>(hubPts.size()) - 1);
    const int b = rng.rangeInt(0, static_cast<int>(hubPts.size()) - 1);
    if (a == b) {
      continue;
    }
    CarveRoad(world, rng, hubPts[static_cast<std::size_t>(a)], hubPts[static_cast<std::size_t>(b)], 2);
  }

  // Ensure at least one connection to the map edge (outside connection).
  const P edgePt = FindClosestEdgeLand(world, hubPts[0]);
  CarveRoad(world, rng, hubPts[0], edgePt, 2);

  // --- New: subdivide large blocks with internal streets before zoning.
  {
    const CityBlocksResult cb0 = BuildCityBlocks(world);
    CarveInternalStreets(world, cb0, seed32 ^ 0x1337C0DEu);
  }

  // Place zones and parks using block-aware inward growth.
  PlaceZonesAndParksFromBlocks(world, hubPts, seed32 ^ 0xD15EA5E5u, cfg);

  // Safety: rebuild masks in one pass.
  world.recomputeRoadMasks();

  return world;
}

} // namespace isocity
