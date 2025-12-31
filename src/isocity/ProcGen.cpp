#include "isocity/ProcGen.hpp"

#include "isocity/Noise.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <vector>

namespace isocity {

namespace {

struct P {
  int x = 0;
  int y = 0;
};

static inline float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }

static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

// Simple value noise based on HashCoords32.
static float noise2D(int xi, int yi, std::uint32_t seed)
{
  const std::uint32_t h = HashCoords32(xi, yi, seed);
  // Use 24 bits for a stable [0,1) float.
  return static_cast<float>((h >> 8) & 0x00FFFFFFu) / 16777216.0f;
}

static float valueNoise2D(float x, float y, std::uint32_t seed)
{
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const float sx = fade(x - static_cast<float>(x0));
  const float sy = fade(y - static_cast<float>(y0));

  const float n00 = noise2D(x0, y0, seed);
  const float n10 = noise2D(x1, y0, seed);
  const float n01 = noise2D(x0, y1, seed);
  const float n11 = noise2D(x1, y1, seed);

  const float ix0 = lerp(n00, n10, sx);
  const float ix1 = lerp(n01, n11, sx);
  return lerp(ix0, ix1, sy);
}

static float fbm(float x, float y, std::uint32_t seed, int octaves)
{
  float value = 0.0f;
  float amplitude = 0.5f;
  float frequency = 1.0f;

  for (int i = 0; i < octaves; ++i) {
    value += amplitude * valueNoise2D(x * frequency, y * frequency, seed + 1337u * static_cast<std::uint32_t>(i));
    frequency *= 2.0f;
    amplitude *= 0.5f;
  }

  return value;
}

static P RandomLand(const World& world, RNG& rng)
{
  for (int i = 0; i < 10000; ++i) {
    const int x = rng.rangeInt(0, world.width() - 1);
    const int y = rng.rangeInt(0, world.height() - 1);
    if (world.isBuildable(x, y)) {
      return {x, y};
    }
  }
  // Fallback.
  return {world.width() / 2, world.height() / 2};
}

static void SetRoadWithLevel(World& world, int x, int y, int level)
{
  if (!world.inBounds(x, y)) {
    return;
  }
  world.setRoad(x, y);

  // setRoad won't place on water; only set level when a road actually exists.
  Tile& t = world.at(x, y);
  if (t.overlay == Overlay::Road) {
    t.level = static_cast<std::uint8_t>(std::clamp(level, 1, 3));
  }
}

static void CarveRoadWiggle(World& world, RNG& rng, P a, P b, int level)
{
  int x = a.x;
  int y = a.y;
  SetRoadWithLevel(world, x, y, level);

  const int wiggly = (std::abs(x - b.x) + std::abs(y - b.y)) / 3;

  while (x != b.x || y != b.y) {
    if (rng.chance(0.5f)) {
      if (x < b.x) {
        x++;
      } else if (x > b.x) {
        x--;
      } else {
        y += (y < b.y) ? 1 : -1;
      }
    } else {
      if (y < b.y) {
        y++;
      } else if (y > b.y) {
        y--;
      } else {
        x += (x < b.x) ? 1 : -1;
      }
    }

    // Occasional wiggle.
    if (wiggly > 0 && rng.chance(0.2f)) {
      if (rng.chance(0.5f)) {
        x += rng.chance(0.5f) ? 1 : -1;
      } else {
        y += rng.chance(0.5f) ? 1 : -1;
      }
      x = std::clamp(x, 0, world.width() - 1);
      y = std::clamp(y, 0, world.height() - 1);
    }

    SetRoadWithLevel(world, x, y, level);

    // Ensure connectivity through wiggles.
    if (wiggly > 0 && rng.chance(0.1f)) {
      SetRoadWithLevel(world, x + 1, y, level);
      SetRoadWithLevel(world, x - 1, y, level);
      SetRoadWithLevel(world, x, y + 1, level);
      SetRoadWithLevel(world, x, y - 1, level);
    }
  }
}

static void CarveRoad(World& world, RNG& rng, P a, P b, int level)
{
  std::vector<Point> path;
  if (!FindRoadBuildPath(world, Point{a.x, a.y}, Point{b.x, b.y}, path)) {
    // Fallback to a simple carve if pathfinding fails.
    CarveRoadWiggle(world, rng, a, b, level);
    return;
  }

  for (const Point& p : path) {
    SetRoadWithLevel(world, p.x, p.y, level);
  }
}

static int ChooseHubConnectionLevel(const World& world, P a, P b)
{
  const int dist = std::abs(a.x - b.x) + std::abs(a.y - b.y);
  const int scale = std::max(world.width(), world.height());

  // Long distances get highways; medium get avenues.
  if (dist > (scale * 3) / 4) {
    return 3;
  }
  if (dist > scale / 3) {
    return 2;
  }
  return 2;
}

static void CarveHubGrid(World& world, RNG& rng, P hub)
{
  // A small arterial cross at the center + surrounding local streets.
  const int minDim = std::min(world.width(), world.height());
  const int radius = std::max(6, minDim / 6);
  const int spacing = std::clamp(rng.rangeInt(4, 7), 3, 8);

  auto carveH = [&](int y, int level) {
    if (y < 0 || y >= world.height()) {
      return;
    }
    for (int x = hub.x - radius; x <= hub.x + radius; ++x) {
      if (!world.inBounds(x, y)) {
        continue;
      }
      if (!world.isBuildable(x, y)) {
        continue;
      }
      SetRoadWithLevel(world, x, y, level);
    }
  };

  auto carveV = [&](int x, int level) {
    if (x < 0 || x >= world.width()) {
      return;
    }
    for (int y = hub.y - radius; y <= hub.y + radius; ++y) {
      if (!world.inBounds(x, y)) {
        continue;
      }
      if (!world.isBuildable(x, y)) {
        continue;
      }
      SetRoadWithLevel(world, x, y, level);
    }
  };

  // Arterial cross.
  carveH(hub.y, 2);
  carveV(hub.x, 2);

  // Local streets (some lines may be skipped to add variety).
  for (int off = spacing; off <= radius; off += spacing) {
    if (rng.chance(0.85f)) {
      carveH(hub.y + off, 1);
    }
    if (rng.chance(0.85f)) {
      carveH(hub.y - off, 1);
    }
    if (rng.chance(0.85f)) {
      carveV(hub.x + off, 1);
    }
    if (rng.chance(0.85f)) {
      carveV(hub.x - off, 1);
    }
  }
}

static int NearestHubDist(const std::vector<P>& hubs, int x, int y)
{
  int best = std::numeric_limits<int>::max();
  for (const P& h : hubs) {
    const int d = std::abs(x - h.x) + std::abs(y - h.y);
    best = std::min(best, d);
  }
  return best == std::numeric_limits<int>::max() ? 0 : best;
}

static Overlay PickZoneType(RNG& rng, float d01, int roadLevel)
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
  const float r = rng.nextF01() * sum;
  if (r < resW) {
    return Overlay::Residential;
  }
  if (r < resW + comW) {
    return Overlay::Commercial;
  }
  return Overlay::Industrial;
}

static int PickZoneLevel(RNG& rng, float d01, int roadLevel)
{
  // Default low density.
  int lvl = 1;

  // Near hubs we allow higher density more often.
  if (d01 < 0.25f && rng.chance(0.35f)) {
    lvl = 2;
  }
  if (d01 < 0.12f && rng.chance(0.18f)) {
    lvl = 3;
  }

  // Higher class roads allow higher density, too.
  if (roadLevel >= 2 && rng.chance(0.25f)) {
    lvl = std::max(lvl, 2);
  }
  if (roadLevel >= 3 && rng.chance(0.18f)) {
    lvl = 3;
  }

  return std::clamp(lvl, 1, 3);
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

        const std::uint32_t jitter = HashCoords32(x, y, seed32 ^ (0x9E3779B9u * static_cast<std::uint32_t>(i + 1))) & 0xFFu;
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
    CarveRoad(world, rng, hubPts[a], hubPts[b], 2);
  }

  // Ensure at least one connection to the map edge (outside connection).
  const P edgePt = FindClosestEdgeLand(world, hubPts[0]);
  CarveRoad(world, rng, hubPts[0], edgePt, 2);

  // Place zones and parks along roads.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Tile& rt = world.at(x, y);
      if (rt.overlay != Overlay::Road) {
        continue;
      }

      const int roadLevel = static_cast<int>(rt.level);

      const int dx[4] = {1, -1, 0, 0};
      const int dy[4] = {0, 0, 1, -1};

      for (int dir = 0; dir < 4; ++dir) {
        const int nx = x + dx[dir];
        const int ny = y + dy[dir];
        if (!world.inBounds(nx, ny)) {
          continue;
        }

        Tile& t = world.at(nx, ny);
        if (t.overlay != Overlay::None) {
          continue;
        }
        if (t.terrain == Terrain::Water) {
          continue;
        }

        // Deterministic per-tile chance gate.
        const float chance = static_cast<float>((HashCoords32(nx, ny, seed32 ^ 0xDEADBEEFu) >> 8) & 0x00FFFFFFu) / 16777216.0f;
        if (chance >= cfg.zoneChance) {
          continue;
        }

        const int hubDist = NearestHubDist(hubPts, nx, ny);
        const float d01 = std::clamp(static_cast<float>(hubDist) / static_cast<float>(width + height), 0.0f, 1.0f);

        // Park chance: a touch higher near hubs, but reduced along highways.
        float parkChance = cfg.parkChance;
        if (d01 < 0.25f) {
          parkChance += 0.05f;
        }
        if (roadLevel >= 3) {
          parkChance *= 0.60f;
        }
        parkChance = std::clamp(parkChance, 0.0f, 1.0f);

        if (rng.chance(parkChance)) {
          t.overlay = Overlay::Park;
          t.level = 1;
          t.occupants = 0;
          continue;
        }

        const Overlay zone = PickZoneType(rng, d01, roadLevel);
        const int zLvl = PickZoneLevel(rng, d01, roadLevel);

        t.overlay = zone;
        t.level = static_cast<std::uint8_t>(zLvl);
        t.occupants = 1;
      }
    }
  }

  // Safety: rebuild masks in one pass.
  world.recomputeRoadMasks();

  return world;
}

} // namespace isocity
