#include "isocity/ProcGen.hpp"

#include "isocity/Noise.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <vector>

namespace isocity {

namespace {

struct P {
  int x = 0;
  int y = 0;
};

inline int Sign(int v) { return (v > 0) - (v < 0); }

P RandomLand(RNG& rng, const World& world)
{
  const int w = world.width();
  const int h = world.height();

  for (int tries = 0; tries < 10000; ++tries) {
    const int x = rng.rangeInt(0, w - 1);
    const int y = rng.rangeInt(0, h - 1);
    if (world.isBuildable(x, y)) return P{x, y};
  }

  return P{w / 2, h / 2};
}

P ClosestBuildableEdge(RNG& rng, const World& world, P from)
{
  const int w = world.width();
  const int h = world.height();

  // Scan the border for a buildable tile nearest to the given point.
  // This gives the world at least one "outside" connection for road networks.
  int bestDist = 1'000'000;
  P best = from;

  auto consider = [&](int x, int y) {
    if (!world.inBounds(x, y)) return;
    if (!world.isBuildable(x, y)) return;
    const int d = std::abs(x - from.x) + std::abs(y - from.y);
    if (d < bestDist) {
      bestDist = d;
      best = P{x, y};
    } else if (d == bestDist && rng.chance(0.25f)) {
      // Small random tie-break for variety (still deterministic for a given seed).
      best = P{x, y};
    }
  };

  // Top/bottom edges
  for (int x = 0; x < w; ++x) {
    consider(x, 0);
    if (h > 1) consider(x, h - 1);
  }
  // Left/right edges (skip corners to avoid duplicates)
  for (int y = 1; y < h - 1; ++y) {
    consider(0, y);
    if (w > 1) consider(w - 1, y);
  }

  return best;
}

void CarveRoadWiggle(World& world, RNG& rng, P a, P b)
{
  P p = a;
  const int maxSteps = world.width() * world.height() * 2;

  world.setRoad(p.x, p.y);

  for (int step = 0; step < maxSteps; ++step) {
    if (p.x == b.x && p.y == b.y) break;

    const int dx = b.x - p.x;
    const int dy = b.y - p.y;

    int sx = Sign(dx);
    int sy = Sign(dy);

    // Sometimes introduce slight "wiggle" for organic roads.
    if (rng.chance(0.08f)) {
      if (rng.chance(0.5f))
        sx = (sx == 0) ? (rng.chance(0.5f) ? 1 : -1) : sx;
      else
        sy = (sy == 0) ? (rng.chance(0.5f) ? 1 : -1) : sy;
    }

    // Choose whether to step in x or y direction.
    bool stepX = false;
    if (sx != 0 && sy != 0) {
      // Weight choice by remaining distance (bias towards larger distance axis).
      const int adx = std::abs(dx);
      const int ady = std::abs(dy);
      const float t = static_cast<float>(adx) / static_cast<float>(adx + ady);
      stepX = rng.nextF01() < t;
    } else {
      stepX = (sx != 0);
    }

    auto tryStep = [&](int nx, int ny) -> bool {
      if (!world.inBounds(nx, ny)) return false;
      if (!world.isBuildable(nx, ny)) return false; // avoid water
      p = P{nx, ny};
      world.setRoad(p.x, p.y);
      return true;
    };

    // Preferred step.
    if (stepX) {
      if (tryStep(p.x + sx, p.y)) continue;
      if (sy != 0 && tryStep(p.x, p.y + sy)) continue;
    } else {
      if (tryStep(p.x, p.y + sy)) continue;
      if (sx != 0 && tryStep(p.x + sx, p.y)) continue;
    }

    // If blocked by water/edges, try random adjacent land.
    constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    bool moved = false;
    for (int i = 0; i < 4; ++i) {
      const int k = rng.rangeInt(0, 3);
      const int nx = p.x + dirs[k][0];
      const int ny = p.y + dirs[k][1];
      if (tryStep(nx, ny)) {
        moved = true;
        break;
      }
    }

    if (!moved) break; // give up
  }
}

void CarveRoad(World& world, RNG& rng, P a, P b)
{
  // Prefer an A* route over buildable tiles so roads reliably go *around* water
  // instead of randomly getting stuck.
  std::vector<Point> path;
  if (FindLandPathAStar(world, Point{a.x, a.y}, Point{b.x, b.y}, path)) {
    for (const Point& p : path) {
      world.setRoad(p.x, p.y);
    }
    return;
  }

  // Fallback: legacy "wiggly" carver (kept for robustness if land is disconnected).
  CarveRoadWiggle(world, rng, a, b);
}

} // namespace

World GenerateWorld(int width, int height, std::uint64_t seed, const ProcGenConfig& cfg)
{
  World world(width, height, seed);
  RNG rng(seed);

  // --- Terrain ---
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Tile& t = world.at(x, y);

      // FBm noise in [0,1]
      const float nx = (static_cast<float>(x) - width * 0.5f) * cfg.terrainScale;
      const float ny = (static_cast<float>(y) - height * 0.5f) * cfg.terrainScale;
      const float e = FBm2D(nx, ny, static_cast<std::uint32_t>(seed), 6, 2.0f, 0.5f);

      t.height = e;
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;

      // Stable per-tile variation
      t.variation = static_cast<std::uint8_t>(HashCoords32(x, y, static_cast<std::uint32_t>(seed)) & 0xFFu);

      if (e < cfg.waterLevel) {
        t.terrain = Terrain::Water;
      } else if (e < cfg.sandLevel) {
        t.terrain = Terrain::Sand;
      } else {
        t.terrain = Terrain::Grass;
      }
    }
  }

  // --- Pick hubs ("town centers") ---
  const int hubs = std::max(2, cfg.hubs);
  std::vector<P> hubPts;
  hubPts.reserve(static_cast<std::size_t>(hubs));

  for (int i = 0; i < hubs; ++i) {
    hubPts.push_back(RandomLand(rng, world));
  }

  // --- Connect hubs with roads ---
  // Ensure at least a chain connecting all hubs.
  for (int i = 1; i < hubs; ++i) {
    CarveRoad(world, rng, hubPts[static_cast<std::size_t>(i - 1)], hubPts[static_cast<std::size_t>(i)]);
  }

  // Add some extra random connections for loops.
  for (int i = 0; i < cfg.extraConnections; ++i) {
    const int a = rng.rangeInt(0, hubs - 1);
    const int b = rng.rangeInt(0, hubs - 1);
    if (a == b) continue;
    CarveRoad(world, rng, hubPts[static_cast<std::size_t>(a)], hubPts[static_cast<std::size_t>(b)]);
  }

  // --- Ensure at least one road reaches the map edge ("outside connection") ---
  // This supports the sim rule where disconnected road networks don't provide access for zones.
  if (!hubPts.empty()) {
    const P edge = ClosestBuildableEdge(rng, world, hubPts[0]);
    CarveRoad(world, rng, hubPts[0], edge);
  }

  // --- Zone placement along roads ---
  constexpr int dirs[4][2] = {
      {1, 0},
      {-1, 0},
      {0, 1},
      {0, -1},
  };

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;

      for (const auto& d : dirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (!world.inBounds(nx, ny)) continue;
        if (!world.isEmptyLand(nx, ny)) continue;

        const float r = rng.nextF01();
        if (r < cfg.parkChance) {
          world.setOverlay(Overlay::Park, nx, ny);
          continue;
        }
        if (r < cfg.parkChance + cfg.zoneChance) {
          // Zone type weights
          const float z = rng.nextF01();
          Overlay zone = Overlay::Residential;
          if (z < 0.65f)
            zone = Overlay::Residential;
          else if (z < 0.85f)
            zone = Overlay::Commercial;
          else
            zone = Overlay::Industrial;

          Tile& nt = world.at(nx, ny);
          nt.overlay = zone;
          nt.level = 1;
          nt.occupants = 0;

          // Some initial variety in levels.
          if (rng.chance(0.12f)) nt.level = 2;
          if (rng.chance(0.04f)) nt.level = 3;
        }
      }
    }
  }

  return world;
}

} // namespace isocity
