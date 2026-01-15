#include "isocity/PovProcedural.hpp"

#include "isocity/Random.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace isocity {
namespace {

inline bool InBounds(const World& world, int x, int y) {
  return x >= 0 && y >= 0 && x < world.width() && y < world.height();
}

inline bool IsRoad(const World& world, int x, int y) {
  if (!InBounds(world, x, y)) {
    return false;
  }
  return world.at(x, y).overlay == Overlay::Road;
}

int RoadDegree4(const World& world, Point p) {
  int deg = 0;
  deg += IsRoad(world, p.x + 1, p.y) ? 1 : 0;
  deg += IsRoad(world, p.x - 1, p.y) ? 1 : 0;
  deg += IsRoad(world, p.x, p.y + 1) ? 1 : 0;
  deg += IsRoad(world, p.x, p.y - 1) ? 1 : 0;
  return deg;
}

float ScenicScore8(const World& world, Point p) {
  // Simple local heuristic:
  //  - nearby water is highly scenic
  //  - parks and civic buildings are mildly scenic
  //  - higher local elevation variation is scenic
  float s = 0.0f;

  const Tile& c = world.at(p.x, p.y);
  const int h0 = static_cast<int>(c.height);

  int maxDh = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int x = p.x + dx;
      const int y = p.y + dy;
      if (!InBounds(world, x, y)) {
        continue;
      }

      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) {
        s += 1.6f;
      }
      if (t.overlay == Overlay::Park) {
        s += 1.0f;
      }
      if (t.overlay == Overlay::Market || t.overlay == Overlay::School || t.overlay == Overlay::Hospital ||
          t.overlay == Overlay::Stadium) {
        s += 0.6f;
      }

      const int dh = std::abs(static_cast<int>(t.height) - h0);
      maxDh = std::max(maxDh, dh);
    }
  }

  s += std::min(1.0f, static_cast<float>(maxDh) / 8.0f) * 0.6f;

  // Road level: higher-level roads tend to have more dramatic vistas (bridges, avenues, etc).
  if (c.overlay == Overlay::Road) {
    s += 0.2f * static_cast<float>(std::max(0, static_cast<int>(c.level) - 1));
  }

  return s;
}

Point RandomRoadTile(const World& world, RNG& rng, int maxTries = 4096) {
  Point p{world.width() / 2, world.height() / 2};
  for (int i = 0; i < maxTries; ++i) {
    const int x = static_cast<int>(rng.nextU32() % static_cast<std::uint32_t>(std::max(1, world.width())));
    const int y = static_cast<int>(rng.nextU32() % static_cast<std::uint32_t>(std::max(1, world.height())));
    if (IsRoad(world, x, y)) {
      return Point{x, y};
    }
  }

  // Fallback: brute scan.
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      if (IsRoad(world, x, y)) {
        return Point{x, y};
      }
    }
  }
  return p;
}

// Weighted random selection.
int ChooseIndexWeighted(const std::vector<float>& weights, RNG& rng) {
  float sum = 0.0f;
  for (float w : weights) {
    sum += std::max(0.0f, w);
  }
  if (sum <= 0.0f) {
    return 0;
  }

  float r = rng.uniform01() * sum;
  for (int i = 0; i < static_cast<int>(weights.size()); ++i) {
    r -= std::max(0.0f, weights[i]);
    if (r <= 0.0f) {
      return i;
    }
  }
  return static_cast<int>(weights.size() - 1);
}

} // namespace

bool FindNearestRoadTile(const World& world, Point hint, int radius, Point& outRoad) {
  if (world.width() <= 0 || world.height() <= 0) {
    return false;
  }

  // Clamp hint into the world.
  hint.x = std::max(0, std::min(world.width() - 1, hint.x));
  hint.y = std::max(0, std::min(world.height() - 1, hint.y));

  if (IsRoad(world, hint.x, hint.y)) {
    outRoad = hint;
    return true;
  }

  int bestDist = std::numeric_limits<int>::max();
  Point best = hint;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = hint.x + dx;
      const int y = hint.y + dy;
      if (!IsRoad(world, x, y)) {
        continue;
      }
      const int d = std::abs(dx) + std::abs(dy);
      if (d < bestDist) {
        bestDist = d;
        best = Point{x, y};
        if (bestDist <= 1) {
          outRoad = best;
          return true;
        }
      }
    }
  }

  if (bestDist != std::numeric_limits<int>::max()) {
    outRoad = best;
    return true;
  }
  return false;
}

std::vector<Point> GeneratePovRoamPath(const World& world, Point startHint, const PovRoamConfig& cfg,
                                      std::uint32_t seed, std::string* outDebug) {
  std::vector<Point> path;
  if (world.width() <= 0 || world.height() <= 0 || cfg.length <= 1) {
    return path;
  }

  RNG rng(seed);

  Point start = startHint;
  if (!FindNearestRoadTile(world, startHint, std::max(1, cfg.findRoadRadius), start)) {
    start = RandomRoadTile(world, rng);
  }

  if (!IsRoad(world, start.x, start.y)) {
    // No roads at all.
    return path;
  }

  path.reserve(static_cast<std::size_t>(cfg.length));
  path.push_back(start);

  Point prev = start;
  Point cur = start;

  // Previous direction (dx,dy) in {-1,0,1}.
  Point prevDir{0, 0};

  int restarts = 0;
  int failIters = 0;

  const auto considerRestart = [&]() {
    ++restarts;
    cur = RandomRoadTile(world, rng);
    prev = cur;
    prevDir = Point{0, 0};
    path.push_back(cur);
    failIters = 0;
  };

  while (static_cast<int>(path.size()) < cfg.length) {
    // Collect candidates.
    struct Cand {
      Point p;
      Point dir;
      float scenic = 0.0f;
      int deg = 0;
    };

    std::vector<Cand> cand;
    cand.reserve(4);

    const Point n4[4] = {Point{cur.x + 1, cur.y}, Point{cur.x - 1, cur.y}, Point{cur.x, cur.y + 1},
                         Point{cur.x, cur.y - 1}};
    for (const Point n : n4) {
      if (!IsRoad(world, n.x, n.y)) {
        continue;
      }
      Cand c;
      c.p = n;
      c.dir = Point{n.x - cur.x, n.y - cur.y};
      c.scenic = ScenicScore8(world, n);
      c.deg = RoadDegree4(world, n);
      cand.push_back(c);
    }

    if (cand.empty()) {
      considerRestart();
      continue;
    }

    // Score candidates.
    std::vector<float> weights;
    weights.reserve(cand.size());

    for (const Cand& c : cand) {
      float score = 0.0f;

      // Don't instantly U-turn unless it's literally the only way.
      const bool isBack = (c.p.x == prev.x && c.p.y == prev.y);
      if (isBack) {
        score -= 4.0f;
      }

      // Prefer continuing direction.
      if (!(prevDir.x == 0 && prevDir.y == 0)) {
        const bool same = (c.dir.x == prevDir.x && c.dir.y == prevDir.y);
        const bool opposite = (c.dir.x == -prevDir.x && c.dir.y == -prevDir.y);
        if (same) {
          score += 3.0f * cfg.straightBias;
        } else if (opposite) {
          score -= 3.0f * cfg.straightBias;
        } else {
          // perpendicular turn
          score += 1.0f * (1.0f - cfg.straightBias);
        }
      }

      // Scenic bias.
      score += c.scenic * (2.0f * cfg.scenicBias);

      // Dead-end avoidance.
      if (cfg.avoidDeadEnds && c.deg <= 1) {
        score -= 3.0f;
      }

      // Mild noise so we don't get stuck in deterministic patterns.
      score += (rng.uniform01() - 0.5f) * 0.35f;

      // Convert score -> weight. Clamp to avoid overflow.
      const float w = std::exp(std::max(-8.0f, std::min(8.0f, score)));
      weights.push_back(w);
    }

    const int idx = ChooseIndexWeighted(weights, rng);
    const Point next = cand[static_cast<std::size_t>(idx)].p;
    const Point nextDir = cand[static_cast<std::size_t>(idx)].dir;

    // Track failures: if we keep ping-ponging, restart.
    if (next.x == prev.x && next.y == prev.y) {
      ++failIters;
    } else {
      failIters = 0;
    }

    prev = cur;
    cur = next;
    prevDir = nextDir;
    path.push_back(cur);

    if (failIters > cfg.maxFailIters) {
      considerRestart();
    }
  }

  if (outDebug) {
    std::ostringstream oss;
    oss << "RoamPath: tiles=" << path.size() << " start=(" << start.x << "," << start.y << ")"
        << " seed=" << seed << " restarts=" << restarts;
    *outDebug = oss.str();
  }

  return path;
}

} // namespace isocity
