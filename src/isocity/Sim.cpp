#include "isocity/Sim.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

namespace isocity {

namespace {
inline int HousingForLevel(int level) { return 10 * level; }
inline int JobsCommercialForLevel(int level) { return 8 * level; }
inline int JobsIndustrialForLevel(int level) { return 12 * level; }

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

struct ScanResult {
  int housingCap = 0;
  int jobsCap = 0;
  int roads = 0;
  int parks = 0;
  int zoneTiles = 0;
  int population = 0;
};

ScanResult ScanWorld(const World& world)
{
  ScanResult r;

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);

      if (t.overlay == Overlay::Road) r.roads++;
      if (t.overlay == Overlay::Park) r.parks++;

      if (t.overlay == Overlay::Residential) {
        r.zoneTiles++;
        r.housingCap += HousingForLevel(t.level);
        r.population += static_cast<int>(t.occupants);
      } else if (t.overlay == Overlay::Commercial) {
        r.zoneTiles++;
        r.jobsCap += JobsCommercialForLevel(t.level);
      } else if (t.overlay == Overlay::Industrial) {
        r.zoneTiles++;
        r.jobsCap += JobsIndustrialForLevel(t.level);
      }
    }
  }

  return r;
}

// Compute which road tiles are connected to the map border ("outside connection").
//
// We treat the map edge as the entry point for citizens/jobs (classic city builder rule).
// A road component that does not touch the edge is considered disconnected and won't provide access.
void ComputeEdgeConnectedRoads(const World& world, std::vector<std::uint8_t>& outRoadToEdge)
{
  const int w = world.width();
  const int h = world.height();

  outRoadToEdge.clear();
  if (w <= 0 || h <= 0) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  outRoadToEdge.assign(n, 0);

  std::vector<int> queue;
  queue.reserve(n / 8);

  auto push = [&](int x, int y) {
    const int idx = y * w + x;
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx >= outRoadToEdge.size()) return;
    if (outRoadToEdge[uidx]) return;
    outRoadToEdge[uidx] = 1;
    queue.push_back(idx);
  };

  auto isRoad = [&](int x, int y) -> bool {
    return world.inBounds(x, y) && world.at(x, y).overlay == Overlay::Road;
  };

  // Seed BFS with any road tile on the map border.
  for (int x = 0; x < w; ++x) {
    if (isRoad(x, 0)) push(x, 0);
    if (h > 1 && isRoad(x, h - 1)) push(x, h - 1);
  }
  for (int y = 1; y < h - 1; ++y) {
    if (isRoad(0, y)) push(0, y);
    if (w > 1 && isRoad(w - 1, y)) push(w - 1, y);
  }

  // No border roads => no outside connection.
  if (queue.empty()) return;

  std::size_t qHead = 0;
  while (qHead < queue.size()) {
    const int idx = queue[qHead++];
    const int x = idx % w;
    const int y = idx / w;

    constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& d : dirs) {
      const int nx = x + d[0];
      const int ny = y + d[1];
      if (!world.inBounds(nx, ny)) continue;
      if (world.at(nx, ny).overlay != Overlay::Road) continue;

      const int nidx = ny * w + nx;
      const std::size_t unidx = static_cast<std::size_t>(nidx);
      if (unidx >= outRoadToEdge.size()) continue;
      if (outRoadToEdge[unidx]) continue;

      outRoadToEdge[unidx] = 1;
      queue.push_back(nidx);
    }
  }
}

bool HasAdjacentEdgeConnectedRoad(const World& world, const std::vector<std::uint8_t>& roadToEdge, int x, int y)
{
  if (!world.inBounds(x, y)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (roadToEdge.size() != expected) return false;

  auto check = [&](int nx, int ny) -> bool {
    if (!world.inBounds(nx, ny)) return false;
    if (world.at(nx, ny).overlay != Overlay::Road) return false;
    const int idx = ny * w + nx;
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx >= roadToEdge.size()) return false;
    return roadToEdge[uidx] != 0;
  };

  return check(x + 1, y) || check(x - 1, y) || check(x, y + 1) || check(x, y - 1);
}

// Parks are modeled as an *area of influence* rather than a global ratio.
// We compute a simple "coverage" ratio: what fraction of zone tiles are within
// a Manhattan distance <= radius of any park that is connected to a road.
//
// Notes:
// - We treat Water as a barrier so disconnected islands don't share park benefits.
// - This is intentionally lightweight: a multi-source BFS on a 96x96 grid is cheap.
float ParkCoverageRatio(const World& world, int radius, const std::vector<std::uint8_t>* roadToEdge)
{
  // Compatibility mode: treat parks as a global ratio (old behavior).
  if (radius <= 0) {
    int zones = 0;
    int parks = 0;
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);

        if (t.overlay == Overlay::Park) {
          bool connected = world.hasAdjacentRoad(x, y);
          if (roadToEdge) connected = HasAdjacentEdgeConnectedRoad(world, *roadToEdge, x, y);
          if (connected) parks++;
        }

        const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                             t.overlay == Overlay::Industrial);
        if (isZone) zones++;
      }
    }
    if (zones <= 0) return 0.0f;
    return static_cast<float>(parks) / static_cast<float>(zones);
  }

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0.0f;

  constexpr int kInf = 1'000'000;
  std::vector<int> dist(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), kInf);
  std::vector<int> queue;
  queue.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) / 8);

  auto idxOf = [&](int x, int y) -> int { return y * w + x; };

  // Initialize BFS sources: parks that have a road neighbor.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Park) continue;
      if (t.terrain == Terrain::Water) continue;
      bool connected = world.hasAdjacentRoad(x, y);
      if (roadToEdge) connected = HasAdjacentEdgeConnectedRoad(world, *roadToEdge, x, y);
      if (!connected) continue;
      const int idx = idxOf(x, y);
      dist[static_cast<std::size_t>(idx)] = 0;
      queue.push_back(idx);
    }
  }

  // No connected parks => no coverage.
  if (queue.empty()) return 0.0f;

  // Multi-source BFS (4-neighborhood) limited to the configured radius.
  std::size_t qHead = 0;
  while (qHead < queue.size()) {
    const int idx = queue[qHead++];
    const int d = dist[static_cast<std::size_t>(idx)];
    if (d >= radius) continue;

    const int x = idx % w;
    const int y = idx / w;

    constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& dir : dirs) {
      const int nx = x + dir[0];
      const int ny = y + dir[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (world.at(nx, ny).terrain == Terrain::Water) continue;

      const int nidx = idxOf(nx, ny);
      int& nd = dist[static_cast<std::size_t>(nidx)];
      if (nd <= d + 1) continue;
      nd = d + 1;
      queue.push_back(nidx);
    }
  }

  int zones = 0;
  int covered = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                           t.overlay == Overlay::Industrial);
      if (!isZone) continue;

      zones++;
      const int idx = idxOf(x, y);
      if (dist[static_cast<std::size_t>(idx)] <= radius) covered++;
    }
  }

  if (zones <= 0) return 0.0f;
  return static_cast<float>(covered) / static_cast<float>(zones);
}
} // namespace

void Simulator::stepOnce(World& world)
{
  // Ensure manual stepping is deterministic and doesn't accidentally queue extra ticks.
  m_accum = 0.0f;
  step(world);
}

void Simulator::update(World& world, float dt)
{
  m_accum += dt;
  while (m_accum >= m_cfg.tickSeconds) {
    m_accum -= m_cfg.tickSeconds;
    step(world);
  }
}

void Simulator::refreshDerivedStats(World& world) const
{
  Stats& s = world.stats();
  const ScanResult scan = ScanWorld(world);

  // Employment: fill jobs up to population.
  const int employed = std::min(scan.population, scan.jobsCap);

  // Happiness: parks help (locally), unemployment hurts.
  std::vector<std::uint8_t> roadToEdge;
  if (m_cfg.requireOutsideConnection) {
    ComputeEdgeConnectedRoads(world, roadToEdge);
  }

  const float parkCoverage = ParkCoverageRatio(world, m_cfg.parkInfluenceRadius,
                                              m_cfg.requireOutsideConnection ? &roadToEdge : nullptr);
  const float parkBonus = std::min(0.25f, parkCoverage * 0.35f);

  const float unemployment = (scan.population > 0)
                                 ? (1.0f - (static_cast<float>(employed) / static_cast<float>(scan.population)))
                                 : 0.0f;

  s.happiness = Clamp01(0.45f + parkBonus - unemployment * 0.35f);

  s.population = scan.population;
  s.housingCapacity = scan.housingCap;
  s.jobsCapacity = scan.jobsCap;
  s.employed = employed;
  s.roads = scan.roads;
  s.parks = scan.parks;
}

void Simulator::step(World& world)
{
  Stats& s = world.stats();
  s.day++;

  // Pass 1: capacities and static counts.
  const ScanResult scan = ScanWorld(world);
  const int housingCap = scan.housingCap;
  const int jobsCap = scan.jobsCap;
  const int roads = scan.roads;
  const int parks = scan.parks;

  // Precompute which roads are connected to the map border ("outside connection").
  // When requireOutsideConnection is enabled, zones only function if they touch a road
  // component that reaches the edge of the map.
  std::vector<std::uint8_t> roadToEdge;
  if (m_cfg.requireOutsideConnection) {
    ComputeEdgeConnectedRoads(world, roadToEdge);
  }

  auto hasZoneAccess = [&](int x, int y) -> bool {
    if (!world.hasAdjacentRoad(x, y)) return false;
    if (!m_cfg.requireOutsideConnection) return true;
    return HasAdjacentEdgeConnectedRoad(world, roadToEdge, x, y);
  };

  // Demand model: housing grows if there are jobs + happiness.
  // Very simple, but good enough as a starter.
  const float jobPressure = (housingCap > 0) ? (static_cast<float>(jobsCap) / static_cast<float>(housingCap)) : 0.0f;
  const float demand = Clamp01(0.15f + 0.70f * std::min(jobPressure, 1.0f) + 0.25f * s.happiness);

  // Pass 2: residential update (population moves toward target occupancy).
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;

      const bool access = hasZoneAccess(x, y);
      const int cap = HousingForLevel(t.level);

      if (!access) {
        const int decay = 1 + t.level;
        t.occupants = static_cast<std::uint16_t>(std::max(0, static_cast<int>(t.occupants) - decay));
        continue;
      }

      const int target = static_cast<int>(std::round(static_cast<float>(cap) * demand));
      const int cur = static_cast<int>(t.occupants);

      if (cur < target) {
        const int grow = 1 + t.level;
        t.occupants = static_cast<std::uint16_t>(std::min(cap, cur + grow));
      } else if (cur > target) {
        t.occupants = static_cast<std::uint16_t>(std::max(0, cur - 1));
      }
    }
  }

  // Recompute population after residential update.
  int population = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay == Overlay::Residential) population += static_cast<int>(t.occupants);
    }
  }

  // Employment: fill jobs up to population.
  const int employed = std::min(population, jobsCap);

  // Pass 3: distribute employment across job tiles.
  int remainingWorkers = employed;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;

      const bool access = hasZoneAccess(x, y);
      if (!access) {
        t.occupants = static_cast<std::uint16_t>(std::max(0, static_cast<int>(t.occupants) - 1));
        continue;
      }

      int cap = 0;
      if (t.overlay == Overlay::Commercial) cap = JobsCommercialForLevel(t.level);
      if (t.overlay == Overlay::Industrial) cap = JobsIndustrialForLevel(t.level);

      const int assigned = std::min(cap, remainingWorkers);
      t.occupants = static_cast<std::uint16_t>(assigned);
      remainingWorkers -= assigned;
    }
  }

  // Economy: tiny placeholder model.
  const int income = employed * 2;
  const int maintenance = roads * 1 + parks * 1;
  s.money += income - maintenance;

  // Happiness: parks help (locally), unemployment hurts.
  const float parkCoverage = ParkCoverageRatio(world, m_cfg.parkInfluenceRadius,
                                               m_cfg.requireOutsideConnection ? &roadToEdge : nullptr);
  const float parkBonus = std::min(0.25f, parkCoverage * 0.35f);

  const float unemployment = (population > 0) ? (1.0f - (static_cast<float>(employed) / static_cast<float>(population)))
                                              : 0.0f;

  s.happiness = Clamp01(0.45f + parkBonus - unemployment * 0.35f);

  // Optional auto-upgrades: if the city is doing well, some buildings level up.
  RNG rng(world.seed() ^ (static_cast<std::uint64_t>(s.day) * 0x9E3779B97F4A7C15ULL));
  if (s.happiness > 0.62f && s.money > 100) {
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        Tile& t = world.at(x, y);
        const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
        if (!isZone) continue;
        if (!hasZoneAccess(x, y)) continue;
        if (t.level >= 3) continue;

        // small chance per tick
        const float p = 0.0025f + 0.0035f * s.happiness;
        if (rng.chance(p)) {
          t.level++;
          // Cosmetic: some eviction during construction
          t.occupants = static_cast<std::uint16_t>(static_cast<int>(t.occupants) * 0.8f);
          s.money -= 25;
        }
      }
    }
  }

  // Update stats snapshot.
  s.population = population;
  s.housingCapacity = housingCap;
  s.jobsCapacity = jobsCap;
  s.employed = employed;
  s.roads = roads;
  s.parks = parks;
}


} // namespace isocity
