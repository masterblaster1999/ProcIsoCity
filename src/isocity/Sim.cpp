#include "isocity/Sim.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>

namespace isocity {

namespace {
inline int HousingForLevel(int level) { return 10 * level; }
inline int JobsCommercialForLevel(int level) { return 8 * level; }
inline int JobsIndustrialForLevel(int level) { return 12 * level; }

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
} // namespace

void Simulator::update(World& world, float dt)
{
  m_accum += dt;
  while (m_accum >= m_cfg.tickSeconds) {
    m_accum -= m_cfg.tickSeconds;
    step(world);
  }
}

void Simulator::step(World& world)
{
  Stats& s = world.stats();
  s.day++;

  // Pass 1: capacities and static counts.
  int housingCap = 0;
  int jobsCap = 0;
  int roads = 0;
  int parks = 0;

  int zoneTiles = 0;

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);

      if (t.overlay == Overlay::Road) roads++;
      if (t.overlay == Overlay::Park) parks++;

      if (t.overlay == Overlay::Residential) {
        zoneTiles++;
        housingCap += HousingForLevel(t.level);
      } else if (t.overlay == Overlay::Commercial) {
        zoneTiles++;
        jobsCap += JobsCommercialForLevel(t.level);
      } else if (t.overlay == Overlay::Industrial) {
        zoneTiles++;
        jobsCap += JobsIndustrialForLevel(t.level);
      }
    }
  }

  // Demand model: housing grows if there are jobs + happiness.
  // Very simple, but good enough as a starter.
  const float jobPressure = (housingCap > 0) ? (static_cast<float>(jobsCap) / static_cast<float>(housingCap)) : 0.0f;
  const float demand = Clamp01(0.15f + 0.70f * std::min(jobPressure, 1.0f) + 0.25f * s.happiness);

  // Pass 2: residential update (population moves toward target occupancy).
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;

      const bool access = world.hasAdjacentRoad(x, y);
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

      const bool access = world.hasAdjacentRoad(x, y);
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

  // Happiness: parks help, unemployment hurts.
  const float parkRatio = (zoneTiles > 0) ? (static_cast<float>(parks) / static_cast<float>(zoneTiles)) : 0.0f;
  const float parkBonus = std::min(0.25f, parkRatio * 0.35f);

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
        if (!world.hasAdjacentRoad(x, y)) continue;
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
