#pragma once

#include "isocity/World.hpp"

namespace isocity {

struct SimConfig {
  float tickSeconds = 0.5f; // how often the sim advances (in real seconds)

  // Parks boost happiness for *nearby* zone tiles (a simple "coverage" model).
  // Manhattan distance in tile space.
  //
  // Set to 0 to disable locality (parks behave like a global ratio again).
  int parkInfluenceRadius = 6;
};

class Simulator {
public:
  explicit Simulator(SimConfig cfg = {}) : m_cfg(cfg) {}

  void update(World& world, float dt);

  // Advance the simulation by exactly one tick (increments day, updates economy, etc.).
  // Resets the internal timer accumulator so stepping is deterministic.
  void stepOnce(World& world);

  // Clears the internal tick accumulator (useful when pausing/unpausing or changing sim speed).
  void resetTimer() { m_accum = 0.0f; }

  const SimConfig& config() const { return m_cfg; }

  // Recompute derived HUD stats (population/capacities/roads/parks/employment/happiness)
  // without advancing time or modifying tiles.
  void refreshDerivedStats(World& world) const;

private:
  void step(World& world);

  SimConfig m_cfg;
  float m_accum = 0.0f;
};

} // namespace isocity
