#pragma once

#include "isocity/World.hpp"

namespace isocity {

struct SimConfig {
  float tickSeconds = 0.5f; // how often the sim advances (in real seconds)
};

class Simulator {
public:
  explicit Simulator(SimConfig cfg = {}) : m_cfg(cfg) {}

  void update(World& world, float dt);

private:
  void step(World& world);

  SimConfig m_cfg;
  float m_accum = 0.0f;
};

} // namespace isocity
