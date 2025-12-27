#pragma once

#include "isocity/Config.hpp"
#include "isocity/Iso.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <optional>

#include "raylib.h"

namespace isocity {

// Small RAII wrapper so raylib window/context lifetime is correct relative to textures.
struct RaylibContext {
  RaylibContext(int w, int h, const char* title, bool vsync);
  ~RaylibContext();

  RaylibContext(const RaylibContext&) = delete;
  RaylibContext& operator=(const RaylibContext&) = delete;
};

class Game {
public:
  explicit Game(Config cfg);
  ~Game() = default;

  void run();

private:
  void resetWorld(std::uint64_t newSeed);

  void handleInput(float dt);
  void update(float dt);
  void draw();

  Config m_cfg;
  RaylibContext m_rl;

  ProcGenConfig m_procCfg{};
  World m_world;

  Simulator m_sim;
  Renderer m_renderer;

  Camera2D m_camera{};

  Tool m_tool = Tool::Road;
  bool m_showHelp = true;
  bool m_drawGrid = false;

  float m_timeSec = 0.0f;
  std::optional<Point> m_hovered;
};

} // namespace isocity
