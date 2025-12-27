#pragma once

#include "isocity/Config.hpp"
#include "isocity/EditHistory.hpp"
#include "isocity/Iso.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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
  void applyToolBrush(int centerX, int centerY);
  void showToast(const std::string& msg, float seconds = 2.5f);

  void beginPaintStroke();
  void endPaintStroke();
  void doUndo();
  void doRedo();

  void handleInput(float dt);
  void update(float dt);
  void draw();

  struct StrokeFeedback {
    bool noMoney = false;
    bool water = false;
    bool noRoad = false;
    bool occupied = false;

    void clear()
    {
      noMoney = false;
      water = false;
      noRoad = false;
      occupied = false;
    }

    bool any() const { return noMoney || water || noRoad || occupied; }
  };

  Config m_cfg;
  RaylibContext m_rl;

  ProcGenConfig m_procCfg{};
  World m_world;

  Simulator m_sim;
  Renderer m_renderer;

  EditHistory m_history;

  Camera2D m_camera{};

  Tool m_tool = Tool::Road;
  int m_brushRadius = 0; // 0 = single tile, 1 = diamond radius 1, etc.

  bool m_painting = false;
  StrokeFeedback m_strokeFeedback{};

  // Per-stroke tile mask: ensure each tile is affected at most once per stroke.
  // This prevents accidental multi-upgrades (zones) when holding the mouse down
  // while the cursor isn't moving.
  int m_strokeApplyW = 0;
  int m_strokeApplyH = 0;
  std::vector<std::uint8_t> m_strokeApplied;

  bool m_showHelp = true;
  bool m_drawGrid = false;

  // Simulation controls (pause/step/speed) are handled at the game layer so the simulator stays simple.
  bool m_simPaused = false;
  int m_simSpeedIndex = 2; // 0.25x, 0.5x, 1x, 2x, ... (default = 1x)

  float m_timeSec = 0.0f;
  std::string m_toast;
  float m_toastTimer = 0.0f;

  std::optional<Point> m_hovered;
};

} // namespace isocity
