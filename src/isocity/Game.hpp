#pragma once

#include "isocity/Config.hpp"
#include "isocity/EditHistory.hpp"
#include "isocity/Iso.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"
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

  // Quick save/load supports multiple slots (UI convenience).
  // Slot 1 intentionally uses the legacy filename so existing quick-saves keep working.
  std::string savePathForSlot(int slot) const;
  void cycleSaveSlot(int delta);

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

  // Quick save slot (1..kMaxSaveSlot). Slot 1 == legacy isocity_save.bin.
  int m_saveSlot = 1;


  // Debug overlay: show which roads (and adjacent zones) are connected to the map edge
  // (an "outside connection"). This helps explain why a road network is disconnected.
  bool m_showOutsideOverlay = false;
  std::vector<std::uint8_t> m_outsideOverlayRoadToEdge;

  // Debug overlay: show extracted road graph (nodes/edges) for the current road network.
  bool m_showRoadGraphOverlay = false;
  bool m_roadGraphDirty = true;
  RoadGraph m_roadGraph;

  // Debug overlay: traffic heatmap (derived commute model).
  bool m_showTrafficOverlay = false;
  bool m_trafficDirty = true;
  TrafficResult m_traffic;

  // Debug overlay: goods logistics heatmap (derived supply chain model).
  bool m_showGoodsOverlay = false;
  bool m_goodsDirty = true;
  GoodsResult m_goods;

  // Simulation controls (pause/step/speed) are handled at the game layer so the simulator stays simple.
  bool m_simPaused = false;
  int m_simSpeedIndex = 2; // 0.25x, 0.5x, 1x, 2x, ... (default = 1x)

  float m_timeSec = 0.0f;
  std::string m_toast;
  float m_toastTimer = 0.0f;

  std::optional<Point> m_hovered;

  // Inspect tool: optional tile selection + debug path to the map edge.
  std::optional<Point> m_inspectSelected;
  std::vector<Point> m_inspectPath;
  int m_inspectPathCost = 0;
  std::string m_inspectInfo;

  // Road tool: Shift+drag builds a cheapest path (preferring existing roads) from
  // a start tile to the current hovered tile.
  bool m_roadDragActive = false;
  std::optional<Point> m_roadDragStart;
  std::optional<Point> m_roadDragEnd;
  std::vector<Point> m_roadDragPath;
  int m_roadDragBuildCost = 0; // number of tiles in the path that are not already roads
  bool m_roadDragValid = false;
};

} // namespace isocity
