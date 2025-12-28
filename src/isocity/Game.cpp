#include "isocity/Game.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace isocity {

namespace {
// Slot 1 uses the legacy filename so existing quick-saves keep working.
constexpr const char* kLegacyQuickSavePath = "isocity_save.bin";
constexpr int kSaveSlotMin = 1;
constexpr int kSaveSlotMax = 5;

// Discrete sim speed presets (dt multiplier).
constexpr float kSimSpeeds[] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f};
constexpr int kSimSpeedCount = static_cast<int>(sizeof(kSimSpeeds) / sizeof(kSimSpeeds[0]));
} // namespace

std::string Game::savePathForSlot(int slot) const
{
  const int s = std::clamp(slot, kSaveSlotMin, kSaveSlotMax);
  if (s == 1) return std::string(kLegacyQuickSavePath);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "isocity_save_slot%d.bin", s);
  return std::string(buf);
}

void Game::cycleSaveSlot(int delta)
{
  const int range = (kSaveSlotMax - kSaveSlotMin + 1);
  if (range <= 0) return;

  int s = m_saveSlot + delta;
  while (s < kSaveSlotMin) s += range;
  while (s > kSaveSlotMax) s -= range;
  m_saveSlot = s;
}

RaylibContext::RaylibContext(int w, int h, const char* title, bool vsync)
{
  if (vsync) SetConfigFlags(FLAG_VSYNC_HINT);
  InitWindow(w, h, title);

  // You can tune this later or expose it as a config.
  SetTargetFPS(60);
}

RaylibContext::~RaylibContext() { CloseWindow(); }

Game::Game(Config cfg)
    : m_cfg(cfg)
    , m_rl(cfg.windowWidth, cfg.windowHeight, "ProcIsoCity", cfg.vsync)
    , m_world()
    , m_sim(SimConfig{})
    , m_renderer(cfg.tileWidth, cfg.tileHeight, cfg.seed)
{
  resetWorld(m_cfg.seed);

  // Camera
  m_camera.zoom = 1.0f;
  m_camera.rotation = 0.0f;
  m_camera.offset = Vector2{m_cfg.windowWidth * 0.5f, m_cfg.windowHeight * 0.5f};

  const Vector2 center =
      TileToWorldCenter(m_cfg.mapWidth / 2, m_cfg.mapHeight / 2, static_cast<float>(m_cfg.tileWidth),
                        static_cast<float>(m_cfg.tileHeight));
  m_camera.target = center;
}

void Game::showToast(const std::string& msg, float seconds)
{
  m_toast = msg;
  m_toastTimer = std::max(0.0f, seconds);
}

void Game::applyToolBrush(int centerX, int centerY)
{
  if (m_tool == Tool::Inspect) return;

  const int r = std::max(0, m_brushRadius);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      // Diamond brush (fits iso grid nicely).
      if (std::abs(dx) + std::abs(dy) > r) continue;
      const int tx = centerX + dx;
      const int ty = centerY + dy;

      // Skip out-of-bounds early.
      if (!m_world.inBounds(tx, ty)) continue;

      // Within a single paint stroke, apply at most once per tile. This avoids
      // accidental "multi-upgrades" (zones) if the cursor is held still.
      if (!m_strokeApplied.empty() && m_strokeApplyW == m_world.width() && m_strokeApplyH == m_world.height()) {
        const int idx = ty * m_strokeApplyW + tx;
        if (idx >= 0) {
          const std::size_t uidx = static_cast<std::size_t>(idx);
          if (uidx < m_strokeApplied.size()) {
            if (m_strokeApplied[uidx]) continue;
            m_strokeApplied[uidx] = 1;
          }
        }
      }

      // Capture pre-edit state for undo/redo.
      //
      // Road auto-tiling masks are fixed up locally by EditHistory (undo/redo) so it's
      // sufficient to track the edited tile itself.
      const Overlay beforeOverlay = m_world.at(tx, ty).overlay;
      m_history.noteTilePreEdit(m_world, tx, ty);

      const ToolApplyResult res = m_world.applyTool(m_tool, tx, ty);
      switch (res) {
      case ToolApplyResult::InsufficientFunds: m_strokeFeedback.noMoney = true; break;
      case ToolApplyResult::BlockedNoRoad: m_strokeFeedback.noRoad = true; break;
      case ToolApplyResult::BlockedWater: m_strokeFeedback.water = true; break;
      case ToolApplyResult::BlockedOccupied: m_strokeFeedback.occupied = true; break;
      default: break;
      }

      // Mark cached road graph dirty when road topology changes.
      if (res == ToolApplyResult::Applied) {
        // Traffic depends on roads + zones + occupancy.
        m_trafficDirty = true;
        // Goods logistics depend on roads + industrial/commercial zoning.
        m_goodsDirty = true;

        if (m_tool == Tool::Road || (m_tool == Tool::Bulldoze && beforeOverlay == Overlay::Road)) {
          m_roadGraphDirty = true;
        }
      }
    }
  }
}

void Game::beginPaintStroke()
{
  if (m_painting) return;
  m_painting = true;
  m_strokeFeedback.clear();
  m_history.beginStroke(m_world);

  // Per-stroke applied tile mask.
  m_strokeApplyW = m_world.width();
  m_strokeApplyH = m_world.height();
  const std::size_t n =
      static_cast<std::size_t>(std::max(0, m_strokeApplyW) * std::max(0, m_strokeApplyH));
  m_strokeApplied.assign(n, 0);
}

void Game::endPaintStroke()
{
  if (!m_painting) return;
  m_painting = false;
  m_history.endStroke(m_world);

  m_strokeApplied.clear();
  m_strokeApplyW = 0;
  m_strokeApplyH = 0;

  // Keep HUD numbers (roads/parks/capacities) responsive even before the next sim tick.
  m_sim.refreshDerivedStats(m_world);

  // Provide one toast per stroke for common build failures (no money, no road access, etc.).
  if (m_strokeFeedback.any()) {
    std::string msg = "Some placements failed: ";
    bool first = true;
    auto add = [&](const char* s) {
      if (!first) msg += ", ";
      msg += s;
      first = false;
    };

    if (m_strokeFeedback.noMoney) add("not enough money");
    if (m_strokeFeedback.noRoad) add("need adjacent road");
    if (m_strokeFeedback.water) add("can't build on water");
    if (m_strokeFeedback.occupied) add("tile occupied");

    showToast(msg, 3.0f);
  }
}

void Game::doUndo()
{
  // Commit any in-progress stroke before undoing.
  endPaintStroke();

  if (m_history.undo(m_world)) {
    m_sim.refreshDerivedStats(m_world);
    m_roadGraphDirty = true;
    m_trafficDirty = true;
    m_goodsDirty = true;
    showToast(TextFormat("Undo (%d left)", static_cast<int>(m_history.undoSize())));
  } else {
    showToast("Nothing to undo");
  }
}

void Game::doRedo()
{
  endPaintStroke();

  if (m_history.redo(m_world)) {
    m_sim.refreshDerivedStats(m_world);
    m_roadGraphDirty = true;
    m_trafficDirty = true;
    m_goodsDirty = true;
    showToast(TextFormat("Redo (%d left)", static_cast<int>(m_history.redoSize())));
  } else {
    showToast("Nothing to redo");
  }
}

void Game::resetWorld(std::uint64_t newSeed)
{
  if (newSeed == 0) newSeed = TimeSeed();

  m_cfg.seed = newSeed;
  m_world = GenerateWorld(m_cfg.mapWidth, m_cfg.mapHeight, newSeed, m_procCfg);
  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;


  // New world invalidates history.
  m_history.clear();
  m_painting = false;

  // Clear inspect selection/debug overlays.
  m_inspectSelected.reset();
  m_inspectPath.clear();
  m_inspectPathCost = 0;
  m_inspectInfo.clear();

  // Clear any in-progress road drag preview.
  m_roadDragActive = false;
  m_roadDragStart.reset();
  m_roadDragEnd.reset();
  m_roadDragPath.clear();
  m_roadDragBuildCost = 0;
  m_roadDragValid = false;

  // Optional: vary procedural textures per seed (still no assets-from-disk).
  m_renderer.rebuildTextures(newSeed);

  // Make HUD stats immediately correct (without waiting for the first sim tick).
  m_sim.refreshDerivedStats(m_world);

  // Update title with seed.
  SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(newSeed)));

  // Recenter camera.
  m_camera.target =
      TileToWorldCenter(m_cfg.mapWidth / 2, m_cfg.mapHeight / 2, static_cast<float>(m_cfg.tileWidth),
                        static_cast<float>(m_cfg.tileHeight));
}

void Game::run()
{
  while (!WindowShouldClose()) {
    const float dt = GetFrameTime();
    m_timeSec += dt;

    handleInput(dt);
    update(dt);
    draw();
  }
}

void Game::handleInput(float dt)
{
  // Update hovered tile from mouse.
  const Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), m_camera);
  m_hovered = WorldToTile(mouseWorld, m_world.width(), m_world.height(), static_cast<float>(m_cfg.tileWidth),
                         static_cast<float>(m_cfg.tileHeight));

  // Undo/redo
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  if (ctrl && shift && IsKeyPressed(KEY_Z)) {
    doRedo();
  } else if (ctrl && IsKeyPressed(KEY_Z)) {
    doUndo();
  } else if (ctrl && IsKeyPressed(KEY_Y)) {
    doRedo();
  }

  // Simulation controls
  auto simSpeed = [&]() -> float {
    const int si = std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1);
    return kSimSpeeds[static_cast<std::size_t>(si)];
  };

  if (IsKeyPressed(KEY_SPACE)) {
    endPaintStroke();
    m_simPaused = !m_simPaused;
    m_sim.resetTimer();
    showToast(m_simPaused ? "Sim paused" : "Sim running");
  }

  if (m_simPaused && IsKeyPressed(KEY_N)) {
    endPaintStroke();
    m_sim.stepOnce(m_world);
    m_trafficDirty = true;
    m_goodsDirty = true;
    showToast("Sim step");
  }

  if (IsKeyPressed(KEY_KP_ADD) || (IsKeyPressed(KEY_EQUAL) && shift)) {
    const int before = m_simSpeedIndex;
    m_simSpeedIndex = std::clamp(m_simSpeedIndex + 1, 0, kSimSpeedCount - 1);
    if (m_simSpeedIndex != before) {
      m_sim.resetTimer();
      showToast(TextFormat("Sim speed: x%.2f", static_cast<double>(simSpeed())));
    }
  }

  if (IsKeyPressed(KEY_KP_SUBTRACT) || IsKeyPressed(KEY_MINUS)) {
    const int before = m_simSpeedIndex;
    m_simSpeedIndex = std::clamp(m_simSpeedIndex - 1, 0, kSimSpeedCount - 1);
    if (m_simSpeedIndex != before) {
      m_sim.resetTimer();
      showToast(TextFormat("Sim speed: x%.2f", static_cast<double>(simSpeed())));
    }
  }

  // Toggle UI
  if (IsKeyPressed(KEY_H)) m_showHelp = !m_showHelp;
  if (IsKeyPressed(KEY_G)) m_drawGrid = !m_drawGrid;
  if (IsKeyPressed(KEY_O)) {
    m_showOutsideOverlay = !m_showOutsideOverlay;
    showToast(m_showOutsideOverlay ? "Outside overlay: ON" : "Outside overlay: OFF");
  }

  if (IsKeyPressed(KEY_T)) {
    m_showRoadGraphOverlay = !m_showRoadGraphOverlay;
    m_roadGraphDirty = true;

    if (m_showRoadGraphOverlay) {
      m_roadGraph = BuildRoadGraph(m_world);
      m_roadGraphDirty = false;
      showToast(TextFormat("Road graph: ON (%d nodes, %d edges)", static_cast<int>(m_roadGraph.nodes.size()),
                           static_cast<int>(m_roadGraph.edges.size())));
    } else {
      showToast("Road graph: OFF");
    }
  }

  if (IsKeyPressed(KEY_V)) {
    m_showTrafficOverlay = !m_showTrafficOverlay;
    m_trafficDirty = true;

    if (m_showTrafficOverlay) {
      const float share = (m_world.stats().population > 0)
                              ? (static_cast<float>(m_world.stats().employed) /
                                 static_cast<float>(m_world.stats().population))
                              : 0.0f;

      TrafficConfig tc;
      tc.requireOutsideConnection = m_sim.config().requireOutsideConnection;
      m_traffic = ComputeCommuteTraffic(m_world, tc, share, nullptr);
      m_trafficDirty = false;

      showToast(TextFormat("Traffic overlay: ON (%d commuters, avg %.1f, cong %.0f%%)", m_traffic.totalCommuters,
                           static_cast<double>(m_traffic.avgCommute),
                           static_cast<double>(m_traffic.congestion * 100.0f)));
    } else {
      showToast("Traffic overlay: OFF");
    }
  }


  if (IsKeyPressed(KEY_B)) {
    m_showGoodsOverlay = !m_showGoodsOverlay;
    m_goodsDirty = true;

    if (m_showGoodsOverlay) {
      GoodsConfig gc;
      gc.requireOutsideConnection = m_sim.config().requireOutsideConnection;
      m_goods = ComputeGoodsFlow(m_world, gc, nullptr);
      m_goodsDirty = false;

      showToast(TextFormat("Goods overlay: ON (deliv %d/%d, sat %.0f%%, imp %d, exp %d)",
                           m_goods.goodsDelivered, m_goods.goodsDemand,
                           static_cast<double>(m_goods.satisfaction * 100.0f),
                           m_goods.goodsImported, m_goods.goodsExported));
    } else {
      showToast("Goods overlay: OFF");
    }
  }

  // Brush radius
  if (IsKeyPressed(KEY_LEFT_BRACKET)) {
    m_brushRadius = std::max(0, m_brushRadius - 1);
    showToast(TextFormat("Brush radius: %d", m_brushRadius));
  }
  if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
    m_brushRadius = std::min(8, m_brushRadius + 1);
    showToast(TextFormat("Brush radius: %d", m_brushRadius));
  }

  // Save slot selection
  if (IsKeyPressed(KEY_F6)) {
    endPaintStroke();
    // Hold Shift to cycle backwards.
    cycleSaveSlot(shift ? -1 : 1);
    showToast(TextFormat("Save slot: %d", m_saveSlot));
  }

  // Save / Load (quick save)
  if (IsKeyPressed(KEY_F5)) {
    std::string err;
    const std::string path = savePathForSlot(m_saveSlot);
    if (SaveWorldBinary(m_world, m_procCfg, path, err)) {
      showToast(TextFormat("Saved slot %d: %s", m_saveSlot, path.c_str()));
    } else {
      showToast(std::string("Save failed: ") + err, 4.0f);
    }
  }

  if (IsKeyPressed(KEY_F9)) {
    endPaintStroke();
    std::string err;
    World loaded;
    ProcGenConfig loadedProcCfg{};
    const std::string path = savePathForSlot(m_saveSlot);
    if (LoadWorldBinary(loaded, loadedProcCfg, path, err)) {
      m_world = std::move(loaded);
      m_procCfg = loadedProcCfg;
      m_roadGraphDirty = true;
      m_trafficDirty = true;
      m_goodsDirty = true;


      // Loading invalidates history.
      m_history.clear();
      m_painting = false;

      // Loaded world invalidates inspect selection/debug overlays.
      m_inspectSelected.reset();
      m_inspectPath.clear();
      m_inspectPathCost = 0;
      m_inspectInfo.clear();

      // Loaded world invalidates any road-drag preview.
      m_roadDragActive = false;
      m_roadDragStart.reset();
      m_roadDragEnd.reset();
      m_roadDragPath.clear();
      m_roadDragBuildCost = 0;
      m_roadDragValid = false;

      // Keep config in sync with loaded world, so regen & camera recenter behave.
      m_cfg.mapWidth = m_world.width();
      m_cfg.mapHeight = m_world.height();
      m_cfg.seed = m_world.seed();

      m_renderer.rebuildTextures(m_cfg.seed);
      SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(m_cfg.seed)));

      // Recenter camera on loaded map.
      m_camera.target =
          TileToWorldCenter(m_cfg.mapWidth / 2, m_cfg.mapHeight / 2, static_cast<float>(m_cfg.tileWidth),
                            static_cast<float>(m_cfg.tileHeight));

      m_sim.refreshDerivedStats(m_world);

      showToast(TextFormat("Loaded slot %d: %s", m_saveSlot, path.c_str()));
    } else {
      showToast(std::string("Load failed: ") + err, 4.0f);
    }
  }

  // Regenerate
  if (IsKeyPressed(KEY_R)) {
    endPaintStroke();
    resetWorld(TimeSeed());
  }

  // Tool selection
  auto setTool = [&](Tool t) {
    if (m_tool == t) return;
    endPaintStroke();
    m_tool = t;

    // Switching tools clears any inspect selection/path.
    m_inspectSelected.reset();
    m_inspectPath.clear();
    m_inspectPathCost = 0;
    m_inspectInfo.clear();

    // Switching tools also cancels any road-drag preview.
    m_roadDragActive = false;
    m_roadDragStart.reset();
    m_roadDragEnd.reset();
    m_roadDragPath.clear();
    m_roadDragBuildCost = 0;
    m_roadDragValid = false;
  };

  if (IsKeyPressed(KEY_Q)) setTool(Tool::Inspect);
  if (IsKeyPressed(KEY_ONE)) setTool(Tool::Road);
  if (IsKeyPressed(KEY_TWO)) setTool(Tool::Residential);
  if (IsKeyPressed(KEY_THREE)) setTool(Tool::Commercial);
  if (IsKeyPressed(KEY_FOUR)) setTool(Tool::Industrial);
  if (IsKeyPressed(KEY_FIVE)) setTool(Tool::Park);
  if (IsKeyPressed(KEY_ZERO)) setTool(Tool::Bulldoze);

  // Camera pan: right mouse drag (raylib example style).
  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    endPaintStroke();
    Vector2 delta = GetMouseDelta();
    delta.x *= -1.0f / std::max(0.001f, m_camera.zoom);
    delta.y *= -1.0f / std::max(0.001f, m_camera.zoom);
    m_camera.target.x += delta.x;
    m_camera.target.y += delta.y;
  }

  // Keyboard pan (optional)
  const float panSpeed = 650.0f * dt / std::max(0.25f, m_camera.zoom);
  if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) m_camera.target.x -= panSpeed;
  if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) m_camera.target.x += panSpeed;
  if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) m_camera.target.y -= panSpeed;
  if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) m_camera.target.y += panSpeed;

  // Zoom around mouse cursor (raylib example style).
  const float wheel = GetMouseWheelMove();
  if (wheel != 0.0f) {
    const Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), m_camera);
    m_camera.offset = GetMousePosition();
    m_camera.target = mouseWorldPos;

    const float zoomIncrement = 0.125f;
    m_camera.zoom += wheel * zoomIncrement;
    m_camera.zoom = std::clamp(m_camera.zoom, 0.25f, 4.0f);
  }

  // Build/paint with left mouse.
  const bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
  const bool leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
  const bool leftReleased = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);

  // Road tool: Shift+drag plans a cheapest road path (prefers existing roads) and
  // commits the whole path on release (single undoable stroke).
  const bool roadDragMode = (m_tool == Tool::Road) && shift && !m_painting;

  if (roadDragMode) {
    // Start drag.
    if (leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Point start = *m_hovered;

      std::vector<Point> tmp;
      int buildCost = 0;
      if (!FindRoadBuildPath(m_world, start, start, tmp, &buildCost)) {
        showToast("Can't start a road path here", 2.5f);
      } else {
        endPaintStroke();
        m_roadDragActive = true;
        m_roadDragStart = start;
        m_roadDragEnd = start;
        m_roadDragPath = std::move(tmp);
        m_roadDragBuildCost = buildCost;
        m_roadDragValid = true;
      }
    }

    // Update preview.
    if (leftDown && m_roadDragActive && m_roadDragStart && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Point end = *m_hovered;
      if (!m_roadDragEnd || (m_roadDragEnd->x != end.x) || (m_roadDragEnd->y != end.y)) {
        m_roadDragEnd = end;

        std::vector<Point> tmp;
        int buildCost = 0;
        const bool ok = FindRoadBuildPath(m_world, *m_roadDragStart, end, tmp, &buildCost);
        if (ok && !tmp.empty()) {
          m_roadDragValid = true;
          m_roadDragPath = std::move(tmp);
          m_roadDragBuildCost = buildCost;
        } else {
          m_roadDragValid = false;
          m_roadDragPath.clear();
          m_roadDragBuildCost = 0;
        }
      }
    }

    // Commit on release.
    if (leftReleased && m_roadDragActive) {
      if (m_roadDragValid && !m_roadDragPath.empty()) {
        const int moneyBefore = m_world.stats().money;

        beginPaintStroke();
        const int savedRadius = m_brushRadius;
        m_brushRadius = 0; // path tool is always 1-tile wide

        for (const Point& p : m_roadDragPath) {
          applyToolBrush(p.x, p.y);
        }

        m_brushRadius = savedRadius;

        const bool hadFailures = m_strokeFeedback.any();
        endPaintStroke();

        if (!hadFailures) {
          const int spent = moneyBefore - m_world.stats().money;
          if (spent > 0) {
            showToast(TextFormat("Built road path (%d new tiles, cost %d)", m_roadDragBuildCost, spent));
          } else {
            showToast(TextFormat("Built road path (%d new tiles)", m_roadDragBuildCost));
          }
        }
      } else {
        showToast("No valid road path", 2.5f);
      }

      // Clear drag state.
      m_roadDragActive = false;
      m_roadDragStart.reset();
      m_roadDragEnd.reset();
      m_roadDragPath.clear();
      m_roadDragBuildCost = 0;
      m_roadDragValid = false;
    }
  }

  // Inspect click: select tile and (if possible) compute the shortest road path to the map edge.
  if (!roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::Inspect) {
    m_inspectSelected = *m_hovered;
    m_inspectPath.clear();
    m_inspectPathCost = 0;
    m_inspectInfo.clear();

    const Point sel = *m_inspectSelected;
    const Tile& t = m_world.at(sel.x, sel.y);

    auto pickAdjacentRoad = [&](Point& outRoad) -> bool {
      // Deterministic neighbor order.
      constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      for (const auto& d : dirs) {
        const int nx = sel.x + d[0];
        const int ny = sel.y + d[1];
        if (!m_world.inBounds(nx, ny)) continue;
        if (m_world.at(nx, ny).overlay == Overlay::Road) {
          outRoad = Point{nx, ny};
          return true;
        }
      }
      return false;
    };

    Point startRoad = sel;
    bool hasStartRoad = false;

    if (t.overlay == Overlay::Road) {
      hasStartRoad = true;
    } else {
      hasStartRoad = pickAdjacentRoad(startRoad);
    }

    if (!hasStartRoad) {
      m_inspectInfo = TextFormat("Inspect (%d,%d): no adjacent road", sel.x, sel.y);
      showToast(m_inspectInfo);
    } else {
      const bool ok = FindRoadPathToEdge(m_world, startRoad, m_inspectPath, &m_inspectPathCost);
      if (ok) {
        m_inspectInfo = TextFormat("Inspect (%d,%d): outside YES (road dist %d)", sel.x, sel.y, m_inspectPathCost);
        showToast(m_inspectInfo);
      } else {
        m_inspectInfo = TextFormat("Inspect (%d,%d): outside NO", sel.x, sel.y);
        showToast(m_inspectInfo, 3.0f);
      }
    }
  }

  if (!roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    beginPaintStroke();
  }

  if (!roadDragMode && leftDown && m_painting && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    applyToolBrush(m_hovered->x, m_hovered->y);
  }

  if (!roadDragMode && leftReleased) {
    endPaintStroke();
  }

  // If the window is resizable, keep the camera offset sane.
  if (IsWindowResized()) {
    // Only reset if not in the middle of a "zoom around cursor" moment.
    // (This is a simple heuristic; you can refine later.)
    if (wheel == 0.0f) {
      m_camera.offset = Vector2{GetScreenWidth() * 0.5f, GetScreenHeight() * 0.5f};
    }
  }
}

void Game::update(float dt)
{
  // Pause simulation while actively painting so an undoable "stroke" doesn't
  // accidentally include sim-driven money changes.
  if (!m_painting && !m_simPaused) {
    const int dayBefore = m_world.stats().day;
    const int si = std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1);
    const float speed = kSimSpeeds[static_cast<std::size_t>(si)];
    m_sim.update(m_world, dt * speed);

    // The sim may have advanced 0..N ticks. Traffic depends on occupants/jobs.
    if (m_world.stats().day != dayBefore) {
      m_trafficDirty = true;
      m_goodsDirty = true;
    }
  }

  if (m_toastTimer > 0.0f) {
    m_toastTimer -= dt;
    if (m_toastTimer < 0.0f) m_toastTimer = 0.0f;
  }
}

void Game::draw()
{
  BeginDrawing();
  ClearBackground(Color{30, 32, 38, 255});

  // World highlights: either the inspect path OR the road-drag preview (if active).
  std::optional<Point> selected = m_inspectSelected;
  const std::vector<Point>* pathPtr = (!m_inspectPath.empty()) ? &m_inspectPath : nullptr;
  int worldBrush = m_brushRadius;

  if (m_roadDragActive) {
    selected = m_roadDragStart;
    pathPtr = (m_roadDragValid && !m_roadDragPath.empty()) ? &m_roadDragPath : nullptr;
    // In drag-path mode, the brush outline is misleading (we're not painting a diamond).
    worldBrush = 0;
  }

  const std::vector<std::uint8_t>* outsideMask = nullptr;
  if (m_showOutsideOverlay) {
    ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
    outsideMask = &m_outsideOverlayRoadToEdge;
  }

  const std::vector<std::uint16_t>* trafficMask = nullptr;
  int trafficMax = 0;
  if (m_showTrafficOverlay) {
    if (m_trafficDirty) {
      const float share = (m_world.stats().population > 0)
                              ? (static_cast<float>(m_world.stats().employed) /
                                 static_cast<float>(m_world.stats().population))
                              : 0.0f;

      TrafficConfig tc;
      tc.requireOutsideConnection = m_sim.config().requireOutsideConnection;

      const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? outsideMask : nullptr);
      m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
      m_trafficDirty = false;
    }

    if (!m_traffic.roadTraffic.empty()) {
      trafficMask = &m_traffic.roadTraffic;
      trafficMax = m_traffic.maxTraffic;
    }
  }

  const std::vector<std::uint16_t>* goodsTrafficMask = nullptr;
  int goodsMax = 0;
  const std::vector<std::uint8_t>* commercialGoodsFill = nullptr;
  if (m_showGoodsOverlay) {
    if (m_goodsDirty) {
      GoodsConfig gc;
      gc.requireOutsideConnection = m_sim.config().requireOutsideConnection;

      const std::vector<std::uint8_t>* pre = (gc.requireOutsideConnection ? outsideMask : nullptr);
      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;
    }

    goodsTrafficMask = &m_goods.roadGoodsTraffic;
    goodsMax = m_goods.maxRoadGoodsTraffic;
    commercialGoodsFill = &m_goods.commercialFill;
  }

  m_renderer.drawWorld(m_world, m_camera, m_timeSec, m_hovered, m_drawGrid, worldBrush, selected, pathPtr, outsideMask,
                       trafficMask, trafficMax,
                       goodsTrafficMask, goodsMax,
                       commercialGoodsFill);

  // Road graph overlay (debug): nodes/edges extracted from the current road tiles.
  if (m_showRoadGraphOverlay) {
    if (m_roadGraphDirty) {
      m_roadGraph = BuildRoadGraph(m_world);
      m_roadGraphDirty = false;
    }

    if (!m_roadGraph.nodes.empty()) {
      BeginMode2D(m_camera);

      const float zoom = std::max(0.25f, m_camera.zoom);
      const float thickness = 2.5f / zoom;
      const float radius = 3.0f / zoom;

      const int w = m_world.width();

      auto edgeIsConnected = [&](const RoadGraphEdge& e) -> bool {
        if (!outsideMask) return true;
        if (w <= 0 || m_world.height() <= 0) return true;
        if (outsideMask->size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(m_world.height())) return true;
        for (const Point& p : e.tiles) {
          const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(p.x);
          if (idx >= outsideMask->size()) continue;
          if ((*outsideMask)[idx] == 0) return false;
        }
        return true;
      };

      // Draw edges as polylines along road tile centers.
      for (const RoadGraphEdge& e : m_roadGraph.edges) {
        const bool connected = edgeIsConnected(e);
        const Color c = connected ? Color{0, 220, 255, 140} : Color{255, 80, 80, 170};

        for (std::size_t i = 1; i < e.tiles.size(); ++i) {
          const Point& a = e.tiles[i - 1];
          const Point& b = e.tiles[i];
          const Vector2 wa = TileToWorldCenter(a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                               static_cast<float>(m_cfg.tileHeight));
          const Vector2 wb = TileToWorldCenter(b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                               static_cast<float>(m_cfg.tileHeight));
          DrawLineEx(wa, wb, thickness, c);
        }
      }

      // Draw nodes as small circles.
      for (const RoadGraphNode& n : m_roadGraph.nodes) {
        Color c = Color{255, 220, 0, 200};
        if (outsideMask && w > 0) {
          const std::size_t idx = static_cast<std::size_t>(n.pos.y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(n.pos.x);
          if (idx < outsideMask->size() && (*outsideMask)[idx] == 0) c = Color{255, 80, 80, 220};
        }

        const Vector2 wp = TileToWorldCenter(n.pos.x, n.pos.y, static_cast<float>(m_cfg.tileWidth),
                                             static_cast<float>(m_cfg.tileHeight));
        DrawCircleV(wp, radius, c);
      }

      EndMode2D();
    }
  }

  const float simSpeed = kSimSpeeds[static_cast<std::size_t>(std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1))];
  const char* inspectInfo = (m_tool == Tool::Inspect && !m_inspectInfo.empty()) ? m_inspectInfo.c_str() : nullptr;
  m_renderer.drawHUD(m_world, m_tool, m_hovered, GetScreenWidth(), GetScreenHeight(), m_showHelp, m_brushRadius,
                     static_cast<int>(m_history.undoSize()), static_cast<int>(m_history.redoSize()), m_simPaused,
                     simSpeed, m_saveSlot, inspectInfo);

  // Road-drag overlay: show preview metrics without touching the HUD layout.
  if (m_roadDragActive) {
    const int fontSize = 18;
    const int pad = 8;

    const char* line1 = nullptr;
    char buf[128];
    if (m_roadDragValid && !m_roadDragPath.empty()) {
      std::snprintf(buf, sizeof(buf), "Road path: %d tiles, %d new", static_cast<int>(m_roadDragPath.size()),
                    m_roadDragBuildCost);
      line1 = buf;
    } else {
      line1 = "Road path: no route";
    }
    const char* line2 = "Release to build";

    const int w1 = MeasureText(line1, fontSize);
    const int w2 = MeasureText(line2, fontSize);
    const int boxW = std::max(w1, w2) + pad * 2;
    const int boxH = fontSize * 2 + pad * 3;

    const int x = GetScreenWidth() - boxW - 12;
    const int y = 44;

    DrawRectangle(x, y, boxW, boxH, Color{0, 0, 0, 160});
    DrawRectangleLines(x, y, boxW, boxH, Color{255, 255, 255, 70});

    DrawText(line1, x + pad, y + pad, fontSize, RAYWHITE);
    DrawText(line2, x + pad, y + pad + fontSize + 6, fontSize, Color{220, 220, 220, 255});
  }

  // Toast / status message
  if (m_toastTimer > 0.0f && !m_toast.empty()) {
    const int fontSize = 18;
    const int pad = 8;
    const int textW = MeasureText(m_toast.c_str(), fontSize);
    const int boxW = textW + pad * 2;
    const int boxH = fontSize + pad * 2;

    const int x = (GetScreenWidth() - boxW) / 2;
    const int y = GetScreenHeight() - boxH - 18;

    DrawRectangle(x, y, boxW, boxH, Color{0, 0, 0, 170});
    DrawRectangleLines(x, y, boxW, boxH, Color{255, 255, 255, 60});
    DrawText(m_toast.c_str(), x + pad, y + pad, fontSize, RAYWHITE);
  }

  EndDrawing();
}

} // namespace isocity
