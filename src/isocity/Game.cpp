#include "isocity/Game.hpp"

#include "isocity/Random.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>

namespace isocity {

namespace {
constexpr const char* kQuickSavePath = "isocity_save.bin";

// Discrete sim speed presets (dt multiplier).
constexpr float kSimSpeeds[] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f};
constexpr int kSimSpeedCount = static_cast<int>(sizeof(kSimSpeeds) / sizeof(kSimSpeeds[0]));
} // namespace

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

  auto noteRoadNeighborhood = [&](int tx, int ty) {
    m_history.noteTilePreEdit(m_world, tx, ty);
    m_history.noteTilePreEdit(m_world, tx, ty - 1);
    m_history.noteTilePreEdit(m_world, tx + 1, ty);
    m_history.noteTilePreEdit(m_world, tx, ty + 1);
    m_history.noteTilePreEdit(m_world, tx - 1, ty);
  };

  const int r = std::max(0, m_brushRadius);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      // Diamond brush (fits iso grid nicely).
      if (std::abs(dx) + std::abs(dy) > r) continue;
      const int tx = centerX + dx;
      const int ty = centerY + dy;

      // Road edits update neighbor road masks for auto-tiling. Capture the neighborhood
      // so undo/redo can restore variations without global recompute.
      const bool affectsRoadMasks =
          (m_tool == Tool::Road) ||
          ((m_tool == Tool::Bulldoze || m_tool == Tool::Park) && m_world.inBounds(tx, ty) &&
           (m_world.at(tx, ty).overlay == Overlay::Road));

      if (affectsRoadMasks) {
        noteRoadNeighborhood(tx, ty);
      } else {
        m_history.noteTilePreEdit(m_world, tx, ty);
      }

      m_world.applyTool(m_tool, tx, ty);
    }
  }
}

void Game::beginPaintStroke()
{
  if (m_painting) return;
  m_painting = true;
  m_history.beginStroke(m_world);
}

void Game::endPaintStroke()
{
  if (!m_painting) return;
  m_painting = false;
  m_history.endStroke(m_world);

  // Keep HUD numbers (roads/parks/capacities) responsive even before the next sim tick.
  m_sim.refreshDerivedStats(m_world);
}

void Game::doUndo()
{
  // Commit any in-progress stroke before undoing.
  endPaintStroke();

  if (m_history.undo(m_world)) {
    m_sim.refreshDerivedStats(m_world);
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

  // New world invalidates history.
  m_history.clear();
  m_painting = false;

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

  // Brush radius
  if (IsKeyPressed(KEY_LEFT_BRACKET)) {
    m_brushRadius = std::max(0, m_brushRadius - 1);
    showToast(TextFormat("Brush radius: %d", m_brushRadius));
  }
  if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
    m_brushRadius = std::min(8, m_brushRadius + 1);
    showToast(TextFormat("Brush radius: %d", m_brushRadius));
  }

  // Save / Load (quick save)
  if (IsKeyPressed(KEY_F5)) {
    std::string err;
    if (SaveWorldBinary(m_world, m_procCfg, kQuickSavePath, err)) {
      showToast(std::string("Saved: ") + kQuickSavePath);
    } else {
      showToast(std::string("Save failed: ") + err, 4.0f);
    }
  }

  if (IsKeyPressed(KEY_F9)) {
    endPaintStroke();
    std::string err;
    World loaded;
    ProcGenConfig loadedProcCfg{};
    if (LoadWorldBinary(loaded, loadedProcCfg, kQuickSavePath, err)) {
      m_world = std::move(loaded);
      m_procCfg = loadedProcCfg;

      // Loading invalidates history.
      m_history.clear();
      m_painting = false;

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

      showToast(std::string("Loaded: ") + kQuickSavePath);
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

  if (leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    beginPaintStroke();
  }

  if (leftDown && m_painting && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    applyToolBrush(m_hovered->x, m_hovered->y);
  }

  if (leftReleased) {
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
    const int si = std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1);
    const float speed = kSimSpeeds[static_cast<std::size_t>(si)];
    m_sim.update(m_world, dt * speed);
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

  m_renderer.drawWorld(m_world, m_camera, m_timeSec, m_hovered, m_drawGrid, m_brushRadius);
  m_renderer.drawHUD(m_world, m_tool, m_hovered, GetScreenWidth(), GetScreenHeight(), m_showHelp, m_brushRadius,
                     static_cast<int>(m_history.undoSize()), static_cast<int>(m_history.redoSize()), m_simPaused,
                     kSimSpeeds[static_cast<std::size_t>(std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1))]);

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
