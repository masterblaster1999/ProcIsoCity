#include "isocity/Game.hpp"

#include "isocity/Random.hpp"

#include <algorithm>

namespace isocity {

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

void Game::resetWorld(std::uint64_t newSeed)
{
  if (newSeed == 0) newSeed = TimeSeed();

  m_cfg.seed = newSeed;
  m_world = GenerateWorld(m_cfg.mapWidth, m_cfg.mapHeight, newSeed, m_procCfg);

  // Optional: vary procedural textures per seed (still no assets-from-disk).
  m_renderer.rebuildTextures(newSeed);

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

  // Toggle UI
  if (IsKeyPressed(KEY_H)) m_showHelp = !m_showHelp;
  if (IsKeyPressed(KEY_G)) m_drawGrid = !m_drawGrid;

  // Regenerate
  if (IsKeyPressed(KEY_R)) resetWorld(TimeSeed());

  // Tool selection
  if (IsKeyPressed(KEY_Q)) m_tool = Tool::Inspect;
  if (IsKeyPressed(KEY_ONE)) m_tool = Tool::Road;
  if (IsKeyPressed(KEY_TWO)) m_tool = Tool::Residential;
  if (IsKeyPressed(KEY_THREE)) m_tool = Tool::Commercial;
  if (IsKeyPressed(KEY_FOUR)) m_tool = Tool::Industrial;
  if (IsKeyPressed(KEY_FIVE)) m_tool = Tool::Park;
  if (IsKeyPressed(KEY_ZERO)) m_tool = Tool::Bulldoze;

  // Camera pan: right mouse drag (raylib example style).
  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
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
  if (m_hovered && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    // Avoid painting when you're currently panning with RMB.
    if (!IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      m_world.applyTool(m_tool, m_hovered->x, m_hovered->y);
    }
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

void Game::update(float dt) { m_sim.update(m_world, dt); }

void Game::draw()
{
  BeginDrawing();
  ClearBackground(Color{30, 32, 38, 255});

  m_renderer.drawWorld(m_world, m_camera, m_timeSec, m_hovered, m_drawGrid);
  m_renderer.drawHUD(m_world, m_tool, m_hovered, GetScreenWidth(), GetScreenHeight(), m_showHelp);

  EndDrawing();
}

} // namespace isocity
