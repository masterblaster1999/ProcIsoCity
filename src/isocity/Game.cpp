#include "isocity/Game.hpp"

#include "isocity/DistrictStats.hpp"
#include "isocity/Export.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/Road.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace isocity {

namespace {
// Slot 1 uses the legacy filename so existing quick-saves keep working.
constexpr const char* kLegacyQuickSavePath = "isocity_save.bin";
constexpr int kSaveSlotMin = 1;
constexpr int kSaveSlotMax = 5;

// Autosaves rotate through a separate set of slots.
constexpr int kAutosaveSlotMin = 1;
constexpr int kAutosaveSlotMax = 3;
constexpr float kAutosaveIntervalSec = 60.0f;

// --- Vehicle micro-sim tuning ---
constexpr int kMaxCommuteVehicles = 160;
constexpr int kMaxGoodsVehicles = 120;
constexpr int kCommutersPerCar = 40; // how many commuters one visible car represents
constexpr int kGoodsPerTruck = 80;   // goods units represented by one visible truck
constexpr int kMaxSpawnPerFrame = 2;

std::string FileTimestamp()
{
  using namespace std::chrono;

  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::time_t tt = system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif

  char base[32]{};
  std::strftime(base, sizeof(base), "%Y%m%d_%H%M%S", &tm);

  char out[40]{};
  std::snprintf(out, sizeof(out), "%s_%03d", base, static_cast<int>(ms.count()));
  return std::string(out);
}

inline float Rand01(std::uint64_t& state)
{
  // 24-bit mantissa float in [0,1)
  const std::uint64_t u = SplitMix64Next(state);
  return static_cast<float>((u >> 40) & 0x00FFFFFFu) / 16777216.0f;
}

inline float RandRange(std::uint64_t& state, float a, float b)
{
  return a + (b - a) * Rand01(state);
}

// Discrete sim speed presets (dt multiplier).
constexpr float kSimSpeeds[] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f};
constexpr int kSimSpeedCount = static_cast<int>(sizeof(kSimSpeeds) / sizeof(kSimSpeeds[0]));

// World render scaling (resolution scale) helpers.
constexpr float kWorldRenderScaleStep = 0.05f;
constexpr float kWorldRenderScaleAbsMin = 0.25f;
constexpr float kWorldRenderScaleAbsMax = 2.0f;
constexpr float kWorldRenderAutoAdjustInterval = 0.35f; // seconds
constexpr float kWorldRenderDtSmoothing = 0.10f;        // EMA factor
constexpr int kWorldRenderRTMaxDim = 8192;              // safety guard
} // namespace

std::string Game::savePathForSlot(int slot) const
{
  const int s = std::clamp(slot, kSaveSlotMin, kSaveSlotMax);
  if (s == 1) return std::string(kLegacyQuickSavePath);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "isocity_save_slot%d.bin", s);
  return std::string(buf);
}

std::string Game::autosavePathForSlot(int slot) const
{
  const int s = std::clamp(slot, kAutosaveSlotMin, kAutosaveSlotMax);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "isocity_autosave_slot%d.bin", s);
  return std::string(buf);
}

std::string Game::thumbPathForSavePath(const std::string& savePath) const
{
  // Convert "*.bin" -> "*.png" (thumbnail image).
  std::string p = savePath;
  const std::size_t dot = p.find_last_of('.');
  if (dot != std::string::npos) {
    p = p.substr(0, dot);
  }
  p += ".png";
  return p;
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

bool Game::saveToPath(const std::string& path, bool makeThumbnail, const char* toastLabel)
{
  endPaintStroke();

  std::string err;
  if (!SaveWorldBinary(m_world, m_procCfg, m_sim.config(), path, err)) {
    showToast(std::string("Save failed: ") + err, 4.0f);
    return false;
  }

  if (makeThumbnail) {
    const std::string thumb = thumbPathForSavePath(path);
    // Best effort: do not fail the save if thumbnail export fails.
    (void)m_renderer.exportMinimapThumbnail(m_world, thumb.c_str(), 256);
  }

  if (toastLabel) {
    showToast(TextFormat("Saved: %s", toastLabel));
  } else {
    showToast(TextFormat("Saved: %s", path.c_str()));
  }

  // If the slot browser is open, refresh it so metadata/thumbnails update.
  if (m_showSaveMenu) refreshSaveMenu();
  return true;
}

bool Game::loadFromPath(const std::string& path, const char* toastLabel)
{
  endPaintStroke();

  std::string err;
  World loaded;
  ProcGenConfig loadedProcCfg{};
  SimConfig loadedSimCfg{};

  if (!LoadWorldBinary(loaded, loadedProcCfg, loadedSimCfg, path, err)) {
    showToast(std::string("Load failed: ") + err, 4.0f);
    return false;
  }

  m_world = std::move(loaded);
  m_procCfg = loadedProcCfg;
  m_sim.config() = loadedSimCfg;
  m_sim.resetTimer();

  m_renderer.markMinimapDirty();
  m_renderer.markBaseCacheDirtyAll();
  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_landValueDirty = true;
  m_vehiclesDirty = true;
  m_vehicles.clear();

  // Deterministic vehicle RNG seed per world seed.
  m_vehicleRngState = (m_world.seed() ^ 0x9E3779B97F4A7C15ULL);

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
  m_roadDragUpgradeTiles = 0;
  m_roadDragBridgeTiles = 0;
  m_roadDragMoneyCost = 0;
  m_roadDragValid = false;

  // Keep config in sync with loaded world, so regen & camera recenter behave.
  m_cfg.mapWidth = m_world.width();
  m_cfg.mapHeight = m_world.height();
  m_cfg.seed = m_world.seed();

  m_renderer.rebuildTextures(m_cfg.seed);
  SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(m_cfg.seed)));

  // Recenter camera on loaded map.
  m_camera.target = TileToWorldCenterElevated(m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2,
                                              static_cast<float>(m_cfg.tileWidth),
                                              static_cast<float>(m_cfg.tileHeight), m_elev);

  m_sim.refreshDerivedStats(m_world);
  clearHistory();
  recordHistorySample(m_world.stats());

  if (toastLabel) {
    showToast(TextFormat("Loaded: %s", toastLabel));
  } else {
    showToast(TextFormat("Loaded: %s", path.c_str()));
  }

  if (m_showSaveMenu) refreshSaveMenu();
  return true;
}

RaylibContext::RaylibContext(const Config& cfg, const char* title)
{
  unsigned int flags = 0;
  if (cfg.vsync) flags |= FLAG_VSYNC_HINT;
  if (cfg.windowResizable) flags |= FLAG_WINDOW_RESIZABLE;
  if (cfg.windowHighDPI) flags |= FLAG_WINDOW_HIGHDPI;
  SetConfigFlags(flags);

  InitWindow(cfg.windowWidth, cfg.windowHeight, title);

  if (cfg.windowResizable) {
    SetWindowMinSize(std::max(1, cfg.windowMinWidth), std::max(1, cfg.windowMinHeight));
  }

  // You can tune this later or expose it as a config.
  SetTargetFPS(60);

  // Ensure vsync state matches config at runtime.
  if (cfg.vsync) {
    SetWindowState(FLAG_VSYNC_HINT);
  } else {
    ClearWindowState(FLAG_VSYNC_HINT);
  }
}

RaylibContext::~RaylibContext() { CloseWindow(); }

Game::~Game()
{
  unloadWorldRenderTarget();
  unloadSaveMenuThumbnails();
}

Game::Game(Config cfg)
    : m_cfg(cfg)
    , m_rl(cfg, "ProcIsoCity")
    , m_world()
    , m_sim(SimConfig{})
    , m_renderer(cfg.tileWidth, cfg.tileHeight, cfg.seed)
{
  // Prevent accidental Alt+F4 style exits while testing.
  SetExitKey(KEY_NULL);

  // Track the initial window geometry so fullscreen/borderless toggles can
  // restore back to the original windowed size/position.
  {
    const Vector2 pos = GetWindowPosition();
    m_windowedX = static_cast<int>(pos.x);
    m_windowedY = static_cast<int>(pos.y);
    m_windowedW = GetScreenWidth();
    m_windowedH = GetScreenHeight();
  }

  // Initialize UI scaling.
  if (m_uiScaleAuto) {
    m_uiScale = computeAutoUiScale(m_windowedW, m_windowedH);
  }

  // Initialize world render scaling (resolution scale) from config.
  m_worldRenderScaleAuto = m_cfg.worldRenderScaleAuto;
  m_worldRenderScale = clampWorldRenderScale(m_cfg.worldRenderScale);
  m_worldRenderScaleMin = clampWorldRenderScale(m_cfg.worldRenderScaleMin);
  m_worldRenderScaleMax = clampWorldRenderScale(m_cfg.worldRenderScaleMax);
  if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
    std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
  }
  m_worldRenderTargetFps = std::max(15, m_cfg.worldRenderTargetFps);
  m_worldRenderFilterPoint = m_cfg.worldRenderFilterPoint;
  if (m_worldRenderScaleAuto) {
    // Prefer best quality first; let the auto-scaler reduce resolution only
    // if we can't hit the target FPS.
    m_worldRenderScale = m_worldRenderScaleMax;
  }

  // Elevation settings derived from config.
  m_elevDefault.maxPixels = static_cast<float>(m_cfg.tileHeight) * std::max(0.0f, m_cfg.elevationScale);
  m_elevDefault.quantizeSteps = std::max(0, m_cfg.elevationSteps);
  m_elevDefault.flattenWater = true;
  m_elev = m_elevDefault;
  m_renderer.setElevationSettings(m_elev);

  resetWorld(m_cfg.seed);

  // Camera
  m_camera.zoom = 1.0f;
  m_camera.rotation = 0.0f;
  m_camera.offset = Vector2{m_windowedW * 0.5f, m_windowedH * 0.5f};

  const Vector2 center = TileToWorldCenterElevated(
      m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2, static_cast<float>(m_cfg.tileWidth),
      static_cast<float>(m_cfg.tileHeight), m_elev);
  m_camera.target = center;

  setupDevConsole();
}

void Game::setupDevConsole()
{
  // Keep the console usable in Release builds: it is primarily a dev/debug
  // productivity tool, but also enables power-users to script common actions.
  m_console.clearLog();
  m_console.print("ProcIsoCity dev console (F4). Type 'help' for commands.");

  auto toLower = [](std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return s;
  };

  auto joinArgs = [](const DevConsole::Args& args, std::size_t start) {
    std::string out;
    for (std::size_t i = start; i < args.size(); ++i) {
      if (!out.empty()) out.push_back(' ');
      out += args[i];
    }
    return out;
  };

  auto parseI64 = [](const std::string& s, long long& out) -> bool {
    try {
      std::size_t idx = 0;
      out = std::stoll(s, &idx, 10);
      return idx == s.size();
    } catch (...) {
      return false;
    }
  };

  auto parseU64 = [](const std::string& s, std::uint64_t& out) -> bool {
    try {
      std::size_t idx = 0;
      const unsigned long long v = std::stoull(s, &idx, 10);
      if (idx != s.size()) return false;
      out = static_cast<std::uint64_t>(v);
      return true;
    } catch (...) {
      return false;
    }
  };

  auto parseF32 = [](const std::string& s, float& out) -> bool {
    try {
      std::size_t idx = 0;
      out = std::stof(s, &idx);
      return idx == s.size();
    } catch (...) {
      return false;
    }
  };

  auto heatmapName = [](HeatmapOverlay h) -> const char* {
    switch (h) {
    case HeatmapOverlay::Off: return "off";
    case HeatmapOverlay::LandValue: return "land";
    case HeatmapOverlay::ParkAmenity: return "park";
    case HeatmapOverlay::WaterAmenity: return "water";
    case HeatmapOverlay::Pollution: return "pollution";
    case HeatmapOverlay::TrafficSpill: return "traffic";
    default: return "?";
    }
  };

  // --- help/utility ---
  m_console.registerCommand(
      "help", "help [cmd]  - list commands or show help for one command",
      [toLower](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          const std::string key = toLower(args[0]);
          const auto it = c.commands().find(key);
          if (it == c.commands().end()) {
            c.print("Unknown command: " + args[0]);
            return;
          }
          c.print(it->first + "  - " + it->second.help);
          return;
        }

        c.print("Commands:");
        for (const std::string& name : c.commandOrder()) {
          const std::string key = toLower(name);
          const auto it = c.commands().find(key);
          if (it != c.commands().end()) {
            c.print("  " + name + "  - " + it->second.help);
          }
        }
      });

  m_console.registerCommand("clear", "clear      - clear the console output",
                            [](DevConsole& c, const DevConsole::Args&) { c.clearLog(); });

  m_console.registerCommand(
      "echo", "echo <text...>  - print text", [joinArgs](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) return;
        c.print(joinArgs(args, 0));
      });

  // --- world/simulation ---
  m_console.registerCommand(
      "seed", "seed <uint64>  - regenerate the world with a specific seed",
      [this, parseU64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: seed <uint64>");
          return;
        }
        std::uint64_t s = 0;
        if (!parseU64(args[0], s)) {
          c.print("Invalid seed: " + args[0]);
          return;
        }
        endPaintStroke();
        resetWorld(s);
        showToast(TextFormat("Seed: %llu", static_cast<unsigned long long>(s)));
        c.print(TextFormat("World regenerated with seed %llu", static_cast<unsigned long long>(s)));
      });

  m_console.registerCommand("regen", "regen        - regenerate the world with a time-based seed",
                            [this](DevConsole& c, const DevConsole::Args&) {
                              endPaintStroke();
                              resetWorld(0);
                              c.print("World regenerated.");
                            });

  m_console.registerCommand(
      "pause", "pause        - toggle simulation pause", [this](DevConsole& c, const DevConsole::Args&) {
        endPaintStroke();
        m_simPaused = !m_simPaused;
        m_sim.resetTimer();
        showToast(m_simPaused ? "Sim paused" : "Sim running");
        c.print(m_simPaused ? "paused" : "running");
      });

  m_console.registerCommand(
      "step", "step         - advance the simulation by one day (like 'N' while paused)",
      [this](DevConsole& c, const DevConsole::Args&) {
        endPaintStroke();
        m_sim.stepOnce(m_world);
        recordHistorySample(m_world.stats());
        m_trafficDirty = true;
        m_goodsDirty = true;
        m_landValueDirty = true;
        m_vehiclesDirty = true;
        showToast("Sim step");
        c.print("stepped");
      });

  m_console.registerCommand(
      "speed", "speed <multiplier>  - set sim speed (e.g. 0.5, 1, 2, 4, 8)",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: speed <multiplier>");
          return;
        }
        float sp = 1.0f;
        if (!parseF32(args[0], sp)) {
          c.print("Invalid speed: " + args[0]);
          return;
        }

        // Pick nearest pre-defined speed.
        int best = 0;
        float bestDist = std::abs(kSimSpeeds[0] - sp);
        for (int i = 1; i < kSimSpeedCount; ++i) {
          const float d = std::abs(kSimSpeeds[i] - sp);
          if (d < bestDist) {
            bestDist = d;
            best = i;
          }
        }
        m_simSpeedIndex = best;
        showToast(TextFormat("Sim speed: x%.2f", static_cast<double>(kSimSpeeds[best])));
        c.print(TextFormat("sim speed set to x%.2f", static_cast<double>(kSimSpeeds[best])));
      });

  m_console.registerCommand(
      "money", "money <amount>  - set current money", [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: money <amount>");
          return;
        }
        long long v = 0;
        if (!parseI64(args[0], v)) {
          c.print("Invalid amount: " + args[0]);
          return;
        }
        m_world.stats().money = static_cast<int>(v);
        showToast(TextFormat("Money: %d", m_world.stats().money));
        c.print(TextFormat("money = %d", m_world.stats().money));
      });

  m_console.registerCommand(
      "give", "give <amount>   - add money", [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: give <amount>");
          return;
        }
        long long v = 0;
        if (!parseI64(args[0], v)) {
          c.print("Invalid amount: " + args[0]);
          return;
        }
        m_world.stats().money += static_cast<int>(v);
        showToast(TextFormat("Money: %d", m_world.stats().money));
        c.print(TextFormat("money = %d", m_world.stats().money));
      });

  // --- tools/rendering ---
  m_console.registerCommand(
      "tool",
      "tool <road|res|com|ind|park|bulldoze|inspect|raise|lower|smooth|district>  - select tool",
      [this, toLower, heatmapName](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: tool <name>");
          return;
        }
        const std::string t = toLower(args[0]);
        Tool newTool = m_tool;
        if (t == "road") newTool = Tool::Road;
        else if (t == "res" || t == "residential") newTool = Tool::Residential;
        else if (t == "com" || t == "commercial") newTool = Tool::Commercial;
        else if (t == "ind" || t == "industrial") newTool = Tool::Industrial;
        else if (t == "park") newTool = Tool::Park;
        else if (t == "bulldoze" || t == "doze" || t == "delete") newTool = Tool::Bulldoze;
        else if (t == "inspect") newTool = Tool::Inspect;
        else if (t == "raise") newTool = Tool::RaiseTerrain;
        else if (t == "lower") newTool = Tool::LowerTerrain;
        else if (t == "smooth") newTool = Tool::SmoothTerrain;
        else if (t == "district") newTool = Tool::District;
        else {
          c.print("Unknown tool: " + args[0]);
          return;
        }

        endPaintStroke();
        m_tool = newTool;
        // Cancel any road drag preview if we changed tools.
        if (m_tool != Tool::Road) {
          m_roadDragActive = false;
          m_roadDragStart.reset();
          m_roadDragEnd.reset();
          m_roadDragPath.clear();
          m_roadDragBuildCost = 0;
          m_roadDragUpgradeTiles = 0;
          m_roadDragBridgeTiles = 0;
          m_roadDragMoneyCost = 0;
          m_roadDragValid = false;
        }
        showToast(TextFormat("Tool: %s", ToString(m_tool)));
        c.print(TextFormat("tool = %s", ToString(m_tool)));
      });

  m_console.registerCommand(
      "brush", "brush <0..8>   - set brush radius (diamond)",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: brush <0..8>");
          return;
        }
        long long r = 0;
        if (!parseI64(args[0], r)) {
          c.print("Invalid radius: " + args[0]);
          return;
        }
        m_brushRadius = std::clamp(static_cast<int>(r), 0, 8);
        showToast(TextFormat("Brush radius: %d", m_brushRadius));
        c.print(TextFormat("brush = %d", m_brushRadius));
      });

  m_console.registerCommand(
      "roadlevel", "roadlevel <1..3> - set road build level", [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: roadlevel <1..3>");
          return;
        }
        long long lv = 0;
        if (!parseI64(args[0], lv)) {
          c.print("Invalid level: " + args[0]);
          return;
        }
        m_roadBuildLevel = std::clamp(static_cast<int>(lv), 1, 3);
        showToast(TextFormat("Road type: %s", RoadClassName(m_roadBuildLevel)));
        c.print(TextFormat("roadlevel = %d", m_roadBuildLevel));
      });

  m_console.registerCommand(
      "heatmap", "heatmap <off|land|park|water|pollution|traffic> - set heatmap overlay",
      [this, toLower, heatmapName](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: heatmap <off|land|park|water|pollution|traffic>");
          return;
        }
        const std::string h = toLower(args[0]);
        if (h == "off") m_heatmapOverlay = HeatmapOverlay::Off;
        else if (h == "land") m_heatmapOverlay = HeatmapOverlay::LandValue;
        else if (h == "park") m_heatmapOverlay = HeatmapOverlay::ParkAmenity;
        else if (h == "water") m_heatmapOverlay = HeatmapOverlay::WaterAmenity;
        else if (h == "pollution") m_heatmapOverlay = HeatmapOverlay::Pollution;
        else if (h == "traffic") m_heatmapOverlay = HeatmapOverlay::TrafficSpill;
        else {
          c.print("Unknown heatmap: " + args[0]);
          return;
        }
        m_landValueDirty = true;
        showToast(TextFormat("Heatmap: %s", heatmapName(m_heatmapOverlay)));
        c.print(TextFormat("heatmap = %s", heatmapName(m_heatmapOverlay)));
      });

  m_console.registerCommand(
      "overlay",
      "overlay <minimap|vehicles|traffic|goods|outside|help|policy|report|cache|traffic_model> [on|off|toggle]",
      [this, toLower](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print("Usage: overlay <name> [on|off|toggle]");
          return;
        }

        const std::string name = toLower(args[0]);
        const std::string mode = (args.size() >= 2) ? toLower(args[1]) : "toggle";

        auto want = [&](bool current) {
          if (mode == "on" || mode == "1" || mode == "true") return true;
          if (mode == "off" || mode == "0" || mode == "false") return false;
          return !current;
        };

        if (name == "minimap") {
          m_showMinimap = want(m_showMinimap);
          showToast(m_showMinimap ? "Minimap: ON" : "Minimap: OFF");
        } else if (name == "vehicles") {
          m_showVehicles = want(m_showVehicles);
          showToast(m_showVehicles ? "Vehicles: ON" : "Vehicles: OFF");
        } else if (name == "traffic") {
          m_showTrafficOverlay = want(m_showTrafficOverlay);
          showToast(m_showTrafficOverlay ? "Traffic overlay: ON" : "Traffic overlay: OFF");
        } else if (name == "goods") {
          m_showGoodsOverlay = want(m_showGoodsOverlay);
          showToast(m_showGoodsOverlay ? "Goods overlay: ON" : "Goods overlay: OFF");
        } else if (name == "outside") {
          m_showOutsideOverlay = want(m_showOutsideOverlay);
          showToast(m_showOutsideOverlay ? "Outside overlay: ON" : "Outside overlay: OFF");
        } else if (name == "help") {
          m_showHelp = want(m_showHelp);
          showToast(m_showHelp ? "Help: ON" : "Help: OFF");
        } else if (name == "policy" || name == "policies") {
          m_showPolicy = want(m_showPolicy);
          showToast(m_showPolicy ? "Policy panel: ON" : "Policy panel: OFF");
        } else if (name == "report") {
          m_showReport = want(m_showReport);
          showToast(m_showReport ? "City report: ON" : "City report: OFF");
        } else if (name == "traffic_model") {
          m_showTrafficModel = want(m_showTrafficModel);
          showToast(m_showTrafficModel ? "Traffic model: ON" : "Traffic model: OFF");
        } else if (name == "cache") {
          const bool enabled = want(m_renderer.baseCacheEnabled());
          m_renderer.setBaseCacheEnabled(enabled);
          m_renderer.markBaseCacheDirtyAll();
          showToast(enabled ? "Render cache: ON" : "Render cache: OFF");
        } else {
          c.print("Unknown overlay: " + args[0]);
          return;
        }
        c.print("ok");
      });

  // --- file export ---
  m_console.registerCommand(
      "shot", "shot          - capture a screenshot to captures/ (same as F12)",
      [this](DevConsole& c, const DevConsole::Args&) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");
        const std::string path =
            TextFormat("captures/screenshot_seed%llu_%s.png", static_cast<unsigned long long>(m_cfg.seed),
                       FileTimestamp().c_str());
        m_pendingScreenshot = true;
        m_pendingScreenshotPath = path;
        showToast(TextFormat("Queued screenshot: %s", path.c_str()), 2.0f);
        c.print("queued: " + path);
      });

  m_console.registerCommand(
      "map", "map [maxSize] [path] - export a world overview PNG to captures/ (queued)",
      [this, parseI64, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        // Defaults
        int maxSize = 4096;
        std::string path =
            TextFormat("captures/map_seed%llu_%s.png", static_cast<unsigned long long>(m_cfg.seed), FileTimestamp().c_str());

        auto clampSize = [](long long v) {
          // Keep this sane; exportWorldOverview may allocate a large render texture.
          return std::clamp(static_cast<int>(v), 64, 16384);
        };

        if (!args.empty()) {
          long long v = 0;
          // Allow either:
          //   map 4096
          //   map 4096 my.png
          //   map my.png
          //   map my.png 4096
          if (parseI64(args[0], v)) {
            maxSize = clampSize(v);
            if (args.size() >= 2) {
              path = joinArgs(args, 1);
            }
          } else {
            // Path first.
            path = joinArgs(args, 0);

            // If the last token is a number, treat it as maxSize.
            if (args.size() >= 2 && parseI64(args.back(), v)) {
              maxSize = clampSize(v);
              std::string p;
              for (std::size_t i = 0; i + 1 < args.size(); ++i) {
                if (!p.empty()) p.push_back(' ');
                p += args[i];
              }
              if (!p.empty()) path = p;
            }
          }
        }

        if (path.empty()) {
          c.print("Usage: map [maxSize] [path]");
          return;
        }

        m_pendingMapExport = true;
        m_pendingMapExportPath = path;
        m_pendingMapExportMaxSize = maxSize;
        showToast(TextFormat("Queued map export (%dpx): %s", maxSize, path.c_str()), 2.0f);
        c.print(TextFormat("queued: %s (maxSize=%d)", path.c_str(), maxSize));
      });

  m_console.registerCommand(
      "tiles_csv",
      "tiles_csv [path] - export per-tile world data to CSV (x,y,terrain,overlay,level,district,height,variation,occupants)",
      [this, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;

        const std::string path = args.empty()
                                    ? TextFormat("captures/tiles_seed%llu_%s.csv",
                                                 static_cast<unsigned long long>(m_cfg.seed),
                                                 FileTimestamp().c_str())
                                    : joinArgs(args, 0);

        if (path.empty()) {
          c.print("Usage: tiles_csv [path]");
          return;
        }

        // Create parent directories if needed.
        std::error_code ec;
        const fs::path p(path);
        if (!p.parent_path().empty()) {
          fs::create_directories(p.parent_path(), ec);
        }

        std::string err;
        if (!WriteTilesCsv(m_world, path, err)) {
          c.print("Failed to write tiles CSV: " + path + (err.empty() ? "" : (" (" + err + ")")));
          showToast("Tiles CSV export failed", 2.5f);
          return;
        }

        showToast(TextFormat("Exported tiles CSV: %s", path.c_str()), 2.0f);
        c.print("wrote: " + path);
      });

  m_console.registerCommand(
      "report_csv", "report_csv [path] - export city history samples to CSV",
      [this, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        const std::string path = args.empty()
                                    ? TextFormat("captures/report_seed%llu_%s.csv",
                                                 static_cast<unsigned long long>(m_cfg.seed),
                                                 FileTimestamp().c_str())
                                    : joinArgs(args, 0);

        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out) {
          c.print("Failed to write: " + path);
          return;
        }

        out << "day,population,money,happiness,demandResidential,avgLandValue,avgTaxPerCapita,income,expenses,taxRevenue,maintenanceCost,commuters,avgCommute,avgCommuteTime,trafficCongestion,goodsSatisfaction\n";
        for (const CityHistorySample& s : m_cityHistory) {
          out << s.day << ',' << s.population << ',' << s.money << ',' << s.happiness << ','
              << s.demandResidential << ',' << s.avgLandValue << ',' << s.avgTaxPerCapita << ',' << s.income << ','
              << s.expenses << ',' << s.taxRevenue << ',' << s.maintenanceCost << ',' << s.commuters << ','
              << s.avgCommute << ',' << s.avgCommuteTime << ',' << s.trafficCongestion << ','
              << s.goodsSatisfaction << '\n';
        }
        out.close();

        showToast(TextFormat("Exported report CSV: %s", path.c_str()), 2.0f);
        c.print("wrote: " + path);
      });

  // --- camera ---
  m_console.registerCommand(
      "goto", "goto <x> <y>   - center camera on tile coordinates",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 2) {
          c.print("Usage: goto <x> <y>");
          return;
        }
        long long x = 0;
        long long y = 0;
        if (!parseI64(args[0], x) || !parseI64(args[1], y)) {
          c.print("Invalid coordinates");
          return;
        }
        const int tx = std::clamp(static_cast<int>(x), 0, m_cfg.mapWidth - 1);
        const int ty = std::clamp(static_cast<int>(y), 0, m_cfg.mapHeight - 1);
        m_camera.target = TileToWorldCenterElevated(m_world, tx, ty, static_cast<float>(m_cfg.tileWidth),
                                                    static_cast<float>(m_cfg.tileHeight), m_elev);
        showToast(TextFormat("Camera -> (%d,%d)", tx, ty), 1.5f);
        c.print(TextFormat("camera centered on (%d,%d)", tx, ty));
      });

  m_console.registerCommand(
      "zoom", "zoom <0.25..4.0> - set camera zoom",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: zoom <value>");
          return;
        }
        float z = 1.0f;
        if (!parseF32(args[0], z)) {
          c.print("Invalid zoom: " + args[0]);
          return;
        }
        m_camera.zoom = std::clamp(z, 0.25f, 4.0f);
        showToast(TextFormat("Zoom: %.2f", static_cast<double>(m_camera.zoom)), 1.5f);
        c.print(TextFormat("zoom = %.2f", static_cast<double>(m_camera.zoom)));
      });

  // --- video/ui ---
  m_console.registerCommand(
      "ui_scale", "ui_scale [auto|value] - set UI scale (0.5..4.0)",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print(TextFormat("ui_scale = %.2f (%s)", static_cast<double>(m_uiScale), m_uiScaleAuto ? "auto" : "manual"));
          return;
        }

        if (args.size() != 1) {
          c.print("Usage: ui_scale [auto|value]");
          return;
        }

        if (args[0] == "auto") {
          m_uiScaleAuto = true;
          m_uiScale = computeAutoUiScale(GetScreenWidth(), GetScreenHeight());
          showToast(TextFormat("UI scale: auto (%.2f)", static_cast<double>(m_uiScale)), 1.5f);
          c.print("ui_scale -> auto");
          return;
        }

        float s = 1.0f;
        if (!parseF32(args[0], s)) {
          c.print("Invalid scale: " + args[0]);
          return;
        }

        m_uiScaleAuto = false;
        m_uiScale = std::clamp(s, 0.5f, 4.0f);
        m_uiScaleManual = m_uiScale;
        showToast(TextFormat("UI scale: %.2f", static_cast<double>(m_uiScale)), 1.5f);
        c.print(TextFormat("ui_scale -> %.2f", static_cast<double>(m_uiScale)));
      });

  m_console.registerCommand(
      "fullscreen", "fullscreen - toggle exclusive fullscreen (F11)",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          c.print("Usage: fullscreen");
          return;
        }
        toggleFullscreen();
        c.print("toggled fullscreen");
      });

  m_console.registerCommand(
      "borderless", "borderless - toggle borderless windowed fullscreen (Alt+Enter)",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          c.print("Usage: borderless");
          return;
        }
        toggleBorderlessWindowed();
        c.print("toggled borderless windowed");
      });

  m_console.registerCommand(
      "resolution", "resolution [w h] - print or set window resolution",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print(TextFormat("window %dx%d", GetScreenWidth(), GetScreenHeight()));
          return;
        }

        if (args.size() != 2) {
          c.print("Usage: resolution <w> <h>");
          return;
        }

        if (IsWindowFullscreen()) {
          c.print("Exit fullscreen first (F11)");
          return;
        }

        long long w = 0;
        long long h = 0;
        if (!parseI64(args[0], w) || !parseI64(args[1], h)) {
          c.print("Invalid size");
          return;
        }

        const int minW = std::max(320, m_cfg.windowMinWidth);
        const int minH = std::max(240, m_cfg.windowMinHeight);
        const int ww = std::max(minW, static_cast<int>(w));
        const int hh = std::max(minH, static_cast<int>(h));
        SetWindowSize(ww, hh);
        showToast(TextFormat("Window: %dx%d", ww, hh), 1.5f);
        c.print(TextFormat("window -> %dx%d", ww, hh));
      });

  m_console.registerCommand(
      "vsync", "vsync - toggle VSync hint",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          c.print("Usage: vsync");
          return;
        }
        toggleVsync();
        c.print(TextFormat("vsync -> %s", m_cfg.vsync ? "on" : "off"));
      });

  m_console.registerCommand(
      "render_scale", "render_scale [auto|value] - set world render resolution scale",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print(TextFormat("render_scale = %.0f%% (%s)", m_worldRenderScale * 100.0f,
                             m_worldRenderScaleAuto ? "auto" : "manual"));
          if (m_worldRenderScaleAuto) {
            c.print(TextFormat("range: %.0f%%..%.0f%%  target: %dfps", m_worldRenderScaleMin * 100.0f,
                               m_worldRenderScaleMax * 100.0f, m_worldRenderTargetFps));
          }
          c.print(TextFormat("filter: %s", m_worldRenderFilterPoint ? "point" : "bilinear"));
          return;
        }

        if (args.size() != 1) {
          c.print("Usage: render_scale [auto|value]");
          return;
        }

        if (args[0] == "auto") {
          m_worldRenderScaleAuto = true;
          m_cfg.worldRenderScaleAuto = true;
          if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
            std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
          }
          m_worldRenderScale = std::clamp(m_worldRenderScaleMax, m_worldRenderScaleMin, m_worldRenderScaleMax);
          m_cfg.worldRenderScale = m_worldRenderScale;
          showToast(TextFormat("World render: auto (%.0f%%)", m_worldRenderScale * 100.0f), 1.5f);
          c.print("render_scale -> auto");
          return;
        }

        float s = 1.0f;
        if (!parseF32(args[0], s)) {
          c.print("Invalid scale: " + args[0]);
          return;
        }

        m_worldRenderScaleAuto = false;
        m_cfg.worldRenderScaleAuto = false;
        m_worldRenderScale = clampWorldRenderScale(s);
        m_cfg.worldRenderScale = m_worldRenderScale;
        showToast(TextFormat("World render scale: %.0f%%", m_worldRenderScale * 100.0f), 1.5f);
        c.print(TextFormat("render_scale -> %.0f%%", m_worldRenderScale * 100.0f));

        if (!wantsWorldRenderTarget()) {
          unloadWorldRenderTarget();
        }
      });

  m_console.registerCommand(
      "render_range", "render_range <min> <max> - set auto render-scale range",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 2) {
          c.print("Usage: render_range <min> <max>");
          return;
        }

        float mn = 0.7f;
        float mx = 1.0f;
        if (!parseF32(args[0], mn) || !parseF32(args[1], mx)) {
          c.print("Invalid range");
          return;
        }

        mn = clampWorldRenderScale(mn);
        mx = clampWorldRenderScale(mx);
        if (mn > mx) std::swap(mn, mx);

        m_worldRenderScaleMin = mn;
        m_worldRenderScaleMax = mx;
        m_cfg.worldRenderScaleMin = mn;
        m_cfg.worldRenderScaleMax = mx;

        if (m_worldRenderScaleAuto) {
          m_worldRenderScale = std::clamp(m_worldRenderScale, m_worldRenderScaleMin, m_worldRenderScaleMax);
          m_cfg.worldRenderScale = m_worldRenderScale;
        }

        showToast(TextFormat("Render range: %.0f%%..%.0f%%", mn * 100.0f, mx * 100.0f), 1.5f);
        c.print(TextFormat("render_range -> %.0f%%..%.0f%%", mn * 100.0f, mx * 100.0f));
      });

  m_console.registerCommand(
      "render_targetfps", "render_targetfps <fps> - set auto render-scale target fps",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: render_targetfps <fps>");
          return;
        }

        long long fps = 60;
        if (!parseI64(args[0], fps)) {
          c.print("Invalid fps");
          return;
        }

        m_worldRenderTargetFps = std::clamp(static_cast<int>(fps), 15, 240);
        m_cfg.worldRenderTargetFps = m_worldRenderTargetFps;
        showToast(TextFormat("Render target: %dfps", m_worldRenderTargetFps), 1.5f);
        c.print(TextFormat("render_targetfps -> %d", m_worldRenderTargetFps));
      });

  m_console.registerCommand(
      "render_filter", "render_filter <bilinear|point> - set world RT scaling filter",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: render_filter <bilinear|point>");
          return;
        }

        const std::string mode = args[0];
        if (mode == "point") {
          m_worldRenderFilterPoint = true;
        } else if (mode == "bilinear") {
          m_worldRenderFilterPoint = false;
        } else {
          c.print("Unknown filter: " + mode);
          return;
        }

        m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;

        if (m_worldRenderRTValid) {
          SetTextureFilter(m_worldRenderRT.texture,
                           m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
        }

        showToast(TextFormat("Render filter: %s", m_worldRenderFilterPoint ? "point" : "bilinear"), 1.5f);
        c.print(TextFormat("render_filter -> %s", m_worldRenderFilterPoint ? "point" : "bilinear"));
      });
}

void Game::showToast(const std::string& msg, float seconds)
{
  m_toast = msg;
  m_toastTimer = std::max(0.0f, seconds);
}

float Game::computeAutoUiScale(int /*screenW*/, int screenH) const
{
  // Use screen height as a good proxy for overall UI readability and merge it
  // with any OS-reported DPI scaling.
  const float base = static_cast<float>(screenH) / 1080.0f;
  const Vector2 dpi = GetWindowScaleDPI();
  const float dpiScale = std::max(dpi.x, dpi.y);

  float scale = std::max(base, dpiScale);

  // Snap to a sensible step to avoid jitter while resizing.
  const float step = 0.25f;
  scale = std::round(scale / step) * step;
  scale = std::clamp(scale, 0.75f, 3.0f);
  return scale;
}

Vector2 Game::mouseUiPosition(float uiScale) const
{
  const Vector2 mp = GetMousePosition();
  if (uiScale <= 0.0f) return mp;
  return Vector2{mp.x / uiScale, mp.y / uiScale};
}

void Game::updateUiScaleHotkeys()
{
  const int screenW = GetScreenWidth();
  const int screenH = GetScreenHeight();
  const float autoScale = computeAutoUiScale(screenW, screenH);

  // Keep scale up-to-date when in auto mode (no toast spam).
  if (m_uiScaleAuto) {
    m_uiScale = autoScale;
  }

  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  if (!ctrl) return;

  // Reserve Ctrl+Alt combinations for other display hotkeys.
  const bool alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
  if (alt) return;

  bool userChanged = false;

  // Ctrl+0 => back to auto scaling.
  if (IsKeyPressed(KEY_ZERO)) {
    m_uiScaleAuto = true;
    m_uiScale = autoScale;
    userChanged = true;
  }

  // Ctrl+=' / Ctrl+'-' => manual adjustment.
  // NOTE: raylib maps both '=' and '+' to KEY_EQUAL.
  if (IsKeyPressed(KEY_EQUAL)) {
    if (m_uiScaleAuto) {
      m_uiScale = autoScale;
      m_uiScaleAuto = false;
    }
    m_uiScale = std::clamp(m_uiScale + 0.10f, 0.50f, 4.00f);
    userChanged = true;
  }
  if (IsKeyPressed(KEY_MINUS)) {
    if (m_uiScaleAuto) {
      m_uiScale = autoScale;
      m_uiScaleAuto = false;
    }
    m_uiScale = std::clamp(m_uiScale - 0.10f, 0.50f, 4.00f);
    userChanged = true;
  }

  if (userChanged) {
    if (!m_uiScaleAuto) {
      m_uiScaleManual = m_uiScale;
    }
    char buf[128];
    if (m_uiScaleAuto) {
      std::snprintf(buf, sizeof(buf), "UI scale: auto (%.2fx)", m_uiScale);
    } else {
      std::snprintf(buf, sizeof(buf), "UI scale: %.2fx (Ctrl+0 for auto)", m_uiScale);
    }
    showToast(buf, 2.0f);
  }
}

float Game::clampWorldRenderScale(float scale) const
{
  if (!std::isfinite(scale)) {
    return 1.0f;
  }

  return std::clamp(scale, kWorldRenderScaleAbsMin, kWorldRenderScaleAbsMax);
}

bool Game::wantsWorldRenderTarget() const
{
  if (m_worldRenderScaleAuto) return true;
  return std::fabs(m_worldRenderScale - 1.0f) > 0.001f;
}

void Game::unloadWorldRenderTarget()
{
  if (!m_worldRenderRTValid) return;
  UnloadRenderTexture(m_worldRenderRT);
  m_worldRenderRT = {};
  m_worldRenderRTValid = false;
  m_worldRenderRTWidth = 0;
  m_worldRenderRTHeight = 0;
}

void Game::ensureWorldRenderTarget(int screenW, int screenH)
{
  if (!wantsWorldRenderTarget()) {
    unloadWorldRenderTarget();
    return;
  }

  float scale = clampWorldRenderScale(m_worldRenderScale);
  if (m_worldRenderScaleAuto) {
    const float lo = clampWorldRenderScale(m_worldRenderScaleMin);
    const float hi = clampWorldRenderScale(m_worldRenderScaleMax);
    scale = std::clamp(scale, std::min(lo, hi), std::max(lo, hi));
  }

  // Prevent absurdly large render targets on extreme resolutions.
  if (screenW > 0 && screenH > 0) {
    const float maxScaleByDim = std::min(kWorldRenderRTMaxDim / static_cast<float>(screenW),
                                        kWorldRenderRTMaxDim / static_cast<float>(screenH));
    scale = std::min(scale, maxScaleByDim);
  }

  // If we had to clamp the effective scale (for example due to max RT size),
  // keep the runtime value consistent so camera mapping stays correct.
  if (std::fabs(scale - m_worldRenderScale) > 0.0005f) {
    m_worldRenderScale = scale;
    m_cfg.worldRenderScale = scale;
  }

  const int desiredW = std::max(1, static_cast<int>(std::lround(screenW * scale)));
  const int desiredH = std::max(1, static_cast<int>(std::lround(screenH * scale)));

  if (m_worldRenderRTValid && (desiredW == m_worldRenderRTWidth) && (desiredH == m_worldRenderRTHeight)) {
    // Keep filter in sync (users can toggle it at runtime).
    SetTextureFilter(m_worldRenderRT.texture,
                     m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    return;
  }

  unloadWorldRenderTarget();
  m_worldRenderRT = LoadRenderTexture(desiredW, desiredH);
  m_worldRenderRTValid = (m_worldRenderRT.texture.id != 0);
  m_worldRenderRTWidth = desiredW;
  m_worldRenderRTHeight = desiredH;

  if (m_worldRenderRTValid) {
    SetTextureFilter(m_worldRenderRT.texture,
                     m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
  }
}

void Game::updateWorldRenderHotkeys()
{
  // Ctrl+Alt combinations are reserved for world render scaling, so they don't
  // clash with Ctrl +/- UI scaling.
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
  if (!ctrl || !alt) return;
  if (m_console.isOpen()) return;

  auto setManualScale = [&](float newScale) {
    m_worldRenderScaleAuto = false;
    m_worldRenderScale = clampWorldRenderScale(newScale);
    m_cfg.worldRenderScaleAuto = false;
    m_cfg.worldRenderScale = m_worldRenderScale;
    if (!wantsWorldRenderTarget()) {
      unloadWorldRenderTarget();
    }
    showToast(TextFormat("World scale: %.2fx", static_cast<double>(m_worldRenderScale)));
  };

  if (IsKeyPressed(KEY_EQUAL)) {
    setManualScale(m_worldRenderScale + kWorldRenderScaleStep);
  }
  if (IsKeyPressed(KEY_MINUS)) {
    setManualScale(m_worldRenderScale - kWorldRenderScaleStep);
  }
  if (IsKeyPressed(KEY_ZERO)) {
    setManualScale(1.0f);
  }
  if (IsKeyPressed(KEY_F)) {
    m_worldRenderFilterPoint = !m_worldRenderFilterPoint;
    m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;
    if (m_worldRenderRTValid) {
      SetTextureFilter(m_worldRenderRT.texture,
                       m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    }
    showToast(m_worldRenderFilterPoint ? "World filter: POINT" : "World filter: BILINEAR");
  }
  if (IsKeyPressed(KEY_A)) {
    m_worldRenderScaleAuto = !m_worldRenderScaleAuto;
    m_cfg.worldRenderScaleAuto = m_worldRenderScaleAuto;
    if (m_worldRenderScaleAuto) {
      m_worldRenderScaleMin = clampWorldRenderScale(m_worldRenderScaleMin);
      m_worldRenderScaleMax = clampWorldRenderScale(m_worldRenderScaleMax);
      if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
        std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
      }
      m_worldRenderScale = std::clamp(m_worldRenderScaleMax, m_worldRenderScaleMin, m_worldRenderScaleMax);
      m_cfg.worldRenderScale = m_worldRenderScale;
      showToast("World scale: AUTO");
    } else {
      showToast("World scale: MANUAL");
      if (!wantsWorldRenderTarget()) {
        unloadWorldRenderTarget();
      }
    }
  }
}

void Game::updateDynamicWorldRenderScale(float dt)
{
  // Exponential smoothing for stability.
  m_frameTimeSmoothed = m_frameTimeSmoothed * (1.0f - kWorldRenderDtSmoothing) + dt * kWorldRenderDtSmoothing;

  if (!m_worldRenderScaleAuto) return;

  m_worldRenderAutoTimer += dt;
  if (m_worldRenderAutoTimer < kWorldRenderAutoAdjustInterval) return;
  m_worldRenderAutoTimer = 0.0f;

  float lo = clampWorldRenderScale(m_worldRenderScaleMin);
  float hi = clampWorldRenderScale(m_worldRenderScaleMax);
  if (lo > hi) std::swap(lo, hi);
  m_worldRenderScaleMin = lo;
  m_worldRenderScaleMax = hi;

  const int targetFps = std::max(15, m_worldRenderTargetFps);
  const float targetDt = 1.0f / static_cast<float>(targetFps);

  // Hysteresis bands to prevent oscillation.
  const float tooSlow = targetDt * 1.08f;  // 8% slower than target
  const float tooFast = targetDt * 0.92f;  // 8% faster than target

  float scale = std::clamp(m_worldRenderScale, lo, hi);
  if (m_frameTimeSmoothed > tooSlow && scale > lo + 0.001f) {
    scale = std::max(lo, scale - kWorldRenderScaleStep);
  } else if (m_frameTimeSmoothed < tooFast && scale < hi - 0.001f) {
    scale = std::min(hi, scale + kWorldRenderScaleStep);
  }

  // Quantize to our step to avoid constant reallocations.
  scale = std::round(scale / kWorldRenderScaleStep) * kWorldRenderScaleStep;
  scale = std::clamp(scale, lo, hi);

  if (std::fabs(scale - m_worldRenderScale) > 0.0001f) {
    m_worldRenderScale = scale;
    m_cfg.worldRenderScale = m_worldRenderScale;
    // No toast here: it would spam while auto-scaling.
  }
}

void Game::adjustVideoSettings(int dir)
{
  const int d = (dir < 0) ? -1 : 1;

  auto toastScale = [&](float scale) {
    showToast(TextFormat("World render scale: %.0f%%", scale * 100.0f));
  };

  switch (m_videoSelection) {
    case 0: {
      toggleFullscreen();
      break;
    }
    case 1: {
      toggleBorderlessWindowed();
      showToast(m_borderlessWindowed ? "Borderless: ON" : "Borderless: OFF");
      break;
    }
    case 2: {
      toggleVsync();
      showToast(m_cfg.vsync ? "VSync: ON" : "VSync: OFF");
      break;
    }
    case 3: {
      // UI scale mode: toggle auto/manual.
      m_uiScaleAuto = !m_uiScaleAuto;
      if (m_uiScaleAuto) {
        m_uiScale = computeAutoUiScale(GetScreenWidth(), GetScreenHeight());
        showToast(TextFormat("UI scale: AUTO (%.2fx)", m_uiScale));
      } else {
        // Seed manual scale from the current value when switching out of auto,
        // so toggling doesn't unexpectedly jump to 1.0x.
        m_uiScaleManual = std::clamp(m_uiScale, 0.5f, 4.0f);
        m_uiScale = m_uiScaleManual;
        showToast(TextFormat("UI scale: %.2fx", m_uiScale));
      }
      break;
    }
    case 4: {
      // UI scale value (manual only).
      if (!m_uiScaleAuto) {
        m_uiScaleManual = std::clamp(m_uiScaleManual + d * 0.25f, 0.5f, 4.0f);
        m_uiScale = m_uiScaleManual;
        showToast(TextFormat("UI scale: %.2fx", m_uiScale));
      }
      break;
    }
    case 5: {
      // World render auto/manual.
      m_worldRenderScaleAuto = !m_worldRenderScaleAuto;
      if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
        std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
      }
      if (m_worldRenderScaleAuto) {
        m_worldRenderScale = std::clamp(m_worldRenderScaleMax, m_worldRenderScaleMin, m_worldRenderScaleMax);
        showToast("World render scale: AUTO");
      } else {
        showToast("World render scale: MANUAL");
      }
      break;
    }
    case 6: {
      // World render scale (manual).
      m_worldRenderScaleAuto = false;
      m_worldRenderScale = clampWorldRenderScale(m_worldRenderScale + d * kWorldRenderScaleStep);
      toastScale(m_worldRenderScale);
      break;
    }
    case 7: {
      // Auto min.
      m_worldRenderScaleMin = clampWorldRenderScale(m_worldRenderScaleMin + d * kWorldRenderScaleStep);
      m_worldRenderScaleMin = std::min(m_worldRenderScaleMin, m_worldRenderScaleMax);
      showToast(TextFormat("World render min: %.0f%%", m_worldRenderScaleMin * 100.0f));
      break;
    }
    case 8: {
      // Auto max.
      m_worldRenderScaleMax = clampWorldRenderScale(m_worldRenderScaleMax + d * kWorldRenderScaleStep);
      m_worldRenderScaleMax = std::max(m_worldRenderScaleMax, m_worldRenderScaleMin);
      showToast(TextFormat("World render max: %.0f%%", m_worldRenderScaleMax * 100.0f));
      break;
    }
    case 9: {
      // Auto target FPS.
      m_worldRenderTargetFps = std::clamp(m_worldRenderTargetFps + d * 5, 30, 240);
      showToast(TextFormat("World render target: %d FPS", m_worldRenderTargetFps));
      break;
    }
    case 10: {
      // Upscale filter.
      m_worldRenderFilterPoint = !m_worldRenderFilterPoint;
      if (m_worldRenderRTValid) {
        SetTextureFilter(m_worldRenderRT.texture,
                         m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
      }
      showToast(m_worldRenderFilterPoint ? "World filter: POINT" : "World filter: BILINEAR");
      break;
    }
    default:
      break;
  }

  // Keep runtime settings mirrored in config for consistency.
  m_cfg.worldRenderScaleAuto = m_worldRenderScaleAuto;
  m_cfg.worldRenderScale = m_worldRenderScale;
  m_cfg.worldRenderScaleMin = m_worldRenderScaleMin;
  m_cfg.worldRenderScaleMax = m_worldRenderScaleMax;
  m_cfg.worldRenderTargetFps = m_worldRenderTargetFps;
  m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;
}

void Game::toggleFullscreen()
{
  // If we are in borderless-windowed mode, disable it first.
  if (m_borderlessWindowed) {
    toggleBorderlessWindowed();
  }

  if (!IsWindowFullscreen()) {
    // Store current windowed geometry before entering fullscreen.
    const Vector2 pos = GetWindowPosition();
    m_windowedX = static_cast<int>(pos.x);
    m_windowedY = static_cast<int>(pos.y);
    m_windowedW = GetScreenWidth();
    m_windowedH = GetScreenHeight();
  }

  ToggleFullscreen();

  if (!IsWindowFullscreen()) {
    // Restore the previous windowed geometry.
    SetWindowSize(m_windowedW, m_windowedH);
    SetWindowPosition(m_windowedX, m_windowedY);
  }

  showToast(IsWindowFullscreen() ? "Fullscreen: on (F11)" : "Fullscreen: off (F11)", 2.0f);
}

void Game::toggleBorderlessWindowed()
{
  // Borderless windowed mode is implemented by making the window undecorated
  // and sizing it to the current monitor.
  if (IsWindowFullscreen()) {
    ToggleFullscreen();
  }

  if (!m_borderlessWindowed) {
    const Vector2 pos = GetWindowPosition();
    m_windowedX = static_cast<int>(pos.x);
    m_windowedY = static_cast<int>(pos.y);
    m_windowedW = GetScreenWidth();
    m_windowedH = GetScreenHeight();

    SetWindowState(FLAG_WINDOW_UNDECORATED);
    const int monitor = GetCurrentMonitor();
    const int mw = GetMonitorWidth(monitor);
    const int mh = GetMonitorHeight(monitor);
    SetWindowPosition(0, 0);
    SetWindowSize(mw, mh);
    m_borderlessWindowed = true;
    showToast("Borderless fullscreen: on (Alt+Enter)", 2.0f);
  } else {
    ClearWindowState(FLAG_WINDOW_UNDECORATED);
    SetWindowSize(m_windowedW, m_windowedH);
    SetWindowPosition(m_windowedX, m_windowedY);
    m_borderlessWindowed = false;
    showToast("Borderless fullscreen: off (Alt+Enter)", 2.0f);
  }
}

void Game::toggleVsync()
{
  m_cfg.vsync = !m_cfg.vsync;

  if (m_cfg.vsync) {
    SetWindowState(FLAG_VSYNC_HINT);
    showToast("VSync: on", 1.5f);
  } else {
    ClearWindowState(FLAG_VSYNC_HINT);
    showToast("VSync: off", 1.5f);
  }
}


void Game::clearHistory()
{
  m_cityHistory.clear();
}

void Game::recordHistorySample(const Stats& s)
{
  // Avoid recording duplicate days (can happen when resetting/loading).
  if (!m_cityHistory.empty() && m_cityHistory.back().day == s.day) return;

  CityHistorySample hs{};
  hs.day = s.day;
  hs.population = s.population;
  hs.money = s.money;
  hs.happiness = s.happiness;
  hs.demandResidential = s.demandResidential;
  hs.avgLandValue = s.avgLandValue;
  hs.avgTaxPerCapita = s.avgTaxPerCapita;
  hs.income = s.income;
  hs.expenses = s.expenses;
  hs.taxRevenue = s.taxRevenue;
  hs.maintenanceCost = s.maintenanceCost;
  hs.commuters = s.commuters;
  hs.avgCommute = s.avgCommute;
  hs.avgCommuteTime = s.avgCommuteTime;
  hs.trafficCongestion = s.trafficCongestion;
  hs.goodsSatisfaction = s.goodsSatisfaction;

  m_cityHistory.push_back(hs);

  // Keep a bounded history window (simple ring behavior).
  const int maxDays = std::max(16, m_cityHistoryMax);
  while (static_cast<int>(m_cityHistory.size()) > maxDays) {
    m_cityHistory.erase(m_cityHistory.begin());
  }
}

void Game::unloadSaveMenuThumbnails()
{
  auto unloadVec = [&](std::vector<SaveMenuSlot>& v) {
    for (SaveMenuSlot& e : v) {
      if (e.thumbLoaded && e.thumb.id != 0) {
        UnloadTexture(e.thumb);
      }
      e.thumb = Texture2D{};
      e.thumbLoaded = false;
    }
  };

  unloadVec(m_saveMenuManual);
  unloadVec(m_saveMenuAutos);
}

void Game::refreshSaveMenu()
{
  namespace fs = std::filesystem;

  unloadSaveMenuThumbnails();
  m_saveMenuManual.clear();
  m_saveMenuAutos.clear();

  auto ageTextForPath = [&](const std::string& path) -> std::string {
    std::error_code ec;
    const fs::file_time_type ft = fs::last_write_time(fs::path(path), ec);
    if (ec) return std::string("(unknown time)");

    const auto now = fs::file_time_type::clock::now();
    auto d = (now >= ft) ? (now - ft) : (ft - now);
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(d).count();
    if (sec < 60) return std::string("just now");
    if (sec < 3600) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%lldm ago", static_cast<long long>(sec / 60));
      return std::string(buf);
    }
    if (sec < 86400) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%lldh ago", static_cast<long long>(sec / 3600));
      return std::string(buf);
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%lldd ago", static_cast<long long>(sec / 86400));
    return std::string(buf);
  };

  auto fill = [&](std::vector<SaveMenuSlot>& out, bool autosave, int minSlot, int maxSlot,
                  auto pathForSlot) {
    for (int slot = minSlot; slot <= maxSlot; ++slot) {
      SaveMenuSlot e;
      e.slot = slot;
      e.autosave = autosave;
      e.path = pathForSlot(slot);
      e.thumbPath = thumbPathForSavePath(e.path);

      std::error_code ec;
      e.exists = fs::exists(fs::path(e.path), ec) && !ec;

      if (e.exists) {
        std::string err;
        e.summaryOk = ReadSaveSummary(e.path, e.summary, err, true);
        e.crcChecked = e.summary.crcChecked;
        e.crcOk = e.summary.crcOk;
        e.timeText = ageTextForPath(e.path);
      } else {
        e.summaryOk = false;
        e.timeText = std::string("(empty)");
      }

      // Load thumbnail if present.
      if (fs::exists(fs::path(e.thumbPath), ec) && !ec) {
        e.thumb = LoadTexture(e.thumbPath.c_str());
        e.thumbLoaded = (e.thumb.id != 0);
      }

      out.push_back(std::move(e));
    }
  };

  fill(m_saveMenuManual, false, kSaveSlotMin, kSaveSlotMax, [&](int s) { return savePathForSlot(s); });
  fill(m_saveMenuAutos, true, kAutosaveSlotMin, kAutosaveSlotMax, [&](int s) { return autosavePathForSlot(s); });

  // Clamp selection indices.
  const int manualCount = static_cast<int>(m_saveMenuManual.size());
  const int autoCount = static_cast<int>(m_saveMenuAutos.size());
  if (m_saveMenuGroup == 0) {
    m_saveMenuSelection = std::clamp(m_saveMenuSelection, 0, std::max(0, manualCount - 1));
  } else {
    m_saveMenuSelection = std::clamp(m_saveMenuSelection, 0, std::max(0, autoCount - 1));
  }
}

void Game::drawSaveMenuPanel(int screenW, int screenH)
{
  if (!m_showSaveMenu) return;

  const bool showDistrictOverlay = m_showDistrictOverlay || m_showDistrictPanel || (m_tool == Tool::District);
  const int highlightDistrict = showDistrictOverlay ? (m_activeDistrict % kDistrictCount) : -1;
  const bool showDistrictBorders = showDistrictOverlay && m_showDistrictBorders;

  const int panelW = 760;
  const int panelH = 420;
  const int x0 = (screenW - panelW) / 2;
  // Center vertically so the panel looks reasonable across different window sizes.
  const int y0 = std::max(24, (screenH - panelH) / 2);

  DrawRectangle(x0, y0, panelW, panelH, Color{0, 0, 0, 200});
  DrawRectangleLines(x0, y0, panelW, panelH, Color{255, 255, 255, 80});

  int x = x0 + 12;
  int y = y0 + 10;
  DrawText("Save Manager", x, y, 22, RAYWHITE);
  y += 26;

  const char* tabName = (m_saveMenuGroup == 0) ? "Manual" : "Autosaves";
  DrawText(TextFormat("Tab: switch  |  Up/Down: select  |  Enter/F9: load  |  F5: save  |  Del: delete  |  Group: %s",
                      tabName),
           x, y, 15, Color{220, 220, 220, 255});
  y += 22;

  const int listW = 470;
  const int previewX = x0 + listW + 24;
  const int previewY = y;
  const int previewW = panelW - listW - 36;
  const int previewH = panelH - (previewY - y0) - 14;

  DrawRectangle(x0 + 12, y, listW, panelH - (y - y0) - 14, Color{0, 0, 0, 120});
  DrawRectangleLines(x0 + 12, y, listW, panelH - (y - y0) - 14, Color{255, 255, 255, 50});

  const std::vector<SaveMenuSlot>& list = (m_saveMenuGroup == 0) ? m_saveMenuManual : m_saveMenuAutos;
  const int rows = static_cast<int>(list.size());
  const int rowH = 52;
  const int rowX = x0 + 18;
  int rowY = y + 6;

  for (int i = 0; i < rows; ++i) {
    const SaveMenuSlot& e = list[static_cast<std::size_t>(i)];
    const bool sel = (i == m_saveMenuSelection);
    if (sel) {
      DrawRectangle(rowX - 4, rowY - 2, listW - 12, rowH - 2, Color{255, 255, 255, 35});
    }

    const char* slotLabel = e.autosave ? "Auto" : "Slot";
    DrawText(TextFormat("%s %d", slotLabel, e.slot), rowX, rowY, 18,
             sel ? Color{255, 255, 255, 255} : Color{220, 220, 220, 255});

    if (!e.exists) {
      DrawText("(empty)", rowX + 90, rowY + 2, 16, Color{180, 180, 180, 255});
    } else if (!e.summaryOk) {
      DrawText("(unreadable)", rowX + 90, rowY + 2, 16, Color{255, 120, 120, 255});
    } else {
      const Stats& s = e.summary.stats;
      DrawText(TextFormat("Day %d  Pop %d  $%d  Happy %.0f%%", s.day, s.population, s.money,
                          static_cast<double>(s.happiness * 100.0f)),
               rowX + 90, rowY + 2, 16, Color{210, 210, 210, 255});
    }

    // Right-aligned metadata.
    Color meta = Color{180, 180, 180, 255};
    if (e.crcChecked && !e.crcOk) meta = Color{255, 90, 90, 255};

    const char* crcText = (e.crcChecked && !e.crcOk) ? "CORRUPT" : nullptr;
    if (crcText) {
      DrawText(crcText, x0 + listW - 40, rowY + 2, 14, meta);
    }
    DrawText(e.timeText.c_str(), x0 + listW - 140, rowY + 24, 14, meta);

    rowY += rowH;
  }

  // Preview panel
  DrawRectangle(previewX, previewY, previewW, previewH, Color{0, 0, 0, 120});
  DrawRectangleLines(previewX, previewY, previewW, previewH, Color{255, 255, 255, 50});
  DrawText("Preview", previewX + 8, previewY + 6, 18, RAYWHITE);

  if (!list.empty()) {
    const int idx = std::clamp(m_saveMenuSelection, 0, static_cast<int>(list.size()) - 1);
    const SaveMenuSlot& e = list[static_cast<std::size_t>(idx)];

    int py = previewY + 30;
    DrawText(TextFormat("Path: %s", e.path.c_str()), previewX + 8, py, 14, Color{220, 220, 220, 255});
    py += 18;

    if (e.exists && e.summaryOk) {
      const Stats& s = e.summary.stats;
      DrawText(TextFormat("Seed: %llu", static_cast<unsigned long long>(e.summary.seed)), previewX + 8, py, 14,
               Color{220, 220, 220, 255});
      py += 18;
      DrawText(TextFormat("Day %d | Pop %d | Money %d", s.day, s.population, s.money), previewX + 8, py, 14,
               Color{220, 220, 220, 255});
      py += 18;
      DrawText(TextFormat("Happiness: %.0f%%", static_cast<double>(s.happiness * 100.0f)), previewX + 8, py, 14,
               Color{220, 220, 220, 255});
      py += 18;
    }

    if (e.thumbLoaded && e.thumb.id != 0) {
      const int margin = 12;
      Rectangle dst{static_cast<float>(previewX + margin), static_cast<float>(py + 8),
                    static_cast<float>(previewW - margin * 2), static_cast<float>(previewH - (py - previewY) - 18)};

      const float sx = dst.width / static_cast<float>(e.thumb.width);
      const float sy = dst.height / static_cast<float>(e.thumb.height);
      const float s = std::min(sx, sy);
      const float w = static_cast<float>(e.thumb.width) * s;
      const float h = static_cast<float>(e.thumb.height) * s;
      const float dx = dst.x + (dst.width - w) * 0.5f;
      const float dy = dst.y + (dst.height - h) * 0.5f;

      DrawTextureEx(e.thumb, Vector2{dx, dy}, 0.0f, s, RAYWHITE);
      DrawRectangleLinesEx(Rectangle{dx, dy, w, h}, 1, Color{255, 255, 255, 80});
    } else {
      DrawText("(no thumbnail)", previewX + 8, py + 18, 14, Color{180, 180, 180, 255});
    }
  }
}


namespace {

inline float U32ToUnitFloat(std::uint32_t u)
{
  // [0,1)
  return static_cast<float>(u) / 4294967296.0f;
}

// NOTE: Weight callback is a generic callable so we can pass lambdas directly.
// A function-pointer parameter would force callers to explicitly convert lambdas
// (MSVC/GCC do not consider that conversion during template argument deduction).
template <typename Item, typename GetWeight>
int PickWeightedIndex(std::uint64_t& rngState, const std::vector<Item>& items, std::uint64_t totalWeight,
                      GetWeight&& getWeight)
{
  if (items.empty()) return -1;
  if (totalWeight == 0) return -1;

  const std::uint64_t r = SplitMix64Next(rngState) % totalWeight;
  std::uint64_t acc = 0;
  for (int i = 0; i < static_cast<int>(items.size()); ++i) {
    const int w = std::max(0, static_cast<int>(getWeight(items[static_cast<std::size_t>(i)])));
    acc += static_cast<std::uint64_t>(w);
    if (r < acc) return i;
  }
  return static_cast<int>(items.size()) - 1;
}

bool BuildPathFollowingParents(int startRoadIdx, int w, int h, const std::vector<int>& parent,
                              std::vector<Point>& outPath)
{
  outPath.clear();
  if (w <= 0 || h <= 0) return false;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (parent.size() != n) return false;
  if (startRoadIdx < 0 || static_cast<std::size_t>(startRoadIdx) >= n) return false;

  int cur = startRoadIdx;
  int guard = 0;
  while (cur != -1 && guard++ < static_cast<int>(n) + 8) {
    const int x = cur % w;
    const int y = cur / w;
    outPath.push_back(Point{x, y});
    const std::size_t ui = static_cast<std::size_t>(cur);
    if (ui >= parent.size()) break;
    cur = parent[ui];
  }
  return outPath.size() >= 2;
}

} // namespace

void Game::rebuildVehiclesRoutingCache()
{
  m_vehiclesDirty = false;
  m_vehicleSpawnAccum = 0.0f;

  m_vehicles.clear();

  m_commuteJobSources.clear();
  m_commuteOrigins.clear();
  m_commuteOriginWeightTotal = 0;
  m_commuteField = RoadFlowField{};

  m_goodsProducerRoads.clear();
  m_goodsProducerSupply.clear();
  m_goodsProducerWeightTotal = 0;
  m_goodsProducerField = RoadFlowField{};

  m_goodsConsumers.clear();
  m_goodsConsumerWeightTotal = 0;

  m_goodsEdgeSources.clear();
  m_goodsEdgeField = RoadFlowField{};

  const int w = m_world.width();
  const int h = m_world.height();
  if (w <= 0 || h <= 0) return;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  // Outside-connection constraint mirrors the core simulation.
  const bool requireOutside = m_sim.config().requireOutsideConnection;
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (requireOutside) {
    ComputeRoadsConnectedToEdge(m_world, roadToEdgeLocal);
    roadToEdge = &roadToEdgeLocal;
  }

  auto isTraversableRoad = [&](int ridx) -> bool {
    if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) return false;
    const int x = ridx % w;
    const int y = ridx / w;
    if (!m_world.inBounds(x, y)) return false;
    if (m_world.at(x, y).overlay != Overlay::Road) return false;
    if (requireOutside) {
      if (!roadToEdge || roadToEdge->size() != n) return false;
      if ((*roadToEdge)[static_cast<std::size_t>(ridx)] == 0) return false;
    }
    return true;
  };

  auto zoneHasAccess = [&](int zx, int zy) -> bool {
    if (!m_world.hasAdjacentRoad(zx, zy)) return false;
    if (!requireOutside) return true;
    return roadToEdge && HasAdjacentRoadConnectedToEdge(m_world, *roadToEdge, zx, zy);
  };

  // --- Commute routing: sources are road tiles adjacent to commercial/industrial zones ---
  std::vector<std::uint8_t> isJobSource(n, 0);
  m_commuteJobSources.reserve(n / 16);

  const int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;
      if (!zoneHasAccess(x, y)) continue;

      for (const auto& d : dirs) {
        const int rx = x + d[0];
        const int ry = y + d[1];
        if (!m_world.inBounds(rx, ry)) continue;
        if (m_world.at(rx, ry).overlay != Overlay::Road) continue;
        const int ridx = ry * w + rx;
        if (requireOutside && roadToEdge && roadToEdge->size() == n) {
          if ((*roadToEdge)[static_cast<std::size_t>(ridx)] == 0) continue;
        }
        const std::size_t ui = static_cast<std::size_t>(ridx);
        if (ui >= isJobSource.size()) continue;
        if (isJobSource[ui]) continue;
        isJobSource[ui] = 1;
        m_commuteJobSources.push_back(ridx);
      }
    }
  }

  RoadFlowFieldConfig commuteCfg;
  commuteCfg.requireOutsideConnection = requireOutside;
  commuteCfg.computeOwner = false;
  commuteCfg.useTravelTime = true;
  m_commuteField = BuildRoadFlowField(m_world, m_commuteJobSources, commuteCfg, roadToEdge);

  // Origins: residential zones with occupants.
  const float employedShare = (m_world.stats().population > 0)
                                  ? (static_cast<float>(m_world.stats().employed) /
                                     static_cast<float>(m_world.stats().population))
                                  : 0.0f;

  const std::uint32_t seedMix = static_cast<std::uint32_t>(m_world.seed() ^ (m_world.seed() >> 32));
  m_commuteOrigins.reserve(n / 16);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      Point road{};
      if (!PickAdjacentRoadTile(m_world, roadToEdge, x, y, road)) continue;
      const int ridx = road.y * w + road.x;
      if (!isTraversableRoad(ridx)) continue;

      if (m_commuteField.dist.empty() || static_cast<std::size_t>(ridx) >= m_commuteField.dist.size()) continue;
      if (m_commuteField.dist[static_cast<std::size_t>(ridx)] < 0) continue; // unreachable to any job

      const float desired = static_cast<float>(t.occupants) * std::clamp(employedShare, 0.0f, 1.0f);
      int commuters = static_cast<int>(std::floor(desired));
      const float frac = desired - static_cast<float>(commuters);
      if (frac > 0.0f) {
        const std::uint32_t h32 = HashCoords32(x, y, seedMix);
        if (U32ToUnitFloat(h32) < frac) commuters += 1;
      }
      commuters = std::clamp(commuters, 0, static_cast<int>(t.occupants));
      if (commuters <= 0) continue;

      m_commuteOrigins.emplace_back(ridx, commuters);
      m_commuteOriginWeightTotal += static_cast<std::uint64_t>(commuters);
    }
  }

  // --- Goods routing (mirrors the core goods model closely enough for visuals) ---
  GoodsConfig gc;
  gc.requireOutsideConnection = requireOutside;
  // Keep allowImports/allowExports as defaults.

  std::vector<int> supplyPerRoad(n, 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Industrial) continue;
      if (t.level == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      Point road{};
      if (!PickAdjacentRoadTile(m_world, roadToEdge, x, y, road)) continue;
      const int ridx = road.y * w + road.x;
      if (!isTraversableRoad(ridx)) continue;

      const float raw = static_cast<float>(12 * std::clamp(static_cast<int>(t.level), 0, 3)) * gc.supplyScale;
      const int supply = std::max(0, static_cast<int>(std::lround(raw)));
      if (supply <= 0) continue;
      supplyPerRoad[static_cast<std::size_t>(ridx)] += supply;
    }
  }

  for (int ridx = 0; ridx < static_cast<int>(n); ++ridx) {
    const int supply = supplyPerRoad[static_cast<std::size_t>(ridx)];
    if (supply <= 0) continue;
    if (!isTraversableRoad(ridx)) continue;
    m_goodsProducerRoads.push_back(ridx);
    m_goodsProducerSupply.push_back(supply);
    m_goodsProducerWeightTotal += static_cast<std::uint64_t>(supply);
  }

  RoadFlowFieldConfig prodCfg;
  prodCfg.requireOutsideConnection = requireOutside;
  prodCfg.computeOwner = true;
  prodCfg.useTravelTime = true;
  m_goodsProducerField = BuildRoadFlowField(m_world, m_goodsProducerRoads, prodCfg, roadToEdge);

  m_goodsConsumers.reserve(n / 16);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Commercial) continue;
      if (t.level == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      const float raw = static_cast<float>(8 * std::clamp(static_cast<int>(t.level), 0, 3)) * gc.demandScale;
      const int demand = std::max(0, static_cast<int>(std::lround(raw)));
      if (demand <= 0) continue;

      Point road{};
      if (!PickAdjacentRoadTile(m_world, roadToEdge, x, y, road)) continue;
      const int ridx = road.y * w + road.x;
      if (!isTraversableRoad(ridx)) continue;

      const int d = (!m_goodsProducerRoads.empty() && static_cast<std::size_t>(ridx) < m_goodsProducerField.dist.size())
                        ? m_goodsProducerField.dist[static_cast<std::size_t>(ridx)]
                        : -1;
      const int own = (d >= 0 && static_cast<std::size_t>(ridx) < m_goodsProducerField.owner.size())
                          ? m_goodsProducerField.owner[static_cast<std::size_t>(ridx)]
                          : -1;

      m_goodsConsumers.push_back(GoodsConsumerLite{ridx, demand, d, own});
      m_goodsConsumerWeightTotal += static_cast<std::uint64_t>(demand);
    }
  }

  // Edge routing (imports/exports) uses border roads as sources.
  m_goodsEdgeSources.reserve(static_cast<std::size_t>(w + h) * 2u);
  auto pushEdge = [&](int ex, int ey) {
    const int ridx = ey * w + ex;
    if (!isTraversableRoad(ridx)) return;
    m_goodsEdgeSources.push_back(ridx);
  };

  for (int x = 0; x < w; ++x) {
    pushEdge(x, 0);
    if (h > 1) pushEdge(x, h - 1);
  }
  for (int y = 1; y < h - 1; ++y) {
    pushEdge(0, y);
    if (w > 1) pushEdge(w - 1, y);
  }

  if (gc.allowImports || gc.allowExports) {
    RoadFlowFieldConfig edgeCfg;
    edgeCfg.requireOutsideConnection = requireOutside;
    edgeCfg.computeOwner = false;
    edgeCfg.useTravelTime = true;
    m_goodsEdgeField = BuildRoadFlowField(m_world, m_goodsEdgeSources, edgeCfg, roadToEdge);
  }
}

void Game::updateVehicles(float dt)
{
  if (!m_showVehicles) return;

  if (m_vehiclesDirty) {
    rebuildVehiclesRoutingCache();
  }

  // --- Integrate movement ---
  if (dt > 0.0f) {
    std::vector<Vehicle> alive;
    alive.reserve(m_vehicles.size());

    for (Vehicle& v : m_vehicles) {
      if (v.path.size() < 2) continue;

      const float maxS = static_cast<float>(static_cast<int>(v.path.size()) - 1);
      v.s += v.dir * v.speed * dt;

      bool keep = true;
      if (v.s >= maxS) {
        v.s = maxS;
        if (v.kind == VehicleKind::Commute && v.turnsRemaining > 0) {
          v.dir = -1.0f;
          v.turnsRemaining -= 1;
        } else {
          keep = false;
        }
      } else if (v.s <= 0.0f) {
        v.s = 0.0f;
        // Commute vehicles despawn when they return to the origin.
        if (v.kind == VehicleKind::Commute && v.dir < 0.0f) {
          keep = false;
        }
      }

      if (keep) alive.push_back(std::move(v));
    }

    m_vehicles.swap(alive);
  }

  // Don't spawn while paused / painting (dt==0).
  if (dt <= 0.0f) return;

  // --- Targets ---
  int targetCommute = std::clamp(m_world.stats().commuters / kCommutersPerCar, 0, kMaxCommuteVehicles);
  int targetGoods = std::clamp((m_world.stats().goodsDelivered + m_world.stats().goodsExported) / kGoodsPerTruck,
                               0, kMaxGoodsVehicles);

  if (m_commuteJobSources.empty() || m_commuteOrigins.empty()) targetCommute = 0;
  if (m_goodsConsumers.empty()) targetGoods = 0;

  int curCommute = 0;
  int curGoods = 0;
  for (const Vehicle& v : m_vehicles) {
    if (v.kind == VehicleKind::Commute)
      ++curCommute;
    else
      ++curGoods;
  }

  auto makeVehicle = [&](VehicleKind kind, std::vector<Point>&& path, float baseSpeed, int turns) {
    Vehicle v;
    v.kind = kind;
    v.path = std::move(path);
    v.s = 0.0f;
    v.dir = 1.0f;
    v.speed = std::max(0.5f, baseSpeed + RandRange(m_vehicleRngState, -0.75f, 0.75f));
    v.laneOffset = RandRange(m_vehicleRngState, -5.0f, 5.0f);
    v.turnsRemaining = turns;
    m_vehicles.push_back(std::move(v));
  };

  auto speedMultForPath = [&](const std::vector<Point>& path) -> float {
    float sum = 0.0f;
    int count = 0;
    for (const Point& p : path) {
      if (!m_world.inBounds(p.x, p.y)) continue;
      const Tile& t = m_world.at(p.x, p.y);
      if (t.overlay != Overlay::Road) continue;
      sum += RoadSpeedMultiplierForLevel(static_cast<int>(t.level));
      ++count;
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 1.0f;
  };

  auto getOriginWeight = [](const std::pair<int, int>& p) -> int { return p.second; };
  auto getConsumerWeight = [](const GoodsConsumerLite& c) -> int { return c.demand; };
  auto getProducerWeight = [&](const int& roadIdx) -> int {
    (void)roadIdx;
    return 1;
  };

  auto spawnCommute = [&]() -> bool {
    if (m_commuteOrigins.empty()) return false;
    if (m_commuteField.dist.empty() || m_commuteField.parent.empty()) return false;

    const int idx = PickWeightedIndex(m_vehicleRngState, m_commuteOrigins, m_commuteOriginWeightTotal, getOriginWeight);
    if (idx < 0 || idx >= static_cast<int>(m_commuteOrigins.size())) return false;

    const int start = m_commuteOrigins[static_cast<std::size_t>(idx)].first;
    if (start < 0 || static_cast<std::size_t>(start) >= m_commuteField.dist.size()) return false;
    if (m_commuteField.dist[static_cast<std::size_t>(start)] < 0) return false;

    std::vector<Point> path;
    if (!BuildPathFollowingParents(start, m_commuteField.w, m_commuteField.h, m_commuteField.parent, path)) return false;

    const float baseSpeed = 7.5f * speedMultForPath(path);
    makeVehicle(VehicleKind::Commute, std::move(path), baseSpeed, 1);
    return true;
  };

  auto spawnGoods = [&]() -> bool {
    const int delivered = std::max(0, m_world.stats().goodsDelivered);
    const int imported = std::max(0, m_world.stats().goodsImported);
    const int exported = std::max(0, m_world.stats().goodsExported);
    const int goodsTotal = delivered + exported;
    if (goodsTotal <= 0) return false;

    const float exportFrac = (goodsTotal > 0) ? (static_cast<float>(exported) / static_cast<float>(goodsTotal)) : 0.0f;
    const float importFrac = (delivered > 0) ? (static_cast<float>(imported) / static_cast<float>(delivered)) : 0.0f;

    const bool wantExport = (Rand01(m_vehicleRngState) < exportFrac);

    // Export: producer -> edge.
    if (wantExport) {
      if (m_goodsProducerRoads.empty()) return false;
      if (m_goodsEdgeField.parent.empty() || m_goodsEdgeField.dist.empty()) return false;

      // Pick producer weighted by supply.
      if (m_goodsProducerSupply.size() != m_goodsProducerRoads.size() || m_goodsProducerWeightTotal == 0) return false;
      // Build a temporary view of producer indices for weighted picking.
      struct ProducerRef { int idx; int w; };
      std::vector<ProducerRef> refs;
      refs.reserve(m_goodsProducerRoads.size());
      for (int i = 0; i < static_cast<int>(m_goodsProducerRoads.size()); ++i) {
        refs.push_back(ProducerRef{i, m_goodsProducerSupply[static_cast<std::size_t>(i)]});
      }
      auto getW = [](const ProducerRef& r) -> int { return r.w; };
      const int pi = PickWeightedIndex(m_vehicleRngState, refs, m_goodsProducerWeightTotal, getW);
      if (pi < 0 || pi >= static_cast<int>(refs.size())) return false;
      const int pidx = refs[static_cast<std::size_t>(pi)].idx;
      if (pidx < 0 || pidx >= static_cast<int>(m_goodsProducerRoads.size())) return false;
      const int start = m_goodsProducerRoads[static_cast<std::size_t>(pidx)];
      if (start < 0 || static_cast<std::size_t>(start) >= m_goodsEdgeField.dist.size()) return false;
      if (m_goodsEdgeField.dist[static_cast<std::size_t>(start)] < 0) return false;

      std::vector<Point> path;
      if (!BuildPathFollowingParents(start, m_goodsEdgeField.w, m_goodsEdgeField.h, m_goodsEdgeField.parent, path)) return false;
      const float baseSpeed = 5.5f * speedMultForPath(path);
      makeVehicle(VehicleKind::GoodsExport, std::move(path), baseSpeed, 0);
      return true;
    }

    // Delivery: (producer or edge) -> consumer.
    if (m_goodsConsumers.empty()) return false;
    const int ci = PickWeightedIndex(m_vehicleRngState, m_goodsConsumers, m_goodsConsumerWeightTotal, getConsumerWeight);
    if (ci < 0 || ci >= static_cast<int>(m_goodsConsumers.size())) return false;
    const GoodsConsumerLite& c = m_goodsConsumers[static_cast<std::size_t>(ci)];
    if (c.roadIdx < 0) return false;

    const bool preferImport = (Rand01(m_vehicleRngState) < importFrac);

    auto tryImport = [&]() -> bool {
      if (m_goodsEdgeField.parent.empty() || m_goodsEdgeField.dist.empty()) return false;
      if (static_cast<std::size_t>(c.roadIdx) >= m_goodsEdgeField.dist.size()) return false;
      if (m_goodsEdgeField.dist[static_cast<std::size_t>(c.roadIdx)] < 0) return false;
      std::vector<Point> path;
      if (!BuildPathFollowingParents(c.roadIdx, m_goodsEdgeField.w, m_goodsEdgeField.h, m_goodsEdgeField.parent, path)) return false;
      std::reverse(path.begin(), path.end());
      const float baseSpeed = 5.0f * speedMultForPath(path);
      makeVehicle(VehicleKind::GoodsImport, std::move(path), baseSpeed, 0);
      return true;
    };

    auto tryLocal = [&]() -> bool {
      if (m_goodsProducerRoads.empty()) return false;
      if (m_goodsProducerField.parent.empty() || m_goodsProducerField.dist.empty() || m_goodsProducerField.owner.empty()) return false;
      if (static_cast<std::size_t>(c.roadIdx) >= m_goodsProducerField.dist.size()) return false;
      if (m_goodsProducerField.dist[static_cast<std::size_t>(c.roadIdx)] < 0) return false;
      const int own = m_goodsProducerField.owner[static_cast<std::size_t>(c.roadIdx)];
      if (own < 0 || own >= static_cast<int>(m_goodsProducerRoads.size())) return false;

      std::vector<Point> path;
      if (!BuildPathFollowingParents(c.roadIdx, m_goodsProducerField.w, m_goodsProducerField.h, m_goodsProducerField.parent, path)) return false;
      std::reverse(path.begin(), path.end());
      const float baseSpeed = 5.2f * speedMultForPath(path);
      makeVehicle(VehicleKind::GoodsDelivery, std::move(path), baseSpeed, 0);
      return true;
    };

    if (preferImport) {
      if (tryImport()) return true;
      return tryLocal();
    }

    if (tryLocal()) return true;
    return tryImport();
  };

  int spawnBudget = kMaxSpawnPerFrame;

  while (spawnBudget > 0 && curCommute < targetCommute) {
    if (!spawnCommute()) break;
    ++curCommute;
    --spawnBudget;
  }

  while (spawnBudget > 0 && curGoods < targetGoods) {
    if (!spawnGoods()) break;
    ++curGoods;
    --spawnBudget;
  }
}

void Game::drawVehicles()
{
  if (!m_showVehicles) return;
  if (m_vehicles.empty()) return;

  BeginMode2D(m_camera);

  const float zoom = std::max(0.25f, m_camera.zoom);
  const float carR = 2.4f / zoom;
  const float truckW = 7.0f / zoom;
  const float truckH = 4.2f / zoom;

  for (const Vehicle& v : m_vehicles) {
    if (v.path.size() < 2) continue;
    const float maxS = static_cast<float>(static_cast<int>(v.path.size()) - 1);
    const float s = std::clamp(v.s, 0.0f, maxS);
    int seg = static_cast<int>(std::floor(s));
    float t = s - static_cast<float>(seg);
    if (seg >= static_cast<int>(v.path.size()) - 1) {
      seg = static_cast<int>(v.path.size()) - 2;
      t = 1.0f;
    }

    const Point a = v.path[static_cast<std::size_t>(seg)];
    const Point b = v.path[static_cast<std::size_t>(seg + 1)];

    const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                 static_cast<float>(m_cfg.tileHeight), m_elev);
    const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                 static_cast<float>(m_cfg.tileHeight), m_elev);

    Vector2 pos{wa.x + (wb.x - wa.x) * t, wa.y + (wb.y - wa.y) * t};
    Vector2 dir{wb.x - wa.x, wb.y - wa.y};
    const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len > 1e-3f) {
      const Vector2 nrm{-dir.y / len, dir.x / len};
      const float off = (v.laneOffset / zoom);
      pos.x += nrm.x * off;
      pos.y += nrm.y * off;
    }

    Color col = Color{255, 255, 255, 200};
    switch (v.kind) {
    case VehicleKind::Commute: col = Color{245, 245, 245, 200}; break;
    case VehicleKind::GoodsDelivery: col = Color{255, 190, 80, 200}; break;
    case VehicleKind::GoodsImport: col = Color{110, 190, 255, 200}; break;
    case VehicleKind::GoodsExport: col = Color{255, 110, 200, 200}; break;
    }

    if (v.kind == VehicleKind::Commute) {
      DrawCircleV(pos, carR, col);
    } else {
      DrawRectangleV(Vector2{pos.x - truckW * 0.5f, pos.y - truckH * 0.5f}, Vector2{truckW, truckH}, col);
    }
  }

  EndMode2D();
}

void Game::applyToolBrush(int centerX, int centerY)
{
  if (m_tool == Tool::Inspect) return;

  // Terrain editing (Raise/Lower/Smooth) uses modifier keys for strength.
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  // Display toggles
  if (IsKeyPressed(KEY_F11)) {
    toggleFullscreen();
  }
  if ((IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) && IsKeyPressed(KEY_ENTER)) {
    toggleBorderlessWindowed();
  }

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
      const Terrain beforeTerrain = m_world.at(tx, ty).terrain;
      const float beforeHeight = m_world.at(tx, ty).height;
      m_history.noteTilePreEdit(m_world, tx, ty);

      bool applied = false;
      ToolApplyResult res = ToolApplyResult::Noop;

      // --- Terraforming tools are handled at the game layer (they need ProcGenConfig thresholds). ---
      if (m_tool == Tool::RaiseTerrain || m_tool == Tool::LowerTerrain || m_tool == Tool::SmoothTerrain) {
        Tile& t = m_world.at(tx, ty);

        auto classifyTerrain = [&](float h) -> Terrain {
          const float wl = std::clamp(m_procCfg.waterLevel, 0.0f, 1.0f);
          const float sl = std::clamp(m_procCfg.sandLevel, 0.0f, 1.0f);
          if (h < wl) return Terrain::Water;
          if (h < std::max(wl, sl)) return Terrain::Sand;
          return Terrain::Grass;
        };

        // Strength modifiers:
        //  - default: medium
        //  - Shift: stronger
        //  - Ctrl: finer
        float delta = 0.05f;
        if (shift) delta = 0.10f;
        if (ctrl) delta = 0.02f;

        float newH = t.height;
        if (m_tool == Tool::RaiseTerrain) {
          newH = std::clamp(t.height + delta, 0.0f, 1.0f);
        } else if (m_tool == Tool::LowerTerrain) {
          newH = std::clamp(t.height - delta, 0.0f, 1.0f);
        } else if (m_tool == Tool::SmoothTerrain) {
          const int w = m_world.width();
          const int h = m_world.height();
          const std::size_t n = static_cast<std::size_t>(w * h);
          if (w > 0 && h > 0 && m_heightSnapshot.size() == n) {
            auto sample = [&](int sx, int sy) -> float {
              if (sx < 0 || sy < 0 || sx >= w || sy >= h) return m_heightSnapshot[static_cast<std::size_t>(ty * w + tx)];
              return m_heightSnapshot[static_cast<std::size_t>(sy * w + sx)];
            };

            // 3x3 neighborhood average from the snapshot so smoothing is order-independent.
            float sum = 0.0f;
            int count = 0;
            for (int oy = -1; oy <= 1; ++oy) {
              for (int ox = -1; ox <= 1; ++ox) {
                sum += sample(tx + ox, ty + oy);
                ++count;
              }
            }
            const float avg = (count > 0) ? (sum / static_cast<float>(count)) : t.height;

            float alpha = 0.5f;
            if (shift) alpha = 0.75f;
            if (ctrl) alpha = 0.25f;

            newH = std::clamp(m_heightSnapshot[static_cast<std::size_t>(ty * w + tx)] + (avg - m_heightSnapshot[static_cast<std::size_t>(ty * w + tx)]) * alpha,
                              0.0f, 1.0f);
          }
        }

        // Apply height.
        t.height = newH;

        // Derive terrain from height thresholds.
        const Terrain newTerrain = classifyTerrain(t.height);
        if (newTerrain == Terrain::Water) {
          // When a tile becomes water we must clear most overlays.
          // Roads are the exception: a Road overlay on a Water tile is treated as a bridge.
          if (t.overlay != Overlay::None && t.overlay != Overlay::Road) {
            m_world.setOverlay(Overlay::None, tx, ty);
            t.overlay = Overlay::None;
            t.level = 1;
            t.occupants = 0;
          }
        }
        t.terrain = newTerrain;

        const Overlay afterOverlay = t.overlay;
        const bool overlayChanged = (afterOverlay != beforeOverlay);
        const bool terrainChanged = (t.terrain != beforeTerrain);
        const bool heightChanged = (t.height != beforeHeight);

        applied = overlayChanged || terrainChanged || heightChanged;

        if (applied) {
          m_landValueDirty = true;
        }

        if (overlayChanged) {
          m_trafficDirty = true;
          m_goodsDirty = true;
          m_vehiclesDirty = true;
        }

        // Road graph only changes if a road was added/removed.
        if (overlayChanged && (beforeOverlay == Overlay::Road || afterOverlay == Overlay::Road)) {
          m_roadGraphDirty = true;
        }
      } else {
        if (m_tool == Tool::District) {
          // Districts are a lightweight label layer; they do not run through the economy rules.
          res = m_world.applyDistrict(tx, ty, m_activeDistrict);
          applied = (res == ToolApplyResult::Applied);
        } else {
          // --- Regular tools go through World::applyTool (economy + rules). ---
          res = (m_tool == Tool::Road) ? m_world.applyRoad(tx, ty, m_roadBuildLevel)
                                       : m_world.applyTool(m_tool, tx, ty);
          switch (res) {
          case ToolApplyResult::InsufficientFunds: m_strokeFeedback.noMoney = true; break;
          case ToolApplyResult::BlockedNoRoad: m_strokeFeedback.noRoad = true; break;
          case ToolApplyResult::BlockedWater: m_strokeFeedback.water = true; break;
          case ToolApplyResult::BlockedOccupied: m_strokeFeedback.occupied = true; break;
          default: break;
          }

          applied = (res == ToolApplyResult::Applied);
          if (applied) {
            m_landValueDirty = true;
            // Traffic depends on roads + zones + occupancy.
            m_trafficDirty = true;
            // Goods logistics depend on roads + industrial/commercial zoning.
            m_goodsDirty = true;
            // Moving vehicles (visualization) also depend on roads + zones + occupancy.
            m_vehiclesDirty = true;

            if (m_tool == Tool::Road || (m_tool == Tool::Bulldoze && beforeOverlay == Overlay::Road)) {
              m_roadGraphDirty = true;
            }
          }
        }
      }

      if (applied) {
        // District edits do not affect cached terrain/overlays, so avoid base-cache rebuild churn.
        if (m_tool != Tool::District) {
          m_tilesEditedThisStroke.push_back(Point{tx, ty});
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
  m_tilesEditedThisStroke.clear();
  m_history.beginStroke(m_world);

  // Snapshot heights for order-independent smoothing.
  m_heightSnapshot.clear();
  if (m_tool == Tool::SmoothTerrain) {
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));
    m_heightSnapshot.resize(n, 0.0f);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        m_heightSnapshot[static_cast<std::size_t>(y * w + x)] = m_world.at(x, y).height;
      }
    }
  }

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

  // A stroke potentially changes many tiles; update the minimap lazily.
  m_renderer.markMinimapDirty();

  // Also refresh the (optional) cached base render for any edited tiles.
  m_renderer.markBaseCacheDirtyForTiles(m_tilesEditedThisStroke, m_world.width(), m_world.height());
  m_tilesEditedThisStroke.clear();

  // Height snapshot is only valid for the current stroke.
  m_heightSnapshot.clear();

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
    m_renderer.markMinimapDirty();
    m_renderer.markBaseCacheDirtyAll();
    m_roadGraphDirty = true;
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
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
    m_renderer.markMinimapDirty();
    m_renderer.markBaseCacheDirtyAll();
    m_roadGraphDirty = true;
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
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
  m_renderer.markMinimapDirty();
  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_landValueDirty = true;
  m_vehiclesDirty = true;
  m_vehicles.clear();

  // Deterministic vehicle RNG seed per world seed.
  m_vehicleRngState = (newSeed ^ 0x9E3779B97F4A7C15ULL);


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
  m_roadDragUpgradeTiles = 0;
  m_roadDragBridgeTiles = 0;
  m_roadDragMoneyCost = 0;
  m_roadDragValid = false;

  // Optional: vary procedural textures per seed (still no assets-from-disk).
  m_renderer.rebuildTextures(newSeed);
  m_renderer.markBaseCacheDirtyAll();

  // Make HUD stats immediately correct (without waiting for the first sim tick).
  m_sim.refreshDerivedStats(m_world);

  clearHistory();
  recordHistorySample(m_world.stats());

  // Update title with seed.
  SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(newSeed)));

  // Recenter camera.
  m_camera.target = TileToWorldCenterElevated(m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2,
                                              static_cast<float>(m_cfg.tileWidth),
                                              static_cast<float>(m_cfg.tileHeight), m_elev);
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

void Game::floodFillDistrict(Point start, bool includeRoads)
{
  if (!m_world.inBounds(start.x, start.y)) return;

  beginPaintStroke();

  const int w = m_world.width();
  const int h = m_world.height();
  const int n = w * h;

  const std::uint8_t targetDistrict = static_cast<std::uint8_t>(std::clamp(m_activeDistrict, 0, kDistrictCount - 1));
  const Tile& seed = m_world.at(start.x, start.y);

  enum class FillMode { RoadComponent, WaterBody, LandBlock };
  const FillMode mode = (seed.overlay == Overlay::Road) ? FillMode::RoadComponent :
                        (seed.terrain == Terrain::Water) ? FillMode::WaterBody :
                                                          FillMode::LandBlock;

  auto canFill = [&](int x, int y) -> bool {
    if (!m_world.inBounds(x, y)) return false;
    const Tile& t = m_world.at(x, y);
    switch (mode) {
    case FillMode::RoadComponent:
      return t.overlay == Overlay::Road;
    case FillMode::WaterBody:
      // Water fill excludes bridges (road overlay).
      return t.terrain == Terrain::Water && t.overlay != Overlay::Road;
    case FillMode::LandBlock:
    default:
      if (t.terrain == Terrain::Water) return false;
      if (!includeRoads && t.overlay == Overlay::Road) return false;
      return true;
    }
  };

  std::vector<std::uint8_t> visited(static_cast<std::size_t>(n), 0);
  std::vector<Point> stack;
  stack.reserve(std::min(n, 4096));

  auto push = [&](int x, int y) {
    const int idx = y * w + x;
    if (visited[static_cast<std::size_t>(idx)]) return;
    visited[static_cast<std::size_t>(idx)] = 1;
    stack.push_back({x, y});
  };

  if (canFill(start.x, start.y)) push(start.x, start.y);

  int changed = 0;
  while (!stack.empty()) {
    const Point p = stack.back();
    stack.pop_back();

    Tile& t = m_world.at(p.x, p.y);
    if (t.district != targetDistrict) {
      m_history.noteTilePreEdit(m_world, p.x, p.y);
      t.district = targetDistrict;
      ++changed;
    }

    const int x = p.x;
    const int y = p.y;
    if (x > 0 && canFill(x - 1, y)) push(x - 1, y);
    if (x + 1 < w && canFill(x + 1, y)) push(x + 1, y);
    if (y > 0 && canFill(x, y - 1)) push(x, y - 1);
    if (y + 1 < h && canFill(x, y + 1)) push(x, y + 1);
  }

  endPaintStroke();
  if (changed > 0) {
    showToast(TextFormat("District fill: %d tiles", changed));
  }
}

void Game::handleInput(float dt)
{
  // Keep UI scaling in sync with monitor DPI and any window resizes.
  updateUiScaleHotkeys();
  updateWorldRenderHotkeys();

  const int screenW = GetScreenWidth();
  const int screenH = GetScreenHeight();
  const float uiScale = m_uiScale;
  const int uiW = static_cast<int>(std::round(static_cast<float>(screenW) / uiScale));
  const int uiH = static_cast<int>(std::round(static_cast<float>(screenH) / uiScale));

  const Vector2 mouse = GetMousePosition();
  const Vector2 mouseUi = mouseUiPosition(uiScale);

  // Update hovered tile from mouse.
  const Vector2 mouseWorld = GetScreenToWorld2D(mouse, m_camera);
  m_hovered = WorldToTileElevated(mouseWorld, m_world, static_cast<float>(m_cfg.tileWidth),
                                 static_cast<float>(m_cfg.tileHeight), m_elev);

  // Undo/redo
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  // Fullscreen/borderless toggles (common PC shortcuts).
  if (IsKeyPressed(KEY_F11)) {
    toggleFullscreen();
  }
  if ((IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) && IsKeyPressed(KEY_ENTER)) {
    toggleBorderlessWindowed();
  }

  // Developer console (toggle with F4). When open it captures keyboard input.
  if (IsKeyPressed(KEY_F4)) {
    endPaintStroke();

    // Avoid overlapping input-capturing UIs.
    if (!m_console.isOpen() && m_showSaveMenu) {
      unloadSaveMenuThumbnails();
      m_showSaveMenu = false;
      m_saveMenuDeleteArmed = false;
    }

    m_console.toggle();
    showToast(m_console.isOpen() ? "Console: ON" : "Console: OFF");
  }

  if (m_console.isOpen()) {
    (void)m_console.update(dt, uiW, uiH, mouseUi.x, mouseUi.y);
    return;
  }

  if (ctrl && shift && IsKeyPressed(KEY_Z)) {
    doRedo();
  } else if (ctrl && IsKeyPressed(KEY_Z)) {
    doUndo();
  } else if (ctrl && IsKeyPressed(KEY_Y)) {
    doRedo();
  }

  // Save manager UI (toggle with F10). When open, it captures most input.
  if (IsKeyPressed(KEY_F10)) {
    endPaintStroke();
    m_showSaveMenu = !m_showSaveMenu;
    if (m_showSaveMenu) {
      m_saveMenuDeleteArmed = false;
      m_saveMenuRefreshTimer = 0.0f;
      refreshSaveMenu();
      showToast("Save menu: ON");
    } else {
      unloadSaveMenuThumbnails();
      m_saveMenuDeleteArmed = false;
      showToast("Save menu: OFF");
    }
  }

  // Capture controls
  // - F12: window screenshot
  // - Ctrl+F12: full city overview export (off-screen render)
  if (IsKeyPressed(KEY_F12)) {
    endPaintStroke();

    std::error_code ec;
    const std::filesystem::path outDir = std::filesystem::path("captures");
    std::filesystem::create_directories(outDir, ec);

    const std::string stamp = FileTimestamp();
    const unsigned long long seed = static_cast<unsigned long long>(m_world.seed());
    const int day = m_world.stats().day;

    auto makeFileName = [&](const char* prefix) {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%s_seed%llu_day%d_%s.png", prefix, seed, day, stamp.c_str());
      return outDir / buf;
    };

    if (ctrl) {
      const std::filesystem::path outPath = makeFileName("map");
      const bool ok = m_renderer.exportWorldOverview(m_world, outPath.string().c_str(), 4096);
      showToast(ok ? (std::string("Map exported: ") + outPath.string()) : "Map export failed", 3.0f);
    } else {
      // Queue the screenshot so it's captured after the frame is drawn.
      m_pendingScreenshotPath = makeFileName("screenshot").string();
      m_pendingScreenshot = true;
    }
  }

  if (m_showSaveMenu) {
    // Group switch.
    if (IsKeyPressed(KEY_TAB)) {
      m_saveMenuGroup = (m_saveMenuGroup == 0) ? 1 : 0;
      m_saveMenuSelection = 0;
      m_saveMenuDeleteArmed = false;
    }

    const std::vector<SaveMenuSlot>& list = (m_saveMenuGroup == 0) ? m_saveMenuManual : m_saveMenuAutos;
    const int count = static_cast<int>(list.size());

    if (IsKeyPressed(KEY_UP)) m_saveMenuSelection = std::max(0, m_saveMenuSelection - 1);
    if (IsKeyPressed(KEY_DOWN)) m_saveMenuSelection = std::min(std::max(0, count - 1), m_saveMenuSelection + 1);

    if (count > 0) {
      const int idx = std::clamp(m_saveMenuSelection, 0, count - 1);
      const SaveMenuSlot& e = list[static_cast<std::size_t>(idx)];

      // Load selected (Enter or F9).
      if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsKeyPressed(KEY_F9)) {
        if (e.exists) {
          const char* label = e.autosave ? TextFormat("Autosave %d", e.slot) : TextFormat("Slot %d", e.slot);
          loadFromPath(e.path, label);
        } else {
          showToast("No save in that slot", 2.0f);
        }
      }

      // Save into selected manual slot (F5).
      if (IsKeyPressed(KEY_F5)) {
        if (!e.autosave) {
          m_saveSlot = e.slot;
          saveToPath(e.path, true, TextFormat("Slot %d", e.slot));
        } else {
          showToast("Autosaves are read-only", 2.0f);
        }
      }

      // Delete selected (Del twice to confirm).
      if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
        if (!e.exists) {
          showToast("Slot is already empty", 2.0f);
        } else if (!m_saveMenuDeleteArmed) {
          m_saveMenuDeleteArmed = true;
          m_saveMenuDeleteTimer = 1.5f;
          showToast("Press Del again to delete", 1.5f);
        } else {
          namespace fs = std::filesystem;
          std::error_code ec;
          fs::remove(fs::path(e.path), ec);
          const std::string tp = thumbPathForSavePath(e.path);
          fs::remove(fs::path(tp), ec);
          m_saveMenuDeleteArmed = false;
          refreshSaveMenu();
          showToast("Deleted save", 1.5f);
        }
      }
    }

    // While the save menu is open we don't want other gameplay inputs to fire.
    return;
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
    recordHistorySample(m_world.stats());
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
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

  if (IsKeyPressed(KEY_F1)) {
    m_showReport = !m_showReport;
    showToast(m_showReport ? "City report: ON" : "City report: OFF");
  }

  if (IsKeyPressed(KEY_F2)) {
    const bool enabled = !m_renderer.baseCacheEnabled();
    m_renderer.setBaseCacheEnabled(enabled);
    m_renderer.markBaseCacheDirtyAll();
    showToast(enabled ? "Render cache: ON" : "Render cache: OFF");
  }

  if (IsKeyPressed(KEY_F3)) {
    m_showTrafficModel = !m_showTrafficModel;
    showToast(m_showTrafficModel ? "Traffic model: ON" : "Traffic model: OFF");
    endPaintStroke();
  }

  if (IsKeyPressed(KEY_F7)) {
    m_showDistrictPanel = !m_showDistrictPanel;
    showToast(m_showDistrictPanel ? "Districts panel: ON" : "Districts panel: OFF");
    endPaintStroke();
  }

  if (IsKeyPressed(KEY_F8)) {
    m_showVideoSettings = !m_showVideoSettings;
    showToast(m_showVideoSettings ? "Video settings: ON" : "Video settings: OFF");
    endPaintStroke();
  }

  if (IsKeyPressed(KEY_P)) {
    m_showPolicy = !m_showPolicy;
    showToast(m_showPolicy ? "Policy: ON" : "Policy: OFF");
  }


  if (IsKeyPressed(KEY_TAB)) {
    // Hold Shift to cycle backwards.
    const int delta = shift ? -1 : 1;

    if (m_showReport) {
      constexpr int kPages = 5;
      m_reportPage = (m_reportPage + delta + kPages) % kPages;
    } else if (m_showPolicy) {
      const int count = 7;
      m_policySelection = (m_policySelection + delta + count) % count;
    } else if (m_showTrafficModel) {
      const int count = 6;
      m_trafficModelSelection = (m_trafficModelSelection + delta + count) % count;
    } else if (m_showDistrictPanel) {
      const int count = 9;
      m_districtSelection = (m_districtSelection + delta + count) % count;
    } else if (m_showVideoSettings) {
      const int count = 11;
      m_videoSelection = (m_videoSelection + delta + count) % count;
    }
  }

  if (IsKeyPressed(KEY_M)) {
    m_showMinimap = !m_showMinimap;
    if (m_showMinimap) m_renderer.markMinimapDirty();
    showToast(m_showMinimap ? "Minimap: ON" : "Minimap: OFF");
  }

  if (IsKeyPressed(KEY_C)) {
    m_showVehicles = !m_showVehicles;
    m_vehiclesDirty = true;
    if (!m_showVehicles) m_vehicles.clear();
    showToast(m_showVehicles ? "Vehicles: ON" : "Vehicles: OFF");
  }

  // Toggle elevation rendering (flat <-> elevated). This is purely visual; terraforming is separate.
  if (IsKeyPressed(KEY_E)) {
    endPaintStroke();
    if (m_elev.maxPixels > 0.0f) {
      m_elev.maxPixels = 0.0f;
      showToast("Elevation: OFF");
    } else {
      m_elev = m_elevDefault;
      showToast(TextFormat("Elevation: ON (max %.0fpx)", static_cast<double>(m_elev.maxPixels)));
    }
    m_renderer.setElevationSettings(m_elev);
  }
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
      {
        const TrafficModelSettings& tm = m_sim.trafficModel();
        tc.congestionAwareRouting = tm.congestionAwareRouting;
        tc.congestionIterations = tm.congestionIterations;
        tc.congestionAlpha = tm.congestionAlpha;
        tc.congestionBeta = tm.congestionBeta;
        tc.congestionCapacityScale = tm.congestionCapacityScale;
        tc.congestionRatioClamp = tm.congestionRatioClamp;
      }

      // Traffic overlay should respect the sim's outside-connection rule even
      // if the connectivity overlay itself is not being drawn.
      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* pre = nullptr;
      if (tc.requireOutsideConnection) {
        ComputeRoadsConnectedToEdge(m_world, roadToEdge);
        pre = &roadToEdge;
      }

      m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
      m_trafficDirty = false;

      showToast(TextFormat(
          "Traffic overlay: ON (%d commuters, avg %.1f (t %.1f), cong %.0f%%, %s x%d)", m_traffic.totalCommuters,
          static_cast<double>(m_traffic.avgCommute), static_cast<double>(m_traffic.avgCommuteTime),
          static_cast<double>(m_traffic.congestion * 100.0f),
          m_traffic.usedCongestionAwareRouting ? "cong" : "free", m_traffic.routingPasses));
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

      // Goods overlay should respect the sim's outside-connection rule even
      // if the connectivity overlay itself is not being drawn.
      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* pre = nullptr;
      if (gc.requireOutsideConnection) {
        ComputeRoadsConnectedToEdge(m_world, roadToEdge);
        pre = &roadToEdge;
      }

      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;

      showToast(TextFormat("Goods overlay: ON (deliv %d/%d, sat %.0f%%, imp %d, exp %d)",
                           m_goods.goodsDelivered, m_goods.goodsDemand,
                           static_cast<double>(m_goods.satisfaction * 100.0f),
                           m_goods.goodsImported, m_goods.goodsExported));
    } else {
      showToast("Goods overlay: OFF");
    }
  }

  // Heatmap overlay: cycle through land value + components.
  if (IsKeyPressed(KEY_L)) {
    auto nameOf = [&](HeatmapOverlay m) -> const char* {
      switch (m) {
      case HeatmapOverlay::Off: return "OFF";
      case HeatmapOverlay::LandValue: return "Land value";
      case HeatmapOverlay::ParkAmenity: return "Park amenity";
      case HeatmapOverlay::WaterAmenity: return "Water amenity";
      case HeatmapOverlay::Pollution: return "Pollution";
      case HeatmapOverlay::TrafficSpill: return "Traffic spill";
      default: return "Heatmap";
      }
    };

    auto toIndex = [&](HeatmapOverlay m) -> int {
      switch (m) {
      case HeatmapOverlay::Off: return 0;
      case HeatmapOverlay::LandValue: return 1;
      case HeatmapOverlay::ParkAmenity: return 2;
      case HeatmapOverlay::WaterAmenity: return 3;
      case HeatmapOverlay::Pollution: return 4;
      case HeatmapOverlay::TrafficSpill: return 5;
      default: return 0;
      }
    };

    auto fromIndex = [&](int i) -> HeatmapOverlay {
      switch (i) {
      case 0: return HeatmapOverlay::Off;
      case 1: return HeatmapOverlay::LandValue;
      case 2: return HeatmapOverlay::ParkAmenity;
      case 3: return HeatmapOverlay::WaterAmenity;
      case 4: return HeatmapOverlay::Pollution;
      case 5: return HeatmapOverlay::TrafficSpill;
      default: return HeatmapOverlay::Off;
      }
    };

    const int count = 6;
    const int delta = shift ? -1 : 1;
    int idx = toIndex(m_heatmapOverlay);
    idx = (idx + delta + count) % count;
    m_heatmapOverlay = fromIndex(idx);

    m_landValueDirty = true;
    showToast(TextFormat("Heatmap: %s", nameOf(m_heatmapOverlay)));
  }

  // Brush radius
  if (IsKeyPressed(KEY_LEFT_BRACKET)) {
    if (m_showPolicy) {
      // Policy adjustments
      const int delta = shift ? -5 : -1;
      SimConfig& cfg = m_sim.config();

      auto clampInt = [&](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

      switch (m_policySelection) {
      case 0: cfg.taxResidential = clampInt(cfg.taxResidential + delta, 0, 10); break;
      case 1: cfg.taxCommercial = clampInt(cfg.taxCommercial + delta, 0, 10); break;
      case 2: cfg.taxIndustrial = clampInt(cfg.taxIndustrial + delta, 0, 10); break;
      case 3: cfg.maintenanceRoad = clampInt(cfg.maintenanceRoad + (shift ? -2 : -1), 0, 5); break;
      case 4: cfg.maintenancePark = clampInt(cfg.maintenancePark + (shift ? -2 : -1), 0, 5); break;
      case 5: cfg.requireOutsideConnection = !cfg.requireOutsideConnection; break;
      case 6: cfg.parkInfluenceRadius = clampInt(cfg.parkInfluenceRadius + (shift ? -2 : -1), 0, 20); break;
      default: break;
      }

      // Updating policies affects derived stats and overlays.
      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_outsideOverlayRoadToEdge.clear();
    } else if (m_showTrafficModel) {
      // Traffic model adjustments
      const float fdelta = shift ? -0.20f : -0.05f;
      TrafficModelSettings& tm = m_sim.trafficModel();

      auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };
      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      switch (m_trafficModelSelection) {
      case 0: tm.congestionAwareRouting = !tm.congestionAwareRouting; break;
      case 1: tm.congestionIterations = clampI(tm.congestionIterations + (shift ? -2 : -1), 1, 16); break;
      case 2: tm.congestionAlpha = clampF(tm.congestionAlpha + fdelta, 0.0f, 2.0f); break;
      case 3: tm.congestionBeta = clampF(tm.congestionBeta + (shift ? -2.0f : -1.0f), 1.0f, 8.0f); break;
      case 4: tm.congestionCapacityScale = clampF(tm.congestionCapacityScale + (shift ? -0.25f : -0.10f), 0.25f, 4.0f);
        break;
      case 5: tm.congestionRatioClamp = clampF(tm.congestionRatioClamp + (shift ? -1.0f : -0.5f), 1.0f, 10.0f);
        break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
    } else if (m_showDistrictPanel) {
      const int deltaI = shift ? -2 : -1;
      const float deltaF = shift ? -0.25f : -0.05f;
      SimConfig& cfg = m_sim.config();

      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      const int d = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
      DistrictPolicy& pol = cfg.districtPolicies[static_cast<std::size_t>(d)];

      switch (m_districtSelection) {
      case 0:
        cfg.districtPoliciesEnabled = !cfg.districtPoliciesEnabled;
        showToast(cfg.districtPoliciesEnabled ? "District policies: ON" : "District policies: OFF");
        break;
      case 1:
        m_activeDistrict = (m_activeDistrict + deltaI + kDistrictCount) % kDistrictCount;
        showToast(TextFormat("Active district: %d", m_activeDistrict));
        break;
      case 2:
        m_showDistrictOverlay = !m_showDistrictOverlay;
        showToast(m_showDistrictOverlay ? "District overlay: ON" : "District overlay: OFF");
        break;
      case 3:
        m_showDistrictBorders = !m_showDistrictBorders;
        showToast(m_showDistrictBorders ? "District borders: ON" : "District borders: OFF");
        break;
      case 4:
        pol.taxResidentialMult = clampF(pol.taxResidentialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d res tax mult: %.2f", d, pol.taxResidentialMult));
        break;
      case 5:
        pol.taxCommercialMult = clampF(pol.taxCommercialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d com tax mult: %.2f", d, pol.taxCommercialMult));
        break;
      case 6:
        pol.taxIndustrialMult = clampF(pol.taxIndustrialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d ind tax mult: %.2f", d, pol.taxIndustrialMult));
        break;
      case 7:
        pol.roadMaintenanceMult = clampF(pol.roadMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d road maint mult: %.2f", d, pol.roadMaintenanceMult));
        break;
      case 8:
        pol.parkMaintenanceMult = clampF(pol.parkMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d park maint mult: %.2f", d, pol.parkMaintenanceMult));
        break;
      default: break;
      }

      // Policies affect derived stats and budget.
      m_sim.refreshDerivedStats(m_world);
    } else if (m_showVideoSettings) {
      adjustVideoSettings(-1);
    } else {
      m_brushRadius = std::max(0, m_brushRadius - 1);
      showToast(TextFormat("Brush radius: %d", m_brushRadius));
    }
  }
  if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
    if (m_showPolicy) {
      const int delta = shift ? 5 : 1;
      SimConfig& cfg = m_sim.config();

      auto clampInt = [&](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

      switch (m_policySelection) {
      case 0: cfg.taxResidential = clampInt(cfg.taxResidential + delta, 0, 10); break;
      case 1: cfg.taxCommercial = clampInt(cfg.taxCommercial + delta, 0, 10); break;
      case 2: cfg.taxIndustrial = clampInt(cfg.taxIndustrial + delta, 0, 10); break;
      case 3: cfg.maintenanceRoad = clampInt(cfg.maintenanceRoad + (shift ? 2 : 1), 0, 5); break;
      case 4: cfg.maintenancePark = clampInt(cfg.maintenancePark + (shift ? 2 : 1), 0, 5); break;
      case 5: cfg.requireOutsideConnection = !cfg.requireOutsideConnection; break;
      case 6: cfg.parkInfluenceRadius = clampInt(cfg.parkInfluenceRadius + (shift ? 2 : 1), 0, 20); break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_outsideOverlayRoadToEdge.clear();
    } else if (m_showTrafficModel) {
      // Traffic model adjustments
      const float fdelta = shift ? 0.20f : 0.05f;
      TrafficModelSettings& tm = m_sim.trafficModel();

      auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };
      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      switch (m_trafficModelSelection) {
      case 0: tm.congestionAwareRouting = !tm.congestionAwareRouting; break;
      case 1: tm.congestionIterations = clampI(tm.congestionIterations + (shift ? 2 : 1), 1, 16); break;
      case 2: tm.congestionAlpha = clampF(tm.congestionAlpha + fdelta, 0.0f, 2.0f); break;
      case 3: tm.congestionBeta = clampF(tm.congestionBeta + (shift ? 2.0f : 1.0f), 1.0f, 8.0f); break;
      case 4: tm.congestionCapacityScale = clampF(tm.congestionCapacityScale + (shift ? 0.25f : 0.10f), 0.25f, 4.0f);
        break;
      case 5: tm.congestionRatioClamp = clampF(tm.congestionRatioClamp + (shift ? 1.0f : 0.5f), 1.0f, 10.0f);
        break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
    } else if (m_showDistrictPanel) {
      const int deltaI = shift ? 2 : 1;
      const float deltaF = shift ? 0.25f : 0.05f;
      SimConfig& cfg = m_sim.config();

      const int d = ((m_activeDistrict % kDistrictCount) + kDistrictCount) % kDistrictCount;
      DistrictPolicy& pol = cfg.districtPolicies[static_cast<std::size_t>(d)];
      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      switch (m_districtSelection) {
      case 0:
        cfg.districtPoliciesEnabled = !cfg.districtPoliciesEnabled;
        showToast(cfg.districtPoliciesEnabled ? "District policies: ON" : "District policies: OFF");
        break;
      case 1:
        m_activeDistrict = (m_activeDistrict + deltaI) % kDistrictCount;
        showToast(TextFormat("Active district: %d", m_activeDistrict));
        break;
      case 2:
        m_showDistrictOverlay = !m_showDistrictOverlay;
        showToast(m_showDistrictOverlay ? "District overlay: ON" : "District overlay: OFF");
        break;
      case 3:
        m_showDistrictBorders = !m_showDistrictBorders;
        showToast(m_showDistrictBorders ? "District borders: ON" : "District borders: OFF");
        break;
      case 4:
        pol.taxResidentialMult = clampF(pol.taxResidentialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d res tax mult: %.2f", d, pol.taxResidentialMult));
        break;
      case 5:
        pol.taxCommercialMult = clampF(pol.taxCommercialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d com tax mult: %.2f", d, pol.taxCommercialMult));
        break;
      case 6:
        pol.taxIndustrialMult = clampF(pol.taxIndustrialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d ind tax mult: %.2f", d, pol.taxIndustrialMult));
        break;
      case 7:
        pol.roadMaintenanceMult = clampF(pol.roadMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d road maint mult: %.2f", d, pol.roadMaintenanceMult));
        break;
      case 8:
        pol.parkMaintenanceMult = clampF(pol.parkMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d park maint mult: %.2f", d, pol.parkMaintenanceMult));
        break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
    } else if (m_showVideoSettings) {
      adjustVideoSettings(+1);
    } else {
      m_brushRadius = std::min(8, m_brushRadius + 1);
      showToast(TextFormat("Brush radius: %d", m_brushRadius));
    }
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
    const std::string path = savePathForSlot(m_saveSlot);
    saveToPath(path, true, TextFormat("Slot %d", m_saveSlot));
  }

  if (IsKeyPressed(KEY_F9)) {
    const std::string path = savePathForSlot(m_saveSlot);
    loadFromPath(path, TextFormat("Slot %d", m_saveSlot));
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
    m_roadDragUpgradeTiles = 0;
    m_roadDragBridgeTiles = 0;
    m_roadDragMoneyCost = 0;
    m_roadDragValid = false;
  };

  if (IsKeyPressed(KEY_Q)) setTool(Tool::Inspect);
  if (IsKeyPressed(KEY_ONE)) setTool(Tool::Road);
  if (IsKeyPressed(KEY_TWO)) setTool(Tool::Residential);
  if (IsKeyPressed(KEY_THREE)) setTool(Tool::Commercial);
  if (IsKeyPressed(KEY_FOUR)) setTool(Tool::Industrial);
  if (IsKeyPressed(KEY_FIVE)) setTool(Tool::Park);
  if (IsKeyPressed(KEY_ZERO)) setTool(Tool::Bulldoze);

  // Road tool: cycle the road class used for placement/upgrade (Street/Avenue/Highway).
  if (IsKeyPressed(KEY_U)) {
    const int delta = shift ? -1 : 1;
    m_roadBuildLevel += delta;
    if (m_roadBuildLevel < 1) m_roadBuildLevel = 3;
    if (m_roadBuildLevel > 3) m_roadBuildLevel = 1;
    showToast(TextFormat("Road type: %s", RoadClassName(m_roadBuildLevel)));
  }
  if (IsKeyPressed(KEY_SIX)) setTool(Tool::RaiseTerrain);
  if (IsKeyPressed(KEY_SEVEN)) setTool(Tool::LowerTerrain);
  if (IsKeyPressed(KEY_EIGHT)) setTool(Tool::SmoothTerrain);
  if (IsKeyPressed(KEY_NINE)) setTool(Tool::District);

  if (m_tool == Tool::District) {
    if (IsKeyPressed(KEY_COMMA)) {
      m_activeDistrict = (m_activeDistrict + kDistrictCount - 1) % kDistrictCount;
      showToast(TextFormat("Active district: %d", m_activeDistrict));
    }
    if (IsKeyPressed(KEY_PERIOD)) {
      m_activeDistrict = (m_activeDistrict + 1) % kDistrictCount;
      showToast(TextFormat("Active district: %d", m_activeDistrict));
    }
  }

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

  // --- Minimap interaction (UI consumes left mouse so we don't accidentally paint the world). ---
  bool consumeLeft = false;
  if (m_showMinimap && m_world.width() > 0 && m_world.height() > 0) {
    const Renderer::MinimapLayout mini = m_renderer.minimapLayout(m_world, uiW, uiH);
    const Vector2 mp = mouseUi;
    const bool over = CheckCollisionPointRec(mp, mini.rect);

    if (leftPressed && over) {
      // Cancel any in-progress stroke before moving the camera.
      endPaintStroke();
      m_minimapDragActive = true;
    }

    if (leftReleased) {
      m_minimapDragActive = false;
    }

    if (leftDown && m_minimapDragActive) {
      const float lx = std::clamp(mp.x - mini.rect.x, 0.0f, std::max(1.0f, mini.rect.width - 1.0f));
      const float ly = std::clamp(mp.y - mini.rect.y, 0.0f, std::max(1.0f, mini.rect.height - 1.0f));

      const float s = std::max(1.0e-3f, mini.pixelsPerTile);
      const int tx = std::clamp(static_cast<int>(std::floor(lx / s)), 0, m_world.width() - 1);
      const int ty = std::clamp(static_cast<int>(std::floor(ly / s)), 0, m_world.height() - 1);

      m_camera.target = TileToWorldCenterElevated(m_world, tx, ty, static_cast<float>(m_cfg.tileWidth),
                                                  static_cast<float>(m_cfg.tileHeight), m_elev);
      consumeLeft = true;
    }

    // If the cursor is over the minimap, don't start any world interactions on press.
    if (over && leftPressed) consumeLeft = true;
  } else {
    m_minimapDragActive = false;
  }

  // Road tool: Shift+drag plans a cheapest (money cost) road path (includes upgrades/bridges)
  // and commits the whole path on release (single undoable stroke).
  const bool roadDragMode = (m_tool == Tool::Road) && shift && !m_painting && !consumeLeft;

  if (roadDragMode) {
    auto computePathEconomy = [&](const std::vector<Point>& path, int& outNewTiles, int& outUpgrades, int& outBridgeTiles, int& outCost) {
      outNewTiles = 0;
      outUpgrades = 0;
      outBridgeTiles = 0;
      outCost = 0;

      const int targetLevel = ClampRoadLevel(m_roadBuildLevel);

      for (const Point& p : path) {
        if (!m_world.inBounds(p.x, p.y)) continue;
        const Tile& t = m_world.at(p.x, p.y);
        const bool isBridge = (t.terrain == Terrain::Water);

        if (t.overlay == Overlay::Road) {
          const int cur = ClampRoadLevel(static_cast<int>(t.level));
          if (cur < targetLevel) {
            outUpgrades += 1;
            if (isBridge) outBridgeTiles += 1;
            outCost += RoadPlacementCost(cur, targetLevel, /*alreadyRoad=*/true, isBridge);
          }
        } else if (t.overlay == Overlay::None) {
          outNewTiles += 1;
          if (isBridge) outBridgeTiles += 1;
          outCost += RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
        }
      }
    };

    // Road planner config: money-aware and bridge-aware.
    RoadBuildPathConfig planCfg;
    planCfg.allowBridges = true;
    planCfg.costModel = RoadBuildPathConfig::CostModel::Money;

    // Start drag.
    if (leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Point start = *m_hovered;

      std::vector<Point> tmp;
      planCfg.targetLevel = m_roadBuildLevel;
      if (!FindRoadBuildPath(m_world, start, start, tmp, nullptr, planCfg)) {
        showToast("Can't start a road path here", 2.5f);
      } else {
        endPaintStroke();
        m_roadDragActive = true;
        m_roadDragStart = start;
        m_roadDragEnd = start;
        m_roadDragPath = std::move(tmp);
        computePathEconomy(m_roadDragPath, m_roadDragBuildCost, m_roadDragUpgradeTiles, m_roadDragBridgeTiles, m_roadDragMoneyCost);
        m_roadDragValid = true;
      }
    }

    // Update preview.
    if (leftDown && m_roadDragActive && m_roadDragStart && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Point end = *m_hovered;
      if (!m_roadDragEnd || (m_roadDragEnd->x != end.x) || (m_roadDragEnd->y != end.y)) {
        m_roadDragEnd = end;

        std::vector<Point> tmp;
        planCfg.targetLevel = m_roadBuildLevel;
        const bool ok = FindRoadBuildPath(m_world, *m_roadDragStart, end, tmp, nullptr, planCfg);
        if (ok && !tmp.empty()) {
          m_roadDragValid = true;
          m_roadDragPath = std::move(tmp);
          computePathEconomy(m_roadDragPath, m_roadDragBuildCost, m_roadDragUpgradeTiles, m_roadDragBridgeTiles, m_roadDragMoneyCost);
        } else {
          m_roadDragValid = false;
          m_roadDragPath.clear();
          m_roadDragBuildCost = 0;
          m_roadDragUpgradeTiles = 0;
          m_roadDragBridgeTiles = 0;
          m_roadDragMoneyCost = 0;
        }
      }
    }

    // Commit on release.
    if (leftReleased && m_roadDragActive) {
      if (m_roadDragValid && !m_roadDragPath.empty()) {
        const int moneyBefore = m_world.stats().money;

        // Make the road-drag tool atomic: if we cannot afford the whole plan, don't build a partial path.
        if (m_roadDragMoneyCost > moneyBefore) {
          showToast(TextFormat("Not enough funds for planned path: need $%d (short $%d)",
                               m_roadDragMoneyCost, m_roadDragMoneyCost - moneyBefore),
                    3.0f);
        } else {
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
            const int bridges = m_roadDragBridgeTiles;
            if (spent > 0) {
              if (bridges > 0) {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded, %d bridge tiles, cost %d)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles, bridges,
                                     spent));
              } else {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded, cost %d)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles, spent));
              }
            } else {
              if (bridges > 0) {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded, %d bridge tiles)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles, bridges));
              } else {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles));
              }
            }
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
      m_roadDragUpgradeTiles = 0;
      m_roadDragBridgeTiles = 0;
      m_roadDragMoneyCost = 0;
      m_roadDragValid = false;
    }
  }

  // Inspect click: select tile and (if possible) compute the shortest road path to the map edge.
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::Inspect) {
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

  // District tool: Alt+click to pick the hovered tile's district ID (avoids accidental repainting).
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::District
      && (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))) {
    const Tile& t = m_world.at(m_hovered->x, m_hovered->y);
    m_activeDistrict = static_cast<int>(t.district) % kDistrictCount;
    showToast(TextFormat("Picked district: %d", m_activeDistrict));
    consumeLeft = true;
  }

  // District tool: Shift+click flood fills a region.
  // Ctrl+Shift allows the flood to cross roads when filling land blocks.
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::District
      && shift && !(IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))) {
    floodFillDistrict(*m_hovered, ctrl);
    consumeLeft = true;
  }

  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    beginPaintStroke();
  }

  if (!consumeLeft && !roadDragMode && leftDown && m_painting && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    applyToolBrush(m_hovered->x, m_hovered->y);
  }

  if (!consumeLeft && !roadDragMode && leftReleased) {
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

void Game::updateAutosave(float dt)
{
  if (!m_autosaveEnabled) return;
  if (m_painting) return;

  m_autosaveTimer += dt;
  if (m_autosaveTimer < kAutosaveIntervalSec) return;

  // Avoid spamming identical autosaves (e.g., if the sim is paused).
  const int day = m_world.stats().day;
  if (day == m_lastAutosaveDay) {
    m_autosaveTimer = kAutosaveIntervalSec;
    return;
  }

  // Rotate through autosave slots.
  const int slot = std::clamp(m_autosaveNextSlot, kAutosaveSlotMin, kAutosaveSlotMax);
  const std::string path = autosavePathForSlot(slot);

  // Best effort: autosaves should never disrupt gameplay.
  std::string err;
  if (SaveWorldBinary(m_world, m_procCfg, m_sim.config(), path, err)) {
    const std::string thumb = thumbPathForSavePath(path);
    (void)m_renderer.exportMinimapThumbnail(m_world, thumb.c_str(), 256);

    m_lastAutosaveDay = day;
    m_autosaveNextSlot = (slot >= kAutosaveSlotMax) ? kAutosaveSlotMin : (slot + 1);
    m_autosaveTimer = 0.0f;

    // Avoid toasts when the save menu is open; the list itself is feedback.
    if (!m_showSaveMenu) {
      showToast(TextFormat("Autosaved (slot %d)", slot), 1.5f);
    }

    if (m_showSaveMenu) refreshSaveMenu();
  } else {
    // If autosave fails, back off a bit to avoid hammering the filesystem.
    m_autosaveTimer = kAutosaveIntervalSec * 0.75f;
  }
}

void Game::update(float dt)
{
  // Pause simulation while actively painting so an undoable "stroke" doesn't
  // accidentally include sim-driven money changes.
  if (!m_painting && !m_simPaused) {
    const int si = std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1);
    const float speed = kSimSpeeds[static_cast<std::size_t>(si)];

    std::vector<Stats> tickStats;
    tickStats.reserve(4);
    const int ticks = m_sim.update(m_world, dt * speed, &tickStats);

    if (ticks > 0) {
      // The sim advanced 1..N ticks. These derived overlays depend on occupants/jobs.
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;

      for (const Stats& s : tickStats) {
        recordHistorySample(s);
      }
    }
  }

  if (m_toastTimer > 0.0f) {
    m_toastTimer -= dt;
    if (m_toastTimer < 0.0f) m_toastTimer = 0.0f;
  }

  // Update vehicle visualization (movement pauses when sim is paused or while painting).
  const float vdt = (!m_painting && !m_simPaused) ? dt : 0.0f;
  updateVehicles(vdt);

  // Autosave uses wall-clock time (so it works regardless of sim speed).
  updateAutosave(dt);

  // Save menu housekeeping.
  if (m_saveMenuDeleteArmed) {
    m_saveMenuDeleteTimer -= dt;
    if (m_saveMenuDeleteTimer <= 0.0f) {
      m_saveMenuDeleteTimer = 0.0f;
      m_saveMenuDeleteArmed = false;
    }
  }

  if (m_showSaveMenu) {
    m_saveMenuRefreshTimer += dt;
    if (m_saveMenuRefreshTimer >= 1.0f) {
      m_saveMenuRefreshTimer = 0.0f;
      refreshSaveMenu();
    }
  } else {
    m_saveMenuRefreshTimer = 0.0f;
  }

  // Optional dynamic resolution scaling for the world layer.
  updateDynamicWorldRenderScale(dt);
}


namespace {

template <typename F>
void DrawHistoryGraph(const std::vector<CityHistorySample>& samples, Rectangle r, const char* title, F getValue,
                      float fixedMin, float fixedMax, bool fixedRange, const char* valueFmt, bool percent = false)
{
  DrawRectangleRec(r, Color{0, 0, 0, 150});
  DrawRectangleLinesEx(r, 1, Color{255, 255, 255, 60});

  const int pad = 10;
  const int fontTitle = 18;
  const int fontSmall = 14;

  DrawText(title, static_cast<int>(r.x) + pad, static_cast<int>(r.y) + 6, fontTitle, RAYWHITE);

  if (samples.size() < 2) {
    DrawText("(no history yet)", static_cast<int>(r.x) + pad, static_cast<int>(r.y) + 30, fontSmall,
             Color{220, 220, 220, 255});
    return;
  }

  const std::size_t n = samples.size();

  // Compute min/max (auto) on the visible window.
  float vmin = fixedMin;
  float vmax = fixedMax;
  if (!fixedRange) {
    vmin = getValue(samples[0]);
    vmax = vmin;
    for (std::size_t i = 1; i < n; ++i) {
      const float v = getValue(samples[i]);
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
    }
    if (std::abs(vmax - vmin) < 1e-6f) {
      vmax = vmin + 1.0f;
    } else {
      // Add a small padding so the line doesn't sit exactly on the border.
      const float padv = 0.05f * (vmax - vmin);
      vmin -= padv;
      vmax += padv;
    }
  } else {
    if (std::abs(vmax - vmin) < 1e-6f) vmax = vmin + 1.0f;
  }

  // Graph area (leave space for title and value labels).
  Rectangle gr = r;
  gr.x += pad;
  gr.width -= pad * 2;
  gr.y += 30;
  gr.height -= 44;

  // Grid lines
  const int gridLines = 3;
  for (int i = 0; i <= gridLines; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(gridLines);
    const int y = static_cast<int>(gr.y + t * gr.height);
    DrawLine(static_cast<int>(gr.x), y, static_cast<int>(gr.x + gr.width), y, Color{255, 255, 255, 25});
  }

  auto mapX = [&](std::size_t i) -> float {
    const float t = static_cast<float>(i) / static_cast<float>(n - 1);
    return gr.x + t * gr.width;
  };
  auto mapY = [&](float v) -> float {
    const float t = (v - vmin) / (vmax - vmin);
    return gr.y + (1.0f - std::clamp(t, 0.0f, 1.0f)) * gr.height;
  };

  // Polyline
  for (std::size_t i = 1; i < n; ++i) {
    const float x0 = mapX(i - 1);
    const float y0 = mapY(getValue(samples[i - 1]));
    const float x1 = mapX(i);
    const float y1 = mapY(getValue(samples[i]));
    DrawLineEx(Vector2{x0, y0}, Vector2{x1, y1}, 2.0f, Color{120, 220, 255, 200});
  }

  // Labels (min/max + latest)
  char buf[96];
  const float latest = getValue(samples.back());
  if (percent) {
    std::snprintf(buf, sizeof(buf), valueFmt, static_cast<double>(latest * 100.0f));
  } else {
    std::snprintf(buf, sizeof(buf), valueFmt, static_cast<double>(latest));
  }
  DrawText(buf, static_cast<int>(r.x) + pad, static_cast<int>(r.y + r.height) - 18, fontSmall,
           Color{230, 230, 230, 255});
}

const char* ReportPageName(int page)
{
  switch (page) {
    default:
    case 0: return "Overview";
    case 1: return "Economy";
    case 2: return "Traffic";
    case 3: return "Land & Goods";
    case 4: return "Districts";
  }
}

} // namespace

void Game::drawReportPanel(int /*screenW*/, int /*screenH*/)
{
  if (!m_showReport) return;

  const int panelW = 520;
  const int panelH = 420;

  const int x0 = 12;
  const int y0 = 96;

  DrawRectangle(x0, y0, panelW, panelH, Color{0, 0, 0, 180});
  DrawRectangleLines(x0, y0, panelW, panelH, Color{255, 255, 255, 70});

  int x = x0 + 12;
  int y = y0 + 10;

  DrawText("City Report", x, y, 20, RAYWHITE);
  y += 24;

  DrawText(TextFormat("Page: %s   Tab: cycle   F1: toggle", ReportPageName(m_reportPage)), x, y, 16,
           Color{220, 220, 220, 255});
  y += 24;

  // Display a fixed window: last N days (bounded by stored history).
  const int maxPoints = 120;
  const int count = static_cast<int>(m_cityHistory.size());
  const int start = (count > maxPoints) ? (count - maxPoints) : 0;
  const std::vector<CityHistorySample> view(m_cityHistory.begin() + start, m_cityHistory.end());

  Rectangle r1{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24), 96};
  Rectangle r2{static_cast<float>(x0 + 12), static_cast<float>(y + 104), static_cast<float>(panelW - 24), 96};
  Rectangle r3{static_cast<float>(x0 + 12), static_cast<float>(y + 208), static_cast<float>(panelW - 24), 96};

  if (m_reportPage == 0) {
    DrawHistoryGraph(view, r1, "Population", [](const CityHistorySample& s) { return static_cast<float>(s.population); },
                     0.0f, 0.0f, false, "Latest: %.0f");
    DrawHistoryGraph(view, r2, "Happiness", [](const CityHistorySample& s) { return s.happiness; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
    DrawHistoryGraph(view, r3, "Residential demand", [](const CityHistorySample& s) { return s.demandResidential; }, 0.0f,
                     1.0f, true, "Latest: %.0f%%", true);
  } else if (m_reportPage == 1) {
    DrawHistoryGraph(view, r1, "Money", [](const CityHistorySample& s) { return static_cast<float>(s.money); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
    DrawHistoryGraph(view, r2, "Income", [](const CityHistorySample& s) { return static_cast<float>(s.income); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
    DrawHistoryGraph(view, r3, "Expenses", [](const CityHistorySample& s) { return static_cast<float>(s.expenses); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
  } else if (m_reportPage == 2) {
    DrawHistoryGraph(view, r1, "Commuters", [](const CityHistorySample& s) { return static_cast<float>(s.commuters); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
    DrawHistoryGraph(view, r2, "Avg commute (time)", [](const CityHistorySample& s) { return s.avgCommuteTime; }, 0.0f, 0.0f, false,
                     "Latest: %.1f");
    DrawHistoryGraph(view, r3, "Congestion", [](const CityHistorySample& s) { return s.trafficCongestion; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
  } else if (m_reportPage == 3) {
    DrawHistoryGraph(view, r1, "Avg land value", [](const CityHistorySample& s) { return s.avgLandValue; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
    DrawHistoryGraph(view, r2, "Tax per capita", [](const CityHistorySample& s) { return s.avgTaxPerCapita; }, 0.0f, 0.0f, false,
                     "Latest: %.2f");
    DrawHistoryGraph(view, r3, "Goods satisfaction", [](const CityHistorySample& s) { return s.goodsSatisfaction; }, 0.0f, 1.0f,
                     true, "Latest: %.0f%%", true);
  } else {
    // Districts
    const SimConfig& cfg = m_sim.config();
    const int w = m_world.width();
    const int h = m_world.height();
    const int n = w * h;

    const std::vector<float>* lv = (static_cast<int>(m_landValue.value.size()) == n) ? &m_landValue.value : nullptr;
    const DistrictStatsResult ds = ComputeDistrictStats(m_world, cfg, lv, nullptr);

    const int headerY = y0 + 70;
    const int tableX = x0 + 12;
    const int rowH = 20;
    const int font = 16;

    auto drawR = [&](int xRight, int yDraw, const char* text, Color c) {
      const int tw = MeasureText(text, font);
      DrawText(text, xRight - tw, yDraw, font, c);
    };

    DrawText("ID", tableX, headerY, font, Color{220, 220, 220, 255});
    DrawText("Pop", tableX + 40, headerY, font, Color{220, 220, 220, 255});
    DrawText("Emp", tableX + 120, headerY, font, Color{220, 220, 220, 255});
    DrawText("Net", tableX + 200, headerY, font, Color{220, 220, 220, 255});
    DrawText("LV", tableX + 280, headerY, font, Color{220, 220, 220, 255});
    DrawText("Acc", tableX + 350, headerY, font, Color{220, 220, 220, 255});

    const int rowStartY = headerY + 18;
    for (int d = 0; d < kDistrictCount; ++d) {
      const DistrictSummary& s = ds.districts[static_cast<std::size_t>(d)];
      const int rowY = rowStartY + d * rowH;

      if (d == std::clamp(m_activeDistrict, 0, kDistrictCount - 1)) {
        DrawRectangle(x0 + 6, rowY - 2, panelW - 12, rowH, Color{255, 255, 255, 25});
      }

      DrawText(TextFormat("%d", d), tableX, rowY, font, RAYWHITE);
      drawR(tableX + 40 + 70, rowY, TextFormat("%d", s.population), RAYWHITE);
      drawR(tableX + 120 + 70, rowY, TextFormat("%d", s.employed), RAYWHITE);
      drawR(tableX + 200 + 70, rowY, TextFormat("%+d", s.net), (s.net < 0) ? Color{255, 120, 120, 255} : Color{160, 255, 160, 255});
      drawR(tableX + 280 + 50, rowY, TextFormat("%.0f%%", static_cast<double>(s.avgLandValue) * 100.0), RAYWHITE);
      if (s.zoneTiles > 0) {
        const double accPct = 100.0 * static_cast<double>(s.zoneTilesAccessible) / static_cast<double>(s.zoneTiles);
        drawR(tableX + 350 + 60, rowY, TextFormat("%.0f%%", accPct), RAYWHITE);
      } else {
        drawR(tableX + 350 + 60, rowY, "--", Color{200, 200, 200, 255});
      }
    }

    // Totals row
    const int totalsY = rowStartY + kDistrictCount * rowH + 6;
    DrawLine(x0 + 8, totalsY - 4, x0 + panelW - 8, totalsY - 4, Color{255, 255, 255, 60});
    DrawText("All", tableX, totalsY, font, Color{220, 220, 220, 255});
    drawR(tableX + 40 + 70, totalsY, TextFormat("%d", ds.total.population), Color{220, 220, 220, 255});
    drawR(tableX + 120 + 70, totalsY, TextFormat("%d", ds.total.employed), Color{220, 220, 220, 255});
    drawR(tableX + 200 + 70, totalsY, TextFormat("%+d", ds.total.net), (ds.total.net < 0) ? Color{255, 120, 120, 255} : Color{160, 255, 160, 255});
    drawR(tableX + 280 + 50, totalsY, TextFormat("%.0f%%", static_cast<double>(ds.total.avgLandValue) * 100.0), Color{220, 220, 220, 255});

    // Detail line for selected district
    const int dSel = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
    const DistrictSummary& sel = ds.districts[static_cast<std::size_t>(dSel)];
    const int detailY = totalsY + 26;
    DrawText(TextFormat("D%d: tax %d  maint %d (roads %d, parks %d)", dSel, sel.taxRevenue, sel.maintenanceCost,
                        sel.roadMaintenanceCost, sel.parkMaintenanceCost),
             x0 + 12, detailY, 14, Color{220, 220, 220, 255});
    DrawText("Note: district budget excludes trade, upgrades, and one-off build costs.", x0 + 12, detailY + 18, 14,
             Color{200, 200, 200, 255});
  }

  // Footer: show day range
  if (!view.empty()) {
    const int d0 = view.front().day;
    const int d1 = view.back().day;
    DrawText(TextFormat("Days: %d..%d (showing %d / stored %d)", d0, d1, static_cast<int>(view.size()),
                        static_cast<int>(m_cityHistory.size())),
             x0 + 12, y0 + panelH - 22, 14, Color{220, 220, 220, 255});
  }
}

void Game::drawVideoSettingsPanel(int uiW, int uiH)
{
  if (!m_showVideoSettings) return;

  // Currently, the panel layout only needs the UI height; keep the width
  // parameter for possible future responsive layouts.
  static_cast<void>(uiW);

  const int panelW = 520;
  const int rowH = 22;
  const int rows = 11;
  const int panelH = 10 + 24 + 24 + rows * rowH + 28;

  const int x0 = 12;
  int y0 = 96;

  // Avoid overlapping the report panel (which also lives on the left).
  if (m_showReport) {
    y0 += 420 + 12;
  }

  // Clamp to screen height.
  if (y0 + panelH > uiH - 12) {
    y0 = std::max(12, uiH - panelH - 12);
  }

  DrawRectangle(x0, y0, panelW, panelH, Color{0, 0, 0, 180});
  DrawRectangleLines(x0, y0, panelW, panelH, Color{255, 255, 255, 70});

  int x = x0 + 12;
  int y = y0 + 10;

  DrawText("Video / Display", x, y, 20, RAYWHITE);
  y += 24;
  DrawText("Tab: select    [ / ]: adjust/toggle    Ctrl+Alt +/-: world scale    F8: toggle", x, y, 16,
           Color{220, 220, 220, 255});
  y += 24;

  auto drawRow = [&](int idx, const char* label, const std::string& value, bool dim = false) {
    const bool selected = (m_videoSelection == idx);
    if (selected) {
      DrawRectangle(x0 + 6, y - 2, panelW - 12, rowH, Color{255, 255, 255, 28});
    }

    Color c = dim ? Color{170, 170, 170, 255} : Color{220, 220, 220, 255};
    if (selected) {
      c = RAYWHITE;
    }

    DrawText(label, x, y, 16, c);
    const int valW = MeasureText(value.c_str(), 16);
    DrawText(value.c_str(), x0 + panelW - 12 - valW, y, 16, c);
    y += rowH;
  };

  // 0..10 must match adjustVideoSettings() and Tab cycling.
  drawRow(0, "Fullscreen", IsWindowFullscreen() ? "On" : "Off");
  drawRow(1, "Borderless windowed", m_borderlessWindowed ? "On" : "Off");
  drawRow(2, "VSync", m_cfg.vsync ? "On" : "Off");
  drawRow(3, "UI scale mode", m_uiScaleAuto ? "Auto" : "Manual");
  drawRow(4, "UI scale", TextFormat("%.2fx", m_uiScale), !m_uiScaleAuto);

  drawRow(5, "World render mode", m_worldRenderScaleAuto ? "Auto" : "Manual");
  drawRow(6, "World render scale", TextFormat("%.0f%%", m_worldRenderScale * 100.0f), m_worldRenderScaleAuto);
  drawRow(7, "World scale min", TextFormat("%.0f%%", m_worldRenderScaleMin * 100.0f), !m_worldRenderScaleAuto);
  drawRow(8, "World scale max", TextFormat("%.0f%%", m_worldRenderScaleMax * 100.0f), !m_worldRenderScaleAuto);
  drawRow(9, "World target FPS", TextFormat("%d", m_worldRenderTargetFps), !m_worldRenderScaleAuto);
  drawRow(10, "World filter", m_worldRenderFilterPoint ? "Point" : "Bilinear");

  // Footer: show current effective world RT size and smoothed FPS.
  const float fps = 1.0f / std::max(0.0001f, m_frameTimeSmoothed);
  const char* rtStr = wantsWorldRenderTarget() ? TextFormat("%dx%d", m_worldRenderRTWidth, m_worldRenderRTHeight) : "native";
  DrawText(TextFormat("Smoothed FPS: %.1f    World RT: %s", fps, rtStr), x0 + 12, y0 + panelH - 22, 14,
           Color{220, 220, 220, 255});
}


void Game::draw()
{
  BeginDrawing();
  ClearBackground(Color{30, 32, 38, 255});

  const int screenW = GetScreenWidth();
  const int screenH = GetScreenHeight();
  const float uiScale = m_uiScale;
  const int uiW = static_cast<int>(std::round(static_cast<float>(screenW) / uiScale));
  const int uiH = static_cast<int>(std::round(static_cast<float>(screenH) / uiScale));

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

  const bool heatmapActive = (m_heatmapOverlay != HeatmapOverlay::Off);
  const bool districtStatsActive = m_showDistrictPanel || (m_showReport && m_reportPage == 4);

  // Many derived systems need the "road component touches map edge" mask.
  // This should be computed regardless of whether the connectivity overlay is *drawn*.
  const bool requireOutside = m_sim.config().requireOutsideConnection;
  const bool needRoadToEdgeMask = requireOutside &&
                                (m_showOutsideOverlay || m_showTrafficOverlay || m_showGoodsOverlay || heatmapActive || districtStatsActive);

  const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
  if (needRoadToEdgeMask) {
    ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
    roadToEdgeMask = &m_outsideOverlayRoadToEdge;
  }

  // Only pass the mask to the renderer if the user wants the overlay.
  const std::vector<std::uint8_t>* outsideMask = m_showOutsideOverlay ? roadToEdgeMask : nullptr;

  // Traffic is used by both the explicit traffic overlay and the land value heatmap.
  const bool needTrafficResult = (m_showTrafficOverlay || heatmapActive || districtStatsActive);
  if (needTrafficResult && m_trafficDirty) {
    const float share = (m_world.stats().population > 0)
                            ? (static_cast<float>(m_world.stats().employed) /
                               static_cast<float>(m_world.stats().population))
                            : 0.0f;

    TrafficConfig tc;
    tc.requireOutsideConnection = requireOutside;
    {
      const TrafficModelSettings& tm = m_sim.trafficModel();
      tc.congestionAwareRouting = tm.congestionAwareRouting;
      tc.congestionIterations = tm.congestionIterations;
      tc.congestionAlpha = tm.congestionAlpha;
      tc.congestionBeta = tm.congestionBeta;
      tc.congestionCapacityScale = tm.congestionCapacityScale;
      tc.congestionRatioClamp = tm.congestionRatioClamp;
    }

    const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? roadToEdgeMask : nullptr);
    m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
    m_trafficDirty = false;
  }

  const std::vector<std::uint16_t>* trafficMask = nullptr;
  int trafficMax = 0;
  if (m_showTrafficOverlay && !m_traffic.roadTraffic.empty()) {
    trafficMask = &m_traffic.roadTraffic;
    trafficMax = m_traffic.maxTraffic;
  }

  const std::vector<std::uint16_t>* goodsTrafficMask = nullptr;
  int goodsMax = 0;
  const std::vector<std::uint8_t>* commercialGoodsFill = nullptr;
  if (m_showGoodsOverlay) {
    if (m_goodsDirty) {
      GoodsConfig gc;
      gc.requireOutsideConnection = requireOutside;

      const std::vector<std::uint8_t>* pre = (gc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;
    }

    goodsTrafficMask = &m_goods.roadGoodsTraffic;
    goodsMax = m_goods.maxRoadGoodsTraffic;
    commercialGoodsFill = &m_goods.commercialFill;
  }

  // --- Land value (heatmap + district stats) ---
  const bool needLandValueResult = heatmapActive || districtStatsActive;
  if (needLandValueResult) {
    if (m_landValueDirty ||
        m_landValue.value.size() != static_cast<std::size_t>(std::max(0, m_world.width()) * std::max(0, m_world.height()))) {
      LandValueConfig lc;
      lc.requireOutsideConnection = requireOutside;
      const TrafficResult* tptr = needTrafficResult ? &m_traffic : nullptr;
      m_landValue = ComputeLandValue(m_world, lc, tptr, roadToEdgeMask);
      m_landValueDirty = false;
    }
  }

  // --- Heatmap overlay (land value + component fields) ---
  const std::vector<float>* heatmap = nullptr;
  Renderer::HeatmapRamp heatmapRamp = Renderer::HeatmapRamp::Good;
  const char* heatmapName = nullptr;

  if (heatmapActive) {
    switch (m_heatmapOverlay) {
    case HeatmapOverlay::LandValue:
      heatmapName = "Land value";
      heatmap = &m_landValue.value;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::ParkAmenity:
      heatmapName = "Park amenity";
      heatmap = &m_landValue.parkAmenity;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WaterAmenity:
      heatmapName = "Water amenity";
      heatmap = &m_landValue.waterAmenity;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::Pollution:
      heatmapName = "Pollution";
      heatmap = &m_landValue.pollution;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::TrafficSpill:
      heatmapName = "Traffic spill";
      heatmap = &m_landValue.traffic;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    default: break;
    }
  }

  // District overlay rendering controls.
  const bool showDistrictOverlay = (m_showDistrictOverlay || m_showDistrictPanel || (m_tool == Tool::District));
  const int highlightDistrict = showDistrictOverlay ? std::clamp(m_activeDistrict, 0, kDistrictCount - 1) : -1;
  const bool showDistrictBorders = showDistrictOverlay && m_showDistrictBorders;

  // World pass: optionally render to an offscreen target for resolution scaling.
  if (wantsWorldRenderTarget()) {
    ensureWorldRenderTarget(screenW, screenH);
  }

  auto drawWorldDirect = [&]() {
    m_renderer.drawWorld(m_world, m_camera, screenW, screenH, m_timeSec, m_hovered, m_drawGrid, worldBrush, selected,
                         pathPtr, outsideMask,
                         trafficMask, trafficMax,
                         goodsTrafficMask, goodsMax,
                         commercialGoodsFill,
                         heatmap, heatmapRamp,
                         showDistrictOverlay, highlightDistrict, showDistrictBorders);
  };

  if (!wantsWorldRenderTarget() || !m_worldRenderRTValid) {
    drawWorldDirect();
  } else {
    Camera2D camRT = m_camera;
    camRT.zoom = m_camera.zoom * m_worldRenderScale;
    camRT.offset.x = m_camera.offset.x * m_worldRenderScale;
    camRT.offset.y = m_camera.offset.y * m_worldRenderScale;

    BeginTextureMode(m_worldRenderRT);
    ClearBackground(Color{30, 32, 38, 255});

    m_renderer.drawWorld(m_world, camRT, m_worldRenderRTWidth, m_worldRenderRTHeight, m_timeSec, m_hovered,
                         m_drawGrid, worldBrush, selected,
                         pathPtr, outsideMask,
                         trafficMask, trafficMax,
                         goodsTrafficMask, goodsMax,
                         commercialGoodsFill,
                         heatmap, heatmapRamp,
                         showDistrictOverlay, highlightDistrict, showDistrictBorders);

    EndTextureMode();

    const Rectangle src = {0.0f, 0.0f, static_cast<float>(m_worldRenderRTWidth),
                           -static_cast<float>(m_worldRenderRTHeight)};
    const Rectangle dst = {0.0f, 0.0f, static_cast<float>(screenW), static_cast<float>(screenH)};
    DrawTexturePro(m_worldRenderRT.texture, src, dst, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
  }

  // Vehicle micro-sim overlay (commuters + goods trucks).
  drawVehicles();

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
          const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                       static_cast<float>(m_cfg.tileHeight), m_elev);
          const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                       static_cast<float>(m_cfg.tileHeight), m_elev);
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

        const Vector2 wp = TileToWorldCenterElevated(m_world, n.pos.x, n.pos.y, static_cast<float>(m_cfg.tileWidth),
                                                     static_cast<float>(m_cfg.tileHeight), m_elev);
        DrawCircleV(wp, radius, c);
      }

      EndMode2D();
    }
  }

  const float simSpeed = kSimSpeeds[static_cast<std::size_t>(std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1))];
  const char* inspectInfo = (m_tool == Tool::Inspect && !m_inspectInfo.empty()) ? m_inspectInfo.c_str() : nullptr;

  std::string heatmapInfo;
  if (heatmapActive && heatmapName && heatmap && m_hovered &&
      heatmap->size() == static_cast<std::size_t>(std::max(0, m_world.width()) * std::max(0, m_world.height()))) {
    const std::size_t idx = static_cast<std::size_t>(m_hovered->y) * static_cast<std::size_t>(m_world.width()) +
                            static_cast<std::size_t>(m_hovered->x);
    const float hv = (*heatmap)[idx];
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Heatmap: %s  %.2f", heatmapName, static_cast<double>(hv));
    heatmapInfo = buf;
  } else if (heatmapActive && heatmapName) {
    heatmapInfo = std::string("Heatmap: ") + heatmapName;
  }

  const char* heatmapInfoC = (!heatmapInfo.empty()) ? heatmapInfo.c_str() : nullptr;

  // ---------------------------------------------------------------------
  // UI (scaled)
  // ---------------------------------------------------------------------
  // The world is rendered at full resolution, but the UI is rendered in a
  // "virtual" coordinate system and scaled up/down. This keeps UI text
  // readable and panels sized consistently across resolutions and DPI.
  Camera2D uiCam{};
  uiCam.zoom = uiScale;
  BeginMode2D(uiCam);

  m_renderer.drawHUD(m_world, m_camera, m_tool, m_roadBuildLevel, m_hovered, uiW, uiH, m_showHelp,
                     m_brushRadius, static_cast<int>(m_history.undoSize()), static_cast<int>(m_history.redoSize()),
                     m_simPaused, simSpeed, m_saveSlot, m_showMinimap, inspectInfo, heatmapInfoC);

  // Policy / budget panel (simple keyboard-driven UI).
  if (m_showPolicy) {
    const SimConfig& cfg = m_sim.config();
    const Stats& st = m_world.stats();

    const int panelW = 420;
    const int panelH = 280;
    const int x0 = uiW - panelW - 12;
    const int y0 = 96;

    DrawRectangle(x0, y0, panelW, panelH, Color{0, 0, 0, 180});
    DrawRectangleLines(x0, y0, panelW, panelH, Color{255, 255, 255, 70});

    int x = x0 + 12;
    int y = y0 + 10;
    DrawText("Policy & Budget", x, y, 20, RAYWHITE);
    y += 24;
    DrawText("Tab: select   [ / ]: adjust   Shift: bigger steps", x, y, 16, Color{220, 220, 220, 255});
    y += 22;

    auto row = [&](int idx, const char* label, const char* value) {
      const bool sel = (m_policySelection == idx);
      if (sel) {
        DrawRectangle(x - 6, y - 2, panelW - 24, 20, Color{255, 255, 255, 40});
      }
      DrawText(TextFormat("%s: %s", label, value), x, y, 18,
               sel ? Color{255, 255, 255, 255} : Color{210, 210, 210, 255});
      y += 22;
    };

    row(0, "Residential tax", TextFormat("%d", cfg.taxResidential));
    row(1, "Commercial tax", TextFormat("%d", cfg.taxCommercial));
    row(2, "Industrial tax", TextFormat("%d", cfg.taxIndustrial));
    row(3, "Road maintenance", TextFormat("%d", cfg.maintenanceRoad));
    row(4, "Park maintenance", TextFormat("%d", cfg.maintenancePark));
    row(5, "Outside connection", cfg.requireOutsideConnection ? "ON" : "OFF");
    row(6, "Park radius", TextFormat("%d", cfg.parkInfluenceRadius));

    y += 4;
    DrawLine(x, y, x0 + panelW - 12, y, Color{255, 255, 255, 70});
    y += 10;

    const int tradeNet = st.exportRevenue - st.importCost;
    const int net = st.income - st.expenses;
    DrawText(TextFormat("Net: %+d   Income: %d   Expenses: %d", net, st.income, st.expenses), x, y, 18, RAYWHITE);
    y += 22;
    DrawText(TextFormat("Tax %d  Maint %d  Upg %d  Trade %+d", st.taxRevenue, st.maintenanceCost, st.upgradeCost, tradeNet),
             x, y, 18, Color{220, 220, 220, 255});
    y += 22;
    DrawText(TextFormat("Land %.0f%%  Demand %.0f%%  Tax/cap %.2f", st.avgLandValue * 100.0f,
                        st.demandResidential * 100.0f, static_cast<double>(st.avgTaxPerCapita)),
             x, y, 18, Color{220, 220, 220, 255});
  }

  // Traffic model panel (experimental, not saved).
  if (m_showTrafficModel) {
    const TrafficModelSettings& tm = m_sim.trafficModel();
    const Stats& st = m_world.stats();

    const int panelW = 420;
    const int panelH = 248;
    const int x0 = uiW - panelW - 12;
    // Stack below policy if both are visible.
    const int y0 = m_showPolicy ? (96 + 280 + 12) : 96;

    DrawRectangle(x0, y0, panelW, panelH, Color{0, 0, 0, 180});
    DrawRectangleLines(x0, y0, panelW, panelH, Color{255, 255, 255, 70});

    int x = x0 + 12;
    int y = y0 + 10;
    DrawText("Traffic Model", x, y, 20, RAYWHITE);
    y += 24;
    DrawText("Tab: select   [ / ]: adjust   Shift: bigger steps", x, y, 16, Color{220, 220, 220, 255});
    y += 22;

    auto row = [&](int idx, const char* label, const char* value) {
      const bool sel = (m_trafficModelSelection == idx);
      if (sel) {
        DrawRectangle(x - 6, y - 2, panelW - 24, 20, Color{255, 255, 255, 40});
      }
      DrawText(TextFormat("%s: %s", label, value), x, y, 18,
               sel ? Color{255, 255, 255, 255} : Color{210, 210, 210, 255});
      y += 22;
    };

    row(0, "Congestion routing", tm.congestionAwareRouting ? "ON" : "OFF");
    row(1, "Passes", TextFormat("%d", tm.congestionIterations));
    row(2, "Alpha", TextFormat("%.2f", static_cast<double>(tm.congestionAlpha)));
    row(3, "Beta", TextFormat("%.1f", static_cast<double>(tm.congestionBeta)));
    row(4, "Cap scale", TextFormat("%.2f", static_cast<double>(tm.congestionCapacityScale)));
    row(5, "Ratio clamp", TextFormat("%.1f", static_cast<double>(tm.congestionRatioClamp)));

    y += 4;
    DrawLine(x, y, x0 + panelW - 12, y, Color{255, 255, 255, 70});
    y += 10;
    DrawText(TextFormat("Avg commute (time): %.1f   Congestion: %.0f%%", static_cast<double>(st.avgCommuteTime),
                        static_cast<double>(st.trafficCongestion * 100.0f)),
             x, y, 18, Color{220, 220, 220, 255});
  }

  // Districts panel (district paint + per-district policy multipliers; saved in v7+).
  if (m_showDistrictPanel) {
    const SimConfig& cfg = m_sim.config();
    const int district = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
    const DistrictPolicy& dp = cfg.districtPolicies[static_cast<std::size_t>(district)];

    const int panelW = 420;
    const int panelH = 308;
    const int x0 = uiW - panelW - 12;
    int y0 = 96;
    if (m_showPolicy) y0 += 280 + 12;
    if (m_showTrafficModel) y0 += 248 + 12;

    DrawRectangle(x0, y0, panelW, panelH, Color{0, 0, 0, 180});
    DrawRectangleLines(x0, y0, panelW, panelH, Color{255, 255, 255, 70});

    int x = x0 + 12;
    int y = y0 + 10;
    DrawText("Districts", x, y, 20, RAYWHITE);
    y += 24;
    DrawText("Tab: select   [ / ]: adjust   Shift: bigger steps", x, y, 16, Color{220, 220, 220, 255});
    y += 22;

    auto row = [&](int idx, const char* label, const char* value) {
      const bool sel = (m_districtSelection == idx);
      if (sel) {
        DrawRectangle(x - 6, y - 2, panelW - 24, 20, Color{255, 255, 255, 40});
      }
      DrawText(TextFormat("%s: %s", label, value), x, y, 18,
               sel ? Color{255, 255, 255, 255} : Color{210, 210, 210, 255});
      y += 22;
    };

    row(0, "Policies enabled", cfg.districtPoliciesEnabled ? "ON" : "OFF");
    row(1, "Active district", (district == 0) ? "0 (Default)" : TextFormat("%d", district));
    row(2, "Overlay", m_showDistrictOverlay ? "ON" : ((m_tool == Tool::District) ? "AUTO (tool)" : "OFF"));
    row(3, "Borders", m_showDistrictBorders ? "ON" : "OFF");

    const int effResTax = static_cast<int>(std::lround(static_cast<double>(cfg.taxResidential) * dp.taxResidentialMult));
    const int effComTax = static_cast<int>(std::lround(static_cast<double>(cfg.taxCommercial) * dp.taxCommercialMult));
    const int effIndTax = static_cast<int>(std::lround(static_cast<double>(cfg.taxIndustrial) * dp.taxIndustrialMult));
    const int effRoadMaint = static_cast<int>(std::lround(static_cast<double>(cfg.maintenanceRoad) * dp.roadMaintenanceMult));
    const int effParkMaint = static_cast<int>(std::lround(static_cast<double>(cfg.maintenancePark) * dp.parkMaintenanceMult));

    row(4, "Res tax mult", TextFormat("x%.2f (eff %d)", static_cast<double>(dp.taxResidentialMult), effResTax));
    row(5, "Com tax mult", TextFormat("x%.2f (eff %d)", static_cast<double>(dp.taxCommercialMult), effComTax));
    row(6, "Ind tax mult", TextFormat("x%.2f (eff %d)", static_cast<double>(dp.taxIndustrialMult), effIndTax));
    row(7, "Road maint mult", TextFormat("x%.2f (eff %d)", static_cast<double>(dp.roadMaintenanceMult), effRoadMaint));
    row(8, "Park maint mult", TextFormat("x%.2f (eff %d)", static_cast<double>(dp.parkMaintenanceMult), effParkMaint));

    y += 4;
    DrawLine(x, y, x0 + panelW - 12, y, Color{255, 255, 255, 70});
    y += 10;
    DrawText("Paint: tool 9.  ,/. change id.  Alt+Click pick.  Shift+Click fill.", x, y, 16,
             Color{220, 220, 220, 255});

    // Quick live snapshot for the selected district (uses cached land value when available).
    y += 18;
    const int w = m_world.width();
    const int h = m_world.height();
    const int n = w * h;
    const std::vector<float>* lv = (static_cast<int>(m_landValue.value.size()) == n) ? &m_landValue.value : nullptr;
    const DistrictStatsResult ds = ComputeDistrictStats(m_world, cfg, lv, roadToEdgeMask);
    const DistrictSummary& s = ds.districts[static_cast<std::size_t>(district)];
    const double lvPct = static_cast<double>(s.avgLandValue) * 100.0;
    const double accPct = (s.zoneTiles > 0) ? (100.0 * static_cast<double>(s.zoneTilesAccessible) / static_cast<double>(s.zoneTiles)) : -1.0;
    if (accPct >= 0.0) {
      DrawText(TextFormat("Stats: Pop %d  Emp %d  LV %.0f%%  Net %+d  Acc %.0f%%", s.population, s.employed, lvPct, s.net, accPct), x, y,
               16, Color{220, 220, 220, 255});
    } else {
      DrawText(TextFormat("Stats: Pop %d  Emp %d  LV %.0f%%  Net %+d  Acc --", s.population, s.employed, lvPct, s.net), x, y, 16,
               Color{220, 220, 220, 255});
    }
  }

  drawVideoSettingsPanel(uiW, uiH);

  drawReportPanel(uiW, uiH);

  // Save manager panel draws on top of the HUD.
  drawSaveMenuPanel(uiW, uiH);

  // Road-drag overlay: show preview metrics without touching the HUD layout.
  if (m_roadDragActive) {
    const int fontSize = 18;
    const int pad = 8;

    const char* line1 = nullptr;
    const char* line2 = nullptr;
    char buf1[160];
    char buf2[160];
    if (m_roadDragValid && !m_roadDragPath.empty()) {
      std::snprintf(buf1, sizeof(buf1), "Road path (%s): %d tiles", RoadClassName(m_roadBuildLevel),
                    static_cast<int>(m_roadDragPath.size()));

      const int have = m_world.stats().money;
      const bool afford = (m_roadDragMoneyCost <= have);
      const int shortfall = afford ? 0 : (m_roadDragMoneyCost - have);

      if (m_roadDragBridgeTiles > 0) {
        if (afford) {
          std::snprintf(buf2, sizeof(buf2), "New %d  Upg %d  Br %d  Est $%d  (release)", m_roadDragBuildCost,
                        m_roadDragUpgradeTiles, m_roadDragBridgeTiles, m_roadDragMoneyCost);
        } else {
          std::snprintf(buf2, sizeof(buf2), "New %d  Upg %d  Br %d  Est $%d  (need $%d)", m_roadDragBuildCost,
                        m_roadDragUpgradeTiles, m_roadDragBridgeTiles, m_roadDragMoneyCost, shortfall);
        }
      } else {
        if (afford) {
          std::snprintf(buf2, sizeof(buf2), "New %d  Upg %d  Est $%d  (release)", m_roadDragBuildCost,
                        m_roadDragUpgradeTiles, m_roadDragMoneyCost);
        } else {
          std::snprintf(buf2, sizeof(buf2), "New %d  Upg %d  Est $%d  (need $%d)", m_roadDragBuildCost,
                        m_roadDragUpgradeTiles, m_roadDragMoneyCost, shortfall);
        }
      }
      line1 = buf1;
      line2 = buf2;
    } else {
      line1 = "Road path: no route";
      line2 = "Release to cancel";
    }

    const int w1 = MeasureText(line1, fontSize);
    const int w2 = MeasureText(line2, fontSize);
    const int boxW = std::max(w1, w2) + pad * 2;
    const int boxH = fontSize * 2 + pad * 3;

    const int x = uiW - boxW - 12;
    const int y = 44;

    DrawRectangle(x, y, boxW, boxH, Color{0, 0, 0, 160});
    DrawRectangleLines(x, y, boxW, boxH, Color{255, 255, 255, 70});

    DrawText(line1, x + pad, y + pad, fontSize, RAYWHITE);
    DrawText(line2, x + pad, y + pad + fontSize + 6, fontSize, Color{220, 220, 220, 255});
  }

  // Developer console draws above the HUD/panels but below transient toasts.
  if (m_console.isOpen()) {
    m_console.draw(uiW, uiH);
  }

  // Screenshot capture (queued from input so we can capture the freshly rendered frame)
  if (m_pendingScreenshot) {
    TakeScreenshot(m_pendingScreenshotPath.c_str());
    showToast(std::string("Screenshot saved: ") + m_pendingScreenshotPath, 3.0f);
    m_pendingScreenshot = false;
    m_pendingScreenshotPath.clear();
  }

  // Toast / status message
  if (m_toastTimer > 0.0f && !m_toast.empty()) {
    const int fontSize = 18;
    const int pad = 8;
    const int textW = MeasureText(m_toast.c_str(), fontSize);
    const int boxW = textW + pad * 2;
    const int boxH = fontSize + pad * 2;

    const int x = (uiW - boxW) / 2;
    const int y = uiH - boxH - 18;

    DrawRectangle(x, y, boxW, boxH, Color{0, 0, 0, 170});
    DrawRectangleLines(x, y, boxW, boxH, Color{255, 255, 255, 60});
    DrawText(m_toast.c_str(), x + pad, y + pad, fontSize, RAYWHITE);
  }

  EndMode2D();

  // Map export (queued from dev console so we can run the renderer with a valid
  // graphics context). Must run *outside* any active BeginMode2D() to avoid
  // nested mode state.
  if (m_pendingMapExport && !m_pendingMapExportPath.empty()) {
    const std::string path = m_pendingMapExportPath;
    const int maxSize = m_pendingMapExportMaxSize;
    m_pendingMapExport = false;
    m_pendingMapExportPath.clear();
    m_pendingMapExportMaxSize = 4096;

    const bool ok = m_renderer.exportWorldOverview(m_world, path.c_str(), maxSize);
    showToast(ok ? (std::string("Map exported: ") + path)
                 : (std::string("Map export failed: ") + path),
             4.0f);
  }

  EndDrawing();
}

} // namespace isocity
