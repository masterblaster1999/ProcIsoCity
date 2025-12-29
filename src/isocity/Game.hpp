#pragma once

#include "isocity/Config.hpp"
#include "isocity/Console.hpp"
#include "isocity/EditHistory.hpp"
#include "isocity/FlowField.hpp"
#include "isocity/Iso.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
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
  explicit RaylibContext(const Config& cfg, const char* title);
  ~RaylibContext();

  RaylibContext(const RaylibContext&) = delete;
  RaylibContext& operator=(const RaylibContext&) = delete;
};

// Small time-series sample used by the in-game City Report panel.
// Stored in the game layer (not saved yet), derived from World::Stats after each sim tick.
struct CityHistorySample {
  int day = 0;

  int population = 0;
  int money = 0;

  float happiness = 0.0f;          // 0..1
  float demandResidential = 0.0f;  // 0..1

  float avgLandValue = 0.0f;       // 0..1
  float avgTaxPerCapita = 0.0f;

  int income = 0;
  int expenses = 0;
  int taxRevenue = 0;
  int maintenanceCost = 0;

  int commuters = 0;
  float avgCommute = 0.0f;      // road steps
  float avgCommuteTime = 0.0f;  // street-step equivalent travel time
  float trafficCongestion = 0.0f;  // 0..1

  float goodsSatisfaction = 1.0f;  // 0..1
};


class Game {
public:
  explicit Game(Config cfg);
  ~Game();

  void run();

private:
  void resetWorld(std::uint64_t newSeed);
  void applyToolBrush(int centerX, int centerY);
  void showToast(const std::string& msg, float seconds = 2.5f);

  // City report (time-series graphs of key stats).
  void clearHistory();
  void recordHistorySample(const Stats& s);
  void drawReportPanel(int screenW, int screenH);

  // Visual micro-sim: moving vehicles along roads (commute + goods).
  void rebuildVehiclesRoutingCache();
  void updateVehicles(float dt);
  void drawVehicles();

  // Quick save/load supports multiple slots (UI convenience).
  // Slot 1 intentionally uses the legacy filename so existing quick-saves keep working.
  std::string savePathForSlot(int slot) const;
  std::string autosavePathForSlot(int slot) const;
  std::string thumbPathForSavePath(const std::string& savePath) const;
  void cycleSaveSlot(int delta);

  bool saveToPath(const std::string& path, bool makeThumbnail, const char* toastLabel = nullptr);
  bool loadFromPath(const std::string& path, const char* toastLabel = nullptr);

  // Save slot browser / manager UI (toggle with F10).
  void refreshSaveMenu();
  void unloadSaveMenuThumbnails();
  void drawSaveMenuPanel(int screenW, int screenH);

  void updateAutosave(float dt);

  void beginPaintStroke();
  void endPaintStroke();
  void doUndo();
  void doRedo();

  // Debug/developer console (toggle with F4).
  void setupDevConsole();

  void handleInput(float dt);
  void update(float dt);
  void draw();

  // Display helpers
  float computeAutoUiScale(int screenW, int screenH) const;
  Vector2 mouseUiPosition(float uiScale) const;
  void updateUiScaleHotkeys();
  void updateWorldRenderHotkeys();
  void updateDynamicWorldRenderScale(float dt);
  bool wantsWorldRenderTarget() const;
  float clampWorldRenderScale(float scale) const;
  void ensureWorldRenderTarget(int screenW, int screenH);
  void unloadWorldRenderTarget();
  void drawVideoSettingsPanel(float uiW, float uiH);
  void adjustVideoSettings(int dir);
  void toggleFullscreen();
  void toggleBorderlessWindowed();
  void toggleVsync();

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

  // Elevation visualization / terraforming.
  ElevationSettings m_elev{};
  ElevationSettings m_elevDefault{};
  std::vector<float> m_heightSnapshot; // used by SmoothTerrain to be order-independent per stroke

  Tool m_tool = Tool::Road;

  // Road tool: selected road class for placement/upgrades.
  // Uses the same 1..3 levels as road tiles (Street/Avenue/Highway).
  int m_roadBuildLevel = 1;
  int m_brushRadius = 0; // 0 = single tile, 1 = diamond radius 1, etc.

  bool m_painting = false;
  StrokeFeedback m_strokeFeedback{};

  // Per-stroke tile mask: ensure each tile is affected at most once per stroke.
  // This prevents accidental multi-upgrades (zones) when holding the mouse down
  // while the cursor isn't moving.
  int m_strokeApplyW = 0;
  int m_strokeApplyH = 0;
  std::vector<std::uint8_t> m_strokeApplied;
  std::vector<Point> m_tilesEditedThisStroke;

  bool m_showHelp = true;
  bool m_drawGrid = false;

  // Small in-game policy/budget panel (toggle with P).
  // This is deliberately lightweight: no GUI library, just a few configurable integers.
  bool m_showPolicy = false;
  int m_policySelection = 0;

  bool m_showTrafficModel = false;
  int m_trafficModelSelection = 0;

  // District UI (toggle with F7). Districts are a lightweight per-tile label (0..kDistrictCount-1)
  // that can be used by the sim to apply per-district policy multipliers (taxes/maintenance).
  bool m_showDistrictPanel = false;
  int m_districtSelection = 0;
  bool m_showDistrictOverlay = false;
  bool m_showDistrictBorders = true;
  int m_activeDistrict = 1; // painting + editing target (0..kDistrictCount-1)

  // City report panel (toggle with F1): time-series graphs of key stats.
  bool m_showReport = false;
  int m_reportPage = 0;
  std::vector<CityHistorySample> m_cityHistory;
  int m_cityHistoryMax = 240; // max stored days

  // UI minimap overlay (toggle with M). Clicking the minimap recenters the camera.
  bool m_showMinimap = false;
  bool m_minimapDragActive = false;

  // Quick save slot (1..kMaxSaveSlot). Slot 1 == legacy isocity_save.bin.
  int m_saveSlot = 1;

  // Autosave (rotating slots). Stored separately from manual quick-saves.
  bool m_autosaveEnabled = true;
  float m_autosaveTimer = 0.0f;
  int m_autosaveNextSlot = 1;
  int m_lastAutosaveDay = -1;

  // Save slot browser / manager UI.
  bool m_showSaveMenu = false;
  int m_saveMenuGroup = 0;      // 0 = manual slots, 1 = autosaves
  int m_saveMenuSelection = 0;  // index within group
  float m_saveMenuRefreshTimer = 0.0f;

  struct SaveMenuSlot {
    int slot = 1;
    bool autosave = false;
    std::string path;
    std::string thumbPath;
    SaveSummary summary{};
    bool exists = false;
    bool summaryOk = false;
    std::string timeText;
    bool crcChecked = false;
    bool crcOk = true;

    Texture2D thumb{};
    bool thumbLoaded = false;
  };

  std::vector<SaveMenuSlot> m_saveMenuManual;
  std::vector<SaveMenuSlot> m_saveMenuAutos;

  bool m_saveMenuDeleteArmed = false;
  float m_saveMenuDeleteTimer = 0.0f;

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

  // Heatmap overlay: land value / amenities / pollution / traffic spill.
  enum class HeatmapOverlay : std::uint8_t {
    Off = 0,
    LandValue,
    ParkAmenity,
    WaterAmenity,
    Pollution,
    TrafficSpill,
  };

  HeatmapOverlay m_heatmapOverlay = HeatmapOverlay::Off;
  bool m_landValueDirty = true;
  LandValueResult m_landValue;

  // Vehicles overlay: small moving agents representing commuting + goods shipments.
  bool m_showVehicles = false;
  bool m_vehiclesDirty = true;

  enum class VehicleKind : std::uint8_t { Commute, GoodsDelivery, GoodsImport, GoodsExport };

  struct Vehicle {
    VehicleKind kind = VehicleKind::Commute;
    std::vector<Point> path; // road tiles

    // Continuous path position in "tile edges": [0, path.size()-1].
    float s = 0.0f;
    float dir = 1.0f;   // +1 forward, -1 backward (commuters bounce)
    float speed = 6.0f; // tiles/sec

    // Small lateral offset (world pixels) so vehicles don't all draw on top of each other.
    float laneOffset = 0.0f;

    // For commuters: number of direction flips remaining (1 => go there and back).
    int turnsRemaining = 0;
  };

  std::vector<Vehicle> m_vehicles;
  float m_vehicleSpawnAccum = 0.0f;
  std::uint64_t m_vehicleRngState = 0;

  // Cached routing data (rebuilt when m_vehiclesDirty is set).
  std::vector<int> m_commuteJobSources;                // road tile indices
  std::vector<std::pair<int, int>> m_commuteOrigins;   // (roadIdx, commuterWeight)
  std::uint64_t m_commuteOriginWeightTotal = 0;
  RoadFlowField m_commuteField;

  std::vector<int> m_goodsProducerRoads;               // road tile indices
  std::vector<int> m_goodsProducerSupply;              // same length as m_goodsProducerRoads
  std::uint64_t m_goodsProducerWeightTotal = 0;
  RoadFlowField m_goodsProducerField;

  struct GoodsConsumerLite {
    int roadIdx = -1;
    int demand = 0;
    int dist = -1;
    int owner = -1;
  };
  std::vector<GoodsConsumerLite> m_goodsConsumers;
  std::uint64_t m_goodsConsumerWeightTotal = 0;

  std::vector<int> m_goodsEdgeSources;
  RoadFlowField m_goodsEdgeField;

  // Simulation controls (pause/step/speed) are handled at the game layer so the simulator stays simple.
  bool m_simPaused = false;
  int m_simSpeedIndex = 2; // 0.25x, 0.5x, 1x, 2x, ... (default = 1x)

  float m_timeSec = 0.0f;
  std::string m_toast;
  float m_toastTimer = 0.0f;

  // UI scaling (helps readability on very high resolutions / high-DPI displays).
  bool m_uiScaleAuto = true;
  float m_uiScale = 1.0f;

  // World render resolution scaling (terrain + world overlays only). When enabled,
  // the world is rendered to an offscreen target and then scaled to the window.
  bool m_worldRenderScaleAuto = false;
  float m_worldRenderScale = 1.0f;
  float m_worldRenderScaleMin = 0.70f;
  float m_worldRenderScaleMax = 1.00f;
  int m_worldRenderTargetFps = 60;
  bool m_worldRenderFilterPoint = false;

  float m_smoothedFrameTime = 1.0f / 60.0f;
  float m_worldRenderAutoAccum = 0.0f;

  RenderTexture2D m_worldRenderRT{};
  bool m_worldRenderRTValid = false;
  int m_worldRenderRTW = 0;
  int m_worldRenderRTH = 0;

  bool m_showVideoSettings = false;
  int m_videoSelection = 0;

  // Window mode tracking for borderless fullscreen restore.
  bool m_borderlessWindowed = false;
  int m_windowedX = 0;
  int m_windowedY = 0;
  int m_windowedW = 1280;
  int m_windowedH = 720;

  DevConsole m_console;

  // Screenshot capture (queued from input and executed in draw() so the capture includes the rendered frame)
  bool m_pendingScreenshot = false;
  std::string m_pendingScreenshotPath;

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
  int m_roadDragUpgradeTiles = 0;
  int m_roadDragMoneyCost = 0;
  bool m_roadDragValid = false;
};

} // namespace isocity
