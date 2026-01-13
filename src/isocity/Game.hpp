#pragma once

#include "isocity/Config.hpp"
#include "isocity/Console.hpp"
#include "isocity/EditHistory.hpp"
#include "isocity/ReplayCapture.hpp"
#include "isocity/Export.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/EvacuationScenario.hpp"
#include "isocity/FlowField.hpp"
#include "isocity/Blueprint.hpp"
#include "isocity/Iso.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/VisualPrefs.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphRouting.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/RoadResilienceBypass.hpp"
#include "isocity/RoadUpgradePlanner.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/TransitPlanner.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <map>
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
  void invalidateHydrology();
  void applyToolBrush(int centerX, int centerY);
  void floodFillDistrict(Point start, bool includeRoads);
  void floodFillTool(Point start, bool includeRoads);
  void showToast(const std::string& msg, float seconds = 2.5f);

  // City report (time-series graphs of key stats).
  void clearHistory();
  void recordHistorySample(const Stats& s);
  void drawReportPanel(int screenW, int screenH);

  // Visual micro-sim: moving vehicles along roads (commute + goods).
  void rebuildVehiclesRoutingCache();
  void updateVehicles(float dt);
  void appendVehicleSprites(const Camera2D& camera, std::vector<Renderer::WorldSprite>& out);

  // Quick save/load supports multiple slots (UI convenience).
  // Slot 1 intentionally uses the legacy filename so existing quick-saves keep working.
  std::string savePathForSlot(int slot) const;
  std::string autosavePathForSlot(int slot) const;
  std::string thumbPathForSavePath(const std::string& savePath) const;
  void cycleSaveSlot(int delta);

  bool saveToPath(const std::string& path, bool makeThumbnail, const char* toastLabel = nullptr);
  bool loadFromPath(const std::string& path, const char* toastLabel = nullptr);

  // Replace the current in-memory world with a freshly loaded state (save/replay/etc)
  // and reset all derived caches/UI state.
  //
  // If tickStats is provided, it is used to seed the City Report history (useful
  // when applying a replay that contains many ticks).
  void adoptLoadedWorld(World&& loaded, const ProcGenConfig& loadedProcCfg, const SimConfig& loadedSimCfg,
                        const std::string& toastMessage, const std::vector<Stats>* tickStats = nullptr);

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

  // Blueprint (copy/paste stamp) tool (toggle with J).
  void clearBlueprint();
  void updateBlueprintTransformed();
  bool stampBlueprintAt(const Point& anchorTile);
  bool tileBlueprintRect(int x0, int y0, int x1, int y1, bool useLibraryTileset);
  void drawBlueprintOverlay();
  void drawBlueprintPanel(int uiW, int uiH);

  // Blueprint "library": save/load blueprint stamps to disk from inside the app.
  void refreshBlueprintLibrary();
  void ensureBlueprintLibraryPreviewUpToDate();
  bool saveBlueprintToLibrary();
  bool loadBlueprintFromLibrarySelection();
  bool deleteBlueprintFromLibrarySelection();

  // Road resilience overlay + bypass planner (Shift+T).
  void ensureRoadGraphUpToDate();
  void ensureRoadResilienceUpToDate();
  void rebuildRoadResilienceBypasses();
  void drawRoadResilienceOverlay();
  bool applyRoadResilienceBypass(std::size_t idx);

  // Transit planner (auto "bus line" suggestions) + overlay.
  void ensureTransitPlanUpToDate();
  void ensureTransitVizUpToDate();
  void drawTransitOverlay();
  void drawTransitPanel(int x0, int y0);
  void adjustTransitPanel(int dir, bool bigStep);
  void exportTransitArtifacts();

  // Road upgrade planner (suggests street->avenue/highway upgrades) + overlay.
  void ensureRoadUpgradePlanUpToDate();
  void ensureRoadUpgradeSelectedMaskUpToDate();
  void drawRoadUpgradeOverlay();
  void drawRoadUpgradePanel(int x0, int y0);
  void adjustRoadUpgradePanel(int dir, bool bigStep);
  bool applyRoadUpgradePlan();
  void exportRoadUpgradeArtifacts();
  // Evacuation / disaster scenario analysis (hazards + evacuation-to-edge).
  void ensureEvacuationScenarioUpToDate();
  void exportEvacuationArtifacts();

  // Flood + evacuation scenario analysis panel (toggle with F).
  void adjustResiliencePanel(int dir, bool bigStep);




  // Display helpers
  float computeAutoUiScale(int screenW, int screenH) const;
  Vector2 mouseUiPosition(float uiScale) const;
  void updateUiScaleHotkeys();
  void updateWorldRenderHotkeys();
  void updateDynamicWorldRenderScale(float dt);
  bool wantsWorldRenderTarget() const;
  float clampWorldRenderScale(float scale) const;
  void setWorldRenderScale(float scale);
  void setWorldRenderScaleMin(float scaleMin);
  void setWorldRenderScaleMax(float scaleMax);
  void updateWorldRenderFilter();
  void ensureWorldRenderTarget(int screenW, int screenH);
  void unloadWorldRenderTarget();
  void drawVideoSettingsPanel(int uiW, int uiH);
  void adjustVideoSettings(int dir);
  void toggleFullscreen();
  void toggleBorderlessWindowed();
  void toggleVsync();

  // Persisted visual/display preferences (defaults to isocity_visual.json next to the executable).
  VisualPrefs captureVisualPrefs() const;
  void applyVisualPrefs(const VisualPrefs& prefs);
  bool loadVisualPrefsFile(const std::string& path, bool showToast = false);
  bool saveVisualPrefsFile(const std::string& path, bool showToast = false);
  void updateVisualPrefsAutosave(float dt);

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

  // Optional deterministic replay capture (debug/regression).
  ReplayCapture m_replayCapture;
  std::uint64_t m_replayStrokeBaseHash = 0;

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

  // Blueprint copy/paste stamp tool (toggle with J).
  enum class BlueprintMode : std::uint8_t { Off = 0, Capture = 1, Stamp = 2 };
  BlueprintMode m_blueprintMode = BlueprintMode::Off;
  bool m_blueprintSelecting = false;
  std::optional<Point> m_blueprintSelStart;
  Point m_blueprintSelEnd{0, 0};

  // Stamp-mode tiling selection (Shift+drag). When `m_blueprintTileUseLibrary` is true,
  // Ctrl+Shift+drag triggers procedural tiling using the blueprint library tileset.
  bool m_blueprintTilingSelecting = false;
  bool m_blueprintTileUseLibrary = false;
  std::optional<Point> m_blueprintTileSelStart;
  Point m_blueprintTileSelEnd{0, 0};

  bool m_hasBlueprint = false;
  Blueprint m_blueprint;
  BlueprintTransform m_blueprintTransform{};
  bool m_blueprintTransformedDirty = false;
  Blueprint m_blueprintTransformed;
  BlueprintCaptureOptions m_blueprintCaptureOpt{};
  BlueprintApplyOptions m_blueprintApplyOpt{};

  // Blueprint library: file-backed stamp collection.
  // This is intentionally lightweight (simple folder scan + preview) so you can
  // build your own tooling workflow around the binary .isobp format.
  bool m_blueprintLibraryOpen = false;

  struct BlueprintLibraryEntry {
    std::string path;
    std::string name;
    std::string timeText;
    std::int64_t sortKey = 0; // seconds since epoch (best-effort)

    int width = 0;
    int height = 0;
    int deltas = 0;
    std::uint32_t version = 0;

    bool ok = false;
    std::string err;
  };

  std::vector<BlueprintLibraryEntry> m_blueprintLibrary;
  int m_blueprintLibrarySelection = 0;

  // Scroll position (first visible row in the list panel).
  int m_blueprintLibraryFirst = 0;

  // Delete confirmation (press Del twice within a short window).
  bool m_blueprintLibraryDeleteArmed = false;
  float m_blueprintLibraryDeleteTimer = 0.0f;

  // Preview cache (loaded from disk when selection changes).
  int m_blueprintLibraryPreviewIndex = -1;
  bool m_blueprintLibraryPreviewOk = false;
  std::string m_blueprintLibraryPreviewError;
  Blueprint m_blueprintLibraryPreview;


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

  // Cached routing helpers built alongside the road graph (used for road A* routing).
  RoadGraphIndex m_roadGraphIndex;
  RoadGraphWeights m_roadGraphWeights;

  // Cached lookups for road-graph overlays (rebuilt with the road graph).
  // Indexing uses idx = y*w + x.
  std::vector<int> m_roadGraphTileToNode;
  std::vector<int> m_roadGraphTileToEdge;

  // Debug overlay: road network resilience (bridge edges + articulation nodes) and optional bypass suggestions.
  bool m_showResilienceOverlay = false;
  bool m_resilienceDirty = true;
  RoadGraphResilienceResult m_roadResilience;


  bool m_resilienceBypassesDirty = true;
  std::vector<RoadResilienceBypassSuggestion> m_resilienceBypasses;

  // Default bypass planner settings (tweak via dev console `res ...`).
  int m_resilienceBypassTop = 5;
  bool m_resilienceBypassMoney = true;
  int m_resilienceBypassTargetLevel = 1;
  bool m_resilienceBypassAllowBridges = false;
  int m_resilienceBypassMaxCost = 0; // 0 => no limit
  int m_resilienceBypassMaxNodesPerSide = 256;

  // Debug overlay: traffic heatmap (derived commute model).
  bool m_showTrafficOverlay = false;
  bool m_trafficDirty = true;
  TrafficResult m_traffic;

  // Debug overlay: goods logistics heatmap (derived supply chain model).
  bool m_showGoodsOverlay = false;
  bool m_goodsDirty = true;
  GoodsResult m_goods;

  // Transit planner overlay (auto bus line suggestions).
  // Toggle panel with Ctrl+T. Export artifacts with Ctrl+Shift+T.
  // Planner + simulation tuning lives in Simulator::transitModel().

  struct TransitLineViz {
    int lineIndex = -1;
    int id = 0;
    std::uint64_t sumDemand = 0;
    std::uint64_t baseCost = 0;

    // Inclusive road-tile polyline for drawing.
    std::vector<Point> tiles;

    // Sampled stop tiles (endpoints always included).
    std::vector<Point> stops;
  };

  bool m_showTransitPanel = false;
  bool m_showTransitOverlay = false;
  TransitPlan m_transitPlan{};
  std::vector<std::uint64_t> m_transitEdgeDemand;

  bool m_transitPlanDirty = true;
  bool m_transitVizDirty = true;

  int m_transitSelection = 0;
  bool m_transitShowStops = true;
  bool m_transitShowOnlySelectedLine = false;
  int m_transitSelectedLine = -1; // -1 = all

  std::vector<TransitLineViz> m_transitViz;

  // Road upgrade planner overlay (street->avenue/highway suggestions).
  // Toggle panel with Ctrl+U. Export artifacts with Ctrl+Shift+U.
  // Uses the same demand-mode enum as the transit planner (commute/goods/combined).
  bool m_showRoadUpgradePanel = false;
  bool m_showRoadUpgradeOverlay = false;
  TransitDemandMode m_roadUpgradeDemandMode = TransitDemandMode::Combined;

  // Planner configuration. Note: budget is overridden when m_roadUpgradeBudgetAuto==true.
  RoadUpgradePlannerConfig m_roadUpgradeCfg{};
  bool m_roadUpgradeBudgetAuto = true;
  int m_roadUpgradeBudget = -1; // used when budgetAuto==false. -1 = unlimited.

  // Flow composition when demandMode includes goods.
  double m_roadUpgradeGoodsWeight = 1.0;

  RoadUpgradePlan m_roadUpgradePlan{};
  bool m_roadUpgradePlanDirty = true;

  int m_roadUpgradeSelection = 0;
  bool m_roadUpgradeShowOnlySelectedEdge = false;
  int m_roadUpgradeSelectedEdge = -1; // index into m_roadUpgradePlan.edges; -1 = all

  bool m_roadUpgradeSelectedMaskDirty = true;
  std::vector<std::uint8_t> m_roadUpgradeSelectedMask;

    // Flood/evacuation analysis panel (toggle with F).
  bool m_showResiliencePanel = false;
  int m_resilienceSelection = 0;

// Heatmap overlay: land value / amenities / pollution / traffic spill.
  enum class HeatmapOverlay : std::uint8_t {
    Off = 0,
    LandValue,
    ParkAmenity,
    WaterAmenity,
    Pollution,
    TrafficSpill,

    // Sea-level coastal flooding depth (derived from the heightfield).
    FloodDepth,

    // Depression-fill depth (Priority-Flood): "ponding potential" in closed basins.
    PondingDepth,

    // Evacuation scenario analysis (time/unreachable/flow).
    EvacuationTime,
    EvacuationUnreachable,
    EvacuationFlow,
  };

  HeatmapOverlay m_heatmapOverlay = HeatmapOverlay::Off;
  bool m_landValueDirty = true;
  LandValueResult m_landValue;

  // Sea-level flooding (heatmap): computed on-demand when the flood heatmap is active.
  bool m_seaFloodDirty = true;
  float m_seaLevel = 0.35f; // 0..1 height threshold
  SeaFloodConfig m_seaFloodCfg{};
  SeaFloodResult m_seaFlood;
  std::vector<float> m_seaFloodHeatmap; // normalized depth: 0 (dry) .. 1 (deepest)

  // Depression-fill / ponding potential (heatmap): computed on-demand when the ponding heatmap is active.
  bool m_pondingDirty = true;
  DepressionFillConfig m_pondingCfg{};
  int m_pondingFilledCells = 0;
  double m_pondingVolume = 0.0;
  float m_pondingMaxDepth = 0.0f;
  std::vector<float> m_pondingHeatmap; // normalized depth: 0 (none) .. 1 (deepest)

  // Evacuation scenario analysis (hazards + evacuation-to-edge): computed on-demand when any
  // evacuation heatmap is active or when explicitly requested via the console.
  bool m_evacDirty = true;
  EvacuationScenarioConfig m_evacCfg{};
  EvacuationScenarioResult m_evacScenario;
  EvacuationScenarioHeatmaps m_evacHeatmaps;



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

    // Stable per-vehicle style id (used to pick deterministic sprite variants).
    int style = 0;

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
  // When switching between auto/manual UI scaling, preserve the last manual
  // value so "toggle" does what users expect.
  float m_uiScaleManual = 1.0f;

  // World render resolution scaling (terrain + world overlays only). When enabled,
  // the world is rendered to an offscreen target and then scaled to the window.
  bool m_worldRenderScaleAuto = false;
  float m_worldRenderScale = 1.0f;
  float m_worldRenderScaleMin = 0.70f;
  float m_worldRenderScaleMax = 1.00f;
  int m_worldRenderTargetFps = 60;
  bool m_worldRenderFilterPoint = false;
  bool m_mergedZoneBuildings = true;

  // Smoothed CPU frame time (for the video/settings panel + auto world scaling).
  float m_frameTimeSmoothed = 1.0f / 60.0f;

  // Accumulator used to throttle auto world render-scale adjustments.
  float m_worldRenderAutoTimer = 0.0f;

  RenderTexture2D m_worldRenderRT{};
  bool m_worldRenderRTValid = false;
  int m_worldRenderRTWidth = 0;
  int m_worldRenderRTHeight = 0;

  bool m_showVideoSettings = false;
  int m_videoPage = 0; // 0=Display, 1=Visual FX, 2=Atmosphere, 3=UI Theme
  int m_videoSelection = 0;
  int m_videoSelectionDisplay = 0;
  int m_videoSelectionVisual = 0;
  int m_videoSelectionAtmos = 0;
  int m_videoSelectionUiTheme = 0;

  // Visual prefs persistence (autosave throttled).
  std::string m_visualPrefsPath = "isocity_visual.json";
  bool m_visualPrefsAutosave = true;
  bool m_visualPrefsDirty = false;
  float m_visualPrefsSaveTimer = 0.0f;
  VisualPrefs m_visualPrefsLastSnapshot{};

  // Window mode tracking for borderless fullscreen restore.
  bool m_borderlessWindowed = false;
  int m_windowedX = 0;
  int m_windowedY = 0;
  int m_windowedW = 1280;
  int m_windowedH = 720;

  DevConsole m_console;

  // Persistent ScriptRunner variables used by the in-game dev console `script` command.
  //
  // This lets you run a sequence of script files/commands that share a small amount
  // of state (e.g. output directories, counters, etc.) without having to re-define
  // variables every time.
  std::map<std::string, std::string> m_consoleScriptVars;
  int m_consoleScriptRunIndex = 0;

  // Screenshot capture (queued from input and executed in draw() so the capture includes the rendered frame)
  bool m_pendingScreenshot = false;
  std::string m_pendingScreenshotPath;

  // World export capture (queued from input and executed in draw() after the
  // frame so the export uses up-to-date textures/state).
  bool m_pendingMapExport = false;
  std::string m_pendingMapExportPath;
  int m_pendingMapExportMaxSize = 4096;

  // Layered world export capture (terrain/decals/structures/overlays as separate PNGs)
  bool m_pendingMapLayersExport = false;
  std::string m_pendingMapLayersPrefix;
  int m_pendingMapLayersMaxSize = 4096;

  // Software 3D render export (headless CPU renderer in Export3D/Soft3D).
  bool m_pendingRender3D = false;
  std::string m_pendingRender3DPath;
  Render3DConfig m_pendingRender3DCfg{};
  ExportLayer m_pendingRender3DLayer = ExportLayer::Overlay;
  bool m_pendingRender3DApplyGrade = true; // apply current day/night + weather grade

  // Optional in-game 3D preview (small, throttled, updated when the world changes).
  bool m_show3DPreview = false;
  bool m_3dPreviewDirty = true;
  float m_3dPreviewTimer = 0.0f;
  Render3DConfig m_3dPreviewCfg{};
  ExportLayer m_3dPreviewLayer = ExportLayer::Overlay;
  bool m_3dPreviewApplyGrade = true;
  Texture2D m_3dPreviewTex{};
  int m_3dPreviewTexW = 0;
  int m_3dPreviewTexH = 0;

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
  int m_roadDragBridgeTiles = 0; // subset of new/upgrade tiles that are bridges (roads on water)
  int m_roadDragUpgradeTiles = 0;
  int m_roadDragMoneyCost = 0;
  bool m_roadDragValid = false;
};

} // namespace isocity
