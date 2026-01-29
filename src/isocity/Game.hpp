#pragma once

#include "isocity/Config.hpp"
#include "isocity/Console.hpp"
#include "isocity/EditHistory.hpp"
#include "isocity/ReplayCapture.hpp"
#include "isocity/Export.hpp"
#include "isocity/Dossier.hpp"
#include "isocity/SeedMiner.hpp"
#include "isocity/ServiceOptimizer.hpp"
#include "isocity/PolicyOptimizer.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/EvacuationScenario.hpp"
#include "isocity/FlowField.hpp"
#include "isocity/Blueprint.hpp"
#include "isocity/AutoBuild.hpp"
#include "isocity/Iso.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/VisualPrefs.hpp"
#include "isocity/PostFx.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphRouting.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/RoadResilienceBypass.hpp"
#include "isocity/RoadUpgradePlanner.hpp"
#include "isocity/StreetNames.hpp"
#include "isocity/Wayfinding.hpp"
#include "isocity/PovController.hpp"
#include "isocity/PovProcedural.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/FireRisk.hpp"
#include "isocity/AirPollution.hpp"
#include "isocity/SolarPotential.hpp"
#include "isocity/SkyView.hpp"
#include "isocity/EnergyModel.hpp"
#include "isocity/CarbonModel.hpp"
#include "isocity/CrimeModel.hpp"
#include "isocity/TrafficSafety.hpp"
#include "isocity/TransitAccessibility.hpp"
#include "isocity/RoadHealth.hpp"
#include "isocity/Walkability.hpp"
#include "isocity/JobOpportunity.hpp"
#include "isocity/Livability.hpp"
#include "isocity/HotspotAnalysis.hpp"
#include "isocity/RunoffPollution.hpp"
#include "isocity/RunoffMitigation.hpp"
#include "isocity/TransitPlanner.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "isocity/RaylibShim.hpp"

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
  float demandCommercial = 0.0f;   // 0..1
  float demandIndustrial = 0.0f;   // 0..1

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

// SimCity-style city news / advisor feed entry.
// Stored in the game layer (not saved yet), derived from World::Stats after each sim day.
enum class CityNewsTone : std::uint8_t { Good = 0, Neutral = 1, Bad = 2, Alert = 3 };

struct CityNewsEntry {
  int day = 0;
  CityNewsTone tone = CityNewsTone::Neutral;

  // Mayor rating (0..100), exponentially smoothed so it doesn't jitter day-to-day.
  float mayorRating = 50.0f;

  std::string headline;
  std::string body;
};

// Optional goal/challenge that nudges sandbox play toward short-term objectives.
//
// Challenges are intentionally game-layer only (not saved yet), similar to City News.
enum class CityChallengeKind : std::uint8_t {
  GrowPopulation = 0,
  BuildParks,
  ReduceCongestion,
  ImproveGoods,
  ImproveServices,
  BalanceBudget,
  RestoreOutsideConnection,
};

enum class CityChallengeStatus : std::uint8_t { Active = 0, Completed = 1, Failed = 2, Canceled = 3 };

struct CityChallenge {
  std::uint32_t id = 0;
  CityChallengeKind kind = CityChallengeKind::GrowPopulation;
  CityChallengeStatus status = CityChallengeStatus::Active;

  int dayIssued = 0;
  int dayDeadline = 0; // inclusive

  int rewardMoney = 0;

  // Generic parameters used by each kind.
  //
  // - GrowPopulation: startInt=popStart, targetInt=popTarget
  // - BuildParks: startInt=parksBaseline, targetInt=parksTarget
  // - ReduceCongestion: startF=congestionStart, targetF=congestionTarget
  // - ImproveGoods: startF=goodsStart, targetF=goodsTarget
  // - ImproveServices: startF=servicesStart, targetF=servicesTarget
  // - BalanceBudget: targetInt=requiredStreak, stateInt=streak
  // - RestoreOutsideConnection: startInt=unreachableStart, targetInt=0
  int startInt = 0;
  int targetInt = 0;
  int stateInt = 0;
  float startF = 0.0f;
  float targetF = 0.0f;

  std::string title;
  std::string description;
};

struct CityChallengeLogEntry {
  int day = 0;
  CityChallengeStatus status = CityChallengeStatus::Completed;
  int rewardMoney = 0;
  std::string title;
};



struct GameStartupOptions {
  // Visual preferences JSON file to load/save (relative or absolute).
  //
  // If empty, the game will keep the built-in default (isocity_visual.json).
  std::string visualPrefsPath = "isocity_visual.json";

  // When false, existing prefs are ignored and the game starts with conservative
  // defaults (useful for 'safe mode' bootstraps).
  bool loadVisualPrefs = true;
};

class Game {
public:
  explicit Game(Config cfg, GameStartupOptions startup = {});
  ~Game();

  void run();

  // Load a save file into the running game instance.
  //
  // Intended for bootstraps/launchers (e.g. --load/--resume).
  bool loadFromFile(const std::string& path, const char* toastLabel = nullptr);

private:
  void resetWorld(std::uint64_t newSeed);
  void invalidateHydrology();
  void invalidateAnalysisLayers();
  void applyToolBrush(int centerX, int centerY);
  void floodFillDistrict(Point start, bool includeRoads);
  void floodFillTool(Point start, bool includeRoads);
  void showToast(const std::string& msg, float seconds = 2.5f);

  // Effective simulation pause state (user pause OR focus/minimize auto pause).
  bool simPaused() const { return m_simPausedUser || m_simPausedAuto; }

  // City report (time-series graphs of key stats).
  void clearHistory();
  void recordHistorySample(const Stats& s);
  void drawReportPanel(int screenW, int screenH);

  // City news / advisor feed (SimCity-style newspaper headlines).
  void clearCityNews();
  void recordCityNews(const Stats& s);
  void drawNewsPanel(const Rectangle& rect);
  void exportCityNews();

  // City challenges / goals (optional guidance for sandbox play).
  void clearCityChallenges();
  // Applies one day's worth of challenge progression.
  // Returns money awarded this day. (ioStats.money is updated.)
  int applyCityChallenges(Stats& ioStats);
  void drawChallengesPanel(const Rectangle& rect);
  void focusChallenge(const CityChallenge& c);
  bool rerollChallenge(std::size_t idx);


  // City treasury / municipal bonds (toggle with Ctrl+B).
  void drawBondsPanel(const Rectangle& rect);
  void issueBond(int principal, int termDays, int aprBasisPoints);
  void payDownBond(std::size_t idx, int payment);


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

  // City dossier export (headless) - runs asynchronously so large exports
  // do not stall the render thread.
  void updateDossierExportJob();

  // Mesh export (OBJ/glTF) - runs asynchronously so exports do not stall the render thread.
  void updateMeshExportJob();

  // Policy advisor / auto-tuner (async policy optimization).
  void updatePolicyOptimizationJob();
  void startPolicyOptimization();
  void applyPolicyCandidate(const PolicyCandidate& p, const char* toastLabel = nullptr);

  // Seed mining integration (headless).
  void updateSeedMining(float dt);

  // Headless Lab panel (seed mining + dossier export). Toggle with Ctrl+F2.
  void drawHeadlessLabPanel(int x0, int y0);

  // Queue dossier exports (processed asynchronously).
  void queueDossierCurrentWorld(CityDossierConfig cfg, const char* toastLabel = nullptr);
  void queueDossierSeeds(CityDossierConfig cfg, std::vector<std::uint64_t> seeds, int bakeDays,
                         const char* toastLabel = nullptr);

  // AutoBuild bot (procedural city growth automation).
  void runAutoBuild(int days, const AutoBuildConfig& cfg, const char* toastPrefix = nullptr);
  void drawAutoBuildPanel(int x0, int y0);

  // In-game navigation / wayfinding.
  void ensureWayfindingUpToDate();
  void clearWayfindingRoute();
  void rebuildWayfindingFocusPath();
  void drawWayfindingPanel(int x0, int y0);
  void drawWayfindingOverlay();

  // Procedural POV (ride-along) camera.
  void stopPov();
  void updatePov(float dt);
  void updatePov3DPreview(float dt);
  bool startRoamPov(std::uint32_t seed, int lengthTiles);

  void handleInput(float dt);
  void update(float dt);
  void draw();
  void drawPerfOverlay(int uiW, int uiH, float uiTime);

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
  void ensureTradeMarketUpToDate();

  // Public services satisfaction (heatmap overlays).
  void ensureServicesHeatmapsUpToDate();

  // Fire risk/coverage (heatmap overlay + city news).
  void ensureFireRiskUpToDate();

  // Public services planner (facility placement suggestions) + overlay.
  void ensureServicesPlanUpToDate();
  void drawServicesOverlay();
  void drawServicesPanel(int x0, int y0);
  void adjustServicesPanel(int dir, bool bigStep);
  bool applyServicePlacement(std::size_t idx);
  void exportServicesArtifacts();

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

  // Policy advisor: async optimizer that suggests taxes/maintenance for a chosen objective.
  struct PolicyOptJob;
  std::unique_ptr<PolicyOptJob> m_policyOptJob;
  PolicyOptProgress m_policyOptProgress;

  // Advisor UI settings.
  int m_policyOptPreset = 0;        // objective preset selector
  int m_policyOptTaxRadius = 3;     // +/- range around current taxes
  int m_policyOptMaintRadius = 1;   // +/- range around current maintenance

  PolicyOptimizerConfig m_policyOptCfg{};

  // Last completed result (cached for display / application).
  bool m_policyOptHaveResult = false;
  bool m_policyOptResultOk = false;
  int m_policyOptResultDay = -1;
  PolicyOptimizationResult m_policyOptResult{};
  std::string m_policyOptResultErr;

  int m_policyOptTopSelection = 0;
  int m_policyOptTopFirst = 0;

  bool m_showTrafficModel = false;
  int m_trafficModelSelection = 0;

  // Public services panel + optimizer (experimental).
  //
  // This is in-progress scaffolding that exposes the headless services model
  // and ServiceOptimizer inside the interactive app.
  bool m_showServicesPanel = false;
  int m_servicesSelection = 0;
  bool m_showServicesOverlay = false;

  // Cached services satisfaction fields (heatmaps), computed on-demand.
  bool m_servicesHeatmapsDirty = true;
  float m_servicesHeatmapsTimer = 0.0f;
  ServicesResult m_servicesHeatmaps;

  // Cached fire risk/coverage field (computed on-demand; used for heatmap + news).
  bool m_fireRiskDirty = true;
  float m_fireRiskTimer = 0.0f;
  FireRiskConfig m_fireRiskCfg{};
  FireRiskResult m_fireRisk;

  // Cached optimizer plan (where to place the next facilities).
  bool m_servicesPlanDirty = true;
  bool m_servicesPlanValid = false;
  ServiceOptimizerConfig m_servicesPlanCfg{};
  ServiceOptimizerResult m_servicesPlan{};
  int m_servicesPlanSelection = 0;
  int m_servicesPlanFirst = 0; // scroll offset (rows) for placement list
  Tool m_servicesPlanTool = Tool::School;

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
  // Full Stats history for headless exports (dossier / analysis tooling).
  // This mirrors m_cityHistory but stores the complete Stats struct for each sampled day.
  std::vector<Stats> m_tickStatsHistory;
  int m_cityHistoryMax = 240; // max stored days

  // City news (SimCity-style newspaper/advisor feed). Not saved yet; derived from daily Stats.
  bool m_showNewsPanel = false;
  std::deque<CityNewsEntry> m_cityNews;
  int m_cityNewsMax = 120;
  int m_newsSelection = 0;
  int m_newsFirst = 0;
  float m_mayorRatingEma = 50.0f;
  float m_mayorRatingPrev = 50.0f;

  // City News can reference expensive analysis layers (air/runoff/livability/etc).
  // To avoid stalling the sim when running at high speed, we recompute these
  // advisor snapshots on a small wall-clock cooldown and reuse cached values
  // across multiple in-game days.
  double m_newsEnvLastComputeSec = -1.0;
  int m_newsEnvLastComputeDay = -1;

  // City challenges panel (toggle with Ctrl+O).
  bool m_showChallengesPanel = false;
  int m_challengeSelection = 0;
  int m_challengeFirst = 0;
  int m_challengeTargetActive = 3;
  int m_challengeRerolls = 0;
  std::uint32_t m_challengeNextId = 1;
  int m_challengeLastProcessedDay = -1;

  std::vector<CityChallenge> m_cityChallenges;
  std::deque<CityChallengeLogEntry> m_challengeLog;
  int m_challengeLogMax = 40;

// City treasury / municipal bonds panel (toggle with Ctrl+B).
bool m_showBondsPanel = false;
int m_bondsType = 1;       // 0=short, 1=medium, 2=long
int m_bondsAmount = 250;   // desired principal for issuance UI
int m_bondsSelection = 0;  // selected outstanding bond in the list
int m_bondsFirst = 0;      // scroll offset for the bond list

  // Daily addendum injected into City News (used to publish challenge completions/failures).
  std::map<int, std::string> m_cityNewsAddendum;

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

  // Procedural trade market cache (used by the policy panel).
  bool m_tradeDirty = true;
  int m_tradeCachedDay = -1;
  TradeModelSettings m_tradeCachedSettings{};
  TradeMarketResult m_tradeMarket{};

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
  bool m_roadUpgradePlanApplied = false;

  int m_roadUpgradeSelection = 0;
  bool m_roadUpgradeShowOnlySelectedEdge = false;
  int m_roadUpgradeSelectedEdge = -1; // index into m_roadUpgradePlan.edges; -1 = all

  bool m_roadUpgradeSelectedMaskDirty = true;
  std::vector<std::uint8_t> m_roadUpgradeSelectedMask;

  // ---------------------------------------------------------------------------------------------
  // In-game navigation / wayfinding.
  //
  // This bridges the headless Wayfinding + StreetNames modules into the interactive app so you can:
  //   - geocode street addresses / intersections / tile coords
  //   - compute a turn-by-turn route
  //   - visualize the route in-world
  //
  // Routes are typically created via the dev console `route` command.
  bool m_showWayfindingPanel = false;
  bool m_showWayfindingOverlay = false;

  // Derived caches. Rebuilt lazily when marked dirty.
  bool m_wayfindingDirty = true;
  StreetNamingResult m_wayfindingStreets;
  std::vector<ParcelAddress> m_wayfindingAddresses;
  AddressIndex m_wayfindingIndex;

  // Most recent query + route result.
  std::string m_wayfindingFromQuery;
  std::string m_wayfindingToQuery;
  RouteResult m_wayfindingRoute;

  // UI state.
  int m_wayfindingManeuverFirst = 0;
  int m_wayfindingFocusManeuver = -1; // -1 = show full route
  std::vector<Point> m_wayfindingFocusPath;

  // ---------------------------------------------------------------------------------------------
  // AutoBuild bot (procedural city growth automation).
  //
  // Exposes the headless AutoBuild module inside the interactive app so you can
  // grow a city automatically from the current world state.
  bool m_showAutoBuildPanel = false;
  AutoBuildConfig m_autoBuildCfg{};
  AutoBuildReport m_autoBuildLastReport{};
  int m_autoBuildRunDays = 30;
  int m_autoBuildPanelScroll = 0;

  // Procedural POV camera.
  //
  // A cinematic "ride-along" camera rig that can follow the current wayfinding route
  // or a procedurally generated TourPlanner tour.
  PovController m_pov;
  PovRoamConfig m_povRoamCfg{};
  std::uint32_t m_povRoamSeed = 1u;
  std::string m_povRoamDebug{};

  // Optional: drive the software 3D preview camera from the POV rig.
  bool m_povDrive3DPreview = false;
  bool m_povSaved3DPreviewValid = false;
  bool m_povSavedShow3DPreview = false;
  Render3DConfig m_povSaved3DPreviewCfg{};
  float m_pov3DPreviewAccumSec = 0.0f;

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

    // Public services satisfaction (derived from the services model).
    ServicesOverall,
    ServicesEducation,
    ServicesHealth,
    ServicesSafety,

  // Heuristic fire hazard (dense development + weak fire station response).
  FireRisk,

    // Sea-level coastal flooding depth (derived from the heightfield).
    FloodDepth,

    // Depression-fill depth (Priority-Flood): "ponding potential" in closed basins.
    PondingDepth,

    // Evacuation scenario analysis (time/unreachable/flow).
    EvacuationTime,
    EvacuationUnreachable,
    EvacuationFlow,

    // Advanced analysis layers (export/dossier integrated).
    AirPollution,
    AirPollutionEmission,
    SkyView,
    CanyonConfinement,
    SolarExposure,
    SolarPotential,
    EnergyDemand,
    EnergySolar,
    EnergyBalance,

    CarbonEmission,
    CarbonSequestration,
    CarbonBalance,

    CrimeRisk,
    PoliceAccess,

    TrafficCrashRisk,
    TrafficCrashExposure,
    TrafficCrashPriority,

    TransitAccess,
    TransitModeSharePotential,

    Walkability,
    WalkabilityPark,
    WalkabilityRetail,
    WalkabilityEducation,
    WalkabilityHealth,
    WalkabilitySafety,

    JobAccess,
    JobOpportunity,

    RoadCentrality,
    RoadVulnerability,
    RoadBypass,

    Livability,
    InterventionPriority,

    LivabilityHotspot,
    InterventionHotspot,

    RunoffPollution,
    RunoffPollutionLoad,
    RunoffMitigationPriority,
    RunoffMitigationPlan,
  };

  HeatmapOverlay m_heatmapOverlay = HeatmapOverlay::Off;
  bool m_landValueDirty = true;
  LandValueResult m_landValue;

  // ---------------------------------------------------------------------------
  // Advanced analysis layers (computed on-demand for heatmaps).
  // These mirror the export/dossier layers so the interactive app can preview them.
  // ---------------------------------------------------------------------------
  bool m_airPollutionDirty = true;
  AirPollutionConfig m_airPollutionCfg{};
  AirPollutionResult m_airPollution;

  bool m_skyViewDirty = true;
  SkyViewConfig m_skyViewCfg{};
  SkyViewResult m_skyView;

  bool m_solarDirty = true;
  SolarPotentialConfig m_solarCfg{};
  SolarPotentialResult m_solar;

  bool m_energyDirty = true;
  EnergyModelConfig m_energyCfg{};
  EnergyModelResult m_energy;

  bool m_carbonDirty = true;
  CarbonModelConfig m_carbonCfg{};
  CarbonModelResult m_carbon;

  bool m_crimeDirty = true;
  CrimeModelConfig m_crimeCfg{};
  CrimeModelResult m_crime;

  bool m_trafficSafetyDirty = true;
  TrafficSafetyConfig m_trafficSafetyCfg{};
  TrafficSafetyResult m_trafficSafety;

  bool m_transitAccessDirty = true;
  TransitAccessibilityConfig m_transitAccessCfg{};
  TransitAccessibilityResult m_transitAccess;

  bool m_walkabilityDirty = true;
  WalkabilityConfig m_walkabilityCfg{};
  WalkabilityResult m_walkability;

  bool m_jobsDirty = true;
  JobOpportunityConfig m_jobsCfg{};
  JobOpportunityResult m_jobs;

  bool m_roadHealthDirty = true;
  RoadHealthConfig m_roadHealthCfg{};
  RoadHealthResult m_roadHealth;

  bool m_livabilityDirty = true;
  LivabilityConfig m_livabilityCfg{};
  LivabilityResult m_livability;

  bool m_hotspotsDirty = true;
  HotspotConfig m_hotspotCfg{};
  HotspotResult m_livabilityHotspot;
  HotspotResult m_interventionHotspot;

  bool m_runoffDirty = true;
  RunoffPollutionConfig m_runoffCfg{};
  RunoffPollutionResult m_runoff;

  bool m_runoffMitigationDirty = true;
  RunoffMitigationConfig m_runoffMitigationCfg{};
  RunoffMitigationResult m_runoffMitigation;

  // Scratch buffer for displaying byte masks (bypass/plan) as a heatmap.
  std::vector<float> m_heatmapTmp01;
  float m_analysisHeatmapsTimer = 0.0f;

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
  //
  // We track user pause separately from focus/minimize auto-pause so alt-tabbing doesn't
  // accidentally change the user's intended pause state.
  bool m_simPausedUser = false;
  bool m_simPausedAuto = false;
  int m_simSpeedIndex = 2; // 0.25x, 0.5x, 1x, 2x, ... (default = 1x)


  // Game loop pacing / hitch protection.
  //
  // The simulator already runs at a fixed tick interval internally, but a long frame can
  // still cause multiple ticks to be processed back-to-back. To keep the interactive app
  // responsive, we optionally cap catch-up work per frame and clamp extreme dt spikes.
  struct LoopConfig {
    // Clamp the per-frame dt used for input/camera/UI timers (seconds).
    // 0 disables clamping.
    float maxFrameDt = 0.25f;

    // Clamp the dt fed into the simulation clock (seconds, before sim speed multiplier).
    // 0 disables clamping.
    float maxSimDt = 0.50f;

    // Pause the real-time clock when the window isn't interactive.
    bool pauseWhenUnfocused = true;
    bool pauseWhenMinimized = true;

    // When the window is not interactive, optionally reduce the target FPS to save CPU/GPU.
    bool throttleFpsWhenUnfocused = true;
    int targetFpsFocused = 60;
    int targetFpsUnfocused = 15;

    // Simulation catch-up limiter.
    bool simTickLimiter = true;

    // If enabled, compute max ticks per frame from a CPU budget estimate.
    bool simTickAuto = true;
    float simBudgetMs = 6.0f;
    int simAutoMinTicks = 1;
    int simAutoMaxTicks = 32;

    // Fixed tick limit used when simTickAuto is false.
    int simMaxTicksPerFrame = 8;

    // Clamp how far the sim accumulator is allowed to fall behind (in ticks).
    // Prevents huge dt spikes (breakpoints, alt-tab) from queuing unbounded work.
    int simMaxBacklogTicks = 240; // 240 * 0.5s = 120s by default
  };

  struct LoopStats {
    float dtRaw = 0.0f;
    float dtFrame = 0.0f;
    float dtSim = 0.0f;
    bool dtClamped = false;

    bool focused = true;
    bool minimized = false;
    bool focusPaused = false;

    int targetFps = 60;

    int simTicks = 0;
    int simTickLimit = 0;
    int simBacklogTicks = 0;
    float simBacklogSec = 0.0f;

    double cpuInputMs = 0.0;
    double cpuUpdateMs = 0.0;
    double cpuDrawMs = 0.0;
    double cpuFrameMs = 0.0;

    double simCpuMs = 0.0;
    double simMsPerTick = 0.0;
    double simMsPerTickEma = 0.0;
  };

  LoopConfig m_loopCfg{};
  LoopStats m_loopStats{};
  bool m_showPerfOverlay = false;
  bool m_focusPausePrev = false;
  int m_targetFpsApplied = 60;

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

  // Stylized post-processing when compositing the world render target.
  PostFxSettings m_postFx{};
  PostFxPipeline m_postFxPipeline{};

  // Temporal AA state (used when PostFX TAA is enabled).
  bool m_taaActiveLastFrame = false;
  bool m_taaPrevCamValid = false;
  Camera2D m_taaPrevCam{};           // unjittered camera used for world RT render
  float m_taaPrevWorldRenderScale = 1.0f;
  int m_taaPrevRTW = 0;
  int m_taaPrevRTH = 0;
  int m_taaFrameIndex = 0;
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

  // Headless Lab panel (Ctrl+F2): integrates headless tools into the base game layer.
  bool m_showHeadlessLab = false;
  int m_headlessLabTab = 0; // 0=Mining, 1=Dossier, 2=Mesh

  // Seed mining (headless) integrated into the base game.
  // This runs incrementally (optionally auto-stepped each frame) so you can
  // search for interesting seeds without leaving the interactive app.
  std::unique_ptr<MineSession> m_mineSession;
  bool m_mineAutoRun = false;
  int m_mineAutoStepsPerFrame = 1;
  float m_mineAutoBudgetMs = 8.0f;
  double m_mineLastAutoStepMs = 0.0;
  MineObjective m_mineObjective = MineObjective::Balanced;
  int m_mineTopK = 20;
  bool m_mineTopDiverse = true;
  bool m_mineTopPareto = false;
  bool m_mineTopParetoCrowding = true;

  // City Lab UI state for mining.
  MineConfig m_labMineCfg{};
  int m_labMineTopScroll = 0;
  int m_labMineTopSelection = 0;
  int m_labMineBakeDays = 120;
  int m_labMineBatchExportN = 5;

  // Cached top-K list (avoid re-sorting every frame).
  int m_labMineTopCacheIndex = -1;
  int m_labMineTopCacheTopK = 0;
  bool m_labMineTopCacheDiverse = true;
  bool m_labMineTopCachePareto = false;
  bool m_labMineTopCacheParetoCrowding = true;
  std::vector<int> m_labMineTopIndices;

  // Manual stepping requests (used by dev console and future UI) so expensive
  // mining runs can be budgeted per frame instead of blocking.
  int m_mineManualStepRequests = 0;

  // Deferred mine load: resetWorld is applied at the start of the next frame
  // to avoid mutating world state from inside console callbacks/UI handlers.
  bool m_pendingMineLoadSeed = false;
  std::uint64_t m_pendingMineLoadSeedValue = 0;

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

  // City dossier export (headless): images + CSV/JSON + HTML using the same exporter as
  // proc_isocity_dossier. Requests are queued and processed asynchronously (so exports
  // don't stall the render thread).
  struct DossierRequest {
    enum class Kind : std::uint8_t { CurrentWorld = 0, SeedBake = 1 };
    Kind kind = Kind::CurrentWorld;

    CityDossierConfig cfg;

    // Snapshot/config for export.
    ProcGenConfig procCfg{};
    SimConfig simCfg{};
    std::vector<Stats> ticks;
    World world;

    // Optional: bake one or more seeds (generate+simulate) without touching the
    // interactive world, then export dossiers for each.
    std::vector<std::uint64_t> seeds;
    int bakeDays = 0;
    int w = 0;
    int h = 0;
  };

  std::deque<DossierRequest> m_dossierQueue;

  // City Lab dossier UI settings (presets + quick options).
  int m_labDossierPreset = 0;      // 0=Full, 1=Fast
  int m_labDossierExportScale = 2; // 1..6
  bool m_labDossierIso = true;
  bool m_labDossier3d = false;
  int m_labDossierFormat = 0;      // 0=png, 1=ppm

  // Last completed dossier export (for UI/console feedback).
  bool m_lastDossierOk = false;
  std::string m_lastDossierOutDir;
  std::string m_lastDossierMsg;

  // Mesh export UI settings (City Lab).
  int m_labMeshFormat = 0; // 0=OBJ+MTL, 1=glTF, 2=GLB
  MeshExportConfig m_labMeshCfg{};

  // Last completed mesh export (for UI/console feedback).
  bool m_lastMeshOk = false;
  std::string m_lastMeshPath;
  std::string m_lastMeshMsg;
  MeshExportStats m_lastMeshStats{};

  // Background mesh export job (async)
  struct MeshJob;
  std::unique_ptr<MeshJob> m_meshJob;

  // Background dossier export job (async)
  struct DossierJob;
  std::unique_ptr<DossierJob> m_dossierJob;

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
