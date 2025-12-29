#include "isocity/EditHistory.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Road.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static int g_failures = 0;

#define EXPECT_TRUE(cond)                                                                                            \
  do {                                                                                                               \
    if (!(cond)) {                                                                                                   \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_TRUE failed: " << #cond << "\n";                          \
    }                                                                                                                \
  } while (0)

#define EXPECT_EQ(a, b)                                                                                              \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    if (!(_a == _b)) {                                                                                               \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_EQ failed: " << #a << " == " << #b << "\n";              \
    }                                                                                                                \
  } while (0)

#define EXPECT_NE(a, b)                                                                                              \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    if ((_a == _b)) {                                                                                                \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_NE failed: " << #a << " != " << #b << "\n";              \
    }                                                                                                                \
  } while (0)
#define EXPECT_NEAR(a, b, eps)                                                                                        \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    const auto _e = (eps);                                                                                           \
    if (std::fabs((_a) - (_b)) > (_e)) {                                                                             \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_NEAR failed: " << #a << " ~= " << #b << " (eps=" << _e   \
                << ")\n";                                                                                            \
    }                                                                                                                \
  } while (0)



namespace {

bool FindEmptyAdjacentPair(const isocity::World& w, int& outX, int& outY)
{
  // Find (x,y) and (x+1,y) that are buildable and empty (Overlay::None).
  for (int y = 1; y < w.height() - 1; ++y) {
    for (int x = 1; x < w.width() - 2; ++x) {
      if (!w.isBuildable(x, y) || !w.isBuildable(x + 1, y)) continue;
      if (w.at(x, y).overlay != isocity::Overlay::None) continue;
      if (w.at(x + 1, y).overlay != isocity::Overlay::None) continue;
      outX = x;
      outY = y;
      return true;
    }
  }
  return false;
}

void TestRoadAutoTilingMasks()
{
  using namespace isocity;

  World w(8, 8, 123);

  w.setRoad(3, 3);
  EXPECT_EQ(w.at(3, 3).overlay, Overlay::Road);
  EXPECT_EQ(static_cast<int>(w.at(3, 3).variation & 0x0F), 0);

  // Add a road to the north (y-1) => bit0 set on (3,3), bit2 set on (3,2).
  w.setRoad(3, 2);
  EXPECT_EQ(static_cast<int>(w.at(3, 3).variation & 0x0F), 1); // north neighbor
  EXPECT_EQ(static_cast<int>(w.at(3, 2).variation & 0x0F), 4); // south neighbor

  // Add a road to the east (x+1) => bit1 set on (3,3), bit3 set on (4,3).
  w.setRoad(4, 3);
  EXPECT_EQ(static_cast<int>(w.at(3, 3).variation & 0x0F), 1 | 2);
  EXPECT_EQ(static_cast<int>(w.at(4, 3).variation & 0x0F), 8);

  // Bulldoze the north road; masks should update.
  w.bulldoze(3, 2);
  EXPECT_EQ(w.at(3, 2).overlay, Overlay::None);
  EXPECT_EQ(static_cast<int>(w.at(3, 3).variation & 0x0F), 2);
}

void TestEditHistoryUndoRedo()
{
  using namespace isocity;

  World w(6, 6, 999);
  w.stats().money = 100;

  EditHistory hist;

  hist.beginStroke(w);
  hist.noteTilePreEdit(w, 2, 2);

  const ToolApplyResult res = w.applyTool(Tool::Road, 2, 2);
  EXPECT_EQ(res, ToolApplyResult::Applied);

  hist.endStroke(w);

  EXPECT_EQ(w.at(2, 2).overlay, Overlay::Road);
  EXPECT_EQ(w.stats().money, 99);
  EXPECT_EQ(hist.undoSize(), static_cast<std::size_t>(1));
  EXPECT_EQ(hist.redoSize(), static_cast<std::size_t>(0));

  EXPECT_TRUE(hist.undo(w));
  EXPECT_EQ(w.at(2, 2).overlay, Overlay::None);
  EXPECT_EQ(w.stats().money, 100);

  EXPECT_TRUE(hist.redo(w));
  EXPECT_EQ(w.at(2, 2).overlay, Overlay::Road);
  EXPECT_EQ(w.stats().money, 99);
}

void TestEditHistoryUndoRedoFixesRoadMasksLocally()
{
  using namespace isocity;

  World w(6, 6, 123u);
  w.stats().money = 1000;

  // Build an initial horizontal road segment: (2,2)-(3,2).
  w.setRoad(2, 2);
  w.setRoad(3, 2);

  // Sanity-check initial masks.
  // (2,2) has an east neighbor => bit1.
  EXPECT_EQ(static_cast<int>(w.at(2, 2).variation & 0x0F), 2);
  // (3,2) has a west neighbor => bit3.
  EXPECT_EQ(static_cast<int>(w.at(3, 2).variation & 0x0F), 8);

  EditHistory hist;

  // Place a new road at (3,1) adjacent to the existing road tile at (3,2).
  // Intentionally record ONLY the edited tile (not its neighbors) to ensure
  // undo/redo can still keep road masks correct via local fixup.
  hist.beginStroke(w);
  hist.noteTilePreEdit(w, 3, 1);
  EXPECT_EQ(w.applyTool(Tool::Road, 3, 1), ToolApplyResult::Applied);
  hist.endStroke(w);

  // Now (3,2) should have west + north connections => bit3 + bit0 = 9.
  EXPECT_EQ(static_cast<int>(w.at(3, 2).variation & 0x0F), 9);

  // Undo should remove the new road and restore the neighbor's mask.
  EXPECT_TRUE(hist.undo(w));
  EXPECT_EQ(w.at(3, 1).overlay, Overlay::None);
  EXPECT_EQ(static_cast<int>(w.at(3, 2).variation & 0x0F), 8);

  // Redo should re-add it and re-apply the correct mask.
  EXPECT_TRUE(hist.redo(w));
  EXPECT_EQ(w.at(3, 1).overlay, Overlay::Road);
  EXPECT_EQ(static_cast<int>(w.at(3, 2).variation & 0x0F), 9);
}


void TestToolsDoNotOverwriteOccupiedTiles()
{
  using namespace isocity;

  World w(8, 8, 42);
  w.stats().money = 200;

  // Build a road and a residential zone.
  EXPECT_EQ(w.applyTool(Tool::Road, 3, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, 4, 3), ToolApplyResult::Applied);

  const int moneyAfterZone = w.stats().money;

  // Road/park tools should not replace existing overlays. Bulldoze first.
  EXPECT_EQ(w.applyTool(Tool::Road, 4, 3), ToolApplyResult::BlockedOccupied);
  EXPECT_EQ(w.at(4, 3).overlay, Overlay::Residential);
  EXPECT_EQ(w.stats().money, moneyAfterZone);

  EXPECT_EQ(w.applyTool(Tool::Park, 4, 3), ToolApplyResult::BlockedOccupied);
  EXPECT_EQ(w.at(4, 3).overlay, Overlay::Residential);
  EXPECT_EQ(w.stats().money, moneyAfterZone);

  // Place a park on empty land and ensure road can't overwrite it either.
  EXPECT_EQ(w.applyTool(Tool::Park, 3, 4), ToolApplyResult::Applied);
  const int moneyAfterPark = w.stats().money;

  EXPECT_EQ(w.applyTool(Tool::Road, 3, 4), ToolApplyResult::BlockedOccupied);
  EXPECT_EQ(w.at(3, 4).overlay, Overlay::Park);
  EXPECT_EQ(w.stats().money, moneyAfterPark);

  // Bulldozing clears the tile, allowing subsequent placement.
  EXPECT_EQ(w.applyTool(Tool::Bulldoze, 4, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.at(4, 3).overlay, Overlay::None);

  EXPECT_EQ(w.applyTool(Tool::Road, 4, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.at(4, 3).overlay, Overlay::Road);
}

void TestRoadHierarchyApplyRoadUpgradeCost()
{
  using namespace isocity;

  World w(5, 5, 123);
  w.stats().money = 100;

  const int costStreet = RoadBuildCostForLevel(1);
  const int costAvenue = RoadBuildCostForLevel(2);
  const int costHighway = RoadBuildCostForLevel(3);

  // New placement at a chosen class.
  EXPECT_EQ(w.applyRoad(2, 2, 1), ToolApplyResult::Applied);
  EXPECT_EQ(w.at(2, 2).overlay, Overlay::Road);
  EXPECT_EQ(static_cast<int>(w.at(2, 2).level), 1);
  EXPECT_EQ(w.stats().money, 100 - costStreet);

  // Upgrading charges only the delta.
  EXPECT_EQ(w.applyRoad(2, 2, 2), ToolApplyResult::Applied);
  EXPECT_EQ(static_cast<int>(w.at(2, 2).level), 2);
  EXPECT_EQ(w.stats().money, 100 - costStreet - (costAvenue - costStreet));

  // Re-applying at same or lower class is a no-op.
  const int moneyAfterAvenue = w.stats().money;
  EXPECT_EQ(w.applyRoad(2, 2, 2), ToolApplyResult::Noop);
  EXPECT_EQ(w.applyRoad(2, 2, 1), ToolApplyResult::Noop);
  EXPECT_EQ(w.stats().money, moneyAfterAvenue);

  // Upgrade again.
  EXPECT_EQ(w.applyRoad(2, 2, 3), ToolApplyResult::Applied);
  EXPECT_EQ(static_cast<int>(w.at(2, 2).level), 3);
  EXPECT_EQ(w.stats().money, 100 - costStreet - (costAvenue - costStreet) - (costHighway - costAvenue));

  // The classic Road tool still builds Streets.
  EXPECT_EQ(w.applyTool(Tool::Road, 1, 1), ToolApplyResult::Applied);
  EXPECT_EQ(w.at(1, 1).overlay, Overlay::Road);
  EXPECT_EQ(static_cast<int>(w.at(1, 1).level), 1);
}


void TestTrafficPrefersHighSpeedRoadsWhenStepsTie()
{
  using namespace isocity;

  // Two equal-step routes from the residential access road to the job access road:
  //  - Upper route is streets (slow)
  //  - Lower route is highways (fast)
  //
  // Weighted routing should choose the highway route when step-count ties.

  World w(7, 5, 3u);

  auto idx = [&](int x, int y) -> std::size_t { return static_cast<std::size_t>(y * w.width() + x); };

  // Shared edge connection + start segment.
  w.setRoad(0, 2);
  w.setRoad(1, 2);
  w.at(0, 2).level = 1;
  w.at(1, 2).level = 1;

  // Goal road (job access).
  w.setRoad(5, 2);
  w.at(5, 2).level = 1;

  // Upper street route (y=1, x=1..5).
  for (int x = 1; x <= 5; ++x) {
    w.setRoad(x, 1);
    w.at(x, 1).level = 1;
  }

  // Lower highway route (y=3, x=1..5).
  for (int x = 1; x <= 5; ++x) {
    w.setRoad(x, 3);
    w.at(x, 3).level = 3;
  }

  // Residential near the start, forced to use the north road (0,2) as access.
  w.at(0, 3).overlay = Overlay::Residential;
  w.at(0, 3).level = 3;
  w.at(0, 3).occupants = 10;

  // Industrial at the right edge, adjacent only to (5,2).
  w.at(6, 2).overlay = Overlay::Industrial;
  w.at(6, 2).level = 3;

  TrafficConfig tc;
  tc.requireOutsideConnection = true;
  tc.roadTileCapacity = 20;

  const TrafficResult tr = ComputeCommuteTraffic(w, tc, 1.0f);

  EXPECT_EQ(tr.totalCommuters, 10);
  EXPECT_EQ(tr.unreachableCommuters, 0);

  // The lower (highway) corridor should carry the commute.
  EXPECT_EQ(tr.roadTraffic[idx(3, 3)], 10);
  EXPECT_EQ(tr.roadTraffic[idx(4, 3)], 10);

  // The upper (street) corridor should be unused.
  EXPECT_EQ(tr.roadTraffic[idx(3, 1)], 0);
  EXPECT_EQ(tr.roadTraffic[idx(4, 1)], 0);
}

void TestTrafficCongestionRespectsRoadClassCapacity()
{
  using namespace isocity;

  World w(5, 5, 1);
  w.setRoad(2, 2);

  w.setOverlay(Overlay::Residential, 2, 1);
  w.at(2, 1).occupants = 40; // commuters

  w.setOverlay(Overlay::Commercial, 2, 3);

  TrafficConfig cfg{};
  cfg.requireOutsideConnection = false;
  cfg.roadTileCapacity = 28; // default Street capacity base

  // Streets should be congested at this demand.
  {
    auto r = ComputeCommuteTraffic(w, cfg, 1.0f, nullptr);
    EXPECT_EQ(r.congestedRoadTiles, 1);
    EXPECT_NEAR(r.congestion, 12.0f / 40.0f, 1e-6f);
  }

  // Upgrading to Avenue increases capacity enough to clear congestion.
  w.at(2, 2).level = 2;
  {
    auto r = ComputeCommuteTraffic(w, cfg, 1.0f, nullptr);
    EXPECT_EQ(r.congestedRoadTiles, 0);
    EXPECT_NEAR(r.congestion, 0.0f, 1e-6f);
  }
}

void TestTrafficCongestionAwareSplitsParallelRoutes()
{
  using namespace isocity;

  // Two identical parallel street corridors from a single origin to a single destination.
  // Free-flow routing is deterministic and will pick the "upper" corridor due to tie-breaking.
  // With congestion-aware incremental assignment enabled, traffic should split between both.

  World w(9, 7, 1u);

  auto idx = [&](int x, int y) -> std::size_t { return static_cast<std::size_t>(y * w.width() + x); };

  // Start and end junctions.
  w.setRoad(1, 3);
  w.at(1, 3).level = 1;
  w.setRoad(7, 3);
  w.at(7, 3).level = 1;

  // Upper and lower corridors.
  for (int x = 1; x <= 7; ++x) {
    w.setRoad(x, 2);
    w.at(x, 2).level = 1;
    w.setRoad(x, 4);
    w.at(x, 4).level = 1;
  }

  // Single residential origin (100 commuters) adjacent to start junction.
  w.at(0, 3).overlay = Overlay::Residential;
  w.at(0, 3).level = 3;
  w.at(0, 3).occupants = 100;

  // Single commercial destination adjacent to end junction.
  w.at(8, 3).overlay = Overlay::Commercial;
  w.at(8, 3).level = 3;

  TrafficConfig cfg;
  cfg.requireOutsideConnection = false;
  cfg.roadTileCapacity = 28;
  cfg.congestionAwareRouting = true;
  cfg.congestionIterations = 4;
  cfg.congestionAlpha = 0.15f;
  cfg.congestionBeta = 4.0f;
  cfg.congestionCapacityScale = 1.0f;
  cfg.congestionRatioClamp = 3.0f;

  const TrafficResult tr = ComputeCommuteTraffic(w, cfg, 1.0f, nullptr);

  EXPECT_EQ(tr.totalCommuters, 100);
  EXPECT_EQ(tr.unreachableCommuters, 0);
  EXPECT_TRUE(tr.usedCongestionAwareRouting);
  EXPECT_EQ(tr.routingPasses, 4);

  // Middle of each corridor should carry roughly half (deterministically 50/50 with 4 passes).
  EXPECT_EQ(tr.roadTraffic[idx(4, 2)], 50);
  EXPECT_EQ(tr.roadTraffic[idx(4, 4)], 50);

  // Shared start/end junctions carry all commuters.
  EXPECT_EQ(tr.roadTraffic[idx(1, 3)], 100);
  EXPECT_EQ(tr.roadTraffic[idx(7, 3)], 100);
}

void TestSaveLoadRoundTrip()
{
  using namespace isocity;

  ProcGenConfig cfg{};
  const std::uint64_t seed = 0xC0FFEEu;

  World w = GenerateWorld(32, 32, seed, cfg);

  // Ensure we have money to place a couple of tiles.
  w.stats().money = 500;

  int x = 0;
  int y = 0;
  EXPECT_TRUE(FindEmptyAdjacentPair(w, x, y));

  // Place a road at (x,y) and a residential zone at (x+1,y).
  EXPECT_EQ(w.applyTool(Tool::Road, x, y), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, x + 1, y), ToolApplyResult::Applied);

  // --- Terraforming persistence (v5) ---
  // Simulate a height edit on the road tile while keeping it above water.
  // (The in-game tools do this via Game::applyToolBrush; here we mutate the tile directly.)
  const float origH = w.at(x, y).height;
  const float newH = std::clamp(origH + 0.20f, 0.0f, 1.0f);
  w.at(x, y).height = newH;
  {
    // Match the TerrainFromHeight logic used by the save/load code.
    const float wl = std::clamp(cfg.waterLevel, 0.0f, 1.0f);
    const float sl = std::clamp(cfg.sandLevel, 0.0f, 1.0f);
    if (newH < wl)
      w.at(x, y).terrain = Terrain::Water;
    else if (newH < std::max(wl, sl))
      w.at(x, y).terrain = Terrain::Sand;
    else
      w.at(x, y).terrain = Terrain::Grass;
  }

  // Save to a temp-ish location (use cwd if temp isn't available).
  fs::path savePath = "isocity_test_save.bin";
  try {
    savePath = fs::temp_directory_path() / "isocity_test_save.bin";
  } catch (...) {
    // keep relative path
  }

  // Also verify that v6 saves persist the simulation/policy config.
  SimConfig simCfg{};
  simCfg.tickSeconds = 0.75f;
  simCfg.parkInfluenceRadius = 9;
  simCfg.requireOutsideConnection = false;
  simCfg.taxResidential = 3;
  simCfg.taxCommercial = 5;
  simCfg.taxIndustrial = 4;
  simCfg.maintenanceRoad = 2;
  simCfg.maintenancePark = 7;
  simCfg.taxHappinessPerCapita = 0.03f;
  simCfg.residentialDesirabilityWeight = 1.25f;
  simCfg.commercialDesirabilityWeight = 0.90f;
  simCfg.industrialDesirabilityWeight = 1.10f;

  std::string err;
  EXPECT_TRUE(SaveWorldBinary(w, cfg, simCfg, savePath.string(), err));

  // Sanity-check that we're writing the newest save version.
  // (We don't parse the whole file here; we just validate the header fields are present.)
  {
    std::ifstream in(savePath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(in));

    char magic[8] = {};
    std::uint32_t version = 0;
    in.read(magic, 8);
    in.read(reinterpret_cast<char*>(&version), static_cast<std::streamsize>(sizeof(version)));

    EXPECT_TRUE(in.good());
    EXPECT_TRUE(std::memcmp(magic, "ISOCITY\0", 8) == 0);
    EXPECT_EQ(version, static_cast<std::uint32_t>(7));
  }

  // Save summary should parse without loading the full world.
  {
    SaveSummary sum{};
    std::string err3;
    EXPECT_TRUE(ReadSaveSummary(savePath.string(), sum, err3, true));
    EXPECT_EQ(sum.version, static_cast<std::uint32_t>(7));
    EXPECT_EQ(sum.width, w.width());
    EXPECT_EQ(sum.height, w.height());
    EXPECT_EQ(sum.seed, w.seed());
    EXPECT_TRUE(sum.hasStats);
    EXPECT_EQ(sum.stats.day, w.stats().day);
    EXPECT_EQ(sum.stats.money, w.stats().money);
    EXPECT_TRUE(sum.hasProcCfg);
    EXPECT_NEAR(sum.procCfg.waterLevel, cfg.waterLevel, 1e-6f);
    EXPECT_TRUE(sum.hasSimCfg);
    EXPECT_EQ(sum.simCfg.taxResidential, simCfg.taxResidential);
    EXPECT_TRUE(sum.crcChecked);
    EXPECT_TRUE(sum.crcOk);
  }

  World loaded;
  ProcGenConfig loadedCfg{};
  SimConfig loadedSimCfg{};
  err.clear();
  EXPECT_TRUE(LoadWorldBinary(loaded, loadedCfg, loadedSimCfg, savePath.string(), err));

  // SimConfig should round-trip (within reasonable float epsilon).
  EXPECT_NEAR(loadedSimCfg.tickSeconds, simCfg.tickSeconds, 1e-6f);
  EXPECT_EQ(loadedSimCfg.parkInfluenceRadius, simCfg.parkInfluenceRadius);
  EXPECT_EQ(loadedSimCfg.requireOutsideConnection, simCfg.requireOutsideConnection);
  EXPECT_EQ(loadedSimCfg.taxResidential, simCfg.taxResidential);
  EXPECT_EQ(loadedSimCfg.taxCommercial, simCfg.taxCommercial);
  EXPECT_EQ(loadedSimCfg.taxIndustrial, simCfg.taxIndustrial);
  EXPECT_EQ(loadedSimCfg.maintenanceRoad, simCfg.maintenanceRoad);
  EXPECT_EQ(loadedSimCfg.maintenancePark, simCfg.maintenancePark);
  EXPECT_NEAR(loadedSimCfg.taxHappinessPerCapita, simCfg.taxHappinessPerCapita, 1e-6f);
  EXPECT_NEAR(loadedSimCfg.residentialDesirabilityWeight, simCfg.residentialDesirabilityWeight, 1e-6f);
  EXPECT_NEAR(loadedSimCfg.commercialDesirabilityWeight, simCfg.commercialDesirabilityWeight, 1e-6f);
  EXPECT_NEAR(loadedSimCfg.industrialDesirabilityWeight, simCfg.industrialDesirabilityWeight, 1e-6f);

  // Basic world identity checks.
  EXPECT_EQ(loaded.width(), w.width());
  EXPECT_EQ(loaded.height(), w.height());
  EXPECT_EQ(loaded.seed(), w.seed());

  // Check our edits survived.
  EXPECT_EQ(loaded.at(x, y).overlay, Overlay::Road);
  EXPECT_EQ(loaded.at(x + 1, y).overlay, Overlay::Residential);

  // Check terraforming survived (height is quantized in v5).
  auto quantizeHeight = [](float h) -> std::uint16_t {
    const float hc = std::clamp(h, 0.0f, 1.0f);
    const float scaled = hc * 65535.0f;
    const int q = static_cast<int>(std::lround(scaled));
    return static_cast<std::uint16_t>(std::clamp(q, 0, 65535));
  };
  EXPECT_EQ(quantizeHeight(loaded.at(x, y).height), quantizeHeight(newH));

  // Check core stats persisted.
  EXPECT_EQ(loaded.stats().money, w.stats().money);

  // Cleanup (best-effort).
  std::error_code ec;
  fs::remove(savePath, ec);
}

void TestSaveLoadDetectsCorruption()
{
  using namespace isocity;

  ProcGenConfig cfg{};
  const std::uint64_t seed = 0x12345678u;

  World w = GenerateWorld(32, 32, seed, cfg);
  w.stats().money = 500;

  int x = 0;
  int y = 0;
  EXPECT_TRUE(FindEmptyAdjacentPair(w, x, y));

  EXPECT_EQ(w.applyTool(Tool::Road, x, y), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, x + 1, y), ToolApplyResult::Applied);

  fs::path savePath = "isocity_test_corrupt_save.bin";
  try {
    savePath = fs::temp_directory_path() / "isocity_test_corrupt_save.bin";
  } catch (...) {
    // keep relative path
  }

  // Also verify that v6 saves persist the simulation/policy config.
  SimConfig simCfg{};
  simCfg.tickSeconds = 0.75f;
  simCfg.parkInfluenceRadius = 9;
  simCfg.requireOutsideConnection = false;
  simCfg.taxResidential = 3;
  simCfg.taxCommercial = 5;
  simCfg.taxIndustrial = 4;
  simCfg.maintenanceRoad = 2;
  simCfg.maintenancePark = 7;
  simCfg.taxHappinessPerCapita = 0.03f;
  simCfg.residentialDesirabilityWeight = 1.25f;
  simCfg.commercialDesirabilityWeight = 0.90f;
  simCfg.industrialDesirabilityWeight = 1.10f;

  std::string err;
  EXPECT_TRUE(SaveWorldBinary(w, cfg, simCfg, savePath.string(), err));

  // Read file bytes.
  std::vector<std::uint8_t> bytes;
  {
    std::ifstream in(savePath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(in));

    in.seekg(0, std::ios::end);
    const std::streamoff n = in.tellg();
    in.seekg(0, std::ios::beg);

    EXPECT_TRUE(n > 16);
    if (n <= 0) return;

    bytes.resize(static_cast<std::size_t>(n));
    in.read(reinterpret_cast<char*>(bytes.data()), n);
  }

  EXPECT_TRUE(bytes.size() > 16);
  if (bytes.size() <= 16) return;

  // Flip a byte in the payload (not in the CRC field at the end).
  const std::size_t flipIndex = std::min(bytes.size() - static_cast<std::size_t>(5), bytes.size() / 2);
  bytes[flipIndex] ^= 0x01u;

  {
    std::ofstream out(savePath, std::ios::binary | std::ios::trunc);
    EXPECT_TRUE(static_cast<bool>(out));
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  }

  // Save summary can still be read, but CRC should report corruption.
  {
    SaveSummary sum{};
    std::string errS;
    EXPECT_TRUE(ReadSaveSummary(savePath.string(), sum, errS, true));
    EXPECT_TRUE(sum.crcChecked);
    EXPECT_TRUE(!sum.crcOk);
  }

  // Loading a corrupted v3 save should fail and mention CRC.
  World loaded;
  ProcGenConfig loadedCfg{};
  std::string err2;
  const bool ok = LoadWorldBinary(loaded, loadedCfg, savePath.string(), err2);
  EXPECT_TRUE(!ok);
  EXPECT_TRUE(err2.find("CRC") != std::string::npos);

  std::error_code ec;
  fs::remove(savePath, ec);
}

void TestSimulatorStepInvariants()
{
  using namespace isocity;

  ProcGenConfig cfg{};
  World w = GenerateWorld(24, 24, 12345u, cfg);

  Simulator sim;
  sim.refreshDerivedStats(w);

  const int day0 = w.stats().day;
  sim.stepOnce(w);

  EXPECT_EQ(w.stats().day, day0 + 1);

  // Invariants: population and employment should not exceed capacities (based on how the sim models them).
  EXPECT_TRUE(w.stats().population <= w.stats().housingCapacity);
  EXPECT_TRUE(w.stats().employed <= w.stats().jobsCapacity);
  EXPECT_TRUE(w.stats().employed <= w.stats().population);
  EXPECT_TRUE(w.stats().employed <= w.stats().jobsCapacityAccessible);
}



void TestEmploymentCountsOnlyAccessibleJobs()
{
  using namespace isocity;

  World w(8, 8, 777u);
  w.stats().money = 10000;

  // Build an edge-connected road strip and a residential tile with outside access.
  EXPECT_EQ(w.applyTool(Tool::Road, 0, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Road, 1, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Road, 2, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Road, 3, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, 4, 3), ToolApplyResult::Applied);

  // Create a *disconnected* job zone: adjacent to a road, but that road component
  // does not touch the map edge.
  EXPECT_EQ(w.applyTool(Tool::Road, 6, 6), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Commercial, 6, 5), ToolApplyResult::Applied);

  Simulator sim;
  sim.refreshDerivedStats(w);

  // Run a few ticks so some residents move in (demand is non-zero even with 0 jobs).
  for (int i = 0; i < 12; ++i) sim.stepOnce(w);

  EXPECT_TRUE(w.stats().population > 0);

  // Total jobs exist, but they are not reachable via an outside-connected road component,
  // so they should not count toward employment or income.
  EXPECT_EQ(w.stats().jobsCapacity, 8);
  EXPECT_EQ(w.stats().jobsCapacityAccessible, 0);
  EXPECT_EQ(w.stats().employed, 0);
}


void TestOutsideConnectionAffectsZoneAccess()
{
  using namespace isocity;

  World w(8, 8, 123u);
  w.stats().money = 1000;

  // Build a disconnected road + residential tile.
  EXPECT_EQ(w.applyTool(Tool::Road, 3, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, 4, 3), ToolApplyResult::Applied);

  Simulator sim;
  sim.refreshDerivedStats(w);
  sim.stepOnce(w);

  // No road reaches the map edge yet, so the zone has no "outside" access.
  EXPECT_EQ(w.at(4, 3).occupants, static_cast<std::uint16_t>(0));

  // Connect the road component to the left border.
  EXPECT_EQ(w.applyTool(Tool::Road, 2, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Road, 1, 3), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Road, 0, 3), ToolApplyResult::Applied);

  sim.stepOnce(w);

  // With an outside connection, the residential tile should start filling.
  EXPECT_TRUE(w.at(4, 3).occupants > 0);
}

void TestRoadPathfindingToEdge()
{
  using namespace isocity;

  World w(6, 6, 123u);

  // Build a road segment not connected to any edge.
  w.setRoad(3, 3);
  {
    std::vector<Point> path;
    int cost = 0;
    EXPECT_TRUE(!FindRoadPathToEdge(w, Point{3, 3}, path, &cost));
    EXPECT_TRUE(path.empty());
  }

  // Connect it to the left edge.
  w.setRoad(2, 3);
  w.setRoad(1, 3);
  w.setRoad(0, 3);

  std::vector<Point> path;
  int cost = 0;
  EXPECT_TRUE(FindRoadPathToEdge(w, Point{3, 3}, path, &cost));
  EXPECT_EQ(cost, 3);
  EXPECT_EQ(static_cast<int>(path.size()), 4);
  EXPECT_EQ(path.front().x, 3);
  EXPECT_EQ(path.front().y, 3);
  EXPECT_EQ(path.back().x, 0);
  EXPECT_EQ(path.back().y, 3);
}


void TestRoadToEdgeMask()
{
  using namespace isocity;

  World w(6, 6, 123u);

  // A single interior road tile should not be marked as outside-connected.
  w.setRoad(3, 3);

  std::vector<std::uint8_t> mask;
  ComputeRoadsConnectedToEdge(w, mask);

  EXPECT_EQ(mask.size(),
            static_cast<std::size_t>(w.width()) * static_cast<std::size_t>(w.height()));

  const std::size_t idx = static_cast<std::size_t>(3) * static_cast<std::size_t>(w.width()) + 3u;
  EXPECT_EQ(mask[idx], static_cast<std::uint8_t>(0));
  EXPECT_TRUE(!HasAdjacentRoadConnectedToEdge(w, mask, 4, 3));

  // Connect that road component to the left edge and recompute.
  w.setRoad(2, 3);
  w.setRoad(1, 3);
  w.setRoad(0, 3);

  ComputeRoadsConnectedToEdge(w, mask);
  EXPECT_EQ(mask[idx], static_cast<std::uint8_t>(1));
  EXPECT_TRUE(HasAdjacentRoadConnectedToEdge(w, mask, 4, 3));

  // A separate isolated interior road remains disconnected.
  w.setRoad(4, 4);
  ComputeRoadsConnectedToEdge(w, mask);
  const std::size_t idx2 = static_cast<std::size_t>(4) * static_cast<std::size_t>(w.width()) + 4u;
  EXPECT_EQ(mask[idx2], static_cast<std::uint8_t>(0));
}


void TestRoadGraphPlusIntersection()
{
  using namespace isocity;

  World w(5, 5, 123u);

  // Plus sign: a 4-way intersection at (2,2) connecting 4 endpoints.
  for (int y = 0; y < w.height(); ++y) {
    w.setRoad(2, y);
  }
  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 2);
  }

  const RoadGraph g = BuildRoadGraph(w);

  EXPECT_EQ(static_cast<int>(g.nodes.size()), 5);
  EXPECT_EQ(static_cast<int>(g.edges.size()), 4);

  auto findNode = [&](int x, int y) -> int {
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
      if (g.nodes[static_cast<std::size_t>(i)].pos.x == x && g.nodes[static_cast<std::size_t>(i)].pos.y == y) return i;
    }
    return -1;
  };

  const int center = findNode(2, 2);
  EXPECT_TRUE(center >= 0);

  struct EP {
    int x;
    int y;
  };
  const EP endpoints[] = {{2, 0}, {2, 4}, {0, 2}, {4, 2}};

  for (const auto& ep : endpoints) {
    const int endId = findNode(ep.x, ep.y);
    EXPECT_TRUE(endId >= 0);

    bool found = false;
    for (const int ei : g.nodes[static_cast<std::size_t>(center)].edges) {
      if (ei < 0 || static_cast<std::size_t>(ei) >= g.edges.size()) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];

      const int other = (e.a == center) ? e.b : (e.b == center ? e.a : -1);
      if (other != endId) continue;

      found = true;
      EXPECT_EQ(e.length, 2);
      EXPECT_EQ(static_cast<int>(e.tiles.size()), 3);

      for (std::size_t i = 0; i < e.tiles.size(); ++i) {
        const Point& p = e.tiles[i];
        EXPECT_TRUE(w.inBounds(p.x, p.y));
        EXPECT_EQ(w.at(p.x, p.y).overlay, Overlay::Road);

        if (i > 0) {
          const Point& prev = e.tiles[i - 1];
          const int dx = std::abs(p.x - prev.x);
          const int dy = std::abs(p.y - prev.y);
          EXPECT_EQ(dx + dy, 1);
        }
      }
      break;
    }

    EXPECT_TRUE(found);
  }
}

void TestRoadGraphCornerCreatesNode()
{
  using namespace isocity;

  World w(4, 4, 123u);

  // L-shape:
  //   (1,1)-(2,1)
  //           |
  //         (2,3)
  w.setRoad(1, 1);
  w.setRoad(2, 1);
  w.setRoad(2, 2);
  w.setRoad(2, 3);

  const RoadGraph g = BuildRoadGraph(w);

  // Endpoints (1,1) and (2,3) plus the corner (2,1).
  EXPECT_EQ(static_cast<int>(g.nodes.size()), 3);
  EXPECT_EQ(static_cast<int>(g.edges.size()), 2);

  auto findNode = [&](int x, int y) -> int {
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
      if (g.nodes[static_cast<std::size_t>(i)].pos.x == x && g.nodes[static_cast<std::size_t>(i)].pos.y == y) return i;
    }
    return -1;
  };

  const int corner = findNode(2, 1);
  EXPECT_TRUE(corner >= 0);
  const int endA = findNode(1, 1);
  const int endB = findNode(2, 3);
  EXPECT_TRUE(endA >= 0);
  EXPECT_TRUE(endB >= 0);

  // Ensure we have an edge from the corner to each endpoint, with expected lengths.
  bool foundA = false;
  bool foundB = false;
  for (const int ei : g.nodes[static_cast<std::size_t>(corner)].edges) {
    if (ei < 0 || static_cast<std::size_t>(ei) >= g.edges.size()) continue;
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const int other = (e.a == corner) ? e.b : (e.b == corner ? e.a : -1);

    if (other == endA) {
      foundA = true;
      EXPECT_EQ(e.length, 1);
    } else if (other == endB) {
      foundB = true;
      EXPECT_EQ(e.length, 2);
    }
  }

  EXPECT_TRUE(foundA);
  EXPECT_TRUE(foundB);
}


void TestRoadPathfindingAStar()
{
  using namespace isocity;

  World w(6, 6, 999u);
  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 2);
  }

  std::vector<Point> path;
  int cost = 0;
  EXPECT_TRUE(FindRoadPathAStar(w, Point{0, 2}, Point{5, 2}, path, &cost));
  EXPECT_EQ(cost, 5);
  EXPECT_EQ(static_cast<int>(path.size()), 6);
  EXPECT_EQ(path.front().x, 0);
  EXPECT_EQ(path.back().x, 5);
}

void TestLandPathfindingAStarAvoidsWater()
{
  using namespace isocity;

  World w(7, 7, 1u);

  // Create a vertical water barrier at x=3, leaving a single land gap at (3,3).
  for (int y = 0; y < w.height(); ++y) {
    if (y == 3) continue;
    w.at(3, y).terrain = Terrain::Water;
  }

  std::vector<Point> path;
  int cost = 0;
  EXPECT_TRUE(FindLandPathAStar(w, Point{1, 3}, Point{5, 3}, path, &cost));

  // Only route is through the gap; ensure the path uses it and never steps on water.
  bool usedGap = false;
  for (const Point& p : path) {
    EXPECT_TRUE(w.at(p.x, p.y).terrain != Terrain::Water);
    if (p.x == 3 && p.y == 3) usedGap = true;
  }
  EXPECT_TRUE(usedGap);
  EXPECT_EQ(cost, 4);
  EXPECT_EQ(static_cast<int>(path.size()), 5);
}

void TestRoadBuildPathPrefersExistingRoads()
{
  using namespace isocity;

  World w(7, 7, 123u);

  // Start and goal are roads, but the direct row between them is empty land.
  // We'll also build a longer (but already-built) road detour.
  //
  // Expected behavior: FindRoadBuildPath should prefer the 0-build-cost detour
  // even though it has more steps.
  //
  //  (1,1)---(5,1)
  //    |       |
  //    |       |
  //  (1,3)---(5,3)
  for (int y = 1; y <= 3; ++y) {
    w.setRoad(1, y);
    w.setRoad(5, y);
  }
  for (int x = 1; x <= 5; ++x) {
    w.setRoad(x, 3);
  }

  std::vector<Point> path;
  int buildCost = 0;
  EXPECT_TRUE(FindRoadBuildPath(w, Point{1, 1}, Point{5, 1}, path, &buildCost));

  EXPECT_EQ(buildCost, 0);
  EXPECT_EQ(static_cast<int>(path.size()), 9); // 8 steps detour
  EXPECT_EQ(path.front().x, 1);
  EXPECT_EQ(path.front().y, 1);
  EXPECT_EQ(path.back().x, 5);
  EXPECT_EQ(path.back().y, 1);

  // Ensure we actually went down to y=3 at some point (took the detour).
  bool visitedY3 = false;
  for (const Point& p : path) {
    if (p.y == 3) visitedY3 = true;
    EXPECT_EQ(w.at(p.x, p.y).overlay, Overlay::Road);
  }
  EXPECT_TRUE(visitedY3);
}

void TestTrafficCommuteHeatmapSimple()
{
  using namespace isocity;

  // Simple horizontal road touching both edges.
  // Res zone at (2,0) commutes to Ind jobs at (6,0) via the road row y=1.
  World w(9, 3, 1u);
  for (int x = 0; x < 9; ++x) {
    w.setRoad(x, 1);
  }

  w.setOverlay(Overlay::Residential, 2, 0);
  w.at(2, 0).occupants = 10;

  w.setOverlay(Overlay::Industrial, 6, 0);

  TrafficConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.roadTileCapacity = 28;

  const TrafficResult tr = ComputeCommuteTraffic(w, cfg, 1.0f);

  EXPECT_EQ(tr.totalCommuters, 10);
  EXPECT_EQ(tr.reachableCommuters, 10);
  EXPECT_EQ(tr.unreachableCommuters, 0);
  EXPECT_TRUE(tr.avgCommute > 3.9f && tr.avgCommute < 4.1f);
  EXPECT_EQ(tr.maxTraffic, 10);
  EXPECT_TRUE(tr.congestion >= 0.0f && tr.congestion <= 0.001f);

  // Path includes x=2..6 along y=1 (origin road -> job access road).
  for (int x = 2; x <= 6; ++x) {
    const std::size_t idx = static_cast<std::size_t>(1) * static_cast<std::size_t>(w.width()) +
                            static_cast<std::size_t>(x);
    EXPECT_EQ(static_cast<int>(tr.roadTraffic[idx]), 10);
  }
}

void TestTrafficUnreachableAcrossDisconnectedEdgeComponents()
{
  using namespace isocity;

  // Two separate edge-connected road components with a gap.
  World w(9, 3, 2u);
  for (int x = 0; x <= 3; ++x) {
    w.setRoad(x, 1); // touches left edge
  }
  for (int x = 5; x <= 8; ++x) {
    w.setRoad(x, 1); // touches right edge
  }

  w.setOverlay(Overlay::Residential, 2, 0);
  w.at(2, 0).occupants = 10;

  w.setOverlay(Overlay::Industrial, 6, 0);

  TrafficConfig cfg;
  cfg.requireOutsideConnection = true;

  const TrafficResult tr = ComputeCommuteTraffic(w, cfg, 1.0f);

  EXPECT_EQ(tr.totalCommuters, 10);
  EXPECT_EQ(tr.reachableCommuters, 0);
  EXPECT_EQ(tr.unreachableCommuters, 10);
  EXPECT_EQ(tr.maxTraffic, 0);
}

void TestGoodsIndustrySuppliesCommercial()
{
  using namespace isocity;

  // Simple edge-connected road row.
  // Industrial at (2,2) supplies Commercial at (6,2) via roads on y=3.
  World w(9, 5, 1u);
  for (int x = 0; x < 9; ++x) {
    w.setRoad(x, 3);
  }

  w.setOverlay(Overlay::Industrial, 2, 2);
  w.at(2, 2).level = 1;

  w.setOverlay(Overlay::Commercial, 6, 2);
  w.at(6, 2).level = 1;

  GoodsConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.allowImports = false;
  cfg.allowExports = false;

  const GoodsResult gr = ComputeGoodsFlow(w, cfg);

  EXPECT_EQ(gr.goodsProduced, 12);
  EXPECT_EQ(gr.goodsDemand, 8);
  EXPECT_EQ(gr.goodsDelivered, 8);
  EXPECT_EQ(gr.goodsImported, 0);
  EXPECT_EQ(gr.goodsExported, 0);
  EXPECT_EQ(gr.unreachableDemand, 0);
  EXPECT_TRUE(gr.satisfaction > 0.999f);

  // Path includes x=2..6 along y=3.
  for (int x = 2; x <= 6; ++x) {
    const std::size_t idx = static_cast<std::size_t>(3) * static_cast<std::size_t>(w.width()) +
                            static_cast<std::size_t>(x);
    EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx]), 8);
  }

  // Commercial tile should show full supply.
  const std::size_t commIdx = static_cast<std::size_t>(2) * static_cast<std::size_t>(w.width()) +
                              static_cast<std::size_t>(6);
  EXPECT_TRUE(gr.commercialFill[commIdx] >= 250);
}

void TestGoodsImportsWhenNoIndustry()
{
  using namespace isocity;

  // A vertical edge-connected road line; commercial has no local industry,
  // so it must import.
  World w(5, 5, 2u);
  for (int y = 0; y < 5; ++y) {
    w.setRoad(2, y);
  }

  w.setOverlay(Overlay::Commercial, 3, 2);
  w.at(3, 2).level = 1;

  GoodsConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.allowImports = true;
  cfg.allowExports = true;

  const GoodsResult gr = ComputeGoodsFlow(w, cfg);

  EXPECT_EQ(gr.goodsProduced, 0);
  EXPECT_EQ(gr.goodsDemand, 8);
  EXPECT_EQ(gr.goodsDelivered, 8);
  EXPECT_EQ(gr.goodsImported, 8);
  EXPECT_EQ(gr.unreachableDemand, 0);
  EXPECT_TRUE(gr.satisfaction > 0.999f);

  // Adjacent road tile to the commercial zone should carry the imported goods.
  const std::size_t ridx = static_cast<std::size_t>(2) * static_cast<std::size_t>(w.width()) +
                           static_cast<std::size_t>(2);
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[ridx]), 8);
}

void TestGoodsUnreachableDemandWhenNoImports()
{
  using namespace isocity;

  // Small isolated road component; no industry and imports disabled => unreachable demand.
  World w(5, 5, 3u);
  w.setRoad(2, 2);

  w.setOverlay(Overlay::Commercial, 3, 2);
  w.at(3, 2).level = 1;

  GoodsConfig cfg;
  cfg.requireOutsideConnection = false;
  cfg.allowImports = false;
  cfg.allowExports = false;

  const GoodsResult gr = ComputeGoodsFlow(w, cfg);

  EXPECT_EQ(gr.goodsProduced, 0);
  EXPECT_EQ(gr.goodsDemand, 8);
  EXPECT_EQ(gr.goodsDelivered, 0);
  EXPECT_EQ(gr.goodsImported, 0);
  EXPECT_EQ(gr.unreachableDemand, 8);
  EXPECT_TRUE(gr.satisfaction < 0.001f);

  const std::size_t commIdx = static_cast<std::size_t>(2) * static_cast<std::size_t>(w.width()) +
                              static_cast<std::size_t>(3);
  EXPECT_EQ(static_cast<int>(gr.commercialFill[commIdx]), 0);
}

void TestLandValueParkAmenityBoostsNearby()
{
  using namespace isocity;

  World w(12, 12, 99u);

  // Place a park with an adjacent road so it counts as a "connected" park.
  w.setRoad(6, 6);
  w.setOverlay(Overlay::Park, 6, 5);

  LandValueConfig cfg;
  cfg.requireOutsideConnection = false;
  cfg.parkRadius = 6;

  const LandValueResult lv = ComputeLandValue(w, cfg);

  const std::size_t nearIdx = static_cast<std::size_t>(5) * static_cast<std::size_t>(w.width()) +
                              static_cast<std::size_t>(6);
  const std::size_t farIdx = static_cast<std::size_t>(1) * static_cast<std::size_t>(w.width()) +
                             static_cast<std::size_t>(1);

  EXPECT_TRUE(lv.parkAmenity[nearIdx] > lv.parkAmenity[farIdx]);
  EXPECT_TRUE(lv.value[nearIdx] > lv.value[farIdx]);
}

void TestLandValueWaterAmenityIncreasesNearCoast()
{
  using namespace isocity;

  World w(12, 12, 123u);

  // Create a water strip on the left edge.
  for (int y = 0; y < w.height(); ++y) {
    w.at(0, y).terrain = Terrain::Water;
  }

  LandValueConfig cfg;
  cfg.requireOutsideConnection = false;
  cfg.waterRadius = 6;

  const LandValueResult lv = ComputeLandValue(w, cfg);

  const std::size_t nearIdx = static_cast<std::size_t>(5) * static_cast<std::size_t>(w.width()) +
                              static_cast<std::size_t>(1);
  const std::size_t farIdx = static_cast<std::size_t>(5) * static_cast<std::size_t>(w.width()) +
                             static_cast<std::size_t>(6);

  EXPECT_TRUE(lv.waterAmenity[nearIdx] > lv.waterAmenity[farIdx]);
}

void TestLandValuePollutionPenalizesNearby()
{
  using namespace isocity;

  World w(12, 12, 7u);
  w.setOverlay(Overlay::Industrial, 2, 2);
  w.at(2, 2).level = 1;

  LandValueConfig cfg;
  cfg.requireOutsideConnection = false;
  cfg.pollutionRadius = 7;

  const LandValueResult lv = ComputeLandValue(w, cfg);

  const std::size_t nearIdx = static_cast<std::size_t>(2) * static_cast<std::size_t>(w.width()) +
                              static_cast<std::size_t>(3);
  const std::size_t farIdx = static_cast<std::size_t>(10) * static_cast<std::size_t>(w.width()) +
                             static_cast<std::size_t>(10);

  EXPECT_TRUE(lv.pollution[nearIdx] > lv.pollution[farIdx]);
  EXPECT_TRUE(lv.value[nearIdx] < lv.value[farIdx]);
}

void TestLandValueTrafficSpillUsesAdjacentRoadTraffic()
{
  using namespace isocity;

  World w(6, 6, 1u);
  w.setRoad(2, 2);

  TrafficResult tr;
  tr.roadTraffic.assign(static_cast<std::size_t>(w.width()) * static_cast<std::size_t>(w.height()), 0);
  tr.maxTraffic = 100;
  tr.roadTraffic[static_cast<std::size_t>(2) * static_cast<std::size_t>(w.width()) + static_cast<std::size_t>(2)] =
      100;

  LandValueConfig cfg;
  cfg.requireOutsideConnection = false;

  const LandValueResult lv = ComputeLandValue(w, cfg, &tr);

  const std::size_t adjIdx = static_cast<std::size_t>(2) * static_cast<std::size_t>(w.width()) +
                             static_cast<std::size_t>(3);
  const std::size_t farIdx = static_cast<std::size_t>(0) * static_cast<std::size_t>(w.width()) +
                             static_cast<std::size_t>(0);

  EXPECT_TRUE(lv.traffic[adjIdx] > 0.001f);
  EXPECT_TRUE(lv.traffic[farIdx] < 0.001f);
}

void TestResidentialDesirabilityPrefersHighLandValue()
{
  using namespace isocity;

  World w(9, 5, 123);

  // Outside-connected road spine.
  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 3);
  }

  // Two residential tiles (both road-adjacent).
  w.setOverlay(Overlay::Residential, 2, 2); // low value (polluted)
  w.setOverlay(Overlay::Residential, 6, 2); // high value (near park)

  // Jobs so demand isn't zero.
  w.setOverlay(Overlay::Commercial, 4, 2);

  // Park next to the road near the high-value residential.
  w.setOverlay(Overlay::Park, 6, 4);

  // Industrial pollution source near the low-value residential.
  w.setOverlay(Overlay::Industrial, 2, 1);

  // Keep the test deterministic: disable money-driven auto upgrades.
  w.stats().money = 0;

  SimConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.taxResidential = 0;
  cfg.taxCommercial = 0;
  cfg.taxIndustrial = 0;
  cfg.maintenanceRoad = 0;
  cfg.maintenancePark = 0;

  Simulator sim(cfg);

  // Run a few ticks so desirability has time to diverge occupancy targets.
  for (int i = 0; i < 6; ++i) {
    sim.stepOnce(w);
  }

  const int lowOcc = w.at(2, 2).occupants;
  const int highOcc = w.at(6, 2).occupants;
  EXPECT_TRUE(highOcc > lowOcc);
}

void TestJobAssignmentPrefersHighLandValueCommercial()
{
  using namespace isocity;

  World w(9, 5, 456);

  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 3);
  }

  // One housing tile provides population.
  w.setOverlay(Overlay::Residential, 4, 2);
  w.at(4, 2).occupants = 8;

  // Two commercial job sites. The *later* one in scan order is made more desirable.
  w.setOverlay(Overlay::Commercial, 2, 2); // low value (polluted)
  w.setOverlay(Overlay::Commercial, 6, 2); // high value (near park)

  w.setOverlay(Overlay::Park, 6, 4);
  w.setOverlay(Overlay::Industrial, 2, 1);

  w.stats().money = 0;

  SimConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.taxResidential = 0;
  cfg.taxCommercial = 0;
  cfg.taxIndustrial = 0;
  cfg.maintenanceRoad = 0;
  cfg.maintenancePark = 0;

  Simulator sim(cfg);
  sim.stepOnce(w);

  const int lowJobs = w.at(2, 2).occupants;
  const int highJobs = w.at(6, 2).occupants;
  EXPECT_TRUE(highJobs > lowJobs);
}

} // namespace

int main()
{
  TestRoadAutoTilingMasks();
  TestEditHistoryUndoRedo();
  TestEditHistoryUndoRedoFixesRoadMasksLocally();
  TestToolsDoNotOverwriteOccupiedTiles();
  TestRoadHierarchyApplyRoadUpgradeCost();
  TestTrafficCongestionRespectsRoadClassCapacity();
  TestTrafficCongestionAwareSplitsParallelRoutes();
  TestSaveLoadRoundTrip();
  TestSaveLoadDetectsCorruption();
  TestOutsideConnectionAffectsZoneAccess();
  TestSimulatorStepInvariants();
  TestEmploymentCountsOnlyAccessibleJobs();
  TestRoadPathfindingToEdge();
  TestRoadToEdgeMask();
  TestRoadGraphPlusIntersection();
  TestRoadGraphCornerCreatesNode();
  TestTrafficCommuteHeatmapSimple();
  TestTrafficPrefersHighSpeedRoadsWhenStepsTie();
  TestTrafficUnreachableAcrossDisconnectedEdgeComponents();
  TestGoodsIndustrySuppliesCommercial();
  TestGoodsImportsWhenNoIndustry();
  TestGoodsUnreachableDemandWhenNoImports();

  TestLandValueParkAmenityBoostsNearby();
  TestLandValueWaterAmenityIncreasesNearCoast();
  TestLandValuePollutionPenalizesNearby();
  TestLandValueTrafficSpillUsesAdjacentRoadTraffic();

  TestResidentialDesirabilityPrefersHighLandValue();
  TestJobAssignmentPrefersHighLandValueCommercial();


  TestRoadPathfindingAStar();
  TestLandPathfindingAStarAvoidsWater();
  TestRoadBuildPathPrefersExistingRoads();

  if (g_failures == 0) {
    std::cout << "proc_isocity_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_tests: FAILED (" << g_failures << ")\n";
  return 1;
}
