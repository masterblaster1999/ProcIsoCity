#include "isocity/EditHistory.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphExport.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/RoadGraphResilience.hpp"
#include "isocity/RoadGraphCentrality.hpp"
#include "isocity/RoadUpgradePlanner.hpp"
#include "isocity/PolicyOptimizer.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/ConfigIO.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"
#include "isocity/ParkOptimizer.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Replay.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Compression.hpp"
#include "isocity/Export.hpp"
#include "isocity/Isochrone.hpp"
#include "isocity/Heightmap.hpp"
#include "isocity/Contours.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/Hydrology.hpp"
#include "isocity/MeshExport.hpp"
#include "isocity/GltfExport.hpp"
#include "isocity/Script.hpp"
#include "isocity/Suite.hpp"
#include "isocity/AutoBuild.hpp"
#include "isocity/DistrictStats.hpp"
#include "isocity/Districting.hpp"
#include "isocity/Brush.hpp"
#include "isocity/ZoneParcels.hpp"
#include "isocity/CityBlocks.hpp"
#include "isocity/Vectorize.hpp"
#include "isocity/CityBlockGraph.hpp"
#include "isocity/BlockDistricting.hpp"
#include "isocity/ZoneMetrics.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/Blueprint.hpp"
#include "isocity/Road.hpp"
#include "isocity/WorldDiff.hpp"
#include "isocity/WorldPatch.hpp"
#include "isocity/WorldTransform.hpp"
#include "isocity/FloodFill.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
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

using namespace isocity;

bool FindEmptyAdjacentPair(isocity::World& w, int& outX, int& outY)
{
  // Find (x,y) and (x+1,y) that are buildable. ProcGen may pre-zone large portions of the map,
  // so we proactively clear the chosen tiles to ensure we have a blank spot for tool application tests.
  for (int y = 1; y < w.height() - 1; ++y) {
    for (int x = 1; x < w.width() - 2; ++x) {
      if (!w.isBuildable(x, y) || !w.isBuildable(x + 1, y)) continue;
      outX = x;
      outY = y;
      w.setOverlay(isocity::Overlay::None, x, y);
      w.setOverlay(isocity::Overlay::None, x + 1, y);
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

static void TestZonePlacementAllowsInteriorTilesInConnectedComponent()
{
  using namespace isocity;

  World w(8, 8, 1);
  w.stats().money = 10'000;

  // Seed one road tile.
  EXPECT_EQ(w.applyRoad(4, 4, 1), ToolApplyResult::Applied);

  // Place a residential tile adjacent to the road (boundary tile).
  EXPECT_EQ(w.applyTool(Tool::Residential, 4, 3), ToolApplyResult::Applied);

  // Now place an interior tile that is NOT adjacent to a road, but is connected
  // (via same-zone adjacency) to the road-adjacent boundary tile.
  EXPECT_EQ(w.hasAdjacentRoad(4, 2), false);
  EXPECT_EQ(w.applyTool(Tool::Residential, 4, 2), ToolApplyResult::Applied);

  // A disconnected zone placement should still be rejected.
  EXPECT_EQ(w.applyTool(Tool::Residential, 0, 0), ToolApplyResult::BlockedNoRoad);
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

void TestEditHistoryUndoDoesNotRequireExactBaseState()
{
  using namespace isocity;

  World w(8, 8, 777);
  w.stats().money = 100;

  EditHistory hist;

  // Stroke 1: place a road at (2,2).
  hist.beginStroke(w);
  hist.noteTilePreEdit(w, 2, 2);
  EXPECT_EQ(w.applyTool(Tool::Road, 2, 2), ToolApplyResult::Applied);
  hist.endStroke(w);
  EXPECT_EQ(w.at(2, 2).overlay, Overlay::Road);
  EXPECT_EQ(w.stats().money, 99);

  // Mutate the world again *without* recording it in the history.
  // The classic EditHistory behavior is to still undo the original stroke,
  // even if other edits or simulation steps have happened since.
  EXPECT_EQ(w.applyTool(Tool::Road, 5, 5), ToolApplyResult::Applied);
  EXPECT_EQ(w.at(5, 5).overlay, Overlay::Road);
  EXPECT_EQ(w.stats().money, 98);

  // Undo should still revert the original stroke only.
  EXPECT_TRUE(hist.undo(w));
  EXPECT_EQ(w.at(2, 2).overlay, Overlay::None);
  EXPECT_EQ(w.at(5, 5).overlay, Overlay::Road);
  EXPECT_EQ(w.stats().money, 99);
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

  // Atomic save should not leave temp/backup files behind on success.
  EXPECT_TRUE(!fs::exists(savePath.string() + ".tmp"));
  EXPECT_TRUE(!fs::exists(savePath.string() + ".bak"));

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
    EXPECT_EQ(version, static_cast<std::uint32_t>(9));
  }

  // Save summary should parse without loading the full world.
  {
    SaveSummary sum{};
    std::string err3;
    EXPECT_TRUE(ReadSaveSummary(savePath.string(), sum, err3, true));
    EXPECT_EQ(sum.version, static_cast<std::uint32_t>(9));
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

void TestSLLZCompressionRoundTrip()
{
  using namespace isocity;

  // Highly repetitive input should compress well.
  std::vector<std::uint8_t> input;
  input.reserve(4096);
  for (int i = 0; i < 1024; ++i) {
    input.push_back('A');
    input.push_back('B');
    input.push_back('C');
    input.push_back('D');
  }

  std::vector<std::uint8_t> compressed;
  EXPECT_TRUE(CompressSLLZ(input.data(), input.size(), compressed));

  std::string err;
  std::vector<std::uint8_t> decoded;
  EXPECT_TRUE(DecompressSLLZ(compressed.data(), compressed.size(), input.size(), decoded, err));
  EXPECT_EQ(decoded.size(), input.size());
  EXPECT_TRUE(std::memcmp(decoded.data(), input.data(), input.size()) == 0);
  EXPECT_TRUE(compressed.size() < input.size());

  // Non-repetitive input should still round-trip even if it doesn't compress.
  input.clear();
  for (int i = 0; i < 2048; ++i) {
    input.push_back(static_cast<std::uint8_t>((i * 131) ^ (i >> 3)));
  }

  compressed.clear();
  EXPECT_TRUE(CompressSLLZ(input.data(), input.size(), compressed));
  decoded.clear();
  err.clear();
  EXPECT_TRUE(DecompressSLLZ(compressed.data(), compressed.size(), input.size(), decoded, err));
  EXPECT_EQ(decoded.size(), input.size());
  EXPECT_TRUE(std::memcmp(decoded.data(), input.data(), input.size()) == 0);
}

void TestSaveV8UsesCompressionForLargeDeltaPayload()
{
  using namespace isocity;

  ProcGenConfig cfg{};
  const std::uint64_t seed = 0xBADC0FFEEu;

  // Create a world and apply a large number of uniform edits to make the delta payload
  // very repetitive (and therefore compressible).
  World w = GenerateWorld(64, 64, seed, cfg);
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      w.setRoad(x, y);
    }
  }

  fs::path savePath = "isocity_test_save_v8_compress.bin";
  try {
    savePath = fs::temp_directory_path() / "isocity_test_save_v8_compress.bin";
  } catch (...) {
    // keep relative path
  }

  SimConfig simCfg{};
  std::string err;
  EXPECT_TRUE(SaveWorldBinary(w, cfg, simCfg, savePath.string(), err));

  // Parse just enough of the binary file to reach the v8 compression header.
  struct ProcGenConfigBinLocal {
    float terrainScale;
    float waterLevel;
    float sandLevel;
    std::int32_t hubs;
    std::int32_t extraConnections;
    float zoneChance;
    float parkChance;
  };

  struct StatsBinLocal {
    std::int32_t day;
    std::int32_t population;
    std::int32_t housingCapacity;
    std::int32_t jobsCapacity;
    std::int32_t employed;
    float happiness;
    std::int32_t money;
    std::int32_t roads;
    std::int32_t parks;
  };

  struct SimConfigBinLocal {
    float tickSeconds;
    std::int32_t parkInfluenceRadius;
    std::uint8_t requireOutsideConnection;
    std::int32_t taxResidential;
    std::int32_t taxCommercial;
    std::int32_t taxIndustrial;
    std::int32_t maintenanceRoad;
    std::int32_t maintenancePark;
    float taxHappinessPerCapita;
    float residentialDesirabilityWeight;
    float commercialDesirabilityWeight;
    float industrialDesirabilityWeight;
  };

  struct DistrictPolicyBinLocal {
    float taxResidentialMult;
    float taxCommercialMult;
    float taxIndustrialMult;
    float roadMaintenanceMult;
    float parkMaintenanceMult;
  };

  auto readVarU32 = [](std::istream& is, std::uint32_t& out) -> bool {
    out = 0;
    std::uint32_t shift = 0;
    for (int i = 0; i < 5; ++i) {
      std::uint8_t b = 0;
      is.read(reinterpret_cast<char*>(&b), 1);
      if (!is.good()) return false;
      out |= static_cast<std::uint32_t>(b & 0x7Fu) << shift;
      if ((b & 0x80u) == 0) return true;
      shift += 7;
    }
    return false;
  };

  std::ifstream in(savePath, std::ios::binary);
  EXPECT_TRUE(static_cast<bool>(in));

  char magic[8] = {};
  std::uint32_t version = 0;
  std::uint32_t wDim = 0;
  std::uint32_t hDim = 0;
  std::uint64_t seedRead = 0;
  in.read(magic, 8);
  in.read(reinterpret_cast<char*>(&version), static_cast<std::streamsize>(sizeof(version)));
  in.read(reinterpret_cast<char*>(&wDim), static_cast<std::streamsize>(sizeof(wDim)));
  in.read(reinterpret_cast<char*>(&hDim), static_cast<std::streamsize>(sizeof(hDim)));
  in.read(reinterpret_cast<char*>(&seedRead), static_cast<std::streamsize>(sizeof(seedRead)));
  EXPECT_TRUE(in.good());
  EXPECT_TRUE(std::memcmp(magic, "ISOCITY\0", 8) == 0);
  EXPECT_EQ(version, static_cast<std::uint32_t>(9));
  EXPECT_EQ(wDim, static_cast<std::uint32_t>(w.width()));
  EXPECT_EQ(hDim, static_cast<std::uint32_t>(w.height()));
  EXPECT_EQ(seedRead, w.seed());

  struct ErosionConfigBinLocal {
    std::uint8_t enabled = 1;
    std::uint8_t riversEnabled = 1;
    std::uint8_t _pad0 = 0;
    std::uint8_t _pad1 = 0;

    std::int32_t thermalIterations = 20;
    float thermalTalus = 0.02f;
    float thermalRate = 0.50f;

    std::int32_t riverMinAccum = 0;
    float riverCarve = 0.055f;
    float riverCarvePower = 0.60f;

    std::int32_t smoothIterations = 1;
    float smoothRate = 0.25f;

    std::int32_t quantizeScale = 4096;
  };

  ProcGenConfigBinLocal pcb{};
  ErosionConfigBinLocal ecb{};
  StatsBinLocal sb{};
  SimConfigBinLocal scb{};
  in.read(reinterpret_cast<char*>(&pcb), static_cast<std::streamsize>(sizeof(pcb)));
  in.read(reinterpret_cast<char*>(&ecb), static_cast<std::streamsize>(sizeof(ecb)));
  in.read(reinterpret_cast<char*>(&sb), static_cast<std::streamsize>(sizeof(sb)));
  in.read(reinterpret_cast<char*>(&scb), static_cast<std::streamsize>(sizeof(scb)));

  std::uint8_t dpEnabled = 0;
  in.read(reinterpret_cast<char*>(&dpEnabled), 1);
  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBinLocal dpb{};
    in.read(reinterpret_cast<char*>(&dpb), static_cast<std::streamsize>(sizeof(dpb)));
  }
  EXPECT_TRUE(in.good());

  std::uint8_t method = 0;
  in.read(reinterpret_cast<char*>(&method), 1);
  EXPECT_TRUE(in.good());

  // For this intentionally repetitive delta payload, we expect SLLZ compression.
  EXPECT_EQ(method, static_cast<std::uint8_t>(CompressionMethod::SLLZ));

  std::uint32_t uncompressedSize = 0;
  std::uint32_t storedSize = 0;
  EXPECT_TRUE(readVarU32(in, uncompressedSize));
  EXPECT_TRUE(readVarU32(in, storedSize));
  EXPECT_TRUE(storedSize < uncompressedSize);

  // Make sure load works end-to-end (exercises decompression).
  World loaded;
  ProcGenConfig cfg2;
  SimConfig sim2;
  std::string err2;
  EXPECT_TRUE(LoadWorldBinary(loaded, cfg2, sim2, savePath.string(), err2));
  EXPECT_EQ(loaded.width(), w.width());
  EXPECT_EQ(loaded.height(), w.height());
  EXPECT_EQ(loaded.at(0, 0).overlay, Overlay::Road);
  EXPECT_EQ(loaded.at(loaded.width() - 1, loaded.height() - 1).overlay, Overlay::Road);
}


void TestSaveLoadBytesRoundTrip()
{
  using namespace isocity;

  ProcGenConfig cfg{};
  const std::uint64_t seed = 0xBADDCAFEu;

  World w = GenerateWorld(24, 20, seed, cfg);

  // Ensure we have money to place a couple of tiles.
  w.stats().money = 500;

  int x = 0;
  int y = 0;
  EXPECT_TRUE(FindEmptyAdjacentPair(w, x, y));

  // Place a road at (x,y) and a residential zone at (x+1,y).
  EXPECT_EQ(w.applyTool(Tool::Road, x, y), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, x + 1, y), ToolApplyResult::Applied);

  // Mutate height directly (matching the save/load terrain mapping used elsewhere).
  const float origH = w.at(x, y).height;
  const float newH = std::clamp(origH + 0.15f, 0.0f, 1.0f);

  // Save format stores height quantized to 16-bit; mirror that so hash round-trips.
  const auto quantize16 = [](float h) {
    const float hc = std::clamp(h, 0.0f, 1.0f);
    const int q = static_cast<int>(std::lround(hc * 65535.0f));
    const int qc = std::clamp(q, 0, 65535);
    return static_cast<float>(qc) / 65535.0f;
  };
  const float qH = quantize16(newH);
  w.at(x, y).height = qH;
  {
    const float wl = std::clamp(cfg.waterLevel, 0.0f, 1.0f);
    const float sl = std::clamp(cfg.sandLevel, 0.0f, 1.0f);
    if (qH < wl)
      w.at(x, y).terrain = Terrain::Water;
    else if (qH < std::max(wl, sl))
      w.at(x, y).terrain = Terrain::Sand;
    else
      w.at(x, y).terrain = Terrain::Grass;
  }

  // Ensure we also persist the simulation config when saving to bytes.
  SimConfig simCfg{};
  simCfg.tickSeconds = 0.8f;
  simCfg.requireOutsideConnection = false;
  simCfg.taxResidential = 2;
  simCfg.maintenanceRoad = 3;
  simCfg.districtPoliciesEnabled = true;

  std::vector<std::uint8_t> bytes;
  std::string err;
  EXPECT_TRUE(SaveWorldBinaryToBytes(w, cfg, simCfg, bytes, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(!bytes.empty());

  World loaded;
  ProcGenConfig loadedCfg{};
  SimConfig loadedSim{};
  std::string err2;
  EXPECT_TRUE(LoadWorldBinaryFromBytes(loaded, loadedCfg, loadedSim, bytes.data(), bytes.size(), err2));
  EXPECT_TRUE(err2.empty());

  // Hash includes tiles + stats (and should match after road mask reconstruction).
  EXPECT_EQ(HashWorld(loaded, true), HashWorld(w, true));
  EXPECT_TRUE(std::abs(loadedSim.tickSeconds - simCfg.tickSeconds) < 0.0001f);
  EXPECT_EQ(loadedSim.maintenanceRoad, simCfg.maintenanceRoad);
  EXPECT_EQ(loadedSim.taxResidential, simCfg.taxResidential);

  // Corrupt a byte and ensure the CRC check triggers.
  if (bytes.size() > 32) {
    bytes[20] ^= 0xFFu;
    World tmp;
    ProcGenConfig pc;
    SimConfig sc;
    std::string err3;
    EXPECT_TRUE(!LoadWorldBinaryFromBytes(tmp, pc, sc, bytes.data(), bytes.size(), err3));
    EXPECT_TRUE(err3.find("CRC") != std::string::npos);
  }
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

void TestZoneAccessMapAllowsInteriorTilesViaZoneConnectivity()
{
  using namespace isocity;

  World w(6, 6, 123u);

  // Build an edge-connected road strip.
  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 1);
  }

  // Zone a 3x2 residential block where the second row is interior (no direct road adjacency).
  for (int x = 1; x <= 3; ++x) {
    EXPECT_EQ(w.applyTool(Tool::Residential, x, 2), ToolApplyResult::Applied);
  }

  // Manually stamp the interior row (zoning tools normally require adjacent road).
  for (int x = 1; x <= 3; ++x) {
    Tile& t = w.at(x, 3);
    t.overlay = Overlay::Residential;
    t.level = 1;
    t.occupants = 0;
  }

  std::vector<std::uint8_t> roadToEdge;
  ComputeRoadsConnectedToEdge(w, roadToEdge);

  const ZoneAccessMap za = BuildZoneAccessMap(w, &roadToEdge);

  // (2,3) is not adjacent to a road, but should be reachable via the connected zone above.
  EXPECT_TRUE(HasZoneAccess(za, 2, 3));

  Point road{};
  EXPECT_TRUE(PickZoneAccessRoadTile(za, 2, 3, road));
  EXPECT_EQ(road.x, 2);
  EXPECT_EQ(road.y, 1);
}

void TestIsochroneTileAccessCostsRespectInteriorZoneAccess()
{
  using namespace isocity;

  // Road spine across the map with a 2x2 residential block above it.
  // The top row of the block is interior (no direct road adjacency) and must
  // be mapped via ZoneAccessMap.
  World w(5, 5, 1u);

  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 2);
  }

  // Bottom row (adjacent) can be applied normally.
  EXPECT_EQ(w.applyTool(Tool::Residential, 2, 1), ToolApplyResult::Applied);
  EXPECT_EQ(w.applyTool(Tool::Residential, 3, 1), ToolApplyResult::Applied);

  // Top interior row: stamp directly.
  w.setOverlay(Overlay::Residential, 2, 0);
  w.setOverlay(Overlay::Residential, 3, 0);

  std::vector<std::uint8_t> roadToEdge;
  ComputeRoadsConnectedToEdge(w, roadToEdge);

  // Source at the left edge.
  const int sourceIdx = 2 * w.width() + 0;

  RoadIsochroneConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.weightMode = IsochroneWeightMode::TravelTime;

  const RoadIsochroneField rf = BuildRoadIsochroneField(w, std::vector<int>{sourceIdx}, cfg, &roadToEdge);

  TileAccessCostConfig tcfg;
  tcfg.includeRoadTiles = true;
  tcfg.includeZones = true;
  tcfg.includeNonZonesAdjacentToRoad = false;
  tcfg.includeWater = false;
  tcfg.accessStepCostMilli = 0;
  tcfg.useZoneAccessMap = true;

  const std::vector<int> tc = BuildTileAccessCostField(w, rf, tcfg, &roadToEdge);

  // Travel time along a level-1 road is 1000 per edge.
  // From (0,2) to (2,2) is 2 edges => 2000.
  const std::size_t idxInterior = static_cast<std::size_t>(0) * static_cast<std::size_t>(w.width()) + 2u; // (2,0)
  EXPECT_EQ(tc[idxInterior], 2000);
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

void TestRoadGraphExportMetricsAndJson()
{
  using namespace isocity;

  World w(8, 8, 123);

  // Build a straight horizontal road segment from (1,3) -> (5,3).
  for (int x = 1; x <= 5; ++x) {
    w.setRoad(x, 3);
  }

  const RoadGraph g = BuildRoadGraph(w);
  EXPECT_EQ(static_cast<int>(g.nodes.size()), 2);
  EXPECT_EQ(static_cast<int>(g.edges.size()), 1);
  EXPECT_EQ(g.edges[0].length, 4);

  const RoadGraphMetrics m = ComputeRoadGraphMetrics(g);
  EXPECT_EQ(m.nodes, 2);
  EXPECT_EQ(m.edges, 1);
  EXPECT_EQ(m.components, 1);
  EXPECT_EQ(m.approxDiameter, 4);
  EXPECT_TRUE((m.diameterA == 0 && m.diameterB == 1) || (m.diameterA == 1 && m.diameterB == 0));

  const RoadGraphDiameter d = ComputeApproxRoadGraphDiameter(g);
  EXPECT_EQ(d.distance, 4);
  EXPECT_EQ(static_cast<int>(d.nodePath.size()), 2);

  std::vector<Point> tiles;
  EXPECT_TRUE(ExpandRoadGraphNodePathToTiles(g, d.nodePath, tiles));
  EXPECT_EQ(static_cast<int>(tiles.size()), 5);

  // Basic JSON export sanity: ensure it contains the expected metrics fields.
  std::ostringstream oss;
  std::string err;
  EXPECT_TRUE(WriteRoadGraphJson(oss, g, &m, &d, RoadGraphExportConfig{}, &err));
  const std::string json = oss.str();
  EXPECT_TRUE(json.find("\"approxDiameter\": 4") != std::string::npos);
  EXPECT_TRUE(json.find("\"nodes\"") != std::string::npos);
  EXPECT_TRUE(json.find("\"edges\"") != std::string::npos);
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



void TestRoadBuildPathMinimizesTurnsWhenStepsTie()
{
  using namespace isocity;

  World w(5, 5, 2026u);

  // Deterministic flat land, no pre-existing roads.
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      Tile& t = w.at(x, y);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
      t.height = 0.5f;
      t.variation = 0;
    }
  }

  RoadBuildPathConfig cfg{};
  cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;
  cfg.allowBridges = false;

  std::vector<Point> path;
  int cost = 0;
  EXPECT_TRUE(FindRoadBuildPath(w, Point{1, 1}, Point{3, 3}, path, &cost, cfg));
  EXPECT_TRUE(!path.empty());

  // Manhattan distance is 4 => any optimal path should have 5 nodes.
  EXPECT_EQ(static_cast<int>(path.size()), 5);

  auto countTurns = [](const std::vector<Point>& p) -> int {
    if (p.size() < 3) return 0;
    int turns = 0;

    int pdx = p[1].x - p[0].x;
    int pdy = p[1].y - p[0].y;

    for (std::size_t i = 2; i < p.size(); ++i) {
      const int dx = p[i].x - p[i - 1].x;
      const int dy = p[i].y - p[i - 1].y;

      // Ensure 4-neighborhood.
      EXPECT_TRUE(std::abs(dx) + std::abs(dy) == 1);

      if (dx != pdx || dy != pdy) ++turns;
      pdx = dx;
      pdy = dy;
    }
    return turns;
  };

  // Optimal Manhattan paths must turn at least once when both dx and dy are non-zero.
  // Among equal cost + equal step paths, the planner should choose a 1-turn path
  // (an "L" shape) rather than a zig-zag.
  EXPECT_EQ(countTurns(path), 1);
}

void TestRoadBuildPathBetweenSetsAccountsForStartTileCost()
{
  using namespace isocity;

  World w(5, 5, 2027u);

  // Deterministic flat land.
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      Tile& t = w.at(x, y);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
      t.height = 0.5f;
      t.variation = 0;
    }
  }

  // Existing road chain to the goal: (0,2)-(1,2)-(2,2)
  w.setRoad(0, 2);
  w.setRoad(1, 2);
  w.setRoad(2, 2);

  // Two candidate starts:
  //  - A: already-road (cost 0) but farther (2 steps)
  //  - B: empty tile adjacent to the goal (1 step) but would require building a new road tile (cost 1)
  const std::vector<Point> starts{Point{0, 2}, Point{2, 1}};
  const std::vector<Point> goals{Point{2, 2}};

  RoadBuildPathConfig cfg{};
  cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;

  std::vector<Point> path;
  int cost = 0;
  EXPECT_TRUE(FindRoadBuildPathBetweenSets(w, starts, goals, path, &cost, cfg));
  EXPECT_TRUE(!path.empty());

  // Correct cost accounting should prefer the all-existing-road route (cost 0),
  // even though the empty-tile start would yield fewer steps.
  EXPECT_EQ(cost, 0);
  EXPECT_EQ(path.front().x, 0);
  EXPECT_EQ(path.front().y, 2);
  EXPECT_EQ(path.back().x, 2);
  EXPECT_EQ(path.back().y, 2);
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

void TestGoodsFlowDebugOdCapturesLocalDeliveries()
{
  using namespace isocity;

  // Same setup as TestGoodsIndustrySuppliesCommercial, but request debug OD output.
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

  GoodsFlowDebug dbg;
  const GoodsResult gr = ComputeGoodsFlow(w, cfg, nullptr, nullptr, &dbg);

  EXPECT_EQ(dbg.w, w.width());
  EXPECT_EQ(dbg.h, w.height());

  // One aggregated local delivery edge: (2,3) -> (6,3) with amount 8.
  const int srcIdx = 3 * w.width() + 2;
  const int dstIdx = 3 * w.width() + 6;

  bool found = false;
  for (const GoodsOdEdge& e : dbg.od) {
    if (e.type != GoodsOdType::Local) continue;
    if (e.srcRoadIdx != srcIdx || e.dstRoadIdx != dstIdx) continue;
    found = true;
    EXPECT_EQ(e.amount, 8);
    EXPECT_EQ(e.minSteps, 4);
    EXPECT_EQ(e.maxSteps, 4);
    EXPECT_EQ(e.totalSteps, static_cast<std::uint64_t>(8u * 4u));
    EXPECT_TRUE(e.minCostMilli > 0);
    EXPECT_TRUE(e.maxCostMilli >= e.minCostMilli);
    EXPECT_EQ(e.totalCostMilli, static_cast<std::uint64_t>(e.amount) * static_cast<std::uint64_t>(e.minCostMilli));
  }

  EXPECT_TRUE(found);
  EXPECT_EQ(gr.goodsDelivered, 8);
}

void TestGoodsSplitsDemandAcrossMultipleProducers()
{
  using namespace isocity;

  // One commercial tile with demand greater than a single producer's supply.
  // The goods model should satisfy demand by pulling from the next-nearest
  // reachable producer instead of immediately marking it unreachable/importing.
  //
  // Road row is edge-connected so requireOutsideConnection can stay true.
  World w(9, 5, 42u);
  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 3);
  }

  // Two industrial producers on the same road component.
  w.setOverlay(Overlay::Industrial, 2, 2);
  w.at(2, 2).level = 1; // supply = 12

  w.setOverlay(Overlay::Industrial, 7, 2);
  w.at(7, 2).level = 1; // supply = 12

  // One commercial consumer in the middle.
  w.setOverlay(Overlay::Commercial, 4, 2);
  w.at(4, 2).level = 3; // demand = 24

  GoodsConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.allowImports = false;
  cfg.allowExports = false;

  const GoodsResult gr = ComputeGoodsFlow(w, cfg);

  EXPECT_EQ(gr.goodsProduced, 24);
  EXPECT_EQ(gr.goodsDemand, 24);
  EXPECT_EQ(gr.goodsDelivered, 24);
  EXPECT_EQ(gr.goodsImported, 0);
  EXPECT_EQ(gr.unreachableDemand, 0);
  EXPECT_TRUE(gr.satisfaction > 0.999f);

  auto idx = [&](int x, int y) -> std::size_t {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w.width()) + static_cast<std::size_t>(x);
  };

  // Traffic should show deliveries from both directions.
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx(2, 3)]), 12);
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx(3, 3)]), 12);
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx(4, 3)]), 24);
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx(5, 3)]), 12);
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx(6, 3)]), 12);
  EXPECT_EQ(static_cast<int>(gr.roadGoodsTraffic[idx(7, 3)]), 12);

  // Commercial tile should show full supply.
  const std::size_t commIdx = idx(4, 2);
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

void TestTrafficAndGoodsAcceptPrecomputedZoneAccessMap()
{
  using namespace isocity;

  // Build a small connected network with *interior* zone tiles that rely on ZoneAccessMap
  // (i.e., zone tiles not directly adjacent to a road).
  World w(9, 5, 123u);

  // Edge-connected road row.
  for (int x = 0; x < w.width(); ++x) {
    w.setRoad(x, 3);
  }

  // Residential component: only (1,2) touches the road; (1,1) is interior.
  w.setOverlay(Overlay::Residential, 1, 2);
  w.at(1, 2).level = 1;
  w.at(1, 2).occupants = 0;

  w.setOverlay(Overlay::Residential, 1, 1);
  w.at(1, 1).level = 3;
  w.at(1, 1).occupants = 10;

  // Commercial component: only (4,2) touches the road; (4,1) is interior.
  w.setOverlay(Overlay::Commercial, 4, 2);
  w.at(4, 2).level = 1;

  w.setOverlay(Overlay::Commercial, 4, 1);
  w.at(4, 1).level = 1;

  // Industrial component: only (7,2) touches the road; (7,1) is interior.
  w.setOverlay(Overlay::Industrial, 7, 2);
  w.at(7, 2).level = 1;

  w.setOverlay(Overlay::Industrial, 7, 1);
  w.at(7, 1).level = 1;

  // Precompute outside-connected roads + zone access map.
  std::vector<std::uint8_t> roadToEdge;
  ComputeRoadsConnectedToEdge(w, roadToEdge);
  const ZoneAccessMap za = BuildZoneAccessMap(w, &roadToEdge);

  // --- Traffic: precomputed ZoneAccessMap should be equivalent to the internal build ---
  TrafficConfig tc;
  tc.requireOutsideConnection = true;

  const TrafficResult t0 = ComputeCommuteTraffic(w, tc, 1.0f, &roadToEdge);
  const TrafficResult t1 = ComputeCommuteTraffic(w, tc, 1.0f, &roadToEdge, &za);

  EXPECT_EQ(t0.totalCommuters, 10);
  EXPECT_EQ(t0.totalCommuters, t1.totalCommuters);
  EXPECT_EQ(t0.reachableCommuters, t1.reachableCommuters);
  EXPECT_EQ(t0.unreachableCommuters, t1.unreachableCommuters);
  EXPECT_EQ(t0.roadTraffic, t1.roadTraffic);

  // A deliberately broken ZoneAccessMap should change the result (proves it's actually used).
  ZoneAccessMap bad;
  bad.w = w.width();
  bad.h = w.height();
  bad.roadIdx.assign(static_cast<std::size_t>(w.width()) * static_cast<std::size_t>(w.height()), -1);

  const TrafficResult tb = ComputeCommuteTraffic(w, tc, 1.0f, &roadToEdge, &bad);
  EXPECT_EQ(tb.totalCommuters, 0);

  // --- Goods: same story, but ensure demand/supply is still computed correctly ---
  GoodsConfig gc;
  gc.requireOutsideConnection = true;
  gc.allowImports = false;
  gc.allowExports = false;

  const GoodsResult g0 = ComputeGoodsFlow(w, gc, &roadToEdge);
  const GoodsResult g1 = ComputeGoodsFlow(w, gc, &roadToEdge, &za);

  EXPECT_TRUE(g0.goodsProduced > 0);
  EXPECT_TRUE(g0.goodsDemand > 0);
  EXPECT_EQ(g0.goodsProduced, g1.goodsProduced);
  EXPECT_EQ(g0.goodsDemand, g1.goodsDemand);
  EXPECT_EQ(g0.goodsDelivered, g1.goodsDelivered);
  EXPECT_EQ(g0.unreachableDemand, g1.unreachableDemand);
  EXPECT_EQ(g0.roadGoodsTraffic, g1.roadGoodsTraffic);
  EXPECT_EQ(g0.commercialFill, g1.commercialFill);

  const GoodsResult gb = ComputeGoodsFlow(w, gc, &roadToEdge, &bad);
  EXPECT_EQ(gb.goodsProduced, 0);
  EXPECT_EQ(gb.goodsDemand, 0);
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

void TestWorldHashDeterministicForSameSeed()
{
  using namespace isocity;

  ProcGenConfig pc;
  const std::uint64_t seed = 1337;

  World a = GenerateWorld(48, 48, seed, pc);
  World b = GenerateWorld(48, 48, seed, pc);

  Simulator sim;
  sim.refreshDerivedStats(a);
  sim.refreshDerivedStats(b);

  const std::uint64_t ha = HashWorld(a);
  const std::uint64_t hb = HashWorld(b);
  EXPECT_EQ(ha, hb);

  World c = GenerateWorld(48, 48, seed + 1, pc);
  sim.refreshDerivedStats(c);
  const std::uint64_t hc = HashWorld(c);

  EXPECT_TRUE(hc != ha);
}

void TestProcGenBlockZoningCreatesInteriorAccessibleZones()
{
  using namespace isocity;

  ProcGenConfig cfg{};
  cfg.zoneChance = 1.0f;
  cfg.parkChance = 0.0f;
  // Avoid water so the test is stable and focuses on zoning connectivity.
  cfg.waterLevel = -1.0f;
  cfg.sandLevel = -0.5f;
  cfg.hubs = 3;
  cfg.extraConnections = 1;

  World w = GenerateWorld(32, 24, 0xCAFEBABEu, cfg);
  const ZoneAccessMap za = BuildZoneAccessMap(w, nullptr);

  bool found = false;
  for (int y = 0; y < w.height() && !found; ++y) {
    for (int x = 0; x < w.width() && !found; ++x) {
      const Tile& t = w.at(x, y);
      const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
      if (!isZone) continue;
      // We want an interior tile (not directly adjacent to a road) that is nevertheless accessible via
      // same-zone connectivity to a road edge. This exercises ProcGen's block-aware inward growth.
      if (w.hasAdjacentRoad(x, y)) continue;
      if (HasZoneAccess(za, x, y)) {
        found = true;
      }
    }
  }

  EXPECT_TRUE(found);
}


void TestProcGenErosionToggleAffectsHash()
{
  using namespace isocity;

  ProcGenConfig pcErosion; // default (erosion enabled)
  ProcGenConfig pcNoErosion = pcErosion;
  pcNoErosion.erosion.enabled = false;

  const std::uint64_t seed = 1337;

  World a = GenerateWorld(48, 48, seed, pcErosion);
  World b = GenerateWorld(48, 48, seed, pcNoErosion);

  Simulator sim;
  sim.refreshDerivedStats(a);
  sim.refreshDerivedStats(b);

  const std::uint64_t ha = HashWorld(a);
  const std::uint64_t hb = HashWorld(b);

  EXPECT_TRUE(ha != hb);

  // Deterministic even when erosion is disabled.
  World c = GenerateWorld(48, 48, seed, pcNoErosion);
  sim.refreshDerivedStats(c);
  EXPECT_EQ(hb, HashWorld(c));
}


void TestSimulationDeterministicHashAfterTicks()
{
  using namespace isocity;

  ProcGenConfig pc;
  const std::uint64_t seed = 424242;

  World a = GenerateWorld(48, 48, seed, pc);
  World b = GenerateWorld(48, 48, seed, pc);

  SimConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.taxResidential = 12;
  cfg.taxCommercial = 14;
  cfg.taxIndustrial = 10;

  Simulator sa(cfg);
  Simulator sb(cfg);

  sa.refreshDerivedStats(a);
  sb.refreshDerivedStats(b);

  for (int i = 0; i < 25; ++i) {
    sa.stepOnce(a);
    sb.stepOnce(b);
  }

  const std::uint64_t ha = HashWorld(a);
  const std::uint64_t hb = HashWorld(b);
  EXPECT_EQ(ha, hb);
}



void TestBridgeRoadsCanBeBuiltOnWater()
{
  World w(7, 7, 123);
  w.stats().money = 1000;

  // Start from a clean slate (procedural gen may place roads/zones).
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      w.setOverlay(Overlay::None, x, y);
      Tile& t = w.at(x, y);
      t.height = 0.8f;
      t.terrain = Terrain::Grass;
      t.level = 1;
      t.occupants = 0;
    }
  }

  // Make a single water tile and ensure road tool can place a bridge there.
  {
    Tile& t = w.at(3, 3);
    t.height = 0.0f;
    t.terrain = Terrain::Water;

    const int before = w.stats().money;
    EXPECT_EQ(w.applyTool(Tool::Road, 3, 3), ToolApplyResult::Applied);
    EXPECT_EQ(w.at(3, 3).overlay, Overlay::Road);
    const int after = w.stats().money;
    EXPECT_EQ(before - after, RoadBridgeBuildCostForLevel(1));
  }

  // Bulldozing on water should remove the bridge road.
  {
    EXPECT_EQ(w.applyTool(Tool::Bulldoze, 3, 3), ToolApplyResult::Applied);
    EXPECT_EQ(w.at(3, 3).overlay, Overlay::None);
  }
}

void TestRoadBuildPathAvoidsBridgesWhenLandAlternativeExists()
{
  World w(7, 7, 42);
  w.stats().money = 1000;

  // Make a simple deterministic terrain: land everywhere, then a water "river" segment.
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      w.setOverlay(Overlay::None, x, y);
      Tile& t = w.at(x, y);
      t.height = 0.8f;
      t.terrain = Terrain::Grass;
      t.level = 1;
      t.occupants = 0;
    }
  }

  // Water barrier in the straight-line path: going straight crosses expensive bridge tiles,
  // but going around (one row up/down) is cheaper.
  for (int x = 1; x <= 5; ++x) {
    Tile& t = w.at(x, 3);
    t.height = 0.0f;
    t.terrain = Terrain::Water;
  }

  const Point start{0, 3};
  const Point goal{6, 3};
  std::vector<Point> path;
  const bool ok = FindRoadBuildPath(w, start, goal, path);
  EXPECT_TRUE(ok);
  EXPECT_TRUE(!path.empty());

  // The chosen path should avoid water entirely (since land is available at low additional steps).
  for (const Point& p : path) {
    EXPECT_NE(w.at(p.x, p.y).terrain, Terrain::Water);
  }
}

void TestRoadBuildPathMoneyAvoidsExpensiveBridge()
{
  using namespace isocity;

  World w(7, 7, 777u);

  // Single water tile on the straight-line route. With bridges allowed, the planner
  // *can* cross water, but the bridge multiplier should make the detour cheaper.
  w.at(3, 3).terrain = Terrain::Water;
  w.at(3, 3).height = 0.0f;

  const Point start{1, 3};
  const Point goal{5, 3};

  RoadBuildPathConfig cfg;
  cfg.allowBridges = true;
  cfg.costModel = RoadBuildPathConfig::CostModel::Money;
  cfg.targetLevel = 3; // highway: bridges are very expensive

  std::vector<Point> path;
  EXPECT_TRUE(FindRoadBuildPath(w, start, goal, path, nullptr, cfg));
  EXPECT_TRUE(!path.empty());

  // The money-aware planner should avoid the water tile.
  for (const Point& p : path) {
    EXPECT_NE(w.at(p.x, p.y).terrain, Terrain::Water);
  }
}

void TestRoadBuildPathMoneyAvoidsExpensiveUpgrades()
{
  using namespace isocity;

  World w(9, 5, 888u);

  // Deterministic flat land.
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      w.setOverlay(Overlay::None, x, y);
      Tile& t = w.at(x, y);
      t.height = 0.8f;
      t.terrain = Terrain::Grass;
      t.level = 1;
      t.occupants = 0;
    }
  }

  // Start/goal are on y=2. Provide a longer existing road detour on y=1.
  const Point start{2, 2};
  const Point goal{6, 2};

  // Existing level-1 roads (street) along the detour path.
  const Point detour[] = {Point{2, 2}, Point{2, 1}, Point{3, 1}, Point{4, 1}, Point{5, 1}, Point{6, 1}, Point{6, 2}};
  for (const Point& p : detour) {
    w.setOverlay(Overlay::Road, p.x, p.y);
    w.at(p.x, p.y).level = 1;
  }

  RoadBuildPathConfig cfg;
  cfg.allowBridges = false;
  cfg.costModel = RoadBuildPathConfig::CostModel::Money;
  cfg.targetLevel = 3; // plan a highway

  std::vector<Point> path;
  EXPECT_TRUE(FindRoadBuildPath(w, start, goal, path, nullptr, cfg));
  EXPECT_TRUE(!path.empty());

  // Money-aware behavior: upgrading 7 street tiles to highway is more expensive than
  // building a new 5-tile highway segment on y=2.
  EXPECT_EQ(static_cast<int>(path.size()), 5);
  for (const Point& p : path) {
    EXPECT_EQ(p.y, 2);
  }
}


void TestExportPpmLayers()
{
  World w(2, 2, 123);

  // Build a tiny deterministic grid with mixed terrain/overlays.
  w.at(0, 0).terrain = Terrain::Water;
  w.at(1, 0).terrain = Terrain::Sand;

  w.at(0, 1).terrain = Terrain::Grass;
  w.at(1, 1).terrain = Terrain::Grass;
  w.at(1, 1).overlay = Overlay::Road;
  w.at(1, 1).level = 3;

  // Basic render sanity
  const PpmImage terrain = RenderPpmLayer(w, ExportLayer::Terrain);
  EXPECT_EQ(terrain.width, 2);
  EXPECT_EQ(terrain.height, 2);
  EXPECT_EQ(static_cast<int>(terrain.rgb.size()), 2 * 2 * 3);

  const PpmImage overlay = RenderPpmLayer(w, ExportLayer::Overlay);
  EXPECT_EQ(overlay.width, 2);
  EXPECT_EQ(overlay.height, 2);
  EXPECT_EQ(static_cast<int>(overlay.rgb.size()), 2 * 2 * 3);

  // The road tile should differ between terrain-only and overlay render.
  const std::size_t idx11 = (1u * 2u + 1u) * 3u;
  EXPECT_TRUE(terrain.rgb[idx11 + 0] != overlay.rgb[idx11 + 0] || terrain.rgb[idx11 + 1] != overlay.rgb[idx11 + 1] ||
              terrain.rgb[idx11 + 2] != overlay.rgb[idx11 + 2]);

  // Parsing
  ExportLayer layer{};
  EXPECT_TRUE(ParseExportLayer("LV", layer));
  EXPECT_EQ(layer, ExportLayer::LandValue);
  EXPECT_TRUE(ParseExportLayer("goods_fill", layer));
  EXPECT_EQ(layer, ExportLayer::GoodsFill);

  // Nearest-neighbor scaling
  const PpmImage up = ScaleNearest(terrain, 3);
  EXPECT_EQ(up.width, 6);
  EXPECT_EQ(up.height, 6);
  EXPECT_EQ(static_cast<int>(up.rgb.size()), 6 * 6 * 3);

  // Sample a few points to verify nearest-neighbor replication:
  // Pixel (0,0) in the upscaled image corresponds to tile (0,0).
  EXPECT_EQ(up.rgb[0], terrain.rgb[0]);
  EXPECT_EQ(up.rgb[1], terrain.rgb[1]);
  EXPECT_EQ(up.rgb[2], terrain.rgb[2]);

  // Pixel (3,0) in upscaled corresponds to original (1,0) tile.
  const std::size_t idx30 = (0u * 6u + 3u) * 3u;
  const std::size_t idx10 = (0u * 2u + 1u) * 3u;
  EXPECT_EQ(up.rgb[idx30 + 0], terrain.rgb[idx10 + 0]);
  EXPECT_EQ(up.rgb[idx30 + 1], terrain.rgb[idx10 + 1]);
  EXPECT_EQ(up.rgb[idx30 + 2], terrain.rgb[idx10 + 2]);
}

void TestExportIsoOverview()
{
  World w(2, 2, 123);

  // Deterministic tiny grid.
  w.at(0, 0).terrain = Terrain::Water;
  w.at(0, 0).height = 0.0f;

  w.at(1, 0).terrain = Terrain::Sand;
  w.at(1, 0).height = 0.25f;

  w.at(0, 1).terrain = Terrain::Grass;
  w.at(0, 1).height = 0.55f;

  w.at(1, 1).terrain = Terrain::Grass;
  w.at(1, 1).height = 0.90f;
  w.at(1, 1).overlay = Overlay::Road;
  w.at(1, 1).level = 3;

  IsoOverviewConfig cfg{};
  cfg.tileW = 16;
  cfg.tileH = 8;
  cfg.heightScalePx = 12;
  cfg.marginPx = 4;
  cfg.drawGrid = false;
  cfg.drawCliffs = true;

  const IsoOverviewResult isoTerrain = RenderIsoOverview(w, ExportLayer::Terrain, cfg);
  const IsoOverviewResult isoOverlay = RenderIsoOverview(w, ExportLayer::Overlay, cfg);

  EXPECT_TRUE(isoTerrain.image.width > 0);
  EXPECT_TRUE(isoTerrain.image.height > 0);
  EXPECT_TRUE(isoOverlay.image.width > 0);
  EXPECT_TRUE(isoOverlay.image.height > 0);

  // Determinism: rendering twice should yield identical buffers.
  const IsoOverviewResult isoOverlay2 = RenderIsoOverview(w, ExportLayer::Overlay, cfg);
  EXPECT_EQ(isoOverlay.image.width, isoOverlay2.image.width);
  EXPECT_EQ(isoOverlay.image.height, isoOverlay2.image.height);
  EXPECT_EQ(static_cast<int>(isoOverlay.image.rgb.size()), static_cast<int>(isoOverlay2.image.rgb.size()));
  EXPECT_TRUE(isoOverlay.image.rgb == isoOverlay2.image.rgb);

  // The road tile should differ between terrain-only and overlay iso renders at the tile center.
  int cxT = 0, cyT = 0;
  int cxO = 0, cyO = 0;
  EXPECT_TRUE(IsoTileCenterToPixel(w, isoTerrain, 1, 1, cxT, cyT));
  EXPECT_TRUE(IsoTileCenterToPixel(w, isoOverlay, 1, 1, cxO, cyO));

  const std::size_t idxT =
      (static_cast<std::size_t>(cyT) * static_cast<std::size_t>(isoTerrain.image.width) + static_cast<std::size_t>(cxT)) * 3u;
  const std::size_t idxO =
      (static_cast<std::size_t>(cyO) * static_cast<std::size_t>(isoOverlay.image.width) + static_cast<std::size_t>(cxO)) * 3u;

  EXPECT_TRUE(idxT + 2u < isoTerrain.image.rgb.size());
  EXPECT_TRUE(idxO + 2u < isoOverlay.image.rgb.size());

  EXPECT_TRUE(isoTerrain.image.rgb[idxT + 0] != isoOverlay.image.rgb[idxO + 0] ||
              isoTerrain.image.rgb[idxT + 1] != isoOverlay.image.rgb[idxO + 1] ||
              isoTerrain.image.rgb[idxT + 2] != isoOverlay.image.rgb[idxO + 2]);
}

void TestPpmReadWriteAndCompare()
{
  using namespace isocity;
  namespace fs = std::filesystem;

  // Create an isolated temp folder for the test.
  const std::uintptr_t tag = reinterpret_cast<std::uintptr_t>(&g_failures);
  fs::path tmp = fs::temp_directory_path() / ("procisocity_ppm_test_" + std::to_string(tag));
  std::error_code ec;
  fs::remove_all(tmp, ec);
  fs::create_directories(tmp, ec);
  EXPECT_TRUE(!ec);

  // --- Write a tiny PPM ---
  PpmImage img{};
  img.width = 2;
  img.height = 2;
  img.rgb = {
      0, 0, 0,       // (0,0) black
      255, 0, 0,     // (1,0) red
      0, 255, 0,     // (0,1) green
      0, 0, 255      // (1,1) blue
  };

  std::string err;
  const fs::path pathA = tmp / "a.ppm";
  EXPECT_TRUE(WritePpm(pathA.string(), img, err));
  EXPECT_TRUE(err.empty());

  // --- Read it back ---
  PpmImage read{};
  EXPECT_TRUE(ReadPpm(pathA.string(), read, err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(read.width, 2);
  EXPECT_EQ(read.height, 2);
  EXPECT_TRUE(read.rgb == img.rgb);

  // --- Round trip PNG (minimal encoder/decoder) ---
  const fs::path pathPng = tmp / "a.png";
  EXPECT_TRUE(WritePng(pathPng.string(), img, err));
  EXPECT_TRUE(err.empty());

  PpmImage readPng{};
  EXPECT_TRUE(ReadPng(pathPng.string(), readPng, err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(readPng.width, 2);
  EXPECT_EQ(readPng.height, 2);
  EXPECT_TRUE(readPng.rgb == img.rgb);

  // --- Auto format helpers (extension or magic) ---
  const fs::path pathAuto = tmp / "auto.png";
  EXPECT_TRUE(WriteImageAuto(pathAuto.string(), img, err));
  EXPECT_TRUE(err.empty());

  PpmImage readAuto{};
  EXPECT_TRUE(ReadImageAuto(pathAuto.string(), readAuto, err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(readAuto.width, 2);
  EXPECT_EQ(readAuto.height, 2);
  EXPECT_TRUE(readAuto.rgb == img.rgb);

  // --- Compare with a single-channel change ---
  PpmImage img2 = img;
  img2.rgb[0] = 10; // bump red on pixel (0,0)

  PpmDiffStats st{};
  PpmImage diff{};
  EXPECT_TRUE(ComparePpm(img, img2, st, /*threshold=*/0, &diff));
  EXPECT_EQ(st.width, 2);
  EXPECT_EQ(st.height, 2);
  EXPECT_EQ(st.pixelsCompared, static_cast<std::uint64_t>(4));
  EXPECT_EQ(st.pixelsDifferent, static_cast<std::uint64_t>(1));
  EXPECT_EQ(static_cast<int>(st.maxAbsDiff), 10);
  EXPECT_TRUE(!diff.rgb.empty());

  // Diff image should have non-zero only at the changed pixel/channel.
  EXPECT_EQ(diff.width, 2);
  EXPECT_EQ(diff.height, 2);
  EXPECT_EQ(static_cast<int>(diff.rgb.size()), 2 * 2 * 3);
  EXPECT_EQ(static_cast<int>(diff.rgb[0]), 10);
  EXPECT_EQ(static_cast<int>(diff.rgb[1]), 0);
  EXPECT_EQ(static_cast<int>(diff.rgb[2]), 0);

  // Threshold should suppress the diff when it is <= threshold.
  PpmDiffStats st2{};
  EXPECT_TRUE(ComparePpm(img, img2, st2, /*threshold=*/10, nullptr));
  EXPECT_EQ(st2.pixelsDifferent, static_cast<std::uint64_t>(0));

  fs::remove_all(tmp, ec);
}




void TestHeightmapApplyReclassifyAndBulldoze()
{
  using namespace isocity;

  World w(2, 2, 1);

  // Start with land everywhere.
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      Tile& t = w.at(x, y);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::None;
      t.height = 1.0f;
      t.level = 1;
      t.occupants = 0;
    }
  }

  // Place a zone (should be bulldozed if the tile becomes water) and a road (bridge allowed).
  w.setOverlay(Overlay::Residential, 0, 0);
  w.setOverlay(Overlay::Road, 1, 0);

  // 2x2 grayscale image:
  //  (0,0)=0   -> water
  //  (1,0)=0   -> water (road should remain)
  //  (0,1)=255 -> grass
  //  (1,1)=128 -> sand
  PpmImage img;
  img.width = 2;
  img.height = 2;
  img.rgb = {
      0,   0,   0,   0,   0,   0,
      255, 255, 255, 128, 128, 128,
  };

  HeightmapApplyConfig cfg;
  cfg.resample = HeightmapResample::None;
  cfg.flipX = false;
  cfg.flipY = false;
  cfg.invert = false;
  cfg.heightScale = 1.0f;
  cfg.heightOffset = 0.0f;
  cfg.clamp01 = true;
  cfg.reclassifyTerrain = true;
  cfg.waterLevel = 0.20f;
  cfg.sandLevel = 0.60f;
  cfg.bulldozeNonRoadOverlaysOnWater = true;

  HeightmapApplyStats st;
  std::string err;
  EXPECT_TRUE(ApplyHeightmap(w, img, cfg, err, &st));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(w.at(0, 0).terrain == Terrain::Water);
  EXPECT_TRUE(w.at(0, 0).overlay == Overlay::None);

  EXPECT_TRUE(w.at(1, 0).terrain == Terrain::Water);
  EXPECT_TRUE(w.at(1, 0).overlay == Overlay::Road);

  EXPECT_TRUE(w.at(0, 1).terrain == Terrain::Grass);
  EXPECT_TRUE(w.at(1, 1).terrain == Terrain::Sand);

  EXPECT_EQ(st.overlaysCleared, 1ULL);
}

void TestHeightmapApplyResampleNearest()
{
  using namespace isocity;

  World w(3, 2, 1);
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      Tile& t = w.at(x, y);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::None;
      t.height = 0.0f;
    }
  }

  // 1x1 white image should upsample to all-ones with nearest.
  PpmImage img;
  img.width = 1;
  img.height = 1;
  img.rgb = {255, 255, 255};

  HeightmapApplyConfig cfg;
  cfg.resample = HeightmapResample::Nearest;
  cfg.reclassifyTerrain = false;

  std::string err;
  EXPECT_TRUE(ApplyHeightmap(w, img, cfg, err, nullptr));
  EXPECT_TRUE(err.empty());

  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      EXPECT_TRUE(std::abs(w.at(x, y).height - 1.0f) < 1e-6f);
    }
  }
}



void TestHydrologyFlowDirAndAccumulation()
{
  using namespace isocity;

  // 3x3 deterministic heightfield with a single sink at (2,2).
  // Heights decrease mostly to the right, then downward on the last column.
  const int w = 3;
  const int h = 3;
  std::vector<float> heights = {
      6.0f, 4.0f, 2.0f,
      5.0f, 3.0f, 1.0f,
      4.0f, 2.0f, 0.0f,
  };

  HydrologyField f = BuildHydrologyField(heights, w, h);
  EXPECT_EQ(f.w, w);
  EXPECT_EQ(f.h, h);
  EXPECT_EQ(static_cast<int>(f.dir.size()), w * h);
  EXPECT_EQ(static_cast<int>(f.accum.size()), w * h);

  // Flow directions (linear indices) expected:
  // (0,0)->(1,0)  (1,0)->(2,0)  (2,0)->(2,1)
  // (0,1)->(1,1)  (1,1)->(2,1)  (2,1)->(2,2)
  // (0,2)->(1,2)  (1,2)->(2,2)  (2,2)->sink
  auto I = [&](int x, int y) { return y * w + x; };

  EXPECT_EQ(f.dir[I(0, 0)], I(1, 0));
  EXPECT_EQ(f.dir[I(1, 0)], I(2, 0));
  EXPECT_EQ(f.dir[I(2, 0)], I(2, 1));
  EXPECT_EQ(f.dir[I(0, 1)], I(1, 1));
  EXPECT_EQ(f.dir[I(1, 1)], I(2, 1));
  EXPECT_EQ(f.dir[I(2, 1)], I(2, 2));
  EXPECT_EQ(f.dir[I(0, 2)], I(1, 2));
  EXPECT_EQ(f.dir[I(1, 2)], I(2, 2));
  EXPECT_EQ(f.dir[I(2, 2)], -1);

  // Accumulation expected:
  // sink (2,2) receives all 9 cells.
  EXPECT_EQ(f.accum[I(2, 2)], 9);
  EXPECT_EQ(f.maxAccum, 9);

  // Confluence at (2,1): receives from (2,0) chain and (1,1) chain.
  EXPECT_EQ(f.accum[I(2, 1)], 6);

  BasinSegmentation basins = SegmentBasins(f.dir, w, h);
  EXPECT_EQ(static_cast<int>(basins.basins.size()), 1);
  EXPECT_EQ(basins.basins[0].sinkX, 2);
  EXPECT_EQ(basins.basins[0].sinkY, 2);
  EXPECT_EQ(basins.basins[0].area, 9);

  for (int v : basins.basinId) {
    EXPECT_EQ(v, 0);
  }
}







void TestDepressionFillPriorityFloodSimpleBowl()
{
  using namespace isocity;

  const int w = 3;
  const int h = 3;
  const std::vector<float> heights = {
      1.0f, 1.0f, 1.0f,
      1.0f, 0.0f, 1.0f,
      1.0f, 1.0f, 1.0f,
  };

  const DepressionFillResult r = FillDepressionsPriorityFlood(heights, w, h);
  EXPECT_EQ(r.w, w);
  EXPECT_EQ(r.h, h);
  EXPECT_EQ(static_cast<int>(r.filled.size()), w * h);
  EXPECT_EQ(static_cast<int>(r.depth.size()), w * h);

  const auto I = [&](int x, int y) { return y * w + x; };

  // Center cell should fill to the spill height (1.0).
  EXPECT_NEAR(r.filled[I(1, 1)], 1.0f, 1e-6f);
  EXPECT_NEAR(r.depth[I(1, 1)], 1.0f, 1e-6f);

  EXPECT_EQ(r.filledCells, 1);
  EXPECT_NEAR(r.maxDepth, 1.0f, 1e-6f);
  EXPECT_NEAR(r.volume, 1.0, 1e-6);
}

void TestDepressionFillRespectsLowEdgeOutlet()
{
  using namespace isocity;

  const int w = 3;
  const int h = 3;
  const std::vector<float> heights = {
      1.0f, 1.0f, 1.0f,
      1.0f, 0.0f, 0.2f,
      1.0f, 1.0f, 1.0f,
  };

  // The low right-edge outlet (2,1) should limit how high the center is filled.
  const DepressionFillResult r = FillDepressionsPriorityFlood(heights, w, h);
  const auto I = [&](int x, int y) { return y * w + x; };

  EXPECT_NEAR(r.filled[I(1, 1)], 0.2f, 1e-6f);
  EXPECT_NEAR(r.depth[I(1, 1)], 0.2f, 1e-6f);
}

void TestSeaFloodEdgeConnectivity()
{
  using namespace isocity;

  const int w = 3;
  const int h = 3;
  const std::vector<float> heights = {
      1.0f, 1.0f, 1.0f,
      1.0f, 0.0f, 1.0f,
      1.0f, 1.0f, 1.0f,
  };

  SeaFloodConfig cfg;
  cfg.requireEdgeConnection = true;

  // No boundary cell is floodable, so connectivity-based flooding shouldn't flood the center.
  const SeaFloodResult r = ComputeSeaLevelFlood(heights, w, h, 0.5f, cfg);
  EXPECT_EQ(r.floodedCells, 0);

  cfg.requireEdgeConnection = false;
  const SeaFloodResult r2 = ComputeSeaLevelFlood(heights, w, h, 0.5f, cfg);
  EXPECT_EQ(r2.floodedCells, 1);

  const auto I = [&](int x, int y) { return y * w + x; };
  EXPECT_TRUE(r2.flooded[I(1, 1)] != 0);
  EXPECT_NEAR(r2.depth[I(1, 1)], 0.5f, 1e-6f);
}

void TestLabelComponentsAboveThresholdSimple()
{
  using namespace isocity;

  const int w = 4;
  const int h = 4;
  const std::vector<float> v = {
      0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 2.0f, 0.0f, 0.0f,
  };

  const ThresholdComponents comps = LabelComponentsAboveThreshold(v, w, h, 0.5f, false);
  EXPECT_EQ(static_cast<int>(comps.components.size()), 2);

  // First component: the 3-tile L-shape.
  EXPECT_EQ(comps.components[0].label, 1);
  EXPECT_EQ(comps.components[0].area, 3);
  EXPECT_NEAR(comps.components[0].maxValue, 1.0f, 1e-6f);

  // Second component: the single (1,3) tile.
  EXPECT_EQ(comps.components[1].label, 2);
  EXPECT_EQ(comps.components[1].area, 1);
  EXPECT_NEAR(comps.components[1].maxValue, 2.0f, 1e-6f);
}

void TestContoursMarchingSquaresDiamondLoop()
{
  using namespace isocity;

  // Corner grid (3x3) corresponds to a 2x2 tile cell area.
  // We set only the center vertex high, creating a closed diamond contour at level 0.5.
  const int cw = 3;
  const int ch = 3;
  std::vector<double> corner(static_cast<std::size_t>(cw) * static_cast<std::size_t>(ch), 0.0);
  auto Idx = [&](int x, int y) { return y * cw + x; };
  corner[static_cast<std::size_t>(Idx(1, 1))] = 1.0;

  std::vector<double> levels = {0.5};

  ContourConfig cfg;
  cfg.quantize = 1e-6;
  cfg.useAsymptoticDecider = true;
  cfg.simplifyTolerance = 0.0;
  cfg.minPoints = 2;

  std::string err;
  const std::vector<ContourLevel> res = ExtractContours(corner, cw, ch, levels, cfg, &err);
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(static_cast<int>(res.size()), 1);

  EXPECT_TRUE(std::abs(res[0].level - 0.5) < 1e-12);

  int closedCount = 0;
  const ContourPolyline* loop = nullptr;
  for (const ContourPolyline& ln : res[0].lines) {
    if (ln.closed) {
      closedCount++;
      loop = &ln;
    }
  }
  EXPECT_EQ(closedCount, 1);
  EXPECT_TRUE(loop != nullptr);

  EXPECT_TRUE(loop->pts.size() >= 5);
  EXPECT_TRUE(loop->pts.front() == loop->pts.back());

  auto HasPoint = [&](double x, double y) {
    for (const FPoint& p : loop->pts) {
      if (std::abs(p.x - x) < 1e-6 && std::abs(p.y - y) < 1e-6) return true;
    }
    return false;
  };

  // Diamond around the center vertex.
  EXPECT_TRUE(HasPoint(1.0, 0.5));
  EXPECT_TRUE(HasPoint(0.5, 1.0));
  EXPECT_TRUE(HasPoint(1.0, 1.5));
  EXPECT_TRUE(HasPoint(1.5, 1.0));
}


void TestDistrictStatsCompute()
{
  // Small 2x2 world with two districts and a simple road access setup.
  World w(2, 2, 1);

  // Layout (x, y):
  // (0,0) Res d0  occ10 lvl1
  // (1,0) Road d0 lvl1 (edge-connected)
  // (0,1) Park d1
  // (1,1) Ind d1  occ5 lvl1
  w.at(0, 0).overlay = Overlay::Residential;
  w.at(0, 0).level = 1;
  w.at(0, 0).occupants = 10;
  w.at(0, 0).district = 0;

  w.at(1, 0).overlay = Overlay::Road;
  w.at(1, 0).level = 1;
  w.at(1, 0).district = 0;

  w.at(0, 1).overlay = Overlay::Park;
  w.at(0, 1).district = 1;

  w.at(1, 1).overlay = Overlay::Industrial;
  w.at(1, 1).level = 1;
  w.at(1, 1).occupants = 5;
  w.at(1, 1).district = 1;

  SimConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.districtPoliciesEnabled = true;
  cfg.taxResidential = 2;
  cfg.taxCommercial = 3;
  cfg.taxIndustrial = 4;
  cfg.maintenanceRoad = 2;
  cfg.maintenancePark = 1;

  // District 1: double industrial tax.
  cfg.districtPolicies[1].taxIndustrialMult = 2.0f;

  // Constant land value field (0.5).
  std::vector<float> lv(4, 0.5f);

  const DistrictStatsResult ds = ComputeDistrictStats(w, cfg, &lv, nullptr);

  const auto& d0 = ds.districts[0];
  EXPECT_EQ(d0.population, 10);
  EXPECT_EQ(d0.housingCapacity, 10);
  EXPECT_EQ(d0.employed, 0);
  EXPECT_EQ(d0.jobsCapacity, 0);
  EXPECT_EQ(d0.taxRevenue, 23);
  EXPECT_EQ(d0.roadMaintenanceCost, 2);
  EXPECT_EQ(d0.maintenanceCost, 2);
  EXPECT_EQ(d0.net, 21);
  EXPECT_EQ(d0.zoneTiles, 1);
  EXPECT_EQ(d0.zoneTilesAccessible, 1);
  EXPECT_NEAR(d0.avgLandValue, 0.5f, 1e-6f);

  const auto& d1 = ds.districts[1];
  EXPECT_EQ(d1.population, 0);
  EXPECT_EQ(d1.housingCapacity, 0);
  EXPECT_EQ(d1.employed, 5);
  EXPECT_EQ(d1.jobsCapacity, 12);
  EXPECT_EQ(d1.taxRevenue, 45);
  EXPECT_EQ(d1.parkMaintenanceCost, 1);
  EXPECT_EQ(d1.maintenanceCost, 1);
  EXPECT_EQ(d1.net, 44);
  EXPECT_EQ(d1.zoneTiles, 1);
  EXPECT_EQ(d1.zoneTilesAccessible, 1);
  EXPECT_NEAR(d1.avgLandValue, 0.5f, 1e-6f);
}

void TestAutoDistrictsSeparatesDisconnectedRoadComponents()
{
  using namespace isocity;

  // Two disconnected road components should not collapse into the same district when
  // we request at least two districts.
  World w(8, 5, 1);

  // Component A: a short road segment on the left.
  w.setRoad(1, 2);
  w.setRoad(2, 2);

  // Component B: a short road segment on the right.
  w.setRoad(5, 2);
  w.setRoad(6, 2);

  AutoDistrictConfig cfg;
  cfg.districts = 2;
  cfg.requireOutsideConnection = false;
  cfg.useTravelTime = true;
  cfg.fillAllTiles = false; // only roads; simpler invariants

  const AutoDistrictResult r = AutoAssignDistricts(w, cfg);
  EXPECT_EQ(r.districtsRequested, 2);
  EXPECT_TRUE(r.districtsUsed >= 2);

  const std::uint8_t dA = w.at(1, 2).district;
  const std::uint8_t dB = w.at(5, 2).district;
  EXPECT_NE(dA, dB);

  // All road tiles in a component should share the same district.
  EXPECT_EQ(w.at(2, 2).district, dA);
  EXPECT_EQ(w.at(6, 2).district, dB);
}

void TestAutoDistrictsFillAllTilesIsDeterministic()
{
  using namespace isocity;

  World a(12, 12, 123);
  World b = a;

  // Build a small cross of roads.
  for (int x = 2; x <= 9; ++x) {
    a.setRoad(x, 6);
    b.setRoad(x, 6);
  }
  for (int y = 2; y <= 9; ++y) {
    a.setRoad(6, y);
    b.setRoad(6, y);
  }

  AutoDistrictConfig cfg;
  cfg.districts = 4;
  cfg.fillAllTiles = true;
  cfg.useTravelTime = true;

  const AutoDistrictResult ra = AutoAssignDistricts(a, cfg);
  const AutoDistrictResult rb = AutoAssignDistricts(b, cfg);
  EXPECT_EQ(ra.districtsUsed, rb.districtsUsed);
  EXPECT_EQ(ra.seedRoadIdx.size(), rb.seedRoadIdx.size());
  EXPECT_EQ(ra.seedRoadIdx, rb.seedRoadIdx);

  for (int y = 0; y < a.height(); ++y) {
    for (int x = 0; x < a.width(); ++x) {
      EXPECT_EQ(a.at(x, y).district, b.at(x, y).district);
      EXPECT_TRUE(a.at(x, y).district < static_cast<std::uint8_t>(kDistrictCount));
    }
  }
}

void TestWorldDiffCounts()
{
  using namespace isocity;

  World a(4, 4, 123u);
  World b = a;

  // Introduce a few controlled differences.
  b.setRoad(1, 1);
  b.setRoad(2, 1); // updates road masks on both tiles => variation differs too.

  b.at(0, 0).height = 0.25f;
  b.at(3, 3).district = 2;

  const WorldDiffStats d = DiffWorldTiles(a, b, 1e-6f);
  EXPECT_EQ(d.tilesCompared, 16);
  EXPECT_EQ(d.sizeMismatch, false);

  EXPECT_EQ(d.terrainDifferent, 0);
  EXPECT_EQ(d.overlayDifferent, 2);
  EXPECT_EQ(d.variationDifferent, 2);
  EXPECT_EQ(d.levelDifferent, 0);
  EXPECT_EQ(d.occupantsDifferent, 0);
  EXPECT_EQ(d.heightDifferent, 1);
  EXPECT_EQ(d.districtDifferent, 1);

  EXPECT_EQ(d.tilesDifferent, 4);
}

void TestWorldPatchRoundTrip()
{
  using namespace isocity;

  // Deterministic base world (flat grass).
  World base(16, 16, 123u);
  base.stats().money = 50000;

  ProcGenConfig baseProc{};
  SimConfig baseSim{};
  baseSim.requireOutsideConnection = false;

  // Give base non-zero derived stats (hash includes Stats fields).
  Simulator simBase(baseSim);
  simBase.refreshDerivedStats(base);

  // Create a target state with a few diverse edits.
  World target = base;
  target.stats().money = 60000;

  // A small road loop.
  target.applyRoad(2, 2, 1);
  target.applyRoad(3, 2, 1);
  target.applyRoad(4, 2, 1);
  target.applyRoad(4, 3, 1);
  target.applyRoad(4, 4, 1);
  target.applyRoad(3, 4, 1);
  target.applyRoad(2, 4, 1);
  target.applyRoad(2, 3, 1);

  // Zone + terraform + district paint.
  target.setOverlay(Overlay::Residential, 3, 3);
  target.at(0, 0).height = 0.125f;
  target.applyDistrict(1, 1, 3);

  // Also change configs so the patch can transport them.
  ProcGenConfig targetProc = baseProc;
  targetProc.hubs = 7;
  SimConfig targetSim = baseSim;
  targetSim.taxResidential = 9;
  targetSim.districtPoliciesEnabled = true;
  targetSim.districtPolicies[1].taxResidentialMult = 1.25f;

  Simulator simTarget(targetSim);
  simTarget.refreshDerivedStats(target);

  // Make + serialize patch.
  WorldPatch patch;
  std::string err;
  EXPECT_TRUE(MakeWorldPatch(base, baseProc, baseSim, target, targetProc, targetSim, patch, err, true, true, true));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(!patch.tiles.empty());

  const fs::path tmpPatch = fs::temp_directory_path() / "isocity_patch_roundtrip.isopatch";
  EXPECT_TRUE(SaveWorldPatchBinary(patch, tmpPatch.string(), err, WorldPatchCompression::SLLZ));
  EXPECT_TRUE(err.empty());

  WorldPatch loaded;
  EXPECT_TRUE(LoadWorldPatchBinary(loaded, tmpPatch.string(), err));
  EXPECT_TRUE(err.empty());

  // Apply to a fresh copy of base.
  World applied = base;
  ProcGenConfig appliedProc = baseProc;
  SimConfig appliedSim = baseSim;
  EXPECT_TRUE(ApplyWorldPatch(applied, appliedProc, appliedSim, loaded, err, false));
  EXPECT_TRUE(err.empty());

  EXPECT_EQ(HashWorld(applied, true), HashWorld(target, true));
  const WorldDiffStats d = DiffWorldTiles(applied, target, 0.0f);
  EXPECT_EQ(d.tilesDifferent, 0);

  // Metadata should have been transported (since included).
  EXPECT_EQ(appliedProc.hubs, targetProc.hubs);
  EXPECT_EQ(appliedSim.taxResidential, targetSim.taxResidential);
  EXPECT_TRUE(appliedSim.districtPoliciesEnabled == targetSim.districtPoliciesEnabled);
  EXPECT_TRUE(appliedSim.districtPolicies[1].taxResidentialMult == targetSim.districtPolicies[1].taxResidentialMult);
}

void TestWorldPatchRejectsMismatchedBaseHash()
{
  using namespace isocity;

  World base(8, 8, 1u);
  base.stats().money = 123;

  ProcGenConfig procA{};
  SimConfig simA{};
  simA.requireOutsideConnection = false;

  Simulator sa(simA);
  sa.refreshDerivedStats(base);

  World target = base;
  target.setRoad(1, 1);

  ProcGenConfig procB = procA;
  SimConfig simB = simA;
  Simulator sb(simB);
  sb.refreshDerivedStats(target);

  WorldPatch patch;
  std::string err;
  EXPECT_TRUE(MakeWorldPatch(base, procA, simA, target, procB, simB, patch, err, true, true, true));
  EXPECT_TRUE(err.empty());

  // Change the base stats (hash includes stats) so strict apply should reject.
  World mutated = base;
  mutated.stats().money += 1;

  ProcGenConfig outProc = procA;
  SimConfig outSim = simA;

  EXPECT_TRUE(!ApplyWorldPatch(mutated, outProc, outSim, patch, err, false));
  EXPECT_TRUE(!err.empty());

  // But force should apply.
  err.clear();
  EXPECT_TRUE(ApplyWorldPatch(mutated, outProc, outSim, patch, err, true));
  EXPECT_TRUE(err.empty());
}


void TestWorldPatchInvertAndCompose()
{
  using namespace isocity;

  // Deterministic base world (flat grass).
  World base(16, 16, 42u);
  base.stats().money = 50000;
  base.stats().day = 3;

  ProcGenConfig baseProc{};
  baseProc.zoneChance = 0.12f;

  SimConfig baseSim{};
  baseSim.requireOutsideConnection = false;
  baseSim.taxResidential = 3;

  // Give base non-zero derived stats (hash includes Stats fields).
  Simulator simBase(baseSim);
  simBase.refreshDerivedStats(base);

  // Mid-state edits.
  World mid = base;
  mid.stats().money = 51000;
  mid.stats().day = 4;

  mid.applyRoad(2, 2, 1);
  mid.applyRoad(3, 2, 1);
  mid.setOverlay(Overlay::Residential, 3, 3);
  mid.at(3, 3).level = 2;
  mid.at(3, 3).occupants = 7;
  mid.applyDistrict(3, 3, 1);

  ProcGenConfig midProc = baseProc;
  midProc.zoneChance = 0.20f;

  SimConfig midSim = baseSim;
  midSim.taxResidential = 5;

  Simulator simMid(midSim);
  simMid.refreshDerivedStats(mid);

  // Target-state edits.
  World target = mid;
  target.stats().money = 52000;
  target.stats().day = 5;

  target.applyRoad(10, 10, 2);
  target.setOverlay(Overlay::Commercial, 4, 3);
  target.at(4, 3).level = 3;
  target.at(4, 3).occupants = 9;
  target.applyDistrict(4, 3, 2);

  ProcGenConfig targetProc = midProc;
  targetProc.zoneChance = 0.33f;

  SimConfig targetSim = midSim;
  targetSim.taxResidential = 9;
  targetSim.districtPoliciesEnabled = true;
  targetSim.districtPolicies[0].taxCommercialMult = 1.50f;

  Simulator simTarget(targetSim);
  simTarget.refreshDerivedStats(target);

  std::string err;

  WorldPatch p1;
  EXPECT_TRUE(MakeWorldPatch(base, baseProc, baseSim, mid, midProc, midSim, p1, err, true, true, true));
  EXPECT_TRUE(err.empty());

  WorldPatch p2;
  EXPECT_TRUE(MakeWorldPatch(mid, midProc, midSim, target, targetProc, targetSim, p2, err, true, true, true));
  EXPECT_TRUE(err.empty());

  // Compose p1+p2 into a single base->target patch.
  WorldPatch composed;
  std::vector<WorldPatch> chain{p1, p2};
  EXPECT_TRUE(ComposeWorldPatches(base, baseProc, baseSim, chain, composed, err, true, true, true, false));
  EXPECT_TRUE(err.empty());

  // Applying composed patch should match target state.
  World applied = base;
  ProcGenConfig appliedProc = baseProc;
  SimConfig appliedSim = baseSim;

  EXPECT_TRUE(ApplyWorldPatch(applied, appliedProc, appliedSim, composed, err, false));
  EXPECT_TRUE(err.empty());

  EXPECT_EQ(HashWorld(applied, true), HashWorld(target, true));
  const WorldDiffStats d = DiffWorldTiles(applied, target, 0.0f);
  EXPECT_EQ(d.tilesDifferent, 0);

  EXPECT_TRUE(appliedProc.zoneChance == targetProc.zoneChance);
  EXPECT_TRUE(appliedSim.taxResidential == targetSim.taxResidential);
  EXPECT_TRUE(appliedSim.districtPoliciesEnabled == targetSim.districtPoliciesEnabled);
  EXPECT_TRUE(appliedSim.districtPolicies[0].taxCommercialMult == targetSim.districtPolicies[0].taxCommercialMult);

  // Inverting the composed patch should undo it back to the base state.
  WorldPatch inv;
  EXPECT_TRUE(InvertWorldPatch(base, baseProc, baseSim, composed, inv, err, false));
  EXPECT_TRUE(err.empty());

  World undone = applied;
  ProcGenConfig undoneProc = appliedProc;
  SimConfig undoneSim = appliedSim;

  EXPECT_TRUE(ApplyWorldPatch(undone, undoneProc, undoneSim, inv, err, false));
  EXPECT_TRUE(err.empty());

  EXPECT_EQ(HashWorld(undone, true), HashWorld(base, true));
  const WorldDiffStats d2 = DiffWorldTiles(undone, base, 0.0f);
  EXPECT_EQ(d2.tilesDifferent, 0);

  EXPECT_TRUE(undoneProc.zoneChance == baseProc.zoneChance);
  EXPECT_TRUE(undoneSim.taxResidential == baseSim.taxResidential);
  EXPECT_TRUE(undoneSim.districtPoliciesEnabled == baseSim.districtPoliciesEnabled);
}


void TestConfigJsonIO()
{
  using namespace isocity;

  // --- ProcGenConfig round-trip ---
  ProcGenConfig proc;
  proc.terrainScale = 0.123f;
  proc.waterLevel = 0.55f;
  proc.sandLevel = 0.60f;
  proc.hubs = 3;
  proc.extraConnections = 7;
  proc.zoneChance = 0.10f;
  proc.parkChance = 0.99f;
  proc.erosion.enabled = false;
  proc.erosion.thermalIterations = 7;
  proc.erosion.thermalTalus = 0.07f;
  proc.erosion.riverMinAccum = 123;
  proc.erosion.riverCarve = 0.091f;
  proc.erosion.riverCarvePower = 0.42f;
  proc.erosion.smoothIterations = 2;
  proc.erosion.smoothRate = 0.12f;
  proc.erosion.quantizeScale = 512;

  const std::string procJson = ProcGenConfigToJson(proc, 2);
  JsonValue procRoot;
  std::string err;
  EXPECT_TRUE(ParseJson(procJson, procRoot, err));
  EXPECT_TRUE(err.empty());

  ProcGenConfig proc2{};
  EXPECT_TRUE(ApplyProcGenConfigJson(procRoot, proc2, err));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(std::fabs(proc2.terrainScale - proc.terrainScale) < 1e-6f);
  EXPECT_TRUE(std::fabs(proc2.waterLevel - proc.waterLevel) < 1e-6f);
  EXPECT_TRUE(std::fabs(proc2.sandLevel - proc.sandLevel) < 1e-6f);
  EXPECT_EQ(proc2.hubs, proc.hubs);
  EXPECT_EQ(proc2.extraConnections, proc.extraConnections);
  EXPECT_TRUE(std::fabs(proc2.zoneChance - proc.zoneChance) < 1e-6f);
  EXPECT_TRUE(std::fabs(proc2.parkChance - proc.parkChance) < 1e-6f);
  EXPECT_EQ(proc2.erosion.enabled, proc.erosion.enabled);
  EXPECT_EQ(proc2.erosion.thermalIterations, proc.erosion.thermalIterations);
  EXPECT_TRUE(std::fabs(proc2.erosion.thermalTalus - proc.erosion.thermalTalus) < 1e-6f);
  EXPECT_EQ(proc2.erosion.riverMinAccum, proc.erosion.riverMinAccum);
  EXPECT_TRUE(std::fabs(proc2.erosion.riverCarve - proc.erosion.riverCarve) < 1e-6f);
  EXPECT_TRUE(std::fabs(proc2.erosion.riverCarvePower - proc.erosion.riverCarvePower) < 1e-6f);
  EXPECT_EQ(proc2.erosion.smoothIterations, proc.erosion.smoothIterations);
  EXPECT_TRUE(std::fabs(proc2.erosion.smoothRate - proc.erosion.smoothRate) < 1e-6f);
  EXPECT_EQ(proc2.erosion.quantizeScale, proc.erosion.quantizeScale);

  // --- SimConfig round-trip ---
  SimConfig sim;
  sim.tickSeconds = 0.42f;
  sim.parkInfluenceRadius = 9;
  sim.requireOutsideConnection = false;
  sim.taxResidential = 11;
  sim.taxCommercial = 2;
  sim.taxIndustrial = 15;
  sim.maintenanceRoad = 7;
  sim.maintenancePark = 13;
  sim.taxHappinessPerCapita = 0.10f;
  sim.residentialDesirabilityWeight = 2.0f;
  sim.commercialDesirabilityWeight = 3.0f;
  sim.industrialDesirabilityWeight = 4.0f;

  sim.districtPoliciesEnabled = true;
  sim.districtPolicies[2].taxCommercialMult = 1.25f;
  sim.districtPolicies[2].roadMaintenanceMult = 0.50f;
  sim.districtPolicies[4].parkMaintenanceMult = 1.75f;

  const std::string simJson = SimConfigToJson(sim, 2);
  JsonValue simRoot;
  err.clear();
  EXPECT_TRUE(ParseJson(simJson, simRoot, err));
  EXPECT_TRUE(err.empty());

  SimConfig sim2{};
  EXPECT_TRUE(ApplySimConfigJson(simRoot, sim2, err));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(std::fabs(sim2.tickSeconds - sim.tickSeconds) < 1e-6f);
  EXPECT_EQ(sim2.parkInfluenceRadius, sim.parkInfluenceRadius);
  EXPECT_EQ(sim2.requireOutsideConnection, sim.requireOutsideConnection);
  EXPECT_EQ(sim2.taxResidential, sim.taxResidential);
  EXPECT_EQ(sim2.taxCommercial, sim.taxCommercial);
  EXPECT_EQ(sim2.taxIndustrial, sim.taxIndustrial);
  EXPECT_EQ(sim2.maintenanceRoad, sim.maintenanceRoad);
  EXPECT_EQ(sim2.maintenancePark, sim.maintenancePark);
  EXPECT_TRUE(std::fabs(sim2.taxHappinessPerCapita - sim.taxHappinessPerCapita) < 1e-6f);
  EXPECT_TRUE(std::fabs(sim2.residentialDesirabilityWeight - sim.residentialDesirabilityWeight) < 1e-6f);
  EXPECT_TRUE(std::fabs(sim2.commercialDesirabilityWeight - sim.commercialDesirabilityWeight) < 1e-6f);
  EXPECT_TRUE(std::fabs(sim2.industrialDesirabilityWeight - sim.industrialDesirabilityWeight) < 1e-6f);

  EXPECT_EQ(sim2.districtPoliciesEnabled, sim.districtPoliciesEnabled);
  EXPECT_TRUE(std::fabs(sim2.districtPolicies[2].taxCommercialMult - sim.districtPolicies[2].taxCommercialMult) < 1e-6f);
  EXPECT_TRUE(std::fabs(sim2.districtPolicies[2].roadMaintenanceMult - sim.districtPolicies[2].roadMaintenanceMult) < 1e-6f);
  EXPECT_TRUE(std::fabs(sim2.districtPolicies[4].parkMaintenanceMult - sim.districtPolicies[4].parkMaintenanceMult) < 1e-6f);

  // --- Merge semantics: missing keys should not clobber existing config ---
  SimConfig base = SimConfig{};
  SimConfig defaults = SimConfig{};

  JsonValue partial;
  err.clear();
  EXPECT_TRUE(ParseJson("{\"tax_residential\":5}", partial, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(ApplySimConfigJson(partial, base, err));
  EXPECT_TRUE(err.empty());

  EXPECT_EQ(base.taxResidential, 5);
  EXPECT_EQ(base.taxCommercial, defaults.taxCommercial);
  EXPECT_EQ(base.maintenanceRoad, defaults.maintenanceRoad);

  // Sparse district policy edit by explicit id.
  SimConfig sparseBase = SimConfig{};
  JsonValue sparseRoot;
  err.clear();
  EXPECT_TRUE(ParseJson("{\"district_policies\":[{\"id\":2,\"tax_commercial_mult\":1.75}]}", sparseRoot, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(ApplySimConfigJson(sparseRoot, sparseBase, err));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(std::fabs(sparseBase.districtPolicies[2].taxCommercialMult - 1.75f) < 1e-6f);
  EXPECT_TRUE(std::fabs(sparseBase.districtPolicies[0].taxCommercialMult - 1.0f) < 1e-6f);
}


void TestBlueprintCaptureApplyRotate()
{
  using namespace isocity;

  World w(16, 16, 1u);

  // Build a small 2x2 pattern:
  // [R R]
  // [H J]
  // (Roads on top, Residential bottom-left, Commercial bottom-right)
  w.setOverlay(Overlay::Road, 2, 2);
  w.setOverlay(Overlay::Road, 3, 2);

  w.setOverlay(Overlay::Residential, 2, 3);
  w.at(2, 3).level = 2;
  w.at(2, 3).district = 2;

  w.setOverlay(Overlay::Commercial, 3, 3);
  w.at(3, 3).level = 1;
  w.at(3, 3).district = 2;

  w.at(2, 2).district = 2;
  w.at(3, 2).district = 2;

  BlueprintCaptureOptions cap;
  cap.fieldMask = static_cast<std::uint8_t>(TileFieldMask::Overlay) |
                  static_cast<std::uint8_t>(TileFieldMask::Level) |
                  static_cast<std::uint8_t>(TileFieldMask::District) |
                  static_cast<std::uint8_t>(TileFieldMask::Variation);
  cap.sparseByOverlay = false;

  Blueprint bp;
  std::string err;
  EXPECT_TRUE(CaptureBlueprintRect(w, 2, 2, 2, 2, bp, err, cap));
  EXPECT_TRUE(err.empty());

  // Round-trip through binary serialization.
  std::vector<std::uint8_t> bytes;
  EXPECT_TRUE(SerializeBlueprintBinary(bp, bytes, err, BlueprintCompression::SLLZ));
  EXPECT_TRUE(err.empty());

  Blueprint bp2;
  EXPECT_TRUE(DeserializeBlueprintBinary(bytes, bp2, err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(bp2.width, 2);
  EXPECT_EQ(bp2.height, 2);
  EXPECT_EQ(bp2.tiles.size(), bp.tiles.size());

  // Apply rotated 90° clockwise at (10, 10). After rotation, roads become vertical.
  BlueprintApplyOptions ap;
  ap.mode = BlueprintApplyMode::Replace;
  ap.fieldMask = 0xFFu;
  ap.allowOutOfBounds = false;
  ap.force = true;
  ap.recomputeRoadMasks = true;
  ap.transform.rotateDeg = 90;
  ap.transform.mirrorX = false;
  ap.transform.mirrorY = false;

  EXPECT_TRUE(ApplyBlueprint(w, bp2, 10, 10, ap, err));
  EXPECT_TRUE(err.empty());

  // Expected mapping for 2x2 rotation (cw):
  // (0,0)->(1,0)  (1,0)->(1,1)
  // (0,1)->(0,0)  (1,1)->(0,1)
  EXPECT_EQ(w.at(11, 10).overlay, Overlay::Road);
  EXPECT_EQ(w.at(11, 11).overlay, Overlay::Road);
  EXPECT_EQ(w.at(10, 10).overlay, Overlay::Residential);
  EXPECT_EQ(w.at(10, 11).overlay, Overlay::Commercial);

  EXPECT_EQ(w.at(10, 10).level, 2);
  EXPECT_EQ(w.at(10, 10).district, 2);
  EXPECT_EQ(w.at(10, 11).district, 2);

  // Road mask bits: top road connected south (bit2=4), bottom road connected north (bit0=1).
  EXPECT_EQ(static_cast<int>(w.at(11, 10).variation & 0x0Fu), 4);
  EXPECT_EQ(static_cast<int>(w.at(11, 11).variation & 0x0Fu), 1);
}


void TestBlueprintDiffCropAndTransform()
{
  using namespace isocity;

  // ---------------------------------------------------------------------------
  // Diff + crop + apply should reproduce the target world.
  // ---------------------------------------------------------------------------
  World base(12, 12, 7u);
  World target = base;

  base.setOverlay(Overlay::Road, 2, 2);
  base.setOverlay(Overlay::Road, 3, 2);
  base.setOverlay(Overlay::Residential, 2, 3);
  base.at(2, 3).level = 1;
  base.at(2, 3).district = 1;
  base.at(2, 2).district = 1;
  base.at(3, 2).district = 1;

  target = base;

  // Upgrade the residential tile and change its district.
  target.at(2, 3).level = 3;
  target.at(2, 3).district = 3;

  // Remove one road tile.
  target.bulldoze(3, 2);

  // Add a park.
  target.setOverlay(Overlay::Park, 5, 5);
  target.at(5, 5).district = 2;

  // Change height in a single tile.
  target.at(6, 6).height = base.at(6, 6).height + 0.25f;

  // Change only the upper variation nibble (road auto-mask uses the low nibble).
  target.at(2, 2).variation = static_cast<std::uint8_t>(target.at(2, 2).variation ^ 0xF0u);

  BlueprintDiffOptions diffOpt;
  diffOpt.fieldMask = static_cast<std::uint8_t>(TileFieldMask::Overlay) |
                      static_cast<std::uint8_t>(TileFieldMask::Level) |
                      static_cast<std::uint8_t>(TileFieldMask::District) |
                      static_cast<std::uint8_t>(TileFieldMask::Variation) |
                      static_cast<std::uint8_t>(TileFieldMask::Height);
  diffOpt.heightEpsilon = 0.0f;

  Blueprint diff;
  std::string err;
  EXPECT_TRUE(CaptureBlueprintDiffRect(base, target, 0, 0, base.width(), base.height(), diff, err, diffOpt));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(diff.width, base.width());
  EXPECT_EQ(diff.height, base.height());
  EXPECT_TRUE(!diff.tiles.empty());

  Blueprint cropped;
  int offX = 0;
  int offY = 0;
  EXPECT_TRUE(CropBlueprintToDeltasBounds(diff, cropped, offX, offY, err, 0));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(cropped.width <= diff.width);
  EXPECT_TRUE(cropped.height <= diff.height);

  World applied = base;
  BlueprintApplyOptions ap;
  ap.mode = BlueprintApplyMode::Replace;
  ap.fieldMask = 0xFFu;
  ap.allowOutOfBounds = false;
  ap.force = true;
  ap.recomputeRoadMasks = true;
  ap.transform.rotateDeg = 0;
  ap.transform.mirrorX = false;
  ap.transform.mirrorY = false;

  EXPECT_TRUE(ApplyBlueprint(applied, cropped, offX, offY, ap, err));
  EXPECT_TRUE(err.empty());

  for (int y = 0; y < base.height(); ++y) {
    for (int x = 0; x < base.width(); ++x) {
      const Tile& a = applied.at(x, y);
      const Tile& b = target.at(x, y);
      EXPECT_EQ(a.overlay, b.overlay);
      EXPECT_EQ(a.level, b.level);
      EXPECT_EQ(a.district, b.district);
      EXPECT_EQ(a.variation, b.variation);
      EXPECT_NEAR(a.height, b.height, 0.0f);
    }
  }

  // ---------------------------------------------------------------------------
  // Pre-transforming a blueprint should be equivalent to apply-time transforms.
  // ---------------------------------------------------------------------------

  World src(8, 8, 1u);
  src.setOverlay(Overlay::Road, 1, 1);
  src.at(1, 1).level = 2;
  src.at(1, 1).district = 4;

  src.setOverlay(Overlay::Residential, 1, 2);
  src.at(1, 2).level = 2;
  src.at(1, 2).district = 5;

  src.setOverlay(Overlay::Commercial, 2, 2);
  src.at(2, 2).level = 3;
  src.at(2, 2).district = 6;

  BlueprintCaptureOptions cap;
  cap.fieldMask = static_cast<std::uint8_t>(TileFieldMask::Overlay) |
                  static_cast<std::uint8_t>(TileFieldMask::Level) |
                  static_cast<std::uint8_t>(TileFieldMask::District) |
                  static_cast<std::uint8_t>(TileFieldMask::Variation);
  cap.sparseByOverlay = false;

  Blueprint stamp;
  EXPECT_TRUE(CaptureBlueprintRect(src, 1, 1, 3, 2, stamp, err, cap));
  EXPECT_TRUE(err.empty());

  BlueprintTransform tr;
  tr.rotateDeg = 90;
  tr.mirrorX = true;
  tr.mirrorY = false;

  Blueprint pre;
  EXPECT_TRUE(TransformBlueprint(stamp, tr, pre, err));
  EXPECT_TRUE(err.empty());

  World wA(16, 16, 9u);
  World wB = wA;

  BlueprintApplyOptions apA;
  apA.mode = BlueprintApplyMode::Replace;
  apA.fieldMask = 0xFFu;
  apA.allowOutOfBounds = false;
  apA.force = true;
  apA.recomputeRoadMasks = true;
  apA.transform = tr;

  BlueprintApplyOptions apB = apA;
  apB.transform.rotateDeg = 0;
  apB.transform.mirrorX = false;
  apB.transform.mirrorY = false;

  EXPECT_TRUE(ApplyBlueprint(wA, stamp, 10, 10, apA, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(ApplyBlueprint(wB, pre, 10, 10, apB, err));
  EXPECT_TRUE(err.empty());

  for (int y = 0; y < wA.height(); ++y) {
    for (int x = 0; x < wA.width(); ++x) {
      const Tile& a = wA.at(x, y);
      const Tile& b = wB.at(x, y);
      EXPECT_EQ(a.overlay, b.overlay);
      EXPECT_EQ(a.level, b.level);
      EXPECT_EQ(a.district, b.district);
      EXPECT_EQ(a.variation, b.variation);
    }
  }
}

void TestReplayRoundTrip()
{
  using namespace isocity;

  const fs::path tmpDir = fs::temp_directory_path();
  const fs::path basePath = tmpDir / "isocity_replay_base.bin";
  const fs::path replayPath = tmpDir / "isocity_replay_roundtrip.isoreplay";

  ProcGenConfig proc{};
  SimConfig sim{};
  sim.requireOutsideConnection = false;

  World base = GenerateWorld(24, 24, 1234ULL, proc);
  std::string err;
  EXPECT_TRUE(SaveWorldBinary(base, proc, sim, basePath.string(), err));
  EXPECT_TRUE(err.empty());

  // Create an edited intermediate state.
  World edited = base;
  edited.setRoad(2, 2);
  edited.setRoad(3, 2);
  edited.setOverlay(Overlay::Residential, 2, 3);
  edited.stats().money -= 42;

  // Patch base -> edited.
  WorldPatch patch;
  EXPECT_TRUE(MakeWorldPatch(base, proc, sim, edited, proc, sim, patch, err, true, true, true));
  EXPECT_TRUE(err.empty());

  std::vector<std::uint8_t> patchBytes;
  EXPECT_TRUE(SerializeWorldPatchBinary(patch, patchBytes, err, WorldPatchCompression::SLLZ));
  EXPECT_TRUE(err.empty());

  // Expected final state after ticks.
  World expected = edited;
  Simulator s2(sim);
  for (int i = 0; i < 7; ++i) s2.stepOnce(expected);

  // Load the base save bytes to embed.
  std::vector<std::uint8_t> baseBytes;
  {
    std::ifstream f(basePath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(f));
    f.seekg(0, std::ios::end);
    const std::streampos end = f.tellg();
    EXPECT_TRUE(end >= 0);
    f.seekg(0, std::ios::beg);
    baseBytes.resize(static_cast<std::size_t>(end));
    if (!baseBytes.empty()) {
      f.read(reinterpret_cast<char*>(baseBytes.data()), static_cast<std::streamsize>(baseBytes.size()));
      EXPECT_TRUE(f.good());
    }
  }

  Replay replay;
  replay.baseSave = std::move(baseBytes);

  ReplayEvent ePatch;
  ePatch.type = ReplayEventType::Patch;
  ePatch.patch = std::move(patchBytes);
  replay.events.push_back(std::move(ePatch));

  ReplayEvent eTick;
  eTick.type = ReplayEventType::Tick;
  eTick.ticks = 7;
  replay.events.push_back(std::move(eTick));

  EXPECT_TRUE(SaveReplayBinary(replay, replayPath.string(), err));
  EXPECT_TRUE(err.empty());

  Replay loaded;
  EXPECT_TRUE(LoadReplayBinary(loaded, replayPath.string(), err));
  EXPECT_TRUE(err.empty());

  World out;
  ProcGenConfig outProc;
  SimConfig outSim;
  std::vector<Stats> trace;
  EXPECT_TRUE(PlayReplay(loaded, out, outProc, outSim, err, true, true, &trace));
  EXPECT_TRUE(err.empty());

  EXPECT_EQ(HashWorld(out, true), HashWorld(expected, true));
  EXPECT_TRUE(trace.size() >= 8u); // base + 7 ticks
}

void TestReplayHashAsserts()
{
  using namespace isocity;

  const fs::path tmpDir = fs::temp_directory_path();
  const fs::path replayPath = tmpDir / "isocity_replay_asserts.isoreplay";

  ProcGenConfig proc{};
  SimConfig sim{};
  sim.requireOutsideConnection = false;

  World base = GenerateWorld(20, 20, 999ULL, proc);

  // Embed base save bytes.
  std::vector<std::uint8_t> baseBytes;
  std::string err;
  EXPECT_TRUE(SaveWorldBinaryToBytes(base, proc, sim, baseBytes, err));
  EXPECT_TRUE(err.empty());

  // Expected world after ticks.
  World expected = base;
  Simulator s(sim);
  for (int i = 0; i < 5; ++i) s.stepOnce(expected);
  const std::uint64_t expectedHash = HashWorld(expected, true);

  Replay replay;
  replay.baseSave = std::move(baseBytes);

  ReplayEvent note;
  note.type = ReplayEventType::Note;
  note.note = "unit_test";
  replay.events.push_back(note);

  ReplayEvent tick;
  tick.type = ReplayEventType::Tick;
  tick.ticks = 5;
  replay.events.push_back(tick);

  ReplayEvent as;
  as.type = ReplayEventType::AssertHash;
  as.expectedHash = expectedHash;
  as.includeStatsInHash = true;
  as.label = "after 5 ticks";
  replay.events.push_back(as);

  EXPECT_TRUE(SaveReplayBinary(replay, replayPath.string(), err));
  EXPECT_TRUE(err.empty());

  Replay loaded;
  EXPECT_TRUE(LoadReplayBinary(loaded, replayPath.string(), err));
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(loaded.version, 2u);
  EXPECT_EQ(loaded.events.size(), replay.events.size());

  // Strict asserts should pass.
  World out;
  ProcGenConfig outProc;
  SimConfig outSim;
  EXPECT_TRUE(PlayReplay(loaded, out, outProc, outSim, err, true, true, nullptr));
  EXPECT_TRUE(err.empty());

  // Corrupt the expected hash and verify strict assert fails.
  loaded.events.back().expectedHash ^= 0x1234ULL;
  EXPECT_TRUE(!PlayReplay(loaded, out, outProc, outSim, err, true, true, nullptr));
  EXPECT_TRUE(!err.empty());

  // But relaxed asserts should ignore it.
  err.clear();
  EXPECT_TRUE(PlayReplay(loaded, out, outProc, outSim, err, true, false, nullptr));
  EXPECT_TRUE(err.empty());
}


} // namespace


static void TestZoneBuildingParcelsDeterministic()
{
  using namespace isocity;

  World world = GenerateWorld(32, 32, 1234567ULL);

  ZoneBuildingParcels a;
  ZoneBuildingParcels b;
  BuildZoneBuildingParcels(world, a);
  BuildZoneBuildingParcels(world, b);

  EXPECT_EQ(a.width, b.width);
  EXPECT_EQ(a.height, b.height);
  EXPECT_EQ(a.parcels.size(), b.parcels.size());
  EXPECT_EQ(a.tileToParcel.size(), b.tileToParcel.size());
  EXPECT_EQ(a.anchorToParcel.size(), b.anchorToParcel.size());

  // Mapping arrays should be byte-for-byte deterministic for identical input.
  for (std::size_t i = 0; i < a.tileToParcel.size(); ++i) {
    EXPECT_EQ(a.tileToParcel[i], b.tileToParcel[i]);
  }
  for (std::size_t i = 0; i < a.anchorToParcel.size(); ++i) {
    EXPECT_EQ(a.anchorToParcel[i], b.anchorToParcel[i]);
  }

  // All zone tiles should be assigned to exactly one parcel, and non-zone tiles
  // should remain unassigned.
  const int w = a.width;
  const int h = a.height;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const int idx = y * w + x;
      const bool isZone = IsZoneOverlay(t.overlay) && t.terrain != Terrain::Water;
      if (isZone) {
        EXPECT_NE(a.tileToParcel[idx], -1);
      } else {
        EXPECT_EQ(a.tileToParcel[idx], -1);
        EXPECT_EQ(a.anchorToParcel[idx], -1);
      }
    }
  }

  // Parcel invariants.
  for (int pi = 0; pi < static_cast<int>(a.parcels.size()); ++pi) {
    const ZoneBuildingParcel& p = a.parcels[static_cast<std::size_t>(pi)];
    EXPECT_TRUE(IsZoneOverlay(p.overlay));
    EXPECT_TRUE(p.w >= 1);
    EXPECT_TRUE(p.h >= 1);

    const int ax = p.x0 + p.w - 1;
    const int ay = p.y0 + p.h - 1;
    EXPECT_EQ(a.anchorToParcel[ay * w + ax], pi);

    int occSum = 0;
    int capSum = 0;
    for (int y = p.y0; y < p.y0 + p.h; ++y) {
      for (int x = p.x0; x < p.x0 + p.w; ++x) {
        const int idx = y * w + x;
        EXPECT_EQ(a.tileToParcel[idx], pi);

        const Tile& t = world.at(x, y);
        EXPECT_EQ(t.overlay, p.overlay);
        EXPECT_EQ(t.level, p.level);

        occSum += static_cast<int>(t.occupants);
        capSum += CapacityForOverlayLevel(t.overlay, static_cast<int>(t.level));
      }
    }

    EXPECT_EQ(occSum, p.occupants);
    EXPECT_EQ(capSum, p.capacity);
  }
}

static void TestBrushRasterShapes()
{
  using namespace isocity;

  auto expect4Connected = [&](const std::vector<Point>& pts) {
    for (std::size_t i = 1; i < pts.size(); ++i) {
      const int dx = std::abs(pts[i].x - pts[i - 1].x);
      const int dy = std::abs(pts[i].y - pts[i - 1].y);
      EXPECT_EQ(dx + dy, 1);
    }
  };

  // Horizontal line.
  {
    const std::vector<Point> pts = RasterLine(Point{0, 0}, Point{3, 0});
    EXPECT_EQ(pts.size(), static_cast<std::size_t>(4));
    EXPECT_EQ(pts.front().x, 0);
    EXPECT_EQ(pts.front().y, 0);
    EXPECT_EQ(pts.back().x, 3);
    EXPECT_EQ(pts.back().y, 0);
    expect4Connected(pts);
  }

  // Diagonal line.
  {
    const std::vector<Point> pts = RasterLine(Point{0, 0}, Point{4, 4});
    // 4-connected raster emits dx+dy+1 points.
    EXPECT_EQ(pts.size(), static_cast<std::size_t>(9));
    EXPECT_EQ(pts.front().x, 0);
    EXPECT_EQ(pts.front().y, 0);
    EXPECT_EQ(pts.back().x, 4);
    EXPECT_EQ(pts.back().y, 4);

    // Monotonic for this direction.
    for (std::size_t i = 1; i < pts.size(); ++i) {
      EXPECT_TRUE(pts[i].x >= pts[i - 1].x);
      EXPECT_TRUE(pts[i].y >= pts[i - 1].y);
    }
    expect4Connected(pts);
  }

  // Steep line: ensure no gaps.
  {
    const std::vector<Point> pts = RasterLine(Point{2, 1}, Point{3, 7});
    EXPECT_TRUE(!pts.empty());
    EXPECT_EQ(pts.front().x, 2);
    EXPECT_EQ(pts.front().y, 1);
    EXPECT_EQ(pts.back().x, 3);
    EXPECT_EQ(pts.back().y, 7);
    expect4Connected(pts);
  }

  // Filled rectangle: inclusive bounds.
  {
    const std::vector<Point> pts = RasterRectFilled(Point{1, 1}, Point{3, 2});
    // width=3 (1..3), height=2 (1..2)
    EXPECT_EQ(pts.size(), static_cast<std::size_t>(6));
  }

  // Outline rectangle: no duplicated corners.
  {
    const std::vector<Point> pts = RasterRectOutline(Point{1, 1}, Point{3, 2});
    // perimeter = 2*w + 2*h - 4 => 2*3 + 2*2 - 4 = 6
    EXPECT_EQ(pts.size(), static_cast<std::size_t>(6));
    expect4Connected(pts);
  }
}

static void TestFloodFillRegions()
{
  // A 5x5 grass world split by a vertical road barrier.
  World w(5, 5, 123);
  for (int y = 0; y < w.height(); ++y) {
    for (int x = 0; x < w.width(); ++x) {
      Tile& t = w.at(x, y);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
    }
  }
  for (int y = 0; y < w.height(); ++y) {
    w.setOverlay(Overlay::Road, 2, y);
    w.at(2, y).level = 1;
  }

  // Land block (default) should not cross roads.
  {
    const FloodFillResult r = FloodFillAuto(w, Point{0, 0}, /*includeRoadsInLandBlock=*/false);
    EXPECT_EQ(r.tiles.size(), static_cast<std::size_t>(10));
  }
  {
    const FloodFillResult r = FloodFillAuto(w, Point{4, 4}, /*includeRoadsInLandBlock=*/false);
    EXPECT_EQ(r.tiles.size(), static_cast<std::size_t>(10));
  }

  // Clicking a road returns the whole connected road component.
  {
    const FloodFillResult r = FloodFillAuto(w, Point{2, 0}, /*includeRoadsInLandBlock=*/false);
    EXPECT_EQ(r.tiles.size(), static_cast<std::size_t>(5));
  }

  // Land block with includeRoads==true crosses the barrier and includes roads.
  {
    const FloodFillResult r = FloodFillAuto(w, Point{0, 0}, /*includeRoadsInLandBlock=*/true);
    EXPECT_EQ(r.tiles.size(), static_cast<std::size_t>(25));
  }

  // Water-body fill excludes bridges (road overlay on water).
  {
    World ww(3, 3, 1);
    for (int y = 0; y < ww.height(); ++y) {
      for (int x = 0; x < ww.width(); ++x) {
        Tile& t = ww.at(x, y);
        t.terrain = Terrain::Grass;
        t.overlay = Overlay::None;
        t.level = 1;
        t.occupants = 0;
      }
    }

    // 2x2 water in the corner.
    ww.at(0, 0).terrain = Terrain::Water;
    ww.at(1, 0).terrain = Terrain::Water;
    ww.at(0, 1).terrain = Terrain::Water;
    ww.at(1, 1).terrain = Terrain::Water;

    // Bridge on (0,0).
    ww.setOverlay(Overlay::Road, 0, 0);
    ww.at(0, 0).level = 1;

    const FloodFillResult r = FloodFillAuto(ww, Point{1, 1}, /*includeRoadsInLandBlock=*/false);
    EXPECT_EQ(ChooseFloodFillMode(ww, Point{1, 1}), FloodFillMode::WaterBody);
    EXPECT_EQ(r.tiles.size(), static_cast<std::size_t>(3));
  }
}

static void TestAutoBuildDeterminism()
{
  using namespace isocity;

  const SimConfig sc{};
  AutoBuildConfig bc{};
  bc.zonesPerDay = 3;
  bc.roadsPerDay = 1;
  bc.parksPerDay = 1;
  bc.landValueRecalcDays = 1;
  bc.minMoneyReserve = 0;

  // Run twice from identical initial conditions and ensure the final hash matches.
  World w1(32, 32, 123);
  w1.stats().money = 2000;
  Simulator sim1(sc);
  sim1.refreshDerivedStats(w1);
  const AutoBuildReport r1 = RunAutoBuild(w1, sim1, bc, 60);
  const std::uint64_t h1 = HashWorld(w1, /*includeDerived=*/true);

  World w2(32, 32, 123);
  w2.stats().money = 2000;
  Simulator sim2(sc);
  sim2.refreshDerivedStats(w2);
  const AutoBuildReport r2 = RunAutoBuild(w2, sim2, bc, 60);
  const std::uint64_t h2 = HashWorld(w2, /*includeDerived=*/true);

  EXPECT_EQ(h1, h2);
  EXPECT_EQ(r1.daysSimulated, r2.daysSimulated);
  EXPECT_EQ(r1.zonesBuilt, r2.zonesBuilt);

  EXPECT_TRUE(w1.stats().roads > 0);
  EXPECT_TRUE(w1.stats().population > 0);
  EXPECT_TRUE(r1.zonesBuilt > 0);

  // Different seeds should very likely diverge.
  World w3(32, 32, 124);
  w3.stats().money = 2000;
  Simulator sim3(sc);
  sim3.refreshDerivedStats(w3);
  (void)RunAutoBuild(w3, sim3, bc, 60);
  const std::uint64_t h3 = HashWorld(w3, /*includeDerived=*/true);

  EXPECT_TRUE(h1 != h3);
}

static void TestScriptRunnerBasic()
{
  using namespace isocity;

  auto hexU64 = [](std::uint64_t v) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
    return oss.str();
  };

  // Capture script outputs so we can assert that command output matches the final world hash.
  std::vector<std::string> printed;

  ScriptRunner runner;
  runner.setOptions(ScriptRunOptions{.quiet = true, .includeDepthLimit = 8});
  runner.setCallbacks(ScriptCallbacks{
      .print = [&printed](const std::string& line) { printed.push_back(line); },
      .info = nullptr,
      .error = nullptr,
  });

  // Build a small but non-trivial city and tick a few days.
  const std::string script =
      "size 24x24\n"
      "seed 12345\n"
      // Make generation deterministic and easy to edit:
      // - No water/sand
      // - No pre-placed zones/parks
      "proc terrainscale 0\n"
      "proc waterlevel -999\n"
      "proc sandlevel -999\n"
      "proc zonechance 0\n"
      "proc parkchance 0\n"
      "proc hubs 4\n"
      "generate\n"
      // Clear any overlays carved by generation so the scripted edits are guaranteed to apply.
      "fill bulldoze 0 0 23 23\n"
      "money 5000\n"
      "road_line 1 1 10 1 1\n"
      "zone res 2 2 1\n"
      "tick 5\n"
      "hash\n";

  EXPECT_TRUE(runner.runText(script, "<unit_test>"));
  EXPECT_TRUE(runner.state().hasWorld);

  // Ensure derived stats are up-to-date for hashing.
  if (runner.state().dirtyDerived) {
    runner.state().sim.config() = runner.state().simCfg;
    runner.state().sim.refreshDerivedStats(runner.state().world);
    runner.state().dirtyDerived = false;
  }

  const std::uint64_t gotHash = HashWorld(runner.state().world, true);
  EXPECT_TRUE(!printed.empty());
  if (!printed.empty()) {
    EXPECT_EQ(printed.back(), hexU64(gotHash));
  }

  // Token expansion should resolve against the current script state.
  const std::string expanded = runner.expandPathTemplate("out_{seed}_{day}_{w}x{h}_{money}_{run}_{hash}", 7);
  EXPECT_TRUE(expanded.find("{seed}") == std::string::npos);
  EXPECT_TRUE(expanded.find("{day}") == std::string::npos);
  EXPECT_TRUE(expanded.find("{w}") == std::string::npos);
  EXPECT_TRUE(expanded.find("{h}") == std::string::npos);
  EXPECT_TRUE(expanded.find("{money}") == std::string::npos);
  EXPECT_TRUE(expanded.find("{run}") == std::string::npos);
  EXPECT_TRUE(expanded.find("{hash}") == std::string::npos);
}

static void TestScriptRunnerVarsAndExpr()
{
  using namespace isocity;

  std::vector<std::string> printed;

  ScriptRunner runner;
  runner.setOptions(ScriptRunOptions{.quiet = true});
  runner.setCallbacks(ScriptCallbacks{
    .print = [&](const std::string& line) { printed.push_back(line); },
    .info = [&](const std::string&) {},
    .error = [&](const std::string&) {},
  });

  const std::string script =
      "set out out_{day}_{run}_{seed}.ppm\n"
      "size 8x8\n"
      "seed 10+5\n"
      "proc terrainScale 0\n"
      "proc waterLevel -999\n"
      "proc sandLevel -999\n"
      "proc zoneChance 0\n"
      "proc parkChance 0\n"
      "generate\n"
      "echo {out}\n"
      "tick 3\n"
      "echo {out}\n"
      "money 1000+200\n";

  EXPECT_TRUE(runner.runText(script, "<unit_test_vars>"));
  EXPECT_TRUE(runner.state().hasWorld);

  if (runner.state().hasWorld) {
    EXPECT_EQ(runner.state().world.seed(), static_cast<std::uint64_t>(15));
    EXPECT_EQ(runner.state().world.stats().money, 1200);
  }

  EXPECT_TRUE(printed.size() >= 2);
  if (printed.size() >= 2) {
    EXPECT_EQ(printed[0], std::string("out_0_0_15.ppm"));
    EXPECT_EQ(printed[1], std::string("out_3_0_15.ppm"));
  }
}

static void TestScriptRunnerControlFlowRepeatIfElse()
{
  using namespace isocity;

  std::vector<std::string> printed;

  ScriptRunner runner;
  runner.setOptions(ScriptRunOptions{.quiet = true});
  runner.setCallbacks(ScriptCallbacks{
    .print = [&](const std::string& line) { printed.push_back(line); },
    .info = [&](const std::string&) {},
    .error = [&](const std::string&) {},
  });

  const std::string script =
      "set x 0\n"
      "repeat 5\n"
      "  add x 1\n"
      "end\n"
      "echo {x}\n"
      "set y 0\n"
      "repeat 3\n"
      "  if {y} == 1\n"
      "    add y 10\n"
      "  else\n"
      "    add y 1\n"
      "  end\n"
      "end\n"
      "echo {y}\n";

  EXPECT_TRUE(runner.runText(script, "<unit_test_ctrl_flow>"));
  EXPECT_EQ(printed.size(), static_cast<std::size_t>(2));
  if (printed.size() >= 2) {
    EXPECT_EQ(printed[0], std::string("5"));
    EXPECT_EQ(printed[1], std::string("12"));
  }
}

static void TestScriptRunnerControlFlowWhileBreakContinue()
{
  using namespace isocity;

  std::vector<std::string> printed;

  ScriptRunner runner;
  runner.setOptions(ScriptRunOptions{.quiet = true});
  runner.setCallbacks(ScriptCallbacks{
    .print = [&](const std::string& line) { printed.push_back(line); },
    .info = [&](const std::string&) {},
    .error = [&](const std::string&) {},
  });

  const std::string script =
      "set i 0\n"
      "set sum 0\n"
      "while {i} < 10\n"
      "  add i 1\n"
      "  if {i} % 2 == 0\n"
      "    continue\n"
      "  end\n"
      "  add sum {i}\n"
      "  if {sum} > 10\n"
      "    break\n"
      "  end\n"
      "end\n"
      "echo {i}\n"
      "echo {sum}\n";

  EXPECT_TRUE(runner.runText(script, "<unit_test_while_flow>"));
  EXPECT_EQ(printed.size(), static_cast<std::size_t>(2));
  if (printed.size() >= 2) {
    EXPECT_EQ(printed[0], std::string("7"));
    EXPECT_EQ(printed[1], std::string("16"));
  }
}

static void TestScriptRunnerExpectFail()
{
  using namespace isocity;

  ScriptRunner runner;
  runner.setOptions(ScriptRunOptions{.quiet = true});
  runner.setCallbacks(ScriptCallbacks{
    .print = [&](const std::string&) {},
    .info = [&](const std::string&) {},
    .error = [&](const std::string&) {},
  });

  const std::string script =
      "set x 5\n"
      "expect {x} == 6\n"
      "echo SHOULD_NOT_PRINT\n";

  EXPECT_TRUE(!runner.runText(script, "<unit_test_expect_fail>"));
  EXPECT_EQ(runner.lastErrorLine(), 2);
  EXPECT_TRUE(runner.lastError().find("expect failed") != std::string::npos);
}

static void TestSuiteManifestDiscoverAndRunScenario()
{
  using namespace isocity;
  namespace fs = std::filesystem;

  // Create an isolated temp folder for the test.
  const std::uintptr_t tag = reinterpret_cast<std::uintptr_t>(&g_failures);
  fs::path tmp = fs::temp_directory_path() / ("procisocity_suite_test_" + std::to_string(tag));
  std::error_code ec;
  fs::remove_all(tmp, ec);
  fs::create_directories(tmp, ec);
  EXPECT_TRUE(!ec);

  // --- Script scenario ---
  const fs::path scriptPath = tmp / "smoke.isocity";
  {
    std::ofstream f(scriptPath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(f));
    f << "size 8x8\n";
    f << "seed 123\n";
    f << "proc terrainScale 0\n";
    f << "proc waterLevel -999\n";
    f << "proc sandLevel -999\n";
    f << "proc zoneChance 0\n";
    f << "proc parkChance 0\n";
    f << "generate\n";
    f << "tick 2\n";
  }

  ScenarioCase scScript;
  scScript.path = scriptPath.string();
  scScript.kind = ScenarioKind::Script;

  ScenarioRunOptions opt;
  opt.quiet = true;
  opt.runIndex = 7;
  opt.scriptVars["hello"] = "world";

  ScenarioRunOutputs out;
  std::string err;
  EXPECT_TRUE(RunScenario(scScript, opt, out, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(out.world.width() == 8 && out.world.height() == 8);
  EXPECT_EQ(out.tickStats.size(), static_cast<std::size_t>(2));
  EXPECT_NE(out.finalHash, static_cast<std::uint64_t>(0));

  // --- Replay scenario with a deliberately wrong hash assert ---
  // Build a minimal replay file on disk.
  const fs::path replayPath = tmp / "bad_assert.isoreplay";
  {
    ProcGenConfig pc;
    pc.terrainScale = 0.0f;
    pc.waterLevel = -999.0f;
    pc.sandLevel = -999.0f;
    pc.zoneChance = 0.0f;
    pc.parkChance = 0.0f;

    SimConfig sc;
    World base = GenerateWorld(8, 8, 123, pc);

    std::vector<std::uint8_t> baseBytes;
    std::string saveErr;
    EXPECT_TRUE(SaveWorldBinaryToBytes(base, pc, sc, baseBytes, saveErr));

    Replay rp;
    rp.version = 2;
    rp.baseSave = std::move(baseBytes);
    ReplayEvent e;
    e.type = ReplayEventType::AssertHash;
    e.expectedHash = 0x1234567890ABCDEFull; // intentionally wrong
    e.includeStatsInHash = true;
    e.label = "wrong";
    rp.events.push_back(std::move(e));

    std::string repErr;
    EXPECT_TRUE(SaveReplayBinary(rp, replayPath.string(), repErr));
  }

  ScenarioCase scReplay;
  scReplay.path = replayPath.string();
  scReplay.kind = ScenarioKind::Replay;

  ScenarioRunOptions strict;
  strict.quiet = true;
  strict.strictReplayAsserts = true;
  strict.strictReplayPatches = true;

  ScenarioRunOutputs outStrict;
  std::string errStrict;
  EXPECT_TRUE(!RunScenario(scReplay, strict, outStrict, errStrict));
  EXPECT_TRUE(!errStrict.empty());

  ScenarioRunOptions lax = strict;
  lax.strictReplayAsserts = false;
  ScenarioRunOutputs outLax;
  std::string errLax;
  EXPECT_TRUE(RunScenario(scReplay, lax, outLax, errLax));
  EXPECT_TRUE(errLax.empty());
  EXPECT_NE(outLax.finalHash, static_cast<std::uint64_t>(0));

  // --- Manifest parsing ---
  const fs::path manifestPath = tmp / "suite.txt";
  {
    std::ofstream f(manifestPath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(f));
    f << "# test suite\n";
    f << "script \"" << scriptPath.string() << "\"\n";
    f << "replay \"" << replayPath.string() << "\"\n";
  }

  std::vector<ScenarioCase> manifestCases;
  std::string mErr;
  EXPECT_TRUE(LoadScenarioManifest(manifestPath.string(), manifestCases, mErr));
  EXPECT_TRUE(mErr.empty());
  EXPECT_EQ(manifestCases.size(), static_cast<std::size_t>(2));
  if (manifestCases.size() == 2) {
    EXPECT_EQ(manifestCases[0].kind, ScenarioKind::Script);
    EXPECT_EQ(manifestCases[1].kind, ScenarioKind::Replay);
  }

  // --- Discovery ---
  std::vector<ScenarioCase> discovered;
  std::string dErr;
  EXPECT_TRUE(DiscoverScenarios(tmp.string(), {".isocity", ".isoreplay"}, discovered, dErr));
  EXPECT_TRUE(dErr.empty());
  EXPECT_TRUE(discovered.size() >= 2);
}


void TestWorldTransformRotateMirrorCrop()
{
  const int srcW = 7;
  const int srcH = 5;

  World src(srcW, srcH, 123);
  src.stats().day = 42;
  src.stats().money = 1337;

  for (int y = 0; y < srcH; ++y) {
    for (int x = 0; x < srcW; ++x) {
      Tile& t = src.at(x, y);
      const int idx1d = y * srcW + x;

      t.terrain = Terrain::Grass;
      t.overlay = Overlay::Residential;
      t.level = 1;
      t.occupants = static_cast<std::uint16_t>(idx1d);
      t.district = static_cast<std::uint8_t>(idx1d % 8);
      t.variation = static_cast<std::uint8_t>((idx1d * 37) & 0xFF);
      t.height = 0.01f * static_cast<float>(idx1d);
    }
  }

  auto rotatedDims = [](int w, int h, int rotateDeg, int& outW, int& outH) {
    if (rotateDeg == 90 || rotateDeg == 270) {
      outW = h;
      outH = w;
    } else {
      outW = w;
      outH = h;
    }
  };

  auto mapManual = [&](const WorldTransformConfig& cfg, int xOut, int yOut, int& xs, int& ys) {
    int wRot = 0;
    int hRot = 0;
    rotatedDims(srcW, srcH, cfg.rotateDeg, wRot, hRot);

    // Undo crop.
    int xRot = xOut;
    int yRot = yOut;
    if (cfg.hasCrop) {
      xRot += cfg.cropX;
      yRot += cfg.cropY;
    }

    // Undo mirrors (mirrors are applied after rotation).
    if (cfg.mirrorX) xRot = wRot - 1 - xRot;
    if (cfg.mirrorY) yRot = hRot - 1 - yRot;

    // Map rotated-space -> source.
    switch (cfg.rotateDeg) {
    case 0:
      xs = xRot;
      ys = yRot;
      break;
    case 90:
      xs = yRot;
      ys = srcH - 1 - xRot;
      break;
    case 180:
      xs = srcW - 1 - xRot;
      ys = srcH - 1 - yRot;
      break;
    case 270:
      xs = srcW - 1 - yRot;
      ys = xRot;
      break;
    default:
      xs = ys = -1;
      break;
    }
  };

  auto runCase = [&](const WorldTransformConfig& cfg) {
    std::string err;
    World dst;
    EXPECT_TRUE(TransformWorld(src, dst, cfg, err));
    EXPECT_TRUE(err.empty());

    // Stats should be copied through the transform.
    EXPECT_EQ(dst.stats().day, 42);
    EXPECT_EQ(dst.stats().money, 1337);

    int wRot = 0;
    int hRot = 0;
    rotatedDims(srcW, srcH, cfg.rotateDeg, wRot, hRot);
    const int expectedW = cfg.hasCrop ? cfg.cropW : wRot;
    const int expectedH = cfg.hasCrop ? cfg.cropH : hRot;

    EXPECT_EQ(dst.width(), expectedW);
    EXPECT_EQ(dst.height(), expectedH);

    for (int y = 0; y < expectedH; ++y) {
      for (int x = 0; x < expectedW; ++x) {
        int xs = 0;
        int ys = 0;
        mapManual(cfg, x, y, xs, ys);
        EXPECT_TRUE(xs >= 0 && ys >= 0 && xs < srcW && ys < srcH);

        const int expectedIdx = ys * srcW + xs;
        const Tile& outT = dst.at(x, y);
        EXPECT_EQ(static_cast<int>(outT.occupants), expectedIdx);
        EXPECT_EQ(static_cast<int>(outT.district), expectedIdx % 8);
        EXPECT_EQ(static_cast<int>(outT.variation), (expectedIdx * 37) & 0xFF);
        EXPECT_NEAR(outT.height, 0.01f * static_cast<float>(expectedIdx), 1e-6f);
      }
    }
  };

  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 0;
    runCase(cfg);
  }
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 90;
    runCase(cfg);
  }
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 180;
    runCase(cfg);
  }
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 270;
    runCase(cfg);
  }
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 90;
    cfg.mirrorX = true;
    runCase(cfg);
  }
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 270;
    cfg.mirrorY = true;
    runCase(cfg);
  }
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 90;
    cfg.mirrorX = true;
    cfg.mirrorY = true;
    cfg.hasCrop = true;
    cfg.cropX = 1;
    cfg.cropY = 2;
    cfg.cropW = 3;
    cfg.cropH = 2;
    runCase(cfg);
  }

  // Invalid rotation.
  {
    WorldTransformConfig cfg;
    cfg.rotateDeg = 45;
    std::string err;
    World dst;
    EXPECT_TRUE(!TransformWorld(src, dst, cfg, err));
    EXPECT_TRUE(!err.empty());
  }
}

void TestMeshExportObjMtlBasic()
{
  using namespace isocity;

  World w(2, 2, 123);

  // (0,0) Grass
  {
    Tile& t = w.at(0, 0);
    t.terrain = Terrain::Grass;
    t.overlay = Overlay::None;
    t.height = 0.0f;
  }

  // (1,0) Road
  {
    Tile& t = w.at(1, 0);
    t.terrain = Terrain::Grass;
    t.overlay = Overlay::Road;
    t.height = 0.0f;
  }

  // (0,1) Water
  {
    Tile& t = w.at(0, 1);
    t.terrain = Terrain::Water;
    t.overlay = Overlay::None;
    t.height = -0.5f;
  }

  // (1,1) Residential zone (one building)
  {
    Tile& t = w.at(1, 1);
    t.terrain = Terrain::Grass;
    t.overlay = Overlay::Residential;
    t.level = 2;
    t.occupants = 15;
    t.variation = 0xA0; // affects building height slightly
    t.height = 0.25f;
  }

  // Export without cliffs/buildings to get deterministic counts.
  {
    MeshExportConfig cfg;
    cfg.mtlFileName = "test.mtl";
    cfg.tileSize = 1.0f;
    cfg.heightScale = 1.0f;
    cfg.overlayOffset = 0.1f;
    cfg.includeCliffs = false;
    cfg.includeBuildings = false;

    std::ostringstream obj;
    std::ostringstream mtl;
    MeshExportStats st;
    std::string err;
    EXPECT_TRUE(WriteWorldObjMtl(obj, mtl, w, cfg, &st, &err));
    EXPECT_TRUE(err.empty());

    EXPECT_EQ(st.vertices, 16ULL);
    EXPECT_EQ(st.triangles, 8ULL);

    const std::string objStr = obj.str();
    const std::string mtlStr = mtl.str();
    EXPECT_TRUE(objStr.find("mtllib test.mtl") != std::string::npos);
    EXPECT_TRUE(objStr.find("usemtl mat_road") != std::string::npos);
    EXPECT_TRUE(objStr.find("usemtl mat_water") != std::string::npos);
    EXPECT_TRUE(objStr.find("usemtl mat_res") != std::string::npos);
    EXPECT_TRUE(mtlStr.find("newmtl mat_grass") != std::string::npos);
    EXPECT_TRUE(mtlStr.find("newmtl mat_cliff") != std::string::npos); // always emitted
  }

  // Export with buildings enabled and verify that it adds geometry.
  {
    MeshExportConfig cfg;
    cfg.mtlFileName = "test.mtl";
    cfg.tileSize = 1.0f;
    cfg.heightScale = 1.0f;
    cfg.overlayOffset = 0.1f;
    cfg.includeCliffs = false;
    cfg.includeBuildings = true;

    std::ostringstream obj;
    std::ostringstream mtl;
    MeshExportStats st;
    std::string err;
    EXPECT_TRUE(WriteWorldObjMtl(obj, mtl, w, cfg, &st, &err));
    EXPECT_TRUE(err.empty());

    // 4 tiles => 16 vertices / 8 tris, plus one box building => 20 vertices / 10 tris.
    EXPECT_EQ(st.vertices, 36ULL);
    EXPECT_EQ(st.triangles, 18ULL);

    const std::string objStr = obj.str();
    EXPECT_TRUE(objStr.find("usemtl mat_building_res") != std::string::npos);
  }
}

void TestMeshExportObjMtlMergeTopSurfaces()
{
  using namespace isocity;

  World w(4, 1, 123);
  for (int x = 0; x < w.width(); ++x) {
    Tile& t = w.at(x, 0);
    t.terrain = Terrain::Grass;
    t.overlay = Overlay::None;
    t.height = 0.0f;
  }

  MeshExportConfig cfg;
  cfg.mtlFileName = "test.mtl";
  cfg.tileSize = 1.0f;
  cfg.heightScale = 1.0f;
  cfg.overlayOffset = 0.1f;
  cfg.includeCliffs = false;
  cfg.includeBuildings = false;
  cfg.mergeTopSurfaces = true;

  std::ostringstream obj;
  std::ostringstream mtl;
  MeshExportStats st;
  std::string err;
  EXPECT_TRUE(WriteWorldObjMtl(obj, mtl, w, cfg, &st, &err));
  EXPECT_TRUE(err.empty());

  // 4 grass tiles in a line should merge into a single quad.
  EXPECT_EQ(st.vertices, 4ULL);
  EXPECT_EQ(st.triangles, 2ULL);
  EXPECT_TRUE(obj.str().find("usemtl mat_grass") != std::string::npos);
}

void TestMeshExportObjMtlMergeBuildings()
{
  using namespace isocity;

  // Find a deterministic seed where the parcel builder merges two adjacent
  // Residential tiles into a single 2x1 parcel.
  std::uint64_t seedFound = 0;
  ZoneBuildingParcels parcels;
  for (std::uint64_t seed = 1; seed <= 4096; ++seed) {
    World w(2, 1, seed);
    for (int x = 0; x < 2; ++x) {
      Tile& t = w.at(x, 0);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::Residential;
      t.level = 2;
      t.occupants = 10;
      t.variation = 0xA0;
      t.height = 0.0f;
    }

    BuildZoneBuildingParcels(w, parcels);
    if (parcels.parcels.size() == 1 && parcels.parcels[0].w == 2 && parcels.parcels[0].h == 1) {
      seedFound = seed;
      break;
    }
  }

  EXPECT_TRUE(seedFound != 0);
  if (seedFound == 0) {
    return;
  }

  World w(2, 1, seedFound);
  for (int x = 0; x < 2; ++x) {
    Tile& t = w.at(x, 0);
    t.terrain = Terrain::Grass;
    t.overlay = Overlay::Residential;
    t.level = 2;
    t.occupants = 10;
    t.variation = 0xA0;
    t.height = 0.0f;
  }

  MeshExportConfig cfg;
  cfg.mtlFileName = "test.mtl";
  cfg.tileSize = 1.0f;
  cfg.heightScale = 1.0f;
  cfg.overlayOffset = 0.1f;
  cfg.includeTopSurfaces = false;
  cfg.includeCliffs = false;
  cfg.includeBuildings = true;

  // First, verify the legacy per-tile behavior.
  {
    cfg.mergeBuildings = false;

    std::ostringstream obj;
    std::ostringstream mtl;
    MeshExportStats st;
    std::string err;
    EXPECT_TRUE(WriteWorldObjMtl(obj, mtl, w, cfg, &st, &err));
    EXPECT_TRUE(err.empty());

    // 2 tiles => 2 independent box buildings.
    // Each building is 5 quads => 20 vertices / 10 tris.
    EXPECT_EQ(st.vertices, 40ULL);
    EXPECT_EQ(st.triangles, 20ULL);
  }

  // Now enable mergeBuildings and confirm the parcel collapses into one building.
  {
    cfg.mergeBuildings = true;

    std::ostringstream obj;
    std::ostringstream mtl;
    MeshExportStats st;
    std::string err;
    EXPECT_TRUE(WriteWorldObjMtl(obj, mtl, w, cfg, &st, &err));
    EXPECT_TRUE(err.empty());

    EXPECT_EQ(st.vertices, 20ULL);
    EXPECT_EQ(st.triangles, 10ULL);
    EXPECT_TRUE(obj.str().find("usemtl mat_building_res") != std::string::npos);
  }
}

void TestGltfExportBasic()
{
  using namespace isocity;

  // Use the same tiny world setup as the OBJ exporter test.
  World w(2, 2, 123);
  w.at(0, 0).terrain = Terrain::Grass;
  w.at(0, 0).overlay = Overlay::None;
  w.at(0, 0).height = 0.0f;

  w.at(1, 0).terrain = Terrain::Grass;
  w.at(1, 0).overlay = Overlay::Road;
  w.at(1, 0).height = 0.0f;

  w.at(0, 1).terrain = Terrain::Water;
  w.at(0, 1).overlay = Overlay::None;
  w.at(0, 1).height = -0.5f;

  w.at(1, 1).terrain = Terrain::Grass;
  w.at(1, 1).overlay = Overlay::Residential;
  w.at(1, 1).level = 2;
  w.at(1, 1).occupants = 15;
  w.at(1, 1).variation = 0xA0;
  w.at(1, 1).height = 0.25f;

  MeshExportConfig cfg;
  cfg.tileSize = 1.0f;
  cfg.heightScale = 1.0f;
  cfg.overlayOffset = 0.1f;
  cfg.includeCliffs = false;
  cfg.includeBuildings = false;

  static int tag = 0;
  ++tag;
  const fs::path tmpDir = fs::temp_directory_path();

  // --- .gltf + .bin ---
  {
    const fs::path gltfPath = tmpDir / ("isocity_gltf_test_" + std::to_string(tag) + ".gltf");
    const fs::path binPath = fs::path(gltfPath).replace_extension(".bin");

    MeshExportStats st;
    std::string err;
    EXPECT_TRUE(ExportWorldGltf(gltfPath.string(), w, cfg, &st, &err));
    EXPECT_TRUE(err.empty());
    EXPECT_TRUE(fs::exists(gltfPath));
    EXPECT_TRUE(fs::exists(binPath));

    EXPECT_EQ(st.vertices, 16ULL);
    EXPECT_EQ(st.triangles, 8ULL);

    // Basic sanity: JSON references the .bin and includes required POSITION min/max.
    std::ifstream f(gltfPath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(f));
    std::ostringstream oss;
    oss << f.rdbuf();
    const std::string json = oss.str();
    EXPECT_TRUE(json.find("\"version\":\"2.0\"") != std::string::npos);
    EXPECT_TRUE(json.find(binPath.filename().string()) != std::string::npos);
    EXPECT_TRUE(json.find("\"POSITION\":0") != std::string::npos);
    EXPECT_TRUE(json.find("\"min\":[") != std::string::npos);
    EXPECT_TRUE(json.find("\"max\":[") != std::string::npos);

    const std::uintmax_t binSize = fs::file_size(binPath);
    EXPECT_TRUE(json.find("\"byteLength\":" + std::to_string(binSize)) != std::string::npos);
  }

  // --- .glb ---
  {
    const fs::path glbPath = tmpDir / ("isocity_gltf_test_" + std::to_string(tag) + ".glb");

    MeshExportStats st;
    std::string err;
    EXPECT_TRUE(ExportWorldGlb(glbPath.string(), w, cfg, &st, &err));
    EXPECT_TRUE(err.empty());
    EXPECT_TRUE(fs::exists(glbPath));

    EXPECT_EQ(st.vertices, 16ULL);
    EXPECT_EQ(st.triangles, 8ULL);

    // Validate the basic GLB container structure:
    // - header magic/version/length
    // - JSON chunk marker
    std::ifstream f(glbPath, std::ios::binary);
    EXPECT_TRUE(static_cast<bool>(f));

    auto readU32LE = [&](std::uint32_t& out) -> bool {
      std::uint8_t b[4] = {0, 0, 0, 0};
      f.read(reinterpret_cast<char*>(b), 4);
      if (!f) return false;
      out = static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
            (static_cast<std::uint32_t>(b[2]) << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
      return true;
    };

    std::uint32_t magic = 0, version = 0, length = 0;
    EXPECT_TRUE(readU32LE(magic));
    EXPECT_TRUE(readU32LE(version));
    EXPECT_TRUE(readU32LE(length));

    EXPECT_EQ(magic, 0x46546C67u); // 'glTF'
    EXPECT_EQ(version, 2u);
    EXPECT_EQ(static_cast<std::uintmax_t>(length), fs::file_size(glbPath));

    std::uint32_t jsonChunkLen = 0, jsonChunkType = 0;
    EXPECT_TRUE(readU32LE(jsonChunkLen));
    EXPECT_TRUE(readU32LE(jsonChunkType));
    EXPECT_EQ(jsonChunkType, 0x4E4F534Au); // 'JSON'
    EXPECT_TRUE((jsonChunkLen % 4u) == 0u);
  }
}


static void TestCityBlocksBasic()
{
  using namespace isocity;

  World world(5, 5, 1);
  for (int y = 0; y < 5; ++y) {
    world.setRoad(2, y);
  }

  const CityBlocksResult r = BuildCityBlocks(world);

  EXPECT_TRUE(r.blocks.size() == 2);
  EXPECT_TRUE(r.blocks[0].id == 0);
  EXPECT_TRUE(r.blocks[1].id == 1);

  EXPECT_TRUE(r.blocks[0].area == 10);
  EXPECT_TRUE(r.blocks[1].area == 10);

  EXPECT_TRUE(r.blocks[0].roadEdges == 5);
  EXPECT_TRUE(r.blocks[1].roadEdges == 5);

  EXPECT_TRUE(r.blocks[0].outsideEdges == 9);
  EXPECT_TRUE(r.blocks[1].outsideEdges == 9);

  EXPECT_TRUE(r.blocks[0].roadAdjTiles == 5);
  EXPECT_TRUE(r.blocks[1].roadAdjTiles == 5);

  // Spot-check tileToBlock mapping on the first row.
  EXPECT_TRUE(r.tileToBlock[0] == 0);  // (0,0)
  EXPECT_TRUE(r.tileToBlock[1] == 0);  // (1,0)
  EXPECT_TRUE(r.tileToBlock[2] == -1); // (2,0) road
  EXPECT_TRUE(r.tileToBlock[3] == 1);  // (3,0)
  EXPECT_TRUE(r.tileToBlock[4] == 1);  // (4,0)

}

static double SignedAreaTest(const std::vector<isocity::IPoint>& ring)
{
  if (ring.size() < 4) return 0.0;
  long long acc = 0;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    acc += static_cast<long long>(ring[i].x) * static_cast<long long>(ring[i + 1].y) -
           static_cast<long long>(ring[i + 1].x) * static_cast<long long>(ring[i].y);
  }
  return static_cast<double>(acc) * 0.5;
}

static void TestVectorizeLabelGridWithHole()
{
  using namespace isocity;

  // 3x3 label grid with a 1-tile "lake" hole in the center.
  const int w = 3;
  const int h = 3;
  std::vector<int> labels(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 1);
  labels[static_cast<std::size_t>(1) * static_cast<std::size_t>(w) + 1] = -1;

  std::vector<LabeledGeometry> out;
  VectorizeStats st;
  std::string err;
  EXPECT_TRUE(VectorizeLabelGridToPolygons(labels, w, h, -1, out, &st, &err));
  EXPECT_TRUE(err.empty());

  EXPECT_EQ(out.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(out[0].label, 1);
  EXPECT_EQ(out[0].geom.polygons.size(), static_cast<std::size_t>(1));

  const VectorPolygon& poly = out[0].geom.polygons[0];
  EXPECT_TRUE(!poly.outer.empty());
  EXPECT_EQ(poly.outer.front(), poly.outer.back());
  EXPECT_EQ(poly.outer.size(), static_cast<std::size_t>(5)); // rectangle 0,0 .. 3,3

  EXPECT_EQ(poly.holes.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(poly.holes[0].front(), poly.holes[0].back());
  EXPECT_EQ(poly.holes[0].size(), static_cast<std::size_t>(5)); // square 1,1 .. 2,2

  // Validate areas (sign depends on orientation; magnitudes should match).
  EXPECT_NEAR(std::abs(SignedAreaTest(poly.outer)), 9.0, 1e-9);
  EXPECT_NEAR(std::abs(SignedAreaTest(poly.holes[0])), 1.0, 1e-9);
}


static void TestCityBlockGraphFrontageAndAdjacency()
{
  using namespace isocity;

  World world(5, 5, 1);

  // Split the map into 2 blocks with a vertical road column. Set road level to 2 so
  // we can validate the per-level aggregation.
  for (int y = 0; y < 5; ++y) {
    world.setRoad(2, y);
    world.at(2, y).level = 2;
  }

  const CityBlockGraphResult g = BuildCityBlockGraph(world);

  EXPECT_TRUE(g.blocks.blocks.size() == 2);
  EXPECT_TRUE(g.edges.size() == 1);
  EXPECT_TRUE(g.frontage.size() == 2);

  EXPECT_TRUE(g.edges[0].a == 0);
  EXPECT_TRUE(g.edges[0].b == 1);
  EXPECT_TRUE(g.edges[0].touchingRoadTiles == 5);
  EXPECT_TRUE(g.edges[0].touchingRoadTilesByLevel[2] == 5);

  EXPECT_TRUE(g.frontage[0].roadEdgesByLevel[2] == 5);
  EXPECT_TRUE(g.frontage[1].roadEdgesByLevel[2] == 5);

  EXPECT_TRUE(g.frontage[0].roadAdjTilesByLevel[2] == 5);
  EXPECT_TRUE(g.frontage[1].roadAdjTilesByLevel[2] == 5);
}


static void TestBlockDistrictingDisconnectedComponents()
{
  using namespace isocity;

  World world(5, 3, 1);

  // Create a water barrier column that splits the land into two disconnected blocks.
  for (int y = 0; y < 3; ++y) {
    world.at(2, y).terrain = Terrain::Water;
    world.at(2, y).overlay = Overlay::None;
  }

  BlockDistrictConfig cfg;
  cfg.districts = 2;
  cfg.fillRoadTiles = false;
  cfg.includeWater = false;

  const BlockDistrictResult r = AssignDistrictsByBlocks(world, cfg);

  EXPECT_TRUE(r.districtsUsed == 2);
  EXPECT_TRUE(r.seedBlockId.size() == 2);
  EXPECT_TRUE(r.blockToDistrict.size() == 2);

  // Left land should be district 0; right land should be district 1.
  for (int y = 0; y < 3; ++y) {
    EXPECT_TRUE(world.at(0, y).district == 0);
    EXPECT_TRUE(world.at(1, y).district == 0);
    EXPECT_TRUE(world.at(3, y).district == 1);
    EXPECT_TRUE(world.at(4, y).district == 1);

    // Water stays unchanged (default district 0).
    EXPECT_TRUE(world.at(2, y).district == 0);
  }
}

void TestRoadGraphTrafficAggregationSimpleLine()
{
  using namespace isocity;

  World world(5, 1, 1);
  for (int x = 0; x < 5; ++x) {
    world.setRoad(x, 0);
  }

  const RoadGraph g = BuildRoadGraph(world);
  EXPECT_TRUE(g.nodes.size() == 2);
  EXPECT_TRUE(g.edges.size() == 1);

  TrafficResult tr;
  tr.roadTraffic.assign(5u, 0);
  tr.roadTraffic[0] = 10;
  tr.roadTraffic[1] = 20;
  tr.roadTraffic[2] = 30;
  tr.roadTraffic[3] = 40;
  tr.roadTraffic[4] = 50;
  tr.maxTraffic = 50;

  RoadGraphTrafficConfig cfg;
  cfg.baseTileCapacity = 10;
  cfg.useRoadLevelCapacity = false;

  const RoadGraphTrafficResult agg = AggregateTrafficOnRoadGraph(world, g, tr, cfg);
  EXPECT_TRUE(agg.nodes.size() == 2);
  EXPECT_TRUE(agg.edges.size() == 1);

  const RoadGraphTrafficEdgeStats& es = agg.edges[0];
  EXPECT_TRUE(es.tileCount == 5);
  EXPECT_TRUE(es.interiorTileCount == 3);

  EXPECT_TRUE(es.sumTrafficInterior == 90u); // 20+30+40
  EXPECT_TRUE(es.maxTrafficInterior == 40);

  EXPECT_TRUE(es.sumCapacityInterior == 30u); // 3 tiles * 10
  EXPECT_TRUE(es.congestedTilesInterior == 3);
  EXPECT_TRUE(es.excessTrafficInterior == 60u); // (20-10)+(30-10)+(40-10)

  // maxUtilInterior should be 40/10 = 4.0
  EXPECT_TRUE(es.maxUtilInterior > 3.99 && es.maxUtilInterior < 4.01);

  // Node endpoints should carry endpoint traffic.
  EXPECT_TRUE(agg.nodes[0].traffic == 10);
  EXPECT_TRUE(agg.nodes[1].traffic == 50);
  EXPECT_TRUE(agg.nodes[0].capacity == 10);
  EXPECT_TRUE(agg.nodes[1].capacity == 10);
  EXPECT_TRUE(agg.nodes[0].util > 0.99 && agg.nodes[0].util < 1.01);
  EXPECT_TRUE(agg.nodes[1].util > 4.99 && agg.nodes[1].util < 5.01);
}


void TestRoadUpgradePlannerPrefersAvenueUpgradeWhenCostBenefitIsBetter()
{
  using namespace isocity;

  // Straight road line with heavy flow. Upgrading to level 2 yields a better
  // excess-reduction-per-cost ratio than upgrading directly to level 3.
  World world(6, 1, 1);
  for (int x = 0; x < 6; ++x) {
    world.setRoad(x, 0);
    world.at(x, 0).level = 1;
  }
  world.recomputeRoadMasks();

  const RoadGraph g = BuildRoadGraph(world);
  EXPECT_TRUE(g.nodes.size() == 2);
  EXPECT_TRUE(g.edges.size() == 1);

  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  std::vector<std::uint32_t> flow(n, 0);
  for (int x = 0; x < 6; ++x) flow[static_cast<std::size_t>(x)] = 60; // heavy traffic

  RoadUpgradePlannerConfig cfg;
  cfg.baseTileCapacity = 28;
  cfg.useRoadLevelCapacity = true;
  cfg.upgradeEndpoints = false; // interior-only
  cfg.maxTargetLevel = 3;
  cfg.minUtilConsider = 0.0;
  cfg.budget = -1;
  cfg.objective = RoadUpgradeObjective::Congestion;

  const RoadUpgradePlan plan = PlanRoadUpgrades(world, g, flow, cfg);

  EXPECT_TRUE(plan.edges.size() == 1);
  EXPECT_TRUE(plan.edges[0].targetLevel == 2);
  EXPECT_TRUE(plan.totalCost == 8); // 4 interior tiles * (level1->2 delta cost=2)
  EXPECT_TRUE(plan.totalExcessReduced == 88u);

  World upgraded = world;
  ApplyRoadUpgradePlan(upgraded, plan);

  // Interior tiles (x=1..4) upgraded to level 2; endpoints unchanged.
  EXPECT_TRUE(upgraded.at(0, 0).level == 1);
  EXPECT_TRUE(upgraded.at(5, 0).level == 1);
  for (int x = 1; x <= 4; ++x) {
    EXPECT_TRUE(upgraded.at(x, 0).level == 2);
  }
}




void TestRoadGraphResilienceFindsBridgesAndArticulations()
{
  using namespace isocity;

  // Build a simple T intersection:
  //
  // (0,0)---(1,0)---(2,0)
  //           |
  //         (1,1)
  //
  // The compressed road graph should have 4 nodes and 3 edges (a tree).
  // All 3 edges are bridges. The intersection at (1,0) is the only articulation node.
  World world(3, 2, 1);
  world.setRoad(0, 0);
  world.setRoad(1, 0);
  world.setRoad(2, 0);
  world.setRoad(1, 1);
  world.recomputeRoadMasks();

  const RoadGraph rg = BuildRoadGraph(world);
  EXPECT_TRUE(rg.nodes.size() == 4);
  EXPECT_TRUE(rg.edges.size() == 3);

  const RoadGraphResilienceResult res = ComputeRoadGraphResilience(rg);

  EXPECT_TRUE(res.bridgeEdges.size() == 3);
  EXPECT_TRUE(res.articulationNodes.size() == 1);

  // Nodes are discovered in scan order: (0,0)=0, (1,0)=1, (2,0)=2, (1,1)=3.
  EXPECT_TRUE(res.isArticulationNode.size() == rg.nodes.size());
  EXPECT_TRUE(res.isArticulationNode[1] == 1);

  EXPECT_TRUE(res.isBridgeEdge.size() == rg.edges.size());
  EXPECT_TRUE(res.isBridgeEdge[0] == 1);
  EXPECT_TRUE(res.isBridgeEdge[1] == 1);
  EXPECT_TRUE(res.isBridgeEdge[2] == 1);
}

void TestRoadGraphCentralityStarGraph()
{
  using namespace isocity;

  // Star graph with 4 leaves:
  //   1
  //   |
  // 2-0-3
  //   |
  //   4
  //
  // Undirected betweenness:
  //  - center node 0: C(4,2) = 6
  //  - leaf nodes: 0
  //  - each edge (0-i): paths (0,i) + (i,otherLeaves) => 1 + 3 = 4

  RoadGraph g;
  g.nodes.resize(5);
  g.edges.resize(4);

  g.nodes[0].pos = Point{0, 0};
  g.nodes[1].pos = Point{0, 1};
  g.nodes[2].pos = Point{-1, 0};
  g.nodes[3].pos = Point{1, 0};
  g.nodes[4].pos = Point{0, -1};

  auto addEdge = [&](int ei, int a, int b) {
    g.edges[static_cast<std::size_t>(ei)].a = a;
    g.edges[static_cast<std::size_t>(ei)].b = b;
    g.edges[static_cast<std::size_t>(ei)].length = 1;
    g.edges[static_cast<std::size_t>(ei)].tiles = {g.nodes[static_cast<std::size_t>(a)].pos,
                                                   g.nodes[static_cast<std::size_t>(b)].pos};
    g.nodes[static_cast<std::size_t>(a)].edges.push_back(ei);
    g.nodes[static_cast<std::size_t>(b)].edges.push_back(ei);
  };

  addEdge(0, 0, 1);
  addEdge(1, 0, 2);
  addEdge(2, 0, 3);
  addEdge(3, 0, 4);

  RoadGraphCentralityConfig cfg;
  cfg.weightMode = RoadGraphEdgeWeightMode::Steps;
  cfg.maxSources = 0;
  cfg.scaleSampleToFull = true;
  cfg.undirected = true;
  cfg.normalizeBetweenness = true;
  cfg.closenessComponentScale = true;

  const RoadGraphCentralityResult r = ComputeRoadGraphCentrality(g, cfg, nullptr);

  EXPECT_TRUE(r.nodeBetweenness.size() == 5);
  EXPECT_TRUE(r.edgeBetweenness.size() == 4);
  EXPECT_TRUE(r.nodeBetweenness[0] > 5.99 && r.nodeBetweenness[0] < 6.01);
  for (int i = 1; i < 5; ++i) {
    EXPECT_TRUE(std::abs(r.nodeBetweenness[static_cast<std::size_t>(i)]) < 1e-9);
  }

  for (int ei = 0; ei < 4; ++ei) {
    EXPECT_TRUE(r.edgeBetweenness[static_cast<std::size_t>(ei)] > 3.99 &&
                r.edgeBetweenness[static_cast<std::size_t>(ei)] < 4.01);
  }

  EXPECT_TRUE(r.nodeBetweennessNorm.size() == 5);
  EXPECT_TRUE(r.edgeBetweennessNorm.size() == 4);
  EXPECT_TRUE(r.nodeBetweennessNorm[0] > 0.999 && r.nodeBetweennessNorm[0] < 1.001);
  for (int i = 1; i < 5; ++i) {
    EXPECT_TRUE(std::abs(r.nodeBetweennessNorm[static_cast<std::size_t>(i)]) < 1e-9);
  }
  // For n=5 undirected, edge normalization factor is 2/(n(n-1)) = 0.1 => 4 * 0.1 = 0.4
  for (int ei = 0; ei < 4; ++ei) {
    EXPECT_TRUE(r.edgeBetweennessNorm[static_cast<std::size_t>(ei)] > 0.399 &&
                r.edgeBetweennessNorm[static_cast<std::size_t>(ei)] < 0.401);
  }

  // Closeness/harmonic are only computed when all sources are processed.
  EXPECT_TRUE(r.nodeCloseness.size() == 5);
  EXPECT_TRUE(r.nodeHarmonicCloseness.size() == 5);

  EXPECT_TRUE(r.nodeCloseness[0] > 0.999 && r.nodeCloseness[0] < 1.001);
  EXPECT_TRUE(r.nodeHarmonicCloseness[0] > 3.999 && r.nodeHarmonicCloseness[0] < 4.001);

  // Leaves: sumDist=7 => closeness=4/7 ~= 0.5714, harmonic=1 + 3*(1/2) = 2.5
  for (int i = 1; i < 5; ++i) {
    EXPECT_TRUE(r.nodeCloseness[static_cast<std::size_t>(i)] > 0.571 &&
                r.nodeCloseness[static_cast<std::size_t>(i)] < 0.572);
    EXPECT_TRUE(r.nodeHarmonicCloseness[static_cast<std::size_t>(i)] > 2.499 &&
                r.nodeHarmonicCloseness[static_cast<std::size_t>(i)] < 2.501);
  }
}

void TestRoadBuildPathBetweenSetsRespectsBlockedMoves()
{
  using namespace isocity;

  // A straight road line:
  // (0,1)-(1,1)-(2,1)-(3,1)-(4,1)
  // Block the move between (2,1) and (3,1); the planner should detour by building 2 new tiles.
  World world(5, 3, 1);
  for (int x = 0; x < 5; ++x) {
    world.setRoad(x, 1);
  }
  world.recomputeRoadMasks();

  const int w = world.width();
  const int fromIdx = 1 * w + 2;
  const int toIdx = 1 * w + 3;

  const std::uint64_t k1 = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(fromIdx)) << 32) |
                           static_cast<std::uint64_t>(static_cast<std::uint32_t>(toIdx));
  const std::uint64_t k2 = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(toIdx)) << 32) |
                           static_cast<std::uint64_t>(static_cast<std::uint32_t>(fromIdx));

  std::vector<std::uint64_t> blocked{ k1, k2 };
  std::sort(blocked.begin(), blocked.end());

  std::vector<Point> starts{ Point{0, 1} };
  std::vector<Point> goals{ Point{4, 1} };

  RoadBuildPathConfig cfg{};
  cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;

  std::vector<Point> path;
  int cost = 0;
  const bool ok = FindRoadBuildPathBetweenSets(world, starts, goals, path, &cost, cfg, &blocked);

  EXPECT_TRUE(ok);
  EXPECT_TRUE(!path.empty());
  EXPECT_TRUE(path.front().x == 0 && path.front().y == 1);
  EXPECT_TRUE(path.back().x == 4 && path.back().y == 1);

  // Minimal detour around a single blocked edge requires 2 new tiles.
  EXPECT_TRUE(cost == 2);

  // Ensure the blocked move never appears in the returned path.
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point a = path[i];
    const Point b = path[i + 1];
    EXPECT_TRUE(!(a.x == 2 && a.y == 1 && b.x == 3 && b.y == 1));
    EXPECT_TRUE(!(a.x == 3 && a.y == 1 && b.x == 2 && b.y == 1));
  }
}

void TestParkOptimizerSuggestsParksInUnderservedAreas()
{
  using namespace isocity;

  // A simple 7x5 world with a horizontal road spine and two residential clusters.
  // We place an existing park near the left cluster, then ask the optimizer to add 1 park.
  // The greedy score should pick the right cluster as it is farthest (and has demand).

  World world(7, 5, 123);
  world.stats().money = 100000;

  // Road spine at y=2 from x=0..6 (touches the map edge, so it satisfies the outside rule).
  for (int x = 0; x < world.width(); ++x) {
    EXPECT_EQ(world.applyRoad(x, 2, 1), ToolApplyResult::Applied);
  }

  // Existing park near the left.
  EXPECT_EQ(world.applyTool(Tool::Park, 1, 1), ToolApplyResult::Applied);

  // Residential demand near the right.
  EXPECT_EQ(world.applyTool(Tool::Residential, 4, 1), ToolApplyResult::Applied);
  EXPECT_EQ(world.applyTool(Tool::Residential, 5, 1), ToolApplyResult::Applied);
  world.at(4, 1).occupants = 100;
  world.at(5, 1).occupants = 100;

  ParkOptimizerConfig cfg;
  cfg.requireOutsideConnection = true;
  cfg.weightMode = IsochroneWeightMode::Steps;
  cfg.demandMode = ParkDemandMode::Occupants;
  cfg.includeResidential = true;
  cfg.includeCommercial = false;
  cfg.includeIndustrial = false;
  cfg.parksToAdd = 1;

  const ParkOptimizerResult r = SuggestParkPlacements(world, cfg);
  EXPECT_EQ(r.placements.size(), static_cast<std::size_t>(1));

  // Expect the suggested park to be placed adjacent to the right-side access road.
  // The candidate search order is N,E,S,W; since (5,1) is occupied by a zone and (6,2) is road,
  // the chosen buildable park tile adjacent to road (5,2) is (5,3).
  EXPECT_EQ(r.placements[0].accessRoad.x, 5);
  EXPECT_EQ(r.placements[0].accessRoad.y, 2);
  EXPECT_EQ(r.placements[0].parkTile.x, 5);
  EXPECT_EQ(r.placements[0].parkTile.y, 3);
}


void TestPolicyOptimizerExhaustivePrefersLowMaintenanceWhenNoRevenue()
{
  using namespace isocity;

  World world(16, 16, 123);

  // Ensure a simple all-land baseline so policy differences show up only in maintenance.
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      Tile& t = world.at(x, y);
      t.terrain = Terrain::Grass;
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
    }
  }

  // A small road + park footprint with no zones (no tax revenue).
  for (int x = 2; x <= 13; ++x) {
    Tile& t = world.at(x, 8);
    t.overlay = Overlay::Road;
    t.level = 1;
  }
  for (int y = 3; y <= 12; ++y) {
    Tile& t = world.at(8, y);
    t.overlay = Overlay::Road;
    t.level = 1;
  }
  world.at(10, 10).overlay = Overlay::Park;

  SimConfig baseCfg;
  Simulator sim(baseCfg);
  sim.refreshDerivedStats(world);

  PolicySearchSpace space;
  space.taxResMin = 0;
  space.taxResMax = 2;
  space.taxComMin = 0;
  space.taxComMax = 2;
  space.taxIndMin = 0;
  space.taxIndMax = 2;
  space.maintRoadMin = 0;
  space.maintRoadMax = 3;
  space.maintParkMin = 0;
  space.maintParkMax = 3;

  PolicyOptimizerConfig cfg;
  cfg.method = PolicyOptMethod::Exhaustive;
  cfg.evalDays = 1;
  cfg.maxExhaustiveCandidates = 100000;
  cfg.topK = 8;
  cfg.objective = {};
  cfg.objective.wMoneyDelta = 1.0;
  cfg.objective.minHappiness = 0.0;

  PolicyOptimizationResult r = OptimizePolicies(world, baseCfg, space, cfg);

  // With no tax revenue, the optimal policy is to minimize maintenance.
  EXPECT_EQ(r.best.policy.maintenanceRoad, 0);
  EXPECT_EQ(r.best.policy.maintenancePark, 0);

  // Taxes don't matter in this setup; tie-break picks the lexicographically smallest policy.
  EXPECT_EQ(r.best.policy.taxResidential, 0);
  EXPECT_EQ(r.best.policy.taxCommercial, 0);
  EXPECT_EQ(r.best.policy.taxIndustrial, 0);
}

int main()
{
  TestRoadAutoTilingMasks();
  TestZonePlacementAllowsInteriorTilesInConnectedComponent();
  TestEditHistoryUndoRedo();
  TestEditHistoryUndoRedoFixesRoadMasksLocally();
  TestEditHistoryUndoDoesNotRequireExactBaseState();
  TestToolsDoNotOverwriteOccupiedTiles();
  TestRoadHierarchyApplyRoadUpgradeCost();
  TestTrafficCongestionRespectsRoadClassCapacity();
  TestTrafficCongestionAwareSplitsParallelRoutes();
  TestSaveLoadRoundTrip();
  TestSaveLoadBytesRoundTrip();
  TestSLLZCompressionRoundTrip();
  TestSaveV8UsesCompressionForLargeDeltaPayload();
  TestSaveLoadDetectsCorruption();
  TestOutsideConnectionAffectsZoneAccess();
  TestZoneAccessMapAllowsInteriorTilesViaZoneConnectivity();
  TestIsochroneTileAccessCostsRespectInteriorZoneAccess();
  TestSimulatorStepInvariants();
  TestEmploymentCountsOnlyAccessibleJobs();
  TestRoadPathfindingToEdge();
  TestRoadToEdgeMask();
  TestRoadGraphPlusIntersection();
  TestRoadGraphCornerCreatesNode();
  TestRoadGraphExportMetricsAndJson();
  TestTrafficCommuteHeatmapSimple();
  TestTrafficPrefersHighSpeedRoadsWhenStepsTie();
  TestTrafficUnreachableAcrossDisconnectedEdgeComponents();
  TestGoodsIndustrySuppliesCommercial();
  TestGoodsFlowDebugOdCapturesLocalDeliveries();
  TestGoodsSplitsDemandAcrossMultipleProducers();
  TestGoodsImportsWhenNoIndustry();
  TestGoodsUnreachableDemandWhenNoImports();
  TestTrafficAndGoodsAcceptPrecomputedZoneAccessMap();

  TestLandValueParkAmenityBoostsNearby();
  TestLandValueWaterAmenityIncreasesNearCoast();
  TestLandValuePollutionPenalizesNearby();
  TestLandValueTrafficSpillUsesAdjacentRoadTraffic();

  TestResidentialDesirabilityPrefersHighLandValue();
  TestJobAssignmentPrefersHighLandValueCommercial();

  TestWorldHashDeterministicForSameSeed();
  TestProcGenBlockZoningCreatesInteriorAccessibleZones();
  TestProcGenErosionToggleAffectsHash();
  TestSimulationDeterministicHashAfterTicks();
  TestWorldDiffCounts();
  TestWorldPatchRoundTrip();
  TestWorldPatchRejectsMismatchedBaseHash();
  TestWorldPatchInvertAndCompose();
  TestConfigJsonIO();
  TestBlueprintCaptureApplyRotate();
  TestBlueprintDiffCropAndTransform();
  TestReplayRoundTrip();
  TestReplayHashAsserts();
  TestDistrictStatsCompute();
  TestAutoDistrictsSeparatesDisconnectedRoadComponents();
  TestAutoDistrictsFillAllTilesIsDeterministic();
  TestExportPpmLayers();
  TestExportIsoOverview();
  TestPpmReadWriteAndCompare();
  TestHeightmapApplyReclassifyAndBulldoze();
  TestHeightmapApplyResampleNearest();
  TestHydrologyFlowDirAndAccumulation();
  TestDepressionFillPriorityFloodSimpleBowl();
  TestDepressionFillRespectsLowEdgeOutlet();
  TestSeaFloodEdgeConnectivity();
  TestLabelComponentsAboveThresholdSimple();
  TestContoursMarchingSquaresDiamondLoop();
  TestMeshExportObjMtlBasic();
  TestMeshExportObjMtlMergeTopSurfaces();
  TestMeshExportObjMtlMergeBuildings();
  TestGltfExportBasic();
  TestWorldTransformRotateMirrorCrop();

  TestZoneBuildingParcelsDeterministic();
  TestBrushRasterShapes();
  TestFloodFillRegions();
  TestCityBlocksBasic();
  TestVectorizeLabelGridWithHole();
  TestCityBlockGraphFrontageAndAdjacency();
  TestBlockDistrictingDisconnectedComponents();
  TestRoadGraphTrafficAggregationSimpleLine();
  TestRoadUpgradePlannerPrefersAvenueUpgradeWhenCostBenefitIsBetter();
  TestPolicyOptimizerExhaustivePrefersLowMaintenanceWhenNoRevenue();
  TestRoadGraphResilienceFindsBridgesAndArticulations();
  TestRoadGraphCentralityStarGraph();
  TestRoadBuildPathBetweenSetsRespectsBlockedMoves();
  TestParkOptimizerSuggestsParksInUnderservedAreas();
  TestAutoBuildDeterminism();
  TestScriptRunnerBasic();
  TestScriptRunnerVarsAndExpr();
  TestScriptRunnerControlFlowRepeatIfElse();
  TestScriptRunnerControlFlowWhileBreakContinue();
  TestScriptRunnerExpectFail();
  TestSuiteManifestDiscoverAndRunScenario();


  TestRoadPathfindingAStar();
  TestLandPathfindingAStarAvoidsWater();
  TestRoadBuildPathPrefersExistingRoads();
  TestRoadBuildPathMinimizesTurnsWhenStepsTie();
  TestRoadBuildPathBetweenSetsAccountsForStartTileCost();

  TestBridgeRoadsCanBeBuiltOnWater();
  TestRoadBuildPathAvoidsBridgesWhenLandAlternativeExists();
  TestRoadBuildPathMoneyAvoidsExpensiveBridge();
  TestRoadBuildPathMoneyAvoidsExpensiveUpgrades();

  if (g_failures == 0) {
    std::cout << "proc_isocity_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_tests: FAILED (" << g_failures << ")\n";
  return 1;
}