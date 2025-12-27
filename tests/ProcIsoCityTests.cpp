#include "isocity/EditHistory.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cstdint>
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

  // Save to a temp-ish location (use cwd if temp isn't available).
  fs::path savePath = "isocity_test_save.bin";
  try {
    savePath = fs::temp_directory_path() / "isocity_test_save.bin";
  } catch (...) {
    // keep relative path
  }

  std::string err;
  EXPECT_TRUE(SaveWorldBinary(w, cfg, savePath.string(), err));

  World loaded;
  ProcGenConfig loadedCfg{};
  err.clear();
  EXPECT_TRUE(LoadWorldBinary(loaded, loadedCfg, savePath.string(), err));

  // Basic world identity checks.
  EXPECT_EQ(loaded.width(), w.width());
  EXPECT_EQ(loaded.height(), w.height());
  EXPECT_EQ(loaded.seed(), w.seed());

  // Check our edits survived.
  EXPECT_EQ(loaded.at(x, y).overlay, Overlay::Road);
  EXPECT_EQ(loaded.at(x + 1, y).overlay, Overlay::Residential);

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

  std::string err;
  EXPECT_TRUE(SaveWorldBinary(w, cfg, savePath.string(), err));

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

} // namespace

int main()
{
  TestRoadAutoTilingMasks();
  TestEditHistoryUndoRedo();
  TestToolsDoNotOverwriteOccupiedTiles();
  TestSaveLoadRoundTrip();
  TestSaveLoadDetectsCorruption();
  TestOutsideConnectionAffectsZoneAccess();
  TestSimulatorStepInvariants();

  if (g_failures == 0) {
    std::cout << "proc_isocity_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_tests: FAILED (" << g_failures << ")\n";
  return 1;
}
