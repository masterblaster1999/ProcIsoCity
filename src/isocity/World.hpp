#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

enum class Terrain : std::uint8_t {
  Water = 0,
  Sand = 1,
  Grass = 2,
};

enum class Overlay : std::uint8_t {
  None = 0,
  Road = 1,
  Residential = 2,
  Commercial = 3,
  Industrial = 4,
  Park = 5,
};

enum class Tool : std::uint8_t {
  Inspect = 0,
  Road,
  Residential,
  Commercial,
  Industrial,
  Park,
  Bulldoze,
};

// Return code for World::applyTool() so the game layer can provide feedback.
enum class ToolApplyResult : std::uint8_t {
  Applied = 0,
  Noop,
  OutOfBounds,
  BlockedWater,
  BlockedNoRoad,
  BlockedOccupied,
  InsufficientFunds,
};

struct Tile {
  Terrain terrain = Terrain::Grass;
  Overlay overlay = Overlay::None;

  float height = 0.0f; // 0..1-ish (procedural)

  // Stable per-tile random bits for shading/variation.
  // When overlay == Road, the low 4 bits are also used to store a road-connection mask
  // so roads can auto-connect visually.
  std::uint8_t variation = 0;

  // Used for zoning overlays.
  std::uint8_t level = 1;      // 1..3
  std::uint16_t occupants = 0; // residents for Residential, workers for job zones
};

struct Stats {
  int day = 0;

  int population = 0;
  int housingCapacity = 0;

  int jobsCapacity = 0;

  // Jobs that are currently reachable via roads (and optionally connected to the map edge).
  // This is a derived stat recomputed by the simulator; it is not persisted in the save file.
  int jobsCapacityAccessible = 0;

  int employed = 0;

  float happiness = 0.5f; // 0..1

  int money = 250;

  int roads = 0;
  int parks = 0;

  // --- Derived traffic/commute stats (recomputed by the simulator; not persisted in saves) ---
  int commuters = 0;            // modeled commuting workers (usually ~= employed)
  int commutersUnreachable = 0; // commuters that couldn't reach any job access point

  float avgCommute = 0.0f;      // road steps (edges)
  float p95Commute = 0.0f;      // road steps (edges)
  float trafficCongestion = 0.0f; // 0..1 (excess traffic ratio)

  int congestedRoadTiles = 0;
  int maxRoadTraffic = 0; // max commuters on any road tile (for heatmap scaling)

  // --- Derived goods/logistics stats (recomputed by the simulator; not persisted in saves) ---
  int goodsProduced = 0;
  int goodsDemand = 0;
  int goodsDelivered = 0;
  int goodsImported = 0;
  int goodsExported = 0;
  int goodsUnreachableDemand = 0;
  float goodsSatisfaction = 1.0f; // delivered/demand, clamped to [0,1]
  int maxRoadGoodsTraffic = 0;
};

const char* ToString(Terrain t);
const char* ToString(Overlay o);
const char* ToString(Tool t);

class World {
public:
  World() = default;
  World(int w, int h, std::uint64_t seed);

  int width() const { return m_w; }
  int height() const { return m_h; }
  std::uint64_t seed() const { return m_seed; }

  bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < m_w && y < m_h; }

  Tile& at(int x, int y) { return m_tiles[static_cast<std::size_t>(y) * m_w + x]; }
  const Tile& at(int x, int y) const { return m_tiles[static_cast<std::size_t>(y) * m_w + x]; }

  const Stats& stats() const { return m_stats; }
  Stats& stats() { return m_stats; }

  bool isBuildable(int x, int y) const;  // Terrain != Water
  bool isEmptyLand(int x, int y) const;  // buildable and overlay == None
  bool hasAdjacentRoad(int x, int y) const;

  // Player actions / tools.
  ToolApplyResult applyTool(Tool tool, int x, int y);

  // Utility editing operations.
  void bulldoze(int x, int y);
  void setRoad(int x, int y);
  void setOverlay(Overlay overlay, int x, int y);

  // Recompute road connection masks for all road tiles.
  // Useful after loading older saves or bulk edits.
  void recomputeRoadMasks();

private:
  std::uint8_t computeRoadMask(int x, int y) const;
  void applyRoadMask(int x, int y);
  void updateRoadMasksAround(int x, int y);

  int m_w = 0;
  int m_h = 0;
  std::uint64_t m_seed = 0;
  std::vector<Tile> m_tiles;
  Stats m_stats;
};

} // namespace isocity
