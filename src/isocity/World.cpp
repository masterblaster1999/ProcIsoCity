#include "isocity/World.hpp"

#include "isocity/Road.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace isocity {

const char* ToString(Terrain t)
{
  switch (t) {
  case Terrain::Water: return "Water";
  case Terrain::Sand: return "Sand";
  case Terrain::Grass: return "Grass";
  default: return "UnknownTerrain";
  }
}

const char* ToString(Overlay o)
{
  switch (o) {
  case Overlay::None: return "None";
  case Overlay::Road: return "Road";
  case Overlay::Residential: return "Residential";
  case Overlay::Commercial: return "Commercial";
  case Overlay::Industrial: return "Industrial";
  case Overlay::Park: return "Park";
  case Overlay::School: return "School";
  case Overlay::Hospital: return "Hospital";
  case Overlay::PoliceStation: return "PoliceStation";
  case Overlay::FireStation: return "FireStation";
  default: return "UnknownOverlay";
  }
}

const char* ToString(Tool t)
{
  switch (t) {
  case Tool::Inspect: return "Inspect";
  case Tool::Road: return "Road";
  case Tool::Residential: return "Residential";
  case Tool::Commercial: return "Commercial";
  case Tool::Industrial: return "Industrial";
  case Tool::Park: return "Park";
  case Tool::School: return "School";
  case Tool::Hospital: return "Hospital";
  case Tool::PoliceStation: return "PoliceStation";
  case Tool::FireStation: return "FireStation";
  case Tool::Bulldoze: return "Bulldoze";
  case Tool::RaiseTerrain: return "Raise";
  case Tool::LowerTerrain: return "Lower";
  case Tool::SmoothTerrain: return "Smooth";
  case Tool::District: return "District";
  default: return "UnknownTool";
  }
}

World::World(int w, int h, std::uint64_t seed)
    : m_w(w)
    , m_h(h)
    , m_seed(seed)
    , m_tiles(static_cast<std::size_t>(w) * h)
{
  // Default funds; you can tune this.
  m_stats.money = 250;
  m_stats.happiness = 0.5f;
}

bool World::isBuildable(int x, int y) const
{
  if (!inBounds(x, y)) return false;
  return at(x, y).terrain != Terrain::Water;
}

bool World::isEmptyLand(int x, int y) const
{
  if (!inBounds(x, y)) return false;
  const Tile& t = at(x, y);
  return (t.terrain != Terrain::Water) && (t.overlay == Overlay::None);
}

bool World::hasAdjacentRoad(int x, int y) const
{
  if (!inBounds(x, y)) return false;

  constexpr int dirs[4][2] = {
      {1, 0},
      {-1, 0},
      {0, 1},
      {0, -1},
  };

  for (const auto& d : dirs) {
    const int nx = x + d[0];
    const int ny = y + d[1];
    if (!inBounds(nx, ny)) continue;
    if (at(nx, ny).overlay == Overlay::Road) return true;
  }

  return false;
}

bool World::wouldZoneHaveRoadAccess(Overlay zoneOverlay, int x, int y) const
{
  if (!inBounds(x, y)) return false;

  // Only meaningful for zoning overlays.
  if (zoneOverlay != Overlay::Residential && zoneOverlay != Overlay::Commercial && zoneOverlay != Overlay::Industrial) {
    return false;
  }

  const Tile& t0 = at(x, y);

  // Zoning is not allowed on water (bridges are roads only).
  if (t0.terrain == Terrain::Water) return false;

  // Callers generally ensure this, but keep the helper safe.
  if (t0.overlay != Overlay::None && t0.overlay != zoneOverlay) return false;

  // Fast path: direct road adjacency.
  if (hasAdjacentRoad(x, y)) return true;

  // If we don't touch any existing same-zone tile, then this would create a new
  // disconnected component (no road access).
  constexpr int dirs[4][2] = {
      {1, 0},
      {-1, 0},
      {0, 1},
      {0, -1},
  };

  bool hasNeighborZone = false;
  for (const auto& d : dirs) {
    const int nx = x + d[0];
    const int ny = y + d[1];
    if (!inBounds(nx, ny)) continue;
    const Tile& nt = at(nx, ny);
    if (nt.terrain == Terrain::Water) continue;
    if (nt.overlay == zoneOverlay) {
      hasNeighborZone = true;
      break;
    }
  }
  if (!hasNeighborZone) return false;

  const int w = m_w;
  const int h = m_h;
  if (w <= 0 || h <= 0) return false;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<std::uint8_t> visited(n, 0);
  std::vector<int> queue;
  queue.reserve(256);

  auto idxOf = [&](int tx, int ty) -> int { return ty * w + tx; };

  const int start = idxOf(x, y);
  if (start < 0 || static_cast<std::size_t>(start) >= visited.size()) return false;
  visited[static_cast<std::size_t>(start)] = 1;
  queue.push_back(start);

  std::size_t qHead = 0;
  while (qHead < queue.size()) {
    const int cur = queue[qHead++];
    const int cx = cur % w;
    const int cy = cur / w;

    // If any tile in the connected zone component touches a road, the component
    // is considered accessible.
    if (hasAdjacentRoad(cx, cy)) return true;

    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (!inBounds(nx, ny)) continue;
      const int ni = idxOf(nx, ny);
      if (ni < 0) continue;
      const std::size_t ui = static_cast<std::size_t>(ni);
      if (ui >= visited.size()) continue;
      if (visited[ui]) continue;

      // Traverse existing zone tiles of the same overlay. The start tile is
      // treated as zoned (even if it's currently empty) and is already queued.
      const Tile& nt = at(nx, ny);
      if (nt.terrain == Terrain::Water) continue;
      if (nt.overlay != zoneOverlay) continue;

      visited[ui] = 1;
      queue.push_back(ni);
    }
  }

  return false;
}

std::uint8_t World::computeRoadMask(int x, int y) const
{
  if (!inBounds(x, y)) return 0;
  if (at(x, y).overlay != Overlay::Road) return 0;

  // Bit layout (tile-space):
  //  bit0: (x, y-1)  (screen up-right)
  //  bit1: (x+1, y)  (screen down-right)
  //  bit2: (x, y+1)  (screen down-left)
  //  bit3: (x-1, y)  (screen up-left)
  std::uint8_t m = 0;
  if (inBounds(x, y - 1) && at(x, y - 1).overlay == Overlay::Road) m |= 1u << 0;
  if (inBounds(x + 1, y) && at(x + 1, y).overlay == Overlay::Road) m |= 1u << 1;
  if (inBounds(x, y + 1) && at(x, y + 1).overlay == Overlay::Road) m |= 1u << 2;
  if (inBounds(x - 1, y) && at(x - 1, y).overlay == Overlay::Road) m |= 1u << 3;
  return m;
}

void World::applyRoadMask(int x, int y)
{
  if (!inBounds(x, y)) return;
  Tile& t = at(x, y);
  if (t.overlay != Overlay::Road) return;

  const std::uint8_t mask = static_cast<std::uint8_t>(computeRoadMask(x, y) & 0x0Fu);

  // Preserve upper bits of the per-tile variation so lighting still has some stable randomness.
  t.variation = static_cast<std::uint8_t>((t.variation & 0xF0u) | mask);
}

void World::updateRoadMasksAround(int x, int y)
{
  applyRoadMask(x, y);
  applyRoadMask(x, y - 1);
  applyRoadMask(x + 1, y);
  applyRoadMask(x, y + 1);
  applyRoadMask(x - 1, y);
}

void World::recomputeRoadMasks()
{
  for (int y = 0; y < m_h; ++y) {
    for (int x = 0; x < m_w; ++x) {
      if (at(x, y).overlay != Overlay::Road) continue;
      applyRoadMask(x, y);
    }
  }
}

void World::setOverlay(Overlay overlay, int x, int y)
{
  if (!inBounds(x, y)) return;
  Tile& t = at(x, y);
  // Allow clearing overlays even if a tile is water (useful for terraforming or future tooling).
  // Most content can't be placed on water, except roads (bridges).
  if (t.terrain == Terrain::Water && overlay != Overlay::None && overlay != Overlay::Road) return;

  const Overlay before = t.overlay;
  t.overlay = overlay;

  // Reset/initialize per-overlay state.
  if (overlay == Overlay::Residential || overlay == Overlay::Commercial || overlay == Overlay::Industrial) {
    // Zone tiles use level 1..3 to represent density / building level.
    // Keep the narrowing explicit to avoid MSVC conversion warnings on uint8_t.
    const int level = std::clamp(static_cast<int>(t.level), 1, 3);
    t.level = static_cast<std::uint8_t>(level);
    t.occupants = 0;
  } else if (overlay == Overlay::Road) {
    // Road tiles use level 1..3 for Street/Avenue/Highway.
    // Preserve the level when the overlay remains Road; otherwise default to Street.
    if (before != Overlay::Road) t.level = 1;
    t.level = static_cast<std::uint8_t>(ClampRoadLevel(static_cast<int>(t.level)));
    t.occupants = 0;
  } else if (overlay == Overlay::School || overlay == Overlay::Hospital || overlay == Overlay::PoliceStation ||
             overlay == Overlay::FireStation) {
    // Service facilities use level 1..3 to represent capacity/upgrade level.
    if (before != overlay) t.level = 1;
    const int level = std::clamp(static_cast<int>(t.level), 1, 3);
    t.level = static_cast<std::uint8_t>(level);
    t.occupants = 0;
  } else {
    // Non-zones and parks don't currently use the level field.
    t.level = 1;
    t.occupants = 0;
  }

  // If this edit adds/removes a road, update the local auto-tiling masks.
  if (before == Overlay::Road || overlay == Overlay::Road) {
    updateRoadMasksAround(x, y);
  }
}

void World::bulldoze(int x, int y) { setOverlay(Overlay::None, x, y); }

void World::setRoad(int x, int y)
{
  if (!inBounds(x, y)) return;
  if (at(x, y).overlay == Overlay::Road) return;
  setOverlay(Overlay::Road, x, y);
}

ToolApplyResult World::applyRoad(int x, int y, int targetLevel)
{
  if (!inBounds(x, y)) return ToolApplyResult::OutOfBounds;
  Tile& t = at(x, y);

  targetLevel = ClampRoadLevel(targetLevel);

  auto spend = [&](int cost) -> bool {
    if (cost <= 0) return true;
    if (m_stats.money < cost) return false;
    m_stats.money -= cost;
    return true;
  };

  if (t.overlay == Overlay::Road) {
    const int cur = ClampRoadLevel(static_cast<int>(t.level));
    if (cur >= targetLevel) return ToolApplyResult::Noop;

    const bool isBridge = (t.terrain == Terrain::Water);
    const int costTarget = isBridge ? RoadBridgeBuildCostForLevel(targetLevel) : RoadBuildCostForLevel(targetLevel);
    const int costCur = isBridge ? RoadBridgeBuildCostForLevel(cur) : RoadBuildCostForLevel(cur);
    const int cost = costTarget - costCur;
    if (!spend(cost)) return ToolApplyResult::InsufficientFunds;

    t.level = static_cast<std::uint8_t>(targetLevel);
    return ToolApplyResult::Applied;
  }

  if (t.overlay != Overlay::None) return ToolApplyResult::BlockedOccupied;

  const bool isBridge = (t.terrain == Terrain::Water);
  const int cost = isBridge ? RoadBridgeBuildCostForLevel(targetLevel) : RoadBuildCostForLevel(targetLevel);
  if (!spend(cost)) return ToolApplyResult::InsufficientFunds;

  setRoad(x, y);
  t.level = static_cast<std::uint8_t>(targetLevel);
  return ToolApplyResult::Applied;
}

ToolApplyResult World::applyDistrict(int x, int y, int districtId)
{
  if (!inBounds(x, y)) return ToolApplyResult::OutOfBounds;

  districtId = std::clamp(districtId, 0, kDistrictCount - 1);
  Tile& t = at(x, y);
  const std::uint8_t d = static_cast<std::uint8_t>(districtId);
  if (t.district == d) return ToolApplyResult::Noop;
  t.district = d;
  return ToolApplyResult::Applied;
}

ToolApplyResult World::applyTool(Tool tool, int x, int y)
{
  if (!inBounds(x, y)) return ToolApplyResult::OutOfBounds;
  Tile& t = at(x, y);

  if (tool == Tool::Inspect) return ToolApplyResult::Noop;

  // Most tools can't build on water.
  // Roads are allowed on water (bridges), and bulldozing should be able to remove bridge tiles.
  if (t.terrain == Terrain::Water && tool != Tool::Road && tool != Tool::Bulldoze)
    return ToolApplyResult::BlockedWater;

  // Tool costs (tiny placeholder economy).
  auto spend = [&](int cost) -> bool {
    if (cost <= 0) return true;
    if (m_stats.money < cost) return false;
    m_stats.money -= cost;
    return true;
  };

  switch (tool) {
  case Tool::Road: {
    return applyRoad(x, y, 1);
  } break;

    case Tool::School:
  case Tool::Hospital:
  case Tool::PoliceStation:
  case Tool::FireStation: {
    Overlay svc = Overlay::School;
    if (tool == Tool::Hospital) svc = Overlay::Hospital;
    if (tool == Tool::PoliceStation) svc = Overlay::PoliceStation;
    if (tool == Tool::FireStation) svc = Overlay::FireStation;

    // Service facilities require direct road adjacency.
    if (!hasAdjacentRoad(x, y)) return ToolApplyResult::BlockedNoRoad;

    auto buildCostForLevel = [&](int lvl) -> int {
      // A small but noticeable capital cost curve.
      lvl = std::clamp(lvl, 1, 3);
      return 25 + 20 * (lvl - 1);
    };

    if (t.overlay == svc) {
      const int cur = std::clamp<int>(static_cast<int>(t.level), 1, 3);
      if (cur >= 3) return ToolApplyResult::Noop;
      const int target = cur + 1;
      const int cost = buildCostForLevel(target) - buildCostForLevel(cur);
      if (!spend(cost)) return ToolApplyResult::InsufficientFunds;
      t.level = static_cast<std::uint8_t>(target);
      return ToolApplyResult::Applied;
    }

    // Don't overwrite other content; bulldoze first.
    if (t.overlay != Overlay::None) return ToolApplyResult::BlockedOccupied;

    const int cost = buildCostForLevel(1);
    if (!spend(cost)) return ToolApplyResult::InsufficientFunds;

    setOverlay(svc, x, y);
    // Stable variation bits so facilities don't all look identical.
    const std::uint32_t seed32 = static_cast<std::uint32_t>(m_seed & 0xFFFFFFFFu) ^
                            (static_cast<std::uint32_t>(static_cast<std::uint8_t>(tool)) * 0x9E3779B9u);
    t.variation = static_cast<std::uint8_t>(HashCoords32(x, y, seed32) & 0xFFu);
    return ToolApplyResult::Applied;
  } break;

case Tool::Park: {
    if (t.overlay == Overlay::Park) return ToolApplyResult::Noop;
    // Parks also shouldn't replace existing content; bulldoze first.
    if (t.overlay != Overlay::None) return ToolApplyResult::BlockedOccupied;
    if (!spend(3)) return ToolApplyResult::InsufficientFunds;
    setOverlay(Overlay::Park, x, y);
    return ToolApplyResult::Applied;
  } break;

  case Tool::Residential:
  case Tool::Commercial:
  case Tool::Industrial: {
    Overlay zone = Overlay::Residential;
    if (tool == Tool::Commercial) zone = Overlay::Commercial;
    if (tool == Tool::Industrial) zone = Overlay::Industrial;

    // Don't overwrite other overlays; bulldoze first.
    if (t.overlay != Overlay::None && t.overlay != zone) return ToolApplyResult::BlockedOccupied;

    // Zones require road access, but we allow multi-tile zoning blocks:
    // a tile is placeable if it is adjacent to a road OR it connects to an
    // existing same-zone component that touches a road.
    if (!wouldZoneHaveRoadAccess(zone, x, y)) return ToolApplyResult::BlockedNoRoad;

    if (t.overlay == zone) {
      // Upgrade with repeated placement.
      if (t.level >= 3) return ToolApplyResult::Noop;
      if (!spend(5)) return ToolApplyResult::InsufficientFunds;
      t.level++;
      return ToolApplyResult::Applied;
    }

    if (!spend(5)) return ToolApplyResult::InsufficientFunds;
    setOverlay(zone, x, y);
    return ToolApplyResult::Applied;
  } break;

  case Tool::Bulldoze: {
    if (t.overlay == Overlay::None) return ToolApplyResult::Noop;
    bulldoze(x, y);
    return ToolApplyResult::Applied;
  } break;

  default: break;
  }

  return ToolApplyResult::Noop;
}

} // namespace isocity
