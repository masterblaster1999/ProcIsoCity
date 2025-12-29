#include "isocity/World.hpp"

#include "isocity/Road.hpp"

#include <algorithm>

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
  // Allow clearing overlays even if a tile is water (useful for terraforming or future tooling),
  // but never allow placing new content on water.
  if (t.terrain == Terrain::Water && overlay != Overlay::None) return;

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

  if (t.terrain == Terrain::Water) return ToolApplyResult::BlockedWater;

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

    const int cost = RoadBuildCostForLevel(targetLevel) - RoadBuildCostForLevel(cur);
    if (!spend(cost)) return ToolApplyResult::InsufficientFunds;

    t.level = static_cast<std::uint8_t>(targetLevel);
    return ToolApplyResult::Applied;
  }

  if (t.overlay != Overlay::None) return ToolApplyResult::BlockedOccupied;

  const int cost = RoadBuildCostForLevel(targetLevel);
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

  // Can't build on water.
  if (t.terrain == Terrain::Water) return ToolApplyResult::BlockedWater;

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
    // Zones require road access.
    if (!hasAdjacentRoad(x, y)) return ToolApplyResult::BlockedNoRoad;

    Overlay zone = Overlay::Residential;
    if (tool == Tool::Commercial) zone = Overlay::Commercial;
    if (tool == Tool::Industrial) zone = Overlay::Industrial;

    if (t.overlay == zone) {
      // Upgrade with repeated placement.
      if (t.level >= 3) return ToolApplyResult::Noop;
      if (!spend(5)) return ToolApplyResult::InsufficientFunds;
      t.level++;
      return ToolApplyResult::Applied;
    }

    if (t.overlay != Overlay::None) return ToolApplyResult::BlockedOccupied; // Don't overwrite other overlays.

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
