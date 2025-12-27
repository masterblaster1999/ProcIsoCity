#include "isocity/World.hpp"

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
  if (t.terrain == Terrain::Water) return;

  const Overlay before = t.overlay;
  t.overlay = overlay;

  // Reset/initialize zone state.
  if (overlay == Overlay::Residential || overlay == Overlay::Commercial || overlay == Overlay::Industrial) {
    t.level = std::clamp<int>(t.level, 1, 3);
    if (t.level < 1) t.level = 1;
    if (t.level > 3) t.level = 3;
    if (t.occupants > 0) t.occupants = 0;
  } else {
    t.level = 1;
    t.occupants = 0;
  }

  // If this edit adds/removes a road, update the local auto-tiling masks.
  if (before == Overlay::Road || overlay == Overlay::Road) {
    updateRoadMasksAround(x, y);
  }
}

void World::bulldoze(int x, int y) { setOverlay(Overlay::None, x, y); }

void World::setRoad(int x, int y) { setOverlay(Overlay::Road, x, y); }

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
    if (t.overlay == Overlay::Road) return ToolApplyResult::Noop;
    // Don't overwrite other overlays (zones/parks). Use the bulldozer first.
    if (t.overlay != Overlay::None) return ToolApplyResult::BlockedOccupied;
    if (!spend(1)) return ToolApplyResult::InsufficientFunds;
    setRoad(x, y);
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
