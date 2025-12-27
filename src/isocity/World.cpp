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

void World::bulldoze(int x, int y)
{
  if (!inBounds(x, y)) return;
  Tile& t = at(x, y);
  if (t.terrain == Terrain::Water) return;

  t.overlay = Overlay::None;
  t.level = 1;
  t.occupants = 0;
}

void World::setRoad(int x, int y)
{
  if (!inBounds(x, y)) return;
  Tile& t = at(x, y);
  if (t.terrain == Terrain::Water) return;

  t.overlay = Overlay::Road;
  t.level = 1;
  t.occupants = 0;
}

void World::setOverlay(Overlay overlay, int x, int y)
{
  if (!inBounds(x, y)) return;
  Tile& t = at(x, y);
  if (t.terrain == Terrain::Water) return;

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
}

void World::applyTool(Tool tool, int x, int y)
{
  if (!inBounds(x, y)) return;
  Tile& t = at(x, y);

  if (tool == Tool::Inspect) return;

  // Can't build on water.
  if (t.terrain == Terrain::Water) return;

  // Tool costs (tiny placeholder economy).
  auto spend = [&](int cost) -> bool {
    if (cost <= 0) return true;
    if (m_stats.money < cost) return false;
    m_stats.money -= cost;
    return true;
  };

  switch (tool) {
  case Tool::Road: {
    if (t.overlay == Overlay::Road) return;
    if (!spend(1)) return;
    setRoad(x, y);
  } break;

  case Tool::Park: {
    if (t.overlay == Overlay::Park) return;
    if (!spend(3)) return;
    t.overlay = Overlay::Park;
    t.level = 1;
    t.occupants = 0;
  } break;

  case Tool::Residential:
  case Tool::Commercial:
  case Tool::Industrial: {
    // Zones require road access.
    if (!hasAdjacentRoad(x, y)) return;

    Overlay zone = Overlay::Residential;
    if (tool == Tool::Commercial) zone = Overlay::Commercial;
    if (tool == Tool::Industrial) zone = Overlay::Industrial;

    if (t.overlay == zone) {
      // Upgrade with repeated placement.
      if (t.level < 3 && spend(5)) t.level++;
      return;
    }

    if (t.overlay != Overlay::None) return; // Don't overwrite other overlays in this minimal template.

    if (!spend(5)) return;
    t.overlay = zone;
    t.level = 1;
    t.occupants = 0;
  } break;

  case Tool::Bulldoze: {
    bulldoze(x, y);
  } break;

  default: break;
  }
}

} // namespace isocity
