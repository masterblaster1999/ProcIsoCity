#include "isocity/FloodFill.hpp"

#include <algorithm>
#include <cstddef>

namespace isocity {

FloodFillMode ChooseFloodFillMode(const World& world, Point start)
{
  if (!world.inBounds(start.x, start.y)) return FloodFillMode::LandBlock;

  const Tile& seed = world.at(start.x, start.y);
  if (seed.overlay == Overlay::Road) return FloodFillMode::RoadComponent;
  if (seed.terrain == Terrain::Water && seed.overlay != Overlay::Road) return FloodFillMode::WaterBody;
  return FloodFillMode::LandBlock;
}

FloodFillResult FloodFillRegion(const World& world, Point start, FloodFillMode mode, bool includeRoadsInLandBlock)
{
  FloodFillResult out;
  out.w = world.width();
  out.h = world.height();

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const int n = w * h;
  out.mask.assign(static_cast<std::size_t>(n), 0);

  auto canFill = [&](int x, int y) -> bool {
    if (!world.inBounds(x, y)) return false;
    const Tile& t = world.at(x, y);

    switch (mode) {
    case FloodFillMode::RoadComponent:
      return t.overlay == Overlay::Road;

    case FloodFillMode::WaterBody:
      // Exclude bridges (road overlay) so water bodies remain stable.
      return (t.terrain == Terrain::Water) && (t.overlay != Overlay::Road);

    case FloodFillMode::LandBlock:
    default:
      // Water is always a hard boundary.
      if (t.terrain == Terrain::Water) return false;
      // Roads may be treated as boundaries unless explicitly included.
      if (!includeRoadsInLandBlock && t.overlay == Overlay::Road) return false;
      return true;
    }
  };

  if (!canFill(start.x, start.y)) return out;

  std::vector<Point> stack;
  stack.reserve(static_cast<std::size_t>(std::min(n, 4096)));

  auto markPush = [&](int x, int y) {
    const int idx = y * w + x;
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (out.mask[uidx]) return;
    out.mask[uidx] = 1;
    stack.push_back(Point{x, y});
  };

  markPush(start.x, start.y);

  while (!stack.empty()) {
    const Point p = stack.back();
    stack.pop_back();

    out.tiles.push_back(p);

    const int x = p.x;
    const int y = p.y;

    // Deterministic neighbor order.
    if (x > 0 && canFill(x - 1, y)) markPush(x - 1, y);
    if (x + 1 < w && canFill(x + 1, y)) markPush(x + 1, y);
    if (y > 0 && canFill(x, y - 1)) markPush(x, y - 1);
    if (y + 1 < h && canFill(x, y + 1)) markPush(x, y + 1);
  }

  return out;
}

FloodFillResult FloodFillAuto(const World& world, Point start, bool includeRoadsInLandBlock)
{
  const FloodFillMode mode = ChooseFloodFillMode(world, start);
  return FloodFillRegion(world, start, mode, includeRoadsInLandBlock);
}

} // namespace isocity
