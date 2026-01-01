#include "isocity/CityBlocks.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace isocity {

namespace {

inline bool IsBlockTile(const Tile& t)
{
  if (t.terrain == Terrain::Water) return false;
  if (t.overlay == Overlay::Road) return false;
  return true;
}

inline std::size_t Idx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

} // namespace

CityBlocksResult BuildCityBlocks(const World& world)
{
  CityBlocksResult out{};
  out.w = world.width();
  out.h = world.height();

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  out.tileToBlock.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);

  std::vector<int> stack;
  stack.reserve(1024);

  int nextId = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t seedIdx = Idx(x, y, w);
      if (out.tileToBlock[seedIdx] >= 0) continue;

      const Tile& seed = world.at(x, y);
      if (!IsBlockTile(seed)) continue;

      CityBlock b{};
      b.id = nextId++;
      b.minX = b.maxX = x;
      b.minY = b.maxY = y;

      stack.clear();
      stack.push_back(static_cast<int>(seedIdx));
      out.tileToBlock[seedIdx] = b.id;

      while (!stack.empty()) {
        const int curIdx = stack.back();
        stack.pop_back();

        const int cx = curIdx % w;
        const int cy = curIdx / w;

        const Tile& t = world.at(cx, cy);

        b.area++;
        b.minX = std::min(b.minX, cx);
        b.minY = std::min(b.minY, cy);
        b.maxX = std::max(b.maxX, cx);
        b.maxY = std::max(b.maxY, cy);

        // Overlay composition (roads are excluded).
        switch (t.overlay) {
        case Overlay::Park: b.parks++; break;
        case Overlay::Residential: b.residential++; break;
        case Overlay::Commercial: b.commercial++; break;
        case Overlay::Industrial: b.industrial++; break;
        default: b.other++; break;
        }

        bool hasRoadNeighbor = false;

        // Deterministic neighbor order: L, R, U, D.
        const int nx[4] = {cx - 1, cx + 1, cx, cx};
        const int ny[4] = {cy, cy, cy - 1, cy + 1};

        for (int k = 0; k < 4; ++k) {
          const int x2 = nx[k];
          const int y2 = ny[k];

          if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) {
            b.outsideEdges++;
            continue;
          }

          const Tile& n = world.at(x2, y2);

          if (n.terrain == Terrain::Water) {
            b.waterEdges++;
            continue;
          }

          if (n.overlay == Overlay::Road) {
            b.roadEdges++;
            hasRoadNeighbor = true;
            continue;
          }

          // Land, non-road.
          const std::size_t nidx = Idx(x2, y2, w);
          if (out.tileToBlock[nidx] >= 0) continue;

          if (!IsBlockTile(n)) continue; // defensive; should be redundant with checks above

          out.tileToBlock[nidx] = b.id;
          stack.push_back(static_cast<int>(nidx));
        }

        if (hasRoadNeighbor) b.roadAdjTiles++;
      }

      out.blocks.push_back(b);
    }
  }

  return out;
}

} // namespace isocity
