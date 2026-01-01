#include "isocity/CityBlockGraph.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace isocity {

namespace {

inline std::size_t Idx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline int ClampRoadLevel(int level)
{
  if (level < 1) return 1;
  if (level > 3) return 3;
  return level;
}

inline std::uint64_t PairKey(int a, int b)
{
  const std::uint32_t lo = static_cast<std::uint32_t>(std::min(a, b));
  const std::uint32_t hi = static_cast<std::uint32_t>(std::max(a, b));
  return (static_cast<std::uint64_t>(lo) << 32) | static_cast<std::uint64_t>(hi);
}

} // namespace

CityBlockGraphResult BuildCityBlockGraph(const World& world, const CityBlocksResult* precomputedBlocks)
{
  CityBlockGraphResult out;

  if (precomputedBlocks) {
    out.blocks = *precomputedBlocks;
  } else {
    out.blocks = BuildCityBlocks(world);
  }

  const int w = out.blocks.w;
  const int h = out.blocks.h;
  if (w <= 0 || h <= 0) return out;

  const int blockCount = static_cast<int>(out.blocks.blocks.size());
  out.frontage.assign(static_cast<std::size_t>(blockCount), CityBlockFrontage{});

  // --- Frontage metrics ---
  // Scan all block tiles and examine adjacent road tiles.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = Idx(x, y, w);
      const int bid = (idx < out.blocks.tileToBlock.size()) ? out.blocks.tileToBlock[idx] : -1;
      if (bid < 0 || bid >= blockCount) continue;

      bool adjLevel[4] = {false, false, false, false};

      const int nx[4] = {x - 1, x + 1, x, x};
      const int ny[4] = {y, y, y - 1, y + 1};

      for (int k = 0; k < 4; ++k) {
        const int x2 = nx[k];
        const int y2 = ny[k];
        if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;
        const Tile& n = world.at(x2, y2);
        if (n.overlay != Overlay::Road) continue;

        const int lvl = ClampRoadLevel(static_cast<int>(n.level));
        out.frontage[static_cast<std::size_t>(bid)].roadEdgesByLevel[static_cast<std::size_t>(lvl)]++;
        adjLevel[lvl] = true;
      }

      for (int lvl = 1; lvl <= 3; ++lvl) {
        if (adjLevel[lvl]) {
          out.frontage[static_cast<std::size_t>(bid)].roadAdjTilesByLevel[static_cast<std::size_t>(lvl)]++;
        }
      }
    }
  }

  // --- Adjacency edges ---
  // Use an ordered map for deterministic edge ordering.
  std::map<std::uint64_t, CityBlockAdjacency> edgeMap;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Road) continue;

      // Collect unique adjacent blocks.
      int adjBlocks[4] = {-1, -1, -1, -1};
      int adjCount = 0;

      const int nx[4] = {x - 1, x + 1, x, x};
      const int ny[4] = {y, y, y - 1, y + 1};

      for (int k = 0; k < 4; ++k) {
        const int x2 = nx[k];
        const int y2 = ny[k];
        if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) continue;
        const std::size_t nidx = Idx(x2, y2, w);
        const int bid = (nidx < out.blocks.tileToBlock.size()) ? out.blocks.tileToBlock[nidx] : -1;
        if (bid < 0 || bid >= blockCount) continue;

        bool exists = false;
        for (int j = 0; j < adjCount; ++j) {
          if (adjBlocks[j] == bid) {
            exists = true;
            break;
          }
        }
        if (!exists && adjCount < 4) {
          adjBlocks[adjCount++] = bid;
        }
      }

      if (adjCount < 2) continue;

      // Sort for deterministic pair enumeration.
      // adjCount is tiny (<=4). Use a small fixed sort to keep compilers happy
      // about bounds on the fixed-size array.
      for (int a = 0; a < adjCount; ++a) {
        for (int b = a + 1; b < adjCount; ++b) {
          if (adjBlocks[b] < adjBlocks[a]) std::swap(adjBlocks[a], adjBlocks[b]);
        }
      }

      const int roadLvl = ClampRoadLevel(static_cast<int>(t.level));

      for (int i = 0; i < adjCount; ++i) {
        for (int j = i + 1; j < adjCount; ++j) {
          const int a = adjBlocks[i];
          const int b = adjBlocks[j];
          if (a == b) continue;
          const std::uint64_t key = PairKey(a, b);
          CityBlockAdjacency& e = edgeMap[key];
          if (e.a < 0) {
            e.a = std::min(a, b);
            e.b = std::max(a, b);
          }
          e.touchingRoadTiles++;
          e.touchingRoadTilesByLevel[static_cast<std::size_t>(roadLvl)]++;
        }
      }
    }
  }

  out.edges.reserve(edgeMap.size());
  for (const auto& kv : edgeMap) {
    out.edges.push_back(kv.second);
  }

  out.blockToEdges.assign(static_cast<std::size_t>(blockCount), {});
  for (int ei = 0; ei < static_cast<int>(out.edges.size()); ++ei) {
    const CityBlockAdjacency& e = out.edges[static_cast<std::size_t>(ei)];
    if (e.a >= 0 && e.a < blockCount) out.blockToEdges[static_cast<std::size_t>(e.a)].push_back(ei);
    if (e.b >= 0 && e.b < blockCount) out.blockToEdges[static_cast<std::size_t>(e.b)].push_back(ei);
  }

  for (auto& v : out.blockToEdges) {
    std::sort(v.begin(), v.end());
  }

  return out;
}

} // namespace isocity
