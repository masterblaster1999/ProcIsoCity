#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

namespace isocity {

namespace {

inline bool IsZoneOverlay(Overlay o)
{
  return o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial;
}

inline bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t expect = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == expect;
}

} // namespace

ZoneAccessMap BuildZoneAccessMap(const World& world, const std::vector<std::uint8_t>* roadToEdgeMask)
{
  ZoneAccessMap out;

  const int w = world.width();
  const int h = world.height();
  out.w = w;
  out.h = h;

  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.roadIdx.assign(n, -1);

  const bool maskOk = MaskUsable(roadToEdgeMask, w, h);

  std::vector<std::uint8_t> visited(n, std::uint8_t{0});
  std::deque<int> q;
  std::vector<int> comp;
  comp.reserve(n / 8);
  std::vector<std::pair<int, int>> sources;
  sources.reserve(64);

  constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}}; // N,E,S,W

  auto idxOf = [&](int x, int y) -> int { return y * w + x; };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int start = idxOf(x, y);
      const std::size_t us = static_cast<std::size_t>(start);
      if (us >= visited.size()) continue;
      if (visited[us]) continue;

      const Tile& t0 = world.at(x, y);
      if (!IsZoneOverlay(t0.overlay)) continue;
      if (t0.terrain == Terrain::Water) continue;

      // --- Gather a connected component of the same zone overlay (4-neighborhood) ---
      const Overlay overlay = t0.overlay;
      visited[us] = 1;
      q.clear();
      comp.clear();
      q.push_back(start);

      while (!q.empty()) {
        const int cur = q.front();
        q.pop_front();
        comp.push_back(cur);

        const int cx = cur % w;
        const int cy = cur / w;

        for (const auto& d : dirs) {
          const int nx = cx + d[0];
          const int ny = cy + d[1];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const int ni = idxOf(nx, ny);
          const std::size_t ui = static_cast<std::size_t>(ni);
          if (ui >= visited.size()) continue;
          if (visited[ui]) continue;

          const Tile& nt = world.at(nx, ny);
          if (nt.terrain == Terrain::Water) continue;
          if (nt.overlay != overlay) continue;

          visited[ui] = 1;
          q.push_back(ni);
        }
      }

      // --- Identify boundary sources (zone tiles that touch a usable road) ---
      sources.clear();
      for (int zi : comp) {
        const int zx = zi % w;
        const int zy = zi / w;

        int bestRoad = -1;
        for (const auto& d : dirs) {
          const int rx = zx + d[0];
          const int ry = zy + d[1];
          if (rx < 0 || ry < 0 || rx >= w || ry >= h) continue;
          if (world.at(rx, ry).overlay != Overlay::Road) continue;

          const int ridx = idxOf(rx, ry);
          const std::size_t ur = static_cast<std::size_t>(ridx);
          if (ur >= out.roadIdx.size()) continue;
          if (maskOk && (*roadToEdgeMask)[ur] == 0) continue;

          if (bestRoad < 0 || ridx < bestRoad) bestRoad = ridx;
        }

        if (bestRoad >= 0) {
          const std::size_t uz = static_cast<std::size_t>(zi);
          out.roadIdx[uz] = bestRoad;
          sources.emplace_back(zi, bestRoad);
        }
      }

      if (sources.empty()) {
        // No road-adjacent tiles in this zone block => no access.
        continue;
      }

      // Deterministic queue order for multi-source propagation.
      std::sort(sources.begin(), sources.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        return a.second < b.second;
      });

      // --- Propagate access roads inward to the rest of the component ---
      q.clear();
      for (const auto& s : sources) {
        q.push_back(s.first);
      }

      while (!q.empty()) {
        const int cur = q.front();
        q.pop_front();
        const int road = out.roadIdx[static_cast<std::size_t>(cur)];
        if (road < 0) continue;

        const int cx = cur % w;
        const int cy = cur / w;

        for (const auto& d : dirs) {
          const int nx = cx + d[0];
          const int ny = cy + d[1];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const int ni = idxOf(nx, ny);
          const std::size_t ui = static_cast<std::size_t>(ni);
          if (ui >= out.roadIdx.size()) continue;
          if (out.roadIdx[ui] >= 0) continue; // already assigned by another (nearer) source

          const Tile& nt = world.at(nx, ny);
          if (nt.terrain == Terrain::Water) continue;
          if (nt.overlay != overlay) continue;

          out.roadIdx[ui] = road;
          q.push_back(ni);
        }
      }
    }
  }

  return out;
}

} // namespace isocity
