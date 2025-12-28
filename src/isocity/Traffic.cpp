#include "isocity/Traffic.hpp"

#include "isocity/Pathfinding.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline std::uint32_t Hash32(std::uint32_t x)
{
  // Small deterministic integer hash.
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

inline float HashToUnitFloat(std::uint32_t h)
{
  // Convert to [0,1) using 24 bits.
  return static_cast<float>(h & 0x00FFFFFFu) / 16777216.0f;
}

inline std::uint16_t SatAddU16(std::uint16_t cur, std::uint32_t add)
{
  std::uint32_t v = static_cast<std::uint32_t>(cur) + add;
  if (v > 65535u) v = 65535u;
  return static_cast<std::uint16_t>(v);
}

bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t expect = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == expect;
}


} // namespace

TrafficResult ComputeCommuteTraffic(const World& world, const TrafficConfig& cfg, float employedShare,
                                   const std::vector<std::uint8_t>* precomputedRoadToEdge)
{
  TrafficResult r;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return r;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  r.roadTraffic.assign(n, 0);

  employedShare = std::clamp(employedShare, 0.0f, 1.0f);
  if (employedShare <= 0.0f) {
    // Nobody commutes.
    return r;
  }

  // Outside connection mask.
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;

  if (cfg.requireOutsideConnection) {
    if (MaskUsable(precomputedRoadToEdge, w, h)) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      ComputeRoadsConnectedToEdge(world, roadToEdgeLocal);
      roadToEdge = &roadToEdgeLocal;
    }
  }

  auto zoneHasAccess = [&](int zx, int zy) -> bool {
    if (!world.hasAdjacentRoad(zx, zy)) return false;
    if (!cfg.requireOutsideConnection) return true;
    return HasAdjacentRoadConnectedToEdge(world, *roadToEdge, zx, zy);
  };

  // --- Collect job access points (BFS sources) ---
  std::vector<std::uint8_t> isSource(n, 0);
  std::vector<int> sources;
  sources.reserve(n / 16);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);

      const bool isCommercial = (t.overlay == Overlay::Commercial);
      const bool isIndustrial = (t.overlay == Overlay::Industrial);
      if (!isCommercial && !isIndustrial) continue;

      if (isCommercial && !cfg.includeCommercialJobs) continue;
      if (isIndustrial && !cfg.includeIndustrialJobs) continue;

      if (!zoneHasAccess(x, y)) continue;

      // Each adjacent road tile is an access point.
      constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
      for (const auto& d : dirs) {
        const int rx = x + d[0];
        const int ry = y + d[1];
        if (!world.inBounds(rx, ry)) continue;
        if (world.at(rx, ry).overlay != Overlay::Road) continue;

        const std::size_t idx = static_cast<std::size_t>(ry) * static_cast<std::size_t>(w) +
                                static_cast<std::size_t>(rx);
        if (cfg.requireOutsideConnection) {
          if (idx >= roadToEdge->size() || (*roadToEdge)[idx] == 0) continue;
        }

        if (!isSource[idx]) {
          isSource[idx] = 1;
          sources.push_back(static_cast<int>(idx));
        }
      }
    }
  }

  // --- Collect residential origins (commuters) ---
  struct Origin {
    int roadIdx = -1;
    int commuters = 0;
  };

  std::vector<Origin> origins;
  origins.reserve(n / 16);

  const std::uint32_t seedMix = static_cast<std::uint32_t>(world.seed() ^ (world.seed() >> 32));

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants == 0) continue;
      if (!zoneHasAccess(x, y)) continue;
      Point road{};
      if (!PickAdjacentRoadTile(world, roadToEdge, x, y, road)) continue;

      const float desired = static_cast<float>(t.occupants) * employedShare;
      int commuters = static_cast<int>(std::floor(desired));
      const float frac = desired - static_cast<float>(commuters);
      if (frac > 0.0f && commuters < static_cast<int>(t.occupants)) {
        // Deterministic dithering so totals are stable but rounding isn't biased.
        const std::uint32_t h0 = Hash32(seedMix ^ (static_cast<std::uint32_t>(x) * 73856093u) ^
                                        (static_cast<std::uint32_t>(y) * 19349663u));
        const float u = HashToUnitFloat(h0);
        if (u < frac) commuters += 1;
      }

      commuters = std::clamp(commuters, 0, static_cast<int>(t.occupants));
      if (commuters <= 0) continue;

      const int idx = road.y * w + road.x;
      origins.push_back(Origin{idx, commuters});
      r.totalCommuters += commuters;
    }
  }

  if (r.totalCommuters <= 0) {
    return r;
  }

  if (sources.empty()) {
    // No reachable jobs => everyone is "unreachable".
    r.unreachableCommuters = r.totalCommuters;
    return r;
  }

  // --- Multi-source BFS on road tiles (shortest paths in road steps) ---
  std::vector<int> dist(n, -1);
  std::vector<int> parent(n, -1);
  std::vector<int> queue;
  queue.reserve(n / 2);

  for (int sidx : sources) {
    if (sidx < 0 || static_cast<std::size_t>(sidx) >= n) continue;
    dist[static_cast<std::size_t>(sidx)] = 0;
    parent[static_cast<std::size_t>(sidx)] = -1;
    queue.push_back(sidx);
  }

  auto isTraversableRoad = [&](int ridx) -> bool {
    if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) return false;
    const int x = ridx % w;
    const int y = ridx / w;
    if (!world.inBounds(x, y)) return false;
    if (world.at(x, y).overlay != Overlay::Road) return false;
    if (cfg.requireOutsideConnection) {
      const std::size_t ui = static_cast<std::size_t>(ridx);
      if (ui >= roadToEdge->size() || (*roadToEdge)[ui] == 0) return false;
    }
    return true;
  };

  std::size_t head = 0;
  while (head < queue.size()) {
    const int idx = queue[head++];
    const int x = idx % w;
    const int y = idx / w;

    const int dcur = dist[static_cast<std::size_t>(idx)];
    if (dcur < 0) continue;

    // Deterministic neighbor order: N, E, S, W.
    constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    for (const auto& d : dirs) {
      const int nx = x + d[0];
      const int ny = y + d[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const int nidx = ny * w + nx;

      const std::size_t ui = static_cast<std::size_t>(nidx);
      if (ui >= n) continue;
      if (dist[ui] != -1) continue;

      if (!isTraversableRoad(nidx)) continue;

      dist[ui] = dcur + 1;
      parent[ui] = idx;
      queue.push_back(nidx);
    }
  }

  // --- Assign commuters and accumulate traffic ---
  std::vector<std::pair<int, int>> commuteSamples;
  commuteSamples.reserve(origins.size());

  double sumDist = 0.0;
  int reachable = 0;

  for (const Origin& o : origins) {
    if (o.roadIdx < 0 || static_cast<std::size_t>(o.roadIdx) >= n) continue;
    if (!isTraversableRoad(o.roadIdx)) {
      r.unreachableCommuters += o.commuters;
      continue;
    }

    const int d = dist[static_cast<std::size_t>(o.roadIdx)];
    if (d < 0) {
      r.unreachableCommuters += o.commuters;
      continue;
    }

    reachable += o.commuters;
    sumDist += static_cast<double>(d) * static_cast<double>(o.commuters);
    commuteSamples.emplace_back(d, o.commuters);

    // Trace the parent pointers back to a job access point and increment traffic.
    int cur = o.roadIdx;
    while (cur != -1) {
      const std::size_t ui = static_cast<std::size_t>(cur);
      if (ui >= n) break;
      r.roadTraffic[ui] = SatAddU16(r.roadTraffic[ui], static_cast<std::uint32_t>(o.commuters));
      r.maxTraffic = std::max(r.maxTraffic, static_cast<int>(r.roadTraffic[ui]));
      cur = parent[ui];
    }
  }

  r.reachableCommuters = reachable;

  if (reachable > 0) {
    r.avgCommute = static_cast<float>(sumDist / static_cast<double>(reachable));

    // Weighted 95th percentile.
    std::sort(commuteSamples.begin(), commuteSamples.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    const int target = static_cast<int>(std::ceil(static_cast<double>(reachable) * 0.95));
    int accum = 0;
    int p95 = 0;
    for (const auto& s : commuteSamples) {
      accum += s.second;
      p95 = s.first;
      if (accum >= target) break;
    }
    r.p95Commute = static_cast<float>(p95);
  }

  // Congestion metric.
  const int cap = std::max(0, cfg.roadTileCapacity);
  std::uint64_t totalTraffic = 0;
  std::uint64_t over = 0;

  if (cap > 0) {
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint32_t tv = static_cast<std::uint32_t>(r.roadTraffic[i]);
      if (tv == 0) continue;
      totalTraffic += tv;
      if (tv > static_cast<std::uint32_t>(cap)) {
        over += (tv - static_cast<std::uint32_t>(cap));
        r.congestedRoadTiles += 1;
      }
    }
  } else {
    // cap == 0 => treat everything as congested.
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint32_t tv = static_cast<std::uint32_t>(r.roadTraffic[i]);
      if (tv == 0) continue;
      totalTraffic += tv;
      over += tv;
      r.congestedRoadTiles += 1;
    }
  }

  if (totalTraffic > 0) {
    r.congestion = static_cast<float>(static_cast<double>(over) / static_cast<double>(totalTraffic));
    r.congestion = std::clamp(r.congestion, 0.0f, 1.0f);
  }

  return r;
}

} // namespace isocity
