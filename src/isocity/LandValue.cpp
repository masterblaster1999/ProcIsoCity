#include "isocity/LandValue.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace isocity {

namespace {

constexpr int kInf = 1'000'000;

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// Multi-source BFS distance-to-feature field (4-neighborhood).
//
// If blockWater is true, Water tiles are treated as impassable.
std::vector<int> MultiSourceDistanceField(const World& world, const std::vector<int>& sources, int maxDist,
                                          bool blockWater)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  std::vector<int> dist(n, kInf);
  std::vector<int> q;
  q.reserve(std::min<std::size_t>(n, 4096));

  for (int idx : sources) {
    if (idx < 0 || static_cast<std::size_t>(idx) >= n) continue;
    dist[static_cast<std::size_t>(idx)] = 0;
    q.push_back(idx);
  }

  if (q.empty()) return dist;

  std::size_t head = 0;
  while (head < q.size()) {
    const int idx = q[head++];
    const int d = dist[static_cast<std::size_t>(idx)];
    if (d >= maxDist) continue;

    const int x = idx % w;
    const int y = idx / w;

    constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto& dir : dirs) {
      const int nx = x + dir[0];
      const int ny = y + dir[1];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (blockWater && world.at(nx, ny).terrain == Terrain::Water) continue;

      const int nidx = ny * w + nx;
      int& nd = dist[static_cast<std::size_t>(nidx)];
      if (nd <= d + 1) continue;
      nd = d + 1;
      q.push_back(nidx);
    }
  }

  return dist;
}

inline float DistToAmenityScore(int dist, int radius)
{
  if (radius <= 0) return 0.0f;
  if (dist < 0 || dist >= kInf) return 0.0f;
  if (dist > radius) return 0.0f;
  const float t = static_cast<float>(dist) / static_cast<float>(radius);
  return Clamp01(1.0f - t);
}

} // namespace

LandValueResult ComputeLandValue(const World& world, const LandValueConfig& cfg, const TrafficResult* traffic,
                                 const std::vector<std::uint8_t>* roadToEdgeMask)
{
  LandValueResult out;
  out.w = world.width();
  out.h = world.height();

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.value.assign(n, 0.0f);
  out.parkAmenity.assign(n, 0.0f);
  out.waterAmenity.assign(n, 0.0f);
  out.pollution.assign(n, 0.0f);
  out.traffic.assign(n, 0.0f);

  // --- Sources ---
  std::vector<int> parkSources;
  std::vector<int> waterSources;
  std::vector<int> indSources;

  parkSources.reserve(n / 32);
  waterSources.reserve(n / 8);
  indSources.reserve(n / 32);

  const bool useEdgeMask = (cfg.requireOutsideConnection && roadToEdgeMask &&
                           roadToEdgeMask->size() == n);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const int idx = y * w + x;

      if (t.terrain == Terrain::Water) {
        waterSources.push_back(idx);
        continue;
      }

      if (t.overlay == Overlay::Industrial) {
        indSources.push_back(idx);
      }

      if (t.overlay == Overlay::Park) {
        // Parks only "count" if they have road access, and optionally if that road
        // component reaches the map edge.
        if (!world.hasAdjacentRoad(x, y)) continue;
        if (useEdgeMask) {
          if (!HasAdjacentRoadConnectedToEdge(world, *roadToEdgeMask, x, y)) continue;
        }
        parkSources.push_back(idx);
      }
    }
  }

  // --- Distance fields ---
  // Note: parks/pollution do not "cross" water (islands are isolated).
  const std::vector<int> distPark = MultiSourceDistanceField(world, parkSources, std::max(0, cfg.parkRadius), true);
  const std::vector<int> distInd =
      MultiSourceDistanceField(world, indSources, std::max(0, cfg.pollutionRadius), true);
  // Water proximity is geometric; we don't treat water as a barrier.
  const std::vector<int> distWater = MultiSourceDistanceField(world, waterSources, std::max(0, cfg.waterRadius), false);

  // --- Traffic penalty field ---
  const bool trafficOk = (traffic && traffic->maxTraffic > 0 && traffic->roadTraffic.size() == n);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = y * w + x;

      float maxTv = 0.0f;
      if (trafficOk) {
        auto consider = [&](int tx, int ty) {
          if (!world.inBounds(tx, ty)) return;
          const Tile& rt = world.at(tx, ty);
          if (rt.overlay != Overlay::Road) return;
          const std::size_t tidx = static_cast<std::size_t>(ty) * static_cast<std::size_t>(w) +
                                   static_cast<std::size_t>(tx);
          const float eff = static_cast<float>(traffic->roadTraffic[tidx]) *
                            RoadTrafficSpillMultiplierForLevel(static_cast<int>(rt.level));
          maxTv = std::max(maxTv, eff);
        };

        consider(x, y);
        consider(x + 1, y);
        consider(x - 1, y);
        consider(x, y + 1);
        consider(x, y - 1);
      }

      if (maxTv <= 0.0f || !trafficOk) {
        out.traffic[static_cast<std::size_t>(idx)] = 0.0f;
      } else {
      const float denom = static_cast<float>(traffic->maxTraffic);
      const float norm = std::clamp(maxTv / std::max(1.0f, denom), 0.0f, 1.0f);
        // Emphasize low flows so the overlay is readable early.
        out.traffic[static_cast<std::size_t>(idx)] = std::pow(norm, 0.45f);
      }
    }
  }

  // --- Compose final land value ---
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = y * w + x;
      const std::size_t sidx = static_cast<std::size_t>(idx);
      const Tile& t = world.at(x, y);

      if (t.terrain == Terrain::Water) {
        // Leave as zeros.
        continue;
      }

      const float park = DistToAmenityScore(distPark[sidx], cfg.parkRadius);
      const float water = DistToAmenityScore(distWater[sidx], cfg.waterRadius);
      const float pol = DistToAmenityScore(distInd[sidx], cfg.pollutionRadius);
      const float traf = out.traffic[sidx];

      out.parkAmenity[sidx] = park;
      out.waterAmenity[sidx] = water;
      out.pollution[sidx] = pol;

      float v = cfg.base;
      v += cfg.parkBonus * park;
      v += cfg.waterBonus * water;
      v -= cfg.pollutionPenalty * pol;
      v -= cfg.trafficPenalty * traf;

      // A bit less valuable if there is no adjacent road (accessibility).
      if (!world.hasAdjacentRoad(x, y)) {
        v -= cfg.noRoadPenalty;
      } else if (useEdgeMask) {
        // Outside connection rule: discourage disconnected neighborhoods.
        if (!HasAdjacentRoadConnectedToEdge(world, *roadToEdgeMask, x, y)) {
          v -= cfg.disconnectedPenalty;
        }
      }

      out.value[sidx] = Clamp01(v);
    }
  }

  return out;
}

} // namespace isocity
