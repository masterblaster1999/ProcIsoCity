#include "isocity/Traffic.hpp"

#include "isocity/FlowField.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"
#include "isocity/ZoneAccess.hpp"

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

void BuildCongestionExtraCostMilli(const World& world, const TrafficConfig& cfg,
                                  const std::vector<std::uint32_t>& traffic,
                                  std::vector<int>& outExtra)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    outExtra.clear();
    return;
  }
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  outExtra.assign(n, 0);
  if (traffic.size() != n) return;

  const double alpha = std::max(0.0, static_cast<double>(cfg.congestionAlpha));
  const double beta = std::max(0.0, static_cast<double>(cfg.congestionBeta));
  if (alpha <= 0.0 || beta <= 0.0) return;

  const double capScale = std::max(0.01, static_cast<double>(cfg.congestionCapacityScale));
  const double ratioClamp = std::clamp(static_cast<double>(cfg.congestionRatioClamp), 0.5, 10.0);

  // Use at least 1 so divisions are safe.
  const int baseCap = std::max(1, cfg.roadTileCapacity);

  for (std::size_t i = 0; i < n; ++i) {
    const std::uint32_t v = traffic[i];
    if (v == 0) continue;
    const int x = static_cast<int>(i % static_cast<std::size_t>(w));
    const int y = static_cast<int>(i / static_cast<std::size_t>(w));
    if (!world.inBounds(x, y)) continue;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) continue;

    const int level = static_cast<int>(t.level);
    const int capRaw = RoadCapacityForLevel(baseCap, level);
    const double cap = std::max(1.0, static_cast<double>(std::max(1, capRaw)) * capScale);

    double ratio = static_cast<double>(v) / cap;
    if (ratio <= 0.0) continue;
    ratio = std::min(ratio, ratioClamp);

    const double mult = alpha * std::pow(ratio, beta);
    if (mult <= 0.0) continue;

    const int baseCost = (t.terrain == Terrain::Water)
      ? RoadBridgeTravelTimeMilliForLevel(level)
      : RoadTravelTimeMilliForLevel(level);
    const double extraD = static_cast<double>(baseCost) * mult;
    int extra = static_cast<int>(std::lround(extraD));
    if (extra < 0) extra = 0;
    // Keep per-tile costs bounded so int path costs stay safe.
    extra = std::min(extra, 200000);
    outExtra[i] = extra;
  }
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

  // Zone access: allows interior tiles of a connected zoned area to be reachable via a
  // road-adjacent boundary tile.
  const ZoneAccessMap zoneAccess = BuildZoneAccessMap(world, roadToEdge);

  // --- Collect job access points (sources) ---
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

      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (zidx >= zoneAccess.roadIdx.size()) continue;
      const int accessRoad = zoneAccess.roadIdx[zidx];
      if (accessRoad < 0) continue;

      // Preserve the older behavior for boundary tiles: if this job tile touches road tiles,
      // treat each adjacent road as a job source. For interior tiles, fall back to the
      // propagated access road.
      bool addedAdjacent = false;
      constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
      for (const auto& d : dirs) {
        const int rx = x + d[0];
        const int ry = y + d[1];
        if (!world.inBounds(rx, ry)) continue;
        if (world.at(rx, ry).overlay != Overlay::Road) continue;

        const int ridx = ry * w + rx;
        const std::size_t ur = static_cast<std::size_t>(ridx);
        if (ur >= isSource.size()) continue;
        if (cfg.requireOutsideConnection) {
          if (!roadToEdge || ur >= roadToEdge->size() || (*roadToEdge)[ur] == 0) continue;
        }

        if (!isSource[ur]) {
          isSource[ur] = 1;
          sources.push_back(ridx);
        }
        addedAdjacent = true;
      }

      if (!addedAdjacent) {
        const std::size_t ur = static_cast<std::size_t>(accessRoad);
        if (ur < isSource.size() && !isSource[ur]) {
          isSource[ur] = 1;
          sources.push_back(accessRoad);
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

      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (zidx >= zoneAccess.roadIdx.size()) continue;
      if (zoneAccess.roadIdx[zidx] < 0) continue;

      // Prefer a directly-adjacent road if available; otherwise use the propagated access road.
      int ridx = -1;
      Point road{};
      if (PickAdjacentRoadTile(world, roadToEdge, x, y, road)) {
        ridx = road.y * w + road.x;
      } else {
        ridx = zoneAccess.roadIdx[zidx];
      }
      if (ridx < 0) continue;

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

      origins.push_back(Origin{ridx, commuters});
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

  // --- Multi-pass routing / assignment ---
  //
  // Classic behavior: 1 pass, assign everyone on the single shortest path.
  // Congestion-aware: multiple incremental passes with travel time penalties derived
  // from the traffic predicted so far.
  RoadFlowFieldConfig fcfg;
  fcfg.requireOutsideConnection = cfg.requireOutsideConnection;
  fcfg.computeOwner = false;
  fcfg.useTravelTime = true;

  const bool useCongestion = cfg.congestionAwareRouting && cfg.congestionIterations > 1 &&
                             cfg.congestionAlpha > 0.0f && cfg.congestionBeta > 0.0f;
  const int passes = useCongestion ? std::clamp(cfg.congestionIterations, 2, 16) : 1;
  r.usedCongestionAwareRouting = useCongestion;
  r.routingPasses = passes;

  std::vector<std::uint32_t> trafficForCost(n, 0);
  std::vector<int> extraCost;

  std::vector<std::pair<int, int>> commuteSamples;
  std::vector<std::pair<int, int>> timeSamples;
  commuteSamples.reserve(origins.size() * static_cast<std::size_t>(passes));
  timeSamples.reserve(origins.size() * static_cast<std::size_t>(passes));

  double sumDist = 0.0;
  double sumCost = 0.0;
  int reachable = 0;

  constexpr double kMilli = 1000.0;

  for (int pass = 0; pass < passes; ++pass) {
    if (useCongestion) {
      BuildCongestionExtraCostMilli(world, cfg, trafficForCost, extraCost);
    } else {
      extraCost.clear();
    }

    const RoadFlowField field = BuildRoadFlowField(world, sources, fcfg, roadToEdge,
                                                   useCongestion ? &extraCost : nullptr);

    for (const Origin& o : origins) {
      if (o.commuters <= 0) continue;
      if (o.roadIdx < 0 || static_cast<std::size_t>(o.roadIdx) >= n) continue;

      // Deterministic partition of each origin's commuters across passes.
      const int a = (o.commuters * pass) / passes;
      const int b = (o.commuters * (pass + 1)) / passes;
      const int chunk = b - a;
      if (chunk <= 0) continue;

      if (static_cast<std::size_t>(o.roadIdx) >= field.dist.size() ||
          static_cast<std::size_t>(o.roadIdx) >= field.cost.size()) {
        r.unreachableCommuters += chunk;
        continue;
      }

      const std::size_t uo = static_cast<std::size_t>(o.roadIdx);
      const int d = field.dist[uo];
      const int c = field.cost[uo];

      if (d < 0 || c < 0) {
        r.unreachableCommuters += chunk;
        continue;
      }

      reachable += chunk;
      sumDist += static_cast<double>(d) * static_cast<double>(chunk);
      sumCost += static_cast<double>(c) * static_cast<double>(chunk);
      commuteSamples.emplace_back(d, chunk);
      timeSamples.emplace_back(c, chunk);

      // Trace the parent pointers back to a job access point and increment traffic.
      int cur = o.roadIdx;
      int guard = 0;
      while (cur != -1 && guard++ < static_cast<int>(n) + 8) {
        const std::size_t ui = static_cast<std::size_t>(cur);
        if (ui >= n) break;
        r.roadTraffic[ui] = SatAddU16(r.roadTraffic[ui], static_cast<std::uint32_t>(chunk));
        trafficForCost[ui] += static_cast<std::uint32_t>(chunk);
        cur = field.parent[ui];
      }
    }
  }

  r.reachableCommuters = reachable;
  // Compute max after assignment so it reflects the final heatmap.
  for (std::size_t i = 0; i < n; ++i) {
    r.maxTraffic = std::max(r.maxTraffic, static_cast<int>(r.roadTraffic[i]));
  }

  if (reachable > 0) {
    r.avgCommute = static_cast<float>(sumDist / static_cast<double>(reachable));
    r.avgCommuteTime = static_cast<float>((sumCost / static_cast<double>(reachable)) / kMilli);

    // Weighted 95th percentile (steps).
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

    // Weighted 95th percentile (travel time).
    std::sort(timeSamples.begin(), timeSamples.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    accum = 0;
    int p95Cost = 0;
    for (const auto& s : timeSamples) {
      accum += s.second;
      p95Cost = s.first;
      if (accum >= target) break;
    }
    r.p95CommuteTime = static_cast<float>(static_cast<double>(p95Cost) / kMilli);
  }

  // Congestion metric.
  //
  // Capacity is road-class dependent: upgraded roads can carry more commuters before
  // being considered congested.
  const int baseCap = cfg.roadTileCapacity;
  std::uint64_t totalTraffic = 0;
  std::uint64_t over = 0;

  if (baseCap > 0) {
    for (std::size_t i = 0; i < n; ++i) {
      const std::uint32_t tv = static_cast<std::uint32_t>(r.roadTraffic[i]);
      if (tv == 0) continue;
      totalTraffic += tv;
      const int rx = static_cast<int>(i % static_cast<std::size_t>(w));
      const int ry = static_cast<int>(i / static_cast<std::size_t>(w));
      const int cap = RoadCapacityForLevel(baseCap, static_cast<int>(world.at(rx, ry).level));
      if (tv > static_cast<std::uint32_t>(std::max(0, cap))) {
        over += (tv - static_cast<std::uint32_t>(std::max(0, cap)));
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
