#include "isocity/RoadGraphTraffic.hpp"

#include "isocity/Road.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace isocity {

namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline int TileCapacity(const World& world, int x, int y, const RoadGraphTrafficConfig& cfg)
{
  if (!world.inBounds(x, y)) return 0;
  const Tile& t = world.at(x, y);
  if (t.overlay != Overlay::Road) return 0;

  const int base = std::max(0, cfg.baseTileCapacity);
  if (!cfg.useRoadLevelCapacity) return std::max(1, base);

  const int cap = RoadCapacityForLevel(base, static_cast<int>(t.level));
  return std::max(1, cap);
}

} // namespace

RoadGraphTrafficResult AggregateTrafficOnRoadGraph(const World& world, const RoadGraph& g, const TrafficResult& traffic,
                                                  const RoadGraphTrafficConfig& cfg)
{
  RoadGraphTrafficResult out;
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t nTiles = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const bool hasTraffic = traffic.roadTraffic.size() == nTiles;

  out.nodes.resize(g.nodes.size());
  out.edges.resize(g.edges.size());

  // --- Nodes ---
  for (std::size_t i = 0; i < g.nodes.size(); ++i) {
    RoadGraphTrafficNodeStats ns{};
    const RoadGraphNode& n = g.nodes[i];
    ns.pos = n.pos;
    ns.degree = static_cast<int>(n.edges.size());

    if (world.inBounds(n.pos.x, n.pos.y) && hasTraffic) {
      const std::size_t idx = FlatIdx(n.pos.x, n.pos.y, w);
      ns.traffic = static_cast<int>(traffic.roadTraffic[idx]);
      ns.capacity = TileCapacity(world, n.pos.x, n.pos.y, cfg);
      if (ns.capacity > 0) ns.util = static_cast<double>(ns.traffic) / static_cast<double>(ns.capacity);
    }

    out.nodes[i] = ns;
  }

  // --- Edges ---
  for (std::size_t i = 0; i < g.edges.size(); ++i) {
    RoadGraphTrafficEdgeStats es{};
    const RoadGraphEdge& e = g.edges[i];
    es.a = e.a;
    es.b = e.b;
    es.length = e.length;
    es.tileCount = static_cast<int>(e.tiles.size());
    es.interiorTileCount = std::max(0, es.tileCount - 2);

    // Initialize mins so they can be updated.
    es.minCapacityAll = std::numeric_limits<int>::max();
    es.minCapacityInterior = std::numeric_limits<int>::max();

    for (int ti = 0; ti < static_cast<int>(e.tiles.size()); ++ti) {
      const Point& p = e.tiles[static_cast<std::size_t>(ti)];
      if (!world.inBounds(p.x, p.y)) continue;

      const bool interior = (ti > 0 && ti + 1 < static_cast<int>(e.tiles.size()));

      const int cap = TileCapacity(world, p.x, p.y, cfg);
      const int v = (hasTraffic) ? static_cast<int>(traffic.roadTraffic[FlatIdx(p.x, p.y, w)]) : 0;

      // All tiles
      es.sumTrafficAll += static_cast<std::uint64_t>(std::max(0, v));
      es.maxTrafficAll = std::max(es.maxTrafficAll, v);
      es.sumCapacityAll += static_cast<std::uint64_t>(std::max(0, cap));
      es.minCapacityAll = std::min(es.minCapacityAll, cap);
      es.maxCapacityAll = std::max(es.maxCapacityAll, cap);

      if (cap > 0) {
        const double u = static_cast<double>(v) / static_cast<double>(cap);
        es.sumUtilAll += u;
        es.maxUtilAll = std::max(es.maxUtilAll, u);

        if (v > cap) {
          es.congestedTilesAll++;
          es.excessTrafficAll += static_cast<std::uint64_t>(v - cap);
        }
      }

      // Interior tiles
      if (interior) {
        es.sumTrafficInterior += static_cast<std::uint64_t>(std::max(0, v));
        es.maxTrafficInterior = std::max(es.maxTrafficInterior, v);
        es.sumCapacityInterior += static_cast<std::uint64_t>(std::max(0, cap));
        es.minCapacityInterior = std::min(es.minCapacityInterior, cap);
        es.maxCapacityInterior = std::max(es.maxCapacityInterior, cap);

        if (cap > 0) {
          const double u = static_cast<double>(v) / static_cast<double>(cap);
          es.sumUtilInterior += u;
          es.maxUtilInterior = std::max(es.maxUtilInterior, u);

          if (v > cap) {
            es.congestedTilesInterior++;
            es.excessTrafficInterior += static_cast<std::uint64_t>(v - cap);
          }
        }
      }
    }

    if (es.minCapacityAll == std::numeric_limits<int>::max()) es.minCapacityAll = 0;
    if (es.minCapacityInterior == std::numeric_limits<int>::max()) es.minCapacityInterior = 0;

    out.edges[i] = es;
  }

  // --- Incident edge aggregates on nodes (interior-only) ---
  for (std::size_t ni = 0; ni < g.nodes.size(); ++ni) {
    RoadGraphTrafficNodeStats& ns = out.nodes[ni];
    const RoadGraphNode& n = g.nodes[ni];

    for (int ei : n.edges) {
      if (ei < 0 || ei >= static_cast<int>(out.edges.size())) continue;
      const RoadGraphTrafficEdgeStats& es = out.edges[static_cast<std::size_t>(ei)];
      ns.incidentSumTraffic += es.sumTrafficInterior;

      const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
      ns.incidentMaxUtil = std::max(ns.incidentMaxUtil, u);
    }
  }

  return out;
}

RoadGraphTrafficResult AggregateFlowOnRoadGraph(const World& world, const RoadGraph& g,
                                                const std::vector<std::uint32_t>& roadFlow,
                                                const RoadGraphTrafficConfig& cfg)
{
  RoadGraphTrafficResult out;
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t nTiles = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const bool hasFlow = roadFlow.size() == nTiles;

  out.nodes.resize(g.nodes.size());
  out.edges.resize(g.edges.size());

  auto flowAt = [&](std::size_t idx) -> int {
    if (!hasFlow || idx >= roadFlow.size()) return 0;
    const std::uint32_t v = roadFlow[idx];
    if (v > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) return std::numeric_limits<int>::max();
    return static_cast<int>(v);
  };

  // --- Nodes ---
  for (std::size_t i = 0; i < g.nodes.size(); ++i) {
    RoadGraphTrafficNodeStats ns{};
    const RoadGraphNode& n = g.nodes[i];
    ns.pos = n.pos;
    ns.degree = static_cast<int>(n.edges.size());

    if (world.inBounds(n.pos.x, n.pos.y) && hasFlow) {
      const std::size_t idx = FlatIdx(n.pos.x, n.pos.y, w);
      ns.traffic = flowAt(idx);
      ns.capacity = TileCapacity(world, n.pos.x, n.pos.y, cfg);
      if (ns.capacity > 0) ns.util = static_cast<double>(ns.traffic) / static_cast<double>(ns.capacity);
    }

    out.nodes[i] = ns;
  }

  // --- Edges ---
  for (std::size_t i = 0; i < g.edges.size(); ++i) {
    RoadGraphTrafficEdgeStats es{};
    const RoadGraphEdge& e = g.edges[i];
    es.a = e.a;
    es.b = e.b;
    es.length = e.length;
    es.tileCount = static_cast<int>(e.tiles.size());
    es.interiorTileCount = std::max(0, es.tileCount - 2);

    // Initialize mins so they can be updated.
    es.minCapacityAll = std::numeric_limits<int>::max();
    es.minCapacityInterior = std::numeric_limits<int>::max();

    for (int ti = 0; ti < static_cast<int>(e.tiles.size()); ++ti) {
      const Point& p = e.tiles[static_cast<std::size_t>(ti)];
      if (!world.inBounds(p.x, p.y)) continue;

      const bool interior = (ti > 0 && ti + 1 < static_cast<int>(e.tiles.size()));

      const int cap = TileCapacity(world, p.x, p.y, cfg);
      const int v = (hasFlow) ? flowAt(FlatIdx(p.x, p.y, w)) : 0;

      // All tiles
      es.sumTrafficAll += static_cast<std::uint64_t>(std::max(0, v));
      es.maxTrafficAll = std::max(es.maxTrafficAll, v);
      es.sumCapacityAll += static_cast<std::uint64_t>(std::max(0, cap));
      es.minCapacityAll = std::min(es.minCapacityAll, cap);
      es.maxCapacityAll = std::max(es.maxCapacityAll, cap);

      if (cap > 0) {
        const double u = static_cast<double>(v) / static_cast<double>(cap);
        es.sumUtilAll += u;
        es.maxUtilAll = std::max(es.maxUtilAll, u);

        if (v > cap) {
          es.congestedTilesAll++;
          es.excessTrafficAll += static_cast<std::uint64_t>(v - cap);
        }
      }

      // Interior tiles
      if (interior) {
        es.sumTrafficInterior += static_cast<std::uint64_t>(std::max(0, v));
        es.maxTrafficInterior = std::max(es.maxTrafficInterior, v);
        es.sumCapacityInterior += static_cast<std::uint64_t>(std::max(0, cap));
        es.minCapacityInterior = std::min(es.minCapacityInterior, cap);
        es.maxCapacityInterior = std::max(es.maxCapacityInterior, cap);

        if (cap > 0) {
          const double u = static_cast<double>(v) / static_cast<double>(cap);
          es.sumUtilInterior += u;
          es.maxUtilInterior = std::max(es.maxUtilInterior, u);

          if (v > cap) {
            es.congestedTilesInterior++;
            es.excessTrafficInterior += static_cast<std::uint64_t>(v - cap);
          }
        }
      }
    }

    if (es.minCapacityAll == std::numeric_limits<int>::max()) es.minCapacityAll = 0;
    if (es.minCapacityInterior == std::numeric_limits<int>::max()) es.minCapacityInterior = 0;

    out.edges[i] = es;
  }

  // --- Incident edge aggregates on nodes (interior-only) ---
  for (std::size_t ni = 0; ni < g.nodes.size(); ++ni) {
    RoadGraphTrafficNodeStats& ns = out.nodes[ni];
    const RoadGraphNode& n = g.nodes[ni];

    for (int ei : n.edges) {
      if (ei < 0 || ei >= static_cast<int>(out.edges.size())) continue;
      const RoadGraphTrafficEdgeStats& es = out.edges[static_cast<std::size_t>(ei)];
      ns.incidentSumTraffic += es.sumTrafficInterior;

      const double u = (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
      ns.incidentMaxUtil = std::max(ns.incidentMaxUtil, u);
    }
  }

  return out;
}

} // namespace isocity
