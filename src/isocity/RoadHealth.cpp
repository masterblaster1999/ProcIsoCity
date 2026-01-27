#include "isocity/RoadHealth.hpp"

#include "isocity/RoadGraph.hpp"

#include <algorithm>

namespace isocity {

namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline float Clamp01(float v)
{
  return std::clamp(v, 0.0f, 1.0f);
}

inline double MaxOrOne(const std::vector<double>& v)
{
  double m = 0.0;
  for (double x : v) m = std::max(m, x);
  return (m > 0.0) ? m : 1.0;
}

inline void StampCross(std::vector<float>& field, int w, int h, int x, int y, float v)
{
  auto stamp = [&](int sx, int sy) {
    if (sx < 0 || sy < 0 || sx >= w || sy >= h) return;
    const std::size_t i = FlatIdx(sx, sy, w);
    field[i] = std::max(field[i], v);
  };
  stamp(x, y);
  stamp(x - 1, y);
  stamp(x + 1, y);
  stamp(x, y - 1);
  stamp(x, y + 1);
}

} // namespace

RoadHealthResult ComputeRoadHealth(const World& world, const RoadHealthConfig& cfg, const TrafficResult* traffic)
{
  RoadHealthResult out{};
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  out.centrality01.assign(n, 0.0f);
  out.vulnerability01.assign(n, 0.0f);
  out.bypassMask.assign(n, 0);

  if (w <= 0 || h <= 0) return out;

  // Build compressed road graph.
  const RoadGraph g = BuildRoadGraph(world);
  out.nodes = static_cast<int>(g.nodes.size());
  out.edges = static_cast<int>(g.edges.size());
  if (out.nodes <= 0 || out.edges <= 0) {
    return out;
  }

  // --- Centrality ---
  RoadGraphCentralityConfig ccfg{};
  ccfg.weightMode = cfg.weightMode;

  // Auto sampling to keep exports/tools responsive on huge graphs.
  if (cfg.maxSources > 0) {
    ccfg.maxSources = cfg.maxSources;
  } else {
    ccfg.maxSources = (out.nodes <= std::max(0, cfg.autoExactMaxNodes)) ? 0 : std::max(1, cfg.autoSampleSources);
  }
  ccfg.scaleSampleToFull = true;
  ccfg.undirected = true;
  ccfg.normalizeBetweenness = true;
  ccfg.closenessComponentScale = true;

  const RoadGraphCentralityResult c = ComputeRoadGraphCentrality(g, ccfg,
                                                                ccfg.weightMode == RoadGraphEdgeWeightMode::TravelTimeMilli
                                                                    ? &world
                                                                    : nullptr);
  out.sourcesUsed = c.sourcesUsed;

  const int m = out.edges;
  const int nn = out.nodes;

  // Choose normalized values when available; otherwise normalize by max.
  const bool edgeNormOk = static_cast<int>(c.edgeBetweennessNorm.size()) == m;
  const bool nodeNormOk = static_cast<int>(c.nodeBetweennessNorm.size()) == nn;

  const std::vector<double>& eVal = edgeNormOk ? c.edgeBetweennessNorm : c.edgeBetweenness;
  const std::vector<double>& nVal = nodeNormOk ? c.nodeBetweennessNorm : c.nodeBetweenness;

  const double eMax = edgeNormOk ? 1.0 : MaxOrOne(eVal);
  const double nMax = nodeNormOk ? 1.0 : MaxOrOne(nVal);

  // Map edge betweenness onto road tiles.
  for (int ei = 0; ei < m; ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    if (e.tiles.empty()) continue;
    const double raw = (static_cast<std::size_t>(ei) < eVal.size()) ? eVal[static_cast<std::size_t>(ei)] : 0.0;
    const float v01 = Clamp01(static_cast<float>(raw / eMax));
    if (v01 <= 0.0f) continue;

    for (const Point& p : e.tiles) {
      if (!world.inBounds(p.x, p.y)) continue;
      const std::size_t i = FlatIdx(p.x, p.y, w);
      out.centrality01[i] = std::max(out.centrality01[i], v01);
    }
  }

  // Optionally stamp node centrality so intersections remain visible at low zoom.
  if (cfg.includeNodeCentrality) {
    for (int ni = 0; ni < nn; ++ni) {
      const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(ni)];
      const double raw = (static_cast<std::size_t>(ni) < nVal.size()) ? nVal[static_cast<std::size_t>(ni)] : 0.0;
      const float v01 = Clamp01(static_cast<float>(raw / nMax));
      if (v01 <= 0.0f) continue;
      StampCross(out.centrality01, w, h, node.pos.x, node.pos.y, v01);
    }
  }

  // --- Vulnerability (bridges + articulation nodes) ---
  const RoadGraphResilienceResult r = ComputeRoadGraphResilience(g);
  out.bridgeEdges = static_cast<int>(r.bridgeEdges.size());
  out.articulationNodes = static_cast<int>(r.articulationNodes.size());

  // Bridge impact score: 2*min(sideA, sideB) / (sideA+sideB), in [0,1].
  for (int ei : r.bridgeEdges) {
    if (ei < 0 || ei >= m) continue;
    const int a = (static_cast<std::size_t>(ei) < r.bridgeSubtreeNodes.size()) ? r.bridgeSubtreeNodes[static_cast<std::size_t>(ei)] : 0;
    const int b = (static_cast<std::size_t>(ei) < r.bridgeOtherNodes.size()) ? r.bridgeOtherNodes[static_cast<std::size_t>(ei)] : 0;
    const int tot = a + b;
    if (tot <= 0) continue;
    const int mn = std::min(a, b);
    const float impact01 = Clamp01(2.0f * (static_cast<float>(mn) / static_cast<float>(tot)));
    if (impact01 <= 0.0f) continue;

    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    for (const Point& p : e.tiles) {
      if (!world.inBounds(p.x, p.y)) continue;
      const std::size_t i = FlatIdx(p.x, p.y, w);
      out.vulnerability01[i] = std::max(out.vulnerability01[i], impact01);
    }
  }

  // Articulation nodes: mark with a stable base vulnerability, optionally boosted by node centrality.
  for (int ni : r.articulationNodes) {
    if (ni < 0 || ni >= nn) continue;
    const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(ni)];

    float v = std::clamp(cfg.articulationVulnerabilityBase, 0.0f, 1.0f);

    // Boost by centrality when present.
    const double raw = (static_cast<std::size_t>(ni) < nVal.size()) ? nVal[static_cast<std::size_t>(ni)] : 0.0;
    const float c01 = Clamp01(static_cast<float>(raw / nMax));
    v = std::max(v, 0.50f + 0.50f * c01);

    StampCross(out.vulnerability01, w, h, node.pos.x, node.pos.y, v);
  }

  // --- Bypass suggestions (optional) ---
  if (cfg.includeBypass && cfg.bypassCfg.top > 0 && out.bridgeEdges > 0) {
    out.bypasses = SuggestRoadResilienceBypasses(world, g, r, cfg.bypassCfg, traffic);
    for (const RoadResilienceBypassSuggestion& s : out.bypasses) {
      for (const Point& p : s.path) {
        if (!world.inBounds(p.x, p.y)) continue;
        out.bypassMask[FlatIdx(p.x, p.y, w)] = 1;
      }
    }
  }

  return out;
}

} // namespace isocity
