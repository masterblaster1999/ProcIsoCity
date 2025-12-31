#include "isocity/RoadGraphExport.hpp"

#include "isocity/Export.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace isocity {

namespace {

inline bool InBoundsImg(const PpmImage& img, int x, int y)
{
  return x >= 0 && y >= 0 && x < img.width && y < img.height;
}

inline void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (!InBoundsImg(img, x, y)) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) +
                           static_cast<std::size_t>(x)) *
                          3;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

struct DijkstraResult {
  std::vector<int> dist;
  std::vector<int> parent;
  std::vector<int> parentEdge;
};

constexpr int kInf = std::numeric_limits<int>::max() / 4;

DijkstraResult Dijkstra(const RoadGraph& g, int start)
{
  DijkstraResult r;
  const int n = static_cast<int>(g.nodes.size());
  r.dist.assign(static_cast<std::size_t>(n), kInf);
  r.parent.assign(static_cast<std::size_t>(n), -1);
  r.parentEdge.assign(static_cast<std::size_t>(n), -1);
  if (start < 0 || start >= n) return r;

  using Item = std::pair<int, int>; // (dist, node)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  r.dist[static_cast<std::size_t>(start)] = 0;
  pq.push({0, start});

  while (!pq.empty()) {
    const Item cur = pq.top();
    pq.pop();

    const int d = cur.first;
    const int u = cur.second;
    if (u < 0 || u >= n) continue;
    if (d != r.dist[static_cast<std::size_t>(u)]) continue;

    const RoadGraphNode& nu = g.nodes[static_cast<std::size_t>(u)];
    for (int ei : nu.edges) {
      if (ei < 0 || ei >= static_cast<int>(g.edges.size())) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
      const int v = (e.a == u) ? e.b : e.a;
      if (v < 0 || v >= n) continue;

      const int w = std::max(0, e.length);
      if (d > kInf - w) continue;
      const int nd = d + w;
      if (nd < r.dist[static_cast<std::size_t>(v)]) {
        r.dist[static_cast<std::size_t>(v)] = nd;
        r.parent[static_cast<std::size_t>(v)] = u;
        r.parentEdge[static_cast<std::size_t>(v)] = ei;
        pq.push({nd, v});
      }
    }
  }

  return r;
}

int PickDiameterStartNode(const RoadGraph& g)
{
  const int n = static_cast<int>(g.nodes.size());
  for (int i = 0; i < n; ++i) {
    if (!g.nodes[static_cast<std::size_t>(i)].edges.empty()) return i;
  }
  return (n > 0) ? 0 : -1;
}

int FarthestNode(const std::vector<int>& dist)
{
  int best = -1;
  int bestD = -1;
  for (int i = 0; i < static_cast<int>(dist.size()); ++i) {
    const int d = dist[static_cast<std::size_t>(i)];
    if (d >= kInf) continue;
    if (d > bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

bool EnsureParentDir(const std::string& path)
{
  if (path.empty()) return true;
  try {
    std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

} // namespace

int ComputeRoadGraphComponents(const RoadGraph& g, std::vector<int>& outNodeComponent)
{
  const int n = static_cast<int>(g.nodes.size());
  outNodeComponent.assign(static_cast<std::size_t>(n), -1);
  if (n == 0) return 0;

  int comp = 0;
  std::queue<int> q;

  for (int i = 0; i < n; ++i) {
    if (outNodeComponent[static_cast<std::size_t>(i)] != -1) continue;
    outNodeComponent[static_cast<std::size_t>(i)] = comp;
    q.push(i);

    while (!q.empty()) {
      const int u = q.front();
      q.pop();
      const RoadGraphNode& nu = g.nodes[static_cast<std::size_t>(u)];
      for (int ei : nu.edges) {
        if (ei < 0 || ei >= static_cast<int>(g.edges.size())) continue;
        const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
        const int v = (e.a == u) ? e.b : e.a;
        if (v < 0 || v >= n) continue;
        if (outNodeComponent[static_cast<std::size_t>(v)] != -1) continue;
        outNodeComponent[static_cast<std::size_t>(v)] = comp;
        q.push(v);
      }
    }

    ++comp;
  }

  return comp;
}

RoadGraphMetrics ComputeRoadGraphMetrics(const RoadGraph& g)
{
  RoadGraphMetrics m;
  m.nodes = static_cast<int>(g.nodes.size());
  m.edges = static_cast<int>(g.edges.size());

  for (const RoadGraphEdge& e : g.edges) {
    m.totalEdgeLength += static_cast<std::uint64_t>(std::max(0, e.length));
  }

  if (m.nodes > 0) {
    m.avgDegree = (m.edges > 0) ? (2.0 * static_cast<double>(m.edges) / static_cast<double>(m.nodes)) : 0.0;
    m.isolatedNodes = 0;
    for (const RoadGraphNode& n : g.nodes) {
      if (n.edges.empty()) ++m.isolatedNodes;
    }
  }
  if (m.edges > 0) {
    m.avgEdgeLength = static_cast<double>(m.totalEdgeLength) / static_cast<double>(m.edges);
  }

  // Components.
  std::vector<int> compId;
  m.components = ComputeRoadGraphComponents(g, compId);

  if (m.components > 0) {
    std::vector<int> compNodes(static_cast<std::size_t>(m.components), 0);
    std::vector<int> compEdgeRefs(static_cast<std::size_t>(m.components), 0);

    for (int i = 0; i < m.nodes; ++i) {
      const int c = compId[static_cast<std::size_t>(i)];
      if (c < 0 || c >= m.components) continue;
      compNodes[static_cast<std::size_t>(c)]++;
      compEdgeRefs[static_cast<std::size_t>(c)] += static_cast<int>(g.nodes[static_cast<std::size_t>(i)].edges.size());
    }

    for (int c = 0; c < m.components; ++c) {
      const int nodes = compNodes[static_cast<std::size_t>(c)];
      const int edges = compEdgeRefs[static_cast<std::size_t>(c)] / 2;
      m.largestComponentNodes = std::max(m.largestComponentNodes, nodes);
      m.largestComponentEdges = std::max(m.largestComponentEdges, edges);
    }
  }

  // Diameter.
  const RoadGraphDiameter d = ComputeApproxRoadGraphDiameter(g);
  m.approxDiameter = d.distance;
  m.diameterA = d.a;
  m.diameterB = d.b;

  return m;
}

RoadGraphDiameter ComputeApproxRoadGraphDiameter(const RoadGraph& g)
{
  RoadGraphDiameter out;
  const int n = static_cast<int>(g.nodes.size());
  if (n <= 0) return out;

  const int start = PickDiameterStartNode(g);
  if (start < 0) return out;

  const DijkstraResult d0 = Dijkstra(g, start);
  const int a = FarthestNode(d0.dist);
  if (a < 0) {
    out.a = start;
    out.b = start;
    out.distance = 0;
    out.nodePath = {start};
    return out;
  }

  const DijkstraResult d1 = Dijkstra(g, a);
  const int b = FarthestNode(d1.dist);
  if (b < 0) {
    out.a = a;
    out.b = a;
    out.distance = 0;
    out.nodePath = {a};
    return out;
  }

  out.a = a;
  out.b = b;
  out.distance = (d1.dist[static_cast<std::size_t>(b)] >= kInf) ? 0 : d1.dist[static_cast<std::size_t>(b)];

  // Reconstruct a path (b -> a).
  std::vector<int> nodes;
  std::vector<int> edges;
  int cur = b;
  nodes.push_back(cur);
  while (cur != -1 && cur != a) {
    const int pe = d1.parentEdge[static_cast<std::size_t>(cur)];
    const int pn = d1.parent[static_cast<std::size_t>(cur)];
    if (pn == -1) break;
    edges.push_back(pe);
    cur = pn;
    nodes.push_back(cur);
  }

  // If reconstruction didn't reach a, fall back to endpoints only.
  if (nodes.empty() || nodes.back() != a) {
    out.nodePath = {a, b};
    out.edgePath.clear();
    return out;
  }

  std::reverse(nodes.begin(), nodes.end());
  std::reverse(edges.begin(), edges.end());
  out.nodePath = std::move(nodes);
  out.edgePath = std::move(edges);

  return out;
}

bool ExpandRoadGraphNodePathToTiles(const RoadGraph& g, const std::vector<int>& nodePath, std::vector<Point>& outTiles)
{
  outTiles.clear();
  if (nodePath.empty()) return true;

  auto nodePos = [&](int id) -> Point {
    if (id < 0 || id >= static_cast<int>(g.nodes.size())) return Point{-999, -999};
    return g.nodes[static_cast<std::size_t>(id)].pos;
  };

  outTiles.push_back(nodePos(nodePath.front()));
  if (outTiles.back().x < -100) return false;

  for (std::size_t i = 1; i < nodePath.size(); ++i) {
    const int a = nodePath[i - 1];
    const int b = nodePath[i];
    if (a < 0 || b < 0) return false;
    if (a >= static_cast<int>(g.nodes.size()) || b >= static_cast<int>(g.nodes.size())) return false;

    int edgeIdx = -1;
    const RoadGraphNode& na = g.nodes[static_cast<std::size_t>(a)];
    for (int ei : na.edges) {
      if (ei < 0 || ei >= static_cast<int>(g.edges.size())) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
      if ((e.a == a && e.b == b) || (e.a == b && e.b == a)) {
        edgeIdx = ei;
        break;
      }
    }

    if (edgeIdx < 0) return false;

    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(edgeIdx)];
    std::vector<Point> seg = e.tiles;
    if (seg.empty()) {
      // Degenerate edge; just connect endpoints.
      outTiles.push_back(nodePos(b));
      continue;
    }

    const Point wantStart = nodePos(a);
    if (seg.front().x != wantStart.x || seg.front().y != wantStart.y) {
      if (seg.back().x == wantStart.x && seg.back().y == wantStart.y) {
        std::reverse(seg.begin(), seg.end());
      } else {
        // Can't orient; still try to append raw segment.
      }
    }

    // Stitch, skipping the first point to avoid duplicates.
    for (std::size_t k = 1; k < seg.size(); ++k) {
      outTiles.push_back(seg[k]);
    }
  }

  return true;
}

bool WriteRoadGraphDot(std::ostream& os, const RoadGraph& g, const RoadGraphMetrics* metrics,
                       const RoadGraphExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> compId;
  if (cfg.colorByComponent) {
    ComputeRoadGraphComponents(g, compId);
  }

  const char* palette[] = {
      "lightcoral",
      "lightskyblue",
      "lightgreen",
      "khaki",
      "plum",
      "lightsalmon",
      "lightgray",
      "palegreen",
      "paleturquoise",
      "wheat",
  };
  constexpr int paletteN = static_cast<int>(sizeof(palette) / sizeof(palette[0]));

  os << "graph RoadGraph {\n";
  os << "  graph [overlap=false, splines=true];\n";
  os << "  node [shape=circle, fontsize=10];\n";

  if (metrics) {
    os << "  // nodes=" << metrics->nodes << " edges=" << metrics->edges << " components=" << metrics->components
       << " approxDiameter=" << metrics->approxDiameter << "\n";
  }

  for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
    const RoadGraphNode& n = g.nodes[static_cast<std::size_t>(i)];
    os << "  " << i << " [label=\"" << i << "\\n(" << n.pos.x << "," << n.pos.y << ")\"";
    if (cfg.colorByComponent && i < static_cast<int>(compId.size())) {
      const int c = std::max(0, compId[static_cast<std::size_t>(i)]);
      const char* col = palette[c % paletteN];
      os << ", style=filled, fillcolor=\"" << col << "\"";
    }
    os << "];\n";
  }

  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    os << "  " << e.a << " -- " << e.b << " [label=\"" << e.length << "\"";
    if (cfg.includeEdgeTiles && !e.tiles.empty()) {
      // Keep it compact: show only endpoints + count.
      os << ", tooltip=\"tiles=" << e.tiles.size() << "\"";
    }
    os << "];\n";
  }

  os << "}\n";

  if (!os.good()) {
    if (outError) *outError = "failed writing DOT";
    return false;
  }
  return true;
}

static void WriteJsonMetrics(std::ostream& os, const RoadGraphMetrics& m, const RoadGraphDiameter* diameter)
{
  os << "  \"metrics\": {\n";
  os << "    \"nodes\": " << m.nodes << ",\n";
  os << "    \"edges\": " << m.edges << ",\n";
  os << "    \"totalEdgeLength\": " << m.totalEdgeLength << ",\n";
  os << "    \"components\": " << m.components << ",\n";
  os << "    \"largestComponentNodes\": " << m.largestComponentNodes << ",\n";
  os << "    \"largestComponentEdges\": " << m.largestComponentEdges << ",\n";
  os << "    \"isolatedNodes\": " << m.isolatedNodes << ",\n";
  os << "    \"avgDegree\": " << m.avgDegree << ",\n";
  os << "    \"avgEdgeLength\": " << m.avgEdgeLength << ",\n";
  os << "    \"approxDiameter\": " << m.approxDiameter << ",\n";
  os << "    \"diameterA\": " << m.diameterA << ",\n";
  os << "    \"diameterB\": " << m.diameterB << "\n";
  os << "  }";

  if (diameter && !diameter->nodePath.empty()) {
    os << ",\n  \"diameterPathNodes\": [";
    for (std::size_t i = 0; i < diameter->nodePath.size(); ++i) {
      os << diameter->nodePath[i];
      if (i + 1 != diameter->nodePath.size()) os << ',';
    }
    os << "]";
  }
}

bool WriteRoadGraphJson(std::ostream& os, const RoadGraph& g, const RoadGraphMetrics* metrics,
                        const RoadGraphDiameter* diameter, const RoadGraphExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();

  RoadGraphMetrics localMetrics;
  if (!metrics) {
    localMetrics = ComputeRoadGraphMetrics(g);
    metrics = &localMetrics;
  }

  std::vector<int> compId;
  ComputeRoadGraphComponents(g, compId);

  os << "{\n";
  WriteJsonMetrics(os, *metrics, diameter);
  os << ",\n";

  os << "  \"nodes\": [\n";
  for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
    const RoadGraphNode& n = g.nodes[static_cast<std::size_t>(i)];
    const int deg = static_cast<int>(n.edges.size());
    const int c = (i < static_cast<int>(compId.size())) ? compId[static_cast<std::size_t>(i)] : -1;
    os << "    {\"id\": " << i << ", \"x\": " << n.pos.x << ", \"y\": " << n.pos.y << ", \"degree\": " << deg
       << ", \"component\": " << c << "}";
    if (i + 1 != static_cast<int>(g.nodes.size())) os << ',';
    os << "\n";
  }
  os << "  ],\n";

  os << "  \"edges\": [\n";
  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const int c = (e.a >= 0 && e.a < static_cast<int>(compId.size())) ? compId[static_cast<std::size_t>(e.a)] : -1;
    os << "    {\"id\": " << ei << ", \"a\": " << e.a << ", \"b\": " << e.b << ", \"length\": " << e.length
       << ", \"component\": " << c;
    if (cfg.includeEdgeTiles) {
      os << ", \"tiles\": [";
      for (std::size_t k = 0; k < e.tiles.size(); ++k) {
        os << "[" << e.tiles[k].x << ',' << e.tiles[k].y << "]";
        if (k + 1 != e.tiles.size()) os << ',';
      }
      os << ']';
    }
    os << "}";
    if (ei + 1 != static_cast<int>(g.edges.size())) os << ',';
    os << "\n";
  }
  os << "  ]\n";

  os << "}\n";

  if (!os.good()) {
    if (outError) *outError = "failed writing JSON";
    return false;
  }
  return true;
}

bool WriteRoadGraphNodesCsv(std::ostream& os, const RoadGraph& g, const std::vector<int>* nodeComponent,
                            std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> local;
  if (!nodeComponent) {
    ComputeRoadGraphComponents(g, local);
    nodeComponent = &local;
  }

  os << "id,x,y,degree,component\n";
  for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
    const RoadGraphNode& n = g.nodes[static_cast<std::size_t>(i)];
    const int deg = static_cast<int>(n.edges.size());
    const int c = (i < static_cast<int>(nodeComponent->size())) ? (*nodeComponent)[static_cast<std::size_t>(i)] : -1;
    os << i << ',' << n.pos.x << ',' << n.pos.y << ',' << deg << ',' << c << "\n";
  }

  if (!os.good()) {
    if (outError) *outError = "failed writing nodes CSV";
    return false;
  }
  return true;
}

bool WriteRoadGraphEdgesCsv(std::ostream& os, const RoadGraph& g, const std::vector<int>* nodeComponent,
                            const RoadGraphExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> local;
  if (!nodeComponent) {
    ComputeRoadGraphComponents(g, local);
    nodeComponent = &local;
  }

  os << "id,a,b,length,component";
  if (cfg.includeEdgeTiles) os << ",tiles";
  os << "\n";

  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const int c = (e.a >= 0 && e.a < static_cast<int>(nodeComponent->size())) ? (*nodeComponent)[static_cast<std::size_t>(e.a)]
                                                                              : -1;
    os << ei << ',' << e.a << ',' << e.b << ',' << e.length << ',' << c;
    if (cfg.includeEdgeTiles) {
      os << ",\"";
      for (std::size_t k = 0; k < e.tiles.size(); ++k) {
        os << e.tiles[k].x << ':' << e.tiles[k].y;
        if (k + 1 != e.tiles.size()) os << ';';
      }
      os << '\"';
    }
    os << "\n";
  }

  if (!os.good()) {
    if (outError) *outError = "failed writing edges CSV";
    return false;
  }
  return true;
}

bool ExportRoadGraphDot(const std::string& path, const RoadGraph& g, const RoadGraphMetrics* metrics,
                        const RoadGraphExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }
  return WriteRoadGraphDot(f, g, metrics, cfg, outError);
}

bool ExportRoadGraphJson(const std::string& path, const RoadGraph& g, const RoadGraphMetrics* metrics,
                         const RoadGraphDiameter* diameter, const RoadGraphExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }
  return WriteRoadGraphJson(f, g, metrics, diameter, cfg, outError);
}

bool ExportRoadGraphNodesCsv(const std::string& path, const RoadGraph& g, const std::vector<int>* nodeComponent,
                             std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }
  return WriteRoadGraphNodesCsv(f, g, nodeComponent, outError);
}

bool ExportRoadGraphEdgesCsv(const std::string& path, const RoadGraph& g, const std::vector<int>* nodeComponent,
                             const RoadGraphExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }
  return WriteRoadGraphEdgesCsv(f, g, nodeComponent, cfg, outError);
}

PpmImage RenderRoadGraphDebugPpm(const World& world, const RoadGraph& g, ExportLayer baseLayer,
                                const std::vector<Point>* highlightTiles)
{
  PpmImage img = RenderPpmLayer(world, baseLayer, nullptr, nullptr, nullptr);
  if (img.width <= 0 || img.height <= 0) return img;
  if (img.rgb.size() != static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3) return img;

  // Mark nodes in yellow so intersections/endpoints are visible.
  for (const RoadGraphNode& n : g.nodes) {
    SetPixel(img, n.pos.x, n.pos.y, 255, 235, 60);
  }

  // Highlight a path (e.g. diameter) in red.
  if (highlightTiles) {
    for (const Point& p : *highlightTiles) {
      SetPixel(img, p.x, p.y, 255, 30, 30);
    }
  }

  return img;
}

} // namespace isocity
