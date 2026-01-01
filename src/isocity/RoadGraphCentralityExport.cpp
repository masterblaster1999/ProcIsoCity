#include "isocity/RoadGraphCentralityExport.hpp"

#include "isocity/RoadGraphExport.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace isocity {

namespace fs = std::filesystem;

namespace {

bool EnsureParentDir(const std::string& path)
{
  fs::path p(path);
  fs::path parent = p.parent_path();
  if (parent.empty()) return true;
  std::error_code ec;
  fs::create_directories(parent, ec);
  return !ec;
}

inline double Clamp01(double v) { return std::max(0.0, std::min(1.0, v)); }

std::string RgbHex(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  std::ostringstream ss;
  ss << "#";
  ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(r);
  ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(g);
  ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(b);
  return ss.str();
}

std::string HeatColor(double t01)
{
  t01 = Clamp01(t01);
  const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * t01), 0, 255));
  const std::uint8_t g = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * (1.0 - t01)), 0, 255));
  const std::uint8_t b = 0;
  return RgbHex(r, g, b);
}

double MaxOrOne(const std::vector<double>& v)
{
  double m = 0.0;
  for (double x : v) m = std::max(m, x);
  return (m > 0.0) ? m : 1.0;
}

bool HasNorm(const std::vector<double>& v, int expectedSize)
{
  return expectedSize > 0 && static_cast<int>(v.size()) == expectedSize;
}

bool HasField(const std::vector<double>& v, int expectedSize)
{
  return expectedSize > 0 && static_cast<int>(v.size()) == expectedSize;
}

template <typename T>
std::vector<int> TopKIndices(const std::vector<T>& values, int k)
{
  std::vector<int> idx;
  const int n = static_cast<int>(values.size());
  if (n <= 0 || k <= 0) return idx;
  k = std::min(k, n);
  idx.resize(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) idx[static_cast<std::size_t>(i)] = i;
  std::nth_element(idx.begin(), idx.begin() + k, idx.end(), [&](int a, int b) {
    if (values[static_cast<std::size_t>(a)] != values[static_cast<std::size_t>(b)])
      return values[static_cast<std::size_t>(a)] > values[static_cast<std::size_t>(b)];
    return a < b;
  });
  idx.resize(static_cast<std::size_t>(k));
  std::sort(idx.begin(), idx.end(), [&](int a, int b) {
    if (values[static_cast<std::size_t>(a)] != values[static_cast<std::size_t>(b)])
      return values[static_cast<std::size_t>(a)] > values[static_cast<std::size_t>(b)];
    return a < b;
  });
  return idx;
}

void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (img.width <= 0 || img.height <= 0) return;
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) +
                           static_cast<std::size_t>(x)) *
                          3u;
  if (idx + 2u >= img.rgb.size()) return;
  img.rgb[idx + 0u] = r;
  img.rgb[idx + 1u] = g;
  img.rgb[idx + 2u] = b;
}

} // namespace


bool WriteRoadGraphCentralityDot(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                 const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                 std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> localComp;
  if (!nodeComponent && cfg.colorByComponent) {
    ComputeRoadGraphComponents(g, localComp);
    nodeComponent = &localComp;
  }

  const int n = static_cast<int>(g.nodes.size());
  const int m = static_cast<int>(g.edges.size());

  const bool nodeNormOk = HasNorm(c.nodeBetweennessNorm, n);
  const bool edgeNormOk = HasNorm(c.edgeBetweennessNorm, m);

  const std::vector<double>& nVal = nodeNormOk ? c.nodeBetweennessNorm : c.nodeBetweenness;
  const std::vector<double>& eVal = edgeNormOk ? c.edgeBetweennessNorm : c.edgeBetweenness;

  const double nMax = MaxOrOne(nVal);
  const double eMax = MaxOrOne(eVal);

  os << "graph G {\n";
  os << "  overlap=false;\n";
  os << "  splines=true;\n";

  // Nodes
  for (int i = 0; i < n; ++i) {
    const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(i)];
    const double v = (i < static_cast<int>(nVal.size())) ? nVal[static_cast<std::size_t>(i)] : 0.0;
    const double t = Clamp01(v / nMax);
    const int deg = static_cast<int>(node.edges.size());
    const int comp = (nodeComponent && i < static_cast<int>(nodeComponent->size()))
                         ? (*nodeComponent)[static_cast<std::size_t>(i)]
                         : -1;

    os << "  " << i << " [label=\"" << i << "\"";
    os << ", tooltip=\"deg=" << deg << "\\nx=" << node.pos.x << " y=" << node.pos.y << "\\ncentrality="
       << std::fixed << std::setprecision(6) << v << "\"";
    os << ", style=filled";
    os << ", fillcolor=\"" << HeatColor(t) << "\"";
    if (cfg.colorByComponent && comp >= 0) {
      // A light outline so components are visible when layouts are messy.
      const std::uint8_t r = static_cast<std::uint8_t>((comp * 53) % 200 + 30);
      const std::uint8_t g0 = static_cast<std::uint8_t>((comp * 97) % 200 + 30);
      const std::uint8_t b = static_cast<std::uint8_t>((comp * 193) % 200 + 30);
      os << ", color=\"" << RgbHex(r, g0, b) << "\"";
    }
    const double w = 0.3 + 0.9 * t;
    os << ", width=" << std::fixed << std::setprecision(2) << w << ", height=" << std::fixed
       << std::setprecision(2) << w;
    os << "];\n";
  }

  // Edges
  for (int ei = 0; ei < m; ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    if (e.a < 0 || e.b < 0) continue;
    if (e.a >= n || e.b >= n) continue;

    const double v = (ei < static_cast<int>(eVal.size())) ? eVal[static_cast<std::size_t>(ei)] : 0.0;
    const double t = Clamp01(v / eMax);

    os << "  " << e.a << " -- " << e.b << " [";
    os << "label=\"c=" << std::fixed << std::setprecision(3) << v << "\\nlen=" << e.length << "\"";
    os << ", tooltip=\"edge=" << ei << "\\ncentrality=" << std::fixed << std::setprecision(6) << v
       << "\\nlen=" << e.length << "\"";
    os << ", color=\"" << HeatColor(t) << "\"";
    const double pw = 1.0 + 4.0 * t;
    os << ", penwidth=" << std::fixed << std::setprecision(2) << pw;
    os << "];\n";
  }

  os << "}\n";

  if (!os.good()) {
    if (outError) *outError = "failed writing centrality DOT";
    return false;
  }
  return true;
}


bool WriteRoadGraphCentralityJson(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                  const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                  std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> localComp;
  if (!nodeComponent) {
    ComputeRoadGraphComponents(g, localComp);
    nodeComponent = &localComp;
  }

  const int n = static_cast<int>(g.nodes.size());
  const int m = static_cast<int>(g.edges.size());

  const bool nodeNormOk = HasNorm(c.nodeBetweennessNorm, n);
  const bool edgeNormOk = HasNorm(c.edgeBetweennessNorm, m);
  const bool closenessOk = HasField(c.nodeCloseness, n);
  const bool harmonicOk = HasField(c.nodeHarmonicCloseness, n);

  os << "{\n";
  os << "  \"centrality\": {\n";
  os << "    \"nodes\": " << n << ",\n";
  os << "    \"edges\": " << m << ",\n";
  os << "    \"sourcesUsed\": " << c.sourcesUsed << ",\n";
  os << "    \"hasNormalized\": " << (nodeNormOk && edgeNormOk ? "true" : "false") << ",\n";
  os << "    \"hasCloseness\": " << (closenessOk ? "true" : "false") << ",\n";
  os << "    \"hasHarmonicCloseness\": " << (harmonicOk ? "true" : "false") << "\n";
  os << "  },\n";

  os << "  \"nodes\": [\n";
  for (int i = 0; i < n; ++i) {
    const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(i)];
    const int deg = static_cast<int>(node.edges.size());
    const int comp = (nodeComponent && i < static_cast<int>(nodeComponent->size()))
                         ? (*nodeComponent)[static_cast<std::size_t>(i)]
                         : -1;

    const double bc = (i < static_cast<int>(c.nodeBetweenness.size())) ? c.nodeBetweenness[static_cast<std::size_t>(i)] : 0.0;
    const double bcn = nodeNormOk ? c.nodeBetweennessNorm[static_cast<std::size_t>(i)] : 0.0;

    os << "    {\"id\": " << i << ", \"x\": " << node.pos.x << ", \"y\": " << node.pos.y
       << ", \"degree\": " << deg << ", \"component\": " << comp;
    os << ", \"betweenness\": " << std::fixed << std::setprecision(9) << bc;
    os << ", \"betweennessNorm\": ";
    if (nodeNormOk) os << std::fixed << std::setprecision(9) << bcn;
    else os << "null";

    os << ", \"closeness\": ";
    if (closenessOk) os << std::fixed << std::setprecision(9) << c.nodeCloseness[static_cast<std::size_t>(i)];
    else os << "null";

    os << ", \"harmonicCloseness\": ";
    if (harmonicOk) os << std::fixed << std::setprecision(9) << c.nodeHarmonicCloseness[static_cast<std::size_t>(i)];
    else os << "null";

    os << "}";
    if (i + 1 != n) os << ',';
    os << "\n";
  }
  os << "  ],\n";

  os << "  \"edges\": [\n";
  for (int ei = 0; ei < m; ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const int comp = (e.a >= 0 && e.a < static_cast<int>(nodeComponent->size()))
                         ? (*nodeComponent)[static_cast<std::size_t>(e.a)]
                         : -1;

    const double bc = (ei < static_cast<int>(c.edgeBetweenness.size())) ? c.edgeBetweenness[static_cast<std::size_t>(ei)] : 0.0;
    const double bcn = edgeNormOk ? c.edgeBetweennessNorm[static_cast<std::size_t>(ei)] : 0.0;

    os << "    {\"id\": " << ei << ", \"a\": " << e.a << ", \"b\": " << e.b << ", \"length\": "
       << e.length << ", \"component\": " << comp;
    os << ", \"betweenness\": " << std::fixed << std::setprecision(9) << bc;
    os << ", \"betweennessNorm\": ";
    if (edgeNormOk) os << std::fixed << std::setprecision(9) << bcn;
    else os << "null";

    if (cfg.includeEdgeTiles) {
      os << ", \"tiles\": [";
      for (std::size_t k = 0; k < e.tiles.size(); ++k) {
        os << '[' << e.tiles[k].x << ',' << e.tiles[k].y << ']';
        if (k + 1 != e.tiles.size()) os << ',';
      }
      os << ']';
    }

    os << "}";
    if (ei + 1 != m) os << ',';
    os << "\n";
  }
  os << "  ]\n";

  os << "}\n";

  if (!os.good()) {
    if (outError) *outError = "failed writing centrality JSON";
    return false;
  }
  return true;
}


bool WriteRoadGraphCentralityNodesCsv(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                      const std::vector<int>* nodeComponent, std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> localComp;
  if (!nodeComponent) {
    ComputeRoadGraphComponents(g, localComp);
    nodeComponent = &localComp;
  }

  const int n = static_cast<int>(g.nodes.size());
  const bool nodeNormOk = HasNorm(c.nodeBetweennessNorm, n);
  const bool closenessOk = HasField(c.nodeCloseness, n);
  const bool harmonicOk = HasField(c.nodeHarmonicCloseness, n);

  os << "id,x,y,degree,component,betweenness,betweenness_norm,closeness,harmonic_closeness\n";
  for (int i = 0; i < n; ++i) {
    const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(i)];
    const int deg = static_cast<int>(node.edges.size());
    const int comp = (i < static_cast<int>(nodeComponent->size())) ? (*nodeComponent)[static_cast<std::size_t>(i)] : -1;
    const double bc = (i < static_cast<int>(c.nodeBetweenness.size())) ? c.nodeBetweenness[static_cast<std::size_t>(i)] : 0.0;

    os << i << ',' << node.pos.x << ',' << node.pos.y << ',' << deg << ',' << comp << ',';
    os << std::fixed << std::setprecision(9) << bc << ',';
    if (nodeNormOk) os << std::fixed << std::setprecision(9) << c.nodeBetweennessNorm[static_cast<std::size_t>(i)];
    os << ',';
    if (closenessOk) os << std::fixed << std::setprecision(9) << c.nodeCloseness[static_cast<std::size_t>(i)];
    os << ',';
    if (harmonicOk) os << std::fixed << std::setprecision(9) << c.nodeHarmonicCloseness[static_cast<std::size_t>(i)];
    os << "\n";
  }

  if (!os.good()) {
    if (outError) *outError = "failed writing centrality nodes CSV";
    return false;
  }
  return true;
}


bool WriteRoadGraphCentralityEdgesCsv(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                      const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                      std::string* outError)
{
  if (outError) outError->clear();

  std::vector<int> localComp;
  if (!nodeComponent) {
    ComputeRoadGraphComponents(g, localComp);
    nodeComponent = &localComp;
  }

  const int m = static_cast<int>(g.edges.size());
  const int n = static_cast<int>(g.nodes.size());
  const bool edgeNormOk = HasNorm(c.edgeBetweennessNorm, m);

  os << "id,a,b,length,component,betweenness,betweenness_norm";
  if (cfg.includeEdgeTiles) os << ",tiles";
  os << "\n";

  for (int ei = 0; ei < m; ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const int comp = (e.a >= 0 && e.a < n && e.a < static_cast<int>(nodeComponent->size()))
                         ? (*nodeComponent)[static_cast<std::size_t>(e.a)]
                         : -1;
    const double bc = (ei < static_cast<int>(c.edgeBetweenness.size())) ? c.edgeBetweenness[static_cast<std::size_t>(ei)] : 0.0;
    os << ei << ',' << e.a << ',' << e.b << ',' << e.length << ',' << comp << ',';
    os << std::fixed << std::setprecision(9) << bc << ',';
    if (edgeNormOk) os << std::fixed << std::setprecision(9) << c.edgeBetweennessNorm[static_cast<std::size_t>(ei)];
    if (cfg.includeEdgeTiles) {
      os << ',';
      for (std::size_t k = 0; k < e.tiles.size(); ++k) {
        os << e.tiles[k].x << ':' << e.tiles[k].y;
        if (k + 1 != e.tiles.size()) os << '|';
      }
    }
    os << "\n";
  }

  if (!os.good()) {
    if (outError) *outError = "failed writing centrality edges CSV";
    return false;
  }
  return true;
}


bool ExportRoadGraphCentralityDot(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                  const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                  std::string* outError)
{
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed creating parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f.is_open()) {
    if (outError) *outError = "failed opening file";
    return false;
  }
  return WriteRoadGraphCentralityDot(f, g, c, nodeComponent, cfg, outError);
}

bool ExportRoadGraphCentralityJson(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                   const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                   std::string* outError)
{
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed creating parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f.is_open()) {
    if (outError) *outError = "failed opening file";
    return false;
  }
  return WriteRoadGraphCentralityJson(f, g, c, nodeComponent, cfg, outError);
}

bool ExportRoadGraphCentralityNodesCsv(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                       const std::vector<int>* nodeComponent, std::string* outError)
{
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed creating parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f.is_open()) {
    if (outError) *outError = "failed opening file";
    return false;
  }
  return WriteRoadGraphCentralityNodesCsv(f, g, c, nodeComponent, outError);
}

bool ExportRoadGraphCentralityEdgesCsv(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                       const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                       std::string* outError)
{
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed creating parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f.is_open()) {
    if (outError) *outError = "failed opening file";
    return false;
  }
  return WriteRoadGraphCentralityEdgesCsv(f, g, c, nodeComponent, cfg, outError);
}


PpmImage RenderRoadGraphCentralityDebugPpm(const World& world, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                          const RoadGraphCentralityVizConfig& cfg)
{
  PpmImage img = RenderPpmLayer(world, cfg.baseLayer);
  if (img.width <= 0 || img.height <= 0) return img;

  const int n = static_cast<int>(g.nodes.size());
  const int m = static_cast<int>(g.edges.size());

  const bool nodeNormOk = HasNorm(c.nodeBetweennessNorm, n);
  const bool edgeNormOk = HasNorm(c.edgeBetweennessNorm, m);

  const std::vector<double>& nVal = nodeNormOk ? c.nodeBetweennessNorm : c.nodeBetweenness;
  const std::vector<double>& eVal = edgeNormOk ? c.edgeBetweennessNorm : c.edgeBetweenness;

  const double nMax = MaxOrOne(nVal);
  const double eMax = MaxOrOne(eVal);

  const std::vector<int> topEdges = TopKIndices(eVal, cfg.topEdges);
  const std::vector<int> topNodes = TopKIndices(nVal, cfg.topNodes);

  // Draw edges first, then nodes on top.
  for (int ei : topEdges) {
    if (ei < 0 || ei >= m) continue;
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    const double v = eVal[static_cast<std::size_t>(ei)];
    const double t = Clamp01(v / eMax);
    const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * t), 0, 255));
    const std::uint8_t g0 = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * (1.0 - t)), 0, 255));

    auto mark = [&](int x, int y) { SetPixel(img, x, y, r, g0, 0); };

    if (cfg.highlightEdgeTiles && !e.tiles.empty()) {
      for (const Point& p : e.tiles) {
        mark(p.x, p.y);
      }
    } else {
      // Endpoints only.
      if (!e.tiles.empty()) {
        mark(e.tiles.front().x, e.tiles.front().y);
        mark(e.tiles.back().x, e.tiles.back().y);
      }
    }
  }

  for (int ni : topNodes) {
    if (ni < 0 || ni >= n) continue;
    const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(ni)];
    const double v = nVal[static_cast<std::size_t>(ni)];
    const double t = Clamp01(v / nMax);

    const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * t), 0, 255));
    const std::uint8_t g0 = 0;
    const std::uint8_t b = static_cast<std::uint8_t>(std::clamp(static_cast<int>(180.0 + 75.0 * t), 0, 255));

    // Small cross so nodes remain visible after scaling.
    const int x = node.pos.x;
    const int y = node.pos.y;
    SetPixel(img, x, y, r, g0, b);
    SetPixel(img, x + 1, y, r, g0, b);
    SetPixel(img, x - 1, y, r, g0, b);
    SetPixel(img, x, y + 1, r, g0, b);
    SetPixel(img, x, y - 1, r, g0, b);
  }

  return img;
}

} // namespace isocity
