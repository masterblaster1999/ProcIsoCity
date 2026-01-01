#include "isocity/RoadGraphTrafficExport.hpp"

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

std::string UtilToColor(double util, double clamp)
{
  if (clamp <= 0.0) clamp = 1.0;
  const double t = Clamp01(util / clamp);
  const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * t), 0, 255));
  const std::uint8_t g = static_cast<std::uint8_t>(std::clamp(static_cast<int>(255.0 * (1.0 - t)), 0, 255));
  const std::uint8_t b = 0;
  return RgbHex(r, g, b);
}

double EdgeUtilForViz(const RoadGraphTrafficEdgeStats& es)
{
  // Prefer interior to avoid double-counting node tiles.
  return (es.interiorTileCount > 0) ? es.maxUtilInterior : es.maxUtilAll;
}

double AvgOrZero(std::uint64_t sum, int denom) { return (denom > 0) ? (static_cast<double>(sum) / static_cast<double>(denom)) : 0.0; }
double AvgOrZero(double sum, int denom) { return (denom > 0) ? (sum / static_cast<double>(denom)) : 0.0; }

} // namespace

bool WriteRoadGraphTrafficDot(std::ostream& os, const RoadGraph& g, const RoadGraphTrafficResult& t,
                              const RoadGraphTrafficExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();

  os << "graph G {\n";
  os << "  overlap=false;\n";
  os << "  splines=true;\n";

  // Nodes
  for (int i = 0; i < static_cast<int>(t.nodes.size()); ++i) {
    const RoadGraphTrafficNodeStats& n = t.nodes[static_cast<std::size_t>(i)];
    os << "  " << i << " [label=\"" << i << "\"";
    os << ", tooltip=\"deg=" << n.degree << "\\ntraffic=" << n.traffic << "\\nutil=" << std::fixed << std::setprecision(3) << n.util
       << "\"";
    os << "];\n";
  }

  // Edges
  for (int i = 0; i < static_cast<int>(t.edges.size()); ++i) {
    const RoadGraphTrafficEdgeStats& es = t.edges[static_cast<std::size_t>(i)];
    if (es.a < 0 || es.b < 0) continue;
    if (es.a >= static_cast<int>(g.nodes.size()) || es.b >= static_cast<int>(g.nodes.size())) continue;

    const double u = EdgeUtilForViz(es);
    const double t01 = Clamp01((cfg.utilColorClamp > 0.0) ? (u / cfg.utilColorClamp) : 0.0);

    os << "  " << es.a << " -- " << es.b << " [";

    // Label: keep it compact.
    if (cfg.labelByUtilization) {
      os << "label=\"u=" << std::fixed << std::setprecision(2) << u << "\\nlen=" << es.length << "\"";
    } else {
      os << "label=\"v=" << es.sumTrafficInterior << "\\nlen=" << es.length << "\"";
    }

    // Tooltip: include more info.
    os << ", tooltip=\"edge=" << i;
    os << "\\nmaxUtilAll=" << std::fixed << std::setprecision(3) << es.maxUtilAll;
    os << "\\nmaxUtilInterior=" << std::fixed << std::setprecision(3) << es.maxUtilInterior;
    os << "\\nsumTrafficAll=" << es.sumTrafficAll;
    os << "\\nsumTrafficInterior=" << es.sumTrafficInterior;
    os << "\\ncongestedInterior=" << es.congestedTilesInterior << "\"";

    if (cfg.colorEdgesByUtilization) {
      os << ", color=\"" << UtilToColor(u, cfg.utilColorClamp) << "\"";
    }
    if (cfg.scalePenWidthByUtilization) {
      const double pw = 1.0 + 4.0 * t01;
      os << ", penwidth=" << std::fixed << std::setprecision(2) << pw;
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

bool ExportRoadGraphTrafficDot(const std::string& path, const RoadGraph& g, const RoadGraphTrafficResult& t,
                               const RoadGraphTrafficExportConfig& cfg, std::string* outError)
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
  return WriteRoadGraphTrafficDot(f, g, t, cfg, outError);
}

bool WriteRoadGraphTrafficJson(std::ostream& os, const RoadGraph& g, const RoadGraphTrafficResult& t, bool includeEdgeTiles,
                               std::string* outError)
{
  if (outError) outError->clear();

  os << "{\n";
  os << "  \"w\": " << t.w << ",\n";
  os << "  \"h\": " << t.h << ",\n";
  os << "  \"capacity\": {\"baseTileCapacity\": " << t.cfg.baseTileCapacity << ", \"useRoadLevelCapacity\": "
     << (t.cfg.useRoadLevelCapacity ? 1 : 0) << "},\n";

  os << "  \"nodes\": [\n";
  for (int i = 0; i < static_cast<int>(t.nodes.size()); ++i) {
    const RoadGraphTrafficNodeStats& n = t.nodes[static_cast<std::size_t>(i)];
    os << "    {\"id\": " << i << ", \"x\": " << n.pos.x << ", \"y\": " << n.pos.y << ", \"degree\": " << n.degree
       << ", \"traffic\": " << n.traffic << ", \"capacity\": " << n.capacity << ", \"util\": " << std::fixed << std::setprecision(6)
       << n.util << ", \"incidentSumTraffic\": " << n.incidentSumTraffic << ", \"incidentMaxUtil\": " << std::fixed
       << std::setprecision(6) << n.incidentMaxUtil << "}";
    if (i + 1 != static_cast<int>(t.nodes.size())) os << ",";
    os << "\n";
  }
  os << "  ],\n";

  os << "  \"edges\": [\n";
  for (int i = 0; i < static_cast<int>(t.edges.size()); ++i) {
    const RoadGraphTrafficEdgeStats& e = t.edges[static_cast<std::size_t>(i)];
    const int allCount = std::max(0, e.tileCount);
    const int intCount = std::max(0, e.interiorTileCount);

    const double avgTrafficAll = AvgOrZero(e.sumTrafficAll, allCount);
    const double avgTrafficInt = AvgOrZero(e.sumTrafficInterior, intCount);
    const double avgCapAll = AvgOrZero(e.sumCapacityAll, allCount);
    const double avgCapInt = AvgOrZero(e.sumCapacityInterior, intCount);
    const double avgUtilAll = AvgOrZero(e.sumUtilAll, allCount);
    const double avgUtilInt = AvgOrZero(e.sumUtilInterior, intCount);

    os << "    {\"id\": " << i << ", \"a\": " << e.a << ", \"b\": " << e.b << ", \"length\": " << e.length << ", \"tileCount\": "
       << e.tileCount << ", \"interiorTileCount\": " << e.interiorTileCount;

    os << ", \"sumTrafficAll\": " << e.sumTrafficAll << ", \"maxTrafficAll\": " << e.maxTrafficAll << ", \"avgTrafficAll\": "
       << std::fixed << std::setprecision(6) << avgTrafficAll;

    os << ", \"sumCapacityAll\": " << e.sumCapacityAll << ", \"minCapacityAll\": " << e.minCapacityAll << ", \"maxCapacityAll\": "
       << e.maxCapacityAll << ", \"avgCapacityAll\": " << std::fixed << std::setprecision(6) << avgCapAll;

    os << ", \"maxUtilAll\": " << std::fixed << std::setprecision(6) << e.maxUtilAll << ", \"avgUtilAll\": " << std::fixed
       << std::setprecision(6) << avgUtilAll;

    os << ", \"congestedTilesAll\": " << e.congestedTilesAll << ", \"excessTrafficAll\": " << e.excessTrafficAll;

    os << ", \"sumTrafficInterior\": " << e.sumTrafficInterior << ", \"maxTrafficInterior\": " << e.maxTrafficInterior
       << ", \"avgTrafficInterior\": " << std::fixed << std::setprecision(6) << avgTrafficInt;

    os << ", \"sumCapacityInterior\": " << e.sumCapacityInterior << ", \"minCapacityInterior\": " << e.minCapacityInterior
       << ", \"maxCapacityInterior\": " << e.maxCapacityInterior << ", \"avgCapacityInterior\": " << std::fixed << std::setprecision(6)
       << avgCapInt;

    os << ", \"maxUtilInterior\": " << std::fixed << std::setprecision(6) << e.maxUtilInterior << ", \"avgUtilInterior\": "
       << std::fixed << std::setprecision(6) << avgUtilInt;

    os << ", \"congestedTilesInterior\": " << e.congestedTilesInterior << ", \"excessTrafficInterior\": " << e.excessTrafficInterior;

    if (includeEdgeTiles) {
      os << ", \"tiles\": [";
      // JSON compactness: [x,y] pairs.
      const RoadGraphEdge* ge = (i >= 0 && i < static_cast<int>(g.edges.size())) ? &g.edges[static_cast<std::size_t>(i)] : nullptr;
      if (ge) {
        for (int ti = 0; ti < static_cast<int>(ge->tiles.size()); ++ti) {
          const Point& p = ge->tiles[static_cast<std::size_t>(ti)];
          os << "[" << p.x << "," << p.y << "]";
          if (ti + 1 != static_cast<int>(ge->tiles.size())) os << ",";
        }
      }
      os << "]";
    }

    os << "}";
    if (i + 1 != static_cast<int>(t.edges.size())) os << ",";
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

bool ExportRoadGraphTrafficJson(const std::string& path, const RoadGraph& g, const RoadGraphTrafficResult& t, bool includeEdgeTiles,
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
  return WriteRoadGraphTrafficJson(f, g, t, includeEdgeTiles, outError);
}

bool ExportRoadGraphTrafficNodesCsv(const std::string& path, const RoadGraphTrafficResult& t, std::string* outError)
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

  // Header
  f << "node_id,x,y,degree,traffic,capacity,util,incident_sumTraffic,incident_maxUtil\n";
  for (int i = 0; i < static_cast<int>(t.nodes.size()); ++i) {
    const RoadGraphTrafficNodeStats& n = t.nodes[static_cast<std::size_t>(i)];
    f << i << "," << n.pos.x << "," << n.pos.y << "," << n.degree << "," << n.traffic << "," << n.capacity << ","
      << std::fixed << std::setprecision(6) << n.util << "," << n.incidentSumTraffic << "," << std::fixed << std::setprecision(6)
      << n.incidentMaxUtil << "\n";
  }

  if (!f.good()) {
    if (outError) *outError = "failed writing nodes CSV";
    return false;
  }
  return true;
}

bool ExportRoadGraphTrafficEdgesCsv(const std::string& path, const RoadGraphTrafficResult& t, std::string* outError)
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

  f << "edge_id,a,b,length,tileCount,interiorTileCount,"
    << "sumTrafficAll,maxTrafficAll,avgTrafficAll,"
    << "sumCapacityAll,minCapacityAll,maxCapacityAll,avgCapacityAll,"
    << "maxUtilAll,avgUtilAll,congestedTilesAll,excessTrafficAll,"
    << "sumTrafficInterior,maxTrafficInterior,avgTrafficInterior,"
    << "sumCapacityInterior,minCapacityInterior,maxCapacityInterior,avgCapacityInterior,"
    << "maxUtilInterior,avgUtilInterior,congestedTilesInterior,excessTrafficInterior\n";

  for (int i = 0; i < static_cast<int>(t.edges.size()); ++i) {
    const RoadGraphTrafficEdgeStats& e = t.edges[static_cast<std::size_t>(i)];
    const int allCount = std::max(0, e.tileCount);
    const int intCount = std::max(0, e.interiorTileCount);

    const double avgTrafficAll = AvgOrZero(e.sumTrafficAll, allCount);
    const double avgTrafficInt = AvgOrZero(e.sumTrafficInterior, intCount);
    const double avgCapAll = AvgOrZero(e.sumCapacityAll, allCount);
    const double avgCapInt = AvgOrZero(e.sumCapacityInterior, intCount);
    const double avgUtilAll = AvgOrZero(e.sumUtilAll, allCount);
    const double avgUtilInt = AvgOrZero(e.sumUtilInterior, intCount);

    f << i << "," << e.a << "," << e.b << "," << e.length << "," << e.tileCount << "," << e.interiorTileCount << ","
      << e.sumTrafficAll << "," << e.maxTrafficAll << "," << std::fixed << std::setprecision(6) << avgTrafficAll << ","
      << e.sumCapacityAll << "," << e.minCapacityAll << "," << e.maxCapacityAll << "," << std::fixed << std::setprecision(6)
      << avgCapAll << ","
      << std::fixed << std::setprecision(6) << e.maxUtilAll << "," << std::fixed << std::setprecision(6) << avgUtilAll << ","
      << e.congestedTilesAll << "," << e.excessTrafficAll << ","
      << e.sumTrafficInterior << "," << e.maxTrafficInterior << "," << std::fixed << std::setprecision(6) << avgTrafficInt << ","
      << e.sumCapacityInterior << "," << e.minCapacityInterior << "," << e.maxCapacityInterior << "," << std::fixed << std::setprecision(6)
      << avgCapInt << ","
      << std::fixed << std::setprecision(6) << e.maxUtilInterior << "," << std::fixed << std::setprecision(6) << avgUtilInt << ","
      << e.congestedTilesInterior << "," << e.excessTrafficInterior << "\n";
  }

  if (!f.good()) {
    if (outError) *outError = "failed writing edges CSV";
    return false;
  }
  return true;
}

} // namespace isocity
