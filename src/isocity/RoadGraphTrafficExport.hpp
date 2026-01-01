#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphTraffic.hpp"

#include <iosfwd>
#include <string>

namespace isocity {

struct RoadGraphTrafficExportConfig {
  // If true, edge label includes "maxUtil", otherwise it includes "sumTraffic".
  bool labelByUtilization = true;

  // If true, color edges by utilization (green -> red).
  bool colorEdgesByUtilization = true;

  // Utilization ratio at which the edge color saturates at "red".
  double utilColorClamp = 2.0;

  // If true, increase penwidth for congested edges.
  bool scalePenWidthByUtilization = true;
};

// DOT (GraphViz) writer. Intended for quick visual inspection in tools like `dot`/`neato`.
bool WriteRoadGraphTrafficDot(std::ostream& os, const RoadGraph& g, const RoadGraphTrafficResult& t,
                              const RoadGraphTrafficExportConfig& cfg = {}, std::string* outError = nullptr);

bool ExportRoadGraphTrafficDot(const std::string& path, const RoadGraph& g, const RoadGraphTrafficResult& t,
                               const RoadGraphTrafficExportConfig& cfg = {}, std::string* outError = nullptr);

// JSON export (single file). If includeEdgeTiles is true, includes a full tile polyline per edge (can be large).
bool WriteRoadGraphTrafficJson(std::ostream& os, const RoadGraph& g, const RoadGraphTrafficResult& t, bool includeEdgeTiles = false,
                               std::string* outError = nullptr);

bool ExportRoadGraphTrafficJson(const std::string& path, const RoadGraph& g, const RoadGraphTrafficResult& t, bool includeEdgeTiles = false,
                                std::string* outError = nullptr);

// Simple CSV exports for analysis pipelines.
bool ExportRoadGraphTrafficNodesCsv(const std::string& path, const RoadGraphTrafficResult& t, std::string* outError = nullptr);
bool ExportRoadGraphTrafficEdgesCsv(const std::string& path, const RoadGraphTrafficResult& t, std::string* outError = nullptr);

} // namespace isocity
