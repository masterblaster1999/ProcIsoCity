#pragma once

#include "isocity/Export.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphCentrality.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace isocity {

// Export/visualization helpers for RoadGraphCentrality.

struct RoadGraphCentralityExportConfig {
  // Include the full per-edge tile polyline in JSON/CSV outputs.
  // This can be large on big maps.
  bool includeEdgeTiles = false;

  // When exporting DOT, color nodes/edges by connected component id.
  // (Centrality-based coloring is still applied; component color becomes a subtle outline.)
  bool colorByComponent = true;
};

// One-pixel-per-tile debug render configuration.
struct RoadGraphCentralityVizConfig {
  ExportLayer baseLayer = ExportLayer::Overlay;

  // How many of the highest-centrality items to highlight.
  int topNodes = 20;
  int topEdges = 30;

  // If true, highlight the full edge polyline (all tiles). Otherwise, only mark endpoints.
  bool highlightEdgeTiles = true;
};

// Write GraphViz DOT (undirected). Returns false on stream failure.
bool WriteRoadGraphCentralityDot(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                 const std::vector<int>* nodeComponent = nullptr,
                                 const RoadGraphCentralityExportConfig& cfg = {}, std::string* outError = nullptr);

// Write JSON. Returns false on stream failure.
bool WriteRoadGraphCentralityJson(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                  const std::vector<int>* nodeComponent = nullptr,
                                  const RoadGraphCentralityExportConfig& cfg = {}, std::string* outError = nullptr);

// Write CSVs.
bool WriteRoadGraphCentralityNodesCsv(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                      const std::vector<int>* nodeComponent = nullptr,
                                      std::string* outError = nullptr);

bool WriteRoadGraphCentralityEdgesCsv(std::ostream& os, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                      const std::vector<int>* nodeComponent = nullptr,
                                      const RoadGraphCentralityExportConfig& cfg = {}, std::string* outError = nullptr);

// Convenience wrappers that write files.
bool ExportRoadGraphCentralityDot(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                  const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                  std::string* outError);

bool ExportRoadGraphCentralityJson(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                   const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                   std::string* outError);

bool ExportRoadGraphCentralityNodesCsv(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                       const std::vector<int>* nodeComponent, std::string* outError);

bool ExportRoadGraphCentralityEdgesCsv(const std::string& path, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                       const std::vector<int>* nodeComponent, const RoadGraphCentralityExportConfig& cfg,
                                       std::string* outError);

// Render a one-pixel-per-tile debug view highlighting the top central nodes/edges.
PpmImage RenderRoadGraphCentralityDebugPpm(const World& world, const RoadGraph& g, const RoadGraphCentralityResult& c,
                                          const RoadGraphCentralityVizConfig& cfg = {});

} // namespace isocity
