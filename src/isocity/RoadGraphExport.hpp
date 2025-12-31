#pragma once

#include "isocity/Export.hpp"
#include "isocity/RoadGraph.hpp"

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace isocity {

// Utilities for analyzing and exporting a compressed RoadGraph.
//
// Primary uses:
//  - debugging disconnected road networks / "outside connection" issues
//  - exporting road graphs to external tools (GraphViz, Python, etc.)
//  - generating deterministic regression artifacts for CI

struct RoadGraphMetrics {
  int nodes = 0;
  int edges = 0;

  // Sum of RoadGraphEdge::length over all edges.
  std::uint64_t totalEdgeLength = 0;

  // Graph connectivity (node-space, undirected).
  int components = 0;
  int largestComponentNodes = 0;
  int largestComponentEdges = 0;
  int isolatedNodes = 0;

  // Simple averages.
  double avgDegree = 0.0;
  double avgEdgeLength = 0.0;

  // Approximate diameter over the road graph using a deterministic double-Dijkstra.
  // Distances are weighted by RoadGraphEdge::length.
  int approxDiameter = 0;
  int diameterA = -1;
  int diameterB = -1;
};

// Deterministic approximate diameter result (includes a path).
//
// The returned nodePath is inclusive of endpoints; edgePath has size nodePath.size()-1.
// If the graph is empty, all fields are defaulted.
struct RoadGraphDiameter {
  int a = -1;
  int b = -1;
  int distance = 0;
  std::vector<int> nodePath;
  std::vector<int> edgePath;
};

// Compute high-level graph metrics.
RoadGraphMetrics ComputeRoadGraphMetrics(const RoadGraph& g);

// Compute an approximate weighted diameter and return a node/edge path.
RoadGraphDiameter ComputeApproxRoadGraphDiameter(const RoadGraph& g);

// Expand a node-path to a tile polyline by stitching the underlying RoadGraphEdge::tiles.
// Returns false if the node path is invalid or an edge cannot be found.
bool ExpandRoadGraphNodePathToTiles(const RoadGraph& g, const std::vector<int>& nodePath, std::vector<Point>& outTiles);


// -----------------------------------------------------------------------------------------------
// Export formats
// -----------------------------------------------------------------------------------------------

struct RoadGraphExportConfig {
  // Include the full per-edge tile polyline in JSON/CSV/DOT outputs.
  // This can be large on big maps.
  bool includeEdgeTiles = false;

  // When exporting DOT, color nodes by connected component.
  bool colorByComponent = true;
};

// Compute the connected component id for each node.
// Returns the number of components.
int ComputeRoadGraphComponents(const RoadGraph& g, std::vector<int>& outNodeComponent);

// Write GraphViz DOT (undirected). Returns false on stream failure.
bool WriteRoadGraphDot(std::ostream& os, const RoadGraph& g, const RoadGraphMetrics* metrics = nullptr,
                       const RoadGraphExportConfig& cfg = {}, std::string* outError = nullptr);

// Write JSON. Returns false on stream failure.
bool WriteRoadGraphJson(std::ostream& os, const RoadGraph& g, const RoadGraphMetrics* metrics = nullptr,
                        const RoadGraphDiameter* diameter = nullptr, const RoadGraphExportConfig& cfg = {},
                        std::string* outError = nullptr);

// Write CSVs.
bool WriteRoadGraphNodesCsv(std::ostream& os, const RoadGraph& g, const std::vector<int>* nodeComponent = nullptr,
                            std::string* outError = nullptr);

bool WriteRoadGraphEdgesCsv(std::ostream& os, const RoadGraph& g, const std::vector<int>* nodeComponent = nullptr,
                            const RoadGraphExportConfig& cfg = {}, std::string* outError = nullptr);


// Convenience wrappers that write files.
bool ExportRoadGraphDot(const std::string& path, const RoadGraph& g, const RoadGraphMetrics* metrics,
                        const RoadGraphExportConfig& cfg, std::string* outError);

bool ExportRoadGraphJson(const std::string& path, const RoadGraph& g, const RoadGraphMetrics* metrics,
                         const RoadGraphDiameter* diameter, const RoadGraphExportConfig& cfg, std::string* outError);

bool ExportRoadGraphNodesCsv(const std::string& path, const RoadGraph& g, const std::vector<int>* nodeComponent,
                             std::string* outError);

bool ExportRoadGraphEdgesCsv(const std::string& path, const RoadGraph& g, const std::vector<int>* nodeComponent,
                             const RoadGraphExportConfig& cfg, std::string* outError);

// Render a one-pixel-per-tile debug view of a road graph.
// - baseLayer: the base RenderPpmLayer layer to start from (usually Overlay or Terrain).
// - If highlightTiles is non-null, those tiles are colored on top (useful for diameter paths).
PpmImage RenderRoadGraphDebugPpm(const World& world, const RoadGraph& g, ExportLayer baseLayer,
                                const std::vector<Point>* highlightTiles = nullptr);

} // namespace isocity
