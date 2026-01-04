#pragma once

#include "isocity/Export.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/TransitPlanner.hpp"

#include <iosfwd>
#include <string>

namespace isocity {

enum class TransitStopMode : std::uint8_t {
  Nodes = 0, // stop at every RoadGraph node along the line (legacy / dense)
  Tiles = 1, // sample stops along the road-tile polyline
};

inline const char* TransitStopModeName(TransitStopMode m)
{
  switch (m) {
    case TransitStopMode::Nodes:
      return "nodes";
    case TransitStopMode::Tiles:
      return "tiles";
  }
  return "nodes";
}

struct TransitPlanExportConfig {
  // If true, include the full road-tile polyline (a potentially large array) per line.
  bool includeTiles = true;

  // If true, emit stop point features in GeoJSON and stop lists in JSON.
  bool includeStops = true;

  // How stops are emitted when includeStops==true.
  TransitStopMode stopMode = TransitStopMode::Nodes;

  // Used when stopMode==Tiles: sample a stop every N road tiles along the polyline.
  // Endpoints are always included.
  int stopSpacingTiles = 12;
};


// JSON export.
bool WriteTransitPlanJson(std::ostream& os, const RoadGraph& g, const TransitPlan& plan,
                          const TransitPlanExportConfig& cfg = {}, std::string* outError = nullptr);

bool ExportTransitPlanJson(const std::string& path, const RoadGraph& g, const TransitPlan& plan,
                           const TransitPlanExportConfig& cfg = {}, std::string* outError = nullptr);

// GeoJSON export.
//
// Lines are exported as LineString features in tile coordinate space.
// If cfg.includeStops is true, each stop/node along each line is also exported as a Point feature.
bool WriteTransitPlanGeoJson(std::ostream& os, const RoadGraph& g, const TransitPlan& plan,
                             const TransitPlanExportConfig& cfg = {}, std::string* outError = nullptr);

bool ExportTransitPlanGeoJson(const std::string& path, const RoadGraph& g, const TransitPlan& plan,
                              const TransitPlanExportConfig& cfg = {}, std::string* outError = nullptr);


// -----------------------------------------------------------------------------------------------
// Debug visualization
// -----------------------------------------------------------------------------------------------

// Render a per-tile (1px-per-tile) transit overlay image.
//
// baseLayer is usually ExportLayer::Overlay.
PpmImage RenderTransitOverlayTile(const World& world, ExportLayer baseLayer, const RoadGraph& g, const TransitPlan& plan,
                                  bool drawStops = true);

// Configurable variant (supports stop sampling).
PpmImage RenderTransitOverlayTile(const World& world, ExportLayer baseLayer, const RoadGraph& g, const TransitPlan& plan,
                                 const TransitPlanExportConfig& cfg);

// Render an isometric overview transit overlay.
//
// baseLayer is usually ExportLayer::Overlay.
IsoOverviewResult RenderTransitIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                         const RoadGraph& g, const TransitPlan& plan, bool drawStops = true);

// Configurable variant (supports stop sampling).
IsoOverviewResult RenderTransitIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                         const RoadGraph& g, const TransitPlan& plan, const TransitPlanExportConfig& cfg);

} // namespace isocity
