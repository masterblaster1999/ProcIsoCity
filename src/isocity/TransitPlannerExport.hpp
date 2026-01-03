#pragma once

#include "isocity/Export.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/TransitPlanner.hpp"

#include <iosfwd>
#include <string>

namespace isocity {

struct TransitPlanExportConfig {
  // If true, include the full road-tile polyline (a potentially large array) per line.
  bool includeTiles = true;

  // If true, emit stop point features in GeoJSON and stop lists in JSON.
  bool includeStops = true;
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

// Render an isometric overview transit overlay.
//
// baseLayer is usually ExportLayer::Overlay.
IsoOverviewResult RenderTransitIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                         const RoadGraph& g, const TransitPlan& plan, bool drawStops = true);

} // namespace isocity
