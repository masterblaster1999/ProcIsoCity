#pragma once

#include "isocity/Export.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadUpgradePlanner.hpp"

#include <iosfwd>
#include <string>

namespace isocity {

inline const char* RoadUpgradeObjectiveName(RoadUpgradeObjective obj)
{
  switch (obj) {
    case RoadUpgradeObjective::Congestion: return "congestion";
    case RoadUpgradeObjective::Time: return "time";
    case RoadUpgradeObjective::Hybrid: return "hybrid";
  }
  return "congestion";
}

struct RoadUpgradePlanExportConfig {
  // If true, include a list of upgraded tiles in the JSON/GeoJSON output.
  // (This is NOT the full w*h array; only tiles with targetLevel>0.)
  bool includeTileUpgrades = true;

  // If true, include the full road-tile polyline per upgraded edge.
  bool includeEdgeTiles = true;
};


// JSON export.
bool WriteRoadUpgradePlanJson(std::ostream& os, const RoadGraph& g, const RoadUpgradePlan& plan,
                              const RoadUpgradePlanExportConfig& cfg = {},
                              std::string* outError = nullptr);

bool ExportRoadUpgradePlanJson(const std::string& path, const RoadGraph& g, const RoadUpgradePlan& plan,
                               const RoadUpgradePlanExportConfig& cfg = {},
                               std::string* outError = nullptr);


// GeoJSON export.
// - Each upgraded edge is emitted as a LineString feature in tile coordinate space.
// - Optionally, each upgraded tile is also emitted as a Point feature.
bool WriteRoadUpgradePlanGeoJson(std::ostream& os, const RoadGraph& g, const RoadUpgradePlan& plan,
                                 const RoadUpgradePlanExportConfig& cfg = {},
                                 std::string* outError = nullptr);

bool ExportRoadUpgradePlanGeoJson(const std::string& path, const RoadGraph& g, const RoadUpgradePlan& plan,
                                  const RoadUpgradePlanExportConfig& cfg = {},
                                  std::string* outError = nullptr);


// -----------------------------------------------------------------------------------------------
// Debug visualization
// -----------------------------------------------------------------------------------------------

// Render a per-tile (1px-per-tile) road upgrade overlay image.
//
// baseLayer is usually ExportLayer::Overlay.
PpmImage RenderRoadUpgradeOverlayTile(const World& world, ExportLayer baseLayer, const RoadUpgradePlan& plan);

// Render an isometric overview with road upgrades overlaid.
IsoOverviewResult RenderRoadUpgradeIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                              const RoadUpgradePlan& plan);

} // namespace isocity
