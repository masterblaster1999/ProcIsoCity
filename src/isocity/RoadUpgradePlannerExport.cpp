#include "isocity/RoadUpgradePlannerExport.hpp"

#include "isocity/Json.hpp" // JsonEscape
#include "isocity/Road.hpp" // ClampRoadLevel

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace isocity {

namespace {

struct RGB {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

bool EnsureParentDir(const std::string& path)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::path p(path);
  fs::path parent = p.parent_path();
  if (parent.empty()) return true;
  fs::create_directories(parent, ec);
  return !ec;
}

inline void WriteGeoJsonPointCoords(std::ostream& os, const Point& p)
{
  os << '[' << p.x << ',' << p.y << ']';
}

inline void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) +
                           static_cast<std::size_t>(x)) * 3u;
  if (idx + 2u >= img.rgb.size()) return;
  img.rgb[idx + 0u] = r;
  img.rgb[idx + 1u] = g;
  img.rgb[idx + 2u] = b;
}

RGB LevelColor(int level)
{
  // Colors chosen to read well on top of the base overlay.
  //  - level 2 (Avenue): teal
  //  - level 3 (Highway): warm red/orange
  switch (ClampRoadLevel(level)) {
    case 2: return RGB{40, 220, 200};
    case 3: return RGB{255, 110, 70};
    default: return RGB{255, 255, 255};
  }
}

} // namespace

bool WriteRoadUpgradePlanJson(std::ostream& os, const RoadGraph& g, const RoadUpgradePlan& plan,
                              const RoadUpgradePlanExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!os) {
    if (outError) *outError = "invalid output stream";
    return false;
  }

  os << "{\n";
  os << "  \"version\":1,\n";
  os << "  \"w\":" << plan.w << ",\n";
  os << "  \"h\":" << plan.h << ",\n";
  os << "  \"objective\":\"" << RoadUpgradeObjectiveName(plan.cfg.objective) << "\",\n";
  os << "  \"baseTileCapacity\":" << plan.cfg.baseTileCapacity << ",\n";
  os << "  \"useRoadLevelCapacity\":" << (plan.cfg.useRoadLevelCapacity ? "true" : "false") << ",\n";
  os << "  \"upgradeEndpoints\":" << (plan.cfg.upgradeEndpoints ? "true" : "false") << ",\n";
  os << "  \"maxTargetLevel\":" << plan.cfg.maxTargetLevel << ",\n";
  os << "  \"minUtilConsider\":" << plan.cfg.minUtilConsider << ",\n";
  os << "  \"budget\":" << plan.cfg.budget << ",\n";
  if (plan.cfg.objective == RoadUpgradeObjective::Hybrid) {
    os << "  \"hybridExcessWeight\":" << plan.cfg.hybridExcessWeight << ",\n";
    os << "  \"hybridTimeWeight\":" << plan.cfg.hybridTimeWeight << ",\n";
  }

  os << "  \"totalCost\":" << plan.totalCost << ",\n";
  os << "  \"totalTimeSaved\":" << static_cast<unsigned long long>(plan.totalTimeSaved) << ",\n";
  os << "  \"totalExcessReduced\":" << static_cast<unsigned long long>(plan.totalExcessReduced) << ",\n";

  os << "  \"edges\":[\n";
  for (std::size_t i = 0; i < plan.edges.size(); ++i) {
    const RoadUpgradeEdge& e = plan.edges[i];
    os << "    {\n";
    os << "      \"edgeIndex\":" << e.edgeIndex << ",\n";
    os << "      \"a\":" << e.a << ",\n";
    os << "      \"b\":" << e.b << ",\n";

    // Node tile positions (helpful even without tiles).
    Point pa{};
    Point pb{};
    if (e.a >= 0 && e.a < static_cast<int>(g.nodes.size())) pa = g.nodes[static_cast<std::size_t>(e.a)].pos;
    if (e.b >= 0 && e.b < static_cast<int>(g.nodes.size())) pb = g.nodes[static_cast<std::size_t>(e.b)].pos;
    os << "      \"aPos\":{\"x\":" << pa.x << ",\"y\":" << pa.y << "},\n";
    os << "      \"bPos\":{\"x\":" << pb.x << ",\"y\":" << pb.y << "},\n";

    os << "      \"targetLevel\":" << e.targetLevel << ",\n";
    os << "      \"cost\":" << e.cost << ",\n";
    os << "      \"timeSaved\":" << static_cast<unsigned long long>(e.timeSaved) << ",\n";
    os << "      \"excessReduced\":" << static_cast<unsigned long long>(e.excessReduced) << ",\n";
    os << "      \"tileCount\":" << e.tileCount;

    if (cfg.includeEdgeTiles && e.edgeIndex >= 0 && e.edgeIndex < static_cast<int>(g.edges.size())) {
      const RoadGraphEdge& ge = g.edges[static_cast<std::size_t>(e.edgeIndex)];
      os << ",\n      \"tiles\":[";
      bool first = true;
      for (const Point& tp : ge.tiles) {
        if (!first) os << ',';
        first = false;
        os << "[" << tp.x << "," << tp.y << "]";
      }
      os << "]";
    }

    os << "\n    }";
    if (i + 1u < plan.edges.size()) os << ',';
    os << "\n";
  }
  os << "  ]";

  if (cfg.includeTileUpgrades) {
    os << ",\n  \"tileUpgrades\":[";
    const int w = plan.w;
    const int h = plan.h;
    bool first = true;
    if (w > 0 && h > 0 && static_cast<int>(plan.tileTargetLevel.size()) == w * h) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          const int lvl = static_cast<int>(plan.tileTargetLevel[idx]);
          if (lvl <= 0) continue;
          if (!first) os << ',';
          first = false;
          os << "{\"x\":" << x << ",\"y\":" << y << ",\"to\":" << lvl << "}";
        }
      }
    }
    os << "]";
  }

  os << "\n}\n";
  return true;
}

bool ExportRoadUpgradePlanJson(const std::string& path, const RoadGraph& g, const RoadUpgradePlan& plan,
                               const RoadUpgradePlanExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create output directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open output file";
    return false;
  }
  return WriteRoadUpgradePlanJson(f, g, plan, cfg, outError);
}


bool WriteRoadUpgradePlanGeoJson(std::ostream& os, const RoadGraph& g, const RoadUpgradePlan& plan,
                                 const RoadUpgradePlanExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!os) {
    if (outError) *outError = "invalid output stream";
    return false;
  }

  os << "{\n";
  os << "  \"type\":\"FeatureCollection\",\n";
  os << "  \"features\":[\n";

  bool firstFeature = true;
  auto emitComma = [&]() {
    if (!firstFeature) os << ",\n";
    firstFeature = false;
  };

  // Upgraded edges.
  for (const RoadUpgradeEdge& e : plan.edges) {
    if (e.edgeIndex < 0 || e.edgeIndex >= static_cast<int>(g.edges.size())) continue;
    const RoadGraphEdge& ge = g.edges[static_cast<std::size_t>(e.edgeIndex)];

    emitComma();
    os << "    {\n";
    os << "      \"type\":\"Feature\",\n";
    os << "      \"properties\":{"
       << "\"edgeIndex\":" << e.edgeIndex
       << ",\"targetLevel\":" << e.targetLevel
       << ",\"cost\":" << e.cost
       << ",\"timeSaved\":" << static_cast<unsigned long long>(e.timeSaved)
       << ",\"excessReduced\":" << static_cast<unsigned long long>(e.excessReduced)
       << "},\n";
    os << "      \"geometry\":{\n";
    os << "        \"type\":\"LineString\",\n";
    os << "        \"coordinates\":[";
    for (std::size_t i = 0; i < ge.tiles.size(); ++i) {
      if (i) os << ',';
      WriteGeoJsonPointCoords(os, ge.tiles[i]);
    }
    os << "]\n";
    os << "      }\n";
    os << "    }";
  }

  // Optional per-tile points.
  if (cfg.includeTileUpgrades) {
    const int w = plan.w;
    const int h = plan.h;
    if (w > 0 && h > 0 && static_cast<int>(plan.tileTargetLevel.size()) == w * h) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          const int lvl = static_cast<int>(plan.tileTargetLevel[idx]);
          if (lvl <= 0) continue;

          emitComma();
          os << "    {\n";
          os << "      \"type\":\"Feature\",\n";
          os << "      \"properties\":{\"to\":" << lvl << "},\n";
          os << "      \"geometry\":{\"type\":\"Point\",\"coordinates\":";
          WriteGeoJsonPointCoords(os, Point{x, y});
          os << "}\n";
          os << "    }";
        }
      }
    }
  }

  os << "\n  ]\n";
  os << "}\n";
  return true;
}

bool ExportRoadUpgradePlanGeoJson(const std::string& path, const RoadGraph& g, const RoadUpgradePlan& plan,
                                  const RoadUpgradePlanExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create output directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open output file";
    return false;
  }
  return WriteRoadUpgradePlanGeoJson(f, g, plan, cfg, outError);
}


PpmImage RenderRoadUpgradeOverlayTile(const World& world, ExportLayer baseLayer, const RoadUpgradePlan& plan)
{
  PpmImage img = RenderPpmLayer(world, baseLayer, nullptr, nullptr, nullptr);
  if (img.width <= 0 || img.height <= 0) return img;
  const int w = plan.w;
  const int h = plan.h;
  if (w <= 0 || h <= 0) return img;
  if (w != img.width || h != img.height) return img;
  if (static_cast<int>(plan.tileTargetLevel.size()) != w * h) return img;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      const int lvl = static_cast<int>(plan.tileTargetLevel[idx]);
      if (lvl <= 0) continue;
      const RGB c = LevelColor(lvl);
      SetPixel(img, x, y, c.r, c.g, c.b);
    }
  }

  return img;
}


IsoOverviewResult RenderRoadUpgradeIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                              const RoadUpgradePlan& plan)
{
  IsoOverviewResult iso = RenderIsoOverview(world, baseLayer, isoCfg, nullptr, nullptr, nullptr);
  if (iso.image.width <= 0 || iso.image.height <= 0) return iso;
  const int w = plan.w;
  const int h = plan.h;
  if (w <= 0 || h <= 0) return iso;
  if (static_cast<int>(plan.tileTargetLevel.size()) != w * h) return iso;

  auto drawDot = [&](int px, int py, const RGB& c, int radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) continue;
        SetPixel(iso.image, px + dx, py + dy, c.r, c.g, c.b);
      }
    }
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      const int lvl = static_cast<int>(plan.tileTargetLevel[idx]);
      if (lvl <= 0) continue;
      int px = 0, py = 0;
      if (!IsoTileCenterToPixel(world, iso, x, y, px, py)) continue;
      const RGB c = LevelColor(lvl);
      drawDot(px, py, c, 2);
    }
  }

  return iso;
}

} // namespace isocity
