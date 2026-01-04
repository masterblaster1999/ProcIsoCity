#include "isocity/TransitPlannerExport.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

struct RGB {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
};

inline bool InBoundsImg(const PpmImage& img, int x, int y)
{
  return x >= 0 && y >= 0 && x < img.width && y < img.height;
}

inline void SetPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (!InBoundsImg(img, x, y)) return;
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) +
                           static_cast<std::size_t>(x)) *
                          3u;
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

// Simple HSV->RGB conversion (all components in [0,1]).
RGB Hsv(double h, double s, double v)
{
  h = h - std::floor(h);
  s = std::clamp(s, 0.0, 1.0);
  v = std::clamp(v, 0.0, 1.0);

  const double c = v * s;
  const double hh = h * 6.0;
  const double x = c * (1.0 - std::abs(std::fmod(hh, 2.0) - 1.0));
  double r1 = 0, g1 = 0, b1 = 0;

  if (0.0 <= hh && hh < 1.0) {
    r1 = c;
    g1 = x;
  } else if (1.0 <= hh && hh < 2.0) {
    r1 = x;
    g1 = c;
  } else if (2.0 <= hh && hh < 3.0) {
    g1 = c;
    b1 = x;
  } else if (3.0 <= hh && hh < 4.0) {
    g1 = x;
    b1 = c;
  } else if (4.0 <= hh && hh < 5.0) {
    r1 = x;
    b1 = c;
  } else {
    r1 = c;
    b1 = x;
  }

  const double m = v - c;
  const auto to8 = [&](double t) -> std::uint8_t {
    const double u = std::clamp(t + m, 0.0, 1.0);
    return static_cast<std::uint8_t>(u * 255.0 + 0.5);
  };

  return RGB{to8(r1), to8(g1), to8(b1)};
}

RGB LineColor(int lineId)
{
  // Golden-ratio hue stepping gives stable, distinct-ish colors.
  constexpr double phi = 0.618033988749895;
  const double h = std::fmod(std::max(0, lineId) * phi, 1.0);
  return Hsv(h, 0.75, 0.95);
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

void WriteJsonPointArray(std::ostream& os, const Point& p)
{
  os << '[' << p.x << ',' << p.y << ']';
}

void WriteGeoJsonPointCoords(std::ostream& os, const Point& p)
{
  // Tile coordinate space; users can interpret these however they want.
  os << '[' << p.x << ',' << p.y << ']';
}

struct StopSet {
  std::vector<Point> points;
  std::vector<int> nodeIds; // -1 for tile-sampled stops
};

StopSet ComputeStops(const RoadGraph& g, const TransitLine& line, const TransitPlanExportConfig& cfg)
{
  StopSet out;

  if (!cfg.includeStops) return out;

  if (cfg.stopMode == TransitStopMode::Nodes) {
    out.points.reserve(line.nodes.size());
    out.nodeIds.reserve(line.nodes.size());
    for (int nid : line.nodes) {
      Point p{};
      if (nid >= 0 && nid < static_cast<int>(g.nodes.size())) p = g.nodes[static_cast<std::size_t>(nid)].pos;
      out.points.push_back(p);
      out.nodeIds.push_back(nid);
    }
  } else {
    std::vector<Point> stops;
    if (!BuildTransitLineStopTiles(g, line, cfg.stopSpacingTiles, stops)) stops.clear();
    out.points = std::move(stops);
    out.nodeIds.assign(out.points.size(), -1);
  }

  // Defensive: collapse consecutive duplicates.
  if (!out.points.empty()) {
    std::vector<Point> uniqPts;
    std::vector<int> uniqIds;
    uniqPts.reserve(out.points.size());
    uniqIds.reserve(out.nodeIds.size());

    for (std::size_t i = 0; i < out.points.size(); ++i) {
      const Point p = out.points[i];
      const int nid = (i < out.nodeIds.size()) ? out.nodeIds[i] : -1;
      if (!uniqPts.empty() && uniqPts.back().x == p.x && uniqPts.back().y == p.y) continue;
      uniqPts.push_back(p);
      uniqIds.push_back(nid);
    }

    out.points = std::move(uniqPts);
    out.nodeIds = std::move(uniqIds);
  }

  return out;
}

} // namespace

bool WriteTransitPlanJson(std::ostream& os, const RoadGraph& g, const TransitPlan& plan,
                          const TransitPlanExportConfig& cfg, std::string* outError)
{
  if (outError) outError->clear();
  if (!os) {
    if (outError) *outError = "invalid output stream";
    return false;
  }

  os << "{\n";
  os << "  \"version\":1,\n";
  os << "  \"weightMode\":\"" << TransitEdgeWeightModeName(plan.cfg.weightMode) << "\",\n";
  os << "  \"stopMode\":\"" << TransitStopModeName(cfg.stopMode) << "\",\n";
  os << "  \"stopSpacingTiles\":" << cfg.stopSpacingTiles << ",\n";
  os << "  \"totalDemand\":" << plan.totalDemand << ",\n";
  os << "  \"coveredDemand\":" << plan.coveredDemand << ",\n";
  os << "  \"lines\":[\n";

  for (std::size_t li = 0; li < plan.lines.size(); ++li) {
    const TransitLine& line = plan.lines[li];

    os << "    {\n";
    os << "      \"id\":" << line.id << ",\n";
    os << "      \"sumDemand\":" << line.sumDemand << ",\n";
    os << "      \"baseCost\":" << line.baseCost << ",\n";

    // Node ids + coords.
    os << "      \"nodes\":[";
    for (std::size_t i = 0; i < line.nodes.size(); ++i) {
      const int nid = line.nodes[i];
      if (i) os << ',';
      Point p{};
      if (nid >= 0 && nid < static_cast<int>(g.nodes.size())) p = g.nodes[static_cast<std::size_t>(nid)].pos;
      os << "{\"id\":" << nid << ",\"x\":" << p.x << ",\"y\":" << p.y << "}";
    }
    os << "]";

    // Optional stops (redundant but handy for some pipelines).
    if (cfg.includeStops) {
      const StopSet stops = ComputeStops(g, line, cfg);
      os << ",\n      \"stops\":[";
      for (std::size_t i = 0; i < stops.points.size(); ++i) {
        if (i) os << ',';
        WriteJsonPointArray(os, stops.points[i]);
      }
      os << "]";
    }

    // Edge indices.
    os << ",\n      \"edges\":[";
    for (std::size_t i = 0; i < line.edges.size(); ++i) {
      if (i) os << ',';
      os << line.edges[i];
    }
    os << "]";

    // Optional tile polyline.
    if (cfg.includeTiles) {
      std::vector<Point> tiles;
      if (!BuildTransitLineTilePolyline(g, line, tiles)) {
        // Keep output valid; just emit empty tiles.
        tiles.clear();
      }
      os << ",\n      \"tiles\":[";
      for (std::size_t i = 0; i < tiles.size(); ++i) {
        if (i) os << ',';
        WriteJsonPointArray(os, tiles[i]);
      }
      os << "]";
    }

    os << "\n    }";
    if (li + 1 < plan.lines.size()) os << ',';
    os << "\n";
  }

  os << "  ]\n";
  os << "}\n";
  return true;
}

bool ExportTransitPlanJson(const std::string& path, const RoadGraph& g, const TransitPlan& plan,
                           const TransitPlanExportConfig& cfg, std::string* outError)
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
  return WriteTransitPlanJson(f, g, plan, cfg, outError);
}

bool WriteTransitPlanGeoJson(std::ostream& os, const RoadGraph& g, const TransitPlan& plan,
                             const TransitPlanExportConfig& cfg, std::string* outError)
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

  for (const TransitLine& line : plan.lines) {
    std::vector<Point> tiles;
    if (!BuildTransitLineTilePolyline(g, line, tiles)) tiles.clear();

    // LineString feature.
    emitComma();
    os << "    {\n";
    os << "      \"type\":\"Feature\",\n";
    os << "      \"properties\":{\"id\":" << line.id
       << ",\"sumDemand\":" << line.sumDemand
       << ",\"baseCost\":" << line.baseCost
       << "},\n";
    os << "      \"geometry\":{\n";
    os << "        \"type\":\"LineString\",\n";
    os << "        \"coordinates\":[";
    for (std::size_t i = 0; i < tiles.size(); ++i) {
      if (i) os << ',';
      WriteGeoJsonPointCoords(os, tiles[i]);
    }
    os << "]\n";
    os << "      }\n";
    os << "    }";

    if (cfg.includeStops) {
      const StopSet stops = ComputeStops(g, line, cfg);
      for (std::size_t si = 0; si < stops.points.size(); ++si) {
        const Point p = stops.points[si];
        const int nid = (si < stops.nodeIds.size()) ? stops.nodeIds[si] : -1;

        emitComma();
        os << "    {\n";
        os << "      \"type\":\"Feature\",\n";
        os << "      \"properties\":{\"lineId\":" << line.id
           << ",\"stop\":" << si
           << ",\"nodeId\":" << nid
           << "},\n";
        os << "      \"geometry\":{\"type\":\"Point\",\"coordinates\":";
        WriteGeoJsonPointCoords(os, p);
        os << "}\n";
        os << "    }";
      }
    }
  }

  os << "\n  ]\n";
  os << "}\n";
  return true;
}

bool ExportTransitPlanGeoJson(const std::string& path, const RoadGraph& g, const TransitPlan& plan,
                              const TransitPlanExportConfig& cfg, std::string* outError)
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
  return WriteTransitPlanGeoJson(f, g, plan, cfg, outError);
}

PpmImage RenderTransitOverlayTile(const World& world, ExportLayer baseLayer, const RoadGraph& g, const TransitPlan& plan,
                                  const TransitPlanExportConfig& cfg)
{
  PpmImage img = RenderPpmLayer(world, baseLayer, nullptr, nullptr, nullptr);
  if (img.width <= 0 || img.height <= 0) return img;

  for (const TransitLine& line : plan.lines) {
    const RGB c = LineColor(line.id);
    std::vector<Point> tiles;
    if (!BuildTransitLineTilePolyline(g, line, tiles)) continue;
    for (const Point& p : tiles) {
      SetPixel(img, p.x, p.y, c.r, c.g, c.b);
    }

    if (cfg.includeStops) {
      const StopSet stops = ComputeStops(g, line, cfg);
      for (const Point& sp : stops.points) {
        SetPixel(img, sp.x, sp.y, 255, 255, 255);
      }
    }
  }

  return img;
}

PpmImage RenderTransitOverlayTile(const World& world, ExportLayer baseLayer, const RoadGraph& g, const TransitPlan& plan,
                                  bool drawStops)
{
  TransitPlanExportConfig cfg{};
  cfg.includeTiles = true;
  cfg.includeStops = drawStops;
  return RenderTransitOverlayTile(world, baseLayer, g, plan, cfg);
}

IsoOverviewResult RenderTransitIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                         const RoadGraph& g, const TransitPlan& plan, const TransitPlanExportConfig& cfg)
{
  IsoOverviewResult iso = RenderIsoOverview(world, baseLayer, isoCfg, nullptr, nullptr, nullptr);
  if (iso.image.width <= 0 || iso.image.height <= 0) return iso;

  auto drawDot = [&](int px, int py, const RGB& c, int radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) continue;
        SetPixel(iso.image, px + dx, py + dy, c.r, c.g, c.b);
      }
    }
  };

  for (const TransitLine& line : plan.lines) {
    const RGB c = LineColor(line.id);
    std::vector<Point> tiles;
    if (!BuildTransitLineTilePolyline(g, line, tiles)) continue;

    for (const Point& p : tiles) {
      int px = 0, py = 0;
      if (!IsoTileCenterToPixel(world, iso, p.x, p.y, px, py)) continue;
      drawDot(px, py, c, 1);
    }

    if (cfg.includeStops) {
      const RGB stop{255, 255, 255};
      const StopSet stops = ComputeStops(g, line, cfg);
      for (const Point& sp : stops.points) {
        int px = 0, py = 0;
        if (!IsoTileCenterToPixel(world, iso, sp.x, sp.y, px, py)) continue;
        drawDot(px, py, stop, 2);
      }
    }
  }

  return iso;
}

IsoOverviewResult RenderTransitIsoOverlay(const World& world, ExportLayer baseLayer, const IsoOverviewConfig& isoCfg,
                                         const RoadGraph& g, const TransitPlan& plan, bool drawStops)
{
  TransitPlanExportConfig cfg{};
  cfg.includeTiles = true;
  cfg.includeStops = drawStops;
  return RenderTransitIsoOverlay(world, baseLayer, isoCfg, g, plan, cfg);
}

} // namespace isocity
