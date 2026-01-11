#include "isocity/TransitPlannerExport.hpp"

#include "isocity/Geometry.hpp"
#include "isocity/Json.hpp"

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

void WriteJsonPointArray(JsonWriter& jw, const Point& p)
{
  jw.beginArray();
  jw.intValue(p.x);
  jw.intValue(p.y);
  jw.endArray();
}

void WriteGeoJsonTileCenterCoords(JsonWriter& jw, const Point& p)
{
  // Tile-center coordinate space (x+0.5, y+0.5) so the output aligns with mapexport road centerlines.
  jw.beginArray();
  jw.numberValue(static_cast<double>(p.x) + 0.5);
  jw.numberValue(static_cast<double>(p.y) + 0.5);
  jw.endArray();
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

  JsonWriteOptions jopt{};
  jopt.pretty = true;
  jopt.indent = 2;
  jopt.sortKeys = false;

  JsonWriter jw(os, jopt);

  jw.beginObject();

  jw.key("version");
  jw.intValue(1);

  jw.key("weightMode");
  jw.stringValue(TransitEdgeWeightModeName(plan.cfg.weightMode));

  jw.key("stopMode");
  jw.stringValue(TransitStopModeName(cfg.stopMode));

  jw.key("stopSpacingTiles");
  jw.intValue(cfg.stopSpacingTiles);

  jw.key("totalDemand");
  jw.uintValue(plan.totalDemand);

  jw.key("coveredDemand");
  jw.uintValue(plan.coveredDemand);

  jw.key("lines");
  jw.beginArray();

  for (const TransitLine& line : plan.lines) {
    jw.beginObject();

    jw.key("id");
    jw.intValue(line.id);

    jw.key("sumDemand");
    jw.uintValue(line.sumDemand);

    jw.key("baseCost");
    jw.uintValue(line.baseCost);

    // Node ids + coords.
    jw.key("nodes");
    jw.beginArray();
    for (int nid : line.nodes) {
      Point p{};
      if (nid >= 0 && nid < static_cast<int>(g.nodes.size())) p = g.nodes[static_cast<std::size_t>(nid)].pos;

      jw.beginObject();
      jw.key("id");
      jw.intValue(nid);
      jw.key("x");
      jw.intValue(p.x);
      jw.key("y");
      jw.intValue(p.y);
      jw.endObject();
    }
    jw.endArray();

    // Optional stops (redundant but handy for some pipelines).
    if (cfg.includeStops) {
      const StopSet stops = ComputeStops(g, line, cfg);
      jw.key("stops");
      jw.beginArray();
      for (const Point& p : stops.points) {
        WriteJsonPointArray(jw, p);
      }
      jw.endArray();
    }

    // Edge indices.
    jw.key("edges");
    jw.beginArray();
    for (int eidx : line.edges) {
      jw.intValue(eidx);
    }
    jw.endArray();

    // Optional tile polyline.
    if (cfg.includeTiles) {
      std::vector<Point> tiles;
      if (!BuildTransitLineTilePolyline(g, line, tiles)) tiles.clear();

      jw.key("tiles");
      jw.beginArray();
      for (const Point& p : tiles) {
        WriteJsonPointArray(jw, p);
      }
      jw.endArray();
    }

    jw.endObject();
  }

  jw.endArray();  // lines
  jw.endObject(); // root

  if (!jw.ok() || !os.good()) {
    if (outError) *outError = jw.error().empty() ? "failed to write JSON" : jw.error();
    return false;
  }

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

  JsonWriteOptions jopt{};
  jopt.pretty = true;
  jopt.indent = 2;
  jopt.sortKeys = false;

  JsonWriter jw(os, jopt);

  auto writeLineCoords = [&](const std::vector<Point>& pts) {
    jw.beginArray();
    for (const Point& p : pts) {
      WriteGeoJsonTileCenterCoords(jw, p);
    }
    jw.endArray();
  };

  jw.beginObject();
  jw.key("type");
  jw.stringValue("FeatureCollection");

  jw.key("properties");
  jw.beginObject();
  jw.key("coordSpace");
  jw.stringValue("tile_center");
  jw.key("weightMode");
  jw.stringValue(TransitEdgeWeightModeName(plan.cfg.weightMode));
  jw.key("stopMode");
  jw.stringValue(TransitStopModeName(cfg.stopMode));
  jw.key("stopSpacingTiles");
  jw.intValue(cfg.stopSpacingTiles);
  jw.endObject();

  jw.key("features");
  jw.beginArray();

  for (const TransitLine& line : plan.lines) {
    std::vector<Point> tiles;
    if (!BuildTransitLineTilePolyline(g, line, tiles)) tiles.clear();

    // For visualization, reduce vertex count (does not change geometry).
    std::vector<Point> simplified = tiles;
    SimplifyPolylineCollinear(simplified);

    // LineString feature.
    jw.beginObject();
    jw.key("type");
    jw.stringValue("Feature");

    jw.key("properties");
    jw.beginObject();
    jw.key("layer");
    jw.stringValue("transit_line");
    jw.key("id");
    jw.intValue(line.id);
    jw.key("sumDemand");
    jw.uintValue(line.sumDemand);
    jw.key("baseCost");
    jw.uintValue(line.baseCost);
    jw.key("tiles");
    jw.intValue(static_cast<int>(tiles.size()));
    jw.key("points");
    jw.intValue(static_cast<int>(simplified.size()));
    jw.endObject();

    jw.key("geometry");
    jw.beginObject();
    jw.key("type");
    jw.stringValue("LineString");
    jw.key("coordinates");
    writeLineCoords(simplified);
    jw.endObject();

    jw.endObject();

    if (cfg.includeStops) {
      const StopSet stops = ComputeStops(g, line, cfg);
      for (std::size_t si = 0; si < stops.points.size(); ++si) {
        const Point p = stops.points[si];
        const int nid = (si < stops.nodeIds.size()) ? stops.nodeIds[si] : -1;

        jw.beginObject();
        jw.key("type");
        jw.stringValue("Feature");

        jw.key("properties");
        jw.beginObject();
        jw.key("layer");
        jw.stringValue("transit_stop");
        jw.key("lineId");
        jw.intValue(line.id);
        jw.key("stop");
        jw.intValue(static_cast<int>(si));
        jw.key("nodeId");
        jw.intValue(nid);
        jw.endObject();

        jw.key("geometry");
        jw.beginObject();
        jw.key("type");
        jw.stringValue("Point");
        jw.key("coordinates");
        WriteGeoJsonTileCenterCoords(jw, p);
        jw.endObject();

        jw.endObject();
      }
    }
  }

  jw.endArray();
  jw.endObject();

  if (!jw.ok() || !os.good()) {
    if (outError) *outError = jw.error().empty() ? "failed to write GeoJSON" : jw.error();
    return false;
  }

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
