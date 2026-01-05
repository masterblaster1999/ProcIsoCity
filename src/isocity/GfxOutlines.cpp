#include "isocity/GfxOutlines.hpp"

#include "isocity/Json.hpp"      // JsonEscape

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace isocity {

namespace {

inline std::uint8_t ClampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<std::uint8_t>(v);
}

inline std::uint8_t AlphaThresholdU8(float t)
{
  if (!(t >= 0.0f)) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const int v = static_cast<int>(t * 255.0f + 0.5f);
  return ClampU8(v);
}

// Monotonic chain convex hull.
// Returns a closed ring (first == last) for convenience.
std::vector<IPoint> ConvexHull(std::vector<IPoint> pts)
{
  if (pts.empty()) return {};

  std::sort(pts.begin(), pts.end(), [](const IPoint& a, const IPoint& b) {
    if (a.x != b.x) return a.x < b.x;
    return a.y < b.y;
  });

  pts.erase(std::unique(pts.begin(), pts.end(), [](const IPoint& a, const IPoint& b) {
    return a.x == b.x && a.y == b.y;
  }), pts.end());

  if (pts.size() == 1) {
    std::vector<IPoint> ring;
    ring.push_back(pts.front());
    ring.push_back(pts.front());
    return ring;
  }

  auto cross = [](const IPoint& o, const IPoint& a, const IPoint& b) -> long long {
    const long long ax = static_cast<long long>(a.x) - static_cast<long long>(o.x);
    const long long ay = static_cast<long long>(a.y) - static_cast<long long>(o.y);
    const long long bx = static_cast<long long>(b.x) - static_cast<long long>(o.x);
    const long long by = static_cast<long long>(b.y) - static_cast<long long>(o.y);
    return ax * by - ay * bx;
  };

  std::vector<IPoint> H;
  H.reserve(pts.size() * 2);

  // Lower hull
  for (const IPoint& p : pts) {
    while (H.size() >= 2 && cross(H[H.size() - 2], H[H.size() - 1], p) <= 0) {
      H.pop_back();
    }
    H.push_back(p);
  }

  // Upper hull
  const std::size_t lowerSize = H.size();
  for (std::size_t i = pts.size(); i-- > 0;) {
    const IPoint& p = pts[i];
    while (H.size() > lowerSize && cross(H[H.size() - 2], H[H.size() - 1], p) <= 0) {
      H.pop_back();
    }
    H.push_back(p);
  }

  if (!H.empty()) H.pop_back(); // last == first
  if (!H.empty()) H.push_back(H.front());
  return H;
}

void OffsetRing(std::vector<IPoint>& ring, int dx, int dy)
{
  for (IPoint& p : ring) {
    p.x += dx;
    p.y += dy;
  }
}

void OffsetGeom(VectorMultiPolygon& g, int dx, int dy)
{
  for (auto& poly : g.polygons) {
    OffsetRing(poly.outer, dx, dy);
    for (auto& hole : poly.holes) {
      OffsetRing(hole, dx, dy);
    }
  }
}

bool ExtractAlphaMask(const RgbaImage& atlas, const GfxAtlasEntry& e, std::vector<std::uint8_t>& outAlpha,
                      std::string& outError)
{
  outError.clear();
  outAlpha.clear();

  if (atlas.width <= 0 || atlas.height <= 0 || atlas.rgba.empty()) {
    outError = "atlas image is empty";
    return false;
  }
  if (e.w <= 0 || e.h <= 0) {
    outError = "sprite has invalid size";
    return false;
  }
  if (e.x < 0 || e.y < 0 || e.x + e.w > atlas.width || e.y + e.h > atlas.height) {
    outError = "sprite rect out of atlas bounds";
    return false;
  }

  outAlpha.resize(static_cast<std::size_t>(e.w) * static_cast<std::size_t>(e.h));
  for (int y = 0; y < e.h; ++y) {
    const int ay = e.y + y;
    for (int x = 0; x < e.w; ++x) {
      const int ax = e.x + x;
      const std::size_t srcIdx = (static_cast<std::size_t>(ay) * static_cast<std::size_t>(atlas.width) +
                                  static_cast<std::size_t>(ax)) *
                                 4u;
      const std::size_t dstIdx = static_cast<std::size_t>(y) * static_cast<std::size_t>(e.w) + static_cast<std::size_t>(x);
      outAlpha[dstIdx] = atlas.rgba[srcIdx + 3u];
    }
  }

  return true;
}

void WriteJsonPointArray(std::ostream& os, const std::vector<IPoint>& ring)
{
  os << "[";
  for (std::size_t i = 0; i < ring.size(); ++i) {
    os << "[" << ring[i].x << "," << ring[i].y << "]";
    if (i + 1 < ring.size()) os << ",";
  }
  os << "]";
}

void WriteSvgRingPath(std::ostream& os, const std::vector<IPoint>& ring)
{
  if (ring.size() < 4) return;
  // Ring is closed; skip the final repeated point.
  os << "M " << ring[0].x << " " << ring[0].y;
  for (std::size_t i = 1; i + 1 < ring.size(); ++i) {
    os << " L " << ring[i].x << " " << ring[i].y;
  }
  os << " Z ";
}

} // namespace

bool ComputeGfxSpriteOutline(const RgbaImage& atlas, const GfxAtlasEntry& entry,
                            const GfxOutlineConfig& cfg,
                            GfxSpriteOutline& out,
                            std::string& outError)
{
  outError.clear();
  out = GfxSpriteOutline{};

  out.name = entry.name;
  out.atlasX = entry.x;
  out.atlasY = entry.y;
  out.w = entry.w;
  out.h = entry.h;
  out.srcW = entry.srcW > 0 ? entry.srcW : entry.w;
  out.srcH = entry.srcH > 0 ? entry.srcH : entry.h;
  out.trimX = entry.trimX;
  out.trimY = entry.trimY;

  std::vector<std::uint8_t> alpha;
  std::string err;
  if (!ExtractAlphaMask(atlas, entry, alpha, err)) {
    outError = entry.name + ": " + err;
    return false;
  }

  const std::uint8_t thr = AlphaThresholdU8(cfg.alphaThreshold);

  // Build a binary label grid: 1 = inside, 0 = background.
  std::vector<int> labels;
  labels.resize(alpha.size());
  for (std::size_t i = 0; i < alpha.size(); ++i) {
    labels[i] = (alpha[i] >= thr) ? 1 : 0;
  }

  std::vector<LabeledGeometry> geoms;
  VectorizeStats stats;
  std::string vErr;
  if (!VectorizeLabelGridToPolygons(labels, entry.w, entry.h, /*backgroundLabel*/ 0, geoms, &stats, &vErr)) {
    outError = entry.name + ": vectorize failed: " + vErr;
    return false;
  }

  // Locate label=1 geometry.
  VectorMultiPolygon mp;
  for (const auto& lg : geoms) {
    if (lg.label == 1) {
      mp = lg.geom;
      break;
    }
  }

  if (!cfg.includeHoles) {
    for (auto& poly : mp.polygons) poly.holes.clear();
  }

  // Offset geometry into the logical canvas coordinate system.
  OffsetGeom(mp, entry.trimX, entry.trimY);
  out.geom = std::move(mp);

  if (cfg.computeConvexHull) {
    std::vector<IPoint> pts;
    for (const auto& poly : out.geom.polygons) {
      if (poly.outer.size() < 4) continue;
      for (std::size_t i = 0; i + 1 < poly.outer.size(); ++i) {
        pts.push_back(poly.outer[i]);
      }
    }
    out.hull = ConvexHull(std::move(pts));
  }

  return true;
}

bool ComputeGfxTilesetOutlines(const GfxTilesetResult& ts,
                              const GfxOutlineConfig& cfg,
                              std::vector<GfxSpriteOutline>& out,
                              std::string& outError)
{
  outError.clear();
  out.clear();

  out.reserve(ts.entries.size());
  for (const auto& e : ts.entries) {
    GfxSpriteOutline o;
    std::string err;
    if (!ComputeGfxSpriteOutline(ts.atlas, e, cfg, o, err)) {
      outError = err;
      return false;
    }
    out.push_back(std::move(o));
  }

  return true;
}

bool WriteGfxTilesetOutlinesJson(const std::string& path,
                                const GfxTilesetResult& ts,
                                const GfxOutlineConfig& cfg,
                                const std::vector<GfxSpriteOutline>& outlines,
                                std::string& outError)
{
  outError.clear();

  std::ofstream f(path);
  if (!f) {
    outError = "failed to open outlines json for writing";
    return false;
  }

  f << "{\n";
  f << "  \"version\": 1,\n";
  f << "  \"atlasW\": " << ts.atlas.width << ",\n";
  f << "  \"atlasH\": " << ts.atlas.height << ",\n";
  f << "  \"tileW\": " << ts.tileW << ",\n";
  f << "  \"tileH\": " << ts.tileH << ",\n";
  f << "  \"alphaThreshold\": " << cfg.alphaThreshold << ",\n";
  f << "  \"sprites\": [\n";

  for (std::size_t i = 0; i < outlines.size(); ++i) {
    const auto& o = outlines[i];
    f << "    {\"name\": \"" << JsonEscape(o.name) << "\"";
    f << ", \"atlasX\": " << o.atlasX << ", \"atlasY\": " << o.atlasY;
    f << ", \"w\": " << o.w << ", \"h\": " << o.h;
    f << ", \"srcW\": " << o.srcW << ", \"srcH\": " << o.srcH;
    f << ", \"trimX\": " << o.trimX << ", \"trimY\": " << o.trimY;

    // Polygons.
    f << ", \"polygons\": [";
    for (std::size_t p = 0; p < o.geom.polygons.size(); ++p) {
      const auto& poly = o.geom.polygons[p];
      f << "{";
      f << "\"outer\":";
      WriteJsonPointArray(f, poly.outer);
      f << ",\"holes\":[";
      for (std::size_t h = 0; h < poly.holes.size(); ++h) {
        WriteJsonPointArray(f, poly.holes[h]);
        if (h + 1 < poly.holes.size()) f << ",";
      }
      f << "]";
      f << "}";
      if (p + 1 < o.geom.polygons.size()) f << ",";
    }
    f << "]";

    // Hull.
    if (!o.hull.empty()) {
      f << ", \"hull\": ";
      WriteJsonPointArray(f, o.hull);
    }

    f << "}";
    if (i + 1 < outlines.size()) f << ",";
    f << "\n";
  }

  f << "  ]\n";
  f << "}\n";

  return true;
}

bool WriteGfxTilesetOutlinesSvg(const std::string& path,
                               const std::string& atlasHref,
                               const GfxTilesetResult& ts,
                               const std::vector<GfxSpriteOutline>& outlines,
                               int svgScale,
                               std::string& outError)
{
  outError.clear();

  std::ofstream f(path);
  if (!f) {
    outError = "failed to open outlines svg for writing";
    return false;
  }

  const int w = std::max(1, ts.atlas.width);
  const int h = std::max(1, ts.atlas.height);
  const int scale = std::max(1, svgScale);

  f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  f << "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" width=\""
    << (w * scale) << "\" height=\"" << (h * scale) << "\" viewBox=\"0 0 " << w << " " << h << "\">\n";

  f << "  <defs>\n";
  f << "    <style>\n";
  f << "      .ol{fill:none;stroke:#ff4dd2;stroke-width:0.6;vector-effect:non-scaling-stroke;stroke-linejoin:round;}\n";
  f << "      .hull{fill:none;stroke:#00e5ff;stroke-width:0.6;vector-effect:non-scaling-stroke;stroke-linejoin:round;stroke-dasharray:2 2;}\n";
  f << "    </style>\n";
  f << "  </defs>\n";

  if (!atlasHref.empty()) {
    // SVG2 prefers 'href'; keep 'xlink:href' as a fallback for older renderers.
    f << "  <image href=\"" << atlasHref << "\" xlink:href=\"" << atlasHref << "\" x=\"0\" y=\"0\" width=\""
      << w << "\" height=\"" << h << "\" image-rendering=\"pixelated\"/>\n";
  }

  f << "  <g fill-rule=\"evenodd\">\n";

  for (const auto& o : outlines) {
    // Outline polygons.
    for (const auto& poly : o.geom.polygons) {
      if (poly.outer.size() < 4) continue;
      f << "    <path class=\"ol\" d=\"";

      // Transform from logical canvas coords to atlas coords: atlas + (p - trim).
      auto writeRing = [&](const std::vector<IPoint>& ring) {
        if (ring.size() < 4) return;
        std::vector<IPoint> r = ring;
        for (IPoint& p : r) {
          p.x = o.atlasX + (p.x - o.trimX);
          p.y = o.atlasY + (p.y - o.trimY);
        }
        WriteSvgRingPath(f, r);
      };

      writeRing(poly.outer);
      for (const auto& hole : poly.holes) {
        writeRing(hole);
      }

      f << "\"/>\n";
    }

    if (!o.hull.empty()) {
      std::vector<IPoint> r = o.hull;
      for (IPoint& p : r) {
        p.x = o.atlasX + (p.x - o.trimX);
        p.y = o.atlasY + (p.y - o.trimY);
      }
      f << "    <path class=\"hull\" d=\"";
      WriteSvgRingPath(f, r);
      f << "\"/>\n";
    }
  }

  f << "  </g>\n";
  f << "</svg>\n";

  return true;
}

} // namespace isocity
