#include "isocity/OsmImport.hpp"

#include "isocity/Brush.hpp"
#include "isocity/Random.hpp"
#include "isocity/Road.hpp"

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace isocity {

namespace {

struct NodeLL {
  double lat = 0.0;
  double lon = 0.0;
};

struct OsmWay {
  std::vector<std::int64_t> refs;

  // Tags of interest (empty == absent)
  std::string highway;
  std::string building;
  int buildingLevels = 0;
  std::string landuse;
  std::string leisure;
  std::string natural;
  std::string water;
  std::string waterway;
  std::string amenity;
};

struct ParsedOsm {
  std::unordered_map<std::int64_t, NodeLL> nodes;
  std::vector<OsmWay> ways;
  OsmLatLonBounds boundsTag;
  OsmLatLonBounds nodeBounds;
  std::size_t totalWays = 0;
};

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kEarthRadiusM = 6378137.0;      // WGS84 sphere radius used by Web Mercator.
constexpr double kMaxMercatorLat = 85.05112878; // practical clamp to avoid infinity.

inline double DegToRad(double deg) { return deg * (kPi / 180.0); }

inline double ClampLatForMercator(double latDeg)
{
  if (latDeg > kMaxMercatorLat) return kMaxMercatorLat;
  if (latDeg < -kMaxMercatorLat) return -kMaxMercatorLat;
  return latDeg;
}

inline double MercatorX(double lonDeg)
{
  return kEarthRadiusM * DegToRad(lonDeg);
}

inline double MercatorY(double latDeg)
{
  const double lat = ClampLatForMercator(latDeg);
  const double r = DegToRad(lat);
  // y = R * ln(tan(pi/4 + lat/2))
  return kEarthRadiusM * std::log(std::tan((kPi / 4.0) + (r / 2.0)));
}

inline int RoundToInt(double v)
{
  // Deterministic rounding without relying on current FP environment.
  return static_cast<int>(std::floor(v + 0.5));
}

bool ReadNextTag(std::istream& is, std::string& out)
{
  out.clear();
  char c = 0;
  // Scan to '<'.
  while (is.get(c)) {
    if (c == '<') break;
  }
  if (!is) return false;

  // Read until '>'.
  while (is.get(c)) {
    if (c == '>') break;
    out.push_back(c);
  }
  return !out.empty();
}

bool ExtractAttr(const std::string& tag, const char* key, std::string& out)
{
  out.clear();
  if (!key) return false;
  const std::string needle = std::string(key) + "=\"";
  const std::size_t p = tag.find(needle);
  if (p == std::string::npos) return false;
  const std::size_t start = p + needle.size();
  const std::size_t end = tag.find('"', start);
  if (end == std::string::npos || end <= start) return false;
  out = tag.substr(start, end - start);
  return true;
}

bool ParseAttrI64(const std::string& tag, const char* key, std::int64_t& out)
{
  std::string s;
  if (!ExtractAttr(tag, key, s)) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long long v = std::strtoll(s.c_str(), &end, 10);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  out = static_cast<std::int64_t>(v);
  return true;
}

bool ParseAttrF64(const std::string& tag, const char* key, double& out)
{
  std::string s;
  if (!ExtractAttr(tag, key, s)) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  out = v;
  return true;
}

bool ParseBoundsTag(const std::string& tag, OsmLatLonBounds& out)
{
  out = OsmLatLonBounds{};
  double minlat = 0.0, minlon = 0.0, maxlat = 0.0, maxlon = 0.0;
  if (!ParseAttrF64(tag, "minlat", minlat)) return false;
  if (!ParseAttrF64(tag, "minlon", minlon)) return false;
  if (!ParseAttrF64(tag, "maxlat", maxlat)) return false;
  if (!ParseAttrF64(tag, "maxlon", maxlon)) return false;
  if (!(minlat < maxlat) || !(minlon < maxlon)) return false;
  out.minLat = minlat;
  out.minLon = minlon;
  out.maxLat = maxlat;
  out.maxLon = maxlon;
  out.valid = true;
  return true;
}

inline void ExpandBounds(OsmLatLonBounds& b, double lat, double lon)
{
  if (!b.valid) {
    b.minLat = b.maxLat = lat;
    b.minLon = b.maxLon = lon;
    b.valid = true;
    return;
  }
  b.minLat = std::min(b.minLat, lat);
  b.maxLat = std::max(b.maxLat, lat);
  b.minLon = std::min(b.minLon, lon);
  b.maxLon = std::max(b.maxLon, lon);
}

bool ParseI32Prefix(const std::string& s, int& out)
{
  out = 0;
  if (s.empty()) return false;
  std::size_t i = 0;
  int sign = 1;
  if (s[i] == '+') {
    i++;
  } else if (s[i] == '-') {
    sign = -1;
    i++;
  }
  if (i >= s.size()) return false;

  long long v = 0;
  bool any = false;
  for (; i < s.size(); ++i) {
    const char c = s[i];
    if (c < '0' || c > '9') break;
    any = true;
    v = v * 10 + (c - '0');
    if (v > static_cast<long long>(std::numeric_limits<int>::max())) break;
  }
  if (!any) return false;
  v *= sign;
  if (v < std::numeric_limits<int>::min()) v = std::numeric_limits<int>::min();
  if (v > std::numeric_limits<int>::max()) v = std::numeric_limits<int>::max();
  out = static_cast<int>(v);
  return true;
}

bool IsIgnoredHighway(const std::string& v)
{
  // We only want drivable-ish road networks by default.
  // Keep this conservative; users can post-process or extend this later.
  return v.empty() || v == "footway" || v == "path" || v == "cycleway" || v == "steps" || v == "track" ||
         v == "bridleway" || v == "pedestrian" || v == "corridor" || v == "construction" || v == "proposed";
}

int RoadLevelForHighwayTag(const std::string& v)
{
  // Coarse mapping from OSM highway tags to the project's 3 road classes.
  //   1 = Street, 2 = Avenue, 3 = Highway
  //
  // "_link" variants usually behave like their parent class.
  const auto is = [&](const char* s) { return v == s; };
  const auto hasPrefix = [&](const char* p) {
    const std::string pref(p);
    return v.rfind(pref, 0) == 0;
  };

  if (hasPrefix("motorway") || hasPrefix("trunk") || is("primary") || is("primary_link")) return 3;
  if (is("secondary") || is("secondary_link") || is("tertiary") || is("tertiary_link")) return 2;

  // Common residential-ish ways.
  return 1;
}

int RadiusForRoadLevel(const OsmImportConfig& cfg, int roadLevel)
{
  if (cfg.fixedRadius >= 0) return std::max(0, cfg.fixedRadius);
  if (!cfg.thickenByClass) return 0;
  // Street=0, Avenue=1, Highway=2
  return std::max(0, ClampRoadLevel(roadLevel) - 1);
}

bool IsClosedWay(const OsmWay& w)
{
  return w.refs.size() >= 4 && w.refs.front() == w.refs.back();
}

bool IsWaterPolygonWay(const OsmWay& w)
{
  if (!IsClosedWay(w)) return false;
  if (w.natural == "water") return true;
  if (!w.water.empty()) return true;
  if (w.waterway == "riverbank") return true;
  if (w.landuse == "reservoir" || w.landuse == "basin") return true;
  return false;
}

bool IsWaterLineWay(const OsmWay& w)
{
  // Exclude polygon water features (riverbank is typically a closed ring).
  if (IsClosedWay(w)) return false;
  if (w.waterway.empty()) return false;
  if (w.waterway == "river" || w.waterway == "stream" || w.waterway == "canal" || w.waterway == "drain") return true;
  return false;
}

bool IsParkPolygonWay(const OsmWay& w)
{
  if (!IsClosedWay(w)) return false;
  if (w.leisure == "park" || w.leisure == "garden" || w.leisure == "playground") return true;
  if (w.landuse == "grass" || w.landuse == "meadow" || w.landuse == "recreation_ground" ||
      w.landuse == "village_green" || w.landuse == "forest")
    return true;
  if (w.natural == "wood" || w.natural == "scrub") return true;
  return false;
}

Overlay ZoneOverlayForLanduse(const OsmWay& w)
{
  if (w.landuse == "residential") return Overlay::Residential;
  if (w.landuse == "commercial" || w.landuse == "retail") return Overlay::Commercial;
  if (w.landuse == "industrial") return Overlay::Industrial;
  return Overlay::None;
}

Overlay ZoneOverlayForBuilding(const OsmWay& w)
{
  if (w.building.empty()) return Overlay::None;
  const std::string& b = w.building;

  auto is = [&](const char* s) { return b == s; };

  // Residential-ish
  if (is("residential") || is("apartments") || is("house") || is("detached") || is("terrace") ||
      is("semidetached_house") || is("bungalow") || is("dormitory") || is("farm") || is("cabin") ||
      is("hut") || is("static_caravan") || is("houseboat")) {
    return Overlay::Residential;
  }

  // Industrial-ish
  if (is("industrial") || is("warehouse") || is("factory") || is("manufacture") || is("hangar") || is("depot")) {
    return Overlay::Industrial;
  }

  // Commercial-ish
  if (is("commercial") || is("retail") || is("office") || is("supermarket") || is("kiosk") || is("mall") ||
      is("hotel")) {
    return Overlay::Commercial;
  }

  // Heuristic: if amenity exists and looks commercial-ish, default to commercial.
  if (!w.amenity.empty()) {
    const std::string& a = w.amenity;
    if (a == "restaurant" || a == "cafe" || a == "bar" || a == "pub" || a == "fast_food" || a == "bank" ||
        a == "pharmacy" || a == "hospital" || a == "school" || a == "college" || a == "university") {
      return Overlay::Commercial;
    }
  }

  // Default: ignore unknown building types (keeps the import conservative).
  return Overlay::None;
}

int ZoneLevelForBuildingLevels(int buildingLevels)
{
  // Rough mapping from real-world number of floors to the 3-tier visual density.
  //  1..2 floors  => level 1
  //  3..7 floors  => level 2
  //  8+ floors    => level 3
  if (buildingLevels >= 8) return 3;
  if (buildingLevels >= 3) return 2;
  return 1;
}

// Paint a diamond (Manhattan) radius around p.
template <typename Fn>
inline void ForEachDiamond(Point p, int r, Fn&& fn)
{
  r = std::max(0, r);
  for (int dy = -r; dy <= r; ++dy) {
    const int xr = r - std::abs(dy);
    for (int dx = -xr; dx <= xr; ++dx) {
      fn(Point{p.x + dx, p.y + dy});
    }
  }
}

struct Projector {
  double minX = 0.0;
  double maxX = 0.0;
  double minY = 0.0;
  double maxY = 0.0;
  double spanX = 0.0;
  double spanY = 0.0;

  int w = 0;
  int h = 0;
  int pad = 0;
  int availW = 0;
  int availH = 0;

  bool valid = false;

  Point map(double latDeg, double lonDeg) const
  {
    assert(valid);
    const double x = MercatorX(lonDeg);
    const double y = MercatorY(latDeg);

    const double u = (spanX > 0.0) ? ((x - minX) / spanX) : 0.0;
    const double v = (spanY > 0.0) ? ((y - minY) / spanY) : 0.0;

    // Invert Y because screen/tile coords increase downward.
    const double vf = 1.0 - v;

    const double fx = static_cast<double>(pad) + u * static_cast<double>(std::max(1, availW - 1));
    const double fy = static_cast<double>(pad) + vf * static_cast<double>(std::max(1, availH - 1));

    return Point{RoundToInt(fx), RoundToInt(fy)};
  }
};

bool BuildProjector(const OsmLatLonBounds& b, int w, int h, int padding, Projector& out, std::string& err)
{
  err.clear();
  out = Projector{};

  if (!b.valid) {
    err = "No valid OSM bounds";
    return false;
  }
  if (w <= 0 || h <= 0) {
    err = "World dimensions must be positive";
    return false;
  }

  const int pad = std::clamp(padding, 0, std::min(w, h) / 4);
  if (w - 2 * pad <= 1 || h - 2 * pad <= 1) {
    err = "World too small for the requested padding";
    return false;
  }

  const double minX = MercatorX(b.minLon);
  const double maxX = MercatorX(b.maxLon);
  const double minY = MercatorY(b.minLat);
  const double maxY = MercatorY(b.maxLat);
  const double spanX = (maxX - minX);
  const double spanY = (maxY - minY);
  if (!(spanX > 0.0) || !(spanY > 0.0)) {
    err = "OSM bounds have zero area";
    return false;
  }

  out.minX = minX;
  out.maxX = maxX;
  out.minY = minY;
  out.maxY = maxY;
  out.spanX = spanX;
  out.spanY = spanY;
  out.w = w;
  out.h = h;
  out.pad = pad;
  out.availW = w - 2 * pad;
  out.availH = h - 2 * pad;
  out.valid = true;
  return true;
}

bool WayHasAnyInterestingTags(const OsmWay& w)
{
  return !w.highway.empty() || !w.building.empty() || !w.landuse.empty() || !w.leisure.empty() ||
         !w.natural.empty() || !w.water.empty() || !w.waterway.empty();
}

bool ParseOsmXml(const std::string& path, ParsedOsm& out, std::string& outError)
{
  outError.clear();
  out = ParsedOsm{};

  std::ifstream f(path, std::ios::in | std::ios::binary);
  if (!f) {
    outError = "Failed to open OSM file";
    return false;
  }

  std::string tag;
  bool inWay = false;
  OsmWay cur;

  while (ReadNextTag(f, tag)) {
    if (tag.empty()) continue;

    // Skip XML declarations / comments / doctype.
    if (tag[0] == '?') continue;
    if (tag.rfind("!--", 0) == 0) continue;
    if (tag.rfind("!DOCTYPE", 0) == 0) continue;

    // Trim leading whitespace.
    const std::size_t first = tag.find_first_not_of(" \t\r\n");
    const std::string_view t = (first == std::string::npos) ? std::string_view{} : std::string_view(tag).substr(first);
    if (t.empty()) continue;

    if (!inWay) {
      if (t.rfind("bounds", 0) == 0) {
        OsmLatLonBounds b;
        if (ParseBoundsTag(std::string(t), b)) out.boundsTag = b;
        continue;
      }

      if (t.rfind("node", 0) == 0) {
        std::int64_t id = 0;
        double lat = 0.0;
        double lon = 0.0;
        if (!ParseAttrI64(std::string(t), "id", id)) continue;
        if (!ParseAttrF64(std::string(t), "lat", lat)) continue;
        if (!ParseAttrF64(std::string(t), "lon", lon)) continue;
        out.nodes[id] = NodeLL{lat, lon};
        ExpandBounds(out.nodeBounds, lat, lon);
        continue;
      }

      if (t.rfind("way", 0) == 0) {
        inWay = true;
        cur = OsmWay{};
        continue;
      }

      // We currently ignore relations.
      continue;
    }

    // --- inside <way> ... </way> ---
    if (t.rfind("/way", 0) == 0) {
      inWay = false;
      out.totalWays++;
      if (cur.refs.size() >= 2 && WayHasAnyInterestingTags(cur)) {
        out.ways.push_back(std::move(cur));
      }
      cur = OsmWay{};
      continue;
    }

    if (t.rfind("nd", 0) == 0) {
      std::int64_t ref = 0;
      if (ParseAttrI64(std::string(t), "ref", ref)) {
        cur.refs.push_back(ref);
      }
      continue;
    }

    if (t.rfind("tag", 0) == 0) {
      std::string k;
      std::string v;
      if (!ExtractAttr(std::string(t), "k", k)) continue;
      if (!ExtractAttr(std::string(t), "v", v)) continue;

      if (k == "highway") {
        cur.highway = v;
      } else if (k == "building") {
        cur.building = v;
      } else if (k == "building:levels") {
        int lv = 0;
        if (ParseI32Prefix(v, lv)) cur.buildingLevels = std::max(0, lv);
      } else if (k == "landuse") {
        cur.landuse = v;
      } else if (k == "leisure") {
        cur.leisure = v;
      } else if (k == "natural") {
        cur.natural = v;
      } else if (k == "water") {
        cur.water = v;
      } else if (k == "waterway") {
        cur.waterway = v;
      } else if (k == "amenity") {
        cur.amenity = v;
      }

      continue;
    }
  }

  // If the file ended while we were inside a way, close it out.
  if (inWay) {
    out.totalWays++;
    if (cur.refs.size() >= 2 && WayHasAnyInterestingTags(cur)) {
      out.ways.push_back(std::move(cur));
    }
  }

  return true;
}

// Collect projected tile points for a way.
// Returns false if any referenced node is missing.
template <typename NodeToPointFn>
bool CollectWayPoints(const OsmWay& w, NodeToPointFn&& nodeToPoint, std::vector<Point>& outPts)
{
  outPts.clear();
  outPts.reserve(w.refs.size());
  for (std::size_t i = 0; i < w.refs.size(); ++i) {
    Point p;
    if (!nodeToPoint(w.refs[i], p)) return false;
    if (!outPts.empty() && outPts.back().x == p.x && outPts.back().y == p.y) continue;
    outPts.push_back(p);
  }
  return outPts.size() >= 2;
}

// Conservative even/odd fill against tile centers.
template <typename Fn>
void RasterizePolygonFilled(const std::vector<Point>& ptsIn, Fn&& fn)
{
  if (ptsIn.size() < 3) return;

  // Copy and strip a duplicate closing vertex if present.
  std::vector<Point> pts = ptsIn;
  if (pts.size() >= 2 && pts.front().x == pts.back().x && pts.front().y == pts.back().y) {
    pts.pop_back();
  }
  if (pts.size() < 3) return;

  // Paint outline first (helps tiny polygons).
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const Point a = pts[i];
    const Point b = pts[(i + 1) % pts.size()];
    ForEachLinePoint(a, b, fn);
  }

  int minY = pts[0].y;
  int maxY = pts[0].y;
  for (const Point& p : pts) {
    minY = std::min(minY, p.y);
    maxY = std::max(maxY, p.y);
  }

  std::vector<double> xs;
  xs.reserve(pts.size());

  for (int y = minY; y <= maxY; ++y) {
    xs.clear();
    const double scanY = static_cast<double>(y) + 0.5; // tile center

    for (std::size_t i = 0; i < pts.size(); ++i) {
      const Point p0i = pts[i];
      const Point p1i = pts[(i + 1) % pts.size()];

      const double x0 = static_cast<double>(p0i.x) + 0.5;
      const double y0 = static_cast<double>(p0i.y) + 0.5;
      const double x1 = static_cast<double>(p1i.x) + 0.5;
      const double y1 = static_cast<double>(p1i.y) + 0.5;

      // Half-open rule avoids double-counting vertices.
      const bool crosses = (y0 <= scanY && y1 > scanY) || (y1 <= scanY && y0 > scanY);
      if (!crosses) continue;
      const double dy = (y1 - y0);
      if (std::abs(dy) < 1e-9) continue;
      const double t = (scanY - y0) / dy;
      const double x = x0 + t * (x1 - x0);
      xs.push_back(x);
    }

    if (xs.size() < 2) continue;
    std::sort(xs.begin(), xs.end());

    for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
      double xL = xs[i];
      double xR = xs[i + 1];
      if (xL > xR) std::swap(xL, xR);

      // Fill tiles whose centers fall between xL..xR.
      const int xStart = static_cast<int>(std::ceil(xL - 0.5));
      const int xEnd = static_cast<int>(std::floor(xR - 0.5));
      for (int x = xStart; x <= xEnd; ++x) {
        fn(Point{x, y});
      }
    }
  }
}

bool ImportOsmParsed(const ParsedOsm& osm,
                     World& world,
                     const OsmImportConfig& cfg,
                     OsmImportStats* outStats,
                     std::string& outError)
{
  outError.clear();
  if (outStats) *outStats = OsmImportStats{};

  OsmLatLonBounds bounds;
  if (cfg.preferBoundsTag && osm.boundsTag.valid) {
    bounds = osm.boundsTag;
  } else {
    bounds = osm.nodeBounds;
  }
  if (!bounds.valid) {
    outError = "No valid bounds found in OSM file";
    return false;
  }

  Projector proj;
  if (!BuildProjector(bounds, world.width(), world.height(), cfg.padding, proj, outError)) return false;

  const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu);

  // Cache projected node ids to avoid repeated mercator math.
  std::unordered_map<std::int64_t, Point> cached;
  cached.reserve(osm.nodes.size());

  auto nodeToPoint = [&](std::int64_t id, Point& out) -> bool {
    const auto it = cached.find(id);
    if (it != cached.end()) {
      out = it->second;
      return true;
    }
    const auto nt = osm.nodes.find(id);
    if (nt == osm.nodes.end()) return false;
    const Point p = proj.map(nt->second.lat, nt->second.lon);
    cached[id] = p;
    out = p;
    return true;
  };

  auto stableVar = [&](int x, int y) -> std::uint8_t {
    return static_cast<std::uint8_t>(HashCoords32(x, y, seed32) & 0xFFu);
  };

  auto paintWaterTile = [&](int x, int y) {
    if (!world.inBounds(x, y)) return;
    Tile& t = world.at(x, y);
    t.terrain = Terrain::Water;
    t.height = 0.0f;

    // Clear non-road overlays so we don't end up with zoning on water.
    if (t.overlay != Overlay::Road) {
      t.overlay = Overlay::None;
      t.level = 1;
      t.occupants = 0;
      t.variation = stableVar(x, y);
    }
  };

  auto paintZoneTile = [&](int x, int y, Overlay ov, int level) {
    if (!world.inBounds(x, y)) return;
    Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water) return;
    if (t.overlay == Overlay::Road) return;

    if (!cfg.overwriteNonRoadOverlays) {
      // If something already exists here, only allow idempotent writes.
      if (t.overlay != Overlay::None && t.overlay != ov) return;
    }

    t.overlay = ov;
    t.level = static_cast<std::uint8_t>(std::clamp(level, 1, 3));
    t.occupants = 0;
    t.variation = stableVar(x, y);
  };

  auto paintParkTile = [&](int x, int y) {
    if (!world.inBounds(x, y)) return;
    Tile& t = world.at(x, y);
    if (t.terrain == Terrain::Water) return;
    if (t.overlay == Overlay::Road) return;

    if (!cfg.overwriteNonRoadOverlays) {
      if (t.overlay != Overlay::None && t.overlay != Overlay::Park) return;
    }

    t.overlay = Overlay::Park;
    t.level = 1;
    t.occupants = 0;
    t.variation = stableVar(x, y);
  };

  auto paintRoadTile = [&](int x, int y, int level) {
    if (!world.inBounds(x, y)) return;
    Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) {
      t.overlay = Overlay::Road;
      t.occupants = 0;
      t.level = static_cast<std::uint8_t>(ClampRoadLevel(level));
      // High bits: stable per-tile variation; low bits reserved for road mask.
      const std::uint8_t varHi = static_cast<std::uint8_t>(HashCoords32(x, y, seed32) & 0xF0u);
      t.variation = varHi;
    } else {
      const int cur = ClampRoadLevel(static_cast<int>(t.level));
      const int tgt = ClampRoadLevel(level);
      t.level = static_cast<std::uint8_t>(std::max(cur, tgt));
      // Preserve existing variation high bits.
      t.variation = static_cast<std::uint8_t>(t.variation & 0xF0u);
    }
  };

  std::vector<Point> pts;

  std::size_t highwaysImported = 0;
  std::size_t waterWaysImported = 0;
  std::size_t landuseWaysImported = 0;
  std::size_t parkWaysImported = 0;
  std::size_t buildingWaysImported = 0;

  // --- Phase 1: water ---
  if (cfg.importWater) {
    for (const OsmWay& w : osm.ways) {
      if (IsWaterPolygonWay(w)) {
        if (!CollectWayPoints(w, nodeToPoint, pts)) continue;
        RasterizePolygonFilled(pts, [&](Point p) { paintWaterTile(p.x, p.y); });
        waterWaysImported++;
        continue;
      }

      if (IsWaterLineWay(w)) {
        if (!CollectWayPoints(w, nodeToPoint, pts)) continue;
        const int r = std::max(0, cfg.waterwayRadius);
        for (std::size_t i = 1; i < pts.size(); ++i) {
          const Point a = pts[i - 1];
          const Point b = pts[i];
          ForEachLinePoint(a, b, [&](Point p) {
            ForEachDiamond(p, r, [&](Point q) { paintWaterTile(q.x, q.y); });
          });
        }
        waterWaysImported++;
        continue;
      }
    }
  }

  // --- Phase 2: parks (non-zoning greenspace) ---
  if (cfg.importParks) {
    for (const OsmWay& w : osm.ways) {
      if (!IsParkPolygonWay(w)) continue;
      if (!CollectWayPoints(w, nodeToPoint, pts)) continue;
      RasterizePolygonFilled(pts, [&](Point p) { paintParkTile(p.x, p.y); });
      parkWaysImported++;
    }
  }

  // --- Phase 3: landuse zones ---
  if (cfg.importLanduse) {
    for (const OsmWay& w : osm.ways) {
      const Overlay ov = ZoneOverlayForLanduse(w);
      if (ov == Overlay::None) continue;
      if (!IsClosedWay(w)) continue;
      if (!CollectWayPoints(w, nodeToPoint, pts)) continue;
      RasterizePolygonFilled(pts, [&](Point p) { paintZoneTile(p.x, p.y, ov, 1); });
      landuseWaysImported++;
    }
  }

  // --- Phase 4: buildings (more specific than landuse) ---
  if (cfg.importBuildings) {
    for (const OsmWay& w : osm.ways) {
      if (w.building.empty()) continue;
      if (!IsClosedWay(w)) continue;
      const Overlay ov = ZoneOverlayForBuilding(w);
      if (ov == Overlay::None) continue;
      if (!CollectWayPoints(w, nodeToPoint, pts)) continue;
      const int lvl = ZoneLevelForBuildingLevels(w.buildingLevels);
      RasterizePolygonFilled(pts, [&](Point p) { paintZoneTile(p.x, p.y, ov, lvl); });
      buildingWaysImported++;
    }
  }

  // --- Phase 5: roads ---
  if (cfg.importRoads) {
    for (const OsmWay& w : osm.ways) {
      if (w.highway.empty()) continue;
      if (IsIgnoredHighway(w.highway)) continue;

      const int level = RoadLevelForHighwayTag(w.highway);
      const int radius = RadiusForRoadLevel(cfg, level);

      Point prev;
      bool hasPrev = false;
      for (std::size_t i = 0; i < w.refs.size(); ++i) {
        Point cur;
        if (!nodeToPoint(w.refs[i], cur)) {
          hasPrev = false;
          continue;
        }
        if (hasPrev) {
          ForEachLinePoint(prev, cur, [&](Point p) {
            ForEachDiamond(p, radius, [&](Point q) { paintRoadTile(q.x, q.y, level); });
          });
        }
        prev = cur;
        hasPrev = true;
      }
      highwaysImported++;
    }

    world.recomputeRoadMasks();
  }

  // Count tiles by final state.
  std::size_t roadTiles = 0;
  std::size_t waterTiles = 0;
  std::size_t zoneTiles = 0;
  std::size_t parkTiles = 0;
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      if (t.overlay == Overlay::Road) roadTiles++;
      if (t.overlay == Overlay::Park) parkTiles++;
      if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial)
        zoneTiles++;
      if (t.terrain == Terrain::Water) waterTiles++;
    }
  }

  if (outStats) {
    outStats->nodesParsed = osm.nodes.size();
    outStats->waysParsed = osm.totalWays;

    outStats->highwayWaysImported = highwaysImported;
    outStats->waterWaysImported = waterWaysImported;
    outStats->landuseWaysImported = landuseWaysImported;
    outStats->parkWaysImported = parkWaysImported;
    outStats->buildingWaysImported = buildingWaysImported;

    outStats->roadTilesPainted = roadTiles;
    outStats->waterTilesPainted = waterTiles;
    outStats->zoneTilesPainted = zoneTiles;
    outStats->parkTilesPainted = parkTiles;

    outStats->bounds = bounds;
    outStats->outWidth = world.width();
    outStats->outHeight = world.height();
  }

  return true;
}

} // namespace

bool ImportOsmXmlRoads(const std::string& osmPath,
                       World& world,
                       const OsmImportConfig& cfg,
                       OsmImportStats* outStats,
                       std::string& outError)
{
  ParsedOsm osm;
  if (!ParseOsmXml(osmPath, osm, outError)) return false;
  return ImportOsmParsed(osm, world, cfg, outStats, outError);
}

bool ImportOsmXmlRoadsToNewWorld(const std::string& osmPath,
                                 std::uint64_t seed,
                                 const OsmImportConfig& cfg,
                                 World& outWorld,
                                 OsmImportStats* outStats,
                                 std::string& outError)
{
  outError.clear();
  if (outStats) *outStats = OsmImportStats{};

  ParsedOsm osm;
  if (!ParseOsmXml(osmPath, osm, outError)) return false;

  OsmLatLonBounds bounds;
  if (cfg.preferBoundsTag && osm.boundsTag.valid) {
    bounds = osm.boundsTag;
  } else {
    bounds = osm.nodeBounds;
  }
  if (!bounds.valid) {
    outError = "No valid bounds found in OSM file";
    return false;
  }

  int w = cfg.width;
  int h = cfg.height;
  if (w <= 0 || h <= 0) {
    const double minX = MercatorX(bounds.minLon);
    const double maxX = MercatorX(bounds.maxLon);
    const double minY = MercatorY(bounds.minLat);
    const double maxY = MercatorY(bounds.maxLat);
    const double spanX = std::max(0.0, maxX - minX);
    const double spanY = std::max(0.0, maxY - minY);
    if (!(spanX > 0.0) || !(spanY > 0.0)) {
      outError = "OSM bounds have zero area";
      return false;
    }

    const double mpt = std::max(0.001, cfg.metersPerTile);
    const int pad = std::max(0, cfg.padding);
    w = std::max(1, static_cast<int>(std::ceil(spanX / mpt)) + 2 * pad);
    h = std::max(1, static_cast<int>(std::ceil(spanY / mpt)) + 2 * pad);

    // Guardrail: importing an entire planet is not a goal.
    const int kMaxDim = 4096;
    if (w > kMaxDim || h > kMaxDim) {
      outError = "Auto-sized world is too large; increase metersPerTile or provide --size";
      return false;
    }
  }

  outWorld = World(w, h, seed);
  return ImportOsmParsed(osm, outWorld, cfg, outStats, outError);
}

} // namespace isocity
