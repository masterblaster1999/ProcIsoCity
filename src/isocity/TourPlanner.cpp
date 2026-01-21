#include "isocity/TourPlanner.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/GfxText.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/RoadGraphCentrality.hpp"
#include "isocity/Random.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace isocity {
namespace {

using gfx::BlendMode;

inline int Idx(int x, int y, int w)
{
  return y * w + x;
}

inline bool IsRoadTile(const World& world, const Point& p)
{
  return world.inBounds(p.x, p.y) && world.at(p.x, p.y).overlay == Overlay::Road;
}

static std::uint64_t TieKey(const World& world, const TourConfig& cfg, int x, int y, std::uint32_t extra = 0)
{
  const std::uint32_t seed32 = static_cast<std::uint32_t>((world.seed() ^ cfg.seedSalt ^ extra) & 0xFFFFFFFFu);
  const std::uint32_t h = HashCoords32(x, y, seed32);
  return (static_cast<std::uint64_t>(h) << 32) ^ static_cast<std::uint64_t>(seed32);
}

static bool FindNearestRoadTile(const World& world, const TourConfig& cfg, const Point& start, int maxDist, Point& out)
{
  if (IsRoadTile(world, start)) {
    out = start;
    return true;
  }

  maxDist = std::max(0, maxDist);
  std::uint64_t bestKey = std::numeric_limits<std::uint64_t>::max();
  Point best{0, 0};
  bool found = false;

  // Diamond ring scan: Manhattan distance d.
  for (int d = 1; d <= maxDist; ++d) {
    bestKey = std::numeric_limits<std::uint64_t>::max();
    found = false;

    for (int dx = -d; dx <= d; ++dx) {
      const int dy = d - std::abs(dx);
      const int xs = start.x + dx;
      const int y0 = start.y + dy;
      const int y1 = start.y - dy;

      const Point p0{xs, y0};
      if (IsRoadTile(world, p0)) {
        const std::uint64_t key = TieKey(world, cfg, p0.x, p0.y, 0xA11u);
        if (key < bestKey) {
          bestKey = key;
          best = p0;
          found = true;
        }
      }

      if (dy != 0) {
        const Point p1{xs, y1};
        if (IsRoadTile(world, p1)) {
          const std::uint64_t key = TieKey(world, cfg, p1.x, p1.y, 0xB22u);
          if (key < bestKey) {
            bestKey = key;
            best = p1;
            found = true;
          }
        }
      }
    }

    if (found) {
      out = best;
      return true;
    }
  }

  return false;
}

static std::vector<std::uint8_t> FloodFillReachableRoads(const World& world, const Point& startRoad)
{
  const int w = world.width();
  const int h = world.height();
  std::vector<std::uint8_t> vis(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), std::uint8_t{0});
  if (!IsRoadTile(world, startRoad)) return vis;

  std::queue<Point> q;
  q.push(startRoad);
  vis[static_cast<std::size_t>(Idx(startRoad.x, startRoad.y, w))] = 1;

  while (!q.empty()) {
    const Point p = q.front();
    q.pop();

    const Point n4[4] = {{p.x + 1, p.y}, {p.x - 1, p.y}, {p.x, p.y + 1}, {p.x, p.y - 1}};
    for (const Point& n : n4) {
      if (!IsRoadTile(world, n)) continue;
      const int idx = Idx(n.x, n.y, w);
      if (idx < 0 || idx >= w * h) continue;
      std::uint8_t& v = vis[static_cast<std::size_t>(idx)];
      if (v) continue;
      v = 1;
      q.push(n);
    }
  }

  return vis;
}

static std::string StreetNameForId(const StreetNamingResult& streets, int streetId)
{
  if (streetId >= 0 && streetId < static_cast<int>(streets.streets.size())) {
    return streets.streets[static_cast<std::size_t>(streetId)].name;
  }
  return "Unnamed Rd";
}

static std::string TrimAscii(std::string s)
{
  auto isSpace = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; };
  while (!s.empty() && isSpace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && isSpace(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

static std::string BaseFromStreetName(const std::string& streetName)
{
  // Strip common street suffixes so POIs are "Oak Lookout" instead of "Oak Street Lookout".
  static const char* kSuffixes[] = {"St",      "Street",  "Rd",    "Road",     "Ln",     "Lane",
                                    "Ave",    "Avenue",  "Blvd",  "Boulevard", "Pkwy",   "Parkway",
                                    "Dr",     "Drive",   "Way",   "Ct",       "Court",  "Pl",
                                    "Place",  "Ter",     "Terrace"};

  std::string s = TrimAscii(streetName);
  if (s.empty()) return s;

  // Highway-ish names keep their full form.
  if (s.rfind("I-", 0) == 0 || s.rfind("Hwy ", 0) == 0 || s.rfind("Rte ", 0) == 0) {
    return s;
  }

  // Tokenize by space.
  std::vector<std::string> toks;
  std::string cur;
  for (char c : s) {
    if (c == ' ') {
      if (!cur.empty()) toks.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) toks.push_back(cur);
  if (toks.empty()) return s;

  auto eqAscii = [](const std::string& a, const char* b) {
    std::string bb(b);
    if (a.size() != bb.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
      const unsigned char ca = static_cast<unsigned char>(a[i]);
      const unsigned char cb = static_cast<unsigned char>(bb[i]);
      if (std::tolower(ca) != std::tolower(cb)) return false;
    }
    return true;
  };

  const std::string& last = toks.back();
  for (const char* suf : kSuffixes) {
    if (eqAscii(last, suf)) {
      toks.pop_back();
      break;
    }
  }

  if (toks.empty()) return s;

  std::ostringstream oss;
  for (std::size_t i = 0; i < toks.size(); ++i) {
    if (i) oss << ' ';
    oss << toks[i];
  }
  return oss.str();
}

static std::string PickSuffix(PoiKind k, RNG& rng)
{
  switch (k) {
    case PoiKind::CivicCenter: {
      static const char* s[] = {"Plaza", "Square", "Circle", "Commons"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::Bottleneck: {
      static const char* s[] = {"Gate", "Pass", "Bridge", "Junction"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::Peak: {
      static const char* s[] = {"Lookout", "Highpoint", "Summit", "Overlook"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::Waterfront: {
      static const char* s[] = {"Quay", "Promenade", "Riverwalk", "Harborfront"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::GrandPark: {
      static const char* s[] = {"Gardens", "Green", "Park", "Commons"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::Market: {
      static const char* s[] = {"Market", "Bazaar", "Hall", "Exchange"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::Works: {
      static const char* s[] = {"Works", "Foundry", "Yards", "Depot"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
    case PoiKind::DistrictHub: {
      static const char* s[] = {"Hub", "Center", "Station", "Square"};
      return s[rng.rangeInt(0, static_cast<int>(std::size(s)) - 1)];
    }
  }
  return "Place";
}

static Poi MakePoi(const World& world,
                   const StreetNamingResult& streets,
                   const std::vector<ParcelAddress>& addresses,
                   const std::vector<std::string>& districtNames,
                   const TourConfig& cfg,
                   PoiKind kind,
                   int id,
                   const Point& roadTile,
                   float featureValue,
                   std::uint64_t score)
{
  Poi p;
  p.kind = kind;
  p.id = id;
  p.roadTile = roadTile;
  p.featureValue = featureValue;
  p.score = score;

  // Context: street id/name.
  if (streets.w == world.width() && streets.h == world.height()) {
    const std::size_t idx = static_cast<std::size_t>(roadTile.y) * static_cast<std::size_t>(streets.w) +
                            static_cast<std::size_t>(roadTile.x);
    if (idx < streets.roadTileToStreetId.size()) {
      p.streetId = streets.roadTileToStreetId[idx];
      p.streetName = StreetNameForId(streets, p.streetId);
    }
  }

  // Context: district.
  if (world.inBounds(roadTile.x, roadTile.y)) {
    p.district = static_cast<int>(world.at(roadTile.x, roadTile.y).district);
    if (p.district >= 0 && p.district < static_cast<int>(districtNames.size())) {
      p.districtName = districtNames[static_cast<std::size_t>(p.district)];
    }
  }

  // Nearest generated parcel address (best-effort).
  if (!addresses.empty()) {
    int best = -1;
    int bestD = std::numeric_limits<int>::max();
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    for (int i = 0; i < static_cast<int>(addresses.size()); ++i) {
      const ParcelAddress& a = addresses[static_cast<std::size_t>(i)];
      const int d = std::abs(a.roadTile.x - roadTile.x) + std::abs(a.roadTile.y - roadTile.y);
      const std::uint64_t tie = TieKey(world, cfg, a.roadTile.x, a.roadTile.y, 0xC33u);
      if (d < bestD || (d == bestD && tie < bestTie)) {
        bestD = d;
        bestTie = tie;
        best = i;
      }
    }
    if (best >= 0) {
      p.nearAddress = addresses[static_cast<std::size_t>(best)].full;
    }
  }

  // Name.
  RNG rng(world.seed() ^ cfg.seedSalt ^ (static_cast<std::uint64_t>(kind) * 0x9E3779B97F4A7C15ULL) ^
          (static_cast<std::uint64_t>(roadTile.x) << 32) ^ static_cast<std::uint64_t>(roadTile.y));

  std::string base;
  if (kind == PoiKind::DistrictHub && !p.districtName.empty()) {
    base = p.districtName;
  } else {
    base = BaseFromStreetName(p.streetName);
    if (base.empty()) base = p.districtName;
    if (base.empty()) base = "Central";
  }

  const std::string suf = PickSuffix(kind, rng);
  if (!suf.empty()) {
    if (base == suf) {
      p.name = base;
    } else {
      p.name = base + " " + suf;
    }
  } else {
    p.name = base;
  }

  // Description (compact, CLI-friendly).
  {
    std::ostringstream oss;
    switch (kind) {
      case PoiKind::CivicCenter:
        oss << "A civic focal point";
        break;
      case PoiKind::Bottleneck:
        oss << "A structural road bottleneck";
        break;
      case PoiKind::Peak:
        oss << "A high elevation viewpoint";
        break;
      case PoiKind::Waterfront:
        oss << "A walkable waterfront stretch";
        break;
      case PoiKind::GrandPark:
        oss << "A major park / green space";
        break;
      case PoiKind::Market:
        oss << "A busy commercial cluster";
        break;
      case PoiKind::Works:
        oss << "An industrial landmark";
        break;
      case PoiKind::DistrictHub:
        oss << "A district hub";
        break;
    }
    if (!p.districtName.empty()) {
      oss << " in " << p.districtName;
    }
    if (!p.nearAddress.empty()) {
      oss << " (near " << p.nearAddress << ")";
    }
    oss << ".";
    p.description = oss.str();
  }

  return p;
}

static ParcelAddress MakeSyntheticEndpoint(const Poi& poi)
{
  ParcelAddress a;
  a.parcelIndex = -1;
  a.streetId = poi.streetId;
  a.houseNumber = 0;
  a.streetName = poi.streetName;
  a.full = poi.name;
  a.roadTile = poi.roadTile;
  a.parcelAnchor = poi.roadTile;
  return a;
}

static ParcelAddress MakeSyntheticEndpoint(const std::string& name, const Point& roadTile)
{
  ParcelAddress a;
  a.parcelIndex = -1;
  a.streetId = -1;
  a.houseNumber = 0;
  a.streetName.clear();
  a.full = name;
  a.roadTile = roadTile;
  a.parcelAnchor = roadTile;
  return a;
}

static bool TooClose(const std::vector<Point>& picked, const Point& p, int minSep)
{
  for (const Point& q : picked) {
    const int d = std::abs(q.x - p.x) + std::abs(q.y - p.y);
    if (d < minSep) return true;
  }
  return false;
}

} // namespace

const char* PoiKindName(PoiKind k)
{
  switch (k) {
    case PoiKind::CivicCenter:
      return "civic_center";
    case PoiKind::Bottleneck:
      return "bottleneck";
    case PoiKind::Peak:
      return "peak";
    case PoiKind::Waterfront:
      return "waterfront";
    case PoiKind::GrandPark:
      return "grand_park";
    case PoiKind::Market:
      return "market";
    case PoiKind::Works:
      return "works";
    case PoiKind::DistrictHub:
      return "district_hub";
  }
  return "poi";
}

TourPlan BuildProceduralTour(const World& world, const std::string& startQuery,
                             const TourConfig& cfg,
                             const StreetNamingConfig& streetCfg,
                             const AddressIndexConfig& indexCfg)
{
  TourPlan out;
  out.seed = world.seed();
  out.width = world.width();
  out.height = world.height();
  out.title = GenerateCityName(world.seed());
  out.startQuery = startQuery;

  // Base semantic layers.
  const std::vector<std::string> districtNames = GenerateDistrictNames(world);
  const StreetNamingResult streets = BuildStreetNames(world, streetCfg);
  const std::vector<ParcelAddress> addresses = BuildParcelAddresses(world, streets);
  const AddressIndex addrIndex = BuildAddressIndex(addresses, indexCfg);

  // Centrality analysis (optional but also useful for default start).
  RoadGraph g = BuildRoadGraph(world);
  RoadGraphCentralityResult cent;
  bool haveCentrality = false;
  if (!g.nodes.empty() && (cfg.centralityMaxSources != 0 || cfg.includeBottleneck || cfg.includeDistrictHubs || startQuery.empty())) {
    RoadGraphCentralityConfig ccfg;
    ccfg.weightMode = RoadGraphEdgeWeightMode::TravelTimeMilli;
    ccfg.maxSources = cfg.centralityMaxSources;
    ccfg.scaleSampleToFull = true;
    ccfg.undirected = true;
    ccfg.normalizeBetweenness = true;
    ccfg.closenessComponentScale = true;
    cent = ComputeRoadGraphCentrality(g, ccfg, &world);
    haveCentrality = true;
  }

  // Resolve start.
  GeocodeMatch gm;
  if (!startQuery.empty()) {
    gm = GeocodeEndpoint(world, streets, addrIndex, startQuery);
  }

  if (startQuery.empty() || !gm.ok) {
    // Pick a deterministic "structural center" if possible.
    Point startRoad{world.width() / 2, world.height() / 2};

    if (haveCentrality && !cent.nodeCloseness.empty()) {
      double best = -1.0;
      std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
      for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
        const double v = cent.nodeCloseness[static_cast<std::size_t>(i)];
        const Point p = g.nodes[static_cast<std::size_t>(i)].pos;
        const std::uint64_t tie = TieKey(world, cfg, p.x, p.y, 0xCE01u);
        if (v > best || (v == best && tie < bestTie)) {
          best = v;
          bestTie = tie;
          startRoad = p;
        }
      }
    } else {
      (void)FindNearestRoadTile(world, cfg, startRoad, std::max(world.width(), world.height()), startRoad);
    }

    // Prefer a real parcel address near the center (more human-friendly).
    if (!addresses.empty()) {
      int best = -1;
      int bestD = std::numeric_limits<int>::max();
      std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
      for (int i = 0; i < static_cast<int>(addresses.size()); ++i) {
        const ParcelAddress& a = addresses[static_cast<std::size_t>(i)];
        const int d = std::abs(a.roadTile.x - startRoad.x) + std::abs(a.roadTile.y - startRoad.y);
        const std::uint64_t tie = TieKey(world, cfg, a.roadTile.x, a.roadTile.y, 0x51A0u);
        if (d < bestD || (d == bestD && tie < bestTie)) {
          bestD = d;
          bestTie = tie;
          best = i;
        }
      }
      if (best >= 0) {
        out.start = addresses[static_cast<std::size_t>(best)];
      } else {
        out.start = MakeSyntheticEndpoint("Start", startRoad);
      }
    } else {
      out.start = MakeSyntheticEndpoint("Start", startRoad);
    }
  } else {
    out.start = gm.endpoint;
  }

  // Road connectivity mask: keep POIs in the same reachable road component as the start.
  const std::vector<std::uint8_t> reach = FloodFillReachableRoads(world, out.start.roadTile);

  auto roadReachable = [&](const Point& p) -> bool {
    if (!IsRoadTile(world, p)) return false;
    const int w = world.width();
    const int idx = Idx(p.x, p.y, w);
    if (idx < 0 || idx >= w * world.height()) return false;
    return reach[static_cast<std::size_t>(idx)] != 0;
  };

  // Build candidate POIs.
  std::vector<Poi> candidates;
  candidates.reserve(32);
  int nextId = 1;

  // 1) Central node (closeness).
  if (haveCentrality && !g.nodes.empty() && !cent.nodeCloseness.empty()) {
    double best = -1.0;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    Point bestP{0, 0};
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
      const double v = cent.nodeCloseness[static_cast<std::size_t>(i)];
      const Point p = g.nodes[static_cast<std::size_t>(i)].pos;
      const std::uint64_t tie = TieKey(world, cfg, p.x, p.y, 0xC1E1u);
      if (v > best || (v == best && tie < bestTie)) {
        best = v;
        bestTie = tie;
        bestP = p;
      }
    }
    if (roadReachable(bestP)) {
      const std::uint64_t score = static_cast<std::uint64_t>(std::llround(best * 1.0e12));
      candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::CivicCenter, nextId++, bestP,
                                   static_cast<float>(best), score));
    }
  }

  // 2) Bottleneck edge (betweenness).
  if (cfg.includeBottleneck && haveCentrality && !g.edges.empty() && !cent.edgeBetweennessNorm.empty()) {
    double best = -1.0;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    Point bestP{0, 0};
    for (int i = 0; i < static_cast<int>(g.edges.size()); ++i) {
      const double v = cent.edgeBetweennessNorm[static_cast<std::size_t>(i)];
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(i)];
      if (e.tiles.empty()) continue;
      const Point p = e.tiles[e.tiles.size() / 2u];
      const std::uint64_t tie = TieKey(world, cfg, p.x, p.y, 0xB0E1u);
      if (v > best || (v == best && tie < bestTie)) {
        best = v;
        bestTie = tie;
        bestP = p;
      }
    }
    if (roadReachable(bestP)) {
      const std::uint64_t score = static_cast<std::uint64_t>(std::llround(best * 1.0e12));
      candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::Bottleneck, nextId++, bestP,
                                   static_cast<float>(best), score));
    }
  }

  // 3) Peak (max height).
  if (cfg.includePeak) {
    float bestH = -1.0f;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    Point bestP{0, 0};
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);
        if (t.terrain == Terrain::Water) continue;
        const float h = t.height;
        const std::uint64_t tie = TieKey(world, cfg, x, y, 0xEAA1u);
        if (h > bestH || (h == bestH && tie < bestTie)) {
          bestH = h;
          bestTie = tie;
          bestP = Point{x, y};
        }
      }
    }
    Point road{0, 0};
    if (FindNearestRoadTile(world, cfg, bestP, std::max(world.width(), world.height()), road) && roadReachable(road)) {
      const std::uint64_t score = static_cast<std::uint64_t>(std::llround(static_cast<double>(bestH) * 1.0e12));
      candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::Peak, nextId++, road, bestH, score));
    }
  }

  // 4) Grand park (largest connected park component).
  if (cfg.includePark) {
    const int w = world.width();
    const int h = world.height();
    std::vector<std::uint8_t> vis(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), std::uint8_t{0});
    int bestSize = 0;
    double bestCx = 0.0;
    double bestCy = 0.0;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();

    std::queue<Point> q;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int idx = Idx(x, y, w);
        if (vis[static_cast<std::size_t>(idx)]) continue;
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Park) continue;

        // BFS component.
        int size = 0;
        double sx = 0.0;
        double sy = 0.0;
        vis[static_cast<std::size_t>(idx)] = 1;
        q.push(Point{x, y});

        while (!q.empty()) {
          const Point p = q.front();
          q.pop();
          ++size;
          sx += p.x;
          sy += p.y;

          const Point n4[4] = {{p.x + 1, p.y}, {p.x - 1, p.y}, {p.x, p.y + 1}, {p.x, p.y - 1}};
          for (const Point& n : n4) {
            if (!world.inBounds(n.x, n.y)) continue;
            const int ni = Idx(n.x, n.y, w);
            if (vis[static_cast<std::size_t>(ni)]) continue;
            if (world.at(n.x, n.y).overlay != Overlay::Park) continue;
            vis[static_cast<std::size_t>(ni)] = 1;
            q.push(n);
          }
        }

        const std::uint64_t tie = TieKey(world, cfg, x, y, 0xFA12u);
        if (size > bestSize || (size == bestSize && tie < bestTie)) {
          bestSize = size;
          bestTie = tie;
          bestCx = sx / static_cast<double>(std::max(1, size));
          bestCy = sy / static_cast<double>(std::max(1, size));
        }
      }
    }

    if (bestSize > 0) {
      const Point centroid{static_cast<int>(std::lround(bestCx)), static_cast<int>(std::lround(bestCy))};
      Point road{0, 0};
      if (FindNearestRoadTile(world, cfg, centroid, std::max(world.width(), world.height()), road) && roadReachable(road)) {
        const std::uint64_t score = static_cast<std::uint64_t>(bestSize) * 1000000ull;
        candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::GrandPark, nextId++, road,
                                     static_cast<float>(bestSize), score));
      }
    }
  }

  // 5) Waterfront (road adjacency to water).
  if (cfg.includeWaterfront) {
    int bestAdj = 0;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    Point bestP{0, 0};
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Point p{x, y};
        if (!IsRoadTile(world, p)) continue;
        int adj = 0;
        const Point n4[4] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
        for (const Point& n : n4) {
          if (!world.inBounds(n.x, n.y)) continue;
          if (world.at(n.x, n.y).terrain == Terrain::Water) ++adj;
        }
        if (adj <= 0) continue;
        const std::uint64_t tie = TieKey(world, cfg, x, y, 0x9A7Eu);
        if (adj > bestAdj || (adj == bestAdj && tie < bestTie)) {
          bestAdj = adj;
          bestTie = tie;
          bestP = p;
        }
      }
    }
    if (bestAdj > 0 && roadReachable(bestP)) {
      const std::uint64_t score = static_cast<std::uint64_t>(bestAdj) * 1000000000ull;
      candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::Waterfront, nextId++, bestP,
                                   static_cast<float>(bestAdj), score));
    }
  }

  // 6) Market (strongest commercial tile).
  if (cfg.includeMarket) {
    int bestMetric = -1;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    Point bestP{0, 0};
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Commercial) continue;
        const int metric = static_cast<int>(t.occupants) * 1000 + static_cast<int>(t.level) * 100;
        const std::uint64_t tie = TieKey(world, cfg, x, y, 0x8A12u);
        if (metric > bestMetric || (metric == bestMetric && tie < bestTie)) {
          bestMetric = metric;
          bestTie = tie;
          bestP = Point{x, y};
        }
      }
    }
    if (bestMetric >= 0) {
      Point road{0, 0};
      if (FindNearestRoadTile(world, cfg, bestP, std::max(world.width(), world.height()), road) && roadReachable(road)) {
        candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::Market, nextId++, road,
                                     static_cast<float>(bestMetric), static_cast<std::uint64_t>(bestMetric) * 100000ull));
      }
    }
  }

  // 7) Works (strongest industrial tile).
  if (cfg.includeIndustry) {
    int bestMetric = -1;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();
    Point bestP{0, 0};
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Industrial) continue;
        const int metric = static_cast<int>(t.occupants) * 1000 + static_cast<int>(t.level) * 100;
        const std::uint64_t tie = TieKey(world, cfg, x, y, 0x70A5u);
        if (metric > bestMetric || (metric == bestMetric && tie < bestTie)) {
          bestMetric = metric;
          bestTie = tie;
          bestP = Point{x, y};
        }
      }
    }
    if (bestMetric >= 0) {
      Point road{0, 0};
      if (FindNearestRoadTile(world, cfg, bestP, std::max(world.width(), world.height()), road) && roadReachable(road)) {
        candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::Works, nextId++, road,
                                     static_cast<float>(bestMetric), static_cast<std::uint64_t>(bestMetric) * 100000ull));
      }
    }
  }

  // 8) District hubs (top-N districts by tile count; choose best closeness node per district).
  if (cfg.includeDistrictHubs && haveCentrality && !g.nodes.empty() && !cent.nodeCloseness.empty()) {
    struct DPick {
      int district = -1;
      int tiles = 0;
      double closeness = -1.0;
      Point pos{0, 0};
      std::uint64_t tie = 0;
    };

    std::vector<int> distTiles(static_cast<std::size_t>(kDistrictCount), 0);
    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const int d = std::clamp(static_cast<int>(world.at(x, y).district), 0, kDistrictCount - 1);
        distTiles[static_cast<std::size_t>(d)]++;
      }
    }

    // Best node per district.
    std::vector<DPick> best(static_cast<std::size_t>(kDistrictCount));
    for (int d = 0; d < kDistrictCount; ++d) {
      best[static_cast<std::size_t>(d)].district = d;
      best[static_cast<std::size_t>(d)].tiles = distTiles[static_cast<std::size_t>(d)];
      best[static_cast<std::size_t>(d)].closeness = -1.0;
      best[static_cast<std::size_t>(d)].tie = std::numeric_limits<std::uint64_t>::max();
    }

    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i) {
      const Point p = g.nodes[static_cast<std::size_t>(i)].pos;
      const int d = std::clamp(static_cast<int>(world.at(p.x, p.y).district), 0, kDistrictCount - 1);
      const double c = cent.nodeCloseness[static_cast<std::size_t>(i)];
      const std::uint64_t tie = TieKey(world, cfg, p.x, p.y, 0xD1B0u);
      DPick& bp = best[static_cast<std::size_t>(d)];
      if (c > bp.closeness || (c == bp.closeness && tie < bp.tie)) {
        bp.closeness = c;
        bp.pos = p;
        bp.tie = tie;
      }
    }

    // Rank districts by size and add up to maxDistrictHubs.
    std::vector<DPick> ranked = best;
    std::sort(ranked.begin(), ranked.end(), [&](const DPick& a, const DPick& b) {
      if (a.tiles != b.tiles) return a.tiles > b.tiles;
      return a.district < b.district;
    });

    int added = 0;
    for (const DPick& d : ranked) {
      if (added >= cfg.maxDistrictHubs) break;
      if (d.closeness <= 0.0) continue;
      if (!roadReachable(d.pos)) continue;
      const std::uint64_t score = static_cast<std::uint64_t>(std::llround(d.closeness * 1.0e12));
      candidates.push_back(MakePoi(world, streets, addresses, districtNames, cfg, PoiKind::DistrictHub, nextId++, d.pos,
                                   static_cast<float>(d.closeness), score));
      ++added;
    }
  }

  // Keep only unique road tiles (rare collisions) and stable order by score.
  std::sort(candidates.begin(), candidates.end(), [&](const Poi& a, const Poi& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.kind != b.kind) return static_cast<int>(a.kind) < static_cast<int>(b.kind);
    const std::uint64_t ta = TieKey(world, cfg, a.roadTile.x, a.roadTile.y, 0x5071u);
    const std::uint64_t tb = TieKey(world, cfg, b.roadTile.x, b.roadTile.y, 0x5071u);
    return ta < tb;
  });

  // Greedy route-aware selection.
  ParcelAddress cur = out.start;
  std::vector<Point> picked;
  picked.reserve(static_cast<std::size_t>(std::max(0, cfg.maxStops)));

  std::vector<std::uint8_t> used(candidates.size(), std::uint8_t{0});
  for (int step = 0; step < cfg.maxStops; ++step) {
    int bestIdx = -1;
    RouteResult bestRoute;
    double bestUtil = -1.0;
    std::uint64_t bestTie = std::numeric_limits<std::uint64_t>::max();

    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
      if (used[static_cast<std::size_t>(i)]) continue;
      const Poi& poi = candidates[static_cast<std::size_t>(i)];

      if (TooClose(picked, poi.roadTile, cfg.minSeparationTiles)) continue;

      ParcelAddress dst = MakeSyntheticEndpoint(poi);
      RouteResult r = RouteBetweenEndpoints(world, streets, cur, dst);
      if (!r.ok) continue;

      const int cost = std::max(1, r.pathCost);
      const double util = static_cast<double>(poi.score) / static_cast<double>(cost + 5);
      const std::uint64_t tie = TieKey(world, cfg, poi.roadTile.x, poi.roadTile.y, 0x7111u);

      if (bestIdx < 0) {
        bestUtil = util;
        bestTie = tie;
        bestIdx = i;
        bestRoute = std::move(r);
        continue;
      }

      const Poi& bestPoi = candidates[static_cast<std::size_t>(bestIdx)];
      if (util > bestUtil || (util == bestUtil && (poi.score > bestPoi.score)) ||
          (util == bestUtil && poi.score == bestPoi.score && tie < bestTie)) {
        bestUtil = util;
        bestTie = tie;
        bestIdx = i;
        bestRoute = std::move(r);
      }
    }

    if (bestIdx < 0) break;

    used[static_cast<std::size_t>(bestIdx)] = 1;
    TourStop ts;
    ts.poi = candidates[static_cast<std::size_t>(bestIdx)];
    ts.routeFromPrev = std::move(bestRoute);
    out.totalSteps += ts.routeFromPrev.pathCost;
    out.stops.push_back(std::move(ts));

    picked.push_back(candidates[static_cast<std::size_t>(bestIdx)].roadTile);
    cur = out.stops.back().routeFromPrev.to;
  }

  return out;
}

TourPosterResult RenderTourPoster(const World& world, const TourPlan& tour, const TourPosterConfig& cfg)
{
  TourPosterResult out;

  // Build a labeled base poster.
  CartographyConfig cartCfg = cfg.cartCfg;
  if (cartCfg.titleOverride.empty()) {
    cartCfg.titleOverride = tour.title;
  }
  CartographyResult base = RenderLabeledIsoPoster(world, cfg.layer, cfg.isoCfg, cfg.streetCfg, cartCfg);
  out.image = std::move(base.image);
  out.title = base.title;

  if (!cfg.drawRoute && !cfg.drawMarkers) return out;

  // Rebuild iso transform to map tile centers into poster space.
  IsoOverviewResult iso = RenderIsoOverview(world, cfg.layer, cfg.isoCfg);

  const int marginTop = cartCfg.poster ? std::max(0, cartCfg.marginTopPx) : 0;
  const int marginSide = cartCfg.poster ? std::max(0, cartCfg.marginSidePx) : 0;
  iso.offsetX += marginSide;
  iso.offsetY += marginTop;

  auto tileCenterPx = [&](const Point& t) -> Point {
    int px = 0;
    int py = 0;
    // Always maps into the same coordinate space as the cartography poster, because we adjusted
    // iso.offsetX/iso.offsetY to account for poster margins.
    (void)IsoTileCenterToPixel(world, iso, t.x, t.y, px, py);
    return Point{px, py};
  };

  auto strokeThick = [&](int x0, int y0, int x1, int y1, int w, Rgba8 c) {
    w = std::max(1, w);

    auto stroke = [&](int ax0, int ay0, int ax1, int ay1) {
      gfx::StrokeLineAA(out.image,
                        static_cast<float>(ax0),
                        static_cast<float>(ay0),
                        static_cast<float>(ax1),
                        static_cast<float>(ay1),
                        c,
                        BlendMode::Alpha);
    };

    // Simple multi-stroke approximation for small widths.
    stroke(x0, y0, x1, y1);
    const int r = w / 2;
    for (int i = 1; i <= r; ++i) {
      stroke(x0 + i, y0, x1 + i, y1);
      stroke(x0 - i, y0, x1 - i, y1);
      stroke(x0, y0 + i, x1, y1 + i);
      stroke(x0, y0 - i, x1, y1 - i);
    }
  };

  // Route overlay.
  if (cfg.drawRoute) {
    const Rgba8 outline{10, 10, 10, static_cast<std::uint8_t>(std::min<int>(255, cfg.routeAlpha))};
    const Rgba8 core{255, 80, 80, cfg.routeAlpha};

    auto drawRouteTiles = [&](const std::vector<Point>& tiles) {
      if (tiles.size() < 2) return;

      auto flush = [&](const Point& aTile, const Point& bTile) {
        const Point a = tileCenterPx(aTile);
        const Point b = tileCenterPx(bTile);
        strokeThick(a.x, a.y, b.x, b.y, cfg.routeLineWidthPx + 2, outline);
        strokeThick(a.x, a.y, b.x, b.y, cfg.routeLineWidthPx, core);
      };

      // Collapse collinear runs into longer segments. This both speeds up export and avoids
      // visible “jitter” where many tiny subpixel strokes accumulate.
      Point segStart = tiles.front();
      Point prev = segStart;
      int dirX = 0;
      int dirY = 0;
      bool haveDir = false;

      for (std::size_t i = 1; i < tiles.size(); ++i) {
        const Point cur = tiles[i];
        int dx = cur.x - prev.x;
        int dy = cur.y - prev.y;
        if (dx == 0 && dy == 0) continue;

        dx = (dx > 0) - (dx < 0);
        dy = (dy > 0) - (dy < 0);

        if (!haveDir) {
          haveDir = true;
          dirX = dx;
          dirY = dy;
        } else if (dx != dirX || dy != dirY) {
          flush(segStart, prev);
          segStart = prev;
          dirX = dx;
          dirY = dy;
        }

        prev = cur;
      }

      if (haveDir) {
        flush(segStart, prev);
      }
    };

    for (std::size_t si = 0; si < tour.stops.size(); ++si) {
      const RouteResult& r = tour.stops[si].routeFromPrev;
      if (!r.ok || r.pathTiles.size() < 2) continue;
      drawRouteTiles(r.pathTiles);
    }
  }

  // Markers (start + stops).
  if (cfg.drawMarkers) {
    const Rgba8 startC{80, 255, 140, cfg.markerAlpha};
    const Rgba8 stopC{255, 230, 120, cfg.markerAlpha};
    const Rgba8 ring{10, 10, 10, static_cast<std::uint8_t>(std::min<int>(255, cfg.markerAlpha))};

    auto drawMarker = [&](const Point& tile, int number, bool isStart) {
      const Point c = tileCenterPx(tile);
      gfx::FillCircleSoft(out.image, static_cast<float>(c.x), static_cast<float>(c.y),
                          static_cast<float>(cfg.markerRadiusPx + 2), 2.5f, ring, BlendMode::Alpha);
      gfx::FillCircleSoft(out.image, static_cast<float>(c.x), static_cast<float>(c.y),
                          static_cast<float>(cfg.markerRadiusPx), 2.0f, isStart ? startC : stopC, BlendMode::Alpha);

      if (cfg.drawStopNumbers && number >= 0) {
        const int sc = 2;
        const std::string text = std::to_string(number);
        const int tw = gfx::MeasureTextWidth5x7(text, sc);
        const int th = gfx::MeasureTextHeight5x7(sc);
        const int tx = c.x - tw / 2;
        const int ty = c.y - th / 2;
        gfx::DrawText5x7Outlined(out.image, tx, ty, text.c_str(), Rgba8{250, 250, 250, 245}, Rgba8{10, 10, 10, 230}, sc);
      }
    };

    drawMarker(tour.start.roadTile, -1, true);
    for (std::size_t i = 0; i < tour.stops.size(); ++i) {
      drawMarker(tour.stops[i].poi.roadTile, static_cast<int>(i + 1), false);
    }
  }

  // Optional key box listing the numbered stops.
  if (cfg.drawKeyBox && !tour.stops.empty()) {
    const int sc = 2;
    std::vector<std::string> lines;
    lines.reserve(tour.stops.size() + 2);
    lines.push_back("Tour Stops");
    for (std::size_t i = 0; i < tour.stops.size(); ++i) {
      std::ostringstream oss;
      oss << (i + 1) << ": " << tour.stops[i].poi.name;
      lines.push_back(oss.str());
    }

    int maxW = 0;
    for (const std::string& l : lines) {
      maxW = std::max(maxW, gfx::MeasureTextWidth5x7(l, sc));
    }
    const int lineH = 16;
    const int pad = 6;
    const int boxW = maxW + pad * 2;
    const int boxH = static_cast<int>(lines.size()) * lineH + pad * 2;

    const int mapX0 = cartCfg.poster ? marginSide : 0;
    const int mapY0 = cartCfg.poster ? marginTop : 0;
    const int mapX1 = out.image.width - 1 - (cartCfg.poster ? marginSide : 0);
    const int x = std::max(mapX0 + 8, mapX1 - boxW - 8);
    const int y = mapY0 + 8;

    const int x1 = x + boxW - 1;
    const int y1 = y + boxH - 1;

    gfx::FillRect(out.image, x, y, x1, y1, Rgba8{20, 20, 20, 140}, BlendMode::Alpha);

    // Minimal 1px stroke (GfxCanvas has FillRect but no built-in stroke helper).
    auto strokeRect1px = [&](int sx0, int sy0, int sx1, int sy1, Rgba8 c) {
      gfx::FillRect(out.image, sx0, sy0, sx1, sy0, c, BlendMode::Alpha);
      gfx::FillRect(out.image, sx0, sy1, sx1, sy1, c, BlendMode::Alpha);
      gfx::FillRect(out.image, sx0, sy0, sx0, sy1, c, BlendMode::Alpha);
      gfx::FillRect(out.image, sx1, sy0, sx1, sy1, c, BlendMode::Alpha);
    };
    strokeRect1px(x, y, x1, y1, Rgba8{0, 0, 0, 180});

    int cy = y + pad;
    for (std::size_t li = 0; li < lines.size(); ++li) {
      const std::string& l = lines[li];
      gfx::DrawText5x7Outlined(out.image, x + pad, cy, l.c_str(), Rgba8{245, 245, 245, 240}, Rgba8{5, 5, 5, 220}, sc);
      cy += lineH;
    }
  }

  return out;
}

} // namespace isocity
