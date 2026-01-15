#include "isocity/StreetNames.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/RoadGraph.hpp"
#include "isocity/Road.hpp"
#include "isocity/World.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/ZoneMetrics.hpp"
#include "isocity/ZoneParcels.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace isocity {

namespace {

// Small 32-bit mix (Murmur3-style finalizer).
inline std::uint32_t HashU32(std::uint32_t v)
{
  v ^= v >> 16;
  v *= 0x7FEB352Du;
  v ^= v >> 15;
  v *= 0x846CA68Bu;
  v ^= v >> 16;
  return v;
}

inline std::uint64_t MixSeed64(std::uint64_t seed, std::uint64_t v)
{
  std::uint64_t s = seed ^ (v + 0x9E3779B97F4A7C15ULL);
  (void)SplitMix64Next(s);
  return s;
}

inline bool IsAlpha(char c)
{
  return std::isalpha(static_cast<unsigned char>(c)) != 0;
}

inline std::string TitleCase(std::string s)
{
  bool start = true;
  for (char& c : s) {
    if (IsAlpha(c)) {
      c = static_cast<char>(start ? std::toupper(static_cast<unsigned char>(c))
                                  : std::tolower(static_cast<unsigned char>(c)));
      start = false;
    } else {
      start = true;
    }
  }
  return s;
}

// 0=N,1=E,2=S,3=W (matches other codebases).
inline int DirFromDelta(int dx, int dy)
{
  if (dx == 0 && dy == -1) return 0;
  if (dx == 1 && dy == 0) return 1;
  if (dx == 0 && dy == 1) return 2;
  if (dx == -1 && dy == 0) return 3;
  return -1;
}

inline int OppDir(int d) { return (d + 2) & 3; }

struct EdgeInfo {
  int id = -1;
  int a = -1;
  int b = -1;
  int lvl = 1;
  int dirAtA = 0; // direction leaving node A along the edge
  int dirAtB = 0; // direction leaving node B along the edge
};

inline bool SamePoint(const Point& a, const Point& b) { return a.x == b.x && a.y == b.y; }

int EdgeDominantLevel(const World& world, const RoadGraphEdge& e)
{
  int best = 1;
  for (const Point& p : e.tiles) {
    if (!world.inBounds(p.x, p.y)) continue;
    const Tile& t = world.at(p.x, p.y);
    if (t.overlay != Overlay::Road) continue;
    best = std::max(best, ClampRoadLevel(static_cast<int>(t.level)));
    if (best >= 3) break;
  }
  return std::clamp(best, 1, 3);
}

// Determine dir leaving a specific node along an edge.
// Returns -1 if the edge doesn't touch the node or is degenerate.
int EdgeDirAtNode(const RoadGraph& g, const RoadGraphEdge& e, int nodeId)
{
  if (nodeId != e.a && nodeId != e.b) return -1;
  const Point np = g.nodes[static_cast<std::size_t>(nodeId)].pos;
  if (e.tiles.size() < 2) return -1;

  const Point front = e.tiles.front();
  const Point back = e.tiles.back();

  if (SamePoint(np, front)) {
    const Point next = e.tiles[1];
    return DirFromDelta(next.x - front.x, next.y - front.y);
  }
  if (SamePoint(np, back)) {
    const Point prev = e.tiles[e.tiles.size() - 2];
    return DirFromDelta(prev.x - back.x, prev.y - back.y);
  }
  // Unexpected ordering; fall back to searching endpoints.
  // (Shouldn't happen if RoadGraph is well-formed, but keep defensive.)
  if (nodeId == e.a) {
    // Find where node A appears.
    for (std::size_t i = 0; i + 1 < e.tiles.size(); ++i) {
      if (SamePoint(e.tiles[i], np)) {
        const Point next = e.tiles[i + 1];
        return DirFromDelta(next.x - np.x, next.y - np.y);
      }
    }
  }
  if (nodeId == e.b) {
    for (std::size_t i = 1; i < e.tiles.size(); ++i) {
      if (SamePoint(e.tiles[i], np)) {
        const Point prev = e.tiles[i - 1];
        return DirFromDelta(prev.x - np.x, prev.y - np.y);
      }
    }
  }
  return -1;
}

// Pick the best continuation edge at a node for a traversal arriving via `incomingEdge`.
// Returns -1 if no good unambiguous continuation exists.
int PickContinuation(const RoadGraph& g, const std::vector<EdgeInfo>& edges, int nodeId, int incomingEdgeId,
                     const StreetNamingConfig& cfg, std::uint32_t seed32)
{
  const RoadGraphNode& node = g.nodes[static_cast<std::size_t>(nodeId)];
  const int deg = static_cast<int>(node.edges.size());
  if (deg <= 1) return -1;

  // Determine incoming heading (direction of travel as we enter the node).
  const EdgeInfo& in = edges[static_cast<std::size_t>(incomingEdgeId)];
  int inDirFromNode = 0;
  if (nodeId == in.a) inDirFromNode = in.dirAtA;
  else if (nodeId == in.b) inDirFromNode = in.dirAtB;
  else return -1;
  const int incomingHeading = OppDir(inDirFromNode);

  const bool allowTurn = (deg == 2) ? cfg.mergeThroughCorners : false;
  const bool allowStraight = cfg.mergeThroughIntersections;

  int bestEdge = -1;
  int bestScore = std::numeric_limits<int>::min();
  bool ambiguous = false;

  for (int ei : node.edges) {
    if (ei < 0) continue;
    if (ei == incomingEdgeId) continue;
    if (static_cast<std::size_t>(ei) >= edges.size()) continue;

    const EdgeInfo& cand = edges[static_cast<std::size_t>(ei)];
    int outDir = -1;
    if (nodeId == cand.a) outDir = cand.dirAtA;
    else if (nodeId == cand.b) outDir = cand.dirAtB;
    if (outDir < 0) continue;

    int dot = 0;
    if (outDir == incomingHeading) dot = 2; // straight
    else if (((outDir + 1) & 3) == incomingHeading || ((outDir + 3) & 3) == incomingHeading) dot = 1; // 90 deg
    else dot = 0; // u-turn/back

    if (dot == 2) {
      if (!allowStraight) continue;
    } else if (dot == 1) {
      if (!allowTurn) continue;
    } else {
      continue;
    }

    // Prefer to continue along higher-class roads.
    const int lvlMin = std::min(in.lvl, cand.lvl);
    const int lvlMax = std::max(in.lvl, cand.lvl);

    // Base score: straightness dominates, then level.
    int score = dot * 100 + lvlMin * 10 + lvlMax;

    // Deterministic tie-breaker based on node position and edge ids.
    const Point p = node.pos;
    const std::uint32_t tie = HashCoords32(p.x, p.y, seed32 ^ HashU32(static_cast<std::uint32_t>(ei * 1315423911u)));
    score = score * 4 - static_cast<int>(tie & 3u);

    if (score > bestScore) {
      bestScore = score;
      bestEdge = ei;
      ambiguous = false;
    } else if (score == bestScore) {
      ambiguous = true;
    }
  }

  if (bestEdge < 0) return -1;
  if (ambiguous) return -1;
  return bestEdge;
}

// -----------------------------------------------------------------------------------------------
// Procedural name generator (pronounceable syllables + a few classic/common bases)
// -----------------------------------------------------------------------------------------------

std::string Ordinal(int n)
{
  const int mod100 = n % 100;
  const int mod10 = n % 10;
  std::string suf = "th";
  if (mod100 < 11 || mod100 > 13) {
    if (mod10 == 1) suf = "st";
    else if (mod10 == 2) suf = "nd";
    else if (mod10 == 3) suf = "rd";
  }
  return std::to_string(n) + suf;
}

std::string PickSuffix(int roadLevel, RNG& rng)
{
  if (roadLevel >= 3) {
    // Highways use special naming.
    return "";
  }
  if (roadLevel == 2) {
    static const char* k[] = {"Ave", "Avenue", "Blvd", "Boulevard", "Pkwy", "Parkway"};
    return k[rng.rangeInt(0, static_cast<int>(sizeof(k) / sizeof(k[0])) - 1)];
  }
  static const char* k[] = {"St", "Street", "Rd", "Road", "Ln", "Lane", "Way", "Dr", "Drive"};
  return k[rng.rangeInt(0, static_cast<int>(sizeof(k) / sizeof(k[0])) - 1)];
}

std::string SyllableName(RNG& rng)
{
  // Small phonotactics-ish tables.
  static const char* onset[] = {
      "b",  "br", "c",  "ch", "cr", "d",  "dr", "f",  "g",  "gr", "h",  "j",  "k",  "l",
      "m",  "n",  "p",  "pr", "qu", "r",  "s",  "sh", "st", "t",  "tr", "v",  "w",  "z"};
  static const char* vowel[] = {"a", "e", "i", "o", "u", "ae", "ai", "ea", "ee", "io", "oa", "oo", "ou", "ui"};
  static const char* coda[] = {"", "n", "m", "r", "s", "t", "nd", "nt", "st", "rd", "ck", "ll", "rn", "sh"};
  static const char* tail[] = {"", "", "", "ton", "field", "wood", "ford", "view", "crest", "haven", "gate", "port"};

  const int syllables = rng.rangeInt(2, 3);
  std::string out;
  out.reserve(16);

  for (int i = 0; i < syllables; ++i) {
    const char* o = onset[rng.rangeInt(0, static_cast<int>(sizeof(onset) / sizeof(onset[0])) - 1)];
    const char* v = vowel[rng.rangeInt(0, static_cast<int>(sizeof(vowel) / sizeof(vowel[0])) - 1)];
    const char* c = coda[rng.rangeInt(0, static_cast<int>(sizeof(coda) / sizeof(coda[0])) - 1)];

    // Avoid double-vowel collisions.
    if (!out.empty() && !std::string(v).empty()) {
      const char last = out.back();
      const char first = v[0];
      auto isV = [](char cc) {
        cc = static_cast<char>(std::tolower(static_cast<unsigned char>(cc)));
        return cc == 'a' || cc == 'e' || cc == 'i' || cc == 'o' || cc == 'u';
      };
      if (isV(last) && isV(first)) {
        // Drop onset occasionally to smooth.
        if (rng.chance(0.6f)) o = "";
      }
    }

    out += o;
    out += v;
    out += c;
  }

  if (rng.chance(0.55f)) {
    out += tail[rng.rangeInt(0, static_cast<int>(sizeof(tail) / sizeof(tail[0])) - 1)];
  }
  return TitleCase(out);
}

std::string CommonBase(RNG& rng)
{
  static const char* k[] = {"Oak", "Maple", "Pine", "Cedar", "Elm", "Birch", "Willow", "Ash", "Spruce", "Juniper",
                             "Park", "Lake", "River", "Hill", "Meadow", "Valley", "Sunset", "Sunrise", "Harbor", "Market",
                             "Broad", "Main", "Center", "Union", "Liberty", "Garden", "Beacon", "Crown", "King", "Queen"};
  return k[rng.rangeInt(0, static_cast<int>(sizeof(k) / sizeof(k[0])) - 1)];
}

std::string HighwayName(std::uint64_t seed, int streetId)
{
  // A tiny deterministic pseudo-route naming.
  std::uint64_t st = MixSeed64(seed, static_cast<std::uint64_t>(streetId) * 0xC0FFEEULL);
  const int kind = static_cast<int>(SplitMix64Next(st) % 3ULL);
  const int num = 1 + static_cast<int>(SplitMix64Next(st) % 399ULL);
  if (kind == 0) return "I-" + std::to_string(num);
  if (kind == 1) return "Rte " + std::to_string(num);
  return "Hwy " + std::to_string(num);
}

std::string GenerateStreetName(std::uint64_t worldSeed, int streetId, int roadLevel, bool allowOrdinal,
                               std::unordered_set<std::string>& used)
{
  if (roadLevel >= 3) {
    std::string hw = HighwayName(worldSeed, streetId);
    used.insert(hw);
    return hw;
  }

  RNG rng(MixSeed64(worldSeed, static_cast<std::uint64_t>(streetId) * 0x9E3779B97F4A7C15ULL));

  // Pick a base name source.
  const float r = rng.nextF01();
  std::string base;

  if (allowOrdinal && r < 0.18f) {
    const int n = 1 + (streetId % 99);
    base = Ordinal(n);
  } else if (r < 0.45f) {
    base = CommonBase(rng);
  } else {
    base = SyllableName(rng);
  }

  // Suffix.
  const std::string suf = PickSuffix(roadLevel, rng);
  std::string name = base;
  if (!suf.empty()) {
    name += " ";
    name += suf;
  }

  // Ensure uniqueness.
  if (used.find(name) == used.end()) {
    used.insert(name);
    return name;
  }

  static const char* dir[] = {"North", "South", "East", "West", "Upper", "Lower"};
  for (int i = 0; i < 8; ++i) {
    const std::string candidate = std::string(dir[rng.rangeInt(0, 5)]) + " " + name;
    if (used.find(candidate) == used.end()) {
      used.insert(candidate);
      return candidate;
    }
  }

  // Last resort: append id.
  name += " #" + std::to_string(streetId);
  used.insert(name);
  return name;
}

inline int Idx(int x, int y, int w)
{
  return y * w + x;
}

} // namespace

StreetNamingResult BuildStreetNames(const World& world, const StreetNamingConfig& cfg)
{
  StreetNamingResult out;

  const int w = world.width();
  const int h = world.height();
  out.w = w;
  out.h = h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.roadTileToStreetId.assign(n, -1);

  const RoadGraph g = BuildRoadGraph(world);
  if (g.edges.empty() || g.nodes.empty()) return out;

  std::vector<EdgeInfo> edges;
  edges.reserve(g.edges.size());

  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];
    EdgeInfo info;
    info.id = ei;
    info.a = e.a;
    info.b = e.b;
    info.lvl = EdgeDominantLevel(world, e);
    info.dirAtA = std::max(0, EdgeDirAtNode(g, e, e.a));
    info.dirAtB = std::max(0, EdgeDirAtNode(g, e, e.b));
    edges.push_back(info);
  }

  std::vector<int> edgeToStreet(edges.size(), -1);

  const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu) ^
                               static_cast<std::uint32_t>((world.seed() >> 32) & 0xFFFFFFFFu) ^ 0x57AEE7u;

  int streetCount = 0;
  out.streets.clear();
  out.streets.reserve(std::min<int>(cfg.maxStreets, static_cast<int>(edges.size())));

  // Visited/stamp array for per-street unique tile aggregation without re-allocating.
  std::vector<int> visitStamp(n, 0);
  int curStamp = 1;

  // Deterministic name uniqueness.
  std::unordered_set<std::string> usedNames;
  usedNames.reserve(1024);

  auto otherNode = [&](const EdgeInfo& e, int nodeId) {
    if (nodeId == e.a) return e.b;
    if (nodeId == e.b) return e.a;
    return -1;
  };

  auto extend = [&](int streetId, int startNode, int incomingEdgeId, bool prepend,
                    std::vector<int>& streetEdges) {
    int nodeId = startNode;
    int inEdge = incomingEdgeId;
    int guard = 0;

    while (guard++ < static_cast<int>(edges.size()) + 8) {
      const int next = PickContinuation(g, edges, nodeId, inEdge, cfg, seed32);
      if (next < 0) break;
      if (static_cast<std::size_t>(next) >= edgeToStreet.size()) break;
      if (edgeToStreet[static_cast<std::size_t>(next)] != -1) break;

      edgeToStreet[static_cast<std::size_t>(next)] = streetId;
      if (prepend) streetEdges.insert(streetEdges.begin(), next);
      else streetEdges.push_back(next);

      const EdgeInfo& ne = edges[static_cast<std::size_t>(next)];
      const int nextNode = otherNode(ne, nodeId);
      if (nextNode < 0) break;

      nodeId = nextNode;
      inEdge = next;
    }
  };

  for (int ei = 0; ei < static_cast<int>(edges.size()); ++ei) {
    if (edgeToStreet[static_cast<std::size_t>(ei)] != -1) continue;
    if (streetCount >= cfg.maxStreets) break;

    const int sid = streetCount++;
    edgeToStreet[static_cast<std::size_t>(ei)] = sid;

    std::vector<int> streetEdges;
    streetEdges.reserve(16);
    streetEdges.push_back(ei);

    const EdgeInfo& seedEdge = edges[static_cast<std::size_t>(ei)];

    // Extend in both directions from the seed edge.
    extend(sid, seedEdge.a, ei, /*prepend=*/true, streetEdges);
    extend(sid, seedEdge.b, ei, /*prepend=*/false, streetEdges);

    // Aggregate street stats + tile mapping.
    StreetInfo si;
    si.id = sid;
    si.edgeCount = static_cast<int>(streetEdges.size());

    int lvl = 1;
    int horiz = 0;
    int vert = 0;
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int maxY = std::numeric_limits<int>::min();

    // Collect the unique tiles belonging to this street (so we can later resolve
    // per-tile ownership by priority).
    std::vector<int> streetTileIdx;
    streetTileIdx.reserve(256);

    // Bump stamp (wrap defensively).
    if (curStamp == std::numeric_limits<int>::max()) {
      std::fill(visitStamp.begin(), visitStamp.end(), 0);
      curStamp = 1;
    }
    const int stamp = curStamp++;

    for (int seid : streetEdges) {
      if (seid < 0 || static_cast<std::size_t>(seid) >= g.edges.size()) continue;
      const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(seid)];
      const int elvl = edges[static_cast<std::size_t>(seid)].lvl;
      lvl = std::max(lvl, elvl);

      // Orientation: infer from endpoints.
      const Point a = g.nodes[static_cast<std::size_t>(e.a)].pos;
      const Point b = g.nodes[static_cast<std::size_t>(e.b)].pos;
      if (a.x == b.x) vert++;
      else if (a.y == b.y) horiz++;

      for (const Point& p : e.tiles) {
        if (!world.inBounds(p.x, p.y)) continue;
        const int idx = Idx(p.x, p.y, w);
        if (idx < 0 || static_cast<std::size_t>(idx) >= n) continue;
        if (visitStamp[static_cast<std::size_t>(idx)] == stamp) {
          continue;
        }
        visitStamp[static_cast<std::size_t>(idx)] = stamp;
        streetTileIdx.push_back(idx);

        si.tileCount++;
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
      }
    }

    if (minX == std::numeric_limits<int>::max()) {
      // Degenerate; skip.
      continue;
    }

    si.roadLevel = std::clamp(lvl, 1, 3);
    si.bboxMin = Point{minX, minY};
    si.bboxMax = Point{maxX, maxY};

    // Choose primary axis for numbering.
    if (horiz >= vert) {
      si.axis = 0;
      si.axisMin = minX;
    } else {
      si.axis = 1;
      si.axisMin = minY;
    }

    si.name = GenerateStreetName(world.seed(), si.id, si.roadLevel, cfg.allowOrdinalNames, usedNames);

    // Resolve per-road-tile ownership: prefer higher-level streets.
    for (int tidx : streetTileIdx) {
      if (tidx < 0 || static_cast<std::size_t>(tidx) >= n) continue;
      const int x = tidx % w;
      const int y = tidx / w;
      if (!world.inBounds(x, y)) continue;
      if (world.at(x, y).overlay != Overlay::Road) continue;

      const int cur = out.roadTileToStreetId[static_cast<std::size_t>(tidx)];
      if (cur < 0 || static_cast<std::size_t>(cur) >= out.streets.size()) {
        out.roadTileToStreetId[static_cast<std::size_t>(tidx)] = sid;
        continue;
      }

      const int curLvl = out.streets[static_cast<std::size_t>(cur)].roadLevel;
      if (si.roadLevel > curLvl) {
        out.roadTileToStreetId[static_cast<std::size_t>(tidx)] = sid;
      } else if (si.roadLevel == curLvl && cur != sid) {
        // Stable tie-break: mix tile + ids.
        const std::uint32_t t = HashCoords32(x, y, seed32 ^ 0x51AEE7u);
        const std::uint32_t ha = HashU32(static_cast<std::uint32_t>(cur) ^ (t * 0x9E3779B1u));
        const std::uint32_t hb = HashU32(static_cast<std::uint32_t>(sid) ^ (t * 0x85EBCA6Bu));
        out.roadTileToStreetId[static_cast<std::size_t>(tidx)] = (hb < ha) ? sid : cur;
      }
    }

    out.streets.push_back(si);
  }

  // Intersections: a road tile can be part of multiple streets; ensure every road tile has a stable assignment.
  // If unassigned (due to degenerate edges), pick the dominant adjacent street.
  constexpr int kDirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (world.at(x, y).overlay != Overlay::Road) continue;
      const std::size_t idx = static_cast<std::size_t>(Idx(x, y, w));
      if (idx >= out.roadTileToStreetId.size()) continue;
      if (out.roadTileToStreetId[idx] != -1) continue;

      int best = -1;
      int bestLvl = -1;
      std::uint32_t bestTie = std::numeric_limits<std::uint32_t>::max();

      for (const auto& d : kDirs) {
        const int nx = x + d[0];
        const int ny = y + d[1];
        if (!world.inBounds(nx, ny)) continue;
        const std::size_t nidx = static_cast<std::size_t>(Idx(nx, ny, w));
        if (nidx >= out.roadTileToStreetId.size()) continue;
        const int sid = out.roadTileToStreetId[nidx];
        if (sid < 0 || static_cast<std::size_t>(sid) >= out.streets.size()) continue;
        const int lvl = out.streets[static_cast<std::size_t>(sid)].roadLevel;
        const std::uint32_t tie = HashCoords32(nx, ny, seed32 ^ 0xA11CEu);
        if (lvl > bestLvl || (lvl == bestLvl && tie < bestTie)) {
          bestLvl = lvl;
          bestTie = tie;
          best = sid;
        }
      }

      if (best >= 0) out.roadTileToStreetId[idx] = best;
    }
  }

  return out;
}

std::vector<ParcelAddress> BuildParcelAddresses(const World& world,
                                                const StreetNamingResult& streets,
                                                const AddressConfig& cfg,
                                                const ZoneAccessMap* precomputedZoneAccess,
                                                const ZoneBuildingParcels* precomputedParcels)
{
  std::vector<ParcelAddress> out;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (streets.roadTileToStreetId.size() != n) return out;

  ZoneAccessMap zoneAccessLocal;
  const ZoneAccessMap* zoneAccess = precomputedZoneAccess;
  if (!zoneAccess || zoneAccess->w != w || zoneAccess->h != h || zoneAccess->roadIdx.size() != n) {
    zoneAccessLocal = BuildZoneAccessMap(world, /*roadToEdgeMask=*/nullptr);
    zoneAccess = &zoneAccessLocal;
  }

  ZoneBuildingParcels parcelsLocal;
  const ZoneBuildingParcels* parcels = precomputedParcels;
  if (!parcels || parcels->width != w || parcels->height != h) {
    BuildZoneBuildingParcels(world, parcelsLocal);
    parcels = &parcelsLocal;
  }

  out.reserve(parcels->parcels.size());

  auto streetIdAtRoad = [&](int rx, int ry) -> int {
    if (rx < 0 || ry < 0 || rx >= w || ry >= h) return -1;
    const std::size_t idx = static_cast<std::size_t>(ry) * static_cast<std::size_t>(w) + static_cast<std::size_t>(rx);
    if (idx >= streets.roadTileToStreetId.size()) return -1;
    return streets.roadTileToStreetId[idx];
  };

  auto pickParcelRoadTile = [&](const ZoneBuildingParcel& p) -> Point {
    // Try to pick a road tile directly adjacent to the parcel's facing edge.
    const int x0 = p.x0;
    const int y0 = p.y0;
    const int x1 = p.x0 + p.w - 1;
    const int y1 = p.y0 + p.h - 1;

    std::vector<Point> candidates;
    candidates.reserve(16);

    auto pushIfRoad = [&](int x, int y) {
      if (!world.inBounds(x, y)) return;
      if (world.at(x, y).overlay != Overlay::Road) return;
      candidates.push_back(Point{x, y});
    };

    const int face = static_cast<int>(p.facing) & 3;
    if (face == 0) { // N
      for (int x = x0; x <= x1; ++x) pushIfRoad(x, y0 - 1);
    } else if (face == 2) { // S
      for (int x = x0; x <= x1; ++x) pushIfRoad(x, y1 + 1);
    } else if (face == 3) { // W
      for (int y = y0; y <= y1; ++y) pushIfRoad(x0 - 1, y);
    } else { // E
      for (int y = y0; y <= y1; ++y) pushIfRoad(x1 + 1, y);
    }

    if (!candidates.empty()) {
      // Deterministic pick: center-ish.
      return candidates[candidates.size() / 2u];
    }

    // Fall back to zone access map at the parcel anchor.
    const int ax = x1;
    const int ay = y1;
    if (world.inBounds(ax, ay)) {
      const std::size_t aidx = static_cast<std::size_t>(ay) * static_cast<std::size_t>(w) + static_cast<std::size_t>(ax);
      if (aidx < zoneAccess->roadIdx.size()) {
        const int ridx = zoneAccess->roadIdx[aidx];
        if (ridx >= 0) {
          const int rx = ridx % w;
          const int ry = ridx / w;
          if (world.inBounds(rx, ry) && world.at(rx, ry).overlay == Overlay::Road) return Point{rx, ry};
        }
      }
    }

    // Final fallback: search from anchor for any adjacent road.
    Point road{};
    if (PickAdjacentRoadTile(world, /*roadToEdgeMask=*/nullptr, ax, ay, road)) {
      return road;
    }
    return Point{ax, ay};
  };

  for (int pi = 0; pi < static_cast<int>(parcels->parcels.size()); ++pi) {
    const ZoneBuildingParcel& p = parcels->parcels[static_cast<std::size_t>(pi)];
    if (!IsZoneOverlay(p.overlay)) continue;
    if (p.w <= 0 || p.h <= 0) continue;

    ParcelAddress a;
    a.parcelIndex = pi;
    a.parcelAnchor = Point{p.x0 + p.w - 1, p.y0 + p.h - 1};

    const Point road = pickParcelRoadTile(p);
    a.roadTile = road;

    const int sid = streetIdAtRoad(road.x, road.y);
    a.streetId = sid;

    const bool sidOk = (sid >= 0 && static_cast<std::size_t>(sid) < streets.streets.size());
    const StreetInfo* st = sidOk ? &streets.streets[static_cast<std::size_t>(sid)] : nullptr;
    a.streetName = st ? st->name : std::string("Unnamed Rd");

    // House number along the primary axis.
    int axis = 0;
    int axisMin = 0;
    if (st) {
      axis = st->axis;
      axisMin = st->axisMin;
    } else {
      // Infer from local road orientation.
      axis = 0;
      axisMin = std::min(road.x, road.y);
    }

    const int coord = (axis == 0) ? road.x : road.y;
    const int base = std::max(1, (coord - axisMin + 1)) * std::max(1, cfg.numberStep);

    // Side parity based on parcel centroid relative to road.
    const float cx = static_cast<float>(p.x0) + 0.5f * static_cast<float>(std::max(1, p.w - 1));
    const float cy = static_cast<float>(p.y0) + 0.5f * static_cast<float>(std::max(1, p.h - 1));
    bool odd = false;
    if (axis == 0) {
      odd = (cy < static_cast<float>(road.y));
    } else {
      odd = (cx < static_cast<float>(road.x));
    }
    if (cfg.flipParity) odd = !odd;

    a.houseNumber = base + (odd ? 1 : 0);
    a.full = std::to_string(a.houseNumber) + " " + a.streetName;

    out.push_back(a);
  }

  // Deterministic order: sort by street then number.
  std::sort(out.begin(), out.end(), [](const ParcelAddress& a, const ParcelAddress& b) {
    if (a.streetId != b.streetId) return a.streetId < b.streetId;
    if (a.houseNumber != b.houseNumber) return a.houseNumber < b.houseNumber;
    return a.parcelIndex < b.parcelIndex;
  });

  return out;
}

} // namespace isocity
