#include "isocity/Wayfinding.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace isocity {

namespace {

static std::string Trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a])) != 0) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1])) != 0) --b;
  return s.substr(a, b - a);
}

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool IsAlphaNum(char c)
{
  const unsigned char uc = static_cast<unsigned char>(c);
  return std::isalnum(uc) != 0;
}

static std::vector<std::string> SplitTokensAlphaNumLower(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (IsAlphaNum(c)) {
      cur.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    } else {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

static std::string CanonSuffixToken(const std::string& t)
{
  // Canonicalize a few common street suffixes so user queries like "Avenue" vs "Ave"
  // can match the same generated street.
  if (t == "street" || t == "st") return "st";
  if (t == "avenue" || t == "ave" || t == "av") return "ave";
  if (t == "road" || t == "rd") return "rd";
  if (t == "boulevard" || t == "blvd") return "blvd";
  if (t == "lane" || t == "ln") return "ln";
  if (t == "drive" || t == "dr") return "dr";
  if (t == "way") return "way";
  if (t == "parkway" || t == "pkwy") return "pkwy";
  if (t == "highway" || t == "hwy") return "hwy";
  return t;
}

static std::string NormalizeStreetKey(const std::string& streetDisplay)
{
  const std::vector<std::string> tokens = SplitTokensAlphaNumLower(streetDisplay);
  std::string out;
  out.reserve(streetDisplay.size());
  for (const std::string& t0 : tokens) {
    const std::string t = CanonSuffixToken(t0);
    if (t.empty()) continue;
    if (!out.empty()) out.push_back(' ');
    out += t;
  }
  return out;
}

static bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

static bool ParsePointXY(const std::string& s, Point& out)
{
  const std::size_t pos = s.find(',');
  if (pos == std::string::npos) return false;
  int x = 0;
  int y = 0;
  if (!ParseI32(Trim(s.substr(0, pos)), &x)) return false;
  if (!ParseI32(Trim(s.substr(pos + 1)), &y)) return false;
  out = Point{x, y};
  return true;
}

static int LevenshteinDistance(const std::string& a, const std::string& b)
{
  const std::size_t n = a.size();
  const std::size_t m = b.size();
  if (n == 0) return static_cast<int>(m);
  if (m == 0) return static_cast<int>(n);

  std::vector<int> prev(m + 1);
  std::vector<int> cur(m + 1);
  for (std::size_t j = 0; j <= m; ++j) prev[j] = static_cast<int>(j);

  for (std::size_t i = 1; i <= n; ++i) {
    cur[0] = static_cast<int>(i);
    for (std::size_t j = 1; j <= m; ++j) {
      const int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
      const int del = prev[j] + 1;
      const int ins = cur[j - 1] + 1;
      const int sub = prev[j - 1] + cost;
      cur[j] = std::min({del, ins, sub});
    }
    prev.swap(cur);
  }

  return prev[m];
}

struct StreetMatch {
  bool ok = false;
  int streetIndex = -1; // index into AddressIndex.streetKeys
  int bestDist = 0;
  std::vector<std::string> suggestions;
  std::string error;
};

static StreetMatch MatchStreetKey(const AddressIndex& index, const std::string& streetQuery)
{
  StreetMatch r;

  const std::string qKey = NormalizeStreetKey(streetQuery);
  if (qKey.empty()) {
    r.error = "Empty street name";
    return r;
  }

  const auto it = index.keyToStreet.find(qKey);
  if (it != index.keyToStreet.end()) {
    r.ok = true;
    r.streetIndex = it->second;
    r.bestDist = 0;
    return r;
  }

  if (!index.cfg.allowFuzzy || index.streetKeys.empty()) {
    r.error = "Unknown street";
    return r;
  }

  struct Cand {
    int dist = 0;
    int idx = -1;
  };

  std::vector<Cand> cands;
  cands.reserve(index.streetKeys.size());

  int bestD = std::numeric_limits<int>::max();
  int bestI = -1;

  for (std::size_t i = 0; i < index.streetKeys.size(); ++i) {
    const std::string& k = index.streetKeys[i];
    const int d = LevenshteinDistance(qKey, k);
    cands.push_back(Cand{d, static_cast<int>(i)});
    if (d < bestD) {
      bestD = d;
      bestI = static_cast<int>(i);
    } else if (d == bestD) {
      // Deterministic tie-break: lexicographic key.
      if (bestI >= 0 && k < index.streetKeys[static_cast<std::size_t>(bestI)]) {
        bestI = static_cast<int>(i);
      }
    }
  }

  std::sort(cands.begin(), cands.end(), [&](const Cand& a, const Cand& b) {
    if (a.dist != b.dist) return a.dist < b.dist;
    const std::string& ka = index.streetKeys[static_cast<std::size_t>(a.idx)];
    const std::string& kb = index.streetKeys[static_cast<std::size_t>(b.idx)];
    return ka < kb;
  });

  const int suggestN = std::max(0, index.cfg.maxSuggestions);
  for (int i = 0; i < suggestN && i < static_cast<int>(cands.size()); ++i) {
    const int si = cands[static_cast<std::size_t>(i)].idx;
    if (si >= 0 && static_cast<std::size_t>(si) < index.streetDisplay.size())
      r.suggestions.push_back(index.streetDisplay[static_cast<std::size_t>(si)]);
  }

  const int maxAuto = index.cfg.maxAutoEditDistance;
  if (bestI >= 0 && (maxAuto <= 0 || bestD <= maxAuto)) {
    r.ok = true;
    r.streetIndex = bestI;
    r.bestDist = bestD;
    return r;
  }

  std::ostringstream oss;
  oss << "Unknown street '" << streetQuery << "'";
  r.error = oss.str();
  return r;
}

static int StreetIdForStreetKey(const AddressIndex& index, int streetIndex)
{
  if (streetIndex < 0 || static_cast<std::size_t>(streetIndex) >= index.streetToAddress.size()) return -1;
  const std::vector<int>& addrIdx = index.streetToAddress[static_cast<std::size_t>(streetIndex)];
  int best = std::numeric_limits<int>::max();
  for (int ai : addrIdx) {
    if (ai < 0 || static_cast<std::size_t>(ai) >= index.addresses.size()) continue;
    const int sid = index.addresses[static_cast<std::size_t>(ai)].streetId;
    if (sid >= 0) best = std::min(best, sid);
  }
  return (best == std::numeric_limits<int>::max()) ? -1 : best;
}

static bool FindIntersectionRoadTile(const StreetNamingResult& streets, int streetA, int streetB,
                                    std::uint32_t seed32, Point& out)
{
  if (streetA < 0 || streetB < 0) return false;
  if (streetA == streetB) return false;

  const int w = streets.w;
  const int h = streets.h;
  if (w <= 0 || h <= 0) return false;

  const std::vector<int>& map = streets.roadTileToStreetId;
  if (static_cast<int>(map.size()) != w * h) return false;

  bool found = false;
  std::uint32_t bestHash = 0;

  auto tryCandidate = [&](int x, int y) {
    const std::uint32_t hv = HashCoords32(x, y, seed32 ^ 0x1D1E1F20u);
    if (!found || hv < bestHash) {
      bestHash = hv;
      out = Point{x, y};
      found = true;
    }
  };

  auto sidAt = [&](int x, int y) -> int {
    if (x < 0 || y < 0 || x >= w || y >= h) return -1;
    return map[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int sid = sidAt(x, y);
      if (sid != streetA) continue;
      const int n = sidAt(x, y - 1);
      const int e = sidAt(x + 1, y);
      const int s = sidAt(x, y + 1);
      const int wv = sidAt(x - 1, y);
      if (n == streetB || e == streetB || s == streetB || wv == streetB) {
        tryCandidate(x, y);
      }
    }
  }

  return found;
}

static bool FindNearestRoadTile(const World& world, Point p, Point& out)
{
  if (!world.inBounds(p.x, p.y)) return false;
  if (world.at(p.x, p.y).overlay == Overlay::Road) {
    out = p;
    return true;
  }

  const int w = world.width();
  const int h = world.height();
  const int maxR = std::max(w, h);

  const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() ^ 0xA1B2C3D4u);

  for (int r = 1; r <= maxR; ++r) {
    bool found = false;
    std::uint32_t bestHash = 0;

    auto consider = [&](int x, int y) {
      if (!world.inBounds(x, y)) return;
      if (world.at(x, y).overlay != Overlay::Road) return;
      const std::uint32_t hv = HashCoords32(x, y, seed32 ^ 0xBEEF1234u);
      if (!found || hv < bestHash) {
        bestHash = hv;
        out = Point{x, y};
        found = true;
      }
    };

    // Manhattan ring (diamond) distance r.
    for (int dy = -r; dy <= r; ++dy) {
      const int dx = r - std::abs(dy);
      consider(p.x + dx, p.y + dy);
      if (dx != 0) consider(p.x - dx, p.y + dy);
    }

    if (found) return true;
  }

  return false;
}

static bool ParseLeadingHouseNumber(const std::string& s, int& outNumber, std::string& outRest)
{
  outNumber = 0;
  outRest = Trim(s);
  if (outRest.empty()) return false;

  std::size_t i = 0;
  while (i < outRest.size() && std::isspace(static_cast<unsigned char>(outRest[i])) != 0) ++i;

  std::size_t j = i;
  while (j < outRest.size() && std::isdigit(static_cast<unsigned char>(outRest[j])) != 0) ++j;

  if (j == i) {
    // No leading number.
    return true;
  }

  const std::string numStr = outRest.substr(i, j - i);
  int num = 0;
  if (!ParseI32(numStr, &num) || num < 0) {
    return false;
  }

  outNumber = num;
  outRest = Trim(outRest.substr(j));
  return true;
}

static int BearingFromDelta(int dx, int dy)
{
  // Bearings clockwise from true north.
  if (dx == 0 && dy == -1) return 0;
  if (dx == 1 && dy == 0) return 90;
  if (dx == 0 && dy == 1) return 180;
  if (dx == -1 && dy == 0) return 270;

  // Fallback for non-cardinal (shouldn't happen for road-tile paths).
  // Choose the dominant axis.
  if (std::abs(dx) >= std::abs(dy)) return (dx >= 0) ? 90 : 270;
  return (dy >= 0) ? 180 : 0;
}

static std::string CardinalNameFromBearing(int bearing)
{
  const int b = ((bearing % 360) + 360) % 360;
  if (b >= 315 || b < 45) return "north";
  if (b >= 45 && b < 135) return "east";
  if (b >= 135 && b < 225) return "south";
  return "west";
}

static std::string TurnModifier(int bearingBefore, int bearingAfter)
{
  const int b0 = ((bearingBefore % 360) + 360) % 360;
  const int b1 = ((bearingAfter % 360) + 360) % 360;
  const int diff = (b1 - b0 + 360) % 360;

  if (diff == 0) return "straight";
  if (diff == 90) return "right";
  if (diff == 270) return "left";
  if (diff == 180) return "uturn";

  // Should not happen for 4-neighborhood.
  return (diff < 180) ? "right" : "left";
}

static std::string StreetNameForId(const StreetNamingResult& streets, int streetId)
{
  if (streetId >= 0 && static_cast<std::size_t>(streetId) < streets.streets.size()) {
    const StreetInfo& si = streets.streets[static_cast<std::size_t>(streetId)];
    if (si.id == streetId) return si.name;
  }

  // Fallback: search.
  for (const StreetInfo& si : streets.streets) {
    if (si.id == streetId) return si.name;
  }
  return "Unnamed Road";
}

static bool ComputeDestinationSide(const ParcelAddress& to, const Point& goalRoad, int finalBearing, std::string& outSide)
{
  outSide.clear();
  if (to.parcelIndex < 0) return false;

  const int dxp = to.parcelAnchor.x - goalRoad.x;
  const int dyp = to.parcelAnchor.y - goalRoad.y;

  if (dxp == 0 && dyp == 0) return false;

  int hx = 0, hy = 0;
  const int b = ((finalBearing % 360) + 360) % 360;
  if (b >= 315 || b < 45) {
    hx = 0; hy = -1;
  } else if (b >= 45 && b < 135) {
    hx = 1; hy = 0;
  } else if (b >= 135 && b < 225) {
    hx = 0; hy = 1;
  } else {
    hx = -1; hy = 0;
  }

  // In screen coords (x right, y down), cross < 0 => left, cross > 0 => right.
  const int cross = hx * dyp - hy * dxp;
  if (cross < 0) {
    outSide = "left";
    return true;
  }
  if (cross > 0) {
    outSide = "right";
    return true;
  }
  return false;
}

} // namespace

AddressIndex BuildAddressIndex(const std::vector<ParcelAddress>& addresses, const AddressIndexConfig& cfg)
{
  AddressIndex idx;
  idx.cfg = cfg;
  idx.addresses = addresses;

  // Deterministic insertion order: scan addresses in given order.
  for (std::size_t ai = 0; ai < addresses.size(); ++ai) {
    const ParcelAddress& a = addresses[ai];
    const std::string key = NormalizeStreetKey(a.streetName);
    if (key.empty()) continue;

    auto it = idx.keyToStreet.find(key);
    int si = -1;
    if (it == idx.keyToStreet.end()) {
      si = static_cast<int>(idx.streetKeys.size());
      idx.keyToStreet[key] = si;
      idx.streetKeys.push_back(key);
      idx.streetDisplay.push_back(a.streetName);
      idx.streetToAddress.push_back({});
    } else {
      si = it->second;
    }

    if (si >= 0 && static_cast<std::size_t>(si) < idx.streetToAddress.size()) {
      idx.streetToAddress[static_cast<std::size_t>(si)].push_back(static_cast<int>(ai));
    }
  }

  // Sort each street's address list deterministically by house number then parcel index.
  for (std::vector<int>& list : idx.streetToAddress) {
    std::sort(list.begin(), list.end(), [&](int a, int b) {
      const ParcelAddress& A = idx.addresses[static_cast<std::size_t>(a)];
      const ParcelAddress& B = idx.addresses[static_cast<std::size_t>(b)];
      if (A.houseNumber != B.houseNumber) return A.houseNumber < B.houseNumber;
      if (A.parcelIndex != B.parcelIndex) return A.parcelIndex < B.parcelIndex;
      if (A.roadTile.y != B.roadTile.y) return A.roadTile.y < B.roadTile.y;
      return A.roadTile.x < B.roadTile.x;
    });
  }

  return idx;
}

GeocodeMatch GeocodeEndpoint(const World& world, const StreetNamingResult& streets,
                             const AddressIndex& index, const std::string& query)
{
  GeocodeMatch out;
  const std::string qTrim = Trim(query);
  if (qTrim.empty()) {
    out.error = "Empty query";
    return out;
  }

  // Coordinate endpoint? "x,y"
  Point xy{0, 0};
  if (ParsePointXY(qTrim, xy)) {
    if (!world.inBounds(xy.x, xy.y)) {
      out.error = "Coordinate is out of bounds";
      return out;
    }

    Point road{0, 0};
    if (!FindNearestRoadTile(world, xy, road)) {
      out.error = "No road tile found near coordinate";
      return out;
    }

    out.ok = true;
    out.addressIndex = -1;
    out.endpoint.parcelIndex = -1;
    out.endpoint.streetId = -1;
    out.endpoint.houseNumber = 0;
    out.endpoint.streetName = "";
    out.endpoint.full = qTrim;
    out.endpoint.roadTile = road;
    out.endpoint.parcelAnchor = road;
    return out;
  }

  // Intersection endpoint? (supports '&', '@', or ' and ')
  {
    const std::string qLower = ToLower(qTrim);
    std::size_t sep = qLower.find('@');
    std::size_t sepLen = 1;
    if (sep == std::string::npos) {
      sep = qLower.find('&');
      sepLen = 1;
    }
    if (sep == std::string::npos) {
      sep = qLower.find(" and ");
      sepLen = 5;
    }

    if (sep != std::string::npos) {
      const std::string a = Trim(qTrim.substr(0, sep));
      const std::string b = Trim(qTrim.substr(sep + sepLen));
      if (a.empty() || b.empty()) {
        out.error = "Invalid intersection query (expected 'Street A & Street B')";
        return out;
      }

      const StreetMatch ma = MatchStreetKey(index, a);
      const StreetMatch mb = MatchStreetKey(index, b);

      if (!ma.ok || !mb.ok) {
        out.error = !ma.ok ? ma.error : mb.error;
        // Merge suggestions.
        out.suggestions = !ma.suggestions.empty() ? ma.suggestions : mb.suggestions;
        return out;
      }

      const int streetIdA = StreetIdForStreetKey(index, ma.streetIndex);
      const int streetIdB = StreetIdForStreetKey(index, mb.streetIndex);

      Point inter{0, 0};
      const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() ^ 0x0BADF00Du);
      if (!FindIntersectionRoadTile(streets, streetIdA, streetIdB, seed32, inter)) {
        out.error = "No intersection found between those streets";
        return out;
      }

      out.ok = true;
      out.addressIndex = -1;
      out.endpoint.parcelIndex = -1;
      out.endpoint.streetId = -1;
      out.endpoint.houseNumber = 0;
      out.endpoint.streetName = index.streetDisplay[static_cast<std::size_t>(ma.streetIndex)] +
                                " & " + index.streetDisplay[static_cast<std::size_t>(mb.streetIndex)];
      out.endpoint.full = out.endpoint.streetName;
      out.endpoint.roadTile = inter;
      out.endpoint.parcelAnchor = inter;
      return out;
    }
  }

  // Address endpoint: optional leading number + street.
  int number = 0;
  std::string streetPart;
  if (!ParseLeadingHouseNumber(qTrim, number, streetPart)) {
    out.error = "Invalid leading house number";
    return out;
  }

  if (streetPart.empty()) {
    // No explicit number provided; treat the entire query as a street name.
    streetPart = qTrim;
    number = 0;
  }

  const StreetMatch sm = MatchStreetKey(index, streetPart);
  if (!sm.ok) {
    out.error = sm.error;
    out.suggestions = sm.suggestions;
    return out;
  }

  if (sm.streetIndex < 0 || static_cast<std::size_t>(sm.streetIndex) >= index.streetToAddress.size()) {
    out.error = "Internal error: street index out of range";
    return out;
  }

  const std::vector<int>& candidates = index.streetToAddress[static_cast<std::size_t>(sm.streetIndex)];
  if (candidates.empty()) {
    out.error = "No addresses found on that street";
    return out;
  }

  int bestAddrIdx = candidates.front();
  if (number > 0) {
    int bestDiff = std::numeric_limits<int>::max();
    for (int ai : candidates) {
      const ParcelAddress& pa = index.addresses[static_cast<std::size_t>(ai)];
      const int d = std::abs(pa.houseNumber - number);
      if (d < bestDiff) {
        bestDiff = d;
        bestAddrIdx = ai;
      } else if (d == bestDiff) {
        // Tie-break deterministically.
        if (pa.houseNumber < index.addresses[static_cast<std::size_t>(bestAddrIdx)].houseNumber) {
          bestAddrIdx = ai;
        } else if (pa.houseNumber == index.addresses[static_cast<std::size_t>(bestAddrIdx)].houseNumber &&
                   pa.parcelIndex < index.addresses[static_cast<std::size_t>(bestAddrIdx)].parcelIndex) {
          bestAddrIdx = ai;
        }
      }
    }
  } else {
    // No number specified => pick median by houseNumber.
    bestAddrIdx = candidates[candidates.size() / 2u];
  }

  if (bestAddrIdx < 0 || static_cast<std::size_t>(bestAddrIdx) >= index.addresses.size()) {
    out.error = "Internal error: address index out of range";
    return out;
  }

  out.ok = true;
  out.addressIndex = bestAddrIdx;
  out.endpoint = index.addresses[static_cast<std::size_t>(bestAddrIdx)];
  return out;
}

RouteResult RouteBetweenEndpoints(const World& world, const StreetNamingResult& streets,
                                  const ParcelAddress& from, const ParcelAddress& to)
{
  RouteResult rr;
  rr.from = from;
  rr.to = to;

  rr.startRoad = from.roadTile;
  rr.goalRoad = to.roadTile;

  if (!world.inBounds(rr.startRoad.x, rr.startRoad.y) || !world.inBounds(rr.goalRoad.x, rr.goalRoad.y)) {
    rr.ok = false;
    rr.error = "Start or goal road tile is out of bounds";
    return rr;
  }

  if (world.at(rr.startRoad.x, rr.startRoad.y).overlay != Overlay::Road ||
      world.at(rr.goalRoad.x, rr.goalRoad.y).overlay != Overlay::Road) {
    rr.ok = false;
    rr.error = "Start or goal is not a road tile";
    return rr;
  }

  std::vector<Point> path;
  int cost = 0;
  if (!FindRoadPathAStar(world, rr.startRoad, rr.goalRoad, path, &cost) || path.size() < 2) {
    rr.ok = false;
    rr.error = "No road path found";
    return rr;
  }

  rr.pathTiles = path;
  rr.pathCost = cost;

  const int w = streets.w;
  const int h = streets.h;

  auto sidAt = [&](const Point& p) -> int {
    if (p.x < 0 || p.y < 0 || p.x >= w || p.y >= h) return -1;
    const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(p.x);
    if (idx >= streets.roadTileToStreetId.size()) return -1;
    return streets.roadTileToStreetId[idx];
  };

  // Edge-based street ids.
  const int edges = static_cast<int>(path.size()) - 1;
  std::vector<int> edgeStreetId(static_cast<std::size_t>(edges), -1);
  for (int i = 0; i < edges; ++i) {
    int sid = sidAt(path[static_cast<std::size_t>(i)]);
    if (sid < 0) sid = sidAt(path[static_cast<std::size_t>(i + 1)]);
    edgeStreetId[static_cast<std::size_t>(i)] = sid;
  }

  // Group contiguous edges by street id.
  struct Seg {
    int streetId = -1;
    int startEdge = 0;
    int endEdge = 0; // inclusive
    int bearing = 0;
  };

  std::vector<Seg> segs;
  segs.reserve(32);

  int curSid = edgeStreetId[0];
  int startE = 0;

  for (int i = 1; i <= edges; ++i) {
    const int sid = (i < edges) ? edgeStreetId[static_cast<std::size_t>(i)] : std::numeric_limits<int>::min();
    if (i == edges || sid != curSid) {
      Seg s;
      s.streetId = curSid;
      s.startEdge = startE;
      s.endEdge = i - 1;

      // Bearing from first edge.
      const Point p0 = path[static_cast<std::size_t>(s.startEdge)];
      const Point p1 = path[static_cast<std::size_t>(s.startEdge + 1)];
      s.bearing = BearingFromDelta(p1.x - p0.x, p1.y - p0.y);

      segs.push_back(s);
      startE = i;
      curSid = sid;
    }
  }

  // Convert segments to maneuvers.
  rr.maneuvers.clear();
  rr.maneuvers.reserve(segs.size() + 1);

  int prevBearing = segs.empty() ? 0 : segs.front().bearing;

  for (std::size_t si = 0; si < segs.size(); ++si) {
    const Seg& s = segs[si];

    RouteManeuver m;

    const int steps = s.endEdge - s.startEdge + 1;
    m.steps = std::max(0, steps);
    m.streetId = s.streetId;
    m.streetName = StreetNameForId(streets, s.streetId);
    m.pathStart = s.startEdge;
    m.pathEnd = s.endEdge + 1;

    const int curBearing = s.bearing;

    if (si == 0) {
      m.type = "depart";
      m.modifier = "";
      m.bearingBefore = curBearing;
      m.bearingAfter = curBearing;

      std::ostringstream oss;
      oss << "Head " << CardinalNameFromBearing(curBearing) << " on " << m.streetName;
      if (m.steps == 1) oss << " for 1 block.";
      else oss << " for " << m.steps << " blocks.";
      m.instruction = oss.str();
    } else {
      m.bearingBefore = prevBearing;
      m.bearingAfter = curBearing;
      m.modifier = TurnModifier(prevBearing, curBearing);

      if (m.modifier == "straight") {
        m.type = "continue";
        std::ostringstream oss;
        oss << "Continue straight on " << m.streetName;
        if (m.steps == 1) oss << " for 1 block.";
        else oss << " for " << m.steps << " blocks.";
        m.instruction = oss.str();
      } else if (m.modifier == "uturn") {
        m.type = "turn";
        std::ostringstream oss;
        oss << "Make a U-turn onto " << m.streetName;
        if (m.steps == 1) oss << " for 1 block.";
        else oss << " for " << m.steps << " blocks.";
        m.instruction = oss.str();
      } else {
        m.type = "turn";
        std::ostringstream oss;
        oss << "Turn " << m.modifier << " onto " << m.streetName;
        if (m.steps == 1) oss << " for 1 block.";
        else oss << " for " << m.steps << " blocks.";
        m.instruction = oss.str();
      }
    }

    rr.maneuvers.push_back(m);
    prevBearing = curBearing;
  }

  // Final arrive maneuver.
  {
    RouteManeuver a;
    a.type = "arrive";
    a.modifier = "";
    a.bearingBefore = prevBearing;
    a.bearingAfter = prevBearing;
    a.steps = 0;
    a.streetId = -1;
    a.streetName = "";
    a.pathStart = static_cast<int>(path.size()) - 1;
    a.pathEnd = a.pathStart;

    std::ostringstream oss;
    oss << "Arrive at " << (to.full.empty() ? "destination" : to.full) << ".";

    std::string side;
    if (ComputeDestinationSide(to, rr.goalRoad, prevBearing, side)) {
      oss << " Destination will be on your " << side << ".";
    }

    a.instruction = oss.str();
    rr.maneuvers.push_back(a);
  }

  rr.ok = true;
  rr.error.clear();
  return rr;
}

} // namespace isocity
