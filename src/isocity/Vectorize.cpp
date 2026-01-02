#include "isocity/Vectorize.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace isocity {

namespace {

struct Edge {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
};

struct Ring {
  std::vector<IPoint> pts; // closed
  double signedArea = 0.0;
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;
};

inline std::uint64_t Key(int x, int y)
{
  // All contour vertices are within [0,w]x[0,h] so non-negative.
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32u) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
}

inline int Sign(double v) { return (v >= 0.0) ? 1 : -1; }

double SignedAreaClosed(const std::vector<IPoint>& ring)
{
  // Shoelace. Expects ring is closed (last == first).
  if (ring.size() < 4) return 0.0;
  long long acc = 0;
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    const IPoint& a = ring[i];
    const IPoint& b = ring[i + 1];
    acc += static_cast<long long>(a.x) * static_cast<long long>(b.y) -
           static_cast<long long>(b.x) * static_cast<long long>(a.y);
  }
  return static_cast<double>(acc) * 0.5;
}

inline bool CollinearSameDir(const IPoint& a, const IPoint& b, const IPoint& c)
{
  const int dx1 = b.x - a.x;
  const int dy1 = b.y - a.y;
  const int dx2 = c.x - b.x;
  const int dy2 = c.y - b.y;
  const int cross = dx1 * dy2 - dy1 * dx2;
  if (cross != 0) return false;
  const int dot = dx1 * dx2 + dy1 * dy2;
  return dot > 0;
}

void SimplifyRing(std::vector<IPoint>& ring)
{
  // Removes collinear points to dramatically reduce vertex count for long grid edges.
  // Keeps closure.
  if (ring.size() < 5) return;
  if (ring.front() != ring.back()) return;

  std::vector<IPoint> pts(ring.begin(), ring.end() - 1); // open
  if (pts.size() < 4) return;

  // Pick a deterministic corner as the start so wrap-around is not collinear.
  const std::size_t n = pts.size();
  std::size_t best = 0;
  bool found = false;
  for (std::size_t i = 0; i < n; ++i) {
    const IPoint& prev = pts[(i + n - 1) % n];
    const IPoint& cur = pts[i];
    const IPoint& next = pts[(i + 1) % n];
    if (!CollinearSameDir(prev, cur, next)) {
      if (!found || (cur.x < pts[best].x) || (cur.x == pts[best].x && cur.y < pts[best].y)) {
        best = i;
        found = true;
      }
    }
  }
  if (found && best != 0) {
    std::rotate(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(best), pts.end());
  }

  // Linear simplification (start is a corner so wrap-around is safe).
  std::vector<IPoint> out;
  out.reserve(pts.size());
  for (const IPoint& p : pts) {
    out.push_back(p);
    while (out.size() >= 3) {
      const std::size_t k = out.size();
      if (CollinearSameDir(out[k - 3], out[k - 2], out[k - 1])) {
        out.erase(out.end() - 2);
      } else {
        break;
      }
    }
  }

  // Ensure we didn't degenerate.
  if (out.size() < 4) return;

  ring = out;
  ring.push_back(ring.front());
}

inline int DirIndex(int dx, int dy)
{
  // Order: up, right, down, left (clockwise in screen coords: +y is down).
  if (dx == 0 && dy == -1) return 0;
  if (dx == 1 && dy == 0) return 1;
  if (dx == 0 && dy == 1) return 2;
  if (dx == -1 && dy == 0) return 3;
  return -1;
}

int PickNextEdge(const std::vector<int>& candidates,
                 const std::vector<Edge>& edges,
                 const std::vector<std::uint8_t>& used,
                 int prevDx,
                 int prevDy)
{
  int prevDir = DirIndex(prevDx, prevDy);
  if (prevDir < 0) {
    for (int idx : candidates) {
      if (idx < 0 || idx >= static_cast<int>(edges.size())) continue;
      if (used[static_cast<std::size_t>(idx)]) continue;
      return idx;
    }
    return -1;
  }

  const int want[4] = {
      (prevDir + 3) % 4, // left turn
      prevDir,           // straight
      (prevDir + 1) % 4, // right turn
      (prevDir + 2) % 4, // back
  };

  for (int k = 0; k < 4; ++k) {
    const int d = want[k];
    for (int idx : candidates) {
      if (idx < 0 || idx >= static_cast<int>(edges.size())) continue;
      if (used[static_cast<std::size_t>(idx)]) continue;
      const Edge& e = edges[static_cast<std::size_t>(idx)];
      const int dir = DirIndex(e.x1 - e.x0, e.y1 - e.y0);
      if (dir == d) return idx;
    }
  }

  // Fallback: any unused.
  for (int idx : candidates) {
    if (idx < 0 || idx >= static_cast<int>(edges.size())) continue;
    if (used[static_cast<std::size_t>(idx)]) continue;
    return idx;
  }
  return -1;
}

inline bool PointOnSegment(const IPoint& p, const IPoint& a, const IPoint& b)
{
  const int dx1 = p.x - a.x;
  const int dy1 = p.y - a.y;
  const int dx2 = b.x - a.x;
  const int dy2 = b.y - a.y;
  if (dx1 * dy2 - dy1 * dx2 != 0) return false;
  const int minX = std::min(a.x, b.x);
  const int maxX = std::max(a.x, b.x);
  const int minY = std::min(a.y, b.y);
  const int maxY = std::max(a.y, b.y);
  return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY;
}

bool PointInRingOrOnEdge(const std::vector<IPoint>& ring, const IPoint& p)
{
  if (ring.size() < 4) return false;
  if (ring.front() != ring.back()) return false;

  // On edge => inside.
  for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
    if (PointOnSegment(p, ring[i], ring[i + 1])) return true;
  }

  // Ray casting to +X.
  bool inside = false;
  for (std::size_t i = 0, j = ring.size() - 2; i < ring.size() - 1; j = i++) {
    const IPoint& a = ring[j];
    const IPoint& b = ring[i];

    const bool cond = ((a.y > p.y) != (b.y > p.y));
    if (!cond) continue;

    const double xInt = static_cast<double>(b.x - a.x) * static_cast<double>(p.y - a.y) /
                            static_cast<double>(b.y - a.y) +
                        static_cast<double>(a.x);
    if (static_cast<double>(p.x) < xInt) inside = !inside;
  }
  return inside;
}

Ring MakeRing(std::vector<IPoint> pts)
{
  Ring r;
  r.pts = std::move(pts);
  r.signedArea = SignedAreaClosed(r.pts);
  r.minX = std::numeric_limits<int>::max();
  r.minY = std::numeric_limits<int>::max();
  r.maxX = std::numeric_limits<int>::min();
  r.maxY = std::numeric_limits<int>::min();
  for (const IPoint& p : r.pts) {
    r.minX = std::min(r.minX, p.x);
    r.minY = std::min(r.minY, p.y);
    r.maxX = std::max(r.maxX, p.x);
    r.maxY = std::max(r.maxY, p.y);
  }
  return r;
}

} // namespace

bool VectorizeLabelGridToPolygons(const std::vector<int>& labels,
                                  int w,
                                  int h,
                                  int backgroundLabel,
                                  std::vector<LabeledGeometry>& out,
                                  VectorizeStats* outStats,
                                  std::string* outError)
{
  if (outError) outError->clear();
  if (outStats) *outStats = {};
  out.clear();

  if (w <= 0 || h <= 0) {
    if (outError) *outError = "invalid grid dimensions";
    return false;
  }
  if (labels.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h)) {
    if (outError) *outError = "label grid size mismatch";
    return false;
  }

  auto at = [&](int x, int y) -> int {
    return labels[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
  };

  // Build boundary edges per label. Edges are oriented so the label region is on the *left*
  // side of the directed edge (in screen coords where +y is down).
  std::unordered_map<int, std::vector<Edge>> edgesByLabel;
  edgesByLabel.reserve(256);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int L = at(x, y);
      if (L == backgroundLabel) continue;

      // Left boundary (neighbor x-1).
      if (x == 0 || at(x - 1, y) != L) {
        edgesByLabel[L].push_back(Edge{x, y, x, y + 1});
      }
      // Right boundary (neighbor x+1).
      if (x == w - 1 || at(x + 1, y) != L) {
        edgesByLabel[L].push_back(Edge{x + 1, y + 1, x + 1, y});
      }
      // Top boundary (neighbor y-1).
      if (y == 0 || at(x, y - 1) != L) {
        edgesByLabel[L].push_back(Edge{x + 1, y, x, y});
      }
      // Bottom boundary (neighbor y+1).
      if (y == h - 1 || at(x, y + 1) != L) {
        edgesByLabel[L].push_back(Edge{x, y + 1, x + 1, y + 1});
      }
    }
  }

  // Deterministic label order.
  std::vector<int> labelKeys;
  labelKeys.reserve(edgesByLabel.size());
  for (const auto& kv : edgesByLabel) labelKeys.push_back(kv.first);
  std::sort(labelKeys.begin(), labelKeys.end());

  out.reserve(labelKeys.size());

  for (int label : labelKeys) {
    const auto itEdges = edgesByLabel.find(label);
    if (itEdges == edgesByLabel.end()) continue;
    const std::vector<Edge>& edges = itEdges->second;
    if (edges.empty()) continue;

    // start vertex -> list of edges starting there
    std::unordered_map<std::uint64_t, std::vector<int>> startToEdges;
    startToEdges.reserve(edges.size() * 2);
    for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
      const Edge& e = edges[static_cast<std::size_t>(i)];
      startToEdges[Key(e.x0, e.y0)].push_back(i);
    }

    std::vector<std::uint8_t> used(edges.size(), 0);
    std::vector<Ring> rings;

    for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
      if (used[static_cast<std::size_t>(i)]) continue;

      const Edge& e0 = edges[static_cast<std::size_t>(i)];
      used[static_cast<std::size_t>(i)] = 1;

      const IPoint start{e0.x0, e0.y0};
      IPoint cur{e0.x1, e0.y1};
      int prevDx = e0.x1 - e0.x0;
      int prevDy = e0.y1 - e0.y0;

      std::vector<IPoint> pts;
      pts.reserve(256);
      pts.push_back(start);

      // Guard against malformed graphs.
      const int maxSteps = static_cast<int>(edges.size()) + 8;
      int steps = 0;

      while (cur != start) {
        pts.push_back(cur);

        const auto it = startToEdges.find(Key(cur.x, cur.y));
        if (it == startToEdges.end()) {
          if (outError) {
            std::ostringstream oss;
            oss << "broken contour graph for label " << label << " (no outgoing edge at " << cur.x << "," << cur.y
                << ")";
            *outError = oss.str();
          }
          return false;
        }

        const int next = PickNextEdge(it->second, edges, used, prevDx, prevDy);
        if (next < 0) {
          if (outError) {
            std::ostringstream oss;
            oss << "broken contour graph for label " << label << " (stuck at " << cur.x << "," << cur.y << ")";
            *outError = oss.str();
          }
          return false;
        }

        used[static_cast<std::size_t>(next)] = 1;
        const Edge& en = edges[static_cast<std::size_t>(next)];
        prevDx = en.x1 - en.x0;
        prevDy = en.y1 - en.y0;
        cur = IPoint{en.x1, en.y1};

        if (++steps > maxSteps) {
          if (outError) {
            std::ostringstream oss;
            oss << "broken contour graph for label " << label << " (loop exceeded guard)";
            *outError = oss.str();
          }
          return false;
        }
      }

      // Close.
      pts.push_back(start);
      SimplifyRing(pts);

      rings.push_back(MakeRing(std::move(pts)));
    }

    if (rings.empty()) continue;

    // Classify rings into outers vs holes by orientation.
    std::size_t biggest = 0;
    double biggestAbs = 0.0;
    for (std::size_t i = 0; i < rings.size(); ++i) {
      const double a = std::abs(rings[i].signedArea);
      if (a > biggestAbs) {
        biggestAbs = a;
        biggest = i;
      }
    }
    const int outerSign = Sign(rings[biggest].signedArea);

    struct TmpPoly {
      std::vector<IPoint> outer;
      double outerAbsArea = 0.0;
      std::vector<std::vector<IPoint>> holes;
    };
    std::vector<TmpPoly> polys;

    std::vector<const Ring*> holes;
    for (const Ring& r : rings) {
      if (Sign(r.signedArea) == outerSign) {
        TmpPoly p;
        p.outer = r.pts;
        p.outerAbsArea = std::abs(r.signedArea);
        polys.push_back(std::move(p));
      } else {
        holes.push_back(&r);
      }
    }

    // Assign holes to the smallest containing outer.
    for (const Ring* hr : holes) {
      if (!hr || hr->pts.empty()) continue;
      const IPoint test = hr->pts.front();

      int bestPoly = -1;
      double bestArea = std::numeric_limits<double>::infinity();
      for (int pi = 0; pi < static_cast<int>(polys.size()); ++pi) {
        const TmpPoly& p = polys[static_cast<std::size_t>(pi)];
        if (!PointInRingOrOnEdge(p.outer, test)) continue;
        if (p.outerAbsArea < bestArea) {
          bestArea = p.outerAbsArea;
          bestPoly = pi;
        }
      }

      if (bestPoly >= 0) {
        polys[static_cast<std::size_t>(bestPoly)].holes.push_back(hr->pts);
      } else {
        // Degenerate case: treat as an outer.
        TmpPoly p;
        p.outer = hr->pts;
        p.outerAbsArea = std::abs(hr->signedArea);
        polys.push_back(std::move(p));
      }
    }

    LabeledGeometry lg;
    lg.label = label;
    for (const TmpPoly& tp : polys) {
      VectorPolygon vp;
      vp.outer = tp.outer;
      vp.holes = tp.holes;
      lg.geom.polygons.push_back(std::move(vp));
    }

    out.push_back(std::move(lg));

    if (outStats) {
      outStats->rings += static_cast<int>(rings.size());
      outStats->polygons += static_cast<int>(polys.size());
      for (const TmpPoly& tp : polys) outStats->holes += static_cast<int>(tp.holes.size());
    }
  }

  if (outStats) {
    outStats->labels = static_cast<int>(out.size());
  }

  return true;
}

} // namespace isocity
