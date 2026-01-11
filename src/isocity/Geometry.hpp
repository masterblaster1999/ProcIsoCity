#pragma once

#include "isocity/Types.hpp"
#include "isocity/Vectorize.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Geometry helpers (dependency-free)
//
// The headless toolchain produces a lot of orthogonal, grid-aligned geometry (roads,
// vectorized zone polygons, district outlines...). When exported naively, these shapes often
// contain long runs of collinear points.
//
// These helpers perform a *topology-preserving* simplification by removing points that lie on
// straight segments (collinear + between neighbors). This keeps the geometry identical, while
// producing dramatically smaller JSON/GeoJSON outputs.
//
// The simplification is deterministic and has no tolerance parameter.
// -----------------------------------------------------------------------------------------------

namespace detail {

inline bool IsBetween(int a, int b, int c)
{
  // true if b is within the closed interval [min(a,c), max(a,c)]
  return (b >= std::min(a, c) && b <= std::max(a, c));
}

inline bool IsCollinearInt(int ax, int ay, int bx, int by, int cx, int cy)
{
  // Cross product (b-a) x (c-a) == 0.
  const long long abx = static_cast<long long>(bx) - static_cast<long long>(ax);
  const long long aby = static_cast<long long>(by) - static_cast<long long>(ay);
  const long long acx = static_cast<long long>(cx) - static_cast<long long>(ax);
  const long long acy = static_cast<long long>(cy) - static_cast<long long>(ay);
  return (abx * acy - aby * acx) == 0;
}

inline bool IsRedundantPoint(int ax, int ay, int bx, int by, int cx, int cy)
{
  if (!IsCollinearInt(ax, ay, bx, by, cx, cy)) return false;
  return IsBetween(ax, bx, cx) && IsBetween(ay, by, cy);
}

} // namespace detail

// Remove consecutive duplicate points from a polyline.
inline void DedupPolyline(std::vector<Point>& pts)
{
  if (pts.empty()) return;
  std::vector<Point> out;
  out.reserve(pts.size());
  for (const Point& p : pts) {
    if (!out.empty() && out.back().x == p.x && out.back().y == p.y) continue;
    out.push_back(p);
  }
  pts.swap(out);
}

// Remove consecutive duplicate points from an open ring (does not consider wrap-around).
inline void DedupOpenRing(std::vector<IPoint>& pts)
{
  if (pts.empty()) return;
  std::vector<IPoint> out;
  out.reserve(pts.size());
  for (const IPoint& p : pts) {
    if (!out.empty() && out.back() == p) continue;
    out.push_back(p);
  }
  pts.swap(out);
}

// Simplify a polyline by removing redundant collinear points.
inline void SimplifyPolylineCollinear(std::vector<Point>& pts)
{
  DedupPolyline(pts);
  if (pts.size() < 3) return;

  const std::vector<Point> in = pts;
  std::vector<Point> out;
  out.reserve(in.size());
  out.push_back(in.front());

  for (std::size_t i = 1; i + 1 < in.size(); ++i) {
    const Point& a = out.back();
    const Point& b = in[i];
    const Point& c = in[i + 1];

    if (detail::IsRedundantPoint(a.x, a.y, b.x, b.y, c.x, c.y)) {
      continue;
    }
    out.push_back(b);
  }

  out.push_back(in.back());
  pts.swap(out);
  DedupPolyline(pts);
}

// Simplify a *closed* linear ring in-place.
//
// Requirements:
//  - If ring is valid, ring.front() == ring.back().
//  - After simplification, we keep it closed.
inline void SimplifyRingCollinear(std::vector<IPoint>& ring)
{
  if (ring.size() < 4) return;
  if (ring.front() != ring.back()) return; // not a closed ring

  // Work on an open ring (drop duplicated closing point).
  std::vector<IPoint> v = ring;
  v.pop_back();
  if (v.size() < 3) return;

  DedupOpenRing(v);
  if (v.size() < 3) return;

  const std::vector<IPoint> original = v;

  // Iteratively remove redundant points in a circular manner.
  for (int iter = 0; iter < 16; ++iter) {
    if (v.size() <= 3) break;
    bool changed = false;

    std::vector<IPoint> out;
    out.reserve(v.size());

    const std::size_t n = v.size();
    for (std::size_t i = 0; i < n; ++i) {
      const IPoint& a = v[(i + n - 1) % n];
      const IPoint& b = v[i];
      const IPoint& c = v[(i + 1) % n];

      if (n > 3 && detail::IsRedundantPoint(a.x, a.y, b.x, b.y, c.x, c.y)) {
        changed = true;
        continue;
      }
      out.push_back(b);
    }

    if (!changed) break;
    v.swap(out);
  }

  // Ensure we didn't degenerate.
  if (v.size() < 3) v = original;

  ring = v;
  ring.push_back(ring.front());
}

inline void SimplifyVectorPolygonCollinear(VectorPolygon& poly)
{
  if (!poly.outer.empty()) SimplifyRingCollinear(poly.outer);
  for (auto& h : poly.holes) {
    if (!h.empty()) SimplifyRingCollinear(h);
  }
}

inline void SimplifyVectorMultiPolygonCollinear(VectorMultiPolygon& mp)
{
  for (auto& p : mp.polygons) {
    SimplifyVectorPolygonCollinear(p);
  }
}

} // namespace isocity
