#pragma once

#include "isocity/Types.hpp"

#include <algorithm>
#include <cstdlib>
#include <vector>

namespace isocity {

// Tiny, deterministic raster helpers for applying tools over simple shapes.
//
// These utilities are intentionally kept in the raylib-free core so they can be
// reused by:
//   - headless tools (batch scripts / regression scenarios)
//   - tests
//   - future in-game "drag" tooling
//
// The emphasis here is *determinism* and *simplicity*, not micro-optimizations.

inline void NormalizeRect(Point& a, Point& b)
{
  if (a.x > b.x) std::swap(a.x, b.x);
  if (a.y > b.y) std::swap(a.y, b.y);
}

// Iterate all points on an inclusive integer line segment.
// Calls fn(Point) for each point in order from a->b.
//
// Guarantees:
//   - Includes both endpoints.
//   - Deterministic for the same (a,b).
//   - Produces a 4-connected raster suitable for grid tools (no diagonal-only adjacency).
template <typename Fn>
inline void ForEachLinePoint(Point a, Point b, Fn&& fn)
{
  int x0 = a.x;
  int y0 = a.y;
  const int x1 = b.x;
  const int y1 = b.y;

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;

  int err = dx - dy;

  // Standard Bresenham may step in both X and Y in one iteration, producing
  // diagonal adjacency. For tools like roads (4-neighborhood connectivity), we
  // split diagonal steps into two axis-aligned steps by emitting an intermediate
  // point. The ordering is deterministic: we step along the dominant axis first.
  const bool dominantX = (dx >= dy);

  for (;;) {
    fn(Point{x0, y0});
    if (x0 == x1 && y0 == y1) break;

    const int e2 = err * 2;
    const bool stepX = (e2 > -dy);
    const bool stepY = (e2 < dx);

    if (stepX && stepY) {
      if (dominantX) {
        err -= dy;
        x0 += sx;
        fn(Point{x0, y0});

        err += dx;
        y0 += sy;
      } else {
        err += dx;
        y0 += sy;
        fn(Point{x0, y0});

        err -= dy;
        x0 += sx;
      }
      continue;
    }

    if (stepX) {
      err -= dy;
      x0 += sx;
    }
    if (stepY) {
      err += dx;
      y0 += sy;
    }
  }
}

inline std::vector<Point> RasterLine(Point a, Point b)
{
  std::vector<Point> out;
  // Reservation for the worst case (a fully diagonal line) where we emit
  // dx+dy+1 points for 4-connectedness.
  out.reserve(static_cast<std::size_t>(std::abs(b.x - a.x) + std::abs(b.y - a.y) + 1));
  ForEachLinePoint(a, b, [&](Point p) { out.push_back(p); });
  return out;
}

// Iterate all points in a filled axis-aligned inclusive rectangle.
template <typename Fn>
inline void ForEachRectFilled(Point a, Point b, Fn&& fn)
{
  NormalizeRect(a, b);
  for (int y = a.y; y <= b.y; ++y) {
    for (int x = a.x; x <= b.x; ++x) {
      fn(Point{x, y});
    }
  }
}

inline std::vector<Point> RasterRectFilled(Point a, Point b)
{
  std::vector<Point> out;
  NormalizeRect(a, b);
  const int w = std::max(0, b.x - a.x + 1);
  const int h = std::max(0, b.y - a.y + 1);
  out.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  ForEachRectFilled(a, b, [&](Point p) { out.push_back(p); });
  return out;
}

// Iterate all points on the outline of an axis-aligned inclusive rectangle.
// The outline is 4-connected and does NOT duplicate corner points.
template <typename Fn>
inline void ForEachRectOutline(Point a, Point b, Fn&& fn)
{
  NormalizeRect(a, b);

  // Degenerate cases collapse to a line.
  if (a.x == b.x || a.y == b.y) {
    ForEachLinePoint(a, b, fn);
    return;
  }

  // Top edge (inclusive corners).
  for (int x = a.x; x <= b.x; ++x) fn(Point{x, a.y});

  // Right edge (excluding top corner).
  for (int y = a.y + 1; y <= b.y; ++y) fn(Point{b.x, y});

  // Bottom edge (excluding right corner).
  for (int x = b.x - 1; x >= a.x; --x) fn(Point{x, b.y});

  // Left edge (excluding bottom + top corners).
  for (int y = b.y - 1; y >= a.y + 1; --y) fn(Point{a.x, y});
}

inline std::vector<Point> RasterRectOutline(Point a, Point b)
{
  std::vector<Point> out;
  ForEachRectOutline(a, b, [&](Point p) { out.push_back(p); });
  return out;
}

} // namespace isocity
