#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "isocity/Elevation.hpp"
#include "isocity/Types.hpp"

#include "isocity/RaylibShim.hpp"

namespace isocity {

inline Vector2 TileToWorldCenter(int tx, int ty, float tileW, float tileH)
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;
  return Vector2{(tx - ty) * halfW, (tx + ty) * halfH};
}

inline Point WorldToTileApprox(Vector2 world, float tileW, float tileH)
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;

  // Inverse of:
  //   world.x = (x - y) * halfW
  //   world.y = (x + y) * halfH
  const float fx = (world.y / halfH + world.x / halfW) * 0.5f;
  const float fy = (world.y / halfH - world.x / halfW) * 0.5f;

  return Point{static_cast<int>(std::floor(fx)), static_cast<int>(std::floor(fy))};
}

inline bool PointInTileDiamond(Vector2 worldPoint, int tx, int ty, float tileW, float tileH)
{
  const Vector2 c = TileToWorldCenter(tx, ty, tileW, tileH);
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;

  const float dx = std::fabs(worldPoint.x - c.x) / halfW;
  const float dy = std::fabs(worldPoint.y - c.y) / halfH;
  return (dx + dy) <= 1.0f;
}

// More accurate than WorldToTileApprox: checks candidates around the approximated tile.
inline std::optional<Point> WorldToTile(Vector2 world, int mapW, int mapH, float tileW, float tileH)
{
  const Point approx = WorldToTileApprox(world, tileW, tileH);

  // Try nearby candidates (handles edges of diamonds better).
  for (int oy = -1; oy <= 1; ++oy) {
    for (int ox = -1; ox <= 1; ++ox) {
      const int tx = approx.x + ox;
      const int ty = approx.y + oy;
      if (tx < 0 || ty < 0 || tx >= mapW || ty >= mapH) continue;
      if (PointInTileDiamond(world, tx, ty, tileW, tileH)) return Point{tx, ty};
    }
  }

  // Fallback: accept approx if in bounds.
  if (approx.x >= 0 && approx.y >= 0 && approx.x < mapW && approx.y < mapH) return approx;
  return std::nullopt;
}

inline void TileDiamondCorners(Vector2 center, float tileW, float tileH, Vector2 out[4])
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;
  out[0] = Vector2{center.x, center.y - halfH}; // top
  out[1] = Vector2{center.x + halfW, center.y}; // right
  out[2] = Vector2{center.x, center.y + halfH}; // bottom
  out[3] = Vector2{center.x - halfW, center.y}; // left
}

// -----------------------------------------------------------------------------------------------
// Elevation helpers
// -----------------------------------------------------------------------------------------------

inline bool PointInDiamond(Vector2 worldPoint, Vector2 center, float tileW, float tileH)
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;
  const float dx = std::fabs(worldPoint.x - center.x) / halfW;
  const float dy = std::fabs(worldPoint.y - center.y) / halfH;
  return (dx + dy) <= 1.0f;
}

inline Vector2 TileToWorldCenterElevated(const World& world, int tx, int ty, float tileW, float tileH,
                                        const ElevationSettings& elev)
{
  Vector2 c = TileToWorldCenter(tx, ty, tileW, tileH);
  if (world.inBounds(tx, ty)) {
    c.y -= TileElevationPx(world.at(tx, ty), elev);
  }
  return c;
}

// Elevation-aware tile picking.
//
// When elevation is enabled, tiles are rendered with a vertical offset (in world/pixel units).
// This function resolves the correct tile under the cursor by testing elevated diamond bounds
// around an approximate inverse transform.
inline std::optional<Point> WorldToTileElevated(Vector2 worldPos, const World& world, float tileW, float tileH,
                                                const ElevationSettings& elev)
{
  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) return std::nullopt;

  const Point approx = WorldToTileApprox(worldPos, tileW, tileH);

  std::optional<Point> best;
  int bestSum = std::numeric_limits<int>::min();
  int bestX = std::numeric_limits<int>::min();

  // Elevation can shift the diamond by up to ~tileH, so search a slightly larger neighborhood.
  constexpr int kSearch = 3;
  for (int oy = -kSearch; oy <= kSearch; ++oy) {
    for (int ox = -kSearch; ox <= kSearch; ++ox) {
      const int tx = approx.x + ox;
      const int ty = approx.y + oy;
      if (tx < 0 || ty < 0 || tx >= mapW || ty >= mapH) continue;

      const Vector2 c = TileToWorldCenterElevated(world, tx, ty, tileW, tileH, elev);
      if (!PointInDiamond(worldPos, c, tileW, tileH)) continue;

      // If multiple elevated diamonds overlap at this pixel, choose the one that would be
      // drawn last (front-most) under our diagonal draw order.
      const int sum = tx + ty;
      if (!best || sum > bestSum || (sum == bestSum && tx > bestX)) {
        best = Point{tx, ty};
        bestSum = sum;
        bestX = tx;
      }
    }
  }

  if (best) return best;
  return WorldToTile(worldPos, mapW, mapH, tileW, tileH);
}

} // namespace isocity
