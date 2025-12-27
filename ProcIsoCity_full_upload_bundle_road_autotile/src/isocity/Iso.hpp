#pragma once

#include <cmath>
#include <optional>

#include "raylib.h"

namespace isocity {

struct Point {
  int x = 0;
  int y = 0;
};

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

} // namespace isocity
