#include "isocity/Renderer.hpp"

#include "isocity/Pathfinding.hpp"

#include "isocity/Random.hpp"
#include "isocity/ZoneMetrics.hpp"
#include "isocity/Road.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <string>

namespace isocity {

// Forward declarations (Renderer.cpp is a single TU with many helpers; keep things
// declared before first use so MSVC doesn't fail with "identifier not found" errors).
static Color TerrainCliffBaseColor(Terrain t);

namespace {

inline unsigned char ClampU8(int v) { return static_cast<unsigned char>(std::clamp(v, 0, 255)); }

inline Color Mul(Color c, float b)
{
  const int r = static_cast<int>(std::round(static_cast<float>(c.r) * b));
  const int g = static_cast<int>(std::round(static_cast<float>(c.g) * b));
  const int bl = static_cast<int>(std::round(static_cast<float>(c.b) * b));
  return Color{ClampU8(r), ClampU8(g), ClampU8(bl), c.a};
}

inline Color Lerp(Color a, Color b, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  const int r = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.r) + t * static_cast<float>(b.r)));
  const int g = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.g) + t * static_cast<float>(b.g)));
  const int bl = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.b) + t * static_cast<float>(b.b)));
  const int al = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.a) + t * static_cast<float>(b.a)));
  return Color{ClampU8(r), ClampU8(g), ClampU8(bl), ClampU8(al)};
}

inline Color DistrictBaseColor(std::uint8_t d)
{
  switch (d & 7u) {
  case 1: return Color{50, 140, 255, 255};   // blue
  case 2: return Color{255, 170, 60, 255};   // orange
  case 3: return Color{80, 200, 120, 255};   // green
  case 4: return Color{190, 90, 255, 255};   // purple
  case 5: return Color{255, 80, 80, 255};    // red
  case 6: return Color{60, 220, 220, 255};   // cyan
  case 7: return Color{255, 230, 70, 255};   // yellow
  default: return Color{0, 0, 0, 0};         // district 0 (unassigned): transparent
  }
}

inline Color DistrictFillColor(std::uint8_t d, unsigned char alpha)
{
  Color c = DistrictBaseColor(d);
  c.a = alpha;
  return c;
}

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

inline bool IsImageReadyCompat(const Image& img)
{
  // raylib has gained helper "Is*Ready" functions over time, but some versions used
  // by FetchContent don't include IsImageReady. This local check keeps builds working
  // across raylib versions.
  return (img.data != nullptr) && (img.width > 0) && (img.height > 0);
}

struct DiamondParams {
  float nx = 0;
  float ny = 0;
  float manhattan = 0;
  float edge = 0; // 0 at edge, 1 at center
};

DiamondParams DiamondAtPixel(int x, int y, int w, int h)
{
  const float cx = (static_cast<float>(w) - 1.0f) * 0.5f;
  const float cy = (static_cast<float>(h) - 1.0f) * 0.5f;
  const float halfW = static_cast<float>(w) * 0.5f;
  const float halfH = static_cast<float>(h) * 0.5f;

  const float nx = (static_cast<float>(x) - cx) / halfW;
  const float ny = (static_cast<float>(y) - cy) / halfH;
  const float man = std::fabs(nx) + std::fabs(ny);
  const float edge = std::clamp(1.0f - man, 0.0f, 1.0f);

  return DiamondParams{nx, ny, man, edge};
}

// Generic diamond texture generator (RGBA) with a per-pixel callback.
template <typename Fn>
Texture2D MakeDiamondTexture(int w, int h, Fn fn)
{
  Image img = GenImageColor(w, h, BLANK);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const DiamondParams d = DiamondAtPixel(x, y, w, h);
      if (d.manhattan > 1.0f) continue;

      const Color c = fn(x, y, d);
      if (c.a == 0) continue;
      ImageDrawPixel(&img, x, y, c);
    }
  }

  Texture2D tex = LoadTextureFromImage(img);
  UnloadImage(img);
  return tex;
}

inline float Dot2(float ax, float ay, float bx, float by) { return ax * bx + ay * by; }
struct TileRect {
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
};

// Compute a conservative tile-coordinate rectangle that covers the current camera viewport.
// This is used to avoid drawing off-screen tiles (big win when panning/zooming on larger maps).
TileRect ComputeVisibleTileRect(const Camera2D& camera, int screenW, int screenH, int mapW, int mapH, float tileW,
                                float tileH, float maxElevPx)
{
  // Viewport corners in world space.
  const Vector2 s0{0.0f, 0.0f};
  const Vector2 s1{static_cast<float>(screenW), 0.0f};
  const Vector2 s2{0.0f, static_cast<float>(screenH)};
  const Vector2 s3{static_cast<float>(screenW), static_cast<float>(screenH)};

  const Vector2 w0 = GetScreenToWorld2D(s0, camera);
  const Vector2 w1 = GetScreenToWorld2D(s1, camera);
  const Vector2 w2 = GetScreenToWorld2D(s2, camera);
  const Vector2 w3 = GetScreenToWorld2D(s3, camera);

  float minWX = std::numeric_limits<float>::infinity();
  float minWY = std::numeric_limits<float>::infinity();
  float maxWX = -std::numeric_limits<float>::infinity();
  float maxWY = -std::numeric_limits<float>::infinity();

  const Vector2 ws[4] = {w0, w1, w2, w3};
  for (const Vector2& wv : ws) {
    minWX = std::min(minWX, wv.x);
    minWY = std::min(minWY, wv.y);
    maxWX = std::max(maxWX, wv.x);
    maxWY = std::max(maxWY, wv.y);
  }

  // Expand by one tile to avoid edge pop-in (dst rect extends beyond the tile center).
  minWX -= tileW;
  maxWX += tileW;
  // Elevation shifts tiles upward in world-space; expand Y bounds by the maximum elevation so we don't
  // cull tiles that still contribute visible pixels above the viewport.
  minWY -= tileH + maxElevPx;
  maxWY += tileH + maxElevPx;

  const Point a = WorldToTileApprox(Vector2{minWX, minWY}, tileW, tileH);
  const Point b = WorldToTileApprox(Vector2{maxWX, minWY}, tileW, tileH);
  const Point c = WorldToTileApprox(Vector2{minWX, maxWY}, tileW, tileH);
  const Point d = WorldToTileApprox(Vector2{maxWX, maxWY}, tileW, tileH);

  const int minTX = std::min({a.x, b.x, c.x, d.x});
  const int maxTX = std::max({a.x, b.x, c.x, d.x});
  const int minTY = std::min({a.y, b.y, c.y, d.y});
  const int maxTY = std::max({a.y, b.y, c.y, d.y});

  // Extra safety margin in tile space (camera rotations / numerical edge cases).
  const int margin = 3;

  TileRect r;
  r.minX = std::clamp(minTX - margin, 0, mapW - 1);
  r.maxX = std::clamp(maxTX + margin, 0, mapW - 1);
  r.minY = std::clamp(minTY - margin, 0, mapH - 1);
  r.maxY = std::clamp(maxTY + margin, 0, mapH - 1);
  return r;
}

struct BandBounds {
  float minX = 0.0f;
  float minY = 0.0f;
  float maxX = 0.0f;
  float maxY = 0.0f;
};

// Compute a conservative world-space AABB for an isometric diagonal band.
//
// A band is defined by a contiguous range of (x+y) sums [sum0..sum1].
// We later render this entire band into a single RenderTexture2D and draw it
// in increasing band order, which preserves the global draw ordering.
BandBounds ComputeBandBounds(int sum0, int sum1, int mapW, int mapH, float tileW, float tileH, float maxElevPx)
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;

  float minCenterX = std::numeric_limits<float>::infinity();
  float maxCenterX = -std::numeric_limits<float>::infinity();

  for (int sum = sum0; sum <= sum1; ++sum) {
    const int x0 = std::max(0, sum - (mapH - 1));
    const int x1 = std::min(mapW - 1, sum);
    if (x0 > x1) continue;

    // For fixed sum, center.x = (x-y)*halfW = (2*x - sum)*halfW.
    const float cx0 = (2.0f * static_cast<float>(x0) - static_cast<float>(sum)) * halfW;
    const float cx1 = (2.0f * static_cast<float>(x1) - static_cast<float>(sum)) * halfW;

    minCenterX = std::min(minCenterX, cx0);
    maxCenterX = std::max(maxCenterX, cx1);
  }

  if (!std::isfinite(minCenterX) || !std::isfinite(maxCenterX)) {
    minCenterX = 0.0f;
    maxCenterX = 0.0f;
  }

  BandBounds b;
  b.minX = minCenterX - halfW;
  b.maxX = maxCenterX + halfW;

  // For fixed sum, base center.y = (x+y)*halfH = sum*halfH.
  // Elevation shifts tiles UP (subtract), so we subtract maxElevPx from minY.
  b.minY = static_cast<float>(sum0) * halfH - halfH - maxElevPx;
  b.maxY = static_cast<float>(sum1) * halfH + halfH;
  return b;
}

// Compute a stable screen-space destination rectangle for the minimap.
Renderer::MinimapLayout ComputeMinimapLayout(int mapW, int mapH, int screenW, int screenH)
{
  Renderer::MinimapLayout out;
  if (mapW <= 0 || mapH <= 0 || screenW <= 0 || screenH <= 0) {
    out.rect = Rectangle{0, 0, 0, 0};
    out.pixelsPerTile = 1.0f;
    return out;
  }

  const float pad = 12.0f;
  // Cap minimap size relative to the window so it stays usable across resolutions.
  const float maxSize = std::min(260.0f, std::min(static_cast<float>(screenW), static_cast<float>(screenH)) * 0.38f);
  const float denom = static_cast<float>(std::max(mapW, mapH));
  float s = (denom > 0.0f) ? (maxSize / denom) : 1.0f;
  s = std::clamp(s, 0.35f, 6.0f);

  const float w = static_cast<float>(mapW) * s;
  const float h = static_cast<float>(mapH) * s;

  out.rect = Rectangle{static_cast<float>(screenW) - pad - w, static_cast<float>(screenH) - pad - h, w, h};
  out.pixelsPerTile = s;
  return out;
}

// Determine a minimap pixel color for a tile.
Color MinimapColorForTile(const isocity::Tile& t)
{
  using namespace isocity;

  // Base terrain colors.
  Color base = Color{70, 160, 90, 255};
  if (t.terrain == Terrain::Water) base = Color{35, 90, 210, 255};
  if (t.terrain == Terrain::Sand) base = Color{195, 170, 95, 255};

  // Simple height shading: higher tiles are slightly brighter.
  const float b = std::clamp(0.70f + t.height * 0.45f, 0.35f, 1.25f);
  base = Mul(base, b);

  // Overlays: mix towards a strong color so gameplay is readable.
  switch (t.overlay) {
  case Overlay::None: return base;
  case Overlay::Road: {
    // Higher-tier roads read darker / stronger on the minimap.
    const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);

    // Roads on water are bridges; render them a bit lighter/warmer so they are readable over water.
    if (t.terrain == Terrain::Water) {
      const Color bridge = (lvl == 1) ? Color{190, 170, 125, 255}
                          : (lvl == 2) ? Color{180, 160, 118, 255}
                                       : Color{170, 152, 110, 255};
      const float k = (lvl == 1) ? 0.82f : (lvl == 2) ? 0.84f : 0.86f;
      return Lerp(base, bridge, k);
    }

    const Color road = (lvl == 1) ? Color{28, 28, 30, 255} : (lvl == 2) ? Color{24, 24, 28, 255} : Color{20, 20, 25, 255};
    const float k = (lvl == 1) ? 0.85f : (lvl == 2) ? 0.88f : 0.90f;
    return Lerp(base, road, k);
  }
  case Overlay::Park: return Lerp(base, Color{70, 200, 95, 255}, 0.70f);
  case Overlay::Residential: return Lerp(base, Color{80, 160, 235, 255}, 0.80f);
  case Overlay::Commercial: return Lerp(base, Color{240, 170, 60, 255}, 0.80f);
  case Overlay::Industrial: return Lerp(base, Color{200, 90, 220, 255}, 0.80f);
  default: break;
  }
  return base;
}

// Draw a simple extruded "building" for zone tiles.
void DrawZoneBuilding(const isocity::Tile& t, float tileW, float tileH, float zoom, const Vector2& tileCenter,
                      float tileBrightness)
{
  using namespace isocity;

  const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                       t.overlay == Overlay::Industrial);
  if (!isZone) return;

  // Fade out when zoomed out.
  if (tileW * zoom < 26.0f) return;

  const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);

  int cap = 0;
  float baseShrink = 0.54f;
  float heightMul = 1.0f;
  Color baseColor = Color{210, 210, 210, 255};

  if (t.overlay == Overlay::Residential) {
    cap = CapacityForOverlayLevel(t.overlay, lvl);
    baseShrink = 0.58f;
    heightMul = 1.00f;
    baseColor = Color{200, 220, 255, 255};
  } else if (t.overlay == Overlay::Commercial) {
    cap = CapacityForOverlayLevel(t.overlay, lvl);
    baseShrink = 0.50f;
    heightMul = 1.40f;
    baseColor = Color{255, 220, 170, 255};
  } else if (t.overlay == Overlay::Industrial) {
    cap = CapacityForOverlayLevel(t.overlay, lvl);
    baseShrink = 0.62f;
    heightMul = 0.95f;
    baseColor = Color{230, 210, 255, 255};
  }

  const float occRatio = (cap > 0) ? std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f)
                                   : 0.0f;
  const float var = static_cast<float>((t.variation >> 4) & 0x0Fu) / 15.0f;

  // Height is primarily driven by level + occupancy, with some stable per-tile variety.
  float heightPx = tileH * (0.55f + 0.35f * static_cast<float>(lvl) + 0.35f * occRatio);
  heightPx *= heightMul;
  heightPx *= 0.85f + 0.35f * var;

  // Clamp so we don't create skyscrapers that overlap too aggressively.
  heightPx = std::clamp(heightPx, tileH * 0.60f, tileH * 4.25f);

  Vector2 diamond[4];
  TileDiamondCorners(tileCenter, tileW, tileH, diamond);

  Vector2 base[4];
  for (int i = 0; i < 4; ++i) {
    base[i].x = tileCenter.x + (diamond[i].x - tileCenter.x) * baseShrink;
    base[i].y = tileCenter.y + (diamond[i].y - tileCenter.y) * baseShrink;
  }

  Vector2 top[4];
  for (int i = 0; i < 4; ++i) {
    top[i] = base[i];
    top[i].y -= heightPx;
  }

  // Per-face shading.
  const float b = std::clamp(tileBrightness, 0.35f, 1.35f);
  const Color topC = Mul(baseColor, 1.10f * b);
  const Color rightC = Mul(baseColor, 0.85f * b);
  const Color leftC = Mul(baseColor, 0.70f * b);

  // Right face: edge 1-2.
  DrawTriangle(base[1], base[2], top[2], rightC);
  DrawTriangle(base[1], top[2], top[1], rightC);

  // Left face: edge 2-3.
  DrawTriangle(base[3], base[2], top[2], leftC);
  DrawTriangle(base[3], top[2], top[3], leftC);

  // Roof (top face) drawn last.
  DrawTriangle(top[0], top[1], top[2], topC);
  DrawTriangle(top[0], top[2], top[3], topC);

  // Optional small roof highlight at high zoom.
  if (tileW * zoom >= 46.0f) {
    const float invZoom = 1.0f / std::max(0.001f, zoom);
    const float thick = 1.0f * invZoom;
    const Color lc = Color{0, 0, 0, 70};
    DrawLineEx(top[0], top[1], thick, lc);
    DrawLineEx(top[1], top[2], thick, lc);
    DrawLineEx(top[2], top[3], thick, lc);
    DrawLineEx(top[3], top[0], thick, lc);
  }
}


// Returns distance from P(px,py) to segment AB. Also returns the projected t in [0,1].
float DistPointSegment(float px, float py, float ax, float ay, float bx, float by, float& outT)
{
  const float vx = bx - ax;
  const float vy = by - ay;
  const float wx = px - ax;
  const float wy = py - ay;

  const float len2 = vx * vx + vy * vy;
  float t = 0.0f;
  if (len2 > 1.0e-6f) t = Dot2(wx, wy, vx, vy) / len2;
  t = std::clamp(t, 0.0f, 1.0f);
  outT = t;

  const float cx = ax + t * vx;
  const float cy = ay + t * vy;
  const float dx = px - cx;
  const float dy = py - cy;
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace


namespace {

// Compute diamond center from 4-corner representation (top, right, bottom, left).
inline Vector2 DiamondCenter(const Vector2 d[4])
{
  return Vector2{(d[0].x + d[2].x) * 0.5f, (d[0].y + d[2].y) * 0.5f};
}

inline void CopyDiamond(Vector2 dst[4], const Vector2 src[4])
{
  for (int i = 0; i < 4; ++i) dst[i] = src[i];
}

inline void ShrinkDiamond(Vector2 out[4], const Vector2 in[4], float factor)
{
  const Vector2 c = DiamondCenter(in);
  for (int i = 0; i < 4; ++i) {
    out[i] = Vector2{Lerp(c.x, in[i].x, factor), Lerp(c.y, in[i].y, factor)};
  }
}

inline Vector2 TileCornerAtMaxElevation(const Elevation& elev, int tx, int ty, float tileW,
                                       float tileH, float baseElevPx, int cornerIndex)
{
  Vector2 c = TileToWorldCenter(tx, ty, tileW, tileH);
  c.y -= baseElevPx;

  Vector2 d[4];
  TileDiamondCorners(c, tileW, tileH, d);
  return d[cornerIndex];
}

inline void DrawIsoPrism(const Vector2 base[4], float heightPx, Color baseColor, float brightness, float tierShade,
                         float zoom)
{
  Vector2 top[4];
  for (int i = 0; i < 4; ++i) {
    top[i] = base[i];
    top[i].y -= heightPx;
  }

  const float b = std::clamp(brightness * tierShade, 0.35f, 1.35f);

  const Color cTop = Mul(baseColor, 1.10f * b);
  const Color cSideR = Mul(baseColor, 0.85f * b);
  const Color cSideL = Mul(baseColor, 0.70f * b);

  // Right face (edge 1-2).
  DrawTriangle(base[1], base[2], top[2], cSideR);
  DrawTriangle(base[1], top[2], top[1], cSideR);

  // Left face (edge 2-3).
  DrawTriangle(base[2], base[3], top[3], cSideL);
  DrawTriangle(base[2], top[3], top[2], cSideL);

  // Roof.
  DrawTriangle(top[0], top[1], top[2], cTop);
  DrawTriangle(top[0], top[2], top[3], cTop);

  // Subtle outline at higher zoom.
  if (zoom >= 0.85f) {
    const float thick = 0.85f * std::clamp(zoom, 0.75f, 1.8f);
    const Color oc = Color{0, 0, 0, 55};

    DrawLineEx(top[0], top[1], thick, oc);
    DrawLineEx(top[1], top[2], thick, oc);
    DrawLineEx(top[2], top[3], thick, oc);
    DrawLineEx(top[3], top[0], thick, oc);
  }
}

} // namespace

static void DrawMergedZoneBuildingAndIndicators(const ZoneBuildingParcel& p, const World& world, const Elevation& elev,
                                               float tileW, float tileH, float zoom, float timeSec)
{
  if (!p.isMultiTile()) return;
  if (!IsZoneOverlay(p.overlay)) return;
  if (tileW * zoom < 26.0f) return;

  const int lvl = ClampZoneLevel(p.level);

  const int pw = std::max(1, p.w);
  const int ph = std::max(1, p.h);
  const int area = std::max(1, pw * ph);
  const float scale = std::sqrt(static_cast<float>(area));
  const float maxDim = static_cast<float>(std::max(pw, ph));

  const int cap = std::max(0, p.capacity);
  const int occ = std::clamp(p.occupants, 0, cap);
  const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
  const float var = static_cast<float>((p.styleSeed >> 4) & 0x0Fu) / 15.0f;

  // Base appearance defaults.
  float heightMul = 1.0f;
  Color baseColor = Color{210, 210, 210, 255};

  // k controls how much we shrink the footprint inward per parcel size (larger parcels shrink less).
  float shrinkK = 0.46f;

  if (p.overlay == Overlay::Residential) {
    baseColor = Color{75, 145, 245, 255};
    heightMul = 1.10f;
    shrinkK = 0.42f;
  } else if (p.overlay == Overlay::Commercial) {
    baseColor = Color{70, 230, 140, 255};
    heightMul = 1.40f;
    shrinkK = 0.46f;
  } else if (p.overlay == Overlay::Industrial) {
    baseColor = Color{250, 195, 55, 255};
    heightMul = 0.95f;
    shrinkK = 0.38f;
  }

  // Determine max elevation across the parcel for a stable base plane.
  float baseElevPx = 0.0f;
  float brightnessSum = 0.0f;
  int tileCount = 0;

  for (int yy = p.y0; yy < p.y0 + ph; ++yy) {
    for (int xx = p.x0; xx < p.x0 + pw; ++xx) {
      const Tile& t = world.at(xx, yy);
      baseElevPx = std::max(baseElevPx, TileElevationPx(t, elev));

      const float v = (t.variation / 255.0f - 0.5f) * 0.10f;
      brightnessSum += 0.85f + t.height * 0.30f + v;
      ++tileCount;
    }
  }

  const float brightness = (tileCount > 0) ? (brightnessSum / static_cast<float>(tileCount)) : 1.0f;

  // Footprint outer corners.
  const int x0 = p.x0;
  const int y0 = p.y0;
  const int x1 = p.x0 + pw - 1;
  const int y1 = p.y0 + ph - 1;

  Vector2 outer[4];
  outer[0] = TileCornerAtMaxElevation(elev, x0, y0, tileW, tileH, baseElevPx, 0); // top
  outer[1] = TileCornerAtMaxElevation(elev, x1, y0, tileW, tileH, baseElevPx, 1); // right
  outer[2] = TileCornerAtMaxElevation(elev, x1, y1, tileW, tileH, baseElevPx, 2); // bottom
  outer[3] = TileCornerAtMaxElevation(elev, x0, y1, tileW, tileH, baseElevPx, 3); // left

  // Inset the base for a nicer margin.
  // Larger parcels shrink slightly less so big footprints don't look overly thin.
  const float baseShrink = std::clamp(1.0f - shrinkK / std::max(1.0f, scale), 0.55f, 0.94f);

  Vector2 base[4];
  ShrinkDiamond(base, outer, baseShrink);

  // Height model (scaled by footprint).
  float heightPx = tileH * (0.55f + 0.65f * static_cast<float>(lvl));
  heightPx += tileH * (0.25f + 0.45f * static_cast<float>(lvl)) * occRatio;

  const float footprintMul = 1.0f + 0.32f * std::max(0.0f, scale - 1.0f);
  heightPx *= heightMul * footprintMul;
  heightPx *= 0.85f + 0.35f * var;

  const float maxH = tileH * (4.25f + 2.25f * std::max(0.0f, scale - 1.0f));
  heightPx = std::clamp(heightPx, tileH * 0.65f, maxH);

  // Tiered silhouettes.
  int tiers = 1;
  if (p.overlay == Overlay::Commercial && lvl >= 2) {
    tiers = (area >= 7) ? 3 : 2;
  } else if (p.overlay == Overlay::Residential && lvl >= 3 && area >= 4) {
    tiers = 2;
  } else if (p.overlay == Overlay::Industrial && lvl >= 3 && area >= 4) {
    tiers = 2;
  }

  float w0 = 1.0f;
  float w1 = 0.0f;
  float w2 = 0.0f;
  if (tiers == 2) {
    w0 = 0.68f;
    w1 = 0.32f;
  } else if (tiers == 3) {
    w0 = 0.62f;
    w1 = 0.26f;
    w2 = 0.12f;
  }

  Vector2 curBase[4];
  CopyDiamond(curBase, base);

  Vector2 top[4];
  CopyDiamond(top, base);

  const float tierShrink = (p.overlay == Overlay::Commercial) ? 0.80f : 0.76f;

  for (int ti = 0; ti < tiers; ++ti) {
    const float w = (ti == 0) ? w0 : ((ti == 1) ? w1 : w2);
    const float hTier = heightPx * w;
    const float shade = 1.0f + 0.06f * static_cast<float>(ti);

    DrawIsoPrism(curBase, hTier, baseColor, brightness, shade, zoom);

    for (int i = 0; i < 4; ++i) {
      top[i] = curBase[i];
      top[i].y -= hTier;
    }

    if (ti + 1 < tiers) {
      Vector2 nextBase[4];
      ShrinkDiamond(nextBase, top, tierShrink);
      CopyDiamond(curBase, nextBase);
    }
  }

  // Roof details on the final top footprint.
  const Vector2 roofCenter = DiamondCenter(top);

  if (p.overlay == Overlay::Residential) {
    // Simple pyramid roof.
    const float roofH = tileH * (0.30f + 0.18f * var);
    Vector2 peak = roofCenter;
    peak.y -= roofH;

    const Color roofC = Mul(baseColor, 1.20f * std::clamp(brightness, 0.55f, 1.20f));
    DrawTriangle(top[0], top[1], peak, roofC);
    DrawTriangle(top[1], top[2], peak, roofC);
    DrawTriangle(top[2], top[3], peak, roofC);
    DrawTriangle(top[3], top[0], peak, roofC);
  } else if (p.overlay == Overlay::Industrial) {
    // Add one or two chimney stacks.
    const int chimneys = std::clamp(1 + area / 6 + static_cast<int>((p.styleSeed >> 22) & 0x1u), 1, 4);
    const float chimScale = 0.18f + 0.02f * var;
    const float chimH = tileH * (0.55f + 0.35f * var);

    Vector2 v[4];
    for (int i = 0; i < 4; ++i) v[i] = Vector2{top[i].x - roofCenter.x, top[i].y - roofCenter.y};

    for (int ci = 0; ci < chimneys; ++ci) {
      const float ox = ((ci == 0) ? 0.28f : -0.18f) * (0.9f + 0.2f * var);
      const float oy = ((ci == 0) ? 0.10f : 0.22f) * (0.9f + 0.2f * var);

      Vector2 chimCenter = Vector2{roofCenter.x + v[0].x * ox + v[3].x * oy, roofCenter.y + v[0].y * ox + v[3].y * oy};

      Vector2 chimBase[4];
      for (int i = 0; i < 4; ++i) {
        chimBase[i] = Vector2{chimCenter.x + v[i].x * chimScale, chimCenter.y + v[i].y * chimScale};
      }

      DrawIsoPrism(chimBase, chimH, baseColor, brightness, 0.65f, zoom);
    }
  } else if (p.overlay == Overlay::Commercial) {
    // Rooftop billboard along a footprint edge. Orient it using the parcel facing so
    // blocks tend to "present" toward nearby roads.
    const float pulse = 0.5f + 0.5f * std::sin(timeSec * 3.0f + static_cast<float>(p.styleSeed & 0xFFu) * 0.06f);
    const unsigned char a = ClampU8(70 + static_cast<int>(150.0f * pulse));
    const Color signC = Color{255, 255, 255, a};

    const float signH = tileH * (0.55f + 0.20f * var);

    int i0 = 0;
    int i1 = 1;
    switch (p.facing & 3u) {
    default:
    case 0: // N
      i0 = 0;
      i1 = 1;
      break;
    case 1: // E
      i0 = 1;
      i1 = 2;
      break;
    case 2: // S
      i0 = 2;
      i1 = 3;
      break;
    case 3: // W
      i0 = 3;
      i1 = 0;
      break;
    }

    Vector2 a0 = Vector2{Lerp(top[i0].x, top[i1].x, 0.22f), Lerp(top[i0].y, top[i1].y, 0.22f)};
    Vector2 b0 = Vector2{Lerp(top[i0].x, top[i1].x, 0.78f), Lerp(top[i0].y, top[i1].y, 0.78f)};
    Vector2 a1 = a0;
    Vector2 b1 = b0;
    a1.y -= signH;
    b1.y -= signH;

    DrawTriangle(a0, b0, b1, signC);
    DrawTriangle(a0, b1, a1, signC);

    if (zoom >= 1.10f) {
      const float thick = 0.85f * std::clamp(zoom, 0.75f, 1.8f);
      const Color oc = Color{0, 0, 0, 55};
      DrawLineEx(a0, b0, thick, oc);
      DrawLineEx(a1, b1, thick, oc);
      DrawLineEx(a0, a1, thick, oc);
      DrawLineEx(b0, b1, thick, oc);
    }
  }

  // Indicators (aggregate across the parcel) when zoomed in.
  const float tileScreenW = tileW * zoom;
  if (tileScreenW >= 28.0f) {
    const float invZoom = 1.0f / std::max(0.001f, zoom);

    Vector2 anchorCenter = TileToWorldCenter(x1, y1, tileW, tileH);
    anchorCenter.y -= baseElevPx;

    const float span = std::max(0.0f, maxDim - 1.0f);
    const float y0 = anchorCenter.y - tileH * (0.18f + 0.07f * span);

    // Pips:
    const float pipR = 2.0f * invZoom;
    const float pipGap = 5.0f * invZoom;
    for (int i = 0; i < lvl; ++i) {
      const float px = anchorCenter.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
      DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
    }

    // Fill bar:
    const float barW = tileW * (0.42f + 0.12f * span) * invZoom;
    const float barH = 3.0f * invZoom;
    const float barX = anchorCenter.x - barW * 0.5f;
    const float barY = y0 + 5.0f * invZoom;
    DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
    DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
  }
}

Renderer::Renderer(int tileW, int tileH, std::uint64_t seed)
    : m_tileW(tileW)
    , m_tileH(tileH)
{
  // Default to flat rendering; Game can enable elevation via setElevationSettings().
  m_elev.maxPixels = 0.0f;
  m_elev.quantizeSteps = 16;
  m_elev.flattenWater = true;
  rebuildTextures(seed);
}

Renderer::~Renderer() { unloadTextures(); }

void Renderer::unloadTextures()
{
  for (auto& t : m_terrainTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
  for (auto& t : m_overlayTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
  for (auto& t : m_roadTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
  for (auto& t : m_bridgeTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }

  unloadBaseCache();

  unloadMinimap();
}

void Renderer::unloadMinimap()
{
  if (m_minimapTex.id != 0) {
    UnloadTexture(m_minimapTex);
    m_minimapTex = Texture2D{};
  }
  m_minimapW = 0;
  m_minimapH = 0;
  m_minimapPixels.clear();
  m_minimapDirty = true;
}

void Renderer::unloadBaseCache()
{
  for (auto& b : m_bands) {
    if (b.rt.id != 0) {
      UnloadRenderTexture(b.rt);
      b.rt = RenderTexture2D{};
    }
  }
  m_bands.clear();
  m_bandMapW = 0;
  m_bandMapH = 0;
  m_bandMaxPixels = 0.0f;
  m_bandCacheDirtyAll = true;
}

void Renderer::markBaseCacheDirtyAll()
{
  m_bandCacheDirtyAll = true;
  for (auto& b : m_bands) b.dirty = true;
}

void Renderer::markBaseCacheDirtyForTiles(const std::vector<Point>& tiles, int mapW, int mapH)
{
  if (tiles.empty()) return;
  if (mapW <= 0 || mapH <= 0) return;

  const int numSums = mapW + mapH - 1;
  const int numBands = (numSums + kBandSums - 1) / kBandSums;

  // If the cache hasn't been created yet (or map dimensions changed), just mark everything dirty.
  if (m_bands.empty() || static_cast<int>(m_bands.size()) != numBands || m_bandMapW != mapW || m_bandMapH != mapH) {
    m_bandCacheDirtyAll = true;
    return;
  }

  auto markSum = [&](int sum) {
    if (sum < 0 || sum >= numSums) return;
    const int bi = sum / kBandSums;
    if (bi >= 0 && bi < static_cast<int>(m_bands.size())) {
      m_bands[static_cast<std::size_t>(bi)].dirty = true;
    }
  };

  auto markTile = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= mapW || y >= mapH) return;
    const int s = x + y;
    markSum(s);
    // Height changes affect cliffs drawn on tiles in front (sum+1). Road edits can also change
    // neighbor auto-tiling masks, so we conservatively dirty the next diagonal too.
    markSum(s + 1);
  };

  for (const Point& p : tiles) {
    // Dirty the edited tile and its 4-neighborhood so auto-tiling road masks update correctly.
    markTile(p.x, p.y);
    markTile(p.x - 1, p.y);
    markTile(p.x + 1, p.y);
    markTile(p.x, p.y - 1);
    markTile(p.x, p.y + 1);
  }
}

void Renderer::ensureBaseCache(const World& world)
{
  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) {
    unloadBaseCache();
    return;
  }

  const int numSums = mapW + mapH - 1;
  const int numBands = (numSums + kBandSums - 1) / kBandSums;

  const bool needsRecreate = (m_bands.empty() || m_bandMapW != mapW || m_bandMapH != mapH ||
                              static_cast<int>(m_bands.size()) != numBands ||
                              m_bandMaxPixels != m_elev.maxPixels);

  if (needsRecreate) {
    unloadBaseCache();

    m_bandMapW = mapW;
    m_bandMapH = mapH;
    m_bandMaxPixels = m_elev.maxPixels;

    m_bands.resize(static_cast<std::size_t>(numBands));

    const float tileW = static_cast<float>(m_tileW);
    const float tileH = static_cast<float>(m_tileH);

    const float pad = 2.0f;
    for (int i = 0; i < numBands; ++i) {
      BandCache& b = m_bands[static_cast<std::size_t>(i)];
      b.sum0 = i * kBandSums;
      b.sum1 = std::min(numSums - 1, b.sum0 + (kBandSums - 1));
      b.dirty = true;

      const BandBounds bb = ComputeBandBounds(b.sum0, b.sum1, mapW, mapH, tileW, tileH, std::max(0.0f, m_bandMaxPixels));
      b.origin = Vector2{bb.minX - pad, bb.minY - pad};

      const int texW = std::max(1, static_cast<int>(std::ceil((bb.maxX - bb.minX) + pad * 2.0f)));
      const int texH = std::max(1, static_cast<int>(std::ceil((bb.maxY - bb.minY) + pad * 2.0f)));

      b.rt = LoadRenderTexture(texW, texH);
      if (b.rt.id != 0) {
        // Keep cached layers crisp when scaling.
        SetTextureFilter(b.rt.texture, TEXTURE_FILTER_POINT);
      }
    }

    m_bandCacheDirtyAll = false;
  }

  if (m_bandCacheDirtyAll) {
    for (auto& b : m_bands) b.dirty = true;
    m_bandCacheDirtyAll = false;
  }
}

void Renderer::rebuildBaseCacheBand(const World& world, BandCache& band)
{
  if (band.rt.id == 0) {
    band.dirty = false;
    return;
  }

  const int mapW = world.width();
  const int mapH = world.height();

  const float tileW = static_cast<float>(m_tileW);
  const float tileH = static_cast<float>(m_tileH);

  const Rectangle src = Rectangle{0, 0, tileW, tileH};
  const Vector2 shift = Vector2{-band.origin.x, -band.origin.y};

  BeginTextureMode(band.rt);
  ClearBackground(BLANK);

  for (int sum = band.sum0; sum <= band.sum1; ++sum) {
    const int x0 = std::max(0, sum - (mapH - 1));
    const int x1 = std::min(mapW - 1, sum);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      const Tile& t = world.at(x, y);

      const float elevPx = TileElevationPx(t, m_elev);
      const Vector2 baseCenterW = TileToWorldCenter(x, y, tileW, tileH);
      const Vector2 baseCenter{baseCenterW.x + shift.x, baseCenterW.y + shift.y};
      const Vector2 center{baseCenter.x, baseCenter.y - elevPx};

      const Rectangle dst = Rectangle{center.x - tileW * 0.5f, center.y - tileH * 0.5f, tileW, tileH};

      // Per-tile lighting based on height + variation (same as the immediate renderer,
      // except we omit animated water shimmer for cache stability).
      const float v = (static_cast<float>(t.variation) / 255.0f - 0.5f) * 0.10f;
      float brightness = 0.85f + t.height * 0.30f + v;

      // Draw terrain
      DrawTexturePro(terrain(t.terrain), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));

      // Draw cliff walls for higher neighbors behind this tile (same logic as immediate path).
      {
        Vector2 baseCorners[4];
        TileDiamondCorners(baseCenter, tileW, tileH, baseCorners);

        const float eps = 0.5f;

        auto drawCliffEdge = [&](Vector2 e0, Vector2 e1, float topElev, float botElev, Color c) {
          if (topElev <= botElev + eps) return;
          Vector2 top0 = e0;
          Vector2 top1 = e1;
          Vector2 bot0 = e0;
          Vector2 bot1 = e1;
          top0.y -= topElev;
          top1.y -= topElev;
          bot0.y -= botElev;
          bot1.y -= botElev;

          DrawTriangle(top0, top1, bot1, c);
          DrawTriangle(top0, bot1, bot0, c);
        };

        if (x > 0) {
          const Tile& n = world.at(x - 1, y);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[3], baseCorners[0], ne, elevPx, Mul(base, 0.70f));
        }

        if (y > 0) {
          const Tile& n = world.at(x, y - 1);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[0], baseCorners[1], ne, elevPx, Mul(base, 0.85f));
        }
      }

      // Draw overlay (base view only: no traffic/goods/outside/heatmap tinting).
      if (t.overlay == Overlay::Road) {
        const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);
        Texture2D& rtex = (t.terrain == Terrain::Water) ? bridge(mask) : road(mask);
        DrawTexturePro(rtex, src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));
      } else if (t.overlay != Overlay::None) {
        DrawTexturePro(overlay(t.overlay), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));
      }
    }
  }

  EndTextureMode();
  band.dirty = false;
}

Renderer::MinimapLayout Renderer::minimapLayout(const World& world, int screenW, int screenH) const
{
  return ComputeMinimapLayout(world.width(), world.height(), screenW, screenH);
}

void Renderer::ensureMinimapUpToDate(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  const bool needsRecreate = (m_minimapTex.id == 0) || (m_minimapW != w) || (m_minimapH != h) ||
                             (m_minimapPixels.size() != n);

  if (needsRecreate) {
    unloadMinimap();
    m_minimapW = w;
    m_minimapH = h;
    m_minimapPixels.assign(n, BLANK);
    m_minimapDirty = true;
  }

  if (!m_minimapDirty && m_minimapTex.id != 0) return;

  // Rebuild pixel buffer.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      m_minimapPixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          MinimapColorForTile(t);
    }
  }

  if (m_minimapTex.id == 0) {
    // Create a GPU texture directly from our CPU pixel buffer.
    Image img{};
    img.data = m_minimapPixels.data();
    img.width = w;
    img.height = h;
    img.mipmaps = 1;
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    m_minimapTex = LoadTextureFromImage(img);
    if (m_minimapTex.id != 0) {
      // Keep the minimap crisp when scaling up.
      SetTextureFilter(m_minimapTex, TEXTURE_FILTER_POINT);
    }
  } else {
    UpdateTexture(m_minimapTex, m_minimapPixels.data());
  }

  m_minimapDirty = false;
}

bool Renderer::exportMinimapThumbnail(const World& world, const char* fileName, int maxSize)
{
  if (fileName == nullptr || fileName[0] == '\0') return false;

  ensureMinimapUpToDate(world);
  if (m_minimapW <= 0 || m_minimapH <= 0) return false;
  if (m_minimapPixels.empty()) return false;

  // Build an Image from our CPU pixel buffer. We copy because raylib image
  // processing utilities can reallocate the data.
  Image base{};
  base.data = (void*)m_minimapPixels.data();
  base.width = m_minimapW;
  base.height = m_minimapH;
  base.mipmaps = 1;
  base.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

  Image img = ImageCopy(base);
  if (!IsImageReadyCompat(img)) return false;

  const int ms = std::max(1, maxSize);
  const int w = img.width;
  const int h = img.height;
  const int maxDim = std::max(w, h);

  if (maxDim > ms) {
    const float scale = static_cast<float>(ms) / static_cast<float>(maxDim);
    const int nw = std::max(1, static_cast<int>(std::lround(static_cast<float>(w) * scale)));
    const int nh = std::max(1, static_cast<int>(std::lround(static_cast<float>(h) * scale)));
    ImageResize(&img, nw, nh);
  }

  const bool ok = ExportImage(img, fileName);
  UnloadImage(img);
  return ok;
}

bool Renderer::exportWorldOverview(const World& world, const char* fileName, int maxSize)
{
  if (fileName == nullptr || fileName[0] == '\0') return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  // Compute a conservative bounding box for the full isometric map in *world space*.
  //
  // Notes:
  // - This is based on the diamond tile geometry and the maximum configured elevation.
  // - We add a small extra margin at the top to avoid clipping tall zone "buildings".
  const float tileW = static_cast<float>(m_tileW);
  const float tileH = static_cast<float>(m_tileH);
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;

  const float maxElev = std::max(0.0f, m_elev.maxPixels);
  const float extraTop = tileH * 5.0f; // safety margin for extruded zone buildings

  const float left = -static_cast<float>(h) * halfW;
  const float right = static_cast<float>(w) * halfW;
  const float top = -halfH - maxElev - extraTop;
  const float bottom = static_cast<float>(w + h - 1) * halfH;

  const float worldW = std::max(1.0f, right - left);
  const float worldH = std::max(1.0f, bottom - top);
  const float maxDim = std::max(worldW, worldH);

  int ms = std::max(64, maxSize);

  // Try to allocate a render texture; if this fails (GPU limits), fall back to smaller sizes.
  RenderTexture2D rt{};
  float zoom = 1.0f;
  int texW = 0;
  int texH = 0;

  for (int attempt = 0; attempt < 4; ++attempt) {
    zoom = (maxDim > static_cast<float>(ms)) ? (static_cast<float>(ms) / maxDim) : 1.0f;
    texW = std::max(1, static_cast<int>(std::lround(worldW * zoom)));
    texH = std::max(1, static_cast<int>(std::lround(worldH * zoom)));

    rt = LoadRenderTexture(texW, texH);
    if (rt.id != 0) break;

    ms = std::max(64, ms / 2);
  }

  if (rt.id == 0) return false;

  Camera2D cam{};
  cam.target = Vector2{(left + right) * 0.5f, (top + bottom) * 0.5f};
  cam.offset = Vector2{static_cast<float>(texW) * 0.5f, static_cast<float>(texH) * 0.5f};
  cam.zoom = zoom;
  cam.rotation = 0.0f;

  // Render the full map without the band cache to avoid nested BeginTextureMode calls.
  const bool prevCache = m_useBandCache;
  m_useBandCache = false;

  BeginTextureMode(rt);
  ClearBackground(BLANK);
  drawWorld(world, cam, texW, texH,
            /*timeSec=*/0.0f,
            /*hovered=*/std::nullopt,
            /*drawGrid=*/false,
            /*brushRadius=*/0,
            /*selected=*/std::nullopt,
            /*highlightPath=*/nullptr,
            /*roadToEdgeMask=*/nullptr,
            /*roadTraffic=*/nullptr,
            /*trafficMax=*/0,
            /*roadGoodsTraffic=*/nullptr,
            /*goodsMax=*/0,
            /*commercialGoodsFill=*/nullptr,
            /*heatmap=*/nullptr,
            /*heatmapRamp=*/HeatmapRamp::Good);
  EndTextureMode();

  m_useBandCache = prevCache;

  Image img = LoadImageFromTexture(rt.texture);
  if (!IsImageReadyCompat(img)) {
    UnloadRenderTexture(rt);
    return false;
  }

  // Render textures are flipped vertically when read back.
  ImageFlipVertical(&img);

  const bool ok = ExportImage(img, fileName);
  UnloadImage(img);
  UnloadRenderTexture(rt);
  return ok;
}

static int TerrainIndex(Terrain t)
{
  switch (t) {
  case Terrain::Water: return 0;
  case Terrain::Sand: return 1;
  case Terrain::Grass: return 2;
  default: return 2;
  }
}

static Color TerrainCliffBaseColor(Terrain t)
{
  switch (t) {
  case Terrain::Water: return Color{20, 60, 120, 255};
  case Terrain::Sand: return Color{180, 150, 90, 255};
  case Terrain::Grass: return Color{45, 120, 65, 255};
  default: return Color{45, 120, 65, 255};
  }
}

static int OverlayIndex(Overlay o)
{
  switch (o) {
  case Overlay::None: return 0;
  case Overlay::Road: return 1;
  case Overlay::Residential: return 2;
  case Overlay::Commercial: return 3;
  case Overlay::Industrial: return 4;
  case Overlay::Park: return 5;
  default: return 0;
  }
}

Texture2D& Renderer::terrain(Terrain t) { return m_terrainTex[static_cast<std::size_t>(TerrainIndex(t))]; }

Texture2D& Renderer::overlay(Overlay o) { return m_overlayTex[static_cast<std::size_t>(OverlayIndex(o))]; }

Texture2D& Renderer::road(std::uint8_t mask) { return m_roadTex[static_cast<std::size_t>(mask & 0x0Fu)]; }
Texture2D& Renderer::bridge(std::uint8_t mask) {
  return m_bridgeTex[static_cast<std::size_t>(mask & 0x0Fu)];
}


Color Renderer::BrightnessTint(float b)
{
  const int v = static_cast<int>(std::round(255.0f * std::clamp(b, 0.0f, 1.5f)));
  const unsigned char u = ClampU8(v);
  return Color{u, u, u, 255};
}

void Renderer::rebuildTextures(std::uint64_t seed)
{
  unloadTextures();

  const std::uint32_t s = static_cast<std::uint32_t>(seed);

  // --- Terrain ---
  // Water
  m_terrainTex[0] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xA1B2C3D4u);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    // Subtle diagonal waves (purely procedural).
    const float waves = 0.06f * std::sin((x * 0.35f + y * 0.70f));
    const float b = 1.0f + n + waves;

    Color base = Color{40, 95, 210, 255};
    base = Mul(base, b);

    // Slightly fade edges to reduce harsh tiling.
    base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 4.0f, 0.0f, 1.0f));
    return base;
  });

  // Sand
  m_terrainTex[1] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xBEEFBEEFu);
    const float n = (Frac01(h) - 0.5f) * 0.18f;

    Color base = Color{200, 186, 135, 255};
    base = Mul(base, 1.0f + n);

    // Grain speckles.
    if ((h & 0x1Fu) == 0x1Fu) base = Mul(base, 0.85f);

    base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
    return base;
  });

  // Grass
  m_terrainTex[2] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0x12345678u);
    const float n = (Frac01(h) - 0.5f) * 0.22f;

    Color base = Color{70, 170, 90, 255};
    base = Mul(base, 1.0f + n);

    // Tiny darker "blades" of grass.
    if ((h & 0x7Fu) == 0x3Fu) base = Mul(base, 0.78f);

    base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
    return base;
  });

  // --- Overlays ---
  // None: keep as an empty texture (id=0), we won't draw it.
  m_overlayTex[0] = Texture2D{};

  // Road: auto-tiling variants (mask stored in tile.variation low bits).
  // We keep m_overlayTex[1] empty; roads are drawn from m_roadTex[0..15].
  m_overlayTex[1] = Texture2D{};

  auto makeRoadVariant = [&](std::uint8_t mask) -> Texture2D {
    return MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
      const std::uint32_t h = HashCoords32(x, y, s ^ 0x0F0F0F0Fu ^ (static_cast<std::uint32_t>(mask) * 0x9E3779B9u));
      const float n = (Frac01(h) - 0.5f) * 0.08f;

      const float px = d.nx;
      const float py = d.ny;

      // Signed distance to a union of "capsule" segments from center to each connected edge.
      const float roadW = 0.14f;
      const float centerR = roadW * 1.10f;
      float sd = std::sqrt(px * px + py * py) - centerR;

      float bestSegDist = 1.0e9f;
      float bestSegT = 0.0f;

      auto consider = [&](bool enabled, float ex, float ey) {
        if (!enabled) return;
        float tproj = 0.0f;
        const float dist = DistPointSegment(px, py, 0.0f, 0.0f, ex, ey, tproj);
        sd = std::min(sd, dist - roadW);
        if (dist < bestSegDist) {
          bestSegDist = dist;
          bestSegT = tproj;
        }
      };

      // Bit layout matches World::computeRoadMask().
      consider((mask & 0x01u) != 0u, 0.5f, -0.5f); // up-right
      consider((mask & 0x02u) != 0u, 0.5f, 0.5f);  // down-right
      consider((mask & 0x04u) != 0u, -0.5f, 0.5f); // down-left
      consider((mask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

      // Outside the road shape.
      if (sd > 0.0f) return Color{0, 0, 0, 0};

      // Asphalt base.
      Color base = Color{90, 90, 95, 230};
      base = Mul(base, 1.0f + n);

      // Dashed centerline on the closest segment (avoid the intersection blob).
      const float centerDist = std::sqrt(px * px + py * py);
      if (bestSegDist < (roadW * 0.25f) && centerDist > centerR * 0.6f) {
        const int dash = static_cast<int>(std::floor(bestSegT * 10.0f + static_cast<float>(mask) * 0.15f));
        if ((dash & 1) == 0) {
          base = Color{220, 220, 210, 240};
        }
      }

      // Soft edges.
      const float edgeSoft = 0.05f;
      const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
      base.a = static_cast<unsigned char>(static_cast<float>(base.a) * a);
      return base;
    });
  };

  for (int i = 0; i < 16; ++i) {
    m_roadTex[static_cast<std::size_t>(i)] = makeRoadVariant(static_cast<std::uint8_t>(i));
  }

  auto makeBridgeVariant = [&](std::uint8_t mask) -> Texture2D {
    return MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
      const std::uint32_t h = HashCoords32(x, y, s ^ 0xB00B1E5u ^ (static_cast<std::uint32_t>(mask) * 0x7F4A7C15u));
      const float n = (Frac01(h) - 0.5f) * 0.10f;

      const float px = d.nx;
      const float py = d.ny;

      // Same connectivity geometry as roads, but rendered as a wooden / concrete-ish bridge deck.
      const float roadW = 0.14f;
      const float centerR = roadW * 1.10f;
      float sd = std::sqrt(px * px + py * py) - centerR;

      float bestSegDist = 1.0e9f;
      float bestSegT = 0.0f;

      auto consider = [&](bool enabled, float ex, float ey) {
        if (!enabled) return;
        float tproj = 0.0f;
        const float dist = DistPointSegment(px, py, 0.0f, 0.0f, ex, ey, tproj);
        sd = std::min(sd, dist - roadW);
        if (dist < bestSegDist) {
          bestSegDist = dist;
          bestSegT = tproj;
        }
      };

      // Bit layout matches World::computeRoadMask().
      consider((mask & 0x01u) != 0u, 0.5f, -0.5f); // up-right
      consider((mask & 0x02u) != 0u, 0.5f, 0.5f);  // down-right
      consider((mask & 0x04u) != 0u, -0.5f, 0.5f); // down-left
      consider((mask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

      if (sd > 0.0f) return Color{0, 0, 0, 0};

      // Deck base.
      Color base = Color{160, 130, 95, 235};
      base = Mul(base, 1.0f + n);

      // Plank pattern along the closest segment direction (avoid the intersection blob).
      const float centerDist = std::sqrt(px * px + py * py);
      if (bestSegDist < (roadW * 0.60f) && centerDist > centerR * 0.55f) {
        const int plank = static_cast<int>(std::floor(bestSegT * 18.0f + static_cast<float>(mask) * 0.21f));
        if ((plank & 1) == 0) base = Mul(base, 0.92f);
      }

      // Darken edges to suggest guard rails / curbs.
      if (-sd < 0.012f) base = Mul(base, 0.68f);

      // Soft edges.
      const float edgeSoft = 0.05f;
      const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
      base.a = static_cast<unsigned char>(static_cast<float>(base.a) * a);
      return base;
    });
  };

  for (int i = 0; i < 16; ++i) {
    m_bridgeTex[static_cast<std::size_t>(i)] = makeBridgeVariant(static_cast<std::uint8_t>(i));
  }

  // Residential
  m_overlayTex[2] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0001u);
    const float n = (Frac01(h) - 0.5f) * 0.12f;

    Color roof = Color{190, 70, 65, 255};
    roof = Mul(roof, 1.0f + n);

    // Simple roof tiles pattern.
    if ((x + y) % 6 == 0) roof = Mul(roof, 0.86f);

    // Slight vignette.
    roof = Mul(roof, 0.92f + 0.10f * d.edge);
    return roof;
  });

  // Commercial
  m_overlayTex[3] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0002u);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    Color c = Color{70, 115, 190, 255};
    c = Mul(c, 1.0f + n);

    // Windows pattern.
    if ((x / 3 + y / 2) % 5 == 0) c = Mul(c, 1.15f);

    c = Mul(c, 0.92f + 0.10f * d.edge);
    return c;
  });

  // Industrial
  m_overlayTex[4] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0003u);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    Color c = Color{210, 180, 75, 255};
    c = Mul(c, 1.0f + n);

    // Hazard stripes
    if (((x + y) / 3) % 2 == 0) c = Mul(c, 0.85f);

    c = Mul(c, 0.92f + 0.10f * d.edge);
    return c;
  });

  // Park (transparent edges so grass can show through)
  m_overlayTex[5] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0004u);
    const float n = (Frac01(h) - 0.5f) * 0.12f;

    Color c = Color{60, 190, 95, 230};
    c = Mul(c, 1.0f + n);

    // Procedural "trees" (dark dots).
    if ((h & 0xFFu) == 0x7Au) c = Color{25, 110, 55, 240};

    const float a = std::clamp(d.edge * 7.0f, 0.0f, 1.0f);
    c.a = static_cast<unsigned char>(static_cast<float>(c.a) * a);
    return c;
  });
}

namespace {

inline unsigned char LerpU8(unsigned char a, unsigned char b, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return static_cast<unsigned char>(std::round(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * t));
}

inline Color LerpColor(Color a, Color b, float t) {
  return Color{LerpU8(a.r, b.r, t), LerpU8(a.g, b.g, t), LerpU8(a.b, b.b, t), LerpU8(a.a, b.a, t)};
}

Color HeatmapColor(float v, Renderer::HeatmapRamp ramp) {
  v = std::clamp(v, 0.0f, 1.0f);
  const float alphaF = std::clamp(70.0f + 110.0f * v, 0.0f, 255.0f);

  const Color red{220, 70, 70, static_cast<unsigned char>(alphaF)};
  const Color yellow{240, 220, 90, static_cast<unsigned char>(alphaF)};
  const Color green{70, 220, 120, static_cast<unsigned char>(alphaF)};

  if (ramp == Renderer::HeatmapRamp::Bad) {
    // 0 (good) -> green ... 1 (bad) -> red
    if (v < 0.5f) {
      return LerpColor(green, yellow, v / 0.5f);
    }
    return LerpColor(yellow, red, (v - 0.5f) / 0.5f);
  }

  // 0 (bad) -> red ... 1 (good) -> green
  if (v < 0.5f) {
    return LerpColor(red, yellow, v / 0.5f);
  }
  return LerpColor(yellow, green, (v - 0.5f) / 0.5f);
}

} // namespace

void Renderer::drawWorld(const World& world, const Camera2D& camera, int screenW, int screenH, float timeSec,
                         std::optional<Point> hovered,
                         bool drawGrid, int brushRadius, std::optional<Point> selected,
                         const std::vector<Point>* highlightPath, const std::vector<std::uint8_t>* roadToEdgeMask,
                         const std::vector<std::uint16_t>* roadTraffic, int trafficMax,
                         const std::vector<std::uint16_t>* roadGoodsTraffic, int goodsMax,
                         const std::vector<std::uint8_t>* commercialGoodsFill,
                         const std::vector<float>* heatmap,
                         HeatmapRamp heatmapRamp,
                         bool showDistrictOverlay,
                         int highlightDistrict,
                         bool showDistrictBorders,
                         bool mergeZoneBuildings)
{
  const int w = world.width();
  const int h = world.height();

  const bool showOutside = (roadToEdgeMask && w > 0 && h > 0 &&
                            roadToEdgeMask->size() ==
                                static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showTraffic = (roadTraffic && trafficMax > 0 && w > 0 && h > 0 &&
                            roadTraffic->size() ==
                                static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showGoods = (roadGoodsTraffic && goodsMax > 0 && w > 0 && h > 0 &&
                         roadGoodsTraffic->size() ==
                             static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showCommercialGoods = (commercialGoodsFill && w > 0 && h > 0 &&
                                  commercialGoodsFill->size() ==
                                      static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showHeatmap = (heatmap && w > 0 && h > 0 &&
                            heatmap->size() ==
                                static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  // Base-cache usage is only valid when the base view is being rendered. Any debug overlay that
  // tints roads/zones per-tile (traffic, goods, outside-access, heatmap) falls back to the
  // immediate path.
  const bool useBaseCache = (m_useBandCache && !showOutside && !showTraffic && !showGoods &&
                             !showCommercialGoods && !showHeatmap);

  bool baseCacheReady = false;
  if (useBaseCache) {
    ensureBaseCache(world);
    for (auto& b : m_bands) {
      if (b.dirty) rebuildBaseCacheBand(world, b);
      if (b.rt.id != 0) baseCacheReady = true;
    }
  }

  // Compute a conservative visible tile range based on the current camera view.
  TileRect vis = ComputeVisibleTileRect(camera, screenW, screenH, w, h,
                                             static_cast<float>(m_tileW), static_cast<float>(m_tileH),
                                             std::max(0.0f, m_elev.maxPixels));


  // Build merged multi-tile zone parcels if enabled and zoomed in enough to see buildings.
  // (At lower zoom levels, buildings/indicators are skipped anyway.)
  const float tileScreenWGlobal = static_cast<float>(m_tileW) * camera.zoom;
  const bool useMergedZoneBuildings = (mergeZoneBuildings && tileScreenWGlobal >= 26.0f);
  if (useMergedZoneBuildings) {
    BuildZoneBuildingParcels(world, m_zoneParcelsScratch);
  } else {
    m_zoneParcelsScratch.clear();
  }

  if (useMergedZoneBuildings) {
    // Parcels can extend up to 3 tiles beyond the visible rect (e.g., 4x2 / 4x1 footprints).
    // Extend the draw rect so parcel anchors aren't culled when only the NW portion is on-screen.
    vis.x1 = std::min(vis.x1 + 3, w);
    vis.y1 = std::min(vis.y1 + 3, h);
  }

  BeginMode2D(camera);

  // Cached base path: draw the static base world (terrain + cliffs + base overlays) from
  // off-screen render targets. Dynamic per-tile extras (grid, buildings, indicators) are drawn
  // in-order on top.
  if (useBaseCache && baseCacheReady) {
    for (const auto& b : m_bands) {
      if (b.rt.id == 0) continue;
      const Rectangle src = Rectangle{0.0f, 0.0f, static_cast<float>(b.rt.texture.width),
                                      -static_cast<float>(b.rt.texture.height)};
      DrawTextureRec(b.rt.texture, src, b.origin, WHITE);
    }

    // Draw order: diagonals by (x+y) so nearer tiles draw last.
    const int minSum = vis.minX + vis.minY;
    const int maxSum = vis.maxX + vis.maxY;
    for (int sum = minSum; sum <= maxSum; ++sum) {
      const int x0 = std::max(vis.minX, sum - vis.maxY);
      const int x1 = std::min(vis.maxX, sum - vis.minY);
      for (int x = x0; x <= x1; ++x) {
        const int y = sum - x;
        if (y < vis.minY || y > vis.maxY) continue;

        const Tile& t = world.at(x, y);

        const float tileW = static_cast<float>(m_tileW);
        const float tileH = static_cast<float>(m_tileH);

        const float elevPx = TileElevationPx(t, m_elev);
        const Vector2 baseCenter = TileToWorldCenter(x, y, tileW, tileH);
        const Vector2 center{baseCenter.x, baseCenter.y - elevPx};

        // Per-tile lighting based on height + variation (matches base-cache draw, minus water shimmer).
        const float v = (static_cast<float>(t.variation) / 255.0f - 0.5f) * 0.10f;
        const float brightness = 0.85f + t.height * 0.30f + v;

        Vector2 corners[4];
        bool haveCorners = false;
        auto ensureCorners = [&]() {
          if (!haveCorners) {
            TileDiamondCorners(center, tileW, tileH, corners);
            haveCorners = true;
          }
        };

        // District overlay (fill). District 0 is treated as "unassigned" and is left transparent.
        if (showDistrictOverlay) {
          const std::uint8_t d = static_cast<std::uint8_t>(t.district) & 7u;
          if (d != 0u) {
            unsigned char alpha = 65;
            if (highlightDistrict >= 0) {
              alpha = (d == static_cast<std::uint8_t>(highlightDistrict & 7)) ? 95 : 22;
            }

            if (alpha > 0) {
              ensureCorners();
              const Color c = DistrictFillColor(d, alpha);
              DrawTriangle(corners[0], corners[1], corners[2], c);
              DrawTriangle(corners[0], corners[2], corners[3], c);
            }
          }
        }

        // Optional grid overlay.
        if (drawGrid) {
          ensureCorners();
          const Color c = Color{255, 255, 255, 35};
          DrawLineV(corners[0], corners[1], c);
          DrawLineV(corners[1], corners[2], c);
          DrawLineV(corners[2], corners[3], c);
          DrawLineV(corners[3], corners[0], c);
        }

        // District borders (draw after grid so they stay visible).
        if (showDistrictBorders) {
          ensureCorners();
          const Color border = Color{0, 0, 0, 170};
          const float thick = 2.0f / std::max(0.001f, camera.zoom);
          const std::uint8_t d = static_cast<std::uint8_t>(t.district) & 7u;
          if (x + 1 < w) {
            const std::uint8_t dn = static_cast<std::uint8_t>(world.at(x + 1, y).district) & 7u;
            if (dn != d) DrawLineEx(corners[1], corners[2], thick, border);
          }
          if (y + 1 < h) {
            const std::uint8_t dn = static_cast<std::uint8_t>(world.at(x, y + 1).district) & 7u;
            if (dn != d) DrawLineEx(corners[2], corners[3], thick, border);
          }
        }

        // Zone buildings + indicators.
        const float tileScreenW = tileW * camera.zoom;
        const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                             t.overlay == Overlay::Industrial);

        if (useMergedZoneBuildings && isZone &&
            idx < static_cast<int>(m_zoneParcelsScratch.tileToParcel.size())) {
          const int parcelAll = m_zoneParcelsScratch.tileToParcel[idx];
          const int parcelAnchor = m_zoneParcelsScratch.anchorToParcel[idx];

          if (parcelAll >= 0) {
            // Only draw once per parcel (on the anchor tile).
            if (parcelAnchor >= 0 &&
                parcelAnchor < static_cast<int>(m_zoneParcelsScratch.parcels.size())) {
              const ZoneBuildingParcel& p =
                  m_zoneParcelsScratch.parcels[static_cast<std::size_t>(parcelAnchor)];

              if (p.isMultiTile()) {
                DrawMergedZoneBuildingAndIndicators(p, world, m_elev, tileW, tileH, camera.zoom, timeSec);
              } else {
                DrawZoneBuilding(t, tileW, tileH, camera.zoom, center, brightness);
              if (tileScreenW >= 28.0f) {
                const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
                const int lvl = std::clamp<int>(t.level, 1, 3);
                const int cap = CapacityForOverlayLevel(t.overlay, lvl);
                const int occ = std::clamp<int>(static_cast<int>(t.occupants), 0, cap);
                const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
                const float y0 = center.y - tileH * 0.18f;

                // Pips:
                const float pipR = 2.0f * invZoom;
                const float pipGap = 5.0f * invZoom;
                for (int i = 0; i < lvl; ++i) {
                  const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
                  DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
                }

                // Fill bar:
                const float barW = tileW * 0.42f * invZoom;
                const float barH = 3.0f * invZoom;
                const float barX = center.x - barW * 0.5f;
                const float barY = y0 + 5.0f * invZoom;
                DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
                DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
              }
              }
            }
          } else {
            // Defensive fallback: weird zone tile not parcelized (e.g., zone-on-water in tests).
            DrawZoneBuilding(t, tileW, tileH, camera.zoom, center, brightness);
              if (tileScreenW >= 28.0f) {
                const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
                const int lvl = std::clamp<int>(t.level, 1, 3);
                const int cap = CapacityForOverlayLevel(t.overlay, lvl);
                const int occ = std::clamp<int>(static_cast<int>(t.occupants), 0, cap);
                const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
                const float y0 = center.y - tileH * 0.18f;

                // Pips:
                const float pipR = 2.0f * invZoom;
                const float pipGap = 5.0f * invZoom;
                for (int i = 0; i < lvl; ++i) {
                  const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
                  DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
                }

                // Fill bar:
                const float barW = tileW * 0.42f * invZoom;
                const float barH = 3.0f * invZoom;
                const float barX = center.x - barW * 0.5f;
                const float barY = y0 + 5.0f * invZoom;
                DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
                DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
              }
          }
        } else {
          // Per-tile fallback.
          DrawZoneBuilding(t, tileW, tileH, camera.zoom, center, brightness);
          if (isZone) {
              if (tileScreenW >= 28.0f) {
                const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
                const int lvl = std::clamp<int>(t.level, 1, 3);
                const int cap = CapacityForOverlayLevel(t.overlay, lvl);
                const int occ = std::clamp<int>(static_cast<int>(t.occupants), 0, cap);
                const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
                const float y0 = center.y - tileH * 0.18f;

                // Pips:
                const float pipR = 2.0f * invZoom;
                const float pipGap = 5.0f * invZoom;
                for (int i = 0; i < lvl; ++i) {
                  const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
                  DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
                }

                // Fill bar:
                const float barW = tileW * 0.42f * invZoom;
                const float barH = 3.0f * invZoom;
                const float barX = center.x - barW * 0.5f;
                const float barY = y0 + 5.0f * invZoom;
                DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
                DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
              }
          }
        }

// Road indicators: show small pips for upgraded road class (2..3) when zoomed in.
        if (t.overlay == Overlay::Road && tileScreenW >= 28.0f) {
          const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
          if (lvl > 1) {
            const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
            const float y0 = center.y - static_cast<float>(m_tileH) * 0.02f;

            const float pip = 4.0f * invZoom;
            const float gap = 1.5f * invZoom;
            const float groupW = pip * 3.0f + gap * 2.0f;
            const float groupX0 = center.x - groupW * 0.5f;

            for (int i = 0; i < 3; ++i) {
              Rectangle r{groupX0 + static_cast<float>(i) * (pip + gap), y0, pip, pip};
              DrawRectangleRec(r, Color{0, 0, 0, 110});
              DrawRectangleLinesEx(r, 1.0f * invZoom, Color{255, 255, 255, 55});
              if (i < lvl) {
                Rectangle f{r.x + 1.0f * invZoom, r.y + 1.0f * invZoom, r.width - 2.0f * invZoom,
                            r.height - 2.0f * invZoom};
                DrawRectangleRec(f, Color{255, 255, 255, 160});
              }
            }
          }
        }
      }
    }
  } else {

  // Draw order: diagonals by (x+y) so nearer tiles draw last.
  const int minSum = vis.minX + vis.minY;
  const int maxSum = vis.maxX + vis.maxY;
  for (int sum = minSum; sum <= maxSum; ++sum) {
    const int x0 = std::max(vis.minX, sum - vis.maxY);
    const int x1 = std::min(vis.maxX, sum - vis.minY);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      if (y < vis.minY || y > vis.maxY) continue;

      const Tile& t = world.at(x, y);

      const float tileW = static_cast<float>(m_tileW);
      const float tileH = static_cast<float>(m_tileH);

      const float elevPx = TileElevationPx(t, m_elev);
      const Vector2 baseCenter = TileToWorldCenter(x, y, tileW, tileH);
      const Vector2 center{baseCenter.x, baseCenter.y - elevPx};

      const Rectangle dst = Rectangle{center.x - tileW * 0.5f, center.y - tileH * 0.5f, tileW, tileH};
      const Rectangle src = Rectangle{0, 0, tileW, tileH};

      // Per-tile lighting based on height + variation.
      const float v = (static_cast<float>(t.variation) / 255.0f - 0.5f) * 0.10f;
      float brightness = 0.85f + t.height * 0.30f + v;
      const float baseBrightness = brightness;

      if (t.terrain == Terrain::Water) {
        // Slight animated shimmer for water (no textures-from-disk, still procedural).
        brightness += 0.05f * std::sin(timeSec * 2.0f + (x + y) * 0.4f);
      }

      // Draw terrain
      DrawTexturePro(terrain(t.terrain), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));

      // Draw cliff walls for higher neighbors behind this tile. We draw these AFTER the terrain top,
      // but BEFORE overlays, so roads/zones stay on top.
      {
        // For the shared edges, we use the base (non-elevated) diamond corners.
        Vector2 baseCorners[4];
        TileDiamondCorners(baseCenter, tileW, tileH, baseCorners);

        const float eps = 0.5f;

        auto drawCliffEdge = [&](Vector2 e0, Vector2 e1, float topElev, float botElev, Color c) {
          if (topElev <= botElev + eps) return;
          Vector2 top0 = e0;
          Vector2 top1 = e1;
          Vector2 bot0 = e0;
          Vector2 bot1 = e1;
          top0.y -= topElev;
          top1.y -= topElev;
          bot0.y -= botElev;
          bot1.y -= botElev;

          DrawTriangle(top0, top1, bot1, c);
          DrawTriangle(top0, bot1, bot0, c);
        };

        // Left neighbor (x-1, y) is behind; if it's higher, we see a cliff along our top-left edge (3-0).
        if (x > 0) {
          const Tile& n = world.at(x - 1, y);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[3], baseCorners[0], ne, elevPx, Mul(base, 0.70f));
        }

        // Up neighbor (x, y-1) is behind; if it's higher, we see a cliff along our top-right edge (0-1).
        if (y > 0) {
          const Tile& n = world.at(x, y - 1);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[0], baseCorners[1], ne, elevPx, Mul(base, 0.85f));
        }
      }

      // Draw overlay
      if (t.overlay == Overlay::Road) {
        const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);

        Color tint = BrightnessTint((t.terrain == Terrain::Water) ? baseBrightness : brightness);

        bool disconnected = false;

        if (showOutside) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < roadToEdgeMask->size() && (*roadToEdgeMask)[idx] == 0) {
            // Disconnected road component: tint red so it's obvious why zones may not function.
            disconnected = true;
            tint.g = ClampU8(static_cast<int>(std::round(static_cast<float>(tint.g) * 0.35f)));
            tint.b = ClampU8(static_cast<int>(std::round(static_cast<float>(tint.b) * 0.35f)));
            tint.r = ClampU8(static_cast<int>(
                std::round(std::min(255.0f, static_cast<float>(tint.r) * 1.10f + 20.0f))));
          }
        }

        if (!disconnected && showTraffic) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < roadTraffic->size()) {
            const int tv = static_cast<int>((*roadTraffic)[idx]);
            if (tv > 0) {
              const float tnorm = std::clamp(static_cast<float>(tv) / std::max(1.0f, static_cast<float>(trafficMax)),
                                             0.0f, 1.0f);
              // Emphasize low flows while keeping very busy roads distinct.
              const float s = std::pow(tnorm, 0.35f);

              const float rf = std::min(255.0f, static_cast<float>(tint.r) * (1.0f + 0.15f * s) + 85.0f * s);
              const float gf = static_cast<float>(tint.g) * (1.0f - 0.70f * s);
              const float bf = static_cast<float>(tint.b) * (1.0f - 0.70f * s);

              tint.r = ClampU8(static_cast<int>(std::round(rf)));
              tint.g = ClampU8(static_cast<int>(std::round(gf)));
              tint.b = ClampU8(static_cast<int>(std::round(bf)));

              // Extra hint: highlight tiles that exceed their class-dependent capacity.
              const int baseCap = std::max(0, TrafficConfig{}.roadTileCapacity);
              if (baseCap > 0) {
                const int cap = RoadCapacityForLevel(baseCap, static_cast<int>(t.level));
                if (cap > 0 && tv > cap) {
                  const float over = std::clamp(static_cast<float>(tv - cap) / static_cast<float>(cap), 0.0f, 1.0f);
                  tint = Lerp(tint, Color{255, 80, 80, 255}, 0.40f + 0.45f * over);
                }
              }
            }
          }
        }


        if (!disconnected && showGoods) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < roadGoodsTraffic->size()) {
            const int gv = static_cast<int>((*roadGoodsTraffic)[idx]);
            if (gv > 0) {
              const float gnorm = std::clamp(static_cast<float>(gv) / std::max(1.0f, static_cast<float>(goodsMax)),
                                            0.0f, 1.0f);
              const float s = std::pow(gnorm, 0.35f);

              const float bf = std::min(255.0f, static_cast<float>(tint.b) * (1.0f + 0.15f * s) + 85.0f * s);
              const float gf = static_cast<float>(tint.g) * (1.0f - 0.70f * s);
              const float rf = static_cast<float>(tint.r) * (1.0f - 0.70f * s);

              tint.b = ClampU8(static_cast<int>(std::round(bf)));
              tint.g = ClampU8(static_cast<int>(std::round(gf)));
              tint.r = ClampU8(static_cast<int>(std::round(rf)));
            }
          }
        }

        Texture2D& rtex = (t.terrain == Terrain::Water) ? bridge(mask) : road(mask);
        DrawTexturePro(rtex, src, dst, Vector2{0, 0}, 0.0f, tint);
      } else if (t.overlay != Overlay::None) {
        Color tint = BrightnessTint(brightness);

        if (showOutside) {
          const bool isZoneOrPark = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                                     t.overlay == Overlay::Industrial || t.overlay == Overlay::Park);

          if (isZoneOrPark && !HasAdjacentRoadConnectedToEdge(world, *roadToEdgeMask, x, y)) {
            // Dim zones/parks that are not adjacent to an outside-connected road.
            tint = Mul(tint, 0.55f);
          }
        }


        if (showCommercialGoods && t.overlay == Overlay::Commercial) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < commercialGoodsFill->size()) {
            const float ratio = static_cast<float>((*commercialGoodsFill)[idx]) / 255.0f;
            const float miss = std::clamp(1.0f - ratio, 0.0f, 1.0f);
            if (miss > 0.01f) {
              const float rf = std::min(255.0f, static_cast<float>(tint.r) * (1.0f + 0.10f * miss) + 70.0f * miss);
              const float gf = static_cast<float>(tint.g) * (1.0f - 0.55f * miss);
              const float bf = static_cast<float>(tint.b) * (1.0f - 0.55f * miss);
              tint.r = ClampU8(static_cast<int>(std::round(rf)));
              tint.g = ClampU8(static_cast<int>(std::round(gf)));
              tint.b = ClampU8(static_cast<int>(std::round(bf)));
            }
          }
        }

        DrawTexturePro(overlay(t.overlay), src, dst, Vector2{0, 0}, 0.0f, tint);
      }

      // District overlay (fill). District 0 is treated as "unassigned" and is left transparent.
      if (showDistrictOverlay) {
        const std::uint8_t d = static_cast<std::uint8_t>(t.district) & 7u;
        if (d != 0u) {
          unsigned char alpha = 65;
          if (highlightDistrict >= 0) {
            alpha = (d == static_cast<std::uint8_t>(highlightDistrict & 7)) ? 95 : 22;
          }
          if (alpha > 0) {
            const Color c = DistrictFillColor(d, alpha);
            Vector2 corners[4];
            TileDiamondCorners(center, tileW, tileH, corners);
            DrawTriangle(corners[0], corners[1], corners[2], c);
            DrawTriangle(corners[0], corners[2], corners[3], c);
          }
        }
      }

      // Heatmap overlay (drawn after tile overlays so it can tint zones/roads).
      if (showHeatmap) {
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                static_cast<std::size_t>(x);
        if (idx < heatmap->size()) {
          const float hv = (*heatmap)[idx];
          const Color c = HeatmapColor(hv, heatmapRamp);
          Vector2 corners[4];
          TileDiamondCorners(center, tileW, tileH, corners);
          DrawTriangle(corners[0], corners[1], corners[2], c);
          DrawTriangle(corners[0], corners[2], corners[3], c);
        }
      }

      if (drawGrid) {
        Vector2 corners[4];
        TileDiamondCorners(center, static_cast<float>(m_tileW), static_cast<float>(m_tileH), corners);
        const Color gc = Color{0, 0, 0, 40};
        DrawLineV(corners[0], corners[1], gc);
        DrawLineV(corners[1], corners[2], gc);
        DrawLineV(corners[2], corners[3], gc);
        DrawLineV(corners[3], corners[0], gc);
      }

      // District borders (drawn after grid/heatmap so boundaries stay readable).
      if (showDistrictBorders) {
        Vector2 corners[4];
        TileDiamondCorners(center, tileW, tileH, corners);
        const Color border = Color{0, 0, 0, 170};
        const float thick = 2.0f / std::max(0.001f, camera.zoom);
        const std::uint8_t d = static_cast<std::uint8_t>(t.district) & 7u;

        if (x + 1 < w) {
          const std::uint8_t dn = static_cast<std::uint8_t>(world.at(x + 1, y).district) & 7u;
          if (dn != d) {
            DrawLineEx(corners[1], corners[2], thick, border);
          }
        }

        if (y + 1 < h) {
          const std::uint8_t dn = static_cast<std::uint8_t>(world.at(x, y + 1).district) & 7u;
          if (dn != d) {
            DrawLineEx(corners[2], corners[3], thick, border);
          }
        }
      }

      // Zone buildings + indicators.
      const float tileScreenW = tileW * camera.zoom;
      const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                           t.overlay == Overlay::Industrial);

      if (useMergedZoneBuildings && isZone &&
          idx < static_cast<int>(m_zoneParcelsScratch.tileToParcel.size())) {
        const int parcelAll = m_zoneParcelsScratch.tileToParcel[idx];
        const int parcelAnchor = m_zoneParcelsScratch.anchorToParcel[idx];

        if (parcelAll >= 0) {
          // Only draw once per parcel (on the anchor tile).
          if (parcelAnchor >= 0 &&
              parcelAnchor < static_cast<int>(m_zoneParcelsScratch.parcels.size())) {
            const ZoneBuildingParcel& p =
                m_zoneParcelsScratch.parcels[static_cast<std::size_t>(parcelAnchor)];

            if (p.isMultiTile()) {
              DrawMergedZoneBuildingAndIndicators(p, world, m_elev, tileW, tileH, camera.zoom, timeSec);
            } else {
              DrawZoneBuilding(t, tileW, tileH, camera.zoom, center, brightness);
            if (tileScreenW >= 28.0f) {
              const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
              const int lvl = std::clamp<int>(t.level, 1, 3);
              const int cap = CapacityForOverlayLevel(t.overlay, lvl);
              const int occ = std::clamp<int>(static_cast<int>(t.occupants), 0, cap);
              const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
              const float y0 = center.y - tileH * 0.18f;

              // Pips:
              const float pipR = 2.0f * invZoom;
              const float pipGap = 5.0f * invZoom;
              for (int i = 0; i < lvl; ++i) {
                const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
                DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
              }

              // Fill bar:
              const float barW = tileW * 0.42f * invZoom;
              const float barH = 3.0f * invZoom;
              const float barX = center.x - barW * 0.5f;
              const float barY = y0 + 5.0f * invZoom;
              DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
              DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
            }
            }
          }
        } else {
          // Defensive fallback: weird zone tile not parcelized (e.g., zone-on-water in tests).
          DrawZoneBuilding(t, tileW, tileH, camera.zoom, center, brightness);
            if (tileScreenW >= 28.0f) {
              const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
              const int lvl = std::clamp<int>(t.level, 1, 3);
              const int cap = CapacityForOverlayLevel(t.overlay, lvl);
              const int occ = std::clamp<int>(static_cast<int>(t.occupants), 0, cap);
              const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
              const float y0 = center.y - tileH * 0.18f;

              // Pips:
              const float pipR = 2.0f * invZoom;
              const float pipGap = 5.0f * invZoom;
              for (int i = 0; i < lvl; ++i) {
                const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
                DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
              }

              // Fill bar:
              const float barW = tileW * 0.42f * invZoom;
              const float barH = 3.0f * invZoom;
              const float barX = center.x - barW * 0.5f;
              const float barY = y0 + 5.0f * invZoom;
              DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
              DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
            }
        }
      } else {
        // Per-tile fallback.
        DrawZoneBuilding(t, tileW, tileH, camera.zoom, center, brightness);
        if (isZone) {
            if (tileScreenW >= 28.0f) {
              const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
              const int lvl = std::clamp<int>(t.level, 1, 3);
              const int cap = CapacityForOverlayLevel(t.overlay, lvl);
              const int occ = std::clamp<int>(static_cast<int>(t.occupants), 0, cap);
              const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
              const float y0 = center.y - tileH * 0.18f;

              // Pips:
              const float pipR = 2.0f * invZoom;
              const float pipGap = 5.0f * invZoom;
              for (int i = 0; i < lvl; ++i) {
                const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
                DrawCircleV(Vector2{px, y0}, pipR, Color{0, 0, 0, 100});
              }

              // Fill bar:
              const float barW = tileW * 0.42f * invZoom;
              const float barH = 3.0f * invZoom;
              const float barX = center.x - barW * 0.5f;
              const float barY = y0 + 5.0f * invZoom;
              DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
              DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
            }
        }
      }

// Road indicators: show small pips for upgraded road class (2..3) when zoomed in.
      if (t.overlay == Overlay::Road && tileScreenW >= 28.0f) {
        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        if (lvl > 1) {
          const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
          const float y0 = center.y - static_cast<float>(m_tileH) * 0.02f;

          const float pip = 4.0f * invZoom;
          const float gap = 1.5f * invZoom;
          const float groupW = pip * 3.0f + gap * 2.0f;
        const float groupX0 = center.x - groupW * 0.5f;

          for (int i = 0; i < 3; ++i) {
          Rectangle r{groupX0 + static_cast<float>(i) * (pip + gap), y0, pip, pip};
            DrawRectangleRec(r, Color{0, 0, 0, 110});
            DrawRectangleLinesEx(r, 1.0f * invZoom, Color{255, 255, 255, 55});
            if (i < lvl) {
              Rectangle f{r.x + 1.0f * invZoom, r.y + 1.0f * invZoom, r.width - 2.0f * invZoom,
                          r.height - 2.0f * invZoom};
              DrawRectangleRec(f, Color{255, 255, 255, 160});
            }
          }
        }
      }
    }
  }

  }

  // Highlight helpers.
  const float thickness = 2.0f / std::max(0.25f, camera.zoom);

  auto drawOutline = [&](int tx, int ty, Color c) {
    if (!world.inBounds(tx, ty)) return;
    const float tileW = static_cast<float>(m_tileW);
    const float tileH = static_cast<float>(m_tileH);
    Vector2 center = TileToWorldCenter(tx, ty, tileW, tileH);
    center.y -= TileElevationPx(world.at(tx, ty), m_elev);
    Vector2 corners[4];
    TileDiamondCorners(center, tileW, tileH, corners);

    DrawLineEx(corners[0], corners[1], thickness, c);
    DrawLineEx(corners[1], corners[2], thickness, c);
    DrawLineEx(corners[2], corners[3], thickness, c);
    DrawLineEx(corners[3], corners[0], thickness, c);
  };

  // Optional debug highlight: path to edge (inspect tool).
  if (highlightPath && !highlightPath->empty()) {
    const Color pc = Color{255, 215, 0, 110};
    for (const Point& p : *highlightPath) {
      drawOutline(p.x, p.y, pc);
    }
  }

  // Selected tile highlight.
  if (selected && world.inBounds(selected->x, selected->y)) {
    drawOutline(selected->x, selected->y, Color{255, 215, 0, 220});
  }

  // Hover highlight (and brush outline)
  if (hovered && world.inBounds(hovered->x, hovered->y)) {
    const int cx = hovered->x;
    const int cy = hovered->y;

    const int r = std::max(0, brushRadius);
    if (r > 0) {
      const Color bc = Color{255, 255, 255, 70};
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::abs(dx) + std::abs(dy) > r) continue; // diamond brush
          const int tx = cx + dx;
          const int ty = cy + dy;
          drawOutline(tx, ty, bc);
        }
      }
    }

    // Center tile gets a brighter outline.
    drawOutline(cx, cy, Color{255, 255, 255, 180});
  }

  EndMode2D();
}

void Renderer::drawHUD(const World& world, const Camera2D& camera, Tool tool, int roadBuildLevel,
                       std::optional<Point> hovered,
                       int screenW, int screenH, bool showHelp, int brushRadius, int undoCount, int redoCount,
                       bool simPaused, float simSpeed, int saveSlot, bool showMinimap, const char* inspectInfo,
                       const char* heatmapInfo)
{
  const Stats& s = world.stats();

  // HUD panel
  const int pad = 12;
  const int panelW = 420;
  // Budget + demand + land value add two always-on HUD lines.
  const int extraLines = 2 + ((inspectInfo && inspectInfo[0] != '\0') ? 1 : 0) +
                         ((heatmapInfo && heatmapInfo[0] != '\0') ? 1 : 0);
  const int panelH = (showHelp ? 360 : 228) + extraLines * 22;

  DrawRectangle(pad, pad, panelW, panelH, Color{0, 0, 0, 150});
  DrawRectangleLines(pad, pad, panelW, panelH, Color{255, 255, 255, 70});

  int y = pad + 10;

  auto line = [&](const std::string& text) {
    DrawText(text.c_str(), pad + 10, y, 18, RAYWHITE);
    y += 22;
  };

  char buf[256];

  std::snprintf(buf, sizeof(buf), "Day: %d    Money: %d    Happiness: %.0f%%", s.day, s.money,
                static_cast<double>(s.happiness * 100.0f));
  line(buf);

  std::snprintf(buf, sizeof(buf), "Sim: %s    Speed: x%.2f", simPaused ? "PAUSED" : "RUNNING", static_cast<double>(simSpeed));
  line(buf);

  {
    const int tradeNet = s.exportRevenue - s.importCost;
    const int net = s.income - s.expenses;
    std::snprintf(buf, sizeof(buf), "Budget: %+d  tax %d  maint %d  trade %+d  upg %d", net, s.taxRevenue, s.maintenanceCost,
                  tradeNet, s.upgradeCost);
    line(buf);
  }

  {
    std::snprintf(buf, sizeof(buf), "Demand: %.0f%%  Land: %.0f%%  Tax/cap: %.2f", static_cast<double>(s.demandResidential * 100.0f),
                  static_cast<double>(s.avgLandValue * 100.0f), static_cast<double>(s.avgTaxPerCapita));
    line(buf);
  }

  // JobsCapacity in the core sim counts *all* job tiles, but not all jobs are necessarily
  // reachable if road networks are disconnected (outside connection rule).
  if (s.jobsCapacityAccessible != s.jobsCapacity) {
    std::snprintf(buf, sizeof(buf), "Pop: %d / %d housing    Jobs: %d / %d access (total %d)", s.population,
                  s.housingCapacity, s.employed, s.jobsCapacityAccessible, s.jobsCapacity);
  } else {
    std::snprintf(buf, sizeof(buf), "Pop: %d / %d housing    Jobs: %d / %d cap", s.population, s.housingCapacity,
                  s.employed, s.jobsCapacity);
  }
  line(buf);

  if (s.commuters > 0) {
    if (s.commutersUnreachable > 0) {
      std::snprintf(buf, sizeof(buf),
                    "Traffic: %d commute (unreach %d)  avg %.1f (t %.1f)  cong %.0f%%",
                    s.commuters, s.commutersUnreachable, static_cast<double>(s.avgCommute),
                    static_cast<double>(s.avgCommuteTime),
                    static_cast<double>(s.trafficCongestion * 100.0f));
    } else {
      std::snprintf(buf, sizeof(buf), "Traffic: %d commute  avg %.1f (t %.1f)  cong %.0f%%", s.commuters,
                    static_cast<double>(s.avgCommute), static_cast<double>(s.avgCommuteTime),
                    static_cast<double>(s.trafficCongestion * 100.0f));
    }
    line(buf);
  } else {
    line("Traffic: (no commuters)");
  }


  if (s.goodsDemand > 0) {
    if (s.goodsUnreachableDemand > 0) {
      std::snprintf(buf, sizeof(buf), "Goods: prod %d  deliv %d/%d (%.0f%%)  unr %d  imp %d  exp %d",
                    s.goodsProduced, s.goodsDelivered, s.goodsDemand,
                    static_cast<double>(s.goodsSatisfaction * 100.0f), s.goodsUnreachableDemand,
                    s.goodsImported, s.goodsExported);
    } else {
      std::snprintf(buf, sizeof(buf), "Goods: prod %d  deliv %d/%d (%.0f%%)  imp %d  exp %d",
                    s.goodsProduced, s.goodsDelivered, s.goodsDemand,
                    static_cast<double>(s.goodsSatisfaction * 100.0f), s.goodsImported, s.goodsExported);
    }
    line(buf);
  } else {
    line("Goods: (no commercial demand)");
  }

  const char* toolName = ToString(tool);
  char toolBuf[64];
  if (tool == Tool::Road) {
    std::snprintf(toolBuf, sizeof(toolBuf), "Road (%s)", RoadClassName(roadBuildLevel));
    toolName = toolBuf;
  }
  std::snprintf(buf, sizeof(buf), "Roads: %d    Parks: %d    Tool: %s    Brush: %d", s.roads, s.parks, toolName,
                brushRadius);
  line(buf);

  if (m_useBandCache) {
    int dirtyBands = 0;
    for (const auto& b : m_bands) {
      if (b.dirty) ++dirtyBands;
    }
    std::snprintf(buf, sizeof(buf), "Undo: %d    Redo: %d    Slot: %d    Cache: ON (dirty %d)", undoCount, redoCount,
                  saveSlot, dirtyBands);
  } else {
    std::snprintf(buf, sizeof(buf), "Undo: %d    Redo: %d    Slot: %d    Cache: OFF", undoCount, redoCount, saveSlot);
  }
  line(buf);

  // Happiness bar
  const int barX = pad + 10;
  const int barY = y + 4;
  const int barW = panelW - 20;
  const int barH = 10;

  DrawRectangle(barX, barY, barW, barH, Color{255, 255, 255, 30});
  DrawRectangle(barX, barY, static_cast<int>(barW * std::clamp(s.happiness, 0.0f, 1.0f)), barH,
                Color{120, 220, 120, 140});
  DrawRectangleLines(barX, barY, barW, barH, Color{255, 255, 255, 50});
  y += 22;

  if (hovered && world.inBounds(hovered->x, hovered->y)) {
    const Tile& t = world.at(hovered->x, hovered->y);
    std::snprintf(buf, sizeof(buf), "Hover: (%d,%d)  %s + %s  dist=%d  h=%.2f  elev=%.0fpx  lvl=%d  occ=%d", hovered->x,
                  hovered->y, ToString(t.terrain), ToString(t.overlay), static_cast<int>(t.district & 7u),
                  static_cast<double>(t.height), static_cast<double>(TileElevationPx(t, m_elev)), t.level,
                  t.occupants);
    DrawText(buf, pad + 10, y + 6, 16, Color{220, 220, 220, 255});
    y += 26;
  }

  if (heatmapInfo && heatmapInfo[0] != '\0') {
    DrawText(heatmapInfo, pad + 10, y + 6, 16, Color{230, 230, 230, 255});
    y += 26;
  }

  if (inspectInfo && inspectInfo[0] != '\0') {
    DrawText(inspectInfo, pad + 10, y + 6, 16, Color{230, 230, 230, 255});
    y += 26;
  }

  if (showHelp) {
    DrawText("Right drag: pan | Wheel: zoom | R regen | G grid | H help | M minimap | E elev | O outside | L heatmap | C vehicles | P policy | F1 report | F2 cache | F3 model | F7 districts | T graph | V traffic | B goods", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("1 Road | 2 Res | 3 Com | 4 Ind | 5 Park | 0 Doze | 6 Raise | 7 Lower | 8 Smooth | 9 District | Q Inspect", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("[/] brush | ,/. district | Space: pause | N: step | +/-: speed | U: road type", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("F4 console | F5 save | F9 load | F6 slot | F10 saves | F12 shot | Ctrl+F12 map | Ctrl+Z undo | Ctrl+Y redo", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("F8 video | F11 fullscreen | Alt+Enter borderless | Ctrl+=/- UI scale | Ctrl+0 UI auto | Ctrl+Alt+=/- world scale", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("Tip: re-place a zone to upgrade. Road: U selects class (paint to upgrade), Shift+drag builds path. Terraform: Shift=strong, Ctrl=fine. District: Alt+click pick, Shift+click fill.", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
  }

  // Minimap overlay (bottom-right). One pixel per tile, scaled up.
  if (showMinimap) {
    ensureMinimapUpToDate(world);
    const MinimapLayout mini = minimapLayout(world, screenW, screenH);

    if (mini.rect.width > 2.0f && mini.rect.height > 2.0f && m_minimapTex.id != 0) {
      // Background + border.
      DrawRectangleRec(mini.rect, Color{0, 0, 0, 140});
      DrawRectangleLines(static_cast<int>(mini.rect.x), static_cast<int>(mini.rect.y), static_cast<int>(mini.rect.width),
                         static_cast<int>(mini.rect.height), Color{255, 255, 255, 70});

      // Draw the minimap texture scaled to the destination rectangle.
      const Rectangle src{0.0f, 0.0f, static_cast<float>(m_minimapW), static_cast<float>(m_minimapH)};
      DrawTexturePro(m_minimapTex, src, mini.rect, Vector2{0, 0}, 0.0f, WHITE);

      // Outline visible world viewport.
      TileRect vis = ComputeVisibleTileRect(camera, screenW, screenH, world.width(), world.height(),
                                                  static_cast<float>(m_tileW), static_cast<float>(m_tileH),
                                                  m_elev.maxPixels);
      const float pixelsPerTile = std::max(1.0e-3f, mini.pixelsPerTile);

      const float vx = mini.rect.x + static_cast<float>(vis.minX) * pixelsPerTile;
      const float vy = mini.rect.y + static_cast<float>(vis.minY) * pixelsPerTile;
      const float vw = static_cast<float>(vis.maxX - vis.minX + 1) * pixelsPerTile;
      const float vh = static_cast<float>(vis.maxY - vis.minY + 1) * pixelsPerTile;

      const int ivx = static_cast<int>(std::floor(vx));
      const int ivy = static_cast<int>(std::floor(vy));
      const int ivw = std::max(1, static_cast<int>(std::ceil(vw)));
      const int ivh = std::max(1, static_cast<int>(std::ceil(vh)));
      DrawRectangleLines(ivx, ivy, ivw, ivh, Color{255, 255, 255, 180});

      // Hovered tile marker.
      if (hovered && world.inBounds(hovered->x, hovered->y)) {
        const int hx = static_cast<int>(std::floor(mini.rect.x + static_cast<float>(hovered->x) * pixelsPerTile));
        const int hy = static_cast<int>(std::floor(mini.rect.y + static_cast<float>(hovered->y) * pixelsPerTile));
        const int hw = std::max(1, static_cast<int>(std::ceil(pixelsPerTile)));
        DrawRectangleLines(hx, hy, hw, hw, Color{255, 255, 0, 200});
      }

      // Camera target marker (approx tile under the camera target).
      if (const auto camTile = WorldToTileElevated(camera.target, world, static_cast<float>(m_tileW),
                                                   static_cast<float>(m_tileH), m_elev)) {
        const float cx = mini.rect.x + (static_cast<float>(camTile->x) + 0.5f) * pixelsPerTile;
        const float cy = mini.rect.y + (static_cast<float>(camTile->y) + 0.5f) * pixelsPerTile;
        const float r = std::clamp(1.0f + 0.35f * pixelsPerTile, 1.0f, 6.0f);
        DrawCircleV(Vector2{cx, cy}, r, Color{255, 255, 255, 190});
        DrawCircleLines(static_cast<int>(cx), static_cast<int>(cy), r + 1.0f, Color{0, 0, 0, 90});
      }

      // Label.
      const int labelY = std::max(0, static_cast<int>(mini.rect.y) - 18);
      DrawText("Minimap (click/drag)", static_cast<int>(mini.rect.x), labelY, 16, Color{230, 230, 230, 230});
    }
  }

  // FPS
  const int fps = GetFPS();
  std::snprintf(buf, sizeof(buf), "FPS: %d", fps);
  DrawText(buf, screenW - 90, 12, 20, Color{255, 255, 255, 200});
}

} // namespace isocity
