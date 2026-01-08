#include "isocity/GfxBuildings.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace isocity {

namespace {

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

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

inline std::uint8_t ClampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<std::uint8_t>(v);
}

inline Rgba8 Mul(Rgba8 c, float m)
{
  const int r = static_cast<int>(std::lround(static_cast<float>(c.r) * m));
  const int g = static_cast<int>(std::lround(static_cast<float>(c.g) * m));
  const int b = static_cast<int>(std::lround(static_cast<float>(c.b) * m));
  return Rgba8{ClampU8(r), ClampU8(g), ClampU8(b), c.a};
}

inline Rgba8 Add(Rgba8 c, int dr, int dg, int db)
{
  return Rgba8{ClampU8(static_cast<int>(c.r) + dr), ClampU8(static_cast<int>(c.g) + dg),
              ClampU8(static_cast<int>(c.b) + db), c.a};
}

inline void Clear(RgbaImage& img)
{
  std::fill(img.rgba.begin(), img.rgba.end(), std::uint8_t{0});
}

inline void BlendPixel(RgbaImage& img, int x, int y, Rgba8 src)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  if (src.a == 0) return;

  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;

  const std::uint8_t dr = img.rgba[i + 0];
  const std::uint8_t dg = img.rgba[i + 1];
  const std::uint8_t db = img.rgba[i + 2];
  const std::uint8_t da = img.rgba[i + 3];

  const int sa = static_cast<int>(src.a);
  const int ida = 255 - sa;
  const int outA = sa + (static_cast<int>(da) * ida + 127) / 255;

  if (outA <= 0) {
    img.rgba[i + 0] = 0;
    img.rgba[i + 1] = 0;
    img.rgba[i + 2] = 0;
    img.rgba[i + 3] = 0;
    return;
  }

  const int premR = static_cast<int>(src.r) * sa + (static_cast<int>(dr) * static_cast<int>(da) * ida + 127) / 255;
  const int premG = static_cast<int>(src.g) * sa + (static_cast<int>(dg) * static_cast<int>(da) * ida + 127) / 255;
  const int premB = static_cast<int>(src.b) * sa + (static_cast<int>(db) * static_cast<int>(da) * ida + 127) / 255;

  const int outR = (premR + outA / 2) / outA;
  const int outG = (premG + outA / 2) / outA;
  const int outB = (premB + outA / 2) / outA;

  img.rgba[i + 0] = ClampU8(outR);
  img.rgba[i + 1] = ClampU8(outG);
  img.rgba[i + 2] = ClampU8(outB);
  img.rgba[i + 3] = ClampU8(outA);
}

inline int EdgeFn(int ax, int ay, int bx, int by, int cx, int cy)
{
  return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
}

void FillTriangle(RgbaImage& img, int x0, int y0, int x1, int y1, int x2, int y2, Rgba8 c)
{
  int minX = std::min({x0, x1, x2});
  int maxX = std::max({x0, x1, x2});
  int minY = std::min({y0, y1, y2});
  int maxY = std::max({y0, y1, y2});

  minX = std::max(minX, 0);
  minY = std::max(minY, 0);
  maxX = std::min(maxX, img.width - 1);
  maxY = std::min(maxY, img.height - 1);

  const int area = EdgeFn(x0, y0, x1, y1, x2, y2);
  if (area == 0) return;
  const bool ccw = (area > 0);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const int w0 = EdgeFn(x1, y1, x2, y2, x, y);
      const int w1 = EdgeFn(x2, y2, x0, y0, x, y);
      const int w2 = EdgeFn(x0, y0, x1, y1, x, y);

      if (ccw) {
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
      } else {
        if (w0 > 0 || w1 > 0 || w2 > 0) continue;
      }
      BlendPixel(img, x, y, c);
    }
  }
}

// Barycentric coordinates for integer pixels, returned as floats.
// Returns false if degenerate.
bool Barycentric(int x, int y, int x0, int y0, int x1, int y1, int x2, int y2, float& w0, float& w1, float& w2)
{
  const float den = static_cast<float>((y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2));
  if (std::fabs(den) < 1.0e-6f) return false;
  const float inv = 1.0f / den;
  w0 = static_cast<float>((y1 - y2) * (x - x2) + (x2 - x1) * (y - y2)) * inv;
  w1 = static_cast<float>((y2 - y0) * (x - x2) + (x0 - x2) * (y - y2)) * inv;
  w2 = 1.0f - w0 - w1;
  return true;
}

// Draw a simple axis-aligned rectangle with alpha blend.
void FillRect(RgbaImage& img, int x0, int y0, int x1, int y1, Rgba8 c)
{
  const int minX = std::max(0, std::min(x0, x1));
  const int maxX = std::min(img.width - 1, std::max(x0, x1));
  const int minY = std::max(0, std::min(y0, y1));
  const int maxY = std::min(img.height - 1, std::max(y0, y1));
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      BlendPixel(img, x, y, c);
    }
  }
}

struct FaceQuad {
  // Screen-space vertices (clockwise): a,b,c,d
  int ax = 0, ay = 0;
  int bx = 0, by = 0;
  int cx = 0, cy = 0;
  int dx = 0, dy = 0;
};

// Sample whether a pixel is inside a face and compute local coords (s,t)
// where s is along the base edge (0..1) and t is vertical (0..1).
// The face is assumed to be a parallelogram produced by extruding the base edge.
bool FaceLocalST(const FaceQuad& q, int px, int py, float& outS, float& outT)
{
  // Split into two triangles: (a,b,c) and (a,c,d)
  float w0, w1, w2;
  if (Barycentric(px, py, q.ax, q.ay, q.bx, q.by, q.cx, q.cy, w0, w1, w2)) {
    if (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f) {
      // a=(0,0), b=(1,0), c=(1,1)
      outS = w1 + w2;
      outT = w2;
      return true;
    }
  }
  if (Barycentric(px, py, q.ax, q.ay, q.cx, q.cy, q.dx, q.dy, w0, w1, w2)) {
    if (w0 >= 0.0f && w1 >= 0.0f && w2 >= 0.0f) {
      // a=(0,0), c=(1,1), d=(0,1)
      outS = w1;
      outT = w1 + w2;
      return true;
    }
  }
  return false;
}

Rgba8 RoofBaseColor(GfxBuildingKind kind, const GfxPalette& pal)
{
  switch (kind) {
  case GfxBuildingKind::Residential: return pal.overlayResidential;
  case GfxBuildingKind::Commercial: return pal.overlayCommercial;
  case GfxBuildingKind::Industrial: return pal.overlayIndustrial;
  default: break;
  }
  return Rgba8{200, 200, 200, 255};
}

Rgba8 WindowTint(GfxBuildingKind kind)
{
  // Daytime window glass tint.
  switch (kind) {
  case GfxBuildingKind::Commercial: return Rgba8{170, 210, 255, 220};
  case GfxBuildingKind::Industrial: return Rgba8{210, 220, 235, 200};
  case GfxBuildingKind::Residential: return Rgba8{200, 220, 240, 210};
  default: break;
  }
  return Rgba8{200, 220, 240, 210};
}

Rgba8 WindowLit(GfxBuildingKind kind, const GfxPalette& pal)
{
  // Night emissive window color.
  // Use the palette's yellow marking as a theme-aware warm light.
  Rgba8 c = pal.roadMarkYellow;
  if (kind == GfxBuildingKind::Commercial) {
    c = Add(c, 10, 0, -10);
  } else if (kind == GfxBuildingKind::Industrial) {
    c = Add(c, -15, -10, 10);
  }
  c.a = 220;
  return c;
}

} // namespace

bool GenerateGfxBuildingSprite(GfxBuildingKind kind, int level, int variant, std::uint32_t seed,
                               const GfxBuildingsConfig& cfgIn, const GfxPalette& pal, GfxBuildingSprite& out,
                               std::string& outError)
{
  outError.clear();
  out = GfxBuildingSprite{};

  if (cfgIn.tileW <= 0 || cfgIn.tileH <= 0) {
    outError = "invalid tile size";
    return false;
  }

  const int lvl = std::clamp(level, 1, 3);

  const int tileW = cfgIn.tileW;
  const int tileH = cfgIn.tileH;
  const int halfW = tileW / 2;
  const int halfH = tileH / 2;

  const int maxHeightPx = (cfgIn.maxHeightPx > 0) ? cfgIn.maxHeightPx : static_cast<int>(std::lround(tileH * 4.0f));
  const int marginTop = 4;
  const int marginBot = 3;
  const int spriteH = (cfgIn.spriteH > 0) ? cfgIn.spriteH : (tileH + maxHeightPx + marginTop + marginBot);

  RgbaImage img;
  img.width = tileW;
  img.height = spriteH;
  img.rgba.assign(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u, 0u);

  RgbaImage emit;
  if (cfgIn.includeEmissive) {
    emit.width = img.width;
    emit.height = img.height;
    emit.rgba.assign(static_cast<std::size_t>(emit.width) * static_cast<std::size_t>(emit.height) * 4u, 0u);
  }

  // Pivot aligns with the ground tile center.
  const int pivotX = halfW;
  const int pivotY = spriteH - marginBot - halfH;
  out.pivotX = pivotX;
  out.pivotY = pivotY;

  // Deterministic per-building RNG.
  const std::uint32_t seedv = seed ^ 0xC001D00Du ^ (static_cast<std::uint32_t>(variant) * 0x9E3779B9u) ^
                              (static_cast<std::uint32_t>(lvl) * 0x85EBCA6Bu) ^
                              (static_cast<std::uint32_t>(kind) * 0xC2B2AE35u);

  auto rand01 = [&](std::uint32_t salt) {
    return Frac01(HashU32(seedv ^ salt));
  };

  // Footprint + height heuristics (loosely matching the in-app extruded buildings).
  float baseShrink = 0.58f;
  float heightMul = 1.00f;

  if (kind == GfxBuildingKind::Residential) {
    baseShrink = 0.60f;
    heightMul = 1.00f;
  } else if (kind == GfxBuildingKind::Commercial) {
    baseShrink = 0.52f;
    heightMul = 1.45f;
  } else if (kind == GfxBuildingKind::Industrial) {
    baseShrink = 0.64f;
    heightMul = 0.92f;
  }

  // Level influence.
  baseShrink *= 1.00f - 0.04f * static_cast<float>(lvl - 1);
  baseShrink = std::clamp(baseShrink + (rand01(0x11u) - 0.5f) * 0.06f, 0.42f, 0.72f);

  float heightPx = static_cast<float>(tileH) * (0.70f + 0.55f * static_cast<float>(lvl));
  heightPx *= heightMul;
  heightPx *= 0.82f + 0.40f * rand01(0x22u);

  // Big commercial buildings get extra variance.
  if (kind == GfxBuildingKind::Commercial && lvl == 3) {
    heightPx *= 1.05f + 0.18f * rand01(0x23u);
  }

  heightPx = std::clamp(heightPx, static_cast<float>(tileH) * 0.65f, static_cast<float>(maxHeightPx));
  const int hPx = static_cast<int>(std::lround(heightPx));

  const float hw = static_cast<float>(tileW) * 0.5f * baseShrink;
  const float hh = static_cast<float>(tileH) * 0.5f * baseShrink;

  auto ix = [&](float x) { return static_cast<int>(std::lround(x)); };
  auto iy = [&](float y) { return static_cast<int>(std::lround(y)); };

  // Base diamond corners (top, right, bottom, left).
  const int bx0 = pivotX;
  const int by0 = pivotY - iy(hh);
  const int bx1 = pivotX + ix(hw);
  const int by1 = pivotY;
  const int bx2 = pivotX;
  const int by2 = pivotY + iy(hh);
  const int bx3 = pivotX - ix(hw);
  const int by3 = pivotY;

  // Top diamond corners.
  const int tx0 = bx0;
  const int ty0 = by0 - hPx;
  const int tx1 = bx1;
  const int ty1 = by1 - hPx;
  const int tx2 = bx2;
  const int ty2 = by2 - hPx;
  const int tx3 = bx3;
  const int ty3 = by3 - hPx;

  // Contact shadow (very subtle).
  {
    const float shadowShrink = std::min(0.98f, baseShrink * 1.12f);
    const float shw = static_cast<float>(tileW) * 0.5f * shadowShrink;
    const float shh = static_cast<float>(tileH) * 0.5f * shadowShrink;
    const int sx0 = pivotX;
    const int sy0 = pivotY - iy(shh);
    const int sx1 = pivotX + ix(shw);
    const int sy1 = pivotY;
    const int sx2 = pivotX;
    const int sy2 = pivotY + iy(shh);
    const int sx3 = pivotX - ix(shw);
    const int sy3 = pivotY;

    Rgba8 sc{0, 0, 0, static_cast<std::uint8_t>(18 + lvl * 6)};
    FillTriangle(img, sx0, sy0, sx1, sy1, sx2, sy2, sc);
    FillTriangle(img, sx0, sy0, sx2, sy2, sx3, sy3, sc);
  }

  // Base materials.
  const Rgba8 baseC = RoofBaseColor(kind, pal);
  const float roofBright = 1.12f + (rand01(0x33u) - 0.5f) * 0.08f;
  const float rightMul = 0.86f;
  const float leftMul = 0.70f;

  const Rgba8 roofC = Mul(baseC, roofBright);
  const Rgba8 rightC = Mul(baseC, rightMul);
  const Rgba8 leftC = Mul(baseC, leftMul);

  // Walls (two visible faces).
  // Right face quad (right->bottom->topBottom->topRight)
  FaceQuad rightQ;
  rightQ.ax = bx1; rightQ.ay = by1;
  rightQ.bx = bx2; rightQ.by = by2;
  rightQ.cx = tx2; rightQ.cy = ty2;
  rightQ.dx = tx1; rightQ.dy = ty1;

  FillTriangle(img, rightQ.ax, rightQ.ay, rightQ.bx, rightQ.by, rightQ.cx, rightQ.cy, rightC);
  FillTriangle(img, rightQ.ax, rightQ.ay, rightQ.cx, rightQ.cy, rightQ.dx, rightQ.dy, rightC);

  // Left face quad (left->bottom->topBottom->topLeft)
  FaceQuad leftQ;
  leftQ.ax = bx3; leftQ.ay = by3;
  leftQ.bx = bx2; leftQ.by = by2;
  leftQ.cx = tx2; leftQ.cy = ty2;
  leftQ.dx = tx3; leftQ.dy = ty3;

  FillTriangle(img, leftQ.ax, leftQ.ay, leftQ.bx, leftQ.by, leftQ.cx, leftQ.cy, leftC);
  FillTriangle(img, leftQ.ax, leftQ.ay, leftQ.cx, leftQ.cy, leftQ.dx, leftQ.dy, leftC);

  // Roof (top diamond).
  // Add slight orientation shading so the roof doesn't look flat.
  {
    const Rgba8 roofA = Mul(roofC, 1.05f);
    const Rgba8 roofB = Mul(roofC, 0.92f);

    // Residential: a gable ridge.
    if (kind == GfxBuildingKind::Residential) {
      FillTriangle(img, tx0, ty0, tx1, ty1, tx2, ty2, roofA);
      FillTriangle(img, tx0, ty0, tx2, ty2, tx3, ty3, roofB);
    } else {
      FillTriangle(img, tx0, ty0, tx1, ty1, tx2, ty2, roofC);
      FillTriangle(img, tx0, ty0, tx2, ty2, tx3, ty3, roofC);
    }

    // Roof edge line.
    const Rgba8 edge = Mul(baseC, 0.55f);
    const int edgeA = 210;
    Rgba8 e = edge; e.a = static_cast<std::uint8_t>(edgeA);
    FillRect(img, tx0 - 1, ty0 - 1, tx0 + 1, ty0 + 1, e);
  }

  // Window patterns (drawn as a blended overlay on top of the wall faces).
  {
    const Rgba8 glass = WindowTint(kind);
    const Rgba8 lit = WindowLit(kind, pal);

    // Grid sizes.
    int cols = 3 + lvl;
    int rows = 2 + lvl;
    if (kind == GfxBuildingKind::Commercial) {
      cols = 4 + lvl * 2;
      rows = 3 + lvl * 2;
    } else if (kind == GfxBuildingKind::Industrial) {
      cols = 2 + lvl;
      rows = 2 + lvl;
    }

    auto paintFace = [&](const FaceQuad& q, float faceShade, std::uint32_t saltBase) {
      // Iterate over a conservative bounding box.
      const int minX = std::max(0, std::min({q.ax, q.bx, q.cx, q.dx}));
      const int maxX = std::min(img.width - 1, std::max({q.ax, q.bx, q.cx, q.dx}));
      const int minY = std::max(0, std::min({q.ay, q.by, q.cy, q.dy}));
      const int maxY = std::min(img.height - 1, std::max({q.ay, q.by, q.cy, q.dy}));

      for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
          float s = 0.0f;
          float t = 0.0f;
          if (!FaceLocalST(q, x, y, s, t)) continue;
          if (t < 0.08f || t > 0.96f) continue;

          // Window cell indices.
          const float u = s * static_cast<float>(cols);
          const float v = t * static_cast<float>(rows);
          const int ci = static_cast<int>(std::floor(u));
          const int ri = static_cast<int>(std::floor(v));
          if (ci < 0 || ri < 0 || ci >= cols || ri >= rows) continue;

          const float fu = u - static_cast<float>(ci);
          const float fv = v - static_cast<float>(ri);

          // Padding between windows.
          const float padU = 0.18f;
          const float padV = 0.22f;
          if (fu < padU || fu > (1.0f - padU) || fv < padV || fv > (1.0f - padV)) continue;

          // Industrial: fewer windows (skip many).
          if (kind == GfxBuildingKind::Industrial) {
            const std::uint32_t hskip = HashCoords32(ci, ri, seedv ^ (saltBase + 0x1234u));
            if ((hskip & 3u) != 0u) continue;
          }

          // Slight per-window tint.
          const std::uint32_t hw = HashCoords32(ci, ri, seedv ^ saltBase);
          const float n = (Frac01(hw) - 0.5f) * 0.10f;
          Rgba8 wc = Mul(glass, std::clamp(1.0f + n, 0.82f, 1.18f));
          wc = Mul(wc, faceShade);
          BlendPixel(img, x, y, wc);

          if (cfgIn.includeEmissive) {
            // Night-time lit probability depends on type.
            float pLit = 0.35f;
            if (kind == GfxBuildingKind::Residential) pLit = 0.28f;
            if (kind == GfxBuildingKind::Commercial) pLit = 0.55f;
            if (kind == GfxBuildingKind::Industrial) pLit = 0.18f;
            // Taller buildings have a bit more activity.
            pLit += 0.04f * static_cast<float>(lvl - 1);

            if (Frac01(hw ^ 0xDEADBEEFu) < pLit) {
              // Emissive pixel: use the same window mask.
              Rgba8 ec = lit;
              ec.a = static_cast<std::uint8_t>(170 + (hw & 0x3Fu));
              BlendPixel(emit, x, y, ec);
            }
          }
        }
      }
    };

    paintFace(rightQ, 1.0f, 0x900Du);
    paintFace(leftQ, 0.92f, 0xBEEFu);

    // Commercial signage on the roof edge.
    if (cfgIn.includeEmissive && kind == GfxBuildingKind::Commercial && lvl >= 2) {
      const int sx = pivotX - ix(hw) + 2;
      const int ex = pivotX + ix(hw) - 2;
      const int sy = ty0 + 3;
      const int ey = ty0 + 6;
      const std::uint32_t hs = HashU32(seedv ^ 0x51A7E1u);
      const int hue = static_cast<int>(hs & 3u);
      Rgba8 sc = lit;
      if (hue == 1) sc = Rgba8{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 220};
      if (hue == 2) sc = Rgba8{static_cast<std::uint8_t>(200), static_cast<std::uint8_t>(100), static_cast<std::uint8_t>(255), 220};
      if (hue == 3) sc = Rgba8{static_cast<std::uint8_t>(90), static_cast<std::uint8_t>(220), static_cast<std::uint8_t>(255), 220};
      FillRect(emit, sx, sy, ex, ey, sc);
    }
  }

  out.color = std::move(img);
  out.emissive = std::move(emit);
  return true;
}

} // namespace isocity
