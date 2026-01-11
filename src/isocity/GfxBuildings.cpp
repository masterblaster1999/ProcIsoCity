#include "isocity/GfxBuildings.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/GfxText.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

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

using gfx::BlendMode;
using gfx::BlendPixel;
using gfx::ClampU8;
using gfx::FillRect;
using gfx::FillTriangle;
using gfx::Mix;
using gfx::Mul;
using gfx::Add;
using gfx::StrokeLine;

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


inline bool PointInTri(int px, int py, int x0, int y0, int x1, int y1, int x2, int y2)
{
  float w0 = 0.0f, w1 = 0.0f, w2 = 0.0f;
  if (!Barycentric(px, py, x0, y0, x1, y1, x2, y2, w0, w1, w2)) return false;
  // A tiny epsilon prevents cracks due to rounding.
  constexpr float eps = -1.0e-4f;
  return (w0 >= eps) && (w1 >= eps) && (w2 >= eps);
}

inline bool PointInDiamond(int px, int py, int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3)
{
  return PointInTri(px, py, x0, y0, x1, y1, x2, y2) || PointInTri(px, py, x0, y0, x2, y2, x3, y3);
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

  // Slightly larger automatic canvas for tall lvl3 commercial silhouettes (setbacks / signage).
  int autoMaxHeight = static_cast<int>(std::lround(tileH * 4.0f));
  if (kind == GfxBuildingKind::Commercial && lvl == 3) autoMaxHeight = static_cast<int>(std::lround(tileH * 4.6f));
  if (kind == GfxBuildingKind::Industrial && lvl == 3) autoMaxHeight = static_cast<int>(std::lround(tileH * 4.25f));

  const int maxHeightPx = (cfgIn.maxHeightPx > 0) ? cfgIn.maxHeightPx : autoMaxHeight;
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

  auto ix = [&](float x) { return static_cast<int>(std::lround(x)); };
  auto iy = [&](float y) { return static_cast<int>(std::lround(y)); };

  // -----------------------------
  // Materials
  // -----------------------------
  // Roof tint is keyed to the zone color, but walls are mixed with a neutral so residential
  // doesn't become a solid red box.
  const Rgba8 roofTint = RoofBaseColor(kind, pal);

  Rgba8 wallNeutral{210, 210, 210, 255};
  float wallMix = 0.52f;
  if (kind == GfxBuildingKind::Residential) {
    wallNeutral = Rgba8{222, 214, 200, 255};
    wallMix = 0.42f;
  } else if (kind == GfxBuildingKind::Commercial) {
    wallNeutral = Rgba8{200, 210, 220, 255};
    wallMix = 0.62f;
  } else if (kind == GfxBuildingKind::Industrial) {
    wallNeutral = Rgba8{206, 206, 200, 255};
    wallMix = 0.55f;
  }

  const Rgba8 wallBase = Mix(wallNeutral, roofTint, wallMix);
  const Rgba8 glassTint = WindowTint(kind);
  const Rgba8 litTint = WindowLit(kind, pal);

  // -----------------------------
  // Footprint + height heuristics (loosely matching the in-app extruded buildings).
  // -----------------------------
  float baseShrink = 0.58f;
  float heightMul = 1.00f;

  if (kind == GfxBuildingKind::Residential) {
    baseShrink = 0.60f;
    heightMul = 1.00f;
  } else if (kind == GfxBuildingKind::Commercial) {
    baseShrink = 0.53f;
    heightMul = 1.52f;
  } else if (kind == GfxBuildingKind::Industrial) {
    baseShrink = 0.66f;
    heightMul = 0.95f;
  }

  // Level influence.
  baseShrink *= 1.00f - 0.04f * static_cast<float>(lvl - 1);
  baseShrink = std::clamp(baseShrink + (rand01(0x11u) - 0.5f) * 0.06f, 0.42f, 0.74f);

  float heightPx = static_cast<float>(tileH) * (0.72f + 0.58f * static_cast<float>(lvl));
  heightPx *= heightMul;
  heightPx *= 0.82f + 0.42f * rand01(0x22u);

  // Big commercial buildings get extra variance.
  if (kind == GfxBuildingKind::Commercial && lvl == 3) {
    heightPx *= 1.05f + 0.22f * rand01(0x23u);
  }

  heightPx = std::clamp(heightPx, static_cast<float>(tileH) * 0.65f, static_cast<float>(maxHeightPx));
  const int totalHPx = static_cast<int>(std::lround(heightPx));

  // -----------------------------
  // Tiered silhouettes (setbacks)
  // -----------------------------
  struct TierDesc {
    float shrink = 0.60f;
    int heightPx = 0;
    bool windows = true;
  };

  int tierCount = 1;
  std::vector<float> fracs;

  if (kind == GfxBuildingKind::Commercial) {
    tierCount = (lvl == 1) ? 1 : (lvl == 2 ? 2 : 3);
    fracs = (tierCount == 1) ? std::vector<float>{1.0f} : (tierCount == 2 ? std::vector<float>{0.42f, 0.58f}
                                                                 : std::vector<float>{0.36f, 0.34f, 0.30f});
  } else if (kind == GfxBuildingKind::Residential) {
    tierCount = (lvl == 3) ? 2 : 1;
    fracs = (tierCount == 1) ? std::vector<float>{1.0f} : std::vector<float>{0.72f, 0.28f};
  } else {
    // Industrial
    tierCount = (lvl >= 2) ? 2 : 1;
    fracs = (tierCount == 1) ? std::vector<float>{1.0f} : std::vector<float>{0.67f, 0.33f};
  }

  std::vector<TierDesc> tiers;
  tiers.resize(static_cast<std::size_t>(tierCount));

  const int minTierH = std::max(8, tileH / 3);
  int remainingH = totalHPx;
  for (int i = 0; i < tierCount; ++i) {
    const bool last = (i == tierCount - 1);
    int h = last ? remainingH : static_cast<int>(std::lround(static_cast<float>(totalHPx) * fracs[static_cast<std::size_t>(i)]));
    const int minRemain = minTierH * (tierCount - i - 1);
    h = std::clamp(h, minTierH, std::max(minTierH, remainingH - minRemain));
    remainingH -= h;
    tiers[static_cast<std::size_t>(i)].heightPx = h;
  }
  // Ensure the last tier consumes any rounding residue.
  if (!tiers.empty()) {
    tiers.back().heightPx = std::max(minTierH, tiers.back().heightPx + remainingH);
  }

  for (int i = 0; i < tierCount; ++i) {
    float k = 1.0f;
    if (kind == GfxBuildingKind::Commercial) {
      k = 1.0f - 0.18f * static_cast<float>(i);
      k += (rand01(0x400u + static_cast<std::uint32_t>(i) * 31u) - 0.5f) * 0.06f;
    } else if (kind == GfxBuildingKind::Industrial) {
      k = 1.0f - 0.14f * static_cast<float>(i);
      k += (rand01(0x500u + static_cast<std::uint32_t>(i) * 29u) - 0.5f) * 0.05f;
    } else {
      k = 1.0f - 0.10f * static_cast<float>(i);
      k += (rand01(0x600u + static_cast<std::uint32_t>(i) * 23u) - 0.5f) * 0.04f;
    }

    tiers[static_cast<std::size_t>(i)].shrink = std::clamp(baseShrink * k, 0.34f, 0.75f);
    tiers[static_cast<std::size_t>(i)].windows = true;
  }

  // Industrial base halls tend to have fewer windows.
  if (kind == GfxBuildingKind::Industrial && tierCount >= 1) {
    tiers[0].windows = (lvl >= 3); // mostly office-like at high level
  }

  // -----------------------------
  // Contact shadow
  // -----------------------------
  {
    const float shBase = !tiers.empty() ? tiers[0].shrink : baseShrink;
    const float shadowShrink = std::min(0.98f, shBase * 1.12f);
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

  struct TierGeom {
    int bx[4]{};
    int by[4]{};
    int tx[4]{};
    int ty[4]{};
    FaceQuad rightQ{};
    FaceQuad leftQ{};
  };

  auto buildGeom = [&](int baseY, float shrink, int hPx) -> TierGeom {
    TierGeom g;

    const float hw = static_cast<float>(tileW) * 0.5f * shrink;
    const float hh = static_cast<float>(tileH) * 0.5f * shrink;

    // Base diamond corners (top, right, bottom, left).
    g.bx[0] = pivotX;
    g.by[0] = baseY - iy(hh);
    g.bx[1] = pivotX + ix(hw);
    g.by[1] = baseY;
    g.bx[2] = pivotX;
    g.by[2] = baseY + iy(hh);
    g.bx[3] = pivotX - ix(hw);
    g.by[3] = baseY;

    // Top diamond corners.
    for (int i = 0; i < 4; ++i) {
      g.tx[i] = g.bx[i];
      g.ty[i] = g.by[i] - hPx;
    }

    // Right face quad (right->bottom->topBottom->topRight)
    g.rightQ.ax = g.bx[1];
    g.rightQ.ay = g.by[1];
    g.rightQ.bx = g.bx[2];
    g.rightQ.by = g.by[2];
    g.rightQ.cx = g.tx[2];
    g.rightQ.cy = g.ty[2];
    g.rightQ.dx = g.tx[1];
    g.rightQ.dy = g.ty[1];

    // Left face quad (left->bottom->topBottom->topLeft)
    g.leftQ.ax = g.bx[3];
    g.leftQ.ay = g.by[3];
    g.leftQ.bx = g.bx[2];
    g.leftQ.by = g.by[2];
    g.leftQ.cx = g.tx[2];
    g.leftQ.cy = g.ty[2];
    g.leftQ.dx = g.tx[3];
    g.leftQ.dy = g.ty[3];

    return g;
  };

  auto paintWindows = [&](const FaceQuad& q, int tierIdx, float tierShrink, int tierHeightPx, float faceShade,
                          std::uint32_t saltBase, bool enabled) {
    if (!enabled) return;

    // Grid sizes are derived from building kind/level, then scaled down for higher tiers.
    const float baseShrinkRef = std::max(0.001f, tiers.empty() ? tierShrink : tiers[0].shrink);
    const float widthFrac = std::clamp(tierShrink / baseShrinkRef, 0.45f, 1.0f);

    int baseCols = 3 + lvl;
    int baseRows = 2 + lvl;
    float padU = 0.18f;
    float padV = 0.22f;

    if (kind == GfxBuildingKind::Commercial) {
      baseCols = 6 + lvl * 3;
      baseRows = 4 + lvl * 3;
      padU = 0.12f;
      padV = 0.14f;
    } else if (kind == GfxBuildingKind::Industrial) {
      baseCols = 2 + lvl;
      baseRows = 2 + lvl;
      padU = 0.22f;
      padV = 0.26f;
    } else {
      // Residential: a little chunkier.
      padU = 0.20f;
      padV = 0.24f;
    }

    int cols = std::max(2, static_cast<int>(std::lround(static_cast<float>(baseCols) * widthFrac)));
    const float hFrac = std::clamp(static_cast<float>(tierHeightPx) / std::max(1.0f, static_cast<float>(totalHPx)), 0.20f, 1.0f);
    int rows = std::max(2, static_cast<int>(std::lround(static_cast<float>(baseRows) * (0.75f + 0.25f * hFrac))));

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

        const float u = s * static_cast<float>(cols);
        const float v = t * static_cast<float>(rows);
        const int ci = static_cast<int>(std::floor(u));
        const int ri = static_cast<int>(std::floor(v));
        if (ci < 0 || ri < 0 || ci >= cols || ri >= rows) continue;

        const float fu = u - static_cast<float>(ci);
        const float fv = v - static_cast<float>(ri);

        if (fu < padU || fu > (1.0f - padU) || fv < padV || fv > (1.0f - padV)) continue;

        const std::uint32_t hw = HashCoords32(ci + tierIdx * 31, ri + tierIdx * 17, seedv ^ saltBase);
        const float n = (Frac01(hw) - 0.5f) * 0.10f;

        // Industrial: fewer windows.
        if (kind == GfxBuildingKind::Industrial) {
          if ((hw & 3u) != 0u) continue;
        }

        // Commercial: occasional dark floors.
        float floorMul = 1.0f;
        if (kind == GfxBuildingKind::Commercial) {
          const int floor = ri;
          if (((floor + tierIdx) & 3) == 0) floorMul = 0.86f;
        }

        Rgba8 wc = Mul(glassTint, std::clamp(1.0f + n, 0.82f, 1.18f));
        wc = Mul(wc, faceShade * floorMul);
        BlendPixel(img, x, y, wc);

        if (cfgIn.includeEmissive && !emit.rgba.empty()) {
          float pLit = 0.35f;
          if (kind == GfxBuildingKind::Residential) pLit = 0.28f;
          if (kind == GfxBuildingKind::Commercial) pLit = 0.58f;
          if (kind == GfxBuildingKind::Industrial) pLit = 0.18f;
          pLit += 0.04f * static_cast<float>(lvl - 1);
          pLit += 0.04f * static_cast<float>(tierIdx);

          if (Frac01(hw ^ 0xDEADBEEFu) < pLit) {
            Rgba8 ec = litTint;
            ec.a = static_cast<std::uint8_t>(170 + (hw & 0x3Fu));
            BlendPixel(emit, x, y, ec, BlendMode::Additive);
          }
        }
      }
    }
  };

  auto drawStack = [&](int baseX, int baseY, int w, int h, Rgba8 body, bool emissiveTop, Rgba8 emitCol) {
    if (w <= 0 || h <= 0) return;
    const int x0 = baseX - w / 2;
    const int x1 = baseX + (w - 1) / 2;
    const int y0 = baseY - h;
    const int y1 = baseY;

    // Simple two-tone shading.
    FillRect(img, x0, y0, x1, y1, body);
    FillRect(img, x0, y0, x0 + w / 3, y1, Mul(body, 0.82f));
    FillRect(img, x0, y0, x1, y0 + 2, Mul(body, 1.12f));

    // Outline.
    const Rgba8 ol{0, 0, 0, 140};
    StrokeLine(img, x0, y1, x0, y0, ol);
    StrokeLine(img, x1, y1, x1, y0, ol);
    StrokeLine(img, x0, y0, x1, y0, ol);

    if (cfgIn.includeEmissive && emissiveTop && !emit.rgba.empty()) {
      emitCol.a = 220;
      FillRect(emit, x0, y0, x1, y0 + 2, emitCol, BlendMode::Additive);
    }
  };

  // -----------------------------
  // Draw tiers from bottom to top.
  // -----------------------------
  int cumH = 0;

  for (int ti = 0; ti < tierCount; ++ti) {
    const TierDesc& td = tiers[static_cast<std::size_t>(ti)];
    const bool topTier = (ti == tierCount - 1);

    const TierGeom g = buildGeom(pivotY - cumH, td.shrink, td.heightPx);

    // Tier shading: upper tiers get a small lift.
    const float tierLift = 1.0f + 0.05f * static_cast<float>(ti);
    const float roofLift = 1.05f + 0.03f * static_cast<float>(ti);

    const float rightMul = (kind == GfxBuildingKind::Commercial) ? 0.90f : 0.86f;
    const float leftMul = (kind == GfxBuildingKind::Commercial) ? 0.76f : 0.70f;

    const Rgba8 wallC = Mul(wallBase, tierLift);
    const Rgba8 roofC = Mul(roofTint, (1.10f + (rand01(0x33u + static_cast<std::uint32_t>(ti) * 19u) - 0.5f) * 0.08f) * roofLift);
    const Rgba8 rightC = Mul(wallC, rightMul);
    const Rgba8 leftC = Mul(wallC, leftMul);

    // Walls.
    FillTriangle(img, g.rightQ.ax, g.rightQ.ay, g.rightQ.bx, g.rightQ.by, g.rightQ.cx, g.rightQ.cy, rightC);
    FillTriangle(img, g.rightQ.ax, g.rightQ.ay, g.rightQ.cx, g.rightQ.cy, g.rightQ.dx, g.rightQ.dy, rightC);

    FillTriangle(img, g.leftQ.ax, g.leftQ.ay, g.leftQ.bx, g.leftQ.by, g.leftQ.cx, g.leftQ.cy, leftC);
    FillTriangle(img, g.leftQ.ax, g.leftQ.ay, g.leftQ.cx, g.leftQ.cy, g.leftQ.dx, g.leftQ.dy, leftC);

    // Roof (top diamond). Use two-tone shading so it reads as a plane.
    {
      const Rgba8 roofA = Mul(roofC, 1.06f);
      const Rgba8 roofB = Mul(roofC, 0.92f);

      const bool gable = (kind == GfxBuildingKind::Residential) && topTier;
      if (gable) {
        FillTriangle(img, g.tx[0], g.ty[0], g.tx[1], g.ty[1], g.tx[2], g.ty[2], roofA);
        FillTriangle(img, g.tx[0], g.ty[0], g.tx[2], g.ty[2], g.tx[3], g.ty[3], roofB);
      } else {
        FillTriangle(img, g.tx[0], g.ty[0], g.tx[1], g.ty[1], g.tx[2], g.ty[2], roofA);
        FillTriangle(img, g.tx[0], g.ty[0], g.tx[2], g.ty[2], g.tx[3], g.ty[3], roofB);
      }

      // Roof texture hint (very light dither).
      const int minX = std::max(0, std::min({g.tx[0], g.tx[1], g.tx[2], g.tx[3]}));
      const int maxX = std::min(img.width - 1, std::max({g.tx[0], g.tx[1], g.tx[2], g.tx[3]}));
      const int minY = std::max(0, std::min({g.ty[0], g.ty[1], g.ty[2], g.ty[3]}));
      const int maxY = std::min(img.height - 1, std::max({g.ty[0], g.ty[1], g.ty[2], g.ty[3]}));

      for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
          if (!PointInDiamond(x, y, g.tx[0], g.ty[0], g.tx[1], g.ty[1], g.tx[2], g.ty[2], g.tx[3], g.ty[3])) continue;
          const std::uint32_t h = HashCoords32(x, y, seedv ^ 0xA11CE5u ^ static_cast<std::uint32_t>(ti) * 0x9E3779B9u);
          if ((h & 31u) != 0u) continue;
          // Speckle.
          BlendPixel(img, x, y, Mul(roofC, 0.86f));
        }
      }
    }

    // Windows.
    paintWindows(g.rightQ, ti, td.shrink, td.heightPx, 1.0f, 0x900Du + static_cast<std::uint32_t>(ti) * 0x101u, td.windows);
    paintWindows(g.leftQ, ti, td.shrink, td.heightPx, 0.92f, 0xBEEFu + static_cast<std::uint32_t>(ti) * 0x211u, td.windows);

    // Silhouette outlines for readability.
    {
      const Rgba8 ol{0, 0, 0, 150};
      // Roof perimeter.
      StrokeLine(img, g.tx[0], g.ty[0], g.tx[1], g.ty[1], ol);
      StrokeLine(img, g.tx[1], g.ty[1], g.tx[2], g.ty[2], ol);
      StrokeLine(img, g.tx[2], g.ty[2], g.tx[3], g.ty[3], ol);
      StrokeLine(img, g.tx[3], g.ty[3], g.tx[0], g.ty[0], ol);
      // Visible vertical edges.
      StrokeLine(img, g.bx[1], g.by[1], g.tx[1], g.ty[1], ol);
      StrokeLine(img, g.bx[3], g.by[3], g.tx[3], g.ty[3], ol);
      StrokeLine(img, g.bx[2], g.by[2], g.tx[2], g.ty[2], ol);
    }

    // -----------------------------
    // Rooftop / facade details
    // -----------------------------
    if (kind == GfxBuildingKind::Residential && topTier) {
      // Chimney
      const int hwPx = std::abs(g.tx[1] - pivotX);
      const int hhPx = std::abs(g.by[2] - (pivotY - cumH));
      const int cx = pivotX + static_cast<int>(std::lround(static_cast<float>(hwPx) * (0.22f + 0.20f * rand01(0x701u))));
      const int cy = g.ty[0] + static_cast<int>(std::lround(static_cast<float>(hhPx) * (0.65f + 0.18f * rand01(0x702u))));
      const int chH = std::max(10, tileH / 2 + lvl * 2);
      drawStack(cx, cy, 3, chH, Rgba8{70, 70, 70, 230}, /*emissiveTop=*/false, Rgba8{});
    }

    if (kind == GfxBuildingKind::Industrial && ti == 0) {
      // A couple of stacks on the main hall roof (kept away from the center so a top office tier can sit there).
      const int hwPx = std::abs(g.tx[1] - pivotX);
      const int hhPx = std::abs(g.by[2] - (pivotY - cumH));
      const int baseY = g.ty[0] + static_cast<int>(std::lround(static_cast<float>(hhPx) * 0.62f));

      const int sCount = (lvl >= 3) ? 2 : 1;
      for (int si = 0; si < sCount; ++si) {
        const float side = (si == 0) ? -1.0f : 1.0f;
        const float off = 0.35f + 0.10f * rand01(0x810u + static_cast<std::uint32_t>(si));
        const int cx = pivotX + static_cast<int>(std::lround(side * static_cast<float>(hwPx) * off));
        const int h = std::max(tileH, static_cast<int>(std::lround(static_cast<float>(tileH) * (1.20f + 0.55f * rand01(0x820u + static_cast<std::uint32_t>(si)) + 0.22f * static_cast<float>(lvl)))));
        drawStack(cx, baseY, 4, h, Rgba8{110, 110, 115, 230}, /*emissiveTop=*/(lvl >= 2), Rgba8{255, 150, 80, 220});
      }
    }

    if (kind == GfxBuildingKind::Commercial && topTier) {
      const int hwPx = std::abs(g.tx[1] - pivotX);
      const int hhPx = std::abs(g.by[2] - (pivotY - cumH));

      // Rooftop HVAC blocks (non-emissive).
      const int hvCount = (lvl == 1) ? 1 : (lvl == 2 ? 2 : 3);
      for (int i = 0; i < hvCount; ++i) {
        const std::uint32_t hs = HashU32(seedv ^ 0xACACACu ^ static_cast<std::uint32_t>(i) * 0x9E3779B9u);
        const float fx = 0.25f + 0.50f * Frac01(hs);
        const float fy = 0.35f + 0.35f * Frac01(hs ^ 0xBADC0DEu);

        const int cx = pivotX + static_cast<int>(std::lround((fx - 0.5f) * static_cast<float>(hwPx) * 1.10f));
        const int cy = g.ty[0] + static_cast<int>(std::lround(fy * static_cast<float>(hhPx) * 0.95f));
        const int w = 5 + static_cast<int>(hs & 3u);
        const int h = 4 + static_cast<int>((hs >> 3u) & 1u);

        const Rgba8 unit{170, 170, 175, 215};
        FillRect(img, cx - w / 2, cy - h, cx + (w - 1) / 2, cy, unit);
        FillRect(img, cx - w / 2, cy - h, cx - w / 2 + w / 3, cy, Mul(unit, 0.78f));
      }

// Signage: a small rooftop sign with deterministic "brand" text and a blurred emissive glow.
//
// This leans on the tiny software font + premultiplied blur helpers so we can add richer detail
// without introducing any external font/rendering dependencies.
if (lvl >= 2) {
  const std::uint32_t hs = HashU32(seedv ^ 0x51A7E1u);
  const int hue = static_cast<int>(hs & 3u);

  Rgba8 neon = litTint;
  if (hue == 1) neon = Rgba8{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 235};
  if (hue == 2) neon = Rgba8{static_cast<std::uint8_t>(200), static_cast<std::uint8_t>(100), static_cast<std::uint8_t>(255), 235};
  if (hue == 3) neon = Rgba8{static_cast<std::uint8_t>(90), static_cast<std::uint8_t>(220), static_cast<std::uint8_t>(255), 235};

  // Board placement near the top of the roof plane.
  const int boardHalf = std::max(10, static_cast<int>(std::lround(static_cast<float>(hwPx) * 0.70f)));
  int sx = pivotX - boardHalf;
  int ex = pivotX + boardHalf;
  int sy = g.ty[0] + 2;
  const int boardH = 9 + (lvl == 3 ? 2 : 0);
  int ey = sy + boardH - 1;

  // Clamp to sprite bounds.
  sx = std::clamp(sx, 0, img.width - 1);
  ex = std::clamp(ex, 0, img.width - 1);
  sy = std::clamp(sy, 0, img.height - 1);
  ey = std::clamp(ey, 0, img.height - 1);

  if (sx < ex && sy < ey) {
    // Board background in albedo.
    Rgba8 board = Mul(roofC, 0.52f);
    board.a = 225;
    FillRect(img, sx, sy, ex, ey, board);

    // A subtle outline helps separate it from the roof.
    const Rgba8 bol{0, 0, 0, 145};
    StrokeLine(img, sx, sy, ex, sy, bol);
    StrokeLine(img, sx, ey, ex, ey, bol);
    StrokeLine(img, sx, sy, sx, ey, bol);
    StrokeLine(img, ex, sy, ex, ey, bol);

    // Deterministic brand text (2-4 chars depending on board width).
    const int pad = 2;
    const int spacing = 1;
    const int availW = std::max(0, (ex - sx + 1) - pad * 2);

    // 5x7 monospace; choose a short token that fits the board.
    const int maxChars = std::max(1, (availW + spacing) / (gfx::Font5x7GlyphW() + spacing));
    const int desired = (lvl == 3) ? 4 : 3;
    const int len = std::clamp(desired - static_cast<int>((hs >> 6) & 1u), 2, maxChars);

    std::string name;
    name.reserve(static_cast<std::size_t>(len));
    constexpr char kAlpha[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    for (int i = 0; i < len; ++i) {
      const std::uint32_t hh = HashU32(hs ^ (static_cast<std::uint32_t>(i + 1) * 0x9E3779B9u));
      name.push_back(kAlpha[hh % 26u]);
    }

    const int textW = gfx::MeasureTextWidth5x7(name, 1, spacing);
    const int textH = gfx::MeasureTextHeight5x7(1);
    const int tx = pivotX - textW / 2;
    const int ty = sy + (boardH - textH) / 2;

    Rgba8 ink{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 235};
    Rgba8 outline{0, 0, 0, 175};
    gfx::DrawText5x7Outlined(img, tx, ty, name, ink, outline, 1, spacing, BlendMode::Alpha);

    // Emissive: draw crisp neon text + blurred halo.
    if (cfgIn.includeEmissive && !emit.rgba.empty()) {
      RgbaImage glow;
      glow.width = emit.width;
      glow.height = emit.height;
      glow.rgba.assign(emit.rgba.size(), 0u);

      Rgba8 glowCol = neon;
      glowCol.a = 220;
      gfx::DrawText5x7(glow, tx, ty, name, glowCol, 1, spacing, BlendMode::Additive);

      // Blur radius scales with level a bit (bigger buildings, stronger glow).
      const int br = 2 + (lvl == 3 ? 1 : 0);
      gfx::BoxBlurPremultiplied(glow, br);

      gfx::CompositeImage(emit, glow, BlendMode::Additive);

      // Re-draw crisp core on top.
      Rgba8 core = neon;
      core.a = 240;
      gfx::DrawText5x7(emit, tx, ty, name, core, 1, spacing, BlendMode::Additive);
    }
  }
}
    }

    cumH += td.heightPx;
  }

  out.color = std::move(img);
  out.emissive = std::move(emit);
  return true;
}

} // namespace isocity
