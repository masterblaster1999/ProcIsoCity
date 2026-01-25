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

enum class FacadeStyle : std::uint8_t {
  Grid = 0,
  CurtainWall = 1,
  Brick = 2,
  Corrugated = 3,
  Striped = 4,
  Checker = 5,
  Clerestory = 6,
};

FacadeStyle PickFacadeStyle(GfxBuildingKind kind, int lvl, std::uint32_t seedv, std::uint32_t salt)
{
  const std::uint32_t h = HashU32(seedv ^ salt);
  const int r = static_cast<int>(h & 7u);

  switch (kind) {
  case GfxBuildingKind::Commercial: {
    // Modern glass / stripes are more common for higher levels.
    if (lvl >= 2) {
      if (r < 3) return FacadeStyle::CurtainWall;
      if (r < 5) return FacadeStyle::Striped;
      return FacadeStyle::Grid;
    }
    // Small commercial blocks: mostly grid, occasional striping.
    return (r < 6) ? FacadeStyle::Grid : FacadeStyle::Striped;
  }
  case GfxBuildingKind::Industrial: {
    // Corrugated metal + clerestory windows are common.
    if (lvl == 1) return (r < 5) ? FacadeStyle::Corrugated : FacadeStyle::Grid;
    if (lvl == 2) {
      if (r < 3) return FacadeStyle::Corrugated;
      if (r < 6) return FacadeStyle::Clerestory;
      return FacadeStyle::Grid;
    }
    // High level industrial gets more office-like.
    return (r < 4) ? FacadeStyle::Grid : FacadeStyle::Clerestory;
  }
  case GfxBuildingKind::Residential:
  default: {
    // Brick & checker patterns add variety; high rises trend toward grid.
    if (lvl == 1) return (r < 5) ? FacadeStyle::Brick : FacadeStyle::Grid;
    if (lvl == 2) {
      if (r < 3) return FacadeStyle::Brick;
      if (r < 6) return FacadeStyle::Checker;
      return FacadeStyle::Grid;
    }
    if (r < 2) return FacadeStyle::Checker;
    if (r < 4) return FacadeStyle::Grid;
    return FacadeStyle::Brick;
  }
  }
}

std::string MakeBrandName(std::uint32_t h, int maxChars)
{
  maxChars = std::clamp(maxChars, 2, 8);

  // Alternating consonant/vowel produces more readable tokens than uniform random letters.
  static constexpr char kVowels[] = "AEIOU";
  static constexpr char kCons[] = "BCDFGHJKLMNPRSTVWXZ";

  int len = 3 + static_cast<int>(h & 1u);
  if ((h & 8u) != 0u) len += 1;
  len = std::clamp(len, 2, maxChars);

  std::string out;
  out.reserve(static_cast<std::size_t>(len));

  bool wantVowel = ((h >> 4u) & 1u) != 0u;
  std::uint32_t s = HashU32(h ^ 0x51A7E1u);

  for (int i = 0; i < len; ++i) {
    s = HashU32(s ^ (static_cast<std::uint32_t>(i + 1) * 0x9E3779B9u));
    if (wantVowel) {
      out.push_back(kVowels[s % (sizeof(kVowels) - 1u)]);
    } else {
      out.push_back(kCons[s % (sizeof(kCons) - 1u)]);
    }
    wantVowel = !wantVowel;
  }

  return out;
}

Rgba8 PickAccentColor(std::uint32_t h, GfxBuildingKind kind, const GfxPalette& pal)
{
  // Accent is used for rails/awnings/sign backgrounds. Keep it palette-aware.
  const int mode = static_cast<int>((h >> 20u) & 3u);
  Rgba8 a = pal.roadMarkYellow;
  if (mode == 1) a = pal.roadMarkWhite;
  if (mode == 2) a = Mix(pal.roadMarkYellow, pal.grass, 0.55f);
  if (mode == 3) a = Mix(pal.roadMarkWhite, RoofBaseColor(kind, pal), 0.35f);
  a.a = 255;
  return a;
}

void ProjectDecalToFace(RgbaImage& dst, const FaceQuad& q, float s0, float s1, float t0, float t1,
                        const RgbaImage& decal, float shade, BlendMode mode)
{
  if (decal.width <= 0 || decal.height <= 0) return;
  if (s1 <= s0 || t1 <= t0) return;

  const int minX = std::max(0, std::min({q.ax, q.bx, q.cx, q.dx}));
  const int maxX = std::min(dst.width - 1, std::max({q.ax, q.bx, q.cx, q.dx}));
  const int minY = std::max(0, std::min({q.ay, q.by, q.cy, q.dy}));
  const int maxY = std::min(dst.height - 1, std::max({q.ay, q.by, q.cy, q.dy}));

  const float invS = 1.0f / (s1 - s0);
  const float invT = 1.0f / (t1 - t0);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      float s = 0.0f;
      float t = 0.0f;
      if (!FaceLocalST(q, x, y, s, t)) continue;
      if (s < s0 || s > s1 || t < t0 || t > t1) continue;

      const float u = (s - s0) * invS;
      const float v = (t - t0) * invT;

      const float sx = u * static_cast<float>(std::max(0, decal.width - 1));
      const float sy = v * static_cast<float>(std::max(0, decal.height - 1));

      Rgba8 c = gfx::SampleBilinearPremultiplied(decal, sx, sy);
      if (c.a == 0) continue;

      c = Mul(c, shade);
      BlendPixel(dst, x, y, c, mode);
    }
  }
}

struct BillboardDecal {
  RgbaImage albedo;
  RgbaImage emissive;
};

BillboardDecal MakeBillboardDecal(const std::string& name, int w, int h, Rgba8 back, Rgba8 ink, Rgba8 border,
                                  bool neon, Rgba8 neonCol)
{
  BillboardDecal d{};

  w = std::clamp(w, 18, 96);
  h = std::clamp(h, 10, 32);

  d.albedo.width = w;
  d.albedo.height = h;
  d.albedo.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);

  FillRect(d.albedo, 0, 0, w - 1, h - 1, back);

  // Border.
  StrokeLine(d.albedo, 0, 0, w - 1, 0, border);
  StrokeLine(d.albedo, 0, h - 1, w - 1, h - 1, border);
  StrokeLine(d.albedo, 0, 0, 0, h - 1, border);
  StrokeLine(d.albedo, w - 1, 0, w - 1, h - 1, border);

  // A small logo mark on the left (simple circle-ish blob).
  const int logoR = std::max(2, h / 5);
  const int logoCx = 2 + logoR;
  const int logoCy = h / 2;
  for (int yy = -logoR; yy <= logoR; ++yy) {
    for (int xx = -logoR; xx <= logoR; ++xx) {
      const int x = logoCx + xx;
      const int y = logoCy + yy;
      if (x < 0 || y < 0 || x >= w || y >= h) continue;
      const float rr = static_cast<float>(xx * xx + yy * yy);
      if (rr > static_cast<float>(logoR * logoR)) continue;
      Rgba8 c = Mul(ink, 0.92f);
      c.a = 220;
      BlendPixel(d.albedo, x, y, c, BlendMode::Alpha);
    }
  }

  // Text.
  const int spacing = 1;
  const int scale = 1;

  std::string token = name;
  // Clip token if it doesn't fit.
  const int maxTextW = w - (logoCx + logoR + 3) - 2;
  while (!token.empty() && gfx::MeasureTextWidth5x7(token, scale, spacing) > maxTextW) {
    token.pop_back();
  }
  if (token.empty()) token = "CO";

  const int textW = gfx::MeasureTextWidth5x7(token, scale, spacing);
  const int textH = gfx::MeasureTextHeight5x7(scale);

  // Center the token within the remaining horizontal space to avoid a
  // consistently left-anchored look (and to silence MSVC's unused variable
  // warning for textW).
  const int txBase = (logoCx + logoR + 3);
  const int tx = txBase + std::max(0, (maxTextW - textW) / 2);
  const int ty = (h - textH) / 2;

  gfx::DrawText5x7Outlined(d.albedo, tx, ty, token, ink, Mul(border, 0.85f), scale, spacing, BlendMode::Alpha);

  if (neon) {
    d.emissive.width = w;
    d.emissive.height = h;
    d.emissive.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);

    Rgba8 core = neonCol;
    core.a = 235;

    // Neon logo ring.
    for (int yy = -logoR; yy <= logoR; ++yy) {
      for (int xx = -logoR; xx <= logoR; ++xx) {
        const int x = logoCx + xx;
        const int y = logoCy + yy;
        if (x < 0 || y < 0 || x >= w || y >= h) continue;
        const float rr = static_cast<float>(xx * xx + yy * yy);
        const float r0 = static_cast<float>(logoR * logoR);
        if (rr < r0 * 0.62f || rr > r0) continue;
        BlendPixel(d.emissive, x, y, core, BlendMode::Additive);
      }
    }

    gfx::DrawText5x7(d.emissive, tx, ty, token, core, scale, spacing, BlendMode::Additive);
  }

  return d;
}

void PaintFireEscape(RgbaImage& img, const FaceQuad& q, float faceShade, std::uint32_t seedv, std::uint32_t salt,
                     int lvl, int tierIdx, int tierCount, Rgba8 metal)
{
  // Cheap pixel-based fire escape hint: a vertical rail + zig-zag stair runs.
  // Only intended as a silhouette break-up; it is not a strict architectural model.

  const int minX = std::max(0, std::min({q.ax, q.bx, q.cx, q.dx}));
  const int maxX = std::min(img.width - 1, std::max({q.ax, q.bx, q.cx, q.dx}));
  const int minY = std::max(0, std::min({q.ay, q.by, q.cy, q.dy}));
  const int maxY = std::min(img.height - 1, std::max({q.ay, q.by, q.cy, q.dy}));

  const std::uint32_t hs = HashU32(seedv ^ salt);
  const float baseS = 0.18f + 0.08f * Frac01(hs);
  const float w = 0.013f + 0.004f * Frac01(hs ^ 0xBADC0DEu);

  const int landings = std::clamp(3 + lvl + (tierCount - 1 - tierIdx), 3, 10);

  const Rgba8 rail = Mul(metal, faceShade);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      float s = 0.0f;
      float t = 0.0f;
      if (!FaceLocalST(q, x, y, s, t)) continue;
      if (t < 0.10f || t > 0.96f) continue;

      const float ft = t * static_cast<float>(landings);
      const int li = static_cast<int>(std::floor(ft));
      const float fr = ft - static_cast<float>(li);

      // Vertical main rail.
      bool draw = (std::fabs(s - baseS) < w);

      // Horizontal landings.
      if (fr < 0.045f) {
        if (s > baseS - 0.12f && s < baseS + 0.12f) draw = true;
      }

      // Zig-zag stair run between landings.
      if (fr > 0.12f && fr < 0.88f) {
        const float dir = ((li & 1) == 0) ? 1.0f : -1.0f;
        const float sStair = baseS + dir * (fr - 0.5f) * 0.22f;
        if (std::fabs(s - sStair) < w * 0.95f) draw = true;
      }

      if (!draw) continue;

      Rgba8 c = rail;
      c.a = 195;
      BlendPixel(img, x, y, c, BlendMode::Alpha);

      // Tiny shadow line to add depth.
      if ((hs & 1u) != 0u) {
        if (std::fabs(s - (baseS + 0.03f)) < w * 0.65f) {
          Rgba8 sh = Mul(metal, 0.45f * faceShade);
          sh.a = 150;
          BlendPixel(img, x, y, sh, BlendMode::Alpha);
        }
      }
    }
  }
}

inline float SmoothStep01(float x)
{
  x = std::clamp(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

// A cheap sprite-space ambient occlusion + rim highlight pass.
//
// We don't have per-pixel normals/lighting for procedural sprites, but we can approximate
// "depth" by darkening pixels that are occluded by other pixels above them (screen-space
// directional AO). We then add a small rim light on exposed top-left edges so silhouettes
// read better at tiny resolutions.
//
// This is deterministic (seed-driven) and fast enough to run during sprite rebuild.
void ApplyDirectionalAORim(RgbaImage& img, std::uint32_t seedv, int pivotY, int tileH, GfxBuildingKind kind)
{
  if (img.width <= 0 || img.height <= 0) return;
  if (img.rgba.empty()) return;

  const int w = img.width;
  const int h = img.height;

  // Occupancy mask from alpha.
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0u);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = img.rgba[i + 3u];
      occ[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = (a >= 20u) ? 1u : 0u;
    }
  }

  auto occAt = [&](int x, int y) -> bool {
    if (x < 0 || y < 0 || x >= w || y >= h) return false;
    return occ[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] != 0u;
  };

  // Tune base strength per kind.
  float baseStrength = 0.30f;
  if (kind == GfxBuildingKind::Industrial) baseStrength = 0.38f;
  if (kind == GfxBuildingKind::Commercial) baseStrength = 0.26f;

  // Slight seed variation so families don't all have identical shading.
  baseStrength *= (0.92f + 0.16f * Frac01(HashU32(seedv ^ 0xA0A0A0u)));

  const int r = 5 + static_cast<int>(HashU32(seedv ^ 0xC0FFEEu) & 1u);

  struct Dir {
    int dx = 0;
    int dy = -1;
    float w = 1.0f;
  };

  // "Occlusion" directions: look for pixels above / above-left / above-right.
  // (Screenspace approximation of cavities under overhangs/setbacks.)
  const Dir dirs[] = {
    {0, -1, 0.55f},
    {-1, -1, 0.25f},
    {1, -1, 0.20f},
  };

  float denom = 0.0f;
  for (const Dir& d : dirs) {
    // Sum_{s=1..r} (r+1-s) = r*(r+1)/2
    denom += d.w * (static_cast<float>(r) * static_cast<float>(r + 1) * 0.5f);
  }
  denom = std::max(1.0e-6f, denom);

  // Ground plane reference: make occlusion a bit stronger near the base so buildings feel anchored.
  const int groundY = pivotY;
  const float invBaseBand = 1.0f / std::max(8.0f, static_cast<float>(tileH) * 1.35f);

  // AO + rim in one pass.
  for (int y = 0; y < h; ++y) {
    const float baseT = SmoothStep01(static_cast<float>(y - (groundY - tileH)) * invBaseBand);
    const float strength = baseStrength * (0.70f + 0.30f * baseT);

    for (int x = 0; x < w; ++x) {
      if (!occAt(x, y)) continue;

      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = img.rgba[i + 3u];
      if (a < 22u) continue;

      float occSum = 0.0f;
      for (const Dir& d : dirs) {
        for (int s = 1; s <= r; ++s) {
          const int sx = x + d.dx * s;
          const int sy = y + d.dy * s;
          if (sx < 0 || sy < 0 || sx >= w || sy >= h) break;
          if (!occAt(sx, sy)) continue;
          occSum += d.w * static_cast<float>(r + 1 - s);
        }
      }

      float o = std::clamp(occSum / denom, 0.0f, 1.0f);
      // Non-linear curve keeps most pixels subtle while emphasizing true "under" regions.
      o = o * o;

      float mul = 1.0f - strength * o;
      mul = std::clamp(mul, 0.55f, 1.0f);

      img.rgba[i + 0u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(img.rgba[i + 0u]) * mul)));
      img.rgba[i + 1u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(img.rgba[i + 1u]) * mul)));
      img.rgba[i + 2u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(img.rgba[i + 2u]) * mul)));

      // Rim highlight on exposed top-left edges (helps silhouettes pop at tiny scale).
      const bool edgeTop = !occAt(x, y - 1);
      const bool edgeTL = !occAt(x - 1, y - 1);
      const bool edgeL = !occAt(x - 1, y);
      if ((edgeTop || edgeTL) && edgeL) {
        // Fade highlight near the ground so it doesn't look like a white outline.
        const float rim = (0.08f + 0.05f * Frac01(HashU32(seedv ^ static_cast<std::uint32_t>(x * 73856093u) ^ static_cast<std::uint32_t>(y * 19349663u))))
                          * (1.0f - 0.85f * baseT);
        const int add = static_cast<int>(std::lround(255.0f * rim));
        img.rgba[i + 0u] = ClampU8(static_cast<int>(img.rgba[i + 0u]) + add);
        img.rgba[i + 1u] = ClampU8(static_cast<int>(img.rgba[i + 1u]) + add);
        img.rgba[i + 2u] = ClampU8(static_cast<int>(img.rgba[i + 2u]) + add);
      }
    }
  }
}

} // namespace

bool GenerateGfxBuildingSprite(GfxBuildingKind kind, int level, int variant, std::uint32_t seed, const GfxBuildingsConfig& cfgIn,
                               const GfxPalette& pal, GfxBuildingSprite& out, std::string& outError)
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

  // -----------------------------
  // Deterministic per-building seeds.
  //
  // Variants are grouped into a small number of "families" so that renderers can pick families
  // coherently for districts/neighborhoods.
  // -----------------------------
  const std::uint32_t famCountU = static_cast<std::uint32_t>(std::max(1, kGfxBuildingVariantFamilies));
  const std::uint32_t uVar = static_cast<std::uint32_t>(std::max(0, variant));
  const std::uint32_t fam = (famCountU > 0u) ? (uVar % famCountU) : 0u;
  const std::uint32_t localVar = (famCountU > 0u) ? (uVar / famCountU) : uVar;

  const std::uint32_t baseKey = seed ^ 0xC001D00Du ^ (static_cast<std::uint32_t>(lvl) * 0x85EBCA6Bu) ^
                                (static_cast<std::uint32_t>(kind) * 0xC2B2AE35u);
  const std::uint32_t familySeed = HashU32(baseKey ^ (fam * 0x9E3779B9u) ^ 0xF00DBABEu);
  const std::uint32_t seedv = HashU32(familySeed ^ (localVar * 0xD1B54A35u) ^ 0xA11CE5u);

  auto randFam01 = [&](std::uint32_t salt) { return Frac01(HashU32(familySeed ^ salt)); };
  auto rand01 = [&](std::uint32_t salt) { return Frac01(HashU32(seedv ^ salt)); };

  auto ix = [&](float x) { return static_cast<int>(std::lround(x)); };
  auto iy = [&](float y) { return static_cast<int>(std::lround(y)); };

  // -----------------------------
  // Style picks (macro decisions should mostly be stable within a family).
  // -----------------------------
  FacadeStyle facadeRight = PickFacadeStyle(kind, lvl, familySeed, 0xFA6ADE01u);
  FacadeStyle facadeLeft = PickFacadeStyle(kind, lvl, familySeed, 0xFA6ADE02u);

  // Low-rise residential benefits from actual roof volume; mid-rise sometimes does.
  const bool pitchedRoof = (kind == GfxBuildingKind::Residential) && (lvl <= 2) &&
                           ((lvl == 1) || (randFam01(0x7711u) < 0.45f));

  // Apartments get occasional balcony rails to break up repetition.
  const bool balconies = (kind == GfxBuildingKind::Residential) && (lvl >= 2) && (randFam01(0xBADC0DEu) < 0.75f);

  // Industrial roofs often hint at sawtooth/corrugated structures.
  const bool industrialRoofRidges = (kind == GfxBuildingKind::Industrial) && (lvl <= 2) && (randFam01(0x51707u) < 0.80f);

  // Some tall commercial buildings get a spire/antenna.
  const bool commercialSpire = (kind == GfxBuildingKind::Commercial) && (lvl == 3) && (randFam01(0x51EADu) < 0.60f);

  // Residential mid/high rises: occasional fire escape overlay.
  const bool fireEscape = (kind == GfxBuildingKind::Residential) && (lvl >= 2) && (randFam01(0xF1AEu) < 0.55f);

  // -----------------------------
  // Materials
  // -----------------------------
  const Rgba8 roofTint = RoofBaseColor(kind, pal);
  const Rgba8 glassTint = WindowTint(kind);
  const Rgba8 litTint = WindowLit(kind, pal);
  const Rgba8 accent = PickAccentColor(familySeed, kind, pal);

  // Choose a coarse "material preset"; keep it correlated with facade style when possible.
  enum class Mat : std::uint8_t { Concrete, Brick, Stucco, Glass, Metal, Stone };
  Mat mat = Mat::Concrete;

  switch (kind) {
  case GfxBuildingKind::Residential:
    mat = (fam == 0u) ? Mat::Brick : (fam == 1u ? Mat::Stucco : (fam == 2u ? Mat::Concrete : Mat::Brick));
    break;
  case GfxBuildingKind::Commercial:
    mat = (fam == 0u) ? Mat::Glass : (fam == 1u ? Mat::Stone : (fam == 2u ? Mat::Concrete : Mat::Glass));
    break;
  case GfxBuildingKind::Industrial:
    mat = (fam == 0u) ? Mat::Metal : (fam == 1u ? Mat::Concrete : (fam == 2u ? Mat::Brick : Mat::Metal));
    break;
  }

  // Style-material coupling.
  if (facadeRight == FacadeStyle::Brick || facadeLeft == FacadeStyle::Brick) mat = Mat::Brick;
  if (facadeRight == FacadeStyle::Corrugated || facadeLeft == FacadeStyle::Corrugated) mat = Mat::Metal;
  if (facadeRight == FacadeStyle::CurtainWall || facadeLeft == FacadeStyle::CurtainWall) mat = Mat::Glass;

  Rgba8 wallNeutral{210, 210, 210, 255};
  float wallMix = 0.52f;

  switch (mat) {
  case Mat::Brick:
    wallNeutral = (kind == GfxBuildingKind::Industrial) ? Rgba8{182, 130, 110, 255} : Rgba8{196, 150, 135, 255};
    wallMix = (kind == GfxBuildingKind::Residential) ? 0.30f : 0.38f;
    break;
  case Mat::Stucco:
    wallNeutral = Rgba8{232, 224, 212, 255};
    wallMix = 0.32f;
    break;
  case Mat::Glass:
    wallNeutral = Rgba8{190, 210, 225, 255};
    wallMix = 0.70f;
    break;
  case Mat::Metal:
    wallNeutral = Rgba8{204, 205, 210, 255};
    wallMix = 0.58f;
    break;
  case Mat::Stone:
    wallNeutral = Rgba8{212, 210, 202, 255};
    wallMix = 0.46f;
    break;
  case Mat::Concrete:
  default:
    wallNeutral = (kind == GfxBuildingKind::Residential) ? Rgba8{222, 214, 200, 255} : Rgba8{206, 210, 214, 255};
    wallMix = (kind == GfxBuildingKind::Commercial) ? 0.62f : 0.50f;
    break;
  }

  const Rgba8 wallBase = Mix(wallNeutral, roofTint, wallMix);

  // -----------------------------
  // Footprint + height heuristics.
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
  baseShrink = std::clamp(baseShrink + (randFam01(0x11u) - 0.5f) * 0.06f, 0.42f, 0.74f);

  float baseShrinkX = baseShrink;
  float baseShrinkY = baseShrink;

  // Rectangular footprint variation (stronger for industrial/commercial).
  float rectChance = 0.45f;
  if (kind == GfxBuildingKind::Industrial) rectChance = 0.80f;
  if (kind == GfxBuildingKind::Commercial) rectChance = 0.65f;
  if (lvl == 3) rectChance = std::min(0.90f, rectChance + 0.10f);

  if (randFam01(0xA5AEC7u) < rectChance) {
    float asp = 0.78f + 0.62f * randFam01(0xA5B2u);
    if (randFam01(0xA5B3u) < 0.50f) asp = 1.0f / std::max(0.01f, asp);
    baseShrinkX = std::clamp(baseShrinkX * asp, 0.36f, 0.80f);
    baseShrinkY = std::clamp(baseShrinkY / asp, 0.36f, 0.80f);
  }

  // Small per-axis wobble.
  baseShrinkX = std::clamp(baseShrinkX + (rand01(0x11AAu) - 0.5f) * 0.05f, 0.36f, 0.80f);
  baseShrinkY = std::clamp(baseShrinkY + (rand01(0x11BBu) - 0.5f) * 0.05f, 0.36f, 0.80f);

  float heightPx = static_cast<float>(tileH) * (0.72f + 0.58f * static_cast<float>(lvl));
  heightPx *= heightMul;
  heightPx *= 0.82f + 0.42f * randFam01(0x22u);

  // Big commercial buildings get extra variance.
  if (kind == GfxBuildingKind::Commercial && lvl == 3) {
    heightPx *= 1.05f + 0.22f * randFam01(0x23u);
  }

  heightPx = std::clamp(heightPx, static_cast<float>(tileH) * 0.65f, static_cast<float>(maxHeightPx));
  const int totalHPx = static_cast<int>(std::lround(heightPx));

  // -----------------------------
  // Tiered silhouettes (setbacks)
  // -----------------------------
  struct TierDesc {
    float shrinkX = 0.60f;
    float shrinkY = 0.60f;
    int heightPx = 0;
    bool windows = true;
  };

  int tierCount = 1;
  std::vector<float> fracs;

  if (kind == GfxBuildingKind::Commercial) {
    tierCount = (lvl == 1) ? 1 : (lvl == 2 ? 2 : 3);
    fracs = (tierCount == 1) ? std::vector<float>{1.0f}
                             : (tierCount == 2 ? std::vector<float>{0.42f, 0.58f}
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
    int h = last ? remainingH
                 : static_cast<int>(std::lround(static_cast<float>(totalHPx) * fracs[static_cast<std::size_t>(i)]));
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
      k += (randFam01(0x400u + static_cast<std::uint32_t>(i) * 31u) - 0.5f) * 0.06f;
    } else if (kind == GfxBuildingKind::Industrial) {
      k = 1.0f - 0.14f * static_cast<float>(i);
      k += (randFam01(0x500u + static_cast<std::uint32_t>(i) * 29u) - 0.5f) * 0.05f;
    } else {
      k = 1.0f - 0.10f * static_cast<float>(i);
      k += (randFam01(0x600u + static_cast<std::uint32_t>(i) * 23u) - 0.5f) * 0.04f;
    }

    const float sx = std::clamp(baseShrinkX * k, 0.34f, 0.82f);
    const float sy = std::clamp(baseShrinkY * k, 0.34f, 0.82f);

    tiers[static_cast<std::size_t>(i)].shrinkX = sx;
    tiers[static_cast<std::size_t>(i)].shrinkY = sy;
    tiers[static_cast<std::size_t>(i)].windows = true;
  }

  // Industrial base halls tend to have fewer windows.
  if (kind == GfxBuildingKind::Industrial && tierCount >= 1) {
    tiers[0].windows = (lvl >= 3); // mostly office-like at high level
  }

  // -----------------------------
  // Annex volumes (a small attached wing) for extra silhouette richness.
  // -----------------------------
  struct VolumeDesc {
    int basePivotX = 0;
    int basePivotY = 0;
    std::vector<TierDesc> tiers;
    FacadeStyle facadeRight = FacadeStyle::Grid;
    FacadeStyle facadeLeft = FacadeStyle::Grid;
    bool allowRooftopDetails = true;
    bool isAnnex = false;
  };

  VolumeDesc mainV{};
  mainV.basePivotX = pivotX;
  mainV.basePivotY = pivotY;
  mainV.tiers = tiers;
  mainV.facadeRight = facadeRight;
  mainV.facadeLeft = facadeLeft;
  mainV.allowRooftopDetails = true;
  mainV.isAnnex = false;

  std::vector<VolumeDesc> volumes;

  // Decide annex.
  float annexChance = 0.0f;
  if (kind == GfxBuildingKind::Industrial) annexChance = (lvl >= 2) ? 0.75f : 0.55f;
  if (kind == GfxBuildingKind::Commercial) annexChance = (lvl >= 2) ? 0.55f : 0.20f;
  if (kind == GfxBuildingKind::Residential) annexChance = (lvl >= 2) ? 0.35f : 0.0f;

  const bool wantAnnex = (annexChance > 0.0f) && (randFam01(0xA66E9u) < annexChance);

  if (wantAnnex && !tiers.empty()) {
    const float hwBase = static_cast<float>(tileW) * 0.5f * tiers[0].shrinkX;
    const float hhBase = static_cast<float>(tileH) * 0.5f * tiers[0].shrinkY;

    VolumeDesc annexV{};
    annexV.isAnnex = true;
    annexV.allowRooftopDetails = false;

    const bool toRight = randFam01(0xA66EAu) < 0.55f;
    const float dx = (0.22f + 0.14f * randFam01(0xA66EBu)) * hwBase;
    const float dy = (0.18f + 0.10f * randFam01(0xA66ECu)) * hhBase;

    annexV.basePivotX = pivotX + (toRight ? ix(dx) : -ix(dx));
    annexV.basePivotY = pivotY + iy(dy);

    TierDesc at{};
    at.shrinkX = std::clamp(tiers[0].shrinkX * (0.52f + 0.18f * randFam01(0xA66EDu)), 0.30f, 0.76f);
    at.shrinkY = std::clamp(tiers[0].shrinkY * (0.52f + 0.18f * randFam01(0xA66EEu)), 0.30f, 0.76f);

    float hFrac = 0.34f + 0.24f * randFam01(0xA66EFu);
    if (kind == GfxBuildingKind::Industrial) hFrac = 0.28f + 0.22f * randFam01(0xA66EFu);
    at.heightPx = std::clamp(static_cast<int>(std::lround(static_cast<float>(totalHPx) * hFrac)), minTierH, std::max(minTierH, totalHPx - minTierH));

    // Industrial annex is office block: always windows.
    at.windows = (kind != GfxBuildingKind::Industrial) ? tiers[0].windows : true;

    annexV.tiers = {at};

    // Annex facade styles: slightly biased toward grid to keep it readable.
    annexV.facadeRight = PickFacadeStyle(kind, lvl, familySeed ^ 0xA66E9u, 0x10001u);
    annexV.facadeLeft = PickFacadeStyle(kind, lvl, familySeed ^ 0xA66E9u, 0x10002u);

    volumes.push_back(annexV);
  }

  // Main volume drawn last so it sits "in front" of annex.
  volumes.push_back(mainV);

  // -----------------------------
  // Contact shadows
  // -----------------------------
  auto drawContactShadow = [&](int basePivotX, int basePivotY, float shrinkX, float shrinkY, std::uint8_t alpha) {
    const float shadowShrinkX = std::min(0.98f, shrinkX * 1.12f);
    const float shadowShrinkY = std::min(0.98f, shrinkY * 1.12f);
    const float shw = static_cast<float>(tileW) * 0.5f * shadowShrinkX;
    const float shh = static_cast<float>(tileH) * 0.5f * shadowShrinkY;

    const int sx0 = basePivotX;
    const int sy0 = basePivotY - iy(shh);
    const int sx1 = basePivotX + ix(shw);
    const int sy1 = basePivotY;
    const int sx2 = basePivotX;
    const int sy2 = basePivotY + iy(shh);
    const int sx3 = basePivotX - ix(shw);
    const int sy3 = basePivotY;

    Rgba8 sc{0, 0, 0, alpha};
    FillTriangle(img, sx0, sy0, sx1, sy1, sx2, sy2, sc);
    FillTriangle(img, sx0, sy0, sx2, sy2, sx3, sy3, sc);
  };

  for (const auto& v : volumes) {
    if (v.tiers.empty()) continue;
    const std::uint8_t a = static_cast<std::uint8_t>(std::clamp(14 + lvl * 6 + (v.isAnnex ? -4 : 0), 8, 32));
    drawContactShadow(v.basePivotX, v.basePivotY, v.tiers[0].shrinkX, v.tiers[0].shrinkY, a);
  }

  // -----------------------------
  // Geometry helpers
  // -----------------------------
  struct TierGeom {
    int bx[4]{};
    int by[4]{};
    int tx[4]{};
    int ty[4]{};
    FaceQuad rightQ{};
    FaceQuad leftQ{};
    int basePivotX = 0;
    int baseY = 0;
    float hw = 0.0f;
    float hh = 0.0f;
  };

  auto buildGeom = [&](int basePivotX, int baseY, float shrinkX, float shrinkY, int hPx) -> TierGeom {
    TierGeom g{};
    g.basePivotX = basePivotX;
    g.baseY = baseY;
    g.hw = static_cast<float>(tileW) * 0.5f * shrinkX;
    g.hh = static_cast<float>(tileH) * 0.5f * shrinkY;

    const int hwPx = ix(g.hw);
    const int hhPx = iy(g.hh);

    // Base diamond corners (top, right, bottom, left).
    g.bx[0] = basePivotX;
    g.by[0] = baseY - hhPx;
    g.bx[1] = basePivotX + hwPx;
    g.by[1] = baseY;
    g.bx[2] = basePivotX;
    g.by[2] = baseY + hhPx;
    g.bx[3] = basePivotX - hwPx;
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

  // -----------------------------
  // Painting helpers
  // -----------------------------
  auto paintFacade = [&](const FaceQuad& q, Rgba8 base, int tierIdx, FacadeStyle style, std::uint32_t saltBase) {
    // Subtle facade material hints (brick courses, mullions, corrugation, etc.).
    const int minX = std::max(0, std::min({q.ax, q.bx, q.cx, q.dx}));
    const int maxX = std::min(img.width - 1, std::max({q.ax, q.bx, q.cx, q.dx}));
    const int minY = std::max(0, std::min({q.ay, q.by, q.cy, q.dy}));
    const int maxY = std::min(img.height - 1, std::max({q.ay, q.by, q.cy, q.dy}));

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        float s = 0.0f;
        float t = 0.0f;
        if (!FaceLocalST(q, x, y, s, t)) continue;
        if (t < 0.01f || t > 0.995f) continue;

        const std::uint32_t hh = HashCoords32(x, y, seedv ^ saltBase);
        const float n = (Frac01(hh) - 0.5f) * 0.10f;

        // Slightly brighter up the facade.
        float shade = 0.94f + 0.08f * t;

        if (style == FacadeStyle::Brick) {
          const int courses = 10 + lvl * 4 + tierIdx * 2;
          const float vf = t * static_cast<float>(courses);
          const float fr = vf - std::floor(vf);
          if (fr < 0.04f) shade *= 0.82f; // mortar line
          shade *= std::clamp(1.0f + n, 0.92f, 1.08f);
        } else if (style == FacadeStyle::Corrugated) {
          const int ridges = 12 + lvl * 3 + tierIdx * 2;
          const float uf = s * static_cast<float>(ridges);
          const float fr = uf - std::floor(uf);
          if (fr < 0.06f) shade *= 0.78f;
          else if (fr > 0.94f) shade *= 0.86f;

          // grime near ground
          const float g = std::clamp(1.0f - t / 0.25f, 0.0f, 1.0f);
          shade *= 1.0f - 0.10f * g;
          shade *= std::clamp(1.0f + n, 0.90f, 1.10f);
        } else if (style == FacadeStyle::CurtainWall) {
          // Mullion-like hints.
          const int mull = 6 + lvl * 3;
          const float uf = s * static_cast<float>(mull);
          const float fr = uf - std::floor(uf);
          if (fr < 0.025f || fr > 0.975f) shade *= 0.75f;

          // Spandrel lines.
          const int floors = 6 + lvl * 5;
          const float vf = t * static_cast<float>(floors);
          const float fr2 = vf - std::floor(vf);
          if (fr2 < 0.020f) shade *= 0.85f;

          shade *= std::clamp(1.0f + 0.5f * n, 0.95f, 1.05f);
        } else if (style == FacadeStyle::Striped) {
          const int stripes = 5 + lvl * 2;
          const int si = static_cast<int>(std::floor(s * static_cast<float>(stripes)));
          shade *= ((si & 1) == 0) ? 1.06f : 0.94f;
        } else if (style == FacadeStyle::Checker) {
          const int blocks = 5 + lvl * 2;
          const int si = static_cast<int>(std::floor(s * static_cast<float>(blocks)));
          const int ti = static_cast<int>(std::floor(t * static_cast<float>(blocks)));
          shade *= (((si + ti + tierIdx) & 1) == 0) ? 1.04f : 0.96f;
        } else if (style == FacadeStyle::Clerestory) {
          // Keep lower wall plainer; upper band slightly brighter.
          shade *= (t > 0.65f) ? 1.06f : 0.97f;
          shade *= std::clamp(1.0f + 0.5f * n, 0.95f, 1.05f);
        }

        Rgba8 oc = Mul(base, shade);
        oc.a = 40;
        BlendPixel(img, x, y, oc, BlendMode::Alpha);
      }
    }
  };

  struct WindowGrid {
    int cols = 2;
    int rows = 2;
    float padU = 0.20f;
    float padV = 0.24f;
  };

  auto computeWindowGrid = [&](float edgeFrac, int tierHeightPx, int totalHPxRef, FacadeStyle style) -> WindowGrid {
    WindowGrid g{};

    edgeFrac = std::clamp(edgeFrac, 0.45f, 1.0f);

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

    // Style tweaks.
    if (style == FacadeStyle::CurtainWall) {
      baseCols += 4;
      baseRows += 4;
      padU = std::min(padU, 0.085f);
      padV = std::min(padV, 0.105f);
    } else if (style == FacadeStyle::Clerestory) {
      // fewer, larger windows
      baseCols = std::max(2, baseCols - 1);
      baseRows = std::max(2, baseRows - 1);
      padU = std::max(padU, 0.20f);
      padV = std::max(padV, 0.22f);
    }

    g.cols = std::max(2, static_cast<int>(std::lround(static_cast<float>(baseCols) * edgeFrac)));

    const float hFrac = std::clamp(static_cast<float>(tierHeightPx) / std::max(1.0f, static_cast<float>(totalHPxRef)), 0.20f, 1.0f);
    g.rows = std::max(2, static_cast<int>(std::lround(static_cast<float>(baseRows) * (0.75f + 0.25f * hFrac))));

    g.padU = padU;
    g.padV = padV;
    return g;
  };

  auto paintWindows = [&](const FaceQuad& q, int tierIdx, float edgeFrac, int tierHeightPx, int totalHPxRef, float faceShade,
                          std::uint32_t saltBase, bool enabled, FacadeStyle style) {
    if (!enabled) return;

    const WindowGrid wg = computeWindowGrid(edgeFrac, tierHeightPx, totalHPxRef, style);
    const int cols = wg.cols;
    const int rows = wg.rows;
    const float padU = wg.padU;
    const float padV = wg.padV;

    const bool drawFrames = (kind != GfxBuildingKind::Industrial) || (style == FacadeStyle::CurtainWall);
    const float frameU = std::clamp(padU * 0.50f, 0.025f, 0.065f);
    const float frameV = std::clamp(padV * 0.50f, 0.025f, 0.070f);

    // Frame tint uses wall color; slightly darker reads as mullions.
    Rgba8 frame = Mul(wallBase, 0.62f * faceShade);
    frame.a = 90;

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

        const std::uint32_t hw = HashCoords32(ci + tierIdx * 31, ri + tierIdx * 17, seedv ^ saltBase);

        // Style-level presence masks (applied per cell).
        bool hasWindow = true;

        if (style == FacadeStyle::Striped) {
          const int period = 3 + (lvl == 3 ? 1 : 0);
          const int band = ci % period;
          hasWindow = (band != (period - 1)); // one thin solid stripe per period
        } else if (style == FacadeStyle::Checker) {
          hasWindow = (((ci + ri + tierIdx) & 1) == 0);
        } else if (style == FacadeStyle::Clerestory) {
          const int topRows = (kind == GfxBuildingKind::Industrial) ? 2 : 3;
          hasWindow = (ri >= rows - topRows);
        }

        // Industrial: fewer windows unless it's an office-like tier.
        if (kind == GfxBuildingKind::Industrial) {
          if ((hw & 3u) != 0u) hasWindow = false;
          if (lvl >= 3 && tierIdx > 0) hasWindow = true;
        }

        if (!hasWindow) continue;

        const bool inWin = (fu >= padU && fu <= (1.0f - padU) && fv >= padV && fv <= (1.0f - padV));
        if (!inWin) {
          if (!drawFrames) continue;
          const bool nearEdge = (fu < padU + frameU || fu > (1.0f - padU - frameU) || fv < padV + frameV ||
                                 fv > (1.0f - padV - frameV));
          if (!nearEdge) continue;
          BlendPixel(img, x, y, frame, BlendMode::Alpha);
          continue;
        }

        const float n = (Frac01(hw) - 0.5f) * 0.10f;

        // Commercial: occasional dark floors.
        float floorMul = 1.0f;
        if (kind == GfxBuildingKind::Commercial) {
          const int floor = ri;
          if (((floor + tierIdx) & 3) == 0) floorMul = 0.86f;
        }

        Rgba8 wc = Mul(glassTint, std::clamp(1.0f + n, 0.82f, 1.18f));
        wc = Mul(wc, faceShade * floorMul);
        BlendPixel(img, x, y, wc);

        // Residential balcony rails (simple pixel hints, aligned to the same grid as windows).
        if (balconies && kind == GfxBuildingKind::Residential && lvl >= 2) {
          const bool rowHasBalcony = (((ri + tierIdx) & 1) == 0) || (Frac01(hw ^ 0xBA1C0u) < 0.35f);
          if (rowHasBalcony) {
            const float railBand0 = (1.0f - padV) - 0.12f;
            const float railBand1 = (1.0f - padV) - 0.05f;
            if (fv > railBand0 && fv < railBand1) {
              Rgba8 rail = Mix(Mul(wallBase, 0.68f), accent, 0.38f);
              rail = Mul(rail, faceShade);
              rail.a = 190;
              BlendPixel(img, x, y, rail, BlendMode::Alpha);
            }
            const float shadowBand0 = (1.0f - padV) - 0.05f;
            const float shadowBand1 = (1.0f - padV) - 0.02f;
            if (fv > shadowBand0 && fv < shadowBand1) {
              Rgba8 sh = Mul(wallBase, 0.55f * faceShade);
              sh.a = 160;
              BlendPixel(img, x, y, sh, BlendMode::Alpha);
            }
          }
        }

        if (cfgIn.includeEmissive && !emit.rgba.empty()) {
          float pLit = 0.35f;
          if (kind == GfxBuildingKind::Residential) pLit = 0.28f;
          if (kind == GfxBuildingKind::Commercial) pLit = 0.58f;
          if (kind == GfxBuildingKind::Industrial) pLit = 0.18f;
          pLit += 0.04f * static_cast<float>(lvl - 1);
          pLit += 0.04f * static_cast<float>(tierIdx);

          // Curtain wall reads better with a slightly higher average lit fraction.
          if (style == FacadeStyle::CurtainWall) pLit += 0.06f;

          if (Frac01(hw ^ 0xDEADBEEFu) < pLit) {
            Rgba8 ec = litTint;
            ec.a = static_cast<std::uint8_t>(170 + (hw & 0x3Fu));
            BlendPixel(emit, x, y, ec, BlendMode::Additive);
          }
        }
      }
    }
  };

  auto paintLoadingDoor = [&](const FaceQuad& q, std::uint32_t saltBase) {
    // A simple industrial roll-up door on the right face of the base tier.
    const int minX = std::max(0, std::min({q.ax, q.bx, q.cx, q.dx}));
    const int maxX = std::min(img.width - 1, std::max({q.ax, q.bx, q.cx, q.dx}));
    const int minY = std::max(0, std::min({q.ay, q.by, q.cy, q.dy}));
    const int maxY = std::min(img.height - 1, std::max({q.ay, q.by, q.cy, q.dy}));

    const float s0 = 0.54f;
    const float s1 = 0.90f;
    const float t0 = 0.02f;
    const float t1 = 0.38f;

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        float s = 0.0f;
        float t = 0.0f;
        if (!FaceLocalST(q, x, y, s, t)) continue;
        if (s < s0 || s > s1 || t < t0 || t > t1) continue;

        const float u = (s - s0) / (s1 - s0);
        const float v = (t - t0) / (t1 - t0);

        const std::uint32_t h = HashCoords32(x, y, seedv ^ saltBase);
        const float n = (Frac01(h) - 0.5f) * 0.10f;

        Rgba8 dc = Mul(wallBase, 0.50f + 0.08f * n);
        dc.a = 235;

        // Ridges.
        const int ridges = 9;
        const float rf = v * static_cast<float>(ridges);
        const float rfr = rf - std::floor(rf);
        if (rfr < 0.10f) dc = Mul(dc, 0.82f);
        if (rfr > 0.92f) dc = Mul(dc, 1.10f);

        // Outline.
        const float edge = 0.06f;
        if (u < edge || u > (1.0f - edge) || v < edge || v > (1.0f - edge)) {
          dc = Mul(dc, 0.76f);
          dc.a = 245;
        }

        BlendPixel(img, x, y, dc, BlendMode::Alpha);

        // Clear emissive windows behind the door.
        if (cfgIn.includeEmissive && !emit.rgba.empty()) {
          const std::size_t ii =
            (static_cast<std::size_t>(y) * static_cast<std::size_t>(emit.width) + static_cast<std::size_t>(x)) * 4u;
          if (ii + 3 < emit.rgba.size()) {
            emit.rgba[ii + 0] = 0;
            emit.rgba[ii + 1] = 0;
            emit.rgba[ii + 2] = 0;
            emit.rgba[ii + 3] = 0;
          }
        }
      }
    }

    // A tiny warm light above the bay.
    if (cfgIn.includeEmissive && !emit.rgba.empty()) {
      const int lx0 = std::clamp(static_cast<int>(std::lround(static_cast<float>(q.ax + q.bx) * 0.5f)), 0, emit.width - 1);
      const int ly0 = std::clamp(std::min(q.ay, q.by) - 6, 0, emit.height - 1);
      const int lx1 = std::clamp(lx0 + 2, 0, emit.width - 1);
      const int ly1 = std::clamp(ly0 + 1, 0, emit.height - 1);
      FillRect(emit, lx0, ly0, lx1, ly1, Rgba8{255, 170, 80, 200}, BlendMode::Additive);
    }
  };

  auto paintStorefront = [&](const FaceQuad& q, std::uint32_t saltBase) {
    // Commercial lvl1: large bottom glass + a tiny awning.
    const int minX = std::max(0, std::min({q.ax, q.bx, q.cx, q.dx}));
    const int maxX = std::min(img.width - 1, std::max({q.ax, q.bx, q.cx, q.dx}));
    const int minY = std::max(0, std::min({q.ay, q.by, q.cy, q.dy}));
    const int maxY = std::min(img.height - 1, std::max({q.ay, q.by, q.cy, q.dy}));

    const float s0 = 0.16f;
    const float s1 = 0.92f;
    const float t0 = 0.03f;
    const float t1 = 0.42f;

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        float s = 0.0f;
        float t = 0.0f;
        if (!FaceLocalST(q, x, y, s, t)) continue;
        if (s < s0 || s > s1 || t < t0 || t > t1) continue;

        const float u = (s - s0) / (s1 - s0);
        const float v = (t - t0) / (t1 - t0);

        const std::uint32_t h = HashCoords32(x, y, seedv ^ saltBase);
        const float n = (Frac01(h) - 0.5f) * 0.08f;

        Rgba8 gc = Mul(glassTint, 1.10f + n);
        gc = Mul(gc, 1.05f);
        gc.a = 235;

        // Thin frame.
        const float edge = 0.045f;
        if (u < edge || u > (1.0f - edge) || v < edge || v > (1.0f - edge)) {
          Rgba8 fc = Mul(wallBase, 0.58f);
          fc.a = 220;
          BlendPixel(img, x, y, fc, BlendMode::Alpha);
        } else {
          BlendPixel(img, x, y, gc, BlendMode::Alpha);
        }

        if (cfgIn.includeEmissive && !emit.rgba.empty()) {
          // Interior glow, stronger near the top.
          float p = 0.35f + 0.45f * v;
          if (Frac01(h ^ 0x51A7E1u) < p) {
            Rgba8 ec = litTint;
            ec.a = 140;
            BlendPixel(emit, x, y, ec, BlendMode::Additive);
          }
        }
      }
    }

    // Awning stripe.
    const float aw0 = 0.42f;
    const float aw1 = 0.50f;
    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        float s = 0.0f;
        float t = 0.0f;
        if (!FaceLocalST(q, x, y, s, t)) continue;
        if (s < s0 || s > s1 || t < aw0 || t > aw1) continue;
        Rgba8 aw = Mix(Mul(wallBase, 0.70f), accent, 0.55f);
        aw.a = 215;
        BlendPixel(img, x, y, aw, BlendMode::Alpha);
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

  auto paintBillboardOnFace = [&](const FaceQuad& q, float faceShade, std::uint32_t saltBase, bool neon) {
    // A mid-facade billboard panel projected into the face.
    const float s0 = 0.20f;
    const float s1 = 0.86f;
    const float t0 = 0.58f;
    const float t1 = 0.80f;

    const std::uint32_t h = HashU32(familySeed ^ saltBase);

    const int boardW = 42;
    const int boardH = 14;

    const int maxChars = 6;
    std::string brand = MakeBrandName(h ^ 0xC0FFEEu, maxChars);

    Rgba8 back = Mul(Mix(wallBase, accent, 0.55f), 0.90f);
    back.a = 235;
    Rgba8 ink = Mul(pal.roadMarkWhite, 0.95f);
    ink.a = 235;
    Rgba8 border = Mul(wallBase, 0.45f);
    border.a = 255;

    // Neon hues keyed off the family.
    const int hue = static_cast<int>((h >> 8u) & 3u);
    Rgba8 neonCol = litTint;
    if (hue == 1) neonCol = Rgba8{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 235};
    if (hue == 2) neonCol = Rgba8{200, 100, 255, 235};
    if (hue == 3) neonCol = Rgba8{90, 220, 255, 235};

    BillboardDecal d = MakeBillboardDecal(brand, boardW, boardH, back, ink, border, neon, neonCol);

    ProjectDecalToFace(img, q, s0, s1, t0, t1, d.albedo, faceShade, BlendMode::Alpha);

    if (cfgIn.includeEmissive && neon && !emit.rgba.empty() && d.emissive.width > 0) {
      // Full-size glow pass for proper blur.
      RgbaImage glow;
      glow.width = emit.width;
      glow.height = emit.height;
      glow.rgba.assign(emit.rgba.size(), 0u);

      ProjectDecalToFace(glow, q, s0, s1, t0, t1, d.emissive, 1.0f, BlendMode::Additive);
      gfx::BoxBlurPremultiplied(glow, 2 + (lvl == 3 ? 1 : 0));
      gfx::CompositeImage(emit, glow, BlendMode::Additive);
      ProjectDecalToFace(emit, q, s0, s1, t0, t1, d.emissive, 1.0f, BlendMode::Additive);
    }
  };

  // -----------------------------
  // Render volumes (annex first, main last).
  // -----------------------------
  for (const auto& vol : volumes) {
    if (vol.tiers.empty()) continue;

    // Reference edge length for window density scaling.
    const float baseEdge = std::sqrt(std::max(0.001f, (static_cast<float>(tileW) * 0.5f * vol.tiers[0].shrinkX) *
                                              (static_cast<float>(tileW) * 0.5f * vol.tiers[0].shrinkX) +
                                              (static_cast<float>(tileH) * 0.5f * vol.tiers[0].shrinkY) *
                                              (static_cast<float>(tileH) * 0.5f * vol.tiers[0].shrinkY)));

    int cumH = 0;

    for (int ti = 0; ti < static_cast<int>(vol.tiers.size()); ++ti) {
      const TierDesc& td = vol.tiers[static_cast<std::size_t>(ti)];
      const bool topTier = (ti == static_cast<int>(vol.tiers.size()) - 1);

      const int baseY = vol.basePivotY - cumH;
      const TierGeom g = buildGeom(vol.basePivotX, baseY, td.shrinkX, td.shrinkY, td.heightPx);

      int roofPeakX = 0;
      int roofPeakY = 0;
      bool roofHasPeak = false;

      // Tier shading: upper tiers get a small lift.
      const float tierLift = 1.0f + 0.05f * static_cast<float>(ti);
      const float roofLift = 1.05f + 0.03f * static_cast<float>(ti);

      const float rightMul = (kind == GfxBuildingKind::Commercial) ? 0.90f : 0.86f;
      const float leftMul = (kind == GfxBuildingKind::Commercial) ? 0.76f : 0.70f;

      const Rgba8 wallC = Mul(wallBase, tierLift);
      const Rgba8 roofC = Mul(roofTint,
                              (1.10f + (rand01(0x33u + static_cast<std::uint32_t>(ti) * 19u) - 0.5f) * 0.08f) * roofLift);
      const Rgba8 rightC = Mul(wallC, rightMul);
      const Rgba8 leftC = Mul(wallC, leftMul);

      // Walls.
      FillTriangle(img, g.rightQ.ax, g.rightQ.ay, g.rightQ.bx, g.rightQ.by, g.rightQ.cx, g.rightQ.cy, rightC);
      FillTriangle(img, g.rightQ.ax, g.rightQ.ay, g.rightQ.cx, g.rightQ.cy, g.rightQ.dx, g.rightQ.dy, rightC);

      FillTriangle(img, g.leftQ.ax, g.leftQ.ay, g.leftQ.bx, g.leftQ.by, g.leftQ.cx, g.leftQ.cy, leftC);
      FillTriangle(img, g.leftQ.ax, g.leftQ.ay, g.leftQ.cx, g.leftQ.cy, g.leftQ.dx, g.leftQ.dy, leftC);

      // Facade material hinting (subtle; windows will be drawn on top).
      paintFacade(g.rightQ, rightC, ti, vol.facadeRight, 0xFACE0001u + static_cast<std::uint32_t>(ti) * 0x101u);
      paintFacade(g.leftQ, leftC, ti, vol.facadeLeft, 0xFACE0002u + static_cast<std::uint32_t>(ti) * 0x101u);

      // Roof. Most buildings use a flat top, but low-rise residential benefits from actual volume.
      {
        const Rgba8 roofA = Mul(roofC, 1.06f);
        const Rgba8 roofB = Mul(roofC, 0.92f);

        const bool pitched = pitchedRoof && topTier && !vol.isAnnex;
        if (pitched) {
          const int lift = std::max(3, tileH / 6 + lvl);
          roofPeakX = vol.basePivotX;
          roofPeakY = ((g.ty[0] + g.ty[2]) / 2) - lift;
          roofHasPeak = true;

          // Pyramid-like hip roof (reads well at this pixel scale).
          const Rgba8 roofN = Mul(roofC, 1.12f);
          const Rgba8 roofE = Mul(roofC, 1.02f);
          const Rgba8 roofS = Mul(roofC, 0.90f);
          const Rgba8 roofW = Mul(roofC, 0.98f);

          FillTriangle(img, g.tx[0], g.ty[0], g.tx[1], g.ty[1], roofPeakX, roofPeakY, roofN);
          FillTriangle(img, g.tx[1], g.ty[1], g.tx[2], g.ty[2], roofPeakX, roofPeakY, roofE);
          FillTriangle(img, g.tx[2], g.ty[2], g.tx[3], g.ty[3], roofPeakX, roofPeakY, roofS);
          FillTriangle(img, g.tx[3], g.ty[3], g.tx[0], g.ty[0], roofPeakX, roofPeakY, roofW);
        } else {
          // Flat roof, split in two for simple shading.
          FillTriangle(img, g.tx[0], g.ty[0], g.tx[1], g.ty[1], g.tx[2], g.ty[2], roofA);
          FillTriangle(img, g.tx[0], g.ty[0], g.tx[2], g.ty[2], g.tx[3], g.ty[3], roofB);
        }

        // Roof texture hint.
        int minX = std::min({g.tx[0], g.tx[1], g.tx[2], g.tx[3]});
        int maxX = std::max({g.tx[0], g.tx[1], g.tx[2], g.tx[3]});
        int minY = std::min({g.ty[0], g.ty[1], g.ty[2], g.ty[3]});
        int maxY = std::max({g.ty[0], g.ty[1], g.ty[2], g.ty[3]});
        if (roofHasPeak) {
          minX = std::min(minX, roofPeakX);
          maxX = std::max(maxX, roofPeakX);
          minY = std::min(minY, roofPeakY);
          maxY = std::max(maxY, roofPeakY);
        }

        minX = std::max(0, minX);
        maxX = std::min(img.width - 1, maxX);
        minY = std::max(0, minY);
        maxY = std::min(img.height - 1, maxY);

        for (int y = minY; y <= maxY; ++y) {
          for (int x = minX; x <= maxX; ++x) {
            bool inside = false;
            if (roofHasPeak) {
              inside = PointInTri(x, y, g.tx[0], g.ty[0], g.tx[1], g.ty[1], roofPeakX, roofPeakY) ||
                       PointInTri(x, y, g.tx[1], g.ty[1], g.tx[2], g.ty[2], roofPeakX, roofPeakY) ||
                       PointInTri(x, y, g.tx[2], g.ty[2], g.tx[3], g.ty[3], roofPeakX, roofPeakY) ||
                       PointInTri(x, y, g.tx[3], g.ty[3], g.tx[0], g.ty[0], roofPeakX, roofPeakY);
            } else {
              inside = PointInDiamond(x, y, g.tx[0], g.ty[0], g.tx[1], g.ty[1], g.tx[2], g.ty[2], g.tx[3], g.ty[3]);
            }
            if (!inside) continue;

            const std::uint32_t h = HashCoords32(x, y, seedv ^ 0xA11CE5u ^ static_cast<std::uint32_t>(ti) * 0x9E3779B9u);

            // Industrial roofs: hint at corrugation/sawtooth.
            if (!roofHasPeak && kind == GfxBuildingKind::Industrial && industrialRoofRidges && topTier && !vol.isAnnex) {
              const std::uint32_t k =
                (static_cast<std::uint32_t>(x) * 3u + static_cast<std::uint32_t>(y) * 5u + (seedv & 0xFFu)) % 7u;
              if (k == 0u) {
                BlendPixel(img, x, y, Mul(roofC, 0.70f));
                continue;
              }
              if (k == 1u) {
                BlendPixel(img, x, y, Mul(roofC, 1.10f));
                continue;
              }
            }

            // Pitched roofs: shingle-ish lines.
            if (roofHasPeak) {
              const std::uint32_t k =
                (static_cast<std::uint32_t>(x) * 2u + static_cast<std::uint32_t>(y) * 7u + (seedv & 0x7Fu)) % 9u;
              if (k == 0u) {
                BlendPixel(img, x, y, Mul(roofC, 0.82f));
                continue;
              }
            }

            // Speckle.
            if ((h & 31u) != 0u) continue;
            BlendPixel(img, x, y, Mul(roofC, 0.86f));
          }
        }

        // Optional rooftop solar hints (flat roofs only).
        if (!roofHasPeak && topTier && !vol.isAnnex && (randFam01(0x50A4Eu) < 0.40f) && (kind != GfxBuildingKind::Industrial || lvl >= 2)) {
          const int panelCount = 2 + static_cast<int>((HashU32(familySeed ^ 0x50A4Eu) & 3u));
          for (int pi = 0; pi < panelCount; ++pi) {
            const std::uint32_t hp = HashU32(seedv ^ 0x50A4Eu ^ static_cast<std::uint32_t>(pi) * 0x9E3779B9u);
            const int px = minX + 2 + static_cast<int>(hp % static_cast<std::uint32_t>(std::max(1, (maxX - minX - 6))));
            const int py = minY + 2 + static_cast<int>((hp >> 8u) % static_cast<std::uint32_t>(std::max(1, (maxY - minY - 6))));
            const int pw = 4 + static_cast<int>((hp >> 16u) & 3u);
            const int ph = 2 + static_cast<int>((hp >> 20u) & 1u);

            for (int yy = 0; yy < ph; ++yy) {
              for (int xx = 0; xx < pw; ++xx) {
                const int x = px + xx;
                const int y = py + yy;
                if (x < minX || x > maxX || y < minY || y > maxY) continue;
                const bool inside = PointInDiamond(x, y, g.tx[0], g.ty[0], g.tx[1], g.ty[1], g.tx[2], g.ty[2], g.tx[3], g.ty[3]);
                if (!inside) continue;
                Rgba8 pc = Rgba8{40, 70, 95, 235};
                pc = Mul(pc, 0.95f);
                BlendPixel(img, x, y, pc, BlendMode::Alpha);
                if (xx == 0 || yy == 0) {
                  BlendPixel(img, x, y, Mul(pc, 1.20f), BlendMode::Alpha);
                }
              }
            }
          }
        }
      }

      // Window density scaling by edge length.
      const float edgeLen = std::sqrt(std::max(0.001f, g.hw * g.hw + g.hh * g.hh));
      const float edgeFrac = std::clamp(edgeLen / std::max(0.001f, baseEdge), 0.45f, 1.0f);

      // Windows.
      paintWindows(g.rightQ, ti, edgeFrac, td.heightPx, totalHPx, 1.0f, 0x900Du + static_cast<std::uint32_t>(ti) * 0x101u, td.windows,
                   vol.facadeRight);
      paintWindows(g.leftQ, ti, edgeFrac, td.heightPx, totalHPx, 0.92f, 0xBEEFu + static_cast<std::uint32_t>(ti) * 0x211u, td.windows,
                   vol.facadeLeft);

      // Fire escapes (residential only, avoid annex to reduce noise).
      if (fireEscape && kind == GfxBuildingKind::Residential && !vol.isAnnex) {
        Rgba8 metal = Mix(Mul(wallBase, 0.42f), accent, 0.25f);
        metal.a = 255;
        PaintFireEscape(img, g.rightQ, rightMul, seedv, 0xF1AEu + static_cast<std::uint32_t>(ti) * 0x101u, lvl, ti,
                        static_cast<int>(vol.tiers.size()), metal);
      }

      // Commercial facade billboards (level >=2) on the main volume.
      if (kind == GfxBuildingKind::Commercial && !vol.isAnnex && lvl >= 2) {
        const bool neon = (randFam01(0xB111u) < 0.60f);
        if ((ti == 0) || (lvl == 3 && ti == 1)) {
          paintBillboardOnFace(g.rightQ, rightMul, 0xB111u + static_cast<std::uint32_t>(ti) * 0x33u, neon);
        }
      }

      // Ground-level details (doors / storefronts) on the base tier.
      if (ti == 0) {
        if (kind == GfxBuildingKind::Industrial && !vol.isAnnex) {
          paintLoadingDoor(g.rightQ, 0x10ADu + static_cast<std::uint32_t>(variant) * 0x101u);
        }
        if (kind == GfxBuildingKind::Commercial && lvl == 1 && !vol.isAnnex) {
          paintStorefront(g.rightQ, 0x570Fu + static_cast<std::uint32_t>(variant) * 0x33u);
        }
      }

      // Silhouette outlines for readability.
      {
        const Rgba8 ol{0, 0, 0, 150};
        // Roof perimeter.
        StrokeLine(img, g.tx[0], g.ty[0], g.tx[1], g.ty[1], ol);
        StrokeLine(img, g.tx[1], g.ty[1], g.tx[2], g.ty[2], ol);
        StrokeLine(img, g.tx[2], g.ty[2], g.tx[3], g.ty[3], ol);
        StrokeLine(img, g.tx[3], g.ty[3], g.tx[0], g.ty[0], ol);

        if (roofHasPeak) {
          // Extra silhouette lines help pitched roofs read as 3D.
          StrokeLine(img, roofPeakX, roofPeakY, g.tx[0], g.ty[0], ol);
          StrokeLine(img, roofPeakX, roofPeakY, g.tx[1], g.ty[1], ol);
          StrokeLine(img, roofPeakX, roofPeakY, g.tx[2], g.ty[2], ol);
          StrokeLine(img, roofPeakX, roofPeakY, g.tx[3], g.ty[3], ol);
        }

        // Visible vertical edges.
        StrokeLine(img, g.bx[1], g.by[1], g.tx[1], g.ty[1], ol);
        StrokeLine(img, g.bx[3], g.by[3], g.tx[3], g.ty[3], ol);
        StrokeLine(img, g.bx[2], g.by[2], g.tx[2], g.ty[2], ol);
      }

      // -----------------------------
      // Rooftop / facade details (main volume only)
      // -----------------------------
      if (!vol.isAnnex && vol.allowRooftopDetails) {
        if (kind == GfxBuildingKind::Residential && topTier) {
          // Chimney
          const int hwPx = std::abs(g.tx[1] - vol.basePivotX);
          const int hhPx = std::abs(g.by[2] - baseY);
          const int cx = vol.basePivotX + static_cast<int>(std::lround(static_cast<float>(hwPx) * (0.22f + 0.20f * rand01(0x701u))));
          const int cy = g.ty[0] + static_cast<int>(std::lround(static_cast<float>(hhPx) * (0.65f + 0.18f * rand01(0x702u))));
          const int chH = std::max(10, tileH / 2 + lvl * 2);
          drawStack(cx, cy, 3, chH, Rgba8{70, 70, 70, 230}, /*emissiveTop=*/false, Rgba8{});

          // Water tank for taller residential.
          if (lvl >= 2 && !roofHasPeak && randFam01(0x77A6u) < 0.40f) {
            const int tx = vol.basePivotX + static_cast<int>(std::lround(static_cast<float>(hwPx) * (rand01(0x77A7u) - 0.5f) * 0.60f));
            const int ty = g.ty[0] + static_cast<int>(std::lround(static_cast<float>(hhPx) * (0.40f + 0.20f * rand01(0x77A8u))));
            drawStack(tx, ty, 5, std::max(8, tileH / 2), Rgba8{95, 95, 100, 220}, /*emissiveTop=*/false, Rgba8{});
          }
        }

        if (kind == GfxBuildingKind::Industrial && ti == 0) {
          // A couple of stacks on the main hall roof (kept away from the center so a top office tier can sit there).
          const int hwPx = std::abs(g.tx[1] - vol.basePivotX);
          const int hhPx = std::abs(g.by[2] - baseY);
          const int roofBaseY = g.ty[0] + static_cast<int>(std::lround(static_cast<float>(hhPx) * 0.62f));

          const int sCount = (lvl >= 3) ? 2 : 1;
          for (int si = 0; si < sCount; ++si) {
            const float side = (si == 0) ? -1.0f : 1.0f;
            const float off = 0.35f + 0.10f * rand01(0x810u + static_cast<std::uint32_t>(si));
            const int cx = vol.basePivotX + static_cast<int>(std::lround(side * static_cast<float>(hwPx) * off));
            const int h = std::max(tileH,
                                   static_cast<int>(std::lround(static_cast<float>(tileH) *
                                                                (1.20f + 0.55f * rand01(0x820u + static_cast<std::uint32_t>(si)) +
                                                                 0.22f * static_cast<float>(lvl)))));
            drawStack(cx, roofBaseY, 4, h, Rgba8{110, 110, 115, 230}, /*emissiveTop=*/(lvl >= 2),
                      Rgba8{255, 150, 80, 220});
          }
        }

        if (kind == GfxBuildingKind::Commercial && topTier) {
          const int hwPx = std::abs(g.tx[1] - vol.basePivotX);
          const int hhPx = std::abs(g.by[2] - baseY);

          // Rooftop HVAC blocks (non-emissive).
          const int hvCount = (lvl == 1) ? 1 : (lvl == 2 ? 2 : 3);
          for (int i = 0; i < hvCount; ++i) {
            const std::uint32_t hs = HashU32(seedv ^ 0xACACACu ^ static_cast<std::uint32_t>(i) * 0x9E3779B9u);
            const float fx = 0.25f + 0.50f * Frac01(hs);
            const float fy = 0.35f + 0.35f * Frac01(hs ^ 0xBADC0DEu);

            const int cx = vol.basePivotX + static_cast<int>(std::lround((fx - 0.5f) * static_cast<float>(hwPx) * 1.10f));
            const int cy = g.ty[0] + static_cast<int>(std::lround(fy * static_cast<float>(hhPx) * 0.95f));
            const int w = 5 + static_cast<int>(hs & 3u);
            const int h = 4 + static_cast<int>((hs >> 3u) & 1u);

            const Rgba8 unit{170, 170, 175, 215};
            FillRect(img, cx - w / 2, cy - h, cx + (w - 1) / 2, cy, unit);
            FillRect(img, cx - w / 2, cy - h, cx - w / 2 + w / 3, cy, Mul(unit, 0.78f));
          }

          // Antenna / spire for tall commercial buildings.
          if (commercialSpire) {
            const std::uint32_t hs = HashU32(seedv ^ 0xC0FFEEu);
            const float fx = 0.40f + 0.20f * Frac01(hs);
            const int cx = vol.basePivotX + static_cast<int>(std::lround((fx - 0.5f) * static_cast<float>(hwPx) * 0.90f));
            const int cy = (g.ty[0] + g.ty[2]) / 2;
            const int spH = std::max(tileH, static_cast<int>(std::lround(static_cast<float>(tileH) * 1.85f)));
            drawStack(cx, cy, 2 + static_cast<int>(hs & 1u), spH, Rgba8{65, 65, 70, 230}, /*emissiveTop=*/true,
                      Rgba8{255, 90, 90, 230});
          }

          // Rooftop sign (lvl >= 2): deterministic text + emissive glow.
          if (lvl >= 2) {
            const std::uint32_t hs = HashU32(familySeed ^ 0x51A7E1u);
            const int hue = static_cast<int>(hs & 3u);

            Rgba8 neon = litTint;
            if (hue == 1) neon = Rgba8{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 235};
            if (hue == 2) neon = Rgba8{200, 100, 255, 235};
            if (hue == 3) neon = Rgba8{90, 220, 255, 235};

            const int boardHalf = std::max(10, static_cast<int>(std::lround(static_cast<float>(hwPx) * 0.70f)));
            int sx = vol.basePivotX - boardHalf;
            int ex = vol.basePivotX + boardHalf;
            int sy = g.ty[0] + 2;
            const int boardH = 9 + (lvl == 3 ? 2 : 0);
            int ey = sy + boardH - 1;

            sx = std::clamp(sx, 0, img.width - 1);
            ex = std::clamp(ex, 0, img.width - 1);
            sy = std::clamp(sy, 0, img.height - 1);
            ey = std::clamp(ey, 0, img.height - 1);

            if (sx < ex && sy < ey) {
              // Board background in albedo.
              Rgba8 board = Mul(Mix(roofC, accent, 0.30f), 0.52f);
              board.a = 225;
              FillRect(img, sx, sy, ex, ey, board);

              // Outline.
              const Rgba8 bol{0, 0, 0, 145};
              StrokeLine(img, sx, sy, ex, sy, bol);
              StrokeLine(img, sx, ey, ex, ey, bol);
              StrokeLine(img, sx, sy, sx, ey, bol);
              StrokeLine(img, ex, sy, ex, ey, bol);

              // Determine how many chars fit.
              const int pad = 2;
              const int spacing = 1;
              const int availW = std::max(0, (ex - sx + 1) - pad * 2);
              const int maxChars =
                std::max(1, (availW + spacing) / (gfx::Font5x7GlyphW() + spacing));

              std::string name = MakeBrandName(hs ^ 0xC0FFEEu, maxChars);

              const int textW = gfx::MeasureTextWidth5x7(name, 1, spacing);
              const int textH = gfx::MeasureTextHeight5x7(1);
              const int tx = vol.basePivotX - textW / 2;
              const int ty = sy + (boardH - textH) / 2;

              Rgba8 ink{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 235};
              Rgba8 outline{0, 0, 0, 175};
              gfx::DrawText5x7Outlined(img, tx, ty, name, ink, outline, 1, spacing, BlendMode::Alpha);

              if (cfgIn.includeEmissive && !emit.rgba.empty()) {
                RgbaImage glow;
                glow.width = emit.width;
                glow.height = emit.height;
                glow.rgba.assign(emit.rgba.size(), 0u);

                Rgba8 glowCol = neon;
                glowCol.a = 220;
                gfx::DrawText5x7(glow, tx, ty, name, glowCol, 1, spacing, BlendMode::Additive);

                const int br = 2 + (lvl == 3 ? 1 : 0);
                gfx::BoxBlurPremultiplied(glow, br);
                gfx::CompositeImage(emit, glow, BlendMode::Additive);

                // Crisp core.
                Rgba8 core = neon;
                core.a = 240;
                gfx::DrawText5x7(emit, tx, ty, name, core, 1, spacing, BlendMode::Additive);
              }
            }
          }
        }
      }

      cumH += td.heightPx;
    }
  }

  // Add an AO + rim-lighting pass to improve depth cues at small pixel scale.
  ApplyDirectionalAORim(img, seedv, pivotY, tileH, kind);

  out.color = std::move(img);
  out.emissive = std::move(emit);
  return true;
}

} // namespace isocity
