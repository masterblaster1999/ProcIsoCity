#include "isocity/GfxFacilities.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/GfxText.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>

namespace isocity {

namespace {

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

// Small 32-bit mix (Murmur3 finalizer-like).
inline std::uint32_t HashU32(std::uint32_t v)
{
  v ^= v >> 16;
  v *= 0x7FEB352Du;
  v ^= v >> 15;
  v *= 0x846CA68Bu;
  v ^= v >> 16;
  return v;
}

struct FaceQuad {
  int ax = 0, ay = 0;
  int bx = 0, by = 0;
  int cx = 0, cy = 0;
  int dx = 0, dy = 0;
};

inline bool Barycentric(int ax, int ay, int bx, int by, int cx, int cy, int px, int py, float& outU, float& outV,
                        float& outW)
{
  const float v0x = static_cast<float>(bx - ax);
  const float v0y = static_cast<float>(by - ay);
  const float v1x = static_cast<float>(cx - ax);
  const float v1y = static_cast<float>(cy - ay);
  const float v2x = static_cast<float>(px - ax);
  const float v2y = static_cast<float>(py - ay);

  const float d00 = v0x * v0x + v0y * v0y;
  const float d01 = v0x * v1x + v0y * v1y;
  const float d11 = v1x * v1x + v1y * v1y;
  const float d20 = v2x * v0x + v2y * v0y;
  const float d21 = v2x * v1x + v2y * v1y;

  const float denom = d00 * d11 - d01 * d01;
  if (std::fabs(denom) < 1.0e-10f) return false;
  const float invDen = 1.0f / denom;

  outV = (d11 * d20 - d01 * d21) * invDen;
  outW = (d00 * d21 - d01 * d20) * invDen;
  outU = 1.0f - outV - outW;
  return true;
}

// Map a point on a face quad to local (s,t) in [0,1]. Returns false if outside.
inline bool FaceLocalST(const FaceQuad& q, int x, int y, float& outS, float& outT)
{
  // Two triangles: (a,b,c) and (a,c,d)
  float u = 0.0f, v = 0.0f, w = 0.0f;
  if (Barycentric(q.ax, q.ay, q.bx, q.by, q.cx, q.cy, x, y, u, v, w)) {
    if (u >= -0.001f && v >= -0.001f && w >= -0.001f) {
      // Interpolate between top edge (a->b) and bottom edge (d->c).
      // We treat barycentric on (a,b,c) to blend s along a->b and t downwards.
      outS = std::clamp(v + w, 0.0f, 1.0f);
      // t: 0 at top edge, 1 at bottom edge.
      outT = std::clamp(w, 0.0f, 1.0f);
      return true;
    }
  }

  if (Barycentric(q.ax, q.ay, q.cx, q.cy, q.dx, q.dy, x, y, u, v, w)) {
    if (u >= -0.001f && v >= -0.001f && w >= -0.001f) {
      // Triangle (a,c,d)
      outS = std::clamp(v, 0.0f, 1.0f);
      outT = std::clamp(v + w, 0.0f, 1.0f);
      return true;
    }
  }

  return false;
}

struct Diamond4 {
  int tx = 0, ty = 0;
  int rx = 0, ry = 0;
  int bx = 0, by = 0;
  int lx = 0, ly = 0;
};

inline Diamond4 MakeDiamond(int cx, int cy, int hw, int hh)
{
  Diamond4 d;
  d.tx = cx;
  d.ty = cy - hh;
  d.rx = cx + hw;
  d.ry = cy;
  d.bx = cx;
  d.by = cy + hh;
  d.lx = cx - hw;
  d.ly = cy;
  return d;
}

inline Diamond4 OffsetY(Diamond4 d, int dy)
{
  d.ty += dy;
  d.ry += dy;
  d.by += dy;
  d.ly += dy;
  return d;
}

inline void FillQuad(RgbaImage& img, int ax, int ay, int bx, int by, int cx, int cy, int dx, int dy, Rgba8 c)
{
  gfx::FillTriangle(img, ax, ay, bx, by, cx, cy, c);
  gfx::FillTriangle(img, ax, ay, cx, cy, dx, dy, c);
}

inline void FillDiamond(RgbaImage& img, const Diamond4& d, Rgba8 c)
{
  gfx::FillTriangle(img, d.tx, d.ty, d.rx, d.ry, d.bx, d.by, c);
  gfx::FillTriangle(img, d.tx, d.ty, d.bx, d.by, d.lx, d.ly, c);
}

inline bool PointInTri(int ax, int ay, int bx, int by, int cx, int cy, int px, int py)
{
  const int w0 = gfx::EdgeFn(ax, ay, bx, by, px, py);
  const int w1 = gfx::EdgeFn(bx, by, cx, cy, px, py);
  const int w2 = gfx::EdgeFn(cx, cy, ax, ay, px, py);
  const bool hasNeg = (w0 < 0) || (w1 < 0) || (w2 < 0);
  const bool hasPos = (w0 > 0) || (w1 > 0) || (w2 > 0);
  return !(hasNeg && hasPos);
}

inline bool PointInDiamond(const Diamond4& d, int x, int y)
{
  if (PointInTri(d.tx, d.ty, d.rx, d.ry, d.bx, d.by, x, y)) return true;
  if (PointInTri(d.tx, d.ty, d.bx, d.by, d.lx, d.ly, x, y)) return true;
  return false;
}

inline std::string Lower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

} // namespace

const char* GfxFacilityKindName(GfxFacilityKind k)
{
  switch (k) {
    case GfxFacilityKind::Education: return "education";
    case GfxFacilityKind::Health: return "health";
    case GfxFacilityKind::Police: return "police";
    case GfxFacilityKind::Fire: return "fire";
    default: return "education";
  }
}

bool ParseGfxFacilityKind(const std::string& s, GfxFacilityKind& out)
{
  const std::string v = Lower(s);
  if (v == "education" || v == "edu" || v == "school") {
    out = GfxFacilityKind::Education;
    return true;
  }
  if (v == "health" || v == "clinic" || v == "hospital") {
    out = GfxFacilityKind::Health;
    return true;
  }
  if (v == "police" || v == "pd") {
    out = GfxFacilityKind::Police;
    return true;
  }
  if (v == "fire" || v == "fd") {
    out = GfxFacilityKind::Fire;
    return true;
  }
  return false;
}

bool GenerateGfxFacilitySprite(GfxFacilityKind kind, int levelIn, int variant, std::uint32_t seed,
                               const GfxFacilitiesConfig& cfgIn, const GfxPalette& pal, GfxFacilitySprite& out,
                               std::string& outError)
{
  const int lvl = std::clamp(levelIn, 1, 3);
  if (cfgIn.tileW <= 0 || cfgIn.tileH <= 0) {
    outError = "invalid tile size";
    return false;
  }

  const int tileW = cfgIn.tileW;
  const int tileH = cfgIn.tileH;
  const int halfW = tileW / 2;
  const int halfH = tileH / 2;

  // Deterministic per-facility RNG.
  const std::uint32_t seedv = seed ^ 0xFAC11E77u ^ (static_cast<std::uint32_t>(variant) * 0x9E3779B9u) ^
                              (static_cast<std::uint32_t>(lvl) * 0x85EBCA6Bu) ^
                              (static_cast<std::uint32_t>(kind) * 0xC2B2AE35u);

  auto rand01 = [&](std::uint32_t salt) { return Frac01(HashU32(seedv ^ salt)); };

  // Height budget (in pixels) for the main block.
  float hMul = 2.0f;
  switch (kind) {
    case GfxFacilityKind::Education: hMul = 1.85f + 0.65f * static_cast<float>(lvl); break;
    case GfxFacilityKind::Health: hMul = 2.05f + 0.70f * static_cast<float>(lvl); break;
    case GfxFacilityKind::Police: hMul = 1.90f + 0.65f * static_cast<float>(lvl); break;
    case GfxFacilityKind::Fire: hMul = 1.80f + 0.60f * static_cast<float>(lvl); break;
    default: break;
  }

  const int baseHeightPx = std::max(12, static_cast<int>(std::lround(static_cast<float>(tileH) * hMul)));

  // Optional small tower (education + police tend to have it).
  const bool wantTower =
      (kind == GfxFacilityKind::Education && lvl >= 2 && rand01(0x51A1C001u) > 0.25f) ||
      (kind == GfxFacilityKind::Police && lvl >= 2 && rand01(0x51A1C002u) > 0.40f);

  const int towerExtraPx = wantTower ? std::max(8, static_cast<int>(std::lround(static_cast<float>(tileH) * 0.8f))) : 0;

  const int marginTop = 4;
  const int marginBot = 3;
  const int maxHeightPx = baseHeightPx + towerExtraPx;
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

  // Footprint shrink: facilities tend to occupy most of the tile.
  const float shrink = std::clamp(0.92f - 0.10f * rand01(0x51A1D1E3u), 0.80f, 0.95f);
  const int hw = std::max(1, static_cast<int>(std::lround(static_cast<float>(halfW) * shrink)));
  const int hh = std::max(1, static_cast<int>(std::lround(static_cast<float>(halfH) * shrink)));

  Diamond4 base = MakeDiamond(pivotX, pivotY, hw, hh);
  Diamond4 roof = OffsetY(base, -baseHeightPx);

  // ---------------------------------------------------------------------------
  // Materials
  // ---------------------------------------------------------------------------

  Rgba8 wallTint{220, 220, 220, 255};
  Rgba8 roofTint{210, 210, 210, 255};
  Rgba8 accent{pal.roadMarkYellow.r, pal.roadMarkYellow.g, pal.roadMarkYellow.b, 255};
  Rgba8 signBg{pal.roadMarkWhite.r, pal.roadMarkWhite.g, pal.roadMarkWhite.b, 255};
  Rgba8 signFg{20, 20, 20, 255};
  std::string_view signText;
  bool signGlow = false;

  Rgba8 fireRed = gfx::Lerp(pal.overlayResidential, Rgba8{220, 50, 40, 255}, 0.60f);
  Rgba8 medRed{220, 60, 60, 255};

  switch (kind) {
    case GfxFacilityKind::Education: {
      roofTint = pal.overlayResidential;
      wallTint = gfx::Lerp(pal.overlayResidential, Rgba8{226, 222, 214, 255}, 0.68f);
      accent = gfx::Lerp(pal.roadMarkYellow, pal.roadMarkWhite, 0.35f);
      signBg = gfx::Lerp(pal.roadMarkWhite, accent, 0.20f);
      signFg = gfx::Mul(pal.overlayResidential, 0.75f);
      signText = "SCH";
      signGlow = false;
    } break;
    case GfxFacilityKind::Health: {
      roofTint = gfx::Lerp(pal.overlayCommercial, Rgba8{235, 235, 235, 255}, 0.82f);
      wallTint = gfx::Lerp(Rgba8{238, 238, 242, 255}, pal.sand, 0.18f);
      signBg = pal.roadMarkWhite;
      signFg = medRed;
      signText = "+";
      signGlow = true;
    } break;
    case GfxFacilityKind::Police: {
      roofTint = gfx::Lerp(pal.overlayCommercial, Rgba8{210, 210, 210, 255}, 0.25f);
      wallTint = gfx::Lerp(pal.overlayCommercial, Rgba8{226, 226, 226, 255}, 0.64f);
      accent = gfx::Mul(pal.overlayCommercial, 1.05f);
      signBg = gfx::Mul(pal.overlayCommercial, 0.85f);
      signFg = pal.roadMarkWhite;
      signText = "PD";
      signGlow = true;
    } break;
    case GfxFacilityKind::Fire: {
      roofTint = gfx::Lerp(pal.overlayIndustrial, fireRed, 0.55f);
      wallTint = gfx::Lerp(pal.overlayIndustrial, Rgba8{228, 224, 220, 255}, 0.62f);
      accent = fireRed;
      signBg = gfx::Mul(fireRed, 0.92f);
      signFg = pal.roadMarkWhite;
      signText = "FD";
      signGlow = true;
    } break;
    default: break;
  }

  Rgba8 leftWall = gfx::Mul(wallTint, 1.06f);
  Rgba8 rightWall = gfx::Mul(wallTint, 0.86f);
  leftWall.a = 255;
  rightWall.a = 255;

  Rgba8 roofBase = gfx::Mul(roofTint, 0.98f);
  roofBase.a = 255;

  Rgba8 outline = gfx::Mul(wallTint, 0.45f);
  outline.a = 150;

  // ---------------------------------------------------------------------------
  // Base block geometry (walls then roof).
  // ---------------------------------------------------------------------------

  // Left wall (roof.l -> roof.b -> base.b -> base.l)
  FillQuad(img, roof.lx, roof.ly, roof.bx, roof.by, base.bx, base.by, base.lx, base.ly, leftWall);
  // Right wall (roof.r -> roof.b -> base.b -> base.r)
  FillQuad(img, roof.rx, roof.ry, roof.bx, roof.by, base.bx, base.by, base.rx, base.ry, rightWall);
  // Roof
  FillDiamond(img, roof, roofBase);

  // Face quads for local mapping.
  FaceQuad qRight{roof.rx, roof.ry, roof.bx, roof.by, base.bx, base.by, base.rx, base.ry};
  FaceQuad qLeft{roof.bx, roof.by, roof.lx, roof.ly, base.lx, base.ly, base.bx, base.by};

  // ---------------------------------------------------------------------------
  // Roof shading pass (adds gables / noise / rooftop details).
  // ---------------------------------------------------------------------------

  {
    const int minX = std::max(0, std::min({roof.tx, roof.rx, roof.bx, roof.lx}) - 1);
    const int maxX = std::min(img.width - 1, std::max({roof.tx, roof.rx, roof.bx, roof.lx}) + 1);
    const int minY = std::max(0, std::min({roof.ty, roof.ry, roof.by, roof.ly}) - 1);
    const int maxY = std::min(img.height - 1, std::max({roof.ty, roof.ry, roof.by, roof.ly}) + 1);

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        if (!PointInDiamond(roof, x, y)) continue;

        float shade = 1.0f;
        if (kind == GfxFacilityKind::Education) {
          // Simple gable: left half darker, right half lighter.
          shade = (x < pivotX) ? 0.92f : 1.08f;
          // Ridge highlight.
          if (std::abs(x - pivotX) <= 1) shade = 1.15f;
        } else {
          // Flat roofs get a subtle gradient towards the back.
          const float ty = static_cast<float>(y - (pivotY - baseHeightPx));
          const float ny = (hh > 0) ? (ty / static_cast<float>(hh)) : 0.0f; // roughly -1..1
          shade = 1.02f - 0.08f * ny;
        }

        const std::uint32_t hv = HashCoords32(x, y, seedv ^ 0xA11CE0F0u);
        const int n = static_cast<int>((hv & 7u)) - 3; // -3..3

        Rgba8 c = gfx::Mul(roofTint, shade);
        c = gfx::Add(c, n, n, n);
        c.a = 255;
        gfx::BlendPixel(img, x, y, c);
      }
    }

    // Rooftop units (vents/AC) for non-gabled roofs.
    if (kind != GfxFacilityKind::Education) {
      const int cx = pivotX;
      const int cy = pivotY - baseHeightPx - static_cast<int>(std::lround(static_cast<float>(hh) * 0.10f));
      const int uW = std::max(6, tileW / 8);
      const int uH = std::max(3, tileH / 10);
      const int ux0 = cx - uW / 2;
      const int uy0 = cy - uH / 2;
      gfx::FillRect(img, ux0, uy0, ux0 + uW, uy0 + uH, gfx::Mul(roofTint, 0.72f));
      gfx::StrokeLine(img, ux0, uy0, ux0 + uW, uy0, outline);
      gfx::StrokeLine(img, ux0, uy0 + uH, ux0 + uW, uy0 + uH, outline);
    }
  }

  // ---------------------------------------------------------------------------
  // Face detail painter (windows / stripes / doors).
  // ---------------------------------------------------------------------------

  auto paintFace = [&](const FaceQuad& fq, bool isRight, Rgba8 baseWallShade) {
    const int minX = std::max(0, std::min({fq.ax, fq.bx, fq.cx, fq.dx}) - 1);
    const int maxX = std::min(img.width - 1, std::max({fq.ax, fq.bx, fq.cx, fq.dx}) + 1);
    const int minY = std::max(0, std::min({fq.ay, fq.by, fq.cy, fq.dy}) - 1);
    const int maxY = std::min(img.height - 1, std::max({fq.ay, fq.by, fq.cy, fq.dy}) + 1);

    // Window grid parameters.
    int cols = 4;
    int floors = 2 + lvl;
    float litChance = 0.35f + 0.10f * static_cast<float>(lvl);

    if (kind == GfxFacilityKind::Education) {
      cols = 3;
      floors = 2 + lvl;
      litChance = 0.25f;
    } else if (kind == GfxFacilityKind::Health) {
      cols = 4;
      floors = 3 + lvl;
      litChance = 0.45f;
    } else if (kind == GfxFacilityKind::Police) {
      cols = 4;
      floors = 2 + lvl;
      litChance = 0.35f;
    } else if (kind == GfxFacilityKind::Fire) {
      cols = 3;
      floors = 2 + lvl;
      litChance = 0.30f;
    }

    const Rgba8 glass = gfx::Lerp(pal.overlayCommercial, pal.roadMarkWhite, 0.70f);
    Rgba8 warm = pal.roadMarkYellow;
    warm.a = 230;

    for (int y = minY; y <= maxY; ++y) {
      for (int x = minX; x <= maxX; ++x) {
        float s = 0.0f, t = 0.0f;
        if (!FaceLocalST(fq, x, y, s, t)) continue;

        // Base shade with a little vertical gradient.
        float g = 0.88f + 0.18f * (1.0f - t);
        Rgba8 c = gfx::Mul(baseWallShade, g);

        // Low-amplitude wall noise.
        const std::uint32_t hv = HashCoords32(x, y, seedv ^ (isRight ? 0xFACEu : 0xBEEFu));
        const int n = static_cast<int>(hv & 7u) - 3;
        c = gfx::Add(c, n, n, n);
        c.a = 255;

        // Stripes / bands.
        if (kind == GfxFacilityKind::Health && t > 0.78f && t < 0.84f) {
          c = gfx::Lerp(c, medRed, 0.35f);
        }
        if (kind == GfxFacilityKind::Police && t > 0.82f && t < 0.88f) {
          c = gfx::Lerp(c, accent, 0.45f);
        }

        bool reserved = false;

        // Fire station garage door: right face, wide door near the bottom.
        if (kind == GfxFacilityKind::Fire && isRight && t > 0.55f && t < 0.97f && s > 0.10f && s < 0.90f) {
          reserved = true;
          float ss = (s - 0.10f) / 0.80f;
          float tt = (t - 0.55f) / 0.42f;

          Rgba8 door = gfx::Lerp(wallTint, roofTint, 0.15f);
          door = gfx::Mul(door, 0.95f);

          // Vertical panels.
          if (std::fmod(ss * 6.0f, 1.0f) < 0.07f) door = gfx::Mul(door, 0.78f);
          // Horizontal ribs.
          if (std::fmod(tt * 5.0f, 1.0f) < 0.08f) door = gfx::Mul(door, 0.84f);

          // Small window band at the top of the door.
          if (tt < 0.20f && ss > 0.12f && ss < 0.88f) {
            if (std::fmod(ss * 8.0f, 1.0f) > 0.18f) {
              door = gfx::Lerp(door, glass, 0.55f);
              if (cfgIn.includeEmissive && (hv & 0x3u) == 0u) {
                gfx::BlendPixel(emit, x, y, warm);
              }
            }
          }

          c = door;
        }

        // Education entrance door (either face, centered).
        if (kind == GfxFacilityKind::Education && t > 0.72f && t < 0.96f && s > 0.42f && s < 0.58f) {
          reserved = true;
          Rgba8 door = gfx::Mul(accent, 0.75f);
          if (std::fmod((t - 0.72f) * 10.0f, 1.0f) < 0.08f) door = gfx::Mul(door, 0.85f);
          c = door;
        }

        // Windows.
        if (!reserved && t > 0.10f && t < 0.90f) {
          const float cs = s * static_cast<float>(cols);
          const float ct = t * static_cast<float>(floors);
          const int ic = static_cast<int>(std::floor(cs));
          const int it = static_cast<int>(std::floor(ct));
          const float fs = cs - static_cast<float>(ic);
          const float ft = ct - static_cast<float>(it);

          // Window margins (vary a bit by kind).
          float mx = 0.18f;
          float my = 0.22f;
          if (kind == GfxFacilityKind::Health) {
            mx = 0.20f;
            my = 0.25f;
          }

          if (fs > mx && fs < 1.0f - mx && ft > my && ft < 1.0f - my) {
            // Per-window lit decision (deterministic by cell).
            const std::uint32_t wh = HashCoords32(ic, it, seedv ^ 0xA11CE0F1u);
            const bool lit = (Frac01(wh) < litChance);

            Rgba8 wcol = gfx::Lerp(c, glass, 0.70f);
            if (lit) wcol = gfx::Lerp(wcol, pal.roadMarkWhite, 0.20f);
            wcol.a = 255;
            c = wcol;

            if (cfgIn.includeEmissive && lit) {
              gfx::BlendPixel(emit, x, y, warm);
            }

            // Window mullions.
            if (fs < mx + 0.05f || fs > (1.0f - mx) - 0.05f || ft < my + 0.05f || ft > (1.0f - my) - 0.05f) {
              c = gfx::Mul(c, 0.80f);
            }
          }
        }

        gfx::BlendPixel(img, x, y, c);
      }
    }
  };

  paintFace(qLeft, false, leftWall);
  paintFace(qRight, true, rightWall);

  // ---------------------------------------------------------------------------
  // Optional tower (small second block on the roof).
  // ---------------------------------------------------------------------------

  if (wantTower) {
    const int tHw = std::max(4, static_cast<int>(std::lround(static_cast<float>(hw) * 0.33f)));
    const int tHh = std::max(3, static_cast<int>(std::lround(static_cast<float>(hh) * 0.33f)));
    const int tCx = pivotX;
    const int tCy = (pivotY - baseHeightPx) - static_cast<int>(std::lround(static_cast<float>(hh) * 0.35f));

    Diamond4 tBase = MakeDiamond(tCx, tCy, tHw, tHh);
    Diamond4 tRoof = OffsetY(tBase, -towerExtraPx);

    const Rgba8 tWallL = gfx::Mul(wallTint, 1.02f);
    const Rgba8 tWallR = gfx::Mul(wallTint, 0.86f);
    const Rgba8 tRoofC = gfx::Mul(roofTint, 1.05f);

    FillQuad(img, tRoof.lx, tRoof.ly, tRoof.bx, tRoof.by, tBase.bx, tBase.by, tBase.lx, tBase.ly, tWallL);
    FillQuad(img, tRoof.rx, tRoof.ry, tRoof.bx, tRoof.by, tBase.bx, tBase.by, tBase.rx, tBase.ry, tWallR);
    FillDiamond(img, tRoof, tRoofC);

    // Flagpole (education) or antenna (police).
    const int poleX = tRoof.tx;
    const int poleY0 = tRoof.ty;
    const int poleY1 = std::max(0, poleY0 - std::max(8, tileH / 2));
    gfx::StrokeLine(img, poleX, poleY0, poleX, poleY1, outline);

    if (kind == GfxFacilityKind::Education) {
      // Small flag.
      const int fx0 = poleX;
      const int fy0 = poleY1 + 1;
      const int fx1 = poleX + std::max(6, tileW / 10);
      const int fy1 = poleY1 + std::max(3, tileH / 10);
      gfx::FillTriangle(img, fx0, fy0, fx1, fy0, fx0, fy1, accent);
      gfx::StrokeLine(img, fx0, fy0, fx1, fy0, outline);
    }
  }

  // ---------------------------------------------------------------------------
  // Roof-mounted signage.
  // ---------------------------------------------------------------------------

  if (!signText.empty()) {
    const int scale = (tileW >= 64) ? 2 : 1;
    const int textW = gfx::MeasureTextWidth5x7(signText, scale, 1);
    const int textH = gfx::MeasureTextHeight5x7(scale);

    const int padX = 3;
    const int padY = 2;
    const int signW = textW + padX * 2;
    const int signH = textH + padY * 2;

    // Anchor near the front edge of the roof.
    const int cx = roof.bx;
    const int cy = roof.by - std::max(2, tileH / 10);
    const int x0 = std::clamp(cx - signW / 2, 0, img.width - signW);
    const int y0 = std::clamp(cy - signH, 0, img.height - signH);

    gfx::FillRect(img, x0, y0, x0 + signW - 1, y0 + signH - 1, signBg);
    gfx::StrokeLine(img, x0, y0, x0 + signW - 1, y0, outline);
    gfx::StrokeLine(img, x0, y0, x0, y0 + signH - 1, outline);
    gfx::StrokeLine(img, x0 + signW - 1, y0, x0 + signW - 1, y0 + signH - 1, outline);
    gfx::StrokeLine(img, x0, y0 + signH - 1, x0 + signW - 1, y0 + signH - 1, outline);

    const int tx = x0 + padX;
    const int ty = y0 + padY;

    // Text with a tiny outline for readability.
    gfx::DrawText5x7Outlined(img, tx, ty, signText, signFg, gfx::Mul(signBg, 0.45f), scale, 1);

    if (cfgIn.includeEmissive && signGlow) {
      Rgba8 glow = signFg;
      glow.a = 220;
      gfx::DrawText5x7(emit, tx, ty, signText, glow, scale, 1);
      // Soft halo.
      gfx::FillCircleSoft(emit, static_cast<float>(cx), static_cast<float>(cy - signH / 2),
                          static_cast<float>(std::max(6, signW / 2)), 2.0f,
                          Rgba8{glow.r, glow.g, glow.b, static_cast<std::uint8_t>(90)});
    }
  }

  // ---------------------------------------------------------------------------
  // Extra recognizable rooftop markers.
  // ---------------------------------------------------------------------------

  if (kind == GfxFacilityKind::Health && lvl == 3) {
    // Helipad diamond with an 'H'.
    const int pHw = std::max(6, static_cast<int>(std::lround(static_cast<float>(hw) * 0.38f)));
    const int pHh = std::max(4, static_cast<int>(std::lround(static_cast<float>(hh) * 0.38f)));
    const int pCx = pivotX;
    const int pCy = pivotY - baseHeightPx - static_cast<int>(std::lround(static_cast<float>(hh) * 0.10f));

    Diamond4 pad = MakeDiamond(pCx, pCy, pHw, pHh);
    Rgba8 padFill = gfx::Mul(pal.roadAsphalt2, 0.85f);
    padFill.a = 220;
    FillDiamond(img, pad, padFill);

    gfx::StrokeLine(img, pad.tx, pad.ty, pad.rx, pad.ry, outline);
    gfx::StrokeLine(img, pad.rx, pad.ry, pad.bx, pad.by, outline);
    gfx::StrokeLine(img, pad.bx, pad.by, pad.lx, pad.ly, outline);
    gfx::StrokeLine(img, pad.lx, pad.ly, pad.tx, pad.ty, outline);

    const int scale = (tileW >= 64) ? 2 : 1;
    const int tw = gfx::MeasureTextWidth5x7("H", scale, 1);
    const int th = gfx::MeasureTextHeight5x7(scale);
    gfx::DrawText5x7Outlined(img, pCx - tw / 2, pCy - th / 2, "H", pal.roadMarkWhite, outline, scale, 1);

    if (cfgIn.includeEmissive) {
      Rgba8 g = pal.roadMarkWhite;
      g.a = 200;
      gfx::DrawText5x7(emit, pCx - tw / 2, pCy - th / 2, "H", g, scale, 1);
    }
  }

  if (kind == GfxFacilityKind::Police || kind == GfxFacilityKind::Fire) {
    // Siren lights.
    const float cx = static_cast<float>(pivotX);
    const float cy = static_cast<float>(pivotY - baseHeightPx - std::max(2, tileH / 10));
    const float r = static_cast<float>(std::max(2, tileH / 12));

    Rgba8 a = (kind == GfxFacilityKind::Police) ? pal.overlayCommercial : fireRed;
    Rgba8 b = (kind == GfxFacilityKind::Police) ? fireRed : pal.roadMarkYellow;
    a.a = 220;
    b.a = 220;

    gfx::FillCircleSoft(img, cx - r * 1.4f, cy, r, 1.5f, a);
    gfx::FillCircleSoft(img, cx + r * 1.4f, cy, r, 1.5f, b);

    if (cfgIn.includeEmissive) {
      a.a = 210;
      b.a = 210;
      gfx::FillCircleSoft(emit, cx - r * 1.4f, cy, r * 1.25f, 2.0f, a);
      gfx::FillCircleSoft(emit, cx + r * 1.4f, cy, r * 1.25f, 2.0f, b);
    }
  }

  // ---------------------------------------------------------------------------
  // Outlines
  // ---------------------------------------------------------------------------

  gfx::StrokeLine(img, roof.tx, roof.ty, roof.rx, roof.ry, outline);
  gfx::StrokeLine(img, roof.rx, roof.ry, roof.bx, roof.by, outline);
  gfx::StrokeLine(img, roof.bx, roof.by, roof.lx, roof.ly, outline);
  gfx::StrokeLine(img, roof.lx, roof.ly, roof.tx, roof.ty, outline);

  gfx::StrokeLine(img, base.lx, base.ly, base.bx, base.by, outline);
  gfx::StrokeLine(img, base.bx, base.by, base.rx, base.ry, outline);

  // Done.
  out.color = std::move(img);
  out.emissive = std::move(emit);
  return true;
}

} // namespace isocity
