#include "isocity/GfxFrames.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/GfxPatterns.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace isocity {

namespace {

using gfx::AffineTranslate;
using gfx::BlendMode;
using gfx::BlitImageAffine;
using gfx::FillCircleSoft;
using gfx::FillRect;
using gfx::FillTriangle;
using gfx::Lerp;
using gfx::Mul;
using gfx::StrokeLine;

inline Rgba8 Opaque(Rgba8 c)
{
  c.a = 255;
  return c;
}

inline Rgba8 WithA(Rgba8 c, std::uint8_t a)
{
  c.a = a;
  return c;
}

inline Rgba8 Darken(Rgba8 c, float m)
{
  c = Mul(c, m);
  c.a = 255;
  return c;
}

inline Rgba8 Lighten(Rgba8 c, float m)
{
  c = Mul(c, m);
  c.a = 255;
  return c;
}

// Cheap seed combiner stable across platforms.
inline std::uint64_t MixSeed(std::uint32_t seed, int variant, std::uint32_t salt)
{
  std::uint64_t s = (static_cast<std::uint64_t>(seed) << 32);
  s ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(variant));
  s ^= static_cast<std::uint64_t>(salt) * 0xD6E8FEB86659FD93ULL;
  return SplitMix64Next(s);
}

Rgba8 PickAccentColor(RNG& rng, const GfxPalette& pal)
{
  const Rgba8 cands[] = {
      pal.roadMarkWhite,
      pal.roadMarkYellow,
      pal.shorelineFoam,
      pal.treeDark,
      pal.overlayResidential,
      pal.overlayCommercial,
      pal.overlayIndustrial,
      pal.overlayPark,
  };

  constexpr std::uint32_t kCount = static_cast<std::uint32_t>(sizeof(cands) / sizeof(cands[0]));
  const std::uint32_t idx = rng.rangeU32(kCount);
  return Opaque(cands[idx]);
}

void ApplyRoundedCornerMask(RgbaImage& img, int radiusPx)
{
  if (radiusPx <= 0) return;
  if (img.width <= 0 || img.height <= 0) return;

  // Clamp to half-size.
  radiusPx = std::min(radiusPx, std::min(img.width, img.height) / 2);
  if (radiusPx <= 0) return;

  const float r = static_cast<float>(radiusPx);
  const float feather = 1.0f; // 1px AA edge.

  auto applyCorner = [&](int cx, int cy, int sx, int sy) {
    // cx,cy are the corner circle center, and sx/sy are the sign (+1/-1) directing into the corner.
    for (int y = 0; y < radiusPx; ++y) {
      for (int x = 0; x < radiusPx; ++x) {
        const int px = cx + sx * x;
        const int py = cy + sy * y;
        if (px < 0 || py < 0 || px >= img.width || py >= img.height) continue;

        const float fx = (static_cast<float>(px) + 0.5f) - (static_cast<float>(cx) + 0.5f);
        const float fy = (static_cast<float>(py) + 0.5f) - (static_cast<float>(cy) + 0.5f);
        const float d = std::sqrt(fx * fx + fy * fy);

        float a = 1.0f;
        if (d > r) {
          a = 0.0f;
        } else if (d > r - feather) {
          a = std::clamp((r - d) / feather, 0.0f, 1.0f);
        }

        const std::size_t i = (static_cast<std::size_t>(py) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(px)) * 4u;
        const float oldA = static_cast<float>(img.rgba[i + 3]) / 255.0f;
        const float newA = oldA * a;
        img.rgba[i + 3] = static_cast<std::uint8_t>(std::lround(std::clamp(newA, 0.0f, 1.0f) * 255.0f));
      }
    }
  };

  // Top-left, top-right, bottom-left, bottom-right.
  applyCorner(radiusPx - 1, radiusPx - 1, -1, -1);
  applyCorner(img.width - radiusPx, radiusPx - 1, +1, -1);
  applyCorner(radiusPx - 1, img.height - radiusPx, -1, +1);
  applyCorner(img.width - radiusPx, img.height - radiusPx, +1, +1);
}

} // namespace

const char* GfxFrameDecoName(GfxFrameDeco d)
{
  switch (d) {
  case GfxFrameDeco::Random: return "random";
  case GfxFrameDeco::CornerTriangles: return "corner_triangles";
  case GfxFrameDeco::CornerDots: return "corner_dots";
  case GfxFrameDeco::TitleBar: return "title_bar";
  default: return "random";
  }
}

bool ParseGfxFrameDeco(const std::string& s, GfxFrameDeco& out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    if (c >= 'A' && c <= 'Z') t.push_back(static_cast<char>(c - 'A' + 'a'));
    else t.push_back(c);
  }
  if (t.empty()) return false;

  if (t == "random" || t == "rand" || t == "r") {
    out = GfxFrameDeco::Random;
    return true;
  }
  if (t == "corner_triangles" || t == "triangles" || t == "triangle" || t == "tri") {
    out = GfxFrameDeco::CornerTriangles;
    return true;
  }
  if (t == "corner_dots" || t == "dots" || t == "dot") {
    out = GfxFrameDeco::CornerDots;
    return true;
  }
  if (t == "title_bar" || t == "titlebar" || t == "bar") {
    out = GfxFrameDeco::TitleBar;
    return true;
  }

  char* end = nullptr;
  const long v = std::strtol(t.c_str(), &end, 10);
  if (end && *end == '\0') {
    switch (v) {
    case 0: out = GfxFrameDeco::Random; return true;
    case 1: out = GfxFrameDeco::CornerTriangles; return true;
    case 2: out = GfxFrameDeco::CornerDots; return true;
    case 3: out = GfxFrameDeco::TitleBar; return true;
    default: break;
    }
  }

  return false;
}

bool GenerateGfxFrame(int variant, std::uint32_t seed, const GfxFrameConfig& cfg,
                      const GfxPalette& pal, RgbaImage& out, std::string& outError)
{
  outError.clear();
  out = RgbaImage{};

  if (cfg.sizePx <= 0 || cfg.sizePx > 2048) {
    outError = "frame sizePx must be in [1,2048]";
    return false;
  }
  if (variant < 0) variant = 0;

  out.width = cfg.sizePx;
  out.height = cfg.sizePx;
  out.rgba.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, std::uint8_t{0});

  const int borderPx = (cfg.borderPx > 0) ? cfg.borderPx : (cfg.sizePx >= 64 ? 6 : 4);
  if (borderPx * 2 >= cfg.sizePx) {
    outError = "frame borderPx too large for sizePx";
    return false;
  }

  // Deterministic per-variant RNG.
  RNG rng(MixSeed(seed, variant, 0x4652414Du /*FRAM*/));

  const Rgba8 baseBg = Darken(Opaque(pal.roadAsphalt2), 0.86f + 0.10f * rng.nextF01());
  const Rgba8 border = Lighten(baseBg, 1.22f);
  const Rgba8 accent = PickAccentColor(rng, pal);
  const Rgba8 hi = Lighten(border, 1.10f);
  const Rgba8 lo = Darken(border, 0.84f);

  // Outer border fill.
  FillRect(out, 0, 0, cfg.sizePx - 1, cfg.sizePx - 1, border, BlendMode::Alpha);

  // Inner background.
  const int x0 = borderPx;
  const int y0 = borderPx;
  const int x1 = cfg.sizePx - 1 - borderPx;
  const int y1 = cfg.sizePx - 1 - borderPx;
  FillRect(out, x0, y0, x1, y1, baseBg, BlendMode::Alpha);

  // Optional interior pattern overlay.
  const float patStrength = std::clamp(cfg.patternStrength, 0.0f, 1.0f);
  if (patStrength > 0.001f) {
    GfxPatternConfig pc;
    pc.sizePx = cfg.sizePx;
    pc.tileable = true;
    pc.period = std::max(8, std::min(64, cfg.sizePx / 2));
    pc.contrast = 1.0f;

    RgbaImage pat;
    std::string err;
    // Use a different salt to avoid perfect correlation with external pattern sheets.
    if (GenerateGfxPattern(variant + 1000, seed ^ 0xF2A5D3C1u, pc, pal, pat, err) &&
        pat.width == out.width && pat.height == out.height &&
        pat.rgba.size() == out.rgba.size()) {

      const float cx = static_cast<float>(cfg.sizePx) * 0.5f;
      const float cy = static_cast<float>(cfg.sizePx) * 0.5f;
      const float invR = 1.0f / std::max(1.0f, static_cast<float>(cfg.sizePx) * 0.50f);

      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x)) * 4u;
          const Rgba8 p{pat.rgba[i + 0], pat.rgba[i + 1], pat.rgba[i + 2], 255};

          Rgba8 c = Lerp(baseBg, p, patStrength);

          // Small vignette so the border reads clearly.
          const float dx = (static_cast<float>(x) + 0.5f) - cx;
          const float dy = (static_cast<float>(y) + 0.5f) - cy;
          const float d2 = (dx * dx + dy * dy) * (invR * invR);
          const float vig = std::clamp(1.0f - 0.18f * d2, 0.70f, 1.0f);
          c = Mul(c, vig);
          c.a = 255;

          out.rgba[i + 0] = c.r;
          out.rgba[i + 1] = c.g;
          out.rgba[i + 2] = c.b;
          out.rgba[i + 3] = 255;
        }
      }
    }
  }

  // Simple bevel/shadow lines for depth.
  StrokeLine(out, x0, y0, x1, y0, hi, BlendMode::Alpha);
  StrokeLine(out, x0, y0, x0, y1, hi, BlendMode::Alpha);
  StrokeLine(out, x0, y1, x1, y1, lo, BlendMode::Alpha);
  StrokeLine(out, x1, y0, x1, y1, lo, BlendMode::Alpha);

  // Decorative accents in the corners.
  GfxFrameDeco deco = cfg.deco;
  if (deco == GfxFrameDeco::Random) {
    deco = static_cast<GfxFrameDeco>(1u + rng.rangeU32(3u));
  }

  const int tick = std::max(2, borderPx / 2);
  if (deco == GfxFrameDeco::CornerTriangles) {
    // Corner triangles.
    const Rgba8 tri = WithA(accent, 220);
    FillTriangle(out, 0, 0, tick * 2, 0, 0, tick * 2, tri, BlendMode::Alpha);
    FillTriangle(out, out.width - 1, 0, out.width - 1 - tick * 2, 0, out.width - 1, tick * 2, tri, BlendMode::Alpha);
    FillTriangle(out, 0, out.height - 1, tick * 2, out.height - 1, 0, out.height - 1 - tick * 2, tri, BlendMode::Alpha);
    FillTriangle(out, out.width - 1, out.height - 1, out.width - 1 - tick * 2, out.height - 1,
                 out.width - 1, out.height - 1 - tick * 2, tri, BlendMode::Alpha);
  } else if (deco == GfxFrameDeco::CornerDots) {
    // Corner dots.
    const float rr = std::max(1.6f, static_cast<float>(borderPx) * 0.33f);
    const float feather = std::max(1.0f, rr * 0.6f);
    const Rgba8 dot = WithA(accent, 210);
    FillCircleSoft(out, rr, rr, rr, feather, dot, BlendMode::Alpha);
    FillCircleSoft(out, static_cast<float>(out.width) - rr, rr, rr, feather, dot, BlendMode::Alpha);
    FillCircleSoft(out, rr, static_cast<float>(out.height) - rr, rr, feather, dot, BlendMode::Alpha);
    FillCircleSoft(out, static_cast<float>(out.width) - rr, static_cast<float>(out.height) - rr, rr, feather, dot, BlendMode::Alpha);
  } else {
    // Top title bar.
    const int barH = std::max(3, borderPx - 1);
    const Rgba8 bar = WithA(accent, 190);
    FillRect(out, x0 + 2, y0 + 2, x1 - 2, y0 + 2 + barH, bar, BlendMode::Alpha);
  }

  // Rounded corners (alpha mask) if requested.
  ApplyRoundedCornerMask(out, cfg.cornerRadiusPx);

  return true;
}

bool GenerateGfxFrameSheet(int count, int columns, std::uint32_t seed, const GfxFrameConfig& cfg,
                           const GfxPalette& pal, RgbaImage& out,
                           std::vector<std::string>* outNames,
                           std::string& outError)
{
  outError.clear();
  out = RgbaImage{};
  if (outNames) outNames->clear();

  if (count <= 0) {
    outError = "frame sheet count must be > 0";
    return false;
  }
  if (columns <= 0) {
    outError = "frame sheet columns must be > 0";
    return false;
  }

  const int size = cfg.sizePx;
  const int rows = (count + columns - 1) / columns;
  if (rows <= 0) {
    outError = "frame sheet computed rows invalid";
    return false;
  }

  out.width = columns * size;
  out.height = rows * size;
  out.rgba.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, std::uint8_t{0});

  for (int i = 0; i < count; ++i) {
    RgbaImage tile;
    std::string err;
    if (!GenerateGfxFrame(i, seed, cfg, pal, tile, err)) {
      outError = "frame generation failed: " + err;
      return false;
    }

    const int ox = (i % columns) * size;
    const int oy = (i / columns) * size;

    BlitImageAffine(out, tile, AffineTranslate(static_cast<float>(ox), static_cast<float>(oy)),
                   gfx::SampleMode::Nearest, BlendMode::Alpha);

    if (outNames) outNames->push_back(std::string("frame_") + std::to_string(i));
  }

  return true;
}

} // namespace isocity
