#include "isocity/GfxSigils.hpp"

#include "isocity/GfxCanvas.hpp"
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
using gfx::Mul;
using gfx::SpriteLight;
using gfx::StrokeLine;
using gfx::StrokeLineAA;

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
  // Run through SplitMix once for diffusion.
  return SplitMix64Next(s);
}

Rgba8 PickColor(RNG& rng, const GfxPalette& pal)
{
  const Rgba8 cands[] = {
      pal.overlayResidential,
      pal.overlayCommercial,
      pal.overlayIndustrial,
      pal.overlayPark,
      pal.grass,
      pal.water,
      pal.sand,
      pal.roadMarkYellow,
      pal.roadMarkWhite,
      pal.treeDark,
  };

  constexpr std::uint32_t kCount = static_cast<std::uint32_t>(sizeof(cands) / sizeof(cands[0]));
  const std::uint32_t idx = rng.rangeU32(kCount);
  Rgba8 c = cands[idx];
  c.a = 255;
  return c;
}

void ApplyCircularMask(RgbaImage& img, float cx, float cy, float rOuter, bool transparentOutside)
{
  if (!transparentOutside) return;

  const float r2 = rOuter * rOuter;
  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const float dx = (static_cast<float>(x) + 0.5f) - cx;
      const float dy = (static_cast<float>(y) + 0.5f) - cy;
      const float d2 = dx * dx + dy * dy;
      if (d2 <= r2) continue;
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
      img.rgba[i + 0] = std::uint8_t{0};
      img.rgba[i + 1] = std::uint8_t{0};
      img.rgba[i + 2] = std::uint8_t{0};
      img.rgba[i + 3] = std::uint8_t{0};
    }
  }
}

void DrawGlossHighlight(RgbaImage& img, float cx, float cy, float rInner)
{
  // Subtle top-left glossy highlight to keep icons readable on dark backgrounds.
  const float hx = cx - rInner * 0.25f;
  const float hy = cy - rInner * 0.28f;
  const float hr = rInner * 0.65f;
  const float feather = std::max(1.0f, rInner * 0.35f);
  FillCircleSoft(img, hx, hy, hr, feather, Rgba8{255, 255, 255, 26}, BlendMode::Alpha);
}

void DrawSymmetricBlocks(RgbaImage& img, RNG& rng, float cx, float cy, float rInner, Rgba8 fg, Rgba8 accent)
{
  // Grid size: prefer odd sizes for a centered design.
  const int options[] = {5, 7, 9};
  const int grid = options[static_cast<int>(rng.rangeU32(3u))];
  const int half = (grid + 1) / 2;
  const float area = rInner * 2.0f * 0.92f;
  const float start = cx - area * 0.5f;
  const float cell = area / static_cast<float>(grid);

  const float density = 0.30f + 0.25f * rng.nextF01();
  const bool mirrorY = rng.chance(0.35f);

  // Determine which cells are filled, ensuring symmetry.
  std::vector<std::uint8_t> bits(static_cast<std::size_t>(grid) * static_cast<std::size_t>(grid), std::uint8_t{0});
  for (int y = 0; y < grid; ++y) {
    for (int x = 0; x < half; ++x) {
      const bool on = (rng.nextF01() < density);
      const int mx = grid - 1 - x;
      bits[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid) + static_cast<std::size_t>(x)] = on ? 1u : 0u;
      bits[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid) + static_cast<std::size_t>(mx)] = on ? 1u : 0u;
    }
  }

  if (mirrorY) {
    for (int y = 0; y < grid / 2; ++y) {
      const int my = grid - 1 - y;
      for (int x = 0; x < grid; ++x) {
        const std::uint8_t on = bits[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid) + static_cast<std::size_t>(x)];
        bits[static_cast<std::size_t>(my) * static_cast<std::size_t>(grid) + static_cast<std::size_t>(x)] = on;
      }
    }
  }

  // Render cells.
  const int pad = std::max(0, static_cast<int>(std::floor(cell * 0.15f)));
  for (int y = 0; y < grid; ++y) {
    for (int x = 0; x < grid; ++x) {
      if (!bits[static_cast<std::size_t>(y) * static_cast<std::size_t>(grid) + static_cast<std::size_t>(x)]) continue;

      const float fx0 = start + static_cast<float>(x) * cell;
      const float fy0 = start + static_cast<float>(y) * cell;
      const float fx1 = start + static_cast<float>(x + 1) * cell;
      const float fy1 = start + static_cast<float>(y + 1) * cell;

      const int x0 = static_cast<int>(std::floor(fx0)) + pad;
      const int y0 = static_cast<int>(std::floor(fy0)) + pad;
      const int x1 = static_cast<int>(std::ceil(fx1)) - 1 - pad;
      const int y1 = static_cast<int>(std::ceil(fy1)) - 1 - pad;
      if (x1 < x0 || y1 < y0) continue;

      // Simple light shading for depth.
      const float cxf = (fx0 + fx1) * 0.5f;
      const float cyf = (fy0 + fy1) * 0.5f;
      const float nx = (cxf - cx) / std::max(1.0f, rInner);
      const float ny = (cyf - cy) / std::max(1.0f, rInner);
      const float lit = SpriteLight(nx, ny);
      Rgba8 cc = Mul(fg, lit);
      cc.a = 210;
      FillRect(img, x0, y0, x1, y1, cc, BlendMode::Alpha);

      // Occasional accent dot.
      if (rng.chance(0.08f)) {
        const int dx = (x0 + x1) / 2;
        const int dy = (y0 + y1) / 2;
        FillCircleSoft(img, static_cast<float>(dx) + 0.5f, static_cast<float>(dy) + 0.5f, 1.6f, 1.0f, WithA(accent, 220));
      }
    }
  }
}

void DrawStarburst(RgbaImage& img, RNG& rng, float cx, float cy, float rInner, Rgba8 fg, Rgba8 accent)
{
  const int rays = 5 + static_cast<int>(rng.rangeU32(8u));
  const float base = rng.rangeFloat(0.0f, 6.28318530718f);
  const float r0 = rInner * 0.15f;
  const float r1 = rInner * 0.92f;

  for (int i = 0; i < rays; ++i) {
    const float t = base + (static_cast<float>(i) / static_cast<float>(rays)) * 6.28318530718f + rng.rangeFloat(-0.10f, 0.10f);
    const float x0 = cx + std::cos(t) * r0;
    const float y0 = cy + std::sin(t) * r0;
    const float x1 = cx + std::cos(t) * r1;
    const float y1 = cy + std::sin(t) * r1;
    StrokeLineAA(img, x0, y0, x1, y1, WithA(fg, 220), BlendMode::Alpha);

    // Slight thickness.
    if (rng.chance(0.35f)) {
      StrokeLineAA(img, x0 + 0.7f, y0, x1 + 0.7f, y1, WithA(fg, 140), BlendMode::Alpha);
    }
  }

  // Center medallion.
  FillCircleSoft(img, cx, cy, rInner * 0.18f, std::max(1.0f, rInner * 0.06f), WithA(accent, 235));
  FillCircleSoft(img, cx, cy, rInner * 0.10f, std::max(1.0f, rInner * 0.05f), WithA(fg, 235));
}

void DrawChevron(RgbaImage& img, RNG& rng, float cx, float cy, float rInner, Rgba8 fg, Rgba8 accent)
{
  const int bands = 4 + static_cast<int>(rng.rangeU32(4u));
  const float w = rInner * 1.55f;
  const float h = rInner * 1.55f;
  const float x0 = cx - w * 0.5f;
  const float y0 = cy - h * 0.5f;

  for (int i = 0; i < bands; ++i) {
    const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(bands);
    const float y = y0 + t * h;
    const float inset = (0.10f + 0.25f * std::fabs(0.5f - t)) * w;
    const int ax = static_cast<int>(std::lround(x0 + inset));
    const int bx = static_cast<int>(std::lround(x0 + w - inset));
    const int yy = static_cast<int>(std::lround(y));
    const int mid = static_cast<int>(std::lround(cx));

    // A single chevron "V".
    const Rgba8 c = (i % 2 == 0) ? WithA(fg, 220) : WithA(accent, 200);
    StrokeLine(img, ax, yy, mid, yy + 6, c, BlendMode::Alpha);
    StrokeLine(img, mid, yy + 6, bx, yy, c, BlendMode::Alpha);
  }
}

void DrawCenterGlyph(RgbaImage& img, RNG& rng, float cx, float cy, float rInner, Rgba8 fg, Rgba8 accent, GfxSigilGlyph glyph)
{
  std::uint32_t kind = 0u;
  switch (glyph) {
  case GfxSigilGlyph::Triangle: kind = 0u; break;
  case GfxSigilGlyph::Dots: kind = 1u; break;
  case GfxSigilGlyph::Tower: kind = 2u; break;
  case GfxSigilGlyph::Random:
  default:
    kind = rng.rangeU32(3u);
    break;
  }

  if (kind == 0u) {
    // Triangle "mountain".
    const int x0 = static_cast<int>(std::lround(cx));
    const int y0 = static_cast<int>(std::lround(cy - rInner * 0.20f));
    const int x1 = static_cast<int>(std::lround(cx - rInner * 0.22f));
    const int y1 = static_cast<int>(std::lround(cy + rInner * 0.18f));
    const int x2 = static_cast<int>(std::lround(cx + rInner * 0.22f));
    const int y2 = y1;
    FillTriangle(img, x0, y0, x1, y1, x2, y2, WithA(accent, 235), BlendMode::Alpha);
    StrokeLine(img, x1, y1, x0, y0, WithA(fg, 190), BlendMode::Alpha);
    StrokeLine(img, x0, y0, x2, y2, WithA(fg, 190), BlendMode::Alpha);
  } else if (kind == 1u) {
    // Dot cluster.
    const int dots = 6 + static_cast<int>(rng.rangeU32(8u));
    for (int i = 0; i < dots; ++i) {
      const float a = rng.rangeFloat(0.0f, 6.28318530718f);
      const float rr = rInner * rng.rangeFloat(0.04f, 0.11f);
      const float r = rInner * rng.rangeFloat(0.10f, 0.35f);
      const float px = cx + std::cos(a) * r;
      const float py = cy + std::sin(a) * r;
      FillCircleSoft(img, px, py, rr, std::max(0.8f, rr * 0.6f), WithA((i % 2 == 0) ? fg : accent, 215));
    }
  } else {
    // Simple "tower".
    const int w = static_cast<int>(std::lround(rInner * 0.20f));
    const int h = static_cast<int>(std::lround(rInner * 0.38f));
    const int x0 = static_cast<int>(std::lround(cx)) - w / 2;
    const int y0 = static_cast<int>(std::lround(cy)) - h / 2;
    FillRect(img, x0, y0, x0 + w, y0 + h, WithA(accent, 215));
    FillRect(img, x0 + 2, y0 + 2, x0 + w - 2, y0 + h - 2, WithA(fg, 190));
  }
}

} // namespace

const char* GfxSigilStyleName(GfxSigilStyle s)
{
  switch (s) {
  case GfxSigilStyle::Random: return "random";
  case GfxSigilStyle::Blocks: return "blocks";
  case GfxSigilStyle::Starburst: return "starburst";
  case GfxSigilStyle::Chevron: return "chevron";
  default: return "random";
  }
}

bool ParseGfxSigilStyle(const std::string& s, GfxSigilStyle& out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    if (c >= 'A' && c <= 'Z') t.push_back(static_cast<char>(c - 'A' + 'a'));
    else t.push_back(c);
  }
  if (t.empty()) return false;

  if (t == "random" || t == "rand" || t == "r") {
    out = GfxSigilStyle::Random;
    return true;
  }
  if (t == "blocks" || t == "block" || t == "grid" || t == "sym") {
    out = GfxSigilStyle::Blocks;
    return true;
  }
  if (t == "starburst" || t == "burst" || t == "rays" || t == "star") {
    out = GfxSigilStyle::Starburst;
    return true;
  }
  if (t == "chevron" || t == "v" || t == "zigzag" || t == "bands") {
    out = GfxSigilStyle::Chevron;
    return true;
  }

  char* end = nullptr;
  const long v = std::strtol(t.c_str(), &end, 10);
  if (end && *end == '\0') {
    switch (v) {
    case 0: out = GfxSigilStyle::Random; return true;
    case 1: out = GfxSigilStyle::Blocks; return true;
    case 2: out = GfxSigilStyle::Starburst; return true;
    case 3: out = GfxSigilStyle::Chevron; return true;
    default: break;
    }
  }

  return false;
}

const char* GfxSigilGlyphName(GfxSigilGlyph g)
{
  switch (g) {
  case GfxSigilGlyph::Random: return "random";
  case GfxSigilGlyph::Triangle: return "triangle";
  case GfxSigilGlyph::Dots: return "dots";
  case GfxSigilGlyph::Tower: return "tower";
  default: return "random";
  }
}

bool ParseGfxSigilGlyph(const std::string& s, GfxSigilGlyph& out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    if (c >= 'A' && c <= 'Z') t.push_back(static_cast<char>(c - 'A' + 'a'));
    else t.push_back(c);
  }
  if (t.empty()) return false;

  if (t == "random" || t == "rand" || t == "r") {
    out = GfxSigilGlyph::Random;
    return true;
  }
  if (t == "triangle" || t == "tri" || t == "mountain") {
    out = GfxSigilGlyph::Triangle;
    return true;
  }
  if (t == "dots" || t == "dot" || t == "cluster") {
    out = GfxSigilGlyph::Dots;
    return true;
  }
  if (t == "tower" || t == "building" || t == "keep") {
    out = GfxSigilGlyph::Tower;
    return true;
  }

  char* end = nullptr;
  const long v = std::strtol(t.c_str(), &end, 10);
  if (end && *end == '\0') {
    switch (v) {
    case 0: out = GfxSigilGlyph::Random; return true;
    case 1: out = GfxSigilGlyph::Triangle; return true;
    case 2: out = GfxSigilGlyph::Dots; return true;
    case 3: out = GfxSigilGlyph::Tower; return true;
    default: break;
    }
  }

  return false;
}

bool GenerateGfxSigil(int variant, std::uint32_t seed, const GfxSigilConfig& cfg,
                      const GfxPalette& pal, RgbaImage& out, std::string& outError)
{
  outError.clear();
  out = RgbaImage{};

  if (cfg.sizePx <= 0 || cfg.sizePx > 2048) {
    outError = "sigil sizePx must be in [1,2048]";
    return false;
  }
  if (variant < 0) variant = 0;

  out.width = cfg.sizePx;
  out.height = cfg.sizePx;
  out.rgba.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, std::uint8_t{0});

  // Deterministic per-variant RNG.
  RNG rng(MixSeed(seed, variant, 0x5A17u));

  const float cx = static_cast<float>(cfg.sizePx) * 0.5f;
  const float cy = static_cast<float>(cfg.sizePx) * 0.5f;
  const float rOuter = static_cast<float>(cfg.sizePx) * 0.48f;
  const int borderPx = (cfg.borderPx > 0) ? cfg.borderPx : (cfg.sizePx >= 64 ? 3 : 2);
  const float rInner = std::max(1.0f, rOuter - static_cast<float>(borderPx));

  // Colors derived from palette.
  const Rgba8 baseBg = Darken(pal.roadAsphalt2, 0.92f + 0.10f * rng.nextF01());
  const Rgba8 border = Lighten(baseBg, 1.22f);
  Rgba8 fg = PickColor(rng, pal);
  Rgba8 accent = PickColor(rng, pal);

  // Keep fg and accent distinct.
  if (fg.r == accent.r && fg.g == accent.g && fg.b == accent.b) {
    accent = Lighten(accent, 1.30f);
  }

  // Base badge circles.
  FillCircleSoft(out, cx, cy, rOuter, std::max(1.0f, rOuter * 0.04f), border, BlendMode::Alpha);
  FillCircleSoft(out, cx, cy, rInner, std::max(1.0f, rInner * 0.04f), baseBg, BlendMode::Alpha);

  // Pattern selection.
  GfxSigilStyle style = cfg.style;
  if (style == GfxSigilStyle::Random) {
    style = static_cast<GfxSigilStyle>(1u + rng.rangeU32(3u));
  }

  switch (style) {
  case GfxSigilStyle::Blocks:
    DrawSymmetricBlocks(out, rng, cx, cy, rInner, fg, accent);
    break;
  case GfxSigilStyle::Starburst:
    DrawStarburst(out, rng, cx, cy, rInner, fg, accent);
    break;
  case GfxSigilStyle::Chevron:
    DrawChevron(out, rng, cx, cy, rInner, fg, accent);
    break;
  case GfxSigilStyle::Random:
  default:
    DrawSymmetricBlocks(out, rng, cx, cy, rInner, fg, accent);
    break;
  }

  // Center glyph for additional identity.
  const float glyphChance = std::clamp(cfg.glyphChance, 0.0f, 1.0f);
  const bool forceGlyph = (cfg.glyph != GfxSigilGlyph::Random) && (glyphChance > 0.0f);
  if (forceGlyph || rng.chance(glyphChance)) {
    DrawCenterGlyph(out, rng, cx, cy, rInner, fg, accent, cfg.glyph);
  }

  DrawGlossHighlight(out, cx, cy, rInner);
  ApplyCircularMask(out, cx, cy, rOuter, cfg.transparentOutside);

  return true;
}

bool GenerateGfxSigilSheet(int count, int columns, std::uint32_t seed, const GfxSigilConfig& cfg,
                           const GfxPalette& pal, RgbaImage& out,
                           std::vector<std::string>* outNames,
                           std::string& outError)
{
  outError.clear();
  out = RgbaImage{};
  if (outNames) outNames->clear();

  if (count <= 0) {
    outError = "sigil sheet count must be > 0";
    return false;
  }
  if (columns <= 0) {
    outError = "sigil sheet columns must be > 0";
    return false;
  }

  const int size = cfg.sizePx;
  const int rows = (count + columns - 1) / columns;
  if (rows <= 0) {
    outError = "sigil sheet computed rows invalid";
    return false;
  }

  out.width = columns * size;
  out.height = rows * size;
  out.rgba.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, std::uint8_t{0});

  for (int i = 0; i < count; ++i) {
    RgbaImage icon;
    std::string err;
    if (!GenerateGfxSigil(i, seed, cfg, pal, icon, err)) {
      outError = "sigil generation failed: " + err;
      return false;
    }

    const int ox = (i % columns) * size;
    const int oy = (i / columns) * size;

    BlitImageAffine(out, icon, AffineTranslate(static_cast<float>(ox), static_cast<float>(oy)), gfx::SampleMode::Nearest, BlendMode::Alpha);

    if (outNames) outNames->push_back(std::string("sigil_") + std::to_string(i));
  }

  return true;
}

} // namespace isocity
