#include "isocity/GfxPatterns.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/Noise.hpp"
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
using gfx::Lerp;
using gfx::Mul;

constexpr float kTau = 6.2831853071795864769f;

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

inline void PutPixel(RgbaImage& img, int x, int y, Rgba8 c)
{
  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
  img.rgba[i + 0] = c.r;
  img.rgba[i + 1] = c.g;
  img.rgba[i + 2] = c.b;
  img.rgba[i + 3] = c.a;
}

// Tileable domain-warp FBM in [0,1].
inline float TileNoise01(float uNorm, float vNorm, std::uint32_t seed, const GfxPatternConfig& cfg,
                         float warpAmp = 1.1f)
{
  const int p = (cfg.tileable ? std::max(1, cfg.period) : 0);
  const float x = uNorm * static_cast<float>(p);
  const float y = vNorm * static_cast<float>(p);
  return DomainWarpFBm2DPeriodic(x, y, seed, p, p, 5, 2.0f, 0.5f, warpAmp);
}

Rgba8 PickBaseColor(RNG& rng, const GfxPalette& pal)
{
  const Rgba8 cands[] = {
      pal.roadAsphalt1,
      pal.roadAsphalt2,
      pal.roadAsphalt3,
      pal.bridgeDeck1,
      pal.bridgeDeck2,
      pal.bridgeDeck3,
      pal.overlayResidential,
      pal.overlayCommercial,
      pal.overlayIndustrial,
      pal.overlayPark,
      pal.water,
      pal.grass,
      pal.sand,
  };

  constexpr std::uint32_t kCount = static_cast<std::uint32_t>(sizeof(cands) / sizeof(cands[0]));
  const std::uint32_t idx = rng.rangeU32(kCount);
  return Opaque(cands[idx]);
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

void RenderGrain(RgbaImage& out, RNG& rng, std::uint32_t seed, const GfxPatternConfig& cfg, const GfxPalette& pal)
{
  const Rgba8 base = Darken(PickBaseColor(rng, pal), 0.85f + 0.12f * rng.nextF01());
  const Rgba8 hi = Lighten(base, 1.22f);

  const float strength = 0.50f * cfg.contrast;
  const float speckChance = 0.006f + 0.010f * rng.nextF01();

  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(out.width);
      const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(out.height);

      const float n = TileNoise01(u, v, seed ^ 0x47524E31u /*GRN1*/, cfg, 1.3f); // [0,1]
      const float m = std::clamp(0.85f + (n - 0.5f) * strength, 0.35f, 1.65f);

      Rgba8 c = Mul(base, m);
      c.a = 255;

      // Sparse speckle highlights to keep it from looking like smooth banding.
      const std::uint32_t h = HashCoords32(x, y, seed ^ 0x53504543u /*SPEC*/);
      const float r01 = static_cast<float>(h) / static_cast<float>(0xFFFFFFFFu);
      if (r01 < speckChance) {
        c = Lerp(c, hi, 0.65f);
        c.a = 255;
      }

      PutPixel(out, x, y, c);
    }
  }
}

void RenderHatch(RgbaImage& out, RNG& rng, std::uint32_t seed, const GfxPatternConfig& cfg, const GfxPalette& pal)
{
  const Rgba8 base = Darken(PickBaseColor(rng, pal), 0.92f);
  const Rgba8 line = Lighten(PickAccentColor(rng, pal), 1.08f);

  // Integer cycle counts guarantee seamless tiling.
  const int cycles = 4 + static_cast<int>(rng.rangeU32(10u));
  const float phase = rng.rangeFloat(0.0f, kTau);
  const float thickness = 0.10f + 0.06f * rng.nextF01();
  const bool diagAlt = rng.chance(0.5f);

  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(out.width);
      const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(out.height);

      const float d = diagAlt ? (u + v) : (u - v);
      const float s = std::sin((d * static_cast<float>(cycles)) * kTau + phase);
      const float a = std::fabs(s);

      // Soft-edged lines.
      const float t = std::clamp((thickness - a) / std::max(1.0e-6f, thickness), 0.0f, 1.0f);
      const float noise = TileNoise01(u, v, seed ^ 0x48415443u /*HATC*/, cfg, 0.8f);
      const float n = (noise - 0.5f) * 0.25f * cfg.contrast;

      Rgba8 c = Lerp(base, line, t);
      c = Mul(c, 1.0f + n);
      c.a = 255;
      PutPixel(out, x, y, c);
    }
  }
}

void RenderBricks(RgbaImage& out, RNG& rng, std::uint32_t seed, const GfxPatternConfig& cfg, const GfxPalette& pal)
{
  (void)cfg;
  const Rgba8 brick = Darken(PickBaseColor(rng, pal), 0.96f);
  const Rgba8 mortar = Darken(brick, 0.65f);
  const Rgba8 hi = Lighten(brick, 1.18f);

  // Choose a brick grid that divides the tile size and has an even row count so the running-bond
  // offset wraps cleanly top-to-bottom.
  int rows = 8;
  if (out.height % rows != 0) rows = 4;
  if (out.height % rows != 0) rows = 2;

  int cols = 8;
  if (out.width % cols != 0) cols = 4;
  if (out.width % cols != 0) cols = 2;

  const int brickW = std::max(1, out.width / cols);
  const int brickH = std::max(1, out.height / rows);
  const int mortarPx = std::max(1, std::min(brickW, brickH) / 12);

  for (int y = 0; y < out.height; ++y) {
    const int row = (brickH > 0) ? (y / brickH) : 0;
    const int shift = (row & 1) ? (brickW / 2) : 0;

    for (int x = 0; x < out.width; ++x) {
      const int xx = x + shift;
      const int lx = (brickW > 0) ? (xx % brickW) : 0;
      const int ly = (brickH > 0) ? (y % brickH) : 0;

      const bool edgeX = (lx < mortarPx) || (lx >= brickW - mortarPx);
      const bool edgeY = (ly < mortarPx) || (ly >= brickH - mortarPx);

      Rgba8 c = (edgeX || edgeY) ? mortar : brick;

      // Slight per-brick variation.
      const int bx = (brickW > 0) ? (xx / brickW) : 0;
      const int by = (brickH > 0) ? (y / brickH) : 0;
      const float vn = static_cast<float>(HashCoords32(bx, by, seed ^ 0x42524943u /*BRIC*/)) / static_cast<float>(0xFFFFFFFFu);
      const float mv = 0.92f + 0.22f * vn;
      c = Mul(c, mv);
      c.a = 255;

      // Occasional highlight speck.
      const std::uint32_t h = HashCoords32(x, y, seed ^ 0x48494C49u /*HILI*/);
      if ((h & 0x1FFFu) == 0u) {
        c = Lerp(c, hi, 0.75f);
        c.a = 255;
      }

      PutPixel(out, x, y, c);
    }
  }
}

void RenderWaves(RgbaImage& out, RNG& rng, std::uint32_t seed, const GfxPatternConfig& cfg, const GfxPalette& pal)
{
  const Rgba8 base = Darken(Opaque(pal.water), 0.92f);
  const Rgba8 hi = Lighten(Opaque(pal.water), 1.12f);
  const Rgba8 foam = Lighten(Opaque(pal.shorelineFoam), 1.05f);

  const int cycles = 3 + static_cast<int>(rng.rangeU32(7u));
  const float phase = rng.rangeFloat(0.0f, kTau);
  const float foamCut = 0.88f - 0.10f * rng.nextF01();

  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(out.width);
      const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(out.height);

      const float n = TileNoise01(u, v, seed ^ 0x57415645u /*WAVE*/, cfg, 1.6f);
      const float warp = (n * 2.0f - 1.0f) * 0.18f;

      const float t = (u * static_cast<float>(cycles) + v * 0.35f + warp) * kTau + phase;
      const float s = 0.5f + 0.5f * std::sin(t);

      Rgba8 c = Lerp(base, hi, std::pow(s, 1.25f) * std::clamp(cfg.contrast, 0.1f, 2.0f));
      if (s > foamCut) {
        const float k = std::clamp((s - foamCut) / std::max(1.0e-6f, 1.0f - foamCut), 0.0f, 1.0f);
        c = Lerp(c, foam, k);
      }
      c.a = 255;
      PutPixel(out, x, y, c);
    }
  }
}

} // namespace

const char* GfxPatternStyleName(GfxPatternStyle s)
{
  switch (s) {
  case GfxPatternStyle::Random: return "random";
  case GfxPatternStyle::Grain: return "grain";
  case GfxPatternStyle::Hatch: return "hatch";
  case GfxPatternStyle::Bricks: return "bricks";
  case GfxPatternStyle::Waves: return "waves";
  default: return "random";
  }
}

bool ParseGfxPatternStyle(const std::string& s, GfxPatternStyle& out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    if (c >= 'A' && c <= 'Z') t.push_back(static_cast<char>(c - 'A' + 'a'));
    else t.push_back(c);
  }

  if (t.empty()) return false;

  if (t == "random" || t == "rand" || t == "r") {
    out = GfxPatternStyle::Random;
    return true;
  }
  if (t == "grain" || t == "grn" || t == "noise") {
    out = GfxPatternStyle::Grain;
    return true;
  }
  if (t == "hatch" || t == "hatching" || t == "lines") {
    out = GfxPatternStyle::Hatch;
    return true;
  }
  if (t == "bricks" || t == "brick" || t == "masonry") {
    out = GfxPatternStyle::Bricks;
    return true;
  }
  if (t == "waves" || t == "wave" || t == "water") {
    out = GfxPatternStyle::Waves;
    return true;
  }

  // Numeric forms (0..4).
  char* end = nullptr;
  const long v = std::strtol(t.c_str(), &end, 10);
  if (end && *end == '\0') {
    switch (v) {
    case 0: out = GfxPatternStyle::Random; return true;
    case 1: out = GfxPatternStyle::Grain; return true;
    case 2: out = GfxPatternStyle::Hatch; return true;
    case 3: out = GfxPatternStyle::Bricks; return true;
    case 4: out = GfxPatternStyle::Waves; return true;
    default: break;
    }
  }

  return false;
}

bool GenerateGfxPattern(int variant, std::uint32_t seed, const GfxPatternConfig& cfg,
                        const GfxPalette& pal, RgbaImage& out, std::string& outError)
{
  outError.clear();
  out = RgbaImage{};

  if (cfg.sizePx <= 0 || cfg.sizePx > 2048) {
    outError = "pattern sizePx must be in [1,2048]";
    return false;
  }
  if (cfg.tileable && cfg.period <= 0) {
    outError = "pattern period must be > 0 when tileable";
    return false;
  }
  if (variant < 0) variant = 0;

  out.width = cfg.sizePx;
  out.height = cfg.sizePx;
  out.rgba.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, std::uint8_t{0});

  RNG rng(MixSeed(seed, variant, 0x50415454u /*PATT*/));

  GfxPatternStyle style = cfg.style;
  if (style == GfxPatternStyle::Random) {
    style = static_cast<GfxPatternStyle>(1u + rng.rangeU32(4u));
  }

  switch (style) {
  case GfxPatternStyle::Grain:
    RenderGrain(out, rng, seed, cfg, pal);
    break;
  case GfxPatternStyle::Hatch:
    RenderHatch(out, rng, seed, cfg, pal);
    break;
  case GfxPatternStyle::Bricks:
    RenderBricks(out, rng, seed, cfg, pal);
    break;
  case GfxPatternStyle::Waves:
    RenderWaves(out, rng, seed, cfg, pal);
    break;
  case GfxPatternStyle::Random:
  default:
    RenderGrain(out, rng, seed, cfg, pal);
    break;
  }

  return true;
}

bool GenerateGfxPatternSheet(int count, int columns, std::uint32_t seed, const GfxPatternConfig& cfg,
                             const GfxPalette& pal, RgbaImage& out,
                             std::vector<std::string>* outNames,
                             std::string& outError)
{
  outError.clear();
  out = RgbaImage{};
  if (outNames) outNames->clear();

  if (count <= 0) {
    outError = "pattern sheet count must be > 0";
    return false;
  }
  if (columns <= 0) {
    outError = "pattern sheet columns must be > 0";
    return false;
  }

  const int size = cfg.sizePx;
  const int rows = (count + columns - 1) / columns;
  if (rows <= 0) {
    outError = "pattern sheet computed rows invalid";
    return false;
  }

  out.width = columns * size;
  out.height = rows * size;
  out.rgba.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, std::uint8_t{0});

  for (int i = 0; i < count; ++i) {
    RgbaImage tile;
    std::string err;
    if (!GenerateGfxPattern(i, seed, cfg, pal, tile, err)) {
      outError = "pattern generation failed: " + err;
      return false;
    }

    const int ox = (i % columns) * size;
    const int oy = (i / columns) * size;

    BlitImageAffine(out, tile, AffineTranslate(static_cast<float>(ox), static_cast<float>(oy)),
                   gfx::SampleMode::Nearest, BlendMode::Alpha);

    if (outNames) outNames->push_back(std::string("pattern_") + std::to_string(i));
  }

  return true;
}

} // namespace isocity
