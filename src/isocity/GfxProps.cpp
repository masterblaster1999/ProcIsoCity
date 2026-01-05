#include "isocity/GfxProps.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace isocity {

namespace {

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

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

inline Rgba8 Lerp(Rgba8 a, Rgba8 b, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  const float it = 1.0f - t;
  const int r = static_cast<int>(std::lround(static_cast<float>(a.r) * it + static_cast<float>(b.r) * t));
  const int g = static_cast<int>(std::lround(static_cast<float>(a.g) * it + static_cast<float>(b.g) * t));
  const int bb = static_cast<int>(std::lround(static_cast<float>(a.b) * it + static_cast<float>(b.b) * t));
  const int aa = static_cast<int>(std::lround(static_cast<float>(a.a) * it + static_cast<float>(b.a) * t));
  return Rgba8{ClampU8(r), ClampU8(g), ClampU8(bb), ClampU8(aa)};
}

inline void BlendPixel(RgbaImage& img, int x, int y, Rgba8 src)
{
  if (src.a == 0) return;
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;

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

  // Accumulate in premultiplied space, then unpremultiply.
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

void FillCircleSoft(RgbaImage& img, float cx, float cy, float r, float feather, Rgba8 c)
{
  if (r <= 0.5f) return;
  feather = std::max(0.0f, feather);
  const int minX = std::max(0, static_cast<int>(std::floor(cx - r - 1.0f)));
  const int maxX = std::min(img.width - 1, static_cast<int>(std::ceil(cx + r + 1.0f)));
  const int minY = std::max(0, static_cast<int>(std::floor(cy - r - 1.0f)));
  const int maxY = std::min(img.height - 1, static_cast<int>(std::ceil(cy + r + 1.0f)));

  const float inner = std::max(0.0f, r - feather);
  const float inner2 = inner * inner;
  const float r2 = r * r;

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const float dx = (static_cast<float>(x) + 0.5f) - cx;
      const float dy = (static_cast<float>(y) + 0.5f) - cy;
      const float d2 = dx * dx + dy * dy;
      if (d2 > r2) continue;

      float a = 1.0f;
      if (feather > 0.001f && d2 > inner2) {
        const float d = std::sqrt(d2);
        a = std::clamp((r - d) / feather, 0.0f, 1.0f);
      }

      Rgba8 cc = c;
      cc.a = static_cast<std::uint8_t>(std::lround(static_cast<float>(c.a) * a));
      BlendPixel(img, x, y, cc);
    }
  }
}

// Slight isometric-ish lighting: brighter on upper-left, darker on lower-right.
inline float SpriteLight(float nx, float ny)
{
  // nx,ny in [-1,1] roughly. Light from (-0.6,-0.5).
  const float lx = -0.60f;
  const float ly = -0.55f;
  const float d = (nx * lx + ny * ly);
  return std::clamp(0.92f + 0.20f * d, 0.70f, 1.20f);
}

struct TallSpriteLayout {
  int tileW = 0;
  int tileH = 0;
  int halfW = 0;
  int halfH = 0;
  int spriteH = 0;
  int pivotX = 0;
  int pivotY = 0;
};

TallSpriteLayout MakeTallLayout(const GfxPropsConfig& cfg)
{
  TallSpriteLayout l;
  l.tileW = cfg.tileW;
  l.tileH = cfg.tileH;
  l.halfW = cfg.tileW / 2;
  l.halfH = cfg.tileH / 2;

  const int marginTop = 4;
  const int marginBot = 3;
  const int autoH = cfg.tileH * 2 + static_cast<int>(std::lround(cfg.tileH * 0.75f)) + marginTop + marginBot;
  l.spriteH = (cfg.tallSpriteH > 0) ? cfg.tallSpriteH : autoH;
  l.pivotX = l.halfW;
  l.pivotY = l.spriteH - marginBot - l.halfH;
  return l;
}

void MakeTreeDeciduous(int variant, std::uint32_t seedv, const GfxPropsConfig& cfg, const GfxPalette& pal,
                       GfxPropSprite& out)
{
  const TallSpriteLayout lay = MakeTallLayout(cfg);

  out.color.width = lay.tileW;
  out.color.height = lay.spriteH;
  out.color.rgba.assign(static_cast<std::size_t>(out.color.width) * static_cast<std::size_t>(out.color.height) * 4u, 0u);

  if (cfg.includeEmissive) {
    out.emissive.width = out.color.width;
    out.emissive.height = out.color.height;
    out.emissive.rgba.assign(static_cast<std::size_t>(out.emissive.width) * static_cast<std::size_t>(out.emissive.height) * 4u, 0u);
  }

  out.pivotX = lay.pivotX;
  out.pivotY = lay.pivotY;

  // Deterministic per-variant RNG.
  auto h01 = [&](int x, int y, std::uint32_t salt) {
    return Frac01(HashCoords32(x, y, seedv ^ salt));
  };

  const float trunkH = static_cast<float>(cfg.tileH) * (0.55f + 0.25f * h01(1, 2, 0xA1u));
  const float trunkW = 2.0f + 1.0f * h01(3, 4, 0xA2u);

  // Canopy size.
  const float canopyR = static_cast<float>(cfg.tileH) * (0.55f + 0.22f * h01(5, 6, 0xA3u));
  const float canopyR2 = canopyR * (0.75f + 0.20f * h01(7, 8, 0xA4u));

  const float cx = static_cast<float>(lay.pivotX) + (h01(9, 10, 0xA5u) - 0.5f) * 2.0f;
  const float cy = static_cast<float>(lay.pivotY) - trunkH - canopyR * 0.45f;

  // Leaf palette derived from theme.
  const Rgba8 leafDark = Mul(pal.treeDark, 0.95f);
  const Rgba8 leafLight = Mul(pal.overlayPark, 1.25f);
  const Rgba8 leafMid = Lerp(leafDark, leafLight, 0.45f);

  // Trunk color from sand/asphalt mix.
  const Rgba8 trunkA = Mul(pal.sand, 0.70f);
  const Rgba8 trunkB = Mul(pal.roadAsphalt2, 1.05f);
  const Rgba8 trunk = Lerp(trunkA, trunkB, 0.35f);

  // Draw trunk first.
  const int tx0 = static_cast<int>(std::lround(static_cast<float>(lay.pivotX) - trunkW * 0.5f));
  const int tx1 = static_cast<int>(std::lround(static_cast<float>(lay.pivotX) + trunkW * 0.5f));
  const int ty0 = static_cast<int>(std::lround(static_cast<float>(lay.pivotY) - trunkH));
  const int ty1 = lay.pivotY;
  FillRect(out.color, tx0, ty0, tx1, ty1, Rgba8{trunk.r, trunk.g, trunk.b, 235});

  // Main canopy blob built from several overlapping circles.
  const float feather = std::max(1.5f, canopyR * 0.18f);
  for (int i = 0; i < 7; ++i) {
    const float a = static_cast<float>(i) / 7.0f;
    const float ox = (h01(i, variant, 0xB1u) - 0.5f) * canopyR * 0.55f;
    const float oy = (h01(i, variant, 0xB2u) - 0.5f) * canopyR * 0.35f;
    const float rr = canopyR * (0.72f + 0.22f * h01(i, variant, 0xB3u));
    const float blend = 0.35f + 0.50f * a;
    const Rgba8 c = Lerp(leafMid, leafLight, blend);
    FillCircleSoft(out.color, cx + ox, cy + oy, rr, feather, Rgba8{c.r, c.g, c.b, 235});
  }

  // Dithered leaf detail + simple lighting.
  for (int y = 0; y < out.color.height; ++y) {
    for (int x = 0; x < out.color.width; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.color.width) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = out.color.rgba[idx + 3];
      if (a == 0) continue;

      const float nx = (static_cast<float>(x) + 0.5f - cx) / (canopyR + 1.0f);
      const float ny = (static_cast<float>(y) + 0.5f - cy) / (canopyR + 1.0f);
      const float light = SpriteLight(nx, ny);

      float jitter = (h01(x, y, 0xC1u) - 0.5f) * 0.12f;
      // Slightly more "sparkle" near the rim.
      const float rim = std::clamp(std::sqrt(nx * nx + ny * ny), 0.0f, 1.0f);
      jitter += rim * (h01(x, y, 0xC2u) - 0.5f) * 0.10f;

      const float m = std::clamp(light + jitter, 0.75f, 1.22f);
      out.color.rgba[idx + 0] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[idx + 0]) * m)));
      out.color.rgba[idx + 1] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[idx + 1]) * m)));
      out.color.rgba[idx + 2] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[idx + 2]) * m)));

      // Occasional dark "holes" for texture.
      if ((HashCoords32(x, y, seedv ^ 0xC3u) & 0xFFu) == 0x5Au) {
        out.color.rgba[idx + 0] = static_cast<std::uint8_t>((static_cast<int>(out.color.rgba[idx + 0]) * 3) / 5);
        out.color.rgba[idx + 1] = static_cast<std::uint8_t>((static_cast<int>(out.color.rgba[idx + 1]) * 3) / 5);
        out.color.rgba[idx + 2] = static_cast<std::uint8_t>((static_cast<int>(out.color.rgba[idx + 2]) * 3) / 5);
      }
    }
  }
}

void MakeTreeConifer(int variant, std::uint32_t seedv, const GfxPropsConfig& cfg, const GfxPalette& pal,
                     GfxPropSprite& out)
{
  const TallSpriteLayout lay = MakeTallLayout(cfg);

  out.color.width = lay.tileW;
  out.color.height = lay.spriteH;
  out.color.rgba.assign(static_cast<std::size_t>(out.color.width) * static_cast<std::size_t>(out.color.height) * 4u, 0u);

  if (cfg.includeEmissive) {
    out.emissive.width = out.color.width;
    out.emissive.height = out.color.height;
    out.emissive.rgba.assign(static_cast<std::size_t>(out.emissive.width) * static_cast<std::size_t>(out.emissive.height) * 4u, 0u);
  }

  out.pivotX = lay.pivotX;
  out.pivotY = lay.pivotY;

  auto h01 = [&](int x, int y, std::uint32_t salt) {
    return Frac01(HashCoords32(x, y, seedv ^ salt));
  };

  const float trunkH = static_cast<float>(cfg.tileH) * (0.45f + 0.20f * h01(1, 2, 0xD1u));
  const float trunkW = 2.0f;
  const float treeH = static_cast<float>(cfg.tileH) * (1.45f + 0.40f * h01(3, 4, 0xD2u));
  const float baseW = static_cast<float>(cfg.tileW) * (0.26f + 0.06f * h01(5, 6, 0xD3u));

  const Rgba8 leafDark = Mul(pal.treeDark, 0.90f);
  const Rgba8 leafLight = Mul(pal.overlayPark, 1.18f);
  const Rgba8 leafMid = Lerp(leafDark, leafLight, 0.35f);

  const Rgba8 trunkA = Mul(pal.sand, 0.68f);
  const Rgba8 trunkB = Mul(pal.roadAsphalt2, 1.02f);
  const Rgba8 trunk = Lerp(trunkA, trunkB, 0.42f);

  // Trunk.
  const int tx0 = static_cast<int>(std::lround(static_cast<float>(lay.pivotX) - trunkW * 0.5f));
  const int tx1 = static_cast<int>(std::lround(static_cast<float>(lay.pivotX) + trunkW * 0.5f));
  const int ty0 = static_cast<int>(std::lround(static_cast<float>(lay.pivotY) - trunkH));
  FillRect(out.color, tx0, ty0, tx1, lay.pivotY, Rgba8{trunk.r, trunk.g, trunk.b, 235});

  // Conifer layers (stacked triangles-ish). We draw as a set of soft circles with shrinking radii.
  const float cx = static_cast<float>(lay.pivotX);
  const float topY = static_cast<float>(lay.pivotY) - trunkH - treeH;
  const int layers = 6 + static_cast<int>(std::lround(h01(7, 8, 0xD4u) * 2.0f));
  for (int i = 0; i < layers; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(std::max(1, layers - 1));
    const float y = topY + treeH * (0.10f + 0.90f * t);
    const float rr = baseW * (1.00f - t) * (0.90f + 0.10f * h01(i, variant, 0xD5u));
    const float feather = std::max(1.0f, rr * 0.25f);
    const Rgba8 c = Lerp(leafLight, leafMid, t);
    FillCircleSoft(out.color, cx, y, rr, feather, Rgba8{c.r, c.g, c.b, 235});
  }

  // Noise/detail.
  for (int y = 0; y < out.color.height; ++y) {
    for (int x = 0; x < out.color.width; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.color.width) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = out.color.rgba[idx + 3];
      if (a == 0) continue;

      const float nx = (static_cast<float>(x) + 0.5f - cx) / (baseW + 1.0f);
      const float ny = (static_cast<float>(y) + 0.5f - (topY + treeH * 0.5f)) / (treeH + 1.0f);
      const float light = SpriteLight(nx, ny);
      const float jitter = (h01(x, y, 0xD6u) - 0.5f) * 0.10f;
      const float m = std::clamp(light + jitter, 0.78f, 1.18f);
      out.color.rgba[idx + 0] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[idx + 0]) * m)));
      out.color.rgba[idx + 1] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[idx + 1]) * m)));
      out.color.rgba[idx + 2] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[idx + 2]) * m)));
    }
  }
}

void MakeStreetLight(int variant, std::uint32_t seedv, const GfxPropsConfig& cfg, const GfxPalette& pal,
                     GfxPropSprite& out)
{
  const TallSpriteLayout lay = MakeTallLayout(cfg);

  // Slightly shorter than trees by default.
  const int spriteH = (cfg.tallSpriteH > 0) ? cfg.tallSpriteH : (cfg.tileH * 2 + cfg.tileH / 2 + 10);

  out.color.width = lay.tileW;
  out.color.height = spriteH;
  out.color.rgba.assign(static_cast<std::size_t>(out.color.width) * static_cast<std::size_t>(out.color.height) * 4u, 0u);

  if (cfg.includeEmissive) {
    out.emissive.width = out.color.width;
    out.emissive.height = out.color.height;
    out.emissive.rgba.assign(static_cast<std::size_t>(out.emissive.width) * static_cast<std::size_t>(out.emissive.height) * 4u, 0u);
  }

  const int marginBot = 3;
  const int pivotY = spriteH - marginBot - (cfg.tileH / 2);
  out.pivotX = cfg.tileW / 2;
  out.pivotY = pivotY;

  auto h01 = [&](int x, int y, std::uint32_t salt) {
    return Frac01(HashCoords32(x, y, seedv ^ salt));
  };

  const float poleH = static_cast<float>(cfg.tileH) * (1.10f + 0.35f * h01(1, 2, 0xE1u));
  const float poleW = 1.6f + 0.8f * h01(3, 4, 0xE2u);
  const int topY = static_cast<int>(std::lround(static_cast<float>(pivotY) - poleH));

  const Rgba8 metalDark = Mul(pal.roadAsphalt3, 1.15f);
  const Rgba8 metalLight = Mul(pal.roadAsphalt1, 1.25f);

  // Pole.
  const int x0 = static_cast<int>(std::lround(static_cast<float>(out.pivotX) - poleW * 0.5f));
  const int x1 = static_cast<int>(std::lround(static_cast<float>(out.pivotX) + poleW * 0.5f));
  for (int y = topY; y <= pivotY; ++y) {
    const float t = (poleH > 1.0f) ? (static_cast<float>(pivotY - y) / poleH) : 0.0f;
    const Rgba8 c = Lerp(metalDark, metalLight, 0.35f + 0.45f * t);
    FillRect(out.color, x0, y, x1, y, Rgba8{c.r, c.g, c.b, 240});
  }

  // Simple arm.
  const bool left = (h01(5, 6, 0xE3u) < 0.5f);
  const int armLen = 4 + static_cast<int>(std::lround(h01(7, 8, 0xE4u) * 4.0f));
  const int armY = topY + 2;
  const int ax0 = out.pivotX;
  const int ax1 = out.pivotX + (left ? -armLen : armLen);
  for (int x = std::min(ax0, ax1); x <= std::max(ax0, ax1); ++x) {
    BlendPixel(out.color, x, armY, Rgba8{metalLight.r, metalLight.g, metalLight.b, 240});
  }

  // Lamp head.
  const int lampX = ax1;
  const int lampY = armY + 1;
  FillCircleSoft(out.color, static_cast<float>(lampX), static_cast<float>(lampY), 2.2f, 0.8f,
                 Rgba8{metalDark.r, metalDark.g, metalDark.b, 245});

  // Glow (emissive).
  if (cfg.includeEmissive && !out.emissive.rgba.empty()) {
    const Rgba8 glow = Rgba8{pal.roadMarkYellow.r, pal.roadMarkYellow.g, pal.roadMarkYellow.b, 255};
    FillCircleSoft(out.emissive, static_cast<float>(lampX), static_cast<float>(lampY + 1), 2.4f, 1.2f,
                   Rgba8{glow.r, glow.g, glow.b, 220});
    // Subtle falloff.
    FillCircleSoft(out.emissive, static_cast<float>(lampX), static_cast<float>(lampY + 2), 4.5f, 2.4f,
                   Rgba8{glow.r, glow.g, glow.b, 85});
  }
}

void MakeVehicle(bool truck, int variant, std::uint32_t seedv, const GfxPropsConfig& cfg, const GfxPalette& pal,
                 GfxPropSprite& out)
{
  out.color.width = cfg.tileW;
  out.color.height = cfg.tileH;
  out.color.rgba.assign(static_cast<std::size_t>(out.color.width) * static_cast<std::size_t>(out.color.height) * 4u, 0u);

  if (cfg.includeEmissive) {
    out.emissive.width = out.color.width;
    out.emissive.height = out.color.height;
    out.emissive.rgba.assign(static_cast<std::size_t>(out.emissive.width) * static_cast<std::size_t>(out.emissive.height) * 4u, 0u);
  }

  out.pivotX = cfg.tileW / 2;
  out.pivotY = cfg.tileH / 2;

  auto h01 = [&](int x, int y, std::uint32_t salt) {
    return Frac01(HashCoords32(x, y, seedv ^ salt));
  };

  // Orient along one of the two isometric diagonals.
  const bool diagNE = (h01(1, 2, 0xF1u) < 0.5f);

  // Base colors: choose a vivid paint from overlays.
  const Rgba8 c0 = pal.overlayCommercial;
  const Rgba8 c1 = pal.overlayIndustrial;
  const Rgba8 c2 = pal.overlayResidential;
  const Rgba8 paint = Lerp(Lerp(c0, c1, h01(3, 4, 0xF2u)), c2, h01(5, 6, 0xF3u));
  const Rgba8 dark = Mul(paint, 0.72f);
  const Rgba8 light = Mul(paint, 1.15f);
  const Rgba8 glass = Mul(pal.water, 0.85f);
  const Rgba8 tire = Mul(pal.roadAsphalt3, 1.05f);

  // Vehicle footprint in diamond coords.
  const float hw = truck ? 0.33f : 0.27f;
  const float hh = truck ? 0.20f : 0.16f;
  const float yOff = truck ? 0.03f : 0.00f;

  const float cx = static_cast<float>(out.pivotX);
  const float cy = static_cast<float>(out.pivotY) + static_cast<float>(cfg.tileH) * yOff;
  const float sx = static_cast<float>(cfg.tileW) * 0.5f;
  const float sy = static_cast<float>(cfg.tileH) * 0.5f;

  // Simple raster: draw a rotated-ish rectangle by sampling diamond-space coordinates.
  for (int y = 0; y < out.color.height; ++y) {
    for (int x = 0; x < out.color.width; ++x) {
      const float nx = ((static_cast<float>(x) + 0.5f) - cx) / sx;
      const float ny = ((static_cast<float>(y) + 0.5f) - cy) / sy;
      // Rotate 45° in diamond space by swapping/negating.
      const float ux = diagNE ? (nx * 0.7071f - ny * 0.7071f) : (nx * 0.7071f + ny * 0.7071f);
      const float uy = diagNE ? (nx * 0.7071f + ny * 0.7071f) : (-nx * 0.7071f + ny * 0.7071f);

      if (std::fabs(ux) > hw || std::fabs(uy) > hh) continue;

      // Height profile: cab on one side.
      const float cab = truck ? 0.35f : 0.42f;
      const bool isCab = (diagNE ? (ux > (hw * (1.0f - cab))) : (ux < -(hw * (1.0f - cab))));

      Rgba8 c = isCab ? light : dark;
      c.a = 245;

      // Window stripe.
      if (isCab && std::fabs(uy) < hh * 0.45f && (std::fabs(ux) > hw * 0.55f)) {
        c = Mul(glass, 1.10f);
        c.a = 235;
      }

      // Tires as darker pixels at the corners.
      const float tireBand = hh * 0.88f;
      if (std::fabs(uy) > tireBand) {
        c = tire;
        c.a = 255;
      }

      BlendPixel(out.color, x, y, c);
    }
  }

  // Headlights emissive.
  if (cfg.includeEmissive && !out.emissive.rgba.empty()) {
    const Rgba8 hl = Rgba8{255, 245, 210, 255};
    const int dir = diagNE ? 1 : -1;
    const int hx = out.pivotX + dir * static_cast<int>(std::lround(static_cast<float>(cfg.tileW) * 0.14f));
    const int hy = out.pivotY + static_cast<int>(std::lround(static_cast<float>(cfg.tileH) * 0.02f));
    FillCircleSoft(out.emissive, static_cast<float>(hx), static_cast<float>(hy), 1.4f, 0.6f, Rgba8{hl.r, hl.g, hl.b, 170});
    FillCircleSoft(out.emissive, static_cast<float>(hx), static_cast<float>(hy), 3.0f, 1.8f, Rgba8{hl.r, hl.g, hl.b, 55});
  }
}

} // namespace

bool GenerateGfxPropSprite(GfxPropKind kind, int variant, std::uint32_t seed,
                           const GfxPropsConfig& cfgIn, const GfxPalette& pal,
                           GfxPropSprite& out, std::string& outError)
{
  outError.clear();
  out = GfxPropSprite{};

  if (cfgIn.tileW <= 0 || cfgIn.tileH <= 0) {
    outError = "invalid tile size";
    return false;
  }

  const int v = std::max(0, variant);
  const std::uint32_t seedv = seed ^ 0x51A7C0DEu ^ (static_cast<std::uint32_t>(v) * 0x9E3779B9u) ^
                              (static_cast<std::uint32_t>(kind) * 0x85EBCA6Bu);

  GfxPropsConfig cfg = cfgIn;

  switch (kind) {
  case GfxPropKind::TreeDeciduous:
    MakeTreeDeciduous(v, seedv, cfg, pal, out);
    return true;
  case GfxPropKind::TreeConifer:
    MakeTreeConifer(v, seedv, cfg, pal, out);
    return true;
  case GfxPropKind::StreetLight:
    MakeStreetLight(v, seedv, cfg, pal, out);
    return true;
  case GfxPropKind::VehicleCar:
    MakeVehicle(false, v, seedv, cfg, pal, out);
    return true;
  case GfxPropKind::VehicleTruck:
    MakeVehicle(true, v, seedv, cfg, pal, out);
    return true;
  default:
    break;
  }

  outError = "unknown prop kind";
  return false;
}

} // namespace isocity
