#include "isocity/GfxProps.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace isocity {

namespace {

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

using gfx::BlendMode;
using gfx::BlendPixel;
using gfx::ClampU8;
using gfx::FillCircleSoft;
using gfx::FillRect;
using gfx::Lerp;
using gfx::Mul;
using gfx::SdfRoundRect;
using gfx::SmoothStep01;
using gfx::SpriteLight;

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
  [[maybe_unused]] const float canopyR2 = canopyR * (0.75f + 0.20f * h01(7, 8, 0xA4u));

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
  // Fold the variant into the seed so multiple styles can co-exist deterministically.
  seedv ^= 0x9E3779B9u * static_cast<std::uint32_t>(variant + 1);

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
                   Rgba8{glow.r, glow.g, glow.b, 220}, BlendMode::Additive);
    // Subtle falloff.
    FillCircleSoft(out.emissive, static_cast<float>(lampX), static_cast<float>(lampY + 2), 4.5f, 2.4f,
                   Rgba8{glow.r, glow.g, glow.b, 85}, BlendMode::Additive);
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

  // Deterministic diagonal orientation.
  //
  // - diagNE=true  => major axis slopes up-right on screen (negative covariance)
  // - diagNE=false => major axis slopes down-right on screen (positive covariance)
  //
  // We base this on variant parity so rebuildVehicleSprites can quickly collect a balanced
  // set of both diagonal orientations.
  const bool diagNE = ((variant & 1) == 0);

  // Style bucket (separate from the diagonal bit).
  const int style = (variant / 2) % (truck ? 4 : 5);
  const bool isTaxi = (!truck && style == 3);

  // Base colors: choose a vivid paint from a small set of variant-driven schemes.
  int scheme = (variant >= 0) ? (variant % 6) : ((-variant) % 6);
  if (isTaxi) scheme = 3;

  Rgba8 s0 = pal.overlayCommercial;
  Rgba8 s1 = pal.overlayIndustrial;
  Rgba8 s2 = pal.overlayResidential;

  switch (scheme) {
  default:
  case 0: // "default" mix
    break;
  case 1: // more residential
    s0 = pal.overlayResidential;
    s1 = pal.overlayCommercial;
    s2 = pal.overlayIndustrial;
    break;
  case 2: // more industrial
    s0 = pal.overlayIndustrial;
    s1 = pal.overlayResidential;
    s2 = pal.overlayCommercial;
    break;
  case 3: // taxi / service vibe
    s0 = pal.roadMarkYellow;
    s1 = pal.overlayCommercial;
    s2 = pal.overlayResidential;
    break;
  case 4: // light paint / fleet vehicles
    s0 = pal.roadMarkWhite;
    s1 = pal.overlayIndustrial;
    s2 = pal.overlayCommercial;
    break;
  case 5: // park-ish / green-ish paint
    s0 = pal.overlayPark;
    s1 = pal.overlayResidential;
    s2 = pal.overlayCommercial;
    break;
  }

  const Rgba8 paintBase = Lerp(Lerp(s0, s1, h01(3, 4, 0xF2u)), s2, h01(5, 6, 0xF3u));
  const Rgba8 paintDark = Mul(paintBase, 0.70f);
  const Rgba8 paintLight = Mul(paintBase, 1.18f);

  // Trucks look better with a slightly more utilitarian cargo body.
  Rgba8 cargoBase = Lerp(Mul(pal.roadMarkWhite, 0.96f), Mul(pal.overlayIndustrial, 1.02f), 0.35f);
  cargoBase = Lerp(cargoBase, Mul(paintBase, 0.92f), 0.20f + 0.35f * h01(9, 10, 0xABu));
  const Rgba8 cargoDark = Mul(cargoBase, 0.82f);
  const Rgba8 cargoLight = Mul(cargoBase, 1.10f);

  const Rgba8 glass = Mul(pal.water, 0.88f);
  const Rgba8 trim = Mul(pal.roadAsphalt2, 1.10f);
  const Rgba8 tire = Mul(pal.roadAsphalt3, 1.08f);

  // Vehicle footprint in (u,v) diamond coords.
  // u aligns with the chosen screen-space diagonal, v is the perpendicular diagonal.
  float halfLen = truck ? 0.40f : 0.33f;
  float halfWid = truck ? 0.22f : 0.18f;

  // Style variation.
  const float vLen = (h01(11, 12, 0xC0u) - 0.5f) * 0.06f;
  const float vWid = (h01(13, 14, 0xC1u) - 0.5f) * 0.05f;
  halfLen = std::clamp(halfLen + vLen, truck ? 0.34f : 0.28f, truck ? 0.46f : 0.40f);
  halfWid = std::clamp(halfWid + vWid, truck ? 0.19f : 0.15f, truck ? 0.26f : 0.21f);

  if (!truck) {
    // van / hatch / sporty tweaks
    if (style == 2) halfLen = std::min(0.41f, halfLen + 0.03f);
    if (style == 1) halfLen = std::min(0.40f, halfLen + 0.01f);
    if (style == 4) halfLen = std::max(0.28f, halfLen - 0.02f);
    if (style == 2) halfWid = std::min(0.22f, halfWid + 0.02f);
  } else {
    // flatbed slightly shorter, box truck slightly longer
    if (style == 1) halfLen = std::max(0.33f, halfLen - 0.03f);
    if (style == 0) halfLen = std::min(0.48f, halfLen + 0.02f);
  }

  const float yOff = truck ? 0.05f : 0.02f;

  const float cx = static_cast<float>(out.pivotX);
  const float cy = static_cast<float>(out.pivotY) + static_cast<float>(cfg.tileH) * yOff;
  const float sx = static_cast<float>(cfg.tileW) * 0.5f;
  const float sy = static_cast<float>(cfg.tileH) * 0.5f;

  // Soft under-shadow to anchor the sprite to the road surface.
  {
    const float shA = truck ? 75.0f : 62.0f;
    const float shR = (truck ? 0.54f : 0.50f) * static_cast<float>(cfg.tileH);
    const float shY = cy + static_cast<float>(cfg.tileH) * 0.14f;
    const float feather = shR * 0.75f;

    FillCircleSoft(out.color, cx - 1.0f, shY, shR, feather, Rgba8{0, 0, 0, static_cast<std::uint8_t>(shA)});
    FillCircleSoft(out.color, cx + 1.0f, shY, shR, feather, Rgba8{0, 0, 0, static_cast<std::uint8_t>(shA)});
  }

  const float rBody = std::min(halfLen, halfWid) * 0.36f;
  const float feather = 0.055f;

  // Wheel placement in (u,v) coords.
  const float wheelUFront = halfLen * 0.55f;
  const float wheelUBack = -halfLen * (truck ? 0.48f : 0.55f);
  const float wheelV = halfWid * 0.88f;
  const float wheelR = halfWid * (truck ? 0.34f : 0.37f);

  // Roof dimensions (cars) / cab roof (trucks).
  float roofHalfLen = halfLen * (truck ? 0.26f : 0.62f);
  float roofHalfWid = halfWid * (truck ? 0.55f : 0.56f);
  float roofCenterU = truck ? (halfLen * 0.55f) : (halfLen * 0.10f);

  // Vehicles are small; we intentionally exaggerate the roof a bit for readability.
  if (!truck && style == 2) {
    // van: longer roof
    roofHalfLen = halfLen * 0.70f;
    roofCenterU = halfLen * 0.05f;
  }

  // Primary raster pass.
  for (int y = 0; y < out.color.height; ++y) {
    for (int x = 0; x < out.color.width; ++x) {
      const float nx = ((static_cast<float>(x) + 0.5f) - cx) / sx;
      const float ny = ((static_cast<float>(y) + 0.5f) - cy) / sy;

      // Rotate into (u,v) (two diagonals). Both variants keep +u pointing screen-right.
      const float ux = diagNE ? (nx * 0.70710678f - ny * 0.70710678f)
                              : (nx * 0.70710678f + ny * 0.70710678f);
      const float uy = diagNE ? (nx * 0.70710678f + ny * 0.70710678f)
                              : (-nx * 0.70710678f + ny * 0.70710678f);

      const float sd = SdfRoundRect(ux, uy, halfLen, halfWid, rBody);
      if (sd > feather) continue;

      float aa = 1.0f;
      if (sd > 0.0f) {
        aa = (feather - sd) / feather;
        aa = SmoothStep01(aa);
      }

      // Base alpha: slightly translucent so sprites blend with the world (keeps them from looking like stickers).
      const float baseA = 245.0f;
      const std::uint8_t a8 = static_cast<std::uint8_t>(std::lround(baseA * aa));
      if (a8 == 0) continue;

      // Wheels (sit "under" the body but still visible at the corners).
      const float dV = std::fabs(std::fabs(uy) - wheelV);
      const float dF = ux - wheelUFront;
      const float dB = ux - wheelUBack;
      const bool wheel = ((dF * dF + dV * dV) <= (wheelR * wheelR)) || ((dB * dB + dV * dV) <= (wheelR * wheelR));

      // Truck body split: cargo vs cab.
      bool cab = false;
      if (truck) {
        const float cabCut = halfLen * (style == 0 ? 0.12f : 0.18f);
        cab = (ux > cabCut);
      }

      // Roof mask.
      const float sdRoof = SdfRoundRect(ux - roofCenterU, uy, roofHalfLen, roofHalfWid, rBody * 0.65f);
      const bool roof = (sdRoof <= 0.0f);

      // Window band (glass) on the roof.
      bool window = false;
      if (!truck) {
        window = roof && (ux > (-halfLen * 0.10f)) && (std::fabs(uy) < roofHalfWid * 0.36f);
      } else {
        // Truck windshield near the very front of the cab.
        window = roof && (ux > (halfLen * 0.42f)) && (std::fabs(uy) < roofHalfWid * 0.42f);
      }

      // Trim/bumpers near the front/back.
      const float bumper = rBody * 0.55f;
      const bool frontTrim = (ux > (halfLen - bumper));
      const bool backTrim = (ux < (-halfLen + bumper));

      // Base material.
      Rgba8 c = paintDark;
      if (truck && !cab) {
        c = cargoDark;
      }

      // Roof reads slightly lighter.
      if (roof) {
        c = truck && !cab ? cargoLight : paintLight;
      }

      // Windows override.
      if (window) {
        c = Mul(glass, 1.05f);
      }

      // Wheels override.
      if (wheel) {
        c = tire;
      }

      // Front/back trim override.
      if (!wheel && (frontTrim || backTrim)) {
        c = Mul(trim, roof ? 1.05f : 0.95f);
      }

      // Taxi roof sign (tiny but readable at high zoom).
      if (isTaxi && roof && (std::fabs(uy) < roofHalfWid * 0.22f) && (ux > halfLen * 0.05f) && (ux < halfLen * 0.28f)) {
        c = Mul(pal.roadMarkYellow, 1.10f);
      }

      // Simple isometric-ish lighting + slight grime.
      float light = SpriteLight(nx, ny);
      // Lower-right side reads darker.
      if (uy > 0.0f && !roof) light *= 0.92f;
      // Tiny per-pixel variation (deterministic).
      const float jitter = (h01(x, y, 0xD1u) - 0.5f) * 0.06f;
      light = std::clamp(light + jitter, 0.70f, 1.25f);

      c = Mul(c, light);
      c.a = a8;
      BlendPixel(out.color, x, y, c);

      // Tiny cargo separation seam for trucks.
      if (truck && !wheel && !window) {
        const float seamU = halfLen * (style == 0 ? 0.12f : 0.18f);
        if (std::fabs(ux - seamU) < (rBody * 0.25f) && std::fabs(uy) < (halfWid * 0.85f)) {
          BlendPixel(out.color, x, y, Rgba8{trim.r, trim.g, trim.b, static_cast<std::uint8_t>(std::min(255, a8 + 15))});
        }
      }
    }
  }

  // Darken boundary pixels to create a crisp outline (improves readability on bright terrain).
  {
    const int w = out.color.width;
    const int h = out.color.height;
    if (w > 2 && h > 2 && static_cast<int>(out.color.rgba.size()) >= w * h * 4) {
      std::vector<std::uint8_t> alpha;
      alpha.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
          alpha[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = out.color.rgba[i + 3u];
        }
      }

      auto aAt = [&](int x, int y) -> std::uint8_t {
        if (x < 0 || y < 0 || x >= w || y >= h) return 0;
        return alpha[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
      };

      for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
          const std::uint8_t a0 = aAt(x, y);
          if (a0 < 170) continue; // skip shadow / feather pixels

          const bool edge = (aAt(x - 1, y) < 60) || (aAt(x + 1, y) < 60) || (aAt(x, y - 1) < 60) || (aAt(x, y + 1) < 60);
          if (!edge) continue;

          const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
          out.color.rgba[i + 0] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 0]) * 0.58f)));
          out.color.rgba[i + 1] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 1]) * 0.58f)));
          out.color.rgba[i + 2] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 2]) * 0.58f)));
        }
      }
    }
  }

  // Emissive: headlights + taillights (and taxi roof sign when applicable).
  if (cfg.includeEmissive && !out.emissive.rgba.empty()) {
    // Forward direction in pixel space (points to screen-right).
    const float slope = (cfg.tileW > 0) ? (static_cast<float>(cfg.tileH) / static_cast<float>(cfg.tileW)) : 0.5f;
    float fx = 1.0f;
    float fy = diagNE ? (-slope) : (slope);
    const float fl = std::sqrt(fx * fx + fy * fy);
    if (fl > 1.0e-6f) {
      fx /= fl;
      fy /= fl;
    }
    // Perpendicular (vehicle width) direction.
    const float rx = -fy;
    const float ry = fx;

    const float frontOff = static_cast<float>(cfg.tileW) * (truck ? 0.17f : 0.155f);
    const float backOff = static_cast<float>(cfg.tileW) * (truck ? 0.150f : 0.135f);
    const float sideOff = static_cast<float>(cfg.tileH) * (truck ? 0.11f : 0.10f);

    const float frontCx = cx + fx * frontOff;
    const float frontCy = cy + fy * frontOff;
    const float backCx = cx - fx * backOff;
    const float backCy = cy - fy * backOff;

    const Rgba8 head = Rgba8{255, 245, 210, 255};
    const Rgba8 tail = Rgba8{235, 70, 55, 255};

    auto addLight = [&](float lx, float ly, float r0, float a0, float r1, float a1, const Rgba8& col) {
      Rgba8 c0 = col;
      c0.a = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(std::lround(a0)), 0, 255));
      FillCircleSoft(out.emissive, lx, ly, r0, r0 * 0.55f, c0, BlendMode::Additive);

      Rgba8 c1 = col;
      c1.a = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(std::lround(a1)), 0, 255));
      FillCircleSoft(out.emissive, lx, ly, r1, r1 * 0.75f, c1, BlendMode::Additive);
    };

    // Headlights (brighter, with a small halo).
    addLight(frontCx + rx * sideOff, frontCy + ry * sideOff, 1.35f, 175.0f, 3.2f, 55.0f, head);
    addLight(frontCx - rx * sideOff, frontCy - ry * sideOff, 1.35f, 175.0f, 3.2f, 55.0f, head);

    // Taillights (smaller, red).
    addLight(backCx + rx * (sideOff * 0.85f), backCy + ry * (sideOff * 0.85f), 1.10f, 130.0f, 2.6f, 35.0f, tail);
    addLight(backCx - rx * (sideOff * 0.85f), backCy - ry * (sideOff * 0.85f), 1.10f, 130.0f, 2.6f, 35.0f, tail);

    // Taxi roof sign (tiny warm marker).
    if (isTaxi) {
      const float sx0 = cx + fx * (static_cast<float>(cfg.tileW) * 0.04f);
      const float sy0 = cy + fy * (static_cast<float>(cfg.tileW) * 0.04f) - static_cast<float>(cfg.tileH) * 0.15f;
      const Rgba8 sign = Rgba8{pal.roadMarkYellow.r, pal.roadMarkYellow.g, pal.roadMarkYellow.b, 255};
      addLight(sx0, sy0, 1.10f, 120.0f, 2.8f, 25.0f, sign);
    }
  }
}


// ---------------------------------------------------------------------------------------------
// Pedestrians (tiny decorative “city life” sprites)
// ---------------------------------------------------------------------------------------------

void MakePedestrian(int variant, std::uint32_t seedv, const GfxPropsConfig& cfg, const GfxPalette& pal,
                    GfxPropSprite& out)
{
  // We treat variants in pairs so the renderer can flip between poses for a simple walk animation:
  //   style = variant/2, pose = variant&1.
  const int v = std::max(0, variant);
  const int style = v >> 1;
  const int pose = v & 1;

  const int w = cfg.tileW;

  // Keep people compact to reduce overdraw.
  const int marginTop = 3;
  const int marginBot = 2;
  const int autoH = static_cast<int>(std::lround(static_cast<float>(cfg.tileH) * 1.65f)) + marginTop + marginBot;
  const int h = (cfg.tallSpriteH > 0) ? cfg.tallSpriteH : autoH;

  out.color.width = w;
  out.color.height = h;
  out.color.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);

  if (cfg.includeEmissive) {
    out.emissive.width = w;
    out.emissive.height = h;
    out.emissive.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);
  }

  // Pivot at the feet.
  out.pivotX = w / 2;
  out.pivotY = h - marginBot;

  auto h01 = [&](int x, int y, std::uint32_t salt) {
    return Frac01(HashCoords32(x, y, seedv ^ salt));
  };

  const float cx = static_cast<float>(out.pivotX) + (h01(style, 1, 0xA11u) - 0.5f) * 1.0f;
  const float footY = static_cast<float>(out.pivotY);

  // Colors: pick from theme-ish palette but add variety.
  const Rgba8 skinA = Mul(pal.sand, 0.98f);
  const Rgba8 skinB = Mul(pal.overlayResidential, 0.92f);
  const Rgba8 skin = Lerp(skinA, skinB, 0.20f + 0.60f * h01(style, 2, 0xA12u));

  const Rgba8 clothA = Lerp(pal.overlayResidential, pal.overlayCommercial, 0.35f);
  const Rgba8 clothB = Lerp(pal.overlayIndustrial, pal.overlayCommercial, 0.55f);
  const Rgba8 shirt = Lerp(clothA, clothB, h01(style, 3, 0xA13u));
  const Rgba8 pants = Mul(pal.roadAsphalt2, 0.85f + 0.22f * h01(style, 4, 0xA14u));
  const Rgba8 shoe = Mul(pal.roadAsphalt1, 0.62f);

  // A tiny soft shadow blob at the feet so people don't look like they float.
  {
    const float r = static_cast<float>(cfg.tileH) * 0.11f;
    const float f = r * 0.75f;
    FillCircleSoft(out.color, cx, footY + 0.8f, r, f, Rgba8{0, 0, 0, 70});
    FillCircleSoft(out.color, cx + 1.4f, footY + 1.0f, r * 0.70f, f * 0.70f, Rgba8{0, 0, 0, 48});
  }

  // Scale varies slightly per style.
  const float scale = 0.90f + 0.18f * h01(style, 5, 0xA15u);

  const float legH = static_cast<float>(cfg.tileH) * (0.28f + 0.08f * h01(style, 6, 0xA16u)) * scale;
  const float torsoH = static_cast<float>(cfg.tileH) * (0.34f + 0.10f * h01(style, 7, 0xA17u)) * scale;
  const float headR = 2.6f + 0.85f * h01(style, 8, 0xA18u);

  const float torsoW = 4.2f + 1.8f * h01(style, 9, 0xA19u);
  const float hipY = footY - legH;
  const float torsoTopY = hipY - torsoH;
  const float headCy = torsoTopY - headR * (0.72f + 0.10f * h01(style, 10, 0xA1Au));

  // Leg pose: alternate forward/back to create a cheap walking cycle.
  const float legSep = 1.05f + 0.45f * h01(style, 11, 0xA1Bu);
  const float step = (pose == 0 ? -1.0f : 1.0f) * (0.8f + 0.55f * h01(style, 12, 0xA1Cu));

  const int yFoot = static_cast<int>(std::lround(footY));
  const int yHip = static_cast<int>(std::lround(hipY));
  const int legW = (h01(style, 13, 0xA1Du) > 0.55f) ? 2 : 1;

  const int lx0 = static_cast<int>(std::lround(cx - legSep + step * 0.35f));
  const int rx0 = static_cast<int>(std::lround(cx + legSep - step * 0.35f));

  FillRect(out.color, lx0 - legW / 2, yHip, lx0 + legW / 2, yFoot, Rgba8{pants.r, pants.g, pants.b, 240});
  FillRect(out.color, rx0 - legW / 2, yHip + (pose ? 1 : 0), rx0 + legW / 2, yFoot, Rgba8{pants.r, pants.g, pants.b, 240});

  // Shoes.
  FillRect(out.color, lx0 - legW / 2, yFoot - 1, lx0 + legW / 2, yFoot, Rgba8{shoe.r, shoe.g, shoe.b, 230});
  FillRect(out.color, rx0 - legW / 2, yFoot - 1, rx0 + legW / 2, yFoot, Rgba8{shoe.r, shoe.g, shoe.b, 230});

  // Torso.
  const int tHalfW = static_cast<int>(std::lround(torsoW * 0.5f));
  const int yTorso0 = static_cast<int>(std::lround(torsoTopY));
  const int yTorso1 = static_cast<int>(std::lround(hipY));
  const int xTorso0 = static_cast<int>(std::lround(cx)) - tHalfW;
  const int xTorso1 = static_cast<int>(std::lround(cx)) + tHalfW;
  FillRect(out.color, xTorso0, yTorso0, xTorso1, yTorso1, Rgba8{shirt.r, shirt.g, shirt.b, 245});

  // A small belt seam helps readability.
  const int yBelt = static_cast<int>(std::lround(hipY - 1.0f));
  FillRect(out.color, xTorso0, yBelt, xTorso1, yBelt, Rgba8{pants.r, pants.g, pants.b, 130});

  // Arms: two short strokes. Occasionally add a bag.
  {
    const bool bag = (h01(style, 14, 0xA1Eu) > 0.72f);
    const int yArm = static_cast<int>(std::lround(torsoTopY + torsoH * 0.45f));
    const int axL = xTorso0 - 1;
    const int axR = xTorso1 + 1;

    FillRect(out.color, axL, yArm, axL, yArm + 2, Rgba8{skin.r, skin.g, skin.b, 235});
    FillRect(out.color, axR, yArm, axR, yArm + 2, Rgba8{skin.r, skin.g, skin.b, 235});

    if (bag) {
      const Rgba8 bagC = Mul(pal.overlayIndustrial, 0.92f);
      FillRect(out.color, axR + 1, yArm + 1, axR + 3, yArm + 5, Rgba8{bagC.r, bagC.g, bagC.b, 210});
    }
  }

  // Head.
  FillCircleSoft(out.color, cx, headCy, headR, 0.75f, Rgba8{skin.r, skin.g, skin.b, 245});

  // Hair/hat: a simple darker cap on top.
  {
    const bool hat = (h01(style, 15, 0xA1Fu) > 0.58f);
    const Rgba8 hair = Mul(pal.roadAsphalt2, 0.58f + 0.25f * h01(style, 16, 0xA20u));
    const float r = headR * (hat ? 1.02f : 0.92f);
    FillCircleSoft(out.color, cx, headCy - headR * 0.35f, r, 0.65f, Rgba8{hair.r, hair.g, hair.b, 235});

    if (hat) {
      const Rgba8 brim = Mul(hair, 0.92f);
      FillRect(out.color,
               static_cast<int>(std::lround(cx - headR * 0.95f)),
               static_cast<int>(std::lround(headCy + headR * 0.20f)),
               static_cast<int>(std::lround(cx + headR * 0.95f)),
               static_cast<int>(std::lround(headCy + headR * 0.35f)),
               Rgba8{brim.r, brim.g, brim.b, 210});
    }
  }

  // Lighting + tiny per-pixel variation to avoid flat silhouettes.
  for (int y = 0; y < out.color.height; ++y) {
    for (int x = 0; x < out.color.width; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.color.width) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = out.color.rgba[i + 3u];
      if (a == 0) continue;
      if (a < 160) continue; // skip shadow pixels

      const float nx = (static_cast<float>(x) + 0.5f - cx) / (torsoW + 1.0f);
      const float ny = (static_cast<float>(y) + 0.5f - headCy) / (torsoH + headR * 2.0f + 1.0f);
      float light = SpriteLight(nx, ny);
      light = std::clamp(light + (h01(x, y, 0xB1u) - 0.5f) * 0.08f, 0.72f, 1.22f);

      out.color.rgba[i + 0u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 0u]) * light)));
      out.color.rgba[i + 1u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 1u]) * light)));
      out.color.rgba[i + 2u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 2u]) * light)));
    }
  }

  // Darken boundary pixels to create a crisp outline.
  {
    const int ww = out.color.width;
    const int hh = out.color.height;
    std::vector<std::uint8_t> alpha;
    alpha.resize(static_cast<std::size_t>(ww) * static_cast<std::size_t>(hh));
    for (int y = 0; y < hh; ++y) {
      for (int x = 0; x < ww; ++x) {
        const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) + static_cast<std::size_t>(x)) * 4u;
        alpha[static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) + static_cast<std::size_t>(x)] = out.color.rgba[i + 3u];
      }
    }

    auto aAt = [&](int x, int y) -> std::uint8_t {
      if (x < 0 || y < 0 || x >= ww || y >= hh) return 0;
      return alpha[static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) + static_cast<std::size_t>(x)];
    };

    for (int y = 1; y < hh - 1; ++y) {
      for (int x = 1; x < ww - 1; ++x) {
        const std::uint8_t a0 = aAt(x, y);
        if (a0 < 180) continue; // skip shadow / feather pixels
        const bool edge = (aAt(x - 1, y) < 70) || (aAt(x + 1, y) < 70) || (aAt(x, y - 1) < 70) || (aAt(x, y + 1) < 70);
        if (!edge) continue;
        const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) + static_cast<std::size_t>(x)) * 4u;
        out.color.rgba[i + 0u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 0u]) * 0.55f)));
        out.color.rgba[i + 1u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 1u]) * 0.55f)));
        out.color.rgba[i + 2u] = ClampU8(static_cast<int>(std::lround(static_cast<float>(out.color.rgba[i + 2u]) * 0.55f)));
      }
    }
  }

  // Optional emissive: a tiny “phone” screen that only reads at high zoom.
  if (cfg.includeEmissive && !out.emissive.rgba.empty()) {
    const bool phone = (h01(style, 17, 0xA21u) > 0.60f);
    if (phone) {
      const float px = static_cast<float>(xTorso1 + 1);
      const float py = torsoTopY + torsoH * 0.55f;
      FillCircleSoft(out.emissive, px, py, 1.55f, 1.25f, Rgba8{215, 245, 255, 175}, BlendMode::Additive);
      FillCircleSoft(out.emissive, px, py, 3.25f, 2.8f, Rgba8{215, 245, 255, 45}, BlendMode::Additive);
    } else {
      // Avoid emitting (and uploading) a fully-transparent emissive texture when it's unused.
      out.emissive = RgbaImage{};
    }
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
  case GfxPropKind::Pedestrian:
    MakePedestrian(v, seedv, cfg, pal, out);
    return true;
  default:
    break;
  }

  outError = "unknown prop kind";
  return false;
}

} // namespace isocity
