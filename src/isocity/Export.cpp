#include "isocity/Export.hpp"
#include "isocity/DeterministicMath.hpp"

#include "isocity/GfxTilesetAtlas.hpp"
#include "isocity/Random.hpp"
#include "isocity/ZoneMetrics.hpp"
#include "isocity/ZoneAccess.hpp"

#include "isocity/Services.hpp"

#include "isocity/FloodRisk.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/NoisePollution.hpp"
#include "isocity/HeatIsland.hpp"
#include "isocity/AirPollution.hpp"
#include "isocity/RunoffPollution.hpp"
#include "isocity/RunoffMitigation.hpp"
#include "isocity/SolarPotential.hpp"
#include "isocity/SkyView.hpp"
#include "isocity/EnergyModel.hpp"
#include "isocity/CarbonModel.hpp"
#include "isocity/CrimeModel.hpp"
#include "isocity/TrafficSafety.hpp"
#include "isocity/TransitAccessibility.hpp"
#include "isocity/FireRisk.hpp"
#include "isocity/LandUseMix.hpp"
#include "isocity/Walkability.hpp"
#include "isocity/RoadHealth.hpp"
#include "isocity/Livability.hpp"
#include "isocity/HotspotAnalysis.hpp"

#include "isocity/JobOpportunity.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline float Lerp(float a, float b, float t)
{
  return a + (b - a) * std::clamp(t, 0.0f, 1.0f);
}

inline std::uint8_t LerpU8(std::uint8_t a, std::uint8_t b, float t)
{
  const float v = Lerp(static_cast<float>(a), static_cast<float>(b), t);
  return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0f, 255.0f)));
}

inline float SmoothStep(float edge0, float edge1, float x)
{
  if (edge0 == edge1) return (x < edge0) ? 0.0f : 1.0f;
  const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline std::uint8_t ToByte(float v)
{
  const float c = std::clamp(v, 0.0f, 255.0f);
  return static_cast<std::uint8_t>(std::lround(c));
}

inline void AlphaBlendPixel(PpmImage& dst, int x, int y,
                            std::uint8_t sr, std::uint8_t sg, std::uint8_t sb, std::uint8_t sa,
                            float rgbMul = 1.0f)
{
  if (sa == 0) return;
  if (x < 0 || y < 0 || x >= dst.width || y >= dst.height) return;
  const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(x)) * 3u;

  int rr = static_cast<int>(std::lround(static_cast<float>(sr) * rgbMul));
  int gg = static_cast<int>(std::lround(static_cast<float>(sg) * rgbMul));
  int bb = static_cast<int>(std::lround(static_cast<float>(sb) * rgbMul));
  rr = std::clamp(rr, 0, 255);
  gg = std::clamp(gg, 0, 255);
  bb = std::clamp(bb, 0, 255);

  const int a = static_cast<int>(sa);
  const int inv = 255 - a;
  const int dr = static_cast<int>(dst.rgb[di + 0]);
  const int dg = static_cast<int>(dst.rgb[di + 1]);
  const int db = static_cast<int>(dst.rgb[di + 2]);

  dst.rgb[di + 0] = static_cast<std::uint8_t>((rr * a + dr * inv + 127) / 255);
  dst.rgb[di + 1] = static_cast<std::uint8_t>((gg * a + dg * inv + 127) / 255);
  dst.rgb[di + 2] = static_cast<std::uint8_t>((bb * a + db * inv + 127) / 255);
}

inline void MultiplyBlendPixel(PpmImage& dst, int x, int y, std::uint8_t maskA, float strength)
{
  if (maskA == 0) return;
  if (strength <= 0.0f) return;
  if (x < 0 || y < 0 || x >= dst.width || y >= dst.height) return;

  const float s = std::clamp(strength, 0.0f, 1.0f);
  const int k = std::clamp<int>(static_cast<int>(std::lround(static_cast<float>(maskA) * s)), 0, 255);
  const int mul = 255 - k;
  const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(x)) * 3u;

  dst.rgb[di + 0] = static_cast<std::uint8_t>((static_cast<int>(dst.rgb[di + 0]) * mul + 127) / 255);
  dst.rgb[di + 1] = static_cast<std::uint8_t>((static_cast<int>(dst.rgb[di + 1]) * mul + 127) / 255);
  dst.rgb[di + 2] = static_cast<std::uint8_t>((static_cast<int>(dst.rgb[di + 2]) * mul + 127) / 255);
}

struct TilesetLightingState {
  const RgbaImage* normalAtlas = nullptr;
  bool enabled = false;

  float lx = 0.0f;
  float ly = 0.0f;
  float lz = 1.0f;
  float flatDot = 1.0f; // dot((0,0,1), L)

  float ambient = 0.35f;
  float diffuse = 0.65f;
  float strength = 0.0f; // blends towards normal lighting
};

inline void DecodeNormalRGB(std::uint8_t r, std::uint8_t g, std::uint8_t b, float& nx, float& ny, float& nz)
{
  // Tangent-space [-1,1] decode.
  nx = static_cast<float>(r) / 127.5f - 1.0f;
  ny = static_cast<float>(g) / 127.5f - 1.0f;
  nz = static_cast<float>(b) / 127.5f - 1.0f;
}

inline float ComputeTilesetNormalLightingMul(const TilesetLightingState& st, float nx, float ny, float nz)
{
  // Use a normalized range where a flat normal yields 1.0.
  const float ndotl = std::max(0.0f, nx * st.lx + ny * st.ly + nz * st.lz);
  const float base = st.ambient + st.diffuse * (ndotl / std::max(1.0e-4f, st.flatDot));
  return Lerp(1.0f, std::clamp(base, 0.0f, 2.0f), st.strength);
}

inline void AdditiveBlendPixel(PpmImage& dst, int x, int y,
                               std::uint8_t sr, std::uint8_t sg, std::uint8_t sb, std::uint8_t sa,
                               float intensity = 1.0f)
{
  if (sa == 0) return;
  if (x < 0 || y < 0 || x >= dst.width || y >= dst.height) return;
  const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(x)) * 3u;

  const float a = (static_cast<float>(sa) / 255.0f) * std::clamp(intensity, 0.0f, 8.0f);
  int dr = static_cast<int>(dst.rgb[di + 0]);
  int dg = static_cast<int>(dst.rgb[di + 1]);
  int db = static_cast<int>(dst.rgb[di + 2]);

  dr += static_cast<int>(std::lround(static_cast<float>(sr) * a));
  dg += static_cast<int>(std::lround(static_cast<float>(sg) * a));
  db += static_cast<int>(std::lround(static_cast<float>(sb) * a));

  dst.rgb[di + 0] = static_cast<std::uint8_t>(std::clamp(dr, 0, 255));
  dst.rgb[di + 1] = static_cast<std::uint8_t>(std::clamp(dg, 0, 255));
  dst.rgb[di + 2] = static_cast<std::uint8_t>(std::clamp(db, 0, 255));
}

inline void BlitAtlasSpriteOver(PpmImage& dst, const RgbaImage& atlas, const GfxAtlasEntry& e, int dstX, int dstY, float rgbMul)
{
  if (atlas.width <= 0 || atlas.height <= 0) return;
  if (atlas.rgba.empty()) return;

  const int w = e.w;
  const int h = e.h;
  for (int y = 0; y < h; ++y) {
    const int sy = e.y + y;
    if (sy < 0 || sy >= atlas.height) continue;
    for (int x = 0; x < w; ++x) {
      const int sx = e.x + x;
      if (sx < 0 || sx >= atlas.width) continue;
      const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(atlas.width) + static_cast<std::size_t>(sx)) * 4u;
      const std::uint8_t sr = atlas.rgba[si + 0];
      const std::uint8_t sg = atlas.rgba[si + 1];
      const std::uint8_t sb = atlas.rgba[si + 2];
      const std::uint8_t sa = atlas.rgba[si + 3];
      AlphaBlendPixel(dst, dstX + x, dstY + y, sr, sg, sb, sa, rgbMul);
    }
  }
}

inline void BlitAtlasSpriteOverLit(PpmImage& dst, const RgbaImage& atlas, const TilesetLightingState* lighting,
                                  const GfxAtlasEntry& e, int dstX, int dstY, float rgbMul)
{
  if (!lighting || !lighting->enabled || !lighting->normalAtlas) {
    BlitAtlasSpriteOver(dst, atlas, e, dstX, dstY, rgbMul);
    return;
  }
  const RgbaImage& nrm = *lighting->normalAtlas;
  if (nrm.width != atlas.width || nrm.height != atlas.height) {
    BlitAtlasSpriteOver(dst, atlas, e, dstX, dstY, rgbMul);
    return;
  }

  const int w = e.w;
  const int h = e.h;
  for (int y = 0; y < h; ++y) {
    const int sy = e.y + y;
    if (sy < 0 || sy >= atlas.height) continue;
    for (int x = 0; x < w; ++x) {
      const int sx = e.x + x;
      if (sx < 0 || sx >= atlas.width) continue;
      const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(atlas.width) + static_cast<std::size_t>(sx)) * 4u;
      const std::uint8_t sr = atlas.rgba[si + 0];
      const std::uint8_t sg = atlas.rgba[si + 1];
      const std::uint8_t sb = atlas.rgba[si + 2];
      const std::uint8_t sa = atlas.rgba[si + 3];
      if (sa == 0) continue;

      float nx = 0.0f, ny = 0.0f, nz = 1.0f;
      DecodeNormalRGB(nrm.rgba[si + 0], nrm.rgba[si + 1], nrm.rgba[si + 2], nx, ny, nz);
      const float shade = ComputeTilesetNormalLightingMul(*lighting, nx, ny, nz);
      AlphaBlendPixel(dst, dstX + x, dstY + y, sr, sg, sb, sa, rgbMul * shade);
    }
  }
}

inline void BlitAtlasSpriteShadowMultiply(PpmImage& dst, const RgbaImage& shadowAtlas, const GfxAtlasEntry& e,
                                         int dstX, int dstY, float strength)
{
  if (strength <= 0.0f) return;
  if (shadowAtlas.width <= 0 || shadowAtlas.height <= 0) return;
  if (shadowAtlas.rgba.empty()) return;

  const int w = e.w;
  const int h = e.h;
  for (int y = 0; y < h; ++y) {
    const int sy = e.y + y;
    if (sy < 0 || sy >= shadowAtlas.height) continue;
    for (int x = 0; x < w; ++x) {
      const int sx = e.x + x;
      if (sx < 0 || sx >= shadowAtlas.width) continue;
      const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(shadowAtlas.width) + static_cast<std::size_t>(sx)) * 4u;
      const std::uint8_t sa = shadowAtlas.rgba[si + 3];
      MultiplyBlendPixel(dst, dstX + x, dstY + y, sa, strength);
    }
  }
}

inline void BlitAtlasSpriteAdditive(PpmImage& dst, const RgbaImage& atlas, const GfxAtlasEntry& e, int dstX, int dstY, float intensity)
{
  if (atlas.width <= 0 || atlas.height <= 0) return;
  if (atlas.rgba.empty()) return;

  const int w = e.w;
  const int h = e.h;
  for (int y = 0; y < h; ++y) {
    const int sy = e.y + y;
    if (sy < 0 || sy >= atlas.height) continue;
    for (int x = 0; x < w; ++x) {
      const int sx = e.x + x;
      if (sx < 0 || sx >= atlas.width) continue;
      const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(atlas.width) + static_cast<std::size_t>(sx)) * 4u;
      const std::uint8_t sr = atlas.rgba[si + 0];
      const std::uint8_t sg = atlas.rgba[si + 1];
      const std::uint8_t sb = atlas.rgba[si + 2];
      const std::uint8_t sa = atlas.rgba[si + 3];
      AdditiveBlendPixel(dst, dstX + x, dstY + y, sr, sg, sb, sa, intensity);
    }
  }
}

inline void SetPixel(std::vector<std::uint8_t>& rgb, int w, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
  rgb[idx + 0] = r;
  rgb[idx + 1] = g;
  rgb[idx + 2] = b;
}

inline void GetPixel(const std::vector<std::uint8_t>& rgb, int w, int x, int y, std::uint8_t& outR, std::uint8_t& outG,
                     std::uint8_t& outB)
{
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
  outR = rgb[idx + 0];
  outG = rgb[idx + 1];
  outB = rgb[idx + 2];
}

inline void BlendPixel(std::vector<std::uint8_t>& rgb, int w, int x, int y, std::uint8_t sr, std::uint8_t sg, std::uint8_t sb,
                       float a01)
{
  const float a = std::clamp(a01, 0.0f, 1.0f);
  if (a <= 0.0f) return;
  if (a >= 1.0f) {
    SetPixel(rgb, w, x, y, sr, sg, sb);
    return;
  }

  std::uint8_t dr, dg, db;
  GetPixel(rgb, w, x, y, dr, dg, db);

  const float ia = 1.0f - a;
  const std::uint8_t rr = ToByte(static_cast<float>(dr) * ia + static_cast<float>(sr) * a);
  const std::uint8_t gg = ToByte(static_cast<float>(dg) * ia + static_cast<float>(sg) * a);
  const std::uint8_t bb = ToByte(static_cast<float>(db) * ia + static_cast<float>(sb) * a);
  SetPixel(rgb, w, x, y, rr, gg, bb);
}

inline void AdditivePixel(std::vector<std::uint8_t>& rgb, int w, int x, int y, std::uint8_t sr, std::uint8_t sg, std::uint8_t sb,
                          float a01)
{
  const float a = std::clamp(a01, 0.0f, 1.0f);
  if (a <= 0.0f) return;

  std::uint8_t dr, dg, db;
  GetPixel(rgb, w, x, y, dr, dg, db);

  const float rr = std::min(255.0f, static_cast<float>(dr) + static_cast<float>(sr) * a);
  const float gg = std::min(255.0f, static_cast<float>(dg) + static_cast<float>(sg) * a);
  const float bb = std::min(255.0f, static_cast<float>(db) + static_cast<float>(sb) * a);

  SetPixel(rgb, w, x, y, ToByte(rr), ToByte(gg), ToByte(bb));
}

inline void AddGlow(PpmImage& img, int cx, int cy, float radiusPx, std::uint8_t r, std::uint8_t g, std::uint8_t b,
                    float intensity01)
{
  if (img.width <= 0 || img.height <= 0) return;
  const float rad = std::max(0.5f, radiusPx);
  const int ir = static_cast<int>(std::ceil(rad));
  const int minX = std::max(0, cx - ir);
  const int maxX = std::min(img.width - 1, cx + ir);
  const int minY = std::max(0, cy - ir);
  const int maxY = std::min(img.height - 1, cy + ir);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const float dx = static_cast<float>(x - cx);
      const float dy = static_cast<float>(y - cy);
      const float d2 = dx * dx + dy * dy;
      if (d2 > rad * rad) continue;
      const float d = std::sqrt(d2);
      const float t = 1.0f - (d / rad);
      // Quadratic falloff looks reasonably "glowy" without being too expensive.
      const float a = intensity01 * (t * t);
      AdditivePixel(img.rgb, img.width, x, y, r, g, b, a);
    }
  }
}

// Convert a uint32 hash -> [0,1) with 24-bit precision.
inline float Frac01(std::uint32_t h)
{
  return static_cast<float>(h & 0x00FFFFFFu) / static_cast<float>(1u << 24);
}

// Smoothed value noise in 2D (deterministic via HashCoords32).
inline float ValueNoise2D(float x, float y, std::uint32_t seed)
{
  const int ix0 = static_cast<int>(std::floor(x));
  const int iy0 = static_cast<int>(std::floor(y));
  const int ix1 = ix0 + 1;
  const int iy1 = iy0 + 1;

  const float fx = x - static_cast<float>(ix0);
  const float fy = y - static_cast<float>(iy0);

  const float sx = fx * fx * (3.0f - 2.0f * fx);
  const float sy = fy * fy * (3.0f - 2.0f * fy);

  const float v00 = Frac01(HashCoords32(ix0, iy0, seed));
  const float v10 = Frac01(HashCoords32(ix1, iy0, seed));
  const float v01 = Frac01(HashCoords32(ix0, iy1, seed));
  const float v11 = Frac01(HashCoords32(ix1, iy1, seed));

  const float vx0 = Lerp(v00, v10, sx);
  const float vx1 = Lerp(v01, v11, sx);
  return Lerp(vx0, vx1, sy);
}

// Fractal Brownian Motion (fBm) over value noise: low-cost "cloudy" noise.
inline float FBm2D(float x, float y, std::uint32_t seed, int octaves = 4)
{
  float sum = 0.0f;
  float amp = 0.55f;
  float freq = 1.0f;
  float norm = 0.0f;
  for (int i = 0; i < octaves; ++i) {
    sum += ValueNoise2D(x * freq, y * freq, seed ^ (0x9E3779B9u * static_cast<std::uint32_t>(i))) * amp;
    norm += amp;
    amp *= 0.55f;
    freq *= 2.0f;
  }
  if (norm > 1.0e-6f) sum /= norm;
  return std::clamp(sum, 0.0f, 1.0f);
}

struct DayNightState {
  float phase = 0.25f;   // 0..1
  float sun = 1.0f;      // -1..1
  float day = 1.0f;      // 0..1
  float night = 0.0f;    // 0..1
  float twilight = 0.0f; // 0..1
};

inline DayNightState ComputeDayNightState(const IsoOverviewConfig::DayNightConfig& cfg)
{
  DayNightState st{};
  if (!cfg.enabled) return st;

  st.phase = Wrap01(cfg.phase01);

  st.sun = FastSinRad(st.phase * kTwoPiF);

  // Mirror the in-app behaviour: day turns on slightly before the horizon.
  st.day = SmoothStep(-0.18f, 0.22f, st.sun);
  st.night = 1.0f - st.day;

  const float absSun = std::fabs(st.sun);
  st.twilight = SmoothStep(0.28f, 0.0f, absSun);
  return st;
}

inline void ApplyOvercastGrade(PpmImage& img, float overcast01, bool snowMode)
{
  const float o = std::clamp(overcast01, 0.0f, 1.0f);
  if (o <= 0.001f) return;

  // Slightly different grade for snow (cooler / brighter).
  const std::uint8_t tr = snowMode ? 175 : 85;
  const std::uint8_t tg = snowMode ? 198 : 95;
  const std::uint8_t tb = snowMode ? 220 : 108;

  // Desaturate + soften contrast via a gentle blend toward a cool grey.
  const float a = 0.32f * o;
  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      BlendPixel(img.rgb, img.width, x, y, tr, tg, tb, a);
    }
  }
}

inline void ApplyFogGradient(PpmImage& img, float fog01, std::uint8_t fr, std::uint8_t fg, std::uint8_t fb)
{
  const float f = std::clamp(fog01, 0.0f, 1.0f);
  if (f <= 0.001f) return;

  // Simple top-of-image haze; far tiles sit toward smaller Y in the iso projection.
  const float invH = (img.height > 1) ? (1.0f / static_cast<float>(img.height - 1)) : 0.0f;
  for (int y = 0; y < img.height; ++y) {
    const float dist01 = 1.0f - static_cast<float>(y) * invH; // top=1 (far), bottom=0 (near)
    const float a = f * (dist01 * dist01);
    if (a <= 0.001f) continue;
    for (int x = 0; x < img.width; ++x) {
      BlendPixel(img.rgb, img.width, x, y, fr, fg, fb, a);
    }
  }
}

inline void ApplyDayNightGrade(PpmImage& img, const DayNightState& dn, const IsoOverviewConfig::DayNightConfig& cfg)
{
  if (!cfg.enabled) return;

  const float n = std::clamp(dn.night * std::clamp(cfg.nightDarken, 0.0f, 1.0f), 0.0f, 1.0f);
  const float tw = std::clamp(dn.twilight * std::clamp(cfg.duskTint, 0.0f, 1.0f), 0.0f, 1.0f);

  const float aNight = n * (210.0f / 255.0f);
  const float aDusk = tw * (110.0f / 255.0f) * (1.0f - n);

  if (aNight > 0.001f) {
    for (int y = 0; y < img.height; ++y) {
      for (int x = 0; x < img.width; ++x) {
        BlendPixel(img.rgb, img.width, x, y, 8, 12, 45, aNight);
      }
    }
  }

  if (aDusk > 0.001f) {
    for (int y = 0; y < img.height; ++y) {
      for (int x = 0; x < img.width; ++x) {
        BlendPixel(img.rgb, img.width, x, y, 255, 150, 90, aDusk);
      }
    }
  }
}

inline void MulPixel(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, float m)
{
  r = ToByte(static_cast<float>(r) * m);
  g = ToByte(static_cast<float>(g) * m);
  b = ToByte(static_cast<float>(b) * m);
}

inline void TerrainBaseColor(const Tile& t, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  switch (t.terrain) {
  case Terrain::Water:
    r = 18; g = 70; b = 180;
    break;
  case Terrain::Sand:
    r = 198; g = 182; b = 120;
    break;
  case Terrain::Grass:
  default:
    r = 60; g = 170; b = 70;
    break;
  }
}

inline void OverlayColor(const Tile& t, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  switch (t.overlay) {
  case Overlay::Road: {
    // Slightly different tint for bridges.
    if (t.terrain == Terrain::Water) { r = 210; g = 210; b = 220; }
    else { r = 120; g = 120; b = 120; }

    // Road class shading (level 1..3).
    const int lvl = std::clamp(static_cast<int>(t.level), 1, 3);
    const float m = 0.85f + 0.10f * static_cast<float>(lvl - 1);
    MulPixel(r, g, b, m);
  } break;
  case Overlay::Residential:
    r = 70; g = 210; b = 90;
    break;
  case Overlay::Commercial:
    r = 70; g = 140; b = 230;
    break;
  case Overlay::Industrial:
    r = 220; g = 170; b = 70;
    break;
  case Overlay::Park:
    r = 40; g = 140; b = 60;
    break;
  case Overlay::School:
    r = 80; g = 150; b = 255;
    break;
  case Overlay::Hospital:
    r = 255; g = 90; b = 90;
    break;
  case Overlay::PoliceStation:
    r = 130; g = 120; b = 230;
    break;
  case Overlay::FireStation:
    r = 255; g = 120; b = 60;
    break;
  case Overlay::None:
  default:
    // Keep base terrain.
    break;
  }
}

inline void HeatRampRedYellowGreen(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  // 0 -> red, 0.5 -> yellow, 1 -> green
  if (t <= 0.5f) {
    r = 255;
    g = ToByte(255.0f * (t * 2.0f));
    b = 0;
  } else {
    r = ToByte(255.0f * (1.0f - (t - 0.5f) * 2.0f));
    g = 255;
    b = 0;
  }
}

// Diverging ramp useful for hotspot/coldspot visualization.
// 0 -> blue (cold), 0.5 -> white (neutral), 1 -> red (hot)
inline void DivergingBlueWhiteRed(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  if (t <= 0.5f) {
    const float u = (t <= 0.0f) ? 0.0f : (t / 0.5f);
    r = ToByte(255.0f * u);
    g = ToByte(255.0f * u);
    b = 255;
  } else {
    const float u = (t >= 1.0f) ? 1.0f : ((t - 0.5f) / 0.5f);
    r = 255;
    g = ToByte(255.0f * (1.0f - u));
    b = ToByte(255.0f * (1.0f - u));
  }
}

inline void HeatRampPurple(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  r = ToByte(80.0f + 175.0f * t);
  g = ToByte(30.0f + 70.0f * t);
  b = ToByte(90.0f + 165.0f * t);
}

// Depth-like ramp used for flood overlays.
// 0 -> black (no flood), 1 -> deep blue.
inline void HeatRampBlue(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  const float vis = SmoothStep(0.0f, 0.02f, t);

  // Shallow (near sea level): light blue; Deep: dark saturated blue.
  constexpr float shallowR = 180.0f, shallowG = 220.0f, shallowB = 255.0f;
  constexpr float deepR = 0.0f, deepG = 30.0f, deepB = 120.0f;

  const float rr = (shallowR + (deepR - shallowR) * t) * vis;
  const float gg = (shallowG + (deepG - shallowG) * t) * vis;
  const float bb = (shallowB + (deepB - shallowB) * t) * vis;

  r = ToByte(rr);
  g = ToByte(gg);
  b = ToByte(bb);
}

inline void DistrictPalette(std::uint8_t id, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  // 8 distinct-ish colors (matches kDistrictCount).
  // Chosen to be readable on dark backgrounds and in PPM viewers.
  static constexpr std::uint8_t k[8][3] = {
      {220, 220, 220}, // 0 (default) - light gray
      {255, 120, 120}, // 1 - red
      {120, 210, 120}, // 2 - green
      {120, 160, 255}, // 3 - blue
      {255, 210, 120}, // 4 - orange
      {200, 120, 255}, // 5 - purple
      {120, 230, 230}, // 6 - cyan
      {255, 120, 220}, // 7 - pink
  };
  const std::uint8_t idx = static_cast<std::uint8_t>(id % 8u);
  r = k[idx][0]; g = k[idx][1]; b = k[idx][2];
}

inline std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline void BuildHeightFieldAndDrainMask(const World& world, std::vector<float>& heights,
                                       std::vector<std::uint8_t>& drainMask)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n =
      static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  heights.assign(n, 0.0f);
  drainMask.assign(n, 0);

  if (w <= 0 || h <= 0) return;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const std::size_t i = FlatIdx(x, y, w);
      heights[i] = t.height;
      // Treat existing water bodies as drains/outlets for depression fill.
      if (t.terrain == Terrain::Water) {
        drainMask[i] = 1;
      }
    }
  }
}

inline float InferCoastalSeaLevel(const World& world)
{
  // We infer sea level by looking at edge-connected "ocean" water tiles (Terrain::Water).
  // This avoids inland lakes artificially raising the sea threshold.
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0.35f;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  std::vector<std::uint8_t> visited(n, std::uint8_t{0});
  std::vector<std::size_t> stack;
  stack.reserve(static_cast<std::size_t>(w + h) * 2u);

  auto pushIfOcean = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    const std::size_t i = FlatIdx(x, y, w);
    if (visited[i]) return;
    const Tile& t = world.at(x, y);
    if (t.terrain != Terrain::Water) return;
    visited[i] = 1;
    stack.push_back(i);
  };

  // Seed with edge water tiles.
  for (int x = 0; x < w; ++x) {
    pushIfOcean(x, 0);
    pushIfOcean(x, h - 1);
  }
  for (int y = 0; y < h; ++y) {
    pushIfOcean(0, y);
    pushIfOcean(w - 1, y);
  }

  const bool anyEdgeWater = !stack.empty();
  float seaLevel = 0.0f;

  while (!stack.empty()) {
    const std::size_t i = stack.back();
    stack.pop_back();

    const int x = static_cast<int>(i % static_cast<std::size_t>(w));
    const int y = static_cast<int>(i / static_cast<std::size_t>(w));

    seaLevel = std::max(seaLevel, world.at(x, y).height);

    pushIfOcean(x - 1, y);
    pushIfOcean(x + 1, y);
    pushIfOcean(x, y - 1);
    pushIfOcean(x, y + 1);
  }

  if (anyEdgeWater) return seaLevel;

  // Fallback: if there is no edge-connected water at all, use max water height (inland lakes),
  // and if there is no water, fall back to the in-game default.
  bool anyWater = false;
  float maxWaterH = 0.0f;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) {
        anyWater = true;
        maxWaterH = std::max(maxWaterH, t.height);
      }
    }
  }

  return anyWater ? maxWaterH : 0.35f;
}


struct TileColorContext {
  int w = 0;
  int h = 0;

  const LandValueResult* landValue = nullptr;
  const TrafficResult* traffic = nullptr;
  const GoodsResult* goods = nullptr;
  const ServicesResult* services = nullptr;
  const NoiseResult* noise = nullptr;
  const LandUseMixResult* landUseMix = nullptr;
  const HeatIslandResult* heatIsland = nullptr;
  const AirPollutionResult* airPollution = nullptr;
  const RunoffPollutionResult* runoff = nullptr;
  const RunoffMitigationResult* runoffMitigation = nullptr;
  const SolarPotentialResult* solar = nullptr;
  const SkyViewResult* skyView = nullptr;
  const EnergyModelResult* energy = nullptr;
  const CarbonModelResult* carbon = nullptr;
  const CrimeModelResult* crime = nullptr;
  const TrafficSafetyResult* trafficSafety = nullptr;
  const TransitAccessibilityResult* transit = nullptr;
  const FireRiskResult* fireRisk = nullptr;
  const WalkabilityResult* walkability = nullptr;
  const JobOpportunityResult* jobs = nullptr;
  const RoadHealthResult* roadHealth = nullptr;
  const LivabilityResult* livability = nullptr;
  const HotspotResult* livabilityHotspot = nullptr;
  const HotspotResult* interventionHotspot = nullptr;

  std::uint16_t maxTraffic = 0;
  std::uint16_t maxGoodsTraffic = 0;

  // Optional derived fields for heightfield-driven layers.
  const std::vector<float>* seaFloodDepth = nullptr;
  float seaFloodMaxDepth = 0.0f;
  float seaLevel = 0.0f;

  const std::vector<float>* pondingDepth = nullptr;
  float pondingMaxDepth = 0.0f;

  // Optional per-zone road access mapping (used by zone pressure layers).
  const ZoneAccessMap* zoneAccess = nullptr;
};

inline TileColorContext MakeTileColorContext(const World& world, const LandValueResult* landValue, const TrafficResult* traffic,
                                            const GoodsResult* goods)
{
  TileColorContext ctx{};
  ctx.w = world.width();
  ctx.h = world.height();
  ctx.landValue = landValue;
  ctx.traffic = traffic;
  ctx.goods = goods;

  // Precompute maxima for heatmaps when available.
  if (traffic && !traffic->roadTraffic.empty()) {
    ctx.maxTraffic = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (ctx.maxTraffic == 0) {
      for (std::uint16_t v : traffic->roadTraffic) ctx.maxTraffic = std::max(ctx.maxTraffic, v);
    }
  }

  if (goods && !goods->roadGoodsTraffic.empty()) {
    ctx.maxGoodsTraffic = static_cast<std::uint16_t>(std::clamp(goods->maxRoadGoodsTraffic, 0, 65535));
    if (ctx.maxGoodsTraffic == 0) {
      for (std::uint16_t v : goods->roadGoodsTraffic) ctx.maxGoodsTraffic = std::max(ctx.maxGoodsTraffic, v);
    }
  }

  return ctx;
}

inline void ComputeTileColor(const World& world, int x, int y, ExportLayer layer, const TileColorContext& ctx,
                             std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const Tile& t = world.at(x, y);

  r = 0;
  g = 0;
  b = 0;
  TerrainBaseColor(t, r, g, b);

  // Height shading for terrain-ish layers.
  const float shade = 0.72f + 0.28f * Clamp01(t.height);

  switch (layer) {
  case ExportLayer::Terrain: {
    MulPixel(r, g, b, shade);
  } break;

  case ExportLayer::Overlay: {
    MulPixel(r, g, b, shade);
    std::uint8_t orr = r, org = g, orb = b;
    OverlayColor(t, orr, org, orb);
    // If overlay == None, OverlayColor leaves it as base. If it's a real overlay, overwrite.
    if (t.overlay != Overlay::None) {
      r = orr; g = org; b = orb;
    }
  } break;

  case ExportLayer::Height: {
    const std::uint8_t c = ToByte(255.0f * Clamp01(t.height));
    r = c; g = c; b = c;
  } break;

  case ExportLayer::LandValue: {
    if (ctx.landValue && ctx.landValue->w == ctx.w && ctx.landValue->h == ctx.h && !ctx.landValue->value.empty()) {
      const float v = ctx.landValue->value[FlatIdx(x, y, ctx.w)];
      HeatRampRedYellowGreen(v, r, g, b);
    } else {
      // Fallback: terrain with height shading.
      MulPixel(r, g, b, shade);
    }
  } break;

  case ExportLayer::Traffic: {
    // Background: terrain.
    MulPixel(r, g, b, shade);
    if (ctx.traffic && ctx.traffic->roadTraffic.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Road) {
      const std::uint16_t v = ctx.traffic->roadTraffic[FlatIdx(x, y, ctx.w)];
      const float t01 = (ctx.maxTraffic > 0) ? Clamp01(static_cast<float>(v) / static_cast<float>(ctx.maxTraffic)) : 0.0f;
      std::uint8_t hr, hg, hb;
      HeatRampRedYellowGreen(t01, hr, hg, hb);
      // Blend a bit so roads still show their context.
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }
  } break;

  case ExportLayer::TrafficCrashRisk: {
    // Crash risk proxy on roads.
    MulPixel(r, g, b, shade);

    const std::size_t nTiles = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
    if (ctx.trafficSafety && ctx.trafficSafety->risk01.size() == nTiles && t.overlay == Overlay::Road) {
      const float v01 = Clamp01(ctx.trafficSafety->risk01[FlatIdx(x, y, ctx.w)]);
      std::uint8_t hr, hg, hb;
      // High risk => red.
      HeatRampRedYellowGreen(1.0f - v01, hr, hg, hb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }
  } break;

  case ExportLayer::TrafficCrashExposure: {
    // Neighborhood exposure to nearby road crash risk.
    MulPixel(r, g, b, shade);

    const std::size_t nTiles = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
    if (ctx.trafficSafety && ctx.trafficSafety->exposure01.size() == nTiles) {
      const float v01 = Clamp01(ctx.trafficSafety->exposure01[FlatIdx(x, y, ctx.w)]);
      std::uint8_t hr, hg, hb;
      HeatRampRedYellowGreen(1.0f - v01, hr, hg, hb);
      const float a = 0.85f;
      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    // Keep overlay context visible (roads/parks/zones/civic).
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::TrafficCrashPriority: {
    // Resident-weighted intervention priority.
    MulPixel(r, g, b, shade);

    const std::size_t nTiles = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
    if (ctx.trafficSafety && ctx.trafficSafety->priority01.size() == nTiles) {
      const float v01 = Clamp01(ctx.trafficSafety->priority01[FlatIdx(x, y, ctx.w)]);
      std::uint8_t hr, hg, hb;
      HeatRampRedYellowGreen(1.0f - v01, hr, hg, hb);
      const float a = 0.85f;
      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::GoodsTraffic: {
    MulPixel(r, g, b, shade);
    if (ctx.goods && ctx.goods->roadGoodsTraffic.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Road) {
      const std::uint16_t v = ctx.goods->roadGoodsTraffic[FlatIdx(x, y, ctx.w)];
      const float t01 = (ctx.maxGoodsTraffic > 0) ? Clamp01(static_cast<float>(v) / static_cast<float>(ctx.maxGoodsTraffic)) : 0.0f;
      std::uint8_t hr, hg, hb;
      HeatRampPurple(t01, hr, hg, hb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }
  } break;

  case ExportLayer::GoodsFill: {
    MulPixel(r, g, b, shade);
    if (ctx.goods && ctx.goods->commercialFill.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Commercial) {
      const std::uint8_t fill = ctx.goods->commercialFill[FlatIdx(x, y, ctx.w)];
      const float t01 = Clamp01(static_cast<float>(fill) / 255.0f);
      HeatRampRedYellowGreen(t01, r, g, b);
    }
  } break;

  case ExportLayer::District: {
    DistrictPalette(t.district, r, g, b);
    // Darken water a bit so coastlines pop.
    if (t.terrain == Terrain::Water) {
      MulPixel(r, g, b, 0.7f);
    }
  } break;

    case ExportLayer::FloodDepth: {
    // Sea-level coastal flooding depth. We prefer a precomputed field (if the caller provided one),
    // but keep a small fallback so exports remain robust even when derived data isn't passed in.
    float depth = 0.0f;
    const std::size_t i = FlatIdx(x, y, ctx.w);

    if (ctx.seaFloodDepth && i < ctx.seaFloodDepth->size()) {
      depth = (*ctx.seaFloodDepth)[i];
    } else {
      const float sea = (ctx.seaLevel > 1e-6f) ? ctx.seaLevel : 0.35f;
      depth = std::max(0.0f, sea - t.height);
    }

    float denom = (ctx.seaFloodMaxDepth > 1e-6f) ? ctx.seaFloodMaxDepth : 0.0f;
    if (denom <= 1e-6f) denom = (ctx.seaLevel > 1e-6f) ? ctx.seaLevel : 0.0f;

    const float depth01 = (denom > 1e-6f) ? Clamp01(depth / denom) : 0.0f;
    HeatRampBlue(depth01, r, g, b);
  } break;

  case ExportLayer::PondingDepth: {
    // Priority-Flood depression-fill depth ("ponding potential").
    float depth = 0.0f;
    const std::size_t i = FlatIdx(x, y, ctx.w);
    if (ctx.pondingDepth && i < ctx.pondingDepth->size()) {
      depth = (*ctx.pondingDepth)[i];
    }

    const float denom = (ctx.pondingMaxDepth > 1e-6f) ? ctx.pondingMaxDepth : 0.0f;
    const float depth01 = (denom > 1e-6f) ? Clamp01(depth / denom) : 0.0f;
    HeatRampBlue(depth01, r, g, b);
  } break;

  case ExportLayer::ServicesOverall:
  case ExportLayer::ServicesEducation:
  case ExportLayer::ServicesHealth:
  case ExportLayer::ServicesSafety: {
    // Public services / civic accessibility satisfaction.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    // Always highlight facility tiles so it's obvious where supply comes from.
    if (t.overlay == Overlay::School || t.overlay == Overlay::Hospital ||
        t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation) {
      OverlayColor(t, r, g, b);
      break;
    }

    const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                         t.overlay == Overlay::Industrial);
    if (!isZone) break;

    if (ctx.services) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);

      const std::vector<float>* field = nullptr;
      if (layer == ExportLayer::ServicesOverall) field = &ctx.services->overall;
      else if (layer == ExportLayer::ServicesEducation) field = &ctx.services->education;
      else if (layer == ExportLayer::ServicesHealth) field = &ctx.services->health;
      else if (layer == ExportLayer::ServicesSafety) field = &ctx.services->safety;

      if (field && field->size() == n && i < field->size()) {
        const float v = (*field)[i];
        std::uint8_t hr, hg, hb;
        HeatRampRedYellowGreen(v, hr, hg, hb);
        // Blend so terrain context still shows through.
        r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
        g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
        b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
      }
    }
  } break;

  case ExportLayer::Noise: {
    // Heuristic noise pollution / soundscape.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.noise && ctx.noise->noise01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const float n01 = ctx.noise->noise01[FlatIdx(x, y, ctx.w)];
      std::uint8_t hr, hg, hb;
      // Invert so *loud* is red, *quiet* is green.
      HeatRampRedYellowGreen(1.0f - n01, hr, hg, hb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }

    // Keep overlay context visible for parks/zones (helps interpret "why" a
    // tile is loud/quiet without switching layers).
    if (t.overlay == Overlay::Park || t.overlay == Overlay::Residential ||
        t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::LandUseMix: {
    // Land-use mix / diversity index.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.landUseMix && ctx.landUseMix->mix01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const float m01 = ctx.landUseMix->mix01[i];

      float dens = 1.0f;
      if (ctx.landUseMix->density01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
        dens = ctx.landUseMix->density01[i];
      }

      std::uint8_t hr, hg, hb;
      // 0 -> red (single-use), 1 -> green (well-mixed).
      HeatRampRedYellowGreen(m01, hr, hg, hb);

      // Fade the heatmap in only where there is meaningful land use in the window.
      const float a = 0.85f * SmoothStep(0.05f, 0.25f, dens);
      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    // Keep overlay context visible for zones/parks (helps interpret the mix).
    if (t.overlay == Overlay::Park || t.overlay == Overlay::Residential ||
        t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::HeatIsland: {
    // Heuristic urban heat island (diffused heat sources/sinks).
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.heatIsland && ctx.heatIsland->heat01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const float h01 = ctx.heatIsland->heat01[FlatIdx(x, y, ctx.w)];

      std::uint8_t hr, hg, hb;
      // Invert so *hot* is red, *cool* is green.
      HeatRampRedYellowGreen(1.0f - h01, hr, hg, hb);

      const float a = 0.85f;
      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    // Keep overlay context visible (roads/parks/zones/civic) so the causes of
    // hot/cool patches are readable in a single layer.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;


  case ExportLayer::AirPollution:
  case ExportLayer::AirPollutionEmission: {
    // Heuristic transported air pollution.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.airPollution && ctx.airPollution->pollution01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        ctx.airPollution->emission01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      float v01 = 0.0f;
      if (layer == ExportLayer::AirPollution) {
        v01 = ctx.airPollution->pollution01[i];
      } else {
        v01 = ctx.airPollution->emission01[i];
      }

      std::uint8_t hr, hg, hb;
      // Invert so polluted is red, clean is green.
      HeatRampRedYellowGreen(1.0f - Clamp01(v01), hr, hg, hb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }

    // Keep overlay context visible for parks/zones (helps interpret sources/sinks).
    if (t.overlay == Overlay::Park) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 30) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 220) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 30) / 2);
    }
    if (t.terrain == Terrain::Water) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 20) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 80) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 200) / 2);
    }
  } break;


  case ExportLayer::RunoffPollution:
  case ExportLayer::RunoffPollutionLoad: {
    // Runoff / stormwater pollution heuristic.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    const std::size_t nTiles = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
    if (ctx.runoff && ctx.runoff->pollution01.size() == nTiles && ctx.runoff->localLoad01.size() == nTiles) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      float v01 = 0.0f;
      if (layer == ExportLayer::RunoffPollution) {
        v01 = ctx.runoff->pollution01[i];

        // Emphasize stream channels slightly so the routed field is readable.
        if (ctx.runoff->flowAccum.size() == nTiles && ctx.runoff->maxFlowAccum > 0) {
          const int a = ctx.runoff->flowAccum[i];
          const int maxA = std::max(1, ctx.runoff->maxFlowAccum);
          const float minA = static_cast<float>(std::max(2, maxA / 128));
          const float maxA2 = static_cast<float>(std::max(std::max(3, maxA / 16), maxA / 32 + 1));
          const float stream01 = SmoothStep(minA, maxA2, static_cast<float>(a));
          v01 = Clamp01(v01 * (0.55f + 0.45f * stream01));
        }
      } else {
        v01 = ctx.runoff->localLoad01[i];
      }

      std::uint8_t hr, hg, hb;
      // Invert so high pollution/load is red, low is green.
      HeatRampRedYellowGreen(1.0f - Clamp01(v01), hr, hg, hb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }

    // Keep overlay context visible for parks/zones (helps interpret sources/sinks).
    if (t.overlay == Overlay::Park) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 30) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 220) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 30) / 2);
    }
    if (t.terrain == Terrain::Water) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 20) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 80) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 200) / 2);
    }
  } break;

  case ExportLayer::RunoffMitigationPriority:
  case ExportLayer::RunoffMitigationPlan: {
    // Stormwater mitigation guidance (park placement):
    // - priority: green=high benefit, red=low
    // - plan: highlights the selected park tiles over the shaded basemap
    MulPixel(r, g, b, shade);

    const std::size_t nTiles = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
    if (ctx.runoffMitigation && ctx.runoffMitigation->priority01.size() == nTiles &&
        ctx.runoffMitigation->planMask.size() == nTiles) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      if (layer == ExportLayer::RunoffMitigationPriority) {
        const float v01 = Clamp01(ctx.runoffMitigation->priority01[i]);
        std::uint8_t hr, hg, hb;
        // High benefit => green.
        HeatRampRedYellowGreen(v01, hr, hg, hb);
        r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
        g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
        b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
      } else {
        // Plan layer: overlay the suggested tiles in a bright cyan.
        if (ctx.runoffMitigation->planMask[i] != 0) {
          r = 40;
          g = 230;
          b = 255;
        }
      }
    }

    // Keep overlay context visible for parks/water/zones.
    if (t.overlay == Overlay::Park) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 30) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 220) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 30) / 2);
    }
    if (t.terrain == Terrain::Water) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 20) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 80) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 200) / 2);
    }
  } break;

  case ExportLayer::SolarExposure:
  case ExportLayer::SolarPotential: {
    // Solar exposure / rooftop PV potential proxy.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.solar && ctx.solar->exposure01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        ctx.solar->roofArea01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        ctx.solar->potential01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const float roof = ctx.solar->roofArea01[i];

      // For rooftop potential, only highlight tiles that plausibly have a roof.
      if (layer == ExportLayer::SolarPotential && roof <= 0.0f) {
        break;
      }

      const float v01 = (layer == ExportLayer::SolarExposure) ? ctx.solar->exposure01[i] : ctx.solar->potential01[i];

      std::uint8_t hr, hg, hb;
      HeatRampRedYellowGreen(Clamp01(v01), hr, hg, hb);

      // Blend so the underlying map context is still visible.
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }
  } break;

  case ExportLayer::SkyView:
  case ExportLayer::CanyonConfinement: {
    // Urban morphology openness / canyon confinement.
    // Background: terrain with height shading for context.
    MulPixel(r, g, b, shade);

    const std::size_t nTiles = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
    if (ctx.skyView && ctx.skyView->skyView01.size() == nTiles && ctx.skyView->canyon01.size() == nTiles) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const float v01 = (layer == ExportLayer::SkyView) ? Clamp01(ctx.skyView->skyView01[i])
                                                       : Clamp01(ctx.skyView->canyon01[i]);

      std::uint8_t hr, hg, hb;
      if (layer == ExportLayer::SkyView) {
        // More open sky is "better" -> green.
        HeatRampRedYellowGreen(v01, hr, hg, hb);
      } else {
        // More confinement is "worse" -> red.
        HeatRampRedYellowGreen(1.0f - v01, hr, hg, hb);
      }

      // Blend strongly on road + built tiles, more lightly elsewhere.
      float a = 0.62f;
      if (t.overlay == Overlay::Road || t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
          t.overlay == Overlay::Industrial || t.overlay == Overlay::School || t.overlay == Overlay::Hospital ||
          t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation) {
        a = 0.84f;
      } else if (t.terrain == Terrain::Water) {
        a = 0.48f;
      }

      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    // Keep overlay context visible for parks/water.
    if (t.overlay == Overlay::Park) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 30) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 220) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 30) / 2);
    }
    if (t.terrain == Terrain::Water) {
      r = static_cast<std::uint8_t>((static_cast<int>(r) + 20) / 2);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + 80) / 2);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + 200) / 2);
    }
  } break;

  case ExportLayer::RoadCentrality: {
    // Road network betweenness centrality.
    // Background: overlay map so it's actionable.
    MulPixel(r, g, b, shade);
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      if (t.overlay != Overlay::None) {
        r = orr; g = org; b = orb;
      }
    }

    if (ctx.roadHealth && ctx.roadHealth->centrality01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Road) {
      const float c01 = ctx.roadHealth->centrality01[FlatIdx(x, y, ctx.w)];
      if (c01 > 0.0f) {
        std::uint8_t hr, hg, hb;
        // Invert so high centrality is red.
        HeatRampRedYellowGreen(1.0f - Clamp01(c01), hr, hg, hb);
        const float a = 0.88f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }
  } break;

  case ExportLayer::RoadVulnerability: {
    // Road network vulnerability: bridge impact + articulation markers.
    // Background: overlay map so it's interpretable.
    MulPixel(r, g, b, shade);
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      if (t.overlay != Overlay::None) {
        r = orr; g = org; b = orb;
      }
    }

    if (ctx.roadHealth && ctx.roadHealth->vulnerability01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Road) {
      const float v01 = ctx.roadHealth->vulnerability01[FlatIdx(x, y, ctx.w)];
      if (v01 > 0.0f) {
        std::uint8_t hr, hg, hb;
        // Invert so high vulnerability is red.
        HeatRampRedYellowGreen(1.0f - Clamp01(v01), hr, hg, hb);
        const float a = 0.90f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }
  } break;

  case ExportLayer::RoadBypass: {
    // Suggested resilience bypass paths (overlay).
    // Background: overlay map, then draw bypass path as a vivid line and
    // keep vulnerability shading on roads.
    MulPixel(r, g, b, shade);
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      if (t.overlay != Overlay::None) {
        r = orr; g = org; b = orb;
      }
    }

    if (ctx.roadHealth) {
      const std::size_t i = FlatIdx(x, y, ctx.w);

      if (ctx.roadHealth->bypassMask.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
          ctx.roadHealth->bypassMask[i]) {
        // Bright cyan for recommended bypass tiles.
        r = 20; g = 245; b = 245;
        break;
      }

      if (t.overlay == Overlay::Road && ctx.roadHealth->vulnerability01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
        const float v01 = ctx.roadHealth->vulnerability01[i];
        if (v01 > 0.0f) {
          std::uint8_t hr, hg, hb;
          HeatRampRedYellowGreen(1.0f - Clamp01(v01), hr, hg, hb);
          const float a = 0.80f;
          r = LerpU8(r, hr, a);
          g = LerpU8(g, hg, a);
          b = LerpU8(b, hb, a);
        }
      }
    }
  } break;


  case ExportLayer::Livability:
  case ExportLayer::InterventionPriority: {
    // Composite livability index + intervention priority.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.livability &&
        ctx.livability->livability01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        ctx.livability->priority01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const std::size_t i = FlatIdx(x, y, ctx.w);

      float v01 = 0.0f;
      if (layer == ExportLayer::Livability) {
        v01 = ctx.livability->livability01[i];
        // 0 -> red (poor), 1 -> green (great).
        std::uint8_t hr, hg, hb;
        HeatRampRedYellowGreen(Clamp01(v01), hr, hg, hb);
        const float a = 0.88f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      } else {
        v01 = ctx.livability->priority01[i];
        // 0 -> green (low urgency), 1 -> red (high urgency).
        std::uint8_t hr, hg, hb;
        HeatRampRedYellowGreen(1.0f - Clamp01(v01), hr, hg, hb);
        const float a = 0.90f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }

    // Keep overlay context visible so interventions are actionable.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;


  case ExportLayer::LivabilityHotspot:
  case ExportLayer::InterventionHotspot: {
    // Getis-Ord Gi* hotspot visualization.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    const HotspotResult* hs = (layer == ExportLayer::LivabilityHotspot) ? ctx.livabilityHotspot : ctx.interventionHotspot;

    if (hs && hs->z01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const float v01 = hs->z01[i];
      std::uint8_t hr, hg, hb;
      DivergingBlueWhiteRed(v01, hr, hg, hb);

      const float a = (t.terrain == Terrain::Water) ? 0.30f : 0.88f;
      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    // Keep overlay context visible so clusters remain actionable.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;


  case ExportLayer::FireRisk: {
    // Heuristic fire risk (dense development + weak fire station coverage).
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    // Always highlight fire stations so the supply is obvious.
    if (t.overlay == Overlay::FireStation) {
      OverlayColor(t, r, g, b);
      break;
    }

    if (ctx.fireRisk && ctx.fireRisk->risk01.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      const float fr01 = ctx.fireRisk->risk01[FlatIdx(x, y, ctx.w)];

      std::uint8_t hr, hg, hb;
      // Invert so *high risk* is red and *low risk* is green.
      HeatRampRedYellowGreen(1.0f - fr01, hr, hg, hb);

      // Make water remain readable.
      const float a = (t.terrain == Terrain::Water) ? 0.35f : 0.85f;
      r = LerpU8(r, hr, a);
      g = LerpU8(g, hg, a);
      b = LerpU8(b, hb, a);
    }

    // Keep overlay context visible (roads/parks/zones/civic) so the causes of
    // risk hot-spots are readable.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::Walkability:
  case ExportLayer::WalkabilityPark:
  case ExportLayer::WalkabilityRetail:
  case ExportLayer::WalkabilityEducation:
  case ExportLayer::WalkabilityHealth:
  case ExportLayer::WalkabilitySafety: {
    // Walkability / 15-minute city amenity accessibility heuristic.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    // Highlight amenity supply tiles so it's obvious *why* a neighborhood scores well/poorly.
    if (t.overlay == Overlay::Park || t.overlay == Overlay::School || t.overlay == Overlay::Hospital ||
        t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation) {
      OverlayColor(t, r, g, b);
      break;
    }

    const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
    if (!isZone) break;

    if (ctx.walkability) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);

      const std::vector<float>* field = nullptr;
      if (layer == ExportLayer::Walkability) field = &ctx.walkability->overall01;
      else if (layer == ExportLayer::WalkabilityPark) field = &ctx.walkability->park01;
      else if (layer == ExportLayer::WalkabilityRetail) field = &ctx.walkability->retail01;
      else if (layer == ExportLayer::WalkabilityEducation) field = &ctx.walkability->education01;
      else if (layer == ExportLayer::WalkabilityHealth) field = &ctx.walkability->health01;
      else if (layer == ExportLayer::WalkabilitySafety) field = &ctx.walkability->safety01;

      if (field && field->size() == n && i < field->size()) {
        const float v01 = std::clamp((*field)[i], 0.0f, 1.0f);

        std::uint8_t hr, hg, hb;
        // 0 -> red (amenity desert), 1 -> green (excellent access).
        HeatRampRedYellowGreen(v01, hr, hg, hb);

        const float a = (t.terrain == Terrain::Water) ? 0.35f : 0.85f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }

    // Keep overlay context visible for zones so the viewer can interpret
    // which land uses drive walkability outcomes.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::JobAccess:
  case ExportLayer::JobOpportunity: {
    // Job accessibility / opportunity analytics.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    // Highlight job source tiles so it's obvious *why* neighborhoods score well/poorly.
    if (t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
      OverlayColor(t, r, g, b);
      break;
    }

    if (ctx.jobs) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);

      const std::vector<float>* field = nullptr;
      if (layer == ExportLayer::JobAccess) field = &ctx.jobs->jobAccess01;
      else if (layer == ExportLayer::JobOpportunity) field = &ctx.jobs->jobOpportunity01;

      if (field && field->size() == n && i < field->size()) {
        const float v01 = std::clamp((*field)[i], 0.0f, 1.0f);

        std::uint8_t hr, hg, hb;
        // 0 -> red (job desert), 1 -> green (excellent access/opportunity).
        HeatRampRedYellowGreen(v01, hr, hg, hb);

        const float a = (t.terrain == Terrain::Water) ? 0.35f : 0.85f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }

    // Keep overlay context visible (roads/parks/zones/civic) so the causes of
    // outcomes are readable.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;


  case ExportLayer::EnergyDemand:
  case ExportLayer::EnergySolar:
  case ExportLayer::EnergyBalance: {
    // Urban energy system analytics.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    const bool isBuilding = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                             t.overlay == Overlay::Industrial || t.overlay == Overlay::School ||
                             t.overlay == Overlay::Hospital || t.overlay == Overlay::PoliceStation ||
                             t.overlay == Overlay::FireStation);
    if (!isBuilding) break;

    if (ctx.energy) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);

      const std::vector<float>* field = nullptr;
      if (layer == ExportLayer::EnergyDemand) field = &ctx.energy->demand01;
      else if (layer == ExportLayer::EnergySolar) field = &ctx.energy->solar01;
      else if (layer == ExportLayer::EnergyBalance) field = &ctx.energy->balance01;

      if (field && field->size() == n && i < field->size()) {
        const float v01 = std::clamp((*field)[i], 0.0f, 1.0f);

        std::uint8_t hr, hg, hb;
        // Demand: high demand is "worse" -> red. Supply/balance: high is "better" -> green.
        if (layer == ExportLayer::EnergyDemand) {
          HeatRampRedYellowGreen(1.0f - v01, hr, hg, hb);
        } else {
          HeatRampRedYellowGreen(v01, hr, hg, hb);
        }

        const float a = (t.terrain == Terrain::Water) ? 0.35f : 0.85f;
        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }

    // Keep overlay context visible so the viewer can interpret causes.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::CarbonEmission:
  case ExportLayer::CarbonSequestration:
  case ExportLayer::CarbonBalance: {
    // Carbon proxy: emissions (bad), sinks (good), and net balance (good).
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.carbon) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);

      const std::vector<float>* field = nullptr;
      if (layer == ExportLayer::CarbonEmission) field = &ctx.carbon->emission01;
      else if (layer == ExportLayer::CarbonSequestration) field = &ctx.carbon->sequestration01;
      else if (layer == ExportLayer::CarbonBalance) field = &ctx.carbon->balance01;

      if (field && field->size() == n && i < field->size()) {
        const float v01 = std::clamp((*field)[i], 0.0f, 1.0f);

        std::uint8_t hr, hg, hb;
        if (layer == ExportLayer::CarbonEmission) {
          // High emission is worse -> red.
          HeatRampRedYellowGreen(1.0f - v01, hr, hg, hb);
        } else {
          // High sinks / positive balance is better -> green.
          HeatRampRedYellowGreen(v01, hr, hg, hb);
        }

        float a = 0.0f;
        if (layer == ExportLayer::CarbonEmission) {
          const bool isBuilding = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                                   t.overlay == Overlay::Industrial || t.overlay == Overlay::School ||
                                   t.overlay == Overlay::Hospital || t.overlay == Overlay::PoliceStation ||
                                   t.overlay == Overlay::FireStation);
          const bool isEmitter = isBuilding || (t.overlay == Overlay::Road);
          a = isEmitter ? 0.85f : 0.25f;
        } else if (layer == ExportLayer::CarbonSequestration) {
          const bool isSink = (t.overlay == Overlay::Park) || (t.overlay == Overlay::None && t.terrain == Terrain::Grass);
          a = isSink ? 0.85f : 0.25f;
        } else {
          // Net balance: show on most tiles, but keep empty land a bit subtler.
          a = (t.terrain == Terrain::Water) ? 0.35f : 0.80f;
          if (t.overlay == Overlay::None) a = (t.terrain == Terrain::Grass) ? 0.65f : 0.55f;
        }

        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }

    // Keep overlay context visible so the viewer can interpret causes.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::CrimeRisk:
  case ExportLayer::PoliceAccess: {
    // Crime risk / policing accessibility.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.crime) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
      if (i < n && ctx.crime->risk01.size() == n && ctx.crime->policeAccess01.size() == n) {
        float v01 = 0.0f;
        if (layer == ExportLayer::CrimeRisk) {
          v01 = std::clamp(ctx.crime->risk01[i], 0.0f, 1.0f);
        } else {
          v01 = std::clamp(ctx.crime->policeAccess01[i], 0.0f, 1.0f);
        }

        std::uint8_t hr, hg, hb;
        if (layer == ExportLayer::CrimeRisk) {
          HeatRampPurple(v01, hr, hg, hb);
        } else {
          HeatRampRedYellowGreen(v01, hr, hg, hb);
        }

        float a = 0.0f;
        if (layer == ExportLayer::PoliceAccess) {
          const bool isDemand = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                                 t.overlay == Overlay::Industrial || t.overlay == Overlay::School ||
                                 t.overlay == Overlay::Hospital || t.overlay == Overlay::PoliceStation ||
                                 t.overlay == Overlay::FireStation);
          a = isDemand ? 0.85f : ((t.overlay == Overlay::Road) ? 0.60f : 0.35f);
        } else {
          const bool meaningful = (t.overlay != Overlay::None && t.terrain != Terrain::Water);
          a = meaningful ? 0.85f : 0.40f;
          if (t.terrain == Terrain::Water) a = 0.25f;
        }

        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);
      }
    }

    // Keep overlay context visible so the viewer can interpret causes.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::TransitAccess:
  case ExportLayer::TransitModeSharePotential: {
    // Transit accessibility + localized mode-share potential.
    // Background: terrain with height shading.
    MulPixel(r, g, b, shade);

    if (ctx.transit) {
      const std::size_t i = FlatIdx(x, y, ctx.w);
      const std::size_t n = static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h);
      if (i < n) {
        float v01 = 0.0f;
        if (layer == ExportLayer::TransitAccess && ctx.transit->access01.size() == n) {
          v01 = std::clamp(ctx.transit->access01[i], 0.0f, 1.0f);
        } else if (layer == ExportLayer::TransitModeSharePotential && ctx.transit->modeSharePotential01.size() == n) {
          v01 = std::clamp(ctx.transit->modeSharePotential01[i], 0.0f, 1.0f);
        }

        std::uint8_t hr, hg, hb;
        // 0 -> red (poor access / low potential), 1 -> green (excellent).
        HeatRampRedYellowGreen(v01, hr, hg, hb);

        float a = 0.0f;
        if (layer == ExportLayer::TransitAccess) {
          // Show on most non-water tiles, but keep roads/empty land a bit subtler.
          const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                               t.overlay == Overlay::Industrial || t.overlay == Overlay::School ||
                               t.overlay == Overlay::Hospital || t.overlay == Overlay::PoliceStation ||
                               t.overlay == Overlay::FireStation);
          a = isZone ? 0.85f : ((t.overlay == Overlay::Road) ? 0.55f : 0.65f);
        } else {
          // Mode-share potential is only meaningful on residential/job tiles.
          const bool isR = (t.overlay == Overlay::Residential);
          const bool isJ = (t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
          a = (isR || isJ) ? 0.85f : 0.25f;
        }

        r = LerpU8(r, hr, a);
        g = LerpU8(g, hg, a);
        b = LerpU8(b, hb, a);

        // Highlight planned stops and served corridors for readability.
        if (ctx.transit->stopMask.size() == n && ctx.transit->stopMask[i]) {
          r = 235;
          g = 250;
          b = 255;
        } else if (ctx.transit->corridorMask.size() == n && ctx.transit->corridorMask[i]) {
          r = LerpU8(r, 64, 0.15f);
          g = LerpU8(g, 96, 0.15f);
          b = LerpU8(b, 200, 0.15f);
        }
      }
    }

    // Keep overlay context visible so viewers can interpret why access differs.
    if (t.overlay != Overlay::None) {
      std::uint8_t orr = r, org = g, orb = b;
      OverlayColor(t, orr, org, orb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) * 2 + static_cast<int>(orr)) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) * 2 + static_cast<int>(org)) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) * 2 + static_cast<int>(orb)) / 3);
    }
  } break;

  case ExportLayer::ZonePressureResidential:
  case ExportLayer::ZonePressureCommercial:
  case ExportLayer::ZonePressureIndustrial: {
    MulPixel(r, g, b, shade);

    if (t.terrain == Terrain::Water) {
      // Leave water as shaded terrain.
      break;
    }

    Overlay target = Overlay::Residential;
    float demand = world.stats().demandResidential;
    float lvWeight = 0.70f;

    if (layer == ExportLayer::ZonePressureCommercial) {
      target = Overlay::Commercial;
      demand = world.stats().demandCommercial;
      lvWeight = 0.80f;
    } else if (layer == ExportLayer::ZonePressureIndustrial) {
      target = Overlay::Industrial;
      demand = world.stats().demandIndustrial;
      lvWeight = 0.80f;
    }

    // Only show pressure on empty land or on matching zone tiles.
    if (!(t.overlay == Overlay::None || t.overlay == target)) {
      MulPixel(r, g, b, 0.55f);
      break;
    }

    bool access = false;
    if (t.overlay == target) {
      if (ctx.zoneAccess) {
        access = HasZoneAccess(*ctx.zoneAccess, x, y);
      } else {
        access = world.hasAdjacentRoad(x, y);
      }
    } else {
      // For zoning guidance, show pressure only where a new zone could connect to a road.
      access = world.hasAdjacentRoad(x, y);
    }

    const float accessFactor = access ? 1.0f : 0.0f;

    float lvVal = 0.5f;
    if (ctx.landValue && ctx.landValue->w == ctx.w && ctx.landValue->h == ctx.h &&
        ctx.landValue->value.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h)) {
      lvVal = ctx.landValue->value[FlatIdx(x, y, ctx.w)];
    }

    float desir = 1.0f;
    if (target == Overlay::Industrial) {
      // Industry prefers cheaper land.
      desir = 1.0f + lvWeight * (0.50f - lvVal);
    } else {
      desir = 1.0f + lvWeight * (lvVal - 0.50f);
    }
    desir = std::clamp(desir, 0.40f, 1.60f);

    const float pressure = Clamp01(demand * desir) * accessFactor;

    std::uint8_t hr, hg, hb;
    HeatRampRedYellowGreen(pressure, hr, hg, hb);
    r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
    g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
    b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
  } break;


  default:
    break;
  }
}

// Convert normalized tile height -> pixel elevation (clamped).
inline int HeightToPx(float h01, int heightScalePx)
{
  if (heightScalePx <= 0) return 0;
  return static_cast<int>(std::lround(Clamp01(h01) * static_cast<float>(heightScalePx)));
}

struct Ipt {
  int x = 0;
  int y = 0;
};

inline Ipt IsoCenter(int tx, int ty, int halfW, int halfH, int heightPx)
{
  return Ipt{(tx - ty) * halfW, (tx + ty) * halfH - heightPx};
}

inline int EdgeFn(const Ipt& a, const Ipt& b, int px, int py)
{
  // 2D cross product (b-a) x (p-a)
  return (px - a.x) * (b.y - a.y) - (py - a.y) * (b.x - a.x);
}

inline void FillTriangle(PpmImage& img, const Ipt& a, const Ipt& b, const Ipt& c, std::uint8_t r, std::uint8_t g, std::uint8_t bl)
{
  if (img.width <= 0 || img.height <= 0) return;

  const int minX = std::max(0, std::min({a.x, b.x, c.x}));
  const int maxX = std::min(img.width - 1, std::max({a.x, b.x, c.x}));
  const int minY = std::max(0, std::min({a.y, b.y, c.y}));
  const int maxY = std::min(img.height - 1, std::max({a.y, b.y, c.y}));

  // Degenerate triangle.
  if (minX > maxX || minY > maxY) return;

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const int w0 = EdgeFn(b, c, x, y);
      const int w1 = EdgeFn(c, a, x, y);
      const int w2 = EdgeFn(a, b, x, y);

      const bool hasNeg = (w0 < 0) || (w1 < 0) || (w2 < 0);
      const bool hasPos = (w0 > 0) || (w1 > 0) || (w2 > 0);
      if (hasNeg && hasPos) continue;

      SetPixel(img.rgb, img.width, x, y, r, g, bl);
    }
  }
}

inline void FillQuad(PpmImage& img, const Ipt& a, const Ipt& b, const Ipt& c, const Ipt& d, std::uint8_t r, std::uint8_t g,
                     std::uint8_t bl)
{
  // Split into two triangles (a,b,c) and (a,c,d)
  FillTriangle(img, a, b, c, r, g, bl);
  FillTriangle(img, a, c, d, r, g, bl);
}

inline void DrawLine(PpmImage& img, int x0, int y0, int x1, int y1, std::uint8_t r, std::uint8_t g, std::uint8_t bl)
{
  if (img.width <= 0 || img.height <= 0) return;

  // Bresenham
  int dx = std::abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    if (x0 >= 0 && x0 < img.width && y0 >= 0 && y0 < img.height) {
      SetPixel(img.rgb, img.width, x0, y0, r, g, bl);
    }
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

} // namespace

bool ParseExportLayer(const std::string& s, ExportLayer& outLayer)
{
  const std::string k = ToLower(s);
  if (k == "terrain") { outLayer = ExportLayer::Terrain; return true; }
  if (k == "overlay" || k == "overlays") { outLayer = ExportLayer::Overlay; return true; }
  if (k == "height" || k == "elevation" || k == "heightmap") { outLayer = ExportLayer::Height; return true; }
  if (k == "landvalue" || k == "land_value" || k == "lv" || k == "land" || k == "value") { outLayer = ExportLayer::LandValue; return true; }
  if (k == "traffic" || k == "commute") { outLayer = ExportLayer::Traffic; return true; }
  if (k == "goods" || k == "goods_traffic" || k == "goodstraffic") { outLayer = ExportLayer::GoodsTraffic; return true; }
  if (k == "goods_fill" || k == "goodsfill" || k == "fill") { outLayer = ExportLayer::GoodsFill; return true; }
  if (k == "district" || k == "districts") { outLayer = ExportLayer::District; return true; }
  if (k == "flooddepth" || k == "flood_depth" || k == "flood") { outLayer = ExportLayer::FloodDepth; return true; }
  if (k == "pondingdepth" || k == "ponding_depth" || k == "pond" || k == "ponding" || k == "depression") {
    outLayer = ExportLayer::PondingDepth;
    return true;
  }
  if (k == "services" || k == "service" || k == "services_overall" || k == "servicesoverall" || k == "svc") {
    outLayer = ExportLayer::ServicesOverall;
    return true;
  }
  if (k == "services_education" || k == "service_education" || k == "servicesedu" || k == "services_edu" || k == "svc_education" || k == "svc_edu") {
    outLayer = ExportLayer::ServicesEducation;
    return true;
  }
  if (k == "services_health" || k == "service_health" || k == "serviceshealth" || k == "services_h" || k == "svc_health" || k == "svc_h") {
    outLayer = ExportLayer::ServicesHealth;
    return true;
  }
  if (k == "services_safety" || k == "service_safety" || k == "servicessafety" || k == "services_s" || k == "svc_safety" || k == "svc_s") {
    outLayer = ExportLayer::ServicesSafety;
    return true;
  }
  if (k == "noise" || k == "sound" || k == "noise_pollution" || k == "noisepollution") {
    outLayer = ExportLayer::Noise;
    return true;
  }
  if (k == "landuse_mix" || k == "land_use_mix" || k == "landusemix" || k == "mix" || k == "diversity") {
    outLayer = ExportLayer::LandUseMix;
    return true;
  }
  if (k == "heat_island" || k == "heat" || k == "uhi" || k == "heatisland" || k == "urban_heat" || k == "temperature") {
    outLayer = ExportLayer::HeatIsland;
    return true;
  }
  
  if (k == "air_pollution" || k == "air" || k == "airquality" || k == "air_quality" || k == "smog" || k == "aq") {
    outLayer = ExportLayer::AirPollution;
    return true;
  }
  if (k == "air_emission" || k == "air_source" || k == "air_sources" || k == "smog_sources" || k == "pollution_source") {
    outLayer = ExportLayer::AirPollutionEmission;
    return true;
  }

  if (k == "runoff_pollution" || k == "runoff" || k == "stormwater" || k == "storm_water" || k == "water_quality") {
    outLayer = ExportLayer::RunoffPollution;
    return true;
  }
  if (k == "runoff_load" || k == "runoff_source" || k == "stormwater_load" || k == "stormwater_source" || k == "runoff_emission") {
    outLayer = ExportLayer::RunoffPollutionLoad;
    return true;
  }

  if (k == "runoff_mitigation_priority" || k == "stormwater_mitigation_priority" || k == "gi_priority" ||
      k == "runoff_priority" || k == "stormwater_priority") {
    outLayer = ExportLayer::RunoffMitigationPriority;
    return true;
  }
  if (k == "runoff_mitigation_plan" || k == "stormwater_mitigation_plan" || k == "gi_plan" ||
      k == "runoff_plan" || k == "stormwater_plan" || k == "runoff_mitigation" || k == "stormwater_mitigation") {
    outLayer = ExportLayer::RunoffMitigationPlan;
    return true;
  }

  if (k == "solar_exposure" || k == "solar" || k == "insolation" || k == "sun" || k == "sunlight") {
    outLayer = ExportLayer::SolarExposure;
    return true;
  }
  if (k == "solar_potential" || k == "solar_rooftop" || k == "pv" || k == "pv_potential" || k == "rooftop_pv") {
    outLayer = ExportLayer::SolarPotential;
    return true;
  }

  if (k == "sky_view" || k == "skyview" || k == "svf" || k == "sky_view_factor") {
    outLayer = ExportLayer::SkyView;
    return true;
  }
  if (k == "canyon" || k == "canyon_confinement" || k == "urban_canyon" || k == "confinement" || k == "canyon01") {
    outLayer = ExportLayer::CanyonConfinement;
    return true;
  }

  if (k == "road_centrality" || k == "roadcentrality" || k == "centrality_road" || k == "road_betweenness" || k == "road_betweenness_centrality") {
    outLayer = ExportLayer::RoadCentrality;
    return true;
  }
  if (k == "road_vulnerability" || k == "roadvulnerability" || k == "road_fragility" || k == "road_fragile" || k == "road_resilience_risk") {
    outLayer = ExportLayer::RoadVulnerability;
    return true;
  }
  if (k == "road_bypass" || k == "roadbypass" || k == "resilience_bypass" || k == "bypass_road" || k == "bypass") {
    outLayer = ExportLayer::RoadBypass;
    return true;
  }

  if (k == "livability" || k == "liveability" || k == "quality_of_life" || k == "qol") {
    outLayer = ExportLayer::Livability;
    return true;
  }
  if (k == "intervention_priority" || k == "priority" || k == "intervention" || k == "action_priority") {
    outLayer = ExportLayer::InterventionPriority;
    return true;
  }

  if (k == "livability_hotspot" || k == "livability_cluster" || k == "hotspot_livability" || k == "livability_gistar") {
    outLayer = ExportLayer::LivabilityHotspot;
    return true;
  }
  if (k == "intervention_hotspot" || k == "priority_hotspot" || k == "need_hotspot" || k == "hotspot_priority" || k == "priority_gistar") {
    outLayer = ExportLayer::InterventionHotspot;
    return true;
  }

  if (k == "fire_risk" || k == "firerisk" || k == "fire" || k == "firehazard" || k == "hazard_fire") {
    outLayer = ExportLayer::FireRisk;
    return true;
  }

  if (k == "walkability" || k == "walk" || k == "walk_score" || k == "walkscore" || k == "15min") {
    outLayer = ExportLayer::Walkability;
    return true;
  }
  if (k == "walkability_park" || k == "walk_park" || k == "walkability_parks" || k == "walk_parks") {
    outLayer = ExportLayer::WalkabilityPark;
    return true;
  }
  if (k == "walkability_retail" || k == "walk_retail" || k == "walkability_shops" || k == "walk_shops") {
    outLayer = ExportLayer::WalkabilityRetail;
    return true;
  }
  if (k == "walkability_education" || k == "walk_education" || k == "walkability_schools" || k == "walk_schools") {
    outLayer = ExportLayer::WalkabilityEducation;
    return true;
  }
  if (k == "walkability_health" || k == "walk_health" || k == "walkability_hospitals" || k == "walk_hospitals") {
    outLayer = ExportLayer::WalkabilityHealth;
    return true;
  }
  if (k == "walkability_safety" || k == "walk_safety" || k == "walkability_emergency" || k == "walk_emergency") {
    outLayer = ExportLayer::WalkabilitySafety;
    return true;
  }
  if (k == "job_access" || k == "jobaccess" || k == "jobs_access" || k == "job_accessibility" || k == "access_jobs" ||
      k == "employment_access" || k == "employment_accessibility") {
    outLayer = ExportLayer::JobAccess;
    return true;
  }
  if (k == "job_opportunity" || k == "jobopportunity" || k == "jobs_opportunity" || k == "employment" || k == "jobs" ||
      k == "job_market" || k == "employment_opportunity") {
    outLayer = ExportLayer::JobOpportunity;
    return true;
  }


  if (k == "energy_demand" || k == "power_demand" || k == "electricity_demand" || k == "demand_energy") {
    outLayer = ExportLayer::EnergyDemand;
    return true;
  }
  if (k == "energy_solar" || k == "solar_supply" || k == "pv_supply" || k == "rooftop_solar_supply") {
    outLayer = ExportLayer::EnergySolar;
    return true;
  }
  if (k == "energy_balance" || k == "energy_net" || k == "net_energy" || k == "renewable_balance") {
    outLayer = ExportLayer::EnergyBalance;
    return true;
  }

  if (k == "carbon_emission" || k == "carbon_emissions" || k == "co2_emission") {
    outLayer = ExportLayer::CarbonEmission;
    return true;
  }
  if (k == "carbon_sequestration" || k == "carbon_sink" || k == "co2_sink") {
    outLayer = ExportLayer::CarbonSequestration;
    return true;
  }
  if (k == "carbon_balance" || k == "carbon_net" || k == "co2_balance") {
    outLayer = ExportLayer::CarbonBalance;
    return true;
  }

  if (k == "crime_risk" || k == "crime" || k == "crime_index" || k == "crime_rate" || k == "crime_heat") {
    outLayer = ExportLayer::CrimeRisk;
    return true;
  }
  if (k == "police_access" || k == "police_response" || k == "police" || k == "police_coverage" ||
      k == "police_accessibility") {
    outLayer = ExportLayer::PoliceAccess;
    return true;
  }

  if (k == "traffic_crash_risk" || k == "traffic_crash" || k == "traffic_safety_risk" || k == "traffic_collision_risk") {
    outLayer = ExportLayer::TrafficCrashRisk;
    return true;
  }
  if (k == "traffic_crash_exposure" || k == "traffic_exposure" || k == "traffic_safety_exposure" || k == "collision_exposure") {
    outLayer = ExportLayer::TrafficCrashExposure;
    return true;
  }
  if (k == "traffic_crash_priority" || k == "traffic_safety_priority" || k == "crash_priority" || k == "collision_priority") {
    outLayer = ExportLayer::TrafficCrashPriority;
    return true;
  }

  if (k == "transit_access" || k == "bus_access" || k == "stop_access" || k == "transit_stop_access" ||
      k == "public_transit_access" || k == "pt_access") {
    outLayer = ExportLayer::TransitAccess;
    return true;
  }
  if (k == "transit_mode_share_potential" || k == "transit_mode_share" || k == "mode_share_transit" ||
      k == "bus_mode_share" || k == "pt_mode_share" || k == "transit_potential") {
    outLayer = ExportLayer::TransitModeSharePotential;
    return true;
  }

  if (k == "zone_pressure_residential" || k == "zonepressure_residential" || k == "res_pressure" || k == "pressure_res" ||
      k == "rci_res" || k == "rci_r" || k == "demand_res" || k == "zoning_res") {
    outLayer = ExportLayer::ZonePressureResidential;
    return true;
  }
  if (k == "zone_pressure_commercial" || k == "zonepressure_commercial" || k == "com_pressure" || k == "pressure_com" ||
      k == "rci_com" || k == "rci_c" || k == "demand_com" || k == "zoning_com") {
    outLayer = ExportLayer::ZonePressureCommercial;
    return true;
  }
  if (k == "zone_pressure_industrial" || k == "zonepressure_industrial" || k == "ind_pressure" || k == "pressure_ind" ||
      k == "rci_ind" || k == "rci_i" || k == "demand_ind" || k == "zoning_ind") {
    outLayer = ExportLayer::ZonePressureIndustrial;
    return true;
  }
  return false;
}

const char* ExportLayerName(ExportLayer layer)
{
  switch (layer) {
  case ExportLayer::Terrain: return "terrain";
  case ExportLayer::Overlay: return "overlay";
  case ExportLayer::Height: return "height";
  case ExportLayer::LandValue: return "landvalue";
  case ExportLayer::Traffic: return "traffic";
  case ExportLayer::GoodsTraffic: return "goods_traffic";
  case ExportLayer::GoodsFill: return "goods_fill";
  case ExportLayer::District: return "district";
  case ExportLayer::FloodDepth: return "flood_depth";
  case ExportLayer::PondingDepth: return "ponding_depth";
  case ExportLayer::ServicesOverall: return "services";
  case ExportLayer::ServicesEducation: return "services_education";
  case ExportLayer::ServicesHealth: return "services_health";
  case ExportLayer::ServicesSafety: return "services_safety";
  case ExportLayer::Noise: return "noise";
  case ExportLayer::LandUseMix: return "landuse_mix";
  case ExportLayer::HeatIsland: return "heat_island";
  case ExportLayer::FireRisk: return "fire_risk";
  case ExportLayer::Walkability: return "walkability";
  case ExportLayer::WalkabilityPark: return "walkability_park";
  case ExportLayer::WalkabilityRetail: return "walkability_retail";
  case ExportLayer::WalkabilityEducation: return "walkability_education";
  case ExportLayer::WalkabilityHealth: return "walkability_health";
  case ExportLayer::WalkabilitySafety: return "walkability_safety";
  case ExportLayer::AirPollution:
    return "air_pollution";
  case ExportLayer::AirPollutionEmission:
    return "air_emission";

  case ExportLayer::RunoffPollution:
    return "runoff_pollution";
  case ExportLayer::RunoffPollutionLoad:
    return "runoff_load";
  case ExportLayer::RunoffMitigationPriority:
    return "runoff_mitigation_priority";
  case ExportLayer::RunoffMitigationPlan:
    return "runoff_mitigation_plan";

  case ExportLayer::SolarExposure:
    return "solar_exposure";
  case ExportLayer::SolarPotential:
    return "solar_potential";

  case ExportLayer::RoadCentrality:
    return "road_centrality";
  case ExportLayer::RoadVulnerability:
    return "road_vulnerability";
  case ExportLayer::RoadBypass:
    return "road_bypass";

  case ExportLayer::Livability:
    return "livability";
  case ExportLayer::InterventionPriority:
    return "intervention_priority";

  case ExportLayer::LivabilityHotspot:
    return "livability_hotspot";
  case ExportLayer::InterventionHotspot:
    return "intervention_hotspot";

  case ExportLayer::JobAccess: return "job_access";
  case ExportLayer::JobOpportunity: return "job_opportunity";
  case ExportLayer::EnergyDemand: return "energy_demand";
  case ExportLayer::EnergySolar: return "energy_solar";
  case ExportLayer::EnergyBalance: return "energy_balance";

  case ExportLayer::CarbonEmission: return "carbon_emission";
  case ExportLayer::CarbonSequestration: return "carbon_sequestration";
  case ExportLayer::CarbonBalance: return "carbon_balance";

  case ExportLayer::CrimeRisk: return "crime_risk";
  case ExportLayer::PoliceAccess: return "police_access";

  case ExportLayer::SkyView: return "sky_view";
  case ExportLayer::CanyonConfinement: return "canyon_confinement";

  case ExportLayer::TrafficCrashRisk: return "traffic_crash_risk";
  case ExportLayer::TrafficCrashExposure: return "traffic_crash_exposure";
  case ExportLayer::TrafficCrashPriority: return "traffic_crash_priority";

  case ExportLayer::TransitAccess: return "transit_access";
  case ExportLayer::TransitModeSharePotential: return "transit_mode_share_potential";
  case ExportLayer::ZonePressureResidential: return "zone_pressure_residential";
  case ExportLayer::ZonePressureCommercial: return "zone_pressure_commercial";
  case ExportLayer::ZonePressureIndustrial: return "zone_pressure_industrial";
  default: return "unknown";
  }
}

PpmImage RenderPpmLayer(const World& world, ExportLayer layer, const LandValueResult* landValue, const TrafficResult* traffic,
                        const GoodsResult* goods)
{
  PpmImage img{};
  img.width = world.width();
  img.height = world.height();
  if (img.width <= 0 || img.height <= 0) return img;

  img.rgb.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u, 0);

  // Derived, heightfield-driven layers may require some precomputation.
  std::vector<float> heights;
  std::vector<std::uint8_t> drainMask;

  SeaFloodResult seaFlood{};
  DepressionFillResult ponding{};

  bool haveSeaFlood = false;
  bool havePonding = false;
  float seaLevel = 0.0f;

  if (layer == ExportLayer::FloodDepth || layer == ExportLayer::PondingDepth) {
    BuildHeightFieldAndDrainMask(world, heights, drainMask);
  }

  if (layer == ExportLayer::FloodDepth) {
    seaLevel = InferCoastalSeaLevel(world);
    SeaFloodConfig cfg{};
    cfg.requireEdgeConnection = true;
    cfg.eightConnected = false;
    seaFlood = ComputeSeaLevelFlood(heights, img.width, img.height, seaLevel, cfg);
    haveSeaFlood = true;
  }

  if (layer == ExportLayer::PondingDepth) {
    DepressionFillConfig cfg{};
    cfg.includeEdges = true;
    cfg.epsilon = 0.0f;
    ponding = FillDepressionsPriorityFlood(heights, img.width, img.height, &drainMask, cfg);
    havePonding = true;
  }


  ServicesResult services{};
  bool haveServices = false;
  if (layer == ExportLayer::ServicesOverall || layer == ExportLayer::ServicesEducation ||
      layer == ExportLayer::ServicesHealth || layer == ExportLayer::ServicesSafety) {
    ServicesModelSettings svcCfg{};
    svcCfg.enabled = true;
    // Keep exports deterministic and reasonably aligned with the in-game defaults.
    svcCfg.requireOutsideConnection = true;
    svcCfg.weightMode = IsochroneWeightMode::TravelTime;
    svcCfg.catchmentRadiusSteps = 18;

    const std::vector<ServiceFacility> facilities = ExtractServiceFacilitiesFromWorld(world);
    services = ComputeServices(world, svcCfg, facilities);
    haveServices = true;
  }

  NoiseResult noise{};
  bool haveNoise = false;
  if (layer == ExportLayer::Noise) {
    NoiseConfig nc{};
    noise = ComputeNoisePollution(world, nc, traffic, goods);
    haveNoise = true;
  }

  LandUseMixResult landUseMix{};
  bool haveLandUseMix = false;
  if (layer == ExportLayer::LandUseMix) {
    LandUseMixConfig mc{};
    landUseMix = ComputeLandUseMix(world, mc);
    haveLandUseMix = true;
  }

  HeatIslandResult heatIsland{};
  bool haveHeatIsland = false;
  if (layer == ExportLayer::HeatIsland ||
      layer == ExportLayer::EnergyDemand || layer == ExportLayer::EnergySolar || layer == ExportLayer::EnergyBalance ||
      layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    HeatIslandConfig hc{};
    heatIsland = ComputeHeatIsland(world, hc, traffic, goods);
    haveHeatIsland = true;
  }


  AirPollutionResult airPollution{};
  bool haveAirPollution = false;
  if (layer == ExportLayer::AirPollution || layer == ExportLayer::AirPollutionEmission) {
    AirPollutionConfig ac{};
    // Keep exports deterministic, and pick a stable "prevailing wind" per seed.
    ac.windFromSeed = true;
    airPollution = ComputeAirPollution(world, ac, traffic, goods);
    haveAirPollution = true;
  }


  RunoffPollutionResult runoff{};
  bool haveRunoff = false;
  if (layer == ExportLayer::RunoffPollution || layer == ExportLayer::RunoffPollutionLoad) {
    RunoffPollutionConfig rc{};
    runoff = ComputeRunoffPollution(world, rc, traffic);
    haveRunoff = true;
  }

  RunoffMitigationResult runoffMitigation{};
  bool haveRunoffMitigation = false;
  if (layer == ExportLayer::RunoffMitigationPriority || layer == ExportLayer::RunoffMitigationPlan) {
    RunoffMitigationConfig mc{};
    // Reasonable defaults for a visual planning layer.
    mc.demandMode = RunoffMitigationDemandMode::ResidentialOccupants;
    mc.parksToAdd = 12;
    mc.minSeparation = 3;
    mc.excludeWater = true;
    mc.allowReplaceRoad = false;
    mc.allowReplaceZones = false;

    runoffMitigation = SuggestRunoffMitigationParks(world, mc, traffic);
    haveRunoffMitigation = true;
  }


  SolarPotentialResult solar{};
  bool haveSolar = false;
  if (layer == ExportLayer::SolarExposure || layer == ExportLayer::SolarPotential ||
      layer == ExportLayer::EnergyDemand || layer == ExportLayer::EnergySolar || layer == ExportLayer::EnergyBalance ||
      layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    SolarPotentialConfig sc{};
    // Keep exports deterministic: no random elements and stable shading sampling.
    sc.azimuthSamples = 16;
    solar = ComputeSolarPotential(world, sc);
    haveSolar = true;
  }


  SkyViewResult skyView{};
  bool haveSkyView = false;
  if (layer == ExportLayer::SkyView || layer == ExportLayer::CanyonConfinement ||
      layer == ExportLayer::TrafficCrashRisk || layer == ExportLayer::TrafficCrashExposure ||
      layer == ExportLayer::TrafficCrashPriority) {
    SkyViewConfig vc{};
    vc.azimuthSamples = 16;
    vc.maxHorizonRadius = 64;
    vc.includeBuildings = true;
    skyView = ComputeSkyViewFactor(world, vc);
    haveSkyView = true;
  }

  TrafficSafetyResult trafficSafety{};
  bool haveTrafficSafety = false;
  if (layer == ExportLayer::TrafficCrashRisk || layer == ExportLayer::TrafficCrashExposure ||
      layer == ExportLayer::TrafficCrashPriority) {
    TrafficSafetyConfig tc{};
    tc.requireOutsideConnection = true;
    tc.exposureRadius = 6;
    trafficSafety = ComputeTrafficSafety(world, tc, traffic, haveSkyView ? &skyView : nullptr);
    haveTrafficSafety = true;
  }



  EnergyModelResult energy{};
  bool haveEnergy = false;
  if (layer == ExportLayer::EnergyDemand || layer == ExportLayer::EnergySolar || layer == ExportLayer::EnergyBalance ||
      layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    EnergyModelConfig ec{};
    energy = ComputeEnergyModel(world, ec, haveSolar ? &solar : nullptr, haveHeatIsland ? &heatIsland : nullptr);
    haveEnergy = true;
  }



  CarbonModelResult carbon{};
  bool haveCarbon = false;
  if (layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    CarbonModelConfig cc{};
    carbon = ComputeCarbonModel(world, cc, haveEnergy ? &energy : nullptr, traffic, goods);
    haveCarbon = true;
  }

  CrimeModelResult crime{};
  bool haveCrime = false;
  NoiseResult crimeNoise{};
  JobOpportunityResult crimeJobs{};
  bool haveCrimeJobs = false;
  if (layer == ExportLayer::CrimeRisk || layer == ExportLayer::PoliceAccess) {
    // Crime uses optional noise + job stress proxies.
    NoiseConfig nc{};
    crimeNoise = ComputeNoisePollution(world, nc, traffic, goods);

    JobOpportunityConfig jc{};
    jc.requireOutsideConnection = true;
    jc.useTravelTime = true;
    jc.congestionCosts = true;
    crimeJobs = ComputeJobOpportunity(world, jc, traffic, /*precomputedRoadToEdge=*/nullptr, /*precomputedZoneAccess=*/nullptr);
    haveCrimeJobs = true;

    CrimeModelConfig crc{};
    crc.requireOutsideConnection = true;
    crc.weightMode = IsochroneWeightMode::TravelTime;
    crime = ComputeCrimeModel(world, crc, traffic, goods, (haveCrimeJobs ? &crimeJobs : nullptr), &crimeNoise,
                              /*precomputedRoadToEdge=*/nullptr, /*precomputedZoneAccess=*/nullptr);
    haveCrime = true;
  }

  TransitAccessibilityResult transit{};
  bool haveTransit = false;
  if (layer == ExportLayer::TransitAccess || layer == ExportLayer::TransitModeSharePotential) {
    TransitAccessibilityConfig tc{};
    // Align with the game: enforce outside connectivity and plan lines from demand.
    tc.requireOutsideConnection = true;
    TransitAccessibilityInputs ti{};
    ti.traffic = traffic;
    ti.goods = goods;
    transit = ComputeTransitAccessibility(world, tc, ti);
    haveTransit = true;
  }

  FireRiskResult fireRisk{};
  bool haveFireRisk = false;
  if (layer == ExportLayer::FireRisk) {
    FireRiskConfig fc{};
    fc.requireOutsideConnection = true;
    fc.weightMode = IsochroneWeightMode::TravelTime;
    fc.responseRadiusSteps = 18;
    fireRisk = ComputeFireRisk(world, fc);
    haveFireRisk = true;
  }

  WalkabilityResult walkability{};
  bool haveWalkability = false;
  if (layer == ExportLayer::Walkability || layer == ExportLayer::WalkabilityPark || layer == ExportLayer::WalkabilityRetail ||
      layer == ExportLayer::WalkabilityEducation || layer == ExportLayer::WalkabilityHealth || layer == ExportLayer::WalkabilitySafety) {
    WalkabilityConfig wc{};
    wc.enabled = true;
    // Keep exports deterministic and aligned with in-game defaults (outside connectivity and travel-time weighting).
    wc.requireOutsideConnection = true;
    wc.weightMode = IsochroneWeightMode::TravelTime;
    wc.coverageThresholdSteps = 15;
    walkability = ComputeWalkability(world, wc);
    haveWalkability = true;
  }

  JobOpportunityResult jobs{};
  bool haveJobs = false;
  if (layer == ExportLayer::JobAccess || layer == ExportLayer::JobOpportunity) {
    JobOpportunityConfig jc{};
    jc.requireOutsideConnection = true;
    jc.useTravelTime = true;
    jc.congestionCosts = true;
    // Exports default to travel-time accessibility; when traffic is provided, congestion is incorporated.
    jobs = ComputeJobOpportunity(world, jc, traffic, /*precomputedRoadToEdge=*/nullptr, /*precomputedZoneAccess=*/nullptr);
    haveJobs = true;
  }


  RoadHealthResult roadHealth{};
  bool haveRoadHealth = false;
  if (layer == ExportLayer::RoadCentrality || layer == ExportLayer::RoadVulnerability || layer == ExportLayer::RoadBypass) {
    RoadHealthConfig rc{};
    rc.weightMode = RoadGraphEdgeWeightMode::TravelTimeMilli;
    rc.maxSources = 0; // auto
    rc.autoExactMaxNodes = 650;
    rc.autoSampleSources = 256;
    rc.includeNodeCentrality = true;
    rc.articulationVulnerabilityBase = 0.70f;

    // Bypass planning is fairly expensive; only do it for the bypass layer.
    rc.includeBypass = (layer == ExportLayer::RoadBypass);
    if (rc.includeBypass) {
      rc.bypassCfg.top = 3;
      rc.bypassCfg.moneyObjective = true;
      rc.bypassCfg.targetLevel = 1;
      rc.bypassCfg.allowBridges = false;
      rc.bypassCfg.rankByTraffic = true;
    }

    roadHealth = ComputeRoadHealth(world, rc, traffic);
    haveRoadHealth = true;
  }


  LivabilityResult livability{};
  bool haveLivability = false;
  if (layer == ExportLayer::Livability || layer == ExportLayer::InterventionPriority ||
      layer == ExportLayer::LivabilityHotspot || layer == ExportLayer::InterventionHotspot) {
    LivabilityConfig lc{};
    lc.requireOutsideConnection = true;
    lc.weightMode = IsochroneWeightMode::TravelTime;
    lc.servicesCatchmentRadiusSteps = 18;
    lc.walkCoverageThresholdSteps = 15;
    livability = ComputeLivability(world, lc, traffic, goods);
    haveLivability = true;
  }


  HotspotResult livHot{};
  HotspotResult priHot{};
  bool haveLivHot = false;
  bool havePriHot = false;
  if (layer == ExportLayer::LivabilityHotspot && haveLivability) {
    HotspotConfig hc{};
    hc.radius = 8;
    hc.excludeWater = true;
    hc.zThreshold = 1.96f;
    hc.zScale = 3.0f;
    livHot = ComputeHotspotsGiStar(world, livability.livability01, hc);
    haveLivHot = true;
  }
  if (layer == ExportLayer::InterventionHotspot && haveLivability) {
    HotspotConfig hc{};
    hc.radius = 8;
    hc.excludeWater = true;
    hc.zThreshold = 1.96f;
    hc.zScale = 3.0f;
    priHot = ComputeHotspotsGiStar(world, livability.priority01, hc);
    havePriHot = true;
  }


  ZoneAccessMap zoneAccess{};
  bool haveZoneAccess = false;
  if (layer == ExportLayer::ZonePressureResidential || layer == ExportLayer::ZonePressureCommercial ||
      layer == ExportLayer::ZonePressureIndustrial) {
    // No outside connection mask here; exports should remain usable for worlds
    // that haven't run the sim or don't enforce border connectivity.
    zoneAccess = BuildZoneAccessMap(world, nullptr);
    haveZoneAccess = true;
  }

  TileColorContext ctx = MakeTileColorContext(world, landValue, traffic, goods);
  if (haveServices) {
    ctx.services = &services;
  }
  if (haveNoise) {
    ctx.noise = &noise;
  }
  if (haveLandUseMix) {
    ctx.landUseMix = &landUseMix;
  }
  if (haveHeatIsland) {
    ctx.heatIsland = &heatIsland;
  }
  if (haveAirPollution) {
    ctx.airPollution = &airPollution;
  }
  if (haveRunoff) {
    ctx.runoff = &runoff;
  }
  if (haveRunoffMitigation) {
    ctx.runoffMitigation = &runoffMitigation;
  }
  if (haveSolar) {
    ctx.solar = &solar;
  }
  if (haveSkyView) {
    ctx.skyView = &skyView;
  }
  if (haveEnergy) {
    ctx.energy = &energy;
  }
  if (haveCarbon) {
    ctx.carbon = &carbon;
  }
  if (haveCrime) {
    ctx.crime = &crime;
  }
  if (haveTrafficSafety) {
    ctx.trafficSafety = &trafficSafety;
  }
  if (haveTransit) {
    ctx.transit = &transit;
  }
  if (haveFireRisk) {
    ctx.fireRisk = &fireRisk;
  }
  if (haveWalkability) {
    ctx.walkability = &walkability;
  }
  if (haveJobs) {
    ctx.jobs = &jobs;
  }
  if (haveRoadHealth) {
    ctx.roadHealth = &roadHealth;
  }
  if (haveLivability) {
    ctx.livability = &livability;
  }
  if (haveLivHot) {
    ctx.livabilityHotspot = &livHot;
  }
  if (havePriHot) {
    ctx.interventionHotspot = &priHot;
  }
  if (haveZoneAccess) {
    ctx.zoneAccess = &zoneAccess;
  }
  if (haveSeaFlood) {
    ctx.seaFloodDepth = &seaFlood.depth;
    ctx.seaFloodMaxDepth = seaFlood.maxDepth;
    ctx.seaLevel = seaLevel;
  }
  if (havePonding) {
    ctx.pondingDepth = &ponding.depth;
    ctx.pondingMaxDepth = ponding.maxDepth;
  }


  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      std::uint8_t r, g, b;
      ComputeTileColor(world, x, y, layer, ctx, r, g, b);
      SetPixel(img.rgb, img.width, x, y, r, g, b);
    }
  }

  return img;
}

IsoOverviewResult RenderIsoOverview(const World& world, ExportLayer layer, const IsoOverviewConfig& cfg,
                                   const LandValueResult* landValue, const TrafficResult* traffic, const GoodsResult* goods,
                                   const GfxTilesetAtlas* tileset)
{
  IsoOverviewResult out{};
  out.tileW = cfg.tileW;
  out.tileH = cfg.tileH;
  out.heightScalePx = std::max(0, cfg.heightScalePx);

  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) return out;

  if (cfg.tileW < 2 || cfg.tileH < 2) return out;

  // Support both even and odd tile sizes. Internally we use half-width/half-height "diamond"
  // units for the iso projection.
  out.halfW = cfg.tileW / 2;
  out.halfH = cfg.tileH / 2;
  if (out.halfW <= 0 || out.halfH <= 0) return out;

  // Compute bounds in iso-space.
  int minX = std::numeric_limits<int>::max();
  int minY = std::numeric_limits<int>::max();
  int maxX = std::numeric_limits<int>::min();
  int maxY = std::numeric_limits<int>::min();

  for (int y = 0; y < mapH; ++y) {
    for (int x = 0; x < mapW; ++x) {
      const int hp = HeightToPx(world.at(x, y).height, out.heightScalePx);
      const Ipt c = IsoCenter(x, y, out.halfW, out.halfH, hp);

      const int leftX = c.x - out.halfW;
      const int rightX = c.x + out.halfW;
      const int topY = c.y - out.halfH;
      const int bottomY = c.y + out.halfH;

      minX = std::min(minX, leftX);
      maxX = std::max(maxX, rightX);
      minY = std::min(minY, topY);

      // Faces can extend below the tile by up to hp pixels.
      maxY = std::max(maxY, bottomY + hp);
    }
  }

  const int margin = std::max(0, cfg.marginPx);
  out.offsetX = -minX + margin;
  out.offsetY = -minY + margin;

  out.image.width = (maxX - minX + 1) + margin * 2 + 1;
  out.image.height = (maxY - minY + 1) + margin * 2 + 1;

  if (out.image.width <= 0 || out.image.height <= 0) return out;

  out.image.rgb.resize(static_cast<std::size_t>(out.image.width) * static_cast<std::size_t>(out.image.height) * 3u, 0);

  // Fill background.
  for (int y = 0; y < out.image.height; ++y) {
    for (int x = 0; x < out.image.width; ++x) {
      SetPixel(out.image.rgb, out.image.width, x, y, cfg.bgR, cfg.bgG, cfg.bgB);
    }
  }

  // Derived, heightfield-driven layers may require some precomputation.
  std::vector<float> heights;
  std::vector<std::uint8_t> drainMask;

  SeaFloodResult seaFlood{};
  DepressionFillResult ponding{};

  bool haveSeaFlood = false;
  bool havePonding = false;
  float seaLevel = 0.0f;

  if (layer == ExportLayer::FloodDepth || layer == ExportLayer::PondingDepth) {
    BuildHeightFieldAndDrainMask(world, heights, drainMask);
  }

  if (layer == ExportLayer::FloodDepth) {
    seaLevel = InferCoastalSeaLevel(world);
    SeaFloodConfig floodCfg{};
    floodCfg.requireEdgeConnection = true;
    floodCfg.eightConnected = false;
    seaFlood = ComputeSeaLevelFlood(heights, mapW, mapH, seaLevel, floodCfg);
    haveSeaFlood = true;
  }

  if (layer == ExportLayer::PondingDepth) {
    DepressionFillConfig pondCfg{};
    pondCfg.includeEdges = true;
    pondCfg.epsilon = 0.0f;
    ponding = FillDepressionsPriorityFlood(heights, mapW, mapH, &drainMask, pondCfg);
    havePonding = true;
  }


  ServicesResult services{};
  bool haveServices = false;
  if (layer == ExportLayer::ServicesOverall || layer == ExportLayer::ServicesEducation ||
      layer == ExportLayer::ServicesHealth || layer == ExportLayer::ServicesSafety) {
    ServicesModelSettings svcCfg{};
    svcCfg.enabled = true;
    svcCfg.requireOutsideConnection = true;
    svcCfg.weightMode = IsochroneWeightMode::TravelTime;
    svcCfg.catchmentRadiusSteps = 18;

    const std::vector<ServiceFacility> facilities = ExtractServiceFacilitiesFromWorld(world);
    services = ComputeServices(world, svcCfg, facilities);
    haveServices = true;
  }

  NoiseResult noise{};
  bool haveNoise = false;
  if (layer == ExportLayer::Noise) {
    NoiseConfig nc{};
    noise = ComputeNoisePollution(world, nc, traffic, goods);
    haveNoise = true;
  }

  LandUseMixResult landUseMix{};
  bool haveLandUseMix = false;
  if (layer == ExportLayer::LandUseMix) {
    LandUseMixConfig mc{};
    landUseMix = ComputeLandUseMix(world, mc);
    haveLandUseMix = true;
  }

  HeatIslandResult heatIsland{};
  bool haveHeatIsland = false;
  if (layer == ExportLayer::HeatIsland ||
      layer == ExportLayer::EnergyDemand || layer == ExportLayer::EnergySolar || layer == ExportLayer::EnergyBalance ||
      layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    HeatIslandConfig hc{};
    heatIsland = ComputeHeatIsland(world, hc, traffic, goods);
    haveHeatIsland = true;
  }


  AirPollutionResult airPollution{};
  bool haveAirPollution = false;
  if (layer == ExportLayer::AirPollution || layer == ExportLayer::AirPollutionEmission) {
    AirPollutionConfig ac{};
    // Keep exports deterministic, and pick a stable "prevailing wind" per seed.
    ac.windFromSeed = true;
    airPollution = ComputeAirPollution(world, ac, traffic, goods);
    haveAirPollution = true;
  }


  RunoffPollutionResult runoff{};
  bool haveRunoff = false;
  if (layer == ExportLayer::RunoffPollution || layer == ExportLayer::RunoffPollutionLoad) {
    RunoffPollutionConfig rc{};
    runoff = ComputeRunoffPollution(world, rc, traffic);
    haveRunoff = true;
  }

  RunoffMitigationResult runoffMitigation{};
  bool haveRunoffMitigation = false;
  if (layer == ExportLayer::RunoffMitigationPriority || layer == ExportLayer::RunoffMitigationPlan) {
    RunoffMitigationConfig mc{};
    // Reasonable defaults for a visual planning layer.
    mc.demandMode = RunoffMitigationDemandMode::ResidentialOccupants;
    mc.parksToAdd = 12;
    mc.minSeparation = 3;
    mc.excludeWater = true;
    mc.allowReplaceRoad = false;
    mc.allowReplaceZones = false;

    runoffMitigation = SuggestRunoffMitigationParks(world, mc, traffic);
    haveRunoffMitigation = true;
  }


  SolarPotentialResult solar{};
  bool haveSolar = false;
  if (layer == ExportLayer::SolarExposure || layer == ExportLayer::SolarPotential ||
      layer == ExportLayer::EnergyDemand || layer == ExportLayer::EnergySolar || layer == ExportLayer::EnergyBalance ||
      layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    SolarPotentialConfig sc{};
    // Keep exports deterministic: no random elements and stable shading sampling.
    sc.azimuthSamples = 16;
    solar = ComputeSolarPotential(world, sc);
    haveSolar = true;
  }


  SkyViewResult skyView{};
  bool haveSkyView = false;
  if (layer == ExportLayer::SkyView || layer == ExportLayer::CanyonConfinement ||
      layer == ExportLayer::TrafficCrashRisk || layer == ExportLayer::TrafficCrashExposure ||
      layer == ExportLayer::TrafficCrashPriority) {
    SkyViewConfig vc{};
    vc.azimuthSamples = 16;
    vc.maxHorizonRadius = 64;
    vc.includeBuildings = true;
    skyView = ComputeSkyViewFactor(world, vc);
    haveSkyView = true;
  }

  TrafficSafetyResult trafficSafety{};
  bool haveTrafficSafety = false;
  if (layer == ExportLayer::TrafficCrashRisk || layer == ExportLayer::TrafficCrashExposure ||
      layer == ExportLayer::TrafficCrashPriority) {
    TrafficSafetyConfig tc{};
    tc.requireOutsideConnection = true;
    tc.exposureRadius = 6;
    trafficSafety = ComputeTrafficSafety(world, tc, traffic, haveSkyView ? &skyView : nullptr);
    haveTrafficSafety = true;
  }




  EnergyModelResult energy{};
  bool haveEnergy = false;
  if (layer == ExportLayer::EnergyDemand || layer == ExportLayer::EnergySolar || layer == ExportLayer::EnergyBalance ||
      layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    EnergyModelConfig ec{};
    energy = ComputeEnergyModel(world, ec, haveSolar ? &solar : nullptr, haveHeatIsland ? &heatIsland : nullptr);
    haveEnergy = true;
  }



  CarbonModelResult carbon{};
  bool haveCarbon = false;
  if (layer == ExportLayer::CarbonEmission || layer == ExportLayer::CarbonSequestration || layer == ExportLayer::CarbonBalance) {
    CarbonModelConfig cc{};
    carbon = ComputeCarbonModel(world, cc, haveEnergy ? &energy : nullptr, traffic, goods);
    haveCarbon = true;
  }

  CrimeModelResult crime{};
  bool haveCrime = false;
  NoiseResult crimeNoise{};
  JobOpportunityResult crimeJobs{};
  bool haveCrimeJobs = false;
  if (layer == ExportLayer::CrimeRisk || layer == ExportLayer::PoliceAccess) {
    NoiseConfig nc{};
    crimeNoise = ComputeNoisePollution(world, nc, traffic, goods);

    JobOpportunityConfig jc{};
    jc.requireOutsideConnection = true;
    jc.useTravelTime = true;
    jc.congestionCosts = true;
    crimeJobs = ComputeJobOpportunity(world, jc, traffic, /*precomputedRoadToEdge=*/nullptr, /*precomputedZoneAccess=*/nullptr);
    haveCrimeJobs = true;

    CrimeModelConfig crc{};
    crc.requireOutsideConnection = true;
    crc.weightMode = IsochroneWeightMode::TravelTime;
    crime = ComputeCrimeModel(world, crc, traffic, goods, (haveCrimeJobs ? &crimeJobs : nullptr), &crimeNoise,
                              /*precomputedRoadToEdge=*/nullptr, /*precomputedZoneAccess=*/nullptr);
    haveCrime = true;
  }
  TransitAccessibilityResult transit{};
  bool haveTransit = false;
  if (layer == ExportLayer::TransitAccess || layer == ExportLayer::TransitModeSharePotential) {
    TransitAccessibilityConfig tc{};
    tc.requireOutsideConnection = true;
    TransitAccessibilityInputs ti{};
    ti.traffic = traffic;
    ti.goods = goods;
    transit = ComputeTransitAccessibility(world, tc, ti);
    haveTransit = true;
  }

  FireRiskResult fireRisk{};
  bool haveFireRisk = false;
  if (layer == ExportLayer::FireRisk) {
    FireRiskConfig fc{};
    fc.requireOutsideConnection = true;
    fc.weightMode = IsochroneWeightMode::TravelTime;
    fc.responseRadiusSteps = 18;
    fireRisk = ComputeFireRisk(world, fc);
    haveFireRisk = true;
  }

  WalkabilityResult walkability{};
  bool haveWalkability = false;
  if (layer == ExportLayer::Walkability || layer == ExportLayer::WalkabilityPark || layer == ExportLayer::WalkabilityRetail ||
      layer == ExportLayer::WalkabilityEducation || layer == ExportLayer::WalkabilityHealth || layer == ExportLayer::WalkabilitySafety) {
    WalkabilityConfig wc{};
    wc.enabled = true;
    wc.requireOutsideConnection = true;
    wc.weightMode = IsochroneWeightMode::TravelTime;
    wc.coverageThresholdSteps = 15;
    walkability = ComputeWalkability(world, wc);
    haveWalkability = true;
  }

  JobOpportunityResult jobs{};
  bool haveJobs = false;
  if (layer == ExportLayer::JobAccess || layer == ExportLayer::JobOpportunity) {
    JobOpportunityConfig jc{};
    jc.requireOutsideConnection = true;
    jc.useTravelTime = true;
    jc.congestionCosts = true;
    // Exports default to travel-time accessibility; when traffic is provided, congestion is incorporated.
    jobs = ComputeJobOpportunity(world, jc, traffic, /*precomputedRoadToEdge=*/nullptr, /*precomputedZoneAccess=*/nullptr);
    haveJobs = true;
  }


  RoadHealthResult roadHealth{};
  bool haveRoadHealth = false;
  if (layer == ExportLayer::RoadCentrality || layer == ExportLayer::RoadVulnerability || layer == ExportLayer::RoadBypass) {
    RoadHealthConfig rc{};
    rc.weightMode = RoadGraphEdgeWeightMode::TravelTimeMilli;
    rc.maxSources = 0; // auto
    rc.autoExactMaxNodes = 650;
    rc.autoSampleSources = 256;
    rc.includeNodeCentrality = true;
    rc.articulationVulnerabilityBase = 0.70f;

    // Bypass planning is fairly expensive; only do it for the bypass layer.
    rc.includeBypass = (layer == ExportLayer::RoadBypass);
    if (rc.includeBypass) {
      rc.bypassCfg.top = 3;
      rc.bypassCfg.moneyObjective = true;
      rc.bypassCfg.targetLevel = 1;
      rc.bypassCfg.allowBridges = false;
      rc.bypassCfg.rankByTraffic = true;
    }

    roadHealth = ComputeRoadHealth(world, rc, traffic);
    haveRoadHealth = true;
  }


  LivabilityResult livability{};
  bool haveLivability = false;
  if (layer == ExportLayer::Livability || layer == ExportLayer::InterventionPriority ||
      layer == ExportLayer::LivabilityHotspot || layer == ExportLayer::InterventionHotspot) {
    LivabilityConfig lc{};
    lc.requireOutsideConnection = true;
    lc.weightMode = IsochroneWeightMode::TravelTime;
    lc.servicesCatchmentRadiusSteps = 18;
    lc.walkCoverageThresholdSteps = 15;
    livability = ComputeLivability(world, lc, traffic, goods);
    haveLivability = true;
  }


  HotspotResult livHot{};
  HotspotResult priHot{};
  bool haveLivHot = false;
  bool havePriHot = false;
  if (layer == ExportLayer::LivabilityHotspot && haveLivability) {
    HotspotConfig hc{};
    hc.radius = 8;
    hc.excludeWater = true;
    hc.zThreshold = 1.96f;
    hc.zScale = 3.0f;
    livHot = ComputeHotspotsGiStar(world, livability.livability01, hc);
    haveLivHot = true;
  }
  if (layer == ExportLayer::InterventionHotspot && haveLivability) {
    HotspotConfig hc{};
    hc.radius = 8;
    hc.excludeWater = true;
    hc.zThreshold = 1.96f;
    hc.zScale = 3.0f;
    priHot = ComputeHotspotsGiStar(world, livability.priority01, hc);
    havePriHot = true;
  }


  ZoneAccessMap zoneAccess{};
  bool haveZoneAccess = false;
  if (layer == ExportLayer::ZonePressureResidential || layer == ExportLayer::ZonePressureCommercial ||
      layer == ExportLayer::ZonePressureIndustrial) {
    zoneAccess = BuildZoneAccessMap(world, nullptr);
    haveZoneAccess = true;
  }

  TileColorContext ctx = MakeTileColorContext(world, landValue, traffic, goods);
  if (haveServices) {
    ctx.services = &services;
  }
  if (haveNoise) {
    ctx.noise = &noise;
  }
  if (haveLandUseMix) {
    ctx.landUseMix = &landUseMix;
  }
  if (haveHeatIsland) {
    ctx.heatIsland = &heatIsland;
  }
  if (haveAirPollution) {
    ctx.airPollution = &airPollution;
  }
  if (haveRunoff) {
    ctx.runoff = &runoff;
  }
  if (haveRunoffMitigation) {
    ctx.runoffMitigation = &runoffMitigation;
  }
  if (haveSolar) {
    ctx.solar = &solar;
  }
  if (haveSkyView) {
    ctx.skyView = &skyView;
  }
  if (haveEnergy) {
    ctx.energy = &energy;
  }
  if (haveCarbon) {
    ctx.carbon = &carbon;
  }
  if (haveCrime) {
    ctx.crime = &crime;
  }
  if (haveTrafficSafety) {
    ctx.trafficSafety = &trafficSafety;
  }
  if (haveTransit) {
    ctx.transit = &transit;
  }
  if (haveFireRisk) {
    ctx.fireRisk = &fireRisk;
  }
  if (haveWalkability) {
    ctx.walkability = &walkability;
  }
  if (haveJobs) {
    ctx.jobs = &jobs;
  }
  if (haveRoadHealth) {
    ctx.roadHealth = &roadHealth;
  }
  if (haveLivability) {
    ctx.livability = &livability;
  }
  if (haveLivHot) {
    ctx.livabilityHotspot = &livHot;
  }
  if (havePriHot) {
    ctx.interventionHotspot = &priHot;
  }
  if (haveZoneAccess) {
    ctx.zoneAccess = &zoneAccess;
  }
  if (haveSeaFlood) {
    ctx.seaFloodDepth = &seaFlood.depth;
    ctx.seaFloodMaxDepth = seaFlood.maxDepth;
    ctx.seaLevel = seaLevel;
  }
  if (havePonding) {
    ctx.pondingDepth = &ponding.depth;
    ctx.pondingMaxDepth = ponding.maxDepth;
  }


  // Atmospheric styling is only meaningful for the visual layers.
  const bool allowAtmosphere = (layer == ExportLayer::Terrain || layer == ExportLayer::Overlay);

  const bool fancy = cfg.fancy && allowAtmosphere;

  // Optional: sprite-based rendering using a generated tileset atlas.
  // This is only used for the visual layers (Terrain/Overlay) and only when the atlas tile size
  // matches the requested iso tile size.
  const bool useTileset = fancy && tileset && tileset->valid() &&
                          tileset->tileW == cfg.tileW && tileset->tileH == cfg.tileH;
  const float texStrength = fancy ? std::clamp(cfg.textureStrength, 0.0f, 1.0f) : 0.0f;
  const bool drawShore = fancy && cfg.drawShore && texStrength > 0.0f;
  const bool drawRoadMarks = fancy && cfg.drawRoadMarkings;
  const bool drawZonePatterns = fancy && cfg.drawZonePatterns;

  // Fold the 64-bit world seed into 32 bits for cheap procedural noise.
  const std::uint32_t seed32 =
      static_cast<std::uint32_t>((world.seed() ^ (world.seed() >> 32)) & 0xFFFFFFFFu) ^ 0xA5F1523Du;

  // ------------------------------
  // Optional atmosphere (visual layers only)
  // ------------------------------
  const DayNightState dayNight = allowAtmosphere ? ComputeDayNightState(cfg.dayNight) : DayNightState{};

  const float wxIntensity = allowAtmosphere ? std::clamp(cfg.weather.intensity, 0.0f, 1.0f) : 0.0f;
  const bool wxRain = allowAtmosphere && (cfg.weather.mode == IsoOverviewConfig::WeatherConfig::Mode::Rain) && (wxIntensity > 0.001f);
  const bool wxSnow = allowAtmosphere && (cfg.weather.mode == IsoOverviewConfig::WeatherConfig::Mode::Snow) && (wxIntensity > 0.001f);
  const float wxOvercast = allowAtmosphere ? std::clamp(cfg.weather.overcast, 0.0f, 1.0f) : 0.0f;
  const float wxFog = allowAtmosphere ? std::clamp(cfg.weather.fog, 0.0f, 1.0f) : 0.0f;
  const float wetness = wxRain ? wxIntensity : 0.0f;
  const float snowCover = wxSnow ? wxIntensity : 0.0f;

  const bool cloudsEnabled = allowAtmosphere && cfg.clouds.enabled;
  const float cloudCoverage = std::clamp(cfg.clouds.coverage, 0.0f, 1.0f);
  const float cloudStrength = std::clamp(cfg.clouds.strength, 0.0f, 1.0f);
  const float cloudScaleTiles = std::max(1.0f, cfg.clouds.scaleTiles);
  const float cloudOffX = cfg.clouds.offsetX;
  const float cloudOffY = cfg.clouds.offsetY;

  auto popCount4 = [](std::uint8_t m) -> int {
    return ((m & 0x01u) ? 1 : 0) + ((m & 0x02u) ? 1 : 0) + ((m & 0x04u) ? 1 : 0) + ((m & 0x08u) ? 1 : 0);
  };

  // Shared road connectivity mask for tileset-driven sprites and prop placement.
  // Bits: 1=North, 2=East, 4=South, 8=West.
  auto roadMaskAt = [&](int rx, int ry) -> std::uint8_t {
    if (!world.inBounds(rx, ry)) return 0;
    const Tile& rt = world.at(rx, ry);
    if (rt.overlay != Overlay::Road) return 0;
    std::uint8_t m = static_cast<std::uint8_t>(rt.variation & 0x0Fu);
    if (m != 0) return m;
    if (world.inBounds(rx, ry - 1) && world.at(rx, ry - 1).overlay == Overlay::Road) m |= 0x01u;
    if (world.inBounds(rx + 1, ry) && world.at(rx + 1, ry).overlay == Overlay::Road) m |= 0x02u;
    if (world.inBounds(rx, ry + 1) && world.at(rx, ry + 1).overlay == Overlay::Road) m |= 0x04u;
    if (world.inBounds(rx - 1, ry) && world.at(rx - 1, ry).overlay == Overlay::Road) m |= 0x08u;
    return m;
  };

  // Optional per-pixel lighting for tileset sprites (normal map shading) and shadow masks.
  TilesetLightingState tilesetLight{};
  const bool tilesetNormalMap = useTileset && tileset && cfg.tilesetLighting.enableNormals && tileset->normalValid();
  if (tilesetNormalMap) {
    // Key light intensity is reduced at night and with heavy overcast.
    const float dayK = (allowAtmosphere && cfg.dayNight.enabled) ? dayNight.day : 1.0f;
    const float overcastK = 1.0f - 0.75f * wxOvercast;
    const float strength = std::clamp(cfg.tilesetLighting.normalStrength, 0.0f, 1.0f) * dayK * overcastK;

    tilesetLight.normalAtlas = &tileset->normalAtlas;
    tilesetLight.enabled = (strength > 0.001f);
    tilesetLight.strength = strength;

    // Normalize light direction.
    float lx = cfg.tilesetLighting.lightDirX;
    float ly = cfg.tilesetLighting.lightDirY;
    float lz = cfg.tilesetLighting.lightDirZ;
    const float len = std::sqrt(lx * lx + ly * ly + lz * lz);
    if (len > 1.0e-6f) {
      lx /= len;
      ly /= len;
      lz /= len;
    } else {
      lx = 0.0f;
      ly = 0.0f;
      lz = 1.0f;
    }
    tilesetLight.lx = lx;
    tilesetLight.ly = ly;
    tilesetLight.lz = lz;
    tilesetLight.flatDot = std::max(1.0e-4f, lz);

    tilesetLight.ambient = std::clamp(cfg.tilesetLighting.ambient, 0.0f, 1.0f);
    tilesetLight.diffuse = std::clamp(cfg.tilesetLighting.diffuse, 0.0f, 2.0f);
  }

  const bool tilesetShadowMap = useTileset && tileset && cfg.tilesetLighting.enableShadows && tileset->shadowValid();
  float tilesetShadowStrength = 0.0f;
  if (tilesetShadowMap) {
    const float dayK = (allowAtmosphere && cfg.dayNight.enabled) ? dayNight.day : 1.0f;
    const float overcastK = 1.0f - 0.75f * wxOvercast;
    tilesetShadowStrength = std::clamp(cfg.tilesetLighting.shadowStrength, 0.0f, 1.0f) * dayK * overcastK;
  }

  // Optional: decorative prop placement when using a tileset atlas.
  const bool tilesetProps = useTileset && cfg.tilesetProps.enabled && allowAtmosphere && (layer == ExportLayer::Overlay);
  const float parkTreeDensity = tilesetProps ? std::clamp(cfg.tilesetProps.treeDensity, 0.0f, 1.0f) : 0.0f;
  const float parkConiferChance = tilesetProps ? std::clamp(cfg.tilesetProps.coniferChance, 0.0f, 1.0f) : 0.0f;
  const bool tilesetStreetlights = tilesetProps && cfg.tilesetProps.drawStreetlights;
  const float streetlightChance = tilesetStreetlights ? std::clamp(cfg.tilesetProps.streetlightChance, 0.0f, 1.0f) : 0.0f;

  auto distPointSegment = [](float px, float py, float ax, float ay, float bx, float by, float& outT) -> float {
    const float vx = bx - ax;
    const float vy = by - ay;
    const float wx = px - ax;
    const float wy = py - ay;
    const float len2 = vx * vx + vy * vy;
    float t = 0.0f;
    if (len2 > 1.0e-6f) t = (wx * vx + wy * vy) / len2;
    t = std::clamp(t, 0.0f, 1.0f);
    outT = t;
    const float cx = ax + t * vx;
    const float cy = ay + t * vy;
    const float dx = px - cx;
    const float dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
  };

  struct RoadStyle {
    float roadW = 0.14f;      // half-width in normalized diamond coords
    float lineThick = 0.010f; // marking half-thickness
    float lineGap = 0.018f;   // used for double center lines
    float laneOff = 0.05f;    // highway lane offset from center
    float shoulderOff = 0.10f;
    float edgeDark = 0.78f;
    float dashFreq = 10.0f;

    std::uint8_t ar = 90, ag = 90, ab = 95;        // asphalt
    std::uint8_t mr = 220, mg = 220, mb = 210;     // marking (white)
    std::uint8_t m2r = 250, m2g = 220, m2b = 110;  // marking2 (yellow-ish)

    bool dashed = true;
    bool doubleCenter = false;
    bool highway = false;
    bool crosswalk = false;
  };

  auto roadStyleForLevel = [&](int level) -> RoadStyle {
    RoadStyle st{};
    level = std::clamp(level, 1, 3);

    if (level == 1) {
      st.roadW = 0.130f;
      st.ar = 95; st.ag = 95; st.ab = 100;
      st.mr = 235; st.mg = 235; st.mb = 230;
      st.dashFreq = 10.0f;
      st.dashed = true;
      st.doubleCenter = false;
      st.highway = false;
      st.crosswalk = true;
      st.edgeDark = 0.78f;
    } else if (level == 2) {
      st.roadW = 0.175f;
      st.ar = 85; st.ag = 85; st.ab = 90;
      st.mr = 240; st.mg = 240; st.mb = 240;
      st.m2r = 250; st.m2g = 215; st.m2b = 95;
      st.dashed = false;
      st.doubleCenter = true;
      st.lineGap = 0.022f;
      st.lineThick = 0.008f;
      st.crosswalk = true;
      st.edgeDark = 0.74f;
    } else { // level 3
      st.roadW = 0.215f;
      st.ar = 72; st.ag = 72; st.ab = 76;
      st.mr = 245; st.mg = 245; st.mb = 245;
      st.dashed = true;
      st.doubleCenter = false;
      st.highway = true;
      st.dashFreq = 14.0f;
      st.lineThick = 0.0075f;
      st.laneOff = st.roadW * 0.34f;
      st.shoulderOff = st.roadW * 0.78f;
      st.crosswalk = false;
      st.edgeDark = 0.70f;
    }

    // Scale a few parameters so markings don't disappear on very small tiles (e.g. 16x8 default).
    const float pxNorm = 0.85f / static_cast<float>(std::max(1, out.halfW));
    st.lineThick = std::max(st.lineThick, pxNorm);
    st.lineGap = std::max(st.lineGap, st.lineThick * 2.2f);
    if (st.highway) {
      st.laneOff = std::max(st.laneOff, st.roadW * 0.28f);
      st.shoulderOff = std::max(st.shoulderOff, st.roadW * 0.70f);
    }
    // Fewer dashes at low resolution.
    const float dashScale = std::clamp(static_cast<float>(out.halfW) / 32.0f, 0.25f, 2.0f);
    st.dashFreq *= dashScale;

    // Minimum road width in pixels.
    const float minHalfPx = (level == 1) ? 1.20f : (level == 2) ? 1.55f : 1.85f;
    st.roadW = std::max(st.roadW, minHalfPx / static_cast<float>(std::max(1, out.halfW)));

    return st;
  };

  auto computeTileBrightness = [&](int tx, int ty) -> float {
    const Tile& t = world.at(tx, ty);

    // Stable per-tile variation: upper bits only (low 4 are road masks).
    const float var01 = static_cast<float>((t.variation >> 4) & 0x0Fu) / 15.0f;

    // Base: height + slight random.
    float b = 0.86f + 0.22f * Clamp01(t.height) + (var01 - 0.5f) * 0.10f;

    // Slope shading via height gradients in map space.
    auto hAt = [&](int x, int y) -> float {
      if (!world.inBounds(x, y)) return Clamp01(t.height);
      return Clamp01(world.at(x, y).height);
    };

    const float h0 = hAt(tx, ty);
    const float hN = hAt(tx, ty - 1);
    const float hS = hAt(tx, ty + 1);
    const float hW = hAt(tx - 1, ty);
    const float hE = hAt(tx + 1, ty);

    const float dzdx = (hE - hW);
    const float dzdy = (hS - hN);

    // Approximate normal = normalize(-dzdx, -dzdy, k).
    float nx = -dzdx;
    float ny = -dzdy;
    float nz = 1.35f;
    const float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (nlen > 1.0e-6f) { nx /= nlen; ny /= nlen; nz /= nlen; }

    // Sun from northwest/up.
    const float sx = -0.62f;
    const float sy = -0.55f;
    const float sz = 0.56f;
    const float ndot = nx * sx + ny * sy + nz * sz;
    const float flat = sz; // dot((0,0,1), sun)
    b += (ndot - flat) * 0.55f;

    // Ambient occlusion from "behind" neighbors (north/west in map space).
    const float occ = std::max(0.0f, hW - h0) + std::max(0.0f, hN - h0);
    b -= occ * 0.30f;

    return std::clamp(b, 0.55f, 1.30f);
  };

  auto terrainAt = [&](int tx, int ty) -> Terrain {
    if (!world.inBounds(tx, ty)) return Terrain::Grass; // treat OOB as land for nicer borders
    return world.at(tx, ty).terrain;
  };

  // Draw order: back-to-front along diagonals (increasing x+y).
  for (int sum = 0; sum <= (mapW - 1) + (mapH - 1); ++sum) {
    for (int x = 0; x < mapW; ++x) {
      const int y = sum - x;
      if (y < 0 || y >= mapH) continue;

      const Tile& t = world.at(x, y);
      const int hp = HeightToPx(t.height, out.heightScalePx);

      const Ipt cIso = IsoCenter(x, y, out.halfW, out.halfH, hp);
      const int cx = cIso.x + out.offsetX;
      const int cy = cIso.y + out.offsetY;

      const Ipt top{cx, cy - out.halfH};
      const Ipt right{cx + out.halfW, cy};
      const Ipt bottom{cx, cy + out.halfH};
      const Ipt left{cx - out.halfW, cy};

      // Base shading used for cliff faces.
      std::uint8_t br = 0, bg = 0, bb = 0;
      if (fancy) {
        TerrainBaseColor(t, br, bg, bb);
        const float b = computeTileBrightness(x, y);
        const float heightLight = 0.90f + 0.10f * Clamp01(t.height);
        MulPixel(br, bg, bb, b * heightLight);
      } else {
        ComputeTileColor(world, x, y, layer, ctx, br, bg, bb);
        const float heightLight = 0.90f + 0.10f * Clamp01(t.height);
        MulPixel(br, bg, bb, heightLight);
      }

      // Vertical faces for height differences (optional).
      if (cfg.drawCliffs && out.heightScalePx > 0) {
        // East neighbor (x+1,y) corresponds to the right edge.
        const int hpE = (x + 1 < mapW) ? HeightToPx(world.at(x + 1, y).height, out.heightScalePx) : 0;
        const int dhR = hp - hpE;
        if (dhR > 0) {
          std::uint8_t fr = br, fg = bg, fb = bb;
          MulPixel(fr, fg, fb, 0.65f);
          FillQuad(out.image, right, bottom, Ipt{bottom.x, bottom.y + dhR}, Ipt{right.x, right.y + dhR}, fr, fg, fb);
        }

        // South neighbor (x,y+1) corresponds to the left edge.
        const int hpS = (y + 1 < mapH) ? HeightToPx(world.at(x, y + 1).height, out.heightScalePx) : 0;
        const int dhL = hp - hpS;
        if (dhL > 0) {
          std::uint8_t fr = br, fg = bg, fb = bb;
          MulPixel(fr, fg, fb, 0.55f);
          FillQuad(out.image, bottom, left, Ipt{left.x, left.y + dhL}, Ipt{bottom.x, bottom.y + dhL}, fr, fg, fb);
        }
      }

      // -------------------------------------------------------------------------------------
      // Optional tileset-atlas path (Terrain/Overlay only).
      // -------------------------------------------------------------------------------------
      if (useTileset) {
        const float tileB = computeTileBrightness(x, y);

        auto pickTerrain = [&]() -> const GfxAtlasEntry* {
          const int tv = std::max(1, tileset->terrainVariants > 0 ? tileset->terrainVariants : 8);
          const int var = static_cast<int>(((t.variation >> 4) & 0x0Fu) % static_cast<std::uint8_t>(tv));

          if (t.terrain == Terrain::Water && drawShore && tileset->transitionVariantsWS > 0) {
            const bool nBase = (terrainAt(x, y - 1) == Terrain::Water);
            const bool eBase = (terrainAt(x + 1, y) == Terrain::Water);
            const bool sBase = (terrainAt(x, y + 1) == Terrain::Water);
            const bool wBase = (terrainAt(x - 1, y) == Terrain::Water);
            const bool need = !(nBase && eBase && sBase && wBase);
            if (need) {
              std::uint8_t mask = 0;
              if (nBase) mask |= 0x01u;
              if (eBase) mask |= 0x02u;
              if (sBase) mask |= 0x04u;
              if (wBase) mask |= 0x08u;
              const int vv = std::max(1, tileset->transitionVariantsWS);
              const int v = static_cast<int>(((t.variation >> 4) & 0x0Fu) % static_cast<std::uint8_t>(vv));
              const std::string name = "terrain_shore_ws_m" + std::to_string(mask) + "_v" + std::to_string(v);
              if (const GfxAtlasEntry* e = FindGfxAtlasEntry(*tileset, name)) return e;
            }
          }

          if (t.terrain == Terrain::Sand && drawShore && tileset->transitionVariantsSG > 0) {
            // Only apply sand->grass transitions when there is nearby grass.
            const bool nGrass = (terrainAt(x, y - 1) == Terrain::Grass);
            const bool eGrass = (terrainAt(x + 1, y) == Terrain::Grass);
            const bool sGrass = (terrainAt(x, y + 1) == Terrain::Grass);
            const bool wGrass = (terrainAt(x - 1, y) == Terrain::Grass);
            const bool need = (nGrass || eGrass || sGrass || wGrass);
            if (need) {
              std::uint8_t mask = 0;
              // Mask bits mean "neighbor is base sand". Treat water as "sand" here so we don't
              // accidentally blend grass along coastlines.
              if (terrainAt(x, y - 1) != Terrain::Grass) mask |= 0x01u;
              if (terrainAt(x + 1, y) != Terrain::Grass) mask |= 0x02u;
              if (terrainAt(x, y + 1) != Terrain::Grass) mask |= 0x04u;
              if (terrainAt(x - 1, y) != Terrain::Grass) mask |= 0x08u;
              const int vv = std::max(1, tileset->transitionVariantsSG);
              const int v = static_cast<int>(((t.variation >> 4) & 0x0Fu) % static_cast<std::uint8_t>(vv));
              const std::string name = "terrain_shore_sg_m" + std::to_string(mask) + "_v" + std::to_string(v);
              if (const GfxAtlasEntry* e = FindGfxAtlasEntry(*tileset, name)) return e;
            }
          }

          if (t.terrain == Terrain::Water) {
            const std::string name = "terrain_water_v" + std::to_string(var);
            return FindGfxAtlasEntry(*tileset, name);
          }
          if (t.terrain == Terrain::Sand) {
            const std::string name = "terrain_sand_v" + std::to_string(var);
            return FindGfxAtlasEntry(*tileset, name);
          }
          const std::string name = "terrain_grass_v" + std::to_string(var);
          return FindGfxAtlasEntry(*tileset, name);
        };

        const GfxAtlasEntry* base = pickTerrain();
        if (base) {
          BlitAtlasSpriteOverLit(out.image, tileset->atlas, &tilesetLight, *base, cx - base->pivotX, cy - base->pivotY, tileB);
        } else {
          // Defensive fallback.
          FillTriangle(out.image, top, right, bottom, br, bg, bb);
          FillTriangle(out.image, top, bottom, left, br, bg, bb);
        }

        if (layer == ExportLayer::Overlay) {
          // Roads.
          if (t.overlay == Overlay::Road) {
            std::uint8_t roadMask = roadMaskAt(x, y);

            const int lvl = std::clamp<int>(t.level, 1, 3);
            const int vv = static_cast<int>((t.variation >> 4) & 0x0Fu);
            const bool isBridge = (t.terrain == Terrain::Water);

            const int vcount = std::max(1, isBridge ? (tileset->bridgeVariants > 0 ? tileset->bridgeVariants : 4)
                                                   : (tileset->roadVariants > 0 ? tileset->roadVariants : 4));
            const int v = vv % vcount;

            const std::string name = std::string(isBridge ? "bridge" : "road") +
                                     "_L" + std::to_string(lvl) +
                                     "_m" + std::to_string(static_cast<int>(roadMask)) +
                                     "_v" + std::to_string(v);
            if (const GfxAtlasEntry* re = FindGfxAtlasEntry(*tileset, name)) {
              BlitAtlasSpriteOverLit(out.image, tileset->atlas, &tilesetLight, *re, cx - re->pivotX, cy - re->pivotY, tileB);
            }
          } else if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial ||
                     t.overlay == Overlay::Park) {
            // Overlays.
            if (t.overlay == Overlay::Park || drawZonePatterns) {
              const char* oname = nullptr;
              if (t.overlay == Overlay::Residential) oname = "overlay_residential";
              else if (t.overlay == Overlay::Commercial) oname = "overlay_commercial";
              else if (t.overlay == Overlay::Industrial) oname = "overlay_industrial";
              else if (t.overlay == Overlay::Park) oname = "overlay_park";

              if (oname) {
                if (const GfxAtlasEntry* oe = FindGfxAtlasEntry(*tileset, std::string(oname))) {
                  BlitAtlasSpriteOverLit(out.image, tileset->atlas, &tilesetLight, *oe, cx - oe->pivotX, cy - oe->pivotY, tileB);
                }
              }
            }

            // Buildings: only for occupied zones.
            if (t.occupants > 0 && (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial)) {
              const int lvl = std::clamp<int>(t.level, 1, 3);
              int kind = 0;
              const char* kname = "res";
              if (t.overlay == Overlay::Commercial) { kind = 1; kname = "com"; }
              else if (t.overlay == Overlay::Industrial) { kind = 2; kname = "ind"; }
              const int vcount = tileset->buildingVariants[kind][lvl - 1];
              if (vcount > 0) {
                const std::uint32_t hv = HashCoords32(x, y, seed32 ^ 0xD15EA5E1u);
                const int v = static_cast<int>(hv % static_cast<std::uint32_t>(vcount));
                const std::string bname = std::string("building_") + kname + "_L" + std::to_string(lvl) + "_v" + std::to_string(v);
                if (const GfxAtlasEntry* be = FindGfxAtlasEntry(*tileset, bname)) {
                  if (tilesetShadowMap && tilesetShadowStrength > 0.001f && be->srcH > tileset->tileH) {
                    BlitAtlasSpriteShadowMultiply(out.image, tileset->shadowAtlas, *be,
                                                 cx - be->pivotX, cy - be->pivotY, tilesetShadowStrength);
                  }
                  BlitAtlasSpriteOverLit(out.image, tileset->atlas, &tilesetLight, *be, cx - be->pivotX, cy - be->pivotY, tileB);
                }
              }
            }
          }
        }

        // Optional deterministic decorative props for tileset-based rendering.
        if (tilesetProps) {
          auto hash01 = [](std::uint32_t u) -> float {
            // 24-bit mantissa (same idea as RNG::nextF01).
            return static_cast<float>(u >> 8) / static_cast<float>(1u << 24);
          };

          // Park trees.
          if (t.overlay == Overlay::Park && parkTreeDensity > 0.001f &&
              (tileset->propTreeDeciduousVariants > 0 || tileset->propTreeConiferVariants > 0)) {
            // Up to 2 trees per park tile at high density.
            const std::uint32_t h0 = HashCoords32(x, y, seed32 ^ 0x2D1B5A49u);
            const std::uint32_t h1 = HashCoords32(x, y, seed32 ^ 0xA12F6B73u);
            int count = 0;
            if (hash01(h0) < parkTreeDensity) count += 1;
            if (parkTreeDensity > 0.5f && hash01(h1) < (parkTreeDensity - 0.5f) * 2.0f) count += 1;

            for (int i = 0; i < count; ++i) {
              const std::uint32_t ht = HashCoords32(x, y, seed32 ^ (0x6C8E9CF5u + static_cast<std::uint32_t>(i) * 0x9E3779B9u));
              const bool wantConifer = (hash01(ht ^ 0x93A5C4E1u) < parkConiferChance);
              const int decidCount = tileset->propTreeDeciduousVariants;
              const int conifCount = tileset->propTreeConiferVariants;
              const bool useConifer = wantConifer && (conifCount > 0);
              const int vcount = useConifer ? conifCount : decidCount;
              if (vcount <= 0) continue;
              const int v = static_cast<int>(ht % static_cast<std::uint32_t>(vcount));
              const std::string pname = useConifer ? (std::string("prop_tree_conifer_v") + std::to_string(v))
                                                  : (std::string("prop_tree_deciduous_v") + std::to_string(v));

              if (const GfxAtlasEntry* pe = FindGfxAtlasEntry(*tileset, pname)) {
                // Local offset inside the diamond to break up the grid.
                float ox = (static_cast<float>((ht & 0xFFu)) / 255.0f) * 2.0f - 1.0f;
                float oy = (static_cast<float>(((ht >> 8) & 0xFFu)) / 255.0f) * 2.0f - 1.0f;
                const float ax = std::abs(ox);
                const float ay = std::abs(oy);
                if (ax + ay > 1.0f) {
                  ox = std::copysign(1.0f - ay, ox);
                  oy = std::copysign(1.0f - ax, oy);
                }
                const float spread = 0.38f;
                const int px = cx + static_cast<int>(std::lround(ox * static_cast<float>(out.halfW) * spread));
                const int py = cy + static_cast<int>(std::lround(oy * static_cast<float>(out.halfH) * spread));

                if (tilesetShadowMap && tilesetShadowStrength > 0.001f && pe->srcH > tileset->tileH) {
                  BlitAtlasSpriteShadowMultiply(out.image, tileset->shadowAtlas, *pe,
                                               px - pe->pivotX, py - pe->pivotY, tilesetShadowStrength);
                }
                BlitAtlasSpriteOverLit(out.image, tileset->atlas, &tilesetLight, *pe,
                                       px - pe->pivotX, py - pe->pivotY, tileB);
              }
            }
          }

          // Road streetlights.
          if (tilesetStreetlights && t.overlay == Overlay::Road && streetlightChance > 0.001f && tileset->propStreetlightVariants > 0) {
            const std::uint8_t roadMask = roadMaskAt(x, y);
            const std::uint32_t hl = HashCoords32(x, y, seed32 ^ 0x57E371A1u);
            if (hash01(hl) < streetlightChance) {
              const int v = static_cast<int>(hl % static_cast<std::uint32_t>(tileset->propStreetlightVariants));
              const std::string lname = std::string("prop_streetlight_v") + std::to_string(v);
              if (const GfxAtlasEntry* le = FindGfxAtlasEntry(*tileset, lname)) {
                const bool ns = (roadMask & 0x01u) || (roadMask & 0x04u);
                const bool ew = (roadMask & 0x02u) || (roadMask & 0x08u);
                const bool flip = ((hl >> 16) & 1u) != 0;
                float ox = 0.0f;
                float oy = 0.0f;
                if (ns && !ew) {
                  ox = flip ? 0.32f : -0.32f;
                  oy = 0.02f;
                } else if (ew && !ns) {
                  ox = 0.0f;
                  oy = flip ? 0.22f : -0.22f;
                } else {
                  ox = flip ? 0.28f : -0.28f;
                  oy = 0.16f;
                }
                const int px = cx + static_cast<int>(std::lround(ox * static_cast<float>(out.halfW)));
                const int py = cy + static_cast<int>(std::lround(oy * static_cast<float>(out.halfH)));

                if (tilesetShadowMap && tilesetShadowStrength > 0.001f && le->srcH > tileset->tileH) {
                  BlitAtlasSpriteShadowMultiply(out.image, tileset->shadowAtlas, *le,
                                               px - le->pivotX, py - le->pivotY, tilesetShadowStrength);
                }
                BlitAtlasSpriteOverLit(out.image, tileset->atlas, &tilesetLight, *le,
                                       px - le->pivotX, py - le->pivotY, tileB);
              }
            }
          }
        }

        if (cfg.drawGrid) {
          const std::uint8_t lr = 25, lg = 25, lb = 25;
          DrawLine(out.image, top.x, top.y, right.x, right.y, lr, lg, lb);
          DrawLine(out.image, right.x, right.y, bottom.x, bottom.y, lr, lg, lb);
          DrawLine(out.image, bottom.x, bottom.y, left.x, left.y, lr, lg, lb);
          DrawLine(out.image, left.x, left.y, top.x, top.y, lr, lg, lb);
        }

        continue;
      }

      if (!fancy) {
        // Simple per-tile fill.
        FillTriangle(out.image, top, right, bottom, br, bg, bb);
        FillTriangle(out.image, top, bottom, left, br, bg, bb);
      } else {
        float tileB = computeTileBrightness(x, y);

        // Optional cloud shadows: dappled sunlight that modulates overall tile brightness.
        if (cloudsEnabled && cloudStrength > 0.001f && cloudCoverage > 0.001f && dayNight.day > 0.05f) {
          const float cx01 = (static_cast<float>(x) + 0.35f + cloudOffX) / cloudScaleTiles;
          const float cy01 = (static_cast<float>(y) + 0.65f + cloudOffY) / cloudScaleTiles;
          const float n = FBm2D(cx01, cy01, seed32 ^ 0xC10D5EEDu, 4);

          const float thr = 1.0f - cloudCoverage;
          const float dens = SmoothStep(thr, std::min(1.0f, thr + 0.26f), n);

          // Overcast kills hard shadows; at night we skip entirely.
          const float vis = dayNight.day * (1.0f - wxOvercast);
          const float shadow = dens * cloudStrength * vis;

          tileB *= (1.0f - shadow * 0.45f);
        }

        // Neighbor terrain for shoreline shading.
        auto neighTerrain = [&](int nx, int ny) -> Terrain {
          if (!world.inBounds(nx, ny)) return Terrain::Grass; // treat out-of-bounds as land for nicer borders
          return world.at(nx, ny).terrain;
        };

        const bool isWater = (t.terrain == Terrain::Water);
        const bool nIsWater = (neighTerrain(x, y - 1) == Terrain::Water);
        const bool eIsWater = (neighTerrain(x + 1, y) == Terrain::Water);
        const bool sIsWater = (neighTerrain(x, y + 1) == Terrain::Water);
        const bool wIsWater = (neighTerrain(x - 1, y) == Terrain::Water);

        const bool nLand = !nIsWater;
        const bool eLand = !eIsWater;
        const bool sLand = !sIsWater;
        const bool wLand = !wIsWater;

        // Precompute road info (overlay layer only).
        const bool isRoad = (layer == ExportLayer::Overlay && t.overlay == Overlay::Road);
        std::uint8_t roadMask = static_cast<std::uint8_t>(t.variation & 0x0Fu);
        if (isRoad && roadMask == 0) {
          // Fallback: recompute mask from neighbors (defensive for older saves).
          std::uint8_t m = 0;
          if (world.inBounds(x, y - 1) && world.at(x, y - 1).overlay == Overlay::Road) m |= 1u << 0;
          if (world.inBounds(x + 1, y) && world.at(x + 1, y).overlay == Overlay::Road) m |= 1u << 1;
          if (world.inBounds(x, y + 1) && world.at(x, y + 1).overlay == Overlay::Road) m |= 1u << 2;
          if (world.inBounds(x - 1, y) && world.at(x - 1, y).overlay == Overlay::Road) m |= 1u << 3;
          roadMask = m;
        }
        const int roadConn = popCount4(roadMask);
        const int level = std::clamp<int>(t.level, 1, 3);
        const int variant = static_cast<int>((t.variation >> 4) & 0x0Fu);
        const bool isBridge = isRoad && isWater;

        RoadStyle st{};
        std::uint32_t roadSeed = 0;
        std::uint8_t deckR = 0, deckG = 0, deckB = 0;
        if (isRoad) {
          st = roadStyleForLevel(level);
          roadSeed =
              seed32 ^ 0x0F0F0F0Fu ^ (static_cast<std::uint32_t>(roadMask) * 0x9E3779B9u) ^
              (static_cast<std::uint32_t>(variant) * 0x85EBCA6Bu) ^
              (static_cast<std::uint32_t>(level) * 0xC2B2AE35u) ^
              (static_cast<std::uint32_t>(x) * 0x27D4EB2Du) ^
              (static_cast<std::uint32_t>(y) * 0x165667B1u);

          // Bridge deck palette per road level.
          deckR = 160; deckG = 130; deckB = 95;
          if (level == 2) { deckR = 170; deckG = 170; deckB = 175; }
          if (level == 3) { deckR = 150; deckG = 150; deckB = 155; }
        }

        // Zone colors for overlay shapes.
        std::uint8_t zoneR = 0, zoneG = 0, zoneB = 0;
        if (layer == ExportLayer::Overlay && t.overlay != Overlay::None && t.overlay != Overlay::Road) {
          OverlayColor(t, zoneR, zoneG, zoneB);
        }

        for (int dy = -out.halfH; dy <= out.halfH; ++dy) {
          const int py = cy + dy;
          if (py < 0 || py >= out.image.height) continue;

          for (int dx = -out.halfW; dx <= out.halfW; ++dx) {
            const int px = cx + dx;
            if (px < 0 || px >= out.image.width) continue;

            const float nx = static_cast<float>(dx) / static_cast<float>(out.halfW);
            const float ny = static_cast<float>(dy) / static_cast<float>(out.halfH);
            const float man = std::fabs(nx) + std::fabs(ny);
            if (man > 1.0f) continue;

            const float edge = 1.0f - man;

            // Base terrain color.
            std::uint8_t tr = 0, tg = 0, tb = 0;
            TerrainBaseColor(t, tr, tg, tb);

            float rr = static_cast<float>(tr);
            float gg = static_cast<float>(tg);
            float bb2 = static_cast<float>(tb);

            // Base lighting: per-tile + slight directional gradient on the top surface.
            const float heightLight = 0.90f + 0.10f * Clamp01(t.height);
            float shade = tileB * heightLight;
            shade *= (0.92f + 0.08f * edge);
            shade *= 1.0f + 0.06f * std::clamp(((-nx - ny) * 0.25f + 0.5f), 0.0f, 1.0f);

            // Terrain micro texture.
            if (texStrength > 0.0f) {
              const std::uint32_t h = HashCoords32(dx + x * 131, dy + y * 173, seed32 ^ (static_cast<std::uint32_t>(t.variation) << 8));
              const float n = (Frac01(h) - 0.5f);

              float amp = 0.08f;
              if (t.terrain == Terrain::Grass) amp = 0.12f;
              else if (t.terrain == Terrain::Sand) amp = 0.10f;
              else if (t.terrain == Terrain::Water) amp = 0.08f;

              shade *= 1.0f + n * amp * texStrength;

              // Occasional speckles / pebbles.
              if (t.terrain == Terrain::Grass && ((h & 0x7Fu) == 0x3Fu)) shade *= 0.88f;
              if (t.terrain == Terrain::Sand && ((h & 0xFFu) == 0xA1u)) shade *= 0.86f;

              // Water wave highlights (subtle).
              if (t.terrain == Terrain::Water) {
                const int wave = (static_cast<int>(std::floor((nx + ny) * 6.0f + static_cast<float>((x + y) & 7))) & 3);
                if (wave == 0 && edge > 0.06f) shade *= 1.06f;
              }
            }

            rr *= shade;
            gg *= shade;
            bb2 *= shade;

            // Shoreline foam / beach highlight (fancy only).
            if (drawShore) {
              const float foamW = 0.14f; // in normalized "edge" units
              if (edge < foamW) {
                // Determine which edge segment this pixel lies on via quadrant.
                int dir = 0;
                if (nx >= 0.0f && ny < 0.0f) dir = 0;         // north edge (top-right) -> neighbor (x, y-1)
                else if (nx > 0.0f && ny >= 0.0f) dir = 1;    // east edge (bottom-right) -> neighbor (x+1, y)
                else if (nx <= 0.0f && ny > 0.0f) dir = 2;    // south edge (bottom-left) -> neighbor (x, y+1)
                else dir = 3;                                 // west edge (top-left) -> neighbor (x-1, y)

                const float tEdge = std::clamp((foamW - edge) / foamW, 0.0f, 1.0f);

                if (isWater) {
                  const bool landAdj = (dir == 0 ? nLand : dir == 1 ? eLand : dir == 2 ? sLand : wLand);
                  if (landAdj) {
                    // Foam tint.
                    const float a = tEdge * 0.85f;
                    const float fr = 235.0f;
                    const float fg = 242.0f;
                    const float fb = 246.0f;
                    rr = rr * (1.0f - a) + fr * a;
                    gg = gg * (1.0f - a) + fg * a;
                    bb2 = bb2 * (1.0f - a) + fb * a;
                  }
                } else {
                  const bool waterAdj = (dir == 0 ? nIsWater : dir == 1 ? eIsWater : dir == 2 ? sIsWater : wIsWater);
                  if (waterAdj) {
                    // Slight bright rim on land near water.
                    const float a = tEdge * 0.28f;
                    rr = rr * (1.0f - a) + rr * 1.12f * a;
                    gg = gg * (1.0f - a) + gg * 1.12f * a;
                    bb2 = bb2 * (1.0f - a) + bb2 * 1.10f * a;
                  }
                }
              }
            }

            // Overlay layer details.
            if (layer == ExportLayer::Overlay) {
              if (isRoad) {
                // Road signed-distance field (matches in-app road mask directions).
                const float pxn = nx;
                const float pyn = ny;
                const float centerR = st.roadW * 1.10f;
                float sd = std::sqrt(pxn * pxn + pyn * pyn) - centerR;

                float bestSegDist = 1.0e9f;
                float bestSegT = 0.0f;
                float bestEx = 0.0f;
                float bestEy = 0.0f;

                auto consider = [&](bool enabled, float ex, float ey) {
                  if (!enabled) return;
                  float tproj = 0.0f;
                  const float dist = distPointSegment(pxn, pyn, 0.0f, 0.0f, ex, ey, tproj);
                  sd = std::min(sd, dist - st.roadW);
                  if (dist < bestSegDist) {
                    bestSegDist = dist;
                    bestSegT = tproj;
                    bestEx = ex;
                    bestEy = ey;
                  }
                };

                consider((roadMask & 0x01u) != 0u, 0.5f, -0.5f);  // up-right
                consider((roadMask & 0x02u) != 0u, 0.5f, 0.5f);   // down-right
                consider((roadMask & 0x04u) != 0u, -0.5f, 0.5f);  // down-left
                consider((roadMask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

                if (sd <= 0.0f) {
                  // Road base color (asphalt or bridge deck).
                  float cr = static_cast<float>(isBridge ? deckR : st.ar);
                  float cg = static_cast<float>(isBridge ? deckG : st.ag);
                  float cb = static_cast<float>(isBridge ? deckB : st.ab);

                  // Noise / wear.
                  const std::uint32_t h = HashCoords32(dx + x * 251, dy + y * 397, roadSeed);
                  const float n = (Frac01(h) - 0.5f) * 0.10f * (0.35f + 0.65f * texStrength);
                  cr *= (1.0f + n);
                  cg *= (1.0f + n);
                  cb *= (1.0f + n);

                  if ((h & 0x7Fu) == 0x3Fu) { cr *= 0.86f; cg *= 0.86f; cb *= 0.86f; }
                  if ((h & 0xFFu) == 0xA1u) { cr *= 1.06f; cg *= 1.06f; cb *= 1.06f; }

                  // Darken near edge (curb feel).
                  const float distToEdge = -sd;
                  const float edgeW = std::max(0.004f, st.roadW * 0.22f);
                  if (distToEdge < edgeW) {
                    const float tt = std::clamp(distToEdge / edgeW, 0.0f, 1.0f);
                    const float mul = st.edgeDark + (1.0f - st.edgeDark) * tt;
                    cr *= mul; cg *= mul; cb *= mul;
                  }

                  // Markings (optional).
                  if (drawRoadMarks && roadConn > 0 && bestSegDist < (st.roadW * 0.55f) &&
                      (std::sqrt(pxn * pxn + pyn * pyn) > centerR * 0.60f)) {
                    const float segLen = std::sqrt(bestEx * bestEx + bestEy * bestEy);
                    if (segLen > 1.0e-6f) {
                      const float vx = bestEx / segLen;
                      const float vy = bestEy / segLen;
                      const float cxp = bestSegT * bestEx;
                      const float cyp = bestSegT * bestEy;
                      const float ddx = pxn - cxp;
                      const float ddy = pyn - cyp;
                      const float signedPerp = ddx * (-vy) + ddy * (vx);
                      const float absPerp = std::fabs(signedPerp);

                      // Crosswalk stripes near intersections.
                      if (st.crosswalk && roadConn >= 3 && bestSegT > 0.12f && bestSegT < 0.28f && absPerp < st.roadW * 0.92f) {
                        const float stripeW = std::max(0.030f, (2.2f / static_cast<float>(std::max(1, out.halfW))));
                        const int stripe = static_cast<int>(std::floor((signedPerp + st.roadW) / stripeW +
                                                                      static_cast<float>(roadMask) * 0.10f));
                        if ((stripe & 1) == 0) { cr = 250.0f; cg = 250.0f; cb = 250.0f; }
                      }

                      if (st.highway) {
                        // Highway: shoulders + dashed lane lines.
                        const float thick = st.lineThick;
                        if (std::fabs(absPerp - st.shoulderOff) < (thick * 1.25f)) {
                          cr = static_cast<float>(st.mr); cg = static_cast<float>(st.mg); cb = static_cast<float>(st.mb);
                        } else if (std::fabs(absPerp - st.laneOff) < thick) {
                          const int dash = static_cast<int>(std::floor(bestSegT * st.dashFreq + static_cast<float>(roadMask) * 0.21f +
                                                                       static_cast<float>(variant) * 0.37f));
                          if ((dash & 1) == 0) { cr = static_cast<float>(st.mr); cg = static_cast<float>(st.mg); cb = static_cast<float>(st.mb); }
                        }
                      } else if (st.doubleCenter) {
                        // Avenue: double solid median.
                        if (std::fabs(absPerp - st.lineGap) < st.lineThick) {
                          cr = static_cast<float>(st.m2r); cg = static_cast<float>(st.m2g); cb = static_cast<float>(st.m2b);
                        }
                      } else {
                        // Street: dashed centerline.
                        if (absPerp < st.lineThick) {
                          const int dash = static_cast<int>(std::floor(bestSegT * st.dashFreq + static_cast<float>(roadMask) * 0.15f +
                                                                       static_cast<float>(variant) * 0.23f));
                          if ((dash & 1) == 0) { cr = static_cast<float>(st.mr); cg = static_cast<float>(st.mg); cb = static_cast<float>(st.mb); }
                        }
                      }
                    }
                  }

                  // Soft edge blend.
                  const float edgeSoft = std::max(0.03f, 0.75f / static_cast<float>(std::max(1, out.halfW)));
                  const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
                  rr = rr * (1.0f - a) + cr * a;
                  gg = gg * (1.0f - a) + cg * a;
                  bb2 = bb2 * (1.0f - a) + cb * a;

                  // Simple bridge rails (subtle).
                  if (isBridge && std::fabs(bestSegDist - st.roadW) < st.lineThick * 1.2f) {
                    rr = rr * 0.65f + 210.0f * 0.35f;
                    gg = gg * 0.65f + 210.0f * 0.35f;
                    bb2 = bb2 * 0.65f + 210.0f * 0.35f;
                  }
                }
              } else if (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial) {
                // Draw a smaller "roof" diamond so the underlying terrain still reads.
                const float roofSize = (out.halfW >= 12) ? 0.70f : 0.66f;
                const float roofEdge = roofSize - man;
                if (roofEdge > 0.0f) {
                  float cr = static_cast<float>(zoneR);
                  float cg = static_cast<float>(zoneG);
                  float cb = static_cast<float>(zoneB);

                  // Roof lighting.
                  float roofShade = 0.94f + 0.10f * std::clamp(((-nx - ny) * 0.25f + 0.5f), 0.0f, 1.0f);
                  const std::uint32_t h = HashCoords32(dx + x * 97, dy + y * 191, seed32 ^ 0xBADC0DEu);
                  roofShade *= 1.0f + (Frac01(h) - 0.5f) * 0.06f * texStrength;

                  // Patterns if there are enough pixels to show them.
                  if (drawZonePatterns && out.halfW >= 12) {
                    if (t.overlay == Overlay::Residential) {
                      // Shingles: alternating rows.
                      const int stripe = ((dx + out.halfW) / 2 + (dy + out.halfH)) & 1;
                      if (stripe == 0) roofShade *= 0.92f;
                    } else if (t.overlay == Overlay::Commercial) {
                      // Window bands.
                      const int stripe = ((dx + out.halfW) / 2) & 1;
                      if (stripe == 0) { cr *= 0.88f; cg *= 0.88f; cb *= 0.90f; }
                    } else {
                      // Industrial: diagonal hazard-ish stripes.
                      const int stripe = ((dx - dy + 64) / 2) & 1;
                      if (stripe == 0) roofShade *= 0.90f;
                    }
                  }

                  cr *= roofShade;
                  cg *= roofShade;
                  cb *= roofShade;

                  // Border line.
                  if (roofEdge < 0.06f) {
                    cr *= 0.70f;
                    cg *= 0.70f;
                    cb *= 0.70f;
                  }

                  const float a = std::clamp(roofEdge / 0.06f, 0.0f, 1.0f);
                  rr = rr * (1.0f - a) + cr * a;
                  gg = gg * (1.0f - a) + cg * a;
                  bb2 = bb2 * (1.0f - a) + cb * a;
                }
              } else if (t.overlay == Overlay::Park) {
                // Park tint + dotted trees.
                const float a = 0.75f;
                const float pr = 40.0f;
                const float pg = 140.0f;
                const float pb = 60.0f;
                rr = rr * (1.0f - a) + pr * a;
                gg = gg * (1.0f - a) + pg * a;
                bb2 = bb2 * (1.0f - a) + pb * a;

                if (drawZonePatterns && man < 0.88f) {
                  const std::uint32_t h = HashCoords32(dx + x * 53, dy + y * 71, seed32 ^ 0xC0FFEEu);
                  if ((h & 0x1Fu) == 0x07u) {
                    rr *= 0.65f;
                    gg *= 0.75f;
                    bb2 *= 0.65f;
                  }
                }
              }
            }

            // Weather surface styling (applied before post-process grading).
            if (allowAtmosphere) {
              if (wxSnow && snowCover > 0.001f && t.terrain != Terrain::Water) {
                // Snow cover: bias toward white/blue with subtle micro variation.
                float s = snowCover * (0.65f + 0.35f * wxOvercast);

                // Roads and industrial roofs get less accumulation; sand retains a bit of warmth.
                if (t.overlay == Overlay::Road) s *= 0.38f;
                if (t.overlay == Overlay::Industrial) s *= 0.75f;
                if (t.terrain == Terrain::Sand) s *= 0.65f;

                const std::uint32_t hs = HashCoords32(px + x * 193, py + y * 317, seed32 ^ 0x51A7E1u);
                const float n = (Frac01(hs) - 0.5f);
                s *= std::clamp(0.92f + n * 0.22f * texStrength, 0.0f, 1.0f);

                const float sr = 245.0f;
                const float sg = 250.0f;
                const float sb = 255.0f;
                rr = rr * (1.0f - s) + sr * s;
                gg = gg * (1.0f - s) + sg * s;
                bb2 = bb2 * (1.0f - s) + sb * s;
              }

              if (wxRain && wetness > 0.001f) {
                // Wet surfaces: slightly darken + add tiny glints/puddles on roads.
                const float w = wetness * (0.75f + 0.25f * wxOvercast);
                rr *= (1.0f - 0.08f * w);
                gg *= (1.0f - 0.08f * w);
                bb2 *= (1.0f - 0.06f * w);

                if (t.overlay == Overlay::Road && man < 0.92f) {
                  const std::uint32_t hpud = HashCoords32(dx + x * 59, dy + y * 101, seed32 ^ 0xB00B135u);
                  if ((hpud & 0x3Fu) == 0x1Du) {
                    const float p = 0.14f * w;
                    rr = rr * (1.0f - p) + 200.0f * p;
                    gg = gg * (1.0f - p) + 215.0f * p;
                    bb2 = bb2 * (1.0f - p) + 235.0f * p;
                  }
                }
              }
            }

            const std::uint8_t outR = ToByte(rr);
            const std::uint8_t outG = ToByte(gg);
            const std::uint8_t outB = ToByte(bb2);
            SetPixel(out.image.rgb, out.image.width, px, py, outR, outG, outB);
          }
        }
      }

      if (cfg.drawGrid) {
        // Dark outline to help depth perception.
        const std::uint8_t lr = 25, lg = 25, lb = 25;
        DrawLine(out.image, top.x, top.y, right.x, right.y, lr, lg, lb);
        DrawLine(out.image, right.x, right.y, bottom.x, bottom.y, lr, lg, lb);
        DrawLine(out.image, bottom.x, bottom.y, left.x, left.y, lr, lg, lb);
        DrawLine(out.image, left.x, left.y, top.x, top.y, lr, lg, lb);
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // Post-process atmosphere (Terrain/Overlay iso exports only)
  // -------------------------------------------------------------------------------------------
  if (allowAtmosphere) {
    // Overcast / haze first, then day-night grade.
    ApplyOvercastGrade(out.image, wxOvercast, wxSnow);

    // Fog blends toward the background (acting as a simple "sky" color).
    // In rainy/snowy modes, bias the fog color slightly cooler so it reads as moisture.
    std::uint8_t fr = cfg.bgR;
    std::uint8_t fg = cfg.bgG;
    std::uint8_t fb = cfg.bgB;
    if (wxRain) {
      fr = static_cast<std::uint8_t>((static_cast<int>(fr) + 120) / 2);
      fg = static_cast<std::uint8_t>((static_cast<int>(fg) + 135) / 2);
      fb = static_cast<std::uint8_t>((static_cast<int>(fb) + 160) / 2);
    } else if (wxSnow) {
      fr = static_cast<std::uint8_t>((static_cast<int>(fr) + 205) / 2);
      fg = static_cast<std::uint8_t>((static_cast<int>(fg) + 220) / 2);
      fb = static_cast<std::uint8_t>((static_cast<int>(fb) + 240) / 2);
    }
    ApplyFogGradient(out.image, wxFog, fr, fg, fb);

    ApplyDayNightGrade(out.image, dayNight, cfg.dayNight);

    // Emissive night lights (roads + occupied zones).
    if (cfg.dayNight.enabled && cfg.dayNight.drawLights && dayNight.night > 0.02f) {
      const float nightK = std::clamp(dayNight.night * std::clamp(cfg.dayNight.lightStrength, 0.0f, 2.0f), 0.0f, 1.0f);

      auto roadMaskAtNL = [&](int rx, int ry) -> std::uint8_t {
        if (!world.inBounds(rx, ry)) return 0;
        const Tile& rt = world.at(rx, ry);
        if (rt.overlay != Overlay::Road) return 0;
        std::uint8_t m = static_cast<std::uint8_t>(rt.variation & 0x0Fu);
        if (m != 0) return m;
        std::uint8_t nm = 0;
        if (world.inBounds(rx, ry - 1) && world.at(rx, ry - 1).overlay == Overlay::Road) nm |= 0x1u;
        if (world.inBounds(rx + 1, ry) && world.at(rx + 1, ry).overlay == Overlay::Road) nm |= 0x2u;
        if (world.inBounds(rx, ry + 1) && world.at(rx, ry + 1).overlay == Overlay::Road) nm |= 0x4u;
        if (world.inBounds(rx - 1, ry) && world.at(rx - 1, ry).overlay == Overlay::Road) nm |= 0x8u;
        return nm;
      };

      const float reflK = (cfg.weather.reflectLights && wxRain) ? (wetness * (0.40f + 0.60f * wxOvercast)) : 0.0f;

      for (int ty = 0; ty < mapH; ++ty) {
        for (int tx = 0; tx < mapW; ++tx) {
          const Tile& t = world.at(tx, ty);
          const int hp = HeightToPx(t.height, out.heightScalePx);
          const int cx = (tx - ty) * out.halfW + out.offsetX;
          const int cy = (tx + ty) * out.halfH - hp + out.offsetY;

          // Roads: streetlights and intersection glows.
          if (t.overlay == Overlay::Road) {
            const std::uint8_t mask = roadMaskAtNL(tx, ty);
            const int conn = popCount4(mask);
            const bool intersection = (conn >= 3);
            const bool major = (static_cast<int>(t.level) >= 2);

            // If we have atlas streetlight props + an emissive atlas, prefer the sprite-based
            // light over a generic point glow (avoids double-lighting and looks more stable).
            bool usedStreetlightSprite = false;
            if (useTileset && tileset && tileset->emissiveValid() && tilesetStreetlights && streetlightChance > 0.001f &&
                tileset->propStreetlightVariants > 0) {
              const std::uint32_t hl = HashCoords32(tx, ty, seed32 ^ 0x57E371A1u);
              if (Frac01(hl) < streetlightChance) {
                const int v = static_cast<int>(hl % static_cast<std::uint32_t>(tileset->propStreetlightVariants));
                const std::string lname = std::string("prop_streetlight_v") + std::to_string(v);
                if (const GfxAtlasEntry* le = FindGfxAtlasEntry(*tileset, lname)) {
                  const bool ns = (mask & 0x01u) || (mask & 0x04u);
                  const bool ew = (mask & 0x02u) || (mask & 0x08u);
                  const bool flip = ((hl >> 16) & 1u) != 0;
                  float ox = 0.0f;
                  float oy = 0.0f;
                  if (ns && !ew) {
                    ox = flip ? 0.32f : -0.32f;
                    oy = 0.02f;
                  } else if (ew && !ns) {
                    ox = 0.0f;
                    oy = flip ? 0.22f : -0.22f;
                  } else {
                    ox = flip ? 0.28f : -0.28f;
                    oy = 0.16f;
                  }

                  const int px = cx + static_cast<int>(std::lround(ox * static_cast<float>(out.halfW)));
                  const int py = cy + static_cast<int>(std::lround(oy * static_cast<float>(out.halfH)));
                  BlitAtlasSpriteAdditive(out.image, tileset->emissiveAtlas, *le, px - le->pivotX, py - le->pivotY, nightK);
                  usedStreetlightSprite = true;
                }
              }
            }

            // Deterministic sparsity: not every road tile gets a lamp.
            const std::uint32_t h = HashCoords32(tx, ty, seed32 ^ 0x4C1A55u);
            const float p = intersection ? 0.92f : major ? 0.45f : 0.28f;
            if (!usedStreetlightSprite && Frac01(h) < p) {
              const int ly = cy - static_cast<int>(std::lround(static_cast<float>(out.tileH) * 0.10f));
              const float baseR = std::max(2.5f, static_cast<float>(out.tileH) * (intersection ? 1.10f : major ? 0.90f : 0.80f));
              const float inten = nightK * (intersection ? 0.95f : major ? 0.75f : 0.65f);
              AddGlow(out.image, cx, ly, baseR, 255, 205, 135, inten);

              // Wet reflections: pull light downward slightly.
              if (reflK > 0.02f) {
                const float reflI = inten * reflK;
                AddGlow(out.image, cx, ly + static_cast<int>(std::lround(baseR * 0.65f)), baseR * 0.85f, 255, 205, 135,
                        reflI * 0.55f);
                AddGlow(out.image, cx, ly + static_cast<int>(std::lround(baseR * 1.30f)), baseR * 0.70f, 255, 205, 135,
                        reflI * 0.35f);
              }
            }
          }

          // Zones: window/building glow scaled by occupancy.
          if (IsZoneOverlay(t.overlay)) {
            // If we have an emissive tileset atlas and a matching building sprite, prefer it over
            // the generic point-glow heuristic (gives stable, nicer-looking "window" lighting).
            bool usedAtlasEmissive = false;
            if (useTileset && tileset && tileset->emissiveValid() && t.occupants > 0) {
              const int lvl = std::clamp<int>(t.level, 1, 3);
              int kind = 0;
              const char* kname = "res";
              if (t.overlay == Overlay::Commercial) { kind = 1; kname = "com"; }
              else if (t.overlay == Overlay::Industrial) { kind = 2; kname = "ind"; }
              const int vcount = tileset->buildingVariants[kind][lvl - 1];
              if (vcount > 0) {
                const std::uint32_t hv = HashCoords32(tx, ty, seed32 ^ 0xE11A5EEDu);
                const int v = static_cast<int>(hv % static_cast<std::uint32_t>(vcount));
                const std::string bname = std::string("building_") + kname + "_L" + std::to_string(lvl) + "_v" + std::to_string(v);
                if (const GfxAtlasEntry* be = FindGfxAtlasEntry(*tileset, bname)) {
                  // Additive blend after the day/night grade (so it stays bright at night).
                  BlitAtlasSpriteAdditive(out.image, tileset->emissiveAtlas, *be, cx - be->pivotX, cy - be->pivotY,
                                          nightK * 1.15f);
                  usedAtlasEmissive = true;
                }
              }
            }

            if (usedAtlasEmissive) {
              // Skip generic per-tile glows to avoid double-lighting.
              continue;
            }

            const int cap = CapacityForTile(t);
            const float occ01 = (cap > 0) ? std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f) : 0.0f;

            float litChance = 0.20f;
            std::uint8_t lr = 255, lg = 235, lb = 200;
            if (t.overlay == Overlay::Residential) {
              litChance = 0.10f + 0.75f * occ01;
              lr = 255; lg = 236; lb = 200;
            } else if (t.overlay == Overlay::Commercial) {
              litChance = 0.25f + 0.55f * occ01;
              lr = 200; lg = 225; lb = 255;
            } else if (t.overlay == Overlay::Industrial) {
              litChance = 0.06f + 0.35f * occ01;
              lr = 255; lg = 210; lb = 150;
            }

            const int count = 1 + std::clamp(static_cast<int>(t.level), 1, 3);
            for (int k = 0; k < count; ++k) {
              const std::uint32_t hk = HashCoords32(tx + k * 17, ty + k * 31, seed32 ^ 0xBADC0FFEu);
              if (Frac01(hk) > litChance) continue;

              // Deterministic point in diamond: sample (-1..1) and fold into |x|+|y|<=1.
              float ox = Frac01(hk ^ 0x9E3779B9u) * 2.0f - 1.0f;
              float oy = Frac01(hk ^ 0xBB67AE85u) * 2.0f - 1.0f;
              if (std::fabs(ox) + std::fabs(oy) > 1.0f) {
                ox = (ox > 0.0f) ? (1.0f - ox) : (-1.0f - ox);
                oy = (oy > 0.0f) ? (1.0f - oy) : (-1.0f - oy);
              }

              const int px = cx + static_cast<int>(std::lround(ox * static_cast<float>(out.halfW) * 0.55f));
              const int py = cy + static_cast<int>(std::lround(oy * static_cast<float>(out.halfH) * 0.35f)) -
                             static_cast<int>(std::lround(static_cast<float>(out.tileH) * 0.18f));

              const float rad = std::max(2.0f, static_cast<float>(out.tileH) * 0.38f);
              const float inten = nightK * 0.40f;
              AddGlow(out.image, px, py, rad, lr, lg, lb, inten);

              if (reflK > 0.02f && t.overlay == Overlay::Commercial) {
                // Commercial zones produce slightly stronger wet reflections.
                AddGlow(out.image, px, py + static_cast<int>(std::lround(rad * 1.15f)), rad * 0.80f, lr, lg, lb,
                        inten * reflK * 0.35f);
              }
            }
          }
        }
      }
    }

    // Precipitation overlays for rain/snow.
    if (cfg.weather.drawPrecipitation && wxIntensity > 0.02f) {
      const float k = wxIntensity;
      if (wxRain) {
        // Long, subtle diagonal streaks.
        const int n = std::clamp(static_cast<int>(k * 0.00030f * static_cast<float>(out.image.width * out.image.height)), 50, 800);
        for (int i = 0; i < n; ++i) {
          const std::uint32_t h = HashCoords32(i, static_cast<int>(seed32 ^ 0x2A9E4F11u), seed32 ^ 0x51A7E1u);
          const int sx = static_cast<int>(h % static_cast<std::uint32_t>(out.image.width));
          const int sy = static_cast<int>((h / 131u) % static_cast<std::uint32_t>(out.image.height));

          const float jx = (Frac01(h ^ 0x1234u) - 0.5f) * 0.35f;
          const float dx = 0.65f + jx;
          const float dy = 1.95f;
          const int len = std::clamp(static_cast<int>(6 + Frac01(h ^ 0xBEEFu) * 14.0f), 6, 22);

          for (int s = 0; s < len; ++s) {
            const int x = sx + static_cast<int>(std::lround(dx * static_cast<float>(s)));
            const int y = sy + static_cast<int>(std::lround(dy * static_cast<float>(s)));
            const float a = 0.10f * k * (1.0f - static_cast<float>(s) / static_cast<float>(len));
            BlendPixel(out.image.rgb, out.image.width, x, y, 225, 235, 255, a);
          }
        }
      } else if (wxSnow) {
        // Small bright flakes.
        const int n = std::clamp(static_cast<int>(k * 0.00060f * static_cast<float>(out.image.width * out.image.height)), 80, 1400);
        for (int i = 0; i < n; ++i) {
          const std::uint32_t h = HashCoords32(i, static_cast<int>(seed32 ^ 0x5A0B1A7Du), seed32 ^ 0xC0FFEEu);
          const int cx = static_cast<int>(h % static_cast<std::uint32_t>(out.image.width));
          const int cy = static_cast<int>((h / 257u) % static_cast<std::uint32_t>(out.image.height));
          const float a = 0.18f * k;
          BlendPixel(out.image.rgb, out.image.width, cx, cy, 245, 250, 255, a);
          // Occasional 2px sparkle.
          if ((h & 0x1Fu) == 0x0Bu) {
            BlendPixel(out.image.rgb, out.image.width, cx + 1, cy, 245, 250, 255, a * 0.75f);
          }
        }
      }
    }
  }

  return out;
}

bool IsoTileCenterToPixel(const World& world, const IsoOverviewResult& iso, int tx, int ty, int& outPx, int& outPy)
{
  if (!world.inBounds(tx, ty)) return false;
  if (iso.halfW <= 0 || iso.halfH <= 0) return false;

  const int hp = HeightToPx(world.at(tx, ty).height, iso.heightScalePx);
  outPx = (tx - ty) * iso.halfW + iso.offsetX;
  outPy = (tx + ty) * iso.halfH - hp + iso.offsetY;
  return true;
}

PpmImage ScaleNearest(const PpmImage& src, int factor)
{
  if (factor <= 1) return src;
  if (src.width <= 0 || src.height <= 0) return src;
  if (src.rgb.size() != static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 3u) return src;

  PpmImage out{};
  out.width = src.width * factor;
  out.height = src.height * factor;
  out.rgb.resize(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 3u);

  for (int y = 0; y < out.height; ++y) {
    const int sy = y / factor;
    for (int x = 0; x < out.width; ++x) {
      const int sx = x / factor;
      const std::size_t sidx = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(sx)) * 3u;
      const std::size_t didx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x)) * 3u;
      out.rgb[didx + 0] = src.rgb[sidx + 0];
      out.rgb[didx + 1] = src.rgb[sidx + 1];
      out.rgb[didx + 2] = src.rgb[sidx + 2];
    }
  }

  return out;
}

bool WritePpm(const std::string& path, const PpmImage& img, std::string& outError)
{
  outError.clear();

  if (img.width <= 0 || img.height <= 0) {
    outError = "Invalid image dimensions";
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u;
  if (img.rgb.size() != expected) {
    std::ostringstream oss;
    oss << "Invalid image buffer size (expected " << expected << ", got " << img.rgb.size() << ")";
    outError = oss.str();
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for writing";
    return false;
  }

  f << "P6\n" << img.width << " " << img.height << "\n255\n";
  f.write(reinterpret_cast<const char*>(img.rgb.data()), static_cast<std::streamsize>(img.rgb.size()));
  if (!f) {
    outError = "Failed while writing file";
    return false;
  }
  return true;
}

namespace {

static bool ReadPpmToken(std::istream& in, std::string& out)
{
  out.clear();

  char c = 0;
  // Skip whitespace and comments.
  while (in.get(c)) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isspace(uc)) continue;
    if (c == '#') {
      std::string dummy;
      std::getline(in, dummy);
      continue;
    }
    out.push_back(c);
    break;
  }

  if (out.empty()) return false;

  // Read until next whitespace (or comment start).
  while (in.get(c)) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isspace(uc)) break;
    if (c == '#') {
      std::string dummy;
      std::getline(in, dummy);
      break;
    }
    out.push_back(c);
  }

  return true;
}

static bool ParseI32Token(const std::string& tok, int& out)
{
  if (tok.empty()) return false;
  std::istringstream iss(tok);
  int v = 0;
  iss >> v;
  if (!iss || !iss.eof()) return false;
  out = v;
  return true;
}

} // namespace

bool ReadPpm(const std::string& path, PpmImage& outImg, std::string& outError)
{
  outError.clear();
  outImg = PpmImage{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for reading";
    return false;
  }

  std::string tok;
  if (!ReadPpmToken(f, tok) || tok != "P6") {
    outError = "Invalid PPM magic (expected P6)";
    return false;
  }

  int w = 0, h = 0, maxv = 0;
  if (!ReadPpmToken(f, tok) || !ParseI32Token(tok, w) || w <= 0) {
    outError = "Invalid PPM width";
    return false;
  }
  if (!ReadPpmToken(f, tok) || !ParseI32Token(tok, h) || h <= 0) {
    outError = "Invalid PPM height";
    return false;
  }
  if (!ReadPpmToken(f, tok) || !ParseI32Token(tok, maxv) || maxv <= 0) {
    outError = "Invalid PPM maxval";
    return false;
  }
  if (maxv > 255) {
    outError = "Unsupported PPM maxval (>255)";
    return false;
  }

  const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u;
  std::vector<std::uint8_t> buf(expected);

  f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
  if (!f || static_cast<std::size_t>(f.gcount()) != expected) {
    outError = "Failed while reading pixel data";
    return false;
  }

  // Scale to 0..255 if maxval != 255.
  if (maxv != 255) {
    for (std::uint8_t& c : buf) {
      const int v = static_cast<int>(c);
      const int scaled = (v * 255 + maxv / 2) / maxv;
      c = static_cast<std::uint8_t>(std::clamp(scaled, 0, 255));
    }
  }

  outImg.width = w;
  outImg.height = h;
  outImg.rgb = std::move(buf);
  return true;
}

bool ComparePpm(const PpmImage& a, const PpmImage& b, PpmDiffStats& outStats, int threshold, PpmImage* outDiff,
                int ssimWindow)
{
  outStats = PpmDiffStats{};

  if (a.width <= 0 || a.height <= 0 || b.width <= 0 || b.height <= 0) return false;
  if (a.width != b.width || a.height != b.height) return false;

  const std::size_t expected = static_cast<std::size_t>(a.width) * static_cast<std::size_t>(a.height) * 3u;
  if (a.rgb.size() != expected || b.rgb.size() != expected) return false;

  const int thr = std::clamp(threshold, 0, 255);

  outStats.width = a.width;
  outStats.height = a.height;
  outStats.pixelsCompared = static_cast<std::uint64_t>(a.width) * static_cast<std::uint64_t>(a.height);

  if (outDiff) {
    outDiff->width = a.width;
    outDiff->height = a.height;
    outDiff->rgb.assign(expected, 0u);
  }

  double sumAbs = 0.0;
  double sumSq = 0.0;
  std::uint8_t maxAbs = 0;

  // Per-pixel compare (track pixelsDifferent with threshold).
  const int w = a.width;
  const int h = a.height;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;

      const int dr = std::abs(static_cast<int>(a.rgb[idx + 0]) - static_cast<int>(b.rgb[idx + 0]));
      const int dg = std::abs(static_cast<int>(a.rgb[idx + 1]) - static_cast<int>(b.rgb[idx + 1]));
      const int db = std::abs(static_cast<int>(a.rgb[idx + 2]) - static_cast<int>(b.rgb[idx + 2]));

      maxAbs = static_cast<std::uint8_t>(std::max<int>(static_cast<int>(maxAbs), std::max({dr, dg, db})));

      sumAbs += static_cast<double>(dr + dg + db);
      sumSq += static_cast<double>(dr * dr + dg * dg + db * db);

      const bool diff = (dr > thr) || (dg > thr) || (db > thr);
      if (diff) outStats.pixelsDifferent++;

      if (outDiff) {
        (*outDiff).rgb[idx + 0] = static_cast<std::uint8_t>((dr > thr) ? dr : 0);
        (*outDiff).rgb[idx + 1] = static_cast<std::uint8_t>((dg > thr) ? dg : 0);
        (*outDiff).rgb[idx + 2] = static_cast<std::uint8_t>((db > thr) ? db : 0);
      }
    }
  }

  outStats.maxAbsDiff = maxAbs;

  const double denom = static_cast<double>(outStats.pixelsCompared) * 3.0;
  if (denom > 0.0) {
    outStats.meanAbsDiff = sumAbs / denom;
    outStats.mse = sumSq / denom;
  }

  if (outStats.mse <= 0.0) {
    outStats.psnr = std::numeric_limits<double>::infinity();
  } else {
    const double peak = 255.0;
    outStats.psnr = 10.0 * std::log10((peak * peak) / outStats.mse);
  }

  // ---------------------------------------------------------------------------
  // SSIM (Structural Similarity Index) on luma.
  //
  // We use a simple uniform window SSIM (box filter) with a caller-provided
  // window size. This is fast enough for regression tooling and provides a much
  // better correlation with perceived differences than raw MSE/PSNR.
  // ---------------------------------------------------------------------------

  auto Luma01 = [&](const PpmImage& img, int x, int y) -> double {
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
    const double r = static_cast<double>(img.rgb[i + 0u]) / 255.0;
    const double g = static_cast<double>(img.rgb[i + 1u]) / 255.0;
    const double bch = static_cast<double>(img.rgb[i + 2u]) / 255.0;
    // ITU-R BT.601 luma coefficients.
    return 0.299 * r + 0.587 * g + 0.114 * bch;
  };

  auto GlobalSsim = [&]() -> double {
    const double c1 = 0.01 * 0.01;
    const double c2 = 0.03 * 0.03;

    const double n = static_cast<double>(outStats.pixelsCompared);
    if (n <= 0.0) return 1.0;

    double sumA = 0.0;
    double sumB = 0.0;
    double sumAA = 0.0;
    double sumBB = 0.0;
    double sumAB = 0.0;

    for (int yy = 0; yy < h; ++yy) {
      for (int xx = 0; xx < w; ++xx) {
        const double la = Luma01(a, xx, yy);
        const double lb = Luma01(b, xx, yy);
        sumA += la;
        sumB += lb;
        sumAA += la * la;
        sumBB += lb * lb;
        sumAB += la * lb;
      }
    }

    const double muA = sumA / n;
    const double muB = sumB / n;
    double varA = sumAA / n - muA * muA;
    double varB = sumBB / n - muB * muB;
    double cov = sumAB / n - muA * muB;

    if (varA < 0.0) varA = 0.0;
    if (varB < 0.0) varB = 0.0;

    const double num = (2.0 * muA * muB + c1) * (2.0 * cov + c2);
    const double den = (muA * muA + muB * muB + c1) * (varA + varB + c2);
    if (den == 0.0) return 1.0;
    const double s = num / den;
    return std::clamp(s, -1.0, 1.0);
  };

  // Sanitize window: min 3, odd.
  int win = ssimWindow;
  if (win < 3) win = 3;
  if ((win % 2) == 0) win += 1;

  if (w < win || h < win) {
    outStats.ssim = GlobalSsim();
    return true;
  }

  const int r = win / 2;
  const int interiorW = w - 2 * r;
  const int interiorH = h - 2 * r;
  const std::uint64_t count = (interiorW > 0 && interiorH > 0)
                                 ? static_cast<std::uint64_t>(interiorW) * static_cast<std::uint64_t>(interiorH)
                                 : 0u;
  if (count == 0u) {
    outStats.ssim = GlobalSsim();
    return true;
  }

  const double c1 = 0.01 * 0.01;
  const double c2 = 0.03 * 0.03;
  const double area = static_cast<double>(win) * static_cast<double>(win);

  // Vertical running sums (one per x) over the last `win` rows of horizontal sums.
  std::vector<double> vA(static_cast<std::size_t>(w), 0.0);
  std::vector<double> vB(static_cast<std::size_t>(w), 0.0);
  std::vector<double> vAA(static_cast<std::size_t>(w), 0.0);
  std::vector<double> vBB(static_cast<std::size_t>(w), 0.0);
  std::vector<double> vAB(static_cast<std::size_t>(w), 0.0);

  // Ring buffer storing the last `win` horizontal-sum rows so we can subtract the leaving row.
  const std::size_t rowStride = static_cast<std::size_t>(w);
  std::vector<double> ringA(static_cast<std::size_t>(win) * rowStride, 0.0);
  std::vector<double> ringB(static_cast<std::size_t>(win) * rowStride, 0.0);
  std::vector<double> ringAA(static_cast<std::size_t>(win) * rowStride, 0.0);
  std::vector<double> ringBB(static_cast<std::size_t>(win) * rowStride, 0.0);
  std::vector<double> ringAB(static_cast<std::size_t>(win) * rowStride, 0.0);

  double sumSsim = 0.0;

  for (int yy = 0; yy < h; ++yy) {
    const int slot = yy % win;
    const std::size_t base = static_cast<std::size_t>(slot) * rowStride;

    double* hA = ringA.data() + base;
    double* hB = ringB.data() + base;
    double* hAA = ringAA.data() + base;
    double* hBB = ringBB.data() + base;
    double* hAB = ringAB.data() + base;

    // Remove the leaving row from the vertical sums once the window is full.
    if (yy >= win) {
      for (int xx = 0; xx < w; ++xx) {
        const std::size_t i = static_cast<std::size_t>(xx);
        vA[i] -= hA[i];
        vB[i] -= hB[i];
        vAA[i] -= hAA[i];
        vBB[i] -= hBB[i];
        vAB[i] -= hAB[i];
      }
    }

    // Build per-row prefix sums so we can compute horizontal box sums quickly.
    std::vector<double> pA(static_cast<std::size_t>(w) + 1u, 0.0);
    std::vector<double> pB(static_cast<std::size_t>(w) + 1u, 0.0);
    std::vector<double> pAA(static_cast<std::size_t>(w) + 1u, 0.0);
    std::vector<double> pBB(static_cast<std::size_t>(w) + 1u, 0.0);
    std::vector<double> pAB(static_cast<std::size_t>(w) + 1u, 0.0);

    for (int xx = 0; xx < w; ++xx) {
      const double la = Luma01(a, xx, yy);
      const double lb = Luma01(b, xx, yy);
      const std::size_t pi = static_cast<std::size_t>(xx) + 1u;
      pA[pi] = pA[pi - 1u] + la;
      pB[pi] = pB[pi - 1u] + lb;
      pAA[pi] = pAA[pi - 1u] + la * la;
      pBB[pi] = pBB[pi - 1u] + lb * lb;
      pAB[pi] = pAB[pi - 1u] + la * lb;
    }

    // Compute horizontal sums for interior x only; zero elsewhere.
    for (int xx = 0; xx < w; ++xx) {
      hA[xx] = 0.0;
      hB[xx] = 0.0;
      hAA[xx] = 0.0;
      hBB[xx] = 0.0;
      hAB[xx] = 0.0;
    }

    for (int xx = r; xx < w - r; ++xx) {
      const int x0 = xx - r;
      const int x1 = xx + r + 1;
      const std::size_t s0 = static_cast<std::size_t>(x0);
      const std::size_t s1 = static_cast<std::size_t>(x1);

      hA[xx] = pA[s1] - pA[s0];
      hB[xx] = pB[s1] - pB[s0];
      hAA[xx] = pAA[s1] - pAA[s0];
      hBB[xx] = pBB[s1] - pBB[s0];
      hAB[xx] = pAB[s1] - pAB[s0];
    }

    // Add the new row into the vertical sums.
    for (int xx = 0; xx < w; ++xx) {
      const std::size_t i = static_cast<std::size_t>(xx);
      vA[i] += hA[i];
      vB[i] += hB[i];
      vAA[i] += hAA[i];
      vBB[i] += hBB[i];
      vAB[i] += hAB[i];
    }

    // Once we have a full `win` rows, compute SSIM for the center row.
    if (yy >= win - 1) {
      for (int xx = r; xx < w - r; ++xx) {
        const std::size_t i = static_cast<std::size_t>(xx);

        const double muA = vA[i] / area;
        const double muB = vB[i] / area;

        double varA = vAA[i] / area - muA * muA;
        double varB = vBB[i] / area - muB * muB;
        double cov = vAB[i] / area - muA * muB;

        if (varA < 0.0) varA = 0.0;
        if (varB < 0.0) varB = 0.0;

        const double num = (2.0 * muA * muB + c1) * (2.0 * cov + c2);
        const double den = (muA * muA + muB * muB + c1) * (varA + varB + c2);
        const double s = (den == 0.0) ? 1.0 : (num / den);
        sumSsim += std::clamp(s, -1.0, 1.0);
      }
    }
  }

  outStats.ssim = sumSsim / static_cast<double>(count);

  return true;
}



bool WriteTilesCsv(const World& world, const std::string& path, std::string& outError)
{
  outError.clear();

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    outError = "Invalid world dimensions";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for writing";
    return false;
  }

  f << "x,y,terrain,overlay,level,district,height,variation,occupants\n";
  f << std::fixed << std::setprecision(6);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      f << x << ',' << y << ',' << ToString(t.terrain) << ',' << ToString(t.overlay) << ','
        << static_cast<int>(t.level) << ',' << static_cast<int>(t.district) << ','
        << static_cast<double>(t.height) << ','
        << static_cast<int>(t.variation) << ','
        << static_cast<int>(t.occupants) << '\n';
    }
  }

  if (!f) {
    outError = "Failed while writing file";
    return false;
  }
  return true;
}

bool WriteTileMetricsCsv(const World& world, const std::string& path, std::string& outError,
                         const TileMetricsCsvInputs& inputs, const TileMetricsCsvOptions& opt)
{
  outError.clear();

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    outError = "Invalid world dimensions";
    return false;
  }

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  auto validateGridSize = [&](const char* name, int bw, int bh, std::size_t sz) -> bool {
    if (bw != w || bh != h) {
      outError = std::string(name) + " dimensions do not match world";
      return false;
    }
    if (sz != n) {
      outError = std::string(name) + " size does not match world";
      return false;
    }
    return true;
  };

  if (inputs.landValue && opt.includeLandValue) {
    if (!validateGridSize("LandValueResult", inputs.landValue->w, inputs.landValue->h, inputs.landValue->value.size())) {
      return false;
    }
    if (opt.includeLandValueComponents) {
      if (inputs.landValue->parkAmenity.size() != n || inputs.landValue->waterAmenity.size() != n ||
          inputs.landValue->pollution.size() != n || inputs.landValue->traffic.size() != n) {
        outError = "LandValueResult component arrays do not match world";
        return false;
      }
    }
  }

  if (inputs.traffic && opt.includeTraffic) {
    if (inputs.traffic->roadTraffic.size() != n) {
      outError = "TrafficResult size does not match world";
      return false;
    }
  }

  if (inputs.goods && opt.includeGoods) {
    if (inputs.goods->roadGoodsTraffic.size() != n || inputs.goods->commercialFill.size() != n) {
      outError = "GoodsResult size does not match world";
      return false;
    }
  }

  if (inputs.noise && opt.includeNoise) {
    if (!validateGridSize("NoiseResult", inputs.noise->w, inputs.noise->h, inputs.noise->noise01.size())) {
      return false;
    }
  }

  if (inputs.landUseMix && opt.includeLandUseMix) {
    if (!validateGridSize("LandUseMixResult", inputs.landUseMix->w, inputs.landUseMix->h, inputs.landUseMix->mix01.size())) {
      return false;
    }
    if (!inputs.landUseMix->density01.empty() && inputs.landUseMix->density01.size() != n) {
      outError = "LandUseMixResult density array does not match world";
      return false;
    }
  }

  if (inputs.heatIsland && opt.includeHeatIsland) {
    if (!validateGridSize("HeatIslandResult", inputs.heatIsland->w, inputs.heatIsland->h, inputs.heatIsland->heat01.size())) {
      return false;
    }
    if (inputs.heatIsland->heat.size() != n) {
      outError = "HeatIslandResult heat array does not match world";
      return false;
    }
  }

  
  if (inputs.airPollution && opt.includeAirPollution) {
    if (!validateGridSize("AirPollutionResult", inputs.airPollution->w, inputs.airPollution->h,
                          inputs.airPollution->pollution01.size())) {
      return false;
    }
    if (inputs.airPollution->emission01.size() != n) {
      outError = "AirPollutionResult emission01 size does not match world";
      return false;
    }
  }

  if (inputs.runoff && opt.includeRunoffPollution) {
    if (!validateGridSize("RunoffPollutionResult", inputs.runoff->w, inputs.runoff->h,
                          inputs.runoff->pollution01.size())) {
      return false;
    }
    if (inputs.runoff->localLoad01.size() != n) {
      outError = "RunoffPollutionResult localLoad01 size does not match world";
      return false;
    }
    if (!inputs.runoff->flowAccum.empty() && inputs.runoff->flowAccum.size() != n) {
      outError = "RunoffPollutionResult flowAccum size does not match world";
      return false;
    }
  }

  if (inputs.runoffMitigation && opt.includeRunoffMitigation) {
    if (!validateGridSize("RunoffMitigationResult", inputs.runoffMitigation->w, inputs.runoffMitigation->h,
                          inputs.runoffMitigation->priority01.size())) {
      return false;
    }
    if (inputs.runoffMitigation->priorityRaw.size() != n || inputs.runoffMitigation->planMask.size() != n) {
      outError = "RunoffMitigationResult arrays do not match world";
      return false;
    }
  }

  if (inputs.solar && opt.includeSolar) {
    if (!validateGridSize("SolarPotentialResult", inputs.solar->w, inputs.solar->h,
                          inputs.solar->exposure01.size())) {
      return false;
    }
    if (inputs.solar->roofArea01.size() != n || inputs.solar->potential01.size() != n) {
      outError = "SolarPotentialResult arrays do not match world";
      return false;
    }
  }

  if (inputs.skyView && opt.includeSkyView) {
    if (!validateGridSize("SkyViewResult", inputs.skyView->w, inputs.skyView->h,
                          inputs.skyView->skyView01.size())) {
      return false;
    }
    if (inputs.skyView->canyon01.size() != n) {
      outError = "SkyViewResult canyon01 size does not match world";
      return false;
    }
  }

  if (inputs.energy && opt.includeEnergy) {
    if (!validateGridSize("EnergyModelResult", inputs.energy->w, inputs.energy->h,
                          inputs.energy->demand01.size())) {
      return false;
    }
    if (inputs.energy->solar01.size() != n || inputs.energy->balance01.size() != n ||
        inputs.energy->demandRaw.size() != n || inputs.energy->solarRaw.size() != n ||
        inputs.energy->netRaw.size() != n) {
      outError = "EnergyModelResult arrays do not match world";
      return false;
    }
  }



  if (inputs.carbon && opt.includeCarbon) {
    if (!validateGridSize("CarbonModelResult", inputs.carbon->w, inputs.carbon->h,
                          inputs.carbon->emission01.size())) {
      return false;
    }
    if (inputs.carbon->sequestration01.size() != n || inputs.carbon->balance01.size() != n ||
        inputs.carbon->emissionRaw.size() != n || inputs.carbon->sequestrationRaw.size() != n ||
        inputs.carbon->netRaw.size() != n) {
      outError = "CarbonModelResult arrays do not match world";
      return false;
    }
  }

  if (inputs.crime && opt.includeCrime) {
    if (!validateGridSize("CrimeModelResult", inputs.crime->w, inputs.crime->h,
                          inputs.crime->risk01.size())) {
      return false;
    }
    if (inputs.crime->policeAccess01.size() != n || inputs.crime->policeCostMilli.size() != n) {
      outError = "CrimeModelResult arrays do not match world";
      return false;
    }
  }

  if (inputs.trafficSafety && opt.includeTrafficSafety) {
    if (!validateGridSize("TrafficSafetyResult", inputs.trafficSafety->w, inputs.trafficSafety->h,
                          inputs.trafficSafety->risk01.size())) {
      return false;
    }
    if (inputs.trafficSafety->exposure01.size() != n || inputs.trafficSafety->priority01.size() != n) {
      outError = "TrafficSafetyResult arrays do not match world";
      return false;
    }
  }

  if (inputs.transit && opt.includeTransit) {
    if (!validateGridSize("TransitAccessibilityResult", inputs.transit->w, inputs.transit->h,
                          inputs.transit->access01.size())) {
      return false;
    }
    if (inputs.transit->modeSharePotential01.size() != n || inputs.transit->stopMask.size() != n ||
        inputs.transit->corridorMask.size() != n) {
      outError = "TransitAccessibilityResult arrays do not match world";
      return false;
    }
    if (opt.includeTransitSteps && inputs.transit->stepsToStop.size() != n) {
      outError = "TransitAccessibilityResult stepsToStop does not match world";
      return false;
    }
  }

  if (inputs.fireRisk && opt.includeFireRisk) {
    if (!validateGridSize("FireRiskResult", inputs.fireRisk->w, inputs.fireRisk->h, inputs.fireRisk->risk01.size())) {
      return false;
    }
    if (inputs.fireRisk->coverage01.size() != n || inputs.fireRisk->responseCostMilli.size() != n) {
      outError = "FireRiskResult arrays do not match world";
      return false;
    }
  }

  if (inputs.walkability && opt.includeWalkability) {
    if (!validateGridSize("WalkabilityResult", inputs.walkability->w, inputs.walkability->h,
                          inputs.walkability->overall01.size())) {
      return false;
    }
    if (opt.includeWalkabilityComponents) {
      if (inputs.walkability->park01.size() != n || inputs.walkability->retail01.size() != n ||
          inputs.walkability->education01.size() != n || inputs.walkability->health01.size() != n ||
          inputs.walkability->safety01.size() != n || inputs.walkability->coverageMask.size() != n) {
        outError = "WalkabilityResult component arrays do not match world";
        return false;
      }
      if (opt.includeWalkabilityDistances) {
        if (inputs.walkability->costParkMilli.size() != n || inputs.walkability->costRetailMilli.size() != n ||
            inputs.walkability->costEducationMilli.size() != n || inputs.walkability->costHealthMilli.size() != n ||
            inputs.walkability->costSafetyMilli.size() != n) {
          outError = "WalkabilityResult cost arrays do not match world";
          return false;
        }
      }
    }
  }

  if (inputs.jobs && opt.includeJobs) {
    if (!validateGridSize("JobOpportunityResult", inputs.jobs->w, inputs.jobs->h, inputs.jobs->jobAccess01.size())) {
      return false;
    }
    if (inputs.jobs->jobOpportunity01.size() != n || inputs.jobs->jobAccessCostMilli.size() != n) {
      outError = "JobOpportunityResult arrays do not match world";
      return false;
    }
  }

  if (inputs.roadHealth && opt.includeRoadHealth) {
    if (!validateGridSize("RoadHealthResult", inputs.roadHealth->w, inputs.roadHealth->h,
                          inputs.roadHealth->centrality01.size())) {
      return false;
    }
    if (inputs.roadHealth->vulnerability01.size() != n) {
      outError = "RoadHealthResult vulnerability01 size does not match world";
      return false;
    }
    if (!inputs.roadHealth->bypassMask.empty() && inputs.roadHealth->bypassMask.size() != n) {
      outError = "RoadHealthResult bypassMask size does not match world";
      return false;
    }
  }

  if (inputs.livability && opt.includeLivability) {
    if (!validateGridSize("LivabilityResult", inputs.livability->w, inputs.livability->h,
                          inputs.livability->livability01.size())) {
      return false;
    }
    if (inputs.livability->priority01.size() != n) {
      outError = "LivabilityResult priority01 size does not match world";
      return false;
    }
  }

  if (inputs.livabilityHotspot && opt.includeHotspots) {
    if (!validateGridSize("HotspotResult(livability)", inputs.livabilityHotspot->w, inputs.livabilityHotspot->h,
                          inputs.livabilityHotspot->z01.size())) {
      return false;
    }
    if (inputs.livabilityHotspot->z.size() != n || inputs.livabilityHotspot->cls.size() != n) {
      outError = "HotspotResult(livability) arrays do not match world";
      return false;
    }
  }

  if (inputs.interventionHotspot && opt.includeHotspots) {
    if (!validateGridSize("HotspotResult(intervention)", inputs.interventionHotspot->w, inputs.interventionHotspot->h,
                          inputs.interventionHotspot->z01.size())) {
      return false;
    }
    if (inputs.interventionHotspot->z.size() != n || inputs.interventionHotspot->cls.size() != n) {
      outError = "HotspotResult(intervention) arrays do not match world";
      return false;
    }
  }

  if (inputs.seaFlood && opt.includeFlood) {
    if (!validateGridSize("SeaFloodResult", inputs.seaFlood->w, inputs.seaFlood->h, inputs.seaFlood->depth.size())) {
      return false;
    }
    if (inputs.seaFlood->flooded.size() != n) {
      outError = "SeaFloodResult flooded mask does not match world";
      return false;
    }
  }

  if (inputs.ponding && opt.includePonding) {
    if (!validateGridSize("DepressionFillResult", inputs.ponding->w, inputs.ponding->h, inputs.ponding->depth.size())) {
      return false;
    }
  }

  ZoneAccessMap zoneAccess{};
  bool haveZoneAccess = false;
  if (opt.includeRciPressure) {
    zoneAccess = BuildZoneAccessMap(world, nullptr);
    haveZoneAccess = true;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for writing";
    return false;
  }

  f << "x,y,terrain,overlay,level,district,height,variation,occupants";
  if (opt.includeLandValue && inputs.landValue) {
    f << ",land_value";
    if (opt.includeLandValueComponents) {
      f << ",park_amenity,water_amenity,pollution,traffic_penalty";
    }
  }
  if (opt.includeTraffic && inputs.traffic) {
    f << ",commute_traffic";
  }
  if (opt.includeGoods && inputs.goods) {
    f << ",goods_traffic,goods_fill";
  }
  if (opt.includeNoise && inputs.noise) {
    f << ",noise";
  }
  if (opt.includeLandUseMix && inputs.landUseMix) {
    f << ",landuse_mix,landuse_density";
  }
  if (opt.includeHeatIsland && inputs.heatIsland) {
    f << ",heat_island,heat_island_raw";
  }
  
  if (opt.includeAirPollution && inputs.airPollution) {
    f << ",air_pollution,air_emission";
  }
  if (opt.includeRunoffPollution && inputs.runoff) {
    f << ",runoff_pollution,runoff_load,runoff_flow_accum";
  }
  if (opt.includeRunoffMitigation && inputs.runoffMitigation) {
    f << ",runoff_mitigation_priority,runoff_mitigation_plan";
  }

  if (opt.includeSolar && inputs.solar) {
    f << ",solar_exposure,solar_roof_area,solar_potential";
  }
  if (opt.includeSkyView && inputs.skyView) {
    f << ",sky_view,canyon_confinement";
  }
  if (opt.includeEnergy && inputs.energy) {
    f << ",energy_demand,energy_solar,energy_balance,energy_demand_raw,energy_solar_raw,energy_net_raw";
  }


  if (opt.includeCarbon && inputs.carbon) {
    f << ",carbon_emission,carbon_sequestration,carbon_balance,carbon_emission_raw,carbon_sequestration_raw,carbon_net_raw";
  }
  if (opt.includeCrime && inputs.crime) {
    f << ",crime_risk,police_access,police_cost";
  }
  if (opt.includeTrafficSafety && inputs.trafficSafety) {
    f << ",traffic_crash_risk,traffic_crash_exposure,traffic_crash_priority";
  }
  if (opt.includeTransit && inputs.transit) {
    f << ",transit_access,transit_mode_share_potential,transit_is_stop,transit_on_corridor";
    if (opt.includeTransitSteps) {
      f << ",transit_stop_steps";
    }
  }

  if (opt.includeFireRisk && inputs.fireRisk) {
    f << ",fire_risk,fire_coverage,fire_response_cost";
  }
  if (opt.includeWalkability && inputs.walkability) {
    f << ",walkability";
    if (opt.includeWalkabilityComponents) {
      f << ",walkability_park,walkability_retail,walkability_education,walkability_health,walkability_safety";
      f << ",walk_cover_mask,walk_cover_count";
      if (opt.includeWalkabilityDistances) {
        f << ",walk_dist_park,walk_dist_retail,walk_dist_education,walk_dist_health,walk_dist_safety";
      }
    }
  }
  if (opt.includeJobs && inputs.jobs) {
    f << ",job_access,job_opportunity,job_access_cost";
  }
  if (opt.includeRoadHealth && inputs.roadHealth) {
    f << ",road_centrality,road_vulnerability,road_bypass";
  }
  if (opt.includeLivability && inputs.livability) {
    f << ",livability,intervention_priority";
  }
  if (opt.includeHotspots && inputs.livabilityHotspot) {
    f << ",livability_hotspot,livability_hotspot_z,livability_hotspot_class";
  }
  if (opt.includeHotspots && inputs.interventionHotspot) {
    f << ",intervention_hotspot,intervention_hotspot_z,intervention_hotspot_class";
  }
  if (opt.includeRciPressure) {
    f << ",zone_pressure_residential,zone_pressure_commercial,zone_pressure_industrial";
  }
  if (opt.includeFlood && inputs.seaFlood) {
    f << ",flooded,flood_depth";
  }
  if (opt.includePonding && inputs.ponding) {
    f << ",ponding_depth";
  }
  f << "\n";

  const int prec = std::clamp(opt.floatPrecision, 0, 12);
  f << std::fixed << std::setprecision(prec);

  auto popcountU8 = [](std::uint8_t v) -> int {
    int c = 0;
    while (v) {
      c += (v & 1u) ? 1 : 0;
      v = static_cast<std::uint8_t>(v >> 1);
    }
    return c;
  };

  auto costToSteps = [](int costMilli) -> int {
    if (costMilli < 0) return -1;
    return costMilli / 1000;
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      f << x << ',' << y << ',' << ToString(t.terrain) << ',' << ToString(t.overlay) << ','
        << static_cast<int>(t.level) << ',' << static_cast<int>(t.district) << ','
        << static_cast<double>(t.height) << ','
        << static_cast<int>(t.variation) << ','
        << static_cast<int>(t.occupants);

      if (opt.includeLandValue && inputs.landValue) {
        f << ',' << static_cast<double>(inputs.landValue->value[i]);
        if (opt.includeLandValueComponents) {
          f << ',' << static_cast<double>(inputs.landValue->parkAmenity[i]);
          f << ',' << static_cast<double>(inputs.landValue->waterAmenity[i]);
          f << ',' << static_cast<double>(inputs.landValue->pollution[i]);
          f << ',' << static_cast<double>(inputs.landValue->traffic[i]);
        }
      }
      if (opt.includeTraffic && inputs.traffic) {
        f << ',' << static_cast<int>(inputs.traffic->roadTraffic[i]);
      }
      if (opt.includeGoods && inputs.goods) {
        f << ',' << static_cast<int>(inputs.goods->roadGoodsTraffic[i]);
        f << ',' << static_cast<int>(inputs.goods->commercialFill[i]);
      }
      if (opt.includeNoise && inputs.noise) {
        f << ',' << static_cast<double>(inputs.noise->noise01[i]);
      }
      if (opt.includeLandUseMix && inputs.landUseMix) {
        const float dens = (!inputs.landUseMix->density01.empty()) ? inputs.landUseMix->density01[i] : 0.0f;
        f << ',' << static_cast<double>(inputs.landUseMix->mix01[i]);
        f << ',' << static_cast<double>(dens);
      }
      if (opt.includeHeatIsland && inputs.heatIsland) {
        f << ',' << static_cast<double>(inputs.heatIsland->heat01[i]);
        f << ',' << static_cast<double>(inputs.heatIsland->heat[i]);
      }
      
      if (opt.includeAirPollution && inputs.airPollution) {
        f << ',' << static_cast<double>(inputs.airPollution->pollution01[i]);
        f << ',' << static_cast<double>(inputs.airPollution->emission01[i]);
      }
      if (opt.includeRunoffPollution && inputs.runoff) {
        f << ',' << static_cast<double>(inputs.runoff->pollution01[i]);
        f << ',' << static_cast<double>(inputs.runoff->localLoad01[i]);
        int acc = 0;
        if (inputs.runoff->flowAccum.size() == n) {
          acc = inputs.runoff->flowAccum[i];
        }
        f << ',' << acc;
      }
      if (opt.includeRunoffMitigation && inputs.runoffMitigation) {
        f << ',' << static_cast<double>(inputs.runoffMitigation->priority01[i]);
        f << ',' << static_cast<int>(inputs.runoffMitigation->planMask[i]);
      }
      if (opt.includeSolar && inputs.solar) {
        f << ',' << static_cast<double>(inputs.solar->exposure01[i]);
        f << ',' << static_cast<double>(inputs.solar->roofArea01[i]);
        f << ',' << static_cast<double>(inputs.solar->potential01[i]);
      }
      if (opt.includeSkyView && inputs.skyView) {
        f << ',' << static_cast<double>(inputs.skyView->skyView01[i]);
        f << ',' << static_cast<double>(inputs.skyView->canyon01[i]);
      }
      if (opt.includeEnergy && inputs.energy) {
        f << ',' << static_cast<double>(inputs.energy->demand01[i]);
        f << ',' << static_cast<double>(inputs.energy->solar01[i]);
        f << ',' << static_cast<double>(inputs.energy->balance01[i]);
        f << ',' << static_cast<double>(inputs.energy->demandRaw[i]);
        f << ',' << static_cast<double>(inputs.energy->solarRaw[i]);
        f << ',' << static_cast<double>(inputs.energy->netRaw[i]);
      }


      if (opt.includeCarbon && inputs.carbon) {
        f << ',' << static_cast<double>(inputs.carbon->emission01[i]);
        f << ',' << static_cast<double>(inputs.carbon->sequestration01[i]);
        f << ',' << static_cast<double>(inputs.carbon->balance01[i]);
        f << ',' << static_cast<double>(inputs.carbon->emissionRaw[i]);
        f << ',' << static_cast<double>(inputs.carbon->sequestrationRaw[i]);
        f << ',' << static_cast<double>(inputs.carbon->netRaw[i]);
      }

      if (opt.includeCrime && inputs.crime) {
        f << ',' << static_cast<double>(inputs.crime->risk01[i]);
        f << ',' << static_cast<double>(inputs.crime->policeAccess01[i]);
        f << ',' << static_cast<int>(inputs.crime->policeCostMilli[i]);
      }

      if (opt.includeTrafficSafety && inputs.trafficSafety) {
        f << ',' << static_cast<double>(inputs.trafficSafety->risk01[i]);
        f << ',' << static_cast<double>(inputs.trafficSafety->exposure01[i]);
        f << ',' << static_cast<double>(inputs.trafficSafety->priority01[i]);
      }

      if (opt.includeTransit && inputs.transit) {
        f << ',' << static_cast<double>(inputs.transit->access01[i]);
        f << ',' << static_cast<double>(inputs.transit->modeSharePotential01[i]);
        f << ',' << static_cast<int>(inputs.transit->stopMask[i]);
        f << ',' << static_cast<int>(inputs.transit->corridorMask[i]);
        if (opt.includeTransitSteps) {
          f << ',' << static_cast<int>(inputs.transit->stepsToStop[i]);
        }
      }

      if (opt.includeFireRisk && inputs.fireRisk) {
        f << ',' << static_cast<double>(inputs.fireRisk->risk01[i]);
        f << ',' << static_cast<double>(inputs.fireRisk->coverage01[i]);
        f << ',' << static_cast<int>(inputs.fireRisk->responseCostMilli[i]);
      }
      if (opt.includeWalkability && inputs.walkability) {
        f << ',' << static_cast<double>(inputs.walkability->overall01[i]);
        if (opt.includeWalkabilityComponents) {
          f << ',' << static_cast<double>(inputs.walkability->park01[i]);
          f << ',' << static_cast<double>(inputs.walkability->retail01[i]);
          f << ',' << static_cast<double>(inputs.walkability->education01[i]);
          f << ',' << static_cast<double>(inputs.walkability->health01[i]);
          f << ',' << static_cast<double>(inputs.walkability->safety01[i]);

          const std::uint8_t m = inputs.walkability->coverageMask[i];
          f << ',' << static_cast<int>(m);
          f << ',' << static_cast<int>(popcountU8(m));

          if (opt.includeWalkabilityDistances) {
            f << ',' << static_cast<int>(costToSteps(inputs.walkability->costParkMilli[i]));
            f << ',' << static_cast<int>(costToSteps(inputs.walkability->costRetailMilli[i]));
            f << ',' << static_cast<int>(costToSteps(inputs.walkability->costEducationMilli[i]));
            f << ',' << static_cast<int>(costToSteps(inputs.walkability->costHealthMilli[i]));
            f << ',' << static_cast<int>(costToSteps(inputs.walkability->costSafetyMilli[i]));
          }
        }
      }
      if (opt.includeJobs && inputs.jobs) {
        f << ',' << static_cast<double>(inputs.jobs->jobAccess01[i]);
        f << ',' << static_cast<double>(inputs.jobs->jobOpportunity01[i]);
        f << ',' << static_cast<int>(costToSteps(inputs.jobs->jobAccessCostMilli[i]));
      }
      if (opt.includeRoadHealth && inputs.roadHealth) {
        f << ',' << static_cast<double>(inputs.roadHealth->centrality01[i]);
        f << ',' << static_cast<double>(inputs.roadHealth->vulnerability01[i]);
        int bypass = 0;
        if (inputs.roadHealth->bypassMask.size() == n) {
          bypass = static_cast<int>(inputs.roadHealth->bypassMask[i]);
        }
        f << ',' << bypass;
      }
      if (opt.includeLivability && inputs.livability) {
        f << ',' << static_cast<double>(inputs.livability->livability01[i]);
        f << ',' << static_cast<double>(inputs.livability->priority01[i]);
      }
      if (opt.includeHotspots && inputs.livabilityHotspot) {
        f << ',' << static_cast<double>(inputs.livabilityHotspot->z01[i]);
        f << ',' << static_cast<double>(inputs.livabilityHotspot->z[i]);
        f << ',' << static_cast<int>(inputs.livabilityHotspot->cls[i]);
      }
      if (opt.includeHotspots && inputs.interventionHotspot) {
        f << ',' << static_cast<double>(inputs.interventionHotspot->z01[i]);
        f << ',' << static_cast<double>(inputs.interventionHotspot->z[i]);
        f << ',' << static_cast<int>(inputs.interventionHotspot->cls[i]);
      }

      if (opt.includeRciPressure) {
        float lvVal = 0.5f;
        if (inputs.landValue && inputs.landValue->value.size() == n) {
          lvVal = inputs.landValue->value[i];
        }

        auto pressureFor = [&](Overlay target, float demand, float lvWeight) -> float {
          if (t.terrain == Terrain::Water) return 0.0f;
          if (!(t.overlay == Overlay::None || t.overlay == target)) return 0.0f;

          bool access = false;
          if (t.overlay == Overlay::None) {
            access = world.hasAdjacentRoad(x, y);
          } else {
            access = haveZoneAccess ? HasZoneAccess(zoneAccess, x, y) : world.hasAdjacentRoad(x, y);
          }
          if (!access) return 0.0f;

          float desir = 1.0f;
          if (target == Overlay::Industrial) {
            desir = 1.0f + lvWeight * (0.50f - lvVal);
          } else {
            desir = 1.0f + lvWeight * (lvVal - 0.50f);
          }
          desir = std::clamp(desir, 0.40f, 1.60f);
          return Clamp01(std::clamp(demand, 0.0f, 1.0f) * desir);
        };

        const float pR = pressureFor(Overlay::Residential, world.stats().demandResidential, 0.70f);
        const float pC = pressureFor(Overlay::Commercial, world.stats().demandCommercial, 0.80f);
        const float pI = pressureFor(Overlay::Industrial, world.stats().demandIndustrial, 0.80f);
        f << ',' << static_cast<double>(pR) << ',' << static_cast<double>(pC) << ',' << static_cast<double>(pI);
      }
      if (opt.includeFlood && inputs.seaFlood) {
        f << ',' << static_cast<int>(inputs.seaFlood->flooded[i]);
        f << ',' << static_cast<double>(inputs.seaFlood->depth[i]);
      }
      if (opt.includePonding && inputs.ponding) {
        f << ',' << static_cast<double>(inputs.ponding->depth[i]);
      }
      f << '\n';
    }
  }

  if (!f) {
    outError = "Failed while writing file";
    return false;
  }
  return true;
}

} // namespace isocity
