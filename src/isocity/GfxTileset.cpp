#include "isocity/GfxTileset.hpp"

#include "isocity/Random.hpp"
#include "isocity/DeterministicMath.hpp"
#include "isocity/GfxBuildings.hpp"
#include "isocity/GfxFacilities.hpp"
#include "isocity/GfxProps.hpp"
#include "isocity/GfxPacker.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

struct DiamondParams {
  float nx = 0.0f;
  float ny = 0.0f;
  float manhattan = 0.0f;
  float edge = 0.0f; // 0 at edge, 1 at center
};

DiamondParams DiamondAtPixel(int x, int y, int w, int h)
{
  const float cx = (static_cast<float>(w) - 1.0f) * 0.5f;
  const float cy = (static_cast<float>(h) - 1.0f) * 0.5f;
  const float halfW = static_cast<float>(w) * 0.5f;
  const float halfH = static_cast<float>(h) * 0.5f;

  const float nx = (static_cast<float>(x) - cx) / halfW;
  const float ny = (static_cast<float>(y) - cy) / halfH;
  const float man = std::fabs(nx) + std::fabs(ny);
  const float edge = std::clamp(1.0f - man, 0.0f, 1.0f);

  return DiamondParams{nx, ny, man, edge};
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

// Generic diamond image generator (RGBA) with a per-pixel callback.
template <typename Fn>
RgbaImage MakeDiamondImage(int w, int h, Fn fn)
{
  RgbaImage img;
  img.width = w;
  img.height = h;
  img.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const DiamondParams d = DiamondAtPixel(x, y, w, h);
      if (d.manhattan > 1.0f) continue;

      const Rgba8 c = fn(x, y, d);
      if (c.a == 0) continue;

      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      img.rgba[i + 0] = c.r;
      img.rgba[i + 1] = c.g;
      img.rgba[i + 2] = c.b;
      img.rgba[i + 3] = c.a;
    }
  }

  return img;
}

inline int Popcount4(std::uint8_t v)
{
  v &= 0x0Fu;
  v = v - ((v >> 1) & 0x55u);
  v = (v & 0x33u) + ((v >> 2) & 0x33u);
  return static_cast<int>((v + (v >> 4)) & 0x0Fu);
}

inline float Dot2(float ax, float ay, float bx, float by) { return ax * bx + ay * by; }

float DistPointSegment(float px, float py, float ax, float ay, float bx, float by, float& outT)
{
  const float abx = bx - ax;
  const float aby = by - ay;
  const float apx = px - ax;
  const float apy = py - ay;

  const float ab2 = Dot2(abx, aby, abx, aby);
  float t = 0.0f;
  if (ab2 > 1.0e-6f) t = Dot2(apx, apy, abx, aby) / ab2;
  t = std::clamp(t, 0.0f, 1.0f);
  outT = t;

  const float cx = ax + abx * t;
  const float cy = ay + aby * t;
  const float dx = px - cx;
  const float dy = py - cy;
  return std::sqrt(dx * dx + dy * dy);
}

struct RoadStyle {
  float roadW = 0.130f;
  float lineThick = 0.010f;
  float lineGap = 0.018f;
  float laneOff = 0.05f;
  float shoulderOff = 0.10f;
  float edgeDark = 0.70f;
  float dashFreq = 10.0f;
  Rgba8 asphalt = Rgba8{90, 90, 95, 230};
  Rgba8 mark = Rgba8{220, 220, 210, 240};
  Rgba8 mark2 = Rgba8{250, 220, 110, 245}; // yellow-ish
  bool dashed = true;
  bool doubleCenter = false;
  bool highway = false;
  bool crosswalk = false;
};

RoadStyle RoadStyleForLevel(int level, const GfxPalette& pal)
{
  RoadStyle st{};
  level = std::clamp(level, 1, 3);
  if (level == 1) {
    st.roadW = 0.130f;
    st.asphalt = pal.roadAsphalt1;
    st.mark = pal.roadMarkWhite;
    st.mark2 = pal.roadMarkYellow;
    st.dashFreq = 10.0f;
    st.dashed = true;
    st.doubleCenter = false;
    st.highway = false;
    st.crosswalk = true;
    st.edgeDark = 0.78f;
  } else if (level == 2) {
    st.roadW = 0.175f;
    st.asphalt = pal.roadAsphalt2;
    st.mark = pal.roadMarkWhite;
    st.mark2 = pal.roadMarkYellow;
    st.dashed = false;
    st.doubleCenter = true;
    st.lineGap = 0.022f;
    st.lineThick = 0.008f;
    st.crosswalk = true;
    st.edgeDark = 0.74f;
  } else {
    st.roadW = 0.215f;
    st.asphalt = pal.roadAsphalt3;
    st.mark = pal.roadMarkWhite;
    st.mark2 = pal.roadMarkYellow;
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
  return st;
}

RgbaImage MakeRoadVariant(std::uint8_t mask, int level, int variant,
                          int tileW, int tileH, std::uint32_t seedv,
                          const GfxPalette& pal)
{
  const RoadStyle st = RoadStyleForLevel(level, pal);
  const float centerR = st.roadW * 1.10f;
  const int conn = Popcount4(mask);

  return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
    const std::uint32_t h = HashCoords32(x, y, seedv);
    const float n = (Frac01(h) - 0.5f) * 0.09f;

    const float px = d.nx;
    const float py = d.ny;

    float sd = std::sqrt(px * px + py * py) - centerR;

    float bestSegDist = 1.0e9f;
    float bestSegT = 0.0f;
    float bestEx = 0.0f;
    float bestEy = 0.0f;

    auto consider = [&](bool enabled, float ex, float ey) {
      if (!enabled) return;
      float tproj = 0.0f;
      const float dist = DistPointSegment(px, py, 0.0f, 0.0f, ex, ey, tproj);
      sd = std::min(sd, dist - st.roadW);
      if (dist < bestSegDist) {
        bestSegDist = dist;
        bestSegT = tproj;
        bestEx = ex;
        bestEy = ey;
      }
    };

    // Bit layout matches World::computeRoadMask().
    consider((mask & 0x01u) != 0u, 0.5f, -0.5f);  // up-right
    consider((mask & 0x02u) != 0u, 0.5f, 0.5f);   // down-right
    consider((mask & 0x04u) != 0u, -0.5f, 0.5f);  // down-left
    consider((mask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

    if (sd > 0.0f) return Rgba8{0, 0, 0, 0};

    Rgba8 base = st.asphalt;
    base = Mul(base, 1.0f + n);

    // Asphalt speckles / wear.
    if ((h & 0x7Fu) == 0x3Fu) base = Mul(base, 0.86f);
    if ((h & 0xFFu) == 0xA1u) base = Mul(base, 1.06f);

    // Darken very near the edge (gives a curb/shoulder feel).
    const float distToEdge = -sd;
    const float edgeW = std::max(0.004f, st.roadW * 0.22f);
    if (distToEdge < edgeW) {
      const float t = std::clamp(distToEdge / edgeW, 0.0f, 1.0f);
      const float mul = st.edgeDark + (1.0f - st.edgeDark) * t;
      base = Mul(base, mul);
    }

    // Markings based on closest segment.
    const float centerDist = std::sqrt(px * px + py * py);
    if (conn > 0 && bestSegDist < (st.roadW * 0.55f) && centerDist > centerR * 0.60f) {
      const float segLen = std::sqrt(bestEx * bestEx + bestEy * bestEy);
      if (segLen > 1.0e-6f) {
        const float vx = bestEx / segLen;
        const float vy = bestEy / segLen;
        const float cx = bestSegT * bestEx;
        const float cy = bestSegT * bestEy;
        const float dx = px - cx;
        const float dy = py - cy;
        const float signedPerp = dx * (-vy) + dy * (vx);
        const float absPerp = std::fabs(signedPerp);

        // Crosswalk stripes near intersections (only for streets/avenues).
        if (st.crosswalk && conn >= 3 && bestSegT > 0.12f && bestSegT < 0.28f && absPerp < st.roadW * 0.92f) {
          const float stripeW = 0.030f;
          const int stripe = static_cast<int>(std::floor((signedPerp + st.roadW) / stripeW +
                                                        static_cast<float>(mask) * 0.10f));
          if ((stripe & 1) == 0) {
            base = Lerp(base, Rgba8{250, 250, 250, 255}, 0.85f);
          }
        }

        // Level-specific lane markings.
        if (st.highway) {
          const float thick = st.lineThick;
          if (std::fabs(absPerp - st.shoulderOff) < (thick * 1.20f)) {
            base = st.mark;
          } else if (std::fabs(absPerp - st.laneOff) < thick) {
            const int dash = static_cast<int>(std::floor(bestSegT * st.dashFreq + static_cast<float>(mask) * 0.21f +
                                                         static_cast<float>(variant) * 0.37f));
            if ((dash & 1) == 0) base = st.mark;
          }
        } else if (st.doubleCenter) {
          if (std::fabs(absPerp - st.lineGap) < st.lineThick) base = st.mark2;
        } else {
          if (absPerp < st.lineThick) {
            const int dash = static_cast<int>(std::floor(bestSegT * st.dashFreq + static_cast<float>(mask) * 0.15f +
                                                         static_cast<float>(variant) * 0.23f));
            if ((dash & 1) == 0) base = st.mark;
          }
        }
      }
    }

    // Soft edges.
    const float edgeSoft = 0.05f;
    const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
    base.a = static_cast<std::uint8_t>(std::lround(static_cast<float>(base.a) * a));
    return base;
  });
}

RgbaImage MakeBridgeVariant(std::uint8_t mask, int level, int variant,
                            int tileW, int tileH, std::uint32_t seedv,
                            const GfxPalette& pal)
{
  RoadStyle st = RoadStyleForLevel(level, pal);
  const float centerR = st.roadW * 1.10f;

  Rgba8 deck = pal.bridgeDeck1;
  if (level == 2) deck = pal.bridgeDeck2;
  if (level == 3) deck = pal.bridgeDeck3;

  const int conn = Popcount4(mask);

  return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
    const std::uint32_t h = HashCoords32(x, y, seedv);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    const float px = d.nx;
    const float py = d.ny;

    float sd = std::sqrt(px * px + py * py) - centerR;

    float bestSegDist = 1.0e9f;
    float bestSegT = 0.0f;
    float bestEx = 0.0f;
    float bestEy = 0.0f;

    auto consider = [&](bool enabled, float ex, float ey) {
      if (!enabled) return;
      float tproj = 0.0f;
      const float dist = DistPointSegment(px, py, 0.0f, 0.0f, ex, ey, tproj);
      sd = std::min(sd, dist - st.roadW);
      if (dist < bestSegDist) {
        bestSegDist = dist;
        bestSegT = tproj;
        bestEx = ex;
        bestEy = ey;
      }
    };

    consider((mask & 0x01u) != 0u, 0.5f, -0.5f);
    consider((mask & 0x02u) != 0u, 0.5f, 0.5f);
    consider((mask & 0x04u) != 0u, -0.5f, 0.5f);
    consider((mask & 0x08u) != 0u, -0.5f, -0.5f);

    if (sd > 0.0f) return Rgba8{0, 0, 0, 0};

    Rgba8 base = deck;
    base = Mul(base, 1.0f + n);

    // Plank / joint pattern along the closest segment (avoid the intersection blob).
    const float centerDist = std::sqrt(px * px + py * py);
    if (conn > 0 && bestSegDist < (st.roadW * 0.70f) && centerDist > centerR * 0.55f) {
      const float freq = (level == 1) ? 18.0f : 22.0f;
      const int plank = static_cast<int>(std::floor(bestSegT * freq + static_cast<float>(mask) * 0.21f +
                                                    static_cast<float>(variant) * 0.19f));
      if ((plank & 1) == 0) base = Mul(base, 0.92f);
    }

    // Guard rails / curbs.
    if (-sd < 0.012f) base = Mul(base, (level == 3) ? 0.58f : 0.68f);

    // Lane markings (skip for level 1 wood bridges to keep them rustic).
    if (level >= 2 && conn > 0 && bestSegDist < (st.roadW * 0.55f) && centerDist > centerR * 0.60f) {
      const float segLen = std::sqrt(bestEx * bestEx + bestEy * bestEy);
      if (segLen > 1.0e-6f) {
        const float vx = bestEx / segLen;
        const float vy = bestEy / segLen;
        const float cx = bestSegT * bestEx;
        const float cy = bestSegT * bestEy;
        const float dx = px - cx;
        const float dy = py - cy;
        const float signedPerp = dx * (-vy) + dy * (vx);
        const float absPerp = std::fabs(signedPerp);

        if (st.highway) {
          const float thick = st.lineThick;
          if (std::fabs(absPerp - st.shoulderOff) < (thick * 1.20f)) {
            base = st.mark;
          } else if (std::fabs(absPerp - st.laneOff) < thick) {
            const int dash = static_cast<int>(std::floor(bestSegT * st.dashFreq + static_cast<float>(mask) * 0.21f +
                                                         static_cast<float>(variant) * 0.37f));
            if ((dash & 1) == 0) base = st.mark;
          }
        } else if (st.doubleCenter) {
          if (std::fabs(absPerp - st.lineGap) < st.lineThick) base = st.mark2;
        }
      }
    }

    const float edgeSoft = 0.05f;
    const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
    base.a = static_cast<std::uint8_t>(std::lround(static_cast<float>(base.a) * a));
    return base;
  });
}

enum class TerrainKind { Water, Sand, Grass };

RgbaImage MakeTerrainVariant(TerrainKind kind, int variant,
                             int tileW, int tileH, std::uint32_t seed,
                             const GfxPalette& pal)
{
  const std::uint32_t sv = seed ^ (static_cast<std::uint32_t>(variant) * 0x9E3779B9u);

  if (kind == TerrainKind::Water) {
    return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
      const std::uint32_t h = HashCoords32(x, y, sv ^ 0xA1B2C3D4u);
      const float n = (Frac01(h) - 0.5f) * 0.10f;

      const float phase = static_cast<float>(variant) * 0.65f;
      const float waves0 = 0.060f * FastSinRad(static_cast<float>(x) * 0.35f + static_cast<float>(y) * 0.70f + phase);
      const float waves1 = 0.030f * FastSinRad(static_cast<float>(x) * 0.90f - static_cast<float>(y) * 0.45f + phase * 1.73f);

      float b = 1.0f + n + waves0 + waves1;
      b *= 0.92f + 0.15f * d.edge;

      Rgba8 c = Mul(pal.water, b);
      const float a = std::clamp(d.edge * 4.0f, 0.0f, 1.0f);
      c.a = static_cast<std::uint8_t>(std::lround(static_cast<float>(c.a) * a));
      return c;
    });
  }

  if (kind == TerrainKind::Sand) {
    return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
      const std::uint32_t h = HashCoords32(x, y, sv ^ 0x55AA7711u);
      const float n = (Frac01(h) - 0.5f) * 0.18f;
      float b = 1.0f + n;
      b *= 0.92f + 0.12f * d.edge;

      Rgba8 c = Mul(pal.sand, b);

      // subtle speckles
      if ((h & 0x3Fu) == 0x2Au) c = Mul(c, 0.92f);
      if ((h & 0xFFu) == 0xC3u) c = Mul(c, 1.05f);
      c.a = 255;
      return c;
    });
  }

  // Grass
  return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
    const std::uint32_t h = HashCoords32(x, y, sv ^ 0x33445566u);
    const float n = (Frac01(h) - 0.5f) * 0.22f;
    float b = 1.0f + n;
    b *= 0.92f + 0.16f * d.edge;

    Rgba8 c = Mul(pal.grass, b);

    // mottling
    if ((h & 0x7Fu) == 0x19u) c = Mul(c, 0.88f);
    if ((h & 0xFFu) == 0x7Au) c = Mul(c, 1.06f);

    c.a = 255;
    return c;
  });
}


enum class TerrainTransitionKind { WaterSand, SandGrass };

inline int PopCount4(std::uint8_t m)
{
  return ((m & 0x01u) ? 1 : 0) + ((m & 0x02u) ? 1 : 0) + ((m & 0x04u) ? 1 : 0) + ((m & 0x08u) ? 1 : 0);
}

inline Rgba8 GetPixelClamped(const RgbaImage& img, int x, int y)
{
  const int w = img.width;
  const int h = img.height;
  if (w <= 0 || h <= 0) return Rgba8{0, 0, 0, 0};
  x = std::clamp(x, 0, w - 1);
  y = std::clamp(y, 0, h - 1);
  const std::size_t i =
      (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
  Rgba8 c{};
  c.r = img.rgba[i + 0];
  c.g = img.rgba[i + 1];
  c.b = img.rgba[i + 2];
  c.a = img.rgba[i + 3];
  return c;
}

RgbaImage MakeTerrainTransitionVariant(TerrainTransitionKind kind, std::uint8_t mask, int variant, int tileW, int tileH,
                                      std::uint32_t seed, const GfxPalette& pal)
{
  // Interpret mask bits as "neighbor is the *base* terrain". Where a bit is 0, we blend in the edge terrain.
  //
  // Bit layout matches World::computeRoadMask():
  //  0x01 = up-right, 0x02 = down-right, 0x04 = down-left, 0x08 = up-left.
  //
  // This gives us 16 deterministic auto-tiling shapes ("blob tiles") that can be used by external renderers to
  // blend terrain types (shorelines, biome transitions) without any extra art assets.
  const bool waterSand = (kind == TerrainTransitionKind::WaterSand);
  const TerrainKind baseKind = waterSand ? TerrainKind::Water : TerrainKind::Sand;
  const TerrainKind edgeKind = waterSand ? TerrainKind::Sand : TerrainKind::Grass;

  // Deterministic per-mask/per-variant seed.
  const std::uint32_t sv =
      seed ^ (static_cast<std::uint32_t>(mask) * 0x9E3779B9u) ^ (static_cast<std::uint32_t>(variant) * 0x85EBCA6Bu);

  // Pick terrain noise variants for the two layers (0..7).
  const int baseVar = static_cast<int>((sv ^ 0xA1B2C3D4u) & 7u);
  const int edgeVar = static_cast<int>(((sv >> 3) ^ 0x55AA7711u) & 7u);

  const RgbaImage baseImg = MakeTerrainVariant(baseKind, baseVar, tileW, tileH, sv ^ 0x13579BDFu, pal);
  const RgbaImage edgeImg = MakeTerrainVariant(edgeKind, edgeVar, tileW, tileH, sv ^ 0x2468ACE0u, pal);

  // Transition width in normalized diamond coordinates.
  const float bw = waterSand ? 0.21f : 0.18f;

  auto smooth01 = [](float t) -> float {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
  };

  RgbaImage out;
  out.width = tileW;
  out.height = tileH;
  out.rgba.assign(static_cast<std::size_t>(tileW) * static_cast<std::size_t>(tileH) * 4u, 0u);

  for (int y = 0; y < tileH; ++y) {
    for (int x = 0; x < tileW; ++x) {
      const DiamondParams d = DiamondAtPixel(x, y, tileW, tileH);
      if (d.manhattan > 1.0f) continue;

      // Distance to each of the four diamond edges (0 at edge -> 1 at center).
      const float dUR = 1.0f - (d.nx - d.ny);  // up-right
      const float dDR = 1.0f - (d.nx + d.ny);  // down-right
      const float dDL = 1.0f - (-d.nx + d.ny); // down-left
      const float dUL = 1.0f - (-d.nx - d.ny); // up-left

      // Small deterministic jitter so coastlines don't look perfectly banded.
      const std::uint32_t hj = HashCoords32(x, y, sv ^ 0xC0FFEE11u);
      const float jitter = (Frac01(hj) - 0.5f) * (waterSand ? 0.060f : 0.050f);

      auto sideW = [&](float dist) -> float {
        const float t = (bw - (dist + jitter)) / bw; // 1 at edge, 0 at bw
        return smooth01(t);
      };

      float inv = 1.0f;
      if ((mask & 0x01u) == 0u) inv *= (1.0f - sideW(dUR));
      if ((mask & 0x02u) == 0u) inv *= (1.0f - sideW(dDR));
      if ((mask & 0x04u) == 0u) inv *= (1.0f - sideW(dDL));
      if ((mask & 0x08u) == 0u) inv *= (1.0f - sideW(dUL));
      float wEdge = 1.0f - inv;

      // Keep the center "pure" base terrain for readability.
      wEdge *= smooth01(1.0f - std::clamp(d.edge, 0.0f, 1.0f) * 0.25f);

      const Rgba8 base = GetPixelClamped(baseImg, x, y);
      const Rgba8 edge = GetPixelClamped(edgeImg, x, y);

      Rgba8 c = Lerp(base, edge, wEdge);

      // Optional shoreline foam stripe when transitioning water->sand.
      if (waterSand && wEdge > 0.02f && wEdge < 0.98f) {
        auto stripeW = [&](float dist) -> float {
          const float t = dist / std::max(1.0e-6f, bw);
          const float w = 1.0f - std::fabs(t - 0.55f) / 0.10f;
          return smooth01(w);
        };

        float foam = 0.0f;
        if ((mask & 0x01u) == 0u) foam = std::max(foam, stripeW(dUR));
        if ((mask & 0x02u) == 0u) foam = std::max(foam, stripeW(dDR));
        if ((mask & 0x04u) == 0u) foam = std::max(foam, stripeW(dDL));
        if ((mask & 0x08u) == 0u) foam = std::max(foam, stripeW(dUL));

        // Concentrate foam around the blend boundary.
        foam *= std::clamp(wEdge * (1.0f - wEdge) * 4.0f, 0.0f, 1.0f);

        // Break up foam with light noise.
        const std::uint32_t hf = HashCoords32(x + 13, y - 7, sv ^ 0xBADA55u);
        foam *= 0.70f + 0.30f * Frac01(hf);

        if (foam > 0.001f) {
          const std::uint8_t keepA = c.a;
          Rgba8 fc = pal.shorelineFoam;
          fc.a = keepA;
          c = Lerp(c, fc, foam);
          c.a = keepA;
        }
      }

      const std::size_t di =
          (static_cast<std::size_t>(y) * static_cast<std::size_t>(tileW) + static_cast<std::size_t>(x)) * 4u;
      out.rgba[di + 0] = c.r;
      out.rgba[di + 1] = c.g;
      out.rgba[di + 2] = c.b;
      out.rgba[di + 3] = c.a;
    }
  }

  return out;
}


enum class OverlayKind { Residential, Commercial, Industrial, Park };

RgbaImage MakeOverlay(OverlayKind kind, int tileW, int tileH, std::uint32_t seed, const GfxPalette& pal)
{
  if (kind == OverlayKind::Residential) {
    return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
      const std::uint32_t h = HashCoords32(x, y, seed ^ 0xCAFE0001u);
      const float n = (Frac01(h) - 0.5f) * 0.12f;

      Rgba8 roof = pal.overlayResidential;
      roof = Mul(roof, 1.0f + n);

      if ((x + y) % 6 == 0) roof = Mul(roof, 0.86f);
      roof = Mul(roof, 0.92f + 0.10f * d.edge);
      roof.a = 255;
      return roof;
    });
  }

  if (kind == OverlayKind::Commercial) {
    return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
      const std::uint32_t h = HashCoords32(x, y, seed ^ 0xCAFE0002u);
      const float n = (Frac01(h) - 0.5f) * 0.10f;

      Rgba8 c = Mul(pal.overlayCommercial, 1.0f + n);
      if ((x / 3 + y / 2) % 5 == 0) c = Mul(c, 1.15f);
      c = Mul(c, 0.92f + 0.10f * d.edge);
      c.a = 255;
      return c;
    });
  }

  if (kind == OverlayKind::Industrial) {
    return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
      const std::uint32_t h = HashCoords32(x, y, seed ^ 0xCAFE0003u);
      const float n = (Frac01(h) - 0.5f) * 0.10f;

      Rgba8 c = Mul(pal.overlayIndustrial, 1.0f + n);
      if (((x + y) / 3) % 2 == 0) c = Mul(c, 0.85f);
      c = Mul(c, 0.92f + 0.10f * d.edge);
      c.a = 255;
      return c;
    });
  }

  // Park (transparent edges so grass can show through)
  return MakeDiamondImage(tileW, tileH, [&](int x, int y, const DiamondParams& d) -> Rgba8 {
    const std::uint32_t h = HashCoords32(x, y, seed ^ 0xCAFE0004u);
    const float n = (Frac01(h) - 0.5f) * 0.12f;

    Rgba8 c = pal.overlayPark;
    c.a = c.a == 0 ? 230 : c.a;
    c = Mul(c, 1.0f + n);

    if ((h & 0xFFu) == 0x7Au) {
      c = pal.treeDark;
      c.a = 240;
    }

    const float a = std::clamp(d.edge * 7.0f, 0.0f, 1.0f);
    c.a = static_cast<std::uint8_t>(std::lround(static_cast<float>(c.a) * a));
    return c;
  });
}

void BlitNoBlend(const RgbaImage& src, RgbaImage& dst, int dstX, int dstY)
{
  for (int y = 0; y < src.height; ++y) {
    const int yy = dstY + y;
    if (yy < 0 || yy >= dst.height) continue;
    for (int x = 0; x < src.width; ++x) {
      const int xx = dstX + x;
      if (xx < 0 || xx >= dst.width) continue;

      const std::size_t si = (static_cast<std::size_t>(y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 4u;
      const std::size_t di = (static_cast<std::size_t>(yy) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(xx)) * 4u;

      dst.rgba[di + 0] = src.rgba[si + 0];
      dst.rgba[di + 1] = src.rgba[si + 1];
      dst.rgba[di + 2] = src.rgba[si + 2];
      dst.rgba[di + 3] = src.rgba[si + 3];
    }
  }
}

// Round a positive integer up to the next power-of-two (or return the value if it is already a pow2).
static int NextPow2(int v)
{
  if (v <= 0) return 0;
  unsigned int x = static_cast<unsigned int>(v - 1);
  x |= x >> 1;
  x |= x >> 2;
  x |= x >> 4;
  x |= x >> 8;
  x |= x >> 16;
  return static_cast<int>(x + 1u);
}

// Compute tight alpha bounds (exclusive max) for an RGBA image.
// Returns false if the image contains no non-zero alpha pixels.
static bool AlphaBounds(const RgbaImage& img, int& outX0, int& outY0, int& outX1, int& outY1)
{
  outX0 = 0; outY0 = 0; outX1 = 0; outY1 = 0;
  if (img.width <= 0 || img.height <= 0) return false;
  if (img.rgba.empty()) return false;

  int minX = img.width;
  int minY = img.height;
  int maxX = -1;
  int maxY = -1;

  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = img.rgba[i + 3];
      if (a == 0u) continue;
      minX = std::min(minX, x);
      minY = std::min(minY, y);
      maxX = std::max(maxX, x);
      maxY = std::max(maxY, y);
    }
  }

  if (maxX < minX || maxY < minY) return false;

  outX0 = minX;
  outY0 = minY;
  outX1 = maxX + 1;
  outY1 = maxY + 1;
  return true;
}

static RgbaImage CropRgba(const RgbaImage& src, int x0, int y0, int w, int h)
{
  RgbaImage dst;
  dst.width = std::max(0, w);
  dst.height = std::max(0, h);
  dst.rgba.assign(static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 4u, 0u);
  if (dst.width <= 0 || dst.height <= 0) return dst;

  const int sx0 = std::clamp(x0, 0, std::max(0, src.width));
  const int sy0 = std::clamp(y0, 0, std::max(0, src.height));
  const int sx1 = std::clamp(x0 + w, 0, std::max(0, src.width));
  const int sy1 = std::clamp(y0 + h, 0, std::max(0, src.height));

  const int copyW = std::max(0, sx1 - sx0);
  const int copyH = std::max(0, sy1 - sy0);

  for (int y = 0; y < copyH; ++y) {
    const std::size_t srcRow = (static_cast<std::size_t>(sy0 + y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(sx0)) * 4u;
    const std::size_t dstRow = (static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width)) * 4u;
    const std::size_t bytes = static_cast<std::size_t>(copyW) * 4u;
    std::copy_n(src.rgba.begin() + static_cast<std::ptrdiff_t>(srcRow), static_cast<std::ptrdiff_t>(bytes), dst.rgba.begin() + static_cast<std::ptrdiff_t>(dstRow));
  }

  return dst;
}

static std::string JsonEscape(const std::string& s)
{
  std::ostringstream oss;
  for (char c : s) {
    switch (c) {
    case '\\': oss << "\\\\"; break;
    case '"': oss << "\\\""; break;
    case '\n': oss << "\\n"; break;
    case '\r': oss << "\\r"; break;
    case '\t': oss << "\\t"; break;
    default:
      if (static_cast<unsigned char>(c) < 0x20u) {
        oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(c));
      } else {
        oss << c;
      }
      break;
    }
  }
  return oss.str();
}

} // namespace

bool GenerateGfxTileset(const GfxTilesetConfig& cfg, GfxTilesetResult& out, std::string& outError)
{
  outError.clear();
  out = GfxTilesetResult{};

  // Preserve the logical tile size in the result metadata (even if sprites are trimmed).
  out.tileW = cfg.tileW;
  out.tileH = cfg.tileH;

  if (cfg.tileW <= 0 || cfg.tileH <= 0) {
    outError = "invalid tile size";
    return false;
  }
  if (cfg.padding < 0) {
    outError = "invalid padding";
    return false;
  }
  if (cfg.trimBorder < 0) {
    outError = "invalid trimBorder";
    return false;
  }

  if (cfg.packMode == GfxAtlasPackMode::Grid) {
    if (cfg.columns <= 0) {
      outError = "invalid columns";
      return false;
    }
  } else {
    // MaxRects
    if (cfg.packWidth < 0) {
      outError = "invalid packWidth";
      return false;
    }
  }


  const GfxPalette pal = GenerateGfxPalette(cfg.seed, cfg.theme);

  struct Item {
    std::string name;
    RgbaImage img;
    RgbaImage emit;
    RgbaImage height;
    RgbaImage normal;
    RgbaImage shadow;
    RgbaImage sdf;
    int pivotX = 0;
    int pivotY = 0;

    // Trimming metadata (logical source canvas before trimming).
    int srcW = 0;
    int srcH = 0;
    int trimX = 0;
    int trimY = 0;
  };

  std::vector<Item> items;
  items.reserve(512);

  const int kTerrainVariants = 8;
  const int kRoadLevels = 3;
  const int kRoadVariants = 4;

  if (cfg.includeTerrain) {
    for (int v = 0; v < kTerrainVariants; ++v) {
      Item it;
      it.name = "terrain_water_v" + std::to_string(v);
      it.img = MakeTerrainVariant(TerrainKind::Water, v, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
    for (int v = 0; v < kTerrainVariants; ++v) {
      Item it;
      it.name = "terrain_sand_v" + std::to_string(v);
      it.img = MakeTerrainVariant(TerrainKind::Sand, v, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
    for (int v = 0; v < kTerrainVariants; ++v) {
      Item it;
      it.name = "terrain_grass_v" + std::to_string(v);
      it.img = MakeTerrainVariant(TerrainKind::Grass, v, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
  }


  if (cfg.includeTerrain && cfg.includeTransitions) {
    const int tv = std::clamp(cfg.transitionVariants, 1, 8);

    // Shoreline / biome transition tiles using the same 4-bit mask convention as roads.
    // These are optional “blob tiles” that external renderers can use to blend terrain types.
    for (int mask = 0; mask < 16; ++mask) {
      for (int v = 0; v < tv; ++v) {
        Item it;
        it.name = "terrain_shore_ws_m" + std::to_string(mask) + "_v" + std::to_string(v);
        it.img = MakeTerrainTransitionVariant(TerrainTransitionKind::WaterSand, static_cast<std::uint8_t>(mask), v,
                                              cfg.tileW, cfg.tileH, cfg.seed, pal);
        it.pivotX = cfg.tileW / 2;
        it.pivotY = cfg.tileH / 2;
        items.push_back(std::move(it));
      }
    }

    for (int mask = 0; mask < 16; ++mask) {
      for (int v = 0; v < tv; ++v) {
        Item it;
        it.name = "terrain_shore_sg_m" + std::to_string(mask) + "_v" + std::to_string(v);
        it.img = MakeTerrainTransitionVariant(TerrainTransitionKind::SandGrass, static_cast<std::uint8_t>(mask), v,
                                              cfg.tileW, cfg.tileH, cfg.seed, pal);
        it.pivotX = cfg.tileW / 2;
        it.pivotY = cfg.tileH / 2;
        items.push_back(std::move(it));
      }
    }
  }

  if (cfg.includeRoads) {
    for (int level = 1; level <= kRoadLevels; ++level) {
      for (int mask = 0; mask < 16; ++mask) {
        for (int v = 0; v < kRoadVariants; ++v) {
          const std::uint32_t seedv =
              cfg.seed ^ 0x0F0F0F0Fu ^ (static_cast<std::uint32_t>(mask) * 0x9E3779B9u) ^
              (static_cast<std::uint32_t>(v) * 0x85EBCA6Bu) ^
              (static_cast<std::uint32_t>(level) * 0xC2B2AE35u);

          Item it;
          it.name = "road_L" + std::to_string(level) + "_m" + std::to_string(mask) + "_v" + std::to_string(v);
          it.img = MakeRoadVariant(static_cast<std::uint8_t>(mask), level, v, cfg.tileW, cfg.tileH, seedv, pal);
          it.pivotX = cfg.tileW / 2;
          it.pivotY = cfg.tileH / 2;
          items.push_back(std::move(it));
        }
      }
    }
  }

  if (cfg.includeBridges) {
    for (int level = 1; level <= kRoadLevels; ++level) {
      for (int mask = 0; mask < 16; ++mask) {
        for (int v = 0; v < kRoadVariants; ++v) {
          const std::uint32_t seedv =
              cfg.seed ^ 0xB00B1E5u ^ (static_cast<std::uint32_t>(mask) * 0x7F4A7C15u) ^
              (static_cast<std::uint32_t>(v) * 0x27D4EB2Du) ^
              (static_cast<std::uint32_t>(level) * 0x165667B1u);

          Item it;
          it.name = "bridge_L" + std::to_string(level) + "_m" + std::to_string(mask) + "_v" + std::to_string(v);
          it.img = MakeBridgeVariant(static_cast<std::uint8_t>(mask), level, v, cfg.tileW, cfg.tileH, seedv, pal);
          it.pivotX = cfg.tileW / 2;
          it.pivotY = cfg.tileH / 2;
          items.push_back(std::move(it));
        }
      }
    }
  }

  if (cfg.includeOverlays) {
    {
      Item it;
      it.name = "overlay_residential";
      it.img = MakeOverlay(OverlayKind::Residential, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
    {
      Item it;
      it.name = "overlay_commercial";
      it.img = MakeOverlay(OverlayKind::Commercial, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
    {
      Item it;
      it.name = "overlay_industrial";
      it.img = MakeOverlay(OverlayKind::Industrial, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
    {
      Item it;
      it.name = "overlay_park";
      it.img = MakeOverlay(OverlayKind::Park, cfg.tileW, cfg.tileH, cfg.seed, pal);
      it.pivotX = cfg.tileW / 2;
      it.pivotY = cfg.tileH / 2;
      items.push_back(std::move(it));
    }
  }

  if (cfg.includeBuildings && cfg.buildingVariants > 0) {
    GfxBuildingsConfig bcfg;
    bcfg.tileW = cfg.tileW;
    bcfg.tileH = cfg.tileH;
    bcfg.spriteH = cfg.buildingSpriteH;
    bcfg.includeEmissive = cfg.includeEmissive;

    const int variants = std::clamp(cfg.buildingVariants, 1, 64);

    auto addKind = [&](GfxBuildingKind kind, const char* name) {
      for (int lvl = 1; lvl <= 3; ++lvl) {
        for (int v = 0; v < variants; ++v) {
          GfxBuildingSprite bs;
          std::string err;
          if (!GenerateGfxBuildingSprite(kind, lvl, v, cfg.seed, bcfg, pal, bs, err)) {
            outError = "building sprite generation failed: " + err;
            return false;
          }

          Item it;
          it.name = std::string("building_") + name + "_L" + std::to_string(lvl) + "_v" + std::to_string(v);
          it.img = std::move(bs.color);
          it.emit = std::move(bs.emissive);
          it.pivotX = bs.pivotX;
          it.pivotY = bs.pivotY;
          items.push_back(std::move(it));
        }
      }
      return true;
    };

    if (!addKind(GfxBuildingKind::Residential, "res")) return false;
    if (!addKind(GfxBuildingKind::Commercial, "com")) return false;
    if (!addKind(GfxBuildingKind::Industrial, "ind")) return false;
  }

  // ---------------------------------------------------------------------------
  // Civic / service facility sprites (schools / clinics / police / fire).
  // ---------------------------------------------------------------------------

  if (cfg.includeFacilities && cfg.facilityVariants > 0) {
    GfxFacilitiesConfig fcfg;
    fcfg.tileW = cfg.tileW;
    fcfg.tileH = cfg.tileH;
    fcfg.spriteH = cfg.facilitySpriteH;
    fcfg.includeEmissive = cfg.includeEmissive;

    const int variants = std::clamp(cfg.facilityVariants, 1, 64);

    auto addKind = [&](GfxFacilityKind kind, const char* name) {
      for (int lvl = 1; lvl <= 3; ++lvl) {
        for (int v = 0; v < variants; ++v) {
          GfxFacilitySprite fs;
          std::string err;
          if (!GenerateGfxFacilitySprite(kind, lvl, v, cfg.seed, fcfg, pal, fs, err)) {
            outError = "facility sprite generation failed: " + err;
            return false;
          }

          Item it;
          it.name = std::string("facility_") + name + "_L" + std::to_string(lvl) + "_v" + std::to_string(v);
          it.img = std::move(fs.color);
          it.emit = std::move(fs.emissive);
          it.pivotX = fs.pivotX;
          it.pivotY = fs.pivotY;
          items.push_back(std::move(it));
        }
      }
      return true;
    };

    if (!addKind(GfxFacilityKind::Education, "edu")) return false;
    if (!addKind(GfxFacilityKind::Health, "health")) return false;
    if (!addKind(GfxFacilityKind::Police, "police")) return false;
    if (!addKind(GfxFacilityKind::Fire, "fire")) return false;
  }

  // ---------------------------------------------------------------------------
  // Prop sprites (trees, streetlights) and vehicle sprites.
  // ---------------------------------------------------------------------------

  if (cfg.includeProps && cfg.propVariants > 0) {
    GfxPropsConfig pcfg;
    pcfg.tileW = cfg.tileW;
    pcfg.tileH = cfg.tileH;
    pcfg.tallSpriteH = cfg.propSpriteH;
    pcfg.includeEmissive = cfg.includeEmissive;

    const int variants = std::clamp(cfg.propVariants, 1, 128);

    auto addProp = [&](GfxPropKind kind, const char* name) {
      for (int v = 0; v < variants; ++v) {
        GfxPropSprite ps;
        std::string err;
        if (!GenerateGfxPropSprite(kind, v, cfg.seed, pcfg, pal, ps, err)) {
          outError = std::string("prop sprite generation failed (") + name + "): " + err;
          return false;
        }

        Item it;
        it.name = std::string("prop_") + name + "_v" + std::to_string(v);
        it.img = std::move(ps.color);
        it.emit = std::move(ps.emissive);
        it.pivotX = ps.pivotX;
        it.pivotY = ps.pivotY;
        items.push_back(std::move(it));
      }
      return true;
    };

    if (!addProp(GfxPropKind::TreeDeciduous, "tree_deciduous")) return false;
    if (!addProp(GfxPropKind::TreeConifer, "tree_conifer")) return false;
    if (!addProp(GfxPropKind::StreetLight, "streetlight")) return false;
  }

  if (cfg.includeVehicles && cfg.vehicleVariants > 0) {
    GfxPropsConfig pcfg;
    pcfg.tileW = cfg.tileW;
    pcfg.tileH = cfg.tileH;
    pcfg.tallSpriteH = cfg.propSpriteH;
    pcfg.includeEmissive = cfg.includeEmissive;

    const int variants = std::clamp(cfg.vehicleVariants, 1, 256);

    auto addVeh = [&](GfxPropKind kind, const char* name) {
      for (int v = 0; v < variants; ++v) {
        GfxPropSprite ps;
        std::string err;
        if (!GenerateGfxPropSprite(kind, v, cfg.seed, pcfg, pal, ps, err)) {
          outError = std::string("vehicle sprite generation failed (") + name + "): " + err;
          return false;
        }

        Item it;
        it.name = std::string("prop_") + name + "_v" + std::to_string(v);
        it.img = std::move(ps.color);
        it.emit = std::move(ps.emissive);
        it.pivotX = ps.pivotX;
        it.pivotY = ps.pivotY;
        items.push_back(std::move(it));
      }
      return true;
    };

    if (!addVeh(GfxPropKind::VehicleCar, "car")) return false;
    if (!addVeh(GfxPropKind::VehicleTruck, "truck")) return false;
  }

  // ---------------------------------------------------------------------------
  // Derived textures (height / normal / shadow) per sprite.
  //
  // We generate these *before* packing so each derived atlas matches the sprite
  // rectangles 1:1 (same layout). This keeps mod pipelines simple.
  // ---------------------------------------------------------------------------

  const bool wantHeight = cfg.includeHeight;
  const bool wantNormal = cfg.includeNormals;
  const bool wantShadow = cfg.includeShadows;
  const bool wantSdf = cfg.includeSdf;

  if (wantHeight || wantNormal || wantShadow || wantSdf) {
    GfxNormalMapConfig ncfg;
    ncfg.heightMode = cfg.heightMode;
    ncfg.strength = cfg.normalStrength;

    for (Item& it : items) {
      std::string derr;
      if (wantHeight) {
        if (!GenerateHeightMap(it.img, cfg.heightMode, it.height, derr)) {
          outError = "height map generation failed for '" + it.name + "': " + derr;
          return false;
        }
      }

      if (wantNormal) {
        if (!GenerateNormalMap(it.img, ncfg, it.normal, derr)) {
          outError = "normal map generation failed for '" + it.name + "': " + derr;
          return false;
        }
      }

      if (wantShadow) {
        const bool tallEnough = (it.img.height > cfg.tileH + 1);
        if (!cfg.shadowTallSpritesOnly || tallEnough) {
          if (!GenerateShadowMap(it.img, cfg.shadow, it.shadow, derr)) {
            outError = "shadow map generation failed for '" + it.name + "': " + derr;
            return false;
          }
        }
      }

      if (wantSdf) {
        if (!GenerateSignedDistanceField(it.img, cfg.sdf, it.sdf, derr)) {
          outError = "sdf generation failed for '" + it.name + "': " + derr;
          return false;
        }
      }
    }
  }

  const int count = static_cast<int>(items.size());
  if (count == 0) {
    outError = "tileset would be empty";
    return false;
  }

  // Initialize trimming metadata (logical source canvas before trimming).
  for (Item& it : items) {
    it.srcW = it.img.width;
    it.srcH = it.img.height;
    it.trimX = 0;
    it.trimY = 0;
  }

  // Optional trimming: crop transparent borders before packing.
  if (cfg.trimTransparent) {
    int border = std::max(0, cfg.trimBorder);

    // If we generated an SDF, keep enough border for the distance field to be meaningful.
    if (cfg.includeSdf) {
      border = std::max(border, static_cast<int>(std::ceil(cfg.sdf.spreadPx)));
    }

    for (Item& it : items) {
      int bx0 = 0, by0 = 0, bx1 = 0, by1 = 0;
      if (!AlphaBounds(it.img, bx0, by0, bx1, by1)) {
        continue;
      }

      bx0 = std::max(0, bx0 - border);
      by0 = std::max(0, by0 - border);
      bx1 = std::min(it.img.width, bx1 + border);
      by1 = std::min(it.img.height, by1 + border);

      const int nw = std::max(1, bx1 - bx0);
      const int nh = std::max(1, by1 - by0);

      if (nw == it.img.width && nh == it.img.height && bx0 == 0 && by0 == 0) {
        continue;
      }

      it.trimX = bx0;
      it.trimY = by0;

      it.img = CropRgba(it.img, bx0, by0, nw, nh);
      if (!it.emit.rgba.empty()) it.emit = CropRgba(it.emit, bx0, by0, nw, nh);
      if (!it.height.rgba.empty()) it.height = CropRgba(it.height, bx0, by0, nw, nh);
      if (!it.normal.rgba.empty()) it.normal = CropRgba(it.normal, bx0, by0, nw, nh);
      if (!it.shadow.rgba.empty()) it.shadow = CropRgba(it.shadow, bx0, by0, nw, nh);
      if (!it.sdf.rgba.empty()) it.sdf = CropRgba(it.sdf, bx0, by0, nw, nh);

      it.pivotX = std::clamp(it.pivotX - bx0, 0, it.img.width);
      it.pivotY = std::clamp(it.pivotY - by0, 0, it.img.height);
    }
  }

  const int pad = cfg.padding;
  const int margin = pad;

  // Pack positions relative to the inner atlas (0,0) excluding margin.
  std::vector<int> posX(static_cast<std::size_t>(count), 0);
  std::vector<int> posY(static_cast<std::size_t>(count), 0);

  int atlasW = 0;
  int atlasH = 0;

  if (cfg.packMode == GfxAtlasPackMode::Grid) {
    const int cols = std::max(1, cfg.columns);
    const int rows = (count + cols - 1) / cols;

    // Support variable sprite heights while keeping a deterministic grid.
    int cellW = 0;
    std::vector<int> rowH(static_cast<std::size_t>(rows), 0);
    for (int i = 0; i < count; ++i) {
      const Item& it = items[static_cast<std::size_t>(i)];
      cellW = std::max(cellW, it.img.width);
      const int row = i / cols;
      if (row >= 0 && row < rows) {
        rowH[static_cast<std::size_t>(row)] = std::max(rowH[static_cast<std::size_t>(row)], it.img.height);
      }
    }
    if (cellW <= 0) cellW = cfg.tileW;

    std::vector<int> rowY(static_cast<std::size_t>(rows), 0);
    int innerH = 0;
    for (int r = 0; r < rows; ++r) {
      rowY[static_cast<std::size_t>(r)] = innerH;
      innerH += rowH[static_cast<std::size_t>(r)];
      if (r + 1 < rows) innerH += pad;
    }

    const int innerW = cols * cellW + (cols - 1) * pad;
    atlasW = margin * 2 + innerW;
    atlasH = margin * 2 + innerH;

    for (int i = 0; i < count; ++i) {
      const int col = i % cols;
      const int row = i / cols;
      posX[static_cast<std::size_t>(i)] = col * (cellW + pad);
      posY[static_cast<std::size_t>(i)] = rowY[static_cast<std::size_t>(row)];
    }
  } else {
    // MaxRects rectangle packing (denser for mixed sprite sizes).
    std::vector<GfxPackRect> rects;
    rects.reserve(static_cast<std::size_t>(count));

    long long totalArea = 0;
    int maxRW = 0;
    int maxRH = 0;
    int sumRH = 0;

    for (int i = 0; i < count; ++i) {
      const Item& it = items[static_cast<std::size_t>(i)];
      const int rw = it.img.width + pad;
      const int rh = it.img.height + pad;
      rects.push_back(GfxPackRect{i, rw, rh, 0, 0});
      totalArea += static_cast<long long>(rw) * static_cast<long long>(rh);
      maxRW = std::max(maxRW, rw);
      maxRH = std::max(maxRH, rh);
      sumRH += rh;
    }

    int binW = cfg.packWidth > 0 ? cfg.packWidth : static_cast<int>(std::ceil(std::sqrt(static_cast<double>(totalArea))));
    binW = std::max(binW, maxRW);
    // Keep the bin width reasonably aligned (helps some compressors and avoids tiny differences).
    binW = (binW + 3) & ~3;

    const int binH = std::max(sumRH, maxRH);

    std::string perr;
    if (!PackMaxRects(binW, binH, rects, perr)) {
      // Fallback: shelf packing always succeeds if binW >= maxRW.
      int usedH = 0;
      if (!PackShelf(binW, rects, usedH, perr)) {
        outError = "atlas packing failed: " + perr;
        return false;
      }
    }

    for (const auto& r : rects) {
      if (r.id < 0 || r.id >= count) continue;
      posX[static_cast<std::size_t>(r.id)] = r.x;
      posY[static_cast<std::size_t>(r.id)] = r.y;
    }

    int usedW = 0;
    int usedH = 0;
    for (int i = 0; i < count; ++i) {
      const Item& it = items[static_cast<std::size_t>(i)];
      usedW = std::max(usedW, posX[static_cast<std::size_t>(i)] + it.img.width);
      usedH = std::max(usedH, posY[static_cast<std::size_t>(i)] + it.img.height);
    }

    atlasW = margin * 2 + usedW;
    atlasH = margin * 2 + usedH;
  }

  if (cfg.packPow2) {
    atlasW = NextPow2(atlasW);
    atlasH = NextPow2(atlasH);
  }

  // Allocate atlases.
  RgbaImage atlas;
  atlas.width = atlasW;
  atlas.height = atlasH;
  atlas.rgba.assign(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 4u, 0u);

  RgbaImage emissiveAtlas;
  if (cfg.includeEmissive) {
    emissiveAtlas.width = atlasW;
    emissiveAtlas.height = atlasH;
    emissiveAtlas.rgba.assign(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 4u, 0u);
  }

  RgbaImage heightAtlas;
  if (cfg.includeHeight) {
    heightAtlas.width = atlasW;
    heightAtlas.height = atlasH;
    heightAtlas.rgba.assign(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 4u, 0u);
  }

  RgbaImage normalAtlas;
  if (cfg.includeNormals) {
    normalAtlas.width = atlasW;
    normalAtlas.height = atlasH;
    normalAtlas.rgba.resize(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 4u, 0u);
    const std::size_t pxCount = static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH);
    for (std::size_t i = 0; i < pxCount; ++i) {
      const std::size_t di = i * 4u;
      normalAtlas.rgba[di + 0] = 128u;
      normalAtlas.rgba[di + 1] = 128u;
      normalAtlas.rgba[di + 2] = 255u;
      normalAtlas.rgba[di + 3] = 0u;
    }
  }

  RgbaImage shadowAtlas;
  if (cfg.includeShadows) {
    shadowAtlas.width = atlasW;
    shadowAtlas.height = atlasH;
    shadowAtlas.rgba.assign(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 4u, 0u);
  }

  RgbaImage sdfAtlas;
  if (cfg.includeSdf) {
    sdfAtlas.width = atlasW;
    sdfAtlas.height = atlasH;
    sdfAtlas.rgba.assign(static_cast<std::size_t>(atlasW) * static_cast<std::size_t>(atlasH) * 4u, 0u);
  }

  out.entries.clear();
  out.entries.reserve(items.size());

  for (int i = 0; i < count; ++i) {
    const int x0 = margin + posX[static_cast<std::size_t>(i)];
    const int y0 = margin + posY[static_cast<std::size_t>(i)];

    const Item& it = items[static_cast<std::size_t>(i)];
    BlitNoBlend(it.img, atlas, x0, y0);
    if (cfg.includeEmissive && !it.emit.rgba.empty()) {
      BlitNoBlend(it.emit, emissiveAtlas, x0, y0);
    }

    if (cfg.includeHeight && !it.height.rgba.empty()) {
      BlitNoBlend(it.height, heightAtlas, x0, y0);
    }
    if (cfg.includeNormals && !it.normal.rgba.empty()) {
      BlitNoBlend(it.normal, normalAtlas, x0, y0);
    }
    if (cfg.includeShadows && !it.shadow.rgba.empty()) {
      BlitNoBlend(it.shadow, shadowAtlas, x0, y0);
    }

    if (cfg.includeSdf && !it.sdf.rgba.empty()) {
      BlitNoBlend(it.sdf, sdfAtlas, x0, y0);
    }

    GfxAtlasEntry e;
    e.name = it.name;
    e.x = x0;
    e.y = y0;
    e.w = it.img.width;
    e.h = it.img.height;
    e.pivotX = it.pivotX;
    e.pivotY = it.pivotY;
    e.srcW = it.srcW;
    e.srcH = it.srcH;
    e.trimX = it.trimX;
    e.trimY = it.trimY;
    out.entries.push_back(std::move(e));
  }

  out.atlas = std::move(atlas);
  out.emissiveAtlas = std::move(emissiveAtlas);
  out.heightAtlas = std::move(heightAtlas);
  out.normalAtlas = std::move(normalAtlas);
  out.shadowAtlas = std::move(shadowAtlas);
  out.sdfAtlas = std::move(sdfAtlas);
  out.sdfSpreadPx = cfg.sdf.spreadPx;
  out.sdfAlphaThreshold = cfg.sdf.alphaThreshold;
  out.sdfOpaqueAlpha = cfg.sdf.opaqueAlpha;
  return true;
}

bool WriteGfxTilesetMetaJson(const std::string& path, const GfxTilesetResult& ts, std::string& outError)
{
  outError.clear();

  std::ofstream f(path);
  if (!f) {
    outError = "failed to open meta file for writing";
    return false;
  }

  f << "{\n";
  f << "  \"atlasW\": " << ts.atlas.width << ",\n";
  f << "  \"atlasH\": " << ts.atlas.height << ",\n";
  f << "  \"tileW\": " << ts.tileW << ",\n";
  f << "  \"tileH\": " << ts.tileH << ",\n";
  f << "  \"hasEmissive\": " << (!ts.emissiveAtlas.rgba.empty() ? "true" : "false") << ",\n";
  f << "  \"hasHeight\": " << (!ts.heightAtlas.rgba.empty() ? "true" : "false") << ",\n";
  f << "  \"hasNormals\": " << (!ts.normalAtlas.rgba.empty() ? "true" : "false") << ",\n";
  f << "  \"hasShadows\": " << (!ts.shadowAtlas.rgba.empty() ? "true" : "false") << ",\n";
  f << "  \"hasSdf\": " << (!ts.sdfAtlas.rgba.empty() ? "true" : "false") << ",\n";
  if (!ts.normalAtlas.rgba.empty()) {
    f << "  \"normalYAxis\": \"up\",\n";
  }
  if (!ts.sdfAtlas.rgba.empty()) {
    f << "  \"sdf\": {\"spreadPx\": " << ts.sdfSpreadPx << ", \"alphaThreshold\": " << ts.sdfAlphaThreshold << ", \"alphaMode\": \"" << (ts.sdfOpaqueAlpha ? "opaque" : "preserve") << "\", \"encoding\": \"0.5 + sd/spreadPx\"},\n";
  }
  f << "  \"sprites\": [\n";

  for (std::size_t i = 0; i < ts.entries.size(); ++i) {
    const auto& e = ts.entries[i];
    f << "    {\"name\": \"" << JsonEscape(e.name) << "\", \"x\": " << e.x << ", \"y\": " << e.y
      << ", \"w\": " << e.w << ", \"h\": " << e.h
      << ", \"pivotX\": " << e.pivotX << ", \"pivotY\": " << e.pivotY << ", \"srcW\": " << e.srcW << ", \"srcH\": " << e.srcH << ", \"trimX\": " << e.trimX << ", \"trimY\": " << e.trimY << "}";
    if (i + 1 < ts.entries.size()) f << ",";
    f << "\n";
  }

  f << "  ]\n";
  f << "}\n";

  return true;
}

} // namespace isocity
