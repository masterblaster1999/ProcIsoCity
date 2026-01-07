#include "isocity/Renderer.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/ZoneAccess.hpp"

#include "isocity/Random.hpp"
#include "isocity/Noise.hpp"
#include "isocity/ZoneMetrics.hpp"
#include "isocity/Road.hpp"
#include "isocity/Traffic.hpp"

#include "isocity/GfxProps.hpp"


#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace isocity {

// Forward declarations (Renderer.cpp is a single TU with many helpers; keep things
// declared before first use so MSVC doesn't fail with "identifier not found" errors).
static Color TerrainCliffBaseColor(Terrain t);

namespace {

inline unsigned char ClampU8(int v) { return static_cast<unsigned char>(std::clamp(v, 0, 255)); }

// Convert an isocity::RgbaImage (byte RGBA) into a raylib Image.
inline Image ImageFromRgbaImage(const RgbaImage& src)
{
  Image img{};
  img.width = src.width;
  img.height = src.height;
  img.mipmaps = 1;
  img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

  const int w = std::max(0, src.width);
  const int h = std::max(0, src.height);
  const int bytes = w * h * 4;
  img.data = (bytes > 0) ? MemAlloc(bytes) : nullptr;
  if (img.data && static_cast<int>(src.rgba.size()) >= bytes) {
    std::memcpy(img.data, src.rgba.data(), static_cast<std::size_t>(bytes));
  }
  return img;
}

// Compute the sign of cov(x,y) for the alpha mask, which is a cheap way to
// classify whether a sprite's major axis aligns to the screen-space +45° or
// -45° diagonal.
inline double AlphaCovXY(const RgbaImage& img)
{
  const int w = img.width;
  const int h = img.height;
  if (w <= 0 || h <= 0) return 0.0;
  if (static_cast<int>(img.rgba.size()) < w * h * 4) return 0.0;

  double sumW = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumXY = 0.0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      const unsigned char a = img.rgba[i + 3u];
      if (a == 0) continue;
      const double ww = static_cast<double>(a) / 255.0;
      sumW += ww;
      sumX += ww * static_cast<double>(x);
      sumY += ww * static_cast<double>(y);
      sumXY += ww * static_cast<double>(x) * static_cast<double>(y);
    }
  }

  if (sumW <= 1e-9) return 0.0;
  const double mx = sumX / sumW;
  const double my = sumY / sumW;
  const double exy = sumXY / sumW;
  return exy - mx * my;
}


inline Color Mul(Color c, float b)
{
  const int r = static_cast<int>(std::round(static_cast<float>(c.r) * b));
  const int g = static_cast<int>(std::round(static_cast<float>(c.g) * b));
  const int bl = static_cast<int>(std::round(static_cast<float>(c.b) * b));
  return Color{ClampU8(r), ClampU8(g), ClampU8(bl), c.a};
}

inline Color Lerp(Color a, Color b, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  const int r = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.r) + t * static_cast<float>(b.r)));
  const int g = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.g) + t * static_cast<float>(b.g)));
  const int bl = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.b) + t * static_cast<float>(b.b)));
  const int al = static_cast<int>(std::round((1.0f - t) * static_cast<float>(a.a) + t * static_cast<float>(b.a)));
  return Color{ClampU8(r), ClampU8(g), ClampU8(bl), ClampU8(al)};
}

inline float Lerp(float a, float b, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  return a + (b - a) * t;
}

inline Vector2 LerpV(const Vector2& a, const Vector2& b, float t)
{
  return Vector2{Lerp(a.x, b.x, t), Lerp(a.y, b.y, t)};
}

inline Color DistrictBaseColor(std::uint8_t d)
{
  switch (d & 7u) {
  case 1: return Color{50, 140, 255, 255};   // blue
  case 2: return Color{255, 170, 60, 255};   // orange
  case 3: return Color{80, 200, 120, 255};   // green
  case 4: return Color{190, 90, 255, 255};   // purple
  case 5: return Color{255, 80, 80, 255};    // red
  case 6: return Color{60, 220, 220, 255};   // cyan
  case 7: return Color{255, 230, 70, 255};   // yellow
  default: return Color{0, 0, 0, 0};         // district 0 (unassigned): transparent
  }
}

inline Color DistrictFillColor(std::uint8_t d, unsigned char alpha)
{
  Color c = DistrictBaseColor(d);
  c.a = alpha;
  return c;
}

// Filled isometric diamond used by several overlay passes.
inline void DrawDiamond(const Vector2& center, float tileW, float tileH, Color c)
{
  Vector2 corners[4];
  TileDiamondCorners(center, tileW, tileH, corners);
  DrawTriangle(corners[0], corners[1], corners[2], c);
  DrawTriangle(corners[0], corners[2], corners[3], c);
}

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

inline bool IsImageReadyCompat(const Image& img)
{
  // raylib has gained helper "Is*Ready" functions over time, but some versions used
  // by FetchContent don't include IsImageReady. This local check keeps builds working
  // across raylib versions.
  return (img.data != nullptr) && (img.width > 0) && (img.height > 0);
}

// ---------------------------------------------------------------------------------------------
// Day / night cycle helpers
// ---------------------------------------------------------------------------------------------

constexpr float kPiF = 3.14159265358979323846f;

inline float SmoothStep(float edge0, float edge1, float x)
{
  if (edge0 == edge1) return (x < edge0) ? 0.0f : 1.0f;
  const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

struct DayNightState {
  float phase = 0.0f;    // 0..1
  float sun = 0.0f;      // -1..1
  float day = 1.0f;      // 0..1
  float night = 0.0f;    // 0..1
  float twilight = 0.0f; // 0..1 (dawn/dusk)

  // Convenience values used by the renderer.
  float dusk = 0.0f;        // 0..1 warm sunrise/sunset tint strength
  float nightLights = 0.0f; // 0..1 emissive lights strength
};

inline DayNightState ComputeDayNightState(float timeSec, const Renderer::DayNightSettings& s)
{
  DayNightState st{};
  if (!s.enabled) {
    st.day = 1.0f;
    st.night = 0.0f;
    st.twilight = 0.0f;
    st.sun = 1.0f;
    st.dusk = 0.0f;
    st.nightLights = 0.0f;
    return st;
  }

  const float len = std::max(1.0f, s.dayLengthSec);
  float t = std::fmod(timeSec + s.timeOffsetSec, len);
  if (t < 0.0f) t += len;
  st.phase = t / len;

  // A simple sine sun curve: sunrise at phase 0, noon at 0.25, sunset at 0.5, midnight at 0.75.
  st.sun = std::sin(st.phase * 2.0f * kPiF);

  // Daylight turns on slightly before the sun reaches the horizon and fades out slightly after.
  st.day = SmoothStep(-0.18f, 0.22f, st.sun);
  st.night = 1.0f - st.day;

  // Twilight is strongest near the horizon (sun ~ 0).
  const float absSun = std::fabs(st.sun);
  st.twilight = SmoothStep(0.28f, 0.0f, absSun);

  // A warm dusk tint is strongest during twilight.
  st.dusk = st.twilight;

  // City lights fade in with night and start to appear a bit during twilight.
  st.nightLights = std::clamp(st.night + 0.35f * st.twilight, 0.0f, 1.0f);

  return st;
}


// ---------------------------------------------------------------------------------------------
// Weather / atmosphere helpers
// ---------------------------------------------------------------------------------------------

struct WeatherState {
  Renderer::WeatherSettings::Mode mode = Renderer::WeatherSettings::Mode::Clear;

  float intensity = 0.0f;   // 0..1 precipitation intensity
  float wetness = 0.0f;     // 0..1 (used for wet sheen/reflections)
  float snow = 0.0f;        // 0..1 ground snow cover
  float overcast = 0.0f;    // 0..1 scene grade strength
  float fog = 0.0f;         // 0..1 screen-space fog gradient strength

  // Screen-space wind direction (normalized, y down).
  float windX = 0.0f;
  float windY = 1.0f;

  float windSpeed = 1.0f;   // multiplier for particle motion
};

inline const char* WeatherModeName(Renderer::WeatherSettings::Mode m)
{
  using M = Renderer::WeatherSettings::Mode;
  switch (m) {
  case M::Rain: return "Rain";
  case M::Snow: return "Snow";
  default: return "Clear";
  }
}

inline WeatherState ComputeWeatherState(float timeSec, const Renderer::WeatherSettings& s)
{
  WeatherState w{};
  w.mode = s.mode;

  const float inten = std::clamp(s.intensity, 0.0f, 1.0f);
  if (s.mode == Renderer::WeatherSettings::Mode::Clear) {
    return w;
  }

  w.intensity = inten;
  w.overcast = std::clamp(s.overcast, 0.0f, 1.0f) * inten;
  w.fog = std::clamp(s.fog, 0.0f, 1.0f) * inten;

  w.windSpeed = std::clamp(s.windSpeed, 0.05f, 6.0f);

  // Wind direction in degrees (screen-space): 0=right, 90=down.
  const float ang = (s.windAngleDeg) * (kPiF / 180.0f);

  // Add a subtle time-varying gust wobble so precipitation doesn't look "stamped on".
  const float gust = 0.25f * std::sin(timeSec * 0.35f) + 0.15f * std::sin(timeSec * 0.73f + 1.2f);
  const float ang2 = ang + gust * 0.25f;

  float wx = std::cos(ang2);
  float wy = std::sin(ang2);

  const float len2 = wx * wx + wy * wy;
  if (len2 > 1.0e-6f) {
    const float inv = 1.0f / std::sqrt(len2);
    wx *= inv;
    wy *= inv;
  } else {
    wx = 0.0f;
    wy = 1.0f;
  }

  // Bias toward downward motion for readability.
  if (wy < 0.15f) wy = 0.15f;

  // Re-normalize after bias.
  const float len3 = wx * wx + wy * wy;
  if (len3 > 1.0e-6f) {
    const float inv = 1.0f / std::sqrt(len3);
    wx *= inv;
    wy *= inv;
  }

  w.windX = wx;
  w.windY = wy;

  if (s.mode == Renderer::WeatherSettings::Mode::Rain) {
    w.wetness = inten;
    w.snow = 0.0f;
  } else { // Snow
    w.snow = inten;
    w.wetness = inten * 0.15f; // a little slush sheen
  }

  return w;
}

inline int Popcount4(std::uint8_t v)
{
  v &= 0x0Fu;
  // Hacker's Delight popcount for 4 bits.
  v = v - ((v >> 1) & 0x55u);
  v = (v & 0x33u) + ((v >> 2) & 0x33u);
  return static_cast<int>((v + (v >> 4)) & 0x0Fu);
}

inline void DrawGlow(const Vector2& p, float radius, Color outer, Color inner)
{
  // Poor-man's radial gradient using two circles; good enough for small emissive decals.
  DrawCircleV(p, radius, outer);
  DrawCircleV(p, radius * 0.55f, inner);
}

// ---------------------------------------------------------------------------------------------
// Procedural micro-details (no external art)
// ---------------------------------------------------------------------------------------------

inline Vector2 SamplePointInDiamond(const Vector2& center, float halfW, float halfH, float nx, float ny, float margin)
{
  // Map a point from the unit square to the L1 unit ball (a diamond) by normalizing if needed.
  float man = std::fabs(nx) + std::fabs(ny);
  if (man > 1.0f) {
    nx /= man;
    ny /= man;
  }
  nx *= margin;
  ny *= margin;
  return Vector2{center.x + nx * halfW, center.y + ny * halfH};
}

inline Vector2 DeterministicDiamondPoint(int tx, int ty, std::uint32_t seed, int idx,
                                        const Vector2& center, float tileW, float tileH, float margin)
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;

  const std::uint32_t h1 = HashCoords32(tx + idx * 37, ty - idx * 53, seed ^ 0x68BC21EBu);
  const std::uint32_t h2 = HashCoords32(tx - idx * 29, ty + idx * 71, seed ^ 0x02E5BE93u);

  const float nx = Frac01(h1) * 2.0f - 1.0f;
  const float ny = Frac01(h2) * 2.0f - 1.0f;

  return SamplePointInDiamond(center, halfW, halfH, nx, ny, margin);
}

inline Color ShadeDetail(Color c, float brightness, float mul, unsigned char alpha)
{
  Color out = Mul(c, std::clamp(brightness * mul, 0.45f, 1.35f));
  out.a = alpha;
  return out;
}

static void DrawProceduralTileDetails(const World& world, int x, int y, const Tile& t, const Vector2& center,
                                     float tileW, float tileH, float zoom, float brightness, std::uint32_t seed32,
                                     float timeSec)
{
  (void)world;
  // Purely aesthetic; keep utility overlays readable by suppressing detail on man-made tiles.
  if (t.overlay == Overlay::Road || t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
      t.overlay == Overlay::Industrial)
    return;

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 30.0f) return;

  const std::uint32_t base = seed32 ^ (static_cast<std::uint32_t>(t.variation) * 0x9E3779B9u);
  const std::uint32_t h0 = HashCoords32(x, y, base ^ 0xA5A5A5A5u);

  const bool isPark = (t.overlay == Overlay::Park);

  const float invZoom = 1.0f / std::max(0.001f, zoom);
  const float thick = 1.15f * invZoom;

  if (t.terrain == Terrain::Grass) {
    // Grass tufts + occasional flowers. Parks get more/larger tree canopies.
    const int tufts = std::clamp(1 + static_cast<int>((h0 >> 28) & 0x3u), 1, isPark ? 4 : 3);

    for (int i = 0; i < tufts; ++i) {
      const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0x13579BDFu, i, center, tileW, tileH, 0.82f);

      const float r = tileH * (isPark ? 0.055f : 0.035f) * (0.85f + 0.35f * Frac01(HashCoords32(x, y, base ^ (i * 0x27D4EB2Du))));

      const Color tuftC = ShadeDetail(Color{35, 115, 55, 255}, brightness, 0.95f, isPark ? 130 : 95);
      DrawCircleV(p, r, tuftC);

      // Small highlight blob so tufts read as volume.
      const Color hiC = ShadeDetail(Color{90, 190, 110, 255}, brightness, 1.05f, isPark ? 75 : 55);
      DrawCircleV(Vector2{p.x - r * 0.20f, p.y - r * 0.18f}, r * 0.55f, hiC);

      // Rare flowers on non-park grass.
      if (!isPark) {
        const std::uint32_t hf = HashCoords32(x + i * 11, y - i * 7, base ^ 0xFACEB00Cu);
        if ((hf & 0xFFu) == 0x7Bu) {
          const Color fl = ShadeDetail(Color{250, 230, 90, 255}, brightness, 1.10f, 130);
          DrawCircleV(Vector2{p.x + r * 0.25f, p.y - r * 0.10f}, r * 0.33f, fl);
        }
      }
    }

    if (isPark && tileScreenW >= 40.0f) {
      // Tree canopies: a couple of larger blobs.
      const int trees = std::clamp(1 + static_cast<int>((h0 >> 23) & 0x3u), 1, 3);
      for (int i = 0; i < trees; ++i) {
        const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0x2468ACE0u, 10 + i, center, tileW, tileH, 0.70f);
        const float r = tileH * (0.10f + 0.04f * Frac01(HashCoords32(x + i * 19, y - i * 13, base ^ 0xBADC0DEu)));
        const Color dark = ShadeDetail(Color{25, 80, 45, 255}, brightness, 0.95f, 150);
        const Color mid = ShadeDetail(Color{45, 140, 70, 255}, brightness, 1.00f, 170);
        DrawCircleV(p, r, dark);
        DrawCircleV(Vector2{p.x - r * 0.20f, p.y - r * 0.22f}, r * 0.70f, mid);
        DrawCircleV(Vector2{p.x - r * 0.35f, p.y - r * 0.34f}, r * 0.38f, ShadeDetail(Color{110, 210, 125, 255}, brightness, 1.05f, 90));
      }
    }
  } else if (t.terrain == Terrain::Sand) {
    // Pebbles + drift specks.
    const int stones = std::clamp(static_cast<int>((h0 >> 27) & 0x3u), 0, 2);
    for (int i = 0; i < stones; ++i) {
      const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0x0DDC0FFEu, i, center, tileW, tileH, 0.86f);
      const float r = tileH * (0.020f + 0.012f * Frac01(HashCoords32(x - i * 17, y + i * 9, base ^ 0xDEAD10CCu)));
      DrawCircleV(p, r, ShadeDetail(Color{120, 105, 70, 255}, brightness, 0.95f, 125));
    }

    // Occasional shell highlight.
    if ((h0 & 0x3FFu) == 0x155u) {
      const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0x51A5EEDu, 7, center, tileW, tileH, 0.75f);
      DrawLineEx(Vector2{p.x - tileH * 0.03f, p.y}, Vector2{p.x + tileH * 0.03f, p.y}, thick,
                 ShadeDetail(Color{245, 240, 230, 255}, brightness, 1.10f, 120));
    }
  } else if (t.terrain == Terrain::Water) {
    // Small specular sparkles; animate lightly so large bodies of water feel alive.
    // Skip bridges for clarity.
    if (t.overlay == Overlay::Road) return;

    const int sparkles = (tileScreenW >= 46.0f) ? 2 : 1;
    for (int i = 0; i < sparkles; ++i) {
      const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0x9A1B2C3Du, 20 + i, center, tileW, tileH, 0.80f);
      const float phase = Frac01(HashCoords32(x + i * 41, y - i * 37, base ^ 0xC001D00Du)) * 6.2831853f;
      const float pulse = 0.40f + 0.60f * (0.5f + 0.5f * std::sin(timeSec * (1.55f + 0.25f * static_cast<float>(i)) + phase));
      const unsigned char a = ClampU8(static_cast<int>(18.0f + 95.0f * pulse));

      const float len = tileH * 0.10f;
      Vector2 a0{p.x - len * 0.40f, p.y - len * 0.05f};
      Vector2 a1{p.x + len * 0.40f, p.y + len * 0.05f};

      DrawLineEx(a0, a1, thick, ShadeDetail(Color{255, 255, 255, 255}, brightness, 1.10f, a));
    }
  }
}


// ---------------------------------------------------------------------------------------------
// Weather ground effects (no external art)
// ---------------------------------------------------------------------------------------------

static void DrawWeatherGroundEffects(const World& world, int x, int y, const Tile& t, const Vector2& center,
                                    float tileW, float tileH, float zoom, float brightness,
                                    const DayNightState& dn, const WeatherState& w,
                                    float timeSec, std::uint32_t seed32)
{
  (void)world;
  (void)timeSec;

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 18.0f) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);

  // -----------------------------
  // Wet sheen on roads (rain)
  // -----------------------------
  if (w.wetness > 0.02f && t.overlay == Overlay::Road) {
    const float wet = std::clamp(w.wetness, 0.0f, 1.0f);

    const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);

    // Direction aligned to the dominant road axis.
    Vector2 dir{0.0f, 0.0f};
    if ((mask & 0x01u) != 0u) { dir.x += tileW * 0.5f; dir.y -= tileH * 0.5f; } // up-right
    if ((mask & 0x02u) != 0u) { dir.x += tileW * 0.5f; dir.y += tileH * 0.5f; } // down-right
    if ((mask & 0x04u) != 0u) { dir.x -= tileW * 0.5f; dir.y += tileH * 0.5f; } // down-left
    if ((mask & 0x08u) != 0u) { dir.x -= tileW * 0.5f; dir.y -= tileH * 0.5f; } // up-left

    float dl2 = dir.x * dir.x + dir.y * dir.y;
    if (dl2 < 1.0e-6f) {
      dir = Vector2{tileW * 0.5f, tileH * 0.5f};
      dl2 = dir.x * dir.x + dir.y * dir.y;
    }
    const float invDl = 1.0f / std::sqrt(dl2);
    dir.x *= invDl;
    dir.y *= invDl;

    Vector2 perp{-dir.y, dir.x};

    const std::uint32_t base = HashCoords32(x, y, seed32 ^ 0x91E10D7Bu);
    const int streaks = (tileScreenW >= 42.0f) ? 2 : 1;

    for (int i = 0; i < streaks; ++i) {
      const std::uint32_t h = HashCoords32(x + i * 19, y - i * 13, base ^ 0xC001D00Du);
      const float r = Frac01(h);

      // Offset within tile.
      const float off = (r - 0.5f) * tileH * 0.16f;
      const float along = (Frac01(h ^ 0xA2C2A2C2u) - 0.5f) * tileH * 0.08f;

      Vector2 p{center.x + perp.x * off + dir.x * along, center.y + perp.y * off + dir.y * along};

      const float len = tileH * (0.20f + 0.12f * r);
      Vector2 a{p.x - dir.x * len * 0.5f, p.y - dir.y * len * 0.5f};
      Vector2 b{p.x + dir.x * len * 0.5f, p.y + dir.y * len * 0.5f};

      const float nightBoost = 0.65f + 0.55f * dn.night; // wet highlights read stronger at night under lights
      const unsigned char alpha = ClampU8(static_cast<int>(12.0f + 70.0f * wet * nightBoost * (0.55f + 0.45f * r)));

      const float thick = (0.95f + 0.35f * wet) * invZoom;
      DrawLineEx(a, b, thick, ShadeDetail(Color{210, 235, 255, 255}, brightness, 1.05f, alpha));
    }

    // A tiny "puddle" spec at intersections makes wet streets pop.
    if (tileScreenW >= 44.0f) {
      const int conn = Popcount4(mask);
      if (conn >= 3) {
        const float nightBoost = 0.55f + 0.75f * dn.night;
        const unsigned char alpha = ClampU8(static_cast<int>(18.0f + 85.0f * wet * nightBoost));
        const float r = tileH * 0.045f;
        DrawCircleV(Vector2{center.x, center.y + tileH * 0.05f}, r,
                    ShadeDetail(Color{220, 240, 255, 255}, brightness, 1.05f, alpha));
      }
    }
  }

  // -----------------------------
  // Snow cover / thin ice (snow)
  // -----------------------------
  if (w.snow > 0.02f) {
    float cover = std::clamp(w.snow, 0.0f, 1.0f);

    // Less accumulation on roads, slightly less on sand.
    if (t.overlay == Overlay::Road) cover *= 0.25f;
    if (t.terrain == Terrain::Sand) cover *= 0.80f;

    const std::uint32_t hn = HashCoords32(x, y, seed32 ^ 0x05A0CAFEu);
    const float patch = 0.65f + 0.70f * Frac01(hn);
    cover *= patch;

    if (cover > 0.01f) {
      Vector2 corners[4];
      TileDiamondCorners(center, tileW, tileH, corners);

      const float nightMul = 0.78f + 0.28f * (1.0f - dn.night); // snow stays fairly visible at night
      const float aBase = 110.0f * cover * nightMul;
      unsigned char a = ClampU8(static_cast<int>(aBase));

      Color c = (t.terrain == Terrain::Water) ? Color{210, 235, 255, 255} : Color{245, 248, 255, 255};
      c = ShadeDetail(c, brightness, (t.terrain == Terrain::Water) ? 1.05f : 1.10f, a);

      // Fill diamond (two triangles).
      DrawTriangle(corners[0], corners[1], corners[2], c);
      DrawTriangle(corners[0], corners[2], corners[3], c);

      // Add subtle drift lines aligned to wind direction for texture.
      if (tileScreenW >= 32.0f) {
        Vector2 wdir{w.windX, w.windY};
        const float wl2 = wdir.x * wdir.x + wdir.y * wdir.y;
        if (wl2 > 1.0e-6f) {
          const float inv = 1.0f / std::sqrt(wl2);
          wdir.x *= inv;
          wdir.y *= inv;
        } else {
          wdir = Vector2{0.0f, 1.0f};
        }

        const int drifts = 1 + static_cast<int>(Frac01(hn ^ 0x1234567u) * 2.0f);
        for (int i = 0; i < drifts; ++i) {
          const Vector2 p = DeterministicDiamondPoint(x, y, seed32 ^ 0x51A5EEDu, 60 + i, center, tileW, tileH, 0.86f);
          const float len = tileH * (0.10f + 0.06f * Frac01(HashCoords32(x + i * 13, y - i * 11, hn ^ 0x0BADC0DEu)));
          Vector2 a0{p.x - wdir.x * len, p.y - wdir.y * len};
          Vector2 a1{p.x + wdir.x * len, p.y + wdir.y * len};

          const unsigned char la = ClampU8(static_cast<int>(12.0f + 55.0f * cover));
          DrawLineEx(a0, a1, 1.05f * invZoom, Color{255, 255, 255, la});
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------
struct TileRect {
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;

  // Diagonal traversal helpers for isometric back-to-front ordering.
  int minSum() const { return minX + minY; }
  int maxSum() const { return maxX + maxY; }
};

// Day / night emissive decals (streetlights + windows)
// ---------------------------------------------------------------------------------------------

static void DrawNightLightsPass(const World& world, const TileRect& vis,
                               float tileW, float tileH, const ElevationSettings& elev,
                               float zoom, float timeSec, float night,
                               float wetness, bool reflectLights,
                               std::uint32_t seed32)
{
  if (night <= 0.001f) return;

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 22.0f) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);

  const int minSum = vis.minX + vis.minY;
  const int maxSum = vis.maxX + vis.maxY;

  for (int sum = minSum; sum <= maxSum; ++sum) {
    const int x0 = std::max(vis.minX, sum - vis.maxY);
    const int x1 = std::min(vis.maxX, sum - vis.minY);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      if (y < vis.minY || y > vis.maxY) continue;

      const Tile& t = world.at(x, y);

      const float elevPx = TileElevationPx(t, elev);
      Vector2 center = TileToWorldCenter(x, y, tileW, tileH);
      center.y -= elevPx;

      // ------------------------------------------------------------
      // Streetlights: roads (stronger at intersections / major roads)
      // ------------------------------------------------------------
      if (t.overlay == Overlay::Road) {
        const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);
        const int conn = Popcount4(mask);

        const bool isIntersection = (conn >= 3);
        const bool isMajor = (static_cast<int>(t.level) >= 2);

        const std::uint32_t hr = HashCoords32(x, y, seed32 ^ 0xC0FFEE11u);

        bool place = isIntersection;
        if (!place && isMajor) {
          // Light every few tiles on major roads.
          place = ((hr & 0x3u) == 0u);
        }
        if (!place) continue;

        // Slight bias toward the back of the tile so the glow doesn't fight the road pips at high zoom.
        Vector2 p = center;
        p.y -= tileH * 0.10f;

        const float flicker = 0.85f +
                              0.15f * std::sin(timeSec * (1.6f + 0.3f * static_cast<float>(hr & 3u)) +
                                               Frac01(hr) * 6.2831853f);

        const float aBase = (isIntersection ? 125.0f : 95.0f) + (isMajor ? 15.0f : 0.0f);
        const int a = static_cast<int>(aBase * night * flicker);

        const float r = (isIntersection ? 7.5f : 6.0f) * invZoom;

        Color outer{255, 205, 135, ClampU8(a)};
        Color inner{255, 245, 220, ClampU8(a + 55)};

        // Bridges read better with a cooler light.
        if (t.terrain == Terrain::Water) {
          outer = Color{205, 235, 255, ClampU8(a)};
          inner = Color{235, 250, 255, ClampU8(a + 55)};
        }

        DrawGlow(p, r, outer, inner);

        if (reflectLights && wetness > 0.05f) {
          const float w = std::clamp(wetness, 0.0f, 1.0f);
          const float refLen = tileH * (0.16f + 0.10f * w);
          const float refOff = tileH * 0.04f;

          Vector2 p0 = p;
          p0.y += refOff;
          Vector2 p1 = p0;
          p1.y += refLen;

          Color refOuter = outer;
          Color refInner = inner;
          refOuter.a = ClampU8(static_cast<int>(static_cast<float>(outer.a) * (0.55f * w)));
          refInner.a = ClampU8(static_cast<int>(static_cast<float>(inner.a) * (0.35f * w)));

          // A simple elongated smear + a second faint glow reads surprisingly well as a wet reflection.
          DrawLineEx(p0, p1, r * (0.45f + 0.12f * w), refOuter);
          DrawGlow(Vector2{p.x, p.y + refLen * 0.55f}, r * (0.80f + 0.25f * w), refOuter, refInner);
        }
        continue;
      }

      // ------------------------------------------------------------
      // Windows: zoned tiles (density + occupants => more lights)
      // ------------------------------------------------------------
      if (IsZoneOverlay(t.overlay)) {
        // Avoid noise when zoomed out.
        if (tileScreenW < 28.0f) continue;

        const int cap = std::max(1, CapacityForTile(t));
        const float occ = std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f);

        float litChance = 0.10f + 0.70f * occ;
        if (t.overlay == Overlay::Commercial) litChance = 0.30f + 0.55f * occ;
        if (t.overlay == Overlay::Industrial) litChance = 0.06f + 0.35f * occ;

        const std::uint32_t hb = HashCoords32(x, y, seed32 ^ 0x5A17B00Bu);

        // Per-tile count scales with zone level; commercial tends to be brighter.
        const int baseCount = 1 + std::clamp(static_cast<int>(t.level), 1, 3) / 2;
        const int count = baseCount + ((t.overlay == Overlay::Commercial) ? 1 : 0);

        for (int i = 0; i < count; ++i) {
          const std::uint32_t hi = HashCoords32(x + i * 97, y - i * 61, hb ^ 0x9E3779B9u);
          if (Frac01(hi) > litChance) continue;

          Vector2 p = DeterministicDiamondPoint(x, y, hb ^ 0x13579BDFu, 40 + i, center, tileW, tileH, 0.55f);
          // Lift above the tile to read like building windows.
          p.y -= tileH * (0.18f + 0.06f * Frac01(hi));

          const float flicker = 0.80f +
                                0.20f * std::sin(timeSec * (1.2f + 0.25f * static_cast<float>((hi >> 6) & 7u)) +
                                                 Frac01(hi) * 6.2831853f);

          const float aBase = 85.0f + 140.0f * occ + 20.0f * static_cast<float>(t.level);
          const int a = static_cast<int>(aBase * night * flicker);

          float r = (4.2f + 1.0f * static_cast<float>((hi >> 3) & 3u)) * invZoom;
          if (t.overlay == Overlay::Commercial) r *= 1.10f;

          Color outer{255, 195, 120, ClampU8(a)};
          Color inner{255, 245, 220, ClampU8(a + 60)};

          if (t.overlay == Overlay::Commercial) {
            outer = Color{190, 235, 255, ClampU8(a)};
            inner = Color{235, 250, 255, ClampU8(a + 60)};
          } else if (t.overlay == Overlay::Industrial) {
            outer = Color{255, 170, 90, ClampU8(a)};
            inner = Color{255, 230, 200, ClampU8(a + 55)};
          }

          DrawGlow(p, r, outer, inner);
        }
      }
    }
  }
}


// ---------------------------------------------------------------------------------------------
// Terrain lighting helpers
// ---------------------------------------------------------------------------------------------

struct TileLighting {
  // Stable lighting (used for cached terrain, overlays, and anything that shouldn't shimmer).
  float base = 1.0f;

  // base + optional animated components (currently: subtle water shimmer).
  float animated = 1.0f;

  // Approximate ambient occlusion factor (0..1). Exposed mainly for debugging/tuning.
  float occlusion = 0.0f;
};

inline float Clamp01f(float v) { return std::clamp(v, 0.0f, 1.0f); }

// Convert Tile::height (0..1) into a pseudo "vertical" height in world-space pixels for lighting.
//
// We intentionally mirror the current elevation rendering knobs when they are enabled so terrain
// lighting stays consistent with terracing/flattened-water settings. When elevation is disabled,
// we still derive a reasonable vertical scale from tileH so the world doesn't look perfectly flat.
inline float VisualHeightPxForLighting(const isocity::Tile& t, const isocity::ElevationSettings& elev, float tileH)
{
  if (elev.flattenWater && t.terrain == isocity::Terrain::Water) return 0.0f;

  float h = Clamp01f(t.height);

  // If elevation rendering is enabled and quantized, mirror that so lighting matches the terraces.
  if (elev.maxPixels > 0.0f && elev.quantizeSteps > 0) {
    const float q = static_cast<float>(elev.quantizeSteps);
    h = std::round(h * q) / q;
  }

  // Use the actual elevation scale if enabled, otherwise pick a stable default.
  const float scale = (elev.maxPixels > 0.0f) ? elev.maxPixels : (tileH * 0.85f);
  return h * std::max(1.0f, scale);
}

inline float BaseTileBrightness(const isocity::Tile& t)
{
  const float v = (static_cast<float>(t.variation) / 255.0f - 0.5f) * 0.10f;
  return 0.85f + t.height * 0.30f + v;
}

inline TileLighting ComputeTileLighting(const isocity::World& world, int x, int y, float tileW, float tileH,
                                       const isocity::ElevationSettings& elev, float timeSec, bool animateWater)
{
  const isocity::Tile& t = world.at(x, y);

  float b = BaseTileBrightness(t);
  float occl = 0.0f;

  // Directional slope lighting + light ambient occlusion to make elevation read better.
  const int w = world.width();
  const int h = world.height();
  if (w > 0 && h > 0 && t.terrain != isocity::Terrain::Water) {
    auto zAt = [&](int tx, int ty) -> float {
      tx = std::clamp(tx, 0, w - 1);
      ty = std::clamp(ty, 0, h - 1);
      return VisualHeightPxForLighting(world.at(tx, ty), elev, tileH);
    };

    const float z0 = zAt(x, y);
    const float zL = zAt(x - 1, y);
    const float zR = zAt(x + 1, y);
    const float zT = zAt(x, y - 1);
    const float zB = zAt(x, y + 1);

    // Central differences.
    const float dzX = zR - zL;
    const float dzY = zB - zT;

    const float halfW = tileW * 0.5f;
    const float halfH = tileH * 0.5f;

    // Unnormalized normal from cross(vX, vY) where:
    // vX = (2*halfW, 2*halfH, dzX),  vY = (-2*halfW, 2*halfH, dzY)
    float nx = halfH * (dzY - dzX);
    float ny = -halfW * (dzX + dzY);
    float nz = 4.0f * halfW * halfH;

    const float len2 = nx * nx + ny * ny + nz * nz;
    if (len2 > 1.0e-6f) {
      const float invLen = 1.0f / std::sqrt(len2);
      nx *= invLen;
      ny *= invLen;
      nz *= invLen;

      // Sun from north-west-ish, slightly above the horizon (in world-space).
      static const Vector3 sun = []() {
        const float sx = -0.62f;
        const float sy = -0.55f;
        const float sz = 0.58f;
        const float sl = std::sqrt(sx * sx + sy * sy + sz * sz);
        return Vector3{sx / sl, sy / sl, sz / sl};
      }();

      const float flatDot = sun.z; // dot((0,0,1), sun)
      const float delta = (nx * sun.x + ny * sun.y + nz * sun.z) - flatDot;

      // Amplify slightly; slope contribution is otherwise subtle on gentle terrain.
      const float slopeStrength = 0.75f;
      b += delta * slopeStrength;
    }

    // Ambient occlusion from higher neighbors in the two "back" directions. This improves
    // cliff readability in the isometric draw order without doing expensive shadow casting.
    const float zScale = std::max(1.0f, (elev.maxPixels > 0.0f) ? elev.maxPixels : (tileH * 0.85f));
    const float d0 = std::max(0.0f, zL - z0);
    const float d1 = std::max(0.0f, zT - z0);
    occl = std::clamp((d0 + d1) / zScale, 0.0f, 1.0f);

    const float aoStrength = 0.20f;
    b *= (1.0f - occl * aoStrength);
  }

  b = std::clamp(b, 0.35f, 1.40f);

  float anim = b;
  if (animateWater && t.terrain == isocity::Terrain::Water) {
    // Subtle shimmer. Kept small so overlays (bridges/roads) don't flicker.
    anim += 0.04f * std::sin((static_cast<float>(x) * 0.35f + static_cast<float>(y) * 0.70f) + timeSec * 2.0f);
  }
  anim = std::clamp(anim, 0.35f, 1.50f);

  return TileLighting{b, anim, occl};
}

struct DiamondParams {
  float nx = 0;
  float ny = 0;
  float manhattan = 0;
  float edge = 0; // 0 at edge, 1 at center
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

// Generic diamond texture generator (RGBA) with a per-pixel callback.
template <typename Fn>
Texture2D MakeDiamondTexture(int w, int h, Fn fn)
{
  Image img = GenImageColor(w, h, BLANK);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const DiamondParams d = DiamondAtPixel(x, y, w, h);
      if (d.manhattan > 1.0f) continue;

      const Color c = fn(x, y, d);
      if (c.a == 0) continue;
      ImageDrawPixel(&img, x, y, c);
    }
  }

  Texture2D tex = LoadTextureFromImage(img);
  UnloadImage(img);
  return tex;
}

inline float Dot2(float ax, float ay, float bx, float by) { return ax * bx + ay * by; }
// Compute a conservative tile-coordinate rectangle that covers the current camera viewport.
// This is used to avoid drawing off-screen tiles (big win when panning/zooming on larger maps).
TileRect ComputeVisibleTileRect(const Camera2D& camera, int screenW, int screenH, int mapW, int mapH, float tileW,
                                float tileH, float maxElevPx)
{
  // Viewport corners in world space.
  const Vector2 s0{0.0f, 0.0f};
  const Vector2 s1{static_cast<float>(screenW), 0.0f};
  const Vector2 s2{0.0f, static_cast<float>(screenH)};
  const Vector2 s3{static_cast<float>(screenW), static_cast<float>(screenH)};

  const Vector2 w0 = GetScreenToWorld2D(s0, camera);
  const Vector2 w1 = GetScreenToWorld2D(s1, camera);
  const Vector2 w2 = GetScreenToWorld2D(s2, camera);
  const Vector2 w3 = GetScreenToWorld2D(s3, camera);

  float minWX = std::numeric_limits<float>::infinity();
  float minWY = std::numeric_limits<float>::infinity();
  float maxWX = -std::numeric_limits<float>::infinity();
  float maxWY = -std::numeric_limits<float>::infinity();

  const Vector2 ws[4] = {w0, w1, w2, w3};
  for (const Vector2& wv : ws) {
    minWX = std::min(minWX, wv.x);
    minWY = std::min(minWY, wv.y);
    maxWX = std::max(maxWX, wv.x);
    maxWY = std::max(maxWY, wv.y);
  }

  // Expand by one tile to avoid edge pop-in (dst rect extends beyond the tile center).
  minWX -= tileW;
  maxWX += tileW;
  // Elevation shifts tiles upward in world-space; expand Y bounds by the maximum elevation so we don't
  // cull tiles that still contribute visible pixels above the viewport.
  minWY -= tileH + maxElevPx;
  maxWY += tileH + maxElevPx;

  const Point a = WorldToTileApprox(Vector2{minWX, minWY}, tileW, tileH);
  const Point b = WorldToTileApprox(Vector2{maxWX, minWY}, tileW, tileH);
  const Point c = WorldToTileApprox(Vector2{minWX, maxWY}, tileW, tileH);
  const Point d = WorldToTileApprox(Vector2{maxWX, maxWY}, tileW, tileH);

  const int minTX = std::min({a.x, b.x, c.x, d.x});
  const int maxTX = std::max({a.x, b.x, c.x, d.x});
  const int minTY = std::min({a.y, b.y, c.y, d.y});
  const int maxTY = std::max({a.y, b.y, c.y, d.y});

  // Extra safety margin in tile space (camera rotations / numerical edge cases).
  const int margin = 3;

  TileRect r;
  r.minX = std::clamp(minTX - margin, 0, mapW - 1);
  r.maxX = std::clamp(maxTX + margin, 0, mapW - 1);
  r.minY = std::clamp(minTY - margin, 0, mapH - 1);
  r.maxY = std::clamp(maxTY + margin, 0, mapH - 1);
  return r;
}

struct WorldRect {
  float minX = 0.0f;
  float minY = 0.0f;
  float maxX = 0.0f;
  float maxY = 0.0f;
};

WorldRect ComputeCameraWorldAABB(const Camera2D& camera, int screenW, int screenH, float expandX, float expandY)
{
  const Vector2 s0{0.0f, 0.0f};
  const Vector2 s1{static_cast<float>(screenW), 0.0f};
  const Vector2 s2{0.0f, static_cast<float>(screenH)};
  const Vector2 s3{static_cast<float>(screenW), static_cast<float>(screenH)};

  const Vector2 w0 = GetScreenToWorld2D(s0, camera);
  const Vector2 w1 = GetScreenToWorld2D(s1, camera);
  const Vector2 w2 = GetScreenToWorld2D(s2, camera);
  const Vector2 w3 = GetScreenToWorld2D(s3, camera);

  float minWX = std::numeric_limits<float>::infinity();
  float minWY = std::numeric_limits<float>::infinity();
  float maxWX = -std::numeric_limits<float>::infinity();
  float maxWY = -std::numeric_limits<float>::infinity();

  const Vector2 ws[4] = {w0, w1, w2, w3};
  for (const Vector2& wv : ws) {
    minWX = std::min(minWX, wv.x);
    minWY = std::min(minWY, wv.y);
    maxWX = std::max(maxWX, wv.x);
    maxWY = std::max(maxWY, wv.y);
  }

  WorldRect r;
  r.minX = minWX - expandX;
  r.maxX = maxWX + expandX;
  r.minY = minWY - expandY;
  r.maxY = maxWY + expandY;
  return r;
}

struct BandBounds {
  float minX = 0.0f;
  float minY = 0.0f;
  float maxX = 0.0f;
  float maxY = 0.0f;
};

// Compute a conservative world-space AABB for an isometric diagonal band.
//
// A band is defined by a contiguous range of (x+y) sums [sum0..sum1].
// We later render this entire band into a single RenderTexture2D and draw it
// in increasing band order, which preserves the global draw ordering.
BandBounds ComputeBandBounds(int sum0, int sum1, int mapW, int mapH, float tileW, float tileH, float maxElevPx)
{
  const float halfW = tileW * 0.5f;
  const float halfH = tileH * 0.5f;

  float minCenterX = std::numeric_limits<float>::infinity();
  float maxCenterX = -std::numeric_limits<float>::infinity();

  for (int sum = sum0; sum <= sum1; ++sum) {
    const int x0 = std::max(0, sum - (mapH - 1));
    const int x1 = std::min(mapW - 1, sum);
    if (x0 > x1) continue;

    // For fixed sum, center.x = (x-y)*halfW = (2*x - sum)*halfW.
    const float cx0 = (2.0f * static_cast<float>(x0) - static_cast<float>(sum)) * halfW;
    const float cx1 = (2.0f * static_cast<float>(x1) - static_cast<float>(sum)) * halfW;

    minCenterX = std::min(minCenterX, cx0);
    maxCenterX = std::max(maxCenterX, cx1);
  }

  if (!std::isfinite(minCenterX) || !std::isfinite(maxCenterX)) {
    minCenterX = 0.0f;
    maxCenterX = 0.0f;
  }

  BandBounds b;
  b.minX = minCenterX - halfW;
  b.maxX = maxCenterX + halfW;

  // For fixed sum, base center.y = (x+y)*halfH = sum*halfH.
  // Elevation shifts tiles UP (subtract), so we subtract maxElevPx from minY.
  b.minY = static_cast<float>(sum0) * halfH - halfH - maxElevPx;
  b.maxY = static_cast<float>(sum1) * halfH + halfH;
  return b;
}

// Compute a stable screen-space destination rectangle for the minimap.
Renderer::MinimapLayout ComputeMinimapLayout(int mapW, int mapH, int screenW, int screenH)
{
  Renderer::MinimapLayout out;
  if (mapW <= 0 || mapH <= 0 || screenW <= 0 || screenH <= 0) {
    out.rect = Rectangle{0, 0, 0, 0};
    out.pixelsPerTile = 1.0f;
    return out;
  }

  const float pad = 12.0f;
  // Cap minimap size relative to the window so it stays usable across resolutions.
  const float maxSize = std::min(260.0f, std::min(static_cast<float>(screenW), static_cast<float>(screenH)) * 0.38f);
  const float denom = static_cast<float>(std::max(mapW, mapH));
  float s = (denom > 0.0f) ? (maxSize / denom) : 1.0f;
  s = std::clamp(s, 0.35f, 6.0f);

  const float w = static_cast<float>(mapW) * s;
  const float h = static_cast<float>(mapH) * s;

  out.rect = Rectangle{static_cast<float>(screenW) - pad - w, static_cast<float>(screenH) - pad - h, w, h};
  out.pixelsPerTile = s;
  return out;
}

// Determine a minimap pixel color for a tile.
Color MinimapColorForTile(const isocity::Tile& t)
{
  using namespace isocity;

  // Base terrain colors.
  Color base = Color{70, 160, 90, 255};
  if (t.terrain == Terrain::Water) base = Color{35, 90, 210, 255};
  if (t.terrain == Terrain::Sand) base = Color{195, 170, 95, 255};

  // Simple height shading: higher tiles are slightly brighter.
  const float b = std::clamp(0.70f + t.height * 0.45f, 0.35f, 1.25f);
  base = Mul(base, b);

  // Overlays: mix towards a strong color so gameplay is readable.
  switch (t.overlay) {
  case Overlay::None: return base;
  case Overlay::Road: {
    // Higher-tier roads read darker / stronger on the minimap.
    const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);

    // Roads on water are bridges; render them a bit lighter/warmer so they are readable over water.
    if (t.terrain == Terrain::Water) {
      const Color bridge = (lvl == 1) ? Color{190, 170, 125, 255}
                          : (lvl == 2) ? Color{180, 160, 118, 255}
                                       : Color{170, 152, 110, 255};
      const float k = (lvl == 1) ? 0.82f : (lvl == 2) ? 0.84f : 0.86f;
      return Lerp(base, bridge, k);
    }

    const Color road = (lvl == 1) ? Color{28, 28, 30, 255} : (lvl == 2) ? Color{24, 24, 28, 255} : Color{20, 20, 25, 255};
    const float k = (lvl == 1) ? 0.85f : (lvl == 2) ? 0.88f : 0.90f;
    return Lerp(base, road, k);
  }
  case Overlay::Park: return Lerp(base, Color{70, 200, 95, 255}, 0.70f);
  case Overlay::Residential: return Lerp(base, Color{80, 160, 235, 255}, 0.80f);
  case Overlay::Commercial: return Lerp(base, Color{240, 170, 60, 255}, 0.80f);
  case Overlay::Industrial: return Lerp(base, Color{200, 90, 220, 255}, 0.80f);
  default: break;
  }
  return base;
}

// Draw a simple extruded "building" for zone tiles.
void DrawZoneBuilding(const isocity::Tile& t, float tileW, float tileH, float zoom, const Vector2& tileCenter,
                      float tileBrightness)
{
  using namespace isocity;

  const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                       t.overlay == Overlay::Industrial);
  if (!isZone) return;

  // Fade out when zoomed out.
  if (tileW * zoom < 26.0f) return;

  const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);

  int cap = 0;
  float baseShrink = 0.54f;
  float heightMul = 1.0f;
  Color baseColor = Color{210, 210, 210, 255};

  if (t.overlay == Overlay::Residential) {
    cap = CapacityForOverlayLevel(t.overlay, lvl);
    baseShrink = 0.58f;
    heightMul = 1.00f;
    baseColor = Color{200, 220, 255, 255};
  } else if (t.overlay == Overlay::Commercial) {
    cap = CapacityForOverlayLevel(t.overlay, lvl);
    baseShrink = 0.50f;
    heightMul = 1.40f;
    baseColor = Color{255, 220, 170, 255};
  } else if (t.overlay == Overlay::Industrial) {
    cap = CapacityForOverlayLevel(t.overlay, lvl);
    baseShrink = 0.62f;
    heightMul = 0.95f;
    baseColor = Color{230, 210, 255, 255};
  }

  const float occRatio = (cap > 0) ? std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f)
                                   : 0.0f;
  const float var = static_cast<float>((t.variation >> 4) & 0x0Fu) / 15.0f;

  // Height is primarily driven by level + occupancy, with some stable per-tile variety.
  float heightPx = tileH * (0.55f + 0.35f * static_cast<float>(lvl) + 0.35f * occRatio);
  heightPx *= heightMul;
  heightPx *= 0.85f + 0.35f * var;

  // Clamp so we don't create skyscrapers that overlap too aggressively.
  heightPx = std::clamp(heightPx, tileH * 0.60f, tileH * 4.25f);

  Vector2 diamond[4];
  TileDiamondCorners(tileCenter, tileW, tileH, diamond);

  Vector2 base[4];
  for (int i = 0; i < 4; ++i) {
    base[i].x = tileCenter.x + (diamond[i].x - tileCenter.x) * baseShrink;
    base[i].y = tileCenter.y + (diamond[i].y - tileCenter.y) * baseShrink;
  }

  Vector2 top[4];
  for (int i = 0; i < 4; ++i) {
    top[i] = base[i];
    top[i].y -= heightPx;
  }

  // Contact shadow: a subtle darkened footprint to anchor the building to the ground.
  {
    const float shadowShrink = std::min(0.98f, baseShrink * 1.10f);
    Vector2 shadow[4];
    for (int i = 0; i < 4; ++i) {
      shadow[i].x = tileCenter.x + (diamond[i].x - tileCenter.x) * shadowShrink;
      shadow[i].y = tileCenter.y + (diamond[i].y - tileCenter.y) * shadowShrink;
    }

    const int a = static_cast<int>(std::lround(20.0f + 12.0f * static_cast<float>(lvl) + 26.0f * occRatio));
    const Color sc = Color{0, 0, 0, ClampU8(a)};
    DrawTriangle(shadow[0], shadow[1], shadow[2], sc);
    DrawTriangle(shadow[0], shadow[2], shadow[3], sc);
  }

  // Per-face shading.
  const float b = std::clamp(tileBrightness, 0.35f, 1.35f);
  const Color topC = Mul(baseColor, 1.10f * b);
  const Color rightC = Mul(baseColor, 0.85f * b);
  const Color leftC = Mul(baseColor, 0.70f * b);

  // Right face: edge 1-2.
  DrawTriangle(base[1], base[2], top[2], rightC);
  DrawTriangle(base[1], top[2], top[1], rightC);

  // Left face: edge 2-3.
  DrawTriangle(base[3], base[2], top[2], leftC);
  DrawTriangle(base[3], top[2], top[3], leftC);

  // Roof (top face) drawn last.
  DrawTriangle(top[0], top[1], top[2], topC);
  DrawTriangle(top[0], top[2], top[3], topC);

  // Procedural facade / roof detail (purely geometric; no external assets).
  if (tileW * zoom >= 44.0f) {
    const float invZoom = 1.0f / std::max(0.001f, zoom);
    const float thick = 1.00f * invZoom;

    const std::uint32_t seed = 0xC0FFEEu ^ (static_cast<std::uint32_t>(t.variation) * 0x9E3779B9u) ^
                               (static_cast<std::uint32_t>(t.level) * 0x85EBCA6Bu);

    auto drawRows = [&](const Vector2& b0, const Vector2& b1, const Vector2& t0, const Vector2& t1, int rows,
                        Color c) {
      for (int r = 0; r < rows; ++r) {
        const float tt = (static_cast<float>(r) + 1.0f) / (static_cast<float>(rows) + 1.0f);
        Vector2 p0 = LerpV(b0, t0, tt);
        Vector2 p1 = LerpV(b1, t1, tt);

        // Slight inset so we don't scribble over the silhouette edges.
        Vector2 q0 = LerpV(p0, p1, 0.12f);
        Vector2 q1 = LerpV(p0, p1, 0.88f);

        // Light stochastic skipping so patterns don't look perfectly uniform.
        const std::uint32_t hr = HashCoords32(r, rows, seed ^ 0x51A5EEDu);
        if ((hr & 0x3u) == 0u) continue;
        DrawLineEx(q0, q1, thick, c);
      }
    };

    const float bclamp = std::clamp(tileBrightness, 0.45f, 1.30f);
    const int rows = std::clamp(static_cast<int>(std::round(heightPx / (tileH * 0.38f))), 2, 7);

    if (t.overlay == Overlay::Commercial) {
      Color wc = Color{255, 255, 255, ClampU8(static_cast<int>(22.0f + 70.0f * bclamp))};
      wc = Mul(wc, 0.90f);
      drawRows(base[1], base[2], top[1], top[2], rows, wc);
      drawRows(base[3], base[2], top[3], top[2], rows - 1, wc);
    } else if (t.overlay == Overlay::Residential) {
      Color rc = Color{255, 255, 255, ClampU8(static_cast<int>(14.0f + 45.0f * bclamp))};
      rc = Mul(rc, 0.70f);
      drawRows(base[1], base[2], top[1], top[2], rows - 1, rc);
    } else if (t.overlay == Overlay::Industrial) {
      Color ic = Color{0, 0, 0, ClampU8(static_cast<int>(18.0f + 55.0f * bclamp))};
      drawRows(base[1], base[2], top[1], top[2], rows - 1, ic);
      drawRows(base[3], base[2], top[3], top[2], rows - 2, ic);
    }

    // Simple roof equipment silhouettes (vents/chimneys) for extra variety at high zoom.
    if (tileW * zoom >= 58.0f) {
      Vector2 roofCenter{(top[0].x + top[2].x) * 0.5f, (top[0].y + top[2].y) * 0.5f};
      Vector2 v[4];
      for (int i = 0; i < 4; ++i) v[i] = Vector2{top[i].x - roofCenter.x, top[i].y - roofCenter.y};

      const int items = (t.overlay == Overlay::Industrial) ? 2 : 1;
      const float scale = (t.overlay == Overlay::Commercial) ? 0.22f : 0.18f;

      for (int i = 0; i < items; ++i) {
        const float ox = 0.22f - 0.24f * static_cast<float>(i);
        const float oy = 0.12f + 0.10f * Frac01(HashCoords32(i, rows, seed ^ 0xBADC0DEu));

        Vector2 c = Vector2{roofCenter.x + v[0].x * ox + v[3].x * oy, roofCenter.y + v[0].y * ox + v[3].y * oy};
        Vector2 d[4];
        for (int k = 0; k < 4; ++k) d[k] = Vector2{c.x + v[k].x * scale, c.y + v[k].y * scale};

        Color dc = (t.overlay == Overlay::Commercial) ? Color{35, 35, 35, 120} : Color{25, 25, 25, 135};
        dc = Mul(dc, 0.85f * bclamp);
        DrawTriangle(d[0], d[1], d[2], dc);
        DrawTriangle(d[0], d[2], d[3], dc);
      }
    }
  }

  // Optional small roof highlight at high zoom.
  if (tileW * zoom >= 46.0f) {
    const float invZoom = 1.0f / std::max(0.001f, zoom);
    const float thick = 1.0f * invZoom;
    const Color lc = Color{0, 0, 0, 70};
    DrawLineEx(top[0], top[1], thick, lc);
    DrawLineEx(top[1], top[2], thick, lc);
    DrawLineEx(top[2], top[3], thick, lc);
    DrawLineEx(top[3], top[0], thick, lc);
  }
}


// Returns distance from P(px,py) to segment AB. Also returns the projected t in [0,1].
float DistPointSegment(float px, float py, float ax, float ay, float bx, float by, float& outT)
{
  const float vx = bx - ax;
  const float vy = by - ay;
  const float wx = px - ax;
  const float wy = py - ay;

  const float len2 = vx * vx + vy * vy;
  float t = 0.0f;
  if (len2 > 1.0e-6f) t = Dot2(wx, wy, vx, vy) / len2;
  t = std::clamp(t, 0.0f, 1.0f);
  outT = t;

  const float cx = ax + t * vx;
  const float cy = ay + t * vy;
  const float dx = px - cx;
  const float dy = py - cy;
  return std::sqrt(dx * dx + dy * dy);
}

} // namespace


namespace {

// Compute diamond center from 4-corner representation (top, right, bottom, left).
inline Vector2 DiamondCenter(const Vector2 d[4])
{
  return Vector2{(d[0].x + d[2].x) * 0.5f, (d[0].y + d[2].y) * 0.5f};
}

inline void CopyDiamond(Vector2 dst[4], const Vector2 src[4])
{
  for (int i = 0; i < 4; ++i) dst[i] = src[i];
}

inline void ShrinkDiamond(Vector2 out[4], const Vector2 in[4], float factor)
{
  const Vector2 c = DiamondCenter(in);
  for (int i = 0; i < 4; ++i) {
    out[i] = Vector2{Lerp(c.x, in[i].x, factor), Lerp(c.y, in[i].y, factor)};
  }
}

inline Vector2 TileCornerAtBaseElevation(int tx, int ty, float tileW, float tileH, float baseElevPx,
                                        int cornerIndex)
{
  Vector2 c = TileToWorldCenter(tx, ty, tileW, tileH);
  c.y -= baseElevPx;

  Vector2 d[4];
  TileDiamondCorners(c, tileW, tileH, d);
  return d[cornerIndex];
}

inline void DrawIsoPrism(const Vector2 base[4], float heightPx, Color baseColor, float brightness, float tierShade,
                         float zoom)
{
  Vector2 top[4];
  for (int i = 0; i < 4; ++i) {
    top[i] = base[i];
    top[i].y -= heightPx;
  }

  const float b = std::clamp(brightness * tierShade, 0.35f, 1.35f);

  const Color cTop = Mul(baseColor, 1.10f * b);
  const Color cSideR = Mul(baseColor, 0.85f * b);
  const Color cSideL = Mul(baseColor, 0.70f * b);

  // Right face (edge 1-2).
  DrawTriangle(base[1], base[2], top[2], cSideR);
  DrawTriangle(base[1], top[2], top[1], cSideR);

  // Left face (edge 2-3).
  DrawTriangle(base[2], base[3], top[3], cSideL);
  DrawTriangle(base[2], top[3], top[2], cSideL);

  // Roof.
  DrawTriangle(top[0], top[1], top[2], cTop);
  DrawTriangle(top[0], top[2], top[3], cTop);

  // Subtle outline at higher zoom.
  if (zoom >= 0.85f) {
    const float thick = 0.85f * std::clamp(zoom, 0.75f, 1.8f);
    const Color oc = Color{0, 0, 0, 55};

    DrawLineEx(top[0], top[1], thick, oc);
    DrawLineEx(top[1], top[2], thick, oc);
    DrawLineEx(top[2], top[3], thick, oc);
    DrawLineEx(top[3], top[0], thick, oc);
  }
}

inline void DrawIsoFacadePattern(const Vector2 base[4], const Vector2 top[4], Overlay overlay, float brightness,
                                 float tierShade, float zoom, std::uint32_t seed)
{
  // Only when zoomed in enough that linework reads well.
  if (zoom < 1.05f) return;

  // These patterns are subtle and are meant to read as "windows" / "panel seams".
  // Thickness is kept roughly constant in screen-space.
  const float invZoom = 1.0f / std::max(0.001f, zoom);
  const float thick = 0.95f * invZoom;

  const float b = std::clamp(brightness * tierShade, 0.45f, 1.30f);

  const float heightPx = std::max(0.0f, base[0].y - top[0].y);
  int rows = std::clamp(static_cast<int>(std::round(heightPx / 14.0f)), 2, 8);
  if (overlay == Overlay::Residential) rows = std::max(1, rows - 2);
  if (overlay == Overlay::Industrial) rows = std::max(1, rows - 1);

  auto drawRows = [&](const Vector2& b0, const Vector2& b1, const Vector2& t0, const Vector2& t1, int rcount,
                      Color c, std::uint32_t salt) {
    if (rcount <= 0) return;
    for (int r = 0; r < rcount; ++r) {
      const float tt = (static_cast<float>(r) + 1.0f) / (static_cast<float>(rcount) + 1.0f);
      Vector2 p0 = LerpV(b0, t0, tt);
      Vector2 p1 = LerpV(b1, t1, tt);

      // Inset so edges stay crisp.
      const Vector2 q0 = LerpV(p0, p1, 0.10f);
      const Vector2 q1 = LerpV(p0, p1, 0.90f);

      // Skip some rows to avoid perfect grids.
      const std::uint32_t hr = HashCoords32(r, rcount, seed ^ salt);
      if ((hr & 0x3u) == 0u) continue;
      DrawLineEx(q0, q1, thick, c);
    }
  };

  if (overlay == Overlay::Commercial) {
    Color c = Color{255, 255, 255, ClampU8(static_cast<int>(18.0f + 75.0f * b))};
    c = Mul(c, 0.90f);
    drawRows(base[1], base[2], top[1], top[2], rows, c, 0x51A5EEDu);
    drawRows(base[3], base[2], top[3], top[2], std::max(1, rows - 1), c, 0x8BADF00Du);

    // Occasional vertical mullions for larger buildings.
    if (zoom >= 1.35f && rows >= 4) {
      const int cols = 2 + static_cast<int>((seed >> 24) & 0x1u);
      for (int cidx = 0; cidx < cols; ++cidx) {
        const float tt = (static_cast<float>(cidx) + 1.0f) / (static_cast<float>(cols) + 1.0f);
        Vector2 a0 = LerpV(base[1], base[2], tt);
        Vector2 a1 = LerpV(top[1], top[2], tt);
        DrawLineEx(a0, a1, thick, Color{c.r, c.g, c.b, ClampU8(static_cast<int>(0.55f * static_cast<float>(c.a)))});
      }
    }
  } else if (overlay == Overlay::Residential) {
    Color c = Color{255, 255, 255, ClampU8(static_cast<int>(12.0f + 50.0f * b))};
    c = Mul(c, 0.65f);
    drawRows(base[1], base[2], top[1], top[2], std::max(1, rows), c, 0xA11CE5EDu);
  } else if (overlay == Overlay::Industrial) {
    Color c = Color{0, 0, 0, ClampU8(static_cast<int>(16.0f + 60.0f * b))};
    drawRows(base[1], base[2], top[1], top[2], std::max(1, rows), c, 0xDEADC0DEu);
    drawRows(base[3], base[2], top[3], top[2], std::max(1, rows - 1), c, 0xC0FFEEu);
  }
}

} // namespace

static void DrawMergedZoneBuildingAndIndicators(const ZoneBuildingParcel& p, const World& world, const ElevationSettings& elev,
                                               float tileW, float tileH, float zoom, float timeSec)
{
  if (!p.isMultiTile()) return;
  if (!IsZoneOverlay(p.overlay)) return;
  if (tileW * zoom < 26.0f) return;

  const int lvl = ClampZoneLevel(p.level);

  const int pw = std::max(1, p.w);
  const int ph = std::max(1, p.h);
  const int area = std::max(1, pw * ph);
  const float scale = std::sqrt(static_cast<float>(area));
  const float maxDim = static_cast<float>(std::max(pw, ph));

  const int cap = std::max(0, p.capacity);
  const int occ = std::clamp(p.occupants, 0, cap);
  const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;
  const float var = static_cast<float>((p.styleSeed >> 4) & 0x0Fu) / 15.0f;

  // Base appearance defaults.
  float heightMul = 1.0f;
  Color baseColor = Color{210, 210, 210, 255};

  // k controls how much we shrink the footprint inward per parcel size (larger parcels shrink less).
  float shrinkK = 0.46f;

  if (p.overlay == Overlay::Residential) {
    baseColor = Color{75, 145, 245, 255};
    heightMul = 1.10f;
    shrinkK = 0.42f;
  } else if (p.overlay == Overlay::Commercial) {
    baseColor = Color{70, 230, 140, 255};
    heightMul = 1.40f;
    shrinkK = 0.46f;
  } else if (p.overlay == Overlay::Industrial) {
    baseColor = Color{250, 195, 55, 255};
    heightMul = 0.95f;
    shrinkK = 0.38f;
  }

  // Determine max elevation across the parcel for a stable base plane.
  float baseElevPx = 0.0f;
  float brightnessSum = 0.0f;
  int tileCount = 0;

  for (int yy = p.y0; yy < p.y0 + ph; ++yy) {
    for (int xx = p.x0; xx < p.x0 + pw; ++xx) {
      const Tile& t = world.at(xx, yy);
      baseElevPx = std::max(baseElevPx, TileElevationPx(t, elev));
      brightnessSum += ComputeTileLighting(world, xx, yy, tileW, tileH, elev, timeSec, false).base;
      ++tileCount;
    }
  }

  const float brightness = (tileCount > 0) ? (brightnessSum / static_cast<float>(tileCount)) : 1.0f;

  // Footprint outer corners.
  const int x0 = p.x0;
  const int y0 = p.y0;
  const int x1 = p.x0 + pw - 1;
  const int y1 = p.y0 + ph - 1;

  Vector2 outer[4];
  outer[0] = TileCornerAtBaseElevation(x0, y0, tileW, tileH, baseElevPx, 0); // top
  outer[1] = TileCornerAtBaseElevation(x1, y0, tileW, tileH, baseElevPx, 1); // right
  outer[2] = TileCornerAtBaseElevation(x1, y1, tileW, tileH, baseElevPx, 2); // bottom
  outer[3] = TileCornerAtBaseElevation(x0, y1, tileW, tileH, baseElevPx, 3); // left

  // Inset the base for a nicer margin.
  // Larger parcels shrink slightly less so big footprints don't look overly thin.
  const float baseShrink = std::clamp(1.0f - shrinkK / std::max(1.0f, scale), 0.55f, 0.94f);

  Vector2 base[4];
  ShrinkDiamond(base, outer, baseShrink);

  // Contact shadow under the full footprint to anchor the building to the ground.
  {
    const float shadowShrink = std::min(0.98f, baseShrink * 1.06f);
    Vector2 shadow[4];
    ShrinkDiamond(shadow, outer, shadowShrink);

    const int a = static_cast<int>(std::lround(24.0f + 10.0f * static_cast<float>(lvl) + 22.0f * occRatio));
    const Color sc = Color{0, 0, 0, ClampU8(a)};
    DrawTriangle(shadow[0], shadow[1], shadow[2], sc);
    DrawTriangle(shadow[0], shadow[2], shadow[3], sc);
  }

  // Height model (scaled by footprint).
  float heightPx = tileH * (0.55f + 0.65f * static_cast<float>(lvl));
  heightPx += tileH * (0.25f + 0.45f * static_cast<float>(lvl)) * occRatio;

  const float footprintMul = 1.0f + 0.32f * std::max(0.0f, scale - 1.0f);
  heightPx *= heightMul * footprintMul;
  heightPx *= 0.85f + 0.35f * var;

  const float maxH = tileH * (4.25f + 2.25f * std::max(0.0f, scale - 1.0f));
  heightPx = std::clamp(heightPx, tileH * 0.65f, maxH);

  // Tiered silhouettes.
  int tiers = 1;
  if (p.overlay == Overlay::Commercial && lvl >= 2) {
    tiers = (area >= 7) ? 3 : 2;
  } else if (p.overlay == Overlay::Residential && lvl >= 3 && area >= 4) {
    tiers = 2;
  } else if (p.overlay == Overlay::Industrial && lvl >= 3 && area >= 4) {
    tiers = 2;
  }

  float w0 = 1.0f;
  float w1 = 0.0f;
  float w2 = 0.0f;
  if (tiers == 2) {
    w0 = 0.68f;
    w1 = 0.32f;
  } else if (tiers == 3) {
    w0 = 0.62f;
    w1 = 0.26f;
    w2 = 0.12f;
  }

  Vector2 curBase[4];
  CopyDiamond(curBase, base);

  Vector2 top[4];
  CopyDiamond(top, base);

  const float tierShrink = (p.overlay == Overlay::Commercial) ? 0.80f : 0.76f;

  for (int ti = 0; ti < tiers; ++ti) {
    const float w = (ti == 0) ? w0 : ((ti == 1) ? w1 : w2);
    const float hTier = heightPx * w;
    const float shade = 1.0f + 0.06f * static_cast<float>(ti);

    DrawIsoPrism(curBase, hTier, baseColor, brightness, shade, zoom);

    for (int i = 0; i < 4; ++i) {
      top[i] = curBase[i];
      top[i].y -= hTier;
    }

    DrawIsoFacadePattern(curBase, top, p.overlay, brightness, shade, zoom,
                         p.styleSeed ^ (static_cast<std::uint32_t>(ti) * 0x9E3779B9u));

    if (ti + 1 < tiers) {
      Vector2 nextBase[4];
      ShrinkDiamond(nextBase, top, tierShrink);
      CopyDiamond(curBase, nextBase);
    }
  }

  // Roof details on the final top footprint.
  const Vector2 roofCenter = DiamondCenter(top);

  if (p.overlay == Overlay::Residential) {
    // Simple pyramid roof.
    const float roofH = tileH * (0.30f + 0.18f * var);
    Vector2 peak = roofCenter;
    peak.y -= roofH;

    const Color roofC = Mul(baseColor, 1.20f * std::clamp(brightness, 0.55f, 1.20f));
    DrawTriangle(top[0], top[1], peak, roofC);
    DrawTriangle(top[1], top[2], peak, roofC);
    DrawTriangle(top[2], top[3], peak, roofC);
    DrawTriangle(top[3], top[0], peak, roofC);
  } else if (p.overlay == Overlay::Industrial) {
    // Add one or two chimney stacks.
    const int chimneys = std::clamp(1 + area / 6 + static_cast<int>((p.styleSeed >> 22) & 0x1u), 1, 4);
    const float chimScale = 0.18f + 0.02f * var;
    const float chimH = tileH * (0.55f + 0.35f * var);

    Vector2 v[4];
    for (int i = 0; i < 4; ++i) v[i] = Vector2{top[i].x - roofCenter.x, top[i].y - roofCenter.y};

    for (int ci = 0; ci < chimneys; ++ci) {
      const float ox = ((ci == 0) ? 0.28f : -0.18f) * (0.9f + 0.2f * var);
      const float oy = ((ci == 0) ? 0.10f : 0.22f) * (0.9f + 0.2f * var);

      Vector2 chimCenter = Vector2{roofCenter.x + v[0].x * ox + v[3].x * oy, roofCenter.y + v[0].y * ox + v[3].y * oy};

      Vector2 chimBase[4];
      for (int i = 0; i < 4; ++i) {
        chimBase[i] = Vector2{chimCenter.x + v[i].x * chimScale, chimCenter.y + v[i].y * chimScale};
      }

      DrawIsoPrism(chimBase, chimH, baseColor, brightness, 0.65f, zoom);
    }
  } else if (p.overlay == Overlay::Commercial) {
    // Rooftop billboard along a footprint edge. Orient it using the parcel facing so
    // blocks tend to "present" toward nearby roads.
    const float pulse = 0.5f + 0.5f * std::sin(timeSec * 3.0f + static_cast<float>(p.styleSeed & 0xFFu) * 0.06f);
    const unsigned char a = ClampU8(70 + static_cast<int>(150.0f * pulse));
    const Color signC = Color{255, 255, 255, a};

    const float signH = tileH * (0.55f + 0.20f * var);

    int i0 = 0;
    int i1 = 1;
    switch (p.facing & 3u) {
    default:
    case 0: // N
      i0 = 0;
      i1 = 1;
      break;
    case 1: // E
      i0 = 1;
      i1 = 2;
      break;
    case 2: // S
      i0 = 2;
      i1 = 3;
      break;
    case 3: // W
      i0 = 3;
      i1 = 0;
      break;
    }

    Vector2 a0 = Vector2{Lerp(top[i0].x, top[i1].x, 0.22f), Lerp(top[i0].y, top[i1].y, 0.22f)};
    Vector2 b0 = Vector2{Lerp(top[i0].x, top[i1].x, 0.78f), Lerp(top[i0].y, top[i1].y, 0.78f)};
    Vector2 a1 = a0;
    Vector2 b1 = b0;
    a1.y -= signH;
    b1.y -= signH;

    DrawTriangle(a0, b0, b1, signC);
    DrawTriangle(a0, b1, a1, signC);

    if (zoom >= 1.10f) {
      const float thick = 0.85f * std::clamp(zoom, 0.75f, 1.8f);
      const Color oc = Color{0, 0, 0, 55};
      DrawLineEx(a0, b0, thick, oc);
      DrawLineEx(a1, b1, thick, oc);
      DrawLineEx(a0, a1, thick, oc);
      DrawLineEx(b0, b1, thick, oc);
    }
  }

  // Indicators (aggregate across the parcel) when zoomed in.
  const float tileScreenW = tileW * zoom;
  if (tileScreenW >= 28.0f) {
    const float invZoom = 1.0f / std::max(0.001f, zoom);

    Vector2 anchorCenter = TileToWorldCenter(x1, y1, tileW, tileH);
    anchorCenter.y -= baseElevPx;

    const float span = std::max(0.0f, maxDim - 1.0f);
    const float yInd = anchorCenter.y - tileH * (0.18f + 0.07f * span);

    // Pips:
    const float pipR = 2.0f * invZoom;
    const float pipGap = 5.0f * invZoom;
    for (int i = 0; i < lvl; ++i) {
      const float px = anchorCenter.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
      DrawCircleV(Vector2{px, yInd}, pipR, Color{0, 0, 0, 100});
    }

    // Fill bar:
    const float barW = tileW * (0.42f + 0.12f * span) * invZoom;
    const float barH = 3.0f * invZoom;
    const float barX = anchorCenter.x - barW * 0.5f;
    const float barY = yInd + 5.0f * invZoom;
    DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
    DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
  }
}


// Draw per-tile zone indicators (level pips + occupancy bar) when zoomed in.
// Used for single-tile buildings and as a defensive fallback when parcelization is disabled.
static void DrawZoneTileIndicators(const Tile& t, float tileW, float tileH, float zoom, const Vector2& center)
{
  if (!IsZoneOverlay(t.overlay)) return;

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 28.0f) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);

  const int lvl = ClampZoneLevel(t.level);
  const int cap = CapacityForOverlayLevel(t.overlay, lvl);
  const int occ = std::clamp(static_cast<int>(t.occupants), 0, cap);
  const float occRatio = (cap > 0) ? (static_cast<float>(occ) / static_cast<float>(cap)) : 0.0f;

  const float yInd = center.y - tileH * 0.18f;

  // Pips:
  const float pipR = 2.0f * invZoom;
  const float pipGap = 5.0f * invZoom;
  for (int i = 0; i < lvl; ++i) {
    const float px = center.x - (static_cast<float>(lvl - 1) * 0.5f - static_cast<float>(i)) * pipGap;
    DrawCircleV(Vector2{px, yInd}, pipR, Color{0, 0, 0, 100});
  }

  // Fill bar:
  const float barW = tileW * 0.42f * invZoom;
  const float barH = 3.0f * invZoom;
  const float barX = center.x - barW * 0.5f;
  const float barY = yInd + 5.0f * invZoom;
  DrawRectangleV(Vector2{barX, barY}, Vector2{barW, barH}, Color{0, 0, 0, 90});
  DrawRectangleV(Vector2{barX, barY}, Vector2{barW * occRatio, barH}, Color{255, 255, 255, 170});
}


// Road indicators: show small pips for upgraded road class (2..3) when zoomed in.
static void DrawRoadIndicators(const Tile& t, float tileW, float tileH, float zoom, const Vector2& tileCenter, float tileBrightness)
{
  (void)tileBrightness; // reserved for future shading tweaks

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 28.0f) return;

  const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
  if (lvl <= 1) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);
  const float y0 = tileCenter.y - tileH * 0.02f;

  const float pip = 4.0f * invZoom;
  const float gap = 1.5f * invZoom;
  const float groupW = pip * 3.0f + gap * 2.0f;
  const float groupX0 = tileCenter.x - groupW * 0.5f;

  for (int i = 0; i < 3; ++i) {
    Rectangle r{groupX0 + static_cast<float>(i) * (pip + gap), y0, pip, pip};
    DrawRectangleRec(r, Color{0, 0, 0, 110});
    DrawRectangleLinesEx(r, 1.0f * invZoom, Color{255, 255, 255, 55});
    if (i < lvl) {
      Rectangle f{r.x + 1.0f * invZoom, r.y + 1.0f * invZoom, r.width - 2.0f * invZoom, r.height - 2.0f * invZoom};
      DrawRectangleRec(f, Color{255, 255, 255, 160});
    }
  }
}

Renderer::Renderer(int tileW, int tileH, std::uint64_t seed)
    : m_tileW(tileW)
    , m_tileH(tileH)
{
  // Default to flat rendering; Game can enable elevation via setElevationSettings().
  m_elev.maxPixels = 0.0f;
  m_elev.quantizeSteps = 16;
  m_elev.flattenWater = true;
  rebuildTextures(seed);
}

Renderer::~Renderer() { unloadTextures(); }

void Renderer::unloadTextures()
{
  for (auto& tv : m_terrainTex) {
    for (auto& t : tv) {
      if (t.id != 0) UnloadTexture(t);
      t = Texture2D{};
    }
  }
  for (auto& t : m_overlayTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
  for (auto& lv : m_roadTex) {
    for (auto& mv : lv) {
      for (auto& t : mv) {
        if (t.id != 0) UnloadTexture(t);
        t = Texture2D{};
      }
    }
  }
  for (auto& lv : m_bridgeTex) {
    for (auto& mv : lv) {
      for (auto& t : mv) {
        if (t.id != 0) UnloadTexture(t);
        t = Texture2D{};
      }
    }
  }

  if (m_cloudShadowTex.id != 0) {
    UnloadTexture(m_cloudShadowTex);
    m_cloudShadowTex = Texture2D{};
  }

  unloadVehicleSprites();

  unloadBaseCache();

  unloadMinimap();
}


void Renderer::unloadVehicleSprites()
{
  auto unloadVec = [](std::vector<VehicleSprite>& v) {
    for (auto& s : v) {
      if (s.color.id != 0) UnloadTexture(s.color);
      if (s.emissive.id != 0) UnloadTexture(s.emissive);
      s = VehicleSprite{};
    }
    v.clear();
  };

  unloadVec(m_vehicleCarPosSlope);
  unloadVec(m_vehicleCarNegSlope);
  unloadVec(m_vehicleTruckPosSlope);
  unloadVec(m_vehicleTruckNegSlope);
}


const Renderer::VehicleSprite* Renderer::carSprite(bool slopePositive, int style) const
{
  const auto& primary = slopePositive ? m_vehicleCarPosSlope : m_vehicleCarNegSlope;
  const auto& fallback = slopePositive ? m_vehicleCarNegSlope : m_vehicleCarPosSlope;
  const auto& v = !primary.empty() ? primary : fallback;
  if (v.empty()) return nullptr;
  const std::uint32_t u = static_cast<std::uint32_t>(style);
  const std::size_t idx = static_cast<std::size_t>(u % static_cast<std::uint32_t>(v.size()));
  return &v[idx];
}

const Renderer::VehicleSprite* Renderer::truckSprite(bool slopePositive, int style) const
{
  const auto& primary = slopePositive ? m_vehicleTruckPosSlope : m_vehicleTruckNegSlope;
  const auto& fallback = slopePositive ? m_vehicleTruckNegSlope : m_vehicleTruckPosSlope;
  const auto& v = !primary.empty() ? primary : fallback;
  if (v.empty()) return nullptr;
  const std::uint32_t u = static_cast<std::uint32_t>(style);
  const std::size_t idx = static_cast<std::size_t>(u % static_cast<std::uint32_t>(v.size()));
  return &v[idx];
}

float Renderer::nightFactor(float timeSec) const
{
  const DayNightState st = ComputeDayNightState(timeSec, m_dayNight);
  return st.night;
}

void Renderer::rebuildVehicleSprites()
{
  unloadVehicleSprites();

  // Small sprites for the traffic micro-sim overlay (decoupled from tile resolution).
  const int sprW = std::max(24, m_tileW / 3);
  const int sprH = std::max(12, m_tileH / 3);

  GfxPropsConfig cfg{};
  cfg.tileW = sprW;
  cfg.tileH = sprH;
  cfg.includeEmissive = true;

  // Use the palette system for vehicle paint materials (keeps the project asset-free while
  // still looking coherent across seeds).
  const GfxPalette pal = GenerateGfxPalette(m_gfxSeed32 ^ 0xB16B00B5u, GfxTheme::Classic);

  auto loadTex = [](const RgbaImage& src) -> Texture2D {
    Texture2D t{};
    if (src.width <= 0 || src.height <= 0) return t;
    if (src.rgba.empty()) return t;
    Image img = ImageFromRgbaImage(src);
    t = LoadTextureFromImage(img);
    UnloadImage(img);
    if (t.id != 0) SetTextureFilter(t, TEXTURE_FILTER_POINT);
    return t;
  };

  auto buildKind = [&](GfxPropKind kind, std::vector<VehicleSprite>& pos, std::vector<VehicleSprite>& neg) {
    constexpr int kWantPerSlope = 8;
    constexpr int kMaxTrials = 64;

    std::string err;
    for (int variant = 0; variant < kMaxTrials; ++variant) {
      if (static_cast<int>(pos.size()) >= kWantPerSlope && static_cast<int>(neg.size()) >= kWantPerSlope) break;

      GfxPropSprite spr{};
      if (!GenerateGfxPropSprite(kind, variant, m_gfxSeed32, cfg, pal, spr, err)) continue;

      VehicleSprite vs{};
      vs.pivotX = spr.pivotX;
      vs.pivotY = spr.pivotY;
      vs.color = loadTex(spr.color);
      if (!spr.emissive.rgba.empty()) vs.emissive = loadTex(spr.emissive);

      if (vs.color.id == 0) {
        if (vs.emissive.id != 0) UnloadTexture(vs.emissive);
        continue;
      }

      const bool slopePositive = (AlphaCovXY(spr.color) >= 0.0);
      auto& dst = slopePositive ? pos : neg;
      if (static_cast<int>(dst.size()) >= kWantPerSlope) {
        UnloadTexture(vs.color);
        if (vs.emissive.id != 0) UnloadTexture(vs.emissive);
        continue;
      }
      dst.push_back(vs);
    }
  };

  buildKind(GfxPropKind::VehicleCar, m_vehicleCarPosSlope, m_vehicleCarNegSlope);
  buildKind(GfxPropKind::VehicleTruck, m_vehicleTruckPosSlope, m_vehicleTruckNegSlope);
}

void Renderer::setCloudShadowSettings(const CloudShadowSettings& s)
{
  const bool regen = (s.coverage != m_cloudShadows.coverage) || (s.softness != m_cloudShadows.softness);
  m_cloudShadows = s;

  // Only the shape parameters require re-synthesizing the mask texture.
  if (regen || m_cloudShadowTex.id == 0) {
    rebuildCloudShadowTexture();
  }
}

void Renderer::rebuildCloudShadowTexture()
{
  if (m_cloudShadowTex.id != 0) {
    UnloadTexture(m_cloudShadowTex);
    m_cloudShadowTex = Texture2D{};
  }

  // Small tileable mask; rendered with TEXTURE_WRAP_REPEAT over the camera AABB.
  constexpr int kSize = 256;
  constexpr int kPeriod = 32;

  const float coverage = std::clamp(m_cloudShadows.coverage, 0.0f, 1.0f);
  const float softness = std::clamp(m_cloudShadows.softness, 0.0f, 1.0f);

  // If there's effectively no coverage, keep the texture empty.
  if (coverage <= 0.001f) {
    return;
  }

  RgbaImage img;
  img.width = kSize;
  img.height = kSize;
  img.rgba.resize(static_cast<std::size_t>(kSize) * static_cast<std::size_t>(kSize) * 4u);

  const float denom = static_cast<float>(kSize - 1);
  const std::uint32_t seed = m_gfxSeed32 ^ 0xC10D15u;

  // More coverage => lower threshold.
  const float threshold = 1.0f - coverage;
  // Transition width in noise units: higher softness => wider boundary.
  const float edge = 0.03f + 0.22f * softness;

  for (int y = 0; y < kSize; ++y) {
    const float fy = (denom > 0.0f) ? (static_cast<float>(y) * static_cast<float>(kPeriod) / denom) : 0.0f;

    for (int x = 0; x < kSize; ++x) {
      const float fx = (denom > 0.0f) ? (static_cast<float>(x) * static_cast<float>(kPeriod) / denom) : 0.0f;

      float n = DomainWarpFBm2DPeriodic(fx, fy, seed, kPeriod, kPeriod, 5, 2.0f, 0.55f, 2.15f);

      // Add a hint of higher-frequency detail so the mask doesn't feel too blobby.
      const float d = FBm2DPeriodic(fx * 2.0f, fy * 2.0f, seed ^ 0xA341316Cu, kPeriod * 2, kPeriod * 2,
                                    3, 2.0f, 0.5f);
      n = std::clamp(n * 0.85f + d * 0.15f, 0.0f, 1.0f);

      // Convert noise into a soft-edged "cloud" mask.
      float m = SmoothStep(threshold - edge, threshold + edge, n);
      // Thicker centers, softer edges.
      m *= (0.75f + 0.25f * n);
      m = std::clamp(m, 0.0f, 1.0f);
      // Slight contrast boost.
      m = m * m;

      const unsigned char a = static_cast<unsigned char>(std::round(255.0f * m));

      const std::size_t idx =
          (static_cast<std::size_t>(y) * static_cast<std::size_t>(kSize) + static_cast<std::size_t>(x)) * 4u;
      img.rgba[idx + 0] = 255;
      img.rgba[idx + 1] = 255;
      img.rgba[idx + 2] = 255;
      img.rgba[idx + 3] = a;
    }
  }

  Image rl = ImageFromRgbaImage(img);
  m_cloudShadowTex = LoadTextureFromImage(rl);
  UnloadImage(rl);

  if (m_cloudShadowTex.id != 0) {
    SetTextureWrap(m_cloudShadowTex, TEXTURE_WRAP_REPEAT);
    SetTextureFilter(m_cloudShadowTex, TEXTURE_FILTER_BILINEAR);
  }
}

void Renderer::unloadMinimap()
{
  if (m_minimapTex.id != 0) {
    UnloadTexture(m_minimapTex);
    m_minimapTex = Texture2D{};
  }
  m_minimapW = 0;
  m_minimapH = 0;
  m_minimapPixels.clear();
  m_minimapDirty = true;
}

void Renderer::unloadBaseCache()
{
  for (auto& b : m_bands) {
    if (b.terrain.id != 0) {
      UnloadRenderTexture(b.terrain);
      b.terrain = RenderTexture2D{};
    }
    if (b.structures.id != 0) {
      UnloadRenderTexture(b.structures);
      b.structures = RenderTexture2D{};
    }
  }
  m_bands.clear();
  m_bandMapW = 0;
  m_bandMapH = 0;
  m_bandMaxPixels = 0.0f;
  m_bandCacheDirtyAll = true;
}

void Renderer::markBaseCacheDirtyAll()
{
  m_bandCacheDirtyAll = true;
  for (auto& b : m_bands) {
    b.dirtyTerrain = true;
    b.dirtyStructures = true;
  }
}

void Renderer::markBaseCacheDirtyForTiles(const std::vector<Point>& tiles, int mapW, int mapH)
{
  if (tiles.empty()) return;
  if (mapW <= 0 || mapH <= 0) return;

  const int numSums = mapW + mapH - 1;
  const int numBands = (numSums + kBandSums - 1) / kBandSums;

  // If the cache hasn't been created yet (or map dimensions changed), just mark everything dirty.
  if (m_bands.empty() || static_cast<int>(m_bands.size()) != numBands || m_bandMapW != mapW || m_bandMapH != mapH) {
    m_bandCacheDirtyAll = true;
    return;
  }

  auto markSum = [&](int sum) {
    if (sum < 0 || sum >= numSums) return;
    const int bi = sum / kBandSums;
    if (bi >= 0 && bi < static_cast<int>(m_bands.size())) {
      BandCache& b = m_bands[static_cast<std::size_t>(bi)];
      b.dirtyTerrain = true;
      b.dirtyStructures = true;
    }
  };

  auto markTile = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= mapW || y >= mapH) return;
    const int s = x + y;
    markSum(s);
    // Height changes affect cliffs drawn on tiles in front (sum+1). Road edits can also change
    // neighbor auto-tiling masks, so we conservatively dirty the next diagonal too.
    markSum(s + 1);
  };

  for (const Point& p : tiles) {
    // Dirty the edited tile and its 4-neighborhood so auto-tiling road masks update correctly.
    markTile(p.x, p.y);
    markTile(p.x - 1, p.y);
    markTile(p.x + 1, p.y);
    markTile(p.x, p.y - 1);
    markTile(p.x, p.y + 1);
  }
}

void Renderer::ensureBaseCache(const World& world)
{
  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) {
    unloadBaseCache();
    return;
  }

  const int numSums = mapW + mapH - 1;
  const int numBands = (numSums + kBandSums - 1) / kBandSums;

  const bool needsRecreate = (m_bands.empty() || m_bandMapW != mapW || m_bandMapH != mapH ||
                              static_cast<int>(m_bands.size()) != numBands ||
                              m_bandMaxPixels != m_elev.maxPixels);

  if (needsRecreate) {
    unloadBaseCache();

    m_bandMapW = mapW;
    m_bandMapH = mapH;
    m_bandMaxPixels = m_elev.maxPixels;

    m_bands.resize(static_cast<std::size_t>(numBands));

    const float tileW = static_cast<float>(m_tileW);
    const float tileH = static_cast<float>(m_tileH);

    const float pad = 2.0f;
    for (int i = 0; i < numBands; ++i) {
      BandCache& b = m_bands[static_cast<std::size_t>(i)];
      b.sum0 = i * kBandSums;
      b.sum1 = std::min(numSums - 1, b.sum0 + (kBandSums - 1));
      b.dirtyTerrain = true;
      b.dirtyStructures = true;

      const BandBounds bb = ComputeBandBounds(b.sum0, b.sum1, mapW, mapH, tileW, tileH, std::max(0.0f, m_bandMaxPixels));
      b.origin = Vector2{bb.minX - pad, bb.minY - pad};

      const int texW = std::max(1, static_cast<int>(std::ceil((bb.maxX - bb.minX) + pad * 2.0f)));
      const int texH = std::max(1, static_cast<int>(std::ceil((bb.maxY - bb.minY) + pad * 2.0f)));

      // Terrain base cache (terrain tops + cliffs).
      b.terrain = LoadRenderTexture(texW, texH);
      if (b.terrain.id != 0) {
        SetTextureFilter(b.terrain.texture, TEXTURE_FILTER_POINT);
      }

      // Structures base cache (roads/zones/parks).
      b.structures = LoadRenderTexture(texW, texH);
      if (b.structures.id != 0) {
        SetTextureFilter(b.structures.texture, TEXTURE_FILTER_POINT);
      }
    }

    m_bandCacheDirtyAll = false;
  }

  if (m_bandCacheDirtyAll) {
    for (auto& b : m_bands) {
      b.dirtyTerrain = true;
      b.dirtyStructures = true;
    }
    m_bandCacheDirtyAll = false;
  }
}

void Renderer::rebuildTerrainCacheBand(const World& world, BandCache& band)
{
  if (band.terrain.id == 0) {
    band.dirtyTerrain = false;
    return;
  }

  const int mapW = world.width();
  const int mapH = world.height();

  const float tileWf = static_cast<float>(m_tileW);
  const float tileHf = static_cast<float>(m_tileH);

  const Rectangle src = Rectangle{0, 0, tileWf, tileHf};
  const Vector2 shift = Vector2{-band.origin.x, -band.origin.y};

  BeginTextureMode(band.terrain);
  ClearBackground(BLANK);

  for (int sum = band.sum0; sum <= band.sum1; ++sum) {
    const int x0 = std::max(0, sum - (mapH - 1));
    const int x1 = std::min(mapW - 1, sum);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      const Tile& t = world.at(x, y);

      const float elevPx = TileElevationPx(t, m_elev);
      const Vector2 baseCenterW = TileToWorldCenter(x, y, tileWf, tileHf);
      const Vector2 baseCenter{baseCenterW.x + shift.x, baseCenterW.y + shift.y};
      const Vector2 center{baseCenter.x, baseCenter.y - elevPx};

      const Rectangle dst = Rectangle{center.x - tileWf * 0.5f, center.y - tileHf * 0.5f, tileWf, tileHf};

      // Per-tile lighting: base height/variation + slope/AO (no animated water shimmer in cache).
      const TileLighting light = ComputeTileLighting(world, x, y, tileWf, tileHf, m_elev, 0.0f, false);
      const float brightness = light.base;

      // Draw terrain tops.
      DrawTexturePro(terrain(t.terrain, t.variation), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));

      // Draw cliff walls for higher neighbors behind this tile.
      {
        Vector2 baseCorners[4];
        TileDiamondCorners(baseCenter, tileWf, tileHf, baseCorners);

        const float eps = 0.5f;

        auto drawCliffEdge = [&](Vector2 e0, Vector2 e1, float topElev, float botElev, Color c) {
          if (topElev <= botElev + eps) return;
          Vector2 top0 = e0;
          Vector2 top1 = e1;
          Vector2 bot0 = e0;
          Vector2 bot1 = e1;
          top0.y -= topElev;
          top1.y -= topElev;
          bot0.y -= botElev;
          bot1.y -= botElev;

          DrawTriangle(top0, top1, bot1, c);
          DrawTriangle(top0, bot1, bot0, c);
        };

        if (x > 0) {
          const Tile& n = world.at(x - 1, y);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[3], baseCorners[0], ne, elevPx, Mul(base, 0.70f));
        }

        if (y > 0) {
          const Tile& n = world.at(x, y - 1);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[0], baseCorners[1], ne, elevPx, Mul(base, 0.85f));
        }
      }
    }
  }

  EndTextureMode();
  band.dirtyTerrain = false;
}

void Renderer::rebuildStructureCacheBand(const World& world, BandCache& band)
{
  if (band.structures.id == 0) {
    band.dirtyStructures = false;
    return;
  }

  const int mapW = world.width();
  const int mapH = world.height();

  const float tileWf = static_cast<float>(m_tileW);
  const float tileHf = static_cast<float>(m_tileH);

  const Rectangle src = Rectangle{0, 0, tileWf, tileHf};
  const Vector2 shift = Vector2{-band.origin.x, -band.origin.y};

  BeginTextureMode(band.structures);
  ClearBackground(BLANK);

  for (int sum = band.sum0; sum <= band.sum1; ++sum) {
    const int x0 = std::max(0, sum - (mapH - 1));
    const int x1 = std::min(mapW - 1, sum);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      const Tile& t = world.at(x, y);

      const float elevPx = TileElevationPx(t, m_elev);
      const Vector2 baseCenterW = TileToWorldCenter(x, y, tileWf, tileHf);
      const Vector2 baseCenter{baseCenterW.x + shift.x, baseCenterW.y + shift.y};
      const Vector2 center{baseCenter.x, baseCenter.y - elevPx};

      const Rectangle dst = Rectangle{center.x - tileWf * 0.5f, center.y - tileHf * 0.5f, tileWf, tileHf};

      // Per-tile lighting: base height/variation + slope/AO (no animated water shimmer in cache).
      const TileLighting light = ComputeTileLighting(world, x, y, tileWf, tileHf, m_elev, 0.0f, false);
      const float brightness = light.base;

      // Draw base overlays (no traffic/goods/outside tinting).
      if (t.overlay == Overlay::Road) {
        const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);
        Texture2D& rtex =
            (t.terrain == Terrain::Water) ? bridge(mask, t.variation, t.level) : road(mask, t.variation, t.level);
        DrawTexturePro(rtex, src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));
      } else if (t.overlay != Overlay::None) {
        DrawTexturePro(overlay(t.overlay), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));
      }
    }
  }

  EndTextureMode();
  band.dirtyStructures = false;
}

Renderer::MinimapLayout Renderer::minimapLayout(const World& world, int screenW, int screenH) const
{
  return ComputeMinimapLayout(world.width(), world.height(), screenW, screenH);
}

void Renderer::ensureMinimapUpToDate(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  const bool needsRecreate = (m_minimapTex.id == 0) || (m_minimapW != w) || (m_minimapH != h) ||
                             (m_minimapPixels.size() != n);

  if (needsRecreate) {
    unloadMinimap();
    m_minimapW = w;
    m_minimapH = h;
    m_minimapPixels.assign(n, BLANK);
    m_minimapDirty = true;
  }

  if (!m_minimapDirty && m_minimapTex.id != 0) return;

  // Rebuild pixel buffer.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      m_minimapPixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          MinimapColorForTile(t);
    }
  }

  if (m_minimapTex.id == 0) {
    // Create a GPU texture directly from our CPU pixel buffer.
    Image img{};
    img.data = m_minimapPixels.data();
    img.width = w;
    img.height = h;
    img.mipmaps = 1;
    img.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    m_minimapTex = LoadTextureFromImage(img);
    if (m_minimapTex.id != 0) {
      // Keep the minimap crisp when scaling up.
      SetTextureFilter(m_minimapTex, TEXTURE_FILTER_POINT);
    }
  } else {
    UpdateTexture(m_minimapTex, m_minimapPixels.data());
  }

  m_minimapDirty = false;
}

bool Renderer::exportMinimapThumbnail(const World& world, const char* fileName, int maxSize)
{
  if (fileName == nullptr || fileName[0] == '\0') return false;

  ensureMinimapUpToDate(world);
  if (m_minimapW <= 0 || m_minimapH <= 0) return false;
  if (m_minimapPixels.empty()) return false;

  // Build an Image from our CPU pixel buffer. We copy because raylib image
  // processing utilities can reallocate the data.
  Image base{};
  base.data = (void*)m_minimapPixels.data();
  base.width = m_minimapW;
  base.height = m_minimapH;
  base.mipmaps = 1;
  base.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

  Image img = ImageCopy(base);
  if (!IsImageReadyCompat(img)) return false;

  const int ms = std::max(1, maxSize);
  const int w = img.width;
  const int h = img.height;
  const int maxDim = std::max(w, h);

  if (maxDim > ms) {
    const float scale = static_cast<float>(ms) / static_cast<float>(maxDim);
    const int nw = std::max(1, static_cast<int>(std::lround(static_cast<float>(w) * scale)));
    const int nh = std::max(1, static_cast<int>(std::lround(static_cast<float>(h) * scale)));
    ImageResize(&img, nw, nh);
  }

  const bool ok = ExportImage(img, fileName);
  UnloadImage(img);
  return ok;
}

bool Renderer::exportWorldOverview(const World& world, const char* fileName, int maxSize,
                                float timeSec, bool includeScreenFx)
{
  if (fileName == nullptr || fileName[0] == '\0') return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  // Compute a conservative bounding box for the full isometric map in *world space*.
  //
  // Notes:
  // - This is based on the diamond tile geometry and the maximum configured elevation.
  // - We add a small extra margin at the top to avoid clipping tall zone "buildings".
  const float tileWf = static_cast<float>(m_tileW);
  const float tileHf = static_cast<float>(m_tileH);
  const float halfW = tileWf * 0.5f;
  const float halfH = tileHf * 0.5f;

  const float maxElev = std::max(0.0f, m_elev.maxPixels);
  const float extraTop = tileHf * 5.0f; // safety margin for extruded zone buildings

  const float left = -static_cast<float>(h) * halfW;
  const float right = static_cast<float>(w) * halfW;
  const float top = -halfH - maxElev - extraTop;
  const float bottom = static_cast<float>(w + h - 1) * halfH;

  const float worldW = std::max(1.0f, right - left);
  const float worldH = std::max(1.0f, bottom - top);
  const float maxDim = std::max(worldW, worldH);

  int ms = std::max(64, maxSize);

  // Try to allocate a render texture; if this fails (GPU limits), fall back to smaller sizes.
  RenderTexture2D rt{};
  float zoom = 1.0f;
  int texW = 0;
  int texH = 0;

  for (int attempt = 0; attempt < 4; ++attempt) {
    zoom = (maxDim > static_cast<float>(ms)) ? (static_cast<float>(ms) / maxDim) : 1.0f;
    texW = std::max(1, static_cast<int>(std::lround(worldW * zoom)));
    texH = std::max(1, static_cast<int>(std::lround(worldH * zoom)));

    rt = LoadRenderTexture(texW, texH);
    if (rt.id != 0) break;

    ms = std::max(64, ms / 2);
  }

  if (rt.id == 0) return false;

  Camera2D cam{};
  cam.target = Vector2{(left + right) * 0.5f, (top + bottom) * 0.5f};
  cam.offset = Vector2{static_cast<float>(texW) * 0.5f, static_cast<float>(texH) * 0.5f};
  cam.zoom = zoom;
  cam.rotation = 0.0f;

  // Render the full map without the band cache to avoid nested BeginTextureMode calls.
  const bool prevCache = m_useBandCache;
  m_useBandCache = false;

  BeginTextureMode(rt);
  ClearBackground(BLANK);
  drawWorld(world, cam, texW, texH,
            /*timeSec=*/timeSec,
            /*hovered=*/std::nullopt,
            /*drawGrid=*/false,
            /*brushRadius=*/0,
            /*selected=*/std::nullopt,
            /*highlightPath=*/nullptr,
            /*roadToEdgeMask=*/nullptr,
            /*roadTraffic=*/nullptr,
            /*trafficMax=*/0,
            /*roadGoodsTraffic=*/nullptr,
            /*goodsMax=*/0,
            /*commercialGoodsFill=*/nullptr,
            /*heatmap=*/nullptr,
            /*heatmapRamp=*/HeatmapRamp::Good);

  if (includeScreenFx) {
    // Screen-space precipitation + fog pass (matches the in-game draw order).
    drawWeatherScreenFX(texW, texH, timeSec, /*allowAestheticDetails=*/true);
  }
  EndTextureMode();

  m_useBandCache = prevCache;

  Image img = LoadImageFromTexture(rt.texture);
  if (!IsImageReadyCompat(img)) {
    UnloadRenderTexture(rt);
    return false;
  }

  // Render textures are flipped vertically when read back.
  ImageFlipVertical(&img);

  const bool ok = ExportImage(img, fileName);
  UnloadImage(img);
  UnloadRenderTexture(rt);
  return ok;
}

static int TerrainIndex(Terrain t)
{
  switch (t) {
  case Terrain::Water: return 0;
  case Terrain::Sand: return 1;
  case Terrain::Grass: return 2;
  default: return 2;
  }
}

static Color TerrainCliffBaseColor(Terrain t)
{
  switch (t) {
  case Terrain::Water: return Color{20, 60, 120, 255};
  case Terrain::Sand: return Color{180, 150, 90, 255};
  case Terrain::Grass: return Color{45, 120, 65, 255};
  default: return Color{45, 120, 65, 255};
  }
}

static int OverlayIndex(Overlay o)
{
  switch (o) {
  case Overlay::None: return 0;
  case Overlay::Road: return 1;
  case Overlay::Residential: return 2;
  case Overlay::Commercial: return 3;
  case Overlay::Industrial: return 4;
  case Overlay::Park: return 5;
  default: return 0;
  }
}

Texture2D& Renderer::terrain(Terrain t, std::uint8_t variation)
{
  const std::size_t ti = static_cast<std::size_t>(TerrainIndex(t));
  const std::size_t vi = static_cast<std::size_t>((variation >> 4) % kTerrainVariants);
  return m_terrainTex[ti][vi];
}

Texture2D& Renderer::overlay(Overlay o) { return m_overlayTex[static_cast<std::size_t>(OverlayIndex(o))]; }

Texture2D& Renderer::road(std::uint8_t mask, std::uint8_t variation, std::uint8_t level)
{
  const int lvl = std::clamp(static_cast<int>(level), 1, kRoadLevels);
  const std::size_t li = static_cast<std::size_t>(lvl - 1);
  const std::size_t mi = static_cast<std::size_t>(mask & 0x0Fu);
  const std::size_t vi = static_cast<std::size_t>((variation >> 4) & (kRoadVariants - 1));
  return m_roadTex[li][mi][vi];
}

Texture2D& Renderer::bridge(std::uint8_t mask, std::uint8_t variation, std::uint8_t level)
{
  const int lvl = std::clamp(static_cast<int>(level), 1, kRoadLevels);
  const std::size_t li = static_cast<std::size_t>(lvl - 1);
  const std::size_t mi = static_cast<std::size_t>(mask & 0x0Fu);
  const std::size_t vi = static_cast<std::size_t>((variation >> 4) & (kRoadVariants - 1));
  return m_bridgeTex[li][mi][vi];
}

Color Renderer::BrightnessTint(float b)
{
  const int v = static_cast<int>(std::round(255.0f * std::clamp(b, 0.0f, 1.5f)));
  const unsigned char u = ClampU8(v);
  return Color{u, u, u, 255};
}

void Renderer::rebuildTextures(std::uint64_t seed)
{
  unloadTextures();

  const std::uint32_t s = static_cast<std::uint32_t>(seed);
  m_gfxSeed32 = s;

  // --- Terrain ---
  // Multiple variants per terrain type drastically reduce visible tiling.
  for (int v = 0; v < kTerrainVariants; ++v) {
    const std::uint32_t sv = s ^ (static_cast<std::uint32_t>(v) * 0x9E3779B9u);

    // Water
    m_terrainTex[0][static_cast<std::size_t>(v)] =
        MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
          const std::uint32_t h = HashCoords32(x, y, sv ^ 0xA1B2C3D4u);
          const float n = (Frac01(h) - 0.5f) * 0.10f;

          // Subtle diagonal waves (purely procedural), with variant-dependent phase.
          const float phase = static_cast<float>(v) * 0.65f;
          const float waves0 = 0.060f * std::sin((x * 0.35f + y * 0.70f) + phase);
          const float waves1 = 0.030f * std::sin((x * 0.90f - y * 0.45f) + phase * 1.73f);
          const float b = 1.0f + n + waves0 + waves1;

          Color base = Color{40, 95, 210, 255};
          base = Mul(base, b);

          // Slightly fade edges to reduce harsh tile seams.
          base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 4.0f, 0.0f, 1.0f));
          return base;
        });

    // Sand
    m_terrainTex[1][static_cast<std::size_t>(v)] =
        MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
          const std::uint32_t h = HashCoords32(x, y, sv ^ 0xBEEFBEEFu);
          const float n = (Frac01(h) - 0.5f) * 0.18f;

          // Low-frequency "ripples" so dunes don't look perfectly flat.
          const float r = 0.040f * std::sin((x * 0.22f + y * 0.31f) + static_cast<float>(v) * 1.10f);

          Color base = Color{200, 186, 135, 255};
          base = Mul(base, 1.0f + n + r);

          // Grain speckles.
          if ((h & 0x1Fu) == 0x1Fu) base = Mul(base, 0.85f);
          if ((h & 0x3Fu) == 0x23u) base = Mul(base, 1.08f);

          base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
          return base;
        });

    // Grass
    m_terrainTex[2][static_cast<std::size_t>(v)] =
        MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
          const std::uint32_t h = HashCoords32(x, y, sv ^ 0x12345678u);
          const float n = (Frac01(h) - 0.5f) * 0.22f;

          // Macro tint variation within a tile (subtle). This plus variants helps break repetition.
          const float patch = 0.040f * std::sin((x * 0.16f - y * 0.19f) + static_cast<float>(v) * 0.95f);

          Color base = Color{70, 170, 90, 255};
          base = Mul(base, 1.0f + n + patch);

          // Tiny darker "blades" of grass.
          if ((h & 0x7Fu) == 0x3Fu) base = Mul(base, 0.78f);
          if ((h & 0xFFu) == 0x5Du) base = Mul(base, 0.88f);

          base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
          return base;
        });
  }

  // --- Overlays ---
  // None: keep as an empty texture (id=0), we won't draw it.
  m_overlayTex[0] = Texture2D{};

  // Road: auto-tiling variants (mask stored in tile.variation low bits).
  // We keep m_overlayTex[1] empty; roads are drawn from m_roadTex[level][mask][variant].
  m_overlayTex[1] = Texture2D{};

  auto popCount4 = [](std::uint8_t m) -> int {
    return ((m & 0x01u) ? 1 : 0) + ((m & 0x02u) ? 1 : 0) + ((m & 0x04u) ? 1 : 0) + ((m & 0x08u) ? 1 : 0);
  };

  struct RoadStyle {
    float roadW = 0.14f;      // half-width in normalized diamond coords
    float lineThick = 0.010f; // marking half-thickness
    float lineGap = 0.018f;   // used for double center lines
    float laneOff = 0.05f;    // used for highway lane lines (computed from roadW)
    float shoulderOff = 0.10f;
    float edgeDark = 0.70f;   // multiplier at edge
    float dashFreq = 10.0f;
    Color asphalt = Color{90, 90, 95, 230};
    Color mark = Color{220, 220, 210, 240};
    Color mark2 = Color{250, 220, 110, 245}; // yellow-ish
    bool dashed = true;
    bool doubleCenter = false;
    bool highway = false;
    bool crosswalk = false;
  };

  auto roadStyleForLevel = [&](int level) -> RoadStyle {
    RoadStyle st{};
    level = std::clamp(level, 1, kRoadLevels);
    if (level == 1) {
      // Street
      st.roadW = 0.130f;
      st.asphalt = Color{95, 95, 100, 230};
      st.mark = Color{235, 235, 230, 245};
      st.dashFreq = 10.0f;
      st.dashed = true;
      st.doubleCenter = false;
      st.highway = false;
      st.crosswalk = true;
      st.edgeDark = 0.78f;
    } else if (level == 2) {
      // Avenue
      st.roadW = 0.175f;
      st.asphalt = Color{85, 85, 90, 235};
      st.mark = Color{240, 240, 240, 245};
      st.mark2 = Color{250, 215, 95, 245};
      st.dashed = false;
      st.doubleCenter = true;
      st.lineGap = 0.022f;
      st.lineThick = 0.008f;
      st.crosswalk = true;
      st.edgeDark = 0.74f;
    } else { // level 3
      // Highway
      st.roadW = 0.215f;
      st.asphalt = Color{72, 72, 76, 240};
      st.mark = Color{245, 245, 245, 245};
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
  };

  auto makeRoadVariant = [&](std::uint8_t mask, int level, int variant) -> Texture2D {
    const RoadStyle st = roadStyleForLevel(level);
    const float centerR = st.roadW * 1.10f;

    const std::uint32_t seedv =
        s ^ 0x0F0F0F0Fu ^ (static_cast<std::uint32_t>(mask) * 0x9E3779B9u) ^
        (static_cast<std::uint32_t>(variant) * 0x85EBCA6Bu) ^
        (static_cast<std::uint32_t>(level) * 0xC2B2AE35u);

    const int conn = popCount4(mask);

    return MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
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
      consider((mask & 0x01u) != 0u, 0.5f, -0.5f); // up-right
      consider((mask & 0x02u) != 0u, 0.5f, 0.5f);  // down-right
      consider((mask & 0x04u) != 0u, -0.5f, 0.5f); // down-left
      consider((mask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

      // Outside the road shape.
      if (sd > 0.0f) return Color{0, 0, 0, 0};

      Color base = st.asphalt;
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
              base = Lerp(base, Color{250, 250, 250, 255}, 0.85f);
            }
          }

          // Level-specific lane markings.
          if (st.highway) {
            // Highway: dashed lane lines and solid shoulders.
            const float thick = st.lineThick;
            if (std::fabs(absPerp - st.shoulderOff) < (thick * 1.20f)) {
              base = st.mark;
            } else if (std::fabs(absPerp - st.laneOff) < thick) {
              const int dash = static_cast<int>(std::floor(bestSegT * st.dashFreq + static_cast<float>(mask) * 0.21f +
                                                           static_cast<float>(variant) * 0.37f));
              if ((dash & 1) == 0) base = st.mark;
            }
          } else if (st.doubleCenter) {
            // Avenue: double solid "median" line.
            if (std::fabs(absPerp - st.lineGap) < st.lineThick) base = st.mark2;
          } else {
            // Street: dashed centerline.
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
      base.a = static_cast<unsigned char>(static_cast<float>(base.a) * a);
      return base;
    });
  };

  for (int level = 1; level <= kRoadLevels; ++level) {
    for (int mask = 0; mask < 16; ++mask) {
      for (int v = 0; v < kRoadVariants; ++v) {
        m_roadTex[static_cast<std::size_t>(level - 1)][static_cast<std::size_t>(mask)][static_cast<std::size_t>(v)] =
            makeRoadVariant(static_cast<std::uint8_t>(mask), level, v);
      }
    }
  }

  auto makeBridgeVariant = [&](std::uint8_t mask, int level, int variant) -> Texture2D {
    // Bridge visuals: inherit lane markings from the road style, but use different deck materials.
    RoadStyle st = roadStyleForLevel(level);
    const float centerR = st.roadW * 1.10f;

    Color deck = Color{160, 130, 95, 235}; // wood for streets
    if (level == 2) deck = Color{170, 170, 175, 240}; // concrete-ish
    if (level == 3) deck = Color{150, 150, 155, 240}; // darker concrete / steel

    const std::uint32_t seedv =
        s ^ 0xB00B1E5u ^ (static_cast<std::uint32_t>(mask) * 0x7F4A7C15u) ^
        (static_cast<std::uint32_t>(variant) * 0x27D4EB2Du) ^
        (static_cast<std::uint32_t>(level) * 0x165667B1u);

    const int conn = popCount4(mask);

    return MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
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

      // Bit layout matches World::computeRoadMask().
      consider((mask & 0x01u) != 0u, 0.5f, -0.5f); // up-right
      consider((mask & 0x02u) != 0u, 0.5f, 0.5f);  // down-right
      consider((mask & 0x04u) != 0u, -0.5f, 0.5f); // down-left
      consider((mask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

      if (sd > 0.0f) return Color{0, 0, 0, 0};

      Color base = deck;
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

      // Soft edges.
      const float edgeSoft = 0.05f;
      const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
      base.a = static_cast<unsigned char>(static_cast<float>(base.a) * a);
      return base;
    });
  };

  for (int level = 1; level <= kRoadLevels; ++level) {
    for (int mask = 0; mask < 16; ++mask) {
      for (int v = 0; v < kRoadVariants; ++v) {
        m_bridgeTex[static_cast<std::size_t>(level - 1)][static_cast<std::size_t>(mask)][static_cast<std::size_t>(v)] =
            makeBridgeVariant(static_cast<std::uint8_t>(mask), level, v);
      }
    }
  }

  // Residential
  m_overlayTex[2] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0001u);
    const float n = (Frac01(h) - 0.5f) * 0.12f;

    Color roof = Color{190, 70, 65, 255};
    roof = Mul(roof, 1.0f + n);

    // Simple roof tiles pattern.
    if ((x + y) % 6 == 0) roof = Mul(roof, 0.86f);

    // Slight vignette.
    roof = Mul(roof, 0.92f + 0.10f * d.edge);
    return roof;
  });

  // Commercial
  m_overlayTex[3] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0002u);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    Color c = Color{70, 115, 190, 255};
    c = Mul(c, 1.0f + n);

    // Windows pattern.
    if ((x / 3 + y / 2) % 5 == 0) c = Mul(c, 1.15f);

    c = Mul(c, 0.92f + 0.10f * d.edge);
    return c;
  });

  // Industrial
  m_overlayTex[4] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0003u);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    Color c = Color{210, 180, 75, 255};
    c = Mul(c, 1.0f + n);

    // Hazard stripes
    if (((x + y) / 3) % 2 == 0) c = Mul(c, 0.85f);

    c = Mul(c, 0.92f + 0.10f * d.edge);
    return c;
  });

  // Park (transparent edges so grass can show through)
  m_overlayTex[5] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xCAFE0004u);
    const float n = (Frac01(h) - 0.5f) * 0.12f;

    Color c = Color{60, 190, 95, 230};
    c = Mul(c, 1.0f + n);

    // Procedural "trees" (dark dots).
    if ((h & 0xFFu) == 0x7Au) c = Color{25, 110, 55, 240};

    const float a = std::clamp(d.edge * 7.0f, 0.0f, 1.0f);
    c.a = static_cast<unsigned char>(static_cast<float>(c.a) * a);
    return c;
  });

  // Procedural vehicle sprites (used by the micro-sim overlay).
  rebuildVehicleSprites();

  // World-space cloud shadow mask (procedural, tileable).
  rebuildCloudShadowTexture();
}

namespace {

inline unsigned char LerpU8(unsigned char a, unsigned char b, float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  return static_cast<unsigned char>(std::round(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * t));
}

inline Color LerpColor(Color a, Color b, float t) {
  return Color{LerpU8(a.r, b.r, t), LerpU8(a.g, b.g, t), LerpU8(a.b, b.b, t), LerpU8(a.a, b.a, t)};
}


// -----------------------------
// Building ground shadows (stylized 2D projection)
// -----------------------------

struct BuildingShadowCaster {
  Vector2 base[4]{};      // base footprint polygon (world coords)
  float heightPx = 0.0f;  // building height in world pixels
  float alphaScale = 1.0f;
};

static Vector2 UnitDirFromAzimuthDeg(float deg) {
  constexpr float kPi = 3.14159265358979323846f;
  const float rad = deg * (kPi / 180.0f);
  Vector2 d{static_cast<float>(std::cos(rad)), static_cast<float>(std::sin(rad))};
  const float len = std::sqrt(d.x * d.x + d.y * d.y);
  if (len > 0.0001f) {
    d.x /= len;
    d.y /= len;
  } else {
    d = Vector2{1.0f, 0.0f};
  }
  return d;
}

static bool BuildZoneTileShadowCaster(const Tile& t, float tileW, float tileH, float zoom, const Vector2& tileCenter,
                                     BuildingShadowCaster& out) {
  (void)zoom;

  const Overlay ov = t.overlay;
  if (!(ov == Overlay::Residential || ov == Overlay::Commercial || ov == Overlay::Industrial || ov == Overlay::Park)) {
    return false;
  }

  int lvl = ClampZoneLevel(static_cast<int>(t.level));
  float heightMul = 1.0f;
  float baseShrink = 0.86f;
  int cap = 0;

  switch (ov) {
    case Overlay::Residential:
      heightMul = 0.95f;
      baseShrink = 0.86f;
      cap = CapacityForOverlayLevel(ov, lvl);
      break;
    case Overlay::Commercial:
      heightMul = 1.10f;
      baseShrink = 0.84f;
      cap = CapacityForOverlayLevel(ov, lvl);
      break;
    case Overlay::Industrial:
      heightMul = 1.25f;
      baseShrink = 0.82f;
      cap = CapacityForOverlayLevel(ov, lvl);
      break;
    case Overlay::Park:
      // small pavilion / kiosk
      heightMul = 0.45f;
      baseShrink = 0.92f;
      cap = 10;
      lvl = 1;
      break;
    default:
      return false;
  }

  const int occ = std::max(0, static_cast<int>(t.occupants));
  const float occRatio = (cap > 0) ? std::clamp(static_cast<float>(occ) / static_cast<float>(cap), 0.0f, 1.0f) : 0.0f;
  const float var = 0.15f + 0.85f * (static_cast<float>(t.variation) / 255.0f);

  float heightPx = tileH * (0.55f + 0.65f * static_cast<float>(lvl)) + tileH * (0.25f + 0.45f * static_cast<float>(lvl)) * occRatio;
  heightPx *= heightMul;
  heightPx *= (0.85f + 0.35f * var);

  const float maxH = tileH * (2.6f + 0.7f * static_cast<float>(lvl)) * heightMul;
  heightPx = std::min(heightPx, maxH);

  Vector2 outer[4];
  TileDiamondCorners(tileCenter, tileW, tileH, outer);
  ShrinkDiamond(out.base, outer, baseShrink);

  out.heightPx = heightPx;
  const float hNorm = std::clamp(heightPx / (tileH * 2.2f), 0.0f, 1.0f);
  out.alphaScale = std::clamp(0.55f + 0.45f * hNorm, 0.45f, 1.0f) * (0.75f + 0.25f * occRatio);
  return true;
}

static bool BuildZoneParcelShadowCaster(const World& world, const ZoneBuildingParcel& p, const ElevationSettings& elev, float tileW,
                                       float tileH, float zoom, float timeSec, BuildingShadowCaster& out) {
  (void)zoom;
  (void)timeSec;

  if (!p.isMultiTile()) {
    return false;
  }

  const Overlay ov = p.overlay;
  if (!IsZoneOverlay(ov)) {
    return false;
  }

  const int lvl = ClampZoneLevel(static_cast<int>(p.level));
  const int x0 = p.x0;
  const int y0 = p.y0;
  const int x1 = p.x0 + p.w - 1;
  const int y1 = p.y0 + p.h - 1;

  float baseElevPx = 0.0f;
  int totalOcc = 0;
  int tiles = 0;

  for (int yy = y0; yy <= y1; ++yy) {
    for (int xx = x0; xx <= x1; ++xx) {
      const Tile& tt = world.at(xx, yy);
      if (tt.overlay != ov) {
        continue;
      }
      baseElevPx = std::max(baseElevPx, TileElevationPx(tt, elev));
      totalOcc += std::max(0, static_cast<int>(tt.occupants));
      ++tiles;
    }
  }

  if (tiles <= 0) {
    return false;
  }

  const int capPerTile = CapacityForOverlayLevel(ov, lvl);
  const int totalCap = capPerTile * tiles;
  const float occRatio = (totalCap > 0) ? std::clamp(static_cast<float>(totalOcc) / static_cast<float>(totalCap), 0.0f, 1.0f) : 0.0f;

  const float scale = std::sqrt(static_cast<float>(p.area()));

  float shrinkK = 0.12f;
  if (ov == Overlay::Commercial) {
    shrinkK = 0.10f;
  } else if (ov == Overlay::Industrial) {
    shrinkK = 0.08f;
  }
  const float baseShrink = std::clamp(1.0f - shrinkK / std::max(1.0f, scale), 0.55f, 0.94f);

  Vector2 outer[4];
  outer[0] = TileCornerAtBaseElevation(x0, y0, tileW, tileH, baseElevPx, 0);
  outer[1] = TileCornerAtBaseElevation(x1, y0, tileW, tileH, baseElevPx, 1);
  outer[2] = TileCornerAtBaseElevation(x1, y1, tileW, tileH, baseElevPx, 2);
  outer[3] = TileCornerAtBaseElevation(x0, y1, tileW, tileH, baseElevPx, 3);

  ShrinkDiamond(out.base, outer, baseShrink);

  float heightMul = 1.0f;
  if (ov == Overlay::Residential) {
    heightMul = 0.95f;
  } else if (ov == Overlay::Commercial) {
    heightMul = 1.10f;
  } else if (ov == Overlay::Industrial) {
    heightMul = 1.25f;
  }

  const float var = static_cast<float>((p.styleSeed >> 4) & 0x0Fu) / 15.0f;

  float heightPx = tileH * (0.55f + 0.65f * static_cast<float>(lvl)) + tileH * (0.25f + 0.45f * static_cast<float>(lvl)) * occRatio;
  const float footprintMul = 1.0f + 0.32f * (scale - 1.0f);
  heightPx *= heightMul * footprintMul;
  heightPx *= (0.85f + 0.35f * (0.15f + 0.85f * var));

  const float maxH = tileH * (2.7f + 0.8f * static_cast<float>(lvl)) * heightMul * (1.0f + 0.25f * (scale - 1.0f));
  heightPx = std::min(heightPx, maxH);

  out.heightPx = heightPx;
  const float hNorm = std::clamp(heightPx / (tileH * 3.2f), 0.0f, 1.0f);
  out.alphaScale = std::clamp(0.60f + 0.40f * hNorm, 0.50f, 1.0f) * (0.80f + 0.20f * occRatio);
  return true;
}

static void DrawBuildingShadowsPass(const std::vector<BuildingShadowCaster>& casters, const Renderer::ShadowSettings& settings,
                                   const DayNightState& dayNight, const WeatherState& weather, float tileW, float tileH, float zoom) {
  if (!settings.enabled || casters.empty()) {
    return;
  }

  // Day/night disabled => we treat it as a perpetual day (dayNight is seeded accordingly in drawWorld).
  const float day = std::clamp(dayNight.day, 0.0f, 1.0f);
  if (day <= 0.03f) {
    return;
  }

  float strength = std::clamp(settings.strength, 0.0f, 1.0f) * day;
  const float overcast = std::clamp(weather.overcast, 0.0f, 1.0f);

  // Overcast suppresses and softens shadows.
  strength *= (1.0f - 0.75f * overcast);
  if (strength <= 0.01f) {
    return;
  }

  const float sun = std::clamp(dayNight.sun, 0.0f, 1.0f);
  float altDeg = Lerp(settings.minAltitudeDeg, settings.maxAltitudeDeg, sun);
  altDeg = std::clamp(altDeg, 1.0f, 89.0f);

  constexpr float kPi = 3.14159265358979323846f;
  const float altRad = altDeg * (kPi / 180.0f);
  const float invTanAlt = 1.0f / std::max(0.08f, static_cast<float>(std::tan(altRad)));

  const Vector2 dir = UnitDirFromAzimuthDeg(settings.azimuthDeg);

  const float tileDiag = 0.5f * std::sqrt(tileW * tileW + tileH * tileH);
  const float maxLenPx = std::max(0.0f, settings.maxLengthTiles) * tileDiag;

  const float invZoom = 1.0f / std::max(0.001f, zoom);
  float softnessPx = std::clamp(settings.softness, 0.0f, 1.0f) * 6.0f * invZoom;
  softnessPx *= (1.0f + 1.25f * overcast);

  const int rings = (softnessPx > 0.25f) ? 3 : 0;

  // Base alpha is intentionally conservative: the scene already has time-of-day grading.
  const float baseAlphaF = 140.0f * strength;

  for (const BuildingShadowCaster& c : casters) {
    if (c.heightPx <= 1.0f) {
      continue;
    }

    Vector2 shift{dir.x * c.heightPx * invTanAlt, dir.y * c.heightPx * invTanAlt};

    const float len = std::sqrt(shift.x * shift.x + shift.y * shift.y);
    if (maxLenPx > 0.0f && len > maxLenPx) {
      const float s = maxLenPx / len;
      shift.x *= s;
      shift.y *= s;
    }

    Vector2 poly[4];
    for (int i = 0; i < 4; ++i) {
      poly[i] = Vector2{c.base[i].x + shift.x, c.base[i].y + shift.y};
    }

    const float alphaF = std::clamp(baseAlphaF * c.alphaScale, 0.0f, 255.0f);
    const unsigned char alphaMain = static_cast<unsigned char>(std::round(alphaF));
    if (alphaMain == 0) {
      continue;
    }

    // Compute center & radius for penumbra scaling.
    Vector2 cen{0.0f, 0.0f};
    for (int i = 0; i < 4; ++i) {
      cen.x += poly[i].x;
      cen.y += poly[i].y;
    }
    cen.x *= 0.25f;
    cen.y *= 0.25f;

    float maxR = 0.0f;
    for (int i = 0; i < 4; ++i) {
      const float dx = poly[i].x - cen.x;
      const float dy = poly[i].y - cen.y;
      maxR = std::max(maxR, std::sqrt(dx * dx + dy * dy));
    }
    maxR = std::max(1.0f, maxR);

    // Penumbra rings (draw outer to inner).
    for (int r = rings; r >= 1; --r) {
      const float k = static_cast<float>(r) / static_cast<float>(rings + 1);
      const float extra = softnessPx * k;
      const float scale = 1.0f + extra / maxR;

      const float aMul = 0.22f * (1.0f - 0.50f * k);
      const unsigned char a = static_cast<unsigned char>(std::round(static_cast<float>(alphaMain) * aMul));
      if (a == 0) {
        continue;
      }

      Vector2 ring[4];
      for (int i = 0; i < 4; ++i) {
        ring[i] = Vector2{cen.x + (poly[i].x - cen.x) * scale, cen.y + (poly[i].y - cen.y) * scale};
      }

      const Color col{0, 0, 0, a};
      DrawTriangle(ring[0], ring[1], ring[2], col);
      DrawTriangle(ring[0], ring[2], ring[3], col);
    }

    const Color col{0, 0, 0, alphaMain};
    DrawTriangle(poly[0], poly[1], poly[2], col);
    DrawTriangle(poly[0], poly[2], poly[3], col);
  }
}

Color HeatmapColor(float v, Renderer::HeatmapRamp ramp) {
  v = std::clamp(v, 0.0f, 1.0f);

  // Alpha scaling: keep "wet" ramps slightly more subtle so they don't obliterate the underlying tiles.
  float alphaF = 0.0f;
  if (ramp == Renderer::HeatmapRamp::Water) {
    alphaF = std::clamp(40.0f + 160.0f * v, 0.0f, 255.0f);
  } else {
    alphaF = std::clamp(70.0f + 110.0f * v, 0.0f, 255.0f);
  }

  const unsigned char a = static_cast<unsigned char>(alphaF);

  // Core ramps: red/yellow/green.
  const Color red{220, 70, 70, a};
  const Color yellow{240, 220, 90, a};
  const Color green{70, 220, 120, a};

  if (ramp == Renderer::HeatmapRamp::Water) {
    // 0 (shallow) -> light blue ... 1 (deep) -> saturated blue
    const Color shallow{150, 220, 255, a};
    const Color deep{40, 120, 255, a};
    return LerpColor(shallow, deep, v);
  }

  if (ramp == Renderer::HeatmapRamp::Bad) {
    // 0 (good) -> green ... 1 (bad) -> red
    if (v < 0.5f) {
      return LerpColor(green, yellow, v / 0.5f);
    }
    return LerpColor(yellow, red, (v - 0.5f) / 0.5f);
  }

  // 0 (bad) -> red ... 1 (good) -> green
  if (v < 0.5f) {
    return LerpColor(red, yellow, v / 0.5f);
  }
  return LerpColor(yellow, green, (v - 0.5f) / 0.5f);
}


} // namespace

void Renderer::drawWorld(const World& world, const Camera2D& camera, int screenW, int screenH, float timeSec,
                         std::optional<Point> hovered,
                         bool drawGrid, int brushRadius, std::optional<Point> selected,
                         const std::vector<Point>* highlightPath, const std::vector<std::uint8_t>* roadToEdgeMask,
                         const std::vector<std::uint16_t>* roadTraffic, int trafficMax,
                         const std::vector<std::uint16_t>* roadGoodsTraffic, int goodsMax,
                         const std::vector<std::uint8_t>* commercialGoodsFill,
                         const std::vector<float>* heatmap,
                         HeatmapRamp heatmapRamp,
                         bool showDistrictOverlay,
                         int highlightDistrict,
                         bool showDistrictBorders,
                         bool mergeZoneBuildings,
                         const WorldOverlayCallback& drawBeforeFx,
                         const WorldOverlayCallback& drawAfterFx,
                         const std::vector<WorldSprite>* sprites)
{
  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) return;

  // -----------------------------
  // Multi-layer rendering toggles
  // -----------------------------
  const std::uint32_t layerMask = m_layerMask;
  const bool layerTerrain = (layerMask & kLayerTerrain) != 0u;
  const bool layerDecals = (layerMask & kLayerDecals) != 0u;
  const bool layerStructures = (layerMask & kLayerStructures) != 0u;
  const bool layerOverlays = (layerMask & kLayerOverlays) != 0u;

  // Overlay inputs are optional; validate their sizes before using.
  const bool wantsOutside =
    (roadToEdgeMask && roadToEdgeMask->size() == static_cast<std::size_t>(mapW * mapH));
  const bool wantsTraffic =
    (roadTraffic && trafficMax > 0 && roadTraffic->size() == static_cast<std::size_t>(mapW * mapH));
  const bool wantsGoods =
    (roadGoodsTraffic && goodsMax > 0 && roadGoodsTraffic->size() == static_cast<std::size_t>(mapW * mapH));
  const bool wantsCommercialGoods =
    (commercialGoodsFill && commercialGoodsFill->size() == static_cast<std::size_t>(mapW * mapH));
  const bool wantsHeatmap = (heatmap && heatmap->size() == static_cast<std::size_t>(mapW * mapH));

  // Respect the overlays layer: if it's disabled, we treat all overlays as off.
  const bool showOutside = layerOverlays && wantsOutside;
  const bool showTraffic = layerOverlays && wantsTraffic;
  const bool showGoods = layerOverlays && wantsGoods;
  const bool showCommercialGoods = layerOverlays && wantsCommercialGoods;
  const bool showHeatmap = layerOverlays && wantsHeatmap;
  const bool drawGridEff = layerOverlays && drawGrid;
  const bool showDistrictOverlayEff = layerOverlays && showDistrictOverlay;
  const bool showDistrictBordersEff = layerOverlays && showDistrictBorders;
  const std::vector<Point>* highlightPathEff = (layerOverlays ? highlightPath : nullptr);

  // -----------------------------
  // Aesthetic detail gating
  // -----------------------------
  // When utility/debug overlays are active we suppress aesthetic details (sparkles, day/night grading, etc.)
  // for readability (matching existing behavior). Decals layer also controls these purely-visual effects.
  const bool suppressAesthetics = (showOutside || showTraffic || showGoods || showCommercialGoods || showHeatmap);
  const bool drawAestheticDetails = layerDecals && !suppressAesthetics;

  // Capacity helper used by the traffic overlay. Keep it local so renderer doesn't need
  // the whole simulation config.
  const auto roadCapacity = [](std::uint8_t roadLevel) -> int {
    constexpr int kBaseRoadTileCapacity = 28;
    return std::max(1, RoadCapacityForLevel(kBaseRoadTileCapacity, static_cast<int>(roadLevel)));
  };

  DayNightState dayNight{};
  if (m_dayNight.enabled && drawAestheticDetails) {
    dayNight = ComputeDayNightState(timeSec, m_dayNight);
  } else {
    dayNight.day = 1.0f;
    dayNight.sun = 1.0f;
    dayNight.dusk = 0.0f;
    dayNight.nightLights = 0.0f;
  }
  const WeatherState weather = (drawAestheticDetails) ? ComputeWeatherState(timeSec, m_weather) : WeatherState{};

  // -----------------------------
  // Outside connectivity overlay
  // -----------------------------
  ZoneAccessMap zoneAccessOutside;
  if (showOutside) {
    zoneAccessOutside = BuildZoneAccessMap(world, roadToEdgeMask);
  }

  // -----------------------------
  // View + visible tile range
  // -----------------------------
  const float tileWf = static_cast<float>(m_tileW);
  const float tileHf = static_cast<float>(m_tileH);
  const float maxElev = std::max(0.0f, m_elev.maxPixels);

  const WorldRect viewAABB = ComputeCameraWorldAABB(camera, screenW, screenH, tileWf, tileHf + maxElev);
  TileRect vis = ComputeVisibleTileRect(camera, screenW, screenH, mapW, mapH, tileWf, tileHf, maxElev);

  // -----------------------------
  // Depth-sorted dynamic sprites
  // -----------------------------
  std::vector<const WorldSprite*> preSprites;
  std::vector<const WorldSprite*> emissiveSprites;
  if (sprites && !sprites->empty()) {
    preSprites.reserve(sprites->size());
    emissiveSprites.reserve(sprites->size());
    for (const WorldSprite& s : *sprites) {
      if (!s.tex || s.tex->id == 0) continue;
      if (s.emissive) emissiveSprites.push_back(&s);
      else preSprites.push_back(&s);
    }

    auto cmp = [](const WorldSprite* a, const WorldSprite* b) {
      if (a->sortSum != b->sortSum) return a->sortSum < b->sortSum;
      if (a->sortX != b->sortX) return a->sortX < b->sortX;
      return a < b;
    };

    std::sort(preSprites.begin(), preSprites.end(), cmp);
    std::sort(emissiveSprites.begin(), emissiveSprites.end(), cmp);
  }

  // -----------------------------
  // Band caches (terrain / structures)
  // -----------------------------
  const bool wantTerrainCache = m_useBandCache && layerTerrain;
  // Structures cache is valid only when we don't need per-tile tinting of base overlays (traffic/goods/outside).
  const bool wantStructureCache =
    m_useBandCache && layerStructures && !(showOutside || showTraffic || showGoods || showCommercialGoods);

  bool terrainCacheReady = false;
  bool structureCacheReady = false;

  if (wantTerrainCache || wantStructureCache) {
    ensureBaseCache(world);

    if (!m_bands.empty()) {
      if (wantTerrainCache) {
        terrainCacheReady = true;
        for (const auto& b : m_bands) {
          if (b.terrain.id == 0) {
            terrainCacheReady = false;
            break;
          }
        }
      }
      if (wantStructureCache) {
        structureCacheReady = true;
        for (const auto& b : m_bands) {
          if (b.structures.id == 0) {
            structureCacheReady = false;
            break;
          }
        }
      }
    }

    // Rebuild dirty bands intersecting the view, with a small pad so panning doesn't thrash.
    if (terrainCacheReady || structureCacheReady) {
      WorldRect rebuildAABB = viewAABB;
      const float pad = tileWf * 2.0f;
      rebuildAABB.minX -= pad;
      rebuildAABB.minY -= pad;
      rebuildAABB.maxX += pad;
      rebuildAABB.maxY += pad;

      for (auto& b : m_bands) {
        int texW = 0;
        int texH = 0;
        if (terrainCacheReady && b.terrain.id != 0) {
          texW = b.terrain.texture.width;
          texH = b.terrain.texture.height;
        } else if (structureCacheReady && b.structures.id != 0) {
          texW = b.structures.texture.width;
          texH = b.structures.texture.height;
        } else {
          continue;
        }

        const float bx0 = b.origin.x;
        const float by0 = b.origin.y;
        const float bx1 = b.origin.x + static_cast<float>(texW);
        const float by1 = b.origin.y + static_cast<float>(texH);

        const bool intersects = !(bx1 < rebuildAABB.minX || bx0 > rebuildAABB.maxX || by1 < rebuildAABB.minY ||
                                  by0 > rebuildAABB.maxY);
        if (!intersects) continue;

        if (terrainCacheReady && b.dirtyTerrain) rebuildTerrainCacheBand(world, b);
        if (structureCacheReady && b.dirtyStructures) rebuildStructureCacheBand(world, b);
      }
    }
  }

  // -----------------------------
  // Zone building merge scratch
  // -----------------------------
  const float tileScreenW = tileWf * camera.zoom;
  const bool useMergedZoneBuildings = mergeZoneBuildings && layerStructures && tileScreenW >= 26.0f;

  if (useMergedZoneBuildings) {
    BuildZoneBuildingParcels(world, m_zoneParcelsScratch);
    // Expand visible rect so merged parcels just outside the viewport can still render correctly.
    vis.minX = std::max(0, vis.minX - 4);
    vis.minY = std::max(0, vis.minY - 4);
    vis.maxX = std::min(mapW - 1, vis.maxX + 4);
    vis.maxY = std::min(mapH - 1, vis.maxY + 4);
  } else {
    m_zoneParcelsScratch.clear();
  }

  // Building shadows (cast onto ground).
  const bool drawShadows = drawAestheticDetails && layerStructures && m_shadows.enabled && (tileScreenW >= 26.0f);
  std::vector<BuildingShadowCaster> shadowCasters;
  if (drawShadows) {
    // Rough heuristic: only a fraction of visible tiles have buildings.
    const int visW = vis.maxX - vis.minX + 1;
    const int visH = vis.maxY - vis.minY + 1;
    shadowCasters.reserve(static_cast<std::size_t>(std::max(0, (visW * visH) / 6)));
  }

  // -----------------------------
  // Render
  // -----------------------------
  const Rectangle src = Rectangle{0, 0, tileWf, tileHf};
  const bool animatedLighting = drawAestheticDetails && !terrainCacheReady;

  BeginMode2D(camera);

  // Depth-sorted sprite injection.
  //
  // We draw "pre" sprites during pass 2, between per-tile overlays and buildings, so they can be
  // occluded by buildings on later diagonals (proper isometric painter's ordering).
  std::size_t preSpriteIdx = 0;
  auto drawWorldSprite = [&](const WorldSprite& s) {
    if (!s.tex || s.tex->id == 0) return;
    DrawTexturePro(*s.tex, s.src, s.dst, s.origin, s.rotation, s.tint);
  };

  auto flushPreSprites = [&](int curSum, float curX) {
    while (preSpriteIdx < preSprites.size()) {
      const WorldSprite* s = preSprites[preSpriteIdx];
      if (!s) {
        ++preSpriteIdx;
        continue;
      }
      if (s->sortSum < curSum) {
        drawWorldSprite(*s);
        ++preSpriteIdx;
        continue;
      }
      if (s->sortSum > curSum) break;
      if (s->sortX <= curX) {
        drawWorldSprite(*s);
        ++preSpriteIdx;
        continue;
      }
      break;
    }
  };

  // Draw cached bands (terrain then structures) in band order (back-to-front).
  if (terrainCacheReady || structureCacheReady) {
    for (const auto& b : m_bands) {
      int texW = 0;
      int texH = 0;
      if (terrainCacheReady && b.terrain.id != 0) {
        texW = b.terrain.texture.width;
        texH = b.terrain.texture.height;
      } else if (structureCacheReady && b.structures.id != 0) {
        texW = b.structures.texture.width;
        texH = b.structures.texture.height;
      } else {
        continue;
      }

      const float bx0 = b.origin.x;
      const float by0 = b.origin.y;
      const float bx1 = b.origin.x + static_cast<float>(texW);
      const float by1 = b.origin.y + static_cast<float>(texH);

      const bool intersects =
        !(bx1 < viewAABB.minX || bx0 > viewAABB.maxX || by1 < viewAABB.minY || by0 > viewAABB.maxY);
      if (!intersects) continue;

      const Rectangle srcRT = Rectangle{0, 0, static_cast<float>(texW), -static_cast<float>(texH)};
      if (terrainCacheReady && layerTerrain) DrawTextureRec(b.terrain.texture, srcRT, b.origin, WHITE);
      if (structureCacheReady && layerStructures) DrawTextureRec(b.structures.texture, srcRT, b.origin, WHITE);
    }
  }

  // Main tile loop: diagonal back-to-front order for correct iso depth.
  for (int sum = vis.minSum(); sum <= vis.maxSum(); ++sum) {
    const int x0 = std::max(vis.minX, sum - vis.maxY);
    const int x1 = std::min(vis.maxX, sum - vis.minY);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      if (!world.inBounds(x, y)) continue;

      const std::size_t tileIdx = static_cast<std::size_t>(x + y * mapW);
      const Tile& tile = world.at(x, y);

      const float elevPx = TileElevationPx(tile, m_elev);

      const Vector2 baseCenter = TileToWorldCenter(x, y, tileWf, tileHf);
      const Vector2 center = Vector2{baseCenter.x, baseCenter.y - elevPx};

      const Rectangle dst = Rectangle{center.x - tileWf * 0.5f, center.y - tileHf * 0.5f, tileWf, tileHf};

      const TileLighting light = ComputeTileLighting(world, x, y, tileWf, tileHf, m_elev, timeSec, animatedLighting);
      const float baseBrightness = light.base;
      const float brightness = animatedLighting ? light.animated : light.base;

      // -----------------------------
      // Terrain (if not cached)
      // -----------------------------
      if (layerTerrain && !terrainCacheReady) {
        DrawTexturePro(terrain(tile.terrain, tile.variation), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));

        // Cliff walls for higher tiles behind.
        Vector2 baseCorners[4];
        TileDiamondCorners(baseCenter, tileWf, tileHf, baseCorners);

        const float eps = 0.5f;
        auto drawCliffEdge = [&](Vector2 e0, Vector2 e1, float topElev, float botElev, Color c) {
          if (topElev <= botElev + eps) return;
          Vector2 top0 = e0;
          Vector2 top1 = e1;
          Vector2 bot0 = e0;
          Vector2 bot1 = e1;
          top0.y -= topElev;
          top1.y -= topElev;
          bot0.y -= botElev;
          bot1.y -= botElev;

          DrawTriangle(top0, top1, bot1, c);
          DrawTriangle(top0, bot1, bot0, c);
        };

        if (x > 0) {
          const Tile& n = world.at(x - 1, y);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[3], baseCorners[0], ne, elevPx, Mul(base, 0.70f));
        }

        if (y > 0) {
          const Tile& n = world.at(x, y - 1);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[0], baseCorners[1], ne, elevPx, Mul(base, 0.85f));
        }
      }

      // -----------------------------
      // Base overlays (structures) if not cached
      // -----------------------------
      if (layerStructures && !structureCacheReady) {
        if (tile.overlay == Overlay::Road) {
          const std::uint8_t mask = static_cast<std::uint8_t>(tile.variation & 0x0Fu);
          const float roadBrightness = (tile.terrain == Terrain::Water) ? baseBrightness : brightness;
          Color tint = BrightnessTint(roadBrightness);

          bool disconnected = false;
          if (showOutside && (*roadToEdgeMask)[tileIdx] == 0) {
            disconnected = true;
            tint = Color{
              static_cast<unsigned char>(std::clamp<int>(tint.r + 80, 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.g - 60, 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.b - 60, 0, 255)),
              255};
          }

          if (!disconnected && showTraffic) {
            const float tnorm =
              std::clamp((*roadTraffic)[tileIdx] / static_cast<float>(trafficMax), 0.0f, 1.0f);
            tint = Color{
              static_cast<unsigned char>(std::clamp<int>(tint.r + static_cast<int>(220.0f * tnorm), 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.g - static_cast<int>(70.0f * tnorm), 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.b - static_cast<int>(70.0f * tnorm), 0, 255)),
              255};
            // Over-capacity highlight.
            if ((*roadTraffic)[tileIdx] > roadCapacity(tile.level)) {
              tint = Color{255, 60, 60, 255};
            }
          }

          if (!disconnected && showGoods) {
            const float gnorm =
              std::clamp((*roadGoodsTraffic)[tileIdx] / static_cast<float>(goodsMax), 0.0f, 1.0f);
            tint = Color{
              static_cast<unsigned char>(std::clamp<int>(tint.r - static_cast<int>(70.0f * gnorm), 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.g - static_cast<int>(70.0f * gnorm), 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.b + static_cast<int>(220.0f * gnorm), 0, 255)),
              255};
          }

          Texture2D& rtex =
            (tile.terrain == Terrain::Water) ? bridge(mask, tile.variation, tile.level) : road(mask, tile.variation, tile.level);
          DrawTexturePro(rtex, src, dst, Vector2{0, 0}, 0.0f, tint);
        } else if (tile.overlay != Overlay::None) {
          Color tint = BrightnessTint(brightness);

          if (showOutside) {
            if (tile.overlay == Overlay::Park) {
              if (!HasAdjacentRoadConnectedToEdge(world, *roadToEdgeMask, x, y)) {
                tint = Mul(tint, 0.55f);
              }
            } else if (tile.overlay == Overlay::Residential || tile.overlay == Overlay::Commercial || tile.overlay == Overlay::Industrial) {
              if (!HasZoneAccess(zoneAccessOutside, x, y)) {
                tint = Mul(tint, 0.55f);
              }
            }
          }

          if (showCommercialGoods && tile.overlay == Overlay::Commercial) {
            const float fill = (*commercialGoodsFill)[tileIdx] / 255.0f;
            tint = Color{
              static_cast<unsigned char>(std::clamp<int>(tint.r + static_cast<int>(255.0f * (1.0f - fill)), 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.g - static_cast<int>(60.0f * (1.0f - fill)), 0, 255)),
              static_cast<unsigned char>(std::clamp<int>(tint.b - static_cast<int>(60.0f * (1.0f - fill)), 0, 255)),
              255};
          }

          DrawTexturePro(overlay(tile.overlay), src, dst, Vector2{0, 0}, 0.0f, tint);
        }
      }

      // -----------------------------
      // Decals (procedural details + ground weather)
      // -----------------------------
      if (drawAestheticDetails) {
        // Procedural micro-detail pass (grass tufts, rocks, water sparkles, etc.)
        DrawProceduralTileDetails(world, x, y, tile, center, tileWf, tileHf,
                                 camera.zoom, brightness, m_gfxSeed32, timeSec);

        // Ground weather effects (wet sheen, snow cover, etc.)
        if (m_weather.affectGround) {
          DrawWeatherGroundEffects(world, x, y, tile, center, tileWf, tileHf,
                                  camera.zoom, brightness, dayNight, weather, timeSec, m_gfxSeed32);
        }
      }

      // Shoreline highlight is a subtle readability aid; keep it as a decal even when aesthetic details are suppressed.
      if (layerDecals && tileScreenW >= 18.0f) {
        const bool tileIsWater = (tile.terrain == Terrain::Water);
        if (tileIsWater) {
          const bool leftLand = (x > 0) && world.at(x - 1, y).terrain != Terrain::Water;
          const bool rightLand = (x < mapW - 1) && world.at(x + 1, y).terrain != Terrain::Water;
          const bool upLand = (y > 0) && world.at(x, y - 1).terrain != Terrain::Water;
          const bool downLand = (y < mapH - 1) && world.at(x, y + 1).terrain != Terrain::Water;

          const bool hasLandNeighbor = leftLand || rightLand || upLand || downLand;
          if (hasLandNeighbor) {
            Vector2 c[4];
            TileDiamondCorners(center, tileWf, tileHf, c);

            const float shoreT = 0.12f;
            const Vector2 cTop = LerpV(c[0], c[1], 0.5f);
            const Vector2 cRight = LerpV(c[1], c[2], 0.5f);
            const Vector2 cBottom = LerpV(c[2], c[3], 0.5f);
            const Vector2 cLeft = LerpV(c[3], c[0], 0.5f);

            auto drawSeg = [&](Vector2 a, Vector2 b) {
              const float thick = std::max(0.8f, 1.6f / std::max(0.25f, camera.zoom));
              DrawLineEx(a, b, thick, Color{255, 255, 255, 60});
              DrawLineEx(a, b, thick * 0.5f, Color{200, 220, 255, 60});
            };

            if (upLand) drawSeg(LerpV(c[0], cTop, shoreT), LerpV(c[1], cTop, shoreT));
            if (rightLand) drawSeg(LerpV(c[1], cRight, shoreT), LerpV(c[2], cRight, shoreT));
            if (downLand) drawSeg(LerpV(c[2], cBottom, shoreT), LerpV(c[3], cBottom, shoreT));
            if (leftLand) drawSeg(LerpV(c[3], cLeft, shoreT), LerpV(c[0], cLeft, shoreT));
          }
        }
      }

      // -----------------------------
      // Building shadow casters (collected in pass 1, rendered in a later shadow pass)
      // -----------------------------
      if (drawShadows) {
        const bool isZone = (tile.overlay == Overlay::Residential || tile.overlay == Overlay::Commercial ||
                             tile.overlay == Overlay::Industrial || tile.overlay == Overlay::Park);
        if (isZone) {
          BuildingShadowCaster caster{};

          if (useMergedZoneBuildings && IsZoneOverlay(tile.overlay) &&
              tileIdx < static_cast<int>(m_zoneParcelsScratch.anchorToParcel.size())) {
            const int parcelId = m_zoneParcelsScratch.anchorToParcel[static_cast<std::size_t>(tileIdx)];
            if (parcelId >= 0 && parcelId < static_cast<int>(m_zoneParcelsScratch.parcels.size())) {
              const ZoneBuildingParcel& p =
                  m_zoneParcelsScratch.parcels[static_cast<std::size_t>(parcelId)];

              if (p.isMultiTile()) {
                if (BuildZoneParcelShadowCaster(world, p, m_elev, tileWf, tileHf, camera.zoom, timeSec, caster)) {
                  shadowCasters.push_back(caster);
                }
              } else {
                if (BuildZoneTileShadowCaster(tile, tileWf, tileHf, camera.zoom, center, caster)) {
                  shadowCasters.push_back(caster);
                }
              }
            }
          } else if (!useMergedZoneBuildings || tile.overlay == Overlay::Park) {
            if (BuildZoneTileShadowCaster(tile, tileWf, tileHf, camera.zoom, center, caster)) {
              shadowCasters.push_back(caster);
            }
          }
        }
      }


    }
  }


  // -----------------------------
  // Shadow pass (decals)
  // -----------------------------
  if (drawShadows && !shadowCasters.empty()) {
    DrawBuildingShadowsPass(shadowCasters, m_shadows, dayNight, weather, tileWf, tileHf, camera.zoom);
  }

  // -----------------------------
  // Pass 2: overlays + structures
  // -----------------------------
  const Rectangle viewRect{viewAABB.minX, viewAABB.minY, viewAABB.maxX - viewAABB.minX, viewAABB.maxY - viewAABB.minY};
  for (int sum = vis.minSum(); sum <= vis.maxSum(); ++sum) {
    const int x0 = std::max(vis.minX, sum - vis.maxY);
    const int x1 = std::min(vis.maxX, sum - vis.minY);

    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      const int tileIdx = y * mapW + x;
      const Tile& t = world.at(x, y);

      float elevPx = 0.0f;
      if (m_elev.maxPixels > 0) {
        elevPx = TileElevationPx(t, m_elev);
      }

      const Vector2 baseCenter{(x - y) * tileWf * 0.5f, (x + y) * tileHf * 0.5f};
      Vector2 center = baseCenter;
      center.y -= elevPx;

      // Screen-space AABB for quick culling.
      const Rectangle tileAABB{center.x - tileWf * 0.5f, center.y - tileHf * 0.5f, tileWf, tileHf};
      if (!CheckCollisionRecs(tileAABB, viewRect)) {
        continue;
      }

      // Lighting (must match pass 1 so overlays/structures agree).
      const TileLighting light = ComputeTileLighting(world, x, y, tileWf, tileHf, m_elev, timeSec, animatedLighting);
      const float baseBrightness = light.base;
      const float brightness = animatedLighting ? light.animated : light.base;

      // -----------------------------
      // District overlay fill (overlay layer)
      // -----------------------------
      if (showDistrictOverlayEff && tileScreenW >= 6.0f) {
        const std::uint8_t did = t.district;
        if (did != 0u) {
          // Soft fill; alpha reduced when zoomed out.
          const float alphaK = std::clamp((tileScreenW - 6.0f) / 18.0f, 0.0f, 1.0f);
          const unsigned char a = static_cast<unsigned char>(40.0f + 80.0f * alphaK);
          DrawDiamond(center, tileWf, tileHf, DistrictFillColor(did, a));
        }
      }

      // -----------------------------
      // Heatmap overlay (overlay layer)
      // -----------------------------
      if (showHeatmap && heatmap) {
        const float v = (*heatmap)[static_cast<std::size_t>(tileIdx)];
        const Color c = HeatmapColor(v, heatmapRamp);
        DrawDiamond(center, tileWf, tileHf, Color{c.r, c.g, c.b, 90});
      }

      // -----------------------------
      // Grid overlay (overlay layer)
      // -----------------------------
      if (drawGridEff && tileScreenW >= 8.0f) {
        const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
        const float w = std::clamp(invZoom * 1.25f, 0.6f * invZoom, 2.2f * invZoom);

        Vector2 corners[4];
        TileDiamondCorners(center, tileWf, tileHf, corners);
        for (int i = 0; i < 4; ++i) {
          const Vector2 a = corners[i];
          const Vector2 b = corners[(i + 1) & 3];
          DrawLineEx(a, b, w, Color{255, 255, 255, 55});
        }
      }

      // -----------------------------
      // District borders (overlay layer)
      // -----------------------------
      if (showDistrictBordersEff && tileScreenW >= 6.0f) {
        const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
        const float w = std::clamp(invZoom * 1.6f, 0.6f * invZoom, 3.0f * invZoom);

        // Check N/E neighbors (avoid double-drawing).
        auto drawEdge = [&](int nx, int ny, int cornerA, int cornerB) {
          if (nx < 0 || ny < 0 || nx >= mapW || ny >= mapH) return;
          const std::uint8_t nd = world.at(nx, ny).district;
          if (nd != t.district) {
            Vector2 corners[4];
            TileDiamondCorners(center, tileWf, tileHf, corners);
            DrawLineEx(corners[cornerA], corners[cornerB], w, Color{0, 0, 0, 160});
            DrawLineEx(corners[cornerA], corners[cornerB], w * 0.5f, Color{255, 255, 255, 70});
          }
        };

        // North edge: between TL (0) and TR (1)
        drawEdge(x, y - 1, 0, 1);
        // East edge: between TR (1) and BR (2)
        drawEdge(x + 1, y, 1, 2);
      }

      // -----------------------------
      // Depth-sorted injected sprites (pre-FX)
      // -----------------------------
      // Draw after per-tile overlays but before buildings/indicators so sprites sit "on top" of ground
      // overlays yet can still be occluded by structures on later diagonals.
      if (!preSprites.empty()) {
        flushPreSprites(sum, static_cast<float>(x));
      }

      // -----------------------------
      // Zone buildings and indicators (structures)
      // -----------------------------
      if (layerStructures && tileScreenW >= 26.0f) {
        const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                             t.overlay == Overlay::Industrial || t.overlay == Overlay::Park);

        if (isZone) {
          if (useMergedZoneBuildings) {
            // For R/C/I we render one building per parcel (at the anchor tile).
            // Parks are not parcelized (keeps them as per-tile small structures).
            if (IsZoneOverlay(t.overlay) && tileIdx < static_cast<int>(m_zoneParcelsScratch.anchorToParcel.size())) {
              const int pid = m_zoneParcelsScratch.anchorToParcel[static_cast<std::size_t>(tileIdx)];
              if (pid >= 0 && pid < static_cast<int>(m_zoneParcelsScratch.parcels.size())) {
                const ZoneBuildingParcel& p = m_zoneParcelsScratch.parcels[static_cast<std::size_t>(pid)];
                if (p.isMultiTile()) {
                  DrawMergedZoneBuildingAndIndicators(p, world, m_elev, tileWf, tileHf, camera.zoom, timeSec);
                } else {
                  // Single-tile parcel: fall back to normal building + indicators.
                  DrawZoneBuilding(t, tileWf, tileHf, camera.zoom, center, brightness);
                  DrawZoneTileIndicators(t, tileWf, tileHf, camera.zoom, center);
                }
              }
            } else {
              // Park or non-parcelized overlay
              DrawZoneBuilding(t, tileWf, tileHf, camera.zoom, center, brightness);
              DrawZoneTileIndicators(t, tileWf, tileHf, camera.zoom, center);
            }
          } else {
            DrawZoneBuilding(t, tileWf, tileHf, camera.zoom, center, brightness);
            DrawZoneTileIndicators(t, tileWf, tileHf, camera.zoom, center);
          }
        }
      }

      // -----------------------------
      // Road indicators (upgrade pips, etc.)
      // -----------------------------
      if (layerStructures && t.overlay == Overlay::Road && tileScreenW >= 18.0f) {
        DrawRoadIndicators(t, tileWf, tileHf, camera.zoom, center, brightness);
      }
    }

    // Flush any remaining sprites anchored to this diagonal sum (including ones with sortX outside the visible range).
    if (!preSprites.empty()) {
      flushPreSprites(sum, std::numeric_limits<float>::infinity());
    }
  }

  // Draw any remaining sprites (usually in front of the visible tile range).
  while (preSpriteIdx < preSprites.size()) {
    const WorldSprite* s = preSprites[preSpriteIdx];
    if (s) drawWorldSprite(*s);
    ++preSpriteIdx;
  }

  // -----------------------------
  // Game/world overlays (pre-FX)
  // -----------------------------
  if (drawBeforeFx) {
    drawBeforeFx(camera);
  }

  // -----------------------------
  // Cloud shadows (decals layer)
  // -----------------------------
  // Draw before grading so the day/night + overcast tint applies on top.
  if (drawAestheticDetails && m_cloudShadows.enabled && m_cloudShadowTex.id != 0 &&
      m_weather.mode != WeatherSettings::Mode::Clear) {
    const float cloudiness = std::clamp(m_weather.overcast, 0.0f, 1.0f);
    if (cloudiness > 0.001f) {
      const float dnMul = (m_dayNight.enabled) ? std::clamp(dayNight.day, 0.0f, 1.0f) : 1.0f;
      const float alpha = std::clamp(m_cloudShadows.strength * cloudiness * dnMul, 0.0f, 1.0f);
      if (alpha > 0.001f) {
        const float pad = tileWf * 2.0f;
        const float dstX = viewAABB.minX - pad;
        const float dstY = viewAABB.minY - pad;
        const float dstW = (viewAABB.maxX - viewAABB.minX) + pad * 2.0f;
        const float dstH = (viewAABB.maxY - viewAABB.minY) + pad * 2.0f;

        const float scale = std::clamp(m_cloudShadows.scale, 0.25f, 8.0f);
        const float worldPeriod = tileWf * 18.0f * scale;
        const float texPerWorld = static_cast<float>(m_cloudShadowTex.width) / worldPeriod;

        const float speedMul = std::max(0.0f, m_cloudShadows.speed);
        const float worldSpeed = tileWf * 0.60f * speedMul * weather.windSpeed;
        const float offX = weather.windX * worldSpeed * timeSec;
        const float offY = weather.windY * worldSpeed * timeSec;

        const Rectangle src{
            (dstX + offX) * texPerWorld,
            (dstY + offY) * texPerWorld,
            dstW * texPerWorld,
            dstH * texPerWorld};
        const Rectangle dst{dstX, dstY, dstW, dstH};

        const unsigned char a = static_cast<unsigned char>(std::round(255.0f * alpha));
        DrawTexturePro(m_cloudShadowTex, src, dst, Vector2{0.0f, 0.0f}, 0.0f, Color{0, 0, 0, a});
      }
    }
  }

  // -----------------------------
  // Weather + day/night grading (decals layer)
  // -----------------------------
  if (drawAestheticDetails && m_weather.affectScreen) {
    const float w = std::clamp(weather.overcast, 0.0f, 1.0f);
    const float dnMul = (m_dayNight.enabled) ? (0.65f + 0.35f * dayNight.day) : 1.0f;
    const float a = std::clamp(w * dnMul, 0.0f, 1.0f);

    const int aWx = static_cast<int>(120.0f * a);
    if (aWx > 0) {
      const float pad = tileWf * 2.0f;
      const Vector2 pos{viewAABB.minX - pad, viewAABB.minY - pad};
      const Vector2 size{(viewAABB.maxX - viewAABB.minX) + pad * 2.0f, (viewAABB.maxY - viewAABB.minY) + pad * 2.0f};

      if (weather.mode == WeatherSettings::Mode::Snow) {
        DrawRectangleV(pos, size, Color{255, 255, 255, static_cast<unsigned char>(aWx)});
      } else {
        DrawRectangleV(pos, size, Color{70, 90, 110, static_cast<unsigned char>(aWx)});
      }
    }
  }

  if (drawAestheticDetails && m_dayNight.enabled) {
    const float nightStrength = 1.0f - dayNight.day;
    const float duskStrength = dayNight.dusk;

    const int aNight = static_cast<int>(220.0f * std::clamp(nightStrength * 0.65f, 0.0f, 1.0f));
    const int aDusk = static_cast<int>(120.0f * std::clamp(duskStrength * 0.55f, 0.0f, 1.0f));

    const float pad = tileWf * 2.0f;
    const Vector2 pos{viewAABB.minX - pad, viewAABB.minY - pad};
    const Vector2 size{(viewAABB.maxX - viewAABB.minX) + pad * 2.0f, (viewAABB.maxY - viewAABB.minY) + pad * 2.0f};

    if (aNight > 0) {
      DrawRectangleV(pos, size, Color{0, 0, 0, static_cast<unsigned char>(aNight)});
    }
    if (aDusk > 0) {
      DrawRectangleV(pos, size, Color{255, 120, 60, static_cast<unsigned char>(aDusk)});
    }
  }

  const bool drawLights = drawAestheticDetails && m_dayNight.enabled && m_dayNight.drawLights &&
                          dayNight.nightLights > 0.01f && tileScreenW >= 24.0f;
  if (drawLights) {
    DrawNightLightsPass(
      world,
      vis,
      tileWf,
      tileHf,
      m_elev,
      camera.zoom,
      timeSec,
      dayNight.nightLights,
      weather.wetness,
      m_weather.reflectLights,
      m_gfxSeed32);
  }

  // -----------------------------
  // Depth-sorted injected sprites (emissive)
  // -----------------------------
  // Draw after grading so emissive elements (e.g., headlights) stay bright at night.
  if (!emissiveSprites.empty()) {
    BeginBlendMode(BLEND_ADDITIVE);
    for (const WorldSprite* s : emissiveSprites) {
      if (s) drawWorldSprite(*s);
    }
    EndBlendMode();
  }

  // -----------------------------
  // Game/world overlays (post-FX)
  // -----------------------------
  if (drawAfterFx) {
    drawAfterFx(camera);
  }

  // -----------------------------
  // Selection/highlight overlays (overlay layer)
  // -----------------------------
  if (layerOverlays) {
    const float thick = std::max(1.0f, 2.0f / std::max(0.25f, camera.zoom));

    const auto drawOutline = [&](int tx, int ty, Color c) {
      if (!world.inBounds(tx, ty)) return;
      const Tile& tt = world.at(tx, ty);
      const float elevPx = TileElevationPx(tt, m_elev);
      const Vector2 baseC = TileToWorldCenter(tx, ty, tileWf, tileHf);
      const Vector2 cc = Vector2{baseC.x, baseC.y - elevPx};

      Vector2 corners[4];
      TileDiamondCorners(cc, tileWf, tileHf, corners);

      DrawLineEx(corners[0], corners[1], thick, c);
      DrawLineEx(corners[1], corners[2], thick, c);
      DrawLineEx(corners[2], corners[3], thick, c);
      DrawLineEx(corners[3], corners[0], thick, c);
    };

    if (highlightPathEff && !highlightPathEff->empty()) {
      for (const Point& p : *highlightPathEff) {
        drawOutline(p.x, p.y, Color{255, 215, 0, 110});
      }
    }

    if (selected) {
      drawOutline(selected->x, selected->y, Color{255, 215, 0, 220});
    }

    if (hovered) {
      const int cx = hovered->x;
      const int cy = hovered->y;

      const int r = std::max(0, brushRadius);
      if (r > 0) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if (std::abs(dx) + std::abs(dy) > r) continue; // diamond brush
            drawOutline(cx + dx, cy + dy, Color{255, 255, 255, 70});
          }
        }
      }

      drawOutline(cx, cy, Color{255, 255, 255, 180});
    }
  }

  EndMode2D();
}



void Renderer::drawWeatherScreenFX(int screenW, int screenH, float timeSec, bool allowAestheticDetails)
{
  if (!allowAestheticDetails) return;

  // Nothing to draw?
  if (m_weather.mode == WeatherSettings::Mode::Clear) return;

  const WeatherState w = ComputeWeatherState(timeSec, m_weather);
  if (w.mode == WeatherSettings::Mode::Clear) return;

  const DayNightState dn = m_dayNight.enabled ? ComputeDayNightState(timeSec, m_dayNight) : DayNightState{};

  // Screen-space fog gradient (top of screen = farther away in iso).
  if (w.fog > 0.01f) {
    const float dnMul = 0.65f + 0.35f * dn.day;
    const int a = static_cast<int>(160.0f * w.fog * dnMul);
    if (a > 0) {
      if (w.mode == WeatherSettings::Mode::Snow) {
        DrawRectangleGradientV(0, 0, screenW, screenH, Color{245, 248, 255, ClampU8(a)}, Color{0, 0, 0, 0});
      } else { // Rain
        DrawRectangleGradientV(0, 0, screenW, screenH, Color{90, 110, 130, ClampU8(a)}, Color{0, 0, 0, 0});
      }
    }
  }

  if (!m_weather.drawParticles) return;
  if (w.intensity <= 0.01f) return;

  const std::uint32_t seed = m_gfxSeed32 ^ 0xA11CE5u;
  const float area = static_cast<float>(screenW) * static_cast<float>(screenH);

  if (w.mode == WeatherSettings::Mode::Rain) {
    const int count = std::clamp<int>(static_cast<int>(area * 0.00012f * w.intensity), 60, 900);

    const float baseLen = 10.0f + 18.0f * w.intensity;
    const float speed = (650.0f + 900.0f * w.intensity) * w.windSpeed;

    const float drift = (w.windY > 0.20f) ? (w.windX / w.windY) : 0.0f;
    const float dnMul = 0.70f + 0.30f * dn.day;

    for (int i = 0; i < count; ++i) {
      const std::uint32_t h0 = HashCoords32(i, i * 17, seed);
      const std::uint32_t h1 = HashCoords32(i, i * 37, seed ^ 0xBEEF123u);
      const std::uint32_t h2 = HashCoords32(i, i * 53, seed ^ 0x1234ABCu);

      float x0 = Frac01(h0) * static_cast<float>(screenW);
      const float phase = Frac01(h1);

      const float wrap = static_cast<float>(screenH) + baseLen + 32.0f;
      float y = std::fmod(timeSec * speed + phase * wrap, wrap) - (baseLen + 20.0f);

      float x = x0 + drift * (y + baseLen) * 0.55f;
      x = std::fmod(x, static_cast<float>(screenW));
      if (x < 0.0f) x += static_cast<float>(screenW);

      const float localLen = baseLen * (0.60f + 0.90f * Frac01(h2));
      Vector2 p0{x, y};
      Vector2 p1{x + w.windX * localLen, y + w.windY * localLen};

      const unsigned char alpha =
        ClampU8(static_cast<int>((18.0f + 88.0f * w.intensity * (0.35f + 0.65f * Frac01(h2 ^ 0x777u))) * dnMul));

      DrawLineEx(p0, p1, 1.0f, Color{210, 220, 235, alpha});
    }
  } else { // Snow
    const int count = std::clamp<int>(static_cast<int>(area * 0.00018f * w.intensity), 120, 1800);

    const float speed = (45.0f + 80.0f * w.intensity) * w.windSpeed;
    const float dnMul = 0.75f + 0.25f * dn.day;

    for (int i = 0; i < count; ++i) {
      const std::uint32_t h0 = HashCoords32(i, i * 19, seed ^ 0x51A5EEDu);
      const std::uint32_t h1 = HashCoords32(i, i * 31, seed ^ 0x7F00BAAu);
      const std::uint32_t h2 = HashCoords32(i, i * 47, seed ^ 0x0DDC0FFEu);

      float x0 = Frac01(h0) * static_cast<float>(screenW);
      const float phase = Frac01(h1);

      const float wrap = static_cast<float>(screenH) + 40.0f;
      float y = std::fmod(timeSec * speed + phase * wrap, wrap) - 20.0f;

      // Gentle drift + a bit of sideways wobble to avoid "straight lines" of flakes.
      const float wobble = 0.8f + 0.4f * std::sin(timeSec * 0.9f + phase * 12.0f);
      float x = x0 + w.windX * (y + 20.0f) * 0.35f * wobble;
      x = std::fmod(x, static_cast<float>(screenW));
      if (x < 0.0f) x += static_cast<float>(screenW);

      const float size = 0.8f + 2.0f * Frac01(h2);
      const unsigned char alpha =
        ClampU8(static_cast<int>((32.0f + 130.0f * w.intensity * (0.25f + 0.75f * Frac01(h2 ^ 0x999u))) * dnMul));

      DrawCircleV(Vector2{x, y}, size, Color{255, 255, 255, alpha});
    }
  }
}

void Renderer::drawHUD(const World& world, const Camera2D& camera, Tool tool, int roadBuildLevel,
                       std::optional<Point> hovered,
                       int screenW, int screenH, bool showHelp, int brushRadius, int undoCount, int redoCount,
                       bool simPaused, float simSpeed, int saveSlot, bool showMinimap, const char* inspectInfo,
                       const char* heatmapInfo)
{
  const Stats& s = world.stats();

  // HUD panel
  const int pad = 12;
  const int panelW = 420;
  // Budget + demand + land value add two always-on HUD lines.
  const int extraLines = 2 + ((inspectInfo && inspectInfo[0] != '\0') ? 1 : 0) +
                         ((heatmapInfo && heatmapInfo[0] != '\0') ? 1 : 0);
  const int panelH = (showHelp ? 360 : 228) + extraLines * 22;

  DrawRectangle(pad, pad, panelW, panelH, Color{0, 0, 0, 150});
  DrawRectangleLines(pad, pad, panelW, panelH, Color{255, 255, 255, 70});

  int y = pad + 10;

  auto line = [&](const std::string& text) {
    DrawText(text.c_str(), pad + 10, y, 18, RAYWHITE);
    y += 22;
  };

  char buf[256];

  std::snprintf(buf, sizeof(buf), "Day: %d    Money: %d    Happiness: %.0f%%", s.day, s.money,
                static_cast<double>(s.happiness * 100.0f));
  line(buf);

  std::snprintf(buf, sizeof(buf), "Sim: %s    Speed: x%.2f", simPaused ? "PAUSED" : "RUNNING", static_cast<double>(simSpeed));
  line(buf);

  {
    const int tradeNet = s.exportRevenue - s.importCost;
    const int net = s.income - s.expenses;
    std::snprintf(buf, sizeof(buf), "Budget: %+d  tax %d  maint %d  trade %+d  upg %d", net, s.taxRevenue, s.maintenanceCost,
                  tradeNet, s.upgradeCost);
    line(buf);
  }

  {
    std::snprintf(buf, sizeof(buf), "Demand: %.0f%%  Land: %.0f%%  Tax/cap: %.2f", static_cast<double>(s.demandResidential * 100.0f),
                  static_cast<double>(s.avgLandValue * 100.0f), static_cast<double>(s.avgTaxPerCapita));
    line(buf);
  }

  // JobsCapacity in the core sim counts *all* job tiles, but not all jobs are necessarily
  // reachable if road networks are disconnected (outside connection rule).
  if (s.jobsCapacityAccessible != s.jobsCapacity) {
    std::snprintf(buf, sizeof(buf), "Pop: %d / %d housing    Jobs: %d / %d access (total %d)", s.population,
                  s.housingCapacity, s.employed, s.jobsCapacityAccessible, s.jobsCapacity);
  } else {
    std::snprintf(buf, sizeof(buf), "Pop: %d / %d housing    Jobs: %d / %d cap", s.population, s.housingCapacity,
                  s.employed, s.jobsCapacity);
  }
  line(buf);

  if (s.commuters > 0) {
    if (s.commutersUnreachable > 0) {
      std::snprintf(buf, sizeof(buf),
                    "Traffic: %d commute (unreach %d)  avg %.1f (t %.1f)  cong %.0f%%",
                    s.commuters, s.commutersUnreachable, static_cast<double>(s.avgCommute),
                    static_cast<double>(s.avgCommuteTime),
                    static_cast<double>(s.trafficCongestion * 100.0f));
    } else {
      std::snprintf(buf, sizeof(buf), "Traffic: %d commute  avg %.1f (t %.1f)  cong %.0f%%", s.commuters,
                    static_cast<double>(s.avgCommute), static_cast<double>(s.avgCommuteTime),
                    static_cast<double>(s.trafficCongestion * 100.0f));
    }
    line(buf);
  } else {
    line("Traffic: (no commuters)");
  }


  if (s.goodsDemand > 0) {
    if (s.goodsUnreachableDemand > 0) {
      std::snprintf(buf, sizeof(buf), "Goods: prod %d  deliv %d/%d (%.0f%%)  unr %d  imp %d  exp %d",
                    s.goodsProduced, s.goodsDelivered, s.goodsDemand,
                    static_cast<double>(s.goodsSatisfaction * 100.0f), s.goodsUnreachableDemand,
                    s.goodsImported, s.goodsExported);
    } else {
      std::snprintf(buf, sizeof(buf), "Goods: prod %d  deliv %d/%d (%.0f%%)  imp %d  exp %d",
                    s.goodsProduced, s.goodsDelivered, s.goodsDemand,
                    static_cast<double>(s.goodsSatisfaction * 100.0f), s.goodsImported, s.goodsExported);
    }
    line(buf);
  } else {
    line("Goods: (no commercial demand)");
  }

  const char* toolName = ToString(tool);
  char toolBuf[64];
  if (tool == Tool::Road) {
    std::snprintf(toolBuf, sizeof(toolBuf), "Road (%s)", RoadClassName(roadBuildLevel));
    toolName = toolBuf;
  }
  std::snprintf(buf, sizeof(buf), "Roads: %d    Parks: %d    Tool: %s    Brush: %d", s.roads, s.parks, toolName,
                brushRadius);
  line(buf);

  // Weather summary for HUD.
  char wxBuf[32];
  if (m_weather.mode == WeatherSettings::Mode::Clear) {
    std::snprintf(wxBuf, sizeof(wxBuf), "%s", WeatherModeName(m_weather.mode));
  } else {
    const int pct = static_cast<int>(std::round(std::clamp(m_weather.intensity, 0.0f, 1.0f) * 100.0f));
    std::snprintf(wxBuf, sizeof(wxBuf), "%s %d%%", WeatherModeName(m_weather.mode), pct);
  }

  if (m_useBandCache) {
    int dirtyBands = 0;
    for (const auto& b : m_bands) {
      if (b.dirtyTerrain || b.dirtyStructures) ++dirtyBands;
    }
    std::snprintf(buf, sizeof(buf), "Undo: %d    Redo: %d    Slot: %d    Cache: ON (dirty %d)    D/N: %s    Wx: %s",
                  undoCount, redoCount, saveSlot, dirtyBands, m_dayNight.enabled ? "ON" : "OFF", wxBuf);
  } else {
    std::snprintf(buf, sizeof(buf), "Undo: %d    Redo: %d    Slot: %d    Cache: OFF    D/N: %s    Wx: %s",
                  undoCount, redoCount, saveSlot, m_dayNight.enabled ? "ON" : "OFF", wxBuf);
  }
  line(buf);

  // Happiness bar
  const int barX = pad + 10;
  const int barY = y + 4;
  const int barW = panelW - 20;
  const int barH = 10;

  DrawRectangle(barX, barY, barW, barH, Color{255, 255, 255, 30});
  DrawRectangle(barX, barY, static_cast<int>(barW * std::clamp(s.happiness, 0.0f, 1.0f)), barH,
                Color{120, 220, 120, 140});
  DrawRectangleLines(barX, barY, barW, barH, Color{255, 255, 255, 50});
  y += 22;

  if (hovered && world.inBounds(hovered->x, hovered->y)) {
    const Tile& t = world.at(hovered->x, hovered->y);
    std::snprintf(buf, sizeof(buf), "Hover: (%d,%d)  %s + %s  dist=%d  h=%.2f  elev=%.0fpx  lvl=%d  occ=%d", hovered->x,
                  hovered->y, ToString(t.terrain), ToString(t.overlay), static_cast<int>(t.district & 7u),
                  static_cast<double>(t.height), static_cast<double>(TileElevationPx(t, m_elev)), t.level,
                  t.occupants);
    DrawText(buf, pad + 10, y + 6, 16, Color{220, 220, 220, 255});
    y += 26;
  }

  if (heatmapInfo && heatmapInfo[0] != '\0') {
    DrawText(heatmapInfo, pad + 10, y + 6, 16, Color{230, 230, 230, 255});
    y += 26;
  }

  if (inspectInfo && inspectInfo[0] != '\0') {
    DrawText(inspectInfo, pad + 10, y + 6, 16, Color{230, 230, 230, 255});
    y += 26;
  }

  if (showHelp) {
    DrawText("Right drag: pan | Wheel: zoom | R regen | G grid | H help | M minimap | E elev | O outside | L heatmap | C vehicles | P policy | F1 report | F2 cache | Shift+F2 day/night | F3 model | Shift+F3 weather | F7 districts | T graph | V traffic | B goods", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("1 Road | 2 Res | 3 Com | 4 Ind | 5 Park | 0 Doze | 6 Raise | 7 Lower | 8 Smooth | 9 District | Q Inspect", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("[/] brush | ,/. district | Space: pause | N: step | +/-: speed | U: road type", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("F4 console | F5 save | F9 load | F6 slot | F10 saves | F12 shot | Ctrl+F12 map | Ctrl+Shift+F12 layers | Ctrl+Z undo | Ctrl+Y redo", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("F8 video (Shift+F8 FX) | F11 fullscreen | Alt+Enter borderless | Ctrl+=/- UI scale | Ctrl+0 UI auto | Ctrl+Alt+=/- world scale", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("Tip: re-place a zone to upgrade. Road: U selects class (paint to upgrade), Shift+drag builds path. Terraform: Shift=strong, Ctrl=fine. District: Alt+click pick, Shift+click fill.", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
  }

  // Minimap overlay (bottom-right). One pixel per tile, scaled up.
  if (showMinimap) {
    ensureMinimapUpToDate(world);
    const MinimapLayout mini = minimapLayout(world, screenW, screenH);

    if (mini.rect.width > 2.0f && mini.rect.height > 2.0f && m_minimapTex.id != 0) {
      // Background + border.
      DrawRectangleRec(mini.rect, Color{0, 0, 0, 140});
      DrawRectangleLines(static_cast<int>(mini.rect.x), static_cast<int>(mini.rect.y), static_cast<int>(mini.rect.width),
                         static_cast<int>(mini.rect.height), Color{255, 255, 255, 70});

      // Draw the minimap texture scaled to the destination rectangle.
      const Rectangle src{0.0f, 0.0f, static_cast<float>(m_minimapW), static_cast<float>(m_minimapH)};
      DrawTexturePro(m_minimapTex, src, mini.rect, Vector2{0, 0}, 0.0f, WHITE);

      // Outline visible world viewport.
      TileRect vis = ComputeVisibleTileRect(camera, screenW, screenH, world.width(), world.height(),
                                                  static_cast<float>(m_tileW), static_cast<float>(m_tileH),
                                                  m_elev.maxPixels);
      const float pixelsPerTile = std::max(1.0e-3f, mini.pixelsPerTile);

      const float vx = mini.rect.x + static_cast<float>(vis.minX) * pixelsPerTile;
      const float vy = mini.rect.y + static_cast<float>(vis.minY) * pixelsPerTile;
      const float vw = static_cast<float>(vis.maxX - vis.minX + 1) * pixelsPerTile;
      const float vh = static_cast<float>(vis.maxY - vis.minY + 1) * pixelsPerTile;

      const int ivx = static_cast<int>(std::floor(vx));
      const int ivy = static_cast<int>(std::floor(vy));
      const int ivw = std::max(1, static_cast<int>(std::ceil(vw)));
      const int ivh = std::max(1, static_cast<int>(std::ceil(vh)));
      DrawRectangleLines(ivx, ivy, ivw, ivh, Color{255, 255, 255, 180});

      // Hovered tile marker.
      if (hovered && world.inBounds(hovered->x, hovered->y)) {
        const int hx = static_cast<int>(std::floor(mini.rect.x + static_cast<float>(hovered->x) * pixelsPerTile));
        const int hy = static_cast<int>(std::floor(mini.rect.y + static_cast<float>(hovered->y) * pixelsPerTile));
        const int hw = std::max(1, static_cast<int>(std::ceil(pixelsPerTile)));
        DrawRectangleLines(hx, hy, hw, hw, Color{255, 255, 0, 200});
      }

      // Camera target marker (approx tile under the camera target).
      if (const auto camTile = WorldToTileElevated(camera.target, world, static_cast<float>(m_tileW),
                                                   static_cast<float>(m_tileH), m_elev)) {
        const float cx = mini.rect.x + (static_cast<float>(camTile->x) + 0.5f) * pixelsPerTile;
        const float cy = mini.rect.y + (static_cast<float>(camTile->y) + 0.5f) * pixelsPerTile;
        const float r = std::clamp(1.0f + 0.35f * pixelsPerTile, 1.0f, 6.0f);
        DrawCircleV(Vector2{cx, cy}, r, Color{255, 255, 255, 190});
        DrawCircleLines(static_cast<int>(cx), static_cast<int>(cy), r + 1.0f, Color{0, 0, 0, 90});
      }

      // Label.
      const int labelY = std::max(0, static_cast<int>(mini.rect.y) - 18);
      DrawText("Minimap (click/drag)", static_cast<int>(mini.rect.x), labelY, 16, Color{230, 230, 230, 230});
    }
  }

  // FPS
  const int fps = GetFPS();
  std::snprintf(buf, sizeof(buf), "FPS: %d", fps);
  DrawText(buf, screenW - 90, 12, 20, Color{255, 255, 255, 200});
}

} // namespace isocity
