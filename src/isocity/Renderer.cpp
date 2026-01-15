#include "isocity/Renderer.hpp"
#include "isocity/Ui.hpp"


#include "isocity/Pathfinding.hpp"
#include "isocity/ZoneAccess.hpp"

#include "isocity/Random.hpp"
#include "isocity/Noise.hpp"
#include "isocity/ZoneMetrics.hpp"
#include "isocity/Road.hpp"
#include "isocity/Traffic.hpp"

#include "isocity/GfxProps.hpp"
#include "isocity/GfxBuildings.hpp"


#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>
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

inline Color LerpColor(Color a, Color b, float t);

inline Vector2 LerpV(const Vector2& a, const Vector2& b, float t)
{
  return Vector2{::isocity::Lerp(a.x, b.x, t), ::isocity::Lerp(a.y, b.y, t)};
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

  // Wind is always computed (even in Clear mode) so other aesthetic systems (e.g. water waves) can use it.
  w.windSpeed = std::clamp(s.windSpeed, 0.05f, 6.0f);

  // Wind direction in degrees (screen-space): 0=right, 90=down.
  const float ang = (s.windAngleDeg) * (kPiF / 180.0f);

  // Add a subtle time-varying gust wobble so motion doesn't look "stamped on".
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

  const float inten = std::clamp(s.intensity, 0.0f, 1.0f);
  if (s.mode == Renderer::WeatherSettings::Mode::Clear) {
    // No precipitation effects in Clear mode.
    return w;
  }

  w.intensity = inten;
  w.overcast = std::clamp(s.overcast, 0.0f, 1.0f) * inten;
  w.fog = std::clamp(s.fog, 0.0f, 1.0f) * inten;

  if (s.mode == Renderer::WeatherSettings::Mode::Rain) {
    w.wetness = inten;
    w.snow = 0.0f;
  } else { // Snow
    w.snow = inten;
    w.wetness = inten * 0.15f; // a little slush sheen
  }

  return w;
}


// ---------------------------------------------------------------------------------------------
// Volumetric clouds shader
// ---------------------------------------------------------------------------------------------
//
// We keep the shader embedded so the project remains asset-free.
// The effect is intentionally stylized and fairly cheap: a small ray-march through a 3D FBM field.
//
// IMPORTANT: We render clouds directly in the world pass (BeginMode2D) instead of an off-screen
// RenderTexture to avoid nested BeginTextureMode() calls (exports already render into an RT).

static const char* kVolumetricCloudVS = R"GLSL(
#version 330

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 2) in vec4 vertexColor;

uniform mat4 mvp;

out vec2 fragTexCoord;
out vec4 fragColor;
out vec2 vWorldPos;

void main()
{
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    vWorldPos = vertexPosition.xy;
    gl_Position = mvp*vec4(vertexPosition, 1.0);
}
)GLSL";

static const char* kVolumetricCloudFS = R"GLSL(
#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
in vec2 vWorldPos;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform vec2 u_viewMin;
uniform vec2 u_viewSize;
uniform float u_time;
uniform vec2 u_windDir;
uniform float u_windSpeed;
uniform float u_scale;
uniform float u_coverage;
uniform float u_density;
uniform float u_softness;
uniform int u_steps;
uniform float u_day;
uniform float u_dusk;
uniform float u_overcast;
uniform float u_seed;
uniform float u_bottomFade;

float hash1(vec3 p)
{
    return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453123);
}

float noise3(vec3 p)
{
    vec3 i = floor(p);
    vec3 f = fract(p);
    vec3 u = f*f*(3.0 - 2.0*f);

    float n000 = hash1(i + vec3(0.0, 0.0, 0.0));
    float n100 = hash1(i + vec3(1.0, 0.0, 0.0));
    float n010 = hash1(i + vec3(0.0, 1.0, 0.0));
    float n110 = hash1(i + vec3(1.0, 1.0, 0.0));
    float n001 = hash1(i + vec3(0.0, 0.0, 1.0));
    float n101 = hash1(i + vec3(1.0, 0.0, 1.0));
    float n011 = hash1(i + vec3(0.0, 1.0, 1.0));
    float n111 = hash1(i + vec3(1.0, 1.0, 1.0));

    float nx00 = mix(n000, n100, u.x);
    float nx10 = mix(n010, n110, u.x);
    float nx01 = mix(n001, n101, u.x);
    float nx11 = mix(n011, n111, u.x);

    float nxy0 = mix(nx00, nx10, u.y);
    float nxy1 = mix(nx01, nx11, u.y);
    return mix(nxy0, nxy1, u.z);
}

float fbm(vec3 p)
{
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 4; ++i) {
        v += a * noise3(p);
        p *= 2.02;
        a *= 0.5;
    }
    return v;
}

float cloudDensity(vec3 p, float cov, float soft)
{
    // Domain warp to break up repetition.
    float w1 = fbm(p + vec3(0.0, 0.0, 0.0));
    float w2 = fbm(p + vec3(5.2, 1.3, 2.1));
    vec3 q = p;
    q.xy += (vec2(w1, w2) - 0.5) * 0.85;

    float n = fbm(q);

    // Higher coverage => lower threshold.
    float thr = mix(0.78, 0.32, clamp(cov, 0.0, 1.0));
    float edge = mix(0.04, 0.18, clamp(soft, 0.0, 1.0));
    float m = smoothstep(thr - edge, thr + edge, n);

    // Vertical shaping: strongest in the middle of the volume.
    float h = smoothstep(0.0, 0.18, q.z) * (1.0 - smoothstep(0.72, 1.0, q.z));
    m *= h;

    // Thicker centers.
    m *= (0.55 + 0.75 * n);
    return clamp(m, 0.0, 1.0);
}

void main()
{
    vec2 uv = (vWorldPos - u_viewMin) / max(u_viewSize, vec2(0.001));

    // World-space -> noise-space.
    vec2 seedOff = vec2(u_seed * 0.00123, u_seed * 0.00173);
    vec2 base = vWorldPos * u_scale + seedOff;
    vec2 wind = u_windDir * (u_time * u_windSpeed);

    // A small internal evolution so clouds "breathe" even if wind is still.
    float evol = u_time * 0.05;

    // Early-out: if a mid-slice is empty, skip the expensive ray-march.
    float c0 = cloudDensity(vec3(base + wind, 0.45 + evol), u_coverage, u_softness);
    if (c0 <= 0.01) {
        finalColor = vec4(0.0);
        return;
    }

    int steps = clamp(u_steps, 8, 64);
    float stepSize = 1.0 / float(steps);

    float alpha = 0.0;
    vec3 col = vec3(0.0);

    // Light direction in noise space (roughly "from upper-left" with a downward component).
    vec3 lightDir = normalize(vec3(-0.55, -0.25, 0.90));

    for (int i = 0; i < 64; ++i) {
        if (i >= steps) break;

        float z = (float(i) + 0.5) * stepSize;
        vec3 p = vec3(base + wind, z + evol);

        float d = cloudDensity(p, u_coverage, u_softness) * c0;
        if (d <= 0.001) continue;

        // Cheap self-shadowing: probe density toward the light.
        float dl = cloudDensity(p + lightDir * 0.35, u_coverage, u_softness);
        float light = clamp(0.35 + 0.65 * (1.0 - dl), 0.0, 1.0);

        // Convert density to alpha contribution.
        float a = clamp(d * u_density * stepSize * 1.45, 0.0, 1.0);

        vec3 sampleCol = mix(vec3(0.55, 0.60, 0.68), vec3(1.0), light);
        col += (1.0 - alpha) * sampleCol * a;
        alpha += (1.0 - alpha) * a;

        if (alpha > 0.985) break;
    }

    // Screen readability: fade clouds toward the bottom of the view.
    float fade = 1.0 - smoothstep(0.55, 0.98, uv.y);
    float fadeMix = mix(1.0, fade, clamp(u_bottomFade, 0.0, 1.0));
    col *= fadeMix;
    alpha *= fadeMix;

    // Day/night tinting (keep subtle; night clouds are darker/less present).
    float day = clamp(u_day, 0.0, 1.0);
    float dusk = clamp(u_dusk, 0.0, 1.0);
    float oc = clamp(u_overcast, 0.0, 1.0);

    vec3 dayTint = vec3(1.02, 1.02, 1.05);
    vec3 duskTint = vec3(1.12, 0.92, 0.78);
    vec3 nightTint = vec3(0.38, 0.42, 0.55);

    vec3 tint = mix(nightTint, dayTint, day);
    tint = mix(tint, duskTint, dusk * 0.75);

    // Overcast makes clouds denser/darker.
    tint *= mix(1.08, 0.85, oc);
    alpha *= mix(0.60, 1.00, oc);

    col *= tint;

    vec4 texel = texture(texture0, fragTexCoord);
    finalColor = vec4(col, clamp(alpha, 0.0, 1.0)) * texel * colDiffuse * fragColor;
}
)GLSL";


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
  // Soft radial glow used by emissive lights (streetlights, windows, signage).
  //
  // We intentionally stay 100% procedural: no textures, no shaders. A small stack of
  // circle gradients approximates a Gaussian falloff well enough for tiny light sprites.
  if (radius <= 0.01f) return;
  if (outer.a == 0 && inner.a == 0) return;

  auto alphaMul = [](Color c, float k) -> Color {
    c.a = ClampU8(static_cast<int>(std::round(static_cast<float>(c.a) * k)));
    return c;
  };

  const int cx = static_cast<int>(std::round(p.x));
  const int cy = static_cast<int>(std::round(p.y));

  // Outer haze: broad and very faint.
  const Color oWide = alphaMul(outer, 0.28f);
  const Color oMid = alphaMul(outer, 0.55f);
  const Color oFade = Color{outer.r, outer.g, outer.b, 0};

  // Inner core: tighter and brighter.
  const Color iMid = alphaMul(inner, 0.70f);
  const Color iFade = Color{inner.r, inner.g, inner.b, 0};

  // Two gradients + a small core disk.
  DrawCircleGradient(cx, cy, radius * 2.15f, oWide, oFade);
  DrawCircleGradient(cx, cy, radius * 1.15f, oMid, oFade);
  DrawCircleGradient(cx, cy, radius * 0.65f, iMid, iFade);
  DrawCircleV(p, radius * 0.32f, inner);
}

// ---------------------------------------------------------------------------------------------
// Terrain macro variation + coastline helpers (purely procedural)
// ---------------------------------------------------------------------------------------------

// Multiply two raylib tint colors (component-wise).
//
// Note: raylib's tint is multiplicative (0..255). We keep the alpha at 255 here and treat opacity
// separately via geometry alpha.
inline Color MulTints(Color a, Color b)
{
  auto mul = [](unsigned char x, unsigned char y) -> unsigned char {
    const int v = (static_cast<int>(x) * static_cast<int>(y) + 127) / 255;
    return ClampU8(v);
  };
  return Color{mul(a.r, b.r), mul(a.g, b.g), mul(a.b, b.b), 255};
}

inline Color TintFromMul(float r, float g, float b, unsigned char a = 255)
{
  const int rr = static_cast<int>(std::round(255.0f * std::clamp(r, 0.0f, 1.0f)));
  const int gg = static_cast<int>(std::round(255.0f * std::clamp(g, 0.0f, 1.0f)));
  const int bb = static_cast<int>(std::round(255.0f * std::clamp(b, 0.0f, 1.0f)));
  return Color{ClampU8(rr), ClampU8(gg), ClampU8(bb), a};
}

inline float TileSlope01Fast(const World& world, int x, int y)
{
  const int w = world.width();
  const int h = world.height();

  auto hAt = [&](int ix, int iy) -> float {
    ix = std::clamp(ix, 0, w - 1);
    iy = std::clamp(iy, 0, h - 1);
    return world.at(ix, iy).height;
  };

  const float dx = std::fabs(hAt(x + 1, y) - hAt(x - 1, y));
  const float dy = std::fabs(hAt(x, y + 1) - hAt(x, y - 1));
  const float slope = std::sqrt(dx * dx + dy * dy);
  return std::clamp(slope * 1.25f, 0.0f, 1.0f);
}

inline int ManhattanDistanceToNonWater(const World& world, int x, int y, int maxR)
{
  const int w = world.width();
  const int h = world.height();
  auto isLand = [&](int ix, int iy) -> bool {
    // Treat OOB as land to keep edges shallow and avoid out-of-bounds branching in the caller.
    if (ix < 0 || iy < 0 || ix >= w || iy >= h) return true;
    return (world.at(ix, iy).terrain != Terrain::Water);
  };

  for (int r = 1; r <= maxR; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      const int dy = r - std::abs(dx);
      if (isLand(x + dx, y + dy)) return r;
      if (dy != 0 && isLand(x + dx, y - dy)) return r;
    }
  }
  return maxR + 1;
}

inline float MacroField01(int tx, int ty, std::uint32_t seed32)
{
  const float x = static_cast<float>(tx);
  const float y = static_cast<float>(ty);

  // Low-frequency domain-warped value noise (cheap, stable, non-tiling).
  const float warpF = 0.012f;
  const float baseF = 0.035f;

  const float wx = ValueNoise2D(x * warpF + 19.37f, y * warpF + 47.11f, seed32 ^ 0x68BC21EBu);
  const float wy = ValueNoise2D(x * warpF - 31.17f, y * warpF + 11.83f, seed32 ^ 0x02E5BE93u);

  const float dx = (wx * 2.0f - 1.0f) * 1.35f;
  const float dy = (wy * 2.0f - 1.0f) * 1.35f;

  const float n0 = ValueNoise2D((x + dx) * baseF, (y + dy) * baseF, seed32 ^ 0xA5A5A5A5u);
  const float n1 = ValueNoise2D(x * 0.11f, y * 0.11f, seed32 ^ 0x0BADC0DEu);

  return std::clamp(0.75f * n0 + 0.25f * n1, 0.0f, 1.0f);
}

struct TerrainMacroVisual {
  Color tint = Color{255, 255, 255, 255};  // multiplicative tint for base textures
  float rock = 0.0f;                       // 0..1 (steepness -> exposed rock)
  float snow = 0.0f;                       // 0..1 (altitude -> permanent snow cap)
  float waterDepth = 0.0f;                 // 0..1 (distance from shore)
};

inline TerrainMacroVisual ComputeTerrainMacroVisual(const World& world, int x, int y, const Tile& t,
                                                   std::uint32_t seed32)
{
  TerrainMacroVisual v{};

  const float macro = MacroField01(x, y, seed32);

  if (t.terrain == Terrain::Grass) {
    const float dry = macro;
    float r = ::isocity::Lerp(0.90f, 1.00f, dry);
    float g = ::isocity::Lerp(1.00f, 0.90f, dry);
    float b = ::isocity::Lerp(0.90f, 0.75f, dry);

    const float slope = TileSlope01Fast(world, x, y);
    v.rock = SmoothStep(0.16f, 0.42f, slope);

    // Exposed rock desaturates/darkens steep slopes.
    r = ::isocity::Lerp(r, 0.78f, v.rock);
    g = ::isocity::Lerp(g, 0.78f, v.rock);
    b = ::isocity::Lerp(b, 0.78f, v.rock);

    // High-altitude snowline (independent of weather), reduced on very steep rock.
    v.snow = SmoothStep(0.76f, 0.90f, t.height) * (1.0f - 0.65f * v.rock);

    v.tint = TintFromMul(r, g, b);
  } else if (t.terrain == Terrain::Sand) {
    // Sand variation: pale beige -> warmer/redder dunes.
    const float warm = SmoothStep(0.35f, 0.85f, macro);
    const float r = 1.00f;
    const float g = ::isocity::Lerp(1.00f, 0.88f, warm);
    const float b = ::isocity::Lerp(1.00f, 0.80f, warm);
    v.tint = TintFromMul(r, g, b);
  } else if (t.terrain == Terrain::Water) {
    constexpr int kMaxR = 3;
    const int d = ManhattanDistanceToNonWater(world, x, y, kMaxR);
    v.waterDepth = std::clamp(static_cast<float>(d - 1) / static_cast<float>(kMaxR), 0.0f, 1.0f);

    // Shallow water stays brighter/turquoise; deep water gets darker/bluer.
    float r = ::isocity::Lerp(0.92f, 0.72f, v.waterDepth);
    float g = ::isocity::Lerp(0.95f, 0.78f, v.waterDepth);
    float b = ::isocity::Lerp(1.00f, 0.88f, v.waterDepth);

    // Patchy algae tint near shore (macro-driven).
    const float algae = SmoothStep(0.55f, 0.85f, macro) * (1.0f - 0.60f * v.waterDepth);
    r *= (1.0f - 0.10f * algae);
    b *= (1.0f - 0.14f * algae);

    v.tint = TintFromMul(r, g, b);
  }

  return v;
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

inline float DotV(const Vector2& a, const Vector2& b) { return a.x * b.x + a.y * b.y; }

inline Vector2 NormalizeV(Vector2 v)
{
  const float len2 = v.x * v.x + v.y * v.y;
  if (len2 > 1.0e-6f) {
    const float inv = 1.0f / std::sqrt(len2);
    v.x *= inv;
    v.y *= inv;
  } else {
    v = Vector2{0.0f, 1.0f};
  }
  return v;
}

inline float Dist2V(const Vector2& a, const Vector2& b)
{
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return dx * dx + dy * dy;
}

// Intersect the infinite line dot(p, dir) == d with a segment p0->p1 (in world space).
static bool IntersectIsoLineWithSegment(const Vector2& p0, const Vector2& p1, const Vector2& dir, float d, Vector2& out)
{
  const float s0 = DotV(p0, dir);
  const float s1 = DotV(p1, dir);

  const float minS = std::min(s0, s1);
  const float maxS = std::max(s0, s1);
  if (d < minS || d > maxS) return false;

  const float denom = (s1 - s0);
  if (std::fabs(denom) < 1.0e-6f) return false;

  const float t = (d - s0) / denom;
  out = Vector2{p0.x + (p1.x - p0.x) * t, p0.y + (p1.y - p0.y) * t};
  return true;
}

static int AddUniquePoint(Vector2* pts, int count, const Vector2& p, float eps2)
{
  for (int i = 0; i < count; ++i) {
    if (Dist2V(pts[i], p) <= eps2) return count;
  }
  pts[count++] = p;
  return count;
}

static bool FarthestPair(const Vector2* pts, int count, Vector2& a, Vector2& b)
{
  if (count < 2) return false;
  float best = -1.0f;
  for (int i = 0; i < count; ++i) {
    for (int j = i + 1; j < count; ++j) {
      const float d2 = Dist2V(pts[i], pts[j]);
      if (d2 > best) {
        best = d2;
        a = pts[i];
        b = pts[j];
      }
    }
  }
  return (best > 1.0e-8f);
}

static void DrawWaveFrontsInDiamond(const Vector2* corners, const Vector2& dir, float timeSec, float speed, float waveLen,
                                   float invZoom, float alphaScale, float pulseSeed,
                                   float brightness, float mul)
{
  // Project the tile polygon into 1D (along dir) so we can draw a few global, time-moving wave fronts.
  float dMin = std::numeric_limits<float>::infinity();
  float dMax = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < 4; ++i) {
    const float d = DotV(corners[i], dir);
    dMin = std::min(dMin, d);
    dMax = std::max(dMax, d);
  }

  // Expand slightly so crests don't pop at exact boundaries.
  const float L = std::max(4.0f, waveLen);
  const float pad = L * 0.75f;
  dMin -= pad;
  dMax += pad;

  const float phase = timeSec * speed;

  const int n0 = static_cast<int>(std::floor((dMin + phase) / L));
  const int n1 = static_cast<int>(std::ceil((dMax + phase) / L));

  const float thickWide = std::clamp(1.65f * invZoom, 0.85f * invZoom, 2.8f * invZoom);
  const float thickThin = std::clamp(0.95f * invZoom, 0.55f * invZoom, 2.0f * invZoom);

  for (int n = n0; n <= n1; ++n) {
    const float d = static_cast<float>(n) * L - phase;

    Vector2 hits[4];
    int hitCount = 0;
    const float eps2 = (0.40f * invZoom) * (0.40f * invZoom);

    for (int e = 0; e < 4; ++e) {
      const Vector2 p0 = corners[e];
      const Vector2 p1 = corners[(e + 1) & 3];
      Vector2 p{};
      if (IntersectIsoLineWithSegment(p0, p1, dir, d, p)) {
        hitCount = AddUniquePoint(hits, hitCount, p, eps2);
        if (hitCount >= 4) break;
      }
    }

    Vector2 a{}, b{};
    if (!FarthestPair(hits, hitCount, a, b)) continue;

    // A little per-front pulsing keeps water from looking like perfectly rigid stripes.
    const float p = 0.70f + 0.30f * std::sin(timeSec * 1.25f + static_cast<float>(n) * 1.37f + pulseSeed);

    const unsigned char aWide = ClampU8(static_cast<int>(28.0f * alphaScale * p));
    const unsigned char aThin = ClampU8(static_cast<int>(62.0f * alphaScale * p));

    const Color wide = ShadeDetail(Color{175, 215, 255, 255}, brightness, mul * 0.98f, aWide);
    const Color thin = ShadeDetail(Color{235, 248, 255, 255}, brightness, mul * 1.06f, aThin);

    DrawLineEx(a, b, thickWide, wide);
    DrawLineEx(a, b, thickThin, thin);
  }
}

static void DrawProceduralTileDetails(const World& world, int x, int y, const Tile& t, const Vector2& center,
                                     float tileW, float tileH, float zoom, float brightness, std::uint32_t seed32,
                                     float timeSec, const WeatherState& weather)
{
  // Purely aesthetic; roads already have their own markings pass.
  if (t.overlay == Overlay::Road) return;

  const float tileScreenW = tileW * zoom;

  // Water benefits from motion even when slightly zoomed out; land micro-details stay more zoom-gated.
  const float minDetailW = (t.terrain == Terrain::Water) ? 20.0f : 30.0f;
  if (tileScreenW < minDetailW) return;

  const std::uint32_t base = seed32 ^ (static_cast<std::uint32_t>(t.variation) * 0x9E3779B9u);
  const std::uint32_t h0 = HashCoords32(x, y, base ^ 0xA5A5A5A5u);

  const bool isPark = (t.overlay == Overlay::Park);

  // -----------------------------
  // Zoned-tile lot decals (procedural)
  // -----------------------------
  // Under high zoom, zoned tiles benefit from small ground decals (sidewalks, driveways, parking stripes)
  // so the city reads as a place rather than flat color blocks. This stays purely draw-time and fully
  // deterministic (no new simulation fields).
  if (IsZoneOverlay(t.overlay) && t.terrain != Terrain::Water) {
    // Keep these subtle and zoom-gated so they don't fight the UI when zoomed out.
    if (tileScreenW < 40.0f) return;

    auto isRoadAt = [&](int px, int py) -> bool {
      return world.inBounds(px, py) && world.at(px, py).overlay == Overlay::Road;
    };

    // Frontage: we orient "lot" details toward an adjacent road if possible.
    std::uint8_t rm = 0;
    if (isRoadAt(x, y - 1)) rm |= 0x01u;
    if (isRoadAt(x + 1, y)) rm |= 0x02u;
    if (isRoadAt(x, y + 1)) rm |= 0x04u;
    if (isRoadAt(x - 1, y)) rm |= 0x08u;

    if (rm == 0) return;

    // Pick a deterministic frontage edge when multiple roads touch this tile.
    int edges[4];
    int edgeCount = 0;
    for (int e = 0; e < 4; ++e) {
      const std::uint8_t bit = static_cast<std::uint8_t>(1u << e);
      if ((rm & bit) != 0u) edges[edgeCount++] = e;
    }

    const std::uint32_t hz = HashCoords32(x, y, base ^ 0xC0FFEE77u);
    const int frontEdge = edges[hz % static_cast<std::uint32_t>(std::max(1, edgeCount))];

    Vector2 c[4];
    TileDiamondCorners(center, tileW, tileH, c);
    const Vector2 edgeA[4] = {c[0], c[1], c[2], c[3]};
    const Vector2 edgeB[4] = {c[1], c[2], c[3], c[0]};

    const float invZoom = 1.0f / std::max(0.001f, zoom);
    const float line = std::clamp(1.35f * invZoom, 0.70f * invZoom, 2.4f * invZoom);

    auto drawBand = [&](int edge, float inset0, float inset1, Color col) {
      Vector2 a0 = LerpV(edgeA[edge], center, inset0);
      Vector2 a1 = LerpV(edgeB[edge], center, inset0);
      Vector2 b0 = LerpV(edgeA[edge], center, inset1);
      Vector2 b1 = LerpV(edgeB[edge], center, inset1);
      DrawTriangle(a0, a1, b1, col);
      DrawTriangle(a0, b1, b0, col);
    };

    // Inward direction (from the frontage edge toward tile center).
    Vector2 emid = LerpV(edgeA[frontEdge], edgeB[frontEdge], 0.5f);
    Vector2 inDir{center.x - emid.x, center.y - emid.y};
    float il2 = inDir.x * inDir.x + inDir.y * inDir.y;
    if (il2 > 1.0e-6f) {
      const float inv = 1.0f / std::sqrt(il2);
      inDir.x *= inv;
      inDir.y *= inv;
    } else {
      inDir = Vector2{0.0f, 1.0f};
    }

    // Edge tangent (used for parked-car orientation).
    Vector2 along{edgeB[frontEdge].x - edgeA[frontEdge].x, edgeB[frontEdge].y - edgeA[frontEdge].y};
    float al2 = along.x * along.x + along.y * along.y;
    if (al2 > 1.0e-6f) {
      const float inv = 1.0f / std::sqrt(al2);
      along.x *= inv;
      along.y *= inv;
    } else {
      along = Vector2{1.0f, 0.0f};
    }
    Vector2 perp{-along.y, along.x};

    // Alpha ramps up a little with zoom for legibility.
    const float zT = std::clamp((tileScreenW - 40.0f) / 38.0f, 0.0f, 1.0f);
    const unsigned char aBase = ClampU8(static_cast<int>(70.0f + 95.0f * zT));

    // Shared curb/sidewalk strip at the frontage.
    const Color curbEdge = ShadeDetail(Color{35, 35, 38, 255}, brightness, 0.92f,
                                       ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.65f)));

    Color sidewalk = ShadeDetail(Color{200, 198, 190, 255}, brightness, 1.02f, aBase);
    if (t.overlay == Overlay::Commercial) {
      sidewalk = ShadeDetail(Color{205, 205, 208, 255}, brightness, 1.02f, aBase);
    } else if (t.overlay == Overlay::Industrial) {
      sidewalk = ShadeDetail(Color{185, 185, 190, 255}, brightness, 0.98f,
                             ClampU8(static_cast<int>(static_cast<float>(aBase) * 1.05f)));
    }

    drawBand(frontEdge, 0.06f, 0.16f, sidewalk);

    // Thin curb line at the road edge.
    {
      Vector2 a = LerpV(edgeA[frontEdge], center, 0.06f);
      Vector2 b = LerpV(edgeB[frontEdge], center, 0.06f);
      DrawLineEx(a, b, line, curbEdge);
    }

    // Interior details by zone type.
    if (t.overlay == Overlay::Residential) {
      // Driveway + walkway.
      const float side = ((hz >> 3u) & 1u) ? 0.32f : 0.68f;
      Vector2 driveEdge = LerpV(edgeA[frontEdge], edgeB[frontEdge], side);
      Vector2 d0 = LerpV(driveEdge, center, 0.09f);
      Vector2 d1 = LerpV(driveEdge, center, 0.58f);

      const float wDrive = tileH * (0.085f + 0.020f * Frac01(hz ^ 0x01234567u));
      const Color driveEdgeC = ShadeDetail(Color{25, 25, 28, 255}, brightness, 0.88f,
                                          ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.55f)));
      const Color driveFillC = ShadeDetail(Color{75, 78, 86, 255}, brightness, 0.92f,
                                          ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.75f)));
      DrawLineEx(d0, d1, wDrive * 1.18f, driveEdgeC);
      DrawLineEx(d0, d1, wDrive, driveFillC);

      // Walkway toward the house center.
      Vector2 w0 = LerpV(d1, center, 0.25f);
      Vector2 w1 = LerpV(center, w0, 0.35f);
      const float wWalk = tileH * 0.040f;
      const Color walkC = ShadeDetail(Color{210, 198, 170, 255}, brightness, 1.04f,
                                      ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.75f)));
      DrawLineEx(w0, w1, wWalk, walkC);

      // Tiny mailbox near the curb when very zoomed in.
      if (tileScreenW >= 70.0f) {
        Vector2 mb = LerpV(driveEdge, center, 0.04f);
        mb.x += perp.x * tileH * 0.03f;
        mb.y += perp.y * tileH * 0.03f;

        const float s = tileH * 0.030f;
        DrawRectangleV(Vector2{mb.x - s * 0.35f, mb.y - s * 0.55f}, Vector2{s * 0.70f, s * 0.40f},
                       ShadeDetail(Color{60, 60, 65, 255}, brightness, 0.95f, 170));
        DrawLineEx(Vector2{mb.x, mb.y - s * 0.15f}, Vector2{mb.x, mb.y + s * 0.50f}, line * 0.65f,
                   ShadeDetail(Color{40, 40, 45, 255}, brightness, 0.90f, 170));
      }
    } else if (t.overlay == Overlay::Commercial) {
      // Parking lot: pad + stripes + occasional parked cars.
      const Color pad = ShadeDetail(Color{170, 170, 175, 255}, brightness, 0.98f,
                                   ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.80f)));
      const Color padEdge = ShadeDetail(Color{55, 55, 58, 255}, brightness, 0.90f,
                                       ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.65f)));

      drawBand(frontEdge, 0.16f, 0.40f, pad);

      // Interior border line.
      {
        Vector2 a = LerpV(edgeA[frontEdge], center, 0.40f);
        Vector2 b = LerpV(edgeB[frontEdge], center, 0.40f);
        DrawLineEx(a, b, line * 0.85f, padEdge);
      }

      if (tileScreenW >= 54.0f) {
        const int stripes = (tileScreenW >= 74.0f) ? 6 : 5;
        const Color stripe = ShadeDetail(Color{250, 250, 245, 255}, brightness, 1.08f,
                                         ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.75f)));
        const float stripeW = std::clamp(1.05f * invZoom, 0.55f * invZoom, 1.8f * invZoom);
        const float len = tileH * (0.11f + 0.02f * Frac01(hz ^ 0x0BADC0DEu));

        for (int i = 0; i < stripes; ++i) {
          const float t01 = 0.20f + (static_cast<float>(i) / static_cast<float>(std::max(1, stripes - 1))) * 0.60f;
          Vector2 baseP = LerpV(edgeA[frontEdge], edgeB[frontEdge], t01);
          baseP = LerpV(baseP, center, 0.22f);
          Vector2 p1{baseP.x + inDir.x * len, baseP.y + inDir.y * len};
          DrawLineEx(baseP, p1, stripeW, stripe);
        }

        // Parked cars (tiny rectangles) scale with occupancy.
        const int cap = std::max(1, CapacityForTile(t));
        const float occ = std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f);
        const int cars =
          std::clamp(static_cast<int>(occ * 3.2f + 0.25f * Frac01(hz ^ 0x13579BDFu)), 0, 3);

        if (cars > 0 && tileScreenW >= 78.0f) {
          for (int cIdx = 0; cIdx < cars; ++cIdx) {
            const std::uint32_t hc = HashCoords32(x + cIdx * 17, y - cIdx * 23, hz ^ 0xDEADBEEFu);
            const float tt = 0.30f + 0.40f * Frac01(hc);
            Vector2 p = LerpV(edgeA[frontEdge], edgeB[frontEdge], tt);
            p = LerpV(p, center, 0.30f + 0.06f * Frac01(hc ^ 0x9E3779B9u));

            const float cw = tileH * 0.090f;
            const float ch = tileH * 0.045f;

            Rectangle rc{p.x - cw * 0.5f, p.y - ch * 0.5f, cw, ch};
            const Vector2 origin{cw * 0.5f, ch * 0.5f};
            const float ang = std::atan2(inDir.y, inDir.x) * 57.2957795f;

            Color carC = Color{
              static_cast<unsigned char>(90 + (hc & 63u)),
              static_cast<unsigned char>(80 + ((hc >> 6) & 63u)),
              static_cast<unsigned char>(85 + ((hc >> 12) & 63u)),
              200};
            carC = ShadeDetail(carC, brightness, 1.00f, ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.90f)));
            DrawRectanglePro(rc, origin, ang, carC);

            // Windshield highlight.
            Color win = ShadeDetail(Color{210, 230, 240, 255}, brightness, 1.05f,
                                   ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.55f)));
            Vector2 p0{p.x - along.x * cw * 0.25f - inDir.x * ch * 0.10f,
                       p.y - along.y * cw * 0.25f - inDir.y * ch * 0.10f};
            Vector2 p1{p.x + along.x * cw * 0.25f - inDir.x * ch * 0.10f,
                       p.y + along.y * cw * 0.25f - inDir.y * ch * 0.10f};
            DrawLineEx(p0, p1, stripeW * 0.85f, win);
          }
        }
      }
    } else if (t.overlay == Overlay::Industrial) {
      // Loading pad with hazard striping.
      const Color pad = ShadeDetail(Color{150, 150, 155, 255}, brightness, 0.95f,
                                   ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.88f)));
      const Color padEdge = ShadeDetail(Color{45, 45, 48, 255}, brightness, 0.90f,
                                       ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.70f)));

      drawBand(frontEdge, 0.16f, 0.46f, pad);

      {
        Vector2 a = LerpV(edgeA[frontEdge], center, 0.46f);
        Vector2 b = LerpV(edgeB[frontEdge], center, 0.46f);
        DrawLineEx(a, b, line * 0.90f, padEdge);
      }

      if (tileScreenW >= 60.0f) {
        const int stripes = 6;
        const Color yel = ShadeDetail(Color{250, 215, 80, 255}, brightness, 1.05f,
                                     ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.70f)));
        const Color blk = ShadeDetail(Color{20, 20, 22, 255}, brightness, 0.90f,
                                     ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.65f)));

        // Stripe direction: diagonal across the pad.
        Vector2 diag{along.x + inDir.x * 0.65f, along.y + inDir.y * 0.65f};
        float dl2 = diag.x * diag.x + diag.y * diag.y;
        if (dl2 > 1.0e-6f) {
          const float inv = 1.0f / std::sqrt(dl2);
          diag.x *= inv;
          diag.y *= inv;
        }

        for (int i = 0; i < stripes; ++i) {
          const float t01 = 0.18f + (static_cast<float>(i) / static_cast<float>(stripes - 1)) * 0.64f;
          Vector2 baseP = LerpV(edgeA[frontEdge], edgeB[frontEdge], t01);
          baseP = LerpV(baseP, center, 0.28f);

          Vector2 p0{baseP.x - diag.x * tileH * 0.06f, baseP.y - diag.y * tileH * 0.06f};
          Vector2 p1{baseP.x + diag.x * tileH * 0.06f, baseP.y + diag.y * tileH * 0.06f};
          DrawLineEx(p0, p1, line * 0.85f, (i & 1) ? yel : blk);
        }
      }

      // A couple of pallets/crates toward the back when extremely zoomed in.
      if (tileScreenW >= 80.0f) {
        const int crates = 1 + static_cast<int>((hz >> 30) & 1u);
        for (int i = 0; i < crates; ++i) {
          Vector2 p = DeterministicDiamondPoint(x, y, hz ^ 0xA55A5AA5u, 100 + i, center, tileW, tileH, 0.55f);
          const float w = tileH * 0.11f;
          const float h = tileH * 0.07f;

          Rectangle rc{p.x - w * 0.5f, p.y - h * 0.5f, w, h};
          const Vector2 origin{w * 0.5f, h * 0.5f};
          const float ang = std::atan2(along.y, along.x) * 57.2957795f;

          Color box = ShadeDetail(Color{95, 70, 45, 255}, brightness, 0.95f,
                                 ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.90f)));
          DrawRectanglePro(rc, origin, ang, box);
          DrawRectangleLinesEx(
            rc,
            line * 0.65f,
            ShadeDetail(Color{30, 25, 20, 255}, brightness, 0.90f,
                       ClampU8(static_cast<int>(static_cast<float>(aBase) * 0.60f))));
        }
      }
    }

    return;
  }


  const float invZoom = 1.0f / std::max(0.001f, zoom);
  const float thick = 1.15f * invZoom;

  if (t.terrain == Terrain::Grass) {
    // Grass tufts + occasional flowers. Parks get more/larger tree canopies.
    const float slope = TileSlope01Fast(world, x, y);
    const float rock = SmoothStep(0.22f, 0.58f, slope);

    int tufts = std::clamp(1 + static_cast<int>((h0 >> 28) & 0x3u), 1, isPark ? 4 : 3);
    if (rock > 0.35f) tufts = std::max(1, tufts - 1);

    // Rocky slopes get a few exposed stones to break up large grassy mountains.
    if (!isPark && rock > 0.12f) {
      const int stones = std::clamp(1 + static_cast<int>(rock * 3.2f) + static_cast<int>((h0 >> 25) & 1u), 1, 4);
      for (int i = 0; i < stones; ++i) {
        const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0xFEEDC0DEu, 40 + i, center, tileW, tileH, 0.86f);
        const float r = tileH * (0.020f + 0.018f * Frac01(HashCoords32(x + i * 19, y - i * 13, base ^ 0x9E3779B9u)));
        const Color stone = ShadeDetail(Color{95, 95, 105, 255}, brightness, 0.95f, 150);
        const Color hi = ShadeDetail(Color{170, 170, 180, 255}, brightness, 1.05f, 85);
        DrawCircleV(p, r, stone);
        DrawCircleV(Vector2{p.x - r * 0.22f, p.y - r * 0.20f}, r * 0.55f, hi);
      }
    }

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
    // Animated, fully procedural wave fronts (global continuous stripes) + a few sparkles.
    // Skip bridges for clarity.
    if (t.overlay == Overlay::Road) return;

    // Fade in with zoom so distant water stays readable.
    const float zT = std::clamp((tileScreenW - 20.0f) / 34.0f, 0.0f, 1.0f);

    // Attenuate waves near shore: the dedicated coastline foam pass already adds a lot of detail there.
    auto isLandOrOob = [&](int px, int py) -> bool {
      if (!world.inBounds(px, py)) return true;
      return world.at(px, py).terrain != Terrain::Water;
    };
    const bool nearShore =
      isLandOrOob(x, y - 1) || isLandOrOob(x + 1, y) || isLandOrOob(x, y + 1) || isLandOrOob(x - 1, y);

    // Wind-aligned wave travel direction (screen-space wind).
    const Vector2 dir = NormalizeV(Vector2{weather.windX, weather.windY});

    Vector2 corners[4];
    TileDiamondCorners(center, tileW, tileH, corners);

    const std::uint32_t hw = HashCoords32(x, y, base ^ 0xC0A57F1Du);
    const float windAmp = std::clamp(0.55f + 0.20f * (weather.windSpeed - 1.0f), 0.25f, 1.10f);

    float alphaScale = zT * windAmp;
    if (nearShore) alphaScale *= 0.65f;

    // Primary set: longer waves at low zoom (fewer lines), denser at high zoom.
    const bool highDetail = (tileScreenW >= 38.0f);
    const float waveLen0 = tileW * (highDetail ? 0.58f : 0.80f);
    const float speed0 = 14.0f * std::clamp(weather.windSpeed, 0.25f, 3.0f);
    const float seed0 = Frac01(hw ^ 0x9E3779B9u) * 6.2831853f;
    DrawWaveFrontsInDiamond(corners, dir, timeSec, speed0, waveLen0,
                            invZoom, alphaScale, seed0,
                            brightness, /*mul=*/1.0f);

    // Secondary angled set at high zoom for richer, more natural interference patterns.
    if (tileScreenW >= 36.0f) {
      const float rc = 0.8660254f; // cos(30deg)
      const float rs = 0.5f;       // sin(30deg)
      const Vector2 dir2 = NormalizeV(Vector2{dir.x * rc - dir.y * rs, dir.x * rs + dir.y * rc});
      const float waveLen1 = tileW * 0.92f;
      const float speed1 = speed0 * 0.72f;
      const float seed1 = Frac01(hw ^ 0xBADC0DEu) * 6.2831853f;
      DrawWaveFrontsInDiamond(corners, dir2, timeSec, speed1, waveLen1,
                              invZoom, alphaScale * 0.65f, seed1,
                              brightness, /*mul=*/0.95f);
    }

    // Small specular sparkles; animate lightly so large bodies of water feel alive.
    if (tileScreenW >= 30.0f) {
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

  // -----------------------------
  // Park details: paths + simple furniture + boundary hedge
  // -----------------------------
  // This is intentionally a pure draw-time decal pass (no textures) so it stays fully procedural,
  // deterministic, and can react to adjacency without adding new tile fields.
  if (isPark && tileScreenW >= 36.0f && t.terrain != Terrain::Water) {
    // 4-neighbor masks in the same layout as roads:
    //   1 = North (y-1), 2 = East (x+1), 4 = South (y+1), 8 = West (x-1)
    auto isParkAt = [&](int px, int py) -> bool {
      return world.inBounds(px, py) && world.at(px, py).overlay == Overlay::Park;
    };
    auto isRoadAt = [&](int px, int py) -> bool {
      return world.inBounds(px, py) && world.at(px, py).overlay == Overlay::Road;
    };

    std::uint8_t pm = 0;
    std::uint8_t rm = 0;

    if (isParkAt(x, y - 1)) pm |= 0x01u;
    if (isParkAt(x + 1, y)) pm |= 0x02u;
    if (isParkAt(x, y + 1)) pm |= 0x04u;
    if (isParkAt(x - 1, y)) pm |= 0x08u;

    if (isRoadAt(x, y - 1)) rm |= 0x01u;
    if (isRoadAt(x + 1, y)) rm |= 0x02u;
    if (isRoadAt(x, y + 1)) rm |= 0x04u;
    if (isRoadAt(x - 1, y)) rm |= 0x08u;

    const std::uint8_t connMask = static_cast<std::uint8_t>(pm | rm);
    const int conn = Popcount4(connMask);

    // A bit of deterministic style variation so large parks don't look stamped.
    const std::uint32_t hp = HashCoords32(x, y, base ^ 0xA11CE5E5u);
    const float style = Frac01(hp ^ 0x9E3779B9u);

    // Precompute diamond edges (edge index maps to direction bits as described above).
    Vector2 c[4];
    TileDiamondCorners(center, tileW, tileH, c);
    const Vector2 edgeA[4] = {c[0], c[1], c[2], c[3]};
    const Vector2 edgeB[4] = {c[1], c[2], c[3], c[0]};

    Vector2 edgeMid[4];
    Vector2 end[4];
    for (int i = 0; i < 4; ++i) {
      edgeMid[i] = LerpV(edgeA[i], edgeB[i], 0.5f);
      // Pull endpoints a bit inward so paint/fence doesn't fight tile edges.
      end[i] = LerpV(edgeMid[i], center, 0.16f);
    }

    // --- Park paths ---
    // Paths exist when there's an adjacency to another park tile, or a road edge (entrance).
    // This yields an organic-looking path network for multi-tile parks without any new simulation.
    if (conn > 0) {
      const unsigned char aPath = ClampU8(static_cast<int>(105.0f + 95.0f * (0.55f + 0.45f * style)));
      const Color pathFill = ShadeDetail(Color{200, 182, 140, 255}, brightness, 1.02f, aPath);
      const Color pathEdge = ShadeDetail(Color{70, 60, 42, 255}, brightness, 0.92f,
                                         ClampU8(static_cast<int>(static_cast<float>(aPath) * 0.55f)));

      const float wPath = tileH * (0.070f + 0.015f * style);
      auto drawPathSeg = [&](Vector2 a, Vector2 b) {
        DrawLineEx(a, b, wPath * 1.35f, pathEdge);
        DrawLineEx(a, b, wPath, pathFill);
      };

      // Slight node jitter to avoid an overly-perfect grid look (kept small so seams remain clean).
      Vector2 node = center;
      if (tileScreenW >= 44.0f) {
        node.x += (Frac01(hp ^ 0x13579BDFu) - 0.5f) * tileW * 0.035f;
        node.y += (Frac01(hp ^ 0x2468ACE0u) - 0.5f) * tileH * 0.035f;
      }

      const bool straightNS = (conn == 2) && ((connMask & 0x01u) != 0u) && ((connMask & 0x04u) != 0u);
      const bool straightEW = (conn == 2) && ((connMask & 0x02u) != 0u) && ((connMask & 0x08u) != 0u);

      if (straightNS) {
        drawPathSeg(end[0], end[2]);
      } else if (straightEW) {
        drawPathSeg(end[3], end[1]);
      } else {
        for (int e = 0; e < 4; ++e) {
          const std::uint8_t bit = static_cast<std::uint8_t>(1u << e);
          if ((connMask & bit) == 0u) continue;
          drawPathSeg(node, end[e]);
        }
      }

      // Plaza at intersections.
      if (conn >= 3 && tileScreenW >= 42.0f) {
        const float r0 = tileH * (0.085f + 0.015f * style);
        DrawCircleV(node, r0 * 1.15f, pathEdge);
        DrawCircleV(node, r0, pathFill);
      }

      // Road entrances: a small brighter strip that reads like a paved ramp.
      if (rm != 0 && tileScreenW >= 44.0f) {
        const Color ramp = ShadeDetail(Color{220, 205, 165, 255}, brightness, 1.06f,
                                       ClampU8(static_cast<int>(static_cast<float>(aPath) * 0.85f)));
        for (int e = 0; e < 4; ++e) {
          const std::uint8_t bit = static_cast<std::uint8_t>(1u << e);
          if ((rm & bit) == 0u) continue;
          Vector2 a = LerpV(edgeA[e], edgeB[e], 0.38f);
          Vector2 b = LerpV(edgeA[e], edgeB[e], 0.62f);
          a = LerpV(a, center, 0.12f);
          b = LerpV(b, center, 0.12f);
          DrawLineEx(a, b, wPath * 0.85f, ramp);
        }
      }

      // Benches: tiny rotated rectangles adjacent to the dominant path direction.
      if (tileScreenW >= 62.0f && ((hp & 0x7u) == 0u)) {
        Vector2 dir{1.0f, 0.0f};
        if (straightNS) {
          dir = Vector2{(end[2].x - end[0].x), (end[2].y - end[0].y)};
        } else if (straightEW) {
          dir = Vector2{(end[1].x - end[3].x), (end[1].y - end[3].y)};
        } else {
          for (int e = 0; e < 4; ++e) {
            const std::uint8_t bit = static_cast<std::uint8_t>(1u << e);
            if ((connMask & bit) != 0u) {
              dir = Vector2{end[e].x - node.x, end[e].y - node.y};
              break;
            }
          }
        }

        float dl2 = dir.x * dir.x + dir.y * dir.y;
        if (dl2 < 1.0e-6f) {
          dir = Vector2{1.0f, 0.0f};
          dl2 = 1.0f;
        }
        const float inv = 1.0f / std::sqrt(dl2);
        dir.x *= inv;
        dir.y *= inv;

        Vector2 perp{-dir.y, dir.x};
        const float side = ((hp >> 4u) & 1u) ? 1.0f : -1.0f;
        const Vector2 bc{node.x + perp.x * tileH * 0.15f * side, node.y + perp.y * tileH * 0.15f * side};

        const float bw = tileH * 0.16f;
        const float bh = tileH * 0.045f;

        Rectangle r{bc.x - bw * 0.5f, bc.y - bh * 0.5f, bw, bh};
        const Vector2 origin{bw * 0.5f, bh * 0.5f};
        const float ang = std::atan2(dir.y, dir.x) * 57.2957795f;

        const Color bench = ShadeDetail(Color{80, 60, 40, 255}, brightness, 0.95f, 200);
        DrawRectanglePro(r, origin, ang, bench);

        const Color benchHi = ShadeDetail(Color{140, 110, 80, 255}, brightness, 1.05f, 125);
        const Vector2 p0{bc.x - dir.x * bw * 0.45f - perp.x * bh * 0.20f, bc.y - dir.y * bw * 0.45f - perp.y * bh * 0.20f};
        const Vector2 p1{bc.x + dir.x * bw * 0.45f - perp.x * bh * 0.20f, bc.y + dir.y * bw * 0.45f - perp.y * bh * 0.20f};
        DrawLineEx(p0, p1, bh * 0.35f, benchHi);
      }
    }

    // --- Boundary hedge / fence ---
    // Draw a subtle hedge along edges that don't connect to adjacent park tiles.
    // We skip edges that touch roads to keep entrances open.
    if (tileScreenW >= 44.0f) {
      const float fThick = std::clamp(1.25f * invZoom, 0.65f * invZoom, 2.1f * invZoom);
      const Color fence = ShadeDetail(Color{25, 75, 40, 255}, brightness, 0.92f, 120);
      const Color fenceHi = ShadeDetail(Color{55, 120, 70, 255}, brightness, 1.05f, 60);

      for (int e = 0; e < 4; ++e) {
        const std::uint8_t bit = static_cast<std::uint8_t>(1u << e);
        if ((pm & bit) != 0u) continue; // interior edge
        if ((rm & bit) != 0u) continue; // keep open toward roads

        Vector2 a = LerpV(edgeA[e], center, 0.06f);
        Vector2 b = LerpV(edgeB[e], center, 0.06f);

        DrawLineEx(a, b, fThick, fence);
        DrawLineEx(LerpV(a, center, 0.08f), LerpV(b, center, 0.08f), fThick * 0.65f, fenceHi);

        // Small ticks/posts when extremely zoomed in.
        if (tileScreenW >= 70.0f) {
          const int ticks = 4;
          Vector2 d{b.x - a.x, b.y - a.y};
          const float dl2 = d.x * d.x + d.y * d.y;
          if (dl2 > 1.0e-6f) {
            const float inv = 1.0f / std::sqrt(dl2);
            d.x *= inv;
            d.y *= inv;
          }
          Vector2 n{-d.y, d.x};
          for (int i = 1; i <= ticks; ++i) {
            const float tt = static_cast<float>(i) / static_cast<float>(ticks + 1);
            Vector2 p = LerpV(a, b, tt);
            Vector2 p0{p.x - n.x * tileH * 0.010f, p.y - n.y * tileH * 0.010f};
            Vector2 p1{p.x + n.x * tileH * 0.010f, p.y + n.y * tileH * 0.010f};
            DrawLineEx(p0, p1, fThick * 0.70f, fence);
          }
        }
      }
    }

    // --- Flower beds ---
    // A rare accent placed only in larger parks away from roads.
    if (tileScreenW >= 64.0f && pm != 0 && rm == 0) {
      const std::uint32_t hf = HashCoords32(x, y, base ^ 0xF10A3F5u);
      if ((hf & 0x1Fu) == 0u) {
        const Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0xF10A3F5u, 90, center, tileW, tileH, 0.62f);
        const float rr = tileH * 0.045f;
        const Color soil = ShadeDetail(Color{60, 45, 30, 255}, brightness, 0.90f, 150);
        DrawCircleV(p, rr, soil);

        const int petals = 6;
        for (int i = 0; i < petals; ++i) {
          const std::uint32_t hi = HashCoords32(i, static_cast<int>(hf), base ^ 0x9E3779B9u);
          Color fl = (i & 1) ? Color{250, 190, 210, 255} : Color{250, 230, 110, 255};
          fl = ShadeDetail(fl, brightness, 1.10f, 160);

          Vector2 q{p.x + (Frac01(hi) - 0.5f) * rr * 1.3f,
                    p.y + (Frac01(hi ^ 0xBADC0DEu) - 0.5f) * rr * 1.0f};
          const float pr = rr * (0.25f + 0.18f * Frac01(hi ^ 0x13579BDFu));
          DrawCircleV(q, pr, fl);
        }
      }
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
  
  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 18.0f) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);

  // -----------------------------
  // Rain ripples on water (rain)
  // -----------------------------
  if (w.mode == Renderer::WeatherSettings::Mode::Rain && w.intensity > 0.02f && t.terrain == Terrain::Water) {
    const float inten = std::clamp(w.intensity, 0.0f, 1.0f);

    // Only spawn ripples on a subset of water tiles to keep the effect light-weight.
    const std::uint32_t base = HashCoords32(x, y, seed32 ^ 0x71A11EE5u);
    const float density = 0.10f + 0.25f * inten;
    if (Frac01(base) <= density) {
      auto drawEllipseRing = [&](Vector2 c, float rx, float ry, float thick, Color col) {
        const int seg = 14;
        Vector2 prev{c.x + rx, c.y};
        for (int si = 1; si <= seg; ++si) {
          const float a = (static_cast<float>(si) / static_cast<float>(seg)) * 2.0f * kPiF;
          Vector2 cur{c.x + std::cos(a) * rx, c.y + std::sin(a) * ry};
          DrawLineEx(prev, cur, thick, col);
          prev = cur;
        }
      };

      const int ripples = (tileScreenW >= 60.0f) ? 2 : 1;
      for (int i = 0; i < ripples; ++i) {
        const std::uint32_t hi = HashCoords32(x + i * 37, y - i * 29, base ^ 0xA3613F13u);
        const float period = 0.85f + 0.55f * Frac01(hi ^ 0x9E3779B9u);
        const float phase = Frac01(hi ^ 0x51A5EEDu) * period;
        const float tt = std::fmod(timeSec + phase, period) / period; // 0..1 expanding

        // Ripple center within the tile.
        Vector2 p = DeterministicDiamondPoint(x, y, base ^ 0x13579BDFu, 240 + i * 11, center, tileW, tileH, 0.76f);

        // Ellipse radii (squashed in Y to match the isometric ground plane).
        const float r = tileH * (0.06f + 0.22f * tt);
        const float rx = r;
        const float ry = r * 0.58f;

        const float fade = (1.0f - tt);
        const float nightBoost = 0.70f + 0.55f * dn.night;
        const unsigned char a = ClampU8(static_cast<int>(55.0f * inten * fade * nightBoost));

        if (a == 0) continue;

        const float thick = std::clamp((0.95f + 0.35f * inten) * invZoom, 0.55f * invZoom, 1.9f * invZoom);
        const Color col = ShadeDetail(Color{220, 240, 255, 255}, brightness, 1.05f, a);

        drawEllipseRing(p, rx, ry, thick, col);

        // A second faint ring adds richness at high zoom.
        if (tileScreenW >= 70.0f) {
          const unsigned char a2 = ClampU8(static_cast<int>(static_cast<float>(a) * 0.55f));
          const Color col2 = ShadeDetail(Color{220, 240, 255, 255}, brightness, 1.04f, a2);
          drawEllipseRing(p, rx * 0.72f, ry * 0.72f, thick * 0.85f, col2);
        }
      }
    }
  }

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
                               bool suppressZoneWindows,
                               std::uint32_t seed32)
{
  if (night <= 0.001f) return;

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 22.0f) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);

  // Additive blend reads much closer to "light" than standard alpha compositing.
  BeginBlendMode(BLEND_ADDITIVE);

  // Soft ground pool (a blurred diamond) to suggest light spilling onto the ground plane.
  // Kept cheap (a few diamonds) and zoom-gated to cap overdraw on dense downtown areas.
  auto drawLightPool = [&](Vector2 c, float scale, Color col) {
    if (col.a == 0) return;
    if (scale <= 0.05f) return;

    const unsigned char a0 = col.a;
    Color c0 = col;
    c0.a = ClampU8(static_cast<int>(static_cast<float>(a0) * 0.55f));
    Color c1 = col;
    c1.a = ClampU8(static_cast<int>(static_cast<float>(a0) * 0.28f));
    Color c2 = col;
    c2.a = ClampU8(static_cast<int>(static_cast<float>(a0) * 0.14f));

    DrawDiamond(c, tileW * scale, tileH * scale, c0);
    DrawDiamond(c, tileW * scale * 1.55f, tileH * scale * 1.55f, c1);
    if (tileScreenW >= 36.0f) {
      DrawDiamond(c, tileW * scale * 2.25f, tileH * scale * 2.25f, c2);
    }
  };

  // Faint water reflections for nearby lights.
  auto drawWaterReflection = [&](int wx, int wy, Color outer, Color inner, float k) {
    if (!world.inBounds(wx, wy)) return;
    const Tile& wt = world.at(wx, wy);
    if (wt.terrain != Terrain::Water) return;

    Vector2 wc = TileToWorldCenter(wx, wy, tileW, tileH);
    wc.y -= TileElevationPx(wt, elev);

    const std::uint32_t hw = HashCoords32(wx, wy, seed32 ^ 0xB16B00B5u);
    const float wobble =
      std::sin(timeSec * (1.4f + 0.35f * Frac01(hw)) + Frac01(hw ^ 0x9E3779B9u) * 6.2831853f);
    wc.x += wobble * tileW * 0.03f;

    // Push toward cooler tones for water.
    auto cool = [&](Color c, float t) -> Color {
      const float it = 1.0f - t;
      c.r = ClampU8(static_cast<int>(it * static_cast<float>(c.r) + t * 120.0f));
      c.g = ClampU8(static_cast<int>(it * static_cast<float>(c.g) + t * 200.0f));
      c.b = ClampU8(static_cast<int>(it * static_cast<float>(c.b) + t * 255.0f));
      return c;
    };

    Color o = cool(outer, 0.60f);
    Color i = cool(inner, 0.45f);

    o.a = ClampU8(static_cast<int>(static_cast<float>(o.a) * k));
    i.a = ClampU8(static_cast<int>(static_cast<float>(i.a) * k));
    if (o.a == 0 && i.a == 0) return;

    const float r = (3.4f + 2.0f * k) * invZoom;

    Vector2 p0 = wc;
    p0.y += tileH * 0.04f;
    Vector2 p1 = p0;
    p1.y += tileH * (0.18f + 0.06f * k);

    DrawLineEx(p0, p1, r * 0.35f, o);
    DrawGlow(Vector2{wc.x, wc.y + tileH * 0.10f}, r * 0.95f, o, i);
  };

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
        // Additive blending is more energetic, so pull intensity down slightly.
        const int a = static_cast<int>(aBase * night * flicker * 0.92f);

        const float r = (isIntersection ? 7.5f : 6.0f) * invZoom;

        Color outer{255, 205, 135, ClampU8(a)};
        Color inner{255, 245, 220, ClampU8(a + 55)};

        // Bridges read better with a cooler light.
        if (t.terrain == Terrain::Water) {
          outer = Color{205, 235, 255, ClampU8(a)};
          inner = Color{235, 250, 255, ClampU8(a + 55)};
        }

        // Light spill on the ground plane.
        {
          Vector2 pool = center;
          pool.y += tileH * 0.08f;

          Color poolC = outer;
          poolC.a = ClampU8(static_cast<int>(static_cast<float>(outer.a) * (isIntersection ? 0.34f : 0.28f)));

          const float poolScale = (isIntersection ? 1.55f : 1.35f) + (isMajor ? 0.10f : 0.0f);
          drawLightPool(pool, poolScale, poolC);
        }

        // Neighbor water reflections (shoreline / canals).
        {
          const float w = std::clamp(wetness, 0.0f, 1.0f);
          const float k = 0.22f + 0.18f * w;
          drawWaterReflection(x, y - 1, outer, inner, k);
          drawWaterReflection(x + 1, y, outer, inner, k);
          drawWaterReflection(x, y + 1, outer, inner, k);
          drawWaterReflection(x - 1, y, outer, inner, k);
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

        if (suppressZoneWindows) continue;

        const int cap = std::max(1, CapacityForTile(t));
        const float occ = std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f);

        float litChance = 0.10f + 0.70f * occ;
        if (t.overlay == Overlay::Commercial) litChance = 0.30f + 0.55f * occ;
        if (t.overlay == Overlay::Industrial) litChance = 0.06f + 0.35f * occ;

        const std::uint32_t hb = HashCoords32(x, y, seed32 ^ 0x5A17B00Bu);

        // Per-tile count scales with zone level; commercial tends to be brighter.
        const int baseCount = 1 + std::clamp(static_cast<int>(t.level), 1, 3) / 2;
        const int count = baseCount + ((t.overlay == Overlay::Commercial) ? 1 : 0);

        // Building-level ambient spill (one per tile) so the city reads as actually illuminated.
        if (tileScreenW >= 32.0f) {
          const float w = std::clamp(wetness, 0.0f, 1.0f);
          const float wetBoost = 0.80f + 0.20f * w;

          const float base = night * (0.20f + 0.80f * occ) * wetBoost;
          const unsigned char aPool = ClampU8(static_cast<int>(55.0f * base));
          if (aPool != 0) {
            Color poolC{255, 190, 120, aPool};
            float poolScale = 1.20f;

            if (t.overlay == Overlay::Commercial) {
              poolC = Color{150, 225, 255, aPool};
              poolScale = 1.30f;
            } else if (t.overlay == Overlay::Industrial) {
              poolC = Color{255, 170, 90, aPool};
              poolScale = 1.25f;
            }

            Vector2 pool = center;
            pool.y += tileH * 0.10f;
            drawLightPool(pool, poolScale, poolC);
          }
        }

        for (int i = 0; i < count; ++i) {
          const std::uint32_t hi = HashCoords32(x + i * 97, y - i * 61, hb ^ 0x9E3779B9u);
          if (Frac01(hi) > litChance) continue;

          Vector2 p = DeterministicDiamondPoint(x, y, hb ^ 0x13579BDFu, 40 + i, center, tileW, tileH, 0.55f);
          // Lift above the tile to read like building windows.
          p.y -= tileH * (0.18f + 0.06f * Frac01(hi));

          const float flicker = 0.80f +
                                0.20f * std::sin(timeSec * (1.2f + 0.25f * static_cast<float>((hi >> 6) & 7u)) +
                                                 Frac01(hi) * 6.2831853f);

          const float aBase = 70.0f + 120.0f * occ + 18.0f * static_cast<float>(t.level);
          // Similar to roads: keep emissives under control in additive blend mode.
          const int a = static_cast<int>(aBase * night * flicker * 0.78f);

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

        // Occasional commercial neon strip signs at mid zoom. This adds visual life when we are
        // not using the high-zoom building sprite system (which already has emissive textures).
        if (t.overlay == Overlay::Commercial && tileScreenW >= 32.0f && tileScreenW < 54.0f) {
          const std::uint32_t hs = HashCoords32(x, y, seed32 ^ 0x4E30A11Cu);
          const float chance = 0.08f + 0.22f * occ;
          if (Frac01(hs) < chance) {
            Vector2 corners[4];
            TileDiamondCorners(center, tileW, tileH, corners);

            const Vector2 edgeA[4] = {corners[0], corners[1], corners[2], corners[3]};
            const Vector2 edgeB[4] = {corners[1], corners[2], corners[3], corners[0]};

            const int e = static_cast<int>((hs >> 3u) & 3u);

            const float u0 = 0.22f + 0.10f * Frac01(hs ^ 0xA11CE5EDu);
            const float u1 = 0.78f - 0.10f * Frac01(hs ^ 0xBADC0DEu);

            Vector2 a0 = LerpV(edgeA[e], edgeB[e], u0);
            Vector2 b0 = LerpV(edgeA[e], edgeB[e], u1);

            const float height = tileH * (0.26f + 0.12f * Frac01(hs ^ 0xC0FFEEu));
            a0.y -= height;
            b0.y -= height;

            auto neonColor = [&](int idx) -> Color {
              switch (idx) {
              case 0: return Color{80, 255, 255, 255};   // cyan
              case 1: return Color{255, 80, 240, 255};   // magenta
              case 2: return Color{255, 200, 60, 255};   // amber
              case 3: return Color{120, 255, 120, 255};  // green
              default: return Color{190, 90, 255, 255};  // purple
              }
            };

            const int ci = static_cast<int>((hs >> 6u) % 5u);
            Color neon = neonColor(ci);

            const float flicker =
              0.78f + 0.22f * std::sin(timeSec * (2.8f + 0.8f * Frac01(hs)) + Frac01(hs ^ 0x13579BDFu) * 6.2831853f);

            const float speed = 4.0f + 3.0f * Frac01(hs ^ 0x2468ACE0u);
            const int tick = static_cast<int>(std::floor(timeSec * speed));
            const std::uint32_t hBlink = HashCoords32(x + tick * 13, y - tick * 7, hs ^ 0xDEADC0DEu);
            const float blink = (Frac01(hBlink) < 0.03f) ? 0.0f : 1.0f;

            const float inten = night * (0.35f + 0.65f * occ) * flicker * blink;
            const unsigned char aN = ClampU8(static_cast<int>(210.0f * inten));
            if (aN != 0) {
              const float thick = (1.10f + 0.55f * Frac01(hs ^ 0x9E3779B9u)) * invZoom;

              Color cWide = neon;
              cWide.a = ClampU8(static_cast<int>(static_cast<float>(aN) * 0.22f));
              Color cMid = neon;
              cMid.a = ClampU8(static_cast<int>(static_cast<float>(aN) * 0.55f));
              Color cCore = neon;
              cCore.a = aN;
              Color cHot{255, 255, 255, ClampU8(static_cast<int>(static_cast<float>(aN) * 0.85f))};

              DrawLineEx(a0, b0, thick * 4.2f, cWide);
              DrawLineEx(a0, b0, thick * 2.3f, cMid);
              DrawLineEx(a0, b0, thick * 1.05f, cCore);
              DrawLineEx(a0, b0, thick * 0.55f, cHot);

              DrawCircleV(a0, thick * 0.55f, cHot);
              DrawCircleV(b0, thick * 0.55f, cHot);

              Vector2 pool = center;
              pool.y += tileH * 0.10f;
              Color poolC = neon;
              poolC.a = ClampU8(static_cast<int>(static_cast<float>(aN) * 0.18f));
              drawLightPool(pool, 1.18f, poolC);
            }
          }
        }
      }
    }
  }

  EndBlendMode();
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
      return LerpColor(base, bridge, k);
    }

    const Color road = (lvl == 1) ? Color{28, 28, 30, 255} : (lvl == 2) ? Color{24, 24, 28, 255} : Color{20, 20, 25, 255};
    const float k = (lvl == 1) ? 0.85f : (lvl == 2) ? 0.88f : 0.90f;
    return LerpColor(base, road, k);
  }
  case Overlay::Park: return LerpColor(base, Color{70, 200, 95, 255}, 0.70f);
  case Overlay::Residential: return LerpColor(base, Color{80, 160, 235, 255}, 0.80f);
  case Overlay::Commercial: return LerpColor(base, Color{240, 170, 60, 255}, 0.80f);
  case Overlay::Industrial: return LerpColor(base, Color{200, 90, 220, 255}, 0.80f);
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


// Road indicators + procedural markings: lane center-lines, crosswalk hints, and subtle wear.
// Kept purely aesthetic and gated by zoom so it doesn't clutter utility overlays.
static void DrawRoadIndicators(const World& world, int x, int y, const Tile& t,
                               float tileW, float tileH, float zoom, const Vector2& tileCenter, float tileBrightness,
                               const DayNightState& dn, const WeatherState& wx, std::uint32_t seed32, float timeSec)
{
  (void)timeSec; // reserved for subtle animation (e.g., wet shimmer) if desired

  const float tileScreenW = tileW * zoom;
  if (tileScreenW < 28.0f) return;

  const float invZoom = 1.0f / std::max(0.001f, zoom);
  const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);

  // -----------------------------
  // Procedural road markings (aesthetic)
  // -----------------------------
  if (tileScreenW >= 34.0f) {
    // Shared road connectivity mask.
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

    std::uint8_t mask = roadMaskAt(x, y);
    const int conn = Popcount4(mask);

    if (conn > 0) {
      const bool n = (mask & 0x01u) != 0u;
      const bool e = (mask & 0x02u) != 0u;
      const bool s = (mask & 0x04u) != 0u;
      const bool w = (mask & 0x08u) != 0u;

      Vector2 c[4];
      TileDiamondCorners(tileCenter, tileW, tileH, c);
      const Vector2 edgeA[4] = {c[0], c[1], c[2], c[3]};
      const Vector2 edgeB[4] = {c[1], c[2], c[3], c[0]};

      Vector2 edgeMid[4];
      Vector2 end[4];
      for (int i = 0; i < 4; ++i) {
        edgeMid[i] = LerpV(edgeA[i], edgeB[i], 0.5f);
        end[i] = LerpV(edgeMid[i], tileCenter, 0.30f);
      }

      // Reflective markings: boost when wet + at night so they remain visible after global grading.
      const float night = std::clamp(dn.nightLights, 0.0f, 1.0f);
      const float wet = std::clamp(wx.wetness, 0.0f, 1.0f);

      const float refl = std::clamp(0.20f + 0.80f * wet, 0.0f, 1.0f) * (0.55f + 0.45f * night);
      const float mul = 1.05f + 0.55f * refl;
      const unsigned char aLine = ClampU8(static_cast<int>(50.0f + 135.0f * refl));

      Color white = ShadeDetail(Color{250, 250, 245, 255}, tileBrightness, mul, aLine);
      Color yellow = ShadeDetail(Color{250, 220, 120, 255}, tileBrightness, mul, ClampU8(static_cast<int>(aLine * 0.85f)));

      if (t.terrain == Terrain::Water) {
        // Bridges read better with slightly cooler paint.
        white = ShadeDetail(Color{235, 250, 255, 255}, tileBrightness, mul, aLine);
        yellow = ShadeDetail(Color{225, 240, 255, 255}, tileBrightness, mul, ClampU8(static_cast<int>(aLine * 0.70f)));
      }

      const float thick = std::clamp(1.30f * invZoom, 0.70f * invZoom, 2.7f * invZoom);
      const float thickDash = std::clamp(1.05f * invZoom, 0.60f * invZoom, 2.2f * invZoom);

      auto drawSolid = [&](Vector2 a, Vector2 b, Color col) {
        DrawLineEx(a, b, thick, col);
      };

      auto drawDashed = [&](Vector2 a, Vector2 b, Color col, int dashes, float phase01) {
        dashes = std::max(1, dashes);
        const float seg = 1.0f / (static_cast<float>(dashes) * 2.0f);
        for (int i = 0; i < dashes; ++i) {
          float t0 = phase01 + static_cast<float>(i) * 2.0f * seg;
          float t1 = t0 + seg;
          if (t1 <= 0.0f || t0 >= 1.0f) continue;
          t0 = std::clamp(t0, 0.0f, 1.0f);
          t1 = std::clamp(t1, 0.0f, 1.0f);
          DrawLineEx(LerpV(a, b, t0), LerpV(a, b, t1), thickDash, col);
        }
      };

      auto drawCenterLine = [&](Vector2 a, Vector2 b) {
        // Per-tile dash phase.
        const std::uint32_t hh = HashCoords32(x, y, seed32 ^ 0xD00DCAFEu);
        const float phase01 = Frac01(hh ^ 0x9E3779B9u) * 0.12f;

        // Perpendicular for double-line offset.
        Vector2 d{b.x - a.x, b.y - a.y};
        const float dl2 = d.x * d.x + d.y * d.y;
        if (dl2 > 1.0e-6f) {
          const float inv = 1.0f / std::sqrt(dl2);
          d.x *= inv;
          d.y *= inv;
        } else {
          d = Vector2{1.0f, 0.0f};
        }
        Vector2 perp{-d.y, d.x};

        if (lvl <= 1) {
          const int dashes = (tileScreenW >= 44.0f) ? 4 : 3;
          drawDashed(a, b, white, dashes, phase01);
        } else if (lvl == 2) {
          drawSolid(a, b, yellow);
        } else {
          // Double center line.
          const float off = 1.65f * invZoom;
          drawSolid(Vector2{a.x + perp.x * off, a.y + perp.y * off},
                    Vector2{b.x + perp.x * off, b.y + perp.y * off}, yellow);
          drawSolid(Vector2{a.x - perp.x * off, a.y - perp.y * off},
                    Vector2{b.x - perp.x * off, b.y - perp.y * off}, yellow);
        }
      };

      // Decide marking geometry.
      if (conn == 2) {
        // Straight vs corner.
        if (n && s) {
          drawCenterLine(end[0], end[2]);
        } else if (e && w) {
          drawCenterLine(end[3], end[1]);
        } else {
          // Corner turn: build a gentle 2-segment polyline through a biased midpoint.
          int aEdge = n ? 0 : (e ? 1 : (s ? 2 : 3));
          int bEdge = aEdge;
          if (aEdge == 0) bEdge = e ? 1 : 3;
          else if (aEdge == 1) bEdge = s ? 2 : 0;
          else if (aEdge == 2) bEdge = w ? 3 : 1;
          else if (aEdge == 3) bEdge = n ? 0 : 2;

          Vector2 dirA{tileCenter.x - edgeMid[aEdge].x, tileCenter.y - edgeMid[aEdge].y};
          Vector2 dirB{tileCenter.x - edgeMid[bEdge].x, tileCenter.y - edgeMid[bEdge].y};

          const float la = std::sqrt(dirA.x * dirA.x + dirA.y * dirA.y);
          const float lb = std::sqrt(dirB.x * dirB.x + dirB.y * dirB.y);
          if (la > 0.001f) { dirA.x /= la; dirA.y /= la; }
          if (lb > 0.001f) { dirB.x /= lb; dirB.y /= lb; }

          Vector2 mid = tileCenter;
          mid.x += (dirA.x + dirB.x) * tileH * 0.10f;
          mid.y += (dirA.y + dirB.y) * tileH * 0.10f;

          drawCenterLine(end[aEdge], mid);
          drawCenterLine(mid, end[bEdge]);
        }
      } else if (conn >= 3) {
        // Intersections: stop line + crosswalk hints per connected edge (only at high zoom).
        if (tileScreenW >= 42.0f) {
          const int stripes = (tileScreenW >= 58.0f) ? 5 : 4;
          for (int edge = 0; edge < 4; ++edge) {
            const std::uint8_t bit = static_cast<std::uint8_t>(1u << edge);
            if ((mask & bit) == 0u) continue;

            // Stop line (parallel to edge).
            Vector2 a0 = LerpV(edgeA[edge], edgeB[edge], 0.28f);
            Vector2 b0 = LerpV(edgeA[edge], edgeB[edge], 0.72f);
            a0 = LerpV(a0, tileCenter, 0.16f);
            b0 = LerpV(b0, tileCenter, 0.16f);
            DrawLineEx(a0, b0, thickDash * 1.15f, white);

            // Crosswalk stripes (perpendicular to edge, pointing inward).
            Vector2 in{tileCenter.x - edgeMid[edge].x, tileCenter.y - edgeMid[edge].y};
            const float il2 = in.x * in.x + in.y * in.y;
            if (il2 > 1.0e-6f) {
              const float inv = 1.0f / std::sqrt(il2);
              in.x *= inv;
              in.y *= inv;
            } else {
              in = Vector2{0.0f, 1.0f};
            }

            for (int sIdx = 0; sIdx < stripes; ++sIdx) {
              const float tStripe = 0.22f + (static_cast<float>(sIdx) / static_cast<float>(std::max(1, stripes - 1))) * 0.56f;
              Vector2 base = LerpV(edgeA[edge], edgeB[edge], tStripe);
              Vector2 p0 = LerpV(base, tileCenter, 0.08f);
              Vector2 p1 = Vector2{p0.x + in.x * tileH * 0.09f, p0.y + in.y * tileH * 0.09f};
              DrawLineEx(p0, p1, thickDash, white);
            }
          }
        }
      } else if (conn == 1) {
        // Dead end: short center line plus an end bar.
        int edge = n ? 0 : (e ? 1 : (s ? 2 : 3));
        drawCenterLine(end[edge], tileCenter);

        if (tileScreenW >= 42.0f) {
          // End bar at the unconnected side (opposite edge), inset slightly.
          const int opp = (edge + 2) & 3;
          Vector2 a0 = LerpV(edgeA[opp], edgeB[opp], 0.28f);
          Vector2 b0 = LerpV(edgeA[opp], edgeB[opp], 0.72f);
          a0 = LerpV(a0, tileCenter, 0.22f);
          b0 = LerpV(b0, tileCenter, 0.22f);
          DrawLineEx(a0, b0, thickDash * 1.10f, white);
        }
      }

      // Subtle wear/cracks: only when very zoomed in so it doesn't look like noise.
      if (tileScreenW >= 48.0f) {
        const std::uint32_t hb = HashCoords32(x, y, seed32 ^ 0xA57A11u);
        const int cracks = 1 + static_cast<int>((hb >> 29) & 1u);

        // Fewer cracks on higher-class roads.
        const float wearMul = (lvl >= 3) ? 0.55f : (lvl == 2) ? 0.75f : 1.0f;

        const unsigned char aCrack = ClampU8(static_cast<int>(22.0f * wearMul + 38.0f * wearMul * (1.0f - night)));
        const Color crackC = ShadeDetail(Color{20, 20, 22, 255}, tileBrightness, 0.95f, aCrack);

        for (int i = 0; i < cracks; ++i) {
          Vector2 p0 = DeterministicDiamondPoint(x, y, hb ^ 0x51A5EEDu, 120 + i * 11, tileCenter, tileW, tileH, 0.80f);
          Vector2 p1 = DeterministicDiamondPoint(x, y, hb ^ 0xBADC0DEu, 160 + i * 17, tileCenter, tileW, tileH, 0.80f);

          // Bias crack to follow main road direction when straight.
          if (conn == 2 && (n && s)) {
            p0.x = tileCenter.x + (p0.x - tileCenter.x) * 0.25f;
            p1.x = tileCenter.x + (p1.x - tileCenter.x) * 0.25f;
          } else if (conn == 2 && (e && w)) {
            p0.y = tileCenter.y + (p0.y - tileCenter.y) * 0.25f;
            p1.y = tileCenter.y + (p1.y - tileCenter.y) * 0.25f;
          }

          DrawLineEx(p0, p1, 0.95f * invZoom, crackC);

          // Tiny branch.
          if (((hb >> (i * 7)) & 3u) == 0u) {
            Vector2 mid = LerpV(p0, p1, 0.55f);
            Vector2 q = DeterministicDiamondPoint(x, y, hb ^ 0x13579BDFu, 200 + i * 5, tileCenter, tileW, tileH, 0.55f);
            DrawLineEx(mid, q, 0.75f * invZoom, crackC);
          }
        }
      }
    }
  }

  // -----------------------------
  // Upgrade pips (road class 2..3)
  // -----------------------------
  if (lvl <= 1) return;

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
  // Geometry-shader programs need to be released before the GL context is torn down.
  m_gpuRibbon.shutdown();

  // Shader-based volumetric clouds also rely on GL resources.
  unloadVolumetricCloudResources();

  for (auto& tv : m_terrainTex) {
    for (auto& t : tv) {
      if (t.id != 0) UnloadTexture(t);
      t = Texture2D{};
    }
  }

  for (auto& mv : m_terrainTransWaterSand) {
    for (auto& t : mv) {
      if (t.id != 0) UnloadTexture(t);
      t = Texture2D{};
    }
  }
  for (auto& mv : m_terrainTransSandGrass) {
    for (auto& t : mv) {
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

  unloadBuildingSprites();

  unloadPropSprites();

  m_organicMaterial.shutdown();

  unloadBaseCache();

  unloadMinimap();
}


void Renderer::unloadVolumetricCloudResources()
{
  if (m_volCloudShader.id != 0) {
    UnloadShader(m_volCloudShader);
    m_volCloudShader = Shader{};
  }

  m_volCloudShaderFailed = false;

  m_volCloudLocViewMin = -1;
  m_volCloudLocViewSize = -1;
  m_volCloudLocTime = -1;
  m_volCloudLocWindDir = -1;
  m_volCloudLocWindSpeed = -1;
  m_volCloudLocScale = -1;
  m_volCloudLocCoverage = -1;
  m_volCloudLocDensity = -1;
  m_volCloudLocSoftness = -1;
  m_volCloudLocSteps = -1;
  m_volCloudLocDay = -1;
  m_volCloudLocDusk = -1;
  m_volCloudLocOvercast = -1;
  m_volCloudLocSeed = -1;
  m_volCloudLocBottomFade = -1;
}

void Renderer::ensureVolumetricCloudShader()
{
  if (m_volCloudShader.id != 0) return;
  if (m_volCloudShaderFailed) return;

  m_volCloudShader = LoadShaderFromMemory(kVolumetricCloudVS, kVolumetricCloudFS);
  if (m_volCloudShader.id == 0) {
    m_volCloudShaderFailed = true;
    return;
  }

  m_volCloudLocViewMin = GetShaderLocation(m_volCloudShader, "u_viewMin");
  m_volCloudLocViewSize = GetShaderLocation(m_volCloudShader, "u_viewSize");
  m_volCloudLocTime = GetShaderLocation(m_volCloudShader, "u_time");
  m_volCloudLocWindDir = GetShaderLocation(m_volCloudShader, "u_windDir");
  m_volCloudLocWindSpeed = GetShaderLocation(m_volCloudShader, "u_windSpeed");
  m_volCloudLocScale = GetShaderLocation(m_volCloudShader, "u_scale");
  m_volCloudLocCoverage = GetShaderLocation(m_volCloudShader, "u_coverage");
  m_volCloudLocDensity = GetShaderLocation(m_volCloudShader, "u_density");
  m_volCloudLocSoftness = GetShaderLocation(m_volCloudShader, "u_softness");
  m_volCloudLocSteps = GetShaderLocation(m_volCloudShader, "u_steps");
  m_volCloudLocDay = GetShaderLocation(m_volCloudShader, "u_day");
  m_volCloudLocDusk = GetShaderLocation(m_volCloudShader, "u_dusk");
  m_volCloudLocOvercast = GetShaderLocation(m_volCloudShader, "u_overcast");
  m_volCloudLocSeed = GetShaderLocation(m_volCloudShader, "u_seed");
  m_volCloudLocBottomFade = GetShaderLocation(m_volCloudShader, "u_bottomFade");
}

void Renderer::drawVolumetricCloudLayer(const WorldRect& viewAABB, float tileW, float timeSec,
                                       float day, float dusk, float overcast,
                                       float windX, float windY, float windSpeed)
{
  if (!m_volClouds.enabled) return;

  // Avoid doing work when cloudiness is essentially zero.
  const float oc = std::clamp(overcast, 0.0f, 1.0f);
  if (oc <= 0.001f) return;

  ensureVolumetricCloudShader();
  if (m_volCloudShader.id == 0) return;

  const float pad = tileW * 2.0f;
  const Rectangle dst{
      viewAABB.minX - pad,
      viewAABB.minY - pad,
      (viewAABB.maxX - viewAABB.minX) + pad * 2.0f,
      (viewAABB.maxY - viewAABB.minY) + pad * 2.0f};

  const Vector2 viewMin{dst.x, dst.y};
  const Vector2 viewSize{dst.width, dst.height};

  // Shader parameters derived from world scale so the effect stays coherent across tile sizes.
  const float scaleMul = std::clamp(m_volClouds.scale, 0.25f, 8.0f);
  const float baseFreq = 1.0f / std::max(1.0f, tileW * 26.0f);
  const float freq = baseFreq / scaleMul;

  const float coverage = std::clamp(m_volClouds.coverage * (0.55f + 0.45f * oc), 0.0f, 1.0f);
  const float density = std::clamp(m_volClouds.density * (0.65f + 0.70f * oc), 0.05f, 3.0f);
  const float softness = std::clamp(m_volClouds.softness, 0.0f, 1.0f);
  const float bottomFade = std::clamp(m_volClouds.bottomFade, 0.0f, 1.0f);

  const int steps = std::clamp(m_volClouds.steps, 8, 64);

  const Vector2 windDir{windX, windY};
  const float speedMul = std::max(0.0f, m_volClouds.speed);
  const float worldSpeed = tileW * 0.80f * speedMul * std::max(0.0f, windSpeed);
  const float windNoiseSpeed = worldSpeed * freq;

  // Overall opacity is controlled on the CPU via tint alpha so the shader can focus on shape.
  const float op = std::clamp(m_volClouds.opacity, 0.0f, 1.0f);
  const float dnMul = 0.45f + 0.55f * std::clamp(day, 0.0f, 1.0f);
  const float alpha = std::clamp(op * (0.30f + 0.70f * oc) * dnMul, 0.0f, 1.0f);
  const unsigned char a = static_cast<unsigned char>(std::round(255.0f * alpha));
  if (a == 0) return;

  const float seed = static_cast<float>(static_cast<std::uint32_t>(m_gfxSeed32) & 0xFFFFu);

  SetShaderValue(m_volCloudShader, m_volCloudLocViewMin, &viewMin, SHADER_UNIFORM_VEC2);
  SetShaderValue(m_volCloudShader, m_volCloudLocViewSize, &viewSize, SHADER_UNIFORM_VEC2);
  SetShaderValue(m_volCloudShader, m_volCloudLocTime, &timeSec, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocWindDir, &windDir, SHADER_UNIFORM_VEC2);
  SetShaderValue(m_volCloudShader, m_volCloudLocWindSpeed, &windNoiseSpeed, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocScale, &freq, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocCoverage, &coverage, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocDensity, &density, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocSoftness, &softness, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocSteps, &steps, SHADER_UNIFORM_INT);
  SetShaderValue(m_volCloudShader, m_volCloudLocDay, &day, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocDusk, &dusk, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocOvercast, &oc, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocSeed, &seed, SHADER_UNIFORM_FLOAT);
  SetShaderValue(m_volCloudShader, m_volCloudLocBottomFade, &bottomFade, SHADER_UNIFORM_FLOAT);

  BeginShaderMode(m_volCloudShader);
  DrawRectangleRec(dst, Color{255, 255, 255, a});
  EndShaderMode();
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

void Renderer::unloadBuildingSprites()
{
  auto unloadLevel = [](std::array<std::vector<BuildingSprite>, 3>& levels) {
    for (auto& v : levels) {
      for (auto& s : v) {
        if (s.color.id != 0) UnloadTexture(s.color);
        if (s.emissive.id != 0) UnloadTexture(s.emissive);
        s = BuildingSprite{};
      }
      v.clear();
    }
  };

  unloadLevel(m_buildingResidential);
  unloadLevel(m_buildingCommercial);
  unloadLevel(m_buildingIndustrial);
}


void Renderer::unloadPropSprites()
{
  auto unloadVec = [](std::vector<PropSprite>& v) {
    for (auto& s : v) {
      if (s.color.id != 0) UnloadTexture(s.color);
      if (s.emissive.id != 0) UnloadTexture(s.emissive);
      s = PropSprite{};
    }
    v.clear();
  };

  unloadVec(m_propTreeDeciduous);
  unloadVec(m_propTreeConifer);
  unloadVec(m_propStreetLight);
  unloadVec(m_propPedestrian);
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

void Renderer::rebuildBuildingSprites()
{
  unloadBuildingSprites();

  GfxBuildingsConfig cfg{};
  cfg.tileW = m_tileW;
  cfg.tileH = m_tileH;
  cfg.includeEmissive = true;

  // Use the same palette system as other procedural sprites so buildings feel cohesive.
  const GfxPalette pal = GenerateGfxPalette(m_gfxSeed32 ^ 0xB1D1B00Du, GfxTheme::Classic);

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

  auto buildLevel = [&](GfxBuildingKind kind, int lvl, int want, std::vector<BuildingSprite>& out) {
    std::string err;
    for (int variant = 0; variant < want; ++variant) {
      GfxBuildingSprite spr{};
      if (!GenerateGfxBuildingSprite(kind, lvl, variant, m_gfxSeed32, cfg, pal, spr, err)) continue;

      BuildingSprite bs{};
      bs.pivotX = spr.pivotX;
      bs.pivotY = spr.pivotY;
      bs.color = loadTex(spr.color);
      if (!spr.emissive.rgba.empty()) bs.emissive = loadTex(spr.emissive);

      if (bs.color.id == 0) {
        if (bs.emissive.id != 0) UnloadTexture(bs.emissive);
        continue;
      }
      out.push_back(bs);
    }
  };

  constexpr int kWantPerLevel = 10;

  for (int lvl = 1; lvl <= 3; ++lvl) {
    buildLevel(GfxBuildingKind::Residential, lvl, kWantPerLevel, m_buildingResidential[static_cast<std::size_t>(lvl - 1)]);
    buildLevel(GfxBuildingKind::Commercial, lvl, kWantPerLevel, m_buildingCommercial[static_cast<std::size_t>(lvl - 1)]);
    buildLevel(GfxBuildingKind::Industrial, lvl, kWantPerLevel, m_buildingIndustrial[static_cast<std::size_t>(lvl - 1)]);
  }
}

void Renderer::rebuildPropSprites()
{
  unloadPropSprites();

  // Full-size sprites ...
  GfxPropsConfig cfgTrees{};
  cfgTrees.tileW = m_tileW;
  cfgTrees.tileH = m_tileH;
  cfgTrees.includeEmissive = false;

  GfxPropsConfig cfgLights = cfgTrees;
  cfgLights.includeEmissive = true;

  // Small decorative sprites (pedestrians). We enable emissive so some variants can include a
  // tiny “phone screen” marker at night, but most variants will omit it.
  GfxPropsConfig cfgPeople = cfgTrees;
  cfgPeople.includeEmissive = true;

  const GfxPalette pal = GenerateGfxPalette(m_gfxSeed32 ^ 0x51A5EEDu, GfxTheme::Classic);

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

  auto buildKind = [&](GfxPropKind kind, const GfxPropsConfig& cfg, int want, std::vector<PropSprite>& out) {
    constexpr int kMaxTrials = 96;
    std::string err;
    for (int variant = 0; variant < kMaxTrials; ++variant) {
      if (static_cast<int>(out.size()) >= want) break;

      GfxPropSprite spr{};
      if (!GenerateGfxPropSprite(kind, variant, m_gfxSeed32, cfg, pal, spr, err)) continue;

      PropSprite ps{};
      ps.pivotX = spr.pivotX;
      ps.pivotY = spr.pivotY;
      ps.color = loadTex(spr.color);
      if (!spr.emissive.rgba.empty()) ps.emissive = loadTex(spr.emissive);

      if (ps.color.id == 0) {
        if (ps.emissive.id != 0) UnloadTexture(ps.emissive);
        continue;
      }

      out.push_back(ps);
    }
  };

  // A handful of variants is enough to avoid obvious repetition, while keeping
  // memory + generation time reasonable.
  buildKind(GfxPropKind::TreeDeciduous, cfgTrees, 10, m_propTreeDeciduous);
  buildKind(GfxPropKind::TreeConifer, cfgTrees, 10, m_propTreeConifer);
  buildKind(GfxPropKind::StreetLight, cfgLights, 8, m_propStreetLight);
  buildKind(GfxPropKind::Pedestrian, cfgPeople, 16, m_propPedestrian);
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

void Renderer::resetOrganicMaterial(std::uint32_t seed)
{
  m_organicHasLastTime = false;

  if (!m_organicMaterial.isReady()) {
    m_organicMaterial.init(m_tileW, m_tileH, seed);
    return;
  }

  m_organicMaterial.reset(seed);
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
      const TerrainMacroVisual macroV = ComputeTerrainMacroVisual(world, x, y, t, m_gfxSeed32);
      const Color terrainTint = MulTints(BrightnessTint(brightness), macroV.tint);
      DrawTexturePro(terrainWithTransitions(world, x, y, t), src, dst, Vector2{0, 0}, 0.0f, terrainTint);

      // Draw cliff walls for higher neighbors behind this tile.
      {
        Vector2 baseCorners[4];
        TileDiamondCorners(baseCenter, tileWf, tileHf, baseCorners);

        const float eps = 0.5f;

        const std::uint32_t cliffSeedBase = HashCoords32(x, y, m_gfxSeed32 ^ 0xC1FFEE11u);

        auto drawCliffEdge = [&](Vector2 e0, Vector2 e1, float topElev, float botElev, Color c,
                                 std::uint32_t edgeSeed, bool wetBase) {
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

          const float h = topElev - botElev;

          // Stratified cliff detail (a few horizontal bands) to avoid large flat walls.
          if (h > tileHf * 0.65f) {
            const int bands = std::clamp(static_cast<int>(h / (tileHf * 0.23f)), 2, 8);
            for (int i = 0; i < bands; ++i) {
              const std::uint32_t hh = HashCoords32(i * 31, bands * 17, edgeSeed ^ 0x9E3779B9u);
              if ((hh & 0x3u) == 0u) continue;

              float t = (static_cast<float>(i) + 1.0f) / (static_cast<float>(bands) + 1.0f);
              t += (Frac01(hh) - 0.5f) * 0.06f;
              t = std::clamp(t, 0.08f, 0.92f);

              const Vector2 a = LerpV(bot0, top0, t);
              const Vector2 b = LerpV(bot1, top1, t);

              Color lc = Mul(c, 0.78f);
              lc.a = 75;
              DrawLineEx(a, b, 0.95f, lc);
            }
          }

          // Wet darkening + faint highlight at the base when cliffs meet water.
          if (wetBase) {
            const float bandH = std::min(h, tileHf * 0.32f);
            const float tt = (h > 0.001f) ? (bandH / h) : 1.0f;
            const Vector2 m0 = LerpV(bot0, top0, tt);
            const Vector2 m1 = LerpV(bot1, top1, tt);
            DrawTriangle(bot0, bot1, m1, Color{0, 0, 0, 38});
            DrawTriangle(bot0, m1, m0, Color{0, 0, 0, 38});
            DrawLineEx(bot0, bot1, 0.8f, Color{210, 230, 255, 26});
          }
        };

        if (x > 0) {
          const Tile& n = world.at(x - 1, y);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[3], baseCorners[0], ne, elevPx, Mul(base, 0.70f),
                        cliffSeedBase ^ 0x51A5EEDu, (t.terrain == Terrain::Water));
        }

        if (y > 0) {
          const Tile& n = world.at(x, y - 1);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[0], baseCorners[1], ne, elevPx, Mul(base, 0.85f),
                        cliffSeedBase ^ 0xBADC0DEu, (t.terrain == Terrain::Water));
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

// Terrain transition masks use the same 4-bit layout as roads (World::computeRoadMask):
//  0x01 = (x, y-1)  (screen up-right)
//  0x02 = (x+1, y)  (screen down-right)
//  0x04 = (x, y+1)  (screen down-left)
//  0x08 = (x-1, y)  (screen up-left)
//
// Mask convention: bit=1 means the neighbor is considered *base terrain* (no transition).
// bit=0 means we blend in the edge terrain along that side.
static std::uint8_t WaterSandTransitionMask(const World& world, int x, int y)
{
  auto isWaterOrOob = [&](int nx, int ny) -> bool {
    if (!world.inBounds(nx, ny)) return true;
    return world.at(nx, ny).terrain == Terrain::Water;
  };

  std::uint8_t m = 0;
  if (isWaterOrOob(x, y - 1)) m |= 0x01u;
  if (isWaterOrOob(x + 1, y)) m |= 0x02u;
  if (isWaterOrOob(x, y + 1)) m |= 0x04u;
  if (isWaterOrOob(x - 1, y)) m |= 0x08u;
  return m;
}

static std::uint8_t SandGrassTransitionMask(const World& world, int x, int y)
{
  auto isNotGrassOrOob = [&](int nx, int ny) -> bool {
    if (!world.inBounds(nx, ny)) return true;
    return world.at(nx, ny).terrain != Terrain::Grass;
  };

  std::uint8_t m = 0;
  if (isNotGrassOrOob(x, y - 1)) m |= 0x01u;
  if (isNotGrassOrOob(x + 1, y)) m |= 0x02u;
  if (isNotGrassOrOob(x, y + 1)) m |= 0x04u;
  if (isNotGrassOrOob(x - 1, y)) m |= 0x08u;
  return m;
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

Texture2D& Renderer::terrainWithTransitions(const World& world, int x, int y, const Tile& t)
{
  const std::size_t vi = static_cast<std::size_t>((t.variation >> 4) % kTerrainVariants);

  // Water tiles get a shoreline blend (Water->Sand) when bordering any non-water tile.
  if (t.terrain == Terrain::Water) {
    const std::uint8_t mask = static_cast<std::uint8_t>(WaterSandTransitionMask(world, x, y) & 0x0Fu);
    if (mask != 0x0Fu) {
      Texture2D& tex = m_terrainTransWaterSand[static_cast<std::size_t>(mask)][vi];
      if (tex.id != 0) return tex;
    }
  }

  // Sand tiles blend into Grass only where they touch grass (avoids turning shore sand green).
  if (t.terrain == Terrain::Sand) {
    const std::uint8_t mask = static_cast<std::uint8_t>(SandGrassTransitionMask(world, x, y) & 0x0Fu);
    if (mask != 0x0Fu) {
      Texture2D& tex = m_terrainTransSandGrass[static_cast<std::size_t>(mask)][vi];
      if (tex.id != 0) return tex;
    }
  }

  // No transition needed (or textures not ready) => fall back to base terrain variant.
  return terrain(t.terrain, t.variation);
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
  auto terrainPixel = [&](Terrain kind, int variant, int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t sv = s ^ (static_cast<std::uint32_t>(variant) * 0x9E3779B9u);

    switch (kind) {
    case Terrain::Water: {
      const std::uint32_t h = HashCoords32(x, y, sv ^ 0xA1B2C3D4u);
      const float n = (Frac01(h) - 0.5f) * 0.10f;

      // Subtle diagonal waves (purely procedural), with variant-dependent phase.
      const float phase = static_cast<float>(variant) * 0.65f;
      const float waves0 = 0.060f * std::sin((x * 0.35f + y * 0.70f) + phase);
      const float waves1 = 0.030f * std::sin((x * 0.90f - y * 0.45f) + phase * 1.73f);
      const float b = 1.0f + n + waves0 + waves1;

      Color base = Color{40, 95, 210, 255};
      base = Mul(base, b);

      // Slightly fade edges to reduce harsh tile seams.
      base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 4.0f, 0.0f, 1.0f));
      return base;
    }
    case Terrain::Sand: {
      const std::uint32_t h = HashCoords32(x, y, sv ^ 0xBEEFBEEFu);
      const float n = (Frac01(h) - 0.5f) * 0.18f;

      // Low-frequency "ripples" so dunes don't look perfectly flat.
      const float r = 0.040f * std::sin((x * 0.22f + y * 0.31f) + static_cast<float>(variant) * 1.10f);

      Color base = Color{200, 186, 135, 255};
      base = Mul(base, 1.0f + n + r);

      // Grain speckles.
      if ((h & 0x1Fu) == 0x1Fu) base = Mul(base, 0.85f);
      if ((h & 0x3Fu) == 0x23u) base = Mul(base, 1.08f);

      base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
      return base;
    }
    case Terrain::Grass: {
      const std::uint32_t h = HashCoords32(x, y, sv ^ 0x12345678u);
      const float n = (Frac01(h) - 0.5f) * 0.22f;

      // Macro tint variation within a tile (subtle). This plus variants helps break repetition.
      const float patch = 0.040f * std::sin((x * 0.16f - y * 0.19f) + static_cast<float>(variant) * 0.95f);

      Color base = Color{70, 170, 90, 255};
      base = Mul(base, 1.0f + n + patch);

      // Tiny darker "blades" of grass.
      if ((h & 0x7Fu) == 0x3Fu) base = Mul(base, 0.78f);
      if ((h & 0xFFu) == 0x5Du) base = Mul(base, 0.88f);

      base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
      return base;
    }
    default: return Color{0, 0, 0, 0};
    }
  };

  for (int v = 0; v < kTerrainVariants; ++v) {
    // Water
    m_terrainTex[0][static_cast<std::size_t>(v)] =
        MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
          return terrainPixel(Terrain::Water, v, x, y, d);
        });

    // Sand
    m_terrainTex[1][static_cast<std::size_t>(v)] =
        MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
          return terrainPixel(Terrain::Sand, v, x, y, d);
        });

    // Grass
    m_terrainTex[2][static_cast<std::size_t>(v)] =
        MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
          return terrainPixel(Terrain::Grass, v, x, y, d);
        });
  }

  // --- Terrain transitions ---
  // To avoid harsh biome seams (especially water->sand and sand->grass), we pre-bake 16 auto-tiling
  // masks per transition type. At draw time we select the mask based on neighbor terrain.
  auto smooth01 = [](float t) -> float {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
  };

  auto makeTerrainTransitionVariant = [&](Terrain baseKind, Terrain edgeKind, std::uint8_t mask, int baseVar,
                                          bool shorelineFoam) -> Texture2D {
    const bool waterSand = (baseKind == Terrain::Water && edgeKind == Terrain::Sand);
    const float bw = waterSand ? 0.21f : 0.18f;
    const float jitterAmp = waterSand ? 0.060f : 0.050f;

    const std::uint32_t seedv =
        s ^ 0x13579BDFu ^ (static_cast<std::uint32_t>(mask) * 0x9E3779B9u) ^
        (static_cast<std::uint32_t>(baseVar) * 0x85EBCA6Bu) ^ (waterSand ? 0xC001D00Du : 0x51A5EEDu);

    // Pick an edge variant independently so the blended region isn't always identical to the base tile.
    const int edgeVar = static_cast<int>(((seedv >> 3) ^ 0x55AA7711u) & (kTerrainVariants - 1));

    return MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
      // Distance-to-sides in normalized diamond space (see GfxTileset transition logic).
      const float dUR = 1.0f - (d.nx - d.ny);
      const float dDR = 1.0f - (d.nx + d.ny);
      const float dDL = 1.0f - (-d.nx + d.ny);
      const float dUL = 1.0f - (-d.nx - d.ny);

      const float jitter = (Frac01(HashCoords32(x, y, seedv ^ 0xA11CE5EDu)) - 0.5f) * jitterAmp;

      auto sideW = [&](float dist) -> float {
        const float t = (bw - (dist + jitter)) / bw;
        return smooth01(t);
      };

      float inv = 1.0f;
      if ((mask & 0x01u) == 0u) inv *= (1.0f - sideW(dUR));
      if ((mask & 0x02u) == 0u) inv *= (1.0f - sideW(dDR));
      if ((mask & 0x04u) == 0u) inv *= (1.0f - sideW(dDL));
      if ((mask & 0x08u) == 0u) inv *= (1.0f - sideW(dUL));

      float wEdge = 1.0f - inv;

      // Keep the tile center closer to the base biome.
      wEdge *= smooth01(1.0f - std::clamp(d.edge, 0.0f, 1.0f) * 0.25f);
      wEdge = std::clamp(wEdge, 0.0f, 1.0f);

      const Color base = terrainPixel(baseKind, baseVar, x, y, d);
      const Color edge = terrainPixel(edgeKind, edgeVar, x, y, d);
      Color c = LerpColor(base, edge, wEdge);

      // Optional shoreline foam stripe (Water->Sand only).
      if (shorelineFoam && waterSand) {
        auto stripeW = [&](float dist) -> float {
          const float t = dist / bw;
          const float w = 1.0f - std::fabs(t - 0.55f) / 0.10f;
          return smooth01(w);
        };

        float foam = 0.0f;
        if ((mask & 0x01u) == 0u) foam = std::max(foam, stripeW(dUR));
        if ((mask & 0x02u) == 0u) foam = std::max(foam, stripeW(dDR));
        if ((mask & 0x04u) == 0u) foam = std::max(foam, stripeW(dDL));
        if ((mask & 0x08u) == 0u) foam = std::max(foam, stripeW(dUL));

        // Only show foam near the actual blend band.
        foam *= std::clamp(wEdge * (1.0f - wEdge) * 4.0f, 0.0f, 1.0f);

        // Break foam up with small random gaps.
        const std::uint32_t hf = HashCoords32(x + 13, y - 7, seedv ^ 0xBADA55u);
        foam *= (Frac01(hf) > 0.25f) ? 1.0f : 0.0f;
        foam *= (Frac01(hf ^ 0xC0FFEE11u) > 0.15f) ? 1.0f : 0.0f;

        if (foam > 0.0f && c.a != 0) {
          const unsigned char keepA = c.a;
          Color foamC = Color{245, 250, 255, keepA};
          c = LerpColor(c, foamC, foam);
          c.a = keepA;
        }
      }

      return c;
    });
  };

  for (int mask = 0; mask < 16; ++mask) {
    for (int v = 0; v < kTerrainVariants; ++v) {
      m_terrainTransWaterSand[static_cast<std::size_t>(mask)][static_cast<std::size_t>(v)] =
          makeTerrainTransitionVariant(Terrain::Water, Terrain::Sand, static_cast<std::uint8_t>(mask), v, true);
      m_terrainTransSandGrass[static_cast<std::size_t>(mask)][static_cast<std::size_t>(v)] =
          makeTerrainTransitionVariant(Terrain::Sand, Terrain::Grass, static_cast<std::uint8_t>(mask), v, false);
    }
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
              base = LerpColor(base, Color{250, 250, 250, 255}, 0.85f);
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

  // Procedural building sprites (zone buildings; optional emissive windows).
  rebuildBuildingSprites();

  // Procedural world props (trees, streetlights).
  rebuildPropSprites();

  // World-space cloud shadow mask (procedural, tileable).
  rebuildCloudShadowTexture();

  // Animated procedural organic material overlay uses its own internal textures.
  m_organicHasLastTime = false;
  m_organicMaterial.init(m_tileW, m_tileH, m_gfxSeed32);

  // Optional GPU geometry-shader effects (safe fallback if unsupported).
  m_gpuRibbon.init();
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

  // Active district highlight (0 = none). When enabled we increase contrast for the selected district
  // and downplay other districts to make painting/inspection easier.
  const std::uint8_t highlightDistrictId =
    (showDistrictOverlayEff && highlightDistrict > 0 && highlightDistrict < 256)
      ? static_cast<std::uint8_t>(highlightDistrict)
      : 0u;
  const bool highlightDistrictActive = (highlightDistrictId != 0u);
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

  // Animated procedural organic material (reaction-diffusion). We drive it even when
  // aesthetic details are suppressed so the simulation keeps evolving.
  if (m_organicSettings.enabled) {
    float dtSec = 1.0f / 60.0f;
    if (m_organicHasLastTime) {
      dtSec = static_cast<float>(timeSec - static_cast<double>(m_organicLastTimeSec));
    }
    m_organicLastTimeSec = static_cast<float>(timeSec);
    m_organicHasLastTime = true;

    // Clamp to avoid huge jumps when stepping through breakpoints, pausing, etc.
    dtSec = std::clamp(dtSec, 0.0f, 0.25f);
    m_organicMaterial.update(dtSec, static_cast<float>(timeSec), m_organicSettings);
  } else {
    m_organicHasLastTime = false;
  }

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
  std::vector<WorldSprite> emissivePropSprites;
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

  // High-detail procedural props are intentionally suppressed when debug overlays are enabled
  // (traffic/outside/goods/heatmap) to keep those overlays legible.
  const bool drawPropSprites = layerStructures && !suppressAesthetics && tileScreenW >= 38.0f;
  const bool wantPropEmissive =
    drawAestheticDetails && m_dayNight.enabled && m_dayNight.drawLights && dayNight.nightLights > 0.01f &&
    tileScreenW >= 24.0f;

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
  const bool drawPropShadows = drawShadows && drawPropSprites;
  std::vector<BuildingShadowCaster> shadowCasters;
  if (drawShadows) {
    // Rough heuristic: only a fraction of visible tiles have buildings.
    // When prop sprites are enabled we also cast shadows for trees/streetlights, so reserve a bit more.
    const int visW = vis.maxX - vis.minX + 1;
    const int visH = vis.maxY - vis.minY + 1;
    const int denom = drawPropShadows ? 4 : 6;
    shadowCasters.reserve(static_cast<std::size_t>(std::max(0, (visW * visH) / denom)));
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

      const TileLighting light =
        ComputeTileLighting(world, x, y, tileWf, tileHf, m_elev, timeSec, animatedLighting);
      const float baseBrightness = light.base;
      const float brightness = animatedLighting ? light.animated : baseBrightness;

      // -----------------------------
      // Terrain (if not cached)
      // -----------------------------
      if (layerTerrain && !terrainCacheReady) {
        const TerrainMacroVisual macroV = ComputeTerrainMacroVisual(world, x, y, tile, m_gfxSeed32);
        const Color terrainTint = MulTints(BrightnessTint(brightness), macroV.tint);
        DrawTexturePro(terrainWithTransitions(world, x, y, tile), src, dst, Vector2{0, 0}, 0.0f, terrainTint);

        // Cliff walls for higher tiles behind.
        Vector2 baseCorners[4];
        TileDiamondCorners(baseCenter, tileWf, tileHf, baseCorners);

        const float eps = 0.5f;
        const float invZoom = 1.0f / std::max(0.001f, camera.zoom);
        const std::uint32_t cliffSeedBase = HashCoords32(x, y, m_gfxSeed32 ^ 0xC1FFEE11u);

        auto drawCliffEdge = [&](Vector2 e0, Vector2 e1, float topElev, float botElev, Color c,
                                 std::uint32_t edgeSeed, bool wetBase) {
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

          const float h = topElev - botElev;

          // Stratified cliff detail (horizontal bands) - tuned to be subtle at low zoom.
          if (h > tileHf * 0.65f && tileScreenW >= 14.0f) {
            const int bands = std::clamp(static_cast<int>(h / (tileHf * 0.23f)), 2, 8);
            const float thick = std::clamp(1.10f * invZoom, 0.55f * invZoom, 2.2f * invZoom);

            for (int i = 0; i < bands; ++i) {
              const std::uint32_t hh = HashCoords32(i * 31, bands * 17, edgeSeed ^ 0x9E3779B9u);
              if ((hh & 0x3u) == 0u) continue;

              float t = (static_cast<float>(i) + 1.0f) / (static_cast<float>(bands) + 1.0f);
              t += (Frac01(hh) - 0.5f) * 0.06f;
              t = std::clamp(t, 0.08f, 0.92f);

              const Vector2 a = LerpV(bot0, top0, t);
              const Vector2 b = LerpV(bot1, top1, t);

              Color lc = Mul(c, 0.78f);
              lc.a = 80;
              DrawLineEx(a, b, thick, lc);
            }
          }

          // Wet darkening + faint highlight at the base when cliffs meet water.
          if (wetBase) {
            const float bandH = std::min(h, tileHf * 0.32f);
            const float tt = (h > 0.001f) ? (bandH / h) : 1.0f;
            const Vector2 m0 = LerpV(bot0, top0, tt);
            const Vector2 m1 = LerpV(bot1, top1, tt);
            DrawTriangle(bot0, bot1, m1, Color{0, 0, 0, 38});
            DrawTriangle(bot0, m1, m0, Color{0, 0, 0, 38});
            DrawLineEx(bot0, bot1, std::clamp(0.95f * invZoom, 0.55f * invZoom, 2.0f * invZoom), Color{210, 230, 255, 26});
          }
        };

        if (x > 0) {
          const Tile& n = world.at(x - 1, y);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[3], baseCorners[0], ne, elevPx, Mul(base, 0.70f),
                        cliffSeedBase ^ 0x51A5EEDu, (tile.terrain == Terrain::Water));
        }

        if (y > 0) {
          const Tile& n = world.at(x, y - 1);
          const float ne = TileElevationPx(n, m_elev);
          const Color base = TerrainCliffBaseColor(n.terrain);
          drawCliffEdge(baseCorners[0], baseCorners[1], ne, elevPx, Mul(base, 0.85f),
                        cliffSeedBase ^ 0xBADC0DEu, (tile.terrain == Terrain::Water));
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
        // Animated, procedural organic material overlay (reaction-diffusion texture).
        if (m_organicSettings.enabled && m_organicMaterial.isReady()) {
          const TerrainMacroVisual macroV = ComputeTerrainMacroVisual(world, x, y, tile, m_gfxSeed32);

          float coverage = 0.0f;
          // Keep it mostly on land tiles and parks; avoid roads and zone overlays.
          const bool allow = (tile.terrain != Terrain::Water) &&
                             (tile.overlay == Overlay::None || tile.overlay == Overlay::Park);
          if (allow) {
            coverage = (tile.overlay == Overlay::Park) ? 1.0f : ((tile.terrain == Terrain::Grass) ? 0.65f : 0.35f);
            coverage *= (1.0f - macroV.snow);
            coverage *= (0.45f + 0.55f * weather.wetness);

            const bool waterAdj = ((x > 0) && (world.at(x - 1, y).terrain == Terrain::Water)) ||
                                  ((x < mapW - 1) && (world.at(x + 1, y).terrain == Terrain::Water)) ||
                                  ((y > 0) && (world.at(x, y - 1).terrain == Terrain::Water)) ||
                                  ((y < mapH - 1) && (world.at(x, y + 1).terrain == Terrain::Water));
            if (waterAdj) coverage *= 1.25f;

            const std::uint32_t h = HashCoords32(x, y, m_gfxSeed32 ^ 0xA53u);
            const float rnd = static_cast<float>(h & 0xFFFFu) * (1.0f / 65535.0f);
            coverage *= (0.55f + 0.45f * rnd);
            coverage = std::clamp(coverage, 0.0f, 1.0f);
          }

          const float alphaF = m_organicSettings.alpha * coverage;
          if (alphaF > 0.01f) {
            const std::uint32_t h = HashCoords32(x, y, m_gfxSeed32 ^ 0xB10u);
            const int vcount = std::max(1, OrganicMaterial::kVariants);
            const int variant = static_cast<int>(h % static_cast<std::uint32_t>(vcount));
            const Texture2D& tex = m_organicMaterial.variantTex(variant);

            Color tint = WHITE;
            switch (m_organicSettings.style) {
              case OrganicMaterial::Style::Moss: tint = Color{90, 220, 140, 255}; break;
              case OrganicMaterial::Style::Slime: tint = Color{70, 230, 220, 255}; break;
              case OrganicMaterial::Style::Mycelium: tint = Color{225, 215, 190, 255}; break;
              case OrganicMaterial::Style::Bioluminescent: tint = Color{45, 255, 190, 255}; break;
            }

            const float wetBoost = 0.90f + 0.25f * weather.wetness;
            float brightnessK = 0.65f + 0.35f * dayNight.sun;
            if (m_organicSettings.glowAtNight && (dayNight.nightLights > 0.001f)) {
              brightnessK += dayNight.nightLights * m_organicSettings.glowStrength;
            }
            tint = Mul(tint, wetBoost * brightnessK);

            if (m_organicSettings.style == OrganicMaterial::Style::Bioluminescent && dayNight.nightLights > 0.001f) {
              const float phase = static_cast<float>(h & 0xFFFFu) * (6.2831853f / 65535.0f);
              const float pulse = 0.85f + 0.15f * std::sin(static_cast<float>(timeSec) * 2.2f + phase);
              tint = Mul(tint, pulse);
            }

            tint.a = ClampU8(static_cast<int>(alphaF * 255.0f));

            const Rectangle srcR = {0.0f, 0.0f, static_cast<float>(tex.width), static_cast<float>(tex.height)};
            const Rectangle dstR = {center.x - 0.5f * tileWf, center.y - 0.5f * tileHf, tileWf, tileHf};
            DrawTexturePro(tex, srcR, dstR, Vector2{0, 0}, 0.0f, tint);
          }
        }

        // Procedural micro-detail pass (grass tufts, rocks, water sparkles, etc.)
        DrawProceduralTileDetails(world, x, y, tile, center, tileWf, tileHf,
                                 camera.zoom, brightness, m_gfxSeed32, timeSec, weather);

        // Permanent altitude-driven snow caps (independent of the active weather mode).
        // Kept subtle so it doesn't fight utility overlays; also fades out when it's actively snowing.
        {
          const TerrainMacroVisual macroV = ComputeTerrainMacroVisual(world, x, y, tile, m_gfxSeed32);
          float snow = macroV.snow * (1.0f - 0.80f * weather.snow);

          if (tile.overlay == Overlay::Road) snow *= 0.40f;
          else if (IsZoneOverlay(tile.overlay) || tile.overlay == Overlay::Park) snow *= 0.25f;

          if (snow > 0.01f) {
            Vector2 c[4];
            TileDiamondCorners(center, tileWf, tileHf, c);
            for (int i = 0; i < 4; ++i) {
              c[i] = LerpV(c[i], center, 0.05f);
            }

            const unsigned char a = ClampU8(static_cast<int>(160.0f * snow));
            const Color snowC = ShadeDetail(Color{250, 250, 255, 255}, brightness, 1.08f, a);
            DrawTriangle(c[0], c[1], c[2], snowC);
            DrawTriangle(c[0], c[2], c[3], snowC);
          }
        }

        // Ground weather effects (wet sheen, snow cover, etc.)
        if (m_weather.affectGround) {
          DrawWeatherGroundEffects(world, x, y, tile, center, tileWf, tileHf,
                                  camera.zoom, brightness, dayNight, weather, timeSec, m_gfxSeed32);
        }
      }

      // Coastline: keep a thin highlight for readability when aesthetics are suppressed.
      // When aesthetics are on, upgrade to procedural shallow-water, foam, and wet-sand bands.
      if (layerDecals && tileScreenW >= 18.0f) {
        const bool rich = drawAestheticDetails && tileScreenW >= 24.0f;

        Vector2 c[4];
        TileDiamondCorners(center, tileWf, tileHf, c);
        const Vector2 edgeA[4] = {c[0], c[1], c[2], c[3]};
        const Vector2 edgeB[4] = {c[1], c[2], c[3], c[0]};

        // Neighbor queries; out-of-bounds treated as "land" for water tiles (so map-edge water gets outlined).
        const bool leftWater = (x > 0) && (world.at(x - 1, y).terrain == Terrain::Water);
        const bool rightWater = (x < mapW - 1) && (world.at(x + 1, y).terrain == Terrain::Water);
        const bool upWater = (y > 0) && (world.at(x, y - 1).terrain == Terrain::Water);
        const bool downWater = (y < mapH - 1) && (world.at(x, y + 1).terrain == Terrain::Water);

        const bool leftLand = !leftWater;
        const bool rightLand = !rightWater;
        const bool upLand = !upWater;
        const bool downLand = !downWater;

        const float invZoom = 1.0f / std::max(0.001f, camera.zoom);

        auto drawBand = [&](int edge, float inset0, float inset1, Color col) {
          const Vector2 a0 = LerpV(edgeA[edge], center, inset0);
          const Vector2 a1 = LerpV(edgeB[edge], center, inset0);
          const Vector2 b0 = LerpV(edgeA[edge], center, inset1);
          const Vector2 b1 = LerpV(edgeB[edge], center, inset1);
          DrawTriangle(a0, a1, b1, col);
          DrawTriangle(a0, b1, b0, col);
        };

        auto drawFoam = [&](int edge, std::uint32_t edgeSeed) {
          // Broken, slightly inset segments so coastlines don't look like a single perfect stroke.
          const Vector2 e0 = LerpV(edgeA[edge], center, 0.06f);
          const Vector2 e1 = LerpV(edgeB[edge], center, 0.06f);

          const int segCount = (tileScreenW >= 44.0f) ? 4 : 3;
          const float thick = std::clamp(1.45f * invZoom, 0.75f * invZoom, 2.6f * invZoom);

          for (int s = 0; s < segCount; ++s) {
            const float tA = static_cast<float>(s) / static_cast<float>(segCount);
            const float tB = static_cast<float>(s + 1) / static_cast<float>(segCount);
            const std::uint32_t hs = HashCoords32(s * 31, edge * 17, edgeSeed ^ 0xA3613F13u);
            const float r0 = Frac01(hs);
            const float r1 = Frac01(hs ^ 0x9E3779B9u);
            if (r0 < 0.20f) continue;

            float s0 = tA + (0.10f + 0.25f * r0) * (tB - tA);
            float s1 = tB - (0.10f + 0.25f * r1) * (tB - tA);
            if (s1 <= s0) continue;

            Vector2 p0 = LerpV(e0, e1, s0);
            Vector2 p1 = LerpV(e0, e1, s1);

            // Small inward push to keep foam away from the exact tile seam.
            const Vector2 mid = LerpV(p0, p1, 0.5f);
            Vector2 dir{center.x - mid.x, center.y - mid.y};
            const float dlen = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (dlen > 0.001f) {
              dir.x /= dlen;
              dir.y /= dlen;
            }
            const float off = (0.8f + 0.6f * Frac01(hs ^ 0x85EBCA6Bu)) * invZoom;
            p0.x += dir.x * off;
            p0.y += dir.y * off;
            p1.x += dir.x * off;
            p1.y += dir.y * off;

            const float pulse01 =
              0.55f +
              0.45f *
                (0.5f + 0.5f * std::sin(timeSec * 2.20f + static_cast<float>(edge) * 1.40f +
                                        Frac01(hs ^ 0xC3A5C85Cu) * 6.2831853f));

            const int aFoam = static_cast<int>((40.0f + 60.0f * (1.0f - 0.35f * r0)) * pulse01);

            Color foam = ShadeDetail(Color{255, 255, 255, 255}, brightness, 1.12f, ClampU8(aFoam));
            DrawLineEx(p0, p1, thick, foam);

            // Occasional bubbles.
            if (tileScreenW >= 40.0f && ((hs >> 28) & 3u) == 0u) {
              const Vector2 bp = LerpV(p0, p1, 0.5f + (Frac01(hs ^ 0xC3A5C85Cu) - 0.5f) * 0.25f);
              DrawCircleV(bp, std::clamp(1.05f * invZoom, 0.65f * invZoom, 2.0f * invZoom),
                          Color{255, 255, 255, 35});
            }
          }
        };

        const bool tileIsWater = (tile.terrain == Terrain::Water);
        if (tileIsWater) {
          const bool edgeLand[4] = {upLand, rightLand, downLand, leftLand};
          const bool hasLandNeighbor = edgeLand[0] || edgeLand[1] || edgeLand[2] || edgeLand[3];
          if (hasLandNeighbor) {
            if (!rich) {
              // Thin fallback highlight.
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
            } else {
              // Shallow water band + foam.
              const std::uint32_t baseSeed = HashCoords32(x, y, m_gfxSeed32 ^ 0xF00DFACEu);
              for (int e = 0; e < 4; ++e) {
                if (!edgeLand[e]) continue;

                const std::uint32_t hs = HashCoords32(e * 97, e * 131, baseSeed ^ 0x27D4EB2Du);
                const float w = 0.18f + 0.05f * (Frac01(hs) - 0.5f);

                Color shallow = ShadeDetail(Color{85, 165, 230, 255}, brightness, 1.05f,
                                            ClampU8(42 + static_cast<int>(40.0f * Frac01(hs ^ 0x9E3779B9u))));
                drawBand(e, 0.02f, 0.02f + w, shallow);

                drawFoam(e, baseSeed ^ (0x9E3779B9u * static_cast<std::uint32_t>(e + 1)));
              }
            }
          }
        }

        // Wet sand / wet grass fringe on the land side (only when aesthetics are on).
        if (rich && tile.overlay == Overlay::None) {
          const bool edgeWater[4] = {upWater, rightWater, downWater, leftWater};
          const bool hasWaterNeighbor = edgeWater[0] || edgeWater[1] || edgeWater[2] || edgeWater[3];
          if (hasWaterNeighbor) {
            if (tile.terrain == Terrain::Sand) {
              Color wet = ShadeDetail(Color{65, 55, 45, 255}, brightness, 1.00f, 55);
              for (int e = 0; e < 4; ++e) {
                if (!edgeWater[e]) continue;
                drawBand(e, 0.02f, 0.16f, wet);
                DrawLineEx(LerpV(edgeA[e], center, 0.08f), LerpV(edgeB[e], center, 0.08f),
                           std::clamp(0.85f * invZoom, 0.55f * invZoom, 1.8f * invZoom),
                           Color{240, 235, 220, 18});
              }
            } else if (tile.terrain == Terrain::Grass) {
              Color wet = ShadeDetail(Color{20, 55, 28, 255}, brightness, 1.00f, 45);
              for (int e = 0; e < 4; ++e) {
                if (!edgeWater[e]) continue;
                drawBand(e, 0.03f, 0.11f, wet);
              }
            }
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

      // -----------------------------
      // Prop shadow casters (procedural trees/streetlights)
      // -----------------------------
      // These reuse the same shadow projection pass as buildings, but their footprints/heights are derived
      // from the procedural prop sprites so they feel grounded and consistent across variants.
      if (drawPropShadows) {
        // Trees (parks)
        if (tile.overlay == Overlay::Park &&
            (!m_propTreeDeciduous.empty() || !m_propTreeConifer.empty()) &&
            tileScreenW >= 44.0f) {
          const std::uint32_t h = HashCoords32(x, y, m_gfxSeed32 ^ 0x7A11EE5u);
          int count = 1;
          if (tileScreenW >= 70.0f && ((h >> 3u) & 3u) == 0u) {
            count = 2;
          }

          for (int i = 0; i < count; ++i) {
            const std::uint32_t hi = HashCoords32(i, static_cast<int>(h), m_gfxSeed32 ^ 0xC0FFEEu);

            // In snow weather we bias toward conifers so parks feel seasonal.
            const bool preferConifer =
              (weather.mode == WeatherSettings::Mode::Snow) ? ((hi & 1u) == 0u) : ((hi & 3u) == 0u);

            const auto& primary = preferConifer ? m_propTreeConifer : m_propTreeDeciduous;
            const auto& fallback = preferConifer ? m_propTreeDeciduous : m_propTreeConifer;
            const auto& v = !primary.empty() ? primary : fallback;
            if (v.empty()) break;

            const std::size_t vidx = static_cast<std::size_t>(hi % static_cast<std::uint32_t>(v.size()));
            const PropSprite& ps = v[vidx];

            Vector2 p = DeterministicDiamondPoint(x, y, m_gfxSeed32 ^ 0x71EED00u, i, center, tileWf, tileHf, 0.78f);

            BuildingShadowCaster caster{};

            // Canopy footprint: larger than trunk so the tree shadow reads as a soft blob.
            const float baseK = 0.42f + 0.10f * (Frac01(hi ^ 0xA1B2C3D4u) - 0.5f);
            const float baseW = tileWf * std::clamp(baseK, 0.30f, 0.55f);
            const float baseH = tileHf * std::clamp(baseK, 0.30f, 0.55f);
            TileDiamondCorners(p, baseW, baseH, caster.base);

            // PropSprite pivotY is the height above the ground pivot in world pixels.
            const float hPx = static_cast<float>(ps.pivotY) * 0.95f;
            caster.heightPx = std::clamp(hPx, tileHf * 1.0f, tileHf * 4.0f);

            const float hNorm = std::clamp(caster.heightPx / (tileHf * 3.2f), 0.0f, 1.0f);
            caster.alphaScale = std::clamp(0.55f + 0.25f * hNorm, 0.45f, 0.85f);
            shadowCasters.push_back(caster);
          }
        }

        // Streetlights (roads)
        if (tile.overlay == Overlay::Road && !m_propStreetLight.empty() &&
            tile.terrain != Terrain::Water && tileScreenW >= 52.0f) {
          const std::uint8_t mask = static_cast<std::uint8_t>(tile.variation & 0x0Fu);
          const int conn = Popcount4(mask);

          const std::uint32_t h = HashCoords32(x, y, m_gfxSeed32 ^ 0x51A7C0DEu);
          bool place = false;
          if (conn >= 3) {
            place = true;
          } else if (conn == 2) {
            place = ((h & 7u) == 0u);
          }

          if (place) {
            const std::size_t vidx = static_cast<std::size_t>((h >> 8u) % static_cast<std::uint32_t>(m_propStreetLight.size()));
            const PropSprite& ps = m_propStreetLight[vidx];

            Vector2 corners[4];
            TileDiamondCorners(center, tileWf, tileHf, corners);

            const bool left = ((h & 1u) == 0u);
            Vector2 pivot = left ? LerpV(corners[0], corners[3], 0.72f) : LerpV(corners[0], corners[1], 0.72f);
            pivot.y += tileHf * 0.05f;

            BuildingShadowCaster caster{};
            TileDiamondCorners(pivot, tileWf * 0.18f, tileHf * 0.18f, caster.base);

            const float hPx = static_cast<float>(ps.pivotY) * 0.85f;
            caster.heightPx = std::clamp(hPx, tileHf * 0.8f, tileHf * 3.0f);

            const float hNorm = std::clamp(caster.heightPx / (tileHf * 2.6f), 0.0f, 1.0f);
            caster.alphaScale = std::clamp(0.32f + 0.18f * hNorm, 0.25f, 0.55f);
            shadowCasters.push_back(caster);
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

  auto popCount4 = [](std::uint8_t m) -> int {
    m &= 0x0Fu;
    // https://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    m = static_cast<std::uint8_t>(m - ((m >> 1u) & 0x55u));
    m = static_cast<std::uint8_t>((m & 0x33u) + ((m >> 2u) & 0x33u));
    return static_cast<int>(((m + (m >> 4u)) & 0x0Fu));
  };

  auto drawProp = [&](const PropSprite& ps, Vector2 pivotWorld, Color tint, float rotationDeg) {
    if (ps.color.id == 0) return;
    const Rectangle s{0.0f, 0.0f, static_cast<float>(ps.color.width), static_cast<float>(ps.color.height)};
    const Rectangle d{pivotWorld.x - static_cast<float>(ps.pivotX),
                      pivotWorld.y - static_cast<float>(ps.pivotY),
                      static_cast<float>(ps.color.width),
                      static_cast<float>(ps.color.height)};
    // Rotate about the sprite pivot so the base stays anchored to the tile.
    const Vector2 origin{static_cast<float>(ps.pivotX), static_cast<float>(ps.pivotY)};
    DrawTexturePro(ps.color, s, d, origin, rotationDeg, tint);
  };

  auto queueEmissiveSprite = [&](const Texture2D& tex, int pivotX, int pivotY, Vector2 pivotWorld,
                                 unsigned char alpha, int sortSum, float sortX) {
    if (alpha == 0) return;
    if (tex.id == 0) return;
    WorldSprite ws{};
    ws.sortSum = sortSum;
    ws.sortX = sortX;
    ws.tex = &tex;
    ws.src = Rectangle{0.0f, 0.0f, static_cast<float>(tex.width), static_cast<float>(tex.height)};
    ws.dst = Rectangle{pivotWorld.x - static_cast<float>(pivotX),
                       pivotWorld.y - static_cast<float>(pivotY),
                       static_cast<float>(tex.width),
                       static_cast<float>(tex.height)};
    ws.tint = Color{255, 255, 255, alpha};
    ws.emissive = true;
    emissivePropSprites.push_back(ws);
  };

  // Pick a pedestrian sprite variant with a cheap walk-cycle (two pose variants per “style”).
  auto pickPedestrianSprite = [&](std::uint32_t h, float tSec) -> const PropSprite* {
    if (m_propPedestrian.empty()) return nullptr;
    const int n = static_cast<int>(m_propPedestrian.size());
    if (n <= 1) return &m_propPedestrian.front();

    const int styles = std::max(1, n / 2);
    const int style = static_cast<int>((h >> 8u) % static_cast<std::uint32_t>(styles));

    const float speed = 1.25f + 1.05f * Frac01(h ^ 0xA11CE5u);
    const float phase = Frac01(h ^ 0xBADC0DEu) * 2.0f;
    const int frame = (static_cast<int>(std::floor(tSec * speed + phase)) & 1);

    int idx = style * 2 + frame;
    if (idx < 0) idx = 0;
    if (idx >= n) idx = idx % n;
    return &m_propPedestrian[static_cast<std::size_t>(idx)];
  };

  // High-zoom procedural building sprites (adds detail on top of the existing prism-based
  // buildings without requiring any external art assets).
  const bool drawZoneBuildingSprites =
    drawAestheticDetails && layerStructures && tileScreenW >= 54.0f &&
    (!m_buildingResidential[0].empty() || !m_buildingCommercial[0].empty() || !m_buildingIndustrial[0].empty());

  const bool wantBuildingEmissive =
    drawZoneBuildingSprites && m_dayNight.enabled && m_dayNight.drawLights && dayNight.nightLights > 0.01f;

  auto pickBuildingSprite = [&](Overlay ov, int lvl, std::uint32_t style) -> const BuildingSprite* {
    const int li = std::clamp(lvl, 1, 3) - 1;
    const std::array<std::vector<BuildingSprite>, 3>* levels = nullptr;
    switch (ov) {
    case Overlay::Residential:
      levels = &m_buildingResidential;
      break;
    case Overlay::Commercial:
      levels = &m_buildingCommercial;
      break;
    case Overlay::Industrial:
      levels = &m_buildingIndustrial;
      break;
    default:
      return nullptr;
    }
    const auto& v = (*levels)[static_cast<std::size_t>(li)];
    if (v.empty()) return nullptr;
    const std::size_t idx = static_cast<std::size_t>(style % static_cast<std::uint32_t>(v.size()));
    return &v[idx];
  };

  auto drawZoneBuildingSprite = [&](const Tile& t, int x, int y, int sum, Vector2 center, float brightness) -> bool {
    if (!drawZoneBuildingSprites) return false;
    if (!IsZoneOverlay(t.overlay)) return false;

    const int lvl = std::clamp(static_cast<int>(t.level), 1, 3);
    const std::uint32_t style =
      HashCoords32(x, y, m_gfxSeed32 ^ 0xB1D1B00Du ^ (static_cast<std::uint32_t>(t.variation) * 0x9E3779B9u));

    const BuildingSprite* bs = pickBuildingSprite(t.overlay, lvl, style);
    if (!bs || bs->color.id == 0) return false;

    const Vector2 topLeft{center.x - static_cast<float>(bs->pivotX), center.y - static_cast<float>(bs->pivotY)};
    DrawTextureV(bs->color, topLeft, BrightnessTint(brightness));

    if (wantBuildingEmissive && bs->emissive.id != 0) {
      const int cap = CapacityForTile(t);
      const float occRatio = (cap > 0) ? std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f)
                                       : 0.0f;
      const float lit = std::clamp(dayNight.nightLights, 0.0f, 1.0f) * std::clamp(0.20f + 0.80f * occRatio, 0.0f, 1.0f);
      const unsigned char a = ClampU8(static_cast<int>(235.0f * lit));
      if (a != 0) {
        queueEmissiveSprite(bs->emissive, bs->pivotX, bs->pivotY, center, a, sum, static_cast<float>(x) + 0.35f);
      }
    }

    return true;
  };

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
      //
      // NOTE: Structures (buildings, trees, streetlights) can extend well above the base diamond,
      // so we expand the AABB upward to prevent edge-of-screen popping when only the top of an
      // object is visible.
      Rectangle tileAABB{center.x - tileWf * 0.5f, center.y - tileHf * 0.5f, tileWf, tileHf};
      if (layerStructures && tileScreenW >= 26.0f) {
        const float padTop = tileHf * 2.9f;
        tileAABB.y -= padTop;
        tileAABB.height += padTop;
      }
      if (!CheckCollisionRecs(tileAABB, viewRect)) {
        continue;
      }

      // Lighting (must match pass 1 so overlays/structures agree).
      const TileLighting light = ComputeTileLighting(world, x, y, tileWf, tileHf, m_elev, timeSec, animatedLighting);
      const float brightness = animatedLighting ? light.animated : light.base;

      // -----------------------------
      // District overlay fill (overlay layer)
      // -----------------------------
      if (showDistrictOverlayEff && tileScreenW >= 6.0f) {
        const std::uint8_t did = t.district;
        if (did != 0u) {
          // Soft fill; alpha reduced when zoomed out. When a district is selected, increase
          // the selected district's contrast and deemphasize all others.
          const float alphaK = std::clamp((tileScreenW - 6.0f) / 18.0f, 0.0f, 1.0f);

          int a = static_cast<int>(40.0f + 80.0f * alphaK);
          if (highlightDistrictActive) {
            if (did == highlightDistrictId) {
              a = static_cast<int>(70.0f + 140.0f * alphaK);
            } else {
              a = static_cast<int>(20.0f + 40.0f * alphaK);
            }
          }

          DrawDiamond(center, tileWf, tileHf, DistrictFillColor(did, ClampU8(a)));
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
            const bool hiEdge = highlightDistrictActive && (t.district == highlightDistrictId || nd == highlightDistrictId);

            Vector2 corners[4];
            TileDiamondCorners(center, tileWf, tileHf, corners);

            if (hiEdge) {
              // Brighter, thicker outline for the active district boundary.
              const float ww = w * 1.35f;
              DrawLineEx(corners[cornerA], corners[cornerB], ww, Color{0, 0, 0, 200});
              DrawLineEx(corners[cornerA], corners[cornerB], ww * 0.70f,
                         DistrictFillColor(highlightDistrictId, 180));
              DrawLineEx(corners[cornerA], corners[cornerB], ww * 0.32f, Color{255, 255, 255, 120});
            } else {
              // If a district is selected, downplay unrelated borders to reduce visual noise.
              const unsigned char aOuter = highlightDistrictActive ? 110 : 160;
              const unsigned char aInner = highlightDistrictActive ? 45 : 70;
              DrawLineEx(corners[cornerA], corners[cornerB], w, Color{0, 0, 0, aOuter});
              DrawLineEx(corners[cornerA], corners[cornerB], w * 0.5f, Color{255, 255, 255, aInner});
            }
          }
        };

        // North edge: between TL (0) and TR (1)
        drawEdge(x, y - 1, 0, 1);
        // East edge: between TR (1) and BR (2)
        drawEdge(x + 1, y, 1, 2);
      }

      // -----------------------------
      // Procedural world props (structures)
      // -----------------------------
      if (drawPropSprites) {
        const Color baseTint = BrightnessTint(brightness);

        // Trees for park tiles.
        if (t.overlay == Overlay::Park && (!m_propTreeDeciduous.empty() || !m_propTreeConifer.empty()) && tileScreenW >= 44.0f) {
          const std::uint32_t h = HashCoords32(x, y, m_gfxSeed32 ^ 0x7A11EE5u);

          // LOD: always one tree when sufficiently zoomed in, with a second tree occasionally at higher zoom.
          int count = 1;
          if (tileScreenW >= 70.0f && ((h >> 3u) & 3u) == 0u) {
            count = 2;
          }

          for (int i = 0; i < count; ++i) {
            const std::uint32_t hi = HashCoords32(i, static_cast<int>(h), m_gfxSeed32 ^ 0xC0FFEEu);

            // Bias toward conifers when it's snowing.
            const bool preferConifer = (weather.mode == WeatherSettings::Mode::Snow) ? ((hi & 1u) == 0u) : ((hi & 3u) == 0u);
            const auto& primary = preferConifer ? m_propTreeConifer : m_propTreeDeciduous;
            const auto& fallback = preferConifer ? m_propTreeDeciduous : m_propTreeConifer;
            const auto& v = !primary.empty() ? primary : fallback;
            if (v.empty()) break;

            const std::size_t vidx = static_cast<std::size_t>(hi % static_cast<std::uint32_t>(v.size()));
            const PropSprite& ps = v[vidx];

            // Keep bases away from tile edges so trunks don't sit on roads/sidewalks.
            Vector2 p = DeterministicDiamondPoint(x, y, m_gfxSeed32 ^ 0x71EED00u, i, center, tileWf, tileHf, 0.78f);

            // Trees are a bit darker than buildings/roads in our stylized lighting.
            Color tint = Mul(baseTint, 0.92f);
            // Simple wind sway (purely visual): small rotation around the pivot.
            float rot = 0.0f;
            {
              const float windN = std::clamp((0.25f + 0.75f * weather.intensity) *
                                              (0.40f + 0.60f * std::clamp(weather.windSpeed / 1.8f, 0.0f, 1.0f)),
                                            0.0f, 1.0f);
              const float ampBase = preferConifer ? 1.55f : 2.35f;
              const float amp = ampBase * (0.75f + 0.55f * Frac01(hi ^ 0xF00DBABEu));
              const float freq = 0.70f + 0.55f * Frac01(hi ^ 0x1234567u);
              const float phase = Frac01(hi ^ 0x9E3779B9u) * 6.2831853f;
              const float gust = 0.70f + 0.30f * std::sin(timeSec * 0.35f + Frac01(hi) * 6.2831853f);

              rot = std::sin(timeSec * freq + phase) * amp * windN * gust;

              // Fade-in so distant trees don't shimmer.
              rot *= std::clamp((tileScreenW - 44.0f) / 40.0f, 0.0f, 1.0f);
            }

            drawProp(ps, p, tint, rot);
          }
        }

        // Streetlights on roads (mostly intersections) for “life” at high zoom.
        if (t.overlay == Overlay::Road && !m_propStreetLight.empty() && t.terrain != Terrain::Water && tileScreenW >= 52.0f) {
          const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);
          const int conn = popCount4(mask);

          const std::uint32_t h = HashCoords32(x, y, m_gfxSeed32 ^ 0x51A7C0DEu);
          bool place = false;
          if (conn >= 3) {
            place = true; // intersections
          } else if (conn == 2) {
            place = ((h & 7u) == 0u); // occasional along straights
          }

          if (place) {
            const std::size_t vidx = static_cast<std::size_t>((h >> 8u) % static_cast<std::uint32_t>(m_propStreetLight.size()));
            const PropSprite& ps = m_propStreetLight[vidx];

            Vector2 corners[4];
            TileDiamondCorners(center, tileWf, tileHf, corners);

            const bool left = ((h & 1u) == 0u);
            Vector2 pivot = left ? LerpV(corners[0], corners[3], 0.72f) : LerpV(corners[0], corners[1], 0.72f);
            pivot.y += tileHf * 0.05f;

            float rot = 0.0f;
            {
              const std::uint32_t hs = HashCoords32(x, y, h ^ 0xD1CEB00Fu);

              const float windN = std::clamp((0.20f + 0.80f * weather.intensity) *
                                              (0.35f + 0.65f * std::clamp(weather.windSpeed / 2.2f, 0.0f, 1.0f)),
                                            0.0f, 1.0f);
              const float amp = 0.85f * (0.70f + 0.50f * Frac01(hs));
              const float freq = 0.85f + 0.45f * Frac01(hs ^ 0x9E3779B9u);
              const float phase = Frac01(hs ^ 0x51A5EEDu) * 6.2831853f;

              rot = std::sin(timeSec * freq + phase) * amp * windN;

              rot *= std::clamp((tileScreenW - 52.0f) / 50.0f, 0.0f, 1.0f);
            }

            drawProp(ps, pivot, Mul(baseTint, 0.98f), rot);

            // Optional emissive sprite (lamp head) so the light stays bright after night grading.
            if (wantPropEmissive) {
              const unsigned char a = ClampU8(static_cast<int>(220.0f * std::clamp(dayNight.nightLights, 0.0f, 1.0f)));
              queueEmissiveSprite(ps.emissive, ps.pivotX, ps.pivotY, pivot, a, sum, static_cast<float>(x) + 0.25f);
            }
          }
        }

        // Pedestrians: small decorative sprites to add “city life” when zoomed in.
        //
        // We spawn them deterministically on:
        //  - park tiles (leisure), and
        //  - road tiles that border an active zone (sidewalk activity).
        if (!m_propPedestrian.empty() && tileScreenW >= 56.0f) {
          const float invZoom = 1.0f / std::max(0.001f, camera.zoom);

          auto crowdFactor = [&]() -> float {
            // Less foot traffic in heavy rain/snow and at night.
            float f = 1.0f;
            if (weather.mode != WeatherSettings::Mode::Clear) {
              f *= std::clamp(1.0f - 0.60f * std::clamp(weather.intensity, 0.0f, 1.0f), 0.18f, 1.0f);
            }
            f *= std::clamp(0.30f + 0.70f * dayNight.day, 0.20f, 1.0f);
            return f;
          };

          const float crowd = crowdFactor();
          const std::uint32_t hPed = HashCoords32(x, y, m_gfxSeed32 ^ 0x0BADC0DEu);

          // --- Park pedestrians (front-edge "path") ---
          if (t.overlay == Overlay::Park && tileScreenW >= 60.0f && crowd > 0.05f) {
            int count = 0;
            if (Frac01(hPed) < 0.16f * crowd) count = 1;
            if (tileScreenW >= 84.0f && Frac01(hPed ^ 0xA17u) < 0.045f * crowd) count = 2;

            if (count > 0) {
              Vector2 corners[4];
              TileDiamondCorners(center, tileWf, tileHf, corners);

              for (int i = 0; i < count; ++i) {
                const std::uint32_t hi = HashCoords32(i, static_cast<int>(hPed), m_gfxSeed32 ^ 0xC0FFEEu);

                const PropSprite* ps = pickPedestrianSprite(hi, timeSec);
                if (!ps) break;

                // Place along the front edge (between BL and BR) and nudge inward.
                const float tEdge = 0.20f + 0.60f * Frac01(hi);
                Vector2 p = LerpV(corners[3], corners[2], tEdge);
                p = LerpV(p, center, 0.18f);
                p.x += (Frac01(hi ^ 0x123u) - 0.5f) * (tileWf * 0.04f);

                // Subtle bobbing to read as "alive". Keep amplitude stable in screen space.
                const float bobAmp = 1.1f * invZoom;
                const float bobFreq = 1.05f + 0.80f * Frac01(hi ^ 0xBEEF123u);
                const float bobPhase = Frac01(hi ^ 0xDEADBEEFu) * 6.2831853f;
                p.y += std::sin(timeSec * bobFreq + bobPhase) * bobAmp;

                Color tint = Mul(baseTint, 0.97f);
                tint.a = ClampU8(static_cast<int>(255.0f * std::clamp(0.35f + 0.65f * crowd, 0.0f, 1.0f)));
                drawProp(*ps, p, tint, 0.0f);

                if (wantPropEmissive && ps->emissive.id != 0) {
                  const unsigned char a = ClampU8(static_cast<int>(95.0f * std::clamp(dayNight.nightLights, 0.0f, 1.0f)));
                  queueEmissiveSprite(ps->emissive, ps->pivotX, ps->pivotY, p, a, sum, static_cast<float>(x) + 0.15f);
                }
              }
            }
          }

          // --- Road pedestrians (sidewalk activity near zones/parks) ---
          if (t.overlay == Overlay::Road && t.terrain != Terrain::Water && tileScreenW >= 56.0f && crowd > 0.05f) {
            // Find which sides border zones/parks.
            std::uint8_t nearMask = 0u;
            auto consider = [&](int nx, int ny, std::uint8_t bit) {
              if (nx < 0 || ny < 0 || nx >= mapW || ny >= mapH) return;
              const Overlay ov = world.at(nx, ny).overlay;
              if (ov == Overlay::Residential || ov == Overlay::Commercial || ov == Overlay::Industrial || ov == Overlay::Park) {
                nearMask |= bit;
              }
            };
            consider(x, y - 1, 0x01u);
            consider(x + 1, y, 0x02u);
            consider(x, y + 1, 0x04u);
            consider(x - 1, y, 0x08u);

            const int nearCount = popCount4(nearMask);
            if (nearCount > 0) {
              // Weight probability by the chosen neighbor's occupancy/capacity when it's a zone.
              int pick = static_cast<int>((hPed >> 9u) % static_cast<std::uint32_t>(nearCount));
              int selNx = x;
              int selNy = y;
              int selEdge = 2; // default to south edge
              for (int e = 0; e < 4; ++e) {
                const std::uint8_t bit = static_cast<std::uint8_t>(1u << e);
                if ((nearMask & bit) == 0u) continue;
                if (pick == 0) {
                  if (e == 0) { selNx = x; selNy = y - 1; selEdge = 0; }
                  if (e == 1) { selNx = x + 1; selNy = y; selEdge = 1; }
                  if (e == 2) { selNx = x; selNy = y + 1; selEdge = 2; }
                  if (e == 3) { selNx = x - 1; selNy = y; selEdge = 3; }
                  break;
                }
                --pick;
              }

              const Tile& nt = world.at(selNx, selNy);
              float activity = 0.45f;
              if (nt.overlay == Overlay::Commercial) activity = 0.70f;
              else if (nt.overlay == Overlay::Residential) activity = 0.58f;
              else if (nt.overlay == Overlay::Industrial) activity = 0.30f;
              else if (nt.overlay == Overlay::Park) activity = 0.62f;

              if (IsZoneOverlay(nt.overlay)) {
                const int cap = std::max(1, CapacityForTile(nt));
                const float occN = std::clamp(static_cast<float>(nt.occupants) / static_cast<float>(cap), 0.0f, 1.0f);
                activity *= (0.55f + 0.75f * occN);
              }

              const float pSpawn = std::clamp(0.06f + 0.22f * activity, 0.0f, 0.30f) * crowd;
              if (Frac01(hPed ^ 0xD00Du) < pSpawn) {
                const std::uint32_t hi = HashCoords32(selNx, selNy, hPed ^ 0xFACEFEEDu);
                const PropSprite* ps = pickPedestrianSprite(hi, timeSec);
                if (ps) {
                  Vector2 corners[4];
                  TileDiamondCorners(center, tileWf, tileHf, corners);
                  const Vector2 edgeA = corners[selEdge];
                  const Vector2 edgeB = corners[(selEdge + 1) & 3];

                  const float tEdge = 0.28f + 0.44f * Frac01(hi);
                  Vector2 p = LerpV(edgeA, edgeB, tEdge);
                  p = LerpV(p, center, 0.16f);
                  p.x += (Frac01(hi ^ 0x777u) - 0.5f) * (tileWf * 0.03f);

                  const float bobAmp = 0.95f * invZoom;
                  const float bobFreq = 1.25f + 0.95f * Frac01(hi ^ 0xB1B2B3B4u);
                  const float bobPhase = Frac01(hi ^ 0xC0FFEEu) * 6.2831853f;
                  p.y += std::sin(timeSec * bobFreq + bobPhase) * bobAmp;

                  Color tint = Mul(baseTint, 0.98f);
                  tint.a = ClampU8(static_cast<int>(255.0f * std::clamp(0.30f + 0.70f * crowd, 0.0f, 1.0f)));
                  drawProp(*ps, p, tint, 0.0f);

                  if (wantPropEmissive && ps->emissive.id != 0) {
                    const unsigned char a = ClampU8(static_cast<int>(85.0f * std::clamp(dayNight.nightLights, 0.0f, 1.0f)));
                    queueEmissiveSprite(ps->emissive, ps->pivotX, ps->pivotY, p, a, sum, static_cast<float>(x) + 0.20f);
                  }
                }
              }
            }
          }
        }
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
                  if (!drawZoneBuildingSprite(t, x, y, sum, center, brightness)) {
                    DrawZoneBuilding(t, tileWf, tileHf, camera.zoom, center, brightness);
                  }
                  DrawZoneTileIndicators(t, tileWf, tileHf, camera.zoom, center);
                }
              }
            } else {
              // Park or non-parcelized overlay
              if (!drawZoneBuildingSprite(t, x, y, sum, center, brightness)) {
                DrawZoneBuilding(t, tileWf, tileHf, camera.zoom, center, brightness);
              }
              DrawZoneTileIndicators(t, tileWf, tileHf, camera.zoom, center);
            }
          } else {
            if (!drawZoneBuildingSprite(t, x, y, sum, center, brightness)) {
              DrawZoneBuilding(t, tileWf, tileHf, camera.zoom, center, brightness);
            }
            DrawZoneTileIndicators(t, tileWf, tileHf, camera.zoom, center);
          }
        }
      }

      // -----------------------------
      // Road indicators (upgrade pips, etc.)
      // -----------------------------
      if (layerStructures && t.overlay == Overlay::Road && tileScreenW >= 18.0f) {
        DrawRoadIndicators(world, x, y, t, tileWf, tileHf, camera.zoom, center, brightness, dayNight, weather, m_gfxSeed32, timeSec);
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
  //
  // Note: cloud shadows can optionally persist in Clear weather (controlled by
  // CloudShadowSettings::clearAmount) so we can have drifting ambience without forcing rain/snow.
  if (drawAestheticDetails && m_cloudShadows.enabled && m_cloudShadowTex.id != 0) {
    const float cloudiness = (m_weather.mode == WeatherSettings::Mode::Clear)
                                 ? std::clamp(m_cloudShadows.clearAmount, 0.0f, 1.0f)
                                 : std::clamp(m_weather.overcast, 0.0f, 1.0f);
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

        const Rectangle srcRect{
            (dstX + offX) * texPerWorld,
            (dstY + offY) * texPerWorld,
            dstW * texPerWorld,
            dstH * texPerWorld};
        const Rectangle dst{dstX, dstY, dstW, dstH};

        const unsigned char a = static_cast<unsigned char>(std::round(255.0f * alpha));
        DrawTexturePro(m_cloudShadowTex, srcRect, dst, Vector2{0.0f, 0.0f}, 0.0f, Color{0, 0, 0, a});
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

  // -----------------------------
  // Volumetric clouds (visible overlay)
  // -----------------------------
  // Draw after grading so the clouds aren't double-tinted, but before emissive passes so
  // city lights remain crisp at night.
  if (drawAestheticDetails && m_volClouds.enabled) {
    const float cloudiness = (m_weather.mode == WeatherSettings::Mode::Clear)
                                 ? std::clamp(m_volClouds.clearAmount, 0.0f, 1.0f)
                                 : std::clamp(m_weather.overcast, 0.0f, 1.0f);
    if (cloudiness > 0.001f) {
      drawVolumetricCloudLayer(viewAABB,
                               tileWf,
                               timeSec,
                               dayNight.day,
                               dayNight.dusk,
                               cloudiness,
                               weather.windX,
                               weather.windY,
                               weather.windSpeed);
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
      /*suppressZoneWindows=*/drawZoneBuildingSprites,
      m_gfxSeed32);
  }

  // -----------------------------
  // Depth-sorted injected sprites (emissive)
  // -----------------------------
  // Draw after grading so emissive elements (e.g., headlights, streetlights) stay bright at night.
  if (!emissiveSprites.empty() || !emissivePropSprites.empty()) {
    if (!emissivePropSprites.empty()) {
      auto cmp = [](const WorldSprite& a, const WorldSprite& b) {
        if (a.sortSum != b.sortSum) return a.sortSum < b.sortSum;
        if (a.sortX != b.sortX) return a.sortX < b.sortX;
        return a.tex < b.tex;
      };
      std::sort(emissivePropSprites.begin(), emissivePropSprites.end(), cmp);
    }

    // Optional cheap bloom/halo around emissive sprites (purely procedural; no shaders).
    //
    // This helps small emissive textures (vehicle headlights, streetlights, building window masks)
    // read better without requiring external art.
    const bool bloomEnabled = drawAestheticDetails && m_dayNight.enabled && m_dayNight.drawLights &&
                              dayNight.nightLights > 0.01f && tileScreenW >= 42.0f;

    const float bloomBase = bloomEnabled ? std::clamp(dayNight.nightLights, 0.0f, 1.0f) : 0.0f;
    const float bloomWetBoost = 0.65f + 0.35f * std::clamp(weather.wetness, 0.0f, 1.0f);
    const float bloom = bloomBase * bloomWetBoost;

    const float invZoom = 1.0f / std::max(0.001f, camera.zoom);

    auto drawBloom = [&](const WorldSprite& sp) {
      if (!sp.tex || sp.tex->id == 0) return;
      if (sp.tint.a == 0) return;

      // Scale bloom by sprite alpha so dim lights don't explode.
      const float aN = static_cast<float>(sp.tint.a) / 255.0f;
      const float b = bloom * (0.35f + 0.65f * aN);
      if (b <= 0.02f) return;

      const float radPx = 1.2f + 3.4f * b;
      const float r1 = radPx * invZoom;
      const float r2 = (radPx * 1.85f) * invZoom;

      const unsigned char a1 = ClampU8(static_cast<int>(static_cast<float>(sp.tint.a) * (0.18f * b)));
      const unsigned char a2 = ClampU8(static_cast<int>(static_cast<float>(sp.tint.a) * (0.10f * b)));

      if (a1 == 0) return;

      Color c1 = sp.tint;
      c1.a = a1;
      Color c2 = sp.tint;
      c2.a = a2;

      // 8-tap halo.
      static const Vector2 dirs[8] = {
        Vector2{1.0f, 0.0f}, Vector2{-1.0f, 0.0f}, Vector2{0.0f, 1.0f}, Vector2{0.0f, -1.0f},
        Vector2{0.70710678f, 0.70710678f}, Vector2{0.70710678f, -0.70710678f},
        Vector2{-0.70710678f, 0.70710678f}, Vector2{-0.70710678f, -0.70710678f},
      };

      for (const Vector2& d : dirs) {
        Rectangle dst = sp.dst;
        dst.x += d.x * r1;
        dst.y += d.y * r1;
        DrawTexturePro(*sp.tex, sp.src, dst, sp.origin, sp.rotation, c1);
      }

      // Second, wider ring (kept only at high zoom to cap overdraw).
      if (a2 != 0 && tileScreenW >= 64.0f) {
        for (int i = 0; i < 4; ++i) {
          Rectangle dst = sp.dst;
          dst.x += dirs[i].x * r2;
          dst.y += dirs[i].y * r2;
          DrawTexturePro(*sp.tex, sp.src, dst, sp.origin, sp.rotation, c2);
        }
      }
    };

    BeginBlendMode(BLEND_ADDITIVE);

    if (bloomEnabled) {
      for (const WorldSprite* s : emissiveSprites) {
        if (s) drawBloom(*s);
      }
      for (const WorldSprite& s : emissivePropSprites) {
        drawBloom(s);
      }
    }

    // Crisp emissive sprites.
    for (const WorldSprite* s : emissiveSprites) {
      if (s) drawWorldSprite(*s);
    }
    for (const WorldSprite& s : emissivePropSprites) {
      drawWorldSprite(s);
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

    const auto drawPolylineRibbonFallback = [&](const std::vector<Vector2>& pts, Color base, const RibbonStyle& st) {
      if (pts.size() < 2) return;

      // In Mode2D, coordinates are in world-space pixels. To keep the ribbon
      // thickness stable in screen pixels we scale by 1/zoom.
      const float z = std::max(0.25f, camera.zoom);
      const float invZ = 1.0f / z;

      const auto withAlphaMul = [](Color c, float mul) -> Color {
        mul = std::clamp(mul, 0.0f, 1.0f);
        const float a = (static_cast<float>(c.a) / 255.0f) * mul;
        int ia = static_cast<int>(std::lround(a * 255.0f));
        ia = std::clamp(ia, 0, 255);
        c.a = static_cast<unsigned char>(ia);
        return c;
      };

      // --- Glow pass (additive) ---
      if (st.glowAlpha > 0.001f && st.glowThicknessPx > 0.01f) {
        BeginBlendMode(BLEND_ADDITIVE);
        const float thick = st.glowThicknessPx * invZ;
        const Color glow = withAlphaMul(base, st.glowAlpha);

        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
          DrawLineEx(pts[i], pts[i + 1], thick, glow);
        }

        EndBlendMode();
      }

      // --- Core pass (dashed) ---
      if (st.coreAlpha <= 0.001f || st.coreThicknessPx <= 0.01f) return;

      const float thick = st.coreThicknessPx * invZ;
      const float dashLen = std::max(2.0f, st.dashLengthPx) * invZ;
      const float dashSpeed = st.dashSpeedPx * invZ;
      const float duty = std::clamp(st.dashDuty, 0.0f, 1.0f);
      const float onLen = dashLen * duty;

      const Color coreOn = withAlphaMul(base, st.coreAlpha);
      const Color coreOff = withAlphaMul(base, st.coreAlpha * 0.25f);

      const float phase = (dashLen > 1e-3f) ? std::fmod(timeSec * dashSpeed, dashLen) : 0.0f;
      float pos = -phase;

      for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Vector2 a = pts[i];
        const Vector2 b = pts[i + 1];
        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float segLen = std::sqrt(dx * dx + dy * dy);
        if (segLen < 1e-3f) continue;

        const float invLen = 1.0f / segLen;
        const Vector2 dir = Vector2{dx * invLen, dy * invLen};

        Vector2 cur = a;
        float remaining = segLen;

        while (remaining > 1e-3f) {
          float mod = std::fmod(pos, dashLen);
          if (mod < 0.0f) mod += dashLen;

          const float periodRemain = dashLen - mod;
          const float step = std::min(remaining, periodRemain);
          const Vector2 next = Vector2{cur.x + dir.x * step, cur.y + dir.y * step};

          const bool on = (mod < onLen);
          DrawLineEx(cur, next, thick, on ? coreOn : coreOff);

          cur = next;
          remaining -= step;
          pos += step;
        }
      }
    };

    if (highlightPathEff && !highlightPathEff->empty()) {
      // Build a world-space polyline for the highlighted path.
      m_pathRibbonScratch.clear();
      m_pathRibbonScratch.reserve(highlightPathEff->size());

      for (const Point& p : *highlightPathEff) {
        if (!world.inBounds(p.x, p.y)) continue;
        const Tile& tt = world.at(p.x, p.y);
        const float elevPx = TileElevationPx(tt, m_elev);
        const Vector2 baseC = TileToWorldCenter(p.x, p.y, tileWf, tileHf);
        m_pathRibbonScratch.push_back(Vector2{baseC.x, baseC.y - elevPx});
      }

      if (m_pathRibbonScratch.size() >= 2) {
        RibbonStyle st;
        // Keep the ribbon thickness mostly stable in screen pixels, with a mild
        // boost when zoomed out so it remains readable.
        const float z = std::max(0.25f, camera.zoom);
        const float t = std::clamp((z - 0.55f) / (1.60f - 0.55f), 0.0f, 1.0f);
        st.coreThicknessPx = 9.0f - 4.0f * t; // ~9px @ zoomed-out, ~5px @ zoomed-in
        st.glowThicknessPx = st.coreThicknessPx * 2.6f;
        st.coreAlpha = 0.75f;
        st.glowAlpha = 0.18f;
        st.dashLengthPx = 32.0f;
        st.dashSpeedPx = 78.0f;
        st.dashDuty = 0.60f;
        st.flowStrength = 0.35f;

        // Prefer the GPU ribbon (geometry shader) for smoother, cheaper path highlights.
        // Fallback to a CPU-drawn dashed polyline if the backend can't compile geometry shaders.
        if (m_gpuRibbon.isReady()) {
          m_gpuRibbon.drawPath(m_pathRibbonScratch, screenW, screenH, timeSec, Color{255, 215, 0, 255}, st,
                               /*additiveBlend=*/true);
        } else {
          drawPolylineRibbonFallback(m_pathRibbonScratch, Color{255, 215, 0, 255}, st);
        }
      } else {
        // Degenerate path: fall back to tile outlines (keeps behavior consistent with old builds).
        for (const Point& p : *highlightPathEff) {
          drawOutline(p.x, p.y, Color{255, 215, 0, 110});
        }
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
  int panelH = (showHelp ? 470 : 228) + extraLines * 22;
  panelH = std::min(panelH, screenH - pad * 2);

  const float uiTime = static_cast<float>(GetTime());
  ui::DrawPanel(Rectangle{static_cast<float>(pad), static_cast<float>(pad), static_cast<float>(panelW),
                          static_cast<float>(panelH)},
                uiTime, /*active=*/true);

  const ui::Theme& uiTh = ui::GetTheme();

  int y = pad + 10;

  auto line = [&](std::string_view text, bool dim = false) {
    ui::Text(pad + 10, y, 18, text, dim ? uiTh.textDim : uiTh.text, /*bold=*/false, /*shadow=*/true, 1);
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
    ui::Text(pad + 10, y + 6, 16, buf, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 26;
  }

  if (heatmapInfo && heatmapInfo[0] != '\0') {
    ui::Text(pad + 10, y + 6, 16, heatmapInfo, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 26;
  }

  if (inspectInfo && inspectInfo[0] != '\0') {
    ui::Text(pad + 10, y + 6, 16, inspectInfo, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 26;
  }

    if (showHelp) {
      // Skinned help overlay (compact keycaps + wrapped tips).
      const float panelBottom = static_cast<float>(pad + panelH);
      const float helpX = static_cast<float>(pad + 10);
      const float helpY = static_cast<float>(y + 10);
      const float helpW = static_cast<float>(panelW - 20);
      const float helpH = std::max(0.0f, panelBottom - helpY - 10.0f);
  
      if (helpH > 40.0f) {
        const Rectangle helpR{helpX, helpY, helpW, helpH};
        ui::DrawPanelInset(helpR, uiTime, /*active=*/true);
  
        const int hx = static_cast<int>(helpR.x) + 10;
        int hy = static_cast<int>(helpR.y) + 8;
  
        ui::TextOutlined(hx, hy, 18, "HOTKEYS", uiTh.text, uiTh.accentDim, /*bold=*/true, /*shadow=*/true, 1);
        hy += 24;
  
        const int keySize = 14;
        const int rowStep = 24;
  
        const int colGap = 14;
        const int colW = std::max(120, static_cast<int>(helpR.width) / 2 - colGap);
        const int x0 = hx;
        const int x1 = hx + colW + colGap;
  
        auto hotkey = [&](int x, int yRow, std::string_view combo, std::string_view desc) {
          const int w = ui::DrawKeyCombo(x, yRow, combo, uiTime, /*strong=*/false, keySize);
          ui::Text(x + w + 8, yRow + 4, 14, desc, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
        };
  
        const int y0 = hy;
        hotkey(x0, y0 + 0 * rowStep, "RMB+Drag", "Pan camera");
        hotkey(x0, y0 + 1 * rowStep, "Wheel", "Zoom");
        hotkey(x0, y0 + 2 * rowStep, "R", "Regenerate");
        hotkey(x0, y0 + 3 * rowStep, "Space", "Pause/Resume");
        hotkey(x0, y0 + 4 * rowStep, "+/-", "Sim speed");
  
        hotkey(x1, y0 + 0 * rowStep, "1-5", "Zones");
        hotkey(x1, y0 + 1 * rowStep, "6-8", "Terraform");
        hotkey(x1, y0 + 2 * rowStep, "Q", "Inspect");
        hotkey(x1, y0 + 3 * rowStep, "Ctrl+Z", "Undo");
        hotkey(x1, y0 + 4 * rowStep, "Ctrl+Y", "Redo");
  
        const float tipY = static_cast<float>(y0 + 5 * rowStep + 6);
        const Rectangle tipR{helpR.x + 10.0f, tipY, helpR.width - 20.0f,
                             std::max(0.0f, helpR.y + helpR.height - tipY - 8.0f)};
  
        if (tipR.height > 12.0f) {
          ui::TextBox(
            tipR, 14,
            "More: F4 console | F5 save/menu | M minimap | L heatmap | F1 report | F2 cache | F3 model | Shift+F3 weather | F11 fullscreen. "
            "Tip: re-place a zone to upgrade. Road: U selects class (paint to upgrade), Shift+drag builds path. Terraform: Shift=strong, Ctrl=fine. "
            "District: Alt+click pick, Shift+click fill.",
            uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1, /*wrap=*/true, /*clip=*/true);
        }
      }
    }
  
  // Minimap overlay (bottom-right). One pixel per tile, scaled up.
  if (showMinimap) {
    ensureMinimapUpToDate(world);
    const MinimapLayout mini = minimapLayout(world, screenW, screenH);

    if (mini.rect.width > 2.0f && mini.rect.height > 2.0f && m_minimapTex.id != 0) {
      // Background + border.
      ui::DrawPanelInset(mini.rect, uiTime, /*active=*/true);

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
      ui::Text(static_cast<int>(mini.rect.x), labelY, 16, "Minimap (click/drag)", uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    }
  }

  // FPS
  const int fps = GetFPS();
  std::snprintf(buf, sizeof(buf), "FPS: %d", fps);
  ui::Text(screenW - 90, 12, 20, buf, uiTh.text, /*bold=*/true, /*shadow=*/true, 1);
}

} // namespace isocity
