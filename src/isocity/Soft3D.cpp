#include "isocity/Soft3D.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace isocity {

namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float DegToRad(float deg) { return deg * (kPi / 180.0f); }

inline float ClampF(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec4 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 1.0f;
};

inline Vec3 V3(float x, float y, float z) { return Vec3{x, y, z}; }

inline Vec3 Add(const Vec3& a, const Vec3& b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 Sub(const Vec3& a, const Vec3& b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 Mul(const Vec3& a, float s) { return Vec3{a.x * s, a.y * s, a.z * s}; }

inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
  return Vec3{a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x};
}

inline float Len(const Vec3& v) { return std::sqrt(std::max(0.0f, Dot(v, v))); }

inline Vec3 Normalize(const Vec3& v)
{
  const float l = Len(v);
  if (!(l > 1e-8f)) return Vec3{0.0f, 1.0f, 0.0f};
  return Vec3{v.x / l, v.y / l, v.z / l};
}

// Row-major 4x4 matrix.
struct Mat4 {
  float m[16] = {0};
};

inline Mat4 Identity()
{
  Mat4 r{};
  r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
  return r;
}

inline Mat4 Mul(const Mat4& a, const Mat4& b)
{
  Mat4 r{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      float s = 0.0f;
      for (int k = 0; k < 4; ++k) {
        s += a.m[row * 4 + k] * b.m[k * 4 + col];
      }
      r.m[row * 4 + col] = s;
    }
  }
  return r;
}

inline Vec4 Mul(const Mat4& a, const Vec4& v)
{
  return Vec4{
      a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z + a.m[3] * v.w,
      a.m[4] * v.x + a.m[5] * v.y + a.m[6] * v.z + a.m[7] * v.w,
      a.m[8] * v.x + a.m[9] * v.y + a.m[10] * v.z + a.m[11] * v.w,
      a.m[12] * v.x + a.m[13] * v.y + a.m[14] * v.z + a.m[15] * v.w,
  };
}

inline Mat4 Translation(const Vec3& t)
{
  Mat4 r = Identity();
  r.m[3] = t.x;
  r.m[7] = t.y;
  r.m[11] = t.z;
  return r;
}

inline Mat4 RotationAxisAngle(const Vec3& axis, float angleRad)
{
  const Vec3 a = Normalize(axis);
  const float c = std::cos(angleRad);
  const float s = std::sin(angleRad);
  const float t = 1.0f - c;

  Mat4 r = Identity();
  r.m[0] = t * a.x * a.x + c;
  r.m[1] = t * a.x * a.y - s * a.z;
  r.m[2] = t * a.x * a.z + s * a.y;

  r.m[4] = t * a.x * a.y + s * a.z;
  r.m[5] = t * a.y * a.y + c;
  r.m[6] = t * a.y * a.z - s * a.x;

  r.m[8] = t * a.x * a.z - s * a.y;
  r.m[9] = t * a.y * a.z + s * a.x;
  r.m[10] = t * a.z * a.z + c;
  return r;
}

inline Mat4 LookAtRH(const Vec3& eye, const Vec3& target, const Vec3& up)
{
  const Vec3 f = Normalize(Sub(target, eye));
  const Vec3 s = Normalize(Cross(f, up));
  const Vec3 u = Cross(s, f);

  Mat4 r = Identity();

  // Row-major
  r.m[0] = s.x;
  r.m[1] = s.y;
  r.m[2] = s.z;
  r.m[3] = -Dot(s, eye);

  r.m[4] = u.x;
  r.m[5] = u.y;
  r.m[6] = u.z;
  r.m[7] = -Dot(u, eye);

  r.m[8] = -f.x;
  r.m[9] = -f.y;
  r.m[10] = -f.z;
  r.m[11] = Dot(f, eye);

  r.m[12] = 0.0f;
  r.m[13] = 0.0f;
  r.m[14] = 0.0f;
  r.m[15] = 1.0f;
  return r;
}

inline Mat4 PerspectiveRH_OpenGL(float fovYRad, float aspect, float nearZ, float farZ)
{
  const float f = 1.0f / std::tan(std::max(1e-6f, fovYRad) * 0.5f);
  Mat4 r{};
  r.m[0] = f / std::max(1e-6f, aspect);
  r.m[5] = f;
  r.m[10] = (farZ + nearZ) / (nearZ - farZ);
  r.m[11] = (2.0f * farZ * nearZ) / (nearZ - farZ);
  r.m[14] = -1.0f;
  return r;
}

inline Mat4 OrthoRH_OpenGL(float halfHeight, float aspect, float nearZ, float farZ)
{
  const float hh = std::max(1e-6f, halfHeight);
  const float hw = hh * std::max(1e-6f, aspect);

  Mat4 r = Identity();
  r.m[0] = 1.0f / hw;
  r.m[5] = 1.0f / hh;
  r.m[10] = -2.0f / (farZ - nearZ);
  r.m[11] = -(farZ + nearZ) / (farZ - nearZ);
  return r;
}

inline std::uint8_t ToU8(float f)
{
  const int v = static_cast<int>(std::lround(ClampF(f, 0.0f, 255.0f)));
  return static_cast<std::uint8_t>(std::clamp(v, 0, 255));
}

inline float Smoothstep(float e0, float e1, float x)
{
  if (!(e1 > e0)) return (x >= e0) ? 1.0f : 0.0f;
  const float t = ClampF((x - e0) / (e1 - e0), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// sRGB <-> linear helpers.
inline float SrgbToLinear01(float cs)
{
  cs = ClampF(cs, 0.0f, 1.0f);
  if (cs <= 0.04045f) return cs / 12.92f;
  return std::pow((cs + 0.055f) / 1.055f, 2.4f);
}

inline float LinearToSrgb01(float cl)
{
  cl = ClampF(cl, 0.0f, 1.0f);
  if (cl <= 0.0031308f) return cl * 12.92f;
  return 1.055f * std::pow(cl, 1.0f / 2.4f) - 0.055f;
}

inline const std::array<float, 256>& SrgbU8ToLinearLut()
{
  static const std::array<float, 256> lut = []() {
    std::array<float, 256> a{};
    for (int i = 0; i < 256; ++i) {
      const float cs = static_cast<float>(i) / 255.0f;
      a[static_cast<std::size_t>(i)] = SrgbToLinear01(cs);
    }
    return a;
  }();
  return lut;
}

inline float SrgbU8ToLinear01(std::uint8_t v) { return SrgbU8ToLinearLut()[v]; }

inline std::uint8_t Linear01ToSrgbU8(float v01)
{
  const float srgb01 = LinearToSrgb01(v01);
  return ToU8(srgb01 * 255.0f);
}

inline std::uint32_t Hash32(std::uint32_t x)
{
  // A small integer hash (finalizer-inspired mix). Good enough for dithering/jitter.
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

inline std::uint32_t HashPixel(std::uint32_t seed, int x, int y)
{
  const std::uint32_t ux = static_cast<std::uint32_t>(x);
  const std::uint32_t uy = static_cast<std::uint32_t>(y);
  return Hash32(seed ^ (ux * 0x9e3779b9u) ^ (uy * 0x85ebca6bu));
}

inline void Clear(PpmImage& img, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  img.rgb.assign(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u, 0);
  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
      img.rgb[i + 0] = r;
      img.rgb[i + 1] = g;
      img.rgb[i + 2] = b;
    }
  }
}

struct SVtx {
  float sx = 0.0f;
  float sy = 0.0f;
  float z01 = 1.0f;
};

inline float Edge(const SVtx& a, const SVtx& b, float px, float py)
{
  return (px - a.sx) * (b.sy - a.sy) - (py - a.sy) * (b.sx - a.sx);
}

inline void PutPixel(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
  img.rgb[i + 0] = r;
  img.rgb[i + 1] = g;
  img.rgb[i + 2] = b;
}

inline void DrawLineZTest(PpmImage& img, std::vector<float>& zbuf,
                          const SVtx& a, const SVtx& b,
                          std::uint8_t r, std::uint8_t g, std::uint8_t bb,
                          float alpha,
                          float depthEps)
{
  const float dx = b.sx - a.sx;
  const float dy = b.sy - a.sy;
  const float adx = std::fabs(dx);
  const float ady = std::fabs(dy);
  const int steps = static_cast<int>(std::ceil(std::max(adx, ady)));
  if (steps <= 0) return;

  const float a01 = ClampF(alpha, 0.0f, 1.0f);

  for (int i = 0; i <= steps; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(steps);
    const float xf = a.sx + dx * t;
    const float yf = a.sy + dy * t;
    const float zf = a.z01 + (b.z01 - a.z01) * t;
    const int x = static_cast<int>(std::lround(xf));
    const int y = static_cast<int>(std::lround(yf));
    if (x < 0 || y < 0 || x >= img.width || y >= img.height) continue;
    const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x);
    if (zf <= zbuf[idx] + depthEps) {
      if (a01 >= 0.999f) {
        PutPixel(img, x, y, r, g, bb);
      } else if (a01 > 0.001f) {
        const std::size_t i3 = idx * 3u;
        const float inv = 1.0f - a01;
        img.rgb[i3 + 0] = ToU8(static_cast<float>(img.rgb[i3 + 0]) * inv + static_cast<float>(r) * a01);
        img.rgb[i3 + 1] = ToU8(static_cast<float>(img.rgb[i3 + 1]) * inv + static_cast<float>(g) * a01);
        img.rgb[i3 + 2] = ToU8(static_cast<float>(img.rgb[i3 + 2]) * inv + static_cast<float>(bb) * a01);
      }
    }
  }
}

inline void DownsampleBox(const PpmImage& src, int ssaa, PpmImage& dst)
{
  dst.width = src.width / ssaa;
  dst.height = src.height / ssaa;
  dst.rgb.assign(static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 3u, 0);

  for (int y = 0; y < dst.height; ++y) {
    for (int x = 0; x < dst.width; ++x) {
      std::uint32_t accR = 0, accG = 0, accB = 0;
      for (int yy = 0; yy < ssaa; ++yy) {
        for (int xx = 0; xx < ssaa; ++xx) {
          const int sx = x * ssaa + xx;
          const int sy = y * ssaa + yy;
          const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(sx)) * 3u;
          accR += src.rgb[si + 0];
          accG += src.rgb[si + 1];
          accB += src.rgb[si + 2];
        }
      }
      const std::uint32_t denom = static_cast<std::uint32_t>(ssaa * ssaa);
      const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(x)) * 3u;
      dst.rgb[di + 0] = static_cast<std::uint8_t>(accR / denom);
      dst.rgb[di + 1] = static_cast<std::uint8_t>(accG / denom);
      dst.rgb[di + 2] = static_cast<std::uint8_t>(accB / denom);
    }
  }
}

inline void DownsampleBoxGamma(const PpmImage& src, int ssaa, PpmImage& dst)
{
  dst.width = src.width / ssaa;
  dst.height = src.height / ssaa;
  dst.rgb.assign(static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 3u, 0);

  const auto& lut = SrgbU8ToLinearLut();
  const float invDenom = 1.0f / static_cast<float>(ssaa * ssaa);

  for (int y = 0; y < dst.height; ++y) {
    for (int x = 0; x < dst.width; ++x) {
      float accR = 0.0f;
      float accG = 0.0f;
      float accB = 0.0f;
      for (int yy = 0; yy < ssaa; ++yy) {
        for (int xx = 0; xx < ssaa; ++xx) {
          const int sx = x * ssaa + xx;
          const int sy = y * ssaa + yy;
          const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(sx)) * 3u;
          accR += lut[src.rgb[si + 0]];
          accG += lut[src.rgb[si + 1]];
          accB += lut[src.rgb[si + 2]];
        }
      }

      const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(x)) * 3u;
      dst.rgb[di + 0] = Linear01ToSrgbU8(accR * invDenom);
      dst.rgb[di + 1] = Linear01ToSrgbU8(accG * invDenom);
      dst.rgb[di + 2] = Linear01ToSrgbU8(accB * invDenom);
    }
  }
}

inline void DownsampleDepthMin(const std::vector<float>& src, int srcW, int srcH, int ssaa,
                               std::vector<float>& dst)
{
  const int dstW = srcW / ssaa;
  const int dstH = srcH / ssaa;
  dst.assign(static_cast<std::size_t>(dstW) * static_cast<std::size_t>(dstH), 1.0f);

  for (int y = 0; y < dstH; ++y) {
    for (int x = 0; x < dstW; ++x) {
      float m = 1.0f;
      for (int yy = 0; yy < ssaa; ++yy) {
        for (int xx = 0; xx < ssaa; ++xx) {
          const int sx = x * ssaa + xx;
          const int sy = y * ssaa + yy;
          const float d = src[static_cast<std::size_t>(sy) * static_cast<std::size_t>(srcW) + static_cast<std::size_t>(sx)];
          m = std::min(m, d);
        }
      }
      dst[static_cast<std::size_t>(y) * static_cast<std::size_t>(dstW) + static_cast<std::size_t>(x)] = m;
    }
  }
}

// ACES filmic curve (fitted) popularized by Krzysztof Narkowicz.
inline float TonemapAcesFitted(float x)
{
  x = std::max(0.0f, x);
  constexpr float a = 2.51f;
  constexpr float b = 0.03f;
  constexpr float c = 2.43f;
  constexpr float d = 0.59f;
  constexpr float e = 0.14f;
  const float y = (x * (a * x + b)) / (x * (c * x + d) + e);
  return ClampF(y, 0.0f, 1.0f);
}

inline void Blur3TapSeparable(const std::vector<float>& src, int w, int h, std::vector<float>& dst)
{
  // Gaussian-ish weights: [1 2 1] / 4.
  std::vector<float> tmp(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0.0f);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int xm = std::max(0, x - 1);
      const int xp = std::min(w - 1, x + 1);
      const std::size_t i0 = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(xm);
      const std::size_t i1 = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const std::size_t i2 = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(xp);
      tmp[i1] = (src[i0] + 2.0f * src[i1] + src[i2]) * 0.25f;
    }
  }

  dst.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0.0f);
  for (int y = 0; y < h; ++y) {
    const int ym = std::max(0, y - 1);
    const int yp = std::min(h - 1, y + 1);
    for (int x = 0; x < w; ++x) {
      const std::size_t i0 = static_cast<std::size_t>(ym) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const std::size_t i1 = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const std::size_t i2 = static_cast<std::size_t>(yp) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      dst[i1] = (tmp[i0] + 2.0f * tmp[i1] + tmp[i2]) * 0.25f;
    }
  }
}

inline void DilationMaxSeparable(std::vector<float>& img, int w, int h, int radius)
{
  if (radius <= 0) return;
  radius = std::min(radius, 32);

  std::vector<float> tmp(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0.0f);
  // Horizontal max.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      float m = 0.0f;
      const int x0 = std::max(0, x - radius);
      const int x1 = std::min(w - 1, x + radius);
      const std::size_t base = static_cast<std::size_t>(y) * static_cast<std::size_t>(w);
      for (int xx = x0; xx <= x1; ++xx) {
        m = std::max(m, img[base + static_cast<std::size_t>(xx)]);
      }
      tmp[base + static_cast<std::size_t>(x)] = m;
    }
  }

  // Vertical max.
  for (int y = 0; y < h; ++y) {
    const int y0 = std::max(0, y - radius);
    const int y1 = std::min(h - 1, y + radius);
    for (int x = 0; x < w; ++x) {
      float m = 0.0f;
      for (int yy = y0; yy <= y1; ++yy) {
        m = std::max(m, tmp[static_cast<std::size_t>(yy) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]);
      }
      img[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = m;
    }
  }
}

inline void ApplyPostFx(PpmImage& img, const std::vector<float>& depth, const Soft3DPostFxConfig& fx)
{
  const int w = img.width;
  const int h = img.height;
  if (w <= 0 || h <= 0) return;
  const std::size_t nPix = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (img.rgb.size() != nPix * 3u) return;
  if (depth.size() != nPix) return;

  const bool needLin = fx.enableAO || fx.enableEdge || fx.enableTonemap || fx.enableBloom;
  const bool needDither = fx.enableDither;
  if (!needLin && !needDither) return;

  // Convert to linear [0..1] if needed.
  std::vector<float> lin;
  if (needLin) {
    lin.assign(nPix * 3u, 0.0f);
    const auto& lut = SrgbU8ToLinearLut();
    for (std::size_t i = 0; i < nPix; ++i) {
      lin[i * 3u + 0] = lut[img.rgb[i * 3u + 0]];
      lin[i * 3u + 1] = lut[img.rgb[i * 3u + 1]];
      lin[i * 3u + 2] = lut[img.rgb[i * 3u + 2]];
    }
  }

  // --- AO pass ---
  if (fx.enableAO && needLin) {
    const int radius = std::max(1, fx.aoRadiusPx);
    const int samples = std::clamp(fx.aoSamples, 4, 32);
    const float range = std::max(1e-6f, fx.aoRange);
    const float bias = std::max(0.0f, fx.aoBias);
    const float strength = ClampF(fx.aoStrength, 0.0f, 1.0f);
    const float power = std::max(0.01f, fx.aoPower);

    struct AOSample {
      int dx = 0;
      int dy = 0;
      float dist = 1.0f;
    };
    std::vector<AOSample> kernel;
    kernel.reserve(static_cast<std::size_t>(samples));

    // Golden-angle spiral (precomputed per render), rounded to pixel taps.
    constexpr float kGolden = 2.39996322972865332f;
    for (int i = 0; i < samples; ++i) {
      const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(samples);
      const float ang = static_cast<float>(i) * kGolden;
      const float r = std::sqrt(t) * static_cast<float>(radius);
      const int dx = static_cast<int>(std::lround(std::cos(ang) * r));
      const int dy = static_cast<int>(std::lround(std::sin(ang) * r));
      if (dx == 0 && dy == 0) continue;
      const float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
      kernel.push_back(AOSample{dx, dy, dist});
    }
    if (kernel.size() < 4u) {
      kernel.clear();
      kernel.push_back(AOSample{radius, 0, static_cast<float>(radius)});
      kernel.push_back(AOSample{-radius, 0, static_cast<float>(radius)});
      kernel.push_back(AOSample{0, radius, static_cast<float>(radius)});
      kernel.push_back(AOSample{0, -radius, static_cast<float>(radius)});
    }

    std::vector<float> occ(nPix, 0.0f);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const float d0 = depth[idx];
        if (!(d0 < 0.9999f)) {
          occ[idx] = 0.0f;
          continue;
        }

        const std::uint32_t hpx = HashPixel(fx.postSeed, x, y);
        const int start = static_cast<int>(hpx % static_cast<std::uint32_t>(kernel.size()));

        float sum = 0.0f;
        int count = 0;
        for (std::size_t s = 0; s < kernel.size(); ++s) {
          const AOSample& k = kernel[(static_cast<std::size_t>(start) + s) % kernel.size()];
          const int sx = x + k.dx;
          const int sy = y + k.dy;
          if (sx < 0 || sy < 0 || sx >= w || sy >= h) continue;
          const std::size_t si = static_cast<std::size_t>(sy) * static_cast<std::size_t>(w) + static_cast<std::size_t>(sx);
          const float d1 = depth[si];
          if (!(d1 < 0.9999f)) continue;
          const float dd = (d0 - d1) - bias;
          if (!(dd > 0.0f)) continue;
          if (dd >= range) continue;
          const float wDepth = 1.0f - (dd / range);
          const float wDist = 1.0f - ClampF(k.dist / static_cast<float>(radius), 0.0f, 1.0f);
          sum += wDepth * wDist;
          ++count;
        }

        float o = (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
        o = std::pow(ClampF(o, 0.0f, 1.0f), power);
        occ[idx] = o;
      }
    }

    if (fx.aoBlurRadiusPx > 0) {
      std::vector<float> blurred;
      Blur3TapSeparable(occ, w, h, blurred);
      occ.swap(blurred);
    }

    for (std::size_t i = 0; i < nPix; ++i) {
      const float o = ClampF(occ[i], 0.0f, 1.0f);
      const float mul = ClampF(1.0f - strength * o, 0.0f, 1.0f);
      lin[i * 3u + 0] *= mul;
      lin[i * 3u + 1] *= mul;
      lin[i * 3u + 2] *= mul;
    }
  }

  // --- Tonemap / grade ---
  if (fx.enableTonemap && needLin) {
    const float exposure = std::max(0.0f, fx.exposure);
    const float contrast = std::max(0.0f, fx.contrast);
    const float sat = std::max(0.0f, fx.saturation);
    const float vignette = ClampF(fx.vignette, 0.0f, 1.0f);

    for (int y = 0; y < h; ++y) {
      const float ny = (static_cast<float>(y) + 0.5f) / static_cast<float>(h) * 2.0f - 1.0f;
      for (int x = 0; x < w; ++x) {
        const float nx = (static_cast<float>(x) + 0.5f) / static_cast<float>(w) * 2.0f - 1.0f;
        const float d2 = nx * nx + ny * ny;
        const float vig = 1.0f - vignette * Smoothstep(0.35f, 1.25f, d2);

        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        float r = lin[i * 3u + 0] * exposure;
        float g = lin[i * 3u + 1] * exposure;
        float b = lin[i * 3u + 2] * exposure;

        // Filmic tonemap.
        r = TonemapAcesFitted(r);
        g = TonemapAcesFitted(g);
        b = TonemapAcesFitted(b);

        // Contrast around 0.5.
        r = ClampF((r - 0.5f) * contrast + 0.5f, 0.0f, 1.0f);
        g = ClampF((g - 0.5f) * contrast + 0.5f, 0.0f, 1.0f);
        b = ClampF((b - 0.5f) * contrast + 0.5f, 0.0f, 1.0f);

        // Saturation.
        const float l = r * 0.2126f + g * 0.7152f + b * 0.0722f;
        r = ClampF(l + (r - l) * sat, 0.0f, 1.0f);
        g = ClampF(l + (g - l) * sat, 0.0f, 1.0f);
        b = ClampF(l + (b - l) * sat, 0.0f, 1.0f);

        r *= vig;
        g *= vig;
        b *= vig;

        lin[i * 3u + 0] = r;
        lin[i * 3u + 1] = g;
        lin[i * 3u + 2] = b;
      }
    }
  }



  // --- Bloom (bright-pass + blur) ---
  if (needLin && fx.enableBloom)
  {
    const float strength = std::max(0.0f, fx.bloomStrength);
    const float threshold = ClampF(fx.bloomThreshold, 0.0f, 1.0f);
    const float radius = std::max(0.0f, fx.bloomRadius);

    if (strength > 1e-6f && radius > 1e-6f)
    {
      const int nPix = w * h;
      std::vector<float> bloom(3 * nPix, 0.0f);
      std::vector<float> tmp(3 * nPix, 0.0f);

      const float invRange = 1.0f / std::max(1e-6f, 1.0f - threshold);

      // Bright pass.
      for (int i = 0; i < nPix; ++i)
      {
        const float r = lin[i * 3 + 0];
        const float g = lin[i * 3 + 1];
        const float b = lin[i * 3 + 2];
        const float m = std::max(r, std::max(g, b));
        if (m <= threshold)
        {
          continue;
        }

        // Smooth ramp from threshold..1.
        const float u = ClampF((m - threshold) * invRange, 0.0f, 1.0f);
        const float k = Smoothstep(0.0f, 1.0f, u);

        bloom[i * 3 + 0] = r * k;
        bloom[i * 3 + 1] = g * k;
        bloom[i * 3 + 2] = b * k;
      }

      const float rNorm = ClampF(radius, 0.0f, 2.0f);
      const int passes = std::clamp(static_cast<int>(std::round(rNorm * 6.0f)), 1, 12);

      auto blurH = [&](const std::vector<float>& src, std::vector<float>& dst) {
        dst.resize(src.size());
        for (int y = 0; y < h; ++y)
        {
          for (int x = 0; x < w; ++x)
          {
            const int xm = std::max(0, x - 1);
            const int xp = std::min(w - 1, x + 1);
            const size_t i = static_cast<size_t>((y * w + x) * 3);
            const size_t im = static_cast<size_t>((y * w + xm) * 3);
            const size_t ip = static_cast<size_t>((y * w + xp) * 3);

            dst[i + 0] = (src[im + 0] + 2.0f * src[i + 0] + src[ip + 0]) * 0.25f;
            dst[i + 1] = (src[im + 1] + 2.0f * src[i + 1] + src[ip + 1]) * 0.25f;
            dst[i + 2] = (src[im + 2] + 2.0f * src[i + 2] + src[ip + 2]) * 0.25f;
          }
        }
      };

      auto blurV = [&](const std::vector<float>& src, std::vector<float>& dst) {
        dst.resize(src.size());
        for (int y = 0; y < h; ++y)
        {
          const int ym = std::max(0, y - 1);
          const int yp = std::min(h - 1, y + 1);
          for (int x = 0; x < w; ++x)
          {
            const size_t i = static_cast<size_t>((y * w + x) * 3);
            const size_t im = static_cast<size_t>((ym * w + x) * 3);
            const size_t ip = static_cast<size_t>((yp * w + x) * 3);

            dst[i + 0] = (src[im + 0] + 2.0f * src[i + 0] + src[ip + 0]) * 0.25f;
            dst[i + 1] = (src[im + 1] + 2.0f * src[i + 1] + src[ip + 1]) * 0.25f;
            dst[i + 2] = (src[im + 2] + 2.0f * src[i + 2] + src[ip + 2]) * 0.25f;
          }
        }
      };

      for (int p = 0; p < passes; ++p)
      {
        blurH(bloom, tmp);
        blurV(tmp, bloom);
      }

      // Additive blend back into the main buffer.
      for (int i = 0; i < nPix; ++i)
      {
        lin[i * 3 + 0] = ClampF(lin[i * 3 + 0] + strength * bloom[i * 3 + 0], 0.0f, 1.0f);
        lin[i * 3 + 1] = ClampF(lin[i * 3 + 1] + strength * bloom[i * 3 + 1], 0.0f, 1.0f);
        lin[i * 3 + 2] = ClampF(lin[i * 3 + 2] + strength * bloom[i * 3 + 2], 0.0f, 1.0f);
      }
    }
  }
  // --- Edge outlines (depth discontinuity) ---
  if (fx.enableEdge && needLin) {
    const float thr = ClampF(fx.edgeThreshold, 0.0f, 1.0f);
    const float soft = std::max(1e-6f, fx.edgeSoftness);
    const float alpha = ClampF(fx.edgeAlpha, 0.0f, 1.0f);

    std::vector<float> edge(nPix, 0.0f);
    for (int y = 0; y < h; ++y) {
      const int ym = std::max(0, y - 1);
      const int yp = std::min(h - 1, y + 1);
      for (int x = 0; x < w; ++x) {
        const int xm = std::max(0, x - 1);
        const int xp = std::min(w - 1, x + 1);

        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const float d = depth[i];
        float md = 0.0f;

        auto upd = [&](int xx, int yy) {
          const std::size_t j = static_cast<std::size_t>(yy) * static_cast<std::size_t>(w) + static_cast<std::size_t>(xx);
          md = std::max(md, std::fabs(d - depth[j]));
        };

        upd(xm, y);
        upd(xp, y);
        upd(x, ym);
        upd(x, yp);
        // Diagonals help for thin silhouettes.
        upd(xm, ym);
        upd(xp, ym);
        upd(xm, yp);
        upd(xp, yp);

        edge[i] = Smoothstep(thr, thr + soft, md);
      }
    }

    // EdgeRadiusPx is "thickness"; radius==1 means no extra dilation.
    if (fx.edgeRadiusPx > 1) {
      DilationMaxSeparable(edge, w, h, fx.edgeRadiusPx - 1);
    }

    const auto& lut = SrgbU8ToLinearLut();
    const float er = lut[fx.edgeR];
    const float eg = lut[fx.edgeG];
    const float eb = lut[fx.edgeB];

    for (std::size_t i = 0; i < nPix; ++i) {
      const float a = alpha * ClampF(edge[i], 0.0f, 1.0f);
      if (!(a > 1e-6f)) continue;
      const float inv = 1.0f - a;
      lin[i * 3u + 0] = lin[i * 3u + 0] * inv + er * a;
      lin[i * 3u + 1] = lin[i * 3u + 1] * inv + eg * a;
      lin[i * 3u + 2] = lin[i * 3u + 2] * inv + eb * a;
    }
  }

  // --- Output conversion ---
  if (needLin) {
    // Convert linear -> sRGB and (optionally) dither/quantize.
    const int bits = std::clamp(fx.ditherBits, 1, 8);
    const int levels = (1 << bits) - 1;
    const float invLevels = 1.0f / static_cast<float>(levels);
    const float dStr = ClampF(fx.ditherStrength, 0.0f, 1.0f);

    // 4x4 Bayer matrix values in [0..15].
    static constexpr int bayer4[4][4] = {
        {0, 8, 2, 10},
        {12, 4, 14, 6},
        {3, 11, 1, 9},
        {15, 7, 13, 5},
    };

    const int ox = static_cast<int>((fx.postSeed >> 0) & 3u);
    const int oy = static_cast<int>((fx.postSeed >> 2) & 3u);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        float r = lin[i * 3u + 0];
        float g = lin[i * 3u + 1];
        float b = lin[i * 3u + 2];

        float sr = LinearToSrgb01(r);
        float sg = LinearToSrgb01(g);
        float sb = LinearToSrgb01(b);

        if (fx.enableDither) {
          const int bv = bayer4[(y + oy) & 3][(x + ox) & 3];
          const float d = (static_cast<float>(bv) + 0.5f) / 16.0f - 0.5f; // [-0.5..0.5]
          const float delta = d * dStr * invLevels;
          sr += delta;
          sg += delta;
          sb += delta;

          sr = ClampF(std::round(ClampF(sr, 0.0f, 1.0f) * static_cast<float>(levels)) * invLevels, 0.0f, 1.0f);
          sg = ClampF(std::round(ClampF(sg, 0.0f, 1.0f) * static_cast<float>(levels)) * invLevels, 0.0f, 1.0f);
          sb = ClampF(std::round(ClampF(sb, 0.0f, 1.0f) * static_cast<float>(levels)) * invLevels, 0.0f, 1.0f);
        }

        const std::size_t o = i * 3u;
        img.rgb[o + 0] = ToU8(sr * 255.0f);
        img.rgb[o + 1] = ToU8(sg * 255.0f);
        img.rgb[o + 2] = ToU8(sb * 255.0f);
      }
    }
    return;
  }

  // Dither-only path (no linear conversion required).
  if (needDither) {
    const int bits = std::clamp(fx.ditherBits, 1, 8);
    const int levels = (1 << bits) - 1;
    const float invLevels = 1.0f / static_cast<float>(levels);
    const float dStr = ClampF(fx.ditherStrength, 0.0f, 1.0f);

    static constexpr int bayer4[4][4] = {
        {0, 8, 2, 10},
        {12, 4, 14, 6},
        {3, 11, 1, 9},
        {15, 7, 13, 5},
    };
    const int ox = static_cast<int>((fx.postSeed >> 0) & 3u);
    const int oy = static_cast<int>((fx.postSeed >> 2) & 3u);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const int bv = bayer4[(y + oy) & 3][(x + ox) & 3];
        const float d = (static_cast<float>(bv) + 0.5f) / 16.0f - 0.5f;
        const float delta = d * dStr * invLevels;

        for (int c = 0; c < 3; ++c) {
          const float v = static_cast<float>(img.rgb[i * 3u + static_cast<std::size_t>(c)]) / 255.0f;
          const float q = std::round(ClampF(v + delta, 0.0f, 1.0f) * static_cast<float>(levels)) * invLevels;
          img.rgb[i * 3u + static_cast<std::size_t>(c)] = ToU8(q * 255.0f);
        }
      }
    }
  }
}

} // namespace

PpmImage RenderQuadsSoft3D(const std::vector<MeshQuad>& quads,
                           Soft3DCamera cam,
                           const Soft3DShading& shade,
                           const Soft3DRenderConfig& cfg,
                           MeshV3* outBoundsMin,
                           MeshV3* outBoundsMax,
                           std::string* outError)
{
  if (outError) outError->clear();

  const int outW = cfg.width;
  const int outH = cfg.height;
  if (outW <= 0 || outH <= 0) {
    if (outError) *outError = "invalid render size";
    return {};
  }

  const int ssaa = std::max(1, cfg.supersample);
  const int w = outW * ssaa;
  const int h = outH * ssaa;

  PpmImage imgSS;
  imgSS.width = w;
  imgSS.height = h;
  Clear(imgSS, shade.bgR, shade.bgG, shade.bgB);

  std::vector<float> zbuf(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 1.0f);

  if (quads.empty()) {
    if (outError) *outError = "no geometry";
    PpmImage out;
    std::vector<float> depthOut;
    if (ssaa > 1) {
      if (cfg.postFx.gammaCorrectDownsample) {
        DownsampleBoxGamma(imgSS, ssaa, out);
      } else {
        DownsampleBox(imgSS, ssaa, out);
      }
      DownsampleDepthMin(zbuf, w, h, ssaa, depthOut);
    } else {
      out = std::move(imgSS);
      depthOut = std::move(zbuf);
    }

    ApplyPostFx(out, depthOut, cfg.postFx);
    return out;
  }

  // --- Bounds ---
  MeshV3 bmin{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
  MeshV3 bmax{-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
  for (const MeshQuad& q : quads) {
    const MeshV3 v[4] = {q.a, q.b, q.c, q.d};
    for (const MeshV3& p : v) {
      bmin.x = std::min(bmin.x, p.x);
      bmin.y = std::min(bmin.y, p.y);
      bmin.z = std::min(bmin.z, p.z);
      bmax.x = std::max(bmax.x, p.x);
      bmax.y = std::max(bmax.y, p.y);
      bmax.z = std::max(bmax.z, p.z);
    }
  }

  if (outBoundsMin) *outBoundsMin = bmin;
  if (outBoundsMax) *outBoundsMax = bmax;

  // --- Auto-fit camera ---
  const float aspect = static_cast<float>(w) / static_cast<float>(h);

  const Vec3 boundsMin = V3(bmin.x, bmin.y, bmin.z);
  const Vec3 boundsMax = V3(bmax.x, bmax.y, bmax.z);
  const Vec3 center = Mul(Add(boundsMin, boundsMax), 0.5f);
  const Vec3 ext = Mul(Sub(boundsMax, boundsMin), 0.5f);
  const float radius = Len(ext);

  if (cam.autoFit) {
    cam.targetX = center.x;
    cam.targetY = center.y;
    cam.targetZ = center.z;

    const float margin = ClampF(cam.fitMargin, 0.0f, 0.50f);
    const float rPad = radius * (1.0f + margin);

    if (cam.projection == Soft3DCamera::Projection::Perspective) {
      const float fovY = DegToRad(std::max(1.0f, cam.fovYDeg));
      const float fovX = 2.0f * std::atan(std::tan(fovY * 0.5f) * aspect);
      const float dY = (std::sin(fovY * 0.5f) > 1e-6f) ? (rPad / std::sin(fovY * 0.5f)) : (rPad * 3.0f);
      const float dX = (std::sin(fovX * 0.5f) > 1e-6f) ? (rPad / std::sin(fovX * 0.5f)) : (rPad * 3.0f);
      cam.distance = std::max(dY, dX);
      cam.nearZ = std::max(0.05f, cam.distance - rPad * 2.5f);
      cam.farZ = std::max(cam.nearZ + 10.0f, cam.distance + rPad * 3.5f);
    } else {
      // For ortho, we fit the rotated bounds into the view rectangle.
      // Compute a rough half-height by projecting the 8 AABB corners into view space (rotation only).

      const float yawR = DegToRad(cam.yawDeg);
      const float pitchR = DegToRad(cam.pitchDeg);

      // Camera offset direction from target.
      const Vec3 dir = Normalize(Vec3{
          std::cos(pitchR) * std::cos(yawR),
          std::sin(pitchR),
          std::cos(pitchR) * std::sin(yawR),
      });

      // Use a large-ish distance just to define a valid view matrix.
      const float tmpDist = std::max(10.0f, rPad * 4.0f);
      const Vec3 eye = Add(center, Mul(dir, tmpDist));
      Mat4 view = LookAtRH(eye, center, V3(0.0f, 1.0f, 0.0f));

      // Apply roll after look-at by rotating around forward axis.
      if (std::fabs(cam.rollDeg) > 1e-4f) {
        const float rollR = DegToRad(cam.rollDeg);
        // forward axis in view space is -Z; in world it's (target-eye) normalized.
        const Vec3 fwd = Normalize(Sub(center, eye));
        Mat4 rroll = RotationAxisAngle(fwd, rollR);
        // roll is a world rotation applied before view => view = view * rroll^{-1}
        // Approximate with post-multiplying by inverse roll (neg angle).
        Mat4 invRoll = RotationAxisAngle(fwd, -rollR);
        view = Mul(view, invRoll);
      }

      float minX = std::numeric_limits<float>::infinity();
      float minY = std::numeric_limits<float>::infinity();
      float minZ = std::numeric_limits<float>::infinity();
      float maxX = -std::numeric_limits<float>::infinity();
      float maxY = -std::numeric_limits<float>::infinity();
      float maxZ = -std::numeric_limits<float>::infinity();

      const Vec3 c[8] = {
          V3(boundsMin.x, boundsMin.y, boundsMin.z),
          V3(boundsMax.x, boundsMin.y, boundsMin.z),
          V3(boundsMin.x, boundsMax.y, boundsMin.z),
          V3(boundsMax.x, boundsMax.y, boundsMin.z),
          V3(boundsMin.x, boundsMin.y, boundsMax.z),
          V3(boundsMax.x, boundsMin.y, boundsMax.z),
          V3(boundsMin.x, boundsMax.y, boundsMax.z),
          V3(boundsMax.x, boundsMax.y, boundsMax.z),
      };

      for (const Vec3& p : c) {
        const Vec4 v = Mul(view, Vec4{p.x, p.y, p.z, 1.0f});
        minX = std::min(minX, v.x);
        minY = std::min(minY, v.y);
        minZ = std::min(minZ, v.z);
        maxX = std::max(maxX, v.x);
        maxY = std::max(maxY, v.y);
        maxZ = std::max(maxZ, v.z);
      }

      const float extentX = (maxX - minX) * 0.5f;
      const float extentY = (maxY - minY) * 0.5f;
      const float hhFit = std::max(extentY, extentX / std::max(1e-6f, aspect));
      cam.orthoHalfHeight = std::max(0.1f, hhFit * (1.0f + margin));

      // Depth range from view-space z (note: view z is negative in front for our LookAt).
      const float nearFit = std::max(0.05f, -maxZ * (1.0f - margin));
      const float farFit = std::max(nearFit + 10.0f, -minZ * (1.0f + margin));
      cam.nearZ = nearFit;
      cam.farZ = farFit;

      // Also pick a distance that avoids clipping.
      cam.distance = tmpDist;
    }
  }

  // --- Build view/projection ---
  const float yawR = DegToRad(cam.yawDeg);
  const float pitchR = DegToRad(cam.pitchDeg);

  const Vec3 camTarget = V3(cam.targetX, cam.targetY, cam.targetZ);
  const Vec3 offsetDir = Normalize(Vec3{
      std::cos(pitchR) * std::cos(yawR),
      std::sin(pitchR),
      std::cos(pitchR) * std::sin(yawR),
  });
  const Vec3 eye = Add(camTarget, Mul(offsetDir, std::max(0.01f, cam.distance)));

  Mat4 view = LookAtRH(eye, camTarget, V3(0.0f, 1.0f, 0.0f));
  if (std::fabs(cam.rollDeg) > 1e-4f) {
    const float rollR = DegToRad(cam.rollDeg);
    const Vec3 fwd = Normalize(Sub(camTarget, eye));
    Mat4 invRoll = RotationAxisAngle(fwd, -rollR);
    view = Mul(view, invRoll);
  }

  Mat4 proj = Identity();
  if (cam.projection == Soft3DCamera::Projection::Perspective) {
    proj = PerspectiveRH_OpenGL(DegToRad(std::max(1.0f, cam.fovYDeg)), aspect,
                                std::max(0.01f, cam.nearZ), std::max(cam.nearZ + 0.1f, cam.farZ));
  } else {
    proj = OrthoRH_OpenGL(std::max(0.01f, cam.orthoHalfHeight), aspect,
                          std::max(0.01f, cam.nearZ), std::max(cam.nearZ + 0.1f, cam.farZ));
  }
  const Mat4 viewProj = Mul(proj, view);

  const Vec3 lightDir = Normalize(V3(shade.lightDirX, shade.lightDirY, shade.lightDirZ));
  const float amb = ClampF(shade.ambient, 0.0f, 2.0f);
  const float diff = ClampF(shade.diffuse, 0.0f, 2.0f);

  auto shadeColor = [&](const MeshC4& c, const MeshN3& n, float extraMul) -> MeshC4 {
    const Vec3 nn = Normalize(V3(n.x, n.y, n.z));
    const float ndl = std::max(0.0f, Dot(nn, lightDir));
    float m = amb + diff * ndl;
    m *= extraMul;
    m = ClampF(m, 0.0f, 1.35f);
    return MeshC4{
        ToU8(static_cast<float>(c.r) * m),
        ToU8(static_cast<float>(c.g) * m),
        ToU8(static_cast<float>(c.b) * m),
        c.a,
    };
  };

  auto fogBlend = [&](std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, float depth01) {
    if (!shade.enableFog) return;
    const float s0 = ClampF(shade.fogStart, 0.0f, 1.0f);
    const float s1 = ClampF(std::max(s0 + 1e-6f, shade.fogEnd), 0.0f, 1.0f);
    const float t = ClampF((depth01 - s0) / (s1 - s0), 0.0f, 1.0f);
    const float a = ClampF(shade.fogStrength, 0.0f, 1.0f) * t;
    r = ToU8(static_cast<float>(r) * (1.0f - a) + static_cast<float>(shade.fogR) * a);
    g = ToU8(static_cast<float>(g) * (1.0f - a) + static_cast<float>(shade.fogG) * a);
    b = ToU8(static_cast<float>(b) * (1.0f - a) + static_cast<float>(shade.fogB) * a);
  };

  auto project = [&](const MeshV3& p) -> Vec4 {
    return Mul(viewProj, Vec4{p.x, p.y, p.z, 1.0f});
  };

  auto toScreen = [&](const Vec4& clip) -> SVtx {
    if (!(std::fabs(clip.w) > 1e-9f)) {
      return SVtx{-1e9f, -1e9f, 1.0f};
    }
    const float invW = 1.0f / clip.w;
    const float ndcX = clip.x * invW;
    const float ndcY = clip.y * invW;
    const float ndcZ = clip.z * invW;

    const float sx = (ndcX * 0.5f + 0.5f) * static_cast<float>(w - 1);
    const float sy = (1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(h - 1);
    const float z01 = ClampF(ndcZ * 0.5f + 0.5f, 0.0f, 1.0f);
    return SVtx{sx, sy, z01};
  };

  // --- Rasterize ---
  for (const MeshQuad& q : quads) {
    // Decide whether to shade using the author-provided normal (q.n) or geometric
    // normals derived from the actual triangle plane.
    //
    // We keep q.n for perfectly-flat quads so callers can provide a smooth/"fake"
    // heightfield normal even when the emitted geometry is a flat tile.
    const float minY = std::min({q.a.y, q.b.y, q.c.y, q.d.y});
    const float maxY = std::max({q.a.y, q.b.y, q.c.y, q.d.y});
    const bool useGeomNormals = (maxY - minY) > 1e-6f;

    // Project quad vertices.
    const Vec4 ca = project(q.a);
    const Vec4 cb = project(q.b);
    const Vec4 cc = project(q.c);
    const Vec4 cd = project(q.d);

    // Skip quads with any vertex behind the camera (simple near-plane reject).
    if (ca.w <= 0.0f || cb.w <= 0.0f || cc.w <= 0.0f || cd.w <= 0.0f) {
      continue;
    }

    const SVtx sa = toScreen(ca);
    const SVtx sb = toScreen(cb);
    const SVtx sc = toScreen(cc);
    const SVtx sd = toScreen(cd);

    std::uint8_t tri0R = 0, tri0G = 0, tri0B = 0;
    std::uint8_t tri1R = 0, tri1G = 0, tri1B = 0;

    if (!useGeomNormals) {
      // Flat quad: use the caller's provided normal (preserves previous behavior).
      const MeshC4 shaded = shadeColor(q.color, q.n, 1.0f);
      tri0R = tri1R = shaded.r;
      tri0G = tri1G = shaded.g;
      tri0B = tri1B = shaded.b;
    } else {
      // Non-flat quad: shade each triangle with its geometric normal.
      const Vec3 refN = Normalize(V3(q.n.x, q.n.y, q.n.z));

      auto triNormal = [&](const MeshV3& p0, const MeshV3& p1, const MeshV3& p2) -> Vec3 {
        const Vec3 a = V3(p0.x, p0.y, p0.z);
        const Vec3 b = V3(p1.x, p1.y, p1.z);
        const Vec3 c = V3(p2.x, p2.y, p2.z);
        Vec3 n = Normalize(Cross(Sub(b, a), Sub(c, a)));
        if (Dot(n, refN) < 0.0f) n = Mul(n, -1.0f);
        return n;
      };

      const Vec3 n0 = triNormal(q.a, q.b, q.c);
      const Vec3 n1 = triNormal(q.a, q.c, q.d);

      const MeshC4 s0 = shadeColor(q.color, MeshN3{n0.x, n0.y, n0.z}, 1.0f);
      const MeshC4 s1 = shadeColor(q.color, MeshN3{n1.x, n1.y, n1.z}, 1.0f);
      tri0R = s0.r;
      tri0G = s0.g;
      tri0B = s0.b;
      tri1R = s1.r;
      tri1G = s1.g;
      tri1B = s1.b;
    }

    auto rasterTri = [&](SVtx v0, SVtx v1, SVtx v2,
                         std::uint8_t baseR, std::uint8_t baseG, std::uint8_t baseB) {
      // Compute signed area.
      const float area = (v1.sx - v0.sx) * (v2.sy - v0.sy) - (v1.sy - v0.sy) * (v2.sx - v0.sx);
      if (!(std::fabs(area) > 1e-6f)) return;

      // Enforce CCW winding.
      if (area < 0.0f) std::swap(v1, v2);
      const float invArea = 1.0f / std::fabs(area);

      const float minXf = std::floor(std::min({v0.sx, v1.sx, v2.sx}));
      const float maxXf = std::ceil(std::max({v0.sx, v1.sx, v2.sx}));
      const float minYf = std::floor(std::min({v0.sy, v1.sy, v2.sy}));
      const float maxYf = std::ceil(std::max({v0.sy, v1.sy, v2.sy}));

      int minX = static_cast<int>(minXf);
      int maxX = static_cast<int>(maxXf);
      int minY = static_cast<int>(minYf);
      int maxY = static_cast<int>(maxYf);
      minX = std::clamp(minX, 0, w - 1);
      maxX = std::clamp(maxX, 0, w - 1);
      minY = std::clamp(minY, 0, h - 1);
      maxY = std::clamp(maxY, 0, h - 1);

      for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
          const float px = static_cast<float>(x) + 0.5f;
          const float py = static_cast<float>(y) + 0.5f;

          const float w0 = Edge(v1, v2, px, py) * invArea;
          const float w1 = Edge(v2, v0, px, py) * invArea;
          const float w2 = Edge(v0, v1, px, py) * invArea;
          if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;

          const float z01 = w0 * v0.z01 + w1 * v1.z01 + w2 * v2.z01;
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
          if (z01 >= zbuf[idx]) continue;
          zbuf[idx] = z01;

          std::uint8_t r = baseR;
          std::uint8_t g = baseG;
          std::uint8_t b = baseB;
          fogBlend(r, g, b, z01);
          PutPixel(imgSS, x, y, r, g, b);
        }
      }
    };

    // Two triangles: (a,b,c) and (a,c,d)
    rasterTri(sa, sb, sc, tri0R, tri0G, tri0B);
    rasterTri(sa, sc, sd, tri1R, tri1G, tri1B);

    if (cfg.drawOutlines) {
      const SVtx e0 = sa;
      const SVtx e1 = sb;
      const SVtx e2 = sc;
      const SVtx e3 = sd;
      DrawLineZTest(imgSS, zbuf, e0, e1, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineAlpha, cfg.outlineDepthEps);
      DrawLineZTest(imgSS, zbuf, e1, e2, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineAlpha, cfg.outlineDepthEps);
      DrawLineZTest(imgSS, zbuf, e2, e3, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineAlpha, cfg.outlineDepthEps);
      DrawLineZTest(imgSS, zbuf, e3, e0, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineAlpha, cfg.outlineDepthEps);
    }
  }

  // Resolve SSAA, build a matching depth buffer for post, then apply post-fx.
  PpmImage out;
  std::vector<float> depthOut;
  if (ssaa > 1) {
    if (cfg.postFx.gammaCorrectDownsample) {
      DownsampleBoxGamma(imgSS, ssaa, out);
    } else {
      DownsampleBox(imgSS, ssaa, out);
    }
    DownsampleDepthMin(zbuf, w, h, ssaa, depthOut);
  } else {
    out = std::move(imgSS);
    depthOut = std::move(zbuf);
  }

  ApplyPostFx(out, depthOut, cfg.postFx);
  return out;
}

} // namespace isocity
