#pragma once

#include "isocity/Export.hpp"     // RgbaImage
#include "isocity/GfxPalette.hpp" // Rgba8

#include "isocity/DeterministicMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace isocity {
namespace gfx {

// -----------------------------------------------------------------------------------------------
// Tiny raylib-free 2D raster helper used by procedural generation.
//
// ProcIsoCity has a few independent generators (tiles, props, buildings) that need the
// same small set of operations:
//  - alpha compositing into an RGBA buffer
//  - simple shapes (rect/triangle/soft circle)
//  - a couple of SDF helpers for crisp silhouettes
//
// Historically those lived as duplicated "anonymous namespace" helpers in each generator.
// This header centralizes them so we can extend capabilities (blend modes, AA lines, etc)
// without copy/paste drift.
// -----------------------------------------------------------------------------------------------

enum class BlendMode : std::uint8_t {
  Alpha = 0,
  Additive = 1,
};

enum class SampleMode : std::uint8_t {
  Nearest = 0,
  Bilinear = 1,
};

// 2D affine transform mapping (x,y) -> (m00*x + m01*y + m02, m10*x + m11*y + m12).
//
// We express coordinates in "pixel center space": pixel (0,0) is at (0,0) and refers to the
// center of the top-left pixel. This matches the way most procedural generators reason about
// their sprites (integer pixel loops).
struct Affine2D {
  float m00 = 1.0f;
  float m01 = 0.0f;
  float m02 = 0.0f;
  float m10 = 0.0f;
  float m11 = 1.0f;
  float m12 = 0.0f;
};

inline Affine2D AffineIdentity() { return Affine2D{}; }

inline Affine2D AffineTranslate(float tx, float ty)
{
  Affine2D a{};
  a.m02 = tx;
  a.m12 = ty;
  return a;
}

inline Affine2D AffineScale(float sx, float sy)
{
  Affine2D a{};
  a.m00 = sx;
  a.m11 = sy;
  return a;
}

inline Affine2D AffineRotate(float radians)
{
  Affine2D a{};
  float s = 0.0f;
  float c = 1.0f;
  FastSinCosRad(radians, s, c);
  a.m00 = c;
  a.m01 = -s;
  a.m10 = s;
  a.m11 = c;
  return a;
}

// Compose transforms: result = a * b (apply b, then a).
inline Affine2D AffineMul(const Affine2D& a, const Affine2D& b)
{
  Affine2D r{};
  r.m00 = a.m00 * b.m00 + a.m01 * b.m10;
  r.m01 = a.m00 * b.m01 + a.m01 * b.m11;
  r.m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02;

  r.m10 = a.m10 * b.m00 + a.m11 * b.m10;
  r.m11 = a.m10 * b.m01 + a.m11 * b.m11;
  r.m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12;
  return r;
}

inline void TransformPoint(const Affine2D& a, float x, float y, float& outX, float& outY)
{
  outX = a.m00 * x + a.m01 * y + a.m02;
  outY = a.m10 * x + a.m11 * y + a.m12;
}

inline bool AffineInverse(const Affine2D& a, Affine2D& outInv)
{
  const float det = a.m00 * a.m11 - a.m01 * a.m10;
  if (std::fabs(det) < 1.0e-12f) return false;
  const float invDet = 1.0f / det;

  outInv.m00 = a.m11 * invDet;
  outInv.m01 = -a.m01 * invDet;
  outInv.m10 = -a.m10 * invDet;
  outInv.m11 = a.m00 * invDet;

  outInv.m02 = -(outInv.m00 * a.m02 + outInv.m01 * a.m12);
  outInv.m12 = -(outInv.m10 * a.m02 + outInv.m11 * a.m12);
  return true;
}

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::uint8_t ClampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<std::uint8_t>(v);
}

inline std::uint8_t F01ToU8(float v) { return ClampU8(static_cast<int>(std::lround(Clamp01(v) * 255.0f))); }

inline Rgba8 Mul(Rgba8 c, float m)
{
  const int r = static_cast<int>(std::lround(static_cast<float>(c.r) * m));
  const int g = static_cast<int>(std::lround(static_cast<float>(c.g) * m));
  const int b = static_cast<int>(std::lround(static_cast<float>(c.b) * m));
  return Rgba8{ClampU8(r), ClampU8(g), ClampU8(b), c.a};
}

inline Rgba8 Add(Rgba8 c, int dr, int dg, int db)
{
  return Rgba8{ClampU8(static_cast<int>(c.r) + dr), ClampU8(static_cast<int>(c.g) + dg),
              ClampU8(static_cast<int>(c.b) + db), c.a};
}

inline Rgba8 Lerp(Rgba8 a, Rgba8 b, float t)
{
  t = Clamp01(t);
  const int r = static_cast<int>(std::lround(static_cast<float>(a.r) + (static_cast<float>(b.r) - static_cast<float>(a.r)) * t));
  const int g = static_cast<int>(std::lround(static_cast<float>(a.g) + (static_cast<float>(b.g) - static_cast<float>(a.g)) * t));
  const int bl = static_cast<int>(std::lround(static_cast<float>(a.b) + (static_cast<float>(b.b) - static_cast<float>(a.b)) * t));
  const int al = static_cast<int>(std::lround(static_cast<float>(a.a) + (static_cast<float>(b.a) - static_cast<float>(a.a)) * t));
  return Rgba8{ClampU8(r), ClampU8(g), ClampU8(bl), ClampU8(al)};
}

inline Rgba8 Mix(Rgba8 a, Rgba8 b, float t) { return Lerp(a, b, t); }

inline void Clear(RgbaImage& img)
{
  std::fill(img.rgba.begin(), img.rgba.end(), std::uint8_t{0});
}


inline void Clear(RgbaImage& img, Rgba8 c)
{
  if (img.width <= 0 || img.height <= 0) return;
  img.rgba.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u);
  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
      img.rgba[i + 0] = c.r;
      img.rgba[i + 1] = c.g;
      img.rgba[i + 2] = c.b;
      img.rgba[i + 3] = c.a;
    }
  }
}

inline Rgba8 ReadPixel(const RgbaImage& img, int x, int y)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return Rgba8{0, 0, 0, 0};
  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
  return Rgba8{img.rgba[i + 0], img.rgba[i + 1], img.rgba[i + 2], img.rgba[i + 3]};
}

inline void WritePixel(RgbaImage& img, int x, int y, Rgba8 c)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
  img.rgba[i + 0] = c.r;
  img.rgba[i + 1] = c.g;
  img.rgba[i + 2] = c.b;
  img.rgba[i + 3] = c.a;
}


inline Rgba8 SampleNearest(const RgbaImage& img, float x, float y)
{
  const int ix = static_cast<int>(std::lround(x));
  const int iy = static_cast<int>(std::lround(y));
  return ReadPixel(img, ix, iy);
}

// Bilinear sampling in premultiplied space to avoid color fringes when alpha < 1.
//
// Coordinates are in pixel-center space: (0,0) samples the center of the top-left pixel.
inline Rgba8 SampleBilinearPremultiplied(const RgbaImage& img, float x, float y)
{
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const float tx = x - static_cast<float>(x0);
  const float ty = y - static_cast<float>(y0);

  const Rgba8 c00 = ReadPixel(img, x0, y0);
  const Rgba8 c10 = ReadPixel(img, x0 + 1, y0);
  const Rgba8 c01 = ReadPixel(img, x0, y0 + 1);
  const Rgba8 c11 = ReadPixel(img, x0 + 1, y0 + 1);

  const float w00 = (1.0f - tx) * (1.0f - ty);
  const float w10 = tx * (1.0f - ty);
  const float w01 = (1.0f - tx) * ty;
  const float w11 = tx * ty;

  const float a = static_cast<float>(c00.a) * w00 + static_cast<float>(c10.a) * w10 + static_cast<float>(c01.a) * w01 +
                  static_cast<float>(c11.a) * w11;

  if (a <= 0.5f) {
    return Rgba8{0, 0, 0, 0};
  }

  const float a00 = static_cast<float>(c00.a) / 255.0f;
  const float a10 = static_cast<float>(c10.a) / 255.0f;
  const float a01 = static_cast<float>(c01.a) / 255.0f;
  const float a11 = static_cast<float>(c11.a) / 255.0f;

  // Premultiplied color channels (0..255).
  const float pr = (static_cast<float>(c00.r) * a00) * w00 + (static_cast<float>(c10.r) * a10) * w10 +
                   (static_cast<float>(c01.r) * a01) * w01 + (static_cast<float>(c11.r) * a11) * w11;
  const float pg = (static_cast<float>(c00.g) * a00) * w00 + (static_cast<float>(c10.g) * a10) * w10 +
                   (static_cast<float>(c01.g) * a01) * w01 + (static_cast<float>(c11.g) * a11) * w11;
  const float pb = (static_cast<float>(c00.b) * a00) * w00 + (static_cast<float>(c10.b) * a10) * w10 +
                   (static_cast<float>(c01.b) * a01) * w01 + (static_cast<float>(c11.b) * a11) * w11;

  const float invA = 255.0f / a;
  const int outA = static_cast<int>(std::lround(a));
  const int outR = static_cast<int>(std::lround(pr * invA));
  const int outG = static_cast<int>(std::lround(pg * invA));
  const int outB = static_cast<int>(std::lround(pb * invA));

  return Rgba8{ClampU8(outR), ClampU8(outG), ClampU8(outB), ClampU8(outA)};
}

// Source-over alpha blend (straight alpha input).
inline void BlendPixelAlpha(RgbaImage& img, int x, int y, Rgba8 src)
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

// Additive blend that preserves "straight" RGB + alpha semantics.
//
// For emissive sprites we want overlapping lights to *accumulate* (commutative) rather than
// overwrite (alpha over). We treat (rgb, a) as:
//   contribution = rgb * a
// and add contributions, re-encoding back into (rgb, a).
inline void BlendPixelAdditive(RgbaImage& img, int x, int y, Rgba8 src)
{
  if (src.a == 0) return;
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;

  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;

  const int dr = static_cast<int>(img.rgba[i + 0]);
  const int dg = static_cast<int>(img.rgba[i + 1]);
  const int db = static_cast<int>(img.rgba[i + 2]);
  const int da = static_cast<int>(img.rgba[i + 3]);

  const int sa = static_cast<int>(src.a);
  const int outA = std::min(255, da + sa);

  // Pre-multiplied contributions (0..255 per channel).
  const int premDr = (dr * da + 127) / 255;
  const int premDg = (dg * da + 127) / 255;
  const int premDb = (db * da + 127) / 255;

  const int premSr = (static_cast<int>(src.r) * sa + 127) / 255;
  const int premSg = (static_cast<int>(src.g) * sa + 127) / 255;
  const int premSb = (static_cast<int>(src.b) * sa + 127) / 255;

  const int premR = std::min(255, premDr + premSr);
  const int premG = std::min(255, premDg + premSg);
  const int premB = std::min(255, premDb + premSb);

  if (outA <= 0) {
    img.rgba[i + 0] = 0;
    img.rgba[i + 1] = 0;
    img.rgba[i + 2] = 0;
    img.rgba[i + 3] = 0;
    return;
  }

  // Re-encode so that (out.rgb * out.a / 255) == prem.
  const int outR = (premR * 255 + outA / 2) / outA;
  const int outG = (premG * 255 + outA / 2) / outA;
  const int outB = (premB * 255 + outA / 2) / outA;

  img.rgba[i + 0] = ClampU8(outR);
  img.rgba[i + 1] = ClampU8(outG);
  img.rgba[i + 2] = ClampU8(outB);
  img.rgba[i + 3] = ClampU8(outA);
}

inline void BlendPixel(RgbaImage& img, int x, int y, Rgba8 src, BlendMode mode = BlendMode::Alpha)
{
  if (mode == BlendMode::Additive) BlendPixelAdditive(img, x, y, src);
  else BlendPixelAlpha(img, x, y, src);
}


inline void CompositeImage(RgbaImage& dst, const RgbaImage& src, BlendMode mode = BlendMode::Alpha)
{
  const int w = std::min(dst.width, src.width);
  const int h = std::min(dst.height, src.height);
  if (w <= 0 || h <= 0) return;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 4u;
      const Rgba8 c{src.rgba[i + 0], src.rgba[i + 1], src.rgba[i + 2], src.rgba[i + 3]};
      BlendPixel(dst, x, y, c, mode);
    }
  }
}


inline void BlitImageAffine(RgbaImage& dst, const RgbaImage& src, const Affine2D& dstFromSrc, SampleMode sample = SampleMode::Bilinear,
                            BlendMode mode = BlendMode::Alpha)
{
  if (dst.width <= 0 || dst.height <= 0) return;
  if (src.width <= 0 || src.height <= 0) return;

  const std::size_t dstNeed = static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 4u;
  const std::size_t srcNeed = static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 4u;
  if (dst.rgba.size() < dstNeed || src.rgba.size() < srcNeed) return;

  Affine2D srcFromDst{};
  if (!AffineInverse(dstFromSrc, srcFromDst)) return;

  // Compute destination bounds by transforming the source image corner pixel centers.
  const float sx0 = 0.0f;
  const float sy0 = 0.0f;
  const float sx1 = static_cast<float>(src.width - 1);
  const float sy1 = static_cast<float>(src.height - 1);

  float x0, y0, x1, y1, x2, y2, x3, y3;
  TransformPoint(dstFromSrc, sx0, sy0, x0, y0);
  TransformPoint(dstFromSrc, sx1, sy0, x1, y1);
  TransformPoint(dstFromSrc, sx1, sy1, x2, y2);
  TransformPoint(dstFromSrc, sx0, sy1, x3, y3);

  const float minXf = std::min({x0, x1, x2, x3});
  const float maxXf = std::max({x0, x1, x2, x3});
  const float minYf = std::min({y0, y1, y2, y3});
  const float maxYf = std::max({y0, y1, y2, y3});

  const float pad = (sample == SampleMode::Bilinear) ? 1.0f : 0.0f;

  int minX = static_cast<int>(std::floor(minXf - pad));
  int maxX = static_cast<int>(std::ceil(maxXf + pad));
  int minY = static_cast<int>(std::floor(minYf - pad));
  int maxY = static_cast<int>(std::ceil(maxYf + pad));

  minX = std::max(minX, 0);
  minY = std::max(minY, 0);
  maxX = std::min(maxX, dst.width - 1);
  maxY = std::min(maxY, dst.height - 1);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      float sx, sy;
      TransformPoint(srcFromDst, static_cast<float>(x), static_cast<float>(y), sx, sy);

      Rgba8 s{};
      if (sample == SampleMode::Bilinear) {
        s = SampleBilinearPremultiplied(src, sx, sy);
      } else {
        s = SampleNearest(src, sx, sy);
      }

      if (s.a == 0) continue;
      BlendPixel(dst, x, y, s, mode);
    }
  }
}

// Box blur in premultiplied space (preserves colored glows / emissive contributions).
//
// Input is assumed to be "straight alpha" (rgb + a) but blurred by treating (rgb*a) as
// the quantity to blur. This makes it suitable for emissive sprites that are later
// composited additively.
//
// Implementation notes:
//  - zero padding outside the image (glows fade to transparent at edges)
//  - separable blur using sliding sums (O(W*H))
inline void BoxBlurPremultiplied(RgbaImage& img, int radius)
{
  if (radius <= 0) return;
  if (img.width <= 0 || img.height <= 0) return;
  if (img.rgba.size() < static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u) return;

  const int w = img.width;
  const int h = img.height;
  const int k = radius * 2 + 1;
  const int denom = k * k;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  // Horizontal sums per pixel.
  std::vector<int> hA(n, 0);
  std::vector<int> hR(n, 0);
  std::vector<int> hG(n, 0);
  std::vector<int> hB(n, 0);

  auto srcAt = [&](int x, int y) -> Rgba8 { return ReadPixel(img, x, y); };

  for (int y = 0; y < h; ++y) {
    int sumA = 0;
    int sumR = 0;
    int sumG = 0;
    int sumB = 0;

    // Initial window centered at x=0.
    for (int dx = -radius; dx <= radius; ++dx) {
      const int xx = dx;
      if (xx < 0 || xx >= w) continue;
      const Rgba8 s = srcAt(xx, y);
      const int a = static_cast<int>(s.a);
      sumA += a;
      sumR += (static_cast<int>(s.r) * a + 127) / 255;
      sumG += (static_cast<int>(s.g) * a + 127) / 255;
      sumB += (static_cast<int>(s.b) * a + 127) / 255;
    }

    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      hA[idx] = sumA;
      hR[idx] = sumR;
      hG[idx] = sumG;
      hB[idx] = sumB;

      // Slide.
      const int xOut = x - radius;
      const int xIn = x + radius + 1;

      if (xOut >= 0 && xOut < w) {
        const Rgba8 s = srcAt(xOut, y);
        const int a = static_cast<int>(s.a);
        sumA -= a;
        sumR -= (static_cast<int>(s.r) * a + 127) / 255;
        sumG -= (static_cast<int>(s.g) * a + 127) / 255;
        sumB -= (static_cast<int>(s.b) * a + 127) / 255;
      }

      if (xIn >= 0 && xIn < w) {
        const Rgba8 s = srcAt(xIn, y);
        const int a = static_cast<int>(s.a);
        sumA += a;
        sumR += (static_cast<int>(s.r) * a + 127) / 255;
        sumG += (static_cast<int>(s.g) * a + 127) / 255;
        sumB += (static_cast<int>(s.b) * a + 127) / 255;
      }
    }
  }

  std::vector<std::uint8_t> out;
  out.resize(n * 4u, 0u);

  for (int x = 0; x < w; ++x) {
    int sumA = 0;
    int sumR = 0;
    int sumG = 0;
    int sumB = 0;

    // Initial window centered at y=0.
    for (int dy = -radius; dy <= radius; ++dy) {
      const int yy = dy;
      if (yy < 0 || yy >= h) continue;
      const std::size_t idx = static_cast<std::size_t>(yy) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      sumA += hA[idx];
      sumR += hR[idx];
      sumG += hG[idx];
      sumB += hB[idx];
    }

    for (int y = 0; y < h; ++y) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);

      if (sumA <= 0) {
        out[idx * 4u + 0] = 0;
        out[idx * 4u + 1] = 0;
        out[idx * 4u + 2] = 0;
        out[idx * 4u + 3] = 0;
      } else {
        // Average alpha across the kernel.
        const int outA = std::clamp((sumA + denom / 2) / denom, 0, 255);

        // Preserve color ratio in premultiplied space by using sums for the unpremultiply step.
        const int outR = std::clamp((sumR * 255 + sumA / 2) / sumA, 0, 255);
        const int outG = std::clamp((sumG * 255 + sumA / 2) / sumA, 0, 255);
        const int outB = std::clamp((sumB * 255 + sumA / 2) / sumA, 0, 255);

        out[idx * 4u + 0] = static_cast<std::uint8_t>(outR);
        out[idx * 4u + 1] = static_cast<std::uint8_t>(outG);
        out[idx * 4u + 2] = static_cast<std::uint8_t>(outB);
        out[idx * 4u + 3] = static_cast<std::uint8_t>(outA);
      }

      // Slide.
      const int yOut = y - radius;
      const int yIn = y + radius + 1;

      if (yOut >= 0 && yOut < h) {
        const std::size_t iOut = static_cast<std::size_t>(yOut) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        sumA -= hA[iOut];
        sumR -= hR[iOut];
        sumG -= hG[iOut];
        sumB -= hB[iOut];
      }

      if (yIn >= 0 && yIn < h) {
        const std::size_t iIn = static_cast<std::size_t>(yIn) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        sumA += hA[iIn];
        sumR += hR[iIn];
        sumG += hG[iIn];
        sumB += hB[iIn];
      }
    }
  }

  img.rgba = std::move(out);
}

// Inclusive fill rectangle.
inline void FillRect(RgbaImage& img, int x0, int y0, int x1, int y1, Rgba8 c, BlendMode mode = BlendMode::Alpha)
{
  if (img.width <= 0 || img.height <= 0) return;
  const int minX = std::max(0, std::min(x0, x1));
  const int maxX = std::min(img.width - 1, std::max(x0, x1));
  const int minY = std::max(0, std::min(y0, y1));
  const int maxY = std::min(img.height - 1, std::max(y0, y1));
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      BlendPixel(img, x, y, c, mode);
    }
  }
}

// Soft circle (anti-aliased edge via feather).
inline void FillCircleSoft(RgbaImage& img, float cx, float cy, float r, float feather, Rgba8 c, BlendMode mode = BlendMode::Alpha)
{
  if (img.width <= 0 || img.height <= 0) return;
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
      BlendPixel(img, x, y, cc, mode);
    }
  }
}

inline int EdgeFn(int ax, int ay, int bx, int by, int cx, int cy)
{
  return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
}

inline void FillTriangle(RgbaImage& img, int x0, int y0, int x1, int y1, int x2, int y2, Rgba8 c, BlendMode mode = BlendMode::Alpha)
{
  if (img.width <= 0 || img.height <= 0) return;

  int minX = std::min({x0, x1, x2});
  int maxX = std::max({x0, x1, x2});
  int minY = std::min({y0, y1, y2});
  int maxY = std::max({y0, y1, y2});

  minX = std::max(minX, 0);
  minY = std::max(minY, 0);
  maxX = std::min(maxX, img.width - 1);
  maxY = std::min(maxY, img.height - 1);

  const int area = EdgeFn(x0, y0, x1, y1, x2, y2);
  if (area == 0) return;
  const bool ccw = (area > 0);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const int w0 = EdgeFn(x1, y1, x2, y2, x, y);
      const int w1 = EdgeFn(x2, y2, x0, y0, x, y);
      const int w2 = EdgeFn(x0, y0, x1, y1, x, y);

      if (ccw) {
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
      } else {
        if (w0 > 0 || w1 > 0 || w2 > 0) continue;
      }

      BlendPixel(img, x, y, c, mode);
    }
  }
}

// Simple 1px stroke line (Bresenham). Used to give procedural sprites readable silhouettes.
inline void StrokeLine(RgbaImage& img, int x0, int y0, int x1, int y1, Rgba8 c, BlendMode mode = BlendMode::Alpha)
{
  int dx = std::abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    BlendPixel(img, x0, y0, c, mode);
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

// Anti-aliased 1px line using Xiaolin Wu's algorithm.
// Useful for smooth procedural signage/cables at very high zoom.
inline void StrokeLineAA(RgbaImage& img, float x0, float y0, float x1, float y1, Rgba8 c, BlendMode mode = BlendMode::Alpha)
{
  auto ipart = [](float x) -> int { return static_cast<int>(std::floor(x)); };
  auto fpart = [](float x) -> float { return x - std::floor(x); };
  auto rfpart = [&](float x) -> float { return 1.0f - fpart(x); };

  auto plot = [&](int x, int y, float a) {
    if (a <= 0.0f) return;
    Rgba8 cc = c;
    cc.a = ClampU8(static_cast<int>(std::lround(static_cast<float>(c.a) * std::clamp(a, 0.0f, 1.0f))));
    BlendPixel(img, x, y, cc, mode);
  };

  const bool steep = (std::fabs(y1 - y0) > std::fabs(x1 - x0));
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  const float dx = x1 - x0;
  const float dy = y1 - y0;
  const float grad = (std::fabs(dx) < 1.0e-6f) ? 0.0f : (dy / dx);

  // first endpoint
  float xend = std::round(x0);
  float yend = y0 + grad * (xend - x0);
  float xgap = rfpart(x0 + 0.5f);
  const int xpxl1 = static_cast<int>(xend);
  const int ypxl1 = ipart(yend);

  if (steep) {
    plot(ypxl1, xpxl1, rfpart(yend) * xgap);
    plot(ypxl1 + 1, xpxl1, fpart(yend) * xgap);
  } else {
    plot(xpxl1, ypxl1, rfpart(yend) * xgap);
    plot(xpxl1, ypxl1 + 1, fpart(yend) * xgap);
  }
  float intery = yend + grad;

  // second endpoint
  xend = std::round(x1);
  yend = y1 + grad * (xend - x1);
  xgap = fpart(x1 + 0.5f);
  const int xpxl2 = static_cast<int>(xend);
  const int ypxl2 = ipart(yend);

  if (steep) {
    plot(ypxl2, xpxl2, rfpart(yend) * xgap);
    plot(ypxl2 + 1, xpxl2, fpart(yend) * xgap);
  } else {
    plot(xpxl2, ypxl2, rfpart(yend) * xgap);
    plot(xpxl2, ypxl2 + 1, fpart(yend) * xgap);
  }

  // main loop
  if (xpxl2 - xpxl1 > 1) {
    for (int x = xpxl1 + 1; x <= xpxl2 - 1; ++x) {
      if (steep) {
        plot(ipart(intery), x, rfpart(intery));
        plot(ipart(intery) + 1, x, fpart(intery));
      } else {
        plot(x, ipart(intery), rfpart(intery));
        plot(x, ipart(intery) + 1, fpart(intery));
      }
      intery += grad;
    }
  }
}

// Slight isometric-ish lighting used by several sprite generators.
inline float SpriteLight(float nx, float ny)
{
  // nx,ny in [-1,1] roughly. Light from (-0.6,-0.5).
  const float lx = -0.60f;
  const float ly = -0.55f;
  const float d = (nx * lx + ny * ly);
  return std::clamp(0.92f + 0.20f * d, 0.70f, 1.20f);
}

inline float SmoothStep01(float t)
{
  t = Clamp01(t);
  return t * t * (3.0f - 2.0f * t);
}

// Signed distance to a rounded rectangle centered at the origin.
// x,y and half extents are in the same coordinate space.
inline float SdfRoundRect(float x, float y, float hx, float hy, float r)
{
  // Inigo Quilez style SDF.
  const float qx = std::fabs(x) - hx + r;
  const float qy = std::fabs(y) - hy + r;
  const float ax = std::max(qx, 0.0f);
  const float ay = std::max(qy, 0.0f);
  const float outside = std::sqrt(ax * ax + ay * ay);
  const float inside = std::min(std::max(qx, qy), 0.0f);
  return outside + inside - r;
}

} // namespace gfx
} // namespace isocity
