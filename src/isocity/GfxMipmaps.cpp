#include "isocity/GfxMipmaps.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline std::uint8_t ClampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<std::uint8_t>(v);
}

inline float U8To01(std::uint8_t u) { return static_cast<float>(u) / 255.0f; }

bool ValidateRgba(const RgbaImage& img, std::string& outError)
{
  outError.clear();
  if (img.width <= 0 || img.height <= 0) {
    outError = "invalid image dimensions";
    return false;
  }
  const std::size_t expected =
      static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u;
  if (img.rgba.size() != expected) {
    std::ostringstream oss;
    oss << "invalid RGBA buffer size (expected " << expected << ", got " << img.rgba.size() << ")";
    outError = oss.str();
    return false;
  }
  return true;
}

inline void ReadPx(const RgbaImage& img, int x, int y, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, std::uint8_t& a)
{
  x = std::clamp(x, 0, img.width - 1);
  y = std::clamp(y, 0, img.height - 1);
  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
  r = img.rgba[i + 0];
  g = img.rgba[i + 1];
  b = img.rgba[i + 2];
  a = img.rgba[i + 3];
}

inline void WritePx(RgbaImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
  img.rgba[i + 0] = r;
  img.rgba[i + 1] = g;
  img.rgba[i + 2] = b;
  img.rgba[i + 3] = a;
}

inline bool ShouldContinueMipChain(int w, int h, int minSize)
{
  if (w <= 0 || h <= 0) return false;
  if (w == 1 && h == 1) return false;
  minSize = std::max(1, minSize);
  if (w <= minSize && h <= minSize) return false;
  return true;
}

} // namespace

bool DownsampleRgba2x(const RgbaImage& src, bool premultiplyAlpha, RgbaImage& outDst, std::string& outError)
{
  outError.clear();
  outDst = RgbaImage{};

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }

  const int w2 = std::max(1, src.width / 2);
  const int h2 = std::max(1, src.height / 2);

  RgbaImage dst;
  dst.width = w2;
  dst.height = h2;
  dst.rgba.resize(static_cast<std::size_t>(w2) * static_cast<std::size_t>(h2) * 4u, 0u);

  for (int y = 0; y < h2; ++y) {
    for (int x = 0; x < w2; ++x) {
      const int sx = x * 2;
      const int sy = y * 2;

      std::uint8_t r[4] = {0, 0, 0, 0};
      std::uint8_t g[4] = {0, 0, 0, 0};
      std::uint8_t b[4] = {0, 0, 0, 0};
      std::uint8_t a[4] = {0, 0, 0, 0};

      ReadPx(src, sx + 0, sy + 0, r[0], g[0], b[0], a[0]);
      ReadPx(src, sx + 1, sy + 0, r[1], g[1], b[1], a[1]);
      ReadPx(src, sx + 0, sy + 1, r[2], g[2], b[2], a[2]);
      ReadPx(src, sx + 1, sy + 1, r[3], g[3], b[3], a[3]);

      if (premultiplyAlpha) {
        float sumA = 0.0f;
        float sumR = 0.0f;
        float sumG = 0.0f;
        float sumB = 0.0f;

        for (int i = 0; i < 4; ++i) {
          const float af = U8To01(a[i]);
          sumA += af;
          sumR += static_cast<float>(r[i]) * af;
          sumG += static_cast<float>(g[i]) * af;
          sumB += static_cast<float>(b[i]) * af;
        }

        const float avgA = sumA * 0.25f;
        std::uint8_t orr = 0u;
        std::uint8_t ogg = 0u;
        std::uint8_t obb = 0u;
        if (sumA > 1.0e-6f) {
          orr = ClampU8(static_cast<int>(std::lround(sumR / sumA)));
          ogg = ClampU8(static_cast<int>(std::lround(sumG / sumA)));
          obb = ClampU8(static_cast<int>(std::lround(sumB / sumA)));
        }
        const std::uint8_t oaa = ClampU8(static_cast<int>(std::lround(std::clamp(avgA, 0.0f, 1.0f) * 255.0f)));
        WritePx(dst, x, y, orr, ogg, obb, oaa);
      } else {
        const int sumR = static_cast<int>(r[0]) + static_cast<int>(r[1]) + static_cast<int>(r[2]) + static_cast<int>(r[3]);
        const int sumG = static_cast<int>(g[0]) + static_cast<int>(g[1]) + static_cast<int>(g[2]) + static_cast<int>(g[3]);
        const int sumB = static_cast<int>(b[0]) + static_cast<int>(b[1]) + static_cast<int>(b[2]) + static_cast<int>(b[3]);
        const int sumA = static_cast<int>(a[0]) + static_cast<int>(a[1]) + static_cast<int>(a[2]) + static_cast<int>(a[3]);

        const std::uint8_t orr = ClampU8((sumR + 2) / 4);
        const std::uint8_t ogg = ClampU8((sumG + 2) / 4);
        const std::uint8_t obb = ClampU8((sumB + 2) / 4);
        const std::uint8_t oaa = ClampU8((sumA + 2) / 4);
        WritePx(dst, x, y, orr, ogg, obb, oaa);
      }
    }
  }

  outDst = std::move(dst);
  return true;
}

bool DownsampleNormalMap2x(const RgbaImage& src, RgbaImage& outDst, std::string& outError)
{
  outError.clear();
  outDst = RgbaImage{};

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }

  const int w2 = std::max(1, src.width / 2);
  const int h2 = std::max(1, src.height / 2);

  RgbaImage dst;
  dst.width = w2;
  dst.height = h2;
  dst.rgba.resize(static_cast<std::size_t>(w2) * static_cast<std::size_t>(h2) * 4u, 0u);

  for (int y = 0; y < h2; ++y) {
    for (int x = 0; x < w2; ++x) {
      const int sx = x * 2;
      const int sy = y * 2;

      std::uint8_t r[4] = {0, 0, 0, 0};
      std::uint8_t g[4] = {0, 0, 0, 0};
      std::uint8_t b[4] = {0, 0, 0, 0};
      std::uint8_t a[4] = {0, 0, 0, 0};

      ReadPx(src, sx + 0, sy + 0, r[0], g[0], b[0], a[0]);
      ReadPx(src, sx + 1, sy + 0, r[1], g[1], b[1], a[1]);
      ReadPx(src, sx + 0, sy + 1, r[2], g[2], b[2], a[2]);
      ReadPx(src, sx + 1, sy + 1, r[3], g[3], b[3], a[3]);

      float sumA = 0.0f;
      float nx = 0.0f;
      float ny = 0.0f;
      float nz = 0.0f;
      for (int i = 0; i < 4; ++i) {
        const float af = U8To01(a[i]);
        sumA += af;
        const float fx = U8To01(r[i]) * 2.0f - 1.0f;
        const float fy = U8To01(g[i]) * 2.0f - 1.0f;
        const float fz = U8To01(b[i]) * 2.0f - 1.0f;
        nx += fx * af;
        ny += fy * af;
        nz += fz * af;
      }

      const float avgA = sumA * 0.25f;
      std::uint8_t oaa = ClampU8(static_cast<int>(std::lround(std::clamp(avgA, 0.0f, 1.0f) * 255.0f)));

      if (sumA > 1.0e-6f) {
        nx /= sumA;
        ny /= sumA;
        nz /= sumA;
      } else {
        nx = 0.0f;
        ny = 0.0f;
        nz = 1.0f;
      }

      const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (len > 1.0e-6f) {
        nx /= len;
        ny /= len;
        nz /= len;
      } else {
        nx = 0.0f;
        ny = 0.0f;
        nz = 1.0f;
      }

      const std::uint8_t orr = ClampU8(static_cast<int>(std::lround((nx * 0.5f + 0.5f) * 255.0f)));
      const std::uint8_t ogg = ClampU8(static_cast<int>(std::lround((ny * 0.5f + 0.5f) * 255.0f)));
      const std::uint8_t obb = ClampU8(static_cast<int>(std::lround((nz * 0.5f + 0.5f) * 255.0f)));
      WritePx(dst, x, y, orr, ogg, obb, oaa);
    }
  }

  outDst = std::move(dst);
  return true;
}

bool GenerateMipChainRgba(const RgbaImage& src, const GfxMipmapChainConfig& cfg,
                          std::vector<RgbaImage>& outMips, std::string& outError)
{
  outError.clear();
  outMips.clear();

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }

  outMips.push_back(src);

  int w = src.width;
  int h = src.height;

  int produced = 0;
  while (ShouldContinueMipChain(w, h, cfg.minSize)) {
    if (cfg.levels > 0 && produced >= cfg.levels) break;

    RgbaImage next;
    std::string derr;
    if (!DownsampleRgba2x(outMips.back(), cfg.premultiplyAlpha, next, derr)) {
      outError = derr;
      return false;
    }

    w = next.width;
    h = next.height;
    outMips.push_back(std::move(next));
    ++produced;
  }

  return true;
}

bool GenerateMipChainNormalMap(const RgbaImage& src, const GfxMipmapChainConfig& cfg,
                               std::vector<RgbaImage>& outMips, std::string& outError)
{
  outError.clear();
  outMips.clear();

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }

  outMips.push_back(src);

  int w = src.width;
  int h = src.height;

  int produced = 0;
  while (ShouldContinueMipChain(w, h, cfg.minSize)) {
    if (cfg.levels > 0 && produced >= cfg.levels) break;

    RgbaImage next;
    std::string derr;
    if (!DownsampleNormalMap2x(outMips.back(), next, derr)) {
      outError = derr;
      return false;
    }
    w = next.width;
    h = next.height;
    outMips.push_back(std::move(next));
    ++produced;
  }

  return true;
}


float AlphaCoverage(const RgbaImage& img, int x, int y, int w, int h, float threshold)
{
  std::string verr;
  if (!ValidateRgba(img, verr)) return 0.0f;

  threshold = std::clamp(threshold, 0.0f, 1.0f);
  const int t = ClampU8(static_cast<int>(std::lround(threshold * 255.0f)));

  // Clamp the rect to the image.
  const int x0 = std::clamp(x, 0, img.width);
  const int y0 = std::clamp(y, 0, img.height);
  const int x1 = std::clamp(x + w, 0, img.width);
  const int y1 = std::clamp(y + h, 0, img.height);

  const int cw = std::max(0, x1 - x0);
  const int ch = std::max(0, y1 - y0);
  if (cw <= 0 || ch <= 0) return 0.0f;

  std::uint64_t count = 0;
  const std::uint64_t total = static_cast<std::uint64_t>(cw) * static_cast<std::uint64_t>(ch);

  for (int yy = y0; yy < y1; ++yy) {
    const std::size_t row = static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) * 4u;
    for (int xx = x0; xx < x1; ++xx) {
      const std::size_t i = row + static_cast<std::size_t>(xx) * 4u;
      const std::uint8_t a = img.rgba[i + 3];
      if (a >= static_cast<std::uint8_t>(t)) ++count;
    }
  }

  return static_cast<float>(count) / static_cast<float>(total);
}

namespace {

inline void RectToMip(const GfxSpriteRect& r0, int mipLevel, int& outX, int& outY, int& outW, int& outH)
{
  if (mipLevel <= 0) {
    outX = r0.x;
    outY = r0.y;
    outW = r0.w;
    outH = r0.h;
    return;
  }

  // Map [x0,x1) -> [floor(x0/2^L), ceil(x1/2^L))
  const int scale = 1 << std::clamp(mipLevel, 0, 30);
  const int x0 = r0.x;
  const int y0 = r0.y;
  const int x1 = r0.x + r0.w;
  const int y1 = r0.y + r0.h;

  const int mx0 = x0 / scale;
  const int my0 = y0 / scale;
  const int mx1 = (x1 + scale - 1) / scale;
  const int my1 = (y1 + scale - 1) / scale;

  outX = mx0;
  outY = my0;
  outW = std::max(0, mx1 - mx0);
  outH = std::max(0, my1 - my0);
}

inline void ClampRectToImage(const RgbaImage& img, int& x, int& y, int& w, int& h)
{
  const int x0 = std::clamp(x, 0, img.width);
  const int y0 = std::clamp(y, 0, img.height);
  const int x1 = std::clamp(x + w, 0, img.width);
  const int y1 = std::clamp(y + h, 0, img.height);
  x = x0;
  y = y0;
  w = std::max(0, x1 - x0);
  h = std::max(0, y1 - y0);
}

inline float AlphaCoverageRectThreshold(const RgbaImage& img, int x, int y, int w, int h, int threshU8)
{
  ClampRectToImage(img, x, y, w, h);
  if (w <= 0 || h <= 0) return 0.0f;

  const std::uint64_t total = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  std::uint64_t count = 0;

  for (int yy = y; yy < y + h; ++yy) {
    const std::size_t row = static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) * 4u;
    for (int xx = x; xx < x + w; ++xx) {
      const std::size_t i = row + static_cast<std::size_t>(xx) * 4u;
      const std::uint8_t a = img.rgba[i + 3];
      if (a >= static_cast<std::uint8_t>(threshU8)) ++count;
    }
  }

  return static_cast<float>(count) / static_cast<float>(total);
}

inline float AlphaCoverageRectNonZero(const RgbaImage& img, int x, int y, int w, int h)
{
  ClampRectToImage(img, x, y, w, h);
  if (w <= 0 || h <= 0) return 0.0f;

  const std::uint64_t total = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  std::uint64_t count = 0;

  for (int yy = y; yy < y + h; ++yy) {
    const std::size_t row = static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) * 4u;
    for (int xx = x; xx < x + w; ++xx) {
      const std::size_t i = row + static_cast<std::size_t>(xx) * 4u;
      const std::uint8_t a = img.rgba[i + 3];
      if (a > 0u) ++count;
    }
  }

  return static_cast<float>(count) / static_cast<float>(total);
}

inline float AlphaCoverageRectScaled(const RgbaImage& img, int x, int y, int w, int h, int threshU8, float scale)
{
  ClampRectToImage(img, x, y, w, h);
  if (w <= 0 || h <= 0) return 0.0f;

  if (scale <= 0.0f) return 0.0f;

  const std::uint64_t total = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  std::uint64_t count = 0;

  const float t = static_cast<float>(threshU8);

  for (int yy = y; yy < y + h; ++yy) {
    const std::size_t row = static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) * 4u;
    for (int xx = x; xx < x + w; ++xx) {
      const std::size_t i = row + static_cast<std::size_t>(xx) * 4u;
      const float a = static_cast<float>(img.rgba[i + 3]) * scale;
      const float aa = std::min(255.0f, a);
      if (aa >= t) ++count;
    }
  }

  return static_cast<float>(count) / static_cast<float>(total);
}

inline void ApplyAlphaScaleRect(RgbaImage& img, int x, int y, int w, int h, float scale)
{
  ClampRectToImage(img, x, y, w, h);
  if (w <= 0 || h <= 0) return;

  if (scale <= 0.0f) {
    // Fast path: clear alpha.
    for (int yy = y; yy < y + h; ++yy) {
      const std::size_t row = static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) * 4u;
      for (int xx = x; xx < x + w; ++xx) {
        const std::size_t i = row + static_cast<std::size_t>(xx) * 4u;
        img.rgba[i + 3] = 0u;
      }
    }
    return;
  }

  for (int yy = y; yy < y + h; ++yy) {
    const std::size_t row = static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) * 4u;
    for (int xx = x; xx < x + w; ++xx) {
      const std::size_t i = row + static_cast<std::size_t>(xx) * 4u;
      const float a = static_cast<float>(img.rgba[i + 3]) * scale;
      const int ai = static_cast<int>(std::lround(std::min(255.0f, std::max(0.0f, a))));
      img.rgba[i + 3] = ClampU8(ai);
    }
  }
}

} // namespace

bool ComputeAlphaCoverageTargets(const RgbaImage& mip0, const std::vector<GfxSpriteRect>& sprites,
                                 float threshold, std::vector<float>& outTargets, std::string& outError)
{
  outError.clear();
  outTargets.clear();

  std::string verr;
  if (!ValidateRgba(mip0, verr)) {
    outError = verr;
    return false;
  }

  threshold = std::clamp(threshold, 0.0f, 1.0f);
  outTargets.resize(sprites.size(), 0.0f);

  for (std::size_t i = 0; i < sprites.size(); ++i) {
    const auto& r = sprites[i];
    outTargets[i] = AlphaCoverage(mip0, r.x, r.y, r.w, r.h, threshold);
  }

  return true;
}

bool PreserveAlphaCoverageForMip(RgbaImage& ioMip, const std::vector<GfxSpriteRect>& sprites,
                                 const std::vector<float>& targets, int mipLevel,
                                 float threshold, int iterations,
                                 std::vector<float>* outScales, std::string& outError)
{
  outError.clear();

  std::string verr;
  if (!ValidateRgba(ioMip, verr)) {
    outError = verr;
    return false;
  }

  if (sprites.size() != targets.size()) {
    outError = "sprites/targets size mismatch";
    return false;
  }

  if (mipLevel < 0) {
    outError = "invalid mipLevel";
    return false;
  }

  threshold = std::clamp(threshold, 0.0f, 1.0f);
  const int threshU8 = ClampU8(static_cast<int>(std::lround(threshold * 255.0f)));

  iterations = std::clamp(iterations, 1, 64);

  if (outScales) {
    outScales->assign(sprites.size(), 1.0f);
  }

  if (mipLevel == 0) return true;
  if (threshU8 <= 0) return true; // threshold 0 => coverage always 1

  for (std::size_t i = 0; i < sprites.size(); ++i) {
    int rx = 0, ry = 0, rw = 0, rh = 0;
    RectToMip(sprites[i], mipLevel, rx, ry, rw, rh);
    ClampRectToImage(ioMip, rx, ry, rw, rh);
    if (rw <= 0 || rh <= 0) continue;

    const float maxCov = AlphaCoverageRectNonZero(ioMip, rx, ry, rw, rh);
    if (maxCov <= 0.0f) continue;

    const float tgt = std::min(std::max(0.0f, targets[i]), maxCov);
    const float cur = AlphaCoverageRectThreshold(ioMip, rx, ry, rw, rh, threshU8);

    // The smallest possible step in coverage for this sprite/level.
    const float step = 1.0f / static_cast<float>(rw * rh);

    if (std::fabs(cur - tgt) <= step * 0.5f) {
      if (outScales) (*outScales)[i] = 1.0f;
      continue;
    }

    // Binary search a scale factor that makes coverage(mid) ~ target.
    float lo = 0.0f;
    float hi = 256.0f;

    // If even a huge scale can't reach tgt (because too many pixels are exactly alpha=0),
    // just saturate everything that exists.
    const float hiCov = AlphaCoverageRectScaled(ioMip, rx, ry, rw, rh, threshU8, hi);
    if (hiCov + step * 0.5f < tgt) {
      ApplyAlphaScaleRect(ioMip, rx, ry, rw, rh, hi);
      if (outScales) (*outScales)[i] = hi;
      continue;
    }

    for (int it = 0; it < iterations; ++it) {
      const float mid = (lo + hi) * 0.5f;
      const float cov = AlphaCoverageRectScaled(ioMip, rx, ry, rw, rh, threshU8, mid);
      if (cov < tgt) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    ApplyAlphaScaleRect(ioMip, rx, ry, rw, rh, hi);
    if (outScales) (*outScales)[i] = hi;
  }

  return true;
}

bool ApplyAlphaScalesForMip(RgbaImage& ioMip, const std::vector<GfxSpriteRect>& sprites,
                            const std::vector<float>& scales, int mipLevel,
                            std::string& outError)
{
  outError.clear();

  std::string verr;
  if (!ValidateRgba(ioMip, verr)) {
    outError = verr;
    return false;
  }

  if (sprites.size() != scales.size()) {
    outError = "sprites/scales size mismatch";
    return false;
  }

  if (mipLevel < 0) {
    outError = "invalid mipLevel";
    return false;
  }

  for (std::size_t i = 0; i < sprites.size(); ++i) {
    int rx = 0, ry = 0, rw = 0, rh = 0;
    RectToMip(sprites[i], mipLevel, rx, ry, rw, rh);
    ApplyAlphaScaleRect(ioMip, rx, ry, rw, rh, scales[i]);
  }

  return true;
}

bool ExtrudeSpritePadding(RgbaImage& ioAtlas, int x, int y, int w, int h, int extrudePx, std::string& outError)
{
  outError.clear();

  std::string verr;
  if (!ValidateRgba(ioAtlas, verr)) {
    outError = verr;
    return false;
  }

  if (extrudePx <= 0) return true;
  if (w <= 0 || h <= 0) return true;

  const int ax0 = x;
  const int ay0 = y;
  const int ax1 = x + w;
  const int ay1 = y + h;

  if (ax0 < 0 || ay0 < 0 || ax1 > ioAtlas.width || ay1 > ioAtlas.height) {
    outError = "sprite rect out of bounds";
    return false;
  }

  const int ex = std::min(extrudePx, std::max(ioAtlas.width, ioAtlas.height));
  const int x0 = std::max(0, ax0 - ex);
  const int y0 = std::max(0, ay0 - ex);
  const int x1 = std::min(ioAtlas.width, ax1 + ex);
  const int y1 = std::min(ioAtlas.height, ay1 + ex);

  for (int yy = y0; yy < y1; ++yy) {
    for (int xx = x0; xx < x1; ++xx) {
      if (xx >= ax0 && xx < ax1 && yy >= ay0 && yy < ay1) continue;
      const std::size_t di = (static_cast<std::size_t>(yy) * static_cast<std::size_t>(ioAtlas.width) + static_cast<std::size_t>(xx)) * 4u;
      if (ioAtlas.rgba[di + 3] != 0u) continue; // only write to transparent pixels

      const int cx = std::clamp(xx, ax0, ax1 - 1);
      const int cy = std::clamp(yy, ay0, ay1 - 1);
      const std::size_t si = (static_cast<std::size_t>(cy) * static_cast<std::size_t>(ioAtlas.width) + static_cast<std::size_t>(cx)) * 4u;
      ioAtlas.rgba[di + 0] = ioAtlas.rgba[si + 0];
      ioAtlas.rgba[di + 1] = ioAtlas.rgba[si + 1];
      ioAtlas.rgba[di + 2] = ioAtlas.rgba[si + 2];
      ioAtlas.rgba[di + 3] = ioAtlas.rgba[si + 3];
    }
  }

  return true;
}

} // namespace isocity
