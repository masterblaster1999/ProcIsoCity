#include "isocity/GfxAtlasFx.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::uint8_t ClampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<std::uint8_t>(v);
}

inline std::uint8_t F01ToU8(float v)
{
  const int iv = static_cast<int>(std::lround(Clamp01(v) * 255.0f));
  return ClampU8(iv);
}

inline float U8To01(std::uint8_t u) { return static_cast<float>(u) / 255.0f; }

inline int Luma709(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  // Integer approximation of Rec.709 luma coefficients (scaled by 256):
  //   0.2126, 0.7152, 0.0722 -> 54, 183, 19 (sum = 256)
  // This keeps the tool deterministic and avoids subtle float differences.
  return (54 * static_cast<int>(r) + 183 * static_cast<int>(g) + 19 * static_cast<int>(b) + 128) >> 8;
}

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

bool BuildHeightField01(const RgbaImage& src, GfxHeightMode mode, std::vector<float>& outH01, std::string& outError)
{
  outError.clear();
  outH01.clear();

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }

  const int w = src.width;
  const int h = src.height;
  outH01.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0.0f);

  const std::size_t pxCount = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  for (std::size_t i = 0; i < pxCount; ++i) {
    const std::size_t si = i * 4u;
    const std::uint8_t r = src.rgba[si + 0];
    const std::uint8_t g = src.rgba[si + 1];
    const std::uint8_t b = src.rgba[si + 2];
    const std::uint8_t a = src.rgba[si + 3];

    const float af = U8To01(a);
    const float lf = U8To01(static_cast<std::uint8_t>(Luma709(r, g, b)));

    float hv = 0.0f;
    switch (mode) {
    case GfxHeightMode::Alpha:
      hv = af;
      break;
    case GfxHeightMode::Luma:
      hv = af * lf;
      break;
    case GfxHeightMode::AlphaLuma:
    default:
      // Mostly alpha (silhouette), with a little luminance modulation.
      hv = af * (0.70f + 0.30f * lf);
      break;
    }

    outH01[i] = Clamp01(hv);
  }

  return true;
}

inline float SampleClamped(const std::vector<float>& h, int w, int hh, int x, int y)
{
  x = std::clamp(x, 0, w - 1);
  y = std::clamp(y, 0, hh - 1);
  return h[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
}

void BoxBlurAlpha(RgbaImage& img, int radiusPx)
{
  if (radiusPx <= 0) return;
  const int w = img.width;
  const int h = img.height;
  if (w <= 0 || h <= 0) return;

  std::vector<std::uint8_t> tmp(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), std::uint8_t{0});

  // Horizontal pass.
  std::vector<int> prefix(static_cast<std::size_t>(w) + 1u, 0);
  for (int y = 0; y < h; ++y) {
    prefix[0] = 0;
    for (int x = 0; x < w; ++x) {
      const std::size_t si = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      prefix[static_cast<std::size_t>(x) + 1u] = prefix[static_cast<std::size_t>(x)] + static_cast<int>(img.rgba[si + 3]);
    }

    for (int x = 0; x < w; ++x) {
      const int l = std::max(0, x - radiusPx);
      const int r = std::min(w - 1, x + radiusPx);
      const int sum = prefix[static_cast<std::size_t>(r) + 1u] - prefix[static_cast<std::size_t>(l)];
      const int denom = (r - l + 1);
      tmp[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          static_cast<std::uint8_t>(sum / std::max(1, denom));
    }
  }

  // Vertical pass.
  prefix.assign(static_cast<std::size_t>(h) + 1u, 0);
  for (int x = 0; x < w; ++x) {
    prefix[0] = 0;
    for (int y = 0; y < h; ++y) {
      prefix[static_cast<std::size_t>(y) + 1u] = prefix[static_cast<std::size_t>(y)] +
          static_cast<int>(tmp[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)]);
    }

    for (int y = 0; y < h; ++y) {
      const int t = std::max(0, y - radiusPx);
      const int b = std::min(h - 1, y + radiusPx);
      const int sum = prefix[static_cast<std::size_t>(b) + 1u] - prefix[static_cast<std::size_t>(t)];
      const int denom = (b - t + 1);

      const std::uint8_t a = static_cast<std::uint8_t>(sum / std::max(1, denom));
      const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      img.rgba[di + 0] = 0u;
      img.rgba[di + 1] = 0u;
      img.rgba[di + 2] = 0u;
      img.rgba[di + 3] = a;
    }
  }
}


constexpr float kDtInf = 1.0e20f;

// 1D squared Euclidean distance transform (Felzenszwalb & Huttenlocher).
// f[i] is 0 at feature pixels, and a large value elsewhere.
void DistanceTransform1DSq(const float* f, int n, float* outD, int* v, float* z)
{
  if (!f || !outD || !v || !z || n <= 0) return;

  int k = 0;
  v[0] = 0;
  z[0] = -kDtInf;
  z[1] = +kDtInf;

  for (int q = 1; q < n; ++q) {
    float s = 0.0f;

    while (k > 0) {
      const int vk = v[k];
      const float num = (f[q] + static_cast<float>(q * q)) - (f[vk] + static_cast<float>(vk * vk));
      const float den = 2.0f * static_cast<float>(q - vk);
      s = num / den;
      if (s <= z[k]) {
        --k;
      } else {
        break;
      }
    }

    {
      const int vk = v[k];
      const float num = (f[q] + static_cast<float>(q * q)) - (f[vk] + static_cast<float>(vk * vk));
      const float den = 2.0f * static_cast<float>(q - vk);
      s = num / den;
    }

    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = +kDtInf;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    const float qf = static_cast<float>(q);
    while (z[k + 1] < qf) ++k;
    const int vk = v[k];
    const float dx = static_cast<float>(q - vk);
    outD[q] = dx * dx + f[vk];
  }
}

// 2D squared distance transform to the nearest "feature" pixel.
// features is a w*h byte mask where 1 indicates a feature pixel.
void DistanceTransform2DSq(const std::vector<std::uint8_t>& features, int w, int h, std::vector<float>& outSq)
{
  const std::size_t npx = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  outSq.assign(npx, kDtInf);
  if (w <= 0 || h <= 0) return;
  if (features.size() != npx) return;

  bool any = false;
  for (std::uint8_t b : features) {
    if (b != 0u) {
      any = true;
      break;
    }
  }
  if (!any) return;

  std::vector<float> g(npx, kDtInf);

  const int n = std::max(w, h);
  std::vector<float> f(static_cast<std::size_t>(n), kDtInf);
  std::vector<float> d(static_cast<std::size_t>(n), kDtInf);
  std::vector<int> v(static_cast<std::size_t>(n), 0);
  std::vector<float> z(static_cast<std::size_t>(n) + 1u, 0.0f);

  // Row pass.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      f[static_cast<std::size_t>(x)] = (features[idx] != 0u) ? 0.0f : kDtInf;
    }
    DistanceTransform1DSq(f.data(), w, d.data(), v.data(), z.data());
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      g[idx] = d[static_cast<std::size_t>(x)];
    }
  }

  // Column pass.
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      f[static_cast<std::size_t>(y)] = g[idx];
    }
    DistanceTransform1DSq(f.data(), h, d.data(), v.data(), z.data());
    for (int y = 0; y < h; ++y) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      outSq[idx] = d[static_cast<std::size_t>(y)];
    }
  }
}

} // namespace

const char* GfxHeightModeName(GfxHeightMode m)
{
  switch (m) {
  case GfxHeightMode::Alpha: return "alpha";
  case GfxHeightMode::Luma: return "luma";
  case GfxHeightMode::AlphaLuma: return "alpha_luma";
  default: return "alpha_luma";
  }
}

bool ParseGfxHeightMode(const std::string& s, GfxHeightMode& out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    if (c >= 'A' && c <= 'Z') t.push_back(static_cast<char>(c - 'A' + 'a'));
    else t.push_back(c);
  }

  if (t == "alpha") {
    out = GfxHeightMode::Alpha;
    return true;
  }
  if (t == "luma" || t == "lum" || t == "luminance") {
    out = GfxHeightMode::Luma;
    return true;
  }
  if (t == "alpha_luma" || t == "alphaluma" || t == "alpha+luma") {
    out = GfxHeightMode::AlphaLuma;
    return true;
  }

  return false;
}

bool GenerateHeightMap(const RgbaImage& src, GfxHeightMode mode, RgbaImage& outHeight, std::string& outError)
{
  outError.clear();
  outHeight = RgbaImage{};

  std::vector<float> h01;
  std::string herr;
  if (!BuildHeightField01(src, mode, h01, herr)) {
    outError = herr;
    return false;
  }

  RgbaImage out;
  out.width = src.width;
  out.height = src.height;
  out.rgba.resize(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4u, 0u);

  const std::size_t pxCount = static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height);
  for (std::size_t i = 0; i < pxCount; ++i) {
    const std::size_t si = i * 4u;
    const std::size_t di = si;

    const std::uint8_t a = src.rgba[si + 3];
    const std::uint8_t v = F01ToU8(h01[i]);
    out.rgba[di + 0] = v;
    out.rgba[di + 1] = v;
    out.rgba[di + 2] = v;
    out.rgba[di + 3] = a;
  }

  outHeight = std::move(out);
  return true;
}

bool GenerateNormalMap(const RgbaImage& src, const GfxNormalMapConfig& cfg, RgbaImage& outNormal, std::string& outError)
{
  outError.clear();
  outNormal = RgbaImage{};

  if (!std::isfinite(cfg.strength) || cfg.strength <= 0.0f) {
    outError = "invalid normal strength";
    return false;
  }

  std::vector<float> h01;
  std::string herr;
  if (!BuildHeightField01(src, cfg.heightMode, h01, herr)) {
    outError = herr;
    return false;
  }

  const int w = src.width;
  const int h = src.height;

  RgbaImage out;
  out.width = w;
  out.height = h;
  out.rgba.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float tl = SampleClamped(h01, w, h, x - 1, y - 1);
      const float tc = SampleClamped(h01, w, h, x + 0, y - 1);
      const float tr = SampleClamped(h01, w, h, x + 1, y - 1);
      const float ml = SampleClamped(h01, w, h, x - 1, y + 0);
      const float mr = SampleClamped(h01, w, h, x + 1, y + 0);
      const float bl = SampleClamped(h01, w, h, x - 1, y + 1);
      const float bc = SampleClamped(h01, w, h, x + 0, y + 1);
      const float br = SampleClamped(h01, w, h, x + 1, y + 1);

      // Sobel derivatives (y increases downwards here).
      const float gx = (-tl + tr) + (-2.0f * ml + 2.0f * mr) + (-bl + br);
      const float gy = (-tl - 2.0f * tc - tr) + (bl + 2.0f * bc + br);

      float nx = -gx * cfg.strength;
      float ny = +gy * cfg.strength;
      float nz = 1.0f;

      const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (len > 1.0e-8f) {
        const float inv = 1.0f / len;
        nx *= inv;
        ny *= inv;
        nz *= inv;
      } else {
        nx = 0.0f;
        ny = 0.0f;
        nz = 1.0f;
      }

      const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      const std::size_t si = di;
      const std::uint8_t a = src.rgba[si + 3];

      // Flat default for fully transparent pixels.
      if (a == 0u) {
        out.rgba[di + 0] = 128u;
        out.rgba[di + 1] = 128u;
        out.rgba[di + 2] = 255u;
        out.rgba[di + 3] = 0u;
        continue;
      }

      out.rgba[di + 0] = F01ToU8(nx * 0.5f + 0.5f);
      out.rgba[di + 1] = F01ToU8(ny * 0.5f + 0.5f);
      out.rgba[di + 2] = F01ToU8(nz * 0.5f + 0.5f);
      out.rgba[di + 3] = a;
    }
  }

  outNormal = std::move(out);
  return true;
}

bool GenerateShadowMap(const RgbaImage& src, const GfxShadowConfig& cfg, RgbaImage& outShadow, std::string& outError)
{
  outError.clear();
  outShadow = RgbaImage{};

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }
  if (!std::isfinite(cfg.dirX) || !std::isfinite(cfg.dirY)) {
    outError = "invalid shadow direction";
    return false;
  }
  if (!std::isfinite(cfg.lengthPx) || cfg.lengthPx < 0.0f) {
    outError = "invalid shadow length";
    return false;
  }
  if (!std::isfinite(cfg.opacity) || cfg.opacity < 0.0f) {
    outError = "invalid shadow opacity";
    return false;
  }
  if (cfg.blurRadiusPx < 0) {
    outError = "invalid shadow blur radius";
    return false;
  }

  float dx = cfg.dirX;
  float dy = cfg.dirY;
  const float dlen = std::sqrt(dx * dx + dy * dy);
  if (dlen < 1.0e-6f) {
    outError = "shadow direction too small";
    return false;
  }
  dx /= dlen;
  dy /= dlen;

  const int w = src.width;
  const int h = src.height;

  RgbaImage out;
  out.width = w;
  out.height = h;
  out.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, 0u);

  const float opacity = std::clamp(cfg.opacity, 0.0f, 1.0f);

  for (int y = 0; y < h; ++y) {
    const float yn = (h > 1) ? (static_cast<float>(y) / static_cast<float>(h - 1)) : 0.0f;
    // Higher near the top of the sprite.
    const float elevBase = std::pow(Clamp01(1.0f - yn), 1.25f);

    for (int x = 0; x < w; ++x) {
      const std::size_t si = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4u;
      const std::uint8_t a = src.rgba[si + 3];
      if (a == 0u) continue;

      const float af = U8To01(a);
      const float elev = elevBase;
      const float height = af * elev;
      const float offset = height * cfg.lengthPx;

      const int tx = x + static_cast<int>(std::lround(dx * offset));
      const int ty = y + static_cast<int>(std::lround(dy * offset));
      if (tx < 0 || ty < 0 || tx >= w || ty >= h) continue;

      const float contrib = Clamp01(af * (0.35f + 0.65f * elev) * opacity);
      const std::uint8_t ca = F01ToU8(contrib);

      const std::size_t di = (static_cast<std::size_t>(ty) * static_cast<std::size_t>(w) + static_cast<std::size_t>(tx)) * 4u;
      out.rgba[di + 3] = std::max(out.rgba[di + 3], ca);
    }
  }

  BoxBlurAlpha(out, cfg.blurRadiusPx);

  outShadow = std::move(out);
  return true;
}


bool GenerateSignedDistanceField(const RgbaImage& src, const GfxSdfConfig& cfg, RgbaImage& outSdf, std::string& outError)
{
  outError.clear();
  outSdf = RgbaImage{};

  std::string verr;
  if (!ValidateRgba(src, verr)) {
    outError = verr;
    return false;
  }
  if (!std::isfinite(cfg.spreadPx) || cfg.spreadPx <= 0.0f) {
    outError = "invalid sdf spread";
    return false;
  }
  if (!std::isfinite(cfg.alphaThreshold) || cfg.alphaThreshold < 0.0f || cfg.alphaThreshold > 1.0f) {
    outError = "invalid sdf alpha threshold";
    return false;
  }

  const int w = src.width;
  const int h = src.height;
  const std::size_t npx = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  const int thr = static_cast<int>(std::lround(cfg.alphaThreshold * 255.0f));
  std::vector<std::uint8_t> inside(npx, std::uint8_t{0});
  std::vector<std::uint8_t> outside(npx, std::uint8_t{0});

  for (std::size_t i = 0; i < npx; ++i) {
    const std::uint8_t a = src.rgba[i * 4u + 3u];
    const bool in = static_cast<int>(a) >= thr;
    inside[i] = in ? 1u : 0u;
    outside[i] = in ? 0u : 1u;
  }

  std::vector<float> distToInsideSq;
  std::vector<float> distToOutsideSq;
  DistanceTransform2DSq(inside, w, h, distToInsideSq);
  DistanceTransform2DSq(outside, w, h, distToOutsideSq);

  RgbaImage out;
  out.width = w;
  out.height = h;
  out.rgba.resize(npx * 4u, 0u);

  for (std::size_t i = 0; i < npx; ++i) {
    const bool in = (inside[i] != 0u);
    const float d = std::sqrt(in ? distToOutsideSq[i] : distToInsideSq[i]);

    // Subtract 0.5 so the implicit surface falls roughly between pixel centers.
    const float sd = in ? (d - 0.5f) : -(d - 0.5f);

    const float v = Clamp01(0.5f + sd / cfg.spreadPx);
    const std::uint8_t u = F01ToU8(v);

    const std::size_t di = i * 4u;
    out.rgba[di + 0] = u;
    out.rgba[di + 1] = u;
    out.rgba[di + 2] = u;

    if (cfg.opaqueAlpha) out.rgba[di + 3] = 255u;
    else out.rgba[di + 3] = src.rgba[di + 3];
  }

  outSdf = std::move(out);
  return true;
}

} // namespace isocity
