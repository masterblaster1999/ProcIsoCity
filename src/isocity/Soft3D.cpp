#include "isocity/Soft3D.hpp"

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
                          float depthEps)
{
  const float dx = b.sx - a.sx;
  const float dy = b.sy - a.sy;
  const float adx = std::fabs(dx);
  const float ady = std::fabs(dy);
  const int steps = static_cast<int>(std::ceil(std::max(adx, ady)));
  if (steps <= 0) return;

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
      PutPixel(img, x, y, r, g, bb);
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
    PpmImage out = imgSS;
    if (ssaa > 1) {
      PpmImage ds;
      DownsampleBox(imgSS, ssaa, ds);
      out = std::move(ds);
    }
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
    r = ToU8(static_cast<float>(r) * (1.0f - a) + static_cast<float>(shade.bgR) * a);
    g = ToU8(static_cast<float>(g) * (1.0f - a) + static_cast<float>(shade.bgG) * a);
    b = ToU8(static_cast<float>(b) * (1.0f - a) + static_cast<float>(shade.bgB) * a);
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
    // Per-quad color (flat) with lambert shading.
    const MeshC4 shaded = shadeColor(q.color, q.n, 1.0f);
    std::uint8_t baseR = shaded.r;
    std::uint8_t baseG = shaded.g;
    std::uint8_t baseB = shaded.b;

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

    auto rasterTri = [&](SVtx v0, SVtx v1, SVtx v2) {
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
    rasterTri(sa, sb, sc);
    rasterTri(sa, sc, sd);

    if (cfg.drawOutlines) {
      const SVtx e0 = sa;
      const SVtx e1 = sb;
      const SVtx e2 = sc;
      const SVtx e3 = sd;
      DrawLineZTest(imgSS, zbuf, e0, e1, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineDepthEps);
      DrawLineZTest(imgSS, zbuf, e1, e2, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineDepthEps);
      DrawLineZTest(imgSS, zbuf, e2, e3, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineDepthEps);
      DrawLineZTest(imgSS, zbuf, e3, e0, cfg.outlineR, cfg.outlineG, cfg.outlineB, cfg.outlineDepthEps);
    }
  }

  if (ssaa > 1) {
    PpmImage out;
    DownsampleBox(imgSS, ssaa, out);
    return out;
  }

  return imgSS;
}

} // namespace isocity
