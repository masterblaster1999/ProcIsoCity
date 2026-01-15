#include "isocity/OrganicMaterial.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace isocity {

namespace {

inline float Clamp01(float v)
{
  return (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v);
}

inline float Lerp(float a, float b, float t)
{
  return a + (b - a) * t;
}

inline std::uint32_t Hash32(std::uint32_t x)
{
  // Tiny integer hash (public domain style).
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

inline std::uint32_t Hash2(std::uint32_t a, std::uint32_t b)
{
  return Hash32(a ^ (Hash32(b) + 0x9e3779b9U + (a << 6) + (a >> 2)));
}

} // namespace

OrganicMaterial::~OrganicMaterial()
{
  shutdown();
}

void OrganicMaterial::init(int tileW, int tileH, std::uint32_t seed)
{
  shutdown();

  m_tileW = tileW;
  m_tileH = tileH;
  m_seed = seed;

  m_u.assign(kSimSize * kSimSize, 1.0f);
  m_v.assign(kSimSize * kSimSize, 0.0f);
  m_u2.assign(kSimSize * kSimSize, 1.0f);
  m_v2.assign(kSimSize * kSimSize, 0.0f);

  initNeighbors();
  seedBlobs(seed);

  // Create the variant textures (diamond-shaped decals).
  for (int i = 0; i < kVariants; ++i) {
    auto& v = m_var[static_cast<size_t>(i)];

    v.pixels.assign(static_cast<size_t>(m_tileW * m_tileH), Color{0, 0, 0, 0});

    Image img = GenImageColor(m_tileW, m_tileH, Color{0, 0, 0, 0});
    v.tex = LoadTextureFromImage(img);
    UnloadImage(img);
    SetTextureFilter(v.tex, TEXTURE_FILTER_BILINEAR);

    // Deterministic offsets/rotations.
    const std::uint32_t h = Hash2(seed, static_cast<std::uint32_t>(i));
    v.ofsX = static_cast<int>(h & 0xffU);
    v.ofsY = static_cast<int>((h >> 8) & 0xffU);
    v.rot = static_cast<int>((h >> 16) & 3U);
  }

  m_timeAccum = 0.0f;
  m_injectAccum = 0.0f;
  m_stepCounter = 0;

  // Run a short warm-up so the material looks interesting immediately.
  warmStart(Settings{});

  // Build initial textures.
  const Settings s{};
  for (int i = 0; i < kVariants; ++i) {
    rebuildVariantTexture(i, s, 0.0f);
    UpdateTexture(m_var[static_cast<size_t>(i)].tex, m_var[static_cast<size_t>(i)].pixels.data());
  }

  m_ready = true;
}

void OrganicMaterial::shutdown()
{
  if (m_ready) {
    for (auto& v : m_var) {
      if (v.tex.id != 0) {
        UnloadTexture(v.tex);
        v.tex = Texture2D{};
      }
      v.pixels.clear();
      v.pixels.shrink_to_fit();
    }
  }

  m_ready = false;
  m_tileW = 0;
  m_tileH = 0;
  m_seed = 0;
  m_u.clear();
  m_v.clear();
  m_u2.clear();
  m_v2.clear();
}

void OrganicMaterial::reset(std::uint32_t seed)
{
  m_seed = seed;
  std::fill(m_u.begin(), m_u.end(), 1.0f);
  std::fill(m_v.begin(), m_v.end(), 0.0f);
  seedBlobs(seed);
  m_injectAccum = 0.0f;
  m_stepCounter = 0;

  // Re-randomize variant offsets/rotations.
  for (int i = 0; i < kVariants; ++i) {
    auto& v = m_var[static_cast<size_t>(i)];
    const std::uint32_t h = Hash2(seed, static_cast<std::uint32_t>(i));
    v.ofsX = static_cast<int>(h & 0xffU);
    v.ofsY = static_cast<int>((h >> 8) & 0xffU);
    v.rot = static_cast<int>((h >> 16) & 3U);
  }
}

const Texture2D& OrganicMaterial::variantTex(int idx) const
{
  const int i = (idx < 0) ? 0 : (idx >= kVariants ? (kVariants - 1) : idx);
  return m_var[static_cast<size_t>(i)].tex;
}

void OrganicMaterial::initNeighbors()
{
  for (int i = 0; i < kSimSize; ++i) {
    m_xL[static_cast<size_t>(i)] = (i == 0) ? (kSimSize - 1) : (i - 1);
    m_xR[static_cast<size_t>(i)] = (i == (kSimSize - 1)) ? 0 : (i + 1);
    m_yU[static_cast<size_t>(i)] = (i == 0) ? (kSimSize - 1) : (i - 1);
    m_yD[static_cast<size_t>(i)] = (i == (kSimSize - 1)) ? 0 : (i + 1);
  }
}

void OrganicMaterial::seedBlobs(std::uint32_t seed)
{
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist(0, kSimSize - 1);

  const int blobs = 18;
  for (int b = 0; b < blobs; ++b) {
    const int cx = dist(rng);
    const int cy = dist(rng);
    const int r = 5 + (dist(rng) % 10);

    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) continue;
        const int x = (cx + dx + kSimSize) & (kSimSize - 1);
        const int y = (cy + dy + kSimSize) & (kSimSize - 1);
        const int idx = x + y * kSimSize;
        m_v[static_cast<size_t>(idx)] = 1.0f;
        m_u[static_cast<size_t>(idx)] = 0.0f;
      }
    }
  }
}

void OrganicMaterial::injectBlob(std::uint32_t seed)
{
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist(0, kSimSize - 1);

  const int cx = dist(rng);
  const int cy = dist(rng);
  const int r = 3 + (dist(rng) % 6);

  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy > r * r) continue;
      const int x = (cx + dx + kSimSize) & (kSimSize - 1);
      const int y = (cy + dy + kSimSize) & (kSimSize - 1);
      const int idx = x + y * kSimSize;
      // Add a small amount of V; don't fully overwrite existing patterns.
      const float vv = m_v[static_cast<size_t>(idx)];
      m_v[static_cast<size_t>(idx)] = std::max(vv, 0.85f);
      m_u[static_cast<size_t>(idx)] = std::min(m_u[static_cast<size_t>(idx)], 0.35f);
    }
  }
}

void OrganicMaterial::warmStart(const Settings& s)
{
  // "Burn in" the sim for a bit so the first frame isn't empty.
  const int warmSteps = 220;
  for (int i = 0; i < warmSteps; ++i) {
    stepOnce(s);
  }
}

void OrganicMaterial::stepOnce(const Settings& s)
{
  // Common RD Laplacian stencil weights (9-sample):
  //  -1.0 center, 0.2 direct neighbors, 0.05 diagonals.
  // This gives stable, pleasant "organic" blobs.
  constexpr float wC = -1.0f;
  constexpr float wN = 0.20f;
  constexpr float wD = 0.05f;

  for (int y = 0; y < kSimSize; ++y) {
    const int yU = m_yU[static_cast<size_t>(y)];
    const int yD = m_yD[static_cast<size_t>(y)];
    const int row = y * kSimSize;
    const int rowU = yU * kSimSize;
    const int rowD = yD * kSimSize;

    for (int x = 0; x < kSimSize; ++x) {
      const int xL = m_xL[static_cast<size_t>(x)];
      const int xR = m_xR[static_cast<size_t>(x)];

      const int idx = row + x;

      const float u = m_u[static_cast<size_t>(idx)];
      const float v = m_v[static_cast<size_t>(idx)];

      const float lapU =
          wC * u +
          wN * (m_u[static_cast<size_t>(row + xL)] + m_u[static_cast<size_t>(row + xR)] +
                m_u[static_cast<size_t>(rowU + x)] + m_u[static_cast<size_t>(rowD + x)]) +
          wD * (m_u[static_cast<size_t>(rowU + xL)] + m_u[static_cast<size_t>(rowU + xR)] +
                m_u[static_cast<size_t>(rowD + xL)] + m_u[static_cast<size_t>(rowD + xR)]);

      const float lapV =
          wC * v +
          wN * (m_v[static_cast<size_t>(row + xL)] + m_v[static_cast<size_t>(row + xR)] +
                m_v[static_cast<size_t>(rowU + x)] + m_v[static_cast<size_t>(rowD + x)]) +
          wD * (m_v[static_cast<size_t>(rowU + xL)] + m_v[static_cast<size_t>(rowU + xR)] +
                m_v[static_cast<size_t>(rowD + xL)] + m_v[static_cast<size_t>(rowD + xR)]);

      const float uvv = u * v * v;
      const float du = s.diffusionU * lapU - uvv + s.feed * (1.0f - u);
      const float dv = s.diffusionV * lapV + uvv - (s.feed + s.kill) * v;

      const float uN = Clamp01(u + du);
      const float vN = Clamp01(v + dv);

      m_u2[static_cast<size_t>(idx)] = uN;
      m_v2[static_cast<size_t>(idx)] = vN;
    }
  }

  m_u.swap(m_u2);
  m_v.swap(m_v2);
  ++m_stepCounter;
}

float OrganicMaterial::sampleV(float fx, float fy) const
{
  // Periodic wrap in power-of-two domain.
  const int x0 = static_cast<int>(std::floor(fx)) & (kSimSize - 1);
  const int y0 = static_cast<int>(std::floor(fy)) & (kSimSize - 1);
  const int x1 = (x0 + 1) & (kSimSize - 1);
  const int y1 = (y0 + 1) & (kSimSize - 1);

  const float tx = fx - std::floor(fx);
  const float ty = fy - std::floor(fy);

  const float a = m_v[static_cast<size_t>(x0 + y0 * kSimSize)];
  const float b = m_v[static_cast<size_t>(x1 + y0 * kSimSize)];
  const float c = m_v[static_cast<size_t>(x0 + y1 * kSimSize)];
  const float d = m_v[static_cast<size_t>(x1 + y1 * kSimSize)];

  return Lerp(Lerp(a, b, tx), Lerp(c, d, tx), ty);
}

void OrganicMaterial::rebuildVariantTexture(int idx, const Settings& s, float timeSec)
{
  auto& v = m_var[static_cast<size_t>(idx)];

  // A gentle drift so the material feels "alive" even when the RD settles.
  const float drift = timeSec * 7.0f * s.speed;

  for (int py = 0; py < m_tileH; ++py) {
    for (int px = 0; px < m_tileW; ++px) {
      const int outIdx = px + py * m_tileW;

      // Normalized diamond coordinates.
      const float nx = (static_cast<float>(px) + 0.5f - static_cast<float>(m_tileW) * 0.5f) /
                       (static_cast<float>(m_tileW) * 0.5f);
      const float ny = (static_cast<float>(py) + 0.5f - static_cast<float>(m_tileH) * 0.5f) /
                       (static_cast<float>(m_tileH) * 0.5f);

      const float manhattan = std::fabs(nx) + std::fabs(ny);
      if (manhattan > 1.0f) {
        v.pixels[static_cast<size_t>(outIdx)] = Color{0, 0, 0, 0};
        continue;
      }

      // Map diamond space into sim UV space.
      float u = nx * 0.5f + 0.5f;
      float w = ny * 0.5f + 0.5f;

      // Optional 90deg rotations for variety.
      switch (v.rot & 3) {
      default:
      case 0: break;
      case 1: {
        const float tmp = u;
        u = 1.0f - w;
        w = tmp;
      } break;
      case 2: {
        u = 1.0f - u;
        w = 1.0f - w;
      } break;
      case 3: {
        const float tmp = u;
        u = w;
        w = 1.0f - tmp;
      } break;
      }

      // Sample RD field.
      const float fx = (u * static_cast<float>(kSimSize) * s.patternScale) +
                       static_cast<float>(v.ofsX) + drift;
      const float fy = (w * static_cast<float>(kSimSize) * s.patternScale) +
                       static_cast<float>(v.ofsY) + drift * 0.73f;

      float val = sampleV(fx, fy);

      // Contrast and edge shaping.
      val = Clamp01((val - 0.12f) / 0.70f);
      val = std::pow(val, 0.80f);

      // Fade toward the diamond edge to avoid hard seams.
      const float edge = Clamp01(1.0f - manhattan);
      const float a = Clamp01(val * (0.25f + 0.75f * edge));

      const unsigned char g = static_cast<unsigned char>(std::clamp(a * 255.0f, 0.0f, 255.0f));
      v.pixels[static_cast<size_t>(outIdx)] = Color{g, g, g, g};
    }
  }
}

void OrganicMaterial::update(float dtSec, float timeSec, const Settings& s)
{
  if (!m_ready) return;

  // Convert dt into an approximate 60Hz step count.
  if (dtSec < 0.0f || !std::isfinite(dtSec)) dtSec = 0.0f;
  dtSec = std::min(dtSec, 0.25f);

  const float scaled = dtSec * 60.0f * std::max(0.0f, s.speed);
  const int steps = std::clamp(static_cast<int>(std::round(scaled * static_cast<float>(std::max(1, s.stepsPerFrame)))),
                               1, 64);

  for (int i = 0; i < steps; ++i) {
    stepOnce(s);
  }

  // Periodically inject a small blob so the texture keeps evolving.
  m_injectAccum += dtSec * std::max(0.0f, s.speed);
  if (m_injectAccum >= 2.75f) {
    m_injectAccum = 0.0f;
    const std::uint32_t h = Hash2(m_seed, static_cast<std::uint32_t>(m_stepCounter));
    injectBlob(h);
  }

  // Refresh decal textures.
  for (int i = 0; i < kVariants; ++i) {
    rebuildVariantTexture(i, s, timeSec);
    UpdateTexture(m_var[static_cast<size_t>(i)].tex, m_var[static_cast<size_t>(i)].pixels.data());
  }
}

} // namespace isocity
