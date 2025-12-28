#include "isocity/Renderer.hpp"

#include "isocity/Pathfinding.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <string>

namespace isocity {

namespace {

inline unsigned char ClampU8(int v) { return static_cast<unsigned char>(std::clamp(v, 0, 255)); }

inline Color Mul(Color c, float b)
{
  const int r = static_cast<int>(std::round(static_cast<float>(c.r) * b));
  const int g = static_cast<int>(std::round(static_cast<float>(c.g) * b));
  const int bl = static_cast<int>(std::round(static_cast<float>(c.b) * b));
  return Color{ClampU8(r), ClampU8(g), ClampU8(bl), c.a};
}

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

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
struct TileRect {
  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
};

// Compute a conservative tile-coordinate rectangle that covers the current camera viewport.
// This is used to avoid drawing off-screen tiles (big win when panning/zooming on larger maps).
TileRect ComputeVisibleTileRect(const Camera2D& camera, int screenW, int screenH, int mapW, int mapH, float tileW,
                                float tileH)
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
  minWY -= tileH;
  maxWY += tileH;

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

Renderer::Renderer(int tileW, int tileH, std::uint64_t seed)
    : m_tileW(tileW)
    , m_tileH(tileH)
{
  rebuildTextures(seed);
}

Renderer::~Renderer() { unloadTextures(); }

void Renderer::unloadTextures()
{
  for (auto& t : m_terrainTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
  for (auto& t : m_overlayTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
  for (auto& t : m_roadTex) {
    if (t.id != 0) UnloadTexture(t);
    t = Texture2D{};
  }
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

Texture2D& Renderer::terrain(Terrain t) { return m_terrainTex[static_cast<std::size_t>(TerrainIndex(t))]; }

Texture2D& Renderer::overlay(Overlay o) { return m_overlayTex[static_cast<std::size_t>(OverlayIndex(o))]; }

Texture2D& Renderer::road(std::uint8_t mask) { return m_roadTex[static_cast<std::size_t>(mask & 0x0Fu)]; }

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

  // --- Terrain ---
  // Water
  m_terrainTex[0] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xA1B2C3D4u);
    const float n = (Frac01(h) - 0.5f) * 0.10f;

    // Subtle diagonal waves (purely procedural).
    const float waves = 0.06f * std::sin((x * 0.35f + y * 0.70f));
    const float b = 1.0f + n + waves;

    Color base = Color{40, 95, 210, 255};
    base = Mul(base, b);

    // Slightly fade edges to reduce harsh tiling.
    base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 4.0f, 0.0f, 1.0f));
    return base;
  });

  // Sand
  m_terrainTex[1] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0xBEEFBEEFu);
    const float n = (Frac01(h) - 0.5f) * 0.18f;

    Color base = Color{200, 186, 135, 255};
    base = Mul(base, 1.0f + n);

    // Grain speckles.
    if ((h & 0x1Fu) == 0x1Fu) base = Mul(base, 0.85f);

    base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
    return base;
  });

  // Grass
  m_terrainTex[2] = MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
    const std::uint32_t h = HashCoords32(x, y, s ^ 0x12345678u);
    const float n = (Frac01(h) - 0.5f) * 0.22f;

    Color base = Color{70, 170, 90, 255};
    base = Mul(base, 1.0f + n);

    // Tiny darker "blades" of grass.
    if ((h & 0x7Fu) == 0x3Fu) base = Mul(base, 0.78f);

    base.a = static_cast<unsigned char>(255.0f * std::clamp(d.edge * 6.0f, 0.0f, 1.0f));
    return base;
  });

  // --- Overlays ---
  // None: keep as an empty texture (id=0), we won't draw it.
  m_overlayTex[0] = Texture2D{};

  // Road: auto-tiling variants (mask stored in tile.variation low bits).
  // We keep m_overlayTex[1] empty; roads are drawn from m_roadTex[0..15].
  m_overlayTex[1] = Texture2D{};

  auto makeRoadVariant = [&](std::uint8_t mask) -> Texture2D {
    return MakeDiamondTexture(m_tileW, m_tileH, [&](int x, int y, const DiamondParams& d) -> Color {
      const std::uint32_t h = HashCoords32(x, y, s ^ 0x0F0F0F0Fu ^ (static_cast<std::uint32_t>(mask) * 0x9E3779B9u));
      const float n = (Frac01(h) - 0.5f) * 0.08f;

      const float px = d.nx;
      const float py = d.ny;

      // Signed distance to a union of "capsule" segments from center to each connected edge.
      const float roadW = 0.14f;
      const float centerR = roadW * 1.10f;
      float sd = std::sqrt(px * px + py * py) - centerR;

      float bestSegDist = 1.0e9f;
      float bestSegT = 0.0f;

      auto consider = [&](bool enabled, float ex, float ey) {
        if (!enabled) return;
        float tproj = 0.0f;
        const float dist = DistPointSegment(px, py, 0.0f, 0.0f, ex, ey, tproj);
        sd = std::min(sd, dist - roadW);
        if (dist < bestSegDist) {
          bestSegDist = dist;
          bestSegT = tproj;
        }
      };

      // Bit layout matches World::computeRoadMask().
      consider((mask & 0x01u) != 0u, 0.5f, -0.5f); // up-right
      consider((mask & 0x02u) != 0u, 0.5f, 0.5f);  // down-right
      consider((mask & 0x04u) != 0u, -0.5f, 0.5f); // down-left
      consider((mask & 0x08u) != 0u, -0.5f, -0.5f); // up-left

      // Outside the road shape.
      if (sd > 0.0f) return Color{0, 0, 0, 0};

      // Asphalt base.
      Color base = Color{90, 90, 95, 230};
      base = Mul(base, 1.0f + n);

      // Dashed centerline on the closest segment (avoid the intersection blob).
      const float centerDist = std::sqrt(px * px + py * py);
      if (bestSegDist < (roadW * 0.25f) && centerDist > centerR * 0.6f) {
        const int dash = static_cast<int>(std::floor(bestSegT * 10.0f + static_cast<float>(mask) * 0.15f));
        if ((dash & 1) == 0) {
          base = Color{220, 220, 210, 240};
        }
      }

      // Soft edges.
      const float edgeSoft = 0.05f;
      const float a = std::clamp((-sd) / edgeSoft, 0.0f, 1.0f);
      base.a = static_cast<unsigned char>(static_cast<float>(base.a) * a);
      return base;
    });
  };

  for (int i = 0; i < 16; ++i) {
    m_roadTex[static_cast<std::size_t>(i)] = makeRoadVariant(static_cast<std::uint8_t>(i));
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
}

void Renderer::drawWorld(const World& world, const Camera2D& camera, float timeSec, std::optional<Point> hovered,
                         bool drawGrid, int brushRadius, std::optional<Point> selected,
                         const std::vector<Point>* highlightPath, const std::vector<std::uint8_t>* roadToEdgeMask,
                         const std::vector<std::uint16_t>* roadTraffic, int trafficMax,
                         const std::vector<std::uint16_t>* roadGoodsTraffic, int goodsMax,
                         const std::vector<std::uint8_t>* commercialGoodsFill)
{
  (void)timeSec;

  BeginMode2D(camera);

  const int w = world.width();
  const int h = world.height();

  const bool showOutside = (roadToEdgeMask && w > 0 && h > 0 &&
                            roadToEdgeMask->size() ==
                                static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showTraffic = (roadTraffic && trafficMax > 0 && w > 0 && h > 0 &&
                            roadTraffic->size() ==
                                static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showGoods = (roadGoodsTraffic && goodsMax > 0 && w > 0 && h > 0 &&
                         roadGoodsTraffic->size() ==
                             static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  const bool showCommercialGoods = (commercialGoodsFill && w > 0 && h > 0 &&
                                  commercialGoodsFill->size() ==
                                      static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  // Compute a conservative visible tile range based on the current camera view.
  const TileRect vis = ComputeVisibleTileRect(camera, GetScreenWidth(), GetScreenHeight(), w, h,
                                              static_cast<float>(m_tileW), static_cast<float>(m_tileH));

  // Draw order: diagonals by (x+y) so nearer tiles draw last.
  const int minSum = vis.minX + vis.minY;
  const int maxSum = vis.maxX + vis.maxY;
  for (int sum = minSum; sum <= maxSum; ++sum) {
    const int x0 = std::max(vis.minX, sum - vis.maxY);
    const int x1 = std::min(vis.maxX, sum - vis.minY);
    for (int x = x0; x <= x1; ++x) {
      const int y = sum - x;
      if (y < vis.minY || y > vis.maxY) continue;

      const Tile& t = world.at(x, y);

      const Vector2 center = TileToWorldCenter(x, y, static_cast<float>(m_tileW), static_cast<float>(m_tileH));
      const Rectangle dst = Rectangle{center.x - m_tileW * 0.5f, center.y - m_tileH * 0.5f,
                                      static_cast<float>(m_tileW), static_cast<float>(m_tileH)};
      const Rectangle src = Rectangle{0, 0, static_cast<float>(m_tileW), static_cast<float>(m_tileH)};

      // Per-tile lighting based on height + variation.
      const float v = (static_cast<float>(t.variation) / 255.0f - 0.5f) * 0.10f;
      float brightness = 0.85f + t.height * 0.30f + v;

      if (t.terrain == Terrain::Water) {
        // Slight animated shimmer for water (no textures-from-disk, still procedural).
        brightness += 0.05f * std::sin(timeSec * 2.0f + (x + y) * 0.4f);
      }

      // Draw terrain
      DrawTexturePro(terrain(t.terrain), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));

      // Draw overlay
      if (t.overlay == Overlay::Road) {
        const std::uint8_t mask = static_cast<std::uint8_t>(t.variation & 0x0Fu);

        Color tint = BrightnessTint(brightness);

        bool disconnected = false;

        if (showOutside) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < roadToEdgeMask->size() && (*roadToEdgeMask)[idx] == 0) {
            // Disconnected road component: tint red so it's obvious why zones may not function.
            disconnected = true;
            tint.g = ClampU8(static_cast<int>(std::round(static_cast<float>(tint.g) * 0.35f)));
            tint.b = ClampU8(static_cast<int>(std::round(static_cast<float>(tint.b) * 0.35f)));
            tint.r = ClampU8(static_cast<int>(
                std::round(std::min(255.0f, static_cast<float>(tint.r) * 1.10f + 20.0f))));
          }
        }

        if (!disconnected && showTraffic) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < roadTraffic->size()) {
            const int tv = static_cast<int>((*roadTraffic)[idx]);
            if (tv > 0) {
              const float tnorm = std::clamp(static_cast<float>(tv) / std::max(1.0f, static_cast<float>(trafficMax)),
                                             0.0f, 1.0f);
              // Emphasize low flows while keeping very busy roads distinct.
              const float s = std::pow(tnorm, 0.35f);

              const float rf = std::min(255.0f, static_cast<float>(tint.r) * (1.0f + 0.15f * s) + 85.0f * s);
              const float gf = static_cast<float>(tint.g) * (1.0f - 0.70f * s);
              const float bf = static_cast<float>(tint.b) * (1.0f - 0.70f * s);

              tint.r = ClampU8(static_cast<int>(std::round(rf)));
              tint.g = ClampU8(static_cast<int>(std::round(gf)));
              tint.b = ClampU8(static_cast<int>(std::round(bf)));
            }
          }
        }


        if (!disconnected && showGoods) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < roadGoodsTraffic->size()) {
            const int gv = static_cast<int>((*roadGoodsTraffic)[idx]);
            if (gv > 0) {
              const float gnorm = std::clamp(static_cast<float>(gv) / std::max(1.0f, static_cast<float>(goodsMax)),
                                            0.0f, 1.0f);
              const float s = std::pow(gnorm, 0.35f);

              const float bf = std::min(255.0f, static_cast<float>(tint.b) * (1.0f + 0.15f * s) + 85.0f * s);
              const float gf = static_cast<float>(tint.g) * (1.0f - 0.70f * s);
              const float rf = static_cast<float>(tint.r) * (1.0f - 0.70f * s);

              tint.b = ClampU8(static_cast<int>(std::round(bf)));
              tint.g = ClampU8(static_cast<int>(std::round(gf)));
              tint.r = ClampU8(static_cast<int>(std::round(rf)));
            }
          }
        }

        DrawTexturePro(road(mask), src, dst, Vector2{0, 0}, 0.0f, tint);
      } else if (t.overlay != Overlay::None) {
        Color tint = BrightnessTint(brightness);

        if (showOutside) {
          const bool isZoneOrPark = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial ||
                                     t.overlay == Overlay::Industrial || t.overlay == Overlay::Park);

          if (isZoneOrPark && !HasAdjacentRoadConnectedToEdge(world, *roadToEdgeMask, x, y)) {
            // Dim zones/parks that are not adjacent to an outside-connected road.
            tint = Mul(tint, 0.55f);
          }
        }


        if (showCommercialGoods && t.overlay == Overlay::Commercial) {
          const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(x);
          if (idx < commercialGoodsFill->size()) {
            const float ratio = static_cast<float>((*commercialGoodsFill)[idx]) / 255.0f;
            const float miss = std::clamp(1.0f - ratio, 0.0f, 1.0f);
            if (miss > 0.01f) {
              const float rf = std::min(255.0f, static_cast<float>(tint.r) * (1.0f + 0.10f * miss) + 70.0f * miss);
              const float gf = static_cast<float>(tint.g) * (1.0f - 0.55f * miss);
              const float bf = static_cast<float>(tint.b) * (1.0f - 0.55f * miss);
              tint.r = ClampU8(static_cast<int>(std::round(rf)));
              tint.g = ClampU8(static_cast<int>(std::round(gf)));
              tint.b = ClampU8(static_cast<int>(std::round(bf)));
            }
          }
        }

        DrawTexturePro(overlay(t.overlay), src, dst, Vector2{0, 0}, 0.0f, tint);
      }

      if (drawGrid) {
        Vector2 corners[4];
        TileDiamondCorners(center, static_cast<float>(m_tileW), static_cast<float>(m_tileH), corners);
        const Color gc = Color{0, 0, 0, 40};
        DrawLineV(corners[0], corners[1], gc);
        DrawLineV(corners[1], corners[2], gc);
        DrawLineV(corners[2], corners[3], gc);
        DrawLineV(corners[3], corners[0], gc);
      }
      // Zone indicators: when zoomed in, draw small pips for building level (1..3)
      // and a tiny occupancy bar (residents/workers vs capacity).
      const bool isZone =
          (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
      const float tileScreenW = static_cast<float>(m_tileW) * camera.zoom;
      if (isZone && tileScreenW >= 28.0f) {
        const float invZoom = 1.0f / std::max(0.001f, camera.zoom);

        const int lvl = std::clamp<int>(static_cast<int>(t.level), 0, 3);

        int cap = 0;
        if (t.overlay == Overlay::Residential) cap = 10 * lvl;
        if (t.overlay == Overlay::Commercial) cap = 8 * lvl;
        if (t.overlay == Overlay::Industrial) cap = 12 * lvl;

        const float ratio = (cap > 0)
                                ? std::clamp(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f)
                                : 0.0f;

        // Anchor near the tile center.
        const float y0 = center.y - static_cast<float>(m_tileH) * 0.06f;

        // Pips: 3 small squares filled according to level.
        const float pip = 5.0f * invZoom;
        const float gap = 2.0f * invZoom;
        const float groupW = pip * 3.0f + gap * 2.0f;
        const float x0 = center.x - groupW * 0.5f;

        for (int i = 0; i < 3; ++i) {
          Rectangle r{x0 + static_cast<float>(i) * (pip + gap), y0, pip, pip};
          DrawRectangleRec(r, Color{0, 0, 0, 120});
          DrawRectangleLinesEx(r, 1.0f * invZoom, Color{255, 255, 255, 70});
          if (i < lvl) {
            Rectangle f{r.x + 1.0f * invZoom, r.y + 1.0f * invZoom, r.width - 2.0f * invZoom,
                        r.height - 2.0f * invZoom};
            DrawRectangleRec(f, Color{255, 255, 255, 170});
          }
        }

        // Occupancy bar.
        const float barW = 22.0f * invZoom;
        const float barH = 3.0f * invZoom;
        const float barX = center.x - barW * 0.5f;
        const float barY = y0 + pip + 2.0f * invZoom;

        Rectangle bg{barX, barY, barW, barH};
        DrawRectangleRec(bg, Color{0, 0, 0, 120});

        Rectangle fg{barX, barY, barW * ratio, barH};
        DrawRectangleRec(fg, Color{255, 255, 255, 170});
        DrawRectangleLinesEx(bg, 1.0f * invZoom, Color{255, 255, 255, 60});
      }
    }
  }

  // Highlight helpers.
  const float thickness = 2.0f / std::max(0.25f, camera.zoom);

  auto drawOutline = [&](int tx, int ty, Color c) {
    if (!world.inBounds(tx, ty)) return;
    const Vector2 center = TileToWorldCenter(tx, ty, static_cast<float>(m_tileW), static_cast<float>(m_tileH));
    Vector2 corners[4];
    TileDiamondCorners(center, static_cast<float>(m_tileW), static_cast<float>(m_tileH), corners);

    DrawLineEx(corners[0], corners[1], thickness, c);
    DrawLineEx(corners[1], corners[2], thickness, c);
    DrawLineEx(corners[2], corners[3], thickness, c);
    DrawLineEx(corners[3], corners[0], thickness, c);
  };

  // Optional debug highlight: path to edge (inspect tool).
  if (highlightPath && !highlightPath->empty()) {
    const Color pc = Color{255, 215, 0, 110};
    for (const Point& p : *highlightPath) {
      drawOutline(p.x, p.y, pc);
    }
  }

  // Selected tile highlight.
  if (selected && world.inBounds(selected->x, selected->y)) {
    drawOutline(selected->x, selected->y, Color{255, 215, 0, 220});
  }

  // Hover highlight (and brush outline)
  if (hovered && world.inBounds(hovered->x, hovered->y)) {
    const int cx = hovered->x;
    const int cy = hovered->y;

    const int r = std::max(0, brushRadius);
    if (r > 0) {
      const Color bc = Color{255, 255, 255, 70};
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::abs(dx) + std::abs(dy) > r) continue; // diamond brush
          const int tx = cx + dx;
          const int ty = cy + dy;
          drawOutline(tx, ty, bc);
        }
      }
    }

    // Center tile gets a brighter outline.
    drawOutline(cx, cy, Color{255, 255, 255, 180});
  }

  EndMode2D();
}

void Renderer::drawHUD(const World& world, Tool tool, std::optional<Point> hovered, int screenW, int screenH,
                       bool showHelp, int brushRadius, int undoCount, int redoCount, bool simPaused, float simSpeed,
                       int saveSlot, const char* inspectInfo)
{
  const Stats& s = world.stats();

  // HUD panel
  const int pad = 12;
  const int panelW = 420;
  const int extraLine = (inspectInfo && inspectInfo[0] != '\0') ? 1 : 0;
  // +2 extra lines for traffic/commute + goods/logistics stats.
  const int panelH = (showHelp ? 360 : 228) + extraLine * 22;

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
      std::snprintf(buf, sizeof(buf), "Traffic: %d commute (unreach %d)  avg %.1f  cong %.0f%%", s.commuters,
                    s.commutersUnreachable, static_cast<double>(s.avgCommute),
                    static_cast<double>(s.trafficCongestion * 100.0f));
    } else {
      std::snprintf(buf, sizeof(buf), "Traffic: %d commute  avg %.1f  cong %.0f%%", s.commuters,
                    static_cast<double>(s.avgCommute), static_cast<double>(s.trafficCongestion * 100.0f));
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

  std::snprintf(buf, sizeof(buf), "Roads: %d    Parks: %d    Tool: %s    Brush: %d", s.roads, s.parks, ToString(tool), brushRadius);
  line(buf);

  std::snprintf(buf, sizeof(buf), "Undo: %d    Redo: %d    Slot: %d", undoCount, redoCount, saveSlot);
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
    std::snprintf(buf, sizeof(buf), "Hover: (%d,%d)  %s + %s  h=%.2f  lvl=%d  occ=%d", hovered->x, hovered->y,
                  ToString(t.terrain), ToString(t.overlay), static_cast<double>(t.height), t.level, t.occupants);
    DrawText(buf, pad + 10, y + 6, 16, Color{220, 220, 220, 255});
    y += 26;
  }

  if (inspectInfo && inspectInfo[0] != '\0') {
    DrawText(inspectInfo, pad + 10, y + 6, 16, Color{230, 230, 230, 255});
    y += 26;
  }

  if (showHelp) {
    DrawText("Right drag: pan | Wheel: zoom | R regen | G grid | H help | O outside | T graph | V traffic | B goods", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("1 Road | 2 Res | 3 Com | 4 Ind | 5 Park | 0 Doze | Q Inspect", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("[/] brush | Space: pause | N: step | +/-: speed", pad + 10, y + 10, 16, Color{220, 220, 220, 255});
    y += 22;
    DrawText("F5 save | F9 load | F6 slot | Ctrl+Z undo | Ctrl+Y redo", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("Tip: re-place a zone to upgrade. Road: Shift+drag path.", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
  }

  // FPS
  const int fps = GetFPS();
  std::snprintf(buf, sizeof(buf), "FPS: %d", fps);
  DrawText(buf, screenW - 90, 12, 20, Color{255, 255, 255, 200});
}

} // namespace isocity
