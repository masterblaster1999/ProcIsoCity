#include "isocity/Renderer.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
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
                         bool drawGrid, int brushRadius)
{
  (void)timeSec;

  BeginMode2D(camera);

  const int w = world.width();
  const int h = world.height();

  // Draw order: diagonals by (x+y) so nearer tiles draw last.
  for (int sum = 0; sum < (w + h - 1); ++sum) {
    for (int x = 0; x < w; ++x) {
      const int y = sum - x;
      if (y < 0 || y >= h) continue;

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
        DrawTexturePro(road(mask), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));
      } else if (t.overlay != Overlay::None) {
        DrawTexturePro(overlay(t.overlay), src, dst, Vector2{0, 0}, 0.0f, BrightnessTint(brightness));
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
    }
  }

  // Hover highlight (and brush outline)
  if (hovered && world.inBounds(hovered->x, hovered->y)) {
    const float thickness = 2.0f / std::max(0.25f, camera.zoom);

    auto drawOutline = [&](int tx, int ty, Color c) {
      const Vector2 center = TileToWorldCenter(tx, ty, static_cast<float>(m_tileW), static_cast<float>(m_tileH));
      Vector2 corners[4];
      TileDiamondCorners(center, static_cast<float>(m_tileW), static_cast<float>(m_tileH), corners);

      DrawLineEx(corners[0], corners[1], thickness, c);
      DrawLineEx(corners[1], corners[2], thickness, c);
      DrawLineEx(corners[2], corners[3], thickness, c);
      DrawLineEx(corners[3], corners[0], thickness, c);
    };

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
          if (!world.inBounds(tx, ty)) continue;
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
                       bool showHelp, int brushRadius, int undoCount, int redoCount)
{
  const Stats& s = world.stats();

  // HUD panel
  const int pad = 12;
  const int panelW = 420;
  const int panelH = showHelp ? 294 : 162;

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

  std::snprintf(buf, sizeof(buf), "Pop: %d / %d housing    Jobs: %d / %d cap", s.population, s.housingCapacity,
                s.employed, s.jobsCapacity);
  line(buf);

  std::snprintf(buf, sizeof(buf), "Roads: %d    Parks: %d    Tool: %s    Brush: %d", s.roads, s.parks, ToString(tool), brushRadius);
  line(buf);

  std::snprintf(buf, sizeof(buf), "Undo: %d    Redo: %d", undoCount, redoCount);
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

  if (showHelp) {
    DrawText("Right drag: pan | Wheel: zoom | R: regen | G: grid | H: help", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("1 Road | 2 Res | 3 Com | 4 Ind | 5 Park | 0 Doze | Q Inspect", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
    y += 22;
    DrawText("[/] brush | F5 save | F9 load", pad + 10, y + 10, 16, Color{220, 220, 220, 255});
    y += 22;
    DrawText("Ctrl+Z: undo | Ctrl+Y: redo", pad + 10, y + 10, 16, Color{220, 220, 220, 255});
    y += 22;
    DrawText("Tip: place the same zone again to upgrade (lvl 1->3).", pad + 10, y + 10, 16,
             Color{220, 220, 220, 255});
  }

  // FPS
  const int fps = GetFPS();
  std::snprintf(buf, sizeof(buf), "FPS: %d", fps);
  DrawText(buf, screenW - 90, 12, 20, Color{255, 255, 255, 200});
}

} // namespace isocity
