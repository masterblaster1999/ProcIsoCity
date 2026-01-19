#include "isocity/Ui.hpp"

#include "isocity/GfxText.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace isocity {
namespace ui {

namespace {

struct FontAtlas {
  Texture2D tex{};
  bool ready = false;

  int firstChar = 32;
  int lastChar = 126;
  int cols = 16;

  int glyphW = 5;
  int glyphH = 7;
  int pad = 1;
  int atlasScale = 3; // upscale factor for the generated atlas

  int cellWpx = 0;
  int cellHpx = 0;
  int padPx = 0;
  int glyphWpx = 0;
  int glyphHpx = 0;

  std::array<Rectangle, 128> src{};

  void unload()
  {
    if (tex.id != 0) {
      UnloadTexture(tex);
      tex = Texture2D{};
    }
    ready = false;
  }
};

struct State {
  int refCount = 0;
  bool ready = false;

  std::uint64_t seed = 0;
  Settings settings{};
  Theme theme{};

  // Immediate-mode widget state (for sliders/toggles).
  int activeWidgetId = -1;

  Texture2D noise{};
  FontAtlas font{};
  FontAtlas fontBold{};

  // Signed-distance-field font atlases.
  // These are used when fontFilterPoint=false and the SDF shader is available.
  FontAtlas fontSdf{};
  FontAtlas fontBoldSdf{};

  Shader sdfShader{};
  bool sdfShaderReady = false;
};

State g;

static std::uint64_t SplitMix64Next(std::uint64_t& state)
{
  std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

static inline unsigned char ClampU8(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<unsigned char>(v);
}

static Color WithAlpha(Color c, unsigned char a)
{
  c.a = a;
  return c;
}

static Color Darken(Color c, float factor)
{
  factor = std::clamp(factor, 0.0f, 1.0f);
  c.r = ClampU8(static_cast<int>(std::lround(static_cast<float>(c.r) * factor)));
  c.g = ClampU8(static_cast<int>(std::lround(static_cast<float>(c.g) * factor)));
  c.b = ClampU8(static_cast<int>(std::lround(static_cast<float>(c.b) * factor)));
  return c;
}

static Color Lerp(Color a, Color b, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  return Color{ClampU8(static_cast<int>(std::lround(static_cast<float>(a.r) + (static_cast<float>(b.r) - a.r) * t))),
               ClampU8(static_cast<int>(std::lround(static_cast<float>(a.g) + (static_cast<float>(b.g) - a.g) * t))),
               ClampU8(static_cast<int>(std::lround(static_cast<float>(a.b) + (static_cast<float>(b.b) - a.b) * t))),
               ClampU8(static_cast<int>(std::lround(static_cast<float>(a.a) + (static_cast<float>(b.a) - a.a) * t)))};
}

static Color Lighten(Color c, float amount)
{
  return Lerp(c, WHITE, amount);
}

static void RegenTheme(std::uint64_t seed)
{
  // Clamp settings defensively (settings can be loaded from JSON).
  Settings s = g.settings;
  s.accentHueDeg = std::clamp(s.accentHueDeg, 0.0f, 360.0f);
  s.accentSaturation = std::clamp(s.accentSaturation, 0.0f, 1.0f);
  s.accentValue = std::clamp(s.accentValue, 0.0f, 1.0f);
  s.roundness = std::clamp(s.roundness, 0.0f, 1.0f);
  s.roundSegments = std::clamp(s.roundSegments, 0, 16);
  s.noiseAlpha = std::clamp(s.noiseAlpha, 0.0f, 1.0f);
  s.noiseScale = std::clamp(s.noiseScale, 0.05f, 4.0f);
  s.headerSheenStrength = std::clamp(s.headerSheenStrength, 0.0f, 1.0f);

  float hue = s.accentHueDeg;
  if (s.accentFromSeed) {
    // Seed -> hue in [180, 330] (cool/cyber range)
    std::uint64_t st = seed ^ 0xC0FFEE1234ULL;
    hue = 180.0f + static_cast<float>(SplitMix64Next(st) % 150ULL);
  }

  Color accent = ColorFromHSV(hue, s.accentSaturation, s.accentValue);
  accent.a = 255;

  g.theme.accent = accent;
  g.theme.accentDim = WithAlpha(accent, 90);

  // Semantic accents (used for status badges, charts, warnings).
  // Keep these stable regardless of user accent hue, so green/red always
  // mean good/bad.
  const float semanticSat = std::max(0.35f, s.accentSaturation);
  const float semanticVal = std::max(0.70f, s.accentValue);
  g.theme.accentOk = ColorFromHSV(130.0f, semanticSat, semanticVal);
  g.theme.accentOk.a = 255;
  g.theme.accentBad = ColorFromHSV(5.0f, semanticSat, semanticVal);
  g.theme.accentBad.a = 255;

  // Bright/highlight accent (e.g. selected rows).
  g.theme.accentHi = WithAlpha(Lighten(accent, 0.20f), 90);

  // Back-compat alias used by some UI call-sites.
  g.theme.bad = g.theme.accentBad;


  // Derive subtle top/bottom panel colors from the accent so each seed feels different,
  // but keep things dark enough for readability.
  const float tint = 0.08f;
  g.theme.panelBgTop = Color{
      ClampU8(static_cast<int>(22 + tint * accent.r)),
      ClampU8(static_cast<int>(24 + tint * accent.g)),
      ClampU8(static_cast<int>(30 + tint * accent.b)),
      235};
  g.theme.panelBgBot = Color{
      ClampU8(static_cast<int>(10 + tint * accent.r * 0.6f)),
      ClampU8(static_cast<int>(12 + tint * accent.g * 0.6f)),
      ClampU8(static_cast<int>(16 + tint * accent.b * 0.6f)),
      235};

  // Subtle chart/table gridline color.
  g.theme.grid = WithAlpha(Lerp(g.theme.panelBgTop, g.theme.text, 0.25f), 55);

  // Apply other tunables.
  g.theme.roundness = s.roundness;
  g.theme.roundSegments = s.roundSegments;
  g.theme.noiseAlpha = s.noiseAlpha;
  g.theme.noiseScale = s.noiseScale;
  g.theme.headerSheenStrength = s.headerSheenStrength;

  // Derived, very-dim text tone (useful for disabled rows).
  g.theme.textFaint = Darken(g.theme.textDim, 0.82f);
}


static Texture2D MakeNoiseTexture(std::uint64_t seed)
{
  const int n = 64;
  Image img = GenImageColor(n, n, Color{0, 0, 0, 0});
  if (!img.data) return Texture2D{};

  Color* px = reinterpret_cast<Color*>(img.data);
  std::uint64_t st = seed ^ 0xA5A5A5A55A5A5A5AULL;

  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      const std::uint64_t r = SplitMix64Next(st);
      const int v = 185 + static_cast<int>(r & 63ULL); // 185..248
      px[y * n + x] = Color{ClampU8(v), ClampU8(v), ClampU8(v), 255};
    }
  }

  Texture2D tex = LoadTextureFromImage(img);
  UnloadImage(img);

  if (tex.id != 0) {
    SetTextureFilter(tex, TEXTURE_FILTER_BILINEAR);
    // We deliberately rely on repeat wrapping for tiling. (raylib supports SetTextureWrap.)
    SetTextureWrap(tex, TEXTURE_WRAP_REPEAT);
  }
  return tex;
}

// -----------------------------------------------------------------------------------------------
// SDF font shader
// -----------------------------------------------------------------------------------------------

static void EnsureSdfShaderLoaded()
{
  if (g.sdfShaderReady) return;

#if defined(PLATFORM_DESKTOP)
  // Raylib shader I/O names match the example shaders shipped with raylib.
  // We keep this shader minimal: alpha is reconstructed from the SDF stored in texture alpha.
  static const char* kVs =
      "#version 330\n"
      "in vec3 vertexPosition;\n"
      "in vec2 vertexTexCoord;\n"
      "in vec4 vertexColor;\n"
      "out vec2 fragTexCoord;\n"
      "out vec4 fragColor;\n"
      "uniform mat4 mvp;\n"
      "void main() {\n"
      "  fragTexCoord = vertexTexCoord;\n"
      "  fragColor = vertexColor;\n"
      "  gl_Position = mvp*vec4(vertexPosition, 1.0);\n"
      "}\n";

  static const char* kFs =
      "#version 330\n"
      "in vec2 fragTexCoord;\n"
      "in vec4 fragColor;\n"
      "out vec4 finalColor;\n"
      "uniform sampler2D texture0;\n"
      "uniform vec4 colDiffuse;\n"
      "void main() {\n"
      "  float d = texture(texture0, fragTexCoord).a;\n"
      "  float w = fwidth(d);\n"
      "  w = max(w, 0.008);\n"
      "  float a = 1.0 - smoothstep(0.5 - w, 0.5 + w, d);\n"
      "  vec4 c = fragColor*colDiffuse;\n"
      "  c.a *= a;\n"
      "  finalColor = c;\n"
      "}\n";
#else
  static const char* kVs =
      "#version 100\n"
      "attribute vec3 vertexPosition;\n"
      "attribute vec2 vertexTexCoord;\n"
      "attribute vec4 vertexColor;\n"
      "varying vec2 fragTexCoord;\n"
      "varying vec4 fragColor;\n"
      "uniform mat4 mvp;\n"
      "void main() {\n"
      "  fragTexCoord = vertexTexCoord;\n"
      "  fragColor = vertexColor;\n"
      "  gl_Position = mvp*vec4(vertexPosition, 1.0);\n"
      "}\n";

  static const char* kFs =
      "#version 100\n"
      "#ifdef GL_ES\n"
      "precision mediump float;\n"
      "#endif\n"
      "#extension GL_OES_standard_derivatives : enable\n"
      "varying vec2 fragTexCoord;\n"
      "varying vec4 fragColor;\n"
      "uniform sampler2D texture0;\n"
      "uniform vec4 colDiffuse;\n"
      "void main() {\n"
      "  float d = texture2D(texture0, fragTexCoord).a;\n"
      "  float w = fwidth(d);\n"
      "  w = max(w, 0.008);\n"
      "  float a = 1.0 - smoothstep(0.5 - w, 0.5 + w, d);\n"
      "  vec4 c = fragColor*colDiffuse;\n"
      "  c.a *= a;\n"
      "  gl_FragColor = c;\n"
      "}\n";
#endif

  g.sdfShader = LoadShaderFromMemory(kVs, kFs);
  g.sdfShaderReady = (g.sdfShader.id != 0);
}

static void UnloadSdfShader()
{
  if (g.sdfShader.id != 0) {
    UnloadShader(g.sdfShader);
    g.sdfShader = Shader{};
  }
  g.sdfShaderReady = false;
}

static FontAtlas MakeFontAtlas(bool bold, int atlasScale, bool filterPoint)
{
  FontAtlas fa;
  fa.ready = false;

  fa.atlasScale = std::clamp(atlasScale, 1, 8);

  const int count = (fa.lastChar - fa.firstChar + 1);
  const int rows = (count + fa.cols - 1) / fa.cols;

  fa.padPx = fa.pad * fa.atlasScale;
  fa.glyphWpx = fa.glyphW * fa.atlasScale;
  fa.glyphHpx = fa.glyphH * fa.atlasScale;
  fa.cellWpx = (fa.glyphW + fa.pad * 2) * fa.atlasScale;
  fa.cellHpx = (fa.glyphH + fa.pad * 2) * fa.atlasScale;

  const int atlasW = fa.cols * fa.cellWpx;
  const int atlasH = rows * fa.cellHpx;

  Image img = GenImageColor(atlasW, atlasH, Color{0, 0, 0, 0});
  if (!img.data) return fa;

  Color* px = reinterpret_cast<Color*>(img.data);

  auto setPixel = [&](int x, int y, Color c) {
    if (x < 0 || y < 0 || x >= atlasW || y >= atlasH) return;
    px[y * atlasW + x] = c;
  };

  // Per glyph, create a small boolean mask in "glyph space" (5x7) first.
  std::array<std::array<bool, 7>, 5> mask{};

  for (int c = fa.firstChar; c <= fa.lastChar; ++c) {
    // Reset mask.
    for (int gx = 0; gx < fa.glyphW; ++gx) {
      for (int gy = 0; gy < fa.glyphH; ++gy) {
        mask[gx][gy] = false;
      }
    }

    const auto& rows5x7 = gfx::GetGlyphRows5x7(static_cast<char>(c));

    for (int gy = 0; gy < fa.glyphH; ++gy) {
      const std::uint8_t row = rows5x7[static_cast<std::size_t>(gy)];
      for (int gx = 0; gx < fa.glyphW; ++gx) {
        if (row & (1u << (4 - gx))) {
          mask[gx][gy] = true;
        }
      }
    }

    if (bold) {
      // Simple dilation in glyph space: thicken right + down one pixel.
      std::array<std::array<bool, 7>, 5> out = mask;
      for (int gy = 0; gy < fa.glyphH; ++gy) {
        for (int gx = 0; gx < fa.glyphW; ++gx) {
          if (!mask[gx][gy]) continue;
          if (gx + 1 < fa.glyphW) out[gx + 1][gy] = true;
          if (gy + 1 < fa.glyphH) out[gx][gy + 1] = true;
        }
      }
      mask = out;
    }

    const int idx = c - fa.firstChar;
    const int cellX = (idx % fa.cols) * fa.cellWpx;
    const int cellY = (idx / fa.cols) * fa.cellHpx;

    const int gx0 = cellX + fa.padPx;
    const int gy0 = cellY + fa.padPx;

    // Remember the source rectangle for this character (crop to glyph area).
    if (c >= 0 && c < static_cast<int>(fa.src.size())) {
      fa.src[static_cast<std::size_t>(c)] = Rectangle{static_cast<float>(gx0), static_cast<float>(gy0),
                                                      static_cast<float>(fa.glyphWpx),
                                                      static_cast<float>(fa.glyphHpx)};
    }

    // Rasterize into the atlas at "atlasScale".
    for (int gy = 0; gy < fa.glyphH; ++gy) {
      for (int gx = 0; gx < fa.glyphW; ++gx) {
        if (!mask[gx][gy]) continue;

        const int px0 = gx0 + gx * fa.atlasScale;
        const int py0 = gy0 + gy * fa.atlasScale;
        for (int sy = 0; sy < fa.atlasScale; ++sy) {
          for (int sx = 0; sx < fa.atlasScale; ++sx) {
            setPixel(px0 + sx, py0 + sy, Color{255, 255, 255, 255});
          }
        }
      }
    }
  }

  fa.tex = LoadTextureFromImage(img);
  UnloadImage(img);

  if (fa.tex.id != 0) {
    SetTextureFilter(fa.tex, filterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(fa.tex, TEXTURE_WRAP_CLAMP);
    fa.ready = true;
  }

  return fa;
}

// Create a signed-distance-field atlas from the same 5x7 glyph set.
//
// Notes:
// - We store the distance in the *alpha* channel, normalized such that 0.5 is the glyph edge.
// - Rendering uses a tiny shader (see EnsureSdfShaderLoaded).
// - This stays fully procedural (no external font assets), but scales much better than a raw mask.
static FontAtlas MakeFontAtlasSdf(bool bold, int atlasScale)
{
  FontAtlas fa;
  fa.ready = false;

  fa.atlasScale = std::clamp(atlasScale, 1, 8);

  const int count = (fa.lastChar - fa.firstChar + 1);
  const int rows = (count + fa.cols - 1) / fa.cols;

  fa.padPx = fa.pad * fa.atlasScale;
  fa.glyphWpx = fa.glyphW * fa.atlasScale;
  fa.glyphHpx = fa.glyphH * fa.atlasScale;
  fa.cellWpx = (fa.glyphW + fa.pad * 2) * fa.atlasScale;
  fa.cellHpx = (fa.glyphH + fa.pad * 2) * fa.atlasScale;

  const int atlasW = fa.cols * fa.cellWpx;
  const int atlasH = rows * fa.cellHpx;

  // White RGB, alpha populated with distance values.
  Image img = GenImageColor(atlasW, atlasH, Color{255, 255, 255, 0});
  if (!img.data) return fa;

  Color* px = reinterpret_cast<Color*>(img.data);

  auto setPixel = [&](int x, int y, unsigned char a) {
    if (x < 0 || y < 0 || x >= atlasW || y >= atlasH) return;
    Color c = px[y * atlasW + x];
    c.a = a;
    px[y * atlasW + x] = c;
  };

  // Per glyph, create a small boolean mask in "glyph space" (5x7) first.
  std::array<std::array<bool, 7>, 5> mask{};

  // Distance range (in atlas pixels) used for normalization.
  // Bigger => smoother gradients, but requires higher resolution. Keep modest for runtime generation.
  const int maxDist = std::clamp(fa.atlasScale * 2, 4, 18);
  const int maxDist2 = maxDist * maxDist;

  // Hi-res mask for a single glyph (glyphWpx x glyphHpx).
  std::vector<unsigned char> hiMask(static_cast<std::size_t>(fa.glyphWpx * fa.glyphHpx), 0);

  for (int c = fa.firstChar; c <= fa.lastChar; ++c) {
    // Reset mask.
    for (int gx = 0; gx < fa.glyphW; ++gx) {
      for (int gy = 0; gy < fa.glyphH; ++gy) {
        mask[gx][gy] = false;
      }
    }

    const auto& rows5x7 = gfx::GetGlyphRows5x7(static_cast<char>(c));
    for (int gy = 0; gy < fa.glyphH; ++gy) {
      const std::uint8_t row = rows5x7[static_cast<std::size_t>(gy)];
      for (int gx = 0; gx < fa.glyphW; ++gx) {
        if (row & (1u << (4 - gx))) {
          mask[gx][gy] = true;
        }
      }
    }

    if (bold) {
      // Simple dilation in glyph space: thicken right + down one pixel.
      std::array<std::array<bool, 7>, 5> out = mask;
      for (int gy = 0; gy < fa.glyphH; ++gy) {
        for (int gx = 0; gx < fa.glyphW; ++gx) {
          if (!mask[gx][gy]) continue;
          if (gx + 1 < fa.glyphW) out[gx + 1][gy] = true;
          if (gy + 1 < fa.glyphH) out[gx][gy + 1] = true;
        }
      }
      mask = out;
    }

    const int idx = c - fa.firstChar;
    const int cellX = (idx % fa.cols) * fa.cellWpx;
    const int cellY = (idx / fa.cols) * fa.cellHpx;

    const int gx0 = cellX + fa.padPx;
    const int gy0 = cellY + fa.padPx;

    // Remember the source rectangle for this character (crop to glyph area).
    if (c >= 0 && c < static_cast<int>(fa.src.size())) {
      fa.src[static_cast<std::size_t>(c)] = Rectangle{static_cast<float>(gx0), static_cast<float>(gy0),
                                                      static_cast<float>(fa.glyphWpx),
                                                      static_cast<float>(fa.glyphHpx)};
    }

    // Rasterize hi-res mask.
    std::fill(hiMask.begin(), hiMask.end(), 0);
    for (int gy = 0; gy < fa.glyphH; ++gy) {
      for (int gx = 0; gx < fa.glyphW; ++gx) {
        if (!mask[gx][gy]) continue;
        const int px0 = gx * fa.atlasScale;
        const int py0 = gy * fa.atlasScale;
        for (int sy = 0; sy < fa.atlasScale; ++sy) {
          for (int sx = 0; sx < fa.atlasScale; ++sx) {
            const int x = px0 + sx;
            const int y = py0 + sy;
            if (x < 0 || y < 0 || x >= fa.glyphWpx || y >= fa.glyphHpx) continue;
            hiMask[static_cast<std::size_t>(y * fa.glyphWpx + x)] = 1;
          }
        }
      }
    }

    auto insideAt = [&](int x, int y) -> bool {
      return hiMask[static_cast<std::size_t>(y * fa.glyphWpx + x)] != 0;
    };

    // Brute-force signed distance to the nearest opposite pixel.
    // Glyph sizes here are tiny, so this is fast enough and keeps the implementation simple.
    for (int y = 0; y < fa.glyphHpx; ++y) {
      for (int x = 0; x < fa.glyphWpx; ++x) {
        const bool inside = insideAt(x, y);
        int best2 = maxDist2 + 1;

        for (int dy = -maxDist; dy <= maxDist; ++dy) {
          const int yy = y + dy;
          if (yy < 0 || yy >= fa.glyphHpx) continue;

          for (int dx = -maxDist; dx <= maxDist; ++dx) {
            const int xx = x + dx;
            if (xx < 0 || xx >= fa.glyphWpx) continue;

            if (insideAt(xx, yy) == inside) continue;
            const int d2 = dx * dx + dy * dy;
            if (d2 < best2) {
              best2 = d2;
              if (best2 == 0) break;
            }
          }
          if (best2 == 0) break;
        }

        const float dist = (best2 <= maxDist2) ? std::sqrt(static_cast<float>(best2)) : static_cast<float>(maxDist);
        const float signedDist = inside ? -dist : dist;

        // Normalize to 0..1 with 0.5 at the edge.
        float v = 0.5f + signedDist / (2.0f * static_cast<float>(maxDist));
        v = std::clamp(v, 0.0f, 1.0f);
        const unsigned char a = ClampU8(static_cast<int>(std::lround(v * 255.0f)));
        setPixel(gx0 + x, gy0 + y, a);
      }
    }
  }

  fa.tex = LoadTextureFromImage(img);
  UnloadImage(img);

  if (fa.tex.id != 0) {
    // SDF requires bilinear sampling for proper reconstruction.
    SetTextureFilter(fa.tex, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(fa.tex, TEXTURE_WRAP_CLAMP);
    fa.ready = true;
  }

  return fa;
}

static bool UseSdfFonts()
{
  if (g.settings.fontFilterPoint) return false;
  if (!g.sdfShaderReady) return false;
  // Require both atlases to be ready to avoid mixing styles.
  return g.fontSdf.ready && g.fontBoldSdf.ready && (g.fontSdf.tex.id != 0) && (g.fontBoldSdf.tex.id != 0);
}

static const FontAtlas& PickFont(bool bold)
{
  if (UseSdfFonts()) {
    return bold ? g.fontBoldSdf : g.fontSdf;
  }
  return bold ? g.fontBold : g.font;
}

static void DrawTextAtlas(const FontAtlas& fa, int x, int y, int sizePx, std::string_view text, Color color,
                          bool shadow, int spacingPx)
{
  if (!fa.ready || fa.tex.id == 0) {
    // Fallback to raylib default if atlas isn't ready.
    DrawText(std::string(text).c_str(), x, y, sizePx, color);
    return;
  }

  if (text.empty()) return;

  const float unitScale = (fa.glyphH > 0) ? (static_cast<float>(sizePx) / static_cast<float>(fa.glyphH)) : 1.0f;
  const float adv = static_cast<float>(fa.glyphW + std::max(0, spacingPx)) * unitScale;
  const float glyphW = static_cast<float>(fa.glyphW) * unitScale;
  const float glyphH = static_cast<float>(sizePx);

  auto drawPass = [&](int ox, int oy, Color col) {
    float cx = static_cast<float>(x + ox);
    float cy = static_cast<float>(y + oy);

    for (char ch : text) {
      if (ch == '\n') {
        cy += glyphH + std::max(1.0f, unitScale);
        cx = static_cast<float>(x + ox);
        continue;
      }

      const unsigned char c = static_cast<unsigned char>(ch);
      unsigned char use = c;
      if (use < fa.firstChar || use > fa.lastChar) {
        use = static_cast<unsigned char>('?');
      }

      const Rectangle src = fa.src[use];
      Rectangle dst{cx, cy, glyphW, glyphH};
      DrawTexturePro(fa.tex, src, dst, Vector2{0.0f, 0.0f}, 0.0f, col);

      cx += adv;
    }
  };

  const bool useSdf = UseSdfFonts();
  if (useSdf) BeginShaderMode(g.sdfShader);

  if (shadow) {
    const int off = std::max(1, static_cast<int>(std::lround(unitScale)));
    drawPass(off, off, Color{0, 0, 0, ClampU8(static_cast<int>(color.a) * 3 / 5)});
  }

  drawPass(0, 0, color);

  if (useSdf) EndShaderMode();
}

[[maybe_unused]] static Rectangle Shrink(Rectangle r, float px)
{
  r.x += px;
  r.y += px;
  r.width -= px * 2.0f;
  r.height -= px * 2.0f;
  if (r.width < 0.0f) r.width = 0.0f;
  if (r.height < 0.0f) r.height = 0.0f;
  return r;
}

static void DrawNoiseOverlay(Rectangle r, float timeSec, float scale, float alpha)
{
  if (g.noise.id == 0) return;

  scale = std::clamp(scale, 0.05f, 4.0f);
  alpha = std::clamp(alpha, 0.0f, 1.0f);

  const float scrollX = timeSec * 9.0f;
  const float scrollY = timeSec * 13.0f;

  // IMPORTANT: With TEXTURE_WRAP_REPEAT, src sizes larger than the texture will tile.
  const Rectangle src{scrollX, scrollY, r.width * scale, r.height * scale};
  const Color tint = Color{255, 255, 255, ClampU8(static_cast<int>(std::lround(alpha * 255.0f)))};
  DrawTexturePro(g.noise, src, r, Vector2{0, 0}, 0.0f, tint);
}

} // namespace

void Init(std::uint64_t seed)
{
  if (g.refCount++ > 0) {
    if (seed != 0) SetSeed(seed);
    return;
  }

  g.seed = seed;
  RegenTheme(seed);

  g.noise = MakeNoiseTexture(seed);
  g.font = MakeFontAtlas(false, g.settings.fontAtlasScale, g.settings.fontFilterPoint);
  g.fontBold = MakeFontAtlas(true, g.settings.fontAtlasScale, g.settings.fontFilterPoint);

  // Attempt to load the SDF shader. If shaders are unsupported on the current backend, this will
  // fail and we'll transparently fall back to bitmap fonts.
  EnsureSdfShaderLoaded();

  // Build SDF atlases only when the user requests smooth (non-point) filtering and the shader is ready.
  g.fontSdf.unload();
  g.fontBoldSdf.unload();
  if (!g.settings.fontFilterPoint && g.sdfShaderReady) {
    g.fontSdf = MakeFontAtlasSdf(false, g.settings.fontAtlasScale);
    g.fontBoldSdf = MakeFontAtlasSdf(true, g.settings.fontAtlasScale);
  }

  g.ready = (g.font.ready && g.fontBold.ready);
}

void Shutdown()
{
  if (g.refCount <= 0) return;
  g.refCount -= 1;
  if (g.refCount > 0) return;

  if (g.noise.id != 0) {
    UnloadTexture(g.noise);
    g.noise = Texture2D{};
  }

  g.font.unload();
  g.fontBold.unload();
  g.fontSdf.unload();
  g.fontBoldSdf.unload();
  UnloadSdfShader();

  g.ready = false;
  g.seed = 0;
}

bool IsReady()
{
  return g.ready;
}

void SetSeed(std::uint64_t seed)
{
  g.seed = seed;
  RegenTheme(seed);

  // Regenerate the noise texture so each seed has a slightly different "grain".
  if (g.noise.id != 0) {
    UnloadTexture(g.noise);
    g.noise = Texture2D{};
  }
  g.noise = MakeNoiseTexture(seed);
}

Settings GetSettings()
{
  return g.settings;
}

static Settings ClampSettings(Settings s)
{
  s.accentHueDeg = std::clamp(s.accentHueDeg, 0.0f, 360.0f);
  s.accentSaturation = std::clamp(s.accentSaturation, 0.0f, 1.0f);
  s.accentValue = std::clamp(s.accentValue, 0.0f, 1.0f);
  s.roundness = std::clamp(s.roundness, 0.0f, 1.0f);
  s.roundSegments = std::clamp(s.roundSegments, 0, 16);
  s.noiseAlpha = std::clamp(s.noiseAlpha, 0.0f, 1.0f);
  s.noiseScale = std::clamp(s.noiseScale, 0.05f, 4.0f);
  s.headerSheenStrength = std::clamp(s.headerSheenStrength, 0.0f, 1.0f);
  s.fontAtlasScale = std::clamp(s.fontAtlasScale, 1, 8);
  return s;
}

void SetSettings(const Settings& s)
{
  const Settings next = ClampSettings(s);

  const bool atlasScaleChanged = (next.fontAtlasScale != g.settings.fontAtlasScale);
  const bool filterChanged = (next.fontFilterPoint != g.settings.fontFilterPoint);

  g.settings = next;
  RegenTheme(g.seed);

  // If we're not initialized yet, store the settings and let Init() build resources.
  if (g.refCount <= 0) return;

  // Font atlas rebuild/update.
  if (atlasScaleChanged) {
    g.font.unload();
    g.fontBold.unload();
    g.font = MakeFontAtlas(false, g.settings.fontAtlasScale, g.settings.fontFilterPoint);
    g.fontBold = MakeFontAtlas(true, g.settings.fontAtlasScale, g.settings.fontFilterPoint);

    // Rebuild SDF atlases if we can/want to use them.
    g.fontSdf.unload();
    g.fontBoldSdf.unload();
    if (!g.settings.fontFilterPoint && g.sdfShaderReady) {
      g.fontSdf = MakeFontAtlasSdf(false, g.settings.fontAtlasScale);
      g.fontBoldSdf = MakeFontAtlasSdf(true, g.settings.fontAtlasScale);
    }
  } else if (filterChanged) {
    if (g.font.tex.id != 0) {
      SetTextureFilter(g.font.tex, g.settings.fontFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    }
    if (g.fontBold.tex.id != 0) {
      SetTextureFilter(g.fontBold.tex, g.settings.fontFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    }

    // Smooth filter implies SDF rendering; point filter implies bitmap rendering.
    if (g.settings.fontFilterPoint) {
      g.fontSdf.unload();
      g.fontBoldSdf.unload();
    } else {
      // Lazily build SDF atlases if the shader is available.
      if (g.sdfShaderReady) {
        if (!g.fontSdf.ready || !g.fontBoldSdf.ready) {
          g.fontSdf.unload();
          g.fontBoldSdf.unload();
          g.fontSdf = MakeFontAtlasSdf(false, g.settings.fontAtlasScale);
          g.fontBoldSdf = MakeFontAtlasSdf(true, g.settings.fontAtlasScale);
        }
      }
    }
  }

  g.ready = (g.font.ready && g.fontBold.ready);
}

void ResetSettings()
{
  SetSettings(Settings{});
}

const Theme& GetTheme()
{
  return g.theme;
}

void ClearActiveWidget()
{
  g.activeWidgetId = -1;
}

void DrawPanel(Rectangle r, float timeSec, bool active)
{
  const Theme& t = g.theme;

  // Base fill.
  Color bot = t.panelBgBot;
  Color top = t.panelBgTop;
  if (!active) {
    bot = Darken(bot, 0.85f);
    top = Darken(top, 0.85f);
  }

  DrawRectangleRounded(r, t.roundness, t.roundSegments, bot);

  // Top highlight layer (clipped so it only affects the upper portion).
  {
    const int sx = static_cast<int>(std::floor(r.x));
    const int sy = static_cast<int>(std::floor(r.y));
    const int sw = static_cast<int>(std::ceil(r.width));
    const int sh = static_cast<int>(std::ceil(r.height * 0.55f));
    BeginScissorMode(sx, sy, sw, sh);
    DrawRectangleRounded(r, t.roundness, t.roundSegments, top);
    EndScissorMode();
  }

  // Subtle animated sheen near the header.
  if (t.headerSheenStrength > 0.001f) {
    const float s = 0.5f + 0.5f * std::sin(timeSec * 1.2f);
    const unsigned char a = ClampU8(static_cast<int>(30.0f * t.headerSheenStrength * (0.25f + 0.75f * s)));
    const Color c0 = WithAlpha(active ? t.accent : t.textDim, a);
    const Color c1 = WithAlpha(active ? t.accentDim : t.textDim, 0);
    const int hx = static_cast<int>(std::floor(r.x));
    const int hy = static_cast<int>(std::floor(r.y));
    const int hw = static_cast<int>(std::ceil(r.width));
    const int hh = std::max(1, static_cast<int>(std::ceil(std::min(18.0f, r.height * 0.20f))));

    BeginScissorMode(hx, hy, hw, hh);
    DrawRectangleGradientH(hx, hy, hw, hh, c0, c1);
    EndScissorMode();
  }

  // Noise overlay.
  DrawNoiseOverlay(r, timeSec, t.noiseScale, t.noiseAlpha);

  // Border.
  DrawRectangleRoundedLinesEx(r, t.roundness, t.roundSegments, 1.0f, t.panelBorder);
}

void DrawPanelInset(Rectangle r, float timeSec, bool active)
{
  const Theme& t = g.theme;

  // Darker inset.
  Color bot = Darken(t.panelBgBot, active ? 0.85f : 0.75f);
  Color top = Darken(t.panelBgTop, active ? 0.85f : 0.75f);

  DrawRectangleRounded(r, t.roundness, t.roundSegments, bot);

  {
    const int sx = static_cast<int>(std::floor(r.x));
    const int sy = static_cast<int>(std::floor(r.y));
    const int sw = static_cast<int>(std::ceil(r.width));
    const int sh = static_cast<int>(std::ceil(r.height * 0.55f));
    BeginScissorMode(sx, sy, sw, sh);
    DrawRectangleRounded(r, t.roundness, t.roundSegments, top);
    EndScissorMode();
  }

  DrawNoiseOverlay(r, timeSec, t.noiseScale * 0.9f, t.noiseAlpha * 0.85f);

  DrawRectangleRoundedLinesEx(r, t.roundness, t.roundSegments, 1.0f, WithAlpha(t.panelBorder, 60));
}

void DrawPanelHeader(Rectangle panel, std::string_view title, float timeSec, bool active, int titleSizePx)
{
  (void)timeSec;
  const Theme& t = g.theme;

  // Small accent bar.
  const int barH = 3;
  DrawRectangle(static_cast<int>(panel.x) + 10, static_cast<int>(panel.y) + 6,
                std::max(0, static_cast<int>(panel.width) - 20), barH,
                WithAlpha(active ? t.accent : t.textDim, active ? 160 : 80));

  Text(static_cast<int>(panel.x) + 12, static_cast<int>(panel.y) + 10, titleSizePx, title,
       active ? t.text : t.textDim,
       /*bold=*/true, /*shadow=*/true, /*spacingPx=*/1);
}

void DrawSelectionHighlight(Rectangle r, float timeSec, bool strong)
{
  (void)timeSec;
  const Theme& t = g.theme;

  const float pulse = 0.65f + 0.35f * std::sin(timeSec * 4.0f);
  const unsigned char a = ClampU8(static_cast<int>(strong ? (90.0f + 60.0f * pulse) : (55.0f + 35.0f * pulse)));

  DrawRectangleRounded(r, t.roundness, t.roundSegments, WithAlpha(t.accent, a));
}

void Text(int x, int y, int sizePx, std::string_view text, Color color, bool bold, bool shadow, int spacingPx)
{
  if (sizePx <= 0) return;
  const FontAtlas& fa = PickFont(bold);
  DrawTextAtlas(fa, x, y, sizePx, text, color, shadow, spacingPx);
}

void TextCentered(Rectangle r, int sizePx, std::string_view text, Color color, bool bold, bool shadow, int spacingPx)
{
  const int w = MeasureTextWidth(text, sizePx, bold, spacingPx);
  const int h = MeasureTextHeight(sizePx);

  const int x = static_cast<int>(std::lround(r.x + (r.width - static_cast<float>(w)) * 0.5f));
  const int y = static_cast<int>(std::lround(r.y + (r.height - static_cast<float>(h)) * 0.5f));

  Text(x, y, sizePx, text, color, bold, shadow, spacingPx);
}

int MeasureTextWidth(std::string_view text, int sizePx, bool bold, int spacingPx)
{
  const FontAtlas& fa = PickFont(bold);
  if (text.empty()) return 0;

  // Handle newlines: measure max line.
  int maxW = 0;
  int curCount = 0;
  for (char c : text) {
    if (c == '\n') {
      const float unitScale = (fa.glyphH > 0) ? (static_cast<float>(sizePx) / static_cast<float>(fa.glyphH)) : 1.0f;
      const float adv = static_cast<float>(fa.glyphW + std::max(0, spacingPx)) * unitScale;
      const float w = static_cast<float>(curCount) * adv - static_cast<float>(std::max(0, spacingPx)) * unitScale;
      maxW = std::max(maxW, static_cast<int>(std::lround(std::max(0.0f, w))));
      curCount = 0;
    } else {
      curCount += 1;
    }
  }

  {
    const float unitScale = (fa.glyphH > 0) ? (static_cast<float>(sizePx) / static_cast<float>(fa.glyphH)) : 1.0f;
    const float adv = static_cast<float>(fa.glyphW + std::max(0, spacingPx)) * unitScale;
    const float w = static_cast<float>(curCount) * adv - static_cast<float>(std::max(0, spacingPx)) * unitScale;
    maxW = std::max(maxW, static_cast<int>(std::lround(std::max(0.0f, w))));
  }

  return maxW;
}

int MeasureTextHeight(int sizePx)
{
  // One line.
  return std::max(0, sizePx);
}



// Draw text with an outline for readability on bright/noisy backgrounds.
void TextOutlined(int x, int y, int sizePx, std::string_view text, Color fill, Color outline,
                  bool bold, bool shadow, int spacingPx)
{
  if (sizePx <= 0) return;
  if (text.empty()) return;

  const FontAtlas& fa = PickFont(bold);

  // 4-neighborhood outline (cheap but effective for tiny bitmap fonts).
  DrawTextAtlas(fa, x - 1, y,     sizePx, text, outline, /*shadow=*/false, spacingPx);
  DrawTextAtlas(fa, x + 1, y,     sizePx, text, outline, /*shadow=*/false, spacingPx);
  DrawTextAtlas(fa, x,     y - 1, sizePx, text, outline, /*shadow=*/false, spacingPx);
  DrawTextAtlas(fa, x,     y + 1, sizePx, text, outline, /*shadow=*/false, spacingPx);

  DrawTextAtlas(fa, x, y, sizePx, text, fill, shadow, spacingPx);
}

static inline int DefaultLineGapPx(int sizePx)
{
  // Tuned by eye for the 5x7 atlas scaled to typical UI sizes (14-22px).
  return std::max(2, static_cast<int>(std::lround(static_cast<float>(sizePx) * 0.22f)));
}

static inline int LineStepPx(int sizePx)
{
  return std::max(1, sizePx) + DefaultLineGapPx(sizePx);
}

static inline int MaxCharsForWidthPx(int widthPx, int sizePx, bool bold, int spacingPx)
{
  const FontAtlas& fa = PickFont(bold);
  if (fa.glyphH <= 0) return 0;

  const float unitScale = static_cast<float>(sizePx) / static_cast<float>(fa.glyphH);
  const float adv = static_cast<float>(fa.glyphW + std::max(0, spacingPx)) * unitScale;
  if (adv <= 0.01f) return 0;

  // Account for the fact that MeasureTextWidth() does not include trailing spacing.
  const float usable = static_cast<float>(widthPx) + static_cast<float>(std::max(0, spacingPx)) * unitScale;
  return std::max(1, static_cast<int>(std::floor(usable / adv)));
}

int TextBox(Rectangle r, int sizePx, std::string_view text, Color color,
            bool bold, bool shadow, int spacingPx, bool wrap, bool clip)
{
  if (sizePx <= 0) return static_cast<int>(std::lround(r.y));
  if (text.empty()) return static_cast<int>(std::lround(r.y));

  const int x0 = static_cast<int>(std::floor(r.x));
  int y = static_cast<int>(std::floor(r.y));
  const int maxW = std::max(0, static_cast<int>(std::floor(r.width)));

  const int step = LineStepPx(sizePx);

  if (clip) {
    BeginScissorMode(static_cast<int>(std::floor(r.x)), static_cast<int>(std::floor(r.y)),
                     static_cast<int>(std::ceil(r.width)), static_cast<int>(std::ceil(r.height)));
  }

  auto flushLine = [&](const std::string& line) {
    Text(x0, y, sizePx, line, color, bold, shadow, spacingPx);
    y += step;
  };

  if (!wrap || maxW <= 0) {
    // Draw as-is; newlines are handled by Text()/MeasureTextWidth() but Text() won't advance Y.
    // We handle newlines here by splitting on '\n' to stack lines.
    std::string cur;
    cur.reserve(text.size());

    for (char c : text) {
      if (c == '\r') continue;
      if (c == '\n') {
        flushLine(cur);
        cur.clear();
        continue;
      }
      cur.push_back(c);
    }
    flushLine(cur);

    if (clip) EndScissorMode();
    return y;
  }

  std::string line;
  line.reserve(128);

  std::size_t i = 0;
  while (i < text.size()) {
    const char c = text[i];

    if (c == '\r') {
      ++i;
      continue;
    }
    if (c == '\n') {
      flushLine(line);
      line.clear();
      ++i;
      continue;
    }
    if (c == ' ' || c == '\t') {
      // Collapse consecutive whitespace to a single space.
      if (!line.empty() && line.back() != ' ') line.push_back(' ');
      while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
      continue;
    }

    // Read a word.
    std::size_t j = i;
    while (j < text.size()) {
      const char cj = text[j];
      if (cj == '\n' || cj == '\r' || cj == ' ' || cj == '\t') break;
      ++j;
    }
    const std::string_view word = text.substr(i, j - i);

    if (line.empty()) {
      if (MeasureTextWidth(word, sizePx, bold, spacingPx) > maxW) {
        // Hard-wrap a too-long word.
        const int maxChars = std::max(1, MaxCharsForWidthPx(maxW, sizePx, bold, spacingPx));
        std::size_t p = 0;
        while (p < word.size()) {
          const std::string_view chunk = word.substr(p, static_cast<std::size_t>(maxChars));
          Text(x0, y, sizePx, chunk, color, bold, shadow, spacingPx);
          y += step;
          p += static_cast<std::size_t>(maxChars);
        }
      } else {
        line.assign(word);
      }
    } else {
      std::string cand = line;
      if (!cand.empty() && cand.back() != ' ') cand.push_back(' ');
      cand.append(word);

      if (MeasureTextWidth(cand, sizePx, bold, spacingPx) <= maxW) {
        line.swap(cand);
      } else {
        flushLine(line);
        line.assign(word);
      }
    }

    i = j;
  }

  if (!line.empty()) flushLine(line);

  if (clip) EndScissorMode();
  return y;
}

int MeasureTextHeight(std::string_view text, int sizePx, bool bold, int spacingPx, int wrapWidthPx)
{
  if (sizePx <= 0) return 0;
  if (text.empty()) return 0;

  const int step = LineStepPx(sizePx);

  // Quick path: no wrapping, just count newlines.
  if (wrapWidthPx <= 0) {
    int lines = 1;
    for (char c : text) {
      if (c == '\n') ++lines;
    }
    return lines * step;
  }

  const int maxChars = std::max(1, MaxCharsForWidthPx(wrapWidthPx, sizePx, bold, spacingPx));

  int lines = 1;
  int curChars = 0;

  std::size_t i = 0;
  while (i < text.size()) {
    const char c = text[i];

    if (c == '\r') {
      ++i;
      continue;
    }
    if (c == '\n') {
      ++lines;
      curChars = 0;
      ++i;
      continue;
    }
    if (c == ' ' || c == '\t') {
      // Collapse whitespace.
      if (curChars > 0 && curChars < maxChars) ++curChars;
      while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
      continue;
    }

    // Read word.
    std::size_t j = i;
    while (j < text.size()) {
      const char cj = text[j];
      if (cj == '\n' || cj == '\r' || cj == ' ' || cj == '\t') break;
      ++j;
    }
    const int wlen = static_cast<int>(j - i);

    if (wlen > maxChars) {
      // Hard-wrap the long word into multiple lines.
      const int full = wlen / maxChars;
      const int rem = wlen % maxChars;
      lines += full;
      curChars = rem;
      if (curChars == 0) curChars = maxChars;
      if (curChars >= maxChars) {
        ++lines;
        curChars = 0;
      }
    } else {
      if (curChars == 0) {
        curChars = wlen;
      } else {
        if (curChars + 1 + wlen <= maxChars) {
          curChars += 1 + wlen;
        } else {
          ++lines;
          curChars = wlen;
        }
      }
    }

    i = j;
  }

  return lines * step;
}

int DrawKeycap(int x, int y, std::string_view label, float timeSec, bool strong, int sizePx)
{
  const Theme& t = g.theme;

  const int padX = std::max(6, sizePx / 3);
  const int padY = std::max(4, sizePx / 4);

  const int textW = MeasureTextWidth(label, sizePx, /*bold=*/true, /*spacingPx=*/1);
  const int w = std::max(10, textW + padX * 2);
  const int h = std::max(10, sizePx + padY * 2);

  Rectangle r{static_cast<float>(x), static_cast<float>(y), static_cast<float>(w), static_cast<float>(h)};

  const float pulse = 0.65f + 0.35f * std::sin(timeSec * 3.0f + static_cast<float>(x) * 0.01f);
  const Color border = strong
                         ? WithAlpha(t.accent, ClampU8(static_cast<int>(140.0f + 70.0f * pulse)))
                         : WithAlpha(t.panelBorderHi, 120);

  // Fill.
  DrawRectangleRounded(r, t.roundness, t.roundSegments, WithAlpha(t.panelBgBot, strong ? 240 : 220));
  // Border.
  DrawRectangleRoundedLinesEx(r, t.roundness, t.roundSegments, 1.0f, border);

  // Subtle top highlight (gives a keycap feel).
  DrawRectangleRounded(Rectangle{r.x + 1.0f, r.y + 1.0f, r.width - 2.0f, r.height * 0.44f},
                       t.roundness, t.roundSegments, WithAlpha(WHITE, 18));

  TextCentered(r, sizePx, label, strong ? t.text : t.textDim, /*bold=*/true, /*shadow=*/true, 1);
  return w;
}

int DrawKeyCombo(int x, int y, std::string_view combo, float timeSec, bool strong, int sizePx)
{
  const Theme& t = g.theme;
  int cx = x;
  bool first = true;

  std::size_t start = 0;
  while (start < combo.size()) {
    const std::size_t plus = combo.find('+', start);
    std::string_view tok =
      (plus == std::string_view::npos) ? combo.substr(start) : combo.substr(start, plus - start);

    // Trim spaces.
    while (!tok.empty() && tok.front() == ' ') tok.remove_prefix(1);
    while (!tok.empty() && tok.back() == ' ') tok.remove_suffix(1);

    if (!tok.empty()) {
      if (!first) {
        const char* plusTxt = "+";
        const int py = y + std::max(0, (sizePx / 4));
        Text(cx, py, sizePx, plusTxt, t.textDim, /*bold=*/true, /*shadow=*/true, 1);
        cx += MeasureTextWidth(plusTxt, sizePx, /*bold=*/true, 1) + 6;
      }

      const int w = DrawKeycap(cx, y, tok, timeSec, strong, sizePx);
      cx += w + 6;
      first = false;
    }

    if (plus == std::string_view::npos) break;
    start = plus + 1;
  }

  return cx - x;
}

bool Toggle(int id, Rectangle r, bool& ioValue, Vector2 mouseUi, float timeSec, bool enabled)
{
  (void)timeSec;
  (void)id;
  const Theme& t = g.theme;

  const bool hovered = CheckCollisionPointRec(mouseUi, r);
  const bool pressed = enabled && hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

  if (pressed) {
    ioValue = !ioValue;
  }

  const float rr = std::clamp(t.roundness * 1.25f, 0.0f, 1.0f);

  Color trackOff = WithAlpha(t.panelBgBot, enabled ? 230 : 160);
  Color trackOn = WithAlpha(t.accent, enabled ? 210 : 120);

  if (hovered && enabled) {
    trackOff = Lighten(trackOff, 0.08f);
    trackOn = Lighten(trackOn, 0.08f);
  }

  DrawRectangleRounded(r, rr, t.roundSegments, ioValue ? trackOn : trackOff);
  DrawRectangleRoundedLinesEx(r, rr, t.roundSegments, 1.0f,
                              WithAlpha(t.panelBorderHi, hovered ? 160 : 120));

  // Knob
  const float knobR = std::max(2.0f, r.height * 0.42f);
  const float knobX = ioValue ? (r.x + r.width - r.height * 0.5f) : (r.x + r.height * 0.5f);
  const Vector2 c{knobX, r.y + r.height * 0.5f};
  DrawCircleV(c, knobR, WithAlpha(WHITE, enabled ? 235 : 170));
  DrawCircleLines(static_cast<int>(std::lround(c.x)), static_cast<int>(std::lround(c.y)), knobR,
                  WithAlpha(BLACK, enabled ? 80 : 50));

  return pressed;
}

bool SliderFloat(int id, Rectangle r, float& ioValue, float minValue, float maxValue,
                 Vector2 mouseUi, float timeSec, bool enabled)
{
  (void)timeSec;
  const Theme& t = g.theme;

  if (r.width <= 1.0f || r.height <= 1.0f) return false;

  // Release focus on mouse-up.
  if (g.activeWidgetId == id && !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    g.activeWidgetId = -1;
  }

  const bool hovered = CheckCollisionPointRec(mouseUi, r);
  if (enabled && hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    g.activeWidgetId = id;
  }

  const bool active = (g.activeWidgetId == id);

  bool changed = false;
  if (enabled && active && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    const float t01 = std::clamp((mouseUi.x - r.x) / r.width, 0.0f, 1.0f);
    const float next = minValue + (maxValue - minValue) * t01;
    if (std::fabs(next - ioValue) > 0.00001f) {
      ioValue = next;
      changed = true;
    }
  }

  // Draw track.
  const Color track = WithAlpha(t.panelBgBot, enabled ? 230 : 150);
  DrawRectangleRounded(r, t.roundness, t.roundSegments, track);

  const float denom = (maxValue - minValue);
  const float frac = (denom != 0.0f) ? std::clamp((ioValue - minValue) / denom, 0.0f, 1.0f) : 0.0f;

  // Filled portion (clipped draw of a rounded rect so the left end stays rounded).
  if (frac > 0.001f) {
    const int sx = static_cast<int>(std::floor(r.x));
    const int sy = static_cast<int>(std::floor(r.y));
    const int sw = static_cast<int>(std::ceil(r.width * frac));
    const int sh = static_cast<int>(std::ceil(r.height));
    BeginScissorMode(sx, sy, sw, sh);
    DrawRectangleRounded(r, t.roundness, t.roundSegments,
                         WithAlpha(enabled ? t.accent : t.textDim, hovered || active ? 165 : 135));
    EndScissorMode();
  }

  // Knob.
  const float knobR = std::max(2.0f, r.height * 0.42f);
  const float knobX = r.x + r.width * frac;
  const Vector2 c{knobX, r.y + r.height * 0.5f};
  DrawCircleV(c, knobR, WithAlpha(WHITE, enabled ? 235 : 170));
  DrawCircleLines(static_cast<int>(std::lround(c.x)), static_cast<int>(std::lround(c.y)), knobR,
                  WithAlpha(BLACK, enabled ? 70 : 50));

  DrawRectangleRoundedLinesEx(r, t.roundness, t.roundSegments, 1.0f,
                              WithAlpha(t.panelBorderHi, (hovered || active) ? 170 : 120));
  return changed;
}

bool SliderInt(int id, Rectangle r, int& ioValue, int minValue, int maxValue, int step,
               Vector2 mouseUi, float timeSec, bool enabled)
{
  step = std::max(1, step);

  float f = static_cast<float>(ioValue);
  const bool floatChanged = SliderFloat(id, r, f, static_cast<float>(minValue), static_cast<float>(maxValue),
                                        mouseUi, timeSec, enabled);

  int q = static_cast<int>(std::lround((f - static_cast<float>(minValue)) / static_cast<float>(step))) * step +
          minValue;
  q = std::clamp(q, minValue, maxValue);

  const bool changed = (q != ioValue);
  if (changed) ioValue = q;

  // If the float slider moved but quantized back to the same int, report no change.
  (void)floatChanged;
  return changed;
}



bool Button(int id, Rectangle r, std::string_view label, Vector2 mouseUi, float timeSec,
            bool enabled, bool primary)
{
  (void)timeSec;
  const Theme& t = g.theme;

  const bool hovered = CheckCollisionPointRec(mouseUi, r);

  if (enabled && hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    g.activeWidgetId = id;
  }

  const bool active = (g.activeWidgetId == id) && IsMouseButtonDown(MOUSE_BUTTON_LEFT);

  bool clicked = false;
  if (enabled && g.activeWidgetId == id && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
    clicked = hovered;
    g.activeWidgetId = -1;
  }

  // If the user released outside the button, clear focus.
  if (g.activeWidgetId == id && !IsMouseButtonDown(MOUSE_BUTTON_LEFT) && !IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    g.activeWidgetId = -1;
  }

  // Colors.
  Color fill = primary ? WithAlpha(t.accent, enabled ? 170 : 90) : WithAlpha(t.panelBgBot, enabled ? 235 : 160);
  Color border = WithAlpha(t.panelBorderHi, hovered ? 170 : 120);
  Color txt = enabled ? t.text : t.textFaint;

  if (hovered && enabled) {
    fill = Lighten(fill, 0.06f);
  }
  if (active && enabled) {
    fill = Darken(fill, 0.92f);
  }

  const float rr = std::clamp(t.roundness * 1.10f, 0.0f, 1.0f);

  DrawRectangleRounded(r, rr, t.roundSegments, fill);
  DrawRectangleRoundedLinesEx(r, rr, t.roundSegments, 1.0f, border);

  // Subtle top highlight.
  DrawRectangleRounded(Rectangle{r.x + 1.0f, r.y + 1.0f, r.width - 2.0f, r.height * 0.45f},
                       rr, t.roundSegments, WithAlpha(WHITE, primary ? 22 : 16));

  // Label.
  const int sizePx = static_cast<int>(std::clamp(r.height * 0.55f, 10.0f, 22.0f));
  TextCentered(r, sizePx, label, txt, /*bold=*/true, /*shadow=*/true, 1);
  return clicked;
}


// ------------------------------------------------------------------------------------------------
// Scrollbar widget (vertical)
// ------------------------------------------------------------------------------------------------

Rectangle ContentRectWithScrollbar(Rectangle r, float scrollbarW, float gap)
{
  if (scrollbarW < 0.0f) scrollbarW = 0.0f;
  if (gap < 0.0f) gap = 0.0f;
  Rectangle out = r;
  out.width = std::max(0.0f, r.width - scrollbarW - gap);
  return out;
}

bool ScrollbarV(int id, Rectangle barR, int contentUnits, int viewUnits, int& ioScrollUnits,
                Vector2 mouseUi, float timeSec, bool enabled)
{
  (void)timeSec;

  const Theme& t = GetTheme();

  // Sanitize.
  contentUnits = std::max(0, contentUnits);
  viewUnits = std::max(0, viewUnits);

  const int maxScroll = std::max(0, contentUnits - viewUnits);
  ioScrollUnits = std::clamp(ioScrollUnits, 0, maxScroll);

  // Nothing to scroll.
  if (maxScroll <= 0 || barR.width <= 2.0f || barR.height <= 2.0f) {
    return false;
  }

  const bool hoveredBar = CheckCollisionPointRec(mouseUi, barR);

  // Release drag on mouse up.
  if (g.activeWidgetId == id && !IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
    g.activeWidgetId = -1;
  }

  // Compute thumb geometry.
  const float minThumbH = 18.0f;
  const float trackH = std::max(1.0f, barR.height);
  const float thumbH = std::clamp(trackH * (static_cast<float>(viewUnits) / static_cast<float>(contentUnits)),
                                  minThumbH, trackH);
  const float moveH = std::max(1.0f, trackH - thumbH);

  const float t01 = (maxScroll > 0) ? (static_cast<float>(ioScrollUnits) / static_cast<float>(maxScroll)) : 0.0f;
  const float thumbY = barR.y + moveH * t01;

  const float pad = 2.0f;
  Rectangle thumbR{barR.x + pad, thumbY + pad, std::max(0.0f, barR.width - pad * 2.0f),
                   std::max(0.0f, thumbH - pad * 2.0f)};

  const bool hoveredThumb = CheckCollisionPointRec(mouseUi, thumbR);

  // Start dragging.
  if (enabled && hoveredThumb && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
    g.activeWidgetId = id;
  }

  bool changed = false;
  const bool active = enabled && (g.activeWidgetId == id) && IsMouseButtonDown(MOUSE_BUTTON_LEFT);

  // Click on the track pages up/down.
  if (enabled && hoveredBar && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !hoveredThumb) {
    if (mouseUi.y < thumbR.y) {
      ioScrollUnits -= std::max(1, viewUnits);
    } else {
      ioScrollUnits += std::max(1, viewUnits);
    }
    ioScrollUnits = std::clamp(ioScrollUnits, 0, maxScroll);
    changed = true;
  }

  // Dragging maps mouse Y -> scroll.
  if (active) {
    const float my = mouseUi.y;
    float u01 = (my - barR.y - thumbH * 0.5f) / std::max(1.0f, moveH);
    u01 = std::clamp(u01, 0.0f, 1.0f);
    const int newScroll = static_cast<int>(std::lround(u01 * static_cast<float>(maxScroll)));
    if (newScroll != ioScrollUnits) {
      ioScrollUnits = newScroll;
      changed = true;
    }
  }

  // Draw track.
  const float rr = std::clamp(t.roundness * 0.95f, 0.0f, 1.0f);
  Color trackFill = WithAlpha(t.panelBgBot, hoveredBar ? 220 : 180);
  Color trackBorder = WithAlpha(t.panelBorder, hoveredBar ? 150 : 110);

  DrawRectangleRounded(barR, rr, t.roundSegments, trackFill);
  DrawRectangleRoundedLinesEx(barR, rr, t.roundSegments, 1.0f, trackBorder);

  // Draw thumb.
  Color thumbFill = t.accent;
  thumbFill.a = static_cast<unsigned char>(active ? 220 : (hoveredThumb ? 185 : 140));
  if (!enabled) thumbFill.a = 80;

  DrawRectangleRounded(thumbR, rr, t.roundSegments, thumbFill);

  // Tiny sheen on thumb.
  DrawRectangleRounded(Rectangle{thumbR.x + 1.0f, thumbR.y + 1.0f, thumbR.width - 2.0f, thumbR.height * 0.45f},
                       rr, t.roundSegments, WithAlpha(WHITE, hoveredThumb ? 26 : 18));

  return changed;
}

} // namespace ui
} // namespace isocity
