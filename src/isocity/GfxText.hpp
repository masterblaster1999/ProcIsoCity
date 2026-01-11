#pragma once

#include "isocity/GfxCanvas.hpp"

#include <array>
#include <cctype>
#include <cstdint>
#include <string_view>

namespace isocity {
namespace gfx {

// -----------------------------------------------------------------------------------------------
// Tiny built-in bitmap font helpers (5x7 monospace)
// -----------------------------------------------------------------------------------------------
//
// This is intentionally:
//  - header-only (used by toolchain + game runtime generators)
//  - dependency-free
//  - deterministic (no system font / platform differences)
//
// It is meant for small in-sprite markings:
//  - commercial rooftop signage
//  - vehicle decals / IDs
//  - debug overlays in exported textures
//
// Glyph format:
//  - 5 pixels wide, 7 pixels tall
//  - each row is a 5-bit mask stored in the low bits of the byte
//    (bit4 is leftmost pixel, bit0 is rightmost)

struct Glyph5x7 {
  char ch = '?';
  std::array<std::uint8_t, 7> rows{};
};

inline constexpr std::array<Glyph5x7, 10 + 26 + 6> kFont5x7 = {{
  // Digits
  {'0', {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110}},
  {'1', {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
  {'2', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}},
  {'3', {0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110}},
  {'4', {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}},
  {'5', {0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110}},
  {'6', {0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}},
  {'7', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}},
  {'8', {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}},
  {'9', {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100}},

  // Uppercase letters
  {'A', {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
  {'B', {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110}},
  {'C', {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110}},
  {'D', {0b11100, 0b10010, 0b10001, 0b10001, 0b10001, 0b10010, 0b11100}},
  {'E', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}},
  {'F', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000}},
  {'G', {0b01110, 0b10001, 0b10000, 0b10000, 0b10011, 0b10001, 0b01110}},
  {'H', {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
  {'I', {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
  {'J', {0b00001, 0b00001, 0b00001, 0b00001, 0b10001, 0b10001, 0b01110}},
  {'K', {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}},
  {'L', {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}},
  {'M', {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001}},
  {'N', {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001}},
  {'O', {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
  {'P', {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}},
  {'Q', {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101}},
  {'R', {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}},
  {'S', {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}},
  {'T', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
  {'U', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
  {'V', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100}},
  {'W', {0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001}},
  {'X', {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001}},
  {'Y', {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100}},
  {'Z', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111}},

  // Punctuation / symbols
  {' ', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}},
  {'-', {0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000}},
  {'.', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00100, 0b00100}},
  {'/', {0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b00000, 0b00000}},
  {'?', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b00000, 0b00100}},
  {':', {0b00000, 0b00100, 0b00100, 0b00000, 0b00100, 0b00100, 0b00000}},
}};

inline constexpr int Font5x7GlyphW() { return 5; }
inline constexpr int Font5x7GlyphH() { return 7; }

inline const std::array<std::uint8_t, 7>& GetGlyphRows5x7(char c)
{
  const unsigned char uc = static_cast<unsigned char>(c);
  const char up = static_cast<char>(std::toupper(uc));
  for (const auto& g : kFont5x7) {
    if (g.ch == up) return g.rows;
  }
  // Fallback: '?'.
  for (const auto& g : kFont5x7) {
    if (g.ch == '?') return g.rows;
  }
  // Should never happen.
  return kFont5x7[0].rows;
}

inline int MeasureTextWidth5x7(std::string_view s, int scale = 1, int spacingPx = 1)
{
  if (s.empty()) return 0;
  const int adv = (Font5x7GlyphW() + spacingPx) * std::max(1, scale);
  return static_cast<int>(s.size()) * adv - spacingPx * std::max(1, scale);
}

inline int MeasureTextHeight5x7(int scale = 1) { return Font5x7GlyphH() * std::max(1, scale); }

inline void DrawGlyph5x7(RgbaImage& img, int x, int y, char c, Rgba8 color, int scale = 1,
                         BlendMode mode = BlendMode::Alpha)
{
  const int sc = std::max(1, scale);
  const auto& rows = GetGlyphRows5x7(c);

  for (int gy = 0; gy < Font5x7GlyphH(); ++gy) {
    const std::uint8_t row = rows[static_cast<std::size_t>(gy)];
    for (int gx = 0; gx < Font5x7GlyphW(); ++gx) {
      const std::uint8_t bit = static_cast<std::uint8_t>(1u << (Font5x7GlyphW() - 1 - gx));
      if ((row & bit) == 0) continue;

      if (sc == 1) {
        BlendPixel(img, x + gx, y + gy, color, mode);
      } else {
        const int x0 = x + gx * sc;
        const int y0 = y + gy * sc;
        FillRect(img, x0, y0, x0 + sc - 1, y0 + sc - 1, color, mode);
      }
    }
  }
}

inline void DrawText5x7(RgbaImage& img, int x, int y, std::string_view s, Rgba8 color, int scale = 1,
                        int spacingPx = 1, BlendMode mode = BlendMode::Alpha)
{
  const int sc = std::max(1, scale);
  const int adv = (Font5x7GlyphW() + spacingPx) * sc;

  int cx = x;
  for (char c : s) {
    DrawGlyph5x7(img, cx, y, c, color, sc, mode);
    cx += adv;
  }
}

inline void DrawText5x7Outlined(RgbaImage& img, int x, int y, std::string_view s, Rgba8 fill, Rgba8 outline,
                               int scale = 1, int spacingPx = 1, BlendMode mode = BlendMode::Alpha)
{
  const int sc = std::max(1, scale);
  const int o = sc; // outline thickness in pixels

  // 4-neighborhood outline (keeps it readable without bloating too much).
  DrawText5x7(img, x - o, y, s, outline, sc, spacingPx, mode);
  DrawText5x7(img, x + o, y, s, outline, sc, spacingPx, mode);
  DrawText5x7(img, x, y - o, s, outline, sc, spacingPx, mode);
  DrawText5x7(img, x, y + o, s, outline, sc, spacingPx, mode);

  DrawText5x7(img, x, y, s, fill, sc, spacingPx, mode);
}

} // namespace gfx
} // namespace isocity
