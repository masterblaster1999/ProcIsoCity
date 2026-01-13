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

inline constexpr std::array kFont5x7 = {
  // Digits
  Glyph5x7{'0', {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110}},
  Glyph5x7{'1', {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
  Glyph5x7{'2', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}},
  Glyph5x7{'3', {0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110}},
  Glyph5x7{'4', {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}},
  Glyph5x7{'5', {0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110}},
  Glyph5x7{'6', {0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}},
  Glyph5x7{'7', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}},
  Glyph5x7{'8', {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}},
  Glyph5x7{'9', {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100}},

  // Uppercase letters
  Glyph5x7{'A', {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
  Glyph5x7{'B', {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110}},
  Glyph5x7{'C', {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110}},
  Glyph5x7{'D', {0b11100, 0b10010, 0b10001, 0b10001, 0b10001, 0b10010, 0b11100}},
  Glyph5x7{'E', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}},
  Glyph5x7{'F', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000}},
  Glyph5x7{'G', {0b01110, 0b10001, 0b10000, 0b10000, 0b10011, 0b10001, 0b01110}},
  Glyph5x7{'H', {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
  Glyph5x7{'I', {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
  Glyph5x7{'J', {0b00001, 0b00001, 0b00001, 0b00001, 0b10001, 0b10001, 0b01110}},
  Glyph5x7{'K', {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}},
  Glyph5x7{'L', {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}},
  Glyph5x7{'M', {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001}},
  Glyph5x7{'N', {0b10001, 0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001}},
  Glyph5x7{'O', {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
  Glyph5x7{'P', {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}},
  Glyph5x7{'Q', {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101}},
  Glyph5x7{'R', {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}},
  Glyph5x7{'S', {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}},
  Glyph5x7{'T', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
  Glyph5x7{'U', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
  Glyph5x7{'V', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100}},
  Glyph5x7{'W', {0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001}},
  Glyph5x7{'X', {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001}},
  Glyph5x7{'Y', {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100}},
  Glyph5x7{'Z', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111}},

  // Lowercase letters
  Glyph5x7{'a', {0b00000, 0b00000, 0b01110, 0b00001, 0b01111, 0b10001, 0b01111}},
  Glyph5x7{'b', {0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b10001, 0b11110}},
  Glyph5x7{'c', {0b00000, 0b00000, 0b01110, 0b10001, 0b10000, 0b10001, 0b01110}},
  Glyph5x7{'d', {0b00001, 0b00001, 0b01111, 0b10001, 0b10001, 0b10001, 0b01111}},
  Glyph5x7{'e', {0b00000, 0b00000, 0b01110, 0b10001, 0b11111, 0b10000, 0b01110}},
  Glyph5x7{'f', {0b00110, 0b01001, 0b01000, 0b11100, 0b01000, 0b01000, 0b01000}},
  Glyph5x7{'g', {0b00000, 0b01111, 0b10001, 0b10001, 0b01111, 0b00001, 0b01110}},
  Glyph5x7{'h', {0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b10001, 0b10001}},
  Glyph5x7{'i', {0b00100, 0b00000, 0b01100, 0b00100, 0b00100, 0b00100, 0b01110}},
  Glyph5x7{'j', {0b00010, 0b00000, 0b00110, 0b00010, 0b00010, 0b10010, 0b01100}},
  Glyph5x7{'k', {0b10000, 0b10000, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010}},
  Glyph5x7{'l', {0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
  Glyph5x7{'m', {0b00000, 0b00000, 0b11010, 0b10101, 0b10101, 0b10101, 0b10101}},
  Glyph5x7{'n', {0b00000, 0b00000, 0b11110, 0b10001, 0b10001, 0b10001, 0b10001}},
  Glyph5x7{'o', {0b00000, 0b00000, 0b01110, 0b10001, 0b10001, 0b10001, 0b01110}},
  Glyph5x7{'p', {0b00000, 0b00000, 0b11110, 0b10001, 0b10001, 0b11110, 0b10000}},
  Glyph5x7{'q', {0b00000, 0b00000, 0b01111, 0b10001, 0b10001, 0b01111, 0b00001}},
  Glyph5x7{'r', {0b00000, 0b00000, 0b10110, 0b11001, 0b10000, 0b10000, 0b10000}},
  Glyph5x7{'s', {0b00000, 0b00000, 0b01111, 0b10000, 0b01110, 0b00001, 0b11110}},
  Glyph5x7{'t', {0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00100, 0b00011}},
  Glyph5x7{'u', {0b00000, 0b00000, 0b10001, 0b10001, 0b10001, 0b10001, 0b01111}},
  Glyph5x7{'v', {0b00000, 0b00000, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100}},
  Glyph5x7{'w', {0b00000, 0b00000, 0b10001, 0b10001, 0b10101, 0b10101, 0b01010}},
  Glyph5x7{'x', {0b00000, 0b00000, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001}},
  Glyph5x7{'y', {0b00000, 0b00000, 0b10001, 0b10001, 0b01111, 0b00001, 0b01110}},
  Glyph5x7{'z', {0b00000, 0b00000, 0b11111, 0b00010, 0b00100, 0b01000, 0b11111}},

  // Punctuation / symbols (subset used by the in-game UI)
  Glyph5x7{' ', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}},
  Glyph5x7{'!', {0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000, 0b00100}},
  Glyph5x7{',', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00100, 0b01000}},
  Glyph5x7{'.', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b00110}},
  Glyph5x7{'-', {0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000}},
  Glyph5x7{'/', {0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b00000, 0b00000}},
  Glyph5x7{'\\', {0b10000, 0b01000, 0b00100, 0b00010, 0b00001, 0b00000, 0b00000}},
  Glyph5x7{':', {0b00000, 0b00110, 0b00110, 0b00000, 0b00110, 0b00110, 0b00000}},
  Glyph5x7{'?', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b00000, 0b00100}},
  Glyph5x7{'|', {0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
  Glyph5x7{'+', {0b00000, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00000}},
  Glyph5x7{'=', {0b00000, 0b00000, 0b11111, 0b00000, 0b11111, 0b00000, 0b00000}},
  Glyph5x7{'(', {0b00010, 0b00100, 0b01000, 0b01000, 0b01000, 0b00100, 0b00010}},
  Glyph5x7{')', {0b01000, 0b00100, 0b00010, 0b00010, 0b00010, 0b00100, 0b01000}},
  Glyph5x7{'[', {0b11110, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11110}},
  Glyph5x7{']', {0b01111, 0b00001, 0b00001, 0b00001, 0b00001, 0b00001, 0b01111}},
  Glyph5x7{'<', {0b00010, 0b00100, 0b01000, 0b10000, 0b01000, 0b00100, 0b00010}},
  Glyph5x7{'>', {0b01000, 0b00100, 0b00010, 0b00001, 0b00010, 0b00100, 0b01000}},
  Glyph5x7{'*', {0b00000, 0b01010, 0b00100, 0b11111, 0b00100, 0b01010, 0b00000}},
  Glyph5x7{'_', {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111}},
  Glyph5x7{'$', {0b00100, 0b01111, 0b10100, 0b01110, 0b00101, 0b11110, 0b00100}},
  Glyph5x7{'%', {0b11001, 0b11010, 0b00100, 0b01011, 0b10011, 0b00000, 0b00000}},
  // More punctuation used by console/scripts/UI
  Glyph5x7{'"',  {0b01010, 0b01010, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}},
  Glyph5x7{'\'' ,{0b00100, 0b00100, 0b01000, 0b00000, 0b00000, 0b00000, 0b00000}},
  Glyph5x7{';',  {0b00000, 0b00110, 0b00110, 0b00000, 0b00110, 0b00100, 0b01000}},
  Glyph5x7{'#',  {0b01010, 0b11111, 0b01010, 0b11111, 0b01010, 0b00000, 0b00000}},
  Glyph5x7{'&',  {0b01100, 0b10010, 0b10100, 0b01000, 0b10101, 0b10010, 0b01101}},
  Glyph5x7{'@',  {0b01110, 0b10001, 0b10111, 0b10101, 0b10111, 0b10000, 0b01110}},
  Glyph5x7{'^',  {0b00100, 0b01010, 0b10001, 0b00000, 0b00000, 0b00000, 0b00000}},
  Glyph5x7{'~',  {0b00000, 0b00000, 0b01001, 0b10110, 0b00000, 0b00000, 0b00000}},
  Glyph5x7{'{',  {0b00011, 0b00100, 0b00100, 0b01000, 0b00100, 0b00100, 0b00011}},
  Glyph5x7{'}',  {0b11000, 0b00100, 0b00100, 0b00010, 0b00100, 0b00100, 0b11000}},
  Glyph5x7{'`',  {0b01000, 0b00100, 0b00010, 0b00000, 0b00000, 0b00000, 0b00000}},
};

inline constexpr int Font5x7GlyphW() { return 5; }
inline constexpr int Font5x7GlyphH() { return 7; }

inline const std::array<std::uint8_t, 7>& GetGlyphRows5x7(char c)
{
  // Prefer an exact match first (so lowercase can be distinct), then fall back to uppercase.
  for (const auto& g : kFont5x7) {
    if (g.ch == c) return g.rows;
  }

  const unsigned char uc = static_cast<unsigned char>(c);
  const char up = static_cast<char>(std::toupper(uc));
  if (up != c) {
    for (const auto& g : kFont5x7) {
      if (g.ch == up) return g.rows;
    }
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
