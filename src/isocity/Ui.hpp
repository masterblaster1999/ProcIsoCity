#pragma once

#include "isocity/RaylibShim.hpp"

#include <cstdint>
#include <string_view>
#include <algorithm>
#include <cmath>

namespace isocity {
namespace ui {

// -----------------------------------------------------------------------------------------------
// Theme + font tuning
// -----------------------------------------------------------------------------------------------

// A small, coherent theme (colors + geometric styling) for the in-game UI.
//
// Values are generated procedurally from a seed in ui::Init()/ui::SetSeed().
struct Theme {
  // Panel "gradient" colors (we approximate with layered fills).
  Color panelBgTop = Color{22, 24, 30, 235};
  Color panelBgBot = Color{10, 12, 16, 235};

  // Border + text colors.
  Color panelBorder = Color{255, 255, 255, 70};
  Color panelBorderHi = Color{255, 255, 255, 110};
  Color text = Color{240, 240, 240, 255};
  Color textDim = Color{210, 210, 210, 255};
  Color textFaint = Color{170, 170, 170, 255};

  // Seed-derived accent (used for headers, highlights).
  Color accent = Color{120, 220, 255, 255};
  Color accentDim = Color{120, 220, 255, 90};

  // Bright/highlight accent (e.g. selected rows).
  // This is derived from `accent` and intentionally semi-transparent.
  Color accentHi = Color{170, 240, 255, 110};

  // Semantic accents used by charts and status badges.
  // These are intentionally not user-editable (they communicate meaning).
  Color accentOk = Color{90, 220, 150, 255};
  Color accentBad = Color{230, 90, 90, 255};

  // Back-compat alias used by some UI call-sites (older code expects theme.bad).
  Color bad = Color{230, 90, 90, 255};

  // Subtle gridline color (used for charts/tables).
  Color grid = Color{255, 255, 255, 30};

  // Geometry.
  float roundness = 0.18f;  // 0..1
  int roundSegments = 8;

  // Noise overlay.
  float noiseAlpha = 0.06f;        // 0..1
  float noiseScale = 0.75f;        // 1.0 => 1 noise texel per UI pixel (before tiling)
  float headerSheenStrength = 0.35f; // 0..1
};

// User-tweakable UI parameters (persistent via VisualPrefs).
//
// These intentionally map 1:1 to VisualPrefs::UiThemePrefs so the game can
// autosave them to `isocity_visual.json`.
struct Settings {
  // Accent selection.
  bool accentFromSeed = true;
  float accentHueDeg = 210.0f;     // 0..360
  float accentSaturation = 0.55f;  // 0..1
  float accentValue = 0.95f;       // 0..1

  // Panel geometry + effects.
  float roundness = 0.18f;       // 0..1 (raylib's DrawRectangleRounded roundness)
  int roundSegments = 8;

  float noiseAlpha = 0.06f;          // 0..1
  float noiseScale = 0.75f;          // tiling density
  float headerSheenStrength = 0.35f; // 0..1

  // Font atlas generation.
  int fontAtlasScale = 3;        // 1..8
  bool fontFilterPoint = false;  // point vs bilinear
};

// Initialize/shutdown the procedural UI system.
// Safe to call multiple times (Init/Shutdown ref-count internally).
void Init(std::uint64_t seed = 0);
void Shutdown();
bool IsReady();

// Change the procedural seed (updates accent palette + noise pattern).
void SetSeed(std::uint64_t seed);
const Theme& GetTheme();

// Apply/get the current UI settings.
// Safe to call before Init(); settings will be applied on the first Init().
Settings GetSettings();
void SetSettings(const Settings& s);
void ResetSettings();

// Draw a raised panel frame (rounded, subtle highlights + noise).
void DrawPanel(Rectangle r, float timeSec, bool active = true);

// Draw a slightly inset panel (used for list boxes / sub-panels).
void DrawPanelInset(Rectangle r, float timeSec, bool active = true);

// Convenience: header text + accent bar.
void DrawPanelHeader(Rectangle panel, std::string_view title, float timeSec, bool active = true,
                     int titleSizePx = 20);

// Selection highlight rectangle (e.g., current row).
void DrawSelectionHighlight(Rectangle r, float timeSec, bool strong = false);

// Text drawing using a procedurally generated bitmap atlas.
// `sizePx` is the requested font height in UI pixels.
void Text(int x, int y, int sizePx, std::string_view text, Color color,
          bool bold = false, bool shadow = true, int spacingPx = 1);

void TextCentered(Rectangle r, int sizePx, std::string_view text, Color color,
                  bool bold = false, bool shadow = true, int spacingPx = 1);

int MeasureTextWidth(std::string_view text, int sizePx, bool bold = false, int spacingPx = 1);
int MeasureTextHeight(int sizePx);


// Draw text with a 1px outline (useful on bright backgrounds).
void TextOutlined(int x, int y, int sizePx, std::string_view text, Color fill, Color outline,
                  bool bold = false, bool shadow = true, int spacingPx = 1);

// Draw wrapped/clipped multi-line text inside a rectangle.
// Returns the Y position after the last rendered line (useful for stacked layouts).
int TextBox(Rectangle r, int sizePx, std::string_view text, Color color,
            bool bold = false, bool shadow = true, int spacingPx = 1,
            bool wrap = true, bool clip = true);

// Measure multi-line height (accounts for '\n' and optional wrapping).
int MeasureTextHeight(std::string_view text, int sizePx, bool bold = false, int spacingPx = 1,
                      int wrapWidthPx = 0);

// Procedural "keycap" widgets (used by the help overlay and UI hints).
// Returns the width consumed in pixels.
int DrawKeycap(int x, int y, std::string_view label, float timeSec, bool strong = false, int sizePx = 16);

// Draw a key combo like "Ctrl+Shift+F3" as multiple keycaps separated by '+'.
// Returns the width consumed in pixels.
int DrawKeyCombo(int x, int y, std::string_view combo, float timeSec, bool strong = false, int sizePx = 16);

// -----------------------------------------------------------------------------------------------
// Tiny immediate-mode widgets (used by the in-game settings panels).
// -----------------------------------------------------------------------------------------------

// Clear any active/dragged widget (useful when closing a panel).
void ClearActiveWidget();

// Toggle switch.
// Returns true if the value changed this frame.
bool Toggle(int id, Rectangle r, bool& ioValue, Vector2 mouseUi, float timeSec, bool enabled = true);

// Horizontal slider.
// Returns true if the value changed this frame.
bool SliderFloat(int id, Rectangle r, float& ioValue, float minValue, float maxValue,
                 Vector2 mouseUi, float timeSec, bool enabled = true);

// Convenience overload: quantized float slider (step size in value units).
inline bool SliderFloat(int id, Rectangle r, float& ioValue, float minValue, float maxValue,
                        float step, Vector2 mouseUi, float timeSec, bool enabled = true)
{
  const bool changed = SliderFloat(id, r, ioValue, minValue, maxValue, mouseUi, timeSec, enabled);
  if (!(step > 0.0f) || !std::isfinite(step)) {
    return changed;
  }
  const float q = std::round(ioValue / step) * step;
  const float clamped = std::clamp(q, minValue, maxValue);
  if (std::abs(clamped - ioValue) > 1e-6f) {
    ioValue = clamped;
    return true;
  }
  return changed;
}

// Integer slider (snaps to step).
bool SliderInt(int id, Rectangle r, int& ioValue, int minValue, int maxValue, int step,
               Vector2 mouseUi, float timeSec, bool enabled = true);

// Convenience overload: int slider with implicit step=1.
inline bool SliderInt(int id, Rectangle r, int& ioValue, int minValue, int maxValue,
                      Vector2 mouseUi, float timeSec, bool enabled = true)
{
  return SliderInt(id, r, ioValue, minValue, maxValue, 1, mouseUi, timeSec, enabled);
}

// 64-bit unsigned integer slider (snaps to step).
bool SliderU64(int id, Rectangle r, std::uint64_t& ioValue, std::uint64_t minValue, std::uint64_t maxValue,
               std::uint64_t step, Vector2 mouseUi, float timeSec, bool enabled = true);

// Convenience overload: u64 slider with implicit step=1.
inline bool SliderU64(int id, Rectangle r, std::uint64_t& ioValue, std::uint64_t minValue, std::uint64_t maxValue,
                      Vector2 mouseUi, float timeSec, bool enabled = true)
{
  return SliderU64(id, r, ioValue, minValue, maxValue, 1ULL, mouseUi, timeSec, enabled);
}

// Simple button.
// Returns true if clicked this frame.
bool Button(int id, Rectangle r, std::string_view label, Vector2 mouseUi, float timeSec,
            bool enabled = true, bool primary = false);

// Simple non-interactive progress bar.
//
// - frac01: progress fraction in [0,1] (clamped defensively).
// - fill: base fill color (theme accent works well).
// - active: if false, the bar is drawn "disabled".
void ProgressBar(Rectangle r, float frac01, Color fill, float timeSec, bool active = true);

// Vertical scrollbar (generic "content units" model).
//
// - contentUnits: total length (e.g. total rows, or content height in pixels)
// - viewUnits: visible length (e.g. visible rows, or viewport height in pixels)
// - ioScrollUnits: current scroll offset from the top in [0, contentUnits-viewUnits]
//
// Returns true if scroll changed this frame. The function also draws the scrollbar.
bool ScrollbarV(int id, Rectangle barR, int contentUnits, int viewUnits, int& ioScrollUnits,
                Vector2 mouseUi, float timeSec, bool enabled = true);

// Convenience helper: shrink a rectangle so content doesn't render under a vertical scrollbar.
Rectangle ContentRectWithScrollbar(Rectangle r, float scrollbarW = 12.0f, float gap = 2.0f);

} // namespace ui
} // namespace isocity
