#pragma once

#include "isocity/RaylibShim.hpp"

#include <vector>

namespace isocity {

// GPU-side procedural geometry helpers.
//
// This module focuses on lightweight geometry-shader effects that can generate
// thick, anti-aliased "ribbons" from line segments.
//
// The interactive renderer uses this to draw smooth, animated path highlights
// (inspect path and road-drag preview) without per-tile outlines.
struct RibbonStyle {
  // Core ribbon width in screen pixels.
  float coreThicknessPx = 6.0f;

  // Glow pass thickness in screen pixels.
  float glowThicknessPx = 14.0f;

  // Alpha multipliers (0..1) for the core and glow passes.
  float coreAlpha = 0.85f;
  float glowAlpha = 0.18f;

  // Dash pattern configuration in screen pixels.
  float dashLengthPx = 26.0f;
  float dashSpeedPx = 42.0f; // px/sec

  // 0..1 fraction of the dash length that is "on".
  float dashDuty = 0.55f;

  // 0..1 strength of a subtle moving center highlight.
  float flowStrength = 0.35f;
};

// Draws a polyline as a thick ribbon using a geometry shader.
//
// This is intentionally tiny and self-contained so it can be optional:
// if geometry shaders are not supported on the current graphics backend,
// init() will fail and isReady() will remain false.
class GpuRibbonPathRenderer {
public:
  void init();
  void shutdown();

  bool isReady() const { return m_ready; }

  // Draw a connected polyline in world space.
  //
  // Expected usage:
  //   BeginMode2D(camera);
  //   gpuRibbon.drawPath(points, GetScreenWidth(), GetScreenHeight(), time, color, style);
  //   EndMode2D();
  void drawPath(const std::vector<Vector2>& points,
                int screenW, int screenH,
                float timeSec,
                Color baseColor,
                const RibbonStyle& style,
                bool additiveBlend = true) const;

private:
  Shader m_shader{};
  bool m_ready = false;

  int m_locScreenSize = -1;
  int m_locThickness = -1;
  int m_locTime = -1;
  int m_locDashLen = -1;
  int m_locDashSpeed = -1;
  int m_locDashDuty = -1;
  int m_locFlowStrength = -1;
};

} // namespace isocity
