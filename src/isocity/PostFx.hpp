#pragma once

#include "isocity/PostFxSettings.hpp"

#include <cstdint>

#include "raylib.h"

namespace isocity {

// Minimal shader-based post-processing pipeline for stylized rendering.
//
// This intentionally focuses on effects that work well for procedural art:
//  - ordered dithering + per-channel quantization
//  - subtle temporal grain
//  - vignette
//  - optional chromatic aberration + scanlines
//
// It is applied when drawing the world render target to the window.
class PostFxPipeline {
public:
  PostFxPipeline() = default;
  ~PostFxPipeline();

  PostFxPipeline(const PostFxPipeline&) = delete;
  PostFxPipeline& operator=(const PostFxPipeline&) = delete;

  // Compile the shader and cache uniform locations.
  // Safe to call multiple times.
  void init();

  // Release GPU resources.
  void shutdown();

  bool ready() const { return m_ready; }
  // Compatibility alias (older code expected isReady()).
  bool isReady() const { return ready(); }
  bool failed() const { return m_failed; }

  // Draw `tex` with post FX if enabled + shader is ready.
  // Falls back to normal DrawTexturePro when disabled or unavailable.
  void drawTexturePro(const Texture2D& tex, const Rectangle& src, const Rectangle& dst,
                      const PostFxSettings& settings, float timeSec, std::uint32_t seed,
                      Color tint = WHITE);

  // Compatibility alias (older code expected drawTexture()).
  void drawTexture(const Texture2D& tex, const Rectangle& src, const Rectangle& dst,
                   const PostFxSettings& settings, float timeSec, std::uint32_t seed,
                   Color tint = WHITE) {
    drawTexturePro(tex, src, dst, settings, timeSec, seed, tint);
  }

private:
  Shader m_shader{};
  bool m_ready = false;
  bool m_failed = false;

  int m_locTime = -1;
  int m_locSeed = -1;
  int m_locBits = -1;
  int m_locDither = -1;
  int m_locGrain = -1;
  int m_locVignette = -1;
  int m_locChroma = -1;
  int m_locScanlines = -1;
};

} // namespace isocity
