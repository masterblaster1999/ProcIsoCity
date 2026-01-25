#pragma once

#include "isocity/PostFxSettings.hpp"

#include <cstdint>

#include "isocity/RaylibShim.hpp"

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

  // Force recompilation (useful when editing external override shaders).
  // Returns true if the shader is ready after reloading.
  bool reload();

  // Release GPU resources.
  void shutdown();

  bool ready() const { return m_ready; }
  // Compatibility alias (older code expected isReady()).
  bool isReady() const { return ready(); }
  bool failed() const { return m_failed; }

  // True if an on-disk override was used for the most recent successful compile.
  bool usedOverride() const { return m_usedOverride; }

  // Temporal AA stage (independent of the main PostFX shader).
  bool taaReady() const { return m_taaReady; }

  // Draw `tex` with post FX if enabled.
  // Falls back to normal DrawTexturePro when disabled.
  //
  // When TAA is enabled, callers should render the world with a small
  // per-frame camera jitter and pass that jitter here so the TAA stage can
  // cancel it before accumulation.
  void drawTexturePro(const Texture2D& tex, const Rectangle& src, const Rectangle& dst,
                      const PostFxSettings& settings, float timeSec, std::uint32_t seed,
                      Color tint = WHITE,
                      Vector2 taaJitterPixels = Vector2{0.0f, 0.0f},
                      bool taaResetHistory = false,
                      // Optional "gameplay" uniforms exposed to the PostFX shader.
                      // These are primarily used for lens precipitation (rain on lens) so
                      // the effect naturally tracks weather intensity and wind.
                      int weatherMode = 0,
                      float weatherIntensity = 0.0f,
                      Vector2 windDir = Vector2{0.0f, 1.0f},
                      float windSpeed = 1.0f);

  // Compatibility alias (older code expected drawTexture()).
  void drawTexture(const Texture2D& tex, const Rectangle& src, const Rectangle& dst,
                   const PostFxSettings& settings, float timeSec, std::uint32_t seed,
                   Color tint = WHITE,
                   Vector2 taaJitterPixels = Vector2{0.0f, 0.0f},
                   bool taaResetHistory = false) {
    drawTexturePro(tex, src, dst, settings, timeSec, seed, tint, taaJitterPixels, taaResetHistory);
  }

private:
  Shader m_shader{};
  bool m_ready = false;
  bool m_failed = false;
  bool m_usedOverride = false;

  // Optional temporal AA shader + history buffers.
  Shader m_taa{};
  bool m_taaReady = false;
  bool m_taaFailed = false;
  bool m_taaUsedOverride = false;

  int m_locTaaHistory = -1;
  int m_locTaaTexelSize = -1;
  int m_locTaaJitterUV = -1;
  int m_locTaaHistoryWeight = -1;
  int m_locTaaResponse = -1;
  int m_locTaaReset = -1;

  RenderTexture2D m_taaRT0{};
  RenderTexture2D m_taaRT1{};
  bool m_taaRTValid = false;
  bool m_taaRTAllocFailed = false;
  bool m_taaHistoryValid = false;
  int m_taaRTWidth = 0;
  int m_taaRTHeight = 0;

  // Optional bloom shaders + intermediate buffers.
  // Bloom is designed to be additive and independent of the main PostFX shader,
  // so a custom PostFX override can fail to compile without disabling bloom.
  Shader m_bloomExtract{};
  Shader m_bloomBlur{};
  bool m_bloomReady = false;
  bool m_bloomFailed = false;
  bool m_bloomUsedOverride = false;

  int m_locTime = -1;
  int m_locSeed = -1;
  int m_locBits = -1;
  int m_locDither = -1;
  int m_locGrain = -1;
  int m_locVignette = -1;
  int m_locChroma = -1;
  int m_locScanlines = -1;

  int m_locFxaa = -1;
  int m_locSharpen = -1;

  // Optional inline bloom uniforms (if present, bloom can be composited
  // inside the main PostFX shader before tonemapping/grading).
  int m_locPostBloomTex = -1;
  int m_locPostBloomStrength = -1;

  // Filmic tonemap / grade uniforms (optional).
  int m_locTonemapEnabled = -1;
  int m_locExposure = -1;
  int m_locContrast = -1;
  int m_locSaturation = -1;

  // Screen-space outline uniforms (optional).
  int m_locOutline = -1;
  int m_locOutlineThreshold = -1;
  int m_locOutlineThickness = -1;

  // Lens precipitation uniforms (optional).
  int m_locLensWeather = -1;
  int m_locLensDistort = -1;
  int m_locLensScale = -1;
  int m_locLensDrips = -1;

  // Weather uniforms (optional).
  int m_locWeatherMode = -1;
  int m_locWeatherIntensity = -1;
  int m_locWindDir = -1;
  int m_locWindSpeed = -1;

  // Optional uniforms for custom shaders (safe to ignore if not present).
  int m_locResolution = -1;
  int m_locTexelSize = -1;

  // Bloom extraction uniforms (optional).
  int m_locBloomThreshold = -1;
  int m_locBloomKnee = -1;
  int m_locBloomExtractResolution = -1;
  int m_locBloomExtractTexelSize = -1;

  // Bloom blur uniforms (optional).
  int m_locBloomBlurTexelSize = -1;
  int m_locBloomBlurDirection = -1;
  int m_locBloomBlurRadius = -1;

  // Bloom render targets (ping-pong).
  RenderTexture2D m_bloomRT0{};
  RenderTexture2D m_bloomRT1{};
  bool m_bloomRTValid = false;
  bool m_bloomRTAllocFailed = false;
  int m_bloomRTWidth = 0;
  int m_bloomRTHeight = 0;
};

} // namespace isocity
