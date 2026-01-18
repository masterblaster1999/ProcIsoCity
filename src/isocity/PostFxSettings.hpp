#pragma once

#include <cstdint>

namespace isocity {

// Purely-visual post-processing options applied when compositing the world
// render target (the main isometric view) to the window.
//
// Notes:
//  - UI is *not* post-processed so text stays crisp.
//  - When enabled, the game forces a world render target even at 1.0x scale.
struct PostFxSettings {
  bool enabled = false;

  // Per-channel quantization bits. 8 == effectively no quantization.
  // Range: [2, 8]
  int colorBits = 6;

  // Ordered dithering strength applied before quantization.
  // Range: [0, 1]
  float ditherStrength = 0.65f;

  // Small temporal grain (adds motion and hides banding).
  // Range: [0, 1]
  float grain = 0.08f;

  // Darken corners of the view.
  // Range: [0, 1]
  float vignette = 0.15f;

  // Simple chromatic aberration (radial RGB split).
  // Range: [0, 1]
  float chroma = 0.0f;

  // CRT-style scanlines.
  // Range: [0, 1]
  float scanlines = 0.0f;

  // Fast Approximate Anti-Aliasing (single pass).
  // Range: [0, 1]
  //
  // Recommended:
  //  - 0.0 for pixel-art look (no smoothing)
  //  - 0.15-0.35 for mild smoothing when using fractional world scaling
  float fxaa = 0.0f;

  // Unsharp-mask style sharpening applied after FXAA.
  // Range: [0, 1]
  //
  // Recommended:
  //  - 0.0 if using point filtering
  //  - 0.10-0.30 if using bilinear/trilinear filtering (restores perceived crispness)
  float sharpen = 0.0f;

  // ---------------------------------------------------------------------------
  // Filmic tonemap + grade
  // ---------------------------------------------------------------------------
  // Optional display-referred filmic tonemap + simple grade controls.
  //
  // This is intentionally lightweight and does not require HDR render targets.
  bool tonemapEnabled = false;

  // Exposure multiplier applied before tonemapping.
  // Range: [0, 4]
  float exposure = 1.0f;

  // Contrast around 0.5 after tonemapping.
  //  1.0 = neutral
  // Range: [0, 2]
  float contrast = 1.0f;

  // Saturation after tonemapping.
  //  1.0 = neutral, 0 = grayscale, >1 = more vivid
  // Range: [0, 2]
  float saturation = 1.0f;

  // ---------------------------------------------------------------------------
  // Screen-space outlines (edge enhancement)
  // ---------------------------------------------------------------------------
  // Cheap edge enhancement on the final image (luma discontinuity).
  //
  // This is useful for readability when zoomed out and complements the tile art style.
  float outline = 0.0f; // Range: [0, 1]

  // Luma edge threshold. Lower values make more edges appear.
  // Range: [0, 1]
  float outlineThreshold = 0.12f;

  // Sample radius in pixels (approx thickness).
  // Range: [0.5, 4]
  float outlineThickness = 1.0f;

  // ---------------------------------------------------------------------------
  // Temporal Anti-Aliasing (TAA-lite)
  // ---------------------------------------------------------------------------
  // Jittered subpixel sampling + history accumulation.
  //
  // This is designed to reduce shimmering on thin isometric edges when panning
  // or zooming (especially with fractional world render scaling).
  //
  // Notes:
  //  - The implementation is intentionally "TAA-lite": it has no motion vectors.
  //    It uses neighborhood clamping + a luminance-based responsiveness factor
  //    to reduce ghosting for moving sprites/particles.
  //  - Requires the world to be rendered into an off-screen target (handled
  //    automatically when Post FX is enabled).
  bool taaEnabled = false;

  // Base history weight (higher = more stable/less shimmering, but more ghosting).
  // Range: [0, 1]
  float taaHistory = 0.85f;

  // Jitter amplitude in pixels as a fraction of a half-pixel.
  //  0.0 => no jitter (still acts as a temporal smoother)
  //  1.0 => +/-0.5px jitter (recommended)
  // Range: [0, 1]
  float taaJitter = 1.0f;

  // Responsiveness: reduces history weight when the current frame disagrees with
  // history (higher = less ghosting, but less stability).
  // Range: [0, 1]
  float taaResponse = 0.65f;

  // If true, render screen-space weather into the world RT so it is also
  // stylized by the post-processing shader. If false, weather is drawn after
  // compositing at full resolution.
  bool includeWeather = false;

  // ---------------------------------------------------------------------------
  // Lens precipitation (rain on lens / wet camera)
  // ---------------------------------------------------------------------------
  // Optional screen-space distortion + highlights to simulate droplets and drips
  // on the "camera lens". The effect is driven by the current weather uniforms
  // passed to the PostFX shader (mode/intensity/wind) and multiplied by this
  // user strength.
  //
  // Range: [0, 1]
  float lensWeather = 0.0f;

  // Refraction amount (UV distortion) for the lens droplets.
  // Range: [0, 1]
  float lensDistort = 0.35f;

  // Scale of the droplet field (affects droplet size/density).
  // Range: [0.5, 2]
  float lensScale = 1.0f;

  // Drip/trail strength for running droplets.
  // Range: [0, 1]
  float lensDrips = 0.65f;

  // ---------------------------------------------------------------------------
  // Bloom (screen-space glow)
  // ---------------------------------------------------------------------------
  // This is implemented as a lightweight bright-pass + separable blur chain and
  // composited additively on top of the final image.

  // Overall bloom intensity.
  // Range: [0, 1]
  float bloom = 0.0f;

  // Brightness threshold for bloom extraction.
  // Range: [0, 1]
  float bloomThreshold = 0.78f;

  // Soft knee around the bloom threshold (higher = smoother transition).
  // Range: [0, 1]
  float bloomKnee = 0.25f;

  // Blur radius multiplier (in bloom-buffer pixels).
  // Range: [0.25, 4]
  float bloomRadius = 1.0f;

  // Downsample factor for the bloom buffer.
  //  1 = full-res, 2 = half, 4 = quarter (recommended), 8 = eighth.
  // Range: [1, 8]
  int bloomDownsample = 4;
};

} // namespace isocity
