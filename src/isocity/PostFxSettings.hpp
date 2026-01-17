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

  // If true, render screen-space weather into the world RT so it is also
  // stylized by the post-processing shader. If false, weather is drawn after
  // compositing at full resolution.
  bool includeWeather = false;
};

} // namespace isocity
