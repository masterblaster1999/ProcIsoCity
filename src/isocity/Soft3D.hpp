#pragma once

#include "isocity/Export.hpp"            // PpmImage
#include "isocity/WorldMeshBuilder.hpp"  // MeshQuad

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Minimal, dependency-free software 3D rasterizer.
//
// This is intended for *offline* rendering / exports (CLI tools, tests) so we
// can generate shaded 3D views (orthographic/isometric or perspective) without
// relying on GPU APIs or third-party libs.

struct Soft3DCamera {
  enum class Projection : std::uint8_t {
    Orthographic = 0,
    Perspective = 1,
  };

  // Camera Euler angles (degrees).
  //
  // - yaw: rotation around +Y (up)
  // - pitch: positive pitches camera upward (camera is above target when pitch>0)
  // - roll: rotation around view forward axis
  float yawDeg = 45.0f;
  float pitchDeg = 35.264f; // classic isometric pitch
  float rollDeg = 0.0f;

  // Camera target (world units).
  float targetX = 0.0f;
  float targetY = 0.0f;
  float targetZ = 0.0f;

  // Distance from camera target.
  float distance = 120.0f;

  Projection projection = Projection::Orthographic;

  // Perspective projection parameters.
  float fovYDeg = 45.0f;

  // Orthographic projection parameter.
  // Defines the half-height of the view volume in world units.
  float orthoHalfHeight = 20.0f;

  // Near/far planes in world units.
  float nearZ = 0.25f;
  float farZ = 5000.0f;

  // If enabled, the camera is automatically fit to the provided geometry
  // bounds (target, distance/orthoHalfHeight, and near/far are derived).
  bool autoFit = true;
  float fitMargin = 0.08f; // fraction of bounds to pad
};

struct Soft3DShading {
  // Directional light direction (world-space, does not need to be normalized).
  // This is the *direction from the surface towards the light*.
  float lightDirX = -0.55f;
  float lightDirY = 0.80f;
  float lightDirZ = -0.25f;

  // Lambert lighting terms.
  float ambient = 0.35f;
  float diffuse = 0.65f;

  // Background clear color.
  std::uint8_t bgR = 30;
  std::uint8_t bgG = 32;
  std::uint8_t bgB = 42;

  // Fog blend target color (RGB). Defaults to a cool gray.
  std::uint8_t fogR = 200;
  std::uint8_t fogG = 210;
  std::uint8_t fogB = 225;

  // Simple depth-based fog.
  bool enableFog = false;
  float fogStrength = 0.35f;
  // fogStart/fogEnd are in depth-buffer units [0..1].
  float fogStart = 0.35f;
  float fogEnd = 1.0f;
};

// Optional post-processing for Soft3D renders.
//
// These are intended to improve the readability / "game-art" look of the
// exported isometric renders while staying dependency-free and deterministic.
struct Soft3DPostFxConfig {
  // When SSAA is enabled, downsampling in sRGB space tends to produce
  // overly-dark results (because sRGB is non-linear). When enabled, the
  // resolve pass converts to linear light, averages, then converts back.
  bool gammaCorrectDownsample = true;

  // --- Screen-space ambient occlusion (SSAO-ish) ---
  // Depth-only approximation (fast, stable, no normals required).
  bool enableAO = false;
  float aoStrength = 0.55f; // 0..1 multiplier applied to occlusion
  int aoRadiusPx = 7;       // sampling radius in pixels
  float aoRange = 0.02f;    // max depth delta in [0..1] considered for occlusion
  float aoBias = 0.0015f;   // small bias to reduce self-occlusion
  float aoPower = 1.25f;    // contrast curve on the final occlusion
  int aoSamples = 12;       // number of samples per pixel (4..32 typical)
  int aoBlurRadiusPx = 1;   // 0 disables, 1 is a small 3-tap blur (separable)

  // --- Depth-based edge outlines ---
  // Finds depth discontinuities and blends an outline color on top.
  bool enableEdge = false;
  float edgeAlpha = 0.90f;     // 0..1 blend over the image
  float edgeThreshold = 0.004f; // depth delta threshold in [0..1]
  float edgeSoftness = 0.003f; // smoothstep width in [0..1]
  int edgeRadiusPx = 1;        // dilation radius in pixels (>=1)
  std::uint8_t edgeR = 0;
  std::uint8_t edgeG = 0;
  std::uint8_t edgeB = 0;

  // --- Tonemap / grade ---
  bool enableTonemap = false;
  float exposure = 1.0f;   // linear multiplier
  float contrast = 1.0f;   // 1=identity
  float saturation = 1.0f; // 1=identity
  float vignette = 0.0f;   // 0..1

  // --- Bloom (bright-pass + blur) ---
  // Applied in linear space after tonemap and before the final color conversion.
  bool enableBloom = false;
  float bloomStrength = 0.18f;  // additive blend amount
  float bloomRadius = 0.80f;    // normalized blur amount (0..1 typical)
  float bloomThreshold = 0.75f; // bright-pass threshold in [0,1]

  // --- Ordered dithering + quantization ---
  bool enableDither = false;
  float ditherStrength = 0.35f; // 0..1
  int ditherBits = 6;           // bits per channel in [1..8]

  // Seed for deterministic noise/jitter in post (AO sample rotation + dithering).
  // 0 means "derive from content" (caller may override).
  std::uint32_t postSeed = 0;
};

struct Soft3DRenderConfig {
  int width = 1280;
  int height = 720;

  // Supersampling factor (1 = off). Render at (width*SSAA, height*SSAA) then
  // downsample with a box filter.
  int supersample = 1;

  // Optional outlines (wireframe) drawn after fill with a depth test.
  bool drawOutlines = true;
  std::uint8_t outlineR = 0;
  std::uint8_t outlineG = 0;
  std::uint8_t outlineB = 0;
  // Outline alpha blended over the filled surface.
  float outlineAlpha = 1.0f;
  float outlineDepthEps = 0.002f;

  // Optional post-processing. All features are off by default except
  // gamma-correct SSAA resolve.
  Soft3DPostFxConfig postFx{};
};

// Render a set of quads (each treated as two triangles) into a PPM image using
// a software z-buffer.
//
// The camera can be auto-fitted to the geometry bounds; pass outBoundsMin/outBoundsMax
// to retrieve those bounds for UI/debugging.
//
// On error, returns an empty image (width/height 0) and (optionally) fills outError.
PpmImage RenderQuadsSoft3D(const std::vector<MeshQuad>& quads,
                           Soft3DCamera cam,
                           const Soft3DShading& shade,
                           const Soft3DRenderConfig& cfg,
                           MeshV3* outBoundsMin = nullptr,
                           MeshV3* outBoundsMax = nullptr,
                           std::string* outError = nullptr);

} // namespace isocity
