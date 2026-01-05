#pragma once

#include "isocity/Export.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Atlas post-processing helpers
// -----------------------------------------------------------------------------------------------
//
// These helpers generate *derived* textures from an RGBA sprite:
//  - Height maps (grayscale)
//  - Normal maps (RGB encoded unit vectors)
//  - Shadow masks (alpha-only, stored as RGBA for convenience)
//
// They are intentionally dependency-free and deterministic, intended for:
//  - toolchains (CLI)
//  - mods / external renderers
//  - CI artifact generation

enum class GfxHeightMode : std::uint8_t {
  // Height purely from alpha coverage.
  Alpha = 0,

  // Height from pixel luminance, multiplied by alpha.
  Luma = 1,

  // Height mostly from alpha, with a small luminance modulation to preserve
  // micro-detail (useful for terrain noise / brick patterns).
  AlphaLuma = 2,
};

const char* GfxHeightModeName(GfxHeightMode m);
bool ParseGfxHeightMode(const std::string& s, GfxHeightMode& out);

struct GfxNormalMapConfig {
  // How to derive the height field.
  GfxHeightMode heightMode = GfxHeightMode::AlphaLuma;

  // Strength of the x/y gradients relative to the z axis.
  // Larger values make the normals "steeper".
  float strength = 2.0f;
};

struct GfxShadowConfig {
  // Shadow direction in pixel space (does not need to be normalized).
  // (1,1) roughly corresponds to a light coming from the top-left.
  float dirX = 1.0f;
  float dirY = 1.0f;

  // Maximum shadow offset in pixels for the highest pixels.
  float lengthPx = 18.0f;

  // Simple box-blur radius in pixels (0 disables blur).
  int blurRadiusPx = 2;

  // Overall opacity multiplier in [0,1].
  float opacity = 0.70f;
};

// Generate a grayscale height map (RGB = height, A = source alpha).
bool GenerateHeightMap(const RgbaImage& src, GfxHeightMode mode, RgbaImage& outHeight, std::string& outError);

// Generate a tangent-space style normal map (RGB = encoded normal, A = source alpha).
// Convention: "green up" (OpenGL-style), i.e. +Y points toward the top of the image.
bool GenerateNormalMap(const RgbaImage& src, const GfxNormalMapConfig& cfg, RgbaImage& outNormal, std::string& outError);

// Generate a soft shadow mask (RGB=0, A=shadow).
// This is a heuristic intended for 2D sprite rendering (not physically based).
bool GenerateShadowMap(const RgbaImage& src, const GfxShadowConfig& cfg, RgbaImage& outShadow, std::string& outError);

struct GfxSdfConfig {
  // Maximum absolute signed distance in pixels encoded by the field.
  // The output is encoded as:
  //   v = clamp(0.5 + signedDistancePx / spreadPx, 0, 1)
  float spreadPx = 8.0f;

  // Alpha threshold in [0,1] used to classify pixels as inside/outside.
  float alphaThreshold = 0.5f;

  // If true, output alpha is forced to 255 so the field is visible everywhere.
  // If false, output alpha is copied from the source.
  bool opaqueAlpha = true;
};

// Generate a signed distance field (SDF) texture (RGB = SDF, A = 255 or source alpha).
// Convention: 0.5 corresponds to the silhouette boundary.
bool GenerateSignedDistanceField(const RgbaImage& src, const GfxSdfConfig& cfg, RgbaImage& outSdf, std::string& outError);

} // namespace isocity
