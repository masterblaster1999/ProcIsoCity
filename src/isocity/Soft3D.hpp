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
