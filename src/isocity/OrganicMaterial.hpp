#pragma once

#include <array>
#include <cstdint>
#include <vector>

// This module is renderer-side (raylib/OpenGL). It is not part of isocity_core.
#include "isocity/RaylibShim.hpp"

namespace isocity {

// Asset-free, animated "organic material" generator.
//
// Implementation: a tiny Gray-Scott reaction-diffusion simulation running on a
// low-resolution grid, mapped onto an isometric diamond tile decal texture.
//
// The goal is to provide a believable, moving organic surface (moss/slime/
// mycelium/biolume) without requiring external textures.
class OrganicMaterial {
public:
  enum class Style : std::uint8_t {
    Moss = 0,
    Slime = 1,
    Mycelium = 2,
    Bioluminescent = 3,
  };

  struct Settings {
    bool enabled = false;
    Style style = Style::Moss;

    // Global overlay opacity (0..1). Actual per-tile opacity is additionally
    // scaled by a per-tile procedural "coverage" heuristic.
    float alpha = 0.25f;

    // Simulation speed multiplier.
    float speed = 1.0f;

    // How many RD steps to run per frame at ~60fps.
    int stepsPerFrame = 4;

    // Gray-Scott parameters (defaults yield blobby, living patterns).
    float diffusionU = 0.16f;
    float diffusionV = 0.08f;
    float feed = 0.035f;
    float kill = 0.065f;

    // Sampling scale from simulation -> tile texture.
    // >1 => more repeats across the tile (finer detail), <1 => larger blobs.
    float patternScale = 1.0f;

    // Additive "glow" pass at night for Bioluminescent style.
    bool glowAtNight = true;
    float glowStrength = 0.35f; // 0..1
  };

  OrganicMaterial() = default;
  ~OrganicMaterial();

  OrganicMaterial(const OrganicMaterial&) = delete;
  OrganicMaterial& operator=(const OrganicMaterial&) = delete;

  void init(int tileW, int tileH, std::uint32_t seed);
  void shutdown();

  bool isReady() const { return m_ready; }
  int tileW() const { return m_tileW; }
  int tileH() const { return m_tileH; }

  // Reseed + reinitialize simulation.
  void reset(std::uint32_t seed);

  // Advance simulation and refresh decal textures.
  void update(float dtSec, float timeSec, const Settings& s);

  // 0..kVariants-1
  static constexpr int kVariants = 4;

  const Texture2D& variantTex(int idx) const;

private:
  static constexpr int kSimSize = 256; // power-of-two for cheap wrapping

  struct Variant {
    Texture2D tex{};
    std::vector<Color> pixels;
    int ofsX = 0;
    int ofsY = 0;
    int rot = 0; // 0..3 => 0/90/180/270 deg sampling rotation
  };

  bool m_ready = false;

  int m_tileW = 0;
  int m_tileH = 0;
  std::uint32_t m_seed = 0;

  // Reaction-diffusion fields (u,v) + scratch buffers.
  std::vector<float> m_u;
  std::vector<float> m_v;
  std::vector<float> m_u2;
  std::vector<float> m_v2;

  // Neighbor lookup tables for periodic boundaries.
  std::array<int, kSimSize> m_xL{};
  std::array<int, kSimSize> m_xR{};
  std::array<int, kSimSize> m_yU{};
  std::array<int, kSimSize> m_yD{};

  std::array<Variant, kVariants> m_var;

  // Timekeeping for stable simulation speed.
  float m_timeAccum = 0.0f;
  float m_injectAccum = 0.0f;
  std::uint64_t m_stepCounter = 0;

  void initNeighbors();
  void warmStart(const Settings& s);
  void seedBlobs(std::uint32_t seed);
  void injectBlob(std::uint32_t seed);
  void stepOnce(const Settings& s);

  float sampleV(float fx, float fy) const;
  void rebuildVariantTexture(int idx, const Settings& s, float timeSec);
};

} // namespace isocity
