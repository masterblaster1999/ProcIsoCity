#pragma once

#include <cstdint>
#include <vector>

namespace isocity {

// Procedural terrain shaping configuration.
//
// This stage operates on the generated heightfield before tile terrain is
// classified (water/sand/grass). It is deliberately deterministic so it can be
// used for delta-saves / regeneration and CI-style headless runs.
struct ErosionConfig {
  // Master enable.
  bool enabled = true;

  // If enabled, carve river-like channels using a simple flow-accumulation
  // model.
  bool riversEnabled = true;

  // --- Thermal erosion ---
  // Number of iterations.
  int thermalIterations = 20;

  // Minimum height delta to trigger material movement.
  float thermalTalus = 0.02f;

  // Movement rate (0..1). Higher values converge faster but can over-flatten.
  float thermalRate = 0.50f;

  // --- Rivers ---
  // Minimum flow accumulation to be considered a river cell.
  // If <= 0, an automatic threshold is chosen based on map size.
  int riverMinAccum = 0;

  // Carve strength (height units).
  float riverCarve = 0.055f;

  // Exponent applied to normalized accumulation.
  float riverCarvePower = 0.60f;

  // --- Smoothing ---
  int smoothIterations = 1;
  float smoothRate = 0.25f;

  // --- Quantization ---
  // If > 0, heights are quantized to 1/quantizeScale increments.
  int quantizeScale = 4096;
};

// Apply erosion + rivers + smoothing to a heightfield in-place.
//
// - heights: row-major [y*w + x]
// - seed: used only for deterministic tie-breaking / minor jitter.
void ApplyErosion(std::vector<float>& heights, int w, int h, const ErosionConfig& cfg, std::uint64_t seed);

} // namespace isocity
