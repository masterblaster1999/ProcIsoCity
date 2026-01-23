#pragma once

#include "isocity/Goods.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A deterministic, lightweight "soundscape" / noise-pollution model.
//
// The city simulation already computes commute traffic and goods flow on the
// road network. This module combines those signals with a few simple land-use
// sources (industry, commerce) and sinks (parks, water) to produce a per-tile
// noise field in [0,1].
//
// Intended uses:
//  - an export layer (heatmap) for quick visual debugging
//  - a column in tile_metrics.csv for external analysis
//
// This is intentionally not a full acoustic simulation; it is a fast,
// predictable heuristic that is "good enough" for gameplay tuning.

struct NoiseConfig {
  // Influence radius in tiles. Larger values make noise spread further.
  int radius = 10;

  // How quickly noise decays with Manhattan distance.
  // weight = 1 / (1 + manhattanDistance * decayPerTile)
  float decayPerTile = 1.0f;

  // Source strengths (added to the emission field and clamped).
  float roadBase = 0.30f;
  float roadClassBoost = 0.20f;   // additional boost for road level (Avenue/Highway)
  float commuteTrafficBoost = 0.55f;
  float goodsTrafficBoost = 0.35f;

  float industrialSource = 0.85f;
  float commercialSource = 0.40f;

  // Negative sources (sinks). These reduce noise locally.
  float parkSink = 0.35f;
  float waterSink = 0.12f;

  // Clamp for the intermediate emission map.
  float emissionClamp = 1.0f;
};

struct NoiseResult {
  int w = 0;
  int h = 0;

  // Flat array of per-tile noise in [0,1], size w*h.
  std::vector<float> noise01;

  // Max value in noise01 (useful for debugging/telemetry).
  float maxNoise = 0.0f;
};

// Compute per-tile noise in [0,1].
//
// traffic/goods are optional; when null, roads still contribute noise based on
// road class only.
NoiseResult ComputeNoisePollution(const World& world, const NoiseConfig& cfg = {},
                                 const TrafficResult* traffic = nullptr,
                                 const GoodsResult* goods = nullptr);

} // namespace isocity
