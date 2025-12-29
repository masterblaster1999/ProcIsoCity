#pragma once

#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A lightweight "city builder" style land value / amenity analysis.
//
// This module is intentionally:
//  - deterministic (no randomness)
//  - derived-only (NOT persisted in saves)
//  - headless (no raylib dependency)
//
// It's used today for heatmap overlays, and provides a solid foundation for
// future simulation hooks (e.g. desirability-driven growth).

struct LandValueConfig {
  // Manhattan distance influence radii.
  int parkRadius = 8;      // positive amenity around parks
  int waterRadius = 6;     // positive amenity near coasts
  int pollutionRadius = 7; // negative influence around industrial zones

  // Weights applied to the normalized influences (0..1).
  float base = 0.35f;
  float parkBonus = 0.35f;
  float waterBonus = 0.15f;
  float pollutionPenalty = 0.30f;
  float trafficPenalty = 0.25f;

  // Small penalties to make remote/isolated tiles feel less valuable.
  float noRoadPenalty = 0.08f;           // applied when the tile has no adjacent road
  float disconnectedPenalty = 0.18f;     // applied when outside connection is required but missing

  // If true, parks only count if they're adjacent to a road component that
  // reaches the map edge (classic "outside connection" rule).
  bool requireOutsideConnection = true;
};

struct LandValueResult {
  int w = 0;
  int h = 0;

  // All arrays are flat [y*w + x] and are size w*h.
  // Values are normalized to [0,1].
  std::vector<float> value;          // overall land value (good = 1)
  std::vector<float> parkAmenity;    // good = 1
  std::vector<float> waterAmenity;   // good = 1
  std::vector<float> pollution;      // bad = 1
  std::vector<float> traffic;        // bad = 1 (road-adjacent congestion proxy)
};

// Compute per-tile land value + components.
//
// If traffic is non-null and has a valid roadTraffic buffer, we derive a simple
// "traffic penalty" field that bleeds road congestion into adjacent tiles.
//
// If cfg.requireOutsideConnection is true and roadToEdgeMask is non-null, it is
// used to decide whether parks are considered connected (and to apply the
// disconnectedPenalty).
LandValueResult ComputeLandValue(const World& world, const LandValueConfig& cfg,
                                 const TrafficResult* traffic = nullptr,
                                 const std::vector<std::uint8_t>* roadToEdgeMask = nullptr);

} // namespace isocity
