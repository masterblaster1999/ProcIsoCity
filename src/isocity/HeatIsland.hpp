#pragma once

#include "isocity/Goods.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <vector>

namespace isocity {

// A lightweight, deterministic "urban heat island" (UHI) heuristic.
//
// Real UHIs are driven by a mix of land cover, urban geometry, anthropogenic
// heat sources, and atmospheric conditions. For ProcIsoCity we want something
// that is:
//   - fast (usable in exports + tooling)
//   - deterministic (stable across runs)
//   - explainable (parks and water cool; dense/impervious areas warm)
//
// This module builds a per-tile "heat" signal from simple sources/sinks derived
// from the world state, then applies a few iterations of neighborhood diffusion
// (a cheap approximation of heat spreading through the urban fabric).

struct HeatIslandConfig {
  // Diffusion iterations (more = smoother / wider spread).
  int iterations = 64;

  // Diffusion strength per iteration: 0 => none, 1 => replace with neighbor average.
  float diffusion = 0.22f;

  // Use 8-connected neighbors instead of 4-connected.
  bool eightConnected = true;

  // --- Source/sink weights (heuristic, tunable) ---
  float roadBase = 0.45f;
  float roadClassBoost = 0.10f;    // extra heat for higher-class roads (level)
  float roadTrafficBoost = 0.35f;  // additional heat from commute traffic (optional)
  float roadGoodsBoost = 0.15f;    // additional heat from goods traffic (optional)

  float residentialSource = 0.25f;
  float commercialSource = 0.35f;
  float industrialSource = 0.55f;
  float civicSource = 0.30f;

  float parkSink = 0.40f;
  float waterSink = 0.60f;

  // Extra heat from local population/employment density (based on Tile::occupants).
  float occupantBoost = 0.20f;
  int occupantScale = 40; // occupants count that maps to ~1.0 for the boost

  // Higher elevations are slightly cooler (Tile::height in [0,1]).
  float elevationCooling = 0.15f;

  // Clamp for the pre-diffusion signal (keeps normalization stable).
  float sourceClampAbs = 1.0f;

  // Fallback normalized traffic when traffic/goods results are not provided.
  float fallbackCommuteTraffic01 = 0.15f;
  float fallbackGoodsTraffic01 = 0.05f;
};

struct HeatIslandResult {
  int w = 0;
  int h = 0;
  int iterations = 0;
  float diffusion = 0.0f;
  bool eightConnected = false;

  // Diffused heat field (heuristic units; roughly in [-sourceClampAbs, +sourceClampAbs]).
  std::vector<float> heat;

  // Normalized heat in [0,1] (0=coolest tile in map, 1=hottest).
  std::vector<float> heat01;

  float minHeat = 0.0f;
  float maxHeat = 0.0f;
};

// Compute per-tile heat.
//
// traffic/goods are optional. If omitted, roads still contribute a small amount
// of traffic-related heat via fallbackCommuteTraffic01/fallbackGoodsTraffic01.
HeatIslandResult ComputeHeatIsland(const World& world,
                                  const HeatIslandConfig& cfg = {},
                                  const TrafficResult* traffic = nullptr,
                                  const GoodsResult* goods = nullptr);

} // namespace isocity
