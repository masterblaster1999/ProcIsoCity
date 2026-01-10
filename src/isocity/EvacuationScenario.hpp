#pragma once

#include "isocity/DepressionFill.hpp"
#include "isocity/Evacuation.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Higher-level evacuation scenario analysis built from:
//  - hazard generation from the heightfield (sea-level inundation + ponding potential)
//  - evacuation-to-edge accessibility and bottleneck analysis on the road network
//
// This module exists so the interactive game can toggle hazard assumptions and
// visualize results without duplicating the CLI tool's logic.

enum class EvacuationHazardMode : std::uint8_t {
  None = 0,
  Sea = 1,
  Ponding = 2,
  Both = 3,
};

const char* EvacuationHazardModeName(EvacuationHazardMode m);

struct EvacuationScenarioConfig {
  EvacuationHazardMode hazardMode = EvacuationHazardMode::Sea;

  // When true, road tiles that sit on top of Terrain::Water (bridges) are treated
  // as passable even if the underlying cell is marked hazardous.
  bool bridgesPassable = true;

  // Sea-level flooding input (derived from heightfield).
  float seaLevel = 0.35f; // 0..1 height threshold
  SeaFloodConfig seaCfg{};

  // Ponding potential via Priority-Flood depression-fill.
  DepressionFillConfig pondCfg{};

  // Minimum depression depth required to be considered hazardous.
  float pondMinDepth = 0.01f;

  // Evacuation routing config.
  EvacuationConfig evac{};
};

struct EvacuationScenarioResult {
  int w = 0;
  int h = 0;

  // Combined hazard mask used for evacuation routing (size w*h, row-major).
  // Non-zero entries are treated as blocked/hazardous tiles.
  std::vector<std::uint8_t> hazardMask;

  // Component hazard outputs (populated when their mode is active).
  SeaFloodResult sea;
  DepressionFillResult pond;

  // Evacuation analysis result computed under hazardMask.
  EvacuationResult evac;
};

// Lightweight 0..1 heatmaps for in-game visualization (size w*h).
struct EvacuationScenarioHeatmaps {
  int w = 0;
  int h = 0;

  // Normalized evacuation time for reachable residential tiles.
  // 0 => fast, 1 => slow (clamped). Non-residential tiles are 0.
  std::vector<float> evacTime;

  // 1 for unreachable residential tiles (not blocked by hazard), else 0.
  std::vector<float> evacUnreachable;

  // Normalized evacuation road flow for road tiles.
  std::vector<float> evacFlow;
};

EvacuationScenarioResult ComputeEvacuationScenario(const World& world, const EvacuationScenarioConfig& cfg);
EvacuationScenarioHeatmaps BuildEvacuationScenarioHeatmaps(const World& world, const EvacuationScenarioResult& r);

} // namespace isocity
