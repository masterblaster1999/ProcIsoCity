#pragma once

#include "isocity/TransitPlanner.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

struct TrafficResult;
struct GoodsResult;
struct RoadGraph;
struct ZoneAccessMap;

// Reuse the simulator's demand-mode enum (forward declared here to avoid pulling in Sim.hpp).
enum class TransitDemandMode : std::uint8_t;

// -----------------------------------------------------------------------------
// Transit accessibility + mode-share potential analysis
//
// Goal: provide a stable, explainable proxy for where transit is "useful" in the
// current city layout.
//
// - We plan bus lines using the existing TransitPlanner (same as the in-game
//   transit overlay), driven by traffic/goods demand.
// - We generate stops along those lines.
// - We compute a road-network walking isochrone to the nearest stop.
// - We convert stop distance to a per-tile accessibility score in [0,1].
// - We estimate a *localized* mode-share potential signal, consistent with the
//   simulator's global mode-share formula but applied per tile.
//
// This is not a full transit assignment model; it's a heuristic layer intended
// for planning / visualization.
// -----------------------------------------------------------------------------

struct TransitAccessibilityConfig {
  // If true, only roads connected to the world edge are considered reachable
  // (matches typical "outside connection" rules).
  bool requireOutsideConnection = true;

  // Planner demand signal mode (commute/goods/combined).
  TransitDemandMode demandMode = static_cast<TransitDemandMode>(2); // Combined

  // Planner parameters (line count, demand bias, etc.).
  TransitPlannerConfig plannerCfg{};

  // Stop spacing along a line (in road-tile steps). Endpoints are always stops.
  int stopSpacingTiles = 12;

  // "Walk to stop" radius (in road steps) used for access share calculations.
  int walkRadiusSteps = 10;

  // Map steps-to-stop into an accessibility score.
  // steps <= goodSteps => 1.0
  // steps >= badSteps  => 0.0
  int goodSteps = 2;
  int badSteps = 25;

  // Simulator-aligned tuning for local mode-share potential.
  // maxModeShare is the theoretical cap; modeSharePotential01 is normalized to that cap.
  float serviceLevel = 1.0f;
  float maxModeShare = 0.35f;
  float travelTimeMultiplier = 0.75f;
};

struct TransitAccessibilityInputs {
  // Optional demand signals used for planning + corridor coverage.
  const TrafficResult* traffic = nullptr;
  const GoodsResult* goods = nullptr;

  // Optional precomputed inputs to avoid rebuilding heavy structures.
  const RoadGraph* roadGraph = nullptr;
  const TransitPlan* plan = nullptr;
  const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
  const ZoneAccessMap* zoneAccess = nullptr;
};

struct TransitAccessibilityResult {
  int w = 0;
  int h = 0;
  TransitAccessibilityConfig cfg{};

  int plannedLines = 0;
  int plannedStops = 0;

  // Corridor and access coverage metrics in [0,1].
  float corridorCoverage = 0.0f;
  float resStopAccessShare = 0.0f;
  float jobsStopAccessShare = 0.0f;
  float accessCoverage = 0.0f;   // geometric mean of res/jobs stop access shares
  float overallCoverage = 0.0f;  // corridorCoverage * accessCoverage

  // Nearest-stop distance in road steps per tile. -1 = unreachable / no stop.
  std::vector<int> stepsToStop;

  // Stop accessibility score in [0,1]. 1=very close, 0=very far/unreachable.
  std::vector<float> access01;

  // Localized transit mode share potential in [0,1], normalized by cfg.maxModeShare.
  // (i.e., 1.0 means "this tile could plausibly hit maxModeShare").
  std::vector<float> modeSharePotential01;

  // Masks for visualization.
  std::vector<std::uint8_t> stopMask;     // 1 on stop tiles
  std::vector<std::uint8_t> corridorMask; // 1 on road tiles served by a planned line
};

TransitAccessibilityResult ComputeTransitAccessibility(const World& world,
                                                      const TransitAccessibilityConfig& cfg = {},
                                                      const TransitAccessibilityInputs& in = {});

} // namespace isocity
