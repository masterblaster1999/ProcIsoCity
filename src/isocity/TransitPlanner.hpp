#pragma once

#include "isocity/RoadGraph.hpp"
#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A lightweight, deterministic transit ("bus line") planner.
//
// It operates on the compressed RoadGraph (intersections + straight segments) and a
// per-edge demand signal (e.g. aggregated commute traffic, goods shipments, or any
// synthetic flow you want to visualize).
//
// The goal is not to solve a full Transit Network Design Problem (TNDP) – instead we
// provide a fast heuristic that is suitable for headless tooling and iteration.

enum class TransitEdgeWeightMode : std::uint8_t {
  Steps = 0,
  TravelTime = 1,
};

inline const char* TransitEdgeWeightModeName(TransitEdgeWeightMode m)
{
  switch (m) {
    case TransitEdgeWeightMode::Steps:
      return "steps";
    case TransitEdgeWeightMode::TravelTime:
      return "time";
  }
  return "steps";
}

struct TransitPlannerConfig {
  // Maximum number of lines to produce.
  int maxLines = 8;

  // Number of endpoint nodes to consider (picked from high-demand nodes and spread
  // out deterministically).
  int endpointCandidates = 24;

  // How edge length is measured for shortest paths.
  TransitEdgeWeightMode weightMode = TransitEdgeWeightMode::TravelTime;

  // Demand-bias for the pathfinder.
  //
  // We compute a per-edge multiplier in [1, 1 + demandBias] based on remaining
  // demand on the edge; high-demand edges are cheaper.
  double demandBias = 2.5;

  // Disallow lines whose base cost exceeds maxDetour * shortestBaseCost.
  // Helps avoid weird, highly-circuitous lines.
  double maxDetour = 1.6;

  // After selecting a line, we "consume" a fraction of demand on its edges to
  // encourage subsequent lines to cover other corridors.
  //
  // 0.0 => no consumption (later lines may stack heavily)
  // 1.0 => full consumption (each edge effectively served once)
  double coverFraction = 0.7;

  // Ignore edges below this remaining demand when computing demand-normalized bias.
  std::uint64_t minEdgeDemand = 1;

  // Minimum total (remaining) demand a candidate line must cover to be accepted.
  std::uint64_t minLineDemand = 50;

  // Deterministic tie-break salt.
  std::uint64_t seedSalt = 0;
};

struct TransitLine {
  int id = 0;

  // Graph path.
  std::vector<int> nodes; // size >= 2 when valid
  std::vector<int> edges; // size == nodes.size()-1

  // Aggregate metrics.
  std::uint64_t sumDemand = 0;
  std::uint64_t baseCost = 0; // steps or milli-cost depending on cfg.weightMode
};

struct TransitPlan {
  TransitPlannerConfig cfg;
  std::uint64_t totalDemand = 0;
  std::uint64_t coveredDemand = 0; // sum of demand consumed across all selected lines
  std::vector<TransitLine> lines;
};

// Plan a set of transit lines on the given road graph.
//
// edgeDemand must have size g.edges.size() and contain non-negative demand weights.
//
// If cfg.weightMode == TravelTime and world != nullptr, travel time is derived from
// per-road-tile speeds (RoadTravelTimeMilliForLevel). If world is null, we fall back
// to a street-weight approximation (steps * 1000).
TransitPlan PlanTransitLines(const RoadGraph& g, const std::vector<std::uint64_t>& edgeDemand,
                             const TransitPlannerConfig& cfg, const World* world = nullptr);

// Build a tile polyline (road tiles) for a transit line by concatenating RoadGraphEdge::tiles.
// The resulting polyline is inclusive of both endpoints.
//
// Returns false on invalid indices.
bool BuildTransitLineTilePolyline(const RoadGraph& g, const TransitLine& line, std::vector<Point>& outTiles);

} // namespace isocity
