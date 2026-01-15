#pragma once

#include "isocity/Types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

class World;

// Configuration for procedurally generating an endless-ish POV path through the road network.
//
// The idea is to create a "cruising" camera path that prefers straight-ish segments, avoids
// immediate backtracking, and optionally biases towards scenic tiles (waterfronts / parks).
struct PovRoamConfig {
  // Desired number of tiles in the generated roam path.
  int length = 900;

  // How strongly to prefer going straight vs taking turns [0..1].
  float straightBias = 0.65f;

  // How strongly to bias towards scenic tiles [0..1].
  float scenicBias = 0.35f;

  // Penalize candidates that would lead into dead-ends.
  bool avoidDeadEnds = true;

  // If we can't find a continuation after this many attempts, we restart from a random road.
  int maxFailIters = 1024;

  // Search radius (in tiles) for finding a nearby road when the hint isn't on a road.
  int findRoadRadius = 32;
};

// Find the nearest road tile to a hint location within a square radius.
bool FindNearestRoadTile(const World& world, Point hint, int radius, Point& outRoad);

// Generate a "roam" path of road tiles.
//
// - startHint: a desired start location (camera center / selection). If not a road tile, the
//              generator searches for a nearby road.
// - seed: deterministic seed for the stochastic choices.
// - outDebug: optional human-readable stats.
std::vector<Point> GeneratePovRoamPath(const World& world, Point startHint, const PovRoamConfig& cfg,
                                      std::uint32_t seed, std::string* outDebug = nullptr);

} // namespace isocity
