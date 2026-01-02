#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Floating-point point in tile-corner coordinates.
//
// Coordinate system matches other headless tooling (Vectorize, GeoJSON exports):
//   - x increases to the right
//   - y increases downward
//   - a tile at (x,y) occupies [x,x+1] x [y,y+1]
struct FPoint {
  double x = 0.0;
  double y = 0.0;
};

inline bool operator==(const FPoint& a, const FPoint& b) { return a.x == b.x && a.y == b.y; }

struct ContourPolyline {
  std::vector<FPoint> pts;
  bool closed = false; // true when pts.front() == pts.back()
};

struct ContourLevel {
  double level = 0.0;
  std::vector<ContourPolyline> lines;
};

struct ContourConfig {
  // Endpoint quantization for stitching (in tile units).
  //
  // Marching-squares intersections are computed by interpolating along shared
  // edges; under ideal arithmetic, adjacent cells agree exactly. In practice,
  // small floating-point differences can prevent stitching. We quantize endpoints
  // onto a fine grid before building polylines.
  double quantize = 1e-6;

  // If true, ambiguous saddle cases (5 and 10) are resolved deterministically
  // using a simple asymptotic-decider style heuristic.
  bool useAsymptoticDecider = true;

  // Optional polyline simplification tolerance (Douglas-Peucker), in tile units.
  // 0 disables.
  double simplifyTolerance = 0.0;

  // Drop any polyline with fewer than this many points (after simplification).
  int minPoints = 2;
};

// Build a (w+1) x (h+1) scalar grid of corner heights from the World's per-tile
// height field.
//
// Corner height at (cx,cy) is computed as the average of the adjacent tiles
// that touch that corner.
std::vector<double> BuildCornerHeightGrid(const World& world, double heightScale = 1.0);

// Extract contour polylines at the requested iso-levels.
//
// - cornerValues: row-major scalar grid of size cornerW*cornerH.
// - cornerW/cornerH: dimensions of the corner grid (typically world.w+1, world.h+1).
// - levels: iso-values to extract.
//
// Returns an array of ContourLevel entries (one per requested level). Each level
// contains zero or more polylines.
//
// On structural failures (broken stitching graphs), returns an empty vector and
// populates outError.
std::vector<ContourLevel> ExtractContours(const std::vector<double>& cornerValues,
                                         int cornerW,
                                         int cornerH,
                                         const std::vector<double>& levels,
                                         const ContourConfig& cfg,
                                         std::string* outError = nullptr);

} // namespace isocity
