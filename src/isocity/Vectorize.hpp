#pragma once

#include <string>
#include <vector>

namespace isocity {

// Simple grid-vectorization utilities.
//
// The headless toolchain (blocks/districts/etc.) often starts with a per-tile integer
// label grid (e.g. block IDs). This module converts such raster labels into vector
// polygons suitable for GeoJSON / external GIS tooling.
//
// Coordinate system:
//  - Input grid coordinates use the same convention as the World / CLI exports:
//      x increases to the right
//      y increases downward
//  - Output polygon vertices are in *tile-corner* coordinates on that same grid.
//    A tile at (x,y) occupies the unit square [x,x+1] x [y,y+1].
//
// IMPORTANT: This module is intentionally dependency-free (no external geometry libs).

struct IPoint {
  int x = 0;
  int y = 0;

  bool operator==(const IPoint& o) const { return x == o.x && y == o.y; }
  bool operator!=(const IPoint& o) const { return !(*this == o); }
};

struct VectorPolygon {
  // Closed ring (outer[0] == outer.back()).
  std::vector<IPoint> outer;

  // Closed rings.
  std::vector<std::vector<IPoint>> holes;
};

struct VectorMultiPolygon {
  std::vector<VectorPolygon> polygons;
};

struct LabeledGeometry {
  int label = 0;
  VectorMultiPolygon geom;
};

struct VectorizeStats {
  int labels = 0;
  int rings = 0;
  int polygons = 0;
  int holes = 0;
};

// Vectorize a raster label grid into polygons.
//
// - labels: row-major label grid of size w*h.
// - backgroundLabel: this label is treated as "empty" and is not vectorized.
// - out: per-label polygon geometry (sorted by label ascending).
//
// Returns false on structural failures (unexpected broken contour graphs).
bool VectorizeLabelGridToPolygons(const std::vector<int>& labels,
                                  int w,
                                  int h,
                                  int backgroundLabel,
                                  std::vector<LabeledGeometry>& out,
                                  VectorizeStats* outStats = nullptr,
                                  std::string* outError = nullptr);

} // namespace isocity
