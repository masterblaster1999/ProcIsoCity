#pragma once

#include <cstdint>
#include <vector>

namespace isocity {

// Minimal, dependency-free terrain hydrology utilities.
//
// These helpers are intended for:
//  - debugging / headless tooling (river extraction, basin analysis)
//  - keeping erosion "river carving" consistent with analysis tools
//
// The algorithms are deliberately simple and deterministic:
//  - 4-neighborhood flow direction (D4)
//  - accumulation computed by descending-height propagation

struct HydrologyField {
  int w = 0;
  int h = 0;

  // Downstream neighbor index (linear y*w + x) or -1 for sinks.
  std::vector<int> dir;

  // Flow accumulation (>= 1). Each cell contributes 1 unit of flow.
  std::vector<int> accum;

  // Max of accum (>= 1 when non-empty).
  int maxAccum = 0;

  bool empty() const { return w <= 0 || h <= 0; }
  std::size_t size() const { return dir.size(); }
};

// Compute a deterministic D4 flow direction field.
//
// outDir[i] = linear index of strictly-lower neighbor with minimum height,
//             or -1 if no strictly-lower neighbor exists.
//
// Tie-breaking is deterministic based on neighbor iteration order.
void ComputeFlowDir4(const std::vector<float>& heights, int w, int h, std::vector<int>& outDir);

// Compute flow accumulation given a flow direction field.
//
// Each cell contributes 1 to itself and propagates its accumulated value to its downstream neighbor.
//
// If outMaxAccum is non-null, it will be set to the maximum accumulation value.
void ComputeFlowAccumulation(const std::vector<float>& heights, int w, int h,
                             const std::vector<int>& dir, std::vector<int>& outAccum,
                             int* outMaxAccum = nullptr);

// Convenience: compute dir + accum for a heightfield.
HydrologyField BuildHydrologyField(const std::vector<float>& heights, int w, int h);

// Default heuristic for choosing a river threshold (minimum accumulation).
//
// This mirrors the heuristic used by Erosion's river carving.
int AutoRiverMinAccum(int w, int h);

// Build a 0/1 river mask where accumulation >= minAccum.
// If minAccum <= 0, AutoRiverMinAccum(w,h) is used.
std::vector<std::uint8_t> BuildRiverMask(const std::vector<int>& accum, int w, int h, int minAccum);

// Basin segmentation (watershed): each cell is labeled by the sink it drains into.

struct BasinInfo {
  int id = -1;        // Basin id in BasinSegmentation::basins ordering.
  int sinkIndex = -1; // Linear index of sink cell.
  int sinkX = 0;
  int sinkY = 0;
  int area = 0;       // Number of cells draining to this sink.
};

struct BasinSegmentation {
  int w = 0;
  int h = 0;

  // Per-cell basin id (0..basins.size()-1). -1 for invalid/unassigned.
  std::vector<int> basinId;

  // Basins sorted by area descending; tie-break by sinkIndex ascending.
  // basin id == index into this vector.
  std::vector<BasinInfo> basins;

  bool empty() const { return w <= 0 || h <= 0; }
};

// Segment basins by following the flow direction to sinks.
//
// dir must have size w*h.
BasinSegmentation SegmentBasins(const std::vector<int>& dir, int w, int h);

} // namespace isocity
