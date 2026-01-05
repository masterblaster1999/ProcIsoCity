#pragma once

#include <string>
#include <vector>

namespace isocity {

// A rectangle to be packed into an atlas.
//
// The caller fills (id,w,h). The packer fills (x,y).
struct GfxPackRect {
  int id = 0;
  int w = 0;
  int h = 0;
  int x = 0;
  int y = 0;
};

// Deterministic MaxRects atlas packer (no rotation).
//
// Uses the "Best Short Side Fit" heuristic and splits free rectangles on each placement.
// This is a common, practical rectangle packing strategy for texture atlases.
//
// Returns false if any rectangle does not fit within the bin.
bool PackMaxRects(int binW, int binH, std::vector<GfxPackRect>& ioRects, std::string& outError);

// Simple shelf/row packer (deterministic), useful as a robust fallback.
// Packs rectangles left-to-right, starting a new row when needed.
//
// outUsedH is the height actually used by the packed rectangles.
bool PackShelf(int binW, std::vector<GfxPackRect>& ioRects, int& outUsedH, std::string& outError);

} // namespace isocity
