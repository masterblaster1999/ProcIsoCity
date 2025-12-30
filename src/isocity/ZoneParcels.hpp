#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// A "parcel" groups one or more adjacent zone tiles into a single logical footprint.
// This is primarily used for rendering merged multi-tile buildings (but is kept in core
// so it can be tested deterministically and reused by headless tools).
//
// Current implementation generates **rectangular** footprints (w x h), anchored at (x0,y0)
// with the render anchor at the bottom-right tile (x0+w-1, y0+h-1).
struct ZoneBuildingParcel {
  int x0 = 0;
  int y0 = 0;
  int w = 1;
  int h = 1;

  Overlay overlay = Overlay::None;
  std::uint8_t level = 1;

  int occupants = 0; // sum across tiles
  int capacity = 0;  // sum across tiles (population/jobs cap)

  std::uint32_t styleSeed = 0; // deterministic style seed for rendering variation

  // 0..3 (cardinal), derived from which side of the footprint is most road-adjacent.
  // Used by the renderer to orient a couple of rooftop details.
  std::uint8_t facing = 0; // 0=N, 1=E, 2=S, 3=W

  int x1() const { return x0 + w - 1; }
  int y1() const { return y0 + h - 1; }
  int area() const { return w * h; }
  bool isMultiTile() const { return w > 1 || h > 1; }
};

struct ZoneBuildingParcels {
  int width = 0;
  int height = 0;

  std::vector<ZoneBuildingParcel> parcels;

  // For each tile (index = y*width + x), the parcel index or -1 if the tile isn't zoned.
  std::vector<int> tileToParcel;

  // For each tile, the parcel index if the tile is that parcel's render anchor, else -1.
  // Anchor tile is always the bottom-right tile of the parcel.
  std::vector<int> anchorToParcel;

  void clear()
  {
    width = 0;
    height = 0;
    parcels.clear();
    tileToParcel.clear();
    anchorToParcel.clear();
  }
};

// Build parcels for all Residential/Commercial/Industrial tiles.
// Deterministic given identical world state.
void BuildZoneBuildingParcels(const World& world, ZoneBuildingParcels& out);

} // namespace isocity
