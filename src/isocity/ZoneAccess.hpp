#pragma once

#include "isocity/Types.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <vector>

namespace isocity {

// Precomputed mapping from zone tiles (Residential/Commercial/Industrial) to a road tile
// that should be treated as that tile's "access point".
//
// Why this exists:
// - The naive rule "a zone tile must have an adjacent road" prevents *interior* tiles in a
//   larger zoned block from ever functioning.
// - The renderer already supports multi-tile buildings (ZoneParcels), so the simulation and
//   derived systems (traffic/goods) should also allow a zone to be accessible through a
//   connected neighbor that *does* touch a road.
//
// This map assigns each zone tile to the *nearest* (in zone-steps) road-adjacent boundary
// tile, and then uses that boundary tile's adjacent road as the access point.
//
// If a zone block has no road-adjacent boundary, tiles remain inaccessible (roadIdx == -1).
struct ZoneAccessMap {
  int w = 0;
  int h = 0;

  // For each tile index (y*w + x):
  // - If the tile is Residential/Commercial/Industrial and has access, this stores the road tile
  //   index (ry*w + rx) that should be used as its access point.
  // - Otherwise this is -1.
  std::vector<int> roadIdx;
};

// Build a ZoneAccessMap for the world.
//
// If roadToEdgeMask is non-null and has size w*h, only road tiles with mask==1 are treated as
// valid access points (used to enforce the "outside connection" rule).
ZoneAccessMap BuildZoneAccessMap(const World& world, const std::vector<std::uint8_t>* roadToEdgeMask);

inline bool HasZoneAccess(const ZoneAccessMap& m, int x, int y)
{
  if (x < 0 || y < 0 || x >= m.w || y >= m.h) return false;
  const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(m.w) + static_cast<std::size_t>(x);
  if (idx >= m.roadIdx.size()) return false;
  return m.roadIdx[idx] >= 0;
}

inline bool PickZoneAccessRoadTile(const ZoneAccessMap& m, int x, int y, Point& outRoad)
{
  if (x < 0 || y < 0 || x >= m.w || y >= m.h) return false;
  const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(m.w) + static_cast<std::size_t>(x);
  if (idx >= m.roadIdx.size()) return false;
  const int ridx = m.roadIdx[idx];
  if (ridx < 0) return false;
  outRoad.x = ridx % m.w;
  outRoad.y = ridx / m.w;
  return true;
}

} // namespace isocity
