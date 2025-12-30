#pragma once

#include "isocity/World.hpp"

#include <algorithm>
#include <cstdint>

namespace isocity {

// Shared helper utilities for zone capacity and classification.
//
// NOTE: These values are used by both the simulation (population/jobs capacity) and
// the renderer/UI (indicators). Keeping them centralized avoids drift.

inline bool IsZoneOverlay(Overlay o)
{
  return (o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial);
}

inline int ClampZoneLevel(int level)
{
  return std::clamp(level, 1, 3);
}
inline int ClampZoneLevel(std::uint8_t level) { return ClampZoneLevel(static_cast<int>(level)); }

// Capacity per *tile* (not per-building).
inline int HousingForLevel(int level) { return 10 * ClampZoneLevel(level); }
inline int JobsCommercialForLevel(int level) { return 8 * ClampZoneLevel(level); }
inline int JobsIndustrialForLevel(int level) { return 12 * ClampZoneLevel(level); }

inline int CapacityForOverlayLevel(Overlay o, int level)
{
  const int lvl = ClampZoneLevel(level);
  if (o == Overlay::Residential) return HousingForLevel(lvl);
  if (o == Overlay::Commercial) return JobsCommercialForLevel(lvl);
  if (o == Overlay::Industrial) return JobsIndustrialForLevel(lvl);
  return 0;
}

inline int CapacityForTile(const Tile& t)
{
  return CapacityForOverlayLevel(t.overlay, static_cast<int>(t.level));
}

} // namespace isocity
