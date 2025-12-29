#pragma once

#include <algorithm>
#include <cstdint>

namespace isocity {

// Road hierarchy
//
// We use Tile::level on road tiles to represent a simple 3-tier road class.
//  1 = Street
//  2 = Avenue
//  3 = Highway
//
// This keeps the save format stable (Tile::level is already persisted) while
// allowing simulation systems to reason about capacity/speed/maintenance.

constexpr int kRoadMinLevel = 1;
constexpr int kRoadMaxLevel = 3;

inline int ClampRoadLevel(int level)
{
  return std::clamp(level, kRoadMinLevel, kRoadMaxLevel);
}

enum class RoadClass : std::uint8_t {
  Street = 1,
  Avenue = 2,
  Highway = 3,
};

inline RoadClass RoadClassForLevel(int level)
{
  return static_cast<RoadClass>(static_cast<std::uint8_t>(ClampRoadLevel(level)));
}

inline const char* RoadClassName(int level)
{
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return "Street";
  case RoadClass::Avenue: return "Avenue";
  case RoadClass::Highway: return "Highway";
  }
  return "Street";
}

// Build cost per tile for creating a road at the given level.
// Upgrades are charged as the difference between target and current cost.
inline int RoadBuildCostForLevel(int level)
{
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return 1;
  case RoadClass::Avenue: return 3;
  case RoadClass::Highway: return 6;
  }
  return 1;
}

// A small integer "maintenance unit" used by the simulator.
// Total road maintenance cost = maintenanceRoad * sum(units).
inline int RoadMaintenanceUnitsForLevel(int level)
{
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return 1;
  case RoadClass::Avenue: return 2;
  case RoadClass::Highway: return 4;
  }
  return 1;
}

// Per-tile traffic capacity derived from a base capacity (street capacity).
// Uses integer math for determinism.
inline int RoadCapacityForLevel(int baseCapacity, int level)
{
  baseCapacity = std::max(0, baseCapacity);
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return baseCapacity;
  case RoadClass::Avenue: return (baseCapacity * 9) / 5;   // ~1.8x
  case RoadClass::Highway: return (baseCapacity * 13) / 5; // ~2.6x
  }
  return baseCapacity;
}

// Relative vehicle speed multiplier for visualization.
inline float RoadSpeedMultiplierForLevel(int level)
{
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return 1.00f;
  case RoadClass::Avenue: return 1.12f;
  case RoadClass::Highway: return 1.25f;
  }
  return 1.00f;
}

// Deterministic travel-time cost per road tile, in "milli-steps".
// A value of 1000 corresponds to 1 street step (one tile edge). Higher speed roads have lower cost.
//
// This is used by the weighted routing / flow-field builder so commuters and goods prefer faster roads
// even if the step-count is the same.
inline int RoadTravelTimeMilliForLevel(int level)
{
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return 1000;
  case RoadClass::Avenue: return 893;  // ~= 1000 / 1.12
  case RoadClass::Highway: return 800; // 1000 / 1.25
  }
  return 1000;
}

// Traffic spill multiplier (noise / externality) for the land value model.
// Higher-class roads penalize nearby land a bit more.
inline float RoadTrafficSpillMultiplierForLevel(int level)
{
  switch (RoadClassForLevel(level)) {
  case RoadClass::Street: return 1.00f;
  case RoadClass::Avenue: return 1.25f;
  case RoadClass::Highway: return 1.50f;
  }
  return 1.00f;
}

} // namespace isocity
