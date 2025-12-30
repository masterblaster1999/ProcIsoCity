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



// -----------------------------------------------------------------------------
// Bridges
// -----------------------------------------------------------------------------
//
// Roads are normally placed on land, but ProcIsoCity also supports building roads on
// Water tiles. These are treated as *bridges* (same connectivity as roads, but with
// different build/maintenance costs and (optionally) slightly different routing
// weights).
//
// We keep this as a pure function layer (no Tile/World dependency) so the rest of
// the codebase can opt-in by checking tile.terrain == Water.

// Multipliers are kept as integers for deterministic gameplay + saves.
constexpr int kBridgeBuildCostMultiplier = 4;
constexpr int kBridgeMaintenanceUnitMultiplier = 2;

// Routing penalty (in milli-steps) added to bridge tiles so pathfinding will prefer
// land routes when they're comparable.
constexpr int kBridgeTravelTimePenaltyMilli = 150;

inline int RoadBridgeBuildCostForLevel(int level)
{
  return RoadBuildCostForLevel(level) * kBridgeBuildCostMultiplier;
}

inline int RoadBridgeMaintenanceUnitsForLevel(int level)
{
  return RoadMaintenanceUnitsForLevel(level) * kBridgeMaintenanceUnitMultiplier;
}

inline int RoadBridgeTravelTimeMilliForLevel(int level)
{
  return RoadTravelTimeMilliForLevel(level) + kBridgeTravelTimePenaltyMilli;
}


// -----------------------------------------------------------------------------
// Road placement cost helpers
// -----------------------------------------------------------------------------
//
// The simulation and tools often need the *money cost* of making a given tile
// a road of a desired class.
//
// We keep this logic in one place so:
//  - UI planners can estimate costs exactly
//  - headless tooling can plan roads deterministically
//  - the economy stays consistent across modules

// Money cost to build or upgrade a single road tile to `targetLevel`.
//
// Parameters:
//  - currentLevel: existing road level (ignored when alreadyRoad==false)
//  - targetLevel: desired road level (clamped to [1..3])
//  - alreadyRoad: whether the tile currently has a Road overlay
//  - isBridge: whether the tile is a bridge (road on water) for pricing
//
// Returns:
//  - For empty tiles: full build cost.
//  - For existing roads: upgrade delta cost (0 if current >= target).
inline int RoadPlacementCost(int currentLevel, int targetLevel, bool alreadyRoad, bool isBridge)
{
  targetLevel = ClampRoadLevel(targetLevel);
  currentLevel = ClampRoadLevel(currentLevel);

  const int costTarget = isBridge ? RoadBridgeBuildCostForLevel(targetLevel) : RoadBuildCostForLevel(targetLevel);
  if (!alreadyRoad) return costTarget;

  const int costCur = isBridge ? RoadBridgeBuildCostForLevel(currentLevel) : RoadBuildCostForLevel(currentLevel);
  return std::max(0, costTarget - costCur);
}

} // namespace isocity
