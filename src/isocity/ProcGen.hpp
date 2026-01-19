#pragma once

#include "isocity/Erosion.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// High-level terrain "macro" presets.
//
// These modify the initial noise heightfield before erosion/river carving and
// before tiles are classified into Water/Sand/Grass.
//
// The goal is to make seeds feel meaningfully different without requiring
// external art assets or hand-authored maps.
enum class ProcGenTerrainPreset : std::uint8_t {
  Classic = 0,     // Current default: general continental noise.
  Island = 1,      // Strong coastal falloff: land mass centered, water at edges.
  Archipelago = 2, // Multiple smaller islands.
  InlandSea = 3,   // Central water body with surrounding land.
  RiverValley = 4, // Major meandering river corridor bisecting the map.
  MountainRing = 5, // Ring-like ridge / crater-style macro relief.

  // More "extreme" presets.
  Fjords = 6,  // Glaciated coast: mountainous shoreline with long narrow inlets.
  Canyon = 7,  // High plateau cut by a deep winding canyon river.
  Volcano = 8, // Central volcanic cone with a caldera (optionally lake-filled).
  Delta = 9,   // Low coastal river delta with branching distributaries.
};

// Human-readable name (stable for saves/CLI).
const char* ToString(ProcGenTerrainPreset p);

// Parse preset from a string (case-insensitive). Accepts common aliases.
bool ParseProcGenTerrainPreset(const std::string& s, ProcGenTerrainPreset& out);

// District assignment modes used during world generation.
//
// Districts influence zoning profiles and can be visualized in the district overlay.
// The legacy mode is a hub-anchored Voronoi partition over the tile grid.
// Newer modes use the generated road network / city blocks so boundaries tend to
// follow streets and feel more like neighborhoods.
enum class ProcGenDistrictingMode : std::uint8_t {
  Voronoi = 0,   // Legacy: hub-anchored Voronoi over the tile grid.
  RoadFlow = 1,  // Road-based travel-time districts (AutoAssignDistricts).
  BlockGraph = 2 // Neighborhood-style districts based on CityBlocks adjacency.
};

// Human-readable name (stable for saves/CLI).
const char* ToString(ProcGenDistrictingMode m);

// Parse mode from a string (case-insensitive). Accepts common aliases.
bool ParseProcGenDistrictingMode(const std::string& s, ProcGenDistrictingMode& out);

// High-level road network layout used during procedural generation.
//
// This influences how hubs are connected at the macro scale before the
// street subdivision and road hierarchy upgrade passes run.
enum class ProcGenRoadLayout : std::uint8_t {
  Organic = 0,           // Existing default: MST backbone + local loops.
  Grid = 1,              // Manhattan-like arterial grid with hubs snapped onto it.
  Radial = 2,            // Hub-and-spoke with an outer ring / beltway bias.
  SpaceColonization = 3, // Branching arterial growth guided by attractor points ("space colonization" style).
};

// Human-readable name (stable for saves/CLI).
const char* ToString(ProcGenRoadLayout m);

// Parse layout from a string (case-insensitive). Accepts common aliases.
bool ParseProcGenRoadLayout(const std::string& s, ProcGenRoadLayout& out);

struct ProcGenConfig {
  float terrainScale = 0.08f;     // noise scale
  // Water/sea level in [0,1].
  // Back-compat: older code refers to this as seaLevel.
  union {
    float waterLevel = 0.35f;       // below => water
    float seaLevel;
  };
  float sandLevel = 0.42f;        // below => sand (above water)
  int hubs = 4;                   // number of "town centers"
  int extraConnections = 2;       // extra road connections between hubs

  // Macro road layout style used to connect hubs (Organic/Grid/Radial).
  ProcGenRoadLayout roadLayout = ProcGenRoadLayout::Organic;
  float zoneChance = 0.22f;       // chance to place a zone next to a road
  float parkChance = 0.06f;       // chance to place a park next to a road

  // Macro terrain preset. Classic preserves the current generation pipeline.
  ProcGenTerrainPreset terrainPreset = ProcGenTerrainPreset::Classic;

  // Strength of the macro preset effect.
  //
  // - 0 disables the preset (equivalent to Classic, but the enum is still stored).
  // - 1 is the intended default strength.
  // - >1 exaggerates the preset for more extreme maps.
  float terrainPresetStrength = 1.0f;

  // Procedural road hierarchy / arterial upgrade pass.
  //
  // When enabled, ProcIsoCity samples shortest paths along the generated road network
  // to approximate *betweenness centrality* and upgrades frequently-traversed corridors
  // to higher road classes (Avenue/Highway). This produces clearer arterial structure,
  // better early-game traffic flow, and more believable zoning gradients.
  bool roadHierarchyEnabled = true;

  // Strength of the road hierarchy effect.
  //
  // - 0 disables the pass (no upgrades beyond the initial carve rules).
  // - 1 is the intended default.
  // - >1 upgrades more tiles / creates stronger arterial spines.
  float roadHierarchyStrength = 1.0f;

  // Procedural district assignment.
  //
  // - Voronoi: hub-anchored tile Voronoi (legacy).
  // - RoadFlow: districts based on road travel time (good for big arterials).
  // - BlockGraph: districts based on contiguous city blocks (neighborhood-like).
  //
  // IMPORTANT: Older save versions default to Voronoi to preserve deterministic regeneration.
  ProcGenDistrictingMode districtingMode = ProcGenDistrictingMode::BlockGraph;

  // Optional post-noise terrain shaping stage.
  // Enabled by default in the current generation pipeline.
  ErosionConfig erosion{};
};

World GenerateWorld(int width, int height, std::uint64_t seed, const ProcGenConfig& cfg = {});

} // namespace isocity
