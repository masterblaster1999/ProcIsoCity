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
  Tectonic = 10, // Plate-driven mountain ranges and rift valleys.

  // Ring-shaped island with a central lagoon (and optional passes to the sea).
  Atoll = 11,

  // A navigable sea channel connecting two map edges, dividing two large landmasses.
  Strait = 12,

  // Sentinel for bounds checks / iteration (must be last; keep enum values contiguous).
  Count
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
  BlockGraph = 2, // Neighborhood-style districts based on CityBlocks adjacency.

  // Hydrology-based watershed districts.
  //
  // District boundaries tend to follow natural ridgelines and river valleys.
  // This can produce "regional" city structure on rugged terrain (uplands vs.
  // lowlands, river basins, coastal plains) while staying fully deterministic.
  Watershed = 3,

  // Sentinel for bounds checks / iteration (must be last; keep enum values contiguous).
  Count
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
  VoronoiCells = 4,      // Voronoi-cell boundary arterials (ring-road neighborhoods / superblocks).

  // Physarum / slime-mold inspired pheromone routing.
  //
  // A set of lightweight agents performs biased random walks between hubs while depositing
  // pheromone. Pheromone evaporates/diffuses over time, and a pheromone-biased A* pass
  // extracts an arterial network that often resembles organic, redundancy-aware transport
  // graphs.
  Physarum = 5,

  // Medial-axis / skeleton road layout.
  //
  // We approximate the medial axis of the buildable landmass by computing a
  // boundary Voronoi diagram (via multi-source BFS) and carving arterials
  // along the Voronoi edges (where distance-fronts from different boundary
  // sources meet).
  //
  // The result tends to create central spines on islands, ridge-like
  // backbones through narrow land corridors, and organic super-arterials that
  // "thread" through the middle of districts.
  MedialAxis = 6,

  // Tensor-field guided routing.
  //
  // Inspired by classic procedural city work (Parish & Müller) and later tensor-field
  // approaches, this layout builds a smooth *orientation field* from terrain
  // (contour-following on steep slopes) and low-frequency noise, then routes
  // hub-to-hub arterials using A* with a strong alignment bias to that field.
  //
  // The resulting macro network often exhibits "district grain" and sweeping
  // boulevard-like curves, while still respecting topography.
  TensorField = 7,

  // Sentinel for bounds checks / iteration (must be last; keep enum values contiguous).
  Count
};

// Human-readable name (stable for saves/CLI).
const char* ToString(ProcGenRoadLayout m);

// Parse layout from a string (case-insensitive). Accepts common aliases.
bool ParseProcGenRoadLayout(const std::string& s, ProcGenRoadLayout& out);

// -----------------------------------------------------------------------------
// Enum validation helpers (for serialized / user-provided values)
// -----------------------------------------------------------------------------

inline constexpr std::uint8_t ProcGenTerrainPresetCountU8()
{
  return static_cast<std::uint8_t>(ProcGenTerrainPreset::Count);
}

inline constexpr std::uint8_t ProcGenDistrictingModeCountU8()
{
  return static_cast<std::uint8_t>(ProcGenDistrictingMode::Count);
}

inline constexpr std::uint8_t ProcGenRoadLayoutCountU8()
{
  return static_cast<std::uint8_t>(ProcGenRoadLayout::Count);
}

inline constexpr bool IsValidProcGenTerrainPresetU8(std::uint8_t v)
{
  return v < ProcGenTerrainPresetCountU8();
}

inline constexpr bool IsValidProcGenDistrictingModeU8(std::uint8_t v)
{
  return v < ProcGenDistrictingModeCountU8();
}

inline constexpr bool IsValidProcGenRoadLayoutU8(std::uint8_t v)
{
  return v < ProcGenRoadLayoutCountU8();
}

inline constexpr std::uint8_t ClampProcGenTerrainPresetU8(std::uint8_t v)
{
  return IsValidProcGenTerrainPresetU8(v) ? v : static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
}

inline constexpr std::uint8_t ClampProcGenDistrictingModeU8(std::uint8_t v)
{
  return IsValidProcGenDistrictingModeU8(v) ? v : static_cast<std::uint8_t>(ProcGenDistrictingMode::Voronoi);
}

inline constexpr std::uint8_t ClampProcGenRoadLayoutU8(std::uint8_t v)
{
  return IsValidProcGenRoadLayoutU8(v) ? v : static_cast<std::uint8_t>(ProcGenRoadLayout::Organic);
}

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
