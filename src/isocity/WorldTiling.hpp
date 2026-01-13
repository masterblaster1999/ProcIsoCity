#pragma once

#include "isocity/Blueprint.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Blueprint-based "macro tile" world tiling.
//
// The goal is to let users author small stampable chunks (Blueprints) and then
// procedurally tile them across a larger region while enforcing simple seam
// constraints (currently: road presence along tile edges).
//
// This is intentionally lightweight (greedy scanline selection) but already
// produces useful results when the tileset is authored to be edge-compatible
// (Wang-tile style).

struct BlueprintTileVariant {
  std::string name;
  BlueprintTransform transform{}; // transform applied to the source blueprint
  Blueprint bp;                  // transformed blueprint (dimensions == tileset cell)

  // Edge signatures used for matching.
  // Currently encode road presence along each edge (length-aware 64-bit hashes).
  std::uint64_t edgeN = 0;
  std::uint64_t edgeE = 0;
  std::uint64_t edgeS = 0;
  std::uint64_t edgeW = 0;
};

struct BlueprintTileset {
  int cellW = 0;
  int cellH = 0;
  std::vector<BlueprintTileVariant> variants;
};

struct BlueprintTilingConfig {
  // If true, enforce that road patterns match between adjacent placed tiles.
  bool matchRoadEdges = true;

  // If true, relax constraints when no candidates fit (best-effort fill).
  bool allowFallback = true;

  // Deterministic seed for variant choice.
  std::uint32_t seed = 0;
};

struct BlueprintTilingSolution {
  int cellsX = 0;
  int cellsY = 0;
  int fallbacks = 0;
  int failures = 0;

  // Row-major chosen variant index per cell (size = cellsX*cellsY). -1 means empty.
  std::vector<int> chosen;
};

// Build a tileset from a list of input blueprints.
//
// - cellW/cellH define the macro-tile cell dimensions.
// - if generateTransforms==true, additional rotations/mirrors are considered
//   (only those producing exactly cellW/cellH are kept).
bool BuildBlueprintTileset(const std::vector<std::pair<std::string, Blueprint>>& sources,
                           int cellW, int cellH, bool generateTransforms, BlueprintTileset& outTileset,
                           std::string& outError);

// Solve a tiling for a grid of size (cellsX, cellsY).
//
// Uses a greedy scanline placement: each cell selects a variant consistent with
// already-placed west/north neighbors. With allowFallback enabled, constraints
// are relaxed if needed.
bool SolveBlueprintTiling(const BlueprintTileset& tileset, int cellsX, int cellsY, const BlueprintTilingConfig& cfg,
                          BlueprintTilingSolution& outSolution, std::string& outError);

} // namespace isocity
