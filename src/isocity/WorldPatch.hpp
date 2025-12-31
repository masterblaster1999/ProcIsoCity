#pragma once

#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Compact, deterministic patches between two save states.
//
// A WorldPatch is a list of per-tile field updates plus optional metadata
// (ProcGenConfig, SimConfig, Stats) so you can reproduce a full save state.
//
// Intended uses:
//   - regression testing / CI: store a small "golden" patch instead of full saves
//   - debugging: bisect which edits or sim changes caused a mismatch
//   - tooling: merge or transform saves deterministically

enum class WorldPatchCompression : std::uint8_t {
  None = 0,
  SLLZ = 1,
};

// Bitmask describing which fields are explicitly updated for a tile.
// (Used for compact patch encoding; consumers can still treat the patch as
// "set these fields to the target value".)
enum class TileFieldMask : std::uint8_t {
  Terrain = 1u << 0,
  Overlay = 1u << 1,
  Height = 1u << 2,
  Variation = 1u << 3,
  Level = 1u << 4,
  Occupants = 1u << 5,
  District = 1u << 6,
};

inline constexpr std::uint8_t operator|(TileFieldMask a, TileFieldMask b)
{
  return static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b);
}

struct WorldPatchTileDelta {
  // Row-major tile index: y*width + x.
  std::uint32_t index = 0;

  // Bitmask of TileFieldMask.
  std::uint8_t mask = 0;

  // Target tile values (only fields included in mask are meaningful).
  Tile value{};
};

struct WorldPatch {
  int width = 0;
  int height = 0;

  // Patch file format version (set when reading from disk).
  std::uint32_t version = 0;

  // Hashes are computed over (width,height,seed,tiles) and optionally Stats.
  // This allows strict patch application (base hash must match unless --force).
  std::uint64_t baseHash = 0;
  std::uint64_t targetHash = 0;

  bool includeProcCfg = true;
  bool includeSimCfg = true;
  bool includeStats = true;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  Stats stats{};

  std::vector<WorldPatchTileDelta> tiles;
};

// Build a patch that transforms (baseWorld, baseProcCfg, baseSimCfg) into
// (targetWorld, targetProcCfg, targetSimCfg).
//
// The worlds must have identical dimensions and seed. On success, outPatch
// contains the tile diffs plus any requested metadata.
bool MakeWorldPatch(const World& baseWorld, const ProcGenConfig& baseProcCfg, const SimConfig& baseSimCfg,
                    const World& targetWorld, const ProcGenConfig& targetProcCfg, const SimConfig& targetSimCfg,
                    WorldPatch& outPatch, std::string& outError,
                    bool includeProcCfg = true, bool includeSimCfg = true, bool includeStats = true);

// Apply a patch to an existing world+configs.
//
// If force==false, the patch will fail unless HashWorld(inOutWorld, includeStats)
// matches patch.baseHash.
//
// On success, HashWorld(inOutWorld, includeStats) will match patch.targetHash.
bool ApplyWorldPatch(World& inOutWorld, ProcGenConfig& inOutProcCfg, SimConfig& inOutSimCfg, const WorldPatch& patch,
                     std::string& outError, bool force = false);

// Serialize/deserialize a patch to a compact binary file.
bool SaveWorldPatchBinary(const WorldPatch& patch, const std::string& path, std::string& outError,
                          WorldPatchCompression compression = WorldPatchCompression::SLLZ);

bool LoadWorldPatchBinary(WorldPatch& outPatch, const std::string& path, std::string& outError);

// In-memory variants useful for higher-level containers (e.g., replay/journaling systems)
// or network transport.
bool SerializeWorldPatchBinary(const WorldPatch& patch, std::vector<std::uint8_t>& outBytes, std::string& outError,
                               WorldPatchCompression compression = WorldPatchCompression::SLLZ);

bool DeserializeWorldPatchBinary(WorldPatch& outPatch, const std::uint8_t* data, std::size_t size,
                                 std::string& outError);

// Build an inverse patch (target -> base) given the original base state and a forward patch.
//
// This is useful for undo/rollback workflows and patch algebra.
//
// If force==false, HashWorld(baseWorld, forwardPatch.includeStats) must match forwardPatch.baseHash.
bool InvertWorldPatch(const World& baseWorld, const ProcGenConfig& baseProcCfg, const SimConfig& baseSimCfg,
                      const WorldPatch& forwardPatch, WorldPatch& outInverse, std::string& outError,
                      bool force = false);

// Compose a sequence of patches (applied in order) into a single patch relative to the original base.
//
// This applies each patch to a working copy and then computes a minimal patch from base -> final.
//
// If force==false, each intermediate ApplyWorldPatch() must pass its baseHash check.
bool ComposeWorldPatches(const World& baseWorld, const ProcGenConfig& baseProcCfg, const SimConfig& baseSimCfg,
                         const std::vector<WorldPatch>& patches, WorldPatch& outPatch, std::string& outError,
                         bool includeProcCfg = true, bool includeSimCfg = true, bool includeStats = true,
                         bool force = false);


} // namespace isocity
