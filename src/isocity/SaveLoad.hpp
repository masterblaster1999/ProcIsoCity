#pragma once

#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// (ProcGenConfig and SimConfig are defined in ProcGen.hpp and Sim.hpp)

// Lightweight header summary for a save file.
// Useful for building in-game slot browsers without loading the full world.
struct SaveSummary {
  std::uint32_t version = 0;

  int width = 0;
  int height = 0;
  std::uint64_t seed = 0;

  ProcGenConfig procCfg{};
  bool hasProcCfg = false;

  // Stored Stats fields from the save header (not derived overlays).
  Stats stats{};
  bool hasStats = false;

  // v6+ stores policy/economy settings.
  SimConfig simCfg{};
  bool hasSimCfg = false;

  // v3+ saves contain a CRC32 checksum appended to the file.
  // This is populated only when verifyCrc=true.
  bool crcChecked = false;
  bool crcOk = true;
};

// Read just the save header (and optionally verify CRC for v3+).
// This avoids loading/regenerating the full world.
bool ReadSaveSummary(const std::string& path, SaveSummary& outSummary, std::string& outError, bool verifyCrc = false);

// Binary save/load for the current world state.
// File format is versioned so you can extend it later.
//
// v1: full tile serialization
// v2: delta serialization (seed + ProcGenConfig + only tiles that differ from a regenerated baseline)
// v3: v2 + CRC32 checksum (detects corrupted / truncated save files)
// v4: v3 + varint/delta encoding for tile diffs (smaller saves, faster IO)
// v5: v4 + height deltas (persists terraforming)
// v6: v5 + SimConfig (persists policy/economy settings)
// v7: v6 + districts (per-tile district IDs) + optional district policy multipliers
// v8: v7 + compressed delta payload (smaller saves; faster disk IO)
// v9: v8 + ProcGen erosion config
// v10: v9 + ProcGen terrain preset config
// v11: v10 + ProcGen road hierarchy config

// Save using an explicit ProcGenConfig + SimConfig (recommended for v2+ delta saves).
bool SaveWorldBinary(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg, const std::string& path,
                     std::string& outError);

// Back-compat helper: save using the default SimConfig{}.
bool SaveWorldBinary(const World& world, const ProcGenConfig& procCfg, const std::string& path, std::string& outError);

// Back-compat helper: save using ProcGenConfig{} and SimConfig{}.
bool SaveWorldBinary(const World& world, const std::string& path, std::string& outError);

// Load and return the ProcGenConfig used by the save (v1 loads return ProcGenConfig{}).
// v6+ saves also return the SimConfig stored in the file; v5 and older return SimConfig{}.
// v7 additionally persists per-tile district IDs and optional district policy multipliers.
// v8 stores the same data as v7 but compresses the delta section.
// v9 additionally persists ProcGen erosion settings.
// v10 additionally persists ProcGen terrain preset settings.
// v11 additionally persists ProcGen road hierarchy settings.
bool LoadWorldBinary(World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg, const std::string& path,
                     std::string& outError);

// Back-compat helper: load and discard SimConfig.
bool LoadWorldBinary(World& outWorld, ProcGenConfig& outProcCfg, const std::string& path, std::string& outError);

// Back-compat helper: load and discard ProcGenConfig + SimConfig.
bool LoadWorldBinary(World& outWorld, const std::string& path, std::string& outError);


// In-memory save/load variants.
//
// These are useful for embedding saves inside higher-level containers (e.g. replay/journaling
// systems) without touching the filesystem. The binary format is identical to the on-disk
// SaveWorldBinary format (including CRC for v3+ saves).
bool SaveWorldBinaryToBytes(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg,
                            std::vector<std::uint8_t>& outBytes, std::string& outError);

// Back-compat helper: save using the default SimConfig{}.
bool SaveWorldBinaryToBytes(const World& world, const ProcGenConfig& procCfg, std::vector<std::uint8_t>& outBytes,
                            std::string& outError);

// Back-compat helper: save using ProcGenConfig{} and SimConfig{}.
bool SaveWorldBinaryToBytes(const World& world, std::vector<std::uint8_t>& outBytes, std::string& outError);

bool LoadWorldBinaryFromBytes(World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg, const std::uint8_t* data,
                              std::size_t size, std::string& outError);

inline bool LoadWorldBinaryFromBytes(World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg,
                                     const std::vector<std::uint8_t>& bytes, std::string& outError)
{
  return LoadWorldBinaryFromBytes(outWorld, outProcCfg, outSimCfg, bytes.data(), bytes.size(), outError);
}

// Back-compat helper: load and discard SimConfig.
inline bool LoadWorldBinaryFromBytes(World& outWorld, ProcGenConfig& outProcCfg, const std::vector<std::uint8_t>& bytes,
                                     std::string& outError)
{
  SimConfig simCfg;
  return LoadWorldBinaryFromBytes(outWorld, outProcCfg, simCfg, bytes, outError);
}

// Back-compat helper: load and discard ProcGenConfig + SimConfig.
inline bool LoadWorldBinaryFromBytes(World& outWorld, const std::vector<std::uint8_t>& bytes, std::string& outError)
{
  ProcGenConfig procCfg;
  SimConfig simCfg;
  return LoadWorldBinaryFromBytes(outWorld, procCfg, simCfg, bytes, outError);
}

} // namespace isocity
