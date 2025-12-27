#pragma once

#include "isocity/World.hpp"

#include <string>

namespace isocity {

struct ProcGenConfig;

// Binary save/load for the current world state.
// File format is versioned so you can extend it later.
//
// v1: full tile serialization
// v2: delta serialization (seed + ProcGenConfig + only tiles that differ from a regenerated baseline)
// v3: v2 + CRC32 checksum (detects corrupted / truncated save files)

// Save using an explicit ProcGenConfig (recommended for v2 delta saves).
bool SaveWorldBinary(const World& world, const ProcGenConfig& procCfg, const std::string& path, std::string& outError);

// Back-compat helper: save using ProcGenConfig{}.
bool SaveWorldBinary(const World& world, const std::string& path, std::string& outError);

// Load and return the ProcGenConfig used by the save (v1 loads return ProcGenConfig{}).
bool LoadWorldBinary(World& outWorld, ProcGenConfig& outProcCfg, const std::string& path, std::string& outError);

// Back-compat helper: load and discard ProcGenConfig.
bool LoadWorldBinary(World& outWorld, const std::string& path, std::string& outError);

} // namespace isocity
