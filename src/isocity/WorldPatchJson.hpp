#pragma once

#include "isocity/Json.hpp"
#include "isocity/WorldPatch.hpp"

#include <string>

namespace isocity {

// Human-readable JSON serialization for WorldPatch.
//
// Goals:
//  - make patches inspectable / diffable in version control
//  - enable lightweight tooling pipelines that don't want a binary format
//  - remain fully deterministic for regression testing / CI
//
// Notes:
//  - Hashes are encoded as hex strings (e.g. "0x0123...") to avoid JSON number precision loss.
//  - Tile heights are stored as u16 "height_q" and reconstructed as height = height_q / 65535.
//
// The format is intentionally stable and self-describing.

// File-level helpers.
bool SaveWorldPatchJson(const WorldPatch& patch, const std::string& path, std::string& outError,
                        const JsonWriteOptions& opt = {});

bool LoadWorldPatchJson(WorldPatch& outPatch, const std::string& path, std::string& outError);

// String-level helpers (useful for embedding patches into larger JSON documents).
bool SerializeWorldPatchJson(const WorldPatch& patch, std::string& outJson, std::string& outError,
                             const JsonWriteOptions& opt = {});

bool DeserializeWorldPatchJson(WorldPatch& outPatch, const std::string& jsonText, std::string& outError);

} // namespace isocity
