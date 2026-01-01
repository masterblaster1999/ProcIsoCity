#pragma once

#include "isocity/MeshExport.hpp"

#include <string>

namespace isocity {

// Export a world as a minimal glTF 2.0 asset.
//
// Design notes:
//  - Dependency-free (pure C++20).
//  - Deterministic output (driven entirely by tile data + cfg).
//  - Produces a single mesh with a single primitive using vertex colors (COLOR_0).
//    This keeps the JSON simple and makes downstream tooling easy.
//
// Export a .gltf JSON file plus a sibling .bin buffer.
// The .bin path is derived from gltfPath by replacing its extension with ".bin".
bool ExportWorldGltf(const std::string& gltfPath, const World& world,
                     const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError);

// Export a single-file binary .glb.
bool ExportWorldGlb(const std::string& glbPath, const World& world,
                    const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError);

} // namespace isocity
