#pragma once

#include "isocity/MeshExport.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// Shared mesh generation logic used by both the OBJ/MTL exporter and the glTF/GLB exporter.
//
// The exporters intentionally output simple, dependency-free geometry. This helper factors out
// the *geometry generation* so that:
//  - OBJ and glTF stay visually consistent (same palette, same buildings)
//  - improvements (like tile merging) automatically benefit both formats
//  - we avoid copy/pasting the same tile traversal logic in multiple files

struct MeshV3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct MeshN3 {
  float x = 0.0f;
  float y = 1.0f;
  float z = 0.0f;
};

struct MeshC4 {
  std::uint8_t r = 255;
  std::uint8_t g = 255;
  std::uint8_t b = 255;
  std::uint8_t a = 255;
};

// Material IDs used by the exporters. These are mapped to:
//  - OBJ material names (mat_*)
//  - glTF vertex colors (matching the OBJ palette)
enum class MeshMaterial : std::uint8_t {
  Water,
  Sand,
  Grass,

  Road,
  Residential,
  Commercial,
  Industrial,
  Park,

  Cliff,

  Building,
  BuildingResidential,
  BuildingCommercial,
  BuildingIndustrial,
};

// Returns the OBJ material name (as used in the .mtl file) for a material ID.
const char* ObjMaterialName(MeshMaterial m);

// Returns the flat color associated with a material ID (used for glTF vertex colors).
MeshC4 MaterialColor(MeshMaterial m);

struct MeshQuad {
  MeshV3 a;
  MeshV3 b;
  MeshV3 c;
  MeshV3 d;
  MeshN3 n;
  MeshMaterial material = MeshMaterial::Grass;
  MeshC4 color{};
};

// Mesh sink interface.
//
// Implementations can stream geometry directly (OBJ) or collect it into arrays (glTF).
class IMeshSink {
public:
  virtual ~IMeshSink() = default;

  // Called once per generated quad.
  virtual void addQuad(const MeshQuad& q) = 0;
};

// Compute the tile bounds that will be exported. This is shared so stream-based
// exporters can validate cfg before writing output.
//
// (x0,y0) inclusive, (x1,y1) exclusive. originX/originY are either 0,0 or the
// crop origin if cfg.originAtCrop is true.
bool ComputeMeshExportBounds(const World& world, const MeshExportConfig& cfg,
                             int& x0, int& y0, int& x1, int& y1,
                             int& originX, int& originY,
                             std::string* outError);

// Generate all quads for the given world/config and emit them to the sink.
// Returns false on invalid cfg (e.g., bad crop) and populates outError.
bool BuildWorldMeshQuads(const World& world, const MeshExportConfig& cfg, IMeshSink& sink, std::string* outError);

} // namespace isocity
