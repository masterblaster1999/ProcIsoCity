#include "isocity/MeshExport.hpp"

#include "isocity/WorldMeshBuilder.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <ostream>
#include <string>

namespace isocity {

namespace {

inline float ClampF(float v, float lo, float hi)
{
  return std::max(lo, std::min(hi, v));
}

void WriteMaterial(std::ostream& mtl, const char* name, float r, float g, float b)
{
  mtl << "newmtl " << name << "\n";
  // Simple diffuse material (good enough for previews).
  mtl << "Kd " << std::fixed << std::setprecision(4) << ClampF(r, 0.0f, 1.0f) << ' ' << ClampF(g, 0.0f, 1.0f)
      << ' ' << ClampF(b, 0.0f, 1.0f) << "\n";
  mtl << "Ka " << std::fixed << std::setprecision(4) << ClampF(r * 0.15f, 0.0f, 1.0f) << ' '
      << ClampF(g * 0.15f, 0.0f, 1.0f) << ' ' << ClampF(b * 0.15f, 0.0f, 1.0f) << "\n";
  mtl << "Ks 0.0000 0.0000 0.0000\n";
  mtl << "Ns 10.0000\n\n";
}

struct ObjWriter {
  std::ostream& obj;
  MeshExportStats* stats = nullptr;

  std::uint64_t nextIndex = 1; // OBJ indices are 1-based.
  std::string currentMtl;

  explicit ObjWriter(std::ostream& o, MeshExportStats* st) : obj(o), stats(st) {}

  void UseMaterial(const char* name)
  {
    if (!name) return;
    if (currentMtl == name) return;
    currentMtl = name;
    obj << "usemtl " << name << "\n";
  }

  std::uint64_t AddVertex(const MeshV3& v)
  {
    obj << "v " << std::fixed << std::setprecision(6) << v.x << ' ' << v.y << ' ' << v.z << "\n";
    if (stats) stats->vertices++;
    return nextIndex++;
  }

  void AddTri(std::uint64_t a, std::uint64_t b, std::uint64_t c)
  {
    obj << "f " << a << ' ' << b << ' ' << c << "\n";
    if (stats) stats->triangles++;
  }

  void AddQuad(const MeshV3& a, const MeshV3& b, const MeshV3& c, const MeshV3& d)
  {
    const std::uint64_t i0 = AddVertex(a);
    const std::uint64_t i1 = AddVertex(b);
    const std::uint64_t i2 = AddVertex(c);
    const std::uint64_t i3 = AddVertex(d);
    AddTri(i0, i1, i2);
    AddTri(i0, i2, i3);
  }
};

class ObjSink final : public IMeshSink {
public:
  explicit ObjSink(ObjWriter& w) : writer_(w) {}

  void addQuad(const MeshQuad& q) override
  {
    writer_.UseMaterial(ObjMaterialName(q.material));
    writer_.AddQuad(q.a, q.b, q.c, q.d);
  }

private:
  ObjWriter& writer_;
};

} // namespace

bool WriteWorldObjMtl(std::ostream& objOut, std::ostream& mtlOut, const World& world,
                      const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError)
{
  if (outError) outError->clear();
  if (outStats) *outStats = {};

  // Validate config early so we don't write partial outputs on error.
  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int originX = 0, originY = 0;
  if (!ComputeMeshExportBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
    return false;
  }

  // --- MTL ---
  // Keep materials stable: same names across versions, so downstream tooling can cache.
  WriteMaterial(mtlOut, "mat_water", 0.10f, 0.35f, 0.90f);
  WriteMaterial(mtlOut, "mat_sand", 0.85f, 0.80f, 0.45f);
  WriteMaterial(mtlOut, "mat_grass", 0.20f, 0.70f, 0.20f);

  WriteMaterial(mtlOut, "mat_road", 0.20f, 0.20f, 0.22f);
  WriteMaterial(mtlOut, "mat_res", 0.25f, 0.80f, 0.35f);
  WriteMaterial(mtlOut, "mat_com", 0.25f, 0.55f, 0.95f);
  WriteMaterial(mtlOut, "mat_ind", 0.95f, 0.55f, 0.20f);
  WriteMaterial(mtlOut, "mat_park", 0.15f, 0.85f, 0.15f);

  WriteMaterial(mtlOut, "mat_cliff", 0.45f, 0.35f, 0.25f);
  WriteMaterial(mtlOut, "mat_building", 0.75f, 0.75f, 0.75f);
  WriteMaterial(mtlOut, "mat_building_res", 0.70f, 0.90f, 0.75f);
  WriteMaterial(mtlOut, "mat_building_com", 0.65f, 0.75f, 0.95f);
  WriteMaterial(mtlOut, "mat_building_ind", 0.95f, 0.75f, 0.55f);

  // --- OBJ header ---
  objOut << "# ProcIsoCity OBJ export\n";
  objOut << "# world: " << world.width() << "x" << world.height() << " seed=" << world.seed() << "\n";

  if (!cfg.mtlFileName.empty()) {
    objOut << "mtllib " << cfg.mtlFileName << "\n";
  }

  const std::string objName = cfg.objectName.empty() ? std::string("world") : cfg.objectName;
  objOut << "o " << objName << "\n";

  ObjWriter w(objOut, outStats);
  ObjSink sink(w);

  if (!BuildWorldMeshQuads(world, cfg, sink, outError)) {
    return false;
  }

  // Flush errors.
  if (!objOut.good() || !mtlOut.good()) {
    if (outError) *outError = "write failed (stream error)";
    return false;
  }

  return true;
}

bool ExportWorldObjMtl(const std::string& objPath, const std::string& mtlPath, const World& world,
                       const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError)
{
  if (outError) outError->clear();

  std::ofstream objFile(objPath, std::ios::binary);
  if (!objFile) {
    if (outError) *outError = "failed to open obj for writing: " + objPath;
    return false;
  }

  std::ofstream mtlFile(mtlPath, std::ios::binary);
  if (!mtlFile) {
    if (outError) *outError = "failed to open mtl for writing: " + mtlPath;
    return false;
  }

  MeshExportConfig local = cfg;
  if (local.mtlFileName.empty()) {
    try {
      local.mtlFileName = std::filesystem::path(mtlPath).filename().string();
    } catch (...) {
      // Fallback: use the full path.
      local.mtlFileName = mtlPath;
    }
  }

  return WriteWorldObjMtl(objFile, mtlFile, world, local, outStats, outError);
}

} // namespace isocity
