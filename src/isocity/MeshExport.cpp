#include "isocity/MeshExport.hpp"

#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>

namespace isocity {

namespace {

struct V3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

inline float ClampF(float v, float lo, float hi)
{
  return std::max(lo, std::min(hi, v));
}

inline int ClampI(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

const char* SurfaceMaterialForTile(const Tile& t)
{
  // Overlays take precedence so exports are visually legible.
  switch (t.overlay) {
    case Overlay::Road: return "mat_road";
    case Overlay::Residential: return "mat_res";
    case Overlay::Commercial: return "mat_com";
    case Overlay::Industrial: return "mat_ind";
    case Overlay::Park: return "mat_park";
    case Overlay::None: break;
  }

  switch (t.terrain) {
    case Terrain::Water: return "mat_water";
    case Terrain::Sand: return "mat_sand";
    case Terrain::Grass: return "mat_grass";
  }

  return "mat_grass";
}

const char* BuildingMaterialForTile(const Tile& t)
{
  switch (t.overlay) {
    case Overlay::Residential: return "mat_building_res";
    case Overlay::Commercial: return "mat_building_com";
    case Overlay::Industrial: return "mat_building_ind";
    default: return "mat_building";
  }
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

  std::uint64_t AddVertex(const V3& v)
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

  void AddQuad(const V3& a, const V3& b, const V3& c, const V3& d)
  {
    const std::uint64_t i0 = AddVertex(a);
    const std::uint64_t i1 = AddVertex(b);
    const std::uint64_t i2 = AddVertex(c);
    const std::uint64_t i3 = AddVertex(d);
    AddTri(i0, i1, i2);
    AddTri(i0, i2, i3);
  }
};

bool ValidateAndComputeBounds(const World& world, const MeshExportConfig& cfg,
                              int& outX0, int& outY0, int& outX1, int& outY1,
                              int& outOriginX, int& outOriginY,
                              std::string* outError)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    if (outError) *outError = "world has invalid dimensions";
    return false;
  }

  int x0 = 0;
  int y0 = 0;
  int x1 = w;
  int y1 = h;

  if (cfg.hasCrop) {
    if (cfg.cropW <= 0 || cfg.cropH <= 0) {
      if (outError) *outError = "cropW/cropH must be positive";
      return false;
    }
    x0 = ClampI(cfg.cropX, 0, w);
    y0 = ClampI(cfg.cropY, 0, h);
    x1 = ClampI(cfg.cropX + cfg.cropW, 0, w);
    y1 = ClampI(cfg.cropY + cfg.cropH, 0, h);
    if (x1 <= x0 || y1 <= y0) {
      if (outError) *outError = "crop rectangle is empty after clamping";
      return false;
    }
  }

  outX0 = x0;
  outY0 = y0;
  outX1 = x1;
  outY1 = y1;
  outOriginX = (cfg.hasCrop && cfg.originAtCrop) ? x0 : 0;
  outOriginY = (cfg.hasCrop && cfg.originAtCrop) ? y0 : 0;
  return true;
}

} // namespace

bool WriteWorldObjMtl(std::ostream& objOut, std::ostream& mtlOut, const World& world,
                      const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError)
{
  if (outError) outError->clear();
  if (outStats) *outStats = {};

  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int originX = 0, originY = 0;
  if (!ValidateAndComputeBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
    return false;
  }

  const float tileSize = (cfg.tileSize > 0.0f) ? cfg.tileSize : 1.0f;
  const float hScale = cfg.heightScale;
  const float overlayOff = cfg.overlayOffset;
  const float cliffThr = std::max(0.0f, cfg.cliffThreshold);

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

  // --- Top surfaces ---
  if (cfg.includeTopSurfaces) {
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const Tile& t = world.at(x, y);
        const float baseH = t.height * hScale;
        const float topY = baseH + ((t.overlay != Overlay::None) ? overlayOff : 0.0f);

        const float fx0 = static_cast<float>(x - originX) * tileSize;
        const float fx1 = static_cast<float>(x + 1 - originX) * tileSize;
        const float fz0 = static_cast<float>(y - originY) * tileSize;
        const float fz1 = static_cast<float>(y + 1 - originY) * tileSize;

        w.UseMaterial(SurfaceMaterialForTile(t));
        w.AddQuad(V3{fx0, topY, fz0}, V3{fx1, topY, fz0}, V3{fx1, topY, fz1}, V3{fx0, topY, fz1});
      }
    }
  }

  // --- Cliffs (vertical walls on height discontinuities) ---
  if (cfg.includeCliffs) {
    w.UseMaterial("mat_cliff");

    auto hAt = [&](int x, int y) -> float {
      if (!world.inBounds(x, y)) return 0.0f;
      return world.at(x, y).height * hScale;
    };

    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const float h0 = hAt(x, y);

        const float fx0 = static_cast<float>(x - originX) * tileSize;
        const float fx1 = static_cast<float>(x + 1 - originX) * tileSize;
        const float fz0 = static_cast<float>(y - originY) * tileSize;
        const float fz1 = static_cast<float>(y + 1 - originY) * tileSize;

        // Right boundary (x+1).
        if (x + 1 < x1) {
          const float h1 = hAt(x + 1, y);
          const float dh = h0 - h1;
          if (std::fabs(dh) > cliffThr) {
            const float top = std::max(h0, h1);
            const float bot = std::min(h0, h1);
            const float xp = fx1;
            // Wall runs along Z.
            w.AddQuad(V3{xp, top, fz0}, V3{xp, top, fz1}, V3{xp, bot, fz1}, V3{xp, bot, fz0});
          }
        }

        // Bottom boundary (y+1).
        if (y + 1 < y1) {
          const float h1 = hAt(x, y + 1);
          const float dh = h0 - h1;
          if (std::fabs(dh) > cliffThr) {
            const float top = std::max(h0, h1);
            const float bot = std::min(h0, h1);
            const float zp = fz1;
            // Wall runs along X.
            w.AddQuad(V3{fx0, top, zp}, V3{fx1, top, zp}, V3{fx1, bot, zp}, V3{fx0, bot, zp});
          }
        }
      }
    }
  }

  // --- Buildings ---
  if (cfg.includeBuildings) {
    const float footprint = ClampF(cfg.buildingFootprint, 0.20f, 0.95f);
    const float margin = 0.5f * (1.0f - footprint) * tileSize;

    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Residential && t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) {
          continue;
        }

        const float baseH = t.height * hScale;
        const float baseY = baseH + overlayOff + 0.001f;

        const int cap = CapacityForTile(t);
        const float occ = (cap > 0) ? ClampF(static_cast<float>(t.occupants) / static_cast<float>(cap), 0.0f, 1.0f) : 0.0f;

        const int lvl = ClampZoneLevel(static_cast<int>(t.level));
        const float var01 = static_cast<float>((t.variation >> 4) & 0x0Fu) / 15.0f;
        const float height = tileSize * (cfg.buildingBaseHeight + cfg.buildingPerLevelHeight * static_cast<float>(lvl) +
                                         cfg.buildingOccHeight * occ + 0.25f * var01);

        const float fx0 = static_cast<float>(x - originX) * tileSize + margin;
        const float fx1 = static_cast<float>(x + 1 - originX) * tileSize - margin;
        const float fz0 = static_cast<float>(y - originY) * tileSize + margin;
        const float fz1 = static_cast<float>(y + 1 - originY) * tileSize - margin;

        const float y0b = baseY;
        const float y1b = baseY + std::max(0.05f * tileSize, height);

        w.UseMaterial(BuildingMaterialForTile(t));

        // Roof (top face).
        w.AddQuad(V3{fx0, y1b, fz0}, V3{fx1, y1b, fz0}, V3{fx1, y1b, fz1}, V3{fx0, y1b, fz1});

        // Sides.
        // North (z0)
        w.AddQuad(V3{fx0, y1b, fz0}, V3{fx1, y1b, fz0}, V3{fx1, y0b, fz0}, V3{fx0, y0b, fz0});
        // South (z1)
        w.AddQuad(V3{fx0, y1b, fz1}, V3{fx1, y1b, fz1}, V3{fx1, y0b, fz1}, V3{fx0, y0b, fz1});
        // West (x0)
        w.AddQuad(V3{fx0, y1b, fz0}, V3{fx0, y1b, fz1}, V3{fx0, y0b, fz1}, V3{fx0, y0b, fz0});
        // East (x1)
        w.AddQuad(V3{fx1, y1b, fz0}, V3{fx1, y1b, fz1}, V3{fx1, y0b, fz1}, V3{fx1, y0b, fz0});
      }
    }
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
