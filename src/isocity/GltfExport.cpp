#include "isocity/GltfExport.hpp"

#include "isocity/Json.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

struct V3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct N3 {
  float x = 0.0f;
  float y = 1.0f;
  float z = 0.0f;
};

struct C4 {
  std::uint8_t r = 255;
  std::uint8_t g = 255;
  std::uint8_t b = 255;
  std::uint8_t a = 255;
};

inline float ClampF(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

inline int ClampI(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

inline std::uint8_t ToU8(float f01)
{
  const float c = ClampF(f01, 0.0f, 1.0f);
  const int v = static_cast<int>(std::lround(c * 255.0f));
  return static_cast<std::uint8_t>(ClampI(v, 0, 255));
}

inline C4 RGB(float r, float g, float b)
{
  return C4{ToU8(r), ToU8(g), ToU8(b), 255};
}

C4 SurfaceColorForTile(const Tile& t)
{
  // Match the OBJ exporter palette so outputs are visually consistent.
  switch (t.overlay) {
    case Overlay::Road: return RGB(0.20f, 0.20f, 0.22f);
    case Overlay::Residential: return RGB(0.25f, 0.80f, 0.35f);
    case Overlay::Commercial: return RGB(0.25f, 0.55f, 0.95f);
    case Overlay::Industrial: return RGB(0.95f, 0.55f, 0.20f);
    case Overlay::Park: return RGB(0.15f, 0.85f, 0.15f);
    case Overlay::None: break;
  }

  switch (t.terrain) {
    case Terrain::Water: return RGB(0.10f, 0.35f, 0.90f);
    case Terrain::Sand: return RGB(0.85f, 0.80f, 0.45f);
    case Terrain::Grass: return RGB(0.20f, 0.70f, 0.20f);
  }

  return RGB(0.20f, 0.70f, 0.20f);
}

C4 CliffColor() { return RGB(0.45f, 0.35f, 0.25f); }

C4 BuildingColorForTile(const Tile& t)
{
  switch (t.overlay) {
    case Overlay::Residential: return RGB(0.70f, 0.90f, 0.75f);
    case Overlay::Commercial: return RGB(0.65f, 0.75f, 0.95f);
    case Overlay::Industrial: return RGB(0.95f, 0.75f, 0.55f);
    default: break;
  }
  return RGB(0.75f, 0.75f, 0.75f);
}

struct MeshArrays {
  std::vector<float> pos;      // x,y,z
  std::vector<float> nrm;      // x,y,z
  std::vector<std::uint8_t> col; // r,g,b,a (normalized)
  std::vector<std::uint32_t> idx;

  float minX = std::numeric_limits<float>::infinity();
  float minY = std::numeric_limits<float>::infinity();
  float minZ = std::numeric_limits<float>::infinity();
  float maxX = -std::numeric_limits<float>::infinity();
  float maxY = -std::numeric_limits<float>::infinity();
  float maxZ = -std::numeric_limits<float>::infinity();

  void addVertex(const V3& p, const N3& n, const C4& c)
  {
    pos.push_back(p.x);
    pos.push_back(p.y);
    pos.push_back(p.z);

    nrm.push_back(n.x);
    nrm.push_back(n.y);
    nrm.push_back(n.z);

    col.push_back(c.r);
    col.push_back(c.g);
    col.push_back(c.b);
    col.push_back(c.a);

    minX = std::min(minX, p.x);
    minY = std::min(minY, p.y);
    minZ = std::min(minZ, p.z);
    maxX = std::max(maxX, p.x);
    maxY = std::max(maxY, p.y);
    maxZ = std::max(maxZ, p.z);
  }

  void addQuad(const V3& a, const V3& b, const V3& c, const V3& d, const N3& n, const C4& color)
  {
    const std::uint32_t base = static_cast<std::uint32_t>(pos.size() / 3);
    addVertex(a, n, color);
    addVertex(b, n, color);
    addVertex(c, n, color);
    addVertex(d, n, color);

    // Match OBJ winding: (0,1,2) and (0,2,3)
    idx.push_back(base + 0);
    idx.push_back(base + 1);
    idx.push_back(base + 2);
    idx.push_back(base + 0);
    idx.push_back(base + 2);
    idx.push_back(base + 3);
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

MeshArrays BuildMeshArrays(const World& world, const MeshExportConfig& cfg, MeshExportStats* outStats)
{
  if (outStats) *outStats = {};

  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int originX = 0, originY = 0;
  std::string err;
  // Validation done by caller; this is just to get bounds.
  (void)ValidateAndComputeBounds(world, cfg, x0, y0, x1, y1, originX, originY, &err);

  const float tileSize = (cfg.tileSize > 0.0f) ? cfg.tileSize : 1.0f;
  const float hScale = cfg.heightScale;
  const float overlayOff = cfg.overlayOffset;
  const float cliffThr = std::max(0.0f, cfg.cliffThreshold);

  MeshArrays m;

  // --- Top surfaces ---
  if (cfg.includeTopSurfaces) {
    const N3 up{0.0f, 1.0f, 0.0f};
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const Tile& t = world.at(x, y);
        const float baseH = t.height * hScale;
        const float topY = baseH + ((t.overlay != Overlay::None) ? overlayOff : 0.0f);

        const float fx0 = static_cast<float>(x - originX) * tileSize;
        const float fx1 = static_cast<float>(x + 1 - originX) * tileSize;
        const float fz0 = static_cast<float>(y - originY) * tileSize;
        const float fz1 = static_cast<float>(y + 1 - originY) * tileSize;

        const C4 c = SurfaceColorForTile(t);
        m.addQuad(V3{fx0, topY, fz0}, V3{fx1, topY, fz0}, V3{fx1, topY, fz1}, V3{fx0, topY, fz1}, up, c);
      }
    }
  }

  // --- Cliffs ---
  if (cfg.includeCliffs) {
    const C4 c = CliffColor();

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

        // Right boundary (x+1): vertical wall at X=fx1.
        if (x + 1 < x1) {
          const float h1 = hAt(x + 1, y);
          const float dh = h0 - h1;
          if (std::fabs(dh) > cliffThr) {
            const float top = std::max(h0, h1);
            const float bot = std::min(h0, h1);
            const float xp = fx1;
            const N3 n{(dh > 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f};
            m.addQuad(V3{xp, top, fz0}, V3{xp, top, fz1}, V3{xp, bot, fz1}, V3{xp, bot, fz0}, n, c);
          }
        }

        // Bottom boundary (y+1): vertical wall at Z=fz1.
        if (y + 1 < y1) {
          const float h1 = hAt(x, y + 1);
          const float dh = h0 - h1;
          if (std::fabs(dh) > cliffThr) {
            const float top = std::max(h0, h1);
            const float bot = std::min(h0, h1);
            const float zp = fz1;
            const N3 n{0.0f, 0.0f, (dh > 0.0f) ? 1.0f : -1.0f};
            m.addQuad(V3{fx0, top, zp}, V3{fx1, top, zp}, V3{fx1, bot, zp}, V3{fx0, bot, zp}, n, c);
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

        const C4 c = BuildingColorForTile(t);

        // Roof.
        m.addQuad(V3{fx0, y1b, fz0}, V3{fx1, y1b, fz0}, V3{fx1, y1b, fz1}, V3{fx0, y1b, fz1}, N3{0.0f, 1.0f, 0.0f}, c);

        // North (z0)
        m.addQuad(V3{fx0, y1b, fz0}, V3{fx1, y1b, fz0}, V3{fx1, y0b, fz0}, V3{fx0, y0b, fz0}, N3{0.0f, 0.0f, -1.0f}, c);
        // South (z1)
        m.addQuad(V3{fx0, y1b, fz1}, V3{fx1, y1b, fz1}, V3{fx1, y0b, fz1}, V3{fx0, y0b, fz1}, N3{0.0f, 0.0f, 1.0f}, c);
        // West (x0)
        m.addQuad(V3{fx0, y1b, fz0}, V3{fx0, y1b, fz1}, V3{fx0, y0b, fz1}, V3{fx0, y0b, fz0}, N3{-1.0f, 0.0f, 0.0f}, c);
        // East (x1)
        m.addQuad(V3{fx1, y1b, fz0}, V3{fx1, y1b, fz1}, V3{fx1, y0b, fz1}, V3{fx1, y0b, fz0}, N3{1.0f, 0.0f, 0.0f}, c);
      }
    }
  }

  if (outStats) {
    outStats->vertices = static_cast<std::uint64_t>(m.pos.size() / 3);
    outStats->triangles = static_cast<std::uint64_t>(m.idx.size() / 3);
  }
  return m;
}

inline void AppendU32LE(std::vector<std::uint8_t>& out, std::uint32_t v)
{
  out.push_back(static_cast<std::uint8_t>(v & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xFFu));
}

inline void AppendF32LE(std::vector<std::uint8_t>& out, float f)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  std::uint32_t u = 0;
  std::memcpy(&u, &f, 4);
  AppendU32LE(out, u);
}

inline void Align4(std::vector<std::uint8_t>& out)
{
  while ((out.size() % 4u) != 0u) out.push_back(0);
}

struct BinLayout {
  std::vector<std::uint8_t> bin;

  std::size_t posOff = 0;
  std::size_t posLen = 0;
  std::size_t nrmOff = 0;
  std::size_t nrmLen = 0;
  std::size_t colOff = 0;
  std::size_t colLen = 0;
  std::size_t idxOff = 0;
  std::size_t idxLen = 0;
};

BinLayout BuildBin(const MeshArrays& m)
{
  BinLayout b;
  b.bin.reserve(m.pos.size() * 4 + m.nrm.size() * 4 + m.col.size() + m.idx.size() * 4 + 64);

  b.posOff = b.bin.size();
  for (float f : m.pos) AppendF32LE(b.bin, f);
  b.posLen = b.bin.size() - b.posOff;
  Align4(b.bin);

  b.nrmOff = b.bin.size();
  for (float f : m.nrm) AppendF32LE(b.bin, f);
  b.nrmLen = b.bin.size() - b.nrmOff;
  Align4(b.bin);

  b.colOff = b.bin.size();
  b.bin.insert(b.bin.end(), m.col.begin(), m.col.end());
  b.colLen = b.bin.size() - b.colOff;
  Align4(b.bin);

  b.idxOff = b.bin.size();
  for (std::uint32_t v : m.idx) AppendU32LE(b.bin, v);
  b.idxLen = b.bin.size() - b.idxOff;
  Align4(b.bin);

  return b;
}

std::string BuildGltfJson(const std::string& binUriOrEmpty, std::size_t binByteLength,
                          const MeshArrays& m, const BinLayout& b,
                          const MeshExportConfig& cfg)
{
  const std::size_t vcount = m.pos.size() / 3;
  const std::size_t icount = m.idx.size();

  const std::string meshName = cfg.objectName.empty() ? std::string("world") : cfg.objectName;

  // NOTE: Keep the JSON intentionally small and stable (use fixed ordering).
  std::ostringstream os;
  os << std::fixed << std::setprecision(6);
  os << "{";
  os << "\"asset\":{";
  os << "\"version\":\"2.0\",";
  os << "\"generator\":\"ProcIsoCity\"";
  os << "},";

  os << "\"scene\":0,";
  os << "\"scenes\":[{";
  os << "\"nodes\":[0]";
  os << "}],";

  os << "\"nodes\":[{";
  os << "\"mesh\":0,";
  os << "\"name\":\"" << JsonEscape(meshName) << "\"";
  os << "}],";

  // One white material; vertex colors provide the palette.
  os << "\"materials\":[{";
  os << "\"name\":\"mat_vertex_colors\",";
  os << "\"pbrMetallicRoughness\":{";
  os << "\"baseColorFactor\":[1,1,1,1],";
  os << "\"metallicFactor\":0,";
  os << "\"roughnessFactor\":1";
  os << "}";
  os << "}],";

  // Mesh with one primitive.
  os << "\"meshes\":[{";
  os << "\"name\":\"" << JsonEscape(meshName) << "\",";
  os << "\"primitives\":[{";
  os << "\"attributes\":{";
  os << "\"POSITION\":0,";
  os << "\"NORMAL\":1,";
  os << "\"COLOR_0\":2";
  os << "},";
  os << "\"indices\":3,";
  os << "\"material\":0";
  os << "}]";
  os << "}],";

  // Buffers.
  os << "\"buffers\":[{";
  if (!binUriOrEmpty.empty()) {
    os << "\"uri\":\"" << JsonEscape(binUriOrEmpty) << "\",";
  }
  os << "\"byteLength\":" << binByteLength;
  os << "}],";

  // BufferViews.
  os << "\"bufferViews\":[";
  // 0: POSITION
  os << "{";
  os << "\"buffer\":0,";
  os << "\"byteOffset\":" << b.posOff << ",";
  os << "\"byteLength\":" << b.posLen << ",";
  os << "\"target\":34962";
  os << "},";
  // 1: NORMAL
  os << "{";
  os << "\"buffer\":0,";
  os << "\"byteOffset\":" << b.nrmOff << ",";
  os << "\"byteLength\":" << b.nrmLen << ",";
  os << "\"target\":34962";
  os << "},";
  // 2: COLOR_0
  os << "{";
  os << "\"buffer\":0,";
  os << "\"byteOffset\":" << b.colOff << ",";
  os << "\"byteLength\":" << b.colLen << ",";
  os << "\"target\":34962";
  os << "},";
  // 3: INDICES
  os << "{";
  os << "\"buffer\":0,";
  os << "\"byteOffset\":" << b.idxOff << ",";
  os << "\"byteLength\":" << b.idxLen << ",";
  os << "\"target\":34963";
  os << "}";
  os << "],";

  // Accessors.
  os << "\"accessors\":[";
  // 0: POSITION
  os << "{";
  os << "\"bufferView\":0,";
  os << "\"componentType\":5126,";
  os << "\"count\":" << vcount << ",";
  os << "\"type\":\"VEC3\",";
  os << "\"min\":[" << m.minX << "," << m.minY << "," << m.minZ << "],";
  os << "\"max\":[" << m.maxX << "," << m.maxY << "," << m.maxZ << "]";
  os << "},";
  // 1: NORMAL
  os << "{";
  os << "\"bufferView\":1,";
  os << "\"componentType\":5126,";
  os << "\"count\":" << vcount << ",";
  os << "\"type\":\"VEC3\"";
  os << "},";
  // 2: COLOR_0 (UNSIGNED_BYTE normalized)
  os << "{";
  os << "\"bufferView\":2,";
  os << "\"componentType\":5121,";
  os << "\"normalized\":true,";
  os << "\"count\":" << vcount << ",";
  os << "\"type\":\"VEC4\"";
  os << "},";
  // 3: indices (UNSIGNED_INT)
  os << "{";
  os << "\"bufferView\":3,";
  os << "\"componentType\":5125,";
  os << "\"count\":" << icount << ",";
  os << "\"type\":\"SCALAR\"";
  os << "}";
  os << "]";

  os << "}";
  return os.str();
}

bool WriteBinaryFile(const std::string& path, const std::vector<std::uint8_t>& data, std::string& outError)
{
  outError.clear();
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open for write: " + path;
    return false;
  }
  if (!data.empty()) {
    f.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  }
  if (!f.good()) {
    outError = "write failed: " + path;
    return false;
  }
  return true;
}

bool WriteTextFile(const std::string& path, const std::string& text, std::string& outError)
{
  outError.clear();
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open for write: " + path;
    return false;
  }
  f.write(text.data(), static_cast<std::streamsize>(text.size()));
  if (!f.good()) {
    outError = "write failed: " + path;
    return false;
  }
  return true;
}

std::vector<std::uint8_t> BuildGlb(const std::string& json, const std::vector<std::uint8_t>& bin)
{
  // GLB chunks must be 4-byte aligned.
  std::string jsonPadded = json;
  while ((jsonPadded.size() % 4u) != 0u) jsonPadded.push_back(' ');

  std::vector<std::uint8_t> binPadded = bin;
  while ((binPadded.size() % 4u) != 0u) binPadded.push_back(0);

  const std::uint32_t jsonLen = static_cast<std::uint32_t>(jsonPadded.size());
  const std::uint32_t binLen = static_cast<std::uint32_t>(binPadded.size());

  const std::uint32_t totalLen = 12u + 8u + jsonLen + 8u + binLen;

  std::vector<std::uint8_t> out;
  out.reserve(totalLen);

  // Header
  // magic 'glTF' = 0x46546C67
  AppendU32LE(out, 0x46546C67u);
  AppendU32LE(out, 2u);
  AppendU32LE(out, totalLen);

  // JSON chunk header
  AppendU32LE(out, jsonLen);
  AppendU32LE(out, 0x4E4F534Au); // 'JSON'
  out.insert(out.end(), jsonPadded.begin(), jsonPadded.end());

  // BIN chunk header
  AppendU32LE(out, binLen);
  AppendU32LE(out, 0x004E4942u); // 'BIN\0'
  out.insert(out.end(), binPadded.begin(), binPadded.end());

  return out;
}

} // namespace

bool ExportWorldGltf(const std::string& gltfPath, const World& world,
                     const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError)
{
  if (outError) outError->clear();
  if (outStats) *outStats = {};

  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int originX = 0, originY = 0;
  if (!ValidateAndComputeBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
    return false;
  }
  (void)x0;
  (void)y0;
  (void)x1;
  (void)y1;
  (void)originX;
  (void)originY;

  const MeshArrays mesh = BuildMeshArrays(world, cfg, outStats);
  if (mesh.pos.empty() || mesh.idx.empty()) {
    if (outError) *outError = "export produced an empty mesh (enable at least one of: top surfaces, cliffs, buildings)";
    return false;
  }
  const BinLayout bin = BuildBin(mesh);

  std::filesystem::path p(gltfPath);
  std::filesystem::path binPath = p;
  if (binPath.has_extension()) {
    binPath.replace_extension(".bin");
  } else {
    binPath += ".bin";
  }
  const std::string binFileName = binPath.filename().string();

  const std::string json = BuildGltfJson(binFileName, bin.bin.size(), mesh, bin, cfg);

  std::string err;
  if (!WriteBinaryFile(binPath.string(), bin.bin, err)) {
    if (outError) *outError = err;
    return false;
  }
  if (!WriteTextFile(p.string(), json, err)) {
    if (outError) *outError = err;
    return false;
  }

  return true;
}

bool ExportWorldGlb(const std::string& glbPath, const World& world,
                    const MeshExportConfig& cfg, MeshExportStats* outStats, std::string* outError)
{
  if (outError) outError->clear();
  if (outStats) *outStats = {};

  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int originX = 0, originY = 0;
  if (!ValidateAndComputeBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
    return false;
  }
  (void)x0;
  (void)y0;
  (void)x1;
  (void)y1;
  (void)originX;
  (void)originY;

  const MeshArrays mesh = BuildMeshArrays(world, cfg, outStats);
  if (mesh.pos.empty() || mesh.idx.empty()) {
    if (outError) *outError = "export produced an empty mesh (enable at least one of: top surfaces, cliffs, buildings)";
    return false;
  }
  const BinLayout bin = BuildBin(mesh);

  const std::string json = BuildGltfJson(/*binUriOrEmpty=*/"", bin.bin.size(), mesh, bin, cfg);
  const std::vector<std::uint8_t> glb = BuildGlb(json, bin.bin);

  std::string err;
  if (!WriteBinaryFile(glbPath, glb, err)) {
    if (outError) *outError = err;
    return false;
  }
  return true;
}

} // namespace isocity
