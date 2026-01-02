#include "isocity/GltfExport.hpp"

#include "isocity/Json.hpp"
#include "isocity/WorldMeshBuilder.hpp"

#include <algorithm>
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

struct MeshArrays {
  std::vector<float> pos;        // x,y,z
  std::vector<float> nrm;        // x,y,z
  std::vector<std::uint8_t> col; // r,g,b,a (normalized)
  std::vector<std::uint32_t> idx;

  float minX = std::numeric_limits<float>::infinity();
  float minY = std::numeric_limits<float>::infinity();
  float minZ = std::numeric_limits<float>::infinity();
  float maxX = -std::numeric_limits<float>::infinity();
  float maxY = -std::numeric_limits<float>::infinity();
  float maxZ = -std::numeric_limits<float>::infinity();

  void addVertex(const MeshV3& p, const MeshN3& n, const MeshC4& c)
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

  void addQuad(const MeshQuad& q)
  {
    const std::uint32_t base = static_cast<std::uint32_t>(pos.size() / 3);

    addVertex(q.a, q.n, q.color);
    addVertex(q.b, q.n, q.color);
    addVertex(q.c, q.n, q.color);
    addVertex(q.d, q.n, q.color);

    // Match OBJ winding: (0,1,2) and (0,2,3)
    idx.push_back(base + 0);
    idx.push_back(base + 1);
    idx.push_back(base + 2);
    idx.push_back(base + 0);
    idx.push_back(base + 2);
    idx.push_back(base + 3);
  }
};

class GltfMeshSink final : public IMeshSink {
public:
  explicit GltfMeshSink(MeshArrays& arrays) : arrays_(arrays) {}

  void addQuad(const MeshQuad& q) override { arrays_.addQuad(q); }

private:
  MeshArrays& arrays_;
};

MeshArrays BuildMeshArrays(const World& world, const MeshExportConfig& cfg, MeshExportStats* outStats)
{
  if (outStats) *outStats = {};

  MeshArrays m;
  GltfMeshSink sink(m);

  // If this fails, cfg is invalid (most commonly bad crop). The caller already
  // validated via ComputeMeshExportBounds, but we keep this defensive.
  std::string err;
  (void)BuildWorldMeshQuads(world, cfg, sink, &err);

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
  if (!ComputeMeshExportBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
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
  if (!ComputeMeshExportBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
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
