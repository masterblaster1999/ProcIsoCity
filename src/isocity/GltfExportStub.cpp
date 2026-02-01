#include "isocity/GltfExport.hpp"

namespace isocity {

static constexpr const char* kDisabledMsg =
  "glTF export is disabled in this build (PROCISOCITY_ENABLE_GLTF_EXPORT=OFF).";

bool ExportWorldGltf(const std::string& gltfPath,
                     const World& world,
                     const MeshExportConfig& cfg,
                     MeshExportStats* outStats,
                     std::string* outError)
{
  (void)gltfPath;
  (void)world;
  (void)cfg;
  if (outStats) *outStats = {};
  if (outError) *outError = kDisabledMsg;
  return false;
}

bool ExportWorldGlb(const std::string& glbPath,
                    const World& world,
                    const MeshExportConfig& cfg,
                    MeshExportStats* outStats,
                    std::string* outError)
{
  (void)glbPath;
  (void)world;
  (void)cfg;
  if (outStats) *outStats = {};
  if (outError) *outError = kDisabledMsg;
  return false;
}

} // namespace isocity
