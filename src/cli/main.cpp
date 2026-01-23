#include "isocity/Hash.hpp"
#include "isocity/ConfigIO.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Export.hpp"
#include "isocity/GfxTilesetAtlas.hpp"
#include "isocity/FileHash.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Version.hpp"

#include <algorithm>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  int base = 10;
  std::size_t offset = 0;
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    base = 16;
    offset = 2;
  }

  char* end = nullptr;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string::npos) return false;

  const std::string a = s.substr(0, pos);
  const std::string b = s.substr(pos + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = static_cast<float>(v);
  return true;
}

bool ParseF32Triple(const std::string& s, float* outA, float* outB, float* outC)
{
  if (!outA || !outB || !outC) return false;
  const std::size_t p0 = s.find_first_of(",xX");
  if (p0 == std::string::npos) return false;
  const std::size_t p1 = s.find_first_of(",xX", p0 + 1);
  if (p1 == std::string::npos) return false;
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
  if (!ParseF32(s.substr(0, p0), &a)) return false;
  if (!ParseF32(s.substr(p0 + 1, p1 - (p0 + 1)), &b)) return false;
  if (!ParseF32(s.substr(p1 + 1), &c)) return false;
  *outA = a;
  *outB = b;
  *outC = c;
  return true;
}

bool ParseU8Triple(const std::string& s, std::uint8_t* outA, std::uint8_t* outB, std::uint8_t* outC)
{
  if (!outA || !outB || !outC) return false;
  float fa = 0.0f, fb = 0.0f, fc = 0.0f;
  if (!ParseF32Triple(s, &fa, &fb, &fc)) return false;
  auto clampU8 = [](float v) -> std::uint8_t {
    const int i = static_cast<int>(std::lround(v));
    return static_cast<std::uint8_t>(std::clamp(i, 0, 255));
  };
  *outA = clampU8(fa);
  *outB = clampU8(fb);
  *outC = clampU8(fc);
  return true;
}

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

bool ParseJsonObjectText(const std::string& text, isocity::JsonValue& outObj, std::string& outErr)
{
  isocity::JsonValue v;
  if (!isocity::ParseJson(text, v, outErr)) return false;
  if (!v.isObject()) {
    outErr = "expected JSON object";
    return false;
  }
  outObj = std::move(v);
  outErr.clear();
  return true;
}


bool LoadJsonFileText(const std::string& path, std::string& outText, std::string& outErr)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outErr = "failed to open file";
    return false;
  }
  std::ostringstream oss;
  oss << f.rdbuf();
  if (!f && !f.eof()) {
    outErr = "failed to read file";
    return false;
  }
  outText = oss.str();
  outErr.clear();
  return true;
}

bool LoadJsonObjectFile(const std::string& path, isocity::JsonValue& outObj, std::string& outErr)
{
  std::string text;
  if (!LoadJsonFileText(path, text, outErr)) return false;

  isocity::JsonValue v;
  std::string err;
  if (!isocity::ParseJson(text, v, err)) {
    outErr = err;
    return false;
  }
  if (!v.isObject()) {
    outErr = "expected JSON object";
    return false;
  }
  outObj = std::move(v);
  outErr.clear();
  return true;
}

bool ApplyCombinedConfigPatch(const isocity::JsonValue& root,
                             isocity::ProcGenConfig& ioProc,
                             isocity::SimConfig& ioSim,
                             std::optional<isocity::JsonValue>* outProcPatch,
                             std::optional<isocity::JsonValue>* outSimPatch,
                             std::string& outErr)
{
  if (!root.isObject()) {
    outErr = "combined config JSON must be an object";
    return false;
  }

  if (outProcPatch) outProcPatch->reset();
  if (outSimPatch) outSimPatch->reset();

  bool any = false;
  std::string err;

  const isocity::JsonValue* proc = isocity::FindJsonMember(root, "proc");
  if (proc) {
    if (!proc->isObject()) {
      outErr = "proc must be an object";
      return false;
    }
    any = true;
    if (!isocity::ApplyProcGenConfigJson(*proc, ioProc, err)) {
      outErr = std::string("proc: ") + err;
      return false;
    }
    if (outProcPatch) *outProcPatch = *proc;
  }

  const isocity::JsonValue* sim = isocity::FindJsonMember(root, "sim");
  if (sim) {
    if (!sim->isObject()) {
      outErr = "sim must be an object";
      return false;
    }
    any = true;
    if (!isocity::ApplySimConfigJson(*sim, ioSim, err)) {
      outErr = std::string("sim: ") + err;
      return false;
    }
    if (outSimPatch) *outSimPatch = *sim;
  }

  if (!any) {
    outErr = "combined config must contain a 'proc' and/or 'sim' object";
    return false;
  }

  outErr.clear();
  return true;
}

struct ArtifactEntry {
  std::string kind;
  std::string path;
  std::string layer; // optional (primarily for export images)
};

bool WriteRunManifestJson(const std::string& outPath,
                          int runIdx,
                          std::uint64_t requestedSeed,
                          std::uint64_t actualSeed,
                          int w,
                          int h,
                          int days,
                          std::uint64_t worldHash,
                          const std::string& loadPath,
                          const std::vector<std::string>& argvList,
                          const isocity::ProcGenConfig& procCfg,
                          const isocity::SimConfig& simCfg,
                          const std::vector<ArtifactEntry>& artifacts,
                          std::string& outErr)
{
  using isocity::JsonValue;

  JsonValue root = JsonValue::MakeObject();
  auto add = [](JsonValue& obj, const char* key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

  // High-level provenance.
  add(root, "tool", JsonValue::MakeString("proc_isocity_cli"));
  add(root, "tool_version", JsonValue::MakeString(isocity::ProcIsoCityVersionString()));
  add(root, "tool_git_sha", JsonValue::MakeString(isocity::ProcIsoCityGitSha()));
  add(root, "build_stamp", JsonValue::MakeString(isocity::ProcIsoCityBuildStamp()));

  // Run parameters.
  add(root, "run_index", JsonValue::MakeNumber(static_cast<double>(runIdx)));
  add(root, "requested_seed", JsonValue::MakeNumber(static_cast<double>(requestedSeed)));
  add(root, "actual_seed", JsonValue::MakeNumber(static_cast<double>(actualSeed)));
  add(root, "seed_hex", JsonValue::MakeString(HexU64(actualSeed)));
  add(root, "width", JsonValue::MakeNumber(static_cast<double>(w)));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(h)));
  add(root, "days", JsonValue::MakeNumber(static_cast<double>(days)));
  add(root, "world_hash", JsonValue::MakeString(HexU64(worldHash)));

  if (!loadPath.empty()) {
    add(root, "load", JsonValue::MakeString(loadPath));
  }

  // Capture the full command line for reproducibility.
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(argvList.size());
    for (const auto& a : argvList) {
      arr.arrayValue.emplace_back(JsonValue::MakeString(a));
    }
    add(root, "argv", std::move(arr));
  }

  // Embed configs (same schema as other tools) so the manifest is standalone.
  {
    std::string err;
    JsonValue procObj;
    if (!ParseJsonObjectText(isocity::ProcGenConfigToJson(procCfg, 2), procObj, err)) {
      outErr = std::string("failed to serialize ProcGenConfig: ") + err;
      return false;
    }
    add(root, "proc", std::move(procObj));
  }

  {
    std::string err;
    JsonValue simObj;
    if (!ParseJsonObjectText(isocity::SimConfigToJson(simCfg, 2), simObj, err)) {
      outErr = std::string("failed to serialize SimConfig: ") + err;
      return false;
    }
    add(root, "sim", std::move(simObj));
  }

  // Output artifacts with file hashes (FNV-1a 64-bit) for quick integrity checks.
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(artifacts.size());

    for (const auto& a : artifacts) {
      JsonValue obj = JsonValue::MakeObject();
      add(obj, "kind", JsonValue::MakeString(a.kind));
      add(obj, "path", JsonValue::MakeString(a.path));
      if (!a.layer.empty()) add(obj, "layer", JsonValue::MakeString(a.layer));

      isocity::FileHashInfo info;
      std::string herr;
      if (!isocity::ComputeFileHashFNV1a64(a.path, info, herr)) {
        add(obj, "hash_error", JsonValue::MakeString(herr));
      } else {
        add(obj, "size_bytes", JsonValue::MakeNumber(static_cast<double>(info.sizeBytes)));
        add(obj, "hash_fnv1a64", JsonValue::MakeString(HexU64(info.fnv1a64)));
      }

      arr.arrayValue.emplace_back(std::move(obj));
    }

    add(root, "artifacts", std::move(arr));
  }

  const isocity::JsonWriteOptions wopt{.pretty = true, .indent = 2, .sortKeys = false};

  // Match the CLI's convention: "-" means stdout.
  if (outPath.empty() || outPath == "-") {
    std::cout << isocity::JsonStringify(root, wopt);
    outErr.clear();
    return true;
  }

  std::string err;
  if (!isocity::WriteJsonFile(outPath, root, err, wopt)) {
    outErr = err;
    return false;
  }

  outErr.clear();
  return true;
}


void PrintHelp()
{
  std::cout
      << "proc_isocity_cli v" << isocity::ProcIsoCityFullVersionString() << " (headless simulation runner)\n\n"
      << "Usage:\n"
      << "  proc_isocity_cli --version\n"
      << "  proc_isocity_cli --build-info\n"
      << "  proc_isocity_cli [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                 [--config <combined.json>] [--proc <proc.json>] [--sim <sim.json>]\n"
      << "                 [--gen-preset <name>] [--gen-preset-strength <N>]\n"
      << "                 [--gen-road-layout <organic|grid|radial|space_colonization>]\n"
      << "                 [--gen-road-hierarchy <0|1>] [--gen-road-hierarchy-strength <N>]\n"
      << "                 [--gen-districting-mode <voronoi|road_flow|block_graph>] [--days <N>]\n"
      << "                 [--out <summary.json>] [--csv <ticks.csv>] [--save <save.bin>] [--manifest <manifest.json>]\n"
      << "                 [--require-outside <0|1>] [--tax-res <N>] [--tax-com <N>] [--tax-ind <N>]\n"
      << "                 [--maint-road <N>] [--maint-park <N>]\n"
      << "                 [--export-ppm <layer> <out.ppm|out.png>]... [--export-scale <N>]\n"
      << "                 [--export-iso <layer> <out.ppm|out.png>]... [--iso-tile <WxH>] [--iso-height <N>]\n"
      << "                 [--export-3d <layer> <out.ppm|out.png>]... [--3d-size <WxH>] [--3d-proj <iso|persp>]\n"
      << "                 [--3d-yaw <deg>] [--3d-pitch <deg>] [--3d-roll <deg>] [--3d-fit <0|1>] [--3d-ssaa <N>]\n"
      << "                 [--3d-target <x,y,z>] [--3d-dist <N>] [--3d-fov <deg>] [--3d-ortho <N>]\n"
      << "                 [--3d-outline <0|1>] [--3d-top <0|1>] [--3d-heightfield <0|1>]\n"
      << "                 [--3d-skirt <0|1>] [--3d-skirt-drop <N>]\n"
      << "                 [--3d-light <x,y,z>] [--3d-ambient <0..100>] [--3d-diffuse <0..100>] [--3d-bg <r,g,b>]\n"
      << "                 [--3d-fog <0|1>] [--3d-fog-strength <0..100>] [--3d-fog-start <0..100>] [--3d-fog-end <0..100>]\n"
      << "                 [--3d-gamma <0|1>] [--3d-ao <0|1>] [--3d-edge <0|1>] [--3d-tonemap <0|1>] [--3d-dither <0|1>] [--3d-post-seed <N>]\n"
      << "                 [--3d-heightscale <N>] [--3d-quant <N>] [--3d-buildings <0|1>] [--3d-cliffs <0|1>]\n"
      << "                 [--iso-margin <N>] [--iso-grid <0|1>] [--iso-cliffs <0|1>] [--iso-fancy <0|1>]\n"
      << "                 [--iso-texture <0..100>] [--iso-shore <0|1>] [--iso-roadmarks <0|1>] [--iso-zonepatterns <0|1>]\n"
      << "                 [--iso-daynight <0|1>] [--iso-time <0..100>] [--iso-lights <0|1>] [--iso-night <0..100>] [--iso-dusk <0..100>]\n"
      << "                 [--iso-weather <clear|rain|snow>] [--iso-wx-intensity <0..100>] [--iso-wx-overcast <0..100>] [--iso-wx-fog <0..100>]\n"
      << "                 [--iso-wx-precip <0|1>] [--iso-wx-reflect <0|1>] [--iso-clouds <0|1>] [--iso-cloud-cover <0..100>] [--iso-cloud-strength <0..100>] [--iso-cloud-scale <N>]\n"
      << "                 [--iso-tileset <atlas.png> <meta.json>] [--iso-tileset-emit <emissive.png>]\n"
      << "                 [--iso-tileset-normal <normal.png>] [--iso-tileset-shadow <shadow.png>]\n"
      << "                 [--iso-tileset-light <x,y,z>] [--iso-tileset-normal-strength <0..100>] [--iso-tileset-shadow-strength <0..100>]\n"
      << "                 [--iso-tileset-props <0|1>] [--iso-tileset-tree-density <0..100>] [--iso-tileset-conifer <0..100>]\n"
      << "                 [--iso-tileset-streetlights <0|1>] [--iso-tileset-streetlight-chance <0..100>]\n"
      << "                 [--export-tiles-csv <tiles.csv>]\n"
      << "                 [--batch <N>]\n\n"
      << "Export layers (for --export-ppm / --export-iso / --export-3d):\n"
      << "  terrain overlay height landvalue traffic goods_traffic goods_fill district"
         " flood_depth ponding_depth services services_education services_health services_safety noise landuse_mix\n\n"
      << "Batch mode:\n"
      << "  - --batch N>1 runs N simulations with seeds (seed, seed+1, ...).\n"
      << "  - To write per-run files, include {seed} or {run} in any output path.\n"
      << "    Example: --out out_{seed}.json  --export-ppm overlay map_{seed}.png\n\n"
      << "Notes:\n"
      << "  - If --load is provided, the world + embedded ProcGenConfig + SimConfig are loaded from the save.\n"
      << "  - Then any CLI config overrides (JSON patches, --gen-*, --require-outside, --tax-*, --maint-*) are applied on top.\n"
      << "  - Otherwise, a new world is generated from (--seed, --size) using the effective ProcGenConfig.\n"
      << "  - When --load is used, --gen-* options do NOT regenerate the world; they only affect the config recorded in outputs / re-saves.\n"
      << "  - --days advances the simulator by N ticks via Simulator::stepOnce().\n"
      << "  - A stable 64-bit hash of the final world is included in the JSON output.\n"
      << "  - --manifest writes a JSON file listing all output artifacts (csv/images/saves/etc.) with their byte sizes and FNV-1a hashes.\n";
}

bool WriteJsonSummary(const isocity::World& world, std::uint64_t hash, const std::string& outPath,
                      const isocity::ProcGenConfig& procCfg, const isocity::SimConfig& simCfg)
{
  const isocity::Stats& s = world.stats();

  using isocity::JsonValue;
  JsonValue root = JsonValue::MakeObject();
  auto add = [](JsonValue& obj, const char* key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

  add(root, "tool", JsonValue::MakeString("proc_isocity_cli"));
  add(root, "tool_version", JsonValue::MakeString(isocity::ProcIsoCityVersionString()));
  add(root, "tool_git_sha", JsonValue::MakeString(isocity::ProcIsoCityGitSha()));
  add(root, "build_stamp", JsonValue::MakeString(isocity::ProcIsoCityBuildStamp()));

  add(root, "width", JsonValue::MakeNumber(static_cast<double>(world.width())));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(world.height())));
  add(root, "seed", JsonValue::MakeNumber(static_cast<double>(world.seed())));
  add(root, "seed_hex", JsonValue::MakeString(HexU64(world.seed())));
  add(root, "hash", JsonValue::MakeString(HexU64(hash)));

  // Embed the generator/tool version so outputs can be compared across releases.
  add(root, "procisocity_version", JsonValue::MakeString(isocity::ProcIsoCityVersionString()));
  add(root, "procisocity_git_sha", JsonValue::MakeString(isocity::ProcIsoCityGitSha()));
  add(root, "procisocity_build_stamp", JsonValue::MakeString(isocity::ProcIsoCityBuildStamp()));

  // Embed the exact configs used for this run so the JSON output is fully reproducible.
  // We reuse ConfigIO's serialization to keep field names consistent across tools.
  {
    std::string err;
    JsonValue procObj;
    if (!ParseJsonObjectText(isocity::ProcGenConfigToJson(procCfg, 2), procObj, err)) {
      std::cerr << "Failed to serialize ProcGenConfig to JSON: " << err << "\n";
      return false;
    }
    add(root, "proc", std::move(procObj));
  }

  {
    std::string err;
    JsonValue simObj;
    if (!ParseJsonObjectText(isocity::SimConfigToJson(simCfg, 2), simObj, err)) {
      std::cerr << "Failed to serialize SimConfig to JSON: " << err << "\n";
      return false;
    }
    add(root, "sim", std::move(simObj));
  }

  JsonValue st = JsonValue::MakeObject();
  add(st, "day", JsonValue::MakeNumber(static_cast<double>(s.day)));
  add(st, "population", JsonValue::MakeNumber(static_cast<double>(s.population)));
  add(st, "housingCapacity", JsonValue::MakeNumber(static_cast<double>(s.housingCapacity)));
  add(st, "jobsCapacity", JsonValue::MakeNumber(static_cast<double>(s.jobsCapacity)));
  add(st, "jobsCapacityAccessible", JsonValue::MakeNumber(static_cast<double>(s.jobsCapacityAccessible)));
  add(st, "employed", JsonValue::MakeNumber(static_cast<double>(s.employed)));
  add(st, "happiness", JsonValue::MakeNumber(static_cast<double>(s.happiness)));
  add(st, "money", JsonValue::MakeNumber(static_cast<double>(s.money)));
  add(st, "roads", JsonValue::MakeNumber(static_cast<double>(s.roads)));
  add(st, "parks", JsonValue::MakeNumber(static_cast<double>(s.parks)));
  add(st, "avgCommuteTime", JsonValue::MakeNumber(static_cast<double>(s.avgCommuteTime)));
  add(st, "trafficCongestion", JsonValue::MakeNumber(static_cast<double>(s.trafficCongestion)));
  add(st, "goodsDemand", JsonValue::MakeNumber(static_cast<double>(s.goodsDemand)));
  add(st, "goodsDelivered", JsonValue::MakeNumber(static_cast<double>(s.goodsDelivered)));
  add(st, "goodsSatisfaction", JsonValue::MakeNumber(static_cast<double>(s.goodsSatisfaction)));
  add(st, "avgLandValue", JsonValue::MakeNumber(static_cast<double>(s.avgLandValue)));
  add(st, "demandResidential", JsonValue::MakeNumber(static_cast<double>(s.demandResidential)));
  add(root, "stats", std::move(st));

  const isocity::JsonWriteOptions wopt{.pretty = true, .indent = 2, .sortKeys = false};
  std::string err;
  if (outPath.empty()) {
    std::cout << isocity::JsonStringify(root, wopt);
    return true;
  }

  return isocity::WriteJsonFile(outPath, root, err, wopt);
}

bool WriteCsvRow(std::ostream& os, const isocity::Stats& s)
{
  os << s.day << ','
     << s.population << ','
     << s.money << ','
     << s.housingCapacity << ','
     << s.jobsCapacity << ','
     << s.jobsCapacityAccessible << ','
     << s.employed << ','
     << s.happiness << ','
     << s.roads << ','
     << s.parks << ','
     << s.avgCommuteTime << ','
     << s.trafficCongestion << ','
     << s.goodsDemand << ','
     << s.goodsDelivered << ','
     << s.goodsSatisfaction << ','
     << s.avgLandValue << ','
     << s.demandResidential
     << '\n';
  return static_cast<bool>(os);
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::vector<std::string> argvList;
  argvList.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    argvList.emplace_back(argv[i] ? argv[i] : "");
  }

  std::string loadPath;
  std::string savePath;
  std::string outJson;
  std::string outCsv;
  std::string manifestPath;

  // --- New headless export options ---
  struct PpmExport {
    ExportLayer layer = ExportLayer::Overlay;
    std::string path;
  };
  std::vector<PpmExport> ppmExports;
  int exportScale = 1;
  std::string tilesCsvPath;

  // Isometric overview exports (PPM, but projected as isometric diamonds)
  struct IsoExport {
    ExportLayer layer = ExportLayer::Overlay;
    std::string path;
  };
  std::vector<IsoExport> isoExports;
  IsoOverviewConfig isoCfg{};

  // Software-rendered 3D exports (orthographic/isometric or perspective)
  struct Render3DExport {
    ExportLayer layer = ExportLayer::Overlay;
    std::string path;
  };
  std::vector<Render3DExport> render3dExports;
  Render3DConfig render3dCfg{};

  // Optional atlas-driven rendering for isometric exports.
  std::string isoTilesetAtlasPath;
  std::string isoTilesetMetaPath;
  std::string isoTilesetEmissivePath;
  std::string isoTilesetNormalPath;
  std::string isoTilesetShadowPath;

  // Batch mode (optional): run multiple seeds in one invocation.
  int batchRuns = 1;


  std::uint64_t seed = 1;
  bool seedProvided = false;

  int w = 96;
  int h = 96;
  int days = 0;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};

  std::vector<std::function<void(ProcGenConfig&, SimConfig&)>> configOps;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string val;

    if (arg == "--version" || arg == "-V") {
      std::cout << "proc_isocity_cli " << isocity::ProcIsoCityFullVersionString() << "\n";
      return 0;
    } else if (arg == "--build-info") {
      std::cout << "proc_isocity_cli " << isocity::ProcIsoCityFullVersionString() << "\n"
                << "built " << isocity::ProcIsoCityBuildStamp() << "\n";
      return 0;
    } else if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, val)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
      loadPath = val;
    } else if (arg == "--save") {
      if (!requireValue(i, val)) {
        std::cerr << "--save requires a path\n";
        return 2;
      }
      savePath = val;
    } else if (arg == "--out" || arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << arg << " requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      outCsv = val;
    } else if (arg == "--manifest") {
      if (!requireValue(i, val)) {
        std::cerr << "--manifest requires a path (use '-' for stdout)\n";
        return 2;
      }
      manifestPath = val;
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
      seedProvided = true;
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &w, &h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--gen-preset") {
      if (!requireValue(i, val)) {
        std::cerr << "--gen-preset requires a name (classic|island|archipelago|inland_sea|river_valley|mountain_ring|fjords|canyon|volcano|delta)\n";
        return 2;
      }
      isocity::ProcGenTerrainPreset p{};
      if (!isocity::ParseProcGenTerrainPreset(val, p)) {
        std::cerr << "Unknown --gen-preset: " << val << "\n";
        std::cerr << "Valid presets: classic, island, archipelago, inland_sea, river_valley, mountain_ring, fjords, canyon, volcano, delta\n";
        return 2;
      }
      procCfg.terrainPreset = p;
      configOps.emplace_back([p](ProcGenConfig& pc, SimConfig&) { pc.terrainPreset = p; });
    } else if (arg == "--gen-preset-strength") {
      float s = 1.0f;
      if (!requireValue(i, val) || !ParseF32(val, &s)) {
        std::cerr << "--gen-preset-strength requires a float\n";
        return 2;
      }
      procCfg.terrainPresetStrength = std::clamp(s, 0.0f, 5.0f);
      configOps.emplace_back([s](ProcGenConfig& pc, SimConfig&) { pc.terrainPresetStrength = std::clamp(s, 0.0f, 5.0f); });
    } else if (arg == "--gen-road-layout" || arg == "--gen-roadlayout") {
      if (!requireValue(i, val)) {
        std::cerr << "--gen-road-layout requires a layout name\n";
        return 2;
      }
      ProcGenRoadLayout layout{};
      if (!ParseProcGenRoadLayout(val, layout)) {
        std::cerr << "--gen-road-layout expects one of: organic|grid|radial|space_colonization\n";
        return 2;
      }
      procCfg.roadLayout = layout;
      configOps.emplace_back([layout](ProcGenConfig& pc, SimConfig&) { pc.roadLayout = layout; });
    } else if (arg == "--gen-road-hierarchy") {
      if (!requireValue(i, val)) {
        std::cerr << "--gen-road-hierarchy requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--gen-road-hierarchy requires 0 or 1\n";
        return 2;
      }
      procCfg.roadHierarchyEnabled = (b != 0);
      configOps.emplace_back([b](ProcGenConfig& pc, SimConfig&) { pc.roadHierarchyEnabled = (b != 0); });
    } else if (arg == "--gen-road-hierarchy-strength") {
      float s = 1.0f;
      if (!requireValue(i, val) || !ParseF32(val, &s)) {
        std::cerr << "--gen-road-hierarchy-strength requires a float\n";
        return 2;
      }
      procCfg.roadHierarchyStrength = std::clamp(s, 0.0f, 3.0f);
      configOps.emplace_back([s](ProcGenConfig& pc, SimConfig&) { pc.roadHierarchyStrength = std::clamp(s, 0.0f, 3.0f); });
    } else if (arg == "--gen-districting-mode") {
      if (!requireValue(i, val)) {
        std::cerr << "--gen-districting-mode requires a mode name\n";
        return 2;
      }
      ProcGenDistrictingMode mode{};
      if (!ParseProcGenDistrictingMode(val, mode)) {
        std::cerr << "--gen-districting-mode expects one of: voronoi|road_flow|block_graph\n";
        return 2;
      }
      procCfg.districtingMode = mode;
      configOps.emplace_back([mode](ProcGenConfig& pc, SimConfig&) { pc.districtingMode = mode; });
    } else if (arg == "--days" || arg == "--ticks") {
      if (!requireValue(i, val) || !ParseI32(val, &days) || days < 0) {
        std::cerr << arg << " requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      if (!requireValue(i, val)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      simCfg.requireOutsideConnection = (b != 0);
      configOps.emplace_back([b](ProcGenConfig&, SimConfig& sc) { sc.requireOutsideConnection = (b != 0); });
    } else if (arg == "--tax-res") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxResidential)) {
        std::cerr << "--tax-res requires an integer\n";
        return 2;
      }
      configOps.emplace_back([v = simCfg.taxResidential](ProcGenConfig&, SimConfig& sc) { sc.taxResidential = v; });
    } else if (arg == "--tax-com") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxCommercial)) {
        std::cerr << "--tax-com requires an integer\n";
        return 2;
      }
      configOps.emplace_back([v = simCfg.taxCommercial](ProcGenConfig&, SimConfig& sc) { sc.taxCommercial = v; });
    } else if (arg == "--tax-ind") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxIndustrial)) {
        std::cerr << "--tax-ind requires an integer\n";
        return 2;
      }
      configOps.emplace_back([v = simCfg.taxIndustrial](ProcGenConfig&, SimConfig& sc) { sc.taxIndustrial = v; });
    } else if (arg == "--maint-road") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenanceRoad)) {
        std::cerr << "--maint-road requires an integer\n";
        return 2;
      }
      configOps.emplace_back([v = simCfg.maintenanceRoad](ProcGenConfig&, SimConfig& sc) { sc.maintenanceRoad = v; });
    } else if (arg == "--maint-park") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenancePark)) {
        std::cerr << "--maint-park requires an integer\n";
        return 2;
      }
      configOps.emplace_back([v = simCfg.maintenancePark](ProcGenConfig&, SimConfig& sc) { sc.maintenancePark = v; });
    } else if (arg == "--proc") {
      if (!requireValue(i, val)) {
        std::cerr << "--proc requires a path\n";
        return 2;
      }

      isocity::JsonValue patch;
      std::string err;
      if (!LoadJsonObjectFile(val, patch, err)) {
        std::cerr << "Failed to load proc config JSON: " << err << "\n";
        return 1;
      }
      if (!isocity::ApplyProcGenConfigJson(patch, procCfg, err)) {
        std::cerr << "Invalid proc config JSON: " << err << "\n";
        return 1;
      }

      configOps.emplace_back([patch](ProcGenConfig& pc, SimConfig&) {
        std::string e;
        (void)isocity::ApplyProcGenConfigJson(patch, pc, e);
      });
    } else if (arg == "--sim") {
      if (!requireValue(i, val)) {
        std::cerr << "--sim requires a path\n";
        return 2;
      }

      isocity::JsonValue patch;
      std::string err;
      if (!LoadJsonObjectFile(val, patch, err)) {
        std::cerr << "Failed to load sim config JSON: " << err << "\n";
        return 1;
      }
      if (!isocity::ApplySimConfigJson(patch, simCfg, err)) {
        std::cerr << "Invalid sim config JSON: " << err << "\n";
        return 1;
      }

      configOps.emplace_back([patch](ProcGenConfig&, SimConfig& sc) {
        std::string e;
        (void)isocity::ApplySimConfigJson(patch, sc, e);
      });
    } else if (arg == "--config") {
      if (!requireValue(i, val)) {
        std::cerr << "--config requires a path\n";
        return 2;
      }

      isocity::JsonValue root;
      std::string err;
      if (!LoadJsonObjectFile(val, root, err)) {
        std::cerr << "Failed to load combined config JSON: " << err << "\n";
        return 1;
      }

      std::optional<isocity::JsonValue> procPatch;
      std::optional<isocity::JsonValue> simPatch;
      if (!ApplyCombinedConfigPatch(root, procCfg, simCfg, &procPatch, &simPatch, err)) {
        std::cerr << "Invalid combined config JSON: " << err << "\n";
        return 1;
      }

      configOps.emplace_back([procPatch, simPatch](ProcGenConfig& pc, SimConfig& sc) {
        std::string e;
        if (procPatch) (void)isocity::ApplyProcGenConfigJson(*procPatch, pc, e);
        if (simPatch) (void)isocity::ApplySimConfigJson(*simPatch, sc, e);
      });
    } else if (arg == "--export-ppm") {
      std::string layerName;
      std::string outPath;
      if (!requireValue(i, layerName) || !requireValue(i, outPath)) {
        std::cerr << "--export-ppm requires: <layer> <out.ppm>\n";
        return 2;
      }
      ExportLayer layer = ExportLayer::Overlay;
      if (!ParseExportLayer(layerName, layer)) {
        std::cerr << "Unknown export layer: " << layerName << "\n";
        std::cerr << "Valid layers: terrain overlay height landvalue traffic goods_traffic goods_fill district\n";
        return 2;
      }
      ppmExports.push_back(PpmExport{layer, outPath});
    } else if (arg == "--export-iso") {
      std::string layerName;
      std::string outPath;
      if (!requireValue(i, layerName) || !requireValue(i, outPath)) {
        std::cerr << "--export-iso requires: <layer> <out.ppm>\n";
        return 2;
      }
      ExportLayer layer = ExportLayer::Overlay;
      if (!ParseExportLayer(layerName, layer)) {
        std::cerr << "Unknown export layer: " << layerName << "\n";
        std::cerr << "Valid layers: terrain overlay height landvalue traffic goods_traffic goods_fill district\n";
        return 2;
      }
      isoExports.push_back(IsoExport{layer, outPath});
    } else if (arg == "--export-3d") {
      std::string layerName;
      std::string outPath;
      if (!requireValue(i, layerName) || !requireValue(i, outPath)) {
        std::cerr << "--export-3d requires: <layer> <out.ppm>\n";
        return 2;
      }
      ExportLayer layer = ExportLayer::Overlay;
      if (!ParseExportLayer(layerName, layer)) {
        std::cerr << "Unknown export layer: " << layerName << "\n";
        std::cerr << "Valid layers: terrain overlay height landvalue traffic goods_traffic goods_fill district\n";
        return 2;
      }
      render3dExports.push_back(Render3DExport{layer, outPath});
    } else if (arg == "--3d-size") {
      int ow = 0;
      int oh = 0;
      if (!requireValue(i, val) || !ParseWxH(val, &ow, &oh)) {
        std::cerr << "--3d-size requires format WxH (e.g. 1600x900)\n";
        return 2;
      }
      render3dCfg.width = ow;
      render3dCfg.height = oh;
    } else if (arg == "--3d-proj") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-proj requires: iso|persp\n";
        return 2;
      }
      if (val == "iso" || val == "isometric" || val == "ortho" || val == "orthographic") {
        render3dCfg.projection = Render3DConfig::Projection::IsometricOrtho;
      } else if (val == "persp" || val == "perspective") {
        render3dCfg.projection = Render3DConfig::Projection::Perspective;
      } else {
        std::cerr << "--3d-proj requires: iso|persp\n";
        return 2;
      }
    } else if (arg == "--3d-yaw") {
      float deg = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &deg)) {
        std::cerr << "--3d-yaw requires a float (degrees)\n";
        return 2;
      }
      render3dCfg.yawDeg = deg;
    } else if (arg == "--3d-pitch") {
      float deg = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &deg)) {
        std::cerr << "--3d-pitch requires a float (degrees)\n";
        return 2;
      }
      render3dCfg.pitchDeg = deg;
    } else if (arg == "--3d-roll") {
      float deg = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &deg)) {
        std::cerr << "--3d-roll requires a float (degrees)\n";
        return 2;
      }
      render3dCfg.rollDeg = deg;
    } else if (arg == "--3d-target") {
      float tx = 0.0f, ty = 0.0f, tz = 0.0f;
      if (!requireValue(i, val) || !ParseF32Triple(val, &tx, &ty, &tz)) {
        std::cerr << "--3d-target requires: x,y,z\n";
        return 2;
      }
      render3dCfg.targetX = tx;
      render3dCfg.targetY = ty;
      render3dCfg.targetZ = tz;
    } else if (arg == "--3d-dist") {
      float d = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &d) || !(d > 0.0f)) {
        std::cerr << "--3d-dist requires a float > 0\n";
        return 2;
      }
      render3dCfg.distance = d;
    } else if (arg == "--3d-fov") {
      float deg = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &deg)) {
        std::cerr << "--3d-fov requires a float (degrees)\n";
        return 2;
      }
      render3dCfg.fovYDeg = deg;
    } else if (arg == "--3d-ortho") {
      float hh = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &hh) || !(hh > 0.0f)) {
        std::cerr << "--3d-ortho requires a float > 0 (half-height in world units)\n";
        return 2;
      }
      render3dCfg.orthoHalfHeight = hh;
    } else if (arg == "--3d-fit") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-fit requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-fit requires 0 or 1\n";
        return 2;
      }
      render3dCfg.autoFit = (b != 0);
    } else if (arg == "--3d-ssaa") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ssaa requires an integer >= 1\n";
        return 2;
      }
      int ss = 1;
      if (!ParseI32(val, &ss) || ss < 1 || ss > 4) {
        std::cerr << "--3d-ssaa requires an integer in [1..4]\n";
        return 2;
      }
      render3dCfg.supersample = ss;
    } else if (arg == "--3d-outline") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-outline requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-outline requires 0 or 1\n";
        return 2;
      }
      render3dCfg.drawOutlines = (b != 0);
    } else if (arg == "--3d-light") {
      float lx = 0.0f, ly = 0.0f, lz = 0.0f;
      if (!requireValue(i, val) || !ParseF32Triple(val, &lx, &ly, &lz)) {
        std::cerr << "--3d-light requires: x,y,z\n";
        return 2;
      }
      render3dCfg.lightDirX = lx;
      render3dCfg.lightDirY = ly;
      render3dCfg.lightDirZ = lz;
    } else if (arg == "--3d-ambient") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ambient requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-ambient requires 0..100\n";
        return 2;
      }
      render3dCfg.ambient = static_cast<float>(p) / 100.0f;
    } else if (arg == "--3d-diffuse") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-diffuse requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-diffuse requires 0..100\n";
        return 2;
      }
      render3dCfg.diffuse = static_cast<float>(p) / 100.0f;
    } else if (arg == "--3d-bg") {
      std::uint8_t r = 0, g = 0, b = 0;
      if (!requireValue(i, val) || !ParseU8Triple(val, &r, &g, &b)) {
        std::cerr << "--3d-bg requires: r,g,b (0..255)\n";
        return 2;
      }
      render3dCfg.bgR = r;
      render3dCfg.bgG = g;
      render3dCfg.bgB = b;
    } else if (arg == "--3d-fog") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-fog requires 0 or 1\n";
        return 2;
      }
      int f = 0;
      if (!ParseI32(val, &f) || (f != 0 && f != 1)) {
        std::cerr << "--3d-fog requires 0 or 1\n";
        return 2;
      }
      render3dCfg.fog = (f != 0);
    } else if (arg == "--3d-fog-strength") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-fog-strength requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-fog-strength requires 0..100\n";
        return 2;
      }
      render3dCfg.fogStrength = static_cast<float>(p) / 100.0f;
    } else if (arg == "--3d-fog-start") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-fog-start requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-fog-start requires 0..100\n";
        return 2;
      }
      render3dCfg.fogStart = static_cast<float>(p) / 100.0f;
    } else if (arg == "--3d-fog-end") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-fog-end requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-fog-end requires 0..100\n";
        return 2;
      }
      render3dCfg.fogEnd = static_cast<float>(p) / 100.0f;
    } else if (arg == "--3d-gamma") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-gamma requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-gamma requires 0 or 1\n";
        return 2;
      }
      render3dCfg.gammaCorrectDownsample = (b != 0);

    } else if (arg == "--3d-ao") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ao requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-ao requires 0 or 1\n";
        return 2;
      }
      render3dCfg.postAO = (b != 0);

    } else if (arg == "--3d-ao-strength") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ao-strength requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-ao-strength requires 0..100\n";
        return 2;
      }
      render3dCfg.aoStrength = static_cast<float>(p) / 100.0f;

    } else if (arg == "--3d-ao-radius") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ao-radius requires an int >= 1\n";
        return 2;
      }
      int r = 0;
      if (!ParseI32(val, &r) || r < 1) {
        std::cerr << "--3d-ao-radius requires an int >= 1\n";
        return 2;
      }
      render3dCfg.aoRadiusPx = r;

    } else if (arg == "--3d-ao-range") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v > 0.0f)) {
        std::cerr << "--3d-ao-range requires a float > 0 (depth units, ~0.01..0.05 typical)\n";
        return 2;
      }
      render3dCfg.aoRange = v;

    } else if (arg == "--3d-ao-bias") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v >= 0.0f)) {
        std::cerr << "--3d-ao-bias requires a float >= 0\n";
        return 2;
      }
      render3dCfg.aoBias = v;

    } else if (arg == "--3d-ao-power") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v > 0.0f)) {
        std::cerr << "--3d-ao-power requires a float > 0\n";
        return 2;
      }
      render3dCfg.aoPower = v;

    } else if (arg == "--3d-ao-samples") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ao-samples requires an int 4..32\n";
        return 2;
      }
      int s = 0;
      if (!ParseI32(val, &s) || s < 4 || s > 32) {
        std::cerr << "--3d-ao-samples requires an int 4..32\n";
        return 2;
      }
      render3dCfg.aoSamples = s;

    } else if (arg == "--3d-ao-blur") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-ao-blur requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-ao-blur requires 0 or 1\n";
        return 2;
      }
      render3dCfg.aoBlurRadiusPx = (b != 0) ? 1 : 0;

    } else if (arg == "--3d-edge") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-edge requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-edge requires 0 or 1\n";
        return 2;
      }
      render3dCfg.postEdge = (b != 0);

    } else if (arg == "--3d-edge-alpha") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-edge-alpha requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-edge-alpha requires 0..100\n";
        return 2;
      }
      render3dCfg.edgeAlpha = static_cast<float>(p) / 100.0f;

    } else if (arg == "--3d-edge-threshold") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v >= 0.0f)) {
        std::cerr << "--3d-edge-threshold requires a float >= 0 (depth delta)\n";
        return 2;
      }
      render3dCfg.edgeThreshold = v;

    } else if (arg == "--3d-edge-softness") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v >= 0.0f)) {
        std::cerr << "--3d-edge-softness requires a float >= 0 (smoothstep width)\n";
        return 2;
      }
      render3dCfg.edgeSoftness = v;

    } else if (arg == "--3d-edge-radius") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-edge-radius requires an int >= 1\n";
        return 2;
      }
      int r = 0;
      if (!ParseI32(val, &r) || r < 1) {
        std::cerr << "--3d-edge-radius requires an int >= 1\n";
        return 2;
      }
      render3dCfg.edgeRadiusPx = r;

    } else if (arg == "--3d-edge-color") {
      std::uint8_t r = 0, g = 0, b = 0;
      if (!requireValue(i, val) || !ParseU8Triple(val, &r, &g, &b)) {
        std::cerr << "--3d-edge-color requires: r,g,b (0..255)\n";
        return 2;
      }
      render3dCfg.edgeR = r;
      render3dCfg.edgeG = g;
      render3dCfg.edgeB = b;

    } else if (arg == "--3d-tonemap") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-tonemap requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-tonemap requires 0 or 1\n";
        return 2;
      }
      render3dCfg.postTonemap = (b != 0);

    } else if (arg == "--3d-exposure") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v >= 0.0f)) {
        std::cerr << "--3d-exposure requires a float >= 0\n";
        return 2;
      }
      render3dCfg.exposure = v;

    } else if (arg == "--3d-contrast") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v >= 0.0f)) {
        std::cerr << "--3d-contrast requires a float >= 0\n";
        return 2;
      }
      render3dCfg.contrast = v;

    } else if (arg == "--3d-saturation") {
      float v = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &v) || !(v >= 0.0f)) {
        std::cerr << "--3d-saturation requires a float >= 0\n";
        return 2;
      }
      render3dCfg.saturation = v;

    } else if (arg == "--3d-vignette") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-vignette requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-vignette requires 0..100\n";
        return 2;
      }
      render3dCfg.vignette = static_cast<float>(p) / 100.0f;

    } else if (arg == "--3d-dither") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-dither requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-dither requires 0 or 1\n";
        return 2;
      }
      render3dCfg.postDither = (b != 0);

    } else if (arg == "--3d-dither-strength") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-dither-strength requires 0..100\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--3d-dither-strength requires 0..100\n";
        return 2;
      }
      render3dCfg.ditherStrength = static_cast<float>(p) / 100.0f;

    } else if (arg == "--3d-dither-bits") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-dither-bits requires an int 1..8\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || b < 1 || b > 8) {
        std::cerr << "--3d-dither-bits requires an int 1..8\n";
        return 2;
      }
      render3dCfg.ditherBits = b;

    } else if (arg == "--3d-post-seed") {
      std::uint64_t s = 0;
      if (!requireValue(i, val) || !ParseU64(val, &s)) {
        std::cerr << "--3d-post-seed requires a u64\n";
        return 2;
      }
      render3dCfg.postSeed = static_cast<std::uint32_t>(s & 0xFFFFFFFFu);

    } else if (arg == "--3d-heightscale") {
      float s = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &s) || !(s > 0.0f)) {
        std::cerr << "--3d-heightscale requires a float > 0\n";
        return 2;
      }
      render3dCfg.meshCfg.heightScale = s;
    } else if (arg == "--3d-quant") {
      float q = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &q) || !(q >= 0.0f)) {
        std::cerr << "--3d-quant requires a float >= 0\n";
        return 2;
      }
      render3dCfg.meshCfg.heightQuantization = q;
    } else if (arg == "--3d-buildings") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-buildings requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-buildings requires 0 or 1\n";
        return 2;
      }
      render3dCfg.meshCfg.includeBuildings = (b != 0);
    } else if (arg == "--3d-cliffs") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-cliffs requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-cliffs requires 0 or 1\n";
        return 2;
      }
      render3dCfg.meshCfg.includeCliffs = (b != 0);
    } else if (arg == "--3d-top") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-top requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-top requires 0 or 1\n";
        return 2;
      }
      render3dCfg.meshCfg.includeTopSurfaces = (b != 0);

    } else if (arg == "--3d-heightfield") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-heightfield requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-heightfield requires 0 or 1\n";
        return 2;
      }
      render3dCfg.heightfieldTopSurfaces = (b != 0);
    } else if (arg == "--3d-skirt") {
      if (!requireValue(i, val)) {
        std::cerr << "--3d-skirt requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--3d-skirt requires 0 or 1\n";
        return 2;
      }
      render3dCfg.addSkirt = (b != 0);
    } else if (arg == "--3d-skirt-drop") {
      float d = 0.0f;
      if (!requireValue(i, val) || !ParseF32(val, &d) || !(d > 0.0f)) {
        std::cerr << "--3d-skirt-drop requires a float > 0 (world units)\n";
        return 2;
      }
      render3dCfg.skirtDrop = d;
    } else if (arg == "--iso-tile") {
      int tw = 0;
      int th = 0;
      if (!requireValue(i, val) || !ParseWxH(val, &tw, &th)) {
        std::cerr << "--iso-tile requires format WxH (e.g. 16x8)\n";
        return 2;
      }
      if ((tw % 2) != 0 || (th % 2) != 0) {
        std::cerr << "--iso-tile requires even dimensions (so halfW/halfH are integers)\n";
        return 2;
      }
      isoCfg.tileW = tw;
      isoCfg.tileH = th;
    } else if (arg == "--iso-height") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-height requires an integer\n";
        return 2;
      }
      int hp = 0;
      if (!ParseI32(val, &hp) || hp < 0) {
        std::cerr << "--iso-height requires an integer >= 0\n";
        return 2;
      }
      isoCfg.heightScalePx = hp;
    } else if (arg == "--iso-margin") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-margin requires an integer\n";
        return 2;
      }
      int mp = 0;
      if (!ParseI32(val, &mp) || mp < 0) {
        std::cerr << "--iso-margin requires an integer >= 0\n";
        return 2;
      }
      isoCfg.marginPx = mp;
    } else if (arg == "--iso-grid") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-grid requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-grid requires 0 or 1\n";
        return 2;
      }
      isoCfg.drawGrid = (b != 0);
    } else if (arg == "--iso-cliffs") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-cliffs requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-cliffs requires 0 or 1\n";
        return 2;
      }
      isoCfg.drawCliffs = (b != 0);
    } else if (arg == "--iso-fancy") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-fancy requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-fancy requires 0 or 1\n";
        return 2;
      }
      isoCfg.fancy = (b != 0);
    } else if (arg == "--iso-texture") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-texture requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 100;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-texture requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.textureStrength = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-shore") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-shore requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-shore requires 0 or 1\n";
        return 2;
      }
      isoCfg.drawShore = (b != 0);
    } else if (arg == "--iso-roadmarks") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-roadmarks requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-roadmarks requires 0 or 1\n";
        return 2;
      }
      isoCfg.drawRoadMarkings = (b != 0);
    } else if (arg == "--iso-zonepatterns") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-zonepatterns requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-zonepatterns requires 0 or 1\n";
        return 2;
      }
      isoCfg.drawZonePatterns = (b != 0);
    } else if (arg == "--iso-daynight") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-daynight requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-daynight requires 0 or 1\n";
        return 2;
      }
      isoCfg.dayNight.enabled = (b != 0);
    } else if (arg == "--iso-time") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-time requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 25;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-time requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.dayNight.phase01 = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-lights") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-lights requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-lights requires 0 or 1\n";
        return 2;
      }
      isoCfg.dayNight.drawLights = (b != 0);
    } else if (arg == "--iso-night") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-night requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 80;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-night requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.dayNight.nightDarken = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-dusk") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-dusk requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 55;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-dusk requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.dayNight.duskTint = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-weather") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-weather requires one of: clear, rain, snow\n";
        return 2;
      }
      if (val == "clear") {
        isoCfg.weather.mode = isocity::IsoOverviewConfig::WeatherConfig::Mode::Clear;
      } else if (val == "rain") {
        isoCfg.weather.mode = isocity::IsoOverviewConfig::WeatherConfig::Mode::Rain;
      } else if (val == "snow") {
        isoCfg.weather.mode = isocity::IsoOverviewConfig::WeatherConfig::Mode::Snow;
      } else {
        std::cerr << "--iso-weather requires one of: clear, rain, snow\n";
        return 2;
      }
    } else if (arg == "--iso-wx-intensity") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-wx-intensity requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-wx-intensity requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.weather.intensity = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-wx-overcast") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-wx-overcast requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 65;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-wx-overcast requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.weather.overcast = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-wx-fog") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-wx-fog requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 0;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-wx-fog requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.weather.fog = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-wx-precip") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-wx-precip requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-wx-precip requires 0 or 1\n";
        return 2;
      }
      isoCfg.weather.drawPrecipitation = (b != 0);
    } else if (arg == "--iso-wx-reflect") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-wx-reflect requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-wx-reflect requires 0 or 1\n";
        return 2;
      }
      isoCfg.weather.reflectLights = (b != 0);
    } else if (arg == "--iso-clouds") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-clouds requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-clouds requires 0 or 1\n";
        return 2;
      }
      isoCfg.clouds.enabled = (b != 0);
    } else if (arg == "--iso-cloud-cover") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-cloud-cover requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 45;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-cloud-cover requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.clouds.coverage = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-cloud-strength") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-cloud-strength requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 45;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-cloud-strength requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.clouds.strength = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-cloud-scale") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-cloud-scale requires an integer >= 1 (tiles)\n";
        return 2;
      }
      int s = 24;
      if (!ParseI32(val, &s) || s < 1) {
        std::cerr << "--iso-cloud-scale requires an integer >= 1 (tiles)\n";
        return 2;
      }
      isoCfg.clouds.scaleTiles = static_cast<float>(s);
    } else if (arg == "--iso-tileset") {
      // Use a generated sprite atlas for ISO overviews.
      if (i + 2 >= argc) {
        std::cerr << "--iso-tileset requires: <atlas.png> <meta.json>\n";
        return 2;
      }
      isoTilesetAtlasPath = argv[++i];
      isoTilesetMetaPath = argv[++i];
    } else if (arg == "--iso-tileset-emit") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-emit requires: <emissive.png>\n";
        return 2;
      }
      isoTilesetEmissivePath = val;
    } else if (arg == "--iso-tileset-normal") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-normal requires: <normal.png>\n";
        return 2;
      }
      isoTilesetNormalPath = val;
      isoCfg.tilesetLighting.enableNormals = true;
    } else if (arg == "--iso-tileset-shadow") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-shadow requires: <shadow.png>\n";
        return 2;
      }
      isoTilesetShadowPath = val;
      isoCfg.tilesetLighting.enableShadows = true;
    } else if (arg == "--iso-tileset-light") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-light requires: <x,y,z>\n";
        return 2;
      }
      float lx = 0.0f, ly = 0.0f, lz = 0.0f;
      if (!ParseF32Triple(val, &lx, &ly, &lz)) {
        std::cerr << "--iso-tileset-light must be three floats: x,y,z\n";
        return 2;
      }
      isoCfg.tilesetLighting.lightDirX = lx;
      isoCfg.tilesetLighting.lightDirY = ly;
      isoCfg.tilesetLighting.lightDirZ = lz;
    } else if (arg == "--iso-tileset-normal-strength") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-normal-strength requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 100;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-tileset-normal-strength requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.tilesetLighting.normalStrength = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-tileset-shadow-strength") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-shadow-strength requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 65;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-tileset-shadow-strength requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.tilesetLighting.shadowStrength = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-tileset-props") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-props requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-tileset-props requires 0 or 1\n";
        return 2;
      }
      isoCfg.tilesetProps.enabled = (b != 0);
    } else if (arg == "--iso-tileset-tree-density") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-tree-density requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 35;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-tileset-tree-density requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.tilesetProps.treeDensity = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-tileset-conifer") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-conifer requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 35;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-tileset-conifer requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.tilesetProps.coniferChance = static_cast<float>(p) / 100.0f;
    } else if (arg == "--iso-tileset-streetlights") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-streetlights requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--iso-tileset-streetlights requires 0 or 1\n";
        return 2;
      }
      isoCfg.tilesetProps.drawStreetlights = (b != 0);
    } else if (arg == "--iso-tileset-streetlight-chance") {
      if (!requireValue(i, val)) {
        std::cerr << "--iso-tileset-streetlight-chance requires an integer percent (0..100)\n";
        return 2;
      }
      int p = 30;
      if (!ParseI32(val, &p) || p < 0 || p > 100) {
        std::cerr << "--iso-tileset-streetlight-chance requires an integer percent (0..100)\n";
        return 2;
      }
      isoCfg.tilesetProps.streetlightChance = static_cast<float>(p) / 100.0f;
    } else if (arg == "--export-scale") {
      if (!requireValue(i, val)) {
        std::cerr << "--export-scale requires an integer\n";
        return 2;
      }
      int s = 1;
      if (!ParseI32(val, &s) || s < 1) {
        std::cerr << "--export-scale requires an integer >= 1\n";
        return 2;
      }
      exportScale = s;
    } else if (arg == "--export-tiles-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--export-tiles-csv requires a path\n";
        return 2;
      }
      tilesCsvPath = val;
    } else if (arg == "--batch") {
      if (!requireValue(i, val)) {
        std::cerr << "--batch requires an integer\n";
        return 2;
      }
      int n = 1;
      if (!ParseI32(val, &n) || n < 1) {
        std::cerr << "--batch requires an integer >= 1\n";
        return 2;
      }
      batchRuns = n;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n\n";
      PrintHelp();
      return 2;
    }
  }

  if (batchRuns > 1 && !loadPath.empty()) {
    std::cerr << "--batch cannot be combined with --load\n";
    return 2;
  }

  if (!seedProvided) {
    // Keep a deterministic default so CI runs are stable.
    seed = 1;
  }

  auto hasBatchToken = [](const std::string& p) -> bool {
    return p.find("{seed}") != std::string::npos || p.find("{run}") != std::string::npos;
  };

  if (batchRuns > 1) {
    auto checkTemplate = [&](const std::string& p, const char* flag) -> bool {
      if (p.empty()) return true;
      if (hasBatchToken(p)) return true;
      std::cerr << "When using --batch, " << flag << " should include {seed} or {run} to avoid overwriting: " << p << "\n";
      return false;
    };

    if (!checkTemplate(outJson, "--out")) return 2;
    if (!checkTemplate(outCsv, "--csv")) return 2;
    if (!checkTemplate(savePath, "--save")) return 2;
    if (!checkTemplate(manifestPath, "--manifest")) return 2;
    if (!checkTemplate(tilesCsvPath, "--export-tiles-csv")) return 2;
    for (const auto& e : ppmExports) {
      if (!checkTemplate(e.path, "--export-ppm")) return 2;
    }
    for (const auto& e : isoExports) {
      if (!checkTemplate(e.path, "--export-iso")) return 2;
    }
    for (const auto& e : render3dExports) {
      if (!checkTemplate(e.path, "--export-3d")) return 2;
    }
  }

  auto replaceAll = [](std::string s, const std::string& from, const std::string& to) -> std::string {
    if (from.empty()) return s;
    std::size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
    return s;
  };

  auto expandPath = [&](const std::string& tmpl, int runIdx, std::uint64_t runSeed) -> std::string {
    if (tmpl.empty()) return {};
    std::string out = tmpl;
    out = replaceAll(out, "{seed}", std::to_string(runSeed));
    out = replaceAll(out, "{run}", std::to_string(runIdx));
    out = replaceAll(out, "{w}", std::to_string(w));
    out = replaceAll(out, "{h}", std::to_string(h));
    out = replaceAll(out, "{days}", std::to_string(days));
    return out;
  };

  auto ensureParentDir = [](const std::string& filePath) {
    if (filePath.empty()) return;
    const std::filesystem::path p(filePath);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
    }
  };

  // Optional tileset atlas for ISO exports (loaded once, reused across batch runs).
  GfxTilesetAtlas isoTileset;
  const GfxTilesetAtlas* isoTilesetPtr = nullptr;
  if (!isoTilesetAtlasPath.empty() || !isoTilesetMetaPath.empty()) {
    if (isoTilesetAtlasPath.empty() || isoTilesetMetaPath.empty()) {
      std::cerr << "--iso-tileset requires both an atlas png and a meta json\n";
      return 2;
    }
    std::string err;
    if (!LoadGfxTilesetAtlas(isoTilesetAtlasPath, isoTilesetMetaPath, isoTileset, err)) {
      std::cerr << "Failed to load ISO tileset atlas: " << err << "\n";
      return 1;
    }
    if (!isoTilesetEmissivePath.empty()) {
      if (!LoadGfxTilesetAtlasEmissive(isoTilesetEmissivePath, isoTileset, err)) {
        std::cerr << "Failed to load ISO tileset emissive atlas: " << err << "\n";
        return 1;
      }
    }
    if (!isoTilesetNormalPath.empty()) {
      if (!LoadGfxTilesetAtlasNormals(isoTilesetNormalPath, isoTileset, err)) {
        std::cerr << "Failed to load ISO tileset normal atlas: " << err << "\n";
        return 1;
      }
    }
    if (!isoTilesetShadowPath.empty()) {
      if (!LoadGfxTilesetAtlasShadows(isoTilesetShadowPath, isoTileset, err)) {
        std::cerr << "Failed to load ISO tileset shadow atlas: " << err << "\n";
        return 1;
      }
    }
    isoTilesetPtr = &isoTileset;
  }

  auto runOne = [&](int runIdx, std::uint64_t requestedSeed) -> int {
    World world;
    ProcGenConfig runProcCfg = procCfg;
    SimConfig runSimCfg = simCfg;

    std::vector<ArtifactEntry> artifacts;
    artifacts.reserve(8);

    if (!loadPath.empty()) {
      std::string err;
      if (!LoadWorldBinary(world, runProcCfg, runSimCfg, loadPath, err)) {
        std::cerr << "Failed to load save: " << err << "\n";
        return 1;
      }

      // Re-apply CLI config overrides on top of the save's embedded configs.
      for (const auto& op : configOps) {
        op(runProcCfg, runSimCfg);
      }
    } else {
      world = GenerateWorld(w, h, requestedSeed, runProcCfg);
    }

    const std::uint64_t actualSeed = world.seed();

    Simulator sim(runSimCfg);
    sim.refreshDerivedStats(world);

    std::ofstream csv;
    const std::string csvPath = expandPath(outCsv, runIdx, actualSeed);
    if (!csvPath.empty()) {
      ensureParentDir(csvPath);
      csv.open(csvPath, std::ios::binary);
      if (!csv) {
        std::cerr << "Failed to open CSV for writing: " << csvPath << "\n";
        return 1;
      }
      csv << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
      WriteCsvRow(csv, world.stats());
      artifacts.push_back(ArtifactEntry{"csv", csvPath, ""});
    }

    for (int i = 0; i < days; ++i) {
      sim.stepOnce(world);
      if (csv) WriteCsvRow(csv, world.stats());
    }

    sim.refreshDerivedStats(world);

    const std::string saveP = expandPath(savePath, runIdx, actualSeed);
    if (!saveP.empty()) {
      ensureParentDir(saveP);
      std::string err;
      if (!SaveWorldBinary(world, runProcCfg, sim.config(), saveP, err)) {
        std::cerr << "Failed to save world: " << err << "\n";
        return 1;
      }
      artifacts.push_back(ArtifactEntry{"save", saveP, ""});
    }

    const std::string tilesP = expandPath(tilesCsvPath, runIdx, actualSeed);
    if (!tilesP.empty()) {
      ensureParentDir(tilesP);
      std::string err;
      if (!WriteTilesCsv(world, tilesP, err)) {
        std::cerr << "Failed to write tiles CSV: " << tilesP;
        if (!err.empty()) std::cerr << " (" << err << ")";
        std::cerr << "\n";
        return 1;
      }
      artifacts.push_back(ArtifactEntry{"tiles_csv", tilesP, ""});
    }

    // Optional derived-map exports (images)
    if (!ppmExports.empty() || !isoExports.empty() || !render3dExports.empty()) {
      bool needTraffic = false;
      bool needGoods = false;
      bool needLandValue = false;

      for (const auto& e : ppmExports) {
        if (e.layer == ExportLayer::Traffic || e.layer == ExportLayer::Noise) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill || e.layer == ExportLayer::Noise) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }
      for (const auto& e : isoExports) {
        if (e.layer == ExportLayer::Traffic || e.layer == ExportLayer::Noise) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill || e.layer == ExportLayer::Noise) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }
      for (const auto& e : render3dExports) {
        if (e.layer == ExportLayer::Traffic || e.layer == ExportLayer::Noise) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill || e.layer == ExportLayer::Noise) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }

      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
      if (sim.config().requireOutsideConnection && (needTraffic || needGoods || needLandValue)) {
        ComputeRoadsConnectedToEdge(world, roadToEdge);
        roadToEdgeMask = &roadToEdge;
      }

      std::optional<TrafficResult> trafficRes;
      if (needTraffic || needLandValue) {
        TrafficConfig tc{};
        tc.requireOutsideConnection = sim.config().requireOutsideConnection;

        // Mirror simulator traffic model settings (so CLI exports match in-game overlays).
        tc.congestionAwareRouting = sim.trafficModel().congestionAwareRouting;
        tc.congestionIterations = sim.trafficModel().congestionIterations;
        tc.congestionAlpha = sim.trafficModel().congestionAlpha;
        tc.congestionBeta = sim.trafficModel().congestionBeta;
        tc.congestionCapacityScale = sim.trafficModel().congestionCapacityScale;
        tc.congestionRatioClamp = sim.trafficModel().congestionRatioClamp;

        const float employedShare = (world.stats().population > 0)
                                        ? static_cast<float>(world.stats().employed) / static_cast<float>(world.stats().population)
                                        : 0.0f;

        trafficRes = ComputeCommuteTraffic(world, tc, employedShare, roadToEdgeMask);
      }

      std::optional<GoodsResult> goodsRes;
      if (needGoods) {
        GoodsConfig gc{};
        gc.requireOutsideConnection = sim.config().requireOutsideConnection;
        goodsRes = ComputeGoodsFlow(world, gc, roadToEdgeMask);
      }

      std::optional<LandValueResult> landValueRes;
      if (needLandValue) {
        LandValueConfig lc{};
        landValueRes = ComputeLandValue(world, lc, trafficRes ? &(*trafficRes) : nullptr, roadToEdgeMask);
      }

      for (const auto& e : ppmExports) {
        const std::string outP = expandPath(e.path, runIdx, actualSeed);
        ensureParentDir(outP);

        PpmImage img = RenderPpmLayer(world, e.layer, landValueRes ? &(*landValueRes) : nullptr,
                                     trafficRes ? &(*trafficRes) : nullptr, goodsRes ? &(*goodsRes) : nullptr);
        if (exportScale > 1) img = ScaleNearest(img, exportScale);

        std::string err;
        if (!WriteImageAuto(outP, img, err)) {
          std::cerr << "Failed to write image (" << ExportLayerName(e.layer) << "): " << outP << " (" << err << ")\n";
          return 1;
        }
        artifacts.push_back(ArtifactEntry{"export_ppm", outP, ExportLayerName(e.layer)});
      }

      for (const auto& e : isoExports) {
        const std::string outP = expandPath(e.path, runIdx, actualSeed);
        ensureParentDir(outP);

        const IsoOverviewResult iso = RenderIsoOverview(world, e.layer, isoCfg,
                                                        landValueRes ? &(*landValueRes) : nullptr,
                                                        trafficRes ? &(*trafficRes) : nullptr,
                                                        goodsRes ? &(*goodsRes) : nullptr,
                                                        isoTilesetPtr);
        if (iso.image.width <= 0 || iso.image.height <= 0) {
          std::cerr << "Failed to render ISO overview (" << ExportLayerName(e.layer) << "): " << outP << "\n";
          return 1;
        }

        std::string err;
        if (!WriteImageAuto(outP, iso.image, err)) {
          std::cerr << "Failed to write ISO image (" << ExportLayerName(e.layer) << "): " << outP << " (" << err << ")\n";
          return 1;
        }
        artifacts.push_back(ArtifactEntry{"export_iso", outP, ExportLayerName(e.layer)});
      }

      for (const auto& e : render3dExports) {
        const std::string outP = expandPath(e.path, runIdx, actualSeed);
        ensureParentDir(outP);

        PpmImage img3d = RenderWorld3D(world, e.layer, render3dCfg,
                                       landValueRes ? &(*landValueRes) : nullptr,
                                       trafficRes ? &(*trafficRes) : nullptr,
                                       goodsRes ? &(*goodsRes) : nullptr);
        if (img3d.width <= 0 || img3d.height <= 0) {
          std::cerr << "Failed to render 3D view (" << ExportLayerName(e.layer) << "): " << outP << "\n";
          return 1;
        }

        std::string err;
        if (!WriteImageAuto(outP, img3d, err)) {
          std::cerr << "Failed to write 3D image (" << ExportLayerName(e.layer) << "): " << outP << " (" << err << ")\n";
          return 1;
        }
        artifacts.push_back(ArtifactEntry{"export_3d", outP, ExportLayerName(e.layer)});
      }
    }

    const std::uint64_t hash = HashWorld(world, true);
    const std::string jsonP = expandPath(outJson, runIdx, actualSeed);
    if (!WriteJsonSummary(world, hash, jsonP, runProcCfg, sim.config())) {
      std::cerr << "Failed to write JSON summary" << (jsonP.empty() ? "" : (": " + jsonP)) << "\n";
      return 1;
    }

    if (!jsonP.empty() && jsonP != "-") {
      artifacts.push_back(ArtifactEntry{"summary_json", jsonP, ""});
    }

    // Close any open streams before computing manifest hashes.
    if (csv) csv.close();

    const std::string manifestP = expandPath(manifestPath, runIdx, actualSeed);
    if (!manifestP.empty()) {
      if (manifestP != "-") ensureParentDir(manifestP);
      std::string err;
      if (!WriteRunManifestJson(manifestP,
                                runIdx,
                                requestedSeed,
                                actualSeed,
                                world.width(),
                                world.height(),
                                days,
                                hash,
                                loadPath,
                                argvList,
                                runProcCfg,
                                sim.config(),
                                artifacts,
                                err)) {
        std::cerr << "Failed to write manifest" << (manifestP.empty() ? "" : (": " + manifestP))
                  << (err.empty() ? "" : (" (" + err + ")")) << "\n";
        return 1;
      }
    }

    return 0;
  };

  for (int runIdx = 0; runIdx < batchRuns; ++runIdx) {
    const std::uint64_t runSeed = seed + static_cast<std::uint64_t>(runIdx);
    const int rc = runOne(runIdx, runSeed);
    if (rc != 0) return rc;
  }

  return 0;
}
