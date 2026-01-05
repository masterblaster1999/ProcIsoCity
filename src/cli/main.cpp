#include "isocity/Hash.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Export.hpp"
#include "isocity/GfxTilesetAtlas.hpp"
#include "isocity/Pathfinding.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

void PrintHelp()
{
  std::cout
      << "proc_isocity_cli (headless simulation runner)\n\n"
      << "Usage:\n"
      << "  proc_isocity_cli [--load <save.bin>] [--seed <u64>] [--size <WxH>] [--days <N>]\n"
      << "                 [--out <summary.json>] [--csv <ticks.csv>] [--save <save.bin>]\n"
      << "                 [--require-outside <0|1>] [--tax-res <N>] [--tax-com <N>] [--tax-ind <N>]\n"
      << "                 [--maint-road <N>] [--maint-park <N>]\n"
      << "                 [--export-ppm <layer> <out.ppm|out.png>]... [--export-scale <N>]\n"
      << "                 [--export-iso <layer> <out.ppm|out.png>]... [--iso-tile <WxH>] [--iso-height <N>]\n"
      << "                 [--export-3d <layer> <out.ppm|out.png>]... [--3d-size <WxH>] [--3d-proj <iso|persp>]\n"
      << "                 [--3d-yaw <deg>] [--3d-pitch <deg>] [--3d-roll <deg>] [--3d-fit <0|1>] [--3d-ssaa <N>]\n"
      << "                 [--3d-target <x,y,z>] [--3d-dist <N>] [--3d-fov <deg>] [--3d-ortho <N>]\n"
      << "                 [--3d-outline <0|1>] [--3d-top <0|1>]\n"
      << "                 [--3d-light <x,y,z>] [--3d-ambient <0..100>] [--3d-diffuse <0..100>] [--3d-bg <r,g,b>]\n"
      << "                 [--3d-fog <0|1>] [--3d-fog-strength <0..100>] [--3d-fog-start <0..100>] [--3d-fog-end <0..100>]\n"
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
      << "  terrain overlay height landvalue traffic goods_traffic goods_fill district\n\n"
      << "Batch mode:\n"
      << "  - --batch N>1 runs N simulations with seeds (seed, seed+1, ...).\n"
      << "  - To write per-run files, include {seed} or {run} in any output path.\n"
      << "    Example: --out out_{seed}.json  --export-ppm overlay map_{seed}.png\n\n"
      << "Notes:\n"
      << "  - If --load is provided, the world + ProcGenConfig + SimConfig are loaded from the save.\n"
      << "  - Otherwise, a new world is generated from (--seed, --size).\n"
      << "  - --days advances the simulator by N ticks via Simulator::stepOnce().\n"
      << "  - A stable 64-bit hash of the final world is included in the JSON output.\n";
}

bool WriteJsonSummary(const isocity::World& world, std::uint64_t hash, const std::string& outPath)
{
  const isocity::Stats& s = world.stats();

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"width\": " << world.width() << ",\n";
  oss << "  \"height\": " << world.height() << ",\n";
  oss << "  \"seed\": " << world.seed() << ",\n";
  oss << "  \"hash\": \"" << HexU64(hash) << "\",\n";
  oss << "  \"stats\": {\n";
  oss << "    \"day\": " << s.day << ",\n";
  oss << "    \"population\": " << s.population << ",\n";
  oss << "    \"housingCapacity\": " << s.housingCapacity << ",\n";
  oss << "    \"jobsCapacity\": " << s.jobsCapacity << ",\n";
  oss << "    \"jobsCapacityAccessible\": " << s.jobsCapacityAccessible << ",\n";
  oss << "    \"employed\": " << s.employed << ",\n";
  oss << "    \"happiness\": " << s.happiness << ",\n";
  oss << "    \"money\": " << s.money << ",\n";
  oss << "    \"roads\": " << s.roads << ",\n";
  oss << "    \"parks\": " << s.parks << ",\n";
  oss << "    \"avgCommuteTime\": " << s.avgCommuteTime << ",\n";
  oss << "    \"trafficCongestion\": " << s.trafficCongestion << ",\n";
  oss << "    \"goodsDemand\": " << s.goodsDemand << ",\n";
  oss << "    \"goodsDelivered\": " << s.goodsDelivered << ",\n";
  oss << "    \"goodsSatisfaction\": " << s.goodsSatisfaction << ",\n";
  oss << "    \"avgLandValue\": " << s.avgLandValue << ",\n";
  oss << "    \"demandResidential\": " << s.demandResidential << "\n";
  oss << "  }\n";
  oss << "}\n";

  if (outPath.empty()) {
    std::cout << oss.str();
    return true;
  }

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
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

  std::string loadPath;
  std::string savePath;
  std::string outJson;
  std::string outCsv;

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

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string val;

    if (arg == "--help" || arg == "-h") {
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
    } else if (arg == "--tax-res") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxResidential)) {
        std::cerr << "--tax-res requires an integer\n";
        return 2;
      }
    } else if (arg == "--tax-com") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxCommercial)) {
        std::cerr << "--tax-com requires an integer\n";
        return 2;
      }
    } else if (arg == "--tax-ind") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxIndustrial)) {
        std::cerr << "--tax-ind requires an integer\n";
        return 2;
      }
    } else if (arg == "--maint-road") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenanceRoad)) {
        std::cerr << "--maint-road requires an integer\n";
        return 2;
      }
    } else if (arg == "--maint-park") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenancePark)) {
        std::cerr << "--maint-park requires an integer\n";
        return 2;
      }
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
        render3dCfg.projection = Render3DConfig::Projection::Isometric;
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

    if (!loadPath.empty()) {
      std::string err;
      if (!LoadWorldBinary(world, runProcCfg, runSimCfg, loadPath, err)) {
        std::cerr << "Failed to load save: " << err << "\n";
        return 1;
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
    }

    // Optional derived-map exports (images)
    if (!ppmExports.empty() || !isoExports.empty() || !render3dExports.empty()) {
      bool needTraffic = false;
      bool needGoods = false;
      bool needLandValue = false;

      for (const auto& e : ppmExports) {
        if (e.layer == ExportLayer::Traffic) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }
      for (const auto& e : isoExports) {
        if (e.layer == ExportLayer::Traffic) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill) needGoods = true;
        if (e.layer == ExportLayer::LandValue) needLandValue = true;
      }
      for (const auto& e : render3dExports) {
        if (e.layer == ExportLayer::Traffic) needTraffic = true;
        if (e.layer == ExportLayer::GoodsTraffic || e.layer == ExportLayer::GoodsFill) needGoods = true;
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
      }
    }

    const std::uint64_t hash = HashWorld(world, true);
    const std::string jsonP = expandPath(outJson, runIdx, actualSeed);
    if (!WriteJsonSummary(world, hash, jsonP)) {
      std::cerr << "Failed to write JSON summary" << (jsonP.empty() ? "" : (": " + jsonP)) << "\n";
      return 1;
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
