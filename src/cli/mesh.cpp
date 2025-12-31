#include "isocity/MeshExport.hpp"

#include "isocity/Hash.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

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

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const float v = std::strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  *out = v;
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

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  if (s == "0" || s == "false" || s == "False" || s == "FALSE") {
    *out = false;
    return true;
  }
  if (s == "1" || s == "true" || s == "True" || s == "TRUE") {
    *out = true;
    return true;
  }
  return false;
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
      << "proc_isocity_mesh (Wavefront OBJ/MTL exporter)\n\n"
      << "Usage:\n"
      << "  proc_isocity_mesh (--load <save.bin> | --seed <u64> --size <WxH>) [--days <N>]\n"
      << "                   --obj <out.obj> [--mtl <out.mtl>]\n"
      << "                   [--tile-size <F>] [--height-scale <F>] [--overlay-offset <F>]\n"
      << "                   [--cliffs <0|1>] [--cliff-threshold <F>]\n"
      << "                   [--buildings <0|1>]\n"
      << "                   [--crop <x> <y> <w> <h>] [--origin-at-crop <0|1>]\n\n"
      << "Exit codes:\n"
      << "  0  success\n"
      << "  1  runtime error (IO / load / export)\n"
      << "  2  bad arguments\n\n"
      << "Notes:\n"
      << "  - If --load is used, ProcGenConfig + SimConfig are loaded from the save and --days will\n"
      << "    advance the simulation deterministically before export.\n"
      << "  - If --seed/--size is used, a new world is generated via ProcGen and then optionally\n"
      << "    simulated for --days ticks (SimConfig defaults).\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  if (argc <= 1) {
    PrintHelp();
    return 2;
  }

  std::string loadPath;
  std::uint64_t seed = 1;
  bool seedProvided = false;
  int w = 96;
  int h = 96;
  int days = 0;

  std::string objPath;
  std::string mtlPath;

  MeshExportConfig meshCfg{};

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
    } else if (arg == "--obj") {
      if (!requireValue(i, val)) {
        std::cerr << "--obj requires a path\n";
        return 2;
      }
      objPath = val;
    } else if (arg == "--mtl") {
      if (!requireValue(i, val)) {
        std::cerr << "--mtl requires a path\n";
        return 2;
      }
      mtlPath = val;
    } else if (arg == "--tile-size") {
      if (!requireValue(i, val) || !ParseF32(val, &meshCfg.tileSize) || meshCfg.tileSize <= 0.0f) {
        std::cerr << "--tile-size requires a positive float\n";
        return 2;
      }
    } else if (arg == "--height-scale") {
      if (!requireValue(i, val) || !ParseF32(val, &meshCfg.heightScale)) {
        std::cerr << "--height-scale requires a float\n";
        return 2;
      }
    } else if (arg == "--overlay-offset") {
      if (!requireValue(i, val) || !ParseF32(val, &meshCfg.overlayOffset)) {
        std::cerr << "--overlay-offset requires a float\n";
        return 2;
      }
    } else if (arg == "--cliffs") {
      if (!requireValue(i, val) || !ParseBool01(val, &meshCfg.includeCliffs)) {
        std::cerr << "--cliffs requires 0|1\n";
        return 2;
      }
    } else if (arg == "--cliff-threshold") {
      if (!requireValue(i, val) || !ParseF32(val, &meshCfg.cliffThreshold)) {
        std::cerr << "--cliff-threshold requires a float\n";
        return 2;
      }
    } else if (arg == "--buildings") {
      if (!requireValue(i, val) || !ParseBool01(val, &meshCfg.includeBuildings)) {
        std::cerr << "--buildings requires 0|1\n";
        return 2;
      }
    } else if (arg == "--crop") {
      std::string sx, sy, sw, sh;
      if (!requireValue(i, sx) || !requireValue(i, sy) || !requireValue(i, sw) || !requireValue(i, sh)) {
        std::cerr << "--crop requires 4 integers: x y w h\n";
        return 2;
      }
      if (!ParseI32(sx, &meshCfg.cropX) || !ParseI32(sy, &meshCfg.cropY) || !ParseI32(sw, &meshCfg.cropW) ||
          !ParseI32(sh, &meshCfg.cropH)) {
        std::cerr << "--crop requires 4 integers: x y w h\n";
        return 2;
      }
      meshCfg.hasCrop = true;
    } else if (arg == "--origin-at-crop") {
      if (!requireValue(i, val) || !ParseBool01(val, &meshCfg.originAtCrop)) {
        std::cerr << "--origin-at-crop requires 0|1\n";
        return 2;
      }
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    } else {
      std::cerr << "unexpected positional argument: " << arg << "\n";
      return 2;
    }
  }

  if (objPath.empty()) {
    std::cerr << "--obj is required\n";
    return 2;
  }

  if (mtlPath.empty()) {
    // Derive a default: <obj>.mtl or replace extension.
    try {
      std::filesystem::path p(objPath);
      if (p.has_extension()) {
        p.replace_extension(".mtl");
      } else {
        p += ".mtl";
      }
      mtlPath = p.string();
    } catch (...) {
      mtlPath = objPath + ".mtl";
    }
  }

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::string err;

  if (!loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "LoadWorldBinary failed: " << err << "\n";
      return 1;
    }
  } else {
    if (!seedProvided) {
      std::cerr << "no --load provided; using default --seed=1\n";
    }
    world = GenerateWorld(w, h, seed, procCfg);
  }

  if (days > 0) {
    Simulator sim(simCfg);
    for (int i = 0; i < days; ++i) {
      sim.stepOnce(world);
    }
  }

  // Provide a nice object name by default.
  if (meshCfg.objectName.empty()) {
    meshCfg.objectName = "world_" + HexU64(HashWorld(world));
  }

  MeshExportStats st;
  if (!ExportWorldObjMtl(objPath, mtlPath, world, meshCfg, &st, &err)) {
    std::cerr << "ExportWorldObjMtl failed: " << err << "\n";
    return 1;
  }

  std::cout << "wrote: " << objPath << "\n";
  std::cout << "wrote: " << mtlPath << "\n";
  std::cout << "world: " << world.width() << "x" << world.height() << " seed=" << world.seed() << "\n";
  std::cout << "hash: " << HexU64(HashWorld(world)) << "\n";
  std::cout << "mesh: vertices=" << st.vertices << " triangles=" << st.triangles << "\n";
  return 0;
}
