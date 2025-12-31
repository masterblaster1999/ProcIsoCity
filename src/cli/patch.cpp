#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/WorldPatch.hpp"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

using namespace isocity;

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_patch (binary save patch tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_patch make  <base.bin> <target.bin> <out.patch> [options]\n"
      << "  proc_isocity_patch apply <base.bin> <patch>      <out.bin>   [options]\n"
      << "  proc_isocity_patch info  <patch>\n\n"
      << "Make options:\n"
      << "  --no-proc         Do not embed ProcGenConfig in the patch (save header may differ).\n"
      << "  --no-sim          Do not embed SimConfig in the patch (save header may differ).\n"
      << "  --no-stats        Patch only tile grid (hash excludes Stats for strictness).\n"
      << "  --no-compress     Store the patch payload uncompressed.\n"
      << "  --quiet           Suppress stdout summary (errors still print).\n\n"
      << "Apply options:\n"
      << "  --force           Apply even if the base hash does not match the patch's base hash.\n"
      << "  --quiet           Suppress stdout summary (errors still print).\n\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  if (argc < 2) {
    PrintHelp();
    return 1;
  }

  const std::string mode = argv[1];
  const auto isHelp = [&]() {
    return mode == "-h" || mode == "--help" || mode == "help";
  };
  if (isHelp()) {
    PrintHelp();
    return 0;
  }

  if (mode == "make") {
    if (argc < 5) {
      PrintHelp();
      return 1;
    }

    const std::string basePath = argv[2];
    const std::string targetPath = argv[3];
    const std::string outPatchPath = argv[4];

    bool includeProc = true;
    bool includeSim = true;
    bool includeStats = true;
    bool compress = true;
    bool quiet = false;

    for (int i = 5; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--no-proc") {
        includeProc = false;
      } else if (a == "--no-sim") {
        includeSim = false;
      } else if (a == "--no-stats") {
        includeStats = false;
      } else if (a == "--no-compress") {
        compress = false;
      } else if (a == "--quiet") {
        quiet = true;
      } else if (a == "-h" || a == "--help") {
        PrintHelp();
        return 0;
      } else {
        std::cerr << "Unknown option: " << a << "\n";
        return 1;
      }
    }

    World baseW;
    World targetW;
    ProcGenConfig baseProc;
    ProcGenConfig targetProc;
    SimConfig baseSim;
    SimConfig targetSim;
    std::string err;

    if (!LoadWorldBinary(baseW, baseProc, baseSim, basePath, err)) {
      std::cerr << "Load base failed: " << err << "\n";
      return 2;
    }
    if (!LoadWorldBinary(targetW, targetProc, targetSim, targetPath, err)) {
      std::cerr << "Load target failed: " << err << "\n";
      return 2;
    }

    WorldPatch patch;
    if (!MakeWorldPatch(baseW, baseProc, baseSim, targetW, targetProc, targetSim, patch, err,
                        includeProc, includeSim, includeStats)) {
      std::cerr << "Make patch failed: " << err << "\n";
      return 3;
    }

    const WorldPatchCompression comp = compress ? WorldPatchCompression::SLLZ : WorldPatchCompression::None;
    if (!SaveWorldPatchBinary(patch, outPatchPath, err, comp)) {
      std::cerr << "Save patch failed: " << err << "\n";
      return 3;
    }

    if (!quiet) {
      std::cout << "Patch written: " << outPatchPath << "\n";
      std::cout << "  size: " << patch.width << "x" << patch.height << "\n";
      std::cout << "  tiles changed: " << patch.tiles.size() << "\n";
      std::cout << "  includeStats: " << (patch.includeStats ? 1 : 0) << "\n";
      std::cout << "  includeProcCfg: " << (patch.includeProcCfg ? 1 : 0) << "\n";
      std::cout << "  includeSimCfg: " << (patch.includeSimCfg ? 1 : 0) << "\n";
      std::cout << "  baseHash:   " << HexU64(patch.baseHash) << "\n";
      std::cout << "  targetHash: " << HexU64(patch.targetHash) << "\n";
    }

    return 0;
  }

  if (mode == "apply") {
    if (argc < 5) {
      PrintHelp();
      return 1;
    }

    const std::string basePath = argv[2];
    const std::string patchPath = argv[3];
    const std::string outPath = argv[4];

    bool force = false;
    bool quiet = false;
    for (int i = 5; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--force") {
        force = true;
      } else if (a == "--quiet") {
        quiet = true;
      } else if (a == "-h" || a == "--help") {
        PrintHelp();
        return 0;
      } else {
        std::cerr << "Unknown option: " << a << "\n";
        return 1;
      }
    }

    World w;
    ProcGenConfig proc;
    SimConfig sim;
    std::string err;
    if (!LoadWorldBinary(w, proc, sim, basePath, err)) {
      std::cerr << "Load base failed: " << err << "\n";
      return 2;
    }

    WorldPatch patch;
    if (!LoadWorldPatchBinary(patch, patchPath, err)) {
      std::cerr << "Load patch failed: " << err << "\n";
      return 2;
    }

    const std::uint64_t beforeHash = HashWorld(w, patch.includeStats);
    if (!ApplyWorldPatch(w, proc, sim, patch, err, force)) {
      std::cerr << "Apply patch failed: " << err << "\n";
      return 3;
    }
    const std::uint64_t afterHash = HashWorld(w, patch.includeStats);

    if (!SaveWorldBinary(w, proc, sim, outPath, err)) {
      std::cerr << "Save output failed: " << err << "\n";
      return 3;
    }

    if (!quiet) {
      std::cout << "Patched save written: " << outPath << "\n";
      std::cout << "  beforeHash: " << HexU64(beforeHash) << "\n";
      std::cout << "  afterHash:  " << HexU64(afterHash) << "\n";
      std::cout << "  expected:   " << HexU64(patch.targetHash) << "\n";
    }

    return 0;
  }

  if (mode == "info") {
    if (argc < 3) {
      PrintHelp();
      return 1;
    }
    const std::string patchPath = argv[2];
    WorldPatch patch;
    std::string err;
    if (!LoadWorldPatchBinary(patch, patchPath, err)) {
      std::cerr << "Load patch failed: " << err << "\n";
      return 2;
    }

    std::cout << "Patch: " << patchPath << "\n";
    std::cout << "  size: " << patch.width << "x" << patch.height << "\n";
    std::cout << "  tiles changed: " << patch.tiles.size() << "\n";
    std::cout << "  includeStats: " << (patch.includeStats ? 1 : 0) << "\n";
    std::cout << "  includeProcCfg: " << (patch.includeProcCfg ? 1 : 0) << "\n";
    std::cout << "  includeSimCfg: " << (patch.includeSimCfg ? 1 : 0) << "\n";
    std::cout << "  baseHash:   " << HexU64(patch.baseHash) << "\n";
    std::cout << "  targetHash: " << HexU64(patch.targetHash) << "\n";
    return 0;
  }

  std::cerr << "Unknown mode: " << mode << "\n\n";
  PrintHelp();
  return 1;
}
