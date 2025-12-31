#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/WorldPatch.hpp"

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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
      << "  proc_isocity_patch make    <base.bin> <target.bin> <out.patch> [options]\n"
      << "  proc_isocity_patch apply   <base.bin> <patch>      <out.bin>   [options]\n"
      << "  proc_isocity_patch invert  <base.bin> <patch>      <out.patch> [options]\n"
      << "  proc_isocity_patch compose <base.bin> <patchA> <patchB> ... <out.patch> [options]\n"
      << "  proc_isocity_patch info    <patch>\n\n"
      << "Make/Compose options:\n"
      << "  --no-proc         Do not embed ProcGenConfig in the output patch.\n"
      << "  --no-sim          Do not embed SimConfig in the output patch.\n"
      << "  --no-stats        Patch only tile grid (hash excludes Stats for strictness).\n"
      << "  --no-compress     Store the patch payload uncompressed.\n"
      << "  --force           (compose only) apply input patches even if base hash mismatches.\n"
      << "  --quiet           Suppress stdout summary (errors still print).\n\n"
      << "Apply options:\n"
      << "  --force           Apply even if the base hash does not match the patch's base hash.\n"
      << "  --quiet           Suppress stdout summary (errors still print).\n\n"
      << "Invert options:\n"
      << "  --force           Generate even if the provided base save's hash doesn't match the patch base.\n"
      << "  --no-compress     Store the output patch payload uncompressed.\n"
      << "  --quiet           Suppress stdout summary (errors still print).\n\n";
}

bool IsOption(const std::string& a)
{
  return !a.empty() && a[0] == '-';
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
  const auto isHelp = [&]() { return mode == "-h" || mode == "--help" || mode == "help"; };
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

  if (mode == "invert") {
    if (argc < 5) {
      PrintHelp();
      return 1;
    }

    const std::string basePath = argv[2];
    const std::string patchPath = argv[3];
    const std::string outPatchPath = argv[4];

    bool force = false;
    bool compress = true;
    bool quiet = false;

    for (int i = 5; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--force") {
        force = true;
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
    ProcGenConfig baseProc;
    SimConfig baseSim;
    std::string err;

    if (!LoadWorldBinary(baseW, baseProc, baseSim, basePath, err)) {
      std::cerr << "Load base failed: " << err << "\n";
      return 2;
    }

    WorldPatch fwd;
    if (!LoadWorldPatchBinary(fwd, patchPath, err)) {
      std::cerr << "Load patch failed: " << err << "\n";
      return 2;
    }

    WorldPatch inv;
    if (!InvertWorldPatch(baseW, baseProc, baseSim, fwd, inv, err, force)) {
      std::cerr << "Invert failed: " << err << "\n";
      return 3;
    }

    const WorldPatchCompression comp = compress ? WorldPatchCompression::SLLZ : WorldPatchCompression::None;
    if (!SaveWorldPatchBinary(inv, outPatchPath, err, comp)) {
      std::cerr << "Save inverted patch failed: " << err << "\n";
      return 3;
    }

    if (!quiet) {
      std::cout << "Inverse patch written: " << outPatchPath << "\n";
      std::cout << "  size: " << inv.width << "x" << inv.height << "\n";
      std::cout << "  tiles changed: " << inv.tiles.size() << "\n";
      std::cout << "  includeStats: " << (inv.includeStats ? 1 : 0) << "\n";
      std::cout << "  includeProcCfg: " << (inv.includeProcCfg ? 1 : 0) << "\n";
      std::cout << "  includeSimCfg: " << (inv.includeSimCfg ? 1 : 0) << "\n";
      std::cout << "  baseHash:   " << HexU64(inv.baseHash) << "\n";
      std::cout << "  targetHash: " << HexU64(inv.targetHash) << "\n";
    }

    return 0;
  }

  if (mode == "compose") {
    if (argc < 6) {
      PrintHelp();
      return 1;
    }

    const std::string basePath = argv[2];

    // Parse file args until options begin. The last file argument is the output path.
    std::vector<std::string> files;
    int optStart = -1;
    for (int i = 3; i < argc; ++i) {
      const std::string a = argv[i];
      if (IsOption(a)) {
        optStart = i;
        break;
      }
      files.push_back(a);
    }
    if (optStart < 0) optStart = argc;

    if (files.size() < 3) {
      std::cerr << "compose expects: compose <base.bin> <patchA> <patchB> ... <out.patch> [options]\n";
      return 1;
    }

    const std::string outPatchPath = files.back();
    files.pop_back(); // remaining are input patch paths

    bool includeProc = true;
    bool includeSim = true;
    bool includeStats = true;
    bool compress = true;
    bool quiet = false;
    bool force = false;

    for (int i = optStart; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--no-proc") {
        includeProc = false;
      } else if (a == "--no-sim") {
        includeSim = false;
      } else if (a == "--no-stats") {
        includeStats = false;
      } else if (a == "--no-compress") {
        compress = false;
      } else if (a == "--force") {
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

    World baseW;
    ProcGenConfig baseProc;
    SimConfig baseSim;
    std::string err;

    if (!LoadWorldBinary(baseW, baseProc, baseSim, basePath, err)) {
      std::cerr << "Load base failed: " << err << "\n";
      return 2;
    }

    std::vector<WorldPatch> patches;
    patches.reserve(files.size());

    for (const std::string& p : files) {
      WorldPatch wp;
      if (!LoadWorldPatchBinary(wp, p, err)) {
        std::cerr << "Load patch failed (" << p << "): " << err << "\n";
        return 2;
      }
      patches.push_back(std::move(wp));
    }

    WorldPatch composed;
    if (!ComposeWorldPatches(baseW, baseProc, baseSim, patches, composed, err,
                             includeProc, includeSim, includeStats, force)) {
      std::cerr << "Compose failed: " << err << "\n";
      return 3;
    }

    const WorldPatchCompression comp = compress ? WorldPatchCompression::SLLZ : WorldPatchCompression::None;
    if (!SaveWorldPatchBinary(composed, outPatchPath, err, comp)) {
      std::cerr << "Save composed patch failed: " << err << "\n";
      return 3;
    }

    if (!quiet) {
      std::cout << "Composed patch written: " << outPatchPath << "\n";
      std::cout << "  input patches: " << patches.size() << "\n";
      std::cout << "  size: " << composed.width << "x" << composed.height << "\n";
      std::cout << "  tiles changed: " << composed.tiles.size() << "\n";
      std::cout << "  includeStats: " << (composed.includeStats ? 1 : 0) << "\n";
      std::cout << "  includeProcCfg: " << (composed.includeProcCfg ? 1 : 0) << "\n";
      std::cout << "  includeSimCfg: " << (composed.includeSimCfg ? 1 : 0) << "\n";
      std::cout << "  baseHash:   " << HexU64(composed.baseHash) << "\n";
      std::cout << "  targetHash: " << HexU64(composed.targetHash) << "\n";
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
