#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/WorldTransform.hpp"

#include <cctype>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_transform (headless save transformer)\n\n"
      << "Usage:\n"
      << "  proc_isocity_transform <in_save.bin> <out_save.bin> [options]\n\n"
      << "Options:\n"
      << "  --rotate <0|90|180|270>  Clockwise rotation (degrees). Default: 0\n"
      << "  --mirror-x               Mirror horizontally after rotation.\n"
      << "  --mirror-y               Mirror vertically after rotation.\n"
      << "  --crop <x> <y> <w> <h>    Crop rectangle applied after rotate/mirror.\n"
      << "  --no-refresh-derived      Do not recompute derived HUD stats (population/capacities/etc).\n"
      << "  --quiet                  Suppress stdout output (errors still print).\n"
      << "  -h, --help               Show this help.\n";
}

static bool ParseI32(const std::string& s, int& out)
{
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  if (v < static_cast<long>(std::numeric_limits<int>::min()) || v > static_cast<long>(std::numeric_limits<int>::max())) {
    return false;
  }
  out = static_cast<int>(v);
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  std::string inPath;
  std::string outPath;

  WorldTransformConfig cfg;
  bool refreshDerived = true;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();

    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }

    if (arg == "--quiet") {
      quiet = true;
      continue;
    }

    if (arg == "--no-refresh-derived") {
      refreshDerived = false;
      continue;
    }

    if (arg == "--rotate") {
      if (i + 1 >= argc) {
        std::cerr << "--rotate requires a value\n";
        return 2;
      }
      int r = 0;
      if (!ParseI32(argv[++i], r)) {
        std::cerr << "--rotate expects an integer (0|90|180|270)\n";
        return 2;
      }
      cfg.rotateDeg = r;
      continue;
    }

    if (arg == "--mirror-x") {
      cfg.mirrorX = true;
      continue;
    }

    if (arg == "--mirror-y") {
      cfg.mirrorY = true;
      continue;
    }

    if (arg == "--crop") {
      if (i + 4 >= argc) {
        std::cerr << "--crop requires 4 integers: x y w h\n";
        return 2;
      }
      int x = 0;
      int y = 0;
      int w = 0;
      int h = 0;
      if (!ParseI32(argv[++i], x) || !ParseI32(argv[++i], y) || !ParseI32(argv[++i], w) || !ParseI32(argv[++i], h)) {
        std::cerr << "--crop expects 4 integers: x y w h\n";
        return 2;
      }
      cfg.hasCrop = true;
      cfg.cropX = x;
      cfg.cropY = y;
      cfg.cropW = w;
      cfg.cropH = h;
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    }

    // Positional args.
    if (inPath.empty()) {
      inPath = arg;
    } else if (outPath.empty()) {
      outPath = arg;
    } else {
      std::cerr << "unexpected positional arg: " << arg << "\n";
      return 2;
    }
  }

  if (inPath.empty() || outPath.empty()) {
    PrintHelp();
    return 2;
  }

  World inWorld;
  ProcGenConfig procCfg;
  SimConfig simCfg;
  std::string err;

  if (!LoadWorldBinary(inWorld, procCfg, simCfg, inPath, err)) {
    std::cerr << "failed to load save: " << err << "\n";
    return 1;
  }

  World outWorld;
  if (!TransformWorld(inWorld, outWorld, cfg, err)) {
    std::cerr << "transform failed: " << err << "\n";
    return 1;
  }

  if (refreshDerived) {
    Simulator sim(simCfg);
    sim.refreshDerivedStats(outWorld);
  }

  if (!SaveWorldBinary(outWorld, procCfg, simCfg, outPath, err)) {
    std::cerr << "failed to write save: " << err << "\n";
    return 1;
  }

  if (!quiet) {
    std::cout << "in:  " << inPath << " (" << inWorld.width() << "x" << inWorld.height() << ")\n";
    std::cout << "out: " << outPath << " (" << outWorld.width() << "x" << outWorld.height() << ")\n";
    std::cout << "rotate: " << cfg.rotateDeg << "\n";
    std::cout << "mirrorX: " << (cfg.mirrorX ? "true" : "false") << "\n";
    std::cout << "mirrorY: " << (cfg.mirrorY ? "true" : "false") << "\n";
    if (cfg.hasCrop) {
      std::cout << "crop: " << cfg.cropX << "," << cfg.cropY << " " << cfg.cropW << "x" << cfg.cropH << "\n";
    }
    std::cout << "day: " << outWorld.stats().day << "\n";
    std::cout << "money: " << outWorld.stats().money << "\n";
  }

  return 0;
}
