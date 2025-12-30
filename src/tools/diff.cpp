#include "isocity/Export.hpp"
#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/WorldDiff.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
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

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_diff (headless save/world comparison tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_diff [options] <a.bin> <b.bin>\n\n"
      << "Options:\n"
      << "  --verify-crc           Verify v3+ save CRC32 in ReadSaveSummary (fast header check)\n"
      << "  --height-eps <eps>     Float tolerance for Tile::height comparisons (default: 1e-6)\n"
      << "  --ppm <out.ppm>        Write a color-coded diff map (1px per tile, PPM P6)\n"
      << "  --scale <N>            Nearest-neighbor upscale factor for --ppm output (default: 1)\n"
      << "  -h, --help             Show help\n\n"
      << "Diff map legend (RGB channels):\n"
      << "  R: overlay/level/occupants differ\n"
      << "  G: terrain/height differ\n"
      << "  B: district/variation differ\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  bool verifyCrc = false;
  float heightEps = 1e-6f;
  std::string outPpm;
  int scale = 1;

  std::vector<std::string> positional;

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
    }
    if (arg == "--verify-crc") {
      verifyCrc = true;
      continue;
    }
    if (arg == "--height-eps") {
      if (!requireValue(i, val)) {
        std::cerr << "--height-eps requires a number\n";
        return 2;
      }
      float v = 0.0f;
      if (!ParseF32(val, &v) || v < 0.0f) {
        std::cerr << "--height-eps requires a non-negative number\n";
        return 2;
      }
      heightEps = v;
      continue;
    }
    if (arg == "--ppm") {
      if (!requireValue(i, val)) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
      outPpm = val;
      continue;
    }
    if (arg == "--scale") {
      if (!requireValue(i, val)) {
        std::cerr << "--scale requires an integer\n";
        return 2;
      }
      int n = 1;
      if (!ParseI32(val, &n) || n < 1) {
        std::cerr << "--scale requires an integer >= 1\n";
        return 2;
      }
      scale = n;
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n\n";
      PrintHelp();
      return 2;
    }

    positional.push_back(arg);
  }

  if (positional.size() != 2) {
    PrintHelp();
    return 2;
  }

  const std::string pathA = positional[0];
  const std::string pathB = positional[1];

  if (!std::filesystem::exists(pathA)) {
    std::cerr << "File not found: " << pathA << "\n";
    return 2;
  }
  if (!std::filesystem::exists(pathB)) {
    std::cerr << "File not found: " << pathB << "\n";
    return 2;
  }

  // Fast header summary (optional CRC verification).
  SaveSummary sumA{};
  SaveSummary sumB{};
  {
    std::string err;
    if (!ReadSaveSummary(pathA, sumA, err, verifyCrc)) {
      std::cerr << "Warning: ReadSaveSummary failed for A: " << err << "\n";
    }
  }
  {
    std::string err;
    if (!ReadSaveSummary(pathB, sumB, err, verifyCrc)) {
      std::cerr << "Warning: ReadSaveSummary failed for B: " << err << "\n";
    }
  }

  // Full load so we can compute hashes and deep diffs.
  World wa;
  World wb;
  ProcGenConfig procA{};
  ProcGenConfig procB{};
  SimConfig simA{};
  SimConfig simB{};

  {
    std::string err;
    if (!LoadWorldBinary(wa, procA, simA, pathA, err)) {
      std::cerr << "Failed to load A: " << err << "\n";
      return 1;
    }
  }
  {
    std::string err;
    if (!LoadWorldBinary(wb, procB, simB, pathB, err)) {
      std::cerr << "Failed to load B: " << err << "\n";
      return 1;
    }
  }

  const std::uint64_t hashA = HashWorld(wa, true);
  const std::uint64_t hashB = HashWorld(wb, true);

  const WorldDiffStats d = DiffWorldTiles(wa, wb, heightEps);

  auto printOne = [&](const char* label, const std::string& path, const SaveSummary& s, const World& w, std::uint64_t h) {
    std::cout << label << ": " << path << "\n";
    if (s.version != 0) {
      std::cout << "  saveVersion: v" << s.version;
      if (s.crcChecked) {
        std::cout << "  crc: " << (s.crcOk ? "OK" : "BAD");
      }
      std::cout << "\n";
    }
    std::cout << "  size: " << w.width() << "x" << w.height() << "\n";
    std::cout << "  seed: " << w.seed() << "\n";
    std::cout << "  worldHash: " << HexU64(h) << "\n";
    const Stats& st = w.stats();
    std::cout << "  day=" << st.day << " pop=" << st.population << " money=" << st.money << " roads=" << st.roads << " parks=" << st.parks
              << " happiness=" << std::fixed << std::setprecision(3) << st.happiness << "\n";
  };

  printOne("A", pathA, sumA, wa, hashA);
  printOne("B", pathB, sumB, wb, hashB);

  std::cout << "\n";
  std::cout << "Tile diff (overlapping region):\n";
  std::cout << "  tilesCompared:  " << d.tilesCompared << "\n";
  std::cout << "  tilesDifferent: " << d.tilesDifferent << "\n";
  if (d.sizeMismatch) {
    std::cout << "  sizeMismatch:   true (A=" << d.widthA << "x" << d.heightA << ", B=" << d.widthB << "x" << d.heightB << ")\n";
  }
  std::cout << "  terrainDifferent:   " << d.terrainDifferent << "\n";
  std::cout << "  overlayDifferent:   " << d.overlayDifferent << "\n";
  std::cout << "  heightDifferent:    " << d.heightDifferent << " (eps=" << std::scientific << heightEps << std::defaultfloat << ")\n";
  std::cout << "  variationDifferent: " << d.variationDifferent << "\n";
  std::cout << "  levelDifferent:     " << d.levelDifferent << "\n";
  std::cout << "  occupantsDifferent: " << d.occupantsDifferent << "\n";
  std::cout << "  districtDifferent:  " << d.districtDifferent << "\n";

  if (!outPpm.empty()) {
    const int w = std::min(wa.width(), wb.width());
    const int h = std::min(wa.height(), wb.height());

    PpmImage img{};
    img.width = w;
    img.height = h;
    img.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u, 0u);

    auto setPix = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
      img.rgb[idx + 0] = r;
      img.rgb[idx + 1] = g;
      img.rgb[idx + 2] = b;
    };

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const Tile& ta = wa.at(x, y);
        const Tile& tb = wb.at(x, y);

        const bool r = (ta.overlay != tb.overlay) || (ta.level != tb.level) || (ta.occupants != tb.occupants);
        const bool g = (ta.terrain != tb.terrain) || (std::fabs(ta.height - tb.height) > std::max(0.0f, heightEps));
        const bool b = (ta.district != tb.district) || (ta.variation != tb.variation);

        setPix(x, y, r ? 255 : 0, g ? 255 : 0, b ? 255 : 0);
      }
    }

    if (scale > 1) {
      img = ScaleNearest(img, scale);
    }

    std::string err;
    if (!WritePpm(outPpm, img, err)) {
      std::cerr << "Failed to write PPM: " << outPpm << " (" << err << ")\n";
      return 1;
    }
    std::cout << "\nWrote diff PPM: " << outPpm << "\n";
  }

  // Non-zero exit code is useful for CI/regression scripts.
  if (hashA != hashB) {
    return 3;
  }

  return 0;
}
