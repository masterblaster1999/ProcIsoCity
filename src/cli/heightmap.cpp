#include "isocity/Export.hpp"
#include "isocity/Heightmap.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_heightmap (headless heightmap import/export)\n\n"
      << "Apply a grayscale heightmap (PPM/PNG) to a world/save, optionally reclassifying terrain.\n\n"
      << "Usage:\n"
      << "  proc_isocity_heightmap [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                         [--import <img.ppm|img.png>] [--export <out.ppm|out.png>]\n"
      << "                         [--save <out_save.bin>] [--json <out.json>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>           Load a save file (overrides --seed/--size).\n"
      << "  --seed <u64>                Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>                World size (default: 96x96).\n"
      << "  --import <ppm|png>          Input heightmap image (RGB treated as grayscale luma).\n\n"
      << "Import mapping:\n"
      << "  --resample <mode>           Size mismatch handling: none|nearest|bilinear (default: none).\n"
      << "  --flip-x <0|1>              Flip the input image horizontally before sampling.\n"
      << "  --flip-y <0|1>              Flip the input image vertically before sampling.\n"
      << "  --invert <0|1>              Invert grayscale (1-gray) before applying.\n"
      << "  --height-scale <F>          height = gray*scale + offset (default: 1).\n"
      << "  --height-offset <F>         height = gray*scale + offset (default: 0).\n"
      << "  --clamp01 <0|1>             Clamp resulting height to [0,1] (default: 1).\n\n"
      << "Terrain reclassification (optional):\n"
      << "  --reclassify <0|1>          Set Terrain from height (default: 1).\n"
      << "  --water-level <F>           height < water => Water (default: 0.35).\n"
      << "  --sand-level <F>            height < sand  => Sand  (default: 0.42).\n"
      << "  --bulldoze-water <0|1>      If a tile becomes water, clear non-road overlays (default: 1).\n\n"
      << "Outputs:\n"
      << "  --export <ppm|png>          Export the world's current heights as a grayscale image.\n"
      << "  --export-normalize <0|1>    Normalize heights using world min/max (default: 0).\n"
      << "  --export-invert <0|1>       Invert exported grayscale (default: 0).\n"
      << "  --export-scale <N>          Nearest-neighbor upscale for exported image (default: 1).\n"
      << "  --save <out_save.bin>       Write the modified world to a save.\n"
      << "  --json <out.json>           Write a JSON stats report.\n\n"
      << "Examples:\n"
      << "  # Export heightmap from a save\n"
      << "  proc_isocity_heightmap --load city.bin --export height.png\n\n"
      << "  # Apply a heightmap to a generated world and save it\n"
      << "  proc_isocity_heightmap --seed 1 --size 128x128 --import hm.png --resample bilinear \\\n"
      << "    --water-level 0.30 --sand-level 0.38 --save out.bin --export preview.png\n";
}

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0) return false;
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
  errno = 0;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const float v = std::strtof(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

bool ParseSize(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t x = s.find('x');
  if (x == std::string::npos) return false;
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, x), &w)) return false;
  if (!ParseI32(s.substr(x + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  if (s == "0") {
    *out = false;
    return true;
  }
  if (s == "1") {
    *out = true;
    return true;
  }
  return false;
}

HeightmapResample ParseResampleMode(const std::string& s, bool* ok)
{
  if (ok) *ok = true;
  std::string v = s;
  for (char& c : v)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  if (v == "none") return HeightmapResample::None;
  if (v == "nearest") return HeightmapResample::Nearest;
  if (v == "bilinear") return HeightmapResample::Bilinear;

  if (ok) *ok = false;
  return HeightmapResample::None;
}

bool EnsureParentDir(const std::string& path)
{
  try {
    fs::path p(path);
    if (!p.has_parent_path()) return true;
    fs::path dir = p.parent_path();
    if (dir.empty()) return true;
    if (fs::exists(dir)) return true;
    return fs::create_directories(dir);
  } catch (...) {
    return false;
  }
}

struct Options {
  // World input.
  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  // Heightmap import/export.
  std::string inHeightmap;
  std::string outHeightmap;

  // Save output.
  std::string outSave;

  // Optional JSON report.
  std::string outJson;

  HeightmapApplyConfig applyCfg;
  HeightmapExportConfig exportCfg;
  int exportScale = 1;
};

void WriteStatsJson(std::ostream& os, const HeightmapApplyStats& st)
{
  os << "{\n";
  os << "  \"world\": {\"w\": " << st.worldW << ", \"h\": " << st.worldH << "},\n";
  os << "  \"sourceImage\": {\"w\": " << st.srcW << ", \"h\": " << st.srcH << "},\n";
  os << "  \"height\": {\n";
  os << "    \"min\": " << std::fixed << std::setprecision(6) << st.minHeight << ",\n";
  os << "    \"max\": " << std::fixed << std::setprecision(6) << st.maxHeight << ",\n";
  os << "    \"mean\": " << std::fixed << std::setprecision(6) << st.meanHeight << ",\n";
  os << "    \"stdev\": " << std::fixed << std::setprecision(6) << st.stdevHeight << "\n";
  os << "  },\n";
  os << "  \"terrain\": {\n";
  os << "    \"waterTiles\": " << st.waterTiles << ",\n";
  os << "    \"sandTiles\": " << st.sandTiles << ",\n";
  os << "    \"grassTiles\": " << st.grassTiles << "\n";
  os << "  },\n";
  os << "  \"overlaysCleared\": " << st.overlaysCleared << "\n";
  os << "}\n";
}

int Run(int argc, char** argv)
{
  Options opt;

  // Defaults match ProcGenConfig defaults.
  opt.applyCfg.waterLevel = 0.35f;
  opt.applyCfg.sandLevel = 0.42f;

  if (argc <= 1) {
    PrintHelp();
    return 0;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];

    auto need = [&](int n) -> bool { return (i + n) < argc; };

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--load") {
      if (!need(1)) {
        std::cerr << "--load requires a value\n";
        return 2;
      }
      opt.loadPath = argv[++i];
    } else if (a == "--seed") {
      if (!need(1)) {
        std::cerr << "--seed requires a value\n";
        return 2;
      }
      if (!ParseU64(argv[++i], &opt.seed)) {
        std::cerr << "Invalid --seed\n";
        return 2;
      }
    } else if (a == "--size") {
      if (!need(1)) {
        std::cerr << "--size requires a value\n";
        return 2;
      }
      if (!ParseSize(argv[++i], &opt.w, &opt.h)) {
        std::cerr << "Invalid --size (expected WxH)\n";
        return 2;
      }
    } else if (a == "--import") {
      if (!need(1)) {
        std::cerr << "--import requires a value\n";
        return 2;
      }
      opt.inHeightmap = argv[++i];
    } else if (a == "--export") {
      if (!need(1)) {
        std::cerr << "--export requires a value\n";
        return 2;
      }
      opt.outHeightmap = argv[++i];
    } else if (a == "--save") {
      if (!need(1)) {
        std::cerr << "--save requires a value\n";
        return 2;
      }
      opt.outSave = argv[++i];
    } else if (a == "--json") {
      if (!need(1)) {
        std::cerr << "--json requires a value\n";
        return 2;
      }
      opt.outJson = argv[++i];
    } else if (a == "--resample") {
      if (!need(1)) {
        std::cerr << "--resample requires a value\n";
        return 2;
      }
      bool ok = false;
      opt.applyCfg.resample = ParseResampleMode(argv[++i], &ok);
      if (!ok) {
        std::cerr << "Invalid --resample (expected none|nearest|bilinear)\n";
        return 2;
      }
    } else if (a == "--flip-x") {
      if (!need(1)) {
        std::cerr << "--flip-x requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyCfg.flipX)) {
        std::cerr << "Invalid --flip-x (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--flip-y") {
      if (!need(1)) {
        std::cerr << "--flip-y requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyCfg.flipY)) {
        std::cerr << "Invalid --flip-y (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--invert") {
      if (!need(1)) {
        std::cerr << "--invert requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyCfg.invert)) {
        std::cerr << "Invalid --invert (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--height-scale") {
      if (!need(1)) {
        std::cerr << "--height-scale requires a value\n";
        return 2;
      }
      if (!ParseF32(argv[++i], &opt.applyCfg.heightScale)) {
        std::cerr << "Invalid --height-scale\n";
        return 2;
      }
    } else if (a == "--height-offset") {
      if (!need(1)) {
        std::cerr << "--height-offset requires a value\n";
        return 2;
      }
      if (!ParseF32(argv[++i], &opt.applyCfg.heightOffset)) {
        std::cerr << "Invalid --height-offset\n";
        return 2;
      }
    } else if (a == "--clamp01") {
      if (!need(1)) {
        std::cerr << "--clamp01 requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyCfg.clamp01)) {
        std::cerr << "Invalid --clamp01 (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--reclassify") {
      if (!need(1)) {
        std::cerr << "--reclassify requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyCfg.reclassifyTerrain)) {
        std::cerr << "Invalid --reclassify (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--water-level") {
      if (!need(1)) {
        std::cerr << "--water-level requires a value\n";
        return 2;
      }
      if (!ParseF32(argv[++i], &opt.applyCfg.waterLevel)) {
        std::cerr << "Invalid --water-level\n";
        return 2;
      }
    } else if (a == "--sand-level") {
      if (!need(1)) {
        std::cerr << "--sand-level requires a value\n";
        return 2;
      }
      if (!ParseF32(argv[++i], &opt.applyCfg.sandLevel)) {
        std::cerr << "Invalid --sand-level\n";
        return 2;
      }
    } else if (a == "--bulldoze-water") {
      if (!need(1)) {
        std::cerr << "--bulldoze-water requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.applyCfg.bulldozeNonRoadOverlaysOnWater)) {
        std::cerr << "Invalid --bulldoze-water (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--export-normalize") {
      if (!need(1)) {
        std::cerr << "--export-normalize requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.exportCfg.normalize)) {
        std::cerr << "Invalid --export-normalize (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--export-invert") {
      if (!need(1)) {
        std::cerr << "--export-invert requires a value 0|1\n";
        return 2;
      }
      if (!ParseBool01(argv[++i], &opt.exportCfg.invert)) {
        std::cerr << "Invalid --export-invert (expected 0|1)\n";
        return 2;
      }
    } else if (a == "--export-scale") {
      if (!need(1)) {
        std::cerr << "--export-scale requires a value\n";
        return 2;
      }
      if (!ParseI32(argv[++i], &opt.exportScale) || opt.exportScale < 1) {
        std::cerr << "Invalid --export-scale\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      return 2;
    }
  }

  // Load or generate the world.
  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::string err;

  if (!opt.loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, opt.loadPath, err)) {
      std::cerr << "Failed to load save: " << opt.loadPath << "\n" << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(opt.w, opt.h, opt.seed, procCfg);
  }

  // Apply heightmap if requested.
  HeightmapApplyStats hmStats{};
  bool hasHmStats = false;
  if (!opt.inHeightmap.empty()) {
    PpmImage img;
    if (!ReadImageAuto(opt.inHeightmap, img, err)) {
      std::cerr << "Failed to read image: " << opt.inHeightmap << "\n" << err << "\n";
      return 2;
    }

    if (!ApplyHeightmap(world, img, opt.applyCfg, err, &hmStats)) {
      std::cerr << "Failed to apply heightmap: " << opt.inHeightmap << "\n" << err << "\n";
      return 2;
    }
    hasHmStats = true;

    // Keep ProcGenConfig in sync with terrain thresholds if we reclassified terrain.
    if (opt.applyCfg.reclassifyTerrain) {
      procCfg.waterLevel = opt.applyCfg.waterLevel;
      procCfg.sandLevel = opt.applyCfg.sandLevel;
    }
  }

  // Export heightmap if requested.
  if (!opt.outHeightmap.empty()) {
    float rawMin = 0.0f;
    float rawMax = 0.0f;
    PpmImage out = ExportHeightmapImage(world, opt.exportCfg, &rawMin, &rawMax);
    if (opt.exportScale > 1) out = ScaleNearest(out, opt.exportScale);

    if (!EnsureParentDir(opt.outHeightmap)) {
      std::cerr << "Failed to create output directory for: " << opt.outHeightmap << "\n";
      return 2;
    }
    if (!WriteImageAuto(opt.outHeightmap, out, err)) {
      std::cerr << "Failed to write image: " << opt.outHeightmap << "\n" << err << "\n";
      return 2;
    }

    if (hasHmStats) {
      // If we imported, update the src image dims in the report but still report raw min/max via stdout.
      (void)rawMin;
      (void)rawMax;
    }
  }

  // Write save if requested.
  if (!opt.outSave.empty()) {
    if (!EnsureParentDir(opt.outSave)) {
      std::cerr << "Failed to create output directory for: " << opt.outSave << "\n";
      return 2;
    }
    if (!SaveWorldBinary(world, procCfg, simCfg, opt.outSave, err)) {
      std::cerr << "Failed to save: " << opt.outSave << "\n" << err << "\n";
      return 2;
    }
  }

  // Optional JSON report.
  if (!opt.outJson.empty()) {
    if (!EnsureParentDir(opt.outJson)) {
      std::cerr << "Failed to create output directory for: " << opt.outJson << "\n";
      return 2;
    }
    std::ofstream os(opt.outJson, std::ios::binary);
    if (!os) {
      std::cerr << "Failed to open: " << opt.outJson << "\n";
      return 2;
    }

    if (hasHmStats) {
      WriteStatsJson(os, hmStats);
    } else {
      // If no import happened, emit a minimal report based on current world heights.
      HeightmapApplyStats st{};
      st.worldW = world.width();
      st.worldH = world.height();
      st.srcW = 0;
      st.srcH = 0;

      float minH = std::numeric_limits<float>::infinity();
      float maxH = -std::numeric_limits<float>::infinity();
      double sum = 0.0;
      double sumSq = 0.0;
      std::uint64_t wC = 0, sC = 0, gC = 0;
      for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
          const Tile& t = world.at(x, y);
          minH = std::min(minH, t.height);
          maxH = std::max(maxH, t.height);
          sum += t.height;
          sumSq += static_cast<double>(t.height) * static_cast<double>(t.height);
          if (t.terrain == Terrain::Water) wC++;
          if (t.terrain == Terrain::Sand) sC++;
          if (t.terrain == Terrain::Grass) gC++;
        }
      }

      const double n = static_cast<double>(world.width()) * static_cast<double>(world.height());
      const double mean = (n > 0.0) ? (sum / n) : 0.0;
      const double var = (n > 0.0) ? std::max(0.0, (sumSq / n) - mean * mean) : 0.0;

      st.minHeight = std::isfinite(minH) ? minH : 0.0f;
      st.maxHeight = std::isfinite(maxH) ? maxH : 0.0f;
      st.meanHeight = mean;
      st.stdevHeight = std::sqrt(var);

      st.waterTiles = wC;
      st.sandTiles = sC;
      st.grassTiles = gC;

      WriteStatsJson(os, st);
    }
  }

  // If nothing was requested, print help so the command isn't a silent noop.
  if (opt.inHeightmap.empty() && opt.outHeightmap.empty() && opt.outSave.empty() && opt.outJson.empty()) {
    PrintHelp();
    return 0;
  }

  return 0;
}

} // namespace

int main(int argc, char** argv) { return Run(argc, argv); }
