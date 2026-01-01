#include "isocity/Export.hpp"
#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/WorldDiff.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static bool ParseI32(const std::string& s, int* out)
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

static bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const float v = std::strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

static std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static std::string JsonEscape(const std::string& s)
{
  std::ostringstream oss;
  for (unsigned char uc : s) {
    const char c = static_cast<char>(uc);
    switch (c) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        if (uc < 0x20) {
          // Control characters must be escaped.
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(uc) << std::dec;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
}

enum class DiffLayer : std::uint8_t {
  Any = 0,      // highlight if ANY tile field differs
  Combined = 1, // encode multiple difference kinds into RGB channels
  Terrain,
  Overlay,
  Height,
  Variation,
  Level,
  Occupants,
  District,
};

static bool ParseDiffLayer(const std::string& s, DiffLayer& out)
{
  const std::string k = ToLower(s);
  if (k == "any") {
    out = DiffLayer::Any;
    return true;
  }
  if (k == "combined" || k == "all") {
    out = DiffLayer::Combined;
    return true;
  }
  if (k == "terrain") {
    out = DiffLayer::Terrain;
    return true;
  }
  if (k == "overlay") {
    out = DiffLayer::Overlay;
    return true;
  }
  if (k == "height") {
    out = DiffLayer::Height;
    return true;
  }
  if (k == "variation" || k == "var") {
    out = DiffLayer::Variation;
    return true;
  }
  if (k == "level") {
    out = DiffLayer::Level;
    return true;
  }
  if (k == "occupants" || k == "occ") {
    out = DiffLayer::Occupants;
    return true;
  }
  if (k == "district" || k == "dist") {
    out = DiffLayer::District;
    return true;
  }
  return false;
}

struct Options {
  float heightEps = 1e-6f;
  int scale = 1;

  std::string outPpm;
  std::string outJson;
  std::string outTilesCsv;

  DiffLayer diffLayer = DiffLayer::Combined;
  bool baseEnabled = true;
  ExportLayer baseLayer = ExportLayer::Overlay;

  bool quiet = false;
};

static void PrintHelp()
{
  std::cout
      << "proc_isocity_diff (headless save diff tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_diff <A.bin> <B.bin> [options]\n\n"
      << "Options:\n"
      << "  --ppm <out.ppm|out.png>    Write a diff visualization (PPM/PNG).\n"
      << "  --scale <N>                Upscale the PPM by N (nearest-neighbor). Default: 1\n"
      << "  --height-eps <eps>          Float tolerance for Tile::height diffs. Default: 1e-6\n"
      << "  --layer <name>              What to highlight in the diff PPM. Default: combined\n"
      << "                              Names: any, combined, terrain, overlay, height, variation, level, occupants, district\n"
      << "  --base <layer|none>          Optional base render under the diff (uses world A when possible). Default: overlay\n"
      << "                              Base layers: terrain, overlay, height, district\n"
      << "  --json <out.json>            Write a JSON summary (diff stats + hashes + basic stats).\n"
      << "  --tiles <out.csv>            Write a CSV of differing tiles (can be large).\n"
      << "  --quiet                      Suppress stdout summary (errors still print).\n"
      << "  -h, --help                   Show this help.\n";
}

static void SetRgb(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = static_cast<std::size_t>((y * img.width + x) * 3);
  if (idx + 2 >= img.rgb.size()) return;
  img.rgb[idx + 0] = r;
  img.rgb[idx + 1] = g;
  img.rgb[idx + 2] = b;
}

static void GetRgb(const PpmImage& img, int x, int y, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  r = g = b = 0;
  if (x < 0 || y < 0 || x >= img.width || y >= img.height) return;
  const std::size_t idx = static_cast<std::size_t>((y * img.width + x) * 3);
  if (idx + 2 >= img.rgb.size()) return;
  r = img.rgb[idx + 0];
  g = img.rgb[idx + 1];
  b = img.rgb[idx + 2];
}

static void WriteStdoutSummary(const std::string& pathA, const std::string& pathB, const World& a, const World& b,
                               std::uint64_t hashA, std::uint64_t hashB, const WorldDiffStats& d)
{
  std::cout << "A: " << pathA << "\n";
  std::cout << "B: " << pathB << "\n";
  std::cout << "A: " << a.width() << "x" << a.height() << " seed=" << a.seed() << " day=" << a.stats().day
            << " pop=" << a.stats().population << " money=" << a.stats().money << "\n";
  std::cout << "B: " << b.width() << "x" << b.height() << " seed=" << b.seed() << " day=" << b.stats().day
            << " pop=" << b.stats().population << " money=" << b.stats().money << "\n";
  std::cout << "hashA: " << HexU64(hashA) << "\n";
  std::cout << "hashB: " << HexU64(hashB) << "\n";
  if (hashA == hashB) {
    std::cout << "world hash: MATCH\n";
  } else {
    std::cout << "world hash: DIFFER\n";
  }

  if (d.sizeMismatch) {
    std::cout << "size: mismatch (diff computed over overlap)\n";
  }
  std::cout << "tilesCompared: " << d.tilesCompared << "\n";
  std::cout << "tilesDifferent: " << d.tilesDifferent << "\n";
  std::cout << "  terrainDifferent:   " << d.terrainDifferent << "\n";
  std::cout << "  overlayDifferent:   " << d.overlayDifferent << "\n";
  std::cout << "  heightDifferent:    " << d.heightDifferent << "\n";
  std::cout << "  variationDifferent: " << d.variationDifferent << "\n";
  std::cout << "  levelDifferent:     " << d.levelDifferent << "\n";
  std::cout << "  occupantsDifferent: " << d.occupantsDifferent << "\n";
  std::cout << "  districtDifferent:  " << d.districtDifferent << "\n";
}

static bool WriteJson(const std::string& outPath, const std::string& pathA, const std::string& pathB, const World& a,
                      const World& b, std::uint64_t hashA, std::uint64_t hashB, const WorldDiffStats& d)
{
  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"fileA\": \"" << JsonEscape(pathA) << "\",\n";
  oss << "  \"fileB\": \"" << JsonEscape(pathB) << "\",\n";
  oss << "  \"hashA\": \"" << HexU64(hashA) << "\",\n";
  oss << "  \"hashB\": \"" << HexU64(hashB) << "\",\n";
  oss << "  \"hashMatch\": " << (hashA == hashB ? "true" : "false") << ",\n";

  auto writeWorld = [&](const char* name, const World& w) {
    const Stats& s = w.stats();
    oss << "  \"" << name << "\": {\n";
    oss << "    \"width\": " << w.width() << ",\n";
    oss << "    \"height\": " << w.height() << ",\n";
    oss << "    \"seed\": " << w.seed() << ",\n";
    oss << "    \"stats\": {\n";
    oss << "      \"day\": " << s.day << ",\n";
    oss << "      \"population\": " << s.population << ",\n";
    oss << "      \"money\": " << s.money << "\n";
    oss << "    }\n";
    oss << "  }";
  };

  writeWorld("worldA", a);
  oss << ",\n";
  writeWorld("worldB", b);
  oss << ",\n";

  oss << "  \"diff\": {\n";
  oss << "    \"sizeMismatch\": " << (d.sizeMismatch ? "true" : "false") << ",\n";
  oss << "    \"tilesCompared\": " << d.tilesCompared << ",\n";
  oss << "    \"tilesDifferent\": " << d.tilesDifferent << ",\n";
  oss << "    \"terrainDifferent\": " << d.terrainDifferent << ",\n";
  oss << "    \"overlayDifferent\": " << d.overlayDifferent << ",\n";
  oss << "    \"heightDifferent\": " << d.heightDifferent << ",\n";
  oss << "    \"variationDifferent\": " << d.variationDifferent << ",\n";
  oss << "    \"levelDifferent\": " << d.levelDifferent << ",\n";
  oss << "    \"occupantsDifferent\": " << d.occupantsDifferent << ",\n";
  oss << "    \"districtDifferent\": " << d.districtDifferent << "\n";
  oss << "  }\n";
  oss << "}\n";

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

static void DiffTileMask(const Tile& ta, const Tile& tb, float heightEps,
                         bool& terrainDiff, bool& overlayDiff, bool& heightDiff, bool& variationDiff, bool& levelDiff,
                         bool& occupantsDiff, bool& districtDiff)
{
  terrainDiff = (ta.terrain != tb.terrain);
  overlayDiff = (ta.overlay != tb.overlay);
  heightDiff = (std::fabs(ta.height - tb.height) > heightEps);
  variationDiff = (ta.variation != tb.variation);
  levelDiff = (ta.level != tb.level);
  occupantsDiff = (ta.occupants != tb.occupants);
  districtDiff = (ta.district != tb.district);
}

static bool AnyDiff(bool terrainDiff, bool overlayDiff, bool heightDiff, bool variationDiff, bool levelDiff,
                    bool occupantsDiff, bool districtDiff)
{
  return terrainDiff || overlayDiff || heightDiff || variationDiff || levelDiff || occupantsDiff || districtDiff;
}

static void CombinedColor(bool terrainDiff, bool overlayDiff, bool heightDiff, bool variationDiff, bool levelDiff,
                          bool occupantsDiff, bool districtDiff, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  // Keep the encoding intentionally simple:
  //   R: overlay/level changes
  //   G: height/occupants changes
  //   B: terrain/district changes
  //   variation contributes to both G and B (roads/tiles changing connection masks, etc.).
  r = (overlayDiff || levelDiff) ? 255 : 0;
  g = (heightDiff || occupantsDiff || variationDiff) ? 255 : 0;
  b = (terrainDiff || districtDiff || variationDiff) ? 255 : 0;

  // If *only* variation differs, render it as cyan to reduce confusion with terrain-only diffs.
  if (!terrainDiff && !overlayDiff && !heightDiff && variationDiff && !levelDiff && !occupantsDiff && !districtDiff) {
    r = 0;
    g = 255;
    b = 255;
  }
}

static void SpecificColor(DiffLayer layer, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  // High-contrast palette for single-field views.
  switch (layer) {
    case DiffLayer::Any:
      r = 255;
      g = 0;
      b = 0;
      return;
    case DiffLayer::Terrain:
      r = 0;
      g = 0;
      b = 255;
      return;
    case DiffLayer::Overlay:
      r = 255;
      g = 0;
      b = 0;
      return;
    case DiffLayer::Height:
      r = 0;
      g = 255;
      b = 0;
      return;
    case DiffLayer::Variation:
      r = 255;
      g = 255;
      b = 0;
      return;
    case DiffLayer::Level:
      r = 255;
      g = 0;
      b = 255;
      return;
    case DiffLayer::Occupants:
      r = 0;
      g = 255;
      b = 255;
      return;
    case DiffLayer::District:
      r = 255;
      g = 128;
      b = 0;
      return;
    case DiffLayer::Combined:
      break;
  }

  r = 255;
  g = 255;
  b = 255;
}

static bool HighlightWanted(DiffLayer layer,
                            bool terrainDiff, bool overlayDiff, bool heightDiff, bool variationDiff, bool levelDiff,
                            bool occupantsDiff, bool districtDiff)
{
  switch (layer) {
    case DiffLayer::Any:
    case DiffLayer::Combined:
      return AnyDiff(terrainDiff, overlayDiff, heightDiff, variationDiff, levelDiff, occupantsDiff, districtDiff);
    case DiffLayer::Terrain:
      return terrainDiff;
    case DiffLayer::Overlay:
      return overlayDiff;
    case DiffLayer::Height:
      return heightDiff;
    case DiffLayer::Variation:
      return variationDiff;
    case DiffLayer::Level:
      return levelDiff;
    case DiffLayer::Occupants:
      return occupantsDiff;
    case DiffLayer::District:
      return districtDiff;
  }
  return false;
}

static PpmImage RenderDiffPpm(const World& a, const World& b, const Options& opt)
{
  const int wA = a.width();
  const int hA = a.height();
  const int wB = b.width();
  const int hB = b.height();

  const int outW = std::max(wA, wB);
  const int outH = std::max(hA, hB);

  PpmImage out;
  out.width = outW;
  out.height = outH;
  out.rgb.assign(static_cast<std::size_t>(outW) * static_cast<std::size_t>(outH) * 3u, 0);

  PpmImage baseA;
  PpmImage baseB;
  if (opt.baseEnabled) {
    baseA = RenderPpmLayer(a, opt.baseLayer, nullptr, nullptr, nullptr);
    baseB = RenderPpmLayer(b, opt.baseLayer, nullptr, nullptr, nullptr);
  }

  // Seed base.
  if (opt.baseEnabled) {
    for (int y = 0; y < outH; ++y) {
      for (int x = 0; x < outW; ++x) {
        std::uint8_t r = 0, g = 0, bb = 0;
        if (x < wA && y < hA) {
          GetRgb(baseA, x, y, r, g, bb);
        } else if (x < wB && y < hB) {
          GetRgb(baseB, x, y, r, g, bb);
        }
        SetRgb(out, x, y, r, g, bb);
      }
    }
  }

  // Diff overlay.
  for (int y = 0; y < outH; ++y) {
    for (int x = 0; x < outW; ++x) {
      const bool hasA = (x >= 0 && y >= 0 && x < wA && y < hA);
      const bool hasB = (x >= 0 && y >= 0 && x < wB && y < hB);

      if (!hasA || !hasB) {
        // Size mismatch region.
        if (hasA || hasB) {
          SetRgb(out, x, y, 255, 0, 255);
        }
        continue;
      }

      const Tile& ta = a.at(x, y);
      const Tile& tb = b.at(x, y);

      bool terrainDiff = false;
      bool overlayDiff = false;
      bool heightDiff = false;
      bool variationDiff = false;
      bool levelDiff = false;
      bool occupantsDiff = false;
      bool districtDiff = false;
      DiffTileMask(ta, tb, opt.heightEps, terrainDiff, overlayDiff, heightDiff, variationDiff, levelDiff, occupantsDiff,
                   districtDiff);

      if (!HighlightWanted(opt.diffLayer, terrainDiff, overlayDiff, heightDiff, variationDiff, levelDiff, occupantsDiff,
                           districtDiff)) {
        continue;
      }

      std::uint8_t r = 0, g = 0, bb = 0;
      if (opt.diffLayer == DiffLayer::Combined) {
        CombinedColor(terrainDiff, overlayDiff, heightDiff, variationDiff, levelDiff, occupantsDiff, districtDiff, r, g,
                      bb);
      } else {
        SpecificColor(opt.diffLayer, r, g, bb);
      }

      SetRgb(out, x, y, r, g, bb);
    }
  }

  if (opt.scale > 1) {
    return ScaleNearest(out, opt.scale);
  }
  return out;
}

static bool WriteDiffTilesCsv(const std::string& path, const World& a, const World& b, float heightEps)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;

  const int wA = a.width();
  const int hA = a.height();
  const int wB = b.width();
  const int hB = b.height();
  const int outW = std::max(wA, wB);
  const int outH = std::max(hA, hB);

  f << "x,y,hasA,hasB,"
       "terrainA,terrainB,overlayA,overlayB,heightA,heightB,variationA,variationB,levelA,levelB,"
       "occupantsA,occupantsB,districtA,districtB\n";

  for (int y = 0; y < outH; ++y) {
    for (int x = 0; x < outW; ++x) {
      const bool hasA = (x >= 0 && y >= 0 && x < wA && y < hA);
      const bool hasB = (x >= 0 && y >= 0 && x < wB && y < hB);

      bool terrainDiff = false;
      bool overlayDiff = false;
      bool heightDiff = false;
      bool variationDiff = false;
      bool levelDiff = false;
      bool occupantsDiff = false;
      bool districtDiff = false;

      Tile ta{};
      Tile tb{};
      if (hasA) ta = a.at(x, y);
      if (hasB) tb = b.at(x, y);

      if (hasA && hasB) {
        DiffTileMask(ta, tb, heightEps, terrainDiff, overlayDiff, heightDiff, variationDiff, levelDiff, occupantsDiff,
                     districtDiff);
        if (!AnyDiff(terrainDiff, overlayDiff, heightDiff, variationDiff, levelDiff, occupantsDiff, districtDiff)) {
          continue;
        }
      } else {
        // Size mismatch => always emit.
      }

      auto iOr = [](bool has, int v) -> int { return has ? v : -1; };
      auto fOr = [](bool has, float v) -> float { return has ? v : 0.0f; };

      f << x << ',' << y << ',' << (hasA ? 1 : 0) << ',' << (hasB ? 1 : 0) << ','
        << iOr(hasA, static_cast<int>(ta.terrain)) << ',' << iOr(hasB, static_cast<int>(tb.terrain)) << ','
        << iOr(hasA, static_cast<int>(ta.overlay)) << ',' << iOr(hasB, static_cast<int>(tb.overlay)) << ','
        << fOr(hasA, ta.height) << ',' << fOr(hasB, tb.height) << ','
        << iOr(hasA, static_cast<int>(ta.variation)) << ',' << iOr(hasB, static_cast<int>(tb.variation)) << ','
        << iOr(hasA, static_cast<int>(ta.level)) << ',' << iOr(hasB, static_cast<int>(tb.level)) << ','
        << iOr(hasA, static_cast<int>(ta.occupants)) << ',' << iOr(hasB, static_cast<int>(tb.occupants)) << ','
        << iOr(hasA, static_cast<int>(ta.district)) << ',' << iOr(hasB, static_cast<int>(tb.district)) << '\n';
    }
  }

  return static_cast<bool>(f);
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  Options opt;

  std::string pathA;
  std::string pathB;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }
    if (arg == "--ppm") {
      if (i + 1 >= argc) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
      opt.outPpm = argv[++i];
      continue;
    }
    if (arg == "--scale") {
      if (i + 1 >= argc) {
        std::cerr << "--scale requires a value\n";
        return 2;
      }
      int v = 1;
      if (!ParseI32(argv[++i], &v) || v < 1 || v > 64) {
        std::cerr << "invalid --scale\n";
        return 2;
      }
      opt.scale = v;
      continue;
    }
    if (arg == "--height-eps") {
      if (i + 1 >= argc) {
        std::cerr << "--height-eps requires a value\n";
        return 2;
      }
      float eps = opt.heightEps;
      if (!ParseF32(argv[++i], &eps) || eps < 0.0f || !std::isfinite(eps)) {
        std::cerr << "invalid --height-eps\n";
        return 2;
      }
      opt.heightEps = eps;
      continue;
    }
    if (arg == "--layer") {
      if (i + 1 >= argc) {
        std::cerr << "--layer requires a value\n";
        return 2;
      }
      DiffLayer layer = opt.diffLayer;
      if (!ParseDiffLayer(argv[++i], layer)) {
        std::cerr << "invalid --layer\n";
        return 2;
      }
      opt.diffLayer = layer;
      continue;
    }
    if (arg == "--base") {
      if (i + 1 >= argc) {
        std::cerr << "--base requires a value (layer name or 'none')\n";
        return 2;
      }
      const std::string v = ToLower(argv[++i]);
      if (v == "none" || v == "off" || v == "0") {
        opt.baseEnabled = false;
        continue;
      }
      ExportLayer layer = opt.baseLayer;
      if (!ParseExportLayer(v, layer)) {
        std::cerr << "invalid --base layer\n";
        return 2;
      }
      opt.baseEnabled = true;
      opt.baseLayer = layer;
      continue;
    }
    if (arg == "--json") {
      if (i + 1 >= argc) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      opt.outJson = argv[++i];
      continue;
    }
    if (arg == "--tiles") {
      if (i + 1 >= argc) {
        std::cerr << "--tiles requires a path\n";
        return 2;
      }
      opt.outTilesCsv = argv[++i];
      continue;
    }
    if (arg == "--quiet") {
      opt.quiet = true;
      continue;
    }

    // Positional args.
    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    }
    if (pathA.empty()) {
      pathA = arg;
    } else if (pathB.empty()) {
      pathB = arg;
    } else {
      std::cerr << "unexpected positional arg: " << arg << "\n";
      return 2;
    }
  }

  if (pathA.empty() || pathB.empty()) {
    PrintHelp();
    return 2;
  }

  World a;
  World b;
  ProcGenConfig procA{};
  ProcGenConfig procB{};
  SimConfig simA{};
  SimConfig simB{};
  std::string err;

  if (!LoadWorldBinary(a, procA, simA, pathA, err)) {
    std::cerr << "failed to load A: " << err << "\n";
    return 1;
  }
  err.clear();
  if (!LoadWorldBinary(b, procB, simB, pathB, err)) {
    std::cerr << "failed to load B: " << err << "\n";
    return 1;
  }

  const std::uint64_t hashA = HashWorld(a, true);
  const std::uint64_t hashB = HashWorld(b, true);

  const WorldDiffStats d = DiffWorldTiles(a, b, opt.heightEps);

  if (!opt.quiet) {
    WriteStdoutSummary(pathA, pathB, a, b, hashA, hashB, d);
  }

  if (!opt.outJson.empty()) {
    if (!WriteJson(opt.outJson, pathA, pathB, a, b, hashA, hashB, d)) {
      std::cerr << "failed to write JSON: " << opt.outJson << "\n";
      return 1;
    }
  }

  if (!opt.outTilesCsv.empty()) {
    if (!WriteDiffTilesCsv(opt.outTilesCsv, a, b, opt.heightEps)) {
      std::cerr << "failed to write tiles CSV: " << opt.outTilesCsv << "\n";
      return 1;
    }
  }

  if (!opt.outPpm.empty()) {
    PpmImage img = RenderDiffPpm(a, b, opt);
    std::string ppmErr;
    if (!WriteImageAuto(opt.outPpm, img, ppmErr)) {
      std::cerr << "failed to write image: " << ppmErr << "\n";
      return 1;
    }
  }

  return 0;
}
