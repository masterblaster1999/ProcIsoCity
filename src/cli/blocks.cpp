#include "isocity/CityBlocks.hpp"
#include "isocity/Export.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
  errno = 0;
  long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0) return false;
  if (end != s.c_str() + s.size()) return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;

  *out = static_cast<int>(v);
  return true;
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  char* end = nullptr;
  errno = 0;
  unsigned long long v = std::strtoull(s.c_str(), &end, 10);
  if (errno != 0) return false;
  if (end != s.c_str() + s.size()) return false;

  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
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

std::uint32_t Hash32(std::uint32_t x)
{
  // Small 32-bit mix for debug coloring (deterministic).
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

void ColorForId(int id, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const std::uint32_t v = Hash32(static_cast<std::uint32_t>(id));
  r = static_cast<std::uint8_t>(64 + (v & 0xBFu));
  g = static_cast<std::uint8_t>(64 + ((v >> 8) & 0xBFu));
  b = static_cast<std::uint8_t>(64 + ((v >> 16) & 0xBFu));
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_blocks (headless city-block extraction)\n\n"
      << "Builds road-separated land blocks (components of non-road, non-water tiles) and exports\n"
      << "summary metrics to JSON/CSV, plus optional debug label images.\n\n"
      << "Usage:\n"
      << "  proc_isocity_blocks [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                    [--json <out.json>] [--csv <out.csv>] [--tile-csv <out.csv>]\n"
      << "                    [--ppm <out.ppm>] [--ppm-scale <N>]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>   Load a save file (overrides --seed/--size).\n"
      << "  --seed <u64>        Seed for procedural generation (default: 1).\n"
      << "  --size <WxH>        World size (default: 96x96).\n\n"
      << "Outputs:\n"
      << "  --json <out.json>       Write a JSON summary.\n"
      << "  --csv <out.csv>         Write a per-block CSV summary.\n"
      << "  --tile-csv <out.csv>    Write a per-tile block-id CSV grid (-1 for road/water).\n"
      << "  --ppm <out.ppm>         Write a debug PPM label image.\n"
      << "  --ppm-scale <N>         Upscale factor for PPM (nearest; default: 4).\n";
}

bool WriteCsv(const std::string& path, const isocity::CityBlocksResult& r)
{
  std::ofstream f(path);
  if (!f) return false;

  f << "id,area,minX,minY,maxX,maxY,roadEdges,waterEdges,outsideEdges,roadAdjTiles,parks,residential,commercial,industrial,other\n";
  for (const auto& b : r.blocks) {
    f << b.id << "," << b.area << "," << b.minX << "," << b.minY << "," << b.maxX << "," << b.maxY << ",";
    f << b.roadEdges << "," << b.waterEdges << "," << b.outsideEdges << ",";
    f << b.roadAdjTiles << ",";
    f << b.parks << "," << b.residential << "," << b.commercial << "," << b.industrial << "," << b.other << "\n";
  }
  return true;
}

bool WriteTileCsv(const std::string& path, const isocity::CityBlocksResult& r)
{
  std::ofstream f(path);
  if (!f) return false;

  for (int y = 0; y < r.h; ++y) {
    for (int x = 0; x < r.w; ++x) {
      const int id = r.tileToBlock[static_cast<std::size_t>(y) * static_cast<std::size_t>(r.w) +
                                   static_cast<std::size_t>(x)];
      if (x) f << ",";
      f << id;
    }
    f << "\n";
  }
  return true;
}

bool WriteJson(const std::string& path, const isocity::World& world, const isocity::CityBlocksResult& r)
{
  std::ofstream f(path);
  if (!f) return false;

  // Summary metrics.
  int maxArea = 0;
  std::int64_t sumArea = 0;
  int edgeTouching = 0;
  for (const auto& b : r.blocks) {
    maxArea = std::max(maxArea, b.area);
    sumArea += static_cast<std::int64_t>(b.area);
    if (b.outsideEdges > 0) edgeTouching++;
  }
  const double meanArea = r.blocks.empty() ? 0.0 : static_cast<double>(sumArea) / static_cast<double>(r.blocks.size());

  f << "{\n";
  f << "  \"width\": " << r.w << ",\n";
  f << "  \"height\": " << r.h << ",\n";
  f << "  \"seed\": " << world.seed() << ",\n";
  f << "  \"blockCount\": " << r.blocks.size() << ",\n";
  f << "  \"summary\": {\n";
  f << "    \"maxArea\": " << maxArea << ",\n";
  f << "    \"meanArea\": " << meanArea << ",\n";
  f << "    \"edgeTouchingBlocks\": " << edgeTouching << "\n";
  f << "  },\n";
  f << "  \"blocks\": [\n";

  for (std::size_t i = 0; i < r.blocks.size(); ++i) {
    const auto& b = r.blocks[i];
    f << "    {\n";
    f << "      \"id\": " << b.id << ",\n";
    f << "      \"area\": " << b.area << ",\n";
    f << "      \"bounds\": {\"minX\": " << b.minX << ", \"minY\": " << b.minY << ", \"maxX\": " << b.maxX
      << ", \"maxY\": " << b.maxY << "},\n";
    f << "      \"edges\": {\"road\": " << b.roadEdges << ", \"water\": " << b.waterEdges << ", \"outside\": "
      << b.outsideEdges << "},\n";
    f << "      \"roadAdjTiles\": " << b.roadAdjTiles << ",\n";
    f << "      \"composition\": {\"parks\": " << b.parks << ", \"residential\": " << b.residential
      << ", \"commercial\": " << b.commercial << ", \"industrial\": " << b.industrial << ", \"other\": " << b.other
      << "}\n";
    f << "    }";
    if (i + 1 < r.blocks.size()) f << ",";
    f << "\n";
  }

  f << "  ]\n";
  f << "}\n";
  return true;
}

bool WritePpmLabels(const std::string& path, const isocity::World& world, const isocity::CityBlocksResult& r, int scale)
{
  using namespace isocity;

  PpmImage img{};
  img.width = r.w;
  img.height = r.h;
  img.rgb.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u, 0);

  const int w = r.w;
  const int h = r.h;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);

      std::uint8_t rr = 0;
      std::uint8_t gg = 0;
      std::uint8_t bb = 0;

      const int bid = r.tileToBlock[idx];
      if (bid >= 0) {
        ColorForId(bid, rr, gg, bb);
      } else if (t.terrain == Terrain::Water) {
        rr = 20;
        gg = 50;
        bb = 170;
      } else if (t.overlay == Overlay::Road) {
        rr = gg = bb = 220;
      } else {
        rr = gg = bb = 0;
      }

      img.rgb[idx * 3u + 0] = rr;
      img.rgb[idx * 3u + 1] = gg;
      img.rgb[idx * 3u + 2] = bb;
    }
  }

  if (scale > 1) {
    img = ScaleNearest(img, scale);
  }

  std::string err;
  if (!WritePpm(path, img, err)) {
    std::cerr << "Failed to write PPM: " << err << "\n";
    return false;
  }
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string jsonPath;
  std::string csvPath;
  std::string tileCsvPath;
  std::string ppmPath;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  int ppmScale = 4;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, loadPath)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
    } else if (arg == "--seed") {
      std::string v;
      if (!requireValue(i, v) || !ParseU64(v, &seed)) {
        std::cerr << "--seed requires a u64\n";
        return 2;
      }
    } else if (arg == "--size") {
      std::string v;
      if (!requireValue(i, v) || !ParseWxH(v, &w, &h)) {
        std::cerr << "--size requires WxH\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, jsonPath)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
    } else if (arg == "--csv") {
      if (!requireValue(i, csvPath)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
    } else if (arg == "--tile-csv") {
      if (!requireValue(i, tileCsvPath)) {
        std::cerr << "--tile-csv requires a path\n";
        return 2;
      }
    } else if (arg == "--ppm") {
      if (!requireValue(i, ppmPath)) {
        std::cerr << "--ppm requires a path\n";
        return 2;
      }
    } else if (arg == "--ppm-scale") {
      std::string v;
      if (!requireValue(i, v) || !ParseI32(v, &ppmScale) || ppmScale <= 0) {
        std::cerr << "--ppm-scale requires a positive integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  // Default outputs if none specified.
  if (jsonPath.empty() && csvPath.empty() && tileCsvPath.empty() && ppmPath.empty()) {
    std::cerr << "No outputs specified. Use --json/--csv/--tile-csv/--ppm.\n";
    PrintHelp();
    return 2;
  }

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  const CityBlocksResult r = BuildCityBlocks(world);

  if (!jsonPath.empty()) {
    if (!WriteJson(jsonPath, world, r)) {
      std::cerr << "Failed to write JSON: " << jsonPath << "\n";
      return 2;
    }
  }

  if (!csvPath.empty()) {
    if (!WriteCsv(csvPath, r)) {
      std::cerr << "Failed to write CSV: " << csvPath << "\n";
      return 2;
    }
  }

  if (!tileCsvPath.empty()) {
    if (!WriteTileCsv(tileCsvPath, r)) {
      std::cerr << "Failed to write tile CSV: " << tileCsvPath << "\n";
      return 2;
    }
  }

  if (!ppmPath.empty()) {
    if (!WritePpmLabels(ppmPath, world, r, ppmScale)) {
      return 2;
    }
  }

  return 0;
}
