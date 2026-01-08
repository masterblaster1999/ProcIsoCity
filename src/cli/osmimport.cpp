#include "isocity/AutoBuild.hpp"
#include "isocity/OsmImport.hpp"
#include "isocity/Random.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
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
  std::size_t off = 0;
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    base = 16;
    off = 2;
  }

  if (off >= s.size()) return false;
  const char* start = s.c_str() + off;
  if (*start == '\0') return false;
  char* end = nullptr;
  errno = 0;
  const unsigned long long v = std::strtoull(start, &end, base);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseF64(const std::string& s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

bool ParseSize(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t x = s.find('x');
  if (x == std::string::npos) return false;
  const std::string a = s.substr(0, x);
  const std::string b = s.substr(x + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

void PrintUsage()
{
  std::cout
      << "proc_isocity_osmimport: import OpenStreetMap (OSM XML) features into ProcIsoCity.\n\n"
      << "Usage:\n"
      << "  proc_isocity_osmimport --osm <extract.osm> --save <out.bin> [options]\n\n"
      << "Core options:\n"
      << "  --osm <path>              Input OSM XML (.osm) file.\n"
      << "  --save <path>             Output save (.bin).\n"
      << "  --seed <u64>              World seed (default: time-based).\n"
      << "  --size <WxH>              Explicit world size (e.g. 512x512).\n"
      << "  --meters-per-tile <f>     Auto-size resolution when --size is omitted (default: 20).\n"
      << "  --padding <n>             Tile padding around imported bounds (default: 2).\n"
      << "  --prefer-bounds <0|1>     Prefer <bounds> tag over scanning nodes (default: 1).\n\n"
      << "Feature toggles (defaults preserve the original behavior: roads-only):\n"
      << "  --roads <0|1>             Import highway ways as roads (default: 1).\n"
      << "  --water <0|1>             Import water areas/ways into Terrain::Water (default: 0).\n"
      << "  --landuse <0|1>           Import landuse=residential/commercial/industrial as zones (default: 0).\n"
      << "  --parks <0|1>             Import leisure=park/garden/etc as parks (default: 0).\n"
      << "  --buildings <0|1>         Import building=* polygons as zones (default: 0).\n"
      << "  --overwrite <0|1>         Allow polygons to overwrite existing non-road overlays (default: 0).\n"
      << "  --full                    Shortcut for: --water 1 --landuse 1 --parks 1 --buildings 1\n"
      << "  --waterway-radius <n>     Manhattan radius for waterway lines (default: 1).\n\n"
      << "Road raster options:\n"
      << "  --thicken-by-class <0|1>  Street=thin, avenues thicker, highways thickest (default: 1).\n"
      << "  --fixed-radius <n>        Override raster width (Manhattan radius in tiles).\n\n"
      << "Optional city growth:\n"
      << "  --autobuild-days <n>      Run the deterministic AutoBuild bot for N days after import.\n"
      << "  --autobuild <k=v>         Override AutoBuildConfig keys (repeatable).\n\n";
}


} // namespace

int main(int argc, char** argv)
{
  std::string osmPath;
  std::string savePath;

  isocity::OsmImportConfig cfg;

  std::uint64_t seed = isocity::TimeSeed();

  int autobuildDays = 0;
  isocity::AutoBuildConfig abCfg;
  std::vector<std::string> abKVs;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto requireValue = [&](std::string* out) -> bool {
      if (!out) return false;
      if (i + 1 >= argc) return false;
      *out = argv[++i];
      return true;
    };

    if (a == "--help" || a == "-h") {
      PrintUsage();
      return 0;
    }

    if (a == "--osm") {
      if (!requireValue(&osmPath)) {
        std::cerr << "--osm requires a value\n";
        return 2;
      }
      continue;
    }

    if (a == "--save") {
      if (!requireValue(&savePath)) {
        std::cerr << "--save requires a value\n";
        return 2;
      }
      continue;
    }

    if (a == "--seed") {
      std::string v;
      if (!requireValue(&v) || !ParseU64(v, &seed)) {
        std::cerr << "Invalid --seed\n";
        return 2;
      }
      continue;
    }

    if (a == "--size") {
      std::string v;
      int w = 0, h = 0;
      if (!requireValue(&v) || !ParseSize(v, &w, &h)) {
        std::cerr << "Invalid --size (expected WxH)\n";
        return 2;
      }
      cfg.width = w;
      cfg.height = h;
      continue;
    }

    if (a == "--meters-per-tile") {
      std::string v;
      double mpt = 0.0;
      if (!requireValue(&v) || !ParseF64(v, &mpt) || !(mpt > 0.0)) {
        std::cerr << "Invalid --meters-per-tile\n";
        return 2;
      }
      cfg.metersPerTile = mpt;
      continue;
    }

    if (a == "--padding") {
      std::string v;
      int p = 0;
      if (!requireValue(&v) || !ParseI32(v, &p) || p < 0) {
        std::cerr << "Invalid --padding\n";
        return 2;
      }
      cfg.padding = p;
      continue;
    }

    if (a == "--prefer-bounds") {
      std::string v;
      bool b = true;
      if (!requireValue(&v) || !ParseBool01(v, &b)) {
        std::cerr << "Invalid --prefer-bounds\n";
        return 2;
      }
      cfg.preferBoundsTag = b;
      continue;
    }
// --- feature toggles ---
if (a == "--full") {
  cfg.importWater = true;
  cfg.importLanduse = true;
  cfg.importParks = true;
  cfg.importBuildings = true;
  continue;
}

if (a == "--roads") {
  std::string v;
  bool b = true;
  if (!requireValue(&v) || !ParseBool01(v, &b)) {
    std::cerr << "Invalid --roads\n";
    return 2;
  }
  cfg.importRoads = b;
  continue;
}

if (a == "--water") {
  std::string v;
  bool b = false;
  if (!requireValue(&v) || !ParseBool01(v, &b)) {
    std::cerr << "Invalid --water\n";
    return 2;
  }
  cfg.importWater = b;
  continue;
}

if (a == "--landuse") {
  std::string v;
  bool b = false;
  if (!requireValue(&v) || !ParseBool01(v, &b)) {
    std::cerr << "Invalid --landuse\n";
    return 2;
  }
  cfg.importLanduse = b;
  continue;
}

if (a == "--parks") {
  std::string v;
  bool b = false;
  if (!requireValue(&v) || !ParseBool01(v, &b)) {
    std::cerr << "Invalid --parks\n";
    return 2;
  }
  cfg.importParks = b;
  continue;
}

if (a == "--buildings") {
  std::string v;
  bool b = false;
  if (!requireValue(&v) || !ParseBool01(v, &b)) {
    std::cerr << "Invalid --buildings\n";
    return 2;
  }
  cfg.importBuildings = b;
  continue;
}

if (a == "--overwrite") {
  std::string v;
  bool b = false;
  if (!requireValue(&v) || !ParseBool01(v, &b)) {
    std::cerr << "Invalid --overwrite\n";
    return 2;
  }
  cfg.overwriteNonRoadOverlays = b;
  continue;
}

if (a == "--waterway-radius") {
  std::string v;
  int r = 0;
  if (!requireValue(&v) || !ParseI32(v, &r) || r < 0) {
    std::cerr << "Invalid --waterway-radius\n";
    return 2;
  }
  cfg.waterwayRadius = r;
  continue;
}


    if (a == "--thicken-by-class") {
      std::string v;
      bool b = true;
      if (!requireValue(&v) || !ParseBool01(v, &b)) {
        std::cerr << "Invalid --thicken-by-class\n";
        return 2;
      }
      cfg.thickenByClass = b;
      continue;
    }

    if (a == "--fixed-radius") {
      std::string v;
      int r = 0;
      if (!requireValue(&v) || !ParseI32(v, &r) || r < 0) {
        std::cerr << "Invalid --fixed-radius\n";
        return 2;
      }
      cfg.fixedRadius = r;
      continue;
    }

    if (a == "--autobuild-days") {
      std::string v;
      if (!requireValue(&v) || !ParseI32(v, &autobuildDays) || autobuildDays < 0) {
        std::cerr << "Invalid --autobuild-days\n";
        return 2;
      }
      continue;
    }

    if (a == "--autobuild") {
      std::string kv;
      if (!requireValue(&kv) || kv.find('=') == std::string::npos) {
        std::cerr << "--autobuild requires key=value\n";
        return 2;
      }
      abKVs.push_back(kv);
      continue;
    }

    std::cerr << "Unknown argument: " << a << "\n";
    PrintUsage();
    return 2;
  }

  if (osmPath.empty() || savePath.empty()) {
    PrintUsage();
    return 2;
  }

  // Parse AutoBuild overrides.
  for (const std::string& kv : abKVs) {
    const std::size_t eq = kv.find('=');
    const std::string key = kv.substr(0, eq);
    const std::string val = kv.substr(eq + 1);
    std::string err;
    if (!isocity::ParseAutoBuildKey(key, val, abCfg, err)) {
      std::cerr << "Invalid --autobuild " << kv << ": " << err << "\n";
      return 2;
    }
  }

  isocity::World world;
  isocity::OsmImportStats stats;
  std::string err;

  if (!isocity::ImportOsmXmlRoadsToNewWorld(osmPath, seed, cfg, world, &stats, err)) {
    std::cerr << "OSM import failed: " << err << "\n";
    return 1;
  }

  // Optionally run the deterministic builder to populate zones around imported roads.
  isocity::Simulator sim;
  sim.refreshDerivedStats(world);

  if (autobuildDays > 0) {
    const isocity::AutoBuildReport rep = isocity::RunAutoBuild(world, sim, abCfg, autobuildDays);
    std::cout << "AutoBuild: days=" << rep.daysSimulated << " roadsBuilt=" << rep.roadsBuilt
              << " roadsUpgraded=" << rep.roadsUpgraded << " zonesBuilt=" << rep.zonesBuilt
              << " parksBuilt=" << rep.parksBuilt << " failed=" << rep.failedBuilds << "\n";
  } else {
    // Still refresh derived stats so roads/parks counts are reasonable in the save.
    sim.refreshDerivedStats(world);
  }

  if (!isocity::SaveWorldBinary(world, savePath, err)) {
    std::cerr << "Failed to write save: " << err << "\n";
    return 1;
  }

  std::cout << "Imported OSM -> save:\n"
          << "  out=" << savePath << "\n"
          << "  size=" << world.width() << "x" << world.height() << " seed=" << world.seed() << "\n"
          << "  bounds=" << stats.bounds.minLat << "," << stats.bounds.minLon << " .. " << stats.bounds.maxLat
          << "," << stats.bounds.maxLon << "\n"
          << "  nodes=" << stats.nodesParsed << " ways=" << stats.waysParsed << "\n"
          << "  ways: highways=" << stats.highwayWaysImported << " water=" << stats.waterWaysImported
          << " landuse=" << stats.landuseWaysImported << " parks=" << stats.parkWaysImported
          << " buildings=" << stats.buildingWaysImported << "\n"
          << "  tiles: roads=" << stats.roadTilesPainted << " water=" << stats.waterTilesPainted
          << " zones=" << stats.zoneTilesPainted << " parks=" << stats.parkTilesPainted << "\n";

  return 0;
}
