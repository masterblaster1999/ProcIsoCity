#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/StreetNames.hpp"
#include "isocity/Wayfinding.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, pos), &w)) return false;
  if (!ParseI32(s.substr(pos + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
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

void PrintHelp()
{
  std::cout
      << "proc_isocity_wayfind (headless wayfinding + turn-by-turn routes)\n\n"
      << "Geocodes procedural parcel addresses / intersections and computes a road route with\n"
      << "turn-by-turn style instructions (derived data; not persisted).\n\n"
      << "Usage:\n"
      << "  proc_isocity_wayfind --from <query> --to <query> [options]\n\n"
      << "Endpoint query formats:\n"
      << "  - \"123 Asterwood Ave\"         (address)\n"
      << "  - \"Asterwood Ave\"             (street name; picks a median address)\n"
      << "  - \"Asterwood Ave & 2nd St\"     (intersection; '&', '@', or ' and ')\n"
      << "  - \"x,y\"                       (tile coordinate; snaps to nearest road)\n\n"
      << "World input:\n"
      << "  --load <save.bin>          Load an existing save\n"
      << "  --seed <u64>               Procedural seed (when not using --load)\n"
      << "  --size <WxH>               Map size (when not using --load)\n\n"
      << "Outputs:\n"
      << "  --out-json <path>          Write route as JSON\n"
      << "  --out-image <path.(png|ppm)>  Write a tile-map snapshot with route overlay\n"
      << "  --image-layer <name>       Base layer for the snapshot (default: overlay)\n"
      << "  --image-scale <N>          Nearest-neighbor upscale factor (default: 6)\n\n"
      << "Street naming knobs (must match what you used for exports if you want exact strings):\n"
      << "  --merge-intersections <0|1>  (default: 1)\n"
      << "  --merge-corners <0|1>        (default: 1)\n"
      << "  --ordinals <0|1>             (default: 1)\n"
      << "  --number-step <N>            (default: 10)\n\n"
      << "Fuzzy matching knobs:\n"
      << "  --fuzzy <0|1>                (default: 1)\n"
      << "  --max-suggestions <N>         (default: 5)\n"
      << "  --max-edit <N>                (default: 4; <=0 means always accept best)\n";
}

static void OverlayRoute(isocity::PpmImage& img, const isocity::World& world,
                         const std::vector<isocity::Point>& path)
{
  using namespace isocity;
  const int w = world.width();
  const int h = world.height();
  if (img.width != w || img.height != h) return;
  if (img.rgb.size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u) return;

  auto setPx = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    if (!world.inBounds(x, y)) return;
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                           static_cast<std::size_t>(x)) *
                          3u;
    img.rgb[i + 0] = r;
    img.rgb[i + 1] = g;
    img.rgb[i + 2] = b;
  };

  // Main polyline.
  for (const Point& p : path) {
    setPx(p.x, p.y, 255, 60, 60);
  }

  // Start/end markers.
  if (!path.empty()) {
    setPx(path.front().x, path.front().y, 80, 255, 80);
    setPx(path.back().x, path.back().y, 80, 200, 255);
  }
}

static bool WriteRouteJson(const std::string& path, const isocity::RouteResult& r, std::string& outError)
{
  using namespace isocity;

  std::ofstream os(path);
  if (!os) {
    outError = "Failed to open output json";
    return false;
  }

  JsonWriteOptions opt;
  opt.pretty = true;
  opt.sortKeys = true;

  JsonWriter jw(os, opt);
  jw.beginObject();

  jw.key("from");
  jw.stringValue(r.from.full);
  jw.key("to");
  jw.stringValue(r.to.full);

  jw.key("start_road");
  jw.beginArray();
  jw.intValue(r.startRoad.x);
  jw.intValue(r.startRoad.y);
  jw.endArray();

  jw.key("goal_road");
  jw.beginArray();
  jw.intValue(r.goalRoad.x);
  jw.intValue(r.goalRoad.y);
  jw.endArray();

  jw.key("path_cost");
  jw.intValue(r.pathCost);

  jw.key("path");
  jw.beginArray();
  for (const Point& p : r.pathTiles) {
    jw.beginArray();
    jw.intValue(p.x);
    jw.intValue(p.y);
    jw.endArray();
  }
  jw.endArray();

  jw.key("maneuvers");
  jw.beginArray();
  for (const RouteManeuver& m : r.maneuvers) {
    jw.beginObject();
    jw.key("type");
    jw.stringValue(m.type);
    jw.key("modifier");
    jw.stringValue(m.modifier);
    jw.key("bearing_before");
    jw.intValue(m.bearingBefore);
    jw.key("bearing_after");
    jw.intValue(m.bearingAfter);
    jw.key("steps");
    jw.intValue(m.steps);
    jw.key("street_id");
    jw.intValue(m.streetId);
    jw.key("street_name");
    jw.stringValue(m.streetName);
    jw.key("path_start");
    jw.intValue(m.pathStart);
    jw.key("path_end");
    jw.intValue(m.pathEnd);
    jw.key("instruction");
    jw.stringValue(m.instruction);
    jw.endObject();
  }
  jw.endArray();

  jw.endObject();

  if (!jw.ok()) {
    outError = jw.error();
    return false;
  }
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  std::string fromQuery;
  std::string toQuery;

  std::string outJsonPath;
  std::string outImagePath;
  ExportLayer imageLayer = ExportLayer::Overlay;
  int imageScale = 6;

  StreetNamingConfig streetCfg;
  AddressConfig addrCfg;
  AddressIndexConfig indexCfg;

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
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &w, &h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--from") {
      if (!requireValue(i, val)) {
        std::cerr << "--from requires a query string\n";
        return 2;
      }
      fromQuery = val;
    } else if (arg == "--to") {
      if (!requireValue(i, val)) {
        std::cerr << "--to requires a query string\n";
        return 2;
      }
      toQuery = val;
    } else if (arg == "--out-json") {
      if (!requireValue(i, val)) {
        std::cerr << "--out-json requires a path\n";
        return 2;
      }
      outJsonPath = val;
    } else if (arg == "--out-image") {
      if (!requireValue(i, val)) {
        std::cerr << "--out-image requires a path (.png or .ppm)\n";
        return 2;
      }
      outImagePath = val;
    } else if (arg == "--image-layer") {
      if (!requireValue(i, val) || !ParseExportLayer(val, imageLayer)) {
        std::cerr << "--image-layer requires a valid layer name (e.g. overlay, terrain, district)\n";
        return 2;
      }
    } else if (arg == "--image-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &imageScale) || imageScale < 1) {
        std::cerr << "--image-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--merge-intersections") {
      if (!requireValue(i, val) || !ParseBool01(val, &streetCfg.mergeThroughIntersections)) {
        std::cerr << "--merge-intersections requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--merge-corners") {
      if (!requireValue(i, val) || !ParseBool01(val, &streetCfg.mergeThroughCorners)) {
        std::cerr << "--merge-corners requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--ordinals") {
      if (!requireValue(i, val) || !ParseBool01(val, &streetCfg.allowOrdinalNames)) {
        std::cerr << "--ordinals requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--number-step") {
      if (!requireValue(i, val) || !ParseI32(val, &addrCfg.numberStep) || addrCfg.numberStep <= 0) {
        std::cerr << "--number-step requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--fuzzy") {
      if (!requireValue(i, val) || !ParseBool01(val, &indexCfg.allowFuzzy)) {
        std::cerr << "--fuzzy requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--max-suggestions") {
      if (!requireValue(i, val) || !ParseI32(val, &indexCfg.maxSuggestions) || indexCfg.maxSuggestions < 0) {
        std::cerr << "--max-suggestions requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--max-edit") {
      if (!requireValue(i, val) || !ParseI32(val, &indexCfg.maxAutoEditDistance)) {
        std::cerr << "--max-edit requires an integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (fromQuery.empty() || toQuery.empty()) {
    std::cerr << "--from and --to are required\n\n";
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

  const StreetNamingResult streets = BuildStreetNames(world, streetCfg);
  const std::vector<ParcelAddress> addrs = BuildParcelAddresses(world, streets, addrCfg);
  const AddressIndex index = BuildAddressIndex(addrs, indexCfg);

  const GeocodeMatch a = GeocodeEndpoint(world, streets, index, fromQuery);
  if (!a.ok) {
    std::cerr << "From geocode failed: " << a.error << "\n";
    for (const std::string& s : a.suggestions) {
      std::cerr << "  suggestion: " << s << "\n";
    }
    return 2;
  }

  const GeocodeMatch b = GeocodeEndpoint(world, streets, index, toQuery);
  if (!b.ok) {
    std::cerr << "To geocode failed: " << b.error << "\n";
    for (const std::string& s : b.suggestions) {
      std::cerr << "  suggestion: " << s << "\n";
    }
    return 2;
  }

  const RouteResult r = RouteBetweenEndpoints(world, streets, a.endpoint, b.endpoint);
  if (!r.ok) {
    std::cerr << "Route failed: " << r.error << "\n";
    return 2;
  }

  std::cout << "Wayfind\n";
  std::cout << "  from: " << r.from.full << "\n";
  std::cout << "  to:   " << r.to.full << "\n";
  std::cout << "  cost: " << r.pathCost << " steps\n";
  std::cout << "  maneuvers: " << r.maneuvers.size() << "\n\n";

  for (std::size_t i = 0; i < r.maneuvers.size(); ++i) {
    std::cout << (i + 1) << ". " << r.maneuvers[i].instruction << "\n";
  }

  if (!outJsonPath.empty()) {
    std::string err;
    if (!WriteRouteJson(outJsonPath, r, err)) {
      std::cerr << "Failed to write json: " << outJsonPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "\nWrote: " << outJsonPath << "\n";
  }

  if (!outImagePath.empty()) {
    PpmImage img = RenderPpmLayer(world, imageLayer);
    OverlayRoute(img, world, r.pathTiles);
    if (imageScale > 1) {
      img = ScaleNearest(img, imageScale);
    }
    std::string err;
    if (!WriteImageAuto(outImagePath, img, err)) {
      std::cerr << "Failed to write image: " << outImagePath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "Wrote: " << outImagePath << "\n";
  }

  return 0;
}
