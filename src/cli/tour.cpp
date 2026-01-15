#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/TourPlanner.hpp"

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

bool EnsureParentDir(const std::string& path)
{
  try {
    const std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (parent.empty()) return true;
    std::filesystem::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_tour (procedural tour guide + itinerary poster)\n\n"
      << "Synthesizes a small set of interesting POIs (parks, peaks, bottlenecks...) and builds\n"
      << "a walking tour between them using the Wayfinding module. Optionally renders a\n"
      << "Cartography-style poster with the route and numbered stops.\n\n"
      << "Usage:\n"
      << "  proc_isocity_tour --load <save.bin> [options]\n"
      << "  proc_isocity_tour --seed <u64> --size <WxH> [options]\n\n"
      << "World input:\n"
      << "  --load <save.bin>          Load an existing save\n"
      << "  --seed <u64>               Procedural seed (when not using --load)\n"
      << "  --size <WxH>               Map size (when not using --load, default: 128x128)\n\n"
      << "Tour inputs:\n"
      << "  --start <query>            Optional start endpoint (address / intersection / x,y)\n"
      << "  --stops <N>                Maximum tour stops (default: 6)\n"
      << "  --min-sep <N>              Minimum POI separation in tiles (default: 10)\n"
      << "  --centrality-sources <N>   Centrality sample sources (0=exact, default: 48)\n"
      << "  --seed-salt <u64>          Tie-break salt to get a different tour for same city\n"
      << "  --no-bottleneck            Disable bottleneck POI\n"
      << "  --no-peak                  Disable peak POI\n"
      << "  --no-park                  Disable grand park POI\n"
      << "  --no-waterfront            Disable waterfront POI\n"
      << "  --no-market                Disable market POI\n"
      << "  --no-industry              Disable works POI\n"
      << "  --no-district-hubs          Disable district hubs\n\n"
      << "Outputs:\n"
      << "  --out-json <file>          Write tour plan as JSON\n"
      << "  --out-md <file>            Write tour plan as Markdown\n"
      << "  --out-image <file.png>     Render tour poster PNG\n\n"
      << "Poster options (subset of proc_isocity_cartography):\n"
      << "  --layer <name>             Base layer for the poster (default: overlay)\n"
      << "  --poster <0|1>             Add title + legend margins (default: 1)\n"
      << "  --title <text>             Override poster title\n";
}

static bool WriteTourJson(const std::string& path, const isocity::TourPlan& t, std::string& outError)
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
  jw.key("title");
  jw.stringValue(t.title);
  jw.key("seed");
  jw.uintValue(t.seed);
  jw.key("width");
  jw.intValue(t.width);
  jw.key("height");
  jw.intValue(t.height);
  jw.key("start_query");
  jw.stringValue(t.startQuery);

  jw.key("start");
  jw.beginObject();
  jw.key("full");
  jw.stringValue(t.start.full);
  jw.key("road");
  jw.beginArray();
  jw.intValue(t.start.roadTile.x);
  jw.intValue(t.start.roadTile.y);
  jw.endArray();
  jw.endObject();

  jw.key("total_steps");
  jw.intValue(t.totalSteps);

  jw.key("stops");
  jw.beginArray();
  for (std::size_t i = 0; i < t.stops.size(); ++i) {
    const TourStop& s = t.stops[i];
    const Poi& p = s.poi;

    jw.beginObject();
    jw.key("index");
    jw.intValue(static_cast<int>(i + 1));
    jw.key("kind");
    jw.stringValue(PoiKindName(p.kind));
    jw.key("name");
    jw.stringValue(p.name);
    jw.key("description");
    jw.stringValue(p.description);
    jw.key("road");
    jw.beginArray();
    jw.intValue(p.roadTile.x);
    jw.intValue(p.roadTile.y);
    jw.endArray();
    jw.key("street_id");
    jw.intValue(p.streetId);
    jw.key("street_name");
    jw.stringValue(p.streetName);
    jw.key("near_address");
    jw.stringValue(p.nearAddress);
    jw.key("district");
    jw.intValue(p.district);
    jw.key("district_name");
    jw.stringValue(p.districtName);
    jw.key("feature_value");
    jw.numberValue(static_cast<double>(p.featureValue));
    jw.key("score");
    jw.uintValue(p.score);

    // Route from previous stop.
    jw.key("route");
    jw.beginObject();
    jw.key("from");
    jw.stringValue(s.routeFromPrev.from.full);
    jw.key("to");
    jw.stringValue(s.routeFromPrev.to.full);
    jw.key("path_cost");
    jw.intValue(s.routeFromPrev.pathCost);

    jw.key("path");
    jw.beginArray();
    for (const Point& pt : s.routeFromPrev.pathTiles) {
      jw.beginArray();
      jw.intValue(pt.x);
      jw.intValue(pt.y);
      jw.endArray();
    }
    jw.endArray();

    jw.key("maneuvers");
    jw.beginArray();
    for (const RouteManeuver& m : s.routeFromPrev.maneuvers) {
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
      jw.key("instruction");
      jw.stringValue(m.instruction);
      jw.endObject();
    }
    jw.endArray();
    jw.endObject();

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

static bool WriteTourMarkdown(const std::string& path, const isocity::TourPlan& t, std::string& outError)
{
  using namespace isocity;
  std::ofstream os(path);
  if (!os) {
    outError = "Failed to open output markdown";
    return false;
  }

  os << "# Walking Tour of " << t.title << "\n\n";
  os << "- Seed: `" << t.seed << "`\n";
  os << "- Size: " << t.width << "x" << t.height << "\n";
  if (!t.startQuery.empty()) {
    os << "- Start query: \"" << t.startQuery << "\"\n";
  }
  os << "- Start: **" << t.start.full << "** (road " << t.start.roadTile.x << "," << t.start.roadTile.y << ")\n";
  os << "- Total steps: " << t.totalSteps << "\n\n";

  for (std::size_t i = 0; i < t.stops.size(); ++i) {
    const TourStop& s = t.stops[i];
    const Poi& p = s.poi;
    os << "## Stop " << (i + 1) << " — " << p.name << "\n\n";
    os << "- Kind: `" << PoiKindName(p.kind) << "`\n";
    if (!p.districtName.empty()) os << "- District: " << p.districtName << "\n";
    if (!p.streetName.empty()) os << "- Street: " << p.streetName << "\n";
    if (!p.nearAddress.empty()) os << "- Nearest address: " << p.nearAddress << "\n";
    os << "- Road tile: " << p.roadTile.x << "," << p.roadTile.y << "\n";
    os << "\n" << p.description << "\n\n";

    os << "### Directions\n\n";
    if (!s.routeFromPrev.ok) {
      os << "(No route)\n\n";
      continue;
    }
    for (std::size_t mi = 0; mi < s.routeFromPrev.maneuvers.size(); ++mi) {
      os << (mi + 1) << ". " << s.routeFromPrev.maneuvers[mi].instruction << "\n";
    }
    os << "\n";
  }

  return true;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;

  std::string startQuery;
  std::string outJson;
  std::string outMd;
  std::string outImage;

  // Tour config.
  TourConfig tourCfg;

  // Poster config.
  TourPosterConfig posterCfg;
  posterCfg.cartCfg.poster = true;

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
    } else if (arg == "--start") {
      if (!requireValue(i, val)) {
        std::cerr << "--start requires a query string\n";
        return 2;
      }
      startQuery = val;
    } else if (arg == "--out-json") {
      if (!requireValue(i, val)) {
        std::cerr << "--out-json requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--out-md") {
      if (!requireValue(i, val)) {
        std::cerr << "--out-md requires a path\n";
        return 2;
      }
      outMd = val;
    } else if (arg == "--out-image") {
      if (!requireValue(i, val)) {
        std::cerr << "--out-image requires a path\n";
        return 2;
      }
      outImage = val;
    } else if (arg == "--stops") {
      if (!requireValue(i, val) || !ParseI32(val, &tourCfg.maxStops) || tourCfg.maxStops < 0) {
        std::cerr << "--stops requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--min-sep") {
      if (!requireValue(i, val) || !ParseI32(val, &tourCfg.minSeparationTiles) || tourCfg.minSeparationTiles < 0) {
        std::cerr << "--min-sep requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--centrality-sources") {
      if (!requireValue(i, val) || !ParseI32(val, &tourCfg.centralityMaxSources) || tourCfg.centralityMaxSources < 0) {
        std::cerr << "--centrality-sources requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--seed-salt") {
      if (!requireValue(i, val) || !ParseU64(val, &tourCfg.seedSalt)) {
        std::cerr << "--seed-salt requires a valid u64\n";
        return 2;
      }
    } else if (arg == "--no-bottleneck") {
      tourCfg.includeBottleneck = false;
    } else if (arg == "--no-peak") {
      tourCfg.includePeak = false;
    } else if (arg == "--no-park") {
      tourCfg.includePark = false;
    } else if (arg == "--no-waterfront") {
      tourCfg.includeWaterfront = false;
    } else if (arg == "--no-market") {
      tourCfg.includeMarket = false;
    } else if (arg == "--no-industry") {
      tourCfg.includeIndustry = false;
    } else if (arg == "--no-district-hubs") {
      tourCfg.includeDistrictHubs = false;
    } else if (arg == "--layer") {
      if (!requireValue(i, val) || !ParseExportLayer(val, posterCfg.layer)) {
        std::cerr << "--layer requires a valid layer name (e.g. overlay)\n";
        return 2;
      }
    } else if (arg == "--poster") {
      if (!requireValue(i, val) || !ParseBool01(val, &posterCfg.cartCfg.poster)) {
        std::cerr << "--poster requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--title") {
      if (!requireValue(i, val)) {
        std::cerr << "--title requires a string\n";
        return 2;
      }
      posterCfg.cartCfg.titleOverride = val;
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (outJson.empty() && outMd.empty() && outImage.empty()) {
    std::cerr << "No outputs specified. Use --out-json, --out-md, and/or --out-image.\n";
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

  const TourPlan tour = BuildProceduralTour(world, startQuery, tourCfg, posterCfg.streetCfg);

  std::cout << "Tour: " << tour.title << "\n";
  std::cout << "Start: " << tour.start.full << "\n";
  std::cout << "Stops: " << tour.stops.size() << "\n";
  std::cout << "Total steps: " << tour.totalSteps << "\n";
  for (std::size_t i = 0; i < tour.stops.size(); ++i) {
    std::cout << "  " << (i + 1) << ") " << tour.stops[i].poi.name << " (" << PoiKindName(tour.stops[i].poi.kind)
              << ")\n";
  }

  std::string err;

  if (!outJson.empty()) {
    if (!EnsureParentDir(outJson)) {
      std::cerr << "Failed to create output directory for: " << outJson << "\n";
      return 2;
    }
    if (!WriteTourJson(outJson, tour, err)) {
      std::cerr << "Failed to write JSON: " << outJson << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  }

  if (!outMd.empty()) {
    if (!EnsureParentDir(outMd)) {
      std::cerr << "Failed to create output directory for: " << outMd << "\n";
      return 2;
    }
    if (!WriteTourMarkdown(outMd, tour, err)) {
      std::cerr << "Failed to write Markdown: " << outMd << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  }

  if (!outImage.empty()) {
    const TourPosterResult poster = RenderTourPoster(world, tour, posterCfg);
    if (!EnsureParentDir(outImage)) {
      std::cerr << "Failed to create output directory for: " << outImage << "\n";
      return 2;
    }
    if (!WritePngRGBA(outImage, poster.image, err)) {
      std::cerr << "Failed to write PNG: " << outImage << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  }

  return 0;
}
