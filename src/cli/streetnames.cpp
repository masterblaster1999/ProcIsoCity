#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/StreetNames.hpp"

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

static std::string CsvEscape(const std::string& s)
{
  bool need = false;
  for (char c : s) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r') {
      need = true;
      break;
    }
  }
  if (!need) return s;
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('"');
  for (char c : s) {
    if (c == '"') out.push_back('"');
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_streetnames (headless street naming + parcel addressing)\n\n"
      << "Generates a deterministic street naming layer from the road network and assigns\n"
      << "simple addresses to zone building parcels. This is derived data (not persisted).\n\n"
      << "Usage:\n"
      << "  proc_isocity_streetnames [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                        [--streets-json <out.json>] [--addresses-csv <out.csv>]\n"
      << "                        [--road-tiles-csv <out.csv>]\n"
      << "                        [--merge-intersections <0|1>] [--merge-corners <0|1>]\n"
      << "                        [--ordinals <0|1>] [--number-step <N>]\n\n"
      << "Notes:\n"
      << "  - If --load is omitted, a world is generated from (--seed, --size).\n"
      << "  - --road-tiles-csv can be very large on big maps.\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string streetsJsonPath;
  std::string addressesCsvPath;
  std::string roadTilesCsvPath;

  std::uint64_t seed = 1;
  int w = 96;
  int h = 96;

  StreetNamingConfig scfg;
  AddressConfig acfg;

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
    } else if (arg == "--streets-json") {
      if (!requireValue(i, val)) {
        std::cerr << "--streets-json requires a path\n";
        return 2;
      }
      streetsJsonPath = val;
    } else if (arg == "--addresses-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--addresses-csv requires a path\n";
        return 2;
      }
      addressesCsvPath = val;
    } else if (arg == "--road-tiles-csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--road-tiles-csv requires a path\n";
        return 2;
      }
      roadTilesCsvPath = val;
    } else if (arg == "--merge-intersections") {
      if (!requireValue(i, val) || !ParseBool01(val, &scfg.mergeThroughIntersections)) {
        std::cerr << "--merge-intersections requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--merge-corners") {
      if (!requireValue(i, val) || !ParseBool01(val, &scfg.mergeThroughCorners)) {
        std::cerr << "--merge-corners requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--ordinals") {
      if (!requireValue(i, val) || !ParseBool01(val, &scfg.allowOrdinalNames)) {
        std::cerr << "--ordinals requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--number-step") {
      if (!requireValue(i, val) || !ParseI32(val, &acfg.numberStep) || acfg.numberStep <= 0) {
        std::cerr << "--number-step requires a positive integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
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

  const StreetNamingResult streets = BuildStreetNames(world, scfg);
  const std::vector<ParcelAddress> addrs = BuildParcelAddresses(world, streets, acfg);

  std::cout << "StreetNames\n";
  std::cout << "  streets:   " << streets.streets.size() << "\n";
  std::cout << "  addresses: " << addrs.size() << "\n";

  // --- Streets JSON ---
  if (!streetsJsonPath.empty()) {
    std::ofstream os(streetsJsonPath);
    if (!os) {
      std::cerr << "Failed to open streets json: " << streetsJsonPath << "\n";
      return 2;
    }

    JsonWriteOptions opt;
    opt.pretty = true;
    opt.sortKeys = true;

    JsonWriter jw(os, opt);
    jw.beginObject();
    jw.key("seed");
    jw.uintValue(world.seed());
    jw.key("width");
    jw.intValue(world.width());
    jw.key("height");
    jw.intValue(world.height());
    jw.key("streets");
    jw.beginArray();
    for (const StreetInfo& s : streets.streets) {
      jw.beginObject();
      jw.key("id");
      jw.intValue(s.id);
      jw.key("name");
      jw.stringValue(s.name);
      jw.key("roadLevel");
      jw.intValue(s.roadLevel);
      jw.key("edgeCount");
      jw.intValue(s.edgeCount);
      jw.key("tileCount");
      jw.intValue(s.tileCount);
      jw.key("axis");
      jw.stringValue((s.axis == 0) ? "x" : "y");
      jw.key("axisMin");
      jw.intValue(s.axisMin);
      jw.key("bbox");
      jw.beginArray();
      jw.intValue(s.bboxMin.x);
      jw.intValue(s.bboxMin.y);
      jw.intValue(s.bboxMax.x);
      jw.intValue(s.bboxMax.y);
      jw.endArray();
      jw.endObject();
    }
    jw.endArray();
    jw.endObject();

    if (!jw.ok()) {
      std::cerr << "Failed to write JSON: " << jw.error() << "\n";
      return 2;
    }
  }

  // --- Addresses CSV ---
  if (!addressesCsvPath.empty()) {
    std::ofstream os(addressesCsvPath);
    if (!os) {
      std::cerr << "Failed to open addresses csv: " << addressesCsvPath << "\n";
      return 2;
    }
    os << "parcelIndex,streetId,houseNumber,streetName,full,roadX,roadY,anchorX,anchorY\n";
    for (const ParcelAddress& a : addrs) {
      os << a.parcelIndex << "," << a.streetId << "," << a.houseNumber << ","
         << CsvEscape(a.streetName) << "," << CsvEscape(a.full) << ","
         << a.roadTile.x << "," << a.roadTile.y << "," << a.parcelAnchor.x << "," << a.parcelAnchor.y << "\n";
    }
  }

  // --- Road tile mapping CSV (optional; huge) ---
  if (!roadTilesCsvPath.empty()) {
    std::ofstream os(roadTilesCsvPath);
    if (!os) {
      std::cerr << "Failed to open road tiles csv: " << roadTilesCsvPath << "\n";
      return 2;
    }
    os << "x,y,streetId,streetName\n";
    const int ww = world.width();
    const int hh = world.height();
    for (int y = 0; y < hh; ++y) {
      for (int x = 0; x < ww; ++x) {
        if (world.at(x, y).overlay != Overlay::Road) continue;
        const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(ww) + static_cast<std::size_t>(x);
        if (idx >= streets.roadTileToStreetId.size()) continue;
        const int sid = streets.roadTileToStreetId[idx];
        const std::string name = (sid >= 0 && static_cast<std::size_t>(sid) < streets.streets.size())
                                     ? streets.streets[static_cast<std::size_t>(sid)].name
                                     : std::string();
        os << x << "," << y << "," << sid << "," << CsvEscape(name) << "\n";
      }
    }
  }

  return 0;
}
