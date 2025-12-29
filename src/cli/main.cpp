#include "isocity/Hash.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

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

  const std::string a = s.substr(0, pos);
  const std::string b = s.substr(pos + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
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
      << "proc_isocity_cli (headless simulation runner)\n\n"
      << "Usage:\n"
      << "  proc_isocity_cli [--load <save.bin>] [--seed <u64>] [--size <WxH>] [--days <N>]\n"
      << "                 [--out <summary.json>] [--csv <ticks.csv>] [--save <save.bin>]\n"
      << "                 [--require-outside <0|1>] [--tax-res <N>] [--tax-com <N>] [--tax-ind <N>]\n"
      << "                 [--maint-road <N>] [--maint-park <N>]\n\n"
      << "Notes:\n"
      << "  - If --load is provided, the world + ProcGenConfig + SimConfig are loaded from the save.\n"
      << "  - Otherwise, a new world is generated from (--seed, --size).\n"
      << "  - --days advances the simulator by N ticks via Simulator::stepOnce().\n"
      << "  - A stable 64-bit hash of the final world is included in the JSON output.\n";
}

bool WriteJsonSummary(const isocity::World& world, std::uint64_t hash, const std::string& outPath)
{
  const isocity::Stats& s = world.stats();

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"width\": " << world.width() << ",\n";
  oss << "  \"height\": " << world.height() << ",\n";
  oss << "  \"seed\": " << world.seed() << ",\n";
  oss << "  \"hash\": \"" << HexU64(hash) << "\",\n";
  oss << "  \"stats\": {\n";
  oss << "    \"day\": " << s.day << ",\n";
  oss << "    \"population\": " << s.population << ",\n";
  oss << "    \"housingCapacity\": " << s.housingCapacity << ",\n";
  oss << "    \"jobsCapacity\": " << s.jobsCapacity << ",\n";
  oss << "    \"jobsCapacityAccessible\": " << s.jobsCapacityAccessible << ",\n";
  oss << "    \"employed\": " << s.employed << ",\n";
  oss << "    \"happiness\": " << s.happiness << ",\n";
  oss << "    \"money\": " << s.money << ",\n";
  oss << "    \"roads\": " << s.roads << ",\n";
  oss << "    \"parks\": " << s.parks << ",\n";
  oss << "    \"avgCommuteTime\": " << s.avgCommuteTime << ",\n";
  oss << "    \"trafficCongestion\": " << s.trafficCongestion << ",\n";
  oss << "    \"goodsDemand\": " << s.goodsDemand << ",\n";
  oss << "    \"goodsDelivered\": " << s.goodsDelivered << ",\n";
  oss << "    \"goodsSatisfaction\": " << s.goodsSatisfaction << ",\n";
  oss << "    \"avgLandValue\": " << s.avgLandValue << ",\n";
  oss << "    \"demandResidential\": " << s.demandResidential << "\n";
  oss << "  }\n";
  oss << "}\n";

  if (outPath.empty()) {
    std::cout << oss.str();
    return true;
  }

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

bool WriteCsvRow(std::ostream& os, const isocity::Stats& s)
{
  os << s.day << ','
     << s.population << ','
     << s.money << ','
     << s.housingCapacity << ','
     << s.jobsCapacity << ','
     << s.jobsCapacityAccessible << ','
     << s.employed << ','
     << s.happiness << ','
     << s.roads << ','
     << s.parks << ','
     << s.avgCommuteTime << ','
     << s.trafficCongestion << ','
     << s.goodsDemand << ','
     << s.goodsDelivered << ','
     << s.goodsSatisfaction << ','
     << s.avgLandValue << ','
     << s.demandResidential
     << '\n';
  return static_cast<bool>(os);
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string savePath;
  std::string outJson;
  std::string outCsv;

  std::uint64_t seed = 1;
  bool seedProvided = false;

  int w = 96;
  int h = 96;
  int days = 0;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};

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
    } else if (arg == "--save") {
      if (!requireValue(i, val)) {
        std::cerr << "--save requires a path\n";
        return 2;
      }
      savePath = val;
    } else if (arg == "--out" || arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << arg << " requires a path\n";
        return 2;
      }
      outJson = val;
    } else if (arg == "--csv") {
      if (!requireValue(i, val)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      outCsv = val;
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
      seedProvided = true;
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &w, &h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--days" || arg == "--ticks") {
      if (!requireValue(i, val) || !ParseI32(val, &days) || days < 0) {
        std::cerr << arg << " requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--require-outside") {
      if (!requireValue(i, val)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      int b = 0;
      if (!ParseI32(val, &b) || (b != 0 && b != 1)) {
        std::cerr << "--require-outside requires 0 or 1\n";
        return 2;
      }
      simCfg.requireOutsideConnection = (b != 0);
    } else if (arg == "--tax-res") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxResidential)) {
        std::cerr << "--tax-res requires an integer\n";
        return 2;
      }
    } else if (arg == "--tax-com") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxCommercial)) {
        std::cerr << "--tax-com requires an integer\n";
        return 2;
      }
    } else if (arg == "--tax-ind") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.taxIndustrial)) {
        std::cerr << "--tax-ind requires an integer\n";
        return 2;
      }
    } else if (arg == "--maint-road") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenanceRoad)) {
        std::cerr << "--maint-road requires an integer\n";
        return 2;
      }
    } else if (arg == "--maint-park") {
      if (!requireValue(i, val) || !ParseI32(val, &simCfg.maintenancePark)) {
        std::cerr << "--maint-park requires an integer\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown argument: " << arg << "\n\n";
      PrintHelp();
      return 2;
    }
  }

  World world;
  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << err << "\n";
      return 1;
    }
  } else {
    // When no seed is provided, keep a deterministic default so CI runs are stable.
    // Users can pass --seed to randomize externally.
    if (!seedProvided) seed = 1;
    world = GenerateWorld(w, h, seed, procCfg);
  }

  Simulator sim(simCfg);
  sim.refreshDerivedStats(world);

  std::ofstream csv;
  if (!outCsv.empty()) {
    csv.open(outCsv, std::ios::binary);
    if (!csv) {
      std::cerr << "Failed to open CSV for writing: " << outCsv << "\n";
      return 1;
    }
    csv << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
    WriteCsvRow(csv, world.stats());
  }

  for (int i = 0; i < days; ++i) {
    sim.stepOnce(world);
    if (csv) WriteCsvRow(csv, world.stats());
  }

  sim.refreshDerivedStats(world);

  if (!savePath.empty()) {
    std::string err;
    if (!SaveWorldBinary(world, procCfg, sim.config(), savePath, err)) {
      std::cerr << "Failed to save world: " << err << "\n";
      return 1;
    }
  }

  const std::uint64_t hash = HashWorld(world, true);
  if (!WriteJsonSummary(world, hash, outJson)) {
    std::cerr << "Failed to write JSON summary" << (outJson.empty() ? "" : (": " + outJson)) << "\n";
    return 1;
  }

  return 0;
}
