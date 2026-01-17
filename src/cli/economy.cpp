#include "isocity/Economy.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <algorithm>
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
  char* end = nullptr;
  const unsigned long long v = std::strtoull(s.c_str(), &end, 0);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
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

bool requireValue(int& i, std::string& out, int argc, char** argv)
{
  if (i + 1 >= argc) return false;
  out = argv[++i];
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
      << "proc_isocity_economy (macro economy snapshot tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_economy [--load <save.bin>] [--seed <u64>] [--size <WxH>]\n"
      << "                     [--day <N>] [--out <out.json>]\n\n"
      << "World input:\n"
      << "  --load <save.bin>      Load an existing world save\n"
      << "  --seed <u64>           Generate a new world (default: 1)\n"
      << "  --size <WxH>           World size for generation (default: 128x128)\n"
      << "  --day <N>              Day to sample (default: save day, else 0)\n\n"
      << "Economy model settings:\n"
      << "  --enabled <0|1>        Whether to compute the model (default: 1)\n"
      << "  --seed-salt <u64>       Seed salt for alternative economies (default: 0)\n"
      << "  --sectors <N>           Sector count (default: 6)\n"
      << "  --period <days>         Macro cycle period in days (default: 28)\n"
      << "  --event-min <days>      Minimum event duration (default: 3)\n"
      << "  --event-max <days>      Maximum event duration (default: 8)\n"
      << "  --scanback <days>       Event scanback window (default: 16)\n\n"
      << "Output:\n"
      << "  --out <out.json>        Write JSON to a file (otherwise prints to stdout)\n";
}

isocity::JsonValue BuildJson(const isocity::World& world, std::uint64_t hash, int day,
                             const isocity::EconomyModelSettings& settings,
                             const isocity::EconomySnapshot& snap)
{
  using isocity::JsonValue;

  JsonValue root = JsonValue::MakeObject();
  auto add = [&](const std::string& k, JsonValue v) { root.objectValue.emplace_back(k, std::move(v)); };

  add("width", JsonValue::MakeNumber(static_cast<double>(world.width())));
  add("height", JsonValue::MakeNumber(static_cast<double>(world.height())));
  add("seed", JsonValue::MakeNumber(static_cast<double>(world.seed())));
  add("seed_hex", JsonValue::MakeString(HexU64(world.seed())));
  add("hash", JsonValue::MakeString(HexU64(hash)));
  add("day", JsonValue::MakeNumber(static_cast<double>(day)));

  // Settings.
  {
    JsonValue s = JsonValue::MakeObject();
    s.objectValue.emplace_back("enabled", JsonValue::MakeBool(settings.enabled));
    s.objectValue.emplace_back("seed_salt", JsonValue::MakeNumber(static_cast<double>(settings.seedSalt)));
    s.objectValue.emplace_back("seed_salt_hex", JsonValue::MakeString(HexU64(settings.seedSalt)));
    s.objectValue.emplace_back("sectors", JsonValue::MakeNumber(static_cast<double>(settings.sectorCount)));
    s.objectValue.emplace_back("macro_period_days", JsonValue::MakeNumber(static_cast<double>(settings.macroPeriodDays)));
    s.objectValue.emplace_back("event_min_days", JsonValue::MakeNumber(static_cast<double>(settings.minEventDurationDays)));
    s.objectValue.emplace_back("event_max_days", JsonValue::MakeNumber(static_cast<double>(settings.maxEventDurationDays)));
    s.objectValue.emplace_back("event_scanback_days", JsonValue::MakeNumber(static_cast<double>(settings.eventScanbackDays)));
    add("settings", std::move(s));
  }

  // Snapshot.
  add("economy_index", JsonValue::MakeNumber(static_cast<double>(snap.economyIndex)));
  add("inflation", JsonValue::MakeNumber(static_cast<double>(snap.inflation)));
  add("city_wealth", JsonValue::MakeNumber(static_cast<double>(snap.cityWealth)));
  add("active_event_days_left", JsonValue::MakeNumber(static_cast<double>(snap.activeEventDaysLeft)));

  {
    JsonValue ev = JsonValue::MakeObject();
    ev.objectValue.emplace_back("kind", JsonValue::MakeString(isocity::EconomyEventKindName(snap.activeEvent.kind)));
    ev.objectValue.emplace_back("kind_id", JsonValue::MakeNumber(static_cast<double>(static_cast<int>(snap.activeEvent.kind))));
    ev.objectValue.emplace_back("start_day", JsonValue::MakeNumber(static_cast<double>(snap.activeEvent.startDay)));
    ev.objectValue.emplace_back("duration_days", JsonValue::MakeNumber(static_cast<double>(snap.activeEvent.durationDays)));
    ev.objectValue.emplace_back("severity", JsonValue::MakeNumber(static_cast<double>(snap.activeEvent.severity)));
    add("active_event", std::move(ev));
  }

  // Sectors.
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(snap.sectors.size());
    for (std::size_t i = 0; i < snap.sectors.size(); ++i) {
      const isocity::EconomySector& s = snap.sectors[i];
      JsonValue o = JsonValue::MakeObject();
      o.objectValue.emplace_back("index", JsonValue::MakeNumber(static_cast<double>(i)));
      o.objectValue.emplace_back("kind", JsonValue::MakeString(isocity::EconomySectorKindName(s.kind)));
      o.objectValue.emplace_back("kind_id", JsonValue::MakeNumber(static_cast<double>(static_cast<int>(s.kind))));
      o.objectValue.emplace_back("name", JsonValue::MakeString(s.name));
      o.objectValue.emplace_back("industrial_affinity", JsonValue::MakeNumber(static_cast<double>(s.industrialAffinity)));
      o.objectValue.emplace_back("commercial_affinity", JsonValue::MakeNumber(static_cast<double>(s.commercialAffinity)));
      o.objectValue.emplace_back("volatility", JsonValue::MakeNumber(static_cast<double>(s.volatility)));
      arr.arrayValue.emplace_back(std::move(o));
    }
    add("sectors", std::move(arr));
  }

  // District profiles.
  {
    JsonValue arr = JsonValue::MakeArray();
    for (int d = 0; d < isocity::kDistrictCount; ++d) {
      const isocity::DistrictEconomyProfile& p = snap.districts[static_cast<std::size_t>(d)];
      JsonValue o = JsonValue::MakeObject();
      o.objectValue.emplace_back("district", JsonValue::MakeNumber(static_cast<double>(d)));
      o.objectValue.emplace_back("dominant_sector", JsonValue::MakeNumber(static_cast<double>(p.dominantSector)));
      o.objectValue.emplace_back("wealth", JsonValue::MakeNumber(static_cast<double>(p.wealth)));
      o.objectValue.emplace_back("productivity", JsonValue::MakeNumber(static_cast<double>(p.productivity)));
      o.objectValue.emplace_back("tax_base_mult", JsonValue::MakeNumber(static_cast<double>(p.taxBaseMult)));
      o.objectValue.emplace_back("industrial_supply_mult", JsonValue::MakeNumber(static_cast<double>(p.industrialSupplyMult)));
      o.objectValue.emplace_back("commercial_demand_mult", JsonValue::MakeNumber(static_cast<double>(p.commercialDemandMult)));

      // Convenience: include sector kind/name when possible.
      if (p.dominantSector >= 0 && static_cast<std::size_t>(p.dominantSector) < snap.sectors.size()) {
        const isocity::EconomySector& s = snap.sectors[static_cast<std::size_t>(p.dominantSector)];
        o.objectValue.emplace_back("dominant_sector_kind", JsonValue::MakeString(isocity::EconomySectorKindName(s.kind)));
        o.objectValue.emplace_back("dominant_sector_name", JsonValue::MakeString(s.name));
      }

      arr.arrayValue.emplace_back(std::move(o));
    }
    add("districts", std::move(arr));
  }

  return root;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;
  int day = -1;
  std::string outPath;

  EconomyModelSettings settings;
  settings.enabled = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    }
    if (a == "--load") {
      if (!requireValue(i, loadPath, argc, argv)) {
        std::cerr << "Missing value for --load\n";
        return 2;
      }
      continue;
    }
    if (a == "--seed") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseU64(v, &seed)) {
        std::cerr << "Invalid --seed\n";
        return 2;
      }
      continue;
    }
    if (a == "--size") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseSize(v, &w, &h)) {
        std::cerr << "Invalid --size (expected WxH)\n";
        return 2;
      }
      continue;
    }
    if (a == "--day") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseI32(v, &day)) {
        std::cerr << "Invalid --day\n";
        return 2;
      }
      continue;
    }
    if (a == "--out") {
      if (!requireValue(i, outPath, argc, argv)) {
        std::cerr << "Missing value for --out\n";
        return 2;
      }
      continue;
    }

    // Economy model settings.
    if (a == "--enabled") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseBool01(v, &settings.enabled)) {
        std::cerr << "Invalid --enabled (expected 0 or 1)\n";
        return 2;
      }
      continue;
    }
    if (a == "--seed-salt") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseU64(v, &settings.seedSalt)) {
        std::cerr << "Invalid --seed-salt\n";
        return 2;
      }
      continue;
    }
    if (a == "--sectors") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseI32(v, &settings.sectorCount)) {
        std::cerr << "Invalid --sectors\n";
        return 2;
      }
      continue;
    }
    if (a == "--period") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseF32(v, &settings.macroPeriodDays)) {
        std::cerr << "Invalid --period\n";
        return 2;
      }
      continue;
    }
    if (a == "--event-min") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseI32(v, &settings.minEventDurationDays)) {
        std::cerr << "Invalid --event-min\n";
        return 2;
      }
      continue;
    }
    if (a == "--event-max") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseI32(v, &settings.maxEventDurationDays)) {
        std::cerr << "Invalid --event-max\n";
        return 2;
      }
      continue;
    }
    if (a == "--scanback") {
      std::string v;
      if (!requireValue(i, v, argc, argv) || !ParseI32(v, &settings.eventScanbackDays)) {
        std::cerr << "Invalid --scanback\n";
        return 2;
      }
      continue;
    }

    std::cerr << "Unknown arg: " << a << "\n";
    PrintHelp();
    return 2;
  }

  // Sanity clamps.
  settings.sectorCount = std::max(1, settings.sectorCount);
  settings.macroPeriodDays = std::max(1.0f, settings.macroPeriodDays);
  settings.minEventDurationDays = std::max(1, settings.minEventDurationDays);
  settings.maxEventDurationDays = std::max(settings.minEventDurationDays, settings.maxEventDurationDays);
  settings.eventScanbackDays = std::max(0, settings.eventScanbackDays);

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::string err;

  if (!loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n";
      if (!err.empty()) std::cerr << err << "\n";
      return 1;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  if (day < 0) day = world.stats().day;

  EconomySnapshot snap{};
  if (settings.enabled) {
    snap = ComputeEconomySnapshot(world, day, settings);
  } else {
    // Baseline output when disabled.
    snap.day = day;
    snap.economyIndex = 1.0f;
    snap.inflation = 0.0f;
    snap.cityWealth = 0.5f;
    snap.activeEvent.kind = EconomyEventKind::None;
    snap.activeEventDaysLeft = 0;
  }

  // For hashing, include stats since the save may have a non-zero day/money state.
  const std::uint64_t hash = HashWorld(world, true);

  const JsonValue json = BuildJson(world, hash, day, settings, snap);
  JsonWriteOptions opt;
  opt.pretty = true;
  opt.indent = 2;

  if (!outPath.empty()) {
    std::string werr;
    if (!WriteJsonFile(outPath, json, werr, opt)) {
      std::cerr << "Failed to write JSON: " << outPath << "\n";
      if (!werr.empty()) std::cerr << werr << "\n";
      return 1;
    }
    return 0;
  }

  std::string werr;
  if (!WriteJson(std::cout, json, werr, opt)) {
    if (!werr.empty()) std::cerr << werr << "\n";
    return 1;
  }

  return 0;
}
