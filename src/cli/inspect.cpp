#include "isocity/SaveLoad.hpp"
#include "isocity/Json.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_inspect (headless save inspector)\n\n"
      << "Usage:\n"
      << "  proc_isocity_inspect <save.bin> [options]\n\n"
      << "Options:\n"
      << "  --verify-crc            Verify CRC for v3+ saves (slower, but detects corruption).\n"
      << "  --json <out.json>        Write a JSON summary (same info as stdout).\n"
      << "  --quiet                 Suppress stdout output (errors still print).\n"
      << "  -h, --help              Show this help.\n";
}

static std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static bool WriteJson(const std::string& outPath, const std::string& inPath, const SaveSummary& s)
{
  using isocity::JsonValue;
  JsonValue root = JsonValue::MakeObject();
  auto add = [](JsonValue& obj, const char* key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

  add(root, "file", JsonValue::MakeString(inPath));
  add(root, "version", JsonValue::MakeNumber(static_cast<double>(s.version)));
  add(root, "width", JsonValue::MakeNumber(static_cast<double>(s.width)));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(s.height)));
  add(root, "seed", JsonValue::MakeNumber(static_cast<double>(s.seed)));

  add(root, "hasProcCfg", JsonValue::MakeBool(s.hasProcCfg));
  add(root, "hasStats", JsonValue::MakeBool(s.hasStats));
  add(root, "hasSimCfg", JsonValue::MakeBool(s.hasSimCfg));

  if (s.hasStats) {
    JsonValue st = JsonValue::MakeObject();
    add(st, "day", JsonValue::MakeNumber(static_cast<double>(s.stats.day)));
    add(st, "population", JsonValue::MakeNumber(static_cast<double>(s.stats.population)));
    add(st, "housingCapacity", JsonValue::MakeNumber(static_cast<double>(s.stats.housingCapacity)));
    add(st, "jobsCapacity", JsonValue::MakeNumber(static_cast<double>(s.stats.jobsCapacity)));
    add(st, "employed", JsonValue::MakeNumber(static_cast<double>(s.stats.employed)));
    add(st, "happiness", JsonValue::MakeNumber(static_cast<double>(s.stats.happiness)));
    add(st, "money", JsonValue::MakeNumber(static_cast<double>(s.stats.money)));
    add(root, "stats", std::move(st));
  }

  add(root, "crcChecked", JsonValue::MakeBool(s.crcChecked));
  add(root, "crcOk", JsonValue::MakeBool(s.crcOk));

  std::string err;
  return isocity::WriteJsonFile(outPath, root, err, isocity::JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false});
}

static void PrintSummary(const std::string& path, const SaveSummary& s)
{
  std::cout << "file: " << path << "\n";
  std::cout << "version: v" << s.version << "\n";
  std::cout << "size: " << s.width << "x" << s.height << "\n";
  std::cout << "seed: " << s.seed << " (" << HexU64(s.seed) << ")\n";

  if (s.hasStats) {
    std::cout << "day: " << s.stats.day << "\n";
    std::cout << "population: " << s.stats.population << "\n";
    std::cout << "money: " << s.stats.money << "\n";
    std::cout << "happiness: " << s.stats.happiness << "\n";
    std::cout << "housingCapacity: " << s.stats.housingCapacity << "\n";
    std::cout << "jobsCapacity: " << s.stats.jobsCapacity << "\n";
  }

  std::cout << "hasProcCfg: " << (s.hasProcCfg ? "true" : "false") << "\n";
  std::cout << "hasSimCfg: " << (s.hasSimCfg ? "true" : "false") << "\n";

  if (s.crcChecked) {
    std::cout << "crc: " << (s.crcOk ? "OK" : "BAD") << "\n";
  }
}

} // namespace

int main(int argc, char** argv)
{
  std::string path;
  bool verifyCrc = false;
  bool quiet = false;
  std::string outJson;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }
    if (arg == "--verify-crc") {
      verifyCrc = true;
      continue;
    }
    if (arg == "--quiet") {
      quiet = true;
      continue;
    }
    if (arg == "--json") {
      if (i + 1 >= argc) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJson = argv[++i];
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    }

    if (path.empty()) {
      path = arg;
    } else {
      std::cerr << "unexpected positional arg: " << arg << "\n";
      return 2;
    }
  }

  if (path.empty()) {
    PrintHelp();
    return 2;
  }

  SaveSummary s;
  std::string err;
  if (!ReadSaveSummary(path, s, err, verifyCrc)) {
    std::cerr << "failed to read save summary: " << err << "\n";
    return 1;
  }

  if (!quiet) {
    PrintSummary(path, s);
  }

  if (!outJson.empty()) {
    if (!WriteJson(outJson, path, s)) {
      std::cerr << "failed to write JSON: " << outJson << "\n";
      return 1;
    }
  }

  return 0;
}
