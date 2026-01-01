#include "isocity/Script.hpp"

#include "isocity/Hash.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <utility>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

static std::string ToLower(std::string s)
{
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

static bool IsIdentStart(unsigned char c)
{
  return (std::isalpha(c) != 0) || c == '_';
}

static bool IsIdentChar(unsigned char c)
{
  return (std::isalnum(c) != 0) || c == '_';
}

static bool IsValidVarName(const std::string& name)
{
  if (name.empty()) return false;
  if (!IsIdentStart(static_cast<unsigned char>(name[0]))) return false;
  for (std::size_t i = 1; i < name.size(); ++i) {
    if (!IsIdentChar(static_cast<unsigned char>(name[i]))) return false;
  }
  return true;
}

static bool IsReservedTemplateToken(const std::string& nameLower)
{
  return nameLower == "seed" || nameLower == "w" || nameLower == "h" || nameLower == "day" || nameLower == "money" || nameLower == "run" || nameLower == "hash";
}

static void PrintHelp()
{
  std::cout
    << "proc_isocity_script - deterministic scenario script runner (headless)\n\n"
    << "Usage:\n"
    << "  proc_isocity_script <script.txt> [options]\n\n"
    << "Options:\n"
    << "  --out <path>        Write a small JSON summary (supports {seed},{day},{w},{h},{money},{run},{hash}).\n"
    << "  --csv <path>        Write per-tick CSV trace from tick/autobuild (supports same tokens).\n"
    << "  --batch N           Run the script N times (fresh state each run).\n"
    << "  --seed <u64>        Initial seed (decimal or 0x...). In batch mode, seed is offset by run index.\n"
    << "  --define k=v        Define a script variable (repeatable). Available as {k} in the script.\n"
    << "  --quiet             Suppress progress output (script `echo` / `hash` still prints).\n"
    << "  -h, --help          Show this help.\n\n"
    << "Script additions:\n"
    << "  set <name> <value>  Set a variable template (expanded via {name}).\n"
    << "  add <name> <delta>  Add integer delta to a variable (supports expressions).\n"
    << "  unset <name>        Remove a variable.\n"
    << "  echo ...            Print expanded text to stdout.\n"
    << "  vars                Print current vars as name=value (expanded).\n";

  std::cout
    << "\nControl flow additions:\n"
    << "  repeat <n>          Repeat a block N times. Terminate the block with `end`.\n"
    << "  while <expr>        Run a block while expr is non-zero. Terminate with `end`.\n"
    << "  if <expr>           Conditional block. Supports optional `else`. Terminate with `end`.\n"
    << "  break               Exit the nearest repeat/while loop.\n"
    << "  continue            Continue the nearest repeat/while loop.\n"
    << "\nAssertions:\n"
    << "  expect <expr>       Fail the script if expr evaluates to zero.\n"
    << "\nExpr operators (C-like): + - * / %  == != < <= > >=  && || !\n";
}

static bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  int base = 10;
  const char* p = s.c_str();
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    base = 16;
    p += 2;
  }

  char* end = nullptr;
  const unsigned long long v = std::strtoull(p, &end, base);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
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

static bool WriteCsvTrace(const std::string& outPath, const std::vector<isocity::Stats>& ticks)
{
  if (outPath.empty()) return true;

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;

  f << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";

  for (const isocity::Stats& s : ticks) {
    f << s.day << ','
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
  }

  return static_cast<bool>(f);
}

static std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static std::string JsonEscape(const std::string& in)
{
  std::ostringstream oss;
  for (unsigned char uc : in) {
    const char c = static_cast<char>(uc);
    switch (c) {
      case '\\': oss << "\\\\"; break;
      case '"': oss << "\\\""; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default:
        if (uc < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(uc) << std::dec;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
}

static bool WriteJsonSummary(const std::string& outPath, const std::string& scriptPath, const isocity::World& world,
                            std::uint64_t hash, int runIndex)
{
  if (outPath.empty()) return true;

  const isocity::Stats& s = world.stats();

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"script\": \"" << JsonEscape(scriptPath) << "\",\n";
  oss << "  \"run\": " << runIndex << ",\n";
  oss << "  \"width\": " << world.width() << ",\n";
  oss << "  \"height\": " << world.height() << ",\n";
  oss << "  \"seed\": " << world.seed() << ",\n";
  oss << "  \"hash\": \"" << HexU64(hash) << "\",\n";
  oss << "  \"stats\": {\n";
  oss << "    \"day\": " << s.day << ",\n";
  oss << "    \"population\": " << s.population << ",\n";
  oss << "    \"money\": " << s.money << ",\n";
  oss << "    \"happiness\": " << s.happiness << "\n";
  oss << "  }\n";
  oss << "}\n";

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

struct Options {
  std::string scriptPath;
  std::string outJson;
  std::string outCsv;
  bool quiet = false;

  int batch = 1;
  bool hasSeed = false;
  std::uint64_t seed = 1;

  std::vector<std::pair<std::string, std::string>> defines;
};

static bool ParseDefineKV(const std::string& s, std::string* outK, std::string* outV)
{
  if (!outK || !outV) return false;
  const std::size_t eq = s.find('=');
  if (eq == std::string::npos) return false;
  const std::string k = s.substr(0, eq);
  const std::string v = s.substr(eq + 1);
  if (k.empty()) return false;
  *outK = k;
  *outV = v;
  return true;
}

static bool RunOnce(const Options& opt, int runIndex)
{
  isocity::ScriptRunner runner;
  runner.setOptions(isocity::ScriptRunOptions{.quiet = opt.quiet});

  runner.setCallbacks(isocity::ScriptCallbacks{
    .print = [](const std::string& line) { std::cout << line << "\n"; },
    .info = [](const std::string& line) { std::cout << line << "\n"; },
    .error = [](const std::string& line) { std::cerr << line << "\n"; },
  });

  runner.state().runIndex = runIndex;

  // Seed policy:
  //  - If --seed is provided: seed = base + runIndex
  //  - Else if --batch>1:     seed = 1 + runIndex (to avoid accidental overwrites)
  if (opt.hasSeed) {
    runner.state().seed = opt.seed + static_cast<std::uint64_t>(runIndex);
  } else if (opt.batch > 1) {
    runner.state().seed = 1u + static_cast<std::uint64_t>(runIndex);
  }

  // Apply CLI-defined vars (stored as raw templates; expanded by ScriptRunner on use).
  for (const auto& kv : opt.defines) {
    runner.state().vars[ToLower(kv.first)] = kv.second;
  }

  if (!runner.runFile(opt.scriptPath)) {
    // ScriptRunner already emitted the formatted error.
    return false;
  }

  if (!runner.state().hasWorld) {
    std::cerr << "script completed but no world was generated/loaded\n";
    return false;
  }

  // Ensure derived stats are fresh.
  if (runner.state().dirtyDerived) {
    runner.state().sim.config() = runner.state().simCfg;
    runner.state().sim.refreshDerivedStats(runner.state().world);
    runner.state().dirtyDerived = false;
  }

  const std::uint64_t hash = isocity::HashWorld(runner.state().world, true);

  if (!opt.quiet) {
    const isocity::Stats& s = runner.state().world.stats();
    std::cout << "done: run=" << runIndex
              << " " << runner.state().world.width() << "x" << runner.state().world.height()
              << " seed=" << runner.state().world.seed()
              << " day=" << s.day
              << " pop=" << s.population
              << " money=" << s.money
              << " hash=" << HexU64(hash)
              << "\n";
  }

  const std::string outJsonPath = runner.expandPathTemplate(opt.outJson, runIndex);
  if (!WriteJsonSummary(outJsonPath, opt.scriptPath, runner.state().world, hash, runIndex)) {
    std::cerr << "failed to write json summary: " << outJsonPath << "\n";
    return false;
  }

  const std::string outCsvPath = runner.expandPathTemplate(opt.outCsv, runIndex);
  if (!WriteCsvTrace(outCsvPath, runner.state().tickStats)) {
    std::cerr << "failed to write csv: " << outCsvPath << "\n";
    return false;
  }

  return true;
}

} // namespace

int main(int argc, char** argv)
{
  Options opt;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();

    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }

    if (arg == "--quiet") {
      opt.quiet = true;
      continue;
    }

    if (arg == "--out" || arg == "--json") {
      if (i + 1 >= argc) {
        std::cerr << arg << " requires a path\n";
        return 2;
      }
      opt.outJson = argv[++i];
      continue;
    }

    if (arg == "--csv") {
      if (i + 1 >= argc) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      opt.outCsv = argv[++i];
      continue;
    }

    if (arg == "--batch") {
      if (i + 1 >= argc) {
        std::cerr << "--batch requires N\n";
        return 2;
      }
      int n = 0;
      if (!ParseI32(argv[++i], &n) || n <= 0) {
        std::cerr << "--batch expects a positive integer\n";
        return 2;
      }
      opt.batch = n;
      continue;
    }

    if (arg == "--seed") {
      if (i + 1 >= argc) {
        std::cerr << "--seed requires a u64\n";
        return 2;
      }
      std::uint64_t v = 0;
      if (!ParseU64(argv[++i], &v)) {
        std::cerr << "--seed expects u64 (decimal or 0x...)\n";
        return 2;
      }
      opt.hasSeed = true;
      opt.seed = v;
      continue;
    }

    if (arg == "--define") {
      if (i + 1 >= argc) {
        std::cerr << "--define requires k=v\n";
        return 2;
      }
      std::string k, v;
      if (!ParseDefineKV(argv[++i], &k, &v)) {
        std::cerr << "--define expects k=v\n";
        return 2;
      }

      k = ToLower(std::move(k));
      if (!IsValidVarName(k)) {
        std::cerr << "--define has invalid variable name: " << k << "\n";
        return 2;
      }
      if (IsReservedTemplateToken(k)) {
        std::cerr << "--define may not override reserved token: " << k << "\n";
        return 2;
      }

      opt.defines.emplace_back(std::move(k), std::move(v));
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    }

    if (opt.scriptPath.empty()) {
      opt.scriptPath = arg;
    } else {
      std::cerr << "unexpected positional arg: " << arg << "\n";
      return 2;
    }
  }

  if (opt.scriptPath.empty()) {
    PrintHelp();
    return 2;
  }

  for (int run = 0; run < opt.batch; ++run) {
    if (!RunOnce(opt, run)) {
      return 1;
    }
  }

  return 0;
}
