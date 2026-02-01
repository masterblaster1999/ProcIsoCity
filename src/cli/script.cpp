#include "isocity/Script.hpp"

#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/StatsCsv.hpp"

#include "cli/CliParse.hpp"

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
  return isocity::cli::ParseU64(s, out);
}

static bool ParseI32(const std::string& s, int* out)
{
  return isocity::cli::ParseI32(s, out);
}

static bool WriteCsvTrace(const std::string& outPath, const std::vector<isocity::Stats>& ticks)
{
  if (outPath.empty()) return true;

  std::ofstream f(outPath, std::ios::binary);
  if (!f) return false;

  if (!isocity::WriteStatsCsvHeader(f)) return false;
  for (const isocity::Stats& s : ticks) {
    if (!isocity::WriteStatsCsvRow(f, s)) return false;
  }

  return static_cast<bool>(f);
}

static std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static bool WriteJsonSummary(const std::string& outPath, const std::string& scriptPath, const isocity::World& world,
                            std::uint64_t hash, int runIndex)
{
  if (outPath.empty()) return true;

  const isocity::Stats& s = world.stats();

  using isocity::JsonValue;
  JsonValue root = JsonValue::MakeObject();
  auto add = [](JsonValue& obj, const char* key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

  add(root, "script", JsonValue::MakeString(scriptPath));
  add(root, "run", JsonValue::MakeNumber(static_cast<double>(runIndex)));
  add(root, "width", JsonValue::MakeNumber(static_cast<double>(world.width())));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(world.height())));
  add(root, "seed", JsonValue::MakeNumber(static_cast<double>(world.seed())));
  add(root, "hash", JsonValue::MakeString(HexU64(hash)));

  JsonValue st = JsonValue::MakeObject();
  add(st, "day", JsonValue::MakeNumber(static_cast<double>(s.day)));
  add(st, "population", JsonValue::MakeNumber(static_cast<double>(s.population)));
  add(st, "money", JsonValue::MakeNumber(static_cast<double>(s.money)));
  add(st, "happiness", JsonValue::MakeNumber(static_cast<double>(s.happiness)));
  add(root, "stats", std::move(st));

  std::string err;
  return isocity::WriteJsonFile(outPath, root, err, isocity::JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false});
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
