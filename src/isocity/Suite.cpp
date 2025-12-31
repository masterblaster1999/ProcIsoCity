#include "isocity/Suite.hpp"

#include "isocity/Hash.hpp"
#include "isocity/Replay.hpp"
#include "isocity/Script.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace isocity {

namespace {

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static std::string Trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

// Parse a token (either a bare word or a quoted string) from `s` starting at `i`.
// Advances i and returns the token. On failure returns empty string.
static std::string ParseToken(const std::string& s, std::size_t& i)
{
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  if (i >= s.size()) return {};

  if (s[i] == '"') {
    ++i;
    std::string out;
    while (i < s.size()) {
      const char c = s[i++];
      if (c == '"') break;
      if (c == '\\' && i < s.size()) {
        // Minimal escape handling for \" and \\.
        const char n = s[i++];
        if (n == '"' || n == '\\') out.push_back(n);
        else {
          out.push_back('\\');
          out.push_back(n);
        }
      } else {
        out.push_back(c);
      }
    }
    return out;
  }

  const std::size_t start = i;
  while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  return s.substr(start, i - start);
}

static void NormalizeExts(std::vector<std::string>& exts)
{
  for (std::string& e : exts) {
    e = ToLower(Trim(e));
    if (!e.empty() && e[0] != '.') e = "." + e;
  }
  exts.erase(std::remove_if(exts.begin(), exts.end(), [](const std::string& e) { return e.empty(); }), exts.end());
}

static bool ExtMatches(const std::string& ext, const std::vector<std::string>& exts)
{
  const std::string e = ToLower(ext);
  for (const std::string& x : exts) {
    if (e == x) return true;
  }
  return false;
}

} // namespace

ScenarioKind GuessScenarioKindFromPath(const std::string& path)
{
  const std::filesystem::path p(path);
  const std::string ext = ToLower(p.extension().string());
  if (ext == ".isoreplay") return ScenarioKind::Replay;
  return ScenarioKind::Script;
}

bool RunScenario(const ScenarioCase& sc, const ScenarioRunOptions& opt, ScenarioRunOutputs& out, std::string& outError)
{
  outError.clear();
  out = ScenarioRunOutputs{};

  if (sc.path.empty()) {
    outError = "scenario path is empty";
    return false;
  }

  const ScenarioKind kind = sc.kind;
  if (kind == ScenarioKind::Script) {
    ScriptRunner runner;
    ScriptRunOptions sopt;
    sopt.quiet = opt.quiet;
    runner.setOptions(sopt);

    runner.state().runIndex = opt.runIndex;

    // Inject vars.
    for (const auto& kv : opt.scriptVars) {
      const std::string k = ToLower(kv.first);
      if (k.empty()) continue;
      runner.state().vars[k] = kv.second;
    }

    if (!runner.runFile(sc.path)) {
      std::ostringstream oss;
      oss << runner.lastErrorPath() << ":" << runner.lastErrorLine() << ": " << runner.lastError();
      outError = oss.str();
      return false;
    }
    if (!runner.state().hasWorld) {
      outError = "script completed without generating/loading a world";
      return false;
    }

    out.world = std::move(runner.state().world);
    out.procCfg = runner.state().procCfg;
    out.simCfg = runner.state().simCfg;
    out.tickStats = std::move(runner.state().tickStats);
    out.finalHash = HashWorld(out.world, true);
    return true;
  }

  if (kind == ScenarioKind::Replay) {
    Replay replay;
    std::string err;
    if (!LoadReplayBinary(replay, sc.path, err)) {
      outError = "LoadReplayBinary failed: " + err;
      return false;
    }

    ProcGenConfig procCfg;
    SimConfig simCfg;
    World world;
    std::vector<Stats> ticks;
    if (!PlayReplay(replay, world, procCfg, simCfg, err, opt.strictReplayPatches, opt.strictReplayAsserts, &ticks)) {
      outError = "PlayReplay failed: " + err;
      return false;
    }

    out.world = std::move(world);
    out.procCfg = procCfg;
    out.simCfg = simCfg;
    out.tickStats = std::move(ticks);
    out.finalHash = HashWorld(out.world, true);
    return true;
  }

  outError = "unknown scenario kind";
  return false;
}

bool LoadScenarioManifest(const std::string& manifestPath, std::vector<ScenarioCase>& out, std::string& outError)
{
  outError.clear();
  out.clear();

  std::ifstream f(manifestPath);
  if (!f) {
    outError = "failed to open manifest: " + manifestPath;
    return false;
  }

  std::string line;
  int lineNo = 0;
  while (std::getline(f, line)) {
    ++lineNo;
    const std::string t = Trim(line);
    if (t.empty()) continue;
    if (!t.empty() && t[0] == '#') continue;

    std::size_t i = 0;
    const std::string tok0 = ParseToken(t, i);
    if (tok0.empty()) continue;

    std::string kindTok = ToLower(tok0);
    ScenarioKind kind = ScenarioKind::Script;
    std::string path;

    if (kindTok == "script" || kindTok == "sc" || kindTok == "isocity") {
      kind = ScenarioKind::Script;
      path = ParseToken(t, i);
    } else if (kindTok == "replay" || kindTok == "rp" || kindTok == "isoreplay") {
      kind = ScenarioKind::Replay;
      path = ParseToken(t, i);
    } else {
      // No explicit kind; treat tok0 as a path.
      path = tok0;
      kind = GuessScenarioKindFromPath(path);
    }

    path = Trim(path);
    if (path.empty()) {
      outError = manifestPath + ":" + std::to_string(lineNo) + ": expected a path";
      return false;
    }

    ScenarioCase sc;
    sc.path = path;
    sc.kind = kind;
    out.push_back(std::move(sc));
  }

  return true;
}

bool DiscoverScenarios(const std::string& rootDir, const std::vector<std::string>& extsIn, std::vector<ScenarioCase>& out,
                      std::string& outError)
{
  outError.clear();
  out.clear();

  namespace fs = std::filesystem;

  std::vector<std::string> exts = extsIn;
  if (exts.empty()) {
    exts = {".isocity", ".isoreplay"};
  }
  NormalizeExts(exts);

  std::error_code ec;
  if (!fs::exists(rootDir, ec)) {
    outError = "discover root does not exist: " + rootDir;
    return false;
  }

  for (fs::recursive_directory_iterator it(rootDir, ec), end; it != end && !ec; it.increment(ec)) {
    const fs::directory_entry& e = *it;
    if (!e.is_regular_file(ec)) continue;
    const fs::path p = e.path();
    const std::string ext = p.extension().string();
    if (!ExtMatches(ext, exts)) continue;
    ScenarioCase sc;
    sc.path = p.string();
    sc.kind = GuessScenarioKindFromPath(sc.path);
    out.push_back(std::move(sc));
  }

  if (ec) {
    outError = "discover failed: " + ec.message();
    return false;
  }

  std::sort(out.begin(), out.end(), [](const ScenarioCase& a, const ScenarioCase& b) { return a.path < b.path; });
  return true;
}

} // namespace isocity
