// proc_isocity_mine
//
// "City mining" / seed search utility.
//
// This tool is a thin CLI wrapper around the core mining engine
// (see isocity/SeedMiner.*). Keeping the mining logic in isocity_core allows
// the interactive app to reuse it directly (e.g., a future in-game "City Lab"
// panel) and keeps headless behavior deterministic.

#include "isocity/ConfigIO.hpp"
#include "isocity/Json.hpp"
#include "isocity/SeedMiner.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

namespace fs = std::filesystem;

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_mine (seed search / city mining)\n\n"
      << "Runs many procedural worlds, simulates them, computes KPI metrics, and ranks seeds.\n\n"
      << "Usage:\n"
      << "  proc_isocity_mine [options]\n\n"
      << "Core options:\n"
      << "  --seed-start <u64>       First seed to test (default: 1).\n"
      << "  --seed-step <u64>        Step added each sample (default: 1).\n"
      << "  --samples <N>            Number of seeds to test (default: 100).\n"
      << "  --size <WxH>             World size (default: 96x96).\n"
      << "  --days <N>               Simulation days per seed (default: 120).\n"
      << "  --objective <name>       Ranking objective: balanced|growth|resilient|chaos (default: balanced).\n"
      << "\nOutputs:\n"
      << "  --csv <out.csv>          Write all results as CSV (default: mine.csv).\n"
      << "  --json <out.json>        Write a JSON summary (top seeds + configs).\n"
      << "  --manifest <out.txt>     Write selected top seeds (one per line).\n"
      << "  --top <K>                Number of seeds to select/print (default: 10).\n"
      << "  --diverse <0|1>          Diversify the top-K selection (default: 1).\n"
      << "  --candidate-pool <N>     Candidate pool size used for diversity (default: max(50,10*K)).\n"
      << "  --mmr-score-weight <F>   Diversity tradeoff in [0,1] (default: 0.70).\n"
      << "\nMulti-objective (Pareto/NSGA-II) selection (optional):\n"
      << "  --pareto <0|1>           Enable Pareto selection instead of scalar score ranking (default: 0).\n"
      << "  --pareto-max <list>      Comma-separated metrics to maximize (default: population,happiness,avg_land_value).\n"
      << "  --pareto-min <list>      Comma-separated metrics to minimize (default: traffic_congestion[,flood_risk]).\n"
      << "  --pareto-crowding <0|1>  Use crowding distance within fronts (default: 1).\n"
      << "\n  Metric names:\n"
      << "    population,happiness,money,avg_land_value,traffic_congestion,goods_satisfaction,services_overall_satisfaction,\n"
      << "    water_frac,road_frac,zone_frac,park_frac,sea_flood_frac,sea_max_depth,pond_frac,pond_max_depth,pond_volume,\n"
      << "    flood_risk,score\n"
      << "\nConfig loading (optional):\n"
      << "  --config <combined.json> Load {proc:{...},sim:{...}} (overrides defaults).\n"
      << "  --proc <proc.json>       Load ProcGenConfig JSON overrides.\n"
      << "  --sim <sim.json>         Load SimConfig JSON overrides.\n"
      << "\nHydrology metrics (optional, default on):\n"
      << "  --hydro <0|1>            Enable/disable sea + ponding metrics (default: 1).\n"
      << "  --sea-level <F>          Sea level height threshold (default: proc.water_level).\n"
      << "  --sea-edge <0|1>         Require edge-connected flooding (default: 1).\n"
      << "  --sea-8conn <0|1>        Use 8-connected flooding (default: 0).\n"
      << "  --dep-eps <F>            Priority-Flood epsilon lift (default: 0).\n"
      << "\nMisc:\n"
      << "  --quiet                  Suppress progress output.\n"
      << "  -h, --help               Show this help.\n\n"
      << "Examples:\n"
      << "  # Find 20 resilient cities (low flood/ponding), save a CSV, and emit a top-seed manifest\n"
      << "  ./build/proc_isocity_mine --samples 500 --size 128x128 --days 160 --objective resilient \\\n"
      << "    --csv out/mine.csv --json out/top.json --manifest out/top_seeds.txt --top 20\n\n"
      << "  # Find pathological/chaotic seeds (stress testing)\n"
      << "  ./build/proc_isocity_mine --samples 200 --objective chaos --top 15 --csv chaos.csv\n";
}

static bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

static bool ParseU64(const std::string& s, std::uint64_t* out)
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
  errno = 0;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

static bool ParseF64(const std::string& s, double* out)
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

static bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  double v = 0.0;
  if (!ParseF64(s, &v)) return false;
  if (v < -1.0e9 || v > 1.0e9) return false;
  *out = static_cast<float>(v);
  return true;
}

static bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  if (s == "0") {
    *out = false;
    return true;
  }
  if (s == "1") {
    *out = true;
    return true;
  }
  return false;
}

static bool ParseWxH(const std::string& s, int* outW, int* outH)
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

static bool EnsureParentDir(const fs::path& file)
{
  try {
    const fs::path parent = file.parent_path();
    if (!parent.empty()) fs::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

static bool ParseJsonObjectText(const std::string& text, JsonValue& outObj, std::string& outErr)
{
  JsonValue v;
  if (!ParseJson(text, v, outErr)) return false;
  if (!v.isObject()) {
    outErr = "expected JSON object";
    return false;
  }
  outObj = std::move(v);
  outErr.clear();
  return true;
}

static std::string Trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

static std::vector<std::string> SplitCsvList(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  cur.reserve(s.size());

  for (char c : s) {
    if (c == ',') {
      const std::string t = Trim(cur);
      if (!t.empty()) out.push_back(t);
      cur.clear();
      continue;
    }
    cur.push_back(c);
  }

  const std::string t = Trim(cur);
  if (!t.empty()) out.push_back(t);
  return out;
}

} // namespace

int main(int argc, char** argv)
{
  std::uint64_t seedStart = 1;
  std::uint64_t seedStep = 1;
  int samples = 100;
  int w = 96;
  int h = 96;
  int days = 120;

  std::string outCsv = "mine.csv";
  std::string outJson;
  std::string outManifest;

  int topK = 10;
  bool diverse = true;
  int candidatePool = 0;
  double mmrScoreWeight = 0.70;

  bool hydro = true;
  bool seaEdge = true;
  bool sea8 = false;
  float seaLevelOverride = std::numeric_limits<float>::quiet_NaN();
  float depEps = 0.0f;

  // Pareto selection (multi-objective).
  bool pareto = false;
  std::string paretoMax;
  std::string paretoMin;
  bool paretoCrowding = true;

  bool quiet = false;

  MineObjective objective = MineObjective::Balanced;

  ProcGenConfig procCfg;
  SimConfig simCfg;

  auto requireArg = [&](int& i, const char* opt) -> std::string {
    if (i + 1 >= argc) {
      std::cerr << opt << " requires a value\n";
      std::exit(2);
    }
    return argv[++i] ? std::string(argv[i]) : std::string();
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();

    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }

    if (arg == "--quiet") {
      quiet = true;
      continue;
    }

    if (arg == "--seed-start") {
      const std::string v = requireArg(i, "--seed-start");
      if (!ParseU64(v, &seedStart)) {
        std::cerr << "invalid --seed-start: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--seed-step") {
      const std::string v = requireArg(i, "--seed-step");
      if (!ParseU64(v, &seedStep)) {
        std::cerr << "invalid --seed-step: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--samples") {
      const std::string v = requireArg(i, "--samples");
      if (!ParseI32(v, &samples) || samples <= 0) {
        std::cerr << "invalid --samples: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--size") {
      const std::string v = requireArg(i, "--size");
      if (!ParseWxH(v, &w, &h)) {
        std::cerr << "invalid --size: " << v << " (expected WxH)\n";
        return 2;
      }
      continue;
    }

    if (arg == "--days") {
      const std::string v = requireArg(i, "--days");
      if (!ParseI32(v, &days) || days < 0) {
        std::cerr << "invalid --days: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--objective") {
      const std::string v = requireArg(i, "--objective");
      if (!ParseMineObjective(v, objective)) {
        std::cerr << "unknown --objective: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--csv") {
      outCsv = requireArg(i, "--csv");
      continue;
    }

    if (arg == "--json") {
      outJson = requireArg(i, "--json");
      continue;
    }

    if (arg == "--manifest") {
      outManifest = requireArg(i, "--manifest");
      continue;
    }

    if (arg == "--top") {
      const std::string v = requireArg(i, "--top");
      if (!ParseI32(v, &topK) || topK < 0) {
        std::cerr << "invalid --top: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--diverse") {
      const std::string v = requireArg(i, "--diverse");
      if (!ParseBool01(v, &diverse)) {
        std::cerr << "invalid --diverse (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--candidate-pool") {
      const std::string v = requireArg(i, "--candidate-pool");
      if (!ParseI32(v, &candidatePool) || candidatePool < 0) {
        std::cerr << "invalid --candidate-pool: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--mmr-score-weight") {
      const std::string v = requireArg(i, "--mmr-score-weight");
      if (!ParseF64(v, &mmrScoreWeight)) {
        std::cerr << "invalid --mmr-score-weight: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--hydro") {
      const std::string v = requireArg(i, "--hydro");
      if (!ParseBool01(v, &hydro)) {
        std::cerr << "invalid --hydro (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--sea-level") {
      const std::string v = requireArg(i, "--sea-level");
      if (!ParseF32(v, &seaLevelOverride)) {
        std::cerr << "invalid --sea-level: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--sea-edge") {
      const std::string v = requireArg(i, "--sea-edge");
      if (!ParseBool01(v, &seaEdge)) {
        std::cerr << "invalid --sea-edge (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--sea-8conn") {
      const std::string v = requireArg(i, "--sea-8conn");
      if (!ParseBool01(v, &sea8)) {
        std::cerr << "invalid --sea-8conn (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--dep-eps") {
      const std::string v = requireArg(i, "--dep-eps");
      if (!ParseF32(v, &depEps)) {
        std::cerr << "invalid --dep-eps: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--pareto") {
      const std::string v = requireArg(i, "--pareto");
      if (!ParseBool01(v, &pareto)) {
        std::cerr << "invalid --pareto (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--pareto-max") {
      paretoMax = requireArg(i, "--pareto-max");
      continue;
    }

    if (arg == "--pareto-min") {
      paretoMin = requireArg(i, "--pareto-min");
      continue;
    }

    if (arg == "--pareto-crowding") {
      const std::string v = requireArg(i, "--pareto-crowding");
      if (!ParseBool01(v, &paretoCrowding)) {
        std::cerr << "invalid --pareto-crowding (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--config") {
      const std::string path = requireArg(i, "--config");
      CombinedConfig cc;
      std::string err;
      if (!LoadCombinedConfigJsonFile(path, cc, err)) {
        std::cerr << "failed to load combined config: " << err << "\n";
        return 1;
      }
      if (cc.hasProc) procCfg = cc.proc;
      if (cc.hasSim) simCfg = cc.sim;
      continue;
    }

    if (arg == "--proc") {
      const std::string path = requireArg(i, "--proc");
      std::string err;
      if (!LoadProcGenConfigJsonFile(path, procCfg, err)) {
        std::cerr << "failed to load proc config: " << err << "\n";
        return 1;
      }
      continue;
    }

    if (arg == "--sim") {
      const std::string path = requireArg(i, "--sim");
      std::string err;
      if (!LoadSimConfigJsonFile(path, simCfg, err)) {
        std::cerr << "failed to load sim config: " << err << "\n";
        return 1;
      }
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    }

    std::cerr << "unexpected positional arg: " << arg << "\n";
    return 2;
  }

  const float seaLevel = std::isfinite(seaLevelOverride) ? seaLevelOverride : procCfg.waterLevel;

  if (!quiet) {
    std::cout << "Mining " << samples << " seeds...\n";
    std::cout << "  seedStart: " << seedStart << " (" << HexU64(seedStart) << ")\n";
    std::cout << "  seedStep:  " << seedStep << "\n";
    std::cout << "  size:      " << w << "x" << h << "\n";
    std::cout << "  days:      " << days << "\n";
    std::cout << "  objective: " << MineObjectiveName(objective) << "\n";
    if (hydro) {
      std::cout << "  hydrology: on (seaLevel=" << seaLevel << ")\n";
    } else {
      std::cout << "  hydrology: off\n";
    }
    if (pareto) {
      std::cout << "  selection: pareto (NSGA-II)\n";
    }
  }

  MineConfig mineCfg;
  mineCfg.seedStart = seedStart;
  mineCfg.seedStep = seedStep;
  mineCfg.samples = samples;
  mineCfg.w = w;
  mineCfg.h = h;
  mineCfg.days = days;
  mineCfg.objective = objective;
  mineCfg.hydrologyEnabled = hydro;
  mineCfg.seaLevelOverride = seaLevelOverride;
  mineCfg.seaRequireEdgeConnection = seaEdge;
  mineCfg.seaEightConnected = sea8;
  mineCfg.depressionEpsilon = depEps;

  MineProgressFn progress;
  if (!quiet) {
    progress = [&](const MineProgress& p) {
      const int i = p.index;
      const int n = p.total;
      if (!p.record) return;
      if (i == 0 || (i + 1) % 25 == 0 || (i + 1) == n) {
        std::cout << "  [" << (i + 1) << "/" << n << "] seed=" << p.record->seed << " score="
                  << std::fixed << std::setprecision(2) << p.record->score << "\n";
      }
    };
  }

  std::vector<MineRecord> recs = MineSeeds(mineCfg, procCfg, simCfg, progress);

  // Optional Pareto analysis/selection.
  ParetoResult paretoRes;
  std::vector<ParetoObjective> paretoObjectives;
  if (pareto) {
    auto addMetricList = [&](const std::string& list, bool maximize) -> bool {
      for (const std::string& name : SplitCsvList(list)) {
        MineMetric m;
        if (!ParseMineMetric(name, m)) {
          std::cerr << "unknown Pareto metric: " << name << "\n";
          return false;
        }
        ParetoObjective o;
        o.metric = m;
        o.maximize = maximize;
        paretoObjectives.push_back(o);
      }
      return true;
    };

    if (!paretoMax.empty() || !paretoMin.empty()) {
      if (!addMetricList(paretoMax, true)) return 2;
      if (!addMetricList(paretoMin, false)) return 2;
    } else {
      // Default set: a compact, interpretable 4-5D tradeoff surface.
      paretoObjectives.push_back({MineMetric::Population, true});
      paretoObjectives.push_back({MineMetric::Happiness, true});
      paretoObjectives.push_back({MineMetric::AvgLandValue, true});
      paretoObjectives.push_back({MineMetric::TrafficCongestion, false});
      if (hydro) paretoObjectives.push_back({MineMetric::FloodRisk, false});
    }

    if (paretoObjectives.empty()) {
      std::cerr << "--pareto requires at least one objective (use --pareto-max/--pareto-min)\n";
      return 2;
    }

    paretoRes = ComputePareto(recs, paretoObjectives);
    for (std::size_t i = 0; i < recs.size(); ++i) {
      recs[i].paretoRank = paretoRes.rank[i];
      recs[i].paretoCrowding = paretoRes.crowding[i];
    }
  }

  // Write CSV.
  {
    if (!EnsureParentDir(fs::path(outCsv))) {
      std::cerr << "failed to create output directory for: " << outCsv << "\n";
      return 1;
    }
    std::ofstream f(outCsv);
    if (!f) {
      std::cerr << "failed to open CSV for write: " << outCsv << "\n";
      return 1;
    }
    WriteMineCsvHeader(f);
    for (const auto& r : recs) WriteMineCsvRow(f, r);
  }

  // Select and print top seeds.
  const std::vector<int> top = pareto
                                  ? SelectTopParetoIndices(paretoRes, topK, paretoCrowding)
                                  : SelectTopIndices(recs, topK, diverse, candidatePool, mmrScoreWeight);

  if (!quiet) {
    std::cout << "\nTop " << top.size() << " seeds (";
    if (pareto) {
      std::cout << "pareto";
    } else {
      std::cout << (diverse ? "diverse" : "ranked");
    }
    std::cout << "):\n";
    for (std::size_t rank = 0; rank < top.size(); ++rank) {
      const MineRecord& r = recs[static_cast<std::size_t>(top[rank])];
      std::cout << "  " << (rank + 1) << ") seed=" << r.seed << " (" << HexU64(r.seed) << ")"
                << " score=" << std::fixed << std::setprecision(2) << r.score
                << " pop=" << r.stats.population
                << " happy=" << std::setprecision(3) << r.stats.happiness
                << " cong=" << std::setprecision(3) << r.stats.trafficCongestion;
      if (pareto) {
        std::cout << " pr=" << r.paretoRank
                  << " cd=" << std::setprecision(3) << r.paretoCrowding;
      }
      if (hydro) {
        std::cout << " seaFrac=" << std::setprecision(3) << r.seaFloodFrac
                  << " pondMax=" << std::setprecision(3) << r.pondMaxDepth;
      }
      std::cout << "\n";
    }
    std::cout << "\nWrote: " << outCsv << "\n";
  }

  // Write manifest.
  if (!outManifest.empty()) {
    if (!EnsureParentDir(fs::path(outManifest))) {
      std::cerr << "failed to create output directory for: " << outManifest << "\n";
      return 1;
    }
    std::ofstream f(outManifest);
    if (!f) {
      std::cerr << "failed to open manifest for write: " << outManifest << "\n";
      return 1;
    }
    for (int id : top) {
      f << recs[static_cast<std::size_t>(id)].seed << "\n";
    }
    if (!quiet) {
      std::cout << "Wrote: " << outManifest << "\n";
    }
  }

  // Write JSON summary.
  if (!outJson.empty()) {
    if (!EnsureParentDir(fs::path(outJson))) {
      std::cerr << "failed to create output directory for: " << outJson << "\n";
      return 1;
    }

    JsonValue root = JsonValue::MakeObject();
    auto add = [](JsonValue& o, const char* key, JsonValue v) { o.objectValue.emplace_back(key, std::move(v)); };

    add(root, "samples", JsonValue::MakeNumber(static_cast<double>(samples)));
    add(root, "seedStart", JsonValue::MakeNumber(static_cast<double>(seedStart)));
    add(root, "seedStartHex", JsonValue::MakeString(HexU64(seedStart)));
    add(root, "seedStep", JsonValue::MakeNumber(static_cast<double>(seedStep)));

    JsonValue size = JsonValue::MakeObject();
    add(size, "w", JsonValue::MakeNumber(static_cast<double>(w)));
    add(size, "h", JsonValue::MakeNumber(static_cast<double>(h)));
    add(root, "size", std::move(size));

    add(root, "days", JsonValue::MakeNumber(static_cast<double>(days)));
    add(root, "objective", JsonValue::MakeString(MineObjectiveName(objective)));
    add(root, "diverse", JsonValue::MakeBool(diverse));
    add(root, "candidatePool", JsonValue::MakeNumber(static_cast<double>(candidatePool)));
    add(root, "mmrScoreWeight", JsonValue::MakeNumber(mmrScoreWeight));

    add(root, "paretoEnabled", JsonValue::MakeBool(pareto));
    add(root, "paretoCrowding", JsonValue::MakeBool(paretoCrowding));
    if (pareto) {
      JsonValue arrObj = JsonValue::MakeArray();
      for (const auto& o : paretoObjectives) {
        JsonValue po = JsonValue::MakeObject();
        add(po, "metric", JsonValue::MakeString(MineMetricName(o.metric)));
        add(po, "maximize", JsonValue::MakeBool(o.maximize));
        arrObj.arrayValue.push_back(std::move(po));
      }
      add(root, "paretoObjectives", std::move(arrObj));
    }

    add(root, "hydroEnabled", JsonValue::MakeBool(hydro));
    add(root, "seaLevel", JsonValue::MakeNumber(static_cast<double>(seaLevel)));

    // Embed configs as JSON objects.
    {
      std::string err;
      JsonValue procObj;
      JsonValue simObj;
      if (ParseJsonObjectText(ProcGenConfigToJson(procCfg, 2), procObj, err)) {
        add(root, "proc", std::move(procObj));
      }
      err.clear();
      if (ParseJsonObjectText(SimConfigToJson(simCfg, 2), simObj, err)) {
        add(root, "sim", std::move(simObj));
      }
    }

    JsonValue arr = JsonValue::MakeArray();
    for (int id : top) {
      arr.arrayValue.push_back(MineRecordToJson(recs[static_cast<std::size_t>(id)]));
    }
    add(root, "top", std::move(arr));

    std::string err;
    JsonWriteOptions opt;
    opt.pretty = true;
    opt.indent = 2;
    opt.sortKeys = false;
    if (!WriteJsonFile(outJson, root, err, opt)) {
      std::cerr << "failed to write JSON: " << err << "\n";
      return 1;
    }

    if (!quiet) {
      std::cout << "Wrote: " << outJson << "\n";
    }
  }

  return 0;
}
