#include "isocity/PolicyOptimizer.hpp"
#include "isocity/PolicyOptimizerExport.hpp"

#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  long v = std::strtol(s.c_str(), &end, 10);
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
  unsigned long long v = std::strtoull(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseF64(const std::string& s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

bool ParseSize(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const auto xPos = s.find('x');
  if (xPos == std::string::npos) return false;
  const std::string a = s.substr(0, xPos);
  const std::string b = s.substr(xPos + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

bool ParseRangeI32(const std::string& s, int* outMin, int* outMax)
{
  if (!outMin || !outMax) return false;

  std::size_t sep = s.find("..");
  std::size_t sepLen = 2;
  if (sep == std::string::npos) {
    sep = s.find(':');
    sepLen = 1;
  }
  if (sep == std::string::npos) return false;

  const std::string a = s.substr(0, sep);
  const std::string b = s.substr(sep + sepLen);
  int mn = 0, mx = 0;
  if (!ParseI32(a, &mn) || !ParseI32(b, &mx)) return false;
  if (mx < mn) return false;
  *outMin = mn;
  *outMax = mx;
  return true;
}

bool ParseMethod(const std::string& s, isocity::PolicyOptMethod* out)
{
  if (!out) return false;
  std::string t = s;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (t == "exhaustive" || t == "grid") {
    *out = isocity::PolicyOptMethod::Exhaustive;
    return true;
  }
  if (t == "cem" || t == "crossentropy" || t == "cross-entropy") {
    *out = isocity::PolicyOptMethod::CEM;
    return true;
  }
  return false;
}

void ApplyObjectivePreset(const std::string& s, isocity::PolicyOptimizerConfig& cfg)
{
  std::string t = s;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (t == "money" || t == "profit" || t == "budget") {
    cfg.objective = {};
    cfg.objective.wMoneyDelta = 1.0;
    cfg.objective.minHappiness = 0.0;
    return;
  }

  if (t == "balanced" || t == "mix") {
    cfg.objective = {};
    cfg.objective.wMoneyDelta = 1.0;
    cfg.objective.wHappyPop = 0.50;
    cfg.objective.wUnemployed = 0.15;
    cfg.objective.wCongestionPop = 0.05;
    cfg.objective.minHappiness = 0.40;
    cfg.objective.minMoneyEnd = 0;
    return;
  }

  if (t == "growth" || t == "population") {
    cfg.objective = {};
    cfg.objective.wMoneyDelta = 0.25;
    cfg.objective.wPopulation = 1.0;
    cfg.objective.wHappyPop = 0.25;
    cfg.objective.wUnemployed = 0.20;
    cfg.objective.minHappiness = 0.35;
    cfg.objective.minMoneyEnd = 0;
    return;
  }

  if (t == "happiness" || t == "happy") {
    cfg.objective = {};
    cfg.objective.wHappyPop = 1.0;
    cfg.objective.wUnemployed = 0.10;
    cfg.objective.minMoneyEnd = 0;
    cfg.objective.minHappiness = 0.0;
    return;
  }

  // Unknown preset: keep defaults.
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_policyopt (headless policy optimization: taxes + maintenance)\n\n"
      << "Loads (or generates) a world, optionally simulates N warmup days, then searches over\n"
      << "SimConfig policy parameters to maximize an objective by repeatedly simulating evalDays.\n\n"
      << "Usage:\n"
      << "  proc_isocity_policyopt [--load save.bin | --seed N --size WxH] [options]\n\n"
      << "Inputs:\n"
      << "  --load <path>         Load a save instead of generating.\n"
      << "  --seed <u64>          World seed (when generating). Default: 1\n"
      << "  --size <WxH>          World size (when generating). Default: 128x128\n"
      << "  --days <N>            Warmup sim days before optimization. Default: 60\n"
      << "  --require-outside <0|1>  Outside-connection rule. Default: 1\n\n"
      << "Optimization:\n"
      << "  --method <cem|exhaustive>   Default: cem (falls back to exhaustive for small spaces)\n"
      << "  --objective <money|balanced|growth|happiness>   Default: balanced\n"
      << "  --eval-days <N>        Days simulated per candidate. Default: 60\n"
      << "  --iters <N>            CEM iterations. Default: 25\n"
      << "  --pop <N>              Candidates per iteration. Default: 64\n"
      << "  --elites <N>           Elite count. Default: 8\n"
      << "  --explore <p>          Uniform exploration probability [0..1]. Default: 0.10\n"
      << "  --opt-seed <u64>       Optimizer RNG seed. Default: 1\n"
      << "  --threads <N>          Candidate eval threads (0=auto). Default: 0\n"
      << "  --top-k <N>            Retain/export top-K candidates. Default: 32\n\n"
      << "Search ranges (inclusive):\n"
      << "  --tax-res <a..b>       Default: 0..6\n"
      << "  --tax-com <a..b>       Default: 0..8\n"
      << "  --tax-ind <a..b>       Default: 0..8\n"
      << "  --maint-road <a..b>    Default: 0..4\n"
      << "  --maint-park <a..b>    Default: 0..4\n\n"
      << "Objective overrides (optional):\n"
      << "  --w-money <f>          Weight for money delta\n"
      << "  --w-pop <f>            Weight for population\n"
      << "  --w-happy <f>          Weight for happy population (avgHappiness * pop)\n"
      << "  --w-unemp <f>          Penalty weight for unemployed\n"
      << "  --w-cong <f>           Penalty weight for congestion-pop (congestion * pop)\n"
      << "  --min-happy <f>        Hard constraint on final happiness\n"
      << "  --min-money-end <i>    Hard constraint on final money\n\n"
      << "Outputs:\n"
      << "  --json <path>          Write JSON report.\n"
      << "  --csv <path>           Write top-K candidates CSV.\n"
      << "  --trace <path>         Write iteration trace CSV.\n"
      << "  --write-save <path>    Save the warmup world with the best policy applied.\n"
      << "  --apply-days <N>       If writing a save, simulate N extra days with the best policy first.\n"
      << "  --no-top-in-json       Omit the 'top' array from JSON (smaller output).\n\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;

  int warmupDays = 60;
  bool requireOutside = true;

  PolicySearchSpace space;
  PolicyOptimizerConfig optCfg;
  optCfg.method = PolicyOptMethod::CEM;
  ApplyObjectivePreset("balanced", optCfg);

  std::string outJson;
  std::string outCsv;
  std::string outTrace;
  std::string outSave;
  int applyDays = 0;
  bool includeTopInJson = true;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    std::string v;

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--load") {
      if (!requireValue(i, v)) {
        std::cerr << "--load requires a value\n";
        return 2;
      }
      loadPath = v;
    } else if (a == "--seed") {
      if (!requireValue(i, v) || !ParseU64(v, &seed)) {
        std::cerr << "--seed requires a u64\n";
        return 2;
      }
    } else if (a == "--size") {
      if (!requireValue(i, v) || !ParseSize(v, &w, &h)) {
        std::cerr << "--size requires WxH\n";
        return 2;
      }
    } else if (a == "--days") {
      if (!requireValue(i, v) || !ParseI32(v, &warmupDays) || warmupDays < 0) {
        std::cerr << "--days requires a non-negative int\n";
        return 2;
      }
    } else if (a == "--require-outside") {
      int t = 1;
      if (!requireValue(i, v) || !ParseI32(v, &t)) {
        std::cerr << "--require-outside requires 0/1\n";
        return 2;
      }
      requireOutside = (t != 0);
    } else if (a == "--method") {
      if (!requireValue(i, v) || !ParseMethod(v, &optCfg.method)) {
        std::cerr << "--method requires cem|exhaustive\n";
        return 2;
      }
    } else if (a == "--objective") {
      if (!requireValue(i, v)) {
        std::cerr << "--objective requires a value\n";
        return 2;
      }
      ApplyObjectivePreset(v, optCfg);
    } else if (a == "--eval-days") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.evalDays) || optCfg.evalDays < 0) {
        std::cerr << "--eval-days requires a non-negative int\n";
        return 2;
      }
    } else if (a == "--iters") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.iterations) || optCfg.iterations <= 0) {
        std::cerr << "--iters requires a positive int\n";
        return 2;
      }
    } else if (a == "--pop") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.population) || optCfg.population <= 0) {
        std::cerr << "--pop requires a positive int\n";
        return 2;
      }
    } else if (a == "--elites") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.elites) || optCfg.elites <= 0) {
        std::cerr << "--elites requires a positive int\n";
        return 2;
      }
    } else if (a == "--explore") {
      double p = 0.0;
      if (!requireValue(i, v) || !ParseF64(v, &p) || p < 0.0 || p > 1.0) {
        std::cerr << "--explore requires a float in [0,1]\n";
        return 2;
      }
      optCfg.exploreProb = static_cast<float>(p);
    } else if (a == "--opt-seed") {
      if (!requireValue(i, v) || !ParseU64(v, &optCfg.rngSeed)) {
        std::cerr << "--opt-seed requires a u64\n";
        return 2;
      }
    } else if (a == "--threads") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.threads) || optCfg.threads < 0) {
        std::cerr << "--threads requires a non-negative int\n";
        return 2;
      }
    } else if (a == "--top-k") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.topK) || optCfg.topK < 0) {
        std::cerr << "--top-k requires a non-negative int\n";
        return 2;
      }
    } else if (a == "--tax-res") {
      if (!requireValue(i, v) || !ParseRangeI32(v, &space.taxResMin, &space.taxResMax)) {
        std::cerr << "--tax-res requires a..b\n";
        return 2;
      }
    } else if (a == "--tax-com") {
      if (!requireValue(i, v) || !ParseRangeI32(v, &space.taxComMin, &space.taxComMax)) {
        std::cerr << "--tax-com requires a..b\n";
        return 2;
      }
    } else if (a == "--tax-ind") {
      if (!requireValue(i, v) || !ParseRangeI32(v, &space.taxIndMin, &space.taxIndMax)) {
        std::cerr << "--tax-ind requires a..b\n";
        return 2;
      }
    } else if (a == "--maint-road") {
      if (!requireValue(i, v) || !ParseRangeI32(v, &space.maintRoadMin, &space.maintRoadMax)) {
        std::cerr << "--maint-road requires a..b\n";
        return 2;
      }
    } else if (a == "--maint-park") {
      if (!requireValue(i, v) || !ParseRangeI32(v, &space.maintParkMin, &space.maintParkMax)) {
        std::cerr << "--maint-park requires a..b\n";
        return 2;
      }
    } else if (a == "--w-money") {
      if (!requireValue(i, v) || !ParseF64(v, &optCfg.objective.wMoneyDelta)) {
        std::cerr << "--w-money requires a float\n";
        return 2;
      }
    } else if (a == "--w-pop") {
      if (!requireValue(i, v) || !ParseF64(v, &optCfg.objective.wPopulation)) {
        std::cerr << "--w-pop requires a float\n";
        return 2;
      }
    } else if (a == "--w-happy") {
      if (!requireValue(i, v) || !ParseF64(v, &optCfg.objective.wHappyPop)) {
        std::cerr << "--w-happy requires a float\n";
        return 2;
      }
    } else if (a == "--w-unemp") {
      if (!requireValue(i, v) || !ParseF64(v, &optCfg.objective.wUnemployed)) {
        std::cerr << "--w-unemp requires a float\n";
        return 2;
      }
    } else if (a == "--w-cong") {
      if (!requireValue(i, v) || !ParseF64(v, &optCfg.objective.wCongestionPop)) {
        std::cerr << "--w-cong requires a float\n";
        return 2;
      }
    } else if (a == "--min-happy") {
      if (!requireValue(i, v) || !ParseF64(v, &optCfg.objective.minHappiness)) {
        std::cerr << "--min-happy requires a float\n";
        return 2;
      }
    } else if (a == "--min-money-end") {
      if (!requireValue(i, v) || !ParseI32(v, &optCfg.objective.minMoneyEnd)) {
        std::cerr << "--min-money-end requires an int\n";
        return 2;
      }
    } else if (a == "--json") {
      if (!requireValue(i, v)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJson = v;
    } else if (a == "--csv") {
      if (!requireValue(i, v)) {
        std::cerr << "--csv requires a path\n";
        return 2;
      }
      outCsv = v;
    } else if (a == "--trace") {
      if (!requireValue(i, v)) {
        std::cerr << "--trace requires a path\n";
        return 2;
      }
      outTrace = v;
    } else if (a == "--write-save") {
      if (!requireValue(i, v)) {
        std::cerr << "--write-save requires a path\n";
        return 2;
      }
      outSave = v;
    } else if (a == "--apply-days") {
      if (!requireValue(i, v) || !ParseI32(v, &applyDays) || applyDays < 0) {
        std::cerr << "--apply-days requires a non-negative int\n";
        return 2;
      }
    } else if (a == "--no-top-in-json") {
      includeTopInJson = false;
    } else {
      std::cerr << "Unknown arg: " << a << "\n";
      PrintHelp();
      return 2;
    }
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;

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

  simCfg.requireOutsideConnection = requireOutside;

  // Warmup.
  Simulator sim(simCfg);
  for (int d = 0; d < warmupDays; ++d) {
    sim.stepOnce(world);
  }
  if (warmupDays == 0) sim.refreshDerivedStats(world);

  // Baseline evaluation uses the current policy embedded in the save (or defaults).
  PolicyCandidate baselinePolicy = ExtractPolicyFromSimConfig(simCfg);
  PolicyEvalResult baselineEval = EvaluatePolicyCandidate(world, simCfg, baselinePolicy, optCfg);

  // Optimize.
  PolicyOptimizationResult r = OptimizePolicies(world, simCfg, space, optCfg);

  std::cout << "Warmup days: " << warmupDays << "\n";
  std::cout << "Eval days:   " << optCfg.evalDays << "\n";
  std::cout << "Candidates evaluated: " << r.candidatesEvaluated << "\n";
  std::cout << "Iterations completed: " << r.iterationsCompleted << "\n\n";

  auto printEval = [&](const char* name, const PolicyEvalResult& e) {
    std::cout << name << ":\n";
    std::cout << "  score: " << e.score << "\n";
    std::cout << "  policy: tax(res/com/ind)=" << e.policy.taxResidential << "/" << e.policy.taxCommercial << "/"
              << e.policy.taxIndustrial << " maint(road/park)=" << e.policy.maintenanceRoad << "/" << e.policy.maintenancePark
              << "\n";
    std::cout << "  money: " << e.metrics.moneyStart << " -> " << e.metrics.moneyEnd << " (delta " << e.metrics.moneyDelta
              << ", avgNet/day " << e.metrics.avgNetPerDay << ")\n";
    std::cout << "  pop: " << e.metrics.populationEnd << " employed: " << e.metrics.employedEnd
              << " jobsAccessible: " << e.metrics.jobsCapacityAccessibleEnd << "\n";
    std::cout << "  happiness: end " << e.metrics.happinessEnd << " avg " << e.metrics.avgHappiness << "\n";
    std::cout << "  congestion: " << e.metrics.trafficCongestionEnd << " avgCommuteTime: " << e.metrics.avgCommuteTimeEnd << "\n";
    std::cout << "  landValue: " << e.metrics.avgLandValueEnd << " demandRes: " << e.metrics.demandResidentialEnd << "\n\n";
  };

  printEval("Baseline", baselineEval);
  printEval("Best", r.best);

  if (!outJson.empty()) {
    std::string err;
    if (!ExportPolicyOptimizationJson(outJson, r, optCfg, space, &baselineEval, includeTopInJson, &err)) {
      std::cerr << "Failed to write JSON: " << outJson << "\n";
      if (!err.empty()) std::cerr << err << "\n";
      return 2;
    }
    std::cout << "Wrote JSON: " << outJson << "\n";
  }
  if (!outCsv.empty()) {
    std::string err;
    if (!ExportPolicyOptimizationTopCsv(outCsv, r, &err)) {
      std::cerr << "Failed to write CSV: " << outCsv << "\n";
      if (!err.empty()) std::cerr << err << "\n";
      return 2;
    }
    std::cout << "Wrote CSV: " << outCsv << "\n";
  }
  if (!outTrace.empty()) {
    std::string err;
    if (!ExportPolicyOptimizationTraceCsv(outTrace, r, &err)) {
      std::cerr << "Failed to write trace CSV: " << outTrace << "\n";
      if (!err.empty()) std::cerr << err << "\n";
      return 2;
    }
    std::cout << "Wrote trace: " << outTrace << "\n";
  }

  if (!outSave.empty()) {
    SimConfig outSimCfg = simCfg;
    ApplyPolicyToSimConfig(r.best.policy, outSimCfg);

    World outWorld = world;
    if (applyDays > 0) {
      Simulator sim2(outSimCfg);
      for (int d = 0; d < applyDays; ++d) sim2.stepOnce(outWorld);
    }

    std::string err;
    if (!SaveWorldBinary(outWorld, procCfg, outSimCfg, outSave, err)) {
      std::cerr << "Failed to write save: " << outSave << "\n";
      std::cerr << err << "\n";
      return 2;
    }
    std::cout << "Wrote save: " << outSave << "\n";
  }

  return 0;
}
