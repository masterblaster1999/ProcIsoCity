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
#include "isocity/MineClustering.hpp"
#include "isocity/MineGallery.hpp"
#include "isocity/MineCheckpoint.hpp"
#include "isocity/MineCheckpointSh.hpp"
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
      << "  --seed-sampler <mode>    Seed enumeration: linear|splitmix|vdc2|halton23 (default: linear).\n"
      << "  --seed-xor <u64>         XOR-scramble generated seeds (default: 0).\n"
      << "  --samples <N>            Number of seeds to test (default: 100).\n"
      << "  --size <WxH>             World size (default: 96x96).\n"
      << "  --days <N>               Simulation days per seed (default: 120).\n"
      << "  --threads <N>            Worker threads for mining (default: 1; <=0 auto).\n"
      << "  --objective <name>       Ranking objective: balanced|growth|resilient|chaos (default: balanced).\n"
      << "  --score-expr <expr>      Override MineRecord::score with a custom expression (optional).\n"
      << "\nOutputs:\n"
      << "  --csv <out.csv>          Write all results as CSV (default: mine.csv).\n"
      << "  --json <out.json>        Write a JSON summary (top seeds + configs).\n"
      << "  --manifest <out.txt>     Write selected top seeds (one per line).\n"
      << "  --top <K>                Number of seeds to select/print (default: 10).\n"
      << "  --gallery <dir>          Write an offline HTML thumbnail gallery for the selected seeds.\n"
      << "  --gallery-scale <N>      Nearest-neighbor upscale for gallery thumbnails (default: 3).\n"
      << "  --gallery-layers <list>  Comma-separated ExportLayer names for gallery thumbnails (default: overlay).\n"
      << "  --gallery-sheet <0|1>    Write a contact sheet image grid (default: 1).\n"
      << "  --gallery-cols <N>       Contact sheet column count (default: 6).\n"
      << "  --gallery-embed <0|1>    Add an interactive 2D embedding plot to the gallery index (default: 0).\n"
      << "  --embed-space <name>     Embedding distance space: scalar|layout|hybrid (default: hybrid).\n"
      << "  --embed-layout-weight <F> When space=hybrid, blend KPI/layout distance in [0,1] (default: 0.50).\n"
      << "  --embed-robust <0|1>     Robust KPI scaling for embedding (median+MAD) (default: 1).\n"
      << "  --embed-metrics <list>   KPI metrics for embedding (default: population,happiness,avg_land_value,traffic_congestion,flood_risk).\n"
      << "  --embed-iters <N>        Power-iteration steps for embedding eigensolve (default: 64).\n"
      << "  --cluster-k <N>          Cluster the selected seeds (k-medoids) and annotate gallery/JSON (default: 0).\n"
      << "  --cluster-space <name>   Clustering distance space: scalar|layout|hybrid (default: hybrid).\n"
      << "  --cluster-layout-weight <F> Blend KPI/layout in [0,1] when space=hybrid (default: 0.50).\n"
      << "  --cluster-robust <0|1>   Robust KPI scaling for clustering (median+MAD) (default: 1).\n"
      << "  --cluster-metrics <list> KPI metrics for scalar/hybrid clustering (default: mixed behavior vector).\n"
      << "  --cluster-iters <N>      Max refinement iterations for k-medoids clustering (default: 30).\n"
      << "  --gallery-neighbors <0|1> Add kNN 'similar seeds' chips + neighbors.json (default: 0).\n"
      << "  --neighbors-k <N>        Neighbors per seed for the kNN graph (default: 8).\n"
      << "  --neighbors-space <name> Neighbor distance space: scalar|layout|hybrid (default: hybrid).\n"
      << "  --neighbors-layout-weight <F> Blend KPI/layout in [0,1] when space=hybrid (default: 0.50).\n"
      << "  --neighbors-robust <0|1> Robust KPI scaling for neighbors (median+MAD) (default: 1).\n"
      << "  --neighbors-metrics <list> KPI metrics for scalar/hybrid neighbor distances (default: mixed behavior vector).\n"
      << "  --gallery-traces <0|1>  Add per-day KPI sparklines + traces.json (default: 0).\n"
      << "  --trace-metrics <list>  Trace metrics for sparklines (default: population,happiness,traffic_congestion,money).\n"
      << "                         Available: population,happiness,money,avg_land_value,traffic_congestion,goods_satisfaction,services_overall_satisfaction,transit_mode_share,avg_commute_time,economy_index,trade_market_index.\n"
      << "  --diverse <0|1>          Diversify the top-K selection (default: 1).\n"
      << "  --candidate-pool <N>     Candidate pool size used for diversity (default: max(50,10*K)).\n"
      << "  --mmr-score-weight <F>   Diversity tradeoff in [0,1] (default: 0.70).\n"
      << "  --diversity-mode <name>   Diverse distance mode: scalar|layout|hybrid (default: scalar).\n"
      << "  --mmr-layout-weight <F>   When mode=hybrid, blend KPI/layout distance in [0,1] (default: 0.50).\n"
      << "\nCheckpointing / resume (optional):\n"
      << "  --checkpoint <out.jsonl>  Stream results to a JSONL checkpoint while mining.\n"
      << "  --resume <in.jsonl>       Resume mining from an existing checkpoint (must match configs).\n"
      << "\nSuccessive halving (multi-fidelity mining, optional):\n"
      << "  --sh <spec>              Enable successive halving with <days>:<keep>[,...].\n"
      << "                           Example: --sh 30:500,90:150,160:50\n"
      << "                           Supports --checkpoint/--resume (uses a staged checkpoint format).\n"
      << "  --sh-diverse <0|1>        Diversify the kept set between stages (default: 1).\n"
      << "  --sh-candidate-pool <N>   Candidate pool size for SH selection (default: max(50,10*keep)).\n"
      << "  --sh-mmr-score-weight <F> Score/diversity tradeoff in [0,1] (default: 0.60).\n"
      << "  --sh-diversity-mode <name> Diversity distance: scalar|layout|hybrid (default: hybrid).\n"
      << "  --sh-layout-weight <F>    When mode=hybrid, blend KPI/layout distance in [0,1] (default: 0.50).\n"
      << "\nMulti-objective (Pareto/NSGA-II) selection (optional):\n"
      << "  --pareto <0|1>           Enable Pareto selection instead of scalar score ranking (default: 0).\n"
      << "  --pareto-max <list>      Comma-separated metrics to maximize (default: population,happiness,avg_land_value).\n"
      << "  --pareto-min <list>      Comma-separated metrics to minimize (default: traffic_congestion[,flood_risk]).\n"
      << "  --pareto-crowding <0|1>  Use crowding distance within fronts (default: 1).\n"
      << "\nQuality-diversity (MAP-Elites) selection (optional):\n"
      << "  --map-elites <0|1>       Enable MAP-Elites selection (default: 0).\n"
      << "  --me-x <metric>          X-axis behavior metric (default: water_frac).\n"
      << "  --me-y <metric>          Y-axis behavior metric (default: traffic_congestion).\n"
      << "  --me-bins <WxH>          Grid resolution in bins (default: 10x10).\n"
      << "  --me-auto-range <0|1>    Auto-range axes based on sampled records (default: 1).\n"
      << "  --me-x-min <F>           X min when auto-range=0 (default: 0).\n"
      << "  --me-x-max <F>           X max when auto-range=0 (default: 1).\n"
      << "  --me-y-min <F>           Y min when auto-range=0 (default: 0).\n"
      << "  --me-y-max <F>           Y max when auto-range=0 (default: 1).\n"
      << "  --me-quality <metric>    Quality metric for cell elites (default: score).\n"
      << "  --me-quality-max <0|1>   Maximize quality metric (default: 1).\n"
      << "  --me-clamp <0|1>         Clamp out-of-range values into bins (default: 1).\n"
      << "\nOutlier / novelty (Local Outlier Factor) selection (optional):\n"
      << "  --outliers <0|1>         Select top-K by LOF (weird cities) instead of score/Pareto/MAP-Elites (default: 0).\n"
      << "  --outlier-k <N>          Neighborhood size for kNN/LOF (default: 20).\n"
      << "  --outlier-space <name>   Distance space: scalar|layout|hybrid (default: scalar).\n"
      << "  --outlier-layout-weight <F> Blend KPI/layout in [0,1] when space=hybrid (default: 0.50).\n"
      << "  --outlier-robust <0|1>   Robust KPI scaling (median+MAD) when scalar/hybrid (default: 1).\n"
      << "  --outlier-metrics <list> KPI metrics for scalar/hybrid LOF (default: population,happiness,avg_land_value,traffic_congestion,flood_risk).\n"
      << "\n  Metric names:\n"
      << "    population,happiness,money,avg_land_value,traffic_congestion,goods_satisfaction,services_overall_satisfaction,\n"
      << "    water_frac,road_frac,zone_frac,park_frac,sea_flood_frac,sea_max_depth,pond_frac,pond_max_depth,pond_volume,\n"
      << "    flood_risk,score,objective_score\n"
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
      << "  --score-expr-help         Print score expression language help and exit.\n"
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

struct SuccessiveHalvingStageStats {
  int stageIndex = 0; // 0-based
  int days = 0;
  int inCount = 0;
  int keepCount = 0;
  std::uint64_t bestSeed = 0;
  double bestScore = 0.0;
};

static bool ParseSuccessiveHalvingSpec(const std::string& s,
                                      std::vector<SuccessiveHalvingStage>& outStages,
                                      std::string& outErr)
{
  outStages.clear();
  outErr.clear();

  const std::vector<std::string> parts = SplitCsvList(s);
  if (parts.empty()) {
    outErr = "empty spec";
    return false;
  }

  int lastDays = -1;
  int lastKeep = std::numeric_limits<int>::max();

  for (const std::string& partRaw : parts) {
    const std::string part = Trim(partRaw);
    if (part.empty()) continue;

    const std::size_t pos = part.find(':');
    if (pos == std::string::npos) {
      outErr = "expected <days>:<keep> entries separated by commas";
      return false;
    }

    int d = 0;
    int k = 0;
    if (!ParseI32(Trim(part.substr(0, pos)), &d) || d <= 0) {
      outErr = "invalid days in stage: " + part;
      return false;
    }
    if (!ParseI32(Trim(part.substr(pos + 1)), &k) || k <= 0) {
      outErr = "invalid keep count in stage: " + part;
      return false;
    }

    if (lastDays >= 0 && d <= lastDays) {
      outErr = "stage days must be strictly increasing";
      return false;
    }
    if (k > lastKeep) {
      outErr = "stage keep counts must be non-increasing";
      return false;
    }

    SuccessiveHalvingStage st;
    st.days = d;
    st.keep = k;
    outStages.push_back(st);

    lastDays = d;
    lastKeep = k;
  }

  if (outStages.empty()) {
    outErr = "empty spec";
    return false;
  }

  return true;
}

} // namespace

int main(int argc, char** argv)
{
  std::uint64_t seedStart = 1;
  std::uint64_t seedStep = 1;
  MineSeedSampler seedSampler = MineSeedSampler::Linear;
  std::uint64_t seedXor = 0;
  int samples = 100;
  int w = 96;
  int h = 96;
  int days = 120;
  int threads = 1;

  std::string outCsv = "mine.csv";
  std::string outJson;
  std::string outManifest;

  // Optional offline gallery output.
  std::string galleryDir;
  int galleryScale = 3;
  std::vector<ExportLayer> galleryLayers = {ExportLayer::Overlay};
  bool gallerySheet = true;
  int galleryCols = 6;
  bool galleryEmbed = false;
  MineEmbeddingConfig embedCfg;
  embedCfg.space = MineDiversityMode::Hybrid;
  embedCfg.layoutWeight = 0.50;
  embedCfg.robustScaling = true;
  embedCfg.powerIters = 64;

  // Optional kNN neighbor graph (for gallery navigation).
  bool galleryNeighbors = false;
  MineNeighborsConfig neighborsCfg;
  neighborsCfg.k = 8;
  neighborsCfg.space = MineDiversityMode::Hybrid;
  neighborsCfg.layoutWeight = 0.50;
  neighborsCfg.robustScaling = true;

  // Optional per-day KPI traces (sparklines) for the gallery.
  bool galleryTraces = false;
  std::vector<MineTraceMetric> traceMetrics; // empty => DefaultMineTraceMetrics()

  // Optional clustering of selected seeds (k-medoids).
  MineClusteringConfig clusterCfg;
  clusterCfg.k = 0; // disabled by default
  clusterCfg.space = MineDiversityMode::Hybrid;
  clusterCfg.layoutWeight = 0.50;
  clusterCfg.robustScaling = true;
  clusterCfg.maxIters = 30;

  // Optional JSONL checkpoint output (and resume input).
  std::string checkpointPath;
  std::string resumePath;

  // Successive halving (multi-fidelity mining).
  //
  // Enable with --sh <spec> where spec is a comma-separated list of
  // <days>:<keep> stages, e.g. "30:500,90:150,160:50".
  std::string shSpec;
  bool shDiverse = true;
  int shCandidatePool = 0;
  double shMmrScoreWeight = 0.60;
  MineDiversityMode shDiversityMode = MineDiversityMode::Hybrid;
  double shLayoutWeight = 0.50;

  int topK = 10;
  bool diverse = true;
  int candidatePool = 0;
  double mmrScoreWeight = 0.70;
  MineDiversityMode diversityMode = MineDiversityMode::Scalar;
  double mmrLayoutWeight = 0.50;

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

  // MAP-Elites (quality-diversity) selection.
  bool mapElites = false;
  MapElitesConfig mapElitesCfg;
  mapElitesCfg.x.metric = MineMetric::WaterFrac;
  mapElitesCfg.y.metric = MineMetric::TrafficCongestion;
  mapElitesCfg.x.bins = 10;
  mapElitesCfg.y.bins = 10;
  mapElitesCfg.x.autoRange = true;
  mapElitesCfg.y.autoRange = true;
  mapElitesCfg.qualityMetric = MineMetric::Score;
  mapElitesCfg.qualityMaximize = true;
  mapElitesCfg.clampToBounds = true;

  // Outlier / novelty (Local Outlier Factor) selection.
  bool outliers = false;
  OutlierConfig outlierCfg;
  outlierCfg.k = 20;
  outlierCfg.space = MineDiversityMode::Scalar;
  outlierCfg.layoutWeight = 0.50;
  outlierCfg.robustScaling = true;
  // Default metrics can be overridden via --outlier-metrics.
  outlierCfg.metrics = {MineMetric::Population, MineMetric::Happiness, MineMetric::AvgLandValue, MineMetric::TrafficCongestion, MineMetric::FloodRisk};

  bool quiet = false;

  MineObjective objective = MineObjective::Balanced;
  std::string scoreExpr;

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

    if (arg == "--threads" || arg == "-j") {
      const std::string v = requireArg(i, "--threads");
      if (!ParseI32(v, &threads)) {
        std::cerr << "invalid --threads: " << v << "\n";
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

    if (arg == "--score-expr") {
      scoreExpr = requireArg(i, "--score-expr");
      continue;
    }

    if (arg == "--score-expr-help") {
      std::cout << MineExprHelpText() << "\n";
      return 0;
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

    if (arg == "--gallery") {
      galleryDir = requireArg(i, "--gallery");
      continue;
    }

    if (arg == "--gallery-scale") {
      const std::string v = requireArg(i, "--gallery-scale");
      if (!ParseI32(v, &galleryScale) || galleryScale <= 0) {
        std::cerr << "invalid --gallery-scale: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--gallery-layers") {
      const std::string v = requireArg(i, "--gallery-layers");
      std::vector<ExportLayer> layers;
      for (const std::string& name : SplitCsvList(v)) {
        ExportLayer l;
        if (!ParseExportLayer(name, l)) {
          std::cerr << "invalid --gallery-layers entry: " << name << "\n";
          return 2;
        }
        layers.push_back(l);
      }
      if (!layers.empty()) galleryLayers = std::move(layers);
      continue;
    }

    if (arg == "--gallery-sheet") {
      const std::string v = requireArg(i, "--gallery-sheet");
      if (!ParseBool01(v, &gallerySheet)) {
        std::cerr << "invalid --gallery-sheet (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--gallery-cols") {
      const std::string v = requireArg(i, "--gallery-cols");
      if (!ParseI32(v, &galleryCols) || galleryCols <= 0) {
        std::cerr << "invalid --gallery-cols: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--gallery-embed") {
      const std::string v = requireArg(i, "--gallery-embed");
      if (!ParseBool01(v, &galleryEmbed)) {
        std::cerr << "invalid --gallery-embed (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--embed-space") {
      const std::string v = requireArg(i, "--embed-space");
      MineDiversityMode m;
      if (!ParseMineDiversityMode(v, m)) {
        std::cerr << "invalid --embed-space: " << v << " (expected scalar|layout|hybrid)\n";
        return 2;
      }
      embedCfg.space = m;
      continue;
    }

    if (arg == "--embed-layout-weight") {
      const std::string v = requireArg(i, "--embed-layout-weight");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --embed-layout-weight: " << v << "\n";
        return 2;
      }
      embedCfg.layoutWeight = d;
      continue;
    }

    if (arg == "--embed-robust") {
      const std::string v = requireArg(i, "--embed-robust");
      if (!ParseBool01(v, &embedCfg.robustScaling)) {
        std::cerr << "invalid --embed-robust (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--embed-iters") {
      const std::string v = requireArg(i, "--embed-iters");
      int it = 0;
      if (!ParseI32(v, &it) || it <= 0) {
        std::cerr << "invalid --embed-iters: " << v << "\n";
        return 2;
      }
      embedCfg.powerIters = it;
      continue;
    }

    if (arg == "--embed-metrics") {
      const std::string v = requireArg(i, "--embed-metrics");
      std::vector<MineMetric> ms;
      for (const std::string& name : SplitCsvList(v)) {
        MineMetric m;
        if (!ParseMineMetric(name, m)) {
          std::cerr << "invalid --embed-metrics entry: " << name << "\n";
          return 2;
        }
        ms.push_back(m);
      }
      if (!ms.empty()) embedCfg.metrics = std::move(ms);
      continue;
    }

    if (arg == "--cluster-k") {
      const std::string v = requireArg(i, "--cluster-k");
      int kk = 0;
      if (!ParseI32(v, &kk) || kk < 0) {
        std::cerr << "invalid --cluster-k: " << v << "\n";
        return 2;
      }
      clusterCfg.k = kk;
      continue;
    }

    if (arg == "--cluster-space") {
      const std::string v = requireArg(i, "--cluster-space");
      MineDiversityMode m;
      if (!ParseMineDiversityMode(v, m)) {
        std::cerr << "invalid --cluster-space: " << v << " (expected scalar|layout|hybrid)\n";
        return 2;
      }
      clusterCfg.space = m;
      continue;
    }

    if (arg == "--cluster-layout-weight") {
      const std::string v = requireArg(i, "--cluster-layout-weight");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --cluster-layout-weight: " << v << "\n";
        return 2;
      }
      clusterCfg.layoutWeight = d;
      continue;
    }

    if (arg == "--cluster-robust") {
      const std::string v = requireArg(i, "--cluster-robust");
      if (!ParseBool01(v, &clusterCfg.robustScaling)) {
        std::cerr << "invalid --cluster-robust (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--cluster-iters") {
      const std::string v = requireArg(i, "--cluster-iters");
      int it = 0;
      if (!ParseI32(v, &it) || it <= 0) {
        std::cerr << "invalid --cluster-iters: " << v << "\n";
        return 2;
      }
      clusterCfg.maxIters = it;
      continue;
    }

    if (arg == "--cluster-metrics") {
      const std::string v = requireArg(i, "--cluster-metrics");
      std::vector<MineMetric> ms;
      for (const std::string& name : SplitCsvList(v)) {
        MineMetric m;
        if (!ParseMineMetric(name, m)) {
          std::cerr << "invalid --cluster-metrics entry: " << name << "\n";
          return 2;
        }
        ms.push_back(m);
      }
      if (!ms.empty()) clusterCfg.metrics = std::move(ms);
      continue;
    }

    if (arg == "--gallery-neighbors") {
      const std::string v = requireArg(i, "--gallery-neighbors");
      if (!ParseBool01(v, &galleryNeighbors)) {
        std::cerr << "invalid --gallery-neighbors (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--neighbors-k") {
      const std::string v = requireArg(i, "--neighbors-k");
      int kk = 0;
      if (!ParseI32(v, &kk) || kk < 0) {
        std::cerr << "invalid --neighbors-k: " << v << "\n";
        return 2;
      }
      neighborsCfg.k = kk;
      continue;
    }

    if (arg == "--neighbors-space") {
      const std::string v = requireArg(i, "--neighbors-space");
      MineDiversityMode m;
      if (!ParseMineDiversityMode(v, m)) {
        std::cerr << "invalid --neighbors-space: " << v << " (expected scalar|layout|hybrid)\n";
        return 2;
      }
      neighborsCfg.space = m;
      continue;
    }

    if (arg == "--neighbors-layout-weight") {
      const std::string v = requireArg(i, "--neighbors-layout-weight");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --neighbors-layout-weight: " << v << "\n";
        return 2;
      }
      neighborsCfg.layoutWeight = d;
      continue;
    }

    if (arg == "--neighbors-robust") {
      const std::string v = requireArg(i, "--neighbors-robust");
      if (!ParseBool01(v, &neighborsCfg.robustScaling)) {
        std::cerr << "invalid --neighbors-robust (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--neighbors-metrics") {
      const std::string v = requireArg(i, "--neighbors-metrics");
      std::vector<MineMetric> ms;
      for (const std::string& name : SplitCsvList(v)) {
        MineMetric m;
        if (!ParseMineMetric(name, m)) {
          std::cerr << "invalid --neighbors-metrics entry: " << name << "\n";
          return 2;
        }
        ms.push_back(m);
      }
      if (!ms.empty()) neighborsCfg.metrics = std::move(ms);
      continue;
    }

    if (arg == "--gallery-traces") {
      const std::string v = requireArg(i, "--gallery-traces");
      if (!ParseBool01(v, &galleryTraces)) {
        std::cerr << "invalid --gallery-traces (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--trace-metrics") {
      const std::string v = requireArg(i, "--trace-metrics");
      std::string perr;
      if (!ParseMineTraceMetricList(v, traceMetrics, &perr)) {
        std::cerr << "invalid --trace-metrics: " << perr << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--checkpoint") {
      checkpointPath = requireArg(i, "--checkpoint");
      continue;
    }

    if (arg == "--resume") {
      resumePath = requireArg(i, "--resume");
      continue;
    }

    if (arg == "--sh") {
      shSpec = requireArg(i, "--sh");
      continue;
    }

    if (arg == "--sh-diverse") {
      const std::string v = requireArg(i, "--sh-diverse");
      if (!ParseBool01(v, &shDiverse)) {
        std::cerr << "invalid --sh-diverse (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--sh-candidate-pool") {
      const std::string v = requireArg(i, "--sh-candidate-pool");
      if (!ParseI32(v, &shCandidatePool) || shCandidatePool < 0) {
        std::cerr << "invalid --sh-candidate-pool: " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--sh-mmr-score-weight") {
      const std::string v = requireArg(i, "--sh-mmr-score-weight");
      double d = 0.0;
      if (!ParseF64(v, &d) || d < 0.0 || d > 1.0) {
        std::cerr << "invalid --sh-mmr-score-weight (expected [0,1]): " << v << "\n";
        return 2;
      }
      shMmrScoreWeight = d;
      continue;
    }

    if (arg == "--sh-diversity-mode") {
      const std::string v = requireArg(i, "--sh-diversity-mode");
      MineDiversityMode m;
      if (!ParseMineDiversityMode(v, m)) {
        std::cerr << "invalid --sh-diversity-mode: " << v << "\n";
        return 2;
      }
      shDiversityMode = m;
      continue;
    }

    if (arg == "--sh-layout-weight") {
      const std::string v = requireArg(i, "--sh-layout-weight");
      double d = 0.0;
      if (!ParseF64(v, &d) || d < 0.0 || d > 1.0) {
        std::cerr << "invalid --sh-layout-weight (expected [0,1]): " << v << "\n";
        return 2;
      }
      shLayoutWeight = d;
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

    if (arg == "--diversity-mode") {
      const std::string v = requireArg(i, "--diversity-mode");
      if (!ParseMineDiversityMode(v, diversityMode)) {
        std::cerr << "invalid --diversity-mode: " << v << " (expected scalar|layout|hybrid)\n";
        return 2;
      }
      continue;
    }

    if (arg == "--mmr-layout-weight") {
      const std::string v = requireArg(i, "--mmr-layout-weight");
      if (!ParseF64(v, &mmrLayoutWeight)) {
        std::cerr << "invalid --mmr-layout-weight: " << v << "\n";
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

    if (arg == "--map-elites") {
      const std::string v = requireArg(i, "--map-elites");
      if (!ParseBool01(v, &mapElites)) {
        std::cerr << "invalid --map-elites (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--me-x") {
      const std::string v = requireArg(i, "--me-x");
      MineMetric m;
      if (!ParseMineMetric(v, m)) {
        std::cerr << "invalid --me-x metric: " << v << "\n";
        return 2;
      }
      mapElitesCfg.x.metric = m;
      continue;
    }

    if (arg == "--me-y") {
      const std::string v = requireArg(i, "--me-y");
      MineMetric m;
      if (!ParseMineMetric(v, m)) {
        std::cerr << "invalid --me-y metric: " << v << "\n";
        return 2;
      }
      mapElitesCfg.y.metric = m;
      continue;
    }

    if (arg == "--me-bins") {
      const std::string v = requireArg(i, "--me-bins");
      int bx = 0;
      int by = 0;
      if (!ParseWxH(v, &bx, &by) || bx <= 0 || by <= 0) {
        std::cerr << "invalid --me-bins: " << v << " (expected WxH)\n";
        return 2;
      }
      mapElitesCfg.x.bins = bx;
      mapElitesCfg.y.bins = by;
      continue;
    }

    if (arg == "--me-auto-range") {
      const std::string v = requireArg(i, "--me-auto-range");
      bool b = true;
      if (!ParseBool01(v, &b)) {
        std::cerr << "invalid --me-auto-range (expected 0|1): " << v << "\n";
        return 2;
      }
      mapElitesCfg.x.autoRange = b;
      mapElitesCfg.y.autoRange = b;
      continue;
    }

    if (arg == "--me-x-min") {
      const std::string v = requireArg(i, "--me-x-min");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --me-x-min: " << v << "\n";
        return 2;
      }
      mapElitesCfg.x.min = d;
      continue;
    }

    if (arg == "--me-x-max") {
      const std::string v = requireArg(i, "--me-x-max");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --me-x-max: " << v << "\n";
        return 2;
      }
      mapElitesCfg.x.max = d;
      continue;
    }

    if (arg == "--me-y-min") {
      const std::string v = requireArg(i, "--me-y-min");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --me-y-min: " << v << "\n";
        return 2;
      }
      mapElitesCfg.y.min = d;
      continue;
    }

    if (arg == "--me-y-max") {
      const std::string v = requireArg(i, "--me-y-max");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --me-y-max: " << v << "\n";
        return 2;
      }
      mapElitesCfg.y.max = d;
      continue;
    }

    if (arg == "--me-quality") {
      const std::string v = requireArg(i, "--me-quality");
      MineMetric m;
      if (!ParseMineMetric(v, m)) {
        std::cerr << "invalid --me-quality metric: " << v << "\n";
        return 2;
      }
      mapElitesCfg.qualityMetric = m;
      continue;
    }

    if (arg == "--me-quality-max") {
      const std::string v = requireArg(i, "--me-quality-max");
      if (!ParseBool01(v, &mapElitesCfg.qualityMaximize)) {
        std::cerr << "invalid --me-quality-max (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--me-clamp") {
      const std::string v = requireArg(i, "--me-clamp");
      if (!ParseBool01(v, &mapElitesCfg.clampToBounds)) {
        std::cerr << "invalid --me-clamp (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--outliers") {
      const std::string v = requireArg(i, "--outliers");
      if (!ParseBool01(v, &outliers)) {
        std::cerr << "invalid --outliers (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--outlier-k") {
      const std::string v = requireArg(i, "--outlier-k");
      int k = 0;
      if (!ParseI32(v, &k) || k <= 0) {
        std::cerr << "invalid --outlier-k: " << v << "\n";
        return 2;
      }
      outlierCfg.k = k;
      continue;
    }

    if (arg == "--outlier-space") {
      const std::string v = requireArg(i, "--outlier-space");
      MineDiversityMode m;
      if (!ParseMineDiversityMode(v, m)) {
        std::cerr << "invalid --outlier-space: " << v << " (expected scalar|layout|hybrid)\n";
        return 2;
      }
      outlierCfg.space = m;
      continue;
    }

    if (arg == "--outlier-layout-weight") {
      const std::string v = requireArg(i, "--outlier-layout-weight");
      double d = 0.0;
      if (!ParseF64(v, &d)) {
        std::cerr << "invalid --outlier-layout-weight: " << v << "\n";
        return 2;
      }
      outlierCfg.layoutWeight = d;
      continue;
    }

    if (arg == "--outlier-robust") {
      const std::string v = requireArg(i, "--outlier-robust");
      if (!ParseBool01(v, &outlierCfg.robustScaling)) {
        std::cerr << "invalid --outlier-robust (expected 0|1): " << v << "\n";
        return 2;
      }
      continue;
    }

    if (arg == "--outlier-metrics") {
      const std::string v = requireArg(i, "--outlier-metrics");
      std::vector<MineMetric> ms;
      for (const std::string& name : SplitCsvList(v)) {
        MineMetric m;
        if (!ParseMineMetric(name, m)) {
          std::cerr << "invalid --outlier-metrics entry: " << name << "\n";
          return 2;
        }
        ms.push_back(m);
      }
      if (!ms.empty()) outlierCfg.metrics = std::move(ms);
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

  if (mapElites && pareto) {
    std::cerr << "cannot combine --map-elites with --pareto (choose one selection mode)\n";
    return 2;
  }

  if (outliers && (mapElites || pareto)) {
    std::cerr << "cannot combine --outliers with --pareto or --map-elites (choose one selection mode)\n";
    return 2;
  }

  const bool shEnabled = !shSpec.empty();
  std::vector<SuccessiveHalvingStage> shStages;
  std::vector<SuccessiveHalvingStageStats> shStageStats;
  if (shEnabled) {
    std::string err;
    if (!ParseSuccessiveHalvingSpec(shSpec, shStages, err)) {
      std::cerr << "invalid --sh spec: " << err << "\n";
      return 2;
    }

    // Validate keep counts against the initial sample budget.
    int cur = samples;
    for (std::size_t si = 0; si < shStages.size(); ++si) {
      if (shStages[si].keep > cur) {
        std::cerr << "invalid --sh spec: stage keep " << shStages[si].keep << " exceeds current candidate count " << cur << "\n";
        return 2;
      }
      cur = shStages[si].keep;
    }

    // For downstream outputs (gallery, JSON, etc.) treat the final stage days
    // as the effective simulation horizon.
    if (!shStages.empty()) days = shStages.back().days;
  }

  const float seaLevel = std::isfinite(seaLevelOverride) ? seaLevelOverride : procCfg.waterLevel;

  // Build the "full" mining config used for checkpoint headers and config matching.
  MineConfig mineCfg;
  mineCfg.seedStart = seedStart;
  mineCfg.seedStep = seedStep;
  mineCfg.seedSampler = seedSampler;
  mineCfg.seedXor = seedXor;
  mineCfg.samples = samples;
  mineCfg.w = w;
  mineCfg.h = h;
  mineCfg.days = days;
  mineCfg.threads = threads;
  mineCfg.objective = objective;
  mineCfg.scoreExpr = scoreExpr;
  mineCfg.hydrologyEnabled = hydro;
  mineCfg.seaLevelOverride = seaLevelOverride;
  mineCfg.seaRequireEdgeConnection = seaEdge;
  mineCfg.seaEightConnected = sea8;
  mineCfg.depressionEpsilon = depEps;

  // Mine records.
  std::vector<MineRecord> recs;
  if (shEnabled) {
    // Successive halving mode: stage budgets increase, candidate set shrinks.
    //
    // Checkpoint/resume in successive halving uses a staged JSONL format
    // (MineCheckpointSh.*) that stores stage index + within-stage index.
    // We only trust contiguous prefixes per stage when resuming.

    // Build desired staged checkpoint header (for config matching).
    MineCheckpointShHeader wantShHdr;
    wantShHdr.version = 1;
    wantShHdr.mineCfg = mineCfg;
    wantShHdr.procCfg = procCfg;
    wantShHdr.simCfg = simCfg;
    wantShHdr.sh.spec = shSpec;
    wantShHdr.sh.stages = shStages;
    wantShHdr.sh.diverse = shDiverse;
    wantShHdr.sh.candidatePool = shCandidatePool;
    wantShHdr.sh.mmrScoreWeight = shMmrScoreWeight;
    wantShHdr.sh.diversityMode = shDiversityMode;
    wantShHdr.sh.layoutWeight = shLayoutWeight;

    // Stage totals: number of candidates evaluated per stage.
    std::vector<int> stageTotals;
    stageTotals.reserve(shStages.size());
    {
      int cur = samples;
      for (const SuccessiveHalvingStage& st : shStages) {
        stageTotals.push_back(cur);
        cur = st.keep;
      }
    }

    // Resume state (optional).
    MineCheckpointShHeader resumeHdr;
    std::vector<std::vector<MineRecord>> resumeStageRecords;
    std::vector<std::vector<bool>> resumeHave;
    std::vector<int> resumeStageDone;

    auto ContiguousPrefix = [](const std::vector<bool>* have, int total) -> int {
      int n = 0;
      while (n < total) {
        const bool ok = (have && n < static_cast<int>(have->size())) ? (*have)[static_cast<std::size_t>(n)] : false;
        if (!ok) break;
        ++n;
      }
      return n;
    };

    if (!resumePath.empty()) {
      std::string err;
      if (!LoadMineCheckpointShFile(resumePath, &resumeHdr, resumeStageRecords, &resumeHave, &err)) {
        std::cerr << "failed to load staged checkpoint: " << err << "\n";
        return 1;
      }

      std::string why;
      if (!MineCheckpointShConfigsMatch(resumeHdr, wantShHdr, &why)) {
        std::cerr << "checkpoint config mismatch: " << why << "\n";
        return 2;
      }

      resumeStageDone.assign(shStages.size(), 0);
      for (std::size_t si = 0; si < shStages.size(); ++si) {
        const int total = stageTotals[si];
        const std::vector<bool>* have = nullptr;
        if (si < resumeHave.size()) have = &resumeHave[si];
        resumeStageDone[si] = ContiguousPrefix(have, total);

        if (resumeStageDone[si] > total) {
          std::cerr << "checkpoint stage " << (si + 1) << " has " << resumeStageDone[si]
                    << " records, but expected at most " << total << "\n";
          return 2;
        }
      }

      // Require that if a stage is incomplete, later stages must be empty.
      bool foundIncomplete = false;
      for (std::size_t si = 0; si < shStages.size(); ++si) {
        const bool complete = (resumeStageDone[si] >= stageTotals[si]);
        if (!complete) foundIncomplete = true;
        if (foundIncomplete && si + 1 < shStages.size()) {
          // If next stage has any prefix, the file is inconsistent.
          const int nextDone = resumeStageDone[si + 1];
          if (nextDone > 0) {
            std::cerr << "checkpoint inconsistency: stage " << (si + 1) << " is incomplete, but stage " << (si + 2)
                      << " already has records\n";
            return 2;
          }
        }
      }

      if (checkpointPath.empty()) checkpointPath = resumePath;
    }

    if (!quiet) {
      if (!resumePath.empty()) {
        // Print resume summary.
        std::cout << "Resuming successive halving from checkpoint:\n";
        for (std::size_t si = 0; si < shStages.size(); ++si) {
          const int done = (si < resumeStageDone.size()) ? resumeStageDone[si] : 0;
          const int total = stageTotals[si];
          std::cout << "  stage " << (si + 1) << "/" << shStages.size() << " (days=" << shStages[si].days << ")"
                    << ": " << done << "/" << total << " complete\n";
        }
      }

      std::cout << "Mining " << samples << " seeds (successive halving)...\n";
      std::cout << "  seedStart: " << seedStart << " (" << HexU64(seedStart) << ")\n";
      std::cout << "  seedStep:  " << seedStep << "\n";
      std::cout << "  seedSampler: " << MineSeedSamplerName(seedSampler) << "\n";
      if (seedXor) std::cout << "  seedXor:    " << HexU64(seedXor) << "\n";
      std::cout << "  size:      " << w << "x" << h << "\n";
      std::cout << "  stages:    " << shSpec << "\n";
      std::cout << "  threads:   " << threads << "\n";
      std::cout << "  objective: " << MineObjectiveName(objective) << "\n";
      if (hydro) {
        std::cout << "  hydrology: on (seaLevel=" << seaLevel << ")\n";
      } else {
        std::cout << "  hydrology: off\n";
      }

      std::cout << "  stageSelection: " << (shDiverse ? "mmr/" : "ranked/") << MineDiversityModeName(shDiversityMode);
      if (shDiverse) {
        std::cout << " (scoreWeight=" << std::fixed << std::setprecision(3) << shMmrScoreWeight;
        if (shDiversityMode == MineDiversityMode::Hybrid) {
          std::cout << " layoutWeight=" << std::fixed << std::setprecision(3) << shLayoutWeight;
        }
        std::cout << ")";
      }
      std::cout << "\n";

      if (outliers) {
        std::cout << "  selection: outliers (LOF)\n";
        std::cout << "    space: " << MineDiversityModeName(outlierCfg.space) << " (k=" << outlierCfg.k << ")\n";
        if (outlierCfg.space == MineDiversityMode::Hybrid) {
          std::cout << "    layoutWeight: " << std::fixed << std::setprecision(3) << outlierCfg.layoutWeight << "\n";
        }
        if (outlierCfg.space != MineDiversityMode::Layout) {
          std::cout << "    robustScaling: " << (outlierCfg.robustScaling ? "on" : "off") << "\n";
          std::cout << "    metrics: ";
          for (std::size_t mi = 0; mi < outlierCfg.metrics.size(); ++mi) {
            if (mi) std::cout << ',';
            std::cout << MineMetricName(outlierCfg.metrics[mi]);
          }
          std::cout << "\n";
        }
      } else if (pareto) {
        std::cout << "  selection: pareto (NSGA-II)\n";
      } else if (mapElites) {
        std::cout << "  selection: map-elites (quality-diversity)\n";
        std::cout << "    x: " << MineMetricName(mapElitesCfg.x.metric) << " (bins=" << mapElitesCfg.x.bins << ")\n";
        std::cout << "    y: " << MineMetricName(mapElitesCfg.y.metric) << " (bins=" << mapElitesCfg.y.bins << ")\n";
        std::cout << "    quality: " << MineMetricName(mapElitesCfg.qualityMetric) << (mapElitesCfg.qualityMaximize ? " (max)" : " (min)") << "\n";
      }
    }

    // Checkpoint file setup (optional).
    std::ofstream checkpoint;
    if (!checkpointPath.empty()) {
      if (!EnsureParentDir(fs::path(checkpointPath))) {
        std::cerr << "failed to create output directory for checkpoint: " << checkpointPath << "\n";
        return 1;
      }

      const bool append = (!resumePath.empty() && checkpointPath == resumePath);
      checkpoint.open(checkpointPath, std::ios::binary | (append ? std::ios::app : std::ios::trunc));
      if (!checkpoint) {
        std::cerr << "failed to open checkpoint for write: " << checkpointPath << "\n";
        return 1;
      }

      if (!append) {
        std::string err;
        if (!WriteMineCheckpointShHeader(checkpoint, wantShHdr, &err)) {
          std::cerr << "failed to write staged checkpoint header: " << err << "\n";
          return 1;
        }

        // If resuming into a *different* checkpoint file, re-emit stable prefixes.
        if (!resumePath.empty()) {
          for (std::size_t si = 0; si < shStages.size(); ++si) {
            const int done = resumeStageDone[si];
            if (done <= 0) continue;

            if (si >= resumeStageRecords.size()) {
              std::cerr << "checkpoint missing stage records for stage " << (si + 1) << "\n";
              return 2;
            }
            if (static_cast<int>(resumeStageRecords[si].size()) < done) {
              std::cerr << "checkpoint missing record data for stage " << (si + 1) << "\n";
              return 2;
            }
            for (int i = 0; i < done; ++i) {
              if (!AppendMineCheckpointShRecord(checkpoint, static_cast<int>(si), i,
                                               resumeStageRecords[si][static_cast<std::size_t>(i)], &err)) {
                std::cerr << "failed to write resumed staged checkpoint record: " << err << "\n";
                return 1;
              }
            }
          }
        }
        checkpoint.flush();
      }
    }

    // Seed schedule for stage 0.
    std::vector<std::uint64_t> seeds;
    seeds.reserve(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i) {
      seeds.push_back(MineSeedForSample(mineCfg, static_cast<std::uint64_t>(i)));
    }

    for (std::size_t si = 0; si < shStages.size(); ++si) {
      const SuccessiveHalvingStage& st = shStages[si];

      const int stageTotal = static_cast<int>(seeds.size());
      const int expectTotal = stageTotals[si];
      if (stageTotal != expectTotal) {
        std::cerr << "internal error: stage candidate count mismatch at stage " << (si + 1)
                  << " (got " << stageTotal << ", expected " << expectTotal << ")\n";
        return 1;
      }

      const int donePrefix = (!resumePath.empty() && si < resumeStageDone.size()) ? resumeStageDone[si] : 0;
      if (donePrefix < 0 || donePrefix > stageTotal) {
        std::cerr << "checkpoint invalid donePrefix for stage " << (si + 1) << "\n";
        return 2;
      }

      MineConfig stageCfg = mineCfg;
      stageCfg.days = st.days;
      stageCfg.samples = static_cast<int>(seeds.size());

      // Assemble stage records, optionally seeded from checkpoint prefix.
      std::vector<MineRecord> stageRecs;
      stageRecs.resize(static_cast<std::size_t>(stageTotal));
      if (donePrefix > 0) {
        if (si >= resumeStageRecords.size()) {
          std::cerr << "checkpoint missing stage " << (si + 1) << " records\n";
          return 2;
        }
        if (static_cast<int>(resumeStageRecords[si].size()) < donePrefix) {
          std::cerr << "checkpoint missing record data for stage " << (si + 1) << "\n";
          return 2;
        }
        for (int i = 0; i < donePrefix; ++i) {
          stageRecs[static_cast<std::size_t>(i)] = resumeStageRecords[si][static_cast<std::size_t>(i)];
        }
      }

      if (donePrefix < stageTotal) {
        std::vector<std::uint64_t> runSeeds;
        runSeeds.reserve(static_cast<std::size_t>(stageTotal - donePrefix));
        for (int i = donePrefix; i < stageTotal; ++i) {
          runSeeds.push_back(seeds[static_cast<std::size_t>(i)]);
        }

        MineProgressFn stageProgress = [&](const MineProgress& p) {
          if (!p.record) return;

          const int globalIndex = donePrefix + p.index;
          if (globalIndex >= 0 && globalIndex < stageTotal) {
            stageRecs[static_cast<std::size_t>(globalIndex)] = *p.record;
          }

          if (!quiet) {
            const int i = globalIndex;
            const int n = stageTotal;
            if (i == 0 || (i + 1) % 25 == 0 || (i + 1) == n) {
              std::cout << "  [stage " << (si + 1) << "/" << shStages.size() << " days=" << st.days << "]"
                        << " [" << (i + 1) << "/" << n << "] seed=" << p.record->seed << " score="
                        << std::fixed << std::setprecision(2) << p.record->score;
              if (!scoreExpr.empty()) {
                std::cout << " (objective=" << std::fixed << std::setprecision(2) << p.record->objectiveScore << ")";
              }
              std::cout << "\n";
            }
          }

          if (checkpoint) {
            std::string err;
            if (!AppendMineCheckpointShRecord(checkpoint, static_cast<int>(si), globalIndex, *p.record, &err)) {
              std::cerr << "checkpoint write failed: " << err << "\n";
              std::exit(1);
            }
            checkpoint.flush();
          }
        };

        std::vector<MineRecord> mined;
        std::string mineErr;
        if (!MineSeedsExplicit(stageCfg, procCfg, simCfg, runSeeds, mined, &mineErr, stageProgress)) {
          std::cerr << "mining failed: " << mineErr << "\n";
          return 1;
        }
      }

      // Stage best.
      std::uint64_t bestSeed = 0;
      double bestScore = -std::numeric_limits<double>::infinity();
      for (const MineRecord& r : stageRecs) {
        if (r.score > bestScore) {
          bestScore = r.score;
          bestSeed = r.seed;
        }
      }

      const int keepN = std::min(st.keep, static_cast<int>(stageRecs.size()));
      std::vector<int> keepIdx;
      keepIdx.reserve(static_cast<std::size_t>(keepN));

      if (keepN >= static_cast<int>(stageRecs.size())) {
        for (int i = 0; i < static_cast<int>(stageRecs.size()); ++i) keepIdx.push_back(i);
      } else if (shDiverse) {
        keepIdx = SelectTopIndices(stageRecs, keepN, true, shCandidatePool, shMmrScoreWeight, shDiversityMode, shLayoutWeight);
      } else {
        std::vector<int> idx(stageRecs.size());
        for (int i = 0; i < static_cast<int>(stageRecs.size()); ++i) idx[static_cast<std::size_t>(i)] = i;
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
          const double sa = stageRecs[static_cast<std::size_t>(a)].score;
          const double sb = stageRecs[static_cast<std::size_t>(b)].score;
          if (sa != sb) return sa > sb;
          const std::uint64_t ua = stageRecs[static_cast<std::size_t>(a)].seed;
          const std::uint64_t ub = stageRecs[static_cast<std::size_t>(b)].seed;
          if (ua != ub) return ua < ub;
          return a < b;
        });
        keepIdx.assign(idx.begin(), idx.begin() + keepN);
      }

      // Record stage stats (for JSON).
      SuccessiveHalvingStageStats stats;
      stats.stageIndex = static_cast<int>(si);
      stats.days = st.days;
      stats.inCount = static_cast<int>(stageRecs.size());
      stats.keepCount = keepN;
      stats.bestSeed = bestSeed;
      stats.bestScore = bestScore;
      shStageStats.push_back(stats);

      if (!quiet) {
        std::cout << "  Stage " << (si + 1) << "/" << shStages.size() << ": keep " << keepN << "/" << stageRecs.size()
                  << " (best seed=" << bestSeed << " score=" << std::fixed << std::setprecision(2) << bestScore << ")\n";
      }

      // Build next stage input.
      if (si + 1 < shStages.size()) {
        std::vector<std::uint64_t> nextSeeds;
        nextSeeds.reserve(keepIdx.size());
        for (int id : keepIdx) {
          nextSeeds.push_back(stageRecs[static_cast<std::size_t>(id)].seed);
        }
        // Ensure deterministic ordering across stages.
        std::sort(nextSeeds.begin(), nextSeeds.end());
        seeds = std::move(nextSeeds);
      } else {
        // Final output records are the kept subset from the last stage.
        recs.clear();
        recs.reserve(keepIdx.size());
        for (int id : keepIdx) {
          recs.push_back(stageRecs[static_cast<std::size_t>(id)]);
        }
        std::stable_sort(recs.begin(), recs.end(), [](const MineRecord& a, const MineRecord& b) {
          return a.seed < b.seed;
        });
      }
    }
  } else {
    // Standard mode: optionally resume from a checkpoint and/or stream a checkpoint.

    // Resume support: load existing checkpoint and continue from the first missing index.
    std::vector<MineRecord> resumeRecords;
    int resumeCount = 0;
    if (!resumePath.empty()) {
      MineCheckpointHeader chkHdr;
      std::vector<bool> have;
      std::string err;
      if (!LoadMineCheckpointFile(resumePath, &chkHdr, resumeRecords, &have, &err)) {
        std::cerr << "failed to load checkpoint: " << err << "\n";
        return 1;
      }

      MineCheckpointHeader wantHdr;
      wantHdr.mineCfg = mineCfg;
      wantHdr.procCfg = procCfg;
      wantHdr.simCfg = simCfg;

      std::string why;
      if (!MineCheckpointConfigsMatch(chkHdr, wantHdr, &why)) {
        std::cerr << "checkpoint config mismatch: " << why << "\n";
        return 2;
      }

      // Determine contiguous prefix length.
      resumeCount = 0;
      const int n = static_cast<int>(have.size());
      while (resumeCount < n && have[static_cast<std::size_t>(resumeCount)]) ++resumeCount;

      // Ignore any non-contiguous tail (corrupted/edited files). We only resume from a stable prefix.
      resumeRecords.resize(static_cast<std::size_t>(resumeCount));

      if (resumeCount > samples) {
        std::cerr << "checkpoint already contains " << resumeCount << " records, but --samples is " << samples << "\n";
        return 2;
      }

      if (checkpointPath.empty()) checkpointPath = resumePath;
    }

    if (!quiet) {
      if (resumeCount > 0) {
        std::cout << "Resuming mining: " << resumeCount << "/" << samples << " complete\n";
      }
      std::cout << "Mining " << (samples - resumeCount) << " seeds...\n";
      std::cout << "  seedStart: " << seedStart << " (" << HexU64(seedStart) << ")\n";
      std::cout << "  seedStep:  " << seedStep << "\n";
      std::cout << "  seedSampler: " << MineSeedSamplerName(seedSampler) << "\n";
      if (seedXor) std::cout << "  seedXor:    " << HexU64(seedXor) << "\n";
      std::cout << "  size:      " << w << "x" << h << "\n";
      std::cout << "  days:      " << days << "\n";
      std::cout << "  threads:   " << threads << "\n";
      std::cout << "  objective: " << MineObjectiveName(objective) << "\n";
      if (hydro) {
        std::cout << "  hydrology: on (seaLevel=" << seaLevel << ")\n";
      } else {
        std::cout << "  hydrology: off\n";
      }
      if (outliers) {
        std::cout << "  selection: outliers (LOF)\n";
        std::cout << "    space: " << MineDiversityModeName(outlierCfg.space) << " (k=" << outlierCfg.k << ")\n";
        if (outlierCfg.space == MineDiversityMode::Hybrid) {
          std::cout << "    layoutWeight: " << std::fixed << std::setprecision(3) << outlierCfg.layoutWeight << "\n";
        }
        if (outlierCfg.space != MineDiversityMode::Layout) {
          std::cout << "    robustScaling: " << (outlierCfg.robustScaling ? "on" : "off") << "\n";
          std::cout << "    metrics: ";
          for (std::size_t mi = 0; mi < outlierCfg.metrics.size(); ++mi) {
            if (mi) std::cout << ',';
            std::cout << MineMetricName(outlierCfg.metrics[mi]);
          }
          std::cout << "\n";
        }
      } else if (pareto) {
        std::cout << "  selection: pareto (NSGA-II)\n";
      } else if (mapElites) {
        std::cout << "  selection: map-elites (quality-diversity)\n";
        std::cout << "    x: " << MineMetricName(mapElitesCfg.x.metric) << " (bins=" << mapElitesCfg.x.bins << ")\n";
        std::cout << "    y: " << MineMetricName(mapElitesCfg.y.metric) << " (bins=" << mapElitesCfg.y.bins << ")\n";
        std::cout << "    quality: " << MineMetricName(mapElitesCfg.qualityMetric) << (mapElitesCfg.qualityMaximize ? " (max)" : " (min)") << "\n";
      }
    }

    // Checkpoint file setup (optional).
    std::ofstream checkpoint;
    if (!checkpointPath.empty()) {
      if (!EnsureParentDir(fs::path(checkpointPath))) {
        std::cerr << "failed to create output directory for checkpoint: " << checkpointPath << "\n";
        return 1;
      }

      const bool append = (resumeCount > 0 && checkpointPath == resumePath);
      checkpoint.open(checkpointPath, std::ios::binary | (append ? std::ios::app : std::ios::trunc));
      if (!checkpoint) {
        std::cerr << "failed to open checkpoint for write: " << checkpointPath << "\n";
        return 1;
      }

      if (!append) {
        MineCheckpointHeader h;
        h.version = 1;
        h.mineCfg = mineCfg;
        h.procCfg = procCfg;
        h.simCfg = simCfg;
        std::string err;
        if (!WriteMineCheckpointHeader(checkpoint, h, &err)) {
          std::cerr << "failed to write checkpoint header: " << err << "\n";
          return 1;
        }

        // If resuming into a *different* checkpoint file, re-emit the already-mined prefix
        // so the new checkpoint is self-contained.
        if (resumeCount > 0) {
          for (int i = 0; i < resumeCount; ++i) {
            if (i >= static_cast<int>(resumeRecords.size())) break;
            if (!AppendMineCheckpointRecord(checkpoint, i, resumeRecords[static_cast<std::size_t>(i)], &err)) {
              std::cerr << "failed to write resumed checkpoint record: " << err << "\n";
              return 1;
            }
          }
        }
        checkpoint.flush();
      }
    }

    // Run mining (only the remaining suffix if resuming).
    MineConfig runCfg = mineCfg;
    runCfg.seedStart = seedStart + static_cast<std::uint64_t>(resumeCount) * seedStep;
    runCfg.samples = samples - resumeCount;

    MineProgressFn progress = [&](const MineProgress& p) {
      if (!p.record) return;

      if (!quiet) {
        const int i = resumeCount + p.index;
        const int n = samples;
        if (i == 0 || (i + 1) % 25 == 0 || (i + 1) == n) {
          std::cout << "  [" << (i + 1) << "/" << n << "] seed=" << p.record->seed << " score="
                    << std::fixed << std::setprecision(2) << p.record->score;
          if (!scoreExpr.empty()) {
            std::cout << " (objective=" << std::fixed << std::setprecision(2) << p.record->objectiveScore << ")";
          }
          std::cout << "\n";
        }
      }

      if (checkpoint) {
        std::string err;
        if (!AppendMineCheckpointRecord(checkpoint, resumeCount + p.index, *p.record, &err)) {
          std::cerr << "checkpoint write failed: " << err << "\n";
          std::exit(1);
        }
        checkpoint.flush();
      }
    };

    recs.reserve(static_cast<std::size_t>(samples));
    for (const MineRecord& r : resumeRecords) recs.push_back(r);

    if (runCfg.samples > 0) {
      std::vector<MineRecord> mined;
      std::string mineErr;
      if (!MineSeeds(runCfg, procCfg, simCfg, mined, &mineErr, progress)) {
        std::cerr << "mining failed: " << mineErr << "\n";
        return 1;
      }
      for (MineRecord& r : mined) recs.push_back(std::move(r));
    }
  }

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

  // Optional outlier/novelty analysis (Local Outlier Factor).
  if (outliers) {
    OutlierResult orr = ComputeLocalOutlierFactor(recs, outlierCfg);
    if (orr.lof.size() == recs.size() && orr.novelty.size() == recs.size()) {
      for (std::size_t i = 0; i < recs.size(); ++i) {
        recs[i].outlierLof = orr.lof[i];
        recs[i].novelty = orr.novelty[i];
      }
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
  MapElitesResult meRes;
  if (mapElites) {
    meRes = ComputeMapElites(recs, mapElitesCfg);
    if (!quiet) {
      std::cout << "\nMAP-Elites: coverage=" << std::fixed << std::setprecision(3) << meRes.coverage
                << " (" << meRes.filledCells << "/" << meRes.grid.size() << " cells)"
                << " qdScore=" << std::setprecision(3) << meRes.qdScore << "\n";
    }
  }

  std::vector<int> top;
  if (outliers) {
    top = SelectTopOutlierIndices(recs, topK);
  } else if (mapElites) {
    top = SelectTopMapElitesIndices(meRes, recs, topK);
  } else if (pareto) {
    top = SelectTopParetoIndices(paretoRes, topK, paretoCrowding);
  } else {
    top = SelectTopIndices(recs, topK, diverse, candidatePool, mmrScoreWeight, diversityMode, mmrLayoutWeight);
  }

  // Optional clustering over the selected set (k-medoids).
  MineClusteringResult clusterRes;
  std::vector<int> recCluster;
  bool haveClusters = false;
  if (clusterCfg.k > 0 && !top.empty()) {
    clusterRes = ComputeMineClusteringKMedoids(recs, top, clusterCfg);
    if (clusterRes.ok && clusterRes.assignment.size() == top.size()) {
      recCluster.assign(recs.size(), -1);
      for (std::size_t i = 0; i < top.size(); ++i) {
        const int recIndex = top[i];
        if (recIndex >= 0 && static_cast<std::size_t>(recIndex) < recCluster.size()) {
          recCluster[static_cast<std::size_t>(recIndex)] = clusterRes.assignment[i];
        }
      }
      haveClusters = true;
      if (!quiet) {
        std::cout << "\nClusters: k=" << clusterRes.cfg.k
                  << " space=" << MineDiversityModeName(clusterRes.cfg.space)
                  << " silhouette=" << std::fixed << std::setprecision(3) << clusterRes.avgSilhouette
                  << " cost=" << std::setprecision(3) << clusterRes.totalCost << "\n";
      }
    } else if (!quiet) {
      std::cout << "\nClusters: disabled (" << (clusterRes.warning.empty() ? "failed" : clusterRes.warning) << ")\n";
    }
  }

  if (!quiet) {
    std::cout << "\nTop " << top.size() << " seeds (";
    if (outliers) {
      std::cout << "outliers/lof/" << MineDiversityModeName(outlierCfg.space);
    } else if (mapElites) {
      std::cout << "map-elites";
    } else if (pareto) {
      std::cout << "pareto";
    } else {
      if (diverse) {
        std::cout << "diverse/" << MineDiversityModeName(diversityMode);
      } else {
        std::cout << "ranked";
      }
    }
    std::cout << "):\n";
    for (std::size_t rank = 0; rank < top.size(); ++rank) {
      const int recIndex = top[rank];
      const MineRecord& r = recs[static_cast<std::size_t>(recIndex)];
      std::cout << "  " << (rank + 1) << ") seed=" << r.seed << " (" << HexU64(r.seed) << ")"
                << " score=" << std::fixed << std::setprecision(2) << r.score
                << " obj=" << std::fixed << std::setprecision(2) << r.objectiveScore
                << " pop=" << r.stats.population
                << " happy=" << std::setprecision(3) << r.stats.happiness
                << " cong=" << std::setprecision(3) << r.stats.trafficCongestion;
      if (haveClusters && static_cast<std::size_t>(recIndex) < recCluster.size()) {
        const int cl = recCluster[static_cast<std::size_t>(recIndex)];
        if (cl >= 0) std::cout << " cl=" << cl;
      }
      if (mapElites) {
        const double vx = MineMetricValue(r, meRes.cfg.x.metric);
        const double vy = MineMetricValue(r, meRes.cfg.y.metric);
        std::cout << " x=" << std::setprecision(3) << vx
                  << " y=" << std::setprecision(3) << vy;
      }
      if (outliers) {
        std::cout << " lof=" << std::setprecision(3) << r.outlierLof
                  << " nov=" << std::setprecision(3) << r.novelty;
        if (outlierCfg.space != MineDiversityMode::Scalar) {
          std::cout << " phash=" << HexU64(r.overlayPHash);
        }
      }
      if (!outliers && !pareto && diverse && diversityMode != MineDiversityMode::Scalar) {
        std::cout << " phash=" << HexU64(r.overlayPHash);
      }
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

  // Optional offline gallery (thumbnails + HTML).
  MineGalleryResult galleryRes;
  bool wroteGallery = false;
  if (!galleryDir.empty()) {
    MineGalleryConfig gcfg;
    gcfg.outDir = fs::path(galleryDir);
    gcfg.format = "png";
    gcfg.exportScale = galleryScale;
    gcfg.layers = galleryLayers;
    gcfg.writeContactSheet = gallerySheet;
    gcfg.contactSheetCols = galleryCols;
    gcfg.writeEmbeddingPlot = galleryEmbed;
    gcfg.embeddingCfg = embedCfg;
    gcfg.writeNeighbors = galleryNeighbors;
    gcfg.neighborsCfg = neighborsCfg;
    gcfg.writeTraces = galleryTraces;
    gcfg.traceMetrics = traceMetrics;
    gcfg.writeClusters = (clusterCfg.k > 0);
    gcfg.clusteringCfg = clusterCfg;

    if (!quiet) {
      std::cout << "Writing gallery to: " << gcfg.outDir.string() << "\n";
    }

    MineGalleryProgressFn gprog;
    if (!quiet) {
      gprog = [&](const MineGalleryProgress& p) {
        if (p.stage == "simulate") {
          std::cout << "  [gallery " << (p.index + 1) << "/" << p.total << "] seed=" << p.seed << "\n";
        }
      };
    }

    std::string gerr;
    if (!WriteMineGallery(gcfg, recs, top, procCfg, simCfg, days, &galleryRes, gerr, gprog)) {
      std::cerr << "failed to write gallery: " << gerr << "\n";
      return 1;
    }
    wroteGallery = true;

    if (!quiet) {
      std::cout << "Wrote gallery index: " << galleryRes.indexHtml.string() << "\n";
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
    add(root, "seedStepHex", JsonValue::MakeString(HexU64(seedStep)));
    add(root, "seedSampler", JsonValue::MakeString(MineSeedSamplerName(seedSampler)));
    add(root, "seedXorHex", JsonValue::MakeString(HexU64(seedXor)));

    // Total number of records that made it through the mining pipeline.
    //
    // In standard mode this matches `samples`. In successive-halving mode this is
    // the final kept count from the last stage.
    add(root, "recordCount", JsonValue::MakeNumber(static_cast<double>(recs.size())));

    JsonValue size = JsonValue::MakeObject();
    add(size, "w", JsonValue::MakeNumber(static_cast<double>(w)));
    add(size, "h", JsonValue::MakeNumber(static_cast<double>(h)));
    add(root, "size", std::move(size));

    add(root, "days", JsonValue::MakeNumber(static_cast<double>(days)));
    add(root, "threads", JsonValue::MakeNumber(static_cast<double>(threads)));
    add(root, "objective", JsonValue::MakeString(MineObjectiveName(objective)));
    add(root, "scoreExpr", JsonValue::MakeString(scoreExpr));
    add(root, "selectionMode", JsonValue::MakeString(outliers ? "outliers_lof" : (mapElites ? "map_elites" : (pareto ? "pareto" : (diverse ? "diverse" : "ranked")))));
    add(root, "diverse", JsonValue::MakeBool(diverse));
    add(root, "candidatePool", JsonValue::MakeNumber(static_cast<double>(candidatePool)));
    add(root, "mmrScoreWeight", JsonValue::MakeNumber(mmrScoreWeight));
    add(root, "diversityMode", JsonValue::MakeString(MineDiversityModeName(diversityMode)));
    add(root, "mmrLayoutWeight", JsonValue::MakeNumber(mmrLayoutWeight));

    add(root, "successiveHalvingEnabled", JsonValue::MakeBool(shEnabled));
    if (shEnabled) {
      add(root, "successiveHalvingSpec", JsonValue::MakeString(shSpec));

      JsonValue sh = JsonValue::MakeObject();
      add(sh, "diverse", JsonValue::MakeBool(shDiverse));
      add(sh, "candidatePool", JsonValue::MakeNumber(static_cast<double>(shCandidatePool)));
      add(sh, "mmrScoreWeight", JsonValue::MakeNumber(shMmrScoreWeight));
      add(sh, "diversityMode", JsonValue::MakeString(MineDiversityModeName(shDiversityMode)));
      add(sh, "layoutWeight", JsonValue::MakeNumber(shLayoutWeight));

      JsonValue stages = JsonValue::MakeArray();
      for (const SuccessiveHalvingStageStats& st : shStageStats) {
        JsonValue e = JsonValue::MakeObject();
        add(e, "stage", JsonValue::MakeNumber(static_cast<double>(st.stageIndex)));
        add(e, "days", JsonValue::MakeNumber(static_cast<double>(st.days)));
        add(e, "inCount", JsonValue::MakeNumber(static_cast<double>(st.inCount)));
        add(e, "keepCount", JsonValue::MakeNumber(static_cast<double>(st.keepCount)));
        add(e, "bestSeed", JsonValue::MakeNumber(static_cast<double>(st.bestSeed)));
        add(e, "bestSeedHex", JsonValue::MakeString(HexU64(st.bestSeed)));
        add(e, "bestScore", JsonValue::MakeNumber(st.bestScore));
        stages.arrayValue.push_back(std::move(e));
      }
      add(sh, "stages", std::move(stages));
      add(root, "successiveHalving", std::move(sh));
    }

    add(root, "paretoEnabled", JsonValue::MakeBool(pareto));
    add(root, "mapElitesEnabled", JsonValue::MakeBool(mapElites));
    add(root, "outliersEnabled", JsonValue::MakeBool(outliers));
    add(root, "clustersEnabled", JsonValue::MakeBool(clusterCfg.k > 0));

    if (clusterCfg.k > 0) {
      JsonValue cc = JsonValue::MakeObject();
      add(cc, "k", JsonValue::MakeNumber(static_cast<double>(clusterCfg.k)));
      add(cc, "space", JsonValue::MakeString(MineDiversityModeName(clusterCfg.space)));
      add(cc, "layoutWeight", JsonValue::MakeNumber(clusterCfg.layoutWeight));
      add(cc, "robustScaling", JsonValue::MakeBool(clusterCfg.robustScaling));
      add(cc, "maxIters", JsonValue::MakeNumber(static_cast<double>(clusterCfg.maxIters)));
      {
        JsonValue ms = JsonValue::MakeArray();
        for (MineMetric m : clusterCfg.metrics) {
          ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
        }
        add(cc, "metrics", std::move(ms));
      }
      add(root, "clusterConfig", std::move(cc));

      JsonValue cr = JsonValue::MakeObject();
      add(cr, "attempted", JsonValue::MakeBool(clusterCfg.k > 0 && !top.empty()));
      add(cr, "ok", JsonValue::MakeBool(clusterRes.ok));
      if (!clusterRes.warning.empty()) add(cr, "warning", JsonValue::MakeString(clusterRes.warning));
      add(cr, "avgSilhouette", JsonValue::MakeNumber(clusterRes.avgSilhouette));
      add(cr, "totalCost", JsonValue::MakeNumber(clusterRes.totalCost));
      {
        JsonValue sizes = JsonValue::MakeArray();
        for (int s : clusterRes.clusterSizes) sizes.arrayValue.push_back(JsonValue::MakeNumber(static_cast<double>(s)));
        add(cr, "clusterSizes", std::move(sizes));
      }
      {
        JsonValue assigns = JsonValue::MakeArray();
        for (int a : clusterRes.assignment) assigns.arrayValue.push_back(JsonValue::MakeNumber(static_cast<double>(a)));
        add(cr, "assignment", std::move(assigns));
      }
      {
        JsonValue med = JsonValue::MakeArray();
        for (std::size_t c = 0; c < clusterRes.medoidRecIndex.size(); ++c) {
          const int recIndex = clusterRes.medoidRecIndex[c];
          if (recIndex < 0 || static_cast<std::size_t>(recIndex) >= recs.size()) continue;
          const MineRecord& r = recs[static_cast<std::size_t>(recIndex)];
          JsonValue m = JsonValue::MakeObject();
          add(m, "cluster", JsonValue::MakeNumber(static_cast<double>(c)));
          add(m, "recIndex", JsonValue::MakeNumber(static_cast<double>(recIndex)));
          add(m, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
          add(m, "seedHex", JsonValue::MakeString(HexU64(r.seed)));
          add(m, "score", JsonValue::MakeNumber(r.score));
          med.arrayValue.push_back(std::move(m));
        }
        add(cr, "medoids", std::move(med));
      }
      {
        JsonValue sel = JsonValue::MakeArray();
        for (std::size_t i = 0; i < clusterRes.selectedIndices.size(); ++i) {
          const int recIndex = clusterRes.selectedIndices[i];
          if (recIndex < 0 || static_cast<std::size_t>(recIndex) >= recs.size()) continue;
          const MineRecord& r = recs[static_cast<std::size_t>(recIndex)];
          JsonValue e = JsonValue::MakeObject();
          add(e, "rank", JsonValue::MakeNumber(static_cast<double>(i + 1)));
          add(e, "recIndex", JsonValue::MakeNumber(static_cast<double>(recIndex)));
          add(e, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
          add(e, "seedHex", JsonValue::MakeString(HexU64(r.seed)));
          const int cl = (i < clusterRes.assignment.size()) ? clusterRes.assignment[i] : -1;
          add(e, "cluster", JsonValue::MakeNumber(static_cast<double>(cl)));
          add(e, "score", JsonValue::MakeNumber(r.score));
          sel.arrayValue.push_back(std::move(e));
        }
        add(cr, "selected", std::move(sel));
      }
      add(root, "clusters", std::move(cr));
    }
    if (outliers) {
      JsonValue oc = JsonValue::MakeObject();
      add(oc, "k", JsonValue::MakeNumber(static_cast<double>(outlierCfg.k)));
      add(oc, "space", JsonValue::MakeString(MineDiversityModeName(outlierCfg.space)));
      add(oc, "layoutWeight", JsonValue::MakeNumber(outlierCfg.layoutWeight));
      add(oc, "robustScaling", JsonValue::MakeBool(outlierCfg.robustScaling));
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : outlierCfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      add(oc, "metrics", std::move(ms));
      add(root, "outlierConfig", std::move(oc));
    }
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

    if (mapElites) {
      JsonValue me = JsonValue::MakeObject();

      // Axes.
      JsonValue x = JsonValue::MakeObject();
      add(x, "metric", JsonValue::MakeString(MineMetricName(meRes.cfg.x.metric)));
      add(x, "bins", JsonValue::MakeNumber(static_cast<double>(meRes.cfg.x.bins)));
      add(x, "autoRange", JsonValue::MakeBool(meRes.cfg.x.autoRange));
      add(x, "min", JsonValue::MakeNumber(meRes.cfg.x.min));
      add(x, "max", JsonValue::MakeNumber(meRes.cfg.x.max));
      add(me, "x", std::move(x));

      JsonValue y = JsonValue::MakeObject();
      add(y, "metric", JsonValue::MakeString(MineMetricName(meRes.cfg.y.metric)));
      add(y, "bins", JsonValue::MakeNumber(static_cast<double>(meRes.cfg.y.bins)));
      add(y, "autoRange", JsonValue::MakeBool(meRes.cfg.y.autoRange));
      add(y, "min", JsonValue::MakeNumber(meRes.cfg.y.min));
      add(y, "max", JsonValue::MakeNumber(meRes.cfg.y.max));
      add(me, "y", std::move(y));

      // Quality.
      add(me, "qualityMetric", JsonValue::MakeString(MineMetricName(meRes.cfg.qualityMetric)));
      add(me, "qualityMaximize", JsonValue::MakeBool(meRes.cfg.qualityMaximize));
      add(me, "clampToBounds", JsonValue::MakeBool(meRes.cfg.clampToBounds));

      // Summary stats.
      add(me, "filledCells", JsonValue::MakeNumber(static_cast<double>(meRes.filledCells)));
      add(me, "coverage", JsonValue::MakeNumber(meRes.coverage));
      add(me, "qdScore", JsonValue::MakeNumber(meRes.qdScore));

      add(root, "mapElites", std::move(me));
    }

    add(root, "hydroEnabled", JsonValue::MakeBool(hydro));
    add(root, "seaLevel", JsonValue::MakeNumber(static_cast<double>(seaLevel)));

    if (wroteGallery) {
      JsonValue g = JsonValue::MakeObject();
      add(g, "outDir", JsonValue::MakeString(galleryRes.outDir.generic_string()));
      if (!galleryRes.indexHtml.empty()) {
        add(g, "indexHtml", JsonValue::MakeString(galleryRes.indexHtml.filename().generic_string()));
      }
      if (!galleryRes.jsonManifest.empty()) {
        add(g, "manifest", JsonValue::MakeString(galleryRes.jsonManifest.filename().generic_string()));
      }
      if (!galleryRes.contactSheet.empty()) {
        add(g, "contactSheet", JsonValue::MakeString(galleryRes.contactSheet.filename().generic_string()));
      }
      if (!galleryRes.embeddingJson.empty()) {
        add(g, "embedding", JsonValue::MakeString(galleryRes.embeddingJson.filename().generic_string()));
      }
      if (!galleryRes.neighborsJson.empty()) {
        add(g, "neighbors", JsonValue::MakeString(galleryRes.neighborsJson.filename().generic_string()));
      }
      if (!galleryRes.tracesJson.empty()) {
        add(g, "traces", JsonValue::MakeString(galleryRes.tracesJson.filename().generic_string()));
      }
      add(root, "gallery", std::move(g));
    }

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
      JsonValue rj = MineRecordToJson(recs[static_cast<std::size_t>(id)]);
      if (haveClusters && static_cast<std::size_t>(id) < recCluster.size()) {
        const int cl = recCluster[static_cast<std::size_t>(id)];
        if (cl >= 0 && rj.isObject()) {
          rj.objectValue.emplace_back("cluster", JsonValue::MakeNumber(static_cast<double>(cl)));
        }
      }
      arr.arrayValue.push_back(std::move(rj));
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
