#pragma once

#include "isocity/DepressionFill.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/MineExpr.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Seed mining ("city mining")
//
// ProcIsoCity is a procedural generator; sometimes interesting worlds are rare
// seeds that produce unusually good (or unusually chaotic) outcomes under the
// simulator. The seed miner runs a batch of worlds, simulates them for N days,
// computes KPI metrics, and ranks seeds by an objective score.
//
// This module intentionally lives in isocity_core (no raylib) so both headless
// tools and the interactive game can reuse the same mining implementation.
// -----------------------------------------------------------------------------

enum class MineObjective : std::uint8_t {
  Balanced = 0,
  Growth = 1,
  Resilient = 2,
  Chaos = 3,
};

const char* MineObjectiveName(MineObjective o);

// Parse objective from a string (case-insensitive). Accepts common aliases.
bool ParseMineObjective(const std::string& s, MineObjective& out);

// Utility: format as a hex string with 0x prefix (fixed width 16).
std::string HexU64(std::uint64_t v);


enum class MineSeedSampler : std::uint8_t {
  // Seed = seedStart + i*seedStep
  Linear = 0,

  // Seed = SplitMix64(seedStart + i*seedStep) (pseudo-random but deterministic).
  SplitMix64 = 1,

  // Seed = bit_reverse(seedStart + i*seedStep) (van der Corput base-2 low-discrepancy sequence).
  VanDerCorput2 = 2,

  // 2D low-discrepancy (Halton base2/base3) folded into a 64-bit seed via Morton (Z-order) interleaving.
  Halton23 = 3,
};

const char* MineSeedSamplerName(MineSeedSampler s);

// Parse sampler from a string (case-insensitive). Accepts common aliases.
bool ParseMineSeedSampler(const std::string& s, MineSeedSampler& out);

struct MineRecord {
  std::uint64_t seed = 0;
  int w = 0;
  int h = 0;

  Stats stats{};

  // Tile counts.
  int waterTiles = 0;
  int roadTiles = 0;
  int resTiles = 0;
  int comTiles = 0;
  int indTiles = 0;
  int parkTiles = 0;

  int schoolTiles = 0;
  int hospitalTiles = 0;
  int policeTiles = 0;
  int fireTiles = 0;

  double waterFrac = 0.0;
  double roadFrac = 0.0;
  double zoneFrac = 0.0;
  double parkFrac = 0.0;

  // Hydrology.
  int seaFloodCells = 0;
  double seaFloodFrac = 0.0;
  double seaMaxDepth = 0.0;

  int pondCells = 0;
  double pondFrac = 0.0;
  double pondMaxDepth = 0.0;
  double pondVolume = 0.0;

  // Objective score used for ranking/selection. This may be overridden by MineConfig::scoreExpr.
  double score = 0.0;

  // Raw MineObjective score before applying MineConfig::scoreExpr.
  double objectiveScore = 0.0;

  // Layout signature (perceptual hash) for diversity/similarity workflows.
  // This is a compact 64-bit hash derived from a downsampled terrain+overlay map.
  std::uint64_t overlayPHash = 0;

  // Pareto analysis (optional): populated by ComputePareto()/SelectTopParetoIndices().
  // Rank 0 is the non-dominated front. Crowding distance follows the NSGA-II
  // convention (front boundaries get +inf).
  int paretoRank = -1;
  double paretoCrowding = 0.0;

  // Outlier / novelty analysis (optional): populated by ComputeLocalOutlierFactor().
  //
  // - outlierLof: LOF>1 indicates the record is in a locally sparse region.
  // - novelty: mean kNN distance in the chosen feature space.
  double outlierLof = 0.0;
  double novelty = 0.0;
};

// ----------------------------------------------------------------------------
// Multi-objective mining (Pareto / NSGA-II style selection)
//
// The existing seed miner produces a single scalar score (MineObjective) for
// simple ranking. For exploratory workflows it is often more useful to treat
// the KPIs as a multi-objective problem and keep a diverse set of
// non-dominated candidates.
// ----------------------------------------------------------------------------

enum class MineMetric : std::uint8_t {
  Population = 0,
  Happiness = 1,
  Money = 2,
  AvgLandValue = 3,
  TrafficCongestion = 4,
  GoodsSatisfaction = 5,
  ServicesOverallSatisfaction = 6,

  WaterFrac = 7,
  RoadFrac = 8,
  ZoneFrac = 9,
  ParkFrac = 10,

  SeaFloodFrac = 11,
  SeaMaxDepth = 12,
  PondFrac = 13,
  PondMaxDepth = 14,
  PondVolume = 15,

  // Derived: combines sea + ponding signals into a single severity proxy.
  FloodRisk = 16,

  // The scalar score used for ranking/selection. If MineConfig::scoreExpr is set,
  // this is the expression result.
  Score = 17,

  // The raw MineObjective score (before scoreExpr). Always populated.
  ObjectiveScore = 18,
};

const char* MineMetricName(MineMetric m);

// Parse metric from a string (case-insensitive). Accepts common aliases.
bool ParseMineMetric(const std::string& s, MineMetric& out);

// Compute a metric value from a MineRecord.
double MineMetricValue(const MineRecord& r, MineMetric m);

struct ParetoObjective {
  MineMetric metric = MineMetric::Population;
  bool maximize = true;
};

struct ParetoResult {
  // Pareto rank per record (0 = nondominated front). Size == recs.size().
  std::vector<int> rank;

  // NSGA-II crowding distance per record (inf for front boundaries). Size == recs.size().
  std::vector<double> crowding;

  // Fronts[k] contains indices of records in the k-th Pareto front.
  std::vector<std::vector<int>> fronts;
};

// Compute Pareto fronts + crowding distance for the given objectives.
//
// Notes:
// - Complexity is O(N^2 * M) which is fine for typical mining sample counts.
// - If objectives is empty, all records are assigned rank 0.
ParetoResult ComputePareto(const std::vector<MineRecord>& recs,
                           const std::vector<ParetoObjective>& objectives);

// Select top-K indices using Pareto rank then crowding distance (NSGA-II style).
// If useCrowding is false, the final partially-selected front is tie-broken by
// score (desc) then seed (asc).
std::vector<int> SelectTopParetoIndices(const ParetoResult& pr, int topK, bool useCrowding = true);

// ----------------------------------------------------------------------------
// Quality-Diversity (MAP-Elites) selection
//
// MAP-Elites partitions a 2D "behavior space" into bins and keeps the best
// (highest-quality) record discovered in each cell. This is useful for mining
// a *diverse* set of interesting seeds rather than a single optimum.
// ----------------------------------------------------------------------------

struct MapElitesAxis {
  MineMetric metric = MineMetric::WaterFrac;
  int bins = 10;

  // If true, infer [min,max] from the provided records.
  bool autoRange = true;

  // Used when autoRange==false (and optionally as initial hints).
  double min = 0.0;
  double max = 1.0;
};

struct MapElitesConfig {
  MapElitesAxis x{};
  MapElitesAxis y{};

  // Quality (fitness) measure used to decide which record wins a cell.
  MineMetric qualityMetric = MineMetric::Score;
  bool qualityMaximize = true;

  // If false, records outside the axis ranges are ignored.
  // If true, values are clamped into [min,max] before binning.
  bool clampToBounds = true;
};

struct MapElitesResult {
  MapElitesConfig cfg{};

  // Grid of size (x.bins * y.bins). Each element is an index into the input
  // record array, or -1 if empty.
  std::vector<int> grid;

  int filledCells = 0;
  double coverage = 0.0;
  double qdScore = 0.0; // sum of quality scores over filled cells
};

// Compute MAP-Elites grid for the given records and configuration.
MapElitesResult ComputeMapElites(const std::vector<MineRecord>& recs, const MapElitesConfig& cfg);

// Select top-K record indices from a MAP-Elites result (sorted by quality).
std::vector<int> SelectTopMapElitesIndices(const MapElitesResult& me,
                                           const std::vector<MineRecord>& recs,
                                           int topK);


struct MineConfig {
  std::uint64_t seedStart = 1;
  std::uint64_t seedStep = 1;
  int samples = 100;

  // Seed enumeration strategy: controls how the i-th sample index is turned into a u64 seed.
  //
  // - Linear: seed = seedStart + i*seedStep
  // - SplitMix64: seed = SplitMix64(seedStart + i*seedStep)  (pseudo-random, deterministic)
  // - VanDerCorput2: seed = bit_reverse(seedStart + i*seedStep)  (low-discrepancy)
  // - Halton23: 2D low-discrepancy (base2/base3) folded into 64 bits via Morton interleaving
  MineSeedSampler seedSampler = MineSeedSampler::Linear;

  // Optional final XOR applied to the generated seed (digital shift / scrambling).
  std::uint64_t seedXor = 0;

  int w = 96;
  int h = 96;
  int days = 120;

  // Mining parallelism.
  //
  // - 1: deterministic single-thread mining (default).
  // - >1: parallel mining with deterministic output order.
  // - <=0: auto (uses std::thread::hardware_concurrency()).
  int threads = 1;

  MineObjective objective = MineObjective::Balanced;

  // Optional MineExpr string. If non-empty, the expression is evaluated per MineRecord
  // and its result becomes MineRecord::score. The original objective score is stored
  // in MineRecord::objectiveScore.
  std::string scoreExpr;

  // Hydrology analysis (sea flooding + depression filling).
  bool hydrologyEnabled = true;

  // If finite, overrides procCfg.waterLevel.
  float seaLevelOverride = std::numeric_limits<float>::quiet_NaN();

  // Sea flood connectivity options.
  bool seaRequireEdgeConnection = true;
  bool seaEightConnected = false;

  // Priority-Flood depression fill epsilon (0 preserves perfectly flat spill surfaces).
  float depressionEpsilon = 0.0f;
};

struct MineProgress {
  // 0-based index of the record that was just produced.
  int index = 0;
  int total = 0;
  const MineRecord* record = nullptr;
};

using MineProgressFn = std::function<void(const MineProgress&)>;

// Compute the actual u64 seed for the given 0-based sample index, according to
// cfg.seedStart/cfg.seedStep/cfg.seedSampler (and cfg.seedXor).
//
// This is the canonical seed enumeration used by MineSeeds() and MineSession.
std::uint64_t MineSeedForSample(const MineConfig& cfg, std::uint64_t sampleIndex);

// Batch mine seeds according to cfg, using ProcGenConfig and SimConfig.
//
// If progress is provided, it is called after each record is produced.
bool MineSeeds(const MineConfig& cfg,
              const ProcGenConfig& procCfg,
              const SimConfig& simCfg,
              std::vector<MineRecord>& outRecords,
              std::string* outError,
              const MineProgressFn& progress = MineProgressFn());

std::vector<MineRecord> MineSeeds(const MineConfig& cfg,
                                 const ProcGenConfig& procCfg,
                                 const SimConfig& simCfg,
                                 const MineProgressFn& progress = MineProgressFn());

// Mine an explicit list of seeds using the provided configs.
//
// This is useful for staged / adaptive mining strategies (e.g., successive
// halving), for resuming from a custom seed manifest, or for UI workflows where
// the candidate seed set is generated by an algorithm rather than a linear scan.
//
// Notes:
//  - cfg.seedStart/cfg.seedStep/cfg.samples are ignored; `seeds.size()` defines
//    the batch size.
//  - Output order matches the `seeds` input order (deterministic).
//  - Parallel mining is supported via cfg.threads with the same determinism
//    guarantees as MineSeeds().
bool MineSeedsExplicit(const MineConfig& cfg,
                      const ProcGenConfig& procCfg,
                      const SimConfig& simCfg,
                      const std::vector<std::uint64_t>& seeds,
                      std::vector<MineRecord>& outRecords,
                      std::string* outError,
                      const MineProgressFn& progress = MineProgressFn());

std::vector<MineRecord> MineSeedsExplicit(const MineConfig& cfg,
                                         const ProcGenConfig& procCfg,
                                         const SimConfig& simCfg,
                                         const std::vector<std::uint64_t>& seeds,
                                         const MineProgressFn& progress = MineProgressFn());

// Diversity distance mode used by SelectTopIndices() when diverse=1.
//
// - Scalar: classic KPI-feature diversity (population density, happiness, congestion, ...).
// - Layout: pHash-based diversity on the city layout signature.
// - Hybrid: blends scalar + layout distances (see mmrLayoutWeight).
enum class MineDiversityMode : std::uint8_t {
  Scalar = 0,
  Layout = 1,
  Hybrid = 2,
};

const char* MineDiversityModeName(MineDiversityMode m);

// Parse mode from a string (case-insensitive). Accepts common aliases.
bool ParseMineDiversityMode(const std::string& s, MineDiversityMode& out);

// Select top-K indices in recs. If diverse is true, selects a diverse subset
// using a simple Maximal Marginal Relevance (MMR) heuristic.
std::vector<int> SelectTopIndices(const std::vector<MineRecord>& recs,
                                 int topK,
                                 bool diverse = true,
                                 int candidatePool = 0,
                                 double mmrScoreWeight = 0.70,
                                 MineDiversityMode mode = MineDiversityMode::Scalar,
                                 double mmrLayoutWeight = 0.50);

// ----------------------------------------------------------------------------
// Outlier / novelty mining analytics (LOF + kNN novelty)
//
// In addition to ranking seeds by a scalar score or multi-objective selection,
// it can be useful to intentionally surface *weird* cities: rare layouts or KPI
// combinations that lie in locally sparse regions of the sampled space.
//
// We implement the Local Outlier Factor (LOF) algorithm and a related "novelty"
// signal (mean kNN distance) over either:
//
// - Scalar: standardized KPI feature space
// - Layout: pHash Hamming distance space
// - Hybrid: weighted sum of scalar + layout distances
// ----------------------------------------------------------------------------

struct OutlierConfig {
  // k used for kNN/LOF. Typical values: 10-50.
  int k = 20;

  // Feature space used for distances.
  MineDiversityMode space = MineDiversityMode::Scalar;

  // Used when space==Hybrid (ignored otherwise). In [0,1].
  double layoutWeight = 0.50;

  // If true, use robust standardization per metric (median + MAD). If false,
  // use mean/stddev standardization.
  bool robustScaling = true;

  // Metrics used when space is Scalar or Hybrid.
  // If empty, a default set of metrics is used.
  std::vector<MineMetric> metrics;
};

struct OutlierResult {
  OutlierConfig cfg{};

  // Size == recs.size(). Values near 1.0 are "normal"; >1 indicates an outlier.
  std::vector<double> lof;

  // Size == recs.size(). Mean distance to k nearest neighbors (plain metric
  // distance, not reachability distance). Higher means "more novel".
  std::vector<double> novelty;
};

// Compute Local Outlier Factor (LOF) and kNN novelty for the given records.
OutlierResult ComputeLocalOutlierFactor(const std::vector<MineRecord>& recs, const OutlierConfig& cfg);

// Select top-K indices by outlierLof (descending), tie-broken by seed (ascending).
std::vector<int> SelectTopOutlierIndices(const std::vector<MineRecord>& recs, int topK);



// Incremental miner suitable for UI integration (spread work across frames).
class MineSession {
public:
  MineSession(MineConfig cfg, ProcGenConfig procCfg, SimConfig simCfg);

  // Mine up to maxSteps seeds; returns the number of records produced.
  int step(int maxSteps = 1, const MineProgressFn& progress = MineProgressFn());

  bool done() const { return m_index >= m_cfg.samples; }
  int index() const { return m_index; }
  int total() const { return m_cfg.samples; }

  const MineConfig& config() const { return m_cfg; }
  const ProcGenConfig& procConfig() const { return m_procCfg; }
  const SimConfig& simConfig() const { return m_simCfg; }

  const std::vector<MineRecord>& records() const { return m_records; }
  std::vector<MineRecord>& records() { return m_records; }

private:
  MineConfig m_cfg{};
  ProcGenConfig m_procCfg{};
  SimConfig m_simCfg{};

  Simulator m_sim;

  float m_seaLevel = 0.0f;
  SeaFloodConfig m_seaCfg{};
  DepressionFillConfig m_depCfg{};

  // Optional custom score expression program.
  MineExprProgram m_scoreExpr{};
  bool m_scoreExprEnabled = false;
  std::string m_scoreExprError;

  int m_index = 0;
  std::vector<MineRecord> m_records;
};

// CSV helpers.
void WriteMineCsvHeader(std::ostream& os);
void WriteMineCsvRow(std::ostream& os, const MineRecord& r);

// JSON helper for a MineRecord (for embedding into larger documents).
JsonValue MineRecordToJson(const MineRecord& r);

// Parse a MineRecord from the JSON object produced by MineRecordToJson().
//
// Notes:
//  - For exact seed roundtrips, this prefers the "seed_hex" string if present
//    (since "seed" is a JSON number and may lose precision for large u64).
//  - Missing optional fields default to 0.
//
// Returns false on parse errors. If outError is provided, it is filled with a
// human-readable message.
bool MineRecordFromJson(const JsonValue& obj, MineRecord& out, std::string* outError = nullptr);

// Convenience: parse a MineRecord from a JSON text string containing an object.
bool MineRecordFromJsonText(const std::string& text, MineRecord& out, std::string* outError = nullptr);

} // namespace isocity
