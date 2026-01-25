#include "isocity/SeedMiner.hpp"
#include "isocity/PerceptualHash.hpp"
#include "isocity/VPTree.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <ostream>
#include <sstream>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace isocity {

namespace {

// We avoid IEEE inf in exported artifacts (CSV/JSON) to keep them portable.
constexpr double kParetoCrowdingInf = 1.0e30;

static std::string NormalizeKey(const std::string& s)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    unsigned char uc = static_cast<unsigned char>(c);
    char out = static_cast<char>(std::tolower(uc));
    if (out == '-' || out == ' ' || out == '.') out = '_';
    t.push_back(out);
  }
  return t;
}

static std::uint64_t ReverseBits64(std::uint64_t x)
{
  // Bit reversal by SWAR. Deterministic across platforms/compilers.
  x = ((x & 0x5555555555555555ULL) << 1) | ((x >> 1) & 0x5555555555555555ULL);
  x = ((x & 0x3333333333333333ULL) << 2) | ((x >> 2) & 0x3333333333333333ULL);
  x = ((x & 0x0F0F0F0F0F0F0F0FULL) << 4) | ((x >> 4) & 0x0F0F0F0F0F0F0F0FULL);
  x = ((x & 0x00FF00FF00FF00FFULL) << 8) | ((x >> 8) & 0x00FF00FF00FF00FFULL);
  x = ((x & 0x0000FFFF0000FFFFULL) << 16) | ((x >> 16) & 0x0000FFFF0000FFFFULL);
  x = (x << 32) | (x >> 32);
  return x;
}

static std::uint64_t SplitMix64Hash(std::uint64_t x)
{
  // Use SplitMix64Next as a pure mixing function by copying the state.
  std::uint64_t s = x;
  return SplitMix64Next(s);
}

// Base-b radical inverse converted to a 64-bit binary fixed-point fraction.
//
// This yields floor(phi_b(n) * 2^64), where phi_b is the van der Corput radical inverse.
// Uses long division to avoid requiring >128-bit intermediates.
static std::uint64_t RadicalInverseU64(std::uint64_t n, std::uint32_t base)
{
  if (base < 2u) return 0u;
  if (n == 0u) return 0u;

  __uint128_t numer = 0;
  __uint128_t denom = 1;
  while (n > 0u) {
    const std::uint64_t digit = (base == 2u) ? (n & 1u) : (n % static_cast<std::uint64_t>(base));
    n = (base == 2u) ? (n >> 1u) : (n / static_cast<std::uint64_t>(base));
    numer = numer * static_cast<std::uint64_t>(base) + digit;
    denom *= static_cast<std::uint64_t>(base);
  }

  std::uint64_t out = 0;
  __uint128_t rem = numer;
  for (int i = 0; i < 64; ++i) {
    rem *= 2;
    out <<= 1;
    if (rem >= denom) {
      rem -= denom;
      out |= 1ULL;
    }
  }
  return out;
}

static std::uint64_t Part1By1(std::uint32_t x)
{
  // Expand 32 bits into 64 by inserting a 0 bit between each original bit.
  std::uint64_t v = static_cast<std::uint64_t>(x);
  v = (v | (v << 16)) & 0x0000FFFF0000FFFFULL;
  v = (v | (v << 8)) & 0x00FF00FF00FF00FFULL;
  v = (v | (v << 4)) & 0x0F0F0F0F0F0F0F0FULL;
  v = (v | (v << 2)) & 0x3333333333333333ULL;
  v = (v | (v << 1)) & 0x5555555555555555ULL;
  return v;
}

static std::uint64_t Morton2D32(std::uint32_t x, std::uint32_t y)
{
  // Interleave x/y bits: x0,y0,x1,y1,...
  return Part1By1(x) | (Part1By1(y) << 1);
}

} // namespace

namespace {

static std::vector<float> ExtractHeights(const World& world)
{
  std::vector<float> heights;
  const int w = world.width();
  const int h = world.height();
  heights.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          world.at(x, y).height;
    }
  }
  return heights;
}

static std::vector<std::uint8_t> BuildWaterDrainMask(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  std::vector<std::uint8_t> mask;
  mask.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          (t.terrain == Terrain::Water) ? 1 : 0;
    }
  }
  return mask;
}

static void ComputeTileCounts(const World& world, MineRecord& r)
{
  r.waterTiles = 0;
  r.roadTiles = 0;
  r.resTiles = 0;
  r.comTiles = 0;
  r.indTiles = 0;
  r.parkTiles = 0;
  r.schoolTiles = 0;
  r.hospitalTiles = 0;
  r.policeTiles = 0;
  r.fireTiles = 0;

  const int w = world.width();
  const int h = world.height();

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) r.waterTiles++;
      switch (t.overlay) {
      case Overlay::Road: r.roadTiles++; break;
      case Overlay::Residential: r.resTiles++; break;
      case Overlay::Commercial: r.comTiles++; break;
      case Overlay::Industrial: r.indTiles++; break;
      case Overlay::Park: r.parkTiles++; break;
      case Overlay::School: r.schoolTiles++; break;
      case Overlay::Hospital: r.hospitalTiles++; break;
      case Overlay::PoliceStation: r.policeTiles++; break;
      case Overlay::FireStation: r.fireTiles++; break;
      default: break;
      }
    }
  }

  const double denom = std::max(1.0, static_cast<double>(w) * static_cast<double>(h));
  r.waterFrac = static_cast<double>(r.waterTiles) / denom;
  r.roadFrac = static_cast<double>(r.roadTiles) / denom;
  r.zoneFrac = static_cast<double>(r.resTiles + r.comTiles + r.indTiles) / denom;
  r.parkFrac = static_cast<double>(r.parkTiles) / denom;
}

struct ScoreWeights {
  // Positive terms.
  double wPopulation = 1.0;
  double wHappiness = 1800.0;
  double wMoney = 0.05;
  double wLandValue = 900.0;
  double wGoodsSatisfaction = 700.0;
  double wServicesSatisfaction = 500.0;

  // Penalties.
  double pCongestion = 1400.0;
  double pSeaFrac = 1000.0;
  double pSeaMaxDepth = 2500.0;
  double pPondFrac = 700.0;
  double pPondMaxDepth = 2000.0;
};

static ScoreWeights WeightsForObjective(MineObjective obj)
{
  ScoreWeights w;
  switch (obj) {
  case MineObjective::Balanced:
    // default weights
    return w;
  case MineObjective::Growth:
    w.wPopulation = 1.4;
    w.wMoney = 0.08;
    w.pCongestion = 1000.0;
    w.pSeaFrac = 650.0;
    w.pSeaMaxDepth = 1600.0;
    w.pPondFrac = 500.0;
    w.pPondMaxDepth = 1200.0;
    return w;
  case MineObjective::Resilient:
    w.wPopulation = 0.9;
    w.wHappiness = 2000.0;
    w.pCongestion = 1500.0;
    w.pSeaFrac = 1600.0;
    w.pSeaMaxDepth = 5200.0;
    w.pPondFrac = 1400.0;
    w.pPondMaxDepth = 4200.0;
    return w;
  case MineObjective::Chaos:
    // Invert the "health" incentives: find worlds that are likely to stress-test
    // flooding, ponding, and congestion behavior.
    w.wPopulation = 0.2;
    w.wHappiness = -1200.0; // prefer unhappy
    w.wMoney = -0.05;       // prefer broke
    w.wLandValue = -700.0;
    w.wGoodsSatisfaction = -600.0;
    w.wServicesSatisfaction = -600.0;

    w.pCongestion = -2500.0; // negative penalty = reward
    w.pSeaFrac = -1800.0;
    w.pSeaMaxDepth = -5200.0;
    w.pPondFrac = -2200.0;
    w.pPondMaxDepth = -6200.0;
    return w;
  }
  return w;
}

static double ComputeScore(const MineRecord& r, const ScoreWeights& w)
{
  const double pop = static_cast<double>(std::max(0, r.stats.population));

  // Happiness matters more once you have a meaningful city.
  const double happyScale = 0.10 * pop + 500.0;
  const double servicesScale = 0.05 * pop + 250.0;

  // Normalize some [0,1] metrics to a ~1000 scale so weights are readable.
  const double kUnit = 1000.0;

  double score = 0.0;
  score += w.wPopulation * pop;
  score += w.wHappiness * static_cast<double>(r.stats.happiness) * happyScale;
  score += w.wMoney * static_cast<double>(r.stats.money);
  score += w.wLandValue * static_cast<double>(r.stats.avgLandValue) * kUnit;
  score += w.wGoodsSatisfaction * static_cast<double>(r.stats.goodsSatisfaction) * (0.25 * kUnit);
  score += w.wServicesSatisfaction * static_cast<double>(r.stats.servicesOverallSatisfaction) * servicesScale;

  score -= w.pCongestion * static_cast<double>(r.stats.trafficCongestion) * (0.05 * pop + 200.0);
  score -= w.pSeaFrac * r.seaFloodFrac * kUnit;
  score -= w.pSeaMaxDepth * r.seaMaxDepth * kUnit;
  score -= w.pPondFrac * r.pondFrac * kUnit;
  score -= w.pPondMaxDepth * r.pondMaxDepth * kUnit;

  return score;
}

static std::array<double, 7> FeatureVectorRaw(const MineRecord& r)
{
  const double area = std::max(1.0, static_cast<double>(r.w) * static_cast<double>(r.h));
  const double popDensity = static_cast<double>(r.stats.population) / area;
  const double roads = static_cast<double>(r.roadTiles) / area;

  return {
      popDensity,
      static_cast<double>(r.stats.happiness),
      static_cast<double>(r.stats.trafficCongestion),
      r.seaFloodFrac,
      r.pondMaxDepth,
      static_cast<double>(r.stats.avgLandValue),
      roads,
  };
}

static double EuclidDist(const std::array<double, 7>& a, const std::array<double, 7>& b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = a[i] - b[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

static MineRecord MineOneSeed(std::uint64_t seed,
                             int w,
                             int h,
                             int days,
                             const ProcGenConfig& procCfg,
                             Simulator& sim,
                             bool hydro,
                             float seaLevel,
                             const SeaFloodConfig& seaCfg,
                             const DepressionFillConfig& depCfg,
                             const ScoreWeights& weights)
{
  World world = GenerateWorld(w, h, seed, procCfg);

  // Reset sim accumulator for determinism across worlds.
  sim.resetTimer();

  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
  }

  // Ensure derived stats are fresh even when days==0.
  sim.refreshDerivedStats(world);

  MineRecord r;
  r.seed = seed;
  r.w = w;
  r.h = h;
  r.stats = world.stats();

  ComputeTileCounts(world, r);

  // Compact layout signature (terrain + overlay) for diversity/similarity workflows.
  r.overlayPHash = ComputeWorldOverlayPHash(world);

  if (hydro) {
    const std::vector<float> heights = ExtractHeights(world);

    const SeaFloodResult sea = ComputeSeaLevelFlood(heights, w, h, seaLevel, seaCfg);
    r.seaFloodCells = sea.floodedCells;
    r.seaMaxDepth = static_cast<double>(sea.maxDepth);
    r.seaFloodFrac = (w > 0 && h > 0) ? (static_cast<double>(sea.floodedCells) / (static_cast<double>(w) * h)) : 0.0;

    const std::vector<std::uint8_t> drainMask = BuildWaterDrainMask(world);
    const DepressionFillResult dep = FillDepressionsPriorityFlood(heights, w, h, &drainMask, depCfg);
    r.pondCells = dep.filledCells;
    r.pondMaxDepth = static_cast<double>(dep.maxDepth);
    r.pondVolume = dep.volume;
    r.pondFrac = (w > 0 && h > 0) ? (static_cast<double>(dep.filledCells) / (static_cast<double>(w) * h)) : 0.0;
  }

  r.objectiveScore = ComputeScore(r, weights);
  r.score = r.objectiveScore;
  return r;
}

} // namespace

namespace {

template <typename SeedAtFn>
static bool MineSeedsImpl(const MineConfig& cfg,
                          const ProcGenConfig& procCfg,
                          const SimConfig& simCfg,
                          int total,
                          const SeedAtFn& seedAt,
                          std::vector<MineRecord>& outRecords,
                          std::string* outError,
                          const MineProgressFn& progress)
{
  if (outError) outError->clear();
  outRecords.clear();
  if (total <= 0 || cfg.w <= 0 || cfg.h <= 0) return true;

  outRecords.resize(static_cast<std::size_t>(total));

  SeaFloodConfig seaCfg;
  seaCfg.requireEdgeConnection = cfg.seaRequireEdgeConnection;
  seaCfg.eightConnected = cfg.seaEightConnected;

  DepressionFillConfig depCfg;
  depCfg.includeEdges = true;
  depCfg.epsilon = cfg.depressionEpsilon;

  const float seaLevel = std::isfinite(cfg.seaLevelOverride) ? cfg.seaLevelOverride : procCfg.waterLevel;
  const ScoreWeights weights = WeightsForObjective(cfg.objective);

  // Optional custom scoring expression.
  MineExprProgram scoreProg;
  const bool useScoreExpr = !cfg.scoreExpr.empty();
  if (useScoreExpr) {
    std::string err;
    if (!CompileMineExpr(cfg.scoreExpr, scoreProg, err)) {
      if (outError) *outError = err;
      return false;
    }
  }

  constexpr double kScoreClamp = 1.0e30;
  constexpr double kBadScore = -1.0e30;

  auto applyScoreExpr = [&](MineRecord& r) {
    if (!useScoreExpr) return;

    double v = 0.0;
    // Runtime failures should be impossible if compilation succeeded, but we
    // still guard and degrade the score to a safe sentinel.
    if (!EvalMineExpr(scoreProg, r, v, nullptr) || !std::isfinite(v)) {
      v = kBadScore;
    }

    if (v > kScoreClamp) v = kScoreClamp;
    if (v < -kScoreClamp) v = -kScoreClamp;
    r.score = v;
  };

  int threads = cfg.threads;
  if (threads <= 0) threads = static_cast<int>(std::thread::hardware_concurrency());
  if (threads <= 0) threads = 1;
  threads = std::min(threads, total);

  // Single-thread fallback (preserves prior behavior and is helpful for debugging).
  if (threads <= 1) {
    Simulator sim(simCfg);
    for (int i = 0; i < total; ++i) {
      const std::uint64_t seed = seedAt(i);

      outRecords[static_cast<std::size_t>(i)] =
          MineOneSeed(seed, cfg.w, cfg.h, cfg.days, procCfg, sim, cfg.hydrologyEnabled, seaLevel, seaCfg, depCfg, weights);

      applyScoreExpr(outRecords[static_cast<std::size_t>(i)]);

      if (progress) {
        MineProgress p;
        p.index = i;
        p.total = total;
        p.record = &outRecords[static_cast<std::size_t>(i)];
        progress(p);
      }
    }
    return true;
  }

  // Parallel mining. We ensure:
  //  - `outRecords` is filled at stable indices (so the returned vector order is deterministic).
  //  - progress callbacks (if any) fire in index order even though work completes out-of-order.
  std::atomic<int> nextIndex{0};

  std::mutex readyMutex;
  std::condition_variable readyCv;
  std::vector<std::uint8_t> ready;
  if (progress) {
    ready.resize(static_cast<std::size_t>(total), 0u);
  }

  auto worker = [&]() {
    Simulator sim(simCfg);

    for (;;) {
      const int i = nextIndex.fetch_add(1);
      if (i >= total) break;

      const std::uint64_t seed = seedAt(i);
      MineRecord r =
          MineOneSeed(seed, cfg.w, cfg.h, cfg.days, procCfg, sim, cfg.hydrologyEnabled, seaLevel, seaCfg, depCfg, weights);

      applyScoreExpr(r);

      outRecords[static_cast<std::size_t>(i)] = std::move(r);

      if (progress) {
        {
          std::lock_guard<std::mutex> lock(readyMutex);
          ready[static_cast<std::size_t>(i)] = 1u;
        }
        readyCv.notify_all();
      }
    }
  };

  std::vector<std::thread> pool;
  pool.reserve(static_cast<std::size_t>(threads));
  for (int t = 0; t < threads; ++t) {
    pool.emplace_back(worker);
  }

  if (progress) {
    for (int i = 0; i < total; ++i) {
      {
        std::unique_lock<std::mutex> lock(readyMutex);
        readyCv.wait(lock, [&]() { return ready[static_cast<std::size_t>(i)] != 0u; });
      }

      MineProgress p;
      p.index = i;
      p.total = total;
      p.record = &outRecords[static_cast<std::size_t>(i)];
      progress(p);
    }
  }

  for (std::thread& th : pool) {
    if (th.joinable()) th.join();
  }

  return true;
}

} // namespace


const char* MineMetricName(MineMetric m)
{
  switch (m) {
  case MineMetric::Population: return "population";
  case MineMetric::Happiness: return "happiness";
  case MineMetric::Money: return "money";
  case MineMetric::AvgLandValue: return "avg_land_value";
  case MineMetric::TrafficCongestion: return "traffic_congestion";
  case MineMetric::GoodsSatisfaction: return "goods_satisfaction";
  case MineMetric::ServicesOverallSatisfaction: return "services_overall_satisfaction";
  case MineMetric::WaterFrac: return "water_frac";
  case MineMetric::RoadFrac: return "road_frac";
  case MineMetric::ZoneFrac: return "zone_frac";
  case MineMetric::ParkFrac: return "park_frac";
  case MineMetric::SeaFloodFrac: return "sea_flood_frac";
  case MineMetric::SeaMaxDepth: return "sea_max_depth";
  case MineMetric::PondFrac: return "pond_frac";
  case MineMetric::PondMaxDepth: return "pond_max_depth";
  case MineMetric::PondVolume: return "pond_volume";
  case MineMetric::FloodRisk: return "flood_risk";
  case MineMetric::Score: return "score";
  case MineMetric::ObjectiveScore: return "objective_score";
  }
  return "population";
}

bool ParseMineMetric(const std::string& s, MineMetric& out)
{
  const std::string t = NormalizeKey(s);
  if (t == "population" || t == "pop") {
    out = MineMetric::Population;
    return true;
  }
  if (t == "happiness" || t == "happy") {
    out = MineMetric::Happiness;
    return true;
  }
  if (t == "money" || t == "cash" || t == "funds") {
    out = MineMetric::Money;
    return true;
  }
  if (t == "avg_land_value" || t == "land_value" || t == "landvalue" || t == "avglandvalue") {
    out = MineMetric::AvgLandValue;
    return true;
  }
  if (t == "traffic_congestion" || t == "congestion" || t == "traffic" || t == "cong") {
    out = MineMetric::TrafficCongestion;
    return true;
  }
  if (t == "goods_satisfaction" || t == "goods" || t == "goods_sat" || t == "goodssatisfaction") {
    out = MineMetric::GoodsSatisfaction;
    return true;
  }
  if (t == "services_overall_satisfaction" || t == "services" || t == "services_sat" ||
      t == "services_satisfaction" || t == "servicesoverall") {
    out = MineMetric::ServicesOverallSatisfaction;
    return true;
  }
  if (t == "water_frac" || t == "water") {
    out = MineMetric::WaterFrac;
    return true;
  }
  if (t == "road_frac" || t == "roads" || t == "road") {
    out = MineMetric::RoadFrac;
    return true;
  }
  if (t == "zone_frac" || t == "zones" || t == "zone") {
    out = MineMetric::ZoneFrac;
    return true;
  }
  if (t == "park_frac" || t == "parks" || t == "park") {
    out = MineMetric::ParkFrac;
    return true;
  }
  if (t == "sea_flood_frac" || t == "sea_flood" || t == "seafloodfrac") {
    out = MineMetric::SeaFloodFrac;
    return true;
  }
  if (t == "sea_max_depth" || t == "sea_depth" || t == "seamaxdepth") {
    out = MineMetric::SeaMaxDepth;
    return true;
  }
  if (t == "pond_frac" || t == "ponding_frac" || t == "pond") {
    out = MineMetric::PondFrac;
    return true;
  }
  if (t == "pond_max_depth" || t == "pond_depth" || t == "pondmaxdepth") {
    out = MineMetric::PondMaxDepth;
    return true;
  }
  if (t == "pond_volume" || t == "pondvolume") {
    out = MineMetric::PondVolume;
    return true;
  }
  if (t == "flood_risk" || t == "floodrisk" || t == "hydro_risk" || t == "hydrorisk") {
    out = MineMetric::FloodRisk;
    return true;
  }
  if (t == "objective_score" || t == "obj_score" || t == "raw_score" || t == "objective") {
    out = MineMetric::ObjectiveScore;
    return true;
  }
  if (t == "score") {
    out = MineMetric::Score;
    return true;
  }
  return false;
}

const char* MineDiversityModeName(MineDiversityMode m)
{
  switch (m) {
  case MineDiversityMode::Scalar: return "scalar";
  case MineDiversityMode::Layout: return "layout";
  case MineDiversityMode::Hybrid: return "hybrid";
  }
  return "scalar";
}

bool ParseMineDiversityMode(const std::string& s, MineDiversityMode& out)
{
  const std::string t = NormalizeKey(s);
  if (t == "scalar" || t == "kpi" || t == "metrics") {
    out = MineDiversityMode::Scalar;
    return true;
  }
  if (t == "layout" || t == "phash" || t == "p_hash" || t == "hash") {
    out = MineDiversityMode::Layout;
    return true;
  }
  if (t == "hybrid" || t == "mix" || t == "mixed") {
    out = MineDiversityMode::Hybrid;
    return true;
  }
  return false;
}

double MineMetricValue(const MineRecord& r, MineMetric m)
{
  switch (m) {
  case MineMetric::Population: return static_cast<double>(r.stats.population);
  case MineMetric::Happiness: return static_cast<double>(r.stats.happiness);
  case MineMetric::Money: return static_cast<double>(r.stats.money);
  case MineMetric::AvgLandValue: return static_cast<double>(r.stats.avgLandValue);
  case MineMetric::TrafficCongestion: return static_cast<double>(r.stats.trafficCongestion);
  case MineMetric::GoodsSatisfaction: return static_cast<double>(r.stats.goodsSatisfaction);
  case MineMetric::ServicesOverallSatisfaction: return static_cast<double>(r.stats.servicesOverallSatisfaction);
  case MineMetric::WaterFrac: return r.waterFrac;
  case MineMetric::RoadFrac: return r.roadFrac;
  case MineMetric::ZoneFrac: return r.zoneFrac;
  case MineMetric::ParkFrac: return r.parkFrac;
  case MineMetric::SeaFloodFrac: return r.seaFloodFrac;
  case MineMetric::SeaMaxDepth: return r.seaMaxDepth;
  case MineMetric::PondFrac: return r.pondFrac;
  case MineMetric::PondMaxDepth: return r.pondMaxDepth;
  case MineMetric::PondVolume: return r.pondVolume;
  case MineMetric::FloodRisk: {
    // A simple, unitless proxy that combines fraction flooded + max depth signals.
    // Depth terms are lightly down-weighted (they tend to have a narrower range).
    constexpr double kDepthScale = 0.25;
    return r.seaFloodFrac + r.pondFrac + kDepthScale * r.seaMaxDepth + kDepthScale * r.pondMaxDepth;
  }
  case MineMetric::Score: return r.score;
  case MineMetric::ObjectiveScore: return r.objectiveScore;
  }
  return 0.0;
}

namespace {

static bool Dominates(const std::vector<double>& values, int a, int b, int m)
{
  bool anyStrict = false;
  const std::size_t baseA = static_cast<std::size_t>(a) * static_cast<std::size_t>(m);
  const std::size_t baseB = static_cast<std::size_t>(b) * static_cast<std::size_t>(m);
  for (int k = 0; k < m; ++k) {
    const double va = values[baseA + static_cast<std::size_t>(k)];
    const double vb = values[baseB + static_cast<std::size_t>(k)];
    if (va < vb) return false;
    if (va > vb) anyStrict = true;
  }
  return anyStrict;
}

static void ComputeCrowding(const std::vector<double>& values,
                            int m,
                            const std::vector<int>& front,
                            std::vector<double>& crowdingOut)
{
  if (front.empty()) return;
  if (front.size() <= 2) {
    for (int idx : front) crowdingOut[static_cast<std::size_t>(idx)] = kParetoCrowdingInf;
    return;
  }

  // For each objective, sort the front and accumulate normalized neighbor distances.
  std::vector<int> order = front;
  for (int obj = 0; obj < m; ++obj) {
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
      const double va = values[static_cast<std::size_t>(a) * static_cast<std::size_t>(m) + static_cast<std::size_t>(obj)];
      const double vb = values[static_cast<std::size_t>(b) * static_cast<std::size_t>(m) + static_cast<std::size_t>(obj)];
      if (va == vb) return a < b;
      return va < vb;
    });

    const double vmin = values[static_cast<std::size_t>(order.front()) * static_cast<std::size_t>(m) + static_cast<std::size_t>(obj)];
    const double vmax = values[static_cast<std::size_t>(order.back()) * static_cast<std::size_t>(m) + static_cast<std::size_t>(obj)];
    const double denom = (vmax > vmin) ? (vmax - vmin) : 0.0;

    // Boundary points get an effectively-infinite crowding distance.
    crowdingOut[static_cast<std::size_t>(order.front())] = kParetoCrowdingInf;
    crowdingOut[static_cast<std::size_t>(order.back())] = kParetoCrowdingInf;

    if (denom <= 0.0) continue;

    for (std::size_t i = 1; i + 1 < order.size(); ++i) {
      const int id = order[i];
      // Once a point is marked as a boundary, keep it there.
      if (crowdingOut[static_cast<std::size_t>(id)] >= kParetoCrowdingInf * 0.5) continue;
      const double vprev = values[static_cast<std::size_t>(order[i - 1]) * static_cast<std::size_t>(m) + static_cast<std::size_t>(obj)];
      const double vnext = values[static_cast<std::size_t>(order[i + 1]) * static_cast<std::size_t>(m) + static_cast<std::size_t>(obj)];
      crowdingOut[static_cast<std::size_t>(id)] += (vnext - vprev) / denom;
    }
  }
}

} // namespace

ParetoResult ComputePareto(const std::vector<MineRecord>& recs, const std::vector<ParetoObjective>& objectives)
{
  ParetoResult pr;
  const int n = static_cast<int>(recs.size());
  if (n <= 0) return pr;

  pr.rank.assign(static_cast<std::size_t>(n), 0);
  pr.crowding.assign(static_cast<std::size_t>(n), 0.0);

  const int m = static_cast<int>(objectives.size());
  if (m <= 0) {
    // Degenerate: everything is in the same front.
    pr.fronts.resize(1);
    pr.fronts[0].reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) pr.fronts[0].push_back(i);
    return pr;
  }

  // Build a transformed objective matrix where larger is always better
  // (minimization objectives are negated).
  std::vector<double> values;
  values.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(m));
  for (int i = 0; i < n; ++i) {
    for (int k = 0; k < m; ++k) {
      double v = MineMetricValue(recs[static_cast<std::size_t>(i)], objectives[static_cast<std::size_t>(k)].metric);
      if (!objectives[static_cast<std::size_t>(k)].maximize) v = -v;
      values[static_cast<std::size_t>(i) * static_cast<std::size_t>(m) + static_cast<std::size_t>(k)] = v;
    }
  }

  // NSGA-II nondominated sorting.
  std::vector<std::vector<int>> S;
  S.resize(static_cast<std::size_t>(n));
  std::vector<int> domCount(static_cast<std::size_t>(n), 0);

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const bool iDomJ = Dominates(values, i, j, m);
      const bool jDomI = !iDomJ && Dominates(values, j, i, m);
      if (iDomJ) {
        S[static_cast<std::size_t>(i)].push_back(j);
        domCount[static_cast<std::size_t>(j)]++;
      } else if (jDomI) {
        S[static_cast<std::size_t>(j)].push_back(i);
        domCount[static_cast<std::size_t>(i)]++;
      }
    }
  }

  std::vector<int> front;
  front.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    if (domCount[static_cast<std::size_t>(i)] == 0) {
      pr.rank[static_cast<std::size_t>(i)] = 0;
      front.push_back(i);
    }
  }

  int rank = 0;
  while (!front.empty()) {
    pr.fronts.push_back(front);
    std::vector<int> next;
    for (int p : front) {
      for (int q : S[static_cast<std::size_t>(p)]) {
        int& c = domCount[static_cast<std::size_t>(q)];
        c -= 1;
        if (c == 0) {
          pr.rank[static_cast<std::size_t>(q)] = rank + 1;
          next.push_back(q);
        }
      }
    }
    ++rank;
    front = std::move(next);
  }

  // Crowding distance per front.
  for (const auto& f : pr.fronts) {
    ComputeCrowding(values, m, f, pr.crowding);
  }

  return pr;
}

std::vector<int> SelectTopParetoIndices(const ParetoResult& pr, int topK, bool useCrowding)
{
  std::vector<int> out;
  if (topK <= 0) return out;
  if (pr.rank.empty()) return out;

  const int n = static_cast<int>(pr.rank.size());
  topK = std::min(topK, n);
  out.reserve(static_cast<std::size_t>(topK));

  for (const auto& front : pr.fronts) {
    if (out.size() >= static_cast<std::size_t>(topK)) break;

    std::vector<int> order = front;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
      const double ca = pr.crowding[static_cast<std::size_t>(a)];
      const double cb = pr.crowding[static_cast<std::size_t>(b)];
      if (useCrowding) {
        if (ca != cb) return ca > cb;
      }
      // Deterministic fallback: prefer lower rank (should be equal within front), then higher crowding, then index.
      const int ra = pr.rank[static_cast<std::size_t>(a)];
      const int rb = pr.rank[static_cast<std::size_t>(b)];
      if (ra != rb) return ra < rb;
      if (ca != cb) return ca > cb;
      return a < b;
    });

    for (int id : order) {
      if (out.size() >= static_cast<std::size_t>(topK)) break;
      out.push_back(id);
    }
  }

  return out;
}


// -------------------------------------------------------------------------------------------------
// MAP-Elites (quality-diversity) grid
// -------------------------------------------------------------------------------------------------

namespace {

struct ResolvedAxisRange {
  int bins = 1;
  double min = 0.0;
  double max = 1.0;
};

static ResolvedAxisRange ResolveAxisRange(const std::vector<MineRecord>& recs, const MapElitesAxis& axis)
{
  ResolvedAxisRange r;
  r.bins = std::max(1, axis.bins);

  if (axis.autoRange && !recs.empty()) {
    double lo = MineMetricValue(recs.front(), axis.metric);
    double hi = lo;
    for (const MineRecord& mr : recs) {
      const double v = MineMetricValue(mr, axis.metric);
      lo = std::min(lo, v);
      hi = std::max(hi, v);
    }
    r.min = lo;
    r.max = hi;
  } else {
    r.min = axis.min;
    r.max = axis.max;
  }

  if (!(r.max > r.min)) {
    // Degenerate range: widen by a tiny epsilon so binning doesn't divide by zero.
    r.max = r.min + 1.0e-9;
  }

  return r;
}

static bool BinForValue(double v, const ResolvedAxisRange& axis, bool clampToBounds, int& outBin)
{
  if (axis.bins <= 1) {
    outBin = 0;
    return true;
  }

  if (clampToBounds) {
    v = std::clamp(v, axis.min, axis.max);
  } else {
    if (v < axis.min || v > axis.max) return false;
  }

  const double t = (v - axis.min) / (axis.max - axis.min); // in [0,1] if clamped
  int b = static_cast<int>(std::floor(t * static_cast<double>(axis.bins)));
  if (b >= axis.bins) b = axis.bins - 1; // handle v==max
  if (b < 0) b = 0;
  outBin = b;
  return true;
}

static double QualityScore(const MineRecord& r, MineMetric m, bool maximize)
{
  const double v = MineMetricValue(r, m);
  return maximize ? v : -v;
}

} // namespace

MapElitesResult ComputeMapElites(const std::vector<MineRecord>& recs, const MapElitesConfig& cfg)
{
  MapElitesResult out;
  out.cfg = cfg;

  if (recs.empty()) return out;

  const ResolvedAxisRange ax = ResolveAxisRange(recs, cfg.x);
  const ResolvedAxisRange ay = ResolveAxisRange(recs, cfg.y);

  // Store resolved ranges so downstream exporters/UI can show the actual binning used.
  out.cfg.x.bins = ax.bins;
  out.cfg.x.min = ax.min;
  out.cfg.x.max = ax.max;

  out.cfg.y.bins = ay.bins;
  out.cfg.y.min = ay.min;
  out.cfg.y.max = ay.max;

  const int xBins = ax.bins;
  const int yBins = ay.bins;
  const int cells = xBins * yBins;

  out.grid.assign(static_cast<std::size_t>(cells), -1);

  auto better = [&](int a, int b) -> bool {
    // True if record a is strictly better (higher quality score) than record b.
    const double qa = QualityScore(recs[static_cast<std::size_t>(a)], cfg.qualityMetric, cfg.qualityMaximize);
    const double qb = QualityScore(recs[static_cast<std::size_t>(b)], cfg.qualityMetric, cfg.qualityMaximize);
    if (qa != qb) return qa > qb;

    // Deterministic tie-breakers.
    const std::uint64_t sa = recs[static_cast<std::size_t>(a)].seed;
    const std::uint64_t sb = recs[static_cast<std::size_t>(b)].seed;
    if (sa != sb) return sa < sb;
    return a < b;
  };

  for (int i = 0; i < static_cast<int>(recs.size()); ++i) {
    const MineRecord& r = recs[static_cast<std::size_t>(i)];

    int bx = 0;
    int by = 0;

    const double vx = MineMetricValue(r, cfg.x.metric);
    const double vy = MineMetricValue(r, cfg.y.metric);

    if (!BinForValue(vx, ax, cfg.clampToBounds, bx)) continue;
    if (!BinForValue(vy, ay, cfg.clampToBounds, by)) continue;

    const int cell = by * xBins + bx;
    if (cell < 0 || cell >= cells) continue;

    int& elite = out.grid[static_cast<std::size_t>(cell)];
    if (elite < 0) {
      elite = i;
    } else {
      if (better(i, elite)) elite = i;
    }
  }

  // Aggregate stats: filled cells, coverage, QD score.
  int filled = 0;
  double qd = 0.0;
  for (int id : out.grid) {
    if (id < 0) continue;
    ++filled;
    qd += QualityScore(recs[static_cast<std::size_t>(id)], cfg.qualityMetric, cfg.qualityMaximize);
  }

  out.filledCells = filled;
  out.coverage = (cells > 0) ? (static_cast<double>(filled) / static_cast<double>(cells)) : 0.0;
  out.qdScore = qd;

  return out;
}

std::vector<int> SelectTopMapElitesIndices(const MapElitesResult& me, const std::vector<MineRecord>& recs, int topK)
{
  std::vector<int> out;
  if (topK <= 0) return out;
  if (me.grid.empty() || recs.empty()) return out;

  std::vector<int> idx;
  idx.reserve(me.grid.size());
  for (int id : me.grid) {
    if (id >= 0) idx.push_back(id);
  }

  // Sort by quality (desc if maximizing; for minimize we invert in QualityScore so still desc).
  std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
    const double qa = QualityScore(recs[static_cast<std::size_t>(a)], me.cfg.qualityMetric, me.cfg.qualityMaximize);
    const double qb = QualityScore(recs[static_cast<std::size_t>(b)], me.cfg.qualityMetric, me.cfg.qualityMaximize);
    if (qa != qb) return qa > qb;

    const std::uint64_t sa = recs[static_cast<std::size_t>(a)].seed;
    const std::uint64_t sb = recs[static_cast<std::size_t>(b)].seed;
    if (sa != sb) return sa < sb;
    return a < b;
  });

  // De-dup (defensive).
  idx.erase(std::unique(idx.begin(), idx.end()), idx.end());

  const int n = std::min(topK, static_cast<int>(idx.size()));
  out.assign(idx.begin(), idx.begin() + n);
  return out;
}

const char* MineObjectiveName(MineObjective o)
{
  switch (o) {
  case MineObjective::Balanced: return "balanced";
  case MineObjective::Growth: return "growth";
  case MineObjective::Resilient: return "resilient";
  case MineObjective::Chaos: return "chaos";
  }
  return "balanced";
}

bool ParseMineObjective(const std::string& s, MineObjective& out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) t.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  if (t == "balanced" || t == "bal") {
    out = MineObjective::Balanced;
    return true;
  }
  if (t == "growth" || t == "grow") {
    out = MineObjective::Growth;
    return true;
  }
  if (t == "resilient" || t == "res") {
    out = MineObjective::Resilient;
    return true;
  }
  if (t == "chaos" || t == "stress") {
    out = MineObjective::Chaos;
    return true;
  }
  return false;
}

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

const char* MineSeedSamplerName(MineSeedSampler s)
{
  switch (s) {
  case MineSeedSampler::Linear: return "linear";
  case MineSeedSampler::SplitMix64: return "splitmix";
  case MineSeedSampler::VanDerCorput2: return "vdc2";
  case MineSeedSampler::Halton23: return "halton23";
  }
  return "linear";
}

bool ParseMineSeedSampler(const std::string& s, MineSeedSampler& out)
{
  const std::string t = NormalizeKey(s);
  if (t == "linear" || t == "seq" || t == "sequential" || t == "scan") {
    out = MineSeedSampler::Linear;
    return true;
  }
  if (t == "splitmix" || t == "splitmix64" || t == "hash" || t == "hashed" || t == "random") {
    out = MineSeedSampler::SplitMix64;
    return true;
  }
  if (t == "vdc2" || t == "vdc" || t == "vandercorput" || t == "van_der_corput" || t == "bitreverse" ||
      t == "bit_reverse" || t == "revbits" || t == "reversebits") {
    out = MineSeedSampler::VanDerCorput2;
    return true;
  }
  if (t == "halton23" || t == "halton_23" || t == "halton2_3" || t == "halton_2_3" || t == "halton") {
    out = MineSeedSampler::Halton23;
    return true;
  }
  return false;
}

std::uint64_t MineSeedForSample(const MineConfig& cfg, std::uint64_t sampleIndex)
{
  const std::uint64_t base = cfg.seedStart + sampleIndex * cfg.seedStep;

  std::uint64_t seed = 0;
  switch (cfg.seedSampler) {
  case MineSeedSampler::Linear:
    seed = base;
    break;
  case MineSeedSampler::SplitMix64:
    seed = SplitMix64Hash(base);
    break;
  case MineSeedSampler::VanDerCorput2:
    seed = ReverseBits64(base);
    break;
  case MineSeedSampler::Halton23: {
    // Build a 2D low-discrepancy point using Halton base2/base3 and fold it into 64 bits.
    const std::uint64_t u = ReverseBits64(base);        // van der Corput base-2 (exact in binary)
    const std::uint64_t v = RadicalInverseU64(base, 3); // van der Corput base-3 (converted to binary fixed-point)

    // Use coarse (high) bits for a stable Morton interleave.
    const std::uint32_t ux = static_cast<std::uint32_t>(u >> 32);
    const std::uint32_t vx = static_cast<std::uint32_t>(v >> 32);
    seed = Morton2D32(ux, vx);
    break;
  }
  }

  seed ^= cfg.seedXor;
  return seed;
}

bool MineSeeds(const MineConfig& cfg,
              const ProcGenConfig& procCfg,
              const SimConfig& simCfg,
              std::vector<MineRecord>& outRecords,
              std::string* outError,
              const MineProgressFn& progress)
{
  const int total = cfg.samples;
  if (total <= 0) {
    if (outError) outError->clear();
    outRecords.clear();
    return true;
  }

  auto seedAt = [&](int i) -> std::uint64_t {
    return MineSeedForSample(cfg, static_cast<std::uint64_t>(i));
  };

  return MineSeedsImpl(cfg, procCfg, simCfg, total, seedAt, outRecords, outError, progress);
}

bool MineSeedsExplicit(const MineConfig& cfg,
                      const ProcGenConfig& procCfg,
                      const SimConfig& simCfg,
                      const std::vector<std::uint64_t>& seeds,
                      std::vector<MineRecord>& outRecords,
                      std::string* outError,
                      const MineProgressFn& progress)
{
  const int total = static_cast<int>(seeds.size());
  if (total <= 0) {
    if (outError) outError->clear();
    outRecords.clear();
    return true;
  }

  auto seedAt = [&](int i) -> std::uint64_t {
    return seeds[static_cast<std::size_t>(i)];
  };

  return MineSeedsImpl(cfg, procCfg, simCfg, total, seedAt, outRecords, outError, progress);
}

std::vector<MineRecord> MineSeeds(const MineConfig& cfg,
                                 const ProcGenConfig& procCfg,
                                 const SimConfig& simCfg,
                                 const MineProgressFn& progress)
{
  std::vector<MineRecord> recs;
  std::string err;
  if (!MineSeeds(cfg, procCfg, simCfg, recs, &err, progress)) {
    return {};
  }
  return recs;
}

std::vector<MineRecord> MineSeedsExplicit(const MineConfig& cfg,
                                         const ProcGenConfig& procCfg,
                                         const SimConfig& simCfg,
                                         const std::vector<std::uint64_t>& seeds,
                                         const MineProgressFn& progress)
{
  std::vector<MineRecord> recs;
  std::string err;
  if (!MineSeedsExplicit(cfg, procCfg, simCfg, seeds, recs, &err, progress)) {
    return {};
  }
  return recs;
}

std::vector<int> SelectTopIndices(const std::vector<MineRecord>& recs,
                                 int topK,
                                 bool diverse,
                                 int candidatePool,
                                 double mmrScoreWeight,
                                 MineDiversityMode mode,
                                 double mmrLayoutWeight)
{
  std::vector<int> out;
  if (topK <= 0 || recs.empty()) return out;

  std::vector<int> idx(recs.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
    const double sa = recs[static_cast<std::size_t>(a)].score;
    const double sb = recs[static_cast<std::size_t>(b)].score;
    if (sa == sb) {
      // Make tie-breaking explicit and independent of input order.
      return recs[static_cast<std::size_t>(a)].seed < recs[static_cast<std::size_t>(b)].seed;
    }
    return sa > sb;
  });

  if (!diverse || topK == 1) {
    const int n = std::min(topK, static_cast<int>(idx.size()));
    out.assign(idx.begin(), idx.begin() + n);
    return out;
  }

  int pool = candidatePool;
  if (pool <= 0) pool = std::max(50, topK * 10);
  pool = std::min(pool, static_cast<int>(idx.size()));

  // Candidate indices (subset of best-scoring rows).
  std::vector<int> cand(idx.begin(), idx.begin() + pool);

  // Build raw feature vectors.
  std::vector<std::array<double, 7>> featsRaw;
  featsRaw.reserve(cand.size());
  for (int id : cand) featsRaw.push_back(FeatureVectorRaw(recs[static_cast<std::size_t>(id)]));

  // Min/max per dimension for normalization.
  std::array<double, 7> fmin = featsRaw.front();
  std::array<double, 7> fmax = featsRaw.front();
  for (const auto& f : featsRaw) {
    for (std::size_t d = 0; d < f.size(); ++d) {
      fmin[d] = std::min(fmin[d], f[d]);
      fmax[d] = std::max(fmax[d], f[d]);
    }
  }

  // Normalize to [0,1].
  std::vector<std::array<double, 7>> feats;
  feats.resize(featsRaw.size());
  for (std::size_t i = 0; i < featsRaw.size(); ++i) {
    for (std::size_t d = 0; d < featsRaw[i].size(); ++d) {
      const double lo = fmin[d];
      const double hi = fmax[d];
      const double denom = (hi > lo) ? (hi - lo) : 1.0;
      feats[i][d] = (featsRaw[i][d] - lo) / denom;
    }
  }

  // Layout pHash per candidate (used when mode != Scalar).
  std::vector<std::uint64_t> phash;
  phash.reserve(cand.size());
  for (int id : cand) phash.push_back(recs[static_cast<std::size_t>(id)].overlayPHash);

  // Normalize score within candidate pool.
  double smin = recs[static_cast<std::size_t>(cand.front())].score;
  double smax = smin;
  for (int id : cand) {
    const double s = recs[static_cast<std::size_t>(id)].score;
    smin = std::min(smin, s);
    smax = std::max(smax, s);
  }
  const double sden = (smax > smin) ? (smax - smin) : 1.0;

  auto scoreNormAt = [&](std::size_t candPos) -> double {
    const double s = recs[static_cast<std::size_t>(cand[candPos])].score;
    return (s - smin) / sden;
  };

  mmrScoreWeight = std::clamp(mmrScoreWeight, 0.0, 1.0);
  mmrLayoutWeight = std::clamp(mmrLayoutWeight, 0.0, 1.0);

  std::vector<bool> picked(cand.size(), false);
  out.reserve(static_cast<std::size_t>(std::min(topK, static_cast<int>(cand.size()))));

  // Always pick the best-scoring city as anchor.
  out.push_back(cand.front());
  picked[0] = true;

  while (static_cast<int>(out.size()) < topK && static_cast<int>(out.size()) < static_cast<int>(cand.size())) {
    int bestPos = -1;
    double bestMmr = -1.0;

    for (std::size_t i = 0; i < cand.size(); ++i) {
      if (picked[i]) continue;

      // Distance to nearest selected (depends on diversity mode).
      double nearestScalar = std::numeric_limits<double>::infinity();
      double nearestLayout = std::numeric_limits<double>::infinity();

      for (std::size_t j = 0; j < cand.size(); ++j) {
        if (!picked[j]) continue;

        // Scalar KPI-feature distance (normalized to [0,1] per dimension).
        nearestScalar = std::min(nearestScalar, EuclidDist(feats[i], feats[j]));

        // Layout distance via pHash (normalized Hamming distance).
        if (mode != MineDiversityMode::Scalar) {
          const int hd = HammingDistance64(phash[i], phash[j]);
          const double d01 = static_cast<double>(hd) / 64.0;
          nearestLayout = std::min(nearestLayout, d01);
        }
      }

      if (!std::isfinite(nearestScalar)) nearestScalar = 0.0;
      if (!std::isfinite(nearestLayout)) nearestLayout = 0.0;

      // Scale layout distance into the same rough range as the scalar Euclid distance
      // (features are in [0,1], so max Euclid ~ sqrt(dim)).
      const double scalarDim = static_cast<double>(feats.front().size());
      const double layoutScaled = nearestLayout * std::sqrt(std::max(1.0, scalarDim));

      double nearest = nearestScalar;
      switch (mode) {
      case MineDiversityMode::Scalar: nearest = nearestScalar; break;
      case MineDiversityMode::Layout: nearest = layoutScaled; break;
      case MineDiversityMode::Hybrid:
        nearest = (1.0 - mmrLayoutWeight) * nearestScalar + mmrLayoutWeight * layoutScaled;
        break;
      }

      const double mmr = mmrScoreWeight * scoreNormAt(i) + (1.0 - mmrScoreWeight) * nearest;
      if (mmr > bestMmr) {
        bestMmr = mmr;
        bestPos = static_cast<int>(i);
      }
    }

    if (bestPos < 0) break;
    picked[static_cast<std::size_t>(bestPos)] = true;
    out.push_back(cand[static_cast<std::size_t>(bestPos)]);
  }

  return out;
}


// -----------------------------------------------------------------------------
// Outlier / novelty analytics (LOF)
// -----------------------------------------------------------------------------

namespace {

static std::vector<MineMetric> DefaultOutlierMetrics()
{
  // A compact "behavior" vector spanning macro KPIs + physical layout fractions.
  // Users can override this via OutlierConfig::metrics.
  return {
      MineMetric::Population,
      MineMetric::Happiness,
      MineMetric::AvgLandValue,
      MineMetric::TrafficCongestion,
      MineMetric::GoodsSatisfaction,
      MineMetric::ServicesOverallSatisfaction,
      MineMetric::WaterFrac,
      MineMetric::RoadFrac,
      MineMetric::ZoneFrac,
      MineMetric::ParkFrac,
      MineMetric::FloodRisk,
  };
}

static double MedianOfSorted(const std::vector<double>& v)
{
  if (v.empty()) return 0.0;
  const std::size_t n = v.size();
  const std::size_t mid = n / 2;
  if ((n & 1u) == 1u) {
    return v[mid];
  }
  // even: average middle two
  return 0.5 * (v[mid - 1] + v[mid]);
}

static double Median(std::vector<double> v)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  return MedianOfSorted(v);
}

static void FitStandardizer(const std::vector<MineRecord>& recs,
                            const std::vector<MineMetric>& metrics,
                            bool robust,
                            std::vector<double>& center,
                            std::vector<double>& scale)
{
  const int n = static_cast<int>(recs.size());
  const int d = static_cast<int>(metrics.size());
  center.assign(static_cast<std::size_t>(d), 0.0);
  scale.assign(static_cast<std::size_t>(d), 1.0);

  if (n <= 0 || d <= 0) return;

  std::vector<double> col;
  col.reserve(static_cast<std::size_t>(n));

  for (int j = 0; j < d; ++j) {
    col.clear();
    const MineMetric m = metrics[static_cast<std::size_t>(j)];
    for (const MineRecord& r : recs) {
      const double v = MineMetricValue(r, m);
      col.push_back(std::isfinite(v) ? v : 0.0);
    }

    if (robust) {
      std::sort(col.begin(), col.end());
      const double med = MedianOfSorted(col);

      std::vector<double> dev;
      dev.reserve(col.size());
      for (double v : col) dev.push_back(std::fabs(v - med));
      std::sort(dev.begin(), dev.end());
      const double mad = MedianOfSorted(dev);

      // Consistent MAD scale factor for normal distributions.
      // 1.4826 ~= 1 / Phi^-1(3/4)
      double s = mad * 1.4826;
      if (!(s > 1.0e-12) || !std::isfinite(s)) s = 1.0;

      center[static_cast<std::size_t>(j)] = med;
      scale[static_cast<std::size_t>(j)] = s;
    } else {
      double mean = 0.0;
      for (double v : col) mean += v;
      mean /= static_cast<double>(std::max(1, n));

      double var = 0.0;
      for (double v : col) {
        const double dv = v - mean;
        var += dv * dv;
      }
      var /= static_cast<double>(std::max(1, n));
      double s = std::sqrt(var);
      if (!(s > 1.0e-12) || !std::isfinite(s)) s = 1.0;

      center[static_cast<std::size_t>(j)] = mean;
      scale[static_cast<std::size_t>(j)] = s;
    }
  }
}

static double ScalarDistanceFromFeatures(const std::vector<double>& feats,
                                        int dim,
                                        int a,
                                        int b)
{
  if (dim <= 0) return 0.0;
  const std::size_t d = static_cast<std::size_t>(dim);
  const std::size_t baseA = static_cast<std::size_t>(a) * d;
  const std::size_t baseB = static_cast<std::size_t>(b) * d;
  double sum = 0.0;
  for (std::size_t j = 0; j < d; ++j) {
    const double dv = feats[baseA + j] - feats[baseB + j];
    sum += dv * dv;
  }
  const double dist = std::sqrt(sum);
  return dist / std::sqrt(static_cast<double>(dim));
}

} // namespace

OutlierResult ComputeLocalOutlierFactor(const std::vector<MineRecord>& recs, const OutlierConfig& cfg)
{
  OutlierResult out;
  out.cfg = cfg;

  const int n = static_cast<int>(recs.size());
  out.lof.assign(static_cast<std::size_t>(std::max(0, n)), 1.0);
  out.novelty.assign(static_cast<std::size_t>(std::max(0, n)), 0.0);

  if (n <= 1) {
    // Not enough points for neighborhood statistics.
    return out;
  }

  int k = cfg.k;
  if (k <= 0) k = 1;
  k = std::min(k, n - 1);

  const MineDiversityMode space = cfg.space;
  const double lw = std::clamp(cfg.layoutWeight, 0.0, 1.0);

  // Resolve metrics for scalar/hybrid spaces.
  std::vector<MineMetric> metrics = cfg.metrics;
  if ((space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) && metrics.empty()) {
    metrics = DefaultOutlierMetrics();
  }

  // Precompute standardized feature vectors (flat) for scalar distance.
  const int dim = static_cast<int>(metrics.size());
  std::vector<double> feats;
  if (space == MineDiversityMode::Scalar || space == MineDiversityMode::Hybrid) {
    std::vector<double> center;
    std::vector<double> scale;
    FitStandardizer(recs, metrics, cfg.robustScaling, center, scale);

    feats.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(std::max(0, dim)), 0.0);
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < dim; ++j) {
        const double v = MineMetricValue(recs[static_cast<std::size_t>(i)], metrics[static_cast<std::size_t>(j)]);
        const double c = center[static_cast<std::size_t>(j)];
        const double s = scale[static_cast<std::size_t>(j)];
        feats[static_cast<std::size_t>(i) * static_cast<std::size_t>(dim) + static_cast<std::size_t>(j)] = (v - c) / s;
      }
    }
  }

  auto distFn = [&](int a, int b) -> double {
    if (space == MineDiversityMode::Layout) {
      const int hd = HammingDistance64(recs[static_cast<std::size_t>(a)].overlayPHash,
                                      recs[static_cast<std::size_t>(b)].overlayPHash);
      return static_cast<double>(hd) / 64.0;
    }

    const double ds = ScalarDistanceFromFeatures(feats, dim, a, b);

    if (space == MineDiversityMode::Scalar) {
      return ds;
    }

    // Hybrid.
    const int hd = HammingDistance64(recs[static_cast<std::size_t>(a)].overlayPHash,
                                    recs[static_cast<std::size_t>(b)].overlayPHash);
    const double dl = static_cast<double>(hd) / 64.0;
    return (1.0 - lw) * ds + lw * dl;
  };

  std::vector<int> items;
  items.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) items.push_back(i);

  // Build VP-tree once; query kNN for each point.
  VPTree<decltype(distFn)> tree(std::move(items), distFn);

  std::vector<std::vector<int>> neigh;
  std::vector<std::vector<double>> neighDist;
  neigh.resize(static_cast<std::size_t>(n));
  neighDist.resize(static_cast<std::size_t>(n));

  std::vector<double> kdist(static_cast<std::size_t>(n), 0.0);

  for (int i = 0; i < n; ++i) {
    const auto knn = tree.kNearest(i, k);

    std::vector<int>& nn = neigh[static_cast<std::size_t>(i)];
    std::vector<double>& nd = neighDist[static_cast<std::size_t>(i)];
    nn.clear();
    nd.clear();
    nn.reserve(knn.size());
    nd.reserve(knn.size());

    double sum = 0.0;
    for (const auto& pr : knn) {
      const double d = pr.first;
      const int id = pr.second;
      nn.push_back(id);
      nd.push_back(d);
      sum += d;
    }

    if (!knn.empty()) {
      kdist[static_cast<std::size_t>(i)] = knn.back().first;
      out.novelty[static_cast<std::size_t>(i)] = sum / static_cast<double>(knn.size());
    } else {
      kdist[static_cast<std::size_t>(i)] = 0.0;
      out.novelty[static_cast<std::size_t>(i)] = 0.0;
    }
  }

  // Local reachability density (LRD).
  std::vector<double> lrd(static_cast<std::size_t>(n), 0.0);
  for (int i = 0; i < n; ++i) {
    const std::vector<int>& nn = neigh[static_cast<std::size_t>(i)];
    const std::vector<double>& nd = neighDist[static_cast<std::size_t>(i)];
    if (nn.empty()) {
      lrd[static_cast<std::size_t>(i)] = 0.0;
      continue;
    }

    double sumReach = 0.0;
    for (std::size_t j = 0; j < nn.size(); ++j) {
      const int o = nn[j];
      const double reach = std::max(kdist[static_cast<std::size_t>(o)], nd[j]);
      sumReach += reach;
    }

    if (!(sumReach > 0.0) || !std::isfinite(sumReach)) {
      lrd[static_cast<std::size_t>(i)] = 0.0;
    } else {
      lrd[static_cast<std::size_t>(i)] = static_cast<double>(nn.size()) / sumReach;
    }
  }

  // LOF.
  for (int i = 0; i < n; ++i) {
    const std::vector<int>& nn = neigh[static_cast<std::size_t>(i)];
    if (nn.empty()) {
      out.lof[static_cast<std::size_t>(i)] = 1.0;
      continue;
    }

    const double lrdp = lrd[static_cast<std::size_t>(i)];
    if (!(lrdp > 0.0) || !std::isfinite(lrdp)) {
      out.lof[static_cast<std::size_t>(i)] = 1.0;
      continue;
    }

    double sumRatio = 0.0;
    for (int o : nn) {
      const double lrdo = lrd[static_cast<std::size_t>(o)];
      if (!(lrdo > 0.0) || !std::isfinite(lrdo)) {
        sumRatio += 1.0;
      } else {
        sumRatio += lrdo / lrdp;
      }
    }

    double lof = sumRatio / static_cast<double>(nn.size());
    if (!std::isfinite(lof) || lof < 0.0) lof = 1.0;
    out.lof[static_cast<std::size_t>(i)] = lof;
  }

  return out;
}

std::vector<int> SelectTopOutlierIndices(const std::vector<MineRecord>& recs, int topK)
{
  std::vector<int> out;
  if (topK <= 0 || recs.empty()) return out;

  std::vector<int> idx(recs.size());
  std::iota(idx.begin(), idx.end(), 0);

  std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
    const double la = recs[static_cast<std::size_t>(a)].outlierLof;
    const double lb = recs[static_cast<std::size_t>(b)].outlierLof;
    if (la != lb) return la > lb;

    const std::uint64_t sa = recs[static_cast<std::size_t>(a)].seed;
    const std::uint64_t sb = recs[static_cast<std::size_t>(b)].seed;
    if (sa != sb) return sa < sb;
    return a < b;
  });

  const int n = std::min(topK, static_cast<int>(idx.size()));
  out.assign(idx.begin(), idx.begin() + n);
  return out;
}


MineSession::MineSession(MineConfig cfg, ProcGenConfig procCfg, SimConfig simCfg)
    : m_cfg(std::move(cfg))
    , m_procCfg(std::move(procCfg))
    , m_simCfg(std::move(simCfg))
    , m_sim(m_simCfg)
{
  m_records.reserve(static_cast<std::size_t>(std::max(0, m_cfg.samples)));

  m_seaLevel = std::isfinite(m_cfg.seaLevelOverride) ? m_cfg.seaLevelOverride : m_procCfg.waterLevel;

  m_seaCfg.requireEdgeConnection = m_cfg.seaRequireEdgeConnection;
  m_seaCfg.eightConnected = m_cfg.seaEightConnected;

  m_depCfg.includeEdges = true;
  m_depCfg.epsilon = m_cfg.depressionEpsilon;

  // Compile optional custom score expression.
  m_scoreExprEnabled = false;
  m_scoreExprError.clear();
  if (!m_cfg.scoreExpr.empty()) {
    std::string err;
    if (CompileMineExpr(m_cfg.scoreExpr, m_scoreExpr, err)) {
      m_scoreExprEnabled = true;
    } else {
      // Degrade gracefully in UI mode; mining falls back to objectiveScore.
      m_scoreExprEnabled = false;
      m_scoreExprError = err;
    }
  }
}

int MineSession::step(int maxSteps, const MineProgressFn& progress)
{
  if (maxSteps <= 0) return 0;
  if (done()) return 0;

  const ScoreWeights weights = WeightsForObjective(m_cfg.objective);

  int produced = 0;
  while (produced < maxSteps && !done()) {
    const int i = m_index;
    const std::uint64_t seed = MineSeedForSample(m_cfg, static_cast<std::uint64_t>(i));

    MineRecord r = MineOneSeed(seed, m_cfg.w, m_cfg.h, m_cfg.days, m_procCfg, m_sim, m_cfg.hydrologyEnabled,
                              m_seaLevel, m_seaCfg, m_depCfg, weights);

    if (m_scoreExprEnabled) {
      constexpr double kScoreClamp = 1.0e30;
      constexpr double kBadScore = -1.0e30;
      double v = 0.0;
      if (!EvalMineExpr(m_scoreExpr, r, v, nullptr) || !std::isfinite(v)) {
        v = kBadScore;
      }
      if (v > kScoreClamp) v = kScoreClamp;
      if (v < -kScoreClamp) v = -kScoreClamp;
      r.score = v;
    }

    m_records.push_back(std::move(r));

    if (progress) {
      MineProgress p;
      p.index = i;
      p.total = m_cfg.samples;
      p.record = &m_records.back();
      progress(p);
    }

    ++m_index;
    ++produced;
  }

  return produced;
}

void WriteMineCsvHeader(std::ostream& os)
{
  os << "seed,seed_hex,score,objective_score,day,population,happiness,money,avg_land_value,traffic_congestion,goods_satisfaction,services_overall_satisfaction,"
        "roads,parks,road_tiles,water_tiles,res_tiles,com_tiles,ind_tiles,park_tiles,"
        "sea_flood_frac,sea_max_depth,pond_frac,pond_max_depth,pond_volume,overlay_phash,pareto_rank,pareto_crowding,outlier_lof,novelty\n";
}

void WriteMineCsvRow(std::ostream& os, const MineRecord& r)
{
  os << r.seed << ',' << HexU64(r.seed) << ',';
  os << std::fixed << std::setprecision(6);
  os << r.score << ',';
  os << r.objectiveScore << ',';
  os << r.stats.day << ',';
  os << r.stats.population << ',';
  os << r.stats.happiness << ',';
  os << r.stats.money << ',';
  os << r.stats.avgLandValue << ',';
  os << r.stats.trafficCongestion << ',';
  os << r.stats.goodsSatisfaction << ',';
  os << r.stats.servicesOverallSatisfaction << ',';

  os << r.stats.roads << ',';
  os << r.stats.parks << ',';
  os << r.roadTiles << ',';
  os << r.waterTiles << ',';
  os << r.resTiles << ',';
  os << r.comTiles << ',';
  os << r.indTiles << ',';
  os << r.parkTiles << ',';

  os << r.seaFloodFrac << ',';
  os << r.seaMaxDepth << ',';
  os << r.pondFrac << ',';
  os << r.pondMaxDepth << ',';
  os << r.pondVolume << ',';
  os << HexU64(r.overlayPHash) << ',';
  os << r.paretoRank << ',';
  os << r.paretoCrowding << ',';
  os << r.outlierLof << ',';
  os << r.novelty;
  os << "\n";
}

JsonValue MineRecordToJson(const MineRecord& r)
{
  JsonValue obj = JsonValue::MakeObject();
  auto add = [](JsonValue& o, const char* key, JsonValue v) { o.objectValue.emplace_back(key, std::move(v)); };

  add(obj, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
  add(obj, "seed_hex", JsonValue::MakeString(HexU64(r.seed)));
  add(obj, "overlay_phash", JsonValue::MakeString(HexU64(r.overlayPHash)));
  add(obj, "score", JsonValue::MakeNumber(r.score));
  add(obj, "objective_score", JsonValue::MakeNumber(r.objectiveScore));
  add(obj, "paretoRank", JsonValue::MakeNumber(static_cast<double>(r.paretoRank)));
  add(obj, "paretoCrowding", JsonValue::MakeNumber(r.paretoCrowding));
  add(obj, "outlierLof", JsonValue::MakeNumber(r.outlierLof));
  add(obj, "novelty", JsonValue::MakeNumber(r.novelty));

  JsonValue st = JsonValue::MakeObject();
  add(st, "day", JsonValue::MakeNumber(static_cast<double>(r.stats.day)));
  add(st, "population", JsonValue::MakeNumber(static_cast<double>(r.stats.population)));
  add(st, "happiness", JsonValue::MakeNumber(static_cast<double>(r.stats.happiness)));
  add(st, "money", JsonValue::MakeNumber(static_cast<double>(r.stats.money)));
  add(st, "avgLandValue", JsonValue::MakeNumber(static_cast<double>(r.stats.avgLandValue)));
  add(st, "trafficCongestion", JsonValue::MakeNumber(static_cast<double>(r.stats.trafficCongestion)));
  add(st, "goodsSatisfaction", JsonValue::MakeNumber(static_cast<double>(r.stats.goodsSatisfaction)));
  add(st, "servicesOverallSatisfaction",
      JsonValue::MakeNumber(static_cast<double>(r.stats.servicesOverallSatisfaction)));
  add(st, "roads", JsonValue::MakeNumber(static_cast<double>(r.stats.roads)));
  add(st, "parks", JsonValue::MakeNumber(static_cast<double>(r.stats.parks)));
  add(obj, "stats", std::move(st));

  JsonValue tiles = JsonValue::MakeObject();
  add(tiles, "roadTiles", JsonValue::MakeNumber(static_cast<double>(r.roadTiles)));
  add(tiles, "waterTiles", JsonValue::MakeNumber(static_cast<double>(r.waterTiles)));
  add(tiles, "resTiles", JsonValue::MakeNumber(static_cast<double>(r.resTiles)));
  add(tiles, "comTiles", JsonValue::MakeNumber(static_cast<double>(r.comTiles)));
  add(tiles, "indTiles", JsonValue::MakeNumber(static_cast<double>(r.indTiles)));
  add(tiles, "parkTiles", JsonValue::MakeNumber(static_cast<double>(r.parkTiles)));
  add(tiles, "schoolTiles", JsonValue::MakeNumber(static_cast<double>(r.schoolTiles)));
  add(tiles, "hospitalTiles", JsonValue::MakeNumber(static_cast<double>(r.hospitalTiles)));
  add(tiles, "policeTiles", JsonValue::MakeNumber(static_cast<double>(r.policeTiles)));
  add(tiles, "fireTiles", JsonValue::MakeNumber(static_cast<double>(r.fireTiles)));
  add(obj, "tiles", std::move(tiles));

  JsonValue hydro = JsonValue::MakeObject();
  add(hydro, "seaFloodFrac", JsonValue::MakeNumber(r.seaFloodFrac));
  add(hydro, "seaMaxDepth", JsonValue::MakeNumber(r.seaMaxDepth));
  add(hydro, "pondFrac", JsonValue::MakeNumber(r.pondFrac));
  add(hydro, "pondMaxDepth", JsonValue::MakeNumber(r.pondMaxDepth));
  add(hydro, "pondVolume", JsonValue::MakeNumber(r.pondVolume));
  add(obj, "hydrology", std::move(hydro));

  return obj;
}

static bool ParseHexU64(const std::string& s, std::uint64_t& out)
{
  if (s.empty()) return false;

  std::size_t i = 0;
  if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) i = 2;
  if (i >= s.size()) return false;

  std::uint64_t v = 0;
  for (; i < s.size(); ++i) {
    const char c = s[i];
    int d = -1;
    if (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'a' && c <= 'f') d = 10 + (c - 'a');
    else if (c >= 'A' && c <= 'F') d = 10 + (c - 'A');
    else return false;

    // overflow check: v*16 + d
    if (v > (std::numeric_limits<std::uint64_t>::max() >> 4)) return false;
    v = (v << 4) | static_cast<std::uint64_t>(d);
  }
  out = v;
  return true;
}

static bool ReadNumberI32(const JsonValue* v, int& out)
{
  if (!v) return false;
  if (!v->isNumber()) return false;
  const double d = v->numberValue;
  if (!std::isfinite(d)) return false;
  if (d < static_cast<double>(std::numeric_limits<int>::min()) || d > static_cast<double>(std::numeric_limits<int>::max())) return false;
  out = static_cast<int>(std::llround(d));
  return true;
}

static bool ReadNumberF64(const JsonValue* v, double& out)
{
  if (!v) return false;
  if (!v->isNumber()) return false;
  if (!std::isfinite(v->numberValue)) return false;
  out = v->numberValue;
  return true;
}

static bool ReadString(const JsonValue* v, std::string& out)
{
  if (!v) return false;
  if (!v->isString()) return false;
  out = v->stringValue;
  return true;
}

bool MineRecordFromJson(const JsonValue& obj, MineRecord& out, std::string* outError)
{
  auto fail = [&](std::string msg) {
    if (outError) *outError = std::move(msg);
    return false;
  };

  if (!obj.isObject()) return fail("expected JSON object");

  MineRecord r;

  // Prefer exact seed parsing via seed_hex.
  {
    const JsonValue* seedHex = FindJsonMember(obj, "seed_hex");
    if (seedHex && seedHex->isString()) {
      std::uint64_t sv = 0;
      if (!ParseHexU64(seedHex->stringValue, sv)) return fail("invalid seed_hex");
      r.seed = sv;
    } else {
      const JsonValue* seed = FindJsonMember(obj, "seed");
      if (!seed || !seed->isNumber() || !std::isfinite(seed->numberValue) || seed->numberValue < 0.0) {
        return fail("missing/invalid seed");
      }
      // Note: may lose precision for large u64 values.
      r.seed = static_cast<std::uint64_t>(seed->numberValue);
    }
  }

  // Optional.
  {
    const JsonValue* ph = FindJsonMember(obj, "overlay_phash");
    if (ph && ph->isString()) {
      std::uint64_t hv = 0;
      if (!ParseHexU64(ph->stringValue, hv)) return fail("invalid overlay_phash");
      r.overlayPHash = hv;
    }
  }

  bool haveScore = false;
  bool haveObjectiveScore = false;

  {
    const JsonValue* score = FindJsonMember(obj, "score");
    if (score && score->isNumber() && std::isfinite(score->numberValue)) {
      r.score = score->numberValue;
      haveScore = true;
    }
  }

  {
    const JsonValue* os = FindJsonMember(obj, "objective_score");
    if (!os) os = FindJsonMember(obj, "objectiveScore"); // legacy camelCase
    if (os && os->isNumber() && std::isfinite(os->numberValue)) {
      r.objectiveScore = os->numberValue;
      haveObjectiveScore = true;
    }
  }

  // Backward compatibility: older artifacts only had a single "score".
  if (!haveObjectiveScore) {
    r.objectiveScore = r.score;
    haveObjectiveScore = haveScore;
  }
  if (!haveScore && haveObjectiveScore) {
    r.score = r.objectiveScore;
    haveScore = true;
  }

  {
    const JsonValue* pr = FindJsonMember(obj, "paretoRank");
    int v = -1;
    if (ReadNumberI32(pr, v)) r.paretoRank = v;
    const JsonValue* pc = FindJsonMember(obj, "paretoCrowding");
    double dv = 0.0;
    if (ReadNumberF64(pc, dv)) r.paretoCrowding = dv;
  }

  // Optional outlier/novelty analytics.
  {
    double dv = 0.0;
    const JsonValue* ol = FindJsonMember(obj, "outlierLof");
    if (ReadNumberF64(ol, dv)) r.outlierLof = dv;
    const JsonValue* nv = FindJsonMember(obj, "novelty");
    if (ReadNumberF64(nv, dv)) r.novelty = dv;
  }

  // Stats.
  {
    const JsonValue* st = FindJsonMember(obj, "stats");
    if (st && st->isObject()) {
      int iv = 0;
      double dv = 0.0;
      if (ReadNumberI32(FindJsonMember(*st, "day"), iv)) r.stats.day = iv;
      if (ReadNumberI32(FindJsonMember(*st, "population"), iv)) r.stats.population = iv;
      if (ReadNumberF64(FindJsonMember(*st, "happiness"), dv)) r.stats.happiness = dv;
      if (ReadNumberF64(FindJsonMember(*st, "money"), dv)) r.stats.money = dv;
      if (ReadNumberF64(FindJsonMember(*st, "avgLandValue"), dv)) r.stats.avgLandValue = dv;
      if (ReadNumberF64(FindJsonMember(*st, "trafficCongestion"), dv)) r.stats.trafficCongestion = dv;
      if (ReadNumberF64(FindJsonMember(*st, "goodsSatisfaction"), dv)) r.stats.goodsSatisfaction = dv;
      if (ReadNumberF64(FindJsonMember(*st, "servicesOverallSatisfaction"), dv)) r.stats.servicesOverallSatisfaction = dv;
      if (ReadNumberI32(FindJsonMember(*st, "roads"), iv)) r.stats.roads = iv;
      if (ReadNumberI32(FindJsonMember(*st, "parks"), iv)) r.stats.parks = iv;
    }
  }

  // Tiles.
  {
    const JsonValue* tiles = FindJsonMember(obj, "tiles");
    if (tiles && tiles->isObject()) {
      int iv = 0;
      if (ReadNumberI32(FindJsonMember(*tiles, "roadTiles"), iv)) r.roadTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "waterTiles"), iv)) r.waterTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "resTiles"), iv)) r.resTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "comTiles"), iv)) r.comTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "indTiles"), iv)) r.indTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "parkTiles"), iv)) r.parkTiles = iv;

      if (ReadNumberI32(FindJsonMember(*tiles, "schoolTiles"), iv)) r.schoolTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "hospitalTiles"), iv)) r.hospitalTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "policeTiles"), iv)) r.policeTiles = iv;
      if (ReadNumberI32(FindJsonMember(*tiles, "fireTiles"), iv)) r.fireTiles = iv;
    }
  }

  // Hydrology.
  {
    const JsonValue* hydro = FindJsonMember(obj, "hydrology");
    if (hydro && hydro->isObject()) {
      double dv = 0.0;
      if (ReadNumberF64(FindJsonMember(*hydro, "seaFloodFrac"), dv)) r.seaFloodFrac = dv;
      if (ReadNumberF64(FindJsonMember(*hydro, "seaMaxDepth"), dv)) r.seaMaxDepth = dv;
      if (ReadNumberF64(FindJsonMember(*hydro, "pondFrac"), dv)) r.pondFrac = dv;
      if (ReadNumberF64(FindJsonMember(*hydro, "pondMaxDepth"), dv)) r.pondMaxDepth = dv;
      if (ReadNumberF64(FindJsonMember(*hydro, "pondVolume"), dv)) r.pondVolume = dv;
    }
  }

  out = std::move(r);
  if (outError) outError->clear();
  return true;
}

bool MineRecordFromJsonText(const std::string& text, MineRecord& out, std::string* outError)
{
  JsonValue v;
  std::string err;
  if (!ParseJson(text, v, err)) {
    if (outError) *outError = err;
    return false;
  }
  return MineRecordFromJson(v, out, outError);
}

} // namespace isocity
