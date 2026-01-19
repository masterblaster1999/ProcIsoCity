#include "isocity/SeedMiner.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <ostream>
#include <sstream>

namespace isocity {

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

  r.score = ComputeScore(r, weights);
  return r;
}

} // namespace

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

std::vector<MineRecord> MineSeeds(const MineConfig& cfg,
                                 const ProcGenConfig& procCfg,
                                 const SimConfig& simCfg,
                                 const MineProgressFn& progress)
{
  std::vector<MineRecord> recs;
  if (cfg.samples <= 0 || cfg.w <= 0 || cfg.h <= 0) return recs;

  recs.reserve(static_cast<std::size_t>(cfg.samples));

  Simulator sim(simCfg);

  SeaFloodConfig seaCfg;
  seaCfg.requireEdgeConnection = cfg.seaRequireEdgeConnection;
  seaCfg.eightConnected = cfg.seaEightConnected;

  DepressionFillConfig depCfg;
  depCfg.includeEdges = true;
  depCfg.epsilon = cfg.depressionEpsilon;

  const float seaLevel = std::isfinite(cfg.seaLevelOverride) ? cfg.seaLevelOverride : procCfg.waterLevel;
  const ScoreWeights weights = WeightsForObjective(cfg.objective);

  for (int i = 0; i < cfg.samples; ++i) {
    const std::uint64_t seed = cfg.seedStart + static_cast<std::uint64_t>(i) * cfg.seedStep;

    MineRecord r = MineOneSeed(seed, cfg.w, cfg.h, cfg.days, procCfg, sim, cfg.hydrologyEnabled,
                              seaLevel, seaCfg, depCfg, weights);

    recs.push_back(std::move(r));

    if (progress) {
      MineProgress p;
      p.index = i;
      p.total = cfg.samples;
      p.record = &recs.back();
      progress(p);
    }
  }

  return recs;
}

std::vector<int> SelectTopIndices(const std::vector<MineRecord>& recs,
                                 int topK,
                                 bool diverse,
                                 int candidatePool,
                                 double mmrScoreWeight)
{
  std::vector<int> out;
  if (topK <= 0 || recs.empty()) return out;

  std::vector<int> idx(recs.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
    return recs[static_cast<std::size_t>(a)].score > recs[static_cast<std::size_t>(b)].score;
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

      // Distance to nearest selected.
      double nearest = std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < cand.size(); ++j) {
        if (!picked[j]) continue;
        nearest = std::min(nearest, EuclidDist(feats[i], feats[j]));
      }
      if (!std::isfinite(nearest)) nearest = 0.0;

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
}

int MineSession::step(int maxSteps, const MineProgressFn& progress)
{
  if (maxSteps <= 0) return 0;
  if (done()) return 0;

  const ScoreWeights weights = WeightsForObjective(m_cfg.objective);

  int produced = 0;
  while (produced < maxSteps && !done()) {
    const int i = m_index;
    const std::uint64_t seed = m_cfg.seedStart + static_cast<std::uint64_t>(i) * m_cfg.seedStep;

    MineRecord r = MineOneSeed(seed, m_cfg.w, m_cfg.h, m_cfg.days, m_procCfg, m_sim, m_cfg.hydrologyEnabled,
                              m_seaLevel, m_seaCfg, m_depCfg, weights);

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
  os << "seed,seed_hex,score,day,population,happiness,money,avg_land_value,traffic_congestion,goods_satisfaction,services_overall_satisfaction,"
        "roads,parks,road_tiles,water_tiles,res_tiles,com_tiles,ind_tiles,park_tiles,"
        "sea_flood_frac,sea_max_depth,pond_frac,pond_max_depth,pond_volume\n";
}

void WriteMineCsvRow(std::ostream& os, const MineRecord& r)
{
  os << r.seed << ',' << HexU64(r.seed) << ',';
  os << std::fixed << std::setprecision(6);
  os << r.score << ',';
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
  os << r.pondVolume;
  os << "\n";
}

JsonValue MineRecordToJson(const MineRecord& r)
{
  JsonValue obj = JsonValue::MakeObject();
  auto add = [](JsonValue& o, const char* key, JsonValue v) { o.objectValue.emplace_back(key, std::move(v)); };

  add(obj, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
  add(obj, "seed_hex", JsonValue::MakeString(HexU64(r.seed)));
  add(obj, "score", JsonValue::MakeNumber(r.score));

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

} // namespace isocity
