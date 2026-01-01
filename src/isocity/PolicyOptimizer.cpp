#include "isocity/PolicyOptimizer.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace isocity {

namespace {

constexpr double kTieEps = 1e-9;

struct PackedPolicyKey {
  // Pack 5 signed-ish ints into 64 bits (16 bits each).
  // Values are assumed to be small non-negative integers, which is true for our policy ranges.
  static std::uint64_t pack(const PolicyCandidate& p)
  {
    // We store 5 small non-negative integers.
    // Pack into 60 bits using 12 bits per field (0..4095).
    // If inputs exceed the mask, they will alias; callers should keep ranges sane.
    const std::uint64_t a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.taxResidential) & 0xFFFu);
    const std::uint64_t b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.taxCommercial) & 0xFFFu);
    const std::uint64_t c = static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.taxIndustrial) & 0xFFFu);
    const std::uint64_t d = static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.maintenanceRoad) & 0xFFFu);
    const std::uint64_t e = static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.maintenancePark) & 0xFFFu);

    return a | (b << 12u) | (c << 24u) | (d << 36u) | (e << 48u);
  }
};

inline bool LexLess(const PolicyCandidate& a, const PolicyCandidate& b)
{
  if (a.taxResidential != b.taxResidential) return a.taxResidential < b.taxResidential;
  if (a.taxCommercial != b.taxCommercial) return a.taxCommercial < b.taxCommercial;
  if (a.taxIndustrial != b.taxIndustrial) return a.taxIndustrial < b.taxIndustrial;
  if (a.maintenanceRoad != b.maintenanceRoad) return a.maintenanceRoad < b.maintenanceRoad;
  return a.maintenancePark < b.maintenancePark;
}

inline bool Better(const PolicyEvalResult& a, const PolicyEvalResult& b)
{
  if (a.score > b.score + kTieEps) return true;
  if (b.score > a.score + kTieEps) return false;
  return LexLess(a.policy, b.policy);
}

inline std::uint64_t CountCandidates(const PolicySearchSpace& s)
{
  auto span = [](int mn, int mx) -> std::uint64_t {
    if (mx < mn) return 0;
    return static_cast<std::uint64_t>(static_cast<std::uint32_t>(mx - mn + 1));
  };

  const std::uint64_t a = span(s.taxResMin, s.taxResMax);
  const std::uint64_t b = span(s.taxComMin, s.taxComMax);
  const std::uint64_t c = span(s.taxIndMin, s.taxIndMax);
  const std::uint64_t d = span(s.maintRoadMin, s.maintRoadMax);
  const std::uint64_t e = span(s.maintParkMin, s.maintParkMax);
  if (a == 0 || b == 0 || c == 0 || d == 0 || e == 0) return 0;
  // Guard overflow; we only use this to decide between exhaustive vs. sampling.
  const long double prod = static_cast<long double>(a) * static_cast<long double>(b) * static_cast<long double>(c) *
                           static_cast<long double>(d) * static_cast<long double>(e);
  if (prod > static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) return std::numeric_limits<std::uint64_t>::max();
  return a * b * c * d * e;
}

inline int ClampI(int v, int mn, int mx)
{
  if (v < mn) return mn;
  if (v > mx) return mx;
  return v;
}

// Deterministic normal sampling using Box-Muller.
inline double SampleStandardNormal(RNG& rng)
{
  // Guard against log(0).
  const double u1 = std::max(1e-12, static_cast<double>(rng.nextF01()));
  const double u2 = static_cast<double>(rng.nextF01());
  const double r = std::sqrt(-2.0 * std::log(u1));
  const double theta = 6.28318530717958647692 * u2; // 2*pi
  return r * std::cos(theta);
}

inline int SampleClampedNormalI(RNG& rng, double mean, double stdDev, int mn, int mx)
{
  if (stdDev < 1e-6) stdDev = 1e-6;
  const double z = SampleStandardNormal(rng);
  const double x = mean + stdDev * z;
  const int xi = static_cast<int>(std::lround(x));
  return ClampI(xi, mn, mx);
}

inline int SampleUniformI(RNG& rng, int mn, int mx)
{
  if (mx < mn) return mn;
  return rng.rangeInt(mn, mx);
}

inline PolicyCandidate SampleCandidate(RNG& rng, const PolicySearchSpace& s, const PolicyDistribution& dist, float exploreProb)
{
  PolicyCandidate p;

  const bool explore = rng.chance(exploreProb);
  if (explore) {
    p.taxResidential = SampleUniformI(rng, s.taxResMin, s.taxResMax);
    p.taxCommercial = SampleUniformI(rng, s.taxComMin, s.taxComMax);
    p.taxIndustrial = SampleUniformI(rng, s.taxIndMin, s.taxIndMax);
    p.maintenanceRoad = SampleUniformI(rng, s.maintRoadMin, s.maintRoadMax);
    p.maintenancePark = SampleUniformI(rng, s.maintParkMin, s.maintParkMax);
    return p;
  }

  p.taxResidential = SampleClampedNormalI(rng, dist.meanTaxResidential, dist.stdTaxResidential, s.taxResMin, s.taxResMax);
  p.taxCommercial = SampleClampedNormalI(rng, dist.meanTaxCommercial, dist.stdTaxCommercial, s.taxComMin, s.taxComMax);
  p.taxIndustrial = SampleClampedNormalI(rng, dist.meanTaxIndustrial, dist.stdTaxIndustrial, s.taxIndMin, s.taxIndMax);
  p.maintenanceRoad = SampleClampedNormalI(rng, dist.meanMaintRoad, dist.stdMaintRoad, s.maintRoadMin, s.maintRoadMax);
  p.maintenancePark = SampleClampedNormalI(rng, dist.meanMaintPark, dist.stdMaintPark, s.maintParkMin, s.maintParkMax);
  return p;
}

inline PolicyDistribution InitialDist(const PolicySearchSpace& s)
{
  auto mid = [](int mn, int mx) -> double { return 0.5 * (static_cast<double>(mn) + static_cast<double>(mx)); };
  auto span = [](int mn, int mx) -> double { return static_cast<double>(std::max(1, mx - mn + 1)); };

  PolicyDistribution d{};
  d.meanTaxResidential = mid(s.taxResMin, s.taxResMax);
  d.meanTaxCommercial = mid(s.taxComMin, s.taxComMax);
  d.meanTaxIndustrial = mid(s.taxIndMin, s.taxIndMax);
  d.meanMaintRoad = mid(s.maintRoadMin, s.maintRoadMax);
  d.meanMaintPark = mid(s.maintParkMin, s.maintParkMax);

  // Start wide: std ~ range/2.
  d.stdTaxResidential = 0.5 * span(s.taxResMin, s.taxResMax);
  d.stdTaxCommercial = 0.5 * span(s.taxComMin, s.taxComMax);
  d.stdTaxIndustrial = 0.5 * span(s.taxIndMin, s.taxIndMax);
  d.stdMaintRoad = 0.5 * span(s.maintRoadMin, s.maintRoadMax);
  d.stdMaintPark = 0.5 * span(s.maintParkMin, s.maintParkMax);
  return d;
}

inline PolicyDistribution FitDist(const std::vector<PolicyEvalResult>& elites)
{
  PolicyDistribution d{};
  if (elites.empty()) return d;

  auto meanStd = [&](auto getVal) -> std::pair<double, double> {
    double mean = 0.0;
    for (const auto& e : elites) mean += static_cast<double>(getVal(e.policy));
    mean /= static_cast<double>(elites.size());
    double var = 0.0;
    for (const auto& e : elites) {
      const double dv = static_cast<double>(getVal(e.policy)) - mean;
      var += dv * dv;
    }
    var /= static_cast<double>(elites.size());
    const double stdDev = std::sqrt(std::max(1e-12, var));
    return {mean, stdDev};
  };

  const auto [m0, s0] = meanStd([](const PolicyCandidate& p) { return p.taxResidential; });
  const auto [m1, s1] = meanStd([](const PolicyCandidate& p) { return p.taxCommercial; });
  const auto [m2, s2] = meanStd([](const PolicyCandidate& p) { return p.taxIndustrial; });
  const auto [m3, s3] = meanStd([](const PolicyCandidate& p) { return p.maintenanceRoad; });
  const auto [m4, s4] = meanStd([](const PolicyCandidate& p) { return p.maintenancePark; });

  d.meanTaxResidential = m0;
  d.stdTaxResidential = std::max(0.5, s0);

  d.meanTaxCommercial = m1;
  d.stdTaxCommercial = std::max(0.5, s1);

  d.meanTaxIndustrial = m2;
  d.stdTaxIndustrial = std::max(0.5, s2);

  d.meanMaintRoad = m3;
  d.stdMaintRoad = std::max(0.5, s3);

  d.meanMaintPark = m4;
  d.stdMaintPark = std::max(0.5, s4);

  return d;
}

inline double ScoreFromMetrics(const PolicyEvalMetrics& m, const PolicyObjective& o)
{
  if (static_cast<double>(m.happinessEnd) < o.minHappiness) return -std::numeric_limits<double>::infinity();
  if (m.moneyEnd < o.minMoneyEnd) return -std::numeric_limits<double>::infinity();

  const double moneyDelta = static_cast<double>(m.moneyDelta);
  const double pop = static_cast<double>(m.populationEnd);
  const double happyPop = static_cast<double>(m.avgHappiness) * pop;

  const double unemployed = static_cast<double>(std::max(0, m.populationEnd - m.employedEnd));
  const double congestionPop = static_cast<double>(m.trafficCongestionEnd) * pop;

  return o.wMoneyDelta * moneyDelta + o.wPopulation * pop + o.wHappyPop * happyPop - o.wUnemployed * unemployed -
         o.wCongestionPop * congestionPop;
}

PolicyEvalResult EvaluateWithCache(const World& baseWorld, const SimConfig& baseSimCfg, const PolicyCandidate& cand,
                                  const PolicyOptimizerConfig& cfg, std::unordered_map<std::uint64_t, PolicyEvalResult>& cache,
                                  std::mutex& cacheMutex)
{
  const std::uint64_t key = PackedPolicyKey::pack(cand);
  {
    std::lock_guard<std::mutex> lk(cacheMutex);
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;
  }

  PolicyEvalResult r = EvaluatePolicyCandidate(baseWorld, baseSimCfg, cand, cfg);

  {
    std::lock_guard<std::mutex> lk(cacheMutex);
    // Keep the "better" result if an identical policy raced.
    auto it = cache.find(key);
    if (it == cache.end() || Better(r, it->second)) cache[key] = r;
    else r = it->second;
  }
  return r;
}

std::vector<PolicyEvalResult> EvaluateBatch(const World& baseWorld, const SimConfig& baseSimCfg, const std::vector<PolicyCandidate>& cands,
                                            const PolicyOptimizerConfig& cfg, std::unordered_map<std::uint64_t, PolicyEvalResult>& cache,
                                            std::mutex& cacheMutex)
{
  std::vector<PolicyEvalResult> out;
  out.resize(cands.size());

  int threads = cfg.threads;
  if (threads <= 0) {
    threads = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = 1;
  }

  // Avoid launching lots of threads for tiny batches.
  threads = std::min<int>(threads, static_cast<int>(cands.size()));
  if (threads <= 1) {
    for (std::size_t i = 0; i < cands.size(); ++i) {
      out[i] = EvaluateWithCache(baseWorld, baseSimCfg, cands[i], cfg, cache, cacheMutex);
    }
    return out;
  }

  std::atomic<std::size_t> next{0};
  std::vector<std::thread> pool;
  pool.reserve(static_cast<std::size_t>(threads));

  for (int t = 0; t < threads; ++t) {
    pool.emplace_back([&]() {
      for (;;) {
        const std::size_t i = next.fetch_add(1);
        if (i >= cands.size()) break;
        out[i] = EvaluateWithCache(baseWorld, baseSimCfg, cands[i], cfg, cache, cacheMutex);
      }
    });
  }
  for (auto& th : pool) th.join();
  return out;
}

void InsertTopK(std::vector<PolicyEvalResult>& top, const PolicyEvalResult& r, int k)
{
  if (k <= 0) return;
  if (static_cast<int>(top.size()) < k) {
    top.push_back(r);
    std::sort(top.begin(), top.end(), [&](const auto& a, const auto& b) { return Better(a, b); });
    return;
  }

  // top is kept sorted descending. If r isn't better than the last element, ignore.
  if (!Better(r, top.back())) return;

  top.push_back(r);
  std::sort(top.begin(), top.end(), [&](const auto& a, const auto& b) { return Better(a, b); });
  if (static_cast<int>(top.size()) > k) top.resize(static_cast<std::size_t>(k));
}

} // namespace

PolicyCandidate ExtractPolicyFromSimConfig(const SimConfig& cfg)
{
  PolicyCandidate p;
  p.taxResidential = cfg.taxResidential;
  p.taxCommercial = cfg.taxCommercial;
  p.taxIndustrial = cfg.taxIndustrial;
  p.maintenanceRoad = cfg.maintenanceRoad;
  p.maintenancePark = cfg.maintenancePark;
  return p;
}

void ApplyPolicyToSimConfig(const PolicyCandidate& p, SimConfig& cfg)
{
  cfg.taxResidential = p.taxResidential;
  cfg.taxCommercial = p.taxCommercial;
  cfg.taxIndustrial = p.taxIndustrial;
  cfg.maintenanceRoad = p.maintenanceRoad;
  cfg.maintenancePark = p.maintenancePark;
}

PolicyEvalResult EvaluatePolicyCandidate(const World& baseWorld, const SimConfig& baseSimCfg, const PolicyCandidate& cand,
                                        const PolicyOptimizerConfig& cfg)
{
  PolicyEvalResult r;
  r.policy = cand;

  World w = baseWorld;
  SimConfig simCfg = baseSimCfg;
  ApplyPolicyToSimConfig(cand, simCfg);

  Simulator sim(simCfg);

  const int days = std::max(0, cfg.evalDays);
  r.metrics.daysSimulated = days;

  r.metrics.moneyStart = w.stats().money;

  double sumHappy = 0.0;
  double sumNet = 0.0;

  for (int d = 0; d < days; ++d) {
    sim.stepOnce(w);
    sumHappy += static_cast<double>(w.stats().happiness);
    sumNet += static_cast<double>(w.stats().income - w.stats().expenses);
  }

  if (days == 0) {
    // Ensure derived fields are up to date even if no ticks were simulated.
    sim.refreshDerivedStats(w);
    sumHappy = static_cast<double>(w.stats().happiness);
  }

  r.metrics.moneyEnd = w.stats().money;
  r.metrics.moneyDelta = r.metrics.moneyEnd - r.metrics.moneyStart;

  r.metrics.populationEnd = w.stats().population;
  r.metrics.employedEnd = w.stats().employed;
  r.metrics.jobsCapacityAccessibleEnd = w.stats().jobsCapacityAccessible;

  r.metrics.happinessEnd = w.stats().happiness;
  r.metrics.avgHappiness = static_cast<float>(sumHappy / static_cast<double>(std::max(1, days)));

  r.metrics.demandResidentialEnd = w.stats().demandResidential;
  r.metrics.avgLandValueEnd = w.stats().avgLandValue;

  r.metrics.avgCommuteTimeEnd = w.stats().avgCommuteTime;
  r.metrics.trafficCongestionEnd = w.stats().trafficCongestion;

  r.metrics.avgNetPerDay = (days > 0) ? (sumNet / static_cast<double>(days)) : 0.0;

  r.score = ScoreFromMetrics(r.metrics, cfg.objective);
  return r;
}

PolicyOptimizationResult OptimizePolicies(const World& baseWorld, const SimConfig& baseSimCfg, const PolicySearchSpace& space,
                                         const PolicyOptimizerConfig& cfg)
{
  PolicyOptimizationResult out;

  const std::uint64_t total = CountCandidates(space);

  const bool canExhaustive = (total > 0 && total <= cfg.maxExhaustiveCandidates);
  const PolicyOptMethod method = (cfg.method == PolicyOptMethod::Exhaustive || canExhaustive) ? PolicyOptMethod::Exhaustive : cfg.method;
  out.methodUsed = method;

  std::unordered_map<std::uint64_t, PolicyEvalResult> cache;
  cache.reserve(2048);
  std::mutex cacheMutex;

  PolicyEvalResult best;
  best.score = -std::numeric_limits<double>::infinity();

  std::vector<PolicyEvalResult> top;
  top.reserve(static_cast<std::size_t>(std::max(0, cfg.topK)));

  if (method == PolicyOptMethod::Exhaustive) {
    // Exhaustive enumeration is deterministic and easy to reason about.
    for (int tr = space.taxResMin; tr <= space.taxResMax; ++tr) {
      for (int tc = space.taxComMin; tc <= space.taxComMax; ++tc) {
        for (int ti = space.taxIndMin; ti <= space.taxIndMax; ++ti) {
          for (int mr = space.maintRoadMin; mr <= space.maintRoadMax; ++mr) {
            for (int mp = space.maintParkMin; mp <= space.maintParkMax; ++mp) {
              PolicyCandidate cand;
              cand.taxResidential = tr;
              cand.taxCommercial = tc;
              cand.taxIndustrial = ti;
              cand.maintenanceRoad = mr;
              cand.maintenancePark = mp;

              PolicyEvalResult r = EvaluateWithCache(baseWorld, baseSimCfg, cand, cfg, cache, cacheMutex);
              out.candidatesEvaluated++;

              if (Better(r, best)) best = r;
              InsertTopK(top, r, cfg.topK);
            }
          }
        }
      }
    }

    out.best = best;
    out.top = std::move(top);
    out.iterationsCompleted = 1;
    out.bestByIteration.push_back(best);
    return out;
  }

  // CEM sampling (int parameters via rounded clamped normals).
  RNG rng(cfg.rngSeed);

  PolicyDistribution dist = InitialDist(space);

  const int iterations = std::max(1, cfg.iterations);
  const int popN = std::max(1, cfg.population);
  const int eliteN = std::clamp(cfg.elites, 1, popN);

  out.distByIteration.reserve(static_cast<std::size_t>(iterations));
  out.bestByIteration.reserve(static_cast<std::size_t>(iterations));

  for (int it = 0; it < iterations; ++it) {
    std::vector<PolicyCandidate> cands;
    cands.reserve(static_cast<std::size_t>(popN));

    // Elitism: include current best policy first once we have one.
    if (it > 0 && best.score > -std::numeric_limits<double>::infinity()) {
      cands.push_back(best.policy);
    }

    while (static_cast<int>(cands.size()) < popN) {
      cands.push_back(SampleCandidate(rng, space, dist, cfg.exploreProb));
    }

    std::vector<PolicyEvalResult> eval = EvaluateBatch(baseWorld, baseSimCfg, cands, cfg, cache, cacheMutex);
    out.candidatesEvaluated += static_cast<int>(eval.size());

    // Sort descending.
    std::sort(eval.begin(), eval.end(), [&](const auto& a, const auto& b) { return Better(a, b); });

    if (!eval.empty() && Better(eval.front(), best)) best = eval.front();

    for (const auto& r : eval) InsertTopK(top, r, cfg.topK);

    // Refit distribution from elites.
    std::vector<PolicyEvalResult> elites;
    elites.reserve(static_cast<std::size_t>(eliteN));
    for (int i = 0; i < eliteN && i < static_cast<int>(eval.size()); ++i) elites.push_back(eval[static_cast<std::size_t>(i)]);
    dist = FitDist(elites);

    out.bestByIteration.push_back(best);
    out.distByIteration.push_back(dist);
    out.iterationsCompleted = it + 1;
  }

  out.best = best;
  out.top = std::move(top);
  return out;
}

} // namespace isocity