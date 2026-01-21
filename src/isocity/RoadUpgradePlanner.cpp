#include "isocity/RoadUpgradePlanner.hpp"

#include "isocity/Road.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline int ClampRoadLevelLocal(int level)
{
  if (level < 1) return 1;
  if (level > 3) return 3;
  return level;
}

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline int CapacityForLevel(int baseCap, int level, bool useRoadLevels)
{
  const int base = std::max(1, baseCap);
  if (!useRoadLevels) return base;
  return std::max(1, RoadCapacityForLevel(base, ClampRoadLevelLocal(level)));
}

inline int TravelTimeForTile(const Tile& t, int level)
{
  const int lvl = ClampRoadLevelLocal(level);
  return (t.terrain == Terrain::Water)
    ? RoadBridgeTravelTimeMilliForLevel(lvl)
    : RoadTravelTimeMilliForLevel(lvl);
}

struct EvalResult {
  int cost = 0;
  std::uint64_t timeSaved = 0;
  std::uint64_t excessReduced = 0;
};

// Evaluate the incremental cost/benefit of upgrading a set of tiles to targetLevel.
//
// - plannedLevels is an optional per-tile level override (0 means "no plan yet").
//   If provided, the upgrade is evaluated relative to max(currentLevel, plannedLevel).
static EvalResult EvaluateUpgrade(const World& world,
                                 const std::vector<Point>& tiles,
                                 int targetLevel,
                                 const std::vector<std::uint32_t>& flow,
                                 int baseCap,
                                 bool useRoadLevels,
                                 const std::vector<std::uint8_t>* plannedLevels)
{
  EvalResult r;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return r;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (flow.size() != n) return r;

  const int tgt = ClampRoadLevelLocal(targetLevel);

  for (const Point& p : tiles) {
    if (!world.inBounds(p.x, p.y)) continue;
    const Tile& t = world.at(p.x, p.y);
    if (t.overlay != Overlay::Road) continue;

    const std::size_t idx = FlatIdx(p.x, p.y, w);
    if (idx >= n) continue;

    const int curLvl = ClampRoadLevelLocal(static_cast<int>(t.level));
    int baseLvl = curLvl;
    if (plannedLevels && idx < plannedLevels->size()) {
      const int planned = ClampRoadLevelLocal(static_cast<int>((*plannedLevels)[idx]));
      if ((*plannedLevels)[idx] != 0) baseLvl = std::max(baseLvl, planned);
    }

    if (baseLvl >= tgt) continue;

    const bool isBridge = (t.terrain == Terrain::Water);
    r.cost += RoadPlacementCost(baseLvl, tgt, /*alreadyRoad=*/true, isBridge);

    const std::uint32_t vU = flow[idx];
    const int v = (vU > static_cast<std::uint32_t>(std::numeric_limits<int>::max()))
      ? std::numeric_limits<int>::max()
      : static_cast<int>(vU);

    const int oldCap = CapacityForLevel(baseCap, baseLvl, useRoadLevels);
    const int newCap = CapacityForLevel(baseCap, tgt, useRoadLevels);

    const int oldExcess = std::max(0, v - oldCap);
    const int newExcess = std::max(0, v - newCap);
    r.excessReduced += static_cast<std::uint64_t>(std::max(0, oldExcess - newExcess));

    const int oldTime = TravelTimeForTile(t, baseLvl);
    const int newTime = TravelTimeForTile(t, tgt);
    if (oldTime > newTime && v > 0) {
      r.timeSaved += static_cast<std::uint64_t>(v) * static_cast<std::uint64_t>(oldTime - newTime);
    }
  }

  return r;
}

static double BenefitScore(const RoadUpgradePlannerConfig& cfg, const EvalResult& r)
{
  switch (cfg.objective) {
    case RoadUpgradeObjective::Congestion:
      return static_cast<double>(r.excessReduced);
    case RoadUpgradeObjective::Time:
      return static_cast<double>(r.timeSaved);
    case RoadUpgradeObjective::Hybrid:
      return cfg.hybridExcessWeight * static_cast<double>(r.excessReduced)
           + cfg.hybridTimeWeight * static_cast<double>(r.timeSaved);
    default:
      return static_cast<double>(r.excessReduced);
  }
}

} // namespace

RoadUpgradePlan PlanRoadUpgrades(const World& world, const RoadGraph& g,
                                 const std::vector<std::uint32_t>& roadFlow,
                                 const RoadUpgradePlannerConfig& cfg)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto finalize = [&](RoadUpgradePlan& plan) {
    plan.runtimeSec = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0)
                         .count();
  };

  RoadUpgradePlan plan;
  plan.w = world.width();
  plan.h = world.height();
  plan.cfg = cfg;

  const int w = plan.w;
  const int h = plan.h;
  if (w <= 0 || h <= 0) {
    finalize(plan);
    return plan;
  }

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  plan.tileTargetLevel.assign(n, 0);
  if (roadFlow.size() != n) {
    finalize(plan);
    return plan;
  }

  const int baseCap = std::max(1, cfg.baseTileCapacity);
  const bool useRoadLevels = cfg.useRoadLevelCapacity;
  const int maxLevel = ClampRoadLevelLocal(cfg.maxTargetLevel);

  struct Candidate {
    int edgeIndex = -1;
    int a = -1;
    int b = -1;
    int targetLevel = 1;
    double ratio = 0.0;
    double benefit = 0.0;
    int baseCost = 0;
    std::uint64_t baseTimeSaved = 0;
    std::uint64_t baseExcessReduced = 0;
    std::vector<Point> tiles;
  };

  std::vector<Candidate> candidates;
  candidates.reserve(g.edges.size() * 2u);

  auto tileUtil = [&](int x, int y) -> double {
    if (!world.inBounds(x, y)) return 0.0;
    const Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) return 0.0;
    const std::size_t idx = FlatIdx(x, y, w);
    if (idx >= roadFlow.size()) return 0.0;
    const int v = static_cast<int>(std::min<std::uint32_t>(roadFlow[idx], 1000000u));
    const int cap = CapacityForLevel(baseCap, static_cast<int>(t.level), useRoadLevels);
    if (cap <= 0) return 0.0;
    return static_cast<double>(v) / static_cast<double>(cap);
  };

  // --- Candidate generation ---
  for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
    const RoadGraphEdge& e = g.edges[static_cast<std::size_t>(ei)];

    // Choose the tile set this edge candidate represents.
    std::vector<Point> tiles;
    tiles.reserve(e.tiles.size());

    if (cfg.upgradeEndpoints || e.tiles.size() <= 2) {
      tiles = e.tiles;
    } else {
      for (std::size_t i = 1; i + 1 < e.tiles.size(); ++i) tiles.push_back(e.tiles[i]);
    }

    if (tiles.empty()) continue;

    // Filter by current utilization.
    double maxUtil = 0.0;
    for (const Point& p : tiles) {
      maxUtil = std::max(maxUtil, tileUtil(p.x, p.y));
    }
    if (cfg.minUtilConsider > 0.0 && maxUtil < cfg.minUtilConsider) continue;

    // Consider upgrades to each level (2..maxLevel).
    for (int tgt = 2; tgt <= maxLevel; ++tgt) {
      const EvalResult baseEval = EvaluateUpgrade(world, tiles, tgt, roadFlow, baseCap, useRoadLevels, nullptr);
      if (baseEval.cost <= 0) continue;

      const double benefit = BenefitScore(cfg, baseEval);
      if (benefit <= 0.0) continue;

      Candidate c;
      c.edgeIndex = ei;
      c.a = e.a;
      c.b = e.b;
      c.targetLevel = tgt;
      c.baseCost = baseEval.cost;
      c.baseTimeSaved = baseEval.timeSaved;
      c.baseExcessReduced = baseEval.excessReduced;
      c.benefit = benefit;
      c.ratio = benefit / static_cast<double>(std::max(1, baseEval.cost));
      c.tiles = std::move(tiles);
      candidates.push_back(std::move(c));

      // Rebuild tiles for next iteration if we moved them.
      tiles = (cfg.upgradeEndpoints || e.tiles.size() <= 2)
        ? e.tiles
        : std::vector<Point>(e.tiles.begin() + 1, e.tiles.end() - 1);
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    if (a.ratio != b.ratio) return a.ratio > b.ratio;
    if (a.benefit != b.benefit) return a.benefit > b.benefit;
    if (a.baseCost != b.baseCost) return a.baseCost < b.baseCost;
    if (a.edgeIndex != b.edgeIndex) return a.edgeIndex < b.edgeIndex;
    return a.targetLevel < b.targetLevel;
  });

  const int budget = cfg.budget;
  std::vector<std::uint8_t> edgeChosen(g.edges.size(), std::uint8_t{0});

  // --- Greedy selection ---
  for (const Candidate& c : candidates) {
    if (c.edgeIndex < 0 || c.edgeIndex >= static_cast<int>(edgeChosen.size())) continue;
    if (edgeChosen[static_cast<std::size_t>(c.edgeIndex)]) continue;

    // Evaluate incrementally relative to any already-selected upgrades.
    const EvalResult inc = EvaluateUpgrade(world, c.tiles, c.targetLevel, roadFlow, baseCap, useRoadLevels, &plan.tileTargetLevel);
    if (inc.cost <= 0) continue;

    const double incBenefit = BenefitScore(cfg, inc);
    if (incBenefit <= 0.0) continue;

    if (budget >= 0 && (plan.totalCost + inc.cost) > budget) continue;

    // Accept.
    edgeChosen[static_cast<std::size_t>(c.edgeIndex)] = 1;
    plan.totalCost += inc.cost;
    plan.totalTimeSaved += inc.timeSaved;
    plan.totalExcessReduced += inc.excessReduced;

    // Update per-tile plan.
    for (const Point& p : c.tiles) {
      if (!world.inBounds(p.x, p.y)) continue;
      const std::size_t idx = FlatIdx(p.x, p.y, w);
      if (idx >= plan.tileTargetLevel.size()) continue;
      const std::uint8_t tgt = static_cast<std::uint8_t>(ClampRoadLevelLocal(c.targetLevel));
      plan.tileTargetLevel[idx] = std::max(plan.tileTargetLevel[idx], tgt);
    }

    RoadUpgradeEdge chosen;
    chosen.edgeIndex = c.edgeIndex;
    chosen.a = c.a;
    chosen.b = c.b;
    chosen.targetLevel = c.targetLevel;
    chosen.cost = inc.cost;
    chosen.timeSaved = inc.timeSaved;
    chosen.excessReduced = inc.excessReduced;
    chosen.tileCount = static_cast<int>(c.tiles.size());
    plan.edges.push_back(chosen);

    if (budget == 0) break;
  }

  // Keep edges deterministically ordered by edge index (useful for stable exports).
  std::sort(plan.edges.begin(), plan.edges.end(), [](const RoadUpgradeEdge& a, const RoadUpgradeEdge& b) {
    if (a.edgeIndex != b.edgeIndex) return a.edgeIndex < b.edgeIndex;
    return a.targetLevel < b.targetLevel;
  });

  finalize(plan);
  return plan;
}

void ApplyRoadUpgradePlan(World& world, const RoadUpgradePlan& plan)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;
  if (plan.w != w || plan.h != h) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (plan.tileTargetLevel.size() != n) return;

  for (std::size_t idx = 0; idx < n; ++idx) {
    const std::uint8_t tgt = plan.tileTargetLevel[idx];
    if (tgt == 0) continue;
    const int x = static_cast<int>(idx % static_cast<std::size_t>(w));
    const int y = static_cast<int>(idx / static_cast<std::size_t>(w));
    if (!world.inBounds(x, y)) continue;
    Tile& t = world.at(x, y);
    if (t.overlay != Overlay::Road) continue;
    const int cur = ClampRoadLevelLocal(static_cast<int>(t.level));
    const int newLvl = std::max(cur, ClampRoadLevelLocal(static_cast<int>(tgt)));
    t.level = static_cast<std::uint8_t>(newLvl);
  }

  // Defensive: upgrades do not change connectivity, but bulk tools should
  // keep masks consistent if future changes touch road rules.
  world.recomputeRoadMasks();
}

} // namespace isocity
