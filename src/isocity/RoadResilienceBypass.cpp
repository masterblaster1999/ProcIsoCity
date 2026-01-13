#include "isocity/RoadResilienceBypass.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/Road.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace isocity {

int CountNewRoadTilesInPath(const World& world, const std::vector<Point>& path)
{
  int out = 0;
  for (const Point& p : path) {
    if (!world.inBounds(p.x, p.y)) continue;
    if (world.at(p.x, p.y).overlay != Overlay::Road) ++out;
  }
  return out;
}

int EstimateMoneyCostForRoadPath(const World& world, const std::vector<Point>& path, int targetLevel)
{
  int outCost = 0;
  targetLevel = ClampRoadLevel(targetLevel);

  for (const Point& p : path) {
    if (!world.inBounds(p.x, p.y)) continue;
    const Tile& t = world.at(p.x, p.y);
    const bool isBridge = (t.terrain == Terrain::Water);

    if (t.overlay == Overlay::Road) {
      const int cur = ClampRoadLevel(static_cast<int>(t.level));
      outCost += RoadPlacementCost(cur, targetLevel, /*alreadyRoad=*/true, isBridge);
    } else {
      // overlay==None by buildability rules
      outCost += RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
    }
  }

  return outCost;
}

std::vector<RoadResilienceBypassSuggestion> SuggestRoadResilienceBypasses(
    const World& world,
    const RoadGraph& roadGraph,
    const RoadGraphResilienceResult& resilience,
    const RoadResilienceBypassConfig& cfg,
    const TrafficResult* traffic)
{
  std::vector<RoadResilienceBypassSuggestion> out;

  if (cfg.top <= 0) return out;
  if (roadGraph.edges.empty() || roadGraph.nodes.empty()) return out;
  if (resilience.bridgeEdges.empty()) return out;

  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) return out;

  const bool haveTraffic =
      (cfg.rankByTraffic && traffic && !traffic->roadTraffic.empty() &&
       static_cast<int>(traffic->roadTraffic.size()) == mapW * mapH);

  struct RankedBridge {
    int ei = -1;
    double score = 0.0;
    int cutSize = 0;
  };

  std::vector<RankedBridge> ranked;
  ranked.reserve(resilience.bridgeEdges.size());

  for (int ei : resilience.bridgeEdges) {
    if (ei < 0 || ei >= static_cast<int>(roadGraph.edges.size())) continue;

    const int sub = (ei < static_cast<int>(resilience.bridgeSubtreeNodes.size()))
                        ? resilience.bridgeSubtreeNodes[static_cast<std::size_t>(ei)]
                        : 0;
    const int oth = (ei < static_cast<int>(resilience.bridgeOtherNodes.size()))
                        ? resilience.bridgeOtherNodes[static_cast<std::size_t>(ei)]
                        : 0;
    const int cut = std::min(sub, oth);

    double score = static_cast<double>(cut);
    if (haveTraffic) {
      const RoadGraphEdge& e = roadGraph.edges[static_cast<std::size_t>(ei)];
      int maxTraffic = 0;
      for (const Point& p : e.tiles) {
        const int idx = p.y * mapW + p.x;
        if (idx < 0 || idx >= static_cast<int>(traffic->roadTraffic.size())) continue;
        maxTraffic = std::max(maxTraffic, static_cast<int>(traffic->roadTraffic[static_cast<std::size_t>(idx)]));
      }
      // Prioritize heavily used bridges, breaking ties by cut size.
      score = static_cast<double>(maxTraffic) + static_cast<double>(cut) * 0.001;
    }

    ranked.push_back(RankedBridge{ei, score, cut});
  }

  std::sort(ranked.begin(), ranked.end(), [](const RankedBridge& a, const RankedBridge& b) {
    if (a.score != b.score) return a.score > b.score;
    if (a.cutSize != b.cutSize) return a.cutSize > b.cutSize;
    return a.ei < b.ei;
  });

  const int want = std::min(cfg.top, static_cast<int>(ranked.size()));
  if (want <= 0) return out;
  out.reserve(static_cast<std::size_t>(want));

  auto sampleNodePositions = [&](const std::vector<int>& nodes, int mustInclude, std::uint64_t seed,
                                 std::vector<Point>& outPts) {
    outPts.clear();
    if (nodes.empty()) return;

    // Always include the bridge-side endpoint if provided.
    if (mustInclude >= 0 && mustInclude < static_cast<int>(roadGraph.nodes.size())) {
      outPts.push_back(roadGraph.nodes[static_cast<std::size_t>(mustInclude)].pos);
    }

    const int maxN = std::max(1, cfg.maxNodesPerSide);
    if (static_cast<int>(nodes.size()) <= maxN) {
      for (int ni : nodes) {
        if (ni == mustInclude) continue;
        if (ni < 0 || ni >= static_cast<int>(roadGraph.nodes.size())) continue;
        outPts.push_back(roadGraph.nodes[static_cast<std::size_t>(ni)].pos);
      }
      return;
    }

    // Deterministic hashed sampling so we don't explode the multi-source frontier.
    std::vector<std::pair<std::uint64_t, int>> scored;
    scored.reserve(nodes.size());
    std::uint64_t st = seed;
    for (int ni : nodes) {
      if (ni == mustInclude) continue;
      if (ni < 0 || ni >= static_cast<int>(roadGraph.nodes.size())) continue;
      st ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(ni)) + 0x9E3779B97F4A7C15ULL;
      const std::uint64_t key = SplitMix64Next(st);
      scored.emplace_back(key, ni);
    }

    const int take = maxN - static_cast<int>(outPts.size());
    if (take <= 0) return;

    if (static_cast<int>(scored.size()) > take) {
      std::nth_element(scored.begin(), scored.begin() + take, scored.end(),
                       [](const auto& a, const auto& b) { return a.first < b.first; });
      scored.resize(static_cast<std::size_t>(take));
    }

    for (const auto& kv : scored) {
      outPts.push_back(roadGraph.nodes[static_cast<std::size_t>(kv.second)].pos);
    }
  };

  RoadBuildPathConfig pcfg;
  pcfg.targetLevel = ClampRoadLevel(cfg.targetLevel);
  pcfg.allowBridges = cfg.allowBridges;
  pcfg.costModel = cfg.moneyObjective ? RoadBuildPathConfig::CostModel::Money : RoadBuildPathConfig::CostModel::NewTiles;

  const int maxCost = (cfg.maxPrimaryCost > 0) ? cfg.maxPrimaryCost : -1;

  for (int i = 0; i < want; ++i) {
    const int bridgeEi = ranked[static_cast<std::size_t>(i)].ei;

    RoadGraphBridgeCut cut;
    if (!ComputeRoadGraphBridgeCut(roadGraph, bridgeEi, cut)) continue;

    // Start from the smaller side so the multi-source frontier stays manageable.
    const std::vector<int>* sideS = &cut.sideA;
    const std::vector<int>* sideG = &cut.sideB;
    int mustS = roadGraph.edges[static_cast<std::size_t>(bridgeEi)].a;
    int mustG = roadGraph.edges[static_cast<std::size_t>(bridgeEi)].b;
    if (cut.sideB.size() < cut.sideA.size()) {
      sideS = &cut.sideB;
      sideG = &cut.sideA;
      mustS = roadGraph.edges[static_cast<std::size_t>(bridgeEi)].b;
      mustG = roadGraph.edges[static_cast<std::size_t>(bridgeEi)].a;
    }

    std::vector<Point> starts;
    std::vector<Point> goals;

    const std::uint64_t seed = (world.seed() ^ (static_cast<std::uint64_t>(bridgeEi) * 0xD6E8FEB86659FD93ULL));
    sampleNodePositions(*sideS, mustS, seed ^ 0xA5A5A5A5A5A5A5A5ULL, starts);
    sampleNodePositions(*sideG, mustG, seed ^ 0x5A5A5A5A5A5A5A5AULL, goals);
    if (starts.empty() || goals.empty()) continue;

    const std::vector<std::uint64_t> blocked = BuildBlockedMovesForRoadGraphEdge(roadGraph, bridgeEi, mapW);

    std::vector<Point> path;
    int primaryCost = 0;

    const bool ok = FindRoadBuildPathBetweenSets(world, starts, goals, path, &primaryCost, pcfg, &blocked, maxCost);
    if (!ok || path.size() < 2) continue;

    RoadResilienceBypassSuggestion s;
    s.bridgeEdge = bridgeEi;
    s.cutSize = ranked[static_cast<std::size_t>(i)].cutSize;
    s.primaryCost = primaryCost;
    s.newTiles = CountNewRoadTilesInPath(world, path);
    s.moneyCost = EstimateMoneyCostForRoadPath(world, path, pcfg.targetLevel);
    s.steps = static_cast<int>(path.size()) - 1;
    s.targetLevel = pcfg.targetLevel;
    s.allowBridges = pcfg.allowBridges;
    s.moneyObjective = cfg.moneyObjective;
    s.path = std::move(path);

    out.push_back(std::move(s));
  }

  return out;
}

RoadResilienceBypassApplyReport ApplyRoadResilienceBypass(World& world, const RoadResilienceBypassSuggestion& s,
                                                         int minMoneyReserve)
{
  RoadResilienceBypassApplyReport rep;
  rep.result = RoadResilienceBypassApplyResult::Noop;
  rep.moneyCost = 0;
  rep.builtTiles = 0;
  rep.upgradedTiles = 0;

  if (s.path.size() < 2) return rep;

  const int targetLevel = ClampRoadLevel(s.targetLevel);

  // Validate buildability and compute the current money cost.
  int moneyCost = 0;
  bool anyChange = false;

  for (const Point& p : s.path) {
    if (!world.inBounds(p.x, p.y)) {
      rep.result = RoadResilienceBypassApplyResult::OutOfBounds;
      return rep;
    }

    const Tile& t = world.at(p.x, p.y);

    if (t.overlay != Overlay::None && t.overlay != Overlay::Road) {
      rep.result = RoadResilienceBypassApplyResult::Blocked;
      return rep;
    }

    if (t.terrain == Terrain::Water && !s.allowBridges) {
      rep.result = RoadResilienceBypassApplyResult::NeedsBridges;
      return rep;
    }

    const bool isBridge = (t.terrain == Terrain::Water);

    if (t.overlay == Overlay::Road) {
      const int cur = ClampRoadLevel(static_cast<int>(t.level));
      const int c = RoadPlacementCost(cur, targetLevel, /*alreadyRoad=*/true, isBridge);
      moneyCost += c;
      if (c > 0) anyChange = true;
    } else {
      const int c = RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
      moneyCost += c;
      if (c > 0) anyChange = true;
    }
  }

  rep.moneyCost = moneyCost;

  if (!anyChange) {
    rep.result = RoadResilienceBypassApplyResult::Noop;
    return rep;
  }

  if (moneyCost > 0 && (world.stats().money - std::max(0, minMoneyReserve) < moneyCost)) {
    rep.result = RoadResilienceBypassApplyResult::InsufficientFunds;
    return rep;
  }

  // Apply.
  const int moneyBefore = world.stats().money;

  for (const Point& p : s.path) {
    const Tile before = world.at(p.x, p.y);
    const ToolApplyResult r = world.applyRoad(p.x, p.y, targetLevel);
    if (r == ToolApplyResult::Applied) {
      if (before.overlay != Overlay::Road) {
        ++rep.builtTiles;
      } else {
        const int cur = ClampRoadLevel(static_cast<int>(before.level));
        if (cur < targetLevel) ++rep.upgradedTiles;
      }
    } else if (r == ToolApplyResult::InsufficientFunds) {
      // Shouldn't happen due to the pre-check, but handle defensively.
      rep.result = RoadResilienceBypassApplyResult::InsufficientFunds;
      rep.moneyCost = moneyBefore - world.stats().money;
      return rep;
    } else if (r == ToolApplyResult::BlockedOccupied || r == ToolApplyResult::BlockedWater) {
      rep.result = RoadResilienceBypassApplyResult::Blocked;
      rep.moneyCost = moneyBefore - world.stats().money;
      return rep;
    }
  }

  rep.moneyCost = moneyBefore - world.stats().money;

  if (rep.builtTiles > 0 || rep.upgradedTiles > 0) {
    rep.result = RoadResilienceBypassApplyResult::Applied;
  } else {
    rep.result = RoadResilienceBypassApplyResult::Noop;
  }

  return rep;
}

} // namespace isocity
