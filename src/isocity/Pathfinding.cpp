#include "isocity/Pathfinding.hpp"

#include "isocity/Road.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>

namespace isocity {

namespace {

inline int Idx(int x, int y, int w) { return y * w + x; }

inline bool IsEdge(int x, int y, int w, int h) { return (x == 0 || y == 0 || x == (w - 1) || y == (h - 1)); }

inline int Manhattan(Point a, Point b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); }

bool IsRoad(const World& world, int x, int y)
{
  return world.inBounds(x, y) && world.at(x, y).overlay == Overlay::Road;
}

bool IsLand(const World& world, int x, int y)
{
  return world.inBounds(x, y) && world.at(x, y).terrain != Terrain::Water;
}

bool IsRoadBuildable(const World& world, int x, int y, const RoadBuildPathConfig& cfg)
{
  if (!world.inBounds(x, y)) return false;
  const Tile& t = world.at(x, y);

  // By default we avoid water entirely, but the in-game road planner can opt-in to
  // allowing roads on water (bridges).
  if (!cfg.allowBridges && t.terrain == Terrain::Water) return false;

  // The planner never bulldozes: roads can only exist on empty tiles or on top of
  // existing roads.
  return (t.overlay == Overlay::None) || (t.overlay == Overlay::Road);
}

void ReconstructPath(int goalIdx, int startIdx, int w, const std::vector<int>& cameFrom, std::vector<Point>& out)
{
  out.clear();
  if (goalIdx < 0 || startIdx < 0) return;

  int cur = goalIdx;
  while (cur != -1) {
    const int x = cur % w;
    const int y = cur / w;
    out.push_back(Point{x, y});
    if (cur == startIdx) break;
    cur = cameFrom[static_cast<std::size_t>(cur)];
  }

  std::reverse(out.begin(), out.end());
}

} // namespace

bool FindRoadPathAStar(const World& world, Point start, Point goal, std::vector<Point>& outPath, int* outCost)
{
  outPath.clear();
  if (outCost) *outCost = 0;

  if (!IsRoad(world, start.x, start.y)) return false;
  if (!IsRoad(world, goal.x, goal.y)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const int startIdx = Idx(start.x, start.y, w);
  const int goalIdx = Idx(goal.x, goal.y, w);

  constexpr int kInf = std::numeric_limits<int>::max() / 4;

  std::vector<int> cameFrom(n, -1);
  std::vector<int> gScore(n, kInf);

  struct Node {
    int idx = 0;
    int f = 0;
    int g = 0;
  };
  struct Cmp {
    bool operator()(const Node& a, const Node& b) const
    {
      if (a.f != b.f) return a.f > b.f;
      if (a.g != b.g) return a.g > b.g;
      return a.idx > b.idx;
    }
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  gScore[static_cast<std::size_t>(startIdx)] = 0;
  open.push(Node{startIdx, Manhattan(start, goal), 0});

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();

    // Ignore stale heap entries.
    if (cur.g != gScore[static_cast<std::size_t>(cur.idx)]) continue;

    if (cur.idx == goalIdx) {
      ReconstructPath(goalIdx, startIdx, w, cameFrom, outPath);
      if (outCost) *outCost = static_cast<int>(outPath.size()) - 1;
      return !outPath.empty();
    }

    const int cx = cur.idx % w;
    const int cy = cur.idx / w;

    const int curG = gScore[static_cast<std::size_t>(cur.idx)];
    if (curG == kInf) continue; // should not happen

    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (!IsRoad(world, nx, ny)) continue;

      const int nidx = Idx(nx, ny, w);
      const std::size_t unidx = static_cast<std::size_t>(nidx);

      const int tentativeG = curG + 1;
      if (tentativeG < gScore[unidx]) {
        cameFrom[unidx] = cur.idx;
        gScore[unidx] = tentativeG;
        const int f = tentativeG + Manhattan(Point{nx, ny}, goal);
        open.push(Node{nidx, f, tentativeG});
      }
    }
  }

  return false;
}

bool FindLandPathAStar(const World& world, Point start, Point goal, std::vector<Point>& outPath, int* outCost)
{
  outPath.clear();
  if (outCost) *outCost = 0;

  if (!IsLand(world, start.x, start.y)) return false;
  if (!IsLand(world, goal.x, goal.y)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const int startIdx = Idx(start.x, start.y, w);
  const int goalIdx = Idx(goal.x, goal.y, w);

  // Trivial.
  if (startIdx == goalIdx) {
    outPath.push_back(start);
    return true;
  }

  constexpr int kInf = std::numeric_limits<int>::max() / 4;

  std::vector<int> cameFrom(n, -1);
  std::vector<int> gScore(n, kInf);

  struct Node {
    int idx = 0;
    int f = 0;
    int g = 0;
    std::uint8_t tie = 0; // stable per-tile bits; used only to pick among equal-cost paths
  };
  struct Cmp {
    bool operator()(const Node& a, const Node& b) const
    {
      if (a.f != b.f) return a.f > b.f;
      if (a.g != b.g) return a.g > b.g;
      if (a.tie != b.tie) return a.tie > b.tie;
      return a.idx > b.idx;
    }
  };

  auto tieVal = [&](int idx) -> std::uint8_t {
    const int x = idx % w;
    const int y = idx / w;
    // Use stable tile variation bits to pick between multiple optimal paths.
    // This keeps worlds deterministic but slightly less "grid perfect".
    return world.at(x, y).variation;
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  gScore[static_cast<std::size_t>(startIdx)] = 0;
  open.push(Node{startIdx, Manhattan(start, goal), 0, tieVal(startIdx)});

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();

    // Ignore stale heap entries.
    if (cur.g != gScore[static_cast<std::size_t>(cur.idx)]) continue;

    if (cur.idx == goalIdx) {
      ReconstructPath(goalIdx, startIdx, w, cameFrom, outPath);
      if (outCost) *outCost = static_cast<int>(outPath.size()) - 1;
      return !outPath.empty();
    }

    const int cx = cur.idx % w;
    const int cy = cur.idx / w;

    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (!IsLand(world, nx, ny)) continue;

      const int nidx = Idx(nx, ny, w);
      const std::size_t unidx = static_cast<std::size_t>(nidx);

      const int tentativeG = cur.g + 1;
      if (tentativeG < gScore[unidx]) {
        cameFrom[unidx] = cur.idx;
        gScore[unidx] = tentativeG;
        const int f = tentativeG + Manhattan(Point{nx, ny}, goal);
        open.push(Node{nidx, f, tentativeG, tieVal(nidx)});
      }
    }
  }

  return false;
}


bool FindRoadBuildPath(const World& world, Point start, Point goal, std::vector<Point>& outPath,
                       int* outPrimaryCost, const RoadBuildPathConfig& cfg)
{
  outPath.clear();
  if (outPrimaryCost) *outPrimaryCost = 0;

  if (!IsRoadBuildable(world, start.x, start.y, cfg)) return false;
  if (!IsRoadBuildable(world, goal.x, goal.y, cfg)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const int startIdx = Idx(start.x, start.y, w);
  const int goalIdx = Idx(goal.x, goal.y, w);

  const int targetLevel = ClampRoadLevel(cfg.targetLevel);

  constexpr int kInf = std::numeric_limits<int>::max() / 4;

  auto slopePenalty = [&](int x0, int y0, int x1, int y1) -> int {
    if (cfg.slopeCost <= 0) return 0;
    if (!world.inBounds(x0, y0) || !world.inBounds(x1, y1)) return 0;

    const Tile& from = world.at(x0, y0);
    const Tile& to = world.at(x1, y1);

    // By default, don't penalize traversing *existing* roads so the planner
    // continues to strongly prefer reuse.
    if (!cfg.slopeCostAffectsExistingRoads && to.overlay == Overlay::Road) {
      return 0;
    }

    float dh = to.height - from.height;
    if (dh < 0.0f) dh = -dh;

    // slopeCost is interpreted as "cost units per 1.0 height delta".
    // Convert to an integer penalty with a stable round-to-nearest.
    const float raw = dh * static_cast<float>(cfg.slopeCost);
    int pen = static_cast<int>(raw + 0.5f);
    if (pen < 0) pen = 0;
    if (pen > (kInf / 8)) pen = (kInf / 8);
    return pen;
  };

  auto tileCost = [&](int x, int y) -> int {
    if (!world.inBounds(x, y)) return kInf;
    const Tile& t = world.at(x, y);

    if (cfg.costModel == RoadBuildPathConfig::CostModel::NewTiles) {
      return (t.overlay == Overlay::Road) ? 0 : 1;
    }

    const bool isBridge = (t.terrain == Terrain::Water);
    if (t.overlay == Overlay::Road) {
      return RoadPlacementCost(static_cast<int>(t.level), targetLevel, /*alreadyRoad=*/true, isBridge);
    }
    // overlay==None by buildability rules
    return RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
  };

  auto computePrimaryCost = [&](const std::vector<Point>& path) -> int {
    int cost = 0;
    if (path.empty()) return 0;

    // Tile costs: cost of building/upgrading each tile in the path.
    for (const Point& p : path) {
      const int c = tileCost(p.x, p.y);
      if (c >= kInf) continue;
      cost += c;
    }

    // Slope costs: per-move penalty (optional).
    for (std::size_t i = 1; i < path.size(); ++i) {
      const Point& a = path[i - 1];
      const Point& b = path[i];
      cost += slopePenalty(a.x, a.y, b.x, b.y);
    }
    return cost;
  };

  // Trivial.
  if (startIdx == goalIdx) {
    outPath.push_back(start);
    if (outPrimaryCost) *outPrimaryCost = computePrimaryCost(outPath);
    return true;
  }

  // Dijkstra on an expanded state space (tile, incoming-direction) so we can tie-break
  // equal-cost/equal-step solutions by fewer turns.
  //
  // Optimization order:
  //   1) primary cost (new tiles OR money cost)
  //   2) steps (tile edges)
  //   3) turns (direction changes)
  //   4) stable per-tile variation bits
  // This yields straighter, more "human" road plans while preserving the cost semantics.
  constexpr int kDirNone = 4;
  constexpr int kDirCount = 5;

  const std::size_t ns = n * static_cast<std::size_t>(kDirCount);
  std::vector<int> bestCost(ns, kInf);
  std::vector<int> bestSteps(ns, kInf);
  std::vector<int> bestTurns(ns, kInf);
  std::vector<int> cameFrom(ns, -1);

  struct Node {
    int state = 0; // idx*kDirCount + dir
    int cost = 0;
    int steps = 0;
    int turns = 0;
    std::uint8_t tie = 0;
  };
  struct Cmp {
    bool operator()(const Node& a, const Node& b) const
    {
      if (a.cost != b.cost) return a.cost > b.cost;
      if (a.steps != b.steps) return a.steps > b.steps;
      if (a.turns != b.turns) return a.turns > b.turns;
      if (a.tie != b.tie) return a.tie > b.tie;
      return a.state > b.state;
    }
  };

  auto tieVal = [&](int tileIdx) -> std::uint8_t {
    const int x = tileIdx % w;
    const int y = tileIdx / w;
    return world.at(x, y).variation;
  };

  auto better = [&](int c, int s, int t, int oc, int os, int ot) -> bool {
    if (c != oc) return c < oc;
    if (s != os) return s < os;
    return t < ot;
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  const int startState = startIdx * kDirCount + kDirNone;
  const int startCost = tileCost(start.x, start.y);
  if (startCost >= kInf) return false;

  bestCost[static_cast<std::size_t>(startState)] = startCost;
  bestSteps[static_cast<std::size_t>(startState)] = 0;
  bestTurns[static_cast<std::size_t>(startState)] = 0;
  open.push(Node{startState, startCost, 0, 0, tieVal(startIdx)});

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();

    const std::size_t ucur = static_cast<std::size_t>(cur.state);
    if (cur.cost != bestCost[ucur] || cur.steps != bestSteps[ucur] || cur.turns != bestTurns[ucur]) continue; // stale

    const int curTileIdx = cur.state / kDirCount;
    const int curDir = cur.state % kDirCount;

    if (curTileIdx == goalIdx) {
      // Reconstruct path by following state parents.
      outPath.clear();
      int s = cur.state;
      while (s != -1) {
        const int idx = s / kDirCount;
        outPath.push_back(Point{idx % w, idx / w});
        s = cameFrom[static_cast<std::size_t>(s)];
      }
      std::reverse(outPath.begin(), outPath.end());
      if (outPrimaryCost) *outPrimaryCost = computePrimaryCost(outPath);
      return !outPath.empty();
    }

    const int cx = curTileIdx % w;
    const int cy = curTileIdx / w;

    for (int d = 0; d < 4; ++d) {
      const int nx = cx + dirs[d][0];
      const int ny = cy + dirs[d][1];
      if (!IsRoadBuildable(world, nx, ny, cfg)) continue;

      const int nTileIdx = Idx(nx, ny, w);
      const int nState = nTileIdx * kDirCount + d;
      const std::size_t un = static_cast<std::size_t>(nState);

      const int stepCost = tileCost(nx, ny);
      if (stepCost >= kInf) continue;

      const int slopeCost = slopePenalty(cx, cy, nx, ny);

      const int nCost = cur.cost + stepCost + slopeCost;
      const int nSteps = cur.steps + 1;
      const int nTurns = cur.turns + ((curDir != kDirNone && d != curDir) ? 1 : 0);

      if (better(nCost, nSteps, nTurns, bestCost[un], bestSteps[un], bestTurns[un])) {
        bestCost[un] = nCost;
        bestSteps[un] = nSteps;
        bestTurns[un] = nTurns;
        cameFrom[un] = cur.state;
        open.push(Node{nState, nCost, nSteps, nTurns, tieVal(nTileIdx)});
      }
    }
  }

  return false;
}



bool FindRoadBuildPathBetweenSets(const World& world,
                                 const std::vector<Point>& starts,
                                 const std::vector<Point>& goals,
                                 std::vector<Point>& outPath,
                                 int* outPrimaryCost,
                                 const RoadBuildPathConfig& cfg,
                                 const std::vector<std::uint64_t>* blockedDirectedMoves,
                                 int maxPrimaryCost)
{
  outPath.clear();
  if (outPrimaryCost) *outPrimaryCost = 0;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  // Filter/validate start & goal sets.
  std::vector<int> startIdxs;
  std::vector<int> goalIdxs;
  startIdxs.reserve(starts.size());
  goalIdxs.reserve(goals.size());

  for (const Point& p : starts) {
    if (!IsRoadBuildable(world, p.x, p.y, cfg)) continue;
    startIdxs.push_back(Idx(p.x, p.y, w));
  }
  for (const Point& p : goals) {
    if (!IsRoadBuildable(world, p.x, p.y, cfg)) continue;
    goalIdxs.push_back(Idx(p.x, p.y, w));
  }

  if (startIdxs.empty() || goalIdxs.empty()) return false;

  // De-dup for determinism / less work.
  std::sort(startIdxs.begin(), startIdxs.end());
  startIdxs.erase(std::unique(startIdxs.begin(), startIdxs.end()), startIdxs.end());
  std::sort(goalIdxs.begin(), goalIdxs.end());
  goalIdxs.erase(std::unique(goalIdxs.begin(), goalIdxs.end()), goalIdxs.end());

  // Prepare goal mask for O(1) membership checks.
  std::vector<std::uint8_t> isGoal(n, std::uint8_t{0});
  for (const int gi : goalIdxs) {
    if (gi >= 0 && static_cast<std::size_t>(gi) < n) isGoal[static_cast<std::size_t>(gi)] = 1;
  }

  const int targetLevel = ClampRoadLevel(cfg.targetLevel);

  constexpr int kInf = std::numeric_limits<int>::max() / 4;

  auto slopePenalty = [&](int x0, int y0, int x1, int y1) -> int {
    if (cfg.slopeCost <= 0) return 0;
    if (!world.inBounds(x0, y0) || !world.inBounds(x1, y1)) return 0;

    const Tile& from = world.at(x0, y0);
    const Tile& to = world.at(x1, y1);

    if (!cfg.slopeCostAffectsExistingRoads && to.overlay == Overlay::Road) {
      return 0;
    }

    float dh = to.height - from.height;
    if (dh < 0.0f) dh = -dh;

    const float raw = dh * static_cast<float>(cfg.slopeCost);
    int pen = static_cast<int>(raw + 0.5f);
    if (pen < 0) pen = 0;
    if (pen > (kInf / 8)) pen = (kInf / 8);
    return pen;
  };

  auto tileCost = [&](int x, int y) -> int {
    if (!world.inBounds(x, y)) return kInf;
    const Tile& t = world.at(x, y);

    if (cfg.costModel == RoadBuildPathConfig::CostModel::NewTiles) {
      return (t.overlay == Overlay::Road) ? 0 : 1;
    }

    const bool isBridge = (t.terrain == Terrain::Water);
    if (t.overlay == Overlay::Road) {
      return RoadPlacementCost(static_cast<int>(t.level), targetLevel, /*alreadyRoad=*/true, isBridge);
    }
    // overlay==None by buildability rules
    return RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
  };

  auto computePrimaryCost = [&](const std::vector<Point>& path) -> int {
    int cost = 0;
    if (path.empty()) return 0;

    for (const Point& p : path) {
      const int c = tileCost(p.x, p.y);
      if (c >= kInf) continue;
      cost += c;
    }

    for (std::size_t i = 1; i < path.size(); ++i) {
      const Point& a = path[i - 1];
      const Point& b = path[i];
      cost += slopePenalty(a.x, a.y, b.x, b.y);
    }
    return cost;
  };

  // Quick win: any start is already a goal.
  // Pick the minimal-cost tile (then lowest idx) so maxPrimaryCost works correctly.
  int bestTrivialIdx = -1;
  int bestTrivialCost = kInf;
  for (const int si : startIdxs) {
    if (si < 0 || static_cast<std::size_t>(si) >= n) continue;
    if (!isGoal[static_cast<std::size_t>(si)]) continue;

    const int x = si % w;
    const int y = si / w;
    const int c = tileCost(x, y);
    if (c < bestTrivialCost || (c == bestTrivialCost && (bestTrivialIdx < 0 || si < bestTrivialIdx))) {
      bestTrivialIdx = si;
      bestTrivialCost = c;
    }
  }
  if (bestTrivialIdx >= 0) {
    outPath.push_back(Point{bestTrivialIdx % w, bestTrivialIdx / w});
    if (outPrimaryCost) *outPrimaryCost = computePrimaryCost(outPath);
    if (maxPrimaryCost >= 0 && outPrimaryCost && *outPrimaryCost > maxPrimaryCost) {
      outPath.clear();
      *outPrimaryCost = 0;
      return false;
    }
    return true;
  }

  // Normalize/prepare blocked directed moves.
  const std::vector<std::uint64_t>* blocked = blockedDirectedMoves;
  std::vector<std::uint64_t> blockedLocal;
  if (blocked && !blocked->empty() && !std::is_sorted(blocked->begin(), blocked->end())) {
    blockedLocal = *blocked;
    std::sort(blockedLocal.begin(), blockedLocal.end());
    blockedLocal.erase(std::unique(blockedLocal.begin(), blockedLocal.end()), blockedLocal.end());
    blocked = &blockedLocal;
  }

  auto packMove = [&](int fromIdx, int toIdx) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(fromIdx)) << 32) |
           static_cast<std::uint64_t>(static_cast<std::uint32_t>(toIdx));
  };
  auto isBlockedMove = [&](int fromIdx, int toIdx) -> bool {
    if (!blocked || blocked->empty()) return false;
    const std::uint64_t key = packMove(fromIdx, toIdx);
    return std::binary_search(blocked->begin(), blocked->end(), key);
  };

  // Multi-source Dijkstra on (tile, incoming-direction) states.
  // Optimization order matches FindRoadBuildPath().
  constexpr int kDirNone = 4;
  constexpr int kDirCount = 5;

  const std::size_t ns = n * static_cast<std::size_t>(kDirCount);
  std::vector<int> bestCost(ns, kInf);
  std::vector<int> bestSteps(ns, kInf);
  std::vector<int> bestTurns(ns, kInf);
  std::vector<int> cameFrom(ns, -1);

  struct Node {
    int state = 0;
    int cost = 0;
    int steps = 0;
    int turns = 0;
    std::uint8_t tie = 0;
  };
  struct Cmp {
    bool operator()(const Node& a, const Node& b) const
    {
      if (a.cost != b.cost) return a.cost > b.cost;
      if (a.steps != b.steps) return a.steps > b.steps;
      if (a.turns != b.turns) return a.turns > b.turns;
      if (a.tie != b.tie) return a.tie > b.tie;
      return a.state > b.state;
    }
  };

  auto tieVal = [&](int tileIdx) -> std::uint8_t {
    const int x = tileIdx % w;
    const int y = tileIdx / w;
    return world.at(x, y).variation;
  };

  auto better = [&](int c, int s, int t, int oc, int os, int ot) -> bool {
    if (c != oc) return c < oc;
    if (s != os) return s < os;
    return t < ot;
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  // Seed all starts (direction = none). Include the start tile's own build/upgrade cost.
  for (const int si : startIdxs) {
    if (si < 0 || static_cast<std::size_t>(si) >= n) continue;

    const int sx = si % w;
    const int sy = si / w;
    const int sCost = tileCost(sx, sy);
    if (sCost >= kInf) continue;
    if (maxPrimaryCost >= 0 && sCost > maxPrimaryCost) continue;

    const int sState = si * kDirCount + kDirNone;
    const std::size_t us = static_cast<std::size_t>(sState);

    if (better(sCost, 0, 0, bestCost[us], bestSteps[us], bestTurns[us])) {
      bestCost[us] = sCost;
      bestSteps[us] = 0;
      bestTurns[us] = 0;
      cameFrom[us] = -1;
      open.push(Node{sState, sCost, 0, 0, tieVal(si)});
    }
  }

  if (open.empty()) return false;

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  int foundGoalState = -1;

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();

    const std::size_t ucur = static_cast<std::size_t>(cur.state);
    if (cur.cost != bestCost[ucur] || cur.steps != bestSteps[ucur] || cur.turns != bestTurns[ucur]) continue; // stale

    // Budget/cost cutoff.
    if (maxPrimaryCost >= 0 && cur.cost > maxPrimaryCost) continue;

    const int curTileIdx = cur.state / kDirCount;
    const int curDir = cur.state % kDirCount;

    if (isGoal[static_cast<std::size_t>(curTileIdx)]) {
      foundGoalState = cur.state;
      break;
    }

    const int cx = curTileIdx % w;
    const int cy = curTileIdx / w;

    for (int d = 0; d < 4; ++d) {
      const int nx = cx + dirs[d][0];
      const int ny = cy + dirs[d][1];
      if (!IsRoadBuildable(world, nx, ny, cfg)) continue;

      const int nTileIdx = Idx(nx, ny, w);
      if (isBlockedMove(curTileIdx, nTileIdx)) continue;

      const int stepCost = tileCost(nx, ny);
      if (stepCost >= kInf) continue;

      const int slopeCost = slopePenalty(cx, cy, nx, ny);

      const int nCost = cur.cost + stepCost + slopeCost;
      const int nSteps = cur.steps + 1;
      const int nTurns = cur.turns + ((curDir != kDirNone && d != curDir) ? 1 : 0);

      if (maxPrimaryCost >= 0 && nCost > maxPrimaryCost) continue;

      const int nState = nTileIdx * kDirCount + d;
      const std::size_t un = static_cast<std::size_t>(nState);

      if (better(nCost, nSteps, nTurns, bestCost[un], bestSteps[un], bestTurns[un])) {
        bestCost[un] = nCost;
        bestSteps[un] = nSteps;
        bestTurns[un] = nTurns;
        cameFrom[un] = cur.state;
        open.push(Node{nState, nCost, nSteps, nTurns, tieVal(nTileIdx)});
      }
    }
  }

  if (foundGoalState < 0) return false;

  // Reconstruct to the multi-source root (cameFrom == -1).
  outPath.clear();
  int s = foundGoalState;
  while (s != -1) {
    const int idx = s / kDirCount;
    outPath.push_back(Point{idx % w, idx / w});
    s = cameFrom[static_cast<std::size_t>(s)];
  }
  std::reverse(outPath.begin(), outPath.end());

  if (outPrimaryCost) *outPrimaryCost = computePrimaryCost(outPath);

  if (maxPrimaryCost >= 0 && outPrimaryCost && *outPrimaryCost > maxPrimaryCost) {
    outPath.clear();
    *outPrimaryCost = 0;
    return false;
  }

  return !outPath.empty();
}


bool FindRoadPathToEdge(const World& world, Point start, std::vector<Point>& outPath, int* outCost)
{
  outPath.clear();
  if (outCost) *outCost = 0;

  if (!IsRoad(world, start.x, start.y)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const int startIdx = Idx(start.x, start.y, w);

  std::vector<int> cameFrom(n, -1);
  std::vector<std::uint8_t> visited(n, std::uint8_t{0});

  std::queue<int> q;
  q.push(startIdx);
  visited[static_cast<std::size_t>(startIdx)] = 1;

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  int foundIdx = -1;
  while (!q.empty()) {
    const int curIdx = q.front();
    q.pop();

    const int cx = curIdx % w;
    const int cy = curIdx / w;

    if (IsEdge(cx, cy, w, h)) {
      foundIdx = curIdx;
      break;
    }

    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (!IsRoad(world, nx, ny)) continue;

      const int nidx = Idx(nx, ny, w);
      const std::size_t unidx = static_cast<std::size_t>(nidx);
      if (visited[unidx]) continue;
      visited[unidx] = 1;
      cameFrom[unidx] = curIdx;
      q.push(nidx);
    }
  }

  if (foundIdx < 0) return false;

  ReconstructPath(foundIdx, startIdx, w, cameFrom, outPath);
  if (outCost) *outCost = static_cast<int>(outPath.size()) - 1;
  return !outPath.empty();
}


void ComputeRoadsConnectedToEdge(const World& world, std::vector<std::uint8_t>& outRoadToEdge)
{
  const int w = world.width();
  const int h = world.height();

  outRoadToEdge.clear();
  if (w <= 0 || h <= 0) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  outRoadToEdge.assign(n, 0);

  std::vector<int> queue;
  queue.reserve(n / 8);

  auto push = [&](int x, int y) {
    const int idx = Idx(x, y, w);
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx >= outRoadToEdge.size()) return;
    if (outRoadToEdge[uidx]) return;
    outRoadToEdge[uidx] = 1;
    queue.push_back(idx);
  };

  // Seed BFS with any road tile on the map border.
  for (int x = 0; x < w; ++x) {
    if (IsRoad(world, x, 0)) push(x, 0);
    if (h > 1 && IsRoad(world, x, h - 1)) push(x, h - 1);
  }
  for (int y = 1; y < h - 1; ++y) {
    if (IsRoad(world, 0, y)) push(0, y);
    if (w > 1 && IsRoad(world, w - 1, y)) push(w - 1, y);
  }

  // No border roads => no outside connection.
  if (queue.empty()) return;

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  std::size_t qHead = 0;
  while (qHead < queue.size()) {
    const int idx = queue[qHead++];
    const int x = idx % w;
    const int y = idx / w;

    for (const auto& d : dirs) {
      const int nx = x + d[0];
      const int ny = y + d[1];
      if (!IsRoad(world, nx, ny)) continue;

      const int nidx = Idx(nx, ny, w);
      const std::size_t unidx = static_cast<std::size_t>(nidx);
      if (unidx >= outRoadToEdge.size()) continue;
      if (outRoadToEdge[unidx]) continue;

      outRoadToEdge[unidx] = 1;
      queue.push_back(nidx);
    }
  }
}

bool HasAdjacentRoadConnectedToEdge(const World& world, const std::vector<std::uint8_t>& roadToEdge, int x, int y)
{
  if (!world.inBounds(x, y)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (roadToEdge.size() != expected) return false;

  auto check = [&](int nx, int ny) -> bool {
    if (!world.inBounds(nx, ny)) return false;
    if (world.at(nx, ny).overlay != Overlay::Road) return false;
    const int idx = Idx(nx, ny, w);
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx >= roadToEdge.size()) return false;
    return roadToEdge[uidx] != 0;
  };

  return check(x + 1, y) || check(x - 1, y) || check(x, y + 1) || check(x, y - 1);
}

bool PickAdjacentRoadTile(const World& world, const std::vector<std::uint8_t>* roadToEdgeMask, int x, int y,
                          Point& outRoad)
{
  outRoad = Point{-1, -1};
  if (!world.inBounds(x, y)) return false;

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return false;

  const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const bool useMask = (roadToEdgeMask && roadToEdgeMask->size() == expected);

  auto ok = [&](int rx, int ry) -> bool {
    if (!world.inBounds(rx, ry)) return false;
    if (world.at(rx, ry).overlay != Overlay::Road) return false;
    if (!useMask) return true;
    const int idx = Idx(rx, ry, w);
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (uidx >= roadToEdgeMask->size()) return false;
    return (*roadToEdgeMask)[uidx] != 0;
  };

  // Deterministic order: N, E, S, W
  const int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (const auto& d : dirs) {
    const int nx = x + d[0];
    const int ny = y + d[1];
    if (!ok(nx, ny)) continue;
    outRoad = Point{nx, ny};
    return true;
  }

  return false;
}

} // namespace isocity
