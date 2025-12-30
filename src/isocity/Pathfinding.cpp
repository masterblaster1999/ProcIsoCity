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

  auto computePrimaryCost = [&](const std::vector<Point>& path) -> int {
    int cost = 0;
    for (const Point& p : path) {
      if (!world.inBounds(p.x, p.y)) continue;
      const Tile& t = world.at(p.x, p.y);
      if (cfg.costModel == RoadBuildPathConfig::CostModel::NewTiles) {
        if (t.overlay != Overlay::Road) ++cost;
      } else {
        const bool isBridge = (t.terrain == Terrain::Water);
        if (t.overlay == Overlay::Road) {
          cost += RoadPlacementCost(static_cast<int>(t.level), targetLevel, /*alreadyRoad=*/true, isBridge);
        } else if (t.overlay == Overlay::None) {
          cost += RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
        }
      }
    }
    return cost;
  };

  // Trivial.
  if (startIdx == goalIdx) {
    outPath.push_back(start);
    if (outPrimaryCost) *outPrimaryCost = computePrimaryCost(outPath);
    return true;
  }

  constexpr int kInf = std::numeric_limits<int>::max() / 4;

  // We use Dijkstra (A* with 0 heuristic). Cost of entering a tile depends on cfg.costModel:
  //  - NewTiles: 0 if it's already a road, 1 if it's empty (needs a new road)
  //  - Money:    exact build/upgrade money cost to reach targetLevel (includes bridges)
  // and we tie-break by fewer steps, then stable per-tile variation bits.
  std::vector<int> bestCost(n, kInf);
  std::vector<int> bestSteps(n, kInf);
  std::vector<int> cameFrom(n, -1);

  struct Node {
    int idx = 0;
    int cost = 0;
    int steps = 0;
    std::uint8_t tie = 0;
  };
  struct Cmp {
    bool operator()(const Node& a, const Node& b) const
    {
      if (a.cost != b.cost) return a.cost > b.cost;
      if (a.steps != b.steps) return a.steps > b.steps;
      if (a.tie != b.tie) return a.tie > b.tie;
      return a.idx > b.idx;
    }
  };

  auto tieVal = [&](int idx) -> std::uint8_t {
    const int x = idx % w;
    const int y = idx / w;
    return world.at(x, y).variation;
  };

  auto tileEnterCost = [&](int x, int y) -> int {
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

  std::priority_queue<Node, std::vector<Node>, Cmp> open;

  bestCost[static_cast<std::size_t>(startIdx)] = 0;
  bestSteps[static_cast<std::size_t>(startIdx)] = 0;
  open.push(Node{startIdx, 0, 0, tieVal(startIdx)});

  constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();

    const std::size_t ucur = static_cast<std::size_t>(cur.idx);
    if (cur.cost != bestCost[ucur] || cur.steps != bestSteps[ucur]) continue; // stale

    if (cur.idx == goalIdx) {
      ReconstructPath(goalIdx, startIdx, w, cameFrom, outPath);
      if (outPrimaryCost) *outPrimaryCost = computePrimaryCost(outPath);
      return !outPath.empty();
    }

    const int cx = cur.idx % w;
    const int cy = cur.idx / w;

    for (const auto& d : dirs) {
      const int nx = cx + d[0];
      const int ny = cy + d[1];
      if (!IsRoadBuildable(world, nx, ny, cfg)) continue;

      const int nidx = Idx(nx, ny, w);
      const std::size_t unidx = static_cast<std::size_t>(nidx);

      const int stepCost = tileEnterCost(nx, ny);
      if (stepCost >= kInf) continue;

      const int nCost = cur.cost + stepCost;
      const int nSteps = cur.steps + 1;

      if (nCost < bestCost[unidx] || (nCost == bestCost[unidx] && nSteps < bestSteps[unidx])) {
        bestCost[unidx] = nCost;
        bestSteps[unidx] = nSteps;
        cameFrom[unidx] = cur.idx;
        open.push(Node{nidx, nCost, nSteps, tieVal(nidx)});
      }
    }
  }

  return false;
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
  std::vector<std::uint8_t> visited(n, 0);

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
