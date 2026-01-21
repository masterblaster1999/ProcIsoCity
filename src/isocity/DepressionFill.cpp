#include "isocity/DepressionFill.hpp"

#include <algorithm>
#include <cstddef>
#include <queue>
#include <vector>

namespace isocity {

namespace {

struct Node {
  int idx = 0;
  float h = 0.0f;
};

// Min-heap by (h, idx) for deterministic behavior.
struct NodeCmp {
  bool operator()(const Node& a, const Node& b) const
  {
    if (a.h > b.h) return true;
    if (a.h < b.h) return false;
    return a.idx > b.idx;
  }
};

inline int Idx(int x, int y, int w) { return y * w + x; }

} // namespace

DepressionFillResult FillDepressionsPriorityFlood(const std::vector<float>& heights,
                                                  int w,
                                                  int h,
                                                  const std::vector<std::uint8_t>* drainMask,
                                                  const DepressionFillConfig& cfg)
{
  DepressionFillResult out;
  out.w = w;
  out.h = h;

  if (w <= 0 || h <= 0) return out;
  const int n = w * h;
  if (static_cast<int>(heights.size()) != n) return out;

  out.filled = heights;
  out.depth.assign(static_cast<std::size_t>(n), 0.0f);

  std::vector<std::uint8_t> closed(static_cast<std::size_t>(n), std::uint8_t{0});
  std::priority_queue<Node, std::vector<Node>, NodeCmp> pq;

  auto pushSeed = [&](int idx) {
    if (idx < 0 || idx >= n) return;
    const std::size_t uidx = static_cast<std::size_t>(idx);
    if (closed[uidx]) return;
    closed[uidx] = 1;
    pq.push(Node{idx, out.filled[uidx]});
  };

  auto seedEdges = [&]() {
    // Top + bottom rows.
    for (int x = 0; x < w; ++x) {
      pushSeed(Idx(x, 0, w));
      if (h > 1) pushSeed(Idx(x, h - 1, w));
    }
    // Left + right columns (excluding corners already seeded).
    for (int y = 1; y + 1 < h; ++y) {
      pushSeed(Idx(0, y, w));
      if (w > 1) pushSeed(Idx(w - 1, y, w));
    }
  };

  if (cfg.includeEdges) seedEdges();

  if (drainMask && drainMask->size() == static_cast<std::size_t>(n)) {
    for (int i = 0; i < n; ++i) {
      if ((*drainMask)[static_cast<std::size_t>(i)] != 0) pushSeed(i);
    }
  }

  // A completely seedless run is ambiguous; fall back to edge outlets so the
  // algorithm always produces a filled surface.
  if (pq.empty()) seedEdges();

  constexpr int dx4[4] = {1, -1, 0, 0};
  constexpr int dy4[4] = {0, 0, 1, -1};

  const float eps = std::max(0.0f, cfg.epsilon);

  while (!pq.empty()) {
    const Node cur = pq.top();
    pq.pop();

    const int x = cur.idx % w;
    const int y = cur.idx / w;

    for (int k = 0; k < 4; ++k) {
      const int nx = x + dx4[k];
      const int ny = y + dy4[k];
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const int nidx = Idx(nx, ny, w);
      const std::size_t unidx = static_cast<std::size_t>(nidx);
      if (closed[unidx]) continue;

      closed[unidx] = 1;

      const float origH = out.filled[unidx]; // still original (unprocessed)
      float fillH = origH;
      const float minH = cur.h + eps;
      if (fillH < minH) fillH = minH;

      out.filled[unidx] = fillH;
      pq.push(Node{nidx, fillH});
    }
  }

  // Compute depths and aggregates.
  out.filledCells = 0;
  out.maxDepth = 0.0f;
  out.volume = 0.0;

  for (int i = 0; i < n; ++i) {
    const float d = std::max(0.0f, out.filled[static_cast<std::size_t>(i)] - heights[static_cast<std::size_t>(i)]);
    out.depth[static_cast<std::size_t>(i)] = d;
    if (d > 0.0f) {
      out.filledCells++;
      out.volume += static_cast<double>(d);
      if (d > out.maxDepth) out.maxDepth = d;
    }
  }

  return out;
}

} // namespace isocity
