#include "isocity/FloodRisk.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace isocity {

namespace {

inline int Idx(int x, int y, int w) { return y * w + x; }

} // namespace

SeaFloodResult ComputeSeaLevelFlood(const std::vector<float>& heights,
                                   int w,
                                   int h,
                                   float seaLevel,
                                   const SeaFloodConfig& cfg)
{
  SeaFloodResult out;
  out.w = w;
  out.h = h;
  out.seaLevel = seaLevel;

  if (w <= 0 || h <= 0) return out;
  const int n = w * h;
  if (static_cast<int>(heights.size()) != n) return out;

  out.flooded.assign(static_cast<std::size_t>(n), 0);
  out.depth.assign(static_cast<std::size_t>(n), 0.0f);

  auto markFlooded = [&](int idx) {
    out.flooded[static_cast<std::size_t>(idx)] = 1;
  };

  auto isFloodable = [&](int idx) -> bool { return heights[static_cast<std::size_t>(idx)] <= seaLevel; };

  if (!cfg.requireEdgeConnection) {
    for (int i = 0; i < n; ++i) {
      if (!isFloodable(i)) continue;
      markFlooded(i);
    }
  } else {
    std::vector<int> queue;
    queue.reserve(static_cast<std::size_t>(n / 8 + 16));

    auto trySeed = [&](int x, int y) {
      const int idx = Idx(x, y, w);
      const std::size_t uidx = static_cast<std::size_t>(idx);
      if (out.flooded[uidx]) return;
      if (!isFloodable(idx)) return;
      out.flooded[uidx] = 1;
      queue.push_back(idx);
    };

    // Seed all floodable boundary cells.
    for (int x = 0; x < w; ++x) {
      trySeed(x, 0);
      if (h > 1) trySeed(x, h - 1);
    }
    for (int y = 1; y + 1 < h; ++y) {
      trySeed(0, y);
      if (w > 1) trySeed(w - 1, y);
    }

    // BFS.
    static constexpr int dx4[4] = {1, -1, 0, 0};
    static constexpr int dy4[4] = {0, 0, 1, -1};

    static constexpr int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static constexpr int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    const int* dx = cfg.eightConnected ? dx8 : dx4;
    const int* dy = cfg.eightConnected ? dy8 : dy4;
    const int kN = cfg.eightConnected ? 8 : 4;

    std::size_t head = 0;
    while (head < queue.size()) {
      const int cur = queue[head++];
      const int cx = cur % w;
      const int cy = cur / w;

      for (int k = 0; k < kN; ++k) {
        const int nx = cx + dx[k];
        const int ny = cy + dy[k];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const int nidx = Idx(nx, ny, w);
        const std::size_t unidx = static_cast<std::size_t>(nidx);
        if (out.flooded[unidx]) continue;
        if (!isFloodable(nidx)) continue;
        out.flooded[unidx] = 1;
        queue.push_back(nidx);
      }
    }
  }

  // Depth + aggregates.
  out.floodedCells = 0;
  out.maxDepth = 0.0f;
  for (int i = 0; i < n; ++i) {
    if (!out.flooded[static_cast<std::size_t>(i)]) continue;
    const float d = std::max(0.0f, seaLevel - heights[static_cast<std::size_t>(i)]);
    out.depth[static_cast<std::size_t>(i)] = d;
    out.floodedCells++;
    if (d > out.maxDepth) out.maxDepth = d;
  }

  return out;
}

ThresholdComponents LabelComponentsAboveThreshold(const std::vector<float>& values,
                                                 int w,
                                                 int h,
                                                 float threshold,
                                                 bool eightConnected)
{
  ThresholdComponents out;
  out.w = w;
  out.h = h;
  out.threshold = threshold;
  out.eightConnected = eightConnected;

  if (w <= 0 || h <= 0) return out;
  const int n = w * h;
  if (static_cast<int>(values.size()) != n) return out;

  out.labels.assign(static_cast<std::size_t>(n), 0);
  out.components.clear();

  static constexpr int dx4[4] = {1, -1, 0, 0};
  static constexpr int dy4[4] = {0, 0, 1, -1};

  static constexpr int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  const int* dx = eightConnected ? dx8 : dx4;
  const int* dy = eightConnected ? dy8 : dy4;
  const int kN = eightConnected ? 8 : 4;

  std::vector<int> queue;
  queue.reserve(static_cast<std::size_t>(n / 8 + 16));

  auto isActive = [&](int idx) -> bool { return values[static_cast<std::size_t>(idx)] > threshold; };

  int nextLabel = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int start = Idx(x, y, w);
      const std::size_t ustart = static_cast<std::size_t>(start);
      if (out.labels[ustart] != 0) continue;
      if (!isActive(start)) continue;

      nextLabel++;
      ThresholdComponent comp;
      comp.label = nextLabel;
      comp.area = 0;
      comp.maxValue = 0.0f;
      comp.sumValue = 0.0;
      comp.sumX = 0.0;
      comp.sumY = 0.0;
      comp.minX = x;
      comp.maxX = x;
      comp.minY = y;
      comp.maxY = y;

      queue.clear();
      queue.push_back(start);
      out.labels[ustart] = nextLabel;

      std::size_t head = 0;
      while (head < queue.size()) {
        const int cur = queue[head++];
        const int cx = cur % w;
        const int cy = cur / w;
        const float v = values[static_cast<std::size_t>(cur)];

        comp.area++;
        comp.sumValue += static_cast<double>(v);
        if (v > comp.maxValue) comp.maxValue = v;

        comp.sumX += static_cast<double>(cx) + 0.5;
        comp.sumY += static_cast<double>(cy) + 0.5;

        comp.minX = std::min(comp.minX, cx);
        comp.maxX = std::max(comp.maxX, cx);
        comp.minY = std::min(comp.minY, cy);
        comp.maxY = std::max(comp.maxY, cy);

        for (int k = 0; k < kN; ++k) {
          const int nx = cx + dx[k];
          const int ny = cy + dy[k];
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
          const int nidx = Idx(nx, ny, w);
          const std::size_t unidx = static_cast<std::size_t>(nidx);
          if (out.labels[unidx] != 0) continue;
          if (!isActive(nidx)) continue;
          out.labels[unidx] = nextLabel;
          queue.push_back(nidx);
        }
      }

      out.components.push_back(comp);
    }
  }

  return out;
}

} // namespace isocity
