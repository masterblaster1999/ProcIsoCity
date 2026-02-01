#include "isocity/Hydrology.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace isocity {

namespace {

static inline std::size_t idx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

} // namespace

void ComputeFlowDir4(const std::vector<float>& heights, int w, int h, std::vector<int>& outDir)
{
  outDir.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);
  if (w <= 0 || h <= 0) return;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (heights.size() != n) return;

  // Deterministic neighbor order.
  constexpr int dx4[4] = {1, -1, 0, 0};
  constexpr int dy4[4] = {0, 0, 1, -1};

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float cur = heights[idx(x, y, w)];
      float bestH = cur;
      int best = -1;

      for (int k = 0; k < 4; ++k) {
        const int nx = x + dx4[k];
        const int ny = y + dy4[k];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

        const float nh = heights[idx(nx, ny, w)];
        if (nh < bestH) {
          bestH = nh;
          best = ny * w + nx;
        }
      }

      outDir[idx(x, y, w)] = best;
    }
  }
}

void ComputeFlowAccumulation(const std::vector<float>& heights, int w, int h,
                             const std::vector<int>& dir, std::vector<int>& outAccum,
                             int* outMaxAccum)
{
  outAccum.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 1);
  if (w <= 0 || h <= 0) {
    if (outMaxAccum) *outMaxAccum = 0;
    return;
  }

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (heights.size() != n || dir.size() != n) {
    if (outMaxAccum) *outMaxAccum = 0;
    return;
  }

  const int total = w * h;

  // Fast path: topological accumulation using in-degree counts.
  //
  // This is O(n) and does not rely on heights being consistent with the flow
  // direction field (dir). It is safe as long as dir is acyclic (the common
  // case, since ComputeFlowDir4 enforces strictly decreasing heights).
  std::vector<int> indeg;
  indeg.assign(n, 0);

  for (int i = 0; i < total; ++i) {
    const int to = dir[static_cast<std::size_t>(i)];
    if (to < 0 || to >= total) continue;
    indeg[static_cast<std::size_t>(to)] += 1;
  }

  std::vector<int> q;
  q.reserve(n);
  for (int i = 0; i < total; ++i) {
    if (indeg[static_cast<std::size_t>(i)] == 0) q.push_back(i);
  }

  std::size_t qHead = 0;
  int processed = 0;

  while (qHead < q.size()) {
    const int i = q[qHead++];
    ++processed;

    const int to = dir[static_cast<std::size_t>(i)];
    if (to < 0 || to >= total) continue;

    outAccum[static_cast<std::size_t>(to)] += outAccum[static_cast<std::size_t>(i)];

    int& d = indeg[static_cast<std::size_t>(to)];
    d -= 1;
    if (d == 0) {
      q.push_back(to);
    }
  }

  if (processed < total) {
    // Malformed dir cycle (should not happen for ComputeFlowDir4) - fall back to
    // a deterministic height-sorted pass (legacy behavior).
    outAccum.assign(n, 1);

    std::vector<int> order;
    order.reserve(n);
    for (int i = 0; i < total; ++i) order.push_back(i);

    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
      const float ha = heights[static_cast<std::size_t>(a)];
      const float hb = heights[static_cast<std::size_t>(b)];

      // Ensure a strict weak ordering even with NaNs.
      const bool na = std::isnan(ha);
      const bool nb = std::isnan(hb);
      if (na != nb) return !na; // non-NaNs first
      if (na && nb) return a < b;

      if (ha == hb) return a < b;
      return ha > hb;
    });

    for (int i : order) {
      const int to = dir[static_cast<std::size_t>(i)];
      if (to < 0 || to >= total) continue;
      outAccum[static_cast<std::size_t>(to)] += outAccum[static_cast<std::size_t>(i)];
    }
  }

  int maxA = 1;
  for (int a : outAccum) maxA = std::max(maxA, a);
  if (outMaxAccum) *outMaxAccum = maxA;
}

HydrologyField BuildHydrologyField(const std::vector<float>& heights, int w, int h)
{
  HydrologyField out;
  out.w = w;
  out.h = h;
  ComputeFlowDir4(heights, w, h, out.dir);
  ComputeFlowAccumulation(heights, w, h, out.dir, out.accum, &out.maxAccum);
  return out;
}

int AutoRiverMinAccum(int w, int h)
{
  if (w <= 0 || h <= 0) return 0;
  const int area = w * h;
  int minA = std::max(32, area / 64);
  minA = std::max(2, minA);
  return minA;
}

std::vector<std::uint8_t> BuildRiverMask(const std::vector<int>& accum, int w, int h, int minAccum)
{
  std::vector<std::uint8_t> mask;
  if (w <= 0 || h <= 0) return mask;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (accum.size() != n) return mask;

  if (minAccum <= 0) minAccum = AutoRiverMinAccum(w, h);
  minAccum = std::max(2, minAccum);

  mask.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    if (accum[i] >= minAccum) mask[i] = 1;
  }
  return mask;
}

BasinSegmentation SegmentBasins(const std::vector<int>& dir, int w, int h)
{
  BasinSegmentation out;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (dir.size() != n) return out;

  out.w = w;
  out.h = h;
  out.basinId.assign(n, -1);

  constexpr int kUnassigned = std::numeric_limits<int>::min();
  std::vector<int> sink(n, kUnassigned);

  std::vector<int> trace;
  trace.reserve(64);

  const int total = w * h;

  for (int i = 0; i < total; ++i) {
    if (sink[static_cast<std::size_t>(i)] != kUnassigned) continue;

    trace.clear();
    int cur = i;
    int steps = 0;
    int sinkVal = -1;

    while (true) {
      if (cur < 0 || cur >= total) {
        sinkVal = -1;
        break;
      }

      const int cached = sink[static_cast<std::size_t>(cur)];
      if (cached != kUnassigned) {
        sinkVal = cached;
        break;
      }

      trace.push_back(cur);

      const int to = dir[static_cast<std::size_t>(cur)];
      if (to < 0) {
        // cur is a sink.
        sinkVal = cur;
        break;
      }

      cur = to;
      if (++steps > total) {
        // Safety guard against malformed dir cycles.
        sinkVal = -1;
        break;
      }
    }

    for (int v : trace) {
      sink[static_cast<std::size_t>(v)] = sinkVal;
    }
    if (sinkVal >= 0 && sink[static_cast<std::size_t>(sinkVal)] == kUnassigned) {
      sink[static_cast<std::size_t>(sinkVal)] = sinkVal;
    }
  }

  // Compute basin areas by sink index.
  std::vector<int> sinkArea(n, 0);
  for (int i = 0; i < total; ++i) {
    const int s = sink[static_cast<std::size_t>(i)];
    if (s < 0 || s >= total) continue;
    sinkArea[static_cast<std::size_t>(s)]++;
  }

  std::vector<int> sinks;
  sinks.reserve(n);
  for (int i = 0; i < total; ++i) {
    if (sinkArea[static_cast<std::size_t>(i)] <= 0) continue;
    // A basin is identified by its sink (dir == -1).
    if (dir[static_cast<std::size_t>(i)] < 0) {
      sinks.push_back(i);
    }
  }

  std::sort(sinks.begin(), sinks.end(), [&](int a, int b) {
    const int aa = sinkArea[static_cast<std::size_t>(a)];
    const int bb = sinkArea[static_cast<std::size_t>(b)];
    if (aa != bb) return aa > bb;
    return a < b;
  });

  std::vector<int> sinkToId(n, -1);
  out.basins.reserve(sinks.size());
  for (int id = 0; id < static_cast<int>(sinks.size()); ++id) {
    const int s = sinks[static_cast<std::size_t>(id)];
    sinkToId[static_cast<std::size_t>(s)] = id;

    BasinInfo info;
    info.id = id;
    info.sinkIndex = s;
    info.sinkX = (w > 0) ? (s % w) : 0;
    info.sinkY = (w > 0) ? (s / w) : 0;
    info.area = sinkArea[static_cast<std::size_t>(s)];
    out.basins.push_back(info);
  }

  for (int i = 0; i < total; ++i) {
    const int s = sink[static_cast<std::size_t>(i)];
    if (s < 0 || s >= total) continue;
    const int id = sinkToId[static_cast<std::size_t>(s)];
    if (id >= 0) out.basinId[static_cast<std::size_t>(i)] = id;
  }

  return out;
}

} // namespace isocity
