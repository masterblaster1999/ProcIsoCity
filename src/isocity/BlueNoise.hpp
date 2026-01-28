#pragma once

#include "isocity/Random.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

// Deterministic, tileable "blue-noise-ish" thresholding for binary placement decisions.
//
// Motivation:
//  - Pure per-tile hashing is great for randomness, but at low densities it tends to clump.
//  - A blue-noise style threshold map produces more even spacing (pleasant for props like trees/lights)
//    while remaining fully deterministic.
//  - We generate a toroidal farthest-point sequence once (size NxN), then use the per-cell rank
//    as a threshold for "is this cell active at density p?" decisions.
//
// This is not a perfect void-and-cluster blue noise implementation, but farthest-point sampling
// yields a very usable blue-noise-like ordering with a tiny, dependency-free implementation.

namespace detail {

inline int WrapMod(int i, int m)
{
  if (m <= 0) return i;
  const int r = i % m;
  return (r < 0) ? (r + m) : r;
}

inline int ToroidalDelta(int a, int b, int period)
{
  const int d = (a > b) ? (a - b) : (b - a);
  return std::min(d, period - d);
}

inline int ToroidalDist2(int ax, int ay, int bx, int by, int period)
{
  const int dx = ToroidalDelta(ax, bx, period);
  const int dy = ToroidalDelta(ay, by, period);
  return dx * dx + dy * dy;
}

// Build a rank map using greedy toroidal farthest-point sampling.
// rank[idx] is the step at which that cell is activated (0..N*N-1).
inline std::vector<std::uint16_t> BuildBlueNoiseRankMapTorus(int size, std::uint64_t seed)
{
  // 255 ensures size*size <= 65025 so ranks fit in uint16_t.
  size = std::clamp(size, 2, 255); // defensive, though we only use 64.
  const int n = size * size;

  std::vector<std::uint16_t> rank(static_cast<std::size_t>(n), 0u);
  std::vector<int> dist2(static_cast<std::size_t>(n), std::numeric_limits<int>::max());
  std::vector<std::uint8_t> used(static_cast<std::size_t>(n), 0u);

  RNG rng(seed);
  const int first = static_cast<int>(rng.rangeU32(static_cast<std::uint32_t>(n)));

  auto idxToXY = [&](int idx, int& outX, int& outY) {
    outX = idx % size;
    outY = idx / size;
  };

  auto dist2To = [&](int aIdx, int bIdx) -> int {
    int ax = 0, ay = 0, bx = 0, by = 0;
    idxToXY(aIdx, ax, ay);
    idxToXY(bIdx, bx, by);
    return ToroidalDist2(ax, ay, bx, by, size);
  };

  used[static_cast<std::size_t>(first)] = 1u;
  rank[static_cast<std::size_t>(first)] = 0u;
  dist2[static_cast<std::size_t>(first)] = -1;

  // Initial distance field.
  for (int idx = 0; idx < n; ++idx) {
    if (idx == first) continue;
    dist2[static_cast<std::size_t>(idx)] = dist2To(idx, first);
  }

  for (int step = 1; step < n; ++step) {
    int bestIdx = -1;
    int bestD2 = -1;

    // Choose the cell farthest from the current set (max of distance-to-nearest).
    for (int idx = 0; idx < n; ++idx) {
      if (used[static_cast<std::size_t>(idx)] != 0u) continue;
      const int d2 = dist2[static_cast<std::size_t>(idx)];
      if (d2 > bestD2) {
        bestD2 = d2;
        bestIdx = idx;
      }
    }

    if (bestIdx < 0) break; // should never happen

    used[static_cast<std::size_t>(bestIdx)] = 1u;
    rank[static_cast<std::size_t>(bestIdx)] = static_cast<std::uint16_t>(step);
    dist2[static_cast<std::size_t>(bestIdx)] = -1;

    // Update distance field (take min with distance to the newly added point).
    for (int idx = 0; idx < n; ++idx) {
      if (used[static_cast<std::size_t>(idx)] != 0u) continue;
      const int d2 = dist2To(idx, bestIdx);
      int& cur = dist2[static_cast<std::size_t>(idx)];
      if (d2 < cur) cur = d2;
    }
  }

  return rank;
}

} // namespace detail

inline constexpr int kBlueNoiseTiledSize = 64;

// Return the shared rank-map for size 64 (computed once, deterministic).
inline const std::vector<std::uint16_t>& BlueNoiseRankMap64()
{
  static const std::vector<std::uint16_t> kRank = []() -> std::vector<std::uint16_t> {
    // Fixed seed so the rank map is identical on every machine/build.
    return detail::BuildBlueNoiseRankMapTorus(kBlueNoiseTiledSize, 0xC0FFEE1234ULL);
  }();
  return kRank;
}

// A stable threshold in [0,1] for the (x,y) cell, with a deterministic offset/rotation derived from (seed,salt).
//
// Use: place if (density > BlueNoiseThreshold01(...)).
inline float BlueNoiseThreshold01(int x, int y, std::uint32_t seed, std::uint32_t salt = 0u)
{
  constexpr int N = kBlueNoiseTiledSize;
  const auto& rank = BlueNoiseRankMap64();

  const std::uint32_t s = seed ^ salt ^ 0x9E3779B9u;

  // Deterministic tile offset (breaks visible alignment across different seeds).
  const int ox = static_cast<int>(HashCoords32(113, 127, s) % static_cast<std::uint32_t>(N));
  const int oy = static_cast<int>(HashCoords32(131, 149, s) % static_cast<std::uint32_t>(N));

  int ix = detail::WrapMod(x + ox, N);
  int iy = detail::WrapMod(y + oy, N);

  // Deterministic global rotation/flip (also seed-dependent).
  const std::uint32_t h = HashCoords32(17, 19, s ^ 0xA341316Cu);
  const int rot = static_cast<int>(h & 3u);
  const bool flipX = ((h >> 2) & 1u) != 0u;

  int rx = ix, ry = iy;
  switch (rot) {
  default:
  case 0: rx = ix;         ry = iy;         break;
  case 1: rx = (N - 1) - iy; ry = ix;         break;
  case 2: rx = (N - 1) - ix; ry = (N - 1) - iy; break;
  case 3: rx = iy;         ry = (N - 1) - ix; break;
  }

  if (flipX) rx = (N - 1) - rx;

  const std::size_t idx = static_cast<std::size_t>(ry) * static_cast<std::size_t>(N) + static_cast<std::size_t>(rx);
  const std::uint16_t r = rank[idx];

  return (static_cast<float>(r) + 0.5f) / static_cast<float>(N * N);
}

inline bool BlueNoiseChance(int x, int y, float p, std::uint32_t seed, std::uint32_t salt = 0u)
{
  p = std::clamp(p, 0.0f, 1.0f);
  return BlueNoiseThreshold01(x, y, seed, salt) < p;
}

} // namespace isocity
