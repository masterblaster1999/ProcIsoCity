#include "isocity/GfxQuantize.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace isocity {

namespace {

inline std::uint32_t PackRGBA(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a)
{
  return (static_cast<std::uint32_t>(r) << 24) | (static_cast<std::uint32_t>(g) << 16) |
         (static_cast<std::uint32_t>(b) << 8) | static_cast<std::uint32_t>(a);
}

inline void UnpackRGBA(std::uint32_t key, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, std::uint8_t& a)
{
  r = static_cast<std::uint8_t>((key >> 24) & 0xFFu);
  g = static_cast<std::uint8_t>((key >> 16) & 0xFFu);
  b = static_cast<std::uint8_t>((key >> 8) & 0xFFu);
  a = static_cast<std::uint8_t>(key & 0xFFu);
}

struct ColorCount {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t a = 255;
  std::uint32_t count = 0;
};

inline std::uint8_t Chan(const ColorCount& c, int chan)
{
  switch (chan) {
    default:
    case 0: return c.r;
    case 1: return c.g;
    case 2: return c.b;
    case 3: return c.a;
  }
}

struct Box {
  std::size_t begin = 0;
  std::size_t end = 0; // exclusive
  std::uint64_t weight = 0;

  std::uint8_t rmin = 255, rmax = 0;
  std::uint8_t gmin = 255, gmax = 0;
  std::uint8_t bmin = 255, bmax = 0;
  std::uint8_t amin = 255, amax = 0;
};

Box ComputeBox(const std::vector<ColorCount>& colors, std::size_t begin, std::size_t end)
{
  Box box;
  box.begin = begin;
  box.end = end;
  box.weight = 0;
  box.rmin = 255;
  box.gmin = 255;
  box.bmin = 255;
  box.amin = 255;
  box.rmax = 0;
  box.gmax = 0;
  box.bmax = 0;
  box.amax = 0;

  for (std::size_t i = begin; i < end; ++i) {
    const ColorCount& c = colors[i];
    box.weight += static_cast<std::uint64_t>(c.count);
    box.rmin = std::min(box.rmin, c.r);
    box.gmin = std::min(box.gmin, c.g);
    box.bmin = std::min(box.bmin, c.b);
    box.amin = std::min(box.amin, c.a);
    box.rmax = std::max(box.rmax, c.r);
    box.gmax = std::max(box.gmax, c.g);
    box.bmax = std::max(box.bmax, c.b);
    box.amax = std::max(box.amax, c.a);
  }
  return box;
}

inline int RangeU8(std::uint8_t mn, std::uint8_t mx) { return static_cast<int>(mx) - static_cast<int>(mn); }

int BestSplitChannel(const Box& b, bool includeAlpha)
{
  const int rr = RangeU8(b.rmin, b.rmax);
  const int gg = RangeU8(b.gmin, b.gmax);
  const int bb = RangeU8(b.bmin, b.bmax);
  const int aa = includeAlpha ? RangeU8(b.amin, b.amax) : 0;

  int bestChan = 0;
  int bestRange = rr;
  if (gg > bestRange) {
    bestRange = gg;
    bestChan = 1;
  }
  if (bb > bestRange) {
    bestRange = bb;
    bestChan = 2;
  }
  if (includeAlpha && aa > bestRange) {
    bestRange = aa;
    bestChan = 3;
  }
  return bestChan;
}

std::uint64_t SplitScore(const Box& b, bool includeAlpha)
{
  const int rr = RangeU8(b.rmin, b.rmax);
  const int gg = RangeU8(b.gmin, b.gmax);
  const int bb = RangeU8(b.bmin, b.bmax);
  const int aa = includeAlpha ? RangeU8(b.amin, b.amax) : 0;
  const int best = std::max(std::max(rr, gg), std::max(bb, aa));
  return static_cast<std::uint64_t>(best + 1) * b.weight;
}

bool BoxSplittable(const Box& b, bool includeAlpha)
{
  if (b.end <= b.begin + 1) return false;
  const int rr = RangeU8(b.rmin, b.rmax);
  const int gg = RangeU8(b.gmin, b.gmax);
  const int bb = RangeU8(b.bmin, b.bmax);
  const int aa = includeAlpha ? RangeU8(b.amin, b.amax) : 0;
  return (rr > 0) || (gg > 0) || (bb > 0) || (includeAlpha && aa > 0);
}

// Split a box in-place by sorting its range and partitioning at the weighted median.
bool SplitBox(std::vector<ColorCount>& colors, const Box& b, bool includeAlpha, Box& outA, Box& outB)
{
  if (!BoxSplittable(b, includeAlpha)) return false;

  const int chan = BestSplitChannel(b, includeAlpha);

  auto less = [&](const ColorCount& lhs, const ColorCount& rhs) {
    const std::uint8_t la = Chan(lhs, chan);
    const std::uint8_t ra = Chan(rhs, chan);
    if (la != ra) return la < ra;
    // Deterministic tie-breaker.
    const std::uint32_t lk = PackRGBA(lhs.r, lhs.g, lhs.b, lhs.a);
    const std::uint32_t rk = PackRGBA(rhs.r, rhs.g, rhs.b, rhs.a);
    if (lk != rk) return lk < rk;
    return lhs.count < rhs.count;
  };

  std::sort(colors.begin() + static_cast<std::ptrdiff_t>(b.begin),
            colors.begin() + static_cast<std::ptrdiff_t>(b.end), less);

  const std::uint64_t half = b.weight / 2u;
  std::uint64_t acc = 0;
  std::size_t split = b.begin + 1;

  for (std::size_t i = b.begin; i < b.end; ++i) {
    acc += static_cast<std::uint64_t>(colors[i].count);
    if (acc >= half) {
      split = i + 1;
      break;
    }
  }

  // Clamp to avoid empty boxes.
  if (split <= b.begin) split = b.begin + 1;
  if (split >= b.end) split = b.end - 1;

  outA = ComputeBox(colors, b.begin, split);
  outB = ComputeBox(colors, split, b.end);
  return true;
}

struct PaletteColor {
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t a = 255;
};

PaletteColor AverageBox(const std::vector<ColorCount>& colors, const Box& b)
{
  std::uint64_t sumR = 0;
  std::uint64_t sumG = 0;
  std::uint64_t sumB = 0;
  std::uint64_t sumA = 0;
  std::uint64_t sumW = 0;

  for (std::size_t i = b.begin; i < b.end; ++i) {
    const ColorCount& c = colors[i];
    const std::uint64_t w = static_cast<std::uint64_t>(c.count);
    sumW += w;
    sumR += w * static_cast<std::uint64_t>(c.r);
    sumG += w * static_cast<std::uint64_t>(c.g);
    sumB += w * static_cast<std::uint64_t>(c.b);
    sumA += w * static_cast<std::uint64_t>(c.a);
  }

  if (sumW == 0) return PaletteColor{0, 0, 0, 0};

  auto divRound = [&](std::uint64_t v) -> std::uint8_t {
    const std::uint64_t q = (v + (sumW / 2u)) / sumW;
    return static_cast<std::uint8_t>(std::min<std::uint64_t>(255u, q));
  };

  return PaletteColor{divRound(sumR), divRound(sumG), divRound(sumB), divRound(sumA)};
}

inline int ClampI(int v)
{
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

inline float ClampF(float v)
{
  if (v < 0.0f) return 0.0f;
  if (v > 255.0f) return 255.0f;
  return v;
}

std::uint8_t NearestPaletteIndex(const std::vector<PaletteColor>& pal, int r, int g, int b, int a, bool includeAlpha)
{
  std::uint32_t bestDist = std::numeric_limits<std::uint32_t>::max();
  std::uint8_t best = 0;

  // Index 0 is reserved for transparent.
  for (std::size_t i = 1; i < pal.size(); ++i) {
    const PaletteColor& p = pal[i];
    const int dr = r - static_cast<int>(p.r);
    const int dg = g - static_cast<int>(p.g);
    const int db = b - static_cast<int>(p.b);
    const int da = a - static_cast<int>(p.a);

    std::uint32_t dist = static_cast<std::uint32_t>(dr * dr + dg * dg + db * db);
    if (includeAlpha) dist += static_cast<std::uint32_t>(da * da);

    if (dist < bestDist) {
      bestDist = dist;
      best = static_cast<std::uint8_t>(i);
    }
  }

  return best;
}

} // namespace

bool QuantizeRgbaToIndexed(const RgbaImage& src, const GfxQuantizeConfig& cfg,
                           IndexedImage& out, std::string& outError)
{
  outError.clear();
  out = IndexedImage{};

  if (src.width <= 0 || src.height <= 0) {
    outError = "invalid image dimensions";
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 4u;
  if (src.rgba.size() != expected) {
    std::ostringstream oss;
    oss << "invalid RGBA buffer size (expected " << expected << ", got " << src.rgba.size() << ")";
    outError = oss.str();
    return false;
  }
  if (cfg.maxColors < 2 || cfg.maxColors > 256) {
    outError = "maxColors must be in [2, 256]";
    return false;
  }
  if (!std::isfinite(cfg.ditherStrength) || cfg.ditherStrength < 0.0f) {
    outError = "ditherStrength must be finite and >= 0";
    return false;
  }

  // Count unique non-transparent colors.
  std::unordered_map<std::uint32_t, std::uint32_t> counts;
  counts.reserve(std::min<std::size_t>(expected / 4u, 1u << 20));

  const std::size_t pxCount = static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height);
  for (std::size_t i = 0; i < pxCount; ++i) {
    const std::size_t si = i * 4u;
    const std::uint8_t a = src.rgba[si + 3u];
    if (a == 0u) continue;
    const std::uint8_t r = src.rgba[si + 0u];
    const std::uint8_t g = src.rgba[si + 1u];
    const std::uint8_t b = src.rgba[si + 2u];
    const std::uint32_t key = PackRGBA(r, g, b, a);
    auto it = counts.find(key);
    if (it == counts.end()) {
      counts.emplace(key, 1u);
    } else {
      // Saturate to avoid overflow.
      if (it->second != std::numeric_limits<std::uint32_t>::max()) it->second += 1u;
    }
  }

  std::vector<ColorCount> colors;
  colors.reserve(counts.size());
  for (const auto& kv : counts) {
    std::uint8_t r = 0, g = 0, b = 0, a = 0;
    UnpackRGBA(kv.first, r, g, b, a);
    colors.push_back(ColorCount{r, g, b, a, kv.second});
  }

  // Deterministic order for median-cut partitioning.
  std::sort(colors.begin(), colors.end(), [](const ColorCount& lhs, const ColorCount& rhs) {
    const std::uint32_t lk = PackRGBA(lhs.r, lhs.g, lhs.b, lhs.a);
    const std::uint32_t rk = PackRGBA(rhs.r, rhs.g, rhs.b, rhs.a);
    if (lk != rk) return lk < rk;
    return lhs.count < rhs.count;
  });

  // Reserve palette slot 0 for fully transparent.
  const int maxNonTransparent = cfg.maxColors - 1;

  std::vector<PaletteColor> pal;
  pal.reserve(static_cast<std::size_t>(cfg.maxColors));
  pal.push_back(PaletteColor{0, 0, 0, 0});

  if (colors.empty()) {
    // Entirely transparent image.
    out.width = src.width;
    out.height = src.height;
    out.indices.assign(pxCount, 0u);
    out.paletteRgba.assign({0u, 0u, 0u, 0u});
    return true;
  }

  if (static_cast<int>(colors.size()) <= maxNonTransparent) {
    // No quantization needed: keep exact colors.
    for (const ColorCount& c : colors) {
      pal.push_back(PaletteColor{c.r, c.g, c.b, c.a});
    }
  } else {
    // Weighted median-cut: split until we reach the desired number of boxes.
    std::vector<Box> boxes;
    boxes.reserve(static_cast<std::size_t>(maxNonTransparent));
    boxes.push_back(ComputeBox(colors, 0u, colors.size()));

    while (static_cast<int>(boxes.size()) < maxNonTransparent) {
      // Find best splittable box.
      std::size_t bestIdx = boxes.size();
      std::uint64_t bestScore = 0;
      for (std::size_t i = 0; i < boxes.size(); ++i) {
        const Box& b = boxes[i];
        if (!BoxSplittable(b, cfg.includeAlphaInDistance)) continue;
        const std::uint64_t score = SplitScore(b, cfg.includeAlphaInDistance);
        if (bestIdx == boxes.size() || score > bestScore) {
          bestIdx = i;
          bestScore = score;
        }
      }

      if (bestIdx == boxes.size()) break; // no more splittable boxes

      const Box toSplit = boxes[bestIdx];
      Box a;
      Box b;
      if (!SplitBox(colors, toSplit, cfg.includeAlphaInDistance, a, b)) break;

      boxes[bestIdx] = a;
      boxes.push_back(b);
    }

    // Convert boxes to palette entries.
    for (const Box& b : boxes) {
      const PaletteColor pc = AverageBox(colors, b);
      pal.push_back(pc);
      if (pal.size() >= static_cast<std::size_t>(cfg.maxColors)) break;
    }
  }

  if (pal.size() > static_cast<std::size_t>(cfg.maxColors)) {
    pal.resize(static_cast<std::size_t>(cfg.maxColors));
  }

  // Build palette RGBA byte buffer.
  std::vector<std::uint8_t> paletteRgba;
  paletteRgba.resize(pal.size() * 4u);
  for (std::size_t i = 0; i < pal.size(); ++i) {
    paletteRgba[i * 4u + 0u] = pal[i].r;
    paletteRgba[i * 4u + 1u] = pal[i].g;
    paletteRgba[i * 4u + 2u] = pal[i].b;
    paletteRgba[i * 4u + 3u] = pal[i].a;
  }

  std::vector<std::uint8_t> outIdx;
  outIdx.resize(pxCount);

  auto alphaAt = [&](int x, int y) -> std::uint8_t {
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 4u;
    return src.rgba[i + 3u];
  };

  if (!cfg.dither || cfg.ditherStrength == 0.0f) {
    // Map each unique color to nearest palette entry once.
    std::unordered_map<std::uint32_t, std::uint8_t> map;
    map.reserve(colors.size() + 16u);

    for (const ColorCount& c : colors) {
      const std::uint32_t key = PackRGBA(c.r, c.g, c.b, c.a);
      const std::uint8_t pi = NearestPaletteIndex(pal, static_cast<int>(c.r), static_cast<int>(c.g), static_cast<int>(c.b),
                                                 static_cast<int>(c.a), cfg.includeAlphaInDistance);
      map.emplace(key, pi);
    }

    for (std::size_t i = 0; i < pxCount; ++i) {
      const std::size_t si = i * 4u;
      const std::uint8_t a = src.rgba[si + 3u];
      if (a == 0u) {
        outIdx[i] = 0u;
        continue;
      }
      const std::uint8_t r = src.rgba[si + 0u];
      const std::uint8_t g = src.rgba[si + 1u];
      const std::uint8_t b = src.rgba[si + 2u];
      const std::uint32_t key = PackRGBA(r, g, b, a);

      auto it = map.find(key);
      if (it != map.end()) {
        outIdx[i] = it->second;
      } else {
        // Should not happen, but be defensive.
        outIdx[i] = NearestPaletteIndex(pal, static_cast<int>(r), static_cast<int>(g), static_cast<int>(b), static_cast<int>(a),
                                        cfg.includeAlphaInDistance);
      }
    }
  } else {
    // Floyd–Steinberg error diffusion (RGB only; alpha is not diffused).
    const int w = src.width;
    const int h = src.height;
    std::vector<float> errCurR(static_cast<std::size_t>(w) + 1u, 0.0f);
    std::vector<float> errCurG(static_cast<std::size_t>(w) + 1u, 0.0f);
    std::vector<float> errCurB(static_cast<std::size_t>(w) + 1u, 0.0f);
    std::vector<float> errNextR(static_cast<std::size_t>(w) + 1u, 0.0f);
    std::vector<float> errNextG(static_cast<std::size_t>(w) + 1u, 0.0f);
    std::vector<float> errNextB(static_cast<std::size_t>(w) + 1u, 0.0f);

    const float s = cfg.ditherStrength;

    for (int y = 0; y < h; ++y) {
      std::fill(errNextR.begin(), errNextR.end(), 0.0f);
      std::fill(errNextG.begin(), errNextG.end(), 0.0f);
      std::fill(errNextB.begin(), errNextB.end(), 0.0f);

      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const std::size_t si = i * 4u;
        const std::uint8_t a0 = src.rgba[si + 3u];
        if (a0 == 0u) {
          outIdx[i] = 0u;
          // Prevent error bleed through transparent pixels.
          errCurR[static_cast<std::size_t>(x)] = 0.0f;
          errCurG[static_cast<std::size_t>(x)] = 0.0f;
          errCurB[static_cast<std::size_t>(x)] = 0.0f;
          continue;
        }

        const float rr = ClampF(static_cast<float>(src.rgba[si + 0u]) + errCurR[static_cast<std::size_t>(x)]);
        const float gg = ClampF(static_cast<float>(src.rgba[si + 1u]) + errCurG[static_cast<std::size_t>(x)]);
        const float bb = ClampF(static_cast<float>(src.rgba[si + 2u]) + errCurB[static_cast<std::size_t>(x)]);

        const int r = ClampI(static_cast<int>(std::lround(rr)));
        const int g = ClampI(static_cast<int>(std::lround(gg)));
        const int b = ClampI(static_cast<int>(std::lround(bb)));
        const int a = static_cast<int>(a0);

        const std::uint8_t pi = NearestPaletteIndex(pal, r, g, b, a, cfg.includeAlphaInDistance);
        outIdx[i] = pi;

        const PaletteColor& pc = pal[pi];
        const float eR = (rr - static_cast<float>(pc.r)) * s;
        const float eG = (gg - static_cast<float>(pc.g)) * s;
        const float eB = (bb - static_cast<float>(pc.b)) * s;

        auto addCur = [&](int xx, float dr, float dg, float db) {
          if (xx < 0 || xx >= w) return;
          if (alphaAt(xx, y) == 0u) return;
          errCurR[static_cast<std::size_t>(xx)] += dr;
          errCurG[static_cast<std::size_t>(xx)] += dg;
          errCurB[static_cast<std::size_t>(xx)] += db;
        };
        auto addNext = [&](int xx, float dr, float dg, float db) {
          if (y + 1 >= h) return;
          if (xx < 0 || xx >= w) return;
          if (alphaAt(xx, y + 1) == 0u) return;
          errNextR[static_cast<std::size_t>(xx)] += dr;
          errNextG[static_cast<std::size_t>(xx)] += dg;
          errNextB[static_cast<std::size_t>(xx)] += db;
        };

        // Floyd–Steinberg kernel:
        //   right: 7/16
        //   down-left: 3/16
        //   down: 5/16
        //   down-right: 1/16
        addCur(x + 1, eR * (7.0f / 16.0f), eG * (7.0f / 16.0f), eB * (7.0f / 16.0f));
        addNext(x - 1, eR * (3.0f / 16.0f), eG * (3.0f / 16.0f), eB * (3.0f / 16.0f));
        addNext(x + 0, eR * (5.0f / 16.0f), eG * (5.0f / 16.0f), eB * (5.0f / 16.0f));
        addNext(x + 1, eR * (1.0f / 16.0f), eG * (1.0f / 16.0f), eB * (1.0f / 16.0f));
      }

      errCurR.swap(errNextR);
      errCurG.swap(errNextG);
      errCurB.swap(errNextB);
    }
  }

  out.width = src.width;
  out.height = src.height;
  out.indices = std::move(outIdx);
  out.paletteRgba = std::move(paletteRgba);
  return true;
}

} // namespace isocity
