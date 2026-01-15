#include "isocity/WorldTiling.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>

namespace {

using isocity::Blueprint;
using isocity::BlueprintTransform;
using isocity::Overlay;
using isocity::TileFieldMask;

inline bool MaskHas(std::uint8_t mask, TileFieldMask bit)
{
  return (mask & static_cast<std::uint8_t>(bit)) != 0;
}

// 64-bit FNV-1a hash (small helper for deterministic signatures).
inline std::uint64_t Fnv1a64(const std::uint8_t* data, std::size_t len, std::uint64_t h = 14695981039346656037ull)
{
  constexpr std::uint64_t kPrime = 1099511628211ull;
  for (std::size_t i = 0; i < len; ++i) {
    h ^= static_cast<std::uint64_t>(data[i]);
    h *= kPrime;
  }
  return h;
}

inline std::uint64_t HashEdgeBits(const std::vector<std::uint8_t>& bits, std::uint32_t salt)
{
  // Include length + salt so e.g. "001" != "0001" and edges don't collide across sides.
  std::uint64_t h = 14695981039346656037ull;
  const std::uint64_t len64 = static_cast<std::uint64_t>(bits.size());
  h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&len64), sizeof(len64), h);
  h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&salt), sizeof(salt), h);
  h = Fnv1a64(bits.data(), bits.size(), h);
  return h;
}

static void BuildOverlayGrid(const Blueprint& bp, std::vector<Overlay>& out)
{
  const int w = std::max(0, bp.width);
  const int h = std::max(0, bp.height);
  out.assign(static_cast<std::size_t>(w * h), Overlay::None);

  for (const auto& d : bp.tiles) {
    const std::size_t idx = static_cast<std::size_t>(d.index);
    if (idx >= out.size()) continue;
    if (!MaskHas(d.mask, TileFieldMask::Overlay)) continue;
    out[idx] = d.value.overlay;
  }
}

static void ComputeRoadEdgeSigs(const Blueprint& bp, std::uint64_t& outN, std::uint64_t& outE,
                                std::uint64_t& outS, std::uint64_t& outW)
{
  const int w = std::max(0, bp.width);
  const int h = std::max(0, bp.height);
  std::vector<Overlay> grid;
  BuildOverlayGrid(bp, grid);

  std::vector<std::uint8_t> bits;

  // North (top row)
  bits.assign(static_cast<std::size_t>(w), 0);
  for (int x = 0; x < w; ++x) {
    const Overlay o = grid[static_cast<std::size_t>(x)];
    bits[static_cast<std::size_t>(x)] = (o == Overlay::Road) ? 1u : 0u;
  }
  outN = HashEdgeBits(bits, 0x4E /*'N'*/);

  // South (bottom row)
  bits.assign(static_cast<std::size_t>(w), 0);
  if (h > 0) {
    const int y = h - 1;
    for (int x = 0; x < w; ++x) {
      const Overlay o = grid[static_cast<std::size_t>(y * w + x)];
      bits[static_cast<std::size_t>(x)] = (o == Overlay::Road) ? 1u : 0u;
    }
  }
  outS = HashEdgeBits(bits, 0x53 /*'S'*/);

  // West (left col)
  bits.assign(static_cast<std::size_t>(h), 0);
  for (int y = 0; y < h; ++y) {
    const Overlay o = grid[static_cast<std::size_t>(y * w + 0)];
    bits[static_cast<std::size_t>(y)] = (o == Overlay::Road) ? 1u : 0u;
  }
  outW = HashEdgeBits(bits, 0x57 /*'W'*/);

  // East (right col)
  bits.assign(static_cast<std::size_t>(h), 0);
  if (w > 0) {
    const int x = w - 1;
    for (int y = 0; y < h; ++y) {
      const Overlay o = grid[static_cast<std::size_t>(y * w + x)];
      bits[static_cast<std::size_t>(y)] = (o == Overlay::Road) ? 1u : 0u;
    }
  }
  outE = HashEdgeBits(bits, 0x45 /*'E'*/);
}

static std::uint64_t HashBlueprintContent(const Blueprint& bp)
{
  std::uint64_t h = 14695981039346656037ull;
  const std::uint64_t w = static_cast<std::uint64_t>(bp.width);
  const std::uint64_t ht = static_cast<std::uint64_t>(bp.height);
  const std::uint64_t n = static_cast<std::uint64_t>(bp.tiles.size());
  h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&w), sizeof(w), h);
  h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&ht), sizeof(ht), h);
  h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&n), sizeof(n), h);

  for (const auto& d : bp.tiles) {
    h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&d.index), sizeof(d.index), h);
    h = Fnv1a64(reinterpret_cast<const std::uint8_t*>(&d.mask), sizeof(d.mask), h);
    // Hash all tile fields deterministically (avoid hashing raw struct bytes).
    const std::uint8_t terrain = static_cast<std::uint8_t>(d.value.terrain);
    const std::uint8_t overlay = static_cast<std::uint8_t>(d.value.overlay);
    const std::uint8_t height = static_cast<std::uint8_t>(d.value.height);
    const std::uint8_t variation = d.value.variation;
    const std::uint8_t level = d.value.level;
    const std::uint16_t occupants = d.value.occupants;
    const std::uint8_t district = d.value.district;
    h = Fnv1a64(&terrain, sizeof(terrain), h);
    h = Fnv1a64(&overlay, sizeof(overlay), h);
    h = Fnv1a64(&height, sizeof(height), h);
    h = Fnv1a64(&variation, sizeof(variation), h);
    h = Fnv1a64(&level, sizeof(level), h);
    h = Fnv1a64(&occupants, sizeof(occupants), h);
    h = Fnv1a64(&district, sizeof(district), h);
  }

  return h;
}

static std::vector<BlueprintTransform> CandidateTransforms(const Blueprint& src, int cellW, int cellH)
{
  std::vector<BlueprintTransform> out;
  out.reserve(8);

  auto add = [&](int rot, bool mx, bool my) {
    BlueprintTransform t;
    t.rotateDeg = rot;
    t.mirrorX = mx;
    t.mirrorY = my;

    // Quick dimension filter to avoid doing expensive transforms.
    int w = src.width;
    int h = src.height;
    if (rot == 90 || rot == 270) std::swap(w, h);
    if (w != cellW || h != cellH) return;
    out.push_back(t);
  };

  // Dihedral-ish subset: rotations + mirrors.
  const int rots[4] = {0, 90, 180, 270};
  for (int r : rots) {
    // Mirror combos.
    add(r, false, false);
    add(r, true, false);
    add(r, false, true);
    add(r, true, true);
  }

  // De-dup exact transform triples.
  auto key = [](const BlueprintTransform& t) -> std::uint32_t {
    return static_cast<std::uint32_t>((t.rotateDeg & 0x3FF) | (t.mirrorX ? 1u << 12 : 0u) |
                                      (t.mirrorY ? 1u << 13 : 0u));
  };
  std::unordered_set<std::uint32_t> seen;
  std::vector<BlueprintTransform> uniq;
  uniq.reserve(out.size());
  for (const auto& t : out) {
    const std::uint32_t k = key(t);
    if (seen.insert(k).second) uniq.push_back(t);
  }
  return uniq;
}

} // namespace

namespace isocity {

bool BuildBlueprintTileset(const std::vector<std::pair<std::string, Blueprint>>& sources, int cellW, int cellH,
                           bool generateTransforms, BlueprintTileset& outTileset, std::string& outError)
{
  outTileset = BlueprintTileset{};
  outTileset.cellW = cellW;
  outTileset.cellH = cellH;

  if (cellW <= 0 || cellH <= 0) {
    outError = "Invalid cell size";
    return false;
  }

  if (sources.empty()) {
    outError = "No blueprint sources";
    return false;
  }

  // De-duplicate exact duplicates (same transformed blueprint content). We intentionally
  // do NOT de-dup by edge signatures alone, because multiple tiles can share the same
  // edge patterns but have different interiors (which we want for variety).
  std::unordered_set<std::uint64_t> seen;

  for (const auto& srcPair : sources) {
    const std::string& name = srcPair.first;
    const Blueprint& src = srcPair.second;
    if (src.width <= 0 || src.height <= 0) continue;

    auto emitVariant = [&](const Blueprint& bp, const BlueprintTransform& tr, const std::string& suffix) {
      if (bp.width != cellW || bp.height != cellH) return;
      BlueprintTileVariant v;
      v.name = suffix.empty() ? name : (name + suffix);
      v.transform = tr;
      v.bp = bp;
      ComputeRoadEdgeSigs(v.bp, v.edgeN, v.edgeE, v.edgeS, v.edgeW);

      const std::uint64_t contentHash = HashBlueprintContent(v.bp);
      if (!seen.insert(contentHash).second) return;

      outTileset.variants.push_back(std::move(v));
    };

    if (!generateTransforms) {
      if (src.width == cellW && src.height == cellH) {
        emitVariant(src, BlueprintTransform{}, "");
      }
      continue;
    }

    const std::vector<BlueprintTransform> trs = CandidateTransforms(src, cellW, cellH);
    for (const BlueprintTransform& tr : trs) {
      Blueprint tmp;
      std::string err;
      if (!TransformBlueprint(src, tr, tmp, err)) {
        // Ignore broken transforms; keep going.
        continue;
      }
      if (tmp.width != cellW || tmp.height != cellH) continue;

      const std::string suffix = " [r" + std::to_string(tr.rotateDeg) + " mx" +
                                 std::to_string(tr.mirrorX ? 1 : 0) + " my" +
                                 std::to_string(tr.mirrorY ? 1 : 0) + "]";
      emitVariant(tmp, tr, suffix);
    }
  }

  if (outTileset.variants.empty()) {
    outError = "No tile variants matched " + std::to_string(cellW) + "x" + std::to_string(cellH);
    return false;
  }

  return true;
}

bool SolveBlueprintTiling(const BlueprintTileset& tileset, int cellsX, int cellsY, const BlueprintTilingConfig& cfg,
                          BlueprintTilingSolution& outSolution, std::string& outError)
{
  outSolution = BlueprintTilingSolution{};
  outSolution.cellsX = cellsX;
  outSolution.cellsY = cellsY;
  outSolution.chosen.assign(static_cast<std::size_t>(std::max(0, cellsX) * std::max(0, cellsY)), -1);

  if (cellsX <= 0 || cellsY <= 0) {
    outError = "Invalid grid size";
    return false;
  }
  if (tileset.variants.empty()) {
    outError = "Tileset is empty";
    return false;
  }

  const auto& vars = tileset.variants;

  auto collectCandidates = [&](bool matchW, std::uint64_t reqW, bool matchN, std::uint64_t reqN,
                               std::vector<int>& out) {
    out.clear();
    out.reserve(vars.size());
    for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
      const auto& v = vars[static_cast<std::size_t>(i)];
      if (matchW && v.edgeW != reqW) continue;
      if (matchN && v.edgeN != reqN) continue;
      out.push_back(i);
    }
  };

  std::vector<int> candidates;

  for (int y = 0; y < cellsY; ++y) {
    for (int x = 0; x < cellsX; ++x) {
      bool needW = false;
      bool needN = false;
      std::uint64_t reqW = 0;
      std::uint64_t reqN = 0;

      if (cfg.matchRoadEdges) {
        if (x > 0) {
          const int li = outSolution.chosen[static_cast<std::size_t>(y * cellsX + (x - 1))];
          if (li >= 0) {
            needW = true;
            reqW = vars[static_cast<std::size_t>(li)].edgeE;
          }
        }
        if (y > 0) {
          const int ui = outSolution.chosen[static_cast<std::size_t>((y - 1) * cellsX + x)];
          if (ui >= 0) {
            needN = true;
            reqN = vars[static_cast<std::size_t>(ui)].edgeS;
          }
        }
      }

      collectCandidates(needW, reqW, needN, reqN, candidates);

      if (candidates.empty() && cfg.matchRoadEdges && cfg.allowFallback) {
        // Relax constraints progressively.
        outSolution.fallbacks++;
        if (needW && needN) {
          collectCandidates(true, reqW, false, 0, candidates);
          if (candidates.empty()) collectCandidates(false, 0, true, reqN, candidates);
          if (candidates.empty()) collectCandidates(false, 0, false, 0, candidates);
        } else {
          collectCandidates(false, 0, false, 0, candidates);
        }
      }

      if (candidates.empty()) {
        outSolution.failures++;
        continue;
      }

      const std::uint32_t h = HashCoords32(x, y, cfg.seed ^ 0xC3A5C85Cu);
      const int pick = candidates[static_cast<std::size_t>(h % static_cast<std::uint32_t>(candidates.size()))];
      outSolution.chosen[static_cast<std::size_t>(y * cellsX + x)] = pick;
    }
  }

  return true;
}

} // namespace isocity
