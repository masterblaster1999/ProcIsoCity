#include "isocity/ZoneParcels.hpp"

#include "isocity/Random.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace isocity {

namespace {

struct Shape {
  int w = 1;
  int h = 1;
};

inline int Area(const Shape& s) { return s.w * s.h; }

inline void AddUnique(std::array<Shape, 16>& out, int& count, int w, int h)
{
  if (count <= 0) {
    out[0] = Shape{w, h};
    count = 1;
    return;
  }

  for (int i = 0; i < count; ++i) {
    if (out[static_cast<std::size_t>(i)].w == w && out[static_cast<std::size_t>(i)].h == h) {
      return;
    }
  }

  if (count < static_cast<int>(out.size())) {
    out[static_cast<std::size_t>(count++)] = Shape{w, h};
  }
}

inline void AddRect(std::array<Shape, 16>& out, int& count, int w, int h, bool flip)
{
  if (w == h) {
    AddUnique(out, count, w, h);
    return;
  }

  // Deterministic but varied orientation ordering.
  if (!flip) {
    AddUnique(out, count, w, h);
    AddUnique(out, count, h, w);
  } else {
    AddUnique(out, count, h, w);
    AddUnique(out, count, w, h);
  }
}

inline bool CanPlaceRect(const World& world, const ZoneBuildingParcels& out, int x0, int y0, int wRect, int hRect,
                         Overlay o, std::uint8_t level)
{
  const int w = out.width;
  const int h = out.height;

  if (x0 < 0 || y0 < 0) return false;
  if (x0 + wRect > w || y0 + hRect > h) return false;

  for (int y = y0; y < y0 + hRect; ++y) {
    for (int x = x0; x < x0 + wRect; ++x) {
      const int idx = y * w + x;
      if (out.tileToParcel[static_cast<std::size_t>(idx)] != -1) return false;

      const Tile& t = world.at(x, y);
      if (!IsZoneOverlay(t.overlay)) return false;
      if (t.overlay != o) return false;
      if (t.level != level) return false;

      // Defensive: don't attempt to parcelize impossible terrain (zones shouldn't
      // exist on water, but editor/testing can create them).
      if (t.terrain == Terrain::Water) return false;
    }
  }

  return true;
}

inline std::uint8_t ComputeFacing(const World& world, int x0, int y0, int wRect, int hRect, std::uint32_t h0)
{
  const int x1 = x0 + wRect - 1;
  const int y1 = y0 + hRect - 1;

  int counts[4] = {0, 0, 0, 0};
  // 0=N, 1=E, 2=S, 3=W

  // North/South edges.
  for (int x = x0; x <= x1; ++x) {
    if (world.inBounds(x, y0 - 1) && world.at(x, y0 - 1).overlay == Overlay::Road) counts[0]++;
    if (world.inBounds(x, y1 + 1) && world.at(x, y1 + 1).overlay == Overlay::Road) counts[2]++;
  }

  // West/East edges.
  for (int y = y0; y <= y1; ++y) {
    if (world.inBounds(x0 - 1, y) && world.at(x0 - 1, y).overlay == Overlay::Road) counts[3]++;
    if (world.inBounds(x1 + 1, y) && world.at(x1 + 1, y).overlay == Overlay::Road) counts[1]++;
  }

  int best = counts[0];
  std::array<int, 4> bestDirs{0, 0, 0, 0};
  int bestCount = 1;

  for (int d = 1; d < 4; ++d) {
    if (counts[d] > best) {
      best = counts[d];
      bestDirs[0] = d;
      bestCount = 1;
    } else if (counts[d] == best) {
      bestDirs[static_cast<std::size_t>(bestCount++)] = d;
    }
  }

  const int pick = (bestCount > 0) ? static_cast<int>((h0 >> 8) % static_cast<std::uint32_t>(bestCount)) : 0;
  const int dir = bestDirs[static_cast<std::size_t>(std::clamp(pick, 0, bestCount - 1))];
  return static_cast<std::uint8_t>(dir & 3);
}

// Produce an ordered list of candidate rectangle footprints for this tile.
// The first entry represents the preferred ("desired") size; subsequent entries
// are fallbacks, generally ordered from larger to smaller.
inline void BuildCandidateShapes(const Tile& t, std::uint32_t h0, std::array<Shape, 16>& out, int& outCount)
{
  outCount = 0;

  if (!IsZoneOverlay(t.overlay)) {
    AddUnique(out, outCount, 1, 1);
    return;
  }

  const int lvl = ClampZoneLevel(t.level);
  if (lvl <= 1) {
    AddUnique(out, outCount, 1, 1);
    return;
  }

  const float r = static_cast<float>(h0 & 0xFFFFu) / 65535.0f;
  const bool flip = ((h0 >> 16) & 1u) != 0u;

  auto addFallback = [&](std::initializer_list<Shape> shapes) {
    // Keep deterministic fallback ordering.
    for (const Shape& s : shapes) {
      AddRect(out, outCount, s.w, s.h, flip);
    }
  };

  if (t.overlay == Overlay::Residential) {
    if (lvl == 2) {
      if (r < 0.30f) {
        AddRect(out, outCount, 2, 2, flip);
      } else if (r < 0.55f) {
        AddRect(out, outCount, 2, 1, flip);
      } else {
        AddUnique(out, outCount, 1, 1);
      }

      addFallback({Shape{2, 2}, Shape{2, 1}, Shape{1, 1}});
      return;
    }

    // lvl >= 3
    if (r < 0.18f) {
      AddRect(out, outCount, 3, 2, flip);
    } else if (r < 0.38f) {
      AddRect(out, outCount, 2, 2, flip);
    } else if (r < 0.52f) {
      AddRect(out, outCount, 3, 1, flip);
    } else if (r < 0.70f) {
      AddRect(out, outCount, 2, 1, flip);
    } else {
      AddUnique(out, outCount, 1, 1);
    }

    addFallback({Shape{3, 2}, Shape{2, 2}, Shape{3, 1}, Shape{2, 1}, Shape{1, 1}});
    return;
  }

  if (t.overlay == Overlay::Commercial) {
    if (lvl == 2) {
      if (r < 0.20f) {
        AddRect(out, outCount, 2, 2, flip);
      } else if (r < 0.36f) {
        AddRect(out, outCount, 3, 1, flip);
      } else if (r < 0.55f) {
        AddRect(out, outCount, 2, 1, flip);
      } else {
        AddUnique(out, outCount, 1, 1);
      }

      addFallback({Shape{2, 2}, Shape{3, 1}, Shape{2, 1}, Shape{1, 1}});
      return;
    }

    // lvl >= 3
    if (r < 0.10f) {
      AddRect(out, outCount, 3, 3, flip);
    } else if (r < 0.24f) {
      AddRect(out, outCount, 4, 2, flip);
    } else if (r < 0.40f) {
      AddRect(out, outCount, 3, 2, flip);
    } else if (r < 0.52f) {
      AddRect(out, outCount, 4, 1, flip);
    } else if (r < 0.66f) {
      AddRect(out, outCount, 2, 2, flip);
    } else if (r < 0.78f) {
      AddRect(out, outCount, 3, 1, flip);
    } else if (r < 0.90f) {
      AddRect(out, outCount, 2, 1, flip);
    } else {
      AddUnique(out, outCount, 1, 1);
    }

    addFallback({Shape{3, 3}, Shape{4, 2}, Shape{3, 2}, Shape{4, 1}, Shape{2, 2}, Shape{3, 1}, Shape{2, 1},
                 Shape{1, 1}});
    return;
  }

  // Industrial
  if (lvl == 2) {
    if (r < 0.22f) {
      AddRect(out, outCount, 3, 1, flip);
    } else if (r < 0.40f) {
      AddRect(out, outCount, 2, 2, flip);
    } else if (r < 0.60f) {
      AddRect(out, outCount, 2, 1, flip);
    } else {
      AddUnique(out, outCount, 1, 1);
    }

    addFallback({Shape{3, 1}, Shape{2, 2}, Shape{2, 1}, Shape{1, 1}});
    return;
  }

  // lvl >= 3
  if (r < 0.16f) {
    AddRect(out, outCount, 4, 2, flip);
  } else if (r < 0.32f) {
    AddRect(out, outCount, 4, 1, flip);
  } else if (r < 0.48f) {
    AddRect(out, outCount, 3, 2, flip);
  } else if (r < 0.62f) {
    AddRect(out, outCount, 2, 2, flip);
  } else if (r < 0.78f) {
    AddRect(out, outCount, 3, 1, flip);
  } else if (r < 0.90f) {
    AddRect(out, outCount, 2, 1, flip);
  } else {
    AddUnique(out, outCount, 1, 1);
  }

  addFallback({Shape{4, 2}, Shape{4, 1}, Shape{3, 2}, Shape{2, 2}, Shape{3, 1}, Shape{2, 1}, Shape{1, 1}});
}

} // namespace

void BuildZoneBuildingParcels(const World& world, ZoneBuildingParcels& out)
{
  out.width = world.width();
  out.height = world.height();

  const int w = out.width;
  const int h = out.height;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.parcels.clear();
  out.tileToParcel.assign(n, -1);
  out.anchorToParcel.assign(n, -1);

  const std::uint32_t seedLo = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu);
  const std::uint32_t seedHi = static_cast<std::uint32_t>((world.seed() >> 32) & 0xFFFFFFFFu);

  std::array<Shape, 16> candidates{};
  int candCount = 0;

  for (int y0 = 0; y0 < h; ++y0) {
    for (int x0 = 0; x0 < w; ++x0) {
      const int idx0 = y0 * w + x0;
      if (out.tileToParcel[static_cast<std::size_t>(idx0)] != -1) continue;

      const Tile& t0 = world.at(x0, y0);
      if (!IsZoneOverlay(t0.overlay)) continue;
      if (t0.terrain == Terrain::Water) continue;

      // Mix overlay/level into the seed so different zone types don't line up into
      // identical parcel patterns.
      const std::uint32_t salt = (static_cast<std::uint32_t>(t0.overlay) << 24) ^
                                 (static_cast<std::uint32_t>(t0.level) << 16);

      const std::uint32_t h0 = HashCoords32(x0, y0, seedLo ^ seedHi ^ salt);

      BuildCandidateShapes(t0, h0, candidates, candCount);

      int chosenW = 1;
      int chosenH = 1;

      for (int ci = 0; ci < candCount; ++ci) {
        const Shape s = candidates[static_cast<std::size_t>(ci)];
        if (s.w <= 0 || s.h <= 0) continue;
        if (CanPlaceRect(world, out, x0, y0, s.w, s.h, t0.overlay, t0.level)) {
          chosenW = s.w;
          chosenH = s.h;
          break;
        }
      }

      ZoneBuildingParcel p;
      p.x0 = x0;
      p.y0 = y0;
      p.w = chosenW;
      p.h = chosenH;
      p.overlay = t0.overlay;
      p.level = t0.level;
      p.styleSeed = h0;
      p.facing = ComputeFacing(world, x0, y0, chosenW, chosenH, h0);

      // Aggregate metrics.
      for (int yy = y0; yy < y0 + chosenH; ++yy) {
        for (int xx = x0; xx < x0 + chosenW; ++xx) {
          const Tile& t = world.at(xx, yy);
          p.occupants += static_cast<int>(t.occupants);
          p.capacity += CapacityForOverlayLevel(t.overlay, static_cast<int>(t.level));
        }
      }

      const int parcelIndex = static_cast<int>(out.parcels.size());
      out.parcels.push_back(p);

      for (int yy = y0; yy < y0 + chosenH; ++yy) {
        for (int xx = x0; xx < x0 + chosenW; ++xx) {
          out.tileToParcel[static_cast<std::size_t>(yy * w + xx)] = parcelIndex;
        }
      }

      const int anchorX = x0 + chosenW - 1;
      const int anchorY = y0 + chosenH - 1;
      out.anchorToParcel[static_cast<std::size_t>(anchorY * w + anchorX)] = parcelIndex;
    }
  }
}

} // namespace isocity
