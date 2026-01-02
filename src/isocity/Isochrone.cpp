#include "isocity/Isochrone.hpp"

#include "isocity/FlowField.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t expect = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == expect;
}

inline bool IsZoneOverlay(Overlay o)
{
  return (o == Overlay::Residential) || (o == Overlay::Commercial) || (o == Overlay::Industrial);
}

} // namespace

RoadIsochroneField BuildRoadIsochroneField(const World& world,
                                           const std::vector<int>& sourceRoadIdx,
                                           const RoadIsochroneConfig& cfg,
                                           const std::vector<std::uint8_t>* precomputedRoadToEdge,
                                           const std::vector<int>* extraCostMilli)
{
  RoadIsochroneField out;
  out.w = world.width();
  out.h = world.height();

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  RoadFlowFieldConfig fcfg;
  fcfg.requireOutsideConnection = cfg.requireOutsideConnection;
  fcfg.useTravelTime = (cfg.weightMode == IsochroneWeightMode::TravelTime);
  fcfg.computeOwner = cfg.computeOwner;

  // BuildRoadFlowField already leaves -1 for non-road tiles.
  const RoadFlowField field = BuildRoadFlowField(world, sourceRoadIdx, fcfg, precomputedRoadToEdge, extraCostMilli);

  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  out.costMilli = field.cost;
  out.steps = field.dist;
  if (cfg.computeOwner) {
    out.owner = field.owner;
  }

  // Defensive: ensure consistent sizing.
  if (out.costMilli.size() != n) out.costMilli.assign(n, -1);
  if (out.steps.size() != n) out.steps.assign(n, -1);
  if (cfg.computeOwner && out.owner.size() != n) out.owner.assign(n, -1);

  return out;
}

std::vector<int> BuildTileAccessCostField(const World& world,
                                         const RoadIsochroneField& roadField,
                                         const TileAccessCostConfig& cfg,
                                         const std::vector<std::uint8_t>* roadToEdgeMask)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  std::vector<int> out(n, -1);

  if (w <= 0 || h <= 0) return out;
  if (roadField.w != w || roadField.h != h) return out;
  if (roadField.costMilli.size() != n) return out;

  // Optional zone access map (supports interior zoning tiles).
  ZoneAccessMap zam;
  bool haveZam = false;
  if (cfg.includeZones && cfg.useZoneAccessMap) {
    zam = BuildZoneAccessMap(world, MaskUsable(roadToEdgeMask, w, h) ? roadToEdgeMask : nullptr);
    haveZam = (zam.w == w && zam.h == h && zam.roadIdx.size() == n);
  }

  const int walkCost = std::max(0, cfg.accessStepCostMilli);
  constexpr int kDirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

  auto bestAdjacentRoadCost = [&](int x, int y) -> int {
    int best = std::numeric_limits<int>::max();
    bool found = false;
    for (const auto& d : kDirs) {
      const int nx = x + d[0];
      const int ny = y + d[1];
      if (!world.inBounds(nx, ny)) continue;
      const Tile& nt = world.at(nx, ny);
      if (nt.overlay != Overlay::Road) continue;
      const int nidx = ny * w + nx;
      const int c = roadField.costMilli[static_cast<std::size_t>(nidx)];
      if (c < 0) continue;
      best = std::min(best, c);
      found = true;
    }
    return found ? best : -1;
  };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      if (t.terrain == Terrain::Water && !cfg.includeWater) {
        out[idx] = -1;
        continue;
      }

      // Road tiles are direct.
      if (cfg.includeRoadTiles && t.overlay == Overlay::Road) {
        out[idx] = roadField.costMilli[idx];
        continue;
      }

      int mappedRoadCost = -1;

      // Zones can use ZoneAccessMap for interior parcels.
      if (cfg.includeZones && IsZoneOverlay(t.overlay)) {
        if (haveZam) {
          const int ridx = zam.roadIdx[idx];
          if (ridx >= 0 && static_cast<std::size_t>(ridx) < n) {
            mappedRoadCost = roadField.costMilli[static_cast<std::size_t>(ridx)];
          }
        }
        if (mappedRoadCost < 0 && cfg.includeNonZonesAdjacentToRoad) {
          mappedRoadCost = bestAdjacentRoadCost(x, y);
        }
      } else if (cfg.includeNonZonesAdjacentToRoad) {
        // Non-zone tiles can optionally be "served" by adjacent roads.
        mappedRoadCost = bestAdjacentRoadCost(x, y);
      }

      if (mappedRoadCost >= 0) {
        // Walking from the road onto the parcel.
        const long long sum = static_cast<long long>(mappedRoadCost) + static_cast<long long>(walkCost);
        const long long clampMax = static_cast<long long>(std::numeric_limits<int>::max());
        out[idx] = static_cast<int>(std::min(sum, clampMax));
      } else {
        out[idx] = -1;
      }
    }
  }

  return out;
}

} // namespace isocity
