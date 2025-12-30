#include "isocity/DistrictStats.hpp"

#include "isocity/Pathfinding.hpp"
#include "isocity/Road.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace isocity {

namespace {

inline int ClampDistrictId(int d)
{
  return std::clamp(d, 0, kDistrictCount - 1);
}

inline int HousingForLevel(int level)
{
  const int lv = std::max(1, level);
  return 10 * lv;
}

inline int JobsCommercialForLevel(int level)
{
  const int lv = std::max(1, level);
  return 8 * lv;
}

inline int JobsIndustrialForLevel(int level)
{
  const int lv = std::max(1, level);
  return 12 * lv;
}

inline bool HasZoneAccess(const World& world, const SimConfig& cfg, const std::vector<std::uint8_t>* edgeMask, int x, int y)
{
  if (!world.hasAdjacentRoad(x, y)) return false;
  if (!cfg.requireOutsideConnection) return true;
  if (!edgeMask || static_cast<int>(edgeMask->size()) != world.width() * world.height()) return false;
  return HasAdjacentRoadConnectedToEdge(world, *edgeMask, x, y);
}

} // namespace

DistrictStatsResult ComputeDistrictStats(const World& world, const SimConfig& cfg, const std::vector<float>* landValueField,
                                        const std::vector<std::uint8_t>* roadToEdgeMask)
{
  const int w = world.width();
  const int h = world.height();
  const int n = w * h;

  DistrictStatsResult result{};
  for (int d = 0; d < kDistrictCount; ++d) {
    result.districts[static_cast<std::size_t>(d)].id = d;
  }
  result.total.id = -1;

  // Land value field validity
  const bool lvOk = landValueField && (static_cast<int>(landValueField->size()) == n);

  // Outside connection mask (computed lazily if required)
  std::vector<std::uint8_t> computedMask;
  const std::vector<std::uint8_t>* edgeMask = nullptr;
  if (cfg.requireOutsideConnection) {
    if (roadToEdgeMask && static_cast<int>(roadToEdgeMask->size()) == n) {
      edgeMask = roadToEdgeMask;
    } else {
      ComputeRoadsConnectedToEdge(world, computedMask);
      edgeMask = &computedMask;
    }
  }

  std::array<double, static_cast<std::size_t>(kDistrictCount)> lvSum{};
  std::array<int, static_cast<std::size_t>(kDistrictCount)> lvCount{};

  const float roadMaintBase = static_cast<float>(std::max(0, cfg.maintenanceRoad));
  const float parkMaintBase = static_cast<float>(std::max(0, cfg.maintenancePark));

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = y * w + x;
      const Tile& t = world.at(x, y);
      const int d = ClampDistrictId(static_cast<int>(t.district));
      DistrictSummary& out = result.districts[static_cast<std::size_t>(d)];

      out.tiles += 1;
      if (t.terrain == Terrain::Water) {
        out.waterTiles += 1;
      } else {
        out.landTiles += 1;
        if (lvOk) {
          lvSum[static_cast<std::size_t>(d)] += (*landValueField)[static_cast<std::size_t>(idx)];
          lvCount[static_cast<std::size_t>(d)] += 1;
        }
      }

      const DistrictPolicy& pol = cfg.districtPolicies[static_cast<std::size_t>(d)];
      const bool policiesEnabled = cfg.districtPoliciesEnabled;

      // Roads / parks maintenance
      if (t.overlay == Overlay::Road) {
        out.roads += 1;
        const bool bridge = (t.terrain == Terrain::Water);
        const int units = bridge ? RoadBridgeMaintenanceUnitsForLevel(t.level) : RoadMaintenanceUnitsForLevel(t.level);
        const float mult = policiesEnabled ? std::max(0.0f, pol.roadMaintenanceMult) : 1.0f;
        const int cost = std::max(0, static_cast<int>(std::lround(static_cast<double>(units) * roadMaintBase * mult)));
        out.roadMaintenanceCost += cost;
        out.maintenanceCost += cost;
      } else if (t.overlay == Overlay::Park) {
        out.parks += 1;
        const float mult = policiesEnabled ? std::max(0.0f, pol.parkMaintenanceMult) : 1.0f;
        const int cost = std::max(0, static_cast<int>(std::lround(static_cast<double>(parkMaintBase) * mult)));
        out.parkMaintenanceCost += cost;
        out.maintenanceCost += cost;
      }

      // Zoning + tax
      const bool accessible = HasZoneAccess(world, cfg, edgeMask, x, y);
      if (t.overlay == Overlay::Residential) {
        out.resTiles += 1;
        out.zoneTiles += 1;
        out.population += t.occupants;
        out.housingCapacity += HousingForLevel(t.level);
        if (accessible) out.zoneTilesAccessible += 1;

        if (lvOk && t.occupants > 0) {
          const float lv = (*landValueField)[static_cast<std::size_t>(idx)];
          const float lvMult = 0.75f + 0.75f * lv;
          const float taxMult = policiesEnabled ? std::max(0.0f, pol.taxResidentialMult) : 1.0f;
          const double raw = static_cast<double>(t.occupants) * static_cast<double>(cfg.taxResidential) * static_cast<double>(lvMult)
                             * static_cast<double>(taxMult);
          out.taxRevenue += std::max(0, static_cast<int>(std::lround(raw)));
        }
      } else if (t.overlay == Overlay::Commercial) {
        out.comTiles += 1;
        out.zoneTiles += 1;
        out.employed += t.occupants;
        const int cap = JobsCommercialForLevel(t.level);
        out.jobsCapacity += cap;
        if (accessible) {
          out.zoneTilesAccessible += 1;
          out.jobsCapacityAccessible += cap;
        }
        if (lvOk && t.occupants > 0) {
          const float lv = (*landValueField)[static_cast<std::size_t>(idx)];
          const float lvMult = 0.75f + 0.75f * lv;
          const float taxMult = policiesEnabled ? std::max(0.0f, pol.taxCommercialMult) : 1.0f;
          const double raw = static_cast<double>(t.occupants) * static_cast<double>(cfg.taxCommercial) * static_cast<double>(lvMult)
                             * static_cast<double>(taxMult);
          out.taxRevenue += std::max(0, static_cast<int>(std::lround(raw)));
        }
      } else if (t.overlay == Overlay::Industrial) {
        out.indTiles += 1;
        out.zoneTiles += 1;
        out.employed += t.occupants;
        const int cap = JobsIndustrialForLevel(t.level);
        out.jobsCapacity += cap;
        if (accessible) {
          out.zoneTilesAccessible += 1;
          out.jobsCapacityAccessible += cap;
        }
        if (lvOk && t.occupants > 0) {
          const float lv = (*landValueField)[static_cast<std::size_t>(idx)];
          const float lvMult = 0.75f + 0.75f * lv;
          const float taxMult = policiesEnabled ? std::max(0.0f, pol.taxIndustrialMult) : 1.0f;
          const double raw = static_cast<double>(t.occupants) * static_cast<double>(cfg.taxIndustrial) * static_cast<double>(lvMult)
                             * static_cast<double>(taxMult);
          out.taxRevenue += std::max(0, static_cast<int>(std::lround(raw)));
        }
      }
    }
  }

  double totalLvSum = 0.0;
  int totalLvCount = 0;

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictSummary& out = result.districts[static_cast<std::size_t>(d)];

    const int count = lvCount[static_cast<std::size_t>(d)];
    if (lvOk && count > 0) {
      out.avgLandValue = static_cast<float>(lvSum[static_cast<std::size_t>(d)] / static_cast<double>(count));
      totalLvSum += lvSum[static_cast<std::size_t>(d)];
      totalLvCount += count;
    } else {
      out.avgLandValue = 0.0f;
    }

    out.net = out.taxRevenue - out.maintenanceCost;

    // Aggregate totals
    result.total.tiles += out.tiles;
    result.total.landTiles += out.landTiles;
    result.total.waterTiles += out.waterTiles;
    result.total.roads += out.roads;
    result.total.parks += out.parks;
    result.total.resTiles += out.resTiles;
    result.total.comTiles += out.comTiles;
    result.total.indTiles += out.indTiles;
    result.total.zoneTiles += out.zoneTiles;
    result.total.zoneTilesAccessible += out.zoneTilesAccessible;
    result.total.population += out.population;
    result.total.housingCapacity += out.housingCapacity;
    result.total.jobsCapacity += out.jobsCapacity;
    result.total.jobsCapacityAccessible += out.jobsCapacityAccessible;
    result.total.employed += out.employed;
    result.total.taxRevenue += out.taxRevenue;
    result.total.roadMaintenanceCost += out.roadMaintenanceCost;
    result.total.parkMaintenanceCost += out.parkMaintenanceCost;
    result.total.maintenanceCost += out.maintenanceCost;
  }

  if (lvOk && totalLvCount > 0) {
    result.total.avgLandValue = static_cast<float>(totalLvSum / static_cast<double>(totalLvCount));
  } else {
    result.total.avgLandValue = 0.0f;
  }
  result.total.net = result.total.taxRevenue - result.total.maintenanceCost;

  return result;
}

} // namespace isocity
