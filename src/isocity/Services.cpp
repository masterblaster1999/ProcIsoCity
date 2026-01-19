#include "isocity/Services.hpp"

#include "isocity/Isochrone.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ZoneAccess.hpp"

#include <algorithm>
#include <cmath>
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

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline int ClampLevel(int lvl) { return std::clamp(lvl, 1, 3); }

inline int ServiceIdx(ServiceType t)
{
  return static_cast<int>(t);
}

float DistanceWeight(const ServicesModelSettings& cfg, int costMilli, int radiusMilli)
{
  if (radiusMilli <= 0) return 0.0f;
  if (costMilli < 0 || costMilli > radiusMilli) return 0.0f;

  const float frac = static_cast<float>(costMilli) / static_cast<float>(radiusMilli);
  const float cut0 = std::clamp(cfg.distanceBandCutFrac[0], 0.0f, 1.0f);
  const float cut1 = std::clamp(cfg.distanceBandCutFrac[1], cut0, 1.0f);

  if (frac <= cut0) return cfg.distanceBandWeight[0];
  if (frac <= cut1) return cfg.distanceBandWeight[1];
  return cfg.distanceBandWeight[2];
}

float BaseDemandForTile(const Tile& t, const ServicesModelSettings& cfg)
{
  bool ok = false;
  if (t.overlay == Overlay::Residential) ok = cfg.demandResidential;
  else if (t.overlay == Overlay::Commercial) ok = cfg.demandCommercial;
  else if (t.overlay == Overlay::Industrial) ok = cfg.demandIndustrial;

  if (!ok) return 0.0f;

  if (cfg.demandMode == ServiceDemandMode::Tiles) return 1.0f;
  return static_cast<float>(t.occupants);
}

float DemandMultForService(const ServicesModelSettings& cfg, ServiceType t)
{
  switch (t) {
    case ServiceType::Education: return cfg.educationDemandMult;
    case ServiceType::Health: return cfg.healthDemandMult;
    case ServiceType::Safety: return cfg.safetyDemandMult;
  }
  return 1.0f;
}

int SupplyForService(const ServicesModelSettings& cfg, ServiceType t, int level)
{
  const int li = ClampLevel(level) - 1;
  switch (t) {
    case ServiceType::Education: return std::max(0, cfg.educationSupplyPerLevel[li]);
    case ServiceType::Health: return std::max(0, cfg.healthSupplyPerLevel[li]);
    case ServiceType::Safety: return std::max(0, cfg.safetySupplyPerLevel[li]);
  }
  return 0;
}

int MaintenanceForService(const ServicesModelSettings& cfg, ServiceType t, int level)
{
  const int li = ClampLevel(level) - 1;
  switch (t) {
    case ServiceType::Education: return std::max(0, cfg.educationMaintenancePerDay[li]);
    case ServiceType::Health: return std::max(0, cfg.healthMaintenancePerDay[li]);
    case ServiceType::Safety: return std::max(0, cfg.safetyMaintenancePerDay[li]);
  }
  return 0;
}

// Smoothly map accessibility (capacity-per-demand) into satisfaction in [0,1].
//
// We use a saturating curve so adding services beyond the baseline still helps,
// but with diminishing returns.
float AccessToSatisfaction(float access, float targetAccess)
{
  if (!(access > 0.0f)) return 0.0f;
  if (!(targetAccess > 0.0f)) return Clamp01(access);

  // Set k so access==targetAccess yields ~0.5.
  const float k = static_cast<float>(std::log(2.0)) / targetAccess;
  const float sat = 1.0f - static_cast<float>(std::exp(-access * k));
  return Clamp01(sat);
}

bool MaskUsable(const std::vector<std::uint8_t>* mask, int w, int h)
{
  if (!mask) return false;
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  return mask->size() == n;
}

} // namespace

ServicesResult ComputeServices(const World& world, const ServicesModelSettings& cfg,
                               const std::vector<ServiceFacility>& facilities,
                               const ZoneAccessMap* precomputedZoneAccess,
                               const std::vector<std::uint8_t>* precomputedRoadToEdge)
{
  ServicesResult out;
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.education.assign(n, 0.0f);
  out.health.assign(n, 0.0f);
  out.safety.assign(n, 0.0f);
  out.overall.assign(n, 0.0f);

  if (!cfg.enabled) return out;

  // Road-to-edge mask (outside connection rule) is optional.
  std::vector<std::uint8_t> roadToEdgeOwned;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (cfg.requireOutsideConnection) {
    if (MaskUsable(precomputedRoadToEdge, w, h)) {
      roadToEdge = precomputedRoadToEdge;
    } else {
      roadToEdgeOwned.resize(n, 0);
      ComputeRoadsConnectedToEdge(world, roadToEdgeOwned);
      roadToEdge = &roadToEdgeOwned;
    }
  }

  // Zone access map (supports interior zoning blocks).
  ZoneAccessMap zamOwned;
  const ZoneAccessMap* zam = nullptr;
  if (precomputedZoneAccess && precomputedZoneAccess->w == w && precomputedZoneAccess->h == h &&
      precomputedZoneAccess->roadIdx.size() == n) {
    zam = precomputedZoneAccess;
  } else {
    zamOwned = BuildZoneAccessMap(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr);
    if (zamOwned.w == w && zamOwned.h == h && zamOwned.roadIdx.size() == n) {
      zam = &zamOwned;
    }
  }

  // Precompute base demand on zone tiles (independent of service type).
  std::vector<float> baseDemand(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      baseDemand[FlatIdx(x, y, w)] = BaseDemandForTile(t, cfg);
    }
  }

  // Temporary accessibility fields (raw, capacity-per-demand).
  std::vector<float> accessEdu(n, 0.0f);
  std::vector<float> accessHealth(n, 0.0f);
  std::vector<float> accessSafety(n, 0.0f);

  const int radiusMilli = std::max(0, cfg.catchmentRadiusSteps) * 1000;

  // For each facility, compute a local supply/demand ratio and distribute it onto demand tiles.
  for (const ServiceFacility& f : facilities) {
    const int si = ServiceIdx(f.type);
    if (si < 0 || si >= 3) continue;
    out.totalFacilities[static_cast<std::size_t>(si)]++;

    if (!f.enabled) continue;

    if (!world.inBounds(f.tile.x, f.tile.y)) continue;

    // Map the facility to a road tile that acts as its access point.
    Point accessRoad{0, 0};
    const Tile& ft = world.at(f.tile.x, f.tile.y);
    bool hasAccess = false;

    if (ft.overlay == Overlay::Road) {
      accessRoad = f.tile;
      hasAccess = true;
    } else {
      // Reuse deterministic adjacency selection used by other subsystems.
      hasAccess = PickAdjacentRoadTile(world, MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr, f.tile.x, f.tile.y,
                                      accessRoad);
    }

    if (!hasAccess) continue;

    // Build a road isochrone from the facility access road.
    const int sourceIdx = accessRoad.y * w + accessRoad.x;
    std::vector<int> sources;
    sources.push_back(sourceIdx);

    RoadIsochroneConfig rcfg;
    rcfg.requireOutsideConnection = cfg.requireOutsideConnection;
    rcfg.weightMode = cfg.weightMode;
    rcfg.computeOwner = false;

    const RoadIsochroneField roadField = BuildRoadIsochroneField(world, sources, rcfg,
                                                                 MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr);

    TileAccessCostConfig tcfg;
    tcfg.includeRoadTiles = false;
    tcfg.includeZones = true;
    tcfg.includeNonZonesAdjacentToRoad = true; // fallback when ZoneAccessMap lacks an assignment
    tcfg.includeWater = false;
    tcfg.accessStepCostMilli = 0;
    tcfg.useZoneAccessMap = true;

    const std::vector<int> tileCost = BuildTileAccessCostField(world, roadField, tcfg,
                                                               MaskUsable(roadToEdge, w, h) ? roadToEdge : nullptr,
                                                               zam);

    if (tileCost.size() != n) continue;

    // Step 1 (2SFCA): compute facility-local demand within the catchment.
    const float demandMult = DemandMultForService(cfg, f.type);
    double demandSum = 0.0;

    if (radiusMilli > 0 && demandMult > 0.0f) {
      for (std::size_t i = 0; i < n; ++i) {
        const float bd = baseDemand[i];
        if (!(bd > 0.0f)) continue;
        const int c = tileCost[i];
        if (c < 0 || c > radiusMilli) continue;
        const float wgt = DistanceWeight(cfg, c, radiusMilli);
        if (!(wgt > 0.0f)) continue;
        demandSum += static_cast<double>(bd) * static_cast<double>(demandMult) * static_cast<double>(wgt);
      }
    }

    const int supply = SupplyForService(cfg, f.type, static_cast<int>(f.level));
    if (supply <= 0) continue;

    // Facilities with no reachable local demand do not contribute.
    if (!(demandSum > 0.0)) continue;

    const double ratio = static_cast<double>(supply) / demandSum;

    // Step 2 (E2SFCA): distribute the ratio onto demand tiles inside the catchment.
    std::vector<float>* targetAccess = nullptr;
    if (f.type == ServiceType::Education) targetAccess = &accessEdu;
    else if (f.type == ServiceType::Health) targetAccess = &accessHealth;
    else if (f.type == ServiceType::Safety) targetAccess = &accessSafety;

    if (targetAccess) {
      for (std::size_t i = 0; i < n; ++i) {
        const float bd = baseDemand[i];
        if (!(bd > 0.0f)) continue;
        const int c = tileCost[i];
        if (c < 0 || c > radiusMilli) continue;
        const float wgt = DistanceWeight(cfg, c, radiusMilli);
        if (!(wgt > 0.0f)) continue;
        (*targetAccess)[i] += static_cast<float>(ratio * static_cast<double>(wgt));
      }
    }

    out.activeFacilities[static_cast<std::size_t>(si)]++;
    out.maintenanceCostPerDay += MaintenanceForService(cfg, f.type, static_cast<int>(f.level));
  }

  // Convert accessibility to satisfaction fields.
  for (std::size_t i = 0; i < n; ++i) {
    out.education[i] = AccessToSatisfaction(accessEdu[i], cfg.targetAccess);
    out.health[i] = AccessToSatisfaction(accessHealth[i], cfg.targetAccess);
    out.safety[i] = AccessToSatisfaction(accessSafety[i], cfg.targetAccess);
    out.overall[i] = (out.education[i] + out.health[i] + out.safety[i]) / 3.0f;
  }

  // Demand-weighted citywide satisfaction metrics.
  double demEdu = 0.0, demHealth = 0.0, demSafety = 0.0;
  double sumEdu = 0.0, sumHealth = 0.0, sumSafety = 0.0;

  for (std::size_t i = 0; i < n; ++i) {
    const float bd = baseDemand[i];
    if (!(bd > 0.0f)) continue;

    const double dEdu = static_cast<double>(bd) * static_cast<double>(cfg.educationDemandMult);
    const double dHealth = static_cast<double>(bd) * static_cast<double>(cfg.healthDemandMult);
    const double dSafety = static_cast<double>(bd) * static_cast<double>(cfg.safetyDemandMult);

    if (dEdu > 0.0) {
      demEdu += dEdu;
      sumEdu += dEdu * static_cast<double>(out.education[i]);
    }
    if (dHealth > 0.0) {
      demHealth += dHealth;
      sumHealth += dHealth * static_cast<double>(out.health[i]);
    }
    if (dSafety > 0.0) {
      demSafety += dSafety;
      sumSafety += dSafety * static_cast<double>(out.safety[i]);
    }
  }

  out.educationSatisfaction = (demEdu > 0.0) ? static_cast<float>(sumEdu / demEdu) : 0.0f;
  out.healthSatisfaction = (demHealth > 0.0) ? static_cast<float>(sumHealth / demHealth) : 0.0f;
  out.safetySatisfaction = (demSafety > 0.0) ? static_cast<float>(sumSafety / demSafety) : 0.0f;
  out.overallSatisfaction = (out.educationSatisfaction + out.healthSatisfaction + out.safetySatisfaction) / 3.0f;

  return out;
}

std::vector<ServiceFacility> ExtractServiceFacilitiesFromWorld(const World& world)
{
  std::vector<ServiceFacility> out;
  out.reserve(64);

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      ServiceType type;
      bool isFacility = true;

      switch (t.overlay) {
        case Overlay::School: type = ServiceType::Education; break;
        case Overlay::Hospital: type = ServiceType::Health; break;
        case Overlay::PoliceStation: type = ServiceType::Safety; break;
        case Overlay::FireStation: type = ServiceType::Safety; break;
        default: isFacility = false; break;
      }

      if (!isFacility) continue;

      ServiceFacility f;
      f.tile = Point{x, y};
      f.type = type;
      // Clamp defensively; saves/scripts might produce odd values.
      f.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(t.level), 1, 3));
      f.enabled = true;
      out.push_back(f);
    }
  }

  return out;
}

} // namespace isocity
