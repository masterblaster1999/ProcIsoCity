#include "isocity/EnergyModel.hpp"

#include "isocity/HeatIsland.hpp"
#include "isocity/SolarPotential.hpp"

#include <algorithm>
#include <cstddef>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool IsZone(Overlay o)
{
  return (o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial);
}

inline bool IsCivic(Overlay o)
{
  return (o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation);
}

} // namespace

EnergyModelResult ComputeEnergyModel(const World& world,
                                    const EnergyModelConfig& cfg,
                                    const SolarPotentialResult* solarIn,
                                    const HeatIslandResult* heatIslandIn)
{
  EnergyModelResult out{};

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.w = w;
  out.h = h;
  out.cfg = cfg;

  out.demandRaw.assign(n, 0.0f);
  out.solarRaw.assign(n, 0.0f);
  out.netRaw.assign(n, 0.0f);

  out.demand01.assign(n, 0.0f);
  out.solar01.assign(n, 0.0f);
  out.balance01.assign(n, 0.5f);

  // --- Ensure we have solar potential ---
  SolarPotentialResult solarOwned{};
  const SolarPotentialResult* solar = solarIn;
  if (!solar || solar->w != w || solar->h != h || solar->potential01.size() != n) {
    SolarPotentialConfig sc{};
    sc.azimuthSamples = 16;
    solarOwned = ComputeSolarPotential(world, sc);
    solar = &solarOwned;
  }

  // --- Ensure we have a heat island field (optional) ---
  HeatIslandResult heatOwned{};
  const HeatIslandResult* heat = nullptr;
  if (cfg.useHeatIslandCooling) {
    heat = heatIslandIn;
    if (!heat || heat->w != w || heat->h != h || heat->heat01.size() != n) {
      HeatIslandConfig hc{};
      heatOwned = ComputeHeatIsland(world, hc, nullptr, nullptr);
      heat = &heatOwned;
    }
  }

  float minNet = 0.0f;
  float maxNet = 0.0f;

  const float lvlBoostD = cfg.levelDemandBoost;
  const float lvlBoostS = cfg.levelSupplyBoost;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
      const float lvlMulDemand = 1.0f + lvlBoostD * static_cast<float>(lvl - 1);
      const float lvlMulSolar = 1.0f + lvlBoostS * static_cast<float>(lvl - 1);

      const bool isZone = IsZone(t.overlay);
      const bool isCivic = IsCivic(t.overlay);
      const bool isBuilding = (isZone || isCivic);

      float base = 0.0f;
      float perOcc = 0.0f;
      if (t.overlay == Overlay::Residential) {
        base = cfg.residentialBaseDemand;
        perOcc = cfg.residentialDemandPerOccupant;
      } else if (t.overlay == Overlay::Commercial) {
        base = cfg.commercialBaseDemand;
        perOcc = cfg.commercialDemandPerOccupant;
      } else if (t.overlay == Overlay::Industrial) {
        base = cfg.industrialBaseDemand;
        perOcc = cfg.industrialDemandPerOccupant;
      } else if (isCivic) {
        base = cfg.civicBaseDemand;
        perOcc = cfg.civicDemandPerOccupant;
      }

      const int occ = static_cast<int>(t.occupants);

      float demand = 0.0f;
      if (base > 0.0f || perOcc > 0.0f) {
        if (cfg.includeBaseDemandWhenEmpty || occ > 0) {
          demand += base;
        }
        if (occ > 0) {
          demand += perOcc * static_cast<float>(occ);
        }

        demand *= lvlMulDemand;

        if (heat) {
          const float h01 = Clamp01(heat->heat01[i]);
          demand *= (1.0f + cfg.heatCoolingBoost * h01);
        }
      }

      // Rooftop solar: use SolarPotential's normalized potential01 and scale it.
      float sol = 0.0f;
      if (!cfg.requireRoofForSolar || isBuilding) {
        if (solar && solar->potential01.size() == n) {
          const float pot = Clamp01(solar->potential01[i]);
          sol = cfg.solarSupplyScale * pot * lvlMulSolar;
        }
      }

      const float net = sol - demand;

      out.demandRaw[i] = demand;
      out.solarRaw[i] = sol;
      out.netRaw[i] = net;

      out.maxDemandRaw = std::max(out.maxDemandRaw, demand);
      out.maxSolarRaw = std::max(out.maxSolarRaw, sol);
      minNet = std::min(minNet, net);
      maxNet = std::max(maxNet, net);

      out.totalDemandRaw += demand;
      out.totalSolarRaw += sol;
      out.totalNetRaw += net;

      if (isBuilding) {
        out.buildingTileCount += 1;
        out.populationOnBuildingTiles += occ;
      }
    }
  }

  out.maxAbsNetRaw = std::max(std::abs(minNet), std::abs(maxNet));

  const float dDenom = std::max(cfg.minNormDenom, out.maxDemandRaw);
  const float sDenom = std::max(cfg.minNormDenom, out.maxSolarRaw);
  const float nDenom = std::max(cfg.minNormDenom, out.maxAbsNetRaw);

  for (std::size_t i = 0; i < n; ++i) {
    out.demand01[i] = Clamp01(out.demandRaw[i] / dDenom);
    out.solar01[i] = Clamp01(out.solarRaw[i] / sDenom);

    const float bal = 0.5f + 0.5f * (out.netRaw[i] / nDenom);
    out.balance01[i] = Clamp01(bal);
  }

  if (out.totalDemandRaw > cfg.minNormDenom) {
    out.renewableShare01 = Clamp01(out.totalSolarRaw / out.totalDemandRaw);
  } else {
    out.renewableShare01 = 0.0f;
  }

  return out;
}

} // namespace isocity
