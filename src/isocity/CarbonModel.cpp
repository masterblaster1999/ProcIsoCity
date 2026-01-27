#include "isocity/CarbonModel.hpp"

#include "isocity/EnergyModel.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/Goods.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace isocity {

static inline float Clamp01(float v) {
  return std::clamp(v, 0.0f, 1.0f);
}

static inline std::size_t Idx(int x, int y, int w) {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

static inline bool IsBuildingOverlay(Overlay o) {
  return (o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial ||
          o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation);
}

CarbonModelResult ComputeCarbonModel(const World& world,
                                    const CarbonModelConfig& cfg,
                                    const EnergyModelResult* energyIn,
                                    const TrafficResult* traffic,
                                    const GoodsResult* goods)
{
  CarbonModelResult out;
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  // Ensure we have an energy result if requested.
  EnergyModelResult energyOwned;
  const EnergyModelResult* energy = energyIn;
  if (cfg.includeEnergy) {
    const bool ok = (energy != nullptr && energy->w == w && energy->h == h &&
                     energy->demandRaw.size() == n && energy->solarRaw.size() == n);
    if (!ok) {
      EnergyModelConfig ec{};
      // Keep this self-contained fallback stable and cheap.
      ec.useHeatIslandCooling = false;
      energyOwned = ComputeEnergyModel(world, ec, /*solar=*/nullptr, /*heatIsland=*/nullptr);
      energy = &energyOwned;
    }
  }

  out.emissionRaw.assign(n, 0.0f);
  out.sequestrationRaw.assign(n, 0.0f);
  out.netRaw.assign(n, 0.0f);
  out.emission01.assign(n, 0.0f);
  out.sequestration01.assign(n, 0.0f);
  out.balance01.assign(n, 0.5f);

  const bool haveTraffic = (traffic && traffic->roadTraffic.size() == n);
  const bool haveGoods = (goods && goods->roadGoodsTraffic.size() == n);

  float maxE = 0.0f;
  float maxS = 0.0f;
  float maxAbsN = 0.0f;

  float totE = 0.0f;
  float totS = 0.0f;
  float totN = 0.0f;
  int contrib = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = Idx(x, y, w);
      const Tile& t = world.at(x, y);

      float emission = 0.0f;
      float sink = 0.0f;

      // Skip water for both emissions and sinks.
      if (t.terrain != Terrain::Water) {
        // Operational energy emissions: net grid demand.
        if (cfg.includeEnergy && energy && IsBuildingOverlay(t.overlay)) {
          const float demand = energy->demandRaw[i];
          const float solar = energy->solarRaw[i];
          const float grid = std::max(0.0f, demand - solar);
          emission += grid * cfg.gridCo2PerEnergy;
        }

        // Simple process emissions for non-residential activity.
        if (cfg.includeProcess) {
          const float occ = static_cast<float>(t.occupants);
          switch (t.overlay) {
          case Overlay::Industrial:
            emission += occ * cfg.industrialProcessPerOccupant;
            break;
          case Overlay::Commercial:
            emission += occ * cfg.commercialProcessPerOccupant;
            break;
          case Overlay::School:
          case Overlay::Hospital:
          case Overlay::PoliceStation:
          case Overlay::FireStation:
            emission += occ * cfg.civicProcessPerOccupant;
            break;
          default:
            break;
          }
        }

        // Road traffic emissions: attribute emissions to the road tiles that carry flows.
        if (cfg.includeTraffic && haveTraffic && t.overlay == Overlay::Road) {
          emission += static_cast<float>(traffic->roadTraffic[i]) * cfg.trafficCo2PerCommuterTile;
        }

        // Goods traffic emissions.
        if (cfg.includeGoods && haveGoods && t.overlay == Overlay::Road) {
          emission += static_cast<float>(goods->roadGoodsTraffic[i]) * cfg.goodsCo2PerTruckTile;
        }

        // Sequestration sinks.
        if (cfg.includeSequestration) {
          if (t.overlay == Overlay::Park) {
            sink += cfg.parkSequestration;
          } else if (t.overlay == Overlay::None && t.terrain == Terrain::Grass) {
            sink += cfg.grassSequestration;
          }
        }
      }

      const float net = sink - emission;

      out.emissionRaw[i] = emission;
      out.sequestrationRaw[i] = sink;
      out.netRaw[i] = net;

      if (emission > 0.0f || sink > 0.0f) {
        contrib += 1;
      }

      maxE = std::max(maxE, emission);
      maxS = std::max(maxS, sink);
      maxAbsN = std::max(maxAbsN, std::abs(net));

      totE += emission;
      totS += sink;
      totN += net;
    }
  }

  const float denomE = std::max(maxE, cfg.minNormDenom);
  const float denomS = std::max(maxS, cfg.minNormDenom);
  const float denomN = std::max(maxAbsN, cfg.minNormDenom);

  for (std::size_t i = 0; i < n; ++i) {
    out.emission01[i] = Clamp01(out.emissionRaw[i] / denomE);
    out.sequestration01[i] = Clamp01(out.sequestrationRaw[i] / denomS);

    // Map net into [0,1] with 0.5 as neutral.
    const float nrm = std::clamp(out.netRaw[i] / denomN, -1.0f, 1.0f);
    out.balance01[i] = Clamp01(0.5f + 0.5f * nrm);
  }

  out.maxEmissionRaw = maxE;
  out.maxSequestrationRaw = maxS;
  out.maxAbsNetRaw = maxAbsN;
  out.totalEmissionRaw = totE;
  out.totalSequestrationRaw = totS;
  out.totalNetRaw = totN;
  out.contributingTileCount = contrib;

  return out;
}

} // namespace isocity
