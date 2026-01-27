#pragma once

#include "isocity/World.hpp"

#include <vector>

namespace isocity {

struct EnergyModelResult;
struct TrafficResult;
struct GoodsResult;

struct CarbonModelConfig {
  // Emission sources
  bool includeEnergy = true;
  float gridCo2PerEnergy = 0.9f;

  bool includeProcess = true;
  float industrialProcessPerOccupant = 0.020f;
  float commercialProcessPerOccupant = 0.010f;
  float civicProcessPerOccupant = 0.008f;

  bool includeTraffic = true;
  float trafficCo2PerCommuterTile = 0.0015f;

  bool includeGoods = true;
  float goodsCo2PerTruckTile = 0.0025f;

  // Sequestration sinks
  bool includeSequestration = true;
  float parkSequestration = 0.050f;
  float grassSequestration = 0.015f;

  // Normalization stability epsilon.
  float minNormDenom = 1e-4f;
};

struct CarbonModelResult {
  int w = 0;
  int h = 0;
  CarbonModelConfig cfg{};

  // Raw proxy units (dimensionless).
  std::vector<float> emissionRaw;
  std::vector<float> sequestrationRaw;
  std::vector<float> netRaw; // sequestrationRaw - emissionRaw

  // Normalized [0,1] fields for visualization.
  std::vector<float> emission01;
  std::vector<float> sequestration01;

  // Net balance mapped into [0,1]: 0 deficit, 0.5 neutral, 1 surplus.
  std::vector<float> balance01;

  float maxEmissionRaw = 0.0f;
  float maxSequestrationRaw = 0.0f;
  float maxAbsNetRaw = 0.0f;

  // Aggregate stats.
  float totalEmissionRaw = 0.0f;
  float totalSequestrationRaw = 0.0f;
  float totalNetRaw = 0.0f;

  int contributingTileCount = 0;
};

CarbonModelResult ComputeCarbonModel(const World& world,
                                    const CarbonModelConfig& cfg = {},
                                    const EnergyModelResult* energy = nullptr,
                                    const TrafficResult* traffic = nullptr,
                                    const GoodsResult* goods = nullptr);

} // namespace isocity
