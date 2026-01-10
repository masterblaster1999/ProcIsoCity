#include "isocity/EvacuationScenario.hpp"

#include <algorithm>
#include <cmath>

namespace isocity {

namespace {

static std::vector<float> ExtractHeights(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  std::vector<float> out;
  if (w <= 0 || h <= 0) return out;
  out.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      out[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
          world.at(x, y).height;
    }
  }
  return out;
}

static std::vector<std::uint8_t> BuildWaterDrainMask(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));
  std::vector<std::uint8_t> mask;
  mask.assign(n, 0);
  if (w <= 0 || h <= 0) return mask;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      // Treat non-bridge water tiles as drains so "existing" lakes/ocean aren't filled.
      if (t.terrain == Terrain::Water && t.overlay != Overlay::Road) {
        mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = 1;
      }
    }
  }
  return mask;
}

} // namespace

const char* EvacuationHazardModeName(EvacuationHazardMode m)
{
  switch (m) {
  case EvacuationHazardMode::None: return "none";
  case EvacuationHazardMode::Sea: return "sea";
  case EvacuationHazardMode::Ponding: return "pond";
  case EvacuationHazardMode::Both: return "both";
  }
  return "unknown";
}

EvacuationScenarioResult ComputeEvacuationScenario(const World& world, const EvacuationScenarioConfig& cfg)
{
  EvacuationScenarioResult out;
  const int w = world.width();
  const int h = world.height();
  out.w = w;
  out.h = h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.hazardMask.assign(n, 0);

  const bool wantSea = (cfg.hazardMode == EvacuationHazardMode::Sea || cfg.hazardMode == EvacuationHazardMode::Both);
  const bool wantPond =
      (cfg.hazardMode == EvacuationHazardMode::Ponding || cfg.hazardMode == EvacuationHazardMode::Both);

  if (wantSea || wantPond) {
    const std::vector<float> heights = ExtractHeights(world);

    if (wantSea) {
      out.sea = ComputeSeaLevelFlood(heights, w, h, cfg.seaLevel, cfg.seaCfg);
      for (std::size_t i = 0; i < n && i < out.sea.flooded.size(); ++i) {
        if (out.sea.flooded[i] != 0) out.hazardMask[i] = 1;
      }
    }

    if (wantPond) {
      std::vector<std::uint8_t> drains = BuildWaterDrainMask(world);
      out.pond = FillDepressionsPriorityFlood(heights, w, h, &drains, cfg.pondCfg);

      const float minDepth = std::max(0.0f, cfg.pondMinDepth);
      for (std::size_t i = 0; i < n && i < out.pond.depth.size(); ++i) {
        if (out.pond.depth[i] >= minDepth) out.hazardMask[i] = 1;
      }
    }
  }

  // Bridges: allow roads on water to remain passable in hazard scenarios.
  if (cfg.bridgesPassable && (cfg.hazardMode != EvacuationHazardMode::None)) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        const Tile& t = world.at(x, y);
        if (t.overlay == Overlay::Road && t.terrain == Terrain::Water) {
          out.hazardMask[i] = 0;
        }
      }
    }
  }

  const std::vector<std::uint8_t>* maskPtr = (cfg.hazardMode == EvacuationHazardMode::None) ? nullptr : &out.hazardMask;
  out.evac = ComputeEvacuationToEdge(world, cfg.evac, maskPtr);
  return out;
}

EvacuationScenarioHeatmaps BuildEvacuationScenarioHeatmaps(const World& world, const EvacuationScenarioResult& r)
{
  EvacuationScenarioHeatmaps out;
  const int w = world.width();
  const int h = world.height();
  out.w = w;
  out.h = h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.evacTime.assign(n, 0.0f);
  out.evacUnreachable.assign(n, 0.0f);
  out.evacFlow.assign(n, 0.0f);

  // Normalize evacuation time by p95 (robust) with a fallback to max.
  int maxCostMilli = 0;
  for (int c : r.evac.resCostMilli) {
    if (c > maxCostMilli) maxCostMilli = c;
  }
  const int p95Milli = static_cast<int>(std::lround(std::max(0.0f, r.evac.p95EvacTime) * 1000.0f));
  const float denomTime = static_cast<float>(std::max(1, std::max(maxCostMilli, p95Milli)));

  const bool hazardOk = (r.hazardMask.size() == n);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      const Tile& t = world.at(x, y);

      if (t.overlay == Overlay::Residential && t.terrain != Terrain::Water) {
        const bool hazard = hazardOk ? (r.hazardMask[i] != 0) : false;
        const int c = (i < r.evac.resCostMilli.size()) ? r.evac.resCostMilli[i] : -1;
        if (c >= 0) {
          out.evacTime[i] = std::clamp(static_cast<float>(c) / denomTime, 0.0f, 1.0f);
        } else if (!hazard) {
          // Unreachable (but not explicitly blocked by hazard).
          out.evacUnreachable[i] = 1.0f;
        }
      }

      if (t.overlay == Overlay::Road) {
        const std::uint32_t flow = (i < r.evac.evacRoadFlow.size()) ? r.evac.evacRoadFlow[i] : 0u;
        if (flow > 0u) {
          const float denomFlow = static_cast<float>(std::max<std::uint32_t>(1u, r.evac.maxEvacRoadFlow));
          out.evacFlow[i] = std::clamp(static_cast<float>(flow) / denomFlow, 0.0f, 1.0f);
        }
      }
    }
  }

  return out;
}

} // namespace isocity
