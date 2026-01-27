#include "isocity/RunoffMitigation.hpp"

#include "isocity/Hydrology.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

inline bool IsCivic(Overlay o)
{
  return (o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation);
}

inline bool IsZone(Overlay o)
{
  return (o == Overlay::Residential || o == Overlay::Commercial || o == Overlay::Industrial);
}

float DemandWeight(const Tile& t, RunoffMitigationDemandMode mode)
{
  switch (mode) {
  case RunoffMitigationDemandMode::ResidentialOccupants:
    return (t.overlay == Overlay::Residential && t.occupants > 0) ? static_cast<float>(t.occupants) : 0.0f;
  case RunoffMitigationDemandMode::AllOccupants:
    return (t.occupants > 0) ? static_cast<float>(t.occupants) : 0.0f;
  case RunoffMitigationDemandMode::ResidentialTiles:
    return (t.overlay == Overlay::Residential) ? 1.0f : 0.0f;
  case RunoffMitigationDemandMode::ZoneTiles:
    return IsZone(t.overlay) ? 1.0f : 0.0f;
  }
  return 0.0f;
}

float RetentionForTile(const Tile& t, Overlay overlayOverride, bool useOverride, const RunoffPollutionConfig& cfg)
{
  const Overlay o = useOverride ? overlayOverride : t.overlay;

  float retain = 0.0f;
  if (t.terrain == Terrain::Water) {
    retain = cfg.waterIsSink ? cfg.filterWater : 0.0f;
  } else {
    if (o == Overlay::Park) retain += cfg.filterPark;
    if (t.terrain == Terrain::Grass) retain += cfg.filterGrass;
    if (t.terrain == Terrain::Sand) retain += cfg.filterSand;
    if (o == Overlay::Road) retain += cfg.filterRoad;
  }

  return std::clamp(retain, 0.0f, 1.0f);
}

bool IsCandidate(const Tile& t, const RunoffMitigationConfig& cfg)
{
  if (cfg.excludeWater && t.terrain == Terrain::Water) return false;

  // Never "place" a park onto an existing park.
  if (t.overlay == Overlay::Park) return false;

  if (t.overlay == Overlay::None) return true;
  if (t.overlay == Overlay::Road) return cfg.allowReplaceRoad;

  if (cfg.allowReplaceZones) {
    if (IsZone(t.overlay)) return true;
    if (IsCivic(t.overlay)) return true;
  }

  return false;
}

double ComputeObjective(const World& world, int w, int h,
                        const std::vector<int>& orderDesc,
                        const std::vector<int>& dir,
                        const std::vector<float>& denom,
                        const std::vector<float>& localLoad,
                        const std::vector<float>& retain,
                        RunoffMitigationDemandMode demandMode)
{
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<float> massIn(n, 0.0f);

  double obj = 0.0;
  for (int idxLin : orderDesc) {
    const std::size_t i = static_cast<std::size_t>(idxLin);
    const int x = (w > 0) ? (idxLin % w) : 0;
    const int y = (w > 0) ? (idxLin / w) : 0;

    const float massTotal = localLoad[i] + massIn[i];
    const float outflow = massTotal * (1.0f - retain[i]);
    const float conc = outflow / denom[i];

    const float weight = DemandWeight(world.at(x, y), demandMode);
    if (weight > 0.0f) {
      obj += static_cast<double>(weight) * static_cast<double>(conc);
    }

    if (dir.size() == n) {
      const int to = dir[i];
      if (to >= 0 && to < w * h) {
        massIn[static_cast<std::size_t>(to)] += outflow;
      }
    }
  }

  return obj;
}

} // namespace

RunoffMitigationResult SuggestRunoffMitigationParks(const World& world,
                                                   const RunoffMitigationConfig& cfg,
                                                   const TrafficResult* traffic)
{
  RunoffMitigationResult out{};

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return out;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  out.w = w;
  out.h = h;
  out.cfg = cfg;
  out.priorityRaw.assign(n, 0.0f);
  out.priority01.assign(n, 0.0f);
  out.planMask.assign(n, std::uint8_t{0});

  // Heightfield for routing.
  std::vector<float> heights(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      heights[FlatIdx(x, y, w)] = world.at(x, y).height;
    }
  }

  const HydrologyField hydro = BuildHydrologyField(heights, w, h);
  const std::vector<int>& dir = hydro.dir;

  // Flow accumulation for dilution.
  std::vector<int> accum(n, 1);
  if (hydro.accum.size() == n) accum = hydro.accum;

  // Precompute dilution denominator per tile.
  std::vector<float> denom(n, 1.0f);
  const float dilExp = cfg.runoffCfg.dilutionExponent;
  if (dilExp != 0.0f) {
    for (std::size_t i = 0; i < n; ++i) {
      const int a = std::max(1, accum[i]);
      float d = std::pow(static_cast<float>(a), dilExp);
      if (!std::isfinite(d) || d <= 0.0f) d = 1.0f;
      denom[i] = d;
    }
  }

  // Normalize traffic if provided (same approach as RunoffPollution).
  std::uint16_t maxTraffic = 0;
  if (traffic && traffic->roadTraffic.size() == n) {
    maxTraffic = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (maxTraffic == 0) {
      for (std::uint16_t v : traffic->roadTraffic) maxTraffic = std::max(maxTraffic, v);
    }
  }

  const float clampAbs = std::max(0.01f, cfg.runoffCfg.clampLoad);
  const float occScale = static_cast<float>(std::max(1, cfg.runoffCfg.occupantScale));

  // Local load field.
  std::vector<float> localLoad(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      float l = 0.0f;
      switch (t.overlay) {
      case Overlay::Road: {
        const int lvl = std::clamp<int>(static_cast<int>(t.level), 1, 3);
        l += cfg.runoffCfg.roadBase + cfg.runoffCfg.roadClassBoost * static_cast<float>(lvl - 1);

        float tr01 = 0.0f;
        if (maxTraffic > 0 && traffic && traffic->roadTraffic.size() == n) {
          tr01 = static_cast<float>(traffic->roadTraffic[i]) / static_cast<float>(maxTraffic);
        } else {
          tr01 = cfg.runoffCfg.fallbackCommuteTraffic01;
        }
        l += cfg.runoffCfg.roadTrafficBoost * Clamp01(tr01);
      } break;
      case Overlay::Residential:
        l += cfg.runoffCfg.residentialLoad;
        break;
      case Overlay::Commercial:
        l += cfg.runoffCfg.commercialLoad;
        break;
      case Overlay::Industrial:
        l += cfg.runoffCfg.industrialLoad;
        break;
      default:
        if (IsCivic(t.overlay)) {
          l += cfg.runoffCfg.civicLoad;
        }
        break;
      }

      if (t.occupants > 0) {
        const float occ01 = Clamp01(static_cast<float>(t.occupants) / occScale);
        l += cfg.runoffCfg.occupantBoost * occ01;
      }

      l = std::clamp(l, 0.0f, clampAbs);
      localLoad[i] = l;
    }
  }

  // Current retention per tile.
  std::vector<float> retain(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      retain[i] = RetentionForTile(world.at(x, y), Overlay::None, false, cfg.runoffCfg);
    }
  }

  // Process order: height descending (higher routes into lower). This mirrors RunoffPollution.
  std::vector<int> orderDesc;
  orderDesc.reserve(n);
  for (int i = 0; i < w * h; ++i) orderDesc.push_back(i);

  std::stable_sort(orderDesc.begin(), orderDesc.end(), [&](int a, int b) {
    const float ha = heights[static_cast<std::size_t>(a)];
    const float hb = heights[static_cast<std::size_t>(b)];
    if (ha == hb) return a < b;
    return ha > hb;
  });

  // Route once to get mass totals at each tile.
  std::vector<float> massIn(n, 0.0f);
  std::vector<float> massTotal(n, 0.0f);
  for (int idxLin : orderDesc) {
    const std::size_t i = static_cast<std::size_t>(idxLin);
    const float m = localLoad[i] + massIn[i];
    massTotal[i] = m;
    const float outflow = m * (1.0f - retain[i]);

    if (dir.size() == n) {
      const int to = dir[i];
      if (to >= 0 && to < w * h) {
        massIn[static_cast<std::size_t>(to)] += outflow;
      }
    }
  }

  // Objective before applying any suggested parks.
  out.objectiveBefore = ComputeObjective(world, w, h, orderDesc, dir, denom, localLoad, retain, cfg.demandMode);
  out.objectiveAfter = out.objectiveBefore;

  // Adjoint pass: compute dObjective / dMassOut at each tile.
  std::vector<float> adjOut(n, 0.0f);
  // Ascending order = reverse of descending order (lowest first).
  for (auto it = orderDesc.rbegin(); it != orderDesc.rend(); ++it) {
    const int idxLin = *it;
    const std::size_t i = static_cast<std::size_t>(idxLin);
    const int x = (w > 0) ? (idxLin % w) : 0;
    const int y = (w > 0) ? (idxLin / w) : 0;

    const float weight = DemandWeight(world.at(x, y), cfg.demandMode);
    const float base = (weight > 0.0f) ? (weight / denom[i]) : 0.0f;

    float down = 0.0f;
    if (dir.size() == n) {
      const int to = dir[i];
      if (to >= 0 && to < w * h) {
        const std::size_t j = static_cast<std::size_t>(to);
        down = adjOut[j] * (1.0f - retain[j]);
      }
    }

    adjOut[i] = base + down;
  }

  // Compute raw benefit for candidates.
  float maxRaw = 0.0f;
  std::vector<int> candidates;
  candidates.reserve(n / 8u);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = FlatIdx(x, y, w);
      const Tile& t = world.at(x, y);

      if (!IsCandidate(t, cfg)) continue;

      const float curR = retain[i];
      const float newR = RetentionForTile(t, Overlay::Park, true, cfg.runoffCfg);
      const float delta = std::max(0.0f, newR - curR);
      if (delta <= 0.0f) continue;

      // First-order objective reduction (linearized).
      const float m = massTotal[i];
      const float raw = adjOut[i] * delta * m;
      if (!(raw > 0.0f)) continue;

      out.priorityRaw[i] = raw;
      maxRaw = std::max(maxRaw, raw);
      candidates.push_back(static_cast<int>(i));
    }
  }

  out.candidateCount = static_cast<int>(candidates.size());

  if (maxRaw > 0.0f) {
    for (std::size_t i = 0; i < n; ++i) {
      if (out.priorityRaw[i] > 0.0f) out.priority01[i] = Clamp01(out.priorityRaw[i] / maxRaw);
    }
  }

  // Sort candidates by benefit (desc), stable tie-break by index.
  std::stable_sort(candidates.begin(), candidates.end(), [&](int a, int b) {
    const float ra = out.priorityRaw[static_cast<std::size_t>(a)];
    const float rb = out.priorityRaw[static_cast<std::size_t>(b)];
    if (ra == rb) return a < b;
    return ra > rb;
  });

  const int toAdd = std::max(0, cfg.parksToAdd);
  const int minSep = std::max(0, cfg.minSeparation);
  std::vector<Point> selected;
  selected.reserve(static_cast<std::size_t>(toAdd));

  for (int idxFlat : candidates) {
    if (static_cast<int>(out.placements.size()) >= toAdd) break;
    const int x = (w > 0) ? (idxFlat % w) : 0;
    const int y = (w > 0) ? (idxFlat / w) : 0;

    bool ok = true;
    if (minSep > 0) {
      for (const Point& p : selected) {
        const int d = std::abs(x - p.x) + std::abs(y - p.y);
        if (d < minSep) {
          ok = false;
          break;
        }
      }
    }

    if (!ok) continue;

    out.planMask[static_cast<std::size_t>(idxFlat)] = std::uint8_t{1};
    out.placements.push_back(RunoffMitigationPlacement{Point{x, y}, static_cast<double>(out.priorityRaw[static_cast<std::size_t>(idxFlat)])});
    selected.push_back(Point{x, y});
  }

  // Compute objective after applying the selected parks (exact reroute, no linearization).
  if (!out.placements.empty()) {
    std::vector<float> retainAfter = retain;
    for (const RunoffMitigationPlacement& p : out.placements) {
      if (p.tile.x < 0 || p.tile.y < 0 || p.tile.x >= w || p.tile.y >= h) continue;
      const std::size_t i = FlatIdx(p.tile.x, p.tile.y, w);
      const Tile& t = world.at(p.tile.x, p.tile.y);
      retainAfter[i] = RetentionForTile(t, Overlay::Park, true, cfg.runoffCfg);
    }

    out.objectiveAfter = ComputeObjective(world, w, h, orderDesc, dir, denom, localLoad, retainAfter, cfg.demandMode);
    out.objectiveReduction = std::max(0.0, out.objectiveBefore - out.objectiveAfter);
  }

  return out;
}

void ApplyRunoffMitigationParks(World& world, const std::vector<RunoffMitigationPlacement>& placements)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return;

  for (const RunoffMitigationPlacement& p : placements) {
    if (p.tile.x < 0 || p.tile.y < 0 || p.tile.x >= w || p.tile.y >= h) continue;
    Tile& t = world.at(p.tile.x, p.tile.y);
    if (t.terrain == Terrain::Water) continue;
    t.overlay = Overlay::Park;
  }
}

} // namespace isocity
