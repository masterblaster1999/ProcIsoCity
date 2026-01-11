#include "isocity/ReplayCapture.hpp"

#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/WorldPatch.hpp"

#include <algorithm>
#include <limits>

namespace isocity {

namespace {

bool ErosionConfigEqual(const ErosionConfig& a, const ErosionConfig& b)
{
  return a.enabled == b.enabled && a.riversEnabled == b.riversEnabled && a.thermalIterations == b.thermalIterations &&
         a.thermalTalus == b.thermalTalus && a.thermalRate == b.thermalRate && a.riverMinAccum == b.riverMinAccum &&
         a.riverCarve == b.riverCarve && a.riverCarvePower == b.riverCarvePower &&
         a.smoothIterations == b.smoothIterations && a.smoothRate == b.smoothRate &&
         a.quantizeScale == b.quantizeScale;
}

bool ProcGenConfigEqual(const ProcGenConfig& a, const ProcGenConfig& b)
{
  return a.terrainScale == b.terrainScale && a.waterLevel == b.waterLevel && a.sandLevel == b.sandLevel &&
         a.hubs == b.hubs && a.extraConnections == b.extraConnections && a.zoneChance == b.zoneChance &&
         a.parkChance == b.parkChance && a.terrainPreset == b.terrainPreset &&
         a.terrainPresetStrength == b.terrainPresetStrength && a.roadHierarchyEnabled == b.roadHierarchyEnabled &&
         a.roadHierarchyStrength == b.roadHierarchyStrength && a.districtingMode == b.districtingMode &&
         ErosionConfigEqual(a.erosion, b.erosion);
}

bool DistrictPolicyEqual(const DistrictPolicy& a, const DistrictPolicy& b)
{
  return a.taxResidentialMult == b.taxResidentialMult && a.taxCommercialMult == b.taxCommercialMult &&
         a.taxIndustrialMult == b.taxIndustrialMult && a.roadMaintenanceMult == b.roadMaintenanceMult &&
         a.parkMaintenanceMult == b.parkMaintenanceMult;
}

bool SimConfigEqual(const SimConfig& a, const SimConfig& b)
{
  if (!(a.tickSeconds == b.tickSeconds && a.parkInfluenceRadius == b.parkInfluenceRadius &&
        a.requireOutsideConnection == b.requireOutsideConnection && a.taxResidential == b.taxResidential &&
        a.taxCommercial == b.taxCommercial && a.taxIndustrial == b.taxIndustrial && a.maintenanceRoad == b.maintenanceRoad &&
        a.maintenancePark == b.maintenancePark && a.taxHappinessPerCapita == b.taxHappinessPerCapita &&
        a.residentialDesirabilityWeight == b.residentialDesirabilityWeight &&
        a.commercialDesirabilityWeight == b.commercialDesirabilityWeight &&
        a.industrialDesirabilityWeight == b.industrialDesirabilityWeight &&
        a.districtPoliciesEnabled == b.districtPoliciesEnabled)) {
    return false;
  }

  for (std::size_t i = 0; i < a.districtPolicies.size(); ++i) {
    if (!DistrictPolicyEqual(a.districtPolicies[i], b.districtPolicies[i])) return false;
  }
  return true;
}

bool TrafficModelSettingsEqual(const TrafficModelSettings& a, const TrafficModelSettings& b)
{
  return a.congestionAwareRouting == b.congestionAwareRouting && a.congestionIterations == b.congestionIterations &&
         a.congestionAlpha == b.congestionAlpha && a.congestionBeta == b.congestionBeta &&
         a.congestionCapacityScale == b.congestionCapacityScale && a.congestionRatioClamp == b.congestionRatioClamp &&
         a.capacityAwareJobs == b.capacityAwareJobs && a.jobAssignmentIterations == b.jobAssignmentIterations &&
         a.jobPenaltyBaseMilli == b.jobPenaltyBaseMilli;
}

bool TransitPlannerConfigEqual(const TransitPlannerConfig& a, const TransitPlannerConfig& b)
{
  return a.maxLines == b.maxLines && a.endpointCandidates == b.endpointCandidates && a.weightMode == b.weightMode &&
         a.demandBias == b.demandBias && a.maxDetour == b.maxDetour && a.coverFraction == b.coverFraction &&
         a.minEdgeDemand == b.minEdgeDemand && a.minLineDemand == b.minLineDemand && a.seedSalt == b.seedSalt;
}

bool TransitModelSettingsEqual(const TransitModelSettings& a, const TransitModelSettings& b)
{
  return a.enabled == b.enabled && a.serviceLevel == b.serviceLevel && a.maxModeShare == b.maxModeShare &&
         a.travelTimeMultiplier == b.travelTimeMultiplier && a.stopSpacingTiles == b.stopSpacingTiles &&
         a.costPerTile == b.costPerTile && a.costPerStop == b.costPerStop && a.demandMode == b.demandMode &&
         TransitPlannerConfigEqual(a.plannerCfg, b.plannerCfg);
}

std::uint8_t TileDiffMask(const Tile& base, const Tile& target)
{
  std::uint8_t mask = 0;
  if (base.terrain != target.terrain) mask |= static_cast<std::uint8_t>(TileFieldMask::Terrain);
  if (base.overlay != target.overlay) mask |= static_cast<std::uint8_t>(TileFieldMask::Overlay);
  if (base.height != target.height) mask |= static_cast<std::uint8_t>(TileFieldMask::Height);
  if (base.variation != target.variation) mask |= static_cast<std::uint8_t>(TileFieldMask::Variation);
  if (base.level != target.level) mask |= static_cast<std::uint8_t>(TileFieldMask::Level);
  if (base.occupants != target.occupants) mask |= static_cast<std::uint8_t>(TileFieldMask::Occupants);
  if (base.district != target.district) mask |= static_cast<std::uint8_t>(TileFieldMask::District);
  return mask;
}

} // namespace

void ReplayCapture::clear()
{
  m_active = false;
  m_replay = {};
  m_haveLastProcCfg = false;
  m_haveLastSimCfg = false;
  m_haveLastTuning = false;
}

bool ReplayCapture::startFromBaseSave(const std::vector<std::uint8_t>& baseSave, std::string& outError)
{
  outError.clear();
  clear();

  if (baseSave.empty()) {
    outError = "Empty base save";
    return false;
  }

  // Validate the save and cache its configs as the initial baseline.
  World tmpWorld;
  ProcGenConfig tmpProc;
  SimConfig tmpSim;
  if (!LoadWorldBinaryFromBytes(tmpWorld, tmpProc, tmpSim, baseSave, outError)) {
    return false;
  }

  m_replay.baseSave = baseSave;
  // Replay default version is current; keep it.

  m_lastProcCfg = tmpProc;
  m_haveLastProcCfg = true;
  m_lastSimCfg = tmpSim;
  m_haveLastSimCfg = true;
  m_haveLastTuning = false;

  m_active = true;
  return true;
}

bool ReplayCapture::startFromWorld(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg,
                                  std::string& outError)
{
  outError.clear();
  std::vector<std::uint8_t> bytes;
  if (!SaveWorldBinaryToBytes(world, procCfg, simCfg, bytes, outError)) {
    return false;
  }
  return startFromBaseSave(bytes, outError);
}

bool ReplayCapture::saveToFile(const std::string& path, std::string& outError) const
{
  outError.clear();
  if (path.empty()) {
    outError = "Empty replay path";
    return false;
  }
  if (m_replay.baseSave.empty()) {
    outError = "Replay has no base save";
    return false;
  }
  return SaveReplayBinary(m_replay, path, outError);
}

void ReplayCapture::recordTicks(std::uint32_t ticks)
{
  if (!m_active) return;
  if (ticks == 0) return;

  if (!m_replay.events.empty() && m_replay.events.back().type == ReplayEventType::Tick) {
    // Saturating add to avoid overflow.
    const std::uint32_t prev = m_replay.events.back().ticks;
    const std::uint32_t max = std::numeric_limits<std::uint32_t>::max();
    m_replay.events.back().ticks = (prev > max - ticks) ? max : (prev + ticks);
    return;
  }

  ReplayEvent e;
  e.type = ReplayEventType::Tick;
  e.ticks = ticks;
  m_replay.events.push_back(std::move(e));
}

void ReplayCapture::recordNote(const std::string& note)
{
  if (!m_active) return;
  ReplayEvent e;
  e.type = ReplayEventType::Note;
  e.note = note;
  m_replay.events.push_back(std::move(e));
}

void ReplayCapture::recordAssertHash(const World& world, bool includeStatsInHash, const std::string& label)
{
  if (!m_active) return;
  ReplayEvent e;
  e.type = ReplayEventType::AssertHash;
  e.includeStatsInHash = includeStatsInHash;
  e.expectedHash = HashWorld(world, includeStatsInHash);
  e.label = label;
  m_replay.events.push_back(std::move(e));
}

bool ReplayCapture::recordSnapshot(const std::vector<std::uint8_t>& saveBytes, std::string& outError)
{
  outError.clear();
  if (!m_active) return true;
  if (saveBytes.empty()) {
    outError = "Empty snapshot blob";
    return false;
  }
  ReplayEvent e;
  e.type = ReplayEventType::Snapshot;
  e.snapshot = saveBytes;
  m_replay.events.push_back(std::move(e));
  return true;
}

bool ReplayCapture::recordSnapshotFromWorld(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg,
                                           std::string& outError)
{
  outError.clear();
  if (!m_active) return true;
  std::vector<std::uint8_t> bytes;
  if (!SaveWorldBinaryToBytes(world, procCfg, simCfg, bytes, outError)) {
    return false;
  }
  return recordSnapshot(bytes, outError);
}

bool ReplayCapture::recordConfigPatch(const World& world, const ProcGenConfig* procCfg, const SimConfig* simCfg,
                                     std::string& outError)
{
  outError.clear();
  if (!m_active) return true;
  if (!procCfg && !simCfg) return true;

  WorldPatch patch;
  patch.width = world.width();
  patch.height = world.height();

  // Config-only patch; do not bind it to Stats (keeps it robust to derived-stat drift).
  patch.includeStats = false;
  patch.stats = {};
  const std::uint64_t h = HashWorld(world, false);
  patch.baseHash = h;
  patch.targetHash = h;

  patch.includeProcCfg = procCfg != nullptr;
  patch.includeSimCfg = simCfg != nullptr;
  if (patch.includeProcCfg) patch.procCfg = *procCfg;
  if (patch.includeSimCfg) patch.simCfg = *simCfg;

  std::vector<std::uint8_t> bytes;
  if (!SerializeWorldPatchBinary(patch, bytes, outError, WorldPatchCompression::SLLZ)) {
    return false;
  }

  ReplayEvent e;
  e.type = ReplayEventType::Patch;
  e.patch = std::move(bytes);
  m_replay.events.push_back(std::move(e));
  return true;
}

void ReplayCapture::recordSimTuning(const TrafficModelSettings& trafficModel, const TransitModelSettings& transitModel)
{
  if (!m_active) return;
  ReplayEvent e;
  e.type = ReplayEventType::SimTuning;
  e.trafficModel = trafficModel;
  e.transitModel = transitModel;
  m_replay.events.push_back(std::move(e));
}

bool ReplayCapture::captureSettingsIfChanged(const World& world, const ProcGenConfig& procCfg, const Simulator& sim,
                                            std::string& outError)
{
  outError.clear();
  if (!m_active) return true;

  const bool procChanged = !m_haveLastProcCfg || !ProcGenConfigEqual(procCfg, m_lastProcCfg);
  const bool simChanged = !m_haveLastSimCfg || !SimConfigEqual(sim.config(), m_lastSimCfg);

  if (procChanged || simChanged) {
    if (!recordConfigPatch(world, procChanged ? &procCfg : nullptr, simChanged ? &sim.config() : nullptr, outError)) {
      return false;
    }
    if (procChanged) {
      m_lastProcCfg = procCfg;
      m_haveLastProcCfg = true;
    }
    if (simChanged) {
      m_lastSimCfg = sim.config();
      m_haveLastSimCfg = true;
    }
  }

  const bool tuningChanged = !m_haveLastTuning || !TrafficModelSettingsEqual(sim.trafficModel(), m_lastTrafficModel) ||
                             !TransitModelSettingsEqual(sim.transitModel(), m_lastTransitModel);

  if (tuningChanged) {
    recordSimTuning(sim.trafficModel(), sim.transitModel());
    m_lastTrafficModel = sim.trafficModel();
    m_lastTransitModel = sim.transitModel();
    m_haveLastTuning = true;
  }

  return true;
}

bool ReplayCapture::recordTileCommandPatch(const World& worldAfter, const EditHistory::Command& cmd, std::uint64_t baseHash,
                                          bool useBeforeAsTarget, std::string& outError)
{
  outError.clear();
  if (!m_active) return true;

  if (cmd.tiles.empty() && cmd.moneyDelta == 0) return true;

  WorldPatch patch;
  patch.width = worldAfter.width();
  patch.height = worldAfter.height();
  patch.includeProcCfg = false;
  patch.includeSimCfg = false;
  patch.includeStats = true;
  patch.stats = worldAfter.stats();
  patch.baseHash = baseHash;
  patch.targetHash = HashWorld(worldAfter, true);

  patch.tiles.reserve(cmd.tiles.size());

  for (const EditHistory::TileChange& c : cmd.tiles) {
    if (!worldAfter.inBounds(c.x, c.y)) continue;
    const std::uint32_t idx = static_cast<std::uint32_t>(c.y * patch.width + c.x);

    const Tile& baseTile = useBeforeAsTarget ? c.after : c.before;
    const Tile& targetTile = useBeforeAsTarget ? c.before : c.after;
    const std::uint8_t mask = TileDiffMask(baseTile, targetTile);
    if (mask == 0) continue;

    WorldPatchTileDelta d;
    d.index = idx;
    d.mask = mask;
    d.value = targetTile;
    patch.tiles.push_back(std::move(d));
  }

  std::sort(patch.tiles.begin(), patch.tiles.end(), [](const WorldPatchTileDelta& a, const WorldPatchTileDelta& b) {
    return a.index < b.index;
  });

  std::vector<std::uint8_t> bytes;
  if (!SerializeWorldPatchBinary(patch, bytes, outError, WorldPatchCompression::SLLZ)) {
    return false;
  }

  ReplayEvent e;
  e.type = ReplayEventType::Patch;
  e.patch = std::move(bytes);
  m_replay.events.push_back(std::move(e));
  return true;
}

} // namespace isocity
