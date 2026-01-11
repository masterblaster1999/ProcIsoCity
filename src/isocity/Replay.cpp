#include "isocity/Replay.hpp"

#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/WorldPatch.hpp"

#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace isocity {

namespace {

constexpr std::uint8_t kMagic[8] = {'I','S','O','R','E','P','L','\0'};
constexpr std::uint32_t kReplayVersionV1 = 1;
constexpr std::uint32_t kReplayVersionV2 = 2;
constexpr std::uint32_t kReplayVersionV3 = 3;
constexpr std::uint32_t kReplayVersion = kReplayVersionV3;

bool ReadExact(std::istream& in, void* dst, std::size_t n)
{
  in.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
  return in.good();
}

bool WriteExact(std::ostream& out, const void* src, std::size_t n)
{
  out.write(reinterpret_cast<const char*>(src), static_cast<std::streamsize>(n));
  return out.good();
}

template <typename T>
bool ReadPOD(std::istream& in, T& out)
{
  return ReadExact(in, &out, sizeof(out));
}

template <typename T>
bool WritePOD(std::ostream& out, const T& v)
{
  return WriteExact(out, &v, sizeof(v));
}

bool ReadBytes(std::istream& in, std::vector<std::uint8_t>& outBytes, std::uint32_t n)
{
  outBytes.clear();
  outBytes.resize(n);
  if (n == 0) return true;
  return ReadExact(in, outBytes.data(), outBytes.size());
}

bool WriteBytes(std::ostream& out, const std::vector<std::uint8_t>& bytes)
{
  if (bytes.empty()) return true;
  return WriteExact(out, bytes.data(), bytes.size());
}

bool ReadString(std::istream& in, std::string& out, std::uint32_t n)
{
  out.clear();
  if (n == 0) return true;
  out.resize(n);
  return ReadExact(in, out.data(), out.size());
}

bool WriteString(std::ostream& out, const std::string& s)
{
  if (s.empty()) return true;
  return WriteExact(out, s.data(), s.size());
}

bool WriteTrafficModelSettings(std::ostream& out, const TrafficModelSettings& s)
{
  const std::uint8_t cong = s.congestionAwareRouting ? 1u : 0u;
  if (!WritePOD(out, cong)) return false;
  if (!WritePOD(out, s.congestionIterations)) return false;
  if (!WritePOD(out, s.congestionAlpha)) return false;
  if (!WritePOD(out, s.congestionBeta)) return false;
  if (!WritePOD(out, s.congestionCapacityScale)) return false;
  if (!WritePOD(out, s.congestionRatioClamp)) return false;

  const std::uint8_t capJobs = s.capacityAwareJobs ? 1u : 0u;
  if (!WritePOD(out, capJobs)) return false;
  if (!WritePOD(out, s.jobAssignmentIterations)) return false;
  if (!WritePOD(out, s.jobPenaltyBaseMilli)) return false;
  return out.good();
}

bool ReadTrafficModelSettings(std::istream& in, TrafficModelSettings& s)
{
  std::uint8_t b = 0;
  if (!ReadPOD(in, b)) return false;
  s.congestionAwareRouting = (b & 1u) != 0u;
  if (!ReadPOD(in, s.congestionIterations)) return false;
  if (!ReadPOD(in, s.congestionAlpha)) return false;
  if (!ReadPOD(in, s.congestionBeta)) return false;
  if (!ReadPOD(in, s.congestionCapacityScale)) return false;
  if (!ReadPOD(in, s.congestionRatioClamp)) return false;
  if (!ReadPOD(in, b)) return false;
  s.capacityAwareJobs = (b & 1u) != 0u;
  if (!ReadPOD(in, s.jobAssignmentIterations)) return false;
  if (!ReadPOD(in, s.jobPenaltyBaseMilli)) return false;
  return in.good();
}

bool WriteTransitPlannerConfig(std::ostream& out, const TransitPlannerConfig& c)
{
  if (!WritePOD(out, c.maxLines)) return false;
  if (!WritePOD(out, c.endpointCandidates)) return false;
  const std::uint8_t wm = static_cast<std::uint8_t>(c.weightMode);
  if (!WritePOD(out, wm)) return false;
  if (!WritePOD(out, c.demandBias)) return false;
  if (!WritePOD(out, c.maxDetour)) return false;
  if (!WritePOD(out, c.coverFraction)) return false;
  if (!WritePOD(out, c.minEdgeDemand)) return false;
  if (!WritePOD(out, c.minLineDemand)) return false;
  if (!WritePOD(out, c.seedSalt)) return false;
  return out.good();
}

bool ReadTransitPlannerConfig(std::istream& in, TransitPlannerConfig& c)
{
  if (!ReadPOD(in, c.maxLines)) return false;
  if (!ReadPOD(in, c.endpointCandidates)) return false;
  std::uint8_t wm = 0;
  if (!ReadPOD(in, wm)) return false;
  c.weightMode = static_cast<TransitEdgeWeightMode>(wm);
  if (!ReadPOD(in, c.demandBias)) return false;
  if (!ReadPOD(in, c.maxDetour)) return false;
  if (!ReadPOD(in, c.coverFraction)) return false;
  if (!ReadPOD(in, c.minEdgeDemand)) return false;
  if (!ReadPOD(in, c.minLineDemand)) return false;
  if (!ReadPOD(in, c.seedSalt)) return false;
  return in.good();
}

bool WriteTransitModelSettings(std::ostream& out, const TransitModelSettings& s)
{
  const std::uint8_t enabled = s.enabled ? 1u : 0u;
  if (!WritePOD(out, enabled)) return false;
  if (!WritePOD(out, s.serviceLevel)) return false;
  if (!WritePOD(out, s.maxModeShare)) return false;
  if (!WritePOD(out, s.travelTimeMultiplier)) return false;
  if (!WritePOD(out, s.stopSpacingTiles)) return false;
  if (!WritePOD(out, s.costPerTile)) return false;
  if (!WritePOD(out, s.costPerStop)) return false;
  const std::uint8_t dm = static_cast<std::uint8_t>(s.demandMode);
  if (!WritePOD(out, dm)) return false;
  if (!WriteTransitPlannerConfig(out, s.plannerCfg)) return false;
  return out.good();
}

bool ReadTransitModelSettings(std::istream& in, TransitModelSettings& s)
{
  std::uint8_t enabled = 0;
  if (!ReadPOD(in, enabled)) return false;
  s.enabled = (enabled & 1u) != 0u;
  if (!ReadPOD(in, s.serviceLevel)) return false;
  if (!ReadPOD(in, s.maxModeShare)) return false;
  if (!ReadPOD(in, s.travelTimeMultiplier)) return false;
  if (!ReadPOD(in, s.stopSpacingTiles)) return false;
  if (!ReadPOD(in, s.costPerTile)) return false;
  if (!ReadPOD(in, s.costPerStop)) return false;
  std::uint8_t dm = 0;
  if (!ReadPOD(in, dm)) return false;
  s.demandMode = static_cast<TransitDemandMode>(dm);
  if (!ReadTransitPlannerConfig(in, s.plannerCfg)) return false;
  return in.good();
}

// Load an embedded save blob into a World.
//
// SaveLoad now supports in-memory loading, so replay playback can avoid temp files.
bool LoadWorldFromSaveBytes(const std::vector<std::uint8_t>& saveBytes, World& outWorld, ProcGenConfig& outProc,
                            SimConfig& outSim, std::string& outError)
{
  outError.clear();
  if (saveBytes.empty()) {
    outError = "Embedded save blob is empty";
    return false;
  }

  return LoadWorldBinaryFromBytes(outWorld, outProc, outSim, saveBytes, outError);
}


} // namespace

bool SaveReplayBinary(const Replay& replay, const std::string& path, std::string& outError)
{
  outError.clear();

  if (path.empty()) {
    outError = "Empty replay path";
    return false;
  }
  if (replay.baseSave.empty()) {
    outError = "Replay has no base save";
    return false;
  }

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    outError = "Unable to open replay for writing: " + path;
    return false;
  }

  if (!WriteExact(out, kMagic, sizeof(kMagic))) {
    outError = "Write failed (magic)";
    return false;
  }

  const std::uint32_t version = (replay.version == 0) ? kReplayVersion : replay.version;
  if (version != kReplayVersionV1 && version != kReplayVersionV2 && version != kReplayVersionV3) {
    outError = "Unsupported replay version (writer)";
    return false;
  }
  if (!WritePOD(out, version)) {
    outError = "Write failed (version)";
    return false;
  }

  if (replay.baseSave.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "Base save blob is too large";
    return false;
  }
  const std::uint32_t baseSize = static_cast<std::uint32_t>(replay.baseSave.size());
  if (!WritePOD(out, baseSize) || !WriteBytes(out, replay.baseSave)) {
    outError = "Write failed (base save blob)";
    return false;
  }

  // v2+ adds an explicit event count.
  if (version >= kReplayVersionV2) {
    if (replay.events.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Replay has too many events";
      return false;
    }
    const std::uint32_t eventCount = static_cast<std::uint32_t>(replay.events.size());
    if (!WritePOD(out, eventCount)) {
      outError = "Write failed (event count)";
      return false;
    }
  }

  for (const ReplayEvent& e : replay.events) {
    const std::uint8_t type = static_cast<std::uint8_t>(e.type);
    if (!WritePOD(out, type)) {
      outError = "Write failed (event type)";
      return false;
    }

    switch (e.type) {
      case ReplayEventType::Tick: {
        if (!WritePOD(out, e.ticks)) {
          outError = "Write failed (tick event)";
          return false;
        }
        break;
      }
      case ReplayEventType::Patch:
      case ReplayEventType::Snapshot: {
        const std::vector<std::uint8_t>& blob = (e.type == ReplayEventType::Patch) ? e.patch : e.snapshot;
        if (blob.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
          outError = "Replay event blob is too large";
          return false;
        }
        const std::uint32_t sz = static_cast<std::uint32_t>(blob.size());
        if (!WritePOD(out, sz) || !WriteBytes(out, blob)) {
          outError = "Write failed (event blob)";
          return false;
        }
        break;
      }
      case ReplayEventType::Note: {
        if (version < kReplayVersionV2) {
          outError = "Replay Note events require replay v2+";
          return false;
        }
        if (e.note.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
          outError = "Replay note string is too large";
          return false;
        }
        const std::uint32_t sz = static_cast<std::uint32_t>(e.note.size());
        if (!WritePOD(out, sz) || !WriteString(out, e.note)) {
          outError = "Write failed (note event)";
          return false;
        }
        break;
      }
      case ReplayEventType::AssertHash: {
        if (version < kReplayVersionV2) {
          outError = "Replay AssertHash events require replay v2+";
          return false;
        }
        if (!WritePOD(out, e.expectedHash)) {
          outError = "Write failed (assert expected hash)";
          return false;
        }
        const std::uint8_t flags = e.includeStatsInHash ? 1u : 0u;
        if (!WritePOD(out, flags)) {
          outError = "Write failed (assert flags)";
          return false;
        }
        if (e.label.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
          outError = "Replay assert label is too large";
          return false;
        }
        const std::uint32_t sz = static_cast<std::uint32_t>(e.label.size());
        if (!WritePOD(out, sz) || !WriteString(out, e.label)) {
          outError = "Write failed (assert label)";
          return false;
        }
        break;
      }
      case ReplayEventType::SimTuning: {
        if (version < kReplayVersionV3) {
          outError = "Replay SimTuning events require replay v3+";
          return false;
        }
        if (!WriteTrafficModelSettings(out, e.trafficModel)) {
          outError = "Write failed (sim tuning traffic settings)";
          return false;
        }
        if (!WriteTransitModelSettings(out, e.transitModel)) {
          outError = "Write failed (sim tuning transit settings)";
          return false;
        }
        break;
      }
    }
  }

  return true;
}

bool LoadReplayBinary(Replay& outReplay, const std::string& path, std::string& outError)
{
  outError.clear();
  outReplay = {};

  if (path.empty()) {
    outError = "Empty replay path";
    return false;
  }

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    outError = "Unable to open replay for reading: " + path;
    return false;
  }

  std::uint8_t magic[8]{};
  if (!ReadExact(in, magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "Bad replay magic (not an ISOREPL file)";
    return false;
  }

  std::uint32_t version = 0;
  if (!ReadPOD(in, version)) {
    outError = "Read failed (version)";
    return false;
  }
  if (version != kReplayVersionV1 && version != kReplayVersionV2 && version != kReplayVersionV3) {
    outError = "Unsupported replay version";
    return false;
  }
  outReplay.version = version;

  std::uint32_t baseSize = 0;
  if (!ReadPOD(in, baseSize)) {
    outError = "Read failed (base size)";
    return false;
  }
  if (!ReadBytes(in, outReplay.baseSave, baseSize)) {
    outError = "Read failed (base save blob)";
    return false;
  }
  if (outReplay.baseSave.empty()) {
    outError = "Replay contains an empty base save blob";
    return false;
  }

  auto readOneEventV1 = [&](ReplayEvent& outEv) -> bool {
    std::uint8_t typeU8 = 0;
    in.read(reinterpret_cast<char*>(&typeU8), 1);
    if (in.eof()) {
      return false; // normal EOF
    }
    if (!in.good()) {
      outError = "Read failed (event type)";
      return false;
    }

    const ReplayEventType type = static_cast<ReplayEventType>(typeU8);
    if (type != ReplayEventType::Tick && type != ReplayEventType::Patch && type != ReplayEventType::Snapshot) {
      outError = "Unknown replay event type";
      return false;
    }

    ReplayEvent e;
    e.type = type;

    if (type == ReplayEventType::Tick) {
      if (!ReadPOD(in, e.ticks)) {
        outError = "Read failed (tick event)";
        return false;
      }
    } else {
      std::uint32_t sz = 0;
      if (!ReadPOD(in, sz)) {
        outError = "Read failed (event blob size)";
        return false;
      }

      std::vector<std::uint8_t> blob;
      if (!ReadBytes(in, blob, sz)) {
        outError = "Read failed (event blob)";
        return false;
      }

      if (type == ReplayEventType::Patch) {
        e.patch = std::move(blob);
      } else {
        e.snapshot = std::move(blob);
      }
    }

    outEv = std::move(e);
    return true;
  };

  if (version == kReplayVersionV1) {
    // v1: events until EOF.
    while (true) {
      ReplayEvent e;
      if (!readOneEventV1(e)) {
        if (outError.empty()) break; // clean EOF
        return false;
      }
      outReplay.events.push_back(std::move(e));
    }
    return true;
  }

  // v2+: explicit event count.
  std::uint32_t eventCount = 0;
  if (!ReadPOD(in, eventCount)) {
    outError = "Read failed (event count)";
    return false;
  }

  outReplay.events.reserve(eventCount);
  for (std::uint32_t i = 0; i < eventCount; ++i) {
    std::uint8_t typeU8 = 0;
    if (!ReadPOD(in, typeU8)) {
      outError = "Read failed (event type)";
      return false;
    }

    const ReplayEventType type = static_cast<ReplayEventType>(typeU8);
    const bool typeAllowed =
      (type == ReplayEventType::Tick) ||
      (type == ReplayEventType::Patch) ||
      (type == ReplayEventType::Snapshot) ||
      ((version >= kReplayVersionV2) && (type == ReplayEventType::Note || type == ReplayEventType::AssertHash)) ||
      ((version >= kReplayVersionV3) && (type == ReplayEventType::SimTuning));
    if (!typeAllowed) {
      outError = "Unknown replay event type";
      return false;
    }

    ReplayEvent e;
    e.type = type;

    switch (type) {
      case ReplayEventType::Tick: {
        if (!ReadPOD(in, e.ticks)) {
          outError = "Read failed (tick event)";
          return false;
        }
        break;
      }
      case ReplayEventType::Patch:
      case ReplayEventType::Snapshot: {
        std::uint32_t sz = 0;
        if (!ReadPOD(in, sz)) {
          outError = "Read failed (event blob size)";
          return false;
        }

        std::vector<std::uint8_t> blob;
        if (!ReadBytes(in, blob, sz)) {
          outError = "Read failed (event blob)";
          return false;
        }

        if (type == ReplayEventType::Patch) {
          e.patch = std::move(blob);
        } else {
          e.snapshot = std::move(blob);
        }
        break;
      }
      case ReplayEventType::Note: {
        std::uint32_t sz = 0;
        if (!ReadPOD(in, sz)) {
          outError = "Read failed (note size)";
          return false;
        }
        if (!ReadString(in, e.note, sz)) {
          outError = "Read failed (note bytes)";
          return false;
        }
        break;
      }
      case ReplayEventType::AssertHash: {
        if (!ReadPOD(in, e.expectedHash)) {
          outError = "Read failed (assert expected hash)";
          return false;
        }
        std::uint8_t flags = 0;
        if (!ReadPOD(in, flags)) {
          outError = "Read failed (assert flags)";
          return false;
        }
        e.includeStatsInHash = (flags & 1u) != 0u;
        std::uint32_t sz = 0;
        if (!ReadPOD(in, sz)) {
          outError = "Read failed (assert label size)";
          return false;
        }
        if (!ReadString(in, e.label, sz)) {
          outError = "Read failed (assert label bytes)";
          return false;
        }
        break;
      }
      case ReplayEventType::SimTuning: {
        if (version < kReplayVersionV3) {
          outError = "Replay SimTuning events require replay v3+";
          return false;
        }
        if (!ReadTrafficModelSettings(in, e.trafficModel)) {
          outError = "Read failed (sim tuning traffic settings)";
          return false;
        }
        if (!ReadTransitModelSettings(in, e.transitModel)) {
          outError = "Read failed (sim tuning transit settings)";
          return false;
        }
        break;
      }
    }

    outReplay.events.push_back(std::move(e));
  }

  return true;
}

bool PlayReplay(const Replay& replay, World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg,
                std::string& outError, bool strictPatches, bool strictAsserts, std::vector<Stats>* outTickStats)
{
  outError.clear();

  if (outTickStats) outTickStats->clear();

  if (replay.baseSave.empty()) {
    outError = "Replay has no base save";
    return false;
  }

  if (!LoadWorldFromSaveBytes(replay.baseSave, outWorld, outProcCfg, outSimCfg, outError)) {
    return false;
  }

  if (outTickStats) outTickStats->push_back(outWorld.stats());

  Simulator sim(outSimCfg);
  sim.resetTimer();

  // These settings are intentionally not part of SimConfig (and therefore not
  // stored in saves). Replays restore them via SimTuning events.
  TrafficModelSettings trafficModel = sim.trafficModel();
  TransitModelSettings transitModel = sim.transitModel();

  auto applyTuning = [&]() {
    sim.trafficModel() = trafficModel;
    sim.transitModel() = transitModel;
  };
  applyTuning();

  for (std::size_t i = 0; i < replay.events.size(); ++i) {
    const ReplayEvent& e = replay.events[i];
    switch (e.type) {
      case ReplayEventType::Tick: {
        for (std::uint32_t t = 0; t < e.ticks; ++t) {
          sim.stepOnce(outWorld);
          if (outTickStats) outTickStats->push_back(outWorld.stats());
        }
        break;
      }
      case ReplayEventType::Patch: {
        WorldPatch patch;
        if (!DeserializeWorldPatchBinary(patch, e.patch.data(), e.patch.size(), outError)) {
          outError = "Replay patch event " + std::to_string(i) + " failed to parse: " + outError;
          return false;
        }
        const bool force = !strictPatches;
        if (!ApplyWorldPatch(outWorld, outProcCfg, outSimCfg, patch, outError, force)) {
          outError = "Replay patch event " + std::to_string(i) + " failed to apply: " + outError;
          return false;
        }
        sim.config() = outSimCfg;
        applyTuning();
        sim.resetTimer();
        break;
      }
      case ReplayEventType::Snapshot: {
        if (!LoadWorldFromSaveBytes(e.snapshot, outWorld, outProcCfg, outSimCfg, outError)) {
          outError = "Replay snapshot event " + std::to_string(i) + " failed to load: " + outError;
          return false;
        }
        sim = Simulator(outSimCfg);
        applyTuning();
        sim.resetTimer();
        if (outTickStats) outTickStats->push_back(outWorld.stats());
        break;
      }
      case ReplayEventType::SimTuning: {
        trafficModel = e.trafficModel;
        transitModel = e.transitModel;
        applyTuning();
        sim.resetTimer();
        break;
      }
      case ReplayEventType::Note: {
        // Metadata only.
        break;
      }
      case ReplayEventType::AssertHash: {
        if (!strictAsserts) break;

        const std::uint64_t got = HashWorld(outWorld, e.includeStatsInHash);
        if (got != e.expectedHash) {
          std::ostringstream oss;
          oss << "Replay assert failed at event " << i << ": expected 0x" << std::hex << e.expectedHash
              << " got 0x" << got;
          if (!e.label.empty()) oss << " (" << e.label << ")";
          outError = oss.str();
          return false;
        }
        break;
      }
    }
  }

  return true;
}

} // namespace isocity
