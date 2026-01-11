#pragma once

#include "isocity/EditHistory.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Replay.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// High-level helper for building deterministic replays from in-game actions.
//
// This lives above the low-level Replay/WorldPatch serialization and provides
// a small API that can be called from the game layer (edit strokes, undo/redo,
// sim ticks, settings changes).
//
// NOTE: This class only *captures* data; starting/stopping capture and exposing
// it via UI/console is intentionally handled elsewhere.
class ReplayCapture {
public:
  ReplayCapture() = default;

  void clear();

  bool active() const { return m_active; }

  // Start a new capture from an existing base save blob (raw bytes produced by
  // SaveWorldBinaryToBytes). This validates that the save can be loaded.
  bool startFromBaseSave(const std::vector<std::uint8_t>& baseSave, std::string& outError);

  // Convenience: start from an in-memory save of the given world.
  bool startFromWorld(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg, std::string& outError);

  // Stop capturing. The captured replay remains accessible via replay().
  void stop() { m_active = false; }

  const Replay& replay() const { return m_replay; }
  Replay& replay() { return m_replay; }

  // Persist the captured replay to disk.
  bool saveToFile(const std::string& path, std::string& outError) const;

  // --- Event recorders ---

  // Coalesces consecutive Tick events.
  void recordTicks(std::uint32_t ticks);

  void recordNote(const std::string& note);

  void recordAssertHash(const World& world, bool includeStatsInHash = true, const std::string& label = {});

  bool recordSnapshot(const std::vector<std::uint8_t>& saveBytes, std::string& outError);

  bool recordSnapshotFromWorld(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg,
                               std::string& outError);

  // Record a Patch event from an EditHistory command.
  //
  // baseHash must be the world hash *before* applying the command.
  //
  // useBeforeAsTarget:
  //   false => forward patch (before -> after)
  //   true  => undo patch (after -> before)
  bool recordTileCommandPatch(const World& worldAfter, const EditHistory::Command& cmd, std::uint64_t baseHash,
                              bool useBeforeAsTarget, std::string& outError);

  // Capture ProcGenConfig / SimConfig / runtime tuning changes compared to the
  // last captured values.
  //
  // - ProcGenConfig/SimConfig changes are recorded as "config-only" Patch events
  //   (WorldPatch with zero tile deltas).
  // - Traffic/Transit model changes are recorded as SimTuning events.
  bool captureSettingsIfChanged(const World& world, const ProcGenConfig& procCfg, const Simulator& sim,
                                std::string& outError);

private:
  bool recordConfigPatch(const World& world, const ProcGenConfig* procCfg, const SimConfig* simCfg, std::string& outError);
  void recordSimTuning(const TrafficModelSettings& trafficModel, const TransitModelSettings& transitModel);

  bool m_active = false;
  Replay m_replay{};

  bool m_haveLastProcCfg = false;
  ProcGenConfig m_lastProcCfg{};
  bool m_haveLastSimCfg = false;
  SimConfig m_lastSimCfg{};

  bool m_haveLastTuning = false;
  TrafficModelSettings m_lastTrafficModel{};
  TransitModelSettings m_lastTransitModel{};
};

} // namespace isocity
