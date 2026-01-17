#pragma once

#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// A self-contained, deterministic "journal" format for reproducing a session.
//
// A replay embeds a base save (the same bytes produced by SaveWorldBinary) and
// a sequence of events:
//   - Tick: advance the simulation by N ticks
//   - Patch: apply an ISOPATCH blob (WorldPatch binary)
//   - Snapshot: replace the whole world with another embedded save blob
//   - SimTuning: set non-persistent runtime tuning (traffic/transit/services/trade/economy model settings)
//
// This is primarily intended for debugging/regression: you can ship a single
// .isoreplay file that deterministically rebuilds a city state.

enum class ReplayEventType : std::uint8_t {
  Tick = 0,
  Patch = 1,
  Snapshot = 2,
  // A human-readable note/marker embedded in the replay (UTF-8).
  // Has no effect on playback.
  Note = 3,
  // Assert that the current world hash matches an expected value.
  // Useful for regression testing and deterministic playback verification.
  AssertHash = 4,
  // Set non-persistent runtime simulation tuning (traffic/transit/trade/services/economy model settings).
  // This exists because these settings are intentionally not part of SimConfig
  // (and therefore not stored in saves).
  SimTuning = 5,
};

struct ReplayEvent {
  ReplayEventType type = ReplayEventType::Tick;

  // Tick: number of Simulator::stepOnce() calls.
  std::uint32_t ticks = 0;

  // Patch: raw ISOPATCH bytes.
  std::vector<std::uint8_t> patch;

  // Snapshot: raw ISOCITY save bytes.
  std::vector<std::uint8_t> snapshot;

  // SimTuning: runtime model settings (not persisted in SimConfig/saves).
  TrafficModelSettings trafficModel{};
  TransitModelSettings transitModel{};
  TradeModelSettings tradeModel{};
  ServicesModelSettings servicesModel{};
  EconomyModelSettings economyModel{};

  // Note: UTF-8 text.
  std::string note;

  // AssertHash:
  // Hash value and flags are interpreted as:
  //   HashWorld(world, includeStats)
  std::uint64_t expectedHash = 0;
  bool includeStatsInHash = true;
  // Optional label for nicer error messages.
  std::string label;
};

struct Replay {
  // On-disk format version.
  // v1: base save blob + events until EOF (Tick/Patch/Snapshot only)
  // v2: adds explicit eventCount + new event types (Note, AssertHash)
  // v3: adds SimTuning events (non-persistent runtime tuning)
  // v4: extends SimTuning to include trade/services/economy model settings
  std::uint32_t version = 4;

  // Base save bytes (ISOCITY binary save format).
  std::vector<std::uint8_t> baseSave;

  std::vector<ReplayEvent> events;
};

// Save/load a replay to disk.
bool SaveReplayBinary(const Replay& replay, const std::string& path, std::string& outError);
bool LoadReplayBinary(Replay& outReplay, const std::string& path, std::string& outError);

// Run a replay and return the final world+configs.
//
// If strictPatches==true, patch events must match their recorded base hashes.
// If strictAsserts==true, AssertHash events must match the computed hash.
bool PlayReplay(const Replay& replay, World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg,
                std::string& outError, bool strictPatches = true, bool strictAsserts = true,
                std::vector<Stats>* outTickStats = nullptr);

} // namespace isocity
