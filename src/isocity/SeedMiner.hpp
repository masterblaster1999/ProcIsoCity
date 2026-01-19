#pragma once

#include "isocity/DepressionFill.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Seed mining ("city mining")
//
// ProcIsoCity is a procedural generator; sometimes interesting worlds are rare
// seeds that produce unusually good (or unusually chaotic) outcomes under the
// simulator. The seed miner runs a batch of worlds, simulates them for N days,
// computes KPI metrics, and ranks seeds by an objective score.
//
// This module intentionally lives in isocity_core (no raylib) so both headless
// tools and the interactive game can reuse the same mining implementation.
// -----------------------------------------------------------------------------

enum class MineObjective : std::uint8_t {
  Balanced = 0,
  Growth = 1,
  Resilient = 2,
  Chaos = 3,
};

const char* MineObjectiveName(MineObjective o);

// Parse objective from a string (case-insensitive). Accepts common aliases.
bool ParseMineObjective(const std::string& s, MineObjective& out);

// Utility: format as a hex string with 0x prefix (fixed width 16).
std::string HexU64(std::uint64_t v);

struct MineRecord {
  std::uint64_t seed = 0;
  int w = 0;
  int h = 0;

  Stats stats{};

  // Tile counts.
  int waterTiles = 0;
  int roadTiles = 0;
  int resTiles = 0;
  int comTiles = 0;
  int indTiles = 0;
  int parkTiles = 0;

  int schoolTiles = 0;
  int hospitalTiles = 0;
  int policeTiles = 0;
  int fireTiles = 0;

  double waterFrac = 0.0;
  double roadFrac = 0.0;
  double zoneFrac = 0.0;
  double parkFrac = 0.0;

  // Hydrology.
  int seaFloodCells = 0;
  double seaFloodFrac = 0.0;
  double seaMaxDepth = 0.0;

  int pondCells = 0;
  double pondFrac = 0.0;
  double pondMaxDepth = 0.0;
  double pondVolume = 0.0;

  // Objective score.
  double score = 0.0;
};

struct MineConfig {
  std::uint64_t seedStart = 1;
  std::uint64_t seedStep = 1;
  int samples = 100;

  int w = 96;
  int h = 96;
  int days = 120;

  MineObjective objective = MineObjective::Balanced;

  // Hydrology analysis (sea flooding + depression filling).
  bool hydrologyEnabled = true;

  // If finite, overrides procCfg.waterLevel.
  float seaLevelOverride = std::numeric_limits<float>::quiet_NaN();

  // Sea flood connectivity options.
  bool seaRequireEdgeConnection = true;
  bool seaEightConnected = false;

  // Priority-Flood depression fill epsilon (0 preserves perfectly flat spill surfaces).
  float depressionEpsilon = 0.0f;
};

struct MineProgress {
  // 0-based index of the record that was just produced.
  int index = 0;
  int total = 0;
  const MineRecord* record = nullptr;
};

using MineProgressFn = std::function<void(const MineProgress&)>;

// Batch mine seeds according to cfg, using ProcGenConfig and SimConfig.
//
// If progress is provided, it is called after each record is produced.
std::vector<MineRecord> MineSeeds(const MineConfig& cfg,
                                 const ProcGenConfig& procCfg,
                                 const SimConfig& simCfg,
                                 const MineProgressFn& progress = MineProgressFn());

// Select top-K indices in recs. If diverse is true, selects a diverse subset
// using a simple Maximal Marginal Relevance (MMR) heuristic.
std::vector<int> SelectTopIndices(const std::vector<MineRecord>& recs,
                                 int topK,
                                 bool diverse = true,
                                 int candidatePool = 0,
                                 double mmrScoreWeight = 0.70);

// Incremental miner suitable for UI integration (spread work across frames).
class MineSession {
public:
  MineSession(MineConfig cfg, ProcGenConfig procCfg, SimConfig simCfg);

  // Mine up to maxSteps seeds; returns the number of records produced.
  int step(int maxSteps = 1, const MineProgressFn& progress = MineProgressFn());

  bool done() const { return m_index >= m_cfg.samples; }
  int index() const { return m_index; }
  int total() const { return m_cfg.samples; }

  const MineConfig& config() const { return m_cfg; }
  const ProcGenConfig& procConfig() const { return m_procCfg; }
  const SimConfig& simConfig() const { return m_simCfg; }

  const std::vector<MineRecord>& records() const { return m_records; }
  std::vector<MineRecord>& records() { return m_records; }

private:
  MineConfig m_cfg{};
  ProcGenConfig m_procCfg{};
  SimConfig m_simCfg{};

  Simulator m_sim;

  float m_seaLevel = 0.0f;
  SeaFloodConfig m_seaCfg{};
  DepressionFillConfig m_depCfg{};

  int m_index = 0;
  std::vector<MineRecord> m_records;
};

// CSV helpers.
void WriteMineCsvHeader(std::ostream& os);
void WriteMineCsvRow(std::ostream& os, const MineRecord& r);

// JSON helper for a MineRecord (for embedding into larger documents).
JsonValue MineRecordToJson(const MineRecord& r);

} // namespace isocity
