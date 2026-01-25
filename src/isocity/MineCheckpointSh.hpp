#pragma once

#include "isocity/ConfigIO.hpp"
#include "isocity/Json.hpp"
#include "isocity/SeedMiner.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Successive-halving mining checkpoint (JSONL)
//
// Standard mining can be checkpointed/resumed via MineCheckpoint.*.
// Successive-halving mining (multi-fidelity) needs additional metadata:
//   - the stage schedule (days/keep)
//   - the stage-to-stage selection parameters (MMR/diversity mode)
//   - per-record stage indexing
//
// This module implements a small staged JSONL format so long-running "--sh"
// mining runs can be safely resumed.
//
// Format: JSON Lines (one JSON object per line).
//  - Line 0: header
//      {
//        "type":"procisocity_mine_checkpoint_sh",
//        "version":1,
//        "mine":{...},
//        "proc":{...},
//        "sim":{...},
//        "sh":{
//          "spec":"30:500,90:150,160:50",
//          "stages":[{"days":30,"keep":500},...],
//          "diverse":true,
//          "candidatePool":0,
//          "mmrScoreWeight":0.6,
//          "diversityMode":"hybrid",
//          "layoutWeight":0.5
//        }
//      }
//  - Line N>0: records
//      {"type":"record","stage":0,"index":123,"record":{...}}
//
// The stage's candidate ordering is defined by the mining pipeline:
//  - stage 0 uses MineSeedForSample(cfg, i)
//  - stage s>0 uses sorted kept seeds from stage s-1
// -----------------------------------------------------------------------------

struct SuccessiveHalvingStage {
  int days = 0;
  int keep = 0;
};

struct MineCheckpointShConfig {
  std::string spec; // Original user spec string (for readability).
  std::vector<SuccessiveHalvingStage> stages;

  bool diverse = true;
  int candidatePool = 0;
  double mmrScoreWeight = 0.60;
  MineDiversityMode diversityMode = MineDiversityMode::Hybrid;
  double layoutWeight = 0.50;
};

struct MineCheckpointShHeader {
  int version = 1;
  MineConfig mineCfg{};
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  MineCheckpointShConfig sh{};
};

// Write a staged checkpoint header as a compact single-line JSON object.
bool WriteMineCheckpointShHeader(std::ostream& os, const MineCheckpointShHeader& h, std::string* outError = nullptr);

// Append a single record line.
bool AppendMineCheckpointShRecord(std::ostream& os, int stage, int index, const MineRecord& r, std::string* outError = nullptr);

// Load a staged checkpoint file.
//
// - outHeader: if non-null, receives the parsed header.
// - outStageRecords: stage-major array. Each stage vector is sized to
//   (maxIndex+1) seen for that stage, and missing indices are filled with
//   default records.
// - outStageHaveIndex: optional; same shape as outStageRecords. True when the
//   corresponding record was present in the checkpoint.
//
// Returns false on parse errors.
bool LoadMineCheckpointShFile(const std::string& path,
                              MineCheckpointShHeader* outHeader,
                              std::vector<std::vector<MineRecord>>& outStageRecords,
                              std::vector<std::vector<bool>>* outStageHaveIndex,
                              std::string* outError = nullptr);

// Compare two configurations to determine if resuming is safe.
//
// threads is intentionally ignored (resume may use different worker counts).
bool MineCheckpointShConfigsMatch(const MineCheckpointShHeader& a, const MineCheckpointShHeader& b, std::string* outWhy = nullptr);

} // namespace isocity
