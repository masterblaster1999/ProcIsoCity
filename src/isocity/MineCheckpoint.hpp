#pragma once

#include "isocity/ConfigIO.hpp"
#include "isocity/Json.hpp"
#include "isocity/SeedMiner.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Mining checkpoint (JSONL)
//
// Long seed-mining runs are easy to interrupt (CI timeouts, laptop sleep, etc.).
// This module implements a tiny, dependency-free checkpoint format so mining can
// be resumed without losing completed work.
//
// Format: JSON Lines (one JSON object per line).
//  - Line 0: header
//      {"type":"procisocity_mine_checkpoint","version":1,"mine":{...},"proc":{...},"sim":{...}}
//  - Line N>0: records
//      {"type":"record","index":<i>,"record":{...MineRecordToJson...}}
//
// See: https://jsonlines.org/ ("JSON Lines")
// -----------------------------------------------------------------------------

struct MineCheckpointHeader {
  int version = 1;
  MineConfig mineCfg{};
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
};

// Write a checkpoint header as a compact single-line JSON object.
bool WriteMineCheckpointHeader(std::ostream& os, const MineCheckpointHeader& h, std::string* outError = nullptr);

// Append a single record line.
bool AppendMineCheckpointRecord(std::ostream& os, int index, const MineRecord& r, std::string* outError = nullptr);

// Load a checkpoint file.
//
// - outHeader: if non-null, receives the parsed header.
// - outRecords: receives records in index order (0..maxIndex). Missing indices are
//   filled with default records and are marked in outHaveIndex if provided.
// - outHaveIndex: optional; same length as outRecords. True when the corresponding
//   entry was present in the checkpoint.
//
// Returns false on parse errors.
bool LoadMineCheckpointFile(const std::string& path,
                            MineCheckpointHeader* outHeader,
                            std::vector<MineRecord>& outRecords,
                            std::vector<bool>* outHaveIndex,
                            std::string* outError = nullptr);

// Compare two configurations to determine if resuming is safe.
//
// threads is intentionally ignored (resume may use different worker counts).
bool MineCheckpointConfigsMatch(const MineCheckpointHeader& a, const MineCheckpointHeader& b, std::string* outWhy = nullptr);

} // namespace isocity
