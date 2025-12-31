#pragma once

#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"

#include <string>

namespace isocity {

// JSON helpers for ProcGenConfig and SimConfig.
//
// Goals:
//  - Enable lightweight, dependency-free config editing (CI, tooling).
//  - Support partial "override" JSON (merge semantics): missing keys leave the
//    existing config unchanged.
//
// The JSON field names are snake_case.

// Serialize configs to JSON (pretty-printed).
std::string ProcGenConfigToJson(const ProcGenConfig& cfg, int indentSpaces = 2);
std::string SimConfigToJson(const SimConfig& cfg, int indentSpaces = 2);
std::string CombinedConfigToJson(const ProcGenConfig& proc, const SimConfig& sim, int indentSpaces = 2);

// Apply JSON overrides into an existing config (merge semantics).
bool ApplyProcGenConfigJson(const JsonValue& root, ProcGenConfig& ioCfg, std::string& outError);
bool ApplySimConfigJson(const JsonValue& root, SimConfig& ioCfg, std::string& outError);

// File helpers.
bool WriteProcGenConfigJsonFile(const std::string& path, const ProcGenConfig& cfg, std::string& outError,
                                int indentSpaces = 2);
bool WriteSimConfigJsonFile(const std::string& path, const SimConfig& cfg, std::string& outError,
                            int indentSpaces = 2);

bool LoadProcGenConfigJsonFile(const std::string& path, ProcGenConfig& ioCfg, std::string& outError);
bool LoadSimConfigJsonFile(const std::string& path, SimConfig& ioCfg, std::string& outError);

// If you want a strict combined config file that fully specifies both configs, use CombinedConfigToJson.
// LoadCombinedConfigJsonFile parses {"proc":{...},"sim":{...}} into brand new config objects.
struct CombinedConfig {
  ProcGenConfig proc{};
  SimConfig sim{};
  bool hasProc = false;
  bool hasSim = false;
};

bool LoadCombinedConfigJsonFile(const std::string& path, CombinedConfig& outCfg, std::string& outError);

} // namespace isocity
