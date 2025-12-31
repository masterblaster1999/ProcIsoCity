#pragma once

#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace isocity {

// A simple runner for regression scenarios used by CI/headless workflows.
//
// Supported scenario kinds:
//  - Script: a text .isocity/.txt file understood by ScriptRunner
//  - Replay: a binary .isoreplay journal (see Replay.hpp)

enum class ScenarioKind {
  Script = 0,
  Replay = 1,
};

// A single scenario case.
struct ScenarioCase {
  std::string path;
  ScenarioKind kind = ScenarioKind::Script;
};

// Options that control how scenarios are executed.
struct ScenarioRunOptions {
  // If true, ScriptRunner will suppress most prints.
  bool quiet = true;

  // Replay playback strictness.
  bool strictReplayPatches = true;
  bool strictReplayAsserts = true;

  // Run index injected into ScriptRunnerState::runIndex (affects {run} token).
  int runIndex = 0;

  // Variables injected into ScriptRunnerState::vars before execution.
  // Names are normalized to lowercase.
  std::map<std::string, std::string> scriptVars;
};

// Outputs captured from a scenario run.
struct ScenarioRunOutputs {
  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};

  // Optional per-tick Stats snapshots (scripts: tick/autobuild; replays: Tick events).
  std::vector<Stats> tickStats;

  // HashWorld(world, includeStats=true).
  std::uint64_t finalHash = 0;
};

// Guess kind by extension.
//
// Rules:
//  - .isoreplay => Replay
//  - otherwise => Script
ScenarioKind GuessScenarioKindFromPath(const std::string& path);

// Run a single scenario and capture the final world/configs.
// Returns true on success; on failure returns false and writes outError.
bool RunScenario(const ScenarioCase& sc, const ScenarioRunOptions& opt, ScenarioRunOutputs& out, std::string& outError);

// Load a simple manifest format:
//
//   # comments allowed
//   script path/to/scenario.isocity
//   replay path/to/case.isoreplay
//   path/to/implicit_kind.isoreplay
//   path/to/implicit_kind.isocity
//
// If a line has no leading kind token, GuessScenarioKindFromPath() is used.
bool LoadScenarioManifest(const std::string& manifestPath, std::vector<ScenarioCase>& out, std::string& outError);

// Recursively discover scenarios under a directory.
//
// If exts is empty, defaults to {".isocity", ".isoreplay"}.
// Matching is case-insensitive.
bool DiscoverScenarios(const std::string& rootDir, const std::vector<std::string>& exts, std::vector<ScenarioCase>& out,
                      std::string& outError);

} // namespace isocity
