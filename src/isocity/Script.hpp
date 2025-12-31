#pragma once

#include "isocity/ProcGen.hpp"
#include "isocity/Blueprint.hpp"
#include "isocity/AutoBuild.hpp"
#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace isocity {

// Callbacks used by ScriptRunner.
//
// - print(): command output intended for "stdout" style consumption (e.g. `hash`).
// - info():  progress messages (suppressed when ScriptRunOptions::quiet is true).
// - error(): error messages (always emitted).
struct ScriptCallbacks {
  std::function<void(const std::string& line)> print;
  std::function<void(const std::string& line)> info;
  std::function<void(const std::string& line)> error;
};

// Mutable script execution state (exposed so callers can seed the runner with an
// existing world/sim and then adopt the results).
struct ScriptRunnerState {
  // Defaults used before a world is generated or loaded.
  int w = 96;
  int h = 96;
  std::uint64_t seed = 1;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  AutoBuildConfig autoBuildCfg{};
  Simulator sim{SimConfig{}};

  World world;
  bool hasWorld = false;
  bool dirtyDerived = true;

  // Optional in-memory blueprint used by bp_* commands.
  Blueprint blueprint;
  bool hasBlueprint = false;

  // Optional per-tick snapshots collected by the `tick` command.
  std::vector<Stats> tickStats;
};

struct ScriptRunOptions {
  bool quiet = false;

  // Safety guard for recursive `include` scripts.
  int includeDepthLimit = 16;
};

// Deterministic, headless scenario script runner.
//
// This was originally implemented directly inside the CLI tool, but is now in core
// so it can be reused by:
//  - the headless `proc_isocity_script` tool
//  - the interactive app dev console
//  - unit tests / CI regression scripts
class ScriptRunner {
public:
  ScriptRunner();

  void setCallbacks(ScriptCallbacks cb) { m_cb = std::move(cb); }
  void setOptions(ScriptRunOptions opt) { m_opt = opt; }

  ScriptRunnerState& state() { return m_ctx; }
  const ScriptRunnerState& state() const { return m_ctx; }

  // Run a script file from disk.
  bool runFile(const std::string& path);

  // Run a script from an in-memory string.
  bool runText(const std::string& text, const std::string& virtualPath = "<script>");

  // Returns the last formatted error message, if any.
  const std::string& lastError() const { return m_lastError; }
  int lastErrorLine() const { return m_lastErrorLine; }
  const std::string& lastErrorPath() const { return m_lastErrorPath; }

  // Emit an error and mark the current run as failed. Returns false for convenience.
  //
  // This is primarily intended for internal helper functions used by the script
  // implementation, but it can also be useful for higher-level wrappers that
  // want to surface a custom error.
  bool fail(const std::string& path, int line, const std::string& msg);

  // Expand tokens in a path template.
  // Supported tokens:
  //   {seed} {day} {w} {h} {money} {run} {hash}
  std::string expandPathTemplate(const std::string& tmpl, int run = 0) const;

private:
  bool runFileInternal(const std::string& path, int depth);
  bool runTextInternal(const std::string& text, const std::string& virtualPath, int depth);

  void clearError();

  void emitPrint(const std::string& line) const;
  void emitInfo(const std::string& line) const;
  void emitError(const std::string& line) const;

  ScriptRunnerState m_ctx;
  ScriptCallbacks m_cb{};
  ScriptRunOptions m_opt{};

  std::string m_lastError;
  std::string m_lastErrorPath;
  int m_lastErrorLine = 0;
};

} // namespace isocity
