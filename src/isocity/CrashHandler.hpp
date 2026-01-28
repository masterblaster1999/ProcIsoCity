#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace isocity {

// Installs best-effort crash handlers (terminate + signals) that emit a
// human-readable crash report to disk.
//
// The crash report is intended to be useful for players ("what happened?")
// and for developers (stack trace + build stamp + argv).

struct CrashHandlerOptions {
  // Directory where crash_*.txt reports are written.
  std::filesystem::path reportDir;

  // Preamble written at the top of every crash report (version, cwd, argv...).
  std::string preamble;

  // Maximum number of stack frames to capture.
  int maxStackFrames = 64;

  // Optional: include a tail of a log file in crash reports.
  //
  // This is best-effort, but enormously improves the usefulness of player
  // crash reports by providing immediate context.
  std::filesystem::path logTailPath;

  // Maximum number of bytes to read from the end of the log file.
  // Clamped to a reasonable range.
  std::size_t logTailMaxBytes = 128u * 1024u;

  // Maximum number of log lines to include (0 disables the line limit).
  int logTailMaxLines = 250;
};

// Install handlers. Safe to call multiple times; the latest options win.
void InstallCrashHandler(const CrashHandlerOptions& opt);

// Best-effort uninstall (restores previous terminate handler and signal
// handlers where possible).
void UninstallCrashHandler();

// Write a crash report immediately using the currently installed settings.
// If handlers were not installed, this is a no-op.
void WriteCrashReport(const std::string& reason, const std::string& detail);

} // namespace isocity
