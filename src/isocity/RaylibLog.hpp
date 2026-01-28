#pragma once

#include <string>

namespace isocity {

// Small helper to route raylib's internal TraceLog() messages into our
// stdout/stderr streams (and therefore into LogTee when enabled).
//
// This module is only compiled into the interactive app target (proc_isocity).

// Parse a user-facing string into a raylib log level.
//
// Accepted values (case-insensitive):
//   all, trace, debug, info, warn, warning, error, fatal, none
//
// Returns `fallback` if `s` is not recognized.
int ParseRaylibLogLevel(const std::string& s, int fallback);

// Best-effort name for a raylib log level.
const char* RaylibLogLevelName(int level);

// Install a TraceLog callback that forwards raylib logs to std::cerr.
// If minLevel >= 0, we also call SetTraceLogLevel(minLevel).
//
// Safe to call multiple times; the latest settings win.
void InstallRaylibLogCallback(int minLevel);

// Best-effort uninstall.
void UninstallRaylibLogCallback();

} // namespace isocity
