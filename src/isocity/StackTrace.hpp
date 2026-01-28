#pragma once

#include <string>

namespace isocity {

// Best-effort stack trace capture for crash diagnostics.
//
// Notes:
//  - Intended for crash reports, not for gameplay logic.
//  - Symbol quality depends on platform and build flags (e.g. debug symbols).
//  - Some platforms emit mangled C++ names; we attempt lightweight demangling
//    where possible.
struct StackTraceOptions {
  // Number of top frames to skip (useful to hide the stacktrace helper itself).
  int skipFrames = 0;

  // Maximum number of frames to capture.
  int maxFrames = 64;
};

// Capture a stack trace for the current thread.
// Returns a multi-line string. Empty string indicates "not available".
std::string CaptureStackTrace(const StackTraceOptions& opt = {});

} // namespace isocity
