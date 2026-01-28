#pragma once

#include "isocity/RaylibShim.hpp"

namespace isocity {

// Small wrapper around raylib's global TraceLog callback.
//
// Motivation:
//   Multiple subsystems in ProcIsoCity may temporarily install a raylib trace
//   callback (e.g. ShaderUtil capturing compile logs) or install a persistent
//   callback (RaylibLog forwarding into stderr/log files).
//
//   raylib exposes only SetTraceLogCallback() (no Get...), so without a wrapper
//   it's easy for one subsystem to accidentally stomp another.

using RaylibTraceLogCallback = TraceLogCallback;

// Set the raylib TraceLog callback and record it so other subsystems can
// restore it later.
void SetRaylibTraceLogCallback(RaylibTraceLogCallback cb);

// Return the last callback installed via SetRaylibTraceLogCallback (nullptr if
// none).
RaylibTraceLogCallback GetRaylibTraceLogCallback();

// RAII helper that temporarily installs a TraceLog callback and restores the
// previous one on scope exit.
class ScopedRaylibTraceLogCallback {
public:
  explicit ScopedRaylibTraceLogCallback(RaylibTraceLogCallback cb);
  ~ScopedRaylibTraceLogCallback();

  ScopedRaylibTraceLogCallback(const ScopedRaylibTraceLogCallback&) = delete;
  ScopedRaylibTraceLogCallback& operator=(const ScopedRaylibTraceLogCallback&) = delete;

private:
  RaylibTraceLogCallback m_prev = nullptr;
  bool m_active = false;
};

} // namespace isocity
