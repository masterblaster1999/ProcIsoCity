#include "isocity/RaylibTrace.hpp"

#include <mutex>

namespace isocity {

namespace {

static std::mutex g_mutex;
static RaylibTraceLogCallback g_current = nullptr;

} // namespace

void SetRaylibTraceLogCallback(RaylibTraceLogCallback cb)
{
  std::scoped_lock<std::mutex> lock(g_mutex);
  g_current = cb;
  // raylib treats nullptr as "restore default".
  SetTraceLogCallback(cb);
}

RaylibTraceLogCallback GetRaylibTraceLogCallback()
{
  std::scoped_lock<std::mutex> lock(g_mutex);
  return g_current;
}

ScopedRaylibTraceLogCallback::ScopedRaylibTraceLogCallback(RaylibTraceLogCallback cb)
{
  m_prev = GetRaylibTraceLogCallback();
  m_active = true;
  SetRaylibTraceLogCallback(cb);
}

ScopedRaylibTraceLogCallback::~ScopedRaylibTraceLogCallback()
{
  if (!m_active) return;
  SetRaylibTraceLogCallback(m_prev);
}

} // namespace isocity
