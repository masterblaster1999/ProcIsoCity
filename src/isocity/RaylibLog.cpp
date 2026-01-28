#include "isocity/RaylibLog.hpp"

#include "isocity/RaylibShim.hpp"
#include "isocity/RaylibTrace.hpp"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>

namespace isocity {

namespace {

static std::mutex g_mutex;
static bool g_installed = false;
static int g_minLevel = -1;
static RaylibTraceLogCallback g_prevCallback = nullptr;

static std::string ToLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static void RaylibTraceLogCallback(int logLevel, const char* text, va_list args)
{
  // raylib might call the callback from internal subsystems; keep it small.
  // We serialize formatting + output to avoid interleaving multi-line messages.
  std::scoped_lock<std::mutex> lock(g_mutex);

  // Filter (raylib also filters via SetTraceLogLevel, but callers may not set it).
  if (g_minLevel >= 0 && logLevel < g_minLevel) {
    return;
  }

  char buf[4096];
  buf[0] = '\0';
  if (text) {
#if defined(_WIN32)
    // MSVC's vsnprintf returns -1 on truncation; we still print best-effort.
    std::vsnprintf(buf, sizeof(buf), text, args);
#else
    std::vsnprintf(buf, sizeof(buf), text, args);
#endif
  }

  // Ensure newline-terminated output for log file readability.
  const std::size_t len = std::strlen(buf);
  const bool hasNl = (len > 0 && buf[len - 1] == '\n');

  std::ostream& os = std::cerr;
  os << "[raylib:" << RaylibLogLevelName(logLevel) << "] ";
  os << (buf[0] ? buf : "(null)");
  if (!hasNl) os << "\n";
  os.flush();
}

} // namespace

int ParseRaylibLogLevel(const std::string& s, int fallback)
{
  const std::string k = ToLower(s);
  if (k == "all") return LOG_ALL;
  if (k == "trace") return LOG_TRACE;
  if (k == "debug") return LOG_DEBUG;
  if (k == "info") return LOG_INFO;
  if (k == "warn" || k == "warning") return LOG_WARNING;
  if (k == "error") return LOG_ERROR;
  if (k == "fatal") return LOG_FATAL;
  if (k == "none" || k == "off" || k == "quiet") return LOG_NONE;
  return fallback;
}

const char* RaylibLogLevelName(int level)
{
  switch (level) {
    case LOG_ALL: return "ALL";
    case LOG_TRACE: return "TRACE";
    case LOG_DEBUG: return "DEBUG";
    case LOG_INFO: return "INFO";
    case LOG_WARNING: return "WARN";
    case LOG_ERROR: return "ERROR";
    case LOG_FATAL: return "FATAL";
    case LOG_NONE: return "NONE";
    default: return "LOG";
  }
}

void InstallRaylibLogCallback(int minLevel)
{
  std::scoped_lock<std::mutex> lock(g_mutex);

  g_minLevel = minLevel;

  // Set the raylib internal threshold if requested.
  if (minLevel >= 0) {
    SetTraceLogLevel(minLevel);
  }

  // Register callback.
  if (!g_installed) {
    g_prevCallback = GetRaylibTraceLogCallback();
  }

  SetRaylibTraceLogCallback(RaylibTraceLogCallback);
  g_installed = true;
}

void UninstallRaylibLogCallback()
{
  std::scoped_lock<std::mutex> lock(g_mutex);
  if (!g_installed) return;

  // Restore whatever callback was present before InstallRaylibLogCallback.
  SetRaylibTraceLogCallback(g_prevCallback);
  g_prevCallback = nullptr;
  g_installed = false;
}

} // namespace isocity
