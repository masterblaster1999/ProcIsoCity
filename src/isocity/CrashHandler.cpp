#include "isocity/CrashHandler.hpp"

#include "isocity/StackTrace.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <vector>

#if !defined(_WIN32)
  #include <csignal>
#endif

namespace isocity {

namespace {

static std::atomic_flag g_inCrash = ATOMIC_FLAG_INIT;
static CrashHandlerOptions g_opt;
static bool g_installed = false;

static std::terminate_handler g_prevTerminate = nullptr;

#if !defined(_WIN32)
// Save/restore previous signal handlers.
static void (*g_prevSigSegv)(int) = nullptr;
static void (*g_prevSigIll)(int) = nullptr;
static void (*g_prevSigFpe)(int) = nullptr;
static void (*g_prevSigBus)(int) = nullptr;
#endif

static std::string TimestampUtcForFilename()
{
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const std::time_t t = clock::to_time_t(now);
  std::tm tm{};

#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02dZ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return std::string(buf);
}

static void DoWriteCrashReportFile(const std::string& reason, const std::string& detail, const std::string& stack)
{
  if (g_opt.reportDir.empty()) return;

  std::error_code ec;
  std::filesystem::create_directories(g_opt.reportDir, ec);

  const std::filesystem::path path = g_opt.reportDir / ("crash_" + TimestampUtcForFilename() + ".txt");
  std::ofstream ofs(path, std::ios::out | std::ios::binary);
  if (!ofs) return;

  if (!g_opt.preamble.empty()) {
    ofs << g_opt.preamble;
    if (g_opt.preamble.back() != '\n') ofs << "\n";
  }

  ofs << "--- crash ---\n";
  ofs << "reason: " << reason << "\n";
  ofs << "detail: " << detail << "\n";
  if (!stack.empty()) {
    ofs << "\n";
    ofs << stack;
    if (stack.back() != '\n') ofs << "\n";
  }

  // Optional log tail.
  if (!g_opt.logTailPath.empty() && g_opt.logTailMaxBytes > 0) {
    ofs << "\n";
    ofs << "--- log tail ---\n";
    ofs << "path: " << g_opt.logTailPath.string() << "\n";

    // Best-effort: read the last N bytes, then optionally trim to the last M lines.
    std::ifstream ifs(g_opt.logTailPath, std::ios::in | std::ios::binary);
    if (!ifs) {
      ofs << "(unable to open log file)\n";
    } else {
      ifs.seekg(0, std::ios::end);
      const std::streamoff size = ifs.tellg();
      if (size <= 0) {
        ofs << "(log file is empty)\n";
        ofs.flush();
        return;
      }
      const std::streamoff maxBytes = static_cast<std::streamoff>(std::min<std::size_t>(g_opt.logTailMaxBytes, 4u * 1024u * 1024u));
      const std::streamoff start = (size > maxBytes) ? (size - maxBytes) : 0;

      ifs.seekg(start, std::ios::beg);
      std::vector<char> buf;
      buf.resize(static_cast<std::size_t>(size - start));
      if (!buf.empty()) {
        ifs.read(buf.data(), static_cast<std::streamsize>(buf.size()));
      }

      std::string tail(buf.begin(), buf.end());

      bool truncated = (start > 0);
      if (truncated) {
        // If we started mid-line, drop the first partial line.
        const std::size_t nl = tail.find('\n');
        if (nl != std::string::npos && nl + 1 < tail.size()) {
          tail = tail.substr(nl + 1);
        }
      }

      if (g_opt.logTailMaxLines > 0) {
        int lines = 0;
        for (std::size_t i = tail.size(); i > 0; --i) {
          if (tail[i - 1] == '\n') {
            ++lines;
            if (lines > g_opt.logTailMaxLines) {
              // Cut to the last maxLines lines.
              tail = tail.substr(i);
              truncated = true;
              break;
            }
          }
        }
      }

      if (truncated) {
        ofs << "(tail truncated)\n";
      }
      if (tail.empty()) {
        ofs << "(log file is empty)\n";
      } else {
        ofs << tail;
        if (tail.back() != '\n') ofs << "\n";
      }
    }
  }
  ofs.flush();
}

static void WriteCrashReportInternal(const std::string& reason, const std::string& detail, int extraSkipFrames)
{
  if (!g_installed) return;

  // Avoid recursion (e.g. crash while writing a crash report).
  if (g_inCrash.test_and_set()) return;

  try {
    StackTraceOptions st;
    st.skipFrames = std::max(0, 2 + extraSkipFrames);
    st.maxFrames = g_opt.maxStackFrames;
    const std::string stack = CaptureStackTrace(st);

    DoWriteCrashReportFile(reason, detail, stack);
  } catch (...) {
    // Best effort; nothing else to do.
  }

  // Allow subsequent reports if the process keeps running (e.g. a controlled
  // shutdown after catching an exception). If we crash mid-report, we might not
  // reach this line, which is fine (it still prevents recursion).
  g_inCrash.clear();
}

static void TerminateHandler() noexcept
{
  try {
    const std::exception_ptr ep = std::current_exception();
    if (ep) {
      try {
        std::rethrow_exception(ep);
      } catch (const std::exception& e) {
        WriteCrashReportInternal("std::terminate", e.what(), 1);
      } catch (...) {
        WriteCrashReportInternal("std::terminate", "unknown exception", 1);
      }
    } else {
      WriteCrashReportInternal("std::terminate", "no active exception", 1);
    }
  } catch (...) {
    // Swallow all errors in the terminate path.
  }

  // If there was a previous terminate handler installed, give it a chance.
  // NOTE: This is best-effort; many handlers call abort() anyway.
  if (g_prevTerminate) {
    try {
      g_prevTerminate();
    } catch (...) {
      // ignore
    }
  }

  std::abort();
}

#if !defined(_WIN32)

static const char* SignalName(int sig)
{
  switch (sig) {
    case SIGSEGV: return "SIGSEGV";
    case SIGILL: return "SIGILL";
    case SIGFPE: return "SIGFPE";
#ifdef SIGBUS
    case SIGBUS: return "SIGBUS";
#endif
    default: return "SIGNAL";
  }
}

static void SignalHandler(int sig)
{
  // Best-effort signal crash report.
  std::ostringstream oss;
  oss << SignalName(sig) << " (" << sig << ")";
  WriteCrashReportInternal("signal", oss.str(), 1);

  // Restore default handling and re-raise to preserve expected exit codes/core.
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

#endif

} // namespace

void InstallCrashHandler(const CrashHandlerOptions& opt)
{
  g_opt = opt;
  g_opt.maxStackFrames = std::clamp(g_opt.maxStackFrames, 0, 256);
  g_opt.logTailMaxBytes = std::clamp<std::size_t>(g_opt.logTailMaxBytes, 0u, 4u * 1024u * 1024u);
  g_opt.logTailMaxLines = std::clamp(g_opt.logTailMaxLines, 0, 10000);

  // Install terminate handler (captures unhandled C++ exceptions).
  g_prevTerminate = std::set_terminate(TerminateHandler);

#if !defined(_WIN32)
  // Install a small set of fatal signal handlers so we can emit a report for
  // segfaults/illegal instructions/etc.
  g_prevSigSegv = std::signal(SIGSEGV, SignalHandler);
  g_prevSigIll = std::signal(SIGILL, SignalHandler);
  g_prevSigFpe = std::signal(SIGFPE, SignalHandler);
#ifdef SIGBUS
  g_prevSigBus = std::signal(SIGBUS, SignalHandler);
#endif
#endif

  g_installed = true;
}

void UninstallCrashHandler()
{
  if (!g_installed) return;

  // Restore previous terminate handler.
  if (g_prevTerminate) {
    std::set_terminate(g_prevTerminate);
  }

#if !defined(_WIN32)
  // Restore previous signal handlers.
  if (g_prevSigSegv && g_prevSigSegv != SIG_ERR) std::signal(SIGSEGV, g_prevSigSegv);
  if (g_prevSigIll && g_prevSigIll != SIG_ERR) std::signal(SIGILL, g_prevSigIll);
  if (g_prevSigFpe && g_prevSigFpe != SIG_ERR) std::signal(SIGFPE, g_prevSigFpe);
#ifdef SIGBUS
  if (g_prevSigBus && g_prevSigBus != SIG_ERR) std::signal(SIGBUS, g_prevSigBus);
#endif
#endif

  g_installed = false;
}

void WriteCrashReport(const std::string& reason, const std::string& detail)
{
  WriteCrashReportInternal(reason, detail, 1);
}

} // namespace isocity
