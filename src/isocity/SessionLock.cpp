#include "isocity/SessionLock.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <system_error>

#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <unistd.h>
#endif

namespace isocity {

namespace {

static std::string Trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) {
    ++a;
  }
  std::size_t b = s.size();
  while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) {
    --b;
  }
  return s.substr(a, b - a);
}

static bool ParseKeyValueLine(const std::string& line, std::string& outKey, std::string& outValue)
{
  const auto pos = line.find('=');
  if (pos == std::string::npos) return false;
  outKey = Trim(line.substr(0, pos));
  outValue = Trim(line.substr(pos + 1));
  return !outKey.empty();
}

static std::string TimestampUtcIso8601()
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
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec);
  return std::string(buf);
}

} // namespace

struct SessionLock::Impl {
  std::filesystem::path lockPath;
  std::filesystem::path markerPath;

  bool previousUnclean = false;
  SessionInfo previousInfo{};

#if defined(_WIN32)
  HANDLE lockHandle = INVALID_HANDLE_VALUE;
#else
  int lockFd = -1;
#endif
};

SessionLock::~SessionLock()
{
  release();
}

bool SessionLock::acquire(const SessionLockOptions& opt, std::string& outError)
{
  outError.clear();
  release();

  if (opt.dir.empty()) {
    outError = "SessionLock: directory is empty";
    return false;
  }

  auto impl = std::make_unique<Impl>();
  impl->lockPath = opt.dir / opt.lockFileName;
  impl->markerPath = opt.dir / opt.markerFileName;

  // 1) Acquire the OS-level lock on the lock file.
#if defined(_WIN32)
  {
    const std::wstring wpath = impl->lockPath.wstring();
    HANDLE h = CreateFileW(
        wpath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,               // no sharing: acts as an exclusive lock
        nullptr,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);

    if (h == INVALID_HANDLE_VALUE) {
      const DWORD e = GetLastError();
      if (e == ERROR_SHARING_VIOLATION || e == ERROR_ACCESS_DENIED) {
        outError = "Another ProcIsoCity instance appears to be using this data directory.";
      } else {
        std::ostringstream oss;
        oss << "Unable to open session lock file '" << impl->lockPath.string() << "' (error " << e << ")";
        outError = oss.str();
      }
      return false;
    }

    impl->lockHandle = h;
  }
#else
  {
    const std::string pathStr = impl->lockPath.string();
    const int fd = ::open(pathStr.c_str(), O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
      outError = "Unable to open session lock file '" + pathStr + "': " + std::strerror(errno);
      return false;
    }

    // POSIX advisory record lock (whole-file write lock).
    struct flock fl{};
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0; // 0 = to EOF

    if (::fcntl(fd, F_SETLK, &fl) == -1) {
      const int e = errno;
      ::close(fd);
      if (e == EACCES || e == EAGAIN) {
        outError = "Another ProcIsoCity instance appears to be using this data directory.";
      } else {
        outError = "Unable to lock session file '" + pathStr + "': " + std::strerror(e);
      }
      return false;
    }

    impl->lockFd = fd;
  }
#endif

  // 2) Detect previous unclean shutdown by looking for a leftover marker file.
  {
    std::error_code ec;
    if (std::filesystem::exists(impl->markerPath, ec) && !ec) {
      impl->previousUnclean = true;
      std::string readErr;
      (void)ReadSessionInfoFile(impl->markerPath, impl->previousInfo, readErr);
    }
  }

  // 3) Write/refresh the marker file for this session.
  {
    std::string writeErr;
    if (!WriteSessionInfoFile(impl->markerPath, opt.info, writeErr)) {
      // Marker failure should not prevent the game from running, but it does
      // reduce recovery reliability. Surface as a warning.
      outError = "Warning: failed to write session marker file '" + impl->markerPath.string() + "': " + writeErr;
      // We still succeed acquisition.
    }
  }

  m_impl = std::move(impl);
  return true;
}

void SessionLock::release()
{
  if (!m_impl) return;

  // Best-effort: remove the marker file to indicate a clean shutdown.
  {
    std::error_code ec;
    (void)std::filesystem::remove(m_impl->markerPath, ec);
  }

#if defined(_WIN32)
  if (m_impl->lockHandle != INVALID_HANDLE_VALUE) {
    CloseHandle(m_impl->lockHandle);
    m_impl->lockHandle = INVALID_HANDLE_VALUE;
  }
#else
  if (m_impl->lockFd >= 0) {
    ::close(m_impl->lockFd);
    m_impl->lockFd = -1;
  }
#endif

  m_impl.reset();
}

bool SessionLock::acquired() const
{
  return m_impl != nullptr;
}

bool SessionLock::previousSessionUnclean() const
{
  return m_impl ? m_impl->previousUnclean : false;
}

const SessionInfo& SessionLock::previousSessionInfo() const
{
  static const SessionInfo kEmpty;
  return m_impl ? m_impl->previousInfo : kEmpty;
}

const std::filesystem::path& SessionLock::lockPath() const
{
  static const std::filesystem::path kEmpty;
  return m_impl ? m_impl->lockPath : kEmpty;
}

const std::filesystem::path& SessionLock::markerPath() const
{
  static const std::filesystem::path kEmpty;
  return m_impl ? m_impl->markerPath : kEmpty;
}

std::uint32_t SessionLock::CurrentPid()
{
#if defined(_WIN32)
  return static_cast<std::uint32_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint32_t>(::getpid());
#endif
}

std::string SessionLock::UtcNowIso8601()
{
  return TimestampUtcIso8601();
}

bool SessionLock::ReadSessionInfoFile(const std::filesystem::path& path, SessionInfo& outInfo, std::string& outError)
{
  outError.clear();
  outInfo = SessionInfo{};

  std::ifstream ifs(path);
  if (!ifs) {
    outError = "Unable to open file: " + path.string();
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    std::string k, v;
    if (!ParseKeyValueLine(line, k, v)) continue;

    if (k == "pid") {
      try {
        outInfo.pid = static_cast<std::uint32_t>(std::stoul(v));
      } catch (...) {
        // ignore
      }
    } else if (k == "started_utc") {
      outInfo.startedUtc = v;
    } else if (k == "exe") {
      outInfo.exePath = v;
    } else if (k == "build") {
      outInfo.buildStamp = v;
    }
  }

  return true;
}

bool SessionLock::WriteSessionInfoFile(const std::filesystem::path& path, const SessionInfo& info, std::string& outError)
{
  outError.clear();

  std::error_code ec;
  const std::filesystem::path parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      outError = "Unable to create directories for '" + parent.string() + "': " + ec.message();
      return false;
    }
  }

  std::ofstream ofs(path, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!ofs) {
    outError = "Unable to open file for writing: " + path.string();
    return false;
  }

  ofs << "pid=" << info.pid << "\n";
  if (!info.startedUtc.empty()) ofs << "started_utc=" << info.startedUtc << "\n";
  if (!info.exePath.empty()) ofs << "exe=" << info.exePath << "\n";
  if (!info.buildStamp.empty()) ofs << "build=" << info.buildStamp << "\n";
  ofs.flush();

  if (!ofs) {
    outError = "Failed to write session info: " + path.string();
    return false;
  }

  return true;
}

} // namespace isocity
