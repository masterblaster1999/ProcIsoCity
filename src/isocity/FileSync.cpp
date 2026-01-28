#include "isocity/FileSync.hpp"

#include <cerrno>
#include <cstring>
#include <sstream>

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

#if defined(_WIN32)

static std::string Win32ErrorMessage(DWORD e)
{
  if (e == 0) return "";

  LPWSTR buf = nullptr;
  const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
  const DWORD len = FormatMessageW(flags,
                                  nullptr,
                                  e,
                                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                                  reinterpret_cast<LPWSTR>(&buf),
                                  0,
                                  nullptr);

  std::string msg;
  if (len && buf) {
    // Convert UTF-16 to UTF-8 (best-effort).
    const int need = WideCharToMultiByte(CP_UTF8, 0, buf, static_cast<int>(len), nullptr, 0, nullptr, nullptr);
    if (need > 0) {
      msg.resize(static_cast<std::size_t>(need));
      WideCharToMultiByte(CP_UTF8, 0, buf, static_cast<int>(len), msg.data(), need, nullptr, nullptr);
    }
  }

  if (buf) {
    LocalFree(buf);
  }
  // Trim trailing newlines/spaces.
  while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r' || msg.back() == ' ' || msg.back() == '\t')) {
    msg.pop_back();
  }
  return msg;
}

static bool FlushHandle(HANDLE h, std::string& outError)
{
  outError.clear();
  if (h == INVALID_HANDLE_VALUE) {
    outError = "invalid handle";
    return false;
  }

  if (!FlushFileBuffers(h)) {
    const DWORD e = GetLastError();
    std::ostringstream oss;
    oss << "FlushFileBuffers failed (error " << e << ")";
    const std::string msg = Win32ErrorMessage(e);
    if (!msg.empty()) {
      oss << ": " << msg;
    }
    outError = oss.str();
    return false;
  }
  return true;
}

#endif

} // namespace

bool SyncFile(const std::filesystem::path& path, std::string& outError)
{
  outError.clear();
  if (path.empty()) {
    outError = "SyncFile path is empty";
    return false;
  }

#if defined(_WIN32)
  // Open with write access to maximize the chance FlushFileBuffers succeeds.
  const std::wstring wpath = path.wstring();
  HANDLE h = CreateFileW(wpath.c_str(),
                         GENERIC_READ | GENERIC_WRITE,
                         FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                         nullptr,
                         OPEN_EXISTING,
                         FILE_ATTRIBUTE_NORMAL,
                         nullptr);

  if (h == INVALID_HANDLE_VALUE) {
    const DWORD e = GetLastError();
    std::ostringstream oss;
    oss << "Unable to open file for sync: " << path.string() << " (error " << e << ")";
    const std::string msg = Win32ErrorMessage(e);
    if (!msg.empty()) oss << ": " << msg;
    outError = oss.str();
    return false;
  }

  std::string flushErr;
  const bool ok = FlushHandle(h, flushErr);
  CloseHandle(h);
  if (!ok) {
    outError = "SyncFile(" + path.string() + "): " + flushErr;
    return false;
  }
  return true;
#else
  const int fd = ::open(path.c_str(), O_RDWR);
  if (fd < 0) {
    // If we cannot open read-write, retry read-only (some FS allow fsync).
    const int fd2 = ::open(path.c_str(), O_RDONLY);
    if (fd2 < 0) {
      outError = "Unable to open file for sync: " + path.string() + ": " + std::strerror(errno);
      return false;
    }
    if (::fsync(fd2) != 0) {
      outError = "fsync failed for file: " + path.string() + ": " + std::strerror(errno);
      ::close(fd2);
      return false;
    }
    ::close(fd2);
    return true;
  }

  if (::fsync(fd) != 0) {
    outError = "fsync failed for file: " + path.string() + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  ::close(fd);
  return true;
#endif
}

bool SyncDirectory(const std::filesystem::path& dir, std::string& outError)
{
  outError.clear();
  if (dir.empty()) {
    outError = "SyncDirectory path is empty";
    return false;
  }

#if defined(_WIN32)
  const std::wstring wdir = dir.wstring();
  HANDLE h = CreateFileW(wdir.c_str(),
                         GENERIC_READ | GENERIC_WRITE,
                         FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                         nullptr,
                         OPEN_EXISTING,
                         FILE_FLAG_BACKUP_SEMANTICS,
                         nullptr);

  if (h == INVALID_HANDLE_VALUE) {
    const DWORD e = GetLastError();
    std::ostringstream oss;
    oss << "Unable to open directory for sync: " << dir.string() << " (error " << e << ")";
    const std::string msg = Win32ErrorMessage(e);
    if (!msg.empty()) oss << ": " << msg;
    outError = oss.str();
    return false;
  }

  std::string flushErr;
  const bool ok = FlushHandle(h, flushErr);
  CloseHandle(h);
  if (!ok) {
    outError = "SyncDirectory(" + dir.string() + "): " + flushErr;
    return false;
  }
  return true;
#else
  int flags = O_RDONLY;
#ifdef O_DIRECTORY
  flags |= O_DIRECTORY;
#endif
  const int fd = ::open(dir.c_str(), flags);
  if (fd < 0) {
    outError = "Unable to open directory for sync: " + dir.string() + ": " + std::strerror(errno);
    return false;
  }

  if (::fsync(fd) != 0) {
    outError = "fsync failed for directory: " + dir.string() + ": " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  ::close(fd);
  return true;
#endif
}

void BestEffortSyncFile(const std::filesystem::path& path)
{
  std::string err;
  (void)SyncFile(path, err);
}

void BestEffortSyncDirectory(const std::filesystem::path& dir)
{
  std::string err;
  (void)SyncDirectory(dir, err);
}

} // namespace isocity
