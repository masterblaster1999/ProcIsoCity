#include "isocity/AppPaths.hpp"

#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#elif defined(__APPLE__)
  #include <mach-o/dyld.h>
#else
  #include <unistd.h>
#endif

namespace isocity {

namespace {

std::mutex g_mutex;
bool g_inited = false;
std::filesystem::path g_exePath;
std::filesystem::path g_exeDir;

static std::optional<std::string> GetEnv(const char* name)
{
  if (!name || !*name) return std::nullopt;
  const char* v = std::getenv(name);
  if (!v || !*v) return std::nullopt;
  return std::string(v);
}

static std::filesystem::path HomeDir()
{
#if defined(_WIN32)
  if (auto v = GetEnv("USERPROFILE")) return std::filesystem::path(*v);
  const auto hd = GetEnv("HOMEDRIVE");
  const auto hp = GetEnv("HOMEPATH");
  if (hd && hp) return std::filesystem::path(*hd + *hp);
  return {};
#else
  if (auto v = GetEnv("HOME")) return std::filesystem::path(*v);
  return {};
#endif
}

static std::filesystem::path DetectExecutablePathFromOS()
{
#if defined(_WIN32)
  // Wide API to support non-ASCII install paths.
  std::wstring buf;
  buf.resize(8192);
  DWORD len = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
  if (len == 0 || len >= buf.size()) return {};
  buf.resize(len);
  return std::filesystem::path(buf);
#elif defined(__APPLE__)
  uint32_t size = 0;
  // When buf is NULL, _NSGetExecutablePath writes the required size and returns -1.
  if (_NSGetExecutablePath(nullptr, &size) != -1 || size == 0) {
    return {};
  }
  std::vector<char> buf(size + 1, 0);
  if (_NSGetExecutablePath(buf.data(), &size) != 0) return {};
  return std::filesystem::path(std::string(buf.data()));
#else
  // Linux (and many other UNIXes) expose the current executable at /proc/self/exe.
  std::vector<char> buf(8192, 0);
  const ssize_t len = ::readlink("/proc/self/exe", buf.data(), buf.size() - 1);
  if (len <= 0) return {};
  buf[static_cast<std::size_t>(len)] = 0;
  return std::filesystem::path(std::string(buf.data()));
#endif
}

static void InitLocked(const char* argv0)
{
  if (g_inited) return;
  g_inited = true;

  g_exePath = DetectExecutablePathFromOS();

  // Fallback to argv[0] if OS APIs fail (best-effort).
  if (g_exePath.empty() && argv0 && *argv0) {
    std::error_code ec;
    std::filesystem::path p(argv0);
    if (p.is_relative()) {
      p = std::filesystem::absolute(p, ec);
      if (ec) p = std::filesystem::path(argv0);
    }
    g_exePath = p;
  }

  if (!g_exePath.empty()) {
    std::error_code ec;
    const std::filesystem::path canon = std::filesystem::weakly_canonical(g_exePath, ec);
    if (!ec && !canon.empty()) g_exePath = canon;
    g_exeDir = g_exePath.parent_path();
  }
}

} // namespace

void AppPaths::Init(const char* argv0)
{
  std::lock_guard<std::mutex> lk(g_mutex);
  InitLocked(argv0);
}

std::filesystem::path AppPaths::ExecutablePath()
{
  std::lock_guard<std::mutex> lk(g_mutex);
  InitLocked(nullptr);
  return g_exePath;
}

std::filesystem::path AppPaths::ExecutableDir()
{
  std::lock_guard<std::mutex> lk(g_mutex);
  InitLocked(nullptr);
  return g_exeDir;
}

std::filesystem::path AppPaths::UserDataDir()
{
#if defined(_WIN32)
  if (auto v = GetEnv("LOCALAPPDATA")) return std::filesystem::path(*v) / "ProcIsoCity";
  if (auto v = GetEnv("APPDATA")) return std::filesystem::path(*v) / "ProcIsoCity";
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / "AppData" / "Local" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCity");
#elif defined(__APPLE__)
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / "Library" / "Application Support" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCity");
#else
  if (auto v = GetEnv("XDG_DATA_HOME")) return std::filesystem::path(*v) / "ProcIsoCity";
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / ".local" / "share" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCity");
#endif
}

std::filesystem::path AppPaths::UserConfigDir()
{
#if defined(_WIN32)
  // Windows typically stores config alongside roaming AppData.
  if (auto v = GetEnv("APPDATA")) return std::filesystem::path(*v) / "ProcIsoCity";
  if (auto v = GetEnv("LOCALAPPDATA")) return std::filesystem::path(*v) / "ProcIsoCity";
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / "AppData" / "Roaming" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCity");
#elif defined(__APPLE__)
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / "Library" / "Application Support" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCity");
#else
  if (auto v = GetEnv("XDG_CONFIG_HOME")) return std::filesystem::path(*v) / "ProcIsoCity";
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / ".config" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCity");
#endif
}

std::filesystem::path AppPaths::UserCacheDir()
{
#if defined(_WIN32)
  if (auto v = GetEnv("LOCALAPPDATA")) return std::filesystem::path(*v) / "ProcIsoCity" / "Cache";
  if (auto v = GetEnv("APPDATA")) return std::filesystem::path(*v) / "ProcIsoCity" / "Cache";
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / "AppData" / "Local" / "ProcIsoCity" / "Cache";
  return std::filesystem::path("ProcIsoCityCache");
#elif defined(__APPLE__)
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / "Library" / "Caches" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCityCache");
#else
  if (auto v = GetEnv("XDG_CACHE_HOME")) return std::filesystem::path(*v) / "ProcIsoCity";
  const std::filesystem::path home = HomeDir();
  if (!home.empty()) return home / ".cache" / "ProcIsoCity";
  return std::filesystem::path("ProcIsoCityCache");
#endif
}

std::filesystem::path AppPaths::PortableDataDir()
{
  const std::filesystem::path exeDir = ExecutableDir();
  if (!exeDir.empty()) return exeDir / "ProcIsoCityData";
  return std::filesystem::path("ProcIsoCityData");
}

bool AppPaths::EnsureDirExists(const std::filesystem::path& dir, std::string& outError)
{
  outError.clear();
  if (dir.empty()) {
    outError = "Empty directory path";
    return false;
  }

  std::error_code ec;
  if (std::filesystem::exists(dir, ec)) {
    if (ec) {
      outError = "Failed to stat directory: " + dir.string() + " (" + ec.message() + ")";
      return false;
    }
    if (!std::filesystem::is_directory(dir, ec) || ec) {
      outError = "Path exists but is not a directory: " + dir.string();
      return false;
    }
    return true;
  }

  std::filesystem::create_directories(dir, ec);
  if (ec) {
    outError = "Failed to create directory: " + dir.string() + " (" + ec.message() + ")";
    return false;
  }
  return true;
}

} // namespace isocity
