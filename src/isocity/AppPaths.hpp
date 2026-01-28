#pragma once

#include <filesystem>
#include <string>

namespace isocity {

// Cross-platform path helpers for the ProcIsoCity executables.
//
// The interactive game historically used the process current working directory for
// saves, blueprints, thumbnails, etc. That makes the "just run the .exe" experience
// fragile:
//   - double-click launches often pick an unexpected working dir
//   - installed locations may be read-only
//   - build directories get polluted with runtime files
//
// AppPaths provides:
//   - executable path/dir discovery (best-effort)
//   - per-user data/config/cache directories (XDG on Linux, AppData on Windows,
//     Library folders on macOS)
//   - a portable data dir next to the executable
//
// This module intentionally depends only on the C++ standard library (plus tiny
// OS-specific shims for executable path discovery).
class AppPaths {
public:
  // Optional initialization; pass argv[0] so we have a fallback if OS APIs fail.
  static void Init(const char* argv0);

  // Best-effort path to the currently running executable.
  // May be empty if unknown.
  static std::filesystem::path ExecutablePath();
  static std::filesystem::path ExecutableDir();

  // Per-user directories (best-effort).
  static std::filesystem::path UserDataDir();
  static std::filesystem::path UserConfigDir();
  static std::filesystem::path UserCacheDir();

  // Portable mode directory next to the executable.
  // Example: <exeDir>/ProcIsoCityData
  static std::filesystem::path PortableDataDir();

  // Create the directory (and parents) if missing.
  // Returns true on success; false and sets outError on failure.
  static bool EnsureDirExists(const std::filesystem::path& dir, std::string& outError);
};

} // namespace isocity
