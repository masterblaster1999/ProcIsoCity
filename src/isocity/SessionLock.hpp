#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace isocity {

// Session lock + crash marker for the interactive executable.
//
// Goals:
//  - Prevent two ProcIsoCity instances from writing to the same save directory.
//  - Detect previous unclean shutdowns (crash, kill -9, power loss) so the
//    launcher can offer safe-mode or auto-recovery behaviour.
//
// Design:
//  - We take an OS-level exclusive lock on a stable lock file (`proc_isocity.lock`).
//    This lock is automatically released by the OS when the process exits,
//    even on crashes.
//  - We also create a lightweight "marker" file (`proc_isocity.running`) and
//    remove it on graceful exit. If the marker is found at startup, the previous
//    session likely ended uncleanly.

struct SessionInfo {
  std::uint32_t pid = 0;
  std::string startedUtc; // ISO8601-ish: 2026-01-27T12:34:56Z
  std::string exePath;
  std::string buildStamp;
};

struct SessionLockOptions {
  std::filesystem::path dir;
  std::string lockFileName = "proc_isocity.lock";
  std::string markerFileName = "proc_isocity.running";

  SessionInfo info;
};

class SessionLock {
public:
  SessionLock() = default;
  ~SessionLock();

  SessionLock(const SessionLock&) = delete;
  SessionLock& operator=(const SessionLock&) = delete;

  bool acquire(const SessionLockOptions& opt, std::string& outError);
  void release();

  bool acquired() const;
  bool previousSessionUnclean() const;
  const SessionInfo& previousSessionInfo() const;

  const std::filesystem::path& lockPath() const;
  const std::filesystem::path& markerPath() const;

  static std::uint32_t CurrentPid();
  static std::string UtcNowIso8601();

  static bool ReadSessionInfoFile(const std::filesystem::path& path, SessionInfo& outInfo, std::string& outError);
  static bool WriteSessionInfoFile(const std::filesystem::path& path, const SessionInfo& info, std::string& outError);

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace isocity
