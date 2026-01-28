#pragma once

#include <filesystem>
#include <memory>
#include <string>

namespace isocity {

// Small RAII helper that duplicates std::cout/std::cerr output to a log file.
//
// Motivation:
//  - "double-click" builds often have no visible console, so failures are hard to debug.
//  - Many parts of ProcIsoCity already emit useful diagnostics to stdout/stderr.
//
// Implementation notes:
//  - Uses a custom std::streambuf to forward writes to both the original streambuf
//    (console) and a file streambuf.
//  - Supports simple rotation: <log> -> <log>.1 -> <log>.2 ... up to keepFiles.

struct LogTeeOptions {
  std::filesystem::path path;

  // Number of rotated backups to keep (>=0).
  // keepFiles=0 disables rotation (existing file is truncated).
  int keepFiles = 3;

  bool teeStdout = true;
  bool teeStderr = true;

  // If true, prefix each *log file* line with a UTC timestamp and a stream tag.
  //
  // Example (stdout):
  //   2026-01-27T16:40:12.345Z [OUT] Hello world
  //
  // This greatly improves the usefulness of log files when users report issues.
  // The console output is not affected.
  bool prefixLines = true;

  // If true (and prefixLines is enabled), include a hashed thread id in the
  // prefix as:
  //   [t=0x1234abcd]
  bool prefixThreadId = false;
};

class LogTee {
public:
  LogTee() = default;

  // Convenience constructor.
  LogTee(const LogTeeOptions& opt, std::string& outError);

  ~LogTee();

  LogTee(const LogTee&) = delete;
  LogTee& operator=(const LogTee&) = delete;

  // Start logging. If already active, it will be stopped first.
  bool start(const LogTeeOptions& opt, std::string& outError);

  // Stop logging and restore original std::cout/std::cerr streambufs.
  void stop();

  bool active() const;
  const std::filesystem::path& path() const;

  // Rotate log files: base -> base.1 -> base.2 ... up to keepFiles.
  // Returns true on success; false on fatal error.
  static bool Rotate(const std::filesystem::path& basePath, int keepFiles, std::string& outError);

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace isocity
