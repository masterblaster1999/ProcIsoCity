#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace isocity {

// Creates a small, self-contained "support bundle" folder that can be attached
// to bug reports.
//
// The interactive executable is often launched without a visible console, and
// users may not know where logs or crash reports are stored. SupportBundle
// collects the most useful artifacts (diagnostics text, recent crash reports,
// log files, visual prefs) into a single directory.

struct SupportBundleOptions {
  // Base directory where the bundle folder will be created.
  // If empty, falls back to `dataDir`.
  std::filesystem::path baseDir;

  // Directory that contains runtime data (saves, logs, crash reports).
  // Used to discover crash reports when `crashReportsMax > 0`.
  std::filesystem::path dataDir;

  // Prefix for the created folder name.
  // Final directory name is: <namePrefix>_YYYYMMDD_HHMMSSZ
  std::string namePrefix = "support";

  // Contents of diagnostics.txt.
  std::string diagnosticsText;

  // Optional log file path to copy.
  std::filesystem::path logPath;

  // Number of rotated log files to attempt to copy alongside `logPath`.
  // If logKeepFiles=3 and logPath is "proc_isocity.log", this attempts:
  //   proc_isocity.log, proc_isocity.log.1, proc_isocity.log.2, proc_isocity.log.3
  int logKeepFiles = 0;

  // Optional visual prefs file path to copy.
  // If present, the support bundle also tries to include adjacent transactional
  // artifacts (".tmp" / ".bak") when they exist.
  std::filesystem::path visualPrefsPath;

  // Copy up to N most recent crash_*.txt files from `dataDir`.
  int crashReportsMax = 3;

  // Optional additional files to copy.
  //
  // If a path refers to a directory and extraDirMaxFiles > 0, the directory is
  // scanned recursively and up to extraDirMaxFiles regular files are included
  // (combined across all extra directories). Files are discovered in
  // deterministic lexicographic order so truncation is stable.
  std::vector<std::filesystem::path> extraFiles;

  // Maximum number of files discovered when expanding extra directory paths.
  //
  // - Default is conservative to avoid accidentally bundling huge trees.
  // - Set to 0 to disable directory expansion (directory extras are skipped).
  int extraDirMaxFiles = 2000;

  // Write a manifest.txt with what was included (and any copy failures).
  bool includeManifest = true;
};

struct SupportBundleResult {
  std::filesystem::path bundleDir;
  std::filesystem::path filesDir;
  std::vector<std::string> warnings;
};

bool CreateSupportBundle(const SupportBundleOptions& opt, SupportBundleResult& out, std::string& outError);

// Creates a zipped support bundle (single file) that mirrors the directory layout
// produced by CreateSupportBundle.
//
// The archive contains a single top-level folder:
//   <namePrefix>_YYYYMMDD_HHMMSSZ/
//     diagnostics.txt
//     manifest.txt (optional)
//     files/
//       <logs, crash reports, prefs...>
//
// This is typically the most convenient format for sending bug reports.
struct SupportBundleArchiveResult {
  std::filesystem::path archivePath;
  std::vector<std::string> warnings;
};

bool CreateSupportBundleZip(const SupportBundleOptions& opt, SupportBundleArchiveResult& out, std::string& outError);

} // namespace isocity
