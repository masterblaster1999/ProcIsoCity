#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace isocity {

// Small helpers for discovering the most recent save/autosave.
//
// Motivation:
//  - Let the interactive executable provide a "--resume" mode.
//  - Provide a tiny reusable utility that can also be used by headless tools.
//
// The interactive app writes a fixed set of canonical save filenames (legacy quicksave,
// numbered manual slots, numbered autosave slots). This module scans only those known
// filenames (rather than wildcarding all *.bin files) so it can be safely used in
// arbitrary directories.

enum class SaveKind : std::uint8_t {
  Manual = 0,
  Autosave = 1,
};

struct SaveCandidate {
  std::filesystem::path path;
  SaveKind kind = SaveKind::Manual;
  int slot = 0; // 1..N (best-effort; depends on file naming)
  std::filesystem::file_time_type timestamp{};
};

struct SaveScanResult {
  std::vector<SaveCandidate> found;

  // Best-effort concatenated filesystem errors (may be empty).
  //
  // Notes:
  //  - Missing files are not errors.
  //  - Errors are recorded when the filesystem reports failures (permissions, etc).
  std::string err;
};

// Scan a directory for the canonical ProcIsoCity save filenames (manual + autosaves).
//
// This deliberately does NOT perform a wildcard scan of all *.bin files; it looks
// only for filenames the app itself writes.
SaveScanResult ScanKnownSaveFiles(const std::filesystem::path& dir,
                                  int manualSlotsMax = 5,
                                  int autosaveSlotsMax = 3);

// Convenience: return the most recently modified known save file in `dir`.
// Returns std::nullopt if no saves were found.
std::optional<SaveCandidate> FindMostRecentSave(const std::filesystem::path& dir,
                                                int manualSlotsMax = 5,
                                                int autosaveSlotsMax = 3);

std::string SaveKindToString(SaveKind k);

} // namespace isocity
