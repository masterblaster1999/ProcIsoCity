#pragma once

#include <filesystem>
#include <string>

namespace isocity {

// Best-effort filesystem synchronization helpers.
//
// Motivation:
//  - ProcIsoCity saves use a temp-file + rename pattern to avoid partial writes.
//  - On many systems, std::ofstream::flush() does NOT guarantee the bytes have
//    reached stable storage. A power loss can still lose the most recent save.
//  - The most robust pattern is:
//      1) write tmp
//      2) fsync(tmp)
//      3) rename(tmp -> final)
//      4) fsync(parent directory)
//
// These functions implement that pattern in a cross-platform, dependency-free
// way (best-effort on platforms/filesystems that don't support all operations).

// Flush file contents/metadata to stable storage.
//
// Returns false if the OS call fails.
bool SyncFile(const std::filesystem::path& path, std::string& outError);

// Flush directory metadata to stable storage (best-effort).
//
// Some platforms/filesystems may not support syncing directories; in that case
// this may return false.
bool SyncDirectory(const std::filesystem::path& dir, std::string& outError);

// Convenience wrappers that ignore errors.
void BestEffortSyncFile(const std::filesystem::path& path);
void BestEffortSyncDirectory(const std::filesystem::path& dir);

} // namespace isocity
