#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace isocity {

// Parser for proc_isocity_cli run manifests written via the --manifest flag.
//
// This provides a small bridging layer between the headless toolchain and the
// interactive renderer/tooling. It lets the app (or other tools) locate
// artifacts (save files, CSVs, image exports) without re-parsing CLI arguments
// or duplicating template-expansion logic.

struct CliManifestArtifact {
  std::string kind;  // e.g. "save", "summary_json", "export_iso", ...
  std::string path;  // path string as written by the CLI (may be relative)
  std::string layer; // optional (for export images)

  std::uint64_t sizeBytes = 0;     // optional
  std::string hashFNV1a64Hex;      // optional (hex string)
  std::string hashError;           // optional (if hashing failed in CLI)
};

struct CliRunManifest {
  // High-level provenance.
  std::string tool;
  std::string toolVersion;
  std::string toolGitSha;
  std::string buildStamp;

  // The working directory of the proc_isocity_cli process when the manifest was
  // written (absolute path, when available).
  //
  // This exists to make relative artifact paths in the manifest robust: the CLI
  // resolves relative paths against its current working directory, which may not
  // match the directory containing the manifest file.
  std::string cwd;

  // Run parameters.
  int runIndex = 0;
  std::uint64_t requestedSeed = 0;
  std::uint64_t actualSeed = 0;
  std::string seedHex;
  int width = 0;
  int height = 0;
  int days = 0;
  std::string worldHashHex;
  std::string loadPath;

  std::vector<std::string> argv;
  std::vector<CliManifestArtifact> artifacts;
};

// Load and parse a manifest JSON file.
//
// Returns true on success. On failure, returns false and sets outError.
bool LoadCliRunManifest(const std::filesystem::path& manifestPath,
                        CliRunManifest& out,
                        std::string& outError);

// Find the first artifact matching a kind (and optionally a layer).
const CliManifestArtifact* FindFirstArtifactByKind(const CliRunManifest& m,
                                                   std::string_view kind,
                                                   std::string_view layer = {});

// Resolve an artifact path against the directory containing the manifest.
//
// If artifactPath is absolute, it is returned as-is.
// If artifactPath is relative, it is interpreted relative to manifestPath.parent_path().
std::filesystem::path ResolveManifestArtifactPath(const std::filesystem::path& manifestPath,
                                                  const std::string& artifactPath);

// Resolve an artifact path robustly, matching proc_isocity_cli semantics.
//
// The CLI records artifact paths as provided on its command line. When those
// are relative, they were interpreted relative to the CLI's current working
// directory at runtime, which may differ from manifestPath.parent_path().
//
// This helper tries several base directories and returns the first candidate
// that exists on disk. If none exist, it falls back to
// ResolveManifestArtifactPath(manifestPath, artifactPath).
//
// If outDebug is non-null, it receives a human-readable summary of which
// candidates were tried and which one was selected.
std::filesystem::path ResolveManifestArtifactPathSmart(const std::filesystem::path& manifestPath,
                                                      const CliRunManifest& manifest,
                                                      const std::string& artifactPath,
                                                      const std::filesystem::path& invocationCwd = {},
                                                      std::string* outDebug = nullptr);

// Expand simple output templates using values from the manifest.
//
// Supported tokens:
//  - {seed}  : actualSeed
//  - {run}   : runIndex
//  - {w}     : width
//  - {h}     : height
//  - {days}  : days
//  - {hash}  : worldHashHex (if present)
std::string ExpandCliManifestTemplate(const std::string& tmpl, const CliRunManifest& m);

// Find the most recently modified proc_isocity_cli manifest JSON within a directory.
//
// This is a convenience for pipelines that generate templated manifest filenames
// (e.g. manifest_{seed}.json) where the caller may not know the exact name.
bool FindLatestCliRunManifestInDir(const std::filesystem::path& dir,
                                   std::filesystem::path& outManifestPath,
                                   std::string& outError);

// Find all proc_isocity_cli manifest JSON files within a directory.
//
// Results are sorted by last_write_time (newest-first by default).
// Returns false if the directory cannot be scanned or if no manifests are found.
bool FindCliRunManifestsInDir(const std::filesystem::path& dir,
                              std::vector<std::filesystem::path>& outManifestPaths,
                              std::string& outError,
                              bool newestFirst = true);

// Add or update an artifact entry inside an existing CLI manifest JSON file.
//
// This is useful when proc_isocity generates rendered outputs (GPU overview PNGs,
// thumbnails, etc.) and wants to record them back into the headless run manifest
// so downstream tooling can discover the full artifact set.
//
// - If an artifact with the same (kind,path,layer) exists, it is replaced.
// - Otherwise, a new artifact object is appended.
// - When artifactDiskPath is provided, the manifest entry will include size_bytes
//   and hash_fnv1a64 fields (or hash_error on failure).
bool UpsertCliRunManifestArtifact(const std::filesystem::path& manifestPath,
                                 const CliManifestArtifact& artifact,
                                 const std::filesystem::path& artifactDiskPath,
                                 std::string& outError,
                                 bool atomicWrite = true);


} // namespace isocity
