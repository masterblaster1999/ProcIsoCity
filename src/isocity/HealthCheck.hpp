#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace isocity {

// Headless runtime validation used by the launcher executable.
//
// Motivation:
//  - Players often report "the game won't start" without a console or useful logs.
//  - This provides a single command that validates the most failure-prone pipeline pieces:
//      * writable data dir
//      * procedural generation
//      * simulation stepping
//      * save -> load roundtrip (including CRC)
//
// The check is intentionally renderer-free (no raylib/OpenGL) so it can run on machines
// where graphics initialization fails.

struct HealthCheckOptions {
  // Base directory where a work folder will be created.
  // If empty, defaults to the current working directory.
  std::filesystem::path baseDir;

  // Prefix for the created folder name.
  // Final directory name is: <dirPrefix>_YYYYMMDD_HHMMSSZ
  std::string dirPrefix = "healthcheck";

  // World size (tiles).
  int width = 64;
  int height = 64;

  // Seed used for procedural generation.
  std::uint64_t seed = 1;

  // Number of simulation steps to advance.
  int steps = 12;

  // Keep temporary artifacts on disk (work folder, save file, etc.).
  bool keepArtifacts = false;

  // Include more verbose timings and file details in the report.
  bool verbose = false;
};

struct HealthCheckResult {
  bool ok = false;
  std::filesystem::path workDir;

  // Path to the generated binary save file written during the check.
  //
  // This is useful for launcher/tooling code that wants to validate the
  // *rendered* pipeline by loading + drawing the saved world after the headless
  // phase.
  std::filesystem::path savePath;

  std::vector<std::filesystem::path> artifacts;
  std::string report;
};

// Runs the health check.
//
// On success, returns true and sets out.ok=true.
// On failure, returns false (out.ok may still be false) and sets outError.
bool RunHealthCheck(const HealthCheckOptions& opt, HealthCheckResult& out, std::string& outError);

} // namespace isocity
