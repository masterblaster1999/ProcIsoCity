#pragma once

#include <filesystem>
#include <string>

namespace isocity {

// Utility helpers that connect headless outputs (saves) to the rendered
// pipeline (raylib/OpenGL).
//
// This is primarily used by launcher-style modes like --health-check to
// optionally validate that a generated save can be loaded and rendered.

struct RenderOverviewOptions {
  // Binary save to load.
  std::filesystem::path savePath;

  // Output image path (any raylib-supported format, e.g. .png).
  std::filesystem::path outImagePath;

  // Maximum output dimension (the exporter will downscale if needed).
  int maxSize = 4096;

  // Time parameter forwarded into the renderer (drives day/night + weather).
  float timeSec = 0.0f;

  // Include screen-space FX (fog/precip particles) in the export.
  bool includeScreenFx = true;

  // Optional: apply VisualPrefs (renderer visual settings) when exporting.
  //
  // This makes rendered exports match the in-game visuals more closely.
  // When enabled, missing prefs files are treated as non-fatal.
  bool useVisualPrefs = true;

  // Visual prefs JSON path. If empty, defaults to "isocity_visual.json" in the
  // current working directory.
  std::filesystem::path visualPrefsPath;

  // When true, apply conservative overrides (disable some expensive effects)
  // even if the prefs file enables them.
  bool safeMode = false;

  // Renderer settings.
  int tileWidth = 64;
  int tileHeight = 32;
  float elevationScale = 0.75f;
  int elevationSteps = 16;

  // Window/context creation.
  int windowWidth = 1280;
  int windowHeight = 720;
  bool hiddenWindow = true;

  bool verbose = false;
};

struct RenderOverviewResult {
  bool ok = false;
  std::filesystem::path outImagePath;
  std::string report;

  // Diagnostic: whether a visual prefs file was applied.
  bool visualPrefsApplied = false;
  std::filesystem::path visualPrefsPathUsed;
};

// Load a save file and export a rendered full-city overview image.
//
// Returns true on success; otherwise returns false and sets outError.
bool RenderWorldOverviewFromSave(const RenderOverviewOptions& opt,
                                RenderOverviewResult& out,
                                std::string& outError);

} // namespace isocity
