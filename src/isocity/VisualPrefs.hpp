#pragma once

#include "isocity/Json.hpp"
#include "isocity/Renderer.hpp"

#include <cstdint>
#include <string>

namespace isocity {

// Lightweight, persisted UI theme + font tuning.
//
// This is intentionally decoupled from the simulation/save-state so users can keep
// their preferred UI look across worlds.
struct UiThemePrefs {
  // Accent selection.
  bool accentFromSeed = true;
  float accentHueDeg = 210.0f;     // 0..360 (used when accentFromSeed=false)
  float accentSaturation = 0.55f;  // 0..1
  float accentValue = 0.95f;       // 0..1

  // Panel geometry + effects.
  float roundness = 0.18f; // 0..1
  int roundSegments = 8;

  float noiseAlpha = 0.06f;          // 0..1
  float noiseScale = 0.75f;          // 0.05..4
  float headerSheenStrength = 0.35f; // 0..1

  // Font atlas generation.
  int fontAtlasScale = 3;       // 1..8
  bool fontFilterPoint = false; // point vs bilinear
};

// User-facing persisted preferences for display + renderer visual settings.
//
// Goals:
//  - Make the richer visual/graphics stack (layer masks, cache, day/night, weather, shadows)
//    feel like a first-class part of the game.
//  - Keep the format small, dependency-free (uses our minimal Json parser), and merge-friendly:
//    missing fields leave existing values untouched.
//
// The preferences are intentionally separate from save files: they represent user taste / hardware
// settings rather than simulation state.
struct VisualPrefs {
  // --- Display / UI ---
  bool vsync = true;

  bool uiScaleAuto = true;
  float uiScaleManual = 1.0f; // used when uiScaleAuto=false

  UiThemePrefs uiTheme{};

  // --- World render resolution scaling (off-screen render target) ---
  bool worldRenderScaleAuto = false;
  float worldRenderScale = 1.0f;
  float worldRenderScaleMin = 0.70f;
  float worldRenderScaleMax = 1.00f;
  int worldRenderTargetFps = 60;
  bool worldRenderFilterPoint = false;

  // --- World visuals ---
  bool mergedZoneBuildings = true;

  // --- Renderer toggles ---
  bool baseCacheEnabled = true;
  std::uint32_t layerMask = Renderer::kLayerAll;

  Renderer::ShadowSettings shadows{};
  Renderer::DayNightSettings dayNight{};
  Renderer::WeatherSettings weather{};

  // Atmospheric ambience (optional). These are visual-only and do not affect simulation.
  Renderer::CloudShadowSettings cloudShadows{};
  Renderer::VolumetricCloudSettings volumetricClouds{};

  // Elevation rendering (visual-only).
  ElevationSettings elevation{};
};

// Approximate equality (for change detection / autosave throttling).
bool VisualPrefsEqual(const VisualPrefs& a, const VisualPrefs& b);

// Serialize to a pretty-printed JSON string.
std::string VisualPrefsToJson(const VisualPrefs& p, int indentSpaces = 2);

// Apply JSON overrides into an existing prefs struct (merge semantics).
bool ApplyVisualPrefsJson(const JsonValue& root, VisualPrefs& ioPrefs, std::string& outError);

// File helpers.
bool LoadVisualPrefsJsonFile(const std::string& path, VisualPrefs& ioPrefs, std::string& outError);
bool WriteVisualPrefsJsonFile(const std::string& path, const VisualPrefs& prefs, std::string& outError,
                              int indentSpaces = 2);

} // namespace isocity
