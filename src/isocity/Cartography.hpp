#pragma once

#include "isocity/Export.hpp"      // RgbaImage, ExportLayer, IsoOverviewConfig
#include "isocity/StreetNames.hpp" // StreetNamingConfig

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

class World;

// -----------------------------------------------------------------------------------------------
// Deterministic naming helpers
// -----------------------------------------------------------------------------------------------
//
// These are used by the Cartography poster renderer, but are also useful for other
// procedural "semantic layers" (tour guides, narrative generators, UI overlays).
//
// NOTE: Naming is derived (not persisted). It is expected to be stable for a given
// world seed and tile content.

// Generate a stable, pronounceable city name from a seed.
std::string GenerateCityName(std::uint64_t seed);

// Generate stable district names (size == kDistrictCount).
std::vector<std::string> GenerateDistrictNames(const World& world);

// Lightweight deterministic cartography helpers.
//
// Goal: turn a generated/loaded world into a "shareable" labeled map image
// (no raylib, no external fonts). This is useful for:
//  - quick visual regression artifacts in CI
//  - printing / sharing "city posters" outside the game
//  - GIS-style debugging overlays that need stable label placement

enum class MapLabelKind : std::uint8_t {
  Title = 0,
  District = 1,
  Street = 2,
};

struct MapLabel {
  MapLabelKind kind = MapLabelKind::Street;
  int id = -1; // district id or street id when applicable
  std::string text;

  // Placed bounding box in pixels (top-left origin).
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;

  // Anchor position (often the underlying tile/district centroid).
  int anchorX = 0;
  int anchorY = 0;

  // Built-in 5x7 font scale used for this label.
  int scale = 1;
};

struct CartographyConfig {
  // If true, output includes margins + title and a small legend.
  bool poster = true;
  int marginTopPx = 72;
  int marginSidePx = 16;
  int marginBottomPx = 16;

  // Labels.
  bool labelTitle = true;
  bool labelDistricts = true;
  bool labelStreets = true;

  // Visual aids.
  bool drawDistrictBoundaries = true;

  // Limits.
  int maxStreetLabels = 36;
  int maxDistrictLabels = 8;

  // Text scales for the built-in 5x7 font.
  int streetTextScale = 2;
  int districtTextScale = 3;
  int titleTextScale = 4;

  // Background boxes behind text labels.
  bool labelBackground = true;
  int labelPaddingPx = 2;
  std::uint8_t labelBgAlpha = 110;

  // Override poster title. If empty, a deterministic name is generated.
  std::string titleOverride;
};

struct CartographyResult {
  RgbaImage image; // RGBA poster/map output
  std::string title;
  std::vector<std::string> districtNames; // size=kDistrictCount
  std::vector<MapLabel> labels;           // title + district + street labels
};

// Render a labeled isometric overview poster.
//
// - The base isometric render comes from RenderIsoOverview.
// - Street labels use StreetNames::BuildStreetNames.
// - District labels are generated deterministically from per-district features.
CartographyResult RenderLabeledIsoPoster(const World& world,
                                        ExportLayer layer,
                                        const IsoOverviewConfig& isoCfg,
                                        const StreetNamingConfig& streetCfg,
                                        const CartographyConfig& cfg);

} // namespace isocity
