#pragma once

#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Small headless export utilities.
// Intended for:
//  - CLI tooling / batch simulation pipelines
//  - debugging derived fields (traffic, land value, goods)
//  - deterministic regression artifacts (simple PPM images)
//
// We intentionally use the very simple PPM "P6" format so we don't need external
// image dependencies.

enum class ExportLayer : std::uint8_t {
  Terrain = 0,      // base terrain + height shading
  Overlay = 1,      // terrain + roads/zones/parks on top
  Height = 2,       // grayscale height (Tile::height)
  LandValue = 3,    // LandValueResult::value heatmap (requires landValue)
  Traffic = 4,      // TrafficResult::roadTraffic heatmap (requires traffic)
  GoodsTraffic = 5, // GoodsResult::roadGoodsTraffic heatmap (requires goods)
  GoodsFill = 6,    // GoodsResult::commercialFill heatmap (requires goods)
  District = 7,     // per-tile district id palette (v7+ saves)
};

struct PpmImage {
  int width = 0;
  int height = 0;
  // RGB bytes, row-major (y major), size = width * height * 3
  std::vector<std::uint8_t> rgb;
};

// Parse a user-facing layer name (case-insensitive).
// Examples:
//   "terrain", "overlay", "height", "landvalue", "traffic", "goods_traffic", "goods_fill", "district"
bool ParseExportLayer(const std::string& s, ExportLayer& outLayer);

// Return a stable user-facing name for a layer (lowercase).
const char* ExportLayerName(ExportLayer layer);

// Render a one-pixel-per-tile image for the requested layer.
// Some layers require optional derived inputs:
//   - LandValue requires landValue != nullptr
//   - Traffic requires traffic != nullptr
//   - GoodsTraffic / GoodsFill require goods != nullptr
//
// If the required pointer is null, the exporter falls back to a reasonable
// "debug" visualization rather than failing.
PpmImage RenderPpmLayer(const World& world, ExportLayer layer, const LandValueResult* landValue = nullptr,
                        const TrafficResult* traffic = nullptr, const GoodsResult* goods = nullptr);

// Nearest-neighbor upscaling (useful for viewing tiny maps).
// factor <= 1 returns src unchanged.
PpmImage ScaleNearest(const PpmImage& src, int factor);

// Write a binary PPM (P6) file.
// Returns true on success; on failure, outError contains a human-friendly error.
bool WritePpm(const std::string& path, const PpmImage& img, std::string& outError);

// Write a per-tile CSV for debugging / analysis.
//
// Columns:
//   x,y,terrain,overlay,level,district,height,variation,occupants
//
// Returns true on success; on failure, outError contains a human-friendly error.
bool WriteTilesCsv(const World& world, const std::string& path, std::string& outError);

} // namespace isocity
