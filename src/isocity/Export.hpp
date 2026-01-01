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


struct PpmDiffStats {
  int width = 0;
  int height = 0;

  // Pixels compared (width*height).
  std::uint64_t pixelsCompared = 0;

  // Pixels whose absolute per-channel difference exceeded `threshold` for at least one channel.
  std::uint64_t pixelsDifferent = 0;

  // Max per-channel absolute difference over all pixels (0..255).
  std::uint8_t maxAbsDiff = 0;

  // Mean absolute difference and mean squared error over channels (denominator = pixelsCompared * 3).
  double meanAbsDiff = 0.0;
  double mse = 0.0;

  // Peak signal-to-noise ratio in dB (computed from MSE). +inf when mse == 0.
  double psnr = 0.0;
};

// Read a binary PPM (P6) file.
// Supports:
//  - whitespace-separated header tokens
//  - comment lines starting with '#'
//  - maxval != 255 (input will be scaled to 0..255)
//
// Returns true on success; on failure, outError contains a human-friendly error.
bool ReadPpm(const std::string& path, PpmImage& outImg, std::string& outError);

// Compare two PPM images.
// - threshold: per-channel absolute tolerance (0 means exact).
// - outDiff (optional): absolute-difference visualization (per-channel |a-b|; zeroed for <=threshold).
//
// Returns false only on invalid inputs (dimension mismatch, bad buffers, etc).
// Use outStats.pixelsDifferent to determine match/mismatch.
bool ComparePpm(const PpmImage& a, const PpmImage& b, PpmDiffStats& outStats, int threshold = 0,
                PpmImage* outDiff = nullptr);

// -----------------------------------------------------------------------------------------------
// PNG IO (minimal, dependency-free)
// -----------------------------------------------------------------------------------------------

// Write a truecolor 8-bit PNG.
//
// The encoder is intentionally minimal:
//  - Color type 2 (RGB), bit depth 8
//  - Filter 0 (None)
//  - DEFLATE stream uses "stored" (uncompressed) blocks for simplicity
//
// Returns true on success; on failure, outError contains a human-friendly error.
bool WritePng(const std::string& path, const PpmImage& img, std::string& outError);

// Read a PNG produced by WritePng.
//
// The decoder supports a small subset of PNG:
//  - Color type 2 (RGB), bit depth 8, no interlace
//  - Filter 0 (None)
//  - Zlib stream with stored (uncompressed) blocks
//
// It is sufficient for deterministic snapshot artifacts and internal tooling.
bool ReadPng(const std::string& path, PpmImage& outImg, std::string& outError);

// Convenience helpers for CLI/tools: read/write either PPM or PNG.
//
// Format selection:
//  - based on filename extension (.ppm/.pnm/.png)
//  - falls back to probing the file magic when reading
bool ReadImageAuto(const std::string& path, PpmImage& outImg, std::string& outError);
bool WriteImageAuto(const std::string& path, const PpmImage& img, std::string& outError);


// -----------------------------------------------------------------------------------------------
// Isometric overview export (headless)
//
// This is intentionally separate from the raylib Renderer so CI/headless tooling can still
// generate a readable city "screenshot" without external dependencies.
// The output is still a PPM (P6) image.
// -----------------------------------------------------------------------------------------------

struct IsoOverviewConfig {
  // Tile pixel size for the top diamond. Must be even numbers (so halfW/halfH are integers).
  int tileW = 16;
  int tileH = 8;

  // Maximum elevation in pixels for tiles with height=1.0.
  // 0 disables vertical relief.
  int heightScalePx = 14;

  // Extra border around the rendered bounds.
  int marginPx = 8;

  // If true, draw simple vertical "cliff" faces when a tile is higher than its east/south neighbor.
  bool drawCliffs = true;

  // If true, draw a thin outline around each tile top diamond.
  bool drawGrid = false;

  // Background color (RGB).
  std::uint8_t bgR = 110;
  std::uint8_t bgG = 160;
  std::uint8_t bgB = 220;
};

struct IsoOverviewResult {
  PpmImage image;

  // Transform info (useful for debugging / tests).
  int tileW = 0;
  int tileH = 0;
  int halfW = 0;
  int halfH = 0;
  int heightScalePx = 0;
  int offsetX = 0; // add to iso-space coords to get image pixels
  int offsetY = 0;
};

// Render an isometric overview image using the same layer coloring as RenderPpmLayer,
// but projected into an isometric diamond grid.
//
// The returned result includes a transform so callers can map tile centers to image pixels
// deterministically (useful for tests, picking, and annotation).
IsoOverviewResult RenderIsoOverview(const World& world, ExportLayer layer, const IsoOverviewConfig& cfg,
                                   const LandValueResult* landValue = nullptr,
                                   const TrafficResult* traffic = nullptr,
                                   const GoodsResult* goods = nullptr);

// Compute the pixel coordinate of a tile center in a rendered isometric overview image.
// Returns false if the tile is out of bounds or the transform is invalid.
bool IsoTileCenterToPixel(const World& world, const IsoOverviewResult& iso, int tx, int ty, int& outPx, int& outPy);

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
