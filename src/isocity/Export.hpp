#pragma once

#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/MeshExport.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

struct SeaFloodResult;
struct DepressionFillResult;

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

  // Sea-level coastal flooding depth (derived from heightfield + edge connectivity).
  FloodDepth = 8,

  // Depression-fill depth (Priority-Flood): ponding potential in closed basins.
  PondingDepth = 9,

  // Public services / civic accessibility satisfaction (0..1).
  ServicesOverall = 10,
  ServicesEducation = 11,
  ServicesHealth = 12,
  ServicesSafety = 13,
};

// Back-compat alias: older UI code refers to the 2D export layer enum as ExportLayer2D.
using ExportLayer2D = ExportLayer;


struct PpmImage {
  int width = 0;
  int height = 0;
  // RGB bytes, row-major (y major), size = width * height * 3
  std::vector<std::uint8_t> rgb;
};


struct RgbaImage {
  int width = 0;
  int height = 0;
  // RGBA bytes, row-major (y major), size = width * height * 4
  std::vector<std::uint8_t> rgba;
};

// -----------------------------------------------------------------------------------------------
// PNG IO (RGBA, minimal, dependency-free)
// -----------------------------------------------------------------------------------------------
//
// This mirrors the existing RGB-only PNG support, but produces an RGBA buffer.
//
// The encoder remains intentionally tiny and always writes:
//  - bit depth 8, no interlace
//  - per-row PNG filters (0..4), chosen by a cheap deterministic heuristic
//  - zlib stream with fixed-Huffman DEFLATE blocks (no external zlib dependency)
//
// The decoder is also intentionally tiny. It can read the subset produced by our tools:
//  - color type 6 (RGBA), 2 (RGB), or 3 (indexed with PLTE + optional tRNS)
//  - bit depth 8, no interlace
//  - filter 0..4
//  - zlib stream with stored/fixed/dynamic DEFLATE blocks
//
// Useful for generated sprite atlases / tilesets where preserving alpha is important.
bool WritePngRGBA(const std::string& path, const RgbaImage& img, std::string& outError);
bool ReadPngRGBA(const std::string& path, RgbaImage& outImg, std::string& outError);

// -----------------------------------------------------------------------------------------------
// PNG IO (indexed color, palette + optional per-entry alpha)
// -----------------------------------------------------------------------------------------------
//
// This is useful for procedural sprite atlases intended for distribution/modding.
// Because our PNG encoder uses proper DEFLATE compression (while staying dependency-free),
// indexed-color output can dramatically reduce file size vs RGBA.
//
// The palette is provided as RGBA bytes: paletteRgba.size() must be 4 * paletteSize,
// where paletteSize is in [1, 256]. Pixels are 8-bit palette indices.
//
// Notes:
//  - The encoder writes:
//      * color type 3 (indexed)
//      * PLTE chunk for RGB
//      * tRNS chunk for alpha (one byte per palette entry)
//      * per-row PNG filters (0..4)
//      * fixed-Huffman DEFLATE in the zlib stream
bool WritePngIndexed(const std::string& path, int width, int height,
                     const std::vector<std::uint8_t>& indices,
                     const std::vector<std::uint8_t>& paletteRgba,
                     std::string& outError);

// Composite an RGBA image over a solid RGB background.
PpmImage CompositeOverSolid(const RgbaImage& img, std::uint8_t bgR, std::uint8_t bgG, std::uint8_t bgB);


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

  // Structural Similarity Index (SSIM) computed on luma (Y') in [0,1].
  // Range is typically [-1,1], where 1 means identical.
  //
  // We compute SSIM using a local window for robustness to small local
  // differences. The default ComparePpm() window is 11x11, but callers can
  // override it.
  double ssim = 1.0;
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
                PpmImage* outDiff = nullptr, int ssimWindow = 11);

// -----------------------------------------------------------------------------------------------
// PNG IO (minimal, dependency-free)
// -----------------------------------------------------------------------------------------------

// Write a truecolor 8-bit PNG.
//
// The encoder is intentionally minimal:
//  - Color type 2 (RGB), bit depth 8
//  - Per-row PNG filters (0..4)
//  - DEFLATE stream uses fixed Huffman codes with a small built-in LZ77 matcher
//
// Returns true on success; on failure, outError contains a human-friendly error.
bool WritePng(const std::string& path, const PpmImage& img, std::string& outError);

// Read a PNG produced by WritePng.
//
// The decoder supports a small subset of PNG:
//  - Color type 2 (RGB), bit depth 8, no interlace
//  - Filter 0..4
//  - Zlib stream with stored/fixed/dynamic DEFLATE blocks
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
  // Tile pixel size for the top diamond.
  //
  // The headless isometric exporter supports both even and odd sizes; smaller tiles render
  // faster but reduce detail in fancy mode.
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

  // If true, render Terrain/Overlay isometric exports with per-pixel procedural detail:
  //  - subtle terrain texture (grass/sand/water)
  //  - shoreline foam/highlights
  //  - proper road geometry with lane markings/crosswalks
  //  - simple zone/park patterns
  //
  // This is purely a visualization feature; analysis layers (height/traffic/etc) can
  // disable it for cleaner heatmaps.
  bool fancy = true;

  // Strength of micro texture noise in fancy mode (0 disables). Typical range: [0, 1].
  float textureStrength = 1.0f;

  // Fancy-mode toggles.
  bool drawShore = true;
  bool drawRoadMarkings = true;
  bool drawZonePatterns = true;

  // -------------------------------------------------------------------------------------------
  // Optional cinematic / atmospheric styling (Terrain/Overlay iso exports only)
  //
  // These controls are intentionally *off* by default so existing tooling and regression
  // artifacts remain stable. When enabled, they allow the headless iso exporter to match
  // the in-app renderer more closely (day/night grading, city lights, weather styling).
  // -------------------------------------------------------------------------------------------

  struct DayNightConfig {
    // Enable day/night grading + optional emissive lights pass.
    bool enabled = false;

    // Cycle phase in [0,1]:
    //   0.00 = sunrise, 0.25 = noon, 0.50 = sunset, 0.75 = midnight.
    // This matches the in-app sine sun curve.
    float phase01 = 0.25f;

    // How strongly to darken at night (0..1). Similar to Renderer::DayNightSettings::nightDarken.
    float nightDarken = 0.80f;

    // Warm tint strength around dawn/dusk (0..1). Similar to Renderer::DayNightSettings::duskTint.
    float duskTint = 0.55f;

    // Draw an emissive lights pass (streetlights + window glows) when night > ~5%.
    bool drawLights = true;

    // Extra intensity multiplier for the lights pass.
    float lightStrength = 1.0f;
  } dayNight;

  struct WeatherConfig {
    enum class Mode : std::uint8_t { Clear = 0, Rain = 1, Snow = 2 };

    Mode mode = Mode::Clear;

    // Overall intensity (0..1). Interpreted as wetness (rain) or snow cover (snow).
    float intensity = 0.0f;

    // Overcast grade strength (0..1). Used to desaturate/dim and soften contrast.
    float overcast = 0.60f;

    // Atmospheric haze (0..1) applied as a simple top-of-image fog gradient.
    float fog = 0.0f;

    // Draw precipitation particles (rain streaks / snow flakes).
    bool drawPrecipitation = true;

    // When raining and lights are enabled, draw simple reflections/smears.
    bool reflectLights = true;

    // Wind angle in degrees (used to orient rain streaks). 0 = +X, 90 = +Y.
    float windAngleDeg = 35.0f;
  } weather;

  struct CloudConfig {
    // Enable moving cloud shadows (dappled sunlight) based on deterministic procedural noise.
    // Note: shadows are only visible when dayNight is enabled and the sun is above the horizon.
    bool enabled = false;

    // Fraction of the sky covered by shadow-casting clouds (0..1).
    float coverage = 0.45f;

    // Darkness of the cloud shadows (0..1).
    float strength = 0.45f;

    // Approximate cloud feature size in *tiles* (higher = larger, smoother patches).
    float scaleTiles = 26.0f;

    // Optional deterministic offset in tile units (lets you "slide" the cloud pattern).
    float offsetX = 0.0f;
    float offsetY = 0.0f;
  } clouds;


  // -------------------------------------------------------------------------------------------
  // Tileset-atlas rendering extras (only used when RenderIsoOverview is given a GfxTilesetAtlas)
  // -------------------------------------------------------------------------------------------

  // Optional per-pixel normal-mapped lighting for atlas-sprite rendering.
  //
  // The normal map is expected to be a tangent-space RGB normal map where:
  //   - R encodes X, G encodes Y, B encodes Z
  //   - (128,128,255) is the flat/default normal
  //   - Positive Y (green) points "up" in texture space (OpenGL-style) rather than "down".
  struct TilesetLightingConfig {
    // Enable normal-mapped lighting when a matching normal atlas is provided.
    bool enableNormals = false;

    // Key light direction in normal-map space:
    //   +X = right, +Y = up, +Z = out of the surface.
    // This will be normalized at use time.
    float lightDirX = -0.62f;
    float lightDirY = 0.55f;
    float lightDirZ = 0.56f;

    // Lighting weights (ambient + diffuse ~= 1 recommended).
    float ambient = 0.35f;
    float diffuse = 0.65f;

    // Overall blend strength in [0,1].
    //  0 => no effect (flat sprite shading)
    //  1 => full normal-mapped lighting
    float normalStrength = 1.0f;

    // Enable multiplicative drop shadows when a matching shadow atlas is provided.
    bool enableShadows = false;

    // Shadow darkening in [0,1].
    float shadowStrength = 0.65f;
  } tilesetLighting;

  // Optional deterministic decorative props for atlas-sprite rendering.
  // This is *only* used when a tileset atlas includes props and the layer is Overlay.
  struct TilesetPropsConfig {
    bool enabled = false;

    // Park tree density in [0,1]. Higher values place more trees per park tile.
    float treeDensity = 0.35f;

    // Chance in [0,1] that a placed tree is a conifer (otherwise deciduous).
    float coniferChance = 0.35f;

    // If true, place streetlight props on some road tiles.
    bool drawStreetlights = true;

    // Streetlight placement chance in [0,1] (only evaluated on road tiles).
    float streetlightChance = 0.30f;
  } tilesetProps;


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

// Optional binding: a generated tileset atlas (from proc_isocity_tileset).
// When provided, RenderIsoOverview may use sprite blitting instead of per-pixel procedural drawing
// for Terrain/Overlay layers.
struct GfxTilesetAtlas;

// Render an isometric overview image using the same layer coloring as RenderPpmLayer,
// but projected into an isometric diamond grid.
//
// The returned result includes a transform so callers can map tile centers to image pixels
// deterministically (useful for tests, picking, and annotation).
IsoOverviewResult RenderIsoOverview(const World& world, ExportLayer layer, const IsoOverviewConfig& cfg,
                                   const LandValueResult* landValue = nullptr,
                                   const TrafficResult* traffic = nullptr,
                                   const GoodsResult* goods = nullptr,
                                   const GfxTilesetAtlas* tileset = nullptr);

// Compute the pixel coordinate of a tile center in a rendered isometric overview image.
// Returns false if the tile is out of bounds or the transform is invalid.
bool IsoTileCenterToPixel(const World& world, const IsoOverviewResult& iso, int tx, int ty, int& outPx, int& outPy);


// -----------------------------------------------------------------------------------------------
// 3D software render export (headless)
//
// This produces a shaded 3D render (orthographic/isometric or perspective) using a
// tiny CPU rasterizer. It's meant for CLI tooling, regression snapshots, and
// batch pipelines. (It is *not* intended as a high-performance runtime renderer.)
// -----------------------------------------------------------------------------------------------

struct Render3DConfig {
  enum class Projection : std::uint8_t { IsometricOrtho = 0, Perspective = 1 };

  // Output image size.
  int width = 1600;
  int height = 900;

  // Camera.
  Projection projection = Projection::IsometricOrtho;
  bool autoFit = true;
  float fitMargin = 0.08f;

  // Angles in degrees. Classic isometric: yaw=45, pitch=35.264.
  float yawDeg = 45.0f;
  float pitchDeg = 35.264f;
  float rollDeg = 0.0f;

  // Used only when autoFit=false.
  float targetX = 0.0f;
  float targetY = 0.0f;
  float targetZ = 0.0f;
  float distance = 120.0f;

  // Projection parameters.
  float fovYDeg = 45.0f;        // perspective
  float orthoHalfHeight = 20.0f; // isometric orthographic

  // Quality/styling.
  int supersample = 1; // SSAA (1=off)
  bool drawOutlines = true;
  std::uint8_t outlineR = 0;
  std::uint8_t outlineG = 0;
  std::uint8_t outlineB = 0;
  float outlineDepthEps = 0.002f;
  // Blend factor for outlines over the filled surface.
  // 1.0 = fully opaque outlines, 0.0 = disabled.
  float outlineAlpha = 1.0f;

  // Lighting (Lambert).
  float lightDirX = -0.55f;
  float lightDirY = 0.80f;
  float lightDirZ = -0.25f;
  float ambient = 0.35f;
  float diffuse = 0.65f;

  // Background clear.
  std::uint8_t bgR = 30;
  std::uint8_t bgG = 32;
  std::uint8_t bgB = 42;

  // Fog blend target color (RGB) used when fog is enabled.
  // Defaults to a cool gray independent of the background clear color.
  std::uint8_t fogR = 200;
  std::uint8_t fogG = 210;
  std::uint8_t fogB = 225;

  // Simple depth fog.
  bool fog = false;
  float fogStrength = 0.35f;
  float fogStart = 0.35f; // [0..1] depth
  float fogEnd = 1.0f;



  // --- Environment (optional) ---
  //
  // If timeOfDay is in [0,1], RenderWorld3D will derive a reasonable sun direction and
  // sky/fog defaults from it. If < 0 (default), explicit lightDir/bg/fog settings are used.
  //
  // timeOfDay convention: 0=midnight, 0.25=dawn, 0.5=noon, 0.75=dusk.
  float timeOfDay = -1.0f;

  // Wind direction in degrees (0=east, 90=north). Used for future cloud/rain drift.
  float windDirDeg = 0.0f;

  // Weather strengths in [0,1].
  float rainStrength = 0.0f;
  float cloudStrength = 0.0f;
  // --- Post-processing (software renderer only) ---
  //
  // These are optional "style" improvements that run after rasterization.
  // They are off by default (except gamma-correct SSAA resolve) to keep
  // backwards-compatible output.
  bool gammaCorrectDownsample = true;

  // Screen-space AO approximation (depth only).
  bool postAO = false;
  float aoStrength = 0.55f;
  int aoRadiusPx = 7;
  float aoRange = 0.02f;
  float aoBias = 0.0015f;
  float aoPower = 1.25f;
  int aoSamples = 12;
  int aoBlurRadiusPx = 1;

  // Depth-edge outlines.
  bool postEdge = false;
  float edgeAlpha = 0.90f;
  float edgeThreshold = 0.004f;
  float edgeSoftness = 0.003f;
  int edgeRadiusPx = 1;
  std::uint8_t edgeR = 0;
  std::uint8_t edgeG = 0;
  std::uint8_t edgeB = 0;

  // Tonemap / grade.
  bool postTonemap = false;
  float exposure = 1.0f;
  float contrast = 1.0f;
  float saturation = 1.0f;
  float vignette = 0.0f;



  // Convenience toggles/params used by the in-game 3D preview UI.
  // These are mapped onto the underlying post pipeline by RenderWorld3D.
  bool postGrade = false;
  bool postVignette = false;

  bool postBloom = false;
  float postBloomStrength = 0.18f;
  float postBloomRadius = 0.80f;

  // If >= 0, overrides ditherStrength (useful for UI sliders); otherwise ditherStrength is used.
  float postDitherStrength = -1.0f;
  // Ordered dithering + quantization.
  bool postDither = false;
  float ditherStrength = 0.35f;
  int ditherBits = 6;

  // Seed for deterministic post jitter/patterns (0 = derive from world seed).
  std::uint32_t postSeed = 0;

  // Terrain surface mode.
  //
  // When disabled (default), top surfaces are generated as flat per-tile quads.
  // When enabled, top surfaces are generated as a continuous heightfield using
  // per-tile-corner heights (averaged from neighboring tiles). This produces
  // smoother slopes in the software 3D renderer.
  bool heightfieldTopSurfaces = false;

  // Add a terrain "skirt" around the exported bounds to visually close the mesh
  // (a common trick for heightfields in engines).
  //
  // When enabled, vertical walls are generated along the crop/world boundary
  // down to (minHeight - skirtDrop).
  bool addSkirt = false;
  float skirtDrop = 6.0f;

  // Mesh extraction parameters (world units).
  //
  // Note: RenderWorld3D always produces per-tile *top* surfaces colored using
  // RenderPpmLayer(...) to support heatmaps. The MeshExportConfig is used for
  // height scaling + quantization, cropping, and for generating cliffs/buildings.
  MeshExportConfig meshCfg{};
};

// Render a shaded 3D view of the current world using ExportLayer coloring for
// tile top surfaces.
//
// If layer requires derived fields, the corresponding pointers should be non-null;
// otherwise RenderPpmLayer fallbacks will be used (same behavior as 2D exports).
PpmImage RenderWorld3D(const World& world, ExportLayer layer, const Render3DConfig& cfg,
                       const LandValueResult* landValue = nullptr,
                       const TrafficResult* traffic = nullptr,
                       const GoodsResult* goods = nullptr);

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

struct TileMetricsCsvInputs {
  const LandValueResult* landValue = nullptr;
  const TrafficResult* traffic = nullptr;
  const GoodsResult* goods = nullptr;
  const SeaFloodResult* seaFlood = nullptr;
  const DepressionFillResult* ponding = nullptr;
};

struct TileMetricsCsvOptions {
  bool includeLandValue = true;
  bool includeLandValueComponents = true;
  bool includeTraffic = true;
  bool includeGoods = true;
  bool includeFlood = true;
  bool includePonding = true;

  // Float precision for heights/metrics.
  int floatPrecision = 6;
};

// Write a per-tile CSV containing base tile fields plus optional derived metrics.
//
// Base columns (always written):
//   x,y,terrain,overlay,level,district,height,variation,occupants
//
// Optional metric columns (enabled by TileMetricsCsvOptions and present when the
// corresponding input pointer is non-null):
//   - land value: land_value,park_amenity,water_amenity,pollution,traffic_penalty
//   - traffic: commute_traffic
//   - goods: goods_traffic,goods_fill
//   - flood: flooded,flood_depth
//   - ponding: ponding_depth
//
// Returns true on success; on failure, outError contains a human-friendly error.
bool WriteTileMetricsCsv(const World& world, const std::string& path, std::string& outError,
                         const TileMetricsCsvInputs& inputs = {},
                         const TileMetricsCsvOptions& opt = {});

} // namespace isocity
