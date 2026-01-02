#include "isocity/Heightmap.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline std::uint8_t ClampU8(int v)
{
  return static_cast<std::uint8_t>(std::clamp(v, 0, 255));
}

// Deterministic integer luma approximation (ITU-R BT.601-ish), weights sum to 256.
inline std::uint8_t Luma8(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  const std::uint32_t acc = 77u * r + 150u * g + 29u * b + 128u;
  return static_cast<std::uint8_t>(acc >> 8);
}

inline std::uint8_t GetLuma8Clamped(const PpmImage& img, int x, int y)
{
  x = std::clamp(x, 0, img.width - 1);
  y = std::clamp(y, 0, img.height - 1);
  const std::size_t idx = (static_cast<std::size_t>(y) * img.width + x) * 3;
  const std::uint8_t r = img.rgb[idx + 0];
  const std::uint8_t g = img.rgb[idx + 1];
  const std::uint8_t b = img.rgb[idx + 2];
  return Luma8(r, g, b);
}

inline float SampleNearestGray01(const PpmImage& img, double u, double v)
{
  // u,v are expected in [0,1].
  int x = static_cast<int>(u * img.width);
  int y = static_cast<int>(v * img.height);

  // In case of numeric drift, clamp.
  x = std::clamp(x, 0, img.width - 1);
  y = std::clamp(y, 0, img.height - 1);

  return static_cast<float>(GetLuma8Clamped(img, x, y)) / 255.0f;
}

inline float SampleBilinearGray01(const PpmImage& img, double u, double v)
{
  // Map u,v (0..1) to source pixel space in a center-aligned way.
  const double fx = u * img.width - 0.5;
  const double fy = v * img.height - 0.5;

  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const double tx = fx - x0;
  const double ty = fy - y0;

  const double a = static_cast<double>(GetLuma8Clamped(img, x0, y0));
  const double b = static_cast<double>(GetLuma8Clamped(img, x1, y0));
  const double c = static_cast<double>(GetLuma8Clamped(img, x0, y1));
  const double d = static_cast<double>(GetLuma8Clamped(img, x1, y1));

  const double ab = a + (b - a) * tx;
  const double cd = c + (d - c) * tx;
  const double v8 = ab + (cd - ab) * ty;

  return static_cast<float>(v8 / 255.0);
}

inline float ApplyScaleOffsetClamp(float gray01, const HeightmapApplyConfig& cfg)
{
  if (cfg.invert) gray01 = 1.0f - gray01;

  float h = gray01 * cfg.heightScale + cfg.heightOffset;
  if (cfg.clamp01) h = std::clamp(h, 0.0f, 1.0f);
  return h;
}

inline Terrain ClassifyTerrainFromHeight(float h, const HeightmapApplyConfig& cfg)
{
  if (h < cfg.waterLevel) return Terrain::Water;
  if (h < cfg.sandLevel) return Terrain::Sand;
  return Terrain::Grass;
}

} // namespace

bool ApplyHeightmap(World& world, const PpmImage& img, const HeightmapApplyConfig& cfg, std::string& outError,
                    HeightmapApplyStats* outStats)
{
  outError.clear();

  if (world.width() <= 0 || world.height() <= 0) {
    outError = "World has invalid dimensions.";
    return false;
  }
  if (img.width <= 0 || img.height <= 0) {
    outError = "Input image has invalid dimensions.";
    return false;
  }
  if (img.rgb.size() != static_cast<std::size_t>(img.width) * img.height * 3) {
    outError = "Input image buffer size does not match width/height.";
    return false;
  }

  if (cfg.resample == HeightmapResample::None) {
    if (img.width != world.width() || img.height != world.height()) {
      outError = "Image dimensions do not match world dimensions. Use --resample to fit.";
      return false;
    }
  }

  const int w = world.width();
  const int h = world.height();

  // Stats accumulation.
  float minH = std::numeric_limits<float>::infinity();
  float maxH = -std::numeric_limits<float>::infinity();
  double sumH = 0.0;
  double sumSq = 0.0;

  std::uint64_t waterTiles = 0;
  std::uint64_t sandTiles = 0;
  std::uint64_t grassTiles = 0;
  std::uint64_t overlaysCleared = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      // Sample grayscale from the input image.
      float gray01 = 0.0f;

      if (cfg.resample == HeightmapResample::None) {
        const int sx = cfg.flipX ? (img.width - 1 - x) : x;
        const int sy = cfg.flipY ? (img.height - 1 - y) : y;
        gray01 = static_cast<float>(GetLuma8Clamped(img, sx, sy)) / 255.0f;
      } else {
        // Normalized UV in [0,1).
        double u = (static_cast<double>(x) + 0.5) / static_cast<double>(w);
        double v = (static_cast<double>(y) + 0.5) / static_cast<double>(h);

        if (cfg.flipX) u = 1.0 - u;
        if (cfg.flipY) v = 1.0 - v;

        u = std::clamp(u, 0.0, 1.0);
        v = std::clamp(v, 0.0, 1.0);

        if (cfg.resample == HeightmapResample::Nearest) {
          gray01 = SampleNearestGray01(img, u, v);
        } else {
          gray01 = SampleBilinearGray01(img, u, v);
        }
      }

      const float newHeight = ApplyScaleOffsetClamp(gray01, cfg);

      Tile& t = world.at(x, y);
      t.height = newHeight;

      if (cfg.reclassifyTerrain) {
        const Terrain newTerr = ClassifyTerrainFromHeight(newHeight, cfg);
        t.terrain = newTerr;

        if (newTerr == Terrain::Water && cfg.bulldozeNonRoadOverlaysOnWater) {
          if (t.overlay != Overlay::None && t.overlay != Overlay::Road) {
            world.setOverlay(Overlay::None, x, y);
            overlaysCleared++;
          }
        }
      }

      minH = std::min(minH, newHeight);
      maxH = std::max(maxH, newHeight);
      sumH += static_cast<double>(newHeight);
      sumSq += static_cast<double>(newHeight) * static_cast<double>(newHeight);

      switch (t.terrain) {
      case Terrain::Water: waterTiles++; break;
      case Terrain::Sand: sandTiles++; break;
      case Terrain::Grass: grassTiles++; break;
      default: break;
      }
    }
  }

  // Recompute road masks in case the tool cleared overlays in bulk.
  world.recomputeRoadMasks();

  const double n = static_cast<double>(w) * static_cast<double>(h);
  const double mean = (n > 0.0) ? (sumH / n) : 0.0;
  const double var = (n > 0.0) ? std::max(0.0, (sumSq / n) - mean * mean) : 0.0;

  if (outStats) {
    outStats->worldW = w;
    outStats->worldH = h;
    outStats->srcW = img.width;
    outStats->srcH = img.height;

    outStats->minHeight = std::isfinite(minH) ? minH : 0.0f;
    outStats->maxHeight = std::isfinite(maxH) ? maxH : 0.0f;
    outStats->meanHeight = mean;
    outStats->stdevHeight = std::sqrt(var);

    outStats->waterTiles = waterTiles;
    outStats->sandTiles = sandTiles;
    outStats->grassTiles = grassTiles;

    outStats->overlaysCleared = overlaysCleared;
  }

  return true;
}

PpmImage ExportHeightmapImage(const World& world, const HeightmapExportConfig& cfg, float* outMinHeight,
                             float* outMaxHeight)
{
  PpmImage img;
  img.width = world.width();
  img.height = world.height();
  img.rgb.assign(static_cast<std::size_t>(img.width) * img.height * 3, 0);

  float minH = std::numeric_limits<float>::infinity();
  float maxH = -std::numeric_limits<float>::infinity();

  // First pass: compute raw min/max if needed.
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const float h = world.at(x, y).height;
      minH = std::min(minH, h);
      maxH = std::max(maxH, h);
    }
  }

  if (outMinHeight) *outMinHeight = std::isfinite(minH) ? minH : 0.0f;
  if (outMaxHeight) *outMaxHeight = std::isfinite(maxH) ? maxH : 0.0f;

  const float denom = (maxH - minH);
  const bool canNormalize = cfg.normalize && std::isfinite(denom) && std::fabs(denom) > 1e-8f;

  // Second pass: write pixels.
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      float v = world.at(x, y).height;

      if (canNormalize) {
        v = (v - minH) / denom;
      }

      if (cfg.clamp01) v = std::clamp(v, 0.0f, 1.0f);
      if (cfg.invert) v = 1.0f - v;

      const int q = static_cast<int>(std::lround(static_cast<double>(v) * 255.0));
      const std::uint8_t c = ClampU8(q);

      const std::size_t idx = (static_cast<std::size_t>(y) * img.width + x) * 3;
      img.rgb[idx + 0] = c;
      img.rgb[idx + 1] = c;
      img.rgb[idx + 2] = c;
    }
  }

  return img;
}

} // namespace isocity
