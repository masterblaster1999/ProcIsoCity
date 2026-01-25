#include "isocity/WorldDiffViz.hpp"

#include <algorithm>
#include <cmath>

namespace isocity {

namespace {

inline bool TileFieldDiffers(const Tile& ta, const Tile& tb, float eps,
                             bool* outTerrain, bool* outOverlay, bool* outHeight,
                             bool* outVariation, bool* outLevel, bool* outOccupants, bool* outDistrict)
{
  const bool terrainDiff = ta.terrain != tb.terrain;
  const bool overlayDiff = ta.overlay != tb.overlay;
  const bool heightDiff = std::fabs(ta.height - tb.height) > eps;
  const bool variationDiff = ta.variation != tb.variation;
  const bool levelDiff = ta.level != tb.level;
  const bool occupantsDiff = ta.occupants != tb.occupants;
  const bool districtDiff = ta.district != tb.district;

  if (outTerrain) *outTerrain = terrainDiff;
  if (outOverlay) *outOverlay = overlayDiff;
  if (outHeight) *outHeight = heightDiff;
  if (outVariation) *outVariation = variationDiff;
  if (outLevel) *outLevel = levelDiff;
  if (outOccupants) *outOccupants = occupantsDiff;
  if (outDistrict) *outDistrict = districtDiff;

  return terrainDiff || overlayDiff || heightDiff || variationDiff || levelDiff || occupantsDiff || districtDiff;
}

inline void PutRgb(PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  const std::size_t i = (static_cast<std::size_t>(y) * img.width + x) * 3u;
  img.rgb[i + 0] = r;
  img.rgb[i + 1] = g;
  img.rgb[i + 2] = b;
}

} // namespace

WorldDiffBounds ComputeWorldDiffBounds(const World& a, const World& b, float heightEps)
{
  WorldDiffBounds out{};
  out.widthA = a.width();
  out.heightA = a.height();
  out.widthB = b.width();
  out.heightB = b.height();
  out.sizeMismatch = (out.widthA != out.widthB) || (out.heightA != out.heightB);

  const int w = std::min(out.widthA, out.widthB);
  const int h = std::min(out.heightA, out.heightB);
  out.overlapW = std::max(0, w);
  out.overlapH = std::max(0, h);

  if (w <= 0 || h <= 0) {
    out.anyDifferent = false;
    out.tilesDifferent = 0;
    out.minX = out.minY = out.maxX = out.maxY = 0;
    return out;
  }

  const float eps = std::max(0.0f, heightEps);

  int minX = w;
  int minY = h;
  int maxX = 0;
  int maxY = 0;
  int tilesDifferent = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& ta = a.at(x, y);
      const Tile& tb = b.at(x, y);
      if (TileFieldDiffers(ta, tb, eps, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr)) {
        ++tilesDifferent;
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x + 1);
        maxY = std::max(maxY, y + 1);
      }
    }
  }

  out.tilesDifferent = tilesDifferent;
  out.anyDifferent = (tilesDifferent > 0);

  if (!out.anyDifferent) {
    out.minX = out.minY = out.maxX = out.maxY = 0;
  } else {
    out.minX = minX;
    out.minY = minY;
    out.maxX = maxX;
    out.maxY = maxY;
  }

  return out;
}

PpmImage RenderWorldDiffColor(const World& a, const World& b, float heightEps)
{
  const int w = std::min(a.width(), b.width());
  const int h = std::min(a.height(), b.height());

  PpmImage img{};
  img.width = std::max(0, w);
  img.height = std::max(0, h);

  if (img.width <= 0 || img.height <= 0) return img;

  img.rgb.assign(static_cast<std::size_t>(img.width) * img.height * 3u, 0);

  const float eps = std::max(0.0f, heightEps);

  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const Tile& ta = a.at(x, y);
      const Tile& tb = b.at(x, y);

      bool terrainDiff = false;
      bool overlayDiff = false;
      bool heightDiff = false;
      bool variationDiff = false;
      bool levelDiff = false;
      bool occupantsDiff = false;
      bool districtDiff = false;

      if (!TileFieldDiffers(ta, tb, eps, &terrainDiff, &overlayDiff, &heightDiff,
                            &variationDiff, &levelDiff, &occupantsDiff, &districtDiff)) {
        continue; // leave black
      }

      int r = 0;
      int g = 0;
      int bl = 0;

      if (terrainDiff) r = 255;
      if (overlayDiff) g = 255;
      if (heightDiff) bl = 255;

      // Secondary fields: medium/low intensity so primary signals remain readable.
      if (variationDiff) { r = std::max(r, 160); g = std::max(g, 160); } // yellowish
      if (levelDiff) { g = std::max(g, 160); bl = std::max(bl, 160); }   // cyan-ish
      if (occupantsDiff) { r = std::max(r, 160); bl = std::max(bl, 160); } // magenta-ish
      if (districtDiff) { r = std::max(r, 96); g = std::max(g, 96); bl = std::max(bl, 96); } // gray/white

      PutRgb(img, x, y, static_cast<std::uint8_t>(std::clamp(r, 0, 255)),
             static_cast<std::uint8_t>(std::clamp(g, 0, 255)),
             static_cast<std::uint8_t>(std::clamp(bl, 0, 255)));
    }
  }

  return img;
}

PpmImage RenderWorldDiffCount(const World& a, const World& b, float heightEps)
{
  const int w = std::min(a.width(), b.width());
  const int h = std::min(a.height(), b.height());

  PpmImage img{};
  img.width = std::max(0, w);
  img.height = std::max(0, h);

  if (img.width <= 0 || img.height <= 0) return img;

  img.rgb.assign(static_cast<std::size_t>(img.width) * img.height * 3u, 0);

  const float eps = std::max(0.0f, heightEps);
  constexpr int kFields = 7; // terrain, overlay, height, variation, level, occupants, district

  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const Tile& ta = a.at(x, y);
      const Tile& tb = b.at(x, y);

      bool terrainDiff = false;
      bool overlayDiff = false;
      bool heightDiff = false;
      bool variationDiff = false;
      bool levelDiff = false;
      bool occupantsDiff = false;
      bool districtDiff = false;

      if (!TileFieldDiffers(ta, tb, eps, &terrainDiff, &overlayDiff, &heightDiff,
                            &variationDiff, &levelDiff, &occupantsDiff, &districtDiff)) {
        continue; // leave black
      }

      const int count = (terrainDiff ? 1 : 0) + (overlayDiff ? 1 : 0) + (heightDiff ? 1 : 0) +
                        (variationDiff ? 1 : 0) + (levelDiff ? 1 : 0) + (occupantsDiff ? 1 : 0) +
                        (districtDiff ? 1 : 0);

      const float t = static_cast<float>(count) / static_cast<float>(kFields);
      const int v = static_cast<int>(std::lround(255.0f * std::clamp(t, 0.0f, 1.0f)));
      const std::uint8_t u = static_cast<std::uint8_t>(std::clamp(v, 0, 255));
      PutRgb(img, x, y, u, u, u);
    }
  }

  return img;
}

} // namespace isocity
