#include "isocity/WorldDiff.hpp"

#include <algorithm>
#include <cmath>

namespace isocity {

WorldDiffStats DiffWorldTiles(const World& a, const World& b, float heightEps)
{
  WorldDiffStats d{};
  d.widthA = a.width();
  d.heightA = a.height();
  d.widthB = b.width();
  d.heightB = b.height();
  d.sizeMismatch = (d.widthA != d.widthB) || (d.heightA != d.heightB);

  const int w = std::min(d.widthA, d.widthB);
  const int h = std::min(d.heightA, d.heightB);
  if (w <= 0 || h <= 0) {
    d.tilesCompared = 0;
    return d;
  }

  d.tilesCompared = w * h;

  const float eps = std::max(0.0f, heightEps);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& ta = a.at(x, y);
      const Tile& tb = b.at(x, y);

      const bool terrainDiff = ta.terrain != tb.terrain;
      const bool overlayDiff = ta.overlay != tb.overlay;
      const bool heightDiff = std::fabs(ta.height - tb.height) > eps;
      const bool variationDiff = ta.variation != tb.variation;
      const bool levelDiff = ta.level != tb.level;
      const bool occupantsDiff = ta.occupants != tb.occupants;
      const bool districtDiff = ta.district != tb.district;

      if (terrainDiff) ++d.terrainDifferent;
      if (overlayDiff) ++d.overlayDifferent;
      if (heightDiff) ++d.heightDifferent;
      if (variationDiff) ++d.variationDifferent;
      if (levelDiff) ++d.levelDifferent;
      if (occupantsDiff) ++d.occupantsDifferent;
      if (districtDiff) ++d.districtDifferent;

      if (terrainDiff || overlayDiff || heightDiff || variationDiff || levelDiff || occupantsDiff || districtDiff) {
        ++d.tilesDifferent;
      }
    }
  }

  return d;
}

} // namespace isocity
