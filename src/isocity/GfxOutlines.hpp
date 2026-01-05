#pragma once

#include "isocity/Export.hpp"     // RgbaImage
#include "isocity/GfxTileset.hpp" // GfxAtlasEntry, GfxTilesetResult
#include "isocity/Vectorize.hpp"  // VectorMultiPolygon, IPoint

#include <string>
#include <vector>

namespace isocity {

// Alpha-mask vectorization configuration.
//
// Used by proc_isocity_tileset to export per-sprite outlines/hulls for external
// renderers, picking, physics, and debug tooling.
struct GfxOutlineConfig {
  // Alpha threshold in [0,1] used to classify pixels as inside/outside.
  float alphaThreshold = 0.5f;

  // If true, compute a simple convex hull around all outline vertices.
  bool computeConvexHull = true;

  // If false, strip holes from the output geometry.
  bool includeHoles = true;
};

// Outline geometry for one sprite.
//
// Coordinates are in *sprite-local pixel-corner space* of the logical canvas
// (srcW x srcH). This means:
//  - vertices are integer coordinates on the grid [0,srcW] x [0,srcH]
//  - the outline follows pixel edges
//  - when trimming was enabled, trimX/trimY are already applied
struct GfxSpriteOutline {
  std::string name;

  // Atlas rectangle (cropped sprite rect).
  int atlasX = 0;
  int atlasY = 0;
  int w = 0;
  int h = 0;

  // Logical untrimmed canvas size and crop offset.
  int srcW = 0;
  int srcH = 0;
  int trimX = 0;
  int trimY = 0;

  // Vectorized outline polygons (may include multiple disconnected components).
  VectorMultiPolygon geom;

  // Optional convex hull (closed ring). Empty if computeConvexHull=false or
  // the sprite had no opaque pixels.
  std::vector<IPoint> hull;
};

// Compute outline polygons (and optional convex hull) for a single sprite entry.
bool ComputeGfxSpriteOutline(const RgbaImage& atlas, const GfxAtlasEntry& entry,
                            const GfxOutlineConfig& cfg,
                            GfxSpriteOutline& out,
                            std::string& outError);

// Compute outlines for all sprites in a generated tileset.
bool ComputeGfxTilesetOutlines(const GfxTilesetResult& ts,
                              const GfxOutlineConfig& cfg,
                              std::vector<GfxSpriteOutline>& out,
                              std::string& outError);

// Write per-sprite outline geometry to a standalone JSON file.
//
// This is intentionally separate from the main meta JSON because outline geometry
// can be large, and not all consumers need it.
bool WriteGfxTilesetOutlinesJson(const std::string& path,
                                const GfxTilesetResult& ts,
                                const GfxOutlineConfig& cfg,
                                const std::vector<GfxSpriteOutline>& outlines,
                                std::string& outError);

// Write an SVG overlay preview of outlines on top of the atlas image.
//
// atlasHref is the path used in the SVG <image href="..."> element (typically a
// relative path from the SVG file to the atlas PNG).
bool WriteGfxTilesetOutlinesSvg(const std::string& path,
                               const std::string& atlasHref,
                               const GfxTilesetResult& ts,
                               const std::vector<GfxSpriteOutline>& outlines,
                               int svgScale,
                               std::string& outError);

} // namespace isocity
