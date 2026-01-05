#pragma once

#include "isocity/Export.hpp"

#include <string>

namespace isocity {

// -----------------------------------------------------------------------------------------------
// Atlas filtering utilities (padding extrusion + mipmaps)
// -----------------------------------------------------------------------------------------------
//
// These utilities are intentionally dependency-free and deterministic. They are designed
// for sprite atlas pipelines where:
//  - padding/extrusion reduces texture-bleeding when using linear sampling + mipmapping
//  - mipmap generation allows exporting a full LOD chain for engines/tools that don't
//    build mipmaps at import time (or want deterministic offline results)

struct GfxMipmapChainConfig {
  // If > 0, generate at most this many mip levels *after* mip0.
  // If 0, generate until the image reaches 1x1.
  int levels = 0;

  // Stop generating once both dimensions are <= minSize.
  // (minSize=1 produces a full chain down to 1x1.)
  int minSize = 1;

  // If true, downsampling uses premultiplied-alpha averaging (recommended for sprites).
  bool premultiplyAlpha = true;
};

// A rectangle in mip0 atlas coordinates (used for per-sprite atlas operations).
struct GfxSpriteRect {
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
};

// Compute alpha coverage of a rectangle at a given threshold (0..1).
// Coverage is the fraction of pixels in the rect whose alpha >= threshold.
float AlphaCoverage(const RgbaImage& img, int x, int y, int w, int h, float threshold);

// Compute per-sprite alpha coverage targets from mip0.
bool ComputeAlphaCoverageTargets(const RgbaImage& mip0, const std::vector<GfxSpriteRect>& sprites,
                                 float threshold, std::vector<float>& outTargets, std::string& outError);

// Adjust ioMip's alpha per sprite so each sprite's alpha coverage at `threshold` matches the provided targets.
// sprites are specified in mip0 coordinates; mipLevel is the index of ioMip (0 for mip0, 1 for mip1, etc).
// If outScales is provided, it will be filled with the per-sprite alpha scales applied.
bool PreserveAlphaCoverageForMip(RgbaImage& ioMip, const std::vector<GfxSpriteRect>& sprites,
                                 const std::vector<float>& targets, int mipLevel,
                                 float threshold, int iterations,
                                 std::vector<float>* outScales, std::string& outError);

// Apply precomputed per-sprite alpha scales to ioMip (useful for applying the same mask adjustment
// to derived atlases like emissive/height/normal).
bool ApplyAlphaScalesForMip(RgbaImage& ioMip, const std::vector<GfxSpriteRect>& sprites,
                            const std::vector<float>& scales, int mipLevel,
                            std::string& outError);


// Downsample an RGBA image by 2x using a simple 2x2 box filter.
// Handles odd dimensions by clamping samples to the image edges.
bool DownsampleRgba2x(const RgbaImage& src, bool premultiplyAlpha, RgbaImage& outDst, std::string& outError);

// Downsample a normal map (RGB-encoded unit vectors, alpha as mask) by 2x.
// The downsample:
//  - averages decoded normals weighted by alpha
//  - renormalizes the vector
//  - encodes back to RGB
bool DownsampleNormalMap2x(const RgbaImage& src, RgbaImage& outDst, std::string& outError);

// Generate a full mip chain for a generic RGBA atlas.
// The output vector includes mip0 as a copy of src.
bool GenerateMipChainRgba(const RgbaImage& src, const GfxMipmapChainConfig& cfg,
                          std::vector<RgbaImage>& outMips, std::string& outError);

// Generate a full mip chain for a normal map atlas.
// The output vector includes mip0 as a copy of src.
bool GenerateMipChainNormalMap(const RgbaImage& src, const GfxMipmapChainConfig& cfg,
                               std::vector<RgbaImage>& outMips, std::string& outError);

// Extrude a sprite's border pixels outward into surrounding transparent padding.
//
// This is intended to be applied to atlases *after packing* and before generating mipmaps.
// It only writes to destination pixels whose alpha is 0, so it is safe to call on the full
// atlas as long as sprites don't overlap.
bool ExtrudeSpritePadding(RgbaImage& ioAtlas, int x, int y, int w, int h, int extrudePx, std::string& outError);

} // namespace isocity
