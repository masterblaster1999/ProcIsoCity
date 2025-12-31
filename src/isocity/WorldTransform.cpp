#include "isocity/WorldTransform.hpp"

#include <algorithm>
#include <sstream>

namespace isocity {

namespace {

bool IsValidRotate(int r)
{
  return r == 0 || r == 90 || r == 180 || r == 270;
}

void RotatedDims(int srcW, int srcH, int rotateDeg, int& outW, int& outH)
{
  if (rotateDeg == 90 || rotateDeg == 270) {
    outW = srcH;
    outH = srcW;
  } else {
    outW = srcW;
    outH = srcH;
  }
}

bool MapRotatedToSource(int srcW, int srcH, int rotateDeg, int xRot, int yRot, int& outSrcX, int& outSrcY)
{
  // xRot/yRot are coordinates in the rotated space (before any crop).
  // Returns src coords.
  switch (rotateDeg) {
  case 0:
    outSrcX = xRot;
    outSrcY = yRot;
    return true;
  case 90:
    // dest(x,y) = src(y, H-1-x)
    outSrcX = yRot;
    outSrcY = srcH - 1 - xRot;
    return true;
  case 180:
    outSrcX = srcW - 1 - xRot;
    outSrcY = srcH - 1 - yRot;
    return true;
  case 270:
    // dest(x,y) = src(W-1-y, x)
    outSrcX = srcW - 1 - yRot;
    outSrcY = xRot;
    return true;
  default:
    return false;
  }
}

} // namespace

bool ValidateWorldTransform(const WorldTransformConfig& cfg, int srcW, int srcH, std::string& outError)
{
  outError.clear();

  if (srcW <= 0 || srcH <= 0) {
    outError = "Invalid source world dimensions";
    return false;
  }

  if (!IsValidRotate(cfg.rotateDeg)) {
    outError = "rotateDeg must be one of: 0, 90, 180, 270";
    return false;
  }

  int wRot = 0;
  int hRot = 0;
  RotatedDims(srcW, srcH, cfg.rotateDeg, wRot, hRot);

  if (cfg.hasCrop) {
    if (cfg.cropW <= 0 || cfg.cropH <= 0) {
      outError = "cropW/cropH must be > 0";
      return false;
    }
    if (cfg.cropX < 0 || cfg.cropY < 0) {
      outError = "cropX/cropY must be >= 0";
      return false;
    }
    if (cfg.cropX + cfg.cropW > wRot || cfg.cropY + cfg.cropH > hRot) {
      std::ostringstream oss;
      oss << "Crop rectangle out of bounds. Rotated world dims=" << wRot << "x" << hRot
          << " crop=" << cfg.cropX << "," << cfg.cropY << " " << cfg.cropW << "x" << cfg.cropH;
      outError = oss.str();
      return false;
    }
  }

  return true;
}

bool ComputeWorldTransformDims(const WorldTransformConfig& cfg, int srcW, int srcH, int& outW, int& outH,
                              std::string& outError)
{
  outError.clear();
  outW = 0;
  outH = 0;

  if (!ValidateWorldTransform(cfg, srcW, srcH, outError)) {
    return false;
  }

  int wRot = 0;
  int hRot = 0;
  RotatedDims(srcW, srcH, cfg.rotateDeg, wRot, hRot);

  if (cfg.hasCrop) {
    outW = cfg.cropW;
    outH = cfg.cropH;
  } else {
    outW = wRot;
    outH = hRot;
  }

  return true;
}

bool MapTransformedToSource(const WorldTransformConfig& cfg, int srcW, int srcH, int xOut, int yOut, int& outSrcX,
                            int& outSrcY, std::string& outError)
{
  outError.clear();

  int outW = 0;
  int outH = 0;
  if (!ComputeWorldTransformDims(cfg, srcW, srcH, outW, outH, outError)) {
    return false;
  }

  if (xOut < 0 || yOut < 0 || xOut >= outW || yOut >= outH) {
    outError = "Output coord out of bounds";
    return false;
  }

  int wRot = 0;
  int hRot = 0;
  RotatedDims(srcW, srcH, cfg.rotateDeg, wRot, hRot);

  // Start in output space (after crop).
  int xRot = xOut;
  int yRot = yOut;

  // Undo crop (crop is applied last in the pipeline).
  if (cfg.hasCrop) {
    xRot += cfg.cropX;
    yRot += cfg.cropY;
  }

  if (xRot < 0 || yRot < 0 || xRot >= wRot || yRot >= hRot) {
    outError = "Internal error: cropped coord out of bounds";
    return false;
  }

  // Undo mirrors (mirrors are applied after rotation).
  if (cfg.mirrorX) xRot = wRot - 1 - xRot;
  if (cfg.mirrorY) yRot = hRot - 1 - yRot;

  if (!MapRotatedToSource(srcW, srcH, cfg.rotateDeg, xRot, yRot, outSrcX, outSrcY)) {
    outError = "Invalid rotation";
    return false;
  }

  if (outSrcX < 0 || outSrcY < 0 || outSrcX >= srcW || outSrcY >= srcH) {
    outError = "Internal error: mapped source coord out of bounds";
    return false;
  }

  return true;
}

bool TransformWorld(const World& src, World& outWorld, const WorldTransformConfig& cfg, std::string& outError,
                    bool copyStats)
{
  outError.clear();

  const int srcW = src.width();
  const int srcH = src.height();

  int outW = 0;
  int outH = 0;
  if (!ComputeWorldTransformDims(cfg, srcW, srcH, outW, outH, outError)) {
    return false;
  }

  int wRot = 0;
  int hRot = 0;
  RotatedDims(srcW, srcH, cfg.rotateDeg, wRot, hRot);

  outWorld = World(outW, outH, src.seed());
  if (copyStats) {
    outWorld.stats() = src.stats();
  }

  // Fast per-tile mapping (avoid repeated validation).
  auto mapOutToSrc = [&](int xOutLocal, int yOutLocal, int& xs, int& ys) -> bool {
    int xRot = xOutLocal;
    int yRot = yOutLocal;

    if (cfg.hasCrop) {
      xRot += cfg.cropX;
      yRot += cfg.cropY;
    }

    if (xRot < 0 || yRot < 0 || xRot >= wRot || yRot >= hRot) {
      return false;
    }

    if (cfg.mirrorX) xRot = wRot - 1 - xRot;
    if (cfg.mirrorY) yRot = hRot - 1 - yRot;

    if (!MapRotatedToSource(srcW, srcH, cfg.rotateDeg, xRot, yRot, xs, ys)) {
      return false;
    }

    return xs >= 0 && ys >= 0 && xs < srcW && ys < srcH;
  };

  for (int y = 0; y < outH; ++y) {
    for (int x = 0; x < outW; ++x) {
      int xs = 0;
      int ys = 0;
      if (!mapOutToSrc(x, y, xs, ys)) {
        outError = "Internal error: transform mapping failed";
        return false;
      }

      outWorld.at(x, y) = src.at(xs, ys);
    }
  }

  // Road auto-tiling masks are directional; rotation/mirroring invalidates the stored low bits.
  // Recompute them so the world can be used immediately without a reload.
  outWorld.recomputeRoadMasks();

  return true;
}

} // namespace isocity
