#include "isocity/PovController.hpp"

#include "isocity/Elevation.hpp"
#include "isocity/Iso.hpp"
#include "isocity/Noise.hpp"
#include "isocity/TourPlanner.hpp"
#include "isocity/World.hpp"

#include "raymath.h"

#include <algorithm>
#include <cstdio>
#include <cmath>

namespace isocity {
namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float Saturate(float x) { return std::max(0.0f, std::min(1.0f, x)); }

inline float SmoothAlpha(float dt, float stiffness) {
  // Convert stiffness to an exponential smoothing alpha in [0,1].
  return 1.0f - std::exp(-std::max(0.0f, dt) * std::max(0.0f, stiffness));
}

inline Vector2 V2Lerp(Vector2 a, Vector2 b, float t) {
  return Vector2{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}


inline Vector2 V2CatmullRom(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t) {
  const float t2 = t * t;
  const float t3 = t2 * t;
  return Vector2{
      0.5f * ((2.0f * p1.x) + (-p0.x + p2.x) * t + (2.0f * p0.x - 5.0f * p1.x + 4.0f * p2.x - p3.x) * t2 +
              (-p0.x + 3.0f * p1.x - 3.0f * p2.x + p3.x) * t3),
      0.5f * ((2.0f * p1.y) + (-p0.y + p2.y) * t + (2.0f * p0.y - 5.0f * p1.y + 4.0f * p2.y - p3.y) * t2 +
              (-p0.y + 3.0f * p1.y - 3.0f * p2.y + p3.y) * t3)};
}

inline Vector2 V2SafeNorm(Vector2 v, Vector2 fallback) {
  const float len = Vector2Length(v);
  if (len < 1e-5f) {
    return fallback;
  }
  return Vector2Scale(v, 1.0f / len);
}

inline float SignedAngleRad(Vector2 a, Vector2 b) {
  // Returns the signed angle from a -> b.
  const float cross = a.x * b.y - a.y * b.x;
  const float dot = a.x * b.x + a.y * b.y;
  return std::atan2(cross, dot);
}

} // namespace

void PovController::clear() {
  m_active = false;
  m_pathTiles.clear();
  m_pathWorld.clear();
  m_markers.clear();
  m_u = 0.0f;
  m_timeSec = 0.0f;
  m_holdSec = 0.0f;
  m_nextMarker = 0;
  m_seed = 0;
  m_smoothTarget = Vector2{0, 0};
  m_prevDir = Vector2{1, 0};
  m_prevLean = 0.0f;
  m_title.clear();
}

bool PovController::startFromPath(const World& world, const std::vector<Point>& pathTiles,
                                  const std::vector<PovMarker>& markers, float tileW, float tileH,
                                  const ElevationSettings& elev, Camera2D& ioCamera, int screenW,
                                  int screenH, std::uint32_t seed) {
  if (pathTiles.size() < 2) {
    return false;
  }

  // Save camera so we can restore it.
  m_savedCam = ioCamera;

  m_active = true;
  m_seed = seed;
  m_u = 0.0f;
  m_timeSec = 0.0f;
  m_holdSec = 0.0f;
  m_nextMarker = 0;

  m_pathTiles = pathTiles;
  m_markers = markers;
  std::sort(m_markers.begin(), m_markers.end(),
            [](const PovMarker& a, const PovMarker& b) { return a.pathIndex < b.pathIndex; });

  rebuildWorldPath(world, tileW, tileH, elev);
  if (m_pathWorld.size() < 2) {
    stop(ioCamera);
    return false;
  }

  // Initialize smoothing to the initial focus point.
  const Vector2 pos0 = sampleWorld(0.0f);
  const Vector2 ahead0 = sampleWorld(m_cfg.lookAheadTiles);
  m_prevDir = V2SafeNorm(Vector2Subtract(ahead0, pos0), Vector2{1, 0});
  m_smoothTarget = ahead0;
  m_prevLean = 0.0f;

  // Nudge framing immediately so the first frame looks correct.
  update(0.0f, world, tileW, tileH, elev, ioCamera, screenW, screenH);
  return true;
}

bool PovController::startFromTour(const World& world, const TourPlan& tour, float tileW, float tileH,
                                  const ElevationSettings& elev, Camera2D& ioCamera, int screenW,
                                  int screenH, std::uint32_t seed) {
  std::vector<Point> path;
  std::vector<PovMarker> markers;

  // Start tile.
  path.push_back(tour.start.roadTile);
  if (!tour.startQuery.empty()) {
    m_title = tour.title + " (" + tour.startQuery + ")";
  } else {
    m_title = tour.title;
  }

  // Flatten the stop routes.
  for (std::size_t i = 0; i < tour.stops.size(); ++i) {
    const auto& stop = tour.stops[i];
    if (!stop.routeFromPrev.ok || stop.routeFromPrev.pathTiles.empty()) {
      continue;
    }

    const auto& seg = stop.routeFromPrev.pathTiles;
    const bool sameStart = (!path.empty() && path.back().x == seg.front().x && path.back().y == seg.front().y);
    const std::size_t startIdx = sameStart ? 1 : 0;
    for (std::size_t j = startIdx; j < seg.size(); ++j) {
      path.push_back(seg[j]);
    }

    PovMarker m;
    m.pathIndex = static_cast<int>(path.size() - 1);
    m.label = stop.poi.name;
    m.holdSec = m_cfg.stopHoldSec;
    markers.push_back(std::move(m));
  }

  if (path.size() < 2) {
    return false;
  }

  // Use the per-plan seed if set; otherwise fall back to the provided seed.
  const std::uint32_t useSeed = (tour.seed != 0) ? static_cast<std::uint32_t>(tour.seed) : seed;
  return startFromPath(world, path, markers, tileW, tileH, elev, ioCamera, screenW, screenH, useSeed);
}

void PovController::stop(Camera2D& ioCamera) {
  if (!m_active) {
    return;
  }

  ioCamera = m_savedCam;
  clear();
}

bool PovController::update(float dt, const World& world, float tileW, float tileH,
                           const ElevationSettings& elev, Camera2D& ioCamera, int screenW,
                           int screenH) {
  if (!m_active) {
    return false;
  }
  if (m_pathWorld.size() < 2) {
    stop(ioCamera);
    return false;
  }

  m_timeSec += dt;

  // Marker pause logic.
  if (m_holdSec > 0.0f) {
    m_holdSec = std::max(0.0f, m_holdSec - dt);
  } else {
    float speed = std::max(0.0f, m_cfg.speedTilesPerSec);

    // Curvature-aware slowdown keeps tight turns readable without requiring explicit markers.
    if (speed > 0.0f && m_cfg.turnSlowdown > 0.0f && m_pathWorld.size() >= 3) {
      const Vector2 d0 = sampleDir(m_u, 0.85f);
      const Vector2 d1 = sampleDir(m_u + 1.0f, 0.85f);
      const float angDeg = std::abs(SignedAngleRad(d0, d1)) * (180.0f / kPi);
      const float s = Saturate(angDeg / std::max(1.0f, m_cfg.slowdownAngleDeg));
      const float factor = std::max(m_cfg.minSpeedFactor, 1.0f - m_cfg.turnSlowdown * s);
      speed *= factor;
    }

    m_u += speed * dt;
  }

  const float maxU = static_cast<float>(m_pathWorld.size() - 1);
  if (maxU <= 0.0f) {
    stop(ioCamera);
    return false;
  }

  if (m_u >= maxU) {
    if (m_cfg.loop) {
      m_u = std::fmod(m_u, maxU);
      m_nextMarker = 0;
    } else {
      stop(ioCamera);
      return false;
    }
  }

  // Trigger marker holds.
  if (m_holdSec <= 0.0f && m_nextMarker >= 0 && m_nextMarker < static_cast<int>(m_markers.size())) {
    const PovMarker& next = m_markers[m_nextMarker];
    if (m_u >= static_cast<float>(next.pathIndex)) {
      m_holdSec = std::max(0.0f, next.holdSec);
      m_nextMarker++;
    }
  }

  // Sample focus and forward direction.
  const Vector2 pos = sampleWorld(m_u);
  const Vector2 look = sampleWorld(m_u + m_cfg.lookAheadTiles);

  // Keep a copy for turn-lean; m_prevDir is the prior-frame direction.
  const Vector2 prevDir = m_prevDir;
  const Vector2 dir = V2SafeNorm(Vector2Subtract(look, pos), prevDir);
  m_prevDir = dir;

  // Smooth the camera target (focus point).
  const float a = SmoothAlpha(dt, m_cfg.followStiffness);
  m_smoothTarget = V2Lerp(m_smoothTarget, look, a);
  ioCamera.target = m_smoothTarget;

  // Zoom blend.
  const float targetZoom = (m_holdSec > 0.0f) ? m_cfg.zoomHold : m_cfg.zoomMove;
  const float az = SmoothAlpha(dt, 6.0f);
  ioCamera.zoom = ioCamera.zoom + (targetZoom - ioCamera.zoom) * az;

  // Procedural bob (screen-space) + subtle noise.
  const float t = m_timeSec;
  const float bobPhase = t * 2.0f * kPi * std::max(0.0f, m_cfg.bobFrequencyHz);
  float bobY = std::sin(bobPhase) * m_cfg.bobAmplitudePx;
  float bobX = std::cos(bobPhase * 0.53f) * (m_cfg.bobAmplitudePx * 0.55f);

  const float n1 = (ValueNoise2D(t * 0.70f, 13.37f, m_seed) - 0.5f) * 2.0f;
  const float n2 = (ValueNoise2D(t * 0.90f, 42.00f, m_seed + 1u) - 0.5f) * 2.0f;
  bobX += n1 * (m_cfg.bobAmplitudePx * 0.28f);
  bobY += n2 * (m_cfg.bobAmplitudePx * 0.28f);

  const float baseOffX = static_cast<float>(screenW) * 0.5f;
  const float baseOffY = static_cast<float>(screenH) * (m_cfg.frameLower ? m_cfg.frameYOffsetFrac : 0.5f);
  ioCamera.offset = Vector2{baseOffX + bobX, baseOffY + bobY};

  // Sway + turn lean (rotation in degrees).
  const float swayPhase = t * 2.0f * kPi * std::max(0.0f, m_cfg.swayFrequencyHz);
  const float sway = std::sin(swayPhase) * m_cfg.swayAmplitudeDeg;

  const float dAng = SignedAngleRad(prevDir, dir);
  const float leanTarget = std::max(-m_cfg.turnLeanDeg, std::min(m_cfg.turnLeanDeg, dAng * (180.0f / kPi)));
  const float al = SmoothAlpha(dt, 8.0f);
  m_prevLean = m_prevLean + (leanTarget - m_prevLean) * al;

  ioCamera.rotation = m_savedCam.rotation + sway + m_prevLean;

  // If the world changed significantly (terrain sculpting), callers can re-start.
  (void)world;
  (void)tileW;
  (void)tileH;
  (void)elev;
  return true;
}

bool PovController::getTilePose(PovTilePose& outPose) const {
  if (!m_active || m_pathTiles.size() < 2) {
    return false;
  }

  const float maxU = static_cast<float>(m_pathTiles.size() - 1);
  const float uClamped = std::max(0.0f, std::min(maxU, m_u));
  const int i = static_cast<int>(std::floor(uClamped));
  const float f = uClamped - static_cast<float>(i);
  const int i0 = std::max(0, std::min(static_cast<int>(m_pathTiles.size() - 2), i));
  const int i1 = i0 + 1;

  const Point a = m_pathTiles[i0];
  const Point b = m_pathTiles[i1];
  const Vector2 posTiles{(static_cast<float>(a.x) + 0.5f) * (1.0f - f) + (static_cast<float>(b.x) + 0.5f) * f,
                         (static_cast<float>(a.y) + 0.5f) * (1.0f - f) + (static_cast<float>(b.y) + 0.5f) * f};

  const Vector2 dirTiles = V2SafeNorm(Vector2{static_cast<float>(b.x - a.x), static_cast<float>(b.y - a.y)},
                                      Vector2{1, 0});

  outPose.posTiles = posTiles;
  outPose.dirTiles = dirTiles;
  outPose.progress01 = (maxU > 0.0f) ? (uClamped / maxU) : 0.0f;
  return true;
}

bool PovController::getTilePose(Vector2& outPosTiles, Vector2& outDirTiles) const {
  PovTilePose pose;
  if (!getTilePose(pose)) {
    return false;
  }
  outPosTiles = pose.posTiles;
  outDirTiles = pose.dirTiles;
  return true;
}

std::string PovController::statusText() const {
  if (!m_active) {
    return "POV: off";
  }

  const float maxU = static_cast<float>(std::max<std::size_t>(1, m_pathTiles.size() - 1));
  const float p = Saturate((maxU > 0.0f) ? (m_u / maxU) : 0.0f);
  char buf[256];
  std::snprintf(buf, sizeof(buf), "POV: %s  %d/%d  %d%%", (m_holdSec > 0.0f) ? "hold" : "ride",
                static_cast<int>(std::floor(m_u)), static_cast<int>(maxU), static_cast<int>(p * 100.0f));
  if (!m_title.empty()) {
    return std::string(buf) + "  " + m_title;
  }
  return std::string(buf);
}

std::string PovController::currentMarkerLabel() const {
  const int idx = std::max(0, std::min(m_nextMarker - 1, static_cast<int>(m_markers.size()) - 1));
  if (m_active && idx >= 0 && idx < static_cast<int>(m_markers.size())) {
    return m_markers[idx].label;
  }
  return {};
}

void PovController::rebuildWorldPath(const World& world, float tileW, float tileH,
                                    const ElevationSettings& elev) {
  m_pathWorld.clear();
  m_pathWorld.reserve(m_pathTiles.size());
  for (const Point p : m_pathTiles) {
    m_pathWorld.push_back(TileToWorldCenterElevated(world, p.x, p.y, tileW, tileH, elev));
  }
}

Vector2 PovController::sampleWorld(float u) const {
  if (m_pathWorld.empty()) {
    return Vector2{0, 0};
  }

  const int n = static_cast<int>(m_pathWorld.size());
  const float maxU = static_cast<float>(n - 1);
  const float uClamped = std::max(0.0f, std::min(maxU, u));
  const int i = static_cast<int>(std::floor(uClamped));
  const float f = uClamped - static_cast<float>(i);

  // Linear fallback for very short paths or when smoothing is disabled.
  if (!m_cfg.useSpline || n < 4) {
    const int i0 = std::max(0, std::min(n - 2, i));
    const int i1 = i0 + 1;
    return V2Lerp(m_pathWorld[i0], m_pathWorld[i1], f);
  }

  // Catmull-Rom spline through tile centers.
  const int i1 = std::max(0, std::min(n - 2, i)); // segment start
  const int i2 = i1 + 1;
  const int i0 = std::max(0, i1 - 1);
  const int i3 = std::min(n - 1, i2 + 1);
  return V2CatmullRom(m_pathWorld[i0], m_pathWorld[i1], m_pathWorld[i2], m_pathWorld[i3], f);
}

Vector2 PovController::sampleDir(float u, float eps) const {
  const Vector2 a = sampleWorld(u);
  const Vector2 b = sampleWorld(u + eps);
  return V2SafeNorm(Vector2Subtract(b, a), m_prevDir);
}

} // namespace isocity
