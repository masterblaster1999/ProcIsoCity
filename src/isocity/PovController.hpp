#pragma once

#include "isocity/Types.hpp"

#include "isocity/RaylibShim.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace isocity {

class World;
struct ElevationSettings;
struct TourPlan;

// A lightweight, procedural "POV" (point-of-view) camera rig.
//
// The game is primarily isometric/2D, but a moving camera with good framing,
// smooth damping, subtle head-bob, and stop markers produces an appealing
// "ride-along" effect for routes and tours.
//
// This controller is intentionally app-layer (raylib Camera2D) and does NOT
// modify simulation state.

struct PovMarker {
  // Index into the flattened path tiles.
  int pathIndex = 0;

  // UI label to show when this marker is reached.
  std::string label;

  // Optional hold/pause time when the marker is reached.
  float holdSec = 0.0f;
};

struct PovConfig {
  // Path traversal speed.
  float speedTilesPerSec = 12.0f;

  // Default hold/pause time for TourPlanner stops.
  float stopHoldSec = 1.25f;

  // Camera smoothing (higher snaps harder).
  float followStiffness = 9.0f;

  // How far ahead to lead the camera.
  float lookAheadTiles = 5.0f;

  // Smooth the ride by sampling a Catmull-Rom spline through tile centers.
  bool useSpline = true;

  // Automatically slow down on sharper turns (helps readability).
  //  - turnSlowdown: 0=off, 1=strong
  //  - minSpeedFactor: clamp so we never fully stop
  //  - slowdownAngleDeg: angle at which slowdown saturates
  float turnSlowdown = 0.55f;
  float minSpeedFactor = 0.35f;
  float slowdownAngleDeg = 80.0f;

  // Framing/zoom.
  float zoomMove = 1.55f;
  float zoomHold = 1.85f;
  bool frameLower = true;
  float frameYOffsetFrac = 0.62f; // 0..1, where the focus point sits on screen.

  // Procedural motion.
  float bobAmplitudePx = 2.6f;
  float bobFrequencyHz = 1.8f;
  float swayAmplitudeDeg = 1.1f;
  float swayFrequencyHz = 0.55f;
  float turnLeanDeg = 2.6f;

  // Loop at the end of the path (otherwise auto-stops).
  bool loop = true;
};

struct PovTilePose {
  // Continuous tile position in "tile units" (center is +0.5,+0.5).
  Vector2 posTiles{0, 0};
  // Normalized direction in tile space.
  Vector2 dirTiles{1, 0};
  // Progress 0..1.
  float progress01 = 0.0f;
};

class PovController {
public:
  void clear();
  bool isActive() const { return m_active; }

  PovConfig& config() { return m_cfg; }
  const PovConfig& config() const { return m_cfg; }

  // Optional: customize the HUD title (e.g. \"Route\", \"Tour\", \"Roam\").
  void setTitle(std::string title) { m_title = std::move(title); }

  // Start a POV ride along an explicit path of road tiles.
  //
  // `markers` may be empty.
  bool startFromPath(const World& world, const std::vector<Point>& pathTiles,
                     const std::vector<PovMarker>& markers, float tileW, float tileH,
                     const ElevationSettings& elev, Camera2D& ioCamera,
                     int screenW, int screenH, std::uint32_t seed);

  // Start a POV ride along a TourPlan.
  bool startFromTour(const World& world, const TourPlan& tour, float tileW, float tileH,
                     const ElevationSettings& elev, Camera2D& ioCamera,
                     int screenW, int screenH, std::uint32_t seed);

  // Stop and restore the camera to its pre-POV state.
  void stop(Camera2D& ioCamera);

  // Advance time and update the camera.
  // Returns true if still active.
  bool update(float dt, const World& world, float tileW, float tileH, const ElevationSettings& elev,
              Camera2D& ioCamera, int screenW, int screenH);

  // Current pose in tile-space (useful for the software 3D preview).
  bool getTilePose(PovTilePose& outPose) const;

  // Convenience overload.
  bool getTilePose(Vector2& outPosTiles, Vector2& outDirTiles) const;

  // A compact status line for an on-screen HUD.
  std::string statusText() const;

  // Label of the most recently reached marker (or empty).
  std::string currentMarkerLabel() const;

private:
  PovConfig m_cfg{};
  bool m_active = false;

  // Saved camera state (restored on stop).
  Camera2D m_savedCam{};

  // Path.
  std::vector<Point> m_pathTiles;
  std::vector<Vector2> m_pathWorld; // tile centers in world coordinates
  std::vector<PovMarker> m_markers;

  // Runtime.
  float m_u = 0.0f;         // progress along path in tile segments
  float m_timeSec = 0.0f;   // local time
  float m_holdSec = 0.0f;   // remaining pause time at marker
  int m_nextMarker = 0;
  std::uint32_t m_seed = 0;

  // Smoothed camera state.
  Vector2 m_smoothTarget{0, 0};
  Vector2 m_prevDir{1, 0};
  float m_prevLean = 0.0f;

  // For UI/debug.
  std::string m_title;

  void rebuildWorldPath(const World& world, float tileW, float tileH, const ElevationSettings& elev);
  Vector2 sampleWorld(float u) const;
  Vector2 sampleDir(float u, float eps) const;
};

} // namespace isocity
