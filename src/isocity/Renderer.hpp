#pragma once

#include "isocity/Iso.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

// raylib on Windows pulls in <windows.h>, which by default defines the `min` and
// `max` macros. Those macros break std::min/std::max and can lead to very
// confusing compile errors (especially around std::max/std::min usage inside
// our game code).
//
// Ensure NOMINMAX is set before any Windows headers are included.
#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
#endif

#include "raylib.h"

// Some environments still define min/max even with NOMINMAX; undef defensively.
#if defined(_WIN32)
  #ifdef min
    #undef min
  #endif
  #ifdef max
    #undef max
  #endif
#endif

namespace isocity {

class Renderer {
public:
  enum class HeatmapRamp : std::uint8_t {
    // 0 => bad (red), 1 => good (green)
    Good = 0,
    // 0 => good (green), 1 => bad (red)
    Bad = 1,
  };

  Renderer(int tileW, int tileH, std::uint64_t seed);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  void rebuildTextures(std::uint64_t seed);

  void setElevationSettings(const ElevationSettings& s)
  {
    m_elev = s;
    // Elevation changes world-space placement; cached base layers must be redrawn.
    markBaseCacheDirtyAll();
  }
  const ElevationSettings& elevationSettings() const { return m_elev; }

  // Performance: optional render cache for the static base world (terrain + cliffs + base overlays).
  void setBaseCacheEnabled(bool enabled) { m_useBandCache = enabled; }
  bool baseCacheEnabled() const { return m_useBandCache; }

  // Mark cached base world dirty (re-rendered lazily when drawn).
  void markBaseCacheDirtyAll();
  void markBaseCacheDirtyForTiles(const std::vector<Point>& tiles, int mapW, int mapH);

  // Screen-space minimap layout helper (used by the game for hit-testing).
  struct MinimapLayout {
    Rectangle rect{};      // destination rectangle in screen space
    float pixelsPerTile = 1.0f; // how many screen pixels represent one world tile
  };

  // Compute where the minimap will be drawn on screen for the given world.
  // This stays stable across frames (depends only on screen size + world size).
  MinimapLayout minimapLayout(const World& world, int screenW, int screenH) const;

  // Mark minimap texture dirty (recomputed lazily when drawn).
  void markMinimapDirty() { m_minimapDirty = true; }

  // Export the current minimap (one pixel per tile) to an image file.
  // The image is optionally downscaled so its max dimension is <= maxSize.
  // Returns true on success.
  bool exportMinimapThumbnail(const World& world, const char* fileName, int maxSize = 256);

  // Export a full-city overview render to an image file.
  //
  // This renders the entire world to an off-screen render texture sized to fit the map.
  // If the resulting image would be large, it is scaled down so its max dimension is <= maxSize.
  //
  // Returns true on success.
  bool exportWorldOverview(const World& world, const char* fileName, int maxSize = 4096);

  void drawWorld(const World& world, const Camera2D& camera, int screenW, int screenH, float timeSec,
                 std::optional<Point> hovered,
                 bool drawGrid, int brushRadius, std::optional<Point> selected, const std::vector<Point>* highlightPath,
                 const std::vector<std::uint8_t>* roadToEdgeMask = nullptr,
                 const std::vector<std::uint16_t>* roadTraffic = nullptr, int trafficMax = 0,
                 const std::vector<std::uint16_t>* roadGoodsTraffic = nullptr, int goodsMax = 0,
                 const std::vector<std::uint8_t>* commercialGoodsFill = nullptr,
                 const std::vector<float>* heatmap = nullptr,
                 HeatmapRamp heatmapRamp = HeatmapRamp::Good,
                 bool showDistrictOverlay = false,
                 int highlightDistrict = -1,
                 bool showDistrictBorders = false);

  void drawHUD(const World& world, const Camera2D& camera, Tool tool, int roadBuildLevel,
               std::optional<Point> hovered, int screenW, int screenH, bool showHelp, int brushRadius,
               int undoCount, int redoCount, bool simPaused, float simSpeed, int saveSlot, bool showMinimap,
               const char* inspectInfo, const char* heatmapInfo);

private:
  int m_tileW = 64;
  int m_tileH = 32;

  ElevationSettings m_elev{};

  std::array<Texture2D, 3> m_terrainTex{};
  std::array<Texture2D, 6> m_overlayTex{};
  std::array<Texture2D, 16> m_roadTex{};   // auto-tiling variants (connection mask 0..15)
  std::array<Texture2D, 16> m_bridgeTex{}; // bridge variants (roads on water)

  // Minimap texture (one pixel per tile). This is purely a UI convenience.
  Texture2D m_minimapTex{};
  int m_minimapW = 0;
  int m_minimapH = 0;
  std::vector<Color> m_minimapPixels;
  bool m_minimapDirty = true;

  // Base world render cache (static layers baked into render textures).
  struct BandCache {
    RenderTexture2D rt{};
    int sum0 = 0; // inclusive
    int sum1 = 0; // inclusive
    Vector2 origin{}; // world-space top-left where this texture should be drawn
    bool dirty = true;
  };

  static constexpr int kBandSums = 8; // how many (x+y) diagonals are baked per band

  bool m_useBandCache = true;
  int m_bandMapW = 0;
  int m_bandMapH = 0;
  float m_bandMaxPixels = 0.0f;
  bool m_bandCacheDirtyAll = true;
  std::vector<BandCache> m_bands;

  void unloadTextures();

  void unloadBaseCache();
  void ensureBaseCache(const World& world);
  void rebuildBaseCacheBand(const World& world, BandCache& band);

  void unloadMinimap();
  void ensureMinimapUpToDate(const World& world);

  Texture2D& terrain(Terrain t);
  Texture2D& overlay(Overlay o);
  Texture2D& road(std::uint8_t mask);
  Texture2D& bridge(std::uint8_t mask);

  static Color BrightnessTint(float b);
};

} // namespace isocity
