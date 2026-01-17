#pragma once

#include "isocity/Iso.hpp"
#include "isocity/World.hpp"
#include "isocity/ZoneParcels.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>
#include <functional>

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

#include "isocity/RaylibShim.hpp"

#include "isocity/GpuGeom.hpp"
#include "isocity/OrganicMaterial.hpp"

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

struct WorldRect {
  float minX = 0.0f;
  float minY = 0.0f;
  float maxX = 0.0f;
  float maxY = 0.0f;
};

class Renderer {
public:
  enum class HeatmapRamp : std::uint8_t {
    // 0 => bad (red), 1 => good (green)
    Good = 0,
    // 0 => good (green), 1 => bad (red)
    Bad = 1,

    // 0 => shallow/low (light blue), 1 => deep/high (dark blue)
    Water = 2,
  };

  // Multi-layer rendering (useful for debugging, capture, and future compositing).
  //
  // These are purely render-time toggles; they do not change simulation state.
  enum class RenderLayer : std::uint8_t {
    Terrain = 0,
    Decals = 1,
    Structures = 2,
    Overlays = 3,
  };

  // Some compilers (notably older MSVC toolsets) can be picky about calling a
  // constexpr helper function in a constant-expression initializer inside the
  // same class definition. Keep the layer mask bits as plain shifts to ensure
  // they are always usable in constant contexts (static_assert, switch labels,
  // template params, etc.).
  static constexpr std::uint32_t kLayerTerrain =
      1u << static_cast<std::uint32_t>(RenderLayer::Terrain);
  static constexpr std::uint32_t kLayerDecals =
      1u << static_cast<std::uint32_t>(RenderLayer::Decals);
  static constexpr std::uint32_t kLayerStructures =
      1u << static_cast<std::uint32_t>(RenderLayer::Structures);
  static constexpr std::uint32_t kLayerOverlays =
      1u << static_cast<std::uint32_t>(RenderLayer::Overlays);
  static constexpr std::uint32_t kLayerAll = kLayerTerrain | kLayerDecals | kLayerStructures | kLayerOverlays;

  // Convert a RenderLayer enum into its bit in a layer mask.
  //
  // Keep this as a small constexpr utility so both the renderer and other
  // systems (visual prefs persistence, exports, debug UI) can reason about
  // the mask without duplicating the bit shift logic.
  static constexpr std::uint32_t LayerBit(RenderLayer layer)
  {
    return 1u << static_cast<std::uint32_t>(layer);
  }

  // Purely-visual day/night cycle controls.
  //
  // This is implemented as a lightweight, screen-space color grade (night darkening + optional
  // dusk tint) plus a second pass that draws small emissive "city lights" (streetlights + windows)
  // when it is dark.
  struct DayNightSettings {
    bool enabled = false;

    // Seconds per full cycle (day + night). (For reference: 180s is 3 minutes.)
    float dayLengthSec = 180.0f;

    // Phase offset in seconds (lets you pick a preferred time-of-day).
    float timeOffsetSec = 0.0f;

    // 0..1 how strong the night darkening overlay is.
    float nightDarken = 0.70f;

    // 0..1 how strong the warm sunrise/sunset tint is.
    float duskTint = 0.35f;

    // Draw emissive lights at night (streetlights + building windows).
    bool drawLights = true;
  };


  // Procedural weather / atmosphere controls.
  //
  // Rendered as:
  //  - subtle overcast grading (rain/snow),
  //  - optional ground effects (wet road sheen / snow cover / thin ice),
  //  - optional screen-space particles (rain streaks / snowflakes) and fog gradient.
  struct WeatherSettings {
    enum class Mode : std::uint8_t { Clear = 0, Rain = 1, Snow = 2 };
    Mode mode = Mode::Clear;

    // 0..1 precipitation intensity (rain streak density / snowflake density).
    float intensity = 0.80f;

    // Wind direction in degrees in screen-space (0=right, 90=down).
    float windAngleDeg = 110.0f;

    // Multiplier for particle motion.
    float windSpeed = 1.0f;

    // 0..1 how strong the scene-wide overcast grade is (rain/snow).
    float overcast = 0.55f;

    // 0..1 how strong the fog gradient is (screen-space; stronger at top of screen).
    float fog = 0.35f;

    // Apply per-tile ground effects (wet sheen / snow cover).
    bool affectGround = true;

    // Apply screen-space effects (fog + precipitation particles).
    // Note: this is separate from affectGround so capture tools can export the
    // world without post-FX while still baking wetness/snow into tiles.
    bool affectScreen = true;

    // Draw precipitation particles (rain streaks / snowflakes).
    bool drawParticles = true;

    // When raining at night, draw a cheap "reflection" under lights on roads.
    bool reflectLights = true;
  };

  // Soft, large-scale cloud shadow mask rendered in world space (purely procedural).
  //
  // This is separate from ShadowSettings (building-cast shadows) and is intended to add
  // slow-moving atmospheric lighting variation over the whole map.
  struct CloudShadowSettings {
    bool enabled = true;

    // 0..1 overall opacity multiplier.
    float strength = 0.22f;

    // World-space feature size multiplier (higher => larger clouds).
    float scale = 1.75f;

    // Speed multiplier applied to weather wind (higher => faster movement).
    float speed = 0.35f;

    // 0..1 approximate coverage amount (higher => more clouds).
    float coverage = 0.58f;

    // 0..1 edge softness (higher => blurrier edges).
    float softness = 0.70f;

    // 0..1 baseline cloudiness used when the weather mode is Clear.
    //
    // If 0, cloud shadows only appear when precipitation weather is enabled (rain/snow).
    float clearAmount = 0.0f;
  };

  // Visible volumetric cloud layer (procedural, shader-based).
  //
  // This draws a soft ray-marched cloud volume that is anchored in world space so it pans
  // with the camera yet drifts over time with the wind. It is intentionally subtle and fades
  // toward the bottom of the screen to keep gameplay readable.
  struct VolumetricCloudSettings {
    bool enabled = true;

    // Overall opacity multiplier for the visible cloud layer.
    float opacity = 0.55f;

    // Approximate coverage amount (higher => more sky covered).
    float coverage = 0.55f;

    // Density/thickness multiplier (higher => thicker/more opaque clouds).
    float density = 0.75f;

    // World-space feature size multiplier (higher => larger cloud structures).
    float scale = 1.75f;

    // Speed multiplier applied to weather wind (higher => faster drift).
    float speed = 0.25f;

    // Edge softness (higher => softer boundaries).
    float softness = 0.70f;

    // Ray-march step count (higher => smoother, but slower).
    int steps = 24;

    // How strongly clouds fade near the bottom of the view (0=no fade, 1=strong fade).
    float bottomFade = 0.85f;

    // 0..1 baseline cloudiness used when the weather mode is Clear.
    //
    // If 0, volumetric clouds only appear when precipitation weather is enabled (rain/snow).
    float clearAmount = 0.0f;
  };

  // Ground shadow casting from structures (buildings).
  //
  // This is a cheap, purely-2D "drop shadow" rendered as a darkened polygon projected from the
  // building footprint based on a sun altitude + azimuth. It is intentionally stylized and fast:
  // no raytracing, no per-tile intersection.
  struct ShadowSettings {
    bool enabled = true;

    // 0..1 overall opacity multiplier.
    float strength = 0.65f;

    // 0..1 soft edge amount (draws a few expanded copies with low alpha).
    float softness = 0.55f;

    // Maximum shadow length in tiles (prevents extreme sunrise/sunset streaks).
    float maxLengthTiles = 4.5f;

    // Shadow direction in degrees (screen-space): 0=right, 90=down.
    // (Think "where the shadow goes", not "where the sun is".)
    float azimuthDeg = 45.0f;

    // Sun altitude range used when day/night is enabled.
    // At sunrise/sunset we use minAltitudeDeg, at noon we use maxAltitudeDeg.
    float minAltitudeDeg = 12.0f;
    float maxAltitudeDeg = 70.0f;
  };

  // Lightweight procedural vehicle sprites (used by the vehicle micro-sim overlay).
  //
  // These are generated at runtime (like terrain/road textures) so the project stays asset-free.
  // Sprites are grouped into two screen-space diagonal orientations to match the isometric grid.
  struct VehicleSprite {
    Texture2D color{};
    Texture2D emissive{}; // optional (headlights)
    int pivotX = 0;
    int pivotY = 0;
  };

  // Tall-ish procedural prop sprites (trees, streetlights, etc.).
  //
  // These are rendered as world-space sprites anchored to the isometric tile grid,
  // keeping the project asset-free while improving “city life” visuals when zoomed in.
  struct PropSprite {
    Texture2D color{};
    Texture2D emissive{}; // optional (e.g., streetlight glow)
    int pivotX = 0;
    int pivotY = 0;
  };

  // Simple depth-sorted sprite primitive that can be injected into the world render.
  //
  // The renderer's world pass is drawn in a deterministic isometric order (diagonals / x+y).
  // Dynamic entities (vehicles, effects, etc.) can be supplied as sprites and will be composited
  // into that order so they can be properly occluded by buildings in front of them.
  //
  // Sort key:
  //  - sortSum: primary diagonal order (tileX + tileY anchor)
  //  - sortX: secondary order within the same diagonal (typically tileX)
  //
  // Sprites with emissive=true are drawn after the grading/night pass in additive blend mode.
  struct WorldSprite {
    int sortSum = 0;
    float sortX = 0.0f;

    const Texture2D* tex = nullptr;
    Rectangle src{0.0f, 0.0f, 0.0f, 0.0f};
    Rectangle dst{0.0f, 0.0f, 0.0f, 0.0f};
    Vector2 origin{0.0f, 0.0f};
    float rotation = 0.0f;
    Color tint{255, 255, 255, 255};
    bool emissive = false;
  };

  // Fetch a deterministic variant for the requested diagonal orientation.
  // - slopePositive: true for directions with positive screen-space slope (dir.x * dir.y >= 0).
  // - style: stable per-vehicle style id (usually random at spawn time).
  const VehicleSprite* carSprite(bool slopePositive, int style) const;
  const VehicleSprite* truckSprite(bool slopePositive, int style) const;

  // 0..1 night amount for the configured day/night settings at timeSec.
  float nightFactor(float timeSec) const;



  Renderer(int tileW, int tileH, std::uint64_t seed);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  void rebuildTextures(std::uint64_t seed);

  // Force recompilation of any optional on-disk shader overrides.
  // (Useful when iterating on custom GLSL files.)
  void reloadShaderOverrides();

  void setElevationSettings(const ElevationSettings& s)
  {
    m_elev = s;
    // Elevation changes world-space placement; cached base layers must be redrawn.
    markBaseCacheDirtyAll();
  }
  const ElevationSettings& elevationSettings() const { return m_elev; }

  // Performance: optional render cache for the static base world.
  //
  // Note: the cache only covers the *static* parts of the scene (terrain/cliffs and, when possible,
  // base overlays like roads/zones). Per-tile debug overlays, weather FX, and zone buildings are
  // still drawn dynamically on top.
  void setBaseCacheEnabled(bool enabled) { m_useBandCache = enabled; }
  bool baseCacheEnabled() const { return m_useBandCache; }

  // Multi-layer rendering controls.
  void setLayerMask(std::uint32_t mask) { m_layerMask = mask; }
  std::uint32_t layerMask() const { return m_layerMask; }

  void setLayerEnabled(RenderLayer layer, bool enabled)
  {
    const std::uint32_t bit = LayerBit(layer);
    if (enabled) m_layerMask |= bit;
    else m_layerMask &= ~bit;
  }

  bool layerEnabled(RenderLayer layer) const
  {
    return (m_layerMask & LayerBit(layer)) != 0u;
  }

  // Day/night cycle controls.
  void setDayNightSettings(const DayNightSettings& s) { m_dayNight = s; }
  const DayNightSettings& dayNightSettings() const { return m_dayNight; }

  void setDayNightEnabled(bool enabled) { m_dayNight.enabled = enabled; }
  bool dayNightEnabled() const { return m_dayNight.enabled; }


  // Weather controls.
  void setWeatherSettings(const WeatherSettings& s) { m_weather = s; }
  const WeatherSettings& weatherSettings() const { return m_weather; }

  void setWeatherMode(WeatherSettings::Mode mode) { m_weather.mode = mode; }
  WeatherSettings::Mode weatherMode() const { return m_weather.mode; }
  bool weatherEnabled() const { return m_weather.mode != WeatherSettings::Mode::Clear; }

  // Procedural animated "organic material" overlay (reaction-diffusion texture).
  void setOrganicMaterialSettings(const OrganicMaterial::Settings& s) { m_organicSettings = s; }
  const OrganicMaterial::Settings& organicMaterialSettings() const { return m_organicSettings; }
  bool organicMaterialEnabled() const { return m_organicSettings.enabled; }

  // Reseed the organic material simulation (keeps current settings).
  void resetOrganicMaterial(std::uint32_t seed);

  // Cloud shadow controls (procedural atmospheric shadows).
  void setCloudShadowSettings(const CloudShadowSettings& s);
  const CloudShadowSettings& cloudShadowSettings() const { return m_cloudShadows; }

  // Volumetric cloud controls (visible cloud layer).
  void setVolumetricCloudSettings(const VolumetricCloudSettings& s) { m_volClouds = s; }
  const VolumetricCloudSettings& volumetricCloudSettings() const { return m_volClouds; }

  void setVolumetricCloudsEnabled(bool enabled) { m_volClouds.enabled = enabled; }
  bool volumetricCloudsEnabled() const { return m_volClouds.enabled; }

  // Shadow controls (stylized building ground shadows).
  void setShadowSettings(const ShadowSettings& s) { m_shadows = s; }
  const ShadowSettings& shadowSettings() const { return m_shadows; }

  void setShadowsEnabled(bool enabled) { m_shadows.enabled = enabled; }
  bool shadowsEnabled() const { return m_shadows.enabled; }


  // Mark cached base world dirty (re-rendered lazily when drawn).
  void markBaseCacheDirtyAll();
  void markBaseCacheDirtyForTiles(const std::vector<Point>& tiles, int mapW, int mapH);

  // Screen-space minimap layout helper (used by the game for hit-testing).
  struct MinimapLayout {
    Rectangle rect{};           // destination rectangle in screen space
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
  bool exportWorldOverview(const World& world, const char* fileName, int maxSize = 4096,
                         float timeSec = 0.0f, bool includeScreenFx = true);

  // Optional callbacks invoked inside the world pass (after base terrain/decals/structures draw).
  //
  // This lets the game integrate world-space overlays (e.g. vehicles) so they inherit
  // day/night grading, wetness, and other post effects.
  using WorldOverlayCallback = std::function<void(const Camera2D& camera)>;

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
                 bool showDistrictBorders = false,
                 bool mergeZoneBuildings = true,
                 const WorldOverlayCallback& drawBeforeFx = WorldOverlayCallback{},
                 const WorldOverlayCallback& drawAfterFx = WorldOverlayCallback{},
                 const std::vector<WorldSprite>* sprites = nullptr);

  // Screen-space weather effects (particles + fog). Draw after the world pass but before UI/HUD.
  void drawWeatherScreenFX(int screenW, int screenH, float timeSec, bool allowAestheticDetails = true);

  void drawHUD(const World& world, const Camera2D& camera, Tool tool, int roadBuildLevel,
               std::optional<Point> hovered, int screenW, int screenH, bool showHelp, int brushRadius,
               int undoCount, int redoCount, bool simPaused, float simSpeed, int saveSlot, bool showMinimap,
               const char* inspectInfo, const char* heatmapInfo);

private:
  // Procedural terrain textures are generated at runtime. Using multiple variants
  // per terrain type dramatically reduces visible tiling repetition without any
  // external art assets.
  static constexpr int kTerrainVariants = 8;
  static constexpr int kTerrainTypes = 3;

  static constexpr int kRoadLevels = 3;
  static constexpr int kRoadVariants = 4;

  int m_tileW = 64;
  int m_tileH = 32;

  ElevationSettings m_elev{};

  DayNightSettings m_dayNight{};

  WeatherSettings m_weather{};

  // Animated reaction-diffusion "organic material" overlay.
  OrganicMaterial m_organicMaterial{};
  OrganicMaterial::Settings m_organicSettings{};
  float m_organicLastTimeSec = 0.0f;
  bool m_organicHasLastTime = false;

  CloudShadowSettings m_cloudShadows{};
  Texture2D m_cloudShadowTex{};

  // Visible volumetric cloud overlay resources.
  VolumetricCloudSettings m_volClouds{};
  Shader m_volCloudShader{};
  bool m_volCloudShaderFailed = false;

  int m_volCloudLocViewMin = -1;
  int m_volCloudLocViewSize = -1;
  int m_volCloudLocTime = -1;
  int m_volCloudLocWindDir = -1;
  int m_volCloudLocWindSpeed = -1;
  int m_volCloudLocScale = -1;
  int m_volCloudLocCoverage = -1;
  int m_volCloudLocDensity = -1;
  int m_volCloudLocSoftness = -1;
  int m_volCloudLocSteps = -1;
  int m_volCloudLocDay = -1;
  int m_volCloudLocDusk = -1;
  int m_volCloudLocOvercast = -1;
  int m_volCloudLocSeed = -1;
  int m_volCloudLocBottomFade = -1;

  ShadowSettings m_shadows{};

  std::array<std::array<Texture2D, kTerrainVariants>, kTerrainTypes> m_terrainTex{};

  // Terrain transitions (auto-tiling) for shorelines and biome edges.
  //
  // Masks use the same 4-bit layout as roads (see World::computeRoadMask):
  //  0x01 = (x, y-1)  (screen up-right)
  //  0x02 = (x+1, y)  (screen down-right)
  //  0x04 = (x, y+1)  (screen down-left)
  //  0x08 = (x-1, y)  (screen up-left)
  //
  // For each mask and each terrain variant we pre-bake a blended tile:
  //  - Water->Sand (shoreline + optional foam)
  //  - Sand->Grass (biome edge)
  std::array<std::array<Texture2D, kTerrainVariants>, 16> m_terrainTransWaterSand{};
  std::array<std::array<Texture2D, kTerrainVariants>, 16> m_terrainTransSandGrass{};
  std::array<Texture2D, 6> m_overlayTex{};

  // Roads are auto-tiled: for each road class (level 1..3) and each connection mask (0..15)
  // we keep several procedural variants to reduce visible repetition.
  std::array<std::array<std::array<Texture2D, kRoadVariants>, 16>, kRoadLevels> m_roadTex{};

  // Bridges share the same mask/variant layout as roads but are drawn on water.
  std::array<std::array<std::array<Texture2D, kRoadVariants>, 16>, kRoadLevels> m_bridgeTex{};

  // Vehicle overlay sprite variants (two diagonal orientations).
  std::vector<VehicleSprite> m_vehicleCarPosSlope;
  std::vector<VehicleSprite> m_vehicleCarNegSlope;
  std::vector<VehicleSprite> m_vehicleTruckPosSlope;
  std::vector<VehicleSprite> m_vehicleTruckNegSlope;

  // World prop sprite variants (trees / streetlights). These are rendered directly in the world pass
  // (not baked into band caches) because they can extend well above the tile diamond.
  std::vector<PropSprite> m_propTreeDeciduous;
  std::vector<PropSprite> m_propTreeConifer;
  std::vector<PropSprite> m_propStreetLight;
  std::vector<PropSprite> m_propPedestrian;

  // Procedural building sprite variants (zone buildings). These are generated at runtime so the
  // project stays asset-free, but provide much richer visuals than simple geometric prisms when
  // zoomed in.
  struct BuildingSprite {
    Texture2D color{};
    Texture2D emissive{}; // optional (lit windows / signage)
    int pivotX = 0;
    int pivotY = 0;
  };

  // Indexed by [level-1] => variant list.
  std::array<std::vector<BuildingSprite>, 3> m_buildingResidential;
  std::array<std::vector<BuildingSprite>, 3> m_buildingCommercial;
  std::array<std::vector<BuildingSprite>, 3> m_buildingIndustrial;

  void unloadVehicleSprites();
  void rebuildVehicleSprites();

  void unloadBuildingSprites();
  void rebuildBuildingSprites();

  void unloadPropSprites();
  void rebuildPropSprites();

  void rebuildCloudShadowTexture();

  void ensureVolumetricCloudShader();
  void unloadVolumetricCloudResources();
  void drawVolumetricCloudLayer(const WorldRect& viewAABB, float tileW, float timeSec,
                                float day, float dusk, float overcast,
                                float windX, float windY, float windSpeed);


  // 32-bit seed used for procedural render details (ties visual variety to world seed).
  std::uint32_t m_gfxSeed32 = 0;

  // Minimap texture (one pixel per tile). This is purely a UI convenience.
  Texture2D m_minimapTex{};
  int m_minimapW = 0;
  int m_minimapH = 0;
  std::vector<Color> m_minimapPixels;
  bool m_minimapDirty = true;

  // Base world render cache (static layers baked into render textures).
  struct BandCache {
    // Static base layers are cached separately to support multi-layer rendering and to allow
    // some overlays to remain dynamic without invalidating everything.
    RenderTexture2D terrain{};     // terrain tops + cliff walls
    RenderTexture2D structures{};  // roads/zones/parks (only when not in utility overlay mode)

    int sum0 = 0;     // inclusive
    int sum1 = 0;     // inclusive
    Vector2 origin{}; // world-space top-left where this band texture should be drawn

    bool dirtyTerrain = true;
    bool dirtyStructures = true;
  };

  static constexpr int kBandSums = 8; // how many (x+y) diagonals are baked per band

  bool m_useBandCache = true;
  std::uint32_t m_layerMask = kLayerAll;
  int m_bandMapW = 0;
  int m_bandMapH = 0;
  float m_bandMaxPixels = 0.0f;
  bool m_bandCacheDirtyAll = true;
  std::vector<BandCache> m_bands;

  // Scratch storage for merged zone-building parcels (reused every frame).
  ZoneBuildingParcels m_zoneParcelsScratch;

  // Optional GPU ribbon renderer (uses a geometry shader when available).
  // If unsupported, it stays disabled and we fall back to CPU overlays.
  GpuRibbonPathRenderer m_gpuRibbon;

  // Scratch buffer for GPU path ribbons (reused when rendering highlight paths).
  std::vector<Vector2> m_pathRibbonScratch;

  void unloadTextures();

  void unloadBaseCache();
  void ensureBaseCache(const World& world);
  void rebuildTerrainCacheBand(const World& world, BandCache& band);
  void rebuildStructureCacheBand(const World& world, BandCache& band);

  void unloadMinimap();
  void ensureMinimapUpToDate(const World& world);

  // Resolve a terrain tile texture, optionally replacing it with a transition tile when the
  // tile borders a different biome (shorelines / sand->grass blends).
  Texture2D& terrainWithTransitions(const World& world, int x, int y, const Tile& t);

  Texture2D& terrain(Terrain t, std::uint8_t variation);
  Texture2D& overlay(Overlay o);
  Texture2D& road(std::uint8_t mask, std::uint8_t variation, std::uint8_t level);
  Texture2D& bridge(std::uint8_t mask, std::uint8_t variation, std::uint8_t level);

  static Color BrightnessTint(float b);
};

} // namespace isocity
