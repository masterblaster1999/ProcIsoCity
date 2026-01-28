#include "isocity/VisualPrefs.hpp"

#include "isocity/FileSync.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace isocity {

namespace {

static bool IsFiniteDouble(double v)
{
  return std::isfinite(v) != 0;
}

static bool GetObj(const JsonValue& obj, const std::string& key, const JsonValue** out)
{
  const JsonValue* v = FindJsonMember(obj, key);
  if (!v) return false;
  if (!v->isObject()) return false;
  *out = v;
  return true;
}

static bool ApplyBool(const JsonValue& root, const char* key, bool& io, std::string& err)
{
  const JsonValue* v = FindJsonMember(root, key);
  if (!v) return true; // missing => keep
  if (!v->isBool()) {
    err = std::string("expected boolean for key '") + key + "'";
    return false;
  }
  io = v->boolValue;
  return true;
}

static bool ApplyI32(const JsonValue& root, const char* key, int& io, std::string& err)
{
  const JsonValue* v = FindJsonMember(root, key);
  if (!v) return true;
  if (!v->isNumber()) {
    err = std::string("expected number for key '") + key + "'";
    return false;
  }
  if (!IsFiniteDouble(v->numberValue)) {
    err = std::string("non-finite number for key '") + key + "'";
    return false;
  }
  io = static_cast<int>(std::lround(v->numberValue));
  return true;
}

static bool ApplyU32(const JsonValue& root, const char* key, std::uint32_t& io, std::string& err)
{
  int tmp = static_cast<int>(io);
  if (!ApplyI32(root, key, tmp, err)) return false;
  io = static_cast<std::uint32_t>(tmp);
  return true;
}

static bool ApplyF32(const JsonValue& root, const char* key, float& io, std::string& err)
{
  const JsonValue* v = FindJsonMember(root, key);
  if (!v) return true;
  if (!v->isNumber()) {
    err = std::string("expected number for key '") + key + "'";
    return false;
  }
  if (!IsFiniteDouble(v->numberValue)) {
    err = std::string("non-finite number for key '") + key + "'";
    return false;
  }
  const double dv = v->numberValue;
  if (dv < -static_cast<double>(std::numeric_limits<float>::max()) ||
      dv > static_cast<double>(std::numeric_limits<float>::max())) {
    err = std::string("out-of-range float for key '") + key + "'";
    return false;
  }
  io = static_cast<float>(dv);
  return true;
}


static void Indent(std::ostringstream& oss, int n)
{
  for (int i = 0; i < n; ++i) oss << ' ';
}

static void WriteBool(std::ostringstream& oss, bool v)
{
  oss << (v ? "true" : "false");
}

static std::string FloatToJson(float v)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << static_cast<double>(v);
  std::string s = oss.str();
  // Trim trailing zeros.
  while (s.size() > 1 && s.find('.') != std::string::npos && s.back() == '0') {
    s.pop_back();
  }
  if (!s.empty() && s.back() == '.') s.pop_back();
  if (s.empty()) s = "0";
  return s;
}

static std::string WeatherModeToString(Renderer::WeatherSettings::Mode m)
{
  switch (m) {
  case Renderer::WeatherSettings::Mode::Clear: return "clear";
  case Renderer::WeatherSettings::Mode::Rain: return "rain";
  case Renderer::WeatherSettings::Mode::Snow: return "snow";
  default: return "clear";
  }
}

static bool ParseWeatherMode(const JsonValue& v, Renderer::WeatherSettings::Mode& out, std::string& err)
{
  if (v.isString()) {
    std::string s = v.stringValue;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (s == "clear" || s == "off" || s == "none") {
      out = Renderer::WeatherSettings::Mode::Clear;
      return true;
    }
    if (s == "rain") {
      out = Renderer::WeatherSettings::Mode::Rain;
      return true;
    }
    if (s == "snow") {
      out = Renderer::WeatherSettings::Mode::Snow;
      return true;
    }
    err = "unknown weather mode: " + v.stringValue;
    return false;
  }

  if (v.isNumber()) {
    if (!IsFiniteDouble(v.numberValue)) {
      err = "non-finite weather.mode";
      return false;
    }
    const int m = static_cast<int>(std::lround(v.numberValue));
    if (m == 0) out = Renderer::WeatherSettings::Mode::Clear;
    else if (m == 1) out = Renderer::WeatherSettings::Mode::Rain;
    else if (m == 2) out = Renderer::WeatherSettings::Mode::Snow;
    else {
      err = "invalid weather.mode (expected 0..2)";
      return false;
    }
    return true;
  }

  err = "expected string or number for weather.mode";
  return false;
}

static bool ApplyLayersObject(const JsonValue& obj, std::uint32_t& ioMask, std::string& err)
{
  const JsonValue* layers = FindJsonMember(obj, "layers");
  if (!layers) return true;

  if (layers->isNumber()) {
    std::uint32_t tmp = ioMask;
    if (!ApplyU32(obj, "layers", tmp, err)) return false;
    ioMask = tmp;
    return true;
  }

  if (!layers->isObject()) {
    err = "expected 'layers' to be an object or number";
    return false;
  }

  const auto setIf = [&](const char* key, Renderer::RenderLayer layer) {
    const JsonValue* v = FindJsonMember(*layers, key);
    if (!v) return true;
    if (!v->isBool()) {
      err = std::string("expected boolean for layers.") + key;
      return false;
    }
    const std::uint32_t bit = Renderer::LayerBit(layer);
    if (v->boolValue) ioMask |= bit;
    else ioMask &= ~bit;
    return true;
  };

  if (!setIf("terrain", Renderer::RenderLayer::Terrain)) return false;
  if (!setIf("decals", Renderer::RenderLayer::Decals)) return false;
  if (!setIf("structures", Renderer::RenderLayer::Structures)) return false;
  if (!setIf("overlays", Renderer::RenderLayer::Overlays)) return false;

  return true;
}

static bool NearlyEqual(float a, float b, float eps = 1e-4f)
{
  return std::abs(a - b) <= eps;
}

} // namespace

bool VisualPrefsEqual(const VisualPrefs& a, const VisualPrefs& b)
{
  if (a.vsync != b.vsync) return false;

  if (a.uiScaleAuto != b.uiScaleAuto) return false;
  if (!NearlyEqual(a.uiScaleManual, b.uiScaleManual)) return false;

  // UI theme
  if (a.uiTheme.accentFromSeed != b.uiTheme.accentFromSeed) return false;
  if (!NearlyEqual(a.uiTheme.accentHueDeg, b.uiTheme.accentHueDeg)) return false;
  if (!NearlyEqual(a.uiTheme.accentSaturation, b.uiTheme.accentSaturation)) return false;
  if (!NearlyEqual(a.uiTheme.accentValue, b.uiTheme.accentValue)) return false;
  if (!NearlyEqual(a.uiTheme.roundness, b.uiTheme.roundness)) return false;
  if (a.uiTheme.roundSegments != b.uiTheme.roundSegments) return false;
  if (!NearlyEqual(a.uiTheme.noiseAlpha, b.uiTheme.noiseAlpha)) return false;
  if (!NearlyEqual(a.uiTheme.noiseScale, b.uiTheme.noiseScale)) return false;
  if (!NearlyEqual(a.uiTheme.headerSheenStrength, b.uiTheme.headerSheenStrength)) return false;
  if (a.uiTheme.fontAtlasScale != b.uiTheme.fontAtlasScale) return false;
  if (a.uiTheme.fontFilterPoint != b.uiTheme.fontFilterPoint) return false;

  if (a.worldRenderScaleAuto != b.worldRenderScaleAuto) return false;
  if (!NearlyEqual(a.worldRenderScale, b.worldRenderScale)) return false;
  if (!NearlyEqual(a.worldRenderScaleMin, b.worldRenderScaleMin)) return false;
  if (!NearlyEqual(a.worldRenderScaleMax, b.worldRenderScaleMax)) return false;
  if (a.worldRenderTargetFps != b.worldRenderTargetFps) return false;
  if (a.worldRenderFilterPoint != b.worldRenderFilterPoint) return false;

  if (a.mergedZoneBuildings != b.mergedZoneBuildings) return false;

  if (a.baseCacheEnabled != b.baseCacheEnabled) return false;
  if (a.layerMask != b.layerMask) return false;

  // Shadows
  if (a.shadows.enabled != b.shadows.enabled) return false;
  if (!NearlyEqual(a.shadows.strength, b.shadows.strength)) return false;
  if (!NearlyEqual(a.shadows.softness, b.shadows.softness)) return false;
  if (!NearlyEqual(a.shadows.maxLengthTiles, b.shadows.maxLengthTiles)) return false;
  if (!NearlyEqual(a.shadows.azimuthDeg, b.shadows.azimuthDeg)) return false;
  if (!NearlyEqual(a.shadows.minAltitudeDeg, b.shadows.minAltitudeDeg)) return false;
  if (!NearlyEqual(a.shadows.maxAltitudeDeg, b.shadows.maxAltitudeDeg)) return false;

  // Day/night
  if (a.dayNight.enabled != b.dayNight.enabled) return false;
  if (!NearlyEqual(a.dayNight.dayLengthSec, b.dayNight.dayLengthSec)) return false;
  if (!NearlyEqual(a.dayNight.timeOffsetSec, b.dayNight.timeOffsetSec)) return false;
  if (!NearlyEqual(a.dayNight.nightDarken, b.dayNight.nightDarken)) return false;
  if (!NearlyEqual(a.dayNight.duskTint, b.dayNight.duskTint)) return false;
  if (a.dayNight.drawLights != b.dayNight.drawLights) return false;

  // Weather
  if (a.weather.mode != b.weather.mode) return false;
  if (!NearlyEqual(a.weather.intensity, b.weather.intensity)) return false;
  if (!NearlyEqual(a.weather.windAngleDeg, b.weather.windAngleDeg)) return false;
  if (!NearlyEqual(a.weather.windSpeed, b.weather.windSpeed)) return false;
  if (!NearlyEqual(a.weather.overcast, b.weather.overcast)) return false;
  if (!NearlyEqual(a.weather.fog, b.weather.fog)) return false;
  if (a.weather.affectGround != b.weather.affectGround) return false;
  if (a.weather.drawParticles != b.weather.drawParticles) return false;
  if (a.weather.reflectLights != b.weather.reflectLights) return false;


  // Material FX
  if (a.materialFx.enabled != b.materialFx.enabled) return false;
  if (!NearlyEqual(a.materialFx.scale, b.materialFx.scale)) return false;
  if (!NearlyEqual(a.materialFx.waterStrength, b.materialFx.waterStrength)) return false;
  if (!NearlyEqual(a.materialFx.waterDistortPx, b.materialFx.waterDistortPx)) return false;
  if (!NearlyEqual(a.materialFx.waterSparkle, b.materialFx.waterSparkle)) return false;
  if (!NearlyEqual(a.materialFx.foamStrength, b.materialFx.foamStrength)) return false;
  if (!NearlyEqual(a.materialFx.foamWidthPx, b.materialFx.foamWidthPx)) return false;
  if (!NearlyEqual(a.materialFx.causticsStrength, b.materialFx.causticsStrength)) return false;
  if (!NearlyEqual(a.materialFx.wetSandStrength, b.materialFx.wetSandStrength)) return false;
  if (!NearlyEqual(a.materialFx.wetSandWidthPx, b.materialFx.wetSandWidthPx)) return false;
  if (!NearlyEqual(a.materialFx.vegetationStrength, b.materialFx.vegetationStrength)) return false;

  // Cloud shadows
  if (a.cloudShadows.enabled != b.cloudShadows.enabled) return false;
  if (!NearlyEqual(a.cloudShadows.strength, b.cloudShadows.strength)) return false;
  if (!NearlyEqual(a.cloudShadows.scale, b.cloudShadows.scale)) return false;
  if (!NearlyEqual(a.cloudShadows.speed, b.cloudShadows.speed)) return false;
  if (!NearlyEqual(a.cloudShadows.evolve, b.cloudShadows.evolve)) return false;
  if (!NearlyEqual(a.cloudShadows.coverage, b.cloudShadows.coverage)) return false;
  if (!NearlyEqual(a.cloudShadows.softness, b.cloudShadows.softness)) return false;
  if (!NearlyEqual(a.cloudShadows.clearAmount, b.cloudShadows.clearAmount)) return false;

  // Volumetric clouds
  if (a.volumetricClouds.enabled != b.volumetricClouds.enabled) return false;
  if (!NearlyEqual(a.volumetricClouds.opacity, b.volumetricClouds.opacity)) return false;
  if (!NearlyEqual(a.volumetricClouds.coverage, b.volumetricClouds.coverage)) return false;
  if (!NearlyEqual(a.volumetricClouds.density, b.volumetricClouds.density)) return false;
  if (!NearlyEqual(a.volumetricClouds.scale, b.volumetricClouds.scale)) return false;
  if (!NearlyEqual(a.volumetricClouds.speed, b.volumetricClouds.speed)) return false;
  if (!NearlyEqual(a.volumetricClouds.softness, b.volumetricClouds.softness)) return false;
  if (a.volumetricClouds.steps != b.volumetricClouds.steps) return false;
  if (!NearlyEqual(a.volumetricClouds.bottomFade, b.volumetricClouds.bottomFade)) return false;
  if (!NearlyEqual(a.volumetricClouds.clearAmount, b.volumetricClouds.clearAmount)) return false;

  // Post FX
  if (a.postFx.enabled != b.postFx.enabled) return false;
  if (a.postFx.colorBits != b.postFx.colorBits) return false;
  if (!NearlyEqual(a.postFx.ditherStrength, b.postFx.ditherStrength)) return false;
  if (!NearlyEqual(a.postFx.grain, b.postFx.grain)) return false;
  if (!NearlyEqual(a.postFx.vignette, b.postFx.vignette)) return false;
  if (!NearlyEqual(a.postFx.chroma, b.postFx.chroma)) return false;
  if (!NearlyEqual(a.postFx.scanlines, b.postFx.scanlines)) return false;
  if (!NearlyEqual(a.postFx.fxaa, b.postFx.fxaa)) return false;
  if (!NearlyEqual(a.postFx.sharpen, b.postFx.sharpen)) return false;

  if (a.postFx.tonemapEnabled != b.postFx.tonemapEnabled) return false;
  if (!NearlyEqual(a.postFx.exposure, b.postFx.exposure)) return false;
  if (!NearlyEqual(a.postFx.contrast, b.postFx.contrast)) return false;
  if (!NearlyEqual(a.postFx.saturation, b.postFx.saturation)) return false;

  if (!NearlyEqual(a.postFx.outline, b.postFx.outline)) return false;
  if (!NearlyEqual(a.postFx.outlineThreshold, b.postFx.outlineThreshold)) return false;
  if (!NearlyEqual(a.postFx.outlineThickness, b.postFx.outlineThickness)) return false;
  if (a.postFx.taaEnabled != b.postFx.taaEnabled) return false;
  if (!NearlyEqual(a.postFx.taaHistory, b.postFx.taaHistory)) return false;
  if (!NearlyEqual(a.postFx.taaJitter, b.postFx.taaJitter)) return false;
  if (!NearlyEqual(a.postFx.taaResponse, b.postFx.taaResponse)) return false;
  if (a.postFx.includeWeather != b.postFx.includeWeather) return false;

  // Lens precipitation (rain on lens / wet camera)
  if (!NearlyEqual(a.postFx.lensWeather, b.postFx.lensWeather)) return false;
  if (!NearlyEqual(a.postFx.lensDistort, b.postFx.lensDistort)) return false;
  if (!NearlyEqual(a.postFx.lensScale, b.postFx.lensScale)) return false;
  if (!NearlyEqual(a.postFx.lensDrips, b.postFx.lensDrips)) return false;

  if (!NearlyEqual(a.postFx.bloom, b.postFx.bloom)) return false;
  if (!NearlyEqual(a.postFx.bloomThreshold, b.postFx.bloomThreshold)) return false;
  if (!NearlyEqual(a.postFx.bloomKnee, b.postFx.bloomKnee)) return false;
  if (!NearlyEqual(a.postFx.bloomRadius, b.postFx.bloomRadius)) return false;
  if (a.postFx.bloomDownsample != b.postFx.bloomDownsample) return false;

  // Elevation
  if (!NearlyEqual(a.elevation.maxPixels, b.elevation.maxPixels)) return false;
  if (a.elevation.quantizeSteps != b.elevation.quantizeSteps) return false;
  if (a.elevation.flattenWater != b.elevation.flattenWater) return false;

  return true;
}

std::string VisualPrefsToJson(const VisualPrefs& p, int indentSpaces)
{
  const int indent = std::max(0, indentSpaces);

  std::ostringstream oss;
  oss << "{\n";

  Indent(oss, indent);
  oss << "\"version\": 1,\n";

  // Display
  Indent(oss, indent);
  oss << "\"display\": {\n";
  Indent(oss, indent * 2);
  oss << "\"vsync\": ";
  WriteBool(oss, p.vsync);
  oss << ",\n";
  Indent(oss, indent * 2);
  oss << "\"ui_scale_auto\": ";
  WriteBool(oss, p.uiScaleAuto);
  oss << ",\n";
  Indent(oss, indent * 2);
  oss << "\"ui_scale_manual\": " << FloatToJson(p.uiScaleManual) << "\n";

  // UI theme
  oss << ",\n";
  Indent(oss, indent * 2);
  oss << "\"ui_theme\": {\n";
  Indent(oss, indent * 3);
  oss << "\"accent_from_seed\": ";
  WriteBool(oss, p.uiTheme.accentFromSeed);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"accent_hue_deg\": " << FloatToJson(p.uiTheme.accentHueDeg) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"accent_saturation\": " << FloatToJson(p.uiTheme.accentSaturation) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"accent_value\": " << FloatToJson(p.uiTheme.accentValue) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"roundness\": " << FloatToJson(p.uiTheme.roundness) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"round_segments\": " << p.uiTheme.roundSegments << ",\n";
  Indent(oss, indent * 3);
  oss << "\"noise_alpha\": " << FloatToJson(p.uiTheme.noiseAlpha) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"noise_scale\": " << FloatToJson(p.uiTheme.noiseScale) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"header_sheen\": " << FloatToJson(p.uiTheme.headerSheenStrength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"font_atlas_scale\": " << p.uiTheme.fontAtlasScale << ",\n";
  Indent(oss, indent * 3);
  oss << "\"font_filter_point\": ";
  WriteBool(oss, p.uiTheme.fontFilterPoint);
  oss << "\n";
  Indent(oss, indent * 2);
  oss << "}\n";
  Indent(oss, indent);
  oss << "},\n";

  // World render scaling
  Indent(oss, indent);
  oss << "\"world_render\": {\n";
  Indent(oss, indent * 2);
  oss << "\"auto\": ";
  WriteBool(oss, p.worldRenderScaleAuto);
  oss << ",\n";
  Indent(oss, indent * 2);
  oss << "\"scale\": " << FloatToJson(p.worldRenderScale) << ",\n";
  Indent(oss, indent * 2);
  oss << "\"min\": " << FloatToJson(p.worldRenderScaleMin) << ",\n";
  Indent(oss, indent * 2);
  oss << "\"max\": " << FloatToJson(p.worldRenderScaleMax) << ",\n";
  Indent(oss, indent * 2);
  oss << "\"target_fps\": " << p.worldRenderTargetFps << ",\n";
  Indent(oss, indent * 2);
  oss << "\"filter_point\": ";
  WriteBool(oss, p.worldRenderFilterPoint);
  oss << "\n";
  Indent(oss, indent);
  oss << "},\n";

  // Renderer / visuals
  Indent(oss, indent);
  oss << "\"renderer\": {\n";

  Indent(oss, indent * 2);
  oss << "\"merged_zone_buildings\": ";
  WriteBool(oss, p.mergedZoneBuildings);
  oss << ",\n";

  Indent(oss, indent * 2);
  oss << "\"gfx_theme\": \"" << GfxThemeName(p.gfxTheme) << "\",\n";

  Indent(oss, indent * 2);
  oss << "\"base_cache\": ";
  WriteBool(oss, p.baseCacheEnabled);
  oss << ",\n";

  Indent(oss, indent * 2);
  oss << "\"layers\": {\n";
  auto writeLayer = [&](const char* name, Renderer::RenderLayer layer, bool comma) {
    Indent(oss, indent * 3);
    oss << "\"" << name << "\": ";
    const bool enabled = (p.layerMask & Renderer::LayerBit(layer)) != 0u;
    WriteBool(oss, enabled);
    if (comma) oss << ",";
    oss << "\n";
  };
  writeLayer("terrain", Renderer::RenderLayer::Terrain, true);
  writeLayer("decals", Renderer::RenderLayer::Decals, true);
  writeLayer("structures", Renderer::RenderLayer::Structures, true);
  writeLayer("overlays", Renderer::RenderLayer::Overlays, false);
  Indent(oss, indent * 2);
  oss << "},\n";

  // Elevation
  Indent(oss, indent * 2);
  oss << "\"elevation\": {\n";
  Indent(oss, indent * 3);
  oss << "\"max_pixels\": " << FloatToJson(p.elevation.maxPixels) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"quantize_steps\": " << p.elevation.quantizeSteps << ",\n";
  Indent(oss, indent * 3);
  oss << "\"flatten_water\": ";
  WriteBool(oss, p.elevation.flattenWater);
  oss << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";

  // Shadows
  Indent(oss, indent * 2);
  oss << "\"shadows\": {\n";
  Indent(oss, indent * 3);
  oss << "\"enabled\": ";
  WriteBool(oss, p.shadows.enabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"strength\": " << FloatToJson(p.shadows.strength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"softness\": " << FloatToJson(p.shadows.softness) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"max_length_tiles\": " << FloatToJson(p.shadows.maxLengthTiles) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"azimuth_deg\": " << FloatToJson(p.shadows.azimuthDeg) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"min_altitude_deg\": " << FloatToJson(p.shadows.minAltitudeDeg) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"max_altitude_deg\": " << FloatToJson(p.shadows.maxAltitudeDeg) << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";

  // Day/Night
  Indent(oss, indent * 2);
  oss << "\"day_night\": {\n";
  Indent(oss, indent * 3);
  oss << "\"enabled\": ";
  WriteBool(oss, p.dayNight.enabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"day_length_sec\": " << FloatToJson(p.dayNight.dayLengthSec) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"time_offset_sec\": " << FloatToJson(p.dayNight.timeOffsetSec) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"night_darken\": " << FloatToJson(p.dayNight.nightDarken) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"dusk_tint\": " << FloatToJson(p.dayNight.duskTint) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"draw_lights\": ";
  WriteBool(oss, p.dayNight.drawLights);
  oss << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";

  // Weather
  Indent(oss, indent * 2);
  oss << "\"weather\": {\n";
  Indent(oss, indent * 3);
  oss << "\"mode\": \"" << JsonEscape(WeatherModeToString(p.weather.mode)) << "\",\n";
  Indent(oss, indent * 3);
  oss << "\"intensity\": " << FloatToJson(p.weather.intensity) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"wind_angle_deg\": " << FloatToJson(p.weather.windAngleDeg) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"wind_speed\": " << FloatToJson(p.weather.windSpeed) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"overcast\": " << FloatToJson(p.weather.overcast) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"fog\": " << FloatToJson(p.weather.fog) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"affect_ground\": ";
  WriteBool(oss, p.weather.affectGround);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"draw_particles\": ";
  WriteBool(oss, p.weather.drawParticles);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"reflect_lights\": ";
  WriteBool(oss, p.weather.reflectLights);
  oss << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";


  // Material FX (shader-based, world-space).
  Indent(oss, indent * 2);
  oss << "\"material_fx\": {\n";
  Indent(oss, indent * 3);
  oss << "\"enabled\": ";
  WriteBool(oss, p.materialFx.enabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"scale\": " << FloatToJson(p.materialFx.scale) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"water_strength\": " << FloatToJson(p.materialFx.waterStrength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"water_distort_px\": " << FloatToJson(p.materialFx.waterDistortPx) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"water_sparkle\": " << FloatToJson(p.materialFx.waterSparkle) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"foam_strength\": " << FloatToJson(p.materialFx.foamStrength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"foam_width_px\": " << FloatToJson(p.materialFx.foamWidthPx) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"caustics_strength\": " << FloatToJson(p.materialFx.causticsStrength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"wet_sand_strength\": " << FloatToJson(p.materialFx.wetSandStrength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"wet_sand_width_px\": " << FloatToJson(p.materialFx.wetSandWidthPx) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"vegetation_strength\": " << FloatToJson(p.materialFx.vegetationStrength) << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";

  // Cloud shadows (procedural, world-space).
  Indent(oss, indent * 2);
  oss << "\"cloud_shadows\": {\n";
  Indent(oss, indent * 3);
  oss << "\"enabled\": ";
  WriteBool(oss, p.cloudShadows.enabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"strength\": " << FloatToJson(p.cloudShadows.strength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"scale\": " << FloatToJson(p.cloudShadows.scale) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"speed\": " << FloatToJson(p.cloudShadows.speed) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"evolve\": " << FloatToJson(p.cloudShadows.evolve) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"coverage\": " << FloatToJson(p.cloudShadows.coverage) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"softness\": " << FloatToJson(p.cloudShadows.softness) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"clear_amount\": " << FloatToJson(p.cloudShadows.clearAmount) << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";

  // Volumetric clouds (shader-based).
  Indent(oss, indent * 2);
  oss << "\"volumetric_clouds\": {\n";
  Indent(oss, indent * 3);
  oss << "\"enabled\": ";
  WriteBool(oss, p.volumetricClouds.enabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"opacity\": " << FloatToJson(p.volumetricClouds.opacity) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"coverage\": " << FloatToJson(p.volumetricClouds.coverage) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"density\": " << FloatToJson(p.volumetricClouds.density) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"scale\": " << FloatToJson(p.volumetricClouds.scale) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"speed\": " << FloatToJson(p.volumetricClouds.speed) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"softness\": " << FloatToJson(p.volumetricClouds.softness) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"steps\": " << p.volumetricClouds.steps << ",\n";
  Indent(oss, indent * 3);
  oss << "\"bottom_fade\": " << FloatToJson(p.volumetricClouds.bottomFade) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"clear_amount\": " << FloatToJson(p.volumetricClouds.clearAmount) << "\n";
  Indent(oss, indent * 2);
  oss << "},\n";

  // Post FX (stylized, screen-space, shader-based).
  Indent(oss, indent * 2);
  oss << "\"post_fx\": {\n";
  Indent(oss, indent * 3);
  oss << "\"enabled\": ";
  WriteBool(oss, p.postFx.enabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"color_bits\": " << p.postFx.colorBits << ",\n";
  Indent(oss, indent * 3);
  oss << "\"dither_strength\": " << FloatToJson(p.postFx.ditherStrength) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"grain\": " << FloatToJson(p.postFx.grain) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"vignette\": " << FloatToJson(p.postFx.vignette) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"chroma\": " << FloatToJson(p.postFx.chroma) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"scanlines\": " << FloatToJson(p.postFx.scanlines) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"fxaa\": " << FloatToJson(p.postFx.fxaa) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"sharpen\": " << FloatToJson(p.postFx.sharpen) << ",\n";

  Indent(oss, indent * 3);
  oss << "\"tonemap_enabled\": ";
  WriteBool(oss, p.postFx.tonemapEnabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"exposure\": " << FloatToJson(p.postFx.exposure) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"contrast\": " << FloatToJson(p.postFx.contrast) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"saturation\": " << FloatToJson(p.postFx.saturation) << ",\n";

  Indent(oss, indent * 3);
  oss << "\"outline\": " << FloatToJson(p.postFx.outline) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"outline_threshold\": " << FloatToJson(p.postFx.outlineThreshold) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"outline_thickness\": " << FloatToJson(p.postFx.outlineThickness) << ",\n";

  Indent(oss, indent * 3);
  oss << "\"taa_enabled\": ";
  WriteBool(oss, p.postFx.taaEnabled);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"taa_history\": " << FloatToJson(p.postFx.taaHistory) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"taa_jitter\": " << FloatToJson(p.postFx.taaJitter) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"taa_response\": " << FloatToJson(p.postFx.taaResponse) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"include_weather\": ";
  WriteBool(oss, p.postFx.includeWeather);
  oss << ",\n";
  Indent(oss, indent * 3);
  oss << "\"lens_weather\": " << FloatToJson(p.postFx.lensWeather) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"lens_distort\": " << FloatToJson(p.postFx.lensDistort) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"lens_scale\": " << FloatToJson(p.postFx.lensScale) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"lens_drips\": " << FloatToJson(p.postFx.lensDrips) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"bloom\": " << FloatToJson(p.postFx.bloom) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"bloom_threshold\": " << FloatToJson(p.postFx.bloomThreshold) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"bloom_knee\": " << FloatToJson(p.postFx.bloomKnee) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"bloom_radius\": " << FloatToJson(p.postFx.bloomRadius) << ",\n";
  Indent(oss, indent * 3);
  oss << "\"bloom_downsample\": " << p.postFx.bloomDownsample << "\n";
  Indent(oss, indent * 2);
  oss << "}\n";

  Indent(oss, indent);
  oss << "}\n"; // renderer

  oss << "}\n"; // root
  return oss.str();
}

bool ApplyVisualPrefsJson(const JsonValue& root, VisualPrefs& ioPrefs, std::string& outError)
{
  if (!root.isObject()) {
    outError = "root must be a JSON object";
    return false;
  }

  // display
  const JsonValue* display = nullptr;
  if (GetObj(root, "display", &display)) {
    if (!ApplyBool(*display, "vsync", ioPrefs.vsync, outError)) return false;
    if (!ApplyBool(*display, "ui_scale_auto", ioPrefs.uiScaleAuto, outError)) return false;
    if (!ApplyF32(*display, "ui_scale_manual", ioPrefs.uiScaleManual, outError)) return false;

    const JsonValue* uiTheme = nullptr;
    if (GetObj(*display, "ui_theme", &uiTheme)) {
      if (!ApplyBool(*uiTheme, "accent_from_seed", ioPrefs.uiTheme.accentFromSeed, outError)) return false;
      if (!ApplyF32(*uiTheme, "accent_hue_deg", ioPrefs.uiTheme.accentHueDeg, outError)) return false;
      if (!ApplyF32(*uiTheme, "accent_saturation", ioPrefs.uiTheme.accentSaturation, outError)) return false;
      if (!ApplyF32(*uiTheme, "accent_value", ioPrefs.uiTheme.accentValue, outError)) return false;
      if (!ApplyF32(*uiTheme, "roundness", ioPrefs.uiTheme.roundness, outError)) return false;
      if (!ApplyI32(*uiTheme, "round_segments", ioPrefs.uiTheme.roundSegments, outError)) return false;
      if (!ApplyF32(*uiTheme, "noise_alpha", ioPrefs.uiTheme.noiseAlpha, outError)) return false;
      if (!ApplyF32(*uiTheme, "noise_scale", ioPrefs.uiTheme.noiseScale, outError)) return false;
      if (!ApplyF32(*uiTheme, "header_sheen", ioPrefs.uiTheme.headerSheenStrength, outError)) return false;
      if (!ApplyI32(*uiTheme, "font_atlas_scale", ioPrefs.uiTheme.fontAtlasScale, outError)) return false;
      if (!ApplyBool(*uiTheme, "font_filter_point", ioPrefs.uiTheme.fontFilterPoint, outError)) return false;
    }
  }

  // world_render
  const JsonValue* wr = nullptr;
  if (GetObj(root, "world_render", &wr)) {
    if (!ApplyBool(*wr, "auto", ioPrefs.worldRenderScaleAuto, outError)) return false;
    if (!ApplyF32(*wr, "scale", ioPrefs.worldRenderScale, outError)) return false;
    if (!ApplyF32(*wr, "min", ioPrefs.worldRenderScaleMin, outError)) return false;
    if (!ApplyF32(*wr, "max", ioPrefs.worldRenderScaleMax, outError)) return false;
    if (!ApplyI32(*wr, "target_fps", ioPrefs.worldRenderTargetFps, outError)) return false;
    if (!ApplyBool(*wr, "filter_point", ioPrefs.worldRenderFilterPoint, outError)) return false;
  }

  // renderer
  const JsonValue* ren = nullptr;
  if (GetObj(root, "renderer", &ren)) {
    if (!ApplyBool(*ren, "merged_zone_buildings", ioPrefs.mergedZoneBuildings, outError)) return false;

    // gfx_theme (palette)
    if (const JsonValue* gt = FindJsonMember(*ren, "gfx_theme")) {
      if (!gt->isString()) {
        outError = "expected string for renderer.gfx_theme";
        return false;
      }
      GfxTheme th;
      if (!ParseGfxTheme(gt->stringValue, th)) {
        outError = "unknown renderer.gfx_theme: " + gt->stringValue;
        return false;
      }
      ioPrefs.gfxTheme = th;
    }
    if (!ApplyBool(*ren, "base_cache", ioPrefs.baseCacheEnabled, outError)) return false;
    if (!ApplyLayersObject(*ren, ioPrefs.layerMask, outError)) return false;

    // elevation
    const JsonValue* el = nullptr;
    if (GetObj(*ren, "elevation", &el)) {
      if (!ApplyF32(*el, "max_pixels", ioPrefs.elevation.maxPixels, outError)) return false;
      if (!ApplyI32(*el, "quantize_steps", ioPrefs.elevation.quantizeSteps, outError)) return false;
      if (!ApplyBool(*el, "flatten_water", ioPrefs.elevation.flattenWater, outError)) return false;
    }

    // shadows
    const JsonValue* sh = nullptr;
    if (GetObj(*ren, "shadows", &sh)) {
      if (!ApplyBool(*sh, "enabled", ioPrefs.shadows.enabled, outError)) return false;
      if (!ApplyF32(*sh, "strength", ioPrefs.shadows.strength, outError)) return false;
      if (!ApplyF32(*sh, "softness", ioPrefs.shadows.softness, outError)) return false;
      if (!ApplyF32(*sh, "max_length_tiles", ioPrefs.shadows.maxLengthTiles, outError)) return false;
      if (!ApplyF32(*sh, "azimuth_deg", ioPrefs.shadows.azimuthDeg, outError)) return false;
      if (!ApplyF32(*sh, "min_altitude_deg", ioPrefs.shadows.minAltitudeDeg, outError)) return false;
      if (!ApplyF32(*sh, "max_altitude_deg", ioPrefs.shadows.maxAltitudeDeg, outError)) return false;
    }

    // day/night
    const JsonValue* dn = nullptr;
    if (GetObj(*ren, "day_night", &dn)) {
      if (!ApplyBool(*dn, "enabled", ioPrefs.dayNight.enabled, outError)) return false;
      if (!ApplyF32(*dn, "day_length_sec", ioPrefs.dayNight.dayLengthSec, outError)) return false;
      if (!ApplyF32(*dn, "time_offset_sec", ioPrefs.dayNight.timeOffsetSec, outError)) return false;
      if (!ApplyF32(*dn, "night_darken", ioPrefs.dayNight.nightDarken, outError)) return false;
      if (!ApplyF32(*dn, "dusk_tint", ioPrefs.dayNight.duskTint, outError)) return false;
      if (!ApplyBool(*dn, "draw_lights", ioPrefs.dayNight.drawLights, outError)) return false;
    }

    // weather
    const JsonValue* we = nullptr;
    if (GetObj(*ren, "weather", &we)) {
      const JsonValue* mode = FindJsonMember(*we, "mode");
      if (mode) {
        Renderer::WeatherSettings::Mode wm = ioPrefs.weather.mode;
        if (!ParseWeatherMode(*mode, wm, outError)) return false;
        ioPrefs.weather.mode = wm;
      }
      if (!ApplyF32(*we, "intensity", ioPrefs.weather.intensity, outError)) return false;
      if (!ApplyF32(*we, "wind_angle_deg", ioPrefs.weather.windAngleDeg, outError)) return false;
      if (!ApplyF32(*we, "wind_speed", ioPrefs.weather.windSpeed, outError)) return false;
      if (!ApplyF32(*we, "overcast", ioPrefs.weather.overcast, outError)) return false;
      if (!ApplyF32(*we, "fog", ioPrefs.weather.fog, outError)) return false;
      if (!ApplyBool(*we, "affect_ground", ioPrefs.weather.affectGround, outError)) return false;
      if (!ApplyBool(*we, "draw_particles", ioPrefs.weather.drawParticles, outError)) return false;
      if (!ApplyBool(*we, "reflect_lights", ioPrefs.weather.reflectLights, outError)) return false;
    }



    // material_fx
    const JsonValue* mfx = nullptr;
    if (GetObj(*ren, "material_fx", &mfx)) {
      if (!ApplyBool(*mfx, "enabled", ioPrefs.materialFx.enabled, outError)) return false;
      if (!ApplyF32(*mfx, "scale", ioPrefs.materialFx.scale, outError)) return false;
      if (!ApplyF32(*mfx, "water_strength", ioPrefs.materialFx.waterStrength, outError)) return false;
      if (!ApplyF32(*mfx, "water_distort_px", ioPrefs.materialFx.waterDistortPx, outError)) return false;
      if (!ApplyF32(*mfx, "water_sparkle", ioPrefs.materialFx.waterSparkle, outError)) return false;
      if (!ApplyF32(*mfx, "foam_strength", ioPrefs.materialFx.foamStrength, outError)) return false;
      if (!ApplyF32(*mfx, "foam_width_px", ioPrefs.materialFx.foamWidthPx, outError)) return false;
      if (!ApplyF32(*mfx, "caustics_strength", ioPrefs.materialFx.causticsStrength, outError)) return false;
      if (!ApplyF32(*mfx, "wet_sand_strength", ioPrefs.materialFx.wetSandStrength, outError)) return false;
      if (!ApplyF32(*mfx, "wet_sand_width_px", ioPrefs.materialFx.wetSandWidthPx, outError)) return false;
      if (!ApplyF32(*mfx, "vegetation_strength", ioPrefs.materialFx.vegetationStrength, outError)) return false;
    }
    // cloud_shadows
    const JsonValue* cs = nullptr;
    if (GetObj(*ren, "cloud_shadows", &cs)) {
      if (!ApplyBool(*cs, "enabled", ioPrefs.cloudShadows.enabled, outError)) return false;
      if (!ApplyF32(*cs, "strength", ioPrefs.cloudShadows.strength, outError)) return false;
      if (!ApplyF32(*cs, "scale", ioPrefs.cloudShadows.scale, outError)) return false;
      if (!ApplyF32(*cs, "speed", ioPrefs.cloudShadows.speed, outError)) return false;
      if (!ApplyF32(*cs, "evolve", ioPrefs.cloudShadows.evolve, outError)) return false;
      if (!ApplyF32(*cs, "coverage", ioPrefs.cloudShadows.coverage, outError)) return false;
      if (!ApplyF32(*cs, "softness", ioPrefs.cloudShadows.softness, outError)) return false;
      if (!ApplyF32(*cs, "clear_amount", ioPrefs.cloudShadows.clearAmount, outError)) return false;
    }

    // volumetric_clouds
    const JsonValue* vc = nullptr;
    if (GetObj(*ren, "volumetric_clouds", &vc)) {
      if (!ApplyBool(*vc, "enabled", ioPrefs.volumetricClouds.enabled, outError)) return false;
      if (!ApplyF32(*vc, "opacity", ioPrefs.volumetricClouds.opacity, outError)) return false;
      if (!ApplyF32(*vc, "coverage", ioPrefs.volumetricClouds.coverage, outError)) return false;
      if (!ApplyF32(*vc, "density", ioPrefs.volumetricClouds.density, outError)) return false;
      if (!ApplyF32(*vc, "scale", ioPrefs.volumetricClouds.scale, outError)) return false;
      if (!ApplyF32(*vc, "speed", ioPrefs.volumetricClouds.speed, outError)) return false;
      if (!ApplyF32(*vc, "softness", ioPrefs.volumetricClouds.softness, outError)) return false;
      if (!ApplyI32(*vc, "steps", ioPrefs.volumetricClouds.steps, outError)) return false;
      if (!ApplyF32(*vc, "bottom_fade", ioPrefs.volumetricClouds.bottomFade, outError)) return false;
      if (!ApplyF32(*vc, "clear_amount", ioPrefs.volumetricClouds.clearAmount, outError)) return false;
    }

    // post_fx
    const JsonValue* pf = nullptr;
    if (GetObj(*ren, "post_fx", &pf)) {
      if (!ApplyBool(*pf, "enabled", ioPrefs.postFx.enabled, outError)) return false;
      if (!ApplyI32(*pf, "color_bits", ioPrefs.postFx.colorBits, outError)) return false;
      if (!ApplyF32(*pf, "dither_strength", ioPrefs.postFx.ditherStrength, outError)) return false;
      if (!ApplyF32(*pf, "grain", ioPrefs.postFx.grain, outError)) return false;
      if (!ApplyF32(*pf, "vignette", ioPrefs.postFx.vignette, outError)) return false;
      if (!ApplyF32(*pf, "chroma", ioPrefs.postFx.chroma, outError)) return false;
      if (!ApplyF32(*pf, "scanlines", ioPrefs.postFx.scanlines, outError)) return false;
      if (!ApplyF32(*pf, "fxaa", ioPrefs.postFx.fxaa, outError)) return false;
      if (!ApplyF32(*pf, "sharpen", ioPrefs.postFx.sharpen, outError)) return false;

      if (!ApplyBool(*pf, "tonemap_enabled", ioPrefs.postFx.tonemapEnabled, outError)) return false;
      if (!ApplyBool(*pf, "tonemapEnabled", ioPrefs.postFx.tonemapEnabled, outError)) return false;
      if (!ApplyF32(*pf, "exposure", ioPrefs.postFx.exposure, outError)) return false;
      if (!ApplyF32(*pf, "contrast", ioPrefs.postFx.contrast, outError)) return false;
      if (!ApplyF32(*pf, "saturation", ioPrefs.postFx.saturation, outError)) return false;

      if (!ApplyF32(*pf, "outline", ioPrefs.postFx.outline, outError)) return false;
      if (!ApplyF32(*pf, "outline_threshold", ioPrefs.postFx.outlineThreshold, outError)) return false;
      if (!ApplyF32(*pf, "outlineThreshold", ioPrefs.postFx.outlineThreshold, outError)) return false;
      if (!ApplyF32(*pf, "outline_thickness", ioPrefs.postFx.outlineThickness, outError)) return false;
      if (!ApplyF32(*pf, "outlineThickness", ioPrefs.postFx.outlineThickness, outError)) return false;
      if (!ApplyBool(*pf, "taa_enabled", ioPrefs.postFx.taaEnabled, outError)) return false;
      if (!ApplyF32(*pf, "taa_history", ioPrefs.postFx.taaHistory, outError)) return false;
      if (!ApplyF32(*pf, "taa_jitter", ioPrefs.postFx.taaJitter, outError)) return false;
      if (!ApplyF32(*pf, "taa_response", ioPrefs.postFx.taaResponse, outError)) return false;
      if (!ApplyBool(*pf, "include_weather", ioPrefs.postFx.includeWeather, outError)) return false;

      // Lens precipitation (optional)
      if (!ApplyF32(*pf, "lens_weather", ioPrefs.postFx.lensWeather, outError)) return false;
      if (!ApplyF32(*pf, "lens_distort", ioPrefs.postFx.lensDistort, outError)) return false;
      if (!ApplyF32(*pf, "lens_scale", ioPrefs.postFx.lensScale, outError)) return false;
      if (!ApplyF32(*pf, "lens_drips", ioPrefs.postFx.lensDrips, outError)) return false;

      // Bloom (optional)
      if (!ApplyF32(*pf, "bloom", ioPrefs.postFx.bloom, outError)) return false;
      if (!ApplyF32(*pf, "bloom_threshold", ioPrefs.postFx.bloomThreshold, outError)) return false;
      if (!ApplyF32(*pf, "bloom_knee", ioPrefs.postFx.bloomKnee, outError)) return false;
      if (!ApplyF32(*pf, "bloom_radius", ioPrefs.postFx.bloomRadius, outError)) return false;
      if (!ApplyI32(*pf, "bloom_downsample", ioPrefs.postFx.bloomDownsample, outError)) return false;
    }
  }

  // Clamp a few common-sense ranges so bad JSON can't completely break the scene.
  ioPrefs.uiScaleManual = std::clamp(ioPrefs.uiScaleManual, 0.5f, 4.0f);

  ioPrefs.worldRenderScale = std::clamp(ioPrefs.worldRenderScale, 0.25f, 2.0f);
  ioPrefs.worldRenderScaleMin = std::clamp(ioPrefs.worldRenderScaleMin, 0.25f, 2.0f);
  ioPrefs.worldRenderScaleMax = std::clamp(ioPrefs.worldRenderScaleMax, 0.25f, 2.0f);
  if (ioPrefs.worldRenderScaleMin > ioPrefs.worldRenderScaleMax) {
    std::swap(ioPrefs.worldRenderScaleMin, ioPrefs.worldRenderScaleMax);
  }
  ioPrefs.worldRenderTargetFps = std::clamp(ioPrefs.worldRenderTargetFps, 15, 240);

  ioPrefs.shadows.strength = std::clamp(ioPrefs.shadows.strength, 0.0f, 1.0f);
  ioPrefs.shadows.softness = std::clamp(ioPrefs.shadows.softness, 0.0f, 1.0f);
  ioPrefs.shadows.maxLengthTiles = std::clamp(ioPrefs.shadows.maxLengthTiles, 0.0f, 20.0f);

  ioPrefs.dayNight.dayLengthSec = std::clamp(ioPrefs.dayNight.dayLengthSec, 30.0f, 1800.0f);
  ioPrefs.dayNight.timeOffsetSec = std::clamp(ioPrefs.dayNight.timeOffsetSec, 0.0f, ioPrefs.dayNight.dayLengthSec);
  ioPrefs.dayNight.nightDarken = std::clamp(ioPrefs.dayNight.nightDarken, 0.0f, 1.0f);
  ioPrefs.dayNight.duskTint = std::clamp(ioPrefs.dayNight.duskTint, 0.0f, 1.0f);

  ioPrefs.weather.intensity = std::clamp(ioPrefs.weather.intensity, 0.0f, 1.0f);
  ioPrefs.weather.windSpeed = std::clamp(ioPrefs.weather.windSpeed, 0.0f, 10.0f);
  ioPrefs.weather.overcast = std::clamp(ioPrefs.weather.overcast, 0.0f, 1.0f);
  ioPrefs.weather.fog = std::clamp(ioPrefs.weather.fog, 0.0f, 1.0f);


  ioPrefs.materialFx.scale = std::clamp(ioPrefs.materialFx.scale, 0.25f, 8.0f);
  ioPrefs.materialFx.waterStrength = std::clamp(ioPrefs.materialFx.waterStrength, 0.0f, 2.0f);
  ioPrefs.materialFx.waterDistortPx = std::clamp(ioPrefs.materialFx.waterDistortPx, 0.0f, 4.0f);
  ioPrefs.materialFx.waterSparkle = std::clamp(ioPrefs.materialFx.waterSparkle, 0.0f, 2.0f);
  ioPrefs.materialFx.foamStrength = std::clamp(ioPrefs.materialFx.foamStrength, 0.0f, 2.0f);
  ioPrefs.materialFx.foamWidthPx = std::clamp(ioPrefs.materialFx.foamWidthPx, 0.0f, 8.0f);
  ioPrefs.materialFx.causticsStrength = std::clamp(ioPrefs.materialFx.causticsStrength, 0.0f, 2.0f);
  ioPrefs.materialFx.wetSandStrength = std::clamp(ioPrefs.materialFx.wetSandStrength, 0.0f, 2.0f);
  ioPrefs.materialFx.wetSandWidthPx = std::clamp(ioPrefs.materialFx.wetSandWidthPx, 0.0f, 8.0f);
  ioPrefs.materialFx.vegetationStrength = std::clamp(ioPrefs.materialFx.vegetationStrength, 0.0f, 2.0f);

  ioPrefs.cloudShadows.strength = std::clamp(ioPrefs.cloudShadows.strength, 0.0f, 1.0f);
  ioPrefs.cloudShadows.scale = std::clamp(ioPrefs.cloudShadows.scale, 0.25f, 8.0f);
  ioPrefs.cloudShadows.speed = std::clamp(ioPrefs.cloudShadows.speed, 0.0f, 5.0f);
  ioPrefs.cloudShadows.evolve = std::clamp(ioPrefs.cloudShadows.evolve, 0.0f, 1.0f);
  ioPrefs.cloudShadows.coverage = std::clamp(ioPrefs.cloudShadows.coverage, 0.0f, 1.0f);
  ioPrefs.cloudShadows.softness = std::clamp(ioPrefs.cloudShadows.softness, 0.0f, 1.0f);
  ioPrefs.cloudShadows.clearAmount = std::clamp(ioPrefs.cloudShadows.clearAmount, 0.0f, 1.0f);

  ioPrefs.volumetricClouds.opacity = std::clamp(ioPrefs.volumetricClouds.opacity, 0.0f, 1.0f);
  ioPrefs.volumetricClouds.coverage = std::clamp(ioPrefs.volumetricClouds.coverage, 0.0f, 1.0f);
  ioPrefs.volumetricClouds.density = std::clamp(ioPrefs.volumetricClouds.density, 0.0f, 2.0f);
  ioPrefs.volumetricClouds.scale = std::clamp(ioPrefs.volumetricClouds.scale, 0.25f, 8.0f);
  ioPrefs.volumetricClouds.speed = std::clamp(ioPrefs.volumetricClouds.speed, 0.0f, 5.0f);
  ioPrefs.volumetricClouds.softness = std::clamp(ioPrefs.volumetricClouds.softness, 0.0f, 1.0f);
  ioPrefs.volumetricClouds.steps = std::clamp(ioPrefs.volumetricClouds.steps, 8, 64);
  ioPrefs.volumetricClouds.bottomFade = std::clamp(ioPrefs.volumetricClouds.bottomFade, 0.0f, 1.0f);
  ioPrefs.volumetricClouds.clearAmount = std::clamp(ioPrefs.volumetricClouds.clearAmount, 0.0f, 1.0f);

  ioPrefs.postFx.colorBits = std::clamp(ioPrefs.postFx.colorBits, 2, 8);
  ioPrefs.postFx.ditherStrength = std::clamp(ioPrefs.postFx.ditherStrength, 0.0f, 1.0f);
  ioPrefs.postFx.grain = std::clamp(ioPrefs.postFx.grain, 0.0f, 1.0f);
  ioPrefs.postFx.vignette = std::clamp(ioPrefs.postFx.vignette, 0.0f, 1.0f);
  ioPrefs.postFx.chroma = std::clamp(ioPrefs.postFx.chroma, 0.0f, 1.0f);
  ioPrefs.postFx.scanlines = std::clamp(ioPrefs.postFx.scanlines, 0.0f, 1.0f);
  ioPrefs.postFx.fxaa = std::clamp(ioPrefs.postFx.fxaa, 0.0f, 1.0f);
  ioPrefs.postFx.sharpen = std::clamp(ioPrefs.postFx.sharpen, 0.0f, 1.0f);

  ioPrefs.postFx.exposure = std::clamp(ioPrefs.postFx.exposure, 0.0f, 4.0f);
  ioPrefs.postFx.contrast = std::clamp(ioPrefs.postFx.contrast, 0.0f, 2.0f);
  ioPrefs.postFx.saturation = std::clamp(ioPrefs.postFx.saturation, 0.0f, 2.0f);

  ioPrefs.postFx.outline = std::clamp(ioPrefs.postFx.outline, 0.0f, 1.0f);
  ioPrefs.postFx.outlineThreshold = std::clamp(ioPrefs.postFx.outlineThreshold, 0.0f, 1.0f);
  ioPrefs.postFx.outlineThickness = std::clamp(ioPrefs.postFx.outlineThickness, 0.5f, 4.0f);
  ioPrefs.postFx.taaHistory = std::clamp(ioPrefs.postFx.taaHistory, 0.0f, 0.98f);
  ioPrefs.postFx.taaJitter = std::clamp(ioPrefs.postFx.taaJitter, 0.0f, 1.0f);
  ioPrefs.postFx.taaResponse = std::clamp(ioPrefs.postFx.taaResponse, 0.0f, 1.0f);

  ioPrefs.postFx.lensWeather = std::clamp(ioPrefs.postFx.lensWeather, 0.0f, 1.0f);
  ioPrefs.postFx.lensDistort = std::clamp(ioPrefs.postFx.lensDistort, 0.0f, 1.0f);
  ioPrefs.postFx.lensScale = std::clamp(ioPrefs.postFx.lensScale, 0.5f, 2.0f);
  ioPrefs.postFx.lensDrips = std::clamp(ioPrefs.postFx.lensDrips, 0.0f, 1.0f);

  ioPrefs.postFx.bloom = std::clamp(ioPrefs.postFx.bloom, 0.0f, 1.0f);
  ioPrefs.postFx.bloomThreshold = std::clamp(ioPrefs.postFx.bloomThreshold, 0.0f, 1.0f);
  ioPrefs.postFx.bloomKnee = std::clamp(ioPrefs.postFx.bloomKnee, 0.0f, 1.0f);
  ioPrefs.postFx.bloomRadius = std::clamp(ioPrefs.postFx.bloomRadius, 0.25f, 4.0f);
  ioPrefs.postFx.bloomDownsample = std::clamp(ioPrefs.postFx.bloomDownsample, 1, 8);

  ioPrefs.elevation.maxPixels = std::clamp(ioPrefs.elevation.maxPixels, 0.0f, 1024.0f);
  ioPrefs.elevation.quantizeSteps = std::clamp(ioPrefs.elevation.quantizeSteps, 0, 128);

  return true;
}

bool LoadVisualPrefsJsonFile(const std::string& path, VisualPrefs& ioPrefs, std::string& outError)
{
  outError.clear();
  if (path.empty()) {
    outError = "empty path";
    return false;
  }

  namespace fs = std::filesystem;
  const fs::path p(path);
  const std::string ps = p.string();
  const bool isTmp = ps.size() >= 4 && ps.compare(ps.size() - 4, 4, ".tmp") == 0;
  const bool isBak = ps.size() >= 4 && ps.compare(ps.size() - 4, 4, ".bak") == 0;

  auto loadExact = [&](const fs::path& file, VisualPrefs& outPrefs, std::string& err) -> bool {
    std::ifstream ifs(file, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
      err = "could not open file";
      return false;
    }

    std::stringstream ss;
    ss << ifs.rdbuf();

    JsonValue root;
    if (!ParseJson(ss.str(), root, err)) {
      return false;
    }

    VisualPrefs tmp = ioPrefs;
    if (!ApplyVisualPrefsJson(root, tmp, err)) {
      return false;
    }
    outPrefs = tmp;
    return true;
  };

  // If the caller explicitly pointed at a transactional artifact, respect it and do not heal.
  if (isTmp || isBak) {
    VisualPrefs loaded;
    if (!loadExact(p, loaded, outError)) return false;
    ioPrefs = loaded;
    return true;
  }

  const fs::path tmpPath = fs::path(ps + ".tmp");
  const fs::path bakPath = fs::path(ps + ".bak");

  // 1) Try the primary file.
  VisualPrefs loaded;
  std::string primaryErr;
  if (loadExact(p, loaded, primaryErr)) {
    ioPrefs = loaded;

    // If there's a stale tmp older than the committed file, clean it up (best-effort).
    std::error_code ec;
    if (fs::exists(tmpPath, ec) && !ec) {
      const auto tTmp = fs::last_write_time(tmpPath, ec);
      if (!ec) {
        const auto tMain = fs::last_write_time(p, ec);
        if (!ec && tTmp < tMain) {
          fs::remove(tmpPath, ec);
        }
      }
    }
    return true;
  }

  // 2) If a temp file exists, try it and (best-effort) promote it into place.
  {
    std::error_code ec;
    if (fs::exists(tmpPath, ec) && !ec) {
      std::string tmpErr;
      if (loadExact(tmpPath, loaded, tmpErr)) {
        ioPrefs = loaded;

        // Heal: rotate current prefs to .bak and move .tmp into place.
        const fs::path dir = p.parent_path().empty() ? fs::path(".") : p.parent_path();
        const bool hadMain = fs::exists(p, ec) && !ec;
        if (hadMain) {
          fs::remove(bakPath, ec);
          ec.clear();
          fs::rename(p, bakPath, ec);
        }
        ec.clear();
        fs::rename(tmpPath, p, ec);
        if (!ec) {
          BestEffortSyncFile(p);
          BestEffortSyncDirectory(dir);
        }
        return true;
      }
    }
  }

  // 3) If a backup exists, try it and (best-effort) restore it.
  {
    std::error_code ec;
    if (fs::exists(bakPath, ec) && !ec) {
      std::string bakErr;
      if (loadExact(bakPath, loaded, bakErr)) {
        ioPrefs = loaded;

        // Preserve the corrupt file (if present) so users can attach it to bug reports.
        if (fs::exists(p, ec) && !ec) {
          std::tm tm{};
          const std::time_t now = std::time(nullptr);
#if defined(_WIN32)
          gmtime_s(&tm, &now);
#else
          gmtime_r(&now, &tm);
#endif
          char buf[32] = {};
          std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02dZ",
                        tm.tm_year + 1900,
                        tm.tm_mon + 1,
                        tm.tm_mday,
                        tm.tm_hour,
                        tm.tm_min,
                        tm.tm_sec);

          const fs::path corrupt = fs::path(ps + ".corrupt_" + std::string(buf));
          fs::rename(p, corrupt, ec);
        }

        ec.clear();
        fs::copy_file(bakPath, p, fs::copy_options::overwrite_existing, ec);
        if (!ec) {
          const fs::path dir = p.parent_path().empty() ? fs::path(".") : p.parent_path();
          BestEffortSyncFile(p);
          BestEffortSyncDirectory(dir);
        }
        return true;
      }
    }
  }

  outError = primaryErr.empty() ? "failed to load prefs" : primaryErr;
  return false;
}

bool WriteVisualPrefsJsonFile(const std::string& path, const VisualPrefs& prefs, std::string& outError, int indentSpaces)
{
  outError.clear();
  if (path.empty()) {
    outError = "empty path";
    return false;
  }

  namespace fs = std::filesystem;
  const fs::path outPath(path);
  const fs::path dir = outPath.parent_path().empty() ? fs::path(".") : outPath.parent_path();
  const fs::path tmpPath = fs::path(outPath.string() + ".tmp");
  const fs::path bakPath = fs::path(outPath.string() + ".bak");

  // Ensure the directory exists (best-effort).
  {
    std::error_code ec;
    fs::create_directories(dir, ec);
  }

  // Write to a temp file first.
  {
    std::ofstream ofs(tmpPath, std::ios::binary | std::ios::trunc);
    if (!ofs.is_open()) {
      outError = "could not open temp file for write";
      return false;
    }

    ofs << VisualPrefsToJson(prefs, indentSpaces);
    ofs.flush();
    if (!ofs.good()) {
      outError = "write failed";
      return false;
    }
  }

  // Best-effort durable write: fsync temp file, atomically rename, then fsync directory.
  BestEffortSyncFile(tmpPath);

  std::error_code ec;
  const bool hadOut = fs::exists(outPath, ec) && !ec;
  if (hadOut) {
    fs::remove(bakPath, ec);
    ec.clear();
    fs::rename(outPath, bakPath, ec);
    if (ec) {
      outError = "failed to rotate prefs backup: " + ec.message();
      return false;
    }
  }

  ec.clear();
  fs::rename(tmpPath, outPath, ec);
  if (ec) {
    // Roll back if we moved the original aside.
    std::error_code ec2;
    if (hadOut && fs::exists(bakPath, ec2) && !ec2) {
      fs::rename(bakPath, outPath, ec2);
    }
    outError = "failed to commit prefs file: " + ec.message();
    return false;
  }

  BestEffortSyncFile(outPath);
  BestEffortSyncDirectory(dir);
  return true;
}

} // namespace isocity
