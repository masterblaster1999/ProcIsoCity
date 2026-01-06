#include "isocity/VisualPrefs.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

static bool ApplyString(const JsonValue& root, const char* key, std::string& io, std::string& err)
{
  const JsonValue* v = FindJsonMember(root, key);
  if (!v) return true;
  if (!v->isString()) {
    err = std::string("expected string for key '") + key + "'";
    return false;
  }
  io = v->stringValue;
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

  ioPrefs.elevation.maxPixels = std::clamp(ioPrefs.elevation.maxPixels, 0.0f, 1024.0f);
  ioPrefs.elevation.quantizeSteps = std::clamp(ioPrefs.elevation.quantizeSteps, 0, 128);

  return true;
}

bool LoadVisualPrefsJsonFile(const std::string& path, VisualPrefs& ioPrefs, std::string& outError)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    outError = "could not open file";
    return false;
  }

  std::stringstream ss;
  ss << ifs.rdbuf();

  JsonValue root;
  if (!ParseJson(ss.str(), root, outError)) {
    return false;
  }

  return ApplyVisualPrefsJson(root, ioPrefs, outError);
}

bool WriteVisualPrefsJsonFile(const std::string& path, const VisualPrefs& prefs, std::string& outError, int indentSpaces)
{
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs.is_open()) {
    outError = "could not open file for write";
    return false;
  }

  ofs << VisualPrefsToJson(prefs, indentSpaces);
  if (!ofs.good()) {
    outError = "write failed";
    return false;
  }

  return true;
}

} // namespace isocity
