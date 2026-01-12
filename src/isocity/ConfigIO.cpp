#include "isocity/ConfigIO.hpp"

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

static bool GetArray(const JsonValue& obj, const std::string& key, const JsonValue** out)
{
  const JsonValue* v = FindJsonMember(obj, key);
  if (!v) return false;
  if (!v->isArray()) return false;
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

static void WriteErosionConfig(std::ostringstream& oss, const ErosionConfig& e, int indent, int depth)
{
  const int base = indent * depth;
  oss << "{\n";
  Indent(oss, base + indent);
  oss << "\"enabled\": ";
  WriteBool(oss, e.enabled);
  oss << ",\n";

  Indent(oss, base + indent);
  oss << "\"rivers_enabled\": ";
  WriteBool(oss, e.riversEnabled);
  oss << ",\n";

  Indent(oss, base + indent);
  oss << "\"thermal_iterations\": " << e.thermalIterations << ",\n";
  Indent(oss, base + indent);
  oss << "\"thermal_talus\": " << FloatToJson(e.thermalTalus) << ",\n";
  Indent(oss, base + indent);
  oss << "\"thermal_rate\": " << FloatToJson(e.thermalRate) << ",\n";

  Indent(oss, base + indent);
  oss << "\"river_min_accum\": " << e.riverMinAccum << ",\n";
  Indent(oss, base + indent);
  oss << "\"river_carve\": " << FloatToJson(e.riverCarve) << ",\n";
  Indent(oss, base + indent);
  oss << "\"river_carve_power\": " << FloatToJson(e.riverCarvePower) << ",\n";

  Indent(oss, base + indent);
  oss << "\"smooth_iterations\": " << e.smoothIterations << ",\n";
  Indent(oss, base + indent);
  oss << "\"smooth_rate\": " << FloatToJson(e.smoothRate) << ",\n";

  Indent(oss, base + indent);
  oss << "\"quantize_scale\": " << e.quantizeScale << "\n";

  Indent(oss, base);
  oss << "}";
}

static void WriteDistrictPolicy(std::ostringstream& oss, const DistrictPolicy& p, int indent, int depth,
                                int districtId)
{
  const int base = indent * depth;
  oss << "{\n";
  Indent(oss, base + indent);
  oss << "\"id\": " << districtId << ",\n";

  Indent(oss, base + indent);
  oss << "\"tax_residential_mult\": " << FloatToJson(p.taxResidentialMult) << ",\n";
  Indent(oss, base + indent);
  oss << "\"tax_commercial_mult\": " << FloatToJson(p.taxCommercialMult) << ",\n";
  Indent(oss, base + indent);
  oss << "\"tax_industrial_mult\": " << FloatToJson(p.taxIndustrialMult) << ",\n";

  Indent(oss, base + indent);
  oss << "\"road_maintenance_mult\": " << FloatToJson(p.roadMaintenanceMult) << ",\n";
  Indent(oss, base + indent);
  oss << "\"park_maintenance_mult\": " << FloatToJson(p.parkMaintenanceMult) << "\n";

  Indent(oss, base);
  oss << "}";
}

static void WriteProcGenConfig(std::ostringstream& oss, const ProcGenConfig& cfg, int indent, int depth)
{
  const int base = indent * depth;
  oss << "{\n";

  Indent(oss, base + indent);
  oss << "\"terrain_scale\": " << FloatToJson(cfg.terrainScale) << ",\n";
  Indent(oss, base + indent);
  oss << "\"water_level\": " << FloatToJson(cfg.waterLevel) << ",\n";
  Indent(oss, base + indent);
  oss << "\"sand_level\": " << FloatToJson(cfg.sandLevel) << ",\n";

  Indent(oss, base + indent);
  oss << "\"hubs\": " << cfg.hubs << ",\n";
  Indent(oss, base + indent);
  oss << "\"extra_connections\": " << cfg.extraConnections << ",\n";

  Indent(oss, base + indent);
  oss << "\"road_layout\": \"" << ToString(cfg.roadLayout) << "\",\n";

  Indent(oss, base + indent);
  oss << "\"zone_chance\": " << FloatToJson(cfg.zoneChance) << ",\n";
  Indent(oss, base + indent);
  oss << "\"park_chance\": " << FloatToJson(cfg.parkChance) << ",\n";

  // Macro terrain preset (v10). These are optional; "classic" is the default.
  Indent(oss, base + indent);
  oss << "\"terrain_preset\": \"" << ToString(cfg.terrainPreset) << "\",\n";
  Indent(oss, base + indent);
  oss << "\"terrain_preset_strength\": " << FloatToJson(cfg.terrainPresetStrength) << ",\n";

  Indent(oss, base + indent);
  oss << "\"road_hierarchy_enabled\": ";
  WriteBool(oss, cfg.roadHierarchyEnabled);
  oss << ",\n";

  Indent(oss, base + indent);
  oss << "\"road_hierarchy_strength\": " << FloatToJson(cfg.roadHierarchyStrength) << ",\n";

  Indent(oss, base + indent);
  oss << "\"districting_mode\": \"" << ToString(cfg.districtingMode) << "\",\n";

  Indent(oss, base + indent);
  oss << "\"erosion\": ";
  WriteErosionConfig(oss, cfg.erosion, indent, depth + 1);
  oss << "\n";

  Indent(oss, base);
  oss << "}";
}

static void WriteSimConfig(std::ostringstream& oss, const SimConfig& cfg, int indent, int depth)
{
  const int base = indent * depth;
  oss << "{\n";

  Indent(oss, base + indent);
  oss << "\"tick_seconds\": " << FloatToJson(cfg.tickSeconds) << ",\n";
  Indent(oss, base + indent);
  oss << "\"park_influence_radius\": " << cfg.parkInfluenceRadius << ",\n";
  Indent(oss, base + indent);
  oss << "\"require_outside_connection\": ";
  WriteBool(oss, cfg.requireOutsideConnection);
  oss << ",\n";

  Indent(oss, base + indent);
  oss << "\"tax_residential\": " << cfg.taxResidential << ",\n";
  Indent(oss, base + indent);
  oss << "\"tax_commercial\": " << cfg.taxCommercial << ",\n";
  Indent(oss, base + indent);
  oss << "\"tax_industrial\": " << cfg.taxIndustrial << ",\n";

  Indent(oss, base + indent);
  oss << "\"maintenance_road\": " << cfg.maintenanceRoad << ",\n";
  Indent(oss, base + indent);
  oss << "\"maintenance_park\": " << cfg.maintenancePark << ",\n";

  Indent(oss, base + indent);
  oss << "\"tax_happiness_per_capita\": " << FloatToJson(cfg.taxHappinessPerCapita) << ",\n";

  Indent(oss, base + indent);
  oss << "\"residential_desirability_weight\": " << FloatToJson(cfg.residentialDesirabilityWeight) << ",\n";
  Indent(oss, base + indent);
  oss << "\"commercial_desirability_weight\": " << FloatToJson(cfg.commercialDesirabilityWeight) << ",\n";
  Indent(oss, base + indent);
  oss << "\"industrial_desirability_weight\": " << FloatToJson(cfg.industrialDesirabilityWeight) << ",\n";

  Indent(oss, base + indent);
  oss << "\"district_policies_enabled\": ";
  WriteBool(oss, cfg.districtPoliciesEnabled);
  oss << ",\n";

  Indent(oss, base + indent);
  oss << "\"district_policies\": [\n";
  for (int i = 0; i < kDistrictCount; ++i) {
    Indent(oss, base + indent + indent);
    WriteDistrictPolicy(oss, cfg.districtPolicies[i], indent, depth + 2, i);
    if (i + 1 < kDistrictCount) oss << ",";
    oss << "\n";
  }
  Indent(oss, base + indent);
  oss << "]\n";

  Indent(oss, base);
  oss << "}";
}

static bool ApplyErosionConfigJson(const JsonValue& root, ErosionConfig& io, std::string& err)
{
  if (!root.isObject()) {
    err = "erosion must be an object";
    return false;
  }

  if (!ApplyBool(root, "enabled", io.enabled, err)) return false;
  if (!ApplyBool(root, "rivers_enabled", io.riversEnabled, err)) return false;

  if (!ApplyI32(root, "thermal_iterations", io.thermalIterations, err)) return false;
  if (!ApplyF32(root, "thermal_talus", io.thermalTalus, err)) return false;
  if (!ApplyF32(root, "thermal_rate", io.thermalRate, err)) return false;

  if (!ApplyI32(root, "river_min_accum", io.riverMinAccum, err)) return false;
  if (!ApplyF32(root, "river_carve", io.riverCarve, err)) return false;
  if (!ApplyF32(root, "river_carve_power", io.riverCarvePower, err)) return false;

  if (!ApplyI32(root, "smooth_iterations", io.smoothIterations, err)) return false;
  if (!ApplyF32(root, "smooth_rate", io.smoothRate, err)) return false;

  if (!ApplyI32(root, "quantize_scale", io.quantizeScale, err)) return false;

  return true;
}

static bool ApplyDistrictPolicyJson(const JsonValue& root, DistrictPolicy& io, std::string& err)
{
  if (!root.isObject()) {
    err = "district policy must be an object";
    return false;
  }

  if (!ApplyF32(root, "tax_residential_mult", io.taxResidentialMult, err)) return false;
  if (!ApplyF32(root, "tax_commercial_mult", io.taxCommercialMult, err)) return false;
  if (!ApplyF32(root, "tax_industrial_mult", io.taxIndustrialMult, err)) return false;
  if (!ApplyF32(root, "road_maintenance_mult", io.roadMaintenanceMult, err)) return false;
  if (!ApplyF32(root, "park_maintenance_mult", io.parkMaintenanceMult, err)) return false;

  return true;
}

static bool ReadFileText(const std::string& path, std::string& out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream oss;
  oss << f.rdbuf();
  out = oss.str();
  return true;
}

static bool WriteFileText(const std::string& path, const std::string& text)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << text;
  return static_cast<bool>(f);
}

} // namespace

std::string ProcGenConfigToJson(const ProcGenConfig& cfg, int indentSpaces)
{
  const int indent = std::max(0, indentSpaces);
  std::ostringstream oss;
  WriteProcGenConfig(oss, cfg, indent, 0);
  oss << "\n";
  return oss.str();
}

std::string SimConfigToJson(const SimConfig& cfg, int indentSpaces)
{
  const int indent = std::max(0, indentSpaces);
  std::ostringstream oss;
  WriteSimConfig(oss, cfg, indent, 0);
  oss << "\n";
  return oss.str();
}

bool ApplyProcGenConfigJson(const JsonValue& root, ProcGenConfig& ioCfg, std::string& outError)
{
  if (!root.isObject()) {
    outError = "ProcGenConfig JSON must be an object";
    return false;
  }

  std::string err;

  if (!ApplyF32(root, "terrain_scale", ioCfg.terrainScale, err)) {
    outError = err;
    return false;
  }
  if (!ApplyF32(root, "water_level", ioCfg.waterLevel, err)) {
    outError = err;
    return false;
  }
  if (!ApplyF32(root, "sand_level", ioCfg.sandLevel, err)) {
    outError = err;
    return false;
  }

  if (!ApplyI32(root, "hubs", ioCfg.hubs, err)) {
    outError = err;
    return false;
  }
  if (!ApplyI32(root, "extra_connections", ioCfg.extraConnections, err)) {
    outError = err;
    return false;
  }

  const JsonValue* layout = FindJsonMember(root, "road_layout");
  if (layout) {
    if (!layout->isString()) {
      outError = "expected string for key 'road_layout'";
      return false;
    }
    ProcGenRoadLayout m{};
    if (!ParseProcGenRoadLayout(layout->stringValue, m)) {
      outError = "unknown road_layout: '" + layout->stringValue + "'";
      return false;
    }
    ioCfg.roadLayout = m;
  }

  if (!ApplyF32(root, "zone_chance", ioCfg.zoneChance, err)) {
    outError = err;
    return false;
  }
  if (!ApplyF32(root, "park_chance", ioCfg.parkChance, err)) {
    outError = err;
    return false;
  }

  // Macro preset overrides (v10).
  const JsonValue* preset = FindJsonMember(root, "terrain_preset");
  if (preset) {
    if (!preset->isString()) {
      outError = "expected string for key 'terrain_preset'";
      return false;
    }
    ProcGenTerrainPreset p{};
    if (!ParseProcGenTerrainPreset(preset->stringValue, p)) {
      outError = "unknown terrain_preset: '" + preset->stringValue + "'";
      return false;
    }
    ioCfg.terrainPreset = p;
  }

  if (!ApplyF32(root, "terrain_preset_strength", ioCfg.terrainPresetStrength, err)) {
    outError = err;
    return false;
  }

  // Keep within a sane range (also matches save-file clamps).
  ioCfg.terrainPresetStrength = std::clamp(ioCfg.terrainPresetStrength, 0.0f, 5.0f);

  // Procedural road hierarchy pass (v11).
  if (!ApplyBool(root, "road_hierarchy_enabled", ioCfg.roadHierarchyEnabled, err)) {
    outError = err;
    return false;
  }
  if (!ApplyF32(root, "road_hierarchy_strength", ioCfg.roadHierarchyStrength, err)) {
    outError = err;
    return false;
  }
  ioCfg.roadHierarchyStrength = std::clamp(ioCfg.roadHierarchyStrength, 0.0f, 3.0f);

  // Procedural district assignment mode (v12).
  {
    const JsonValue* districtingMode = FindJsonMember(root, "districting_mode");
    if (districtingMode) {
      if (!districtingMode->isString()) {
        outError = "expected string for key 'districting_mode'";
        return false;
      }
      ProcGenDistrictingMode mode{};
      if (!ParseProcGenDistrictingMode(districtingMode->stringValue, mode)) {
        outError = "unknown districting_mode: '" + districtingMode->stringValue + "'";
        return false;
      }
      ioCfg.districtingMode = mode;
    }
  }

  const JsonValue* erosion = nullptr;
  if (GetObj(root, "erosion", &erosion)) {
    if (!ApplyErosionConfigJson(*erosion, ioCfg.erosion, err)) {
      outError = std::string("erosion: ") + err;
      return false;
    }
  }

  outError.clear();
  return true;
}

bool ApplySimConfigJson(const JsonValue& root, SimConfig& ioCfg, std::string& outError)
{
  if (!root.isObject()) {
    outError = "SimConfig JSON must be an object";
    return false;
  }

  std::string err;

  if (!ApplyF32(root, "tick_seconds", ioCfg.tickSeconds, err)) {
    outError = err;
    return false;
  }

  if (!ApplyI32(root, "park_influence_radius", ioCfg.parkInfluenceRadius, err)) {
    outError = err;
    return false;
  }

  if (!ApplyBool(root, "require_outside_connection", ioCfg.requireOutsideConnection, err)) {
    outError = err;
    return false;
  }

  if (!ApplyI32(root, "tax_residential", ioCfg.taxResidential, err)) {
    outError = err;
    return false;
  }
  if (!ApplyI32(root, "tax_commercial", ioCfg.taxCommercial, err)) {
    outError = err;
    return false;
  }
  if (!ApplyI32(root, "tax_industrial", ioCfg.taxIndustrial, err)) {
    outError = err;
    return false;
  }

  if (!ApplyI32(root, "maintenance_road", ioCfg.maintenanceRoad, err)) {
    outError = err;
    return false;
  }
  if (!ApplyI32(root, "maintenance_park", ioCfg.maintenancePark, err)) {
    outError = err;
    return false;
  }

  if (!ApplyF32(root, "tax_happiness_per_capita", ioCfg.taxHappinessPerCapita, err)) {
    outError = err;
    return false;
  }

  if (!ApplyF32(root, "residential_desirability_weight", ioCfg.residentialDesirabilityWeight, err)) {
    outError = err;
    return false;
  }
  if (!ApplyF32(root, "commercial_desirability_weight", ioCfg.commercialDesirabilityWeight, err)) {
    outError = err;
    return false;
  }
  if (!ApplyF32(root, "industrial_desirability_weight", ioCfg.industrialDesirabilityWeight, err)) {
    outError = err;
    return false;
  }

  if (!ApplyBool(root, "district_policies_enabled", ioCfg.districtPoliciesEnabled, err)) {
    outError = err;
    return false;
  }

  const JsonValue* policies = nullptr;
  if (GetArray(root, "district_policies", &policies)) {
    // Two supported encodings:
    //  1) Full array length=kDistrictCount (index => district id)
    //  2) Sparse array of objects with an explicit "id"
    const bool looksFull = static_cast<int>(policies->arrayValue.size()) == kDistrictCount;

    for (std::size_t i = 0; i < policies->arrayValue.size(); ++i) {
      const JsonValue& v = policies->arrayValue[i];
      if (!v.isObject()) {
        outError = "district_policies elements must be objects";
        return false;
      }

      int districtId = static_cast<int>(i);
      const JsonValue* idVal = FindJsonMember(v, "id");
      if (idVal && idVal->isNumber()) {
        districtId = static_cast<int>(std::lround(idVal->numberValue));
      } else if (!looksFull) {
        // Sparse form requires explicit id.
        outError = "sparse district_policies entries require an 'id'";
        return false;
      }

      if (districtId < 0 || districtId >= kDistrictCount) {
        outError = "district policy id out of range";
        return false;
      }

      if (!ApplyDistrictPolicyJson(v, ioCfg.districtPolicies[districtId], err)) {
        outError = std::string("district_policies[") + std::to_string(districtId) + "]: " + err;
        return false;
      }
    }
  }

  outError.clear();
  return true;
}

bool WriteProcGenConfigJsonFile(const std::string& path, const ProcGenConfig& cfg, std::string& outError,
                                int indentSpaces)
{
  const std::string text = ProcGenConfigToJson(cfg, indentSpaces);
  if (!WriteFileText(path, text)) {
    outError = "failed to write file";
    return false;
  }
  outError.clear();
  return true;
}

bool WriteSimConfigJsonFile(const std::string& path, const SimConfig& cfg, std::string& outError,
                            int indentSpaces)
{
  const std::string text = SimConfigToJson(cfg, indentSpaces);
  if (!WriteFileText(path, text)) {
    outError = "failed to write file";
    return false;
  }
  outError.clear();
  return true;
}

bool LoadProcGenConfigJsonFile(const std::string& path, ProcGenConfig& ioCfg, std::string& outError)
{
  std::string text;
  if (!ReadFileText(path, text)) {
    outError = "failed to read file";
    return false;
  }

  JsonValue root;
  std::string err;
  if (!ParseJson(text, root, err)) {
    outError = err;
    return false;
  }

  if (!ApplyProcGenConfigJson(root, ioCfg, err)) {
    outError = err;
    return false;
  }

  outError.clear();
  return true;
}

bool LoadSimConfigJsonFile(const std::string& path, SimConfig& ioCfg, std::string& outError)
{
  std::string text;
  if (!ReadFileText(path, text)) {
    outError = "failed to read file";
    return false;
  }

  JsonValue root;
  std::string err;
  if (!ParseJson(text, root, err)) {
    outError = err;
    return false;
  }

  if (!ApplySimConfigJson(root, ioCfg, err)) {
    outError = err;
    return false;
  }

  outError.clear();
  return true;
}

std::string CombinedConfigToJson(const ProcGenConfig& proc, const SimConfig& sim, int indentSpaces)
{
  const int indent = std::max(0, indentSpaces);
  std::ostringstream oss;
  oss << "{\n";

  Indent(oss, indent);
  oss << "\"proc\": ";
  WriteProcGenConfig(oss, proc, indent, 1);
  oss << ",\n";

  Indent(oss, indent);
  oss << "\"sim\": ";
  WriteSimConfig(oss, sim, indent, 1);
  oss << "\n";

  oss << "}\n";
  return oss.str();
}

bool LoadCombinedConfigJsonFile(const std::string& path, CombinedConfig& outCfg, std::string& outError)
{
  std::string text;
  if (!ReadFileText(path, text)) {
    outError = "failed to read file";
    return false;
  }

  JsonValue root;
  std::string err;
  if (!ParseJson(text, root, err)) {
    outError = err;
    return false;
  }
  if (!root.isObject()) {
    outError = "combined config JSON must be an object";
    return false;
  }

  outCfg.hasProc = false;
  outCfg.hasSim = false;

  const JsonValue* proc = FindJsonMember(root, "proc");
  if (proc) {
    if (!proc->isObject()) {
      outError = "proc must be an object";
      return false;
    }
    outCfg.hasProc = true;
    outCfg.proc = ProcGenConfig{};
    if (!ApplyProcGenConfigJson(*proc, outCfg.proc, err)) {
      outError = std::string("proc: ") + err;
      return false;
    }
  }

  const JsonValue* sim = FindJsonMember(root, "sim");
  if (sim) {
    if (!sim->isObject()) {
      outError = "sim must be an object";
      return false;
    }
    outCfg.hasSim = true;
    outCfg.sim = SimConfig{};
    if (!ApplySimConfigJson(*sim, outCfg.sim, err)) {
      outError = std::string("sim: ") + err;
      return false;
    }
  }

  outError.clear();
  return true;
}

} // namespace isocity
