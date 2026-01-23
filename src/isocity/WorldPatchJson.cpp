#include "isocity/WorldPatchJson.hpp"

#include "isocity/ConfigIO.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string_view>

namespace {

using namespace isocity;

// We keep the JSON format version separate from the binary patch version so we
// can evolve them independently.
inline constexpr std::int64_t kWorldPatchJsonVersion = 1;

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

bool ParseHexU64(const std::string& s, std::uint64_t& out)
{
  // Accept 0x prefix.
  std::string_view sv(s);
  while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.front())) != 0) sv.remove_prefix(1);
  while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.back())) != 0) sv.remove_suffix(1);
  if (sv.size() >= 2 && sv[0] == '0' && (sv[1] == 'x' || sv[1] == 'X')) sv.remove_prefix(2);
  if (sv.empty()) return false;

  std::uint64_t value = 0;
  for (char c : sv) {
    int d = -1;
    if (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'a' && c <= 'f') d = 10 + (c - 'a');
    else if (c >= 'A' && c <= 'F') d = 10 + (c - 'A');
    else return false;

    if (value > (std::numeric_limits<std::uint64_t>::max() >> 4)) return false;
    value = (value << 4) | static_cast<std::uint64_t>(d);
  }

  out = value;
  return true;
}

bool ReadFileText(const std::string& path, std::string& outText, std::string& outError)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file";
    return false;
  }
  std::ostringstream oss;
  oss << f.rdbuf();
  outText = oss.str();
  outError.clear();
  return true;
}

bool WriteFileText(const std::string& path, const std::string& text, std::string& outError)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open file for write";
    return false;
  }
  f << text;
  if (!f) {
    outError = "failed to write file";
    return false;
  }
  outError.clear();
  return true;
}

std::uint16_t QuantizeHeight(float h)
{
  // SaveLoad clamps heights to [0,1] before quantization, so we do the same.
  const float clamped = std::clamp(h, 0.0f, 1.0f);
  const float scaled = clamped * 65535.0f;
  const float rounded = std::round(scaled);
  const float r2 = std::clamp(rounded, 0.0f, 65535.0f);
  return static_cast<std::uint16_t>(r2);
}

float DequantizeHeight(std::uint16_t q)
{
  return static_cast<float>(q) / 65535.0f;
}

// --- JSON writing helpers ---

bool WriteErosionConfig(JsonWriter& w, const ErosionConfig& cfg, std::string& outError)
{
  if (!w.beginObject()) {
    outError = w.error();
    return false;
  }

  if (!w.key("enabled") || !w.boolValue(cfg.enabled)) {
    outError = w.error();
    return false;
  }
  if (!w.key("rivers_enabled") || !w.boolValue(cfg.riversEnabled)) {
    outError = w.error();
    return false;
  }

  if (!w.key("thermal_iterations") || !w.intValue(cfg.thermalIterations)) {
    outError = w.error();
    return false;
  }
  if (!w.key("thermal_talus") || !w.numberValue(cfg.thermalTalus)) {
    outError = w.error();
    return false;
  }
  if (!w.key("thermal_rate") || !w.numberValue(cfg.thermalRate)) {
    outError = w.error();
    return false;
  }

  if (!w.key("river_min_accum") || !w.intValue(cfg.riverMinAccum)) {
    outError = w.error();
    return false;
  }
  if (!w.key("river_carve") || !w.numberValue(cfg.riverCarve)) {
    outError = w.error();
    return false;
  }
  if (!w.key("river_carve_power") || !w.numberValue(cfg.riverCarvePower)) {
    outError = w.error();
    return false;
  }

  if (!w.key("smooth_iterations") || !w.intValue(cfg.smoothIterations)) {
    outError = w.error();
    return false;
  }
  if (!w.key("smooth_rate") || !w.numberValue(cfg.smoothRate)) {
    outError = w.error();
    return false;
  }

  if (!w.key("quantize_scale") || !w.intValue(cfg.quantizeScale)) {
    outError = w.error();
    return false;
  }

  if (!w.endObject()) {
    outError = w.error();
    return false;
  }
  return true;
}

bool WriteProcGenConfig(JsonWriter& w, const ProcGenConfig& cfg, std::string& outError)
{
  if (!w.beginObject()) {
    outError = w.error();
    return false;
  }

  if (!w.key("terrain_scale") || !w.numberValue(cfg.terrainScale)) {
    outError = w.error();
    return false;
  }
  if (!w.key("water_level") || !w.numberValue(cfg.waterLevel)) {
    outError = w.error();
    return false;
  }
  if (!w.key("sand_level") || !w.numberValue(cfg.sandLevel)) {
    outError = w.error();
    return false;
  }

  if (!w.key("hubs") || !w.intValue(cfg.hubs)) {
    outError = w.error();
    return false;
  }
  if (!w.key("extra_connections") || !w.intValue(cfg.extraConnections)) {
    outError = w.error();
    return false;
  }

  if (!w.key("road_layout") || !w.stringValue(ToString(cfg.roadLayout))) {
    outError = w.error();
    return false;
  }

  if (!w.key("zone_chance") || !w.numberValue(cfg.zoneChance)) {
    outError = w.error();
    return false;
  }
  if (!w.key("park_chance") || !w.numberValue(cfg.parkChance)) {
    outError = w.error();
    return false;
  }

  if (!w.key("terrain_preset") || !w.stringValue(ToString(cfg.terrainPreset))) {
    outError = w.error();
    return false;
  }
  if (!w.key("terrain_preset_strength") || !w.numberValue(cfg.terrainPresetStrength)) {
    outError = w.error();
    return false;
  }

  if (!w.key("road_hierarchy_enabled") || !w.boolValue(cfg.roadHierarchyEnabled)) {
    outError = w.error();
    return false;
  }
  if (!w.key("road_hierarchy_strength") || !w.numberValue(cfg.roadHierarchyStrength)) {
    outError = w.error();
    return false;
  }

  if (!w.key("districting_mode") || !w.stringValue(ToString(cfg.districtingMode))) {
    outError = w.error();
    return false;
  }

  if (!w.key("erosion")) {
    outError = w.error();
    return false;
  }
  if (!WriteErosionConfig(w, cfg.erosion, outError)) return false;

  if (!w.endObject()) {
    outError = w.error();
    return false;
  }
  return true;
}

bool WriteDistrictPolicies(JsonWriter& w, const SimConfig& cfg, std::string& outError)
{
  if (!w.beginArray()) {
    outError = w.error();
    return false;
  }

  for (int i = 0; i < kDistrictCount; ++i) {
    const DistrictPolicy& p = cfg.districtPolicies[i];

    if (!w.beginObject()) {
      outError = w.error();
      return false;
    }

    if (!w.key("id") || !w.intValue(i)) {
      outError = w.error();
      return false;
    }

    if (!w.key("tax_residential_mult") || !w.numberValue(p.taxResidentialMult)) {
      outError = w.error();
      return false;
    }
    if (!w.key("tax_commercial_mult") || !w.numberValue(p.taxCommercialMult)) {
      outError = w.error();
      return false;
    }
    if (!w.key("tax_industrial_mult") || !w.numberValue(p.taxIndustrialMult)) {
      outError = w.error();
      return false;
    }
    if (!w.key("road_maintenance_mult") || !w.numberValue(p.roadMaintenanceMult)) {
      outError = w.error();
      return false;
    }
    if (!w.key("park_maintenance_mult") || !w.numberValue(p.parkMaintenanceMult)) {
      outError = w.error();
      return false;
    }

    if (!w.endObject()) {
      outError = w.error();
      return false;
    }
  }

  if (!w.endArray()) {
    outError = w.error();
    return false;
  }
  return true;
}

bool WriteSimConfig(JsonWriter& w, const SimConfig& cfg, std::string& outError)
{
  if (!w.beginObject()) {
    outError = w.error();
    return false;
  }

  if (!w.key("tick_seconds") || !w.numberValue(cfg.tickSeconds)) {
    outError = w.error();
    return false;
  }
  if (!w.key("park_influence_radius") || !w.intValue(cfg.parkInfluenceRadius)) {
    outError = w.error();
    return false;
  }
  if (!w.key("require_outside_connection") || !w.boolValue(cfg.requireOutsideConnection)) {
    outError = w.error();
    return false;
  }

  if (!w.key("tax_residential") || !w.intValue(cfg.taxResidential)) {
    outError = w.error();
    return false;
  }
  if (!w.key("tax_commercial") || !w.intValue(cfg.taxCommercial)) {
    outError = w.error();
    return false;
  }
  if (!w.key("tax_industrial") || !w.intValue(cfg.taxIndustrial)) {
    outError = w.error();
    return false;
  }

  if (!w.key("maintenance_road") || !w.intValue(cfg.maintenanceRoad)) {
    outError = w.error();
    return false;
  }
  if (!w.key("maintenance_park") || !w.intValue(cfg.maintenancePark)) {
    outError = w.error();
    return false;
  }

  if (!w.key("tax_happiness_per_capita") || !w.numberValue(cfg.taxHappinessPerCapita)) {
    outError = w.error();
    return false;
  }

  if (!w.key("residential_desirability_weight") || !w.numberValue(cfg.residentialDesirabilityWeight)) {
    outError = w.error();
    return false;
  }
  if (!w.key("commercial_desirability_weight") || !w.numberValue(cfg.commercialDesirabilityWeight)) {
    outError = w.error();
    return false;
  }
  if (!w.key("industrial_desirability_weight") || !w.numberValue(cfg.industrialDesirabilityWeight)) {
    outError = w.error();
    return false;
  }

  if (!w.key("district_policies_enabled") || !w.boolValue(cfg.districtPoliciesEnabled)) {
    outError = w.error();
    return false;
  }

  if (!w.key("district_policies")) {
    outError = w.error();
    return false;
  }
  if (!WriteDistrictPolicies(w, cfg, outError)) return false;

  if (!w.endObject()) {
    outError = w.error();
    return false;
  }
  return true;
}

bool WriteStats(JsonWriter& w, const Stats& s, std::string& outError)
{
  if (!w.beginObject()) {
    outError = w.error();
    return false;
  }

  // Base
  if (!w.key("day") || !w.intValue(s.day)) { outError = w.error(); return false; }
  if (!w.key("population") || !w.intValue(s.population)) { outError = w.error(); return false; }
  if (!w.key("housing_capacity") || !w.intValue(s.housingCapacity)) { outError = w.error(); return false; }
  if (!w.key("jobs_capacity") || !w.intValue(s.jobsCapacity)) { outError = w.error(); return false; }
  if (!w.key("jobs_capacity_accessible") || !w.intValue(s.jobsCapacityAccessible)) { outError = w.error(); return false; }
  if (!w.key("employed") || !w.intValue(s.employed)) { outError = w.error(); return false; }
  if (!w.key("happiness") || !w.numberValue(s.happiness)) { outError = w.error(); return false; }
  if (!w.key("money") || !w.intValue(s.money)) { outError = w.error(); return false; }
  if (!w.key("roads") || !w.intValue(s.roads)) { outError = w.error(); return false; }
  if (!w.key("parks") || !w.intValue(s.parks)) { outError = w.error(); return false; }

  // Traffic / commute
  if (!w.key("commuters") || !w.intValue(s.commuters)) { outError = w.error(); return false; }
  if (!w.key("commuters_unreachable") || !w.intValue(s.commutersUnreachable)) { outError = w.error(); return false; }
  if (!w.key("avg_commute") || !w.numberValue(s.avgCommute)) { outError = w.error(); return false; }
  if (!w.key("p95_commute") || !w.numberValue(s.p95Commute)) { outError = w.error(); return false; }
  if (!w.key("avg_commute_time") || !w.numberValue(s.avgCommuteTime)) { outError = w.error(); return false; }
  if (!w.key("p95_commute_time") || !w.numberValue(s.p95CommuteTime)) { outError = w.error(); return false; }
  if (!w.key("traffic_congestion") || !w.numberValue(s.trafficCongestion)) { outError = w.error(); return false; }
  if (!w.key("congested_road_tiles") || !w.intValue(s.congestedRoadTiles)) { outError = w.error(); return false; }
  if (!w.key("max_road_traffic") || !w.intValue(s.maxRoadTraffic)) { outError = w.error(); return false; }

  // Transit
  if (!w.key("transit_lines") || !w.intValue(s.transitLines)) { outError = w.error(); return false; }
  if (!w.key("transit_stops") || !w.intValue(s.transitStops)) { outError = w.error(); return false; }
  if (!w.key("transit_riders") || !w.intValue(s.transitRiders)) { outError = w.error(); return false; }
  if (!w.key("transit_mode_share") || !w.numberValue(s.transitModeShare)) { outError = w.error(); return false; }
  if (!w.key("transit_commute_coverage") || !w.numberValue(s.transitCommuteCoverage)) { outError = w.error(); return false; }

  // Goods
  if (!w.key("goods_produced") || !w.intValue(s.goodsProduced)) { outError = w.error(); return false; }
  if (!w.key("goods_demand") || !w.intValue(s.goodsDemand)) { outError = w.error(); return false; }
  if (!w.key("goods_delivered") || !w.intValue(s.goodsDelivered)) { outError = w.error(); return false; }
  if (!w.key("goods_imported") || !w.intValue(s.goodsImported)) { outError = w.error(); return false; }
  if (!w.key("goods_exported") || !w.intValue(s.goodsExported)) { outError = w.error(); return false; }
  if (!w.key("goods_unreachable_demand") || !w.intValue(s.goodsUnreachableDemand)) { outError = w.error(); return false; }
  if (!w.key("goods_satisfaction") || !w.numberValue(s.goodsSatisfaction)) { outError = w.error(); return false; }
  if (!w.key("max_road_goods_traffic") || !w.intValue(s.maxRoadGoodsTraffic)) { outError = w.error(); return false; }

  // Trade
  if (!w.key("trade_import_partner") || !w.intValue(s.tradeImportPartner)) { outError = w.error(); return false; }
  if (!w.key("trade_export_partner") || !w.intValue(s.tradeExportPartner)) { outError = w.error(); return false; }
  if (!w.key("trade_import_capacity_pct") || !w.intValue(s.tradeImportCapacityPct)) { outError = w.error(); return false; }
  if (!w.key("trade_export_capacity_pct") || !w.intValue(s.tradeExportCapacityPct)) { outError = w.error(); return false; }
  if (!w.key("trade_import_disrupted") || !w.boolValue(s.tradeImportDisrupted)) { outError = w.error(); return false; }
  if (!w.key("trade_export_disrupted") || !w.boolValue(s.tradeExportDisrupted)) { outError = w.error(); return false; }
  if (!w.key("trade_market_index") || !w.numberValue(s.tradeMarketIndex)) { outError = w.error(); return false; }

  // Economy snapshot
  if (!w.key("income") || !w.intValue(s.income)) { outError = w.error(); return false; }
  if (!w.key("expenses") || !w.intValue(s.expenses)) { outError = w.error(); return false; }
  if (!w.key("tax_revenue") || !w.intValue(s.taxRevenue)) { outError = w.error(); return false; }
  if (!w.key("maintenance_cost") || !w.intValue(s.maintenanceCost)) { outError = w.error(); return false; }
  if (!w.key("upgrade_cost") || !w.intValue(s.upgradeCost)) { outError = w.error(); return false; }
  if (!w.key("import_cost") || !w.intValue(s.importCost)) { outError = w.error(); return false; }
  if (!w.key("export_revenue") || !w.intValue(s.exportRevenue)) { outError = w.error(); return false; }
  if (!w.key("avg_tax_per_capita") || !w.numberValue(s.avgTaxPerCapita)) { outError = w.error(); return false; }
  if (!w.key("transit_cost") || !w.intValue(s.transitCost)) { outError = w.error(); return false; }

  // Demand / valuation
  if (!w.key("demand_residential") || !w.numberValue(s.demandResidential)) { outError = w.error(); return false; }
  if (!w.key("demand_commercial") || !w.numberValue(s.demandCommercial)) { outError = w.error(); return false; }
  if (!w.key("demand_industrial") || !w.numberValue(s.demandIndustrial)) { outError = w.error(); return false; }
  if (!w.key("avg_land_value") || !w.numberValue(s.avgLandValue)) { outError = w.error(); return false; }

  if (!w.endObject()) {
    outError = w.error();
    return false;
  }
  return true;
}

// --- JSON parsing helpers ---

bool GetRequiredBool(const JsonValue& obj, const char* key, bool& out, std::string& err)
{
  const JsonValue* v = FindJsonMember(obj, key);
  if (!v) {
    err = std::string("missing key '") + key + "'";
    return false;
  }
  if (!v->isBool()) {
    err = std::string("expected bool for key '") + key + "'";
    return false;
  }
  out = v->boolValue;
  return true;
}

bool GetRequiredNumber(const JsonValue& obj, const char* key, double& out, std::string& err)
{
  const JsonValue* v = FindJsonMember(obj, key);
  if (!v) {
    err = std::string("missing key '") + key + "'";
    return false;
  }
  if (!v->isNumber()) {
    err = std::string("expected number for key '") + key + "'";
    return false;
  }
  if (std::isfinite(v->numberValue) == 0) {
    err = std::string("non-finite number for key '") + key + "'";
    return false;
  }
  out = v->numberValue;
  return true;
}

bool GetRequiredI32(const JsonValue& obj, const char* key, int& out, std::string& err)
{
  double n = 0.0;
  if (!GetRequiredNumber(obj, key, n, err)) return false;
  const double r = std::lround(n);
  if (r < static_cast<double>(std::numeric_limits<int>::min()) ||
      r > static_cast<double>(std::numeric_limits<int>::max())) {
    err = std::string("integer out of range for key '") + key + "'";
    return false;
  }
  out = static_cast<int>(r);
  return true;
}

[[maybe_unused]] bool GetRequiredU32(const JsonValue& obj, const char* key, std::uint32_t& out, std::string& err)
{
  double n = 0.0;
  if (!GetRequiredNumber(obj, key, n, err)) return false;
  const double r = std::lround(n);
  if (r < 0.0 || r > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    err = std::string("unsigned out of range for key '") + key + "'";
    return false;
  }
  out = static_cast<std::uint32_t>(r);
  return true;
}

bool GetRequiredU16ArrayElem(const JsonValue& arrElem, std::uint16_t& out, std::string& err)
{
  if (!arrElem.isNumber() || std::isfinite(arrElem.numberValue) == 0) {
    err = "expected number";
    return false;
  }
  const double r = std::lround(arrElem.numberValue);
  if (r < 0.0 || r > 65535.0) {
    err = "value out of range";
    return false;
  }
  out = static_cast<std::uint16_t>(r);
  return true;
}

bool GetRequiredU8ArrayElem(const JsonValue& arrElem, std::uint8_t& out, std::string& err)
{
  if (!arrElem.isNumber() || std::isfinite(arrElem.numberValue) == 0) {
    err = "expected number";
    return false;
  }
  const double r = std::lround(arrElem.numberValue);
  if (r < 0.0 || r > 255.0) {
    err = "value out of range";
    return false;
  }
  out = static_cast<std::uint8_t>(r);
  return true;
}

bool ReadStats(const JsonValue& obj, Stats& outStats, std::string& outError)
{
  if (!obj.isObject()) {
    outError = "stats must be an object";
    return false;
  }

  Stats s{};
  std::string err;

  if (!GetRequiredI32(obj, "day", s.day, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "population", s.population, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "housing_capacity", s.housingCapacity, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "jobs_capacity", s.jobsCapacity, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "jobs_capacity_accessible", s.jobsCapacityAccessible, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "employed", s.employed, err)) { outError = err; return false; }
  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "happiness", n, err)) { outError = err; return false; }
    s.happiness = static_cast<float>(n);
  }
  if (!GetRequiredI32(obj, "money", s.money, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "roads", s.roads, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "parks", s.parks, err)) { outError = err; return false; }

  if (!GetRequiredI32(obj, "commuters", s.commuters, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "commuters_unreachable", s.commutersUnreachable, err)) { outError = err; return false; }
  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "avg_commute", n, err)) { outError = err; return false; }
    s.avgCommute = static_cast<float>(n);
    if (!GetRequiredNumber(obj, "p95_commute", n, err)) { outError = err; return false; }
    s.p95Commute = static_cast<float>(n);
    if (!GetRequiredNumber(obj, "avg_commute_time", n, err)) { outError = err; return false; }
    s.avgCommuteTime = static_cast<float>(n);
    if (!GetRequiredNumber(obj, "p95_commute_time", n, err)) { outError = err; return false; }
    s.p95CommuteTime = static_cast<float>(n);
    if (!GetRequiredNumber(obj, "traffic_congestion", n, err)) { outError = err; return false; }
    s.trafficCongestion = static_cast<float>(n);
  }
  if (!GetRequiredI32(obj, "congested_road_tiles", s.congestedRoadTiles, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "max_road_traffic", s.maxRoadTraffic, err)) { outError = err; return false; }

  if (!GetRequiredI32(obj, "transit_lines", s.transitLines, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "transit_stops", s.transitStops, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "transit_riders", s.transitRiders, err)) { outError = err; return false; }
  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "transit_mode_share", n, err)) { outError = err; return false; }
    s.transitModeShare = static_cast<float>(n);
    if (!GetRequiredNumber(obj, "transit_commute_coverage", n, err)) { outError = err; return false; }
    s.transitCommuteCoverage = static_cast<float>(n);
  }

  if (!GetRequiredI32(obj, "goods_produced", s.goodsProduced, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "goods_demand", s.goodsDemand, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "goods_delivered", s.goodsDelivered, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "goods_imported", s.goodsImported, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "goods_exported", s.goodsExported, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "goods_unreachable_demand", s.goodsUnreachableDemand, err)) { outError = err; return false; }
  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "goods_satisfaction", n, err)) { outError = err; return false; }
    s.goodsSatisfaction = static_cast<float>(n);
  }
  if (!GetRequiredI32(obj, "max_road_goods_traffic", s.maxRoadGoodsTraffic, err)) { outError = err; return false; }

  if (!GetRequiredI32(obj, "trade_import_partner", s.tradeImportPartner, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "trade_export_partner", s.tradeExportPartner, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "trade_import_capacity_pct", s.tradeImportCapacityPct, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "trade_export_capacity_pct", s.tradeExportCapacityPct, err)) { outError = err; return false; }
  if (!GetRequiredBool(obj, "trade_import_disrupted", s.tradeImportDisrupted, err)) { outError = err; return false; }
  if (!GetRequiredBool(obj, "trade_export_disrupted", s.tradeExportDisrupted, err)) { outError = err; return false; }
  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "trade_market_index", n, err)) { outError = err; return false; }
    s.tradeMarketIndex = static_cast<float>(n);
  }

  if (!GetRequiredI32(obj, "income", s.income, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "expenses", s.expenses, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "tax_revenue", s.taxRevenue, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "maintenance_cost", s.maintenanceCost, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "upgrade_cost", s.upgradeCost, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "import_cost", s.importCost, err)) { outError = err; return false; }
  if (!GetRequiredI32(obj, "export_revenue", s.exportRevenue, err)) { outError = err; return false; }
  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "avg_tax_per_capita", n, err)) { outError = err; return false; }
    s.avgTaxPerCapita = static_cast<float>(n);
  }
  if (!GetRequiredI32(obj, "transit_cost", s.transitCost, err)) { outError = err; return false; }

  {
    double n = 0.0;
    if (!GetRequiredNumber(obj, "demand_residential", n, err)) { outError = err; return false; }
    s.demandResidential = static_cast<float>(n);

    // Optional newer fields (back-compat for old JSON patches).
    const JsonValue* dc = FindJsonMember(obj, "demand_commercial");
    if (dc && dc->isNumber()) {
      s.demandCommercial = static_cast<float>(dc->numberValue);
    } else {
      s.demandCommercial = 0.0f;
    }
    const JsonValue* di = FindJsonMember(obj, "demand_industrial");
    if (di && di->isNumber()) {
      s.demandIndustrial = static_cast<float>(di->numberValue);
    } else {
      s.demandIndustrial = 0.0f;
    }

    if (!GetRequiredNumber(obj, "avg_land_value", n, err)) { outError = err; return false; }
    s.avgLandValue = static_cast<float>(n);
  }

  outStats = s;
  outError.clear();
  return true;
}

bool DeserializePatchJson(const JsonValue& root, WorldPatch& outPatch, std::string& outError)
{
  if (!root.isObject()) {
    outError = "WorldPatch JSON must be an object";
    return false;
  }

  // Format guard (optional but recommended).
  const JsonValue* fmt = FindJsonMember(root, "format");
  if (fmt && fmt->isString()) {
    if (fmt->stringValue != "isocity_world_patch") {
      outError = "unknown patch format: '" + fmt->stringValue + "'";
      return false;
    }
  }

  int width = 0;
  int height = 0;
  std::string err;
  if (!GetRequiredI32(root, "width", width, err)) {
    outError = err;
    return false;
  }
  if (!GetRequiredI32(root, "height", height, err)) {
    outError = err;
    return false;
  }
  if (width <= 0 || height <= 0) {
    outError = "width/height must be positive";
    return false;
  }

  // Version.
  {
    double ver = 0.0;
    if (!GetRequiredNumber(root, "format_version", ver, err)) {
      outError = err;
      return false;
    }
    const int v = static_cast<int>(std::lround(ver));
    if (v != static_cast<int>(kWorldPatchJsonVersion)) {
      outError = "unsupported patch JSON format_version";
      return false;
    }
  }

  // Hashes stored as strings to avoid precision loss.
  const JsonValue* baseHashVal = FindJsonMember(root, "base_hash");
  const JsonValue* targetHashVal = FindJsonMember(root, "target_hash");
  if (!baseHashVal || !baseHashVal->isString()) {
    outError = "base_hash must be a string";
    return false;
  }
  if (!targetHashVal || !targetHashVal->isString()) {
    outError = "target_hash must be a string";
    return false;
  }

  std::uint64_t baseHash = 0;
  std::uint64_t targetHash = 0;
  if (!ParseHexU64(baseHashVal->stringValue, baseHash)) {
    outError = "failed to parse base_hash";
    return false;
  }
  if (!ParseHexU64(targetHashVal->stringValue, targetHash)) {
    outError = "failed to parse target_hash";
    return false;
  }

  bool includeProc = true;
  bool includeSim = true;
  bool includeStats = true;
  if (!GetRequiredBool(root, "include_proc_cfg", includeProc, err)) {
    outError = err;
    return false;
  }
  if (!GetRequiredBool(root, "include_sim_cfg", includeSim, err)) {
    outError = err;
    return false;
  }
  if (!GetRequiredBool(root, "include_stats", includeStats, err)) {
    outError = err;
    return false;
  }

  WorldPatch patch;
  patch.width = width;
  patch.height = height;
  patch.baseHash = baseHash;
  patch.targetHash = targetHash;
  patch.includeProcCfg = includeProc;
  patch.includeSimCfg = includeSim;
  patch.includeStats = includeStats;
  patch.version = 0;

  // Optional metadata.
  if (includeProc) {
    const JsonValue* procCfg = FindJsonMember(root, "proc_cfg");
    if (!procCfg) {
      outError = "include_proc_cfg=true but proc_cfg is missing";
      return false;
    }
    patch.procCfg = ProcGenConfig{};
    if (!ApplyProcGenConfigJson(*procCfg, patch.procCfg, err)) {
      outError = std::string("proc_cfg: ") + err;
      return false;
    }
  }
  if (includeSim) {
    const JsonValue* simCfg = FindJsonMember(root, "sim_cfg");
    if (!simCfg) {
      outError = "include_sim_cfg=true but sim_cfg is missing";
      return false;
    }
    patch.simCfg = SimConfig{};
    if (!ApplySimConfigJson(*simCfg, patch.simCfg, err)) {
      outError = std::string("sim_cfg: ") + err;
      return false;
    }
  }
  if (includeStats) {
    const JsonValue* stats = FindJsonMember(root, "stats");
    if (!stats) {
      outError = "include_stats=true but stats is missing";
      return false;
    }
    if (!ReadStats(*stats, patch.stats, err)) {
      outError = std::string("stats: ") + err;
      return false;
    }
  }

  // Tile deltas.
  const JsonValue* tiles = FindJsonMember(root, "tiles");
  if (!tiles || !tiles->isArray()) {
    outError = "tiles must be an array";
    return false;
  }

  patch.tiles.clear();
  patch.tiles.reserve(tiles->arrayValue.size());

  for (std::size_t i = 0; i < tiles->arrayValue.size(); ++i) {
    const JsonValue& e = tiles->arrayValue[i];
    if (!e.isArray() || e.arrayValue.size() != 9) {
      outError = "tiles[" + std::to_string(i) + "] must be an array of length 9";
      return false;
    }

    const JsonValue& idxV = e.arrayValue[0];
    const JsonValue& maskV = e.arrayValue[1];
    const JsonValue& terrainV = e.arrayValue[2];
    const JsonValue& overlayV = e.arrayValue[3];
    const JsonValue& heightQV = e.arrayValue[4];
    const JsonValue& varV = e.arrayValue[5];
    const JsonValue& lvlV = e.arrayValue[6];
    const JsonValue& occV = e.arrayValue[7];
    const JsonValue& distV = e.arrayValue[8];

    if (!idxV.isNumber() || std::isfinite(idxV.numberValue) == 0) {
      outError = "tiles[" + std::to_string(i) + "][0] (index) must be a number";
      return false;
    }
    const double idxR = std::lround(idxV.numberValue);
    if (idxR < 0.0 || idxR > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "tiles[" + std::to_string(i) + "][0] index out of range";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idxR);

    std::uint8_t mask = 0;
    if (!GetRequiredU8ArrayElem(maskV, mask, err)) {
      outError = "tiles[" + std::to_string(i) + "][1] (mask): " + err;
      return false;
    }

    std::uint8_t terrainU8 = 0;
    if (!GetRequiredU8ArrayElem(terrainV, terrainU8, err)) {
      outError = "tiles[" + std::to_string(i) + "][2] (terrain): " + err;
      return false;
    }
    std::uint8_t overlayU8 = 0;
    if (!GetRequiredU8ArrayElem(overlayV, overlayU8, err)) {
      outError = "tiles[" + std::to_string(i) + "][3] (overlay): " + err;
      return false;
    }

    std::uint16_t heightQ = 0;
    if (!GetRequiredU16ArrayElem(heightQV, heightQ, err)) {
      outError = "tiles[" + std::to_string(i) + "][4] (height_q): " + err;
      return false;
    }

    std::uint8_t variation = 0;
    if (!GetRequiredU8ArrayElem(varV, variation, err)) {
      outError = "tiles[" + std::to_string(i) + "][5] (variation): " + err;
      return false;
    }

    std::uint8_t level = 0;
    if (!GetRequiredU8ArrayElem(lvlV, level, err)) {
      outError = "tiles[" + std::to_string(i) + "][6] (level): " + err;
      return false;
    }

    std::uint16_t occupants = 0;
    if (!GetRequiredU16ArrayElem(occV, occupants, err)) {
      outError = "tiles[" + std::to_string(i) + "][7] (occupants): " + err;
      return false;
    }

    std::uint8_t district = 0;
    if (!GetRequiredU8ArrayElem(distV, district, err)) {
      outError = "tiles[" + std::to_string(i) + "][8] (district): " + err;
      return false;
    }

    WorldPatchTileDelta d;
    d.index = idx;
    d.mask = mask;
    d.value.terrain = static_cast<Terrain>(terrainU8);
    d.value.overlay = static_cast<Overlay>(overlayU8);
    d.value.height = DequantizeHeight(heightQ);
    d.value.variation = variation;
    d.value.level = level;
    d.value.occupants = occupants;
    d.value.district = district;
    patch.tiles.push_back(d);
  }

  outPatch = std::move(patch);
  outError.clear();
  return true;
}

bool SerializePatchJson(const WorldPatch& patch, std::string& outJson, std::string& outError,
                        const JsonWriteOptions& opt)
{
  std::ostringstream oss;
  JsonWriter w(oss, opt);

  if (!w.beginObject()) {
    outError = w.error();
    return false;
  }

  if (!w.key("format") || !w.stringValue("isocity_world_patch")) { outError = w.error(); return false; }
  if (!w.key("format_version") || !w.intValue(kWorldPatchJsonVersion)) { outError = w.error(); return false; }
  if (!w.key("width") || !w.intValue(patch.width)) { outError = w.error(); return false; }
  if (!w.key("height") || !w.intValue(patch.height)) { outError = w.error(); return false; }
  if (!w.key("base_hash") || !w.stringValue(HexU64(patch.baseHash))) { outError = w.error(); return false; }
  if (!w.key("target_hash") || !w.stringValue(HexU64(patch.targetHash))) { outError = w.error(); return false; }

  if (!w.key("include_proc_cfg") || !w.boolValue(patch.includeProcCfg)) { outError = w.error(); return false; }
  if (!w.key("include_sim_cfg") || !w.boolValue(patch.includeSimCfg)) { outError = w.error(); return false; }
  if (!w.key("include_stats") || !w.boolValue(patch.includeStats)) { outError = w.error(); return false; }

  if (patch.includeProcCfg) {
    if (!w.key("proc_cfg")) { outError = w.error(); return false; }
    if (!WriteProcGenConfig(w, patch.procCfg, outError)) return false;
  }
  if (patch.includeSimCfg) {
    if (!w.key("sim_cfg")) { outError = w.error(); return false; }
    if (!WriteSimConfig(w, patch.simCfg, outError)) return false;
  }
  if (patch.includeStats) {
    if (!w.key("stats")) { outError = w.error(); return false; }
    if (!WriteStats(w, patch.stats, outError)) return false;
  }

  if (!w.key("tiles") || !w.beginArray()) { outError = w.error(); return false; }

  for (const WorldPatchTileDelta& d : patch.tiles) {
    if (!w.beginArray()) { outError = w.error(); return false; }

    // [index, mask, terrain, overlay, height_q, variation, level, occupants, district]
    if (!w.uintValue(d.index)) { outError = w.error(); return false; }
    if (!w.uintValue(d.mask)) { outError = w.error(); return false; }
    if (!w.uintValue(static_cast<std::uint8_t>(d.value.terrain))) { outError = w.error(); return false; }
    if (!w.uintValue(static_cast<std::uint8_t>(d.value.overlay))) { outError = w.error(); return false; }
    if (!w.uintValue(QuantizeHeight(d.value.height))) { outError = w.error(); return false; }
    if (!w.uintValue(d.value.variation)) { outError = w.error(); return false; }
    if (!w.uintValue(d.value.level)) { outError = w.error(); return false; }
    if (!w.uintValue(d.value.occupants)) { outError = w.error(); return false; }
    if (!w.uintValue(d.value.district)) { outError = w.error(); return false; }

    if (!w.endArray()) { outError = w.error(); return false; }
  }

  if (!w.endArray()) { outError = w.error(); return false; }
  if (!w.endObject()) { outError = w.error(); return false; }

  if (!w.ok()) {
    outError = w.error();
    return false;
  }

  outJson = oss.str();
  if (opt.pretty && !outJson.empty() && outJson.back() != '\n') outJson.push_back('\n');
  outError.clear();
  return true;
}

} // namespace

namespace isocity {

bool SaveWorldPatchJson(const WorldPatch& patch, const std::string& path, std::string& outError,
                        const JsonWriteOptions& opt)
{
  std::string text;
  if (!SerializePatchJson(patch, text, outError, opt)) return false;
  return WriteFileText(path, text, outError);
}

bool LoadWorldPatchJson(WorldPatch& outPatch, const std::string& path, std::string& outError)
{
  std::string text;
  if (!ReadFileText(path, text, outError)) return false;
  return DeserializeWorldPatchJson(outPatch, text, outError);
}

bool SerializeWorldPatchJson(const WorldPatch& patch, std::string& outJson, std::string& outError,
                             const JsonWriteOptions& opt)
{
  return SerializePatchJson(patch, outJson, outError, opt);
}

bool DeserializeWorldPatchJson(WorldPatch& outPatch, const std::string& jsonText, std::string& outError)
{
  JsonValue root;
  std::string err;
  if (!ParseJson(jsonText, root, err)) {
    outError = err;
    return false;
  }

  if (!DeserializePatchJson(root, outPatch, err)) {
    outError = err;
    return false;
  }

  outError.clear();
  return true;
}

} // namespace isocity
