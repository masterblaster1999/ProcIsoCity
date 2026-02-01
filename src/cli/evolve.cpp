#include "isocity/AutoBuild.hpp"
#include "isocity/Dossier.hpp"
#include "isocity/Hash.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Random.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Version.hpp"

#include "cli/CliParse.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

// ----------------------------------------------------------------------------
// Small CLI parsing helpers
// ----------------------------------------------------------------------------

bool ParseI32(const std::string& s, int* out)
{
  return isocity::cli::ParseI32(s, out);
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  return isocity::cli::ParseU64(s, out);
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  return isocity::cli::ParseWxH(s, outW, outH);
}

bool ParseF64(const std::string& s, double* out)
{
  return isocity::cli::ParseF64(s, out);
}

std::string HexU64(std::uint64_t v)
{
  return isocity::cli::HexU64(v);
}

bool EnsureParentDir(const std::filesystem::path& file)
{
  return isocity::cli::EnsureParentDir(file);
}

// ----------------------------------------------------------------------------
// Score expression parser (RPN via shunting-yard)
// ----------------------------------------------------------------------------

enum class TokKind : std::uint8_t { Number, Ident, Op, LParen, RParen, Comma };

struct Token {
  TokKind kind{};
  std::string text;
  double number = 0.0;
};

bool IsIdentStart(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

bool IsIdentChar(char c)
{
  return IsIdentStart(c) || (c >= '0' && c <= '9');
}

bool TokenizeExpr(const std::string& expr, std::vector<Token>& out, std::string& err)
{
  out.clear();
  err.clear();

  const std::size_t n = expr.size();
  std::size_t i = 0;
  while (i < n) {
    const char c = expr[i];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
      ++i;
      continue;
    }

    if (c == '(') {
      out.push_back(Token{TokKind::LParen, "("});
      ++i;
      continue;
    }
    if (c == ')') {
      out.push_back(Token{TokKind::RParen, ")"});
      ++i;
      continue;
    }
    if (c == ',') {
      out.push_back(Token{TokKind::Comma, ","});
      ++i;
      continue;
    }

    // Operators
    if (c == '+' || c == '-' || c == '*' || c == '/' || c == '^') {
      out.push_back(Token{TokKind::Op, std::string(1, c)});
      ++i;
      continue;
    }

    // Number (support leading '.' and scientific notation)
    if ((c >= '0' && c <= '9') || c == '.') {
      std::size_t start = i;
      bool sawDot = (c == '.');
      ++i;
      while (i < n) {
        const char d = expr[i];
        if (d >= '0' && d <= '9') {
          ++i;
          continue;
        }
        if (d == '.' && !sawDot) {
          sawDot = true;
          ++i;
          continue;
        }
        break;
      }

      // exponent
      if (i < n && (expr[i] == 'e' || expr[i] == 'E')) {
        std::size_t j = i + 1;
        if (j < n && (expr[j] == '+' || expr[j] == '-')) ++j;
        bool hasDigits = false;
        while (j < n && (expr[j] >= '0' && expr[j] <= '9')) {
          hasDigits = true;
          ++j;
        }
        if (hasDigits) {
          i = j;
        }
      }

      const std::string numStr = expr.substr(start, i - start);
      double v = 0.0;
      if (!ParseF64(numStr, &v)) {
        err = "invalid number: " + numStr;
        return false;
      }
      Token t;
      t.kind = TokKind::Number;
      t.text = numStr;
      t.number = v;
      out.push_back(std::move(t));
      continue;
    }

    // Identifier
    if (IsIdentStart(c)) {
      std::size_t start = i;
      ++i;
      while (i < n && IsIdentChar(expr[i])) ++i;
      out.push_back(Token{TokKind::Ident, expr.substr(start, i - start)});
      continue;
    }

    err = std::string("unexpected character in expression: '") + c + "'";
    return false;
  }

  return true;
}

struct RpnItem {
  enum class Kind : std::uint8_t { Number, Var, Op, Func };
  Kind kind{};
  std::string text;
  double number = 0.0;
};

int OpPrec(const std::string& op)
{
  if (op == "u-") return 4;
  if (op == "^") return 3;
  if (op == "*" || op == "/") return 2;
  if (op == "+" || op == "-") return 1;
  return 0;
}

bool OpRightAssoc(const std::string& op)
{
  return op == "^" || op == "u-";
}

bool IsFuncName(const std::string& s)
{
  const std::string_view v(s);
  return v == "abs" || v == "sqrt" || v == "log" || v == "exp" || v == "min" || v == "max" || v == "clamp" ||
         v == "pow";
}

int FuncArity(const std::string& name)
{
  if (name == "abs" || name == "sqrt" || name == "log" || name == "exp") return 1;
  if (name == "min" || name == "max" || name == "pow") return 2;
  if (name == "clamp") return 3;
  return -1;
}

bool ToRpn(const std::vector<Token>& toks, std::vector<RpnItem>& out, std::string& err)
{
  out.clear();
  err.clear();

  struct OpEntry {
    std::string text;
    TokKind kind{}; // Op or Ident(for func) or LParen
  };

  std::vector<OpEntry> stack;
  stack.reserve(toks.size());

  auto popOpsToOutput = [&](const std::string& until) {
    while (!stack.empty() && stack.back().text != until) {
      const OpEntry e = stack.back();
      stack.pop_back();
      if (e.kind == TokKind::Op) {
        out.push_back(RpnItem{RpnItem::Kind::Op, e.text});
      } else if (e.kind == TokKind::Ident) {
        out.push_back(RpnItem{RpnItem::Kind::Func, e.text});
      }
    }
  };

  TokKind prev = TokKind::Comma; // treat as boundary so leading '-' becomes unary
  for (std::size_t i = 0; i < toks.size(); ++i) {
    const Token& t = toks[i];
    switch (t.kind) {
      case TokKind::Number:
        out.push_back(RpnItem{RpnItem::Kind::Number, t.text, t.number});
        prev = TokKind::Number;
        break;
      case TokKind::Ident: {
        // Function call if followed by '(' and name recognized.
        const bool isFunc = (i + 1 < toks.size() && toks[i + 1].kind == TokKind::LParen && IsFuncName(t.text));
        if (isFunc) {
          stack.push_back(OpEntry{t.text, TokKind::Ident});
        } else {
          out.push_back(RpnItem{RpnItem::Kind::Var, t.text});
        }
        prev = TokKind::Ident;
      } break;
      case TokKind::LParen:
        stack.push_back(OpEntry{"(", TokKind::LParen});
        prev = TokKind::LParen;
        break;
      case TokKind::Comma:
        // Function argument separator: pop until '('.
        popOpsToOutput("(");
        if (stack.empty()) {
          err = "misplaced comma";
          return false;
        }
        prev = TokKind::Comma;
        break;
      case TokKind::Op: {
        std::string op = t.text;
        if (op == "-" && (prev == TokKind::Op || prev == TokKind::LParen || prev == TokKind::Comma)) {
          op = "u-"; // unary minus
        }

        const int p = OpPrec(op);
        if (p <= 0) {
          err = "unknown operator: " + op;
          return false;
        }

        while (!stack.empty()) {
          const OpEntry& top = stack.back();
          if (top.kind == TokKind::Op) {
            const int pTop = OpPrec(top.text);
            if (pTop > p || (pTop == p && !OpRightAssoc(op))) {
              out.push_back(RpnItem{RpnItem::Kind::Op, top.text});
              stack.pop_back();
              continue;
            }
          } else if (top.kind == TokKind::Ident) {
            // Functions bind tighter than ops.
            out.push_back(RpnItem{RpnItem::Kind::Func, top.text});
            stack.pop_back();
            continue;
          }
          break;
        }
        stack.push_back(OpEntry{op, TokKind::Op});
        prev = TokKind::Op;
      } break;
      case TokKind::RParen:
        popOpsToOutput("(");
        if (stack.empty() || stack.back().kind != TokKind::LParen) {
          err = "mismatched ')'";
          return false;
        }
        stack.pop_back(); // remove '('
        // If top is a function, pop it.
        if (!stack.empty() && stack.back().kind == TokKind::Ident) {
          out.push_back(RpnItem{RpnItem::Kind::Func, stack.back().text});
          stack.pop_back();
        }
        prev = TokKind::RParen;
        break;
    }
  }

  while (!stack.empty()) {
    const OpEntry e = stack.back();
    stack.pop_back();
    if (e.kind == TokKind::LParen) {
      err = "mismatched '('";
      return false;
    }
    if (e.kind == TokKind::Op) out.push_back(RpnItem{RpnItem::Kind::Op, e.text});
    if (e.kind == TokKind::Ident) out.push_back(RpnItem{RpnItem::Kind::Func, e.text});
  }

  return true;
}

bool EvalRpn(const std::vector<RpnItem>& rpn, const std::unordered_map<std::string, double>& vars, double& out,
             std::string& err)
{
  err.clear();
  std::vector<double> st;
  st.reserve(rpn.size());

  auto pop1 = [&](double& a) -> bool {
    if (st.empty()) return false;
    a = st.back();
    st.pop_back();
    return true;
  };
  auto pop2 = [&](double& a, double& b) -> bool {
    if (st.size() < 2) return false;
    b = st.back();
    st.pop_back();
    a = st.back();
    st.pop_back();
    return true;
  };

  for (const auto& it : rpn) {
    switch (it.kind) {
      case RpnItem::Kind::Number: st.push_back(it.number); break;
      case RpnItem::Kind::Var: {
        auto f = vars.find(it.text);
        if (f == vars.end()) {
          err = "unknown variable: " + it.text;
          return false;
        }
        st.push_back(f->second);
      } break;
      case RpnItem::Kind::Op: {
        if (it.text == "u-") {
          double a = 0.0;
          if (!pop1(a)) {
            err = "stack underflow for unary -";
            return false;
          }
          st.push_back(-a);
          break;
        }
        double a = 0.0, b = 0.0;
        if (!pop2(a, b)) {
          err = "stack underflow for operator: " + it.text;
          return false;
        }
        double r = 0.0;
        if (it.text == "+") r = a + b;
        else if (it.text == "-") r = a - b;
        else if (it.text == "*") r = a * b;
        else if (it.text == "/") r = (b == 0.0) ? std::numeric_limits<double>::infinity() : (a / b);
        else if (it.text == "^") r = std::pow(a, b);
        else {
          err = "unknown operator: " + it.text;
          return false;
        }
        st.push_back(r);
      } break;
      case RpnItem::Kind::Func: {
        const int ar = FuncArity(it.text);
        if (ar <= 0) {
          err = "unknown function: " + it.text;
          return false;
        }
        if (static_cast<int>(st.size()) < ar) {
          err = "stack underflow for function: " + it.text;
          return false;
        }
        auto popN = [&](int n) {
          std::vector<double> args;
          args.reserve(static_cast<std::size_t>(n));
          for (int k = 0; k < n; ++k) {
            args.push_back(st.back());
            st.pop_back();
          }
          std::reverse(args.begin(), args.end());
          return args;
        };

        const auto args = popN(ar);
        double r = 0.0;
        if (it.text == "abs") r = std::fabs(args[0]);
        else if (it.text == "sqrt") r = std::sqrt(std::max(0.0, args[0]));
        else if (it.text == "log") r = std::log(std::max(1.0e-12, args[0]));
        else if (it.text == "exp") r = std::exp(args[0]);
        else if (it.text == "min") r = std::min(args[0], args[1]);
        else if (it.text == "max") r = std::max(args[0], args[1]);
        else if (it.text == "pow") r = std::pow(args[0], args[1]);
        else if (it.text == "clamp") r = std::clamp(args[0], args[1], args[2]);
        else {
          err = "unhandled function: " + it.text;
          return false;
        }
        st.push_back(r);
      } break;
    }

    if (!st.empty() && !std::isfinite(st.back())) {
      // Keep the evaluator robust; clamp to a large finite value.
      const double sign = (st.back() < 0.0) ? -1.0 : 1.0;
      st.back() = sign * 1.0e30;
    }
  }

  if (st.size() != 1) {
    err = "expression did not reduce to a single value";
    return false;
  }
  out = st.back();
  return true;
}

void AddVar(std::unordered_map<std::string, double>& vars, const std::string& name, double v)
{
  vars[name] = v;
}

std::unordered_map<std::string, double> BuildStatsVars(const isocity::Stats& s)
{
  std::unordered_map<std::string, double> vars;
  vars.reserve(128);

  AddVar(vars, "day", static_cast<double>(s.day));
  AddVar(vars, "population", static_cast<double>(s.population));
  AddVar(vars, "pop", static_cast<double>(s.population));
  AddVar(vars, "housingCapacity", static_cast<double>(s.housingCapacity));
  AddVar(vars, "jobsCapacity", static_cast<double>(s.jobsCapacity));
  AddVar(vars, "jobsCapacityAccessible", static_cast<double>(s.jobsCapacityAccessible));
  AddVar(vars, "employed", static_cast<double>(s.employed));
  AddVar(vars, "happiness", static_cast<double>(s.happiness));
  AddVar(vars, "money", static_cast<double>(s.money));
  AddVar(vars, "roads", static_cast<double>(s.roads));
  AddVar(vars, "parks", static_cast<double>(s.parks));

  AddVar(vars, "commuters", static_cast<double>(s.commuters));
  AddVar(vars, "commutersUnreachable", static_cast<double>(s.commutersUnreachable));
  AddVar(vars, "avgCommute", static_cast<double>(s.avgCommute));
  AddVar(vars, "p95Commute", static_cast<double>(s.p95Commute));
  AddVar(vars, "avgCommuteTime", static_cast<double>(s.avgCommuteTime));
  AddVar(vars, "p95CommuteTime", static_cast<double>(s.p95CommuteTime));
  AddVar(vars, "trafficCongestion", static_cast<double>(s.trafficCongestion));
  AddVar(vars, "congestion", static_cast<double>(s.trafficCongestion));
  AddVar(vars, "congestedRoadTiles", static_cast<double>(s.congestedRoadTiles));
  AddVar(vars, "maxRoadTraffic", static_cast<double>(s.maxRoadTraffic));

  AddVar(vars, "transitLines", static_cast<double>(s.transitLines));
  AddVar(vars, "transitStops", static_cast<double>(s.transitStops));
  AddVar(vars, "transitRiders", static_cast<double>(s.transitRiders));
  AddVar(vars, "transitModeShare", static_cast<double>(s.transitModeShare));
  AddVar(vars, "transitCommuteCoverage", static_cast<double>(s.transitCommuteCoverage));

  AddVar(vars, "servicesEducationFacilities", static_cast<double>(s.servicesEducationFacilities));
  AddVar(vars, "servicesHealthFacilities", static_cast<double>(s.servicesHealthFacilities));
  AddVar(vars, "servicesSafetyFacilities", static_cast<double>(s.servicesSafetyFacilities));
  AddVar(vars, "servicesEducationSatisfaction", static_cast<double>(s.servicesEducationSatisfaction));
  AddVar(vars, "servicesHealthSatisfaction", static_cast<double>(s.servicesHealthSatisfaction));
  AddVar(vars, "servicesSafetySatisfaction", static_cast<double>(s.servicesSafetySatisfaction));
  AddVar(vars, "servicesOverallSatisfaction", static_cast<double>(s.servicesOverallSatisfaction));
  AddVar(vars, "servicesMaintenanceCost", static_cast<double>(s.servicesMaintenanceCost));

  AddVar(vars, "goodsProduced", static_cast<double>(s.goodsProduced));
  AddVar(vars, "goodsDemand", static_cast<double>(s.goodsDemand));
  AddVar(vars, "goodsDelivered", static_cast<double>(s.goodsDelivered));
  AddVar(vars, "goodsImported", static_cast<double>(s.goodsImported));
  AddVar(vars, "goodsExported", static_cast<double>(s.goodsExported));
  AddVar(vars, "goodsUnreachableDemand", static_cast<double>(s.goodsUnreachableDemand));
  AddVar(vars, "goodsSatisfaction", static_cast<double>(s.goodsSatisfaction));
  AddVar(vars, "maxRoadGoodsTraffic", static_cast<double>(s.maxRoadGoodsTraffic));

  AddVar(vars, "tradeImportCapacityPct", static_cast<double>(s.tradeImportCapacityPct));
  AddVar(vars, "tradeExportCapacityPct", static_cast<double>(s.tradeExportCapacityPct));
  AddVar(vars, "tradeMarketIndex", static_cast<double>(s.tradeMarketIndex));

  AddVar(vars, "economyIndex", static_cast<double>(s.economyIndex));
  AddVar(vars, "economyInflation", static_cast<double>(s.economyInflation));
  AddVar(vars, "economyCityWealth", static_cast<double>(s.economyCityWealth));

  AddVar(vars, "income", static_cast<double>(s.income));
  AddVar(vars, "expenses", static_cast<double>(s.expenses));
  AddVar(vars, "taxRevenue", static_cast<double>(s.taxRevenue));
  AddVar(vars, "maintenanceCost", static_cast<double>(s.maintenanceCost));
  AddVar(vars, "upgradeCost", static_cast<double>(s.upgradeCost));
  AddVar(vars, "importCost", static_cast<double>(s.importCost));
  AddVar(vars, "exportRevenue", static_cast<double>(s.exportRevenue));
  AddVar(vars, "avgTaxPerCapita", static_cast<double>(s.avgTaxPerCapita));
  AddVar(vars, "transitCost", static_cast<double>(s.transitCost));

  AddVar(vars, "demandResidential", static_cast<double>(s.demandResidential));
  AddVar(vars, "demandCommercial", static_cast<double>(s.demandCommercial));
  AddVar(vars, "demandIndustrial", static_cast<double>(s.demandIndustrial));
  AddVar(vars, "avgLandValue", static_cast<double>(s.avgLandValue));

  AddVar(vars, "trafficSafetyResidentMeanExposure", static_cast<double>(s.trafficSafetyResidentMeanExposure));
  AddVar(vars, "trafficSafetyResidentMeanPriority", static_cast<double>(s.trafficSafetyResidentMeanPriority));
  AddVar(vars, "trafficSafetyHappinessPenalty", static_cast<double>(s.trafficSafetyHappinessPenalty));
  AddVar(vars, "airPollutionResidentAvg01", static_cast<double>(s.airPollutionResidentAvg01));
  AddVar(vars, "airPollutionResidentHighExposureFrac", static_cast<double>(s.airPollutionResidentHighExposureFrac));
  AddVar(vars, "airPollutionHappinessPenalty", static_cast<double>(s.airPollutionHappinessPenalty));

  AddVar(vars, "fireIncidentDamaged", static_cast<double>(s.fireIncidentDamaged));
  AddVar(vars, "fireIncidentDestroyed", static_cast<double>(s.fireIncidentDestroyed));
  AddVar(vars, "fireIncidentDisplaced", static_cast<double>(s.fireIncidentDisplaced));
  AddVar(vars, "fireIncidentJobsLostCap", static_cast<double>(s.fireIncidentJobsLostCap));
  AddVar(vars, "fireIncidentCost", static_cast<double>(s.fireIncidentCost));
  AddVar(vars, "fireIncidentHappinessPenalty", static_cast<double>(s.fireIncidentHappinessPenalty));
  AddVar(vars, "trafficIncidentInjuries", static_cast<double>(s.trafficIncidentInjuries));
  AddVar(vars, "trafficIncidentCost", static_cast<double>(s.trafficIncidentCost));
  AddVar(vars, "trafficIncidentHappinessPenalty", static_cast<double>(s.trafficIncidentHappinessPenalty));

  // Convenience constants.
  AddVar(vars, "pi", 3.14159265358979323846);
  AddVar(vars, "e", 2.71828182845904523536);
  AddVar(vars, "true", 1.0);
  AddVar(vars, "false", 0.0);

  return vars;
}

// ----------------------------------------------------------------------------
// Evolution engine: cross-entropy-style parameter tuning + seed elitism
// ----------------------------------------------------------------------------

struct Genome {
  std::uint64_t seed = 1;

  // ProcGen knobs (subset; still produces a very wide space).
  float terrainScale = 0.08f;
  float waterLevel = 0.35f;
  float sandLevel = 0.42f;
  int hubs = 4;
  int extraConnections = 2;
  isocity::ProcGenRoadLayout roadLayout = isocity::ProcGenRoadLayout::Organic;
  float zoneChance = 0.22f;
  float parkChance = 0.06f;
  isocity::ProcGenTerrainPreset terrainPreset = isocity::ProcGenTerrainPreset::Classic;
  float terrainPresetStrength = 1.0f;
  bool roadHierarchyEnabled = true;
  float roadHierarchyStrength = 1.0f;
  isocity::ProcGenDistrictingMode districtingMode = isocity::ProcGenDistrictingMode::BlockGraph;
};

// Quantized key so we can cache expensive evaluations without worrying about float bit patterns.
struct GenomeKey {
  std::uint64_t seed = 0;
  int terrainScale = 0;
  int waterLevel = 0;
  int sandLevel = 0;
  int hubs = 0;
  int extraConnections = 0;
  int roadLayout = 0;
  int zoneChance = 0;
  int parkChance = 0;
  int terrainPreset = 0;
  int terrainPresetStrength = 0;
  int roadHierarchyEnabled = 0;
  int roadHierarchyStrength = 0;
  int districtingMode = 0;

  bool operator==(const GenomeKey& o) const
  {
    return seed == o.seed && terrainScale == o.terrainScale && waterLevel == o.waterLevel && sandLevel == o.sandLevel &&
           hubs == o.hubs && extraConnections == o.extraConnections && roadLayout == o.roadLayout &&
           zoneChance == o.zoneChance && parkChance == o.parkChance && terrainPreset == o.terrainPreset &&
           terrainPresetStrength == o.terrainPresetStrength && roadHierarchyEnabled == o.roadHierarchyEnabled &&
           roadHierarchyStrength == o.roadHierarchyStrength && districtingMode == o.districtingMode;
  }
};

struct GenomeKeyHash {
  std::size_t operator()(const GenomeKey& k) const
  {
    // Simple mix.
    std::uint64_t x = k.seed;
    auto mix = [&](std::uint64_t v) {
      x ^= v + 0x9E3779B97F4A7C15ULL + (x << 6) + (x >> 2);
    };
    mix(static_cast<std::uint64_t>(k.terrainScale));
    mix(static_cast<std::uint64_t>(k.waterLevel));
    mix(static_cast<std::uint64_t>(k.sandLevel));
    mix(static_cast<std::uint64_t>(k.hubs));
    mix(static_cast<std::uint64_t>(k.extraConnections));
    mix(static_cast<std::uint64_t>(k.roadLayout));
    mix(static_cast<std::uint64_t>(k.zoneChance));
    mix(static_cast<std::uint64_t>(k.parkChance));
    mix(static_cast<std::uint64_t>(k.terrainPreset));
    mix(static_cast<std::uint64_t>(k.terrainPresetStrength));
    mix(static_cast<std::uint64_t>(k.roadHierarchyEnabled));
    mix(static_cast<std::uint64_t>(k.roadHierarchyStrength));
    mix(static_cast<std::uint64_t>(k.districtingMode));
    return static_cast<std::size_t>(x);
  }
};

GenomeKey MakeKey(const Genome& g)
{
  auto q = [](float v, float scale) -> int { return static_cast<int>(std::lround(static_cast<double>(v) * scale)); };
  GenomeKey k;
  k.seed = g.seed;
  k.terrainScale = q(g.terrainScale, 100000.0f);
  k.waterLevel = q(g.waterLevel, 100000.0f);
  k.sandLevel = q(g.sandLevel, 100000.0f);
  k.hubs = g.hubs;
  k.extraConnections = g.extraConnections;
  k.roadLayout = static_cast<int>(g.roadLayout);
  k.zoneChance = q(g.zoneChance, 100000.0f);
  k.parkChance = q(g.parkChance, 100000.0f);
  k.terrainPreset = static_cast<int>(g.terrainPreset);
  k.terrainPresetStrength = q(g.terrainPresetStrength, 100000.0f);
  k.roadHierarchyEnabled = g.roadHierarchyEnabled ? 1 : 0;
  k.roadHierarchyStrength = q(g.roadHierarchyStrength, 100000.0f);
  k.districtingMode = static_cast<int>(g.districtingMode);
  return k;
}

double RandNormal(isocity::RNG& rng)
{
  // Box-Muller (avoid log(0)).
  const double u1 = std::max(1.0e-12, static_cast<double>(rng.nextF01()));
  const double u2 = static_cast<double>(rng.nextF01());
  const double r = std::sqrt(-2.0 * std::log(u1));
  const double theta = 2.0 * 3.14159265358979323846 * u2;
  return r * std::cos(theta);
}

float Clamp01(float v)
{
  return std::clamp(v, 0.0f, 1.0f);
}

float ClampFloat(float v, float lo, float hi)
{
  return std::clamp(v, lo, hi);
}

int ClampInt(int v, int lo, int hi)
{
  return std::clamp(v, lo, hi);
}

struct Dist {
  // Means/variances (the cross-entropy "belief" about good cities).
  double terrainScaleMean = 0.08;
  double terrainScaleStd = 0.02;
  double waterLevelMean = 0.35;
  double waterLevelStd = 0.06;
  double sandDeltaMean = 0.07; // sandLevel - waterLevel
  double sandDeltaStd = 0.03;

  double zoneChanceMean = 0.22;
  double zoneChanceStd = 0.06;
  double parkChanceMean = 0.06;
  double parkChanceStd = 0.03;

  double terrainPresetStrengthMean = 1.0;
  double terrainPresetStrengthStd = 0.35;
  double roadHierarchyStrengthMean = 1.0;
  double roadHierarchyStrengthStd = 0.40;

  double hubsMean = 4.0;
  double hubsStd = 1.6;
  double extraConnMean = 2.0;
  double extraConnStd = 1.2;

  // Categorical frequencies for enums.
  std::vector<double> roadLayoutP;
  std::vector<isocity::ProcGenRoadLayout> roadLayoutVals;
  std::vector<double> terrainPresetP;
  std::vector<isocity::ProcGenTerrainPreset> terrainPresetVals;
  std::vector<double> districtModeP;
  std::vector<isocity::ProcGenDistrictingMode> districtModeVals;

  double roadHierarchyEnabledP = 0.85;
};

template <typename T>
T SampleCategorical(isocity::RNG& rng, const std::vector<double>& p, const std::vector<T>& v)
{
  if (p.empty() || v.empty() || p.size() != v.size()) return T{};
  double sum = 0.0;
  for (double x : p) sum += std::max(0.0, x);
  if (!(sum > 0.0)) return v[0];
  double r = static_cast<double>(rng.nextF01()) * sum;
  for (std::size_t i = 0; i < p.size(); ++i) {
    r -= std::max(0.0, p[i]);
    if (r <= 0.0) return v[i];
  }
  return v.back();
}

void NormalizeP(std::vector<double>& p, double floor = 0.001)
{
  if (p.empty()) return;
  double sum = 0.0;
  for (double& x : p) {
    x = std::max(floor, x);
    sum += x;
  }
  if (!(sum > 0.0)) {
    const double u = 1.0 / static_cast<double>(p.size());
    for (double& x : p) x = u;
    return;
  }
  for (double& x : p) x /= sum;
}

Dist MakeDefaultDist()
{
  Dist d;
  d.roadLayoutVals = {isocity::ProcGenRoadLayout::Organic, isocity::ProcGenRoadLayout::Grid,
                      isocity::ProcGenRoadLayout::Radial, isocity::ProcGenRoadLayout::TensorField,
                      isocity::ProcGenRoadLayout::SpaceColonization, isocity::ProcGenRoadLayout::VoronoiCells,
                      isocity::ProcGenRoadLayout::Physarum, isocity::ProcGenRoadLayout::MedialAxis};
  d.roadLayoutP = {0.28, 0.14, 0.14, 0.10, 0.13, 0.09, 0.08, 0.04};
  NormalizeP(d.roadLayoutP);

  d.terrainPresetVals = {
      isocity::ProcGenTerrainPreset::Classic,
      isocity::ProcGenTerrainPreset::Island,
      isocity::ProcGenTerrainPreset::Archipelago,
      isocity::ProcGenTerrainPreset::InlandSea,
      isocity::ProcGenTerrainPreset::RiverValley,
      isocity::ProcGenTerrainPreset::MountainRing,
      isocity::ProcGenTerrainPreset::Fjords,
      isocity::ProcGenTerrainPreset::Canyon,
      isocity::ProcGenTerrainPreset::Volcano,
      isocity::ProcGenTerrainPreset::Delta,
      isocity::ProcGenTerrainPreset::Tectonic,
  };
  d.terrainPresetP = {0.20, 0.10, 0.08, 0.08, 0.10, 0.08, 0.08, 0.07, 0.07, 0.07, 0.07};
  NormalizeP(d.terrainPresetP);

  d.districtModeVals = {isocity::ProcGenDistrictingMode::Voronoi, isocity::ProcGenDistrictingMode::RoadFlow,
                        isocity::ProcGenDistrictingMode::BlockGraph, isocity::ProcGenDistrictingMode::Watershed};
  d.districtModeP = {0.12, 0.30, 0.46, 0.12};
  NormalizeP(d.districtModeP);

  return d;
}

Genome SampleGenome(isocity::RNG& rng, const Dist& d, const std::vector<std::uint64_t>& eliteSeeds, double seedReuseP,
                    double seedMutBits)
{
  Genome g;

  // Seeds: exploit elite seeds with bit mutation, otherwise explore.
  const bool useElite = !eliteSeeds.empty() && rng.nextF01() < static_cast<float>(seedReuseP);
  if (useElite) {
    const std::uint64_t base = eliteSeeds[static_cast<std::size_t>(rng.rangeU32(static_cast<std::uint32_t>(eliteSeeds.size())))] ;
    // Flip a sparse set of bits.
    std::uint64_t mask = 0;
    const int bits = std::max(0, static_cast<int>(std::lround(seedMutBits)));
    for (int i = 0; i < bits; ++i) {
      mask |= (std::uint64_t{1} << (rng.rangeU32(64)));
    }
    g.seed = base ^ mask;
  } else {
    g.seed = rng.nextU64();
  }
  if (g.seed == 0) g.seed = 1;

  auto sampleN = [&](double mean, double std, double lo, double hi) {
    const double v = mean + std * RandNormal(rng);
    return std::clamp(v, lo, hi);
  };
  auto sampleNi = [&](double mean, double std, int lo, int hi) {
    const double v = mean + std * RandNormal(rng);
    const int iv = static_cast<int>(std::lround(v));
    return std::clamp(iv, lo, hi);
  };

  g.terrainScale = static_cast<float>(sampleN(d.terrainScaleMean, d.terrainScaleStd, 0.02, 0.18));
  g.waterLevel = static_cast<float>(sampleN(d.waterLevelMean, d.waterLevelStd, 0.15, 0.65));
  const double sandDelta = sampleN(d.sandDeltaMean, d.sandDeltaStd, 0.02, 0.22);
  g.sandLevel = static_cast<float>(std::min<double>(0.85, std::max<double>(g.waterLevel + 0.01, g.waterLevel + sandDelta)));

  g.zoneChance = static_cast<float>(sampleN(d.zoneChanceMean, d.zoneChanceStd, 0.05, 0.55));
  g.parkChance = static_cast<float>(sampleN(d.parkChanceMean, d.parkChanceStd, 0.0, 0.35));

  g.terrainPresetStrength = static_cast<float>(sampleN(d.terrainPresetStrengthMean, d.terrainPresetStrengthStd, 0.0, 2.5));
  g.roadHierarchyStrength = static_cast<float>(sampleN(d.roadHierarchyStrengthMean, d.roadHierarchyStrengthStd, 0.0, 3.0));
  g.roadHierarchyEnabled = rng.nextF01() < static_cast<float>(d.roadHierarchyEnabledP);

  g.hubs = sampleNi(d.hubsMean, d.hubsStd, 2, 10);
  g.extraConnections = sampleNi(d.extraConnMean, d.extraConnStd, 0, 12);

  g.roadLayout = SampleCategorical(rng, d.roadLayoutP, d.roadLayoutVals);
  g.terrainPreset = SampleCategorical(rng, d.terrainPresetP, d.terrainPresetVals);
  g.districtingMode = SampleCategorical(rng, d.districtModeP, d.districtModeVals);

  // Keep consistent (sand above water).
  if (g.sandLevel < g.waterLevel + 0.01f) g.sandLevel = g.waterLevel + 0.01f;
  g.sandLevel = ClampFloat(g.sandLevel, 0.0f, 0.95f);
  g.waterLevel = ClampFloat(g.waterLevel, 0.0f, g.sandLevel - 0.01f);

  return g;
}

void UpdateDistFromElites(Dist& d, const std::vector<Genome>& elites)
{
  if (elites.empty()) return;

  auto meanStd = [&](auto getter, double& outMean, double& outStd, double minStd) {
    double m = 0.0;
    for (const auto& g : elites) m += static_cast<double>(getter(g));
    m /= static_cast<double>(elites.size());
    double v = 0.0;
    for (const auto& g : elites) {
      const double x = static_cast<double>(getter(g));
      v += (x - m) * (x - m);
    }
    v /= static_cast<double>(elites.size());
    outMean = m;
    outStd = std::max(minStd, std::sqrt(std::max(0.0, v)) * 1.15);
  };

  meanStd([](const Genome& g) { return g.terrainScale; }, d.terrainScaleMean, d.terrainScaleStd, 0.004);
  meanStd([](const Genome& g) { return g.waterLevel; }, d.waterLevelMean, d.waterLevelStd, 0.01);
  meanStd([](const Genome& g) { return (g.sandLevel - g.waterLevel); }, d.sandDeltaMean, d.sandDeltaStd, 0.008);
  meanStd([](const Genome& g) { return g.zoneChance; }, d.zoneChanceMean, d.zoneChanceStd, 0.01);
  meanStd([](const Genome& g) { return g.parkChance; }, d.parkChanceMean, d.parkChanceStd, 0.008);
  meanStd([](const Genome& g) { return g.terrainPresetStrength; }, d.terrainPresetStrengthMean,
          d.terrainPresetStrengthStd, 0.02);
  meanStd([](const Genome& g) { return g.roadHierarchyStrength; }, d.roadHierarchyStrengthMean,
          d.roadHierarchyStrengthStd, 0.02);
  meanStd([](const Genome& g) { return static_cast<double>(g.hubs); }, d.hubsMean, d.hubsStd, 0.15);
  meanStd([](const Genome& g) { return static_cast<double>(g.extraConnections); }, d.extraConnMean, d.extraConnStd, 0.12);

  // Categorical updates via frequency + smoothing.
  auto updateCat = [&](auto getter, std::vector<double>& p, const auto& vals) {
    if (p.size() != vals.size()) return;
    std::fill(p.begin(), p.end(), 0.0);
    for (const auto& g : elites) {
      const auto v = getter(g);
      for (std::size_t i = 0; i < vals.size(); ++i) {
        if (vals[i] == v) {
          p[i] += 1.0;
          break;
        }
      }
    }
    // Smooth to keep exploration.
    for (double& x : p) x = x + 0.25;
    NormalizeP(p, 0.001);
  };

  updateCat([](const Genome& g) { return g.roadLayout; }, d.roadLayoutP, d.roadLayoutVals);
  updateCat([](const Genome& g) { return g.terrainPreset; }, d.terrainPresetP, d.terrainPresetVals);
  updateCat([](const Genome& g) { return g.districtingMode; }, d.districtModeP, d.districtModeVals);

  // Bernoulli update.
  int enabled = 0;
  for (const auto& g : elites) enabled += g.roadHierarchyEnabled ? 1 : 0;
  const double frac = static_cast<double>(enabled) / static_cast<double>(elites.size());
  d.roadHierarchyEnabledP = std::clamp(0.15 + 0.70 * frac, 0.05, 0.95);
}

isocity::ProcGenConfig BuildProcCfgFromGenome(const Genome& g)
{
  isocity::ProcGenConfig cfg;
  cfg.terrainScale = g.terrainScale;
  cfg.waterLevel = g.waterLevel;
  cfg.sandLevel = g.sandLevel;
  cfg.hubs = g.hubs;
  cfg.extraConnections = g.extraConnections;
  cfg.roadLayout = g.roadLayout;
  cfg.zoneChance = g.zoneChance;
  cfg.parkChance = g.parkChance;
  cfg.terrainPreset = g.terrainPreset;
  cfg.terrainPresetStrength = g.terrainPresetStrength;
  cfg.roadHierarchyEnabled = g.roadHierarchyEnabled;
  cfg.roadHierarchyStrength = g.roadHierarchyStrength;
  cfg.districtingMode = g.districtingMode;
  return cfg;
}

std::string GenomeSummary(const Genome& g)
{
  std::ostringstream oss;
  oss << "seed=" << HexU64(g.seed)
      << " preset=" << isocity::ToString(g.terrainPreset) << "(" << std::fixed << std::setprecision(2)
      << g.terrainPresetStrength << ")"
      << " layout=" << isocity::ToString(g.roadLayout)
      << " hubs=" << g.hubs << " extra=" << g.extraConnections
      << " water=" << std::setprecision(3) << g.waterLevel << " sand=" << g.sandLevel
      << " terrainScale=" << g.terrainScale
      << " zoneChance=" << g.zoneChance << " parkChance=" << g.parkChance
      << " roadHier=" << (g.roadHierarchyEnabled ? "on" : "off") << "(" << std::setprecision(2)
      << g.roadHierarchyStrength << ")"
      << " districts=" << isocity::ToString(g.districtingMode);
  return oss.str();
}

// Compact share-string: a stable-ish genome encoding for copying into bug reports.
// Not meant for cryptographic use; just a convenient text handle.
std::string GenomeCode(const Genome& g)
{
  // Pack a few quantized fields into a human-friendly hex token.
  auto q01 = [](float v) -> std::uint16_t { return static_cast<std::uint16_t>(std::clamp<int>(static_cast<int>(std::lround(v * 65535.0f)), 0, 65535)); };
  const std::uint16_t ts = static_cast<std::uint16_t>(std::clamp<int>(static_cast<int>(std::lround(g.terrainScale * 10000.0f)), 0, 65535));
  const std::uint16_t wl = q01(g.waterLevel);
  const std::uint16_t sl = q01(g.sandLevel);
  const std::uint16_t zc = q01(Clamp01(g.zoneChance));
  const std::uint16_t pc = q01(Clamp01(g.parkChance));
  const std::uint16_t ps = q01(Clamp01(g.terrainPresetStrength / 2.5f));
  const std::uint16_t rh = q01(Clamp01(g.roadHierarchyStrength / 3.0f));
  // A compact bit-pack for discrete fields.
  // NOTE: This used to be a u16, but road layouts outgrew 2 bits.
  // We now use a u32 and accept both formats in ParseGenomeCode.
  const std::uint32_t bits = static_cast<std::uint32_t>((static_cast<int>(g.terrainPreset) & 0xF) |
                                                        ((static_cast<int>(g.roadLayout) & 0xF) << 4) |
                                                        ((static_cast<int>(g.districtingMode) & 0x3) << 8) |
                                                        ((g.roadHierarchyEnabled ? 1 : 0) << 10) |
                                                        ((std::clamp(g.hubs, 0, 15) & 0xF) << 11) |
                                                        ((std::clamp(g.extraConnections, 0, 15) & 0xF) << 15));

  // A tiny checksum so typos are obvious.
  std::uint64_t chk = g.seed;
  chk ^= (static_cast<std::uint64_t>(ts) << 16) ^ wl ^ (static_cast<std::uint64_t>(sl) << 32);
  chk ^= (static_cast<std::uint64_t>(zc) << 48) ^ pc;
  chk ^= (static_cast<std::uint64_t>(ps) << 8) ^ (static_cast<std::uint64_t>(rh) << 24);
  chk ^= static_cast<std::uint64_t>(bits) * 0x9E3779B97F4A7C15ULL;
  // Mix.
  chk ^= chk >> 33;
  chk *= 0xff51afd7ed558ccdULL;
  chk ^= chk >> 33;
  chk *= 0xc4ceb9fe1a85ec53ULL;
  chk ^= chk >> 33;

  std::ostringstream oss;
  oss << "G";
  oss << std::hex << std::setw(16) << std::setfill('0') << g.seed;
  oss << "-" << std::setw(4) << ts;
  oss << std::setw(4) << wl;
  oss << std::setw(4) << sl;
  oss << std::setw(4) << zc;
  oss << std::setw(4) << pc;
  oss << std::setw(4) << ps;
  oss << std::setw(4) << rh;
  oss << std::setw(8) << bits;
  oss << "-" << std::setw(8) << (static_cast<std::uint32_t>(chk & 0xffffffffu));
  return oss.str();
}

bool ParseHexU64Exact(std::string_view s, std::uint64_t& out)
{
  if (s.empty()) return false;
  if (s.size() > 16) return false;
  std::uint64_t v = 0;
  for (const char c : s) {
    std::uint8_t d = 0;
    if (c >= '0' && c <= '9') d = static_cast<std::uint8_t>(c - '0');
    else if (c >= 'a' && c <= 'f') d = static_cast<std::uint8_t>(10 + (c - 'a'));
    else if (c >= 'A' && c <= 'F') d = static_cast<std::uint8_t>(10 + (c - 'A'));
    else return false;
    v = (v << 4) | static_cast<std::uint64_t>(d);
  }
  out = v;
  return true;
}

bool ParseHexU32Exact(std::string_view s, std::uint32_t& out)
{
  std::uint64_t v = 0;
  if (!ParseHexU64Exact(s, v)) return false;
  if (v > 0xffffffffu) return false;
  out = static_cast<std::uint32_t>(v);
  return true;
}

bool ParseGenomeCode(const std::string& code, Genome& out, std::string& outErr)
{
  outErr.clear();
  if (code.empty() || code[0] != 'G') {
    outErr = "Genome code must start with 'G'.";
    return false;
  }

  // Format (new): G<seed16>-<ts wl sl zc pc ps rh bits32>-<chk8>
  //   pack field length: 36 hex digits (7x u16 + 1x u32)
  // Format (legacy): G<seed16>-<ts wl sl zc pc ps rh bits16 (8x u16 hex)>-<chk8>
  //   pack field length: 32 hex digits
  const std::size_t p0 = code.find('-');
  if (p0 == std::string::npos) {
    outErr = "Genome code missing first '-' separator.";
    return false;
  }
  const std::size_t p1 = code.find('-', p0 + 1);
  if (p1 == std::string::npos) {
    outErr = "Genome code missing checksum '-' separator.";
    return false;
  }

  const std::string_view seedHex(code.data() + 1, p0 - 1);
  const std::string_view pack(code.data() + p0 + 1, p1 - (p0 + 1));
  const std::string_view chkHex(code.data() + p1 + 1, code.size() - (p1 + 1));

  if (seedHex.size() != 16) {
    outErr = "Genome seed field must be 16 hex digits.";
    return false;
  }
  const bool legacy = (pack.size() == 32);
  const bool modern = (pack.size() == 36);
  if (!legacy && !modern) {
    outErr = "Genome pack field must be 32 (legacy) or 36 (modern) hex digits.";
    return false;
  }
  if (chkHex.size() != 8) {
    outErr = "Genome checksum field must be 8 hex digits.";
    return false;
  }

  std::uint64_t seed = 0;
  if (!ParseHexU64Exact(seedHex, seed)) {
    outErr = "Genome seed hex parse failed.";
    return false;
  }

  auto u16 = [&](int i) -> std::uint16_t {
    const std::string_view s(pack.data() + i * 4, 4);
    std::uint64_t v = 0;
    if (!ParseHexU64Exact(s, v) || v > 0xffffu) {
      outErr = "Genome pack hex parse failed.";
      return 0;
    }
    return static_cast<std::uint16_t>(v);
  };

  const std::uint16_t ts = u16(0);
  const std::uint16_t wl = u16(1);
  const std::uint16_t sl = u16(2);
  const std::uint16_t zc = u16(3);
  const std::uint16_t pc = u16(4);
  const std::uint16_t ps = u16(5);
  const std::uint16_t rh = u16(6);
  if (!outErr.empty()) return false;

  std::uint32_t bits = 0;
  if (legacy) {
    bits = static_cast<std::uint32_t>(u16(7));
  } else {
    const std::string_view bitsHex(pack.data() + 28, 8);
    if (!ParseHexU32Exact(bitsHex, bits)) {
      outErr = "Genome bits32 parse failed.";
      return false;
    }
  }

  std::uint32_t chkGot = 0;
  if (!ParseHexU32Exact(chkHex, chkGot)) {
    outErr = "Genome checksum parse failed.";
    return false;
  }

  // Recompute checksum (matches GenomeCode).
  std::uint64_t chk = seed;
  chk ^= (static_cast<std::uint64_t>(ts) << 16) ^ wl ^ (static_cast<std::uint64_t>(sl) << 32);
  chk ^= (static_cast<std::uint64_t>(zc) << 48) ^ pc;
  chk ^= (static_cast<std::uint64_t>(ps) << 8) ^ (static_cast<std::uint64_t>(rh) << 24);
  chk ^= static_cast<std::uint64_t>(bits) * 0x9E3779B97F4A7C15ULL;
  chk ^= chk >> 33;
  chk *= 0xff51afd7ed558ccdULL;
  chk ^= chk >> 33;
  chk *= 0xc4ceb9fe1a85ec53ULL;
  chk ^= chk >> 33;
  const std::uint32_t chkWant = static_cast<std::uint32_t>(chk & 0xffffffffu);

  if (chkWant != chkGot) {
    std::ostringstream oss;
    oss << "Genome checksum mismatch (expected 0x" << std::hex << std::setw(8) << std::setfill('0') << chkWant
        << ", got 0x" << std::setw(8) << chkGot << ").";
    outErr = oss.str();
    return false;
  }

  auto deq01 = [](std::uint16_t q) -> float { return static_cast<float>(q) / 65535.0f; };
  Genome g;
  g.seed = seed;
  g.terrainScale = static_cast<float>(ts) / 10000.0f;
  g.waterLevel = deq01(wl);
  g.sandLevel = deq01(sl);
  g.zoneChance = deq01(zc);
  g.parkChance = deq01(pc);
  g.terrainPresetStrength = deq01(ps) * 2.5f;
  g.roadHierarchyStrength = deq01(rh) * 3.0f;
  g.terrainPreset = static_cast<isocity::ProcGenTerrainPreset>(bits & 0xF);
  if (legacy) {
    g.roadLayout = static_cast<isocity::ProcGenRoadLayout>((bits >> 4) & 0x3);
    g.districtingMode = static_cast<isocity::ProcGenDistrictingMode>((bits >> 6) & 0x3);
    g.roadHierarchyEnabled = ((bits >> 8) & 0x1) != 0;
    g.hubs = static_cast<int>((bits >> 9) & 0xF);
    g.extraConnections = static_cast<int>((bits >> 13) & 0xF);
  } else {
    g.roadLayout = static_cast<isocity::ProcGenRoadLayout>((bits >> 4) & 0xF);
    g.districtingMode = static_cast<isocity::ProcGenDistrictingMode>((bits >> 8) & 0x3);
    g.roadHierarchyEnabled = ((bits >> 10) & 0x1) != 0;
    g.hubs = static_cast<int>((bits >> 11) & 0xF);
    g.extraConnections = static_cast<int>((bits >> 15) & 0xF);
  }

  // Clamp out-of-range enums defensively.
  {
    const std::uint8_t tp = isocity::ClampProcGenTerrainPresetU8(static_cast<std::uint8_t>(g.terrainPreset));
    g.terrainPreset = static_cast<isocity::ProcGenTerrainPreset>(tp);
  }
  {
    const std::uint8_t rl = isocity::ClampProcGenRoadLayoutU8(static_cast<std::uint8_t>(g.roadLayout));
    g.roadLayout = static_cast<isocity::ProcGenRoadLayout>(rl);
  }
  {
    const std::uint8_t dm = isocity::ClampProcGenDistrictingModeU8(static_cast<std::uint8_t>(g.districtingMode));
    g.districtingMode = static_cast<isocity::ProcGenDistrictingMode>(dm);
  }

  // Minimal sanity clamping for safety.
  g.hubs = std::clamp(g.hubs, 2, 12);
  g.extraConnections = std::clamp(g.extraConnections, 0, 12);
  g.terrainScale = std::clamp(g.terrainScale, 0.01f, 0.25f);
  g.waterLevel = std::clamp(g.waterLevel, 0.05f, 0.90f);
  g.sandLevel = std::clamp(g.sandLevel, 0.05f, 0.98f);
  if (g.sandLevel <= g.waterLevel + 0.01f) g.sandLevel = std::min(0.98f, g.waterLevel + 0.05f);
  g.zoneChance = Clamp01(g.zoneChance);
  g.parkChance = Clamp01(g.parkChance);
  g.terrainPresetStrength = std::clamp(g.terrainPresetStrength, 0.0f, 2.5f);
  g.roadHierarchyStrength = std::clamp(g.roadHierarchyStrength, 0.0f, 3.0f);
  out = g;
  return true;
}

struct Eval {
  Genome g;
  isocity::Stats s;
  std::uint64_t hash = 0;
  double score = -1.0e30;
  bool cached = false;
};

struct CacheVal {
  isocity::Stats s;
  std::uint64_t hash = 0;
  double score = -1.0e30;
};

bool WriteCsv(const std::filesystem::path& path, const std::vector<Eval>& rows, std::string& err)
{
  err.clear();
  if (path.empty()) return true;
  if (!EnsureParentDir(path)) {
    err = "failed to create output directory";
    return false;
  }
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    err = "failed to open output: " + path.string();
    return false;
  }

  f << "rank,score,cached,seed,genome_code,hash,population,happiness,money,avgCommuteTime,trafficCongestion,goodsSatisfaction,servicesOverallSatisfaction,transitModeShare\n";
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& e = rows[i];
    f << (i + 1) << ',';
    f << std::fixed << std::setprecision(6) << e.score << ',';
    f << (e.cached ? 1 : 0) << ',';
    f << HexU64(e.g.seed) << ',';
    f << '"' << GenomeCode(e.g) << '"' << ',';
    f << HexU64(e.hash) << ',';
    f << e.s.population << ',';
    f << std::fixed << std::setprecision(4) << e.s.happiness << ',';
    f << e.s.money << ',';
    f << std::fixed << std::setprecision(3) << e.s.avgCommuteTime << ',';
    f << std::fixed << std::setprecision(4) << e.s.trafficCongestion << ',';
    f << std::fixed << std::setprecision(4) << e.s.goodsSatisfaction << ',';
    f << std::fixed << std::setprecision(4) << e.s.servicesOverallSatisfaction << ',';
    f << std::fixed << std::setprecision(4) << e.s.transitModeShare;
    f << "\n";
  }

  return static_cast<bool>(f);
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_evolve (procedural city evolution lab)\n\n"
      << "Searches the procedural generation space (seed + macro ProcGen config)\n"
      << "and evaluates each candidate by running the deterministic AutoBuild bot.\n\n"
      << "Usage:\n"
      << "  proc_isocity_evolve [options]\n\n"
      << "Core options:\n"
      << "  --size WxH               World size (default 96x96)\n"
      << "  --days N                 AutoBuild days per evaluation (default 120)\n"
      << "  --population N           Candidates per generation (default 48)\n"
      << "  --generations N          Number of generations (default 10)\n"
      << "  --elite N                Elite set size (default 10)\n"
      << "  --rng-seed <u64>          Deterministic RNG seed for the search process\n"
      << "  --money N                Starting money override (default: keep world default)\n\n"
      << "Scoring:\n"
      << "  --score \"<expr>\"         Score expression over final Stats (default shown below)\n"
      << "  --minimize               Minimize score instead of maximize\n\n"
      << "AutoBuild tuning (repeatable):\n"
      << "  --bot <key> <value>      Same keys as proc_isocity_autobuild (e.g. zonesPerDay 4)\n\n"
      << "Outputs:\n"
      << "  --out <results.csv>      Write ranked results CSV\n"
      << "  --best-save <world.bin>  Save best world (delta save includes ProcGen+Sim cfg)\n"
      << "  --best-dossier <dir>     Export a full dossier bundle for the best world\n\n"
      << "Repro:\n"
      << "  --genome <code>          Evaluate a single genome code and exit (works with --best-save/--best-dossier)\n\n"
      << "Default score:\n"
      << "  population*(0.60+0.80*happiness) - 120*trafficCongestion - 0.10*avgCommuteTime\n"
      << "  - 80*trafficSafetyResidentMeanPriority - 50*airPollutionResidentAvg01\n\n"
      << "Expression functions:\n"
      << "  abs(x), sqrt(x), log(x), exp(x), min(a,b), max(a,b), pow(a,b), clamp(x,lo,hi)\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  if (argc < 2) {
    PrintHelp();
    return 1;
  }

  int w = 96;
  int h = 96;
  int days = 120;
  int populationN = 48;
  int generations = 10;
  int eliteN = 10;
  std::uint64_t rngSeed = 0;
  int startMoney = -1;
  bool minimize = false;

  std::filesystem::path outCsv;
  std::filesystem::path bestSave;
  std::filesystem::path bestDossier;

  std::string scoreExpr =
      "population*(0.60+0.80*happiness) - 120*trafficCongestion - 0.10*avgCommuteTime "
      "- 80*trafficSafetyResidentMeanPriority - 50*airPollutionResidentAvg01";

  // If provided, evaluate exactly one genome and exit.
  std::string singleGenome;

  AutoBuildConfig botCfg;

  // Evolution tuning (kept minimal; power users can adjust via these flags).
  double seedReuseP = 0.70;   // probability to sample from elite seeds
  double seedMutBits = 6.0;   // number of bits to flip when mutating a seed
  double eliteFracMin = 0.15; // if eliteN not set, fallback

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](int n) {
      if (i + n >= argc) {
        std::cerr << "Missing value after " << a << "\n";
        std::exit(2);
      }
    };

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--version" || a == "-V") {
      std::cout << "ProcIsoCity " << ProcIsoCityFullVersionString() << "\n";
      return 0;
    } else if (a == "--size") {
      need(1);
      if (!ParseWxH(argv[++i], &w, &h)) {
        std::cerr << "Invalid --size (expected WxH)\n";
        return 2;
      }
    } else if (a == "--days") {
      need(1);
      if (!ParseI32(argv[++i], &days) || days < 0) {
        std::cerr << "Invalid --days\n";
        return 2;
      }
    } else if (a == "--population") {
      need(1);
      if (!ParseI32(argv[++i], &populationN) || populationN < 2) {
        std::cerr << "Invalid --population\n";
        return 2;
      }
    } else if (a == "--generations") {
      need(1);
      if (!ParseI32(argv[++i], &generations) || generations < 1) {
        std::cerr << "Invalid --generations\n";
        return 2;
      }
    } else if (a == "--elite") {
      need(1);
      if (!ParseI32(argv[++i], &eliteN) || eliteN < 1) {
        std::cerr << "Invalid --elite\n";
        return 2;
      }
    } else if (a == "--rng-seed") {
      need(1);
      if (!ParseU64(argv[++i], &rngSeed)) {
        std::cerr << "Invalid --rng-seed\n";
        return 2;
      }
    } else if (a == "--money") {
      need(1);
      if (!ParseI32(argv[++i], &startMoney)) {
        std::cerr << "Invalid --money\n";
        return 2;
      }
    } else if (a == "--score") {
      need(1);
      scoreExpr = argv[++i];
    } else if (a == "--minimize") {
      minimize = true;
    } else if (a == "--out") {
      need(1);
      outCsv = argv[++i];
    } else if (a == "--best-save") {
      need(1);
      bestSave = argv[++i];
    } else if (a == "--best-dossier") {
      need(1);
      bestDossier = argv[++i];
    } else if (a == "--genome") {
      need(1);
      singleGenome = argv[++i];
    } else if (a == "--bot") {
      need(2);
      const std::string key = argv[++i];
      const std::string val = argv[++i];
      std::string err;
      if (!ParseAutoBuildKey(key, val, botCfg, err)) {
        std::cerr << "Invalid --bot key/value: " << key << "=" << val;
        if (!err.empty()) std::cerr << " (" << err << ")";
        std::cerr << "\n";
        return 2;
      }
    } else if (a == "--seed-reuse") {
      need(1);
      double v = 0.0;
      if (!ParseF64(argv[++i], &v)) {
        std::cerr << "Invalid --seed-reuse\n";
        return 2;
      }
      seedReuseP = std::clamp(v, 0.0, 1.0);
    } else if (a == "--seed-mutate-bits") {
      need(1);
      double v = 0.0;
      if (!ParseF64(argv[++i], &v)) {
        std::cerr << "Invalid --seed-mutate-bits\n";
        return 2;
      }
      seedMutBits = std::clamp(v, 0.0, 64.0);
    } else {
      std::cerr << "Unknown arg: " << a << " (try --help)\n";
      return 2;
    }
  }

  // Parse score expression.
  std::vector<Token> toks;
  std::vector<RpnItem> rpn;
  std::string exprErr;
  if (!TokenizeExpr(scoreExpr, toks, exprErr)) {
    std::cerr << "Score expression error: " << exprErr << "\n";
    return 2;
  }
  if (!ToRpn(toks, rpn, exprErr)) {
    std::cerr << "Score expression error: " << exprErr << "\n";
    return 2;
  }
  {
    // Quick sanity check: evaluate against an empty stats map should fail with unknown variable
    // if the expression mentions stats variables. That's okay; we just ensure evaluation machinery works.
    double tmp = 0.0;
    std::unordered_map<std::string, double> empty;
    std::string evalErr;
    (void)EvalRpn(rpn, empty, tmp, evalErr);
  }

  if (eliteN <= 0) eliteN = std::max(1, static_cast<int>(std::lround(populationN * eliteFracMin)));
  eliteN = std::clamp(eliteN, 1, populationN);

  if (rngSeed == 0) rngSeed = TimeSeed();
  RNG rng(rngSeed);

  std::cout << "ProcIsoCity evolve\n";
  std::cout << "  version: " << ProcIsoCityFullVersionString() << "\n";
  std::cout << "  size: " << w << "x" << h << "  days: " << days << "\n";
  std::cout << "  population: " << populationN << "  generations: " << generations << "  elite: " << eliteN << "\n";
  std::cout << "  rngSeed: " << HexU64(rngSeed) << "\n";
  std::cout << "  score: " << scoreExpr << (minimize ? "  (minimize)" : "") << "\n\n";

  Dist dist = MakeDefaultDist();

  std::unordered_map<GenomeKey, CacheVal, GenomeKeyHash> cache;
  cache.reserve(static_cast<std::size_t>(populationN * generations * 2));

  std::vector<std::uint64_t> eliteSeeds;
  eliteSeeds.reserve(static_cast<std::size_t>(eliteN));

  std::vector<Eval> allEvaluations;
  allEvaluations.reserve(static_cast<std::size_t>(populationN * generations));

  auto evaluate = [&](const Genome& g) -> Eval {
    Eval e;
    e.g = g;
    const GenomeKey key = MakeKey(g);
    auto it = cache.find(key);
    if (it != cache.end()) {
      e.s = it->second.s;
      e.hash = it->second.hash;
      e.score = it->second.score;
      e.cached = true;
      return e;
    }

    ProcGenConfig procCfg = BuildProcCfgFromGenome(g);
    SimConfig simCfg;

    World world = GenerateWorld(w, h, g.seed, procCfg);
    if (startMoney >= 0) world.stats().money = startMoney;

    Simulator sim(simCfg);
    (void)RunAutoBuild(world, sim, botCfg, days, nullptr);
    sim.refreshDerivedStats(world);

    e.s = world.stats();
    e.hash = HashWorld(world, true);

    const auto vars = BuildStatsVars(e.s);
    double raw = 0.0;
    std::string evalErr;
    if (!EvalRpn(rpn, vars, raw, evalErr)) {
      // Penalize invalid expressions (shouldn't happen if vars exist).
      raw = -1.0e30;
    }
    e.score = minimize ? -raw : raw;

    cache[key] = CacheVal{e.s, e.hash, e.score};
    return e;
  };

  Eval bestEver;
  bestEver.score = -1.0e30;

  if (!singleGenome.empty()) {
    Genome g;
    std::string gErr;
    if (!ParseGenomeCode(singleGenome, g, gErr)) {
      std::cerr << "Invalid --genome: " << gErr << "\n";
      return 2;
    }

    std::cout << "Evaluating single genome:\n";
    std::cout << "  " << GenomeSummary(g) << "\n";
    std::cout << "  genome=" << singleGenome << "\n\n";
    bestEver = evaluate(g);
    allEvaluations.push_back(bestEver);
    goto after_search;
  }

  for (int gen = 0; gen < generations; ++gen) {
    std::vector<Eval> pop;
    pop.reserve(static_cast<std::size_t>(populationN));

    for (int i = 0; i < populationN; ++i) {
      Genome g = SampleGenome(rng, dist, eliteSeeds, seedReuseP, seedMutBits);
      pop.push_back(evaluate(g));
    }

    std::sort(pop.begin(), pop.end(), [](const Eval& a, const Eval& b) { return a.score > b.score; });
    for (const auto& e : pop) allEvaluations.push_back(e);

    const int eliteTake = std::min(eliteN, static_cast<int>(pop.size()));
    std::vector<Genome> elites;
    elites.reserve(static_cast<std::size_t>(eliteTake));
    eliteSeeds.clear();
    for (int i = 0; i < eliteTake; ++i) {
      elites.push_back(pop[static_cast<std::size_t>(i)].g);
      eliteSeeds.push_back(pop[static_cast<std::size_t>(i)].g.seed);
    }

    if (!pop.empty() && pop.front().score > bestEver.score) {
      bestEver = pop.front();
    }

    UpdateDistFromElites(dist, elites);

    const auto& b = pop.front();
    std::cout << "Gen " << (gen + 1) << "/" << generations << ": best score=" << std::fixed << std::setprecision(6)
              << b.score << (b.cached ? " (cached)" : "") << "\n";
    std::cout << "  " << GenomeSummary(b.g) << "\n";
    std::cout << "  genome=" << GenomeCode(b.g) << "  hash=" << HexU64(b.hash) << "\n";
    std::cout << "  pop=" << b.s.population << " happy=" << std::fixed << std::setprecision(3) << b.s.happiness
              << " money=" << b.s.money << " commute=" << b.s.avgCommuteTime
              << " congestion=" << b.s.trafficCongestion << "\n\n";
  }

after_search:

  // Final ranked output over all evaluated candidates (dedupe by cache key would be possible, but raw list is more honest).
  std::sort(allEvaluations.begin(), allEvaluations.end(), [](const Eval& a, const Eval& b) { return a.score > b.score; });
  if (allEvaluations.size() > static_cast<std::size_t>(populationN * generations)) {
    allEvaluations.resize(static_cast<std::size_t>(populationN * generations));
  }

  std::string outErr;
  if (!WriteCsv(outCsv, allEvaluations, outErr)) {
    std::cerr << "Failed to write CSV: " << outErr << "\n";
    return 2;
  }

  // Re-run best for artifacts (ticks + dossier) with deterministic configs.
  if (!bestSave.empty() || !bestDossier.empty()) {
    ProcGenConfig procCfg = BuildProcCfgFromGenome(bestEver.g);
    SimConfig simCfg;
    World world = GenerateWorld(w, h, bestEver.g.seed, procCfg);
    if (startMoney >= 0) world.stats().money = startMoney;
    Simulator sim(simCfg);
    std::vector<Stats> ticks;
    ticks.reserve(static_cast<std::size_t>(std::max(0, days) + 1));
    (void)RunAutoBuild(world, sim, botCfg, days, &ticks);
    sim.refreshDerivedStats(world);

    if (!bestSave.empty()) {
      std::string err;
      if (!SaveWorldBinary(world, procCfg, simCfg, bestSave.string(), err)) {
        std::cerr << "Failed to write best save: " << err << "\n";
        return 2;
      }
      std::cout << "Wrote best save: " << bestSave.string() << "\n";
    }

    if (!bestDossier.empty()) {
      CityDossierConfig cfg;
      cfg.outDir = bestDossier;
      CityDossierResult res;
      std::string err;
      if (!WriteCityDossier(world, procCfg, simCfg, ticks, cfg, &res, err)) {
        std::cerr << "Failed to write dossier: " << err << "\n";
        return 2;
      }
      std::cout << "Wrote dossier: " << res.outDir.string() << " (hash " << HexU64(res.hash) << ")\n";
    }
  }

  if (!allEvaluations.empty()) {
    const auto& b = allEvaluations.front();
    std::cout << "\nBest overall:\n";
    std::cout << "  score=" << std::fixed << std::setprecision(6) << b.score << "\n";
    std::cout << "  " << GenomeSummary(b.g) << "\n";
    std::cout << "  genome=" << GenomeCode(b.g) << "  hash=" << HexU64(b.hash) << "\n";
  }

  return 0;
}
