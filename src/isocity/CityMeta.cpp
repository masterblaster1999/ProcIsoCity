#include "isocity/CityMeta.hpp"

#include "isocity/FileSync.hpp"
#include "isocity/Json.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

namespace isocity {
namespace {

constexpr std::size_t kMaxHistoryEntries = 4096;
constexpr std::size_t kMaxNewsEntries = 4096;
constexpr std::size_t kMaxChallenges = 256;
constexpr std::size_t kMaxChallengeLog = 2048;
constexpr std::size_t kMaxNewsAddendum = 2048;

static std::string ToLowerAscii(std::string s)
{
  for (char& c : s) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return s;
}

static const char* ToneToString(CityNewsTone t)
{
  switch (t) {
    case CityNewsTone::Good: return "good";
    case CityNewsTone::Neutral: return "neutral";
    case CityNewsTone::Bad: return "bad";
    case CityNewsTone::Alert: return "alert";
  }
  return "neutral";
}

static bool ToneFromString(const std::string& s, CityNewsTone& out)
{
  const std::string k = ToLowerAscii(s);
  if (k == "good") {
    out = CityNewsTone::Good;
    return true;
  }
  if (k == "neutral") {
    out = CityNewsTone::Neutral;
    return true;
  }
  if (k == "bad") {
    out = CityNewsTone::Bad;
    return true;
  }
  if (k == "alert") {
    out = CityNewsTone::Alert;
    return true;
  }
  return false;
}

static const char* ChallengeKindToString(CityChallengeKind k)
{
  switch (k) {
    case CityChallengeKind::GrowPopulation: return "grow_population";
    case CityChallengeKind::BuildParks: return "build_parks";
    case CityChallengeKind::ReduceCongestion: return "reduce_congestion";
    case CityChallengeKind::ImproveGoods: return "improve_goods";
    case CityChallengeKind::ImproveServices: return "improve_services";
    case CityChallengeKind::BalanceBudget: return "balance_budget";
    case CityChallengeKind::RestoreOutsideConnection: return "restore_outside_connection";
  }
  return "grow_population";
}

static bool ChallengeKindFromString(const std::string& s, CityChallengeKind& out)
{
  const std::string k = ToLowerAscii(s);
  if (k == "grow_population") {
    out = CityChallengeKind::GrowPopulation;
    return true;
  }
  if (k == "build_parks") {
    out = CityChallengeKind::BuildParks;
    return true;
  }
  if (k == "reduce_congestion") {
    out = CityChallengeKind::ReduceCongestion;
    return true;
  }
  if (k == "improve_goods") {
    out = CityChallengeKind::ImproveGoods;
    return true;
  }
  if (k == "improve_services") {
    out = CityChallengeKind::ImproveServices;
    return true;
  }
  if (k == "balance_budget") {
    out = CityChallengeKind::BalanceBudget;
    return true;
  }
  if (k == "restore_outside_connection") {
    out = CityChallengeKind::RestoreOutsideConnection;
    return true;
  }
  return false;
}

static const char* ChallengeStatusToString(CityChallengeStatus s)
{
  switch (s) {
    case CityChallengeStatus::Active: return "active";
    case CityChallengeStatus::Completed: return "completed";
    case CityChallengeStatus::Failed: return "failed";
    case CityChallengeStatus::Canceled: return "canceled";
  }
  return "active";
}

static bool ChallengeStatusFromString(const std::string& s, CityChallengeStatus& out)
{
  const std::string k = ToLowerAscii(s);
  if (k == "active") {
    out = CityChallengeStatus::Active;
    return true;
  }
  if (k == "completed") {
    out = CityChallengeStatus::Completed;
    return true;
  }
  if (k == "failed") {
    out = CityChallengeStatus::Failed;
    return true;
  }
  if (k == "canceled") {
    out = CityChallengeStatus::Canceled;
    return true;
  }
  return false;
}

static bool IsFinite(double v)
{
  return std::isfinite(v);
}

static bool JsonGetNumber(const JsonValue& obj, const char* key, double& out, std::string& err, bool required)
{
  const JsonValue* v = FindJsonMember(obj, key);
  if (!v) {
    if (required) err = std::string("missing '") + key + "'";
    return !required;
  }
  if (!v->isNumber() || !IsFinite(v->numberValue)) {
    err = std::string("'") + key + "' is not a finite number";
    return false;
  }
  out = v->numberValue;
  return true;
}

static bool JsonGetInt(const JsonValue& obj, const char* key, int& out, std::string& err, bool required)
{
  double d = 0.0;
  if (!JsonGetNumber(obj, key, d, err, required)) return !required;
  if (!IsFinite(d) || d < static_cast<double>(std::numeric_limits<int>::min()) ||
      d > static_cast<double>(std::numeric_limits<int>::max())) {
    err = std::string("'") + key + "' out of range";
    return false;
  }
  // Require integral.
  if (std::floor(d) != d) {
    err = std::string("'") + key + "' must be an integer";
    return false;
  }
  out = static_cast<int>(d);
  return true;
}

static bool JsonGetU32(const JsonValue& obj, const char* key, std::uint32_t& out, std::string& err, bool required)
{
  double d = 0.0;
  if (!JsonGetNumber(obj, key, d, err, required)) return !required;
  if (!IsFinite(d) || d < 0.0 || d > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    err = std::string("'") + key + "' out of range";
    return false;
  }
  if (std::floor(d) != d) {
    err = std::string("'") + key + "' must be an integer";
    return false;
  }
  out = static_cast<std::uint32_t>(d);
  return true;
}

static bool JsonGetFloat(const JsonValue& obj, const char* key, float& out, std::string& err, bool required)
{
  double d = 0.0;
  if (!JsonGetNumber(obj, key, d, err, required)) return !required;
  if (!IsFinite(d) || d < -static_cast<double>(std::numeric_limits<float>::max()) ||
      d > static_cast<double>(std::numeric_limits<float>::max())) {
    err = std::string("'") + key + "' out of range";
    return false;
  }
  out = static_cast<float>(d);
  return true;
}

static bool JsonGetString(const JsonValue& obj, const char* key, std::string& out, std::string& err, bool required)
{
  const JsonValue* v = FindJsonMember(obj, key);
  if (!v) {
    if (required) err = std::string("missing '") + key + "'";
    return !required;
  }
  if (!v->isString()) {
    err = std::string("'") + key + "' is not a string";
    return false;
  }
  out = v->stringValue;
  return true;
}

static bool JsonGetSeed(const JsonValue& obj, std::uint64_t& outSeed, std::string& err)
{
  const JsonValue* v = FindJsonMember(obj, "seed");
  if (!v) {
    err = "missing 'seed'";
    return false;
  }

  if (v->isString()) {
    const std::string& s = v->stringValue;
    try {
      std::size_t pos = 0;
      const unsigned long long tmp = std::stoull(s, &pos, 10);
      if (pos != s.size()) {
        err = "seed string has trailing characters";
        return false;
      }
      outSeed = static_cast<std::uint64_t>(tmp);
      return true;
    } catch (const std::exception&) {
      err = "failed to parse seed";
      return false;
    }
  }

  if (v->isNumber() && IsFinite(v->numberValue)) {
    // Best-effort compatibility: numeric seeds are only safe up to 2^53.
    const double d = v->numberValue;
    if (d < 0.0 || d > 18446744073709551615.0) {
      err = "seed number out of range";
      return false;
    }
    if (std::floor(d) != d) {
      err = "seed must be an integer";
      return false;
    }
    outSeed = static_cast<std::uint64_t>(d);
    return true;
  }

  err = "seed must be a string or number";
  return false;
}

static JsonValue CityHistorySampleToJson(const CityHistorySample& s)
{
  JsonValue obj = JsonValue::MakeObject();
  obj.objectValue.push_back({"day", JsonValue::MakeNumber(s.day)});
  obj.objectValue.push_back({"population", JsonValue::MakeNumber(s.population)});
  obj.objectValue.push_back({"money", JsonValue::MakeNumber(s.money)});
  obj.objectValue.push_back({"happiness", JsonValue::MakeNumber(s.happiness)});
  obj.objectValue.push_back({"demandResidential", JsonValue::MakeNumber(s.demandResidential)});
  obj.objectValue.push_back({"demandCommercial", JsonValue::MakeNumber(s.demandCommercial)});
  obj.objectValue.push_back({"demandIndustrial", JsonValue::MakeNumber(s.demandIndustrial)});
  obj.objectValue.push_back({"avgLandValue", JsonValue::MakeNumber(s.avgLandValue)});
  obj.objectValue.push_back({"avgTaxPerCapita", JsonValue::MakeNumber(s.avgTaxPerCapita)});
  obj.objectValue.push_back({"income", JsonValue::MakeNumber(s.income)});
  obj.objectValue.push_back({"expenses", JsonValue::MakeNumber(s.expenses)});
  obj.objectValue.push_back({"taxRevenue", JsonValue::MakeNumber(s.taxRevenue)});
  obj.objectValue.push_back({"maintenanceCost", JsonValue::MakeNumber(s.maintenanceCost)});
  obj.objectValue.push_back({"commuters", JsonValue::MakeNumber(s.commuters)});
  obj.objectValue.push_back({"avgCommute", JsonValue::MakeNumber(s.avgCommute)});
  obj.objectValue.push_back({"avgCommuteTime", JsonValue::MakeNumber(s.avgCommuteTime)});
  obj.objectValue.push_back({"trafficCongestion", JsonValue::MakeNumber(s.trafficCongestion)});
  obj.objectValue.push_back({"goodsSatisfaction", JsonValue::MakeNumber(s.goodsSatisfaction)});
  return obj;
}

static bool CityHistorySampleFromJson(const JsonValue& v, CityHistorySample& out, std::string& err)
{
  if (!v.isObject()) {
    err = "history entry must be an object";
    return false;
  }
  if (!JsonGetInt(v, "day", out.day, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "population", out.population, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "money", out.money, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "happiness", out.happiness, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "demandResidential", out.demandResidential, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "demandCommercial", out.demandCommercial, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "demandIndustrial", out.demandIndustrial, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "avgLandValue", out.avgLandValue, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "avgTaxPerCapita", out.avgTaxPerCapita, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "income", out.income, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "expenses", out.expenses, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "taxRevenue", out.taxRevenue, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "maintenanceCost", out.maintenanceCost, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "commuters", out.commuters, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "avgCommute", out.avgCommute, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "avgCommuteTime", out.avgCommuteTime, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "trafficCongestion", out.trafficCongestion, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "goodsSatisfaction", out.goodsSatisfaction, err, /*required=*/true)) return false;
  return true;
}

static JsonValue CityNewsEntryToJson(const CityNewsEntry& e)
{
  JsonValue obj = JsonValue::MakeObject();
  obj.objectValue.push_back({"day", JsonValue::MakeNumber(e.day)});
  obj.objectValue.push_back({"tone", JsonValue::MakeString(ToneToString(e.tone))});
  obj.objectValue.push_back({"mayorRating", JsonValue::MakeNumber(e.mayorRating)});
  obj.objectValue.push_back({"headline", JsonValue::MakeString(e.headline)});
  obj.objectValue.push_back({"body", JsonValue::MakeString(e.body)});
  return obj;
}

static bool CityNewsEntryFromJson(const JsonValue& v, CityNewsEntry& out, std::string& err)
{
  if (!v.isObject()) {
    err = "news entry must be an object";
    return false;
  }
  if (!JsonGetInt(v, "day", out.day, err, /*required=*/true)) return false;

  // tone: prefer string (readable), accept numeric for older/best-effort.
  {
    const JsonValue* t = FindJsonMember(v, "tone");
    if (!t) {
      err = "missing 'tone'";
      return false;
    }
    if (t->isString()) {
      CityNewsTone tone = CityNewsTone::Neutral;
      if (!ToneFromString(t->stringValue, tone)) {
        err = "invalid news tone";
        return false;
      }
      out.tone = tone;
    } else if (t->isNumber()) {
      const double d = t->numberValue;
      if (!IsFinite(d) || std::floor(d) != d) {
        err = "tone must be integer";
        return false;
      }
      const int ti = static_cast<int>(d);
      switch (ti) {
        case 0: out.tone = CityNewsTone::Good; break;
        case 1: out.tone = CityNewsTone::Neutral; break;
        case 2: out.tone = CityNewsTone::Bad; break;
        case 3: out.tone = CityNewsTone::Alert; break;
        default:
          err = "invalid tone value";
          return false;
      }
    } else {
      err = "tone must be string or number";
      return false;
    }
  }

  if (!JsonGetFloat(v, "mayorRating", out.mayorRating, err, /*required=*/true)) return false;
  if (!JsonGetString(v, "headline", out.headline, err, /*required=*/true)) return false;
  if (!JsonGetString(v, "body", out.body, err, /*required=*/true)) return false;
  return true;
}

static JsonValue CityChallengeToJson(const CityChallenge& c)
{
  JsonValue obj = JsonValue::MakeObject();
  obj.objectValue.push_back({"id", JsonValue::MakeNumber(c.id)});
  obj.objectValue.push_back({"kind", JsonValue::MakeString(ChallengeKindToString(c.kind))});
  obj.objectValue.push_back({"status", JsonValue::MakeString(ChallengeStatusToString(c.status))});
  obj.objectValue.push_back({"dayIssued", JsonValue::MakeNumber(c.dayIssued)});
  obj.objectValue.push_back({"dayDeadline", JsonValue::MakeNumber(c.dayDeadline)});
  obj.objectValue.push_back({"rewardMoney", JsonValue::MakeNumber(c.rewardMoney)});
  obj.objectValue.push_back({"startInt", JsonValue::MakeNumber(c.startInt)});
  obj.objectValue.push_back({"targetInt", JsonValue::MakeNumber(c.targetInt)});
  obj.objectValue.push_back({"stateInt", JsonValue::MakeNumber(c.stateInt)});
  obj.objectValue.push_back({"startF", JsonValue::MakeNumber(c.startF)});
  obj.objectValue.push_back({"targetF", JsonValue::MakeNumber(c.targetF)});
  obj.objectValue.push_back({"title", JsonValue::MakeString(c.title)});
  obj.objectValue.push_back({"description", JsonValue::MakeString(c.description)});
  return obj;
}

static bool CityChallengeFromJson(const JsonValue& v, CityChallenge& out, std::string& err)
{
  if (!v.isObject()) {
    err = "challenge entry must be an object";
    return false;
  }
  if (!JsonGetU32(v, "id", out.id, err, /*required=*/true)) return false;

  // kind: prefer string, accept numeric.
  {
    const JsonValue* k = FindJsonMember(v, "kind");
    if (!k) {
      err = "missing 'kind'";
      return false;
    }
    if (k->isString()) {
      CityChallengeKind kind = CityChallengeKind::GrowPopulation;
      if (!ChallengeKindFromString(k->stringValue, kind)) {
        err = "invalid challenge kind";
        return false;
      }
      out.kind = kind;
    } else if (k->isNumber()) {
      const double d = k->numberValue;
      if (!IsFinite(d) || std::floor(d) != d) {
        err = "kind must be integer";
        return false;
      }
      const int ki = static_cast<int>(d);
      switch (ki) {
        case 0: out.kind = CityChallengeKind::GrowPopulation; break;
        case 1: out.kind = CityChallengeKind::BuildParks; break;
        case 2: out.kind = CityChallengeKind::ReduceCongestion; break;
        case 3: out.kind = CityChallengeKind::ImproveGoods; break;
        case 4: out.kind = CityChallengeKind::ImproveServices; break;
        case 5: out.kind = CityChallengeKind::BalanceBudget; break;
        case 6: out.kind = CityChallengeKind::RestoreOutsideConnection; break;
        default:
          err = "invalid kind value";
          return false;
      }
    } else {
      err = "kind must be string or number";
      return false;
    }
  }

  // status: prefer string, accept numeric.
  {
    const JsonValue* s = FindJsonMember(v, "status");
    if (!s) {
      err = "missing 'status'";
      return false;
    }
    if (s->isString()) {
      CityChallengeStatus st = CityChallengeStatus::Active;
      if (!ChallengeStatusFromString(s->stringValue, st)) {
        err = "invalid challenge status";
        return false;
      }
      out.status = st;
    } else if (s->isNumber()) {
      const double d = s->numberValue;
      if (!IsFinite(d) || std::floor(d) != d) {
        err = "status must be integer";
        return false;
      }
      const int si = static_cast<int>(d);
      switch (si) {
        case 0: out.status = CityChallengeStatus::Active; break;
        case 1: out.status = CityChallengeStatus::Completed; break;
        case 2: out.status = CityChallengeStatus::Failed; break;
        case 3: out.status = CityChallengeStatus::Canceled; break;
        default:
          err = "invalid status value";
          return false;
      }
    } else {
      err = "status must be string or number";
      return false;
    }
  }

  if (!JsonGetInt(v, "dayIssued", out.dayIssued, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "dayDeadline", out.dayDeadline, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "rewardMoney", out.rewardMoney, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "startInt", out.startInt, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "targetInt", out.targetInt, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "stateInt", out.stateInt, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "startF", out.startF, err, /*required=*/true)) return false;
  if (!JsonGetFloat(v, "targetF", out.targetF, err, /*required=*/true)) return false;
  if (!JsonGetString(v, "title", out.title, err, /*required=*/true)) return false;
  if (!JsonGetString(v, "description", out.description, err, /*required=*/true)) return false;
  return true;
}

static JsonValue CityChallengeLogToJson(const CityChallengeLogEntry& e)
{
  JsonValue obj = JsonValue::MakeObject();
  obj.objectValue.push_back({"day", JsonValue::MakeNumber(e.day)});
  obj.objectValue.push_back({"status", JsonValue::MakeString(ChallengeStatusToString(e.status))});
  obj.objectValue.push_back({"rewardMoney", JsonValue::MakeNumber(e.rewardMoney)});
  obj.objectValue.push_back({"title", JsonValue::MakeString(e.title)});
  return obj;
}

static bool CityChallengeLogFromJson(const JsonValue& v, CityChallengeLogEntry& out, std::string& err)
{
  if (!v.isObject()) {
    err = "challenge log entry must be an object";
    return false;
  }
  if (!JsonGetInt(v, "day", out.day, err, /*required=*/true)) return false;

  // status
  {
    const JsonValue* s = FindJsonMember(v, "status");
    if (!s) {
      err = "missing 'status'";
      return false;
    }
    if (s->isString()) {
      CityChallengeStatus st = CityChallengeStatus::Completed;
      if (!ChallengeStatusFromString(s->stringValue, st)) {
        err = "invalid challenge status";
        return false;
      }
      out.status = st;
    } else if (s->isNumber()) {
      const double d = s->numberValue;
      if (!IsFinite(d) || std::floor(d) != d) {
        err = "status must be integer";
        return false;
      }
      const int si = static_cast<int>(d);
      switch (si) {
        case 0: out.status = CityChallengeStatus::Active; break;
        case 1: out.status = CityChallengeStatus::Completed; break;
        case 2: out.status = CityChallengeStatus::Failed; break;
        case 3: out.status = CityChallengeStatus::Canceled; break;
        default:
          err = "invalid status value";
          return false;
      }
    } else {
      err = "status must be string or number";
      return false;
    }
  }

  if (!JsonGetInt(v, "rewardMoney", out.rewardMoney, err, /*required=*/true)) return false;
  if (!JsonGetString(v, "title", out.title, err, /*required=*/true)) return false;
  return true;
}

static JsonValue CityMetaToJson(const CityMeta& meta)
{
  JsonValue root = JsonValue::MakeObject();
  root.objectValue.push_back({"version", JsonValue::MakeNumber(meta.version)});
  // Seed is stored as a string to preserve full 64-bit values (JSON numbers are doubles).
  root.objectValue.push_back({"seed", JsonValue::MakeString(std::to_string(meta.seed))});
  root.objectValue.push_back({"width", JsonValue::MakeNumber(meta.width)});
  root.objectValue.push_back({"height", JsonValue::MakeNumber(meta.height)});
  root.objectValue.push_back({"day", JsonValue::MakeNumber(meta.day)});

  root.objectValue.push_back({"historyMax", JsonValue::MakeNumber(meta.historyMax)});
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(meta.history.size());
    for (const CityHistorySample& s : meta.history) {
      arr.arrayValue.push_back(CityHistorySampleToJson(s));
    }
    root.objectValue.push_back({"history", std::move(arr)});
  }

  root.objectValue.push_back({"newsMax", JsonValue::MakeNumber(meta.newsMax)});
  root.objectValue.push_back({"newsSelection", JsonValue::MakeNumber(meta.newsSelection)});
  root.objectValue.push_back({"newsFirst", JsonValue::MakeNumber(meta.newsFirst)});
  root.objectValue.push_back({"mayorRatingEma", JsonValue::MakeNumber(meta.mayorRatingEma)});
  root.objectValue.push_back({"mayorRatingPrev", JsonValue::MakeNumber(meta.mayorRatingPrev)});
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(meta.news.size());
    for (const CityNewsEntry& e : meta.news) {
      arr.arrayValue.push_back(CityNewsEntryToJson(e));
    }
    root.objectValue.push_back({"news", std::move(arr)});
  }

  // Daily addendum (map<day, text>) stored as a stable list.
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(meta.newsAddendum.size());
    for (const auto& kv : meta.newsAddendum) {
      JsonValue o = JsonValue::MakeObject();
      o.objectValue.push_back({"day", JsonValue::MakeNumber(kv.first)});
      o.objectValue.push_back({"text", JsonValue::MakeString(kv.second)});
      arr.arrayValue.push_back(std::move(o));
    }
    root.objectValue.push_back({"newsAddendum", std::move(arr)});
  }

  root.objectValue.push_back({"challengeTargetActive", JsonValue::MakeNumber(meta.challengeTargetActive)});
  root.objectValue.push_back({"challengeRerolls", JsonValue::MakeNumber(meta.challengeRerolls)});
  root.objectValue.push_back({"challengeNextId", JsonValue::MakeNumber(meta.challengeNextId)});
  root.objectValue.push_back({"challengeLastProcessedDay", JsonValue::MakeNumber(meta.challengeLastProcessedDay)});
  root.objectValue.push_back({"challengeSelection", JsonValue::MakeNumber(meta.challengeSelection)});
  root.objectValue.push_back({"challengeFirst", JsonValue::MakeNumber(meta.challengeFirst)});
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(meta.challenges.size());
    for (const CityChallenge& c : meta.challenges) {
      arr.arrayValue.push_back(CityChallengeToJson(c));
    }
    root.objectValue.push_back({"challenges", std::move(arr)});
  }
  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(meta.challengeLog.size());
    for (const CityChallengeLogEntry& e : meta.challengeLog) {
      arr.arrayValue.push_back(CityChallengeLogToJson(e));
    }
    root.objectValue.push_back({"challengeLog", std::move(arr)});
  }
  return root;
}

static bool CityMetaFromJson(const JsonValue& v, CityMeta& out, std::string& err)
{
  if (!v.isObject()) {
    err = "CityMeta root must be an object";
    return false;
  }

  if (!JsonGetInt(v, "version", out.version, err, /*required=*/true)) return false;
  if (!JsonGetSeed(v, out.seed, err)) return false;
  if (!JsonGetInt(v, "width", out.width, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "height", out.height, err, /*required=*/true)) return false;
  if (!JsonGetInt(v, "day", out.day, err, /*required=*/true)) return false;

  // City report history.
  (void)JsonGetInt(v, "historyMax", out.historyMax, err, /*required=*/false);
  {
    const JsonValue* a = FindJsonMember(v, "history");
    if (a && a->isArray()) {
      if (a->arrayValue.size() > kMaxHistoryEntries) {
        err = "history array too large";
        return false;
      }
      out.history.clear();
      out.history.reserve(a->arrayValue.size());
      for (const JsonValue& it : a->arrayValue) {
        CityHistorySample s;
        if (!CityHistorySampleFromJson(it, s, err)) return false;
        out.history.push_back(std::move(s));
      }
    } else if (a) {
      err = "history must be an array";
      return false;
    }
  }

  // News.
  (void)JsonGetInt(v, "newsMax", out.newsMax, err, /*required=*/false);
  (void)JsonGetInt(v, "newsSelection", out.newsSelection, err, /*required=*/false);
  (void)JsonGetInt(v, "newsFirst", out.newsFirst, err, /*required=*/false);
  (void)JsonGetFloat(v, "mayorRatingEma", out.mayorRatingEma, err, /*required=*/false);
  (void)JsonGetFloat(v, "mayorRatingPrev", out.mayorRatingPrev, err, /*required=*/false);
  {
    const JsonValue* a = FindJsonMember(v, "news");
    if (a && a->isArray()) {
      if (a->arrayValue.size() > kMaxNewsEntries) {
        err = "news array too large";
        return false;
      }
      out.news.clear();
      for (const JsonValue& it : a->arrayValue) {
        CityNewsEntry e;
        if (!CityNewsEntryFromJson(it, e, err)) return false;
        out.news.push_back(std::move(e));
      }
    } else if (a) {
      err = "news must be an array";
      return false;
    }
  }

  // News addendum.
  {
    const JsonValue* a = FindJsonMember(v, "newsAddendum");
    if (a && a->isArray()) {
      if (a->arrayValue.size() > kMaxNewsAddendum) {
        err = "newsAddendum array too large";
        return false;
      }
      out.newsAddendum.clear();
      for (const JsonValue& it : a->arrayValue) {
        if (!it.isObject()) {
          err = "newsAddendum entry must be an object";
          return false;
        }
        int day = 0;
        std::string text;
        if (!JsonGetInt(it, "day", day, err, /*required=*/true)) return false;
        if (!JsonGetString(it, "text", text, err, /*required=*/true)) return false;
        out.newsAddendum[day] = std::move(text);
      }
    } else if (a) {
      err = "newsAddendum must be an array";
      return false;
    }
  }

  // Challenges.
  (void)JsonGetInt(v, "challengeTargetActive", out.challengeTargetActive, err, /*required=*/false);
  (void)JsonGetInt(v, "challengeRerolls", out.challengeRerolls, err, /*required=*/false);
  (void)JsonGetU32(v, "challengeNextId", out.challengeNextId, err, /*required=*/false);
  (void)JsonGetInt(v, "challengeLastProcessedDay", out.challengeLastProcessedDay, err, /*required=*/false);
  (void)JsonGetInt(v, "challengeSelection", out.challengeSelection, err, /*required=*/false);
  (void)JsonGetInt(v, "challengeFirst", out.challengeFirst, err, /*required=*/false);
  {
    const JsonValue* a = FindJsonMember(v, "challenges");
    if (a && a->isArray()) {
      if (a->arrayValue.size() > kMaxChallenges) {
        err = "challenges array too large";
        return false;
      }
      out.challenges.clear();
      out.challenges.reserve(a->arrayValue.size());
      for (const JsonValue& it : a->arrayValue) {
        CityChallenge c;
        if (!CityChallengeFromJson(it, c, err)) return false;
        out.challenges.push_back(std::move(c));
      }
    } else if (a) {
      err = "challenges must be an array";
      return false;
    }
  }
  {
    const JsonValue* a = FindJsonMember(v, "challengeLog");
    if (a && a->isArray()) {
      if (a->arrayValue.size() > kMaxChallengeLog) {
        err = "challengeLog array too large";
        return false;
      }
      out.challengeLog.clear();
      for (const JsonValue& it : a->arrayValue) {
        CityChallengeLogEntry e;
        if (!CityChallengeLogFromJson(it, e, err)) return false;
        out.challengeLog.push_back(std::move(e));
      }
    } else if (a) {
      err = "challengeLog must be an array";
      return false;
    }
  }

  return true;
}

} // namespace

bool SerializeCityMetaJson(const CityMeta& meta, std::string& outJson, std::string& outError)
{
  outError.clear();
  outJson.clear();

  const JsonValue root = CityMetaToJson(meta);
  std::ostringstream oss;
  JsonWriteOptions opt;
  opt.pretty = true;
  opt.indent = 2;
  opt.sortKeys = false;
  if (!WriteJson(oss, root, outError, opt)) return false;
  outJson = oss.str();
  return true;
}

bool DeserializeCityMetaJson(CityMeta& outMeta, const std::string& json, std::string& outError)
{
  outError.clear();

  JsonValue root;
  if (!ParseJson(json, root, outError)) return false;
  CityMeta meta;
  if (!CityMetaFromJson(root, meta, outError)) return false;
  outMeta = std::move(meta);
  return true;
}

std::string CityMetaPathForSavePath(const std::string& savePath)
{
  namespace fs = std::filesystem;
  fs::path p(savePath);

  // If invoked on a recovery path (e.g. *.tmp/*.bak), strip that suffix first.
  const std::string ext = p.extension().string();
  if (ext == ".tmp" || ext == ".bak") {
    p = p.stem();
  }

  // Strip the main save extension (typically .bin).
  p.replace_extension("");
  p += ".meta.json";
  return p.string();
}

bool SaveCityMetaForSavePath(const std::string& savePath, const CityMeta& meta, std::string& outError)
{
  outError.clear();
  namespace fs = std::filesystem;

  const fs::path outPath = fs::path(CityMetaPathForSavePath(savePath));
  fs::path tmpPath = outPath;
  tmpPath += ".tmp";
  fs::path bakPath = outPath;
  bakPath += ".bak";

  std::error_code ec;
  if (outPath.has_parent_path()) {
    fs::create_directories(outPath.parent_path(), ec);
  }

  // Remove stale tmp (best-effort).
  fs::remove(tmpPath, ec);

  const JsonValue root = CityMetaToJson(meta);
  {
    std::string err;
    JsonWriteOptions opt;
    opt.pretty = true;
    opt.indent = 2;
    opt.sortKeys = false;
    if (!WriteJsonFile(tmpPath.string(), root, err, opt)) {
      outError = err;
      return false;
    }
  }
  BestEffortSyncFile(tmpPath.string());

  // Move previous to .bak (best-effort), then commit.
  const bool hadOut = fs::exists(outPath, ec);
  if (hadOut) {
    fs::remove(bakPath, ec);
    fs::rename(outPath, bakPath, ec);
    // If rename fails, we still try to overwrite by continuing.
    ec.clear();
  }

  fs::rename(tmpPath, outPath, ec);
  if (ec) {
    // Try to roll back.
    if (hadOut) {
      std::error_code ec2;
      fs::rename(bakPath, outPath, ec2);
    }
    outError = "failed to rename tmp meta to final";
    return false;
  }

  if (outPath.has_parent_path()) BestEffortSyncDirectory(outPath.parent_path().string());
  return true;
}

bool LoadCityMetaJsonFile(const std::string& path, CityMeta& outMeta, std::string& outError)
{
  outError.clear();

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open: " + path;
    return false;
  }

  std::ostringstream oss;
  oss << f.rdbuf();
  const std::string text = oss.str();
  if (text.empty()) {
    outError = "empty meta file";
    return false;
  }

  return DeserializeCityMetaJson(outMeta, text, outError);
}

} // namespace isocity
