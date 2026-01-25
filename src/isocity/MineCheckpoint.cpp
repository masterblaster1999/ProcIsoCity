#include "isocity/MineCheckpoint.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace isocity {
namespace {

static bool ParseJsonObjectText(const std::string& text, JsonValue& outObj, std::string& outErr)
{
  JsonValue v;
  if (!ParseJson(text, v, outErr)) return false;
  if (!v.isObject()) {
    outErr = "expected JSON object";
    return false;
  }
  outObj = std::move(v);
  outErr.clear();
  return true;
}

static std::string CanonicalizeJsonObjectText(const std::string& text)
{
  JsonValue obj;
  std::string err;
  if (!ParseJsonObjectText(text, obj, err)) return std::string();
  JsonWriteOptions opt;
  opt.pretty = false;
  opt.sortKeys = true;
  return JsonStringify(obj, opt);
}

static bool WriteJsonLine(std::ostream& os, const JsonValue& v, std::string* outError)
{
  JsonWriteOptions opt;
  opt.pretty = false;
  opt.sortKeys = false;
  std::string err;
  if (!WriteJson(os, v, err, opt)) {
    if (outError) *outError = err;
    return false;
  }
  os << "\n";
  if (!os) {
    if (outError) *outError = "stream write failure";
    return false;
  }
  if (outError) outError->clear();
  return true;
}

static void AddMember(JsonValue& obj, const char* key, JsonValue v)
{
  obj.objectValue.emplace_back(key, std::move(v));
}

static bool ReadBool(const JsonValue* v, bool& out)
{
  if (!v) return false;
  if (!v->isBool()) return false;
  out = v->boolValue;
  return true;
}

static bool ReadNumberI32(const JsonValue* v, int& out)
{
  if (!v || !v->isNumber()) return false;
  const double d = v->numberValue;
  if (!std::isfinite(d)) return false;
  if (d < static_cast<double>(std::numeric_limits<int>::min()) || d > static_cast<double>(std::numeric_limits<int>::max())) return false;
  out = static_cast<int>(std::llround(d));
  return true;
}

static bool ReadNumberU64(const JsonValue* v, std::uint64_t& out)
{
  if (!v || !v->isNumber()) return false;
  const double d = v->numberValue;
  if (!std::isfinite(d)) return false;
  if (d < 0.0 || d > 18446744073709551615.0) return false;
  out = static_cast<std::uint64_t>(std::llround(d));
  return true;
}

static bool ReadNumberF32(const JsonValue* v, float& out)
{
  if (!v || !v->isNumber()) return false;
  if (!std::isfinite(v->numberValue)) return false;
  const double d = v->numberValue;
  if (d < -1.0e20 || d > 1.0e20) return false;
  out = static_cast<float>(d);
  return true;
}

static bool ReadString(const JsonValue* v, std::string& out)
{
  if (!v || !v->isString()) return false;
  out = v->stringValue;
  return true;
}

static bool ParseU64Text(const std::string& s, std::uint64_t& out)
{
  try {
    std::size_t idx = 0;
    unsigned long long v = std::stoull(s, &idx, 0);
    if (idx != s.size()) return false;
    out = static_cast<std::uint64_t>(v);
    return true;
  } catch (...) {
    return false;
  }
}

static bool EqualNaNFloat(float a, float b)
{
  const bool an = !std::isfinite(a);
  const bool bn = !std::isfinite(b);
  if (an && bn) return true;
  return a == b;
}

static bool ParseMineConfigJson(const JsonValue& obj, MineConfig& outCfg, std::string& outErr)
{
  if (!obj.isObject()) {
    outErr = "mine config: expected object";
    return false;
  }

  MineConfig c;

  // seedStart/seedStep: prefer hex strings to avoid JSON number precision loss for large u64 values.
  {
    std::string tmp;
    std::uint64_t v = 0;
    if (ReadString(FindJsonMember(obj, "seedStartHex"), tmp) && ParseU64Text(tmp, v)) {
      c.seedStart = v;
    } else {
      ReadNumberU64(FindJsonMember(obj, "seedStart"), c.seedStart);
    }

    if (ReadString(FindJsonMember(obj, "seedStepHex"), tmp) && ParseU64Text(tmp, v)) {
      c.seedStep = v;
    } else {
      ReadNumberU64(FindJsonMember(obj, "seedStep"), c.seedStep);
    }

    if (ReadString(FindJsonMember(obj, "seedXorHex"), tmp) && ParseU64Text(tmp, v)) {
      c.seedXor = v;
    } else if (ReadNumberU64(FindJsonMember(obj, "seedXor"), v)) {
      c.seedXor = v;
    }
  }

  // Optional sampler mode. Default is Linear.
  {
    std::string sampler;
    if (ReadString(FindJsonMember(obj, "seedSampler"), sampler) ||
        ReadString(FindJsonMember(obj, "seed_sampler"), sampler)) {
      MineSeedSampler sm;
      if (!ParseMineSeedSampler(sampler, sm)) {
        outErr = "mine config: invalid seedSampler";
        return false;
      }
      c.seedSampler = sm;
    } else {
      int smId = static_cast<int>(c.seedSampler);
      if (ReadNumberI32(FindJsonMember(obj, "seedSamplerId"), smId)) {
        if (smId < 0 || smId > 255) {
          outErr = "mine config: invalid seedSamplerId";
          return false;
        }
        c.seedSampler = static_cast<MineSeedSampler>(static_cast<std::uint8_t>(smId));
      }
    }
  }

  ReadNumberI32(FindJsonMember(obj, "samples"), c.samples);
  ReadNumberI32(FindJsonMember(obj, "w"), c.w);
  ReadNumberI32(FindJsonMember(obj, "h"), c.h);
  ReadNumberI32(FindJsonMember(obj, "days"), c.days);

  int objv = static_cast<int>(c.objective);
  if (ReadNumberI32(FindJsonMember(obj, "objective"), objv)) {
    if (objv < 0 || objv > 255) {
      outErr = "mine config: invalid objective";
      return false;
    }
    c.objective = static_cast<MineObjective>(static_cast<std::uint8_t>(objv));
  }

  // Optional custom score expression.
  ReadString(FindJsonMember(obj, "scoreExpr"), c.scoreExpr);
  if (c.scoreExpr.empty()) ReadString(FindJsonMember(obj, "score_expr"), c.scoreExpr);

  ReadBool(FindJsonMember(obj, "hydrologyEnabled"), c.hydrologyEnabled);

  float f = c.seaLevelOverride;
  if (ReadNumberF32(FindJsonMember(obj, "seaLevelOverride"), f)) c.seaLevelOverride = f;

  ReadBool(FindJsonMember(obj, "seaRequireEdgeConnection"), c.seaRequireEdgeConnection);
  ReadBool(FindJsonMember(obj, "seaEightConnected"), c.seaEightConnected);
  ReadNumberF32(FindJsonMember(obj, "depressionEpsilon"), c.depressionEpsilon);

  // threads intentionally omitted from checkpoint comparisons, but we keep it in the header for diagnostics.
  ReadNumberI32(FindJsonMember(obj, "threads"), c.threads);

  outCfg = c;
  outErr.clear();
  return true;
}

static JsonValue MineConfigToJsonObject(const MineConfig& c)
{
  JsonValue obj = JsonValue::MakeObject();
  AddMember(obj, "seedStart", JsonValue::MakeNumber(static_cast<double>(c.seedStart)));
  AddMember(obj, "seedStartHex", JsonValue::MakeString(HexU64(c.seedStart)));
  AddMember(obj, "seedStep", JsonValue::MakeNumber(static_cast<double>(c.seedStep)));
  AddMember(obj, "seedStepHex", JsonValue::MakeString(HexU64(c.seedStep)));
  AddMember(obj, "seedSampler", JsonValue::MakeString(MineSeedSamplerName(c.seedSampler)));
  AddMember(obj, "seedXor", JsonValue::MakeNumber(static_cast<double>(c.seedXor)));
  AddMember(obj, "seedXorHex", JsonValue::MakeString(HexU64(c.seedXor)));
  AddMember(obj, "samples", JsonValue::MakeNumber(static_cast<double>(c.samples)));
  AddMember(obj, "w", JsonValue::MakeNumber(static_cast<double>(c.w)));
  AddMember(obj, "h", JsonValue::MakeNumber(static_cast<double>(c.h)));
  AddMember(obj, "days", JsonValue::MakeNumber(static_cast<double>(c.days)));
  AddMember(obj, "threads", JsonValue::MakeNumber(static_cast<double>(c.threads)));
  AddMember(obj, "objective", JsonValue::MakeNumber(static_cast<double>(static_cast<int>(c.objective))));
  AddMember(obj, "scoreExpr", JsonValue::MakeString(c.scoreExpr));
  AddMember(obj, "hydrologyEnabled", JsonValue::MakeBool(c.hydrologyEnabled));
  if (std::isfinite(c.seaLevelOverride)) {
    AddMember(obj, "seaLevelOverride", JsonValue::MakeNumber(static_cast<double>(c.seaLevelOverride)));
  } else {
    AddMember(obj, "seaLevelOverride", JsonValue::MakeNull());
  }
  AddMember(obj, "seaRequireEdgeConnection", JsonValue::MakeBool(c.seaRequireEdgeConnection));
  AddMember(obj, "seaEightConnected", JsonValue::MakeBool(c.seaEightConnected));
  AddMember(obj, "depressionEpsilon", JsonValue::MakeNumber(static_cast<double>(c.depressionEpsilon)));
  return obj;
}

static bool ParseLine(const std::string& line, JsonValue& outObj, std::string& outErr)
{
  std::string s = line;
  // Trim whitespace.
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  if (b <= a) {
    outObj = JsonValue::MakeNull();
    outErr.clear();
    return true;
  }

  JsonValue v;
  if (!ParseJson(s.substr(a, b - a), v, outErr)) return false;
  if (!v.isObject()) {
    outErr = "line is not a JSON object";
    return false;
  }
  outObj = std::move(v);
  outErr.clear();
  return true;
}

} // namespace

bool WriteMineCheckpointHeader(std::ostream& os, const MineCheckpointHeader& h, std::string* outError)
{
  JsonValue root = JsonValue::MakeObject();
  AddMember(root, "type", JsonValue::MakeString("procisocity_mine_checkpoint"));
  AddMember(root, "version", JsonValue::MakeNumber(static_cast<double>(h.version)));
  AddMember(root, "mine", MineConfigToJsonObject(h.mineCfg));

  // Embed proc/sim configs as full JSON objects.
  JsonValue procObj;
  JsonValue simObj;
  {
    std::string err;
    if (!ParseJsonObjectText(ProcGenConfigToJson(h.procCfg, 0), procObj, err)) {
      if (outError) *outError = "proc config serialize/parse failed: " + err;
      return false;
    }
    if (!ParseJsonObjectText(SimConfigToJson(h.simCfg, 0), simObj, err)) {
      if (outError) *outError = "sim config serialize/parse failed: " + err;
      return false;
    }
  }

  AddMember(root, "proc", std::move(procObj));
  AddMember(root, "sim", std::move(simObj));

  return WriteJsonLine(os, root, outError);
}

bool AppendMineCheckpointRecord(std::ostream& os, int index, const MineRecord& r, std::string* outError)
{
  JsonValue root = JsonValue::MakeObject();
  AddMember(root, "type", JsonValue::MakeString("record"));
  AddMember(root, "index", JsonValue::MakeNumber(static_cast<double>(index)));
  AddMember(root, "record", MineRecordToJson(r));
  return WriteJsonLine(os, root, outError);
}

bool LoadMineCheckpointFile(const std::string& path,
                            MineCheckpointHeader* outHeader,
                            std::vector<MineRecord>& outRecords,
                            std::vector<bool>* outHaveIndex,
                            std::string* outError)
{
  outRecords.clear();
  if (outHaveIndex) outHaveIndex->clear();

  std::ifstream is(path, std::ios::binary);
  if (!is) {
    if (outError) *outError = "failed to open checkpoint";
    return false;
  }

  std::string line;
  int lineNo = 0;
  MineCheckpointHeader hdr;
  bool haveHeader = false;

  int maxIndex = -1;
  struct Item {
    int index = 0;
    MineRecord rec;
  };
  std::vector<Item> items;

  while (std::getline(is, line)) {
    JsonValue obj;
    std::string err;
    if (!ParseLine(line, obj, err)) {
      if (outError) *outError = "checkpoint parse error at line " + std::to_string(lineNo + 1) + ": " + err;
      return false;
    }

    ++lineNo;
    if (obj.isNull()) continue;

    std::string type;
    if (!ReadString(FindJsonMember(obj, "type"), type)) {
      if (outError) *outError = "checkpoint missing 'type' at line " + std::to_string(lineNo);
      return false;
    }

    if (!haveHeader) {
      if (type != "procisocity_mine_checkpoint") {
        if (outError) *outError = "checkpoint first non-empty line must be a header";
        return false;
      }

      int v = hdr.version;
      if (ReadNumberI32(FindJsonMember(obj, "version"), v)) hdr.version = v;

      const JsonValue* mine = FindJsonMember(obj, "mine");
      if (!mine || !mine->isObject()) {
        if (outError) *outError = "checkpoint header missing mine object";
        return false;
      }

      if (!ParseMineConfigJson(*mine, hdr.mineCfg, err)) {
        if (outError) *outError = "checkpoint header mine parse failed: " + err;
        return false;
      }

      // Load full proc/sim configs by applying the serialized full objects onto defaults.
      hdr.procCfg = ProcGenConfig{};
      hdr.simCfg = SimConfig{};

      const JsonValue* proc = FindJsonMember(obj, "proc");
      const JsonValue* sim = FindJsonMember(obj, "sim");
      if (!proc || !proc->isObject()) {
        if (outError) *outError = "checkpoint header missing proc object";
        return false;
      }
      if (!sim || !sim->isObject()) {
        if (outError) *outError = "checkpoint header missing sim object";
        return false;
      }

      if (!ApplyProcGenConfigJson(*proc, hdr.procCfg, err)) {
        if (outError) *outError = "checkpoint header proc parse failed: " + err;
        return false;
      }
      if (!ApplySimConfigJson(*sim, hdr.simCfg, err)) {
        if (outError) *outError = "checkpoint header sim parse failed: " + err;
        return false;
      }

      haveHeader = true;
      continue;
    }

    if (type != "record") continue;

    int idx = -1;
    if (!ReadNumberI32(FindJsonMember(obj, "index"), idx) || idx < 0) {
      if (outError) *outError = "checkpoint record missing/invalid index at line " + std::to_string(lineNo);
      return false;
    }

    const JsonValue* recObj = FindJsonMember(obj, "record");
    if (!recObj || !recObj->isObject()) {
      if (outError) *outError = "checkpoint record missing record object at line " + std::to_string(lineNo);
      return false;
    }

    MineRecord rec;
    if (!MineRecordFromJson(*recObj, rec, &err)) {
      if (outError) *outError = "checkpoint record parse failed at line " + std::to_string(lineNo) + ": " + err;
      return false;
    }

    items.push_back(Item{idx, std::move(rec)});
    maxIndex = std::max(maxIndex, idx);
  }

  if (!haveHeader) {
    if (outError) *outError = "checkpoint missing header";
    return false;
  }

  // Sort and fill output arrays.
  std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
    return a.index < b.index;
  });

  outRecords.resize(static_cast<std::size_t>(maxIndex + 1));
  if (outHaveIndex) outHaveIndex->assign(static_cast<std::size_t>(maxIndex + 1), false);

  for (const Item& it : items) {
    if (it.index < 0) continue;
    if (it.index > maxIndex) continue;
    outRecords[static_cast<std::size_t>(it.index)] = it.rec;
    if (outHaveIndex) (*outHaveIndex)[static_cast<std::size_t>(it.index)] = true;
  }

  if (outHeader) *outHeader = std::move(hdr);
  if (outError) outError->clear();
  return true;
}

bool MineCheckpointConfigsMatch(const MineCheckpointHeader& a, const MineCheckpointHeader& b, std::string* outWhy)
{
  auto why = [&](std::string s) {
    if (outWhy) *outWhy = std::move(s);
    return false;
  };

  const MineConfig& x = a.mineCfg;
  const MineConfig& y = b.mineCfg;

  if (x.seedStart != y.seedStart) return why("seedStart differs");
  if (x.seedStep != y.seedStep) return why("seedStep differs");
  if (x.seedSampler != y.seedSampler) return why("seedSampler differs");
  if (x.seedXor != y.seedXor) return why("seedXor differs");
  if (x.w != y.w || x.h != y.h) return why("size differs");
  if (x.days != y.days) return why("days differs");
  if (x.samples != y.samples) return why("samples differs");
  if (x.objective != y.objective) return why("objective differs");
  if (x.scoreExpr != y.scoreExpr) return why("scoreExpr differs");
  if (x.hydrologyEnabled != y.hydrologyEnabled) return why("hydrologyEnabled differs");
  if (x.seaRequireEdgeConnection != y.seaRequireEdgeConnection) return why("sea edge setting differs");
  if (x.seaEightConnected != y.seaEightConnected) return why("sea connectivity differs");
  if (!EqualNaNFloat(x.seaLevelOverride, y.seaLevelOverride)) return why("seaLevelOverride differs");
  if (x.depressionEpsilon != y.depressionEpsilon) return why("depressionEpsilon differs");

  // Compare canonical config JSON strings.
  const std::string ap = CanonicalizeJsonObjectText(ProcGenConfigToJson(a.procCfg, 0));
  const std::string bp = CanonicalizeJsonObjectText(ProcGenConfigToJson(b.procCfg, 0));
  if (ap.empty() || bp.empty()) return why("failed to canonicalize proc config");
  if (ap != bp) return why("proc config differs");

  const std::string as = CanonicalizeJsonObjectText(SimConfigToJson(a.simCfg, 0));
  const std::string bs = CanonicalizeJsonObjectText(SimConfigToJson(b.simCfg, 0));
  if (as.empty() || bs.empty()) return why("failed to canonicalize sim config");
  if (as != bs) return why("sim config differs");

  if (outWhy) outWhy->clear();
  return true;
}

} // namespace isocity
