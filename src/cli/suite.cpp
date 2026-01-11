#include "isocity/Suite.hpp"

#include "isocity/Export.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;

std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string Trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  *out = static_cast<int>(v);
  return true;
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  int base = 10;
  std::size_t offset = 0;
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    base = 16;
    offset = 2;
  }

  char* end = nullptr;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

[[maybe_unused]] bool ParseU8(const std::string& s, std::uint8_t* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v < 0 || v > 255) return false;
  *out = static_cast<std::uint8_t>(v);
  return true;
}

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string::npos) return false;

  int w = 0, h = 0;
  if (!ParseI32(s.substr(0, pos), &w)) return false;
  if (!ParseI32(s.substr(pos + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

bool ParseShard(const std::string& s, int* outIndex, int* outCount)
{
  if (!outIndex || !outCount) return false;
  const std::size_t pos = s.find('/');
  if (pos == std::string::npos) return false;
  int i = 0, n = 0;
  if (!ParseI32(s.substr(0, pos), &i)) return false;
  if (!ParseI32(s.substr(pos + 1), &n)) return false;
  if (n <= 0) return false;
  if (i < 0 || i >= n) return false;
  *outIndex = i;
  *outCount = n;
  return true;
}

std::string SanitizeName(const std::string& s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) out.push_back(static_cast<char>(uc));
    else if (c == '-' || c == '_' || c == '.') out.push_back(c);
    else out.push_back('_');
  }
  // Avoid empty.
  if (out.empty()) out = "case";
  return out;
}

std::string EscapeXml(const std::string& s)
{
  std::ostringstream oss;
  for (char c : s) {
    switch (c) {
      case '&': oss << "&amp;"; break;
      case '<': oss << "&lt;"; break;
      case '>': oss << "&gt;"; break;
      case '\"': oss << "&quot;"; break;
      case '\'': oss << "&apos;"; break;
      default: oss << c; break;
    }
  }
  return oss.str();
}

const char* KindName(isocity::ScenarioKind k)
{
  return (k == isocity::ScenarioKind::Replay) ? "replay" : "script";
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_suite (scenario suite runner)\n\n"
      << "Usage:\n"
      << "  proc_isocity_suite [options] <case1> <case2> ...\n\n"
      << "Inputs may be:\n"
      << "  - a script file (any extension; typically .isocity)\n"
      << "  - a replay file (.isoreplay)\n\n"
      << "Options:\n"
      << "  --manifest <file>           Load scenarios from a manifest file (repeatable).\n"
      << "  --discover <dir>            Recursively discover scenarios in a directory (repeatable).\n"
      << "  --ext <ext>                 Extension filter for --discover (repeatable). Default: .isocity and .isoreplay\n"
      << "  --shard <i>/<n>              Run only shard i of n (0-based).\n"
      << "  --jobs <N>                  Run up to N cases in parallel (0 = auto). Default: 1\n"
      << "  --define k=v                Inject a script variable (repeatable).\n"
      << "  --ignore-replay-asserts      Ignore AssertHash events when running replays.\n"
      << "  --lax-replay-patches         Do not require patch base hashes to match when playing replays.\n"
      << "  --out-dir <dir>              Write per-case artifacts (summary.json, final.bin, ticks.csv).\n"
      << "  --json-report <file>         Write a suite summary JSON.\n"
      << "  --junit <file>               Write a JUnit XML report (useful for CI).\n"
      << "  --html-report <file>         Write an HTML dashboard (links to artifacts + golden previews).\n"
      << "  --html-title <title>         Title string for the HTML report (optional).\n"
      << "  --fail-fast                  Stop on first failure.\n"
      << "  --verbose                    Print script output (default is quiet).\n"
      << "  --timing                     Print per-case and total timing information.\n"
      << "\n"
      << "Golden image regression (snapshot testing):\n"
      << "  --golden                     Compare a rendered image (PPM/PNG) against a golden snapshot.\n"
      << "  --update-golden              Create/overwrite golden snapshots instead of failing.\n"
      << "  --golden-dir <dir>            Base directory for goldens (default: next to scenario file).\n"
      << "  --golden-ext <ppm|png>        Golden image file extension. Default: ppm\n"
      << "  --golden-format <top|iso>     Render format for goldens. Default: top\n"
      << "  --golden-layer <layer>        Layer to render. Default: overlay\n"
      << "  --golden-scale <N>            Scale factor for top-down golden renders. Default: 1\n"
      << "  --golden-threshold <N>        Per-channel tolerance (0..255). Default: 0\n"
      << "  --golden-iso-tile <WxH>       Iso tile size. Default: 16x8\n"
      << "  --golden-iso-height <N>       Iso height scale (px). Default: 14\n"
      << "  --golden-iso-margin <N>       Iso margin (px). Default: 8\n"
      << "  --golden-iso-grid <0|1>       Iso draw grid lines. Default: 0\n"
      << "  --golden-iso-cliffs <0|1>     Iso draw cliffs. Default: 1\n"
      << "\n"
      << "Golden hash regression (world state snapshot testing):\n"
      << "  --hash-golden                Compare final world hash against a per-scenario golden hash sidecar.\n"
      << "  --update-hash-golden         Create/overwrite golden hash files instead of failing.\n"
      << "  --hash-golden-dir <dir>      Base directory for hash goldens (default: next to scenario file).\n"
      << "  --hash-golden-ext <ext>      Golden hash extension (default: hash).\n"
      << "\n"
      << "  --help                       Show this help.\n";
}

struct GoldenConfig {
  bool enabled = false;
  bool update = false;

  bool iso = false;
  isocity::ExportLayer layer = isocity::ExportLayer::Overlay;
  int scale = 1;      // top-down only
  int threshold = 0;  // per-channel abs threshold (0 = exact)

  // If empty, snapshots are written next to the scenario file.
  std::string goldenDir;

  // Golden image extension ("ppm" or "png").
  std::string goldenExt = "ppm";

  isocity::IsoOverviewConfig isoCfg{};
};

struct GoldenResult {
  bool attempted = false;
  bool ok = true;
  bool updated = false;
  bool matched = true;
  std::string goldenPath;
  std::string error;
  isocity::PpmDiffStats stats{};
  bool hasStats = false;
};

struct HashGoldenConfig {
  bool enabled = false;
  bool update = false;

  // If empty, snapshots are written next to the scenario file.
  std::string dir;

  // Extension for the golden hash sidecar. Default: "hash" (written as <stem>.golden.hash).
  std::string ext = "hash";
};

struct HashGoldenResult {
  bool attempted = false;
  bool ok = true;
  bool updated = false;
  bool matched = true;

  std::string path;
  std::string error;

  std::uint64_t expected = 0;
  bool hasExpected = false;
};

struct CaseResult {
  isocity::ScenarioCase sc;
  bool ok = false;
  std::string error;
  std::uint64_t hash = 0;

  // When --out-dir is provided and the case ran successfully, we write per-case artifacts
  // and store the directory here so reports can link to them.
  std::string artifactsDir;

  // Wall-clock time spent running this case (including golden compare/update and artifact writes).
  double seconds = 0.0;

  // Non-fatal warnings encountered while producing artifacts.
  std::vector<std::string> warnings;

  GoldenResult golden;

  HashGoldenResult hashGolden;
};

std::string FormatPpmDiffSummary(const isocity::PpmDiffStats& st)
{
  std::ostringstream oss;
  oss << st.pixelsDifferent << " pixels differ"
      << " (maxAbsDiff=" << static_cast<int>(st.maxAbsDiff)
      << ", meanAbsDiff=" << std::fixed << std::setprecision(6) << st.meanAbsDiff
      << ", mse=" << std::fixed << std::setprecision(6) << st.mse;
  if (std::isinf(st.psnr)) {
    oss << ", psnr=inf";
  } else {
    oss << ", psnr=" << std::fixed << std::setprecision(3) << st.psnr << "dB";
  }
  oss << ")";
  return oss.str();
}

fs::path ComputeGoldenPath(const isocity::ScenarioCase& sc, const GoldenConfig& g)
{
  fs::path base;

  if (!g.goldenDir.empty()) {
    fs::path rel = fs::path(sc.path);
    if (rel.is_absolute()) rel = rel.filename();
    rel.replace_extension(); // remove .isocity / .isoreplay etc
    base = fs::path(g.goldenDir) / rel;
  } else {
    // Sidecar next to scenario file.
    fs::path p(sc.path);
    base = p.parent_path() / p.stem();
  }

  std::ostringstream suffix;
  suffix << ".golden";
  if (g.iso) suffix << ".iso";
  {
    std::string ext = g.goldenExt;
    if (!ext.empty() && ext[0] == '.') ext = ext.substr(1);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (ext.empty()) ext = "ppm";
    suffix << "." << isocity::ExportLayerName(g.layer) << "." << ext;
  }

  fs::path out = base;
  out += suffix.str();
  return out;
}

fs::path ComputeHashGoldenPath(const isocity::ScenarioCase& sc, const HashGoldenConfig& g)
{
  fs::path base;

  if (!g.dir.empty()) {
    fs::path rel = fs::path(sc.path);
    if (rel.is_absolute()) rel = rel.filename();
    rel.replace_extension();
    base = fs::path(g.dir) / rel;
  } else {
    // Sidecar next to scenario file.
    fs::path p(sc.path);
    base = p.parent_path() / p.stem();
  }

  std::string ext = g.ext;
  if (!ext.empty() && ext[0] == '.') ext = ext.substr(1);
  ext = ToLower(ext);
  if (ext.empty()) ext = "hash";

  fs::path out = base;
  out += ".golden." + ext;
  return out;
}

bool RenderGoldenImage(const isocity::ScenarioRunOutputs& out, const GoldenConfig& g, isocity::PpmImage& outImg,
                       std::string& outErr)
{
  outErr.clear();
  outImg = isocity::PpmImage{};

  if (g.iso) {
    const isocity::IsoOverviewResult iso = isocity::RenderIsoOverview(out.world, g.layer, g.isoCfg);
    outImg = iso.image;
    if (outImg.width <= 0 || outImg.height <= 0 || outImg.rgb.empty()) {
      outErr = "RenderIsoOverview produced an empty image";
      return false;
    }
    return true;
  }

  isocity::PpmImage img = isocity::RenderPpmLayer(out.world, g.layer);
  if (img.width <= 0 || img.height <= 0 || img.rgb.empty()) {
    outErr = "RenderPpmLayer produced an empty image";
    return false;
  }
  if (g.scale > 1) img = isocity::ScaleNearest(img, g.scale);
  outImg = std::move(img);
  return true;
}

bool WriteGoldenArtifacts(const fs::path& caseDir, const GoldenConfig& cfg, const GoldenResult& res,
                          const isocity::PpmImage& actual, const isocity::PpmImage* expectedImg,
                          const isocity::PpmImage* diffImg, std::string& outErr)
{
  outErr.clear();

  std::string ext = cfg.goldenExt;
  if (!ext.empty() && ext[0] == '.') ext = ext.substr(1);
  ext = ToLower(ext);
  if (ext.empty()) ext = "ppm";

  const fs::path expectedPath = caseDir / (std::string("golden_expected.") + ext);
  const fs::path actualPath = caseDir / (std::string("golden_actual.") + ext);
  const fs::path diffPath = caseDir / (std::string("golden_diff.") + ext);

  // Always write PNG previews so the HTML report can embed images even when the
  // main artifacts are written as PPM.
  const fs::path expectedPreviewPath = caseDir / "golden_expected_preview.png";
  const fs::path actualPreviewPath = caseDir / "golden_actual_preview.png";
  const fs::path diffPreviewPath = caseDir / "golden_diff_preview.png";

  auto writePreview = [&](const fs::path& p, const isocity::PpmImage& img) -> bool {
    std::string err;
    if (!isocity::WriteImageAuto(p.string(), img, err)) {
      outErr = "failed to write preview: " + p.string() + " (" + err + ")";
      return false;
    }
    return true;
  };

  // golden_expected
  if (expectedImg && !expectedImg->rgb.empty() && expectedImg->width > 0 && expectedImg->height > 0) {
    std::string err;
    if (!isocity::WriteImageAuto(expectedPath.string(), *expectedImg, err)) {
      outErr = "failed to write golden_expected: " + err;
      return false;
    }
    if (!writePreview(expectedPreviewPath, *expectedImg)) return false;
  }

  // golden_actual
  {
    std::string err;
    if (!isocity::WriteImageAuto(actualPath.string(), actual, err)) {
      outErr = "failed to write golden_actual: " + err;
      return false;
    }
  }

  if (!writePreview(actualPreviewPath, actual)) return false;

  // golden_diff
  if (diffImg && !diffImg->rgb.empty() && diffImg->width > 0 && diffImg->height > 0) {
    std::string err;
    if (!isocity::WriteImageAuto(diffPath.string(), *diffImg, err)) {
      outErr = "failed to write golden_diff: " + err;
      return false;
    }
    if (!writePreview(diffPreviewPath, *diffImg)) return false;
  }

  // golden.json
  {
    using isocity::JsonValue;
    JsonValue root = JsonValue::MakeObject();
    auto add = [](JsonValue& obj, const char* key, JsonValue v) {
      obj.objectValue.emplace_back(key, std::move(v));
    };

    add(root, "enabled", JsonValue::MakeBool(cfg.enabled));
    add(root, "updateMode", JsonValue::MakeBool(cfg.update));
    add(root, "format", JsonValue::MakeString(cfg.iso ? "iso" : "top"));
    add(root, "layer", JsonValue::MakeString(isocity::ExportLayerName(cfg.layer)));
    add(root, "scale", JsonValue::MakeNumber(static_cast<double>(cfg.scale)));
    add(root, "threshold", JsonValue::MakeNumber(static_cast<double>(cfg.threshold)));
    add(root, "goldenPath", JsonValue::MakeString(res.goldenPath));
    add(root, "attempted", JsonValue::MakeBool(res.attempted));
    add(root, "ok", JsonValue::MakeBool(res.ok));
    add(root, "updated", JsonValue::MakeBool(res.updated));
    add(root, "matched", JsonValue::MakeBool(res.matched));
    add(root, "error", JsonValue::MakeString(res.error));

    if (res.hasStats) {
      JsonValue st = JsonValue::MakeObject();
      add(st, "width", JsonValue::MakeNumber(static_cast<double>(res.stats.width)));
      add(st, "height", JsonValue::MakeNumber(static_cast<double>(res.stats.height)));
      add(st, "pixelsCompared", JsonValue::MakeNumber(static_cast<double>(res.stats.pixelsCompared)));
      add(st, "pixelsDifferent", JsonValue::MakeNumber(static_cast<double>(res.stats.pixelsDifferent)));
      add(st, "maxAbsDiff", JsonValue::MakeNumber(static_cast<double>(res.stats.maxAbsDiff)));
      add(st, "meanAbsDiff", JsonValue::MakeNumber(static_cast<double>(res.stats.meanAbsDiff)));
      add(st, "mse", JsonValue::MakeNumber(static_cast<double>(res.stats.mse)));
      if (std::isinf(res.stats.psnr)) {
        add(st, "psnr", JsonValue::MakeNull());
        add(st, "psnrIsInf", JsonValue::MakeBool(true));
      } else {
        add(st, "psnr", JsonValue::MakeNumber(static_cast<double>(res.stats.psnr)));
        add(st, "psnrIsInf", JsonValue::MakeBool(false));
      }
      add(root, "stats", std::move(st));
    }

    const std::string outPath = (caseDir / "golden.json").string();
    std::string jsonErr;
    if (!isocity::WriteJsonFile(outPath, root, jsonErr, isocity::JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false})) {
      outErr = "failed to write golden.json: " + jsonErr;
      return false;
    }
  }

  return true;
}

bool WriteCaseArtifacts(const fs::path& caseDir, const isocity::ScenarioRunOutputs& out, const isocity::ScenarioCase& sc,
                        std::string& outErr)
{
  outErr.clear();
  std::error_code ec;
  fs::create_directories(caseDir, ec);
  if (ec) {
    outErr = "failed to create out dir: " + caseDir.string() + " (" + ec.message() + ")";
    return false;
  }

  // summary.json
  {
    const isocity::Stats& s = out.world.stats();

    using isocity::JsonValue;
    JsonValue root = JsonValue::MakeObject();
    auto add = [](JsonValue& obj, const char* key, JsonValue v) {
      obj.objectValue.emplace_back(key, std::move(v));
    };

    JsonValue caseObj = JsonValue::MakeObject();
    add(caseObj, "path", JsonValue::MakeString(sc.path));
    add(caseObj, "kind", JsonValue::MakeString(KindName(sc.kind)));
    add(root, "case", std::move(caseObj));

    add(root, "width", JsonValue::MakeNumber(static_cast<double>(out.world.width())));
    add(root, "height", JsonValue::MakeNumber(static_cast<double>(out.world.height())));
    add(root, "seed", JsonValue::MakeNumber(static_cast<double>(out.world.seed())));
    add(root, "hash", JsonValue::MakeString(HexU64(out.finalHash)));

    JsonValue st = JsonValue::MakeObject();
    add(st, "day", JsonValue::MakeNumber(static_cast<double>(s.day)));
    add(st, "population", JsonValue::MakeNumber(static_cast<double>(s.population)));
    add(st, "housingCapacity", JsonValue::MakeNumber(static_cast<double>(s.housingCapacity)));
    add(st, "jobsCapacity", JsonValue::MakeNumber(static_cast<double>(s.jobsCapacity)));
    add(st, "jobsCapacityAccessible", JsonValue::MakeNumber(static_cast<double>(s.jobsCapacityAccessible)));
    add(st, "employed", JsonValue::MakeNumber(static_cast<double>(s.employed)));
    add(st, "happiness", JsonValue::MakeNumber(static_cast<double>(s.happiness)));
    add(st, "money", JsonValue::MakeNumber(static_cast<double>(s.money)));
    add(st, "roads", JsonValue::MakeNumber(static_cast<double>(s.roads)));
    add(st, "parks", JsonValue::MakeNumber(static_cast<double>(s.parks)));
    add(st, "avgCommuteTime", JsonValue::MakeNumber(static_cast<double>(s.avgCommuteTime)));
    add(st, "trafficCongestion", JsonValue::MakeNumber(static_cast<double>(s.trafficCongestion)));
    add(st, "goodsDemand", JsonValue::MakeNumber(static_cast<double>(s.goodsDemand)));
    add(st, "goodsDelivered", JsonValue::MakeNumber(static_cast<double>(s.goodsDelivered)));
    add(st, "goodsSatisfaction", JsonValue::MakeNumber(static_cast<double>(s.goodsSatisfaction)));
    add(st, "avgLandValue", JsonValue::MakeNumber(static_cast<double>(s.avgLandValue)));
    add(st, "demandResidential", JsonValue::MakeNumber(static_cast<double>(s.demandResidential)));
    add(root, "stats", std::move(st));

    const std::string outPath = (caseDir / "summary.json").string();
    std::string jsonErr;
    if (!isocity::WriteJsonFile(outPath, root, jsonErr, isocity::JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false})) {
      outErr = "failed to write summary.json: " + jsonErr;
      return false;
    }
  }

  // final.bin
  {
    std::vector<std::uint8_t> bytes;
    std::string err;
    if (!isocity::SaveWorldBinaryToBytes(out.world, out.procCfg, out.simCfg, bytes, err)) {
      outErr = "SaveWorldBinaryToBytes failed: " + err;
      return false;
    }
    std::ofstream f(caseDir / "final.bin", std::ios::binary);
    if (!f) {
      outErr = "failed to write final.bin";
      return false;
    }
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  }

  // ticks.csv (if available)
  if (!out.tickStats.empty()) {
    std::ofstream f(caseDir / "ticks.csv", std::ios::binary);
    if (!f) {
      outErr = "failed to write ticks.csv";
      return false;
    }
    f << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,"
         "avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
    for (const isocity::Stats& s : out.tickStats) {
      f << s.day << ','
        << s.population << ','
        << s.money << ','
        << s.housingCapacity << ','
        << s.jobsCapacity << ','
        << s.jobsCapacityAccessible << ','
        << s.employed << ','
        << s.happiness << ','
        << s.roads << ','
        << s.parks << ','
        << s.avgCommuteTime << ','
        << s.trafficCongestion << ','
        << s.goodsDemand << ','
        << s.goodsDelivered << ','
        << s.goodsSatisfaction << ','
        << s.avgLandValue << ','
        << s.demandResidential
        << '\n';
    }
  }

  return true;
}

CaseResult ProcessCase(std::size_t index, const isocity::ScenarioCase& sc, const isocity::ScenarioRunOptions& baseOpt,
                       const GoldenConfig& golden, const HashGoldenConfig& hashGolden, const std::string& outDir)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();

  isocity::ScenarioRunOutputs out;
  std::string runErr;

  isocity::ScenarioRunOptions runOpt = baseOpt;
  runOpt.runIndex = static_cast<int>(index);
  const bool runOk = isocity::RunScenario(sc, runOpt, out, runErr);

  CaseResult cr;
  cr.sc = sc;
  cr.hash = out.finalHash;
  cr.error = runErr;

  fs::path caseDir;
  const bool wantArtifacts = !outDir.empty() && runOk;
  if (wantArtifacts) {
    const fs::path base = fs::path(outDir);
    const fs::path name = SanitizeName(fs::path(sc.path).stem().string());
    std::ostringstream sub;
    sub << std::setw(4) << std::setfill('0') << index << "_" << name;
    caseDir = base / sub.str();
    cr.artifactsDir = caseDir.string();

    std::string artErr;
    if (!WriteCaseArtifacts(caseDir, out, sc, artErr)) {
      cr.warnings.push_back("artifact write failed: " + artErr);
    }
  }

  // Golden hash compare/update (optional): store/verify the final world hash per scenario.
  if (runOk && hashGolden.enabled) {
    cr.hashGolden.attempted = true;

    const fs::path goldenPath = ComputeHashGoldenPath(sc, hashGolden);
    cr.hashGolden.path = goldenPath.string();

    std::error_code fec;
    const bool exists = fs::exists(goldenPath, fec) && fs::is_regular_file(goldenPath, fec);

    auto readExpected = [&](std::uint64_t& outVal, std::string& outErr) -> bool {
      outErr.clear();
      std::ifstream f(goldenPath, std::ios::binary);
      if (!f) {
        outErr = "failed to open golden hash file";
        return false;
      }
      std::ostringstream ss;
      ss << f.rdbuf();
      std::string s = Trim(ss.str());
      if (s.empty()) {
        outErr = "golden hash file is empty";
        return false;
      }
      std::uint64_t v = 0;
      if (!ParseU64(s, &v)) {
        outErr = "invalid hash format (expected decimal or 0x...)";
        return false;
      }
      outVal = v;
      return true;
    };

    if (hashGolden.update) {
      bool needsWrite = !exists;

      if (exists) {
        std::uint64_t expected = 0;
        std::string rerr;
        if (readExpected(expected, rerr)) {
          cr.hashGolden.expected = expected;
          cr.hashGolden.hasExpected = true;
          needsWrite = (expected != cr.hash);
        } else {
          needsWrite = true;
        }
      }

      if (needsWrite) {
        std::error_code ec;
        fs::create_directories(goldenPath.parent_path(), ec);
        if (ec) {
          cr.hashGolden.ok = false;
          cr.hashGolden.matched = false;
          cr.hashGolden.error = "failed to create hash golden directory: " + goldenPath.parent_path().string() + " (" + ec.message() + ")";
        } else {
          std::ofstream f(goldenPath, std::ios::binary);
          if (!f) {
            cr.hashGolden.ok = false;
            cr.hashGolden.matched = false;
            cr.hashGolden.error = "failed to write golden hash: " + goldenPath.string();
          } else {
            f << HexU64(cr.hash) << "\n";
            cr.hashGolden.updated = true;
            cr.hashGolden.matched = true;
            cr.hashGolden.ok = true;
          }
        }
      } else {
        cr.hashGolden.updated = false;
        cr.hashGolden.matched = true;
        cr.hashGolden.ok = true;
      }

    } else {
      if (!exists) {
        cr.hashGolden.ok = false;
        cr.hashGolden.matched = false;
        cr.hashGolden.error = "missing golden hash: " + goldenPath.string() + " (run with --update-hash-golden to create)";
      } else {
        std::uint64_t expected = 0;
        std::string rerr;
        if (!readExpected(expected, rerr)) {
          cr.hashGolden.ok = false;
          cr.hashGolden.matched = false;
          cr.hashGolden.error = "failed to read golden hash: " + rerr;
        } else {
          cr.hashGolden.expected = expected;
          cr.hashGolden.hasExpected = true;
          if (expected != cr.hash) {
            cr.hashGolden.ok = false;
            cr.hashGolden.matched = false;
            cr.hashGolden.error = "hash mismatch: expected " + HexU64(expected) + ", got " + HexU64(cr.hash);
          } else {
            cr.hashGolden.ok = true;
            cr.hashGolden.matched = true;
          }
        }
      }
    }
  }

  // Golden compare/update (optional).
  if (runOk && golden.enabled) {
    cr.golden.attempted = true;

    isocity::PpmImage actual;
    std::string rerr;
    if (!RenderGoldenImage(out, golden, actual, rerr)) {
      cr.golden.ok = false;
      cr.golden.matched = false;
      cr.golden.error = "golden render failed: " + rerr;
    } else {
      const fs::path goldenPath = ComputeGoldenPath(sc, golden);
      cr.golden.goldenPath = goldenPath.string();

      std::error_code fec;
      const bool goldenExists = fs::exists(goldenPath, fec) && fs::is_regular_file(goldenPath, fec);

      isocity::PpmImage expected;
      isocity::PpmImage diff;

      if (golden.update) {
        // Update mode: write snapshot if missing or different.
        bool needsWrite = !goldenExists;

        if (goldenExists) {
          std::string readErr;
          if (isocity::ReadImageAuto(goldenPath.string(), expected, readErr)) {
            isocity::PpmDiffStats st{};
            if (isocity::ComparePpm(expected, actual, st, golden.threshold, nullptr)) {
              cr.golden.stats = st;
              cr.golden.hasStats = true;
              needsWrite = (st.pixelsDifferent != 0);
            } else {
              needsWrite = true;
            }
          } else {
            needsWrite = true;
          }
        }

        if (needsWrite) {
          std::error_code ec;
          fs::create_directories(goldenPath.parent_path(), ec);
          if (ec) {
            cr.golden.ok = false;
            cr.golden.matched = false;
            cr.golden.error = "failed to create golden directory: " + goldenPath.parent_path().string() + " (" + ec.message() + ")";
          } else {
            std::string werr;
            if (!isocity::WriteImageAuto(goldenPath.string(), actual, werr)) {
              cr.golden.ok = false;
              cr.golden.matched = false;
              cr.golden.error = "failed to update golden: " + werr;
            } else {
              cr.golden.updated = true;
              cr.golden.matched = true;
              cr.golden.ok = true;
            }
          }
        } else {
          cr.golden.updated = false;
          cr.golden.matched = true;
          cr.golden.ok = true;
        }

      } else {
        // Compare mode.
        if (!goldenExists) {
          cr.golden.ok = false;
          cr.golden.matched = false;
          cr.golden.error = "missing golden image: " + goldenPath.string() + " (run with --update-golden to create)";
        } else {
          std::string readErr;
          if (!isocity::ReadImageAuto(goldenPath.string(), expected, readErr)) {
            cr.golden.ok = false;
            cr.golden.matched = false;
            cr.golden.error = "failed to read golden image: " + readErr;
          } else {
            isocity::PpmDiffStats st{};
            isocity::PpmImage* diffPtr = wantArtifacts ? &diff : nullptr;
            if (!isocity::ComparePpm(expected, actual, st, golden.threshold, diffPtr)) {
              cr.golden.ok = false;
              cr.golden.matched = false;
              cr.golden.error = "golden compare failed (dimension mismatch or invalid buffers)";
            } else {
              cr.golden.stats = st;
              cr.golden.hasStats = true;

              if (st.pixelsDifferent != 0) {
                cr.golden.ok = false;
                cr.golden.matched = false;
                cr.golden.error = "golden mismatch: " + FormatPpmDiffSummary(st);
              } else {
                cr.golden.ok = true;
                cr.golden.matched = true;
              }
            }
          }
        }
      }

      // Write golden artifacts when requested.
      if (wantArtifacts) {
        std::string gerr;
        const isocity::PpmImage* expectedPtr = (!expected.rgb.empty()) ? &expected : nullptr;
        const isocity::PpmImage* diffPtr = (!diff.rgb.empty()) ? &diff : nullptr;
        if (!WriteGoldenArtifacts(caseDir, golden, cr.golden, actual, expectedPtr, diffPtr, gerr)) {
          cr.warnings.push_back("golden artifact write failed: " + gerr);
        }
      }
    }
  }

  cr.ok = runOk && (!golden.enabled || cr.golden.ok) && (!hashGolden.enabled || cr.hashGolden.ok);

  if (golden.enabled && runOk && !cr.golden.ok) {
    if (!cr.error.empty()) cr.error += " | ";
    cr.error += cr.golden.error;
  }

  if (hashGolden.enabled && runOk && !cr.hashGolden.ok) {
    if (!cr.error.empty()) cr.error += " | ";
    cr.error += cr.hashGolden.error;
  }

  const auto t1 = Clock::now();
  cr.seconds = std::chrono::duration<double>(t1 - t0).count();

  return cr;
}

std::string RelLinkForHtml(const fs::path& target, const fs::path& htmlDir)
{
  std::error_code ec;
  fs::path rel = fs::relative(target, htmlDir, ec);
  fs::path use = ec ? target : rel;
  std::string s = use.generic_string();
  if (s.empty()) s = target.generic_string();
  return s;
}

bool WriteHtmlReport(const std::string& htmlPath, const std::string& htmlTitle, const std::vector<CaseResult>& results,
                     int passed, int failed, double seconds, int jobsRequested, int jobsUsed, bool goldenEnabled,
                     bool hashGoldenEnabled, std::string& outErr)
{
  outErr.clear();

  if (htmlPath.empty()) {
    outErr = "empty html report path";
    return false;
  }

  const fs::path outPath = fs::path(htmlPath);
  const fs::path htmlDir = outPath.parent_path().empty() ? fs::path(".") : outPath.parent_path();

  {
    std::error_code ec;
    if (!outPath.parent_path().empty()) {
      fs::create_directories(outPath.parent_path(), ec);
      if (ec) {
        outErr = "failed to create html report directory: " + outPath.parent_path().string() + " (" + ec.message() + ")";
        return false;
      }
    }
  }

  std::ofstream f(outPath, std::ios::binary);
  if (!f) {
    outErr = "failed to write html report: " + outPath.string();
    return false;
  }

  const std::string title = htmlTitle.empty() ? std::string("ProcIsoCity Suite Report") : htmlTitle;

  f << "<!doctype html>\n";
  f << "<html lang=\"en\">\n";
  f << "<head>\n";
  f << "  <meta charset=\"utf-8\">\n";
  f << "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  f << "  <title>" << EscapeXml(title) << "</title>\n";
  f << "  <style>\n";
  f << "    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:20px;line-height:1.35;}\n";
  f << "    h1{margin:0 0 12px 0;}\n";
  f << "    .meta{margin:8px 0 16px 0;color:#333;}\n";
  f << "    .chips{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 16px 0;}\n";
  f << "    .chip{padding:4px 10px;border-radius:999px;border:1px solid #ccc;font-size:12px;}\n";
  f << "    .pass{background:#eaffea;border-color:#b7e1b7;}\n";
  f << "    .fail{background:#ffecec;border-color:#e1b7b7;}\n";
  f << "    details.case{border:1px solid #ddd;border-radius:8px;padding:8px 10px;margin:10px 0;}\n";
  f << "    details.case[data-ok=\"0\"]{border-color:#e1b7b7;background:#fff7f7;}\n";
  f << "    details.case[data-ok=\"1\"]{border-color:#b7e1b7;background:#f7fff7;}\n";
  f << "    details.case > summary{cursor:pointer;user-select:none;}\n";
  f << "    code{background:#f2f2f2;padding:1px 4px;border-radius:4px;}\n";
  f << "    pre{white-space:pre-wrap;word-break:break-word;background:#111;color:#f5f5f5;padding:10px;border-radius:8px;overflow:auto;}\n";
  f << "    .links a{margin-right:12px;}\n";
  f << "    .images{display:flex;gap:12px;flex-wrap:wrap;margin-top:10px;}\n";
  f << "    .imgbox{min-width:220px;}\n";
  f << "    .imgbox img{max-width:340px;width:100%;height:auto;border:1px solid #ccc;border-radius:6px;}\n";
  f << "    .small{font-size:12px;color:#444;}\n";
  f << "  </style>\n";
  f << "  <script>\n";
  f << "    function updateFilter(){\n";
  f << "      var showPass=document.getElementById('showPass').checked;\n";
  f << "      var showFail=document.getElementById('showFail').checked;\n";
  f << "      var nodes=document.querySelectorAll('details.case');\n";
  f << "      for(var i=0;i<nodes.length;i++){\n";
  f << "        var ok=nodes[i].getAttribute('data-ok')==='1';\n";
  f << "        nodes[i].style.display = (ok?showPass:showFail) ? '' : 'none';\n";
  f << "      }\n";
  f << "    }\n";
  f << "    window.addEventListener('load', updateFilter);\n";
  f << "  </script>\n";
  f << "</head>\n";
  f << "<body>\n";

  f << "  <h1>" << EscapeXml(title) << "</h1>\n";
  f << "  <div class=\"meta\">\n";
  f << "    <div><strong>Total:</strong> " << results.size() << " &nbsp; <strong>Passed:</strong> " << passed
    << " &nbsp; <strong>Failed:</strong> " << failed << "</div>\n";
  f << "    <div><strong>Time:</strong> " << std::fixed << std::setprecision(3) << seconds << "s &nbsp; <strong>Jobs:</strong> "
    << jobsUsed << " (requested " << jobsRequested << ")</div>\n";
  f << "    <div class=\"small\">Generated by proc_isocity_suite. Open this file in a browser (local file).";
  if (!goldenEnabled) {
    f << " Image previews require <code>--golden</code> (and <code>--out-dir</code> for artifacts).";
  }
  if (!hashGoldenEnabled) {
    f << " Hash regression requires <code>--hash-golden</code>.";
  }
  f << "</div>\n";
  f << "  </div>\n";

  f << "  <div class=\"chips\">\n";
  f << "    <label class=\"chip pass\"><input type=\"checkbox\" id=\"showPass\" checked onchange=\"updateFilter()\"> show passed</label>\n";
  f << "    <label class=\"chip fail\"><input type=\"checkbox\" id=\"showFail\" checked onchange=\"updateFilter()\"> show failed</label>\n";
  f << "  </div>\n";

  for (std::size_t i = 0; i < results.size(); ++i) {
    const CaseResult& r = results[i];
    const bool ok = r.ok;
    f << "  <details class=\"case\" data-ok=\"" << (ok ? '1' : '0') << "\"";
    if (!ok) f << " open";
    f << ">\n";

    f << "    <summary>";
    f << (ok ? "✅" : "❌") << " [" << std::setw(4) << std::setfill('0') << i << "] " << EscapeXml(r.sc.path)
      << " <span class=\"small\">(" << KindName(r.sc.kind) << ", " << std::fixed << std::setprecision(3) << r.seconds
      << "s)</span>";
    f << "</summary>\n";

    f << "    <div class=\"small\"><strong>Hash:</strong> <code>" << HexU64(r.hash) << "</code></div>\n";

    if (!r.warnings.empty()) {
      f << "    <div class=\"small\"><strong>Warnings:</strong><ul>\n";
      for (const std::string& w : r.warnings) {
        f << "      <li>" << EscapeXml(w) << "</li>\n";
      }
      f << "    </ul></div>\n";
    }

    if (!r.error.empty()) {
      f << "    <pre>" << EscapeXml(r.error) << "</pre>\n";
    }

    // Links to artifacts if present.
    if (!r.artifactsDir.empty()) {
      const fs::path caseDir = fs::path(r.artifactsDir);
      f << "    <div class=\"links small\"><strong>Artifacts:</strong> ";

      const fs::path summary = caseDir / "summary.json";
      const fs::path finalBin = caseDir / "final.bin";
      const fs::path ticks = caseDir / "ticks.csv";
      const fs::path goldenJson = caseDir / "golden.json";

      auto linkIfExists = [&](const fs::path& p, const char* label) {
        std::error_code ec;
        if (fs::exists(p, ec)) {
          f << "<a href=\"" << EscapeXml(RelLinkForHtml(p, htmlDir)) << "\">" << EscapeXml(label) << "</a>";
        }
      };

      linkIfExists(summary, "summary.json");
      linkIfExists(ticks, "ticks.csv");
      linkIfExists(finalBin, "final.bin");
      if (goldenEnabled) linkIfExists(goldenJson, "golden.json");
      f << "</div>\n";

      // Golden previews.
      if (goldenEnabled && r.golden.attempted) {
        f << "    <div class=\"small\"><strong>Golden image:</strong> ";
        if (!r.golden.goldenPath.empty()) {
          f << "<code>" << EscapeXml(r.golden.goldenPath) << "</code>";
        }
        if (r.golden.hasStats) {
          f << " &nbsp; <span class=\"small\">" << EscapeXml(FormatPpmDiffSummary(r.golden.stats)) << "</span>";
        }
        f << "</div>\n";

        const fs::path expPrev = caseDir / "golden_expected_preview.png";
        const fs::path actPrev = caseDir / "golden_actual_preview.png";
        const fs::path diffPrev = caseDir / "golden_diff_preview.png";

        std::error_code ec;
        const bool haveAct = fs::exists(actPrev, ec);
        const bool haveExp = fs::exists(expPrev, ec);
        const bool haveDiff = fs::exists(diffPrev, ec);

        if (haveExp || haveAct || haveDiff) {
          f << "    <div class=\"images\">\n";
          if (haveExp) {
            f << "      <div class=\"imgbox\"><div class=\"small\">expected</div><img src=\"" << EscapeXml(RelLinkForHtml(expPrev, htmlDir)) << "\" alt=\"expected\"></div>\n";
          }
          if (haveAct) {
            f << "      <div class=\"imgbox\"><div class=\"small\">actual</div><img src=\"" << EscapeXml(RelLinkForHtml(actPrev, htmlDir)) << "\" alt=\"actual\"></div>\n";
          }
          if (haveDiff) {
            f << "      <div class=\"imgbox\"><div class=\"small\">diff</div><img src=\"" << EscapeXml(RelLinkForHtml(diffPrev, htmlDir)) << "\" alt=\"diff\"></div>\n";
          }
          f << "    </div>\n";
        }
      }
    }

    // Hash golden section.
    if (hashGoldenEnabled && r.hashGolden.attempted) {
      f << "    <div class=\"small\"><strong>Golden hash:</strong> ";
      if (!r.hashGolden.path.empty()) {
        f << "<code>" << EscapeXml(r.hashGolden.path) << "</code>";
      }
      if (r.hashGolden.hasExpected) {
        f << " &nbsp; expected <code>" << HexU64(r.hashGolden.expected) << "</code>";
      }
      f << "</div>\n";
    }

    f << "  </details>\n";
  }

  f << "</body>\n";
  f << "</html>\n";

  return static_cast<bool>(f);
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::vector<std::string> manifestPaths;
  std::vector<std::string> discoverDirs;
  std::vector<std::string> discoverExts;
  std::vector<std::string> inputs;
  std::map<std::string, std::string> scriptVars;

  bool failFast = false;
  bool verbose = false;
  bool ignoreReplayAsserts = false;
  bool laxReplayPatches = false;

  int jobs = 1;        // 0 = auto
  bool timing = false;

  std::string outDir;
  std::string jsonReport;
  std::string junitPath;
  std::string htmlReport;
  std::string htmlTitle;

  int shardIndex = 0;
  int shardCount = 1;

  GoldenConfig golden;
  HashGoldenConfig hashGolden;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string val;

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--manifest") {
      if (!requireValue(i, val)) {
        std::cerr << "--manifest requires a path\n";
        return 2;
      }
      manifestPaths.push_back(val);
    } else if (arg == "--discover") {
      if (!requireValue(i, val)) {
        std::cerr << "--discover requires a directory\n";
        return 2;
      }
      discoverDirs.push_back(val);
    } else if (arg == "--ext") {
      if (!requireValue(i, val)) {
        std::cerr << "--ext requires a value (e.g. .isocity)\n";
        return 2;
      }
      discoverExts.push_back(val);
    } else if (arg == "--shard") {
      if (!requireValue(i, val)) {
        std::cerr << "--shard requires i/n\n";
        return 2;
      }
      if (!ParseShard(val, &shardIndex, &shardCount)) {
        std::cerr << "invalid --shard (expected 0-based i/n, e.g. 0/4): " << val << "\n";
        return 2;
      }
    } else if (arg == "--jobs" || arg == "-j") {
      if (!requireValue(i, val)) {
        std::cerr << "--jobs requires an integer (0 = auto)\n";
        return 2;
      }
      if (!ParseI32(val, &jobs) || jobs < 0) {
        std::cerr << "invalid --jobs (expected integer >= 0): " << val << "\n";
        return 2;
      }
    } else if (arg == "--define") {
      if (!requireValue(i, val)) {
        std::cerr << "--define requires k=v\n";
        return 2;
      }
      const std::size_t eq = val.find('=');
      if (eq == std::string::npos) {
        std::cerr << "--define requires k=v\n";
        return 2;
      }
      std::string k = ToLower(Trim(val.substr(0, eq)));
      std::string v = val.substr(eq + 1);
      if (k.empty()) {
        std::cerr << "--define requires non-empty key\n";
        return 2;
      }
      scriptVars[k] = v;
    } else if (arg == "--fail-fast") {
      failFast = true;
    } else if (arg == "--verbose") {
      verbose = true;
    } else if (arg == "--timing") {
      timing = true;
    } else if (arg == "--ignore-replay-asserts" || arg == "--ignore-asserts") {
      ignoreReplayAsserts = true;
    } else if (arg == "--lax-replay-patches" || arg == "--lax-patches") {
      laxReplayPatches = true;
    } else if (arg == "--out-dir") {
      if (!requireValue(i, val)) {
        std::cerr << "--out-dir requires a directory\n";
        return 2;
      }
      outDir = val;
    } else if (arg == "--json-report") {
      if (!requireValue(i, val)) {
        std::cerr << "--json-report requires a path\n";
        return 2;
      }
      jsonReport = val;
    } else if (arg == "--junit") {
      if (!requireValue(i, val)) {
        std::cerr << "--junit requires a path\n";
        return 2;
      }
      junitPath = val;

    } else if (arg == "--html-report") {
      if (!requireValue(i, val)) {
        std::cerr << "--html-report requires a path\n";
        return 2;
      }
      htmlReport = val;
    } else if (arg == "--html-title") {
      if (!requireValue(i, val)) {
        std::cerr << "--html-title requires a string\n";
        return 2;
      }
      htmlTitle = val;

    } else if (arg == "--golden") {
      golden.enabled = true;
    } else if (arg == "--update-golden") {
      golden.enabled = true;
      golden.update = true;
    } else if (arg == "--golden-dir") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-dir requires a directory\n";
        return 2;
      }
      golden.enabled = true;
      golden.goldenDir = val;
    } else if (arg == "--golden-ext") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-ext requires ppm|png\n";
        return 2;
      }
      golden.enabled = true;
      std::string v = ToLower(Trim(val));
      if (!v.empty() && v[0] == '.') v = v.substr(1);
      if (v == "ppm" || v == "png") {
        golden.goldenExt = v;
      } else {
        std::cerr << "invalid --golden-ext (expected ppm|png): " << val << "\n";
        return 2;
      }
    } else if (arg == "--golden-format") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-format requires top|iso\n";
        return 2;
      }
      golden.enabled = true;
      const std::string v = ToLower(Trim(val));
      if (v == "top" || v == "ppm" || v == "topdown") {
        golden.iso = false;
      } else if (v == "iso" || v == "isometric") {
        golden.iso = true;
      } else {
        std::cerr << "invalid --golden-format (expected top|iso): " << val << "\n";
        return 2;
      }
    } else if (arg == "--golden-layer") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-layer requires a layer name\n";
        return 2;
      }
      golden.enabled = true;
      ExportLayer layer{};
      if (!ParseExportLayer(val, layer)) {
        std::cerr << "invalid --golden-layer: " << val << "\n";
        return 2;
      }
      golden.layer = layer;
    } else if (arg == "--golden-scale") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-scale requires an integer\n";
        return 2;
      }
      golden.enabled = true;
      if (!ParseI32(val, &golden.scale) || golden.scale <= 0) {
        std::cerr << "invalid --golden-scale: " << val << "\n";
        return 2;
      }
    } else if (arg == "--golden-threshold") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-threshold requires an integer (0..255)\n";
        return 2;
      }
      golden.enabled = true;
      if (!ParseI32(val, &golden.threshold) || golden.threshold < 0 || golden.threshold > 255) {
        std::cerr << "invalid --golden-threshold (expected 0..255): " << val << "\n";
        return 2;
      }
    } else if (arg == "--golden-iso-tile") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-iso-tile requires WxH\n";
        return 2;
      }
      golden.enabled = true;
      golden.iso = true;
      int tw = 0, th = 0;
      if (!ParseWxH(val, &tw, &th)) {
        std::cerr << "invalid --golden-iso-tile (expected WxH): " << val << "\n";
        return 2;
      }
      golden.isoCfg.tileW = tw;
      golden.isoCfg.tileH = th;
    } else if (arg == "--golden-iso-height") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-iso-height requires an integer\n";
        return 2;
      }
      golden.enabled = true;
      golden.iso = true;
      if (!ParseI32(val, &golden.isoCfg.heightScalePx) || golden.isoCfg.heightScalePx < 0) {
        std::cerr << "invalid --golden-iso-height: " << val << "\n";
        return 2;
      }
    } else if (arg == "--golden-iso-margin") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-iso-margin requires an integer\n";
        return 2;
      }
      golden.enabled = true;
      golden.iso = true;
      if (!ParseI32(val, &golden.isoCfg.marginPx) || golden.isoCfg.marginPx < 0) {
        std::cerr << "invalid --golden-iso-margin: " << val << "\n";
        return 2;
      }
    } else if (arg == "--golden-iso-grid") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-iso-grid requires 0|1\n";
        return 2;
      }
      golden.enabled = true;
      golden.iso = true;
      bool b = false;
      if (!ParseBool01(val, &b)) {
        std::cerr << "invalid --golden-iso-grid (expected 0|1): " << val << "\n";
        return 2;
      }
      golden.isoCfg.drawGrid = b;
    } else if (arg == "--golden-iso-cliffs") {
      if (!requireValue(i, val)) {
        std::cerr << "--golden-iso-cliffs requires 0|1\n";
        return 2;
      }
      golden.enabled = true;
      golden.iso = true;
      bool b = false;
      if (!ParseBool01(val, &b)) {
        std::cerr << "invalid --golden-iso-cliffs (expected 0|1): " << val << "\n";
        return 2;
      }
      golden.isoCfg.drawCliffs = b;

    } else if (arg == "--hash-golden") {
      hashGolden.enabled = true;
    } else if (arg == "--update-hash-golden") {
      hashGolden.enabled = true;
      hashGolden.update = true;
    } else if (arg == "--hash-golden-dir") {
      if (!requireValue(i, val)) {
        std::cerr << "--hash-golden-dir requires a directory\n";
        return 2;
      }
      hashGolden.enabled = true;
      hashGolden.dir = val;
    } else if (arg == "--hash-golden-ext") {
      if (!requireValue(i, val)) {
        std::cerr << "--hash-golden-ext requires a value (e.g. hash)\n";
        return 2;
      }
      hashGolden.enabled = true;
      std::string v = ToLower(Trim(val));
      if (!v.empty() && v[0] == '.') v = v.substr(1);
      if (v.empty()) {
        std::cerr << "invalid --hash-golden-ext (empty)\n";
        return 2;
      }
      hashGolden.ext = v;

    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    } else {
      inputs.push_back(arg);
    }
  }

  // Collect scenario cases.
  std::vector<ScenarioCase> cases;
  std::string err;

  for (const std::string& m : manifestPaths) {
    std::vector<ScenarioCase> tmp;
    if (!LoadScenarioManifest(m, tmp, err)) {
      std::cerr << err << "\n";
      return 2;
    }
    cases.insert(cases.end(), tmp.begin(), tmp.end());
  }

  for (const std::string& d : discoverDirs) {
    std::vector<ScenarioCase> tmp;
    if (!DiscoverScenarios(d, discoverExts, tmp, err)) {
      std::cerr << err << "\n";
      return 2;
    }
    cases.insert(cases.end(), tmp.begin(), tmp.end());
  }

  for (const std::string& p : inputs) {
    ScenarioCase sc;
    sc.path = p;
    sc.kind = GuessScenarioKindFromPath(p);
    cases.push_back(std::move(sc));
  }

  if (cases.empty()) {
    PrintHelp();
    return 2;
  }

  // Sort for determinism.
  std::sort(cases.begin(), cases.end(), [](const ScenarioCase& a, const ScenarioCase& b) { return a.path < b.path; });

  // Shard filter.
  std::vector<ScenarioCase> sharded;
  sharded.reserve(cases.size());
  for (std::size_t idx = 0; idx < cases.size(); ++idx) {
    if (shardCount > 1) {
      if (static_cast<int>(idx % static_cast<std::size_t>(shardCount)) != shardIndex) continue;
    }
    sharded.push_back(cases[idx]);
  }
  cases.swap(sharded);

  if (cases.empty()) {
    std::cout << "No cases selected (shard filter removed all cases).\n";
    return 0;
  }

  ScenarioRunOptions runOpt;
  runOpt.quiet = !verbose;
  runOpt.strictReplayAsserts = !ignoreReplayAsserts;
  runOpt.strictReplayPatches = !laxReplayPatches;
  runOpt.scriptVars = scriptVars;

  using Clock = std::chrono::steady_clock;
  const auto suiteT0 = Clock::now();

  int threads = jobs;
  if (threads <= 0) {
    threads = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = 1;
  }

  // Avoid launching lots of threads for tiny suites.
  threads = std::min<int>(threads, static_cast<int>(cases.size()));

  std::vector<CaseResult> results;
  results.reserve(cases.size());

  int passed = 0;
  int failed = 0;

  auto printCase = [&](const CaseResult& cr) {
    if (cr.ok) {
      std::cout << "[PASS] " << cr.sc.path << "  " << HexU64(cr.hash);
      if (golden.enabled && cr.golden.attempted && golden.update && cr.golden.updated) {
        std::cout << "  [golden updated]";
      }
      if (hashGolden.enabled && cr.hashGolden.attempted && hashGolden.update && cr.hashGolden.updated) {
        std::cout << "  [hash updated]";
      }
      if (timing) {
        std::cout << "  (" << std::fixed << std::setprecision(3) << cr.seconds << "s)";
      }
      std::cout << "\n";
    } else {
      std::cout << "[FAIL] " << cr.sc.path;
      if (timing) {
        std::cout << "  (" << std::fixed << std::setprecision(3) << cr.seconds << "s)";
      }
      std::cout << "\n";
      std::cout << "       " << cr.error << "\n";
    }

    for (const std::string& w : cr.warnings) {
      std::cerr << "  [WARN] " << w << "\n";
    }
  };

  if (threads <= 1) {
    for (std::size_t i = 0; i < cases.size(); ++i) {
      CaseResult cr = ProcessCase(i, cases[i], runOpt, golden, hashGolden, outDir);

      if (cr.ok) ++passed;
      else ++failed;

      printCase(cr);
      results.push_back(std::move(cr));

      if (failFast && !cr.ok) break;
    }
  } else {
    std::vector<CaseResult> all;
    all.resize(cases.size());
    std::vector<unsigned char> done(cases.size(), 0);

    std::atomic<std::size_t> next{0};
    std::atomic<bool> stop{false};

    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(threads));

    for (int t = 0; t < threads; ++t) {
      pool.emplace_back([&]() {
        for (;;) {
          if (failFast && stop.load()) break;
          const std::size_t i = next.fetch_add(1);
          if (i >= cases.size()) break;
          if (failFast && stop.load()) break;

          all[i] = ProcessCase(i, cases[i], runOpt, golden, hashGolden, outDir);
          done[i] = 1;

          if (failFast && !all[i].ok) stop.store(true);
        }
      });
    }
    for (auto& th : pool) th.join();

    for (std::size_t i = 0; i < cases.size(); ++i) {
      if (!done[i]) continue;
      CaseResult& cr = all[i];
      if (cr.ok) ++passed;
      else ++failed;
      printCase(cr);
      results.push_back(std::move(cr));
    }
  }

  const auto suiteT1 = Clock::now();
  const double suiteSeconds = std::chrono::duration<double>(suiteT1 - suiteT0).count();

  // Suite JSON report.
  if (!jsonReport.empty()) {
    using isocity::JsonValue;
    JsonValue root = JsonValue::MakeObject();
    auto add = [](JsonValue& obj, const char* key, JsonValue v) {
      obj.objectValue.emplace_back(key, std::move(v));
    };

    add(root, "total", JsonValue::MakeNumber(static_cast<double>(results.size())));
    add(root, "passed", JsonValue::MakeNumber(static_cast<double>(passed)));
    add(root, "failed", JsonValue::MakeNumber(static_cast<double>(failed)));
    add(root, "seconds", JsonValue::MakeNumber(suiteSeconds));
    add(root, "jobsRequested", JsonValue::MakeNumber(static_cast<double>(jobs)));
    add(root, "jobsUsed", JsonValue::MakeNumber(static_cast<double>(threads)));

    JsonValue casesArr = JsonValue::MakeArray();
    casesArr.arrayValue.reserve(results.size());
    for (const CaseResult& r : results) {
      JsonValue c = JsonValue::MakeObject();
      add(c, "path", JsonValue::MakeString(r.sc.path));
      add(c, "kind", JsonValue::MakeString(KindName(r.sc.kind)));
      add(c, "ok", JsonValue::MakeBool(r.ok));
      add(c, "seconds", JsonValue::MakeNumber(r.seconds));
      add(c, "hash", JsonValue::MakeString(HexU64(r.hash)));
      add(c, "error", JsonValue::MakeString(r.error));

      if (!r.artifactsDir.empty()) {
        add(c, "artifactsDir", JsonValue::MakeString(r.artifactsDir));
      }
      if (!r.warnings.empty()) {
        JsonValue warn = JsonValue::MakeArray();
        warn.arrayValue.reserve(r.warnings.size());
        for (const auto& w : r.warnings) warn.arrayValue.emplace_back(JsonValue::MakeString(w));
        add(c, "warnings", std::move(warn));
      }
      if (golden.enabled) {
        JsonValue g = JsonValue::MakeObject();
        add(g, "attempted", JsonValue::MakeBool(r.golden.attempted));
        add(g, "ok", JsonValue::MakeBool(r.golden.ok));
        add(g, "updated", JsonValue::MakeBool(r.golden.updated));
        add(g, "matched", JsonValue::MakeBool(r.golden.matched));
        add(g, "goldenPath", JsonValue::MakeString(r.golden.goldenPath));
        add(g, "error", JsonValue::MakeString(r.golden.error));
        add(c, "golden", std::move(g));
      }
      if (hashGolden.enabled) {
        JsonValue g = JsonValue::MakeObject();
        add(g, "attempted", JsonValue::MakeBool(r.hashGolden.attempted));
        add(g, "ok", JsonValue::MakeBool(r.hashGolden.ok));
        add(g, "updated", JsonValue::MakeBool(r.hashGolden.updated));
        add(g, "matched", JsonValue::MakeBool(r.hashGolden.matched));
        add(g, "goldenPath", JsonValue::MakeString(r.hashGolden.path));
        add(g, "expected", JsonValue::MakeString(r.hashGolden.hasExpected ? HexU64(r.hashGolden.expected) : std::string()));
        add(g, "error", JsonValue::MakeString(r.hashGolden.error));
        add(c, "hashGolden", std::move(g));
      }

      casesArr.arrayValue.emplace_back(std::move(c));
    }
    add(root, "cases", std::move(casesArr));

    std::string jsonErr;
    if (!isocity::WriteJsonFile(jsonReport, root, jsonErr, isocity::JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false})) {
      std::cerr << "failed to write json report: " << jsonErr << "\n";
    }
  }

  // JUnit report.
  if (!junitPath.empty()) {
    std::ofstream f(junitPath, std::ios::binary);
    if (!f) {
      std::cerr << "failed to write junit report: " << junitPath << "\n";
    } else {
      f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
      f << "<testsuite name=\"ProcIsoCitySuite\" tests=\"" << results.size() << "\" failures=\"" << failed
        << "\" time=\"" << std::fixed << std::setprecision(6) << suiteSeconds << "\">\n";
      for (const CaseResult& r : results) {
        f << "  <testcase classname=\"" << KindName(r.sc.kind) << "\" name=\"" << EscapeXml(r.sc.path)
          << "\" time=\"" << std::fixed << std::setprecision(6) << r.seconds << "\">\n";
        if (!r.ok) {
          f << "    <failure message=\"" << EscapeXml(r.error) << "\"/>\n";
        }
        f << "  </testcase>\n";
      }
      f << "</testsuite>\n";
    }
  }

  // HTML report/dashboard.
  if (!htmlReport.empty()) {
    std::string herr;
    if (!WriteHtmlReport(htmlReport, htmlTitle, results, passed, failed, suiteSeconds, jobs, threads, golden.enabled,
                         hashGolden.enabled, herr)) {
      std::cerr << "failed to write html report: " << herr << "\n";
    }
  }

  std::cout << "\nSuite results: " << passed << " passed, " << failed << " failed (" << results.size() << " total)\n";
  if (timing) {
    std::cout << "Suite time: " << std::fixed << std::setprecision(3) << suiteSeconds << "s";
    std::cout << " (jobsUsed=" << threads << ")\n";
  }

  return (failed == 0) ? 0 : 1;
}
