#include "isocity/Suite.hpp"

#include "isocity/Export.hpp"
#include "isocity/Hash.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
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

bool ParseU8(const std::string& s, std::uint8_t* out)
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

std::string EscapeJson(const std::string& s)
{
  std::ostringstream oss;
  for (char c : s) {
    switch (c) {
      case '\\': oss << "\\\\"; break;
      case '"': oss << "\\\""; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)(unsigned char)c << std::dec;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
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
      << "  --define k=v                Inject a script variable (repeatable).\n"
      << "  --ignore-replay-asserts      Ignore AssertHash events when running replays.\n"
      << "  --lax-replay-patches         Do not require patch base hashes to match when playing replays.\n"
      << "  --out-dir <dir>              Write per-case artifacts (summary.json, final.bin, ticks.csv).\n"
      << "  --json-report <file>         Write a suite summary JSON.\n"
      << "  --junit <file>               Write a JUnit XML report (useful for CI).\n"
      << "  --fail-fast                  Stop on first failure.\n"
      << "  --verbose                    Print script output (default is quiet).\n"
      << "\n"
      << "Golden image regression (snapshot testing):\n"
      << "  --golden                     Compare a rendered PPM against a golden snapshot.\n"
      << "  --update-golden              Create/overwrite golden snapshots instead of failing.\n"
      << "  --golden-dir <dir>            Base directory for goldens (default: next to scenario file).\n"
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

struct CaseResult {
  isocity::ScenarioCase sc;
  bool ok = false;
  std::string error;
  std::uint64_t hash = 0;

  GoldenResult golden;
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
  suffix << "." << isocity::ExportLayerName(g.layer) << ".ppm";

  fs::path out = base;
  out += suffix.str();
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
                          const isocity::PpmImage& actual, const isocity::PpmImage* diffImg, std::string& outErr)
{
  outErr.clear();

  // golden_actual.ppm
  {
    std::string err;
    if (!isocity::WritePpm((caseDir / "golden_actual.ppm").string(), actual, err)) {
      outErr = "failed to write golden_actual.ppm: " + err;
      return false;
    }
  }

  // golden_diff.ppm
  if (diffImg && !diffImg->rgb.empty() && diffImg->width > 0 && diffImg->height > 0) {
    std::string err;
    if (!isocity::WritePpm((caseDir / "golden_diff.ppm").string(), *diffImg, err)) {
      outErr = "failed to write golden_diff.ppm: " + err;
      return false;
    }
  }

  // golden.json
  {
    std::ofstream f(caseDir / "golden.json", std::ios::binary);
    if (!f) {
      outErr = "failed to write golden.json";
      return false;
    }

    f << "{\n";
    f << "  \"enabled\": " << (cfg.enabled ? "true" : "false") << ",\n";
    f << "  \"updateMode\": " << (cfg.update ? "true" : "false") << ",\n";
    f << "  \"format\": \"" << (cfg.iso ? "iso" : "top") << "\",\n";
    f << "  \"layer\": \"" << isocity::ExportLayerName(cfg.layer) << "\",\n";
    f << "  \"scale\": " << cfg.scale << ",\n";
    f << "  \"threshold\": " << cfg.threshold << ",\n";
    f << "  \"goldenPath\": \"" << EscapeJson(res.goldenPath) << "\",\n";
    f << "  \"attempted\": " << (res.attempted ? "true" : "false") << ",\n";
    f << "  \"ok\": " << (res.ok ? "true" : "false") << ",\n";
    f << "  \"updated\": " << (res.updated ? "true" : "false") << ",\n";
    f << "  \"matched\": " << (res.matched ? "true" : "false") << ",\n";
    f << "  \"error\": \"" << EscapeJson(res.error) << "\"";
    if (res.hasStats) {
      f << ",\n";
      f << "  \"stats\": {\n";
      f << "    \"width\": " << res.stats.width << ",\n";
      f << "    \"height\": " << res.stats.height << ",\n";
      f << "    \"pixelsCompared\": " << res.stats.pixelsCompared << ",\n";
      f << "    \"pixelsDifferent\": " << res.stats.pixelsDifferent << ",\n";
      f << "    \"maxAbsDiff\": " << static_cast<int>(res.stats.maxAbsDiff) << ",\n";
      f << "    \"meanAbsDiff\": " << std::fixed << std::setprecision(9) << res.stats.meanAbsDiff << ",\n";
      f << "    \"mse\": " << std::fixed << std::setprecision(9) << res.stats.mse << ",\n";
      if (std::isinf(res.stats.psnr)) {
        f << "    \"psnr\": null,\n";
        f << "    \"psnrIsInf\": true\n";
      } else {
        f << "    \"psnr\": " << std::fixed << std::setprecision(6) << res.stats.psnr << ",\n";
        f << "    \"psnrIsInf\": false\n";
      }
      f << "  }\n";
      f << "}\n";
    } else {
      f << "\n";
      f << "}\n";
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
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"case\": {\n";
    oss << "    \"path\": \"" << EscapeJson(sc.path) << "\",\n";
    oss << "    \"kind\": \"" << KindName(sc.kind) << "\"\n";
    oss << "  },\n";
    oss << "  \"width\": " << out.world.width() << ",\n";
    oss << "  \"height\": " << out.world.height() << ",\n";
    oss << "  \"seed\": " << out.world.seed() << ",\n";
    oss << "  \"hash\": \"" << HexU64(out.finalHash) << "\",\n";
    oss << "  \"stats\": {\n";
    oss << "    \"day\": " << s.day << ",\n";
    oss << "    \"population\": " << s.population << ",\n";
    oss << "    \"housingCapacity\": " << s.housingCapacity << ",\n";
    oss << "    \"jobsCapacity\": " << s.jobsCapacity << ",\n";
    oss << "    \"jobsCapacityAccessible\": " << s.jobsCapacityAccessible << ",\n";
    oss << "    \"employed\": " << s.employed << ",\n";
    oss << "    \"happiness\": " << s.happiness << ",\n";
    oss << "    \"money\": " << s.money << ",\n";
    oss << "    \"roads\": " << s.roads << ",\n";
    oss << "    \"parks\": " << s.parks << ",\n";
    oss << "    \"avgCommuteTime\": " << s.avgCommuteTime << ",\n";
    oss << "    \"trafficCongestion\": " << s.trafficCongestion << ",\n";
    oss << "    \"goodsDemand\": " << s.goodsDemand << ",\n";
    oss << "    \"goodsDelivered\": " << s.goodsDelivered << ",\n";
    oss << "    \"goodsSatisfaction\": " << s.goodsSatisfaction << ",\n";
    oss << "    \"avgLandValue\": " << s.avgLandValue << ",\n";
    oss << "    \"demandResidential\": " << s.demandResidential << "\n";
    oss << "  }\n";
    oss << "}\n";

    std::ofstream f(caseDir / "summary.json", std::ios::binary);
    if (!f) {
      outErr = "failed to write summary.json";
      return false;
    }
    f << oss.str();
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

  std::string outDir;
  std::string jsonReport;
  std::string junitPath;

  int shardIndex = 0;
  int shardCount = 1;

  GoldenConfig golden;

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

  std::vector<CaseResult> results;
  results.reserve(cases.size());

  int passed = 0;
  int failed = 0;

  for (std::size_t i = 0; i < cases.size(); ++i) {
    const ScenarioCase& sc = cases[i];
    ScenarioRunOutputs out;
    std::string runErr;

    runOpt.runIndex = static_cast<int>(i);
    const bool runOk = RunScenario(sc, runOpt, out, runErr);

    CaseResult cr;
    cr.sc = sc;
    cr.hash = out.finalHash;

    fs::path caseDir;
    const bool wantArtifacts = !outDir.empty() && runOk;
    if (wantArtifacts) {
      const fs::path base = fs::path(outDir);
      const fs::path name = SanitizeName(fs::path(sc.path).stem().string());
      std::ostringstream sub;
      sub << std::setw(4) << std::setfill('0') << i << "_" << name;
      caseDir = base / sub.str();

      std::string artErr;
      if (!WriteCaseArtifacts(caseDir, out, sc, artErr)) {
        std::cerr << "  [WARN] artifact write failed: " << artErr << "\n";
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
            if (isocity::ReadPpm(goldenPath.string(), expected, readErr)) {
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
              if (!isocity::WritePpm(goldenPath.string(), actual, werr)) {
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
            if (!isocity::ReadPpm(goldenPath.string(), expected, readErr)) {
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
          const isocity::PpmImage* diffPtr = (!diff.rgb.empty()) ? &diff : nullptr;
          if (!WriteGoldenArtifacts(caseDir, golden, cr.golden, actual, diffPtr, gerr)) {
            std::cerr << "  [WARN] golden artifact write failed: " << gerr << "\n";
          }
        }
      }
    }

    const bool ok = runOk && (!golden.enabled || cr.golden.ok);

    cr.ok = ok;
    cr.error = runErr;
    if (golden.enabled && runOk && !cr.golden.ok) {
      if (!cr.error.empty()) cr.error += " | ";
      cr.error += cr.golden.error;
    }

    if (ok) {
      ++passed;
      std::cout << "[PASS] " << sc.path << "  " << HexU64(out.finalHash);
      if (golden.enabled && cr.golden.attempted && golden.update && cr.golden.updated) {
        std::cout << "  [golden updated]";
      }
      std::cout << "\n";
    } else {
      ++failed;
      std::cout << "[FAIL] " << sc.path << "\n";
      std::cout << "       " << cr.error << "\n";
      if (failFast) {
        results.push_back(std::move(cr));
        break;
      }
    }

    results.push_back(std::move(cr));
  }

  // Suite JSON report.
  if (!jsonReport.empty()) {
    std::ofstream f(jsonReport, std::ios::binary);
    if (!f) {
      std::cerr << "failed to write json report: " << jsonReport << "\n";
    } else {
      f << "{\n";
      f << "  \"total\": " << results.size() << ",\n";
      f << "  \"passed\": " << passed << ",\n";
      f << "  \"failed\": " << failed << ",\n";
      f << "  \"cases\": [\n";
      for (std::size_t i = 0; i < results.size(); ++i) {
        const CaseResult& r = results[i];
        f << "    {\"path\": \"" << EscapeJson(r.sc.path) << "\", \"kind\": \"" << KindName(r.sc.kind)
          << "\", \"ok\": " << (r.ok ? "true" : "false")
          << ", \"hash\": \"" << HexU64(r.hash) << "\", \"error\": \"" << EscapeJson(r.error) << "\"";
        if (golden.enabled) {
          f << ", \"golden\": {\"attempted\": " << (r.golden.attempted ? "true" : "false")
            << ", \"ok\": " << (r.golden.ok ? "true" : "false")
            << ", \"updated\": " << (r.golden.updated ? "true" : "false")
            << ", \"matched\": " << (r.golden.matched ? "true" : "false")
            << ", \"goldenPath\": \"" << EscapeJson(r.golden.goldenPath) << "\""
            << ", \"error\": \"" << EscapeJson(r.golden.error) << "\"}";
        }
        f << "}";
        if (i + 1 < results.size()) f << ',';
        f << "\n";
      }
      f << "  ]\n";
      f << "}\n";
    }
  }

  // JUnit report.
  if (!junitPath.empty()) {
    std::ofstream f(junitPath, std::ios::binary);
    if (!f) {
      std::cerr << "failed to write junit report: " << junitPath << "\n";
    } else {
      f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
      f << "<testsuite name=\"ProcIsoCitySuite\" tests=\"" << results.size() << "\" failures=\"" << failed << "\">\n";
      for (const CaseResult& r : results) {
        f << "  <testcase classname=\"" << KindName(r.sc.kind) << "\" name=\"" << EscapeXml(r.sc.path) << "\">\n";
        if (!r.ok) {
          f << "    <failure message=\"" << EscapeXml(r.error) << "\"/>\n";
        }
        f << "  </testcase>\n";
      }
      f << "</testsuite>\n";
    }
  }

  std::cout << "\nSuite results: " << passed << " passed, " << failed << " failed (" << results.size() << " total)\n";

  return (failed == 0) ? 0 : 1;
}
