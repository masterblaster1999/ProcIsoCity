#include "isocity/Game.hpp"
#include "isocity/Random.hpp"
#include "isocity/AppPaths.hpp"
#include "isocity/CrashHandler.hpp"
#include "isocity/HealthCheck.hpp"
#include "isocity/LogTee.hpp"
#include "isocity/RaylibLog.hpp"
#include "isocity/RenderPipeline.hpp"
#include "isocity/CliManifest.hpp"
#include "isocity/FileHash.hpp"
#include "isocity/SaveDiscovery.hpp"
#include "isocity/SessionLock.hpp"
#include "isocity/ShaderUtil.hpp"
#include "isocity/SupportBundle.hpp"
#include "isocity/Version.hpp"

#if defined(PROCISOCITY_HAS_EMBEDDED_CLI) && PROCISOCITY_HAS_EMBEDDED_CLI
#include "cli/CliMain.hpp"
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

static std::string JoinArgs(int argc, char** argv)
{
  std::ostringstream oss;
  for (int i = 0; i < argc; ++i) {
    if (i) oss << ' ';
    const char* a = argv[i] ? argv[i] : "";
    // Minimal quoting for spaces.
    const bool needQuote = (std::string(a).find(' ') != std::string::npos);
    if (needQuote) oss << '"';
    oss << a;
    if (needQuote) oss << '"';
  }
  return oss.str();
}


static std::string TimestampUtcForFilename()
{
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const std::time_t t = clock::to_time_t(now);
  std::tm tm{};

#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02dZ",
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec);
  return std::string(buf);
}


} // namespace

static bool StartsWith(const std::string& s, const std::string& prefix)
{
  return s.rfind(prefix, 0) == 0;
}

static std::uint64_t ParseU64(const std::string& s)
{
  // Supports: 1234, 0x1234
  std::size_t idx = 0;
  int base = 10;
  if (StartsWith(s, "0x") || StartsWith(s, "0X")) {
    base = 16;
    idx = 2;
  }
  std::uint64_t value = 0;
  for (; idx < s.size(); ++idx) {
    const char c = s[idx];
    int digit = -1;
    if (c >= '0' && c <= '9') digit = (c - '0');
    else if (base == 16 && c >= 'a' && c <= 'f') digit = 10 + (c - 'a');
    else if (base == 16 && c >= 'A' && c <= 'F') digit = 10 + (c - 'A');
    else break;
    value = value * static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(digit);
  }
  return value;
}

static bool TryParseHashU64(const std::string& s, std::uint64_t& out)
{
  out = 0;

  std::string_view sv(s);

  // Trim common ASCII whitespace (manifests should not contain it, but be robust).
  while (!sv.empty()) {
    const unsigned char c = static_cast<unsigned char>(sv.front());
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') sv.remove_prefix(1);
    else break;
  }
  while (!sv.empty()) {
    const unsigned char c = static_cast<unsigned char>(sv.back());
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') sv.remove_suffix(1);
    else break;
  }

  if (sv.empty()) return false;

  int base = 10;

  // Accept both "0x..." (CLI) and raw hex without prefix (manifest upserts).
  if (sv.size() >= 2 && sv[0] == '0' && (sv[1] == 'x' || sv[1] == 'X')) {
    base = 16;
    sv.remove_prefix(2);
  } else {
    for (const char c : sv) {
      if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
        base = 16;
        break;
      }
    }
  }

  if (sv.empty()) return false;

  std::uint64_t value = 0;
  const std::uint64_t max = std::numeric_limits<std::uint64_t>::max();

  for (const char c : sv) {
    int digit = -1;
    if (c >= '0' && c <= '9') digit = (c - '0');
    else if (base == 16 && c >= 'a' && c <= 'f') digit = 10 + (c - 'a');
    else if (base == 16 && c >= 'A' && c <= 'F') digit = 10 + (c - 'A');
    else return false;

    if (value > (max - static_cast<std::uint64_t>(digit)) / static_cast<std::uint64_t>(base)) {
      return false; // overflow
    }
    value = value * static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(digit);
  }

  out = value;
  return true;
}


static bool ParseWxH(const std::string& s, int& outW, int& outH)
{
  auto pos = s.find('x');
  if (pos == std::string::npos) pos = s.find('X');
  if (pos == std::string::npos) return false;

  try {
    outW = std::stoi(s.substr(0, pos));
    outH = std::stoi(s.substr(pos + 1));
    return outW > 0 && outH > 0;
  } catch (...) {
    return false;
  }
}

static float ParseFloat(const std::string& s, float fallback)
{
  try {
    // std::stof accepts leading/trailing whitespace and stops at first invalid char.
    return std::stof(s);
  } catch (...) {
    return fallback;
  }
}

static int ParseInt(const std::string& s, int fallback)
{
  try {
    return std::stoi(s);
  } catch (...) {
    return fallback;
  }
}

int main(int argc, char** argv)
{
  // Initialize path helpers early so portable mode and shader override search can
  // work reliably regardless of the current working directory.
  isocity::AppPaths::Init((argc > 0) ? argv[0] : nullptr);

  // Capture the invocation working directory before we potentially chdir into the
  // data directory. Tool modes (render export, manifest loaders) should interpret
  // relative paths against the invocation CWD, not the per-user data dir.
  std::filesystem::path invocationCwd;
  {
    std::error_code ec;
    invocationCwd = std::filesystem::current_path(ec);
    if (ec) invocationCwd.clear();
  }


  // --- Multi-tool entrypoint ---
  //
  // When built with the embedded CLI (PROCISOCITY_HAS_EMBEDDED_CLI=1), the interactive
  // `proc_isocity` executable can also run the headless CLI toolchain:
  //
  //   proc_isocity cli --help
  //   proc_isocity cli --seed 1 --size 128x128 --days 120 --export-iso overlay out.ppm
  //
  // This makes it easier to ship a single "do everything" binary.
  bool wantCli = false;
  bool wantPipeline = false;
  if (argc >= 2) {
    const std::string a1 = argv[1] ? argv[1] : "";
    wantCli = (a1 == "cli" || a1 == "--cli");
    wantPipeline = (a1 == "pipeline" || a1 == "--pipeline");
  }

  if (wantCli) {
#if defined(PROCISOCITY_HAS_EMBEDDED_CLI) && PROCISOCITY_HAS_EMBEDDED_CLI
    std::vector<char*> forwarded;
    forwarded.reserve(static_cast<std::size_t>(argc));

    // Present as the same program name, but drop the dispatch token ("cli" / "--cli").
    forwarded.push_back(argv[0]);
    for (int i = 2; i < argc; ++i) {
      forwarded.push_back(argv[i]);
    }

    return isocity::ProcIsoCityCliMain(static_cast<int>(forwarded.size()), forwarded.data());
#else
    std::cerr << "This build of proc_isocity does not include the embedded CLI.\n"
              << "Reconfigure with PROCISOCITY_BUILD_CLI=ON (and rebuild), or run proc_isocity_cli.\n";
    return 2;
#endif
  }


  if (wantPipeline) {
#if defined(PROCISOCITY_HAS_EMBEDDED_CLI) && PROCISOCITY_HAS_EMBEDDED_CLI
    struct PipelineOptions {
      std::filesystem::path dir;
      std::string runTag;

      std::string saveTemplate;
      std::string manifestTemplate;

      bool render = true;
      std::string renderTemplate;
      int renderMaxSize = 4096;
      float renderTimeSec = 0.0f;
      bool renderScreenFx = true;

      bool updateManifest = true;
      bool skipExisting = true;

      bool verifySave = true;
      bool verifySaveStrict = false;

      bool ignorePrefs = false;
      bool safeMode = false;
      std::string prefsPathOverride;

      bool verbose = false;
    };

    PipelineOptions opt;
    opt.runTag = TimestampUtcForFilename();
    opt.dir = invocationCwd.empty() ? std::filesystem::path{} : invocationCwd;

    // Default to unique, collision-resistant filenames so repeated runs don't clobber outputs.
    opt.saveTemplate = "pipeline_" + opt.runTag + "_save_{seed}_{run}.bin";
    opt.manifestTemplate = "pipeline_" + opt.runTag + "_manifest_{seed}_{run}.json";
    opt.renderTemplate = "pipeline_" + opt.runTag + "_overview_{seed}_{run}.png";

    auto PrintPipelineHelp = [&]() {
      std::cout
          << "proc_isocity pipeline v" << isocity::ProcIsoCityFullVersionString() << "\n"
          << "End-to-end headless -> rendered bridge: run the embedded CLI, then GPU-render overview images.\n\n"
          << "Usage:\n"
          << "  proc_isocity pipeline [pipeline options] [--] [proc_isocity_cli args...]\n\n"
          << "Pipeline options:\n"
          << "  --pipeline-dir <dir>                 Output directory (used as working dir for both stages).\n"
          << "  --pipeline-save <template>           CLI --save path template (default: " << opt.saveTemplate << ")\n"
          << "  --pipeline-manifest <template>       CLI --manifest path template (default: " << opt.manifestTemplate << ")\n"
          << "  --pipeline-render <template>         Render output template (default: " << opt.renderTemplate << ")\n"
          << "  --pipeline-no-render                 Run CLI only; skip rendering.\n"
          << "  --pipeline-render-max-size <N>       Max output dimension in pixels (default: 4096)\n"
          << "  --pipeline-render-time <sec>         Time-of-day for screen FX (default: 0)\n"
          << "  --pipeline-render-screenfx           Enable screen FX (default)\n"
          << "  --pipeline-render-no-screenfx        Disable screen FX\n"
          << "  --pipeline-update-manifest           Upsert render output into the CLI manifest (default)\n"
          << "  --pipeline-no-update-manifest        Do not touch manifests after rendering\n"
          << "  --pipeline-skip-existing             Skip rendering when output already exists (default)\n"
          << "  --pipeline-force                     Re-render even if output exists\n"
          << "  --pipeline-verify-save               Verify manifest save hash/size before render (default)\n"
          << "  --pipeline-no-verify-save            Skip save verification\n"
          << "  --pipeline-verify-save-strict        Fail on hash/size mismatch\n"
          << "  --pipeline-verbose                   Print extra pipeline diagnostics\n\n"
          << "Render prefs options:\n"
          << "  --prefs <path>                       Override visual prefs path for the renderer\n"
          << "  --ignore-prefs                       Do not load visual prefs\n"
          << "  --safe                               Safe-mode (also implies --ignore-prefs)\n\n"
          << "Notes:\n"
          << "  - Arguments after '--' are forwarded to proc_isocity_cli unchanged.\n"
          << "  - If you omit '--', the first unrecognized flag will be treated as a CLI flag and forwarded.\n\n"
          << "Example:\n"
          << "  proc_isocity pipeline --pipeline-dir out -- --seed 123 --size 160x160 --days 250\n";
    };

    std::vector<std::string> forwardedCliArgs;
    bool forwarding = false;

    for (int i = 2; i < argc; ++i) {
      const std::string a = argv[i] ? argv[i] : "";

      if (!forwarding && a == "--") {
        forwarding = true;
        continue;
      }

      if (!forwarding) {
        if (a == "--help" || a == "-h") {
          PrintPipelineHelp();
          return 0;
        } else if (a == "--pipeline-dir") {
          if (i + 1 >= argc) {
            std::cerr << "--pipeline-dir requires a path\n";
            return 2;
          }
          opt.dir = std::filesystem::path(argv[++i] ? argv[i] : "");
          continue;
        } else if (a == "--pipeline-save") {
          if (i + 1 >= argc) {
            std::cerr << "--pipeline-save requires a template\n";
            return 2;
          }
          opt.saveTemplate = argv[++i] ? argv[i] : "";
          continue;
        } else if (a == "--pipeline-manifest") {
          if (i + 1 >= argc) {
            std::cerr << "--pipeline-manifest requires a template\n";
            return 2;
          }
          opt.manifestTemplate = argv[++i] ? argv[i] : "";
          continue;
        } else if (a == "--pipeline-render") {
          if (i + 1 >= argc) {
            std::cerr << "--pipeline-render requires a template\n";
            return 2;
          }
          opt.renderTemplate = argv[++i] ? argv[i] : "";
          opt.render = true;
          continue;
        } else if (a == "--pipeline-no-render") {
          opt.render = false;
          continue;
        } else if (a == "--pipeline-render-max-size") {
          if (i + 1 >= argc) {
            std::cerr << "--pipeline-render-max-size requires a number\n";
            return 2;
          }
          opt.renderMaxSize = ParseInt(argv[++i] ? argv[i] : "", opt.renderMaxSize);
          continue;
        } else if (a == "--pipeline-render-time") {
          if (i + 1 >= argc) {
            std::cerr << "--pipeline-render-time requires a number\n";
            return 2;
          }
          opt.renderTimeSec = ParseFloat(argv[++i] ? argv[i] : "", opt.renderTimeSec);
          continue;
        } else if (a == "--pipeline-render-screenfx") {
          opt.renderScreenFx = true;
          continue;
        } else if (a == "--pipeline-render-no-screenfx") {
          opt.renderScreenFx = false;
          continue;
        } else if (a == "--pipeline-update-manifest") {
          opt.updateManifest = true;
          continue;
        } else if (a == "--pipeline-no-update-manifest") {
          opt.updateManifest = false;
          continue;
        } else if (a == "--pipeline-skip-existing") {
          opt.skipExisting = true;
          continue;
        } else if (a == "--pipeline-force") {
          opt.skipExisting = false;
          continue;
        } else if (a == "--pipeline-verify-save") {
          opt.verifySave = true;
          continue;
        } else if (a == "--pipeline-no-verify-save") {
          opt.verifySave = false;
          continue;
        } else if (a == "--pipeline-verify-save-strict") {
          opt.verifySave = true;
          opt.verifySaveStrict = true;
          continue;
        } else if (a == "--pipeline-verbose") {
          opt.verbose = true;
          continue;
        } else if (a == "--prefs") {
          if (i + 1 >= argc) {
            std::cerr << "--prefs requires a path\n";
            return 2;
          }
          opt.prefsPathOverride = argv[++i] ? argv[i] : "";
          continue;
        } else if (a == "--ignore-prefs" || a == "--no-prefs") {
          opt.ignorePrefs = true;
          continue;
        } else if (a == "--safe" || a == "--safe-mode") {
          opt.safeMode = true;
          opt.ignorePrefs = true;
          continue;
        }

        // Unknown flag: assume the remaining args are intended for the embedded CLI.
        forwarding = true;
      }

      if (forwarding) {
        forwardedCliArgs.push_back(a);
      }
    }

    // Resolve the pipeline output dir relative to the invocation CWD.
    std::filesystem::path pipelineDir = opt.dir;
    if (pipelineDir.empty()) {
      std::error_code ec;
      pipelineDir = std::filesystem::current_path(ec);
      if (ec) pipelineDir.clear();
      if (pipelineDir.empty() && !invocationCwd.empty()) pipelineDir = invocationCwd;
    }
    if (pipelineDir.is_relative() && !invocationCwd.empty()) {
      pipelineDir = invocationCwd / pipelineDir;
    }

    std::error_code dec;
    std::filesystem::create_directories(pipelineDir, dec);
    if (dec) {
      std::cerr << "Failed to create pipeline dir '" << pipelineDir.string() << "': " << dec.message() << "\n";
      return 4;
    }

    // Resolve prefs override relative to the invocation CWD (not the pipeline dir).
    std::filesystem::path prefsPath;
    if (!opt.prefsPathOverride.empty()) {
      prefsPath = std::filesystem::path(opt.prefsPathOverride);
      if (prefsPath.is_relative() && !invocationCwd.empty()) prefsPath = invocationCwd / prefsPath;
    }

    // Convert the manifest template into an absolute path so we can later locate the emitted files.
    std::filesystem::path manifestTemplateAbs = std::filesystem::path(opt.manifestTemplate);
    if (manifestTemplateAbs.is_relative()) manifestTemplateAbs = pipelineDir / manifestTemplateAbs;
    std::filesystem::path manifestDirAbs = manifestTemplateAbs.parent_path();
    if (manifestDirAbs.empty()) manifestDirAbs = pipelineDir;

    // Detect whether we can filter output manifests by the pipeline's unique run tag.
    const bool filterByTag = (!opt.runTag.empty() && opt.manifestTemplate.find(opt.runTag) != std::string::npos);

    // Build CLI argv (we auto-inject --manifest/--save if absent).
    std::vector<std::string> cliArgs;
    cliArgs.reserve(1 + forwardedCliArgs.size() + 8);
    cliArgs.emplace_back("proc_isocity_cli");
    for (const auto& a : forwardedCliArgs) {
      cliArgs.push_back(a);
    }

    auto HasFlag = [](const std::vector<std::string>& args, const char* flag) -> bool {
      for (std::size_t i = 1; i < args.size(); ++i) {
        if (args[i] == flag) return true;
      }
      return false;
    };

    if (!HasFlag(cliArgs, "--manifest")) {
      cliArgs.push_back("--manifest");
      cliArgs.push_back(opt.manifestTemplate);
    }
    if (!HasFlag(cliArgs, "--save")) {
      cliArgs.push_back("--save");
      cliArgs.push_back(opt.saveTemplate);
    }

    // Clamp a couple of obvious footguns.
    opt.renderMaxSize = std::max(64, opt.renderMaxSize);

    std::cout << "proc_isocity pipeline (" << isocity::ProcIsoCityFullVersionString() << ")\n"
              << "  dir: " << pipelineDir.string() << "\n";

    if (opt.verbose) {
      std::cout << "  embedded_cli: yes\n";
      std::cout << "  cli_args: ";
      for (std::size_t i = 0; i < cliArgs.size(); ++i) {
        if (i) std::cout << ' ';
        std::cout << cliArgs[i];
      }
      std::cout << "\n";
    }

    struct ScopedCurrentDir {
      std::filesystem::path oldDir;
      bool ok = false;
      explicit ScopedCurrentDir(const std::filesystem::path& newDir) {
        std::error_code ec;
        oldDir = std::filesystem::current_path(ec);
        if (ec) return;
        std::filesystem::current_path(newDir, ec);
        if (ec) return;
        ok = true;
      }
      ~ScopedCurrentDir() {
        if (!ok) return;
        std::error_code ec;
        std::filesystem::current_path(oldDir, ec);
      }
    };

    ScopedCurrentDir scoped(pipelineDir);
    if (!scoped.ok) {
      std::cerr << "Failed to change working directory to pipeline dir: " << pipelineDir.string() << "\n";
      return 4;
    }

    const auto tBefore = std::filesystem::file_time_type::clock::now();

    // Run the embedded CLI.
    std::vector<char*> cliArgv;
    cliArgv.reserve(cliArgs.size());
    for (auto& s : cliArgs) cliArgv.push_back(s.data());

    const int cliExit = isocity::ProcIsoCityCliMain(static_cast<int>(cliArgv.size()), cliArgv.data());
    if (cliExit != 0) {
      std::cerr << "pipeline: embedded CLI failed with exit code " << cliExit << "\n";
      return cliExit;
    }

    if (!opt.render) {
      std::cout << "pipeline: render disabled (--pipeline-no-render)\n";
      return 0;
    }

    struct ProducedManifest {
      std::filesystem::path path;
      std::filesystem::file_time_type t;
      isocity::CliRunManifest manifest;
    };

    std::vector<ProducedManifest> produced;

    const auto threshold = tBefore - std::chrono::seconds(2);

    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(manifestDirAbs, ec)) {
      if (ec) break;
      if (!entry.is_regular_file(ec) || ec) {
        ec.clear();
        continue;
      }
      const std::filesystem::path p = entry.path();
      if (p.extension() != ".json") continue;

      std::error_code ec2;
      const auto wt = std::filesystem::last_write_time(p, ec2);
      if (ec2) continue;

      const bool timeOk = (wt >= threshold);
      bool nameOk = false;
      if (filterByTag && !opt.runTag.empty()) {
        const std::string fn = p.filename().string();
        nameOk = (fn.find(opt.runTag) != std::string::npos);
      }

      if (!timeOk && !nameOk) continue;

      isocity::CliRunManifest m;
      std::string mErr;
      if (!isocity::LoadCliRunManifest(p, m, mErr)) {
        continue; // not a CLI manifest
      }
      if (!m.tool.empty() && m.tool != "proc_isocity_cli") continue;

      produced.push_back(ProducedManifest{p, wt, std::move(m)});
    }

    if (produced.empty()) {
      // Fall back to "newest manifest" (useful if clocks are weird or the run was extremely fast).
      std::filesystem::path latest;
      std::string findErr;
      if (!isocity::FindLatestCliRunManifestInDir(manifestDirAbs, latest, findErr)) {
        std::cerr << "pipeline: could not locate any proc_isocity_cli manifest in: " << manifestDirAbs.string() << "\n";
        if (!findErr.empty()) std::cerr << findErr << "\n";
        return 6;
      }

      isocity::CliRunManifest m;
      std::string mErr;
      if (!isocity::LoadCliRunManifest(latest, m, mErr)) {
        std::cerr << "pipeline: failed to load manifest: " << latest.string() << "\n";
        if (!mErr.empty()) std::cerr << mErr << "\n";
        return 6;
      }

      produced.push_back(ProducedManifest{latest, std::filesystem::file_time_type{}, std::move(m)});
    }

    std::sort(produced.begin(), produced.end(), [](const ProducedManifest& a, const ProducedManifest& b) {
      if (a.manifest.runIndex != b.manifest.runIndex) return a.manifest.runIndex < b.manifest.runIndex;
      return a.manifest.actualSeed < b.manifest.actualSeed;
    });

    int okCount = 0;
    int skipCount = 0;
    int failCount = 0;

    for (const auto& pm : produced) {
      const isocity::CliManifestArtifact* saveArt = isocity::FindFirstArtifactByKind(pm.manifest, "save");
      if (!saveArt) {
        std::cerr << "pipeline: manifest missing 'save' artifact: " << pm.path.string() << "\n";
        ++failCount;
        continue;
      }

      std::string dbg;
      const std::filesystem::path savePath =
          isocity::ResolveManifestArtifactPathSmart(pm.path, pm.manifest, saveArt->path, invocationCwd, &dbg);

      std::error_code ec3;
      const bool saveExists = std::filesystem::exists(savePath, ec3) && !ec3;
      if (!saveExists) {
        std::cerr << "pipeline: save artifact missing on disk for manifest: " << pm.path.string() << "\n";
        if (!dbg.empty()) std::cerr << dbg;
        ++failCount;
        continue;
      }

      if (opt.verifySave && (!saveArt->hashFNV1a64Hex.empty() || saveArt->sizeBytes != 0)) {
        isocity::FileHashInfo info;
        std::string herr;
        if (!isocity::ComputeFileHashFNV1a64(savePath.string(), info, herr)) {
          std::cerr << "pipeline: warning: failed to hash save '" << savePath.string() << "': " << herr << "\n";
        } else {
          bool mismatch = false;
          if (saveArt->sizeBytes != 0 && info.sizeBytes != saveArt->sizeBytes) mismatch = true;

          std::uint64_t expectedHash = 0;
          if (!saveArt->hashFNV1a64Hex.empty()) {
            std::uint64_t parsed = 0;
            if (TryParseHashU64(saveArt->hashFNV1a64Hex, parsed)) {
              expectedHash = parsed;
              if (expectedHash != info.fnv1a64) mismatch = true;
            }
          }

          if (mismatch) {
            std::cerr << "pipeline: warning: manifest save hash/size mismatch for: " << pm.path.string() << "\n"
                      << "  save: " << savePath.string() << "\n"
                      << "  manifest.size_bytes=" << saveArt->sizeBytes << "  disk.size_bytes=" << info.sizeBytes << "\n"
                      << "  manifest.hash_fnv1a64=" << saveArt->hashFNV1a64Hex << "  disk.hash_fnv1a64=0x"
                      << std::hex << info.fnv1a64 << std::dec << "\n";
            if (opt.verifySaveStrict) {
              ++failCount;
              continue;
            }
          }
        }
      }

      const std::string outExpanded = isocity::ExpandCliManifestTemplate(opt.renderTemplate, pm.manifest);
      std::filesystem::path outPath = std::filesystem::path(outExpanded);

      // Write relative outputs next to the manifest to keep artifacts grouped.
      if (outPath.is_relative()) {
        const std::filesystem::path baseDir = pm.path.parent_path();
        if (!baseDir.empty()) outPath = baseDir / outPath;
      }

      if (opt.verbose) {
        std::cout << "pipeline: render seed=" << pm.manifest.actualSeed << " run=" << pm.manifest.runIndex << "\n"
                  << "  manifest: " << pm.path.string() << "\n"
                  << "  save    : " << savePath.string() << "\n"
                  << "  out     : " << outPath.string() << "\n";
      }

      const bool outExists = std::filesystem::exists(outPath, ec3) && !ec3;

      if (opt.skipExisting && outExists) {
        ++skipCount;

        if (opt.updateManifest) {
          isocity::CliManifestArtifact a;
          a.kind = "render_overview";
          a.layer = opt.renderScreenFx ? "gpu_fx" : "gpu";
          a.path = outExpanded;

          // If the template omitted an extension, record the actual extension.
          if (!a.path.empty()) {
            const std::filesystem::path rp = std::filesystem::path(a.path);
            if (rp.extension().empty() && !outPath.extension().empty()) {
              a.path += outPath.extension().string();
            }
          }

          std::string uErr;
          if (!isocity::UpsertCliRunManifestArtifact(pm.path, a, outPath, uErr, true)) {
            std::cerr << "pipeline: warning: failed to upsert existing render output into manifest: " << pm.path.string()
                      << "\n";
            if (!uErr.empty()) std::cerr << uErr << "\n";
          }
        }

        continue;
      }

      isocity::RenderOverviewOptions ro;
      ro.savePath = savePath;
      ro.outImagePath = outPath;
      ro.maxSize = opt.renderMaxSize;
      ro.timeSec = opt.renderTimeSec;
      ro.includeScreenFx = opt.renderScreenFx;
      ro.useVisualPrefs = !opt.ignorePrefs;
      ro.visualPrefsPath = prefsPath;
      ro.safeMode = opt.safeMode;

      // Reasonable defaults for tool mode; avoid massive window allocation.
      ro.tileWidth = 64;
      ro.tileHeight = 32;
      ro.elevationScale = 0.75f;
      ro.elevationSteps = 16;
      ro.windowWidth = 640;
      ro.windowHeight = 640;
      ro.hiddenWindow = true;

      isocity::RenderOverviewResult roRes;
      std::string roErr;
      const bool ok = isocity::RenderWorldOverviewFromSave(ro, roRes, roErr);
      if (!roRes.report.empty()) {
        std::cout << roRes.report;
      }
      if (!ok) {
        std::cerr << "pipeline: failed to render overview for manifest: " << pm.path.string() << "\n";
        if (!roErr.empty()) std::cerr << roErr << "\n";
        ++failCount;
        continue;
      }

      ++okCount;

      const std::filesystem::path finalOutPath = roRes.outImagePath.empty() ? outPath : roRes.outImagePath;

      if (opt.updateManifest) {
        isocity::CliManifestArtifact a;
        a.kind = "render_overview";
        a.layer = opt.renderScreenFx ? "gpu_fx" : "gpu";
        a.path = outExpanded;

        if (!a.path.empty()) {
          const std::filesystem::path rp = std::filesystem::path(a.path);
          if (rp.extension().empty() && !finalOutPath.extension().empty()) {
            a.path += finalOutPath.extension().string();
          }
        }

        std::string uErr;
        if (!isocity::UpsertCliRunManifestArtifact(pm.path, a, finalOutPath, uErr, true)) {
          std::cerr << "pipeline: warning: failed to update manifest with rendered overview: " << pm.path.string()
                    << "\n";
          if (!uErr.empty()) std::cerr << uErr << "\n";
        }
      }
    }

    std::cout << "pipeline render summary: ok=" << okCount << " skipped=" << skipCount << " failed=" << failCount
              << "\n";

    if (failCount == 0) return 0;
    return 7;
#else
    std::cerr << "This build of proc_isocity does not include the embedded CLI.\n"
              << "Reconfigure with PROCISOCITY_BUILD_CLI=ON (and rebuild), or run proc_isocity_cli.\n";
    return 2;
#endif
  }

  isocity::Config cfg;
  std::string dataDirOverride;
  bool portableData = false;
  bool noChdir = false;
  bool printDirs = false;
  bool diagnoseOnly = false;
  bool makeSupportBundle = false;
  bool makeSupportBundleZip = false;
  std::string supportBundleDirOverride;

  // Headless health check / smoke test
  bool healthCheck = false;
  std::string healthCheckDirOverride;
  int healthCheckW = 64;
  int healthCheckH = 64;
  std::uint64_t healthCheckSeed = 0;
  bool healthCheckSeedExplicit = false;
  int healthCheckSteps = 12;
  bool healthCheckKeep = false;
  bool healthCheckVerbose = false;

  // Optional: validate headless->rendered pipeline by loading the health-check
  // save and exporting a GPU-rendered overview image.
  bool healthCheckRender = false;
  int healthCheckRenderMaxSize = 2048;
  bool healthCheckRenderScreenFx = false;
  float healthCheckRenderTimeSec = 0.0f;

  // Non-interactive render tool: load a save and export an overview image.
  bool renderOverviewTool = false;
  bool renderOverviewFromManifest = false;
  bool renderOverviewFromManifestDir = false;
  std::string renderOverviewSavePath;
  std::string renderOverviewManifestPath;
  std::string renderOverviewOutPath;
  int renderOverviewMaxSize = 4096;
  bool renderOverviewScreenFx = true;
  float renderOverviewTimeSec = 0.0f;
  bool renderOverviewUpdateManifest = false;

  // Session/logging
  bool multiInstance = false;
  bool autoRecoverOnCrash = true;

  bool enableLog = true;
  std::string logPathOverride;
  int logKeepFiles = 3;
  bool logPlain = false;
  bool logThread = false;

  // raylib logging (interactive app)
  bool raylibLogEnabled = true;
  int raylibLogLevel = LOG_INFO;
  bool raylibLogLevelExplicit = false;

  // Startup options
  std::string loadSavePath;
  std::string loadManifestPath;
  bool dataDirFromManifest = false;
  bool resumeLatestSave = false;

  std::string prefsPathOverride;
  bool ignorePrefs = false;
  bool safeMode = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--diagnose") {
      diagnoseOnly = true;
      continue;
    }

    if (arg == "--support-bundle") {
      makeSupportBundle = true;
      continue;
    }

    if (arg == "--support-bundle-zip") {
      makeSupportBundleZip = true;
      continue;
    }

    if (arg == "--support-bundle-dir" && i + 1 < argc) {
      makeSupportBundle = true;
      supportBundleDirOverride = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--support-bundle-zip-dir" && i + 1 < argc) {
      makeSupportBundleZip = true;
      supportBundleDirOverride = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--health-check" || arg == "--healthcheck") {
      healthCheck = true;
      continue;
    }

    if (arg == "--health-check-dir" && i + 1 < argc) {
      healthCheck = true;
      healthCheckDirOverride = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--health-check-size" && i + 1 < argc) {
      healthCheck = true;
      int w = 0, h = 0;
      if (ParseWxH(argv[++i] ? argv[i] : "", w, h)) {
        healthCheckW = w;
        healthCheckH = h;
      }
      continue;
    }

    if (arg == "--health-check-seed" && i + 1 < argc) {
      healthCheck = true;
      healthCheckSeed = ParseU64(argv[++i] ? argv[i] : "");
      healthCheckSeedExplicit = true;
      continue;
    }

    if (arg == "--health-check-steps" && i + 1 < argc) {
      healthCheck = true;
      healthCheckSteps = std::max(0, ParseInt(argv[++i] ? argv[i] : "", healthCheckSteps));
      continue;
    }

    if (arg == "--health-check-keep") {
      healthCheck = true;
      healthCheckKeep = true;
      continue;
    }

    if (arg == "--health-check-verbose") {
      healthCheck = true;
      healthCheckVerbose = true;
      continue;
    }

    // Optional integration test: run the headless health check and then use the
    // renderer to export a GPU overview image from the produced save.
    if (arg == "--health-check-render") {
      healthCheck = true;
      healthCheckRender = true;
      continue;
    }

    if (arg == "--health-check-render-max-size" && i + 1 < argc) {
      healthCheck = true;
      healthCheckRender = true;
      healthCheckRenderMaxSize = std::max(64, ParseInt(argv[++i] ? argv[i] : "", healthCheckRenderMaxSize));
      continue;
    }

    if (arg == "--health-check-render-screenfx" || arg == "--health-check-render-fx") {
      healthCheck = true;
      healthCheckRender = true;
      healthCheckRenderScreenFx = true;
      continue;
    }

    if (arg == "--health-check-render-time" && i + 1 < argc) {
      healthCheck = true;
      healthCheckRender = true;
      healthCheckRenderTimeSec = ParseFloat(argv[++i] ? argv[i] : "", healthCheckRenderTimeSec);
      continue;
    }

    // Non-interactive renderer tool: load a save and export a world overview image.
    if (arg == "--render-overview-manifest-dir" && i + 2 < argc) {
      renderOverviewTool = true;
      renderOverviewFromManifest = true;
      renderOverviewFromManifestDir = true;
      renderOverviewManifestPath = argv[++i] ? argv[i] : "";
      renderOverviewOutPath = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--render-overview-manifest" && i + 2 < argc) {
      renderOverviewTool = true;
      renderOverviewFromManifest = true;
      renderOverviewFromManifestDir = false;
      renderOverviewManifestPath = argv[++i] ? argv[i] : "";
      renderOverviewOutPath = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--render-overview" && i + 2 < argc) {
      renderOverviewTool = true;
      renderOverviewFromManifest = false;
      renderOverviewFromManifestDir = false;
      renderOverviewSavePath = argv[++i] ? argv[i] : "";
      renderOverviewOutPath = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--render-overview-max-size" && i + 1 < argc) {
      renderOverviewMaxSize = std::max(64, ParseInt(argv[++i] ? argv[i] : "", renderOverviewMaxSize));
      continue;
    }

    if (arg == "--render-overview-screenfx" || arg == "--render-overview-fx") {
      renderOverviewScreenFx = true;
      continue;
    }

    if (arg == "--render-overview-no-screenfx" || arg == "--render-overview-no-fx") {
      renderOverviewScreenFx = false;
      continue;
    }

    if (arg == "--render-overview-time" && i + 1 < argc) {
      renderOverviewTimeSec = ParseFloat(argv[++i] ? argv[i] : "", renderOverviewTimeSec);
      continue;
    }

    if (arg == "--render-overview-update-manifest") {
      renderOverviewUpdateManifest = true;
      continue;
    }

    if (arg == "--render-overview-no-update-manifest") {
      renderOverviewUpdateManifest = false;
      continue;
    }


    if (arg == "--multi-instance") {
      multiInstance = true;
      continue;
    }

    if (arg == "--no-recover" || arg == "--no-autorecover") {
      autoRecoverOnCrash = false;
      continue;
    }

    if (arg == "--log" && i + 1 < argc) {
      enableLog = true;
      logPathOverride = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--log-plain") {
      logPlain = true;
      continue;
    }

    if (arg == "--log-thread") {
      logThread = true;
      continue;
    }

    if (arg == "--no-log") {
      enableLog = false;
      continue;
    }

    if (arg == "--raylib-log" && i + 1 < argc) {
      raylibLogLevel = isocity::ParseRaylibLogLevel(argv[++i] ? argv[i] : "", raylibLogLevel);
      raylibLogLevelExplicit = true;
      continue;
    }

    if (arg == "--no-raylib-log") {
      raylibLogEnabled = false;
      continue;
    }

    if (arg == "--log-keep" && i + 1 < argc) {
      logKeepFiles = std::max(0, ParseInt(argv[++i], logKeepFiles));
      continue;
    }

    if (arg == "--data-dir" && i + 1 < argc) {
      dataDirOverride = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--portable") {
      portableData = true;
      continue;
    }

    if (arg == "--no-chdir" || arg == "--cwd") {
      noChdir = true;
      continue;
    }

    if (arg == "--print-dirs") {
      printDirs = true;
      continue;
    }

    if (arg == "--load" && i + 1 < argc) {
      loadSavePath = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--load-manifest" && i + 1 < argc) {
      loadManifestPath = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--data-dir-from-manifest") {
      dataDirFromManifest = true;
      continue;
    }


    if (arg == "--resume" || arg == "--continue") {
      resumeLatestSave = true;
      continue;
    }

    if (arg == "--prefs" && i + 1 < argc) {
      prefsPathOverride = argv[++i] ? argv[i] : "";
      continue;
    }

    if (arg == "--ignore-prefs" || arg == "--no-prefs") {
      ignorePrefs = true;
      continue;
    }

    if (arg == "--safe" || arg == "--safe-mode") {
      safeMode = true;
      ignorePrefs = true;
      continue;
    }


    if (arg == "--help" || arg == "-h") {
      std::cout << "ProcIsoCity v" << isocity::ProcIsoCityFullVersionString() << "\n"
                << "  --version\n"
                << "  --diagnose         (print a startup/asset diagnostic report and exit)\n"
                << "  --health-check     (headless smoke test: procgen + sim + save/load; no renderer)\n"
                << "  --support-bundle   (collect logs/crash reports/prefs into a folder and exit)\n"
                << "  --support-bundle-zip (collect logs/crash reports/prefs into a .zip file and exit)\n"
                << "  --support-bundle-dir <path>  (write the support bundle into <path>)\n"
                << "  --support-bundle-zip-dir <path>  (write the support bundle .zip into <path>)\n"
                << "  --health-check-dir <path> (write health-check artifacts under <path>)\n"
                << "  --health-check-size <W>x<H> (default 64x64)\n"
                << "  --health-check-seed <u64|0xHEX> (default: random)\n"
                << "  --health-check-steps <N>  (default 12)\n"
                << "  --health-check-keep   (keep artifacts on disk)\n"
                << "  --health-check-verbose (extra timings/details)\n"
                << "  --health-check-render (after headless check, export a GPU overview image)\n"
                << "  --health-check-render-max-size <N> (default 2048)\n"
                << "  --health-check-render-screenfx (include fog/precip screen FX in the overview)\n"
                << "  --health-check-render-time <sec> (time parameter for day/night + weather; default 0)\n"

                << "  --render-overview <save.bin> <out.png> (non-interactive GPU render export and exit)\n"
                << "  --render-overview-manifest <manifest.json|dir> <out.png> (render the save referenced by a proc_isocity_cli manifest)\n"
                << "  --render-overview-manifest-dir <dir> <out_pattern> (render overviews for ALL proc_isocity_cli manifests in dir; supports {seed},{run},{w},{h},{days},{hash})\n"
                << "  --render-overview-update-manifest (record the generated image back into the manifest)\n"
                << "  --render-overview-max-size <N> (default 4096)\n"
                << "  --render-overview-screenfx|--render-overview-no-screenfx\n"
                << "  --render-overview-time <sec>\n"
                << "  --data-dir <path>  (store runtime data there; chdir before running)\n"
                << "  --portable         (store runtime data next to the executable in ./ProcIsoCityData)\n"
                << "  --no-chdir|--cwd    (do not change working directory; legacy behavior)\n"
                << "  --print-dirs        (print resolved directories and exit)\n"
                << "  --multi-instance   (disable the single-instance data-dir lock)\n"
                << "  --no-recover       (do not auto-resume on detected previous crash)\n"
                << "  --log <path>       (tee stdout/stderr into this log file)\n"
                << "  --no-log           (disable log file tee)\n"
                << "  --log-keep <N>     (keep N rotated backups; default 3)\n"
                << "  --log-plain        (log file without timestamps/tags; raw output)\n"
                << "  --log-thread       (include hashed thread id in log file prefixes)\n"
                << "  --raylib-log <lvl> (raylib TraceLog threshold: all|trace|debug|info|warn|error|fatal|none)\n"
                << "  --no-raylib-log    (disable forwarding raylib TraceLog into stderr/log file)\n"
                << "  --load <path>      (load a save on startup)\n"
                << "  --load-manifest <manifest.json|dir> (load the save referenced by a proc_isocity_cli manifest)\n"
                << "  --data-dir-from-manifest (with --load-manifest, use the manifest\'s directory as --data-dir)\n"
                << "  --resume           (load the most recently modified save/autosave in the data dir)\n"
                << "  --prefs <path>     (use this visual prefs file instead of isocity_visual.json)\n"
                << "  --ignore-prefs     (do not load visual prefs on startup)\n"
                << "  --safe-mode        (ignore prefs + force conservative graphics defaults)\n"
                << "  --seed <u64|0xHEX>\n"
                << "  --size <W>x<H>      (map size)\n"
                << "  --window <W>x<H>    (window size)\n"
                << "  --elev <scale>      (elevation scale multiplier, 0=flat; default 0.75)\n"
                << "  --elevsteps <N>     (0=smooth, otherwise quantize to N steps; default 16)\n"
                << "  --flat              (shortcut for --elev 0)\n"
                << "  --novsync\n";

      std::cout << "\nEnvironment overrides:\n"
                << "  PROCISOCITY_DATA_DIR   Default data directory (unless --data-dir/--portable/--no-chdir)\n"
                << "  PROCISOCITY_SHADER_DIR Explicit shader override directory (see shader_reload)\n"
                << "  PROCISOCITY_LOG_FILE   Default log file path (unless --log/--no-log)\n"
                << "  PROCISOCITY_RAYLIB_LOG raylib TraceLog threshold (unless --raylib-log/--no-raylib-log)\n";

#if defined(PROCISOCITY_HAS_EMBEDDED_CLI) && PROCISOCITY_HAS_EMBEDDED_CLI
      std::cout << "\n"
                << "Multi-tool mode (headless):\n"
                << "  cli [args...]        (run the headless CLI; same flags as proc_isocity_cli)\n"
                << "  --cli [args...]      (same as above)\n"
                << "  pipeline [opts] [--] [cli args...]\n"
                << "                      (run embedded CLI then GPU-render overview images)\n"
                << "\n"
                << "Example:\n"
                << "  proc_isocity cli --seed 1 --size 128x128 --days 120 --export-iso overlay out.ppm\n"
                << "  proc_isocity pipeline --pipeline-dir out -- --seed 1 --size 128x128 --days 120\n";
#endif

      return 0;
    }

    if (arg == "--version" || arg == "-V") {
      std::cout << "ProcIsoCity " << isocity::ProcIsoCityFullVersionString() << "\n";
      return 0;
    }

    if (arg == "--seed" && i + 1 < argc) {
      cfg.seed = ParseU64(argv[++i]);
      continue;
    }

    if (arg == "--size" && i + 1 < argc) {
      int w = 0, h = 0;
      if (ParseWxH(argv[++i], w, h)) {
        cfg.mapWidth = w;
        cfg.mapHeight = h;
      }
      continue;
    }

    if (arg == "--window" && i + 1 < argc) {
      int w = 0, h = 0;
      if (ParseWxH(argv[++i], w, h)) {
        cfg.windowWidth = w;
        cfg.windowHeight = h;
      }
      continue;
    }

    if (arg == "--novsync") {
      cfg.vsync = false;
      continue;
    }

    if (arg == "--elev" && i + 1 < argc) {
      cfg.elevationScale = ParseFloat(argv[++i], cfg.elevationScale);
      if (cfg.elevationScale < 0.0f) cfg.elevationScale = 0.0f;
      continue;
    }

    if (arg == "--elevsteps" && i + 1 < argc) {
      cfg.elevationSteps = std::max(0, ParseInt(argv[++i], cfg.elevationSteps));
      continue;
    }

    if (arg == "--flat") {
      cfg.elevationScale = 0.0f;
      continue;
    }
  }

  // Resolve --load-manifest early (before we potentially chdir) so we can optionally
  // derive the data directory from the manifest location.
  std::filesystem::path loadManifestResolvedPath;
  if (!loadManifestPath.empty()) {
    loadManifestResolvedPath = std::filesystem::path(loadManifestPath);
    if (loadManifestResolvedPath.is_relative() && !invocationCwd.empty()) {
      loadManifestResolvedPath = invocationCwd / loadManifestResolvedPath;
    }
    std::error_code ec;
    if (std::filesystem::is_directory(loadManifestResolvedPath, ec) && !ec) {
      std::filesystem::path latest;
      std::string findErr;
      if (isocity::FindLatestCliRunManifestInDir(loadManifestResolvedPath, latest, findErr)) {
        loadManifestResolvedPath = latest;
      }
    }
  }

  if (dataDirFromManifest && !loadManifestResolvedPath.empty() && dataDirOverride.empty() && !portableData) {
    const std::filesystem::path mdir = loadManifestResolvedPath.parent_path();
    if (!mdir.empty()) {
      dataDirOverride = mdir.string();
    }
  }

  // --- Data directory bootstrap (interactive app only) ---
  //
  // By default we chdir into a per-user data directory so saves, thumbnails and
  // blueprint libraries are always writable and don't pollute build folders.
  std::filesystem::path resolvedDataDir;
  const char* envDataDir = std::getenv("PROCISOCITY_DATA_DIR");
  if (!dataDirOverride.empty()) {
    resolvedDataDir = std::filesystem::path(dataDirOverride);
  } else if (portableData) {
    resolvedDataDir = isocity::AppPaths::PortableDataDir();
  } else if (envDataDir && *envDataDir) {
    resolvedDataDir = std::filesystem::path(envDataDir);
  } else {
    resolvedDataDir = isocity::AppPaths::UserDataDir();
  }

  // Print resolved directories and exit (useful for debugging installs).
  if (printDirs) {
    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);

    std::cout << "ProcIsoCity directories\n"
              << "  version:     " << isocity::ProcIsoCityFullVersionString() << "\n"
              << "  exe:         " << isocity::AppPaths::ExecutablePath().string() << "\n"
              << "  exe_dir:     " << isocity::AppPaths::ExecutableDir().string() << "\n"
              << "  cwd:         " << (ec ? std::string("(unknown)") : cwd.string()) << "\n"
              << "  user_data:   " << isocity::AppPaths::UserDataDir().string() << "\n"
              << "  user_config: " << isocity::AppPaths::UserConfigDir().string() << "\n"
              << "  user_cache:  " << isocity::AppPaths::UserCacheDir().string() << "\n"
              << "  portable:    " << isocity::AppPaths::PortableDataDir().string() << "\n"
              << "  resolved:    " << resolvedDataDir.string() << "\n"
              << "  will_chdir:  " << ((noChdir) ? "no" : "yes") << "\n";

    if (envDataDir && *envDataDir) {
      std::cout << "  env(PROCISOCITY_DATA_DIR): " << envDataDir << "\n";
    }
    const char* envShader = std::getenv("PROCISOCITY_SHADER_DIR");
    if (envShader && *envShader) {
      std::cout << "  env(PROCISOCITY_SHADER_DIR): " << envShader << "\n";
    }
    const char* envLogFile = std::getenv("PROCISOCITY_LOG_FILE");
    if (envLogFile && *envLogFile) {
      std::cout << "  env(PROCISOCITY_LOG_FILE): " << envLogFile << "\n";
    }

    const char* envRaylibLog = std::getenv("PROCISOCITY_RAYLIB_LOG");
    if (envRaylibLog && *envRaylibLog) {
      std::cout << "  env(PROCISOCITY_RAYLIB_LOG): " << envRaylibLog << "\n";
    }
    return 0;
  }

  // Enter the data directory unless explicitly disabled.
  if (!noChdir) {
    std::string err;
    if (!isocity::AppPaths::EnsureDirExists(resolvedDataDir, err)) {
      std::cerr << "Warning: failed to prepare data dir '" << resolvedDataDir.string() << "': " << err
                << "\n";
    } else {
      std::error_code ec;
      std::filesystem::current_path(resolvedDataDir, ec);
      if (ec) {
        std::cerr << "Warning: failed to chdir to data dir '" << resolvedDataDir.string() << "': " << ec.message()
                  << "\n";
      }
    }
  }

  // --- Single-instance session lock + crash marker ---
  //
  // This prevents multiple instances from writing to the same data directory,
  // and provides a best-effort signal when the previous run ended uncleanly.
  isocity::SessionLock sessionLock;
  bool prevUncleanShutdown = false;
  bool autoResumeAfterCrash = false;

  if (!multiInstance) {
    std::error_code ec;
    const std::filesystem::path lockDir = std::filesystem::current_path(ec);
    if (ec) {
      std::cerr << "Warning: cannot determine current directory for session lock: " << ec.message() << "\n";
    } else {
      isocity::SessionLockOptions opt;
      opt.dir = lockDir;
      opt.info.pid = isocity::SessionLock::CurrentPid();
      opt.info.startedUtc = isocity::SessionLock::UtcNowIso8601();
      opt.info.exePath = isocity::AppPaths::ExecutablePath().string();
      opt.info.buildStamp = "ProcIsoCity " + isocity::ProcIsoCityFullVersionString() + " | " + isocity::ProcIsoCityBuildStamp();

      std::string lockErr;
      const bool ok = sessionLock.acquire(opt, lockErr);
      if (!ok) {
        std::cerr << lockErr << "\n";
        return 3;
      }
      if (!lockErr.empty()) {
        std::cerr << lockErr << "\n";
      }
      prevUncleanShutdown = sessionLock.previousSessionUnclean();
    }
  }

  // --- Log file tee (stdout/stderr) ---
  //
  // This runs after the session lock to avoid two instances fighting over log rotation.
  isocity::LogTee logTee;
  std::filesystem::path resolvedLogPath;
  if (enableLog) {
    const char* envLogFile = std::getenv("PROCISOCITY_LOG_FILE");
    if (logPathOverride.empty() && envLogFile && *envLogFile) {
      logPathOverride = envLogFile;
    }

    if (!logPathOverride.empty()) {
      resolvedLogPath = std::filesystem::path(logPathOverride);
    } else {
      resolvedLogPath = std::filesystem::path("proc_isocity.log");
    }

    isocity::LogTeeOptions opt;
    opt.path = resolvedLogPath;
    opt.keepFiles = logKeepFiles;
    opt.prefixLines = !logPlain;
    opt.prefixThreadId = logThread;

    std::string logErr;
    if (!logTee.start(opt, logErr)) {
      std::cerr << "Warning: failed to start log file tee '" << resolvedLogPath.string() << "': " << logErr << "\n";
    } else {
      std::cout << "Logging to: " << logTee.path().string() << "\n";
    }
  }

  // --- raylib TraceLog forwarding ---
  //
  // raylib can emit important diagnostics during initialization (OpenGL context
  // creation, audio, file access). Forwarding it into stderr ensures it lands
  // in proc_isocity.log for "double-click" builds.
  if (raylibLogEnabled) {
    if (!raylibLogLevelExplicit) {
      const char* envRaylibLog = std::getenv("PROCISOCITY_RAYLIB_LOG");
      if (envRaylibLog && *envRaylibLog) {
        raylibLogLevel = isocity::ParseRaylibLogLevel(envRaylibLog, raylibLogLevel);
      }
    }

    isocity::InstallRaylibLogCallback(raylibLogLevel);
    std::cout << "raylib TraceLog threshold: " << isocity::RaylibLogLevelName(raylibLogLevel) << "\n";
  }

  // --- Crash recovery hints / auto-behaviour ---
  if (prevUncleanShutdown) {
    const isocity::SessionInfo& prev = sessionLock.previousSessionInfo();
    std::cerr << "Detected previous unclean shutdown";
    if (!prev.startedUtc.empty()) {
      std::cerr << " (previous session started: " << prev.startedUtc << ")";
    }
    std::cerr << ".\n";

    if (autoRecoverOnCrash && loadSavePath.empty() && !resumeLatestSave) {
      resumeLatestSave = true;
      autoResumeAfterCrash = true;
      std::cerr << "Auto-recovery enabled: will attempt to resume the most recent save/autosave.\n";
    }

    // If the previous run ended uncleanly and the user didn't explicitly request
    // prefs behaviour, default to safe-mode to maximize the chance of a successful boot.
    if (!safeMode && !ignorePrefs && prefsPathOverride.empty()) {
      safeMode = true;
      ignorePrefs = true;
      std::cerr << "Entering safe-mode after crash detection (ignoring visual prefs).\n";
    }
  }

  // Crash report setup (best-effort). We write crash logs into the *current* working
  // directory at this point, which will be the data dir unless --no-chdir was used.
  {
    std::error_code ec;
    std::filesystem::path reportDir = std::filesystem::current_path(ec);
    if (ec) reportDir.clear();

    std::ostringstream oss;
    oss << "ProcIsoCity crash report\n";
    oss << "version: " << isocity::ProcIsoCityFullVersionString() << "\n";
    oss << "build: " << isocity::ProcIsoCityBuildStamp() << "\n";
    oss << "exe: " << isocity::AppPaths::ExecutablePath().string() << "\n";
    oss << "exe_dir: " << isocity::AppPaths::ExecutableDir().string() << "\n";
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    oss << "cwd: " << (ec ? std::string("(unknown)") : cwd.string()) << "\n";
    oss << "resolved_data_dir: " << resolvedDataDir.string() << "\n";
    if (logTee.active()) {
      oss << "log_file: " << logTee.path().string() << "\n";
    } else {
      oss << "log_file: (disabled)\n";
    }
    if (sessionLock.acquired()) {
      oss << "session_lock: " << sessionLock.lockPath().string() << "\n";
      oss << "session_marker: " << sessionLock.markerPath().string() << "\n";
      oss << "prev_unclean_shutdown: " << (prevUncleanShutdown ? "yes" : "no") << "\n";
    }
    oss << "argv: " << JoinArgs(argc, argv) << "\n";
    if (envDataDir && *envDataDir) {
      oss << "env(PROCISOCITY_DATA_DIR): " << envDataDir << "\n";
    }
    const char* envShader = std::getenv("PROCISOCITY_SHADER_DIR");
    if (envShader && *envShader) {
      oss << "env(PROCISOCITY_SHADER_DIR): " << envShader << "\n";
    }
    const char* envLogFile = std::getenv("PROCISOCITY_LOG_FILE");
    if (envLogFile && *envLogFile) {
      oss << "env(PROCISOCITY_LOG_FILE): " << envLogFile << "\n";
    }

    const char* envRaylibLog = std::getenv("PROCISOCITY_RAYLIB_LOG");
    if (envRaylibLog && *envRaylibLog) {
      oss << "env(PROCISOCITY_RAYLIB_LOG): " << envRaylibLog << "\n";
    }
    oss << "raylib_log_forwarding: " << (raylibLogEnabled ? "yes" : "no") << "\n";
    if (raylibLogEnabled) {
      oss << "raylib_log_level: " << isocity::RaylibLogLevelName(raylibLogLevel) << "\n";
    }

    isocity::CrashHandlerOptions ch;
    ch.reportDir = reportDir;
    ch.preamble = oss.str();
    ch.maxStackFrames = 96;

    // If log tee is active (or a log file path is known), include a log tail in
    // crash reports so players immediately capture pre-crash diagnostics.
    if (logTee.active()) {
      ch.logTailPath = logTee.path();
    } else if (!resolvedLogPath.empty()) {
      ch.logTailPath = resolvedLogPath;
    }
    ch.logTailMaxBytes = 256u * 1024u;
    ch.logTailMaxLines = 400;
    isocity::InstallCrashHandler(ch);
  }

  // --- Diagnostics / support bundle ---
  //
  // These modes allow the executable to be used as a lightweight "launcher tool"
  // without initializing the renderer.
  if (diagnoseOnly || makeSupportBundle || makeSupportBundleZip || healthCheck) {
    std::error_code ec;
    const std::filesystem::path cwd = std::filesystem::current_path(ec);
    const std::filesystem::path diagDataDir = ec ? resolvedDataDir : cwd;

    std::ostringstream d;
    d << "ProcIsoCity diagnostics\n";
    d << "version: " << isocity::ProcIsoCityFullVersionString() << "\n";
    d << "build: " << isocity::ProcIsoCityBuildStamp() << "\n";
    d << "exe: " << isocity::AppPaths::ExecutablePath().string() << "\n";
    d << "exe_dir: " << isocity::AppPaths::ExecutableDir().string() << "\n";
    d << "cwd: " << (ec ? std::string("(unknown)") : cwd.string()) << "\n";
    d << "resolved_data_dir: " << resolvedDataDir.string() << "\n";

#if defined(_WIN32)
    d << "platform: Windows\n";
#elif defined(__APPLE__)
    d << "platform: macOS\n";
#elif defined(__linux__)
    d << "platform: Linux\n";
#else
    d << "platform: (unknown)\n";
#endif
    d << "arch_bits: " << (8 * sizeof(void*)) << "\n";

    if (logTee.active()) {
      d << "log_file: " << logTee.path().string() << "\n";
    } else {
      d << "log_file: (disabled or unavailable)\n";
    }

    // Shader override directory search
    {
      const isocity::ShaderOverrideSearch s = isocity::FindShaderOverrideDir();
      d << "shader_override_dir: " << (s.dir.empty() ? std::string("(not found)") : s.dir.string()) << "\n";
      if (!s.triedPaths.empty()) {
        d << "shader_search_tried:\n";
        for (const auto& p : s.triedPaths) {
          d << "  - " << p.string() << "\n";
        }
      }

      if (!s.dir.empty()) {
        static const char* kRequiredShaders[] = {
          "postfx.vs.glsl", "postfx.fs.glsl",
          "taa.vs.glsl", "taa.fs.glsl",
          "bloom_extract.vs.glsl", "bloom_extract.fs.glsl",
          "bloom_blur.vs.glsl", "bloom_blur.fs.glsl",
          "volcloud.vs.glsl", "volcloud.fs.glsl",
          "weatherfx.vs.glsl", "weatherfx.fs.glsl",
          "materialfx.vs.glsl", "materialfx.fs.glsl",
          "cloudmask.vs.glsl", "cloudmask.fs.glsl",
          "common.glsl",
        };

        int missing = 0;
        for (const char* f : kRequiredShaders) {
          const std::filesystem::path fp = s.dir / f;
          if (!std::filesystem::exists(fp)) {
            if (missing == 0) d << "missing_shader_files:\n";
            d << "  - " << fp.string() << "\n";
            ++missing;
          }
        }
        if (missing == 0) {
          d << "missing_shader_files: (none)\n";
        }
      }
    }

    // Saves
    {
      const auto scan = isocity::ScanKnownSaveFiles(diagDataDir);
      if (!scan.err.empty()) {
        d << "save_scan_errors: " << scan.err << "\n";
      }
      if (scan.found.empty()) {
        d << "saves: (none found)\n";
      } else {
        d << "saves:\n";
        // Sort newest-first for readability.
        auto found = scan.found;
        std::sort(found.begin(), found.end(), [](const isocity::SaveCandidate& a, const isocity::SaveCandidate& b) {
          return a.timestamp > b.timestamp;
        });

        for (const auto& s : found) {
          d << "  - " << isocity::SaveKindToString(s.kind) << " slot=" << s.slot
            << " path=" << s.path.string() << "\n";
        }
      }
    }

    // Visual prefs (and transactional artifacts)
    {
      std::filesystem::path prefsPath = prefsPathOverride.empty() ? std::filesystem::path("isocity_visual.json")
                                                                  : std::filesystem::path(prefsPathOverride);
      const std::filesystem::path prefsTmp = std::filesystem::path(prefsPath.string() + ".tmp");
      const std::filesystem::path prefsBak = std::filesystem::path(prefsPath.string() + ".bak");

      d << "visual_prefs_path: " << prefsPath.string() << "\n";
      d << "visual_prefs_exists: " << (std::filesystem::exists(prefsPath) ? "yes" : "no") << "\n";
      d << "visual_prefs_tmp: " << (std::filesystem::exists(prefsTmp) ? prefsTmp.string() : std::string("(none)"))
        << "\n";
      d << "visual_prefs_bak: " << (std::filesystem::exists(prefsBak) ? prefsBak.string() : std::string("(none)"))
        << "\n";

      d << "visual_prefs_load: " << ((!ignorePrefs) ? "yes" : "no") << "\n";
      d << "safe_mode: " << (safeMode ? "yes" : "no") << "\n";
    }

    // Optional health-check (headless).
    isocity::HealthCheckResult hcRes;
    std::string hcErr;
    // Optional rendered-stage integration check (performed only when explicitly
    // requested). This validates that a save produced headlessly can be loaded
    // and rendered via OpenGL.
    bool hcArtifactsKeptOnDisk = false;
    bool hcCleanupWorkDir = false;
    bool hcRenderRan = false;
    bool hcRenderOk = true;
    isocity::RenderOverviewResult hcRenderRes;
    std::string hcRenderErr;
    if (healthCheck) {
      isocity::HealthCheckOptions hcOpt;
      hcOpt.baseDir = healthCheckDirOverride.empty() ? diagDataDir : std::filesystem::path(healthCheckDirOverride);
      hcOpt.width = std::max(1, healthCheckW);
      hcOpt.height = std::max(1, healthCheckH);
      hcOpt.steps = std::max(0, healthCheckSteps);
      hcOpt.seed = healthCheckSeedExplicit ? healthCheckSeed : ((cfg.seed != 0) ? cfg.seed : isocity::TimeSeed());
      // Keep artifacts whenever we need to post-process them (support bundle or
      // headless->rendered integration check).
      hcOpt.keepArtifacts = healthCheckKeep || makeSupportBundle || makeSupportBundleZip || healthCheckRender;
      hcOpt.verbose = healthCheckVerbose;

      hcArtifactsKeptOnDisk = hcOpt.keepArtifacts;

      (void)isocity::RunHealthCheck(hcOpt, hcRes, hcErr);

      // If we kept artifacts only for this session, clean them up at the end.
      hcCleanupWorkDir = (!healthCheckKeep) && hcArtifactsKeptOnDisk && !hcRes.workDir.empty();

      // Optional: validate that the rendered pipeline can load and draw the
      // health-check save.
      if (healthCheckRender) {
        const std::filesystem::path savePath = hcRes.savePath;
        const bool saveExists = !savePath.empty() && std::filesystem::exists(savePath);

        if (saveExists) {
          hcRenderRan = true;

          isocity::RenderOverviewOptions ro;
          ro.savePath = savePath;
          ro.outImagePath = hcRes.workDir.empty() ? (diagDataDir / "healthcheck_overview.png")
                                                 : (hcRes.workDir / "healthcheck_overview.png");
          ro.maxSize = healthCheckRenderMaxSize;
          ro.timeSec = healthCheckRenderTimeSec;
          ro.includeScreenFx = healthCheckRenderScreenFx;
          // Health-checks should be deterministic and not depend on user prefs.
          ro.useVisualPrefs = false;
          ro.safeMode = false;
          ro.tileWidth = cfg.tileWidth;
          ro.tileHeight = cfg.tileHeight;
          ro.elevationScale = cfg.elevationScale;
          ro.elevationSteps = cfg.elevationSteps;
          // Tiny hidden window is sufficient to create a GL context.
          ro.windowWidth = std::max(64, std::min(640, cfg.windowWidth));
          ro.windowHeight = std::max(64, std::min(640, cfg.windowHeight));
          ro.hiddenWindow = true;

          hcRenderOk = isocity::RenderWorldOverviewFromSave(ro, hcRenderRes, hcRenderErr);
          if (hcRenderOk && !hcRenderRes.outImagePath.empty()) {
            hcRes.artifacts.push_back(hcRenderRes.outImagePath);
          }
        } else {
          // Requested but no save produced (likely earlier health check failure).
          hcRenderRan = false;
          hcRenderOk = false;
          hcRenderErr = "health_check_render requested but no save file exists";
        }
      }

      d << "health_check: " << (hcRes.ok ? "PASS" : "FAIL") << "\n";
      if (!hcRes.workDir.empty()) {
        d << "health_check_work_dir: " << hcRes.workDir.string() << "\n";
      }
      if (!hcErr.empty() && !hcRes.ok) {
        d << "health_check_error: " << hcErr << "\n";
      }

      if (healthCheckRender) {
        const char* status = hcRenderOk ? "PASS" : (hcRenderRan ? "FAIL" : "SKIP");
        d << "health_check_render: " << status << "\n";
        if (!hcRenderRes.outImagePath.empty()) {
          d << "health_check_render_image: " << hcRenderRes.outImagePath.string() << "\n";
        }
        if (!hcRenderErr.empty() && !hcRenderOk) {
          d << "health_check_render_error: " << hcRenderErr << "\n";
        }
        if (!hcRenderRes.report.empty()) {
          d << "health_check_render_report:\n";
          std::istringstream iss(hcRenderRes.report);
          std::string line;
          while (std::getline(iss, line)) {
            d << "  " << line << "\n";
          }
        }
      }

      if (!hcRes.report.empty()) {
        d << "health_check_report:\n";
        std::istringstream iss(hcRes.report);
        std::string line;
        while (std::getline(iss, line)) {
          d << "  " << line << "\n";
        }
      }
    }

    d << "argv: " << JoinArgs(argc, argv) << "\n";

    const std::string diagText = d.str();

    // Health check-only mode: print the report (and optional render stage) and exit.
    if (healthCheck && !makeSupportBundle && !makeSupportBundleZip && !diagnoseOnly) {
      if (!hcRes.report.empty()) {
        std::cout << hcRes.report;
      } else {
        std::cout << "ProcIsoCity health check\n";
        if (!hcErr.empty()) std::cout << "error: " << hcErr << "\n";
      }

      if (healthCheckRender) {
        std::cout << "\n";
        if (!hcRenderRes.report.empty()) {
          std::cout << hcRenderRes.report;
        } else {
          std::cout << "ProcIsoCity health check render overview\n";
          if (!hcRenderErr.empty()) std::cout << "error: " << hcRenderErr << "\n";
        }
      }

      // Cleanup if we only kept artifacts temporarily.
      if (hcCleanupWorkDir) {
        std::error_code ecCleanup;
        std::filesystem::remove_all(hcRes.workDir, ecCleanup);
      }

      const bool overallOk = hcRes.ok && (!healthCheckRender || hcRenderOk);
      return overallOk ? 0 : 5;
    }

    if (diagnoseOnly) {
      std::cout << diagText;
      if (hcCleanupWorkDir) {
        std::error_code ecCleanup;
        std::filesystem::remove_all(hcRes.workDir, ecCleanup);
      }
      return 0;
    }

    // Support bundle: stop log tee so the file can be copied/read on all platforms.
    if (logTee.active()) {
      logTee.stop();
    }

    isocity::SupportBundleOptions sb;
    sb.baseDir = supportBundleDirOverride.empty() ? std::filesystem::path() : std::filesystem::path(supportBundleDirOverride);
    sb.dataDir = diagDataDir;
    sb.diagnosticsText = diagText;
    sb.logPath = resolvedLogPath.empty() ? std::filesystem::path("proc_isocity.log") : resolvedLogPath;
    sb.logKeepFiles = logKeepFiles;

    const std::filesystem::path prefsPath = prefsPathOverride.empty() ? std::filesystem::path("isocity_visual.json")
                                                                      : std::filesystem::path(prefsPathOverride);
    sb.visualPrefsPath = prefsPath;

    // If health-check artifacts were kept on disk (either explicitly or
    // temporarily for a support bundle / render integration stage), include them.
    if (healthCheck && hcArtifactsKeptOnDisk) {
      for (const auto& p : hcRes.artifacts) {
        sb.extraFiles.push_back(p);
      }
    }

    int exitCode = 0;

    if (makeSupportBundle) {
      isocity::SupportBundleResult sbRes;
      std::string sbErr;
      if (!isocity::CreateSupportBundle(sb, sbRes, sbErr)) {
        std::cerr << "Failed to create support bundle: " << sbErr << "\n";
        return 4;
      }

      std::cout << "Support bundle created: " << sbRes.bundleDir.string() << "\n";
      if (!sbRes.warnings.empty()) {
        std::cout << "Warnings:\n";
        for (const auto& w : sbRes.warnings) {
          std::cout << "  - " << w << "\n";
        }
      }
    }

    if (makeSupportBundleZip) {
      isocity::SupportBundleArchiveResult zbRes;
      std::string zbErr;
      if (!isocity::CreateSupportBundleZip(sb, zbRes, zbErr)) {
        std::cerr << "Failed to create support bundle zip: " << zbErr << "\n";
        return 4;
      }

      std::cout << "Support bundle zip created: " << zbRes.archivePath.string() << "\n";
      if (!zbRes.warnings.empty()) {
        std::cout << "Warnings:\n";
        for (const auto& w : zbRes.warnings) {
          std::cout << "  - " << w << "\n";
        }
      }
    }

    // Cleanup temporary health-check artifacts after the bundle is created.
    if (hcCleanupWorkDir) {
      std::error_code ecCleanup;
      std::filesystem::remove_all(hcRes.workDir, ecCleanup);
    }

    return exitCode;
  }

  // Non-interactive render tool: load a save and export a GPU overview image.
  // This is intentionally separate from --health-check (which stays renderer-free
  // unless --health-check-render is requested).

  if (renderOverviewTool) {
    // Resolve inputs. Tool modes should interpret relative paths against the invocation
    // working directory (not the per-user data dir we may have chdir'd into).
    std::filesystem::path savePath = std::filesystem::path(renderOverviewSavePath);
    std::filesystem::path outPath = std::filesystem::path(renderOverviewOutPath);

    // Resolve prefs path override relative to the invocation CWD (not the data dir).
    std::filesystem::path prefsPath = prefsPathOverride.empty() ? std::filesystem::path{} : std::filesystem::path(prefsPathOverride);
    if (!prefsPath.empty() && prefsPath.is_relative() && !invocationCwd.empty()) {
      prefsPath = invocationCwd / prefsPath;
    }

    if (!renderOverviewFromManifest && !invocationCwd.empty()) {
      if (savePath.is_relative()) savePath = invocationCwd / savePath;
      if (outPath.is_relative()) outPath = invocationCwd / outPath;
    }

    // Batch bridge: render overviews for all proc_isocity_cli manifests in a directory (useful for --batch runs).
    if (renderOverviewFromManifestDir) {
      std::filesystem::path dirPath = std::filesystem::path(renderOverviewManifestPath);
      if (dirPath.is_relative() && !invocationCwd.empty()) {
        dirPath = invocationCwd / dirPath;
      }

      std::vector<std::filesystem::path> manifests;
      std::string findErr;
      if (!isocity::FindCliRunManifestsInDir(dirPath, manifests, findErr, true)) {
        std::cerr << "Failed to locate proc_isocity_cli manifests in dir: " << dirPath.string() << "\n";
        if (!findErr.empty()) std::cerr << findErr << "\n";
        return 6;
      }

      // Avoid accidental clobbering when rendering multiple manifests.
      {
        const bool hasSeed = (renderOverviewOutPath.find("{seed}") != std::string::npos);
        const bool hasRun  = (renderOverviewOutPath.find("{run}")  != std::string::npos);
        if (!hasSeed && !hasRun && manifests.size() > 1) {
          std::cerr << "Warning: output pattern does not include {seed} or {run}; renders may overwrite: "
                    << renderOverviewOutPath << "\n";
        }
      }

      int okCount = 0;
      int skipCount = 0;
      int failCount = 0;

      for (const auto& manifestPathUsed : manifests) {
        isocity::CliRunManifest manifest;
        std::string mErr;
        if (!isocity::LoadCliRunManifest(manifestPathUsed, manifest, mErr)) {
          std::cerr << "Skipping unreadable manifest: " << manifestPathUsed.string() << "\n";
          if (!mErr.empty()) std::cerr << mErr << "\n";
          ++skipCount;
          continue;
        }

        const isocity::CliManifestArtifact* saveArt = isocity::FindFirstArtifactByKind(manifest, "save");
        if (!saveArt) {
          std::cerr << "Skipping manifest without a 'save' artifact: " << manifestPathUsed.string() << "\n";
          ++skipCount;
          continue;
        }

        std::string dbg;
        const std::filesystem::path saveDiskPath =
            isocity::ResolveManifestArtifactPathSmart(manifestPathUsed, manifest, saveArt->path, invocationCwd, &dbg);

        std::error_code ec2;
        const bool exists = std::filesystem::exists(saveDiskPath, ec2) && !ec2;
        if (!exists) {
          std::cerr << "Skipping manifest with missing save artifact: " << manifestPathUsed.string() << "\n";
          if (!dbg.empty()) std::cerr << dbg;
          ++skipCount;
          continue;
        }

        const std::string outExpanded = isocity::ExpandCliManifestTemplate(renderOverviewOutPath, manifest);
        std::filesystem::path outDiskPath = std::filesystem::path(outExpanded);

        if (outDiskPath.is_relative()) {
          const std::filesystem::path baseDir = manifestPathUsed.parent_path();
          if (!baseDir.empty()) outDiskPath = baseDir / outDiskPath;
        }

        std::cout << "render-overview(manifest): " << manifestPathUsed.string() << "\n";
        std::cout << "  save: " << saveDiskPath.string() << "\n";
        std::cout << "  out : " << outDiskPath.string() << "\n";

        isocity::RenderOverviewOptions ro;
        ro.savePath = saveDiskPath;
        ro.outImagePath = outDiskPath;
        ro.maxSize = renderOverviewMaxSize;
        ro.timeSec = renderOverviewTimeSec;
        ro.includeScreenFx = renderOverviewScreenFx;
        ro.useVisualPrefs = !ignorePrefs;
        ro.visualPrefsPath = prefsPath;
        ro.safeMode = safeMode;
        ro.tileWidth = cfg.tileWidth;
        ro.tileHeight = cfg.tileHeight;
        ro.elevationScale = cfg.elevationScale;
        ro.elevationSteps = cfg.elevationSteps;
        ro.windowWidth = std::max(64, std::min(640, cfg.windowWidth));
        ro.windowHeight = std::max(64, std::min(640, cfg.windowHeight));
        ro.hiddenWindow = true;

        isocity::RenderOverviewResult roRes;
        std::string roErr;
        const bool ok = isocity::RenderWorldOverviewFromSave(ro, roRes, roErr);
        if (!roRes.report.empty()) {
          std::cout << roRes.report;
        }
        if (!ok) {
          std::cerr << "Failed to render overview for manifest: " << manifestPathUsed.string() << "\n";
          if (!roErr.empty()) std::cerr << roErr << "\n";
          ++failCount;
          continue;
        }

        ++okCount;

        // Optional: record the generated rendered output back into the CLI manifest so downstream tooling can discover it.
        if (renderOverviewUpdateManifest) {
          isocity::CliManifestArtifact a;
          a.kind = "render_overview";
          a.layer = renderOverviewScreenFx ? "gpu_fx" : "gpu";

          // Prefer the expanded template string (often relative to the manifest dir) so the manifest stays relocatable.
          std::string recPath = outExpanded.empty() ? roRes.outImagePath.string() : outExpanded;
          if (!recPath.empty()) {
            const std::filesystem::path rp = std::filesystem::path(recPath);
            if (rp.extension().empty() && !roRes.outImagePath.extension().empty()) {
              recPath += roRes.outImagePath.extension().string();
            }
          }
          a.path = recPath;

          std::string uErr;
          if (!isocity::UpsertCliRunManifestArtifact(manifestPathUsed, a, roRes.outImagePath, uErr, true)) {
            std::cerr << "Warning: failed to update manifest with rendered overview: " << manifestPathUsed.string() << "\n";
            if (!uErr.empty()) std::cerr << uErr << "\n";
          }
        }
      }

      std::cout << "render-overview-manifest-dir summary: ok=" << okCount
                << " skipped=" << skipCount
                << " failed=" << failCount << "\n";

      if (okCount > 0 && failCount == 0) return 0;
      return 6;
    }

    isocity::CliRunManifest manifest;
    std::filesystem::path manifestPathUsed;
    std::string outExpandedForManifest;
    if (renderOverviewFromManifest) {
      manifestPathUsed = std::filesystem::path(renderOverviewManifestPath);
      if (manifestPathUsed.is_relative() && !invocationCwd.empty()) {
        manifestPathUsed = invocationCwd / manifestPathUsed;
      }

      // Convenience: allow passing a directory. We'll pick the newest proc_isocity_cli manifest in it.
      std::error_code ec;
      if (std::filesystem::is_directory(manifestPathUsed, ec) && !ec) {
        std::filesystem::path latest;
        std::string findErr;
        if (!isocity::FindLatestCliRunManifestInDir(manifestPathUsed, latest, findErr)) {
          std::cerr << "Failed to locate a proc_isocity_cli manifest in dir: " << manifestPathUsed.string() << "\n";
          if (!findErr.empty()) std::cerr << findErr << "\n";
          return 6;
        }
        manifestPathUsed = latest;
      }

      std::string mErr;
      if (!isocity::LoadCliRunManifest(manifestPathUsed, manifest, mErr)) {
        std::cerr << "Failed to load manifest: " << manifestPathUsed.string() << "\n";
        if (!mErr.empty()) std::cerr << mErr << "\n";
        return 6;
      }

      const isocity::CliManifestArtifact* saveArt = isocity::FindFirstArtifactByKind(manifest, "save");
      if (!saveArt) {
        std::cerr << "Manifest does not contain a 'save' artifact: " << manifestPathUsed.string() << "\n";
        return 6;
      }
      {
        std::string dbg;
        savePath = isocity::ResolveManifestArtifactPathSmart(manifestPathUsed, manifest, saveArt->path,
                                                            invocationCwd, &dbg);
        std::error_code ec2;
        const bool exists = std::filesystem::exists(savePath, ec2) && !ec2;
        if (!exists) {
          std::cerr << "Manifest save artifact could not be resolved to an existing file.\n";
          if (!dbg.empty()) std::cerr << dbg;
          return 6;
        }
      }

      // Expand output templates from the manifest (supports {seed},{run},{w},{h},{days},{hash}).
      outExpandedForManifest = isocity::ExpandCliManifestTemplate(renderOverviewOutPath, manifest);
      outPath = std::filesystem::path(outExpandedForManifest);

      // Convenience: if the output path is relative, write next to the manifest.
      if (outPath.is_relative()) {
        const std::filesystem::path baseDir = manifestPathUsed.parent_path();
        if (!baseDir.empty()) outPath = baseDir / outPath;
      }
    }

    isocity::RenderOverviewOptions ro;
    ro.savePath = savePath;
    ro.outImagePath = outPath;
    ro.maxSize = renderOverviewMaxSize;
    ro.timeSec = renderOverviewTimeSec;
    ro.includeScreenFx = renderOverviewScreenFx;
    ro.useVisualPrefs = !ignorePrefs;
    ro.visualPrefsPath = prefsPath;
    ro.safeMode = safeMode;
    ro.tileWidth = cfg.tileWidth;
    ro.tileHeight = cfg.tileHeight;
    ro.elevationScale = cfg.elevationScale;
    ro.elevationSteps = cfg.elevationSteps;
    ro.windowWidth = std::max(64, std::min(640, cfg.windowWidth));
    ro.windowHeight = std::max(64, std::min(640, cfg.windowHeight));
    ro.hiddenWindow = true;

    isocity::RenderOverviewResult roRes;
    std::string roErr;
    const bool ok = isocity::RenderWorldOverviewFromSave(ro, roRes, roErr);
    if (!roRes.report.empty()) {
      std::cout << roRes.report;
    }
    if (!ok) {
      if (!roErr.empty()) {
        std::cerr << roErr << "\n";
      }
      return 6;
    }

    const std::filesystem::path finalOutPath = roRes.outImagePath.empty() ? outPath : roRes.outImagePath;

    // Optional: record the generated rendered output back into the CLI manifest so downstream tooling can discover it.
    if (renderOverviewFromManifest && renderOverviewUpdateManifest && !manifestPathUsed.empty()) {
      isocity::CliManifestArtifact a;
      a.kind = "render_overview";
      a.layer = renderOverviewScreenFx ? "gpu_fx" : "gpu";

      // Prefer the expanded template string (often relative to the manifest dir) so the manifest stays relocatable.
      std::string recPath = outExpandedForManifest.empty() ? finalOutPath.string() : outExpandedForManifest;
      if (!recPath.empty()) {
        const std::filesystem::path rp = std::filesystem::path(recPath);
        if (rp.extension().empty() && !finalOutPath.extension().empty()) {
          recPath += finalOutPath.extension().string();
        }
      }
      a.path = recPath;

      std::string uErr;
      if (!isocity::UpsertCliRunManifestArtifact(manifestPathUsed, a, finalOutPath, uErr, true)) {
        std::cerr << "Warning: failed to update manifest with rendered overview: " << manifestPathUsed.string() << "\n";
        if (!uErr.empty()) std::cerr << uErr << "\n";
      }
    }

    return 0;
  }
  if (safeMode) {
    // Safe-mode is intended to get the app running even if a user has
    // persisted aggressive visual settings (post FX, heavy atmosphere, etc.).
    // We keep the overrides conservative and fully local (no file writes).
    cfg.vsync = true;
    cfg.windowHighDPI = false;
    cfg.worldRenderScaleAuto = false;
    cfg.worldRenderScale = 1.0f;
    cfg.worldRenderScaleMin = 0.70f;
    cfg.worldRenderScaleMax = 1.00f;
    cfg.worldRenderTargetFps = 60;
    cfg.worldRenderFilterPoint = false;
  }

  if (cfg.seed == 0) cfg.seed = isocity::TimeSeed();

  // Resolve the initial save to load (if any).
  std::string startupLoadPath;
  std::string startupLoadLabel;
  if (!loadSavePath.empty()) {
    startupLoadPath = loadSavePath;
  } else if (!loadManifestResolvedPath.empty()) {
    std::filesystem::path manifestPath = loadManifestResolvedPath;

    // If the user accidentally passed a directory (or the earlier resolution could not pick a file),
    // try to locate the newest CLI manifest inside it.
    std::error_code ec;
    if (std::filesystem::is_directory(manifestPath, ec) && !ec) {
      std::filesystem::path latest;
      std::string findErr;
      if (isocity::FindLatestCliRunManifestInDir(manifestPath, latest, findErr)) {
        manifestPath = latest;
      } else {
        std::cerr << "Warning: --load-manifest did not find any CLI manifest in dir: " << manifestPath.string() << "\n";
        if (!findErr.empty()) std::cerr << findErr << "\n";
      }
    }

    isocity::CliRunManifest m;
    std::string mErr;
    if (!isocity::LoadCliRunManifest(manifestPath, m, mErr)) {
      std::cerr << "Warning: failed to load manifest for --load-manifest: " << manifestPath.string() << "\n";
      if (!mErr.empty()) std::cerr << mErr << "\n";
    } else {
      const isocity::CliManifestArtifact* saveArt = isocity::FindFirstArtifactByKind(m, "save");
      if (!saveArt) {
        std::cerr << "Warning: manifest does not contain a 'save' artifact: " << manifestPath.string() << "\n";
      } else {
        {
          std::string dbg;
          const std::filesystem::path resolved = isocity::ResolveManifestArtifactPathSmart(
              manifestPath, m, saveArt->path, invocationCwd, &dbg);
          startupLoadPath = resolved.string();

          std::error_code ec2;
          const bool exists = std::filesystem::exists(resolved, ec2) && !ec2;
          if (!exists) {
            std::cerr << "Warning: --load-manifest resolved to a missing save path: " << startupLoadPath << "\n";
            if (!dbg.empty()) std::cerr << dbg;
          }
        }

        // Give the loaded save a useful label in the UI.
        startupLoadLabel = "CLI seed " + std::to_string(m.actualSeed) + " (run " + std::to_string(m.runIndex) + ")";
      }
    }
  } else if (resumeLatestSave) {
    std::error_code ec;
    const std::filesystem::path dir = std::filesystem::current_path(ec);
    if (ec) {
      std::cerr << "Warning: cannot determine current directory for --resume: " << ec.message() << "\n";
    } else {
      const auto candOpt = isocity::FindMostRecentSave(dir);
      if (candOpt) {
        startupLoadPath = candOpt->path.string();
        if (candOpt->kind == isocity::SaveKind::Autosave) {
          startupLoadLabel = "Autosave " + std::to_string(candOpt->slot);
        } else {
          startupLoadLabel = "Save slot " + std::to_string(candOpt->slot);
        }

        if (autoResumeAfterCrash && !startupLoadLabel.empty()) {
          startupLoadLabel = "Recovered: " + startupLoadLabel;
        }
      } else {
        std::cout << "No known saves found to resume in: " << dir.string() << "\n";
      }
    }
  }
  isocity::GameStartupOptions startup{};
  if (!prefsPathOverride.empty()) {
    startup.visualPrefsPath = prefsPathOverride;
  }
  startup.loadVisualPrefs = !ignorePrefs;

  try {
    isocity::Game game(cfg, startup);
    if (!startupLoadPath.empty()) {
      const char* label = startupLoadLabel.empty() ? nullptr : startupLoadLabel.c_str();
      if (!game.loadFromFile(startupLoadPath, label)) {
        std::cerr << "Warning: failed to load save at startup: " << startupLoadPath << "\n";
      }
    }
    game.run();
  } catch (const std::exception& e) {
    isocity::WriteCrashReport("fatal exception", e.what());
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    isocity::WriteCrashReport("fatal exception", "unknown exception");
    std::cerr << "Fatal error: unknown exception\n";
    return 1;
  }

  return 0;
}
