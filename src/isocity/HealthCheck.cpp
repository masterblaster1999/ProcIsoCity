#include "isocity/HealthCheck.hpp"

#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Version.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace isocity {

namespace {

static std::string UtcTimestampForFolder()
{
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const std::time_t tt = clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &tt);
#else
  gmtime_r(&tt, &tm);
#endif

  char buf[32] = {};
  std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02dZ",
                tm.tm_year + 1900,
                tm.tm_mon + 1,
                tm.tm_mday,
                tm.tm_hour,
                tm.tm_min,
                tm.tm_sec);
  return std::string(buf);
}

static std::string FormatMs(double ms)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << ms << "ms";
  return oss.str();
}

static std::uintmax_t FileSizeOrZero(const std::filesystem::path& p)
{
  std::error_code ec;
  const auto s = std::filesystem::file_size(p, ec);
  return ec ? 0u : static_cast<std::uintmax_t>(s);
}

static void BestEffortRemoveAll(const std::filesystem::path& p)
{
  if (p.empty()) return;
  std::error_code ec;
  std::filesystem::remove_all(p, ec);
}

} // namespace

bool RunHealthCheck(const HealthCheckOptions& opt, HealthCheckResult& out, std::string& outError)
{
  outError.clear();
  out = HealthCheckResult{};

  if (opt.width <= 0 || opt.height <= 0) {
    outError = "healthcheck: invalid size";
    return false;
  }
  if (opt.steps < 0) {
    outError = "healthcheck: steps must be >= 0";
    return false;
  }

  // Resolve base directory.
  std::filesystem::path base = opt.baseDir;
  if (base.empty()) {
    std::error_code ec;
    base = std::filesystem::current_path(ec);
    if (ec) base = std::filesystem::path(".");
  }

  // Create a unique work directory.
  const std::string stamp = UtcTimestampForFolder();
  const std::filesystem::path workDir = base / (opt.dirPrefix + "_" + stamp);

  {
    std::error_code ec;
    std::filesystem::create_directories(workDir, ec);
    if (ec) {
      outError = "healthcheck: unable to create work dir '" + workDir.string() + "': " + ec.message();
      return false;
    }
  }

  out.workDir = workDir;

  // Always collect artifacts so callers can include them in support bundles if desired.
  const std::filesystem::path writeTestPath = workDir / "write_test.txt";
  const std::filesystem::path savePath = workDir / "healthcheck_save.bin";
  out.savePath = savePath;
  out.artifacts.push_back(writeTestPath);
  out.artifacts.push_back(savePath);

  std::ostringstream rep;
  rep << "ProcIsoCity health check\n";
  rep << "version: " << ProcIsoCityFullVersionString() << "\n";
  rep << "build: " << ProcIsoCityBuildStamp() << "\n";
  rep << "work_dir: " << workDir.string() << "\n";
  rep << "size: " << opt.width << "x" << opt.height << "\n";
  rep << "seed: " << opt.seed << "\n";
  rep << "steps: " << opt.steps << "\n";

  // 1) Writable directory sanity.
  {
    std::ofstream ofs(writeTestPath, std::ios::out | std::ios::binary);
    if (!ofs) {
      outError = "healthcheck: cannot write to '" + writeTestPath.string() + "'";
      if (!opt.keepArtifacts) BestEffortRemoveAll(workDir);
      return false;
    }
    ofs << "ok\n";
    ofs.flush();
    if (!ofs.good()) {
      outError = "healthcheck: write failed for '" + writeTestPath.string() + "'";
      if (!opt.keepArtifacts) BestEffortRemoveAll(workDir);
      return false;
    }
  }

  // 2) Proc-gen + sim.
  ProcGenConfig procCfg{};
  World world;
  {
    const auto t0 = std::chrono::steady_clock::now();
    world = GenerateWorld(opt.width, opt.height, opt.seed, procCfg);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rep << "procgen_ms: " << FormatMs(ms) << "\n";
  }

  Simulator sim(SimConfig{});
  {
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < opt.steps; ++i) {
      sim.stepOnce(world);
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rep << "sim_ms: " << FormatMs(ms) << "\n";
  }

  // 3) Save -> load roundtrip (includes CRC for v3+ saves).
  {
    const auto t0 = std::chrono::steady_clock::now();
    std::string err;
    if (!SaveWorldBinary(world, procCfg, sim.config(), savePath.string(), err)) {
      outError = "healthcheck: save failed: " + err;
      if (!opt.keepArtifacts) BestEffortRemoveAll(workDir);
      return false;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rep << "save_ms: " << FormatMs(ms) << "\n";
  }

  rep << "save_bytes: " << FileSizeOrZero(savePath) << "\n";

  // Verify summary CRC (fast path).
  {
    SaveSummary sum;
    std::string err;
    if (ReadSaveSummary(savePath.string(), sum, err, /*verifyCrc=*/true)) {
      rep << "save_version: v" << sum.version << "\n";
      if (sum.crcChecked) {
        rep << "save_crc_ok: " << (sum.crcOk ? "yes" : "no") << "\n";
      }
    } else {
      rep << "save_summary_error: " << err << "\n";
    }
  }

  World loaded;
  ProcGenConfig loadedProc;
  SimConfig loadedSim;
  {
    const auto t0 = std::chrono::steady_clock::now();
    std::string err;
    if (!LoadWorldBinary(loaded, loadedProc, loadedSim, savePath.string(), err)) {
      outError = "healthcheck: load failed: " + err;
      if (!opt.keepArtifacts) BestEffortRemoveAll(workDir);
      return false;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rep << "load_ms: " << FormatMs(ms) << "\n";
  }

  // Minimal consistency checks.
  bool ok = true;
  if (loaded.width() != world.width() || loaded.height() != world.height()) {
    rep << "check_world_size: FAIL (loaded " << loaded.width() << "x" << loaded.height()
        << ", expected " << world.width() << "x" << world.height() << ")\n";
    ok = false;
  } else {
    rep << "check_world_size: ok\n";
  }

  // Optional deeper info.
  if (opt.verbose) {
    rep << "loaded_seed: " << loaded.seed() << "\n";
    rep << "loaded_tiles: " << (static_cast<std::uint64_t>(loaded.width()) * static_cast<std::uint64_t>(loaded.height()))
        << "\n";
  }

  out.ok = ok;
  out.report = rep.str();

  if (!ok) {
    outError = "healthcheck: consistency check failed";
  }

  if (!opt.keepArtifacts) {
    BestEffortRemoveAll(workDir);
  }

  return out.ok;
}

} // namespace isocity
