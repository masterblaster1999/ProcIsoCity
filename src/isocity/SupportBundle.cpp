#include "isocity/SupportBundle.hpp"

#include "isocity/ZipWriter.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace isocity {

namespace {

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

static bool IsCrashReportName(const std::filesystem::path& p)
{
  const std::string name = p.filename().string();
  if (name.size() < 10) return false;
  if (name.rfind("crash_", 0) != 0) return false;
  return p.extension() == ".txt";
}

static void AddWarning(SupportBundleResult& out, const std::string& w)
{
  out.warnings.push_back(w);
}

static bool CopyOneFile(const std::filesystem::path& src,
                        const std::filesystem::path& dst,
                        std::string& outError)
{
  outError.clear();
  if (src.empty() || dst.empty()) return true;

  std::error_code ec;
  if (!std::filesystem::exists(src, ec) || ec) return true;

  std::filesystem::create_directories(dst.parent_path(), ec);
  if (ec) {
    outError = "Failed to create directory '" + dst.parent_path().string() + "': " + ec.message();
    return false;
  }

  std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    outError = "Failed to copy '" + src.string() + "' -> '" + dst.string() + "': " + ec.message();
    return false;
  }

  return true;
}

static std::vector<std::filesystem::path> FindRecentCrashReports(const std::filesystem::path& dir, int maxFiles)
{
  std::vector<std::filesystem::path> found;
  if (dir.empty() || maxFiles <= 0) return found;

  std::error_code ec;
  if (!std::filesystem::exists(dir, ec) || ec) return found;

  for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
    if (ec) break;
    if (!entry.is_regular_file(ec) || ec) continue;
    const std::filesystem::path p = entry.path();
    if (!IsCrashReportName(p)) continue;
    found.push_back(p);
  }

  // Sort by modification time (descending).
  std::sort(found.begin(), found.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
    std::error_code eca, ecb;
    const auto ta = std::filesystem::last_write_time(a, eca);
    const auto tb = std::filesystem::last_write_time(b, ecb);
    if (eca || ecb) return a.string() < b.string();
    return ta > tb;
  });

  if (static_cast<int>(found.size()) > maxFiles) {
    found.resize(static_cast<std::size_t>(maxFiles));
  }
  return found;
}

static std::filesystem::path WithRotationSuffix(const std::filesystem::path& base, int idx)
{
  if (idx <= 0) return base;
  std::filesystem::path p = base;
  p += ".";
  p += std::to_string(idx);
  return p;
}

} // namespace

bool CreateSupportBundle(const SupportBundleOptions& opt, SupportBundleResult& out, std::string& outError)
{
  outError.clear();
  out = SupportBundleResult{};

  const std::filesystem::path base = opt.baseDir.empty() ? opt.dataDir : opt.baseDir;
  if (base.empty()) {
    outError = "SupportBundle base directory is empty";
    return false;
  }

  std::error_code ec;
  std::filesystem::create_directories(base, ec);
  if (ec) {
    outError = "Failed to create base directory '" + base.string() + "': " + ec.message();
    return false;
  }

  const std::filesystem::path bundleDir = base / (opt.namePrefix + "_" + TimestampUtcForFilename());
  const std::filesystem::path filesDir = bundleDir / "files";

  std::filesystem::create_directories(filesDir, ec);
  if (ec) {
    outError = "Failed to create bundle directory '" + filesDir.string() + "': " + ec.message();
    return false;
  }

  out.bundleDir = bundleDir;
  out.filesDir = filesDir;

  // diagnostics.txt
  {
    const std::filesystem::path diagPath = bundleDir / "diagnostics.txt";
    std::ofstream ofs(diagPath, std::ios::out | std::ios::binary);
    if (!ofs) {
      outError = "Failed to write '" + diagPath.string() + "'";
      return false;
    }
    ofs << opt.diagnosticsText;
    if (!opt.diagnosticsText.empty() && opt.diagnosticsText.back() != '\n') ofs << "\n";
    ofs.flush();
  }

  // log file + rotated logs
  if (!opt.logPath.empty()) {
    for (int i = 0; i <= std::max(0, opt.logKeepFiles); ++i) {
      const std::filesystem::path src = WithRotationSuffix(opt.logPath, i);
      const std::filesystem::path dst = filesDir / src.filename();
      std::string err;
      if (!CopyOneFile(src, dst, err)) {
        AddWarning(out, err);
      }
    }
  }

  // visual prefs (including transactional artifacts, if present)
  if (!opt.visualPrefsPath.empty()) {
    auto copyIfExists = [&](const std::filesystem::path& src) {
      if (src.empty()) return;
      std::error_code ec;
      if (!std::filesystem::exists(src, ec) || ec) return;
      const std::filesystem::path dst = filesDir / src.filename();
      std::string err;
      if (!CopyOneFile(src, dst, err)) {
        AddWarning(out, err);
      }
    };

    copyIfExists(opt.visualPrefsPath);
    std::filesystem::path bak = opt.visualPrefsPath;
    bak += ".bak";
    copyIfExists(bak);
    std::filesystem::path tmp = opt.visualPrefsPath;
    tmp += ".tmp";
    copyIfExists(tmp);
  }

  // recent crash reports
  for (const auto& crash : FindRecentCrashReports(opt.dataDir, opt.crashReportsMax)) {
    const std::filesystem::path dst = filesDir / crash.filename();
    std::string err;
    if (!CopyOneFile(crash, dst, err)) {
      AddWarning(out, err);
    }
  }

  // extra files
  for (const auto& extra : opt.extraFiles) {
    if (extra.empty()) continue;
    const std::filesystem::path dst = filesDir / extra.filename();
    std::string err;
    if (!CopyOneFile(extra, dst, err)) {
      AddWarning(out, err);
    }
  }

  // manifest
  if (opt.includeManifest) {
    const std::filesystem::path manPath = bundleDir / "manifest.txt";
    std::ofstream ofs(manPath, std::ios::out | std::ios::binary);
    if (ofs) {
      ofs << "ProcIsoCity support bundle\n";
      ofs << "bundle_dir: " << bundleDir.string() << "\n";
      ofs << "files_dir: " << filesDir.string() << "\n";
      ofs << "\nIncluded:\n";
      ofs << "  diagnostics.txt\n";

      std::error_code iec;
      for (const auto& entry : std::filesystem::directory_iterator(filesDir, iec)) {
        if (iec) break;
        if (!entry.is_regular_file(iec) || iec) continue;
        ofs << "  files/" << entry.path().filename().string() << "\n";
      }

      if (!out.warnings.empty()) {
        ofs << "\nWarnings:\n";
        for (const auto& w : out.warnings) {
          ofs << "  - " << w << "\n";
        }
      }
      ofs.flush();
    }
  }

  return true;
}

bool CreateSupportBundleZip(const SupportBundleOptions& opt, SupportBundleArchiveResult& out, std::string& outError)
{
  outError.clear();
  out = SupportBundleArchiveResult{};

  const std::filesystem::path base = opt.baseDir.empty() ? opt.dataDir : opt.baseDir;
  if (base.empty()) {
    outError = "SupportBundle base directory is empty";
    return false;
  }

  std::error_code ec;
  std::filesystem::create_directories(base, ec);
  if (ec) {
    outError = "Failed to create base directory '" + base.string() + "': " + ec.message();
    return false;
  }

  const std::string rootName = opt.namePrefix + "_" + TimestampUtcForFilename();
  const std::filesystem::path zipPath = base / (rootName + ".zip");

  ZipWriter zw;
  std::string zipErr;
  if (!zw.open(zipPath, zipErr)) {
    outError = zipErr;
    return false;
  }

  std::vector<std::string> included;

  auto addTextFile = [&](const std::string& rel, const std::string& text) -> bool {
    std::string content = text;
    if (!content.empty() && content.back() != '\n') content.push_back('\n');
    std::string err;
    if (!zw.addFileFromString(rootName + "/" + rel, content, err)) {
      outError = err;
      return false;
    }
    included.push_back(rel);
    return true;
  };

  auto tryAddDiskFile = [&](const std::filesystem::path& src, const std::string& rel) {
    if (src.empty()) return;
    std::error_code e2;
    if (!std::filesystem::exists(src, e2) || e2) return;
    std::string err;
    if (!zw.addFileFromPath(rootName + "/" + rel, src, err)) {
      out.warnings.push_back(err);
    } else {
      included.push_back(rel);
    }
  };

  // diagnostics.txt (required)
  if (!addTextFile("diagnostics.txt", opt.diagnosticsText)) {
    return false;
  }

  // log file + rotated logs
  if (!opt.logPath.empty()) {
    for (int i = 0; i <= std::max(0, opt.logKeepFiles); ++i) {
      const std::filesystem::path src = WithRotationSuffix(opt.logPath, i);
      const std::string rel = std::string("files/") + src.filename().string();
      tryAddDiskFile(src, rel);
    }
  }

  // visual prefs (including transactional artifacts, if present)
  if (!opt.visualPrefsPath.empty()) {
    auto addIfExists = [&](const std::filesystem::path& src) {
      if (src.empty()) return;
      std::error_code ec;
      if (!std::filesystem::exists(src, ec) || ec) return;
      const std::string rel = std::string("files/") + src.filename().string();
      tryAddDiskFile(src, rel);
    };

    addIfExists(opt.visualPrefsPath);
    std::filesystem::path bak = opt.visualPrefsPath;
    bak += ".bak";
    addIfExists(bak);
    std::filesystem::path tmp = opt.visualPrefsPath;
    tmp += ".tmp";
    addIfExists(tmp);
  }

  // recent crash reports
  for (const auto& crash : FindRecentCrashReports(opt.dataDir, opt.crashReportsMax)) {
    const std::string rel = std::string("files/") + crash.filename().string();
    tryAddDiskFile(crash, rel);
  }

  // extra files
  for (const auto& extra : opt.extraFiles) {
    if (extra.empty()) continue;
    const std::string rel = std::string("files/") + extra.filename().string();
    tryAddDiskFile(extra, rel);
  }

  // manifest
  if (opt.includeManifest) {
    std::ostringstream man;
    man << "ProcIsoCity support bundle\n";
    man << "archive: " << zipPath.string() << "\n";
    man << "root: " << rootName << "\n";
    man << "\nIncluded:\n";
    for (const auto& p : included) {
      man << "  " << p << "\n";
    }
    if (!out.warnings.empty()) {
      man << "\nWarnings:\n";
      for (const auto& w : out.warnings) {
        man << "  - " << w << "\n";
      }
    }

    if (!addTextFile("manifest.txt", man.str())) {
      return false;
    }
  }

  if (!zw.finalize(zipErr)) {
    outError = zipErr;
    return false;
  }

  out.archivePath = zipPath;
  return true;
}

} // namespace isocity
