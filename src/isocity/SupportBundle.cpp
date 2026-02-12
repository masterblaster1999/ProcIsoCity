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
#include <string_view>
#include <unordered_set>

namespace isocity {

namespace {

static bool IsAsciiAlnum(char c)
{
  const unsigned char u = static_cast<unsigned char>(c);
  if (u >= static_cast<unsigned char>('0') && u <= static_cast<unsigned char>('9')) return true;
  if (u >= static_cast<unsigned char>('A') && u <= static_cast<unsigned char>('Z')) return true;
  if (u >= static_cast<unsigned char>('a') && u <= static_cast<unsigned char>('z')) return true;
  return false;
}

static bool IsWindowsReservedDeviceName(const std::string& baseUpper)
{
  if (baseUpper == "CON" || baseUpper == "PRN" || baseUpper == "AUX" || baseUpper == "NUL") return true;
  if (baseUpper.size() == 4 && baseUpper.rfind("COM", 0) == 0) {
    const char d = baseUpper[3];
    return d >= '1' && d <= '9';
  }
  if (baseUpper.size() == 4 && baseUpper.rfind("LPT", 0) == 0) {
    const char d = baseUpper[3];
    return d >= '1' && d <= '9';
  }
  return false;
}

static std::string SanitizeFilenameComponent(std::string_view in)
{
  // Keep this conservative and cross-platform (Windows + POSIX).
  // Replace any risky characters with '_' and collapse repeats.
  // Also avoid trailing '.' or ' ' which are problematic on Windows.
  std::string out;
  out.reserve(in.size());

  bool lastUnderscore = false;
  for (char c : in) {
    if (IsAsciiAlnum(c)) {
      out.push_back(c);
      lastUnderscore = false;
      continue;
    }

    // Common safe separators.
    if (c == '-' || c == '_') {
      if (!out.empty() && !lastUnderscore) {
        out.push_back(c);
        lastUnderscore = (c == '_');
      }
      continue;
    }

    // Convert whitespace and all other punctuation to underscores.
    if (!out.empty() && !lastUnderscore) {
      out.push_back('_');
      lastUnderscore = true;
    }
  }

  // Strip leading/trailing separators and collapse to a default.
  while (!out.empty() && (out.front() == '_' || out.front() == '-' || out.front() == '.')) out.erase(out.begin());
  while (!out.empty() && (out.back() == '_' || out.back() == '-' || out.back() == '.' || out.back() == ' ')) out.pop_back();

  if (out.empty() || out == "." || out == "..") out = "support";

  // Windows reserved device names (case-insensitive). We only check the base
  // prefix, not any extension.
  std::string up;
  up.reserve(out.size());
  for (char c : out) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (u >= static_cast<unsigned char>('a') && u <= static_cast<unsigned char>('z')) {
      up.push_back(static_cast<char>(u - static_cast<unsigned char>('a') + static_cast<unsigned char>('A')));
    } else {
      up.push_back(c);
    }
  }
  if (IsWindowsReservedDeviceName(up)) out.push_back('_');

  // Keep bundle folder names reasonably short.
  constexpr std::size_t kMax = 64;
  if (out.size() > kMax) out.resize(kMax);

  return out;
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

static std::filesystem::path ChooseUniqueBasename(const std::filesystem::path& filename,
                                                  const std::unordered_set<std::string>& used)
{
  std::string orig = filename.filename().string();
  if (orig.empty()) orig = "file";
  if (used.find(orig) == used.end()) return std::filesystem::path(orig);

  std::string base = filename.stem().string();
  const std::string ext = filename.extension().string();
  if (base.empty()) base = orig;

  for (int i = 2; i < 10000; ++i) {
    std::string cand = base;
    cand += "__";
    cand += std::to_string(i);
    cand += ext;
    if (used.find(cand) == used.end()) return std::filesystem::path(cand);
  }

  // Extremely unlikely fallback.
  {
    std::string cand = base;
    cand += "__";
    cand += TimestampUtcForFilename();
    cand += ext;
    if (used.find(cand) == used.end()) return std::filesystem::path(cand);

    int i = 2;
    while (true) {
      std::string c2 = cand;
      c2 += "__";
      c2 += std::to_string(i++);
      if (used.find(c2) == used.end()) return std::filesystem::path(c2);
    }
  }
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

  std::filesystem::copy_file(src, dst, std::filesystem::copy_options::none, ec);
  if (ec) {
    outError = "Failed to copy '" + src.string() + "' -> '" + dst.string() + "': " + ec.message();
    return false;
  }

  return true;
}

static bool CopyOneFileToDirUniqueBasename(const std::filesystem::path& src,
                                          const std::filesystem::path& dstDir,
                                          std::unordered_set<std::string>& usedBasenames,
                                          std::filesystem::path& outDst,
                                          std::string& outError)
{
  outError.clear();
  outDst.clear();
  if (src.empty() || dstDir.empty()) return true;

  std::error_code ec;
  if (!std::filesystem::exists(src, ec) || ec) return true;

  const std::filesystem::path baseName = ChooseUniqueBasename(src.filename(), usedBasenames);
  const std::filesystem::path dst = dstDir / baseName;

  std::string err;
  if (!CopyOneFile(src, dst, err)) {
    outError = err;
    return false;
  }

  usedBasenames.insert(baseName.string());
  outDst = dst;
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

static void CollectDirFilesDeterministic(const std::filesystem::path& dir,
                                        std::vector<std::filesystem::path>& out,
                                        std::size_t maxFiles,
                                        std::vector<std::string>& outWarnings,
                                        bool& outTruncated)
{
  if (maxFiles > 0 && out.size() >= maxFiles) {
    outTruncated = true;
    return;
  }

  std::error_code ec;
  std::vector<std::filesystem::directory_entry> entries;
  for (std::filesystem::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
    entries.push_back(*it);
  }

  if (ec) {
    outWarnings.push_back("Unable to scan extra directory '" + dir.string() + "': " + ec.message());
    return;
  }

  std::sort(entries.begin(), entries.end(), [](const std::filesystem::directory_entry& a,
                                               const std::filesystem::directory_entry& b) {
    return a.path().filename().generic_string() < b.path().filename().generic_string();
  });

  for (const auto& e : entries) {
    if (maxFiles > 0 && out.size() >= maxFiles) {
      outTruncated = true;
      return;
    }

    std::error_code sec;
    if (e.is_symlink(sec) && !sec) {
      continue;
    }

    if (e.is_regular_file(sec) && !sec) {
      out.push_back(e.path());
      continue;
    }

    if (e.is_directory(sec) && !sec) {
      CollectDirFilesDeterministic(e.path(), out, maxFiles, outWarnings, outTruncated);
      continue;
    }
  }
}

static void ExpandExtraPaths(const SupportBundleOptions& opt,
                            std::vector<std::filesystem::path>& outFiles,
                            std::vector<std::string>& outWarnings)
{
  outFiles.clear();
  outWarnings.clear();

  std::size_t remaining = 0;
  if (opt.extraDirMaxFiles > 0) remaining = static_cast<std::size_t>(opt.extraDirMaxFiles);

  for (const auto& extra : opt.extraFiles) {
    if (extra.empty()) continue;

    std::error_code ec;
    const bool exists = std::filesystem::exists(extra, ec);
    if (ec || !exists) continue;

    if (std::filesystem::is_regular_file(extra, ec) && !ec) {
      outFiles.push_back(extra);
      continue;
    }

    if (std::filesystem::is_directory(extra, ec) && !ec) {
      if (opt.extraDirMaxFiles <= 0) {
        outWarnings.push_back("Extra directory skipped (directory expansion disabled): " + extra.string());
        continue;
      }

      if (remaining == 0) {
        outWarnings.push_back("Extra directory skipped (extraDirMaxFiles cap reached): " + extra.string());
        continue;
      }

      const std::size_t before = outFiles.size();
      bool truncated = false;
      CollectDirFilesDeterministic(extra, outFiles, before + remaining, outWarnings, truncated);
      const std::size_t added = outFiles.size() - before;
      if (added >= remaining) remaining = 0;
      else remaining -= added;

      if (truncated) {
        outWarnings.push_back("Extra directory truncated to " + std::to_string(added) +
                              " file(s): " + extra.string());
      }
      continue;
    }

    // Unsupported file type.
    outWarnings.push_back("Extra path is not a regular file or directory (skipped): " + extra.string());
  }
}

} // namespace

std::string SanitizeSupportBundleNamePrefix(std::string_view prefix)
{
  return SanitizeFilenameComponent(prefix);
}

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

  const std::string prefix = SanitizeSupportBundleNamePrefix(opt.namePrefix);
  const std::filesystem::path bundleDir = base / (prefix + "_" + TimestampUtcForFilename());
  const std::filesystem::path filesDir = bundleDir / "files";

  std::filesystem::create_directories(filesDir, ec);
  if (ec) {
    outError = "Failed to create bundle directory '" + filesDir.string() + "': " + ec.message();
    return false;
  }

  out.bundleDir = bundleDir;
  out.filesDir = filesDir;

  const std::filesystem::path diagPath = bundleDir / "diagnostics.txt";

  // diagnostics.txt
  {
    std::ofstream ofs(diagPath, std::ios::out | std::ios::binary);
    if (!ofs) {
      outError = "Failed to write '" + diagPath.string() + "'";
      return false;
    }
    ofs << opt.diagnosticsText;
    if (!opt.diagnosticsText.empty() && opt.diagnosticsText.back() != '\n') ofs << "\n";
    ofs.flush();
  }

  std::unordered_set<std::string> usedBasenames;

  auto copyToFilesDir = [&](const std::filesystem::path& src) {
    std::filesystem::path dst;
    std::string err;
    if (!CopyOneFileToDirUniqueBasename(src, filesDir, usedBasenames, dst, err)) {
      AddWarning(out, err);
    }
  };

  // log file + rotated logs
  if (!opt.logPath.empty()) {
    for (int i = 0; i <= std::max(0, opt.logKeepFiles); ++i) {
      const std::filesystem::path src = WithRotationSuffix(opt.logPath, i);
      copyToFilesDir(src);
    }
  }

  // visual prefs (including transactional artifacts, if present)
  if (!opt.visualPrefsPath.empty()) {
    copyToFilesDir(opt.visualPrefsPath);
    std::filesystem::path bak = opt.visualPrefsPath;
    bak += ".bak";
    copyToFilesDir(bak);
    std::filesystem::path tmp = opt.visualPrefsPath;
    tmp += ".tmp";
    copyToFilesDir(tmp);
  }

  // recent crash reports
  for (const auto& crash : FindRecentCrashReports(opt.dataDir, opt.crashReportsMax)) {
    copyToFilesDir(crash);
  }

  // extra files
  {
    std::vector<std::filesystem::path> expanded;
    std::vector<std::string> w;
    ExpandExtraPaths(opt, expanded, w);
    for (const auto& ww : w) AddWarning(out, ww);
    for (const auto& extra : expanded) copyToFilesDir(extra);
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

      {
        std::error_code sec;
        const auto diagSize = std::filesystem::file_size(diagPath, sec);
        ofs << "  diagnostics.txt";
        if (!sec) ofs << " (" << diagSize << " bytes)";
        ofs << "\n";
      }

      std::vector<std::filesystem::path> fileEntries;
      {
        std::error_code iec;
        for (const auto& entry : std::filesystem::directory_iterator(filesDir, iec)) {
          if (iec) break;
          if (!entry.is_regular_file(iec) || iec) continue;
          fileEntries.push_back(entry.path());
        }
      }

      std::sort(fileEntries.begin(), fileEntries.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return a.filename().string() < b.filename().string();
      });

      for (const auto& p : fileEntries) {
        std::error_code sec;
        const auto sz = std::filesystem::file_size(p, sec);
        ofs << "  files/" << p.filename().string();
        if (!sec) ofs << " (" << sz << " bytes)";
        ofs << "\n";
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

  const std::string prefix = SanitizeSupportBundleNamePrefix(opt.namePrefix);
  const std::string rootName = prefix + "_" + TimestampUtcForFilename();
  const std::filesystem::path zipPath = base / (rootName + ".zip");

  ZipWriter zw;
  std::string zipErr;
  if (!zw.open(zipPath, zipErr)) {
    outError = zipErr;
    return false;
  }

  struct IncludedEntry {
    std::string rel;
    std::uintmax_t size = 0;
  };

  std::vector<IncludedEntry> included;
  std::unordered_set<std::string> usedBasenames;

  auto addTextFile = [&](const std::string& rel, const std::string& text) -> bool {
    std::string content = text;
    if (!content.empty() && content.back() != '\n') content.push_back('\n');
    std::string err;
    if (!zw.addFileFromString(rootName + "/" + rel, content, err)) {
      outError = err;
      return false;
    }
    IncludedEntry e;
    e.rel = rel;
    e.size = static_cast<std::uintmax_t>(content.size());
    included.push_back(e);
    return true;
  };

  auto tryAddDiskFile = [&](const std::filesystem::path& src) {
    if (src.empty()) return;
    std::error_code e2;
    if (!std::filesystem::exists(src, e2) || e2) return;

    const std::filesystem::path baseName = ChooseUniqueBasename(src.filename(), usedBasenames);
    const std::string rel = std::string("files/") + baseName.string();

    std::string err;
    if (!zw.addFileFromPath(rootName + "/" + rel, src, err)) {
      out.warnings.push_back(err);
      return;
    }

    usedBasenames.insert(baseName.string());

    std::error_code sec;
    const auto sz = std::filesystem::file_size(src, sec);

    IncludedEntry e;
    e.rel = rel;
    e.size = sec ? 0 : sz;
    included.push_back(e);
  };

  // diagnostics.txt (required)
  if (!addTextFile("diagnostics.txt", opt.diagnosticsText)) {
    return false;
  }

  // log file + rotated logs
  if (!opt.logPath.empty()) {
    for (int i = 0; i <= std::max(0, opt.logKeepFiles); ++i) {
      const std::filesystem::path src = WithRotationSuffix(opt.logPath, i);
      tryAddDiskFile(src);
    }
  }

  // visual prefs (including transactional artifacts, if present)
  if (!opt.visualPrefsPath.empty()) {
    tryAddDiskFile(opt.visualPrefsPath);
    std::filesystem::path bak = opt.visualPrefsPath;
    bak += ".bak";
    tryAddDiskFile(bak);
    std::filesystem::path tmp = opt.visualPrefsPath;
    tmp += ".tmp";
    tryAddDiskFile(tmp);
  }

  // recent crash reports
  for (const auto& crash : FindRecentCrashReports(opt.dataDir, opt.crashReportsMax)) {
    tryAddDiskFile(crash);
  }

  // extra files
  {
    std::vector<std::filesystem::path> expanded;
    std::vector<std::string> w;
    ExpandExtraPaths(opt, expanded, w);
    for (const auto& ww : w) out.warnings.push_back(ww);
    for (const auto& extra : expanded) tryAddDiskFile(extra);
  }

  // manifest
  if (opt.includeManifest) {
    std::ostringstream man;
    man << "ProcIsoCity support bundle\n";
    man << "archive: " << zipPath.string() << "\n";
    man << "root: " << rootName << "\n";
    man << "\nIncluded:\n";

    std::vector<IncludedEntry> sorted = included;
    std::sort(sorted.begin(), sorted.end(), [](const IncludedEntry& a, const IncludedEntry& b) {
      return a.rel < b.rel;
    });

    for (const auto& p : sorted) {
      man << "  " << p.rel << " (" << p.size << " bytes)\n";
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
