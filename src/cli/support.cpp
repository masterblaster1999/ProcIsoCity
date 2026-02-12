#include "cli/CliParse.hpp"

#include "isocity/SupportBundle.hpp"
#include "isocity/Version.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_support (headless support bundle generator)\n\n"
      << "Creates a self-contained folder or .zip containing diagnostics + logs + crash reports\n"
      << "to attach to bug reports. This tool does not require raylib.\n\n"
      << "Usage:\n"
      << "  proc_isocity_support [options]\n\n"
      << "Options:\n"
      << "  --out <dir>          Base output directory. Default: captures\n"
      << "  --zip <0|1>          Write a .zip (1) or a folder (0). Default: 1\n"
      << "  --prefix <name>      Bundle name prefix. Default: support\n"
      << "  --data-dir <dir>     Data directory to scan for crash_*.txt. Default: cwd\n"
      << "  --log <path>         Log file to include (rotations optional). Default: proc_isocity.log\n"
      << "  --log-keep <N>       Include rotated logs (.1..N). Default: 3\n"
      << "  --prefs <path>       Visual prefs file to include (optional).\n"
      << "  --crash-max <N>      Max recent crash reports to copy. Default: 5\n"
      << "  --extra <path>       Extra file or directory to include (repeatable).\n"
      << "  --extra-dir-max-files <N>\n"
      << "                      Max files included from --extra directories (combined). Default: 2000.\n"
      << "                      Set to 0 to disable directory expansion.\n"
      << "  --diag <text>        Extra diagnostic text appended to diagnostics.txt.\n"
      << "  --diag-file <path>   Append diagnostic text from a file.\n"
      << "  --version            Print version/build info and exit.\n"
      << "  -h, --help           Show this help.\n";
}

static std::string ReadFileToString(const std::filesystem::path& p)
{
  std::ifstream ifs(p, std::ios::in | std::ios::binary);
  if (!ifs) return std::string{};
  std::ostringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

static const char* GetEnvCStr(const char* name)
{
  if (!name || !*name) return nullptr;
  return std::getenv(name);
}

} // namespace

int main(int argc, char** argv)
{
  namespace fs = std::filesystem;
  using namespace isocity::cli;

  fs::path outDir = fs::path("captures");
  fs::path dataDir;
  std::string prefix = "support";
  fs::path logPath;
  int logKeep = 3;
  fs::path prefsPath;
  int crashMax = 5;
  int extraDirMaxFiles = 2000;
  bool zip = true;
  std::vector<fs::path> extraFiles;
  std::string diagExtra;

  std::vector<std::string> errors;

  auto needValue = [&](int i) -> bool {
    if (i + 1 >= argc) {
      errors.push_back(std::string("Missing value for '") + argv[i] + "'");
      return false;
    }
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      PrintHelp();
      return 0;
    }
    if (a == "--version") {
      std::cout << "version: " << ProcIsoCityFullVersionString() << "\n";
      std::cout << "build: " << ProcIsoCityBuildStamp() << "\n";
      return 0;
    }
    if (a == "--out") {
      if (!needValue(i)) break;
      outDir = fs::path(argv[++i]);
      continue;
    }
    if (a == "--data-dir") {
      if (!needValue(i)) break;
      dataDir = fs::path(argv[++i]);
      continue;
    }
    if (a == "--prefix") {
      if (!needValue(i)) break;
      prefix = argv[++i];
      continue;
    }
    if (a == "--zip") {
      if (!needValue(i)) break;
      bool v = true;
      if (!ParseBool01(argv[++i], &v)) {
        errors.push_back("Invalid --zip (expected 0/1)");
      } else {
        zip = v;
      }
      continue;
    }
    if (a == "--log") {
      if (!needValue(i)) break;
      logPath = fs::path(argv[++i]);
      continue;
    }
    if (a == "--log-keep") {
      if (!needValue(i)) break;
      int v = 0;
      if (!ParseI32(argv[++i], &v) || v < 0) {
        errors.push_back("Invalid --log-keep (expected >= 0)");
      } else {
        logKeep = v;
      }
      continue;
    }
    if (a == "--prefs") {
      if (!needValue(i)) break;
      prefsPath = fs::path(argv[++i]);
      continue;
    }
    if (a == "--crash-max") {
      if (!needValue(i)) break;
      int v = 0;
      if (!ParseI32(argv[++i], &v) || v < 0) {
        errors.push_back("Invalid --crash-max (expected >= 0)");
      } else {
        crashMax = v;
      }
      continue;
    }
    if (a == "--extra-dir-max-files" || a == "--extra-dir-max") {
      if (!needValue(i)) break;
      int v = 0;
      if (!ParseI32(argv[++i], &v) || v < 0) {
        errors.push_back("Invalid --extra-dir-max-files (expected >= 0)");
      } else {
        extraDirMaxFiles = v;
      }
      continue;
    }
    if (a == "--extra") {
      if (!needValue(i)) break;
      extraFiles.push_back(fs::path(argv[++i]));
      continue;
    }
    if (a == "--diag") {
      if (!needValue(i)) break;
      if (!diagExtra.empty() && diagExtra.back() != '\n') diagExtra.push_back('\n');
      diagExtra += argv[++i];
      diagExtra.push_back('\n');
      continue;
    }
    if (a == "--diag-file") {
      if (!needValue(i)) break;
      const fs::path p = fs::path(argv[++i]);
      const std::string text = ReadFileToString(p);
      if (text.empty()) {
        errors.push_back(std::string("Failed to read diag file: ") + p.string());
      } else {
        if (!diagExtra.empty() && diagExtra.back() != '\n') diagExtra.push_back('\n');
        diagExtra += text;
        if (!diagExtra.empty() && diagExtra.back() != '\n') diagExtra.push_back('\n');
      }
      continue;
    }

    errors.push_back(std::string("Unknown arg: ") + a);
  }

  if (!errors.empty()) {
    for (const auto& e : errors) {
      std::cerr << "Error: " << e << "\n";
    }
    std::cerr << "\n";
    PrintHelp();
    return 2;
  }

  std::error_code ec;
  fs::path cwd = fs::current_path(ec);
  if (ec) cwd = fs::path(".");
  if (dataDir.empty()) dataDir = cwd;

  if (!EnsureDir(outDir)) {
    std::cerr << "Error: Unable to create output directory: " << outDir.string() << "\n";
    return 1;
  }

  SupportBundleOptions opt;
  opt.baseDir = outDir;
  opt.dataDir = dataDir;
  opt.namePrefix = prefix;
  opt.logKeepFiles = logKeep;
  opt.crashReportsMax = crashMax;
  opt.extraDirMaxFiles = extraDirMaxFiles;

  if (!logPath.empty()) {
    opt.logPath = logPath;
  } else {
    if (const char* envLog = GetEnvCStr("PROCISOCITY_LOG_FILE")) {
      opt.logPath = fs::path(envLog);
    } else {
      opt.logPath = fs::path("proc_isocity.log");
    }
  }

  if (!prefsPath.empty()) {
    opt.visualPrefsPath = prefsPath;
  }

  for (const auto& p : extraFiles) {
    opt.extraFiles.push_back(p);
  }

  {
    std::ostringstream ss;
    ss << "ProcIsoCity support bundle (proc_isocity_support)\n";
    ss << "version: " << ProcIsoCityFullVersionString() << "\n";
    ss << "build: " << ProcIsoCityBuildStamp() << "\n";
    ss << "cwd: " << cwd.string() << "\n";
    ss << "dataDir: " << dataDir.string() << "\n";
    if (!diagExtra.empty()) {
      ss << "\nUser notes:\n";
      ss << diagExtra;
      if (!diagExtra.empty() && diagExtra.back() != '\n') ss << "\n";
    }
    opt.diagnosticsText = ss.str();
  }

  if (zip) {
    SupportBundleArchiveResult res;
    std::string err;
    if (!CreateSupportBundleZip(opt, res, err)) {
      std::cerr << "Failed to create support bundle zip: " << err << "\n";
      return 1;
    }

    std::cout << res.archivePath.string() << "\n";
    for (const auto& w : res.warnings) {
      std::cerr << "Warning: " << w << "\n";
    }
    return 0;
  }

  SupportBundleResult res;
  std::string err;
  if (!CreateSupportBundle(opt, res, err)) {
    std::cerr << "Failed to create support bundle: " << err << "\n";
    return 1;
  }

  std::cout << res.bundleDir.string() << "\n";
  for (const auto& w : res.warnings) {
    std::cerr << "Warning: " << w << "\n";
  }
  return 0;
}
