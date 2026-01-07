#include "isocity/Export.hpp"

#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {

[[maybe_unused]] std::string Trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

bool ParseF64(const std::string& s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = v;
  return true;
}

[[maybe_unused]] std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

std::string EscapeJson(const std::string& s)
{
  std::ostringstream oss;
  for (unsigned char uc : s) {
    const char c = static_cast<char>(uc);
    switch (c) {
      case '\\': oss << "\\\\"; break;
      case '"': oss << "\\\""; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default:
        if (uc < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(uc) << std::dec;
        } else {
          oss << c;
        }
        break;
    }
  }
  return oss.str();
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_imagediff (PPM/PNG comparison tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_imagediff <A.ppm|A.png> <B.ppm|B.png> [options]\n\n"
      << "Exit codes:\n"
      << "  0  images match (within threshold)\n"
      << "  1  images differ\n"
      << "  2  error (bad args or IO)\n\n"
      << "Options:\n"
      << "  --out <diff.ppm|diff.png>  Write an absolute-difference visualization (PPM/PNG).\n"
      << "  --threshold <N>            Per-channel tolerance (0..255). Default: 0\n"
      << "  --ssim-window <N>          SSIM window size (odd >=3). Default: 11\n"
      << "  --match-ssim <f>           Consider images a MATCH if SSIM >= f (0..1).\n"
      << "  --json <out.json>          Write a JSON summary of diff stats.\n"
      << "  --quiet                    Suppress stdout summary (errors still print).\n"
      << "  -h, --help                 Show this help.\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  if (argc < 3) {
    PrintHelp();
    return 2;
  }

  std::string pathA;
  std::string pathB;
  std::string outDiffPath;
  std::string outJsonPath;
  int threshold = 0;
  int ssimWindow = 11;
  double matchSsimMin = -1.0;
  bool quiet = false;

  // Positional args (first two non-flag tokens).
  int posCount = 0;

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
    } else if (arg == "--out") {
      if (!requireValue(i, val)) {
        std::cerr << "--out requires a path\n";
        return 2;
      }
      outDiffPath = val;
    } else if (arg == "--threshold") {
      if (!requireValue(i, val)) {
        std::cerr << "--threshold requires an integer\n";
        return 2;
      }
      if (!ParseI32(val, &threshold) || threshold < 0 || threshold > 255) {
        std::cerr << "invalid --threshold (expected 0..255): " << val << "\n";
        return 2;
      }
    } else if (arg == "--json") {
      if (!requireValue(i, val)) {
        std::cerr << "--json requires a path\n";
        return 2;
      }
      outJsonPath = val;
    } else if (arg == "--ssim-window") {
      if (!requireValue(i, val)) {
        std::cerr << "--ssim-window requires an integer\n";
        return 2;
      }
      if (!ParseI32(val, &ssimWindow) || ssimWindow < 3 || ssimWindow > 999) {
        std::cerr << "invalid --ssim-window (expected odd integer >=3): " << val << "\n";
        return 2;
      }
      if ((ssimWindow % 2) == 0) {
        // We enforce an odd window (required for symmetric radius).
        ++ssimWindow;
      }
    } else if (arg == "--match-ssim") {
      if (!requireValue(i, val)) {
        std::cerr << "--match-ssim requires a float\n";
        return 2;
      }
      if (!ParseF64(val, &matchSsimMin) || matchSsimMin < 0.0 || matchSsimMin > 1.0) {
        std::cerr << "invalid --match-ssim (expected 0..1): " << val << "\n";
        return 2;
      }
    } else if (arg == "--quiet") {
      quiet = true;
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    } else {
      // Positional path.
      if (posCount == 0) pathA = arg;
      else if (posCount == 1) pathB = arg;
      else {
        std::cerr << "unexpected extra positional argument: " << arg << "\n";
        return 2;
      }
      ++posCount;
    }
  }

  if (pathA.empty() || pathB.empty()) {
    PrintHelp();
    return 2;
  }

  PpmImage a;
  PpmImage b;
  std::string err;

  if (!ReadImageAuto(pathA, a, err)) {
    std::cerr << "ReadImage failed for A: " << err << "\n";
    return 2;
  }
  if (!ReadImageAuto(pathB, b, err)) {
    std::cerr << "ReadImage failed for B: " << err << "\n";
    return 2;
  }

  PpmDiffStats stats;
  PpmImage diffImg;
  PpmImage* diffPtr = outDiffPath.empty() ? nullptr : &diffImg;

  if (!ComparePpm(a, b, stats, threshold, diffPtr, ssimWindow)) {
    std::cerr << "ComparePpm failed (dimension mismatch or invalid buffers)\n";
    return 2;
  }

  bool match = (stats.pixelsDifferent == 0);
  if (matchSsimMin >= 0.0) {
    // SSIM is a perceptual similarity metric, so it can be a better regression gate
    // than exact pixel matching for certain rendering pipelines.
    match = match || (stats.ssim >= matchSsimMin);
  }

  if (!quiet) {
    std::cout << "A: " << pathA << "\n";
    std::cout << "B: " << pathB << "\n";
    std::cout << "size: " << stats.width << "x" << stats.height << "\n";
    std::cout << "threshold: " << threshold << "\n";
    std::cout << "pixelsCompared: " << stats.pixelsCompared << "\n";
    std::cout << "pixelsDifferent: " << stats.pixelsDifferent << "\n";
    std::cout << "maxAbsDiff: " << static_cast<int>(stats.maxAbsDiff) << "\n";
    std::cout << "meanAbsDiff: " << std::fixed << std::setprecision(6) << stats.meanAbsDiff << "\n";
    std::cout << "mse: " << std::fixed << std::setprecision(6) << stats.mse << "\n";
    if (std::isinf(stats.psnr)) {
      std::cout << "psnr: inf\n";
    } else {
      std::cout << "psnr: " << std::fixed << std::setprecision(3) << stats.psnr << " dB\n";
    }
    std::cout << "ssim: " << std::fixed << std::setprecision(6) << stats.ssim << "\n";
    std::cout << "result: " << (match ? "MATCH" : "DIFFER") << "\n";
  }

  // Write diff image if requested.
  if (!outDiffPath.empty()) {
    if (!WriteImageAuto(outDiffPath, diffImg, err)) {
      std::cerr << "WriteImage failed for diff output: " << err << "\n";
      return 2;
    }
  }

  // JSON report.
  if (!outJsonPath.empty()) {
    std::ofstream f(outJsonPath, std::ios::binary);
    if (!f) {
      std::cerr << "failed to write json report: " << outJsonPath << "\n";
      return 2;
    }

    f << "{\n";
    f << "  \"fileA\": \"" << EscapeJson(pathA) << "\",\n";
    f << "  \"fileB\": \"" << EscapeJson(pathB) << "\",\n";
    f << "  \"width\": " << stats.width << ",\n";
    f << "  \"height\": " << stats.height << ",\n";
    f << "  \"threshold\": " << threshold << ",\n";
    f << "  \"pixelsCompared\": " << stats.pixelsCompared << ",\n";
    f << "  \"pixelsDifferent\": " << stats.pixelsDifferent << ",\n";
    f << "  \"maxAbsDiff\": " << static_cast<int>(stats.maxAbsDiff) << ",\n";
    f << "  \"meanAbsDiff\": " << std::fixed << std::setprecision(9) << stats.meanAbsDiff << ",\n";
    f << "  \"mse\": " << std::fixed << std::setprecision(9) << stats.mse << ",\n";
    if (std::isinf(stats.psnr)) {
      f << "  \"psnr\": null,\n";
      f << "  \"psnrIsInf\": true,\n";
    } else {
      f << "  \"psnr\": " << std::fixed << std::setprecision(6) << stats.psnr << ",\n";
      f << "  \"psnrIsInf\": false,\n";
    }
    f << "  \"ssim\": " << std::fixed << std::setprecision(9) << stats.ssim << ",\n";
    f << "  \"ssimWindow\": " << ssimWindow << ",\n";
    if (matchSsimMin >= 0.0) {
      f << "  \"matchSsimMin\": " << std::fixed << std::setprecision(9) << matchSsimMin << ",\n";
    } else {
      f << "  \"matchSsimMin\": null,\n";
    }
    f << "  \"match\": " << (match ? "true" : "false") << "\n";
    f << "}\n";
  }

  return match ? 0 : 1;
}
