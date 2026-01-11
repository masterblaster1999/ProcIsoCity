#include "isocity/Export.hpp"
#include "isocity/Json.hpp"

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

[[maybe_unused]] std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
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

  if (!ComparePpm(a, b, stats, threshold, diffPtr)) {
    std::cerr << "ComparePpm failed (dimension mismatch or invalid buffers)\n";
    return 2;
  }

  const bool match = (stats.pixelsDifferent == 0);

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
    using isocity::JsonValue;
    JsonValue root = JsonValue::MakeObject();
    auto add = [](JsonValue& obj, const char* key, JsonValue v) {
      obj.objectValue.emplace_back(key, std::move(v));
    };

    add(root, "fileA", JsonValue::MakeString(pathA));
    add(root, "fileB", JsonValue::MakeString(pathB));
    add(root, "width", JsonValue::MakeNumber(static_cast<double>(stats.width)));
    add(root, "height", JsonValue::MakeNumber(static_cast<double>(stats.height)));
    add(root, "threshold", JsonValue::MakeNumber(static_cast<double>(threshold)));
    add(root, "pixelsCompared", JsonValue::MakeNumber(static_cast<double>(stats.pixelsCompared)));
    add(root, "pixelsDifferent", JsonValue::MakeNumber(static_cast<double>(stats.pixelsDifferent)));
    add(root, "maxAbsDiff", JsonValue::MakeNumber(static_cast<double>(stats.maxAbsDiff)));
    add(root, "meanAbsDiff", JsonValue::MakeNumber(static_cast<double>(stats.meanAbsDiff)));
    add(root, "mse", JsonValue::MakeNumber(static_cast<double>(stats.mse)));
    if (std::isinf(stats.psnr)) {
      add(root, "psnr", JsonValue::MakeNull());
      add(root, "psnrIsInf", JsonValue::MakeBool(true));
    } else {
      add(root, "psnr", JsonValue::MakeNumber(static_cast<double>(stats.psnr)));
      add(root, "psnrIsInf", JsonValue::MakeBool(false));
    }
    add(root, "match", JsonValue::MakeBool(match));

    std::string jsonErr;
    if (!isocity::WriteJsonFile(outJsonPath, root, jsonErr, isocity::JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false})) {
      std::cerr << "failed to write json report: " << jsonErr << "\n";
      return 2;
    }
  }

  return match ? 0 : 1;
}
