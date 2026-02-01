#pragma once

// Shared CLI parsing + a couple of small filesystem helpers.
//
// Many ProcIsoCity CLI tools historically duplicated tiny parsing helpers.
// Centralizing them keeps behavior consistent across tools (hex seeds, strict
// finite floats, consistent WxH parsing) and reduces maintenance churn.

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace isocity::cli {

inline bool EnsureDir(const std::filesystem::path& p)
{
  if (p.empty()) return false;
  std::error_code ec;
  std::filesystem::create_directories(p, ec);
  if (ec) return false;
  return std::filesystem::exists(p);
}

inline bool EnsureParentDir(const std::filesystem::path& file)
{
  if (file.empty()) return false;
  std::error_code ec;
  const std::filesystem::path parent = file.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) return false;
  }
  return true;
}

inline bool ParseI32(std::string_view s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  // std::from_chars for signed ints is locale-independent and non-throwing.
  // It does not accept leading '+' on some standard library implementations.
  if (!s.empty() && s.front() == '+') s.remove_prefix(1);
  if (s.empty()) return false;

  int v = 0;
  const char* begin = s.data();
  const char* end = s.data() + s.size();
  const auto res = std::from_chars(begin, end, v, 10);
  if (res.ec != std::errc() || res.ptr != end) return false;
  *out = v;
  return true;
}

inline bool ParseU64(std::string_view s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  int base = 10;
  if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    base = 16;
    s.remove_prefix(2);
  }
  if (s.empty()) return false;

  std::uint64_t v = 0;
  const char* begin = s.data();
  const char* end = s.data() + s.size();
  const auto res = std::from_chars(begin, end, v, base);
  if (res.ec != std::errc() || res.ptr != end) return false;
  *out = v;
  return true;
}

inline bool ParseF64(std::string_view s, double* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  // strtod is widely available; we keep it strict:
  //  - entire string must parse
  //  - finite only (reject inf/nan)
  std::string tmp(s);
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(tmp.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = v;
  return true;
}

inline bool ParseF32(std::string_view s, float* out)
{
  if (!out) return false;
  double v = 0.0;
  if (!ParseF64(s, &v)) return false;
  // clamp to finite float range
  if (v < -static_cast<double>(std::numeric_limits<float>::max()) ||
      v > static_cast<double>(std::numeric_limits<float>::max())) {
    return false;
  }
  *out = static_cast<float>(v);
  return true;
}

inline bool ParseBool01(std::string_view s, bool* out)
{
  if (!out) return false;
  if (s == "0" || s == "false" || s == "FALSE" || s == "False" || s == "off" || s == "OFF" ||
      s == "Off" || s == "no" || s == "NO" || s == "No") {
    *out = false;
    return true;
  }
  if (s == "1" || s == "true" || s == "TRUE" || s == "True" || s == "on" || s == "ON" ||
      s == "On" || s == "yes" || s == "YES" || s == "Yes") {
    *out = true;
    return true;
  }
  return false;
}

inline bool ParseWxH(std::string_view s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string_view::npos) return false;
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, pos), &w)) return false;
  if (!ParseI32(s.substr(pos + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

inline bool ParseF32Triple(std::string_view s, float* outA, float* outB, float* outC)
{
  if (!outA || !outB || !outC) return false;
  const std::size_t p0 = s.find_first_of(",xX");
  if (p0 == std::string_view::npos) return false;
  const std::size_t p1 = s.find_first_of(",xX", p0 + 1);
  if (p1 == std::string_view::npos) return false;
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
  if (!ParseF32(s.substr(0, p0), &a)) return false;
  if (!ParseF32(s.substr(p0 + 1, p1 - (p0 + 1)), &b)) return false;
  if (!ParseF32(s.substr(p1 + 1), &c)) return false;
  *outA = a;
  *outB = b;
  *outC = c;
  return true;
}

inline bool ParseU8Triple(std::string_view s, std::uint8_t* outA, std::uint8_t* outB, std::uint8_t* outC)
{
  if (!outA || !outB || !outC) return false;
  float fa = 0.0f;
  float fb = 0.0f;
  float fc = 0.0f;
  if (!ParseF32Triple(s, &fa, &fb, &fc)) return false;
  auto clampU8 = [](float v) -> std::uint8_t {
    const int i = static_cast<int>(std::lround(v));
    return static_cast<std::uint8_t>(std::clamp(i, 0, 255));
  };
  *outA = clampU8(fa);
  *outB = clampU8(fb);
  *outC = clampU8(fc);
  return true;
}

inline std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

inline std::vector<std::string> SplitCommaList(std::string_view s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(c))) continue;
    cur.push_back(c);
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

} // namespace isocity::cli
