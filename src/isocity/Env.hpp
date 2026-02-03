#pragma once

#include <cstdlib>
#include <optional>
#include <string>

namespace isocity {

// Cross-platform environment variable read helper.
//
// MSVC warns that getenv is "unsafe" (C4996) unless _CRT_SECURE_NO_WARNINGS is set.
// We use _dupenv_s on MSVC so callers can read env vars without warnings.
inline std::optional<std::string> GetEnvVar(const char* name)
{
  if (!name || !*name) return std::nullopt;

#if defined(_WIN32) && defined(_MSC_VER)
  char* buf = nullptr;
  std::size_t len = 0;
  if (_dupenv_s(&buf, &len, name) != 0 || !buf) return std::nullopt;

  std::string out(buf);
  std::free(buf);
  if (out.empty()) return std::nullopt;
  return out;
#else
  const char* v = std::getenv(name);
  if (!v || !*v) return std::nullopt;
  return std::string(v);
#endif
}

} // namespace isocity
