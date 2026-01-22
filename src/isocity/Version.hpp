#pragma once

#include <string>

// Build/version metadata for ProcIsoCity.
//
// CMake defines these macros for all targets via isocity_core's PUBLIC compile
// definitions (see CMakeLists.txt). We still provide conservative fallbacks so
// the header remains usable in IDEs or non-CMake builds.

#ifndef PROCISOCITY_VERSION_MAJOR
#define PROCISOCITY_VERSION_MAJOR 0
#endif

#ifndef PROCISOCITY_VERSION_MINOR
#define PROCISOCITY_VERSION_MINOR 0
#endif

#ifndef PROCISOCITY_VERSION_PATCH
#define PROCISOCITY_VERSION_PATCH 0
#endif

#ifndef PROCISOCITY_VERSION_STRING
#define PROCISOCITY_VERSION_STRING "0.0.0"
#endif

#ifndef PROCISOCITY_GIT_SHA
#define PROCISOCITY_GIT_SHA "unknown"
#endif

namespace isocity {

struct ProcIsoCityVersion {
  int major;
  int minor;
  int patch;
};

inline constexpr ProcIsoCityVersion ProcIsoCityVersionNumbers()
{
  return ProcIsoCityVersion{PROCISOCITY_VERSION_MAJOR, PROCISOCITY_VERSION_MINOR, PROCISOCITY_VERSION_PATCH};
}

inline constexpr const char* ProcIsoCityVersionString()
{
  return PROCISOCITY_VERSION_STRING;
}

inline constexpr const char* ProcIsoCityGitSha()
{
  return PROCISOCITY_GIT_SHA;
}

inline std::string ProcIsoCityFullVersionString()
{
  std::string s = std::string(ProcIsoCityVersionString());
  const std::string sha = std::string(ProcIsoCityGitSha());
  if (!sha.empty() && sha != "unknown") {
    s += " (";
    s += sha;
    s += ")";
  }
  return s;
}

inline constexpr const char* ProcIsoCityBuildDate()
{
  return __DATE__;
}

inline constexpr const char* ProcIsoCityBuildTime()
{
  return __TIME__;
}

} // namespace isocity
