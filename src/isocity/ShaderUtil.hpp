#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "isocity/RaylibShim.hpp"

namespace isocity {

// Minimal GLSL shader override + preprocessing utilities.
//
// Goals:
//  - Allow modders/devs to drop shader files into a `shaders/` directory
//    without changing C++.
//  - Support a lightweight `#include "file.glsl"` directive by preprocessing
//    sources on the CPU before compilation.
//  - Preserve a safe fallback to embedded shader strings if overrides are
//    missing or fail to compile.
//
// Notes:
//  - `#include` is not a core GLSL feature on many drivers; we intentionally
//    implement it ourselves.
//  - This module is renderer-side (raylib/OpenGL) and should only be linked
//    into the interactive app target.

struct ShaderOverrideSearch {
  std::filesystem::path dir;                       // empty => not found
  std::vector<std::filesystem::path> triedPaths;   // for diagnostics
};

// Try to find a `shaders/` directory by searching from the current working
// directory upward. This helps when running from `build/`.
ShaderOverrideSearch FindShaderOverrideDir(int maxParentHops = 4);

struct ShaderSourceLoad {
  std::string vs;
  std::string fs;
  bool vsFromFile = false;
  bool fsFromFile = false;
  std::filesystem::path vsPath;
  std::filesystem::path fsPath;
};

struct ShaderBuildResult {
  Shader shader{};          // shader.id == 0 on failure
  std::string log;          // compiler/linker log (may be empty)
  ShaderSourceLoad source;  // resolved sources
};

// Load/compile a shader program.
//
// - Looks for `<name>.vs.glsl` and `<name>.fs.glsl` in the discovered override
//   directory. Missing stages fall back to the provided embedded strings.
// - Preprocesses `#include "..."` directives (relative to the including file).
// - Optionally injects preprocessor defines after the `#version` line.
ShaderBuildResult LoadShaderProgramWithOverrides(
    std::string_view name,
    const char* fallbackVS,
    const char* fallbackFS,
    const std::vector<std::string>& defineLines = {},
    int maxParentHops = 4);

} // namespace isocity
