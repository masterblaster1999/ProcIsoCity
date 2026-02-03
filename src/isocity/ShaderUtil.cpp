#include "isocity/ShaderUtil.hpp"

#include "isocity/AppPaths.hpp"
#include "isocity/Env.hpp"
#include "isocity/RaylibTrace.hpp"

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace isocity {

namespace {

static thread_local std::string* g_traceCapture = nullptr;

static const char* TraceLevelName(int level)
{
  switch (level) {
    case LOG_TRACE: return "TRACE";
    case LOG_DEBUG: return "DEBUG";
    case LOG_INFO: return "INFO";
    case LOG_WARNING: return "WARNING";
    case LOG_ERROR: return "ERROR";
    case LOG_FATAL: return "FATAL";
    default: return "LOG";
  }
}

static void TraceCaptureCallback(int logLevel, const char* text, va_list args)
{
  if (!g_traceCapture) return;

  char buf[1024];
  va_list copy;
  va_copy(copy, args);
  std::vsnprintf(buf, sizeof(buf), text, copy);
  va_end(copy);

  (*g_traceCapture) += "[raylib ";
  (*g_traceCapture) += TraceLevelName(logLevel);
  (*g_traceCapture) += "] ";
  (*g_traceCapture) += buf;
  if (!g_traceCapture->empty() && g_traceCapture->back() != '\n') {
    (*g_traceCapture) += "\n";
  }
}

struct ScopedTraceCapture {
  bool enabled = false;
  RaylibTraceLogCallback prev = nullptr;

  explicit ScopedTraceCapture(std::string* out)
  {
    if (!out) return;
    g_traceCapture = out;
    // Preserve any callback installed by the app (e.g. RaylibLog) so we don't
    // accidentally disable global logging after compiling a shader.
    prev = GetRaylibTraceLogCallback();
    SetRaylibTraceLogCallback(TraceCaptureCallback);
    enabled = true;
  }

  ~ScopedTraceCapture()
  {
    if (!enabled) return;
    // Restore the previous callback.
    SetRaylibTraceLogCallback(prev);
    g_traceCapture = nullptr;
  }
};

static std::string StripUtf8Bom(std::string s)
{
  if (s.size() >= 3 && static_cast<unsigned char>(s[0]) == 0xEF &&
      static_cast<unsigned char>(s[1]) == 0xBB && static_cast<unsigned char>(s[2]) == 0xBF) {
    s.erase(s.begin(), s.begin() + 3);
  }
  return s;
}

static bool ReadTextFile(const std::filesystem::path& path, std::string& outText, std::string& outErr)
{
  outErr.clear();
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    outErr = "Failed to open file: " + path.string();
    return false;
  }
  std::ostringstream oss;
  oss << ifs.rdbuf();
  outText = StripUtf8Bom(oss.str());
  return true;
}

static inline std::string LTrim(std::string s)
{
  std::size_t i = 0;
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  if (i > 0) s.erase(0, i);
  return s;
}

static bool ParseIncludeDirective(const std::string& line, std::string& outFile)
{
  // Matches:   #include "file"   (with optional leading whitespace)
  outFile.clear();
  std::string s = LTrim(line);
  if (s.rfind("#include", 0) != 0) return false;

  // Find first quote.
  const std::size_t q0 = s.find('"');
  if (q0 == std::string::npos) return false;
  const std::size_t q1 = s.find('"', q0 + 1);
  if (q1 == std::string::npos || q1 <= q0 + 1) return false;
  outFile = s.substr(q0 + 1, q1 - (q0 + 1));
  return !outFile.empty();
}

static bool LooksLikeVersionLine(const std::string& line)
{
  std::string s = LTrim(line);
  return s.rfind("#version", 0) == 0;
}

static std::string InjectDefinesAfterVersion(const std::string& src, const std::vector<std::string>& defineLines)
{
  if (defineLines.empty()) return src;

  std::istringstream iss(src);
  std::string line;
  std::vector<std::string> lines;
  lines.reserve(256);
  while (std::getline(iss, line)) {
    // Preserve original lines without the trailing '\n' from getline.
    lines.push_back(line);
  }

  int versionLine = -1;
  for (int i = 0; i < static_cast<int>(lines.size()); ++i) {
    if (LooksLikeVersionLine(lines[i])) {
      versionLine = i;
      break;
    }
    // Stop searching once we hit a non-empty, non-comment line.
    const std::string t = LTrim(lines[i]);
    if (!t.empty() && t.rfind("//", 0) != 0) {
      break;
    }
  }

  std::ostringstream oss;
  if (versionLine < 0) {
    // Default to GLSL 330 on desktop; this project targets raylib OpenGL 3.3.
    oss << "#version 330\n";
    for (const auto& d : defineLines) oss << d << "\n";
    oss << src;
    if (!src.empty() && src.back() != '\n') oss << "\n";
    return oss.str();
  }

  // If the shader has only whitespace or line comments before #version, drop
  // that prefix so `#version` becomes the first line (some drivers are strict).
  int startLine = 0;
  if (versionLine > 0) {
    bool prefixSkippable = true;
    for (int i = 0; i < versionLine; ++i) {
      const std::string t = LTrim(lines[i]);
      if (!t.empty() && t.rfind("//", 0) != 0) {
        prefixSkippable = false;
        break;
      }
    }
    if (prefixSkippable) startLine = versionLine;
  }

  for (int i = startLine; i < static_cast<int>(lines.size()); ++i) {
    oss << lines[i] << "\n";
    if (i == versionLine) {
      for (const auto& d : defineLines) oss << d << "\n";
    }
  }
  return oss.str();
}

static std::string PreprocessIncludesRecursive(const std::string& src,
                                               const std::filesystem::path& curFile,
                                               std::vector<std::filesystem::path>& includeStack,
                                               std::unordered_set<std::string>& pragmaOnceFiles,
                                               std::string& outErr,
                                               int depth)
{
  if (depth > 32) {
    outErr = "GLSL include depth exceeded (possible recursive include).";
    return {};
  }

  // Detect `#pragma once` if it is the first directive-like thing in the file.
  // We treat it as advisory (best-effort).
  bool sawNonEmpty = false;
  bool hasPragmaOnce = false;

  std::istringstream iss(src);
  std::ostringstream oss;

  std::string line;
  while (std::getline(iss, line)) {
    const std::string t = LTrim(line);
    if (!sawNonEmpty) {
      if (t.empty() || t.rfind("//", 0) == 0) {
        // ignore
      } else {
        sawNonEmpty = true;
        if (t.rfind("#pragma once", 0) == 0) {
          hasPragmaOnce = true;
          // Do not emit this line.
          continue;
        }
      }
    }

    std::string inc;
    if (ParseIncludeDirective(line, inc)) {
      const std::filesystem::path incPath = curFile.parent_path() / inc;

      // Normalize for stack comparisons.
      std::error_code ec;
      const std::filesystem::path abs = std::filesystem::weakly_canonical(incPath, ec);
      const std::string absKey = ec ? incPath.lexically_normal().string() : abs.string();

      // Honor pragma once.
      if (pragmaOnceFiles.find(absKey) != pragmaOnceFiles.end()) {
        oss << "// [ShaderUtil] skipped #include (pragma once): " << inc << "\n";
        continue;
      }

      // Prevent infinite recursion.
      for (const auto& p : includeStack) {
        std::error_code ec2;
        const std::filesystem::path ap = std::filesystem::weakly_canonical(p, ec2);
        const std::string pKey = ec2 ? p.lexically_normal().string() : ap.string();
        if (pKey == absKey) {
          oss << "// [ShaderUtil] skipped recursive #include: " << inc << "\n";
          inc.clear();
          break;
        }
      }
      if (inc.empty()) continue;

      std::string incText;
      if (!ReadTextFile(incPath, incText, outErr)) {
        outErr = "Failed to read include '" + inc + "' from '" + curFile.string() + "': " + outErr;
        return {};
      }

      includeStack.push_back(incPath);
      oss << "// [ShaderUtil] begin include: " << inc << "\n";
      std::string expanded =
          PreprocessIncludesRecursive(incText, incPath, includeStack, pragmaOnceFiles, outErr, depth + 1);
      if (!outErr.empty()) return {};
      oss << expanded;
      if (!expanded.empty() && expanded.back() != '\n') oss << "\n";
      oss << "// [ShaderUtil] end include: " << inc << "\n";
      includeStack.pop_back();
      continue;
    }

    oss << line << "\n";
  }

  if (hasPragmaOnce) {
    std::error_code ec;
    const std::filesystem::path abs = std::filesystem::weakly_canonical(curFile, ec);
    const std::string key = ec ? curFile.lexically_normal().string() : abs.string();
    pragmaOnceFiles.insert(key);
  }

  return oss.str();
}

static void BindCommonRaylibLocations(Shader& sh)
{
  if (sh.id == 0) return;

  // Core MVP.
  sh.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(sh, "mvp");

  // Common 2D texture/tint uniforms used by raylib's batch.
  sh.locs[SHADER_LOC_COLOR_DIFFUSE] = GetShaderLocation(sh, "colDiffuse");
  sh.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(sh, "texture0");
}

static Shader CompileRaylibShaderFromStrings(const char* vsCode, const char* fsCode, std::string* outLog)
{
  ScopedTraceCapture cap(outLog);
  Shader sh = LoadShaderFromMemory(vsCode, fsCode);
  BindCommonRaylibLocations(sh);
  return sh;
}

} // namespace

ShaderOverrideSearch FindShaderOverrideDir(int maxParentHops)
{
  ShaderOverrideSearch out;

  // Deduplicate attempted paths (FindShaderOverrideDir is often called for diagnostics).
  std::unordered_set<std::string> triedKeys;

  auto tryDir = [&](const std::filesystem::path& d) -> bool {
    const std::string key = d.lexically_normal().string();
    if (triedKeys.insert(key).second) {
      out.triedPaths.push_back(d);
    }
    std::error_code ec2;
    if (std::filesystem::exists(d, ec2) && std::filesystem::is_directory(d, ec2)) {
      out.dir = d;
      return true;
    }
    return false;
  };

  auto searchUpwardFrom = [&](std::filesystem::path p) -> bool {
    for (int hop = 0; hop <= maxParentHops; ++hop) {
      if (tryDir(p / "shaders")) return true;
      if (tryDir(p / "assets" / "shaders")) return true;
      const std::filesystem::path parent = p.parent_path();
      if (parent == p || parent.empty()) break;
      p = parent;
    }
    return false;
  };

  // 1) Explicit override dir.
  const auto env = GetEnvVar("PROCISOCITY_SHADER_DIR");
  if (env) {
    const std::filesystem::path e(*env);
    if (tryDir(e)) return out;
    if (tryDir(e / "shaders")) return out;
    if (tryDir(e / "assets" / "shaders")) return out;
  }

  // 2) Search from current working directory upward.
  std::error_code ec;
  std::filesystem::path cwd = std::filesystem::current_path(ec);
  if (ec || cwd.empty()) cwd = std::filesystem::path(".");
  if (searchUpwardFrom(cwd)) return out;

  // 3) Search from executable directory upward (supports installed binaries / portable zips).
  const std::filesystem::path exeDir = AppPaths::ExecutableDir();
  if (!exeDir.empty()) {
    if (searchUpwardFrom(exeDir)) return out;
  }

  return out;
}

ShaderBuildResult LoadShaderProgramWithOverrides(std::string_view name,
                                                 const char* fallbackVS,
                                                 const char* fallbackFS,
                                                 const std::vector<std::string>& defineLines,
                                                 int maxParentHops)
{
  ShaderBuildResult out;

  // Resolve override directory.
  const ShaderOverrideSearch search = FindShaderOverrideDir(maxParentHops);
  const std::filesystem::path dir = search.dir;

  const std::filesystem::path vsPath =
      dir.empty() ? std::filesystem::path{} : (dir / (std::string(name) + ".vs.glsl"));
  const std::filesystem::path fsPath =
      dir.empty() ? std::filesystem::path{} : (dir / (std::string(name) + ".fs.glsl"));

  // Load sources (file if present, else fallback).
  std::string err;
  std::error_code ec;
  const bool vsExists = (!vsPath.empty()) && std::filesystem::exists(vsPath, ec) && !ec;
  ec.clear();
  const bool fsExists = (!fsPath.empty()) && std::filesystem::exists(fsPath, ec) && !ec;

  if (vsExists) {
    out.source.vsFromFile = ReadTextFile(vsPath, out.source.vs, err);
    out.source.vsPath = vsPath;
    if (!out.source.vsFromFile) {
      TraceLog(LOG_WARNING, "[ShaderUtil] %s", err.c_str());
      out.source.vs = (fallbackVS != nullptr) ? std::string(fallbackVS) : std::string{};
    }
  } else {
    out.source.vs = (fallbackVS != nullptr) ? std::string(fallbackVS) : std::string{};
  }

  if (fsExists) {
    out.source.fsFromFile = ReadTextFile(fsPath, out.source.fs, err);
    out.source.fsPath = fsPath;
    if (!out.source.fsFromFile) {
      TraceLog(LOG_WARNING, "[ShaderUtil] %s", err.c_str());
      out.source.fs = (fallbackFS != nullptr) ? std::string(fallbackFS) : std::string{};
    }
  } else {
    out.source.fs = (fallbackFS != nullptr) ? std::string(fallbackFS) : std::string{};
  }

  // Preprocess includes (per-stage). NOTE: `#pragma once` behavior must not leak
  // across stages, since vertex + fragment shaders are compiled separately.
  //
  // Important: preprocessing failures should not brick rendering. If an override stage
  // fails to preprocess (e.g., missing #include), fall back to the embedded stage for
  // that shader stage and continue.
  if (out.source.vsFromFile) {
    std::string perr;
    std::vector<std::filesystem::path> stack;
    std::unordered_set<std::string> once;
    stack.push_back(out.source.vsPath);

    std::string expanded =
        PreprocessIncludesRecursive(out.source.vs, out.source.vsPath, stack, once, perr, 0);

    if (!perr.empty()) {
      out.log += "[ShaderUtil] Vertex shader preprocessing failed for ";
      out.log += out.source.vsPath.string();
      out.log += ":\n";
      out.log += perr;
      if (!out.log.empty() && out.log.back() != '\n') out.log += "\n";
      out.log += "[ShaderUtil] Falling back to embedded vertex shader stage.\n";

      out.source.vs = (fallbackVS != nullptr) ? std::string(fallbackVS) : std::string{};
      out.source.vsFromFile = false;
      out.source.vsPath.clear();
    } else {
      out.source.vs = std::move(expanded);
    }
  }

  if (out.source.fsFromFile) {
    std::string perr;
    std::vector<std::filesystem::path> stack;
    std::unordered_set<std::string> once;
    stack.push_back(out.source.fsPath);

    std::string expanded =
        PreprocessIncludesRecursive(out.source.fs, out.source.fsPath, stack, once, perr, 0);

    if (!perr.empty()) {
      out.log += "[ShaderUtil] Fragment shader preprocessing failed for ";
      out.log += out.source.fsPath.string();
      out.log += ":\n";
      out.log += perr;
      if (!out.log.empty() && out.log.back() != '\n') out.log += "\n";
      out.log += "[ShaderUtil] Falling back to embedded fragment shader stage.\n";

      out.source.fs = (fallbackFS != nullptr) ? std::string(fallbackFS) : std::string{};
      out.source.fsFromFile = false;
      out.source.fsPath.clear();
    } else {
      out.source.fs = std::move(expanded);
    }
  }


  // Decide which stages we can provide to raylib. If a fallback stage is not provided
  // (nullptr) and there is no override file, pass nullptr to let raylib use its
  // built-in default stage.
  const bool haveVS = out.source.vsFromFile || (fallbackVS != nullptr);
  const bool haveFS = out.source.fsFromFile || (fallbackFS != nullptr);

  // Inject define lines after the `#version` line for stages we provide.
  if (haveVS) out.source.vs = InjectDefinesAfterVersion(out.source.vs, defineLines);
  if (haveFS) out.source.fs = InjectDefinesAfterVersion(out.source.fs, defineLines);

  // Compile using raylib. This avoids depending on OpenGL loader headers (GLAD/GLEW)
  // which can vary across system-installed raylib packages.
  const bool captureCompileLog = (out.source.vsFromFile || out.source.fsFromFile);
  out.shader = CompileRaylibShaderFromStrings(haveVS ? out.source.vs.c_str() : nullptr,
                                             haveFS ? out.source.fs.c_str() : nullptr,
                                             captureCompileLog ? &out.log : nullptr);

  // Safety: if an on-disk override was used but failed to compile/link, fall back to
  // the embedded shader strings so a broken override doesn't brick rendering.
  if (out.shader.id == 0 && (out.source.vsFromFile || out.source.fsFromFile)) {
    std::string fbVS = (fallbackVS != nullptr) ? std::string(fallbackVS) : std::string{};
    std::string fbFS = (fallbackFS != nullptr) ? std::string(fallbackFS) : std::string{};

    const bool haveFbVS = (fallbackVS != nullptr);
    const bool haveFbFS = (fallbackFS != nullptr);

    if (haveFbVS) fbVS = InjectDefinesAfterVersion(fbVS, defineLines);
    if (haveFbFS) fbFS = InjectDefinesAfterVersion(fbFS, defineLines);

    out.log += "[ShaderUtil] Override compile failed; attempting embedded fallback...\n";

    Shader fb = CompileRaylibShaderFromStrings(haveFbVS ? fbVS.c_str() : nullptr,
                                               haveFbFS ? fbFS.c_str() : nullptr,
                                               &out.log);
    if (fb.id != 0) {
      out.shader = fb;

      // Use the embedded sources as the authoritative ones.
      out.source.vs = std::move(fbVS);
      out.source.fs = std::move(fbFS);
      out.source.vsFromFile = false;
      out.source.fsFromFile = false;
      out.source.vsPath.clear();
      out.source.fsPath.clear();
    }
  }

  return out;
}

} // namespace isocity
