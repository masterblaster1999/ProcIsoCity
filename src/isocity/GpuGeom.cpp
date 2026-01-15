#include "isocity/GpuGeom.hpp"

// rlgl provides access to the underlying OpenGL calls used by raylib.
// We use it here to compile a custom program with a geometry shader stage
// and to emit GL_LINES primitives that get expanded into thick ribbons.
#include "rlgl.h"

// Raylib uses modern OpenGL, but platform headers (notably Windows GL) do not
// declare shader/program entry points by default. Raylib bundles GLAD in
// src/external/glad.h; include it if shader constants are missing.
#ifndef GL_VERTEX_SHADER
  #include "external/glad.h"
#endif

#include <array>
#include <algorithm>
#include <cstring>
#include <string>
#include <type_traits>

// Some build configurations may not expose GL_GEOMETRY_SHADER even though the
// runtime driver supports it (it is core in OpenGL 3.2+). Define it defensively.
#ifndef GL_GEOMETRY_SHADER
  #define GL_GEOMETRY_SHADER 0x8DD9
#endif

namespace isocity {

namespace {

static bool CompileStage(GLuint shaderId, const char* code, std::string& outLog)
{
  if (shaderId == 0 || code == nullptr) return false;

  glShaderSource(shaderId, 1, &code, nullptr);
  glCompileShader(shaderId);

  GLint ok = 0;
  glGetShaderiv(shaderId, GL_COMPILE_STATUS, &ok);

  GLint logLen = 0;
  glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &logLen);
  if (logLen > 1) {
    std::string log;
    log.resize(static_cast<std::size_t>(logLen));
    GLsizei written = 0;
    glGetShaderInfoLog(shaderId, logLen, &written, log.data());
    if (written > 0) {
      log.resize(static_cast<std::size_t>(written));
      outLog += log;
    }
  }

  return ok != 0;
}

static GLuint BuildProgramWithGeometry(const char* vsCode, const char* gsCode, const char* fsCode, std::string& outLog)
{
  outLog.clear();

  const GLuint vs = glCreateShader(GL_VERTEX_SHADER);
  const GLuint gs = glCreateShader(GL_GEOMETRY_SHADER);
  const GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);

  bool ok = true;
  ok = ok && CompileStage(vs, vsCode, outLog);
  ok = ok && CompileStage(gs, gsCode, outLog);
  ok = ok && CompileStage(fs, fsCode, outLog);

  if (!ok) {
    if (vs) glDeleteShader(vs);
    if (gs) glDeleteShader(gs);
    if (fs) glDeleteShader(fs);
    return 0;
  }

  const GLuint prog = glCreateProgram();
  glAttachShader(prog, vs);
  glAttachShader(prog, gs);
  glAttachShader(prog, fs);
  glLinkProgram(prog);

  GLint linked = 0;
  glGetProgramiv(prog, GL_LINK_STATUS, &linked);

  GLint logLen = 0;
  glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &logLen);
  if (logLen > 1) {
    std::string log;
    log.resize(static_cast<std::size_t>(logLen));
    GLsizei written = 0;
    glGetProgramInfoLog(prog, logLen, &written, log.data());
    if (written > 0) {
      log.resize(static_cast<std::size_t>(written));
      outLog += log;
    }
  }

  glDetachShader(prog, vs);
  glDetachShader(prog, gs);
  glDetachShader(prog, fs);
  glDeleteShader(vs);
  glDeleteShader(gs);
  glDeleteShader(fs);

  if (!linked) {
    glDeleteProgram(prog);
    return 0;
  }

  return prog;
}

static Shader MakeRaylibShaderFromProgram(GLuint programId)
{
  Shader sh{};
  if (programId == 0) return sh;

  sh.id = programId;

  // raylib has used two different representations for Shader::locs over time:
  //   - older: a fixed array embedded directly in the Shader struct
  //   - newer: an `int*` heap allocation owned/freed by raylib (UnloadShader)
  //
  // This helper must work with either, otherwise we risk a nullptr write and an
  // immediate access violation during init.
  if constexpr (std::is_pointer_v<std::remove_reference_t<decltype(sh.locs)>>) {
    if (sh.locs == nullptr) {
      sh.locs = static_cast<int*>(MemAlloc(RL_MAX_SHADER_LOCATIONS * sizeof(int)));
    }
    if (sh.locs == nullptr) {
      // Out of memory: avoid leaking the raw GL program.
      glDeleteProgram(programId);
      sh.id = 0;
      return sh;
    }
  }

  // Mark all built-in locations as invalid; then fill the ones we need.
  for (int i = 0; i < RL_MAX_SHADER_LOCATIONS; ++i) {
    sh.locs[i] = -1;
  }

  // Attribute locations used by rlgl batches.
  sh.locs[SHADER_LOC_VERTEX_POSITION] = GetShaderLocationAttrib(sh, "vertexPosition");
  sh.locs[SHADER_LOC_VERTEX_TEXCOORD01] = GetShaderLocationAttrib(sh, "vertexTexCoord");
  sh.locs[SHADER_LOC_VERTEX_COLOR] = GetShaderLocationAttrib(sh, "vertexColor");

  // raylib uses this to auto-set the camera MVP matrix for 2D/3D drawing.
  sh.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(sh, "mvp");

  return sh;
}

static std::array<float, 4> ColorToVec4(Color c, float alphaMul = 1.0f)
{
  const float a = (static_cast<float>(c.a) / 255.0f) * alphaMul;
  return {static_cast<float>(c.r) / 255.0f,
          static_cast<float>(c.g) / 255.0f,
          static_cast<float>(c.b) / 255.0f,
          a};
}

// Geometry-shader ribbon program.
//
// Vertex shader: transforms vertices into clip space and forwards the clip-space
// position into the geometry stage.
static const char* kRibbonVS = R"GLSL(
#version 330

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 2) in vec4 vertexColor;

uniform mat4 mvp;

out vec4 vColor;

void main()
{
    gl_Position = mvp*vec4(vertexPosition, 1.0);
    vColor = vertexColor;
}
)GLSL";

// Geometry shader: expands a GL_LINES primitive into a screen-space quad ribbon.
static const char* kRibbonGS = R"GLSL(
#version 330

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec2  u_screenSize;
uniform float u_thickness;

// Animation / pattern controls.
uniform float u_time;
uniform float u_dashLen;
uniform float u_dashSpeed;
uniform float u_dashDuty;
uniform float u_flowStrength;

in vec4 vColor[];

out vec4 fColor;
out vec2 fLocal;      // (along_px, across_px)
out float fDashSeed;

void emitVertex(vec2 ndcBase, vec2 ndcOffset, vec4 clipRef, float alongPx, float acrossPx)
{
    // Preserve the clip-space depth and W from the reference vertex so the
    // ribbon respects the camera's ordering.
    float w = clipRef.w;
    float z = clipRef.z;

    gl_Position = vec4((ndcBase + ndcOffset) * w, z, w);
    fLocal = vec2(alongPx, acrossPx);
    EmitVertex();
}

void main()
{
    vec4 p0c = gl_in[0].gl_Position;
    vec4 p1c = gl_in[1].gl_Position;

    // Convert to NDC and then to screen pixels for a thickness in px.
    vec2 ndc0 = p0c.xy / max(1e-6, p0c.w);
    vec2 ndc1 = p1c.xy / max(1e-6, p1c.w);

    vec2 s0 = (ndc0*0.5 + 0.5) * u_screenSize;
    vec2 s1 = (ndc1*0.5 + 0.5) * u_screenSize;

    vec2 d = s1 - s0;
    float len = length(d);
    if (len < 0.5) return;

    d = d / len;
    vec2 n = vec2(-d.y, d.x);

    float halfW = u_thickness * 0.5;
    vec2 offPx = n * halfW;

    // Convert pixel offset back to NDC.
    vec2 offNdc = (offPx / max(vec2(1.0), u_screenSize)) * 2.0;

    // Small per-segment seed so the dash doesn't "lock" to the exact same phase
    // everywhere (helps reduce moiré on long straight segments).
    fDashSeed = dot(s0, vec2(0.071, 0.113));

    // Use a single color per primitive.
    fColor = vColor[0];

    // Triangle strip: start(+), start(-), end(+), end(-)
    emitVertex(ndc0,  offNdc, p0c, 0.0, +halfW);
    emitVertex(ndc0, -offNdc, p0c, 0.0, -halfW);
    emitVertex(ndc1,  offNdc, p1c, len, +halfW);
    emitVertex(ndc1, -offNdc, p1c, len, -halfW);

    EndPrimitive();
}
)GLSL";

// Fragment shader: anti-alias edges, apply dash pattern, and add a subtle
// moving "flow" highlight along the centerline.
static const char* kRibbonFS = R"GLSL(
#version 330

in vec4 fColor;
in vec2 fLocal;
in float fDashSeed;

uniform float u_time;
uniform float u_thickness;
uniform float u_dashLen;
uniform float u_dashSpeed;
uniform float u_dashDuty;
uniform float u_flowStrength;

out vec4 finalColor;

void main()
{
    float halfW = max(0.5, u_thickness * 0.5);

    // Soft edge (roughly 1px feather).
    float edge = abs(fLocal.y);
    float edgeA = smoothstep(halfW, halfW - 1.25, edge);

    // Dash pattern in pixel space along the segment.
    float dashLen = max(2.0, u_dashLen);
    float phase = (fLocal.x + u_time*u_dashSpeed + fDashSeed) / dashLen;
    float f = fract(phase);

    // Duty cycle: on if f < duty, otherwise keep a faint residual so the
    // path still reads as continuous.
    float on = (f < u_dashDuty) ? 1.0 : 0.25;

    // A gentle moving highlight centered on the ribbon.
    float center = 1.0 - smoothstep(0.0, halfW, edge);
    float flow = 0.5 + 0.5*sin((fLocal.x * 0.12 - u_time * 2.4) * 6.2831853);
    float glow = 1.0 + u_flowStrength * center * flow;

    vec4 c = fColor;
    c.rgb *= glow;
    c.a *= edgeA * on;

    if (c.a <= 0.001) discard;
    finalColor = c;
}
)GLSL";

} // namespace

void GpuRibbonPathRenderer::init()
{
  shutdown();

  // Avoid touching OpenGL entry points if init is called before raylib has
  // created a graphics context.
  if (!IsWindowReady()) {
    m_ready = false;
    TraceLog(LOG_WARNING, "[GpuRibbon] init() called before window/context is ready (disabled).");
    return;
  }

  // Geometry shaders are core in OpenGL 3.2+. If the current context is older,
  // don't even try to compile the program.
  GLint major = 0;
  GLint minor = 0;
  glGetIntegerv(GL_MAJOR_VERSION, &major);
  glGetIntegerv(GL_MINOR_VERSION, &minor);
  if (major > 0 && (major < 3 || (major == 3 && minor < 2))) {
    m_ready = false;
    TraceLog(LOG_WARNING, "[GpuRibbon] OpenGL %d.%d < 3.2 (no geometry shader support).", major, minor);
    return;
  }


  // Attempt to compile the geometry-shader program. If it fails, we keep the
  // renderer disabled (safe fallback to CPU path highlights).
  std::string log;
  const GLuint prog = BuildProgramWithGeometry(kRibbonVS, kRibbonGS, kRibbonFS, log);
  if (prog == 0) {
    m_ready = false;
    if (!log.empty()) {
      TraceLog(LOG_WARNING, "[GpuRibbon] Geometry shader program failed to compile/link:\n%s", log.c_str());
    } else {
      TraceLog(LOG_WARNING, "[GpuRibbon] Geometry shader program failed to compile/link (no log).");
    }
    return;
  }

  m_shader = MakeRaylibShaderFromProgram(prog);

  // If we couldn't create a valid raylib Shader wrapper (e.g. Shader::locs
  // allocation failed), disable gracefully.
  if (m_shader.id == 0) {
    m_ready = false;
    TraceLog(LOG_WARNING, "[GpuRibbon] Failed to wrap GL program into raylib Shader (disabled).");
    return;
  }

  m_locScreenSize = GetShaderLocation(m_shader, "u_screenSize");
  m_locThickness = GetShaderLocation(m_shader, "u_thickness");
  m_locTime = GetShaderLocation(m_shader, "u_time");

  // The actual per-draw parameters.
  m_locDashLen = GetShaderLocation(m_shader, "u_dashLen");
  m_locDashSpeed = GetShaderLocation(m_shader, "u_dashSpeed");
  m_locDashDuty = GetShaderLocation(m_shader, "u_dashDuty");
  m_locFlowStrength = GetShaderLocation(m_shader, "u_flowStrength");

  bool ok = true;
  const auto requireLoc = [&](int loc, const char* name) {
    if (loc < 0) {
      TraceLog(LOG_ERROR, "[GpuRibbon] Missing uniform '%s' (shader disabled).", name);
      ok = false;
    }
  };

  // These uniforms are required for correct positioning and animation.
  requireLoc(m_locScreenSize, "u_screenSize");
  requireLoc(m_locThickness, "u_thickness");
  requireLoc(m_locTime, "u_time");

  if (!ok) {
    shutdown();
    return;
  }

  m_ready = true;
  TraceLog(LOG_INFO, "[GpuRibbon] Enabled (geometry shader path ribbons).");
}

void GpuRibbonPathRenderer::shutdown()
{
  if (m_shader.id != 0) {
    UnloadShader(m_shader);
    m_shader = Shader{};
  }
  m_ready = false;

  m_locScreenSize = -1;
  m_locThickness = -1;
  m_locTime = -1;
  m_locDashLen = -1;
  m_locDashSpeed = -1;
  m_locDashDuty = -1;
  m_locFlowStrength = -1;
}

void GpuRibbonPathRenderer::drawPath(const std::vector<Vector2>& points,
                                    int screenW, int screenH,
                                    float timeSec,
                                    Color baseColor,
                                    const RibbonStyle& style,
                                    bool additiveBlend) const
{
  if (!m_ready) return;
  if (points.size() < 2) return;
  if (screenW <= 0 || screenH <= 0) return;

  // Convert the desired RGBA to a vertex color. The shader uses fColor as a
  // per-primitive flat varying (copied from vertex 0).
  const auto core = ColorToVec4(baseColor, style.coreAlpha);
  const auto glow = ColorToVec4(baseColor, style.glowAlpha);

  // We draw two passes:
  //  1) glow: thicker, mostly solid
  //  2) core: thinner, dashed + animated
  const struct Pass {
    float thickness;
    std::array<float, 4> color;
    float dashDuty;
    float dashSpeed;
    float flowStrength;
  } passes[2] = {
      {style.glowThicknessPx, glow, 1.0f, 0.0f, 0.10f},
      {style.coreThicknessPx, core, style.dashDuty, style.dashSpeedPx, style.flowStrength},
  };

  const float screenSize[2] = {static_cast<float>(screenW), static_cast<float>(screenH)};

  BeginShaderMode(m_shader);

  // Static per-frame uniform.
  SetShaderValue(m_shader, m_locScreenSize, screenSize, SHADER_UNIFORM_VEC2);

  for (int passIndex = 0; passIndex < 2; ++passIndex) {
    const Pass& p = passes[passIndex];

    // Render the glow pass additively, and the core pass with normal alpha blending.
    if (additiveBlend && passIndex == 0) BeginBlendMode(BLEND_ADDITIVE);
    const float thickness = p.thickness;
    const float dashLen = style.dashLengthPx;
    const float dashSpeed = p.dashSpeed;
    const float dashDuty = p.dashDuty;
    const float flowStrength = p.flowStrength;

    SetShaderValue(m_shader, m_locThickness, &thickness, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_shader, m_locTime, &timeSec, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_shader, m_locDashLen, &dashLen, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_shader, m_locDashSpeed, &dashSpeed, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_shader, m_locDashDuty, &dashDuty, SHADER_UNIFORM_FLOAT);
    SetShaderValue(m_shader, m_locFlowStrength, &flowStrength, SHADER_UNIFORM_FLOAT);

    // Provide the flat varying color via per-vertex color.
    const Color vc = Color{
        static_cast<unsigned char>(std::clamp(p.color[0], 0.0f, 1.0f) * 255.0f),
        static_cast<unsigned char>(std::clamp(p.color[1], 0.0f, 1.0f) * 255.0f),
        static_cast<unsigned char>(std::clamp(p.color[2], 0.0f, 1.0f) * 255.0f),
        static_cast<unsigned char>(std::clamp(p.color[3], 0.0f, 1.0f) * 255.0f),
    };

    rlBegin(RL_LINES);
    rlColor4ub(vc.r, vc.g, vc.b, vc.a);

    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const Vector2 a = points[i];
      const Vector2 b = points[i + 1];
      rlVertex2f(a.x, a.y);
      rlVertex2f(b.x, b.y);
    }

    rlEnd();

    if (additiveBlend && passIndex == 0) EndBlendMode();
  }

  EndShaderMode();
}

} // namespace isocity
