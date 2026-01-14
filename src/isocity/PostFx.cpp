#include "isocity/PostFx.hpp"

#include <algorithm>
#include <cmath>

namespace isocity {
namespace {

// Raylib-compatible passthrough vertex shader (GLSL 330).
// Matches the built-in attributes/varyings used by raylib's default batch renderer.
const char* kPostFxVS = R"GLSL(
#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;

out vec2 fragTexCoord;
out vec4 fragColor;

uniform mat4 mvp;

void main()
{
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = mvp*vec4(vertexPosition, 1.0);
}
)GLSL";

// Stylized post-processing fragment shader.
//
// Effects (all optional):
//  - Per-channel quantization (u_bits)
//  - Ordered dithering (Bayer 4x4) to mask quantization banding (u_dither)
//  - Temporal grain (u_grain)
//  - Vignette (u_vignette)
//  - Radial chromatic aberration (u_chroma)
//  - Scanlines (u_scanlines)
const char* kPostFxFS = R"GLSL(
#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform float u_time;
uniform float u_seed;
uniform int u_bits;
uniform float u_dither;
uniform float u_grain;
uniform float u_vignette;
uniform float u_chroma;
uniform float u_scanlines;

float hash12(vec2 p)
{
    // Small, fast hash. Good enough for grain.
    // Based on a common sine-dot hash.
    float h = dot(p, vec2(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

float bayer4(vec2 pixel)
{
    // 4x4 Bayer matrix (normalized to [0,1)).
    // Layout:
    //  0  8  2 10
    // 12  4 14  6
    //  3 11  1  9
    // 15  7 13  5
    ivec2 p = ivec2(int(floor(pixel.x)) & 3, int(floor(pixel.y)) & 3);
    int idx = p.x + p.y * 4;
    int m[16] = int[16](0, 8, 2, 10,
                        12, 4, 14, 6,
                        3, 11, 1, 9,
                        15, 7, 13, 5);
    return (float(m[idx]) + 0.5) / 16.0;
}

void main()
{
    vec2 uv = fragTexCoord;

    // Base sample.
    vec4 base = texture(texture0, uv);

    // Optional chromatic aberration (screen-space radial RGB split).
    // Sample additional taps only if the effect is non-trivial.
    vec3 col = base.rgb;
    if (u_chroma > 0.0001)
    {
        vec2 c = uv - vec2(0.5);
        // Scale keeps the effect subtle even at 4K.
        vec2 off = c * (0.010 * u_chroma);
        float r = texture(texture0, uv + off).r;
        float g = base.g;
        float b = texture(texture0, uv - off).b;
        col = vec3(r, g, b);
    }

    // Apply raylib tinting.
    vec4 modulate = colDiffuse * fragColor;
    col *= modulate.rgb;
    float a = base.a * modulate.a;

    // Vignette (kept mild; centered on the screen).
    if (u_vignette > 0.0001)
    {
        vec2 d = uv - vec2(0.5);
        // Slight aspect compensation so it feels rounder on wide screens.
        d.x *= 1.1;
        float dist = length(d);
        float v = smoothstep(0.35, 0.85, dist);
        col *= (1.0 - u_vignette * 0.55 * v);
    }

    // Temporal grain: stable per-pixel, animated over time.
    if (u_grain > 0.0001)
    {
        float n = hash12(gl_FragCoord.xy + vec2(u_seed * 531.0, u_time * 60.0));
        // Small amplitude; grain should be felt more than seen.
        col += (n - 0.5) * (0.08 * u_grain);
    }

    // Scanlines (very subtle). Uses screen-space Y so it stays stable during camera motion.
    if (u_scanlines > 0.0001)
    {
        float s = sin(gl_FragCoord.y * 3.14159265);
        col *= (1.0 - u_scanlines * 0.06 * (0.5 + 0.5 * s));
    }

    // Dithered quantization.
    // Quantization is done in sRGB space (good enough for stylization).
    int bits = clamp(u_bits, 2, 8);
    float levels = pow(2.0, float(bits)) - 1.0;

    float b = bayer4(gl_FragCoord.xy);
    float d = (b - 0.5) * u_dither;

    col = floor(col * levels + d + 0.5) / levels;
    col = clamp(col, 0.0, 1.0);

    finalColor = vec4(col, a);
}
)GLSL";

} // namespace

PostFxPipeline::~PostFxPipeline()
{
  shutdown();
}

void PostFxPipeline::init()
{
  if (m_ready || m_failed) return;

  m_shader = LoadShaderFromMemory(kPostFxVS, kPostFxFS);
  if (m_shader.id == 0) {
    m_failed = true;
    return;
  }

  m_locTime = GetShaderLocation(m_shader, "u_time");
  m_locSeed = GetShaderLocation(m_shader, "u_seed");
  m_locBits = GetShaderLocation(m_shader, "u_bits");
  m_locDither = GetShaderLocation(m_shader, "u_dither");
  m_locGrain = GetShaderLocation(m_shader, "u_grain");
  m_locVignette = GetShaderLocation(m_shader, "u_vignette");
  m_locChroma = GetShaderLocation(m_shader, "u_chroma");
  m_locScanlines = GetShaderLocation(m_shader, "u_scanlines");

  m_ready = true;
}

void PostFxPipeline::shutdown()
{
  if (m_ready && m_shader.id != 0) {
    UnloadShader(m_shader);
  }
  m_shader = Shader{};
  m_ready = false;
  m_failed = false;

  m_locTime = -1;
  m_locSeed = -1;
  m_locBits = -1;
  m_locDither = -1;
  m_locGrain = -1;
  m_locVignette = -1;
  m_locChroma = -1;
  m_locScanlines = -1;
}

void PostFxPipeline::drawTexturePro(const Texture2D& tex, const Rectangle& src, const Rectangle& dst,
                                   const PostFxSettings& settings, float timeSec, std::uint32_t seed,
                                   Color tint)
{
  if (!settings.enabled || !m_ready) {
    DrawTexturePro(tex, src, dst, Vector2{0.0f, 0.0f}, 0.0f, tint);
    return;
  }

  const int bits = std::clamp(settings.colorBits, 2, 8);
  const float dither = std::clamp(settings.ditherStrength, 0.0f, 1.0f);
  const float grain = std::clamp(settings.grain, 0.0f, 1.0f);
  const float vignette = std::clamp(settings.vignette, 0.0f, 1.0f);
  const float chroma = std::clamp(settings.chroma, 0.0f, 1.0f);
  const float scan = std::clamp(settings.scanlines, 0.0f, 1.0f);

  // Map seed to [0,1) as a float. This keeps the shader portable (no 64-bit ints).
  const float seedF = static_cast<float>(static_cast<double>(seed) / 4294967296.0);

  BeginShaderMode(m_shader);

  if (m_locTime >= 0) SetShaderValue(m_shader, m_locTime, &timeSec, SHADER_UNIFORM_FLOAT);
  if (m_locSeed >= 0) SetShaderValue(m_shader, m_locSeed, &seedF, SHADER_UNIFORM_FLOAT);
  if (m_locBits >= 0) SetShaderValue(m_shader, m_locBits, &bits, SHADER_UNIFORM_INT);
  if (m_locDither >= 0) SetShaderValue(m_shader, m_locDither, &dither, SHADER_UNIFORM_FLOAT);
  if (m_locGrain >= 0) SetShaderValue(m_shader, m_locGrain, &grain, SHADER_UNIFORM_FLOAT);
  if (m_locVignette >= 0) SetShaderValue(m_shader, m_locVignette, &vignette, SHADER_UNIFORM_FLOAT);
  if (m_locChroma >= 0) SetShaderValue(m_shader, m_locChroma, &chroma, SHADER_UNIFORM_FLOAT);
  if (m_locScanlines >= 0) SetShaderValue(m_shader, m_locScanlines, &scan, SHADER_UNIFORM_FLOAT);

  DrawTexturePro(tex, src, dst, Vector2{0.0f, 0.0f}, 0.0f, tint);

  EndShaderMode();
}

} // namespace isocity
