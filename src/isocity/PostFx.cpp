#include "isocity/PostFx.hpp"

#include "isocity/ShaderUtil.hpp"

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

// Optional convenience uniforms (also used by the built-in FXAA/sharpen path).
uniform vec2 u_resolution;
uniform vec2 u_texelSize;

uniform float u_time;
uniform float u_seed;
uniform int u_bits;
uniform float u_dither;
uniform float u_grain;
uniform float u_vignette;
uniform float u_chroma;
uniform float u_scanlines;
uniform float u_fxaa;
uniform float u_sharpen;

float hash12(vec2 p)
{
    // Small, fast hash. Good enough for grain.
    // Based on a common sine-dot hash.
    float h = dot(p, vec2(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

float bayer4(vec2 pixel, float seed)
{
    // 4x4 Bayer matrix (normalized to [0,1)).
    // Layout:
    //  0  8  2 10
    // 12  4 14  6
    //  3 11  1  9
    // 15  7 13  5
    ivec2 p = ivec2(int(floor(pixel.x)) & 3, int(floor(pixel.y)) & 3);

    // Seeded permutation: shift and optionally transpose the 4x4 matrix.
    // This keeps the same distribution as Bayer dithering but avoids a
    // "one true" dither pattern across all worlds.
    int si = int(seed * 65535.0);
    int s = si & 15;
    int ox = s & 3;
    int oy = (s >> 2) & 3;
    p = ivec2((p.x + ox) & 3, (p.y + oy) & 3);
    if ((s & 8) != 0) {
        int tmp = p.x;
        p.x = p.y;
        p.y = tmp;
    }
    int idx = p.x + p.y * 4;
    int m[16] = int[16](0, 8, 2, 10,
                        12, 4, 14, 6,
                        3, 11, 1, 9,
                        15, 7, 13, 5);
    return (float(m[idx]) + 0.5) / 16.0;
}

float luma(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec3 fxaaFromNeighbors(vec2 uv, vec2 texel,
                       vec3 rgbM,
                       vec3 rgbNW, vec3 rgbNE,
                       vec3 rgbSW, vec3 rgbSE)
{
    // Fast Approximate Anti-Aliasing (FXAA) - simplified 3.11 style.
    // This is intentionally compact and tuned for a stylized pipeline.
    float lumaNW = luma(rgbNW);
    float lumaNE = luma(rgbNE);
    float lumaSW = luma(rgbSW);
    float lumaSE = luma(rgbSE);
    float lumaM  = luma(rgbM);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    float lumaRange = lumaMax - lumaMin;

    // Early out: nothing to smooth.
    if (lumaRange < 0.0312) return rgbM;

    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    // Tweak constants to keep the filter modest.
    const float FXAA_REDUCE_MIN = 1.0/128.0;
    const float FXAA_REDUCE_MUL = 1.0/8.0;
    const float FXAA_SPAN_MAX   = 8.0;

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);

    dir = clamp(dir * rcpDirMin, vec2(-FXAA_SPAN_MAX), vec2(FXAA_SPAN_MAX)) * texel;

    vec3 rgbA = 0.5 * (
        texture(texture0, uv + dir * (1.0/3.0 - 0.5)).rgb +
        texture(texture0, uv + dir * (2.0/3.0 - 0.5)).rgb);

    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(texture0, uv + dir * -0.5).rgb +
        texture(texture0, uv + dir *  0.5).rgb);

    float lumaB = luma(rgbB);
    if ((lumaB < lumaMin) || (lumaB > lumaMax)) return rgbA;

    return rgbB;
}

void main()
{
    vec2 uv = fragTexCoord;

    // Base sample.
    vec4 base = texture(texture0, uv);
    vec3 col = base.rgb;

    // FXAA + sharpening are applied before stylization (quantization/dither/grain)
    // to keep those effects crisp.
    if ((u_fxaa > 0.0001) || (u_sharpen > 0.0001))
    {
        vec2 t = u_texelSize;

        vec3 rgbNW = texture(texture0, uv + t * vec2(-1.0, -1.0)).rgb;
        vec3 rgbNE = texture(texture0, uv + t * vec2( 1.0, -1.0)).rgb;
        vec3 rgbSW = texture(texture0, uv + t * vec2(-1.0,  1.0)).rgb;
        vec3 rgbSE = texture(texture0, uv + t * vec2( 1.0,  1.0)).rgb;

        // FXAA: blend between original and filtered color.
        if (u_fxaa > 0.0001)
        {
            vec3 aa = fxaaFromNeighbors(uv, t, col, rgbNW, rgbNE, rgbSW, rgbSE);
            col = mix(col, aa, clamp(u_fxaa, 0.0, 1.0));
        }

        // Unsharp mask using a cheap diagonal blur kernel.
        if (u_sharpen > 0.0001)
        {
            vec3 blur = (rgbNW + rgbNE + rgbSW + rgbSE) * 0.25;
            float amt = clamp(u_sharpen, 0.0, 1.0);
            // Slight scale so small UI slider values have impact.
            col = clamp(col + (col - blur) * (amt * 1.25), 0.0, 1.0);
        }
    }

    // Optional chromatic aberration (screen-space radial RGB split).
    // Sample additional taps only if the effect is non-trivial.
    if (u_chroma > 0.0001)
    {
        vec2 c = uv - vec2(0.5);
        // Scale keeps the effect subtle even at 4K.
        vec2 off = c * (0.010 * u_chroma);
        vec3 split = vec3(texture(texture0, uv + off).r, col.g, texture(texture0, uv - off).b);
        // Blend rather than fully replacing channels, so FXAA still has some effect.
        col = mix(col, split, clamp(u_chroma, 0.0, 1.0));
    }

    // Apply raylib tinting.
    vec4 modulate = colDiffuse * fragColor;
    col *= modulate.rgb;
    float a = base.a * modulate.a;

    // Vignette (kept mild; centered on the screen).
    if (u_vignette > 0.0001)
    {
        vec2 d = uv - vec2(0.5);
        // Use actual aspect ratio when provided.
        float aspect = (u_resolution.y > 0.0) ? (u_resolution.x / u_resolution.y) : 1.0;
        d.x *= aspect;
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

    float b = bayer4(gl_FragCoord.xy, u_seed);
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

  const std::vector<std::string> defines = {
      "#define PROCISOCITY 1",
      "#define PROCISOCITY_POSTFX 1",
  };

  ShaderBuildResult r = LoadShaderProgramWithOverrides("postfx", kPostFxVS, kPostFxFS, defines);
  m_shader = r.shader;
  m_usedOverride = r.source.vsFromFile || r.source.fsFromFile;

  if (m_shader.id == 0) {
    m_failed = true;
    if (!r.log.empty()) {
      TraceLog(LOG_WARNING, "[PostFx] shader compile failed:\n%s", r.log.c_str());
    } else {
      TraceLog(LOG_WARNING, "[PostFx] shader compile failed (no log)");
    }
    return;
  }

  if (!r.log.empty()) {
    TraceLog(LOG_INFO, "[PostFx] shader log:\n%s", r.log.c_str());
  }

  m_locTime = GetShaderLocation(m_shader, "u_time");
  m_locSeed = GetShaderLocation(m_shader, "u_seed");
  m_locBits = GetShaderLocation(m_shader, "u_bits");
  m_locDither = GetShaderLocation(m_shader, "u_dither");
  m_locGrain = GetShaderLocation(m_shader, "u_grain");
  m_locVignette = GetShaderLocation(m_shader, "u_vignette");
  m_locChroma = GetShaderLocation(m_shader, "u_chroma");
  m_locScanlines = GetShaderLocation(m_shader, "u_scanlines");

  m_locFxaa = GetShaderLocation(m_shader, "u_fxaa");
  m_locSharpen = GetShaderLocation(m_shader, "u_sharpen");

  // Optional convenience uniforms for custom override shaders.
  m_locResolution = GetShaderLocation(m_shader, "u_resolution");
  m_locTexelSize = GetShaderLocation(m_shader, "u_texelSize");

  m_ready = true;
}

bool PostFxPipeline::reload()
{
  shutdown();
  m_failed = false;
  init();
  return m_ready;
}

void PostFxPipeline::shutdown()
{
  if (m_ready && m_shader.id != 0) {
    UnloadShader(m_shader);
  }
  m_shader = Shader{};
  m_ready = false;
  m_failed = false;
  m_usedOverride = false;

  m_locTime = -1;
  m_locSeed = -1;
  m_locBits = -1;
  m_locDither = -1;
  m_locGrain = -1;
  m_locVignette = -1;
  m_locChroma = -1;
  m_locScanlines = -1;

  m_locFxaa = -1;
  m_locSharpen = -1;

  m_locResolution = -1;
  m_locTexelSize = -1;
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

  const float fxaa = std::clamp(settings.fxaa, 0.0f, 1.0f);
  const float sharpen = std::clamp(settings.sharpen, 0.0f, 1.0f);

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

  if (m_locFxaa >= 0) SetShaderValue(m_shader, m_locFxaa, &fxaa, SHADER_UNIFORM_FLOAT);
  if (m_locSharpen >= 0) SetShaderValue(m_shader, m_locSharpen, &sharpen, SHADER_UNIFORM_FLOAT);

  if (m_locResolution >= 0) {
    const Vector2 res{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())};
    SetShaderValue(m_shader, m_locResolution, &res, SHADER_UNIFORM_VEC2);
  }
  if (m_locTexelSize >= 0) {
    const Vector2 texel{(tex.width > 0) ? (1.0f / static_cast<float>(tex.width)) : 0.0f,
                        (tex.height > 0) ? (1.0f / static_cast<float>(tex.height)) : 0.0f};
    SetShaderValue(m_shader, m_locTexelSize, &texel, SHADER_UNIFORM_VEC2);
  }

  DrawTexturePro(tex, src, dst, Vector2{0.0f, 0.0f}, 0.0f, tint);

  EndShaderMode();
}

} // namespace isocity
