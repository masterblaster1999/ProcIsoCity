#include "isocity/PostFx.hpp"

#include "isocity/ShaderUtil.hpp"

#include <algorithm>
#include <utility>
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
//  - Ordered dithering (Bayer 8x8) to mask quantization banding (u_dither)
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

// Filmic tonemap + grade.
uniform float u_tonemapEnabled;
uniform float u_exposure;
uniform float u_contrast;
uniform float u_saturation;

// Screen-space outlines.
uniform float u_outline;
uniform float u_outlineThreshold;
uniform float u_outlineThickness;

// Weather + lens precipitation (optional).
uniform int u_weatherMode;
uniform float u_weatherIntensity;
uniform vec2 u_windDir;
uniform float u_windSpeed;

uniform float u_lensWeather;
uniform float u_lensDistort;
uniform float u_lensScale;
uniform float u_lensDrips;

float hash12(vec2 p)
{
    // Small, fast hash. Good enough for grain.
    // Based on a common sine-dot hash.
    float h = dot(p, vec2(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

vec2 hash22(vec2 p)
{
    // Two uncorrelated hashes derived from hash12.
    return vec2(hash12(p), hash12(p + vec2(5.2, 1.3)));
}

vec2 safeNorm(vec2 v)
{
    float l2 = dot(v, v);
    if (l2 > 1.0e-6) return v * inversesqrt(l2);
    return vec2(0.0, 1.0);
}

vec4 lensRainLayer(vec2 p, float cellSize, float radius, float trailLen,
                   float speed, float seed, float coverage, float drips, vec2 dir)
{
    vec2 q = p + dir * (u_time * speed);

    vec2 g = floor(q / cellSize);

    float wsum = 0.0;
    vec2 offSum = vec2(0.0);
    float hiSum = 0.0;

    // Small neighborhood search so droplets can cross cell boundaries.
    for (int y = -1; y <= 1; ++y)
    {
        for (int x = -1; x <= 1; ++x)
        {
            vec2 cell = g + vec2(float(x), float(y));
            vec2 rnd = hash22(cell + vec2(seed * 17.0, seed * 29.0));
            float spawn = step(rnd.x, coverage);

            vec2 center = (cell + rnd) * cellSize;
            vec2 d = q - center;

            float r = radius * mix(0.70, 1.25, rnd.y);
            vec2 de = d * vec2(1.0, 1.20);
            float dist = length(de);

            float droplet = spawn * (1.0 - smoothstep(r, r + 1.5, dist));
            float rim = spawn * smoothstep(r * 0.60, r, dist) * (1.0 - smoothstep(r, r * 1.35, dist));

            // Drip trail below the droplet (thin, vertically stretched).
            float yv = d.y;
            float trail = spawn * smoothstep(-r, 0.0, yv) * (1.0 - smoothstep(0.0, trailLen, yv));
            float w = mix(r * 0.20, r * 0.45, rnd.x);
            trail *= (1.0 - smoothstep(w, w + 1.0, abs(d.x)));
            trail *= drips;

            float m = max(droplet, trail * 0.65);

            vec2 n = safeNorm(d);
            vec2 off = n * rim * (r * 0.45) + vec2(0.0, 1.0) * trail * (r * 0.18);

            wsum += m;
            offSum += off * m;
            hiSum += (rim * 0.80 + droplet * 0.20 + trail * 0.15) * m;
        }
    }

    if (wsum > 0.0) offSum /= wsum;
    float mask = clamp(wsum, 0.0, 1.0);
    float hi = clamp(hiSum, 0.0, 1.0);
    return vec4(offSum, mask, hi);
}

float bayer8(vec2 pixel, float seed)
{
    // 8x8 Bayer matrix (normalized to [0,1)). Generated procedurally.
    // We use the classic recursive Bayer construction with the 2x2 base:
    //  0 2
    //  3 1
    // and expand it to 8x8.
    ivec2 p = ivec2(int(floor(pixel.x)) & 7, int(floor(pixel.y)) & 7);

    // Seeded permutation: shift + transpose + flips.
    // This keeps the same distribution as Bayer dithering but avoids a
    // "one true" dither pattern across all worlds.
    //
    // Note: u_seed is a float in [0,1). We hash it into a stable 24-bit integer.
    int si = int(floor(hash12(vec2(seed * 173.3, seed * 941.7)) * 16777216.0));
    int ox = si & 7;
    int oy = (si >> 3) & 7;
    int flags = (si >> 6) & 7;

    p = ivec2((p.x + ox) & 7, (p.y + oy) & 7);
    if ((flags & 1) != 0) {
        int tmp = p.x;
        p.x = p.y;
        p.y = tmp;
    }
    if ((flags & 2) != 0) p.x = 7 - p.x;
    if ((flags & 4) != 0) p.y = 7 - p.y;

    int x = p.x;
    int y = p.y;

    // Compute Bayer index in [0..63].
    // Each bit-plane contributes a 2-bit quadrant code.
    int v = 0;
    for (int bit = 0; bit < 3; ++bit) {
        int bx = (x >> bit) & 1;
        int by = (y >> bit) & 1;
        int q = ((bx ^ by) << 1) | by; // 0,2,3,1
        v = v * 4 + q;
    }

    return (float(v) + 0.5) / 64.0;
}

float luma(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec3 srgbToLinear(vec3 c)
{
    // Cheap gamma decode. Good enough for a stylized pipeline.
    return pow(max(c, vec3(0.0)), vec3(2.2));
}

vec3 linearToSrgb(vec3 c)
{
    return pow(max(c, vec3(0.0)), vec3(1.0/2.2));
}

vec3 tonemapAcesFitted(vec3 x)
{
    // ACES filmic curve (fitted) popularized by Krzysztof Narkowicz.
    x = max(x, vec3(0.0));
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0);
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

    float lensHi = 0.0;

    // Lens precipitation: raindrops/drips on the camera lens.
    // Driven by weather uniforms and user settings.
    if ((u_lensWeather > 0.0001) && (u_weatherMode == 1) && (u_weatherIntensity > 0.0001))
    {
        float lensAmt = clamp(u_lensWeather, 0.0, 1.0) * clamp(u_weatherIntensity, 0.0, 1.0);
        vec2 res = max(u_resolution, vec2(1.0));
        vec2 p = uv * res;

        vec2 wdir = safeNorm(u_windDir);
        if (wdir.y < 0.15) wdir.y = 0.15;
        wdir = safeNorm(wdir);

        // Bias droplets to fall mostly downward, with wind drift.
        vec2 dir = safeNorm(vec2(wdir.x * 0.75, wdir.y * 0.25 + 1.0));

        float ws = clamp(u_windSpeed, 0.05, 6.0);
        float wsN = clamp((ws - 0.05) / 2.0, 0.0, 1.0);
        float speedMul = mix(0.75, 1.35, wsN);

        float scale = clamp(u_lensScale, 0.5, 2.0);
        float cov = clamp(mix(0.08, 0.72, lensAmt), 0.0, 1.0);
        float drips = clamp(u_lensDrips, 0.0, 1.0);

        vec4 l0 = lensRainLayer(p, 140.0 * scale, 16.0 * scale, 220.0 * scale,
                                50.0 * speedMul, u_seed + 0.17, cov, drips, dir);
        vec4 l1 = lensRainLayer(p, 75.0 * scale, 8.0 * scale, 140.0 * scale,
                                95.0 * speedMul, u_seed + 1.91, cov * 1.15, drips * 0.80, dir);

        float w0 = l0.z;
        float w1 = l1.z;
        float w = max(w0 + w1, 1.0e-4);
        vec2 offPx = (l0.xy * w0 + l1.xy * w1) / w;

        vec2 uvOff = (offPx / res) * (clamp(u_lensDistort, 0.0, 1.0) * 0.65) * lensAmt;
        uv = clamp(uv + uvOff, vec2(0.0), vec2(1.0));

        lensHi = clamp((l0.w + l1.w) * 0.10 * lensAmt, 0.0, 0.20);
    }

    // Base sample.
    vec4 base = texture(texture0, uv);
    vec3 col = clamp(base.rgb + lensHi, 0.0, 1.0);

    // Outline factor (computed from the neighborhood, applied later).
    float edge = 0.0;

    // FXAA + sharpening + outlines share neighborhood taps.
    if ((u_fxaa > 0.0001) || (u_sharpen > 0.0001) || (u_outline > 0.0001))
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

        // Outline edge factor from luma discontinuity.
        if (u_outline > 0.0001)
        {
            float thr = clamp(u_outlineThreshold, 0.0, 1.0);
            float rad = max(u_outlineThickness, 0.5);
            vec2 to = u_texelSize * rad;

            float lM = luma(col);
            float lN = luma(texture(texture0, uv + vec2(0.0, -to.y)).rgb);
            float lS = luma(texture(texture0, uv + vec2(0.0,  to.y)).rgb);
            float lE = luma(texture(texture0, uv + vec2( to.x, 0.0)).rgb);
            float lW = luma(texture(texture0, uv + vec2(-to.x, 0.0)).rgb);

            float md = max(max(abs(lM - lN), abs(lM - lS)), max(abs(lM - lE), abs(lM - lW)));
            const float soft = 0.08;
            edge = smoothstep(thr, thr + soft, md);
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

    // Filmic tonemap + grade (optional).
    if (u_tonemapEnabled > 0.5)
    {
        float exposure = max(0.0, u_exposure);
        float contrast = max(0.0, u_contrast);
        float sat = max(0.0, u_saturation);

        vec3 lin = srgbToLinear(clamp(col, 0.0, 1.0)) * exposure;
        lin = tonemapAcesFitted(lin);
        col = linearToSrgb(lin);

        // Contrast around 0.5.
        col = clamp((col - 0.5) * contrast + 0.5, 0.0, 1.0);

        // Saturation.
        float l = dot(col, vec3(0.2126, 0.7152, 0.0722));
        col = clamp(vec3(l) + (col - vec3(l)) * sat, 0.0, 1.0);
    }

    // Apply outline darkening after tonemap.
    if (u_outline > 0.0001)
    {
        float st = clamp(u_outline, 0.0, 1.0);
        col = mix(col, vec3(0.0), edge * st);
    }

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

    float b = bayer8(gl_FragCoord.xy, u_seed);
    float d = (b - 0.5) * u_dither;

    col = floor(col * levels + d + 0.5) / levels;
    col = clamp(col, 0.0, 1.0);

    finalColor = vec4(col, a);
}
)GLSL";

// ------------------------------------------------------------
// Bloom shaders (bright-pass extraction + separable blur)
// ------------------------------------------------------------

// Bloom extraction: keep only pixels above a soft threshold.
//
// Inputs:
//  - u_threshold: brightness threshold
//  - u_knee: soft threshold knee (already scaled, in brightness units)
const char* kBloomExtractFS = R"GLSL(
#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Optional convenience uniforms (for override shaders).
uniform vec2 u_resolution;
uniform vec2 u_texelSize;

uniform float u_threshold;
uniform float u_knee;

float max3(vec3 v) { return max(v.r, max(v.g, v.b)); }

void main()
{
    vec4 base = texture(texture0, fragTexCoord);
    vec3 col = base.rgb;

    // Apply raylib tinting.
    vec4 modulate = colDiffuse * fragColor;
    col *= modulate.rgb;

    float b = max3(col);
    float t = clamp(u_threshold, 0.0, 1.0);
    float knee = max(u_knee, 0.0);

    // Soft threshold based on the common "knee" bloom function.
    float soft = b - t;
    soft = clamp(soft + knee, 0.0, 2.0 * knee);
    soft = (knee > 0.0) ? (soft*soft) / (4.0*knee + 1e-5) : 0.0;

    float contrib = max(soft, b - t);
    contrib = (b > 1e-5) ? contrib / b : 0.0;

    vec3 outCol = col * max(contrib, 0.0);
    finalColor = vec4(outCol, 1.0);
}
)GLSL";

// Bloom blur: separable gaussian-ish blur.
//
// Inputs:
//  - u_texelSize: 1.0 / source texture size
//  - u_direction: (1,0) for horizontal, (0,1) for vertical
//  - u_radius: blur radius multiplier
const char* kBloomBlurFS = R"GLSL(
#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform vec2 u_texelSize;
uniform vec2 u_direction;
uniform float u_radius;

void main()
{
    vec2 dir = u_direction;
    float r = max(u_radius, 0.0);
    vec2 stepUV = u_texelSize * r;
    vec2 o1 = dir * stepUV * 1.0;
    vec2 o2 = dir * stepUV * 2.0;
    vec2 o3 = dir * stepUV * 3.0;
    vec2 o4 = dir * stepUV * 4.0;

    // 9-tap weights.
    const float w0 = 0.2270270270;
    const float w1 = 0.1945945946;
    const float w2 = 0.1216216216;
    const float w3 = 0.0540540541;
    const float w4 = 0.0162162162;

    vec3 sum = texture(texture0, fragTexCoord).rgb * w0;
    sum += texture(texture0, fragTexCoord + o1).rgb * w1;
    sum += texture(texture0, fragTexCoord - o1).rgb * w1;
    sum += texture(texture0, fragTexCoord + o2).rgb * w2;
    sum += texture(texture0, fragTexCoord - o2).rgb * w2;
    sum += texture(texture0, fragTexCoord + o3).rgb * w3;
    sum += texture(texture0, fragTexCoord - o3).rgb * w3;
    sum += texture(texture0, fragTexCoord + o4).rgb * w4;
    sum += texture(texture0, fragTexCoord - o4).rgb * w4;

    // Apply raylib tinting (normally WHITE).
    vec4 modulate = colDiffuse * fragColor;
    sum *= modulate.rgb;

    finalColor = vec4(sum, 1.0);
}
)GLSL";


// Temporal AA resolve shader (TAA-lite).
//
// This stage is designed to reduce shimmering on thin geometry without motion vectors:
//  - Camera jitter is applied during world rendering (subpixel sampling).
//  - Here we cancel the jitter, then blend against a history buffer.
//  - A small neighborhood clamp + luminance-based responsiveness helps reduce ghosting.
const char* kTaaFS = R"GLSL(
#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;   // current frame (jittered)
uniform sampler2D u_history;  // previous resolved frame (stable)
uniform vec4 colDiffuse;

uniform vec2 u_texelSize;      // 1/texture0 size
uniform vec2 u_jitterUV;       // UV offset used to cancel the camera jitter
uniform float u_historyWeight; // 0..1 (higher = more stable)
uniform float u_response;      // 0..1 (higher = less ghosting)
uniform int u_reset;           // 1 => ignore history this frame

float luma(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec2 clampUv(vec2 uv)
{
    return clamp(uv, vec2(0.0), vec2(1.0));
}

// Manual bilinear sample of texture0.
// This keeps TAA effective even if the underlying texture filter is set to point.
vec4 sample0Bilinear(vec2 uv)
{
    vec2 texSize = 1.0 / max(u_texelSize, vec2(1e-6));

    // Convert to texel space.
    vec2 pos = uv * texSize - vec2(0.5);
    vec2 i = floor(pos);
    vec2 f = fract(pos);

    // Clamp integer base so we don't sample outside.
    vec2 i0 = clamp(i, vec2(0.0), texSize - vec2(2.0));

    vec2 uv00 = (i0 + vec2(0.5, 0.5)) / texSize;
    vec2 uv10 = (i0 + vec2(1.5, 0.5)) / texSize;
    vec2 uv01 = (i0 + vec2(0.5, 1.5)) / texSize;
    vec2 uv11 = (i0 + vec2(1.5, 1.5)) / texSize;

    vec4 c00 = texture(texture0, clampUv(uv00));
    vec4 c10 = texture(texture0, clampUv(uv10));
    vec4 c01 = texture(texture0, clampUv(uv01));
    vec4 c11 = texture(texture0, clampUv(uv11));

    vec4 cx0 = mix(c00, c10, f.x);
    vec4 cx1 = mix(c01, c11, f.x);
    return mix(cx0, cx1, f.y);
}

void main()
{
    vec2 uv = fragTexCoord;

    // Sample the current frame at an offset that cancels the camera jitter.
    vec2 cuv = clampUv(uv + u_jitterUV);

    // Preserve raylib tinting semantics.
    vec4 curS = sample0Bilinear(cuv) * colDiffuse * fragColor;
    vec3 cur = curS.rgb;

    // First frame after reset: just seed the history.
    if (u_reset != 0) {
        finalColor = curS;
        return;
    }

    vec3 hist = texture(u_history, uv).rgb;

    // --- Neighborhood clamp (TAA-lite)
    // Use a 5-tap min/max to avoid severe ghosting without motion vectors.
    vec3 cN = sample0Bilinear(clampUv(cuv + vec2(0.0, -u_texelSize.y))).rgb;
    vec3 cS = sample0Bilinear(clampUv(cuv + vec2(0.0,  u_texelSize.y))).rgb;
    vec3 cE = sample0Bilinear(clampUv(cuv + vec2( u_texelSize.x, 0.0))).rgb;
    vec3 cW = sample0Bilinear(clampUv(cuv + vec2(-u_texelSize.x, 0.0))).rgb;

    vec3 mn = min(cur, min(min(cN, cS), min(cE, cW)));
    vec3 mx = max(cur, max(max(cN, cS), max(cE, cW)));

    vec3 histClamped = clamp(hist, mn, mx);

    // Responsiveness term: if the current frame disagrees with history, reduce
    // the history weight to avoid trails.
    float diff = abs(luma(cur) - luma(histClamped));
    float resp = mix(0.0, 8.0, clamp(u_response, 0.0, 1.0));

    float w = clamp(u_historyWeight, 0.0, 1.0);
    w *= clamp(1.0 - diff * resp, 0.0, 1.0);
    w = min(w, 0.98);

    vec3 outRgb = mix(cur, histClamped, w);
    finalColor = vec4(outRgb, curS.a);
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
      "#define PROCISOCITY_BLOOM 1",
  };

  // Main PostFX shader.
  {
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
    } else {
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

      m_locTonemapEnabled = GetShaderLocation(m_shader, "u_tonemapEnabled");
      m_locExposure = GetShaderLocation(m_shader, "u_exposure");
      m_locContrast = GetShaderLocation(m_shader, "u_contrast");
      m_locSaturation = GetShaderLocation(m_shader, "u_saturation");

      m_locOutline = GetShaderLocation(m_shader, "u_outline");
      m_locOutlineThreshold = GetShaderLocation(m_shader, "u_outlineThreshold");
      m_locOutlineThickness = GetShaderLocation(m_shader, "u_outlineThickness");

      // Lens precipitation (optional).
      m_locLensWeather = GetShaderLocation(m_shader, "u_lensWeather");
      m_locLensDistort = GetShaderLocation(m_shader, "u_lensDistort");
      m_locLensScale = GetShaderLocation(m_shader, "u_lensScale");
      m_locLensDrips = GetShaderLocation(m_shader, "u_lensDrips");

      // Weather uniforms (optional; used by lens precipitation).
      m_locWeatherMode = GetShaderLocation(m_shader, "u_weatherMode");
      m_locWeatherIntensity = GetShaderLocation(m_shader, "u_weatherIntensity");
      m_locWindDir = GetShaderLocation(m_shader, "u_windDir");
      m_locWindSpeed = GetShaderLocation(m_shader, "u_windSpeed");

      // Optional convenience uniforms for custom override shaders.
      m_locResolution = GetShaderLocation(m_shader, "u_resolution");
      m_locTexelSize = GetShaderLocation(m_shader, "u_texelSize");

      m_ready = true;
    }
  }



  // Temporal AA resolve shader (optional). Compile it even if PostFX failed so
  // the user can still use TAA in the shader fallback mode.
  {
    ShaderBuildResult t = LoadShaderProgramWithOverrides("taa", kPostFxVS, kTaaFS, defines);
    m_taa = t.shader;
    m_taaUsedOverride = t.source.vsFromFile || t.source.fsFromFile;

    if (m_taa.id == 0) {
      m_taaFailed = true;
      if (!t.log.empty()) {
        TraceLog(LOG_WARNING, "[TAA] shader compile failed:\n%s", t.log.c_str());
      } else {
        TraceLog(LOG_WARNING, "[TAA] shader compile failed (no log)");
      }
    } else {
      if (!t.log.empty()) {
        TraceLog(LOG_INFO, "[TAA] shader log:\n%s", t.log.c_str());
      }

      m_locTaaHistory = GetShaderLocation(m_taa, "u_history");
      m_locTaaTexelSize = GetShaderLocation(m_taa, "u_texelSize");
      m_locTaaJitterUV = GetShaderLocation(m_taa, "u_jitterUV");
      m_locTaaHistoryWeight = GetShaderLocation(m_taa, "u_historyWeight");
      m_locTaaResponse = GetShaderLocation(m_taa, "u_response");
      m_locTaaReset = GetShaderLocation(m_taa, "u_reset");

      m_taaReady = true;
    }
  }

  // Bloom shaders (optional). Compile them even if PostFX failed so the user
  // can still get glow in the "shader fallback" mode.
  {
    ShaderBuildResult ex = LoadShaderProgramWithOverrides("bloom_extract", kPostFxVS, kBloomExtractFS, defines);
    ShaderBuildResult bl = LoadShaderProgramWithOverrides("bloom_blur", kPostFxVS, kBloomBlurFS, defines);

    m_bloomExtract = ex.shader;
    m_bloomBlur = bl.shader;
    m_bloomUsedOverride = ex.source.vsFromFile || ex.source.fsFromFile || bl.source.vsFromFile || bl.source.fsFromFile;

    if (m_bloomExtract.id == 0 || m_bloomBlur.id == 0) {
      m_bloomFailed = true;
      if (!ex.log.empty()) TraceLog(LOG_WARNING, "[Bloom] extract shader log:\n%s", ex.log.c_str());
      if (!bl.log.empty()) TraceLog(LOG_WARNING, "[Bloom] blur shader log:\n%s", bl.log.c_str());

      if (m_bloomExtract.id != 0) UnloadShader(m_bloomExtract);
      if (m_bloomBlur.id != 0) UnloadShader(m_bloomBlur);
      m_bloomExtract = Shader{};
      m_bloomBlur = Shader{};
      m_bloomReady = false;
    } else {
      if (!ex.log.empty()) TraceLog(LOG_INFO, "[Bloom] extract shader log:\n%s", ex.log.c_str());
      if (!bl.log.empty()) TraceLog(LOG_INFO, "[Bloom] blur shader log:\n%s", bl.log.c_str());

      // Extract uniforms.
      m_locBloomThreshold = GetShaderLocation(m_bloomExtract, "u_threshold");
      m_locBloomKnee = GetShaderLocation(m_bloomExtract, "u_knee");
      m_locBloomExtractResolution = GetShaderLocation(m_bloomExtract, "u_resolution");
      m_locBloomExtractTexelSize = GetShaderLocation(m_bloomExtract, "u_texelSize");

      // Blur uniforms.
      m_locBloomBlurTexelSize = GetShaderLocation(m_bloomBlur, "u_texelSize");
      m_locBloomBlurDirection = GetShaderLocation(m_bloomBlur, "u_direction");
      m_locBloomBlurRadius = GetShaderLocation(m_bloomBlur, "u_radius");

      m_bloomReady = true;
    }
  }
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

  m_locTonemapEnabled = -1;
  m_locExposure = -1;
  m_locContrast = -1;
  m_locSaturation = -1;

  m_locOutline = -1;
  m_locOutlineThreshold = -1;
  m_locOutlineThickness = -1;

  m_locLensWeather = -1;
  m_locLensDistort = -1;
  m_locLensScale = -1;
  m_locLensDrips = -1;

  m_locWeatherMode = -1;
  m_locWeatherIntensity = -1;
  m_locWindDir = -1;
  m_locWindSpeed = -1;

  m_locResolution = -1;
  m_locTexelSize = -1;



  // TAA resources.
  if (m_taaReady || m_taaFailed) {
    if (m_taa.id != 0) UnloadShader(m_taa);
  }
  m_taa = Shader{};
  m_taaReady = false;
  m_taaFailed = false;
  m_taaUsedOverride = false;

  m_locTaaHistory = -1;
  m_locTaaTexelSize = -1;
  m_locTaaJitterUV = -1;
  m_locTaaHistoryWeight = -1;
  m_locTaaResponse = -1;
  m_locTaaReset = -1;

  if (m_taaRTValid) {
    UnloadRenderTexture(m_taaRT0);
    UnloadRenderTexture(m_taaRT1);
  }
  m_taaRT0 = RenderTexture2D{};
  m_taaRT1 = RenderTexture2D{};
  m_taaRTValid = false;
  m_taaRTAllocFailed = false;
  m_taaHistoryValid = false;
  m_taaRTWidth = 0;
  m_taaRTHeight = 0;

  // Bloom resources.
  if (m_bloomReady || m_bloomFailed) {
    if (m_bloomExtract.id != 0) UnloadShader(m_bloomExtract);
    if (m_bloomBlur.id != 0) UnloadShader(m_bloomBlur);
  }
  m_bloomExtract = Shader{};
  m_bloomBlur = Shader{};
  m_bloomReady = false;
  m_bloomFailed = false;
  m_bloomUsedOverride = false;

  m_locBloomThreshold = -1;
  m_locBloomKnee = -1;
  m_locBloomExtractResolution = -1;
  m_locBloomExtractTexelSize = -1;

  m_locBloomBlurTexelSize = -1;
  m_locBloomBlurDirection = -1;
  m_locBloomBlurRadius = -1;

  if (m_bloomRTValid) {
    UnloadRenderTexture(m_bloomRT0);
    UnloadRenderTexture(m_bloomRT1);
  }
  m_bloomRT0 = RenderTexture2D{};
  m_bloomRT1 = RenderTexture2D{};
  m_bloomRTValid = false;
  m_bloomRTAllocFailed = false;
  m_bloomRTWidth = 0;
  m_bloomRTHeight = 0;
}

void PostFxPipeline::drawTexturePro(const Texture2D& tex, const Rectangle& src, const Rectangle& dst,
                                   const PostFxSettings& settings, float timeSec, std::uint32_t seed,
                                   Color tint, Vector2 taaJitterPixels, bool taaResetHistory,
                                   int weatherMode, float weatherIntensity,
                                   Vector2 windDir, float windSpeed)
{
  // If PostFX is disabled, act like a pass-through renderer.
  if (!settings.enabled) {
    // Avoid resuming an old TAA history when the user toggles PostFX back on.
    m_taaHistoryValid = false;
    DrawTexturePro(tex, src, dst, Vector2{0.0f, 0.0f}, 0.0f, tint);
    return;
  }



  // ---------------------------------------------------------------------------
  // Temporal AA resolve (optional)
  // ---------------------------------------------------------------------------
  const Texture2D* baseTex = &tex;
  Rectangle baseSrc = src;

  const bool wantTaa = settings.taaEnabled && m_taaReady;
  if (!wantTaa) {
    // Ensure the next enable starts clean.
    m_taaHistoryValid = false;
  }

  if (wantTaa) {
    const int tw = std::max(1, static_cast<int>(std::abs(src.width)));
    const int th = std::max(1, static_cast<int>(std::abs(src.height)));

    const bool sizeChanged = (m_taaRTWidth != tw) || (m_taaRTHeight != th);
    if (sizeChanged) {
      // New resolution: allow another allocation attempt even if a previous one failed.
      m_taaRTAllocFailed = false;
      m_taaHistoryValid = false;
    }

    // Avoid hammering the GPU with allocation attempts every frame if we already
    // failed at this resolution.
    const bool shouldTryAlloc = !m_taaRTValid && !m_taaRTAllocFailed;

    if (sizeChanged || shouldTryAlloc) {
      if (m_taaRT0.id != 0) UnloadRenderTexture(m_taaRT0);
      if (m_taaRT1.id != 0) UnloadRenderTexture(m_taaRT1);
      m_taaRT0 = RenderTexture2D{};
      m_taaRT1 = RenderTexture2D{};

      m_taaRT0 = LoadRenderTexture(tw, th);
      m_taaRT1 = LoadRenderTexture(tw, th);
      m_taaRTWidth = tw;
      m_taaRTHeight = th;

      if ((m_taaRT0.id == 0) || (m_taaRT1.id == 0)) {
        if (m_taaRT0.id != 0) UnloadRenderTexture(m_taaRT0);
        if (m_taaRT1.id != 0) UnloadRenderTexture(m_taaRT1);
        m_taaRT0 = RenderTexture2D{};
        m_taaRT1 = RenderTexture2D{};
        m_taaRTValid = false;
        m_taaRTAllocFailed = true;
        m_taaHistoryValid = false;
        TraceLog(LOG_WARNING, "[TAA] failed to allocate TAA render targets (%dx%d)", tw, th);
      } else {
        m_taaRTValid = true;
        m_taaRTAllocFailed = false;
        SetTextureFilter(m_taaRT0.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureFilter(m_taaRT1.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureWrap(m_taaRT0.texture, TEXTURE_WRAP_CLAMP);
        SetTextureWrap(m_taaRT1.texture, TEXTURE_WRAP_CLAMP);
      }
    }

    if (m_taaRTValid) {
      const Vector2 texel{(tw > 0) ? (1.0f / static_cast<float>(tw)) : 0.0f,
                          (th > 0) ? (1.0f / static_cast<float>(th)) : 0.0f};

      // fragTexCoord is generated from DrawTexturePro, and when drawing render textures
      // the caller typically flips the Y axis (negative src.height). Convert pixel jitter
      // in screen space to UV space in the same coordinate system.
      const Vector2 jitterUV{taaJitterPixels.x * texel.x, -taaJitterPixels.y * texel.y};

      const float histW = std::clamp(settings.taaHistory, 0.0f, 1.0f);
      const float resp = std::clamp(settings.taaResponse, 0.0f, 1.0f);

      const bool reset = taaResetHistory || !m_taaHistoryValid;
      const int resetI = reset ? 1 : 0;

      const Rectangle dstTaa{0.0f, 0.0f, static_cast<float>(tw), static_cast<float>(th)};

      BeginTextureMode(m_taaRT1);
      BeginShaderMode(m_taa);

      if (m_locTaaHistory >= 0) SetShaderValueTexture(m_taa, m_locTaaHistory, m_taaRT0.texture);
      if (m_locTaaTexelSize >= 0) SetShaderValue(m_taa, m_locTaaTexelSize, &texel, SHADER_UNIFORM_VEC2);
      if (m_locTaaJitterUV >= 0) SetShaderValue(m_taa, m_locTaaJitterUV, &jitterUV, SHADER_UNIFORM_VEC2);
      if (m_locTaaHistoryWeight >= 0) SetShaderValue(m_taa, m_locTaaHistoryWeight, &histW, SHADER_UNIFORM_FLOAT);
      if (m_locTaaResponse >= 0) SetShaderValue(m_taa, m_locTaaResponse, &resp, SHADER_UNIFORM_FLOAT);
      if (m_locTaaReset >= 0) SetShaderValue(m_taa, m_locTaaReset, &resetI, SHADER_UNIFORM_INT);

      // Draw the current frame into the resolve target.
      DrawTexturePro(tex, src, dstTaa, Vector2{0.0f, 0.0f}, 0.0f, tint);

      EndShaderMode();
      EndTextureMode();

      // Swap: RT0 becomes the new history.
      std::swap(m_taaRT0, m_taaRT1);
      m_taaHistoryValid = true;

      baseTex = &m_taaRT0.texture;
      baseSrc = Rectangle{0.0f, 0.0f, static_cast<float>(tw), -static_cast<float>(th)};
    }
  }

  // ---------------------------------------------------------------------------
  // Bloom pre-pass (optional)
  // ---------------------------------------------------------------------------
  const float bloomStrength = std::clamp(settings.bloom, 0.0f, 1.0f);
  bool drewBloom = false;

  if (bloomStrength > 0.0001f && m_bloomReady) {
    const int screenW = GetScreenWidth();
    const int screenH = GetScreenHeight();
    const int ds = std::clamp(settings.bloomDownsample, 1, 8);

    const int bw = std::max(1, screenW / ds);
    const int bh = std::max(1, screenH / ds);

    const bool sizeChanged = (m_bloomRTWidth != bw) || (m_bloomRTHeight != bh);
    if (sizeChanged) {
      // New resolution: allow another allocation attempt even if a previous one failed.
      m_bloomRTAllocFailed = false;
    }

    // Avoid hammering the GPU with allocation attempts every frame if we already
    // failed at this resolution.
    const bool shouldTryAlloc = !m_bloomRTValid && !m_bloomRTAllocFailed;

    if (sizeChanged || shouldTryAlloc) {
      // Unload any previous buffers (even if the last allocation attempt was only partial).
      if (m_bloomRT0.id != 0) UnloadRenderTexture(m_bloomRT0);
      if (m_bloomRT1.id != 0) UnloadRenderTexture(m_bloomRT1);
      m_bloomRT0 = RenderTexture2D{};
      m_bloomRT1 = RenderTexture2D{};

      m_bloomRT0 = LoadRenderTexture(bw, bh);
      m_bloomRT1 = LoadRenderTexture(bw, bh);
      m_bloomRTWidth = bw;
      m_bloomRTHeight = bh;

      if ((m_bloomRT0.id == 0) || (m_bloomRT1.id == 0)) {
        if (m_bloomRT0.id != 0) UnloadRenderTexture(m_bloomRT0);
        if (m_bloomRT1.id != 0) UnloadRenderTexture(m_bloomRT1);
        m_bloomRT0 = RenderTexture2D{};
        m_bloomRT1 = RenderTexture2D{};
        m_bloomRTValid = false;
        m_bloomRTAllocFailed = true;
        TraceLog(LOG_WARNING, "[Bloom] failed to allocate bloom render targets (%dx%d)", bw, bh);
      } else {
        m_bloomRTValid = true;
        m_bloomRTAllocFailed = false;
        SetTextureFilter(m_bloomRT0.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureFilter(m_bloomRT1.texture, TEXTURE_FILTER_BILINEAR);
        SetTextureWrap(m_bloomRT0.texture, TEXTURE_WRAP_CLAMP);
        SetTextureWrap(m_bloomRT1.texture, TEXTURE_WRAP_CLAMP);
      }
    }

    if (m_bloomRTValid) {
      const float threshold = std::clamp(settings.bloomThreshold, 0.0f, 1.0f);
      const float knee = std::clamp(settings.bloomKnee, 0.0f, 1.0f) * threshold;
      const float radius = std::clamp(settings.bloomRadius, 0.25f, 4.0f);

      const Rectangle dstBloom{0.0f, 0.0f, static_cast<float>(bw), static_cast<float>(bh)};
      const Rectangle srcBloom{0.0f, 0.0f, static_cast<float>(bw), -static_cast<float>(bh)};

      const Vector2 res{static_cast<float>(bw), static_cast<float>(bh)};
      const Vector2 texel{(bw > 0) ? (1.0f / static_cast<float>(bw)) : 0.0f,
                          (bh > 0) ? (1.0f / static_cast<float>(bh)) : 0.0f};

      // Bright-pass extract into RT0.
      BeginTextureMode(m_bloomRT0);
      ClearBackground(BLACK);
      BeginShaderMode(m_bloomExtract);

      if (m_locBloomThreshold >= 0) SetShaderValue(m_bloomExtract, m_locBloomThreshold, &threshold, SHADER_UNIFORM_FLOAT);
      if (m_locBloomKnee >= 0) SetShaderValue(m_bloomExtract, m_locBloomKnee, &knee, SHADER_UNIFORM_FLOAT);
      if (m_locBloomExtractResolution >= 0) SetShaderValue(m_bloomExtract, m_locBloomExtractResolution, &res, SHADER_UNIFORM_VEC2);
      if (m_locBloomExtractTexelSize >= 0) SetShaderValue(m_bloomExtract, m_locBloomExtractTexelSize, &texel, SHADER_UNIFORM_VEC2);

      DrawTexturePro(*baseTex, baseSrc, dstBloom, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
      EndShaderMode();
      EndTextureMode();

      // Blur (RT0 -> RT1 horizontal, then RT1 -> RT0 vertical).
      {
        // Horizontal.
        BeginTextureMode(m_bloomRT1);
        ClearBackground(BLACK);
        BeginShaderMode(m_bloomBlur);

        const Vector2 dirH{1.0f, 0.0f};
        if (m_locBloomBlurTexelSize >= 0) SetShaderValue(m_bloomBlur, m_locBloomBlurTexelSize, &texel, SHADER_UNIFORM_VEC2);
        if (m_locBloomBlurDirection >= 0) SetShaderValue(m_bloomBlur, m_locBloomBlurDirection, &dirH, SHADER_UNIFORM_VEC2);
        if (m_locBloomBlurRadius >= 0) SetShaderValue(m_bloomBlur, m_locBloomBlurRadius, &radius, SHADER_UNIFORM_FLOAT);

        DrawTexturePro(m_bloomRT0.texture, srcBloom, dstBloom, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
        EndShaderMode();
        EndTextureMode();

        // Vertical.
        BeginTextureMode(m_bloomRT0);
        ClearBackground(BLACK);
        BeginShaderMode(m_bloomBlur);

        const Vector2 dirV{0.0f, 1.0f};
        if (m_locBloomBlurTexelSize >= 0) SetShaderValue(m_bloomBlur, m_locBloomBlurTexelSize, &texel, SHADER_UNIFORM_VEC2);
        if (m_locBloomBlurDirection >= 0) SetShaderValue(m_bloomBlur, m_locBloomBlurDirection, &dirV, SHADER_UNIFORM_VEC2);
        if (m_locBloomBlurRadius >= 0) SetShaderValue(m_bloomBlur, m_locBloomBlurRadius, &radius, SHADER_UNIFORM_FLOAT);

        DrawTexturePro(m_bloomRT1.texture, srcBloom, dstBloom, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
        EndShaderMode();
        EndTextureMode();
      }

      drewBloom = true;
    }
  }

  // ---------------------------------------------------------------------------
  // Main PostFX shader pass (optional, can fail independently of bloom)
  // ---------------------------------------------------------------------------
  if (m_ready) {
    const int bits = std::clamp(settings.colorBits, 2, 8);
    const float dither = std::clamp(settings.ditherStrength, 0.0f, 1.0f);
    const float grain = std::clamp(settings.grain, 0.0f, 1.0f);
    const float vignette = std::clamp(settings.vignette, 0.0f, 1.0f);
    const float chroma = std::clamp(settings.chroma, 0.0f, 1.0f);
    const float scan = std::clamp(settings.scanlines, 0.0f, 1.0f);

    const float fxaa = std::clamp(settings.fxaa, 0.0f, 1.0f);
    const float sharpen = std::clamp(settings.sharpen, 0.0f, 1.0f);

    const float tonemapEnabled = settings.tonemapEnabled ? 1.0f : 0.0f;
    const float exposure = std::clamp(settings.exposure, 0.0f, 4.0f);
    const float contrast = std::clamp(settings.contrast, 0.0f, 2.0f);
    const float saturation = std::clamp(settings.saturation, 0.0f, 2.0f);

    const float outline = std::clamp(settings.outline, 0.0f, 1.0f);
    const float outlineThreshold = std::clamp(settings.outlineThreshold, 0.0f, 1.0f);
    const float outlineThickness = std::clamp(settings.outlineThickness, 0.5f, 4.0f);

    // Lens precipitation controls (optional).
    const float lensWeather = std::clamp(settings.lensWeather, 0.0f, 1.0f);
    const float lensDistort = std::clamp(settings.lensDistort, 0.0f, 1.0f);
    const float lensScale = std::clamp(settings.lensScale, 0.5f, 2.0f);
    const float lensDrips = std::clamp(settings.lensDrips, 0.0f, 1.0f);

    // Weather uniforms (optional; primarily used by lens precipitation).
    const int wxMode = std::clamp(weatherMode, 0, 2);
    const float wxIntensity = std::clamp(weatherIntensity, 0.0f, 1.0f);

    Vector2 wxWindDir = windDir;
    {
      // Normalize, and bias slightly "down" (y+) so the shader remains stable.
      const float l2 = wxWindDir.x * wxWindDir.x + wxWindDir.y * wxWindDir.y;
      if (l2 > 1.0e-6f) {
        const float inv = 1.0f / std::sqrt(l2);
        wxWindDir.x *= inv;
        wxWindDir.y *= inv;
      } else {
        wxWindDir = Vector2{0.0f, 1.0f};
      }
      if (wxWindDir.y < 0.15f) wxWindDir.y = 0.15f;
      const float l3 = wxWindDir.x * wxWindDir.x + wxWindDir.y * wxWindDir.y;
      if (l3 > 1.0e-6f) {
        const float inv = 1.0f / std::sqrt(l3);
        wxWindDir.x *= inv;
        wxWindDir.y *= inv;
      }
    }
    const float wxWindSpeed = std::clamp(windSpeed, 0.05f, 6.0f);

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

    if (m_locTonemapEnabled >= 0) SetShaderValue(m_shader, m_locTonemapEnabled, &tonemapEnabled, SHADER_UNIFORM_FLOAT);
    if (m_locExposure >= 0) SetShaderValue(m_shader, m_locExposure, &exposure, SHADER_UNIFORM_FLOAT);
    if (m_locContrast >= 0) SetShaderValue(m_shader, m_locContrast, &contrast, SHADER_UNIFORM_FLOAT);
    if (m_locSaturation >= 0) SetShaderValue(m_shader, m_locSaturation, &saturation, SHADER_UNIFORM_FLOAT);

    if (m_locOutline >= 0) SetShaderValue(m_shader, m_locOutline, &outline, SHADER_UNIFORM_FLOAT);
    if (m_locOutlineThreshold >= 0) SetShaderValue(m_shader, m_locOutlineThreshold, &outlineThreshold, SHADER_UNIFORM_FLOAT);
    if (m_locOutlineThickness >= 0) SetShaderValue(m_shader, m_locOutlineThickness, &outlineThickness, SHADER_UNIFORM_FLOAT);

    // Lens precipitation uniforms (optional).
    if (m_locLensWeather >= 0) SetShaderValue(m_shader, m_locLensWeather, &lensWeather, SHADER_UNIFORM_FLOAT);
    if (m_locLensDistort >= 0) SetShaderValue(m_shader, m_locLensDistort, &lensDistort, SHADER_UNIFORM_FLOAT);
    if (m_locLensScale >= 0) SetShaderValue(m_shader, m_locLensScale, &lensScale, SHADER_UNIFORM_FLOAT);
    if (m_locLensDrips >= 0) SetShaderValue(m_shader, m_locLensDrips, &lensDrips, SHADER_UNIFORM_FLOAT);

    // Weather uniforms (optional; used by lens precipitation).
    if (m_locWeatherMode >= 0) SetShaderValue(m_shader, m_locWeatherMode, &wxMode, SHADER_UNIFORM_INT);
    if (m_locWeatherIntensity >= 0) SetShaderValue(m_shader, m_locWeatherIntensity, &wxIntensity, SHADER_UNIFORM_FLOAT);
    if (m_locWindDir >= 0) SetShaderValue(m_shader, m_locWindDir, &wxWindDir, SHADER_UNIFORM_VEC2);
    if (m_locWindSpeed >= 0) SetShaderValue(m_shader, m_locWindSpeed, &wxWindSpeed, SHADER_UNIFORM_FLOAT);

    if (m_locResolution >= 0) {
      const Vector2 res{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())};
      SetShaderValue(m_shader, m_locResolution, &res, SHADER_UNIFORM_VEC2);
    }
    if (m_locTexelSize >= 0) {
      const Vector2 texel{(baseTex->width > 0) ? (1.0f / static_cast<float>(baseTex->width)) : 0.0f,
                          (baseTex->height > 0) ? (1.0f / static_cast<float>(baseTex->height)) : 0.0f};
      SetShaderValue(m_shader, m_locTexelSize, &texel, SHADER_UNIFORM_VEC2);
    }

    DrawTexturePro(*baseTex, baseSrc, dst, Vector2{0.0f, 0.0f}, 0.0f, tint);
    EndShaderMode();
  } else {
    // Shader fallback: still draw the scene normally.
    DrawTexturePro(*baseTex, baseSrc, dst, Vector2{0.0f, 0.0f}, 0.0f, tint);
  }

  // ---------------------------------------------------------------------------
  // Bloom composite pass (additive)
  // ---------------------------------------------------------------------------
  if (drewBloom && m_bloomRTValid) {
    BeginBlendMode(BLEND_ADDITIVE);
    Color bcol = WHITE;
    bcol.a = static_cast<unsigned char>(std::clamp(bloomStrength, 0.0f, 1.0f) * 255.0f);

    const Rectangle srcBloomOut{0.0f, 0.0f, static_cast<float>(m_bloomRTWidth),
                                -static_cast<float>(m_bloomRTHeight)};
    DrawTexturePro(m_bloomRT0.texture, srcBloomOut, dst, Vector2{0.0f, 0.0f}, 0.0f, bcol);
    EndBlendMode();
  }
}

} // namespace isocity
