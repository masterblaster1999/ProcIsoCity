#version 330

#include "common.glsl"

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

// These are optional but the built-in pipeline will provide them when present.
uniform vec2 u_resolution; // screen resolution in pixels
uniform vec2 u_texelSize;  // 1/texture0 size

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

float luma(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec3 fxaaFromDiag(vec2 uv, vec2 texel, vec3 rgbM, vec3 rgbNW, vec3 rgbNE, vec3 rgbSW, vec3 rgbSE)
{
    float lumaNW = luma(rgbNW);
    float lumaNE = luma(rgbNE);
    float lumaSW = luma(rgbSW);
    float lumaSE = luma(rgbSE);
    float lumaM  = luma(rgbM);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    float lumaRange = lumaMax - lumaMin;

    // No edge: skip the expensive taps.
    if (lumaRange < 0.0312)
        return rgbM;

    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

    const float FXAA_REDUCE_MIN = 1.0/128.0;
    const float FXAA_REDUCE_MUL = 1.0/8.0;
    const float FXAA_SPAN_MAX   = 8.0;

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);

    dir = clamp(dir * rcpDirMin, vec2(-FXAA_SPAN_MAX), vec2(FXAA_SPAN_MAX)) * texel;

    // Two candidate blends along the estimated edge direction.
    vec3 rgbA = 0.5 * (
        texture(texture0, uv + dir * (1.0/3.0 - 0.5)).rgb +
        texture(texture0, uv + dir * (2.0/3.0 - 0.5)).rgb);

    vec3 rgbB = rgbA * 0.5 + 0.25 * (
        texture(texture0, uv + dir * -0.5).rgb +
        texture(texture0, uv + dir *  0.5).rgb);

    float lumaB = luma(rgbB);
    if (lumaB < lumaMin || lumaB > lumaMax)
        return rgbA;

    return rgbB;
}

void main()
{
    vec2 uv = fragTexCoord;

    // Base sample.
    vec4 base = texture(texture0, uv);
    vec3 col = base.rgb;

    // FXAA + unsharp mask, both optional.
    float fxaaStrength = clamp(u_fxaa, 0.0, 1.0);
    float sharpenStrength = clamp(u_sharpen, 0.0, 1.0);

    if (fxaaStrength > 0.0001 || sharpenStrength > 0.0001)
    {
        vec3 rgbNW = texture(texture0, uv + u_texelSize * vec2(-1.0, -1.0)).rgb;
        vec3 rgbNE = texture(texture0, uv + u_texelSize * vec2( 1.0, -1.0)).rgb;
        vec3 rgbSW = texture(texture0, uv + u_texelSize * vec2(-1.0,  1.0)).rgb;
        vec3 rgbSE = texture(texture0, uv + u_texelSize * vec2( 1.0,  1.0)).rgb;

        if (fxaaStrength > 0.0001)
        {
            vec3 aa = fxaaFromDiag(uv, u_texelSize, col, rgbNW, rgbNE, rgbSW, rgbSE);
            col = mix(col, aa, fxaaStrength);
        }

        if (sharpenStrength > 0.0001)
        {
            vec3 blur = (rgbNW + rgbNE + rgbSW + rgbSE) * 0.25;
            vec3 sharp = col + (col - blur) * (sharpenStrength * 1.25);
            col = clamp(sharp, 0.0, 1.0);
        }
    }

    // Chromatic aberration (screen-space radial RGB split).
    if (u_chroma > 0.0001)
    {
        vec2 c = uv - vec2(0.5);
        vec2 off = c * (0.010 * u_chroma);
        vec3 split = vec3(
            texture(texture0, uv + off).r,
            col.g,
            texture(texture0, uv - off).b);
        col = mix(col, split, clamp(u_chroma, 0.0, 1.0));
    }

    // Apply raylib tinting.
    vec4 modulate = colDiffuse * fragColor;
    col *= modulate.rgb;
    float a = base.a * modulate.a;

    // Vignette (centered; aspect-aware).
    if (u_vignette > 0.0001)
    {
        vec2 d = uv - vec2(0.5);
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
        col += (n - 0.5) * (0.08 * u_grain);
    }

    // Scanlines (very subtle). Uses screen-space Y so it stays stable during camera motion.
    if (u_scanlines > 0.0001)
    {
        float s = sin(gl_FragCoord.y * 3.14159265);
        col *= (1.0 - u_scanlines * 0.06 * (0.5 + 0.5 * s));
    }

    // Dithered quantization.
    int bits = clamp(u_bits, 2, 8);
    float levels = pow(2.0, float(bits)) - 1.0;

    float b = bayer4(gl_FragCoord.xy, u_seed);
    float d = (b - 0.5) * u_dither;

    col = floor(col * levels + d + 0.5) / levels;
    col = clamp(col, 0.0, 1.0);

    finalColor = vec4(col, a);
}
