#version 330

#include "common.glsl"

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Optional convenience uniforms (safe to ignore in custom overrides).
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

// Filmic tonemap + grade (optional).
uniform float u_tonemapEnabled;
uniform float u_exposure;
uniform float u_contrast;
uniform float u_saturation;

// Screen-space outlines (optional).
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

float luma(vec3 c)
{
    return dot(c, vec3(0.299, 0.587, 0.114));
}

vec2 hash22(vec2 p)
{
    // Two uncorrelated hashes derived from hash12 (from common.glsl).
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
    // Fast Approximate Anti-Aliasing (FXAA) - simplified.
    float lumaNW = luma(rgbNW);
    float lumaNE = luma(rgbNE);
    float lumaSW = luma(rgbSW);
    float lumaSE = luma(rgbSE);
    float lumaM  = luma(rgbM);

    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    float lumaRange = lumaMax - lumaMin;

    if (lumaRange < 0.0312) return rgbM;

    vec2 dir;
    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

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

    vec4 base = texture(texture0, uv);
    vec3 col = clamp(base.rgb + lensHi, 0.0, 1.0);

    // Outline factor (computed from neighborhood, applied later).
    float edge = 0.0;

    // FXAA + sharpening + outlines share neighborhood taps.
    if ((u_fxaa > 0.0001) || (u_sharpen > 0.0001) || (u_outline > 0.0001))
    {
        vec2 t = u_texelSize;

        vec3 rgbNW = texture(texture0, uv + t * vec2(-1.0, -1.0)).rgb;
        vec3 rgbNE = texture(texture0, uv + t * vec2( 1.0, -1.0)).rgb;
        vec3 rgbSW = texture(texture0, uv + t * vec2(-1.0,  1.0)).rgb;
        vec3 rgbSE = texture(texture0, uv + t * vec2( 1.0,  1.0)).rgb;

        if (u_fxaa > 0.0001)
        {
            vec3 aa = fxaaFromNeighbors(uv, t, col, rgbNW, rgbNE, rgbSW, rgbSE);
            col = mix(col, aa, clamp(u_fxaa, 0.0, 1.0));
        }

        if (u_sharpen > 0.0001)
        {
            vec3 blur = (rgbNW + rgbNE + rgbSW + rgbSE) * 0.25;
            float amt = clamp(u_sharpen, 0.0, 1.0);
            col = clamp(col + (col - blur) * (amt * 1.25), 0.0, 1.0);
        }

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
    if (u_chroma > 0.0001)
    {
        vec2 c = uv - vec2(0.5);
        vec2 off = c * (0.010 * u_chroma);
        vec3 split = vec3(texture(texture0, uv + off).r, col.g, texture(texture0, uv - off).b);
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

    // Vignette.
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

    // Scanlines (very subtle).
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
