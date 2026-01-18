#version 330

// Weather screen-space FX (procedural precipitation particles)
//
// Override for Renderer.cpp embedded weatherfx shader.
//
// Uniforms:
//   u_resolution : vec2 (pixels)
//   u_time       : float seconds
//   u_seed       : float [0,1)
//   u_mode       : int (0=clear, 1=rain, 2=snow)
//   u_intensity  : float 0..1
//   u_windDir    : vec2 (normalized-ish, y-down)
//   u_windSpeed  : float multiplier
//   u_day        : float 0..1 (1=day)

in vec2 fragTexCoord;
in vec4 fragColor;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform vec2  u_resolution;   // pixels
uniform float u_time;         // seconds
uniform float u_seed;         // [0,1)
uniform int   u_mode;         // 0=clear, 1=rain, 2=snow
uniform float u_intensity;    // 0..1
uniform vec2  u_windDir;      // normalized-ish, y-down
uniform float u_windSpeed;    // multiplier
uniform float u_day;          // 0..1 (1=day, 0=night)

float saturate(float x) { return clamp(x, 0.0, 1.0); }

// Fast hash without sin/cos (good enough for particle jitter).
float hash12(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

vec2 hash22(vec2 p)
{
    float n = hash12(p);
    return vec2(n, hash12(p + n + 37.0));
}

vec2 safeNormalize(vec2 v)
{
    float l2 = dot(v, v);
    if (l2 < 1.0e-8) return vec2(0.0, 1.0);
    return v * inversesqrt(l2);
}

// Coordinates are in a rotated basis aligned to wind: x=perp, y=dir.
float rainParticles(vec2 uv, float intensity, vec2 seed2, float dayMul, float windSpeed)
{
    float area = max(1.0, u_resolution.x * u_resolution.y);
    float count = clamp(area * 0.00012 * intensity, 60.0, 900.0);
    float cellSize = sqrt(area / max(1.0, count));

    float baseLen = 10.0 + 18.0 * intensity;
    float speed = (650.0 + 900.0 * intensity) * windSpeed;

    vec2 uvm = uv - vec2(0.0, u_time * speed);
    vec2 cell = floor(uvm / cellSize);

    float a = 0.0;

    for (int j = -1; j <= 1; ++j) {
        for (int i = -1; i <= 1; ++i) {
            vec2 g = cell + vec2(float(i), float(j));

            vec2 r2 = hash22(g + seed2 * 19.31);
            float r3 = hash12(g + seed2 * 41.17 + 7.13);

            vec2 center = (g + r2) * cellSize;
            vec2 d = uvm - center;

            float len = baseLen * (0.60 + 0.90 * r3);
            float halfLen = 0.5 * len;

            float w = (0.55 + 0.85 * r2.x) * (0.80 + 0.45 * intensity);

            float lx = 1.0 - smoothstep(w, w + 1.0, abs(d.x));
            float ly = 1.0 - smoothstep(halfLen, halfLen + 1.5, abs(d.y));

            float amp = (18.0 + 88.0 * intensity * (0.35 + 0.65 * r3)) * dayMul / 255.0;
            a += lx * ly * amp;
        }
    }

    return saturate(a);
}

float snowParticles(vec2 uv, float intensity, vec2 seed2, float dayMul, float windSpeed)
{
    float area = max(1.0, u_resolution.x * u_resolution.y);
    float count = clamp(area * 0.00018 * intensity, 120.0, 1800.0);
    float cellSize = sqrt(area / max(1.0, count));

    float speed = (45.0 + 80.0 * intensity) * windSpeed;

    vec2 uvm = uv - vec2(0.0, u_time * speed);
    vec2 cell = floor(uvm / cellSize);

    float a = 0.0;

    for (int j = -1; j <= 1; ++j) {
        for (int i = -1; i <= 1; ++i) {
            vec2 g = cell + vec2(float(i), float(j));

            vec2 r2 = hash22(g + seed2 * 23.71);
            float r3 = hash12(g + seed2 * 55.91 + 3.7);

            vec2 center = (g + r2) * cellSize;

            // Gentle sideways wobble so flakes do not look like a static grid.
            float wobble = sin(u_time * 0.9 + r2.y * 12.0 + u_seed * 6.2831853);
            center.x += wobble * cellSize * 0.12;

            vec2 d = uvm - center;

            float radius = 0.8 + 2.0 * r3;
            float flake = 1.0 - smoothstep(radius, radius + 1.2, length(d));

            float amp = (32.0 + 130.0 * intensity * (0.25 + 0.75 * r3)) * dayMul / 255.0;
            a += flake * amp;
        }
    }

    return saturate(a);
}

void main()
{
    // Raylib's screen coordinate system is y-down. Convert gl_FragCoord (y-up) to y-down.
    vec2 p = vec2(gl_FragCoord.x, u_resolution.y - gl_FragCoord.y);

    float intensity = saturate(u_intensity);
    if (u_mode == 0 || intensity <= 0.001) {
        finalColor = vec4(0.0);
        return;
    }

    vec2 wind = safeNormalize(u_windDir);

    // Snow reads better with a stronger vertical component (less horizontal drift) so it
    // does not smear across the screen when windY is small.
    if (u_mode == 2) {
        wind = safeNormalize(vec2(wind.x * 0.35, wind.y));
    }

    vec2 dir = wind;
    vec2 perp = vec2(-dir.y, dir.x);

    // Rotate into wind basis (orthonormal, preserves lengths).
    vec2 uv = vec2(dot(p, perp), dot(p, dir));

    // Match the existing CPU implementation's "day multiplier" so particles read slightly
    // more strongly at night.
    float dayMul = (u_mode == 2) ? (0.75 + 0.25 * saturate(u_day))
                                  : (0.70 + 0.30 * saturate(u_day));

    vec2 seed2 = vec2(u_seed, u_seed * 1.37 + 0.11);

    vec3 col = vec3(1.0);
    float a = 0.0;

    if (u_mode == 1) {
        a = rainParticles(uv, intensity, seed2, dayMul, max(0.05, u_windSpeed));
        col = vec3(210.0/255.0, 220.0/255.0, 235.0/255.0);
    } else {
        a = snowParticles(uv, intensity, seed2, dayMul, max(0.05, u_windSpeed));
        col = vec3(1.0);
    }

    vec4 tint = colDiffuse * fragColor;
    finalColor = vec4(col * tint.rgb, a * tint.a);
}
