#version 330

// Volumetric clouds fragment shader.
//
// Optional on-disk override. The engine will fall back to an embedded shader
// if this file is not present.

in vec2 fragTexCoord;
in vec4 fragColor;
in vec2 vWorldPos;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform vec2 u_viewMin;
uniform vec2 u_viewSize;
uniform float u_time;
uniform vec2 u_windDir;
uniform float u_windSpeed;
uniform float u_scale;
uniform float u_coverage;
uniform float u_density;
uniform float u_softness;
uniform int u_steps;
uniform float u_day;
uniform float u_dusk;
uniform float u_overcast;
uniform float u_seed;
uniform float u_bottomFade;

float hash1(vec3 p)
{
    return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453123);
}

float noise3(vec3 p)
{
    vec3 i = floor(p);
    vec3 f = fract(p);
    vec3 u = f*f*(3.0 - 2.0*f);

    float n000 = hash1(i + vec3(0.0, 0.0, 0.0));
    float n100 = hash1(i + vec3(1.0, 0.0, 0.0));
    float n010 = hash1(i + vec3(0.0, 1.0, 0.0));
    float n110 = hash1(i + vec3(1.0, 1.0, 0.0));
    float n001 = hash1(i + vec3(0.0, 0.0, 1.0));
    float n101 = hash1(i + vec3(1.0, 0.0, 1.0));
    float n011 = hash1(i + vec3(0.0, 1.0, 1.0));
    float n111 = hash1(i + vec3(1.0, 1.0, 1.0));

    float nx00 = mix(n000, n100, u.x);
    float nx10 = mix(n010, n110, u.x);
    float nx01 = mix(n001, n101, u.x);
    float nx11 = mix(n011, n111, u.x);

    float nxy0 = mix(nx00, nx10, u.y);
    float nxy1 = mix(nx01, nx11, u.y);
    return mix(nxy0, nxy1, u.z);
}

float fbm(vec3 p)
{
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 4; ++i) {
        v += a * noise3(p);
        p *= 2.02;
        a *= 0.5;
    }
    return v;
}

float cloudDensity(vec3 p, float cov, float soft)
{
    // Domain warp to break up repetition.
    float w1 = fbm(p + vec3(0.0, 0.0, 0.0));
    float w2 = fbm(p + vec3(5.2, 1.3, 2.1));
    vec3 q = p;
    q.xy += (vec2(w1, w2) - 0.5) * 0.85;

    float n = fbm(q);

    // Higher coverage => lower threshold.
    float thr = mix(0.78, 0.32, clamp(cov, 0.0, 1.0));
    float edge = mix(0.04, 0.18, clamp(soft, 0.0, 1.0));
    float m = smoothstep(thr - edge, thr + edge, n);

    // Vertical shaping: strongest in the middle of the volume.
    float h = smoothstep(0.0, 0.18, q.z) * (1.0 - smoothstep(0.72, 1.0, q.z));
    m *= h;

    // Thicker centers.
    m *= (0.55 + 0.75 * n);
    return clamp(m, 0.0, 1.0);
}

void main()
{
    vec2 uv = (vWorldPos - u_viewMin) / max(u_viewSize, vec2(0.001));

    // World-space -> noise-space.
    vec2 seedOff = vec2(u_seed * 0.00123, u_seed * 0.00173);
    vec2 base = vWorldPos * u_scale + seedOff;
    vec2 wind = u_windDir * (u_time * u_windSpeed);

    // A small internal evolution so clouds "breathe" even if wind is still.
    float evol = u_time * 0.05;

    // Early-out: if a mid-slice is empty, skip the expensive ray-march.
    float c0 = cloudDensity(vec3(base + wind, 0.45 + evol), u_coverage, u_softness);
    if (c0 <= 0.01) {
        finalColor = vec4(0.0);
        return;
    }

    int steps = clamp(u_steps, 8, 64);
    float stepSize = 1.0 / float(steps);

    float alpha = 0.0;
    vec3 col = vec3(0.0);

    // Light direction in noise space (roughly "from upper-left" with a downward component).
    vec3 lightDir = normalize(vec3(-0.55, -0.25, 0.90));

    for (int i = 0; i < 64; ++i) {
        if (i >= steps) break;

        float z = (float(i) + 0.5) * stepSize;
        vec3 p = vec3(base + wind, z + evol);

        float d = cloudDensity(p, u_coverage, u_softness) * c0;
        if (d <= 0.001) continue;

        // Cheap self-shadowing: probe density toward the light.
        float dl = cloudDensity(p + lightDir * 0.35, u_coverage, u_softness);
        float light = clamp(0.35 + 0.65 * (1.0 - dl), 0.0, 1.0);

        // Convert density to alpha contribution.
        float a = clamp(d * u_density * stepSize * 1.45, 0.0, 1.0);

        vec3 sampleCol = mix(vec3(0.55, 0.60, 0.68), vec3(1.0), light);
        col += (1.0 - alpha) * sampleCol * a;
        alpha += (1.0 - alpha) * a;

        if (alpha > 0.985) break;
    }

    // Screen readability: fade clouds toward the bottom of the view.
    float fade = 1.0 - smoothstep(0.55, 0.98, uv.y);
    float fadeMix = mix(1.0, fade, clamp(u_bottomFade, 0.0, 1.0));
    col *= fadeMix;
    alpha *= fadeMix;

    // Day/night tinting (keep subtle; night clouds are darker/less present).
    float day = clamp(u_day, 0.0, 1.0);
    float dusk = clamp(u_dusk, 0.0, 1.0);
    float oc = clamp(u_overcast, 0.0, 1.0);

    vec3 dayTint = vec3(1.02, 1.02, 1.05);
    vec3 duskTint = vec3(1.12, 0.92, 0.78);
    vec3 nightTint = vec3(0.38, 0.42, 0.55);

    vec3 tint = mix(nightTint, dayTint, day);
    tint = mix(tint, duskTint, dusk * 0.75);

    // Overcast makes clouds denser/darker.
    tint *= mix(1.08, 0.85, oc);
    alpha *= mix(0.60, 1.00, oc);

    col *= tint;

    vec4 texel = texture(texture0, fragTexCoord);
    finalColor = vec4(col, clamp(alpha, 0.0, 1.0)) * texel * colDiffuse * fragColor;
}
