#version 330

in vec2 fragTexCoord;
in vec4 fragColor;
in vec2 vWorldPos;

out vec4 finalColor;

uniform sampler2D texture0;
uniform sampler2D u_materialMask;
uniform vec4 colDiffuse;

uniform float u_time;
uniform vec2  u_windDir;
uniform float u_windSpeed;

// 1/texture dimensions for texture0 (in UV units).
uniform vec2  u_texelSize;

// World-space noise frequency multiplier.
uniform float u_freq;

uniform float u_seed;

uniform float u_waterStrength;
uniform float u_waterDistortPx;
uniform float u_waterSparkle;

uniform float u_foamStrength;
uniform float u_foamWidthPx;
uniform float u_causticsStrength;

uniform float u_wetSandStrength;
uniform float u_wetSandWidthPx;

uniform float u_vegStrength;

float saturate(float x) { return clamp(x, 0.0, 1.0); }

vec2 safeNormalize(vec2 v)
{
    float l = length(v);
    return (l > 1e-6) ? (v / l) : vec2(0.0, 1.0);
}

float hash12(vec2 p)
{
    float h = dot(p, vec2(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

float noise21(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f*f*(3.0 - 2.0*f);

    float a = hash12(i + vec2(0.0, 0.0));
    float b = hash12(i + vec2(1.0, 0.0));
    float c = hash12(i + vec2(0.0, 1.0));
    float d = hash12(i + vec2(1.0, 1.0));

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float fbm21(vec2 p)
{
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 5; ++i) {
        v += a * noise21(p);
        p = p * 2.0 + vec2(37.1, 17.7);
        a *= 0.5;
    }
    return v;
}

float sampleWater(vec2 uv)
{
    return texture(u_materialMask, clamp(uv, vec2(0.0), vec2(1.0))).r;
}

float waterEdge(vec2 uv, vec2 off)
{
    float c = sampleWater(uv);
    float n = sampleWater(uv + vec2(0.0, -off.y));
    float s = sampleWater(uv + vec2(0.0,  off.y));
    float e = sampleWater(uv + vec2( off.x, 0.0));
    float w = sampleWater(uv + vec2(-off.x, 0.0));
    float m = min(min(n, s), min(e, w));
    return c * (1.0 - m);
}

float waterNearby(vec2 uv, vec2 off)
{
    float n = sampleWater(uv + vec2(0.0, -off.y));
    float s = sampleWater(uv + vec2(0.0,  off.y));
    float e = sampleWater(uv + vec2( off.x, 0.0));
    float w = sampleWater(uv + vec2(-off.x, 0.0));
    return max(max(n, s), max(e, w));
}

void main()
{
    vec4 base = texture(texture0, fragTexCoord);

    vec4 mask = texture(u_materialMask, fragTexCoord);
    float water = mask.r;
    float veg = mask.g;
    float sand = mask.b;

    // Early out for pixels with no material effects.
    if (water <= 0.001 && veg <= 0.001 && sand <= 0.001) {
        vec4 modulate = colDiffuse * fragColor;
        finalColor = vec4(base.rgb * modulate.rgb, base.a * modulate.a);
        return;
    }

    vec2 windDir = safeNormalize(u_windDir);
    float windSpeed = max(u_windSpeed, 0.0);
    vec2 wind = windDir * max(windSpeed, 0.05);

    // World-space coordinates drive the noise so the pattern sticks to the world.
    vec2 p = vWorldPos * u_freq;
    p += wind * (u_time * 0.035);
    p += vec2(u_seed * 17.13, u_seed * 9.71);

    float n1 = fbm21(p);
    float n2 = fbm21(p * 1.71 + vec2(13.2, 7.9));

    // Signed flow field from noise.
    vec2 flow = (vec2(n1, n2) - 0.5) * 2.0;

    // --- Water: subtle UV distortion + sparkle highlights ---
    vec3 col = base.rgb;
    if (water > 0.001) {
        float distortPx = max(u_waterDistortPx, 0.0);
        vec2 uvD = fragTexCoord + flow * u_texelSize * distortPx;
        uvD = clamp(uvD, vec2(0.0), vec2(1.0));

        vec3 warped = texture(texture0, uvD).rgb;
        float w = saturate(water * u_waterStrength);
        col = mix(col, warped, w);

        float ridges = fbm21(p * 3.10 + windDir * (u_time * 0.05) + wind * 0.35);
        float sp = smoothstep(0.78, 0.98, ridges);
        sp *= water * clamp(u_waterSparkle, 0.0, 2.0);
        col += sp * vec3(0.10, 0.12, 0.15);
    }

    // --- Water: shoreline foam ---
    float foamStrength = clamp(u_foamStrength, 0.0, 2.0);
    if (water > 0.001 && foamStrength > 0.001) {
        float wpx = max(u_foamWidthPx, 0.5);
        vec2 off = u_texelSize * wpx;

        float shore = waterEdge(fragTexCoord, off);

        // Break up the foam band with moving noise and ridges.
        vec2 fp = p * 4.20 + windDir * (u_time * 0.25) + vec2(2.3, 7.1);
        float fn = fbm21(fp);

        vec2 rp = p * 8.50 - windDir * (u_time * 0.20) + vec2(9.7, 2.1);
        float rn = fbm21(rp);

        float f = smoothstep(0.40, 0.92, fn) * (0.55 + 0.45 * smoothstep(0.35, 0.85, rn));
        float foam = saturate(shore * f);

        vec3 foamCol = vec3(0.93, 0.94, 0.95);
        float a = saturate(foam * foamStrength * 0.45);
        col = mix(col, foamCol, a);
    }

    // --- Water: caustics (subtle bright patterns) ---
    float causticsStrength = clamp(u_causticsStrength, 0.0, 2.0);
    if (water > 0.001 && causticsStrength > 0.001) {
        vec2 cp = p * 6.60 + windDir * (u_time * 0.55) + vec2(11.3, 4.9);
        float cn = fbm21(cp);
        float c = smoothstep(0.55, 0.90, cn);
        col += water * c * causticsStrength * vec3(0.07, 0.09, 0.12);
    }

    // --- Shoreline sand: wet darkening + sparkles ---
    float wetStrength = clamp(u_wetSandStrength, 0.0, 2.0);
    if (sand > 0.001 && wetStrength > 0.001) {
        float wpx = max(u_wetSandWidthPx, 0.5);
        vec2 off = u_texelSize * wpx;

        float nearW = waterNearby(fragTexCoord, off);
        float wet = saturate(sand * nearW);

        // Darken toward the water.
        float dark = saturate(wet * wetStrength * 0.55);
        col *= (1.0 - dark * 0.18);

        // Add occasional small sparkles from micro ripples at the edge.
        vec2 spP = p * 5.00 + windDir * (u_time * 0.35) + vec2(21.1, 7.4);
        float sn = fbm21(spP);
        float sp = smoothstep(0.82, 0.98, sn) * wet * wetStrength;
        col += sp * vec3(0.05, 0.04, 0.03);
    }

    // --- Vegetation: brightness flutter ---
    if (veg > 0.001) {
        float flutter = fbm21(p * 1.35 - wind * 0.22) - 0.5;
        float v = veg * clamp(u_vegStrength, 0.0, 2.0);

        col *= (1.0 + v * flutter * 0.18);
        col.g += v * flutter * 0.05;
    }

    col = clamp(col, vec3(0.0), vec3(1.0));

    vec4 modulate = colDiffuse * fragColor;
    finalColor = vec4(col * modulate.rgb, base.a * modulate.a);
}
