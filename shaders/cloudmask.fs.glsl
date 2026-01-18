#version 330

// Cloud shadow mask fragment shader (tileable, GPU-generated).
//
// Renders a tileable alpha mask into a small RenderTexture2D, which the engine
// repeats across the world to darken terrain/buildings slightly.

in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform float u_time;
uniform float u_seed;
uniform float u_coverage;
uniform float u_softness;
uniform float u_evolve;

// Hash without trig (stable and cheap).
uint hash_u32(uint x)
{
    x ^= x >> 16;
    x *= 2246822519u;
    x ^= x >> 13;
    x *= 3266489917u;
    x ^= x >> 16;
    return x;
}

float hash01(uvec2 v, uint seed)
{
    uint h = hash_u32(v.x * 1664525u + v.y * 1013904223u + seed);
    return float(h & 0x00FFFFFFu) / 16777216.0; // 24-bit mantissa
}

// Tileable value noise in [0,1] over a discrete period.
float noiseTile(vec2 p, int period, uint seed)
{
    vec2 i0f = floor(p);
    vec2 f = fract(p);
    vec2 u = f*f*(3.0 - 2.0*f);

    ivec2 i0 = ivec2(i0f);
    int px0 = int(mod(float(i0.x), float(period)));
    int py0 = int(mod(float(i0.y), float(period)));
    int px1 = (px0 + 1) % period;
    int py1 = (py0 + 1) % period;

    float a = hash01(uvec2(uint(px0), uint(py0)), seed);
    float b = hash01(uvec2(uint(px1), uint(py0)), seed);
    float c = hash01(uvec2(uint(px0), uint(py1)), seed);
    float d = hash01(uvec2(uint(px1), uint(py1)), seed);

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float fbmTile(vec2 p, int period, uint seed)
{
    float v = 0.0;
    float a = 0.5;
    int per = period;
    for (int i = 0; i < 5; ++i) {
        v += a * noiseTile(p, per, seed + uint(i) * 101u);
        p = p * 2.02 + vec2(37.1, 17.7);
        per *= 2;
        a *= 0.5;
    }
    return v;
}

void main()
{
    // We render into a small mask texture; use texCoord as stable [0,1] domain.
    const int basePeriod = 32;

    // Convert seed float -> stable uint.
    uint s = uint(fract(u_seed) * 65535.0) ^ 0xC10D15u;

    float cov = clamp(u_coverage, 0.0, 1.0);
    float soft = clamp(u_softness, 0.0, 1.0);
    float evol = clamp(u_evolve, 0.0, 1.0);

    // Time is only used for morphing; translation is handled in world space.
    float t = u_time * (0.12 + 0.65 * evol);

    // Base coords in a repeating domain.
    vec2 p = fragTexCoord * float(basePeriod);
    p += vec2(11.3, 4.9);

    // Domain warp to break up repetition; keep warp strength small to preserve tiling.
    vec2 w;
    w.x = fbmTile(p + vec2(0.0, 0.0) + t * 0.75, basePeriod, s);
    w.y = fbmTile(p + vec2(5.2, 1.3) - t * 0.55, basePeriod, s ^ 0xA341316Cu);
    p += (w - 0.5) * (2.2 + 1.5 * soft);

    float n = fbmTile(p + vec2(19.7, 7.1) + t * 0.18, basePeriod, s ^ 0x51A5EEDu);

    // Higher coverage => lower threshold.
    float thr = mix(0.80, 0.32, cov);
    float edge = mix(0.04, 0.18, soft);
    float m = smoothstep(thr - edge, thr + edge, n);

    // Thicker centers; softer edges.
    m *= (0.70 + 0.45 * n);
    m = clamp(m, 0.0, 1.0);
    m = m * m;

    // Raylib pipeline modulation.
    vec4 base = texture(texture0, fragTexCoord);
    vec4 modulate = base * colDiffuse * fragColor;
    finalColor = vec4(modulate.rgb, modulate.a * m);
}
