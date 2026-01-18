#ifndef PROCISOCITY_COMMON_GLSL
#define PROCISOCITY_COMMON_GLSL

// Shared shader helpers for the optional on-disk overrides.
//
// These helpers are intentionally simple and deterministic so they work well
// for procedural, asset-free visuals.

float hash12(vec2 p)
{
    // Small, fast hash. Good enough for film grain and subtle noise.
    float h = dot(p, vec2(127.1, 311.7));
    return fract(sin(h) * 43758.5453123);
}

// Smooth value noise in 2D.
float noise21(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);

    // Cubic Hermite curve for smooth interpolation.
    vec2 u = f*f*(3.0 - 2.0*f);

    float a = hash12(i + vec2(0.0, 0.0));
    float b = hash12(i + vec2(1.0, 0.0));
    float c = hash12(i + vec2(0.0, 1.0));
    float d = hash12(i + vec2(1.0, 1.0));

    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Fractal Brownian motion (FBM) using noise21.
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

#endif // PROCISOCITY_COMMON_GLSL
