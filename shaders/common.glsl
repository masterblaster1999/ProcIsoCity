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
