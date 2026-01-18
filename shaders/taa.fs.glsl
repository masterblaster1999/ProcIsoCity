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
