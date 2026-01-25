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

vec3 srgbToLinear(vec3 c)
{
    // Cheap gamma decode. Good enough for a stylized pipeline.
    return pow(max(c, vec3(0.0)), vec3(2.2));
}

vec3 linearToSrgb(vec3 c)
{
    return pow(max(c, vec3(0.0)), vec3(1.0/2.2));
}

// YCoCg conversion is useful for TAA clamping: it reduces chroma shifts compared to RGB
// variance clipping. Operates in linear space.
vec3 rgbToYCoCg(vec3 rgb)
{
    float Co = rgb.r - rgb.b;
    float t  = rgb.b + 0.5 * Co;   // (r + b) * 0.5
    float Cg = rgb.g - t;
    float Y  = t + 0.5 * Cg;       // (r + 2g + b) * 0.25
    return vec3(Y, Co, Cg);
}

vec3 yCoCgToRgb(vec3 ycg)
{
    float Y  = ycg.x;
    float Co = ycg.y;
    float Cg = ycg.z;

    float t = Y - 0.5 * Cg;
    float g = Cg + t;
    float b = t - 0.5 * Co;
    float r = b + Co;
    return vec3(r, g, b);
}

// Manual bilinear sample of texture0.
// This keeps TAA effective even if the underlying texture filter is set to point.
vec4 sample0Bilinear(vec2 uv)
{
    if (u_texelSize.x <= 0.0 || u_texelSize.y <= 0.0)
        return texture(texture0, clampUv(uv));

    vec2 texSize = 1.0 / u_texelSize;

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
    vec4 modulate = colDiffuse * fragColor;

    vec4 curS = sample0Bilinear(cuv) * modulate;
    vec3 curSrgb = curS.rgb;

    // First frame after reset: just seed the history.
    if (u_reset != 0) {
        finalColor = curS;
        return;
    }

    vec3 histSrgb = texture(u_history, uv).rgb;

    // --- Neighborhood clamp + variance clip (YCoCg, linear) ---
    // Convert to linear and YCoCg so variance clipping is less likely to skew chroma.
    vec3 curLin  = srgbToLinear(curSrgb);
    vec3 histLin = srgbToLinear(histSrgb);

    // 9-tap neighborhood improves diagonal edges and reduces flicker.
    vec3 cN  = sample0Bilinear(clampUv(cuv + vec2(0.0, -u_texelSize.y))).rgb * modulate.rgb;
    vec3 cS  = sample0Bilinear(clampUv(cuv + vec2(0.0,  u_texelSize.y))).rgb * modulate.rgb;
    vec3 cE  = sample0Bilinear(clampUv(cuv + vec2( u_texelSize.x, 0.0))).rgb * modulate.rgb;
    vec3 cW  = sample0Bilinear(clampUv(cuv + vec2(-u_texelSize.x, 0.0))).rgb * modulate.rgb;

    vec3 cNW = sample0Bilinear(clampUv(cuv + vec2(-u_texelSize.x, -u_texelSize.y))).rgb * modulate.rgb;
    vec3 cNE = sample0Bilinear(clampUv(cuv + vec2( u_texelSize.x, -u_texelSize.y))).rgb * modulate.rgb;
    vec3 cSW = sample0Bilinear(clampUv(cuv + vec2(-u_texelSize.x,  u_texelSize.y))).rgb * modulate.rgb;
    vec3 cSE = sample0Bilinear(clampUv(cuv + vec2( u_texelSize.x,  u_texelSize.y))).rgb * modulate.rgb;

    vec3 curYC  = rgbToYCoCg(curLin);
    vec3 histYC = rgbToYCoCg(histLin);

    vec3 yN  = rgbToYCoCg(srgbToLinear(cN));
    vec3 yS  = rgbToYCoCg(srgbToLinear(cS));
    vec3 yE  = rgbToYCoCg(srgbToLinear(cE));
    vec3 yW  = rgbToYCoCg(srgbToLinear(cW));
    vec3 yNW = rgbToYCoCg(srgbToLinear(cNW));
    vec3 yNE = rgbToYCoCg(srgbToLinear(cNE));
    vec3 ySW = rgbToYCoCg(srgbToLinear(cSW));
    vec3 ySE = rgbToYCoCg(srgbToLinear(cSE));

    vec3 mn = min(curYC, min(min(min(yN, yS), min(yE, yW)), min(min(yNW, yNE), min(ySW, ySE))));
    vec3 mx = max(curYC, max(max(max(yN, yS), max(yE, yW)), max(max(yNW, yNE), max(ySW, ySE))));

    // Variance clip: clamp history to mean +/- k*sigma (per-channel) to reduce ghosting.
    vec3 sum  = curYC + yN + yS + yE + yW + yNW + yNE + ySW + ySE;
    vec3 mean = sum * (1.0/9.0);

    vec3 sum2 = curYC*curYC + yN*yN + yS*yS + yE*yE + yW*yW + yNW*yNW + yNE*yNE + ySW*ySW + ySE*ySE;
    vec3 var  = max(sum2 * (1.0/9.0) - mean*mean, vec3(0.0));
    vec3 sigma = sqrt(var);

    const float k = 1.25;
    vec3 clipMin = mean - sigma * k;
    vec3 clipMax = mean + sigma * k;

    vec3 histClamped = clamp(histYC, mn, mx);
    histClamped = clamp(histClamped, clipMin, clipMax);

    // Responsiveness term: if the current frame disagrees with history, reduce
    // the history weight to avoid trails.
    float diff = abs(curYC.x - histClamped.x); // Y channel is a good luminance proxy in YCoCg
    float resp = mix(0.0, 8.0, clamp(u_response, 0.0, 1.0));

    float w = clamp(u_historyWeight, 0.0, 1.0);
    w *= clamp(1.0 - diff * resp, 0.0, 1.0);
    w = min(w, 0.98);

    vec3 outYC = mix(curYC, histClamped, w);

    vec3 outLin = yCoCgToRgb(outYC);
    vec3 outSrgb = linearToSrgb(outLin);
    outSrgb = clamp(outSrgb, 0.0, 1.0);

    finalColor = vec4(outSrgb, curS.a);
}
