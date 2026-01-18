#version 330

// Bright-pass extraction for bloom.
//
// Notes:
//  - u_threshold is typically in [0,1].
//  - u_knee provides a soft transition band around the threshold.

in vec2 fragTexCoord;
in vec4 fragColor;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform float u_threshold;
uniform float u_knee;

// Optional convenience uniforms (set if present).
uniform vec2 u_resolution;
uniform vec2 u_texelSize;

float max3(vec3 v)
{
    return max(v.r, max(v.g, v.b));
}

void main()
{
    vec3 c = texture(texture0, fragTexCoord).rgb;
    c *= (colDiffuse.rgb * fragColor.rgb);

    float b = max3(c);
    float t = u_threshold;
    float knee = max(u_knee, 1e-5);

    // Soft threshold: transitions smoothly across a knee band.
    float soft = b - t;
    soft = clamp(soft + knee, 0.0, 2.0*knee);
    soft = (soft*soft) / (4.0*knee + 1e-5);

    float contrib = max(soft, b - t);
    contrib /= max(b, 1e-5);

    vec3 outC = c * contrib;
    finalColor = vec4(outC, 1.0);
}
