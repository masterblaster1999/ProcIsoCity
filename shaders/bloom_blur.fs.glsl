#version 330

// Separable Gaussian blur (run twice: horizontal + vertical).

in vec2 fragTexCoord;
in vec4 fragColor;

out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 colDiffuse;

uniform vec2 u_texelSize;
uniform vec2 u_direction;
uniform float u_radius;

void main()
{
    vec2 step = u_direction * u_texelSize * max(u_radius, 0.0);

    // 9-tap Gaussian-ish blur.
    const float w0 = 0.2270270270;
    const float w1 = 0.1945945946;
    const float w2 = 0.1216216216;
    const float w3 = 0.0540540541;
    const float w4 = 0.0162162162;

    vec3 sum = texture(texture0, fragTexCoord).rgb * w0;

    sum += texture(texture0, fragTexCoord + step*1.0).rgb * w1;
    sum += texture(texture0, fragTexCoord - step*1.0).rgb * w1;

    sum += texture(texture0, fragTexCoord + step*2.0).rgb * w2;
    sum += texture(texture0, fragTexCoord - step*2.0).rgb * w2;

    sum += texture(texture0, fragTexCoord + step*3.0).rgb * w3;
    sum += texture(texture0, fragTexCoord - step*3.0).rgb * w3;

    sum += texture(texture0, fragTexCoord + step*4.0).rgb * w4;
    sum += texture(texture0, fragTexCoord - step*4.0).rgb * w4;

    sum *= (colDiffuse.rgb * fragColor.rgb);
    finalColor = vec4(sum, 1.0);
}
