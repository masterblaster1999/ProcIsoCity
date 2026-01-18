#version 330

// Volumetric clouds vertex shader (raylib 2D batch compatible).
//
// Optional on-disk override. The engine will fall back to an embedded shader
// if this file is not present.

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 2) in vec4 vertexColor;

uniform mat4 mvp;

out vec2 fragTexCoord;
out vec4 fragColor;
out vec2 vWorldPos;

void main()
{
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    vWorldPos = vertexPosition.xy;
    gl_Position = mvp*vec4(vertexPosition, 1.0);
}
