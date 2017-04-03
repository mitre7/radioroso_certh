#version 330

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Normal;
layout (location = 2) in vec3 Color;
layout (location = 5) in vec2 TexCoords;

out vec3 Normal0;
out vec3 WorldPos0;
out vec2 TexCoord;
out vec3 Color0;

uniform mat4 gProj;
uniform mat4 gModel;
uniform mat3 gNormal ;

void main()
{
    vec4 PosL    = vec4(Position, 1.0);
    gl_Position  = gProj * PosL;
    Normal0      = gNormal * Normal;

    WorldPos0    = (gModel * PosL).xyz;
	TexCoord    = vec2(TexCoords.x, TexCoords.y);
	Color0 = Color ;
}

