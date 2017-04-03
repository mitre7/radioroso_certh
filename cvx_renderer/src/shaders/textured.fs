#version 330

uniform sampler2D texUnit;

in vec2 TexCoord;
out vec4 fragColor;

void main()
{
   fragColor = texture(texUnit, TexCoord);
};
