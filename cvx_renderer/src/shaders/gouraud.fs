#version 330
	
in vec3 Color0;
out vec4 FragColor;

void main (void)
{
  FragColor = vec4(Color0, 1);
}
