#version 330

struct Material {
	vec4 ambient ;
	vec4 diffuse ;
};

uniform Material gMaterial ;
out vec4 FragColor;
                   
void main()
{
/*    FragColor = gMaterial.diffuse ; */
    FragColor = vec4(1, 1, 1, 1) ;
};

