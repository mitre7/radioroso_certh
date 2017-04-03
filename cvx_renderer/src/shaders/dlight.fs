#version 330
	
in vec3 Normal0;
in vec3 WorldPos0;

struct DirectionalLight {
	vec3 dir;
	vec4 diffuse ;
	vec4 ambient ;
} ;


struct Material {
	vec4 ambient ;
	vec4 diffuse ;
};

struct VSOutput
{
	vec3 Normal;
	vec3 WorldPos;
};

uniform DirectionalLight gDirectionalLight;
uniform Material gMaterial ;

vec4 CalcDirectionalLight(DirectionalLight light, Material mat, VSOutput In)
{
	vec3 surfaceToLight = normalize(light.dir);
 
    float brightness = max(dot(In.Normal, surfaceToLight), 0) ;
        
    vec4 DiffuseColor = brightness * light.diffuse * mat.diffuse  ;
    vec4 AmbientColor = light.ambient * mat.ambient ;
        
	return clamp(DiffuseColor + AmbientColor, 0, 1);
}

out vec4 FragColor;
                   
void main()
{
	VSOutput In;
	In.Normal   = normalize(Normal0);
	In.WorldPos = WorldPos0;
                              
	vec4 TotalLight = CalcDirectionalLight(gDirectionalLight, gMaterial, In);
	FragColor = TotalLight;
};
