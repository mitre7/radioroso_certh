#version 330

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Normal;

layout (location = 3) in ivec4 BoneIDs;
layout (location = 4) in vec4 Weights;
layout (location = 5) in vec2 TexCoords;

out vec3 Normal0;
out vec3 WorldPos0;
out vec2 TexCoord;

const int MAX_BONES = 100;

uniform mat4 gProj;
uniform mat4 gModel;
uniform mat3 gNormal ;
uniform mat4 gBones[MAX_BONES];

void main()
{
	mat4 BoneTransform = gBones[BoneIDs[0]] * Weights[0];
	if ( BoneIDs[1] >= 0 ) BoneTransform     += gBones[BoneIDs[1]] * Weights[1];
	if ( BoneIDs[2] >= 0 ) BoneTransform     += gBones[BoneIDs[2]] * Weights[2];
	if ( BoneIDs[3] >= 0 ) BoneTransform     += gBones[BoneIDs[3]] * Weights[3];
    
	vec4 PosL    =  BoneTransform * vec4(Position, 1.0);
	gl_Position  = gProj * PosL;

    vec3 NormalL = mat3(BoneTransform) * Normal;
	Normal0      = gNormal * NormalL;

    WorldPos0    = (gModel * PosL).xyz;
	TexCoord    = vec2(TexCoords.x, TexCoords.y);
}
