#version 430 core

in int gl_InstanceID; 

uniform float scale = 1.0;

uniform mat4 mvp;
uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;

uniform bool viewAlign = false;
uniform vec2 outputSize = vec2(0,0);

layout( location = 0 ) in vec3 Position;
layout( location = 1 ) in vec3 Normal;
layout( location = 2 ) in vec2 TexCoord;

layout(std430, binding = 0) buffer InstanceInfo
{
	float position[];
} instances ;

out vData
{
	vec2 TexCoord;
	flat int Instance;
} Out;

void main()
{	
	vec3 instancePos = vec3(instances.position[gl_InstanceID * 3], instances.position[gl_InstanceID * 3 + 1], instances.position[gl_InstanceID * 3 + 2]);
	vec3 worldPos = scale * Position + instancePos;

	if (viewAlign)
	{ // Code for camera-aligned quads
		mat4 vMatrixInv = inverse(vMatrix);

		vec3 camPos = (vMatrixInv * vec4(0, 0, 0, 1)).xyz;
		//vec3 camDir   = normalize(instancePos - camPos);
		vec3 camDir = normalize((vMatrixInv * vec4(0, 0, 1, 1)).xyz - camPos);
		vec3 camUp = vec3(0, 1, 0);
		vec3 camRight = normalize(cross(camDir, camUp));
		camUp = normalize(cross(camDir, camRight));

		mat3 rotInv;
		rotInv[0] = vec3(camRight.x, camUp.x, camDir.x);
		rotInv[1] = vec3(camRight.y, camUp.y, camDir.y);
		rotInv[2] = vec3(camRight.z, camUp.z, camDir.z);
		rotInv = inverse(rotInv);
		worldPos = scale * -rotInv * Position + instancePos;

	}

	gl_Position  = mvp * vec4(worldPos, 1.0);

	Out.TexCoord = TexCoord;
	Out.Instance = gl_InstanceID;
}
