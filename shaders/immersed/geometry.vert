#version 450

in int gl_InstanceID; 

uniform float scale = 1.0;

uniform mat4 mvp;
uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;
uniform mat4 vMatrixInv;
uniform vec3 cameraPos;
uniform vec3 cameraRight;
uniform vec3 cameraUp;
uniform vec3 cameraForward;

uniform sampler2D densityTex;

uniform bool viewAlign = false;

layout( location = 0 ) in vec3 Position;
layout( location = 1 ) in vec3 Normal;
layout( location = 2 ) in vec2 TexCoord;

layout(std430, binding = 0) buffer InstanceInfo
{
	float position[];
} instances ;


out vData
{
	vec3  WorldPos;
	vec3  Normal;
	vec2  TexCoord;
	vec3  InstancePos;
	float Scale;
	flat int Instance;
} Out;

void main()
{	
	vec3 instancePos = vec3(instances.position[gl_InstanceID * 3], instances.position[gl_InstanceID * 3 + 1], instances.position[gl_InstanceID * 3 + 2]);

	vec4 ndcInstance = mvp * vec4(instancePos, 1.0);
	ndcInstance.xyz /= ndcInstance.w;
	ndcInstance.xyz  = ndcInstance.xyz * 0.5 + 0.5;

	float newScale = scale;
	vec3 worldPos = newScale * Position + instancePos;

	vec3 normal = Normal;

	if (viewAlign)
	{ // Code for camera-aligned quads

//		vec3 camPos = (vMatrixInv * vec4(0, 0, 0, 1)).xyz;
//		//vec3 camDir   = normalize(instancePos - camPos);
//		vec3 camDir = normalize((vMatrixInv * vec4(0, 0, 1, 1)).xyz - camPos);
//		vec3 camUp = vec3(0, 1, 0);
//		vec3 camRight = normalize(cross(camDir, camUp));
//		camUp = normalize(cross(camDir, camRight));

		mat3 rotInv;
		rotInv[0] = vec3(cameraRight.x, cameraUp.x, -cameraForward.x);
		rotInv[1] = vec3(cameraRight.y, cameraUp.y, -cameraForward.y);
		rotInv[2] = vec3(cameraRight.z, cameraUp.z, -cameraForward.z);
		rotInv = inverse(rotInv);
		worldPos = newScale * -rotInv * Position + instancePos;

		normal = -cameraForward;
	}

	gl_Position  = mvp * vec4(worldPos, 1.0);

	Out.Scale = newScale;
	Out.WorldPos = worldPos;
	Out.Normal   = normal;
	Out.TexCoord = TexCoord;
	Out.Instance = gl_InstanceID;
	Out.InstancePos = instancePos;


}
