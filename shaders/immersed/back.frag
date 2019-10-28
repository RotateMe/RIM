#version 430 core

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uniform mat4 mvp;
uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;

uniform bool viewAlign = false;
uniform float scale = 1.0;

uniform sampler2D quadTex;
uniform bool      quadTexAvailable = false;
uniform vec3      cameraPos;

//layout( location = 0) out vec4 o_normal;

in vData
{
	vec3 WorldPos;
	vec2 TexCoord;
	flat int Instance;
} In;

vec2 rotAround(vec2 v, vec2 axis, float theta)
{
	float c = cos(theta), s = sin(theta);
	v = v - axis;
	v = vec2(c * v.x - s * v.y, s * v.x + c * v.y);
	v = v + axis;
	return v;
}

void main()
{

	vec3 wsPos = In.WorldPos;

	gl_FragDepth = gl_FragCoord.z;

	if (viewAlign)
	{
		float radius = distance(In.TexCoord, vec2(0.5, 0.5));
	
		if (radius > 0.5) discard;
		if (quadTexAvailable && texture(quadTex, rotAround(In.TexCoord, vec2(0.5, 0.5), In.Instance * 341.41284912) ).x > 0.9 ) discard;

		float roundness = 1.0; // 1.0 -> circle
		wsPos -= normalize(cameraPos - wsPos) * scale * cos(radius * M_PI) * roundness; 
		vec4 csPos = pMatrix * vMatrix * vec4(wsPos, 1.0);
		gl_FragDepth = (csPos.z /  csPos.w) * 0.5 + 0.5;

	}
	 

}
