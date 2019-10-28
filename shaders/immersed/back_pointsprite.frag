#version 430 core

#include "../Reusable/math_constants.glsl"
#include "../Reusable/intersection.glsl"

uniform mat4 mvp;
uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;

uniform bool viewAlign = false;
uniform float scale = 1.0;

uniform sampler2D quadTex;
uniform bool      quadTexAvailable = false;
uniform vec3      cameraPos;
uniform vec3      cameraRight;
uniform vec3      cameraUp;

in vData
{
	vec3 InstancePos;
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

#define MOD_DEPTH 1

void main()
{
	const vec2 texCoord = gl_PointCoord.st;

	float radius = distance(texCoord, vec2(0.5, 0.5));
	
	if (radius >= 0.5) discard;
	if (quadTexAvailable && texture(quadTex, rotAround(texCoord, vec2(0.5, 0.5), In.Instance * 341.41284912) ).x > 0.5 ) discard;

#if MOD_DEPTH
 	vec3 wsPos = In.InstancePos + 2.0 * scale * ((texCoord.s-0.5) * cameraRight + (texCoord.t-0.5) * cameraUp);
	const vec3 rd = normalize(wsPos-cameraPos);
	const float t = sphereIntersect(cameraPos, rd, vec4(In.InstancePos, scale));
	if (t < 0) discard;
	wsPos = cameraPos + t * rd;

	vec4 csPos = pMatrix * vMatrix * vec4(wsPos, 1.0);
	gl_FragDepth = (csPos.z /  csPos.w) * 0.5 + 0.5;
#endif

}
