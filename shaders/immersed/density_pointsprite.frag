#version 450

uniform mat4 mvp;
uniform mat4 mMatrix;
uniform mat4 vMatrix;
uniform mat4 pMatrix;

uniform bool viewAlign = false;

uniform sampler2D sceneDepthTex;
uniform sampler2D quadTex;
uniform bool      quadTexAvailable = false;

layout( location = 0) out vec4 o_density;

uniform float scale = 1.0;

in vec2 gl_PointCoord;
in vec3 gl_FragCoord;

in vData
{
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
	if (texelFetch(sceneDepthTex, ivec2(gl_FragCoord.xy), 0).x < gl_FragCoord.z) discard;

	float density = 1.0;

	const vec2 texCoord = gl_PointCoord.st;

	const float radius = distance(texCoord, vec2(0.5, 0.5));
	
	if (radius > 0.5) discard;
	if (quadTexAvailable && texture(quadTex, rotAround(texCoord, vec2(0.5, 0.5), In.Instance * 341.41284912) ).x > 0.8 ) discard;

	density *= cos(radius * 3.141592); // Make density smaller at edges

	o_density = vec4(density, 0.f, 0.f, 1.f);
}
