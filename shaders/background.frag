#version 450

#include "reusable/math_constants.glsl"
#include "reusable/environment.glsl"

uniform sampler2D map;
uniform vec3 wsCamPos;

uniform int backgroundtype = 0; 

in vData
{
	vec2 texcoord;
	vec3 dir;
} In;

layout(location = 0) out vec4 out_color;

void main()
{
	const vec3 dir = normalize(In.dir);

	vec3 color = textureLod(map, mapViewToEquirectTexCoord(dir), 0).rgb;

	out_color = vec4(color, 1.0);
}