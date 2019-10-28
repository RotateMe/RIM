#version 450

#define LINEARIZE_DEPTH 0

#include "reusable/linear_depth.glsl"

#define SHAPE_SPHERE 0
#define SHAPE_CUBE   1
#define SHAPE_QUAD   2

 in vData
{
	vec3 fragWsPos;
	vec3 fragWsNormal;
	vec2 fragTexCoord;
	vec3 fragColor;
	flat int fragMaterialId;
} In;



layout(location = 0) out vec4 out_color;

uniform int  shapeType = SHAPE_SPHERE;
uniform vec3 wsCamPos     = vec3(100, 100, 100);
uniform vec3 wsCamForward = vec3(0, 0, 1);
uniform vec3 lightColor   = vec3(1, 1, 1);
uniform vec3 sphereCenter = vec3(0, 0, 0);
uniform vec2 cam_nearfar;

void main()
{
	vec3 viewDir   = normalize(In.fragWsPos - wsCamPos);
	vec3 wsNormal  = normalize(In.fragWsNormal);

	switch (shapeType) {
	default:
	case SHAPE_SPHERE:
		out_color = vec4(lightColor * dot(wsNormal, normalize(wsCamPos - sphereCenter) ), 1.0);
		break;
	}

#if LINEARIZE_DEPTH
	gl_FragDepth = linearizeDepth(gl_FragCoord.z, cam_nearfar.x, cam_nearfar.y);
#endif
	
}