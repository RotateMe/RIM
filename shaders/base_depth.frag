#version 450

#define LINEARIZE_DEPTH 0

#include "reusable/linear_depth.glsl"

uniform vec2 cam_nearfar;

void main()
{
#if LINEARIZE_DEPTH
	gl_FragDepth = linearizeDepth(gl_FragCoord.z, cam_nearfar.x, cam_nearfar.y);
#endif
}