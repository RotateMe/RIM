#ifndef INTERSECTION_GLSL
#define INTERSECTION_GLSL

// Assumes rd is normalized (from user iq at https://www.shadertoy.com/view/4d2XWV)
float sphereIntersect(in vec3 ro, in vec3 rd, in vec4 sph)
{
	vec3 oc = ro - sph.xyz;
	float b = dot(oc, rd);
	float c = dot(oc, oc) - sph.w*sph.w;
	float h = b * b - c;
	if (h < 0.0) return -1.0;
	return -b - sqrt(h);
}

#endif // INTERSECTION_GLSL