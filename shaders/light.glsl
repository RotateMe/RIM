#ifndef LIGHT_GLSL
#define LIGHT_GLSL

#define LT_DIRECTIONAL (0)
#define LT_POINT       (1)
#define LT_SPOT        (2)

struct Light {
	int   type;
	float intensity;
	float spot_angle;
	float radius;

	vec3 color;
	vec3 position;
	vec3 direction;
	vec3 v1, v2, v3, v4; 

	// v1, .. , v4 parallelogram corners
};

#endif // LIGHT_GLSL