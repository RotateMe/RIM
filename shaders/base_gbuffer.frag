#version 450

#define LINEARIZE_DEPTH 0

#include "reusable/linear_depth.glsl"
#include "reusable/depth_to_geometry.glsl"

 in vData
{
	vec3 fragWsPos;
	vec3 fragWsNormal;
	vec2 fragTexCoord;
	vec3 fragColor;
	flat int fragMaterialId;
} In;

layout(location = 0) out vec4 out_wsPos;
layout(location = 1) out vec4 out_wsNormal;
layout(location = 2) out vec2 out_wsTexCoord;
layout(location = 3) out vec4 out_ssMotion;

uniform mat4      vp_prev;

uniform mat4      vinv_prev;
uniform mat4      pinv_prev;

uniform sampler2D depthtex_prev;

uniform vec2      cam_nearfar;


void main()
{
	out_wsPos      = vec4(In.fragWsPos, 1.0);
	out_wsNormal   = vec4(In.fragWsNormal, 1.0);
	out_wsTexCoord = In.fragTexCoord;
	out_ssMotion = vec4(0.0, 0.0, 0.0, 0.0);
		
#if LINEARIZE_DEPTH
	gl_FragDepth = linearizeDepth(gl_FragCoord.z, cam_nearfar.x, cam_nearfar.y);
#endif

	////////////////////// Motion vector computation
	const vec2 buffer_size = textureSize(depthtex_prev, 0).xy;

	vec4 tsPosOld = vp_prev * vec4(In.fragWsPos, 1.0);
	tsPosOld = (tsPosOld/tsPosOld.w) * 0.5 + 0.5;

	const ivec2 texelPosOld = ivec2(tsPosOld.xy * buffer_size );

	if ( any(greaterThanEqual(tsPosOld.xyz, vec3(1.0))) || any(lessThanEqual(tsPosOld.xyz, vec3(0.0))) )  return;

#if LINEARIZE_DEPTH
	const float depthOld = linearizeDepth(tsPosOld.z, cam_nearfar.x, cam_nearfar.y);
#else 
	const float depthOld = tsPosOld.z;
#endif

	const float depthThreshold = 0.005;

	const ivec2 offsets[] = { ivec2(0,0), ivec2(1,0), ivec2(0,1), ivec2(1,1) };
	//vec4  pixelDepths = textureGatherOffsets(depthtex_prev, tsPosOld.xy, offsets, 0);

	bvec4 pass = bvec4(false, false, false, false);

	vec3 npos[4] = {vec3(0), vec3(0), vec3(0), vec3(0)};

	for(int i = 0; i < 4; ++i) {
		float pixelDepth = texelFetch(depthtex_prev, texelPosOld + offsets[i], 0).x; //!!

#if LINEARIZE_DEPTH
		vec3 ndc_prev = vec3((texelPosOld + offsets[i] + 0.5)/buffer_size, unlinearizeDepth(pixelDepth,cam_nearfar.x, cam_nearfar.y)) * 2.0 - 1.0;
#else
		vec3 ndc_prev = vec3((texelPosOld + offsets[i] + 0.5)/buffer_size, pixelDepth) * 2.0 - 1.0;
#endif
		npos[i] = wsFromDepth(ndc_prev, vinv_prev, pinv_prev); 
		
		float pixel_difference = distance(depthOld, pixelDepth);
		pass[i] = pixel_difference < depthThreshold; 
	}

	const vec3 ab = normalize(npos[0]-npos[1]);
	const vec3 ac = normalize(npos[0]-npos[2]);
	const vec3 ad = normalize(npos[0]-npos[3]);
	float coplanarness = abs( dot( (ad), normalize(cross(ab,ac)) ) );

	if (all(pass) && coplanarness < 0.075) {
		out_ssMotion = vec4( tsPosOld.xy, 1.0, 1.0);
	} else {
		float pixelDepth = texelFetch(depthtex_prev, texelPosOld, 0).x;
		if (distance(depthOld, pixelDepth) < depthThreshold) out_ssMotion = vec4( tsPosOld.xy , 0.0, 1.0);
	}

	//out_ssMotion.xy = vec2(0);


}