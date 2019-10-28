#version 450
#extension GL_ARB_bindless_texture : enable

#define LINEARIZE_DEPTH 0

#include "material.glsl"
#include "light.glsl"
#include "reusable/phong.glsl"
#include "reusable/linear_depth.glsl"

in fData {
	vec3 fragWsPos;
	vec3 fragWsNormal;
	vec3 fragWsTangent;
	vec2 fragTexCoord;
	vec3 fragColor;
	flat int fragMaterialId;
} In;

struct fragmentProperties
{
	vec3  diffuse_color;
	vec3  specular_color;
	vec3  ambient_color;
	float shininess;
	vec3  normal;
} frag;

layout(std430) buffer material_ssbo
{
	Material materials[];
};

layout(std430) buffer light_ssbo
{
	Light lights[];
}; 


layout(location = 0) out vec4 out_color;

uniform vec3 wsCamPos   = vec3(100, 100, 100);

uniform bool      enableAlphaTest    = true;
uniform float     alphaTestThreshold = 0.5;

uniform int       light_qty = 0;
uniform vec3      ambient_light = vec3(0.2);

uniform vec2      cam_nearfar;

void main()
{
	const Material m = materials[In.fragMaterialId];

	// Set fragment value
	const vec3  wsViewDir = normalize(wsCamPos - In.fragWsPos);

	frag.shininess = m.shininess;

	if (m.diffuse_texhandle[0] != 0 || m.diffuse_texhandle[1] != 0) {
		vec4 texValue = texture(sampler2D(m.diffuse_texhandle), In.fragTexCoord);
		if (enableAlphaTest && texValue.a < alphaTestThreshold) discard;
		frag.diffuse_color = texValue.rgb ;
	} else {
		frag.diffuse_color = m.diffuse;
	}	

	if (m.specular_highlight_texhandle[0] != 0 || m.specular_highlight_texhandle[1] != 0) {
		frag.specular_color = texture(sampler2D(m.specular_highlight_texhandle), In.fragTexCoord).rgb;
	} else {
		frag.specular_color = m.specular;
	}

	if (m.ambient_texhandle[0] != 0 || m.ambient_texhandle[1] != 0) {
		frag.ambient_color = texture(sampler2D(m.ambient_texhandle), In.fragTexCoord).rgb;
	}
	else {
		frag.ambient_color = m.ambient;
	}

	if (m.bump_texhandle[0] != 0 || m.bump_texhandle[1] != 0) {
		vec3 normalCoeffs = texture(sampler2D(m.bump_texhandle), In.fragTexCoord).xyz - 0.5;
		vec3 bitangent    = normalize(cross(In.fragWsTangent, In.fragWsNormal));
		frag.normal       = normalize(normalCoeffs.x * In.fragWsTangent + normalCoeffs.y * bitangent + 4.0 * normalCoeffs.z * In.fragWsNormal );
	
	} else {
		frag.normal = In.fragWsNormal;
	}

	vec3 color = frag.diffuse_color * ambient_light;

	for (int i = 0; i < light_qty; ++i)
	{
		const Light l = lights[i];

		const vec3  wsLightDir = l.type == LT_DIRECTIONAL ? -l.direction : normalize(l.position - In.fragWsPos);
		 
		vec3 reflected_color = phong_color(-wsLightDir, frag.normal, wsViewDir, frag.diffuse_color, frag.specular_color, m.shininess);

		if (l.type == LT_SPOT) { 
			const float cosLightAngle = acos(dot(-wsLightDir, l.direction));
			const float angleDiff     = max(0, (l.spot_angle - cosLightAngle) / l.spot_angle);
			reflected_color *= pow(angleDiff, 0.5);
		}

		color += reflected_color * l.intensity * l.color;
	}

	//color = vec3(fract((1+In.fragMaterialId) *  5.15687865), fract((1+In.fragMaterialId) * 7.5416156), fract((1+In.fragMaterialId) * 3.1856876) );
	//color = normalize(frag.normal * 0.5f + 0.5f);
//
#if LINEARIZE_DEPTH
	gl_FragDepth = linearizeDepth(gl_FragCoord.z, cam_nearfar.x, cam_nearfar.y);
#endif


	out_color = vec4(color, 1.0);
}