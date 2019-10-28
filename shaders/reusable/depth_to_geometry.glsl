#ifndef DEPTHTOGEOMETRY_GLSL
#define DEPTHTOGEOMETRY_GLSL

vec3 wsFromDepth(in vec3 ndcPos, in mat4 viewMatrixInv, in mat4 projMatrixInv)
{
	vec4 vsPos = projMatrixInv * vec4(ndcPos, 1.0);

	// Perspective division
	vsPos /= vsPos.w;

	vec4 wsPos = viewMatrixInv * vsPos;

	return wsPos.xyz;
}

vec3 wsFromDepth(in sampler2D depthTex, in vec2 texCoord, in mat4 viewMatrixInv, in mat4 projMatrixInv, in float mipmapLevel)
{
	vec3 ndcPos = vec3(texCoord, textureLod(depthTex, texCoord, mipmapLevel).x) * 2.0 - 1.0;

	if (ndcPos.z >= 1.0 || ndcPos.z <= -1.0) return vec3(0.0);

	return wsFromDepth(ndcPos, viewMatrixInv, projMatrixInv);
}

vec3 wsFromDepth(in sampler2D depthTex, in vec2 texCoord, in mat4 viewMatrixInv, in mat4 projMatrixInv)
{
	return wsFromDepth(depthTex, texCoord, viewMatrixInv, projMatrixInv, 0);
}

vec3 normalFromDepth(in sampler2D depthTex, in vec2 texCoord, in mat4 viewMatrixInv, in mat4 projMatrixInv, in float mipmapLevel)
{
	const bool computePrecise = true;

	vec2 texSize = textureSize(depthTex, 0).xy;
	vec2 texUnit = 1.0 / texSize;

	vec3 ws  = wsFromDepth(depthTex, texCoord, viewMatrixInv, projMatrixInv, mipmapLevel);
	vec3 wsX = wsFromDepth(depthTex, texCoord + vec2(texUnit.x, 0), viewMatrixInv, projMatrixInv, mipmapLevel);
	vec3 wsY = wsFromDepth(depthTex, texCoord + vec2(0, texUnit.y), viewMatrixInv, projMatrixInv, mipmapLevel);

	vec3 dx = wsX - ws;
	vec3 dy = wsY - ws;

	if (computePrecise)
	{
		vec3 wsX_ = wsFromDepth(depthTex, texCoord - vec2(texUnit.x, 0), viewMatrixInv, projMatrixInv, mipmapLevel);
		vec3 wsY_ = wsFromDepth(depthTex, texCoord - vec2(0, texUnit.y), viewMatrixInv, projMatrixInv, mipmapLevel);

		vec3 dx_ = ws - wsX_;
		vec3 dy_ = ws - wsY_;

		if (dot(dx_, dx_) < dot(dx, dx)) dx = dx_;
		if (dot(dy_, dy_) < dot(dy, dy)) dy = dy_;
	}

	vec3 normal = cross(dx, dy);

	return clamp(normalize(normal), -1.0, 1.0); // Clamp to avoid numerical errors
}

vec3 normalFromDepth(in sampler2D depthTex, in vec2 texCoord, in mat4 viewMatrixInv, in mat4 projMatrixInv)
{
	return normalFromDepth(depthTex, texCoord, viewMatrixInv, projMatrixInv, 0);
}


// Helper function to do normal and ws at the same time
void geomFromDepth(in sampler2D depthTex, in vec2 texCoord, in mat4 viewMatrixInv, in mat4 projMatrixInv, inout vec3 ws, inout vec3 normal, float mipmapLevel)
{
	const bool computePrecise = true;

	vec2 texSize = textureSize(depthTex, 0).xy;
	vec2 texUnit = 1.0 / texSize;

	ws = wsFromDepth(depthTex, texCoord, viewMatrixInv, projMatrixInv, mipmapLevel);
	vec3 wsX = wsFromDepth(depthTex, texCoord + vec2(texUnit.x, 0), viewMatrixInv, projMatrixInv, mipmapLevel);
	vec3 wsY = wsFromDepth(depthTex, texCoord + vec2(0, texUnit.y), viewMatrixInv, projMatrixInv, mipmapLevel);

	vec3 dx = wsX - ws;
	vec3 dy = wsY - ws;

	if (computePrecise)
	{
		vec3 wsX_ = wsFromDepth(depthTex, texCoord - vec2(texUnit.x, 0), viewMatrixInv, projMatrixInv, mipmapLevel);
		vec3 wsY_ = wsFromDepth(depthTex, texCoord - vec2(0, texUnit.y), viewMatrixInv, projMatrixInv, mipmapLevel);

		vec3 dx_ = ws - wsX_;
		vec3 dy_ = ws - wsY_;

		if (dot(dx_,dx_) < dot(dx,dx)) dx = dx_;
		if (dot(dy_,dy_) < dot(dy,dy)) dy = dy_;
	}

	normal = clamp(normalize(cross(dx, dy)), -1.0, 1.0);

}

void geomFromDepth(in sampler2D depthTex, in vec2 texCoord, in mat4 viewMatrixInv, in mat4 projMatrixInv, inout vec3 ws, inout vec3 normal)
{
	geomFromDepth(depthTex, texCoord, viewMatrixInv, projMatrixInv, ws, normal, 0);
}

#endif // DEPTHTOGEOMETRY_GLSL
