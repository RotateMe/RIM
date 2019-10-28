#ifndef MATH_GLSL
#define MATH_GLSL

#include "math_constants.glsl"
#include "quaternion.glsl"
#include "matrix.glsl"

#define Vector2 vec2
#define Point2  vec2
#define Vector3 vec3
#define Point3  vec3
#define Vector4 vec4

#define Color3  vec3
#define Radiance3 vec3
#define Biradiance3 vec3
#define Irradiance3 vec3
#define Radiosity3 vec3
#define Power3 vec3

#define Color4  vec4
#define Radiance4 vec4
#define Biradiance4 vec4
#define Irradiance4 vec4
#define Radiosity4 vec4
#define Power4 vec4

#define Vector2int32 int2
#define Vector3int32 int3
#define Matrix4      mat4
#define Matrix3      mat3
#define Matrix2      mat2


float log10(float a) {
	//return log(a) * INV_LOG10;
	return log(a) / log(10);
};




//vec3 pow(vec3 x, float y) {
//	return pow(x, vec3(y));
//}

/*
 float log10(float a) { return ::log10(a); }

 vec2 log10(const vec2& a) {
    return vec2( ::log10(a.x()),
                  ::log10(a.y()) );
  }
vec3 log10(const vec3& a) {
    return vec3( ::log10(a.x()),
                  ::log10(a.y()),
                  ::log10(a.z()) );
  }
vec4 log10(const vec4& a) {
    return vec4( ::log10(a.x()),
                  ::log10(a.y()),
                  ::log10(a.z()),
                  ::log10(a.w()) );
  }
  */

vec3 TransformPoint(in mat4 t, in vec3 v)
{
	vec4 _v = t * vec4(v, 1.0);
	return _v.xyz / _v.w;
}

vec3 TransformDir(mat4 t, vec3 v)
{
	vec4 _v = t * vec4(v, 0.0);
	return _v.xyz;
}

float frac(float x) {
    return fract(x);
}

vec2 frac(vec2 x) {
    return fract(x);
}

vec3 frac(vec3 x) {
    return fract(x);
}

vec4 frac(vec4 x) {
    return fract(x);
}

float atan2(float y, float x) {
    return atan(y, x);
}

float saturate(float value) {
	return clamp(value, 0.0, 1.0);
}

vec2 saturate(vec2 value) {
	return clamp(value.rg, 0.0, 1.0);
}

vec3 saturate(vec3 value) {
	return clamp(value.rgb, 0.0, 1.0);
}

vec4 saturate(vec4 value) {
	return clamp(value.rgba, 0.0, 1.0);
}

float Square(float value)	{ return value * value; }
vec2 Square(vec2 value)		{ return value * value; }
vec3 Square(vec3 value)		{ return value * value; }
vec4 Square(vec4 value)		{ return value * value; }

float Pow5( float x )
{
	float x2 = x*x;
	return x2 * x2 * x;
}

float rcp( float x)
{
	return 1.0 / x;
}

float radicalInverse_VdC(uint bits) {
     bits = (bits << 16u) | (bits >> 16u);
     bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
     bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
     bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
     bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);

     return float(bits) * 2.3283064365386963e-10; // / 0x100000000
 }

 // http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
 vec2 Hammersley(uint i, uint N) {
     return vec2(float(i) / float(N), radicalInverse_VdC(i));
 }
 
float lineToPointDistance(vec3 linePoint, vec3 lineDirection, vec3 point) {
	vec3 ab = normalize(lineDirection);
	vec3 ac = point - linePoint;
	vec3 acab = dot(ac, ab) * ab;
	return distance(linePoint + acab, point);
}


#if 0

vec4 getHammersleyPoint(int i, int nbrSample) {
	vec4 result = vec4(0);
	
	float phi;
	int ui = i;
	result.x = (float) i / (float) nbrSample;

	/* From http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
		* Radical Inverse : Van der Corput */
	ui = (ui << 16) | (ui >> 16);
	ui = ((ui & 0x55555555) << 1) | ((ui & 0xAAAAAAAA) >>> 1);
	ui = ((ui & 0x33333333) << 2) | ((ui & 0xCCCCCCCC) >>> 2);
	ui = ((ui & 0x0F0F0F0F) << 4) | ((ui & 0xF0F0F0F0) >>> 4);
	ui = ((ui & 0x00FF00FF) << 8) | ((ui & 0xFF00FF00) >>> 8);

	ui = ui & 0xffffffff;
	result.y = 2.3283064365386963e-10f * (float) (ui); /* 0x100000000 */

	phi = 2.0f * M_PI * store.y;
	
	result.z = cos(phi);
	result.w = sin(phi);

	return result;
}

#endif

float reconstructCSZ(float depthBufferValue, vec3 clipInfo) {
      return clipInfo[0] / (depthBufferValue * clipInfo[1] + clipInfo[2]);
}


void swap(inout float a, inout float b) {
     float temp = a;
     a = b;
     b = temp;
}


float distanceSquared(vec2 a, vec2 b) {
    a -= b;
    return dot(a, a);
}

float distanceLinesegmentToPointSquared(vec2 linePoint, vec2 lineDirection, vec2 point) {

	// squared length of line segment
	float lineLengthSquared = dot(lineDirection, lineDirection);

	// point to point distance, if line segment has zero length
	if (lineLengthSquared == 0)
		return distanceSquared(linePoint, point);

	// project point onto line
	float t = dot(point - linePoint, lineDirection) / lineLengthSquared;

	// point to point distance, if point 'to the left' of the line segment
	if (t < 0) return distanceSquared(linePoint, point);

	// point to point distance, if point 'to the right' of the line segment
	if (t > 1) return distanceSquared(linePoint + lineDirection, point);
	
	// project point onto the line segment and use point to that projected point distance
	vec2 pointOnLine = linePoint + t * lineDirection;
	return distanceSquared(pointOnLine, point);
}

float distanceLinesegmentToPoint(vec2 linePoint, vec2 lineDirection, vec2 point) {
	return sqrt(distanceLinesegmentToPointSquared(linePoint, lineDirection, point));
}


#endif // MATH_GLSL