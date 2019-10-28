#include "helpers/CameraFPS.h"
#include <random>


struct JitterGenerator 
{
	std::mt19937 rng;
	std::uniform_real_distribution<float> d1, d2;
	JitterGenerator::JitterGenerator() : d1(-0.5f,0.5f), d2(-0.5f,0.5f) { rng.seed(0);}

	glm::vec2 operator()() { return glm::vec2(d1(rng), d2(rng)); }
};

static JitterGenerator jg;

static glm::vec3 operator*(const glm::mat4 m, const glm::vec3 v)
{
	glm::vec4 r = m * glm::vec4(v, 1.f);
	return glm::vec3(r.x/r.w, r.y / r.w, r.z / r.w);
}

CameraFPS::CameraFPS()
{
	reset();
	nearplane = 0.1f;
	farplane = 100.f;
	rot_speed = 1.f;
	mov_speed = 1.f;
	moved = false;
	jitter_plane = glm::ivec2(0, 0);
	jitter       = glm::vec2(0.f, 0.f);
}

void CameraFPS::reset()
{
	position = glm::vec3(0.f, 0.f, 0.f);
	orientation = glm::normalize(glm::quat(1.f, 0.0f, 0.0f, 0.0f));
}

glm::vec3 CameraFPS::forward()
{
	return glm::normalize ( orientation * glm::vec3(0.f, 0.f,-1.f) );
}

glm::vec3 CameraFPS::right()
{
	return glm::normalize ( orientation * glm::vec3(1.f, 0.f, 0.f) );
}

glm::vec3 CameraFPS::up()
{
	return glm::normalize ( orientation * glm::vec3(0.f, 1.f, 0.f) );
}

glm::mat4 CameraFPS::proj_matrix()
{
	glm::mat4 p = glm::perspective(glm::radians(verticalFov_deg), aspect, nearplane, farplane);

	if (jitter_plane == glm::ivec2(0, 0)) return p;

	glm::vec2 j = jitter / glm::vec2(jitter_plane);

	glm::mat4 o = glm::ortho(-1.f + j.x, 1.f + j.x, -1.f + j.y, 1.f + j.y, 1.f, -1.f);

	return o * p;



	//return glm::ortho(0.f, 1.f, 0.f, 1.f, 0.f, 1.f) * p;
}

glm::mat4 CameraFPS::view_matrix()
{
	return glm::lookAt(position, position + forward(), up());
}

void CameraFPS::update(glm::vec2 rot, glm::vec3 motion)
{
	updateOrientation(rot);
	updatePosition(motion);
	updateJitter();
}

void CameraFPS::update()
{
	updateJitter();
}

void CameraFPS::updateJitter()
{
	jitter = jg();
}

void CameraFPS::updateOrientation(glm::vec2 rot)
{
	float pitch = -rot.y * rot_speed;
	float yaw   = -rot.x * rot_speed;

	glm::quat q_yaw   = glm::quat(glm::vec3(0.f  , yaw, 0.f));
	glm::quat q_pitch = glm::quat(glm::vec3(pitch, 0.f, 0.f));
	orientation = q_yaw * orientation * q_pitch;
}

void CameraFPS::updatePosition(glm::vec3 motion)
{
	motion *= mov_speed;
	position += right() * motion.x + up() * motion.y + forward() * motion.z;
}

void CameraFPS::getUVW(glm::vec3& U, glm::vec3& V, glm::vec3&W)
{
	// Should be this but it does not match
	//U = right()   * 0.5f  * atan(glm::radians(verticalFov_deg)) * aspect;
	//V = up()      * 0.5f  * atan(glm::radians(verticalFov_deg));
	//W = forward();

	glm::mat4 vMatrix = view_matrix();
	glm::mat4 pMatrix = proj_matrix();
	glm::mat4 vpInv   = glm::inverse(pMatrix * vMatrix);

	glm::vec3 centerNear = vpInv * glm::vec3(0.f, 0.f, 1.f);
	glm::vec3 rightNear  = vpInv * glm::vec3(1.f, 0.f, 1.f);
	glm::vec3 upNear     = vpInv * glm::vec3(0.f, 1.f, 1.f);

	U = rightNear - centerNear;
	V = upNear     - centerNear;
	W = centerNear - position;
}
