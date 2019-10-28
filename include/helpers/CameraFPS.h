#pragma once

#include "common.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class CameraFPS
{
public:
	CameraFPS();
	void reset();

	glm::vec3 forward();
	glm::vec3 right();
	glm::vec3 up();

	glm::mat4 view_matrix();
	glm::mat4 proj_matrix();

	void update(glm::vec2 rot, glm::vec3 motion);
	void update();
	void updateOrientation(glm::vec2 rot);
	void updatePosition(glm::vec3 motion);
	void updateJitter();

	void getUVW(glm::vec3& U, glm::vec3& V, glm::vec3&W);

	float verticalFov_deg;
	float aspect;
	float nearplane, farplane;
	float rot_speed;
	float mov_speed;
	glm::ivec2 jitter_plane;
	bool  moved;

	glm::vec3 position;
	glm::quat orientation;

	glm::vec2 jitter;
};