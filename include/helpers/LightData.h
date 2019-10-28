#pragma once

#include "common.h"
#include "helpers/OpenGLBufferObject.h"
#include "helpers/LightDataStruct.h"

#include <vector_types.h>

#include <glm/glm.hpp>
#include <vector>

namespace LightType {
	constexpr GLint LT_DIRECTIONAL   = 0;
	constexpr GLint LT_POINT         = 1;
	constexpr GLint LT_SPOT          = 2;
	constexpr GLint LT_SPHERE        = 3;
	constexpr GLint LT_PARALLELOGRAM = 4;
	constexpr GLint LT_POLYGON       = 5;
};



static_assert (sizeof(LightDataStructCUDA) == sizeof(LightDataStructGL), "LightDataStructCUDA and LightDataStructGL should have the same size");

void resetLightDataStruct(LightDataStructGL& l);

struct LightDataGL
{
	std::vector<LightDataStructGL> lightList;
	GLHelpers::BufferObject        lightListBuffer;

	glm::vec3 ambient_light;

};

namespace std {
	template<> struct hash<LightDataGL> {
		typedef LightDataGL argument_type;
		typedef std::size_t result_type;
		result_type operator()(argument_type const& l) const noexcept;
	};
}

struct LightData
{
	LightDataGL gl;
	std::hash<LightDataGL>::result_type hash;

	int loadFromJSON(std::string filename);
	int  clear();
	void setDirty(bool e = true);
	bool isDirty();

	int updateGLResources();

private:

	bool dirty;
	bool dirtygl;

	int releaseGLResources();
};