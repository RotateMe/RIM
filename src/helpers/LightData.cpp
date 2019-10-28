#include "common.h"
#include "helpers/LightData.h"

#include <json.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <functional>

int LightData::loadFromJSON(std::string filename)
{

	std::ifstream filestream(filename, std::ifstream::in);
	if (!filestream.is_open()) 
	{
		return ERROR_FILE_NOT_FOUND;
	}

	std::stringstream jsonstream;
	jsonstream << filestream.rdbuf();

	filestream.close();

	jsonreader::json j = jsonreader::json::parse(jsonstream.str());

	if (j.is_null()) {
		std::cerr << "Error: Empty light json file " << filename << std::endl;
		return ERROR_LOADING_RESOURCE;
	}
	
	jsonreader::json& ambient = j["Ambient"];
	if (ambient.is_array() && ambient.size() == 3 && ambient[0].is_number() && ambient[1].is_number() && ambient[2].is_number()) {
		gl.ambient_light = glm::vec3(ambient[0], ambient[1], ambient[2]);
	}

	jsonreader::json& lightsArray = j["Lights"];
	if (lightsArray.is_null() || !lightsArray.is_array()) {
		std::cerr << "Error: Lights element not present correctly in light json file " << filename << std::endl;
		return ERROR_LOADING_RESOURCE;
	}

	size_t lightQty = lightsArray.size();

	for (size_t i = 0; i < lightsArray.size(); ++i)
	{
		jsonreader::json& l = lightsArray[i];
		LightDataStructGL lgl;
		resetLightDataStruct(lgl);

		for (jsonreader::json::iterator it = l.begin(); it != l.end(); ++it) {

			std::string key = it.key();
			std::transform(key.begin(), key.end(), key.begin(), ::tolower);

			if (key == "type")
			{
				if (it.value().is_string()) {
					if (it.value() == "directional")    lgl.type = LightType::LT_DIRECTIONAL;
					if (it.value() == "point")          lgl.type = LightType::LT_POINT;
					if (it.value() == "spot")           lgl.type = LightType::LT_SPOT;
					if (it.value() == "sphere")         lgl.type = LightType::LT_SPHERE;
					if (it.value() == "parallelogram")  lgl.type = LightType::LT_PARALLELOGRAM;
					if (it.value() == "polygon")        lgl.type = LightType::LT_POLYGON;
				}
			}
			
			else if (key == "direction")
			{
				if (it.value().is_array() && it.value().size() == 3 && it.value()[0].is_number() && it.value()[1].is_number() && it.value()[2].is_number()) {
					lgl.direction = glm::normalize(glm::vec3(it.value()[0], it.value()[1], it.value()[2]));
				}
			}

			else if (key == "position")
			{
				if (it.value().is_array() && it.value().size() == 3 && it.value()[0].is_number() && it.value()[1].is_number() && it.value()[2].is_number()) {
					lgl.position = glm::vec3(it.value()[0], it.value()[1], it.value()[2]);
				}
			}

			else if (key == "color")
			{
				if (it.value().is_array() && it.value().size() == 3 && it.value()[0].is_number() && it.value()[1].is_number() && it.value()[2].is_number()) {
					lgl.color = glm::vec3(it.value()[0], it.value()[1], it.value()[2]);
				}
			}

			else if (key == "radius")
			{
				if (it.value().is_number()) {
					lgl.radius = it.value();
				}
			}

			else if (key == "intensity")
			{
				if (it.value().is_number()) {
					lgl.intensity = it.value();
				}
			}

			else if (key == "spotangle")
			{
				if (it.value().is_number()) {
					lgl.spot_angle = it.value();
				}
			}
		}

		gl.lightList.push_back(lgl);
	}
	
	dirty = false;
	dirtygl = true;

	int resultGL =  updateGLResources();
	if (resultGL != SUCCESS) return resultGL;

	return SUCCESS;
}

int LightData::clear()
{
	releaseGLResources();
	
	gl.lightList.clear();
	gl.lightList.shrink_to_fit();

	releaseGLResources();

	return SUCCESS;
}

void LightData::setDirty(bool e)
{
	if (e) dirtygl = true;
	dirty = e;
}

bool LightData::isDirty()
{
	return dirty;
}

std::hash<LightDataGL>::result_type std::hash<LightDataGL>::operator()(std::hash<LightDataGL>::argument_type const& l) const noexcept
{
	result_type h = 0;
	byte* hbyte = (byte*)&h;
	byte * data = (byte *)l.lightList.data();
	size_t data_size = l.lightList.size() * sizeof(LightDataStructGL);
	for (size_t i = 0; i < data_size; ++i) hbyte[i % sizeof(result_type)] ^= data[i];
	return h;
}


int LightData::updateGLResources()
{
	if (!gl.lightListBuffer.m_uiBufferIdx) {
		gl.lightListBuffer.Release();
		gl.lightListBuffer.Create(GL_SHADER_STORAGE_BUFFER);
		hash = 0;
		dirtygl = true;
	}

	if (!dirtygl) return SUCCESS;

	if (gl.lightList.empty())
	{
		std::cerr << "Warning: Creating light resources but no lights specified." << std::endl;
	}

	size_t bufferSize = gl.lightList.size() * sizeof(LightDataStructGL);
	std::hash<LightDataGL>::result_type newhash = std::hash<LightDataGL>{}(gl);

	if (bufferSize != gl.lightListBuffer.m_sizeInBytes || newhash != hash) {
		gl.lightListBuffer.UploadData(gl.lightList.size() * sizeof(LightDataStructGL), GL_STATIC_READ, gl.lightList.data());
		hash = newhash;
	}

	dirtygl = false;

	return SUCCESS;
}

int LightData::releaseGLResources()
{
	gl.lightListBuffer.Release();

	return SUCCESS;
}

void resetLightDataStruct(LightDataStructGL& l)
{
	l.type = LightType::LT_DIRECTIONAL;
	l.intensity = 1.f;
	l.color = glm::vec3(1.f, 1.f, 1.f);
	l.position = glm::vec3(0.f, 1, 0.f);
	l.direction = glm::vec3(0.f, -1.f, 0.f);
	l.spot_angle = 0.5f;
	l.radius = 0.1f;
	l.v1 = glm::vec3(-1.f, -1.f, 0.f);
	l.v2 = glm::vec3( 1.f, -1.f, 0.f);
	l.v3 = glm::vec3( 1.f,  1.f, 0.f);
	l.v4 = glm::vec3(-1.f,  1.f, 0.f);
}
