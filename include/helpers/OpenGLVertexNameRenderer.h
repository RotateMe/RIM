#pragma once

#include "common.h"
#include "helpers/OpenGLTextureObject.h"
#include "helpers/OpenGLFramebufferObject.h"
#include "helpers/OpenGLBufferObject.h"
#include "helpers/OpenGLProgramObject.h"
#include "helpers/CameraFPS.h"

class OpenGLVertexNameRenderer
{
public: 

	void setup(size2D size);
	void render(size2D size, CameraFPS& cam, GLHelpers::BufferObject& vertex_buffer, float pointsize = 5.f);
	void destroy();

	void reload_programs();

	GLHelpers::FramebufferObject fbo;
	GLHelpers::TextureObject2D   nametex, depthtex;

private:

	void recreate_textures(size2D size);
	GLHelpers::ProgramObject     program;

};