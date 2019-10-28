#pragma once

#include "common.h"

#include "helpers/OpenGLTextureObject.h"
#include "helpers/OpenGLFramebufferObject.h"

class GBufferGL
{
public:
	GBufferGL();
	int setup(unsigned int width, unsigned int height, bool create_mipmap_storage = false);
	void destroy();
	int attach(GLHelpers::FramebufferObject& fbo, int base_attachment = 0);

	bool valid;
	unsigned int width, height;
	GLHelpers::TextureObject2D   colortex;
	GLHelpers::TextureObject2D   postex;
	GLHelpers::TextureObject2D   normaltex;
	GLHelpers::TextureObject2D   texcoordtex;
	GLHelpers::TextureObject2D   motiontex;

	GLenum colortex_format;
	GLenum postex_format;
	GLenum normaltex_format;
	GLenum texcoordtex_format;
	GLenum motiontex_format;
};