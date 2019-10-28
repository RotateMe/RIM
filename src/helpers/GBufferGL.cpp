#include "helpers/GBufferGL.h"


GBufferGL::GBufferGL()
{
	width = 0; 
	height = 0;
	colortex_format    = GL_RGBA8;
	postex_format      = GL_RGBA16F;
	normaltex_format   = GL_RGBA16F;
	texcoordtex_format = GL_RG16F;
	motiontex_format   = GL_RGBA32F;
	valid = false;
}

int GBufferGL::setup(unsigned int width, unsigned int height, bool create_mipmap_storage)
{
	if (width == 0 || height == 0) return ERROR_INVALID_PARAMETER;

	if (colortex.width != width || colortex.height != height || colortex.internalFormat != colortex_format) {
		colortex.create(glm::ivec2(width, height), colortex_format, nullptr, create_mipmap_storage);
		colortex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	if (postex.width != width || postex.height != height || postex.internalFormat != postex_format) {
		postex.create(glm::ivec2(width, height), postex_format, nullptr, create_mipmap_storage);
		postex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	if (normaltex.width != width || normaltex.height != height || normaltex.internalFormat != normaltex_format) {
		normaltex.create(glm::ivec2(width, height), normaltex_format, nullptr, create_mipmap_storage);
		normaltex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	if (texcoordtex.width != width || texcoordtex.height != height || texcoordtex.internalFormat != texcoordtex_format) {
		texcoordtex.create(glm::ivec2(width, height), texcoordtex_format, nullptr, create_mipmap_storage);
		texcoordtex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	if (motiontex.width != width || motiontex.height != height || motiontex.internalFormat != motiontex_format) {
		motiontex.create(glm::ivec2(width, height), motiontex_format, nullptr, create_mipmap_storage);
		motiontex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	this->width  = width;
	this->height = height;

	valid = true;

	return SUCCESS;
}

void GBufferGL::destroy()
{
	colortex.Release();
	postex.Release();
	normaltex.Release();
	texcoordtex.Release();
	motiontex.Release();

	valid = false;
}

int GBufferGL::attach(GLHelpers::FramebufferObject& fbo, int base_attachment)
{
	if (!valid || !fbo.m_uiOpenGLID) return ERROR_INVALID_OPERATION;

	fbo.AttachColorTexture(colortex,    base_attachment++);
	fbo.AttachColorTexture(postex,      base_attachment++);
	fbo.AttachColorTexture(normaltex,   base_attachment++);
	fbo.AttachColorTexture(texcoordtex, base_attachment++);
	fbo.AttachColorTexture(motiontex,   base_attachment++);

	return SUCCESS;
}

