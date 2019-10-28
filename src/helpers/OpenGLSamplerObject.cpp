#include "helpers/OpenGLSamplerObject.h"

namespace GLHelpers {

	SamplerObject::SamplerObject()
		: id(0)
	{}

	SamplerObject::~SamplerObject()
	{}

	void SamplerObject::Release()
	{
		if (id) glDeleteSamplers(1, &id);
	}

	void SamplerObject::Bind(GLuint textureUnit)
	{
		glBindSampler(textureUnit, id);
	}

	void SamplerObject::Unbind(GLuint textureUnit)
	{
		glBindSampler(textureUnit, 0);
	}

	void SamplerObject::create()
	{
		Release();
		glCreateSamplers(1, &id);
	}

	void SamplerObject::setInterpolationMethod(GLenum method)
	{
		glSamplerParameteri(id, GL_TEXTURE_MIN_FILTER, method);
		glSamplerParameteri(id, GL_TEXTURE_MAG_FILTER, method);
	}

	void SamplerObject::setInterpolationMethod(GLenum minMethod, GLenum magMethod)
	{
		glSamplerParameteri(id, GL_TEXTURE_MIN_FILTER, minMethod);
		glSamplerParameteri(id, GL_TEXTURE_MAG_FILTER, magMethod);
	}

	void SamplerObject::setWrapMethod(GLenum method)
	{
		glSamplerParameteri(id, GL_TEXTURE_WRAP_S, method);
		glSamplerParameteri(id, GL_TEXTURE_WRAP_T, method);
		glSamplerParameteri(id, GL_TEXTURE_WRAP_R, method);
	}

	void SamplerObject::setWrapMethod(GLenum methodS, GLenum methodT, GLenum methodR)
	{
		glSamplerParameteri(id, GL_TEXTURE_WRAP_S, methodS);
		glSamplerParameteri(id, GL_TEXTURE_WRAP_T, methodT);
		glSamplerParameteri(id, GL_TEXTURE_WRAP_R, methodR);
	}

	void SamplerObject::setParameter(GLenum paramName, GLint param)
	{
		glSamplerParameteri(id, paramName, param);
	}

	void SamplerObject::setParameterf(GLenum paramName, GLfloat param)
	{
		glSamplerParameterf(id, paramName, param);
	}

	GLint SamplerObject::getParameter(GLenum paramName)
	{
		GLint param;
		glGetSamplerParameteriv(id, paramName, &param);
		return param;
	}

	GLfloat SamplerObject::getParameterf(GLenum paramName)
	{
		GLfloat param;
		glGetSamplerParameterfv(id, paramName, &param);
		return param;
	}

}
