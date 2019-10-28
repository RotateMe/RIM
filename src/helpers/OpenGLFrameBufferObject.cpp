#include "helpers/OpenGLFrameBufferObject.h"
#include "helpers/OpenGLTextureObject.h"

#include <sstream>

namespace GLHelpers {


	bool CheckFrameBufferStatus( GLuint uiFBO, LoggerInterface* pLogger )
	{
		GLenum eStatus = glCheckFramebufferStatus( GL_FRAMEBUFFER );

		// early outs: success or no logger
		if ( eStatus == GL_FRAMEBUFFER_COMPLETE || pLogger == nullptr )
		{
			return ( eStatus == GL_FRAMEBUFFER_COMPLETE );
		}
		
		Assert( eStatus != GL_FRAMEBUFFER_COMPLETE );


				// find out what's wrong exactly
		std::string strError;
		switch( eStatus )
		{
			// In order of hex-codes
			case GL_FRAMEBUFFER_UNDEFINED:
				strError = "OpenGL Error: GL_FRAMEBUFFER_UNDEFINED\n"
						   "You are trying to access the default framebuffer, but it does not exist.\n";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
				strError = "OpenGL Error: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n"
						   "Your missed to bind an important attachment. Either for the drawbuffer or readbuffer.\n";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
				strError = "OpenGL Error: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n"
						   "At least one image must be attached to the drawbuffer or readbuffer.\n"
						   "Or OpenGL 4.3: GL_FRAMEBUFFER_DEFAULT_WIDTH and GL_FRAMEBUFFER_DEFAULT_HEIGHT must be greater than zero.\n";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
				strError = "OpenGL Error: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n"
						   "An attachment for the draw buffer is missing.\n";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
				strError = "OpenGL Error: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n"
						   "An attachment for the read buffer is missing.\n";
				break;
			case GL_FRAMEBUFFER_UNSUPPORTED:
				strError = "OpenGL Error: GL_FRAMEBUFFER_UNSUPPORTED\n"
						   "The framebuffer format you set up is not supported.\n";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
				strError = "OpenGL Error: GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE\n"
						   "One of your images has different multisample settings.\n";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
				strError = "OpenGL Error: GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS\n"
						   "You bound a layered target, then all targets must be layered.\n";
				break;
			default:
				Assert( 0 );
				{
					std::stringstream ss;
					ss << "OpenGL Error: Unknown Error: 0x";
					ss << std::hex;
					ss << std::uppercase;
					ss << eStatus;
					strError = ss.str();
				}
				break;
		}

		if (pLogger) pLogger->LogError( strError );
		return false;
	}

	FramebufferObject::FramebufferObject()
	{
		m_uiOpenGLID = 0;
	}

	void FramebufferObject::Create()
	{
		Delete();
		if (glCreateFramebuffers != nullptr) glCreateFramebuffers(1, &m_uiOpenGLID);
		else                                 glGenFramebuffers(1, &m_uiOpenGLID);

		check_opengl();
	}

	void FramebufferObject::Delete()
	{
		if (m_uiOpenGLID) 
		{
			glDeleteFramebuffers(1, &m_uiOpenGLID);
			m_uiOpenGLID = 0;
		}

		check_opengl();
	}

	void FramebufferObject::Bind(GLenum target) const
	{
		switch (target)
		{
		case GL_READ_FRAMEBUFFER:
			glBindFramebuffer(GL_READ_FRAMEBUFFER, m_uiOpenGLID);
			return;
		case GL_DRAW_FRAMEBUFFER:
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_uiOpenGLID);
			return;
		default:
			glBindFramebuffer(GL_FRAMEBUFFER, m_uiOpenGLID);
			return;
		}
	}
	
	void FramebufferObject::Unbind(GLenum target) 
	{
		switch (target)
		{
		case GL_READ_FRAMEBUFFER:
			glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
			return;
		case GL_DRAW_FRAMEBUFFER:
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
			return;
		default:
			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			return;
		}	
	}

	void FramebufferObject::AttachTexture(TextureObject& tex, GLenum framebufferAttachment, GLint texLevel, GLenum framebufferTarget) const
	{
#if OPENGL_BINDLESS
		glNamedFramebufferTexture(m_uiOpenGLID, framebufferAttachment, tex.m_uiOpenGLID, texLevel);
#else
		Bind(framebufferTarget);
		glFramebufferTexture(framebufferTarget, framebufferAttachment, tex.m_uiOpenGLID, texLevel);
#endif
		check_opengl();
	}

	void FramebufferObject::AttachStencilTexture(TextureObject& tex, GLint texLevel, GLenum framebufferTarget) const
	{
		AttachTexture(tex, GL_STENCIL_ATTACHMENT, texLevel, framebufferTarget);
	}

	void FramebufferObject::AttachDepthStencilTexture(TextureObject& tex, GLint texLevel, GLenum framebufferTarget) const
	{
		AttachTexture(tex, GL_DEPTH_STENCIL_ATTACHMENT, texLevel, framebufferTarget);
	}

	void FramebufferObject::AttachDepthTexture(TextureObject& tex, GLint texLevel, GLenum framebufferTarget) const
	{
		AttachTexture(tex, GL_DEPTH_ATTACHMENT, texLevel, framebufferTarget);
	}
	
	void FramebufferObject::AttachColorTexture(TextureObject& tex, int colorAttachmentIndex, GLint texLevel, GLenum framebufferTarget) const
	{
		AttachTexture(tex, GL_COLOR_ATTACHMENT0 + colorAttachmentIndex, texLevel, framebufferTarget);
	}

	void FramebufferObject::AttachCubemapFace(TextureObjectCube& tex, GLenum texFaceTarget, GLenum framebufferAttachment, GLint texLevel, GLenum framebufferTarget) const
	{
		Bind(framebufferTarget);
		glFramebufferTexture2D(framebufferTarget, framebufferAttachment, texFaceTarget, tex.m_uiOpenGLID, texLevel);
		check_opengl();
	}

	void FramebufferObject::AttachTexture2DLayer(TextureObject2DArray& tex, GLenum framebufferAttachment, GLint texLayer, GLint texlevel, GLenum framebufferTarget) const
	{
		Bind(framebufferTarget);
		glFramebufferTextureLayer(framebufferTarget, framebufferAttachment, tex.m_uiOpenGLID, texlevel, texLayer);
		check_opengl();
	}

	void FramebufferObject::AttachTextureCubeLayer(TextureObjectCubeArray& tex, GLenum framebufferAttachment, GLint texLayer, GLint texlevel, GLenum framebufferTarget) const
	{
		Bind(framebufferTarget);
		glFramebufferTextureLayer(framebufferTarget, framebufferAttachment, tex.m_uiOpenGLID, texlevel, texLayer);
		check_opengl();
	}

	static const GLenum c_colorAttachments[] =
	{
		GL_COLOR_ATTACHMENT0,
		GL_COLOR_ATTACHMENT0 + 1,
		GL_COLOR_ATTACHMENT0 + 2,
		GL_COLOR_ATTACHMENT0 + 3,
		GL_COLOR_ATTACHMENT0 + 4,
		GL_COLOR_ATTACHMENT0 + 5,
		GL_COLOR_ATTACHMENT0 + 6,
		GL_COLOR_ATTACHMENT0 + 7,
		GL_COLOR_ATTACHMENT0 + 8,
		GL_COLOR_ATTACHMENT0 + 9,
		GL_COLOR_ATTACHMENT0 + 10,
		GL_COLOR_ATTACHMENT0 + 11,
		GL_COLOR_ATTACHMENT0 + 12,
		GL_COLOR_ATTACHMENT0 + 13,
		GL_COLOR_ATTACHMENT0 + 14,
		GL_COLOR_ATTACHMENT0 + 15
	};

	void FramebufferObject::EnableConsecutiveDrawbuffers(GLuint drawbufferQty, GLuint startAttachmentIndex, GLenum framebufferTarget) const
	{
	#if OPENGL_BINDLESS
		glNamedFramebufferDrawBuffers(m_uiOpenGLID, drawbufferQty, c_colorAttachments + startAttachmentIndex);
	#else
		Bind();
		glDrawBuffers(drawbufferQty, c_colorAttachments + startAttachmentIndex);
	#endif
		check_opengl();
	}



}
