#pragma once

#include "common.h"

namespace GLHelpers {

	class TextureArrayObject
	{
	public:
		TextureArrayObject() :
			m_uiOpenGLID( 0 )
		{}

		virtual ~TextureArrayObject()
		{}

		GLuint					m_uiOpenGLID;
		virtual void			Release();
	};

}

