#include <common.h>
#include <helpers/OpenGLTextureArrayObject.h>
#include <memory>

//template class std::shared_ptr< GLHelpers::TextureArrayObject >;

namespace GLHelpers {

	void TextureArrayObject::Release()
	{
		if ( m_uiOpenGLID ) {
			glDeleteTextures( 1, &m_uiOpenGLID );
			m_uiOpenGLID = 0;
			check_opengl();
		}
	}

}
