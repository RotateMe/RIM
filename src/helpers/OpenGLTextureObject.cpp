#include "helpers/OpenGLTextureObject.h"
#include "helpers/OpenGLProgramObject.h"
#include "helpers/OpenGLTexture.h"

#include <map>
#include <iostream>
#include <algorithm>


namespace GLHelpers {


	std::map<GLenum, GLenum> GL_DATATYPES;

	static void init_datatypes()
	{
		GL_DATATYPES[GL_R8] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_R8_SNORM] = GL_BYTE;
		//GL_DATATYPES[GL_R16] =		        16
		//GL_DATATYPES[GL_R16_SNORM] =        	s16
		GL_DATATYPES[GL_RG8] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_RG8_SNORM] = GL_BYTE;
		//GL_DATATYPES[GL_RG16] =		        16
		//GL_DATATYPES[GL_RG16_SNORM] =	        s16
		//GL_DATATYPES[GL_R3_G3_B2] =        	3	3	2
		//GL_DATATYPES[GL_RGB4] =		        4
		//GL_DATATYPES[GL_RGB5] =		        5
		GL_DATATYPES[GL_RGB8] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_RGB8_SNORM] = GL_BYTE;
		//GL_DATATYPES[GL_RGB10] =		        10
		//GL_DATATYPES[GL_RGB12] =		        12
		//GL_DATATYPES[GL_RGB16_SNORM] =        16
		//GL_DATATYPES[GL_RGBA2] =		        2
		//GL_DATATYPES[GL_RGBA4] =		        4
		//GL_DATATYPES[GL_RGB5_A1] =	        5
		GL_DATATYPES[GL_RGBA8] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_RGBA8_SNORM] = GL_BYTE;
		GL_DATATYPES[GL_RGB10_A2] = GL_UNSIGNED_INT_2_10_10_10_REV;
		GL_DATATYPES[GL_RGB10_A2UI] = GL_UNSIGNED_INT_2_10_10_10_REV;
		//GL_DATATYPES[GL_RGBA12] =		        12
		//GL_DATATYPES[GL_RGBA16] =		        16
		GL_DATATYPES[GL_SRGB8] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_SRGB8_ALPHA8] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_R16F] = GL_FLOAT /*GL_HALF_FLOAT*/;
		GL_DATATYPES[GL_RG16F] = GL_FLOAT /*GL_HALF_FLOAT*/;
		GL_DATATYPES[GL_RGB16F] = GL_FLOAT /*GL_HALF_FLOAT*/;
		GL_DATATYPES[GL_RGBA16F] = GL_FLOAT /*GL_HALF_FLOAT*/;
		GL_DATATYPES[GL_R32F] = GL_FLOAT;
		GL_DATATYPES[GL_RG32F] = GL_FLOAT;
		GL_DATATYPES[GL_RGB32F] = GL_FLOAT;
		GL_DATATYPES[GL_RGBA32F] = GL_FLOAT;
		GL_DATATYPES[GL_R11F_G11F_B10F] = GL_UNSIGNED_INT_10F_11F_11F_REV;
		GL_DATATYPES[GL_RGB9_E5] = GL_UNSIGNED_INT_5_9_9_9_REV;
		GL_DATATYPES[GL_R8I] = GL_BYTE;
		GL_DATATYPES[GL_R8UI] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_R16I] = GL_SHORT;
		GL_DATATYPES[GL_R16UI] = GL_UNSIGNED_SHORT;
		GL_DATATYPES[GL_R32I] = GL_INT;
		GL_DATATYPES[GL_R32UI] = GL_UNSIGNED_INT;
		GL_DATATYPES[GL_RG8I] = GL_BYTE;
		GL_DATATYPES[GL_RG8UI] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_RG16I] = GL_SHORT;
		GL_DATATYPES[GL_RG16UI] = GL_UNSIGNED_SHORT;
		GL_DATATYPES[GL_RG32I] = GL_INT;
		GL_DATATYPES[GL_RG32UI] = GL_UNSIGNED_INT;
		GL_DATATYPES[GL_RGB8I] = GL_BYTE;
		GL_DATATYPES[GL_RGB8UI] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_RGB16I] = GL_SHORT;
		GL_DATATYPES[GL_RGB16UI] = GL_UNSIGNED_SHORT;
		GL_DATATYPES[GL_RGB32I] = GL_INT;
		GL_DATATYPES[GL_RGB32UI] = GL_UNSIGNED_INT;
		GL_DATATYPES[GL_RGBA8I] = GL_BYTE;
		GL_DATATYPES[GL_RGBA8UI] = GL_UNSIGNED_BYTE;
		GL_DATATYPES[GL_RGBA16I] = GL_SHORT;
		GL_DATATYPES[GL_RGBA16UI] = GL_UNSIGNED_SHORT;
		GL_DATATYPES[GL_RGBA32I] = GL_INT;
		GL_DATATYPES[GL_RGBA32UI] = GL_UNSIGNED_INT;
		GL_DATATYPES[GL_DEPTH_COMPONENT16] = GL_UNSIGNED_INT;
		GL_DATATYPES[GL_DEPTH_COMPONENT24] = GL_UNSIGNED_INT;
		GL_DATATYPES[GL_DEPTH_COMPONENT32F] = GL_FLOAT;
		GL_DATATYPES[GL_DEPTH24_STENCIL8] = GL_UNSIGNED_INT_24_8;
		GL_DATATYPES[GL_DEPTH32F_STENCIL8] = GL_FLOAT_32_UNSIGNED_INT_24_8_REV;

	};



	// association of base formats and component count
	std::map<GLenum, uint8_t> GL_COMPONENTS;

	static void init_components()
	{
		GL_COMPONENTS[GL_RED] = 1;
		GL_COMPONENTS[GL_RED_INTEGER] = 1;
		GL_COMPONENTS[GL_GREEN] = 1;
		GL_COMPONENTS[GL_BLUE] = 1;
		GL_COMPONENTS[GL_RG] = 2;
		GL_COMPONENTS[GL_RG_INTEGER] = 2;
		GL_COMPONENTS[GL_RGB] = 3;
		GL_COMPONENTS[GL_RGB_INTEGER] = 3;
		GL_COMPONENTS[GL_BGR] = 3;
		GL_COMPONENTS[GL_RGBA] = 4;
		GL_COMPONENTS[GL_RGBA_INTEGER] = 4;
		GL_COMPONENTS[GL_BGRA] = 4;
		GL_COMPONENTS[GL_STENCIL_INDEX] = 1;
		GL_COMPONENTS[GL_DEPTH_COMPONENT] = 1;
		GL_COMPONENTS[GL_DEPTH_STENCIL] = 2;

	}

	// association of base formats and internal formats
	std::map<GLenum, GLenum> GL_DATAFORMATS;
	static void init_dataformats()
	{
		GL_DATAFORMATS[GL_RED] = GL_RED;
		GL_DATAFORMATS[GL_GREEN] = GL_RED;
		GL_DATAFORMATS[GL_BLUE] = GL_RED;
		GL_DATAFORMATS[GL_R8] = GL_RED;
		GL_DATAFORMATS[GL_R8_SNORM] = GL_RED;
		GL_DATAFORMATS[GL_R16] = GL_RED;
		GL_DATAFORMATS[GL_R16_SNORM] = GL_RED;
		GL_DATAFORMATS[GL_R16F] = GL_RED;
		GL_DATAFORMATS[GL_R32F] = GL_RED;
		GL_DATAFORMATS[GL_R8I] = GL_RED_INTEGER;
		GL_DATAFORMATS[GL_R8UI] = GL_RED_INTEGER;
		GL_DATAFORMATS[GL_R16I] = GL_RED_INTEGER;
		GL_DATAFORMATS[GL_R16UI] = GL_RED_INTEGER;
		GL_DATAFORMATS[GL_R32I] = GL_RED_INTEGER;
		GL_DATAFORMATS[GL_R32UI] = GL_RED_INTEGER;
		GL_DATAFORMATS[GL_COMPRESSED_RED] = GL_RED;
		GL_DATAFORMATS[GL_COMPRESSED_RED_RGTC1] = GL_RED;
		GL_DATAFORMATS[GL_COMPRESSED_SIGNED_RED_RGTC1] = GL_RED;

		GL_DATAFORMATS[GL_RG] = GL_RG;
		GL_DATAFORMATS[GL_RG8] = GL_RG;
		GL_DATAFORMATS[GL_RG8_SNORM] = GL_RG;
		GL_DATAFORMATS[GL_RG16] = GL_RG;
		GL_DATAFORMATS[GL_RG16_SNORM] = GL_RG;
		GL_DATAFORMATS[GL_RG16F] = GL_RG;
		GL_DATAFORMATS[GL_RG32F] = GL_RG;
		GL_DATAFORMATS[GL_RG8I] = GL_RG_INTEGER;
		GL_DATAFORMATS[GL_RG8UI] = GL_RG_INTEGER;
		GL_DATAFORMATS[GL_RG16I] = GL_RG_INTEGER;
		GL_DATAFORMATS[GL_RG16UI] = GL_RG_INTEGER;
		GL_DATAFORMATS[GL_RG32I] = GL_RG_INTEGER;
		GL_DATAFORMATS[GL_RG32UI] = GL_RG_INTEGER;
		GL_DATAFORMATS[GL_COMPRESSED_RG] = GL_RG;
		GL_DATAFORMATS[GL_COMPRESSED_RG_RGTC2] = GL_RG;
		GL_DATAFORMATS[GL_COMPRESSED_SIGNED_RG_RGTC2] = GL_RG;

		GL_DATAFORMATS[GL_RGB] = GL_RGB;
		GL_DATAFORMATS[GL_R3_G3_B2] = GL_RGB;
		GL_DATAFORMATS[GL_RGB4] = GL_RGB;
		GL_DATAFORMATS[GL_RGB5] = GL_RGB;
		GL_DATAFORMATS[GL_RGB8] = GL_RGB;
		GL_DATAFORMATS[GL_RGB8_SNORM] = GL_RGB;
		GL_DATAFORMATS[GL_RGB10] = GL_RGB;
		GL_DATAFORMATS[GL_RGB12] = GL_RGB;
		GL_DATAFORMATS[GL_RGB16_SNORM] = GL_RGB;
		GL_DATAFORMATS[GL_RGBA2] = GL_RGB;
		GL_DATAFORMATS[GL_RGBA4] = GL_RGB;
		GL_DATAFORMATS[GL_SRGB8] = GL_RGB;
		GL_DATAFORMATS[GL_RGB16F] = GL_RGB;
		GL_DATAFORMATS[GL_RGB32F] = GL_RGB;
		GL_DATAFORMATS[GL_R11F_G11F_B10F] = GL_RGB;
		GL_DATAFORMATS[GL_RGB9_E5] = GL_RGB;
		GL_DATAFORMATS[GL_RGB8I] = GL_RGB_INTEGER;
		GL_DATAFORMATS[GL_RGB8UI] = GL_RGB_INTEGER;
		GL_DATAFORMATS[GL_RGB16I] = GL_RGB_INTEGER;
		GL_DATAFORMATS[GL_RGB16UI] = GL_RGB_INTEGER;
		GL_DATAFORMATS[GL_RGB32I] = GL_RGB_INTEGER;
		GL_DATAFORMATS[GL_RGB32UI] = GL_RGB_INTEGER;
		GL_DATAFORMATS[GL_COMPRESSED_RGB] = GL_RGB;
		GL_DATAFORMATS[GL_COMPRESSED_SRGB] = GL_RGB;
		GL_DATAFORMATS[GL_COMPRESSED_RGB_BPTC_SIGNED_FLOAT] = GL_RGB;
		GL_DATAFORMATS[GL_COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT] = GL_RGB;

		GL_DATAFORMATS[GL_RGBA] = GL_RGBA;
		GL_DATAFORMATS[GL_RGB5_A1] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA8] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA8_SNORM] = GL_RGBA;
		GL_DATAFORMATS[GL_RGB10_A2] = GL_RGBA;
		GL_DATAFORMATS[GL_RGB10_A2UI] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA12] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA16] = GL_RGBA;
		GL_DATAFORMATS[GL_SRGB8_ALPHA8] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA16F] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA32F] = GL_RGBA;
		GL_DATAFORMATS[GL_RGBA8I] = GL_RGBA_INTEGER;
		GL_DATAFORMATS[GL_RGBA8UI] = GL_RGBA_INTEGER;
		GL_DATAFORMATS[GL_RGBA16I] = GL_RGBA_INTEGER;
		GL_DATAFORMATS[GL_RGBA16UI] = GL_RGBA_INTEGER;
		GL_DATAFORMATS[GL_RGBA32I] = GL_RGBA_INTEGER;
		GL_DATAFORMATS[GL_RGBA32UI] = GL_RGBA_INTEGER;
		GL_DATAFORMATS[GL_COMPRESSED_RGBA] = GL_RGBA;
		GL_DATAFORMATS[GL_COMPRESSED_SRGB_ALPHA] = GL_RGBA;
		GL_DATAFORMATS[GL_COMPRESSED_RGBA_BPTC_UNORM] = GL_RGBA;
		GL_DATAFORMATS[GL_COMPRESSED_SRGB_ALPHA_BPTC_UNORM] = GL_RGBA;

		GL_DATAFORMATS[GL_STENCIL_INDEX] = GL_STENCIL_INDEX;
		GL_DATAFORMATS[GL_STENCIL_INDEX1] = GL_STENCIL_INDEX;
		GL_DATAFORMATS[GL_STENCIL_INDEX4] = GL_STENCIL_INDEX;
		GL_DATAFORMATS[GL_STENCIL_INDEX8] = GL_STENCIL_INDEX;
		GL_DATAFORMATS[GL_STENCIL_INDEX16] = GL_STENCIL_INDEX;

		GL_DATAFORMATS[GL_DEPTH_COMPONENT] = GL_DEPTH_COMPONENT;
		GL_DATAFORMATS[GL_DEPTH_COMPONENT16] = GL_DEPTH_COMPONENT;
		GL_DATAFORMATS[GL_DEPTH_COMPONENT24] = GL_DEPTH_COMPONENT;
		GL_DATAFORMATS[GL_DEPTH_COMPONENT32F] = GL_DEPTH_COMPONENT;

		GL_DATAFORMATS[GL_DEPTH_STENCIL] = GL_DEPTH_STENCIL;
		GL_DATAFORMATS[GL_DEPTH24_STENCIL8] = GL_DEPTH_STENCIL;
		GL_DATAFORMATS[GL_DEPTH32F_STENCIL8] = GL_DEPTH_STENCIL;
	}

	std::set<GLenum> GL_COMPRESSED_FORMATS;
	static void init_compressed_dataformats()
	{
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RED);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RED_RGTC1);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_SIGNED_RED_RGTC1);

		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RG);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RG_RGTC2);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_SIGNED_RG_RGTC2);

		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RGB);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_SRGB);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RGB_BPTC_SIGNED_FLOAT);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT);

		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RGBA);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_SRGB_ALPHA);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_RGBA_BPTC_UNORM);
		GL_COMPRESSED_FORMATS.insert(GL_COMPRESSED_SRGB_ALPHA_BPTC_UNORM);
	}



	static unsigned int getBitsPerChannel(GLenum internalFormat)
	{
		switch (internalFormat) {
		case GL_R32F:
		case GL_RG32F:
		case GL_RGB32F:
		case GL_RGBA32F:
		case GL_DEPTH_COMPONENT32F:
		case GL_DEPTH_COMPONENT32:
			return 32;
		case GL_DEPTH_COMPONENT24:
			return 24;
		case GL_R16F:
		case GL_RG16F:
		case GL_RGB16F:
		case GL_RGBA16F:
		case GL_DEPTH_COMPONENT16:
			return 16;
		default:
			return 8;
		}
	}

	static GLenum getDataType(GLenum internalFormat)
	{
		if (GL_DATATYPES.empty()) init_datatypes();
		return GL_DATATYPES[internalFormat];
	}

	static GLenum getBaseFormat(GLenum internalFormat)
	{
		if (GL_DATAFORMATS.empty()) init_dataformats();
		return GL_DATAFORMATS[internalFormat];
	}

	static unsigned int getFormatComponentCount(GLenum baseFormat)
	{
		if (GL_COMPONENTS.empty()) init_components();
		return GL_COMPONENTS[baseFormat];
	}

	static bool isCompressedFormat(GLenum internalFormat)
	{
		if (GL_COMPRESSED_FORMATS.empty()) init_compressed_dataformats();
		return GL_COMPRESSED_FORMATS.find(internalFormat) != GL_COMPRESSED_FORMATS.end();
	}

	static GLenum getAttachment(GLenum baseFormat)
	{
		switch (baseFormat) {
		case GL_STENCIL_INDEX:   return GL_STENCIL_ATTACHMENT;
		case GL_DEPTH_COMPONENT: return GL_DEPTH_ATTACHMENT;
		case GL_DEPTH_STENCIL:   return GL_DEPTH_STENCIL_ATTACHMENT;
		default:                 return GL_COLOR_ATTACHMENT0;
		}
	}

	static void _create(GLuint* outputTextureName, GLenum textureTarget)
	{
#if OPENGL_BINDLESS
		glCreateTextures(textureTarget, 1, outputTextureName);
#else
		glGenTextures(1, outputTextureName);
#endif
		Assert(*outputTextureName);

		check_opengl();
	}

	static void _bind(GLuint textureName, GLint textureUnit, GLenum textureTarget)
	{
		if (!textureName) return;

#if OPENGL_BINDLESS
		if (textureUnit < 0) {
			glGetIntegerv(GL_ACTIVE_TEXTURE, &textureUnit);
			textureUnit -= GL_TEXTURE0;
		}
		glBindTextureUnit(textureUnit, textureName);
#else
		if (textureUnit >= 0) glActiveTexture(GL_TEXTURE0 + textureUnit);
		glBindTexture(textureTarget, textureName);
#endif

		check_opengl();
	}

	static void _unbind(GLint textureUnit, GLenum textureTarget)
	{
#if OPENGL_BINDLESS
		if (textureUnit < 0) {
			glGetIntegerv(GL_ACTIVE_TEXTURE, &textureUnit);
			textureUnit -= GL_TEXTURE0;
		}
		glBindTextureUnit(textureUnit, 0);
#else
		if (textureUnit >= 0) glActiveTexture(GL_TEXTURE0 + textureUnit);
		glBindTexture(textureTarget, 0);
#endif

		check_opengl();
	}

	static void _release(GLuint& textureName)
	{
		if (textureName) {
			glDeleteTextures(1, &textureName);
			textureName = 0;
			check_opengl();
		}
	}

	///////////////////////////////////////////////////////////
	/////////////// TextureObject

	TextureObject::TextureObject()
		: m_uiOpenGLID(0)
		, internalFormat(GL_INVALID_ENUM)
		, dataType(GL_INVALID_ENUM)
		, dataFormat(GL_INVALID_ENUM)
		, width(0)
		, height(0)
		, depth(0)
		, channels(0)
		, bpp(0)
		, textureTarget(GL_INVALID_ENUM)
		, mipmapLevels(1)
	{}

	TextureObject::~TextureObject()
	{}

	void TextureObject::Release()
	{
		_release(m_uiOpenGLID);
		internalFormat = GL_INVALID_ENUM;
		dataType = GL_INVALID_ENUM;
		dataFormat = GL_INVALID_ENUM;
		width = 0;
		height = 0;
		depth = 0;
		channels = 0;
		bpp = 0;
		mipmapLevels = 1;
	}

	void TextureObject::Bind(GLint textureUnit) const
	{
		_bind(m_uiOpenGLID, textureUnit, textureTarget);
	}

	void TextureObject::Unbind(GLint textureUnit) const
	{
		_unbind(textureUnit, textureTarget);
	}

	void TextureObject::create(glm::ivec2 size, GLenum internalFormat, void* data, bool createMipmapStorage, bool mutableStorage)
	{
		create(glm::ivec3(size.x, size.y, 1), internalFormat, data, createMipmapStorage, mutableStorage);
	}

	void TextureObject::create(glm::ivec3 size, GLenum internalFormat, void* data, bool createMipmapStorage, bool mutableStorage)
	{
		Release();

		width = size.x;
		height = size.y;
		depth = size.z;
		setupTextureFormat(internalFormat);
		bpp = getBitsPerChannel(internalFormat) * channels;

		_create(&m_uiOpenGLID, textureTarget);

		initializeTextureMemory(data, createMipmapStorage, mutableStorage);
	}

	//void TextureObject::setInternalFormat(int numChannels, unsigned int bytesperppixel) {

	//	// set format
	//	m_bpp = bytesperppixel;
	//	m_channels = numChannels;
	//	m_internalFormat = getInternalFormat();
	//	m_dataType = getDataType(m_internalFormat);				// data type from internal tex format
	//	m_dataFormat = getBaseFormat(m_internalFormat);			// data format from internal tex format

	//}

	void TextureObject::setupTextureFormat(GLenum textureFormat) {

		internalFormat = textureFormat;
		dataType = getDataType(internalFormat);				// data type from internal tex format
		dataFormat = getBaseFormat(internalFormat);			    // data format from internal tex format
		channels = getFormatComponentCount(dataFormat);		// number of channels from data format
		bpp = getBitsPerChannel(internalFormat) * channels;
	}

	glm::ivec2 TextureObject::size()
	{
		return glm::ivec2(width, height);
	}

	glm::ivec3 TextureObject::size3()
	{
		return glm::ivec3(width, height, depth);
	}

	void TextureObject::setInterpolationMethod(GLenum method)
	{
#if OPENGL_BINDLESS
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MIN_FILTER, method);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MAG_FILTER, method);
#else
		Bind();
		glTexParameteri(textureTarget, GL_TEXTURE_MIN_FILTER, method);
		glTexParameteri(textureTarget, GL_TEXTURE_MAG_FILTER, method);
#endif
	}

	void TextureObject::setInterpolationMethod(GLenum minMethod, GLenum magMethod)
	{
#if OPENGL_BINDLESS
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MIN_FILTER, minMethod);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MAG_FILTER, magMethod);
#else
		Bind();
		glTexParameteri(textureTarget, GL_TEXTURE_MIN_FILTER, minMethod);
		glTexParameteri(textureTarget, GL_TEXTURE_MAG_FILTER, magMethod);
#endif
	}

	void TextureObject::setWrapMethod(GLenum method)
	{
#if OPENGL_BINDLESS
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_WRAP_S, method);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_WRAP_T, method);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_WRAP_R, method);
#else
		Bind();
		glTexParameteri(textureTarget, GL_TEXTURE_WRAP_S, method);
		glTexParameteri(textureTarget, GL_TEXTURE_WRAP_T, method);
		glTexParameteri(textureTarget, GL_TEXTURE_WRAP_R, method);
#endif
	}

	void TextureObject::setWrapMethod(GLenum methodS, GLenum methodT, GLenum methodR)
	{
#if OPENGL_BINDLESS
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_WRAP_S, methodS);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_WRAP_T, methodT);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_WRAP_R, methodR);
#else
		Bind();
		glTexParameteri(textureTarget, GL_TEXTURE_WRAP_S, methodS);
		glTexParameteri(textureTarget, GL_TEXTURE_WRAP_T, methodT);
		glTexParameteri(textureTarget, GL_TEXTURE_WRAP_R, methodR);
#endif
	}

	void TextureObject::setParameter(GLenum paramName, GLint param)
	{
#if OPENGL_BINDLESS
		glTextureParameteri(m_uiOpenGLID, paramName, param);
#else
		Bind();
		glTexParameteri(textureTarget, paramName, param);
#endif
	}

	GLint TextureObject::getParameter(GLenum paramName)
	{
		GLint param;
#if OPENGL_BINDLESS
		glGetTextureParameteriv(m_uiOpenGLID, paramName, &param);
#else
		Bind();
		glGetTexParameteriv(textureTarget, paramName, &param);
#endif
		return param;
	}

	GLuint64 TextureObject::getHandle()
	{
		return glGetTextureHandleARB ? glGetTextureHandleARB(m_uiOpenGLID) : 0;
	}

	bool  TextureObject::isResident()
	{
		GLuint64 h = getHandle();
		if (h && glIsTextureHandleResidentARB && glIsTextureHandleResidentARB) {
			return glIsTextureHandleResidentARB(h);
		}

		return false;
	}

	void  TextureObject::makeResident()
	{
		GLuint64 h = getHandle();
		if (glMakeTextureHandleResidentARB && !isResident()) glMakeTextureHandleResidentARB(h);
	}

	void  TextureObject::makeNotResident()
	{
		GLuint64 h = getHandle();
		if (glMakeTextureHandleNonResidentARB && isResident()) glMakeTextureHandleNonResidentARB(h);
	}

	void TextureObject::generateMipmap()
	{
		if (mipmapLevels == 1) 
		{
			std::cout << "Warning: attempting to generate mipmap for texture created with no mipmap storage\n";
			return;
		}

#if OPENGL_BINDLESS
		glGenerateTextureMipmap( m_uiOpenGLID );
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#else
		Bind();
		glGenerateMipmap( textureTarget );
		glTexParameteri(textureTarget, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(textureTarget, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#endif

	}

	void TextureObject::generateMipmap(GLint maxLevel, GLenum minMethod, GLenum magMethod) 
	{

		if (maxLevel == 1)
		{
			std::cout << "Warning: attempting to generate mipmap for texture created with no mipmap storage\n";
			return;
		}

		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_BASE_LEVEL, 0);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MAX_LEVEL, maxLevel);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MIN_FILTER, minMethod);
		glTextureParameteri(m_uiOpenGLID, GL_TEXTURE_MAG_FILTER, magMethod);

#if OPENGL_BINDLESS
		glGenerateTextureMipmap( m_uiOpenGLID );
#else
		Bind();
		glGenerateMipmap( textureTarget );
#endif
	}


	///////////////////////////////////////////////////////////
	/////////////// TextureObject1D

	TextureObject1D::TextureObject1D() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_1D;
	}

	TextureObject1D::~TextureObject1D()
	{}

	void TextureObject1D::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(width)) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: use initializeTextureMemoryCompressed for compressed formats");

#if OPENGL_BINDLESS
		if (mutableStorage) {
			for (GLuint level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				glTextureImage1DEXT(m_uiOpenGLID, textureTarget, level, internalFormat, levelWidth, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
				glTextureStorage1D( m_uiOpenGLID, mipmapLevels, internalFormat, width);
				if (data) glTextureSubImage1D(m_uiOpenGLID, 0, 0, width, dataFormat, dataType, data);
		}
#else
		Bind();
		if (mutableStorage) {
			for (int level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				glTexImage1D(textureTarget, level, internalFormat, levelWidth, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
			glTexStorage1D(textureTarget, mipmapLevels, internalFormat, width);
			glTexSubImage1D(textureTarget, 0, 0, width, dataFormat, dataType, data);
		}
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		height = 1;
		depth  = 1;

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();

	}

	void TextureObject1D::initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage)
	{
		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(width)) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (!compressed) Assert(!"Error: initializeTextureMemory non-compressed formats");

#if OPENGL_BINDLESS
		glCompressedTextureImage1DEXT( m_uiOpenGLID, GL_TEXTURE_1D, 0, internalFormat, width, 0, dataSize, data);
#else
		Bind();
		glCompressedTexImage1D( GL_TEXTURE_1D, 0, internalFormat, width, 0, dataSize, data);
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		height = 1;
		depth  = 1;

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();

	}


	void TextureObject1D::uploadTextureData(const void* data) 
	{
#if OPENGL_BINDLESS
		glTextureSubImage1D(m_uiOpenGLID, 0, 0, width, dataFormat, dataType, data);
#else
		Bind();
		glTexSubImage1D(textureTarget, 0, 0, width, dataFormat, dataType, data);
#endif
	}

	void TextureObject1D::loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage)
	{
		Release();
		
		// Assert( img.m_sNumberOfMipMapLevels == 1 && "Todo: load all mipmap levels" );

		setupTextureFormat(GFXImageFormatToOpenGL(img.m_eFormat));

		mipmapLevels = bCreateMipMapStorage ? (GLuint) floor(log2(width)) + 1 : 1;

		// GFX Info
		auto forDesc = GetImageFormatDescriptor( img.m_eFormat );

		width = img.m_LayerSize[0];
		height = depth = 1;

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		_create(&m_uiOpenGLID, textureTarget);

		if (forDesc.m_bIsCompressed) {
			GLsizei  sMipMapLevelSizeInBytes = (GLsizei) img.GetLevelSize( 0 );
			initializeTextureMemoryCompressed(img.m_Data.data(), sMipMapLevelSizeInBytes, bCreateMipMapStorage);
		} else {
			initializeTextureMemory(img.m_Data.data(), bCreateMipMapStorage, bMutable);
		}

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		setParameter(GL_TEXTURE_MIN_LOD, 0);
		setParameter(GL_TEXTURE_MAX_LOD, mipmapLevels - 1);
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(bCreateMipMapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		check_opengl();

	}

	///////////////////////////////////////////////////////////
	/////////////// TextureObject2D

	TextureObject2D::TextureObject2D() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_2D;
	}

	TextureObject2D::~TextureObject2D()
	{}

	void TextureObject2D::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{
		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(width, height))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: use initializeTextureMemoryCompressed for compressed formats");

#if OPENGL_BINDLESS
		if (mutableStorage) {
			for (GLuint level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				GLint levelHeight = std::max(1u, height >> level);
				glTextureImage2DEXT(m_uiOpenGLID, textureTarget, level, internalFormat, levelWidth, levelHeight, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
				glTextureStorage2D(m_uiOpenGLID, mipmapLevels, internalFormat, width, height);
				if (data) glTextureSubImage2D(m_uiOpenGLID, 0, 0, 0, width, height, dataFormat, dataType, data);
		}
#else
		Bind();
		if (mutableStorage) {
			for (int level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				GLint levelHeight = std::max(1u, height >> level);
				glTexImage2D(textureTarget, level, internalFormat, levelWidth, levelHeight, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
			glTexStorage2D(textureTarget, mipmapLevels, internalFormat, width, height);
			if (data) glTexSubImage2D(textureTarget, 0, 0, 0, width, height, dataFormat, dataType, data);
		}
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		depth  = 1;

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

	void TextureObject2D::initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage)
	{

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(width, height))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: use initializeTextureMemory for non-compressed formats");

#if OPENGL_BINDLESS
		glCompressedTextureImage2DEXT( m_uiOpenGLID, GL_TEXTURE_2D, 0, internalFormat, width, height, 0, dataSize, data);
#else
		Bind();
		glCompressedTexImage2D( GL_TEXTURE_2D, 0, internalFormat, width, height, 0, dataSize, data);
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		depth  = 1;

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();

	}

	void TextureObject2D::uploadTextureData(const void* data) 
	{
#if OPENGL_BINDLESS
		glTextureSubImage2D(m_uiOpenGLID, 0, 0, 0, width, height, dataFormat, dataType, data);
#else
		Bind();
		glTexSubImage2D(textureTarget, 0, 0, 0, width, height, dataFormat, dataType, data);
#endif
		check_opengl();
	}

	void TextureObject2D::downloadTextureData(void* data, GLint lod, GLsizei bufsize) const
	{
		if (bufsize == -1) bufsize = std::numeric_limits<GLsizei>::max();

#if OPENGL_BINDLESS
		glGetTextureImage(m_uiOpenGLID, lod, dataFormat, dataType, bufsize, data);
#else
		Bind();
		glGetTexImage(textureTarget, lod, dataFormat, dataType, data);
#endif
		check_opengl();
	}

	void TextureObject2D::loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage)
	{
		Release();

		setupTextureFormat(GFXImageFormatToOpenGL(img.m_eFormat));

		width  = img.m_LayerSize.x;
		height = img.m_LayerSize.y;
		depth  = 1;

		_create(&m_uiOpenGLID, textureTarget);

		initializeTextureMemory(img.m_Data.data(), bCreateMipMapStorage, bMutable);

		return;

		//Assert( img.m_sNumberOfMipMapLevels == 1 && "Todo: load all mipmap levels" );

		setupTextureFormat(GFXImageFormatToOpenGL(img.m_eFormat));

		width  = img.m_LayerSize[0];
		height = img.m_LayerSize[1];
		depth = 1;

		GLuint mipmapLevels = bCreateMipMapStorage ? (GLuint) floor(log2(std::max(width, height))) + 1 : 1;

		// GFX Info
		auto forDesc = GetImageFormatDescriptor( img.m_eFormat );

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		check_opengl();

		_create(&m_uiOpenGLID, textureTarget);

		if (forDesc.m_bIsCompressed) {
			GLsizei sMipMapLevelSizeInBytes = (GLsizei) img.GetLevelSize( 0 );
			initializeTextureMemoryCompressed(img.m_Data.data(), sMipMapLevelSizeInBytes, bCreateMipMapStorage);
		} else {
			initializeTextureMemory(img.m_Data.data(), bCreateMipMapStorage, bMutable);
		}
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		setParameter(GL_TEXTURE_MIN_LOD, 0);
		setParameter(GL_TEXTURE_MAX_LOD, mipmapLevels - 1);
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(bCreateMipMapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		check_opengl();

	}

	void TextureObject2D::setSubImage(glm::ivec2 offset, glm::ivec2 size, const void* data, GLuint mipmaplevel)
	{
		if (mipmaplevel >= this->mipmapLevels) return; 
		 
		glTextureSubImage2D(m_uiOpenGLID, mipmaplevel, offset.x, offset.y, size.x, size.y, dataFormat, dataType, data);
		check_opengl();
	}


///////////////////////////////////////////////////////////
	/////////////// TextureObject2DArray

	TextureObject2DArray::TextureObject2DArray() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_2D_ARRAY;
	}

	TextureObject2DArray::~TextureObject2DArray()
	{}

	void TextureObject2DArray::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{
		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: initializeTextureMemory not possible for compressed formats");

#if OPENGL_BINDLESS
		if (mutableStorage) {
			for (GLuint level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				GLint levelHeight = std::max(1u, height >> level);
				GLint levelDepth = depth;
				glTextureImage3DEXT(m_uiOpenGLID, textureTarget, level, internalFormat, levelWidth, levelHeight, levelDepth, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
				glTextureStorage3D( m_uiOpenGLID, mipmapLevels, internalFormat, width, height, depth);
				if (data) glTextureSubImage3D(m_uiOpenGLID, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
		}
#else
		Bind();
		if (mutableStorage) {
			for (int level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				GLint levelHeight = std::max(1u, height >> level);
				GLint levelDepth = depth;
				glTexImage3D(textureTarget, level, internalFormat, levelWidth, levelHeight, levelDepth, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
			glTexStorage3D(textureTarget, mipmapLevels, internalFormat, width, height, depth);
			if (data) glTexSubImage3D(textureTarget, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
		}
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

	void TextureObject2DArray::initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage)
	{
		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: initializeTextureMemory not possible for compressed formats");

#if OPENGL_BINDLESS
		glCompressedTextureImage3DEXT( m_uiOpenGLID, GL_TEXTURE_1D, 0,  internalFormat, width, height, depth, 0, dataSize, data);
#else
		Bind();
		glCompressedTexImage3D( GL_TEXTURE_1D, 0,  internalFormat, width, height, depth, 0, dataSize, data);
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

	void TextureObject2DArray::uploadTextureData(const void* data) 
	{
#if OPENGL_BINDLESS
		glTextureSubImage3D(m_uiOpenGLID, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
#else
		Bind();
		glTexSubImage3D(textureTarget, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
		Unbind();
#endif

	}

	void TextureObject2DArray::downloadTextureData(void* data, GLint lod, GLsizei bufsize) const
	{
		if (bufsize == -1) bufsize = std::numeric_limits<GLsizei>::max();

#if OPENGL_BINDLESS
		glGetTextureImage(m_uiOpenGLID, lod, dataFormat, dataType, bufsize, data);
#else
		Bind();
		glGetTexImage(textureTarget, lod, dataFormat, dataType, data);
#endif
		check_opengl();
	}

	void TextureObject2DArray::loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage)
	{
		Release();

		// Assert(img.m_sNumberOfMipMapLevels == 1 && "Todo: load all mipmap levels");

		setupTextureFormat(GFXImageFormatToOpenGL(img.m_eFormat));

		// GFX Info
		auto forDesc = GetImageFormatDescriptor( img.m_eFormat );

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		check_opengl();

		width  = img.m_LayerSize[0];
		height = img.m_LayerSize[1];
		depth  = img.m_LayerSize[2];

		GLuint mipmapLevels = bCreateMipMapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;

		_create(&m_uiOpenGLID, textureTarget);

		if (forDesc.m_bIsCompressed) {
			GLsizei sMipMapLevelSizeInBytes = (GLsizei) img.GetLevelSize( 0 );
			initializeTextureMemoryCompressed(img.m_Data.data(), sMipMapLevelSizeInBytes, bCreateMipMapStorage);
		} else {
			initializeTextureMemory(img.m_Data.data(), bCreateMipMapStorage, bMutable);
		}

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		setParameter(GL_TEXTURE_MIN_LOD, 0);
		setParameter(GL_TEXTURE_MAX_LOD, mipmapLevels - 1);
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(bCreateMipMapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		check_opengl();
	}

	void TextureObject2DArray::setSubImage(glm::ivec3 offset, glm::ivec3 size, const void* data, GLuint mipmaplevel)
	{
		if (mipmaplevel >= this->mipmapLevels) return;

		glTextureSubImage3D(m_uiOpenGLID, mipmaplevel, offset.x, offset.y, offset.z, size.x, size.y, size.z, dataFormat, dataType, data);
		check_opengl();
	}


	///////////////////////////////////////////////////////////
	/////////////// TextureObject2DMultisample

	TextureObject2DMultisample::TextureObject2DMultisample() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_2D_MULTISAMPLE;
	}

	TextureObject2DMultisample::~TextureObject2DMultisample()
	{}

	void TextureObject2DMultisample::create(glm::ivec3 size, GLenum internalFormat, void* data)
	{
		Assert(!"Please use alternative create function that defines sample count");
	}

	void TextureObject2DMultisample::create(glm::ivec2 size, int samples, GLenum internalFormat, GLboolean fixedSampleLocations)
	{
		this->samples = samples;
		this->fixedSampleLocations = fixedSampleLocations;
		this->width  = size.x;
		this->height = size.y;
		this->depth  = 1;
		bpp = getBitsPerChannel(internalFormat) * channels;

		// create texture and upload image data

		_create(&m_uiOpenGLID, textureTarget);

#if OPENGL_BINDLESS
		glTextureImage2DMultisampleNV(m_uiOpenGLID, textureTarget, samples, internalFormat, width, height, fixedSampleLocations);
#else
		Bind();
		glTexImage2DMultisample(textureTarget, samples, internalFormat, width, height, fixedSampleLocations);
#endif

		setupTextureFormat(internalFormat);
	}

	void TextureObject2DMultisample::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{
		Assert(!"Use alternative create function to set texture format");
	}

	void TextureObject2DMultisample::uploadTextureData(const void* data) 
	{
		Assert(!"Unable to upload texture data for multisampled textures");
	}

	///////////////////////////////////////////////////////////
	/////////////// TextureObject3D

	TextureObject3D::TextureObject3D() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_3D;
	}

	TextureObject3D::~TextureObject3D()
	{}

	void TextureObject3D::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{
		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: initializeTextureMemory not possible for compressed formats");

#if OPENGL_BINDLESS
		if (mutableStorage) {
			for (GLuint level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				GLint levelHeight = std::max(1u, height >> level);
				GLint levelDepth = std::max(1u, depth >> level);
				glTextureImage3DEXT(m_uiOpenGLID, textureTarget, level, internalFormat, levelWidth, levelHeight, levelDepth, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
				glTextureStorage3D( m_uiOpenGLID, mipmapLevels, internalFormat, width, height, depth);
				if (data) glTextureSubImage3D(m_uiOpenGLID, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
		}
#else
		Bind();
		if (mutableStorage) {
			for (int level = 0; level < mipmapLevels; ++level) {
				GLint levelWidth = std::max(1u, width >> level);
				GLint levelHeight = std::max(1u, height >> level);
				GLint levelDepth = std::max(1u, depth >> level);
				glTexImage3D(textureTarget, level, internalFormat, levelWidth, levelHeight, levelDepth, 0, dataFormat, dataType, level == 0 ? data : nullptr);
			}
		} else {
			glTexStorage3D(textureTarget, mipmapLevels, internalFormat, width, height, depth);
			if (data) glTexSubImage3D(textureTarget, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
		}
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

	void TextureObject3D::initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage)
	{
		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: initializeTextureMemory not possible for compressed formats");

#if OPENGL_BINDLESS
		glCompressedTextureImage3DEXT( m_uiOpenGLID, GL_TEXTURE_1D, 0,  internalFormat, width, height, depth, 0, dataSize, data);
#else
		Bind();
		glCompressedTexImage3D( GL_TEXTURE_1D, 0,  internalFormat, width, height, depth, 0, dataSize, data);
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

	void TextureObject3D::uploadTextureData(const void* data) 
	{
#if OPENGL_BINDLESS
		glTextureSubImage3D(m_uiOpenGLID, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
#else
		Bind();
		glTexSubImage3D(textureTarget, 0, 0, 0, 0, width, height, depth, dataFormat, dataType, data);
		Unbind();
#endif

	}

	void TextureObject3D::loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage)
	{
		Release();

		// Assert(img.m_sNumberOfMipMapLevels == 1 && "Todo: load all mipmap levels");

		setupTextureFormat(GFXImageFormatToOpenGL(img.m_eFormat));

		// GFX Info
		auto forDesc = GetImageFormatDescriptor( img.m_eFormat );

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		check_opengl();

		width  = img.m_LayerSize[0];
		height = img.m_LayerSize[1];
		depth  = img.m_LayerSize[2];

		GLuint mipmapLevels = bCreateMipMapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;

		_create(&m_uiOpenGLID, textureTarget);

		if (forDesc.m_bIsCompressed) {
			GLsizei sMipMapLevelSizeInBytes = (GLsizei) img.GetLevelSize( 0 );
			initializeTextureMemoryCompressed(img.m_Data.data(), sMipMapLevelSizeInBytes, bCreateMipMapStorage);
		} else {
			initializeTextureMemory(img.m_Data.data(), bCreateMipMapStorage, bMutable);
		}

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		setParameter(GL_TEXTURE_MIN_LOD, 0);
		setParameter(GL_TEXTURE_MAX_LOD, mipmapLevels - 1);
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(bCreateMipMapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		check_opengl();
	}

	///////////////////////////////////////////////////////////
	/////////////// TextureObjectBuffer

	TextureObjectBuffer::TextureObjectBuffer() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_BUFFER;
	}

	TextureObjectBuffer::~TextureObjectBuffer()
	{}


	void TextureObjectBuffer::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{
		Assert(!"Cannot upload data directly to a buffer texture");
	}

	void TextureObjectBuffer::uploadTextureData(const void* data) 
	{
		Assert(!"Cannot upload data directly to a buffer texture");
	}

	///////////////////////////////////////////////////////////
	/////////////// TextureObjectCube

	TextureObjectCube::TextureObjectCube() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_CUBE_MAP;
	}

	TextureObjectCube::~TextureObjectCube()
	{}

	void TextureObjectCube::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(width, height))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: initializeTextureMemory not possible for compressed formats");

#if OPENGL_BINDLESS
		if (mutableStorage) {
			for (int i = 0; i < 6; ++i) {
				for (GLuint level = 0; level < mipmapLevels; ++level) {
					GLint levelWidth = std::max(1u, width >> level);
					GLint levelHeight = std::max(1u, height >> level);
					glTextureImage2DEXT(m_uiOpenGLID, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, level, internalFormat, levelWidth, levelHeight, 0, dataFormat, dataType, nullptr);
					// TODO : initializing the texture like this strangely prevents glGetTexImage from working 
				}
			}
		} else {
			glTextureStorage2D( m_uiOpenGLID, mipmapLevels,	internalFormat, width, height);
		}
#else
		Bind();
		if (mutableStorage) {
			for (int i = 0; i < 6; ++i) {
				for (int level = 0; level < mipmapLevels; ++level) {
					GLint levelWidth = std::max(1u, width >> level);
					GLint levelHeight = std::max(1u, height >> level);
					glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, level, internalFormat, levelWidth, levelHeight, 0, dataFormat, dataType, nullptr);
				}
			}
		} else {
			glTexStorage2D(textureTarget, mipmapLevels, internalFormat, width, height);
		}
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		depth  = 1;

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

	void TextureObjectCube::uploadTextureData(GLenum face, const void* data) 
	{
#if OPENGL_BINDLESS
		glTextureSubImage2DEXT(m_uiOpenGLID, face, 0, 0, 0, width, height, dataFormat, dataType, data);
#else
		Bind();
		glTexSubImage2D(face, 0, 0, 0, width, height, dataFormat, dataType, data);
#endif
	}

	void TextureObjectCube::uploadTextureData(const void* data[6]) 
	{
		for (int i = 0; i < 6; ++i)
		{
			uploadTextureData(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, data[i]);
		}
	}

	void TextureObjectCube::loadFromHorizontalCrossImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage)
	{
		Release();

		// Cubemaps might still show seams
		// http://the-witness.net/news/2012/02/seamless-cube-map-filtering/

		glm::uvec2 vSize( img.m_LayerSize );
		vSize[ 0 ] /= 4;
		vSize[ 1 ] /= 3;

		if ( vSize[ 0 ] != vSize[ 1 ] ) {
			throw( std::runtime_error( "CubeMaps require 4/3 size." ) );
		}
		Assert( vSize[ 0 ] == vSize[ 1 ] );

		// We will copy the subparts from a 2D temporary texture
		TextureObject2D texSource2D;
		texSource2D.loadFromImageData(img, bMutable, false);

		setupTextureFormat(GFXImageFormatToOpenGL(img.m_eFormat));

		width  = vSize[ 0 ];
		height = vSize[ 1 ];
		depth  = 1;

		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Support for compressed cubemaps not implemented yet.");

		GLuint mipmapLevels = bCreateMipMapStorage ? (GLuint) floor(log2(std::max(width, height))) + 1 : 1;

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		check_opengl();

		_create(&m_uiOpenGLID, textureTarget);
		initializeTextureMemory(nullptr, bCreateMipMapStorage, bMutable);

		//////// copy from 2D texture to cubemap faces

		// @todo: mipmap-levels:
		// for-each level {
		int iLevel = 0;
		auto sSize = vSize[ 0 ];
		// signature: (GLuint srcName, GLenum srcTarget, GLint srcLevel, GLint srcX, GLint srcY, GLint srcZ,
		//			   GLuint dstName, GLenum dstTarget, GLint dstLevel, GLint dstX, GLint dstY, GLint dstZ,
		//			   GLsizei srcWidth, GLsizei srcHeight, GLsizei srcDepth);

		check_opengl();
		Unbind();
		// +X
		glCopyImageSubData( texSource2D.m_uiOpenGLID, GL_TEXTURE_2D, GLint( iLevel ), 
			                2 * sSize, 1 * sSize, 0,
							m_uiOpenGLID, GL_TEXTURE_CUBE_MAP, GLint( iLevel ), 0, 0, 0,
							GLsizei( sSize ), GLsizei( sSize ), 1 );
		check_opengl();

		// -X
		glCopyImageSubData( texSource2D.m_uiOpenGLID, GL_TEXTURE_2D, GLint( iLevel ), 
			                0 * sSize, 1 * sSize, 0,
							m_uiOpenGLID, GL_TEXTURE_CUBE_MAP, GLint( iLevel ), 0, 0, 1,
							GLsizei( sSize ), GLsizei( sSize ), 1 );
		check_opengl();

		// +Y
		glCopyImageSubData( texSource2D.m_uiOpenGLID, GL_TEXTURE_2D, GLint( iLevel ), 
			                1 * sSize, 0 * sSize, 0, 
							m_uiOpenGLID, GL_TEXTURE_CUBE_MAP, GLint( iLevel ), 0, 0, 2,
							GLsizei( sSize ), GLsizei( sSize ), 1 );
		check_opengl();

		// -Y
		glCopyImageSubData( texSource2D.m_uiOpenGLID, GL_TEXTURE_2D, GLint( iLevel ), 
			                1 * sSize, 2 * sSize, 0, 
							m_uiOpenGLID, GL_TEXTURE_CUBE_MAP, GLint( iLevel ), 0, 0, 3,
							GLsizei( sSize ), GLsizei( sSize ), 1 );
		check_opengl();

		// +Z
		glCopyImageSubData( texSource2D.m_uiOpenGLID, GL_TEXTURE_2D, GLint( iLevel ), 
			                1 * sSize, 1 * sSize, 0, 
							m_uiOpenGLID, GL_TEXTURE_CUBE_MAP, GLint( iLevel ), 0, 0, 4,
							GLsizei( sSize ), GLsizei( sSize ), 1 );
		check_opengl();

		// -Z
		glCopyImageSubData( texSource2D.m_uiOpenGLID, GL_TEXTURE_2D, GLint( iLevel ), 
			                3 * sSize, 1 * sSize, 0, 
							m_uiOpenGLID, GL_TEXTURE_CUBE_MAP, GLint( iLevel ), 0, 0, 5,
							GLsizei( sSize ), GLsizei( sSize ), 1 );
		check_opengl();


		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		// release the 2D texture
		texSource2D.Release();

		check_opengl();

	}

	///////////////////////////////////////////////////////////
	/////////////// TextureObjectCubeArray

	TextureObjectCubeArray::TextureObjectCubeArray() 
		: TextureObject()
	{
		textureTarget = GL_TEXTURE_CUBE_MAP_ARRAY;
	}

	TextureObjectCubeArray::~TextureObjectCubeArray()
	{}

	void TextureObjectCubeArray::initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage)
	{

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		mipmapLevels = createMipmapStorage ? (GLuint) floor(log2(std::max(std::max(width, height), depth))) + 1 : 1;
		bool compressed = isCompressedFormat(internalFormat);
		if (compressed) Assert(!"Error: initializeTextureMemory not possible for compressed formats");

#if OPENGL_BINDLESS
		// Only non-mutable storage allowed
		glTextureStorage3D( m_uiOpenGLID, mipmapLevels,	internalFormat, width, height, depth);
#else
		Bind();
		glTexStorage2D(textureTarget, mipmapLevels, internalFormat, width, height);
#endif
		check_opengl();

		if (dataType == GL_BYTE || dataType == GL_UNSIGNED_BYTE) glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

		depth  = 1;

		// set texture params
		setParameter(GL_TEXTURE_BASE_LEVEL, 0);
		setParameter(GL_TEXTURE_MAX_LEVEL, mipmapLevels - 1);
		setInterpolationMethod(createMipmapStorage ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR, GL_LINEAR);
		setWrapMethod(GL_REPEAT);
		check_opengl();
	}

}
