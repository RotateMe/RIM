#pragma once

#include "common.h"
#include "helpers/ImageData.h"

#include <glm/glm.hpp>

namespace GLHelpers {

	class TextureObject
	{
	public:

		TextureObject();
		virtual ~TextureObject();

		// keep this in mind for state changes
		// http://www.opengl.org/wiki/Common_Mistakes#The_Object_Oriented_Language_Problem

		GLuint					m_uiOpenGLID;          // teture id
		GLenum                  internalFormat;        // internal OpenGL format
		GLenum                  dataType;              // data type
		GLenum                  dataFormat;            // data format
		GLuint                  width, height, depth;  // texture resolution
		GLuint                  channels;              // number of texture channels
		GLuint                  bpp;                   // pixel resolution = bits per pixel
		GLenum                  textureTarget;         // default target
		GLuint                  mipmapLevels;          // mipmap levels

		void                    create(glm::ivec2 size, GLenum internalFormat, void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		void                    create(glm::ivec3 size, GLenum internalFormat, void* data, bool createMipmapStorage = false, bool mutableStorage = false);

		virtual void			Release();
		virtual void            Bind(GLint textureUnit = -1) const;
		virtual void            Unbind(GLint textureUnit = -1) const;

		GLenum                  GetInternalFormat();

		void                    setupTextureFormat(GLenum textureFormat);
		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage, bool mutableStorage) = 0;

		virtual void  setInterpolationMethod(GLenum method);
		virtual void  setInterpolationMethod(GLenum minMethod, GLenum magMethod);
		virtual void  setWrapMethod(GLenum method);
		virtual void  setWrapMethod(GLenum methodS, GLenum methodT, GLenum methodR);
		virtual void  setParameter(GLenum paramName, GLint param);
		virtual GLint getParameter(GLenum paramName);

		virtual GLuint64 getHandle();
		virtual bool     isResident();
		virtual void     makeResident();
		virtual void     makeNotResident();

		void generateMipmap();
		void generateMipmap(GLint maxLevel, GLenum minMethod = GL_LINEAR_MIPMAP_LINEAR, GLenum magMethod = GL_LINEAR);

		glm::ivec2 size();
		glm::ivec3 size3();
	};

	class TextureObject1D : public TextureObject
	{
	public:
		TextureObject1D(); 
		~TextureObject1D();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage = false);
		virtual void            uploadTextureData(const void* data);
//		virtual void            downloadTextureData(void* outputData); // TODO
		virtual void            loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage);
	};

	class TextureObject2D : public TextureObject
	{
	public:
		TextureObject2D();
		~TextureObject2D();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage = false);
		virtual void            uploadTextureData(const void* data);
		virtual void            downloadTextureData(void* outputData, GLint  lod = 0, GLsizei bufsize = -1) const; 
//		virtual void            getTextureDataSize(void* outputData, GLint  lod = 0); // TODO
		virtual void            loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage);
		virtual void            setSubImage(glm::ivec2 offset, glm::ivec2 size, const void* data, GLuint mipmaplevel = 0);
	};

	class TextureObject2DArray : public TextureObject
	{
	public:
		TextureObject2DArray();
		~TextureObject2DArray();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage = false);
		virtual void            uploadTextureData(const void* data);
		virtual void            downloadTextureData(void* outputData, GLint  lod = 0, GLsizei bufsize = -1) const; 
//		virtual void            getTextureDataSize(void* outputData, GLint  lod = 0); // TODO
		virtual void            loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage);
		virtual void            setSubImage(glm::ivec3 offset, glm::ivec3 size, const void* data, GLuint mipmaplevel = 0);
	};

	class TextureObject2DMultisample : public TextureObject
	{
	public:
		TextureObject2DMultisample();
		~TextureObject2DMultisample();

		GLuint                  samples;
		GLboolean               fixedSampleLocations;

		virtual void            create(glm::ivec3 size, GLenum internalFormat, void* data);
		virtual void            create(glm::ivec2 size, int samples, GLenum internalFormat, GLboolean fixedSampleLocations = true);
		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            uploadTextureData(const void* data);
	};

	class TextureObject3D : public TextureObject
	{
	public:
		TextureObject3D();
		~TextureObject3D();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            initializeTextureMemoryCompressed(const void* data, GLsizei dataSize, bool createMipmapStorage = false);
		virtual void            uploadTextureData(const void* data);
//		virtual void            downloadTextureData(void* outputData); // TODO
		virtual void            loadFromImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage);
	};

	class TextureObjectBuffer : public TextureObject
	{
	public:
		TextureObjectBuffer();
		~TextureObjectBuffer();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            uploadTextureData(const void* data);
	};

	class TextureObjectCube : public TextureObject
	{
	public:
		TextureObjectCube();
		~TextureObjectCube();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
		virtual void            uploadTextureData(GLenum face, const void* data);
		virtual void            uploadTextureData(const void* data[6]);
		virtual void            loadFromHorizontalCrossImageData(const ImageData& img, bool bMutable, bool bCreateMipMapStorage);
	};

	class TextureObjectCubeArray : public TextureObject
	{
	public:
		TextureObjectCubeArray();
		~TextureObjectCubeArray();

		virtual void            initializeTextureMemory(const void* data, bool createMipmapStorage = false, bool mutableStorage = false);
	};

}

