#pragma once

#include "common.h"

#include <vector>
#include <unordered_map>

namespace GLHelpers {

	class BufferObject
	{
	public:

		BufferObject();

	//private:
		void					Create(GLenum targetHint);
		void					Release();
		// GLint size, GLenum type, GLboolean normalized

		void					Bind(GLenum target = GL_INVALID_ENUM) const;
		void					Unbind(GLenum target = GL_INVALID_ENUM) const;

		void                    UploadData(GLsizeiptr byteSize, GLenum usage, const void* data, GLenum target = GL_INVALID_ENUM);
		// @todo: should use some sort of buffer hint, otherwise the contained memory is known as void only
		// probably a string as well, as custom data is already difficult enough to handle in glsl with
		// structs, std140 and other issues

		GLenum                  m_targetHint;
		GLuint					m_uiBufferIdx;

		GLsizeiptr              m_sizeInBytes;
		GLenum                  m_usage;
	};

	class VertexArrayObject
	{
	public:

		VertexArrayObject();

	//private:
		void					Create();
		void					Release();

		void					Bind() const;
static	void					Unbind();

		void                    SetAttributeBufferSource(const BufferObject& bufferObject, GLuint index, GLint components, GLenum type = GL_FLOAT, GLboolean normalized = GL_FALSE, GLsizei stride = 0, const GLvoid * pointer = nullptr);

		void                    EnableAttribute(GLuint index, bool preBind = false);
		void                    DisableAttribute(GLuint index, bool preBind = false);

		GLuint					                m_uiVAOIdx; 
		std::unordered_map<GLuint, bool>        m_enabledAttributes; // unordered_map initializes to false if non-initialized :)
	};
	

	// This convenience class is for packing all typical mesh buffers and material info
	//class PackedMeshBuffers
	//{
	//public:

	//	PackedMeshBuffers();

	//	BufferObject*					                positionBuffer;
	//	BufferObject*					                normalBuffer;
	//	BufferObject*					                texCoordBuffer;
	//	BufferObject*					                indexBuffer;
	//	BufferObject*					                materialIndicesBuffer;
	//	std::vector< SimpleMaterial >*	    materials;
	//	std::vector< SimpleMaterialGroup >*	materialGroups;
	//	Mx::GFX::Mesh*                                  mesh;

	//};

	struct SceneMeshBuffers
	{

#define     MAX_BONES_PER_VERTEX 4

		VertexArrayObject vao;
		BufferObject      indirectDrawBuffer; // GL_DRAW_INDIRECT_BUFFER with the mesh draw values for a call to glDrawElementsIndirect

		BufferObject positions;
		BufferObject normals;
		BufferObject texCoords;
		BufferObject indices;

		GLenum       indexBufferType;

		BufferObject boneIndices;
		BufferObject boneWeights;

		std::vector < std::array < int, MAX_BONES_PER_VERTEX > > boneIndicesData;
		std::vector < std::array < float, MAX_BONES_PER_VERTEX > > boneWeightsData;
	};

	// This convenience class is for packing all typical scene buffers and material info
	//class PackedSceneBuffers
	//{
	//public:


	//	PackedSceneBuffers();

	//	std::vector<SceneMeshBuffers>*		            meshBuffers;
	//	std::vector< SimpleMaterial >*	    materials;

	//	BufferObject*                       materialSSBO;

	//	Scene*                              scene;

	//};

}