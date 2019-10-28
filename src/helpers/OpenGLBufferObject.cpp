#include "helpers/OpenGLBufferObject.h"

#include <set>

namespace GLHelpers {


	///////////////////////////////////////////////////////
	///////////////////////// BufferObject

	BufferObject::BufferObject() 
		: m_uiBufferIdx( 0 )
		, m_targetHint( GL_INVALID_ENUM )
		, m_sizeInBytes( 0 )
		, m_usage( GL_INVALID_ENUM )
	{}

	void BufferObject::Create(GLenum targetHint)
	{
		Release();
#if OPENGL_BINDLESS
		glCreateBuffers(1, &m_uiBufferIdx);
#else
		glGenBuffers(1, &m_uiBufferIdx);
#endif
		m_targetHint = targetHint;

		check_opengl();
	}

	void BufferObject::Release()
	{
		if ( m_uiBufferIdx ) {
			glDeleteBuffers( 1, &m_uiBufferIdx );
			*this = BufferObject(); // Set initial values
		}
		check_opengl();
	}

	void BufferObject::Bind(GLenum target) const
	{
		if (target == GL_INVALID_ENUM) target = m_targetHint;
		glBindBuffer(target, m_uiBufferIdx);

		check_opengl();
	}

	void BufferObject::Unbind(GLenum target) const
	{
		if (target == GL_INVALID_ENUM) target = m_targetHint;
		glBindBuffer(target, 0);

		check_opengl();
	}

	void BufferObject::UploadData(GLsizeiptr byteSize, GLenum usage, const void* data, GLenum target)
	{
		if (target == GL_INVALID_ENUM) target = m_targetHint;
#if OPENGL_BINDLESS
		glNamedBufferData( m_uiBufferIdx, byteSize, data, usage);
#else
		Bind(target);
		glBufferData( target, byteSize, data, usage);
#endif
		
		m_usage = usage;
		m_sizeInBytes = byteSize;

		check_opengl();
	}

	///////////////////////////////////////////////////////
	///////////////////////// VertexArrayObject

	VertexArrayObject::VertexArrayObject() :
		m_uiVAOIdx( 0 )
	{}

	void VertexArrayObject::Create()
	{
		Release();
		glGenVertexArrays(1, &m_uiVAOIdx);

		check_opengl();
	}

	void VertexArrayObject::Release()
	{
		if ( m_uiVAOIdx ) 
		{
			glDeleteVertexArrays( 1, &m_uiVAOIdx );
			m_uiVAOIdx = 0;
			m_enabledAttributes.clear();
		}
		check_opengl();
	}

	void VertexArrayObject::Bind() const
	{
		Assert(m_uiVAOIdx);
		glBindVertexArray(m_uiVAOIdx);

		check_opengl();
	}

	void VertexArrayObject::Unbind()
	{
		glBindVertexArray(0);

		check_opengl();
	}

	void VertexArrayObject::SetAttributeBufferSource(const BufferObject& bufferObject, GLuint index, GLint components, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid * pointer)
	{
		Bind();
		bufferObject.Bind(GL_ARRAY_BUFFER);
		
		static const std::set<GLenum> int_types   { GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT, GL_INT, GL_UNSIGNED_INT };
		static const std::set<GLenum> float_types { GL_HALF_FLOAT, GL_FLOAT, GL_DOUBLE, GL_FIXED, GL_INT_2_10_10_10_REV, GL_UNSIGNED_INT_2_10_10_10_REV, GL_UNSIGNED_INT_10F_11F_11F_REV };
		static const std::set<GLenum> long_types  { GL_DOUBLE };

		if (float_types.count(type) > 0) {
			glVertexAttribPointer(index, components, type, normalized, stride, pointer);
		} else if (int_types.count(type) > 0) {
			glVertexAttribIPointer(index, components, type, stride, pointer);
		} else if (long_types.count(type) > 0) {
			glVertexAttribLPointer(index, components, type, stride, pointer);
		} else {
			throw(ExceptionOpenGL("Wrong type sent to VertexArrayObject::SetAttributeBufferSource"));
			return;
		}



		EnableAttribute(index, false);

		check_opengl();
	}
	
	void VertexArrayObject::EnableAttribute(GLuint index, bool preBind)
	{

#if OPENGL_BINDLESS
		if (!m_enabledAttributes[index]) {
			glEnableVertexArrayAttrib(m_uiVAOIdx, index);
			m_enabledAttributes[index] = true;
		}
#else 
		if (preBind) Bind();

		if (!m_enabledAttributes[index]) {
			glEnableVertexAttribArray(index);
			m_enabledAttributes[index] = true;
		}
#endif

		check_opengl();
	}

	void VertexArrayObject::DisableAttribute(GLuint index, bool preBind)
	{
#if OPENGL_BINDLESS
		if (m_enabledAttributes[index]) {
			glDisableVertexArrayAttrib(m_uiVAOIdx, index);
			m_enabledAttributes[index] = false;
		}
#else 
		if (preBind) Bind();

		if (m_enabledAttributes[index]) {
			glDisableVertexAttribArray(index);
			m_enabledAttributes[index] = false;
		}
#endif

		check_opengl();
	}

	///////////////////////////////////////////////////////


	//PackedMeshBuffers::PackedMeshBuffers()
	//	: positionBuffer(nullptr)
	//	, normalBuffer(nullptr)
	//	, texCoordBuffer(nullptr)
	//	, indexBuffer(nullptr)
	//	, materialIndicesBuffer(nullptr)
	//	, materials(nullptr)
	//	, materialGroups(nullptr)
	//{}

	//PackedSceneBuffers::PackedSceneBuffers()
	//	: meshBuffers(nullptr)
	//	, materials(nullptr)
	//	, materialSSBO(nullptr)
	//	, scene(nullptr)
	//{}
}
