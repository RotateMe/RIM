#include "helpers/OpenGLVertexNameRenderer.h"
#include "managers/GLStateManager.h"

#include <iostream>
#include <array>

using namespace GLHelpers;

void OpenGLVertexNameRenderer::setup(size2D size)
{
	fbo.Create();
	reload_programs();
	recreate_textures(size);
}

void OpenGLVertexNameRenderer::recreate_textures(size2D size)
{
	if (!depthtex.m_uiOpenGLID || depthtex.width != size.width || depthtex.height != size.height ) {
		depthtex.create(glm::ivec2(size.width, size.height), GL_DEPTH_COMPONENT32F, nullptr);
	}

	if (!nametex.m_uiOpenGLID || nametex.width != size.width || nametex.height != size.height) {
		nametex.create(glm::ivec2(size.width, size.height), GL_R32F, nullptr);
	}

}

void OpenGLVertexNameRenderer::reload_programs()
{
	GLHelpers::Detail::ShaderCommandLine cmd;
	cmd.m_Defines.push_back(GLHelpers::NameValue(std::string("LINEARIZE_DEPTH"), std::string("0")));

	ProgramObject::ShaderPaths p;
	p.commonPath = "shaders/";
	p.vertexShaderFilename = "name.vert";
	p.fragmentShaderFilename = "name.frag";

	try {
		program.CompileProgram(p, nullptr, cmd);
	}
	catch (std::runtime_error(e)) {
		std::cout << "Error creating opengl program" << std::endl;
	}
}

void OpenGLVertexNameRenderer::render(size2D size, CameraFPS& cam, GLHelpers::BufferObject& vertex_buffer, float pointsize)
{
	recreate_textures(size);

	fbo.Bind();
	fbo.AttachDepthTexture(depthtex);
	fbo.AttachColorTexture(nametex, 0);
	fbo.EnableConsecutiveDrawbuffers(1, 0);

	GLState state;
	state.setViewportBox(0, 0, size.width, size.height);
	state.enable_cull_face = false;
	state.enable_depth_test = true;
	state.enable_depth_write = true;
	state.depth_test_func = GL_LESS;
	GLStateManager::instance().setState(state, true);

	float clearDepth = 1.f;
	glClearNamedFramebufferfv(fbo.m_uiOpenGLID, GL_DEPTH, 0, &clearDepth);

	std::array<float, 4> clearColor = { -1.f, -1.f, -1.f, -1.f };
	glClearNamedFramebufferfv(fbo.m_uiOpenGLID, GL_COLOR, 0, clearColor.data());

	check_opengl();

	glm::mat4 mMatrix, vMatrix, pMatrix;

	vMatrix = cam.view_matrix();
	pMatrix = cam.proj_matrix();
	mMatrix = glm::identity<glm::mat4>();

	program.Use();

	program.SetUniform("mvpMatrix", pMatrix * vMatrix * mMatrix);

	check_opengl();

	GLsizei vertex_count = GLsizei(vertex_buffer.m_sizeInBytes / ( 3*sizeof(float) ) );
	
	glPointSize(pointsize);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer.m_uiBufferIdx);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
	glDrawArrays(GL_POINTS, 0, vertex_count);

	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	program.Use(false);
	fbo.Unbind();

}

void OpenGLVertexNameRenderer::destroy()
{
	program.Delete();
	depthtex.Release();
	nametex.Release();
	fbo.Delete();
	check_opengl();
}
