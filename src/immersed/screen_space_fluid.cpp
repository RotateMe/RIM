#include "immersed/screen_space_fluid.h"
#include "helpers/shapes.h"

#include "managers/TextureManager.h"
#include "managers/GLStateManager.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <fstream>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

using namespace glm;

ScreenSpaceFluid::ScreenSpaceFluid() 
{
	o_waterColor = vec4(0.5f, 0.5f, 1.0f, 0.001f);
	o_size = 0.06f;

	o_filterSize   = 4;
	o_filterWeightNor = 0.5f;
	o_filterWeightPos = 0.5f;
	o_normalSmooth = 0.0f;
	o_refractionIndex = 1.15f;
	o_enableSSRef = true;
	o_reflectionMult = 1.f;
	o_reflectionGamma = 1.f;
	o_refractionMult = 1.f;
	o_refractionGamma = 1.f;
	o_densityThreshold = 0.3f;
	o_flatColorMix = 0.05f;
	o_bypass = false;
	o_useNormalTexFiltering = false;
	o_cheapDensity = false;

	outputSize = ivec2(0, 0);

	shapeType = SHAPE_TYPE::ST_QUAD;
	//shapeType = SHAPE_TYPE::ST_POINTSPRITE; 

	o_fxaa = true;

	dirtyShaders = true;
	dirtyShape   = true;
}

void ScreenSpaceFluid::ReleaseResources()
{
	fbo.Delete();
	programBackDepth.Delete();
	programBackDepthPointSprite.Delete();
	programDensity.Delete();
	programDensityPointSprite.Delete();
	programDensityCheap.Delete();
	programGeometry.Delete();
	programGeometryPointSprite.Delete();
	programBilateralFilter.Delete();
	programSSFluid.Delete();
	programFXAA.Delete();

	shapePosBuffer.Release();
	shapeNorBuffer.Release();
	shapeTexCoordBuffer.Release();
	shapeIndexBuffer.Release();
	shapeVAO.Release();

	quadPosBuffer.Release();
	quadNorBuffer.Release();
	quadTexCoordBuffer.Release();
	quadIndexBuffer.Release();
	quadVAO.Release();

	depthTex.Release();
	normalTex.Release();
	geometryTex.Release();
	geometryBilateralAuxTex.Release();
	geometryBilateralAuxTex2.Release();
	densityTex.Release();
	backDepthTex.Release();
	quadTex.Release();
	sceneColorAuxTex.Release();

	sampler.Release();

	outputSize = ivec2(0, 0);
}

void ScreenSpaceFluid::InitResources()
{

	fbo.Create();
	shapeVAO.Create();
	quadVAO.Create();
	positionsVAO.Create();

	//// Load quad buffers
	ShapeBuffers sb; createQuad(sb);
	quadPosBuffer.Create(GL_ARRAY_BUFFER);
	quadNorBuffer.Create(GL_ARRAY_BUFFER);
	quadTexCoordBuffer.Create(GL_ARRAY_BUFFER);
	quadIndexBuffer.Create(GL_ELEMENT_ARRAY_BUFFER);

	quadPosBuffer.UploadData(sb.positions.size() * sizeof(vec3), GL_STATIC_DRAW, sb.positions.data());
	quadNorBuffer.UploadData(sb.normals.size() * sizeof(vec3), GL_STATIC_DRAW, sb.normals.data());
	quadTexCoordBuffer.UploadData(sb.texcoords.size() * sizeof(vec2), GL_STATIC_DRAW, sb.texcoords.data());
	quadIndexBuffer.UploadData(sb.indices.size() * sizeof(GLushort), GL_STATIC_DRAW, sb.indices.data());

	quadVAO.SetAttributeBufferSource(quadPosBuffer, 0, 3, GL_FLOAT, false);
	quadVAO.SetAttributeBufferSource(quadNorBuffer, 1, 3, GL_FLOAT, true);
	quadVAO.SetAttributeBufferSource(quadTexCoordBuffer, 2, 2, GL_FLOAT, false);
	quadVAO.EnableAttribute(0);
	quadVAO.EnableAttribute(1);
	quadVAO.EnableAttribute(2);

	//// Create sampler for textures
	sampler.create();
	sampler.setWrapMethod(GL_CLAMP_TO_EDGE);
	sampler.setInterpolationMethod(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);
	sampler.setParameter(GL_TEXTURE_COMPARE_MODE, GL_NONE);

}

void ScreenSpaceFluid::loadShape(SHAPE_TYPE type)
{
	shapePosBuffer.Release();
	shapeNorBuffer.Release();
	shapeTexCoordBuffer.Release();
	shapeIndexBuffer.Release();

	ShapeBuffers sb;

	switch (type)
	{
	default:
	case SHAPE_TYPE::ST_SPHERE:
		createSphere(8, 10, sb);
		break;
	case SHAPE_TYPE::ST_CUBE:
		createCube(sb);
		break;
	case SHAPE_TYPE::ST_QUAD:
		createQuad(sb);
		break;
	}

	shapePosBuffer.Create(GL_ARRAY_BUFFER);
	shapeNorBuffer.Create(GL_ARRAY_BUFFER);
	shapeTexCoordBuffer.Create(GL_ARRAY_BUFFER);
	shapeIndexBuffer.Create(GL_ELEMENT_ARRAY_BUFFER);

	shapePosBuffer.UploadData(sb.positions.size() * sizeof(vec3), GL_STATIC_DRAW, sb.positions.data());
	shapeNorBuffer.UploadData(sb.normals.size() * sizeof(vec3), GL_STATIC_DRAW, sb.normals.data());
	shapeTexCoordBuffer.UploadData(sb.texcoords.size() * sizeof(vec2), GL_STATIC_DRAW, sb.texcoords.data());
	shapeIndexBuffer.UploadData(sb.indices.size() * sizeof(GLushort), GL_STATIC_DRAW, sb.indices.data());

	shapeVAO.SetAttributeBufferSource(shapePosBuffer, 0, 3, GL_FLOAT, false);
	shapeVAO.SetAttributeBufferSource(shapeNorBuffer, 1, 3, GL_FLOAT, true);
	shapeVAO.SetAttributeBufferSource(shapeTexCoordBuffer, 2, 2, GL_FLOAT, false);
	shapeVAO.EnableAttribute(0);
	shapeVAO.EnableAttribute(1);
	shapeVAO.EnableAttribute(2);

	dirtyShape = false;
}

void ScreenSpaceFluid::setDirtyShaders()
{
	dirtyShaders = true;
}

void ScreenSpaceFluid::createPrograms()
{
	String commonPath = "shaders/immersed/";

	{
		ProgramObject::ShaderPaths sp;
		sp.commonPath = commonPath;
		sp.vertexShaderFilename   = "density.vert";
		sp.fragmentShaderFilename = "density.frag";
		programDensity.CompileProgram(sp, nullptr);
	}

	{
		ProgramObject::ShaderPaths sp;
		sp.commonPath = commonPath;
		sp.vertexShaderFilename = "density_pointsprite.vert";
		sp.fragmentShaderFilename = "density_pointsprite.frag";
		programDensityPointSprite.CompileProgram(sp, nullptr);
	}

	{
		programDensityCheap.CompileComputeProgram(nullptr, commonPath + "density_depth.comp");
	}

	{
		ProgramObject::ShaderPaths sp;
		sp.commonPath = commonPath;
		sp.vertexShaderFilename   = "geometry.vert";
		sp.fragmentShaderFilename = "geometry.frag";
		programGeometry.CompileProgram(sp, nullptr);
	}

	{
		ProgramObject::ShaderPaths sp;
		sp.commonPath = commonPath;
		sp.vertexShaderFilename = "geometry_pointsprite.vert";
		sp.fragmentShaderFilename = "geometry_pointsprite.frag";
		programGeometryPointSprite.CompileProgram(sp, nullptr);
	}

	{
		ProgramObject::ShaderPaths sp;
		sp.commonPath = commonPath;
		sp.vertexShaderFilename   = "back.vert";
		sp.fragmentShaderFilename = "back.frag";
		programBackDepth.CompileProgram(sp, nullptr);
	}

	{
		ProgramObject::ShaderPaths sp;
		sp.commonPath = commonPath;
		sp.vertexShaderFilename = "back_pointsprite.vert";
		sp.fragmentShaderFilename = "back_pointsprite.frag";
		programBackDepthPointSprite.CompileProgram(sp, nullptr);
	}

	{
		programBilateralFilter.CompileComputeProgram(nullptr, commonPath + "bilateral.comp");
	}

	{
		programSSFluid.CompileComputeProgram(nullptr, commonPath + "ssfluid.comp");
	}
	
	{
		programFXAA.CompileComputeProgram(nullptr, commonPath + "fxaa.comp");
	}

	dirtyShaders = false;
}

void ScreenSpaceFluid::createTextures(glm::ivec2 size)
{
	depthTex.create(size,                 GL_DEPTH_COMPONENT32F, nullptr, true);
	backDepthTex.create(size,             GL_DEPTH_COMPONENT32F, nullptr, true);
	normalTex.create(size,                GL_RGBA16F, nullptr, true);
	normalBilateralAuxTex.create(size,    GL_RGBA16F, nullptr, true);
	normalBilateralAuxTex2.create(size,   GL_RGBA16F, nullptr, true);
	geometryTex.create(size,              GL_RGBA16F, nullptr, true);
	geometryBilateralAuxTex.create(size,  GL_RGBA16F, nullptr, true);
	geometryBilateralAuxTex2.create(size, GL_RGBA32F, nullptr, true);
	densityTex.create(size,               GL_R16F, nullptr, true);
	sceneColorAuxTex.create(size,         GL_RGBA16F, nullptr, true);

	depthTex.setParameter    (GL_TEXTURE_COMPARE_MODE, GL_NONE);
	backDepthTex.setParameter(GL_TEXTURE_COMPARE_MODE, GL_NONE);

	fbo.AttachColorTexture(normalTex, 0);
	fbo.AttachColorTexture(geometryTex, 1);
	fbo.AttachColorTexture(densityTex, 2);

	outputSize = size;
}

void ScreenSpaceFluid::loadQuadTex()
{

	quadTex.Release();

	std::string quadtex_name = "data/immersed/cloud.png"; //!!
	if (!TextureManager::instance().hasTexture(quadtex_name)) TextureManager::instance().loadTexture(quadtex_name, true);

	if (TextureManager::instance().hasTexture(quadtex_name))
	{
		TextureData& texdata = TextureManager::instance().getTexture(quadtex_name);
		quadTex = *((GLHelpers::TextureObject2D*)(texdata.gltex));
		quadTex.generateMipmap();
	}
}

void ScreenSpaceFluid::Render(CameraFPS& cam)
{
	if (o_bypass) return;
 
	if (dirtyShaders) createPrograms();
	if (dirtyShape) loadShape(shapeType);

	mat4 mMatrix = identity<mat4>();
	mat4 vMatrix = cam.view_matrix();
	mat4 pMatrix = cam.proj_matrix();

	mat4 mvpMatrix = pMatrix * vMatrix * mMatrix;

	mat4 vMatrixInv = glm::inverse(vMatrix);
	mat4 pMatrixInv = glm::inverse(pMatrix);

	vec3 cameraPos   = cam.position;
	float cameraNear = cam.nearplane;
	float cameraFar  = cam.farplane;

	vec3 cameraUp      = cam.up();
	vec3 cameraRight   = cam.right();
	vec3 cameraForward = cam.forward();

	if (outputSize != sceneColorTex.size()) createTextures(sceneColorTex.size());


	if (quadTex.width == 0 || quadTex.height == 0) loadQuadTex();

	fbo.Bind();
	fbo.EnableConsecutiveDrawbuffers(3, 0);
	fbo.AttachDepthTexture(depthTex);

	// Clear textures
	{
		glDepthMask(GL_TRUE); // To make sure we can clear depth
		float clearDepth = 1.f;
		vec4 clearColor = vec4(0.f, 0.f, 0.f, 0.f);
		glClearBufferfv(GL_DEPTH, 0, &clearDepth);
		glClearBufferfv(GL_COLOR, 0, glm::value_ptr(clearColor));
		glClearBufferfv(GL_COLOR, 1, glm::value_ptr(clearColor));
		glClearBufferfv(GL_COLOR, 2, glm::value_ptr(clearColor));
	}

	size_t numInstances = (fluidPosition.m_sizeInBytes / sizeof(vec3));

	// retrieve the size, so we can create a full viewport
	ivec2 imgSize = depthTex.size();
	glViewport(0, 0, imgSize[0], imgSize[1]);

	check_opengl();

	GLState state;
	state.setViewportBox(0, 0, outputSize.x, outputSize.y);

	// Draw geometry
	{
		ProgramObject& program = (shapeType == SHAPE_TYPE::ST_POINTSPRITE) ? programGeometryPointSprite : programGeometry;

		program.Use();
		program.SetUniform("mvp", mvpMatrix);
		program.SetUniform("mMatrix", mMatrix);
		program.SetUniform("vMatrix", vMatrix);
		program.SetUniform("pMatrix", pMatrix);
		program.SetUniform("vMatrixInv", vMatrixInv);
		program.SetUniform("cameraPos", cameraPos);
		program.SetUniform("cameraRight", cameraRight);
		program.SetUniform("cameraUp", cameraUp);
		program.SetUniform("cameraForward", cameraForward);
		program.SetUniform("scale", o_size);
		program.SetTexture("sceneDepthTex", sceneDepthTex, 0);
		program.SetTexture("quadTex", quadTex, 2, sampler);
		program.SetUniform("quadTexAvailable", quadTex.m_uiOpenGLID ? true : false);

		fbo.EnableConsecutiveDrawbuffers(2, 0);

		state.enable_cull_face = false;
		state.cull_face = GL_BACK;
		state.enable_depth_test = true;
		state.enable_depth_write = true;
		state.depth_test_func = GL_LESS;
		state.enable_blend = false;
		GLStateManager::instance().setState(state);

		size_t numElements = (shapeIndexBuffer.m_sizeInBytes / sizeof(GLushort));

		if (shapeType == SHAPE_TYPE::ST_POINTSPRITE) {
			program.SetUniform("outputSize", outputSize);
			positionsVAO.SetAttributeBufferSource(fluidPosition, 0, 3, GL_FLOAT, GL_FALSE);
			positionsVAO.Bind();
			glEnable(GL_PROGRAM_POINT_SIZE);
			glEnable(GL_POINT_SPRITE);
			glPointParameteri(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);
			glDrawArrays(GL_POINTS, 0, GLsizei(numInstances));
		} else {
			bool viewAligned = (shapeType == SHAPE_TYPE::ST_QUAD);
			program.SetUniform("viewAlign", viewAligned);
			program.SetSSBO("InstanceInfo", fluidPosition, 0);
			shapeVAO.Bind();
			shapeIndexBuffer.Bind(GL_ELEMENT_ARRAY_BUFFER);
			glDrawElementsInstanced(GL_TRIANGLES, GLsizei(numElements), GL_UNSIGNED_SHORT, nullptr, GLsizei(numInstances));
		}
	}

	// Draw fluid back
	if (o_cheapDensity || o_waterColor.w == 0.f)
	{
		ProgramObject& program = shapeType == SHAPE_TYPE::ST_POINTSPRITE ? programBackDepthPointSprite : programBackDepth;

		program.Use();

		program.SetUniform("mvp", mvpMatrix);
		program.SetUniform("mMatrix", mMatrix);
		program.SetUniform("vMatrix", vMatrix);
		program.SetUniform("pMatrix", pMatrix);
		program.SetUniform("cameraPos", cameraPos);
		program.SetUniform("scale", o_size);
		program.SetTexture("quadTex", quadTex, 2, sampler);
		program.SetUniform("quadTexAvailable", quadTex.m_uiOpenGLID ? true : false);

		state.enable_cull_face = true;
		state.cull_face = GL_BACK;
		state.enable_depth_test = true;
		state.enable_depth_write = true;
		state.depth_test_func = GL_GREATER;
		state.enable_blend = false;
		GLStateManager::instance().setState(state);

		fbo.EnableConsecutiveDrawbuffers(0, 0);
		fbo.AttachDepthTexture(backDepthTex);

		float clearDepth = 0.f;
		glClearBufferfv(GL_DEPTH, 0, &clearDepth);

		size_t numElements = (quadIndexBuffer.m_sizeInBytes / sizeof(GLushort));

		if (shapeType == SHAPE_TYPE::ST_POINTSPRITE) {
			program.SetUniform("outputSize", outputSize);
			positionsVAO.SetAttributeBufferSource(fluidPosition, 0, 3, GL_FLOAT, GL_FALSE);
			positionsVAO.Bind();
			program.SetUniform("cameraRight", cameraRight);
			program.SetUniform("cameraUp", cameraUp);
			glEnable(GL_PROGRAM_POINT_SIZE);
			glEnable(GL_POINT_SPRITE);
			glPointParameteri(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);
			glDrawArrays(GL_POINTS, 0, GLsizei(numInstances));
		}
		else {
			program.SetUniform("viewAlign", true);
			program.SetSSBO("InstanceInfo", fluidPosition, 0);
			quadVAO.Bind();
			quadIndexBuffer.Bind(GL_ELEMENT_ARRAY_BUFFER);
			glDrawElementsInstanced(GL_TRIANGLES, GLsizei(numElements), GL_UNSIGNED_SHORT, nullptr, GLsizei(numInstances));
		}
	}

	// Draw density
	if (o_cheapDensity || o_waterColor.w == 0.f)
	{
		fbo.Unbind();

		programDensityCheap.SetUniform("mvp", mvpMatrix);
		programDensityCheap.SetUniform("mMatrix", mMatrix);
		programDensityCheap.SetUniform("vMatrix", vMatrix);
		programDensityCheap.SetUniform("pMatrix", pMatrix);
		programDensityCheap.SetUniform("cameraPos", cameraPos);
		programDensityCheap.SetUniform("scale", o_size);
		programDensityCheap.SetUniform("cameraNear", cameraNear);
		programDensityCheap.SetUniform("cameraFar", cameraFar);
		programDensityCheap.SetTexture("sceneDepthTex",      sceneDepthTex, 1, sampler);
		programDensityCheap.SetTexture("fluidFrontDepthTex", depthTex,      2, sampler);
		programDensityCheap.SetTexture("fluidBackDepthTex",  backDepthTex,  3, sampler);
		programDensityCheap.SetImageTexture(densityTex, 0, GL_WRITE_ONLY);

		programDensityCheap.DispatchCompute(sceneColorTex.size(), ivec2(32, 32), true);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

	} else {
		ProgramObject& program = shapeType == SHAPE_TYPE::ST_POINTSPRITE ? programDensityPointSprite : programDensity;

		program.Use();
		program.SetUniform("mvp", mvpMatrix);
		program.SetUniform("mMatrix", mMatrix);
		program.SetUniform("vMatrix", vMatrix);
		program.SetUniform("pMatrix", pMatrix);
		program.SetUniform("cameraPos", cameraPos);
		program.SetUniform("scale", o_size);
		program.SetTexture("sceneDepthTex", sceneDepthTex, 0);
		program.SetTexture("quadTex", quadTex, 2, sampler);
		program.SetUniform("quadTexAvailable", quadTex.m_uiOpenGLID ? true : false);

		fbo.EnableConsecutiveDrawbuffers(1, 2);

		state.enable_cull_face = true;
		state.cull_face = GL_BACK;
		state.enable_depth_test = false;
		state.enable_depth_write = false;
		state.enable_blend = true;
		state.blend_function_dst = GL_ONE;
		state.blend_function_src = GL_ONE;
		GLStateManager::instance().setState(state);

		size_t numElements = (quadIndexBuffer.m_sizeInBytes / sizeof(GLushort));

		if (shapeType == SHAPE_TYPE::ST_POINTSPRITE) {
			program.SetUniform("outputSize", sceneDepthTex.size());
			positionsVAO.SetAttributeBufferSource(fluidPosition, 0, 3, GL_FLOAT, GL_FALSE);
			positionsVAO.Bind();
			glEnable(GL_PROGRAM_POINT_SIZE);
			glEnable(GL_POINT_SPRITE);
			program.SetUniform("vMatrixInv", vMatrixInv);
			glPointParameteri(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);
			glDrawArrays(GL_POINTS, 0, GLsizei(numInstances));
		}
		else {
			program.SetUniform("viewAlign", true);
			program.SetSSBO("InstanceInfo", fluidPosition, 0);
			quadVAO.Bind();
			quadIndexBuffer.Bind(GL_ELEMENT_ARRAY_BUFFER);
			glDrawElementsInstanced(GL_TRIANGLES, GLsizei(numElements), GL_UNSIGNED_SHORT, nullptr, GLsizei(numInstances));
		}
	}

	shapeVAO.Unbind();
	fbo.Unbind();

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);


	// Bilateral filter on geometry (not used for now)
	if (!o_useNormalTexFiltering)
	{
		programBilateralFilter.SetImageTexture(geometryBilateralAuxTex, 0, GL_WRITE_ONLY);
		programBilateralFilter.SetTexture("inputTex", geometryTex, 1, sampler);
		programBilateralFilter.SetTexture("comparePosTex", geometryTex, 2, sampler);

		programBilateralFilter.SetUniform("normalFiltering", false);
		programBilateralFilter.SetUniform("distMaxNor", o_filterWeightNor);
		programBilateralFilter.SetUniform("distMaxPos", o_filterWeightPos);
		programBilateralFilter.SetUniform("mipmap", 0);
		programBilateralFilter.SetUniform("kernelSize", o_filterSize);
		programBilateralFilter.SetUniform("dotComparison", false);
		programBilateralFilter.SetUniform("iteration", 0);

		programBilateralFilter.DispatchCompute(geometryTex.size(), ivec2(16, 16), true);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

		programBilateralFilter.SetImageTexture(geometryBilateralAuxTex2, 0, GL_WRITE_ONLY);
		programBilateralFilter.SetTexture("inputTex", geometryBilateralAuxTex, 1, sampler);
		programBilateralFilter.SetTexture("comparePosTex", geometryTex, 2, sampler);

		programBilateralFilter.SetUniform("iteration", 1);

		programBilateralFilter.DispatchCompute(geometryTex.size(), ivec2(16, 16), true);
	}

	// Bilateral filter on normal
	if (o_useNormalTexFiltering)
	{
		programBilateralFilter.SetImageTexture(normalBilateralAuxTex, 0, GL_WRITE_ONLY);
		programBilateralFilter.SetTexture("inputTex", normalTex, 1, sampler);
		programBilateralFilter.SetTexture("compareNorTex", normalTex, 2, sampler);
		programBilateralFilter.SetTexture("comparePosTex", geometryTex, 3, sampler);
		programBilateralFilter.SetUniform("normalFiltering", true);

		programBilateralFilter.SetUniform("distMaxNor", o_filterWeightNor);
		programBilateralFilter.SetUniform("distMaxPos", o_filterWeightPos);
		programBilateralFilter.SetUniform("mipmap", 0);
		programBilateralFilter.SetUniform("kernelSize", o_filterSize);
		programBilateralFilter.SetUniform("dotComparison", true);
		programBilateralFilter.SetUniform("iteration", 0);

		programBilateralFilter.DispatchCompute(normalTex.size(), ivec2(16, 16), true);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

		normalBilateralAuxTex.generateMipmap();

		programBilateralFilter.SetImageTexture(normalBilateralAuxTex2, 0, GL_WRITE_ONLY);
		programBilateralFilter.SetTexture("inputTex", normalBilateralAuxTex, 1, sampler);
		programBilateralFilter.SetTexture("compareNorTex", normalTex, 2, sampler);
		programBilateralFilter.SetTexture("comparePosTex", geometryTex, 3, sampler);

		programBilateralFilter.SetUniform("iteration", 1);

		programBilateralFilter.DispatchCompute(normalTex.size(), ivec2(16, 16), true);
	}

	depthTex.generateMipmap();
	densityTex.generateMipmap();
	sceneColorTex.generateMipmap();
	sceneDepthTex.generateMipmap();

	if (o_useNormalTexFiltering) {
		normalBilateralAuxTex2.generateMipmap();
	} else {
		geometryBilateralAuxTex2.generateMipmap();
	}

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

	// Draw ss fluid
	{
		programSSFluid.SetImageTexture(sceneColorTex, 0, GL_WRITE_ONLY);
		//programSSFluid.SetImageTexture(normalTex,    1, GL_WRITE_ONLY);

		programSSFluid.SetTexture("sceneDepthTex", sceneDepthTex, 1, sampler);
		programSSFluid.SetTexture("sceneColorTex", sceneColorTex, 2, sampler);
		programSSFluid.SetTexture("depthTex", depthTex, 3, sampler);
		programSSFluid.SetTexture("normalTex", normalBilateralAuxTex2, 4, sampler);
		programSSFluid.SetTexture("geometryTex", geometryBilateralAuxTex2, 5, sampler);
		programSSFluid.SetTexture("densityTex", densityTex, 6, sampler);
		programSSFluid.SetTexture("backDepthTex", backDepthTex, 7, sampler);
		programSSFluid.SetTexture("envTex",     envTex, 9, sampler);
		programSSFluid.SetTexture("envCubeTex", envCubeTex, 10, sampler);

		programSSFluid.SetUniform("cameraPos", cameraPos);
		programSSFluid.SetUniform("cameraNear", cameraNear);
		programSSFluid.SetUniform("cameraFar", cameraFar);
		programSSFluid.SetUniform("waterColor", o_waterColor);
		programSSFluid.SetUniform("scale", o_size);
		programSSFluid.SetUniform("normalFiltering", o_useNormalTexFiltering);
		programSSFluid.SetUniform("normalSmoothing", o_normalSmooth);
		programSSFluid.SetUniform("vMatrix", vMatrix);
		programSSFluid.SetUniform("pMatrix", pMatrix);
		programSSFluid.SetUniform("vMatrixInv", vMatrixInv);
		programSSFluid.SetUniform("pMatrixInv", pMatrixInv);
		programSSFluid.SetUniform("refractionIndex", o_refractionIndex);
		programSSFluid.SetUniform("reflectionMult", o_reflectionMult);
		programSSFluid.SetUniform("reflectionGamma", o_reflectionGamma);
		programSSFluid.SetUniform("refractionMult", o_refractionMult);
		programSSFluid.SetUniform("refractionGamma", o_refractionGamma);
		programSSFluid.SetUniform("densityThreshold", o_densityThreshold);
		programSSFluid.SetUniform("flatColorMix", o_flatColorMix);
		programSSFluid.SetUniform("enableScreenSpaceRefractions", o_enableSSRef);

		programSSFluid.DispatchCompute(sceneColorTex.size(), ivec2(16, 16), true);
	}

	if (o_fxaa)
	{
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);

		programFXAA.SetImageTexture(sceneColorTex, 0, GL_WRITE_ONLY);
		programFXAA.SetTexture("inputTex", sceneColorTex, 1, sampler);

		programFXAA.DispatchCompute(sceneColorTex.size(), ivec2(32, 32), true);
	}

	check_opengl();
}

ScreenSpaceFluid::GUI::GUI(ScreenSpaceFluid* ssf)
{
	ssfptr = ssf;
	visible = true;
	displayname = "Fluid rendering options";
}

ScreenSpaceFluid::GUI::~GUI()
{
}

void ScreenSpaceFluid::GUI::draw(GLFWwindow* window, int order)
{
	if (!ssfptr) return;

	ImVec2 outerSize = ImGui::GetItemRectSize();

	bool dirty = false;

#if 1
	if (ImGui::ColorEdit3("Fluid color", value_ptr(ssfptr->o_waterColor))) {}

	float murkiness = ssfptr->o_waterColor.w * 100;
	if (ImGui::DragFloat("Fluid murkiness", &murkiness, 0.01f, 0.f, 1.f)) {}
	ssfptr->o_waterColor.w = murkiness / 100;

	float particle_size = ssfptr->o_size * 5;
	if (ImGui::DragFloat("Particle size", &particle_size, 0.01f, 0.1f, 1.f)) {}
	ssfptr->o_size = particle_size / 5;
	ssfptr->o_densityThreshold = 0.4 + 0.6 * particle_size;

	int smoothness = ssfptr->o_filterSize - 4;
	if (ImGui::SliderInt("Smoothness", &smoothness, 0, 4)) {}
	ssfptr->o_filterSize = smoothness + 4;

	if (ImGui::Checkbox("Disable rendering", &ssfptr->o_bypass)) {}

	if (ImGui::Checkbox("Low quality (faster)", &ssfptr->o_cheapDensity)) {}

    if (ImGui::DragFloat("Refraction index", &ssfptr->o_refractionIndex, 0.001f, 0.f, 2.f)) {}
    

#else
    if (ImGui::Checkbox("Enable screen space reflecions/refractions", &ssfptr->o_enableSSRef)) {}
	if (ImGui::Button("Reload shaders")) { ssfptr->dirtyShaders = true; }
	if (ImGui::ColorEdit3("Fluid color", value_ptr(ssfptr->o_waterColor)) ) {}
	if (ImGui::DragFloat("Fluid murkiness", &ssfptr->o_waterColor.w, 0.001f, 0.f, 1.f)) {}
	if (ImGui::DragFloat("Flat color mix", &ssfptr->o_flatColorMix, 0.001f, 0.f, 1.f)) {}
	if (ImGui::DragFloat("Particle size", &ssfptr->o_size, 0.001f, 0.0f, 1.f)) {}
	if (ImGui::SliderInt("Bilateral filter size", &ssfptr->o_filterSize, 0, 15)) {}
	if (ImGui::DragFloat("Filter position weight", &ssfptr->o_filterWeightPos, 0.001f, 0.f, 3.f)) {}
	if (ImGui::DragFloat("Filter normal weight", &ssfptr->o_filterWeightNor, 0.001f, 0.f, 3.f)) {}
	if (ImGui::DragFloat("Normal smoothing", &ssfptr->o_normalSmooth, 0.001f, 0.f, 3.f)) {}
	if (ImGui::DragFloat("Refraction index", &ssfptr->o_refractionIndex, 0.001f, 0.f, 2.f)) {}
	if (ImGui::Checkbox("Enable screen space reflecions/refractions", &ssfptr->o_enableSSRef)) {}
	if (ImGui::DragFloat("Refraction multiplier", &ssfptr->o_reflectionMult, 0.01f, 0.f, 10.f)) {}
	if (ImGui::DragFloat("Refraction gamma", &ssfptr->o_reflectionGamma, 0.01f, 0.f, 10.f)) {}
	if (ImGui::DragFloat("Reflection multiplier", &ssfptr->o_refractionMult, 0.01f, 0.f, 10.f)) {}
	if (ImGui::DragFloat("Reflection gamma", &ssfptr->o_refractionGamma, 0.01f, 0.f, 10.f)) {}
	if (ImGui::DragFloat("Density threshold", &ssfptr->o_densityThreshold, 0.01f, 0.f, 5.f)) {}
	if (ImGui::Checkbox("Disable rendering", &ssfptr->o_bypass)) {}
	if (ImGui::Checkbox("Use normal filtering (old)", &ssfptr->o_useNormalTexFiltering)) {}
	if (ImGui::Checkbox("Use cheap density estimate", &ssfptr->o_cheapDensity)) {}
	if (ImGui::Checkbox("FXAA", &ssfptr->o_fxaa)) {}
	
	bool select;
	std::string typeStr;

	switch (ssfptr->shapeType)
	{
	case SHAPE_TYPE::ST_CUBE:
		typeStr = "Cube";
		break;
	case SHAPE_TYPE::ST_SPHERE:
		typeStr = "Sphere";
		break;
	case SHAPE_TYPE::ST_QUAD:
		typeStr = "Quad";
		break;
	case SHAPE_TYPE::ST_POINTSPRITE:
		typeStr = "Point sprite";
		break;
	}

	if (ImGui::BeginCombo("Particle type", typeStr.c_str())) {
		if (ImGui::Selectable("Sphere", &select))       { ssfptr->shapeType = SHAPE_TYPE::ST_SPHERE; ssfptr->dirtyShape = true; }
		if (ImGui::Selectable("Cube", &select))         { ssfptr->shapeType = SHAPE_TYPE::ST_CUBE; ssfptr->dirtyShape = true; }
		if (ImGui::Selectable("Quad", &select))         { ssfptr->shapeType = SHAPE_TYPE::ST_QUAD; ssfptr->dirtyShape = true; }
		if (ImGui::Selectable("Point sprite", &select)) { ssfptr->shapeType = SHAPE_TYPE::ST_POINTSPRITE; ssfptr->dirtyShape = true; }
		ImGui::EndCombo();
	}
#endif

}

