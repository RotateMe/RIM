#pragma once


#include "helpers/OpenGLBufferObject.h"
#include "helpers/OpenGLTextureObject.h"
#include "helpers/OpenGLFramebufferObject.h"
#include "helpers/OpenGLProgramObject.h"
#include "helpers/CameraFPS.h"
#include "gui/BaseGUI.h"

using namespace GLHelpers;

class ScreenSpaceFluid
{
public:

	class GUI : public BaseGUI
	{
	public:
		GUI(ScreenSpaceFluid* ssf = nullptr);
		virtual ~GUI();
		virtual void draw(GLFWwindow* window, int order = 0);
	private:
		ScreenSpaceFluid* ssfptr;
	};



public:

	ScreenSpaceFluid(  );

	//void									InitializePorts();

	void									InitResources(  );
	void									ReleaseResources(  );

	//void                                    Instrument();
	//void									Serialize(  );

	void									Render(CameraFPS& cam);

private:

	void                                    loadQuadTex();

	enum class SHAPE_TYPE
	{
		ST_SPHERE        = 0,
		ST_CUBE          = 1,
		ST_QUAD          = 2,
		ST_POINTSPRITE   = 3,
	};

	void createPrograms();
	void createTextures(glm::ivec2 size);


public:

	void setDirtyShaders();

	// In
	TextureObject2D                         sceneColorTex;
	TextureObject2D                         sceneDepthTex;
	
	BufferObject                            fluidPosition;
	BufferObject                            fluidColor;

	TextureObjectCube						envCubeTex;
	TextureObject2D  						envTex; 

	// Options
	bool                                    o_bypass;
	float                                   o_flatColorMix;
	glm::vec4                               o_waterColor;
	float                                   o_size;
	String                                  o_quadtexname;
	bool                                    o_useNormalTexFiltering; // Decides between filtering normals or geometry

	int                                     o_filterSize;
	float                                   o_filterWeightPos;
	float                                   o_filterWeightNor;
	float                                   o_normalSmooth;
	float                                   o_refractionIndex;
	float                                   o_reflectionMult;
	float                                   o_reflectionGamma;
	float                                   o_refractionMult;
	float                                   o_refractionGamma;
	bool                                    o_enableSSRef;
	float                                   o_densityThreshold;
	bool                                    o_cheapDensity;
	bool                                    o_fxaa;

private:

	// Internal
	FramebufferObject			fbo;

	bool                        dirtyShaders;
	ProgramObject				programBackDepth;
	ProgramObject				programBackDepthPointSprite;
	ProgramObject				programDensity;
	ProgramObject				programDensityPointSprite;
	ProgramObject				programDensityCheap;
	ProgramObject				programGeometry;
	ProgramObject				programGeometryPointSprite;
	ProgramObject				programBilateralFilter;
	ProgramObject				programBilateralFilterSeparable;
	ProgramObject				programSSFluid;
	ProgramObject				programFXAA;

	glm::ivec2                  outputSize;

	TextureObject2D				depthTex;
	TextureObject2D				backDepthTex;
	TextureObject2D				normalTex;
	TextureObject2D				geometryTex;
	TextureObject2D				geometryBilateralAuxTex;
	TextureObject2D				geometryBilateralAuxTex2;
	TextureObject2D				normalBilateralAuxTex;
	TextureObject2D				normalBilateralAuxTex2;
	TextureObject2D				sceneColorAuxTex;
	TextureObject2D				densityTex;
	TextureObject2D				quadTex;

	bool                                    dirtyShape;

	VertexArrayObject           positionsVAO;

	VertexArrayObject           shapeVAO;
	BufferObject         		shapePosBuffer;
	BufferObject         		shapeNorBuffer;
	BufferObject         		shapeTexCoordBuffer;
	BufferObject         		shapeIndexBuffer;

	VertexArrayObject           quadVAO;
	BufferObject         		quadPosBuffer;
	BufferObject         		quadNorBuffer;
	BufferObject         		quadTexCoordBuffer;
	BufferObject         		quadIndexBuffer;

	void loadShape(SHAPE_TYPE type);
	SHAPE_TYPE shapeType;

	SamplerObject               sampler;
};
