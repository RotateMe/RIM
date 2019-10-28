#include "common.h"

#include "helpers/CameraFPS.h"
#include "helpers/OpenGLProgramObject.h"
#include "helpers/OpenGLTextureObject.h"
#include "helpers/OpenGLFramebufferObject.h"
#include "helpers/OpenGLVertexNameRenderer.h"
#include "helpers/MeshData.h"
#include "helpers/LightData.h"
#include "helpers/ScopeTimer.h"

#include "gui/MeshGUI.h"
#include "gui/LightGUI.h"
#include "gui/CameraGUI.h"
#include "gui/RenderGUI.h"
#include "gui/SimSetupGUI.h"

#include "managers/GLStateManager.h"
#include "managers/TextureManager.h"
#include "managers/GUIManager.h"
#include "managers/GLShapeManager.h"

#include "immersed/screen_space_fluid.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
#include <freeimage.h>
#include <tiny_obj_loader.h>

#include <iostream>

#include <fstream>

#include "dynamics/ProjDynTypeDef.h"
#include "dynamics/ProjDynSimulator.h"
//#include "main.h"

using namespace glm;

int loadFluidPositions(const std::string& filename, std::vector<vec3>& positions)
{
	positions.clear();

	std::ifstream ifile;
	ifile.open(filename, std::ios::binary);
	if (!ifile.is_open()) return ERROR_FILE_NOT_FOUND;

	ifile.seekg(0, std::ios::end); size_t sz = size_t(ifile.tellg()); ifile.seekg(0, std::ios::beg);

	if (sz == 0) return ERROR_LOADING_RESOURCE;
	
	positions.resize(sz / sizeof(vec3) );
	ifile.read((char*)(positions.data()), sz);
	ifile.close();

	return SUCCESS;
}

bool display_gui = true;
bool full_screen = false;

RenderStats renderstats;

int window_width  = 1280;
int window_height = 720;

GLHelpers::ProgramObject render_program, background_program, depth_program;

GLHelpers::FramebufferObject fbo;
GLHelpers::TextureObject2D   depthtex, colortex;

CameraFPS camera, prev_camera;
vec3 camera_speed(0.f, 0.f, 0.f);
vec2 camera_rotational_speed(0.f, 0.f);
bool cursor_lock = false;

OpenGLVertexNameRenderer name_renderer;

using namespace GLHelpers;

ProjDynSimulator* sim = NULL;

std::vector<unsigned int> grippedVerts;
bool gripping = false;
vec3 gripWorldCenter(0, 0, 0);
vec4 gripScreenCenter(0, 0, 0, 0);
PDPositions grippos;

void resize_textures(GLint width, GLint height)
{
	if (depthtex.width != width || depthtex.height != height || depthtex.internalFormat != GL_DEPTH_COMPONENT32F) {
		depthtex.create(ivec2(window_width, window_height), GL_DEPTH_COMPONENT32F, nullptr, true);
		depthtex.setParameter(GL_TEXTURE_COMPARE_MODE, GL_NONE);
		depthtex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	if (colortex.width != width || colortex.height != height || colortex.internalFormat != GL_RGBA16F) {
		colortex.create(glm::ivec2(width, height), GL_RGBA16F, nullptr, true);
		colortex.setWrapMethod(GL_CLAMP_TO_EDGE);
	}

	name_renderer.setup(size2D(width, height));
}

static void resize_callback(GLFWwindow* window, int width, int height)
{
	window_width = width;
	window_height = height;

	camera.aspect = window_width / float(window_height);

	resize_textures(window_width, window_height);
}

static void camera_key_callback(int key, int scancode, int action, int mods)
{
	if (action != GLFW_PRESS && action != GLFW_RELEASE) return;

	if (key == GLFW_KEY_W) camera_speed.z = action == GLFW_PRESS ?  1.0f : 0.0f;
	if (key == GLFW_KEY_S) camera_speed.z = action == GLFW_PRESS ? -1.0f : 0.0f;
	if (key == GLFW_KEY_D) camera_speed.x = action == GLFW_PRESS ?  1.0f : 0.0f;
	if (key == GLFW_KEY_A) camera_speed.x = action == GLFW_PRESS ? -1.0f : 0.0f;
	if (key == GLFW_KEY_E) camera_speed.y = action == GLFW_PRESS ?  1.0f : 0.0f;
	if (key == GLFW_KEY_Q) camera_speed.y = action == GLFW_PRESS ? -1.0f : 0.0f;

}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);

	if (key == GLFW_KEY_F) {
		if (action == GLFW_PRESS) cursor_lock = !cursor_lock;
		
		if (cursor_lock) {
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN | GLFW_CURSOR_DISABLED);
			glfwSetCursorPos(window, window_width/2.0, window_height/2.0);
		} else {
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
	}

	if (key == GLFW_KEY_M && action == GLFW_PRESS) {
		display_gui = !display_gui;
	}

	camera_key_callback(key, scancode, action, mods);
} 

int mouseX = -1, mouseY = -1, selectionRadius=25;
static void cursor_callback(GLFWwindow* window, double xpos, double ypos)
{
	double half_width  = window_width  / 2.0;
	double half_height = window_height / 2.0;

	mouseX = xpos;
	mouseY = window_height - ypos;

	if (!cursor_lock) {
		if (gripping) {
			//printf("Moving grip \n");
			mat4 camtrans = camera.proj_matrix() *
				camera.view_matrix();
			mat4 p = glm::inverse(camtrans);

			vec4 newScreenCenter = gripScreenCenter;

			newScreenCenter.x = (((float)mouseX / (float)half_width) - 1.f) * newScreenCenter.w;
			newScreenCenter.y = (((float)mouseY / (float)half_height) - 1.f) * newScreenCenter.w;
			newScreenCenter = p * newScreenCenter;

			vec3 trans(newScreenCenter.x - gripWorldCenter.x,
				newScreenCenter.y - gripWorldCenter.y,
				newScreenCenter.z - gripWorldCenter.z);
			PD3dVector transEigen(trans.x, trans.y, trans.z);

			PDPositions newPos(grippos.rows(), 3);
			for (unsigned int v = 0; v < grippos.rows(); v++) {
				newPos.row(v) = grippos.row(v) + transEigen.transpose();
			}
			sim->setGrip(grippedVerts, newPos);
		}
	}
	else {
		// First person camera mouse movement
		if (xpos == half_width && ypos == half_height) return;

		camera_rotational_speed = vec2(float(xpos - half_width), float(ypos - half_height));
		glfwSetCursorPos(window, half_width, half_height);
	}

	return;
}

static void click_callback(GLFWwindow* window, int button, int action, int mods) {
	//printf("Mouse button event \n");
	// Do nothing in first person mode
	if (cursor_lock) return;

	// Do nothing if simulator is not set
	if (sim == NULL) return;

	if (action == GLFW_PRESS) {
		//printf("Gripping initial \n");
		if (gripping) {
			//printf("unexpected release \n");
			sim->releaseGrip();
			gripping = false;
		}
		if (mouseX >= 0 && mouseY >= 0 && mouseX <= window_width && mouseY <= window_height) {
			// Check for vertices below mouse
			float* namebuffer = new float[selectionRadius*selectionRadius * 4];
			for (int i = 0; i < selectionRadius*selectionRadius * 4; i++) namebuffer[i] = -1;
			glGetTextureSubImage(name_renderer.nametex.m_uiOpenGLID, 0,
				std::max(0, mouseX - selectionRadius), std::max(0, mouseY - selectionRadius), 0,
				std::min(window_width - mouseX - 1, selectionRadius * 2), std::min(window_height - mouseY - 1, selectionRadius * 2), 1,
				name_renderer.nametex.dataFormat, name_renderer.nametex.dataType, selectionRadius * selectionRadius * 4 * sizeof(float), namebuffer);
			GLenum err = glGetError();
			String errCode = "";
			if (err != GL_NO_ERROR) {
				switch (err) {
				case GL_INVALID_VALUE:
					errCode = "GL_INVALID_VALUE";
					break;
				case GL_INVALID_OPERATION:
					errCode = "GL_INVALID_OPERATION ";
					break;
				default:
					errCode = "Unknown error!";
				}
				std::cout << "GL error on gettexturesubimage: " << errCode << std::endl;
			}
			grippedVerts.clear();
			std::vector<unsigned int>& simVertices = sim->getUsedVertices();
			for (int i = 0; i < selectionRadius*selectionRadius * 4; i++) {
				if (namebuffer[i] >= 0) {
					unsigned int v = (unsigned int)namebuffer[i];
					if (std::find(grippedVerts.begin(), grippedVerts.end(), v) == grippedVerts.end()) {
						if (std::find(simVertices.begin(), simVertices.end(), v) != simVertices.end()) {
							grippedVerts.push_back(v);
						}
					}
				}
			}
            std::sort(grippedVerts.begin(), grippedVerts.end());
            grippedVerts.erase(std::unique(grippedVerts.begin(), grippedVerts.end()), grippedVerts.end());
			// Determine world and screen space center of the grip (requires camera transform)
			if (grippedVerts.size() > 0) {
				mat4 camtrans = camera.proj_matrix() * camera.view_matrix();
				PDPositions& curpos = sim->getPositions();
				grippos.setZero(grippedVerts.size(), 3);
				gripWorldCenter = vec3(0, 0, 0);
				gripScreenCenter = vec4(0, 0, 0, 0);

                grippos.setZero(grippedVerts.size(), 3);
                int ind = 0;
                for (unsigned int v : grippedVerts) {
                    grippos.row(ind) = curpos.row(v);
                    ind++;
                }
				for (int i = 0; i < grippedVerts.size(); i++) {
					vec3 vp(grippos(i, 0), grippos(i, 1), grippos(i, 2));
					gripWorldCenter += vp;
					vec4 vpp(grippos(i, 0), grippos(i, 1), grippos(i, 2), 1);
					gripScreenCenter += camtrans * vpp;
				}
				gripWorldCenter /= (float)grippedVerts.size();
				gripScreenCenter /= (float)grippedVerts.size();
				// Set grip in simulator
				sim->setGripType(false);
                sim->setGrip(grippedVerts, grippos);
				gripping = true;
			}
		}
	}
	else {
		//printf("Releasing grip \n");
		if (gripping) {
			sim->releaseGrip();
			gripping = false;
		}
	}
};

void setup_program(ProgramObject& program, std::string vertex_filename, std::string fragment_filename = std::string(), std::string geometry_filename = std::string())
{

	ProgramObject::ShaderPaths p;
	p.commonPath = "shaders/";
	p.vertexShaderFilename = vertex_filename;
	p.fragmentShaderFilename = fragment_filename;
	p.geometryShaderFilename = geometry_filename;

	GLHelpers::Detail::ShaderCommandLine cmd;
	cmd.m_Defines.push_back(GLHelpers::NameValue(std::string("LINEARIZE_DEPTH"), std::string("0") ));

	try {
		program.CompileProgram(p, nullptr, cmd);
	} catch (std::runtime_error(e)) {
		std::cout << "Error creating opengl program" << std::endl;
	}

}

void load_programs() {
	setup_program(depth_program, "base_depth.vert", "base_depth.frag");
	setup_program(background_program, "quad.vert", "background.frag");
	setup_program(render_program, "base.vert", "base_shade.frag", "base.geom");
}

void clearScreen()
{
	GLState state = GLStateManager::instance().getState();
	state.enable_depth_write = true;
	GLStateManager::instance().setState(state, true);

	glClearDepth(1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	check_opengl();
}

void renderBackground(TextureObject2D tex)
{
	background_program.Use();

	bool hasCopyImage = tex.m_uiOpenGLID;

	fbo.EnableConsecutiveDrawbuffers(1, 0);

	if (hasCopyImage) {
		background_program.SetTexture("map", tex, 0);
	} else {
		clearScreen();
		return;
	}


	GLState state;
	state.setViewportBox(0, 0, window_width, window_height);
	state.enable_depth_test  = false;
	state.enable_depth_write = false;
	state.enable_blend = false;
	state.enable_cull_face = false;
	GLStateManager::instance().setState(state, true);

	mat4 vMatrix = camera.view_matrix();
	mat4 pMatrix = camera.proj_matrix();
	background_program.SetUniform("vpMatrixInv", inverse(pMatrix * vMatrix) );
	background_program.SetUniform("wsCamPos", camera.position);

	glBindVertexArray(0);

	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
}

void renderDepth(MeshData& mesh)
{
	depth_program.Use();

	fbo.EnableConsecutiveDrawbuffers(0);

	GLState state;
	state.setViewportBox(0, 0, window_width, window_height);
	state.enable_cull_face = true;
	state.cull_face = GL_BACK;
	state.enable_depth_test = true;
	state.enable_depth_write = true;
	state.depth_test_func = GL_LESS;
	GLStateManager::instance().setState(state, true);

	mat4 mMatrix, vMatrix, pMatrix;

	vMatrix = camera.view_matrix();
	pMatrix = camera.proj_matrix();

	mMatrix = identity<mat4>();

	depth_program.SetUniform("mvpMatrix", pMatrix * vMatrix * mMatrix);
	depth_program.SetUniform("cam_nearfar", vec2(camera.nearplane, camera.farplane));

	mesh.gl.vao.Bind();
	mesh.gl.indexbuffer.Bind();

	const int bytesPerElement = mesh.gl.indexbuffertype == GL_UNSIGNED_INT ? 4 : 2;
	GLsizei total_elements = GLsizei(mesh.gl.indexbuffer.m_sizeInBytes / bytesPerElement);
	glDrawElements(GL_TRIANGLES, total_elements, mesh.gl.indexbuffertype, OFFSET_POINTER(0));

	check_opengl();
}

void render(MeshData& mesh, LightData& lights, bool flat = false)
{
	render_program.Use();

	fbo.EnableConsecutiveDrawbuffers(1);

	GLState state;
	state.setViewportBox(0, 0, window_width, window_height);
	state.enable_cull_face = true;
	state.cull_face = GL_BACK;
	state.enable_depth_test = true;
	state.enable_depth_write = true;
	state.depth_test_func = GL_LEQUAL;
	GLStateManager::instance().setState(state, true);

	mat4 mMatrix, vMatrix, pMatrix;
	mat3 nMatrix;

	vMatrix = camera.view_matrix();
	pMatrix = camera.proj_matrix();

	mMatrix = identity<mat4>();
	nMatrix = transpose(inverse(mMatrix));

	render_program.SetUniform("mMatrix", mMatrix);
	render_program.SetUniform("mvpMatrix", pMatrix * vMatrix * mMatrix);
	render_program.SetUniform("normalMatrix", nMatrix);
	render_program.SetUniform("wsCamPos", camera.position);
	render_program.SetUniform("enableAlphaTest", DefaultRenderOptions().alphatest);
	render_program.SetUniform("cam_nearfar", vec2(camera.nearplane, camera.farplane));
	render_program.SetUniform("useFlatNormals", flat);

	check_opengl();

	render_program.SetUniform("light_qty", GLint(lights.gl.lightList.size()));
	render_program.SetUniform("ambient_light", lights.gl.ambient_light);

	check_opengl();

	mesh.gl.vao.Bind(); 
	mesh.gl.indexbuffer.Bind();

	render_program.SetSSBO("material_ssbo", mesh.materialdata.materialListBuffer, 0);
	render_program.SetSSBO("light_ssbo", lights.gl.lightListBuffer, 1);

	check_opengl();

	const int bytesPerElement = mesh.gl.indexbuffertype == GL_UNSIGNED_INT ? 4 : 2;
	GLsizei total_elements = GLsizei(mesh.gl.indexbuffer.m_sizeInBytes / bytesPerElement);
	glDrawElements(GL_TRIANGLES, total_elements, mesh.gl.indexbuffertype, OFFSET_POINTER(0) );

	check_opengl();
}

void renderFluid(ScreenSpaceFluid& ssfluid)
{
	ssfluid.sceneDepthTex = depthtex;
	ssfluid.sceneColorTex = colortex;
	ssfluid.Render(camera);
	check_opengl();
}

/* Constructs a simulator object, sets constraints, adds gravity, a floor with friction and
triggers the precomputation. */
ProjDynSimulator* initSimulator(MeshData& deformable, SimSettings& simSettings, std::string meshURL, bool directBufferMapping, BufferObject* fluidBuffer = NULL) {
	// Turn deformable into Eigen matrices
	int numVerts = deformable.positions.size();
	PDPositions verts(numVerts, 3);
	PDPositions velos(numVerts, 3);
	velos.setZero();
	for (int i = 0; i < numVerts; i++) {
		verts(i, 0) = deformable.positions[i].x;
		verts(i, 1) = deformable.positions[i].y;
		verts(i, 2) = deformable.positions[i].z;
	}
	int numTris = deformable.indices.size() / 3;
	PDTriangles tris(numTris, 3);
	for (int i = 0; i < numTris; i++) {
		tris(i, 0) = deformable.indices[i * 3 + 0];
		tris(i, 1) = deformable.indices[i * 3 + 1];
		tris(i, 2) = deformable.indices[i * 3 + 2];
	}
	// Construct simulator object from this mesh and provide initial velcocities,
	// as well as a few reduction and simulation parameters, which are explained
	// in the _README.txt
	double timeStep = 0.016666;
	float p = simSettings.reductionParam;
	int numberSamplesForVertexPosSubspace = (1 - p) * 20 + p * 200; // The number of degrees of freedom for the mesh vertex positions will be 12 times that
	double radiusMultiplierForVertexPosSubspace = 1.1 + p * 0.3; // The larger this number, the larger the support of the base functions.
	int dimensionOfConstraintProjectionsSubspace = 120 + p * 120; // The constraint projections subspace will be constructed to be twice that size and then condensed via an SVD
	double radiusMultiplierForConstraintProjectionsSubspace = 2.7;
	int numberSampledConstraints = 300 + p * 1000; // Number of constraints that will be evaluated each iteration
	ProjDynSimulator* sim =
		new ProjDynSimulator(tris, verts, velos, timeStep,
			numberSamplesForVertexPosSubspace, radiusMultiplierForVertexPosSubspace,
			dimensionOfConstraintProjectionsSubspace, radiusMultiplierForConstraintProjectionsSubspace,
			numberSampledConstraints,
			1000, 0, true, meshURL,
			0., 8, 1.5, 10000,
			4);

	// For the simulation to be meaningful we need some sorts of constraints
	// The following method adds volume preservation constraints to all tets
	sim->addTetStrain(125000, 1.f, 1.f);

	// We add gravity and floor friction+repulsion to the simulation
	float floorHeight = 0;
	sim->addGravity(9.81);
	sim->addFloor(1, floorHeight, 1);
	sim->setFrictionCoefficient(0.5, 0.05);

	// Add fluid
	float fq = simSettings.fluidQuality;
	sim->addPICFLIP(20 + fq*80, simSettings.preventSolidPen, simSettings.addPool, simSettings.streamSize > 0, simSettings.poolGap, simSettings.poolHeight, simSettings.gridSize, 1, true, floorHeight, false, simSettings.streamSize);
	sim->setPICFLIPParams(9, 1, 0.03, 50, false, 0.5, 0, 1, 1, 1, 0, 0.5, 150);

	// The call to the setup function computes the subspaces for vertex
	// positions and constraint projections and factorizes all linear systems
	// for local and global steps
	sim->setup();

	if (directBufferMapping) {
		// Connect deformable positions buffer to simulator
		// I have to reload the mesh here, since the buffers are broken
		// after initializing the fluid simulation for some reason
		if (deformable.load_obj(meshURL, 0.f, false) != SUCCESS)
		{
			std::cerr << "Error loading model file " << meshURL << std::endl;
			glfwTerminate();
			return NULL;
		}
		sim->initializeGPUVPosMapping(deformable.gl.posbuffer.m_uiBufferIdx);
	}

	// Connect particle buffer to screenspace fluid
	if (sim->hasParticles() && fluidBuffer) {
		fluidBuffer->m_uiBufferIdx = sim->m_particleBufferID;
		fluidBuffer->m_sizeInBytes = sim->getNumFreeParticles() * sizeof(float) * 3;
		fluidBuffer->m_usage = GL_STATIC_DRAW;
	}

	return sim;
}


int main()
{
	//////////////// 
	FreeImage_Initialise();

	////////////////
	if (!glfwInit())
	{
		std::cerr << "Error initializing GLFW." << std::endl;
		return -1;
	}
	std::cout << "Initialized glfw." << std::endl;
	glfwSetErrorCallback(errorCallbackGLFW);

	////////////////

	////////////////
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
	glfwWindowHint(GLFW_DOUBLEBUFFER, true);
	glfwWindowHint(GLFW_DEPTH_BITS, 0);

	GLFWwindow* window = glfwCreateWindow(window_width, window_height, "Immersed Real-Time Fluids", NULL, NULL);
	if (!window)
	{
		std::cerr << "Error creating window." << std::endl;
		glfwTerminate();
		return -1;
	}
	std::cout << "Created window." << std::endl;

	////////////////
	glfwSetWindowSizeCallback(window, resize_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, cursor_callback);
	glfwSetMouseButtonCallback(window, click_callback);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);

	////////////////
	GLenum glewInitResult = glewInit();
	if (glewInitResult != GL_NO_ERROR) {
		std::cerr << "Error creating window." << std::endl;
		glfwTerminate();
		return -1;
	}
	std::cout << "Initialized glew." << std::endl;


	//////////////// Default render options
	DefaultRenderOptions().vsync = false;
	DefaultRenderOptions().alphatest = false;
	for (auto& o : DefaultRenderOptions().misc_options) o = 0.5f;

	////////////////
	// During init, enable debug output
#ifdef _DEBUG
	glEnable(GL_DEBUG_OUTPUT);
	glDebugMessageCallback(DebugCallbackGL, 0);
#endif

	//////////////// 
	fbo.Create();
	resize_textures(window_width, window_height);
	check_opengl();

	////////////////
	MeshData scene_mesh;
	std::string scene_mesh_filename = "data/immersed/scenes/floor.obj";
	if (scene_mesh.load_obj(scene_mesh_filename, 0.f, false) != SUCCESS)
	{
		std::cerr << "Error loading model file " << scene_mesh_filename << std::endl;
		glfwTerminate();
		int test = 0;
		std::cin >> test;
		return -1;
	}

	/////////////////
	LightData lights;
	std::string lights_filename = "data/immersed/baselight.json";
	int lightsLoadResult = lights.loadFromJSON(lights_filename);
	if (lightsLoadResult != SUCCESS)
	{
		std::cerr << "Error loading light file " << lights_filename << std::endl;
		glfwTerminate();
		return -1;
	}

	////////////////
	load_programs(); 
	std::cout << "Created opengl program." << std::endl;

	////////////////
	camera.reset();
	camera.position = vec3(0.0f, 5.0f, 16.0f);
	camera.verticalFov_deg = 45;
	camera.aspect = window_width / float(window_height);
	camera.nearplane = 0.1f;
	camera.farplane  = 2000.f;
	camera.mov_speed = 10.0f;
	camera.rot_speed = 0.001f;
	prev_camera = camera;

	////////////////
	GLStateManager::instance().resetState(true);

	////////////////
	GUIManager::instance().initialize(window);
	GUIManager::instance().addGUI("camgui", std::shared_ptr<BaseGUI>(new CameraGUI(&camera)));
	GUIManager::instance().addGUI("meshgui", std::shared_ptr<BaseGUI>(new MeshGUI(&scene_mesh)));
	GUIManager::instance().addGUI("lightgui", std::shared_ptr<BaseGUI>(new LightGUI(&lights)));
	GUIManager::instance().addGUI("rendergui", std::shared_ptr<BaseGUI>(new RenderGUI(&DefaultRenderOptions(), &renderstats)));

	////////////////
	GLShapeManager::instance().initialize();

	////////////////
	name_renderer.setup(size2D(window_width, window_height));


	//////////////// ssfluid
	ScreenSpaceFluid ssfluid;
	ssfluid.InitResources();
	//std::string equimap_name = "data/envmaps/werfenweng_Austria.exr";
	std::string equimap_name = "data/envmaps/sky.exr";
	TextureManager::instance().loadTexture(equimap_name, true);
	if (TextureManager::instance().hasTexture(equimap_name))
	{
		TextureData& texdata = TextureManager::instance().getTexture(equimap_name);
		ssfluid.envTex = *((GLHelpers::TextureObject2D*)(texdata.gltex)); //!!
	}
	std::vector<vec3> fluid_positions;
	// Some placeholder fluid positions, in case no fluid is in the simulation
	//loadFluidPositions("data/immersed/positions.bin", fluid_positions);
	ssfluid.fluidPosition.Create(GL_ARRAY_BUFFER);
	//ssfluid.fluidPosition.UploadData(fluid_positions.size() * sizeof(vec3), GL_STATIC_DRAW, fluid_positions.data());
	GUIManager::instance().addGUI("ssfluidgui", std::shared_ptr<BaseGUI>(new ScreenSpaceFluid::GUI(&ssfluid)));

	//////////////// projective dynamics
	// Load deformable
	MeshData deformable;
	std::string deformable_filename = "data/immersed/Meshes/armadillo_simple_fixed.obj";
	if (deformable.load_obj(deformable_filename, 0.f, false) != SUCCESS)
	{
		std::cerr << "Error loading model file " << deformable_filename << std::endl;
		glfwTerminate();
		return -1;
	}
	std::cout << "Loaded deformable." << std::endl;
	std::cout << "Number of verts: " << deformable.positions.size() << ", number of indices: " << deformable.indices.size() << "." << std::endl;

	// Setup simulation
	std::cout << "Converted deformable, setting up simulation" << std::endl;
	bool directBufferMapping = true;
	SimSettings simSettings;
	sim = initSimulator(deformable, simSettings, deformable_filename, directBufferMapping, &ssfluid.fluidPosition);
	GUIManager::instance().addGUI("pdsimgui", std::shared_ptr<BaseGUI>(new ProjDynSimulator::GUI(sim)));
	GUIManager::instance().addGUI("pdsetupgui", std::shared_ptr<BaseGUI>(new SimSetupGUI(&simSettings)));

	////////////////
	std::cout << "Starting main loop." << std::endl;
	int numVerts = deformable.positions.size();
	check_opengl();
	std::vector<float> curPos;
	curPos.resize(numVerts * 3);
	while (!glfwWindowShouldClose(window))
	{
		PROFILE_SCOPE("Main loop")

		glfwMakeContextCurrent(window);

		{ PROFILE_SCOPE("Render")
			//Render scene, deformable and fluid
			lights.updateGLResources();
			scene_mesh.materialdata.updateGLResources();
			deformable.materialdata.updateGLResources();

			name_renderer.render(size2D(window_width, window_height), camera, deformable.gl.posbuffer, 15.f);

			fbo.Bind();
			fbo.AttachDepthTexture(depthtex);
			fbo.AttachColorTexture(colortex, 0);
			fbo.EnableConsecutiveDrawbuffers(1);

			clearScreen();
			renderBackground(ssfluid.envTex);
			render(scene_mesh, lights);
			render(deformable, lights, true);
			renderFluid(ssfluid);

			// Perform Simulation step
			sim->step();

			if (!directBufferMapping) {
				// Upload current deformable positions to buffer
				PDPositions& curVerts = sim->getPositions();
				PROJ_DYN_PARALLEL_FOR
					for (int v = 0; v < numVerts; v++) {
						curPos[v * 3 + 0] = curVerts(v, 0);
						curPos[v * 3 + 1] = curVerts(v, 1);
						curPos[v * 3 + 2] = curVerts(v, 2);
					}
				deformable.gl.posbuffer.UploadData(sizeof(float) * numVerts * 3, GL_STATIC_DRAW, curPos.data());
			}

			// Reset simulation if desired by user
			if (simSettings.doSetup) {
				// Remove old simulator
				GUIManager::instance().removeGUI("pdsimgui");
				delete sim;

				// Setup new simulator and add updated GUI
				sim = initSimulator(deformable, simSettings, deformable_filename, directBufferMapping, &ssfluid.fluidPosition);
				GUIManager::instance().addGUI("pdsimgui", std::shared_ptr<BaseGUI>(new ProjDynSimulator::GUI(sim)));

				simSettings.doSetup = false;
			}
		}

		fbo.Unbind();
		check_opengl();

		//// Copy fbo contents to default window buffer
		glBlitNamedFramebuffer( //name_renderer.fbo.m_uiOpenGLID, 0, 0, 0, window_width, window_height, 0, 0, window_width, window_height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
			fbo.m_uiOpenGLID, 0, 0, 0, window_width, window_height, 0, 0, window_width, window_height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

		//// Draw GUI
		if (display_gui) GUIManager::instance().draw();

		//// Swap buffers
		glfwSwapBuffers(window);

		//// Update timings
		TimingStats s = TimingManager::instance().getTimingStats("Render");
		renderstats.time_cpu_ms  = s.cpu_time;
		renderstats.time_gl_ms   = s.gl_time;
		renderstats.time_cuda_ms = s.cuda_time;
		renderstats.fps = 1000. / (s.cpu_time);
		TimingManager::instance().endFrame();  

		//// Get input 
		glfwPollEvents();

		//// Update camera
		s = TimingManager::instance().getTimingStats("Main loop");
		float time = std::max(s.gl_time, std::max(s.cpu_time, s.cuda_time));
		
		prev_camera = camera;
		if (camera_rotational_speed != vec2(0.f, 0.f) || camera_speed != vec3(0.f, 0.f, 0.f)) {
			camera.update(camera_rotational_speed, camera_speed * time * 0.001f);
			camera_rotational_speed = vec2(0.f, 0.f);
			camera.moved = true;
		} else {
			camera.update(camera_rotational_speed, camera_speed);
			camera.moved = false;
		}

		//// Update vsync options
		glfwSwapInterval(DefaultRenderOptions().vsync ? 1 : 0);
		 
		//// Load programs again if requested
		if (DefaultRenderOptions().dirtyShaders) {
			load_programs();
			ssfluid.setDirtyShaders();
			name_renderer.reload_programs();
		}
	}

	render_program.Delete();
	background_program.Delete();
	depth_program.Delete();

	fbo.Delete();
	depthtex.Release();
	colortex.Release();

	ssfluid.ReleaseResources();


	// Clear textures before leaving context
	TextureManager::instance().clear();

	GUIManager::instance().finalize(window);

	glfwDestroyWindow(window);

	glfwTerminate();

	return 0;
}