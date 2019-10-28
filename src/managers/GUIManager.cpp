#include "managers/GUIManager.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

void GUIManagerInstance::initialize(GLFWwindow* window)
{
	// Create a window called "My First Tool", with a menu bar.
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

	ImGui_ImplGlfw_InitForOpenGL(window, false);
	const char* glsl_version = "#version 450";
	ImGui_ImplOpenGL3_Init(glsl_version);
	ImGui::StyleColorsDark();

	ImGui::GetIO().IniFilename = nullptr;

	this->window = window;
}

void GUIManagerInstance::finalize(GLFWwindow* window)
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

int GUIManagerInstance::addGUI(std::string name, GUIptr gui)
{
	if (name.empty() || guis.count(name) != 0) {
		return ERROR_INCORRECT_NAME;
	}

	if (gui == nullptr) {
		return ERROR_INVALID_POINTER;
	}

	guis[name] = gui;

	return SUCCESS;
}

int GUIManagerInstance::removeGUI(std::string name)
{
	if (name.empty() || guis.count(name) == 0) {
		return ERROR_INCORRECT_NAME;
	}

	guis.erase(name);

	return SUCCESS;
}

void GUIManagerInstance::draw()
{
	bool anyvisible = false;
	for (auto& it : guis) anyvisible = anyvisible || it.second->visible;
	if (!anyvisible) return;

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	
	if (false) {
		ImGui::ShowTestWindow();
	}
		
	ImGui::SetNextWindowPos(ImVec2(10.f, 10.f), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(500.f, 500.f), ImGuiCond_Once);
	ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
	ImGui::Begin("Options", nullptr, ImGuiWindowFlags_NoSavedSettings);

	int order = 0;
	for (auto& it : guis) 
	{
		GUIptr& gui = it.second;
		if (gui->visible) {
			if (ImGui::TreeNode(gui->displayname.c_str())) {
				gui->draw(window, order);
				ImGui::TreePop();
			}
		}
		order++;
	}

	ImGui::Text("\n\nUse AWSD for movement.\n\nPress F to toggle mouse camera motion.\n\n", "");


	ImGui::End();
	ImGui::EndFrame();
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
