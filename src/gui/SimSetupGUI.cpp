#include "common.h"
#include "gui/SimSetupGUI.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/type_ptr.hpp>

SimSetupGUI::SimSetupGUI(SimSettings* ss)
{
	simSettings = ss;
	visible = true;
	displayname = "Simulation Setup (REQ. RESET!)";
}

SimSetupGUI::~SimSetupGUI()
{}


void SimSetupGUI::draw(GLFWwindow* window, int order)
{
	if (ImGui::Button("Reset Simulation")) { simSettings->doSetup = true; }

	ImGui::DragFloat("Reduction (0 = speed, 1 = quality)", &simSettings->reductionParam, 0.01, 0, 1);
	ImGui::DragFloat("Fluid res. (0 = speed, 1 = quality)", &simSettings->fluidQuality, 0.01, 0, 1);
	ImGui::DragFloat("Grid size (relative)", &simSettings->gridSize, 0.01, 0, 2);
	ImGui::Checkbox("Add pool", &simSettings->addPool);
	ImGui::DragFloat("Pool height (relative)", &simSettings->poolHeight, 0.01, 0.1, 2);
	ImGui::DragFloat("Pool gap", &simSettings->poolGap, 0.1, 0, 10);
	ImGui::DragFloat("Fluid stream", &simSettings->streamSize, 0.1, 0, 2);
	ImGui::Checkbox("Prevent intersections", &simSettings->preventSolidPen);
	
}