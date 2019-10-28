#include "common.h"
#include "gui/CameraGUI.h"
#include "managers/TextureManager.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/type_ptr.hpp>

CameraGUI::CameraGUI(CameraFPS* cam)
{
	setCamera(cam);
	visible     = true;
	displayname = "Camera options";
}

CameraGUI::~CameraGUI()
{}

void CameraGUI::setCamera(CameraFPS* cam)
{
	this->camptr = cam;
}

void CameraGUI::draw(GLFWwindow* window, int order)
{
	if (!camptr) return;

	ImVec2 outerSize = ImGui::GetItemRectSize();

	bool dirty = false;

	if (ImGui::DragFloat3("Position", glm::value_ptr(camptr->position), 0.01f + abs(glm::length(camptr->position) / 100.f ) )) dirty = true;
	//if (ImGui::DragFloat3("Orientation", glm::value_ptr(m.gl.diffuse), 0.01f, 0.0f, 1e12f)) dirty = true;
	if (ImGui::DragFloat("Vertical FOV", &camptr->verticalFov_deg, 0.05f, 0.0f, 180.f)) dirty = true;
	if (ImGui::DragFloat("Near", &camptr->nearplane, 0.0001f + camptr->nearplane / 100.f, 0.00001f, camptr->farplane - 0.0001f)) dirty = true;
	if (ImGui::DragFloat("Far", &camptr->farplane, 0.0001f + camptr->farplane / 100.f, camptr->nearplane + 0.0001f, 1e12f)) dirty = true;
	if (ImGui::DragFloat("Rotation speed", &camptr->rot_speed, 0.001f, 0.0f, 100.f)) dirty = true;
	if (ImGui::DragFloat("Motion speed", &camptr->mov_speed, 0.001f, 0.0f, 100.f)) dirty = true;

}