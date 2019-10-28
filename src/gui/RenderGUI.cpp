#include "common.h"
#include "gui/RenderGUI.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/type_ptr.hpp>

RenderGUI::RenderGUI(RenderOptions* options, RenderStats* stats)
{
	visible     = true;
	fullscreen  = false;
	displayname = "Render options";
	setOptions(options);
	setStats(stats);
}

void RenderGUI::setOptions(RenderOptions* options)
{
	this->options = options;
	this->options->dirtyShaders = false;
}

void RenderGUI::setStats(RenderStats* stats)
{
	this->stats = stats;
}

RenderGUI::~RenderGUI()
{}

void RenderGUI::draw(GLFWwindow* window, int order)
{

	ImVec2 outerSize = ImGui::GetItemRectSize();

	bool dirty = false;
	this->options->dirtyShaders = false;

	ImGui::Text("FPS: %.3f", stats->fps);

	ImGui::Checkbox("Enable apha test", &options->alphatest);

	////////////////////////////////////////////////////
	////////////////////////// Timing display
	////////////////////////////////////////////////////
	ImGui::Text("Render time cpu: %.3f", stats->time_cpu_ms);
	ImGui::Text("Render time opengl: %.3f", stats->time_gl_ms);
	ImGui::Text("Render time cuda: %.3f", stats->time_cuda_ms);

	////////////////////////////////////////////////////
	////////////////////////// Reload shaders button
	////////////////////////////////////////////////////
	
	if (ImGui::Button("Reload shaders")) {
		this->options->dirtyShaders = true;
	} else {
		this->options->dirtyShaders = false;
	}

	////////////////////////////////////////////////////
	////////////////////////// Full screen options
	////////////////////////////////////////////////////
	int monitor_count; GLFWmonitor** monitors = glfwGetMonitors(&monitor_count);
	static int monitor_choice = 0;

	if (ImGui::Checkbox("Full screen", &fullscreen))
	{
		GLFWmonitor* monitor = monitor_choice < monitor_count ? glfwGetMonitors(&monitor_count)[monitor_choice] : glfwGetPrimaryMonitor();
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		static int window_width, window_height, window_xpos, window_ypos;

		if (fullscreen) {
			glfwGetWindowSize(window, &window_width, &window_height);
			glfwGetWindowPos(window, &window_xpos, &window_ypos);
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, GLFW_DONT_CARE);
		} else {
			glfwSetWindowMonitor(window, nullptr, window_xpos, window_ypos, window_width, window_height, GLFW_DONT_CARE);
		}
	}

	static bool monitor_choice_dirty = false, _b;
	if (ImGui::BeginCombo("Full screen monitor", std::to_string(monitor_choice).c_str())) {
		for (int i = 0; i < monitor_count; ++i) {
			if (ImGui::Selectable(std::to_string(i).c_str(), _b)) {
				monitor_choice = i;
				monitor_choice_dirty = true;
			}
		}
		ImGui::EndCombo();
	}

	//for (int i = 0; i < options->misc_options.size(); ++i) {
	//	std::string name = "Debug option" + std::to_string(i);
	//	ImGui::SliderFloat(name.c_str(), &(options->misc_options[i]), 0.0f, 1.f);
	//}

}