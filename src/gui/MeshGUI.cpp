#include "common.h"
#include "gui/MeshGUI.h"
#include "managers/TextureManager.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtc/type_ptr.hpp>

MeshGUI::MeshGUI(MeshData* mesh)
{
	setMesh(mesh);
	visible     = true;
	displayname = "Mesh options";
}

MeshGUI::~MeshGUI()
{}

void MeshGUI::setMesh(MeshData* mesh)
{
	this->meshptr = mesh;
}

void createTextureFrame(std::string& texname, std::string& type)
{
	if (texname.empty() || !TextureManager::instance().hasTexture(texname))  return;

	const TextureData& texdata = TextureManager::instance().getTexture(texname);

	if (ImGui::TreeNode(type.c_str())) {
		ImGui::Text("Filename: %s", texname.c_str());

		float scrollbarWidth = ImGui::GetWindowWidth() - ImGui::GetWindowContentRegionWidth();
		float width = std::max(100.f, ImGui::GetContentRegionAvailWidth() + scrollbarWidth - 50);
		float height = width * (texdata.gltex->height / (float)texdata.gltex->width);
		ImGui::Image((void*)(intptr_t)texdata.gltex->m_uiOpenGLID, ImVec2(width, height), ImVec2(0, 1), ImVec2(1, 0), ImColor(255, 255, 255, 255), ImColor(255, 255, 255, 128));

		ImGui::TreePop();
	}
}

bool createSubmeshFrame(Submesh& sm)
{
	ImGui::Text("Start index: %s",         std::to_string(sm.start_index).c_str());
	ImGui::Text("End index: %s",           std::to_string(sm.end_index).c_str());
	ImGui::Text("Largest index value: %s", std::to_string(sm.largest_index_value).c_str());
	ImGui::Text("Lowest index value: %s",  std::to_string(sm.lowest_index_value).c_str());
	ImGui::Text("Default material: %s",    std::to_string(sm.default_material_index).c_str());

	return false;
}

bool createMaterialFrame(Material& m)
{
	bool dirty = false;

	if (ImGui::DragFloat3("Ambient",    glm::value_ptr(m.gl.ambient),  0.01f, 0.0f, 1e12f)) dirty = true;
	if (ImGui::DragFloat3("Diffuse",    glm::value_ptr(m.gl.diffuse),  0.01f, 0.0f, 1e12f)) dirty = true;
	if (ImGui::DragFloat3("Specular",   glm::value_ptr(m.gl.specular), 0.01f, 0.0f, 1e12f)) dirty = true;
	if (ImGui::DragFloat("Shininess",   &m.gl.shininess,               0.1f,  0.0f, 1e12f)) dirty = true;
	if (ImGui::DragFloat3("Emission",   glm::value_ptr(m.gl.emission), 0.01f, 0.0f, 1e12f)) dirty = true;

	if (ImGui::DragFloat ("Metallic",      &m.gl.metallic,                     0.001f, 0.f, 1.f)) dirty = true;
	if (ImGui::DragFloat ("Roughness",     &m.gl.roughness,                    0.001f, 0.f, 1.f)) dirty = true;
	if (ImGui::DragFloat ("Sheen",         &m.gl.sheen,                        0.001f, 0.f, 1.f)) dirty = true;
	if (ImGui::DragFloat ("Transmittance", glm::value_ptr(m.gl.transmittance), 0.01f)) dirty = true;

	if (ImGui::TreeNode("Textures"))
	{
		createTextureFrame(m.alpha_texname,              std::string("Alpha"));
		createTextureFrame(m.ambient_texname,            std::string("Ambient"));
		createTextureFrame(m.diffuse_texname,            std::string("Diffuse"));
		createTextureFrame(m.specular_highlight_texname, std::string("Specular"));
		createTextureFrame(m.emissive_texname,           std::string("Emissive"));
		createTextureFrame(m.bump_texname,               std::string("Bump"));
		createTextureFrame(m.normal_texname,             std::string("Normal"));
		createTextureFrame(m.displacement_texname,       std::string("Displacement"));
		createTextureFrame(m.metallic_texname,           std::string("Metallic"));
		createTextureFrame(m.reflection_texname,         std::string("Reflection"));
		createTextureFrame(m.roughness_texname,          std::string("Roughness"));
		createTextureFrame(m.sheen_texname,              std::string("Sheen"));
		ImGui::TreePop();
	}

	return dirty;
}

void MeshGUI::draw(GLFWwindow* window, int order)
{
	if (!meshptr) return;

	std::vector< Material >& materialList = meshptr->materialdata.materials;
	std::vector<Submesh>& submeshes = meshptr->submeshes;

	ImVec2 outerSize = ImGui::GetItemRectSize();

	if (ImGui::TreeNode("Submeshes"))
	{
		for (size_t i = 0; i < submeshes.size(); i++) {
			std::string nodeName = submeshes[i].name + "##" + std::to_string(i);
			if (ImGui::TreeNode(nodeName.c_str()))
			{
				createSubmeshFrame(submeshes[i]);
				ImGui::TreePop();
			}
		}

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Materials"))
	{

		for (size_t i = 0; i < materialList.size(); i++) {
			std::string nodeName = materialList[i].name + "##" + std::to_string(i);
			if (ImGui::TreeNode(nodeName.c_str()))
			{
				if (createMaterialFrame(materialList[i])) meshptr->materialdata.setDirty();
				ImGui::TreePop();
			}
		}
		ImGui::TreePop();
	}

}