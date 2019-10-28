#include "common.h"
#include "helpers/Singleton.h"
#include "gui/BaseGUI.h"

#include <GLFW/glfw3.h>
#include <map>

class GUIManagerInstance
{

public:

	void      initialize(GLFWwindow* window);
	void      finalize(GLFWwindow* window);
	int       addGUI(std::string name, GUIptr gui);
	int       removeGUI(std::string name);
	void      draw();

protected:
	GUIManagerInstance()  = default;
	~GUIManagerInstance() = default;

private:
	GLFWwindow* window;
	std::map<std::string, GUIptr> guis;
	friend Singleton<GUIManagerInstance>;
};

typedef Singleton<GUIManagerInstance> GUIManager;
