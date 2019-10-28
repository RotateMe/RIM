#pragma once

#include "common.h"
#include <gui/BaseGUI.h>
#include <helpers/MeshData.h>

class SimSettings
{
public:
	float reductionParam = 0.5;
	float fluidQuality = 0.2;
	bool addPool = true;
	float poolGap = 0;
	float poolHeight = 1.;
	float streamSize = 0;
	float gridSize = 0.35;
	bool preventSolidPen = false;
	bool doSetup = false;
};

class SimSetupGUI : public BaseGUI
{
public:
	SimSetupGUI(SimSettings* simSettings);
	virtual ~SimSetupGUI();
	virtual void draw(GLFWwindow* window, int order = 0);
private:
	SimSettings* simSettings;
};

