#pragma once

#include "common.h"
#include "gui/BaseGUI.h"
#include "helpers/CameraFPS.h"

class CameraGUI : public BaseGUI
{
public:
	CameraGUI(CameraFPS* cam = nullptr);
	void setCamera(CameraFPS* cam);
	virtual ~CameraGUI();
	virtual void draw(GLFWwindow* window, int order = 0);
private:
	CameraFPS* camptr;
};

