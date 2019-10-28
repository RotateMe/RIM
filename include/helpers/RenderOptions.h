#pragma once

#include "common.h"
#include "helpers/Singleton.h"

#include <array>

// Quick and dirty structure to pass around data between parts of the program
//    by using a singleton object (DefaultRenderOptions)

struct RenderOptions
{
	bool alphatest = false;
	bool vsync     = false;
	bool dirtyShaders;

	std::array<float, 5> misc_options;

private:
	friend Singleton<RenderOptions>;
};

struct RenderStats
{
	float time_cpu_ms;
	float time_gl_ms;
	float time_cuda_ms;
	float fps;
};

RenderOptions& DefaultRenderOptions();
