#pragma once

#include "common.h"
#include "helpers/RenderOptions.h"


RenderOptions& DefaultRenderOptions() { return Singleton<RenderOptions>::instance(); }
