-- Premake file
--
local path = require ("path")

-- Function to obtaim latest SDK version to use in win10 VS
function os.winSdkVersion()
    local reg_arch = iif( os.is64bit(), "\\Wow6432Node\\", "\\" )
    local sdk_version = os.getWindowsRegistry( "HKLM:SOFTWARE" .. reg_arch .."Microsoft\\Microsoft SDKs\\Windows\\v10.0\\ProductVersion" )
    if sdk_version ~= nil then return sdk_version end
end

-- Function to add " at beginning and end of string
function quot(s)
	return '"'..s..'"'
end

workspace "Demo"
    location "build/"
    platforms { "x64" }
    configurations { "release" }
	startproject "demo"
    
	language "C++"

    cppdialect "C++14"
    flags "NoPCH"
    vectorextensions "AVX"
    flags "MultiProcessorCompile"

    objdir "build/%{cfg.buildcfg}-%{cfg.platform}-%{cfg.toolset}"
    targetsuffix "-%{cfg.buildcfg}-%{cfg.platform}-%{cfg.toolset}"

    newoption {
        trigger = "toolset",
        description = "Select toolset on Linux / MacOS",
        allowed = {
            { "gcc", "Use GCC" },
            { "clang", "Use Clang" }
        }
    };

    -- Workaround empty "toolset"
    filter "system:linux or system:macos"
        toolset( _OPTIONS["toolset"] or "gcc" );
    filter "system:windows"
        toolset( "msc" );
    filter "*"

    -- default compiler flags
    filter "toolset:gcc or toolset:clang"
        linkoptions { "-pthread" }
        buildoptions { "-march=native", "-Wall", "-pthread" }

    filter "toolset:msc"
        defines { "_CRT_SECURE_NO_WARNINGS=1", "NOMINMAX" }

    filter "action:vs2015"
        buildoptions { "/utf-8", "/openmp" }
    
    filter "action:vs2017"
        buildoptions { "/utf-8", "/openmp"  }
        --buildoptions { "/std:c++latest" }
    
    filter "action:vs2019"
        buildoptions { "/utf-8", "/openmp"  }

    filter "*"

    -- default libraries
    filter "system:linux"
        links "GL"
        links "GLU"

    filter "system:windows"
        links "OpenGL32"

        libdirs "%{wks.location}/freeglut/lib/x64"
        includedirs "%{wks.location}/freeglut/include"

    filter "*"

    -- default outputs
    filter "kind:StaticLib"
        targetdir "lib/"

    filter "kind:ConsoleApp"
        targetdir "bin"
        targetextension ".exe"
    
    filter "*"

    --configurations

    configuration "release"
        optimize "On"
        defines { "NDEBUG=1" }
        flags "LinkTimeOptimization"

    configuration "*"
	
	-- Required to make eigen work with OpenMP
	defines { "EIGEN_DONT_PARALLELIZE" }

    
filter {"system:windows", "kind:ConsoleApp"}

    postbuildcommands {
        "{COPY} \"%{wks.location}/../external/win/glew-2.1.0/bin/Release/x64/glew32.dll\"   \"%{cfg.targetdir}\"",
        "{COPY} \"%{wks.location}/../external/win/glfw-3.2.1/lib-vc2015/glfw3.dll\"         \"%{cfg.targetdir}\"",
        "{COPY} \"%{wks.location}/../external/win/FreeImage-3.18.0/Dist/x64/FreeImage.dll\" \"%{cfg.targetdir}\""
    }
    
    filter "*"  

project( "imgui" )
    kind "StaticLib"
    location "build/imgui"

    filter {"system:windows", "action:vs*"}
        systemversion(os.winSdkVersion() .. ".0")
	filter "*"

    local source_files  = { "external/imgui-1.65/*.cpp" }
    local include_files = { "external/imgui-1.65/*.h" }
    files { source_files, include_files}

    local glfw_includedirs
    filter "system:linux"
        glfw_includedirs = { }
    filter "system:windows"
        glfw_includedirs = { "external/win/glfw-3.2.1/include" }
	filter "*"

    
    local glew_includedirs
    filter "system:linux"
        glew_includedirs = {  }
    filter "system:windows"
        glew_includedirs = { "external/win/glew-2.1.0/include" }
	filter "*"

	
	includedirs { glfw_includedirs, glew_includedirs }

project( "meshoptimizer" )
    kind "StaticLib"
    location "build/meshoptimizer"

    filter {"system:windows", "action:vs*"}
        systemversion(os.winSdkVersion() .. ".0")
	filter "*"

    local source_files  = { "external/meshoptimizer/src/*.cpp" }
    local include_files = { "external/meshoptimizer/src/*.h" }
    files { source_files, include_files}

    
project( "demo" )
    kind "ConsoleApp"
    location "build/demo"

    filter "system:windows"
        debugdir "%{wks.location}/.." 
    
    filter {"system:windows", "action:vs*"}
        systemversion(os.winSdkVersion() .. ".0")

    local source_files  = { "src/*.cpp"    , "src/helpers/*.cpp"     , "src/managers/*.cpp",     "src/gui/*.cpp",	"src/dynamics/*.cpp"}
    local include_files = { "include/*.h"  , "include/helpers/*.h"   , "include/managers/*.h",   "include/gui/*.h", 	"include/dynamics/*.h",
                            "include/*.inl", "include/helpers/*.inl" , "include/managers/*.inl", "include/gui/*.inl"  }

    local imm_source_files  = { "src/immersed/*.cpp" }
    local imm_include_files = { "include/immersed/*.h" }

	local cuda_source_files  = { "src/cuda/*.cu"}
	local cuda_include_files = { "include/cuda/*.h", "include/cuda/*.cuh", "include/cuda/*.inl" }

    files { source_files, include_files, cuda_source_files, cuda_include_files, imm_source_files, imm_include_files }

    local include_dirs = { "include" }

	local external_include_dirs = { "external/glm-0.9.9.2/include",
									"external/imgui-1.65",
									"external/tinyobjloader",
									"external/meshoptimizer/src",
									"external/json",
									"external/json",
									"external/eigen-3.2.10"}
										
	local shader_dirs = { "shaders" }
	local shader_files = { "shaders/*.frag", "shaders/*.vert", "shaders/*.geom", "shaders/*.comp", "shaders/*.glsl", "shaders/Reusable/*.glsl" }
	local imm_shader_files = { "shaders/immersed/*.frag", "shaders/immersed/*.vert", "shaders/immersed/*.geom", "shaders/immersed/*.comp", "shaders/immersed/*.glsl" }

	files  { shader_files, imm_shader_files } 


	---------------------------------- GLFW
    filter "system:linux"
        glfw_libdirs = { }
        glfw_includedirs = { }
        glfw_libs = { os.findlib("glfw3") }
    filter "system:windows"
        glfw_libdirs =     { "external/win/glfw-3.2.1/lib-vc2015" }
        glfw_includedirs = { "external/win/glfw-3.2.1/include" }
        glfw_libs = { "glfw3dll" }
	filter "*"

    
	---------------------------------- GLEW
    filter "system:linux"
        glew_libdirs = { }
        glew_includedirs = {  }
        glew_libs = { os.findlib("glew") }
    filter "system:windows"
        glew_libdirs =     { "external/win/glew-2.1.0/lib/Release/x64" }
        glew_includedirs = { "external/win/glew-2.1.0/include" }
        glew_libs = { "glew32" }
	filter "*"

    
	---------------------------------- FREEIMAGE
    filter "system:linux"
        freeimage_libdirs = { }
		freeimage_includedirs = {}
        freeimage_libs = { os.findlib("freeimage") }
    filter "system:windows"
        freeimage_libdirs =     { "external/win/FreeImage-3.18.0/Dist/x64" }
		freeimage_includedirs = { "external/win/FreeImage-3.18.0/Dist/x64" }
        freeimage_libs = { "FreeImage" }
	filter "*"

	---------------------------------- CUDA
	cuda_path = os.getenv("CUDA_PATH")
    filter "system:linux"
        cuda_libdirs = { }
		cuda_includedirs = {}
        cuda_libs = { os.findlib("cuda"), os.findlib("cublas"), os.findlib("cusparse") }
    filter "system:windows"
        cuda_libdirs =     { cuda_path .. "/lib/x64" }
		cuda_includedirs = { cuda_path .. "/include" }
        cuda_libs = { "cuda", "cudart", "cublas", "cusparse" }
	filter "*"

		
    includedirs { include_dirs, external_include_dirs, glfw_includedirs, glew_includedirs, freeimage_includedirs, cuda_includedirs }
	libdirs { glfw_libdirs, glew_libdirs, freeimage_libdirs, cuda_libdirs }
    links { glfw_libs, glew_libs, freeimage_libs, "imgui", "meshoptimizer", cuda_libs }

	CUDA_CC = "nvcc"
	CUDA_FLAGS = "--use_fast_math"
	CUDA_SOURCE_DIR  = os.getcwd().."/src/cuda"
	
	CUDA_WIN_FLAGS_RELEASE  = '-Xcompiler "/MD"'
	CUDA_WIN_FLAGS_DEBUG = '-Xcompiler "/MDd"'

	------------- Custom build commands for cuda files (debug)
	filter { 'files:src/cuda/*.cu' }
	   -- A message to display while this build step is running (optional)
	   buildmessage 'Compiling %{file.relpath}'

	   -- One or more commands to run (required)
	   filter {'configurations:release', 'files:src/cuda/*.cu'}
		   buildcommands { '%{CUDA_CC} %{CUDA_FLAGS} %{CUDA_WIN_FLAGS_RELEASE} -c -I"%{cuda_path}/include" -I"%{wks.location}/../include" "%{file.path}"  -o "%{cfg.objdir}/%{file.basename}.cu.obj"' }
	   filter {'configurations:debug', 'files:src/cuda/*.cu'}
		   buildcommands { '%{CUDA_CC} %{CUDA_FLAGS} %{CUDA_WIN_FLAGS_DEBUG} -c -I"%{cuda_path}/include" -I"%{wks.location}/../include" "%{file.path}"  -o "%{cfg.objdir}/%{file.basename}.cu.obj"' }
	   filter {'files:src/cuda/*.cu'}

	   -- One or more outputs resulting from the build (required)
	   buildoutputs { '%{cfg.objdir}/%{file.basename}.cu.obj' }

	   -- One or more additional dependencies for this build command (optional)
	   buildinputs { '%{wks.location}/../include/cuda/*.cuh' }
    filter "*"

	files {cuda_source_files, cuda_include_files }
    
	
--EOF
