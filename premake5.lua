workspace "ToyRasterizer"
	architecture "x64"
	configurations { "Debug", "Release" }

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"	--e.g., Debug-Windows-x64

project "ToyRasterizer"
	location "ToyRasterizer"
	kind "ConsoleApp"
	language "C++"

	objdir ("bin/tmp/" .. outputdir .. "/%{prj.name}")
	targetdir ("bin/" .. outputdir .. "/%{prj.name}")

	files {
		"%{prj.name}/src/**.h",
		"%{prj.name}/src/**.cpp",
	}

	includedirs {
		"%{prj.name}/src",
		"%{prj.name}/third-party",
	}

	buildoptions {
		"/utf-8",
	}

	postbuildcommands {
		("{MKDIR} ../bin/" .. outputdir .. "/%{prj.name}/src"),
		("{MKDIR} ../bin/" .. outputdir .. "/%{prj.name}/src/assets"),
		("{MKDIR} ../bin/" .. outputdir .. "/%{prj.name}/src/outputs"),
		("{COPYFILE} src/assets ../bin/" .. outputdir .. "/%{prj.name}/src/assets/"),
	}

	filter "system:linux"
		pic "On"
		cppdialect "C++20"
		staticruntime "On"
		systemversion "latest"

	filter "system:windows"
		cppdialect "C++20"
		staticruntime "On"
		systemversion "latest"

	filter "configurations:Debug"
		runtime "Debug"
		symbols "On"

	filter "configurations:Release"
		runtime "Release"
		optimize "On"
