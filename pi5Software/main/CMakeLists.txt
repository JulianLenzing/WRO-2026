cmake_minimum_required(VERSION 3.1.6)
project(main)

option(SIMULATION "Enable support for the godot simulation" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
find_package(PiPCA9685 REQUIRED)

set(IMGUI_SRC
    ../lib/imgui/imgui.cpp
    ../lib/imgui/imgui_draw.cpp
    ../lib/imgui/imgui_widgets.cpp
    ../lib/imgui/imgui_tables.cpp
    ../lib/imgui/imgui_demo.cpp
    ../lib/imgui/backends/imgui_impl_glfw.cpp
    ../lib/imgui/backends/imgui_impl_opengl3.cpp
)

add_executable(main
	main.cpp
	${IMGUI_SRC}
	../src/glad.c
	../src/guidance.cpp
	../src/slam.cpp
)
	
target_include_directories( main PUBLIC
	../include
	../include/include
	../include/states
	../lib/imgui
	../lib/imgui/backends
	../lib/glfw
	../lib/rplidar_sdk/sdk/include
)

target_link_libraries(main PRIVATE 
	${CMAKE_SOURCE_DIR}/../lib/glfw/libglfw3.a
	OpenGL::GL
)

if(NOT SIMULATION)
    target_sources(main PRIVATE 
		# Files that are replaced in a simulation build
		../src/lidar.cpp
		../src/EncoderController.cpp
		../src/PwmController.cpp
		../src/GpioController.cpp

		# Files not needed in a simulation build
		../src/gpioControl.cpp
	)

	target_include_directories( main PRIVATE
		${LIBGPIOD_INCLUDE_DIRS}
	)

	target_link_libraries(main PRIVATE
		${CMAKE_SOURCE_DIR}/../lib/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
		${LIBGPIOD_LIBRARIES}
		PiPCA9685::PiPCA9685
		Threads::Threads
		i2c
		rt
	)
endif()

if(SIMULATION)
	target_compile_definitions(main PRIVATE NO_PCA9685)

	target_sources(main PRIVATE 
		../simulation/lidar.cpp
		../simulation/EncoderController.cpp
		../simulation/PwmController.cpp
		../simulation/GpioController.cpp
	)
endif()
