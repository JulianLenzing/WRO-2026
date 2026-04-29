cmake_minimum_required(VERSION 3.1.6)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)
project(main)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT SIMULATION)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
find_package(PiPCA9685 REQUIRED)
endif()

if(SIMULATION)
find_package(glfw3 REQUIRED)
endif()

# Common components

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
	../src/Pathfinder.cpp
)
	
target_include_directories( main PUBLIC
	${OpenCV_INCLUDE_DIRS}
	../include
	../include/include
	../include/states
	../lib/imgui
	../lib/imgui/backends
	../lib/glfw
	../lib/rplidar_sdk/sdk/include
)

target_link_libraries(main PRIVATE 
	OpenGL::GL
	${OpenCV_LIBS}
)

option(OPENING_RUN "Prepare the software for variable dimensions of the inner Walls and no obstacles" OFF)
if(OPENING_RUN)
	target_compile_definitions(main PRIVATE OPENING_RUN)
endif()

option(SIMULATION "Enable support for the godot simulation" OFF)
if(SIMULATION)
	target_compile_definitions(main PRIVATE SIMULATION)
endif()

# Normal components

if(NOT SIMULATION)
    target_sources(main PRIVATE 
		# Files that are replaced in a simulation build
		../src/lidar.cpp
		../src/EncoderController.cpp
		../src/PwmController.cpp
		../src/GpioController.cpp
		../src/Gyro.cpp
		../src/Camera.cpp

		# Files not needed in a simulation build
		../src/gpioControl.cpp
	)

	target_include_directories( main PRIVATE
		${LIBGPIOD_INCLUDE_DIRS}
	)

	target_link_libraries(main PRIVATE
		${CMAKE_SOURCE_DIR}/../lib/glfw/libglfw3.a
		${CMAKE_SOURCE_DIR}/../lib/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
		${LIBGPIOD_LIBRARIES}
		PiPCA9685::PiPCA9685
		Threads::Threads
		i2c
		rt
	)
endif()

# Simulation components

if(SIMULATION)
	target_sources(main PRIVATE 
		../simulation/lidar.cpp
		../simulation/EncoderController.cpp
		../simulation/PwmController.cpp
		../simulation/GpioController.cpp
		../simulation/Gyro.cpp
		../simulation/Camera.cpp
	)

	target_link_libraries(main PRIVATE 
		glfw
	)
endif()
