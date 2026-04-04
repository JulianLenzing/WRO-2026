cmake_minimum_required(VERSION 3.1.6)
project(main)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(../guidance ${CMAKE_BINARY_DIR}/guidance-build)

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
	../src/lidar.cpp
	../src/slam.cpp
	../src/gpioControl.cpp

	../src/EncoderController.cpp
	../src/PwmController.cpp
	../src/GpioController.cpp
)

target_include_directories( main PUBLIC
	../include
	../include/include
	../include/states
	../lib/imgui
	../lib/imgui/backends
	../lib/glfw
	../lib/rplidar_sdk/sdk/include
	${LIBGPIOD_INCLUDE_DIRS}
)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
find_package(PiPCA9685 REQUIRED)

target_link_libraries(main PRIVATE 
	${CMAKE_SOURCE_DIR}/../lib/glfw/libglfw3.a
	OpenGL::GL
	${CMAKE_SOURCE_DIR}/../lib/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
	PiPCA9685::PiPCA9685
	guidance
	${LIBGPIOD_LIBRARIES}
	i2c
)