cmake_minimum_required(VERSION 3.1.6)
project(main)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(../glad ${CMAKE_BINARY_DIR}/glad-build)
add_subdirectory(../lidar ${CMAKE_BINARY_DIR}/lidar-build)
add_subdirectory(../slam ${CMAKE_BINARY_DIR}/slam-build)
add_subdirectory(../guidance ${CMAKE_BINARY_DIR}/guidance-build)
add_subdirectory(../gpioControl ${CMAKE_BINARY_DIR}/gpioControl-build)

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
)

target_include_directories( main PUBLIC
	../include
	../include/states
	../lib/imgui
	../lib/imgui/backends
	../lib/glfw
	${LIBGPIOD_INCLUDE_DIRS}
)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
find_package(PiPCA9685 REQUIRED)

target_link_libraries(main PRIVATE 
	glad
	lidar
	slam
	PiPCA9685::PiPCA9685
	guidance
	${LIBGPIOD_LIBRARIES}
	gpioControl
	i2c
)