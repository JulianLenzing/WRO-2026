cmake_minimum_required(VERSION 3.1.6)
project(main)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(../glad ${CMAKE_BINARY_DIR}/glad-build)
add_subdirectory(../lidar ${CMAKE_BINARY_DIR}/lidar-build)
add_subdirectory(../poseEstimation ${CMAKE_BINARY_DIR}/poseEstimation-build)
add_subdirectory(../servoControl ${CMAKE_BINARY_DIR}/servoControl-build)
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
)

target_include_directories( main PUBLIC
	../include
	../lib/imgui
	../lib/imgui/backends
	../lib/glfw
)

find_package(OpenGL REQUIRED)

target_link_libraries(main PRIVATE 
	glad
	lidar
	poseEstimation
	guidance
)