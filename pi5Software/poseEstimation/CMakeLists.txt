cmake_minimum_required(VERSION 3.1.6)

set(CMAKE_CXX_STANDARD 20)

add_library(poseEstimation STATIC
	    poseEstimation.cpp
	    slam.cpp
	    DisplayData.cpp
)

target_include_directories( poseEstimation PUBLIC
	./include
	../include
	../lib/glfw
)

find_package(OpenGL REQUIRED)

target_link_libraries( poseEstimation PRIVATE
	${CMAKE_SOURCE_DIR}/../lib/glfw/libglfw3.a
	OpenGL::GL
	glad
)