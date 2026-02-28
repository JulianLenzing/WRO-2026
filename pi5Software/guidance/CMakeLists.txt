cmake_minimum_required(VERSION 3.1.6)

add_library( guidance STATIC
    	guidance.cpp
)

target_include_directories( guidance PRIVATE
	../include
	../include/include
)

find_package(Threads REQUIRED)

target_link_libraries( guidance PUBLIC
    servoControl
    rt
    Threads::Threads
)