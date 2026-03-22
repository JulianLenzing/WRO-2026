cmake_minimum_required(VERSION 3.1.6)

add_library( guidance STATIC
    	guidance.cpp
)

target_include_directories( guidance PRIVATE
	../include
	../include/include
)

find_package(Threads REQUIRED)
find_package(PiPCA9685 REQUIRED)

target_link_libraries( guidance PUBLIC
    rt
    Threads::Threads
	PiPCA9685::PiPCA9685
	i2c
)