cmake_minimum_required(VERSION 3.25)

add_library(servoControl 
	servoControl.cpp
)

target_include_directories( servoControl PRIVATE
	../include
	../include/include
)

find_package(PiPCA9685 REQUIRED)

target_link_libraries(servoControl PiPCA9685::PiPCA9685)
