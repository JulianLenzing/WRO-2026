cmake_minimum_required(VERSION 3.1.6)

add_library( encoder STATIC
   	encoder.cpp
)

target_include_directories( encoder PRIVATE
	../include
	../include/include
)

target_link_libraries( encoder
	i2c
)