cmake_minimum_required(VERSION 3.1.6)

add_library( glad STATIC
    	glad.c
)

target_include_directories( glad PRIVATE
	../include
	../include/include
)
