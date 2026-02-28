cmake_minimum_required(VERSION 3.1.6)

add_library( lidar STATIC
    	lidar.cpp
)

target_include_directories( lidar PRIVATE
	../include
)

target_include_directories( lidar PUBLIC
	../lib/rplidar_sdk/sdk/include
)

target_link_libraries( lidar PUBLIC
	${CMAKE_SOURCE_DIR}/../lib/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a
)