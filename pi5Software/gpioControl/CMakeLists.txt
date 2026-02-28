cmake_minimum_required(VERSION 3.25)

add_library(gpioControl 
	gpioControl.cpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

target_include_directories( gpioControl PRIVATE
	../include
	../include/include
	${LIBGPIOD_INCLUDE_DIRS}
)

target_link_libraries(gpioControl PRIVATE 
	${LIBGPIOD_LIBRARIES}
)


