cmake_minimum_required(VERSION 3.0)
project( calibrateCamera )

## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )
# generate compile commands
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( calibrateCamera calibrateCamera.cpp )
# link OpenCV
target_link_libraries( calibrateCamera ${OpenCV_LIBS} )