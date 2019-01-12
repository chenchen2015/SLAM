cmake_minimum_required( VERSION 2.8 )
project( CeresBasics1 )

## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )
# generate compile commands for YCM
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# Ceres
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

## sources
add_executable( CeresBasics1 ceres_basics.cpp )
# Link Ceres and OpenCV
target_link_libraries( CeresBasics1 ${CERES_LIBRARIES} ${OpenCV_LIBS} )