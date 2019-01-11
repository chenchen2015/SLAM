cmake_minimum_required( VERSION 2.8 )
project( pclBasics1 )

## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )

## Libraries
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Eigen
find_package( Eigen3 REQUIRED )
include_directories( ${Eigen3_INCLUDE_DIRS} )
# PCL
find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

## sources
add_executable( pclBasics1 joinMap.cpp )
target_link_libraries( pclBasics1 ${OpenCV_LIBS} ${PCL_LIBRARIES} )