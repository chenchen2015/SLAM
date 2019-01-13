cmake_minimum_required( VERSION 2.8 )
project( g2o_basics1 )

## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )
# generate compile commands for YCM
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# Eigen
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
# G2O
find_package( g2o REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( g2o_basics1 g2o_basics.cpp )
# link g2o and OpenCV
target_link_libraries( g2o_basics1
    ${Eigen3_LIBS}
    ${OpenCV_LIBS}
    g2o_core g2o_stuff
)
