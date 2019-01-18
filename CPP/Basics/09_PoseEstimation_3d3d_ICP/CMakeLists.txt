cmake_minimum_required( VERSION 2.8 )
project( poseEstimationICP )

## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )
# generate compile commands
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# Eigen
find_package( Eigen3 REQUIRED )
# G2O
find_package( g2o REQUIRED )
find_package( CSparse REQUIRED )
# OpenCV
find_package( OpenCV REQUIRED )

# include libraries
include_directories( 
    ${OpenCV_DIRS} 
    ${EIGEN3_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
)

add_executable( poseEstimationICP main.cpp )
# link g2o and OpenCV
target_link_libraries( poseEstimationICP 
    ${Eigen3_LIBS}
    ${OpenCV_LIBS}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension 
    ${CSPARSE_LIBRARY}
)
