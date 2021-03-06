cmake_minimum_required( VERSION 2.8 )
project ( xSLAM )

set( CMAKE_CXX_COMPILER "clang++-7" )
## configure build
set( CMAKE_BUILD_TYPE Release )
add_compile_options(-std=c++11)
# Optimization flag, (low) O1 -> O3 (high)
set( CMAKE_CXX_FLAGS_RELEASE "-O3" )
# generate compile commands
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
find_package( Eigen3 REQUIRED )
# OpenCV
find_package( OpenCV REQUIRED )
set( OpenGL_GL_PREFERENCE GLVND )
# Sophus
find_package( Sophus REQUIRED )
# G2O
find_package( g2o REQUIRED )
set( G2O_LIBS 
    g2o_core 
    g2o_types_sba 
    g2o_solver_csparse 
    g2o_stuff 
    g2o_csparse_extension 
)

include_directories( 
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
)

set( THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${G2O_LIBS}
    ${CSPARSE_LIBRARY}
)

############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
