cmake_minimum_required( VERSION 2.8 )
project( directMethods )

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
# Eigen
find_package( Eigen3 REQUIRED )
# g2o
find_package( g2o )

include_directories( 
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
)
set( G2O_LIBS 
    g2o_core 
    g2o_types_sba 
    g2o_solver_csparse 
    g2o_stuff 
    g2o_csparse_extension 
)

# CPU - Sparse
add_executable( directMethodSparse main_sparse.cpp )
target_link_libraries( directMethodSparse 
    ${OpenCV_LIBS}
    ${G2O_LIBS}
    ${CSPARSE_LIBRARY}
)