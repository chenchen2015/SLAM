cmake_minimum_required( VERSION 2.8 )
project( sophusBasics )

# Eigen
find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( sophusBasics useSophus.cpp )
target_link_libraries( sophusBasics ${Sophus_LIBRARIES} )