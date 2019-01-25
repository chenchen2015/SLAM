# add shared src files
add_library( xSLAM SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
)

# ${THIRD_PARTY_LIBS} defined in 
target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
