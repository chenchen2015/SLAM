# ADD SHARED SRC FILES
add_library( xSLAM SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    vo.cpp
    g2o_types.cpp
)

# ${THIRD_PARTY_LIBS} defined in
target_link_libraries( xSLAM
    ${THIRD_PARTY_LIBS}
)
