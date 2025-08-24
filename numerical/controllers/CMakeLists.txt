add_library(numerical.controllers ${EMIL_EXCLUDE_FROM_ALL} STATIC)

target_include_directories(numerical.controllers PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.controllers PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.controllers PRIVATE
    FieldOrientedController.cpp
    FieldOrientedController.hpp
    Pid.cpp
    Pid.hpp
    SpaceVectorModulation.cpp
    SpaceVectorModulation.hpp
    TrajectoryGenerator.hpp
    TransformsClarkePark.cpp
    TransformsClarkePark.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
