add_library(dsp.controllers ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.controllers PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.controllers PUBLIC
    infra.util
    dsp.math
)

target_sources(dsp.controllers PRIVATE
    FieldOrientedController.cpp
    FieldOrientedController.hpp
    Pid.cpp
    Pid.hpp
    SpaceVectorModulation.cpp
    SpaceVectorModulation.hpp
    TransformsClarkePark.cpp
    TransformsClarkePark.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
