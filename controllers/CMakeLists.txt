add_library(dsp.controllers ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(dsp.controllers INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.controllers INTERFACE
    infra.util
    dsp.math
)

target_sources(dsp.controllers PRIVATE
    FieldOrientedController.hpp
    Pid.hpp
    SpaceVectorModulation.hpp
    TransformsClarkePark.hpp
)

add_subdirectory(test)
