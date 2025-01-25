add_library(dsp.filters.passive ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(dsp.filters.passive PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.filters.passive PUBLIC
    infra.util
    dsp.math
)

target_sources(dsp.filters.passive PRIVATE
    Fir.cpp
    Fir.hpp
    Iir.cpp
    Iir.hpp
)

add_subdirectory(test)
