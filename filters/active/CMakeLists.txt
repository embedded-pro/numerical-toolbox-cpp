add_library(dsp.filters.active ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(dsp.filters.active INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.filters.active INTERFACE
    infra.util
    dsp.math
)

target_sources(dsp.filters.active PRIVATE
    KalmanFilter.hpp
)

add_subdirectory(test)
