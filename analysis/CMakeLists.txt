add_library(dsp.analysis ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(dsp.analysis INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.analysis INTERFACE
    infra.util
    dsp.math
)

target_sources(dsp.analysis PRIVATE
    FrequencyResponse.hpp
)

add_subdirectory(test)
