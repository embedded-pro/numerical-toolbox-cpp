add_library(dsp.windowing ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(dsp.windowing INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(dsp.windowing INTERFACE
    dsp.math
)

target_sources(dsp.windowing PRIVATE
    Windowing.hpp
)

add_subdirectory(test)
