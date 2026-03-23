add_library(numerical.filters.passive ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.filters.passive INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.filters.passive INTERFACE
    infra.util
    numerical.math
)

target_sources(numerical.filters.passive PRIVATE
    Fir.hpp
    Iir.hpp
)

add_subdirectory(test)
