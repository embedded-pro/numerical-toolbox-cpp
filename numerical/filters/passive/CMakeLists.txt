numerical_add_header_library(numerical.filters.passive)

target_include_directories(numerical.filters.passive ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.filters.passive ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.filters.passive PRIVATE
    Fir.hpp
    Iir.hpp
)

numerical_add_coverage_sources(numerical.filters.passive
    Fir.cpp
    Iir.cpp
)

add_subdirectory(test)
