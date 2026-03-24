numerical_add_header_library(numerical.windowing)

target_include_directories(numerical.windowing ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.windowing ${NUMERICAL_VISIBILITY}
    numerical.math
)

target_sources(numerical.windowing PRIVATE
    Windowing.hpp
)

numerical_add_coverage_sources(numerical.windowing
    Windowing.cpp
)

add_subdirectory(test)
