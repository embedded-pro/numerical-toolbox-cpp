numerical_add_header_library(numerical.control_analysis)

target_include_directories(numerical.control_analysis ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.control_analysis ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.control_analysis PRIVATE
    FrequencyResponse.hpp
    RootLocus.hpp
)

numerical_add_coverage_sources(numerical.control_analysis
    FrequencyResponse.cpp
    RootLocus.cpp
)

add_subdirectory(test)
