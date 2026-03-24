numerical_add_header_library(numerical.neural_network.layer)

target_include_directories(numerical.neural_network.layer ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.layer ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.layer PRIVATE
    Dense.hpp
    Layer.hpp
)

numerical_add_coverage_sources(numerical.neural_network.layer
    Dense.cpp
)

add_subdirectory(test)
