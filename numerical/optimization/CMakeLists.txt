numerical_add_header_library(numerical.optimization)

target_include_directories(numerical.optimization ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.optimization ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.neural_network.losses
)

target_sources(numerical.optimization PRIVATE
    GradientDescent.hpp
    Optimizer.hpp
)

numerical_add_coverage_sources(numerical.optimization
    GradientDescent.cpp
)

add_subdirectory(test)
