numerical_add_header_library(numerical.neural_network.regularization)

target_include_directories(numerical.neural_network.regularization ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.regularization ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.regularization PRIVATE
    L1.hpp
    L2.hpp
    Regularization.hpp
)

numerical_add_coverage_sources(numerical.neural_network.regularization
    L1.cpp
    L2.cpp
)

add_subdirectory(test)
