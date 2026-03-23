add_library(numerical.neural_network.optimizer ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.neural_network.optimizer INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.optimizer INTERFACE
    infra.util
    numerical.math
    numerical.neural_network.losses
)

target_sources(numerical.neural_network.optimizer PRIVATE
    GradientDescent.hpp
    Optimizer.hpp
)

add_subdirectory(test)
