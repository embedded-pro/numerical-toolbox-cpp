add_library(numerical.neural_network.optimizer ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.neural_network.optimizer PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.optimizer PUBLIC
    infra.util
    numerical.math
    numerical.neural_network.losses
)

target_sources(numerical.neural_network.optimizer PRIVATE
    GradientDescent.hpp
    Optimizer.cpp
    Optimizer.hpp
)

add_subdirectory(test)
