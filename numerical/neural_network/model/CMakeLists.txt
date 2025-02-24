add_library(numerical.neural_network.model ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.neural_network.model PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.model PUBLIC
    infra.util
    numerical.math
    numerical.neural_network.activation
    numerical.neural_network.layer
    numerical.neural_network.losses
    numerical.neural_network.optimizer
)

target_sources(numerical.neural_network.model PRIVATE
    Model.cpp
    Model.hpp
)

add_subdirectory(test)
