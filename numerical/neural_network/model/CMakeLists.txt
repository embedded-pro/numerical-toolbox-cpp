numerical_add_header_library(numerical.neural_network.model)

target_include_directories(numerical.neural_network.model ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.model ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.neural_network.activation
    numerical.neural_network.layer
    numerical.neural_network.losses
    numerical.neural_network.optimizer
)

target_sources(numerical.neural_network.model PRIVATE
    Model.hpp
)

add_subdirectory(test)
