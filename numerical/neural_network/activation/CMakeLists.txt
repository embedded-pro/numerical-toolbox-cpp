add_library(numerical.neural_network.activation ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.neural_network.activation INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.activation INTERFACE
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.activation PRIVATE
    ActivationFunction.hpp
    LeakyReLU.hpp
    ReLU.hpp
    Sigmoid.hpp
    Softmax.hpp
    Tanh.hpp
)

add_subdirectory(test)
