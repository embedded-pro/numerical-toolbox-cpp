numerical_add_header_library(numerical.neural_network.activation)

target_include_directories(numerical.neural_network.activation ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.activation ${NUMERICAL_VISIBILITY}
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

numerical_add_coverage_sources(numerical.neural_network.activation
    ReLU.cpp
    LeakyReLU.cpp
    Sigmoid.cpp
    Tanh.cpp
    Softmax.cpp
)

add_subdirectory(test)
