add_library(numerical.neural_network.activation ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.neural_network.activation PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.activation PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.activation PRIVATE
    ActivationFunction.cpp
    ActivationFunction.hpp
    LeakyReLU.hpp
    ReLU.hpp
    Sigmoid.hpp
    Softmax.hpp
    Tanh.hpp
)

add_subdirectory(test)
