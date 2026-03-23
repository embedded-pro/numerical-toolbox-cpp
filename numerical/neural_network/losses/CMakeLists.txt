add_library(numerical.neural_network.losses ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.neural_network.losses INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.losses INTERFACE
    infra.util
    numerical.math
    numerical.neural_network.activation
    numerical.neural_network.regularization
)

target_sources(numerical.neural_network.losses PRIVATE
    BinaryCrossEntropy.hpp
    CategoricalCrossEntropy.hpp
    Loss.hpp
    MeanAbsoluteError.hpp
    MeanSquaredError.hpp
)

add_subdirectory(test)
