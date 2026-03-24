numerical_add_header_library(numerical.neural_network.losses)

target_include_directories(numerical.neural_network.losses ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.losses ${NUMERICAL_VISIBILITY}
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

numerical_add_coverage_sources(numerical.neural_network.losses
    MeanSquaredError.cpp
    MeanAbsoluteError.cpp
    BinaryCrossEntropy.cpp
    CategoricalCrossEntropy.cpp
)

add_subdirectory(test)
