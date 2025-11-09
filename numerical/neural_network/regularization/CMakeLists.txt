add_library(numerical.neural_network.regularization ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.neural_network.regularization PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.regularization PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.regularization PRIVATE
    L1.hpp
    L2.hpp
    Regularization.cpp
    Regularization.hpp
)

add_subdirectory(test)
