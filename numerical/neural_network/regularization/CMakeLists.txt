add_library(numerical.neural_network.regularization ${EMIL_EXCLUDE_FROM_ALL} INTERFACE)

target_include_directories(numerical.neural_network.regularization INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.regularization INTERFACE
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.regularization PRIVATE
    L1.hpp
    L2.hpp
    Regularization.hpp
)

add_subdirectory(test)
