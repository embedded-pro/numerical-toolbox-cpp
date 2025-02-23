add_library(numerical.neural_network.layer ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.neural_network.layer PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.neural_network.layer PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.neural_network.layer PRIVATE
    Dense.cpp
    Dense.hpp
    Layer.hpp
)

add_subdirectory(test)
