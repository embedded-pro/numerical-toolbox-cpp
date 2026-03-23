add_library(numerical.solver ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(numerical.solver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.solver PUBLIC
    infra.util
    numerical.math
)

target_sources(numerical.solver PRIVATE
    CholeskyDecomposition.hpp
    DiscreteAlgebraicRiccatiEquation.hpp
    DurandKerner.hpp
    GaussianElimination.hpp
    LevinsonDurbin.hpp
    Solver.hpp
)

if (EMIL_ENABLE_COVERAGE)
    target_sources(numerical.solver PRIVATE
        DiscreteAlgebraicRiccatiEquation.cpp
        DurandKerner.cpp
        GaussianElimination.cpp
    )
endif()

add_subdirectory(test)
