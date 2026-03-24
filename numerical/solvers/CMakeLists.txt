numerical_add_header_library(numerical.solver)

target_include_directories(numerical.solver ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.solver ${NUMERICAL_VISIBILITY}
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

numerical_add_coverage_sources(numerical.solver
    DiscreteAlgebraicRiccatiEquation.cpp
    DurandKerner.cpp
    GaussianElimination.cpp
)

add_subdirectory(test)
