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
    ActiveSetQp.hpp
    ConditionNumber.hpp
    DampedPseudoInverse.hpp
    DiscreteAlgebraicRiccatiEquation.hpp
    DormandPrince45.hpp
    DurandKerner.hpp
    GaussianElimination.hpp
    JacobiEigenSolver.hpp
    LevinsonDurbin.hpp
    LuDecomposition.hpp
    LyapunovSylvester.hpp
    OdeSystem.hpp
    QrDecomposition.hpp
    RungeKuttaIntegrators.hpp
    SecondOrderSystem.hpp
    SingularValueDecomposition.hpp
    Solver.hpp
    SpectralRadius.hpp
    SymplecticIntegrators.hpp
)

numerical_add_coverage_sources(numerical.solver
    ActiveSetQp.cpp
    DampedPseudoInverse.cpp
    DiscreteAlgebraicRiccatiEquation.cpp
    DurandKerner.cpp
    GaussianElimination.cpp
    JacobiEigenSolver.cpp
    LevinsonDurbin.cpp
    LuDecomposition.cpp
    LyapunovSylvester.cpp
    QrDecomposition.cpp
    RungeKuttaIntegrators.cpp
    SingularValueDecomposition.cpp
    SpectralRadius.cpp
    SymplecticIntegrators.cpp
)

add_subdirectory(test)
