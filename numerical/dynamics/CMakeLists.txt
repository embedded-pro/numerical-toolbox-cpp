numerical_add_header_library(numerical.dynamics)

target_include_directories(numerical.dynamics ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.dynamics ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
    numerical.solver
)

target_sources(numerical.dynamics PRIVATE
    ArticulatedBodyAlgorithm.hpp
    EulerLagrangeDynamics.hpp
    EulerLagrangeSolver.hpp
    NewtonEulerBody.hpp
    NewtonEulerSolver.hpp
    RecursiveNewtonEuler.hpp
    RevoluteJointLink.hpp
)

numerical_add_coverage_sources(numerical.dynamics
    ArticulatedBodyAlgorithm.cpp
    EulerLagrangeSolver.cpp
    NewtonEulerSolver.cpp
    RecursiveNewtonEuler.cpp
)

add_subdirectory(test)
